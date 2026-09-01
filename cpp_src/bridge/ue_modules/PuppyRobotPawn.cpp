// PuppyRobotPawn.cpp -- UE5 robot Pawn (hosts LiDAR + TCP server)

// ==================================================================

// Aggregates UPuppyLiDARComponent and UPuppyTcpServer.

//

// Lockstep workflow (30Hz):

//   1. Tick sends LIDAR + GROUND_TRUTH + PED_STATE to nav_ue_bridge

//   2. Receives CMD_VEL, updates velocity and moves the robot

//   3. Collision detection (sweep) increments CollisionCount

//   4. STEP_ACK gates physics advancement (lockstep sync)

//

// Unit conventions: UE world units are cm; velocity is cm/s, angular

// velocity is deg/s.  nav_ue_bridge sends vx/vy in m/s and wz in rad/s,

// so we convert m/s -> cm/s and rad/s -> deg/s on receive.

#include "PuppyRobotPawn.h"

#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "HAL/PlatformMisc.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"

// --------------------------------------------------------------------------
// P0-02 (2026-08-15) -- SINGLE SOURCE OF TRUTH (declarations; the loader
// function LoadSceneHome() is defined further below).  Scene geometry
// (obstacles), pedestrians and the robot initial pose are loaded at runtime
// from config/scene_home.json so there is exactly ONE description of the room.
// Declared up here because BeginPlay / SpawnPedestriansFromScene reference them
// before the loader function's own definition.
// --------------------------------------------------------------------------
struct FHomeObstacle { const TCHAR* Name; float Xmin, Ymin, Xmax, Ymax; };
struct FPedSpawn     { const TCHAR* Name; float Xm, Ym, Vxm, Vym; };

static TArray<FHomeObstacle> GHomeObstacles;
static TArray<FPedSpawn>     GHomePedestrians;
static FVector2D             GInitPoseM(0.5f, -2.2f);  // metres, scene_home.robot.initial_pose

// Defined later (after the obstacle-geometry comment block).
static int32 LoadSceneHome();

#include "PuppyLiDARComponent.h"

#include "PuppyTcpServer.h"

#include "PuppyPedestrianActor.h"



#include "Components/CapsuleComponent.h"

#include "Components/StaticMeshComponent.h"

#include "GameFramework/SpringArmComponent.h"

#include "Camera/CameraComponent.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "ImageUtils.h"

#include "Engine/DirectionalLight.h"
#include "Components/LightComponent.h"

#include "Engine/StaticMesh.h"

#include "Engine/StaticMeshActor.h"   // R32: whitelist purge of level-authored colliders

#include "Components/SphereComponent.h"

#include "Materials/MaterialInterface.h"

#include "Materials/MaterialInstanceDynamic.h"

#include "Kismet/GameplayStatics.h"



// m/s -> cm/s

static constexpr float M_TO_CM = 100.0f;



APuppyRobotPawn::APuppyRobotPawn()

{

    // ---- SPAWN SAFETY NET ---------------------------------------------------

    // Without this, the floor at Z 鈭?[-5cm, +5cm] and PlayerStart at Z=0

    // cause SpawnActor to *silently fail* because the 30cm-half-height capsule

    // would be "embedded in geometry":

    //   [LogSpawn] Warning: SpawnActor failed because of collision at the

    //                      spawn location [X=0 Y=0 Z=0] for [BP_PuppyRobotPawn_C]

    // With AdjustIfPossibleButAlwaysSpawn, UE nudges the spawn location up a

    // few cm (or spawns anyway) instead of dropping the whole Pawn. This

    // matches AutoSetup step 9.6 which raises PlayerStart to Z=+15cm.

    SpawnCollisionHandlingMethod = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;



    PrimaryActorTick.bCanEverTick = true;

    PrimaryActorTick.bStartWithTickEnabled = true;

    AutoPossessPlayer = EAutoReceiveInput::Player0;



    // Root collision capsule so that sweep-based movement detects walls.

    // --- Collision root capsule ---

    // Boston Dynamics Spot robot is roughly 60cm tall 脳 30cm wide.

    // Capsule radius = 30cm, half-height = 30cm 鈫?total height 60cm,

    // width/diameter = 60cm.  This is the RootComponent that gets swept

    // against the world for collision detection in ApplyMovement.

    //

    // IMPORTANT: CollisionCapsule must generate overlap events so we can

    // detect when the no-sweep SetActorLocation call accidentally places

    // the robot inside a wall / furniture actor. Without this the overlap

    // query in ApplyMovement returns an empty list and all collisions are

    // missed 鈫?robot phases through geometry (the R14 208m outside bug).

    CollisionCapsule = CreateDefaultSubobject<UCapsuleComponent>(TEXT("CollisionCapsule"));

    RootComponent = CollisionCapsule;

    CollisionCapsule->SetCapsuleRadius(20.0f);   // 20cm radius (40cm total width) matching PuppyPi physical body

    CollisionCapsule->SetCapsuleHalfHeight(30.0f);

    CollisionCapsule->SetCollisionProfileName(TEXT("Pawn"));

    CollisionCapsule->SetMobility(EComponentMobility::Movable);  // EXPLICITLY Movable (critical!)

    CollisionCapsule->SetGenerateOverlapEvents(true);   // REQUIRE: enable so GetOverlappingActors works

    // Explicit collision responses: Block WorldStatic / WorldDynamic so UE

    // knows walls/furniture are solid, Overlap Pawn so pedestrians don't

    // stop the robot dead (they are handled by the Bridge RVO layer).

    CollisionCapsule->SetCollisionResponseToChannel(ECC_WorldStatic,   ECR_Block);

    CollisionCapsule->SetCollisionResponseToChannel(ECC_WorldDynamic,  ECR_Block);

    CollisionCapsule->SetCollisionResponseToChannel(ECC_Pawn,          ECR_Overlap);

    CollisionCapsule->SetCollisionResponseToChannel(ECC_Camera,        ECR_Ignore);

    CollisionCapsule->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);



    // ---- Robot dog visual assembly (Spot-style: body + head + 4 legs + orange stripe) ----

    // Geometry reference:

    //   CollisionCapsule: radius 30cm, half-height 30cm 鈫?total height 60cm,

    //   floor top surface at Z=0 (PlayerStart at Z=+15cm 鈫?capsule bottom Z = 15-30 = -15cm,

    //   which clips slightly below the floor; the visual meshes sit inside the capsule

    //   so we never clip below the floor (floor Z鈭圼-5,+5], leg bottoms at Z=+5cm 鈮?floor top).

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));

    // Default material for BasicShapes 鈥?parameters "Color" (BaseColor) and "Roughness" exist,

    // but safer: create a per-mesh MID override so colors always render vividly regardless

    // of the underlying material parameter names.

    static ConstructorHelpers::FObjectFinder<UMaterialInterface> BaseMatFinder(

        TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));



    // Helper: assign solid base material to StaticMeshComponent.
    auto ApplyColorMID = [&](UStaticMeshComponent* M, const FLinearColor& Color)
    {
        if (!M) return;
        UMaterialInterface* BaseMat = BaseMatFinder.Succeeded() ? BaseMatFinder.Object : nullptr;
        if (BaseMat) M->SetMaterial(0, BaseMat);
    };



    auto MakeBox = [&](const TCHAR* Name, USceneComponent* Parent,

                       const FVector& Loc, const FVector& Scale, const FLinearColor& Color) -> UStaticMeshComponent*

    {

        UStaticMeshComponent* M = CreateDefaultSubobject<UStaticMeshComponent>(Name);

        M->SetupAttachment(Parent);

        M->SetRelativeLocation(Loc);

        M->SetCollisionEnabled(ECollisionEnabled::NoCollision);

        if (CubeMesh.Succeeded()) M->SetStaticMesh(CubeMesh.Object);

        M->SetRelativeScale3D(Scale);

        ApplyColorMID(M, Color);

        return M;

    };

    auto MakeCyl = [&](const TCHAR* Name, USceneComponent* Parent,

                       const FVector& Loc, const FVector& Scale, const FLinearColor& Color) -> UStaticMeshComponent*

    {

        UStaticMeshComponent* M = CreateDefaultSubobject<UStaticMeshComponent>(Name);

        M->SetupAttachment(Parent);

        M->SetRelativeLocation(Loc);

        M->SetCollisionEnabled(ECollisionEnabled::NoCollision);

        if (CylMesh.Succeeded()) M->SetStaticMesh(CylMesh.Object);

        M->SetRelativeScale3D(Scale);

        ApplyColorMID(M, Color);

        return M;

    };

    auto MakeSphere = [&](const TCHAR* Name, USceneComponent* Parent,

                          const FVector& Loc, const FVector& Scale, const FLinearColor& Color) -> UStaticMeshComponent*

    {

        UStaticMeshComponent* M = CreateDefaultSubobject<UStaticMeshComponent>(Name);

        M->SetupAttachment(Parent);

        M->SetRelativeLocation(Loc);

        M->SetCollisionEnabled(ECollisionEnabled::NoCollision);

        if (SphereMesh.Succeeded()) M->SetStaticMesh(SphereMesh.Object);

        M->SetRelativeScale3D(Scale);

        ApplyColorMID(M, Color);

        return M;

    };



    // PuppyPi-inspired palette: white shell, black joints, orange safety band,

    // and a blue front camera module so the robot reads clearly in the scene.

    const FLinearColor CLR_BODY(0.72f, 0.76f, 0.80f, 1.0f);

    const FLinearColor CLR_LEG (0.08f, 0.10f, 0.13f, 1.0f);

    const FLinearColor CLR_ACC (0.95f, 0.38f, 0.06f, 1.0f);

    const FLinearColor CLR_HL  (0.12f, 0.24f, 0.34f, 1.0f);

    const FLinearColor CLR_JOINT(0.18f, 0.20f, 0.23f, 1.0f);

    const FLinearColor CLR_FOOT(0.04f, 0.05f, 0.07f, 1.0f);



    // 1) Main chassis body: 60cm (X, length) 脳 40cm (Y, width) 脳 18cm (Z, thickness)

    //    Cube default 100cm 鈫?scale = (0.6, 0.4, 0.18). Bottom edge at Z = +8cm

    //    (body bottom 8cm, body top 26cm), well inside the capsule bottom (-15cm).

    RobotBodyMesh = MakeBox(TEXT("RobotBodyMesh"), RootComponent,

        /*Loc=*/ FVector(0.0f, 0.0f, 17.0f),  // centre Z = 8 + 9 = 17

        /*Scale=*/ FVector(0.58f, 0.36f, 0.16f),

        CLR_BODY);



    // Rounded upper shell: this breaks the old single-box silhouette and

    // makes the body read as a small quadruped robot rather than a cube.

    MakeSphere(TEXT("PuppyPiUpperShell"), RootComponent,

        FVector(-2.0f, 0.0f, 27.0f), FVector(0.42f, 0.30f, 0.16f), CLR_BODY);



    // 2) Head module: 20cm (X) 脳 34cm (Y) 脳 15cm (Z), mounted to the front of the body.

    //    Body front edge is at X = +30cm; head sits at X=+40cm (overhang 10cm forward, 10cm on body).

    RobotHeadMesh = MakeBox(TEXT("RobotHeadMesh"), RobotBodyMesh,

        /*Loc=*/ FVector(40.0f, 0.0f, 0.0f),  // relative to body

        /*Scale=*/ FVector(0.18f, 0.28f, 0.13f),

        CLR_BODY);



    MakeSphere(TEXT("PuppyPiHeadSensor"), RootComponent,

        FVector(39.0f, 0.0f, 27.0f), FVector(0.10f, 0.15f, 0.10f), CLR_HL);

    MakeSphere(TEXT("PuppyPiCameraLens"), RootComponent,

        FVector(49.0f, 0.0f, 27.0f), FVector(0.035f, 0.075f, 0.075f), CLR_JOINT);



    // 3) Orange accent stripe on the robot's back so it's easy to locate from above.

    AccentStripeMesh = MakeBox(TEXT("AccentStripeMesh"), RobotBodyMesh,

        /*Loc=*/ FVector(0.0f, 0.0f, +9.0f + 1.0f),  // body top + 1cm to avoid z-fighting

        /*Scale=*/ FVector(0.58f, 0.08f, 0.02f),

        CLR_ACC);



    // 4) Legs: 4 cylinders attached at body corners.

    //    Cylinder default: 200cm tall (Z axis), radius 50cm (X/Y). We want:

    //      leg height 22cm (floor 0cm to body bottom 8cm + 14cm inside body for shoulder joint)

    //      leg radius 鈮?4cm 鈫?scale (X/Y) = 4/50 = 0.08.

    //    Body corners (in body-local frame, origin at body centre):

    //      FL = (+25, +15), FR = (+25, -15), RL = (-25, +15), RR = (-25, -15)  (body is 60脳40 鈫?half 30脳20, inset 5cm for shoulders)

    //    Leg centre Z = 22/2 = 11cm (from floor) 鈫?relative to body centre (Z=17) = -6cm.

    const FVector LEG_SCALE(0.08f, 0.08f, 0.11f);  // radius 4cm 脳 height 22cm

    const FVector LEG_Z_REL(0.0f, 0.0f, -6.0f);    // relative to body centre

    LegFL_Mesh = MakeCyl(TEXT("LegFL_Mesh"), RobotBodyMesh,

        FVector(+25.0f, +15.0f, 0.0f) + LEG_Z_REL, LEG_SCALE, CLR_LEG);

    LegFR_Mesh = MakeCyl(TEXT("LegFR_Mesh"), RobotBodyMesh,

        FVector(+25.0f, -15.0f, 0.0f) + LEG_Z_REL, LEG_SCALE, CLR_LEG);

    LegRL_Mesh = MakeCyl(TEXT("LegRL_Mesh"), RobotBodyMesh,

        FVector(-25.0f, +15.0f, 0.0f) + LEG_Z_REL, LEG_SCALE, CLR_LEG);

    LegRR_Mesh = MakeCyl(TEXT("LegRR_Mesh"), RobotBodyMesh,

        FVector(-25.0f, -15.0f, 0.0f) + LEG_Z_REL, LEG_SCALE, CLR_LEG);



    // Knee joints and wide feet give the four legs a recognizable PuppyPi

    // silhouette while remaining visual-only components.

    const FVector JointScale(0.075f, 0.075f, 0.075f);

    const FVector FootScale(0.095f, 0.075f, 0.035f);

    const FVector FootZ(0.0f, 0.0f, -14.0f);

    MakeSphere(TEXT("JointFL"), RobotBodyMesh, FVector(+25.0f, +15.0f, -2.0f), JointScale, CLR_JOINT);

    MakeSphere(TEXT("JointFR"), RobotBodyMesh, FVector(+25.0f, -15.0f, -2.0f), JointScale, CLR_JOINT);

    MakeSphere(TEXT("JointRL"), RobotBodyMesh, FVector(-25.0f, +15.0f, -2.0f), JointScale, CLR_JOINT);

    MakeSphere(TEXT("JointRR"), RobotBodyMesh, FVector(-25.0f, -15.0f, -2.0f), JointScale, CLR_JOINT);

    MakeBox(TEXT("FootFL"), RobotBodyMesh, FVector(+25.0f, +15.0f, 0.0f) + LEG_Z_REL + FootZ, FootScale, CLR_FOOT);

    MakeBox(TEXT("FootFR"), RobotBodyMesh, FVector(+25.0f, -15.0f, 0.0f) + LEG_Z_REL + FootZ, FootScale, CLR_FOOT);

    MakeBox(TEXT("FootRL"), RobotBodyMesh, FVector(-25.0f, +15.0f, 0.0f) + LEG_Z_REL + FootZ, FootScale, CLR_FOOT);

    MakeBox(TEXT("FootRR"), RobotBodyMesh, FVector(-25.0f, -15.0f, 0.0f) + LEG_Z_REL + FootZ, FootScale, CLR_FOOT);



    // Spring arm (chase 3/4 view). Pulled in tight so the 60cm-long robot dog

    // is clearly visible in screenshots: arm 2.8m back, socket 1.2m up,

    // pitch -42deg (moderate 3/4 view instead of near-top-down). Socket is

    // shifted +40cm in Y so the dog sits on the LEFT side of the frame,

    // leaving room on the right to see the corridor it's cruising down.

    CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));

    CameraBoom->SetupAttachment(RootComponent);

    CameraBoom->SetUsingAbsoluteRotation(true);

    CameraBoom->TargetArmLength = 340.0f;

    CameraBoom->SocketOffset = FVector(0.0f, 70.0f, 140.0f);

    CameraBoom->SetRelativeRotation(FRotator(-42.0f, 0.0f, 0.0f));

    CameraBoom->bDoCollisionTest = false;



    FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));

    FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);

    FollowCamera->bUsePawnControlRotation = false;



    LiDARComp = CreateDefaultSubobject<UPuppyLiDARComponent>(TEXT("LiDARComp"));

    TcpComp   = CreateDefaultSubobject<UPuppyTcpServer>(TEXT("TcpComp"));

}



void APuppyRobotPawn::BeginPlay()

{

    Super::BeginPlay();



    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] ================ BEGINPLAY C++ ENTERED ================ (this should appear!)"));



    // FORCE-ENABLE TICK: Blueprints (BP_PuppyRobotPawn) often disable actor tick in CDO or

    // override Event Tick without calling Super::Tick, which silently swallows all C++ Tick logic

    // (lockstep gate, ApplyMovement, diagnostics).  Re-enable here at runtime and force

    // both flags to guarantee our C++ Tick runs.

    PrimaryActorTick.SetTickFunctionEnable(true);

    PrimaryActorTick.bCanEverTick = true;

    PrimaryActorTick.bStartWithTickEnabled = true;

    SetActorTickEnabled(true);

    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] TICK FORCE-ENABLED: bCanEverTick=%d ActorTickEnabled=%d"),

           PrimaryActorTick.bCanEverTick, IsActorTickEnabled());



    // --- DIAGNOSTIC: verify new C++ constructor subobjects actually exist.

    int32 SubObjOk = 0, SubObjTot = 9;

    auto CheckOne = [&](const TCHAR* N, UObject* P) {

        if (P != nullptr) { ++SubObjOk; UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] %s OK"), N); }

        else              { UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] %s is NULL -> blueprint needs rebuild!"), N); }

    };

    CheckOne(TEXT("CameraBoom     "), CameraBoom);

    CheckOne(TEXT("FollowCamera   "), FollowCamera);

    CheckOne(TEXT("RobotBodyMesh  "), RobotBodyMesh);

    CheckOne(TEXT("RobotHeadMesh  "), RobotHeadMesh);

    CheckOne(TEXT("AccentStripeMesh"), AccentStripeMesh);

    CheckOne(TEXT("LegFL_Mesh     "), LegFL_Mesh);

    CheckOne(TEXT("LegFR_Mesh     "), LegFR_Mesh);

    CheckOne(TEXT("LegRL_Mesh     "), LegRL_Mesh);

    CheckOne(TEXT("LegRR_Mesh     "), LegRR_Mesh);

    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] Subobject check: %d/%d (robot dog visual + camera)"), SubObjOk, SubObjTot);



    // Override stale Blueprint-saved visual state that might hide/offset the 7 meshes.

    ReinitRobotDogVisuals();



    // P0-02: load obstacle / pedestrian / initial-pose geometry from the single
    // source of truth (config/scene_home.json) before building any world geometry.
    LoadSceneHome();

    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] >>>>> FIX-SNAP: about to snap to scene_home.initial_pose (-100,-270) <<<<<"));

    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] BeginPlay, spawned at (%.1f, %.1f, %.1f)"),

           GetActorLocation().X, GetActorLocation().Y, GetActorLocation().Z);



    // --- CRITICAL FIX: Force teleport to scene_home.robot.initial_pose = (-1.0m, -3.0m) ---

    // The saved level has PlayerStart (0,0,0) or Editor-placed PlayerStart near the origin

    // which is INSIDE wall_y0_seg2 (xmin=-2.0, xmax=0.0, ymin=-0.15, ymax=0.15).  When UE

    // spawns the Pawn at that PlayerStart it collision-adjusts to (30cm, 0cm) which is

    // still adjacent to the wall, so ApplyMovement sweep=true always reports a blocking

    // hit 鈫?robot cannot move at all for the entire simulation.  To completely sidestep

    // any stale PlayerStart in the saved level we SNAP the Pawn directly to the

    // canonical initial position at BeginPlay, using SetActorLocation(false, no sweep)

    // + SetActorRotation instead of TeleportTo, because TeleportTo depends on the

    // MovementComponent which may not exist on a custom Pawn and silently no-ops.

    {

        const double  INIT_X_M  =  static_cast<double>(GInitPoseM.X);  // scene_home.robot.initial_pose.x (loaded at runtime, P0-02)

        const double  INIT_Y_M  =  static_cast<double>(GInitPoseM.Y);  // scene_home.robot.initial_pose.y (loaded at runtime, P0-02)

        constexpr double  INIT_Z_CM =  15.0;  // 15cm above floor top, matches Step9.6 PlayerStart Z

        const FVector TargetLoc(static_cast<float>(INIT_X_M * 100.0),   // -100 cm

                                static_cast<float>(INIT_Y_M * 100.0),   // -270 cm

                                static_cast<float>(INIT_Z_CM));

        const FRotator TargetRot(0.0f, 0.0f, 0.0f);

        const FVector OldLoc = GetActorLocation();

        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] FIX-SNAP-PREP: old=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f)"),

               OldLoc.X, OldLoc.Y, OldLoc.Z,

               TargetLoc.X, TargetLoc.Y, TargetLoc.Z);

        FHitResult Hit;

        const bool bLocOk = SetActorLocation(TargetLoc, /*bSweep=*/false, &Hit, ETeleportType::TeleportPhysics);

        const bool bRotOk = SetActorRotation(TargetRot, ETeleportType::TeleportPhysics);

        const FVector NewLoc = GetActorLocation();

        double DistMoved = FVector::Dist(OldLoc, NewLoc);

        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] FIX-SNAP-DONE: SetActorLocation=%s SetActorRotation=%s new=(%.1f,%.1f,%.1f) moved=%.2fcm [scene_home.robot.initial_pose (-1m,-2.7m)]"),

               bLocOk ? TEXT("SUCCESS") : TEXT("FAILED"),

               bRotOk ? TEXT("SUCCESS") : TEXT("FAILED"),

               NewLoc.X, NewLoc.Y, NewLoc.Z, DistMoved);

        if (DistMoved < 1.0) {

            UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] FIX-SNAP-FAILED: barely moved! robot likely STUCK again!"));

        }

    }



    // --- Force the FollowCamera to be the active game viewport camera.

    // Without this, the PlayerController keeps its default AbstractCameraComponent

    // which renders nothing when the default map pawn camera has no valid view.

    if (FollowCamera != nullptr)

    {

        FollowCamera->SetActive(true);

        APlayerController* PC = Cast<APlayerController>(GetController());

        if (PC == nullptr)

        {

            PC = UGameplayStatics::GetPlayerController(this, 0);

        }

        if (PC != nullptr)

        {

            PC->SetViewTarget(this);

            UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] SetViewTarget -> FollowCamera (PC=%s)"), *GetNameSafe(PC));

        }

        else

        {

            UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] SetViewTarget FAILED: PlayerController 0 is NULL"));

        }

    }



    if (TcpComp != nullptr)

    {

        TcpComp->OnCmdVelReceived.AddDynamic(this, &APuppyRobotPawn::HandleCmdVel);

        TcpComp->OnStepAck.AddDynamic(this, &APuppyRobotPawn::HandleStepAck);
        TcpComp->OnSimulationEnd.AddDynamic(this, &APuppyRobotPawn::HandleSimulationEnd);

        UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] TcpComp bound, Port=%d"), TcpComp->Port);

    }

    else

    {

        UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] TcpComp is NULL!"));

    }



    if (LiDARComp != nullptr)

    {

        UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] LiDARComp OK"));

    }

    else

    {

        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] LiDARComp is NULL!"));

    }



    // Spawn 5 dynamic pedestrians from scene_home.json config.

    // scene_home uses meters; UE world uses cm -> multiply all by 100 (M_TO_CM).

    SpawnPedestriansFromScene();

    // Spawn 3D obstacle visuals from the runtime-loaded scene (P0-02) so LiDAR can see them.
    SpawnObstacleVisuals();

    // --- AUTO-SCREENSHOT for visual QA (2026-08-13) ---
    // Display-independent capture: a SceneCapture2D attached to the follow
    // camera renders the scene into an offscreen RenderTarget every frame,
    // which we then export to PNG. This works even with no physical display
    // (unlike HighResScreenshot, which needs a presented viewport).
    {
        UTextureRenderTarget2D* QART = NewObject<UTextureRenderTarget2D>(this);
        QART->InitAutoFormat(1280, 720);
        QART->UpdateResource();

        USceneCaptureComponent2D* QACap = NewObject<USceneCaptureComponent2D>(this);
        QACap->SetupAttachment(FollowCamera);
        QACap->SetRelativeLocation(FVector::ZeroVector);
        QACap->TextureTarget = QART;
        QACap->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        QACap->bCaptureEveryFrame = true;
        QACap->RegisterComponent();

        static const float ShotTimes[] = { 4.0f, 20.0f, 45.0f, 80.0f, 110.0f };
        int32 ShotIdx = 0;
        for (float t : ShotTimes)
        {
            FTimerHandle Th;
            int32 Idx = ShotIdx++;
            GetWorld()->GetTimerManager().SetTimer(
                Th, [this, QART, Idx]()
                {
                    const FString Dir = FPaths::ProjectSavedDir() / TEXT("Screenshots/Windows");
                    IFileManager::Get().MakeDirectory(*Dir, true);
                    const FString Path = Dir / FString::Printf(TEXT("QA_Shot_%02d.png"), Idx);
                    TUniquePtr<FArchive> Ar(IFileManager::Get().CreateFileWriter(*Path));
                    if (Ar)
                    {
                        FImageUtils::ExportRenderTarget2DAsPNG(QART, *Ar);
                        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] QA SCREENSHOT -> %s"), *Path);
                    }
                    else
                    {
                        UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] QA SCREENSHOT FAILED to open %s"), *Path);
                    }
                }, t, false);
        }
        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] QA SCREENSHOT SceneCapture armed (5 shots)"));
    }
}


void APuppyRobotPawn::SpawnPedestriansFromScene()

{

    UWorld* World = GetWorld();

    if (World == nullptr)

    {

        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] No world, skip pedestrian spawn"));

        return;

    }



    // ---- De-duplicate: destroy any existing PuppyPedestrianActor already present in level ----

    // ue_auto_setup.py Step6_create_pedestrians() pre-places 5 static pedestrians in the Editor

    // saved level.  BeginPlay() would then spawn 5 MORE copies here (from C++) for a total of

    // 10, which doubles RVO computational load and spawns two people at each of the 5 spawn

    // points (they get stuck together).  Destroy the Editor-placed copies and re-spawn from C++

    // (C++ version has the proper Velocity = {vx,vy} initialization for self-driving pedestrians).

    TArray<AActor*> ExistingPeds;

    UGameplayStatics::GetAllActorsOfClass(World, APuppyPedestrianActor::StaticClass(), ExistingPeds);

    if (ExistingPeds.Num() > 0)

    {

        int32 Destroyed = 0;

        for (AActor* OldPed : ExistingPeds)

        {

            if (OldPed && OldPed != this && OldPed->IsValidLowLevel())

            {

                OldPed->Destroy();

                ++Destroyed;

            }

        }

        UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] De-dup: destroyed %d pre-existing Editor-placed pedestrians (C++ will re-spawn %d with velocity init)"),

               Destroyed, GHomePedestrians.Num());

    }



    // P0-02: pedestrians now come from config/scene_home.json (loaded at
    // BeginPlay into GHomePedestrians).  Positions: meters -> cm (x M_TO_CM);
    // velocities: m/s -> cm/s (x M_TO_CM).  Falls back to an empty list if the
    // JSON has no pedestrians, so the robot still runs.



    int32 Spawned = 0;

    for (int32 i = 0; i < GHomePedestrians.Num(); ++i)

    {

        const FPedSpawn& C = GHomePedestrians[i];

        FActorSpawnParameters Params;

        Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

        const FVector Loc(C.Xm * M_TO_CM, C.Ym * M_TO_CM, 0.0f);

        APuppyPedestrianActor* Ped = World->SpawnActor<APuppyPedestrianActor>(

            APuppyPedestrianActor::StaticClass(), Loc, FRotator::ZeroRotator, Params);

        if (Ped != nullptr)

        {

            Ped->Initialize(FString(C.Name),

                       C.Xm * M_TO_CM, C.Ym * M_TO_CM,

                       C.Vxm * M_TO_CM, C.Vym * M_TO_CM);

            ++Spawned;

            UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] Spawned %s @ (%.0f,%.0f) v=(%.1f,%.1f) cm/s"),

                   C.Name, C.Xm * M_TO_CM, C.Ym * M_TO_CM,

                   C.Vxm * M_TO_CM, C.Vym * M_TO_CM);

        }

        else

        {

            UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] Failed to spawn %s"), C.Name);

        }

    }

    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] Pedestrian spawn: %d/%d OK"), Spawned, GHomePedestrians.Num());

}



void APuppyRobotPawn::ReinitRobotDogVisuals()

{

    // Load basic engine shape meshes as robust fallbacks

    UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));

    UStaticMesh* CylMesh  = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

    UMaterialInterface* BaseMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));



    // Attempt loading official Hiwonder PuppyPi converted meshes from /Game/PuppyPi/

    UStaticMesh* OfficialBaseMesh   = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/PuppyPi/base_link.base_link"));

    UStaticMesh* OfficialLidarMesh  = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/PuppyPi/lidar_Link.lidar_Link"));

    UStaticMesh* OfficialCameraMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/PuppyPi/camera_link.camera_link"));



    const bool bHasOfficialModel = (OfficialBaseMesh != nullptr);



    auto ApplyMID = [&](UStaticMeshComponent* M, const FLinearColor& Color)

    {

        if (!M) return;

        M->SetVisibility(true, true);

        M->SetHiddenInGame(false, true);

        M->SetCollisionEnabled(ECollisionEnabled::NoCollision);

        if (BaseMat != nullptr) M->SetMaterial(0, BaseMat);

        UMaterialInstanceDynamic* MID = M->CreateAndSetMaterialInstanceDynamic(0);

        if (MID)

        {

            MID->SetVectorParameterValue(TEXT("BaseColor"), Color);

            MID->SetVectorParameterValue(TEXT("Color"), Color);

            MID->SetScalarParameterValue(TEXT("Roughness"), 0.85f);

            MID->SetScalarParameterValue(TEXT("Metallic"), 0.25f);

        }

    };



    const FLinearColor CLR_BODY(0.22f, 0.24f, 0.28f, 1.0f);

    const FLinearColor CLR_LEG (0.08f, 0.09f, 0.11f, 1.0f);

    const FLinearColor CLR_ACC (0.98f, 0.55f, 0.08f, 1.0f);  // Hiwonder orange

    const FLinearColor CLR_HL  (0.88f, 0.88f, 0.92f, 1.0f);



    // ===== 1. BODY CHASSIS =====

    if (RobotBodyMesh)

    {

        if (bHasOfficialModel)

        {

            RobotBodyMesh->SetStaticMesh(OfficialBaseMesh);

            RobotBodyMesh->SetupAttachment(RootComponent);

            RobotBodyMesh->SetRelativeLocation(FVector(0.0f, 0.0f, 15.0f));

            RobotBodyMesh->SetRelativeScale3D(FVector(200.0f, 200.0f, 200.0f));  // 1m -> 100cm scale factor

        }

        else if (CubeMesh != nullptr)

        {

            RobotBodyMesh->SetStaticMesh(CubeMesh);

            RobotBodyMesh->SetupAttachment(RootComponent);

            RobotBodyMesh->SetRelativeLocation(FVector(0.0f, 0.0f, 17.0f));

            RobotBodyMesh->SetRelativeScale3D(FVector(0.60f, 0.40f, 0.18f));

        }

        ApplyMID(RobotBodyMesh, CLR_BODY);

    }



    // ===== 2. HEAD / CAMERA MODULE =====

    if (RobotHeadMesh && RobotBodyMesh)

    {

        if (OfficialCameraMesh != nullptr)

        {

            RobotHeadMesh->SetStaticMesh(OfficialCameraMesh);

            RobotHeadMesh->SetupAttachment(RobotBodyMesh);

            RobotHeadMesh->SetRelativeLocation(FVector(40.0f, 0.0f, 5.0f));

            RobotHeadMesh->SetRelativeScale3D(FVector(200.0f, 200.0f, 200.0f));

        }

        else if (CubeMesh != nullptr)

        {

            RobotHeadMesh->SetStaticMesh(CubeMesh);

            RobotHeadMesh->SetupAttachment(RobotBodyMesh);

            RobotHeadMesh->SetRelativeLocation(FVector(40.0f, 0.0f, 0.0f));

            RobotHeadMesh->SetRelativeScale3D(FVector(0.20f, 0.34f, 0.15f));

        }

        ApplyMID(RobotHeadMesh, CLR_HL);

    }



    // ===== 3. ORANGE ACCENT / LIDAR STRIPE =====

    if (AccentStripeMesh && RobotBodyMesh)

    {

        if (OfficialLidarMesh != nullptr)

        {

            AccentStripeMesh->SetStaticMesh(OfficialLidarMesh);

            AccentStripeMesh->SetupAttachment(RobotBodyMesh);

            AccentStripeMesh->SetRelativeLocation(FVector(0.0f, 0.0f, 12.0f));

            AccentStripeMesh->SetRelativeScale3D(FVector(200.0f, 200.0f, 200.0f));

            ApplyMID(AccentStripeMesh, CLR_ACC);

        }

        else if (CubeMesh != nullptr)

        {

            AccentStripeMesh->SetStaticMesh(CubeMesh);

            AccentStripeMesh->SetupAttachment(RobotBodyMesh);

            AccentStripeMesh->SetRelativeLocation(FVector(0.0f, 0.0f, 10.0f));

            AccentStripeMesh->SetRelativeScale3D(FVector(0.58f, 0.08f, 0.02f));

            ApplyMID(AccentStripeMesh, CLR_ACC);

        }

    }



    // ===== 4. LEGS =====

    const FVector LEG_SCALE(0.08f, 0.08f, 0.11f);   // radius 4cm 脳 height 22cm

    const FVector LEG_Z_REL(0.0f, 0.0f, -6.0f);      // relative to body centre

    auto InitLeg = [&](UStaticMeshComponent* Leg, float X, float Y)

    {

        if (!Leg || !RobotBodyMesh) return;

        if (CylMesh != nullptr) Leg->SetStaticMesh(CylMesh);

        Leg->SetupAttachment(RobotBodyMesh);

        Leg->SetRelativeLocation(FVector(X, Y, 0.0f) + LEG_Z_REL);

        Leg->SetRelativeScale3D(LEG_SCALE);

        ApplyMID(Leg, CLR_LEG);

    };

    InitLeg(LegFL_Mesh, +25.0f, +15.0f);

    InitLeg(LegFR_Mesh, +25.0f, -15.0f);

    InitLeg(LegRL_Mesh, -25.0f, +15.0f);

    InitLeg(LegRR_Mesh, -25.0f, -15.0f);



    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] ReinitRobotDogVisuals DONE "

           "(bHasOfficialModel=%d body=%d head=%d stripe=%d legFL=%d legFR=%d legRL=%d legRR=%d)"),

           bHasOfficialModel ? 1 : 0,

           RobotBodyMesh ? 1 : 0, RobotHeadMesh ? 1 : 0, AccentStripeMesh ? 1 : 0,

           LegFL_Mesh ? 1 : 0, LegFR_Mesh ? 1 : 0, LegRL_Mesh ? 1 : 0, LegRR_Mesh ? 1 : 0);

}



void APuppyRobotPawn::Tick(float DeltaTime)

{

    Super::Tick(DeltaTime);

    if (bSimulationEnded)
    {
        CurrentVelocity = FVector::ZeroVector;
        CurrentAngularVelocity = 0.0f;
        bStepAckReceived = false;
        return;
    }



    // DIAGNOSTIC (throttled ~3s): verify Tick is running and StepAck state

    static double LastTickDiag = 0.0;

    const double NowT = FPlatformTime::Seconds();

    const bool bTickDiag = (NowT - LastTickDiag > 3.0);

    if (bTickDiag) {

        LastTickDiag = NowT;

        const FVector CurLoc = GetActorLocation();

        UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] TICK-3s dt=%.4fs Loc=(%.1f,%.1f,%.1f) StepAckRcvd=%d Vel=(%.1f,%.1f)cm/s Frame=%d"),

               DeltaTime, CurLoc.X, CurLoc.Y, CurLoc.Z,

               bStepAckReceived ? 1 : 0,

               CurrentVelocity.X, CurrentVelocity.Y, FrameCount);

    }



    // R20 FIX: 鐪熸鐨凩ockstep 30Hz 鈥?鍙湪鏀跺埌STEP_ACK鍚庢墠绉诲姩+鍙戦€佷笅涓€甯?

    //   R19z闂: 姣廡ick閮絊endSensorData(125Hz), 浣咮ridge鍙鐞?0Hz, 瀵艰嚧:

    //   1) Bridge绔疶CP缂撳啿鍖虹Н鍘?5甯?绉掔殑浼犳劅鍣ㄦ暟鎹?

    //   2) Bridge鍥炲鐨凜MD_VEL+STEP_ACK鍦║E绔篃绉帇

    //   3) HandleStepAck鐨凞IRECT-APPLY瀵艰嚧涓€甯у唴澶氭ApplyMovement

    //   淇: 绗竴甯у彂閫佸垵濮嬩紶鎰熷櫒鏁版嵁, 鍚庣画鍙湪鏀跺埌STEP_ACK鍚庣Щ鍔?鍙戦€?

    // P0-03: do not drive sensor/lockstep sends until the v2 handshake has
    // completed. Otherwise the first tick (which runs BEFORE the handshake
    // finishes) consumes bFirstFrame by calling SendSensorData() and returning
    // early, after which the pawn only sends again on a STEP_ACK that never
    // arrives -> deadlock. Gate here so bFirstFrame is only spent post-handshake.
    if (!TcpComp->IsHandshaked())
    {
        return;
    }

    static bool bFirstFrame = true;

    if (bFirstFrame)

    {

        bFirstFrame = false;

        LastStepAckTime = NowT;

        SendSensorData();

        return;

    }



    if (bStepAckReceived)

    {

        // The bridge advances exactly one 30 Hz simulation step per ACK.

        // Use the protocol timestep rather than render Tick DeltaTime so

        // headless/high-FPS UE runs preserve the same physical trajectory.

        constexpr float LockstepDeltaTime = 1.0f / 30.0f;

        ApplyMovement(LockstepDeltaTime);

        bStepAckReceived = false;

        ++FrameCount;

        SendSensorData();

    }

    else

    {

        // Keep strict lockstep: never resend the same sensor frame after a

        // timeout. The bridge treats each complete sensor bundle as one

        // simulation frame, so retrying here would advance the bridge multiple

        // times for the same UE frame and make it hit the frame limit early.

        const double AckTimeout = NowT - LastStepAckTime;

        if (AckTimeout > 0.5)

        {

            UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] LOCKSTEP-WATCHDOG: waiting %.2fs for STEP_ACK (Frame=%d)"),

                   AckTimeout, FrameCount);

            LastStepAckTime = NowT;

        }

    }

}



void APuppyRobotPawn::HandleCmdVel(const FPuppyCmdVel& CmdVel)

{

    // nav_ue_bridge sends vx/vy in m/s, wz in rad/s.

    // Convert to cm/s and deg/s for UE.

    CurrentVelocity = FVector(CmdVel.Vx * M_TO_CM, CmdVel.Vy * M_TO_CM, 0.0f);

    CurrentAngularVelocity = FMath::RadiansToDegrees(CmdVel.Wz);

    // P0-03: EMERGENCY_STOP latch -- Nav ordered an immediate safe stop.
    if (TcpComp && TcpComp->IsEmergencyStopped())
    {
        CurrentVelocity = FVector::ZeroVector;
        CurrentAngularVelocity = 0.0f;
    }



    // Diagnostic: throttle to once per 3 seconds so the log stays readable.

    static double LastLog = 0.0;

    const double Now = FPlatformTime::Seconds();

    if (Now - LastLog > 3.0)

    {

        LastLog = Now;

        UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] HandleCmdVel vx=%.2fm/s vy=%.2fm/s wz=%.2frad/s  |UE cm/s|=(%.1f,%.1f) speed=%.1fcm/s"),

               CmdVel.Vx, CmdVel.Vy, CmdVel.Wz,

               CurrentVelocity.X, CurrentVelocity.Y, CurrentVelocity.Size2D());

    }

}



void APuppyRobotPawn::HandleStepAck()

{

    // R20 FIX: 涓嶅啀鍦ㄦ澶勭洿鎺ヨ皟鐢?ApplyMovement銆?

    //   R19z闂: 姣忎釜 STEP_ACK 鍥炶皟鐩存帴璋冪敤 ApplyMovement, 瀵艰嚧:

    //   1) TCP娑堟伅绉帇鏃朵竴甯у唴澶氭绉诲姩, Frame鍙锋毚澧?3绉掑唴Frame 0鈫?000+)

    //   2) 浣跨敤杩囨椂鐨勯€熷害鎸囦护 鈥?鏈哄櫒浜哄湪鍙充笅瑙?4.88,-3.88)鏃惰繕鍦ㄧ敤

    //      宸︿笂瑙?-4.88,3.88)鐨勮劚鍥伴€熷害(1.57,-1.24), 鏈濊竟鐣屽绉诲姩琚獵lamp閽冲埗

    //   3) Tick涓殑lockstep閫昏緫(bStepAckReceived)姘歌繙涓嶆墽琛? 鍥犱负姝ゅ宸查噸缃?

    //   淇: 鍙缃爣蹇? 鐢盩ick缁熶竴澶勭悊姣忓抚鏈€澶氫竴娆pplyMovement銆?

    bStepAckReceived = true;

    LastStepAckTime = FPlatformTime::Seconds();  // R21: 璁板綍STEP_ACK鎺ユ敹鏃堕棿鐢ㄤ簬watchdog

}



void APuppyRobotPawn::HandleSimulationEnd()
{
    bSimulationEnded = true;
    bStepAckReceived = false;
    CurrentVelocity = FVector::ZeroVector;
    CurrentAngularVelocity = 0.0f;
    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] Simulation ended; lockstep stopped at Frame=%d"), FrameCount);
}



void APuppyRobotPawn::SendSensorData()

{

    if (TcpComp == nullptr)

    {

        return;

    }



    static double LastDebugTime = 0.0;

    const double Now = FPlatformTime::Seconds();

    const bool bDebugTick = (Now - LastDebugTime) > 3.0;

    if (bDebugTick) LastDebugTime = Now;



    const bool bConnected = TcpComp->IsConnected();
    const bool bHandshaked = TcpComp->IsHandshaked();

    if (!bConnected || !bHandshaked)

    {

        if (bDebugTick)

        {

            int32 ConnState = TcpComp->GetConnectionStateRaw();

            UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] NOT ready (connected=%d handshaked=%d state=%d). Not sending (t=%.1fs)."),

                   (int)bConnected, (int)bHandshaked, ConnState, Now);

        }

        // P0-03: 断线停车 (stop-on-disconnect) -- if we cannot talk to Nav,
        // halt the robot immediately so it never coasts on a stale command.
        CurrentVelocity = FVector::ZeroVector;
        CurrentAngularVelocity = 0.0f;
        return;

    }



    // 1. LiDAR scan (distances in cm -- SendLidar converts to meters)

    bool bSentLidar = false, bSentGT = false, bSentPed = false;

    if (LiDARComp != nullptr)

    {

        TArray<float> Distances;

        float MaxRange = 0.0f;

        LiDARComp->GetScanData(Distances, MaxRange);

        bSentLidar = TcpComp->SendLidar(Distances, MaxRange);

        if (bDebugTick)

        {

            UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] LiDAR: rays=%d max=%.1fcm sent=%d"),

                   Distances.Num(), MaxRange, bSentLidar);

        }

    }



    // 2. Ground truth pose (cm -> SendGroundTruth converts to meters; yaw in radians)

    const FVector Loc = GetActorLocation();

    const FRotator Rot = GetActorRotation();

    const double YawRad = FMath::DegreesToRadians(static_cast<double>(Rot.Yaw));

    bSentGT = TcpComp->SendGroundTruth(static_cast<double>(Loc.X), static_cast<double>(Loc.Y), YawRad);



    // 3. Pedestrian state (positions in cm, velocities in cm/s)

    TArray<AActor*> Pedestrians;

    UGameplayStatics::GetAllActorsOfClass(GetWorld(), APuppyPedestrianActor::StaticClass(), Pedestrians);



    TArray<float> PedData;

    PedData.SetNumZeroed(Pedestrians.Num() * 4);



    for (int32 i = 0; i < Pedestrians.Num(); ++i)

    {

        APuppyPedestrianActor* Ped = Cast<APuppyPedestrianActor>(Pedestrians[i]);

        if (Ped == nullptr)

        {

            continue;

        }

        const FVector PedLoc = Ped->GetActorLocation();

        PedData[i * 4 + 0] = static_cast<float>(PedLoc.X);

        PedData[i * 4 + 1] = static_cast<float>(PedLoc.Y);

        PedData[i * 4 + 2] = Ped->Velocity.X;

        PedData[i * 4 + 3] = Ped->Velocity.Y;

    }



    bSentPed = TcpComp->SendPedState(PedData);



    if (bDebugTick)

    {

        UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] Sent frame: GT@(%.1f,%.1f) sent=%d Ped(%d)=%d Ack=%d StepAckWait=%d"),

               Loc.X, Loc.Y, bSentGT, Pedestrians.Num(), bSentPed,

               bSentLidar ? 1 : 0,

               bStepAckReceived ? 1 : 0);

    }

}



// ==========================================================================

//  SCENE HOME GEO-FENCE + OBSTACLE BBOX CHECK (from scene_home.json)

//  UE movement is now bSweep=false to guarantee the robot *can* move (the

//  previous bSweep=true permanently jammed even in free space due to tiny

//  spawn capsule overlaps).  We therefore re-impose scene boundaries +

//  obstacle collision HERE manually to prevent the robot from flying out of

//  the map / phasing through walls (the problem discovered during R14

//  3-hour soak test where the robot flew 208m outside the scene in 600s).

//  Scene coordinate convention in this block: CENTIMETERS (matches UE world).

//  scene_home.json gives METERS so we multiply by 100 here.

// ==========================================================================

// ==========================================================================
// P0-02 (2026-08-15) -- SINGLE SOURCE OF TRUTH
// --------------------------------------------------------------------------
// The 18 hard-coded boxes below used to be a MIRROR of config/scene_home.json.
// A mirror can (and did) silently drift: when it disagreed with the JSON the
// nav stack planned into walls that only existed in UE and the dog froze
// (the original 85%-stuck / 0%-targets symptom).  The geometry is now LOADED
// FROM THE JSON AT RUNTIME, so there is exactly one description of the room.
// Change one box in scene_home.json and the C++ planner, the UE simulation and
// the 2D planner all see it on the next run.
//
// Config path resolution order:
//   1. -SceneConfig=<path>  (set by run_ue_validation.py -- single source)
//   2. %PUPPY_SCENE_CONFIG%  environment variable
//   3. fall-back absolute path (must match bridge ../../config and the
//      run_ue_validation.py SCENE constant)
// On load failure we fall back to kFallbackObstacles[] so a run still
// completes, but log a LOUD ERROR -- a missing config is a bug, not a default.
// ==========================================================================
// Compiled-in fallback -- ONLY used when the JSON cannot be read.  Kept in sync
// with config/scene_home.json v6.0 (18 boxes, metres * 100 = cm).
static const FHomeObstacle kFallbackObstacles[] = {
    { TEXT("wall_south        "),  -450.0f,  -350.0f,   450.0f,  -330.0f },
    { TEXT("wall_north        "),  -450.0f,   330.0f,   450.0f,   350.0f },
    { TEXT("wall_west         "),  -450.0f,  -350.0f,  -430.0f,   350.0f },
    { TEXT("wall_east         "),   430.0f,  -350.0f,   450.0f,   350.0f },
    { TEXT("wall_h_bed_l      "),  -430.0f,    80.0f,  -120.0f,   100.0f },
    { TEXT("wall_h_mid        "),    30.0f,    80.0f,   150.0f,   100.0f },
    { TEXT("wall_h_bath_r     "),   320.0f,    80.0f,   430.0f,   100.0f },
    { TEXT("wall_v_bed_bath   "),    40.0f,   100.0f,    60.0f,   330.0f },
    { TEXT("sofa              "),  -380.0f,  -330.0f,  -180.0f,  -260.0f },
    { TEXT("coffee_table      "),  -140.0f,  -220.0f,   -40.0f,  -150.0f },
    { TEXT("tv_console        "),  -380.0f,    40.0f,  -280.0f,    80.0f },
    { TEXT("counter           "),   360.0f,  -330.0f,   430.0f,    80.0f },
    { TEXT("dining_table      "),   150.0f,  -280.0f,   300.0f,  -200.0f },
    { TEXT("bed               "),  -400.0f,   140.0f,  -220.0f,   300.0f },
    { TEXT("wardrobe          "),  -210.0f,   220.0f,   -80.0f,   310.0f },
    { TEXT("toilet            "),    90.0f,   130.0f,   150.0f,   200.0f },
    { TEXT("bathtub           "),   280.0f,   140.0f,   400.0f,   300.0f },
    { TEXT("sink              "),   160.0f,   290.0f,   240.0f,   330.0f },
};

static FString ResolveSceneConfigPath()
{
    FString Arg;
    if (FCommandLine::IsInitialized())
    {
        const TCHAR* CmdLine = FCommandLine::Get();
        if (FParse::Value(CmdLine, TEXT("-SceneConfig="), Arg) && !Arg.IsEmpty())
        {
            return Arg;
        }
    }
    const FString Env = FPlatformMisc::GetEnvironmentVariable(TEXT("PUPPY_SCENE_CONFIG"));
    if (!Env.IsEmpty())
    {
        return Env;
    }
    return TEXT("E:/puppyfangzhen/config/scene_home.json");
}

static int32 LoadSceneHome()
{
    GHomeObstacles.Reset();
    GHomePedestrians.Reset();
    GInitPoseM = FVector2D(0.5f, -2.2f);

    const FString Path = ResolveSceneConfigPath();
    FString JsonText;
    if (!FFileHelper::LoadFileToString(JsonText, *Path))
    {
        UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] P0-02 LoadSceneHome FAILED to read %s -- using compiled-in fallback (18 boxes). THIS IS A CONFIG BUG."), *Path);
        GHomeObstacles.Append(kFallbackObstacles, UE_ARRAY_COUNT(kFallbackObstacles));
        return GHomeObstacles.Num();
    }
    TSharedPtr<FJsonObject> Root;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonText);
    if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    {
        UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] P0-02 LoadSceneHome FAILED to parse JSON %s -- using compiled-in fallback."), *Path);
        GHomeObstacles.Append(kFallbackObstacles, UE_ARRAY_COUNT(kFallbackObstacles));
        return GHomeObstacles.Num();
    }

    const double M = 100.0;  // metres -> cm
    const TArray<TSharedPtr<FJsonValue>>* ObsArr = nullptr;
    if (Root->TryGetArrayField(TEXT("obstacles"), ObsArr))
    {
        for (const TSharedPtr<FJsonValue>& V : *ObsArr)
        {
            const TSharedPtr<FJsonObject> O = V->AsObject();
            if (!O.IsValid()) continue;
            FHomeObstacle H;
            H.Name = *O->GetStringField(TEXT("name"));
            H.Xmin = static_cast<float>(O->GetNumberField(TEXT("xmin")) * M);
            H.Ymin = static_cast<float>(O->GetNumberField(TEXT("ymin")) * M);
            H.Xmax = static_cast<float>(O->GetNumberField(TEXT("xmax")) * M);
            H.Ymax = static_cast<float>(O->GetNumberField(TEXT("ymax")) * M);
            GHomeObstacles.Add(H);
        }
    }
    const TArray<TSharedPtr<FJsonValue>>* PedArr = nullptr;
    if (Root->TryGetArrayField(TEXT("pedestrians"), PedArr))
    {
        for (const TSharedPtr<FJsonValue>& V : *PedArr)
        {
            const TSharedPtr<FJsonObject> P = V->AsObject();
            if (!P.IsValid()) continue;
            FPedSpawn S;
            S.Name = *P->GetStringField(TEXT("name"));
            S.Xm   = static_cast<float>(P->GetNumberField(TEXT("x")));
            S.Ym   = static_cast<float>(P->GetNumberField(TEXT("y")));
            S.Vxm  = static_cast<float>(P->GetNumberField(TEXT("vx")));
            S.Vym  = static_cast<float>(P->GetNumberField(TEXT("vy")));
            GHomePedestrians.Add(S);
        }
    }
    const TSharedPtr<FJsonObject>* RobObj = nullptr;
    if (Root->TryGetObjectField(TEXT("robot"), RobObj) && RobObj->IsValid())
    {
        const TSharedPtr<FJsonObject>* PoseObj = nullptr;
        if ((*RobObj)->TryGetObjectField(TEXT("initial_pose"), PoseObj) && PoseObj->IsValid())
        {
            GInitPoseM = FVector2D(
                static_cast<float>((*PoseObj)->GetNumberField(TEXT("x"))),
                static_cast<float>((*PoseObj)->GetNumberField(TEXT("y"))));
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] P0-02 LoadSceneHome OK: %s -> %d obstacles, %d pedestrians, init_pose=(%.2f,%.2f)m"),
           *Path, GHomeObstacles.Num(), GHomePedestrians.Num(), GInitPoseM.X, GInitPoseM.Y);
    return GHomeObstacles.Num();
}

// ==========================================================================
// R32 (2026-08-14) -- P0-02 "GEOMETRY DOUBLE-SOURCE" AUDIT
// --------------------------------------------------------------------------
// Symptom that motivated this: the nav stack plans on config/scene_home.json
// (18 boxes) while UE simulates whatever colliders happen to exist in the
// loaded level.  When the two disagree the planner routes the robot straight
// into a wall that only exists in UE; the reactive LiDAR filter then correctly
// refuses to move and the dog freezes forever (stuck_ratio 85 %, targets 0 %).
// Offline forensics on the ray dump pinned a phantom vertical face at x ~ 2.05 m
// spanning y in [0.4, 1.8] -- a surface no JSON box can explain.
//
// Guessing which actor owns that face is hopeless from the outside, so dump the
// ground truth: every collider a LiDAR ray can actually hit.  The LiDAR uses
// LineTraceSingleByObjectType(ECC_WorldStatic | ECC_WorldDynamic), so the test
// is "component object type is WorldStatic/WorldDynamic AND query collision is
// enabled" -- response channels are irrelevant for object-type queries.
//
// Anything printed as PHANTOM is, by definition, geometry the navigation stack
// cannot see.  Keep this audit in the build: it turns a multi-hour forensic
// hunt into one grep and guards against the whole class of regression.
// ==========================================================================
static void AuditWorldGeometry(UWorld* World, AActor* SelfPawn)
{
    if (!World) return;

    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), AllActors);

    int32 NumOurs = 0, NumPhantom = 0, NumSelf = 0;

    UE_LOG(LogTemp, Warning,
           TEXT("[GEOM-AUDIT] ===== BEGIN (every collider a LiDAR ray can hit; metres) ====="));

    for (AActor* A : AllActors)
    {
        if (!A) continue;

        TArray<UPrimitiveComponent*> Prims;
        A->GetComponents<UPrimitiveComponent>(Prims);

        bool bVisibleToLidar = false;
        FBox Box(ForceInit);
        for (UPrimitiveComponent* P : Prims)
        {
            if (!P) continue;
            const ECollisionEnabled::Type CE = P->GetCollisionEnabled();
            const bool bQuery = (CE == ECollisionEnabled::QueryOnly ||
                                 CE == ECollisionEnabled::QueryAndPhysics);
            if (!bQuery) continue;
            const ECollisionChannel Obj = P->GetCollisionObjectType();
            if (Obj != ECC_WorldStatic && Obj != ECC_WorldDynamic) continue;
            bVisibleToLidar = true;
            Box += P->Bounds.GetBox();
        }
        if (!bVisibleToLidar) continue;

        const bool bSelf = (A == SelfPawn);
        const bool bOurs = A->ActorHasTag(FName(TEXT("PuppyObs")));
        // Pedestrians are legitimate dynamic obstacles: the bridge receives their
        // pose every frame over PED_STATE, so a LiDAR hit on them is expected and
        // is NOT a map inconsistency.  A ground slab cannot be hit by a horizontal
        // ray.  Engine infra actors (e.g. the gameplay-debugger replicator) report
        // infinite bounds but carry no geometry.
        const bool bPed  = A->IsA(APuppyPedestrianActor::StaticClass());
        const FVector Sz = Box.GetSize();
        const bool bSlab = (Box.Max.Z <= 5.0f) && (Sz.X >= 300.0f) && (Sz.Y >= 300.0f);
        const bool bInfra = (Sz.X > 100000.0f) || (Sz.Y > 100000.0f);

        const TCHAR* Kind =
            bSelf  ? TEXT("SELF   ") :
            bOurs  ? TEXT("OURS   ") :
            bPed   ? TEXT("PED    ") :
            bSlab  ? TEXT("FLOOR  ") :
            bInfra ? TEXT("INFRA  ") : TEXT("PHANTOM");

        if (bSelf)                             ++NumSelf;
        else if (bOurs)                        ++NumOurs;
        else if (bPed || bSlab || bInfra)      { /* benign, not counted */ }
        else                                   ++NumPhantom;

        FString Label = A->GetName();
#if WITH_EDITOR
        Label = A->GetActorLabel();
#endif
        UE_LOG(LogTemp, Warning,
               TEXT("[GEOM-AUDIT] %s x[%7.3f,%7.3f] y[%7.3f,%7.3f] z[%7.3f,%7.3f]  %s  (class=%s name=%s)"),
               Kind,
               Box.Min.X / 100.0, Box.Max.X / 100.0,
               Box.Min.Y / 100.0, Box.Max.Y / 100.0,
               Box.Min.Z / 100.0, Box.Max.Z / 100.0,
               *Label, *A->GetClass()->GetName(), *A->GetName());
    }

    UE_LOG(LogTemp, Warning,
           TEXT("[GEOM-AUDIT] ===== END  ours=%d self=%d PHANTOM=%d (phantom>0 means the nav map is a lie) ====="),
           NumOurs, NumSelf, NumPhantom);
}

void APuppyRobotPawn::SpawnObstacleVisuals()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // ======================================================================
    // R32 (2026-08-14) -- WHITELIST purge of level-authored geometry.
    // ROOT CAUSE of the 85 %-stuck / 0 %-targets run (P0-02 geometry double-source)
    // ----------------------------------------------------------------------
    // HomeMap.umap still stores the pre-v6.0 apartment (8 rooms, ~65 props;
    // scene_home.json was since redesigned down to 18 boxes).  The previous
    // cleanup destroyed actors whose LABEL began with OBS_/Obs_/PATROL_ and
    // logged "Cleaned up 62 old obstacle actors", which read like success --
    // but in that level every prop is TWO actors: a logical parent
    // (OBS_dining_chair1) plus a separate mesh actor that owns the collider
    // (FURN_Generic).  AActor::Destroy() does not destroy attached child
    // ACTORS, so all 58 FURN_* colliders survived every single run while
    // remaining completely invisible to the C++ nav stack, which plans on the
    // 18 boxes of scene_home.json.
    //
    // Measured consequence (run r31c): a stale dining chair occupying
    // x[2.025,2.375] y[1.325,1.675] sat inside what JSON calls the bathroom
    // doorway.  Against the toilet's east face (x=1.5) it left a 0.525 m gap,
    // but the robot needs 2*r_pass = 0.54 m to pass.  A* -- reading JSON --
    // routed straight through a corridor that is physically impassable; the
    // reactive LiDAR filter then correctly commanded zero velocity, and the
    // dog froze at (1.75,1.64) for 2659 frames.  true_x stayed pinned at
    // 1.7529 == 2.0529 - 0.30 (capsule radius against the phantom chair),
    // which is what finally gave the bug away.
    //
    // A prefix blacklist can never be complete -- the next stale prefix
    // reintroduces the bug silently.  So invert the rule: the runtime-loaded scene config is
    // the single source of truth, therefore ANY AStaticMeshActor present in the
    // level is stale BY DEFINITION.  The only whitelisted exception is a ground
    // slab (flat, top at/below z=5 cm, at least 3 m across) so the room keeps a
    // visible floor; a horizontal LiDAR ray can never hit it anyway.
    // ======================================================================
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), AllActors);
    int32 Destroyed = 0, KeptSlabs = 0;
    for (AActor* A : AllActors)
    {
        if (!A || A == this) continue;
        bool bShouldDestroy = false;

        // (1) our own obstacles from a previous SpawnObstacleVisuals() call
        if (A->ActorHasTag(FName(TEXT("PuppyObs"))))
            bShouldDestroy = true;

        // (2) every level-authored static mesh collider (the actual fix)
        if (A->IsA(AStaticMeshActor::StaticClass()))
        {
            const FBox B = A->GetComponentsBoundingBox(false);
            const FVector Sz = B.IsValid ? B.GetSize() : FVector::ZeroVector;
            const bool bGroundSlab = (B.IsValid != 0) && (B.Max.Z <= 5.0f) &&
                                     (Sz.X >= 300.0f) && (Sz.Y >= 300.0f);
            if (bGroundSlab) { ++KeptSlabs; }
            else             { bShouldDestroy = true; }
        }

        // (3) legacy label prefixes, for stale leftovers that are not static
        //     mesh actors (BSP brushes, patrol markers, pedestrian placeholders)
#if WITH_EDITOR
        const FString Label = A->GetActorLabel();
        if (Label.StartsWith(TEXT("OBS_"))  || Label.StartsWith(TEXT("Obs_")) ||
            Label.StartsWith(TEXT("FURN_")) || Label.StartsWith(TEXT("PED_")) ||
            Label.StartsWith(TEXT("PATROL_")))
            bShouldDestroy = true;
#endif

        if (bShouldDestroy)
        {
            A->Destroy();
            ++Destroyed;
        }
    }
    UE_LOG(LogTemp, Warning,
           TEXT("[PuppyRobotPawn] R32 purge: destroyed %d stale level colliders, kept %d ground slab(s)"),
           Destroyed, KeptSlabs);

    // Load assets at runtime (ConstructorHelpers only works in constructors)
    UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    UMaterialInterface* BaseMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (!CubeMesh) return;

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    int32 Spawned = 0;
    for (int32 i = 0; i < GHomeObstacles.Num(); ++i)
    {
        const FHomeObstacle& O = GHomeObstacles[i];
        FString NameStr(O.Name);
        NameStr.TrimEndInline();
        bool bIsWall = NameStr.Contains(TEXT("wall"));
        bool bIsCounter = NameStr.Contains(TEXT("counter"));

        float Cx = (O.Xmin + O.Xmax) * 0.5f;
        float Cy = (O.Ymin + O.Ymax) * 0.5f;
        float Sizex = FMath::Max(1.0f, O.Xmax - O.Xmin);
        float Sizey = FMath::Max(1.0f, O.Ymax - O.Ymin);

        float Sizez;
        FColor Color;
        // Colors brightened (2026-08-13) so furniture reads clearly under the
        // new lighting instead of collapsing to near-black.  Hue still encodes
        // the furniture type (blue=sofa, purple=bed, steel=fridge, etc.).
        if (bIsWall) { Sizez = 280.0f; Color = FColor(230, 226, 218); }        // warm white wall
        else if (bIsCounter) { Sizez = 90.0f; Color = FColor(185, 155, 120); } // light wood
        else if (NameStr.Contains(TEXT("bed"))) { Sizez = 45.0f; Color = FColor(165, 135, 190); } // light purple
        else if (NameStr.Contains(TEXT("sofa"))) { Sizez = 70.0f; Color = FColor(125, 165, 215); } // light blue
        else if (NameStr.Contains(TEXT("coffee_table"))) { Sizez = 45.0f; Color = FColor(180, 135, 95); }
        else if (NameStr.Contains(TEXT("dining_table"))) { Sizez = 75.0f; Color = FColor(195, 155, 110); }
        else if (NameStr.Contains(TEXT("chair"))) { Sizez = 80.0f; Color = FColor(175, 135, 100); }
        else if (NameStr.Contains(TEXT("fridge"))) { Sizez = 170.0f; Color = FColor(228, 228, 238); } // light steel
        else if (NameStr.Contains(TEXT("shower")) || NameStr.Contains(TEXT("toilet")) || NameStr.Contains(TEXT("sink"))) { Sizez = 80.0f; Color = FColor(208, 228, 238); }
        else if (NameStr.Contains(TEXT("bookshelf"))) { Sizez = 180.0f; Color = FColor(185, 135, 90); }
        else if (NameStr.Contains(TEXT("wardrobe"))) { Sizez = 200.0f; Color = FColor(175, 130, 100); }
        else if (NameStr.Contains(TEXT("desk"))) { Sizez = 75.0f; Color = FColor(195, 145, 105); }
        else if (NameStr.Contains(TEXT("tv"))) { Sizez = 50.0f; Color = FColor(78, 82, 98); }    // dark slate, still visible
        else if (NameStr.Contains(TEXT("nightstand"))) { Sizez = 50.0f; Color = FColor(180, 140, 105); }
        else if (NameStr.Contains(TEXT("island"))) { Sizez = 90.0f; Color = FColor(195, 173, 140); }
        else { Sizez = 60.0f; Color = FColor(195, 195, 188); }

        float Zcenter = Sizez * 0.5f;
        FVector Location(Cx, Cy, Zcenter);
        FRotator Rotation(0.f, 0.f, 0.f);

        // ===================================================================
        // R28 ROOT-CAUSE FIX (2026-08-02) -- "every obstacle sat on the origin"
        // -------------------------------------------------------------------
        // The previous code did:
        //     SpawnActor<AActor>(AActor::StaticClass(), Location, Rotation)
        //     ... NewObject<UStaticMeshComponent>() ... SetRootComponent(Mesh)
        //
        // A *bare* AActor has NO RootComponent.  AActor stores its transform
        // *inside* the root component, so SpawnActor() had nowhere to put
        // `Location` and silently discarded it (GetActorLocation() on a
        // root-less actor returns FVector::ZeroVector).  When MeshComp was then
        // installed as the root, it carried an identity relative transform, so
        // EVERY one of the 18 obstacle boxes materialised at world (0,0,0),
        // stacked on top of each other, with the correct SIZE but the wrong
        // PLACE (Zcenter was lost too, so they straddled the floor plane).
        //
        // Consequence: the UE LiDAR ray-cast the *origin pile*, while the C++
        // navigation stack ray-cast the *correct* occupancy grid.  The two were
        // geometrically unrelated: only 1 of 72 beams agreed with the real map,
        // whereas 65 of 72 agreed with an "all boxes at the origin" model.
        // AMCL therefore converged to the pose that best explains the *pile*
        // -- systematically ~0.66 m north-west of ground truth -- and no rigid
        // transform could ever reconcile the scan with the map.
        //
        // The fix: install the root component and REGISTER it FIRST, then apply
        // the transform through the (now valid) root.  Mobility must be Movable
        // because we move the component after registration.
        // ===================================================================
        AActor* ObsActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
        if (!ObsActor) continue;
#if WITH_EDITOR
        ObsActor->SetActorLabel(FString("Obs_") + NameStr);
#endif
        ObsActor->Tags.Add(FName(TEXT("PuppyObs")));

        UStaticMeshComponent* MeshComp = NewObject<UStaticMeshComponent>(ObsActor);
        // Movable, NOT Static: a Static component may not be moved after it has
        // been registered, and we set the transform below (post-registration).
        MeshComp->SetMobility(EComponentMobility::Movable);
        ObsActor->SetRootComponent(MeshComp);
        ObsActor->AddInstanceComponent(MeshComp);
        MeshComp->SetStaticMesh(CubeMesh);
        MeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        MeshComp->SetCollisionProfileName(TEXT("BlockAll"));
        MeshComp->SetGenerateOverlapEvents(true);

        // Create colored MID from base material
        if (BaseMat)
        {
            UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(BaseMat, MeshComp);
            if (MID) {
                MID->SetVectorParameterValue(TEXT("Color"), FLinearColor(Color));
                MeshComp->SetMaterial(0, MID);
            }
        }

        // Register BEFORE transforming: the root now exists, so the transform
        // actually lands on the actor instead of being dropped on the floor.
        MeshComp->RegisterComponent();
        ObsActor->SetActorLocation(Location);
        ObsActor->SetActorRotation(Rotation);
        MeshComp->SetWorldScale3D(FVector(Sizex / 100.0f, Sizey / 100.0f, Sizez / 100.0f));
        MeshComp->UpdateBounds();

        // Post-condition check: if the actor is still sitting on the origin the
        // bug has regressed -- shout about it instead of silently mis-localising.
        const FVector Placed = ObsActor->GetActorLocation();
        if (FVector::DistSquared2D(Placed, Location) > 1.0f)
        {
            UE_LOG(LogTemp, Error,
                   TEXT("[PuppyRobotPawn] OBSTACLE-PLACEMENT-FAILED [%s]: wanted (%.1f,%.1f) got (%.1f,%.1f)"),
                   *NameStr, Location.X, Location.Y, Placed.X, Placed.Y);
        }
        else if (Spawned < 4)
        {
            UE_LOG(LogTemp, Warning,
                   TEXT("[PuppyRobotPawn] OBSTACLE-PLACED [%-16s] loc=(%.1f,%.1f,%.1f) size=(%.0f,%.0f,%.0f)cm"),
                   *NameStr, Placed.X, Placed.Y, Placed.Z, Sizex, Sizey, Sizez);
        }
        ++Spawned;
    }

    // --- CEILING PLANE (2026-08-13) ---
    // The room has four walls (280 cm tall) but no roof, so the UE viewport
    // shows blue sky above every wall edge and distant furniture appears to
    // "float" in empty space.  Add a thin white ceiling that closes the volume.
    {
        AActor* CeilActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
        if (CeilActor)
        {
            UStaticMeshComponent* CeilMesh = NewObject<UStaticMeshComponent>(CeilActor);
            CeilMesh->SetMobility(EComponentMobility::Static);
            CeilActor->SetRootComponent(CeilMesh);
            CeilActor->AddInstanceComponent(CeilMesh);
            CeilMesh->SetStaticMesh(CubeMesh);
            CeilMesh->SetWorldScale3D(FVector(1400.0f / 100.0f, 1200.0f / 100.0f, 2.0f / 100.0f)); // overhang walls by 2.5 m each side
            CeilMesh->RegisterComponent();
            CeilActor->SetActorLocation(FVector(0.0f, 0.0f, 281.0f)); // 1 cm atop 280-cm walls
            CeilMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); // invisible to LiDAR/physics
            if (BaseMat)
            {
                UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(BaseMat, CeilMesh);
                if (MID) { MID->SetVectorParameterValue(TEXT("Color"), FLinearColor(0.95f, 0.95f, 0.92f)); CeilMesh->SetMaterial(0, MID); }
            }
#if WITH_EDITOR
            CeilActor->SetActorLabel(TEXT("Ceiling"));
#endif
            ++Spawned;
        }
    }

    // --- LIGHTING (2026-08-13) ---
    // The floor is ~60 % pitch-black because only default ambient reaches it.
    // Add a directional "sun" from the NW so every surface gets direct light,
    // plus a soft fill from the opposite side to lift shadow regions.
    {
        FActorSpawnParameters LP;
        LP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

        // Main directional light (afternoon sun from NW)
        ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>(
            ADirectionalLight::StaticClass(),
            FVector(-300.0f, -300.0f, 500.0f),
            FRotator(-50.0f, -120.0f, 0.0f), LP);
        if (Sun)
        {
            Sun->GetLightComponent()->SetIntensity(8.0f);
            Sun->GetLightComponent()->SetLightColor(FLinearColor(1.0f, 0.97f, 0.92f));
#if WITH_EDITOR
            Sun->SetActorLabel(TEXT("QA_Sun"));
#endif
        }

        // Fill light (soft, from SE) to brighten shadow areas
        ADirectionalLight* Fill = World->SpawnActor<ADirectionalLight>(
            ADirectionalLight::StaticClass(),
            FVector(200.0f, 200.0f, 300.0f),
            FRotator(-30.0f, 60.0f, 0.0f), LP);
        if (Fill)
        {
            Fill->GetLightComponent()->SetIntensity(2.5f);
            Fill->GetLightComponent()->SetLightColor(FLinearColor(0.85f, 0.88f, 1.0f));
#if WITH_EDITOR
            Fill->SetActorLabel(TEXT("QA_Fill"));
#endif
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] SpawnObstacleVisuals: spawned %d obstacle actors (R28: root-component fix + ceiling + lighting)"), Spawned);

    // R32: the nav stack is only as correct as the geometry it plans on -- prove
    // that UE and scene_home.json describe the same room before trusting a run.
    AuditWorldGeometry(World, this);
}




// ===== R19i BUG#4 姝ｇ‘淇: 鍦烘櫙纭竟鐣?= Costmap 鑷敱鍖鸿竟缂?- 2cm =====

//   R19g: 5-cell margin = 0.5m LETHAL = 鎶婃埧闂村唴閮ㄥ彉澧?鏈哄櫒浜虹粫澶栧湀璧板埌涓嶄簡鐩爣!

//   R19i: 1-cell margin = 0.1m 鏈€澶?鏍?LETHAL 鎵嶆纭?= 鍙槸鍦板浘澶栬竟缂樺睆钄?

//     Costmap 灏哄: 100脳80 cells, 0.1m/cell

//       x: -5.0 鈫?+5.0,  lethal 1鏍?= x鈭圼-5.0,-4.9] 鈭?[4.9,5.0] 鈫?FREE x 鈭?[-4.9, 4.9]

//       y: -4.0 鈫?+4.0,  lethal 1鏍?= y鈭圼-4.0,-3.9] 鈭?[3.9,4.0] 鈫?FREE y 鈭?[-3.9, 3.9]

//     鈫?UE clamp: FREE鍖哄煙杈圭紭 - 2cm 瑁曞害 = 鏈哄櫒浜烘案涓嶇 lethal 杈圭紭

static constexpr float HOME_BOUND_XMIN_CM = -440.0f;   // -4.40 m (room x[-4.5,4.5], v6.0)

static constexpr float HOME_BOUND_XMAX_CM =  440.0f;   //  4.40 m

static constexpr float HOME_BOUND_YMIN_CM = -340.0f;   // -3.40 m (room y[-3.5,3.5], v6.0)

static constexpr float HOME_BOUND_YMAX_CM =  340.0f;   //  3.40 m



// Robot radius for BBOX collision inflation.

// scene_home.json "radius_planning" = 0.35m was used for A* costmap inflation

// but UE-side physics capsule is only 30cm radius, and doorways at y=0/y=2m

// walls are only 1.0m wide (between seg2 xmax=0 and seg3 xmin=100cm).

// At 35cm inflation the door shrinks to 30cm 鈫?impassable 鈫?99% frames collide

// (R15 had 5938 collisions in 6000 frames!). Using 10cm here acts like a

// "tight fit" clearance: still stops true wall penetrations while leaving

// enough room for the planner to squeeze the 30cm capsule through doorways.

static constexpr float ROBOT_RADIUS_CM = 20.0f;  // R25a (2026-08-13): PHYSICAL body radius of the
                                        // quadruped, NOT the planner's safety margin.
                                        // History: 5cm (too small - dog grazed furniture, its true
                                        // pose sat inside the A* inflated grid -> start embedded);
                                        // 30cm in R24 (WRONG - equal to radius_planning, so the
                                        // rigid-body test rejected almost every step: collisions
                                        // 552 -> 3005, stuck 75.6% -> 95.1%, targets 3 -> 0,
                                        // only 175/3600 frames actually moved).
                                        // Correct invariant: physical_radius (0.20m)
                                        //                  < radius_planning (0.30m),
                                        // leaving a 10cm margin for control + localization error.



static bool IsInsideHomeObstacle(float Xcm, float Ycm, const FHomeObstacle** OutHit = nullptr)

{

    for (int i = 0; i < GHomeObstacles.Num(); ++i)

    {

        const FHomeObstacle& O = GHomeObstacles[i];

        // R33 DISC FIX (2026-08-14): exact point-to-AABB Euclidean distance.
        //
        // The body is a CAPSULE -- a DISC of radius ROBOT_RADIUS_CM in plan view
        // (CollisionCapsule->SetCapsuleRadius(20.0f), line 136).  The forbidden
        // region for a disc centre is therefore the Minkowski sum of the AABB
        // with a DISC: the box grown by R with ROUNDED corners.
        //
        // The previous test grew the box by R on all four sides independently,
        // i.e. the Minkowski sum with a SQUARE.  That models a 40x40cm
        // axis-aligned square robot, and at a convex corner it reaches
        // R*sqrt(2) = 28.3cm instead of R = 20cm -- a 41% over-reach that
        // reports contact while the body is still 8.3cm clear of the geometry.
        //
        // Evidence (3600-frame r32 acceptance run, all 4 reported events):
        //   #1 wall_h_mid (1.69,0.61) true body gap = +6.9cm  -> false positive
        //   #2 wall_h_mid (0.11,0.61) true body gap = +6.9cm  -> false positive
        //   #4 wall_h_mid (1.68,0.60) true body gap = +6.9cm  -> false positive
        // All three sit at a door-frame convex corner, all three fall in the
        // (R, R*sqrt(2)] = (20.0, 28.3]cm band that only the square test flags.
        // Independent out-of-band check over the same trace (trace_val.csv x
        // scene_home.json, exact distance-to-box): min body clearance = 26.6cm,
        // i.e. the robot never came within 6.6cm of touching anything.
        //
        // This change is strictly a correction, not a relaxation: every centre
        // whose distance to the box is < R is still flagged.  Only centres that
        // are demonstrably NOT in contact are no longer reported.
        const float ClampedDx = FMath::Max3(O.Xmin - Xcm, 0.0f, Xcm - O.Xmax);
        const float ClampedDy = FMath::Max3(O.Ymin - Ycm, 0.0f, Ycm - O.Ymax);

        if (ClampedDx * ClampedDx + ClampedDy * ClampedDy
            < ROBOT_RADIUS_CM * ROBOT_RADIUS_CM)

        {

            if (OutHit) *OutHit = &O;

            return true;

        }

    }

    return false;

}


// R33 CONTACT TELEMETRY (2026-08-14)
// -----------------------------------------------------------------------------
// Signed clearance from the BODY SURFACE to the nearest obstacle, in cm.
//   > 0  the body is clear by that many cm
//  == 0  the body surface is exactly touching
//   < 0  the body is interpenetrating by that many cm
//
// Why this exists: CollisionCount is a RISING-EDGE counter gated by the
// bObstacleContactActive latch, so a run that grinds along a wall for hundreds
// of frames can still report a single "collision".  A rising-edge count alone
// therefore cannot substantiate the section-16 "collisions = 0" claim.  This
// function feeds two scalars that can not hide contact: the minimum clearance
// over the whole run, and the number of ticks spent interpenetrating.
static float BodySurfaceClearanceCm(float Xcm, float Ycm,
                                    const FHomeObstacle** OutNearest = nullptr)
{
    float BestDist = TNumericLimits<float>::Max();
    const FHomeObstacle* BestObs = nullptr;
    for (int i = 0; i < GHomeObstacles.Num(); ++i)
    {
        const FHomeObstacle& O = GHomeObstacles[i];
        const float dx = FMath::Max3(O.Xmin - Xcm, 0.0f, Xcm - O.Xmax);
        const float dy = FMath::Max3(O.Ymin - Ycm, 0.0f, Ycm - O.Ymax);
        const float d = FMath::Sqrt(dx * dx + dy * dy);
        if (d < BestDist) { BestDist = d; BestObs = &O; }
    }
    if (OutNearest) *OutNearest = BestObs;
    return BestDist - ROBOT_RADIUS_CM;
}


static void ClampToHomeBounds(FVector& Pos_Cm)

{

    Pos_Cm.X = FMath::Clamp(Pos_Cm.X, HOME_BOUND_XMIN_CM, HOME_BOUND_XMAX_CM);

    Pos_Cm.Y = FMath::Clamp(Pos_Cm.Y, HOME_BOUND_YMIN_CM, HOME_BOUND_YMAX_CM);

    // keep Z unchanged (robot height is irrelevant for 2D nav)

}



void APuppyRobotPawn::ApplyMovement(float DeltaTime)

{

    // =====================================================================

    //  GUARANTEED-MOVEMENT + UE OVERLAP-BASED COLLISION REVERT

    //  - Always move with bSweep=false 鈫?GUARANTEES robot never jams from

    //    tiny capsule overlaps at spawn (the original frozen-robot bug from

    //    R01鈥揜09 where even an empty room would produce zero displacement).

    //  - After moving, query the UE physics engine for overlapping colliders

    //    on the capsule. If overlap is detected with ANY world obstacle actor

    //    鈫?revert to the pre-move position. This correctly handles UE-side

    //    actor positions that don't exactly match the scene_home.json BBOX

    //    list (the manual-BBOX approach caused 3897 false-positive collisions

    //    in R17 because of ~5cm coordinate mismatches between UE placements

    //    and the hand-written JSON centimetre values).

    //  - Scene hard bounds clamp is still kept as a last-resort safety net

    //    so the robot never drifts outside the house entirely (the R14

    //    208m-outside-the-map flight).

    // =====================================================================

    const FVector LocBefore = GetActorLocation();

    const FVector Delta = CurrentVelocity * DeltaTime;

    const double  IntendedCm = Delta.Size2D();



    // ================================================================

    // R22 COLLISION FIX: 涓夊眰纰版挒妫€娴?

    //   Layer 1: 鎵嬪姩BBOX棰勬鏌?(IsInsideHomeObstacle) 鈥?鏈€鍙潬

    //   Layer 2: UE overlap妫€娴?(GetOverlappingActors) 鈥?琛ュ厖

    //   Layer 3: 鍦烘櫙杈圭晫閽冲埗 (ClampToHomeBounds) 鈥?鍏滃簳

    // 淇鍘嗗彶闂:

    //   - bSweep=false瀵艰嚧绌垮: 鐜板湪鐢˙BOX棰勬鏌ユ嫤鎴?

    //   - bOverlappedBefore閫冮€搁€昏緫瀵艰嚧鍦ㄩ殰纰嶇墿閲岀┛姊? 鐜板湪鍙厑璁歌繙绂婚殰纰嶇墿涓績鐨勭Щ鍔?

    //   - bNearBoundary璺宠繃纰版挒: 鐜板湪鍙湪瀹為檯琚獵lamp鏃惰烦杩? 涓嶅啀鍥犻潬杩戣竟鐣岃€岀鐢ㄧ鎾炴娴?

    // ================================================================



    // 1) Tentative no-sweep move

    SetActorLocation(LocBefore + Delta, /*bSweep=*/false, nullptr, ETeleportType::None);

    FVector LocAfter = GetActorLocation();



    // 2) 鍦烘櫙纭竟鐣岄挸鍒?(x鈭圼-4.88,4.88]m, y鈭圼-3.88,3.88]m)

    const FVector PreClamp = LocAfter;

    ClampToHomeBounds(LocAfter);

    bool bWasClamped = (PreClamp.X != LocAfter.X || PreClamp.Y != LocAfter.Y);

    if (bWasClamped)

    {

        SetActorLocation(LocAfter, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

        static double LastBoundClampLog = 0.0;

        const double NowClamp = FPlatformTime::Seconds();

        if (NowClamp - LastBoundClampLog > 3.0)

        {

            LastBoundClampLog = NowClamp;

            UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] BOUND-CLAMP: (%.0f,%.0f)->(%.0f,%.0f)cm Frame=%d"),

                   PreClamp.X, PreClamp.Y, LocAfter.X, LocAfter.Y, FrameCount);

        }

    }



    // ================================================================

    // Layer 1: 鎵嬪姩BBOX纰版挒妫€鏌?(鏈€鍙潬)

    // 妫€鏌ョЩ鍔ㄥ墠鍚庣殑浣嶇疆鏄惁鍦ㄩ殰纰嶇墿BBOX鍐? 杩欐槸闃叉绌垮鐨勬牳蹇冩満鍒躲€?

    // IsInsideHomeObstacle宸插寘鍚?0涓殰纰嶇墿(瀹跺叿+澧欏), 姣忎釜閮借啫鑳€浜哛OBOT_RADIUS_CM銆?

    // ================================================================

    const FHomeObstacle* ObsAfter = nullptr;

    const FHomeObstacle* ObsBefore = nullptr;

    const bool bInObstacleAfter  = IsInsideHomeObstacle(LocAfter.X,  LocAfter.Y,  &ObsAfter);

    const bool bInObstacleBefore = IsInsideHomeObstacle(LocBefore.X, LocBefore.Y, &ObsBefore);



    bool bBboxCollision = false;
    bool bContactThisFrame = false;

    if (bInObstacleAfter && !bInObstacleBefore)
    {
        // R23 SLIDE FIX (2026-08-13): per-axis sliding instead of whole-move revert.
        // BUG: any contact reverted the ENTIRE delta, so a robot pressed against a
        // furniture face could never slide along it -> permanent wedge.  Observed in
        // the GPU smoke test: 16206 collisions, 99.9% stuck frames, the dog rammed
        // coffee_table at 2 m/s for 387 s and never reached patrol target #1.
        // FIX: retry the move on each axis separately (mirrors the standalone
        // simulator's x_safe / y_safe logic) and only revert fully when both axes
        // are blocked.  This lets the robot graze furniture and slide free.
        const FVector SlideX(LocAfter.X,  LocBefore.Y, LocAfter.Z);
        const FVector SlideY(LocBefore.X, LocAfter.Y,  LocAfter.Z);
        const bool bSlideXFree = !IsInsideHomeObstacle(SlideX.X, SlideX.Y);
        const bool bSlideYFree = !IsInsideHomeObstacle(SlideY.X, SlideY.Y);

        FVector ResolvedLoc = LocBefore;
        const TCHAR* ResolveMode = TEXT("REVERT");
        bool bSlid = false;
        if (bSlideXFree && bSlideYFree)
        {
            const double GainX = FMath::Abs(SlideX.X - LocBefore.X);
            const double GainY = FMath::Abs(SlideY.Y - LocBefore.Y);
            if (GainX >= GainY) { ResolvedLoc = SlideX; ResolveMode = TEXT("SLIDE-X"); }
            else                { ResolvedLoc = SlideY; ResolveMode = TEXT("SLIDE-Y"); }
            bSlid = true;
        }
        else if (bSlideXFree) { ResolvedLoc = SlideX; ResolveMode = TEXT("SLIDE-X"); bSlid = true; }
        else if (bSlideYFree) { ResolvedLoc = SlideY; ResolveMode = TEXT("SLIDE-Y"); bSlid = true; }

        bBboxCollision = true;
        bContactThisFrame = true;

        SetActorLocation(ResolvedLoc, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

        const bool bNewContact = !bObstacleContactActive;
        bObstacleContactActive = true;
        if (bNewContact)
        {
            ++CollisionCount;
        }

        if (TcpComp != nullptr && ObsAfter != nullptr && bNewContact)
        {
            const float Bx = (ObsAfter->Xmin + ObsAfter->Xmax) * 0.5f;
            const float By = (ObsAfter->Ymin + ObsAfter->Ymax) * 0.5f;
            TcpComp->SendCollision(CollisionCount, 0.01 * Bx, 0.01 * By);
        }

        // Throttle: the old code logged every frame and produced 16k log lines.
        static double LastBboxLogT = 0.0;
        const double NowBboxT = FPlatformTime::Seconds();
        if (NowBboxT - LastBboxLogT > 2.0)
        {
            LastBboxLogT = NowBboxT;
            UE_LOG(LogTemp, Warning,
                   TEXT("[PuppyRobotPawn] BBOX-COLLIDE #%d: obs=[%s] (%.0f,%.0f)->(%.0f,%.0f) resolve=%s slid=%d"),
                   CollisionCount,
                   ObsAfter ? ObsAfter->Name : TEXT("?"),
                   LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y,
                   ResolveMode, bSlid ? 1 : 0);
        }

        LocAfter = ResolvedLoc;

    }

    else if (bInObstacleAfter && bInObstacleBefore)

    {

        // 绉诲姩鍓嶅悗閮藉湪闅滅鐗╁唴 鈫?宸插祵鍏? 鍙厑璁歌繙绂婚殰纰嶇墿涓績鐨勭Щ鍔?

        // (淇R22鏍稿績bug: 鍘熼€昏緫鍏佽浠绘剰鏂瑰悜绉诲姩鈫掓満鍣ㄤ汉鍦ㄩ殰纰嶇墿閲岀┛姊?

        if (ObsAfter != nullptr && ObsBefore != nullptr)

        {

            const float ObsCx = (ObsAfter->Xmin + ObsAfter->Xmax) * 0.5f;

            const float ObsCy = (ObsAfter->Ymin + ObsAfter->Ymax) * 0.5f;

            const float DistBefore = FVector::Dist2D(LocBefore, FVector(ObsCx, ObsCy, 0.0f));

            const float DistAfter  = FVector::Dist2D(LocAfter,  FVector(ObsCx, ObsCy, 0.0f));

            if (DistAfter < DistBefore)

            {

                // 绉诲姩鍚庢洿闈犺繎闅滅鐗╀腑蹇?鈫?鍦ㄥ線閲岄捇, 鍥為€€!

                bBboxCollision = true;

                SetActorLocation(LocBefore, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

                if (!bObstacleContactActive)
                {
                    ++CollisionCount;
                }

                UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] BBOX-ESCAPE-BLOCK #%d: obs=[%s] moving DEEPER (%.0f,%.0f)鈫?%.0f,%.0f) dist %.1f鈫?.1f 鈫?REVERT"),

                       CollisionCount, ObsAfter->Name,

                       LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y,

                       DistBefore, DistAfter);

                LocAfter = LocBefore;

            }

            else

            {

                // 绉诲姩鍚庢洿杩滅闅滅鐗╀腑蹇?鈫?鍦ㄥ線澶栭€? 鍏佽

                static double LastEscapeLog = 0.0;

                const double NowT = FPlatformTime::Seconds();

                if (NowT - LastEscapeLog > 3.0) {

                    LastEscapeLog = NowT;

                    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] BBOX-ESCAPE-OK: obs=[%s] moving OUT dist %.1f鈫?.1f, allow"),

                           ObsAfter->Name, DistBefore, DistAfter);

                }

            }

        }

    }



    // ================================================================

    // Layer 2: UE overlap妫€娴?(琛ュ厖BBOX妫€鏌ョ殑閬楁紡)

    // 浠呭綋BBOX妫€鏌ユ湭瑙﹀彂纰版挒鏃舵墠鎵ц, 閬垮厤閲嶅璁℃暟

    // ================================================================

    if (!bBboxCollision)

    {

        bool bHasBlockerOverlap = false;

        AActor* BlockerActor = nullptr;

        if (CollisionCapsule != nullptr)

        {

            TArray<AActor*> Overlaps;

            CollisionCapsule->GetOverlappingActors(Overlaps);

            for (AActor* A : Overlaps)

            {

                if (A == nullptr || A == this) continue;

                const FString N = A->GetName();

                if (N.StartsWith(TEXT("PlayerStart")) ||

                    N.StartsWith(TEXT("PuppyPedestrian")) ||

                    N.StartsWith(TEXT("BP_PuppyPedestrian")) ||

                    N.StartsWith(TEXT("GameMode")) ||

                    N.StartsWith(TEXT("GameState")) ||

                    N.Contains(TEXT("SkyAtmosphere")) ||

                    N.Contains(TEXT("DirectionalLight")) ||

                    N.Contains(TEXT("SkyLight")) ||

                    N.Contains(TEXT("HeightFog")) ||

                    N.Contains(TEXT("Floor")) ||

                    N.Contains(TEXT("Ground")))

                {

                    continue;

                }

                bHasBlockerOverlap = true;
                bContactThisFrame = true;

                BlockerActor = A;

                break;

            }

        }



        bool bOverlappedBefore = false;

        if (bHasBlockerOverlap && CollisionCapsule != nullptr)

        {

            SetActorLocation(LocBefore, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

            TArray<AActor*> OverlapsBefore;

            CollisionCapsule->GetOverlappingActors(OverlapsBefore);

            for (AActor* A : OverlapsBefore)

            {

                if (A == nullptr || A == this) continue;

                const FString N = A->GetName();

                if (N.StartsWith(TEXT("PlayerStart")) || N.StartsWith(TEXT("PuppyPedestrian")) ||

                    N.StartsWith(TEXT("BP_PuppyPedestrian")) ||

                    N.Contains(TEXT("SkyAtmosphere")) || N.Contains(TEXT("DirectionalLight")) ||

                    N.Contains(TEXT("SkyLight")) || N.Contains(TEXT("HeightFog")) ||

                    N.Contains(TEXT("Floor")) || N.Contains(TEXT("Ground")) ||

                    N.StartsWith(TEXT("GameMode")) || N.StartsWith(TEXT("GameState")))

                {

                    continue;

                }

                bOverlappedBefore = true;

                break;

            }

            SetActorLocation(LocAfter, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

        }



        // R22 FIX: 绉婚櫎 !bWasClamped 鏉′欢 鈥?鍙湪瀹為檯琚竟鐣孋lamp鏃惰烦杩? 涓嶅啀鍥犻潬杩戣竟鐣岃烦杩?

        // R22 FIX: bOverlappedBefore鏃朵笉鍐嶆棤鏉′欢鍏佽, 鏀逛负妫€鏌ユ槸鍚﹀湪杩滅闅滅鐗?

        if (bHasBlockerOverlap && !bOverlappedBefore)

        {

            // 鏂扮鎾?(涔嬪墠涓嶅湪overlap, 鐜板湪鍦? 鈫?鍥為€€

            // 渚嬪: 濡傛灉琚獵lampToHomeBounds閽冲埗浜? 涓嶅洖閫€ (Clamp宸茬粡淇濊瘉鍦ㄨ竟鐣屽唴)

            if (!bWasClamped)

            {

                SetActorLocation(LocBefore, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

                const bool bNewContact = !bObstacleContactActive;
                if (bNewContact)
                {
                    ++CollisionCount;
                }
                bObstacleContactActive = true;

                if (TcpComp != nullptr && bNewContact)

                {

                    const FVector BLoc = BlockerActor ? BlockerActor->GetActorLocation() : FVector(LocBefore + LocAfter) * 0.5f;

                    TcpComp->SendCollision(CollisionCount, 0.01 * BLoc.X, 0.01 * BLoc.Y);

                }

                UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] OVERLAP-COLLIDE #%d: actor=[%s] fresh-overlap (%.0f,%.0f)鈫?%.0f,%.0f) 鈫?REVERT"),

                       CollisionCount,

                       BlockerActor ? *BlockerActor->GetName() : TEXT("unknown"),

                       LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y);

                LocAfter = LocBefore;

            }

        }

        else if (bHasBlockerOverlap && bOverlappedBefore)

        {

            // 宸插祵鍏? 鍙褰曟棩蹇? 涓嶉樆姝㈢Щ鍔?(BBOX灞傚凡澶勭悊鏂瑰悜妫€鏌?

            static double LastStuckWarn = 0.0;

            const double NowT = FPlatformTime::Seconds();

            if (NowT - LastStuckWarn > 3.0) {

                LastStuckWarn = NowT;

                UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] OVERLAP-STUCK: pos=(%.0f,%.0f) inside [%s] (BBox layer handles direction)"),

                       LocAfter.X, LocAfter.Y,

                       BlockerActor ? *BlockerActor->GetName() : TEXT("?"));

            }

        }

    }



    const double ActualMovedCm = FVector::Dist(LocBefore, LocAfter);



    // Throttled ~3s diagnostic so we can see the robot is actually moving

    static double LastLog = 0.0;

    const double Now = FPlatformTime::Seconds();

    if (Now - LastLog > 3.0) {

        LastLog = Now;

        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] MOVE-3s: Intended=%.2fcm Actual=%.2fcm | Before=(%.1f,%.1f) After=(%.1f,%.1f) | Vel=(%.1f,%.1f)cm/s dt=%.4fs Frame=%d"),

               IntendedCm, ActualMovedCm,

               LocBefore.X, LocBefore.Y,

               LocAfter.X, LocAfter.Y,

               CurrentVelocity.X, CurrentVelocity.Y, DeltaTime, FrameCount);

    }



    // ================================================================
    // R33 CONTACT TELEMETRY -- unfalsifiable evidence for "collisions = 0".
    // Measured on the FINAL pose of this tick (post slide/revert resolution),
    // which is the pose the body actually occupied.  ResolvedViolationTicks
    // counts ticks where resolution failed to keep the body clear; that number
    // must be 0 for the section-16 claim to hold, independently of the
    // rising-edge CollisionCount.
    // ================================================================
    {
        static float  MinClearanceCm         = TNumericLimits<float>::Max();
        static float  MinAtX                 = 0.0f;
        static float  MinAtY                 = 0.0f;
        static FString MinAtObs              = TEXT("-");
        static int32  ResolvedViolationTicks = 0;
        static int32  AttemptedViolationTicks = 0;
        static int32  TelemetryTicks         = 0;
        static double LastTelemetryLogT      = 0.0;

        const FVector FinalLoc = GetActorLocation();
        const FHomeObstacle* NearestObs = nullptr;
        const float ClearanceCm =
            BodySurfaceClearanceCm(FinalLoc.X, FinalLoc.Y, &NearestObs);

        ++TelemetryTicks;
        if (ClearanceCm < 0.0f)   { ++ResolvedViolationTicks; }
        if (bInObstacleAfter)     { ++AttemptedViolationTicks; }
        if (ClearanceCm < MinClearanceCm)
        {
            MinClearanceCm = ClearanceCm;
            MinAtX = FinalLoc.X;
            MinAtY = FinalLoc.Y;
            MinAtObs = NearestObs ? FString(NearestObs->Name) : TEXT("-");
        }

        const double NowTelemetryT = FPlatformTime::Seconds();
        if (NowTelemetryT - LastTelemetryLogT > 10.0)
        {
            LastTelemetryLogT = NowTelemetryT;
            UE_LOG(LogTemp, Warning,
                   TEXT("[CONTACT-TELEMETRY] min_body_clearance=%.2fcm at (%.0f,%.0f) near=[%s]")
                   TEXT(" | resolved_violation_ticks=%d attempted=%d of %d | edge_events=%d"),
                   MinClearanceCm, MinAtX, MinAtY, *MinAtObs,
                   ResolvedViolationTicks, AttemptedViolationTicks,
                   TelemetryTicks, CollisionCount);
        }
    }

    if (!bContactThisFrame)
    {
        bObstacleContactActive = false;
    }

    // Rotate around Z (yaw).  CurrentAngularVelocity is in deg/s.

    const float YawDelta = CurrentAngularVelocity * DeltaTime;

    AddActorWorldRotation(FRotator(0.0f, YawDelta, 0.0f), false);



    // R21 FIX: 杈圭晫鍗℃妫€娴?鈥?鍦ˋpplyMovement鏈熬妫€鏌ユ槸鍚︽寔缁湪杈圭晫瑙掕惤

    TryBoundaryEscapeTeleport();

}



bool APuppyRobotPawn::TryBoundaryEscapeTeleport()

{

    // R21 FIX: 褰撴満鍣ㄤ汉鍦ㄨ竟鐣岃钀藉崱姝昏秴杩?0绉? 鐩存帴浼犻€佸埌鍦烘櫙涓績瀹夊叏鍖恒€?

    //   鏍瑰洜: 杈圭晫瑙掕惤(-4.88,-3.88)鐨凙MCL鍙戞暎+瀵艰埅閿欒+鐗╃悊纰版挒鍙犲姞,

    //   鍗充娇鑴卞洶閫熷害姝ｇ‘(1.57,1.24 m/s)涔熸棤娉曞彲闈犺劚绂汇€?

    //   淇: 浣滀负缁堟瀬淇濋櫓, 鍗℃20绉掑悗浼犻€佽嚦(0,0)涓績(宸茬煡鑷敱鍖?銆?

    const FVector Loc = GetActorLocation();

    const double NowT = FPlatformTime::Seconds();



    // 鍒ゅ畾鏄惁鍦ㄨ竟鐣岃钀?璺濊竟鐣?0.5m)

    const bool atCorner = (FMath::Abs(Loc.X) > HOME_BOUND_XMAX_CM - 50.0f &&

                           FMath::Abs(Loc.Y) > FMath::Abs(HOME_BOUND_YMAX_CM) - 50.0f);

    // 涔熷垽瀹氭槸鍚﹀湪杈圭晫(鍗曡酱璐村)

    const bool atBoundary = (FMath::Abs(Loc.X) > HOME_BOUND_XMAX_CM - 30.0f ||

                              FMath::Abs(Loc.Y) > FMath::Abs(HOME_BOUND_YMAX_CM) - 30.0f);



    if (atCorner || atBoundary)

    {

        if (!bBoundaryStuckActive)

        {

            bBoundaryStuckActive = true;

            BoundaryStuckStart = NowT;

        }

        else if (NowT - BoundaryStuckStart > 20.0)

        {

            // 鍗℃瓒呰繃20绉? 浼犻€佸埌涓績瀹夊叏鍖?

            // 鐩爣: 璺濊竟鐣屾渶杩滅殑鐐?= (0, 0)

            const FVector SafeLoc(0.0f, 0.0f, Loc.Z);

            SetActorLocation(SafeLoc, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

            UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] BOUNDARY-TELEPORT: stuck %.1fs at (%.0f,%.0f) 鈫?teleport to (0,0) Frame=%d"),

                   NowT - BoundaryStuckStart, Loc.X, Loc.Y, FrameCount);

            bBoundaryStuckActive = false;

            BoundaryStuckStart = 0.0;

            return true;

        }

    }

    else

    {

        // 绂诲紑杈圭晫, 閲嶇疆璁℃椂

        if (bBoundaryStuckActive)

        {

            bBoundaryStuckActive = false;

            BoundaryStuckStart = 0.0;

        }

    }

    return false;

}


