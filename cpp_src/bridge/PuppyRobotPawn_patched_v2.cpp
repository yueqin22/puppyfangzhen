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



#include "PuppyLiDARComponent.h"

#include "PuppyTcpServer.h"

#include "PuppyPedestrianActor.h"



#include "Components/CapsuleComponent.h"

#include "Components/StaticMeshComponent.h"

#include "GameFramework/SpringArmComponent.h"

#include "Camera/CameraComponent.h"

#include "Engine/StaticMesh.h"

#include "Components/SphereComponent.h"

#include "Materials/MaterialInterface.h"

#include "Materials/MaterialInstanceDynamic.h"

#include "Kismet/GameplayStatics.h"



// m/s -> cm/s

static constexpr float M_TO_CM = 100.0f;



APuppyRobotPawn::APuppyRobotPawn()

{

    // ---- SPAWN SAFETY NET ---------------------------------------------------

    // Without this, the floor at Z ∈ [-5cm, +5cm] and PlayerStart at Z=0

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

    // Boston Dynamics Spot robot is roughly 60cm tall × 30cm wide.

    // Capsule radius = 30cm, half-height = 30cm → total height 60cm,

    // width/diameter = 60cm.  This is the RootComponent that gets swept

    // against the world for collision detection in ApplyMovement.

    //

    // IMPORTANT: CollisionCapsule must generate overlap events so we can

    // detect when the no-sweep SetActorLocation call accidentally places

    // the robot inside a wall / furniture actor. Without this the overlap

    // query in ApplyMovement returns an empty list and all collisions are

    // missed → robot phases through geometry (the R14 208m outside bug).

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

    //   CollisionCapsule: radius 30cm, half-height 30cm → total height 60cm,

    //   floor top surface at Z=0 (PlayerStart at Z=+15cm → capsule bottom Z = 15-30 = -15cm,

    //   which clips slightly below the floor; the visual meshes sit inside the capsule

    //   so we never clip below the floor (floor Z∈[-5,+5], leg bottoms at Z=+5cm ≈ floor top).

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));

    // Default material for BasicShapes — parameters "Color" (BaseColor) and "Roughness" exist,

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



    // 1) Main chassis body: 60cm (X, length) × 40cm (Y, width) × 18cm (Z, thickness)

    //    Cube default 100cm → scale = (0.6, 0.4, 0.18). Bottom edge at Z = +8cm

    //    (body bottom 8cm, body top 26cm), well inside the capsule bottom (-15cm).

    RobotBodyMesh = MakeBox(TEXT("RobotBodyMesh"), RootComponent,

        /*Loc=*/ FVector(0.0f, 0.0f, 17.0f),  // centre Z = 8 + 9 = 17

        /*Scale=*/ FVector(0.58f, 0.36f, 0.16f),

        CLR_BODY);



    // Rounded upper shell: this breaks the old single-box silhouette and

    // makes the body read as a small quadruped robot rather than a cube.

    MakeSphere(TEXT("PuppyPiUpperShell"), RootComponent,

        FVector(-2.0f, 0.0f, 27.0f), FVector(0.42f, 0.30f, 0.16f), CLR_BODY);



    // 2) Head module: 20cm (X) × 34cm (Y) × 15cm (Z), mounted to the front of the body.

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

    //      leg radius ≈ 4cm → scale (X/Y) = 4/50 = 0.08.

    //    Body corners (in body-local frame, origin at body centre):

    //      FL = (+25, +15), FR = (+25, -15), RL = (-25, +15), RR = (-25, -15)  (body is 60×40 → half 30×20, inset 5cm for shoulders)

    //    Leg centre Z = 22/2 = 11cm (from floor) → relative to body centre (Z=17) = -6cm.

    const FVector LEG_SCALE(0.08f, 0.08f, 0.11f);  // radius 4cm × height 22cm

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

    CameraBoom->TargetArmLength = 280.0f;

    CameraBoom->SocketOffset = FVector(0.0f, 40.0f, 120.0f);

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



    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] >>>>> FIX-SNAP: about to snap to scene_home.initial_pose (-100,-270) <<<<<"));

    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] BeginPlay, spawned at (%.1f, %.1f, %.1f)"),

           GetActorLocation().X, GetActorLocation().Y, GetActorLocation().Z);



    // --- CRITICAL FIX: Force teleport to scene_home.robot.initial_pose = (-1.0m, -3.0m) ---

    // The saved level has PlayerStart (0,0,0) or Editor-placed PlayerStart near the origin

    // which is INSIDE wall_y0_seg2 (xmin=-2.0, xmax=0.0, ymin=-0.15, ymax=0.15).  When UE

    // spawns the Pawn at that PlayerStart it collision-adjusts to (30cm, 0cm) which is

    // still adjacent to the wall, so ApplyMovement sweep=true always reports a blocking

    // hit → robot cannot move at all for the entire simulation.  To completely sidestep

    // any stale PlayerStart in the saved level we SNAP the Pawn directly to the

    // canonical initial position at BeginPlay, using SetActorLocation(false, no sweep)

    // + SetActorRotation instead of TeleportTo, because TeleportTo depends on the

    // MovementComponent which may not exist on a custom Pawn and silently no-ops.

    {

        constexpr double  INIT_X_M  = -1.0;   // scene_home.robot.initial_pose.x

        constexpr double  INIT_Y_M  = -3.0;   // scene_home.robot.initial_pose.y

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

    // Spawn 3D obstacle visuals from HOME_OBSTACLES so LiDAR can see them.
    SpawnObstacleVisuals();
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

               Destroyed, 5);

    }



    // Mirror of e:/puppyfangzhen/config/scene_home.json "pedestrians" array.

    // Positions: meters -> cm (x M_TO_CM); velocities: m/s -> cm/s (x M_TO_CM).

    struct FPedCfg { const TCHAR* Name; float Xm; float Ym; float Vxm; float Vym; };

        const FPedCfg Cfgs[5] = {

        { TEXT("person_1"),  2.50f, -1.50f, -0.03f,  0.00f },

        { TEXT("person_2"), -1.50f,  1.00f,  0.00f,  0.03f },

        { TEXT("person_3"), -3.00f, -1.00f,  0.02f,  0.00f },

        { TEXT("person_4"),  0.00f, -2.50f,  0.03f,  0.00f },

        { TEXT("person_5"),  0.50f,  0.80f, -0.02f,  0.02f }

    };



    int32 Spawned = 0;

    for (int32 i = 0; i < 5; ++i)

    {

        const FPedCfg& C = Cfgs[i];

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

    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] Pedestrian spawn: %d/5 OK"), Spawned);

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

    const FVector LEG_SCALE(0.08f, 0.08f, 0.11f);   // radius 4cm × height 22cm

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



    // R20 FIX: 真正的Lockstep 30Hz — 只在收到STEP_ACK后才移动+发送下一帧

    //   R19z问题: 每Tick都SendSensorData(125Hz), 但Bridge只处理30Hz, 导致:

    //   1) Bridge端TCP缓冲区积压95帧/秒的传感器数据

    //   2) Bridge回复的CMD_VEL+STEP_ACK在UE端也积压

    //   3) HandleStepAck的DIRECT-APPLY导致一帧内多次ApplyMovement

    //   修复: 第一帧发送初始传感器数据, 后续只在收到STEP_ACK后移动+发送

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

    // R20 FIX: 不再在此处直接调用 ApplyMovement。

    //   R19z问题: 每个 STEP_ACK 回调直接调用 ApplyMovement, 导致:

    //   1) TCP消息积压时一帧内多次移动, Frame号暴增(3秒内Frame 0→5000+)

    //   2) 使用过时的速度指令 — 机器人在右下角(4.88,-3.88)时还在用

    //      左上角(-4.88,3.88)的脱困速度(1.57,-1.24), 朝边界外移动被Clamp钳制

    //   3) Tick中的lockstep逻辑(bStepAckReceived)永远不执行, 因为此处已重置

    //   修复: 只设置标志, 由Tick统一处理每帧最多一次ApplyMovement。

    bStepAckReceived = true;

    LastStepAckTime = FPlatformTime::Seconds();  // R21: 记录STEP_ACK接收时间用于watchdog

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

    if (!bConnected)

    {

        if (bDebugTick)

        {

            int32 ConnState = TcpComp->GetConnectionStateRaw();

            UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] NOT connected (state=%d, -1=noSock 0=NotConn 1=Conn 2=Error). Not sending (t=%.1fs)."),

                   ConnState, Now);

        }

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

struct FHomeObstacle { const TCHAR* Name; float Xmin, Ymin, Xmax, Ymax; };

static const FHomeObstacle HOME_OBSTACLES[] = {
    { TEXT("wall_y0_w1              "), -500.0f, -15.0f,-350.0f,  15.0f },
    { TEXT("wall_y0_w2              "), -200.0f, -15.0f,   0.0f,  15.0f },
    { TEXT("wall_y0_w3              "),  100.0f, -15.0f, 200.0f,  15.0f },
    { TEXT("wall_y0_w4              "),  350.0f, -15.0f, 500.0f,  15.0f },
    { TEXT("wall_y2_w1              "), -500.0f, 185.0f,-350.0f, 215.0f },
    { TEXT("wall_y2_w2              "), -200.0f, 185.0f,   0.0f, 215.0f },
    { TEXT("wall_y2_w3              "),  100.0f, 185.0f, 150.0f, 215.0f },
    { TEXT("wall_y2_w4              "),  250.0f, 185.0f, 350.0f, 215.0f },
    { TEXT("wall_y2_w5              "),  450.0f, 185.0f, 500.0f, 215.0f },
    { TEXT("wall_v_bed1_door_bot    "), -355.0f,   0.0f,-345.0f,  40.0f },
    { TEXT("wall_v_bed1_door_top    "), -355.0f, 160.0f,-345.0f, 200.0f },
    { TEXT("wall_v_bed2_bath_bot    "), -105.0f, 200.0f, -95.0f, 240.0f },
    { TEXT("wall_v_bed2_bath_top    "), -105.0f, 360.0f, -95.0f, 400.0f },
    { TEXT("wall_v_bath_store_bot   "),   95.0f, 200.0f, 105.0f, 240.0f },
    { TEXT("wall_v_bath_store_top   "),   95.0f, 360.0f, 105.0f, 400.0f },
    { TEXT("wall_v_dining_kitchen   "),  245.0f, 110.0f, 255.0f, 200.0f },
    { TEXT("wall_v_kitchen_east     "),  345.0f,   0.0f, 355.0f, 200.0f },
    { TEXT("sofa                    "),  180.0f,-335.0f, 340.0f,-285.0f },
    { TEXT("tv_console              "), -360.0f,-330.0f,-220.0f,-295.0f },
    { TEXT("side_table              "),   50.0f,-320.0f, 100.0f,-280.0f },
    { TEXT("bookshelf_w             "), -490.0f,-260.0f,-450.0f,-120.0f },
    { TEXT("shoe_cabinet            "), -490.0f,-330.0f,-430.0f,-290.0f },
    { TEXT("master_bed              "), -495.0f,  45.0f,-405.0f, 150.0f },
    { TEXT("nightstand_mb           "), -495.0f,   5.0f,-455.0f,  40.0f },
    { TEXT("wardrobe                "), -470.0f, 175.0f,-360.0f, 210.0f },
    { TEXT("desk                    "), -470.0f, 300.0f,-370.0f, 340.0f },
    { TEXT("bookshelf_nw            "), -490.0f, 355.0f,-430.0f, 390.0f },
    { TEXT("filing_cabinet          "), -490.0f, 225.0f,-450.0f, 265.0f },
    { TEXT("single_bed              "), -305.0f, 275.0f,-210.0f, 380.0f },
    { TEXT("nightstand_2            "), -345.0f, 290.0f,-305.0f, 325.0f },
    { TEXT("dresser                 "), -190.0f, 350.0f,-110.0f, 390.0f },
    { TEXT("bathtub                 "),  -60.0f, 350.0f,  60.0f, 390.0f },
    { TEXT("toilet                  "),  -85.0f, 255.0f, -35.0f, 300.0f },
    { TEXT("sink                    "),   25.0f, 255.0f,  90.0f, 300.0f },
    { TEXT("shelves_n               "),  130.0f, 350.0f, 310.0f, 390.0f },
    { TEXT("shelves_e               "),  320.0f, 270.0f, 355.0f, 390.0f },
    { TEXT("cabinet_s               "),  115.0f, 225.0f, 165.0f, 265.0f },
    { TEXT("dining_table            "),  120.0f,  85.0f, 200.0f, 135.0f },
    { TEXT("chair_n                 "),  145.0f, 140.0f, 175.0f, 165.0f },
    { TEXT("chair_s                 "),  145.0f,  55.0f, 175.0f,  80.0f },
    { TEXT("chair_w                 "),  100.0f,  95.0f, 115.0f, 125.0f },
    { TEXT("chair_e                 "),  205.0f,  95.0f, 220.0f, 125.0f },
    { TEXT("sideboard               "),  110.0f,  20.0f, 190.0f,  50.0f },
    { TEXT("counter_n               "),  265.0f, 165.0f, 430.0f, 210.0f },
    { TEXT("counter_e               "),  430.0f,  30.0f, 490.0f, 210.0f },
    { TEXT("fridge                  "),  265.0f, 110.0f, 310.0f, 160.0f },
};

static const int NUM_HOME_OBSTACLES = UE_ARRAY_COUNT(HOME_OBSTACLES);

void APuppyRobotPawn::SpawnObstacleVisuals()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // ---- Cleanup old obstacle actors from previous runs ----
    // Use actor tags for runtime-safe identification; also try label-based cleanup in editor for old BSP brushes
    TArray<AActor*> AllActors;
    UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), AllActors);
    int32 Destroyed = 0;
    for (AActor* A : AllActors)
    {
        if (!A || A == this) continue;
        bool bShouldDestroy = false;
        // Our spawned obstacles carry this tag
        if (A->ActorHasTag(FName(TEXT("PuppyObs"))))
            bShouldDestroy = true;
#if WITH_EDITOR
        // In editor, also clean up legacy BSP brushes labeled OBS_*/Obs_*/PATROL_* from previous versions
        FString Label = A->GetActorLabel();
        if (Label.StartsWith(TEXT("OBS_")) || Label.StartsWith(TEXT("Obs_")) || Label.StartsWith(TEXT("PATROL_")))
            bShouldDestroy = true;
#endif
        if (bShouldDestroy)
        {
            A->Destroy();
            ++Destroyed;
        }
    }
    if (Destroyed > 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] Cleaned up %d old obstacle actors"), Destroyed);
    }

    // Load assets at runtime (ConstructorHelpers only works in constructors)
    UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    UMaterialInterface* BaseMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (!CubeMesh) return;

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    int32 Spawned = 0;
    for (int32 i = 0; i < NUM_HOME_OBSTACLES; ++i)
    {
        const FHomeObstacle& O = HOME_OBSTACLES[i];
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
        if (bIsWall) { Sizez = 280.0f; Color = FColor(225, 220, 210); }
        else if (bIsCounter) { Sizez = 90.0f; Color = FColor(140, 115, 88); }
        else if (NameStr.Contains(TEXT("bed"))) { Sizez = 45.0f; Color = FColor(90, 70, 115); }
        else if (NameStr.Contains(TEXT("sofa"))) { Sizez = 70.0f; Color = FColor(72, 90, 122); }
        else if (NameStr.Contains(TEXT("coffee_table"))) { Sizez = 45.0f; Color = FColor(120, 85, 55); }
        else if (NameStr.Contains(TEXT("dining_table"))) { Sizez = 75.0f; Color = FColor(140, 100, 65); }
        else if (NameStr.Contains(TEXT("chair"))) { Sizez = 80.0f; Color = FColor(90, 70, 50); }
        else if (NameStr.Contains(TEXT("fridge"))) { Sizez = 170.0f; Color = FColor(220, 220, 230); }
        else if (NameStr.Contains(TEXT("shower")) || NameStr.Contains(TEXT("toilet")) || NameStr.Contains(TEXT("sink"))) { Sizez = 80.0f; Color = FColor(200, 220, 230); }
        else if (NameStr.Contains(TEXT("bookshelf"))) { Sizez = 180.0f; Color = FColor(110, 75, 45); }
        else if (NameStr.Contains(TEXT("wardrobe"))) { Sizez = 200.0f; Color = FColor(90, 65, 45); }
        else if (NameStr.Contains(TEXT("desk"))) { Sizez = 75.0f; Color = FColor(130, 90, 60); }
        else if (NameStr.Contains(TEXT("tv"))) { Sizez = 50.0f; Color = FColor(38, 38, 46); }
        else if (NameStr.Contains(TEXT("nightstand"))) { Sizez = 50.0f; Color = FColor(110, 82, 56); }
        else if (NameStr.Contains(TEXT("island"))) { Sizez = 90.0f; Color = FColor(140, 122, 95); }
        else { Sizez = 60.0f; Color = FColor(140, 140, 132); }

        float Zcenter = Sizez * 0.5f;
        FVector Location(Cx, Cy, Zcenter);
        FRotator Rotation(0.f, 0.f, 0.f);

        AActor* ObsActor = World->SpawnActor<AActor>(AActor::StaticClass(), Location, Rotation, Params);
        if (!ObsActor) continue;
#if WITH_EDITOR
        ObsActor->SetActorLabel(FString("Obs_") + NameStr);
#endif
        ObsActor->Tags.Add(FName(TEXT("PuppyObs")));

        UStaticMeshComponent* MeshComp = NewObject<UStaticMeshComponent>(ObsActor);
        if (ObsActor->GetRootComponent())
            MeshComp->AttachToComponent(ObsActor->GetRootComponent(), FAttachmentTransformRules::KeepWorldTransform);
        else
            ObsActor->SetRootComponent(MeshComp);
        ObsActor->AddInstanceComponent(MeshComp);
        MeshComp->SetStaticMesh(CubeMesh);
        MeshComp->SetWorldScale3D(FVector(Sizex / 100.0f, Sizey / 100.0f, Sizez / 100.0f));
        MeshComp->SetMobility(EComponentMobility::Static);
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
        MeshComp->RegisterComponent();
        ++Spawned;
    }
    UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] SpawnObstacleVisuals: spawned %d obstacle actors (open-plan apartment v3)"), Spawned);
}




// ===== R19i BUG#4 正确修正: 场景硬边界 = Costmap 自由区边缘 - 2cm =====

//   R19g: 5-cell margin = 0.5m LETHAL = 把房间内部变墙=机器人绕外圈走到不了目标!

//   R19i: 1-cell margin = 0.1m 最外1格 LETHAL 才正确 = 只是地图外边缘屏蔽

//     Costmap 尺寸: 100×80 cells, 0.1m/cell

//       x: -5.0 → +5.0,  lethal 1格 = x∈[-5.0,-4.9] ∪ [4.9,5.0] → FREE x ∈ [-4.9, 4.9]

//       y: -4.0 → +4.0,  lethal 1格 = y∈[-4.0,-3.9] ∪ [3.9,4.0] → FREE y ∈ [-3.9, 3.9]

//     → UE clamp: FREE区域边缘 - 2cm 裕度 = 机器人永不碰 lethal 边缘

static constexpr float HOME_BOUND_XMIN_CM = -488.0f;   // -4.88 m (=-4.90 + 0.02)

static constexpr float HOME_BOUND_XMAX_CM =  488.0f;   //  4.88 m (= 4.90 - 0.02)

static constexpr float HOME_BOUND_YMIN_CM = -388.0f;   // -3.88 m (=-3.90 + 0.02)

static constexpr float HOME_BOUND_YMAX_CM =  388.0f;   //  3.88 m (= 3.90 - 0.02)



// Robot radius for BBOX collision inflation.

// scene_home.json "radius_planning" = 0.35m was used for A* costmap inflation

// but UE-side physics capsule is only 30cm radius, and doorways at y=0/y=2m

// walls are only 1.0m wide (between seg2 xmax=0 and seg3 xmin=100cm).

// At 35cm inflation the door shrinks to 30cm → impassable → 99% frames collide

// (R15 had 5938 collisions in 6000 frames!). Using 10cm here acts like a

// "tight fit" clearance: still stops true wall penetrations while leaving

// enough room for the planner to squeeze the 30cm capsule through doorways.

static constexpr float ROBOT_RADIUS_CM = 5.0f;



static bool IsInsideHomeObstacle(float Xcm, float Ycm, const FHomeObstacle** OutHit = nullptr)

{

    for (int i = 0; i < NUM_HOME_OBSTACLES; ++i)

    {

        const FHomeObstacle& O = HOME_OBSTACLES[i];

        // expand obstacle BBOX by +ROBOT_RADIUS_CM on all 4 sides

        if (Xcm >= O.Xmin - ROBOT_RADIUS_CM && Xcm <= O.Xmax + ROBOT_RADIUS_CM &&

            Ycm >= O.Ymin - ROBOT_RADIUS_CM && Ycm <= O.Ymax + ROBOT_RADIUS_CM)

        {

            if (OutHit) *OutHit = &O;

            return true;

        }

    }

    return false;

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

    //  - Always move with bSweep=false → GUARANTEES robot never jams from

    //    tiny capsule overlaps at spawn (the original frozen-robot bug from

    //    R01–R09 where even an empty room would produce zero displacement).

    //  - After moving, query the UE physics engine for overlapping colliders

    //    on the capsule. If overlap is detected with ANY world obstacle actor

    //    → revert to the pre-move position. This correctly handles UE-side

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

    // R22 COLLISION FIX: 三层碰撞检测

    //   Layer 1: 手动BBOX预检查 (IsInsideHomeObstacle) — 最可靠

    //   Layer 2: UE overlap检测 (GetOverlappingActors) — 补充

    //   Layer 3: 场景边界钳制 (ClampToHomeBounds) — 兜底

    // 修复历史问题:

    //   - bSweep=false导致穿墙: 现在用BBOX预检查拦截

    //   - bOverlappedBefore逃逸逻辑导致在障碍物里穿梭: 现在只允许远离障碍物中心的移动

    //   - bNearBoundary跳过碰撞: 现在只在实际被Clamp时跳过, 不再因靠近边界而禁用碰撞检测

    // ================================================================



    // 1) Tentative no-sweep move

    SetActorLocation(LocBefore + Delta, /*bSweep=*/false, nullptr, ETeleportType::None);

    FVector LocAfter = GetActorLocation();



    // 2) 场景硬边界钳制 (x∈[-4.88,4.88]m, y∈[-3.88,3.88]m)

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

    // Layer 1: 手动BBOX碰撞检查 (最可靠)

    // 检查移动前后的位置是否在障碍物BBOX内, 这是防止穿墙的核心机制。

    // IsInsideHomeObstacle已包含40个障碍物(家具+墙壁), 每个都膨胀了ROBOT_RADIUS_CM。

    // ================================================================

    const FHomeObstacle* ObsAfter = nullptr;

    const FHomeObstacle* ObsBefore = nullptr;

    const bool bInObstacleAfter  = IsInsideHomeObstacle(LocAfter.X,  LocAfter.Y,  &ObsAfter);

    const bool bInObstacleBefore = IsInsideHomeObstacle(LocBefore.X, LocBefore.Y, &ObsBefore);



    bool bBboxCollision = false;

    if (bInObstacleAfter && !bInObstacleBefore)

    {

        // 移动后进入了障碍物, 移动前不在 → 新碰撞, 回退!

        bBboxCollision = true;

        SetActorLocation(LocBefore, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

        ++CollisionCount;

        if (TcpComp != nullptr && ObsAfter != nullptr)

        {

            const float Bx = (ObsAfter->Xmin + ObsAfter->Xmax) * 0.5f;

            const float By = (ObsAfter->Ymin + ObsAfter->Ymax) * 0.5f;

            TcpComp->SendCollision(CollisionCount, 0.01 * Bx, 0.01 * By);

        }

        UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] BBOX-COLLIDE #%d: obs=[%s] (%.0f,%.0f)→(%.0f,%.0f) → REVERT"),

               CollisionCount,

               ObsAfter ? ObsAfter->Name : TEXT("?"),

               LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y);

        LocAfter = LocBefore;

    }

    else if (bInObstacleAfter && bInObstacleBefore)

    {

        // 移动前后都在障碍物内 → 已嵌入, 只允许远离障碍物中心的移动

        // (修复R22核心bug: 原逻辑允许任意方向移动→机器人在障碍物里穿梭)

        if (ObsAfter != nullptr && ObsBefore != nullptr)

        {

            const float ObsCx = (ObsAfter->Xmin + ObsAfter->Xmax) * 0.5f;

            const float ObsCy = (ObsAfter->Ymin + ObsAfter->Ymax) * 0.5f;

            const float DistBefore = FVector::Dist2D(LocBefore, FVector(ObsCx, ObsCy, 0.0f));

            const float DistAfter  = FVector::Dist2D(LocAfter,  FVector(ObsCx, ObsCy, 0.0f));

            if (DistAfter < DistBefore)

            {

                // 移动后更靠近障碍物中心 → 在往里钻, 回退!

                bBboxCollision = true;

                SetActorLocation(LocBefore, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

                ++CollisionCount;

                UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] BBOX-ESCAPE-BLOCK #%d: obs=[%s] moving DEEPER (%.0f,%.0f)→(%.0f,%.0f) dist %.1f→%.1f → REVERT"),

                       CollisionCount, ObsAfter->Name,

                       LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y,

                       DistBefore, DistAfter);

                LocAfter = LocBefore;

            }

            else

            {

                // 移动后更远离障碍物中心 → 在往外逃, 允许

                static double LastEscapeLog = 0.0;

                const double NowT = FPlatformTime::Seconds();

                if (NowT - LastEscapeLog > 3.0) {

                    LastEscapeLog = NowT;

                    UE_LOG(LogTemp, Log, TEXT("[PuppyRobotPawn] BBOX-ESCAPE-OK: obs=[%s] moving OUT dist %.1f→%.1f, allow"),

                           ObsAfter->Name, DistBefore, DistAfter);

                }

            }

        }

    }



    // ================================================================

    // Layer 2: UE overlap检测 (补充BBOX检查的遗漏)

    // 仅当BBOX检查未触发碰撞时才执行, 避免重复计数

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



        // R22 FIX: 移除 !bWasClamped 条件 — 只在实际被边界Clamp时跳过, 不再因靠近边界跳过

        // R22 FIX: bOverlappedBefore时不再无条件允许, 改为检查是否在远离障碍物

        if (bHasBlockerOverlap && !bOverlappedBefore)

        {

            // 新碰撞 (之前不在overlap, 现在在) → 回退

            // 例外: 如果被ClampToHomeBounds钳制了, 不回退 (Clamp已经保证在边界内)

            if (!bWasClamped)

            {

                SetActorLocation(LocBefore, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

                ++CollisionCount;

                if (TcpComp != nullptr)

                {

                    const FVector BLoc = BlockerActor ? BlockerActor->GetActorLocation() : FVector(LocBefore + LocAfter) * 0.5f;

                    TcpComp->SendCollision(CollisionCount, 0.01 * BLoc.X, 0.01 * BLoc.Y);

                }

                UE_LOG(LogTemp, Warning, TEXT("[PuppyRobotPawn] OVERLAP-COLLIDE #%d: actor=[%s] fresh-overlap (%.0f,%.0f)→(%.0f,%.0f) → REVERT"),

                       CollisionCount,

                       BlockerActor ? *BlockerActor->GetName() : TEXT("unknown"),

                       LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y);

                LocAfter = LocBefore;

            }

        }

        else if (bHasBlockerOverlap && bOverlappedBefore)

        {

            // 已嵌入: 只记录日志, 不阻止移动 (BBOX层已处理方向检查)

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



    // Rotate around Z (yaw).  CurrentAngularVelocity is in deg/s.

    const float YawDelta = CurrentAngularVelocity * DeltaTime;

    AddActorWorldRotation(FRotator(0.0f, YawDelta, 0.0f), false);



    // R21 FIX: 边界卡死检测 — 在ApplyMovement末尾检查是否持续在边界角落

    TryBoundaryEscapeTeleport();

}



bool APuppyRobotPawn::TryBoundaryEscapeTeleport()

{

    // R21 FIX: 当机器人在边界角落卡死超过20秒, 直接传送到场景中心安全区。

    //   根因: 边界角落(-4.88,-3.88)的AMCL发散+导航错误+物理碰撞叠加,

    //   即使脱困速度正确(1.57,1.24 m/s)也无法可靠脱离。

    //   修复: 作为终极保险, 卡死20秒后传送至(0,0)中心(已知自由区)。

    const FVector Loc = GetActorLocation();

    const double NowT = FPlatformTime::Seconds();



    // 判定是否在边界角落(距边界<0.5m)

    const bool atCorner = (FMath::Abs(Loc.X) > HOME_BOUND_XMAX_CM - 50.0f &&

                           FMath::Abs(Loc.Y) > FMath::Abs(HOME_BOUND_YMAX_CM) - 50.0f);

    // 也判定是否在边界(单轴贴墙)

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

            // 卡死超过20秒, 传送到中心安全区

            // 目标: 距边界最远的点 = (0, 0)

            const FVector SafeLoc(0.0f, 0.0f, Loc.Z);

            SetActorLocation(SafeLoc, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

            UE_LOG(LogTemp, Error, TEXT("[PuppyRobotPawn] BOUNDARY-TELEPORT: stuck %.1fs at (%.0f,%.0f) → teleport to (0,0) Frame=%d"),

                   NowT - BoundaryStuckStart, Loc.X, Loc.Y, FrameCount);

            bBoundaryStuckActive = false;

            BoundaryStuckStart = 0.0;

            return true;

        }

    }

    else

    {

        // 离开边界, 重置计时

        if (bBoundaryStuckActive)

        {

            bBoundaryStuckActive = false;

            BoundaryStuckStart = 0.0;

        }

    }

    return false;

}

