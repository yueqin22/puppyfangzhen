// PuppyLiDARComponent.cpp -- UE5 360-degree LiDAR sensor component
// ==================================================================
// Simulates a 360-degree LiDAR: NumRays rays evenly distributed across
// [AngleMin, AngleMax], max range 8m (800cm).  Each tick performs a
// LineTrace per ray against scene geometry and stores the hit distance
// (in cm) in LastDistances_.  Out-of-range rays are marked with MaxRange.
//
// The robot pawn calls GetScanData() to fetch the scan result and forwards
// it via PuppyTcpServer (packed as a LIDAR message, see protocol.h).
#include "PuppyLiDARComponent.h"

#include "Engine/World.h"
#include "CollisionQueryParams.h"

UPuppyLiDARComponent::UPuppyLiDARComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = true;
    // Run before physics so the scan reflects the pre-tick pose
    PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UPuppyLiDARComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    PerformScan();
}

void UPuppyLiDARComponent::PerformScan()
{
    AActor* Owner = GetOwner();
    if (Owner == nullptr)
    {
        return;
    }

    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }

    // UActorComponent has no transform of its own -- use the owner's pose.
    FVector ScanStart = Owner->GetActorLocation();
    ScanStart.Z += 20.0f;  // Elevate scan plane to waist height (+20cm) to prevent floor mesh self-tracing
    // For 2D planar LiDAR we must ignore any 3D tilt (Pitch / Roll) and only
    // keep the Yaw component.  PlayerStart / Spawn collision adjustments often
    // introduce tiny Pitch/Roll (~0.1°) which is enough to tilt a 7m ray by
    // 12cm vertically at max range, causing ALL rays to overshoot / undershoot
    // the 10cm-thick wall colliders entirely (returns 8m MaxRange on every
    // ray → AMCL likelihood field sees no walls → pure odometry drift →
    // 1–2m avg error after 200s).  Extract Yaw from FRotator directly.
    const FRotator OwnerRPY = Owner->GetActorRotation();
    const float YawRad = FMath::DegreesToRadians(OwnerRPY.Yaw);
    const float CosY = FMath::Cos(YawRad);
    const float SinY = FMath::Sin(YawRad);

    LastDistances_.SetNum(NumRays);

    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(Owner);
    QueryParams.bTraceComplex = false;

    // CRITICAL FIX: trace against OBJECT types (WorldStatic + WorldDynamic),
    // not a TRACE channel (ECC_Visibility / ECC_Camera).  The default UE5
    // collision profile for BasicShapes cubes (walls, sofas, tables) does not
    // reliably set ECC_Visibility to Block — it depends on project settings.
    // Tracing by ObjectType guarantees we hit every solid actor regardless of
    // trace-channel configuration: walls / furniture / kitchen counters are
    // all WorldStatic; moving pedestrians / dynamic props are WorldDynamic.
    // Without this fix, every LiDAR ray passes cleanly through walls at
    // MaxRange (8m), so AMCL's likelihood field never finds a wall match
    // → the particle cloud drifts on pure odometry (avg 1–1.5m error after
    // 200s, max 4m+) and can never reconverge.
    FCollisionObjectQueryParams ObjParams;
    ObjParams.AddObjectTypesToQuery(ECC_WorldStatic);
    ObjParams.AddObjectTypesToQuery(ECC_WorldDynamic);

    const float Range = MaxRange;

    for (int32 i = 0; i < NumRays; ++i)
    {
        // Local angle in [-PI, +PI], evenly spaced
        const float Angle = AngleMin + (AngleMax - AngleMin) * static_cast<float>(i) / static_cast<float>(NumRays);

        // Direction in LOCAL space (XY plane): local +X = robot forward.
        const float Lx = FMath::Cos(Angle);
        const float Ly = FMath::Sin(Angle);

        // Strictly 2D ROTATION by Yaw (no Pitch/Roll): rotate local direction
        // into WORLD space using only the Yaw component extracted above.
        //   Wx = Lx*cos(yaw) - Ly*sin(yaw)
        //   Wy = Lx*sin(yaw) + Ly*cos(yaw)
        //   Wz = 0  (guaranteed flat in XY plane — cannot overshoot walls)
        const float Wx = Lx * CosY - Ly * SinY;
        const float Wy = Lx * SinY + Ly * CosY;
        FVector WorldDir(Wx, Wy, 0.0f);
        WorldDir.Normalize();

        // Perform the trace at the robot's current Z height, NOT at capsule
        // bottom (-15cm).  ScanStart is typically at Z=+15cm (PlayerStart)
        // so the trace passes through the middle of walls (Z=0±10cm collider
        // height), ensuring intersection even with the thinnest AABB wall.
        const FVector End = ScanStart + WorldDir * Range;

        FHitResult HitResult;
        const bool bHit = World->LineTraceSingleByObjectType(HitResult, ScanStart, End, ObjParams, QueryParams);

        if (bHit)
        {
            // Hit.Distance is already in cm (UE world units)
            LastDistances_[i] = HitResult.Distance;
        }
        else
        {
            // No hit -- mark as max range (out of range)
            LastDistances_[i] = MaxRange;
        }
    }
}

void UPuppyLiDARComponent::GetScanData(TArray<float>& OutDistances, float& OutMaxRange) const
{
    OutDistances = LastDistances_;
    OutMaxRange = MaxRange;
}
