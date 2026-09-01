// PuppyPedestrianActor.cpp -- UE5 dynamic pedestrian actor
// ==================================================================
// Mirrors the C++ simulation's pedestrian behavior (simulation.h):
//   - Constant-velocity straight-line motion (velocity in cm/s)
//   - Wall bounce via IsInsideWall overlap test (flips velocity component)
//   - Outer boundary bounce at +/-450 cm X, +/-350 cm Y
//
// Unit conventions: UE world units are cm; Velocity is in cm/s
// (e.g. 0.04 m/s -> 4 cm/s).
#include "PuppyPedestrianActor.h"

#include "Engine/World.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"

APuppyPedestrianActor::APuppyPedestrianActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    CapsuleComp = CreateDefaultSubobject<UCapsuleComponent>(TEXT("CapsuleComp"));
    RootComponent = CapsuleComp;

    if (CapsuleComp != nullptr)
    {
        CapsuleComp->SetCapsuleRadius(Radius);
        CapsuleComp->SetCapsuleHalfHeight(Radius);
        CapsuleComp->SetCollisionProfileName(TEXT("OverlapAll"));
        CapsuleComp->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
        CapsuleComp->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
    }
}

void APuppyPedestrianActor::Initialize(const FString& Name, float X, float Y, float Vx, float Vy)
{
    SetActorLocation(FVector(X, Y, 0.0f));

    Velocity = FVector2D(Vx, Vy);

    InitialPosition = FVector2D(X, Y);
    InitialVelocity = FVector2D(Vx, Vy);
}

void APuppyPedestrianActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Move by Velocity * DeltaTime (cm/s * s = cm)
    const FVector Loc = GetActorLocation();
    const FVector NewLoc = Loc + FVector(Velocity.X * DeltaTime, Velocity.Y * DeltaTime, 0.0f);
    SetActorLocation(NewLoc);

    CheckWallBounce();
}

void APuppyPedestrianActor::CheckWallBounce()
{
    const FVector Loc = GetActorLocation();
    float X = Loc.X;
    float Y = Loc.Y;
    bool bBounced = false;

    // Outer boundary bounce (matches simulation.h: +/-4.5 m X, +/-3.5 m Y)
    if (X <= -450.0f)
    {
        Velocity.X = FMath::Abs(Velocity.X);
        bBounced = true;
    }
    else if (X >= 450.0f)
    {
        Velocity.X = -FMath::Abs(Velocity.X);
        bBounced = true;
    }

    if (Y <= -350.0f)
    {
        Velocity.Y = FMath::Abs(Velocity.Y);
        bBounced = true;
    }
    else if (Y >= 350.0f)
    {
        Velocity.Y = -FMath::Abs(Velocity.Y);
        bBounced = true;
    }

    // Wall overlap test (equivalent to is_inside_wall in simulation.h)
    if (IsInsideWall(X, Y))
    {
        // Determine which axis caused the collision by checking the
        // previous position (one step back).
        const float DeltaTime = (GetWorld() != nullptr) ? GetWorld()->GetDeltaSeconds() : 0.0333f;
        const float PrevX = X - Velocity.X * DeltaTime;
        const float PrevY = Y - Velocity.Y * DeltaTime;

        if (!IsInsideWall(X, PrevY))
        {
            // Y change drove the collision -> bounce Y
            Velocity.Y = -Velocity.Y;
            bBounced = true;
        }
        else if (!IsInsideWall(PrevX, Y))
        {
            // X change drove the collision -> bounce X
            Velocity.X = -Velocity.X;
            bBounced = true;
        }
        else
        {
            // Both axes blocked -> bounce both
            Velocity.X = -Velocity.X;
            Velocity.Y = -Velocity.Y;
            bBounced = true;
        }
    }

    if (bBounced)
    {
        ++BounceCount;
    }
}

bool APuppyPedestrianActor::IsInsideWall(float X, float Y) const
{
    UWorld* World = GetWorld();
    if (World == nullptr || CapsuleComp == nullptr)
    {
        return false;
    }

    // Use the capsule shape to test overlap against WorldStatic geometry
    const FCollisionShape Shape = CapsuleComp->GetCollisionShape();
    const FVector Location(static_cast<double>(X), static_cast<double>(Y), GetActorLocation().Z);
    const FQuat Rotation = FQuat::Identity;

    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this);

    return World->OverlapAnyTestByChannel(Location, Rotation, ECC_WorldStatic, Shape, QueryParams);
}
