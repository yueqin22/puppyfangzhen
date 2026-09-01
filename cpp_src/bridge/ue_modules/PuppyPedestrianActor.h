// PuppyPedestrianActor.h -- UE5 dynamic pedestrian actor (auto patrol + wall bounce)
// ================================================================
// Represents a dynamic pedestrian in the scene; behavior matches the C++
// simulation (simulation.h):
//   - Constant-velocity straight-line motion (speed 0.03~0.04 m/s, i.e. 3~4 cm/s)
//   - Wall bounce (detected via IsInsideWall; on bounce the velocity component is flipped)
//   - Supports setting initial position and velocity at runtime
//
// Unit conventions: UE world units are cm; Velocity is in cm/s.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/CapsuleComponent.h"
#include "PuppyPedestrianActor.generated.h"

UCLASS()
class PUPPYNAV_API APuppyPedestrianActor : public AActor
{
    GENERATED_BODY()
public:
    APuppyPedestrianActor();

    // Capsule collision component (root component)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UCapsuleComponent* CapsuleComp;

    // Current velocity (cm/s, in the XY plane)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Pedestrian")
    FVector2D Velocity = FVector2D(0.0f, 0.0f);

    // Pedestrian radius (cm), matches the C++ simulation (30cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Pedestrian")
    float Radius = 30.0f;

    // Accumulated bounce count (for debugging)
    UPROPERTY(BlueprintReadOnly, Category="Pedestrian")
    int32 BounceCount = 0;

    // Runtime initialization: sets name, initial position and velocity
    // Name: pedestrian name (debug label); X/Y: initial position (cm); Vx/Vy: initial velocity (cm/s)
    void Initialize(const FString& Name, float X, float Y, float Vx, float Vy);

protected:
    virtual void Tick(float DeltaTime) override;

private:
    // Records initial position and velocity (can be restored on RESET)
    FVector2D InitialPosition;
    FVector2D InitialVelocity;

    // Detects a wall hit and performs a bounce (flips the relevant velocity component, BounceCount++)
    void CheckWallBounce();

    // Tests whether point (X, Y) lies inside a wall (BSP brush / obstacle)
    bool IsInsideWall(float X, float Y) const;
};
