// PuppyLiDARComponent.h -- UE5 360-degree LiDAR sensor component
// ================================================================
// Simulates a 360-degree LiDAR: 72 rays evenly distributed across
// [-PI, +PI], max range 8m (800cm).  Each tick performs a LineTrace per ray
// against scene BSP brushes, returning a float[72] distance array; rays
// beyond max_range are marked as max_range.
//
// When communicating with nav_ue_bridge, PuppyRobotPawn calls GetScanData to
// fetch the scan result, then PuppyTcpServer packs it as a LIDAR message
// (see protocol.h for the protocol).
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PuppyLiDARComponent.generated.h"

UCLASS(ClassGroup=(PuppyNav), meta=(BlueprintSpawnableComponent))
class PUPPYNAV_API UPuppyLiDARComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UPuppyLiDARComponent();

    // Number of rays (default 72, one every 5 degrees to cover 360 degrees)
    UPROPERTY(EditAnywhere, Category="LiDAR")
    int32 NumRays = 72;

    // Maximum range (cm), 8m = 800cm; rays beyond this are marked with max_range
    UPROPERTY(EditAnywhere, Category="LiDAR")
    float MaxRange = 800.0f;

    // Start angle (radians), default -PI
    UPROPERTY(EditAnywhere, Category="LiDAR")
    float AngleMin = -PI;

    // End angle (radians), default +PI
    UPROPERTY(EditAnywhere, Category="LiDAR")
    float AngleMax = PI;

    // Gets the latest scan data
    // OutDistances: distance array (length = NumRays, in cm; out-of-range entries set to MaxRange)
    // OutMaxRange:  maximum range (cm)
    UFUNCTION(BlueprintCallable, Category="LiDAR")
    void GetScanData(TArray<float>& OutDistances, float& OutMaxRange) const;

protected:
    // Performs one scan per tick and stores the result in LastDistances_
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    // Distance result of the most recent scan (cm), length = NumRays
    TArray<float> LastDistances_;

    // Performs one 360-degree scan, calling LineTraceByChannel for each ray
    void PerformScan();
};
