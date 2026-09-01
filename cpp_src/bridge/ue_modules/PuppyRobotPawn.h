// PuppyRobotPawn.h -- UE5 robot Pawn (hosts LiDAR + TCP Server)
// ================================================================
// Pawn representing the navigation robot; aggregates UPuppyLiDARComponent
// and UPuppyTcpServer.
// Workflow (30Hz lockstep):
//   1. Tick sends LIDAR + GROUND_TRUTH + PED_STATE to nav_ue_bridge
//   2. Receives CMD_VEL, updates linear/angular velocity and moves the robot
//   3. Collision detection (accumulates CollisionCount)
//   4. Advances physics by one frame after STEP_ACK is received (lockstep sync)
//
// Unit conventions: UE world units are cm; linear velocity in cm/s,
// angular velocity in deg/s (with conversion).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PuppyRobotPawn.generated.h"

class UPuppyLiDARComponent;
class UPuppyTcpServer;
class APuppyPedestrianActor;
class UCameraComponent;
class USpringArmComponent;
class UCapsuleComponent;
class UStaticMeshComponent;

UCLASS()
class PUPPYNAV_API APuppyRobotPawn : public APawn
{
    GENERATED_BODY()
public:
    APuppyRobotPawn();

    // LiDAR sensor component (subobject)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UPuppyLiDARComponent* LiDARComp;

    // TCP Server component (subobject, communicates with nav_ue_bridge)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UPuppyTcpServer* TcpComp;

    // Spring arm (third-person camera boom). Target length = 8m, height offset = 6m (top-down chase).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Camera")
    USpringArmComponent* CameraBoom;

    // Third-person camera attached to the spring arm (UE game viewport camera).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Camera")
    UCameraComponent* FollowCamera;

    // Visual mesh body: 60cm x 40cm x 18cm box body (Spot-style robot dog chassis).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* RobotBodyMesh;

    // Head mesh: smaller box mounted at front of body (with orange accent stripe like Boston Dynamics Spot).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* RobotHeadMesh;

    // 4 legs: short cylinders as simplified robotic legs (FR/FL/RR/RL).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* LegFL_Mesh;  // Front-Left
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* LegFR_Mesh;  // Front-Right
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* LegRL_Mesh;  // Rear-Left
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* LegRR_Mesh;  // Rear-Right

    // Accent stripe (orange) on the robot's back so it's easy to spot in the scene.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UStaticMeshComponent* AccentStripeMesh;

    // Capsule collision (root component). Radius=30cm, half-height=30cm → total height 60cm.
    // Stored as member so ApplyMovement can run GetOverlappingActors() for
    // fresh-overlap revert collision detection.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Components")
    UCapsuleComponent* CollisionCapsule;

    // Linear velocity upper limit (cm/s), 2 m/s = 200 cm/s
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Robot")
    float MoveSpeed = 200.0f;

    // Angular velocity upper limit (deg/s)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Robot")
    float RotateSpeed = 180.0f;

    // Accumulated collision count (reported in COLLISION messages)
    UPROPERTY(BlueprintReadOnly, Category="Robot")
    int32 CollisionCount = 0;

    // Current frame counter (lockstep frame)
    UPROPERTY(BlueprintReadOnly, Category="Robot")
    int32 FrameCount = 0;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

private:
    // Current linear velocity (cm/s, world space)
    FVector CurrentVelocity = FVector::ZeroVector;
    // Current angular velocity (deg/s)
    float CurrentAngularVelocity = 0.0f;
    // Whether STEP_ACK has been received this frame (lockstep: physics only advances after ACK)
    bool bStepAckReceived = false;
    bool bObstacleContactActive = false;
    bool bSimulationEnded = false;

    // R21 FIX: Lockstep watchdog — if no STEP_ACK for >0.5s, force-send sensor data
    double LastStepAckTime = 0.0;

    // R21 FIX: Boundary stuck tracking — for GT teleport escape after 20s stuck
    double BoundaryStuckStart = 0.0;
    bool bBoundaryStuckActive = false;

    // CMD_VEL callback: parses velocity command and updates CurrentVelocity / CurrentAngularVelocity
    UFUNCTION()
    void HandleCmdVel(const FPuppyCmdVel& CmdVel);

    // STEP_ACK callback: marks this frame as acknowledged, allowing physics to advance
    UFUNCTION()
    void HandleStepAck();

    UFUNCTION()
    void HandleSimulationEnd();

    // Sends this frame's sensor data (LiDAR + GroundTruth + PedState) to nav_ue_bridge
    void SendSensorData();

    // Applies velocity to move the robot (with collision detection)
    void ApplyMovement(float DeltaTime);

    // R21 FIX: Check if robot is stuck at boundary corner and teleport to safe position
    bool TryBoundaryEscapeTeleport();

    // Spawns 5 dynamic pedestrians from scene_home.json config (positions: cm; velocities: cm/s)
    void SpawnPedestriansFromScene();
    // Spawns 3D obstacle visuals from the runtime-loaded scene (P0-02) at BeginPlay (LiDAR + visual)
    void SpawnObstacleVisuals();

    // Forces the 7-part robot-dog visual (body/head/stripe/4 legs) to the correct
    // geometry + material colors, overriding any stale Blueprint CDO saved-state
    // that may hide or transform these C++-constructor-created subobjects.
    void ReinitRobotDogVisuals();
};
