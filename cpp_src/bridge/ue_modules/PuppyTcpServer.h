// PuppyTcpServer.h -- UE5 TCP client component (talks to nav_ue_bridge)
// ================================================================
// Connects (as a CLIENT) to the nav_ue_bridge TCP SERVER on localhost:Port,
// performs the v2 handshake, then maintains a long-lived connection.
//
// Frame layout (v2, matches bridge::FrameHeader in protocol.h, 16 bytes):
//   4B length(LE) + 2B type + 2B version + 4B sequence + 4B timestamp_ms
//   followed by `length` bytes of payload.
//
// Handshake (v2, P0-03):
//   * On connect, UE sends HELLO (magic + version + capabilities).
//   * Bridge replies HELLO_ACK (status=0 => OK). UE only processes normal
//     traffic after the handshake completes.
//   * When idle, UE sends HEARTBEAT to keep the bridge watchdog alive.
//
// Receive (Nav->UE): RESET / CMD_VEL / DEBUG_POSE / DEBUG_PATH / STEP_ACK /
//                    HELLO_ACK / HEARTBEAT / EMERGENCY_STOP / SHUTDOWN_ACK /
//                    SIM_END / FAULT / STATUS
// Send    (UE->Nav): LIDAR / GROUND_TRUTH / PED_STATE / COLLISION / HELLO /
//                    HEARTBEAT / SHUTDOWN_ACK
//
// Lockstep 30Hz: UE sends one sensor frame then waits for STEP_ACK from Nav
// before advancing physics. EMERGENCY_STOP forces an immediate safe stop.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Networking.h"
#include "Sockets.h"
#include "PuppyTcpServer.generated.h"

// Protocol version (must match bridge::PROTOCOL_VERSION in protocol.h)
constexpr uint16 PUPPY_PROTOCOL_VERSION = 2;
// Handshake magic (must match bridge::PROTOCOL_MAGIC = 0x5050)
constexpr uint16 PUPPY_PROTOCOL_MAGIC   = 0x5050;

// Message type constants (aligned with bridge::MsgType values, see protocol.h)
namespace PuppyMsgType {
    // ---- handshake / control (v2) ----
    constexpr uint16 HELLO          = 0x0006;  // UE->Nav: connect (version + caps)
    constexpr uint16 HELLO_ACK      = 0x0007;  // Nav->UE: handshake confirm
    constexpr uint16 RESUME         = 0x0008;  // Nav->UE: resume after pause
    constexpr uint16 HEARTBEAT      = 0x0009;  // either: keepalive
    constexpr uint16 EMERGENCY_STOP = 0x000A;  // either: safe stop
    constexpr uint16 SHUTDOWN_ACK   = 0x000B;  // UE->Nav: ack SIM_END
    constexpr uint16 FAULT          = 0x000C;  // either: protocol fault
    constexpr uint16 STATUS         = 0x000D;  // either: diagnostics
    // ---- control (bidirectional) ----
    constexpr uint16 RESET          = 0x0001;  // UE->Nav: reset scene
    constexpr uint16 STEP_ACK       = 0x0002;  // Nav->UE: frame completion ack (lockstep)
    constexpr uint16 SIM_END        = 0x0005;  // Nav->UE: simulation finished
    // ---- UE->Nav sensors ----
    constexpr uint16 LIDAR          = 0x0010;  // UE->Nav: LiDAR scan data
    constexpr uint16 GROUND_TRUTH  = 0x0011;  // UE->Nav: ground truth pose
    constexpr uint16 PED_STATE      = 0x0012;  // UE->Nav: pedestrian state
    constexpr uint16 COLLISION      = 0x0013;  // UE->Nav: collision event
    // ---- Nav->UE commands ----
    constexpr uint16 CMD_VEL        = 0x0020;  // Nav->UE: velocity command
    constexpr uint16 DEBUG_POSE     = 0x0021;  // Nav->UE: AMCL estimated pose
    constexpr uint16 DEBUG_PATH     = 0x0022;  // Nav->UE: A* path + particle cloud
}

// Velocity command (corresponds to bridge::CmdVelMsg; units: vx/vy in m/s, wz in rad/s)
USTRUCT(BlueprintType)
struct FPuppyCmdVel
{
    GENERATED_BODY()
    UPROPERTY(BlueprintReadWrite) float Vx = 0.0f;
    UPROPERTY(BlueprintReadWrite) float Vy = 0.0f;
    UPROPERTY(BlueprintReadWrite) float Wz = 0.0f;
};

// Triggered when CMD_VEL is received
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnCmdVelReceived, const FPuppyCmdVel&, CmdVel);
// Triggered when STEP_ACK is received (lockstep frame acknowledgement)
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnStepAck);
// Triggered when Nav ends the simulation (SIM_END received).
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnSimulationEnd);
// Triggered when an EMERGENCY_STOP arrives (caller must halt the robot).
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnEmergencyStop);

UCLASS(ClassGroup=(PuppyNav), meta=(BlueprintSpawnableComponent))
class PUPPYNAV_API UPuppyTcpServer : public UActorComponent
{
    GENERATED_BODY()
public:
    UPuppyTcpServer();

    // Listening port (default 7777, matches nav_ue_bridge --port)
    UPROPERTY(EditAnywhere, Category="Network")
    int32 Port = 7777;

    // Triggered when a velocity command is received
    UPROPERTY(BlueprintAssignable, Category="Network")
    FOnCmdVelReceived OnCmdVelReceived;

    // Triggered when a frame acknowledgement is received (lockstep sync)
    UPROPERTY(BlueprintAssignable, Category="Network")
    FOnStepAck OnStepAck;

    UPROPERTY(BlueprintAssignable, Category="Network")
    FOnSimulationEnd OnSimulationEnd;

    // Triggered on EMERGENCY_STOP (safe-stop request from Nav)
    UPROPERTY(BlueprintAssignable, Category="Network")
    FOnEmergencyStop OnEmergencyStop;

    // ===== Send messages (UE->Nav) =====

    // Sends LIDAR: distance array + max_range
    bool SendLidar(const TArray<float>& Distances, float MaxRange);

    // Sends GROUND_TRUTH: robot ground-truth pose (x, y in cm, yaw in radians)
    bool SendGroundTruth(double X, double Y, double Yaw);

    // Sends PED_STATE: pedestrian state array (interleaved as x, y, vx, vy)
    bool SendPedState(const TArray<float>& PedData);

    // Sends COLLISION: accumulated collision count + collision position
    bool SendCollision(int32 Count, double HitX, double HitY);

    // Sends SIM_END to Nav and waits (asynchronously) for SHUTDOWN_ACK.
    void SendSimEnd();

    // Whether TCP connection is available
    bool IsConnected() const {
        if (ClientSocket == nullptr) return false;
        ESocketConnectionState State = ClientSocket->GetConnectionState();
        return State == SCS_Connected;
    }

    // True only after the v2 handshake (HELLO/HELLO_ACK) has completed.
    // Sensor frames must not be sent before this is true.
    bool IsHandshaked() const { return bHandshaked_; }

    // True after an EMERGENCY_STOP arrived; motion should be halted.
    bool IsEmergencyStopped() const { return bEmergencyStop_; }

    // Clears the emergency-stop latch (e.g. after Nav resumes).
    void ResumeFromEmergencyStop() { bEmergencyStop_ = false; }

    // For debugging: return the raw ESocketConnectionState enum value (0=NotConnected,1=Connected,2=Error)
    int32 GetConnectionStateRaw() const {
        if (ClientSocket == nullptr) return -1;
        return static_cast<int32>(ClientSocket->GetConnectionState());
    }

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    // Connected client socket (we are the client of the bridge server)
    FSocket* ClientSocket = nullptr;

    // Receive buffer (accumulates partial data across ticks)
    TArray<uint8> RecvBuffer_;

    // ---- v2 protocol state ----
    bool     bHandshaked_    = false;  // HELLO/HELLO_ACK done
    bool     bHelloSent_     = false;  // HELLO dispatched to bridge
    uint32   TxSeq_          = 1;      // next outgoing sequence number
    double   LastSendTime_   = 0.0;    // wall clock of last send (heartbeat)
    bool     bEmergencyStop_ = false;  // EMERGENCY_STOP latch
    bool     bAwaitingShutdownAck_ = false; // sent SIM_END, waiting for ack

    // Processes the received byte stream, splits it into v2 frames and dispatches
    void ProcessIncomingData();

    // Packs and sends one v2 frame (16-byte header + payload)
    bool SendFrame(uint16 MsgType, const uint8* Payload, uint32 PayloadLen);

    // Sends the v2 HELLO handshake frame
    bool SendHello();

    // Handles one complete message (parses payload and fires delegates)
    void HandleMessage(uint16 MsgType, const uint8* Data, uint32 Len);
};
