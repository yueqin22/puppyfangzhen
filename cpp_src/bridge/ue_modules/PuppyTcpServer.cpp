// PuppyTcpServer.cpp -- UE5 TCP client component (talks to nav_ue_bridge)
// ==================================================================
// Connects (as CLIENT) to the nav_ue_bridge TCP SERVER on localhost:Port,
// performs the v2 handshake, then maintains a long-lived connection.
//
// Frame layout (v2, matches bridge::FrameHeader in protocol.h, 16 bytes):
//   4B length(LE) + 2B type + 2B version + 4B sequence + 4B timestamp_ms
//   followed by `length` bytes of payload.
//
// Receive (Nav->UE): RESET / CMD_VEL / DEBUG_POSE / DEBUG_PATH / STEP_ACK /
//                    HELLO_ACK / HEARTBEAT / EMERGENCY_STOP / SHUTDOWN_ACK /
//                    SIM_END / FAULT / STATUS
// Send    (UE->Nav): LIDAR / GROUND_TRUTH / PED_STATE / COLLISION / HELLO /
//                    HEARTBEAT / SHUTDOWN_ACK
//
// Lockstep 30Hz: UE sends one sensor frame then waits for STEP_ACK
// before advancing physics.
#include "PuppyTcpServer.h"

#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "HAL/PlatformTime.h"
#include "Serialization/ArrayWriter.h"
#include "Serialization/ArrayReader.h"

// Unit conversion: UE uses cm, protocol uses meters for geometry
static constexpr double CM_TO_M = 0.01;
static constexpr float  CM_TO_M_F = 0.01f;

// Idle heartbeat period (seconds). Bridge watchdog degrades at 300ms silence,
// so we must keepalive well under that when no sensor frame is sent.
constexpr double HEARTBEAT_INTERVAL_S = 0.2;

UPuppyTcpServer::UPuppyTcpServer()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UPuppyTcpServer::BeginPlay()
{
    Super::BeginPlay();

    RecvBuffer_.Reset();
    bHandshaked_    = false;
    bHelloSent_     = false;
    TxSeq_          = 1;
    bEmergencyStop_ = false;
    bAwaitingShutdownAck_ = false;
    LastSendTime_   = FPlatformTime::Seconds();

    // Client mode: connect to nav_ue_bridge (server on localhost:Port)
    ClientSocket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("PuppyTcpClient"), false);
    if (ClientSocket)
    {
        ClientSocket->SetNonBlocking(true);

        FIPv4Address IP(127, 0, 0, 1);
        TSharedRef<FInternetAddr> Addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        Addr->SetIp(IP.Value);
        Addr->SetPort(Port);

        bool bConnected = ClientSocket->Connect(*Addr);
        UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] Connecting to bridge on port %d (immediate=%s)"),
               Port, bConnected ? TEXT("yes") : TEXT("pending"));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] Failed to create client socket"));
    }
}

void UPuppyTcpServer::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Gracefully notify the bridge if we are still connected.
    if (ClientSocket && IsConnected())
    {
        SendSimEnd();
    }
    if (ClientSocket)
    {
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
        ClientSocket = nullptr;
    }

    Super::EndPlay(EndPlayReason);
}

void UPuppyTcpServer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!ClientSocket)
    {
        return;
    }

    // Check connection state (non-blocking connect may still be pending)
    ESocketConnectionState ConnState = ClientSocket->GetConnectionState();
    if (ConnState == SCS_ConnectionError)
    {
        // Connection failed -- retry once per second
        static double LastRetryTime = 0.0;
        double Now = FPlatformTime::Seconds();
        if (Now - LastRetryTime > 1.0)
        {
            LastRetryTime = Now;
            FIPv4Address IP(127, 0, 0, 1);
            TSharedRef<FInternetAddr> Addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
            Addr->SetIp(IP.Value);
            Addr->SetPort(Port);
            ClientSocket->Connect(*Addr);
            UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] Retrying connection to port %d..."), Port);
        }
        return;
    }

    // Not yet connected (pending)
    if (ConnState == SCS_NotConnected)
    {
        return;
    }

    // Connected -- drive the v2 handshake: send HELLO once, then wait for ACK.
    if (!bHandshaked_ && !bHelloSent_)
    {
        if (SendHello())
        {
            bHelloSent_ = true;
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] Failed to send HELLO"));
        }
    }

    // Drain pending data from the socket
    uint32 PendingData = 0;
    if (ClientSocket->HasPendingData(PendingData) && PendingData > 0)
    {
        TArray<uint8> TempBuffer;
        TempBuffer.SetNumUninitialized(static_cast<int32>(PendingData));
        int32 BytesRead = 0;
        if (ClientSocket->Recv(TempBuffer.GetData(), static_cast<int32>(PendingData), BytesRead) && BytesRead > 0)
        {
            RecvBuffer_.Append(TempBuffer.GetData(), BytesRead);
        }
    }

    ProcessIncomingData();

    // Idle keepalive: emit HEARTBEAT if we haven't sent anything recently.
    // (Under 30Hz lockstep this rarely fires, but it protects pause/idle states.)
    const double Now = FPlatformTime::Seconds();
    if (bHandshaked_ && (Now - LastSendTime_) > HEARTBEAT_INTERVAL_S)
    {
        if (SendFrame(PuppyMsgType::HEARTBEAT, nullptr, 0))
        {
            UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] HEARTBEAT (idle keepalive)"));
        }
    }
}

void UPuppyTcpServer::ProcessIncomingData()
{
    // Parse complete v2 frames (16-byte header + payload) from RecvBuffer_.
    constexpr int32 HeaderSize = 16;

    // R20 FIX: scan the whole buffer first; if there are multiple CMD_VEL/
    // STEP_ACK frames, only the LAST one should be processed (TCP backlog can
    // otherwise drive the robot with a stale command).
    int32 LastCmdVelOffset = -1;   // offset of the last CMD_VEL frame
    int32 LastStepAckOffset = -1;  // offset of the last STEP_ACK frame
    int32 ScanOffset = 0;

    while (ScanOffset + HeaderSize <= RecvBuffer_.Num())
    {
        uint32 PayloadLen = 0;
        uint16 MsgType = 0;
        uint16 Version = 0;
        FMemory::Memcpy(&PayloadLen, &RecvBuffer_[ScanOffset],        4);
        FMemory::Memcpy(&MsgType,    &RecvBuffer_[ScanOffset + 4],    2);
        FMemory::Memcpy(&Version,    &RecvBuffer_[ScanOffset + 6],    2);

        if (PayloadLen > 16 * 1024 * 1024)
        {
            UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] Implausible payload length %u, flushing buffer"), PayloadLen);
            RecvBuffer_.Reset();
            return;
        }

        if (ScanOffset + HeaderSize + static_cast<int32>(PayloadLen) > RecvBuffer_.Num())
        {
            break;  // Incomplete frame, wait for more data
        }

        if (Version != PUPPY_PROTOCOL_VERSION)
        {
            UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] Bad frame version %u (expected %u), skipping"),
                   Version, PUPPY_PROTOCOL_VERSION);
            ScanOffset += HeaderSize + static_cast<int32>(PayloadLen);
            continue;
        }

        if (MsgType == PuppyMsgType::CMD_VEL)
        {
            LastCmdVelOffset = ScanOffset;
        }
        else if (MsgType == PuppyMsgType::STEP_ACK)
        {
            LastStepAckOffset = ScanOffset;
        }

        ScanOffset += HeaderSize + static_cast<int32>(PayloadLen);
    }

    // Now process frames sequentially, but skip stale CMD_VEL/STEP_ACK.
    int32 ProcessOffset = 0;
    while (ProcessOffset + HeaderSize <= RecvBuffer_.Num())
    {
        uint32 PayloadLen = 0;
        uint16 MsgType = 0;
        uint16 Version = 0;
        FMemory::Memcpy(&PayloadLen, &RecvBuffer_[ProcessOffset],     4);
        FMemory::Memcpy(&MsgType,    &RecvBuffer_[ProcessOffset + 4], 2);
        FMemory::Memcpy(&Version,    &RecvBuffer_[ProcessOffset + 6], 2);

        if (ProcessOffset + HeaderSize + static_cast<int32>(PayloadLen) > RecvBuffer_.Num())
        {
            break;  // Incomplete frame
        }

        if (Version != PUPPY_PROTOCOL_VERSION)
        {
            ProcessOffset += HeaderSize + static_cast<int32>(PayloadLen);
            continue;
        }

        bool bSkip = false;
        if (!bHandshaked_)
        {
            // Before the handshake completes only HELLO_ACK is meaningful.
            if (MsgType != PuppyMsgType::HELLO_ACK) bSkip = true;
        }
        else
        {
            // R20 FIX: only process the last CMD_VEL / STEP_ACK in the buffer.
            if (MsgType == PuppyMsgType::CMD_VEL && ProcessOffset != LastCmdVelOffset)
            {
                bSkip = true;
            }
            else if (MsgType == PuppyMsgType::STEP_ACK && ProcessOffset != LastStepAckOffset)
            {
                bSkip = true;
            }
        }

        if (!bSkip)
        {
            const uint8* PayloadPtr = PayloadLen > 0 ? (RecvBuffer_.GetData() + ProcessOffset + HeaderSize) : nullptr;
            HandleMessage(MsgType, PayloadPtr, PayloadLen);
        }

        ProcessOffset += HeaderSize + static_cast<int32>(PayloadLen);
    }

    // Remove all processed frames from the buffer
    if (ProcessOffset > 0)
    {
        RecvBuffer_.RemoveAt(0, ProcessOffset);
    }
}

bool UPuppyTcpServer::SendFrame(uint16 MsgType, const uint8* Payload, uint32 PayloadLen)
{
    if (!ClientSocket)
    {
        return false;
    }

    // Pack the v2 16-byte header (length + type + version + sequence + timestamp)
    // then the payload.
    FArrayWriter Writer;
    Writer.SetNumUninitialized(static_cast<int32>(16 + PayloadLen));

    uint32 LengthField  = PayloadLen;
    uint16 TypeField    = MsgType;
    uint16 VersionField = PUPPY_PROTOCOL_VERSION;
    uint32 SeqField     = TxSeq_++;
    uint32 TsField      = static_cast<uint32>(FPlatformTime::Seconds() * 1000.0);

    int32 Offset = 0;
    FMemory::Memcpy(&Writer[Offset], &LengthField,  4); Offset += 4;
    FMemory::Memcpy(&Writer[Offset], &TypeField,    2); Offset += 2;
    FMemory::Memcpy(&Writer[Offset], &VersionField, 2); Offset += 2;
    FMemory::Memcpy(&Writer[Offset], &SeqField,     4); Offset += 4;
    FMemory::Memcpy(&Writer[Offset], &TsField,      4); Offset += 4;

    if (PayloadLen > 0 && Payload != nullptr)
    {
        FMemory::Memcpy(&Writer[Offset], Payload, PayloadLen);
    }

    int32 TotalSent = 0;
    while (TotalSent < Writer.Num())
    {
        int32 BytesSent = 0;
        const bool bOk = ClientSocket->Send(Writer.GetData() + TotalSent,
                                             Writer.Num() - TotalSent,
                                             BytesSent);
        if (!bOk || BytesSent <= 0)
        {
            UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] SendFrame failed type=0x%04x offset=%d/%d"),
                   MsgType, TotalSent, Writer.Num());
            return false;
        }
        TotalSent += BytesSent;
    }
    LastSendTime_ = FPlatformTime::Seconds();
    return true;
}

bool UPuppyTcpServer::SendHello()
{
    // HelloMsg wire layout (must match bridge::HelloMsg):
    //   uint16 magic; uint16 version; char capabilities[32];  => 36 bytes
    FArrayWriter W;
    W.SetNumUninitialized(36);

    uint16 Magic = PUPPY_PROTOCOL_MAGIC;
    uint16 Ver   = PUPPY_PROTOCOL_VERSION;
    char   Caps[32] = {0};
    const char* CapStr = "ue-puppy-v2";
    for (int32 i = 0; i < 31 && CapStr[i] != '\0'; ++i) Caps[i] = CapStr[i];

    int32 Off = 0;
    FMemory::Memcpy(&W[Off], &Magic, 2); Off += 2;
    FMemory::Memcpy(&W[Off], &Ver,   2); Off += 2;
    FMemory::Memcpy(&W[Off], Caps,   32);

    UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] Sending HELLO (protocol v%d)"), (int)Ver);
    return SendFrame(PuppyMsgType::HELLO, W.GetData(), 36);
}

bool UPuppyTcpServer::SendLidar(const TArray<float>& Distances, float MaxRange)
{
    if (!ClientSocket)
    {
        return false;
    }

    int32 NumRays = Distances.Num();

    // Payload = LidarMsg(n_rays + max_range) + n_rays * float distances
    // Convert cm -> m to match the protocol (Nav expects meters)
    FArrayWriter Writer;
    Writer.SetNumUninitialized(8 + NumRays * 4);

    int32 NumRaysField = NumRays;
    float  MaxRangeM    = MaxRange * CM_TO_M_F;

    int32 Offset = 0;
    FMemory::Memcpy(&Writer[Offset], &NumRaysField, 4); Offset += 4;
    FMemory::Memcpy(&Writer[Offset], &MaxRangeM,    4); Offset += 4;

    for (int32 i = 0; i < NumRays; ++i)
    {
        float DistM = Distances[i] * CM_TO_M_F;
        FMemory::Memcpy(&Writer[Offset], &DistM, 4);
        Offset += 4;
    }

    return SendFrame(PuppyMsgType::LIDAR, Writer.GetData(), Writer.Num());
}

bool UPuppyTcpServer::SendGroundTruth(double X, double Y, double Yaw)
{
    if (!ClientSocket)
    {
        return false;
    }

    // Convert cm -> m; yaw is already in radians
    double Xm   = X   * CM_TO_M;
    double Ym   = Y   * CM_TO_M;
    double YawR = Yaw;

    FArrayWriter Writer;
    Writer.SetNumUninitialized(24);

    int32 Offset = 0;
    FMemory::Memcpy(&Writer[Offset], &Xm,   8); Offset += 8;
    FMemory::Memcpy(&Writer[Offset], &Ym,   8); Offset += 8;
    FMemory::Memcpy(&Writer[Offset], &YawR, 8);

    return SendFrame(PuppyMsgType::GROUND_TRUTH, Writer.GetData(), Writer.Num());
}

bool UPuppyTcpServer::SendPedState(const TArray<float>& PedData)
{
    if (!ClientSocket)
    {
        return false;
    }

    // PedData is interleaved [x0, y0, vx0, vy0, x1, y1, vx1, vy1, ...] in cm / cm/s.
    // Convert to m / m/s for the protocol.
    int32 NumPeds = PedData.Num() / 4;

    FArrayWriter Writer;
    Writer.SetNumUninitialized(4 + NumPeds * 16);

    int32 Offset = 0;
    FMemory::Memcpy(&Writer[Offset], &NumPeds, 4); Offset += 4;

    for (int32 i = 0; i < NumPeds; ++i)
    {
        float Xm  = PedData[i * 4 + 0] * CM_TO_M_F;
        float Ym  = PedData[i * 4 + 1] * CM_TO_M_F;
        float Vxm = PedData[i * 4 + 2] * CM_TO_M_F;
        float Vym = PedData[i * 4 + 3] * CM_TO_M_F;

        FMemory::Memcpy(&Writer[Offset], &Xm,  4); Offset += 4;
        FMemory::Memcpy(&Writer[Offset], &Ym,  4); Offset += 4;
        FMemory::Memcpy(&Writer[Offset], &Vxm, 4); Offset += 4;
        FMemory::Memcpy(&Writer[Offset], &Vym, 4); Offset += 4;
    }

    return SendFrame(PuppyMsgType::PED_STATE, Writer.GetData(), Writer.Num());
}

bool UPuppyTcpServer::SendCollision(int32 Count, double HitX, double HitY)
{
    if (!ClientSocket)
    {
        return false;
    }

    // Convert hit position cm -> m
    double HitXm = HitX * CM_TO_M;
    double HitYm = HitY * CM_TO_M;

    FArrayWriter Writer;
    Writer.SetNumUninitialized(20);

    int32 Offset = 0;
    FMemory::Memcpy(&Writer[Offset], &Count, 4);  Offset += 4;
    FMemory::Memcpy(&Writer[Offset], &HitXm, 8); Offset += 8;
    FMemory::Memcpy(&Writer[Offset], &HitYm, 8);

    return SendFrame(PuppyMsgType::COLLISION, Writer.GetData(), Writer.Num());
}

void UPuppyTcpServer::SendSimEnd()
{
    if (!ClientSocket || !IsConnected())
    {
        return;
    }
    UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] Sending SIM_END"));
    if (SendFrame(PuppyMsgType::SIM_END, nullptr, 0))
    {
        bAwaitingShutdownAck_ = true;
    }
}

void UPuppyTcpServer::HandleMessage(uint16 MsgType, const uint8* Data, uint32 Len)
{
    if (MsgType == PuppyMsgType::CMD_VEL)
    {
        // Parse 3 floats (vx, vy, wz) in m/s and rad/s
        if (Data == nullptr || Len < sizeof(float) * 3)
        {
            return;
        }

        FArrayReader Reader;
        Reader.SetNumUninitialized(static_cast<int32>(Len));
        FMemory::Memcpy(Reader.GetData(), Data, Len);

        float Vx = 0.0f, Vy = 0.0f, Wz = 0.0f;
        Reader << Vx;
        Reader << Vy;
        Reader << Wz;

        FPuppyCmdVel CmdVel;
        CmdVel.Vx = Vx;
        CmdVel.Vy = Vy;
        CmdVel.Wz = Wz;

        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] CMD_VEL vx=%.3f vy=%.3f wz=%.3f"), Vx, Vy, Wz);
        OnCmdVelReceived.Broadcast(CmdVel);
    }
    else if (MsgType == PuppyMsgType::STEP_ACK)
    {
        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] STEP_ACK received"));
        OnStepAck.Broadcast();
    }
    else if (MsgType == PuppyMsgType::HELLO_ACK)
    {
        // v2 handshake confirmation from the bridge.
        if (Data != nullptr && Len >= 6)
        {
            uint16 Magic  = 0;
            uint16 Ver    = 0;
            uint16 Status = 0;
            FMemory::Memcpy(&Magic,  Data + 0, 2);
            FMemory::Memcpy(&Ver,    Data + 2, 2);
            FMemory::Memcpy(&Status, Data + 4, 2);
            if (Magic == PUPPY_PROTOCOL_MAGIC && Status == 0)
            {
                bHandshaked_ = true;
                UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] Handshake OK (protocol v%d)"), (int)Ver);
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] Handshake REJECTED (magic=0x%04x status=%d)"), Magic, Status);
            }
        }
    }
    else if (MsgType == PuppyMsgType::HEARTBEAT)
    {
        // Keepalive from Nav; nothing to act on at the UE side.
        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] HEARTBEAT received"));
    }
    else if (MsgType == PuppyMsgType::EMERGENCY_STOP)
    {
        UE_LOG(LogTemp, Warning, TEXT("[PuppyTcpServer] EMERGENCY_STOP received -> safe stop"));
        bEmergencyStop_ = true;
        OnEmergencyStop.Broadcast();
    }
    else if (MsgType == PuppyMsgType::SHUTDOWN_ACK)
    {
        UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] SHUTDOWN_ACK received"));
        bAwaitingShutdownAck_ = false;
    }
    else if (MsgType == PuppyMsgType::FAULT)
    {
        UE_LOG(LogTemp, Error, TEXT("[PuppyTcpServer] FAULT received from Nav -> safe stop"));
        bEmergencyStop_ = true;
        OnEmergencyStop.Broadcast();
    }
    else if (MsgType == PuppyMsgType::STATUS)
    {
        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] STATUS received (%u bytes)"), Len);
    }
    else if (MsgType == PuppyMsgType::SIM_END)
    {
        UE_LOG(LogTemp, Log, TEXT("[PuppyTcpServer] SIM_END received -> ack + end"));
        // Reply SHUTDOWN_ACK so the bridge can exit gracefully, then notify.
        SendFrame(PuppyMsgType::SHUTDOWN_ACK, nullptr, 0);
        OnSimulationEnd.Broadcast();
    }
    else if (MsgType == PuppyMsgType::DEBUG_POSE)
    {
        // AMCL estimated pose + confidence -- used for debug visualization
        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] DEBUG_POSE received (%u bytes)"), Len);
    }
    else if (MsgType == PuppyMsgType::DEBUG_PATH)
    {
        // A* path + particle cloud -- used for debug visualization
        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] DEBUG_PATH received (%u bytes)"), Len);
    }
    else
    {
        UE_LOG(LogTemp, Verbose, TEXT("[PuppyTcpServer] Unknown message type 0x%04x (%u bytes)"), MsgType, Len);
    }
}
