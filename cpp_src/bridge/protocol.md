# UE-Bridge TCP Protocol (v2, P0-03, 2026-08-16)

> **变更记录 (v1 → v2)**: 帧头由 8 字节升级为 16 字节，新增 `protocol version` /
> `sequence` / `sender-timestamp` 字段；新增握手（HELLO/HELLO_ACK）、心跳
> （HEARTBEAT）、紧急停车（EMERGENCY_STOP）、结束确认（SHUTDOWN_ACK）、故障
> （FAULT）、状态（STATUS）等控制消息；全部收发统一经 `bridge::Link`，序列号由
> Link 自动分配与校验；接收支持带超时（select），作为断线看门狗基础。传感/控制
> 消息类型与 v1 兼容，仅新增上述握手类消息。

## Overview

Binary TCP protocol between `nav_ue_bridge.exe` (server) and UE5 `PuppyTcpServer` (client).
Lockstep 30Hz: UE sends sensor data -> bridge processes -> bridge sends command -> UE advances frame.
Bridge is the TCP **server** (binds first); UE pawn is the TCP **client** (connects at BeginPlay).

## Connection

- Transport: localhost TCP, port 7777 (configurable via `--port`)
- Bridge is TCP server, UE is TCP client
- Single connection, no multiplexing
- Endianness: little-endian (x86 native)

## Frame Format (v2, 16 bytes)

Every message is prefixed by a 16-byte header (`bridge::FrameHeader`, `#pragma pack(1)`):

```
Offset  Size  Field         Description
0       4     length        Payload bytes (excluding header), uint32 LE
4       2     type          MsgType enum value, uint16 LE
6       2     version       PROTOCOL_VERSION (must be 2)
8       4     sequence      Per-direction monotonic sequence number (Link assigns)
12      4     timestamp_ms  Sender wall-clock timestamp (ms)
16      N     payload       N = length bytes
```

The receiver (`bridge::Link::recv_timeout`) rejects any frame whose `version != 2`
with `RECV_CLOSED`, so a v1 peer can never silently desync the stream.

## Message Types

### Handshake / control (v2)

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| HELLO | 0x0006 | UE->Nav | Connect: magic(0x5050) + version + capabilities |
| HELLO_ACK | 0x0007 | Nav->UE | Handshake confirm: status (0=OK, 1=version mismatch) |
| RESUME | 0x0008 | Nav->UE | Resume after pause |
| HEARTBEAT | 0x0009 | Either | Keepalive; resets silence window on receiver |
| EMERGENCY_STOP | 0x000A | Either | Safe stop; receiver halts motion immediately |
| SHUTDOWN_ACK | 0x000B | UE->Nav | Ack of SIM_END (graceful shutdown) |
| FAULT | 0x000C | Either | Protocol/state fault; receiver safe-stops & exits |
| STATUS | 0x000D | Either | Diagnostics / state report |

### Control (bidirectional)

| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| RESET | 0x0001 | UE->Nav | Reset scene with seed + initial pose |
| STEP_ACK | 0x0002 | Nav->UE | Frame complete, advance to next |
| PAUSE | 0x0003 | Bidirectional | Pause/resume simulation |
| SET_SPEED | 0x0004 | UE->Nav | Set simulation speed multiplier |
| SIM_END | 0x0005 | Nav->UE | Simulation finished; UE replies SHUTDOWN_ACK |

### UE -> Nav (sensor data)

| Type | Value | Description |
|------|-------|-------------|
| LIDAR | 0x0010 | 72-ray LiDAR scan distances |
| GROUND_TRUTH | 0x0011 | True robot pose (x, y, yaw) for evaluation |
| PED_STATE | 0x0012 | Pedestrian positions and velocities |
| COLLISION | 0x0013 | Collision event with hit position |

### Nav -> UE (control + debug)

| Type | Value | Description |
|------|-------|-------------|
| CMD_VEL | 0x0020 | Velocity command (vx, vy, wz) |
| DEBUG_POSE | 0x0021 | AMCL estimated pose + confidence |
| DEBUG_PATH | 0x0022 | A* path polyline + AMCL particle cloud |

## Payload Structures

All structs are `#pragma pack(push, 1)` (no padding).

### ResetMsg (96 bytes)
```c
struct ResetMsg {
    int32_t  seed;          // Random seed
    char     scene_name[64]; // Scene name (e.g. "home")
    double   init_x;        // Initial X (meters)
    double   init_y;        // Initial Y (meters)
    double   init_yaw;      // Initial yaw (radians)
};
```

### LidarMsg (8 bytes + n_rays * 4 bytes)
```c
struct LidarMsg {
    int32_t  n_rays;     // Ray count (typically 72)
    float    max_range;   // Max range (8.0m), misses = max_range
};
// Followed by n_rays * float distance values (meters)
```

### GroundTruthMsg (24 bytes)
```c
struct GroundTruthMsg {
    double x;    // X position (meters)
    double y;    // Y position (meters)
    double yaw;  // Heading (radians)
};
```

### PedStateMsg (4 bytes + n_peds * 16 bytes)
```c
struct PedStateMsg {
    int32_t n_peds;   // Pedestrian count (typically 5)
};
// Followed by n_peds * PedData
struct PedData {
    float x, y;     // Position (meters)
    float vx, vy;   // Velocity (m/s)
};
```

### CollisionMsg (24 bytes)
```c
struct CollisionMsg {
    int32_t  collision_count;  // Cumulative collision count
    double   hit_x;            // Collision X (meters)
    double   hit_y;            // Collision Y (meters)
};
```

### CmdVelMsg (12 bytes)
```c
struct CmdVelMsg {
    float vx;   // Linear velocity X (m/s)
    float vy;   // Linear velocity Y (m/s)
    float wz;   // Angular velocity (rad/s)
};
```

### DebugPoseMsg (28 bytes)
```c
struct DebugPoseMsg {
    double est_x;        // AMCL estimated X (meters)
    double est_y;        // AMCL estimated Y (meters)
    double est_yaw;      // AMCL estimated yaw (radians)
    float  confidence;   // AMCL confidence [0, 1]
};
```

### DebugPathMsg (8 bytes + variable)
```c
struct DebugPathMsg {
    int32_t n_path_points;   // Path points (max 256)
    int32_t n_particles;     // Particle samples (max 200)
};
// Followed by n_path_points * PathPoint (8 bytes each)
// Then n_particles * ParticleSample (12 bytes each)
struct PathPoint { float x, y; };
struct ParticleSample { float x, y, yaw; };
```

## Lockstep Flow

```
    UE5                              nav_ue_bridge
    |                                     |
    |-------- RESET (seed, init_pose) --> |
    |                                     | (init AMCL, load scene)
    |<-- (bridge ready, waiting) -------|
    |                                     |
    |--- LIDAR (72 distances) ---------->|
    |--- GROUND_TRUTH (x,y,yaw) ------->|
    |--- PED_STATE (5 peds) ------------>| (optional, not every frame)
    |                                     | (AMCL update + A* plan + CBF)
    |<--- CMD_VEL (vx,vy,wz) ------------|
    |<--- DEBUG_POSE (est,conf) ---------| (optional)
    |<--- DEBUG_PATH (path,particles) ---| (optional)
    |<--- STEP_ACK --------------------->|
    |                                     |
    |   (UE advances physics frame)        |
    |                                     |
    |--- LIDAR (next frame) ------------>|
    |   ... (repeat) ...                  |
```

## Unit Conversions

UE5 uses centimeters internally. The bridge protocol uses meters.

| Field | UE internal | Protocol | Conversion |
|-------|------------|----------|------------|
| Position | cm | m | divide by 100 |
| Velocity | cm/s | m/s | divide by 100 |
| Angular velocity | deg/s | rad/s | multiply by pi/180 |
| LiDAR distance | cm | m | divide by 100 |
| Yaw | degrees | radians | multiply by pi/180 |

## Command Line

```
nav_ue_bridge.exe [options]
  --port N        TCP port (default 7777)
  --frames N      Max frames before auto-stop (0 = unlimited)
  --seed N        Random seed for standalone mode
  --scene PATH    Scene JSON file path
  --standalone    Run without UE (C++ sensor model self-test)
```

## Standalone Mode

`--standalone` runs the bridge without UE, using the C++ sensor model (`simulate_lidar`)
for self-testing the navigation logic (AMCL + A* + path following + RVO safety).

Output format (same as sim_test):
```
SUMMARY: collisions=N astar_rate=%.1f avg_err=%.3f max_err=%.3f rooms=N rounds=N confidence=%.3f frames=N
```

## Handshake & Watchdog (v2)

1. **Connect**: UE (client) connects to the bridge (server) on BeginPlay.
2. **HELLO**: UE sends `HELLO` (magic `0x5050`, version `2`, capabilities string).
   Bridge `handshake_server` reads it; if `version != 2` it replies `HELLO_ACK`
   with `status=1` and enters `FAULT`. Otherwise `status=0`, enters `RUNNING`.
3. **HELLO_ACK**: UE `handshake_client` validates `status==0`; only then does it
   process normal traffic and start sending sensor frames.
4. **Heartbeat**: when UE is idle (no sensor frame for >200ms), it sends
   `HEARTBEAT` so the bridge watchdog never trips during pause/quiet states.
5. **Watchdog (§6.4)**: bridge tracks silence since last received frame —
   100ms no data → no advance; 300ms → `EMERGENCY_STOP` + `DEGRADED`;
   500ms → `DEGRADED`; 2000ms → `FAULT` + release link (断线停车 ≤300ms).

## Shutdown

- **Bridge-initiated**: bridge sends `SIM_END` → UE replies `SHUTDOWN_ACK` →
  bridge exits. UE also halts on `EMERGENCY_STOP`/`FAULT`.
- **UE-initiated**: UE sends `SIM_END` (e.g. editor shutdown) → bridge replies
  `SHUTDOWN_ACK` and exits. UE `EndPlay` also calls `SendSimEnd()`.

## Error Handling

- **Connection drop**: bridge `recv_timeout` returns `RECV_CLOSED` → safe-stop &
  exit; UE detects socket error → stops sending and **zeroes velocity (断线停车)**.
- **Version mismatch / wrong version frame**: receiver rejects with `RECV_CLOSED`
  (bridge) or skips the frame (UE); handshake enters `FAULT`.
- **Half/partial frame + disconnect**: `recv_timeout` reads header/payload in
  select-gated segments so it can never block past the timeout — a truncated
  frame followed by a close returns `RECV_CLOSED` promptly.
- **Sequence check** (`bridge::check_rx_seq`): duplicate frame → ignored (no
  double-advance); gap/late frame → dropped + `protocol_errors++`. Applied by
  the bridge after `recv_timeout` (so `recv_timeout` itself stays seq-agnostic).
- **Timeout**: every receive is deadline-bounded via `select`; no unbounded block.
