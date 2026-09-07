#pragma once

// =============================================================================
// UE-Bridge TCP Protocol  --  v2 (P0-03, 2026-08-16)
// -----------------------------------------------------------------------------
// 相对 v1 的变更（详见 protocol.md §变更记录）：
//   * 帧头加入 protocol version / sequence / sender-timestamp
//   * 新增握手（HELLO/HELLO_ACK）、心跳（HEARTBEAT）、紧急停车
//     （EMERGENCY_STOP）、结束确认（SHUTDOWN_ACK）、故障（FAULT）、状态（STATUS）
//   * 所有收发统一经 bridge::Link，序列号由 Link 自动分配与校验
//   * 接收支持带超时（select），作为断线看门狗的基础
// 保留 v1 的传感/控制消息类型与载荷布局，仅新增握手类消息。
// =============================================================================

#include <cstdint>
#include <vector>
#include <cstring>
#include <chrono>
#include <cstdio>
#include <algorithm>
#include <tuple>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <sys/select.h>
#  include <unistd.h>
#  include <cerrno>

using SOCKET = int;
constexpr int INVALID_SOCKET = -1;
constexpr int SOCKET_ERROR = -1;
inline int closesocket(int s) { return ::close(s); }
inline int WSAGetLastError() { return errno; }
#endif

namespace bridge {

constexpr uint16_t PROTOCOL_VERSION = 2;
constexpr uint16_t PROTOCOL_MAGIC   = 0x5050;   // "PP" —— 握手自描述标记

// ===== 消息类型 =====
enum class MsgType : uint16_t {
    // ---- 握手类 ----
    HELLO        = 0x0006,   // 建链：携带版本 + 能力串
    HELLO_ACK    = 0x0007,   // 建链确认：携带协商版本 + 状态
    RESUME       = 0x0008,   // 恢复运行
    HEARTBEAT    = 0x0009,   // 心跳（空闲/暂停时保活）
    EMERGENCY_STOP = 0x000A, // 紧急停车（任意一方发出）
    SHUTDOWN_ACK = 0x000B,   // 收到 SIM_END 后回确认
    FAULT        = 0x000C,   // 协议/状态故障
    STATUS       = 0x000D,   // 状态上报

    // ---- 控制（双向）----
    RESET        = 0x0001,   // UE→Nav: 重置场景
    STEP_ACK     = 0x0002,   // Nav→UE: 帧完成确认（lockstep 推进）
    PAUSE        = 0x0003,   // 双向: 暂停/恢复
    SET_SPEED    = 0x0004,   // UE→Nav: 设置倍速
    SIM_END      = 0x0005,   // Nav→UE: 仿真结束

    // ---- UE → Nav (传感数据) ----
    LIDAR        = 0x0010,   // LiDAR 扫描数据
    GROUND_TRUTH = 0x0011,   // 真值位姿 (供评估, AMCL不消费)
    PED_STATE    = 0x0012,   // 行人状态
    COLLISION    = 0x0013,   // 碰撞事件

    // ---- Nav → UE (控制指令) ----
    CMD_VEL      = 0x0020,   // 速度指令
    DEBUG_POSE   = 0x0021,   // AMCL 估计位姿 + 置信度
    DEBUG_PATH   = 0x0022,   // A* 路径 + 粒子云
};

// ===== 帧头 (v2, 16 字节) =====
#pragma pack(push, 1)
struct FrameHeader {
    uint32_t length;        // payload 字节数 (不含 header)
    uint16_t type;          // MsgType 值
    uint16_t version;       // PROTOCOL_VERSION
    uint32_t sequence;      // 单方向递增序列号（每发一帧 +1）
    uint32_t timestamp_ms;  // 发送方单调时钟 (ms)
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 16, "FrameHeader v2 must be 16 bytes");

// ===== 握手载荷 =====
#pragma pack(push, 1)
struct HelloMsg {
    uint16_t magic;                 // PROTOCOL_MAGIC
    uint16_t version;               // PROTOCOL_VERSION
    char     capabilities[32];      // 例如 "nav-core-v2"
};

struct HelloAckMsg {
    uint16_t magic;                 // PROTOCOL_MAGIC
    uint16_t version;               // 协商版本
    uint16_t status;                // 0=OK, 1=版本不符, 2=不兼容
    char     server_info[30];       // 服务端信息
};
#pragma pack(pop)

// ===== 消息载荷结构（与 v1 兼容，未改动布局）=====
#pragma pack(push, 1)

struct ResetMsg {
    int32_t  seed;
    char     scene_name[64];
    double   init_x;
    double   init_y;
    double   init_yaw;
};

struct LidarMsg {
    int32_t  n_rays;
    float    max_range;
};

struct GroundTruthMsg {
    double x;
    double y;
    double yaw;
};

struct PedStateMsg {
    int32_t n_peds;
};

struct PedData {
    float x, y, vx, vy;
};

struct CollisionMsg {
    int32_t  collision_count;
    double   hit_x;
    double   hit_y;
};

struct CmdVelMsg {
    float vx;
    float vy;
    float wz;
};

struct DebugPoseMsg {
    double est_x;
    double est_y;
    double est_yaw;
    float  confidence;
};

struct DebugPathMsg {
    int32_t n_path_points;
    int32_t n_particles;
};

struct PathPoint {
    float x, y;
};

struct ParticleSample {
    float x, y, yaw;
};

#pragma pack(pop)

// ===== 连接状态机 =====
enum class ConnState : int {
    CONNECTING = 0,
    HANDSHAKE  = 1,
    RUNNING    = 2,
    PAUSED     = 3,
    DEGRADED   = 4,
    FAULT      = 5,
    CLOSED     = 6,
};

// ===== 协议运行时状态（每连接一个）=====
struct ProtocolState {
    uint32_t next_tx_seq = 1;   // 下一个待分配的出向序列号
    uint32_t last_rx_seq = 0;   // 最近一次接受的入向序列号（0=尚未收到）
    bool     handshaked  = false;
    int64_t  last_rx_ms  = 0;   // 最近一次成功接收的墙钟时间
    int64_t  last_tx_ms  = 0;
    ConnState conn_state = ConnState::CONNECTING;
    int      protocol_errors = 0;   // 丢弃/非法帧计数
    int      duplicate_frames = 0;  // 重复帧计数（不二次推进）
    bool     safe_stop   = false;   // 看门狗已触发停车
};

// 序列号校验结果
enum class SeqCheck {
    OK       = 0,  // 接受并处理
    DUPLICATE_FRAME = 1,  // 重复帧，忽略（不二次推进）
    DROP     = 2,  // 缺口/乱序/过期，丢弃并记录协议错误
};

inline int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 分配下一个出向序列号
inline uint32_t alloc_tx_seq(ProtocolState& ps) { return ps.next_tx_seq++; }

// 校验入向序列号（lockstep 下期望严格递增）
inline SeqCheck check_rx_seq(ProtocolState& ps, uint32_t seq) {
    if (ps.last_rx_seq == 0) { ps.last_rx_seq = seq; return SeqCheck::OK; }
    if (seq == ps.last_rx_seq) { ps.duplicate_frames++; return SeqCheck::DUPLICATE_FRAME; }
    if (seq > ps.last_rx_seq) { ps.last_rx_seq = seq; return SeqCheck::OK; }
    ps.protocol_errors++;
    return SeqCheck::DROP;  // 过期/乱序
}

// =============================================================================
// Link —— 单一收发通道（socket + 协议状态），所有帧 I/O 经由此对象
// =============================================================================
struct Link {
    int      sock = -1;
    ProtocolState ps;

    Link() = default;
    explicit Link(int s) : sock(s) {}

    bool valid() const { return sock >= 0; }

    // ---- 原始收发（带超时）----
    bool send_all_raw(const char* data, int len) const {
        int sent_total = 0;
        while (sent_total < len) {
#ifdef _WIN32
            int n = ::send(sock, data + sent_total, len - sent_total, 0);
#else
            int n = ::send(sock, data + sent_total, len - sent_total, 0);
#endif
            if (n <= 0) return false;
            sent_total += n;
        }
        return true;
    }

    bool recv_all_raw(char* buf, int len) const {
        int received = 0;
        while (received < len) {
#ifdef _WIN32
            int n = ::recv(sock, buf + received, len - received, 0);
#else
            int n = ::recv(sock, buf + received, len - received, 0);
#endif
            if (n <= 0) return false;
            received += n;
        }
        return true;
    }

    // ---- 帧发送（自动填入 version/seq/timestamp）----
    bool send(MsgType type, const void* payload, uint32_t payload_len) {
        FrameHeader hdr{};
        hdr.length      = payload_len;
        hdr.type        = static_cast<uint16_t>(type);
        hdr.version     = PROTOCOL_VERSION;
        hdr.sequence    = alloc_tx_seq(ps);
        hdr.timestamp_ms= static_cast<uint32_t>(now_ms());
        ps.last_tx_ms   = hdr.timestamp_ms;
        if (!send_all_raw(reinterpret_cast<const char*>(&hdr), static_cast<int>(sizeof(hdr))))
            return false;
        if (payload_len > 0 &&
            !send_all_raw(reinterpret_cast<const char*>(payload), static_cast<int>(payload_len)))
            return false;
        return true;
    }

    enum RecvStatus { RECV_OK, RECV_TIMEOUT, RECV_CLOSED };

    // ---- 帧接收（带超时；用于看门狗）----
    // 鲁棒读取：对 header 与 payload 逐段 select 门控，绝不超出 timeout_ms 阻塞。
    // 这样半包/粘包/对端停滞都不会导致无限阻塞（看门狗仍可按时触发）。
    RecvStatus recv_timeout(FrameHeader& hdr, std::vector<char>& payload, int timeout_ms) {
        payload.clear();
        const int64_t deadline = now_ms() + timeout_ms;

        auto wait_readable = [&](int64_t rem_ms) -> bool {
            if (rem_ms <= 0) return false;
            fd_set fds;
            FD_ZERO(&fds);
#ifdef _WIN32
            FD_SET(static_cast<SOCKET>(sock), &fds);
            timeval tv{ static_cast<long>(rem_ms / 1000),
                        static_cast<long>((rem_ms % 1000) * 1000) };
            int r = ::select(0, &fds, nullptr, nullptr, &tv);
#else
            FD_SET(sock, &fds);
            timeval tv{ static_cast<time_t>(rem_ms / 1000),
                        static_cast<suseconds_t>((rem_ms % 1000) * 1000) };
            int r = ::select(sock + 1, &fds, nullptr, nullptr, &tv);
#endif
            return r > 0;
        };

        char* hbuf = reinterpret_cast<char*>(&hdr);
        size_t hgot = 0;
        while (hgot < sizeof(hdr)) {
            if (!wait_readable(deadline - now_ms())) return RECV_TIMEOUT;
            int n = ::recv(sock, hbuf + hgot, static_cast<int>(sizeof(hdr) - hgot), 0);
            if (n <= 0) return RECV_CLOSED;
            hgot += static_cast<size_t>(n);
        }
        if (hdr.version != PROTOCOL_VERSION) return RECV_CLOSED;

        payload.resize(hdr.length);
        size_t pgot = 0;
        while (pgot < hdr.length) {
            if (!wait_readable(deadline - now_ms())) return RECV_TIMEOUT;
            int n = ::recv(sock, payload.data() + pgot,
                           static_cast<int>(hdr.length - pgot), 0);
            if (n <= 0) return RECV_CLOSED;
            pgot += static_cast<size_t>(n);
        }
        ps.last_rx_ms = now_ms();
        return RECV_OK;
    }

    // 阻塞接收（无超时）—— 用于握手阶段前的确定性读取
    bool recv(FrameHeader& hdr, std::vector<char>& payload) {
        payload.clear();
        if (!recv_all_raw(reinterpret_cast<char*>(&hdr), static_cast<int>(sizeof(hdr))))
            return false;
        if (hdr.version != PROTOCOL_VERSION) return false;
        payload.resize(hdr.length);
        if (hdr.length > 0 &&
            !recv_all_raw(payload.data(), static_cast<int>(hdr.length)))
            return false;
        ps.last_rx_ms = now_ms();
        return true;
    }

    // ---- 握手：服务端（Nav/Bridge 侧）----
    bool handshake_server(const char* caps, int timeout_ms = 5000) {
        ps.conn_state = ConnState::HANDSHAKE;
        FrameHeader hdr;
        std::vector<char> pl;
        RecvStatus st = recv_timeout(hdr, pl, timeout_ms);
        if (st != RECV_OK || static_cast<MsgType>(hdr.type) != MsgType::HELLO) {
            // §5.1: 握手失败必须留下可追溯记录, 否则"版本不匹配"是近似静默的失败
            std::printf("[HELLO] FAILED peer did not send HELLO "
                        "(recv_status=%d, type=0x%04x, timeout=%dms)\n",
                        (int)st, hdr.type, timeout_ms);
            std::fflush(stdout);
            ps.conn_state = ConnState::FAULT;
            return false;
        }
        HelloMsg hello{};
        if (pl.size() >= sizeof(hello)) std::memcpy(&hello, pl.data(), sizeof(hello));

        // §5.1: 记录收到的 HELLO (magic/版本/能力串/载荷长度)
        char caps_buf[33] = {0};
        std::memcpy(caps_buf, hello.capabilities, sizeof(hello.capabilities));
        caps_buf[32] = '\0';
        std::printf("[HELLO] recv magic=0x%04x version=%u caps=\"%s\" payload=%zu\n",
                    hello.magic, hello.version, caps_buf, pl.size());
        std::fflush(stdout);

        HelloAckMsg ack{};
        ack.magic   = PROTOCOL_MAGIC;
        ack.version = PROTOCOL_VERSION;
        if (hello.magic != PROTOCOL_MAGIC || hello.version != PROTOCOL_VERSION) {
            ack.status = 1;  // 版本不符
            // §5.1: 明确记录不匹配的具体原因, 便于定位 UE 侧协议版本
            std::printf("[HELLO_ACK] REJECT local(magic=0x%04x,version=%u) != "
                        "peer(magic=0x%04x,version=%u)\n",
                        PROTOCOL_MAGIC, PROTOCOL_VERSION, hello.magic, hello.version);
            std::fflush(stdout);
            send(MsgType::HELLO_ACK, &ack, sizeof(ack));
            ps.conn_state = ConnState::FAULT;
            return false;
        }
        ack.status = 0;
        std::snprintf(ack.server_info, sizeof(ack.server_info), "%s", caps);
        if (!send(MsgType::HELLO_ACK, &ack, sizeof(ack))) {
            std::printf("[HELLO_ACK] SEND FAILED (caps=\"%s\")\n", caps);
            std::fflush(stdout);
            ps.conn_state = ConnState::FAULT;
            return false;
        }
        std::printf("[HELLO_ACK] sent status=0 version=%u server_info=\"%s\"\n",
                    ack.version, caps);
        std::fflush(stdout);
        ps.handshaked = true;
        ps.conn_state = ConnState::RUNNING;
        return true;
    }

    // ---- 握手：客户端（UE 侧）----
    bool handshake_client(const char* caps, int timeout_ms = 5000) {
        ps.conn_state = ConnState::HANDSHAKE;
        HelloMsg hello{};
        hello.magic   = PROTOCOL_MAGIC;
        hello.version = PROTOCOL_VERSION;
        std::snprintf(hello.capabilities, sizeof(hello.capabilities), "%s", caps);
        if (!send(MsgType::HELLO, &hello, sizeof(hello))) {
            ps.conn_state = ConnState::FAULT;
            return false;
        }
        FrameHeader hdr;
        std::vector<char> pl;
        RecvStatus st = recv_timeout(hdr, pl, timeout_ms);
        if (st != RECV_OK || static_cast<MsgType>(hdr.type) != MsgType::HELLO_ACK) {
            ps.conn_state = ConnState::FAULT;
            return false;
        }
        HelloAckMsg ack{};
        if (pl.size() >= sizeof(ack)) std::memcpy(&ack, pl.data(), sizeof(ack));
        if (ack.magic != PROTOCOL_MAGIC || ack.status != 0) {
            ps.conn_state = ConnState::FAULT;
            return false;
        }
        ps.handshaked = true;
        ps.conn_state = ConnState::RUNNING;
        return true;
    }

    // ---- 便捷发送 ----
    bool send_cmd_vel(float vx, float vy, float wz) {
        CmdVelMsg m{vx, vy, wz};
        return send(MsgType::CMD_VEL, &m, sizeof(m));
    }
    bool send_step_ack()            { return send(MsgType::STEP_ACK, nullptr, 0); }
    bool send_sim_end()             { return send(MsgType::SIM_END, nullptr, 0); }
    bool send_heartbeat()           { return send(MsgType::HEARTBEAT, nullptr, 0); }
    bool send_emergency_stop()      { return send(MsgType::EMERGENCY_STOP, nullptr, 0); }
    bool send_shutdown_ack()        { return send(MsgType::SHUTDOWN_ACK, nullptr, 0); }
    bool send_fault()               { return send(MsgType::FAULT, nullptr, 0); }
    bool send_debug_pose(double ex, double ey, double eyaw, float conf) {
        DebugPoseMsg m{ex, ey, eyaw, conf};
        return send(MsgType::DEBUG_POSE, &m, sizeof(m));
    }
    bool send_debug_path(const std::vector<std::pair<float,float>>& path,
                         const std::vector<std::tuple<float,float,float>>& particles) {
        int32_t n_path = (int32_t)std::min(path.size(), (size_t)256);
        int32_t n_part = (int32_t)std::min(particles.size(), (size_t)200);
        DebugPathMsg hdr_msg{n_path, n_part};
        size_t total = sizeof(hdr_msg) + (size_t)n_path * sizeof(PathPoint)
                                   + (size_t)n_part * sizeof(ParticleSample);
        std::vector<char> buf(total);
        std::memcpy(buf.data(), &hdr_msg, sizeof(hdr_msg));
        size_t off = sizeof(hdr_msg);
        for (int32_t i = 0; i < n_path; ++i) {
            PathPoint pp{path[i].first, path[i].second};
            std::memcpy(buf.data() + off, &pp, sizeof(pp)); off += sizeof(pp);
        }
        for (int32_t i = 0; i < n_part; ++i) {
            ParticleSample ps2{std::get<0>(particles[i]), std::get<1>(particles[i]), std::get<2>(particles[i])};
            std::memcpy(buf.data() + off, &ps2, sizeof(ps2)); off += sizeof(ps2);
        }
        return send(MsgType::DEBUG_PATH, buf.data(), (uint32_t)total);
    }
};

// ===== WinSock 初始化/清理（仅 Windows）=====
#ifdef _WIN32
struct WinSockInit {
    WinSockInit() { WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa); }
    ~WinSockInit() { WSACleanup(); }
};
#else
struct WinSockInit {
    WinSockInit() {}
    ~WinSockInit() {}
};
#endif

} // namespace bridge
