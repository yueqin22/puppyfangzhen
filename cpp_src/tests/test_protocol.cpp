// ============================================================================
// test_protocol.cpp -- P0-03 (2026-08-17)
//
// 协议 v2 故障注入 / 状态机白盒测试。覆盖 guihua20260812.md §10.2 协议测试
// 与 §6.4 超时策略要求的场景：
//   * 握手成功（HELLO/HELLO_ACK 双向）
//   * 版本不符被拒（握手进入 FAULT）
//   * 序列号校验：重复帧忽略、缺口/旧帧丢弃
//   * 接收超时（无数据）→ RECV_TIMEOUT（看门狗基础）
//   * 对端断开 → RECV_CLOSED（断线停车基础）
//   * 半包（截断 header）+ 断开 → RECV_CLOSED（绝不无限阻塞）
//   * 完整帧（带 payload）往返正确
//   * SIM_END / SHUTDOWN_ACK 往返
//
// 零第三方依赖，使用 test_framework.h。需要 Winsock（Windows）。
// ============================================================================
#include "test_framework.h"
#include "../bridge/protocol.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <sys/socket.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#endif

namespace {

// ---- Winsock 初始化（测试进程级）----
struct TestWsInit {
    TestWsInit() {
#ifdef _WIN32
        WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    }
    ~TestWsInit() {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};
static TestWsInit g_ws;

using Sock = int;  // 与 bridge::Link 的 int sock 一致

// 在回环地址上建立一对已连接的 TCP socket（server 端 / client 端）。
bool make_connected_pair(Sock& server, Sock& client) {
#ifdef _WIN32
    Sock listen_sock = (Sock)::socket(AF_INET, SOCK_STREAM, 0);
#else
    Sock listen_sock = ::socket(AF_INET, SOCK_STREAM, 0);
#endif
    if (listen_sock < 0) return false;
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    // 显式绑到回环地址（INADDR_ANY 会让 getsockname 返回 0.0.0.0，无法 connect）
    addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
    addr.sin_port = 0;  // 系统分配端口
    if (::bind(listen_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        ::closesocket(listen_sock); return false;
    }
    if (::listen(listen_sock, 1) != 0) {
        ::closesocket(listen_sock); return false;
    }
    struct sockaddr_in la;
    socklen_t ln = sizeof(la);
    ::getsockname(listen_sock, (struct sockaddr*)&la, &ln);

    client = (Sock)::socket(AF_INET, SOCK_STREAM, 0);
    if (client < 0) { ::closesocket(listen_sock); return false; }
    // 连接到回环地址 + 已分配端口
    struct sockaddr_in ca;
    std::memset(&ca, 0, sizeof(ca));
    ca.sin_family = AF_INET;
    ca.sin_addr.s_addr = ::inet_addr("127.0.0.1");
    ca.sin_port = la.sin_port;
    if (::connect(client, (struct sockaddr*)&ca, sizeof(ca)) != 0) {
        ::closesocket(client); ::closesocket(listen_sock); return false;
    }
    server = (Sock)::accept(listen_sock, nullptr, nullptr);
    ::closesocket(listen_sock);
    return server >= 0;
}

// 发送一个任意版本的原始帧（用于伪造版本不符 / 半包）。
bool send_raw_frame(Sock sk, uint16_t type, uint16_t version,
                    const void* payload, uint32_t len) {
    bridge::FrameHeader h{};
    h.length = len;
    h.type = type;
    h.version = version;
    h.sequence = 1;
    h.timestamp_ms = 0;
    int sent = 0;
    const char* p = reinterpret_cast<const char*>(&h);
    int total = (int)sizeof(h);
    while (sent < total) {
        int n = (int)::send(sk, p + sent, total - sent, 0);
        if (n <= 0) return false;
        sent += n;
    }
    if (len > 0) {
        sent = 0;
        p = reinterpret_cast<const char*>(payload);
        total = (int)len;
        while (sent < total) {
            int n = (int)::send(sk, p + sent, total - sent, 0);
            if (n <= 0) return false;
            sent += n;
        }
    }
    return true;
}

// ============================================================
TEST(protocol, handshake_ok) {
    Sock s, c;
    ASSERT_TRUE(make_connected_pair(s, c));
    bridge::Link server(s);
    bridge::Link client(c);

    std::atomic<bool> client_ok{false};
    std::thread cli([&] {
        client_ok = client.handshake_client("ue-v2");
    });
    bool server_ok = server.handshake_server("nav-core-v2");
    cli.join();

    ASSERT_TRUE(server_ok);
    ASSERT_TRUE(client_ok.load());
    ASSERT_TRUE(server.ps.handshaked);
    ASSERT_TRUE(client.ps.handshaked);
    ASSERT_TRUE(server.ps.conn_state == bridge::ConnState::RUNNING);
    ASSERT_TRUE(client.ps.conn_state == bridge::ConnState::RUNNING);
    ::closesocket(s);
    ::closesocket(c);
}

TEST(protocol, handshake_version_mismatch_rejected) {
    Sock s, c;
    ASSERT_TRUE(make_connected_pair(s, c));
    bridge::Link server(s);
    bridge::Link client(c);

    std::thread cli([&] {
        // 伪造一个版本不符的 HELLO
        bridge::HelloMsg hello{};
        hello.magic = bridge::PROTOCOL_MAGIC;
        hello.version = 99;  // 错误版本
        std::snprintf(hello.capabilities, sizeof(hello.capabilities), "bad");
        send_raw_frame(c, (uint16_t)bridge::MsgType::HELLO, 99, &hello, sizeof(hello));
        // 不阻塞等待 ACK（避免对端异常时无限挂起）；短暂等待让服务端发出 HELLO_ACK
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    });
    bool server_ok = server.handshake_server("nav-core-v2");
    cli.join();

    ASSERT_FALSE(server_ok);
    ASSERT_TRUE(server.ps.conn_state == bridge::ConnState::FAULT);
    ::closesocket(s);
    ::closesocket(c);
}

TEST(protocol, sequence_check) {
    bridge::ProtocolState ps;
    ASSERT_TRUE(bridge::check_rx_seq(ps, 1) == bridge::SeqCheck::OK);
    // 重复帧：忽略，不计入错误
    ASSERT_TRUE(bridge::check_rx_seq(ps, 1) == bridge::SeqCheck::DUPLICATE_FRAME);
    // 向前跳跃：接受
    ASSERT_TRUE(bridge::check_rx_seq(ps, 5) == bridge::SeqCheck::OK);
    // 旧帧（乱序/过期）：丢弃并记录协议错误
    ASSERT_TRUE(bridge::check_rx_seq(ps, 3) == bridge::SeqCheck::DROP);
    ASSERT_EQ(ps.protocol_errors, 1);
    ASSERT_EQ(ps.duplicate_frames, 1);
    ASSERT_EQ(ps.last_rx_seq, 5u);
}

TEST(protocol, recv_timeout_no_data) {
    Sock s, c;
    ASSERT_TRUE(make_connected_pair(s, c));
    bridge::Link server(s);
    bridge::FrameHeader hdr;
    std::vector<char> pl;
    int64_t t0 = bridge::now_ms();
    auto st = server.recv_timeout(hdr, pl, 100);
    int64_t dt = bridge::now_ms() - t0;
    ASSERT_EQ(st, bridge::Link::RECV_TIMEOUT);
    // 不应超过 timeout 太多（看门狗必须按时触发）
    ASSERT_TRUE(dt >= 90 && dt < 600);
    ::closesocket(s);
    ::closesocket(c);
}

TEST(protocol, recv_closed_on_disconnect) {
    Sock s, c;
    ASSERT_TRUE(make_connected_pair(s, c));
    bridge::Link server(s);
    ::closesocket(c);  // 客户端断开
    bridge::FrameHeader hdr;
    std::vector<char> pl;
    auto st = server.recv_timeout(hdr, pl, 1000);
    ASSERT_EQ(st, bridge::Link::RECV_CLOSED);
    ::closesocket(s);
}

TEST(protocol, half_packet_then_close_is_safe) {
    Sock s, c;
    ASSERT_TRUE(make_connected_pair(s, c));
    bridge::Link server(s);
    // 只发送 8 字节（header 共 16 字节）——截断的半包
    bridge::FrameHeader h{};
    h.length = 0;
    h.type = (uint16_t)bridge::MsgType::LIDAR;
    h.version = bridge::PROTOCOL_VERSION;
    ::send(c, reinterpret_cast<char*>(&h), 8, 0);
    ::closesocket(c);  // 随后断开
    bridge::FrameHeader hdr;
    std::vector<char> pl;
    // 绝不无限阻塞：应在合理时间内返回（截断 + 关闭 → CLOSED）
    auto st = server.recv_timeout(hdr, pl, 1000);
    ASSERT_EQ(st, bridge::Link::RECV_CLOSED);
    ::closesocket(s);
}

TEST(protocol, full_frame_roundtrip_with_payload) {
    Sock s, c;
    ASSERT_TRUE(make_connected_pair(s, c));
    bridge::Link server(s);
    bridge::Link client(c);

    // client -> server 发送一个带 payload 的 LIDAR 帧
    bridge::LidarMsg lm{};
    lm.n_rays = 72;
    lm.max_range = 8.0f;
    std::thread cli([&] {
        client.send(bridge::MsgType::LIDAR, &lm, sizeof(lm));
    });

    bridge::FrameHeader hdr;
    std::vector<char> pl;
    auto st = server.recv_timeout(hdr, pl, 1000);
    cli.join();

    ASSERT_EQ(st, bridge::Link::RECV_OK);
    ASSERT_TRUE(hdr.type == (uint16_t)bridge::MsgType::LIDAR);
    ASSERT_EQ((uint32_t)pl.size(), (uint32_t)sizeof(bridge::LidarMsg));
    bridge::LidarMsg got{};
    std::memcpy(&got, pl.data(), sizeof(got));
    ASSERT_EQ(got.n_rays, 72);
    ASSERT_NEAR(got.max_range, 8.0f, 1e-3f);

    // 接收方应按桥接主循环的真实做法：recv_timeout 只负责帧 I/O（不触碰序列号
    // 状态，否则桥接端随后的 check_rx_seq 会把每帧误判为重复），接收后再单独做
    // 序列号校验，校验通过才将 last_rx_seq 推进到 1。
    bridge::SeqCheck sc = bridge::check_rx_seq(server.ps, hdr.sequence);
    ASSERT_TRUE(sc == bridge::SeqCheck::OK);
    ASSERT_EQ(server.ps.last_rx_seq, 1u);
    ::closesocket(s);
    ::closesocket(c);
}

TEST(protocol, sim_end_and_shutdown_ack_roundtrip) {
    Sock s2, c2;
    ASSERT_TRUE(make_connected_pair(s2, c2));
    bridge::Link server(s2);
    bridge::Link client(c2);

    // server 发送 SIM_END，client 应收到
    std::thread srv([&] { server.send_sim_end(); });
    bridge::FrameHeader hdr;
    std::vector<char> pl;
    auto st = client.recv_timeout(hdr, pl, 1000);
    srv.join();
    ASSERT_EQ(st, bridge::Link::RECV_OK);
    ASSERT_TRUE(hdr.type == (uint16_t)bridge::MsgType::SIM_END);

    // client 回 SHUTDOWN_ACK，server 应收到
    std::thread cli([&] { client.send_shutdown_ack(); });
    st = server.recv_timeout(hdr, pl, 1000);
    cli.join();
    ASSERT_EQ(st, bridge::Link::RECV_OK);
    ASSERT_TRUE(hdr.type == (uint16_t)bridge::MsgType::SHUTDOWN_ACK);
    ::closesocket(s2);
    ::closesocket(c2);
}

} // namespace

int main() {
    return RUN_ALL_TESTS();
}
