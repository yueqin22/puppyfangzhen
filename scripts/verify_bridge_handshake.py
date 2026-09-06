#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_bridge_handshake.py — jihua20260905.md §5.1 启动与握手 复验工具

不依赖 UE 即可取得 bridge 握手/断链行为的**真实证据**（真实 TCP 连接 + 真实
协议帧），避免只靠读代码下结论。

覆盖的 §5.1 条目:
  A. bridge 先监听，客户端可连接（不使用固定 13 秒等待作为唯一同步方式）
  B. HELLO / HELLO_ACK / 协议版本 / 序列号 全部被记录
  C. 版本不匹配时明确记录拒绝原因（原先此路径近似静默）
  D. 无客户端时 bridge 在 --connect-timeout 到期退出，不无限挂起

用法:
    python scripts/verify_bridge_handshake.py [--bridge <exe>] [--report <json>]

退出码:
    0 = 全部通过
    1 = 存在失败项
    3 = 运行错误（找不到 bridge 可执行文件等）
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import time

# ---- 与 protocol.h 保持一致 ----
PROTOCOL_VERSION = 2
PROTOCOL_MAGIC = 0x5050
HELLO = 0x0006
HELLO_ACK = 0x0007
FRAME_HDR = struct.Struct("<IHHII")     # length, type, version, sequence, ts_ms
HELLO_MSG = struct.Struct("<HH32s")     # magic, version, capabilities[32]
HELLO_ACK_MSG = struct.Struct("<HHH30s")  # magic, version, status, server_info[30]

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)


def find_bridge(explicit=None):
    """定位 bridge 可执行文件"""
    if explicit:
        return explicit if os.path.exists(explicit) else None
    for rel in (
        os.path.join("cpp_src", "bridge", "nav_ue_bridge_test.exe"),
        os.path.join("cpp_src", "bridge", "build", "Release", "nav_ue_bridge.exe"),
        os.path.join("cpp_src", "bridge", "nav_ue_bridge.exe"),
    ):
        p = os.path.join(_ROOT, rel)
        if os.path.exists(p):
            return p
    return None


def build_hello(magic=PROTOCOL_MAGIC, version=PROTOCOL_VERSION,
                caps=b"verify-client", seq=1):
    """构造一个 HELLO 帧（header + payload）"""
    payload = HELLO_MSG.pack(magic, version, caps.ljust(32, b"\0")[:32])
    hdr = FRAME_HDR.pack(len(payload), HELLO, PROTOCOL_VERSION, seq,
                         int(time.time() * 1000) & 0xFFFFFFFF)
    return hdr + payload


def recv_frame(sock, timeout=5.0):
    """接收一帧，返回 (type, version, sequence, payload) 或 None"""
    sock.settimeout(timeout)
    hdr = b""
    while len(hdr) < FRAME_HDR.size:
        chunk = sock.recv(FRAME_HDR.size - len(hdr))
        if not chunk:
            return None
        hdr += chunk
    length, mtype, version, seq, _ts = FRAME_HDR.unpack(hdr)
    payload = b""
    while len(payload) < length:
        chunk = sock.recv(length - len(payload))
        if not chunk:
            return None
        payload += chunk
    return mtype, version, seq, payload


def wait_listening(port, deadline=10.0):
    """轮询直到能连上 bridge，返回**已建立的 socket**。

    注意: 不能"先探测连一次再断开"——bridge 用 listen(backlog=1) 且只 accept
    一次，探测连接会占用该连接槽并被 bridge 当作真正的 UE 客户端，导致后续
    真实连接被拒绝。因此这里直接把重试得到的连接交给调用方使用。
    """
    end = time.time() + deadline
    last = None
    while time.time() < end:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=5)
        except OSError as e:
            last = e
            time.sleep(0.15)
    raise RuntimeError(f"bridge 未在 {deadline}s 内开始监听 (last={last})")


def run_bridge(bridge, args, timeout=60):
    """启动 bridge 子进程"""
    return subprocess.Popen([bridge] + args, cwd=os.path.dirname(bridge),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace")


def case_silent_emergency_stop(bridge, port, silent_sec=0.6):
    """§5.1 断链安全停止: 握手后静默 >=300ms -> bridge 进入 EMERGENCY_STOP (DEGRADED)

    注意: bridge 在 DEGRADED 后**不退出循环**（持续安全停止而非终止），因此本用例
    保持 socket 打开、静默等待、读取日志后即终止子进程。
    """
    proc = run_bridge(bridge, ["--port", str(port), "--connect-timeout", "10",
                               "--frames", "0"])
    res = {"name": "silent_emergency_stop", "pass": False,
           "silent_sec": silent_sec}
    s = None
    try:
        try:
            s = wait_listening(port)
        except RuntimeError as e:
            res["error"] = str(e)
            return res
        s.sendall(build_hello(caps=b"silent-test"))
        got = recv_frame(s, timeout=5)
        if got is None:
            res["error"] = "握手未完成（未收到 HELLO_ACK）"
            return res
        # 握手完成后保持静默，等待断链看门狗触发（>=300ms）
        time.sleep(silent_sec)
        # 关闭 socket 触发 DISCONNECT，再收集全部输出（含 DEGRADED/EMERGENCY_STOP）
        s.close()
        try:
            out, _ = proc.communicate(timeout=5)
        except Exception:
            proc.kill()
            out, _ = proc.communicate()
        res["bridge_log"] = out or ""
        log = res["bridge_log"]
        checks = {
            "logged_degraded": "[Bridge][DEGRADED] no data" in log,
            "logged_emergency_stop":
                "EMERGENCY_STOP" in log or "safe-stop" in log.lower(),
        }
        res["checks"] = checks
        res["pass"] = all(checks.values())
    finally:
        if s is not None:
            try:
                s.close()
            except Exception:
                pass
        try:
            proc.terminate()
            proc.communicate(timeout=5)
        except Exception:
            proc.kill()
    return res


def drain(proc):
    """读取子进程已有输出（非阻塞式尽力读取）"""
    out = []
    try:
        proc.stdout.settimeout(0.2) if hasattr(proc.stdout, "settimeout") else None
    except Exception:
        pass
    return out


def case_valid_hello(bridge, port):
    """A+B: 合法 HELLO -> HELLO_ACK(status=0)，且 bridge 记录 HELLO/HELLO_ACK"""
    proc = run_bridge(bridge, ["--port", str(port), "--connect-timeout", "8",
                               "--frames", "0"])
    res = {"name": "valid_hello", "pass": False}
    try:
        try:
            s = wait_listening(port)
        except RuntimeError as e:
            res["error"] = str(e)
            return res
        s.sendall(build_hello(caps=b"ue5-verify"))
        got = recv_frame(s, timeout=5)
        s.close()
        if got is None:
            res["error"] = "未收到 HELLO_ACK"
            return res
        mtype, version, seq, payload = got
        magic, ack_ver, status, info = HELLO_ACK_MSG.unpack(payload[:HELLO_ACK_MSG.size])
        res["hello_ack"] = {
            "type": hex(mtype), "version": version, "sequence": seq,
            "magic": hex(magic), "ack_version": ack_ver, "status": status,
            "server_info": info.rstrip(b"\0").decode("utf-8", "replace"),
        }
        checks = {
            "type_is_hello_ack": mtype == HELLO_ACK,
            "magic_matches": magic == PROTOCOL_MAGIC,
            "version_matches": ack_ver == PROTOCOL_VERSION,
            "status_ok": status == 0,
        }
        res["checks"] = checks
        res["pass"] = all(checks.values())
    finally:
        try:
            proc.terminate()
            out, _ = proc.communicate(timeout=10)
        except Exception:
            proc.kill()
            out = ""
        res["bridge_log"] = out or ""
    # 日志必须记录 HELLO 与 HELLO_ACK（§5.1 “全部记录”）
    log = res.get("bridge_log", "")
    res["log_records"] = {
        "hello_recv": "[HELLO] recv" in log,
        "hello_ack_sent": "[HELLO_ACK] sent" in log,
        "handshake_ok": "v2 handshake OK" in log,
    }
    if res["pass"]:
        res["pass"] = all(res["log_records"].values())
    return res


def case_version_mismatch(bridge, port):
    """C: 版本不匹配 -> HELLO_ACK(status=1) 且明确记录拒绝原因"""
    proc = run_bridge(bridge, ["--port", str(port), "--connect-timeout", "8",
                               "--frames", "0"])
    res = {"name": "version_mismatch", "pass": False}
    try:
        try:
            s = wait_listening(port)
        except RuntimeError as e:
            res["error"] = str(e)
            return res
        s.sendall(build_hello(version=99, caps=b"bad-version"))
        got = recv_frame(s, timeout=5)
        s.close()
        if got is None:
            res["error"] = "未收到 HELLO_ACK（版本不符时应回 status=1 而非静默断开）"
            return res
        mtype, version, seq, payload = got
        magic, ack_ver, status, _info = HELLO_ACK_MSG.unpack(payload[:HELLO_ACK_MSG.size])
        res["hello_ack"] = {"type": hex(mtype), "status": status,
                            "magic": hex(magic)}
        checks = {
            "type_is_hello_ack": mtype == HELLO_ACK,
            "status_is_reject": status == 1,
        }
        res["checks"] = checks
        res["pass"] = all(checks.values())
    finally:
        try:
            proc.terminate()
            out, _ = proc.communicate(timeout=10)
        except Exception:
            proc.kill()
            out = ""
        res["bridge_log"] = out or ""
    log = res.get("bridge_log", "")
    # §5.1 关键：拒绝原因必须可追溯（本项在补齐前是近似静默的）
    res["log_records"] = {
        "records_reject_reason": "[HELLO_ACK] REJECT" in log,
        "records_peer_hello": "[HELLO] recv" in log,
    }
    if res["pass"]:
        res["pass"] = all(res["log_records"].values())
    return res


def case_no_client_timeout(bridge, port, timeout_sec=3):
    """D: 无客户端 -> bridge 在 --connect-timeout 到期退出，不无限挂起"""
    t0 = time.time()
    proc = run_bridge(bridge, ["--port", str(port),
                               "--connect-timeout", str(timeout_sec),
                               "--frames", "0"])
    res = {"name": "no_client_timeout", "pass": False,
           "connect_timeout": timeout_sec}
    try:
        out, _ = proc.communicate(timeout=timeout_sec + 15)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
        res["error"] = "bridge 未能在超时后退出（无限挂起）"
        res["elapsed_sec"] = round(time.time() - t0, 2)
        return res
    elapsed = time.time() - t0
    res["elapsed_sec"] = round(elapsed, 2)
    res["bridge_log"] = out or ""
    checks = {
        "exited": proc.returncode is not None,
        "not_hanging": elapsed < (timeout_sec + 10),
        "logged_timeout": "[Bridge][TIMEOUT] no UE client" in (out or ""),
    }
    res["checks"] = checks
    res["pass"] = all(checks.values())
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description="§5.1 bridge 握手复验")
    ap.add_argument("--bridge", default=None, help="bridge 可执行文件路径")
    ap.add_argument("--port-base", type=int, default=17777)
    ap.add_argument("--report", default="artifacts/bridge_handshake.json")
    args = ap.parse_args(argv)

    bridge = find_bridge(args.bridge)
    if not bridge:
        print("[FATAL] 找不到 bridge 可执行文件；请先用 cl.exe 构建 "
              "cpp_src/bridge，或显式传入 --bridge", file=sys.stderr)
        return 3
    print(f"[INFO] bridge: {bridge}")

    cases = []
    cases.append(case_valid_hello(bridge, args.port_base))
    cases.append(case_version_mismatch(bridge, args.port_base + 1))
    cases.append(case_no_client_timeout(bridge, args.port_base + 2, timeout_sec=3))
    cases.append(case_silent_emergency_stop(bridge, args.port_base + 3, silent_sec=0.6))

    report = {
        "test_name": "bridge_handshake_5_1",
        "bridge": bridge,
        "protocol_version": PROTOCOL_VERSION,
        "cases": cases,
        "passed": sum(1 for c in cases if c.get("pass")),
        "total": len(cases),
    }
    report["status"] = "PASS" if report["passed"] == report["total"] else "FAIL"
    report["exit_code"] = 0 if report["status"] == "PASS" else 1

    out_path = os.path.join(_ROOT, args.report)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    for c in cases:
        mark = "PASS" if c.get("pass") else "FAIL"
        print(f"  [{mark}] {c['name']}")
        if not c.get("pass"):
            for k in ("error",):
                if k in c:
                    print(f"         {k}: {c[k]}")
            for k, v in (c.get("checks") or {}).items():
                if not v:
                    print(f"         check failed: {k}")
            for k, v in (c.get("log_records") or {}).items():
                if not v:
                    print(f"         log missing: {k}")
    print(f"\n{report['passed']}/{report['total']} passed -> {report['status']}")
    print(f"[OK] 报告: {out_path}")
    return report["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
