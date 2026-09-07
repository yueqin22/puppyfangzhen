#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_bridge_loopback.py — jihua20260905.md §5.2 / §5.3 闭环复验工具

不依赖 UE，用一个最小 TCP 客户端驱动 nav_ue_bridge 跑若干 lockstep 帧，取得:
  §5.2 首帧 GT 与 scene initial_pose 一致性（GT_FIRST_FRAME 偏差记录 + est 种在 initial_pose）
  §5.3 逐帧 CSV trace 真实写入（含 amcl_updated / motion_delta / proc_ms 三列）

协议帧格式与 protocol.h 保持一致（pragma pack(1), 小端）。
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

PROTOCOL_VERSION = 2
FRAME_HDR = struct.Struct("<IHHII")          # length, type, version, seq, ts_ms
HDR = struct.Struct("<i64sddd")              # ResetMsg: seed, scene[64], x, y, yaw
LID = struct.Struct("<if")                    # LidarMsg: n_rays, max_range
GT = struct.Struct("<ddd")                    # GroundTruthMsg: x, y, yaw
PED = struct.Struct("<i")                     # PedStateMsg: n_peds

HELLO = 0x0006
HELLO_ACK = 0x0007
RESET = 0x0001
LIDAR = 0x0010
GROUND_TRUTH = 0x0011
PED_STATE = 0x0012
SIM_END = 0x0005

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)


def find_bridge(explicit=None):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    for name in ("nav_ue_bridge_test", "nav_ue_bridge_test.exe", "nav_ue_bridge", "nav_ue_bridge.exe"):
        p = os.path.join(_ROOT, "cpp_src", "bridge", name)
        if os.path.exists(p):
            return p
    return None


def build(type_, payload, seq, version=PROTOCOL_VERSION):
    hdr = FRAME_HDR.pack(len(payload), type_, version, seq,
                         int(time.time() * 1000) & 0xFFFFFFFF)
    return hdr + payload


def build_hello(seq=1, caps=b"loopback", version=PROTOCOL_VERSION):
    payload = struct.pack("<HH32s", 0x5050, version, caps.ljust(32, b"\0")[:32])
    return build(HELLO, payload, seq)


def build_reset(seed, init_x, init_y, init_yaw, scene=b"scene_home", seq=2):
    payload = HDR.pack(seed, scene.ljust(64, b"\0")[:64], init_x, init_y, init_yaw)
    return build(RESET, payload, seq)


def build_lidar(n_rays, max_range, seq):
    body = LID.pack(n_rays, max_range) + struct.pack("<%df" % n_rays, *([max_range] * n_rays))
    return build(LIDAR, body, seq)


def build_gt(x, y, yaw, seq):
    return build(GROUND_TRUTH, GT.pack(x, y, yaw), seq)


def build_ped(n_peds, seq):
    return build(PED_STATE, PED.pack(n_peds), seq)


def build_sim_end(seq):
    return build(SIM_END, b"", seq)


def recv_frame(sock, timeout=5.0):
    sock.settimeout(timeout)
    hdr = b""
    while len(hdr) < FRAME_HDR.size:
        c = sock.recv(FRAME_HDR.size - len(hdr))
        if not c:
            return None
        hdr += c
    length, mtype, version, seq, _ts = FRAME_HDR.unpack(hdr)
    payload = b""
    while len(payload) < length:
        c = sock.recv(length - len(payload))
        if not c:
            return None
        payload += c
    return mtype, version, seq, payload


def wait_connected(port, deadline=10.0):
    end = time.time() + deadline
    while time.time() < end:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=5)
        except OSError:
            time.sleep(0.15)
    raise RuntimeError(f"bridge 未在 {deadline}s 内监听")


def main(argv=None):
    ap = argparse.ArgumentParser(description="§5.2/§5.3 bridge 闭环复验")
    ap.add_argument("--bridge", default=None)
    ap.add_argument("--scene", default=os.path.join(_ROOT, "config", "scene_home.json"))
    ap.add_argument("--port", type=int, default=17877)
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--trace", default=os.path.join(_ROOT, "_loopback_trace.csv"))
    ap.add_argument("--report", default=os.path.join(_ROOT, "artifacts", "bridge_loopback.json"))
    args = ap.parse_args(argv)

    bridge = find_bridge(args.bridge)
    if not bridge:
        print("[FATAL] 找不到 bridge 可执行文件", file=sys.stderr)
        return 3

    proc = subprocess.Popen(
        [bridge, "--port", str(args.port), "--connect-timeout", "10",
         "--scene", args.scene, "--trace", args.trace, "--frames", "0"],
        cwd=os.path.dirname(bridge),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding="utf-8", errors="replace")
    res = {"name": "bridge_loopback_5_2_5_3", "pass": False}
    s = None
    try:
        s = wait_connected(args.port)
        # 握手
        s.sendall(build_hello())
        got = recv_frame(s, timeout=5)
        if got is None or got[0] != HELLO_ACK:
            res["error"] = "握手失败"
            return res
        # 发送 RESET，初始位姿 = scene initial_pose (0.50,-2.20,0)
        seq = 2
        s.sendall(build_reset(1, 0.50, -2.20, 0.0, seq=seq)); seq += 1
        # 发送 frames 个 lockstep 帧（每帧 LIDAR+GT+PED）
        init_x, init_y = 0.50, -2.20
        for f in range(args.frames):
            x = init_x + f * 0.02   # 每帧前进 2cm，制造可测运动距离
            y = init_y
            yaw = 0.0
            s.sendall(build_lidar(72, 8.0, seq)); seq += 1
            s.sendall(build_gt(x, y, yaw, seq)); seq += 1
            s.sendall(build_ped(0, seq)); seq += 1
            time.sleep(0.03)   # 让 bridge 处理并推进帧
        # 结束
        s.sendall(build_sim_end(seq)); seq += 1
        time.sleep(0.3)
        out, _ = proc.communicate(timeout=8)
        res["bridge_log"] = out or ""

        checks = {}
        # §5.2: 首帧 GT 一致性记录
        import re
        m = re.search(r"GT_FIRST_FRAME gt=\([^)]*\) scene_init=\([^)]*\) deviation=([0-9.]+) m", out)
        checks["first_frame_logged"] = m is not None
        checks["deviation_recorded"] = "deviation=" in (out or "")
        checks["seeded_at_initial_pose"] = "GT_FIRST_FRAME" in (out or "")
        # est 应种在 initial_pose: trace 第 1 数据行 est_x≈0.50 est_y≈-2.20
        if os.path.exists(args.trace):
            with open(args.trace, encoding="utf-8") as fh:
                lines = [l for l in fh.read().splitlines() if l]
            header = lines[0].split(",")
            data = lines[1:] if len(lines) > 1 else []
            res["trace_rows"] = len(data)
            checks["trace_has_data"] = len(data) >= 1
            # 列检查
            for col in ("amcl_updated", "motion_delta", "proc_ms"):
                checks[f"col_{col}"] = col in header
            if data:
                first = data[0].split(",")
                # est_x 在 header 索引 5, est_y 在 6。首帧 est 由 AMCL 从 initial_pose
                # 种子后处理一帧扫描得到，应在 AMCL 初始云尺度(~0.12m)内接近 initial_pose。
                try:
                    est_x = float(first[header.index("est_x")])
                    est_y = float(first[header.index("est_y")])
                    checks["est_seeded_at_init"] = (abs(est_x - 0.50) < 0.12 and
                                                    abs(est_y + 2.20) < 0.12)
                except (ValueError, IndexError):
                    checks["est_seeded_at_init"] = False
                # motion_delta 在后续帧 > 0
                if len(data) > 1:
                    md_idx = header.index("motion_delta")
                    mds = [float(r.split(",")[md_idx]) for r in data[1:] if r]
                    checks["motion_delta_positive"] = any(v > 0.0005 for v in mds)
        else:
            checks["trace_has_data"] = False

        res["checks"] = checks
        res["pass"] = all(checks.values())
    except Exception as e:
        res["error"] = str(e)
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
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    mark = "PASS" if res.get("pass") else "FAIL"
    print(f"  [{mark}] {res['name']}")
    for k, v in (res.get("checks") or {}).items():
        print(f"      {k}: {v}")
    print(f"\n  -> {res.get('pass') and 'PASS' or 'FAIL'}")
    print(f"[OK] 报告: {args.report}")
    return 0 if res.get("pass") else 1


if __name__ == "__main__":
    sys.exit(main())
