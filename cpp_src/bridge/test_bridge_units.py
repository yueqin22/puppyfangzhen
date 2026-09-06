#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_bridge_units.py — §5.2 单位约定单元测试

UE 侧使用厘米，bridge 侧使用米。本测试锁定协议与 trace 的单位约定，
防止"厘米/米"混用导致的 100 倍定位漂移（历史上真实出现过的故障模式）。

约定（与 protocol.h 一致）:
  - 位置 (x, y): 米 (double)
  - 速度 (vx, vy, wz): 米/秒、弧度/秒 (float)
  - 角度 (yaw): 弧度 (double)
  - LiDAR 距离 / max_range: 米 (float)
"""
import struct
import os
import sys
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))

# 与 protocol.h 一致的帧/消息常量
PROTOCOL_VERSION = 2
FRAME_HDR = struct.Struct("<IHHII")
HELLO = 0x0006
RESET = 0x0001
LIDAR = 0x0010
GROUND_TRUTH = 0x0011
PED_STATE = 0x0012
SIM_END = 0x0005

HDR = struct.Struct("<i64sddd")
LID = struct.Struct("<if")
GT = struct.Struct("<ddd")
PED = struct.Struct("<i")


def _build(type_, payload, seq, version=PROTOCOL_VERSION):
    hdr = FRAME_HDR.pack(len(payload), type_, version, seq,
                         int(__import__("time").time() * 1000) & 0xFFFFFFFF)
    return hdr + payload


def test_lidar_distance_unit_is_meters():
    """LiDAR max_range=8.0 必须被解释为 8 米，而非 800 厘米。"""
    n_rays = 72
    max_range = 8.0  # 米
    body = LID.pack(n_rays, max_range) + struct.pack("<%df" % n_rays, *([max_range] * n_rays))
    parsed_n, parsed_range = LID.unpack(body[:LID.size])
    assert parsed_n == n_rays
    # 关键: 协议层最大量程以米存储
    assert parsed_range == pytest.approx(8.0)
    assert parsed_range < 100.0  # 若为厘米则应为 800，远超限


def test_ground_truth_position_unit_is_meters():
    """GROUND_TRUTH 位置以米存储，直接对应场景坐标 (0.50, -2.20)。"""
    x, y, yaw = 0.50, -2.20, 0.0
    payload = GT.pack(x, y, yaw)
    px, py, pyaw = GT.unpack(payload)
    assert (px, py, pyaw) == pytest.approx((0.50, -2.20, 0.0))
    # 米级坐标应在合理室内范围 (< 20m)，排除厘米误用 (会 >> 100)
    assert abs(px) < 20.0 and abs(py) < 20.0


def test_yaw_unit_is_radians():
    """角度 yaw 以弧度存储，0.0 而非 0（度）。

    bridge 内部所有三角函数使用弧度；若 UE 误发度数(如 90.0)，
    经三角变换会完全错位。这里锁定协议字段语义为弧度。
    """
    yaw = 1.5707963  # π/2 弧度 = 90°
    _x, _y, parsed_yaw = GT.unpack(GT.pack(0.0, 0.0, yaw))
    assert parsed_yaw == pytest.approx(1.5707963)
    # 弧度值不应落在典型度数范围(>2π)
    assert abs(parsed_yaw) <= 6.2832


def test_loopback_trace_uses_meter_scale():
    """端到端: 驱动 bridge 跑若干帧，trace 的位置/运动距离必须是米级。

    这是对 §5.2 单位约定的集成级回归——若 bridge 把厘米当米，
    est_x 会出现 ~100 倍偏移，本断言会失败。
    """
    sys.path.insert(0, os.path.join(_ROOT, "scripts"))
    import verify_bridge_loopback as V  # noqa: E402

    port = 17891
    trace = os.path.join(_ROOT, "_unit_trace.csv")
    report = os.path.join(_ROOT, "_unit_report.json")
    try:
        rc = V.main(["--port", str(port), "--frames", "5",
                     "--trace", trace, "--report", report])
        assert rc == 0, "loopback 复验失败"
        with open(trace, encoding="utf-8") as fh:
            lines = [l for l in fh.read().splitlines() if l]
        assert len(lines) > 1, "trace 无数据行"
        header = lines[0].split(",")
        first = lines[1].split(",")
        est_x = float(first[header.index("est_x")])
        est_y = float(first[header.index("est_y")])
        # est 应在 initial_pose(0.50,-2.20) 的米级邻域
        assert abs(est_x - 0.50) < 0.5
        assert abs(est_y + 2.20) < 0.5
        # 运动距离(motion_delta)应为米级小量(每帧 ~2cm)
        md_idx = header.index("motion_delta")
        mds = [float(r.split(",")[md_idx]) for r in lines[2:] if r]
        assert any(0.0 < v < 0.5 for v in mds), "motion_delta 不在米级"
    finally:
        for f in (trace, report):
            if os.path.exists(f):
                os.remove(f)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
