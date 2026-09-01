#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_python_smoke.py — 无头 Python nav 栈冒烟 (M4 / §8 / §14.2)
================================================================
依据 jihua20260818.md: Python 可视化仿真需支持无头短跑, 且卡死必须有超时出口。

本环境无 pygame/显示, 故冒烟直接驱动 Python nav 栈的 PathValidator
(nav_core/validation/path_validator.py, 纯 math, 自带 160x120 栅格),
在"可通行验收场景"上跑全屋巡逻闭环 + 负向边, 验证:
  * Python 模块可导入并运行 (nav 栈未损坏);
  * 验收几何对足印(0.7x0.5)安全 (与 C++ M3 结论一致);
  * 全程受看门狗超时保护, 绝不卡死。

与 C++ nav_integration_test.cpp 共用同一份 acceptance 场景 (同名同序),
参数从 config/simulation_contract.yaml 加载 (M1 单一事实源)。

退出码: 0=通过, 1=必达边失败, 2=负向边被错误接受, 3=IO/运行错误, 124=看门狗超时。
"""
import sys
import os
import json
import math
import argparse
import threading
import importlib.util

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


# 直接按文件加载 path_validator.py (纯 math, 自带 OccupancyGrid),
# 绕过 nav_core/__init__.py 包链 (其经 nav_core.mapping 拉入 numpy,
# 本环境未装 numpy). 该方案已在 nav_core/validation/test_consistency.py 验证可用.
def _load_path_validator():
    spec = importlib.util.spec_from_file_location(
        "pv_mirror",
        os.path.join(_ROOT, "nav_core", "validation", "path_validator.py"),
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_pv = _load_path_validator()
OccupancyGrid = _pv.OccupancyGrid
PathValidator = _pv.PathValidator
PathValidationOptions = _pv.PathValidationOptions
PathValidationCode = _pv.PathValidationCode

# --------------------------------------------------------------------------
# 验收场景 (与 C++ nav_integration_test.cpp acceptance 场景同名同序)
# --------------------------------------------------------------------------
ACCEPTANCE_TARGETS = [
    (-1.0, -3.0, 0.0, "dock"),
    (0.0, -2.0, math.pi / 2, "living_room"),
    (-2.75, 0.0, 0.0, "door_living_bed1"),
    (-3.8, 1.0, 0.0, "bedroom1"),
    (-3.3, 1.0, 0.0, "door_bed1_study"),
    (-2.75, 2.3, 0.0, "door_study_y2"),
    (-4.0, 2.4, 0.0, "study"),
    (-2.5, 2.4, 0.0, "bedroom2"),
    (-1.0, 3.0, 0.0, "door_bed2_bath"),
    (-0.3, 2.4, 0.0, "bathroom"),
    (1.0, 3.0, 0.0, "door_bath_storage"),
    (2.45, 2.35, 0.0, "storage"),
    (2.0, 2.3, 0.0, "door_storage_dining"),
    (2.35, 1.4, 0.0, "dining_detour"),
    (2.35, 0.25, 0.0, "dining_entry"),
    (1.75, 0.3, 0.0, "dining"),
    (2.5, 0.5, 0.0, "door_dining_kitchen"),
    (2.65, 0.45, math.pi, "kitchen"),
    (2.75, 0.0, 0.0, "door_kitchen_living"),
]

ACCEPTANCE_OBSTACLES = [
    (-4.85, -2.9, -4.4, -1.6),   # NW 房间
    (3.70, 1.4, 4.85, 2.9),      # NE 房间
    (-4.85, -3.9, -4.4, -3.3),   # SW 房间
    (3.70, -3.9, 4.85, -3.3),    # SE 房间
    (-0.10, -0.2, 0.10, 0.9),    # 中央竖墙
]


def _build_grid(obstacles, w=160, h=120, res=0.1, ox=-8.0, oy=-6.0):
    g = OccupancyGrid(w, h, res, ox, oy)
    for (x0, y0, x1, y1) in obstacles:
        gx0, gy0 = g.world_to_grid(x0, y0)
        gx1, gy1 = g.world_to_grid(x1, y1)
        gx0, gx1 = max(0, min(g.width - 1, gx0)), max(0, min(g.width - 1, gx1))
        gy0, gy1 = max(0, min(g.height - 1, gy0)), max(0, min(g.height - 1, gy1))
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                g.log_odds[gy * g.width + gx] = 3.5  # 占用 (log_odds > 0.6)
    return g


def _straight_path(sx, sy, gx, gy, n=40):
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append((sx + (gx - sx) * t, sy + (gy - sy) * t))
    return pts


def _load_options(contract_path):
    """从统一契约加载足印与规划阈值; 失败则回退到契约默认值。"""
    opt = PathValidationOptions()
    opt.unknown_is_blocked = False
    try:
        from config.config_loader import load_contract
        data, _sha, _sv = load_contract(contract_path)
        robot = data.get("robot", {}) or {}
        planning = data.get("planning", {}) or {}
        fp = robot.get("footprint")
        if fp:
            flat = []
            for p in fp:
                if isinstance(p, (list, tuple)) and len(p) >= 2:
                    flat.append((float(p[0]), float(p[1])))
                elif isinstance(p, dict):
                    flat.append((float(p.get("x")), float(p.get("y"))))
            if flat:
                opt.footprint = flat
        if planning.get("min_path_clearance") is not None:
            opt.min_path_clearance = float(planning["min_path_clearance"])
        if planning.get("goal_tolerance") is not None:
            opt.goal_tolerance = float(planning["goal_tolerance"])
    except Exception as e:  # noqa: BLE001
        sys.stderr.write("[WARN] 契约加载失败, 回退默认值: %s\n" % e)
    return opt


def _dump(report, path):
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description="无头 Python nav 栈冒烟")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--config", default="config/simulation_contract.yaml")
    ap.add_argument("--report", default="artifacts/python_smoke.json")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args(argv)

    # 看门狗: 超时强制退出 (防 Python 卡死, §8 要求)
    def _force_exit():
        sys.stderr.write("[WATCHDOG] python smoke 超时 (%.1fs), 强制退出\n" % args.timeout)
        os._exit(124)

    timer = threading.Timer(args.timeout, _force_exit)
    timer.daemon = True
    timer.start()

    report = {
        "test_name": "python_smoke",
        "status": "FAIL",
        "exit_code": 3,
        "config": args.config,
        "frames": args.frames,
        "seed": args.seed,
        "edges": [],
    }
    try:
        grid = _build_grid(ACCEPTANCE_OBSTACLES)
        pv = PathValidator(grid)
        opt = _load_options(args.config)
        report["footprint"] = opt.footprint
        report["min_path_clearance"] = opt.min_path_clearance
        report["goal_tolerance"] = opt.goal_tolerance

        edges = []  # (from, to, required, negative, sx, sy, gx, gy)
        n = len(ACCEPTANCE_TARGETS)
        for i in range(n - 1):
            a, b = ACCEPTANCE_TARGETS[i], ACCEPTANCE_TARGETS[i + 1]
            edges.append((a[3], b[3], True, False, a[0], a[1], b[0], b[1]))
        # 闭合: door_kitchen_living -> dock
        a, b = ACCEPTANCE_TARGETS[-1], ACCEPTANCE_TARGETS[0]
        edges.append((a[3], b[3], True, False, a[0], a[1], b[0], b[1]))
        # 条件可达: kitchen -> living_room
        k = next(t for t in ACCEPTANCE_TARGETS if t[3] == "kitchen")
        lr = next(t for t in ACCEPTANCE_TARGETS if t[3] == "living_room")
        edges.append(("kitchen", "living_room", False, False, k[0], k[1], lr[0], lr[1]))
        # 负向 1: dock -> 障碍内部
        o0 = ACCEPTANCE_OBSTACLES[0]
        gx1 = (o0[0] + o0[2]) / 2.0
        gy1 = (o0[1] + o0[3]) / 2.0
        edges.append(("dock", "<inside-obstacle>", False, True, -1.0, -3.0, gx1, gy1))
        # 负向 2: dock -> 地图外
        edges.append(("dock", "<out-of-map>", False, True, -1.0, -3.0, 100.0, 100.0))

        required_total = required_pass = 0
        neg_total = neg_pass = 0
        for (frm, to, req, neg, sx, sy, gx, gy) in edges:
            path = _straight_path(sx, sy, gx, gy)
            vr = pv.validate(path, sx, sy, gx, gy, opt)
            if vr.valid:
                status = "OK"
            else:
                status = PathValidationCode.to_string(vr.code)
            if neg:
                passed = (not vr.valid)  # 负向: 必须被拒
                neg_total += 1
                if passed:
                    neg_pass += 1
            else:
                passed = vr.valid
                if req:
                    required_total += 1
                    if passed:
                        required_pass += 1
            clr = vr.min_clearance
            if not math.isfinite(clr):
                clr = None
            else:
                clr = round(clr, 4)
            report["edges"].append({
                "from": frm, "to": to, "required": req, "negative": neg,
                "status": status, "reason": vr.reason,
                "min_clearance": clr,
                "passed": passed,
            })

        report["required_total"] = required_total
        report["required_pass"] = required_pass
        report["negative_total"] = neg_total
        report["negative_pass"] = neg_pass

        if required_pass < required_total:
            report["status"] = "FAIL"
            report["exit_code"] = 1
            report["summary"] = "必达边 %d/%d 失败" % (required_pass, required_total)
        elif neg_pass < neg_total:
            report["status"] = "FAIL"
            report["exit_code"] = 2
            report["summary"] = "负向边 %d/%d 被错误接受" % (neg_pass, neg_total)
        else:
            report["status"] = "PASS"
            report["exit_code"] = 0
            report["summary"] = "必达 %d/%d, 负向 %d/%d" % (
                required_pass, required_total, neg_pass, neg_total)
    except Exception as e:  # noqa: BLE001
        report["error"] = str(e)
        report["status"] = "FAIL"
        report["exit_code"] = 3
        report["summary"] = "运行异常: %s" % e
    finally:
        timer.cancel()

    _dump(report, args.report)
    return report["exit_code"]


if __name__ == "__main__":
    sys.exit(main())
