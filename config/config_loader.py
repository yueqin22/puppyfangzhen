#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
config_loader.py — Puppy 仿真导航统一配置加载器 (Python 侧)
================================================================
依据 jihua20260818.md 模块一 (§4)。与 C++ cpp_src/common/config_contract.h
共享同一份 config/simulation_contract.yaml，保证 Python / C++ / ROS / UE
读取同一来源。

功能:
  - 零依赖解析契约 YAML (含嵌套段与 footprint 列表)。
  - 启动时计算文件 SHA-256 与 schema_version。
  - 一致性校验 (§4.3):
      * footprint 闭合 / 不自交 / 面积 > 0 / 与 robot.radius 一致。
      * inflation_radius >= robot_radius + safety_margin。
      * map.resolution / width / height > 0。
      * goal_tolerance 合理性 (<= 关键门宽的一半, 这里用 footprint 长边近似)。
  - 生成扁平快照 snapshot(), 供字段级 diff (白盒测试 §4.4)。

用法:
  python config/config_loader.py config/simulation_contract.yaml
  python config/config_loader.py config/simulation_contract.yaml --check-radii \
      config/runtime.yaml config/unified_params.yaml
"""
import sys
import os
import json
import hashlib


# --------------------------------------------------------------------------
# 极简 YAML 解析 (仅支持本项目契约所需的子集: 嵌套段 / 标量 / 内联列表)
# --------------------------------------------------------------------------
def _strip_inline_comment(s):
    # 仅当 '#' 前为空白时视为注释 (避免误伤 hex/URL 中的 #)
    out = []
    for i, ch in enumerate(s):
        if ch == "#" and (i == 0 or s[i - 1] in " \t"):
            break
        out.append(ch)
    return "".join(out)


def _parse_scalar(s):
    s = _strip_inline_comment(s.strip())
    if s.startswith("[") and s.endswith("]"):
        inner = s[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(x.strip()) for x in inner.split(",")]
    if s == "" or s == "null" or s == "~":
        return None
    if s in ("true", "True", "TRUE"):
        return True
    if s in ("false", "False", "FALSE"):
        return False
    try:
        if "." in s or "e" in s.lower():
            return float(s)
        return int(s)
    except ValueError:
        return s


def _parse_block(lines, i, indent):
    node = {}
    while i < len(lines):
        raw = lines[i]
        if not raw.strip():
            i += 1
            continue
        _s = raw.strip()
        if _s.startswith("#"):
            i += 1
            continue
        cur = len(raw) - len(raw.lstrip(" "))
        if cur < indent:
            break
        if cur > indent:
            i += 1
            continue
        stripped = raw.strip()
        if stripped.startswith("- "):
            i += 1
            continue
        if ":" not in stripped:
            i += 1
            continue
        key, _, rest = stripped.partition(":")
        key = key.strip()
        rest = _strip_inline_comment(rest).strip()
        nxt = i + 1
        if nxt < len(lines):
            nxt_raw = lines[nxt]
            nxt_stripped = nxt_raw.strip()
            nxt_indent = len(nxt_raw) - len(nxt_raw.lstrip(" ")) if nxt_stripped else indent
            if rest == "" and nxt_stripped.startswith("- "):
                lst = []
                j = nxt
                while j < len(lines):
                    lr = lines[j]
                    if not lr.strip():
                        j += 1
                        continue
                    lcur = len(lr) - len(lr.lstrip(" "))
                    lr_l = lr.lstrip(" ")
                    if lcur < nxt_indent or not lr_l.startswith("- "):
                        break
                    lst.append(_parse_scalar(lr_l[2:].strip()))
                    j += 1
                node[key] = lst
                i = j
                continue
            elif rest == "" and nxt_stripped and nxt_indent > indent:
                child, i = _parse_block(lines, nxt, nxt_indent)
                node[key] = child
                continue
        node[key] = _parse_scalar(rest) if rest != "" else None
        i = nxt
    return node, i


def parse_yaml(text):
    lines = text.splitlines()
    root, _ = _parse_block(lines, 0, 0)
    return root


def load_contract(path):
    """返回 (data:dict, sha256:str, schema_version:int)。文件缺失时抛异常。"""
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    sha = hashlib.sha256(text.encode("utf-8")).hexdigest()
    data = parse_yaml(text)
    sv = data.get("schema_version", 0)
    return data, sha, sv


# --------------------------------------------------------------------------
# 几何工具
# --------------------------------------------------------------------------
def _polygon_area(pts):
    n = len(pts)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x1, y1 = pts[i][0], pts[i][1]
        x2, y2 = pts[(i + 1) % n][0], pts[(i + 1) % n][1]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def _seg_intersect(p1, p2, p3, p4):
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])
    d1 = ccw(p3, p4, p1)
    d2 = ccw(p3, p4, p2)
    d3 = ccw(p1, p2, p3)
    d4 = ccw(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return False


def _self_intersecting(pts):
    n = len(pts)
    for i in range(n):
        a1, a2 = pts[i], pts[(i + 1) % n]
        for j in range(i + 2, n):
            if (i == 0 and j == n - 1):
                continue  # 相邻边 (闭合)
            b1, b2 = pts[j], pts[(j + 1) % n]
            if _seg_intersect(a1, a2, b1, b2):
                return True
    return False


# --------------------------------------------------------------------------
# 校验 (§4.3)
# --------------------------------------------------------------------------
def validate_contract(data):
    errors = []
    warnings = []

    robot = data.get("robot", {}) or {}
    if not isinstance(robot, dict):
        errors.append("robot section missing or wrong type")
        robot = {}
    r_radius = robot.get("radius")
    if not (isinstance(r_radius, (int, float)) and r_radius > 0):
        errors.append("robot.radius must be a positive number")

    fp = robot.get("footprint")
    if not isinstance(fp, list) or len(fp) < 3:
        errors.append("robot.footprint must be a list of >=3 points")
    else:
        pts = []
        ok = True
        for p in fp:
            if (not isinstance(p, list)) or len(p) != 2:
                errors.append("robot.footprint point must be [x, y]")
                ok = False
                break
            pts.append((float(p[0]), float(p[1])))
        if ok:
            area = _polygon_area(pts)
            if area <= 0:
                errors.append("robot.footprint area must be > 0 (got %.4f)" % area)
            if _self_intersecting(pts):
                errors.append("robot.footprint is self-intersecting")
            # 与 robot.radius 一致性: 外接半径应 >= radius (footprint 覆盖 radius 圆)
            max_r = max((x * x + y * y) ** 0.5 for x, y in pts)
            if r_radius is not None and max_r < float(r_radius) - 1e-6:
                errors.append(
                    "robot.footprint circumscribed radius %.3f < robot.radius %.3f"
                    % (max_r, float(r_radius)))
            # 闭合性: footprint 采用"开放 4 角点"标准表示 (与 C++ test_config_contract
            # 期望 size()==4 一致)。面积/自交/外接半径校验均按隐式闭合处理, 因此不再对
            # 未显式闭合发警告 (jihua20260905.md P0-3: 消除 footprint 警告)。

    planning = data.get("planning", {}) or {}
    infl = planning.get("inflation_radius")
    margin = robot.get("safety_margin")
    if (isinstance(infl, (int, float)) and isinstance(r_radius, (int, float))
            and isinstance(margin, (int, float))):
        if float(infl) < float(r_radius) + float(margin) - 1e-9:
            errors.append(
                "planning.inflation_radius %.3f < robot.radius %.3f + safety_margin %.3f"
                % (float(infl), float(r_radius), float(margin)))

    gtol = planning.get("goal_tolerance")
    if isinstance(gtol, (int, float)):
        if float(gtol) <= 0:
            errors.append("planning.goal_tolerance must be > 0")
        # 关键门宽近似: 用 footprint 长边 (length) 作为参考门宽
        length = robot.get("length")
        if isinstance(length, (int, float)) and float(gtol) > float(length) / 2.0 + 1e-9:
            errors.append(
                "planning.goal_tolerance %.3f > half key-door-width %.3f"
                % (float(gtol), float(length) / 2.0))

    mp = data.get("map", {}) or {}
    for k in ("resolution", "width", "height"):
        v = mp.get(k)
        if not (isinstance(v, (int, float)) and v > 0):
            errors.append("map.%s must be a positive number" % k)

    sim = data.get("simulation", {}) or {}
    df = sim.get("default_frames")
    if not (isinstance(df, int) and df > 0):
        errors.append("simulation.default_frames must be a positive integer")
    mf = sim.get("max_frames")
    if isinstance(df, int) and isinstance(mf, int) and df > mf:
        errors.append("simulation.default_frames > max_frames")

    return errors, warnings


# --------------------------------------------------------------------------
# 快照 (§4.4 字段级 diff)
# --------------------------------------------------------------------------
def snapshot(data):
    robot = data.get("robot", {}) or {}
    planning = data.get("planning", {}) or {}
    mp = data.get("map", {}) or {}
    sim = data.get("simulation", {}) or {}
    acc = data.get("acceptance", {}) or {}
    return {
        "schema_version": data.get("schema_version"),
        "robot.radius": robot.get("radius"),
        "robot.length": robot.get("length"),
        "robot.width": robot.get("width"),
        "robot.safety_margin": robot.get("safety_margin"),
        "robot.footprint": robot.get("footprint"),
        "map.resolution": mp.get("resolution"),
        "map.width": mp.get("width"),
        "map.height": mp.get("height"),
        "planning.inflation_radius": planning.get("inflation_radius"),
        "planning.goal_tolerance": planning.get("goal_tolerance"),
        "planning.min_path_clearance": planning.get("min_path_clearance"),
        "simulation.default_frames": sim.get("default_frames"),
        "simulation.max_frames": sim.get("max_frames"),
        "simulation.seed": sim.get("seed"),
        "acceptance.max_average_localization_error":
            acc.get("max_average_localization_error"),
        "acceptance.max_peak_localization_error":
            acc.get("max_peak_localization_error"),
        "acceptance.min_required_rooms": acc.get("min_required_rooms"),
    }


# --------------------------------------------------------------------------
# 跨配置文件 robot.radius 一致性检查 (§4.4 黑盒)
# --------------------------------------------------------------------------
def _extract_robot_radius_yaml(path):
    """从现有 sim_cpp.yaml / runtime.yaml / unified_params.yaml 提取 robot_radius。"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
    except FileNotFoundError:
        return None
    import re
    # unified_params.yaml: robot.radius: 0.35
    m = re.search(r"^\s*radius:\s*([0-9.]+)", text, re.MULTILINE)
    if m:
        return float(m.group(1))
    # runtime.yaml: robot_radius: 0.35
    m = re.search(r"^\s*robot_radius:\s*([0-9.]+)", text, re.MULTILINE)
    if m:
        return float(m.group(1))
    return None


def main(argv):
    if len(argv) < 2:
        print("usage: config_loader.py <contract.yaml> [--check-radii f1 f2 ...]")
        return 2
    path = argv[1]
    try:
        data, sha, sv = load_contract(path)
    except Exception as e:
        print("[FATAL] cannot load contract %s: %s" % (path, e))
        return 5

    errors, warnings = validate_contract(data)
    snap = snapshot(data)

    print("schema_version : %s" % sv)
    print("sha256         : %s" % sha)
    print("robot.radius   : %s" % snap["robot.radius"])
    print("footprint      : %s" % snap["robot.footprint"])
    for w in warnings:
        print("[WARN] %s" % w)
    for e in errors:
        print("[ERROR] %s" % e)

    if "--check-radii" in argv:
        others = argv[argv.index("--check-radii") + 1:]
        contract_r = snap["robot.radius"]
        print("\n-- robot.radius cross-check (must all equal %.3f) --"
              % (contract_r if isinstance(contract_r, float) else 0))
        all_ok = True
        for op in others:
            r = _extract_robot_radius_yaml(op)
            if r is None:
                print("  %-40s : <not found>" % op)
                continue
            match = (isinstance(contract_r, (int, float))
                     and abs(r - float(contract_r)) < 1e-9)
            all_ok = all_ok and match
            print("  %-40s : %.3f %s" % (op, r, "OK" if match else "MISMATCH"))
        if not all_ok:
            errors.append("robot.radius mismatch across configs")

    if errors:
        print("\nVALIDATION: FAIL (%d error(s))" % len(errors))
        return 5
    print("\nRESULT: VALIDATION PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
