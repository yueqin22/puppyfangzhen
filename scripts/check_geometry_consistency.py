#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_geometry_consistency.py — 几何尺寸单一真源校验 (jihua20260905.md §4 P0-3)
==============================================================================
计划依据 (§4 P0-3 "清理配置和模型尺寸漂移"):

    1. 把"规划 footprint""碰撞 footprint""视觉表现尺寸"明确分成三个字段。
    3. 用脚本从 contract 生成/校验 URDF、C++、UE capsule 和 Nav2 footprint。
    出口标准: scene、contract、URDF、UE capsule 和 adapter 限速的差异
              都有自动检查结果。

权威来源 (只读, 本脚本不修改):
    config/simulation_contract.yaml        robot.radius / length / width / footprint
                                           planning.inflation_radius / robot.safety_margin
    src/puppy_description/urdf/puppy.urdf.xacro   视觉体尺寸 (xacro:property)

受校验的派生真源:
    config/geometry_spec.yaml              三尺寸字段 + Nav2/UE 派生值 (受校验)

受校验的下游消费者:
    src/puppy_nav/config/nav2_params.yaml                 robot_radius / inflation_radius
    cpp_src/bridge/ue_modules/PuppyRobotPawn.cpp          UE 物理胶囊 (cm)

设计立场 (重要, 防止后人"顺手改回"):
    * 视觉体 (0.30x0.18) 明显小于规划 footprint (0.70x0.50) 是**有意为之**:
      规划 footprint 是仿真安全基线, 实体尺寸差异走 profile, 不覆盖仿真基线。
    * UE 物理胶囊 20cm 小于规划半径 0.35m 也是**有意为之**: 它是 Pawn 的物理
      扫掠体, 门道可通行性由它保证; 导航安全间距由 bridge/C++ 的 0.35m 膨胀负责。
      因此校验的不是"必须等于规划半径", 而是"必须等于已记录值 + 偏离有据"。
    * 但 Nav2 的 robot_radius/inflation_radius **必须**等于契约值: 历史正是这里
      写死了 0.25/0.30, 造成"契约说 0.35、实际按 0.25 规划"的隐性偷安全。

退出码:
    0 = PASS (可含 WARN)
    1 = 几何/尺寸漂移 (硬失败)
    2 = 契约自身非法
    3 = IO / 运行错误

用法:
    python scripts/check_geometry_consistency.py
    python scripts/check_geometry_consistency.py --report artifacts/geometry_consistency.json
    python scripts/check_geometry_consistency.py --fix      # 就地修正 nav2_params 的写死漂移
"""
import argparse
import json
import math
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_HERE, _ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

# 刻意不依赖 PyYAML: 本机 `python` (3.13) 没有 PyYAML, 只有 C:\Program Files\Python312
# 有。为了避免"换解释器就跑不了"的双解释器陷阱, 这里统一复用 config_loader 自带的
# 迷你 YAML 解析器 —— 与 check_config_consistency.py / validate_scene.py 保持一致。
from config.config_loader import (  # noqa: E402
    load_contract, validate_contract, parse_yaml,
)

CONTRACT_REL = "config/simulation_contract.yaml"
SPEC_REL = "config/geometry_spec.yaml"
URDF_REL = "src/puppy_description/urdf/puppy.urdf.xacro"
NAV2_REL = "src/puppy_nav/config/nav2_params.yaml"
UE_PAWN_REL = "cpp_src/bridge/ue_modules/PuppyRobotPawn.cpp"

EPS = 1e-9


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------
def _close(a, b, tol=1e-6):
    return a is not None and b is not None and abs(float(a) - float(b)) <= tol


def _polygon_area(pts):
    """鞋带公式; 返回带符号面积 (逆时针为正)。"""
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def _segments_intersect(p1, p2, p3, p4):
    """判断线段 p1p2 与 p3p4 是否真相交 (共享端点不算)。"""

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def on_seg(a, b, p):
        return (min(a[0], b[0]) - EPS <= p[0] <= max(a[0], b[0]) + EPS
                and min(a[1], b[1]) - EPS <= p[1] <= max(a[1], b[1]) + EPS)

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    if ((d1 > EPS and d2 < -EPS) or (d1 < -EPS and d2 > EPS)) and \
       ((d3 > EPS and d4 < -EPS) or (d3 < -EPS and d4 > EPS)):
        return True
    for (a, b, c) in ((p3, p4, p1), (p3, p4, p2), (p1, p2, p3), (p1, p2, p4)):
        if abs(cross(a, b, c)) <= EPS and on_seg(a, b, c):
            return True
    return False


def _polygon_self_intersects(pts):
    n = len(pts)
    if n < 4:
        return False
    edges = [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue  # 相邻边共享端点
            if _segments_intersect(*edges[i], *edges[j]):
                return True
    return False


def _point_in_convex_or_any(pts, q):
    """射线法判断点是否在多边形内 (对凹多边形也成立)。"""
    n = len(pts)
    inside = False
    x, y = q
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xint = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xint:
                inside = not inside
    return inside


# --------------------------------------------------------------------------
# 解析各来源
# --------------------------------------------------------------------------
_XACRO_PROP_RE = re.compile(
    r'<xacro:property\s+name="([A-Za-z0-9_]+)"\s+value="([^"]+)"\s*/>')


def parse_urdf_properties(path):
    """从 puppy.urdf.xacro 抽取 <xacro:property>, 只认纯数值 (不 eval ${})。"""
    props = {}
    with open(path, encoding="utf-8") as f:
        for m in _XACRO_PROP_RE.finditer(f.read()):
            name, raw = m.group(1), m.group(2).strip()
            try:
                props[name] = float(raw)
            except ValueError:
                continue  # 形如 ${...} 的表达式不参与几何真源
    return props


_UE_CAP_RE = re.compile(
    r'CollisionCapsule->SetCapsuleRadius\(\s*([0-9.]+)\s*f?\s*\)')
_UE_HALFH_RE = re.compile(
    r'CollisionCapsule->SetCapsuleHalfHeight\(\s*([0-9.]+)\s*f?\s*\)')


def parse_ue_capsule(path):
    """从 UE 权威镜像读物理胶囊 (cm)。返回 (radius_cm, half_height_cm)。"""
    if not os.path.exists(path):
        return None, None
    with open(path, encoding="utf-8") as f:
        text = f.read()
    r = _UE_CAP_RE.search(text)
    h = _UE_HALFH_RE.search(text)
    return (float(r.group(1)) if r else None,
            float(h.group(1)) if h else None)


def _walk_yaml_leaves(node, key, out, path=""):
    if isinstance(node, dict):
        for k, v in node.items():
            if k == key and isinstance(v, (int, float)) and not isinstance(v, bool):
                out.append((path + "/" + str(k), float(v)))
            _walk_yaml_leaves(v, key, out, path + "/" + str(k))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            _walk_yaml_leaves(v, key, out, path + "/[%d]" % i)


def parse_nav2_numbers(path):
    """抽出 nav2_params.yaml 中所有 robot_radius / inflation_radius 数值。"""
    if not os.path.exists(path):
        return {}, {}
    with open(path, encoding="utf-8") as f:
        doc = parse_yaml(f.read()) or {}
    out = {}
    for key in ("robot_radius", "inflation_radius"):
        leaves = []
        _walk_yaml_leaves(doc, key, leaves)
        out[key] = leaves
    return out


# --------------------------------------------------------------------------
# 主校验
# --------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description="几何尺寸单一真源校验 (P0-3)")
    ap.add_argument("--contract", default=CONTRACT_REL)
    ap.add_argument("--spec", default=SPEC_REL)
    ap.add_argument("--urdf", default=URDF_REL)
    ap.add_argument("--nav2", default=NAV2_REL)
    ap.add_argument("--ue-pawn", default=UE_PAWN_REL)
    ap.add_argument("--report", default="artifacts/geometry_consistency.json")
    ap.add_argument("--fix", action="store_true",
                    help="就地修正 nav2_params.yaml 中写死的半径/膨胀漂移")
    args = ap.parse_args(argv)

    report = {
        "test_name": "geometry_consistency",
        "status": "FAIL",
        "exit_code": 3,
        "errors": [],
        "warnings": [],
        "checks": {},
        "sources": {},
    }

    def err(msg):
        report["errors"].append(msg)

    def warn(msg):
        report["warnings"].append(msg)

    abs_ = lambda rel: rel if os.path.isabs(rel) else os.path.join(_ROOT, rel)  # noqa: E731

    # ---- 1. 契约 (权威) ----
    try:
        contract, sha, sv = load_contract(abs_(args.contract))
    except Exception as e:  # noqa: BLE001
        report["error"] = "load_contract failed: %s" % e
        return 3
    cerr, cwarn = validate_contract(contract)
    if cerr:
        report["errors"] = list(cerr)
        report["status"] = "FAIL"
        report["exit_code"] = 2
        _dump(report, args.report)
        return 2

    robot = contract.get("robot") or {}
    planning = contract.get("planning") or {}
    r_radius = float(robot.get("radius"))
    r_len = float(robot.get("length"))
    r_wid = float(robot.get("width"))
    safety_margin = float(robot.get("safety_margin"))
    footprint = [[float(p[0]), float(p[1])] for p in (robot.get("footprint") or [])]
    inflation = planning.get("inflation_radius")

    report["sources"]["contract"] = {
        "path": args.contract, "sha256": sha, "schema_version": sv,
        "robot_radius": r_radius, "robot_length": r_len, "robot_width": r_wid,
        "safety_margin": safety_margin, "footprint": footprint,
        "inflation_radius": inflation,
    }

    # ---- 2. URDF 视觉体 (权威) ----
    urdf_props = {}
    if os.path.exists(abs_(args.urdf)):
        urdf_props = parse_urdf_properties(abs_(args.urdf))
    report["sources"]["urdf"] = {
        "path": args.urdf,
        "body_length": urdf_props.get("body_length"),
        "body_width": urdf_props.get("body_width"),
        "body_height": urdf_props.get("body_height"),
        "stand_height": urdf_props.get("stand_height"),
    }
    if "body_length" not in urdf_props or "body_width" not in urdf_props:
        warn("%s: 未能解析 body_length/body_width, 视觉体校验跳过" % args.urdf)

    # ---- 3. geometry_spec.yaml (受校验派生真源) ----
    spec = {}
    with open(abs_(args.spec), encoding="utf-8") as f:
        spec = parse_yaml(f.read()) or {}
    report["sources"]["spec"] = {"path": args.spec, "schema_version": spec.get("schema_version")}

    checks = report["checks"]

    # 3.1 规划 footprint 与契约一致
    spec_fp = ((spec.get("planning_footprint") or {}).get("corners")) or []
    spec_fp = [[float(p[0]), float(p[1])] for p in spec_fp]
    ok_fp = (len(spec_fp) == len(footprint)
             and all(_close(a, b) for pa, pb in zip(spec_fp, footprint)
                     for a, b in zip(pa, pb)))
    checks["spec_footprint_matches_contract"] = ok_fp
    if not ok_fp:
        err("geometry_spec.planning_footprint %s != contract robot.footprint %s"
            % (spec_fp, footprint))

    # 3.2 footprint 几何合法性: 闭合 / 面积>0 / 不自交
    area = abs(_polygon_area(footprint)) if len(footprint) >= 3 else 0.0
    closed = len(footprint) >= 3
    non_self = not _polygon_self_intersects(footprint) if closed else False
    bbox_ok = _close(max(p[0] for p in footprint) - min(p[0] for p in footprint), r_len) and \
        _close(max(p[1] for p in footprint) - min(p[1] for p in footprint), r_wid)
    checks["footprint_closed_nonzero_area"] = bool(closed and area > 0.0 and non_self)
    checks["footprint_bbox_matches_length_width"] = bool(bbox_ok)
    if not (closed and area > 0.0):
        err("robot.footprint 未闭合或面积为 0 (area=%.4f)" % area)
    if non_self is False and closed:
        err("robot.footprint 自交")
    if not bbox_ok:
        err("robot.footprint 包围盒与 robot.length/width (%.2fx%.2f) 不符" % (r_len, r_wid))

    # 3.3 规划半径 <= footprint 外接圆半径
    circum = 0.5 * math.hypot(r_len, r_wid)
    ok_circ = r_radius <= circum + 1e-9
    checks["radius_le_circumradius"] = ok_circ
    if not ok_circ:
        err("robot.radius %.3f > footprint 外接圆半径 %.3f" % (r_radius, circum))

    # 3.4 spec 的 planning_radius / collision_capsule 与契约一致
    spec_radius = spec.get("planning_radius")
    spec_coll_r = (spec.get("collision_capsule") or {}).get("radius")
    checks["spec_radius_matches_contract"] = _close(spec_radius, r_radius)
    checks["collision_radius_eq_planning_radius"] = (
        _close(spec_coll_r, r_radius) and _close(spec_coll_r, spec_radius))
    if not _close(spec_radius, r_radius):
        err("geometry_spec.planning_radius %s != contract robot.radius %.3f" % (spec_radius, r_radius))
    if not _close(spec_coll_r, r_radius):
        err("geometry_spec.collision_capsule.radius %s != 规划半径 %.3f" % (spec_coll_r, r_radius))

    # 3.5 视觉体与契约记录一致 + 被 footprint 包络
    vb = spec.get("visual_body") or {}
    checks["spec_visual_body_matches_urdf"] = (
        _close(vb.get("length"), urdf_props.get("body_length"))
        and _close(vb.get("width"), urdf_props.get("body_width"))
        and _close(vb.get("height"), urdf_props.get("body_height"))
        and _close(vb.get("stand_height"), urdf_props.get("stand_height")))
    if not checks["spec_visual_body_matches_urdf"] and urdf_props:
        err("geometry_spec.visual_body 与 %s 的 xacro:property 不一致: spec=%s urdf=%s"
            % (args.urdf,
               {k: vb.get(k) for k in ("length", "width", "height", "stand_height")},
               {k: urdf_props.get(k) for k in ("body_length", "body_width", "body_height", "stand_height")}))

    bl = urdf_props.get("body_length")
    bw = urdf_props.get("body_width")
    if bl and bw and closed and area > 0.0:
        corners_in = all(_point_in_convex_or_any(footprint, (sx * bl / 2.0, sy * bw / 2.0))
                         for sx in (-1.0, 1.0) for sy in (-1.0, 1.0))
        checks["footprint_encloses_visual_body"] = corners_in
        if not corners_in:
            err("规划 footprint 未包络视觉体 (%.2fx%.2f 的角点落在足迹外)" % (bl, bw))
    else:
        checks["footprint_encloses_visual_body"] = None

    # 3.6 inflation_radius >= radius + safety_margin
    if inflation is not None:
        ok_inf = float(inflation) >= r_radius + safety_margin - 1e-9
        checks["inflation_ge_radius_plus_margin"] = ok_inf
        if not ok_inf:
            err("planning.inflation_radius %s < robot.radius + safety_margin = %.3f"
                % (inflation, r_radius + safety_margin))
    else:
        checks["inflation_ge_radius_plus_margin"] = None

    # ---- 4. Nav2 下游: 禁止写死漂移 ----
    nav2 = parse_nav2_numbers(abs_(args.nav2))
    nav2_report = {}
    drift = []
    for key, expect in (("robot_radius", r_radius),
                        ("inflation_radius", float(inflation) if inflation is not None else None)):
        leaves = nav2.get(key, [])
        nav2_report[key] = {"expected": expect, "found": [v for _, v in leaves]}
        if expect is None:
            continue
        for path, v in leaves:
            if not _close(v, expect):
                drift.append({"key": key, "path": path, "found": v, "expected": expect})
    checks["nav2_matches_contract"] = (not drift)
    report["checks"]["nav2_detail"] = nav2_report
    if drift:
        for d in drift:
            err("%s: %s = %s, 契约要求 %.3f (写死漂移)"
                % (args.nav2, d["key"], d["found"], d["expected"]))

    # ---- 5. Nav2 footprint 多边形 ----
    spec_nav2_poly = ((spec.get("nav2_footprint") or {}).get("polygon")) or []
    spec_nav2_poly = [[float(p[0]), float(p[1])] for p in spec_nav2_poly]
    checks["nav2_footprint_matches_planning"] = (
        len(spec_nav2_poly) == len(footprint)
        and all(_close(a, b) for pa, pb in zip(spec_nav2_poly, footprint)
                for a, b in zip(pa, pb)))
    if not checks["nav2_footprint_matches_planning"]:
        err("geometry_spec.nav2_footprint 与 contract robot.footprint 不一致")

    # ---- 6. UE 物理胶囊: 必须等于记录值 (允许有意偏离规划半径) ----
    ue_spec = spec.get("ue_physics_capsule") or {}
    ue_r, ue_h = parse_ue_capsule(abs_(args.ue_pawn))
    report["sources"]["ue_pawn"] = {"path": args.ue_pawn,
                                    "radius_cm": ue_r, "half_height_cm": ue_h}
    if ue_r is None or ue_h is None:
        checks["ue_capsule_matches_recorded"] = None
        warn("%s: 未找到 CollisionCapsule->SetCapsule(Radius|HalfHeight), UE 胶囊校验跳过"
             % args.ue_pawn)
    else:
        ok_r = _close(ue_r, ue_spec.get("radius_cm"), tol=1e-4)
        ok_h = _close(ue_h, ue_spec.get("half_height_cm"), tol=1e-4)
        checks["ue_capsule_matches_recorded"] = bool(ok_r and ok_h)
        if not ok_r:
            err("%s: UE 胶囊半径 %.1fcm != geometry_spec 记录值 %scm"
                % (args.ue_pawn, ue_r, ue_spec.get("radius_cm")))
        if not ok_h:
            err("%s: UE 胶囊半高 %.1fcm != geometry_spec 记录值 %scm"
                % (args.ue_pawn, ue_h, ue_spec.get("half_height_cm")))

    # 有意偏离的"有据"检查: 偏离必须显式声明, 否则视为未解释差异
    declared = ue_spec.get("deviation_from_planning_radius")
    checks["ue_capsule_deviation_declared"] = (declared is not None)
    if declared is None:
        err("geometry_spec.ue_physics_capsule 未声明 deviation_from_planning_radius "
            "(UE 胶囊 与 规划半径 不一致必须有书面理由)")
    report["ue_capsule_deviation"] = {
        "planning_radius_m": r_radius,
        "ue_capsule_radius_m": (ue_r / 100.0) if ue_r is not None else None,
        "declared": declared,
        "rationale": (ue_spec.get("rationale") or "").strip() or None,
    }

    # ---- 7. --fix: 就地修正 nav2 写死漂移 ----
    if args.fix and drift:
        fixed = _fix_nav2_file(abs_(args.nav2), drift)
        report["fix"] = {"file": args.nav2, "replacements": fixed}
        if fixed:
            # 修正后重读复核
            nav2b = parse_nav2_numbers(abs_(args.nav2))
            still = []
            for key, expect in (("robot_radius", r_radius),
                                ("inflation_radius", float(inflation) if inflation is not None else None)):
                if expect is None:
                    continue
                for path, v in nav2b.get(key, []):
                    if not _close(v, expect):
                        still.append({"key": key, "path": path, "found": v})
            report["fix"]["residual"] = still
            if not still:
                report["errors"] = [e for e in report["errors"]
                                    if "写死漂移" not in e]
                checks["nav2_matches_contract"] = True
    elif args.fix:
        report["fix"] = {"file": args.nav2, "replacements": [], "note": "无漂移"}

    # ---- 汇总 ----
    report["status"] = "PASS" if not report["errors"] else "FAIL"
    report["exit_code"] = 0 if not report["errors"] else 1
    report["summary"] = ("几何单一真源一致: planning r=%.2f, footprint %.2fx%.2f, "
                         "visual %.2fx%.2f, UE 胶囊 %.0fcm%s"
                         % (r_radius, r_len, r_wid,
                            bl or 0.0, bw or 0.0, ue_r or 0.0,
                            "" if not report["errors"] else "  (%d 处漂移)" % len(report["errors"])))
    _dump(report, args.report)
    _print(report)
    return report["exit_code"]


def _fix_nav2_file(path, drift):
    """按行就地替换写死的半径数值, 保留注释与缩进 (契约值才是唯一真源)。"""
    want = {}
    for d in drift:
        want[d["key"]] = d["expected"]
    if not want:
        return []
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    out = []
    replaced = []
    pat = re.compile(r'^(\s*)(%s)(\s*:\s*)([0-9.eE+-]+)(\s*(?:#.*)?)$'
                     % "|".join(sorted(want)))
    for line in lines:
        m = pat.match(line.rstrip("\n"))
        if m:
            key = m.group(2)
            new_val = want[key]
            new_line = "%s%s%s%s%s\n" % (m.group(1), key, m.group(3),
                                         ("%g" % new_val), m.group(5))
            if new_line != line:
                replaced.append({"line": line.strip(), "new": new_line.strip()})
            out.append(new_line)
        else:
            out.append(line)
    if replaced:
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.writelines(out)
    return replaced


def _dump(report, path):
    out_dir = os.path.dirname(path) or "."
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    abs_path = path if os.path.isabs(path) else os.path.join(_ROOT, path)
    with open(abs_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


def _print(report):
    print("=" * 68)
    print("  几何尺寸单一真源校验 (jihua20260905 §4 P0-3)")
    print("=" * 68)
    for name, val in sorted(report["checks"].items()):
        if name.endswith("_detail"):
            continue
        mark = "n/a " if val is None else ("PASS" if val else "FAIL")
        print("  %-38s [%s]" % (name, mark))
    if report.get("ue_capsule_deviation", {}).get("declared"):
        d = report["ue_capsule_deviation"]
        print("  %-38s 规划 %.2fm vs UE 胶囊 %sm (声明: %s)"
              % ("UE 胶囊有意偏离", d["planning_radius_m"],
                 d["ue_capsule_radius_m"], d["declared"]))
    if report["warnings"]:
        print("-" * 68)
        for w in report["warnings"]:
            print("  WARN  %s" % w)
    if report["errors"]:
        print("-" * 68)
        for e in report["errors"]:
            print("  ERROR %s" % e)
    print("-" * 68)
    print("RESULT: %s  (%s)" % (report["status"], report.get("summary", "")))


if __name__ == "__main__":
    sys.exit(main())
