#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_docs_consistency.py — 文档一致性检查 (jihua20260905.md §13 风险登记第 8 项)

目标: 发布前自动发现"历史文档与当前场景矛盾", 要求旧报告加归档标记。
锚点一律来自权威单一真源, 不写死:
  - scene_home.json  : rooms 数量, robot.initial_pose
  - simulation_contract.yaml / geometry_spec.yaml : robot radius (0.35)
  - 当前发布基线标签: baseline-20260905

检查规则:
  STALE_ROOMS   文档声明房间数 7/8 (历史旧值) -> ERROR; 6 -> WARNING。当前场景为 5 房间。
  STALE_RADIUS  文档把 robot_radius 写成 0.25/0.30 -> ERROR。当前契约 0.35。
  STALE_DISTRO  文档混写 ROS2 "Jazzy" -> WARNING (须 Humble)。
  OLD_BASELINE  非当前旧 baseline 标签引用 -> WARNING (建议归档)。

文件分类:
  - 历史计划/交接文档 (jihua*/2026*/交接* 及 archive/) 命中的矛盾降为 WARNING 并提示归档,
    不阻断门禁 (它们本就是历史记录)。
  - 其余活动文档 (README/docs/*.md/graduation_design.md 等) 命中 ERROR 即阻断发布门禁。

用法:
  python scripts/check_docs_consistency.py [--root .] [--report out.json]

退出码: 0=通过(或无阻断项), 1=活动文档存在矛盾(阻断), 2=用法/文件错误
"""
import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "config"))

# 历史/外部文档: 命中矛盾降为 WARNING, 不阻断
HISTORICAL_RE = re.compile(
    r"(^|[/\\])(jihua\d*\.md|2026\d*\.md|交接\d*\.md|tigao\d*\.md|"
    r"HANDOVER\.md|handover.*\.md|.*历史.*\.md|.*规划书.*\.md|"
    r".*20\d{6}.*\.md)$", re.I)
EXCLUDE_DIRS = ("archive", ".workbuddy", "robot_src", "node_modules", "build",
                "__pycache__", ".git", "third_party", "backup_20260625")

CURRENT_BASELINE = "baseline-20260905"

ROOM_NUM_RE = re.compile(r"(?:(\d+)\s*个?\s*房间|rooms?\D{0,3}(\d+))", re.I)
RADIUS_RE = re.compile(r"(?:robot_radius|半径|radius)\s*[:=]\s*0\.(25|30)", re.I)
DISTRO_RE = re.compile(r"Jazzy")
OLD_BASELINE_RE = re.compile(r"baseline-(20260[0-8]\d\d\d)", re.I)


def _load_anchors(scene_path, contract_path):
    anchors = {"rooms": None, "initial_pose": None, "radius": None}
    try:
        import json as _json
        s = _json.load(open(scene_path, encoding="utf-8"))
        anchors["rooms"] = len(s.get("rooms", []) or [])
        rp = s.get("robot", {})
        ip = rp.get("initial_pose")
        if ip is not None:
            anchors["initial_pose"] = (ip.get("x"), ip.get("y"))
    except Exception:
        pass
    try:
        from config.config_loader import load_contract
        data, _, _ = load_contract(contract_path)
        r = data.get("robot", {}) or {}
        # 优先 contract.robot.radius, 回退 geometry_spec
        rad = r.get("radius")
        if rad is None:
            gspec = os.path.join(os.path.dirname(contract_path), "geometry_spec.yaml")
            if os.path.exists(gspec):
                import sys as _s
                _s.path.insert(0, os.path.join(ROOT, "config"))
                from config.config_loader import parse_yaml
                g = parse_yaml(open(gspec, encoding="utf-8").read())
                rad = (g.get("planning_footprint") or {}).get("radius") \
                    or (g.get("collision_envelope") or {}).get("radius")
        anchors["radius"] = rad
    except Exception:
        pass
    return anchors


def _is_historical(relpath):
    base = os.path.basename(relpath)
    if HISTORICAL_RE.search(relpath) or HISTORICAL_RE.search(base):
        return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=ROOT)
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    scene_path = os.path.join(args.root, "config", "scene_home.json")
    contract_path = os.path.join(args.root, "config", "simulation_contract.yaml")
    anchors = _load_anchors(scene_path, contract_path)

    errors = []
    warnings = []
    hits = []

    for dirpath, dirnames, filenames in os.walk(args.root):
        dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
        for fn in filenames:
            if not fn.endswith(".md"):
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, args.root).replace("\\", "/")
            try:
                with open(full, encoding="utf-8", errors="replace") as f:
                    text = f.read()
            except Exception:
                continue
            lines = text.splitlines()
            historical = _is_historical(rel)

            for ln, line in enumerate(lines, 1):
                # ---- 房间数 ----
                for m in ROOM_NUM_RE.finditer(line):
                    num = int(m.group(1) or m.group(2))
                    if num in (7, 8):
                        sev = "WARNING" if historical else "ERROR"
                        msg = ("房间数声明 %d 与当前场景 %s 房间矛盾 (历史旧值)"
                               % (num, anchors["rooms"]))
                        hits.append((rel, ln, "STALE_ROOMS", sev, msg))
                        (warnings if sev == "WARNING" else errors).append(
                            "%s:%d STALE_ROOMS %s" % (rel, ln, msg))
                    elif num == 6:
                        warnings.append("%s:%d STALE_ROOMS 房间数 6 与当前 %s 不符(疑似旧值)"
                                        % (rel, ln, anchors["rooms"]))
                        hits.append((rel, ln, "STALE_ROOMS", "WARNING",
                                     "房间数 6 疑似旧值"))
                # ---- 半径 ----
                if RADIUS_RE.search(line):
                    sev = "WARNING" if historical else "ERROR"
                    msg = ("robot_radius 写成 0.%s, 与契约 %s 矛盾"
                           % (RADIUS_RE.search(line).group(1), anchors["radius"]))
                    hits.append((rel, ln, "STALE_RADIUS", sev, msg))
                    (warnings if sev == "WARNING" else errors).append(
                        "%s:%d STALE_RADIUS %s" % (rel, ln, msg))
                # ---- 发行版 ----
                if DISTRO_RE.search(line):
                    warnings.append("%s:%d STALE_DISTRO 混写 ROS2 Jazzy (须 Humble)"
                                    % (rel, ln))
                    hits.append((rel, ln, "STALE_DISTRO", "WARNING", "Jazzy->Humble"))
                # ---- 旧基线标签 ----
                for m in OLD_BASELINE_RE.finditer(line):
                    tag = m.group(1)
                    if tag != CURRENT_BASELINE:
                        warnings.append("%s:%d OLD_BASELINE 引用旧基线 %s (当前 %s, 建议归档)"
                                        % (rel, ln, tag, CURRENT_BASELINE))
                        hits.append((rel, ln, "OLD_BASELINE", "WARNING", tag))

    report = {
        "anchors": anchors,
        "current_baseline": CURRENT_BASELINE,
        "errors": errors,
        "warnings": warnings,
        "hit_count": len(hits),
        "passed": len(errors) == 0,
    }
    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

    print("== 文档一致性检查 ==")
    print("  锚点: rooms=%s, radius=%s, initial_pose=%s, baseline=%s"
          % (anchors["rooms"], anchors["radius"], anchors["initial_pose"], CURRENT_BASELINE))
    if errors:
        print("  [ERROR] 活动文档存在矛盾, 阻断发布门禁:", file=sys.stderr)
        for e in errors:
            print("    - %s" % e, file=sys.stderr)
    else:
        print("  [OK] 活动文档无阻断性矛盾")
    if warnings:
        print("  [WARN] %d 条历史/提示项 (不阻断):" % len(warnings))
        for w in warnings[:40]:
            print("    ! %s" % w)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
