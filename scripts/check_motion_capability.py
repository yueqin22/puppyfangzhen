#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_motion_capability.py — vx/vy/wz 运动能力表校验 (jihua20260905 §4 P0-3 / §7.4)
==================================================================================
计划要求 (§4 P0-3 第 4 条):

    为 vx/vy/wz 建立运动能力表。若当前 Gazebo planar move 的 yaw 或 vy 不稳定,
    导航测试不得把未标定的角速度当作可靠能力; 优先使用已测量轴,
    或切换到真实 gait/controller 路径。

(§7.4) 为 vx/vy/wz 分别设置"已标定/待标定"状态。

本脚本把这份表从"文档里的告诫"变成"CI 里的门禁":

  1. 校验 config/motion_capability.yaml 自身: 状态取值合法、比例区间自洽、
     may/must_not 依赖清单不矛盾、声明可用之轴必须有实测比例。
  2. **交叉核对契约限速**: 声明"导航可依赖"的轴, 其 adapter 限速必须 > 0。
     命中即硬失败 —— 那就是"导航指望这个轴、限速却把它夹成 0"的静默失效。
  3. 声明"不可依赖"的轴限速为 0 是允许的 (显式禁用), 只报 INFO。

退出码:
  0 = PASS (可含 WARN/INFO)
  1 = 能力表非法 或 与契约限速矛盾
  2 = 契约自身非法
  3 = IO / 运行错误

用法:
  python scripts/check_motion_capability.py
  python scripts/check_motion_capability.py --profile gazebo_gait_hover
  python scripts/check_motion_capability.py --axis wz      # 单轴查询 (供门禁脚本用)
  python scripts/check_motion_capability.py --report artifacts/motion_capability.json
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_HERE, _ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

from config.config_loader import (  # noqa: E402
    load_contract, validate_contract, parse_yaml,
)

CAP_REL = "config/motion_capability.yaml"
CONTRACT_REL = "config/simulation_contract.yaml"

VALID_STATUS = ("calibrated", "derated", "uncalibrated")
AXES = ("vx", "vy", "wz")


def _get_path(doc, dotted):
    """按 'adapter.max_linear_y' 取契约里的值。"""
    cur = doc
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def main(argv=None):
    ap = argparse.ArgumentParser(description="vx/vy/wz 运动能力表校验")
    ap.add_argument("--capability", default=CAP_REL)
    ap.add_argument("--contract", default=CONTRACT_REL)
    ap.add_argument("--profile", default=None,
                    help="覆盖 active_profile (用于多平台门禁)")
    ap.add_argument("--axis", choices=AXES, default=None,
                    help="单轴查询: 打印该轴状态并以退出码表达是否可靠")
    ap.add_argument("--report", default="artifacts/motion_capability.json")
    args = ap.parse_args(argv)

    report = {
        "test_name": "motion_capability",
        "status": "FAIL",
        "exit_code": 3,
        "errors": [],
        "warnings": [],
        "info": [],
        "checks": {},
    }
    err = report["errors"].append
    warn = report["warnings"].append
    info = report["info"].append

    abspath = lambda rel: rel if os.path.isabs(rel) else os.path.join(_ROOT, rel)  # noqa: E731

    # ---- 契约 ----
    try:
        contract, sha, sv = load_contract(abspath(args.contract))
    except Exception as e:  # noqa: BLE001
        report["error"] = "load_contract failed: %s" % e
        _dump(report, args.report)
        return 3
    cerr, _ = validate_contract(contract)
    if cerr:
        report["errors"] = list(cerr)
        report["exit_code"] = 2
        _dump(report, args.report)
        return 2

    # ---- 能力表 ----
    with open(abspath(args.capability), encoding="utf-8") as f:
        cap = parse_yaml(f.read()) or {}

    profiles = cap.get("profiles") or {}
    active = args.profile or cap.get("active_profile")
    report["active_profile"] = active
    report["available_profiles"] = sorted(profiles)

    if active not in profiles:
        err("active_profile '%s' 不在 profiles %s 中" % (active, sorted(profiles)))
        _finish(report, args.report)
        return 1

    prof = profiles[active]
    axes = prof.get("axes") or {}
    report["profile_description"] = (prof.get("description") or "").strip()

    # ---- 1. 逐轴校验 ----
    axis_rows = {}
    for ax in AXES:
        a = axes.get(ax)
        if not isinstance(a, dict):
            err("profile '%s' 缺少 axis '%s'" % (active, ax))
            continue
        status = a.get("status")
        if status not in VALID_STATUS:
            err("axis %s: status '%s' 非法 (应为 %s)" % (ax, status, "/".join(VALID_STATUS)))
            continue

        ratio = a.get("measured_ratio")
        usable = a.get("usable_for") or []
        not_usable = a.get("not_usable_for") or []

        # 声明可用却拿不出实测比例 -> 无据, 不允许
        if status in ("calibrated", "derated"):
            if not isinstance(ratio, dict) or ratio.get("min") is None or ratio.get("max") is None:
                err("axis %s: status=%s 必须给出 measured_ratio.min/max" % (ax, status))
            elif float(ratio["min"]) > float(ratio["max"]):
                err("axis %s: measured_ratio.min > max" % ax)
            if not usable:
                warn("axis %s: status=%s 但 usable_for 为空" % (ax, status))
        if status == "uncalibrated" and usable:
            err("axis %s: status=uncalibrated 却声明 usable_for=%s (自相矛盾)" % (ax, usable))
        if set(usable) & set(not_usable):
            err("axis %s: usable_for 与 not_usable_for 有交集: %s"
                % (ax, sorted(set(usable) & set(not_usable))))

        axis_rows[ax] = {
            "status": status,
            "measured_ratio": ratio,
            "rtf_dependent": a.get("rtf_dependent"),
            "usable_for": usable,
            "not_usable_for": not_usable,
        }
    report["axes"] = axis_rows

    # ---- 2. 依赖声明 ----
    may = prof.get("navigation_may_depend_on") or []
    must_not = prof.get("navigation_must_not_depend_on") or []
    report["navigation_may_depend_on"] = may
    report["navigation_must_not_depend_on"] = must_not

    if cap.get("policy", {}).get("require_explicit_dependency_declaration", True):
        if not may and not must_not:
            err("profile '%s' 未声明 navigation_may/must_not_depend_on" % active)
    clash = sorted(set(may) & set(must_not))
    report["checks"]["dependency_declaration_consistent"] = (not clash)
    if clash:
        err("同一轴既在 may 又在 must_not: %s" % clash)

    missing = [a for a in AXES if a not in may and a not in must_not]
    if missing:
        warn("以下轴未出现在任何依赖清单中 (默认按不可依赖处理): %s" % missing)

    # ---- 3. 交叉核对契约限速 ----
    key_map = (cap.get("policy") or {}).get("usable_axis_requires_nonzero_limit") or {
        "vx": "adapter.max_linear_x",
        "vy": "adapter.max_linear_y",
        "wz": "adapter.max_angular_z",
    }
    limits = {}
    for ax in AXES:
        dotted = key_map.get(ax)
        limits[ax] = _get_path(contract, dotted) if dotted else None
    report["contract_limits"] = limits

    contradictions = []
    for ax in AXES:
        declared_usable = ax in may
        lim = limits.get(ax)
        lim_f = float(lim) if isinstance(lim, (int, float)) else None
        row = axis_rows.get(ax, {})
        status = row.get("status")

        if declared_usable and lim_f is not None and lim_f <= 0.0:
            contradictions.append({
                "axis": ax, "status": status, "limit": lim_f,
                "detail": "导航声明可依赖 %s (status=%s), 但 %s = %s 会把该轴夹成 0"
                          % (ax, status, key_map.get(ax), lim_f),
            })
        elif not declared_usable and lim_f is not None and lim_f <= 0.0:
            info("axis %s: 声明不可依赖且限速为 0 -> 显式禁用, 符合预期" % ax)

    report["checks"]["usable_axis_has_nonzero_limit"] = (not contradictions)
    report["contradictions"] = contradictions
    for c in contradictions:
        err(c["detail"])

    # ---- 单轴查询模式 ----
    if args.axis:
        row = axis_rows.get(args.axis, {})
        print(json.dumps({"axis": args.axis, "profile": active, **row},
                         ensure_ascii=False, indent=2))
        reliable = args.axis in may
        _dump(report, args.report)
        return 0 if reliable else 1

    _finish(report, args.report)
    return report["exit_code"]


def _finish(report, report_path):
    report["status"] = "PASS" if not report["errors"] else "FAIL"
    report["exit_code"] = 0 if not report["errors"] else 1
    report["summary"] = ("%s: %s" % (report["active_profile"],
                                     ", ".join("%s=%s" % (a, v.get("status"))
                                               for a, v in sorted(report.get("axes", {}).items()))))
    _dump(report, report_path)
    _print(report)


def _dump(report, path):
    abs_path = path if os.path.isabs(path) else os.path.join(_ROOT, path)
    d = os.path.dirname(abs_path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(abs_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


def _print(report):
    print("=" * 68)
    print("  运动能力表校验 (jihua20260905 §4 P0-3 / §7.4)")
    print("  profile: %s" % report.get("active_profile"))
    print("=" * 68)
    print("  %-4s %-14s %-18s %-6s %s" % ("axis", "status", "measured_ratio", "RTF?", "导航可依赖"))
    may = report.get("navigation_may_depend_on") or []
    for ax, v in sorted(report.get("axes", {}).items()):
        r = v.get("measured_ratio")
        rs = ("%.2f~%.2f" % (float(r["min"]), float(r["max"]))) if isinstance(r, dict) else "-"
        print("  %-4s %-14s %-18s %-6s %s"
              % (ax, v.get("status"), rs,
                 "yes" if v.get("rtf_dependent") else "no",
                 "YES" if ax in may else "no"))
    for name, val in sorted(report["checks"].items()):
        print("  %-38s [%s]" % (name, "PASS" if val else "FAIL"))
    for w in report["warnings"]:
        print("  WARN  %s" % w)
    for i in report["info"]:
        print("  INFO  %s" % i)
    for e in report["errors"]:
        print("  ERROR %s" % e)
    print("-" * 68)
    print("RESULT: %s  (%s)" % (report["status"], report.get("summary", "")))


if __name__ == "__main__":
    sys.exit(main())
