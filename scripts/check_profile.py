#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_profile.py — 校验 unified_params.yaml 的激活 profile 与 profile 定义 (jihua20260905.md §11 第 1 周出口)

职责:
  1. 激活 profile (顶部 `profile:` 字段) 必须存在于 `profiles:` 定义中。
  2. 每个 profile 必须具备完整字段: description / acceptance_level / default_frames /
     backend / allow_mock(nav_core,vision,puppypi) / enforce_single_source / require_seed_sweep。
  3. acceptance_level 必须是 SMOKE/INTEGRATION/REGRESSION/RELEASE 之一。
  4. 可选 --assert-release: 仅当激活档为 release 且 enforce_single_source=true 时通过
     (用于发布门禁, 防止"配置漂移悄悄过关")。

用法:
  python scripts/check_profile.py [--unified config/unified_params.yaml] [--assert-release] [--report out.json]

退出码: 0=通过, 1=配置错误, 2=用法/文件错误
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "config"))

from config.config_loader import parse_yaml  # noqa: E402

VALID_LEVELS = ("SMOKE", "INTEGRATION", "REGRESSION", "RELEASE")
REQUIRED_KEYS = (
    "description",
    "acceptance_level",
    "default_frames",
    "backend",
    "allow_mock",
    "enforce_single_source",
    "require_seed_sweep",
)
ALLOW_MOCK_KEYS = ("nav_core", "vision", "puppypi")


def _truthy(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes")
    return bool(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unified", default=os.path.join(ROOT, "config", "unified_params.yaml"))
    ap.add_argument("--assert-release", action="store_true",
                    help="仅当激活档=release 且 enforce_single_source=true 时通过")
    ap.add_argument("--report", default=None, help="写出 JSON 报告路径")
    args = ap.parse_args()

    errors = []
    warnings = []
    report = {"errors": errors, "warnings": warnings}

    if not os.path.exists(args.unified):
        errors.append("unified_params.yaml 不存在: %s" % args.unified)
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 2

    try:
        doc = parse_yaml(open(args.unified, encoding="utf-8").read())
    except Exception as e:  # pragma: no cover
        errors.append("解析 unified_params.yaml 失败: %s" % e)
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 2

    active = doc.get("profile")
    profiles = doc.get("profiles") or {}
    report["active_profile"] = active
    report["available_profiles"] = sorted(profiles)

    if not profiles:
        errors.append("unified_params.yaml 缺少 `profiles:` 定义块")
    if active is None:
        errors.append("unified_params.yaml 缺少顶部 `profile:` 激活字段")
    elif active not in profiles:
        errors.append("激活 profile '%s' 不在 profiles %s 中" % (active, sorted(profiles)))

    # ---- 逐个 profile 结构校验 ----
    for name, prof in (profiles or {}).items():
        if not isinstance(prof, dict):
            errors.append("profile '%s' 不是映射" % name)
            continue
        for key in REQUIRED_KEYS:
            if key not in prof:
                errors.append("profile '%s' 缺少字段 '%s'" % (name, key))
        lvl = prof.get("acceptance_level")
        if lvl is not None and lvl not in VALID_LEVELS:
            errors.append("profile '%s' acceptance_level '%s' 非法 (须为 %s)"
                          % (name, lvl, "/".join(VALID_LEVELS)))
        am = prof.get("allow_mock")
        if not isinstance(am, dict):
            errors.append("profile '%s' allow_mock 必须是映射" % name)
        else:
            for k in ALLOW_MOCK_KEYS:
                if k not in am:
                    errors.append("profile '%s' allow_mock 缺少 '%s'" % (name, k))
            bad = am.get("puppypi")
            if bad is not None and bad not in ("sim", "mock", "real"):
                errors.append("profile '%s' puppypi 必须是 sim/mock/real 之一 (现为 %r)"
                              % (name, bad))
        df = prof.get("default_frames")
        if df is not None and not isinstance(df, int):
            try:
                int(df)
            except (TypeError, ValueError):
                errors.append("profile '%s' default_frames 必须是整数" % name)

    # ---- 激活档有效性摘要 ----
    if active in profiles:
        aprof = profiles[active]
        report["active_config"] = {
            "description": aprof.get("description"),
            "acceptance_level": aprof.get("acceptance_level"),
            "default_frames": aprof.get("default_frames"),
            "backend": aprof.get("backend"),
            "allow_mock": aprof.get("allow_mock"),
            "enforce_single_source": _truthy(aprof.get("enforce_single_source")),
            "require_seed_sweep": _truthy(aprof.get("require_seed_sweep")),
        }
        if _truthy(aprof.get("enforce_single_source")):
            warnings.append("激活档 '%s' 强制单一真源校验 (geometry/motion/config 门禁必须全绿)"
                            % active)
        if aprof.get("allow_mock", {}).get("nav_core") in (True, "true"):
            errors.append("激活档 '%s' 不允许 nav_core=mock (确定性导航核不得被 mock 替代)"
                          % active)

    # ---- --assert-release 约束 ----
    if args.assert_release:
        if active != "release":
            errors.append("--assert-release 要求激活档为 release (现为 %r)" % active)
        elif not _truthy((profiles.get("release") or {}).get("enforce_single_source")):
            errors.append("release 档 enforce_single_source 必须为 true")

    if args.report:
        report["passed"] = not errors
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

    # ---- 输出 ----
    if errors:
        print("== [FAIL] profile 校验 ==", file=sys.stderr)
        for e in errors:
            print("  - %s" % e, file=sys.stderr)
    else:
        print("== [PASS] profile 校验 ==")
        print("  激活档: %s (%s) | 后端: %s | 默认帧数: %s | 强制单一真源: %s"
              % (active, report["active_config"]["acceptance_level"],
                 report["active_config"]["backend"],
                 report["active_config"]["default_frames"],
                 report["active_config"]["enforce_single_source"]))
    for w in warnings:
        print("  ! %s" % w, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
