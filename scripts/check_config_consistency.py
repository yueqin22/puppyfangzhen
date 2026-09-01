#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_config_consistency.py — 跨配置一致性校验 (M4 / §4.3 / §4.4)
================================================================
依据 jihua20260818.md 模块一: 统一配置必须是单一事实源。

职责:
  1. 校验 simulation_contract.yaml 自身合法性 (复用 config.config_loader)。
  2. 跨 config/ 下所有 YAML 核对 robot.radius 一致 (黑盒一致性)。
  3. (可选) 比对 scene_home.json 与契约的 robot.radius 一致。

退出码:
  0 = 全部一致
  1 = 契约自身非法 (validate_contract 返回 errors)
  2 = 跨配置 robot.radius 不一致
  3 = IO / 运行错误

输出结构化 JSON 到 --report。
"""
import sys
import os
import json
import glob
import argparse

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from config.config_loader import (
    load_contract, validate_contract, _extract_robot_radius_yaml,
)


def _dump(report, path):
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description="跨配置一致性校验")
    ap.add_argument("--contract", default="config/simulation_contract.yaml")
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--report", default="artifacts/config_consistency.json")
    ap.add_argument("--radius-files", nargs="*", default=None,
                    help="参与 robot.radius 核对的 YAML; 默认自动扫描 config/*.yaml")
    args = ap.parse_args(argv)

    report = {
        "test_name": "config_consistency",
        "status": "FAIL",
        "exit_code": 3,
        "contract_path": args.contract,
        "contract_valid": False,
        "contract_errors": [],
        "contract_warnings": [],
        "radius_consistency": {},
        "radius_mismatches": [],
        "cross_config_pass": True,
        "checked_files": [],
    }

    try:
        # 1. 契约自身合法性
        try:
            data, sha, sv = load_contract(args.contract)
        except Exception as e:
            report["error"] = "load_contract failed: %s" % e
            _dump(report, args.report)
            return 3

        report["schema_version"] = sv
        report["sha256"] = sha
        errors, warnings = validate_contract(data)
        report["contract_valid"] = (len(errors) == 0)
        report["contract_errors"] = errors
        report["contract_warnings"] = warnings
        if errors:
            report["status"] = "FAIL"
            report["exit_code"] = 1
            report["summary"] = "契约自身非法: %d 个错误" % len(errors)
            _dump(report, args.report)
            return 1

        # 契约中的 robot.radius
        snap_r = None
        try:
            snap_r = float(_extract_robot_radius_yaml(args.contract))
        except Exception:
            snap_r = None
        report["contract_radius"] = snap_r

        # 2. 跨配置 robot.radius 核对
        files = args.radius_files
        if files is None:
            files = sorted(glob.glob(os.path.join(args.config_dir, "*.yaml")))
        mism = []
        for f in files:
            if not os.path.exists(f):
                continue
            r = _extract_robot_radius_yaml(f)
            if r is None:
                continue
            name = os.path.basename(f)
            report["radius_consistency"][name] = r
            report["checked_files"].append(name)
            if snap_r is not None and abs(r - snap_r) > 1e-9:
                mism.append({"file": name, "radius": r, "contract_radius": snap_r})
        report["radius_mismatches"] = mism
        if mism:
            report["cross_config_pass"] = False
            report["status"] = "FAIL"
            report["exit_code"] = 2
            report["summary"] = "robot.radius 跨配置不一致: %d 个文件" % len(mism)
            _dump(report, args.report)
            return 2

        report["status"] = "PASS"
        report["exit_code"] = 0
        report["summary"] = "契约合法, %d 个配置文件 robot.radius 一致 (%.3f)" % (
            len(report["checked_files"]), snap_r if snap_r is not None else 0.0)
        _dump(report, args.report)
        return 0

    except Exception as e:  # noqa: BLE001
        report["error"] = str(e)
        report["status"] = "FAIL"
        report["exit_code"] = 3
        _dump(report, args.report)
        return 3


if __name__ == "__main__":
    sys.exit(main())
