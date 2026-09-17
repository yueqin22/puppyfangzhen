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
    load_contract, validate_contract, _extract_robot_radius_yaml, parse_yaml,
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
        "scene_consistency": {},
        "scene_error": None,
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

        # 3. scene_home.json 与契约 robot.radius 一致 (P0-3 单一真源:
        #    scene 的规划/CBF 半径不得私自漂移出契约声明的安全基线)
        scene_path = os.path.join(args.config_dir, "scene_home.json")
        if os.path.exists(scene_path):
            try:
                with open(scene_path, encoding="utf-8") as sf:
                    scene = json.load(sf)
                srad = scene.get("robot", {}) or {}
                for key in ("radius_planning", "radius_cbf"):
                    v = srad.get(key)
                    if isinstance(v, (int, float)):
                        scene_consistency = report.setdefault("scene_consistency", {})
                        scene_consistency[key] = v
                        report["checked_files"].append("scene_home.json:%s" % key)
                        if snap_r is not None and abs(float(v) - snap_r) > 1e-9:
                            mism.append({"file": "scene_home.json:%s" % key,
                                          "radius": float(v),
                                          "contract_radius": snap_r})
            except Exception as e:  # noqa: BLE001
                report["scene_error"] = "load scene_home.json failed: %s" % e

        report["radius_mismatches"] = mism
        if mism:
            report["cross_config_pass"] = False
            report["status"] = "FAIL"
            report["exit_code"] = 2
            report["summary"] = "robot.radius 跨配置/场景不一致: %d 处" % len(mism)
            _dump(report, args.report)
            return 2

        # 4. P0-4: 契约 adapter 段 vs adapter_params.yaml 一致
        #    cmd_vel 超时与运动限速必须是单一真源, 防止 ROS2 参数与契约漂移
        #    (契约 adapter.cmd_vel_timeout_sec <-> adapter_params.yaml cmd_timeout)
        contract_adapter = (data.get("adapter") or {}) if data else {}
        if contract_adapter:
            adapter_mism = []
            # 契约字段 -> (参数文件里的键名, 契约键名)
            key_map = [
                ("cmd_vel_timeout_sec", "cmd_timeout"),
                ("max_linear_x", "max_linear_x"),
                ("max_linear_y", "max_linear_y"),
                ("max_angular_z", "max_angular_z"),
            ]
            adapter_files = [
                os.path.join("src", "puppypi_adapter", "config",
                             "adapter_params.yaml"),
                os.path.join("cpp_src", "puppypi_adapter", "config",
                             "adapter_params.yaml"),
            ]
            checked_adapter = {}
            for rel in adapter_files:
                path = os.path.join(_ROOT, rel)
                if not os.path.exists(path):
                    continue
                try:
                    import yaml
                    with open(path, encoding="utf-8") as af:
                        doc = yaml.safe_load(af) or {}
                except Exception as e:  # noqa: BLE001
                    report.setdefault("adapter_errors", []).append(
                        "%s: %s" % (rel, e))
                    continue
                params = ((doc.get("motion_adapter") or {})
                          .get("ros__parameters") or {})
                if not params:
                    continue
                for ckey, pkey in key_map:
                    cv = contract_adapter.get(ckey)
                    pv = params.get(pkey)
                    if not isinstance(cv, (int, float)) or \
                            not isinstance(pv, (int, float)):
                        continue
                    checked_adapter.setdefault(rel, {})[pkey] = pv
                    report["checked_files"].append("%s:%s" % (rel, pkey))
                    if abs(float(cv) - float(pv)) > 1e-9:
                        adapter_mism.append({
                            "file": "%s:%s" % (rel, pkey),
                            "value": float(pv),
                            "contract_value": float(cv),
                        })
            report["adapter_consistency"] = {
                "contract": {k: contract_adapter.get(k)
                             for k, _ in key_map
                             if k in contract_adapter},
                "params": checked_adapter,
            }
            report["adapter_mismatches"] = adapter_mism
            if adapter_mism:
                report["cross_config_pass"] = False
                report["status"] = "FAIL"
                report["exit_code"] = 2
                report["summary"] = "adapter 参数与契约不一致: %d 处" % len(adapter_mism)
                _dump(report, args.report)
                return 2

        # 5. The legacy unified_params.yaml is still consumed by a few
        # analysis/C++ entry points.  It must mirror safety-owned values or a
        # profile can silently run with a different speed/timeout/inflation.
        unified_path = os.path.join(args.config_dir, "unified_params.yaml")
        if os.path.exists(unified_path):
            try:
                with open(unified_path, encoding="utf-8") as uf:
                    unified = parse_yaml(uf.read()) or {}
                u_robot = unified.get("robot") or {}
                u_costmap = unified.get("costmap") or {}
                contract_checks = {
                    "robot.max_linear_x": (u_robot.get("max_linear_x"),
                                           contract_adapter.get("max_linear_x")),
                    "robot.max_angular_z": (u_robot.get("max_angular_z"),
                                             contract_adapter.get("max_angular_z")),
                    "robot.cmd_timeout": (u_robot.get("cmd_timeout"),
                                           contract_adapter.get("cmd_vel_timeout_sec")),
                    "costmap.inflation_radius": (
                        u_costmap.get("inflation_radius"),
                        (data.get("planning") or {}).get("inflation_radius")),
                }
                unified_mismatches = []
                report["unified_consistency"] = {}
                for key, (actual, expected) in contract_checks.items():
                    report["unified_consistency"][key] = {
                        "value": actual, "contract_value": expected}
                    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
                        if abs(float(actual) - float(expected)) > 1e-9:
                            unified_mismatches.append({
                                "file": "config/unified_params.yaml:" + key,
                                "value": actual, "contract_value": expected})
                report["unified_mismatches"] = unified_mismatches
                report["checked_files"].append("unified_params.yaml")
                if unified_mismatches:
                    report["cross_config_pass"] = False
                    report["status"] = "FAIL"
                    report["exit_code"] = 2
                    report["summary"] = (
                        "unified_params.yaml 与 simulation_contract.yaml 不一致: %d 项"
                        % len(unified_mismatches))
                    _dump(report, args.report)
                    return 2
            except Exception as e:  # noqa: BLE001
                report["unified_error"] = str(e)
                report["status"] = "FAIL"
                report["exit_code"] = 3
                _dump(report, args.report)
                return 3

        report["status"] = "PASS"
        report["exit_code"] = 0
        report["summary"] = "契约合法, %d 个配置/场景 robot.radius 一致 (%.3f)" % (
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
