#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_baseline.py — 生成可复现的发布基线 artifacts (jihua20260905.md §4 / P0-1)
================================================================================
把"当前版本到底通过了什么"冻结到 artifacts/<baseline_id>/, 任何结论都能追溯到:

    - environment.json        : 平台/Python/Git SHA; WSL/UE/GPU 标为 not-measured
    - config_hashes.json      : 关键配置 SHA-256
    - config_consistency.json : check_config_consistency.py 结果
    - scene_validation.txt     : validate_scene.py 结果
    - python_pytest.json       : pytest 结构化结果
    - metadata.json           : 运行时间、命令、退出码、阈值版本

基线目录只读、不被后续运行覆盖 (本脚本带 --force 才重写)。

用法:
    python scripts/make_baseline.py                       # 默认 baseline-20260905
    python scripts/make_baseline.py --id baseline-20260905 --force
"""
import os
import sys
import json
import glob
import shutil
import argparse
import subprocess
import hashlib
import platform
import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _sha256(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except FileNotFoundError:
        return None


def _git_sha():
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=_ROOT,
            stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return "unavailable"


def _git_describe():
    try:
        out = subprocess.check_output(
            ["git", "describe", "--tags", "--always"], cwd=_ROOT,
            stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return "unavailable"


def _pytest_python():
    """找到装了 pytest 的解释器, 避免 `python` 指向无 pytest 的运行时导致基线
    pytest 误记 exit=1 / null。优先 sys.executable, 其次 PATH 上的 python3/python,
    再回退到本机已知装有项目 dev 依赖的系统 Python (jihua20260905.md P0-1 可复现性)。"""
    import shutil
    candidates = [sys.executable, "python3", "python"]
    for cand in ("C:/Program Files/Python312/python.exe",
                "C:/Program Files/Python312/pythonw.exe"):
        if os.path.exists(cand) and cand not in candidates:
            candidates.append(cand)
    seen = set()
    for c in candidates:
        if c in seen:
            continue
        seen.add(c)
        exe = shutil.which(c) if c in ("python3", "python") else c
        if not exe:
            continue
        try:
            r = subprocess.run([exe, "-m", "pytest", "--version"],
                               capture_output=True, text=True, timeout=30)
            if r.returncode == 0:
                return exe
        except Exception:  # noqa: BLE001
            pass
    return None


def _run(cmd, cwd):
    """返回 (returncode, stdout_text)。失败不抛异常, 留给调用方记录。"""
    try:
        p = subprocess.run(cmd, cwd=cwd, capture_output=True,
                           text=True, timeout=600)
        return p.returncode, p.stdout + p.stderr
    except Exception as e:  # noqa: BLE001
        return -1, "subprocess error: %s" % e


def _find_bridge():
    """定位 nav_ue_bridge.exe; 未编译返回 None (诚实标记 not-measured)。"""
    cands = [
        os.path.join(_ROOT, "cpp_src", "bridge", "build", "Release", "nav_ue_bridge.exe"),
        os.path.join(_ROOT, "cpp_src", "bridge", "build", "nav_ue_bridge.exe"),
        os.path.join(_ROOT, "cpp_src", "bridge", "nav_ue_bridge.exe"),
    ]
    for c in cands:
        if os.path.exists(c):
            return c
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="生成发布基线 artifacts")
    ap.add_argument("--id", default="baseline-20260905")
    ap.add_argument("--out-root", default="artifacts")
    ap.add_argument("--force", action="store_true",
                    help="覆盖已存在的基线目录 (默认只读保护)")
    args = ap.parse_args(argv)

    out_dir = os.path.join(_ROOT, args.out_root, args.id)
    if os.path.exists(out_dir) and not args.force:
        print("[ABORT] %s 已存在; 基线只读, 用 --force 重写" % out_dir)
        return 10
    os.makedirs(out_dir, exist_ok=True)

    now = datetime.datetime.now(datetime.timezone.utc).isoformat()

    # ---- environment.json ----
    env = {
        "generated_utc": now,
        "baseline_id": args.id,
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "python_version": platform.python_version(),
        },
        "git": {
            "sha": _git_sha(),
            "describe": _git_describe(),
        },
        "not_measured_here": {
            "wsl_ros2": "在 WSL/ROS2 Humble 环境单独复现 (scripts/run_regression.sh)",
            "ue5_bridge": "在 Windows + UE5.3 + GPU 环境单独复现 (bridge --check-scene)",
            "cpp_build": "需本地 C++ 工具链, 见 cpp_src CMake",
        },
    }
    with open(os.path.join(out_dir, "environment.json"), "w", encoding="utf-8") as f:
        json.dump(env, f, indent=2, ensure_ascii=False)

    # ---- config_hashes.json ----
    config_files = [
        "config/scene_home.json",
        "config/simulation_contract.yaml",
        "config/unified_params.yaml",
        "config/runtime.yaml",
        "config/profile_raspberry_pi.yaml",
        # jihua20260905 §4 P0-3 新增的单一真源 (几何/运动能力), 必须一起锁哈希
        "config/geometry_spec.yaml",
        "config/motion_capability.yaml",
        "INTERFACES.md",
        "cpp_src/bridge/protocol.h",
    ]
    hashes = {}
    for cf in config_files:
        p = os.path.join(_ROOT, cf)
        h = _sha256(p)
        if h is not None:
            hashes[cf] = h
    with open(os.path.join(out_dir, "config_hashes.json"), "w", encoding="utf-8") as f:
        json.dump(hashes, f, indent=2, ensure_ascii=False)

    # ---- config_consistency.json ----
    rc, out = _run(
        [sys.executable, "scripts/check_config_consistency.py",
         "--report", os.path.join(out_dir, "config_consistency.json")],
        cwd=_ROOT)

    # ---- scene_validation.txt ----
    rc_s, out_s = _run(
        [sys.executable, "scripts/validate_scene.py"], cwd=_ROOT)
    with open(os.path.join(out_dir, "scene_validation.txt"), "w", encoding="utf-8") as f:
        f.write(out_s)

    # ---- geometry_consistency.json (§4 P0-3 出口标准) ----
    # 锁死 scene/contract/URDF/Nav2/UE capsule 几何单一真源, 防止 0.25 vs 0.35 这类
    # "偷安全"的隐性漂移在发布后才被发现。
    rc_g, out_g = _run(
        [sys.executable, "scripts/check_geometry_consistency.py",
         "--report", os.path.join(out_dir, "geometry_consistency.json")],
        cwd=_ROOT)
    with open(os.path.join(out_dir, "geometry_consistency.txt"), "w", encoding="utf-8") as f:
        f.write(out_g)

    # ---- motion_capability.json (§4 P0-3 第4条 / §7.4) ----
    # 记录 vx/vy/wz 各轴是否已标定, 禁止把未标定轴当可靠导航能力。
    rc_mc, out_mc = _run(
        [sys.executable, "scripts/check_motion_capability.py",
         "--report", os.path.join(out_dir, "motion_capability.json")],
        cwd=_ROOT)
    with open(os.path.join(out_dir, "motion_capability.txt"), "w", encoding="utf-8") as f:
        f.write(out_mc)

    # ---- python_pytest.json ----
    # 仓库根无 pytest 配置, 直接 `pytest -q` 会扫到全仓其它不可跑的 test_*.py
    # (nav_core / robot_src / 根级), 得到混乱数字。固定指向已知可跑的
    # ROS2 适配器套件 src/puppy_minicpm_robot/test (即计划 §2.2 的 76/1)。
    suite_dir = os.path.join(_ROOT, "src", "puppy_minicpm_robot")
    os.environ["PYTHONPATH"] = suite_dir + os.pathsep + os.environ.get("PYTHONPATH", "")
    json_path = os.path.join(out_dir, "python_pytest.json")
    pytest_py = _pytest_python()
    if pytest_py is None:
        # 诚实失败: 不假装测试通过, 也不写混乱数字。明确告知需要装有 dev 依赖的解释器。
        rep = {
            "summary": {
                "passed": None, "skipped": 0, "failed": 0,
                "exit_code": 127, "blocked": True,
                "reason": "pytest 未安装在任何可用解释器中; 请用装有项目 dev 依赖的 Python "
                          "(如系统 Python 3.12) 运行 make_baseline.py",
            },
            "raw_tail": "",
        }
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(rep, f, indent=2, ensure_ascii=False)
        rc_t = 127
    else:
        # 优先用 pytest-json-report 插件; 不可用时回退到解析 -q 摘要行。
        rc_t, out_t = _run(
            [pytest_py, "-m", "pytest", "test", "-q", "--no-header",
             "-p", "no:cacheprovider",
             "--json-report", "--json-report-file", json_path],
            cwd=suite_dir)
        if rc_t != 0 or not os.path.exists(json_path):
            rc_t, out_t = _run(
                [pytest_py, "-m", "pytest", "test", "-q", "--no-header",
                 "-p", "no:cacheprovider"],
                cwd=suite_dir)
            import re
            m = re.search(
                r"(\d+)\s+passed(?:,\s*(\d+)\s+skipped)?"
                r"(?:,\s*(\d+)\s+failed)?", out_t)
            passed = int(m.group(1)) if m else None
            skipped = int(m.group(2)) if (m and m.group(2)) else 0
            failed = int(m.group(3)) if (m and m.group(3)) else 0
            rep = {
                "summary": {
                    "passed": passed, "skipped": skipped, "failed": failed,
                    "exit_code": rc_t,
                },
                "raw_tail": out_t[-2000:],
            }
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(rep, f, indent=2, ensure_ascii=False)
            # 同时保留纯文本便于人工查看
            with open(os.path.join(out_dir, "python_pytest.txt"), "w", encoding="utf-8") as f:
                f.write(out_t)

    # ---- cpp_results/ (§4 P0-1: C++ 帧结果 + 结构化 JSON + 时延 csv) ----
    # 不重跑实验 (长稳耗时长), 只把 artifacts/ 下已有的 Plan B 结果冻结进基线,
    # 并生成索引便于不看聊天记录也能回答"当前版本跑过哪些帧数/种子"。
    cpp_index = {"count": 0, "runs": [], "note": None, "timing_csvs": []}
    planb_files = sorted(glob.glob(
        os.path.join(_ROOT, "artifacts", "cpp_*_planB.json")))
    if planb_files:
        cpp_dir = os.path.join(out_dir, "cpp_results")
        os.makedirs(cpp_dir, exist_ok=True)
        for src in planb_files:
            shutil.copy2(src, os.path.join(cpp_dir, os.path.basename(src)))
            # 同名 timing csv (frame,ms) 一并冻结
            tcsv = src[:-len(".json")] + ".timing.csv"
            if os.path.exists(tcsv):
                shutil.copy2(tcsv, os.path.join(cpp_dir, os.path.basename(tcsv)))
                cpp_index["timing_csvs"].append(os.path.basename(tcsv))
            try:
                d = json.load(open(src, encoding="utf-8"))
                m = d.get("metrics", {})
                cpp_index["runs"].append({
                    "file": os.path.basename(src),
                    "test_name": d.get("test_name"),
                    "seed": d.get("seed"),
                    "frames": d.get("frames"),
                    "status": d.get("status"),
                    "exit_code": d.get("exit_code"),
                    "acceptance_level": d.get("acceptance_level"),
                    "config_hash": d.get("config_hash"),
                    "astar_rate_pct": m.get("astar_rate_pct"),
                    "amcl_avg_err_m": m.get("amcl_avg_err_m"),
                    "amcl_max_err_m": m.get("amcl_max_err_m"),
                    "collisions": m.get("collisions"),
                    "rooms_visited": m.get("rooms_visited"),
                    "total_distance_m": m.get("total_distance_m"),
                    "stuck_ratio_pct": m.get("stuck_ratio_pct"),
                })
            except Exception:  # noqa: BLE001
                cpp_index["runs"].append({"file": os.path.basename(src),
                                          "parse_error": True})
        cpp_index["count"] = len(cpp_index["runs"])
    else:
        cpp_index["note"] = ("NO DATA: artifacts/ 下无 cpp_*_planB.json；"
                             "需先跑 C++ 长稳 (scripts/run_regression.sh 或 sim_test)")
    with open(os.path.join(out_dir, "cpp_results.json"), "w", encoding="utf-8") as f:
        json.dump(cpp_index, f, indent=2, ensure_ascii=False)

    # ---- ue_bridge_checks.txt (§4 P0-1: bridge --help/--check-scene) ----
    bridge = _find_bridge()
    lines = []
    lines.append("# UE bridge 基线检查 (jihua20260905 §4 P0-1)")
    lines.append("generated_utc: %s" % now)
    lines.append("git_sha: %s" % env["git"]["sha"])
    lines.append("")
    if bridge is None:
        lines.append("RESULT: NOT MEASURED")
        lines.append("")
        lines.append("原因: 未找到 nav_ue_bridge.exe (需先用 MSVC 编译 cpp_src/bridge)。")
        lines.append("本基线不假装该检查通过; 复现命令:")
        lines.append("  source scripts/msvc_env.sh && cl.exe /O2 /std:c++17 /EHsc /utf-8 ...")
        lines.append("  nav_ue_bridge.exe --check-scene --scene config/scene_home.json")
        rc_b = None
    else:
        lines.append("bridge: %s" % bridge)
        lines.append("")
        # 注意: bridge 的参数形式是 `--check-scene <path>` (路径直接跟在后面),
        # 不是 `--check-scene --scene <path>`。写错会变成
        # "[FATAL] scene file not found: --scene" 并被误记成检查失败。
        for label, extra in (("--help", ["--help"]),
                             ("--check-scene",
                              ["--check-scene",
                               os.path.join(_ROOT, "config", "scene_home.json")])):
            rc_x, out_x = _run([bridge] + extra, cwd=_ROOT)
            lines.append("=== %s (exit=%s) ===" % (label, rc_x))
            lines.append((out_x or "").strip()[:4000])
            lines.append("")
            if label == "--check-scene":
                rc_b = rc_x
        rc_b = rc_b if rc_b is not None else 0
    with open(os.path.join(out_dir, "ue_bridge_checks.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # ---- metadata.json ----
    meta = {
        "baseline_id": args.id,
        "generated_utc": now,
        "git_sha": env["git"]["sha"],
        "pytest_interpreter": pytest_py,
        "commands": {
            "config_consistency": "python scripts/check_config_consistency.py",
            "scene_validation": "python scripts/validate_scene.py",
            "geometry_consistency": "python scripts/check_geometry_consistency.py",
            "motion_capability": "python scripts/check_motion_capability.py",
            "python_pytest": "python -m pytest -q (suite: src/puppy_minicpm_robot/test)",
            "cpp_results": "冻结 artifacts/cpp_*_planB.json (不重跑, 见 cpp_results.json)",
            "ue_bridge_checks": "nav_ue_bridge.exe --help / --check-scene",
        },
        "exit_codes": {
            "config_consistency": rc,
            "scene_validation": rc_s,
            "geometry_consistency": rc_g,
            "motion_capability": rc_mc,
            "python_pytest": rc_t,
            "ue_bridge_check_scene": rc_b,
        },
        "cpp_results_frozen": cpp_index["count"],
        "threshold_version": "scene_home.json v6.0 / simulation_contract schema_version 1",
        "min_required_rooms": 4,
    }
    with open(os.path.join(out_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print("[OK] 基线已写入: %s" % out_dir)
    for k, v in meta["exit_codes"].items():
        print("  %-18s exit=%s" % (k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
