"""根目录清理脚本 (P2-4)

依据 guihua20260809.md P2 阶段要求:
  - 清理根目录临时脚本和生成物
  - 将 _*.sh / _*.py 调试脚本归档到 scripts/archive/
  - 将临时输出文件归档到 archive/temp_output/

用法:
    python scripts/cleanup_root.py --dry-run  # 预览(不移动)
    python scripts/cleanup_root.py            # 执行清理
"""
import os
import shutil
import argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.resolve()
ARCHIVE_DIR = PROJECT_ROOT / "scripts" / "archive"
TEMP_ARCHIVE = PROJECT_ROOT / "archive" / "temp_output"

# 临时脚本模式 (以 _ 开头的 .sh / .py 文件)
TEMP_SCRIPT_PATTERNS = [
    "_*.sh",
    "_*.py",
]

# 临时输出文件模式
TEMP_OUTPUT_PATTERNS = [
    "*.log",
    "*.log.err",
    "*.txt",       # 临时文本输出 (test_py.txt, test_echo.txt, etc.)
    "*.jpg",       # 临时测试图片 (mjpeg_test*.jpg, snap_test.jpg)
    "*.gv",        # TF 调试文件
    "*.pdf",       # TF 调试文件
    "reattach_*",  # 重新附着调试输出
    "coppelia_log.txt",
    "base_footprint",  # 临时文件
]

# 保留的核心文件 (不移动)
KEEP_FILES = {
    # 核心模块
    "amcl.py", "occupancy_grid.py", "costmap.py", "astar_planner.py",
    "dwa_planner.py", "teb_planner.py", "odometry.py",
    "vo_avoidance.py", "orca_avoidance.py", "apf_avoidance.py",
    "stvoc_avoidance.py", "rvo_avoidance.py", "sfm_avoidance.py",
    "cbf_safety.py", "dstar_lite.py", "hybrid_astar.py",
    "mpc_planner.py", "gnn_trajectory_predictor.py",
    # 实验脚本
    "compare_algorithms.py", "ablation_study_v3.py", "test_performance.py",
    "auto_patrol_simulation.py", "long_patrol_simulation.py",
    "ablation_study.py", "ablation_study_v2.py",
    "parameter_sensitivity.py", "stress_test_algorithms.py",
    "benchmark_pipeline.py", "run_all_experiments.py",
    "evaluator.py", "failure_analysis.py", "pareto.py",
    # 基础设施
    "metrics_schema.py", "test_metrics_schema.py",
    "latency_monitor.py", "unified_config.py", "config_loader.py",
    "ci_pipeline.py", "test_regression_scenarios.py",
    # 避障/安全
    "dynamic_obstacles.py", "safety_subsystem.py",
    # SLAM/探索
    "active_slam.py", "lifelong_slam.py", "rbpf_slam.py",
    "semantic_slam.py", "semantic_info_gain.py",
    "loop_closure.py", "unified_uncertainty.py",
    # 规划器
    "risk_aware_astar.py", "belief_planner.py", "pomdp_navigation.py",
    "aufe_framework.py", "rl_agent.py",
    # 其他核心
    "vision.py", "visual_sim.py", "visual_path_planner.py",
    "random_walk.py", "baselines.py", "patrol.py",
    "regression_test.py", "long_duration_test.py",
    "open_field_test.py", "theory_analysis.py",
    # ROS2
    "load_ros2_plugin.py", "attach_script.py",
}


def find_temp_scripts():
    """找到临时脚本文件。"""
    scripts = []
    for pattern in TEMP_SCRIPT_PATTERNS:
        for f in PROJECT_ROOT.glob(pattern):
            if f.name not in KEEP_FILES:
                scripts.append(f)
    return scripts


def find_temp_outputs():
    """找到临时输出文件。"""
    outputs = []
    for pattern in TEMP_OUTPUT_PATTERNS:
        for f in PROJECT_ROOT.glob(pattern):
            if f.name not in KEEP_FILES and f.is_file():
                outputs.append(f)
    return outputs


def archive_files(files, dest_dir, dry_run=True):
    """归档文件到目标目录。"""
    dest_dir.mkdir(parents=True, exist_ok=True)
    moved = 0
    for f in files:
        dest = dest_dir / f.name
        if dest.exists():
            # 避免覆盖，跳过
            continue
        if dry_run:
            print(f"  [DRY-RUN] {f.name} -> {dest.relative_to(PROJECT_ROOT)}")
        else:
            shutil.move(str(f), str(dest))
            print(f"  [MOVED] {f.name}")
        moved += 1
    return moved


def main():
    parser = argparse.ArgumentParser(description="根目录清理 (P2-4)")
    parser.add_argument("--dry-run", action="store_true",
                        help="预览模式，不实际移动文件")
    args = parser.parse_args()

    print(f"项目根目录: {PROJECT_ROOT}")
    print(f"归档目录: {ARCHIVE_DIR}")
    print(f"临时输出归档: {TEMP_ARCHIVE}")
    print(f"模式: {'预览' if args.dry_run else '执行'}")
    print()

    # 1. 临时脚本
    temp_scripts = find_temp_scripts()
    print(f"=== 临时脚本 ({len(temp_scripts)} 个) ===")
    n1 = archive_files(temp_scripts, ARCHIVE_DIR, args.dry_run)

    # 2. 临时输出
    temp_outputs = find_temp_outputs()
    print(f"\n=== 临时输出 ({len(temp_outputs)} 个) ===")
    n2 = archive_files(temp_outputs, TEMP_ARCHIVE, args.dry_run)

    print(f"\n=== 汇总 ===")
    print(f"  临时脚本: {n1} 个")
    print(f"  临时输出: {n2} 个")
    print(f"  总计: {n1 + n2} 个文件{'将' if args.dry_run else '已'}被归档")

    if args.dry_run and (n1 + n2) > 0:
        print(f"\n  使用 --dry-run=False 执行实际清理")
    elif not args.dry_run:
        print(f"\n  [完成] 根目录清理完毕")


if __name__ == "__main__":
    main()
