"""4种避障算法对比测试脚本 v3 - 长时多轮对比

依据 guihua20260809.md P0 阶段要求重构:
  - P0-1: 修复日志解析失败处理，不再返回 -1/Infinity
  - P0-2: 禁止 -1/None/NaN/Infinity 进入统计
  - P0-3: 保存每次 trial 的随机种子和完整配置
  - P0-4: 明确标注并隔离 demo / official 结果
  - P0-6: 增加 status / failure_reason / missing_fields 字段
  - P0-7: 固定场景、初始位姿、障碍物轨迹和仿真步长

循环巡航 N 轮，累计碰撞数据，获得统计意义。
"""
import os
import sys
import json
import time
import subprocess
import re
import datetime

# 引入统一指标模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics_schema import (
    TrialResult, create_invalid_result, aggregate_valid_results,
    is_invalid_metric_value,
    get_results_dir, make_experiment_id, save_trial_config,
    mark_demo_results,
)


# =====================================================================
# 固定实验配置 (P0-7: 固定场景/初始位姿/障碍物轨迹/仿真步长)
# =====================================================================
SCENARIO_NAME = "dynamic_crossing"
SIM_DT = 0.0333  # 仿真步长 30Hz
INITIAL_POSE = {"x": 0.0, "y": 0.0, "yaw": 0.0}
# 固定障碍物轨迹种子，保证不同算法使用完全相同的障碍物运动
OBSTACLE_SEED_BASE = 20260809
# 巡航路径点（固定，保证不同算法跑同一组目标点）
PATROL_WAYPOINTS = [
    (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0),
    (2.5, 2.5), (5.0, 0.0), (0.0, 5.0), (5.0, 5.0),
    (0.0, 0.0), (2.5, 2.5),
]


def parse_round_output(output: str) -> dict:
    """解析单轮仿真输出，返回指标字典和缺失字段。

    依据 P0-1: 不再用默认值 -1 掩盖解析失败，
    而是明确记录缺失字段并返回 status。
    """
    patterns = {
        "dyn_collisions": r"动态障碍碰撞:\s*(\d+)",
        "wall_hits": r"穿墙事件:\s*(\d+)",
        "arrived": r"到达率:\s*(\d+)/(\d+)",
        "frames": r"总帧数:\s*(\d+)",
        "near_miss": r"近距离规避:\s*(\d+)",
    }

    parsed = {}
    missing = []

    for key, pattern in patterns.items():
        match = re.search(pattern, output)
        if not match:
            missing.append(key)
            continue
        if key == "arrived":
            parsed["arrived"] = int(match.group(1))
            parsed["total_points"] = int(match.group(2))
        else:
            parsed[key] = int(match.group(1))

    # 评估解析完整性
    critical_fields = ["arrived", "total_points", "frames"]
    missing_critical = [f for f in critical_fields if f in missing]
    if missing_critical:
        status = "invalid"
        failure_reason = "metric_parser_failed"
    elif missing:
        status = "partial"
        failure_reason = "partial_parse"
    else:
        status = "valid"
        failure_reason = None

    return {
        "parsed": parsed,
        "missing": missing,
        "status": status,
        "failure_reason": failure_reason,
    }


def run_one_round(algo_name, env_vars, round_idx=0, seed=0, mode="official"):
    """运行一轮完整巡航，返回 TrialResult。

    依据 P0-3: 保存随机种子和完整配置。
    依据 P0-6: 返回标准化 TrialResult，包含 status/failure_reason。
    """
    full_env = os.environ.copy()
    # P0-7: 注入固定场景参数
    full_env["SIM_DT"] = str(SIM_DT)
    full_env["INITIAL_POSE_X"] = str(INITIAL_POSE["x"])
    full_env["INITIAL_POSE_Y"] = str(INITIAL_POSE["y"])
    full_env["INITIAL_POSE_YAW"] = str(INITIAL_POSE["yaw"])
    full_env["OBSTACLE_SEED"] = str(OBSTACLE_SEED_BASE + seed)
    full_env["PATROL_WAYPOINTS"] = json.dumps(PATROL_WAYPOINTS)
    full_env["PYTHONHASHSEED"] = str(seed)
    full_env["NPY_SEED"] = str(seed)
    full_env["RANDOM_SEED"] = str(seed)
    for k, v in env_vars.items():
        full_env[k] = v

    experiment_id = make_experiment_id(
        SCENARIO_NAME, algo_name, seed,
        date_str=datetime.datetime.now().strftime("%Y%m%d"))

    # 保存配置 (P0-3)
    results_dir = get_results_dir(mode)
    config = {
        "algorithm": algo_name,
        "scenario": SCENARIO_NAME,
        "sim_dt": SIM_DT,
        "initial_pose": INITIAL_POSE,
        "patrol_waypoints": PATROL_WAYPOINTS,
        "obstacle_seed": OBSTACLE_SEED_BASE + seed,
        "round_idx": round_idx,
    }
    save_trial_config(results_dir, experiment_id, config, env_vars, seed)

    try:
        result = subprocess.run(
            [sys.executable, "auto_patrol_simulation.py", "--no-coppelia"],
            env=full_env,
            capture_output=True,
            text=True,
            timeout=120,
            cwd=os.path.dirname(os.path.abspath(__file__)) or ".",
        )
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        print(f"  [轮{round_idx}] 超时!")
        return create_invalid_result(
            experiment_id=experiment_id,
            algorithm=algo_name,
            scenario=SCENARIO_NAME,
            seed=seed,
            failure_reason="subprocess_timeout",
            missing_fields=["dyn_collisions", "wall_hits",
                                          "arrived", "total_points", "frames"],
            sim_dt=SIM_DT,
            mode=mode,
        )
    except Exception as e:
        print(f"  [轮{round_idx}] 异常: {e}")
        return create_invalid_result(
            experiment_id=experiment_id,
            algorithm=algo_name,
            scenario=SCENARIO_NAME,
            seed=seed,
            failure_reason=f"subprocess_error:{type(e).__name__}",
            missing_fields=["dyn_collisions", "wall_hits",
                                          "arrived", "total_points", "frames"],
            sim_dt=SIM_DT,
            mode=mode,
        )

    # 解析输出 (P0-1)
    parse_result = parse_round_output(output)
    parsed = parse_result["parsed"]
    missing = parse_result["missing"]
    status = parse_result["status"]
    failure_reason = parse_result["failure_reason"]

    # 保存原始日志 (P0-3: 保证可复现)
    log_path = os.path.join(results_dir, f"{experiment_id}_run.log")
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(output)

    if status == "invalid":
        return create_invalid_result(
            experiment_id=experiment_id,
            algorithm=algo_name,
            scenario=SCENARIO_NAME,
            seed=seed,
            failure_reason=failure_reason,
            missing_fields=missing,
            frames=parsed.get("frames", 0),
            sim_dt=SIM_DT,
            mode=mode,
        )

    # 构建标准化指标 (P0-2: 禁止无效值)
    metrics = {
        "collision_events": float(parsed.get("dyn_collisions", 0)),
        "collision_frames": float(parsed.get("dyn_collisions", 0)),
        "near_miss_events": float(parsed.get("near_miss", 0)),
        "wall_hits": float(parsed.get("wall_hits", 0)),
        "goal_success_rate": (
            parsed["arrived"] / parsed["total_points"]
            if "arrived" in parsed and "total_points" in parsed
               and parsed["total_points"] > 0
            else 0.0
        ),
    }
    # 清洗无效值 (P0-2)
    clean_metrics, still_missing = [], []
    for k, v in metrics.items():
        if is_invalid_metric_value(v):
            still_missing.append(k)
        else:
            clean_metrics.append((k, v))
    if still_missing:
        missing.extend(still_missing)
        if status == "valid":
            status = "partial"
            failure_reason = "invalid_metric_values"

    return TrialResult(
        experiment_id=experiment_id,
        algorithm=algo_name,
        scenario=SCENARIO_NAME,
        seed=seed,
        sim_dt=SIM_DT,
        frames=parsed.get("frames", 0),
        success=(status == "valid"),
        status=status,
        failure_reason=failure_reason,
        missing_fields=missing,
        metrics=dict(clean_metrics),
        artifacts={
            "log": os.path.basename(log_path),
            "config": f"{experiment_id}_config.json",
        },
        config=config,
        mode=mode,
    )


def test_algorithm(algo_name, env_vars, num_rounds=10, mode="official"):
    """测试一个算法，跑 num_rounds 轮。

    依据 P0-2: 统计时自动过滤无效结果。
    """
    print(f"\n{'='*60}")
    print(f"  测试算法: {algo_name} ({num_rounds}轮, mode={mode})")
    print(f"  环境变量: {env_vars}")
    print(f"{'='*60}")

    trial_results = []
    start = time.time()
    for i in range(num_rounds):
        seed = (i + 1) * 31 + hash(algo_name) % (2**16)
        r = run_one_round(algo_name, env_vars, i + 1, seed, mode)
        trial_results.append(r)
        if r.status == "invalid":
            print(f"  轮 {i+1:>3}/{num_rounds}: [INVALID] {r.failure_reason}, "
                  f"missing={r.missing_fields}")
        else:
            coll = r.metrics.get("collision_events", 0)
            nm = r.metrics.get("near_miss_events", 0)
            rate = r.metrics.get("goal_success_rate", 0)
            print(f"  轮 {i+1:>3}/{num_rounds}: "
                  f"碰撞={coll:>2.0f} 近距={nm:>2.0f} "
                  f"到达率={rate*100:.1f}% "
                  f"[{r.status}]")

    elapsed = time.time() - start

    # P0-2: 聚合时过滤无效结果
    valid_collisions, skipped = aggregate_valid_results(
        trial_results, "collision_events")
    valid_near_miss, _ = aggregate_valid_results(
        trial_results, "near_miss_events")
    valid_frames, _ = aggregate_valid_results(
        trial_results, "frames")
    # frames 存在 metrics 里? 不在。从 TrialResult.frames 取
    valid_frames_vals = [r.frames for r in trial_results
                         if r.status != "invalid" and r.frames > 0]
    valid_arrival, _ = aggregate_valid_results(
        trial_results, "goal_success_rate")

    total_collisions = sum(valid_collisions)
    total_near_miss = sum(valid_near_miss)
    total_frames = sum(valid_frames_vals)
    avg_arrival = (sum(valid_arrival) / len(valid_arrival)
                   if valid_arrival else 0.0)
    collisions_per_1000 = (total_collisions / total_frames * 1000
                           if total_frames > 0 else 0.0)
    n_valid = len(valid_collisions)
    n_invalid = len([r for r in trial_results if r.status == "invalid"])

    print(f"\n  --- 汇总 ---")
    print(f"  有效轮数: {n_valid}/{num_rounds}"
          f"{' (跳过' + str(n_invalid) + '个无效)' if n_invalid else ''}")
    if skipped:
        print(f"  跳过项: {skipped}")
    print(f"  总帧数: {total_frames}")
    print(f"  总碰撞: {total_collisions}")
    print(f"  碰撞率: {collisions_per_1000:.2f} / 1000帧")
    print(f"  近距规避: {total_near_miss}")
    print(f"  平均到达率: {avg_arrival*100:.1f}%")
    print(f"  总耗时: {elapsed:.1f}s")

    return {
        "algo": algo_name,
        "rounds": num_rounds,
        "valid_rounds": n_valid,
        "invalid_rounds": n_invalid,
        "total_collisions": total_collisions,
        "total_near_miss": total_near_miss,
        "total_frames": total_frames,
        "avg_arrival_rate": avg_arrival,
        "collisions_per_1000_frames": collisions_per_1000,
        "elapsed": elapsed,
        "trials": [r.to_dict() for r in trial_results],
        "mode": mode,
    }


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="避障算法对比测试 v3 (P0 修复版)")
    parser.add_argument("--rounds", type=int, default=10,
                        help="每算法跑N轮 (默认10)")
    parser.add_argument("--mode", choices=["official", "demo"],
                        default="official",
                        help="结果模式: official(正式) / demo(演示)")
    parser.add_argument("--algorithms", nargs="*",
                        default=None,
                        help="指定算法子集，默认全部")
    args = parser.parse_args()

    num_rounds = args.rounds
    mode = args.mode

    all_algorithms = [
        {"name": "基线(反应式)", "env": {
            "USE_STVOC": "0", "USE_ORCA": "0", "USE_APF": "0",
            "USE_VO": "0", "USE_CBF": "0"}},
        {"name": "STVOC", "env": {
            "USE_STVOC": "1", "USE_ORCA": "0", "USE_APF": "0",
            "USE_VO": "0", "USE_CBF": "0"}},
        {"name": "ORCA", "env": {
            "USE_STVOC": "0", "USE_ORCA": "1", "USE_APF": "0",
            "USE_VO": "0", "USE_CBF": "0"}},
        {"name": "APF", "env": {
            "USE_STVOC": "0", "USE_ORCA": "0", "USE_APF": "1",
            "USE_VO": "0", "USE_CBF": "0"}},
        {"name": "VO", "env": {
            "USE_STVOC": "0", "USE_ORCA": "0", "USE_APF": "0",
            "USE_VO": "1", "USE_CBF": "0"}},
        {"name": "CBF(VO+CBF)", "env": {
            "USE_STVOC": "0", "USE_ORCA": "0", "USE_APF": "0",
            "USE_VO": "0", "USE_CBF": "1"}},
    ]

    if args.algorithms:
        all_algorithms = [a for a in all_algorithms
                          if a["name"] in args.algorithms]
        if not all_algorithms:
            print(f"未找到指定算法: {args.algorithms}")
            return 1

    all_results = []
    for algo in all_algorithms:
        r = test_algorithm(algo["name"], algo["env"], num_rounds, mode)
        all_results.append(r)

    # 打印对比表格
    print(f"\n\n{'='*100}")
    print(f"  避障算法对比结果 ({num_rounds}轮 / 每算法 / mode={mode})")
    print(f"{'='*100}")
    print(f"{'算法':<16} {'有效轮':>8} {'总帧数':>10} {'碰撞':>8} "
          f"{'碰撞率/1000帧':>14} {'近距':>8} {'到达率':>8} {'耗时':>8}")
    print(f"{'-'*100}")
    for r in all_results:
        print(f"{r['algo']:<16} {r['valid_rounds']:>3}/{r['rounds']:<4} "
              f"{r['total_frames']:>10} {r['total_collisions']:>8} "
              f"{r['collisions_per_1000_frames']:>14.2f} "
              f"{r['total_near_miss']:>8} "
              f"{r['avg_arrival_rate']*100:>7.1f}% "
              f"{r['elapsed']:>7.1f}s")

    # 排名 (P0-2: 只排有效结果)
    valid_for_ranking = [r for r in all_results if r["valid_rounds"] > 0]
    ranked = sorted(valid_for_ranking,
                    key=lambda x: x["collisions_per_1000_frames"])
    print(f"\n  碰撞率排名 (越低越好, 仅有效结果):")
    for i, r in enumerate(ranked):
        print(f"    {i+1}. {r['algo']:<16}: "
              f"{r['collisions_per_1000_frames']:.2f} / 1000帧 "
              f"(有效{r['valid_rounds']}/{r['rounds']}轮)")

    invalid_algos = [r["algo"] for r in all_results if r["valid_rounds"] == 0]
    if invalid_algos:
        print(f"\n  [警告] 以下算法全部轮次无效，未参与排名: {invalid_algos}")
        print(f"  请检查 auto_patrol_simulation.py 的输出格式是否匹配解析正则")

    # P0-4: 按 mode 分目录保存
    results_dir = get_results_dir(mode)
    os.makedirs(results_dir, exist_ok=True)
    output_path = os.path.join(results_dir, "algo_comparison.json")

    # 顶层摘要
    summary = {
        "timestamp": datetime.datetime.now().isoformat(),
        "mode": mode,
        "scenario": SCENARIO_NAME,
        "sim_dt": SIM_DT,
        "num_rounds_per_algo": num_rounds,
        "fixed_config": {
            "initial_pose": INITIAL_POSE,
            "patrol_waypoints": PATROL_WAYPOINTS,
            "obstacle_seed_base": OBSTACLE_SEED_BASE,
        },
        "algorithms": all_results,
    }
    if mode == "demo":
        summary = mark_demo_results(summary)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2, default=str)
    print(f"\n  详细结果已保存到 {output_path}")
    if mode == "demo":
        print(f"  [注意] demo 模式结果不可用于正式结论")

    # 同时清理根目录的旧版无效结果文件
    legacy_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "algo_comparison.json")
    if os.path.exists(legacy_path) and mode == "official":
        backup_path = legacy_path + ".legacy_bak"
        if not os.path.exists(backup_path):
            os.rename(legacy_path, backup_path)
            print(f"  旧版 algo_comparison.json 已备份为 {backup_path}")


if __name__ == "__main__":
    main()
