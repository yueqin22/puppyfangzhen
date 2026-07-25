"""4种避障算法对比测试脚本 v2 - 长时多轮对比

循环巡航 N 轮，累计碰撞数据，获得统计意义。
"""
import os
import sys
import json
import time
import subprocess
import re


def run_one_round(algo_name, env_vars, round_idx=0):
    """运行一轮完整巡航，返回结果"""
    full_env = os.environ.copy()
    for k, v in env_vars.items():
        full_env[k] = v

    try:
        result = subprocess.run(
            [sys.executable, 'auto_patrol_simulation.py', '--no-coppelia'],
            env=full_env,
            capture_output=True,
            text=True,
            timeout=120,
            cwd=os.path.dirname(os.path.abspath(__file__)) or '.',
        )
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        print(f"  [轮{round_idx}] 超时!")
        return None

    # 解析输出
    dyn_collision_match = re.search(r'动态障碍碰撞:\s*(\d+)', output)
    wall_hit_match = re.search(r'穿墙事件:\s*(\d+)', output)
    arrived_match = re.search(r'到达率:\s*(\d+)/(\d+)', output)
    frame_match = re.search(r'总帧数:\s*(\d+)', output)
    near_miss_match = re.search(r'近距离规避:\s*(\d+)', output)

    return {
        'dyn_collisions': int(dyn_collision_match.group(1)) if dyn_collision_match else 0,
        'wall_hits': int(wall_hit_match.group(1)) if wall_hit_match else 0,
        'arrived': int(arrived_match.group(1)) if arrived_match else 0,
        'total_points': int(arrived_match.group(2)) if arrived_match else 0,
        'frames': int(frame_match.group(1)) if frame_match else 0,
        'near_miss': int(near_miss_match.group(1)) if near_miss_match else 0,
    }


def test_algorithm(algo_name, env_vars, num_rounds=10):
    """测试一个算法，跑 num_rounds 轮"""
    print(f"\n{'='*60}")
    print(f"  测试算法: {algo_name} ({num_rounds}轮)")
    print(f"  环境变量: {env_vars}")
    print(f"{'='*60}")

    total = {
        'dyn_collisions': 0,
        'wall_hits': 0,
        'arrived': 0,
        'total_points': 0,
        'frames': 0,
        'near_miss': 0,
    }
    rounds_with_collision = 0

    start = time.time()
    for i in range(num_rounds):
        r = run_one_round(algo_name, env_vars, i + 1)
        if r is None:
            continue
        for k in total:
            total[k] += r[k]
        if r['dyn_collisions'] > 0:
            rounds_with_collision += 1
        if (i + 1) % 2 == 0 or i == num_rounds - 1:
            print(f"  轮 {i+1:>3}/{num_rounds}: "
                  f"碰撞={r['dyn_collisions']:>2} "
                  f"近距={r['near_miss']:>2} "
                  f"累计碰撞={total['dyn_collisions']:>3} "
                  f"到达={r['arrived']}/{r['total_points']}")

    elapsed = time.time() - start
    arrival_rate = total['arrived'] / total['total_points'] if total['total_points'] > 0 else 0
    collisions_per_1000_frames = total['dyn_collisions'] / total['frames'] * 1000 if total['frames'] > 0 else 0

    print(f"\n  --- 汇总 ---")
    print(f"  总帧数: {total['frames']}")
    print(f"  总碰撞: {total['dyn_collisions']}")
    print(f"  碰撞率: {collisions_per_1000_frames:.2f} / 1000帧")
    print(f"  近距规避: {total['near_miss']}")
    print(f"  穿墙: {total['wall_hits']}")
    print(f"  到达率: {arrival_rate*100:.1f}%")
    print(f"  有碰撞的轮数: {rounds_with_collision}/{num_rounds}")
    print(f"  总耗时: {elapsed:.1f}s")

    return {
        'algo': algo_name,
        'rounds': num_rounds,
        **total,
        'elapsed': elapsed,
        'arrival_rate': arrival_rate,
        'collisions_per_1000_frames': collisions_per_1000_frames,
        'rounds_with_collision': rounds_with_collision,
    }


def main():
    num_rounds = 10  # 每算法跑10轮（约100个巡航点，~4000帧）

    algorithms = [
        {
            'name': '基线(反应式)',
            'env': {
                'USE_STVOC': '0',
                'USE_ORCA': '0',
                'USE_APF': '0',
                'USE_VO': '0',
                'USE_CBF': '0',
            }
        },
        {
            'name': 'STVOC',
            'env': {
                'USE_STVOC': '1',
                'USE_ORCA': '0',
                'USE_APF': '0',
                'USE_VO': '0',
                'USE_CBF': '0',
            }
        },
        {
            'name': 'ORCA',
            'env': {
                'USE_STVOC': '0',
                'USE_ORCA': '1',
                'USE_APF': '0',
                'USE_VO': '0',
                'USE_CBF': '0',
            }
        },
        {
            'name': 'APF',
            'env': {
                'USE_STVOC': '0',
                'USE_ORCA': '0',
                'USE_APF': '1',
                'USE_VO': '0',
                'USE_CBF': '0',
            }
        },
        {
            'name': 'VO',
            'env': {
                'USE_STVOC': '0',
                'USE_ORCA': '0',
                'USE_APF': '0',
                'USE_VO': '1',
                'USE_CBF': '0',
            }
        },
        {
            'name': 'CBF(VO+CBF)',
            'env': {
                'USE_STVOC': '0',
                'USE_ORCA': '0',
                'USE_APF': '0',
                'USE_VO': '0',
                'USE_CBF': '1',
            }
        },
    ]

    all_results = []
    for algo in algorithms:
        r = test_algorithm(algo['name'], algo['env'], num_rounds)
        all_results.append(r)

    # 打印对比表格
    print(f"\n\n{'='*90}")
    print(f"  避障算法对比结果 ({num_rounds}轮 / 每算法)")
    print(f"{'='*90}")
    print(f"{'算法':<16} {'总帧数':>10} {'碰撞':>8} {'碰撞率/1000帧':>14} "
          f"{'近距':>8} {'穿墙':>6} {'到达率':>8} {'耗时':>8}")
    print(f"{'-'*90}")
    for r in all_results:
        print(f"{r['algo']:<16} {r['frames']:>10} {r['dyn_collisions']:>8} "
              f"{r['collisions_per_1000_frames']:>14.2f} {r['near_miss']:>8} "
              f"{r['wall_hits']:>6} {r['arrival_rate']*100:>7.1f}% "
              f"{r['elapsed']:>7.1f}s")

    # 排名（按碰撞率升序）
    ranked = sorted(all_results, key=lambda x: x['collisions_per_1000_frames'])
    print(f"\n  碰撞率排名 (越低越好):")
    for i, r in enumerate(ranked):
        print(f"    {i+1}. {r['algo']:<16}: {r['collisions_per_1000_frames']:.2f} / 1000帧")

    # 保存
    with open('algo_comparison.json', 'w', encoding='utf-8') as f:
        json.dump(all_results, f, ensure_ascii=False, indent=2)
    print(f"\n  详细结果已保存到 algo_comparison.json")


if __name__ == '__main__':
    main()
