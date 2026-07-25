"""开阔场景避障算法对比测试

在一个无内墙的开阔场地中测试，避免路径规划干扰，
纯测试避障算法的性能。
机器人在4个角落之间循环移动，8个行人随机穿越。
"""
import os
import sys
import json
import time
import math
import random

os.environ['USE_STVOC'] = '1'
os.environ['USE_ORCA'] = '1'
os.environ['USE_APF'] = '1'
os.environ['USE_VO'] = '1'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from auto_patrol_simulation import (
    AutoPatrolSimulator, DynamicObstacle,
)


# 开阔场景：只有外墙，没有内部墙和家具
OPEN_SCENE_BBOX = [
    ("wall_left",   (-5.0, -4.0, -4.8, 4.0)),
    ("wall_right",  ( 4.8, -4.0,  5.0, 4.0)),
    ("wall_bottom", (-5.0, -4.0,  5.0, -3.8)),
    ("wall_top",    (-5.0,  3.8,  5.0,  4.0)),
]


def create_open_scene_obstacles(num_pedestrians=8, seed=42):
    """创建开阔场景中的行人（随机方向直线运动）"""
    random.seed(seed)
    obstacles = []
    for i in range(num_pedestrians):
        # 随机初始位置（避开边缘）
        x = random.uniform(-4.0, 4.0)
        y = random.uniform(-3.0, 3.0)
        # 随机速度方向
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(0.02, 0.05)  # m/step
        vx = speed * math.cos(angle)
        vy = speed * math.sin(angle)

        obs = DynamicObstacle(f"ped_{i+1}", x=x, y=y, vx=vx, vy=vy, pattern='linear')
        obstacles.append(obs)
    return obstacles


def is_safe_open(x, y, margin=0.3):
    """检查开阔场景中位置是否安全（不撞外墙）"""
    return (-4.8 + margin) <= x <= (4.8 - margin) and (-3.8 + margin) <= y <= (3.8 - margin)


def run_open_test(algo_name, sim, num_frames=8000):
    """在开阔场景中测试一个算法"""
    print(f"\n{'='*60}")
    print(f"  开阔场景测试: {algo_name} ({num_frames}帧, 8行人)")
    print(f"{'='*60}")

    # 重置状态
    sim.robot_x = -3.0
    sim.robot_y = -2.0
    sim.robot_yaw = 0.0
    sim._last_vx = 0.0
    sim.dyn_collision_count = 0
    sim.wall_hit_count = 0

    # 8个目标点（4个角落 + 4条边中点）循环
    targets = [
        ( 3.5, -2.5, 0.0),
        ( 3.5,  2.5, 0.0),
        (-3.5,  2.5, math.pi),
        (-3.5, -2.5, math.pi),
        ( 0.0, -3.0, 0.0),
        ( 3.5,  0.0, 0.0),
        ( 0.0,  3.0, math.pi),
        (-3.5,  0.0, math.pi),
    ]
    target_idx = 0
    rounds_completed = 0

    # 碰撞去重
    collision_hysteresis = {}
    total_collisions = 0
    near_miss = 0
    total_distance = 0.0
    prev_x, prev_y = sim.robot_x, sim.robot_y

    start = time.time()

    for frame in range(num_frames):
        # 更新行人
        for obs in sim.dynamic_obstacles:
            obs.update(frame)

        rx, ry = sim.robot_x, sim.robot_y
        ryaw = sim.robot_yaw

        # 选目标
        tx, ty, tyaw = targets[target_idx]
        dist_to_target = math.sqrt((rx - tx)**2 + (ry - ty)**2)
        if dist_to_target < 0.4:
            target_idx = (target_idx + 1) % len(targets)
            if target_idx == 0:
                rounds_completed += 1

        # 收集行人（m/step → m/s）
        obs_list = []
        min_dist = float('inf')
        nearest = None
        for obs in sim.dynamic_obstacles:
            d = obs.distance_to(rx, ry)
            if d < min_dist:
                min_dist = d
                nearest = obs.name
            obs_list.append((
                obs.name, obs.x, obs.y,
                obs.vx * 30, obs.vy * 30, 'linear'
            ))

        curr_vx = sim._last_vx if hasattr(sim, '_last_vx') else 0.0

        # 默认巡航
        dx = tx - rx
        dy = ty - ry
        dist_t = math.sqrt(dx * dx + dy * dy)
        if dist_t > 1e-6:
            default_dir_x = dx / dist_t
            default_dir_y = dy / dist_t
        else:
            default_dir_x, default_dir_y = 1.0, 0.0
        default_step = 0.20
        default_ang = math.atan2(default_dir_y, default_dir_x)

        step_x, step_y, ang = rx, ry, ryaw
        algo_action = 'cruise'

        if algo_name == '基线':
            # 反应式：近距时反向逃离
            if min_dist < 0.55:
                nearest_obs = min(sim.dynamic_obstacles,
                                  key=lambda o: o.distance_to(rx, ry))
                flee_dx = rx - nearest_obs.x
                flee_dy = ry - nearest_obs.y
                flee_d = math.sqrt(flee_dx**2 + flee_dy**2)
                if flee_d > 1e-6:
                    step_x = rx + 0.15 * (flee_dx / flee_d)
                    step_y = ry + 0.15 * (flee_dy / flee_d)
                    ang = math.atan2(flee_dy / flee_d, flee_dx / flee_d)
                algo_action = 'flee'
            else:
                step_x = rx + default_step * default_dir_x
                step_y = ry + default_step * default_dir_y
                ang = default_ang
        else:
            cmd = None
            if algo_name == 'STVOC':
                cmd = sim.stvoc.compute_avoidance(
                    rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'ORCA':
                cmd = sim.orca.compute_avoidance(
                    rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'APF':
                cmd = sim.apf.compute_avoidance(
                    rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'VO':
                cmd = sim.vo.compute_avoidance(
                    rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)

            algo_action = cmd.action if cmd else 'cruise'

            if cmd and (cmd.conflict_detected or cmd.action in ('avoid', 'flee', 'evade')):
                step_size = math.sqrt(cmd.vx**2 + cmd.vy**2) / 30.0
                step_size = max(0.02, min(0.30, step_size))
                if cmd.vx != 0 or cmd.vy != 0:
                    vel_ang = math.atan2(cmd.vy, cmd.vx)
                else:
                    vel_ang = default_ang
                step_x = rx + step_size * math.cos(vel_ang)
                step_y = ry + step_size * math.sin(vel_ang)
                ang = vel_ang
            else:
                step_x = rx + default_step * default_dir_x
                step_y = ry + default_step * default_dir_y
                ang = default_ang

        # 边界限制（不撞外墙）
        if not is_safe_open(step_x, step_y):
            # 投影回安全区域
            step_x = max(-4.5, min(4.5, step_x))
            step_y = max(-3.5, min(3.5, step_y))

        # 更新机器人
        sim.robot_x = step_x
        sim.robot_y = step_y
        sim.robot_yaw = ang
        sim._last_vx = (step_x - rx) / (1/30)

        total_distance += math.sqrt((step_x - prev_x)**2 + (step_y - prev_y)**2)
        prev_x, prev_y = step_x, step_y

        # 近距统计
        if min_dist < 0.8:
            near_miss += 1

        # 碰撞检测（迟滞去重，每个行人30帧内只记一次）
        for obs in sim.dynamic_obstacles:
            d = obs.distance_to(sim.robot_x, sim.robot_y)
            if d < sim.dyn_collision_radius:
                key = obs.name
                if key not in collision_hysteresis or collision_hysteresis[key] <= 0:
                    total_collisions += 1
                    collision_hysteresis[key] = 30
                else:
                    collision_hysteresis[key] = max(0, collision_hysteresis[key] - 1)
            else:
                if obs.name in collision_hysteresis and collision_hysteresis[obs.name] > 0:
                    if d > sim.dyn_collision_radius + 0.3:
                        collision_hysteresis[obs.name] = max(
                            0, collision_hysteresis[obs.name] - 1)

        if (frame + 1) % 2000 == 0:
            print(f"  帧 {frame+1:>5}/{num_frames}: "
                  f"碰撞={total_collisions:>3} "
                  f"轮次={rounds_completed} "
                  f"最近={min_dist:.2f}m({nearest}) "
                  f"动作={algo_action}")

    elapsed = time.time() - start
    collisions_per_1000 = total_collisions / num_frames * 1000
    avg_speed = total_distance / num_frames * 30  # m/s

    print(f"\n  --- 结果 ---")
    print(f"  总帧数: {num_frames}")
    print(f"  总碰撞: {total_collisions}")
    print(f"  碰撞率: {collisions_per_1000:.2f} / 1000帧")
    print(f"  完成轮次: {rounds_completed}")
    print(f"  总移动距离: {total_distance:.1f}m")
    print(f"  平均速度: {avg_speed:.2f} m/s")
    print(f"  近距事件(<0.8m): {near_miss}")
    print(f"  耗时: {elapsed:.1f}s")

    return {
        'algo': algo_name,
        'frames': num_frames,
        'collisions': total_collisions,
        'collisions_per_1000': collisions_per_1000,
        'rounds': rounds_completed,
        'distance': total_distance,
        'avg_speed_mps': avg_speed,
        'near_miss': near_miss,
        'elapsed': elapsed,
    }


def main():
    num_frames = 8000

    # 创建仿真器
    sim = AutoPatrolSimulator(use_coppelia=False)

    # 替换为开阔场景行人（8个）
    sim.dynamic_obstacles = create_open_scene_obstacles(8, seed=42)

    algorithms = ['基线', 'STVOC', 'ORCA', 'APF', 'VO']

    all_results = []
    for algo in algorithms:
        # 每次测试前重置行人位置和速度（同样的初始条件）
        sim.dynamic_obstacles = create_open_scene_obstacles(8, seed=42)
        # 重置算法状态
        if sim.stvoc_enabled and hasattr(sim.stvoc, 'reset'):
            sim.stvoc.reset()
        if sim.orca_enabled and hasattr(sim.orca, 'reset'):
            sim.orca.reset()
        if sim.apf_enabled and hasattr(sim.apf, 'reset'):
            sim.apf.reset()
        if sim.vo_enabled and hasattr(sim.vo, 'reset'):
            sim.vo.reset()

        r = run_open_test(algo, sim, num_frames)
        all_results.append(r)

    # 对比表格
    print(f"\n\n{'='*90}")
    print(f"  开阔场景避障算法对比 ({num_frames}帧 / 8行人)")
    print(f"{'='*90}")
    print(f"{'算法':<10} {'碰撞':>6} {'碰撞/1000帧':>12} {'轮次':>6} "
          f"{'距离m':>8} {'均速m/s':>10} {'近距<0.8m':>10} {'耗时':>7}")
    print(f"{'-'*90}")
    for r in all_results:
        print(f"{r['algo']:<10} {r['collisions']:>6} "
              f"{r['collisions_per_1000']:>12.2f} {r['rounds']:>6} "
              f"{r['distance']:>8.1f} {r['avg_speed_mps']:>10.2f} "
              f"{r['near_miss']:>10} {r['elapsed']:>6.1f}s")

    # 排名
    ranked = sorted(all_results, key=lambda x: x['collisions_per_1000'])
    print(f"\n  碰撞率排名 (越低越好):")
    baseline_r = next((x for x in all_results if x['algo'] == '基线'), None)
    for i, r in enumerate(ranked):
        reduction = ''
        if baseline_r and r['algo'] != '基线' and baseline_r['collisions'] > 0:
            red = (baseline_r['collisions'] - r['collisions']) / baseline_r['collisions'] * 100
            reduction = f"  (vs基线: {red:+.1f}%)"
        print(f"    {i+1}. {r['algo']:<10}: {r['collisions_per_1000']:.2f} / 1000帧{reduction}")

    # 效率分析：每碰撞能走多远
    print(f"\n  效率分析 (每移动100m的碰撞数):")
    for r in all_results:
        if r['distance'] > 0:
            per_100m = r['collisions'] / r['distance'] * 100
            print(f"    {r['algo']:<10}: {per_100m:.2f} 次/100m")

    with open('open_field_results.json', 'w', encoding='utf-8') as f:
        json.dump(all_results, f, ensure_ascii=False, indent=2)
    print(f"\n  结果已保存到 open_field_results.json")


if __name__ == '__main__':
    main()
