"""压力测试：高密度行人场景下的避障算法对比

创建一个有 8 个行人的密集场景，运行更长时间，
让各算法的碰撞差异充分体现。
"""
import os
import sys
import json
import time
import math
import random

# 在import仿真模块之前设置环境变量，启用所有算法
os.environ['USE_STVOC'] = '1'
os.environ['USE_ORCA'] = '1'
os.environ['USE_APF'] = '1'
os.environ['USE_VO'] = '1'

# 确保能 import 仿真模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from auto_patrol_simulation import (
    AutoPatrolSimulator, DynamicObstacle, PATROL_POINTS,
    OBSTACLES_BBOX, GRID_RESOLUTION,
    check_wall_penetration, is_position_safe,
)


def create_dense_obstacles():
    """创建高密度行人场景 (8个行人)"""
    configs = [
        ("person_1", 2.0, -2.5, -0.04, 0.0, 'linear'),
        ("person_2", -2.5, 2.8, 0.0, 0.04, 'linear'),
        ("person_3", 2.0, 0.5, -0.035, 0.0, 'linear'),
        ("person_4", -1.0, -1.0, 0.0, 0.03, 'linear'),
        ("person_5", 0.5, 2.0, 0.03, 0.0, 'linear'),
        ("person_6", -0.5, 0.0, 0.0, -0.035, 'linear'),
        ("person_7", 1.0, -1.5, -0.03, 0.0, 'linear'),
        ("person_8", -2.0, 1.0, 0.04, 0.0, 'linear'),
    ]

    obstacles = []
    for name, x, y, vx, vy, pattern in configs:
        obs = DynamicObstacle(name, x=x, y=y, vx=vx, vy=vy, pattern=pattern)
        obstacles.append(obs)

    return obstacles


def run_stress_test(algo_name, num_frames=6000):
    """运行压力测试

    Args:
        algo_name: 算法名称
        num_frames: 总帧数

    Returns:
        dict: 统计结果
    """
    print(f"\n{'='*60}")
    print(f"  压力测试: {algo_name} ({num_frames}帧)")
    print(f"{'='*60}")

    # 创建仿真器（纯模拟模式）
    sim = AutoPatrolSimulator(use_coppelia=False)

    # 替换为密集行人
    sim.dynamic_obstacles = create_dense_obstacles()
    sim.dyn_num = len(sim.dynamic_obstacles)

    # 重置统计
    sim.dyn_collision_count = 0
    sim.wall_hit_count = 0
    sim.frame = 0
    sim._dyn_wait_count = 0

    # 设置初始位置（从第一个目标点开始）
    first_pt = PATROL_POINTS[0]
    sim.robot_x = first_pt['x']
    sim.robot_y = first_pt['y']
    sim.robot_yaw = first_pt['yaw']

    # 目标列表：从PATROL_POINTS选房间中心点（非waypoint）循环访问
    target_candidates = [
        (pt['x'], pt['y'], pt['yaw'])
        for pt in PATROL_POINTS
        if not pt.get('waypoint', False)
    ]
    targets = target_candidates
    target_idx = 0
    rounds_completed = 0

    # 碰撞去重
    collision_set = set()
    collision_hysteresis = {}  # name -> remaining_frames

    start = time.time()
    near_miss_count = 0

    for frame in range(num_frames):
        sim.frame = frame

        # 更新行人
        for obs in sim.dynamic_obstacles:
            obs.update(frame)

        rx, ry = sim.robot_x, sim.robot_y
        ryaw = sim.robot_yaw

        # 选目标
        tx, ty, tyaw = targets[target_idx]
        dist_to_target = math.sqrt((rx - tx)**2 + (ry - ty)**2)
        if dist_to_target < 0.3:
            target_idx = (target_idx + 1) % len(targets)
            if target_idx == 0:
                rounds_completed += 1
            tx, ty, tyaw = targets[target_idx]

        # 计算避障
        step_x, step_y, ang = rx, ry, ryaw
        algo_action = 'cruise'

        # 收集行人（m/step → m/s）
        obs_list = []
        min_dist = float('inf')
        for obs in sim.dynamic_obstacles:
            d = obs.distance_to(rx, ry)
            if d < min_dist:
                min_dist = d
            obs_list.append((
                obs.name, obs.x, obs.y,
                obs.vx * 30, obs.vy * 30,
                getattr(obs, 'pattern', 'linear')
            ))

        # 当前速度
        if hasattr(sim, '_last_vx'):
            curr_vx = sim._last_vx
        else:
            curr_vx = 0.0

        # 默认巡航速度（向目标）
        dx = tx - rx
        dy = ty - ry
        dist_t = math.sqrt(dx * dx + dy * dy)
        if dist_t > 1e-6:
            default_dir_x = dx / dist_t
            default_dir_y = dy / dist_t
        else:
            default_dir_x, default_dir_y = 1.0, 0.0
        default_step = 0.20  # 默认步长（与基线一致）
        default_ang = math.atan2(default_dir_y, default_dir_x)

        # 根据算法选择
        use_avoidance = False

        if algo_name == '基线':
            # 纯反应式：直接向目标移动，近距时紧急避让
            if min_dist < sim.dyn_collision_radius * 1.5:
                # 找最近行人，反方向逃离
                nearest_obs = min(sim.dynamic_obstacles,
                                  key=lambda o: o.distance_to(rx, ry))
                flee_dx = rx - nearest_obs.x
                flee_dy = ry - nearest_obs.y
                flee_d = math.sqrt(flee_dx**2 + flee_dy**2)
                if flee_d > 1e-6:
                    step_x = rx + 0.15 * (flee_dx / flee_d)
                    step_y = ry + 0.15 * (flee_dy / flee_d)
                    ang = math.atan2(flee_dy / flee_d, flee_dx / flee_d)
                use_avoidance = True
                algo_action = 'flee'
            else:
                step_x = rx + default_step * default_dir_x
                step_y = ry + default_step * default_dir_y
                ang = default_ang
                algo_action = 'cruise'

        else:
            # 速度空间算法：计算命令
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

            if cmd and (cmd.conflict_detected or cmd.action != 'cruise'):
                # 有冲突时用算法的避障速度
                use_avoidance = True
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
                # 无冲突时正常巡航
                step_x = rx + default_step * default_dir_x
                step_y = ry + default_step * default_dir_y
                ang = default_ang

        # 确保不穿墙
        pen, _ = check_wall_penetration(rx, ry, step_x, step_y, OBSTACLES_BBOX)
        if pen or not is_position_safe(step_x, step_y, OBSTACLES_BBOX):
            step_x = rx
            step_y = ry

        # 更新机器人位置
        sim.robot_x = step_x
        sim.robot_y = step_y
        sim.robot_yaw = ang
        sim._last_vx = (step_x - rx) / (1/30)

        # 近距统计
        if min_dist < sim.dyn_collision_radius * 1.5:
            near_miss_count += 1

        # 碰撞检测（迟滞去重）
        new_collisions = []
        for obs in sim.dynamic_obstacles:
            d = obs.distance_to(sim.robot_x, sim.robot_y)
            if d < sim.dyn_collision_radius:
                key = obs.name
                if key not in collision_hysteresis or collision_hysteresis[key] <= 0:
                    new_collisions.append((obs.name, d))
                    collision_hysteresis[key] = 30  # 30帧去重窗口
                else:
                    collision_hysteresis[key] = max(0, collision_hysteresis[key] - 1)
            else:
                if obs.name in collision_hysteresis:
                    # 离开碰撞圈0.3m才重置
                    if d > sim.dyn_collision_radius + 0.3:
                        collision_hysteresis[obs.name] = max(
                            0, collision_hysteresis[obs.name] - 1)

        sim.dyn_collision_count += len(new_collisions)

        # 进度输出
        if (frame + 1) % 1000 == 0:
            print(f"  帧 {frame+1:>5}/{num_frames}: "
                  f"碰撞={sim.dyn_collision_count:>3} "
                  f"轮次={rounds_completed} "
                  f"最近={min_dist:.2f}m "
                  f"动作={algo_action}")

    elapsed = time.time() - start

    # 结果
    collisions_per_1000 = sim.dyn_collision_count / num_frames * 1000

    print(f"\n  --- 结果 ---")
    print(f"  总帧数: {num_frames}")
    print(f"  总碰撞: {sim.dyn_collision_count}")
    print(f"  碰撞率: {collisions_per_1000:.2f} / 1000帧")
    print(f"  完成轮次: {rounds_completed}")
    print(f"  近距事件: {near_miss_count}")
    print(f"  耗时: {elapsed:.1f}s")

    return {
        'algo': algo_name,
        'frames': num_frames,
        'collisions': sim.dyn_collision_count,
        'collisions_per_1000': collisions_per_1000,
        'rounds': rounds_completed,
        'near_miss': near_miss_count,
        'wall_hits': sim.wall_hit_count,
        'elapsed': elapsed,
    }


def main():
    num_frames = 6000  # 每算法6000帧（约3.3分钟实时，约20轮）

    algorithms = ['基线', 'STVOC', 'ORCA', 'APF', 'VO']

    all_results = []
    for algo in algorithms:
        r = run_stress_test(algo, num_frames)
        all_results.append(r)

    # 对比表格
    print(f"\n\n{'='*80}")
    print(f"  避障算法压力测试对比 ({num_frames}帧 / 8行人)")
    print(f"{'='*80}")
    print(f"{'算法':<12} {'碰撞':>8} {'碰撞率/1000帧':>14} {'轮次':>8} "
          f"{'近距事件':>10} {'耗时':>8}")
    print(f"{'-'*80}")
    for r in all_results:
        print(f"{r['algo']:<12} {r['collisions']:>8} "
              f"{r['collisions_per_1000']:>14.2f} {r['rounds']:>8} "
              f"{r['near_miss']:>10} {r['elapsed']:>7.1f}s")

    # 排名
    ranked = sorted(all_results, key=lambda x: x['collisions_per_1000'])
    print(f"\n  碰撞率排名 (越低越好):")
    for i, r in enumerate(ranked):
        reduction = ''
        baseline_r = next((x for x in all_results if x['algo'] == '基线'), None)
        if baseline_r and r['algo'] != '基线' and baseline_r['collisions'] > 0:
            red = (baseline_r['collisions'] - r['collisions']) / baseline_r['collisions'] * 100
            reduction = f"  (减少 {red:+.1f}%)"
        print(f"    {i+1}. {r['algo']:<12}: {r['collisions_per_1000']:.2f} / 1000帧{reduction}")

    # 保存
    with open('stress_test_results.json', 'w', encoding='utf-8') as f:
        json.dump(all_results, f, ensure_ascii=False, indent=2)
    print(f"\n  结果已保存到 stress_test_results.json")


if __name__ == '__main__':
    main()
