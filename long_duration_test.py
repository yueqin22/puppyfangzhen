"""长时间开阔场景避障算法对比

8种算法 x 2小时模拟时间(216000帧) = 真正的统计对比

算法列表:
    1. 基线 (反应式急退)
    2. STVOC (时空速度障碍锥)
    3. ORCA (最优互惠避障)
    4. APF (人工势场法)
    5. VO (速度障碍法)
    6. RVO (互惠速度障碍法)
    7. SFM (社会力模型)
    8. DWA (动态窗口法)
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from auto_patrol_simulation import (
    AutoPatrolSimulator, DynamicObstacle,
)
from stvoc_avoidance import AvoidanceCommand
from rvo_avoidance import RVOController
from sfm_avoidance import SFMController
from dwa_avoidance import DWAController


def is_safe_open(x, y, margin=0.3):
    """检查开阔场景中位置是否安全"""
    return (-4.8 + margin) <= x <= (4.8 - margin) and (-3.8 + margin) <= y <= (3.8 - margin)


def create_pedestrians(num=8, seed=42):
    """创建行人（固定种子，保证各算法面对相同条件）"""
    random.seed(seed)
    obstacles = []
    for i in range(num):
        x = random.uniform(-4.0, 4.0)
        y = random.uniform(-3.0, 3.0)
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(0.02, 0.05)
        vx = speed * math.cos(angle)
        vy = speed * math.sin(angle)
        obs = DynamicObstacle(f"ped_{i+1}", x=x, y=y, vx=vx, vy=vy, pattern='linear')
        obstacles.append(obs)
    return obstacles


def run_long_test(algo_name, sim, num_frames=216000, report_interval=10000):
    """运行长时间测试

    Args:
        algo_name: 算法名称
        sim: 仿真器实例
        num_frames: 总帧数 (216000 = 2小时@30fps)
        report_interval: 报告间隔
    """
    print(f"\n{'='*60}")
    print(f"  长时间测试: {algo_name}")
    print(f"  帧数: {num_frames} ({num_frames/30/3600:.1f}小时模拟)")
    print(f"{'='*60}")

    # 重置机器人
    sim.robot_x = -3.0
    sim.robot_y = -2.0
    sim.robot_yaw = 0.0
    sim._last_vx = 0.0

    # 重置算法
    for algo_obj in [sim.stvoc, sim.orca, sim.apf, sim.vo]:
        if algo_obj and hasattr(algo_obj, 'reset'):
            algo_obj.reset()

    # 额外算法
    rvo = RVOController(dt=1/30, max_speed=6.0, robot_radius=0.25,
                        obstacle_radius=0.30, time_horizon=1.5,
                        safe_distance=sim.dyn_collision_radius)
    sfm = SFMController(dt=1/30, max_speed=6.0, desired_speed=6.0,
                        relax_time=0.5, soc_strength=2.0, soc_range=1.5,
                        robot_radius=0.25, obstacle_radius=0.30,
                        safe_distance=sim.dyn_collision_radius)
    dwa = DWAController(dt=1/30, max_speed=6.0, max_angular=2.0,
                        robot_radius=0.25, obstacle_radius=0.30,
                        safe_distance=sim.dyn_collision_radius)

    # 8个目标点循环
    targets = [
        ( 3.5, -2.5, 0.0), ( 3.5,  2.5, 0.0),
        (-3.5,  2.5, math.pi), (-3.5, -2.5, math.pi),
        ( 0.0, -3.0, 0.0), ( 3.5,  0.0, 0.0),
        ( 0.0,  3.0, math.pi), (-3.5,  0.0, math.pi),
    ]
    target_idx = 0
    rounds = 0

    # 碰撞去重
    collision_hysteresis = {}
    total_collisions = 0
    near_miss = 0
    total_distance = 0.0
    prev_x, prev_y = sim.robot_x, sim.robot_y

    # 分段统计（每36000帧=20分钟一段）
    segment_size = 36000
    segment_collisions = []

    start = time.time()

    for frame in range(num_frames):
        # 更新行人
        for obs in sim.dynamic_obstacles:
            obs.update(frame)

        rx, ry = sim.robot_x, sim.robot_y
        ryaw = sim.robot_yaw

        # 选目标
        tx, ty, tyaw = targets[target_idx]
        dist_t = math.sqrt((rx - tx)**2 + (ry - ty)**2)
        if dist_t < 0.4:
            target_idx = (target_idx + 1) % len(targets)
            if target_idx == 0:
                rounds += 1

        # 收集行人
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
                cmd = sim.stvoc.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'ORCA':
                cmd = sim.orca.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'APF':
                cmd = sim.apf.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'VO':
                cmd = sim.vo.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'RVO':
                cmd = rvo.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'SFM':
                cmd = sfm.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)
            elif algo_name == 'DWA':
                cmd = dwa.compute_avoidance(rx, ry, ryaw, curr_vx, 0.0, tx, ty, obs_list)

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

        # 边界限制
        if not is_safe_open(step_x, step_y):
            step_x = max(-4.5, min(4.5, step_x))
            step_y = max(-3.5, min(3.5, step_y))

        sim.robot_x = step_x
        sim.robot_y = step_y
        sim.robot_yaw = ang
        sim._last_vx = (step_x - rx) / (1/30)

        total_distance += math.sqrt((step_x - prev_x)**2 + (step_y - prev_y)**2)
        prev_x, prev_y = step_x, step_y

        if min_dist < 0.8:
            near_miss += 1

        # 碰撞检测（迟滞去重）
        for obs in sim.dynamic_obstacles:
            d = obs.distance_to(sim.robot_x, sim.robot_y)
            key = obs.name
            if d < sim.dyn_collision_radius:
                if key not in collision_hysteresis or collision_hysteresis[key] <= 0:
                    total_collisions += 1
                    collision_hysteresis[key] = 30
                else:
                    collision_hysteresis[key] = max(0, collision_hysteresis[key] - 1)
            else:
                if key in collision_hysteresis and collision_hysteresis[key] > 0:
                    if d > sim.dyn_collision_radius + 0.3:
                        collision_hysteresis[key] = max(0, collision_hysteresis[key] - 1)

        # 分段统计
        if (frame + 1) % segment_size == 0:
            seg = (frame + 1) // segment_size
            seg_col = total_collisions - sum(segment_collisions)
            segment_collisions.append(seg_col)
            elapsed = time.time() - start
            print(f"  [{seg}]{frame+1:>7}/{num_frames} ({(frame+1)/30/3600:.1f}h): "
                  f"碰撞={total_collisions:>5} "
                  f"轮次={rounds:>3} "
                  f"最近={min_dist:.2f}m "
                  f"距={total_distance:.0f}m "
                  f"耗时={elapsed:.0f}s")

    elapsed = time.time() - start
    collisions_per_1000 = total_collisions / num_frames * 1000
    collisions_per_100m = total_collisions / max(1, total_distance) * 100
    avg_speed = total_distance / num_frames * 30

    print(f"\n  === 最终结果 ===")
    print(f"  模拟时间: {num_frames/30/3600:.1f}小时 ({num_frames}帧)")
    print(f"  总碰撞: {total_collisions}")
    print(f"  碰撞率: {collisions_per_1000:.2f}/1000帧")
    print(f"  每100m碰撞: {collisions_per_100m:.2f}")
    print(f"  完成轮次: {rounds}")
    print(f"  总移动: {total_distance:.0f}m (均速{avg_speed:.2f}m/s)")
    print(f"  近距事件(<0.8m): {near_miss}")
    print(f"  分段碰撞: {segment_collisions}")
    print(f"  计算耗时: {elapsed:.0f}s ({elapsed/60:.1f}分钟)")

    return {
        'algo': algo_name,
        'frames': num_frames,
        'sim_hours': num_frames / 30 / 3600,
        'collisions': total_collisions,
        'collisions_per_1000': collisions_per_1000,
        'collisions_per_100m': collisions_per_100m,
        'rounds': rounds,
        'distance_m': total_distance,
        'avg_speed_mps': avg_speed,
        'near_miss': near_miss,
        'segment_collisions': segment_collisions,
        'elapsed_s': elapsed,
    }


def main():
    # 2小时模拟 = 216000帧
    num_frames = 108000  # 1小时模拟@30fps

    sim = AutoPatrolSimulator(use_coppelia=False)
    sim.dynamic_obstacles = create_pedestrians(8, seed=42)

    algorithms = ['基线', 'STVOC', 'ORCA', 'APF', 'VO', 'RVO', 'SFM', 'DWA']

    all_results = []
    for algo in algorithms:
        # 每次重置行人到相同初始状态
        sim.dynamic_obstacles = create_pedestrians(8, seed=42)
        r = run_long_test(algo, sim, num_frames)
        all_results.append(r)

        # 每完成一个算法就保存（防止中途崩溃丢失数据）
        with open('long_test_results.json', 'w', encoding='utf-8') as f:
            json.dump(all_results, f, ensure_ascii=False, indent=2)

    # 最终对比
    print(f"\n\n{'='*100}")
    print(f"  长时间避障算法对比 ({num_frames/30/3600:.0f}小时模拟 / 8行人)")
    print(f"{'='*100}")
    print(f"{'算法':<10} {'碰撞':>6} {'碰撞/1000帧':>12} {'碰撞/100m':>10} "
          f"{'轮次':>6} {'距离km':>8} {'均速m/s':>10} {'近距':>8} {'耗时min':>8}")
    print(f"{'-'*100}")
    for r in all_results:
        print(f"{r['algo']:<10} {r['collisions']:>6} "
              f"{r['collisions_per_1000']:>12.2f} {r['collisions_per_100m']:>10.2f} "
              f"{r['rounds']:>6} {r['distance_m']/1000:>8.2f} "
              f"{r['avg_speed_mps']:>10.2f} {r['near_miss']:>8} "
              f"{r['elapsed_s']/60:>7.1f}")

    # 排名
    ranked = sorted(all_results, key=lambda x: x['collisions_per_1000'])
    print(f"\n  碰撞率排名 (越低越好):")
    baseline = next((x for x in all_results if x['algo'] == '基线'), None)
    for i, r in enumerate(ranked):
        red = ''
        if baseline and r['algo'] != '基线' and baseline['collisions'] > 0:
            reduction = (baseline['collisions'] - r['collisions']) / baseline['collisions'] * 100
            red = f"  (vs基线: {reduction:+.1f}%)"
        print(f"    {i+1}. {r['algo']:<10}: {r['collisions_per_1000']:.2f}/1000帧{red}")

    # 效率排名 (碰撞/100m)
    ranked2 = sorted(all_results, key=lambda x: x['collisions_per_100m'])
    print(f"\n  效率排名 (碰撞/100m, 越低越好):")
    for i, r in enumerate(ranked2):
        print(f"    {i+1}. {r['algo']:<10}: {r['collisions_per_100m']:.2f}/100m")

    # 综合评分 (碰撞率 * 0.5 + 碰撞/100m * 0.5, 越低越好)
    print(f"\n  综合排名 (碰撞率40% + 效率40% + 速度20%):")
    max_col = max(r['collisions_per_1000'] for r in all_results) or 1
    max_eff = max(r['collisions_per_100m'] for r in all_results) or 1
    max_speed = max(r['avg_speed_mps'] for r in all_results) or 1
    for r in all_results:
        col_score = r['collisions_per_1000'] / max_col  # 越低越好
        eff_score = r['collisions_per_100m'] / max_eff  # 越低越好
        spd_score = 1 - r['avg_speed_mps'] / max_speed  # 越高速度越好，所以取反
        r['composite_score'] = col_score * 0.4 + eff_score * 0.4 + spd_score * 0.2

    ranked3 = sorted(all_results, key=lambda x: x['composite_score'])
    for i, r in enumerate(ranked3):
        print(f"    {i+1}. {r['algo']:<10}: {r['composite_score']:.3f}")

    with open('long_test_results.json', 'w', encoding='utf-8') as f:
        json.dump(all_results, f, ensure_ascii=False, indent=2)
    print(f"\n  详细结果已保存到 long_test_results.json")


if __name__ == '__main__':
    main()
