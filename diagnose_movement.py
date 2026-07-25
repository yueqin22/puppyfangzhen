"""诊断脚本：检查机器人是否真的在巡航，还是卡住不动

每5秒记录一次机器人位置和状态，分析运动模式。
"""
import os
import sys
import math
import time

os.environ['USE_STVOC'] = '1'
os.environ['USE_ORCA'] = '1'
os.environ['USE_APF'] = '1'
os.environ['USE_VO'] = '1'
os.environ['SDL_VIDEODRIVER'] = 'dummy'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import visual_sim

# 10分钟 = 18000帧，每150帧(5秒)采样一次
TOTAL_FRAMES = 18000
SAMPLE_INTERVAL = 150  # 5秒
REPORT_INTERVAL = 1800  # 60秒

sim = visual_sim.VisualSimulator()
sim.current_algo = 'A*+CBF'
sim.current_algo_idx = visual_sim.ALGORITHMS.index('A*+CBF')

# 位置采样
positions = []
stall_segments = []  # 记录卡住片段
moving_segments = []  # 记录移动片段

prev_x, prev_y = sim.sim.robot_x, sim.sim.robot_y
stall_start = -1
moving_start = 0

print("10分钟运动诊断 (每5秒采样一次)")
print("="*80)
print(f"{'时间':>6} {'X':>7} {'Y':>7} {'速度':>6} {'目标':>10} {'动作':>8} {'轮次':>4}")
print("-" * 80)

for frame in range(TOTAL_FRAMES):
    sim.step()

    if (frame + 1) % SAMPLE_INTERVAL == 0:
        rx, ry = sim.sim.robot_x, sim.sim.robot_y
        vx = rx - prev_x
        vy = ry - prev_y
        speed = math.sqrt(vx * vx + vy * vy) * 30  # m/s
        target = sim.patrol_targets[sim.target_idx][3]
        action = sim.current_action
        rounds = sim.rounds_completed
        t_sec = (frame + 1) / 30

        positions.append((t_sec, rx, ry, speed, target, action, rounds))

        # 检测移动/卡住状态
        if speed < 0.1:  # 速度<0.1m/s 认为卡住
            if stall_start < 0:
                stall_start = t_sec
                if moving_start >= 0:
                    moving_segments.append((moving_start, t_sec))
                    moving_start = -1
        else:
            if moving_start < 0:
                moving_start = t_sec
                if stall_start >= 0:
                    stall_segments.append((stall_start, t_sec))
                    stall_start = -1

        prev_x, prev_y = rx, ry
        print(f"{t_sec:>5.0f}s {rx:>7.2f} {ry:>7.2f} {speed:>5.2f}  {target:>10} {action:>8} {rounds:>4}")

    if (frame + 1) % REPORT_INTERVAL == 0:
        t_min = (frame + 1) / 30 / 60
        print(f"\n--- {t_min:.0f}分钟小结: 碰撞={sim.total_collisions} "
              f"近距={sim.near_miss} 跳点={sim.skip_count} "
              f"卡住={sim.stall_events} 轮次={sim.rounds_completed} ---\n")

# 收尾
if stall_start >= 0:
    stall_segments.append((stall_start, TOTAL_FRAMES / 30))
if moving_start >= 0:
    moving_segments.append((moving_start, TOTAL_FRAMES / 30))

# 分析
print("\n" + "=" * 80)
print("运动分析")
print("=" * 80)

# 1. 总移动时间 vs 卡住时间
total_stall = sum(b - a for a, b in stall_segments)
total_moving = sum(b - a for a, b in moving_segments)
total_time = TOTAL_FRAMES / 30

print(f"\n总时间: {total_time:.0f}s ({total_time/60:.1f}min)")
print(f"移动时间: {total_moving:.0f}s ({total_moving/60:.1f}min, {total_moving/total_time*100:.1f}%)")
print(f"卡住时间: {total_stall:.0f}s ({total_stall/60:.1f}min, {total_stall/total_time*100:.1f}%)")

# 2. 卡住片段详情
print(f"\n卡住片段 ({len(stall_segments)}次):")
for i, (start, end) in enumerate(stall_segments):
    duration = end - start
    if duration > 3:  # 只显示>3秒的卡住
        print(f"  #{i+1}: {start:.0f}s - {end:.0f}s ({duration:.0f}s)")

# 3. 速度统计
speeds = [p[3] for p in positions]
avg_speed = sum(speeds) / len(speeds)
max_speed = max(speeds)
moving_speeds = [s for s in speeds if s > 0.1]
avg_moving_speed = sum(moving_speeds) / len(moving_speeds) if moving_speeds else 0

print(f"\n速度统计:")
print(f"  平均速度: {avg_speed:.2f} m/s")
print(f"  最大速度: {max_speed:.2f} m/s")
print(f"  移动时均速: {avg_moving_speed:.2f} m/s")
print(f"  速度>0.5的帧比例: {len([s for s in speeds if s > 0.5])/len(speeds)*100:.1f}%")
print(f"  速度<0.1的帧比例: {len([s for s in speeds if s < 0.1])/len(speeds)*100:.1f}%")

# 4. 位置覆盖
xs = [p[1] for p in positions]
ys = [p[2] for p in positions]
print(f"\n位置覆盖:")
print(f"  X范围: [{min(xs):.2f}, {max(xs):.2f}]")
print(f"  Y范围: [{min(ys):.2f}, {max(ys):.2f}]")

# 5. 到达过的目标
targets_visited = set()
for p in positions:
    targets_visited.add(p[4])
print(f"  到达过的目标: {targets_visited}")
print(f"  目标数: {len(targets_visited)}/{len(sim.patrol_targets)}")
