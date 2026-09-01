"""自动运行visual_sim.py并定期截图 - 无界面模式(headless)"""
import os
import sys
import time
import math

# 设置SDL使用dummy视频驱动（无窗口）
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pygame
import numpy as np

# 先初始化pygame
pygame.init()

# 现在导入visual_sim的组件
from auto_patrol_simulation import (
    AutoPatrolSimulator, DynamicObstacle, PATROL_POINTS,
    OBSTACLES_BBOX, GRID_RESOLUTION,
    check_wall_penetration, is_position_safe,
)
from visual_sim import (
    SCREEN_W, SCREEN_H, FPS, VISUAL_MAX_SPEED, VISUAL_MAX_STEP,
    ARRIVAL_RADIUS, MARGIN,
    WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX,
    C_BG, C_FLOOR, C_WALL, C_FURNITURE, C_ROBOT, C_ROBOT_DIR,
    C_PED, C_PED_RADIUS, C_TARGET, C_TRAJ, C_TEXT, C_TEXT_HL,
    C_DANGER, C_SAFE, C_PANEL,
    ALGORITHMS, ALGO_COLORS, world_to_screen, screen_scale
)

def run_auto_simulation(screenshot_dir, duration_frames=900, save_interval=150):
    """自动运行仿真并定期截图"""
    os.makedirs(screenshot_dir, exist_ok=True)
    
    # 创建surface（虚拟屏幕）
    screen = pygame.Surface((SCREEN_W, SCREEN_H))
    
    # 初始化仿真
    sim = AutoPatrolSimulator(use_coppelia=False)
    
    # 从scene_home.json读取行人配置（如果有的话）
    import json
    scene_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'scene_home.json')
    peds = []
    if os.path.exists(scene_path):
        with open(scene_path, 'r', encoding='utf-8') as f:
            scene = json.load(f)
        ped_configs = scene.get('pedestrians', [])
        for i, p in enumerate(ped_configs):
            peds.append(DynamicObstacle(
                p.get('name', f'person_{i}'),
                x=p['x'], y=p['y'],
                vx=p.get('vx', 0.02), vy=p.get('vy', 0.01),
                radius=p.get('radius', 0.3),
                pattern=p.get('pattern', 'linear')
            ))
    if not peds:
        peds = [
            DynamicObstacle("person_1", x=-2.0, y=-1.5, vx=0.02, vy=0.01, pattern='linear'),
            DynamicObstacle("person_2", x=2.5, y=-2.5, vx=-0.01, vy=0.02, pattern='linear'),
            DynamicObstacle("person_3", x=5.0, y=0.0, vx=-0.02, vy=0.0, pattern='linear'),
            DynamicObstacle("person_4", x=-5.5, y=4.0, vx=0.01, vy=-0.015, pattern='linear'),
            DynamicObstacle("person_5", x=0.0, y=3.0, vx=0.015, vy=0.01, pattern='linear'),
            DynamicObstacle("person_6", x=-4.0, y=0.0, vx=0.01, vy=-0.01, pattern='linear'),
        ]
    sim.dynamic_obstacles = peds
    
    # 设置初始位置（从scene配置读取）
    init_x, init_y, init_yaw = -0.79, -4.36, 0.0
    if os.path.exists(scene_path):
        with open(scene_path, 'r', encoding='utf-8') as f:
            scene = json.load(f)
        init = scene.get('robot', {}).get('initial_pose', {})
        init_x = init.get('x', init_x)
        init_y = init.get('y', init_y)
        init_yaw = init.get('yaw', init_yaw)
    
    sim.robot_x = init_x
    sim.robot_y = init_y
    sim.robot_yaw = init_yaw
    sim._last_vx = 0.0
    sim._last_vy = 0.0
    
    # 使用VO算法（综合最优）
    current_algo = 'VO'
    current_algo_idx = ALGORITHMS.index(current_algo)
    
    # 巡航状态
    target_idx = 0
    robot_trail = []
    max_trail = 300
    total_collisions = 0
    near_miss = 0
    rounds_completed = 0
    
    # 字体
    try:
        font_small = pygame.font.SysFont("Microsoft YaHei", 14)
        font_med = pygame.font.SysFont("Microsoft YaHei", 18)
        font_large = pygame.font.SysFont("Microsoft YaHei", 24)
    except:
        font_small = pygame.font.Font(None, 16)
        font_med = pygame.font.Font(None, 20)
        font_large = pygame.font.Font(None, 28)
    
    print(f"[AUTO-SIM] 开始运行: {duration_frames}帧 ({duration_frames/FPS:.1f}秒)")
    print(f"[AUTO-SIM] 障碍物数: {len(OBSTACLES_BBOX)}, 巡逻点数: {len(PATROL_POINTS)}, 行人数: {len(peds)}")
    print(f"[AUTO-SIM] 初始位置: ({init_x:.2f}, {init_y:.2f})")
    
    screenshots = []
    
    for frame in range(duration_frames + 1):
        # === 仿真更新 ===
        # 获取当前目标
        if target_idx < len(PATROL_POINTS):
            target = PATROL_POINTS[target_idx]
            tx, ty = target['x'], target['y']
        else:
            target_idx = 0
            continue
        
        # 计算到目标的距离
        dx = tx - sim.robot_x
        dy = ty - sim.robot_y
        dist_to_target = math.sqrt(dx*dx + dy*dy)
        
        # 到达目标检查
        if dist_to_target < ARRIVAL_RADIUS:
            target_idx = (target_idx + 1) % len(PATROL_POINTS)
            if target_idx == 0:
                rounds_completed += 1
            continue
        
        # 计算期望速度（简单朝向目标）
        desired_speed = VISUAL_MAX_SPEED * 0.7
        if dist_to_target < 1.0:
            desired_speed *= (dist_to_target / 1.0)
        vx_desired = dx / dist_to_target * desired_speed if dist_to_target > 0.01 else 0
        vy_desired = dy / dist_to_target * desired_speed if dist_to_target > 0.01 else 0
        
        # 使用VO避障（默认算法）
        vx, vy = vx_desired, vy_desired
        if sim.vo is not None:
            try:
                robot_state = (sim.robot_x, sim.robot_y, sim._last_vx, sim._last_vy, 0.0)
                obstacles_list = [
                    (obs.x, obs.y, obs.vx, obs.vy, obs.radius)
                    for obs in sim.dynamic_obstacles
                ]
                new_vx, new_vy, _ = sim.vo.compute_velocity(
                    robot_state, obstacles_list,
                    (vx_desired, vy_desired),
                    preferred_speed=desired_speed
                )
                # 限速
                speed = math.sqrt(new_vx**2 + new_vy**2)
                if speed > VISUAL_MAX_SPEED:
                    scale = VISUAL_MAX_SPEED / speed
                    new_vx *= scale
                    new_vy *= scale
                vx, vy = new_vx, new_vy
            except Exception as e:
                pass
        
        # 静态障碍物检查（位置钳制）
        new_x = sim.robot_x + vx * (1.0/FPS)
        new_y = sim.robot_y + vy * (1.0/FPS)
        
        pen, _ = check_wall_penetration(sim.robot_x, sim.robot_y, new_x, new_y, OBSTACLES_BBOX)
        if pen or not is_position_safe(new_x, new_y, OBSTACLES_BBOX):
            # 尝试不同方向
            for angle_offset in [0.3, -0.3, 0.6, -0.6, 1.0, -1.0, math.pi]:
                angle = math.atan2(vy, vx) + angle_offset
                test_vx = math.cos(angle) * desired_speed * 0.5
                test_vy = math.sin(angle) * desired_speed * 0.5
                test_x = sim.robot_x + test_vx * (1.0/FPS)
                test_y = sim.robot_y + test_vy * (1.0/FPS)
                pen2, _ = check_wall_penetration(sim.robot_x, sim.robot_y, test_x, test_y, OBSTACLES_BBOX)
                if not pen2 and is_position_safe(test_x, test_y, OBSTACLES_BBOX):
                    new_x, new_y = test_x, test_y
                    vx, vy = test_vx, test_vy
                    break
            else:
                new_x, new_y = sim.robot_x, sim.robot_y  # 停下
        
        # 更新位置
        sim.robot_x = new_x
        sim.robot_y = new_y
        sim._last_vx = vx
        sim._last_vy = vy
        
        # 添加轨迹
        robot_trail.append((sim.robot_x, sim.robot_y))
        if len(robot_trail) > max_trail:
            robot_trail.pop(0)
        
        # 更新行人
        for obs in sim.dynamic_obstacles:
            obs.update(1.0/FPS)
            # 边界反弹
            if obs.x < WORLD_X_MIN + 1 or obs.x > WORLD_X_MAX - 1:
                obs.vx = -obs.vx
            if obs.y < WORLD_Y_MIN + 1 or obs.y > WORLD_Y_MAX - 1:
                obs.vy = -obs.vy
            # 与静态障碍物碰撞反弹（简化）
            if not is_position_safe(obs.x, obs.y, OBSTACLES_BBOX):
                obs.vx = -obs.vx + np.random.uniform(-0.01, 0.01)
                obs.vy = -obs.vy + np.random.uniform(-0.01, 0.01)
                obs.x += obs.vx * 2
                obs.y += obs.vy * 2
        
        # 碰撞检测
        for _, (xmin, ymin, xmax, ymax) in OBSTACLES_BBOX:
            if (xmin - 0.25 < sim.robot_x < xmax + 0.25 and
                ymin - 0.25 < sim.robot_y < ymax + 0.25):
                total_collisions += 1
                break
        
        # === 截图 ===
        if frame % save_interval == 0:
            # 清空屏幕
            screen.fill(C_BG)
            
            # 绘制地板区域
            floor_rect = pygame.Rect(
                world_to_screen(WORLD_X_MIN, WORLD_Y_MAX)[0],
                world_to_screen(WORLD_X_MIN, WORLD_Y_MAX)[1],
                world_to_screen(WORLD_X_MAX, WORLD_Y_MIN)[0] - world_to_screen(WORLD_X_MIN, WORLD_Y_MAX)[0],
                world_to_screen(WORLD_X_MAX, WORLD_Y_MIN)[1] - world_to_screen(WORLD_X_MIN, WORLD_Y_MAX)[1]
            )
            pygame.draw.rect(screen, C_FLOOR, floor_rect)
            
            # 绘制障碍物
            for name, (xmin, ymin, xmax, ymax) in OBSTACLES_BBOX:
                tl = world_to_screen(xmin, ymax)
                br = world_to_screen(xmax, ymin)
                rect = pygame.Rect(tl[0], tl[1], br[0]-tl[0], br[1]-tl[1])
                color = C_WALL if name.startswith('wall') else C_FURNITURE
                pygame.draw.rect(screen, color, rect)
                pygame.draw.rect(screen, (color[0]+30, color[1]+30, color[2]+30), rect, 1)
            
            # 绘制巡逻目标点
            for i, pt in enumerate(PATROL_POINTS):
                sx, sy = world_to_screen(pt['x'], pt['y'])
                is_current = (i == target_idx)
                radius = 8 if is_current else 5
                if pt.get('waypoint', False):
                    pygame.draw.circle(screen, (100, 100, 100), (sx, sy), radius, 1)
                else:
                    pygame.draw.circle(screen, C_TARGET if is_current else (60, 120, 60), (sx, sy), radius)
                    if is_current:
                        pygame.draw.circle(screen, C_TARGET, (sx, sy), radius + 4, 2)
            
            # 绘制轨迹
            if len(robot_trail) > 1:
                trail_points = [world_to_screen(x, y) for x, y in robot_trail]
                for i in range(len(trail_points)-1):
                    alpha = int(255 * (i / len(trail_points)))
                    trail_color = (C_TRAJ[0], C_TRAJ[1], C_TRAJ[2], alpha)
                    pygame.draw.line(screen, C_TRAJ, trail_points[i], trail_points[i+1], 2)
            
            # 绘制行人
            for obs in sim.dynamic_obstacles:
                sx, sy = world_to_screen(obs.x, obs.y)
                r_px = int(screen_scale(obs.radius))
                pygame.draw.circle(screen, C_PED_RADIUS, (sx, sy), r_px + 5)
                pygame.draw.circle(screen, C_PED, (sx, sy), r_px)
                # 速度方向
                spd = math.sqrt(obs.vx**2 + obs.vy**2)
                if spd > 0.001:
                    end_x = sx + int(obs.vx / spd * r_px * 1.5)
                    end_y = sy - int(obs.vy / spd * r_px * 1.5)
                    pygame.draw.line(screen, (255, 200, 200), (sx, sy), (end_x, end_y), 2)
            
            # 绘制机器人
            rx, ry = world_to_screen(sim.robot_x, sim.robot_y)
            robot_r_px = int(screen_scale(0.25))
            # 安全圈
            pygame.draw.circle(screen, (80, 80, 100), (rx, ry), robot_r_px + 8, 1)
            # 本体
            pygame.draw.circle(screen, C_ROBOT, (rx, ry), robot_r_px)
            # 方向指示
            yaw = sim.robot_yaw
            nose_x = rx + int(math.cos(yaw) * robot_r_px * 1.5)
            nose_y = ry - int(math.sin(yaw) * robot_r_px * 1.5)
            pygame.draw.line(screen, C_ROBOT_DIR, (rx, ry), (nose_x, nose_y), 3)
            pygame.draw.circle(screen, C_ROBOT_DIR, (nose_x, nose_y), 3)
            
            # 绘制信息面板
            panel_rect = pygame.Rect(5, 5, 280, 180)
            pygame.draw.rect(screen, C_PANEL, panel_rect)
            pygame.draw.rect(screen, (80, 80, 100), panel_rect, 1)
            
            info_lines = [
                f"布局版本: v5.1 (16m x 12m 开放式公寓)",
                f"算法: {current_algo}",
                f"帧: {frame} / {duration_frames} ({frame/FPS:.1f}s)",
                f"当前目标: {target.get('name', 'N/A')}",
                f"位置: ({sim.robot_x:.2f}, {sim.robot_y:.2f})",
                f"目标距离: {dist_to_target:.2f}m",
                f"完成轮数: {rounds_completed}",
                f"碰撞次数: {total_collisions}",
                f"行人数: {len(sim.dynamic_obstacles)}",
            ]
            
            for i, line in enumerate(info_lines):
                text_surf = font_small.render(line, True, C_TEXT)
                screen.blit(text_surf, (15, 15 + i * 18))
            
            # 标题
            title = font_large.render("PuppyPi 机器狗巡航仿真 - v5.1 家居布局", True, C_TEXT_HL)
            screen.blit(title, (SCREEN_W//2 - title.get_width()//2, SCREEN_H - 40))
            
            # 保存截图
            shot_num = frame // save_interval
            shot_path = os.path.join(screenshot_dir, f"v51_sim_shot_{shot_num:03d}_f{frame:04d}.png")
            pygame.image.save(screen, shot_path)
            screenshots.append(shot_path)
            print(f"[AUTO-SIM] 已保存截图: {os.path.basename(shot_path)} (目标: {target.get('name', 'N/A')}, 碰撞: {total_collisions})")
    
    pygame.quit()
    print(f"\n[AUTO-SIM] 完成！共保存 {len(screenshots)} 张截图")
    print(f"[AUTO-SIM] 最终统计 - 碰撞: {total_collisions}, 完成轮数: {rounds_completed}")
    return screenshots

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=900, help='运行帧数')
    parser.add_argument('--interval', type=int, default=150, help='截图间隔(帧)')
    args = parser.parse_args()
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    shot_dir = os.path.join(script_dir, 'screenshots', 'v51_auto_simulation')
    
    shots = run_auto_simulation(shot_dir, args.frames, args.interval)
    
    # 打印所有截图路径
    print("\n截图列表:")
    for s in shots:
        print(f"  {s}")
