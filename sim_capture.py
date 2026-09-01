"""
Improved auto-simulation renderer with robust navigation and anti-stuck logic (v5.5)
Uses:
- Multi-angle candidate direction selection with heuristic scoring
- Wall-following (Bug-like) recovery when stuck
- Oscillation detection and escape maneuvers
- Dynamic obstacle avoidance
Directly reads scene from scene_home.json
v5.5: Works with fixed entry-trap layout v5.5
"""
import os
import sys
import math
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Circle

# World bounds
WORLD_X_MIN = -8.5
WORLD_X_MAX = 8.5
WORLD_Y_MIN = -6.5
WORLD_Y_MAX = 6.5

ROBOT_RADIUS = 0.25
MAX_SPEED = 0.10  # Slightly faster for open layout
ARRIVAL_RADIUS = 0.45
WAYPOINT_ARRIVAL = 0.40
STUCK_RADIUS = 0.25
STUCK_FRAMES = 35
WALL_FOLLOW_STEPS = 60  # Steps to wall-follow before retrying direct


def load_scene():
    scene_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'scene_home.json')
    with open(scene_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def build_obstacles_bbox(scene):
    obstacles = []
    for obs in scene.get('obstacles', []):
        obstacles.append((obs['name'], (obs['xmin'], obs['ymin'], obs['xmax'], obs['ymax'])))
    return obstacles


def check_wall_penetration(x1, y1, x2, y2, obstacles_bbox, steps=8):
    """Check if a line segment penetrates any obstacle"""
    for i in range(steps + 1):
        t = i / steps
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
            if xmin < x < xmax and ymin < y < ymax:
                return True, name
    return False, None


def is_position_safe(x, y, obstacles_bbox, radius=0.12):
    """Check if position is safe (outside all obstacles by given radius)"""
    for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
        if x > xmin - radius and x < xmax + radius and y > ymin - radius and y < ymax + radius:
            cx = max(xmin, min(x, xmax))
            cy = max(ymin, min(y, ymax))
            dist = math.sqrt((x - cx)**2 + (y - cy)**2)
            if dist < radius:
                return False
    # World bounds
    if x < WORLD_X_MIN + 0.3 or x > WORLD_X_MAX - 0.3 or y < WORLD_Y_MIN + 0.3 or y > WORLD_Y_MAX - 0.3:
        return False
    return True


def nearest_obstacle_distance(x, y, obstacles_bbox):
    """Compute distance to nearest obstacle boundary"""
    min_dist = float('inf')
    for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
        cx = max(xmin, min(x, xmax))
        cy = max(ymin, min(y, ymax))
        dist = math.sqrt((x - cx)**2 + (y - cy)**2)
        if dist < min_dist:
            min_dist = dist
    return min_dist


def nearest_pedestrian_distance(x, y, peds):
    """Compute distance to nearest pedestrian"""
    min_d = float('inf')
    for p in peds:
        d = math.sqrt((x - p['x'])**2 + (y - p['y'])**2)
        min_d = min(min_d, d)
    return min_d


def compute_velocity_vo(robot_x, robot_y, target_x, target_y, peds, obstacles_bbox, preferred_speed=MAX_SPEED):
    """
    Velocity Obstacle-inspired direction selection:
    - Evaluate many candidate angles
    - Score based on: heading to target, obstacle clearance, pedestrian clearance
    - Return best (vx, vy)
    """
    desired_angle = math.atan2(target_y - robot_y, target_x - robot_x)
    dist_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
    
    # Many candidate angles for better coverage
    angle_candidates = []
    for delta_deg in [0, 5, -5, 12, -12, 22, -22, 35, -35, 50, -50, 70, -70, 95, -95, 125, -125, 155, -155, 180]:
        angle_candidates.append(desired_angle + math.radians(delta_deg))
    
    best_score = -float('inf')
    best_vx, best_vy = 0, 0
    
    for angle in angle_candidates:
        # Test with slightly reduced speed for safety
        for speed_mult in [1.0, 0.7, 0.4]:
            test_speed = preferred_speed * speed_mult
            test_vx = math.cos(angle) * test_speed
            test_vy = math.sin(angle) * test_speed
            test_x = robot_x + test_vx
            test_y = robot_y + test_vy
            
            # Check path doesn't go through walls
            pen, _ = check_wall_penetration(robot_x, robot_y, test_x, test_y, obstacles_bbox, steps=6)
            if pen:
                continue
            
            # Check final position safety
            if not is_position_safe(test_x, test_y, obstacles_bbox, radius=0.10):
                continue
            
            # Check pedestrian clearance
            min_ped_d = nearest_pedestrian_distance(test_x, test_y, peds)
            if min_ped_d < ROBOT_RADIUS + 0.28:
                continue
            
            # Compute nearest obstacle distance at test point
            obs_d = nearest_obstacle_distance(test_x, test_y, obstacles_bbox)
            
            # Scoring:
            # 1. Heading alignment (higher = facing target)
            heading_score = math.cos(angle - desired_angle)
            
            # 2. Speed incentive (prefer going faster)
            speed_score = speed_mult
            
            # 3. Obstacle clearance (prefer staying away from walls)
            obs_clearance = min(1.0, obs_d / 1.0)
            
            # 4. Pedestrian clearance
            ped_safety = min(1.0, min_ped_d / 1.2)
            
            # Weighted combination
            score = (heading_score * 2.5 + 
                     speed_score * 0.8 + 
                     obs_clearance * 0.6 + 
                     ped_safety * 0.7)
            
            if score > best_score:
                best_score = score
                best_vx, best_vy = test_vx, test_vy
    
    return best_vx, best_vy


def get_wall_follow_direction(robot_x, robot_y, target_x, target_y, obstacles_bbox, follow_side='left'):
    """
    Find a direction that follows the nearest wall contour.
    This is a simplified wall-following: find the direction perpendicular
    to the nearest obstacle surface that moves us along the wall.
    """
    # First, find nearest obstacle and which side we're on
    min_dist = float('inf')
    nearest_obs = None
    nearest_normal = None
    
    for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
        cx = max(xmin, min(robot_x, xmax))
        cy = max(ymin, min(robot_y, ymax))
        dist = math.sqrt((robot_x - cx)**2 + (robot_y - cy)**2)
        if dist < min_dist:
            min_dist = dist
            nearest_obs = (xmin, ymin, xmax, ymax)
            # Compute normal vector pointing away from obstacle
            if abs(robot_x - xmin) < 0.05 or abs(robot_x - xmax) < 0.05:
                # Near vertical wall - normal is horizontal
                nx = -1 if robot_x < (xmin + xmax) / 2 else 1
                ny = 0
            elif abs(robot_y - ymin) < 0.05 or abs(robot_y - ymax) < 0.05:
                # Near horizontal wall - normal is vertical
                nx = 0
                ny = -1 if robot_y < (ymin + ymax) / 2 else 1
            else:
                # Near corner - point away from corner
                nx = robot_x - cx
                ny = robot_y - cy
                mag = math.sqrt(nx*nx + ny*ny)
                if mag > 0.001:
                    nx /= mag
                    ny /= mag
            nearest_normal = (nx, ny)
    
    if nearest_normal is None:
        return 0, 0
    
    nx, ny = nearest_normal
    
    # Wall tangent direction (perpendicular to normal)
    # Left-follow: tangent = normal rotated +90 deg; Right-follow: -90 deg
    if follow_side == 'left':
        tx = -ny
        ty = nx
    else:
        tx = ny
        ty = -nx
    
    # Also have a component pointing slightly away from wall
    desired_angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
    
    # Try several angles around the wall-follow direction
    best_vx, best_vy = 0, 0
    best_score = -float('inf')
    
    for angle_offset in np.linspace(-math.pi/3, math.pi/3, 12):
        base_angle = math.atan2(ty, tx) + angle_offset
        for speed_mult in [0.8, 0.5, 0.3]:
            test_speed = MAX_SPEED * 0.7 * speed_mult
            test_vx = math.cos(base_angle) * test_speed
            test_vy = math.sin(base_angle) * test_speed
            test_x = robot_x + test_vx
            test_y = robot_y + test_vy
            
            if not is_position_safe(test_x, test_y, obstacles_bbox, radius=0.10):
                continue
            
            pen, _ = check_wall_penetration(robot_x, robot_y, test_x, test_y, obstacles_bbox)
            if pen:
                continue
            
            # Prefer directions that also have some component toward target
            heading_to_target = math.cos(base_angle - desired_angle_to_target)
            # Prefer keeping reasonable distance from walls
            obs_d = nearest_obstacle_distance(test_x, test_y, obstacles_bbox)
            clearance = min(1.0, obs_d / 0.6)
            
            score = heading_to_target * 0.5 + clearance * 1.5
            if score > best_score:
                best_score = score
                best_vx, best_vy = test_vx, test_vy
    
    return best_vx, best_vy


def create_pedestrians(scene):
    peds = []
    for p in scene.get('pedestrians', []):
        peds.append({
            'name': p.get('name', 'ped'),
            'x': p['x'], 'y': p['y'],
            'vx': p.get('vx', 0.02) * 0.4, 'vy': p.get('vy', 0.01) * 0.4,
            'radius': p.get('radius', 0.3)
        })
    if not peds:
        peds = [
            {'name': 'person_1', 'x': -2.0, 'y': -2.0, 'vx': 0.006, 'vy': 0.004, 'radius': 0.3},
            {'name': 'person_2', 'x': 2.0, 'y': -4.0, 'vx': 0.004, 'vy': 0.006, 'radius': 0.3},
            {'name': 'person_3', 'x': -6.0, 'y': -1.5, 'vx': 0.006, 'vy': -0.004, 'radius': 0.3},
            {'name': 'person_4', 'x': 5.5, 'y': -2.5, 'vx': -0.004, 'vy': 0.004, 'radius': 0.3},
            {'name': 'person_5', 'x': 5.5, 'y': 3.5, 'vx': 0.004, 'vy': -0.006, 'radius': 0.3},
            {'name': 'person_6', 'x': -2.0, 'y': 0.5, 'vx': 0.004, 'vy': -0.004, 'radius': 0.3},
        ]
    return peds


def update_pedestrians(peds, obstacles_bbox):
    """Update pedestrian positions with bouncing off walls and each other"""
    for ped in peds:
        ped['x'] += ped['vx']
        ped['y'] += ped['vy']
        
        # Bounce off boundaries
        if ped['x'] < -7.3 or ped['x'] > 7.3:
            ped['vx'] = -ped['vx']
            ped['x'] += ped['vx'] * 2
        if ped['y'] < -5.3 or ped['y'] > 5.3:
            ped['vy'] = -ped['vy']
            ped['y'] += ped['vy'] * 2
        
        # Bounce off obstacles
        if not is_position_safe(ped['x'], ped['y'], obstacles_bbox, radius=ped['radius'] + 0.08):
            ped['vx'] = -ped['vx'] + np.random.uniform(-0.003, 0.003)
            ped['vy'] = -ped['vy'] + np.random.uniform(-0.003, 0.003)
            ped['x'] += ped['vx'] * 3
            ped['y'] += ped['vy'] * 3
        
        # Pedestrian-pedestrian repulsion
        for other in peds:
            if other is ped:
                continue
            dx = other['x'] - ped['x']
            dy = other['y'] - ped['y']
            d = math.sqrt(dx*dx + dy*dy)
            min_d = ped['radius'] + other['radius'] + 0.20
            if d < min_d and d > 0.001:
                ped['vx'] -= dx/d * (min_d - d) * 0.025
                ped['vy'] -= dy/d * (min_d - d) * 0.025


def render_frame(ax, robot_x, robot_y, robot_yaw, peds, target_idx, trail,
                 frame_num, total_collisions, rounds, current_target_name, mode_text,
                 obstacles_bbox, patrol_points, visited_targets=None):
    ax.clear()
    ax.set_xlim(WORLD_X_MIN, WORLD_X_MAX)
    ax.set_ylim(WORLD_Y_MIN, WORLD_Y_MAX)
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1a23')
    
    # Floor
    ax.add_patch(patches.Rectangle((-8.0, -6.0), 16.0, 12.0,
                                   facecolor='#282837', edgecolor='none', zorder=1))
    
    # Grid
    ax.grid(True, alpha=0.06, color='#404050', linewidth=0.5)
    
    # Obstacles
    for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
        w = xmax - xmin
        h = ymax - ymin
        if name.startswith('wall'):
            color = '#78788c'
            ec = '#9898ac'
            lw = 1.0
        elif 'bed' in name:
            color = '#605048'
            ec = '#807060'
            lw = 0.8
        elif 'chair' in name:
            color = '#4a5a6a'
            ec = '#6a7a8a'
            lw = 0.6
        elif 'table' in name or 'island' in name or 'counter' in name:
            color = '#5a4a3a'
            ec = '#7a6a5a'
            lw = 0.8
        elif 'sofa' in name:
            color = '#4a5a4a'
            ec = '#6a7a6a'
            lw = 0.8
        else:
            color = '#50463c'
            ec = '#706050'
            lw = 0.7
        rect = FancyBboxPatch((xmin, ymin), w, h,
                              boxstyle="round,pad=0.02",
                              facecolor=color, edgecolor=ec, linewidth=lw, zorder=3)
        ax.add_patch(rect)
    
    # Patrol points
    for i, pt in enumerate(patrol_points):
        is_current = (i == target_idx)
        is_wp = pt.get('waypoint', False)
        is_visited = visited_targets is not None and i in visited_targets
        
        if is_current:
            color = '#64ff64'
            circle = Circle((pt['x'], pt['y']), 0.45, fill=False,
                           edgecolor=color, linewidth=2.5, linestyle='--', alpha=0.8, zorder=4)
            ax.add_patch(circle)
            ax.plot(pt['x'], pt['y'], marker='*', markersize=16, color=color, zorder=5)
        elif is_visited:
            color = '#4488ff'
            ax.plot(pt['x'], pt['y'], marker='o', markersize=5, color=color, zorder=4)
        else:
            color = '#3c783c' if not is_wp else '#aaaa44'
            ax.plot(pt['x'], pt['y'], marker='o', markersize=5, color=color, zorder=4)
        
        # Label for non-waypoint targets
        if not is_wp:
            ax.text(pt['x'], pt['y'] + 0.38, pt['name'], fontsize=6,
                   ha='center', color='#88cc88', alpha=0.85, fontfamily='Microsoft YaHei')
    
    # Trail (path history) - gradient from old (faint) to recent (bright)
    if len(trail) > 1:
        trail_x = [p[0] for p in trail]
        trail_y = [p[1] for p in trail]
        for i in range(len(trail) - 1):
            alpha = 0.1 + 0.6 * (i / max(1, len(trail) - 1))
            ax.plot(trail_x[i:i+2], trail_y[i:i+2],
                   color='#64b4ff', alpha=alpha, linewidth=1.3, zorder=5)
    
    # Pedestrians
    for ped in peds:
        safe_circle = Circle((ped['x'], ped['y']), ped['radius'] + 0.22,
                            fill=False, edgecolor='#ff9696', alpha=0.2, linewidth=1, zorder=5)
        ax.add_patch(safe_circle)
        ped_circle = Circle((ped['x'], ped['y']), ped['radius'],
                           facecolor='#ff6464', edgecolor='#ffc8c8', linewidth=1.5, zorder=6, alpha=0.85)
        ax.add_patch(ped_circle)
        spd = math.sqrt(ped['vx']**2 + ped['vy']**2)
        if spd > 0.0001:
            ax.arrow(ped['x'], ped['y'],
                    ped['vx']/spd * 0.3, ped['vy']/spd * 0.3,
                    head_width=0.07, head_length=0.10,
                    fc='#ffc8c8', ec='#ffc8c8', linewidth=1, zorder=6)
    
    # Robot
    robot_safe = Circle((robot_x, robot_y), ROBOT_RADIUS + 0.12,
                       fill=False, edgecolor='#50c8ff', alpha=0.25, linewidth=1, linestyle='--', zorder=7)
    ax.add_patch(robot_safe)
    robot_circle = Circle((robot_x, robot_y), ROBOT_RADIUS,
                         facecolor='#50c8ff', edgecolor='#ffffff', linewidth=2, zorder=8)
    ax.add_patch(robot_circle)
    nose_x = robot_x + math.cos(robot_yaw) * ROBOT_RADIUS * 1.5
    nose_y = robot_y + math.sin(robot_yaw) * ROBOT_RADIUS * 1.5
    ax.plot([robot_x, nose_x], [robot_y, nose_y],
           color='#ffff00', linewidth=3, zorder=9)
    ax.plot(nose_x, nose_y, marker='o', markersize=4, color='#ffff00', zorder=9)
    
    # Info panel
    version = "v5.5"
    info = (
        f"PuppyPi 巡航仿真 | {version} 开放式家居 (16m×12m)\n"
        f"算法: VO避障 + 沿墙脱困 | 帧: {frame_num} ({frame_num/30:.1f}s)\n"
        f"目标: {current_target_name} | 模式: {mode_text}\n"
        f"位置: ({robot_x:.2f}, {robot_y:.2f}) | 轮次: {rounds} | 碰撞: {total_collisions}"
    )
    ax.text(0.02, 0.98, info, transform=ax.transAxes,
           fontsize=9, verticalalignment='top', fontfamily='Microsoft YaHei',
           bbox=dict(boxstyle='round,pad=0.5', facecolor='#14141e', alpha=0.85, edgecolor='#505064'),
           color='#dcdcdc', zorder=20)
    
    ax.set_xlabel('X (m)', color='#808090', fontsize=8)
    ax.set_ylabel('Y (m)', color='#808090', fontsize=8)
    ax.tick_params(colors='#606070', labelsize=7)


def run_simulation(screenshot_dir, total_frames=2000, save_interval=120):
    os.makedirs(screenshot_dir, exist_ok=True)
    scene = load_scene()
    obstacles_bbox = build_obstacles_bbox(scene)
    patrol_points = scene.get('patrol_targets', [])
    
    init = scene.get('robot', {}).get('initial_pose', {})
    robot_x = init.get('x', -0.75)
    robot_y = init.get('y', -4.23)
    robot_yaw = init.get('yaw', 0.0)
    
    peds = create_pedestrians(scene)
    
    target_idx = 0
    trail = []
    total_collisions = 0
    rounds_completed = 0
    screenshots = []
    frames_on_target = 0
    stuck_timer = 0
    stuck_positions = []
    
    # Wall following state
    wall_follow_mode = False
    wall_follow_steps_left = 0
    wall_follow_side = 'left'  # Try left first
    wall_follow_start_dist = float('inf')
    
    visited_targets = set()
    mode_text = "巡航中"
    
    print("[INIT] 验证巡逻点...")
    for i, pt in enumerate(patrol_points):
        is_wp = pt.get('waypoint', False)
        safe = is_position_safe(pt['x'], pt['y'], obstacles_bbox, radius=0.20 if not is_wp else 0.15)
        wp_str = " [WAYPOINT]" if is_wp else ""
        print(f"  [{i}] {pt['name']} ({pt['x']:.2f}, {pt['y']:.2f}) - {'OK' if safe else 'UNSAFE!'}{wp_str}")
    
    print(f"\n[SIM] 开始: {total_frames}帧 ({total_frames/30:.0f}秒)")
    print(f"[SIM] 障碍物: {len(obstacles_bbox)}, 巡逻点: {len(patrol_points)}, 行人: {len(peds)}")
    print(f"[SIM] 版本: {scene.get('version', 'unknown')}")
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 10), dpi=100)
    fig.patch.set_facecolor('#14141e')
    
    # Initial frame
    current_target = patrol_points[target_idx]
    render_frame(ax, robot_x, robot_y, robot_yaw, peds, target_idx, trail,
                0, total_collisions, rounds_completed, current_target['name'], "起始位置",
                obstacles_bbox, patrol_points, visited_targets)
    plt.tight_layout()
    init_path = os.path.join(screenshot_dir, "v55_sim_000_initial.png")
    fig.savefig(init_path, dpi=100, facecolor=fig.get_facecolor(), bbox_inches='tight')
    screenshots.append(init_path)
    print(f"[SIM] 初始位置截图保存")
    
    for frame in range(1, total_frames + 1):
        if target_idx >= len(patrol_points):
            target_idx = 0
        
        target = patrol_points[target_idx]
        tx, ty = target['x'], target['y']
        is_wp = target.get('waypoint', False)
        arrival_r = WAYPOINT_ARRIVAL if is_wp else ARRIVAL_RADIUS
        dist = math.sqrt((tx - robot_x)**2 + (ty - robot_y)**2)
        
        # Arrival check
        if dist < arrival_r:
            print(f"[SIM] 帧{frame}: ✓ 到达 {target['name']} ({tx:.2f}, {ty:.2f})  [轮次{rounds_completed}]")
            visited_targets.add(target_idx)
            target_idx = (target_idx + 1) % len(patrol_points)
            frames_on_target = 0
            stuck_timer = 0
            stuck_positions = []
            wall_follow_mode = False
            wall_follow_steps_left = 0
            mode_text = "巡航中"
            if target_idx == 0:
                rounds_completed += 1
                visited_targets = set()
                print(f"[SIM] *** 完成第 {rounds_completed} 轮完整巡逻! ***")
            continue
        
        frames_on_target += 1
        
        # Timeout for current target - skip it
        if frames_on_target > 400:
            print(f"[SIM] 帧{frame}: ⚠ 超时，跳过 {target['name']}")
            target_idx = (target_idx + 1) % len(patrol_points)
            frames_on_target = 0
            stuck_timer = 0
            stuck_positions = []
            wall_follow_mode = False
            wall_follow_steps_left = 0
            mode_text = "巡航中"
            continue
        
        # Stuck detection
        stuck_positions.append((robot_x, robot_y))
        if len(stuck_positions) > STUCK_FRAMES:
            stuck_positions.pop(0)
        if len(stuck_positions) >= STUCK_FRAMES:
            xs = [p[0] for p in stuck_positions]
            ys = [p[1] for p in stuck_positions]
            displacement = math.sqrt((max(xs) - min(xs))**2 + (max(ys) - min(ys))**2)
            if displacement < STUCK_RADIUS:
                stuck_timer += 1
                if stuck_timer == 1 or (stuck_timer % 20 == 0):
                    print(f"[SIM] 帧{frame}: ⚠ 检测到卡住! 启动沿墙走脱困 (侧={wall_follow_side})")
                    wall_follow_mode = True
                    wall_follow_steps_left = WALL_FOLLOW_STEPS
                    wall_follow_start_dist = dist
                    mode_text = f"沿墙走({wall_follow_side})"
                    stuck_positions = []
            else:
                stuck_timer = 0
        else:
            stuck_timer = 0
        
        # Decide velocity
        vx, vy = 0, 0
        
        if wall_follow_mode and wall_follow_steps_left > 0:
            # Wall following mode
            vx, vy = get_wall_follow_direction(robot_x, robot_y, tx, ty, obstacles_bbox, wall_follow_side)
            wall_follow_steps_left -= 1
            
            # Check if we should exit wall-follow: if we're closer to target than when we started,
            # or if we've been wall-following long enough
            current_dist = math.sqrt((tx - robot_x)**2 + (ty - robot_y)**2)
            if wall_follow_steps_left <= 0 or current_dist < wall_follow_start_dist * 0.7:
                wall_follow_mode = False
                # Switch side for next time
                wall_follow_side = 'right' if wall_follow_side == 'left' else 'left'
                mode_text = "巡航中"
                stuck_positions = []
        else:
            # Normal navigation: VO-based
            vx, vy = compute_velocity_vo(robot_x, robot_y, tx, ty, peds, obstacles_bbox, MAX_SPEED)
            
            # If completely stuck (no valid direction), try wall-follow immediately
            if abs(vx) < 0.001 and abs(vy) < 0.001:
                vx, vy = get_wall_follow_direction(robot_x, robot_y, tx, ty, obstacles_bbox, wall_follow_side)
                wall_follow_mode = True
                wall_follow_steps_left = 30
                mode_text = f"紧急脱困({wall_follow_side})"
        
        # If still no movement, try random directions
        if abs(vx) < 0.001 and abs(vy) < 0.001:
            for ang in np.linspace(0, 2*math.pi, 32, endpoint=False):
                for spd in [MAX_SPEED * 0.6, MAX_SPEED * 0.3]:
                    tvx = math.cos(ang) * spd
                    tvy = math.sin(ang) * spd
                    tx_test = robot_x + tvx
                    ty_test = robot_y + tvy
                    if is_position_safe(tx_test, ty_test, obstacles_bbox, radius=0.08):
                        min_pd = nearest_pedestrian_distance(tx_test, ty_test, peds)
                        if min_pd > ROBOT_RADIUS + 0.25:
                            vx, vy = tvx, tvy
                            break
                if abs(vx) > 0.001 or abs(vy) > 0.001:
                    break
        
        # Update facing direction
        if abs(vx) > 0.001 or abs(vy) > 0.001:
            robot_yaw = math.atan2(vy, vx)
        
        # Execute movement with safety checking
        new_x = robot_x + vx
        new_y = robot_y + vy
        
        moved = False
        if is_position_safe(new_x, new_y, obstacles_bbox, radius=0.06):
            pen, _ = check_wall_penetration(robot_x, robot_y, new_x, new_y, obstacles_bbox, steps=4)
            if not pen:
                robot_x, robot_y = new_x, new_y
                moved = True
        
        if not moved:
            # Try partial movements at decreasing fractions
            for frac in [0.7, 0.5, 0.3, 0.15]:
                test_x = robot_x + vx * frac
                test_y = robot_y + vy * frac
                if is_position_safe(test_x, test_y, obstacles_bbox, radius=0.06):
                    pen, _ = check_wall_penetration(robot_x, robot_y, test_x, test_y, obstacles_bbox, steps=3)
                    if not pen:
                        robot_x, robot_y = test_x, test_y
                        moved = True
                        break
        
        # World bounds clamp (shouldn't be needed due to safety checks, but just in case)
        robot_x = max(WORLD_X_MIN + 0.35, min(WORLD_X_MAX - 0.35, robot_x))
        robot_y = max(WORLD_Y_MIN + 0.35, min(WORLD_Y_MAX - 0.35, robot_y))
        
        # Record trail
        trail.append((robot_x, robot_y))
        if len(trail) > 800:
            trail.pop(0)
        
        # Collision counting
        collision = False
        for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
            cx = max(xmin, min(robot_x, xmax))
            cy = max(ymin, min(robot_y, ymax))
            if math.sqrt((robot_x - cx)**2 + (robot_y - cy)**2) < ROBOT_RADIUS * 0.55:
                collision = True
                break
        if collision:
            total_collisions += 1
        
        # Update pedestrians every 2 frames
        if frame % 2 == 0:
            update_pedestrians(peds, obstacles_bbox)
        
        # Screenshot
        if frame % save_interval == 0:
            render_frame(ax, robot_x, robot_y, robot_yaw, peds, target_idx, trail,
                        frame, total_collisions, rounds_completed,
                        patrol_points[target_idx]['name'], mode_text,
                        obstacles_bbox, patrol_points, visited_targets)
            plt.tight_layout()
            shot_num = frame // save_interval
            shot_path = os.path.join(screenshot_dir, f"v55_sim_{shot_num:03d}_f{frame:04d}.png")
            fig.savefig(shot_path, dpi=100, facecolor=fig.get_facecolor(), bbox_inches='tight')
            screenshots.append(shot_path)
            t_name = patrol_points[target_idx]['name']
            wf = " [WF]" if wall_follow_mode else ""
            print(f"[SIM] 截图 {shot_num}: 帧={frame}, 目标={t_name}, 位置=({robot_x:.2f},{robot_y:.2f}), "
                  f"轮次={rounds_completed}, 碰撞={total_collisions}{wf}")
    
    # Final screenshot
    render_frame(ax, robot_x, robot_y, robot_yaw, peds, target_idx, trail,
                total_frames, total_collisions, rounds_completed,
                patrol_points[target_idx]['name'], "结束",
                obstacles_bbox, patrol_points, visited_targets)
    plt.tight_layout()
    final_path = os.path.join(screenshot_dir, "v55_sim_final.png")
    fig.savefig(final_path, dpi=100, facecolor=fig.get_facecolor(), bbox_inches='tight')
    screenshots.append(final_path)
    
    plt.close(fig)
    print(f"\n{'='*60}")
    print(f"[SIM] 完成! {len(screenshots)} 张截图已保存")
    print(f"[SIM] 最终: 完成轮次={rounds_completed}, 总碰撞次数={total_collisions}")
    print(f"[SIM] 已访问巡逻点: {len(visited_targets)}/{len(patrol_points)}")
    print(f"{'='*60}")
    
    return screenshots, rounds_completed, total_collisions


if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    shot_dir = os.path.join(script_dir, 'screenshots', 'v55_simulation')
    
    import shutil
    if os.path.exists(shot_dir):
        shutil.rmtree(shot_dir)
    
    shots, rounds, collisions = run_simulation(shot_dir, total_frames=3000, save_interval=150)
    print(f"\n截图 ({len(shots)} 张):")
    for s in shots:
        print(f"  {os.path.basename(s)}")
