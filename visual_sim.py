"""可视化巡航仿真 - pygame实时显示

机器狗在多房间场景中持续巡航，实时显示避障过程。
支持选择避障算法（默认VO，综合最优）。

操作:
    ESC / 关闭窗口: 退出
    SPACE: 暂停/继续
    1-9: 切换算法 (1基线 2STVOC 3ORCA 4APF 5VO 6RVO 7SFM 8DWA 9CBF)
    F: 加速 (x2/x4/x8)
    R: 重置
"""
import os
import sys
import math
import time
import random
import json

os.environ['USE_STVOC'] = '1'
os.environ['USE_ORCA'] = '1'
os.environ['USE_APF'] = '1'
os.environ['USE_VO'] = '1'

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pygame
import numpy as np
from auto_patrol_simulation import (
    AutoPatrolSimulator, DynamicObstacle, PATROL_POINTS,
    OBSTACLES_BBOX, GRID_RESOLUTION,
    check_wall_penetration, is_position_safe,
)
from rvo_avoidance import RVOController
from sfm_avoidance import SFMController
from dwa_avoidance import DWAController
from cbf_safety import CBFSafety
from visual_path_planner import plan_and_smooth, PathFollower

# ============================================================
# 显示参数
# ============================================================
SCREEN_W = 1200
SCREEN_H = 800
FPS = 30
VISUAL_MAX_SPEED = 2.0
VISUAL_MAX_STEP = VISUAL_MAX_SPEED / FPS
ARRIVAL_RADIUS = 0.35
SOFT_SKIP_RADIUS = 0.95
SOFT_SKIP_FRAMES = FPS * 20
HARD_SKIP_FRAMES = FPS * 40
DYNAMIC_CLEARANCE = 0.85  # 动态安全圈

# 世界坐标范围 → 屏幕坐标（适配v5.1 16m×12m布局）
WORLD_X_MIN = -8.5
WORLD_X_MAX = 8.5
WORLD_Y_MIN = -6.5
WORLD_Y_MAX = 6.5

MARGIN = 50  # 边距(像素)

def world_to_screen(x, y):
    """世界坐标 → 屏幕坐标"""
    sx = MARGIN + (x - WORLD_X_MIN) / (WORLD_X_MAX - WORLD_X_MIN) * (SCREEN_W - 2 * MARGIN)
    sy = MARGIN + (WORLD_Y_MAX - y) / (WORLD_Y_MAX - WORLD_Y_MIN) * (SCREEN_H - 2 * MARGIN)
    return int(sx), int(sy)

def screen_scale(meters):
    """世界距离 → 屏幕像素"""
    return meters / (WORLD_X_MAX - WORLD_X_MIN) * (SCREEN_W - 2 * MARGIN)

# 颜色
C_BG = (25, 25, 35)
C_FLOOR = (40, 40, 55)
C_WALL = (120, 120, 140)
C_FURNITURE = (80, 70, 60)
C_ROBOT = (80, 200, 255)
C_ROBOT_DIR = (255, 255, 0)
C_PED = (255, 100, 100)
C_PED_RADIUS = (255, 150, 150)
C_TARGET = (100, 255, 100)
C_TRAJ = (100, 180, 255)
C_TEXT = (220, 220, 220)
C_TEXT_HL = (255, 255, 100)
C_DANGER = (255, 60, 60)
C_SAFE = (60, 255, 100)
C_PANEL = (20, 20, 30)

# 算法列表
ALGORITHMS = ['基线', 'STVOC', 'ORCA', 'APF', 'VO', 'RVO', 'SFM', 'DWA', 'CBF', 'A*+CBF']
ALGO_COLORS = [
    (200, 200, 200),  # 基线
    (255, 150, 50),   # STVOC
    (150, 200, 255),  # ORCA
    (200, 255, 100),  # APF
    (100, 255, 200),  # VO
    (255, 200, 100),  # RVO
    (255, 100, 200),  # SFM
    (200, 100, 255),  # DWA
    (120, 255, 180),  # CBF
    (100, 255, 255),  # A*+CBF
]


class VisualSimulator:
    """可视化仿真器"""

    def __init__(self):
        # pygame初始化
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("机器狗巡航仿真 - 可视化")
        self.clock = pygame.time.Clock()

        # 字体
        try:
            self.font_small = pygame.font.SysFont("Microsoft YaHei", 14)
            self.font_med = pygame.font.SysFont("Microsoft YaHei", 18)
            self.font_large = pygame.font.SysFont("Microsoft YaHei", 24)
        except:
            self.font_small = pygame.font.Font(None, 16)
            self.font_med = pygame.font.Font(None, 20)
            self.font_large = pygame.font.Font(None, 28)

        # 仿真器
        self.sim = AutoPatrolSimulator(use_coppelia=False)
        self.sim.dynamic_obstacles = self._create_pedestrians(5, seed=42)

        # 额外算法
        self.rvo = RVOController(dt=1/30, max_speed=6.0, robot_radius=0.25,
                                 obstacle_radius=0.30, time_horizon=1.5,
                                 safe_distance=self.sim.dyn_collision_radius)
        self.sfm = SFMController(dt=1/30, max_speed=6.0, desired_speed=6.0,
                                 relax_time=0.5, soc_strength=2.0, soc_range=1.5,
                                 robot_radius=0.25, obstacle_radius=0.30,
                                 safe_distance=self.sim.dyn_collision_radius)
        self.dwa = DWAController(dt=1/30, max_speed=6.0, max_angular=2.0,
                                robot_radius=0.25, obstacle_radius=0.30,
                                safe_distance=self.sim.dyn_collision_radius)
        self.cbf = CBFSafety(robot_radius=0.25, d_safe=0.35, d_critical=0.08)
        self.cbf.max_speed = VISUAL_MAX_SPEED

        # A*路径规划
        self.path_follower = PathFollower(lookahead=0.6)
        self.current_path = []  # 当前A*路径（用于可视化）
        self.path_replan_interval = 30  # 每30帧重规划一次
        self.path_replan_counter = 0

        # 防卡机制
        self._stall_timer = 0  # 卡住计时器（帧）
        self._stall_prev_pos = (0.0, 0.0)  # 上一帧位置
        self._stall_history = []  # 位置历史（滑动窗口）
        self._recovery_mode = False  # 是否处于脱困模式
        self._recovery_dir = None  # 脱困方向
        self._recovery_timeout = 0  # 脱困模式剩余帧数

        # 巡航状态
        self.target_idx = 0
        self.patrol_targets = self._build_visual_patrol_targets()
        self.frames_on_target = 0
        self.robot_trail = []
        self.max_trail = 200

        # 统计
        self.frame = 0
        self.total_collisions = 0
        self.collision_hysteresis = {}
        self.near_miss = 0  # 近距事件数（进入<0.8m区域算一次事件）
        self._near_miss_active = False  # 当前是否在近距状态中
        self.near_miss_frames = 0  # 近距帧数（总共有多少帧在近距状态）
        self.skip_count = 0  # 超时跳点次数
        self.stall_events = 0  # 卡住事件次数
        self._stall_positions = []  # 用于检测卡住
        self._stall_count = 0
        self.rounds_completed = 0
        self.current_action = 'cruise'
        initial_algo = os.environ.get('VISUAL_ALGO', 'VO')
        if initial_algo not in ALGORITHMS:
            initial_algo = 'VO'
        self.current_algo = initial_algo
        self.current_algo_idx = ALGORITHMS.index(initial_algo)

        # 控制
        self.paused = False
        self.speed_mult = 1
        self.show_debug = True
        self.status_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'visual_status.json')

        # 初始位置
        first = self.patrol_targets[0]
        self.sim.robot_x = first[0]
        self.sim.robot_y = first[1]
        self.sim.robot_yaw = first[2]
        self.sim._last_vx = 0.0
        self.sim._last_vy = 0.0

        # 碰撞闪烁
        self.collision_flash = 0

    def _build_visual_patrol_targets(self):
        targets = [(pt['x'], pt['y'], pt['yaw'], pt['name']) for pt in PATROL_POINTS]
        visual_overrides = {
            '储物间': (2.45, 2.35),
            '厨房': (2.65, 0.45),
        }
        targets = [
            (visual_overrides[name][0], visual_overrides[name][1], yaw, name)
            if name in visual_overrides else (x, y, yaw, name)
            for x, y, yaw, name in targets
        ]
        dining_idx = next((i for i, pt in enumerate(targets) if pt[3] == '餐厅'), None)
        if dining_idx is not None:
            # The straight segment from storage to dining crosses the table.
            # These waypoints route the visual demo around the table's east side.
            targets[dining_idx:dining_idx] = [
                (2.35, 1.45, 0.0, '餐厅绕行'),
                (2.35, 0.25, 0.0, '餐厅入口'),
            ]
        return targets

    def _create_pedestrians(self, num=5, seed=42):
        random.seed(seed)
        obstacles = []
        configs = [
            ("person_1", 2.0, -2.5, -0.04, 0.0),
            ("person_2", -2.5, 2.8, 0.0, 0.04),
            ("person_3", 2.0, 0.5, -0.035, 0.0),
            ("person_4", -1.0, -1.0, 0.0, 0.03),
            ("person_5", 0.5, 2.0, 0.03, 0.0),
        ]
        for i, (name, x, y, vx, vy) in enumerate(configs[:num]):
            obs = DynamicObstacle(name, x=x, y=y, vx=vx, vy=vy, pattern='linear')
            obstacles.append(obs)
        return obstacles

    def reset(self):
        self.sim.dynamic_obstacles = self._create_pedestrians(5, seed=42)
        first = self.patrol_targets[0]
        self.sim.robot_x = first[0]
        self.sim.robot_y = first[1]
        self.sim.robot_yaw = first[2]
        self.sim._last_vx = 0.0
        self.sim._last_vy = 0.0
        self.target_idx = 0
        self.robot_trail = []
        self.frames_on_target = 0
        self.frame = 0
        self.total_collisions = 0
        self.collision_hysteresis = {}
        self.near_miss = 0
        self._near_miss_active = False
        self.near_miss_frames = 0
        self.skip_count = 0
        self.stall_events = 0
        self._stall_positions = []
        self._stall_count = 0
        self.rounds_completed = 0
        self.collision_flash = 0
        self.current_path = []
        self.path_replan_counter = 0
        self._stall_timer = 0
        self._stall_prev_pos = (0.0, 0.0)
        self._stall_history = []
        self._recovery_mode = False
        self._recovery_dir = None
        self._recovery_timeout = 0
        for algo in [self.sim.stvoc, self.sim.orca, self.sim.apf, self.sim.vo,
                     self.rvo, self.sfm, self.dwa]:
            if algo and hasattr(algo, 'reset'):
                algo.reset()

    def _cbf_obstacles(self):
        return [
            {'type': 'circle', 'center': (obs.x, obs.y), 'radius': obs.radius}
            for obs in self.sim.dynamic_obstacles
        ]

    def _limit_velocity(self, vx, vy, max_speed=VISUAL_MAX_SPEED):
        speed = math.sqrt(vx * vx + vy * vy)
        if speed > max_speed and speed > 1e-9:
            scale = max_speed / speed
            return vx * scale, vy * scale
        return vx, vy

    def _min_dynamic_distance(self, x, y):
        if not self.sim.dynamic_obstacles:
            return float('inf'), None
        nearest = min(self.sim.dynamic_obstacles, key=lambda o: o.distance_to(x, y))
        return nearest.distance_to(x, y), nearest

    def _candidate_is_static_safe(self, rx, ry, x, y):
        pen, _ = check_wall_penetration(rx, ry, x, y, OBSTACLES_BBOX)
        return (not pen) and is_position_safe(x, y, OBSTACLES_BBOX)

    def _is_near_doorway(self, x, y):
        """判断机器人是否在门道附近(1.0m以内)"""
        for pt in PATROL_POINTS:
            if pt.get('waypoint', False):
                dx = x - pt['x']
                dy = y - pt['y']
                if math.sqrt(dx * dx + dy * dy) < 1.0:
                    return True
        return False

    def _apply_dynamic_clearance(self, rx, ry, step_x, step_y, ang, tx, ty):
        min_next, nearest = self._min_dynamic_distance(step_x, step_y)
        if min_next >= DYNAMIC_CLEARANCE or nearest is None:
            return step_x, step_y, ang

        goal_ang = math.atan2(ty - ry, tx - rx)
        away_ang = math.atan2(ry - nearest.y, rx - nearest.x)
        attempted_step = math.sqrt((step_x - rx) ** 2 + (step_y - ry) ** 2)
        fallback_step = max(0.04, min(VISUAL_MAX_STEP, attempted_step))
        offsets = [
            0.0, math.pi/12, -math.pi/12, math.pi/6, -math.pi/6,
            math.pi/4, -math.pi/4, math.pi/3, -math.pi/3,
            math.pi/2, -math.pi/2, 2*math.pi/3, -2*math.pi/3,
            math.pi
        ]

        best = None
        best_score = -float('inf')
        for base_ang in (away_ang, goal_ang, ang):
            for offset in offsets:
                cand_ang = base_ang + offset
                cand_x = rx + fallback_step * math.cos(cand_ang)
                cand_y = ry + fallback_step * math.sin(cand_ang)
                if not self._candidate_is_static_safe(rx, ry, cand_x, cand_y):
                    continue
                dyn_dist, _ = self._min_dynamic_distance(cand_x, cand_y)
                target_dist = math.sqrt((cand_x - tx) ** 2 + (cand_y - ty) ** 2)
                score = dyn_dist * 2.0 - target_dist
                if dyn_dist >= DYNAMIC_CLEARANCE:
                    score += 5.0
                if score > best_score:
                    best_score = score
                    best = (cand_x, cand_y, cand_ang, dyn_dist)

        if best is not None:
            return best[0], best[1], best[2]

        current_dyn, current_nearest = self._min_dynamic_distance(rx, ry)
        if current_nearest is not None and current_dyn < DYNAMIC_CLEARANCE:
            retreat_ang = math.atan2(ry - current_nearest.y, rx - current_nearest.x)
            retreat_x = rx + 0.04 * math.cos(retreat_ang)
            retreat_y = ry + 0.04 * math.sin(retreat_ang)
            if self._candidate_is_static_safe(rx, ry, retreat_x, retreat_y):
                return retreat_x, retreat_y, retreat_ang

        return rx, ry, ang

    def step(self):
        """推进一帧仿真"""
        self.frame += 1

        # 更新行人
        for obs in self.sim.dynamic_obstacles:
            obs.update(self.frame)

        rx, ry = self.sim.robot_x, self.sim.robot_y
        ryaw = self.sim.robot_yaw

        # 选目标
        tx, ty, tyaw, tname = self.patrol_targets[self.target_idx]
        dist_t = math.sqrt((rx - tx)**2 + (ry - ty)**2)
        self.frames_on_target += 1
        if (dist_t < ARRIVAL_RADIUS or
                (self.frames_on_target > SOFT_SKIP_FRAMES and dist_t < SOFT_SKIP_RADIUS) or
                self.frames_on_target > HARD_SKIP_FRAMES):
            # 记录跳点
            if self.frames_on_target > SOFT_SKIP_FRAMES:
                self.skip_count += 1
            self.target_idx = (self.target_idx + 1) % len(self.patrol_targets)
            self.frames_on_target = 0
            if self.target_idx == 0:
                self.rounds_completed += 1
            tx, ty, tyaw, tname = self.patrol_targets[self.target_idx]
            # A*路径需要重新规划
            self.path_replan_counter = self.path_replan_interval

        # 收集行人
        obs_list = []
        min_dist = float('inf')
        nearest_name = ''
        for obs in self.sim.dynamic_obstacles:
            d = obs.distance_to(rx, ry)
            if d < min_dist:
                min_dist = d
                nearest_name = obs.name
            obs_list.append((
                obs.name, obs.x, obs.y,
                obs.vx * 30, obs.vy * 30, 'linear'
            ))

        curr_vx = self.sim._last_vx if hasattr(self.sim, '_last_vx') else 0.0
        curr_vy = self.sim._last_vy if hasattr(self.sim, '_last_vy') else 0.0

        # 默认巡航
        dx = tx - rx
        dy = ty - ry
        dist_t = math.sqrt(dx * dx + dy * dy)
        if dist_t > 1e-6:
            ddir_x = dx / dist_t
            ddir_y = dy / dist_t
        else:
            ddir_x, ddir_y = 1.0, 0.0
        default_step = VISUAL_MAX_STEP
        default_ang = math.atan2(ddir_y, ddir_x)

        step_x = rx + default_step * ddir_x
        step_y = ry + default_step * ddir_y
        ang = default_ang
        self.current_action = 'cruise'

        # 避障算法
        algo = self.current_algo
        cmd = None
        if algo == 'STVOC':
            cmd = self.sim.stvoc.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'ORCA':
            cmd = self.sim.orca.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'APF':
            cmd = self.sim.apf.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'VO':
            cmd = self.sim.vo.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'RVO':
            cmd = self.rvo.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'SFM':
            cmd = self.sfm.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'DWA':
            cmd = self.dwa.compute_avoidance(rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
        elif algo == 'CBF':
            base_cmd = self.sim.vo.compute_avoidance(
                rx, ry, ryaw, curr_vx, curr_vy, tx, ty, obs_list)
            desired_vx = base_cmd.vx
            desired_vy = base_cmd.vy
            if math.sqrt(desired_vx * desired_vx + desired_vy * desired_vy) < 1e-6:
                desired_vx = default_step * FPS * ddir_x
                desired_vy = default_step * FPS * ddir_y
            safe_v = self.cbf.safety_filter(
                (desired_vx, desired_vy), rx, ry, self._cbf_obstacles())
            safe_vx, safe_vy = self._limit_velocity(float(safe_v[0]), float(safe_v[1]))
            speed = math.sqrt(safe_vx * safe_vx + safe_vy * safe_vy)
            if speed > 1e-6:
                step_size = min(VISUAL_MAX_STEP, speed / FPS)
                vel_ang = math.atan2(safe_vy, safe_vx)
                step_x = rx + step_size * math.cos(vel_ang)
                step_y = ry + step_size * math.sin(vel_ang)
                ang = vel_ang
            risk = self.cbf.get_risk_level(rx, ry, self._cbf_obstacles())
            changed = math.sqrt((safe_vx - desired_vx) ** 2 + (safe_vy - desired_vy) ** 2)
            self.current_action = (
                'avoid' if risk > 0 or changed > 0.05 or base_cmd.conflict_detected
                else 'cruise')
        elif algo == 'A*+CBF':
            # A*全局路径规划 + CBF安全过滤 + 三层防卡
            # 1. 定期重规划A*路径
            self.path_replan_counter += 1
            if self.path_replan_counter >= self.path_replan_interval or not self.current_path:
                self.path_replan_counter = 0
                smoothed, raw = plan_and_smooth(rx, ry, tx, ty, OBSTACLES_BBOX)
                if smoothed and len(smoothed) >= 2:
                    self.path_follower.set_path(smoothed)
                    self.current_path = smoothed
                else:
                    self.current_path = [(rx, ry), (tx, ty)]
                    self.path_follower.set_path(self.current_path)

            # 2. 路径跟随：获取前瞻目标点
            follow_target = self.path_follower.get_target(rx, ry)
            if follow_target is None:
                follow_target = (tx, ty)

            # 3. 计算期望速度（朝向路径跟随目标）
            fdx = follow_target[0] - rx
            fdy = follow_target[1] - ry
            fdist = math.sqrt(fdx * fdx + fdy * fdy)
            if fdist > 1e-6:
                desired_vx = (fdx / fdist) * VISUAL_MAX_SPEED
                desired_vy = (fdy / fdist) * VISUAL_MAX_SPEED
            else:
                desired_vx = default_step * FPS * ddir_x
                desired_vy = default_step * FPS * ddir_y

            # === 三层防卡机制 ===
            # 用滑动窗口检测净位移（防止微小振荡绕过逐帧检测）
            self._stall_history.append((rx, ry))
            if len(self._stall_history) > 60:  # 2秒窗口
                self._stall_history.pop(0)
            if len(self._stall_history) >= 30:  # 至少1秒数据
                hxs = [p[0] for p in self._stall_history]
                hys = [p[1] for p in self._stall_history]
                net_spread = (max(hxs) - min(hxs)) + (max(hys) - min(hys))
                if net_spread < 0.15:  # 2秒内净移动<15cm = 卡住
                    if self.current_action != 'wait':
                        self._stall_timer += 1
                    else:
                        self._stall_timer = max(0, self._stall_timer - 1)
                else:
                    self._stall_timer = max(0, self._stall_timer - 1)  # 缓慢恢复
            else:
                self._stall_timer = max(0, self._stall_timer - 1)

            # 根据卡住程度及地理区域（门道 vs 阔域）设置 CBF 参数
            if self._recovery_mode:
                self.cbf.d_safe = 0.12
                self.cbf.alpha = 0.8
            elif self._stall_timer > 60:
                self.cbf.d_safe = 0.15  # 卡2秒，降低安全阈值
                self.cbf.alpha = 0.8
            elif self._is_near_doorway(rx, ry):
                self.cbf.d_safe = 0.20  # 门道区域自适应降低安全阈值
                self.cbf.alpha = 1.5
            else:
                self.cbf.d_safe = 0.35  # 正常
                self.cbf.alpha = 2.0

            # 层2: 卡住>3秒(90帧) → 强制横向绕行
            if self._stall_timer > 90 and not self._recovery_mode:
                self._recovery_mode = True
                self._recovery_timeout = 90  # 持续3秒
                # 找一个垂直于目标方向的通行方向
                goal_ang = math.atan2(ty - ry, tx - rx)
                best_dir = None
                best_score = -float('inf')
                for trial_ang in [goal_ang + math.pi/2, goal_ang - math.pi/2,
                                  goal_ang + math.pi*3/4, goal_ang - math.pi*3/4,
                                  goal_ang + math.pi/4, goal_ang - math.pi/4,
                                  goal_ang, goal_ang + math.pi]:
                    trial_x = rx + 0.5 * math.cos(trial_ang)
                    trial_y = ry + 0.5 * math.sin(trial_ang)
                    if self._candidate_is_static_safe(rx, ry, trial_x, trial_y):
                        dyn_d, _ = self._min_dynamic_distance(trial_x, trial_y)
                        score = dyn_d + (1.0 if trial_ang == goal_ang else 0.5)
                        if score > best_score:
                            best_score = score
                            best_dir = trial_ang
                if best_dir is not None:
                    self._recovery_dir = best_dir
                else:
                    self._recovery_dir = goal_ang  # 实在不行就硬冲目标

            # 层3: 脱困模式执行
            if self._recovery_mode and self._recovery_timeout > 0:
                self._recovery_timeout -= 1
                # 朝脱困方向走，CBF极宽松
                rec_vx = math.cos(self._recovery_dir) * VISUAL_MAX_SPEED
                rec_vy = math.sin(self._recovery_dir) * VISUAL_MAX_SPEED
                safe_v = self.cbf.safety_filter(
                    (rec_vx, rec_vy), rx, ry, self._cbf_obstacles())
                safe_vx, safe_vy = self._limit_velocity(float(safe_v[0]), float(safe_v[1]))
                speed = math.sqrt(safe_vx * safe_vx + safe_vy * safe_vy)
                if speed > 1e-6:
                    step_size = min(VISUAL_MAX_STEP, speed / FPS)
                    vel_ang = math.atan2(safe_vy, safe_vx)
                    step_x = rx + step_size * math.cos(vel_ang)
                    step_y = ry + step_size * math.sin(vel_ang)
                    ang = vel_ang
                self.current_action = 'recover'
                if self._recovery_timeout == 0 or speed > 0.1:
                    # 脱困成功或超时
                    self._recovery_mode = False
                    self._stall_timer = 0
                    self.path_replan_counter = self.path_replan_interval  # 强制重规划
            else:
                # 正常CBF安全过滤
                safe_v = self.cbf.safety_filter(
                    (desired_vx, desired_vy), rx, ry, self._cbf_obstacles())
                safe_vx, safe_vy = self._limit_velocity(float(safe_v[0]), float(safe_v[1]))

                # 动态安全采样（最终安全过滤）
                speed = math.sqrt(safe_vx * safe_vx + safe_vy * safe_vy)
                if speed > 1e-6:
                    step_size = min(VISUAL_MAX_STEP, speed / FPS)
                    vel_ang = math.atan2(safe_vy, safe_vx)
                    step_x = rx + step_size * math.cos(vel_ang)
                    step_y = ry + step_size * math.sin(vel_ang)
                    ang = vel_ang
                risk = self.cbf.get_risk_level(rx, ry, self._cbf_obstacles())
                changed = math.sqrt((safe_vx - desired_vx) ** 2 + (safe_vy - desired_vy) ** 2)
                self.current_action = (
                    'avoid' if risk > 0 or changed > 0.05
                    else 'cruise')
        elif algo == '基线':
            if min_dist < 0.55:
                nearest_obs = min(self.sim.dynamic_obstacles,
                                  key=lambda o: o.distance_to(rx, ry))
                flee_dx = rx - nearest_obs.x
                flee_dy = ry - nearest_obs.y
                flee_d = math.sqrt(flee_dx**2 + flee_dy**2)
                if flee_d > 1e-6:
                    step_x = rx + 0.15 * (flee_dx / flee_d)
                    step_y = ry + 0.15 * (flee_dy / flee_d)
                    ang = math.atan2(flee_dy / flee_d, flee_dx / flee_d)
                self.current_action = 'flee'

        if cmd:
            self.current_action = cmd.action
            if cmd.conflict_detected or cmd.action in ('avoid', 'flee', 'evade'):
                cmd_speed = math.sqrt(cmd.vx**2 + cmd.vy**2)
                if cmd.action == 'wait' or cmd_speed < 1e-6:
                    step_x = rx
                    step_y = ry
                    ang = ryaw
                else:
                    step_size = min(VISUAL_MAX_STEP, cmd_speed / FPS)
                    vel_ang = math.atan2(cmd.vy, cmd.vx)
                    step_x = rx + step_size * math.cos(vel_ang)
                    step_y = ry + step_size * math.sin(vel_ang)
                    ang = vel_ang

        # 穿墙检测
        pen, _ = check_wall_penetration(rx, ry, step_x, step_y, OBSTACLES_BBOX)
        if pen or not is_position_safe(step_x, step_y, OBSTACLES_BBOX):
            goal_ang = math.atan2(ty - ry, tx - rx)
            attempted_step = math.sqrt((step_x - rx) ** 2 + (step_y - ry) ** 2)
            fallback_step = max(0.04, min(VISUAL_MAX_STEP, attempted_step))
            offsets = [
                0.0, math.pi/12, -math.pi/12, math.pi/6, -math.pi/6,
                math.pi/4, -math.pi/4, math.pi/3, -math.pi/3,
                math.pi/2, -math.pi/2, 2*math.pi/3, -2*math.pi/3,
                math.pi
            ]
            best = None
            best_score = float('inf')
            for base_ang in (goal_ang, ang):
                for offset in offsets:
                    alt_ang = base_ang + offset
                    alt_x = rx + fallback_step * math.cos(alt_ang)
                    alt_y = ry + fallback_step * math.sin(alt_ang)
                    pen2, _ = check_wall_penetration(rx, ry, alt_x, alt_y, OBSTACLES_BBOX)
                    if pen2 or not is_position_safe(alt_x, alt_y, OBSTACLES_BBOX):
                        continue
                    target_dist = math.sqrt((alt_x - tx) ** 2 + (alt_y - ty) ** 2)
                    turn_penalty = abs(math.atan2(math.sin(alt_ang - goal_ang),
                                                  math.cos(alt_ang - goal_ang)))
                    score = target_dist + 0.15 * turn_penalty
                    if score < best_score:
                        best_score = score
                        best = (alt_x, alt_y, alt_ang)
            if best is not None:
                step_x, step_y, ang = best
            else:
                step_x = rx
                step_y = ry

        # 动态安全圈检查（A*+CBF脱困模式时跳过，避免覆盖脱困指令）
        if self.current_algo == 'A*+CBF' and self._recovery_mode:
            pass  # 脱困模式：信任CBF已做的安全过滤，不再二次限制
        elif self.current_algo == 'A*+CBF' and self._stall_timer > 60:
            # 卡住降阈值模式：放宽动态安全圈
            min_next, nearest = self._min_dynamic_distance(step_x, step_y)
            if min_next < 0.25 and nearest is not None:
                # 只在极近时才避让，否则放行
                step_x, step_y, ang = self._apply_dynamic_clearance(
                    rx, ry, step_x, step_y, ang, tx, ty)
        else:
            step_x, step_y, ang = self._apply_dynamic_clearance(
                rx, ry, step_x, step_y, ang, tx, ty)

        # 更新位置
        self.sim.robot_x = step_x
        self.sim.robot_y = step_y
        self.sim.robot_yaw = ang
        self.sim._last_vx = (step_x - rx) / (1/30)
        self.sim._last_vy = (step_y - ry) / (1/30)

        # 轨迹
        self.robot_trail.append((step_x, step_y))
        if len(self.robot_trail) > self.max_trail:
            self.robot_trail.pop(0)

        # 近距事件计数（进入<0.8m区域算一次事件，离开后重置）
        if min_dist < 0.8:
            self.near_miss_frames += 1
            if not self._near_miss_active:
                self.near_miss += 1
                self._near_miss_active = True
        else:
            self._near_miss_active = False

        # 卡住检测（60帧内位移<0.15m算一次卡住事件，需持续2秒）
        self._stall_positions.append((self.sim.robot_x, self.sim.robot_y))
        if len(self._stall_positions) > 60:
            self._stall_positions.pop(0)
            xs = [p[0] for p in self._stall_positions]
            ys = [p[1] for p in self._stall_positions]
            spread = (max(xs) - min(xs)) + (max(ys) - min(ys))
            if spread < 0.15:
                if self.current_action != 'wait':
                    self._stall_count += 1
                    # 持续卡住超过90帧(3秒)才算一次stall事件
                    if self._stall_count == 90:
                        self.stall_events += 1
                else:
                    self._stall_count = max(0, self._stall_count - 1)
            else:
                self._stall_count = 0

        # 碰撞检测
        for obs in self.sim.dynamic_obstacles:
            d = obs.distance_to(self.sim.robot_x, self.sim.robot_y)
            key = obs.name
            if d < self.sim.dyn_collision_radius:
                if key not in self.collision_hysteresis or self.collision_hysteresis[key] <= 0:
                    self.total_collisions += 1
                    self.collision_flash = 15
                    self.collision_hysteresis[key] = 30
                else:
                    self.collision_hysteresis[key] = max(0, self.collision_hysteresis[key] - 1)
            else:
                if key in self.collision_hysteresis and self.collision_hysteresis[key] > 0:
                    if d > self.sim.dyn_collision_radius + 0.3:
                        self.collision_hysteresis[key] = max(0, self.collision_hysteresis[key] - 1)

        if self.collision_flash > 0:
            self.collision_flash -= 1

        if self.frame % FPS == 0:
            self._write_status()

    def _write_status(self):
        target = self.patrol_targets[self.target_idx]
        status = {
            'frame': self.frame,
            'algorithm': self.current_algo,
            'target_idx': self.target_idx,
            'target': target[3],
            'rounds_completed': self.rounds_completed,
            'action': self.current_action,
            'robot_x': self.sim.robot_x,
            'robot_y': self.sim.robot_y,
            'collisions': self.total_collisions,
            'near_miss_events': self.near_miss,
            'near_miss_frames': self.near_miss_frames,
            'skip_count': self.skip_count,
            'stall_events': self.stall_events,
            'paused': self.paused,
            'timestamp': time.time(),
        }
        tmp_path = self.status_path + '.tmp'
        try:
            with open(tmp_path, 'w', encoding='utf-8') as f:
                json.dump(status, f, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self.status_path)
        except OSError:
            pass

    def draw(self):
        """绘制画面"""
        screen = self.screen
        screen.fill(C_BG)

        # 地板
        sx0, sy0 = world_to_screen(WORLD_X_MIN, WORLD_Y_MAX)
        sx1, sy1 = world_to_screen(WORLD_X_MAX, WORLD_Y_MIN)
        pygame.draw.rect(screen, C_FLOOR, (sx0, sy0, sx1-sx0, sy1-sy0))

        # 障碍物（家具 + 墙体）
        for name, (xmin, ymin, xmax, ymax) in OBSTACLES_BBOX:
            x1, y1 = world_to_screen(xmin, ymin)
            x2, y2 = world_to_screen(xmax, ymax)
            if name.startswith('wall_'):
                color = C_WALL
                pygame.draw.rect(screen, color, (min(x1,x2), min(y1,y2), abs(x2-x1), abs(y2-y1)))
                pygame.draw.rect(screen, (160,160,180), (min(x1,x2), min(y1,y2), abs(x2-x1), abs(y2-y1)), 1)
            else:
                color = C_FURNITURE
                pygame.draw.rect(screen, color, (min(x1,x2), min(y1,y2), abs(x2-x1), abs(y2-y1)))
                pygame.draw.rect(screen, (110,100,80), (min(x1,x2), min(y1,y2), abs(x2-x1), abs(y2-y1)), 1)
                # 家具标签
                label = self.font_small.render(name[:8], True, (140,130,110))
                screen.blit(label, (min(x1,x2)+2, min(y1,y2)+2))

        # 目标点
        tx, ty = self.sim.robot_x, self.sim.robot_y
        target = self.patrol_targets[self.target_idx]
        tsx, tsy = world_to_screen(target[0], target[1])
        pygame.draw.circle(screen, C_TARGET, (tsx, tsy), 10, 2)
        pygame.draw.circle(screen, C_TARGET, (tsx, tsy), 4)
        # 目标标签
        tlabel = self.font_small.render(target[3], True, C_TARGET)
        screen.blit(tlabel, (tsx + 12, tsy - 8))

        # 巡航路径线（简单连线）
        for i in range(len(self.patrol_targets)):
            p1 = self.patrol_targets[i]
            p2 = self.patrol_targets[(i+1) % len(self.patrol_targets)]
            x1, y1 = world_to_screen(p1[0], p1[1])
            x2, y2 = world_to_screen(p2[0], p2[1])
            pygame.draw.line(screen, (50, 60, 50), (x1, y1), (x2, y2), 1)

        # 机器人轨迹
        if len(self.robot_trail) > 1:
            pts = [world_to_screen(x, y) for x, y in self.robot_trail]
            if len(pts) > 1:
                pygame.draw.lines(screen, C_TRAJ, False, pts, 2)

        # A*规划路径（如果当前算法使用A*）
        if self.current_algo == 'A*+CBF' and len(self.current_path) > 1:
            path_pts = [world_to_screen(x, y) for x, y in self.current_path]
            if len(path_pts) > 1:
                pygame.draw.lines(screen, (80, 180, 180), False, path_pts, 1)
                for px, py in path_pts:
                    pygame.draw.circle(screen, (80, 180, 180), (px, py), 2)

        # 行人
        for obs in self.sim.dynamic_obstacles:
            px, py = world_to_screen(obs.x, obs.y)
            r = int(screen_scale(obs.radius))
            # 碰撞圈
            pygame.draw.circle(screen, C_PED_RADIUS, (px, py), r, 1)
            # 行人体
            pygame.draw.circle(screen, C_PED, (px, py), max(3, r-2))
            # 方向指示
            if abs(obs.vx) > 1e-6 or abs(obs.vy) > 1e-6:
                vang = math.atan2(obs.vy, obs.vx)
                ex = px + int(15 * math.cos(vang))
                ey = py + int(15 * math.sin(vang))
                pygame.draw.line(screen, C_PED, (px, py), (ex, ey), 2)
            # 名字
            nlabel = self.font_small.render(obs.name, True, (200, 150, 150))
            screen.blit(nlabel, (px - 15, py - 20))

        # 机器人（机器狗）
        rx, ry = world_to_screen(self.sim.robot_x, self.sim.robot_y)
        r_radius = int(screen_scale(0.25))

        # 碰撞闪烁
        if self.collision_flash > 0:
            flash_color = C_DANGER
            pygame.draw.circle(screen, flash_color, (rx, ry), r_radius + 10, 2)

        # 机器人本体
        algo_color = ALGO_COLORS[self.current_algo_idx]
        pygame.draw.circle(screen, algo_color, (rx, ry), r_radius)
        pygame.draw.circle(screen, C_ROBOT_DIR, (rx, ry), r_radius, 2)

        # 方向指示
        ryaw = self.sim.robot_yaw
        dir_len = r_radius + 12
        ex = rx + int(dir_len * math.cos(ryaw))
        ey = ry + int(dir_len * math.sin(ryaw))
        pygame.draw.line(screen, C_ROBOT_DIR, (rx, ry), (ex, ey), 3)
        # 前方小三角
        tip = (ex, ey)
        left = (rx + int((dir_len-6)*math.cos(ryaw-0.3)), ry + int((dir_len-6)*math.sin(ryaw-0.3)))
        right = (rx + int((dir_len-6)*math.cos(ryaw+0.3)), ry + int((dir_len-6)*math.sin(ryaw+0.3)))
        pygame.draw.polygon(screen, C_ROBOT_DIR, [tip, left, right])

        # === 信息面板 ===
        self._draw_panel()

        # === 算法选择栏 ===
        self._draw_algo_bar()

        pygame.display.flip()

    def _draw_panel(self):
        """绘制信息面板"""
        panel_x = SCREEN_W - 280
        panel_y = 10
        panel_w = 270
        panel_h = 200

        pygame.draw.rect(self.screen, C_PANEL, (panel_x, panel_y, panel_w, panel_h))
        pygame.draw.rect(self.screen, (60,60,80), (panel_x, panel_y, panel_w, panel_h), 1)

        y = panel_y + 8
        # 标题
        title = self.font_med.render("仿真状态", True, C_TEXT_HL)
        self.screen.blit(title, (panel_x + 10, y))
        y += 26

        sim_time = self.frame / 30.0
        sim_min = int(sim_time // 60)
        sim_sec = sim_time % 60

        info_lines = [
            f"算法: {self.current_algo}",
            f"模拟时间: {sim_min}m{sim_sec:.0f}s",
            f"帧数: {self.frame}",
            f"完成轮次: {self.rounds_completed}",
            f"碰撞次数: {self.total_collisions}",
            f"近距事件: {self.near_miss}",
            f"跳点次数: {self.skip_count}",
            f"卡住事件: {self.stall_events}",
            f"当前动作: {self.current_action}",
            f"目标: {self.patrol_targets[self.target_idx][3]}",
        ]
        for line in info_lines:
            color = C_TEXT
            if '碰撞' in line and self.total_collisions > 0:
                color = C_DANGER
            if '当前动作' in line:
                if self.current_action == 'flee':
                    color = C_DANGER
                elif self.current_action == 'avoid':
                    color = (255, 200, 50)
                elif self.current_action == 'cruise':
                    color = C_SAFE
            text = self.font_small.render(line, True, color)
            self.screen.blit(text, (panel_x + 10, y))
            y += 18

        # 控制说明
        y = panel_y + panel_h + 8
        pygame.draw.rect(self.screen, C_PANEL, (panel_x, y, panel_w, 135))
        pygame.draw.rect(self.screen, (60,60,80), (panel_x, y, panel_w, 135), 1)
        y += 8
        controls = [
            "操作说明:",
            "  SPACE: 暂停/继续",
            "  1-0: 切换避障算法",
            "  F: 加速 x2/x4/x8/正常",
            "  R: 重置仿真",
            "  ESC: 退出",
        ]
        for line in controls:
            text = self.font_small.render(line, True, C_TEXT)
            self.screen.blit(text, (panel_x + 10, y))
            y += 18

    def _draw_algo_bar(self):
        """绘制算法选择栏"""
        bar_y = SCREEN_H - 40
        bar_x = 10
        bar_w = SCREEN_W - 300

        pygame.draw.rect(self.screen, C_PANEL, (bar_x, bar_y, bar_w, 30))
        pygame.draw.rect(self.screen, (60,60,80), (bar_x, bar_y, bar_w, 30), 1)

        x = bar_x + 10
        for i, algo in enumerate(ALGORITHMS):
            color = ALGO_COLORS[i]
            if i == self.current_algo_idx:
                pygame.draw.rect(self.screen, color, (x, bar_y+3, 72, 24))
                text = self.font_small.render(f"{i+1 if i < 9 else 0}.{algo}", True, (0,0,0))
            else:
                pygame.draw.rect(self.screen, (40,40,50), (x, bar_y+3, 72, 24))
                pygame.draw.rect(self.screen, color, (x, bar_y+3, 72, 24), 1)
                text = self.font_small.render(f"{i+1 if i < 9 else 0}.{algo}", True, color)
            self.screen.blit(text, (x + 4, bar_y + 7))
            x += 82

    def run(self):
        """主循环"""
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key == pygame.K_f:
                        self.speed_mult = {1: 2, 2: 4, 4: 8, 8: 1}[self.speed_mult]
                    elif event.key == pygame.K_r:
                        self.reset()
                    elif pygame.K_1 <= event.key <= pygame.K_9:
                        idx = event.key - pygame.K_1
                        if idx < len(ALGORITHMS):
                            self.current_algo_idx = idx
                            self.current_algo = ALGORITHMS[idx]
                            # 重置统计但保持位置
                            self.robot_trail = []
                            self.current_path = []
                    elif event.key == pygame.K_0:
                        if 9 < len(ALGORITHMS):
                            self.current_algo_idx = 9
                            self.current_algo = ALGORITHMS[9]
                            self.robot_trail = []
                            self.current_path = []

            if not self.paused:
                for _ in range(self.speed_mult):
                    self.step()

            self.draw()
            self.clock.tick(FPS)

        pygame.quit()


if __name__ == '__main__':
    sim = VisualSimulator()
    sim.run()
