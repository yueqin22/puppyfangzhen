"""SFM: Social Force Model 社会力模型

Helbing等人的行人动力学模型 (1995, 2001)。

核心思想：
    行人运动由"社会力"驱动，包括：
    1. 自驱动力：朝目标方向加速到期望速度
    2. 社会排斥力：与其他行人的排斥（类似APF但有各向异性）
    3. 障碍排斥力：与障碍物的排斥
    4. 吸引力：对目标/同伴的吸引

数学表达：
    自驱动力:
        F_drive = (v0 * e_alpha - v_alpha) / tau
        e_alpha = (目标 - 当前位置) / |目标 - 当前位置|
        v0 = 期望速度, tau = 弛豫时间

    社会力 (各向异性！):
        F_ab = A * exp((r_ab - d_ab) / B) * n_ab
        其中：
        - A, B: 力强度和范围参数
        - r_ab = r_a + r_b (半径之和)
        - d_ab = |位置差| (实际距离)
        - n_ab = (位置A - 位置B) / d_ab (方向单位向量)

    各向异性改进：
        F_ab *= lambda + (1-lambda) * (1+cos(theta)) / 2
        theta = 行人b相对a的朝向角度
        lambda: 前方视野权重 (前方排斥力更大，后方更小)

    与APF的区别：
    1. 各向异性：前方排斥力大，后方小（更符合人类行为）
    2. 自驱动力有弛豫时间（不是瞬时达到目标速度）
    3. 可以模拟行人的排队、阻塞等群体现象
    4. 排斥力用指数函数（更平滑）

参考:
    Dirk Helbing, Peter Molnar. "Social force model for pedestrian
    dynamics." Physical Review E 51, 4282 (1995).
    Dirk Helbing et al. "Self-Organizing Pedestrian Movement."
    Environment and Planning B 2001.
"""
import math
from typing import List, Tuple

import numpy as np

from stvoc_avoidance import AvoidanceCommand


class SFMController:
    """SFM 社会力模型控制器

    用法:
        sfm = SFMController(dt=1/30, max_speed=6.0,
                            desired_speed=6.0, relax_time=0.5,
                            soc_strength=2.0, soc_range=1.5,
                            robot_radius=0.25, obstacle_radius=0.30)
        cmd = sfm.compute_avoidance(rx, ry, yaw, vx, vy, tx, ty, obstacles)
    """

    def __init__(self,
                 dt: float = 1/30,
                 max_speed: float = 6.0,
                 desired_speed: float = 6.0,
                 relax_time: float = 0.5,
                 soc_strength: float = 2.0,
                 soc_range: float = 1.5,
                 obs_strength: float = 3.0,
                 obs_range: float = 1.0,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30,
                 safe_distance: float = 0.55,
                 anisotropy: float = 0.5):
        """初始化SFM控制器

        Args:
            dt: 控制周期
            max_speed: 最大速度限幅
            desired_speed: 期望速度（自驱动力目标速度）
            relax_time: 弛豫时间（越小，加速越快）
            soc_strength: 社会力强度 A
            soc_range: 社会力作用范围 B
            obs_strength: 障碍物力强度
            obs_range: 障碍物力范围
            robot_radius: 机器人半径
            obstacle_radius: 行人半径
            safe_distance: 近距保护触发距离
            anisotropy: 各向异性参数 (0=各向同性, 1=完全各向异性)
                        前方排斥力 = 1.0, 后方 = (1-anisotropy)
        """
        self.dt = dt
        self.max_speed = max_speed
        self.desired_speed = desired_speed
        self.relax_time = relax_time
        self.soc_strength = soc_strength
        self.soc_range = soc_range
        self.obs_strength = obs_strength
        self.obs_range = obs_range
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.combined_radius = robot_radius + obstacle_radius
        self.safe_distance = safe_distance
        self.anisotropy = anisotropy

        self._last_vx = 0.0
        self._last_vy = 0.0

        self.stats = {
            'sfm_computes': 0,
            'near_miss': 0,
            'stagnation_escape': 0,
        }

        # 局部极小值逃逸
        self._stagnation_count = 0
        self._last_positions = []

    def compute_avoidance(self,
                          robot_x: float, robot_y: float, robot_yaw: float,
                          robot_vx: float, robot_vy: float,
                          target_x: float, target_y: float,
                          obstacles: List[Tuple]) -> AvoidanceCommand:
        """计算SFM避障指令"""
        self.stats['sfm_computes'] += 1

        robot_pos = np.array([robot_x, robot_y])
        robot_vel = np.array([robot_vx, robot_vy])

        # === 1. 自驱动力 ===
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist_to_target = math.sqrt(dx * dx + dy * dy)

        if dist_to_target > 1e-6:
            e_dir = np.array([dx / dist_to_target, dy / dist_to_target])
        else:
            e_dir = np.array([1.0, 0.0])

        # 期望速度方向
        v_desired = e_dir * self.desired_speed
        # 自驱动力 = (v_desired - v_current) / tau
        f_drive = (v_desired - robot_vel) / self.relax_time

        # === 2. 社会力（行人间排斥力）===
        f_social = np.array([0.0, 0.0])
        min_obs_dist = float('inf')
        nearest_obs = None
        ttc_min = float('inf')

        for name, ox, oy, ovx, ovy, _pattern in obstacles:
            obs_pos = np.array([ox, oy])
            rel_pos = robot_pos - obs_pos  # 从行人指向机器人
            dist = float(np.linalg.norm(rel_pos))

            if dist < min_obs_dist:
                min_obs_dist = dist
                nearest_obs = name

            if dist < 1e-6 or dist > self.soc_range * 3:
                continue

            # 方向单位向量
            if dist > 1e-6:
                n_dir = rel_pos / dist
            else:
                n_dir = np.array([1.0, 0.0])

            # 社会力大小: A * exp((r - d) / B)
            r_ab = self.combined_radius
            force_mag = self.soc_strength * math.exp((r_ab - dist) / self.soc_range)

            # 各向异性修正：行人在机器人前方时排斥力更大
            # theta = 行人相对机器人朝向的角度
            to_obs = -n_dir  # 从机器人指向行人
            cos_theta = np.dot(to_obs, e_dir)  # 与目标方向的夹角
            # 各向异性因子
            aniso_factor = self.anisotropy + (1 - self.anisotropy) * (1 + cos_theta) / 2
            force_mag *= aniso_factor

            f_social += n_dir * force_mag

            # TTC
            obs_vel = np.array([ovx, ovy])
            rel_vel = robot_vel - obs_vel
            rel_speed_along = float(np.dot(rel_vel, to_obs)) if dist > 1e-6 else 0
            if rel_speed_along > 0 and dist > self.combined_radius:
                t = (dist - self.combined_radius) / rel_speed_along
                if t < ttc_min and t < 3.0:
                    ttc_min = t

        # === 3. 速度感知修正 ===
        # 行人靠近时增加额外排斥（速度方向分量）
        for name, ox, oy, ovx, ovy, _pattern in obstacles:
            obs_pos = np.array([ox, oy])
            obs_vel = np.array([ovx, ovy])
            rel_pos = robot_pos - obs_pos
            dist = float(np.linalg.norm(rel_pos))

            if dist < 1e-6 or dist > self.soc_range:
                continue

            # 行人相对速度
            rel_v = obs_vel - robot_vel
            # 行人靠近速度
            if dist > 1e-6:
                approach_speed = float(np.dot(rel_v, rel_pos / dist))
            else:
                approach_speed = 0

            if approach_speed > 0:  # 行人正在靠近
                # 额外排斥力与接近速度成正比
                extra = approach_speed / self.max_speed * self.soc_strength * 0.5
                n_dir = rel_pos / dist
                f_social += n_dir * extra

        # === 4. 合力 ===
        f_total = f_drive + f_social

        # 加速度 = 力 / 质量（假设质量=1）
        accel = f_total

        # 速度更新: v_new = v_old + a * dt
        new_vx = robot_vx + accel[0] * self.dt
        new_vy = robot_vy + accel[1] * self.dt

        # 速度限幅
        speed = math.sqrt(new_vx ** 2 + new_vy ** 2)
        if speed > self.max_speed:
            new_vx = new_vx * (self.max_speed / speed)
            new_vy = new_vy * (self.max_speed / speed)

        # 速度平滑
        alpha = 0.5
        smooth_vx = alpha * new_vx + (1 - alpha) * self._last_vx
        smooth_vy = alpha * new_vy + (1 - alpha) * self._last_vy
        speed = math.sqrt(smooth_vx ** 2 + smooth_vy ** 2)
        if speed > self.max_speed:
            smooth_vx = smooth_vx * (self.max_speed / speed)
            smooth_vy = smooth_vy * (self.max_speed / speed)

        self._last_vx = float(smooth_vx)
        self._last_vy = float(smooth_vy)

        # === 局部极小值检测 ===
        self._last_positions.append((robot_x, robot_y))
        if len(self._last_positions) > 40:
            self._last_positions.pop(0)

        if len(self._last_positions) >= 40:
            xs = [p[0] for p in self._last_positions]
            ys = [p[1] for p in self._last_positions]
            spread = (max(xs) - min(xs)) + (max(ys) - min(ys))
            if spread < 0.2 and len(obstacles) > 0:
                self._stagnation_count += 1
                if self._stagnation_count > 30:
                    # 增加切向力逃逸
                    import random
                    angle = random.uniform(-math.pi, math.pi)
                    smooth_vx += math.cos(angle) * self.max_speed * 0.3
                    smooth_vy += math.sin(angle) * self.max_speed * 0.3
                    speed = math.sqrt(smooth_vx ** 2 + smooth_vy ** 2)
                    if speed > self.max_speed:
                        smooth_vx = smooth_vx * (self.max_speed / speed)
                        smooth_vy = smooth_vy * (self.max_speed / speed)
                    self.stats['stagnation_escape'] += 1
                    self._stagnation_count = 0
            else:
                self._stagnation_count = max(0, self._stagnation_count - 1)

        # === 决策分类 ===
        in_collision = min_obs_dist < self.combined_radius
        proximity = min_obs_dist < self.safe_distance

        if in_collision:
            action = 'flee'
            reason = f'collision_with_{nearest_obs}'
            confidence = 1.0
        elif proximity:
            action = 'avoid'
            reason = f'proximity_dist={min_obs_dist:.2f}m'
            confidence = 0.8
            self.stats['near_miss'] += 1
        elif min_obs_dist < self.soc_range:
            action = 'avoid'
            reason = f'social_force_d={min_obs_dist:.2f}m'
            confidence = 0.7
        else:
            action = 'cruise'
            reason = 'clear'
            confidence = 1.0

        conflict_detected = min_obs_dist < self.soc_range or min_obs_dist < self.safe_distance
        ttc = ttc_min if ttc_min != float('inf') else 999.0

        # 角速度
        speed_out = math.sqrt(smooth_vx ** 2 + smooth_vy ** 2)
        if speed_out > 1e-6:
            vel_angle = math.atan2(smooth_vy, smooth_vx)
            ang_diff = vel_angle - robot_yaw
            while ang_diff > math.pi:
                ang_diff -= 2 * math.pi
            while ang_diff < -math.pi:
                ang_diff += 2 * math.pi
            wz = max(-2.0, min(2.0, ang_diff / self.dt))
        else:
            wz = 0.0

        return AvoidanceCommand(
            vx=float(smooth_vx),
            vy=float(smooth_vy),
            wz=wz,
            action=action,
            reason=reason,
            confidence=confidence,
            conflict_detected=conflict_detected,
            time_to_conflict=ttc,
        )

    def reset(self):
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._stagnation_count = 0
        self._last_positions = []
        self.stats = {k: 0 for k in self.stats}
