"""ORCA: Optimal Reciprocal Collision Avoidance 最优互惠避障

经典多智能体避障算法 (van den Berg et al. 2011)。

核心思想：
    传统VO (Velocity Obstacle) 假设对方不动，导致双方都过度避让，
    出现振荡。ORCA假设对方也会避让，因此每方只需避让一半，
    形成互惠（reciprocal）行为，避免过度反应和振荡。

算法步骤：
    1. 对每个行人B，计算机器人A的VO锥
    2. 取VO锥的一半作为ORCA半平面（假设B也避让另一半）
    3. 在所有半平面的交集中，求解线性规划：
       找到最接近 preferred_velocity 的速度 v_new
    4. 速度限幅到 max_speed

数学推导：
    相对位置 r = B.pos - A.pos
    相对速度 v_rel = A.vel - B.vel
    combined_radius R = R_A + R_B
    距离 d = |r|

    如果 d > R:
        dir = r / d
        cutoff_dist = d - R
        approaching_speed = -v_rel · dir  (正值表示靠近)
        if approaching_speed <= 0: 无VO（远离）
        else:
            t_cutoff = cutoff_dist / approaching_speed
            if t_cutoff > time_horizon: 不考虑（太远）
            # cutoff point 在 v_rel 中切去VO锥尖端
            cutoff_point = v_rel + dir * (cutoff_dist / time_horizon)
            u = cutoff_point - v_rel  # 机器人需要调整的速度
            normal = u / |u|
            point = A.vel + 0.5 * u   # 双方各避让一半
    如果 d <= R: 已碰撞，沿 -dir 方向推开

ORCA优势:
    - 计算量小: O(n) per agent (n = 行人数)
    - 互惠: 假设对方也避让，避免双方都让
    - 速度最优: LP求解最接近目标方向的速度
    - 无振荡: 双方各避一半，不会过度让步

参考:
    Jur van den Berg, Stephen J. Guy, Ming Lin, Dinesh Manocha.
    "Reciprocal n-Body Collision Avoidance." ISRR 2009.
"""
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np

# 复用STVOC的AvoidanceCommand保持接口一致
from stvoc_avoidance import AvoidanceCommand


# ====================================================================
# ORCA 半平面
# ====================================================================

@dataclass
class ORCAPlane:
    """ORCA半平面: {v : (v - point) · normal >= 0}

    机器人新速度必须落在此半平面内才安全。
    """
    normal: np.ndarray   # 单位法向量（指向安全区域）
    point: np.ndarray    # 半平面上的一个点


# ====================================================================
# ORCA 控制器
# ====================================================================

class ORCAController:
    """ORCA 最优互惠避障控制器

    用法:
        orca = ORCAController(dt=1/30, max_speed=6.0,
                              robot_radius=0.25, obstacle_radius=0.30,
                              time_horizon=0.5)
        cmd = orca.compute_avoidance(robot_x, robot_y, yaw,
                                     vx, vy, tx, ty, obstacles)
    """

    def __init__(self,
                 dt: float = 1/30,
                 max_speed: float = 6.0,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30,
                 time_horizon: float = 0.5,
                 safe_distance: float = 0.55):
        """初始化ORCA控制器

        Args:
            dt: 控制周期（秒）
            max_speed: 机器人最大线速度（m/s）
            robot_radius: 机器人半径（米）
            obstacle_radius: 行人半径（米）
            time_horizon: 预测时间范围（秒），超出此时间的冲突忽略
            safe_distance: 触发主动避让的距离阈值（米）
        """
        self.dt = dt
        self.max_speed = max_speed
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.combined_radius = robot_radius + obstacle_radius
        self.time_horizon = time_horizon
        self.safe_distance = safe_distance

        # 统计
        self.stats = {
            'orca_computes': 0,
            'planes_count': 0,
            'speed_clipped': 0,
            'collision_active': 0,
            'proximity_avoid': 0,
        }

        # 平滑：缓存上一次速度，用于速度连续性
        self._last_vel = np.array([0.0, 0.0])

    def compute_avoidance(self,
                          robot_x: float, robot_y: float, robot_yaw: float,
                          robot_vx: float, robot_vy: float,
                          target_x: float, target_y: float,
                          obstacles: List[Tuple]) -> AvoidanceCommand:
        """计算ORCA避避让指令

        Args:
            robot_x, robot_y: 机器人当前位置
            robot_yaw: 机器人朝向（弧度）
            robot_vx, robot_vy: 机器人当前速度（m/s）
            target_x, target_y: 目标位置
            obstacles: 行人列表 [(name, x, y, vx, vy, pattern), ...]
                      vx/vy 单位 m/s

        Returns:
            AvoidanceCommand: 避障指令
        """
        self.stats['orca_computes'] += 1

        robot_pos = np.array([robot_x, robot_y])
        robot_vel = np.array([robot_vx, robot_vy])

        # 期望速度：朝向目标方向，速度= max_speed
        dx = target_x - robot_x
        dy = target_y - robot_y
        target_dist = math.sqrt(dx * dx + dy * dy)
        if target_dist > 1e-6:
            pref_dir = np.array([dx / target_dist, dy / target_dist])
        else:
            pref_dir = np.array([1.0, 0.0])
        # 距离近时降速
        speed_scale = min(1.0, target_dist / 0.5)
        preferred_vel = pref_dir * self.max_speed * speed_scale

        # === 步骤1: 对每个行人生成ORCA半平面 ===
        orca_planes: List[ORCAPlane] = []
        min_obs_dist = float('inf')
        nearest_obs = None
        nearest_obs_vel = np.array([0.0, 0.0])
        in_collision = False
        ttc_min = float('inf')

        for name, ox, oy, ovx, ovy, _pattern in obstacles:
            obs_pos = np.array([ox, oy])
            obs_vel = np.array([ovx, ovy])
            rel_pos = obs_pos - robot_pos
            rel_vel = robot_vel - obs_vel
            dist = float(np.linalg.norm(rel_pos))

            if dist < min_obs_dist:
                min_obs_dist = dist
                nearest_obs = name
                nearest_obs_vel = obs_vel

            if dist < self.combined_radius:
                # 已碰撞：生成推开半平面
                in_collision = True
                if dist > 1e-6:
                    normal = -rel_pos / dist  # 指向远离行人方向
                else:
                    normal = np.array([1.0, 0.0])
                point = obs_vel * 0.5 + robot_vel * 0.5
                orca_planes.append(ORCAPlane(normal=normal, point=point))
                continue

            if dist < 1e-6:
                continue

            dir_to_obs = rel_pos / dist
            # rel_vel = robot_vel - obs_vel（机器人相对行人的速度）
            # dir_to_obs 从机器人指向行人
            # 碰撞条件: |rel_pos - rel_vel * t| < R
            # 简化：rel_vel · dir_to_obs > 0 表示机器人在朝行人方向移动（相对行人而言）
            vel_along_dir = float(np.dot(rel_vel, dir_to_obs))

            if vel_along_dir <= 0:
                # 机器人在远离行人（在行人参考系下），无VO
                continue

            # 计算 v_rel 进入碰撞圆的最近时间
            # |rel_pos - rel_vel * t| = R → a*t² - 2b*t + c = 0
            # a = rel_vel · rel_vel, b = rel_pos · rel_vel, c = rel_pos · rel_pos - R²
            a_coef = float(np.dot(rel_vel, rel_vel))
            c_coef = float(np.dot(rel_pos, rel_pos) - self.combined_radius ** 2)
            b_coef = float(np.dot(rel_pos, rel_vel))

            if a_coef < 1e-9:
                continue  # 相对速度太小

            disc = b_coef * b_coef - a_coef * c_coef
            if disc < 0:
                continue  # 无碰撞

            sqrt_disc = math.sqrt(disc)
            t_enter = (b_coef - sqrt_disc) / a_coef  # 进入碰撞时间
            # t_exit = (b_coef + sqrt_disc) / a_coef

            if t_enter < 0:
                # 已进入碰撞区域（应该已被dist<R分支处理）
                continue

            if t_enter > self.time_horizon:
                # 太远的冲突，忽略
                continue

            if t_enter < ttc_min:
                ttc_min = t_enter

            # === 计算 ORCA 半平面 ===
            # VO 锥的 leg 方向：从机器人位置到 combined_radius 圆切线
            # leg_angle = arcsin(R / dist)
            if dist > self.combined_radius:
                leg_angle = math.asin(min(1.0, self.combined_radius / dist))
                leg_len = math.sqrt(max(0.0, dist * dist - self.combined_radius ** 2))
                # 旋转 dir_to_obs ±leg_angle 得到两个 leg 方向
                cos_l = math.cos(leg_angle)
                sin_l = math.sin(leg_angle)
                leg_dir_1 = np.array([dir_to_obs[0] * cos_l - dir_to_obs[1] * sin_l,
                                       dir_to_obs[0] * sin_l + dir_to_obs[1] * cos_l])
                leg_dir_2 = np.array([dir_to_obs[0] * cos_l + dir_to_obs[1] * sin_l,
                                      -dir_to_obs[0] * sin_l + dir_to_obs[1] * cos_l])
                # leg 上的目标速度方向：朝 leg 方向，大小 = leg_len / time_horizon
                leg_speed = leg_len / self.time_horizon
                v1 = leg_dir_1 * leg_speed
                v2 = leg_dir_2 * leg_speed
                # 选择让 v_rel 离 leg 更近的 leg（推出VO的最短路径）
                d1 = float(np.linalg.norm(rel_vel - v1))
                d2 = float(np.linalg.norm(rel_vel - v2))
                if d1 < d2:
                    u = v1 - rel_vel
                else:
                    u = v2 - rel_vel
            else:
                # 距离过近，使用径向推开
                u = -dir_to_obs * (self.combined_radius - dist) / self.time_horizon - rel_vel

            u_norm = float(np.linalg.norm(u))
            if u_norm < 1e-9:
                continue

            normal = u / u_norm
            # 双方各避让一半: point = robot_vel + 0.5*u
            point = robot_vel + 0.5 * u
            orca_planes.append(ORCAPlane(normal=normal, point=point))

        self.stats['planes_count'] += len(orca_planes)

        # === 步骤2: 在半平面交集中求LP最优速度 ===
        new_vel = self._solve_lp(preferred_vel, orca_planes)

        # === 步骤3: 速度平滑 + 限幅 ===
        # 速度连续性：与上次速度做插值，避免突变
        alpha = 0.6  # 60%新速度 + 40%旧速度
        new_vel = alpha * new_vel + (1 - alpha) * self._last_vel

        speed = float(np.linalg.norm(new_vel))
        if speed > self.max_speed:
            new_vel = new_vel * (self.max_speed / speed)
            self.stats['speed_clipped'] += 1
            speed = self.max_speed

        self._last_vel = new_vel

        # === 步骤4: 决策分类 ===
        if in_collision:
            action = 'flee'
            reason = f'collision_with_{nearest_obs}'
            confidence = 1.0
            self.stats['collision_active'] += 1
        elif min_obs_dist < self.safe_distance:
            # 近距行人保护：强制减速
            # 但ORCA已经计算了避让速度，这里只降速不改变方向
            action = 'avoid'
            reason = f'proximity_dist={min_obs_dist:.2f}m'
            confidence = 0.9
            self.stats['proximity_avoid'] += 1
        elif len(orca_planes) > 0:
            action = 'avoid'
            reason = f'orca_{len(orca_planes)}_planes_ttc={ttc_min:.2f}s'
            confidence = 0.7
        else:
            action = 'cruise'
            reason = 'clear'
            confidence = 1.0

        conflict_detected = len(orca_planes) > 0 or min_obs_dist < self.safe_distance
        ttc = ttc_min if ttc_min != float('inf') else 999.0

        # 计算角速度：用速度方向与机器人当前朝向的差
        if speed > 1e-6:
            vel_angle = math.atan2(new_vel[1], new_vel[0])
            ang_diff = vel_angle - robot_yaw
            # 归一化到[-pi, pi]
            while ang_diff > math.pi:
                ang_diff -= 2 * math.pi
            while ang_diff < -math.pi:
                ang_diff += 2 * math.pi
            wz = max(-2.0, min(2.0, ang_diff / self.dt))
        else:
            wz = 0.0

        return AvoidanceCommand(
            vx=float(new_vel[0]),
            vy=float(new_vel[1]),
            wz=wz,
            action=action,
            reason=reason,
            confidence=confidence,
            conflict_detected=conflict_detected,
            time_to_conflict=ttc,
        )

    def _solve_lp(self, preferred_vel: np.ndarray,
                  planes: List[ORCAPlane]) -> np.ndarray:
        """求解线性规划：在半平面交集中找最接近 preferred_vel 的速度

        采用迭代投影法（类似SOR方法），复杂度O(iterations * n_planes)
        对于少量行人是足够快的。

        Args:
            preferred_vel: 期望速度
            planes: ORCA半平面列表

        Returns:
            最优安全速度
        """
        if not planes:
            # 无约束，直接返回期望速度（限幅）
            v = preferred_vel.copy()
            speed = float(np.linalg.norm(v))
            if speed > self.max_speed:
                v = v * (self.max_speed / speed)
            return v

        # 迭代投影：每次将当前速度投影到违反的半平面边界上
        v = preferred_vel.copy()
        # 速度上限的"软"约束也作为一个圆形约束
        max_iterations = 3  # 经验值，3次足够收敛

        for _ in range(max_iterations):
            for plane in planes:
                diff = v - plane.point
                proj = float(np.dot(diff, plane.normal))
                if proj < 0:  # 违反约束，投影到边界
                    v = v - proj * plane.normal

            # 速度限幅
            speed = float(np.linalg.norm(v))
            if speed > self.max_speed:
                v = v * (self.max_speed / speed)

        return v

    def reset(self):
        """重置控制器状态"""
        self._last_vel = np.array([0.0, 0.0])
        self.stats = {k: 0 for k in self.stats}
