"""RVO: Reciprocal Velocity Obstacle 互惠速度障碍法

在VO基础上加入互惠假设 (van den Berg et al. 2008)。

核心改进：
    VO假设对方不动，全部避让责任在我方（过度保守）。
    RVO假设对方也会避让，每方只需避让一半，形成互惠行为。

数学表达：
    VO_B^A = { v | v - v_B ∈ VO_cone(B relative to A) }

    RVO_B^A = { v | 2*v - v_A_current - v_B ∈ VO_B^A }
    即：将VO锥的顶点从 v_B 移到 (v_A + v_B) / 2

    选择 v_A 使得 v_A ∉ ∪ RVO_B^A，且最接近 preferred_velocity

与VO的区别：
    - VO锥顶点在 v_B（行人速度）
    - RVO锥顶点在 (v_A + v_B) / 2（双方速度均值）
    - RVO只排除半个锥，减少了过度避让和振荡

与ORCA的区别：
    - ORCA是RVO的进一步优化，用半平面代替锥
    - RVO保持锥形结构，实现更简单
    - 在非互惠场景中（行人不让），RVO比ORCA更保守（偏安全）

参考：
    Jur van den Berg, Ming Lin, Dinesh Manocha. "Reciprocal Velocity
    Obstacles for Real-Time Multi-Agent Navigation." ICRA 2008.
"""
import math
from typing import List, Tuple

import numpy as np

from stvoc_avoidance import AvoidanceCommand


class RVOController:
    """RVO 互惠速度障碍法控制器"""

    def __init__(self,
                 dt: float = 1/30,
                 max_speed: float = 6.0,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30,
                 time_horizon: float = 1.5,
                 safe_distance: float = 0.55,
                 neighbor_dist: float = 3.0):
        """初始化RVO控制器

        Args:
            dt: 控制周期（秒）
            max_speed: 最大线速度（m/s）
            robot_radius: 机器人半径
            obstacle_radius: 行人半径
            time_horizon: 预测时间范围（秒）
            safe_distance: 近距保护触发距离
            neighbor_dist: 只考虑此距离内的行人
        """
        self.dt = dt
        self.max_speed = max_speed
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.combined_radius = robot_radius + obstacle_radius
        self.time_horizon = time_horizon
        self.safe_distance = safe_distance
        self.neighbor_dist = neighbor_dist

        self._num_samples = 36
        self._last_vx = 0.0
        self._last_vy = 0.0

        self.stats = {
            'rvo_computes': 0,
            'rvo_cones': 0,
            'all_in_rvo': 0,
            'near_miss': 0,
        }

    def compute_avoidance(self,
                          robot_x: float, robot_y: float, robot_yaw: float,
                          robot_vx: float, robot_vy: float,
                          target_x: float, target_y: float,
                          obstacles: List[Tuple]) -> AvoidanceCommand:
        """计算RVO避障指令"""
        self.stats['rvo_computes'] += 1

        robot_pos = np.array([robot_x, robot_y])
        robot_vel = np.array([robot_vx, robot_vy])

        # 期望速度
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist_to_target = math.sqrt(dx * dx + dy * dy)
        if dist_to_target > 1e-6:
            pref_dir = np.array([dx / dist_to_target, dy / dist_to_target])
        else:
            pref_dir = np.array([1.0, 0.0])
        speed_scale = min(1.0, dist_to_target / 0.5)
        preferred_vel = pref_dir * self.max_speed * speed_scale

        # === 对每个行人计算RVO锥 ===
        rvo_cones = []  # [(apex_vx, apex_vy, axis_angle, half_angle)]
        min_obs_dist = float('inf')
        nearest_obs = None
        ttc_min = float('inf')

        for name, ox, oy, ovx, ovy, _pattern in obstacles:
            obs_pos = np.array([ox, oy])
            obs_vel = np.array([ovx, ovy])
            rel_pos = obs_pos - robot_pos
            dist = float(np.linalg.norm(rel_pos))

            if dist < min_obs_dist:
                min_obs_dist = dist
                nearest_obs = name

            if dist < 1e-6 or dist > self.neighbor_dist:
                continue

            # 半角
            if dist <= self.combined_radius:
                half_angle = math.pi
            else:
                half_angle = math.asin(min(1.0, self.combined_radius / dist))

            # RVO锥的轴线方向（从机器人指向行人）
            axis_angle = math.atan2(rel_pos[1], rel_pos[0])

            # RVO的关键改进：锥顶点在 (v_A + v_B) / 2
            # 而不是VO的 v_B
            apex_vx = (robot_vx + ovx) / 2.0
            apex_vy = (robot_vy + ovy) / 2.0

            rvo_cones.append((apex_vx, apex_vy, axis_angle, half_angle))

            # TTC计算
            rel_vel = robot_vel - obs_vel
            rel_speed_along = float(np.dot(rel_vel, rel_pos / dist)) if dist > 1e-6 else 0
            if rel_speed_along > 0 and dist > self.combined_radius:
                t = (dist - self.combined_radius) / rel_speed_along
                if t < ttc_min and t < self.time_horizon:
                    ttc_min = t

        self.stats['rvo_cones'] += len(rvo_cones)

        # === 在速度空间采样找最优安全速度 ===
        best_vel = None
        best_cost = float('inf')

        candidates = []
        for i in range(self._num_samples):
            angle = 2 * math.pi * i / self._num_samples
            candidates.append(angle)
        # preferred方向
        pref_angle = math.atan2(preferred_vel[1], preferred_vel[0])
        candidates.append(pref_angle)
        # 当前速度方向
        if abs(robot_vx) > 1e-6 or abs(robot_vy) > 1e-6:
            candidates.append(math.atan2(robot_vy, robot_vx))

        for angle in candidates:
            vel = np.array([math.cos(angle), math.sin(angle)]) * self.max_speed

            # 检查是否在任何RVO锥内
            in_any_rvo = False
            for apex_vx, apex_vy, axis_angle, half_angle in rvo_cones:
                # 相对速度 = v_A - apex（RVO: 减去均值速度）
                rel_v = vel - np.array([apex_vx, apex_vy])
                if np.linalg.norm(rel_v) < 1e-6:
                    continue

                rel_angle = math.atan2(rel_v[1], rel_v[0])
                angle_diff = rel_angle - axis_angle
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if abs(angle_diff) < half_angle:
                    in_any_rvo = True
                    break

            if in_any_rvo:
                continue

            cost = float(np.linalg.norm(vel - preferred_vel))
            if cost < best_cost:
                best_cost = cost
                best_vel = vel

        # === 所有方向都在RVO内 ===
        if best_vel is None and rvo_cones:
            self.stats['all_in_rvo'] += 1
            best_margin = -float('inf')
            best_angle = 0.0
            for i in range(72):
                angle = 2 * math.pi * i / 72
                vel = np.array([math.cos(angle), math.sin(angle)]) * self.max_speed * 0.5

                min_margin = float('inf')
                for apex_vx, apex_vy, axis_angle, half_angle in rvo_cones:
                    rel_v = vel - np.array([apex_vx, apex_vy])
                    if np.linalg.norm(rel_v) < 1e-6:
                        margin = 0.0
                    else:
                        rel_angle = math.atan2(rel_v[1], rel_v[0])
                        angle_diff = rel_angle - axis_angle
                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                            angle_diff += 2 * math.pi
                        margin = abs(angle_diff) - half_angle
                    if margin < min_margin:
                        min_margin = margin

                if min_margin > best_margin:
                    best_margin = min_margin
                    best_angle = angle

            best_vel = np.array([math.cos(best_angle), math.sin(best_angle)]) * self.max_speed * 0.3

        if best_vel is None:
            best_vel = preferred_vel.copy()

        # 速度平滑
        alpha = 0.6
        smooth_vx = alpha * best_vel[0] + (1 - alpha) * self._last_vx
        smooth_vy = alpha * best_vel[1] + (1 - alpha) * self._last_vy
        speed = math.sqrt(smooth_vx ** 2 + smooth_vy ** 2)
        if speed > self.max_speed:
            smooth_vx = smooth_vx * (self.max_speed / speed)
            smooth_vy = smooth_vy * (self.max_speed / speed)

        self._last_vx = float(smooth_vx)
        self._last_vy = float(smooth_vy)

        # 决策分类
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
        elif len(rvo_cones) > 0:
            action = 'avoid'
            reason = f'rvo_{len(rvo_cones)}_cones'
            confidence = 0.7
        else:
            action = 'cruise'
            reason = 'clear'
            confidence = 1.0

        conflict_detected = len(rvo_cones) > 0 or min_obs_dist < self.safe_distance
        ttc = ttc_min if ttc_min != float('inf') else 999.0

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
        self.stats = {k: 0 for k in self.stats}
