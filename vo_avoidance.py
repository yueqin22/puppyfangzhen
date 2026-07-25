"""VO: Velocity Obstacle 速度障碍法

经典速度空间避障算法 (Fiorini & Shiller 1998)。

核心思想：
    在速度空间中，计算会导致碰撞的速度集合——称为"速度障碍锥"(VO)。
    选择不在任何VO锥内的速度，就能保证在预测时间内不碰撞。

数学表达：
    对于行人B，相对位置 r = B.pos - A.pos
    相对速度 v_rel = A.vel - B.vel
    combined_radius R = R_A + R_B
    距离 d = |r|

    方向单位向量: u = r / d
    半角: alpha = arcsin(R / d)
    左边界: u 旋转 +alpha
    右边界: u 旋转 -alpha

    VO 锥 = { v_rel | 方向在 [right, left] 范围内 }

    选择 v_A 使得 v_A - v_B 不在 VO 锥内，且最接近 preferred_velocity

与ORCA的区别：
    - VO假设对方不动（不避让），全部避让责任在我方
    - ORCA假设对方各让一半，更高效
    - VO更保守，振荡更少但可能避让过度

参考：
    Paolo Fiorini, Zvi Shiller. "Motion Planning in Dynamic
    Environments Using Velocity Obstacles." IJRR 1998.
"""
import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from stvoc_avoidance import AvoidanceCommand


class VOController:
    """VO 速度障碍法控制器

    用法:
        vo = VOController(dt=1/30, max_speed=6.0,
                          robot_radius=0.25, obstacle_radius=0.30,
                          time_horizon=1.5)
        cmd = vo.compute_avoidance(rx, ry, yaw, vx, vy, tx, ty, obstacles)
    """

    def __init__(self,
                 dt: float = 1/30,
                 max_speed: float = 6.0,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30,
                 time_horizon: float = 1.5,
                 safe_distance: float = 0.55):
        """初始化VO控制器

        Args:
            dt: 控制周期（秒）
            max_speed: 最大线速度（m/s）
            robot_radius: 机器人半径
            obstacle_radius: 行人半径
            time_horizon: 预测时间范围（秒）
            safe_distance: 近距保护触发距离
        """
        self.dt = dt
        self.max_speed = max_speed
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.combined_radius = robot_radius + obstacle_radius
        self.time_horizon = time_horizon
        self.safe_distance = safe_distance

        # 速度采样数（用于在VO外找最优速度）
        self._num_samples = 36  # 36方向采样

        # 统计
        self.stats = {
            'vo_computes': 0,
            'vo_cones': 0,
            'all_in_vo': 0,  # 所有方向都在VO内的次数
            'near_miss': 0,
        }

        # 速度平滑
        self._last_vx = 0.0
        self._last_vy = 0.0

    def compute_avoidance(self,
                          robot_x: float, robot_y: float, robot_yaw: float,
                          robot_vx: float, robot_vy: float,
                          target_x: float, target_y: float,
                          obstacles: List[Tuple]) -> AvoidanceCommand:
        """计算VO避障指令

        Args:
            robot_x, robot_y: 机器人位置
            robot_yaw: 机器人朝向
            robot_vx, robot_vy: 机器人当前速度（m/s）
            target_x, target_y: 目标位置
            obstacles: 行人列表 [(name, x, y, vx, vy, pattern), ...]

        Returns:
            AvoidanceCommand
        """
        self.stats['vo_computes'] += 1

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

        # === 步骤1: 对每个行人，判断是否在VO锥内 ===
        # 收集所有VO锥（用半角表示）
        vo_cones = []  # [(axis_angle, half_angle, obs_vel), ...]
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

            if dist < 1e-6:
                continue

            if dist > self.time_horizon * self.max_speed * 1.5:
                # 太远，time_horizon内不会碰
                continue

            # 半角
            if dist <= self.combined_radius:
                half_angle = math.pi  # 已碰撞，全空间都是VO
            else:
                half_angle = math.asin(min(1.0, self.combined_radius / dist))

            # 轴线方向（从机器人指向行人，在相对速度空间中）
            # VO锥的轴线方向 = 相对位置方向（即：如果相对速度沿这个方向，会碰撞）
            axis_angle = math.atan2(rel_pos[1], rel_pos[0])

            # 计算TTC
            rel_vel = robot_vel - obs_vel
            rel_speed_along = float(np.dot(rel_vel, rel_pos / dist))
            if rel_speed_along > 0 and dist > self.combined_radius:
                t = (dist - self.combined_radius) / rel_speed_along
                if t < ttc_min and t < self.time_horizon:
                    ttc_min = t

            vo_cones.append((axis_angle, half_angle, obs_vel))

        self.stats['vo_cones'] += len(vo_cones)

        # === 步骤2: 在速度空间采样，找最优安全速度 ===
        # 策略：在速度圆上均匀采样方向，找到不在任何VO锥内且最接近preferred的方向
        best_vel = None
        best_cost = float('inf')

        # 加上preferred方向作为候选（如果安全的话直接用）
        candidates = []

        # 均匀采样方向
        for i in range(self._num_samples):
            angle = 2 * math.pi * i / self._num_samples
            candidates.append(angle)

        # 加上preferred方向
        pref_angle = math.atan2(preferred_vel[1], preferred_vel[0])
        candidates.append(pref_angle)
        # 加上当前速度方向
        if abs(robot_vx) > 1e-6 or abs(robot_vy) > 1e-6:
            curr_angle = math.atan2(robot_vy, robot_vx)
            candidates.append(curr_angle)

        for angle in candidates:
            vel = np.array([math.cos(angle), math.sin(angle)]) * self.max_speed

            # 检查是否在任何VO锥内
            in_any_vo = False
            min_margin = float('inf')  # 到最近VO边界的距离（角度差）

            for axis_angle, half_angle, obs_vel in vo_cones:
                # 相对速度 = 机器人速度 - 行人速度
                rel_v = vel - obs_vel
                if np.linalg.norm(rel_v) < 1e-6:
                    continue  # 零速度，不构成VO冲突

                rel_angle = math.atan2(rel_v[1], rel_v[0])

                # 角度差（归一化到[-pi, pi]）
                angle_diff = rel_angle - axis_angle
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                if abs(angle_diff) < half_angle:
                    in_any_vo = True
                    break

                margin = abs(angle_diff) - half_angle
                if margin < min_margin:
                    min_margin = margin

            if in_any_vo:
                continue  # 这个速度会碰撞，跳过

            # 计算代价：与preferred速度的差
            cost = float(np.linalg.norm(vel - preferred_vel))

            if cost < best_cost:
                best_cost = cost
                best_vel = vel

        # === 步骤3: 如果所有方向都在VO内，找"最安全"的方向 ===
        if best_vel is None and vo_cones:
            self.stats['all_in_vo'] += 1
            # 找离所有VO边界最远的方向
            best_margin = -float('inf')
            best_angle = 0.0
            for i in range(72):  # 更密的采样
                angle = 2 * math.pi * i / 72
                vel = np.array([math.cos(angle), math.sin(angle)]) * self.max_speed * 0.5  # 减速

                min_margin = float('inf')
                for axis_angle, half_angle, obs_vel in vo_cones:
                    rel_v = vel - obs_vel
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

        # === 步骤4: 速度平滑 + 限幅 ===
        alpha = 0.6
        smooth_vx = alpha * best_vel[0] + (1 - alpha) * self._last_vx
        smooth_vy = alpha * best_vel[1] + (1 - alpha) * self._last_vy
        speed = math.sqrt(smooth_vx ** 2 + smooth_vy ** 2)
        if speed > self.max_speed:
            smooth_vx = smooth_vx * (self.max_speed / speed)
            smooth_vy = smooth_vy * (self.max_speed / speed)

        self._last_vx = float(smooth_vx)
        self._last_vy = float(smooth_vy)

        # === 步骤5: 决策分类 ===
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
        elif len(vo_cones) > 0:
            action = 'avoid'
            reason = f'vo_{len(vo_cones)}_cones'
            confidence = 0.7
        else:
            action = 'cruise'
            reason = 'clear'
            confidence = 1.0

        conflict_detected = len(vo_cones) > 0 or min_obs_dist < self.safe_distance
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
        """重置状态"""
        self._last_vx = 0.0
        self._last_vy = 0.0
        self.stats = {k: 0 for k in self.stats}
