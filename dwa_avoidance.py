"""DWA: Dynamic Window Approach 动态窗口法

经典局部规划算法 (Fox, Burgard, Thrun 1997)。

核心思想：
    在(v, omega)速度空间中采样，模拟短期轨迹，
    用评分函数选择最优速度：
    score = alpha * heading + beta * dist + gamma * velocity

    heading: 朝向目标的程度（角度越小越好）
    dist: 轨迹到最近障碍物的距离（越大越好）
    velocity: 速度大小（越快越好）

动态窗口：
    只考虑在加速度限制内可达的速度：
    v ∈ [v - a_max * dt, v + a_max * dt]
    omega ∈ [omega - alpha_max * dt, omega + alpha_max * dt]

与VO/RVO的区别：
    - DWA考虑机器人运动学约束（加速度限制、非完整约束）
    - DWA模拟完整轨迹而非单步速度
    - DWA同时考虑静态和动态障碍物
    - DWA更适合差速驱动机器人

参考：
    Dieter Fox, Wolfram Burgard, Sebastian Thrun.
    "The Dynamic Window Approach to Collision Avoidance."
    IEEE Robotics & Automation Magazine, 1997.
"""
import math
from typing import List, Tuple

import numpy as np

from stvoc_avoidance import AvoidanceCommand


class DWAController:
    """DWA 动态窗口法控制器"""

    def __init__(self,
                 dt: float = 1/30,
                 max_speed: float = 6.0,
                 max_angular: float = 2.0,
                 accel_limit: float = 3.0,
                 angular_accel_limit: float = 4.0,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30,
                 safe_distance: float = 0.55,
                 predict_steps: int = 15,
                 num_v_samples: int = 9,
                 num_w_samples: int = 15):
        """初始化DWA控制器

        Args:
            dt: 控制周期
            max_speed: 最大线速度
            max_angular: 最大角速度
            accel_limit: 最大线加速度
            angular_accel_limit: 最大角加速度
            robot_radius: 机器人半径
            obstacle_radius: 行人半径
            safe_distance: 近距保护距离
            predict_steps: 轨迹预测步数
            num_v_samples: 线速度采样数
            num_w_samples: 角速度采样数
        """
        self.dt = dt
        self.max_speed = max_speed
        self.max_angular = max_angular
        self.accel_limit = accel_limit
        self.angular_accel_limit = angular_accel_limit
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.combined_radius = robot_radius + obstacle_radius
        self.safe_distance = safe_distance
        self.predict_steps = predict_steps
        self.num_v_samples = num_v_samples
        self.num_w_samples = num_w_samples

        # 评分权重
        self.alpha_heading = 0.4   # 朝向目标
        self.alpha_dist = 0.4      # 障碍物距离
        self.alpha_vel = 0.2       # 速度

        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0

        self.stats = {
            'dwa_computes': 0,
            'near_miss': 0,
            'collision_trajs': 0,
        }

    def _simulate_trajectory(self, x, y, yaw, v, wz, steps):
        """模拟给定速度下的轨迹

        返回轨迹点列表 [(x, y, yaw), ...]
        """
        traj = []
        cx, cy, cyaw = x, y, yaw
        for _ in range(steps):
            cx = cx + v * math.cos(cyaw) * self.dt
            cy = cy + v * math.sin(cyaw) * self.dt
            cyaw = cyaw + wz * self.dt
            traj.append((cx, cy, cyaw))
        return traj

    def _traj_min_obstacle_dist(self, traj, obstacles):
        """计算轨迹到所有行人的最小距离"""
        min_dist = float('inf')
        collision = False
        for tx, ty, _ in traj:
            for name, ox, oy, ovx, ovy, _ in obstacles:
                # 预测行人位置（线性外推）
                for step, (tx2, ty2, _) in enumerate([(tx, ty, 0)]):
                    pass
                # 简化：用当前位置 + 速度 * step * dt
                d = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
                if d < min_dist:
                    min_dist = d
                if d < self.combined_radius:
                    collision = True
        return min_dist, collision

    def _traj_min_obstacle_dist_predicted(self, traj, obstacles):
        """计算轨迹到行人预测位置的最小距离（考虑行人运动）"""
        min_dist = float('inf')
        collision = False
        for step, (tx, ty, _) in enumerate(traj):
            t = step * self.dt
            for name, ox, oy, ovx, ovy, _ in obstacles:
                # 行人预测位置
                px = ox + ovx * t
                py = oy + ovy * t
                d = math.sqrt((tx - px) ** 2 + (ty - py) ** 2)
                if d < min_dist:
                    min_dist = d
                if d < self.combined_radius:
                    collision = True
        return min_dist, collision

    def compute_avoidance(self,
                          robot_x: float, robot_y: float, robot_yaw: float,
                          robot_vx: float, robot_vy: float,
                          target_x: float, target_y: float,
                          obstacles: List[Tuple]) -> AvoidanceCommand:
        """计算DWA避障指令"""
        self.stats['dwa_computes'] += 1

        # 当前速度
        curr_speed = math.sqrt(robot_vx ** 2 + robot_vy ** 2)
        curr_wz = self._last_wz

        # === 动态窗口 ===
        v_min = max(0, curr_speed - self.accel_limit * self.dt)
        v_max = min(self.max_speed, curr_speed + self.accel_limit * self.dt)
        w_min = max(-self.max_angular, curr_wz - self.angular_accel_limit * self.dt)
        w_max = min(self.max_angular, curr_wz + self.angular_accel_limit * self.dt)

        # 距离太近时允许后退
        min_obs_dist = float('inf')
        nearest_obs = None
        for name, ox, oy, _, _, _ in obstacles:
            d = math.sqrt((robot_x - ox) ** 2 + (robot_y - oy) ** 2)
            if d < min_obs_dist:
                min_obs_dist = d
                nearest_obs = name

        if min_obs_dist < self.safe_distance:
            v_min = max(-self.max_speed * 0.5, curr_speed - self.accel_limit * self.dt)

        # === 采样评分 ===
        best_score = -float('inf')
        best_v = 0.0
        best_w = 0.0
        best_traj = None

        for i in range(self.num_v_samples):
            v = v_min + (v_max - v_min) * i / max(1, self.num_v_samples - 1)
            for j in range(self.num_w_samples):
                w = w_min + (w_max - w_min) * j / max(1, self.num_w_samples - 1)

                # 模拟轨迹
                traj = self._simulate_trajectory(
                    robot_x, robot_y, robot_yaw, v, w, self.predict_steps)

                # 计算轨迹到行人的最小距离（考虑行人运动）
                traj_dist, collision = self._traj_min_obstacle_dist_predicted(traj, obstacles)

                if collision:
                    self.stats['collision_trajs'] += 1
                    continue  # 碰撞轨迹跳过

                # === 评分 ===
                # 1. heading: 终点朝向目标的角度
                end_x, end_y, end_yaw = traj[-1]
                dx = target_x - end_x
                dy = target_y - end_y
                goal_angle = math.atan2(dy, dx)
                heading_diff = abs(goal_angle - end_yaw)
                while heading_diff > math.pi:
                    heading_diff -= 2 * math.pi
                heading_diff = abs(heading_diff)
                heading_score = math.pi - heading_diff  # [0, pi]

                # 2. dist: 障碍物距离
                if traj_dist >= 2.0:
                    dist_score = 1.0
                elif traj_dist <= self.combined_radius:
                    dist_score = 0.0
                else:
                    dist_score = (traj_dist - self.combined_radius) / (2.0 - self.combined_radius)
                    dist_score = max(0, min(1, dist_score))

                # 3. velocity
                vel_score = v / self.max_speed

                # 总分
                score = (self.alpha_heading * heading_score / math.pi +
                         self.alpha_dist * dist_score +
                         self.alpha_vel * vel_score)

                # 太近时偏好减速
                if min_obs_dist < self.safe_distance and v > 0:
                    score *= 0.5

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w
                    best_traj = traj

        # 如果所有轨迹都碰撞，减速+转向
        if best_traj is None:
            self.stats['collision_trajs'] += 1
            # 找最近的行人，远离它
            nearest = None
            nearest_d = float('inf')
            for name, ox, oy, _, _, _ in obstacles:
                d = math.sqrt((robot_x - ox) ** 2 + (robot_y - oy) ** 2)
                if d < nearest_d:
                    nearest_d = d
                    nearest = (ox, oy)

            if nearest:
                flee_dx = robot_x - nearest[0]
                flee_dy = robot_y - nearest[1]
                flee_ang = math.atan2(flee_dy, flee_dx)
                ang_diff = flee_ang - robot_yaw
                while ang_diff > math.pi:
                    ang_diff -= 2 * math.pi
                while ang_diff < -math.pi:
                    ang_diff += 2 * math.pi
                best_w = max(-self.max_angular, min(self.max_angular, ang_diff / self.dt))
                best_v = -0.5  # 后退

        # 转换为 vx, vy
        new_vx = best_v * math.cos(robot_yaw + best_w * self.dt)
        new_vy = best_v * math.sin(robot_yaw + best_w * self.dt)

        # 速度平滑
        alpha = 0.7
        smooth_vx = alpha * new_vx + (1 - alpha) * self._last_vx
        smooth_vy = alpha * new_vy + (1 - alpha) * self._last_vy

        self._last_vx = float(smooth_vx)
        self._last_vy = float(smooth_vy)
        self._last_wz = float(best_w)

        # TTC
        ttc_min = float('inf')
        for name, ox, oy, ovx, ovy, _ in obstacles:
            dx_o = robot_x - ox
            dy_o = robot_y - oy
            d = math.sqrt(dx_o ** 2 + dy_o ** 2)
            if d > 1e-6 and d < 3.0:
                rel_vx = robot_vx - ovx
                rel_vy = robot_vy - ovy
                approach = (rel_vx * dx_o + rel_vy * dy_o) / d
                if approach > 0 and d > self.combined_radius:
                    t = (d - self.combined_radius) / approach
                    if t < ttc_min:
                        ttc_min = t

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
        elif min_obs_dist < 2.0:
            action = 'avoid'
            reason = f'dwa_avoid_d={min_obs_dist:.2f}m'
            confidence = 0.7
        else:
            action = 'cruise'
            reason = 'clear'
            confidence = 1.0

        conflict_detected = min_obs_dist < 2.0 or min_obs_dist < self.safe_distance
        ttc = ttc_min if ttc_min != float('inf') else 999.0

        return AvoidanceCommand(
            vx=float(smooth_vx),
            vy=float(smooth_vy),
            wz=float(best_w),
            action=action,
            reason=reason,
            confidence=confidence,
            conflict_detected=conflict_detected,
            time_to_conflict=ttc,
        )

    def reset(self):
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0
        self.stats = {k: 0 for k in self.stats}
