"""动态障碍物时空规划 (v6.7)

集成动态障碍物跟踪和速度障碍到局部规划中。
预测动态障碍物的未来轨迹，并规划时移路径避开它们。

功能：
  1. 接收动态障碍物跟踪器数据
  2. 预测障碍物未来轨迹
  3. 计算速度障碍锥
  4. 在局部规划器的速度空间中排除VO区域
"""
import os
import math
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

USE_SPATIOTEMPORAL = os.environ.get("USE_SPATIOTEMPORAL", "0") == "1"


@dataclass
class DynamicObstacleState:
    """动态障碍物状态"""
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.3
    confidence: float = 1.0


class SpatioTemporalPlanner:
    """时空规划器 (v6.7)

    在局部规划器的速度空间中排除会导致与动态障碍物碰撞的速度。
    """

    def __init__(self, config=None):
        cfg = config or {}
        self.robot_radius = cfg.get('robot_radius', 0.35)
        self.time_horizon = cfg.get('time_horizon', 2.0)  # 预测时域(秒)
        self.dt = cfg.get('dt', 0.1)
        self.safety_margin = cfg.get('safety_margin', 0.1)

    def predict_obstacle_trajectory(self, obstacle: DynamicObstacleState, n_steps=20):
        """预测障碍物未来轨迹

        Args:
            obstacle: 动态障碍物当前状态
            n_steps: 预测步数

        Returns:
            list of (x, y, t): 未来位置和时间
        """
        trajectory = []
        for i in range(n_steps):
            t = i * self.dt
            x = obstacle.x + obstacle.vx * t
            y = obstacle.y + obstacle.vy * t
            trajectory.append((x, y, t))
        return trajectory

    def compute_velocity_obstacle(self, robot_x, robot_y, obstacle: DynamicObstacleState):
        """计算速度障碍锥

        返回速度空间中被禁止的区域描述。

        Returns:
            (apex_angle, left_bound, right_bound, min_dist, max_dist)
            apex_angle: 障碍物相对机器人的角度
            left_bound, right_bound: VO锥的左右边界角度
            min_dist, max_dist: VO锥的距离范围
        """
        dx = obstacle.x - robot_x
        dy = obstacle.y - robot_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 1e-6:
            return None

        apex_angle = math.atan2(dy, dx)

        # VO锥的半角
        combined_radius = self.robot_radius + obstacle.radius + self.safety_margin
        if dist > combined_radius:
            half_angle = math.asin(combined_radius / dist)
        else:
            half_angle = math.pi / 2  # 已碰撞

        left_bound = apex_angle - half_angle
        right_bound = apex_angle + half_angle

        return (apex_angle, left_bound, right_bound, combined_radius, dist)

    def filter_velocity(self, v, w, robot_x, robot_y, robot_yaw,
                        obstacles: List[DynamicObstacleState]):
        """过滤速度命令，避开动态障碍物

        如果速度(v, w)会导致与任何动态障碍物碰撞，
        尝试调整速度方向。

        Returns:
            (adjusted_v, adjusted_w, is_safe): 调整后的速度和安全性标志
        """
        if not obstacles:
            return v, w, True

        is_safe = True
        best_v, best_w = v, w
        min_collision_time = float('inf')

        for obs in obstacles:
            # 预测机器人和障碍物的未来位置
            for step in range(int(self.time_horizon / self.dt)):
                t = step * self.dt
                # 机器人未来位置
                pred_rx = robot_x + v * math.cos(robot_yaw) * t
                pred_ry = robot_y + v * math.sin(robot_yaw) * t
                # 障碍物未来位置
                pred_ox = obs.x + obs.vx * t
                pred_oy = obs.y + obs.vy * t
                # 距离
                dist = math.sqrt((pred_rx - pred_ox)**2 + (pred_ry - pred_oy)**2)
                collision_dist = self.robot_radius + obs.radius + self.safety_margin
                if dist < collision_dist:
                    collision_time = t
                    if collision_time < min_collision_time:
                        min_collision_time = collision_time
                    is_safe = False
                    break

        if not is_safe and min_collision_time < 1.0:
            # 即将碰撞，尝试减速或转向
            # 简化策略：减速到原来的一半
            adjusted_v = v * 0.3
            # 尝试找到安全转向方向
            best_w_candidate = w
            for w_try in [w + 0.5, w - 0.5, w + 1.0, w - 1.0]:
                safe = True
                for step in range(int(self.time_horizon / self.dt)):
                    t = step * self.dt
                    pred_yaw = robot_yaw + w_try * t
                    pred_rx = robot_x + v * 0.3 * math.cos(pred_yaw) * t
                    pred_ry = robot_y + v * 0.3 * math.sin(pred_yaw) * t
                    for obs in obstacles:
                        pred_ox = obs.x + obs.vx * t
                        pred_oy = obs.y + obs.vy * t
                        dist = math.sqrt((pred_rx - pred_ox)**2 + (pred_ry - pred_oy)**2)
                        if dist < self.robot_radius + obs.radius + self.safety_margin:
                            safe = False
                            break
                    if not safe:
                        break
                if safe:
                    best_w_candidate = w_try
                    break

            return adjusted_v, best_w_candidate, False

        return v, w, is_safe

    def plan_temporal_shift(self, path, obstacles, robot_x, robot_y):
        """v6.7: 路径时移

        如果路径上的某个点在未来某个时间会被动态障碍物占据，
        建议在路径上等待或选择替代路径段。

        Returns:
            (shifted_path, wait_points): 调整后的路径和需要等待的点
        """
        if not path or not obstacles:
            return path, []

        wait_points = []
        shifted_path = list(path)

        for i, (wx, wy) in enumerate(path):
            for obs in obstacles:
                # 预测障碍物到达此路径点的时间
                dx = wx - obs.x
                dy = wy - obs.y
                dist_to_point = math.sqrt(dx**2 + dy**2)
                obs_speed = math.sqrt(obs.vx**2 + obs.vy**2)

                if obs_speed > 0.01:
                    # 估计障碍物到达时间
                    time_to_reach = dist_to_point / obs_speed
                    if time_to_reach < self.time_horizon:
                        # 障碍物可能在此点附近
                        wait_points.append((wx, wy, time_to_reach))

        return shifted_path, wait_points
