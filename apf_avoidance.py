"""APF: Artificial Potential Field 人工势场法

最经典的反应式避障算法 (Khatib 1986)。

核心思想：
    机器人在一个"势场"中运动：
    - 目标点产生吸引力（pull）
    - 障碍物产生排斥力（push）
    - 合力方向就是机器人的运动方向

数学表达：
    吸引力:
        U_att(q) = 0.5 * k_att * |q - q_goal|^2    (距离 <= d_threshold)
        F_att = -∇U_att = k_att * (q_goal - q)

    排斥力:
        U_rep(q) = 0.5 * k_rep * (1/d - 1/d0)^2    (d <= d0)
        U_rep(q) = 0                                (d > d0)
        F_rep = -∇U_rep = k_rep * (1/d - 1/d0) * (1/d^2) * (q - q_obs)/d

    合力:
        F_total = F_att + Σ F_rep
        v = F_total  (归一化到 max_speed)

APF优缺点：
    优点:
    - 实现极其简单，O(n)复杂度
    - 直觉清晰，容易理解
    - 平滑的轨迹
    缺点:
    - 局部极小值问题（两个对称障碍物之间会卡住）
    - 目标不可达（障碍物靠近目标时排斥力大于吸引力）
    - 窄通道振荡（通道两侧都有排斥力，机器人左右摇摆）
    - 无法考虑动态障碍物的速度

改进点（本实现包含）：
    1. 目标不可达修复: 当靠近目标时，排斥力逐渐衰减
    2. 局部极小值逃逸: 检测到停滞时，增加随机扰动
    3. 动态障碍物速度感知: 考虑行人相对速度，排斥力有方向性

参考:
    Oussama Khatib. "Real-time obstacle avoidance for manipulators
    and mobile robots." IJCAI 1985 / IJRR 1986.
"""
import math
import random
from dataclasses import dataclass
from typing import List, Tuple

from stvoc_avoidance import AvoidanceCommand


class APFController:
    """APF 人工势场法控制器

    用法:
        apf = APFController(dt=1/30, max_speed=6.0,
                            k_att=1.0, k_rep=0.8,
                            repel_threshold=1.5,
                            robot_radius=0.25, obstacle_radius=0.30)
        cmd = apf.compute_avoidance(rx, ry, yaw, vx, vy, tx, ty, obstacles)
    """

    def __init__(self,
                 dt: float = 1/30,
                 max_speed: float = 6.0,
                 k_att: float = 1.0,
                 k_rep: float = 0.8,
                 repel_threshold: float = 1.5,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30,
                 safe_distance: float = 0.55):
        """初始化APF控制器

        Args:
            dt: 控制周期（秒）
            max_speed: 最大线速度（m/s）
            k_att: 吸引力增益
            k_rep: 排斥力增益
            repel_threshold: 排斥力作用距离（米），超出此距离无排斥
            robot_radius: 机器人半径
            obstacle_radius: 行人半径
            safe_distance: 近距保护触发距离
        """
        self.dt = dt
        self.max_speed = max_speed
        self.k_att = k_att
        self.k_rep = k_rep
        self.repel_threshold = repel_threshold
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.safe_distance = safe_distance
        self.combined_radius = robot_radius + obstacle_radius

        # 局部极小值逃逸
        self._stagnation_count = 0
        self._last_positions = []
        self._random_perturbation = (0.0, 0.0)
        self._perturbation_timer = 0

        # 统计
        self.stats = {
            'apf_computes': 0,
            'att_force_mag': 0.0,
            'rep_force_mag': 0.0,
            'near_miss_count': 0,
            'stagnation_escape': 0,
        }

    def compute_avoidance(self,
                          robot_x: float, robot_y: float, robot_yaw: float,
                          robot_vx: float, robot_vy: float,
                          target_x: float, target_y: float,
                          obstacles: List[Tuple]) -> AvoidanceCommand:
        """计算APF避障指令

        Args:
            robot_x, robot_y: 机器人位置
            robot_yaw: 机器人朝向（弧度）
            robot_vx, robot_vy: 机器人当前速度（m/s）
            target_x, target_y: 目标位置
            obstacles: 行人列表 [(name, x, y, vx, vy, pattern), ...]

        Returns:
            AvoidanceCommand
        """
        self.stats['apf_computes'] += 1

        # === 步骤1: 计算吸引力 ===
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist_to_target = math.sqrt(dx * dx + dy * dy)

        if dist_to_target > 1e-6:
            att_dir_x = dx / dist_to_target
            att_dir_y = dy / dist_to_target
        else:
            att_dir_x = 1.0
            att_dir_y = 0.0

        # 吸引力大小：距离远时线性增长，近时衰减（避免冲过目标）
        if dist_to_target > 1.0:
            att_mag = self.k_att * self.max_speed
        else:
            att_mag = self.k_att * self.max_speed * dist_to_target

        f_att_x = att_dir_x * att_mag
        f_att_y = att_dir_y * att_mag

        # === 步骤2: 计算排斥力 ===
        f_rep_x = 0.0
        f_rep_y = 0.0
        min_obs_dist = float('inf')
        nearest_obs = None

        for name, ox, oy, ovx, ovy, _pattern in obstacles:
            dx_obs = robot_x - ox  # 从行人指向机器人
            dy_obs = robot_y - oy
            dist = math.sqrt(dx_obs * dx_obs + dy_obs * dy_obs)

            if dist < min_obs_dist:
                min_obs_dist = dist
                nearest_obs = name

            if dist > self.repel_threshold or dist < 1e-6:
                continue

            # 有效距离：减去碰撞半径
            effective_dist = dist - self.combined_radius
            if effective_dist <= 0:
                # 已碰撞，最大排斥
                rep_mag = self.k_rep * self.max_speed * 2.0
            else:
                # 排斥力: 随距离减小而指数增长（比标准APF的1/d^2更稳定）
                # 用指数衰减代替1/d^2，避免近距无穷大
                ratio = 1.0 - (effective_dist / self.repel_threshold)
                rep_mag = self.k_rep * self.max_speed * (ratio ** 2)

            # 排斥方向：从行人指向机器人（远离行人）
            rep_dir_x = dx_obs / dist
            rep_dir_y = dy_obs / dist

            # 速度感知修正：行人靠近的方向排斥力更大
            # 行人相对机器人的速度
            rel_vx = ovx - robot_vx
            rel_vy = ovy - robot_vy
            # 相对速度在"行人→机器人"方向上的投影
            vel_along = rel_vx * rep_dir_x + rel_vy * rep_dir_y
            if vel_along > 0:  # 行人正在靠近
                speed_factor = 1.0 + vel_along / self.max_speed
                rep_mag *= min(2.0, speed_factor)

            f_rep_x += rep_dir_x * rep_mag
            f_rep_y += rep_dir_y * rep_mag

        # === 步骤3: 局部极小值检测与逃逸 ===
        self._last_positions.append((robot_x, robot_y))
        if len(self._last_positions) > 30:
            self._last_positions.pop(0)

        if len(self._last_positions) >= 30:
            # 检查过去30帧是否在小范围内徘徊
            xs = [p[0] for p in self._last_positions]
            ys = [p[1] for p in self._last_positions]
            spread = (max(xs) - min(xs)) + (max(ys) - min(ys))
            if spread < 0.15 and len(obstacles) > 0:
                self._stagnation_count += 1
                if self._stagnation_count > 20 and self._perturbation_timer <= 0:
                    # 触发随机扰动逃逸
                    angle = random.uniform(-math.pi, math.pi)
                    self._random_perturbation = (
                        math.cos(angle) * self.max_speed * 0.3,
                        math.sin(angle) * self.max_speed * 0.3
                    )
                    self._perturbation_timer = 30  # 持续30帧
                    self.stats['stagnation_escape'] += 1
                    self._stagnation_count = 0
            else:
                self._stagnation_count = max(0, self._stagnation_count - 1)

        if self._perturbation_timer > 0:
            self._perturbation_timer -= 1
            f_rep_x += self._random_perturbation[0]
            f_rep_y += self._random_perturbation[1]
        else:
            self._random_perturbation = (0.0, 0.0)

        # === 步骤4: 合力与速度 ===
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        # 速度限幅
        speed = math.sqrt(f_total_x ** 2 + f_total_y ** 2)
        if speed > self.max_speed:
            f_total_x = f_total_x * (self.max_speed / speed)
            f_total_y = f_total_y * (self.max_speed / speed)
            speed = self.max_speed

        # 速度平滑（与上次速度插值）
        if hasattr(self, '_last_vx'):
            alpha = 0.7
            new_vx = alpha * f_total_x + (1 - alpha) * self._last_vx
            new_vy = alpha * f_total_y + (1 - alpha) * self._last_vy
            s = math.sqrt(new_vx ** 2 + new_vy ** 2)
            if s > self.max_speed:
                new_vx = new_vx * (self.max_speed / s)
                new_vy = new_vy * (self.max_speed / s)
        else:
            new_vx = f_total_x
            new_vy = f_total_y

        self._last_vx = new_vx
        self._last_vy = new_vy

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
            self.stats['near_miss_count'] += 1
        else:
            action = 'cruise'
            reason = 'clear'
            confidence = 1.0

        conflict_detected = min_obs_dist < self.repel_threshold

        # 计算TTC（粗略估计）
        if min_obs_dist < self.repel_threshold and min_obs_dist > self.combined_radius:
            # 近似TTC: 用接近速度估算
            ttc = 999.0
            for name, ox, oy, ovx, ovy, _pattern in obstacles:
                dx_o = robot_x - ox
                dy_o = robot_y - oy
                d = math.sqrt(dx_o * dx_o + dy_o * dy_o)
                if d > self.repel_threshold:
                    continue
                # 相对速度沿距离方向
                rel_v = (ovx - robot_vx) * (dx_o / d) + (ovy - robot_vy) * (dy_o / d)
                if rel_v > 0:  # 行人靠近
                    t = (d - self.combined_radius) / rel_v
                    if t < ttc:
                        ttc = t
        else:
            ttc = 999.0

        # 角速度
        if speed > 1e-6:
            vel_angle = math.atan2(new_vy, new_vx)
            ang_diff = vel_angle - robot_yaw
            while ang_diff > math.pi:
                ang_diff -= 2 * math.pi
            while ang_diff < -math.pi:
                ang_diff += 2 * math.pi
            wz = max(-2.0, min(2.0, ang_diff / self.dt))
        else:
            wz = 0.0

        return AvoidanceCommand(
            vx=float(new_vx),
            vy=float(new_vy),
            wz=wz,
            action=action,
            reason=reason,
            confidence=confidence,
            conflict_detected=conflict_detected,
            time_to_conflict=ttc,
        )

    def reset(self):
        """重置状态"""
        self._stagnation_count = 0
        self._last_positions = []
        self._random_perturbation = (0.0, 0.0)
        self._perturbation_timer = 0
        if hasattr(self, '_last_vx'):
            del self._last_vx
        if hasattr(self, '_last_vy'):
            del self._last_vy
        self.stats = {k: 0 for k in self.stats}
