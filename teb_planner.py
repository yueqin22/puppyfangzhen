#!/usr/bin/env python3
"""
Timed Elastic Band (TEB) Local Planner
=======================================
Trajectory optimization-based local planner, more sophisticated than DWA.

Based on Rösmann et al. "Trajectory modification considering dynamic
constraints of autonomous robots" (2012). Key ideas:

  1. Represent the trajectory as a sequence of poses (x, y, yaw) with
     time intervals dt — the "elastic band".
  2. Initialize the band from the global path (A* waypoints).
  3. Optimize the band by gradient descent on a cost function:
       - Obstacle cost: push poses away from obstacles (via costmap)
       - Smoothness cost: penalize sharp turns (second-order differences)
       - Path-following cost: stay near the original global path
       - Velocity cost: prefer higher forward speed
       - Goal cost: pull toward the goal
  4. Extract velocity command (v, w) from the first two poses of the
     optimized band.

Compared to DWA:
  - DWA samples velocity space and simulates forward — open-loop.
  - TEB optimizes the entire trajectory — closed-loop, smoother paths.
  - TEB naturally handles narrow passages (optimization pushes the band
    through the center) and dynamic obstacles (re-optimize each cycle).

Interface matches DWAPlanner for drop-in replacement:
    planner = TEBPlanner(costmap)
    v, w = planner.compute_velocity(rx, ry, ryaw, path, goal_x, goal_y)

Performance: ~5-15ms per call (5 gradient descent iterations × 20 poses).
"""
import math
import numpy as np
from costmap import Costmap, COST_INSCRIBED, COST_LETHAL


class TEBPlanner:
    """Timed Elastic Band local planner.

    Optimizes a trajectory band to minimize obstacle cost + smoothness +
    path-following + goal-reaching, subject to velocity/acceleration limits.
    """

    def __init__(self, costmap, profile_cfg=None):
        """Initialize TEB planner.

        Args:
            costmap: Costmap instance for obstacle cost queries
            profile_cfg: Optional dict from config/profile_*.yaml to override
                         defaults for deployment targets (e.g. Raspberry Pi).
                         Supported keys: n_iterations, n_poses, learning_rate,
                         max_v, dt. When None, uses desktop-tuned defaults.
        """
        self.costmap = costmap
        profile_cfg = profile_cfg or {}

        # Band configuration
        # v2.7b: n_poses/n_iterations overridable via PROFILE=raspberry_pi
        self.n_poses = profile_cfg.get('n_poses', 15)   # number of poses in the band
        self.dt = profile_cfg.get('dt', 0.1)             # time interval between consecutive poses (s)
        self.horizon = self.n_poses * self.dt  # total lookahead time (s)

        # Velocity limits
        self.max_v = profile_cfg.get('max_v', 0.3)       # max linear velocity (m/s)
        self.min_v = 0.0         # min linear velocity
        self.max_w = 1.2         # max angular velocity (rad/s)
        self.max_accel = 2.0     # max linear acceleration (m/s²)
        self.max_alpha = 4.0     # max angular acceleration (rad/s²)

        # Robot geometry
        self.robot_radius = 0.35

        # Cost function weights (tuned empirically for indoor navigation)
        # Tuned v2.3: w_path 0.4→0.6, w_goal 1.0→1.2 (tighter path following
        # reduces unnecessary detours, lowering total distance traveled)
        # v3.0: Added w_time (time-optimal) and w_jerk (jerk constraint) per
        # Rösmann et al. (2012) "Trajectory modification considering dynamic
        # constraints". w_time minimizes total trajectory time T, w_jerk
        # penalizes 3rd-order position differences for passenger comfort.
        self.w_obstacle = 3.0    # push away from obstacles
        self.w_smooth = 0.8      # penalize curvature (2nd-order, smooth path)
        self.w_path = 0.6        # stay near global path (was 0.4)
        self.w_velocity = 0.2    # prefer higher forward speed
        self.w_goal = 1.2        # pull toward goal (was 1.0)
        self.w_kinematic = 0.5   # enforce velocity/accel limits
        self.w_time = 0.3        # v3.0: time-optimal cost (Rösmann 2012 §IV-C)
        self.w_jerk = 0.05       # v3.0: jerk (3rd derivative) comfort cost
        # 注: 原 v3.0 设为 0.4，是为当时不完整的 jerk 梯度 (仅 1 项) 调的。
        # 修正为完整四项累加梯度后效果显著增强，0.4 会导致优化发散，
        # 故下调至 0.05 以维持与其它代价项的平衡。

        # Optimization parameters
        # v2.7b: n_iterations/learning_rate overridable via PROFILE=raspberry_pi
        self.n_iterations = profile_cfg.get('n_iterations', 4)    # gradient descent iterations
        self.learning_rate = profile_cfg.get('learning_rate', 0.08)  # step size
        self.obstacle_threshold = 80  # cost above which obstacle penalty applies

        # Multi-start 多初始化: 代价阈值，所有候选 band 代价超过此值则判定无解
        self.multistart_cost_threshold = 1e4

        # Goal tolerance
        self.goal_tolerance = 0.3  # meters

        # Previous velocity (for acceleration limiting)
        self.prev_v = 0.0
        self.prev_w = 0.0

    def compute_velocity(self, rx, ry, ryaw, path, goal_x, goal_y):
        """Compute best (v, w) by optimizing an elastic band trajectory.

        Args:
            rx, ry, ryaw: robot current pose
            path: list of (wx, wy) waypoints from global planner
            goal_x, goal_y: final goal position

        Returns:
            (v, w): linear and angular velocity command
        """
        # v2.6: Adaptive obstacle weight near doorway. The doorway is a 2m-wide
        # gap at y=0 between x=[-1,1]. Near the doorway, the inflation zone
        # makes w_obstacle=3.0 too aggressive, causing the band to oscillate
        # between the two walls. Reducing w_obstacle to 1.5 near the doorway
        # lets the band pass through the narrow corridor more smoothly.
        # Detection: |y|<1.0 and |x|<2.5 (within 1m of doorway center line)
        # No restore needed: each call re-sets these values.
        #
        # v2.7: w_obstacle 1.5→0.8 and detection range |y|<1.0→|y|<1.5.
        # v2.6b run showed 12 doorway crossings and 85m path because 1.5 was
        # still too aggressive — the band kept getting pushed side-to-side
        # in the narrow corridor. 0.8 lets the band pass through with minimal
        # lateral force. Wider detection range (1.5m) catches the approach
        # earlier so the band is already relaxed when entering the doorway.
        near_doorway = abs(ry) < 1.5 and abs(rx) < 2.5
        if near_doorway:
            self.w_obstacle = 0.8  # v2.7: 1.5→0.8 (1.5 still caused oscillation)
            self.w_goal = 1.5      # increased from 1.2 to pull through faster
        else:
            self.w_obstacle = 3.0
            self.w_goal = 1.2

        # Check goal reached
        dist_to_goal = math.sqrt((rx - goal_x) ** 2 + (ry - goal_y) ** 2)
        if dist_to_goal < self.goal_tolerance:
            self.prev_v = 0.0
            self.prev_w = 0.0
            return 0.0, 0.0

        # If no path, rotate toward goal
        # v2.9i: Kept threshold at 0.3 rad. Tried 0.2 but it INCREASED
        # static frames from 13.8% to 30.3% — the robot needs to rotate
        # to a more precise heading before moving, causing more spinning.
        # 0.3 rad (17°) is the sweet spot: rotate when significantly off,
        # drive forward with a slight curve otherwise.
        if not path or len(path) < 2:
            angle_to_goal = math.atan2(goal_y - ry, goal_x - rx)
            heading_err = self._angle_diff(angle_to_goal, ryaw)
            if abs(heading_err) > 0.3:
                w_cmd = max(-self.max_w, min(self.max_w, heading_err * 2.0))
                self.prev_v = 0.0
                self.prev_w = w_cmd
                return 0.0, w_cmd
            # Drive straight toward goal
            v_cmd = self.max_v * 0.5
            self.prev_v = v_cmd
            self.prev_w = 0.0
            return v_cmd, 0.0

        # Step 1: Initialize band from path
        band = self._init_band(rx, ry, ryaw, path)

        # Step 2: Check if current position is blocked (high cost)
        # v2.9i: Changed from pure rotation to creep-forward + rotation.
        # v2.3.1's pure rotation caused 87 frames of zero-velocity spinning
        # in high-cost zones (13.8% of all static frames in v2.9h). Instead,
        # rotate toward goal while creeping forward at 0.08 m/s — this lets
        # the robot exit high-cost zones via gradient descent in the costmap
        # instead of spinning in place waiting for RECOVER.
        current_cost = self.costmap.get_cost(rx, ry)
        if current_cost >= COST_INSCRIBED:
            # Robot is in a high-cost zone — rotate toward goal + creep forward
            angle_to_goal = math.atan2(goal_y - ry, goal_x - rx)
            heading_err = self._angle_diff(angle_to_goal, ryaw)
            w_cmd = max(-self.max_w, min(self.max_w, heading_err * 2.0))
            # v2.9i: creep forward to exit high-cost zone (was: v=0 pure rotation)
            v_cmd = 0.08
            self.prev_v = v_cmd
            self.prev_w = w_cmd
            return v_cmd, w_cmd

        # Step 3: Optimize band via gradient descent (multi-start 多初始化)
        band = self.optimize_multistart(band, path, goal_x, goal_y)
        if band is None:
            # 所有初始 band 代价都过高，无可行解 — 原地旋转寻找出路
            angle_to_goal = math.atan2(goal_y - ry, goal_x - rx)
            heading_err = self._angle_diff(angle_to_goal, ryaw)
            w_cmd = max(-self.max_w, min(self.max_w, heading_err * 2.0))
            self.prev_v = 0.0
            self.prev_w = w_cmd
            return 0.0, w_cmd

        # Step 4: Extract velocity from first two poses
        v, w = self._extract_velocity(band, ryaw)

        # Step 5: Apply kinematic constraints (velocity + acceleration)
        v, w = self._apply_limits(v, w)

        self.prev_v = v
        self.prev_w = w
        return v, w

    def _init_band(self, rx, ry, ryaw, path):
        """Initialize the elastic band by sampling points along the path.

        The band starts at the robot's current pose and follows the path
        for self.horizon meters (or until the path ends).
        """
        band = np.zeros((self.n_poses, 3))  # (x, y, yaw) per pose
        band[0] = [rx, ry, ryaw]

        # Walk along the path to fill the band
        path_idx = 0
        accumulated = 0.0
        segment_spacing = self.max_v * self.dt  # distance between poses

        for i in range(1, self.n_poses):
            # Find the point on the path at distance accumulated + spacing
            target_dist = accumulated + segment_spacing
            while path_idx < len(path) - 1:
                wx0, wy0 = path[path_idx]
                wx1, wy1 = path[path_idx + 1]
                seg_len = math.sqrt((wx1 - wx0) ** 2 + (wy1 - wy0) ** 2)
                if accumulated + seg_len >= target_dist or path_idx == len(path) - 2:
                    # Interpolate
                    remaining = target_dist - accumulated
                    if seg_len > 1e-6:
                        t = max(0.0, min(1.0, remaining / seg_len))
                    else:
                        t = 0.0
                    px = wx0 + t * (wx1 - wx0)
                    py = wy0 + t * (wy1 - wy0)
                    # Yaw from path direction
                    pyaw = math.atan2(wy1 - wy0, wx1 - wx0)
                    band[i] = [px, py, pyaw]
                    accumulated = target_dist
                    break
                accumulated += seg_len
                path_idx += 1
            else:
                # Path ended — extrapolate from last point
                wx, wy = path[-1]
                pyaw = band[i - 1, 2]
                band[i] = [wx, wy, pyaw]

        return band

    def _optimize_step(self, band, path, goal_x, goal_y):
        """One iteration of gradient descent on the elastic band.

        Computes the gradient of the total cost w.r.t. each pose's (x, y)
        and moves poses in the negative gradient direction. Yaw is updated
        based on the direction to the next pose.
        """
        n = len(band)
        gradient = np.zeros_like(band[:, :2])  # only x, y gradients

        for i in range(1, n - 1):  # skip first (robot) and last (goal)
            # --- Obstacle cost gradient ---
            gx, gy = band[i, 0], band[i, 1]
            cost = self.costmap.get_cost(gx, gy)
            if cost > self.obstacle_threshold:
                # Gradient: push away from obstacle
                # Use costmap gradient (finite difference)
                eps = 0.1
                cost_x_plus = self.costmap.get_cost(gx + eps, gy)
                cost_x_minus = self.costmap.get_cost(gx - eps, gy)
                cost_y_plus = self.costmap.get_cost(gx, gy + eps)
                cost_y_minus = self.costmap.get_cost(gx, gy - eps)
                # Gradient points toward higher cost → move opposite
                # v2.7b: cast to float to avoid uint8 subtraction underflow
                # (costmap returns uint8; 0 - 1 wraps to 255 → overflow warning)
                grad_obs_x = (float(cost_x_plus) - float(cost_x_minus)) / (2 * eps)
                grad_obs_y = (float(cost_y_plus) - float(cost_y_minus)) / (2 * eps)
                # Scale by cost (higher cost = stronger push)
                scale = (cost - self.obstacle_threshold) / 50.0
                gradient[i, 0] += self.w_obstacle * scale * grad_obs_x
                gradient[i, 1] += self.w_obstacle * scale * grad_obs_y

            # --- Smoothness cost gradient (second-order difference) ---
            # Penalize: band[i+1] - 2*band[i] + band[i-1]
            # Gradient w.r.t. band[i]: 2 * (2*band[i] - band[i-1] - band[i+1])
            prev_x, prev_y = band[i - 1, 0], band[i - 1, 1]
            next_x, next_y = band[i + 1, 0], band[i + 1, 1]
            gradient[i, 0] += self.w_smooth * 2.0 * (2 * gx - prev_x - next_x)
            gradient[i, 1] += self.w_smooth * 2.0 * (2 * gy - prev_y - next_y)

            # --- Path-following cost gradient ---
            # Pull toward nearest point on path
            nearest_x, nearest_y = self._nearest_path_point(gx, gy, path)
            gradient[i, 0] += self.w_path * (gx - nearest_x)
            gradient[i, 1] += self.w_path * (gy - nearest_y)

        # --- Velocity cost gradient: encourage spacing close to max_v * dt ---
        ideal_spacing = self.max_v * self.dt
        for i in range(1, n - 1):
            prev = band[i - 1, :2]
            curr = band[i, :2]
            nxt = band[i + 1, :2]
            d_prev = np.linalg.norm(curr - prev)
            d_next = np.linalg.norm(nxt - curr)
            if d_prev > 1e-6:
                gradient[i, :2] += self.w_velocity * (d_prev - ideal_spacing) * (curr - prev) / d_prev
            if d_next > 1e-6:
                gradient[i, :2] += self.w_velocity * (d_next - ideal_spacing) * (nxt - curr) / d_next

        # --- Kinematic cost gradient: penalize sharp heading changes ---
        for i in range(1, n - 2):
            seg1 = band[i, :2] - band[i - 1, :2]
            seg2 = band[i + 1, :2] - band[i, :2]
            n1 = np.linalg.norm(seg1)
            n2 = np.linalg.norm(seg2)
            if n1 > 1e-6 and n2 > 1e-6:
                cos_angle = np.dot(seg1, seg2) / (n1 * n2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                gradient[i, :2] += self.w_kinematic * (1.0 - cos_angle) * (seg2 - seg1) / (n1 + n2)

        # --- v3.0: Time-optimal cost gradient (Rösmann 2012 §IV-C) ---
        # Minimize total trajectory time by penalizing small inter-pose spacing
        # (small spacing = slow motion). Gradient pushes poses apart to
        # encourage faster traversal.
        # J_time = Σ (ideal_spacing - d_i)² for d_i < ideal_spacing
        for i in range(1, n - 1):
            prev = band[i - 1, :2]
            curr = band[i, :2]
            d_prev = np.linalg.norm(curr - prev)
            if d_prev > 1e-6 and d_prev < ideal_spacing:
                # Penalize being too close (too slow) — push apart
                diff = (curr - prev) / d_prev
                gradient[i, :2] -= self.w_time * (ideal_spacing - d_prev) * diff

        # --- v3.0: Jerk cost gradient (3rd-order difference, Rösmann 2012) ---
        # Jerk = 位置的三阶向后差分:
        #   J[i] = (x[i] - 3*x[i-1] + 3*x[i-2] - x[i-3]) / dt³
        # 代价 J_jerk = Σ ||J[i]||²，惩罚加速度的剧烈变化以提升舒适性。
        # 每个位置点 x[i] 出现在 4 个 Jerk 项中 (J[i], J[i+1], J[i+2], J[i+3])，
        # 其系数分别为 +1, -3, +3, -1，需累加所有贡献:
        #   ∂(Jerk²)/∂x[i] = 2*w_jerk * (
        #       J[i]*(+1) + J[i+1]*(-3) + J[i+2]*(+3) + J[i+3]*(-1))
        # 注: 数学上系数含 1/dt⁶，但本代码库所有代价项 (smooth/velocity/time)
        # 均不除 dt 幂次，权重本身吸收 dt 缩放，此处保持一致以避免数值发散。
        # 循环范围: x[i] 需要访问 x[i-3..i+3]，故 i ∈ [3, n-4]
        jerk_coeff = 2.0 * self.w_jerk
        for i in range(3, n - 3):
            # 计算 4 个相关的 Jerk 项 (对 x, y 分量分别计算，向量化处理 :2)
            j_i   = (band[i, :2]     - 3 * band[i - 1, :2] + 3 * band[i - 2, :2] - band[i - 3, :2])
            j_ip1 = (band[i + 1, :2] - 3 * band[i, :2]     + 3 * band[i - 1, :2] - band[i - 2, :2])
            j_ip2 = (band[i + 2, :2] - 3 * band[i + 1, :2] + 3 * band[i, :2]     - band[i - 1, :2])
            j_ip3 = (band[i + 3, :2] - 3 * band[i + 2, :2] + 3 * band[i + 1, :2] - band[i, :2])
            # 累加 x[i] 在各 Jerk 项中的梯度贡献 (系数 1, -3, +3, -1)
            jerk_grad = jerk_coeff * (
                j_i   * 1.0  +
                j_ip1 * (-3.0) +
                j_ip2 * 3.0  +
                j_ip3 * (-1.0)
            )
            gradient[i, :2] += jerk_grad

        # --- Goal cost: pull last pose toward goal ---
        gradient[-1, 0] += self.w_goal * (band[-1, 0] - goal_x) * 0.5
        gradient[-1, 1] += self.w_goal * (band[-1, 1] - goal_y) * 0.5

        # Apply gradient descent (move opposite to gradient)
        new_band = band.copy()
        new_band[1:-1, :2] -= self.learning_rate * gradient[1:-1]
        # Last pose moves toward goal
        new_band[-1, :2] -= self.learning_rate * gradient[-1]

        # Update yaws based on direction to next pose
        for i in range(n - 1):
            dx = new_band[i + 1, 0] - new_band[i, 0]
            dy = new_band[i + 1, 1] - new_band[i, 1]
            if dx * dx + dy * dy > 1e-6:
                new_band[i, 2] = math.atan2(dy, dx)
        # Last pose yaw = direction from second-to-last to last
        dx = new_band[-1, 0] - new_band[-2, 0]
        dy = new_band[-1, 1] - new_band[-2, 1]
        if dx * dx + dy * dy > 1e-6:
            new_band[-1, 2] = math.atan2(dy, dx)

        return new_band

    def _lateral_offset_band(self, band, offset):
        """对 band 应用横向偏移 (保留起始位姿)。

        偏移方向垂直于机器人当前航向，用于 multi-start 产生不同初始解。
        起始位姿 (机器人当前位置) 保持不变，仅偏移后续位姿。

        Args:
            band: 原始 band (n_poses x 3)
            offset: 横向偏移量 (m)，正值左偏，负值右偏

        Returns:
            偏移后的新 band
        """
        new_band = band.copy()
        yaw = band[0, 2]
        # 垂直于航向的单位向量 (左转方向为正)
        perp_x = -math.sin(yaw)
        perp_y = math.cos(yaw)
        # 除起始点外的所有位姿施加横向偏移
        new_band[1:, 0] += offset * perp_x
        new_band[1:, 1] += offset * perp_y
        return new_band

    def _compute_band_cost(self, band, path, goal_x, goal_y):
        """计算 band 的总代价 (用于 multi-start 选择最优解)。

        代价组成与 _optimize_step 中的梯度项保持一致: 障碍物、平滑度、
        路径跟随、速度/间距、时间最优、Jerk 和目标代价。
        """
        n = len(band)
        total = 0.0
        ideal_spacing = self.max_v * self.dt

        for i in range(1, n):
            gx, gy = band[i, 0], band[i, 1]
            # --- 障碍物代价 ---
            cost = self.costmap.get_cost(gx, gy)
            if cost > self.obstacle_threshold:
                total += self.w_obstacle * ((cost - self.obstacle_threshold) / 50.0) ** 2
            # --- 速度/间距代价 (鼓励接近 ideal_spacing) ---
            prev = band[i - 1, :2]
            curr = band[i, :2]
            d_prev = np.linalg.norm(curr - prev)
            total += self.w_velocity * (d_prev - ideal_spacing) ** 2
            # --- 时间最优代价 (间距过小时惩罚) ---
            if d_prev > 1e-6 and d_prev < ideal_spacing:
                total += self.w_time * (ideal_spacing - d_prev) ** 2
            # --- 路径跟随代价 (不含末位姿，末位姿由目标代价处理) ---
            if i < n - 1:
                nearest_x, nearest_y = self._nearest_path_point(gx, gy, path)
                total += self.w_path * ((gx - nearest_x) ** 2 + (gy - nearest_y) ** 2)

        # --- 平滑度代价 (二阶差分) ---
        for i in range(1, n - 1):
            diff = band[i + 1, :2] - 2 * band[i, :2] + band[i - 1, :2]
            total += self.w_smooth * np.dot(diff, diff)

        # --- Jerk 代价 (三阶向后差分，与梯度约定一致不除 dt⁶) ---
        for i in range(3, n):
            jerk = band[i, :2] - 3 * band[i - 1, :2] + 3 * band[i - 2, :2] - band[i - 3, :2]
            total += self.w_jerk * np.dot(jerk, jerk)

        # --- 目标代价 ---
        dx = band[-1, 0] - goal_x
        dy = band[-1, 1] - goal_y
        total += self.w_goal * (dx * dx + dy * dy) * 0.5

        return total

    def optimize_multistart(self, band, path, goal_x, goal_y):
        """Multi-start 多初始化优化策略。

        从不同初始 band 开始优化 (原始路径、左偏移 0.3m、右偏移 0.3m)，
        每个初始 band 独立优化 n_iterations 次梯度下降，选择代价最小的
        作为最终结果。若所有初始 band 代价都超过阈值，返回 None 表示无解。

        Args:
            band: 初始 band (由 _init_band 生成)
            path: 全局路径
            goal_x, goal_y: 目标位置

        Returns:
            优化后代价最小的 band，或 None (无解时)
        """
        # 3 个初始 band: 原始、左偏移 (+0.3m)、右偏移 (-0.3m)
        candidates = [
            band.copy(),
            self._lateral_offset_band(band, 0.3),
            self._lateral_offset_band(band, -0.3),
        ]

        best_band = None
        best_cost = float('inf')
        # 代价阈值: 所有候选代价都超过此值则认为无可行解
        cost_threshold = self.multistart_cost_threshold

        for init_band in candidates:
            # 每个初始 band 独立优化 n_iterations 次梯度下降
            opt_band = init_band
            for _ in range(self.n_iterations):
                opt_band = self._optimize_step(opt_band, path, goal_x, goal_y)
            # 计算优化后代价，保留最小者
            cost = self._compute_band_cost(opt_band, path, goal_x, goal_y)
            if cost < best_cost:
                best_cost = cost
                best_band = opt_band

        # 若所有候选代价过高，返回 None 表示无解
        if best_cost > cost_threshold:
            return None
        return best_band

    def _nearest_path_point(self, x, y, path):
        """Find the nearest point on the path to (x, y)."""
        min_dist = float('inf')
        nearest = (path[0][0], path[0][1])
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            seg_dx = bx - ax
            seg_dy = by - ay
            seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len2 < 1e-9:
                t = 0.0
            else:
                t = ((x - ax) * seg_dx + (y - ay) * seg_dy) / seg_len2
                t = max(0.0, min(1.0, t))
            px = ax + t * seg_dx
            py = ay + t * seg_dy
            d = (x - px) ** 2 + (y - py) ** 2
            if d < min_dist:
                min_dist = d
                nearest = (px, py)
        return nearest

    def _extract_velocity(self, band, ryaw):
        """Extract (v, w) from the first two poses of the optimized band."""
        x0, y0, yaw0 = band[0]
        x1, y1, yaw1 = band[1]

        # Displacement
        dx = x1 - x0
        dy = y1 - y0
        dist = math.sqrt(dx * dx + dy * dy)

        # Linear velocity
        v = dist / self.dt
        v = min(v, self.max_v)

        # Angular velocity: yaw difference / dt
        # Use the band's optimized yaw, but blend with direction to next point
        target_yaw = math.atan2(dy, dx) if dist > 1e-6 else yaw1
        yaw_diff = self._angle_diff(target_yaw, ryaw)
        w = yaw_diff / self.dt
        w = max(-self.max_w, min(self.max_w, w))

        # If heading error is large, decelerate (but keep moving) and turn
        if abs(yaw_diff) > 0.5:
            v = v * max(0.0, 1.0 - abs(yaw_diff) / 1.0)
            w = max(-self.max_w, min(self.max_w, yaw_diff * 2.0))

        return v, w

    def _apply_limits(self, v, w):
        """Apply velocity and acceleration limits."""
        # Clamp to max velocity
        v = max(self.min_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))

        # Acceleration limit
        dv = v - self.prev_v
        dw = w - self.prev_w
        max_dv = self.max_accel * self.dt
        max_dw = self.max_alpha * self.dt
        if abs(dv) > max_dv:
            v = self.prev_v + math.copysign(max_dv, dv)
        if abs(dw) > max_dw:
            w = self.prev_w + math.copysign(max_dw, dw)

        return v, w

    @staticmethod
    def _angle_diff(a, b):
        """Smallest angular difference a - b, normalized to [-pi, pi]."""
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def reset(self):
        """Reset velocity state (e.g., after teleport or collision)."""
        self.prev_v = 0.0
        self.prev_w = 0.0

    def velocity_to_step(self, v, w, ryaw, dt=0.1):
        """Convert velocity command to position step for CoppeliaSim.

        Same interface as DWAPlanner.velocity_to_step for compatibility.
        """
        mid_yaw = ryaw + w * dt * 0.5
        dx = v * math.cos(mid_yaw) * dt
        dy = v * math.sin(mid_yaw) * dt
        dyaw = w * dt
        return dx, dy, dyaw
