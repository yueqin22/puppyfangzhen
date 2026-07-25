"""
Model Predictive Control (MPC) Local Planner (v4.0)
=====================================================
Local planner that solves a finite-horizon optimal control problem
at each time step and executes only the first control input (receding
horizon).

The optimization finds the velocity sequence that minimizes:
  J = w_track * tracking_error + w_smooth * control_variation
    + w_obstacle * collision_cost + w_time * horizon_time

Subject to:
  - Kinematic constraints (bounded v, w, accel)
  - Obstacle avoidance constraints

Uses a simple gradient-based optimizer (no external QP solver needed).

References:
  - Kuhne et al. (2004) "Model Predictive Control of a Mobile Robot
    Using Linear Parameter Varying Models"
  - Li & Shi (2012) "Model Predictive Control for Mobile Robot Path
    Tracking"
  - Rawlings & Mayne (2009) "Model Predictive Control: Theory and Design"
"""
import os
import math
import numpy as np
from costmap import Costmap, COST_LETHAL

USE_MPC = os.environ.get("USE_MPC", "0") == "1"


class MPCPlanner:
    """MPC-based local planner for differential-drive robots.

    Solves a finite-horizon optimization problem at each step.
    """

    def __init__(self, costmap, cfg=None, horizon_steps=10, dt=0.1,
                 max_linear_vel=0.3, max_angular_vel=1.0,
                 max_linear_acc=0.5, max_angular_acc=1.5,
                 mpc_iterations=20, robot_radius=0.25,
                 w_track=1.0, w_obstacle=2.0, w_speed=0.1):
        self.costmap = costmap
        cfg = cfg or {}

        # Velocity limits
        self.max_v = cfg.get('max_linear_vel', max_linear_vel)
        self.max_w = cfg.get('max_angular_vel', max_angular_vel)
        self.max_acc_v = cfg.get('max_linear_acc', max_linear_acc)
        self.max_acc_w = cfg.get('max_angular_acc', max_angular_acc)

        # Horizon
        self.horizon = cfg.get('horizon_steps', horizon_steps)
        self.dt = cfg.get('dt', dt)

        # Weights
        self.w_track = cfg.get('w_track', w_track)
        self.w_smooth_v = cfg.get('w_smooth_v', 0.3)
        self.w_smooth_w = cfg.get('w_smooth_w', 0.2)
        self.w_obstacle = cfg.get('w_obstacle', w_obstacle)
        self.w_speed = cfg.get('w_speed', w_speed)

        # Obstacle cost parameters
        self.obstacle_dist_thresh = cfg.get('obstacle_dist_thresh', 0.5)
        self.robot_radius = cfg.get('robot_radius', robot_radius)

        # Optimization
        self.n_iterations = cfg.get('mpc_iterations', mpc_iterations)
        self.step_size = cfg.get('step_size', 0.01)

        # Previous control (for warm start)
        self.prev_v = 0.0
        self.prev_w = 0.0

    def compute_velocity(self, rx, ry, ryaw, path, goal_x, goal_y):
        """Compute optimal velocity command using MPC.

        Args:
            rx, ry, ryaw: current robot pose
            path: list of (x, y) waypoints (global path)
            goal_x, goal_y: final goal position

        Returns:
            (v, w) velocity command
        """
        if not path:
            return 0.0, 0.0

        # Get reference path points along the horizon
        ref = self._extract_reference(rx, ry, path)

        # Initial guess: straight line toward goal
        angle_to_goal = math.atan2(goal_y - ry, goal_x - rx)
        heading_err = angle_to_goal - ryaw
        while heading_err > math.pi:
            heading_err -= 2 * math.pi
        while heading_err < -math.pi:
            heading_err += 2 * math.pi

        v_init = min(self.max_v, 0.2) * max(0, 1 - abs(heading_err) / math.pi)
        w_init = max(-self.max_w, min(self.max_w, heading_err * 2.0))

        # Control sequence: [v_0, w_0, v_1, w_1, ..., v_N-1, w_N-1]
        u = np.zeros(2 * self.horizon)
        u[0::2] = v_init
        u[1::2] = w_init

        # Gradient descent optimization
        best_u = u.copy()
        best_cost = self._compute_cost(u, rx, ry, ryaw, ref)

        for iteration in range(self.n_iterations):
            # Compute gradient numerically
            grad = np.zeros_like(u)
            eps = 1e-4
            for i in range(len(u)):
                u_plus = u.copy()
                u_plus[i] += eps
                u_minus = u.copy()
                u_minus[i] -= eps
                cost_plus = self._compute_cost(u_plus, rx, ry, ryaw, ref)
                cost_minus = self._compute_cost(u_minus, rx, ry, ryaw, ref)
                grad[i] = (cost_plus - cost_minus) / (2 * eps)

            # Gradient descent with line search (simple)
            for alpha in [self.step_size * 2, self.step_size, self.step_size / 2, self.step_size / 4]:
                u_new = u - alpha * grad
                # Project to feasible set
                u_new = self._project_feasible(u_new)
                cost_new = self._compute_cost(u_new, rx, ry, ryaw, ref)
                if cost_new < best_cost:
                    best_cost = cost_new
                    best_u = u_new.copy()
                    u = u_new
                    break

        # Return first control input
        v = best_u[0]
        w = best_u[1]

        # Store for next iteration (warm start)
        self.prev_v = v
        self.prev_w = w

        return v, w

    def _extract_reference(self, rx, ry, path):
        """Extract reference path points along the horizon."""
        # Find nearest point on path
        min_dist = float('inf')
        nearest_idx = 0
        for i, (px, py) in enumerate(path):
            d = math.sqrt((px - rx)**2 + (py - ry)**2)
            if d < min_dist:
                min_dist = d
                nearest_idx = i

        # Sample points along the path at roughly step_size * v * dt intervals
        ref = []
        accumulated = 0.0
        step_dist = 0.2  # ~ 2m/s * 0.1s = 0.2m per step

        current_idx = nearest_idx
        ref.append(path[min(current_idx, len(path)-1)])

        for _ in range(self.horizon):
            if current_idx >= len(path) - 1:
                ref.append(path[-1])
                continue

            # Move along path
            while current_idx < len(path) - 1 and accumulated < step_dist:
                dx = path[current_idx + 1][0] - path[current_idx][0]
                dy = path[current_idx + 1][1] - path[current_idx][1]
                seg_len = math.sqrt(dx**2 + dy**2)
                if accumulated + seg_len <= step_dist:
                    accumulated += seg_len
                    current_idx += 1
                else:
                    break
            if current_idx >= len(path) - 1:
                ref.append(path[-1])
            else:
                # Interpolate
                remaining = step_dist - accumulated
                dx = path[current_idx + 1][0] - path[current_idx][0]
                dy = path[current_idx + 1][1] - path[current_idx][1]
                seg_len = math.sqrt(dx**2 + dy**2)
                if seg_len > 1e-6:
                    ratio = remaining / seg_len
                    rx_p = path[current_idx][0] + ratio * dx
                    ry_p = path[current_idx][1] + ratio * dy
                    ref.append((rx_p, ry_p))
                    accumulated = remaining
                else:
                    ref.append(path[current_idx])

        return ref

    def _simulate_trajectory(self, u, rx, ry, ryaw):
        """Simulate robot trajectory given control sequence u.

        Returns list of (x, y) positions.
        """
        xs = [rx]
        ys = [ry]
        yaws = [ryaw]

        x, y, t = rx, ry, ryaw
        for i in range(self.horizon):
            v = u[2*i]
            w = u[2*i + 1]

            # Simple motion model
            if abs(w) > 1e-6:
                R = v / w
                x += -R * math.sin(t) + R * math.sin(t + w * self.dt)
                y += R * math.cos(t) - R * math.cos(t + w * self.dt)
                t += w * self.dt
            else:
                x += v * self.dt * math.cos(t)
                y += v * self.dt * math.sin(t)

            xs.append(x)
            ys.append(y)
            yaws.append(t)

        return xs, ys, yaws

    def _compute_cost(self, u, rx, ry, ryaw, ref):
        """Compute total cost for a control sequence."""
        xs, ys, yaws = self._simulate_trajectory(u, rx, ry, ryaw)

        cost = 0.0

        # 1. Tracking cost (distance to reference path)
        for i in range(self.horizon + 1):
            ref_x, ref_y = ref[min(i, len(ref) - 1)]
            dx = xs[i] - ref_x
            dy = ys[i] - ref_y
            cost += self.w_track * (dx**2 + dy**2)

        # 2. Control smoothness cost (penalize changes in control)
        for i in range(1, self.horizon):
            dv = u[2*i] - u[2*(i-1)]
            dw = u[2*i+1] - u[2*(i-1)+1]
            cost += self.w_smooth_v * dv**2 + self.w_smooth_w * dw**2

        # 3. Obstacle cost
        for i in range(1, self.horizon + 1):
            cost += self.w_obstacle * self._obstacle_cost(xs[i], ys[i])

        # 4. Speed reward (negative cost = incentive to move fast)
        for i in range(self.horizon):
            cost -= self.w_speed * u[2*i]  # higher v = lower cost

        return max(0.0, cost)

    def _obstacle_cost(self, x, y):
        """Compute obstacle cost at position (x, y).

        High cost near obstacles, zero in free space far from obstacles.
        """
        cost_val = self.costmap.get_cost(x, y)
        if cost_val >= COST_LETHAL:
            return 100.0  # in collision
        # Normalize by threshold distance
        dist = self._distance_to_obstacle(x, y)
        if dist >= self.obstacle_dist_thresh:
            return 0.0
        # Quadratic penalty near obstacles
        ratio = 1.0 - dist / self.obstacle_dist_thresh
        return ratio * ratio * 10.0

    def _distance_to_obstacle(self, x, y):
        """Approximate distance to nearest obstacle (grid search)."""
        gx, gy = self.costmap.world_to_grid(x, y)
        max_r = int(self.obstacle_dist_thresh / self.costmap.resolution) + 2

        min_dist = float('inf')
        for dy in range(-max_r, max_r + 1):
            for dx in range(-max_r, max_r + 1):
                cx, cy = gx + dx, gy + dy
                if not (0 <= cx < self.costmap.width and 0 <= cy < self.costmap.height):
                    continue
                if self.costmap.cost[cy, cx] >= COST_LETHAL:
                    d = math.sqrt(dx**2 + dy**2) * self.costmap.resolution
                    if d < min_dist:
                        min_dist = d

        return min_dist if min_dist != float('inf') else self.obstacle_dist_thresh * 2

    def _project_feasible(self, u):
        """Project control sequence to feasible set (velocity + accel limits)."""
        u_new = u.copy()
        for i in range(self.horizon):
            # Velocity limits
            u_new[2*i] = max(-self.max_v, min(self.max_v, u_new[2*i]))
            u_new[2*i+1] = max(-self.max_w, min(self.max_w, u_new[2*i+1]))

        # Acceleration limits (from previous control)
        prev_v, prev_w = self.prev_v, self.prev_w
        for i in range(self.horizon):
            max_dv = self.max_acc_v * self.dt
            max_dw = self.max_acc_w * self.dt
            u_new[2*i] = max(prev_v - max_dv, min(prev_v + max_dv, u_new[2*i]))
            u_new[2*i+1] = max(prev_w - max_dw, min(prev_w + max_dw, u_new[2*i+1]))
            prev_v = u_new[2*i]
            prev_w = u_new[2*i+1]

        return u_new

    def reset(self):
        """Reset planner state."""
        self.prev_v = 0.0
        self.prev_w = 0.0
