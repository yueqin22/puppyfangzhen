#!/usr/bin/env python3
"""
Dynamic Window Approach (DWA) Local Planner
============================================
Samples velocity space (v, ω) and simulates trajectories, scoring by:
  - Heading: alignment with path direction toward goal
  - Clearance: minimum obstacle distance along trajectory
  - Velocity: prefer higher forward speed

Based on Fox, Burgard, Thrun (1997) "The Dynamic Window Approach to
Collision Avoidance". Adapted for CoppeliaSim position-control interface.

Output: (linear_vel, angular_vel) → converted to position step for sim.
"""
import numpy as np
import math
from costmap import Costmap, COST_INSCRIBED


class DWAPlanner:
    """DWA local planner for real-time obstacle avoidance."""

    def __init__(self, costmap):
        self.costmap = costmap

        # Velocity limits
        self.max_v = 0.3       # max linear velocity (m/s)
        self.min_v = 0.0       # min linear velocity
        self.max_w = 1.2       # max angular velocity (rad/s)
        self.max_accel = 2.0   # max linear acceleration (m/s^2) — high so v can ramp fast
        self.max_alpha = 4.0   # max angular acceleration (rad/s^2)

        # Velocity sampling resolution
        self.v_samples = 9     # linear velocity samples
        self.w_samples = 15    # angular velocity samples

        # Trajectory simulation
        self.horizon = 2.0     # seconds to look ahead (longer = sees dead ends)
        self.dt = 0.1          # time step for simulation
        self.steps = int(self.horizon / self.dt)

        # Scoring weights (base values, sum = 1.0)
        # Reweighted to emphasize path-following in narrow passages:
        #   - w_path_align (NEW): trajectory direction vs A* path direction.
        #     This is the KEY fix for doorway oscillation: even if a trajectory
        #     drifts off the path, if it moves ALONG the path direction it gets
        #     a high score. This prevents DWA from preferring sideways drift
        #     toward open space when the path goes through a doorway.
        #   - w_path: cross-track error (stays on path geometrically)
        #   - w_clearance reduced: inflation costs are caution, not collision
        self.w_heading = 0.20
        self.w_clearance = 0.10
        self.w_velocity = 0.10
        self.w_goal = 0.25
        self.w_path = 0.15       # cross-track error to A* path
        self.w_path_align = 0.20  # trajectory direction alignment with path

        # Narrow passage adaptive weights (gaijin1.md section 6.5)
        # When robot is in a high-cost area (doorway, gap), further reduce
        # clearance preference and boost path-following to prevent oscillation.
        self.narrow_passage_threshold = 50  # cost above which = narrow passage
        self.narrow_w_clearance = 0.05
        self.narrow_w_path = 0.20
        self.narrow_w_path_align = 0.25

        # Robot radius for collision checking
        self.robot_radius = 0.35

        # Previous velocity (for dynamic window)
        self.prev_v = 0.0
        self.prev_w = 0.0

        # Goal reaching tolerance
        self.goal_tolerance = 0.3  # meters

    def compute_velocity(self, rx, ry, ryaw, path, goal_x, goal_y):
        """Compute best (v, w) to follow path while avoiding obstacles.

        Two-phase approach:
          1. ALIGN: if heading error > threshold, rotate in place toward goal
          2. DRIVE: sample (v, w) space, pick best-scoring trajectory

        Args:
            rx, ry, ryaw: robot position and heading
            path: list of (wx, wy) waypoints from global planner
            goal_x, goal_y: final goal position

        Returns:
            (v, w): linear and angular velocity commands
        """
        # Check if goal reached
        dist_to_goal = math.sqrt((rx - goal_x) ** 2 + (ry - goal_y) ** 2)
        if dist_to_goal < self.goal_tolerance:
            self.prev_v = 0.0
            self.prev_w = 0.0
            return 0.0, 0.0

        # Local goal from path
        local_goal = self._get_local_goal(rx, ry, path)
        target_x = local_goal[0] if local_goal else goal_x
        target_y = local_goal[1] if local_goal else goal_y

        # Heading error to target
        angle_to_target = math.atan2(target_y - ry, target_x - rx)
        heading_err = angle_to_target - ryaw
        while heading_err > math.pi:
            heading_err -= 2 * math.pi
        while heading_err < -math.pi:
            heading_err += 2 * math.pi

        # Phase 1: ALIGN — rotate in place if heading error is large.
        # Threshold 0.5 rad (~28°) prevents ALIGN/DRIVE oscillation when
        # heading_err hovers near the boundary due to shifting local_goal.
        if abs(heading_err) > 0.5:
            # Proportional rotation toward target, capped at max_w
            w_cmd = max(-self.max_w, min(self.max_w, heading_err * 2.0))
            # Ensure minimum rotation speed to overcome static friction
            if abs(w_cmd) < 0.3:
                w_cmd = 0.3 if heading_err > 0 else -0.3
            self.prev_v = 0.0
            self.prev_w = w_cmd
            return 0.0, w_cmd

        # Phase 2: DRIVE — sample velocity space for forward motion
        # Sample v across full range (not restricted by dynamic window,
        # to avoid getting stuck when prev_v=0)
        v_range = np.linspace(self.max_v * 0.3, self.max_v, self.v_samples)
        # Sample w in a narrower band around 0 (we're roughly aligned)
        w_band = min(self.max_w, 0.8)
        w_range = np.linspace(-w_band, w_band, self.w_samples)

        best_score = -float('inf')
        best_v = 0.0
        best_w = 0.0

        for v in v_range:
            for w in w_range:
                traj = self._simulate_trajectory(rx, ry, ryaw, v, w)
                if traj is None:
                    continue  # collision

                score = self._score_trajectory(
                    traj, rx, ry, v, w, local_goal, goal_x, goal_y, path)

                # Penalize large angular velocity (prefer straighter paths)
                score -= 0.1 * abs(w) / self.max_w

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w

        # If no valid forward trajectory (all blocked), rotate to find opening
        if best_score < -0.5:
            w_cmd = 0.5 if heading_err >= 0 else -0.5
            self.prev_v = 0.0
            self.prev_w = w_cmd
            return 0.0, w_cmd

        self.prev_v = best_v
        self.prev_w = best_w
        return best_v, best_w

    def _get_local_goal(self, rx, ry, path):
        """Get the local goal point by sequential waypoint tracking.

        Walks the path in order and returns the first waypoint the robot
        hasn't reached yet (distance > reach_threshold). Then advances a
        short lookahead along the path for smoother motion. This ensures
        the DWA follows path order (e.g. goes through a doorway waypoint
        before heading to the final goal behind a wall) instead of cutting
        straight to whichever waypoint is geometrically nearest.
        """
        if not path:
            return None

        reach_threshold = 0.3  # waypoint considered reached if closer than this

        # Step 1: find the first unreached waypoint (sequential tracking)
        start_idx = 0
        for i, (wx, wy) in enumerate(path):
            d = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
            if d > reach_threshold:
                start_idx = i
                break
        else:
            # All waypoints reached
            return path[-1]

        # Step 2: walk a short lookahead from start_idx for smoother motion
        lookahead_dist = 0.5
        accumulated = 0.0
        for i in range(start_idx, len(path) - 1):
            wx0, wy0 = path[i]
            wx1, wy1 = path[i + 1]
            seg_len = math.sqrt((wx1 - wx0) ** 2 + (wy1 - wy0) ** 2)
            if accumulated + seg_len >= lookahead_dist:
                remaining = lookahead_dist - accumulated
                if seg_len > 1e-6:
                    t = remaining / seg_len
                else:
                    t = 0.0
                lx = wx0 + t * (wx1 - wx0)
                ly = wy0 + t * (wy1 - wy0)
                return (lx, ly)
            accumulated += seg_len

        # Lookahead exceeds remaining path — return last waypoint
        return path[-1]

    def _simulate_trajectory(self, rx, ry, ryaw, v, w):
        """Simulate trajectory for given (v, w) over horizon.

        Returns list of (x, y, yaw) points, or None if collision.
        """
        traj = []
        x, y, yaw = rx, ry, ryaw
        for _ in range(self.steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            # Normalize yaw
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi

            # Check collision (use robot radius)
            if self._check_collision(x, y):
                return None
            traj.append((x, y, yaw))

        return traj

    def _check_collision(self, x, y):
        """Check if position (x, y) collides with obstacle.

        Uses strict > (not >=) so the robot can still generate trajectories
        when sitting exactly at COST_INSCRIBED (e.g. at the edge of an
        inflation zone near a doorway). With >=, the robot gets stuck if
        RECOVER or a previous move placed it at cost==COST_INSCRIBED because
        every simulated trajectory's first step is rejected. Using > lets
        the robot move OUT of high-cost zones while still blocking actual
        lethal obstacles (cost > 128).
        """
        cost = self.costmap.get_cost(x, y)
        return cost > COST_INSCRIBED

    def _score_trajectory(self, traj, rx, ry, v, w, local_goal, goal_x, goal_y, path):
        """Score a trajectory: heading + clearance + velocity + goal + path + path_align."""
        if not traj:
            return -float('inf')

        last_x, last_y, last_yaw = traj[-1]

        # --- Narrow passage detection: adapt weights based on local cost ---
        # When robot is in a high-cost area (doorway, narrow gap), reduce
        # clearance preference and boost path-following to prevent the DWA
        # from oscillating between open space and the passage entrance.
        robot_cost = self.costmap.get_cost(rx, ry)
        in_narrow = robot_cost >= self.narrow_passage_threshold
        w_clearance = self.narrow_w_clearance if in_narrow else self.w_clearance
        w_path = self.narrow_w_path if in_narrow else self.w_path
        w_path_align = self.narrow_w_path_align if in_narrow else self.w_path_align

        # --- Heading score: alignment of trajectory displacement with goal dir ---
        dx = last_x - rx
        dy = last_y - ry
        move_dist = math.sqrt(dx * dx + dy * dy)
        if local_goal:
            angle_to_goal = math.atan2(
                local_goal[1] - ry, local_goal[0] - rx)
            if move_dist > 1e-4:
                move_angle = math.atan2(dy, dx)
                heading_diff = abs(angle_to_goal - move_angle)
                while heading_diff > math.pi:
                    heading_diff = 2 * math.pi - heading_diff
                heading_score = 1.0 - heading_diff / math.pi
            else:
                angle_to_goal_from_yaw = math.atan2(
                    local_goal[1] - ry, local_goal[0] - rx)
                yaw_diff = abs(angle_to_goal_from_yaw - last_yaw)
                while yaw_diff > math.pi:
                    yaw_diff = 2 * math.pi - yaw_diff
                heading_score = 0.5 * (1.0 - yaw_diff / math.pi)
        else:
            heading_score = 0.0

        # --- Clearance score ---
        min_cost = COST_INSCRIBED
        for px, py, _ in traj:
            c = self.costmap.get_cost(px, py)
            if c < min_cost:
                min_cost = c
        if min_cost < 50:
            clearance_score = 1.0
        else:
            clearance_score = 1.0 - (min_cost - 50) / (COST_INSCRIBED - 50)

        # --- Velocity score ---
        velocity_score = v / self.max_v

        # --- Goal progress score ---
        dist_start = math.sqrt((rx - goal_x) ** 2 + (ry - goal_y) ** 2)
        dist_end = math.sqrt((last_x - goal_x) ** 2 + (last_y - goal_y) ** 2)
        if dist_start > 1e-4:
            progress = (dist_start - dist_end) / dist_start
            goal_score = max(0.0, min(1.0, 0.5 + progress * 2.0))
        else:
            goal_score = 0.5

        # --- Path following score (cross-track error) ---
        # Also computes path_dir for path_alignment scoring.
        # KEY FIX for doorway oscillation: choose the reference direction
        # based on the robot's geometric relationship to the nearest path
        # segment:
        #   1. Robot is far from path (cross-track > rejoin_threshold): the
        #      priority is to REJOIN the path, so use direction from robot to
        #      the nearest projection point. This is the critical case — e.g.
        #      robot is east of a doorway while path starts west of it; using
        #      the segment direction (NE) would penalize the correct W-bound
        #      trajectory. Using "robot → projection" rewards it.
        #   2. Robot is alongside the segment (small cross-track, t in [0,1]):
        #      use the segment direction (encourage forward progress along path).
        #   3. Robot is beyond segment start (t < 0): head toward segment start.
        #   4. Robot is beyond segment end (t > 1): head toward segment end.
        path_score = 0.5
        path_dir = None  # will be set for path_alignment
        rejoin_threshold = 0.3  # if cross-track > this, prioritize rejoining path
        if path and len(path) >= 2:
            min_cross = float('inf')
            nearest_seg = 0
            nearest_t = 0.0       # raw (unclamped) projection parameter
            nearest_proj = (path[0][0], path[0][1])  # projection point
            for i in range(len(path) - 1):
                ax, ay = path[i]
                bx, by = path[i + 1]
                seg_dx = bx - ax
                seg_dy = by - ay
                seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
                if seg_len2 < 1e-9:
                    d = math.sqrt((last_x - ax) ** 2 + (last_y - ay) ** 2)
                    t = 0.0
                    proj = (ax, ay)
                else:
                    t = ((last_x - ax) * seg_dx + (last_y - ay) * seg_dy) / seg_len2
                    t_clamped = max(0.0, min(1.0, t))
                    proj_x = ax + t_clamped * seg_dx
                    proj_y = ay + t_clamped * seg_dy
                    d = math.sqrt((last_x - proj_x) ** 2 + (last_y - proj_y) ** 2)
                    proj = (proj_x, proj_y)
                if d < min_cross:
                    min_cross = d
                    nearest_seg = i
                    nearest_t = t
                    nearest_proj = proj
            path_score = max(0.0, 1.0 - min_cross / 1.0)
            ax, ay = path[nearest_seg]
            bx, by = path[min(nearest_seg + 1, len(path) - 1)]
            seg_len = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            if seg_len > 1e-6:
                if min_cross > rejoin_threshold:
                    # Case 1: robot is far from path — rejoin by heading to
                    # the nearest projection point on the path.
                    tx, ty = nearest_proj[0] - last_x, nearest_proj[1] - last_y
                    if (tx * tx + ty * ty) > 1e-9:
                        path_dir = math.atan2(ty, tx)
                elif nearest_t < 0.0:
                    # Case 3: robot is before segment start
                    tx, ty = ax - last_x, ay - last_y
                    if (tx * tx + ty * ty) > 1e-9:
                        path_dir = math.atan2(ty, tx)
                elif nearest_t > 1.0:
                    # Case 4: robot is after segment end
                    tx, ty = bx - last_x, by - last_y
                    if (tx * tx + ty * ty) > 1e-9:
                        path_dir = math.atan2(ty, tx)
                else:
                    # Case 2: robot is on the segment — use segment direction
                    path_dir = math.atan2(by - ay, bx - ax)
        elif path and len(path) == 1:
            d = math.sqrt((last_x - path[0][0]) ** 2 + (last_y - path[0][1]) ** 2)
            path_score = max(0.0, 1.0 - d / 1.0)
            tx, ty = path[0][0] - last_x, path[0][1] - last_y
            if (tx * tx + ty * ty) > 1e-9:
                path_dir = math.atan2(ty, tx)

        # --- Path alignment score (NEW): trajectory direction vs path direction ---
        # This is the KEY fix for doorway oscillation. Even if a trajectory
        # drifts laterally off the path, if it moves ALONG the path direction
        # (e.g. straight through a doorway), it gets a high score. This prevents
        # the DWA from preferring trajectories that drift sideways toward open
        # space when the planned path goes through a narrow passage.
        if path_dir is not None and move_dist > 1e-4:
            traj_dir = math.atan2(dy, dx)
            dir_diff = abs(path_dir - traj_dir)
            while dir_diff > math.pi:
                dir_diff = 2 * math.pi - dir_diff
            path_align_score = 1.0 - dir_diff / math.pi
        else:
            path_align_score = 0.5  # neutral when no path or no displacement

        # Total score with adaptive weights
        total = (self.w_heading * heading_score +
                 w_clearance * clearance_score +
                 self.w_velocity * velocity_score +
                 self.w_goal * goal_score +
                 w_path * path_score +
                 w_path_align * path_align_score)

        return total

    def reset(self):
        """Reset velocity state (e.g., after teleport or collision)."""
        self.prev_v = 0.0
        self.prev_w = 0.0

    def velocity_to_step(self, v, w, ryaw, dt=0.1):
        """Convert velocity command to position step for CoppeliaSim.

        Uses midpoint heading (ryaw + half the yaw change) for accurate
        displacement. Must pass the robot's current yaw.

        Returns (dx, dy, dyaw) for position/orientation update.
        """
        mid_yaw = ryaw + w * dt * 0.5  # midpoint heading during this step
        dx = v * math.cos(mid_yaw) * dt
        dy = v * math.sin(mid_yaw) * dt
        dyaw = w * dt
        return dx, dy, dyaw
