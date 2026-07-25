"""
Dynamic Obstacle Tracking + Spatio-Temporal Planning (v4.0)
=============================================================
Tracks moving obstacles with Kalman filters and performs velocity
obstacle (VO) based collision avoidance in the velocity space.

Components:
  1. ObstacleTracker: detects and tracks dynamic obstacles from LiDAR scans
  2. VelocityObstacle: computes forbidden velocities that would cause collision
  3. SpatioTemporalPlanner: integrates VO into local planner

References:
  - Van den Berg et al. (2011) "Reciprocal Velocity Obstacles for
    Real-Time Multi-Agent Navigation" (RVO)
  - Fiorini & Shiller (1998) "Motion Planning in Dynamic Environments
    Using Velocity Obstacles"
  - Snape et al. (2011) "Hybrid Reciprocal Velocity Obstacles" (HRVO)
  - Kalman (1960) "A New Approach to Linear Filtering and Prediction Problems"
"""
import os
import math
import numpy as np
from collections import deque

USE_DYNAMIC_OBSTACLES = os.environ.get("USE_DYNAMIC_OBSTACLES", "0") == "1"
USE_SPATIOTEMPORAL = os.environ.get("USE_SPATIOTEMPORAL", "0") == "1"


class KalmanTracker:
    """2D constant-velocity Kalman filter for tracking a single obstacle."""

    def __init__(self, x, y, vx=0.0, vy=0.0, dt=0.1):
        # State: [x, y, vx, vy]
        self.x = np.array([x, y, vx, vy], dtype=np.float64)
        self.dt = dt

        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        # Measurement matrix (observe position only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float64)

        # Process noise covariance
        sigma_ax, sigma_ay = 0.5, 0.5  # acceleration noise
        Q_x = np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ])
        self.Q = Q_x * np.array([[sigma_ax**2], [sigma_ay**2], [sigma_ax**2], [sigma_ay**2]])
        # Fix: proper Q
        self.Q = np.array([
            [dt**4/4 * sigma_ax**2, 0, dt**3/2 * sigma_ax**2, 0],
            [0, dt**4/4 * sigma_ay**2, 0, dt**3/2 * sigma_ay**2],
            [dt**3/2 * sigma_ax**2, 0, dt**2 * sigma_ax**2, 0],
            [0, dt**3/2 * sigma_ay**2, 0, dt**2 * sigma_ay**2]
        ], dtype=np.float64)

        # Measurement noise covariance
        self.R = np.diag([0.05**2, 0.05**2])

        # Initial covariance
        self.P = np.diag([0.5**2, 0.5**2, 0.3**2, 0.3**2])

        self.age = 0
        self.hits = 0
        self.missed = 0

    def predict(self):
        """Predict state to next time step."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1

    def update(self, z):
        """Update with measurement z = [x, y]."""
        y = z - self.H @ self.x  # innovation
        S = self.H @ self.P @ self.H.T + self.R  # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.hits += 1
        self.missed = 0

    def predict_position(self, t):
        """Predict position at time t seconds ahead."""
        x_future = self.x[0] + self.x[2] * t
        y_future = self.x[1] + self.x[3] * t
        return x_future, y_future

    @property
    def pos(self):
        return self.x[0], self.x[1]

    @property
    def vel(self):
        return self.x[2], self.x[3]


class ObstacleTracker:
    """Tracks multiple dynamic obstacles from LiDAR scans.

    Detects moving objects by subtracting the static map and clusters
    remaining points into obstacles, then tracks each with a Kalman filter.
    """

    def __init__(self, static_map, cfg=None, max_obstacles=10, dt=0.1,
                 cluster_distance=0.5, min_cluster_size=3,
                 match_distance=1.0, max_missed=10):
        self.static_map = static_map
        cfg = cfg or {}
        self.max_obstacles = cfg.get('max_obstacles', max_obstacles)
        self.cluster_distance = cfg.get('cluster_distance_m', cluster_distance)
        self.min_cluster_size = cfg.get('min_cluster_size', min_cluster_size)
        self.match_distance = cfg.get('match_distance_m', match_distance)
        self.max_missed = cfg.get('max_missed_frames', max_missed)
        self.dt = cfg.get('dt', dt)

        self.trackers = []  # list of KalmanTracker
        self.next_id = 0

    def update(self, rx, ry, ryaw, angles, distances, frame):
        """Process a new scan and update obstacle tracks.

        Args:
            rx, ry, ryaw: robot pose
            angles, distances: LiDAR scan
            frame: current frame number

        Returns:
            list of (id, x, y, vx, vy, radius) tuples for active tracks
        """
        # Step 1: Extract dynamic points (not matching static map)
        dynamic_points = []
        for angle, dist in zip(angles, distances):
            if dist >= 8.0 or dist < 0.3:
                continue
            # World position of hit
            wx = rx + dist * math.cos(ryaw + angle)
            wy = ry + dist * math.sin(ryaw + angle)

            # Check if this point matches static map obstacles
            gx, gy = self.static_map.world_to_grid(wx, wy)
            if not self.static_map.in_bounds(gx, gy):
                continue

            # If the cell is unknown, it might be a new obstacle
            if self.static_map.is_unknown(gx, gy):
                dynamic_points.append((wx, wy))
            elif self.static_map.is_free(gx, gy):
                # Point in free space = obstacle hit
                dynamic_points.append((wx, wy))
            # else: occupied (matches static map) -> skip

        if not dynamic_points:
            # All trackers missed
            for t in self.trackers:
                t.predict()
                t.missed += 1
            self._cleanup_old()
            return self._active_tracks()

        # Step 2: Cluster dynamic points
        clusters = self._cluster_points(dynamic_points)

        # Step 3: Predict existing trackers
        for t in self.trackers:
            t.predict()

        # Step 4: Match clusters to trackers (greedy nearest neighbor)
        matched_tracker = set()
        matched_cluster = set()

        # Compute distances
        dists = []
        for ci, (cx, cy) in enumerate(clusters):
            for ti, t in enumerate(self.trackers):
                d = math.sqrt((cx - t.pos[0])**2 + (cy - t.pos[1])**2)
                if d < self.match_distance:
                    dists.append((d, ci, ti))

        dists.sort()

        for d, ci, ti in dists:
            if ci in matched_cluster or ti in matched_tracker:
                continue
            self.trackers[ti].update(np.array([clusters[ci][0], clusters[ci][1]]))
            matched_cluster.add(ci)
            matched_tracker.add(ti)

        # Step 5: Mark missed trackers
        for ti, t in enumerate(self.trackers):
            if ti not in matched_tracker:
                t.missed += 1

        # Step 6: Create new trackers for unmatched clusters
        for ci, (cx, cy) in enumerate(clusters):
            if ci in matched_cluster:
                continue
            if len(self.trackers) < self.max_obstacles:
                new_t = KalmanTracker(cx, cy, 0.0, 0.0, self.dt)
                self.trackers.append(new_t)

        # Step 7: Cleanup
        self._cleanup_old()

        return self._active_tracks()

    def _cluster_points(self, points):
        """Simple DBSCAN-like clustering of 2D points."""
        if not points:
            return []

        n = len(points)
        visited = [False] * n
        clusters = []

        for i in range(n):
            if visited[i]:
                continue
            # Start new cluster
            cluster = [i]
            visited[i] = True
            queue = [i]
            while queue:
                idx = queue.pop()
                px, py = points[idx]
                for j in range(n):
                    if visited[j]:
                        continue
                    qx, qy = points[j]
                    d = math.sqrt((px - qx)**2 + (py - qy)**2)
                    if d < self.cluster_distance:
                        visited[j] = True
                        cluster.append(j)
                        queue.append(j)

            if len(cluster) >= self.min_cluster_size:
                # Compute centroid
                cx = sum(points[k][0] for k in cluster) / len(cluster)
                cy = sum(points[k][1] for k in cluster) / len(cluster)
                clusters.append((cx, cy))

        return clusters

    def _cleanup_old(self):
        """Remove trackers that have been missed too many times."""
        self.trackers = [t for t in self.trackers if t.missed < self.max_missed]

    def _active_tracks(self):
        """Return list of active obstacle tracks."""
        tracks = []
        for i, t in enumerate(self.trackers):
            if t.hits < 2:  # need at least 2 measurements
                continue
            vx, vy = t.vel
            speed = math.sqrt(vx**2 + vy**2)
            if speed < 0.05:  # not moving much -> probably static noise
                continue
            x, y = t.pos
            tracks.append((i, x, y, vx, vy, 0.3))  # radius estimate
        return tracks


class VelocityObstacle:
    """Velocity Obstacle computation for collision avoidance.

    For a robot at position A with radius r_A, and an obstacle at
    position B with velocity v_B and radius r_B, the velocity obstacle
    VO_A|B is the set of velocities v_A for the robot that would result
    in collision at some future time.
    """

    @staticmethod
    def compute_vo(pa, pb, va, vb, r_a, r_b, time_horizon=3.0):
        """Compute velocity obstacle for robot A avoiding obstacle B.

        Args:
            pa: (x, y) robot position
            pb: (x, y) obstacle position
            va: (vx, vy) robot velocity (not used directly for VO shape)
            vb: (vx, vy) obstacle velocity
            r_a: robot radius
            r_b: obstacle radius
            time_horizon: look-ahead time

        Returns:
            (apex_x, apex_y, half_angle, direction) defining the VO cone
            The VO is a Minkowski sum of the obstacle's velocity with a
            velocity cone.
        """
        # Relative position of B w.r.t. A
        dx = pb[0] - pa[0]
        dy = pb[1] - pa[1]
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 1e-6:
            # Already collided - VO is full space
            return vb[0], vb[1], math.pi, 0.0

        # Combined radius
        R = r_a + r_b

        if dist <= R:
            # Already in collision - full space is VO
            return vb[0], vb[1], math.pi, 0.0

        # Cone angle: sin(alpha) = R / dist
        alpha = math.asin(min(R / dist, 1.0))

        # Direction from A to B
        direction = math.atan2(dy, dx)

        # VO apex = obstacle velocity
        # VO is a cone starting at v_B, opening in direction away from B
        apex_x = vb[0]
        apex_y = vb[1]

        return apex_x, apex_y, alpha, direction + math.pi  # cone points opposite

    @staticmethod
    def velocity_in_vo(vx, vy, vo):
        """Check if a velocity is inside a velocity obstacle cone."""
        apex_x, apex_y, half_angle, direction = vo
        # Relative velocity from apex
        rvx = vx - apex_x
        rvy = vy - apex_y

        speed_r = math.sqrt(rvx**2 + rvy**2)
        if speed_r < 1e-6:
            return True  # at apex = definitely in VO

        # Angle of relative velocity
        angle_r = math.atan2(rvy, rvx)
        # Angle difference from cone direction
        diff = abs(angle_r - direction)
        while diff > math.pi:
            diff = abs(diff - 2 * math.pi)

        return diff <= half_angle

    @staticmethod
    def find_safe_velocity(desired_v, desired_w, robot_x, robot_y, robot_yaw,
                            obstacles, robot_radius, max_v, max_w, time_horizon=3.0):
        """Find a safe velocity that avoids all obstacles.

        Uses ORCA-like approach: if desired velocity is in VO, find the
        closest safe velocity on the VO boundary.

        Args:
            desired_v, desired_w: desired command
            robot_x, robot_y, robot_yaw: robot pose
            obstacles: list of (x, y, vx, vy, radius)
            robot_radius: robot radius
            max_v, max_w: velocity limits

        Returns:
            (safe_v, safe_w) adjusted velocities
        """
        # Convert desired (v, w) to Cartesian velocity of robot center
        vx = desired_v * math.cos(robot_yaw)
        vy = desired_v * math.sin(robot_yaw)

        # Check each obstacle
        for (ox, oy, ovx, ovy, orad) in obstacles:
            vo = VelocityObstacle.compute_vo(
                (robot_x, robot_y), (ox, oy),
                (vx, vy), (ovx, ovy),
                robot_radius, orad, time_horizon)

            if VelocityObstacle.velocity_in_vo(vx, vy, vo):
                # Project to VO boundary
                vx, vy = VelocityObstacle._project_out_vo(vx, vy, vo)

        # Convert back to (v, w)
        # For differential drive, this is approximate
        safe_v = math.sqrt(vx**2 + vy**2)
        # Check direction
        desired_dir = math.atan2(vy, vx)
        angle_diff = desired_dir - robot_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # If moving backward
        if abs(angle_diff) > math.pi / 2:
            safe_v = -safe_v

        # Compute omega needed to turn toward desired direction
        # Simple proportional controller
        safe_w = max(-max_w, min(max_w, angle_diff * 2.0))

        # Clamp
        safe_v = max(-max_v, min(max_v, safe_v))

        return safe_v, safe_w

    @staticmethod
    def _project_out_vo(vx, vy, vo):
        """Project a velocity onto the boundary of a velocity obstacle."""
        apex_x, apex_y, half_angle, direction = vo

        # Relative to apex
        rvx = vx - apex_x
        rvy = vy - apex_y
        speed = math.sqrt(rvx**2 + rvy**2)

        if speed < 1e-6:
            # At apex - push out along cone direction
            nx = math.cos(direction + half_angle)
            ny = math.sin(direction + half_angle)
            return apex_x + nx * 0.1, apex_y + ny * 0.1

        # Current angle
        current_angle = math.atan2(rvy, rvx)

        # Find nearest cone boundary
        left_angle = direction - half_angle
        right_angle = direction + half_angle

        # Normalize angles
        def _ang_diff(a, b):
            d = a - b
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            return d

        diff_left = abs(_ang_diff(current_angle, left_angle))
        diff_right = abs(_ang_diff(current_angle, right_angle))

        if diff_left < diff_right:
            bound_angle = left_angle
        else:
            bound_angle = right_angle

        # Project to boundary (same speed, different angle)
        new_rvx = speed * math.cos(bound_angle)
        new_rvy = speed * math.sin(bound_angle)

        return apex_x + new_rvx, apex_y + new_rvy
