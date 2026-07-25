"""
Loop Closure Detection + Pose Graph Optimization (v4.0)
=========================================================
Detects loop closures via scan matching (ICP-based) and optimizes the
pose graph using Gauss-Newton to correct accumulated drift.

The system maintains a graph of keyframes with odometry edges. When a
loop closure is detected, a new constraint is added and the graph is
optimized to minimize squared error.

References:
  - Grisetti et al. (2007) "Improved Techniques for Grid Mapping with
    Rao-Blackwellized Particle Filters" (GMapping, scan matching)
  - Hess et al. (2016) "Real-Time Loop Closure in 2D LIDAR SLAM"
    (Cartographer)
  - Lu & Milios (1997) "Globally Consistent Range Scan Alignment for
    Environment Mapping" (pose graph optimization)
  - Konolige (2010) "Sparse Sparse Bundle Adjustment" (g2o-style)
"""
import os
import math
import numpy as np
from collections import defaultdict
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION

USE_LOOP_CLOSURE = os.environ.get("USE_LOOP_CLOSURE", "0") == "1"


class Keyframe:
    """A keyframe in the pose graph."""
    __slots__ = ['id', 'x', 'y', 'theta', 'scan']

    def __init__(self, kid, x, y, theta, scan=None):
        self.id = kid
        self.x = x
        self.y = y
        self.theta = theta
        self.scan = scan  # (angles, distances) tuple


class PoseGraph:
    """Pose graph with Gauss-Newton optimization.

    Nodes = robot poses (x, y, theta)
    Edges = constraints between poses (odometry or loop closure)

    Optimizes using iterative Gauss-Newton (or Levenberg-Marquardt)
    to minimize sum of squared constraint residuals.
    """

    def __init__(self):
        self.keyframes = []  # list of Keyframe
        self.edges = []      # list of (from_id, to_id, dx, dy, dtheta, info_mat, type)
        self.next_id = 0

    def add_keyframe(self, x, y, theta, scan=None):
        """Add a new keyframe and return its ID."""
        kid = self.next_id
        self.next_id += 1
        kf = Keyframe(kid, x, y, theta, scan)
        self.keyframes.append(kf)

        # Add odometry edge from previous keyframe
        if len(self.keyframes) > 1:
            prev = self.keyframes[-2]
            dx = x - prev.x
            dy = y - prev.y
            dtheta = self._normalize_angle(theta - prev.theta)
            info = np.diag([1.0 / 0.05**2, 1.0 / 0.05**2, 1.0 / 0.1**2])
            self.edges.append((prev.id, kid, dx, dy, dtheta, info, 'odom'))

        return kid

    def add_loop_closure(self, from_id, to_id, dx, dy, dtheta, confidence=0.7):
        """Add a loop closure constraint between two keyframes."""
        info = np.diag([confidence / 0.1**2, confidence / 0.1**2, confidence / 0.3**2])
        self.edges.append((from_id, to_id, dx, dy, dtheta, info, 'loop'))

    def optimize(self, n_iterations=20, lambda_damping=0.01):
        """Optimize the pose graph using Gauss-Newton with LM damping.

        Minimizes: sum_e ||e_i||^2_Omega
        where e = measurement - h(x) is the residual for each edge.
        """
        if len(self.keyframes) < 3:
            return 0.0

        n = len(self.keyframes)
        x = np.zeros(n * 3)  # state vector [x0, y0, t0, x1, y1, t1, ...]
        for i, kf in enumerate(self.keyframes):
            x[3*i] = kf.x
            x[3*i+1] = kf.y
            x[3*i+2] = kf.theta

        # Fix the first pose (gauge freedom)
        fixed_mask = np.zeros(n * 3, dtype=bool)
        fixed_mask[0:3] = True  # fix first keyframe

        prev_err = float('inf')
        for iteration in range(n_iterations):
            # Compute H and b for the normal equations: H * dx = b
            H = np.zeros((n * 3, n * 3))
            b = np.zeros(n * 3)
            total_err = 0.0

            for (from_id, to_id, meas_dx, meas_dy, meas_dt, info, etype) in self.edges:
                i, j = from_id, to_id
                xi = x[3*i:3*i+3]
                xj = x[3*j:3*j+3]

                # Prediction h(x_i, x_j)
                dx_pred, dy_pred, dt_pred = self._inverse_compose(xi, xj)

                # Error e = measurement - prediction
                ex = meas_dx - dx_pred
                ey = meas_dy - dy_pred
                et = self._normalize_angle(meas_dt - dt_pred)
                e = np.array([ex, ey, et])

                # Weighted error contribution
                total_err += e @ info @ e

                # Jacobian of h w.r.t x_i and x_j
                # h(x_i, x_j) = R(-theta_i) * (x_j - x_i)
                # dh/dx_i = ..., dh/dx_j = ...
                # Derivatives for the 2D pose composition:
                # dx_ij = cos(theta_i)*(xj - xi) + sin(theta_i)*(yj - yi)
                # dy_ij = -sin(theta_i)*(xj - xi) + cos(theta_i)*(yj - yi)
                # dt_ij = theta_j - theta_i
                cos_i = math.cos(xi[2])
                sin_i = math.sin(xi[2])
                dx = xj[0] - xi[0]
                dy = xj[1] - xi[1]

                # J_i = dh/d(x_i, y_i, theta_i)  (3x3)
                J_i = np.zeros((3, 3))
                J_i[0, 0] = -cos_i
                J_i[0, 1] = -sin_i
                J_i[0, 2] = -sin_i * dx + cos_i * dy
                J_i[1, 0] = sin_i
                J_i[1, 1] = -cos_i
                J_i[1, 2] = -cos_i * dx - sin_i * dy
                J_i[2, 2] = -1.0

                # J_j = dh/d(x_j, y_j, theta_j)  (3x3)
                J_j = np.zeros((3, 3))
                J_j[0, 0] = cos_i
                J_j[0, 1] = sin_i
                J_j[1, 0] = -sin_i
                J_j[1, 1] = cos_i
                J_j[2, 2] = 1.0

                # H += J^T Omega J, b += J^T Omega e
                H_ii = J_i.T @ info @ J_i
                H_ij = J_i.T @ info @ J_j
                H_jj = J_j.T @ info @ J_j
                b_i = J_i.T @ info @ e
                b_j = J_j.T @ info @ e

                si, sj = 3*i, 3*j
                H[si:si+3, si:si+3] += H_ii
                H[si:si+3, sj:sj+3] += H_ij
                H[sj:sj+3, si:si+3] += H_ij.T
                H[sj:sj+3, sj:sj+3] += H_jj
                b[si:si+3] += b_i
                b[sj:sj+3] += b_j

            # Apply LM damping
            H_damped = H + lambda_damping * np.eye(n * 3) * np.diag(H).clip(min=1e-6)

            # Fix first pose
            for idx in range(n * 3):
                if fixed_mask[idx]:
                    H_damped[idx, :] = 0
                    H_damped[idx, idx] = 1.0
                    b[idx] = 0.0

            # Solve H_damped * dx = b
            try:
                dx = np.linalg.solve(H_damped, b)
            except np.linalg.LinAlgError:
                break

            # Update
            x = x + dx  # Gauss-Newton: x_new = x + H^{-1} * J^T Omega e

            # Check convergence
            if abs(prev_err - total_err) < 1e-6:
                break
            prev_err = total_err

        # Update keyframes
        for i, kf in enumerate(self.keyframes):
            kf.x = x[3*i]
            kf.y = x[3*i+1]
            kf.theta = x[3*i+2]

        return total_err

    def _inverse_compose(self, xi, xj):
        """Compute relative pose from xi to xj: x_ij = x_i^{-1} * x_j"""
        dx = xj[0] - xi[0]
        dy = xj[1] - xi[1]
        cos_i = math.cos(xi[2])
        sin_i = math.sin(xi[2])
        local_dx = cos_i * dx + sin_i * dy
        local_dy = -sin_i * dx + cos_i * dy
        local_dt = self._normalize_angle(xj[2] - xi[2])
        return local_dx, local_dy, local_dt

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class LoopClosureDetector:
    """Detects loop closures by scan matching candidate keyframes.

    Strategy:
      1. For each new keyframe, find candidate keyframes near current
         position (geometric proximity)
      2. Perform ICP scan matching between current scan and candidate scan
      3. If matching error below threshold + overlap sufficient, accept
    """

    def __init__(self, cfg=None, min_keyframes_between=10,
                 icp_threshold=0.05, position_threshold=1.5,
                 overlap_threshold=0.5, icp_max_iter=20):
        cfg = cfg or {}
        self.min_keyframes_between = cfg.get('min_keyframes_between', min_keyframes_between)
        self.icp_max_iter = cfg.get('icp_max_iter', icp_max_iter)
        self.icp_threshold = cfg.get('icp_threshold', icp_threshold)
        self.overlap_threshold = cfg.get('overlap_threshold', overlap_threshold)
        self.position_threshold = cfg.get('position_threshold_m', position_threshold)
        self.angle_threshold = cfg.get('angle_threshold_rad', 0.5)

    def detect(self, current_kf, pose_graph, scan_angles, scan_dists):
        """Check if current keyframe closes a loop with any previous one.

        Returns:
            list of (candidate_id, dx, dy, dtheta, confidence) tuples
        """
        candidates = []
        n = len(pose_graph.keyframes)
        if n < self.min_keyframes_between + 2:
            return candidates

        # Find candidates by position proximity (skip recent ones)
        for i in range(n - self.min_keyframes_between - 1):
            kf = pose_graph.keyframes[i]
            pos_dist = math.sqrt(
                (kf.x - current_kf.x)**2 + (kf.y - current_kf.y)**2)
            angle_diff = abs(PoseGraph._normalize_angle(
                kf.theta - current_kf.theta))
            if pos_dist < self.position_threshold and angle_diff < self.angle_threshold:
                # Attempt ICP
                if kf.scan is not None:
                    dx, dy, dtheta, error, overlap = self._icp(
                        scan_angles, scan_dists, kf.scan[0], kf.scan[1])
                    if error < self.icp_threshold and overlap > self.overlap_threshold:
                        confidence = max(0.3, 1.0 - error / self.icp_threshold)
                        candidates.append((i, dx, dy, dtheta, confidence))

        return candidates

    def _icp(self, ang1, dist1, ang2, dist2):
        """Iterative Closest Point between two scans (simplified).

        Returns: (dx, dy, dtheta, mean_error, overlap_ratio)
        """
        # Convert scans to point clouds
        pts1 = []
        pts2 = []
        for a, d in zip(ang1, dist1):
            if d < 0.5 or d > 8.0:
                continue
            pts1.append([d * math.cos(a), d * math.sin(a)])
        for a, d in zip(ang2, dist2):
            if d < 0.5 or d > 8.0:
                continue
            pts2.append([d * math.cos(a), d * math.sin(a)])

        if len(pts1) < 5 or len(pts2) < 5:
            return 0.0, 0.0, 0.0, 1.0, 0.0

        pts1 = np.array(pts1)
        pts2 = np.array(pts2)

        # Initial transform = 0
        tx, ty, ttheta = 0.0, 0.0, 0.0

        prev_error = float('inf')
        mean_error = 1.0  # 初始化，防止首次迭代即break时未定义
        correspondences = []
        for iteration in range(self.icp_max_iter):
            # Transform pts1
            cos_t = math.cos(ttheta)
            sin_t = math.sin(ttheta)
            transformed = pts1.copy()
            transformed[:, 0] = pts1[:, 0] * cos_t - pts1[:, 1] * sin_t + tx
            transformed[:, 1] = pts1[:, 0] * sin_t + pts1[:, 1] * cos_t + ty

            # Find correspondences (nearest neighbor)
            errors = []
            correspondences = []
            for i, p in enumerate(transformed):
                if len(pts2) == 0:
                    break
                dists = np.sum((pts2 - p)**2, axis=1)
                j = int(np.argmin(dists))
                d = math.sqrt(dists[j])
                if d < 0.5:  # max correspondence distance
                    errors.append(d)
                    correspondences.append((i, j))

            if len(correspondences) < 3:
                break

            mean_error = np.mean(errors)

            # Compute transform using SVD-like approach for 2D
            # For simplicity: use centroid-based alignment
            src_pts = pts1[[c[0] for c in correspondences]]
            dst_pts = pts2[[c[1] for c in correspondences]]

            src_mean = np.mean(src_pts, axis=0)
            dst_mean = np.mean(dst_pts, axis=0)

            src_centered = src_pts - src_mean
            dst_centered = dst_pts - dst_mean

            # H = sum(src_c * dst_c^T)
            H = src_centered.T @ dst_centered
            # Angle from H
            angle = math.atan2(H[1, 0] - H[0, 1], H[0, 0] + H[1, 1])

            # Update transform
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            # Compose: new_T = T_delta * current_T
            delta_t = dst_mean - np.array([
                cos_a * src_mean[0] - sin_a * src_mean[1],
                sin_a * src_mean[0] + cos_a * src_mean[1]
            ])

            # Compose transformations
            new_theta = ttheta + angle
            new_tx = tx + cos_t * delta_t[0] - sin_t * delta_t[1]
            new_ty = ty + sin_t * delta_t[0] + cos_t * delta_t[1]

            tx, ty, ttheta = new_tx, new_ty, new_theta

            if abs(prev_error - mean_error) < 1e-5:
                break
            prev_error = mean_error

        overlap = len(correspondences) / max(len(pts1), len(pts2), 1)
        return tx, ty, ttheta, mean_error, overlap
