"""
Active SLAM + Next-Best-View Planner (v4.0)
============================================
Active SLAM: robot actively chooses actions that reduce both map and
localization uncertainty, not just explore new space.

NBV Planner: samples candidate observation poses and selects the one
with the highest expected information gain, considering both geometric
and semantic uncertainty.

References:
  - Bourgault et al. (2002) "Information-based adaptive robotic exploration"
  - Julian et al. (2013) "On mutual information-based control of range sensing
    robots for map building"
  - Conroy et al. (2009) "Minimizing exploration time with expected information"
  - Huber et al. (2009) "Optimal pose selection for autonomous exploration"
"""
import os
import math
import numpy as np
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION
from costmap import Costmap

USE_ACTIVE_SLAM = os.environ.get("USE_ACTIVE_SLAM", "1") == "1"
USE_NBV = os.environ.get("USE_NBV", "0") == "1"


class ActiveSLAMEstimator:
    """Estimates localization uncertainty map and information gain for Active SLAM.

    Maintains a grid of expected localization improvement if we observe each cell.
    Combines:
      1. Geometric map information gain (from occupancy grid entropy)
      2. Localization information gain (reduction in AMCL covariance)
      3. Semantic information gain (from probabilistic semantic map)
    """

    def __init__(self, occ_grid, costmap, semantic_mapper=None, cfg=None,
                 w_geometric=0.5, w_localization=0.3, w_semantic=0.2,
                 sensor_range=8.0):
        self.occ_grid = occ_grid
        self.costmap = costmap
        self.semantic_mapper = semantic_mapper
        cfg = cfg or {}

        # Weights for combining information gains
        self.w_geo = cfg.get('w_geometric', w_geometric)
        self.w_loc = cfg.get('w_localization', w_localization)
        self.w_sem = cfg.get('w_semantic', w_semantic)

        # Sensor parameters
        self.sensor_range = cfg.get('sensor_range_m', sensor_range)
        self.sensor_range_cells = int(self.sensor_range / GRID_RESOLUTION)
        self.sensor_fov_deg = cfg.get('sensor_fov_deg', 360)

        # Localization uncertainty estimate (tracked externally via AMCL)
        self.loc_covariance = np.eye(3) * 0.1  # initial covariance

    def update_localization_covariance(self, covariance):
        """Update the current localization covariance estimate."""
        self.loc_covariance = np.array(covariance)

    def update_from_amcl(self, amcl_instance):
        """Extract localization uncertainty from an AMCL instance.

        Computes weighted covariance of the particle cloud as a measure
        of localization uncertainty.
        """
        particles = amcl_instance.particles
        weights = amcl_instance.weights

        # Weighted mean
        mean = np.average(particles, axis=0, weights=weights)

        # Weighted covariance
        diff = particles - mean
        cov = np.zeros((3, 3))
        for i in range(len(weights)):
            d = diff[i:i+1].T
            cov += weights[i] * (d @ d.T)

        self.loc_covariance = cov
        return cov

    def expected_info_gain(self, wx, wy, wyaw=0.0):
        """Compute expected total information gain from pose (wx, wy, wyaw).

        Combines geometric, localization, and semantic IG.
        Returns a scalar utility value (higher = better).
        """
        geo_ig = self._geometric_ig(wx, wy)
        loc_ig = self._localization_ig(wx, wy)
        sem_ig = self._semantic_ig(wx, wy) if self.semantic_mapper else 0.0

        total = (self.w_geo * geo_ig
                 + self.w_loc * loc_ig
                 + self.w_sem * sem_ig)
        return total, geo_ig, loc_ig, sem_ig

    def _geometric_ig(self, wx, wy):
        """Expected geometric (occupancy) information gain from pose (wx, wy).

        Count of unknown cells within sensor range (proxy for map entropy reduction).
        """
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        r = self.sensor_range_cells
        count = 0
        total = 0
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx*dx + dy*dy > r*r:
                    continue
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
                    total += 1
                    if self.occ_grid.is_unknown(cx, cy):
                        count += 1
        return count / max(total, 1)

    def _localization_ig(self, wx, wy):
        """Expected localization information gain from pose (wx, wy).

        Approximated by the number of occupied (landmark) cells in view,
        weighted by the current localization uncertainty.

        More landmarks in view + higher current uncertainty = more IG.
        """
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        r = self.sensor_range_cells
        occupied_count = 0
        total = 0
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx*dx + dy*dy > r*r:
                    continue
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < GRID_W and 0 <= cy < GRID_H:
                    total += 1
                    if self.occ_grid.is_occupied(cx, cy):
                        occupied_count += 1

        # IG proportional to: landmarks_in_view * current_uncertainty
        # Current uncertainty = trace of covariance
        uncertainty = np.trace(self.loc_covariance)
        landmark_density = occupied_count / max(total, 1)
        return landmark_density * min(1.0, uncertainty * 10)

    def _semantic_ig(self, wx, wy):
        """Expected semantic information gain from pose (wx, wy)."""
        if self.semantic_mapper is None:
            return 0.0
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        return self.semantic_mapper.semantic_info_gain_region(
            gx, gy, radius_cells=self.sensor_range_cells
        ) / (self.sensor_range_cells ** 2 * math.pi)


class NBVPlanner:
    """Next-Best-View planner for Active SLAM.

    Samples candidate observation poses and selects the one with highest
    expected information gain, subject to reachability constraints.

    Two modes:
      1. 'frontier_based': NBV on frontier cells (faster, exploration-focused)
      2. 'full_sampling': sample poses across unknown boundary (more thorough)
    """

    def __init__(self, occ_grid, costmap, active_slam, astar_planner=None, cfg=None,
                 n_candidates=20, sensor_range=8.0, mode='frontier_based'):
        self.occ_grid = occ_grid
        self.costmap = costmap
        self.active_slam = active_slam
        self.astar = astar_planner
        cfg = cfg or {}

        self.mode = cfg.get('nbv_mode', mode)
        self.n_candidates = cfg.get('n_candidates', n_candidates)
        self.sensor_range = cfg.get('sensor_range_m', sensor_range)
        self.min_path_cost = cfg.get('min_path_cost_weight', 0.3)

    def plan_nbv(self, rx, ry, ryaw, frontiers=None):
        """Select the next best view pose.

        Args:
            rx, ry, ryaw: current robot pose
            frontiers: list of (fx, fy, size, ...) frontier tuples (optional)

        Returns:
            (nbv_x, nbv_y, nbv_yaw, info_gain) or None if no candidates
        """
        if self.mode == 'frontier_based' and frontiers:
            return self._frontier_based(rx, ry, ryaw, frontiers)
        else:
            return self._boundary_sampling(rx, ry, ryaw)

    def _frontier_based(self, rx, ry, ryaw, frontiers):
        """Evaluate NBV for each frontier candidate."""
        best = None
        best_ig = -1.0

        for f in frontiers[:self.n_candidates]:
            fx, fy = f[0], f[1]
            # Compute expected IG from a pose slightly before the frontier
            angle_to_frontier = math.atan2(fy - ry, fx - rx)
            offset_dist = 1.0  # stand back 1m from frontier
            pose_x = fx - offset_dist * math.cos(angle_to_frontier)
            pose_y = fy - offset_dist * math.sin(angle_to_frontier)
            pose_yaw = angle_to_frontier

            # Check safety
            if not self.costmap.is_safe(pose_x, pose_y):
                continue

            ig_total, _, _, _ = self.active_slam.expected_info_gain(
                pose_x, pose_y, pose_yaw)

            # Path cost penalty
            dist = math.sqrt((fx - rx)**2 + (fy - ry)**2)
            score = ig_total - self.min_path_cost * dist / self.sensor_range

            if score > best_ig:
                best_ig = score
                best = (fx, fy, pose_yaw, ig_total)

        return best

    def _boundary_sampling(self, rx, ry, ryaw):
        """Sample poses along the known-unknown boundary."""
        candidates = []
        # Find unknown cells adjacent to known cells
        for gy in range(1, GRID_H - 1):
            for gx in range(1, GRID_W - 1):
                if not self.occ_grid.is_unknown(gx, gy):
                    continue
                # Check if at least one neighbor is known
                known_neighbor = False
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        if not self.occ_grid.is_unknown(nx, ny):
                            known_neighbor = True
                            break
                if known_neighbor:
                    wx, wy = self.occ_grid.grid_to_world(gx, gy)
                    candidates.append((wx, wy))

        if not candidates:
            return None

        # Sample a subset
        if len(candidates) > self.n_candidates:
            step = len(candidates) // self.n_candidates
            candidates = candidates[::step]

        best = None
        best_ig = -1.0
        for cx, cy in candidates:
            angle = math.atan2(cy - ry, cx - rx)
            pose_x = cx - 0.5 * math.cos(angle)
            pose_y = cy - 0.5 * math.sin(angle)

            if not self.costmap.is_safe(pose_x, pose_y):
                continue

            ig_total, _, _, _ = self.active_slam.expected_info_gain(pose_x, pose_y, angle)
            dist = math.sqrt((cx - rx)**2 + (cy - ry)**2)
            score = ig_total - self.min_path_cost * dist / self.sensor_range

            if score > best_ig:
                best_ig = score
                best = (cx, cy, angle, ig_total)

        return best
