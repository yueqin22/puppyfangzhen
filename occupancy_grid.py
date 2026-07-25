#!/usr/bin/env python3
"""
Occupancy Grid Map
==================
2D occupancy grid with Bresenham ray tracing for LiDAR-based mapping.
Resolution: 0.1m per cell (commercial-grade, vs 1m in original code).

Values:
  0   = free (observed empty)
  100 = occupied (observed obstacle)
  -1  = unknown (not yet observed)

Based on Gmapping-style occupancy grid mapping with inverse sensor model.
"""
import numpy as np
import math
import json
import os

# v3.0: Ablation switch — USE_INFO_THEORY=1 (default) uses Shannon entropy
# info gain (Bourgault 2002), USE_INFO_THEORY=0 reverts to count-based heuristic
USE_INFO_THEORY = os.environ.get("USE_INFO_THEORY", "1") == "1"

# Grid configuration
GRID_RESOLUTION = 0.1  # meters per cell
GRID_W = 100            # 10m / 0.1m = 100 cells wide
GRID_H = 80             # 8m / 0.1m = 80 cells tall
ORIGIN_X = -5.0         # world x of grid cell (0,0)
ORIGIN_Y = -4.0         # world y of grid cell (0,0)

# Log-odds bounds for stable mapping
LOG_ODDS_MIN = -2.0
LOG_ODDS_MAX = 3.5
LOG_ODDS_HIT = 0.85     # occupied observation
LOG_ODDS_MISS = -0.4    # free observation
LOG_ODDS_PRIOR = 0.0    # unknown


class OccupancyGrid:
    """2D occupancy grid with log-odds Bayesian update."""

    def __init__(self):
        self.resolution = GRID_RESOLUTION
        self.width = GRID_W
        self.height = GRID_H
        self.origin_x = ORIGIN_X
        self.origin_y = ORIGIN_Y
        # Log-odds representation for probabilistic mapping
        self.log_odds = np.zeros((GRID_H, GRID_W), dtype=np.float32)
        self.visited = np.zeros((GRID_H, GRID_W), dtype=bool)

    # --- Coordinate conversion ---
    def world_to_grid(self, x, y):
        """Convert world coords to grid indices."""
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coords (cell center)."""
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    # --- Occupancy query ---
    def get_probability(self, gx, gy):
        """Get occupancy probability [0, 1] for a cell."""
        if not self.in_bounds(gx, gy):
            return 0.5  # unknown for out-of-bounds
        lo = self.log_odds[gy, gx]
        return 1.0 - 1.0 / (1.0 + math.exp(lo))

    def is_free(self, gx, gy):
        """Cell is observed free (probability < 0.2)."""
        if not self.in_bounds(gx, gy):
            return False
        return self.log_odds[gy, gx] < -0.5

    def is_occupied(self, gx, gy):
        """Cell is observed occupied (probability > 0.65)."""
        if not self.in_bounds(gx, gy):
            return True  # out-of-bounds = wall
        return self.log_odds[gy, gx] > 0.6

    def is_unknown(self, gx, gy):
        """Cell has not been observed."""
        if not self.in_bounds(gx, gy):
            return False
        return abs(self.log_odds[gy, gx]) < 0.3

    def is_visited(self, x, y):
        """Check if a world position's cell has been visited by robot."""
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return False
        return self.visited[gy, gx]

    def mark_visited(self, x, y):
        """Mark a world position as visited. Returns True if new."""
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return False
        if self.visited[gy, gx]:
            return False
        self.visited[gy, gx] = True
        return True

    # --- Bresenham ray tracing ---
    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Yield cells along a Bresenham line from (x0,y0) to (x1,y1)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # --- Map update from LiDAR scan ---
    def update_from_scan(self, rx, ry, angles, distances, max_range=8.0):
        """Update grid using LiDAR rays (Bresenham ray tracing).

        For each ray:
        - Cells along the ray (robot -> hit point) are marked FREE
        - Cell at hit point is marked OCCUPIED
        - If ray reaches max_range (no hit), all cells are FREE

        Uses log-odds update for probabilistic stability.
        """
        rgx, rgy = self.world_to_grid(rx, ry)
        if not self.in_bounds(rgx, rgy):
            return

        for angle, dist in zip(angles, distances):
            # Calculate hit point in world coords
            if dist >= max_range or not np.isfinite(dist):
                dist = max_range
                hit_is_obstacle = False
            else:
                hit_is_obstacle = True

            hx = rx + dist * math.cos(angle)
            hy = ry + dist * math.sin(angle)
            hgx, hgy = self.world_to_grid(hx, hy)

            # Trace ray from robot to hit point
            for gx, gy in self._bresenham(rgx, rgy, hgx, hgy):
                if not self.in_bounds(gx, gy):
                    break
                if gx == hgx and gy == hgy and hit_is_obstacle:
                    self.log_odds[gy, gx] = min(
                        self.log_odds[gy, gx] + LOG_ODDS_HIT, LOG_ODDS_MAX)
                else:
                    self.log_odds[gy, gx] = max(
                        self.log_odds[gy, gx] + LOG_ODDS_MISS, LOG_ODDS_MIN)

    # --- Frontier detection for exploration ---
    def find_frontiers(self, rx, ry, max_frontiers=20):
        """Find frontier cells (free cells adjacent to unknown cells).

        Returns list of (wx, wy) world coordinates of frontier centroids,
        sorted by distance to robot. Used for exploration goal selection.
        """
        rgx, rgy = self.world_to_grid(rx, ry)
        frontiers = []

        # Find all frontier cells (free with unknown neighbor)
        frontier_mask = np.zeros_like(self.visited, dtype=bool)
        free_mask = self.log_odds < -0.5  # free cells
        unknown_mask = np.abs(self.log_odds) < 0.3  # unknown cells

        # Check 4-connected neighbors for unknown
        # Use np.pad + slice offset so out-of-bounds neighbors return False
        # (np.roll would wrap around grid edges and create false frontiers)
        padded_unknown = np.pad(unknown_mask, 1, constant_values=False)
        H, W = unknown_mask.shape
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                shifted = padded_unknown[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
                frontier_mask |= (free_mask & shifted)

        # Get frontier cell coordinates
        fy, fx = np.where(frontier_mask)
        if len(fy) == 0:
            return []

        # BFS 4-connected clustering (replaces greedy O(n²) approach).
        # O(n) complexity; produces natural clusters instead of square
        # Chebyshev-threshold clusters that could merge adjacent independent
        # frontiers.
        min_cluster_size = 3  # filter noise points
        frontier_points = list(zip(fy.tolist(), fx.tolist()))
        frontier_set = set(frontier_points)
        visited_cluster = set()
        clusters = []
        for (gy, gx) in frontier_points:
            if (gy, gx) in visited_cluster:
                continue
            # BFS flood fill over 4-connected frontier cells
            cluster = []
            queue = [(gy, gx)]
            visited_cluster.add((gy, gx))
            while queue:
                cy, cx = queue.pop(0)
                cluster.append((cy, cx))
                for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ny, nx = cy + dy, cx + dx
                    if (ny, nx) in frontier_set and (ny, nx) not in visited_cluster:
                        visited_cluster.add((ny, nx))
                        queue.append((ny, nx))
            if len(cluster) >= min_cluster_size:
                clusters.append(cluster)

        # Compute centroid of each cluster and distance to robot
        for cluster in clusters:
            avg_gy = sum(p[0] for p in cluster) / len(cluster)
            avg_gx = sum(p[1] for p in cluster) / len(cluster)
            wx, wy = self.grid_to_world(int(avg_gx), int(avg_gy))
            dist = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
            frontiers.append((dist, wx, wy, len(cluster)))

        frontiers.sort(key=lambda f: f[0])
        return [(f[1], f[2], f[3]) for f in frontiers[:max_frontiers]]

    def find_frontiers_with_info_gain(self, rx, ry, sensor_range=4.0,
                                      max_frontiers=20):
        """Find frontiers with information-theoretic info gain (v3.0).

        v3.0: Uses Shannon entropy-based information gain instead of simple
        cell counting (Bourgault et al. 2002, "Information-based robotic
        exploration").

        For each frontier, computes the expected information gain:
            I(M;Z) = Σ H(p_i)  for cells within sensor range
        where H(p) = -p·log2(p) - (1-p)·log2(1-p) is the Shannon entropy
        of the occupancy probability p_i of each cell.

        Unknown cells (p=0.5) have H=1 bit (maximum uncertainty).
        Known free/occupied cells have H≈0 (minimum uncertainty).
        Partially observed cells have intermediate H, capturing the
        gradient of knowledge at frontier boundaries.

        The utility function follows Bourgault et al.:
            utility = α · I(M;Z)_norm - β · d_norm
        balancing exploration value (information gain) against travel cost.

        Args:
            rx, ry: robot position (world frame)
            sensor_range: LiDAR range used for info gain estimation (meters)
            max_frontiers: max number of frontiers to return

        Returns:
            List of (wx, wy, cluster_size, info_gain, distance) tuples,
            sorted by descending utility. info_gain is in bits.
        """
        # Get basic frontiers (sorted by distance)
        base = self.find_frontiers(rx, ry, max_frontiers=max_frontiers * 2)
        if not base:
            return []

        # Pre-compute occupancy probability and entropy for entire grid
        # p = 1 / (1 + exp(-log_odds)), clamped for numerical stability
        lo = np.clip(self.log_odds, -10, 10)
        p = 1.0 / (1.0 + np.exp(-lo))
        # Shannon entropy: H(p) = -p·log2(p) - (1-p)·log2(1-p)
        eps = 1e-12
        p_safe = np.clip(p, eps, 1 - eps)
        cell_entropy = -p_safe * np.log2(p_safe) - (1 - p_safe) * np.log2(1 - p_safe)
        # v3.0 ablation: when USE_INFO_THEORY=0, use binary mask (count-based)
        # instead of Shannon entropy (reverts to pre-v3.0 behavior)
        if not USE_INFO_THEORY:
            cell_entropy = (np.abs(self.log_odds) < 0.3).astype(np.float64)

        sensor_cells = int(sensor_range / self.resolution)
        results = []
        for wx, wy, size in base:
            fgx, fgy = self.world_to_grid(wx, wy)
            # Bounding box of sensor range around frontier
            x0 = max(0, fgx - sensor_cells)
            x1 = min(self.width, fgx + sensor_cells + 1)
            y0 = max(0, fgy - sensor_cells)
            y1 = min(self.height, fgy + sensor_cells + 1)

            # Circular mask
            yy, xx = np.mgrid[y0:y1, x0:x1]
            mask = (xx - fgx) ** 2 + (yy - fgy) ** 2 <= sensor_cells ** 2

            # v3.0: Information gain = sum of Shannon entropies of observable cells
            # This captures both unknown cells (H=1) and partially observed
            # cells (0 < H < 1), providing a smoother gradient than binary counting
            local_entropy = cell_entropy[y0:y1, x0:x1]
            info_gain = float(np.sum(local_entropy * mask))

            # Subtract entropy already observable from robot's position
            # to avoid over-prioritizing nearby frontiers
            rgx, rgy = self.world_to_grid(rx, ry)
            r_sensor_cells = int(sensor_range / self.resolution)
            rx0 = max(0, rgx - r_sensor_cells)
            rx1 = min(self.width, rgx + r_sensor_cells + 1)
            ry0 = max(0, rgy - r_sensor_cells)
            ry1 = min(self.height, rgy + r_sensor_cells + 1)
            ryy, rxx = np.mgrid[ry0:ry1, rx0:rx1]
            r_mask = (rxx - rgx) ** 2 + (ryy - rgy) ** 2 <= r_sensor_cells ** 2
            robot_entropy = cell_entropy[ry0:ry1, rx0:rx1]
            robot_info = float(np.sum(robot_entropy * r_mask))
            # //2 to avoid over-subtracting shared entropy regions
            info_gain = max(0.0, info_gain - robot_info / 2.0)

            dist = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
            results.append((wx, wy, size, info_gain, dist))

        # Bourgault et al. utility: α · I_norm - β · d_norm
        # α=1.0 (exploration value), β=0.5 (travel cost)
        alpha = 1.0
        beta = 0.5
        max_gain = max((r[3] for r in results), default=1.0)
        max_dist = max((r[4] for r in results), default=1.0)
        if max_gain <= 0:
            max_gain = 1.0

        def utility(r):
            gain_norm = r[3] / max_gain
            dist_norm = r[4] / max_dist if max_dist > 0 else 0
            return alpha * gain_norm - beta * dist_norm

        results.sort(key=utility, reverse=True)
        return results[:max_frontiers]

    # --- Coverage statistics ---
    def coverage_percent(self):
        """Percentage of in-bounds cells that have been observed."""
        total = self.width * self.height
        observed = np.sum(np.abs(self.log_odds) > 0.3)
        return 100.0 * observed / total

    def visited_count(self):
        """Number of visited cells (robot positions)."""
        return int(np.sum(self.visited))

    # --- Persistence ---
    def save(self, filepath):
        """Save grid to file (numpy format)."""
        np.savez(filepath,
                 log_odds=self.log_odds,
                 visited=self.visited)

    def load(self, filepath):
        """Load grid from file."""
        if not os.path.exists(filepath):
            return False
        data = np.load(filepath)
        self.log_odds = data['log_odds']
        self.visited = data['visited']
        return True
