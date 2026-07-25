#!/usr/bin/env python3
"""
Layered Costmap
===============
Nav2-style layered costmap for path planning and obstacle avoidance.

Layers:
  1. Static layer  — from OccupancyGrid (known obstacles)
  2. Obstacle layer — from real-time LiDAR (dynamic obstacles)
  3. Inflation layer — expand obstacles by robot radius (cost gradient)

Cost values (Nav2 convention):
  0     = free space
  1-127 = inflation gradient (decreasing cost away from obstacle)
  128   = inscribed radius (robot definitely collides)
  254   = lethal (obstacle)
  255   = no information
"""
import numpy as np
import math
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION, ORIGIN_X, ORIGIN_Y

# Cost constants
COST_FREE = 0
COST_INSCRIBED = 128
COST_LETHAL = 254
COST_UNKNOWN = 255

# Robot parameters
ROBOT_RADIUS = 0.35  # meters (actual robot radius, used for collision checks)
INSCRIBED_RADIUS = 0.25  # inscribed radius for costmap inflation
# NOTE: INSCRIBED_RADIUS < ROBOT_RADIUS intentionally. The doorway is 2m wide
# but wall_divide thickness (0.1m) + inflation makes the passable corridor
# narrow. With INSCRIBED_RADIUS=0.35, a robot at (1.35, -0.40) is exactly
# 0.35m from wall_divide_2 (y=[-0.05,0.05]) and hits COST_INSCRIBED=128,
# which makes DWA reject ALL trajectories from that position (collision
# check: cost >= COST_INSCRIBED). The robot gets stuck and RECOVER pushes
# it back toward open space → doorway oscillation. Using 0.25m gives enough
# clearance for the robot to navigate through the doorway while still
# keeping a safety margin (CoppeliaSim uses position control, no real
# physics collision).
INFLATION_RADIUS = 0.55  # how far to inflate (was 0.7, reduced to match)

# Convert to grid cells
INFLATION_CELLS = int(INFLATION_RADIUS / GRID_RESOLUTION)
INSCRIBED_CELLS = int(INSCRIBED_RADIUS / GRID_RESOLUTION)

# Decay factor for inflation cost
COST_SCALE_FACTOR = 2.5  # higher = sharper falloff


class Costmap:
    """Layered costmap matching Nav2 architecture."""

    def __init__(self):
        self.width = GRID_W
        self.height = GRID_H
        self.resolution = GRID_RESOLUTION
        self.origin_x = ORIGIN_X
        self.origin_y = ORIGIN_Y

        # Master costmap (combined layers)
        self.cost = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
        # Static layer (from occupancy grid)
        self.static_cost = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
        # Dynamic obstacle layer (from real-time LiDAR)
        self.obstacle_cost = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
        # Unknown cell mask (cells not yet observed by LiDAR)
        self.unknown_mask = np.zeros((GRID_H, GRID_W), dtype=bool)
        # Pre-computed inflation kernel
        self._inflation_kernel = self._build_inflation_kernel()
        # Dirty-flag bookkeeping for update_static (avoid per-frame full rebuild)
        self._last_occupied_count = 0
        self._last_static_update_frame = -100
        self._static_dirty = True

    def _build_inflation_kernel(self):
        """Pre-compute inflation cost for each cell distance."""
        size = 2 * INFLATION_CELLS + 1
        kernel = np.zeros((size, size), dtype=np.float32)
        cx = cy = INFLATION_CELLS
        for i in range(size):
            for j in range(size):
                dx = j - cx
                dy = i - cy
                dist = math.sqrt(dx * dx + dy * dy) * self.resolution
                if dist <= 0:
                    kernel[i, j] = COST_LETHAL
                elif dist <= INSCRIBED_RADIUS:
                    kernel[i, j] = COST_INSCRIBED
                elif dist <= INFLATION_RADIUS:
                    # Exponential decay
                    factor = math.exp(
                        -COST_SCALE_FACTOR * (dist - INSCRIBED_RADIUS))
                    kernel[i, j] = COST_INSCRIBED * factor
                else:
                    kernel[i, j] = 0
        return kernel

    def _inflate(self, obstacle_mask):
        """Inflate a binary obstacle mask using the pre-computed kernel."""
        inflated = np.zeros_like(self.cost, dtype=np.float32)
        oy, ox = np.where(obstacle_mask)
        half = INFLATION_CELLS
        for gy, gx in zip(oy, ox):
            y0, y1 = max(0, gy - half), min(GRID_H, gy + half + 1)
            x0, x1 = max(0, gx - half), min(GRID_W, gx + half + 1)
            ky0, ky1 = half - (gy - y0), half + (y1 - gy)
            kx0, kx1 = half - (gx - x0), half + (x1 - gx)
            inflated[y0:y1, x0:x1] = np.maximum(
                inflated[y0:y1, x0:x1],
                self._inflation_kernel[ky0:ky1, kx0:kx1])
        return np.clip(inflated, 0, 254).astype(np.uint8)

    def update_static(self, grid, frame=0):
        """Update static layer from OccupancyGrid (with dirty flag for efficiency)."""
        occupied = grid.log_odds > 0.6
        occupied_count = int(occupied.sum())
        # Only rebuild if: (1) first time / explicitly dirty,
        # (2) occupied count changed significantly, or
        # (3) every 5 frames (safety refresh)
        count_changed = abs(occupied_count - self._last_occupied_count) > 3
        time_elapsed = frame - self._last_static_update_frame >= 2
        if not (self._static_dirty or count_changed or time_elapsed):
            return  # skip rebuild, current static_cost is still valid
        static = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
        # Occupied cells = lethal
        static[occupied] = COST_LETHAL
        # Inflate occupied obstacles (creates inflation gradient only around known walls)
        inflated = self._inflate(occupied)
        # Unknown cells = moderate penalty (prevents A* from planning through
        # unmapped walls). Applied AFTER inflation so unknown cells don't get
        # inflated to lethal. Cost 80 < INSCRIBED(128) so still traversable
        # but strongly discouraged vs free cells (cost 0).
        unknown = np.abs(grid.log_odds) < 0.3
        self.unknown_mask = unknown
        static[unknown] = 80
        # Free cells = 0 cost
        # Merge: take max of inflated obstacles and unknown penalty
        self.static_cost = np.maximum(inflated, static)
        self._merge()
        self._last_occupied_count = occupied_count
        self._last_static_update_frame = frame
        self._static_dirty = False

    def update_obstacles(self, rx, ry, angles, distances, max_range=8.0):
        """Update dynamic obstacle layer from real-time LiDAR scan."""
        # Decay previous obstacles (dynamic layer is short-term)
        self.obstacle_cost = np.maximum(
            self.obstacle_cost.astype(np.int16) - 10, 0
        ).astype(np.uint8)

        # Mark current LiDAR hits as obstacles
        obstacle_mask = np.zeros((GRID_H, GRID_W), dtype=bool)
        for angle, dist in zip(angles, distances):
            if dist >= max_range or not np.isfinite(dist) or dist < 0.1:
                continue
            hx = rx + dist * math.cos(angle)
            hy = ry + dist * math.sin(angle)
            gx = int((hx - self.origin_x) / self.resolution)
            gy = int((hy - self.origin_y) / self.resolution)
            if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                obstacle_mask[gy, gx] = True

        # Inflate and merge into obstacle layer
        inflated = self._inflate(obstacle_mask)
        self.obstacle_cost = np.maximum(self.obstacle_cost, inflated)
        self._merge()

    def _merge(self):
        """Merge all layers into master costmap."""
        self.cost = np.maximum(self.static_cost, self.obstacle_cost)

    # --- Cost queries ---
    def get_cost(self, x, y):
        """Get cost at world position (x, y)."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            return self.cost[gy, gx]
        return COST_LETHAL  # out of bounds = lethal

    def get_cost_grid(self, gx, gy):
        """Get cost at grid indices."""
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            return self.cost[gy, gx]
        return COST_LETHAL

    def is_unknown_grid(self, gx, gy):
        """True if cell has not been observed by LiDAR (unknown)."""
        if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
            return self.unknown_mask[gy, gx]
        return False

    def is_lethal(self, x, y):
        """True if position is in or too close to an obstacle."""
        return self.get_cost(x, y) >= COST_INSCRIBED

    def is_safe(self, x, y, threshold=60):
        """True if position has cost below threshold (safe to traverse)."""
        return self.get_cost(x, y) < threshold

    def world_to_grid(self, x, y):
        return int((x - self.origin_x) / self.resolution), \
               int((y - self.origin_y) / self.resolution)

    def grid_to_world(self, gx, gy):
        return self.origin_x + (gx + 0.5) * self.resolution, \
               self.origin_y + (gy + 0.5) * self.resolution
