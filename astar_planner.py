#!/usr/bin/env python3
"""
A* Global Path Planner
======================
8-connected A* on the costmap with octile distance heuristic.
Includes path smoothing via line-of-sight shortcutting.

Features:
  - 8-connected movement (orthogonal + diagonal)
  - Costmap-aware: penalizes paths near obstacles
  - Octile distance heuristic (admissible for 8-connected)
  - Path smoothing: shortcut waypoints with clear line-of-sight
  - Goal tolerance: snaps to nearest free cell if goal is blocked
"""
import numpy as np
import math
import heapq
import time
from costmap import Costmap, COST_INSCRIBED, COST_LETHAL, GRID_W, GRID_H

# Planning threshold: A* treats cells with cost >= 110 as blocked.
# This is lower than COST_INSCRIBED (128) so A* avoids narrow gaps
# where inflation is high but still technically traversable.
PLAN_BLOCKED = 110

# Max nodes expanded before A* gives up (prevents runaway search on large grids)
# v2.9: 5000→3000→2000 — Run 5 showed astar_avg=90ms even with 3000 nodes.
# 2000 is sufficient for reachable goals on a 10x8m map (typically <1500 nodes)
# while cutting unreachable-goal search time further.
MAX_NODES = 2000


class AStarPlanner:
    """A* path planner on costmap."""

    def __init__(self, costmap):
        self.costmap = costmap
        # 8-connected neighbors: (dx, dy, move_cost)
        self.neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

    def _heuristic(self, gx, gy, tx, ty):
        """Octile distance (admissible for 8-connected grid)."""
        dx = abs(gx - tx)
        dy = abs(gy - ty)
        return (dx + dy) + (1.414 - 2) * min(dx, dy)

    def _cell_cost(self, gx, gy):
        """Get traversal cost for a cell (higher = more expensive).

        Uses quadratic penalty so high-cost cells (narrow gaps near
        obstacles) are strongly discouraged. This makes A* prefer longer
        paths through open space over shortcuts through tight passages
        that the DWA can't follow.

        Tuned v2.3: increased penalty coefficient (8.0→12.0) so paths stay
        further from walls. This reduces RECOVER triggers caused by the
        robot getting stuck in inflation zones near obstacles.
        """
        c = self.costmap.get_cost_grid(gx, gy)
        if c >= PLAN_BLOCKED:
            return float('inf')  # blocked
        # Quadratic penalty: free=1, cost 50→2.7, cost 100→11.0, cost 109→13.0
        return 1.0 + (c / PLAN_BLOCKED) ** 2 * 12.0

    def _is_traversable(self, gx, gy):
        """Cell can be entered (observed free + cost below threshold).

        Blocks:
          - Cells with cost >= PLAN_BLOCKED (narrow gaps, near-obstacle)
          - UNKNOWN cells (not yet observed by LiDAR) — prevents A* from
            planning through unobserved obstacles (e.g. sofa's unobserved
            south face). The robot must observe cells via LiDAR before
            planning through them.
        """
        c = self.costmap.get_cost_grid(gx, gy)
        if c >= PLAN_BLOCKED:
            return False
        if self.costmap.is_unknown_grid(gx, gy):
            return False
        return True

    def _find_nearest_free(self, gx, gy, max_radius=10):
        """Find nearest traversable cell to (gx, gy) using BFS."""
        if self._is_traversable(gx, gy):
            return gx, gy
        for r in range(1, max_radius):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        if self._is_traversable(nx, ny):
                            return nx, ny
        return None, None

    def plan(self, start_x, start_y, goal_x, goal_y):
        """Plan path from start to goal (world coordinates).

        Returns list of (wx, wy) waypoints (excluding start, including goal),
        or None if no path found.
        """
        start_time = time.time()
        iterations = 0
        sgx, sgy = self.costmap.world_to_grid(start_x, start_y)
        ggx, ggy = self.costmap.world_to_grid(goal_x, goal_y)

        # Snap start/goal to nearest free cell if blocked
        sgx, sgy = self._find_nearest_free(sgx, sgy)
        if sgx is None:
            return None
        ggx, ggy = self._find_nearest_free(ggx, ggy)
        if ggx is None:
            return None

        if (sgx, sgy) == (ggx, ggy):
            return [(goal_x, goal_y)]

        # A* search
        open_heap = []
        counter = 0  # tie-breaker for heap
        g_score = {(sgx, sgy): 0.0}
        came_from = {}
        visited = set()

        h_start = self._heuristic(sgx, sgy, ggx, ggy)
        heapq.heappush(open_heap, (h_start, counter, (sgx, sgy)))

        while open_heap:
            if time.time() - start_time > 0.1:  # 100ms 超时
                return None
            iterations += 1
            if iterations > MAX_NODES:
                return None
            _, _, (cx, cy) = heapq.heappop(open_heap)
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))

            if (cx, cy) == (ggx, ggy):
                # Reconstruct path
                path = []
                cur = (cx, cy)
                while cur is not None:
                    wx, wy = self.costmap.grid_to_world(cur[0], cur[1])
                    path.append((wx, wy))
                    cur = came_from.get(cur)
                path.reverse()
                # Smooth path
                path = self._smooth_path(path)
                return path[1:]  # exclude start

            base_cost = g_score[(cx, cy)]
            for dx, dy, move_cost in self.neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < GRID_W and 0 <= ny < GRID_H):
                    continue
                if (nx, ny) in visited:
                    continue
                if not self._is_traversable(nx, ny):
                    continue
                cell_cost = self._cell_cost(nx, ny)
                if cell_cost == float('inf'):
                    continue
                # Diagonal move: check both orthogonal cells aren't blocked
                if dx != 0 and dy != 0:
                    if not self._is_traversable(cx + dx, cy) or \
                       not self._is_traversable(cx, cy + dy):
                        continue
                new_g = base_cost + move_cost * cell_cost
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    came_from[(nx, ny)] = (cx, cy)
                    h = self._heuristic(nx, ny, ggx, ggy)
                    counter += 1
                    heapq.heappush(open_heap, (new_g + h, counter, (nx, ny)))

        return None  # no path found

    def _smooth_path(self, path):
        """Smooth path by removing redundant waypoints (line-of-sight test).

        Uses Bresenham to check if two waypoints have clear line-of-sight
        (all cells traversable). If so, intermediate waypoints are removed.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Find farthest reachable waypoint
            j = len(path) - 1
            while j > i + 1:
                if self._has_line_of_sight(path[i], path[j]):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j

        return smoothed

    def _has_line_of_sight(self, p1, p2):
        """Check if line from p1 to p2 is clear (all cells traversable).

        Does NOT shortcut through unknown cells — this prevents path smoothing
        from cutting through unmapped walls.
        """
        x0, y0 = self.costmap.world_to_grid(p1[0], p1[1])
        x1, y1 = self.costmap.world_to_grid(p2[0], p2[1])
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if not self._is_traversable(x, y):
                return False
            # Don't shortcut through unknown cells (unmapped areas)
            if self.costmap.is_unknown_grid(x, y):
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
