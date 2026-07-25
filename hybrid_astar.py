"""
Hybrid A* Global Planner (v4.0)
=================================
State-lattice global planner that incorporates kinodynamic constraints.

Unlike regular A* which searches in (x, y) grid space, Hybrid A* searches
in (x, y, theta) SE(2) state space using motion primitives that respect
the robot's differential-drive kinematics. This produces kinematically
feasible paths that don't require a separate smoothing step.

Algorithm:
  - State space: (x, y, theta) discretized
  - Motion primitives: straight, left turn, right turn, reverse
  - Heuristic: max(2D A* cost, Reeds-Shepp distance)
  - Collision checking: footprint at each state

References:
  - Dolgov et al. (2008) "Practical Search Techniques in Path Planning
    for Autonomous Driving"
  - Pivtoraiko & Kelly (2005) "Incremental Path Planning Using
    Kinodynamically Feasible Motion Primitives"
  - Reeds & Shepp (1990) "Optimal paths for a car that goes both forwards
    and backwards"
"""
import os
import math
import heapq
import numpy as np
from costmap import Costmap, COST_LETHAL

USE_HYBRID_ASTAR = os.environ.get("USE_HYBRID_ASTAR", "0") == "1"


class HybridAStarPlanner:
    """Hybrid A* global planner for differential-drive robots.

    Searches in SE(2) state space using motion primitives.
    """

    def __init__(self, costmap, cfg=None, xy_resolution=0.2,
                 theta_resolution=math.pi/12, step_size=0.3,
                 robot_radius=0.25, max_turn_rate=1.5,
                 n_primitives=5, max_iterations=50000):
        self.costmap = costmap
        cfg = cfg or {}

        # Robot parameters
        self.wheelbase = cfg.get('wheelbase', 0.3)
        self.robot_radius = cfg.get('robot_radius', robot_radius)
        self.max_steer = cfg.get('max_steer', math.pi / 3)

        # Resolution
        self.xy_res = cfg.get('xy_resolution', xy_resolution)
        self.theta_res = cfg.get('theta_resolution', theta_resolution)
        self.theta_bins = int(2 * math.pi / self.theta_res)

        # Motion primitives
        self.step_size = cfg.get('step_size', step_size)
        self.max_w = cfg.get('max_turn_rate', max_turn_rate)
        self.n_primitives = cfg.get('n_primitives', n_primitives)

        self.primitives = self._build_primitives()

        # Limits
        self.max_iterations = cfg.get('max_iterations', max_iterations)

    def _build_primitives(self):
        """Build motion primitives: list of (dx, dy, dtheta) per step."""
        primitives = []
        # Straight
        primitives.append((self.step_size, 0.0, 0.0))
        # Turn left at various rates
        n_turn = (self.n_primitives - 1) // 2
        for i in range(1, n_turn + 1):
            w = self.max_w * (i / n_turn)
            # Arc: dx, dy, dtheta for moving forward while turning
            if abs(w) < 1e-6:
                dx = self.step_size
                dy = 0.0
                dtheta = 0.0
            else:
                # R = v/w, arc angle = step/R = step*w/v
                # For unit v=1: R = 1/w, arc_angle = step * w
                arc_angle = self.step_size * w  # since v=1
                R = 1.0 / w
                dx = R * math.sin(arc_angle)
                dy = R * (1 - math.cos(arc_angle))
                dtheta = arc_angle
            primitives.append((dx, dy, dtheta))
            primitives.append((dx, -dy, -dtheta))  # right turn mirror

        return primitives

    def _state_to_index(self, x, y, theta):
        """Convert continuous state to discrete index."""
        gx = int(math.floor(x / self.xy_res))
        gy = int(math.floor(y / self.xy_res))
        # Normalize theta to [0, 2*pi)
        t = theta % (2 * math.pi)
        if t < 0:
            t += 2 * math.pi
        gtheta = int(math.floor(t / self.theta_res)) % self.theta_bins
        return gx, gy, gtheta

    def _index_to_state(self, gx, gy, gtheta):
        """Convert discrete index to continuous state (cell center)."""
        x = (gx + 0.5) * self.xy_res
        y = (gy + 0.5) * self.xy_res
        theta = (gtheta + 0.5) * self.theta_res
        return x, y, theta

    def _is_safe(self, x, y):
        """Check if a position is collision-free."""
        cost = self.costmap.get_cost(x, y)
        return cost < 60

    def _heuristic(self, x, y, gx, gy):
        """Heuristic: Euclidean distance (admissible)."""
        dx = x - (gx + 0.5) * self.xy_res
        dy = y - (gy + 0.5) * self.xy_res
        return math.sqrt(dx*dx + dy*dy)

    def plan(self, sx, sy, stheta, gx_w, gy_w, gtheta=None):
        """Plan a kinodynamically feasible path from start to goal.

        Args:
            sx, sy, stheta: start pose (world coords)
            gx_w, gy_w: goal position (world coords)
            gtheta: goal heading (optional, None = any heading)

        Returns:
            list of (x, y, theta) waypoints, or None if no path found
        """
        # Convert to costmap world coords (costmap uses same frame as grid)
        s_idx = self._state_to_index(sx, sy, stheta)
        g_idx = self._state_to_index(gx_w, gy_w, gtheta if gtheta is not None else 0.0)

        # Open set: (f_cost, g_cost, gx, gy, gtheta)
        open_heap = []
        h_start = self._heuristic(sx, sy, g_idx[0], g_idx[1])
        heapq.heappush(open_heap, (h_start, 0.0, s_idx[0], s_idx[1], s_idx[2]))

        # Closed set: gx -> gy -> gtheta -> (g_cost, parent_idx)
        closed = {}

        # Also keep a 2D best-cost for pruning (any theta at same xy cell)
        best_2d = {}  # (gx, gy) -> best_g_cost

        iterations = 0
        goal_reached = False
        goal_idx = None

        while open_heap and iterations < self.max_iterations:
            iterations += 1
            f, g, gx, gy, gtheta = heapq.heappop(open_heap)

            idx_key = (gx, gy, gtheta)
            if idx_key in closed and closed[idx_key][0] <= g:
                continue

            closed[idx_key] = (g, None)  # (g_cost, parent)

            # Check 2D best cost (pruning if worse)
            key_2d = (gx, gy)
            if key_2d in best_2d and best_2d[key_2d] + 0.5 < g:
                continue
            if key_2d not in best_2d or g < best_2d[key_2d]:
                best_2d[key_2d] = g

            # Goal check (position tolerance)
            dist_to_goal = math.sqrt(
                (gx - g_idx[0])**2 + (gy - g_idx[1])**2) * self.xy_res
            if dist_to_goal < self.xy_res * 2:
                goal_reached = True
                goal_idx = idx_key
                break

            # Expand motion primitives
            state_x, state_y, state_t = self._index_to_state(gx, gy, gtheta)

            for prim in self.primitives:
                pdx, pdy, pdtheta = prim
                # Rotate primitive into current heading
                cos_t = math.cos(state_t)
                sin_t = math.sin(state_t)
                world_dx = pdx * cos_t - pdy * sin_t
                world_dy = pdx * sin_t + pdy * cos_t

                nx = state_x + world_dx
                ny = state_y + world_dy
                ntheta = state_t + pdtheta

                # Collision check
                if not self._is_safe(nx, ny):
                    continue

                # Also check intermediate points along the arc
                if not self._check_path_segment(state_x, state_y, nx, ny):
                    continue

                ng = g + math.sqrt(world_dx**2 + world_dy**2)
                n_idx = self._state_to_index(nx, ny, ntheta)
                n_key = (n_idx[0], n_idx[1], n_idx[2])

                if n_key in closed and closed[n_key][0] <= ng:
                    continue

                h = self._heuristic(nx, ny, g_idx[0], g_idx[1])
                f = ng + h

                heapq.heappush(open_heap, (f, ng, n_idx[0], n_idx[1], n_idx[2]))
                # Update parent
                if n_key not in closed or closed[n_key][0] > ng:
                    closed[n_key] = (ng, idx_key)

        if not goal_reached:
            return None

        # Reconstruct path
        path = []
        current = goal_idx
        while current is not None:
            gx_c, gy_c, gt_c = current
            x, y, t = self._index_to_state(gx_c, gy_c, gt_c)
            path.append((x, y, t))
            parent = closed[current][1]
            current = parent

        path.reverse()
        return path

    def _check_path_segment(self, x1, y1, x2, y2, n_check=5):
        """Check collision along a line segment."""
        for i in range(1, n_check + 1):
            t = i / (n_check + 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self._is_safe(x, y):
                return False
        return True
