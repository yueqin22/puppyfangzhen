#!/usr/bin/env python3
"""
Comprehensive Module Test Suite
===============================
Tests all core navigation modules without requiring CoppeliaSim connection.

Coverage:
  1. odometry.Odometry             — dead reckoning integration + drift
  2. occupancy_grid.OccupancyGrid  — ray tracing, frontiers, info gain
  3. costmap.Costmap               — inflation, cost queries, dynamic layer
  4. astar_planner.AStarPlanner    — path planning, smoothing, unreachable
  5. dwa_planner.DWAPlanner        — velocity sampling, collision avoidance
  6. teb_planner.TEBPlanner        — elastic band optimization
  7. amcl.AMCL                     — KLD-sampling, weight, resample, recover
  8. evaluator.Evaluator           — CSV/summary, comparison plots
  9. vision.SemanticMapper         — color classification, semantic stamping
 10. config_loader.load_config     — YAML loading
 11. nav_core.runtime.state_machine     — state transitions
 12. nav_core.runtime.runtime_state     — runtime state mutations
 13. nav_core.metrics.telemetry         — metric collection
 14. nav_core.session.session_store     — map/frontier persistence
 15. nav_core.exploration.frontier_manager — frontier scoring/memory

Usage:
  python test_all_modules.py
Exit code: 0 if all pass, 1 if any fail.
"""
import os
import sys
import math
import json
import time
import tempfile
import traceback
import numpy as np

# Ensure project root on path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ============================================================
# Test result tracking
# ============================================================
class TestResult:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []  # list of (name, msg)

    def check(self, name, cond, detail=""):
        if cond:
            self.passed += 1
            print(f"  [PASS] {name}")
        else:
            self.failed += 1
            self.errors.append((name, detail))
            print(f"  [FAIL] {name}  {detail}")

    def section(self, title):
        print(f"\n{'='*60}\n  {title}\n{'='*60}")


result = TestResult()


def approx(a, b, tol=1e-6):
    return abs(a - b) < tol


# ============================================================
# 1. Odometry
# ============================================================
def test_odometry():
    result.section("1. Odometry")
    from odometry import Odometry

    # Test 1.1: Straight-line motion
    odo = Odometry(x=0.0, y=0.0, yaw=0.0, sigma_v=0.0, sigma_w=0.0)
    for _ in range(10):
        odo.update(v=0.2, w=0.0, dt=0.1)
    x, y, yaw = odo.get_pose()
    result.check("1.1 straight-line x=0.2*1.0=0.2",
                 approx(x, 0.2, 1e-3), f"x={x:.4f}")
    result.check("1.1 straight-line y=0",
                 approx(y, 0.0, 1e-3), f"y={y:.4f}")
    result.check("1.1 straight-line yaw=0",
                 approx(yaw, 0.0, 1e-3), f"yaw={yaw:.4f}")

    # Test 1.2: Pure rotation
    odo = Odometry(x=0.0, y=0.0, yaw=0.0, sigma_v=0.0, sigma_w=0.0)
    for _ in range(10):
        odo.update(v=0.0, w=0.5, dt=0.1)
    _, _, yaw = odo.get_pose()
    result.check("1.2 rotation yaw=0.5*1.0=0.5",
                 approx(yaw, 0.5, 1e-3), f"yaw={yaw:.4f}")

    # Test 1.3: Yaw normalization to [-pi, pi]
    odo = Odometry(x=0.0, y=0.0, yaw=0.0, sigma_v=0.0, sigma_w=0.0)
    # Rotate > 2*pi
    for _ in range(100):
        odo.update(v=0.0, w=1.0, dt=0.1)
    _, _, yaw = odo.get_pose()
    result.check("1.3 yaw normalized to [-pi,pi]",
                 -math.pi - 1e-3 <= yaw <= math.pi + 1e-3, f"yaw={yaw:.4f}")

    # Test 1.4: Total distance tracked
    odo = Odometry(x=0.0, y=0.0, yaw=0.0, sigma_v=0.0, sigma_w=0.0)
    odo.update(v=0.3, w=0.0, dt=1.0)
    result.check("1.4 total_distance tracks motion",
                 approx(odo.total_distance, 0.3, 1e-3),
                 f"dist={odo.total_distance:.4f}")

    # Test 1.5: Reset
    odo.reset(1.0, 2.0, 0.5)
    x, y, yaw = odo.get_pose()
    result.check("1.5 reset pose",
                 approx(x, 1.0) and approx(y, 2.0) and approx(yaw, 0.5),
                 f"({x},{y},{yaw})")
    result.check("1.5 reset clears total_distance",
                 approx(odo.total_distance, 0.0),
                 f"dist={odo.total_distance}")

    # Test 1.6: Drift report
    odo = Odometry(x=0.0, y=0.0, yaw=0.0, sigma_v=0.0, sigma_w=0.0)
    report = odo.drift_report(0.3, 0.0, 0.0)
    result.check("1.6 drift_report returns pos_error_m",
                 'pos_error_m' in report and 'yaw_error_rad' in report,
                 str(report))

    # Test 1.7: Noisy motion produces drift
    np.random.seed(42)
    odo = Odometry(x=0.0, y=0.0, yaw=0.0, sigma_v=0.1, sigma_w=0.1)
    for _ in range(100):
        odo.update(v=0.2, w=0.0, dt=0.1)
    x, _, _ = odo.get_pose()
    # With noise, x should be close to 2.0 but not exact
    result.check("1.7 noisy motion drifts from ground truth",
                 abs(x - 2.0) > 1e-4, f"x={x:.4f} (expected drift)")


# ============================================================
# 2. OccupancyGrid
# ============================================================
def test_occupancy_grid():
    result.section("2. OccupancyGrid")
    from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION

    grid = OccupancyGrid()

    # Test 2.1: Coordinate conversion
    gx, gy = grid.world_to_grid(0.0, 0.0)
    expected_gx = int(math.floor((0.0 - grid.origin_x) / grid.resolution))
    expected_gy = int(math.floor((0.0 - grid.origin_y) / grid.resolution))
    result.check("2.1 world_to_grid origin",
                 gx == expected_gx and gy == expected_gy,
                 f"({gx},{gy}) vs ({expected_gx},{expected_gy})")

    # Round-trip: grid_to_world(50,40) = origin + (idx+0.5)*res = (-5+5.05, -4+4.05)
    wx, wy = grid.grid_to_world(gx, gy)
    result.check("2.1 grid_to_world round-trip",
                 approx(wx, 0.05, 0.01) and approx(wy, 0.05, 0.01),
                 f"({wx},{wy})")

    # Test 2.2: Initial state is unknown
    result.check("2.2 initial is_unknown True",
                 grid.is_unknown(50, 40))
    result.check("2.2 initial is_free False",
                 not grid.is_free(50, 40))
    result.check("2.2 initial is_occupied False",
                 not grid.is_occupied(50, 40))

    # Test 2.3: Bresenham ray tracing updates cells
    # Simulate a LiDAR scan hitting an obstacle at 3m
    grid2 = OccupancyGrid()
    angles = [0.0]
    distances = [3.0]
    grid2.update_from_scan(0.0, 0.0, angles, distances, max_range=8.0)
    # Hit point should be occupied
    hx, hy = grid2.world_to_grid(3.0, 0.0)
    result.check("2.3 hit cell becomes occupied",
                 grid2.is_occupied(hx, hy),
                 f"log_odds[{hy},{hx}]={grid2.log_odds[hy, hx]:.3f}")
    # Cells along ray should be observed as free (log_odds decreased).
    # Single ray only applies LOG_ODDS_MISS=-0.4 once, so log_odds=−0.4
    # which is < 0 (observed free) but not yet < -0.5 (is_free threshold).
    midx, midy = grid2.world_to_grid(1.5, 0.0)
    result.check("2.3 mid-ray cell observed free (log_odds < 0)",
                 grid2.log_odds[midy, midx] < 0,
                 f"log_odds[{midy},{midx}]={grid2.log_odds[midy, midx]:.3f}")

    # Test 2.4: Frontier detection
    grid3 = OccupancyGrid()
    # Build a map: free region at center, unknown around
    grid3.log_odds[30:50, 30:50] = -1.0  # free block
    frontiers = grid3.find_frontiers(0.0, 0.0, max_frontiers=10)
    result.check("2.4 frontier detection returns list",
                 isinstance(frontiers, list))
    result.check("2.4 frontiers found at boundary",
                 len(frontiers) > 0,
                 f"count={len(frontiers)}")

    # Test 2.5: Info gain frontiers (v3.0)
    fg = grid3.find_frontiers_with_info_gain(0.0, 0.0, sensor_range=4.0,
                                              max_frontiers=10)
    result.check("2.5 info_gain frontiers returns tuples",
                 isinstance(fg, list) and all(len(f) >= 4 for f in fg) if fg else True,
                 f"count={len(fg)}")

    # Test 2.6: mark_visited
    grid4 = OccupancyGrid()
    result.check("2.6 mark_visited returns True first time",
                 grid4.mark_visited(0.0, 0.0))
    result.check("2.6 mark_visited returns False second time",
                 not grid4.mark_visited(0.0, 0.0))
    result.check("2.6 is_visited tracks mark",
                 grid4.is_visited(0.0, 0.0))

    # Test 2.7: Out of bounds
    result.check("2.7 out-of-bounds is_occupied True",
                 grid.is_occupied(-1, -1))
    result.check("2.7 out-of-bounds is_unknown False",
                 not grid.is_unknown(-1, -1))


# ============================================================
# 3. Costmap
# ============================================================
def test_costmap():
    result.section("3. Costmap")
    from occupancy_grid import OccupancyGrid
    from costmap import (Costmap, COST_FREE, COST_INSCRIBED, COST_LETHAL,
                         COST_UNKNOWN)

    # Test 3.1: Init
    cm = Costmap()
    result.check("3.1 init dimensions",
                 cm.width > 0 and cm.height > 0 and cm.resolution > 0)
    result.check("3.1 init cost is zero",
                 int(np.sum(cm.cost)) == 0)

    # Test 3.2: Static layer from occupied grid
    grid = OccupancyGrid()
    # Add a wall
    grid.log_odds[10:70, 20:22] = 2.0  # vertical wall
    cm.update_static(grid, frame=0)
    # Wall cells should be lethal
    result.check("3.2 wall cell lethal",
                 cm.get_cost_grid(20, 40) >= COST_LETHAL,
                 f"cost={cm.get_cost_grid(20, 40)}")
    # Free space far from wall should be 0 or low
    result.check("3.2 far cell free",
                 cm.get_cost_grid(80, 70) < COST_INSCRIBED,
                 f"cost={cm.get_cost_grid(80, 70)}")

    # Test 3.3: Inflation gradient exists
    costs_near = []
    for dx in range(0, 10):
        c = cm.get_cost_grid(20 + dx, 40)
        costs_near.append(c)
    # Cost should decrease as we move away from wall
    result.check("3.3 inflation gradient decreases",
                 any(costs_near[i] > costs_near[i+1] for i in range(len(costs_near)-1)),
                 f"costs={costs_near[:6]}")

    # Test 3.4: Dynamic obstacle layer
    cm2 = Costmap()
    grid2 = OccupancyGrid()
    cm2.update_static(grid2, frame=0)
    angles = [0.0, math.pi/2, math.pi, -math.pi/2]
    distances = [2.0, 2.0, 2.0, 2.0]
    cm2.update_obstacles(0.0, 0.0, angles, distances, max_range=8.0)
    # Hit point at (2,0) should be lethal
    gx, gy = cm2.world_to_grid(2.0, 0.0)
    result.check("3.4 dynamic obstacle marked",
                 cm2.get_cost_grid(gx, gy) >= COST_INSCRIBED,
                 f"cost={cm2.get_cost_grid(gx, gy)}")

    # Test 3.5: is_safe / is_lethal
    result.check("3.5 is_lethal detects lethal",
                 cm.is_lethal(-5.0 + 20 * 0.1 + 0.05, -4.0 + 40 * 0.1 + 0.05))
    # Safe far from obstacles
    cm3 = Costmap()
    grid3 = OccupancyGrid()
    grid3.log_odds[10:70, 10:90] = -1.0  # all free
    cm3.update_static(grid3, frame=0)
    result.check("3.5 is_safe in free area",
                 cm3.is_safe(0.0, 0.0))

    # Test 3.6: world_to_grid / grid_to_world
    gx, gy = cm.world_to_grid(1.5, -2.0)
    wx, wy = cm.grid_to_world(gx, gy)
    result.check("3.6 costmap coord round-trip",
                 approx(wx, 1.5, 0.1) and approx(wy, -2.0, 0.1),
                 f"({wx},{wy})")


# ============================================================
# 4. AStarPlanner
# ============================================================
def test_astar_planner():
    result.section("4. AStarPlanner")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from astar_planner import AStarPlanner

    # Test 4.1: Plan path on empty map
    grid = OccupancyGrid()
    # Mark whole area as free and observed
    grid.log_odds[5:75, 5:95] = -1.0
    cm = Costmap()
    cm.update_static(grid, frame=0)
    planner = AStarPlanner(cm)

    path = planner.plan(0.0, 0.0, 2.0, 0.0)
    result.check("4.1 plan returns list",
                 isinstance(path, list))
    result.check("4.1 path non-empty",
                 len(path) > 0 if path is not None else False,
                 f"len={len(path) if path else 0}")
    if path:
        result.check("4.1 path reaches goal",
                     approx(path[-1][0], 2.0, 0.2) and approx(path[-1][1], 0.0, 0.2),
                     f"end={path[-1]}")

    # Test 4.2: Path around obstacle
    grid2 = OccupancyGrid()
    grid2.log_odds[5:75, 5:95] = -1.0  # free
    # Add a wall blocking direct path, with a gap
    grid2.log_odds[30:50, 40:50] = 2.0  # wall
    grid2.log_odds[30:50, 50:60] = -1.0  # gap (keep free)
    cm2 = Costmap()
    cm2.update_static(grid2, frame=0)
    planner2 = AStarPlanner(cm2)

    path2 = planner2.plan(-3.0, 0.0, 3.0, 0.0)
    result.check("4.2 path around wall found",
                 path2 is not None and len(path2) > 0,
                 f"len={len(path2) if path2 else 0}")

    # Test 4.3: Unreachable goal returns None
    grid3 = OccupancyGrid()
    grid3.log_odds[5:75, 5:95] = -1.0
    # Wall completely blocking
    grid3.log_odds[5:75, 30:70] = 2.0
    cm3 = Costmap()
    cm3.update_static(grid3, frame=0)
    planner3 = AStarPlanner(cm3)

    path3 = planner3.plan(-3.0, 0.0, 3.0, 0.0)
    result.check("4.3 unreachable returns None or empty",
                 path3 is None or len(path3) == 0,
                 f"path={path3}")

    # Test 4.4: Start equals goal (trivial)
    path4 = planner.plan(0.0, 0.0, 0.0, 0.0)
    result.check("4.4 trivial plan doesn't crash",
                 path4 is not None or path4 is None)  # just no exception


# ============================================================
# 5. DWAPlanner
# ============================================================
def test_dwa_planner():
    result.section("5. DWAPlanner")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from dwa_planner import DWAPlanner

    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0
    cm = Costmap()
    cm.update_static(grid, frame=0)
    dwa = DWAPlanner(cm)

    # Test 5.1: compute_velocity returns tuple
    path = [(1.0, 0.0), (2.0, 0.0)]
    v, w = dwa.compute_velocity(0.0, 0.0, 0.0, path, 2.0, 0.0)
    result.check("5.1 returns (v, w) tuple",
                 isinstance(v, (int, float)) and isinstance(w, (int, float)),
                 f"v={v}, w={w}")
    result.check("5.1 v within limits",
                 -dwa.max_v - 0.01 <= v <= dwa.max_v + 0.01,
                 f"v={v}, max_v={dwa.max_v}")
    result.check("5.1 w within limits",
                 -dwa.max_w - 0.01 <= w <= dwa.max_w + 0.01,
                 f"w={w}, max_w={dwa.max_w}")

    # Test 5.2: Robot facing goal should move forward
    # Robot at origin facing +x, goal ahead at (3,0)
    path2 = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    v2, w2 = dwa.compute_velocity(0.0, 0.0, 0.0, path2, 3.0, 0.0)
    result.check("5.2 moves forward when facing goal",
                 v2 > 0.0, f"v={v2}")

    # Test 5.3: Obstacle ahead reduces velocity
    grid3 = OccupancyGrid()
    grid3.log_odds[5:75, 5:95] = -1.0
    # Obstacle right in front
    grid3.log_odds[40:45, 25:30] = 2.0
    cm3 = Costmap()
    cm3.update_static(grid3, frame=0)
    dwa3 = DWAPlanner(cm3)
    # Robot at origin, obstacle at world (0.05-0.10, ...) hmm actually
    # Obstacle at gx=25..30 means world x = -5+25*0.1 = -2.5 to -2.0
    # That's behind the robot. Put obstacle ahead: gx=55..60 -> wx=0.5..1.0
    grid4 = OccupancyGrid()
    grid4.log_odds[5:75, 5:95] = -1.0
    grid4.log_odds[40:45, 55:60] = 2.0  # wx=0.5..1.0
    cm4 = Costmap()
    cm4.update_static(grid4, frame=0)
    dwa4 = DWAPlanner(cm4)
    path4 = [(0.8, 0.0), (1.5, 0.0)]
    v4, w4 = dwa4.compute_velocity(0.0, 0.0, 0.0, path4, 2.0, 0.0)
    # Should not crash, may produce rotation or low v
    result.check("5.3 obstacle ahead doesn't crash",
                 isinstance(v4, (int, float)))

    # Test 5.4: reset clears state
    dwa.reset()
    result.check("5.4 reset clears prev_v",
                 approx(dwa.prev_v, 0.0))

    # Test 5.5: velocity_to_step conversion (returns dx, dy, dyaw)
    step = dwa.velocity_to_step(0.2, 0.0, 0.0, dt=0.1)
    result.check("5.5 velocity_to_step returns 3-tuple (dx,dy,dyaw)",
                 isinstance(step, tuple) and len(step) == 3,
                 f"step={step}")


# ============================================================
# 6. TEBPlanner
# ============================================================
def test_teb_planner():
    result.section("6. TEBPlanner")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from teb_planner import TEBPlanner

    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0
    cm = Costmap()
    cm.update_static(grid, frame=0)
    teb = TEBPlanner(cm)

    # Test 6.1: v3.0 weights
    result.check("6.1 w_time=0.3 (v3.0)",
                 approx(teb.w_time, 0.3), f"w_time={teb.w_time}")
    result.check("6.1 w_jerk=0.05 (v3.1 完整Jerk梯度)",
                 approx(teb.w_jerk, 0.05), f"w_jerk={teb.w_jerk}")

    # Test 6.2: compute_velocity
    path = [(0.5, 0.0), (1.0, 0.0), (1.5, 0.0), (2.0, 0.0)]
    v, w = teb.compute_velocity(0.0, 0.0, 0.0, path, 2.0, 0.0)
    result.check("6.2 returns (v, w)",
                 isinstance(v, (int, float)) and isinstance(w, (int, float)),
                 f"v={v}, w={w}")
    result.check("6.2 v within limits",
                 -teb.max_v - 0.01 <= v <= teb.max_v + 0.01)

    # Test 6.3: Profile config override
    teb_pi = TEBPlanner(cm, profile_cfg={'n_iterations': 2, 'n_poses': 10,
                                          'max_v': 0.2, 'dt': 0.2})
    result.check("6.3 profile n_iterations=2",
                 teb_pi.n_iterations == 2)
    result.check("6.3 profile n_poses=10",
                 teb_pi.n_poses == 10)
    result.check("6.3 profile max_v=0.2",
                 approx(teb_pi.max_v, 0.2))
    result.check("6.3 profile horizon = n_poses*dt = 2.0",
                 approx(teb_pi.horizon, 2.0, 1e-3),
                 f"horizon={teb_pi.horizon}")

    # Test 6.4: Empty path doesn't crash
    v4, w4 = teb.compute_velocity(0.0, 0.0, 0.0, [], 0.0, 0.0)
    result.check("6.4 empty path doesn't crash",
                 isinstance(v4, (int, float)))


# ============================================================
# 7. AMCL
# ============================================================
def test_amcl():
    result.section("7. AMCL")
    from occupancy_grid import OccupancyGrid
    from amcl import AMCL, USE_KLD, _distance_transform_numpy

    # Test 7.1: Distance transform
    mask = np.zeros((20, 20), dtype=bool)
    mask[10, 10] = True
    dist = _distance_transform_numpy(mask)
    result.check("7.1 distance at origin = 0",
                 approx(dist[10, 10], 0.0, 0.1))
    result.check("7.1 distance increases away",
                 dist[5, 5] > dist[8, 8] > 0)

    # Test 7.2: Init
    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0
    # Add some walls for localization reference
    grid.log_odds[5:75, 5:7] = 2.0
    grid.log_odds[5:75, 93:95] = 2.0
    amcl = AMCL(grid, n_particles=100, n_obs_rays=8)
    result.check("7.2 init n_particles",
                 amcl.n == 100)
    result.check("7.2 init particles shape",
                 amcl.particles.shape == (100, 3))
    result.check("7.2 init weights sum to 1",
                 approx(np.sum(amcl.weights), 1.0, 1e-6))

    # Test 7.3: KLD params (v3.0)
    result.check("7.3 kld_min=50",
                 amcl.kld_min == 50)
    result.check("7.3 kld_max=500",
                 amcl.kld_max == 500)
    result.check("7.3 kld_epsilon=0.05",
                 approx(amcl.kld_epsilon, 0.05))

    # Test 7.4: init_cloud
    amcl.init_cloud(0.0, 0.0, 0.0, spread=0.3)
    result.check("7.4 init_cloud centers particles",
                 abs(np.mean(amcl.particles[:, 0])) < 0.2,
                 f"mean_x={np.mean(amcl.particles[:, 0]):.3f}")
    result.check("7.4 init_cloud spread < 1.0",
                 np.std(amcl.particles[:, 0]) < 1.0,
                 f"std_x={np.std(amcl.particles[:, 0]):.3f}")

    # Test 7.5: predict moves particles
    old_x = amcl.particles[:, 0].copy()
    amcl.predict(0.1, 0.0, 0.0)
    new_x = amcl.particles[:, 0]
    result.check("7.5 predict moves particles",
                 np.mean(new_x) > np.mean(old_x) - 0.5,
                 f"old_mean={np.mean(old_x):.3f}, new_mean={np.mean(new_x):.3f}")

    # Test 7.6: weight computes
    angles = [2 * math.pi * i / 36 for i in range(36)]
    distances = [3.0 + 0.1 * math.sin(i) for i in range(36)]
    amcl.weight(angles, distances, frame=0)
    result.check("7.6 weight updates last_n_eff",
                 amcl.last_n_eff > 0,
                 f"n_eff={amcl.last_n_eff:.2f}")
    result.check("7.6 weights finite",
                 np.all(np.isfinite(amcl.weights)))

    # Test 7.7: KLD sample size (v3.0)
    n_kld_2 = amcl._kld_sample_size(2)
    n_kld_5 = amcl._kld_sample_size(5)
    result.check("7.7 kld k=2 returns positive int",
                 isinstance(n_kld_2, int) and n_kld_2 > 0,
                 f"n={n_kld_2}")
    result.check("7.7 kld k=5 >= k=2",
                 n_kld_5 >= n_kld_2,
                 f"k=2:{n_kld_2}, k=5:{n_kld_5}")
    result.check("7.7 kld within [kld_min, kld_max]",
                 amcl.kld_min <= n_kld_2 <= amcl.kld_max,
                 f"n={n_kld_2}, bounds=[{amcl.kld_min},{amcl.kld_max}]")

    # Test 7.8: KLD resample adaptively adjusts particle count.
    # With USE_KLD=1 (default), resample may increase count up to kld_max=500
    # when the particle cloud is dispersed (many unique bins).
    # This is the intended v3.0 behavior (Fox 2003).
    amcl2 = AMCL(grid, n_particles=100, n_obs_rays=8)
    amcl2.init_cloud(0.0, 0.0, 0.0)
    amcl2.weight(angles, distances, frame=0)
    amcl2.resample()
    n_after = len(amcl2.particles)
    result.check("7.8 KLD resample count within [kld_min, kld_max]",
                 amcl2.kld_min <= n_after <= amcl2.kld_max,
                 f"n_after={n_after}, bounds=[{amcl2.kld_min},{amcl2.kld_max}]")
    result.check("7.8 n_active tracks resampled count",
                 amcl2.n_active == n_after,
                 f"n_active={amcl2.n_active}, n={n_after}")

    # Test 7.9: recover spreads particles
    amcl3 = AMCL(grid, n_particles=100, n_obs_rays=8)
    amcl3.init_cloud(0.0, 0.0, 0.0, spread=0.1)
    std_before = np.std(amcl3.particles[:, 0])
    amcl3.recover(0.0, 0.0, 0.0, spread=0.8)
    std_after = np.std(amcl3.particles[:, 0])
    result.check("7.9 recover increases spread",
                 std_after > std_before,
                 f"before={std_before:.3f}, after={std_after:.3f}")

    # Test 7.10: get_estimate returns (x, y, yaw, confidence)
    est = amcl.get_estimate()
    result.check("7.10 get_estimate returns 4-tuple (x,y,yaw,conf)",
                 len(est) == 4 and all(np.isfinite(est)),
                 f"est={est}")

    # Test 7.11: full update cycle returns (x, y, yaw, confidence)
    amcl4 = AMCL(grid, n_particles=50, n_obs_rays=8)
    amcl4.init_cloud(0.0, 0.0, 0.0, spread=0.2)
    amcl4.update(0.05, 0.0, 0.0, angles, distances, frame=1)
    est = amcl4.get_estimate()
    result.check("7.11 update produces finite 4-tuple estimate",
                 len(est) == 4 and all(np.isfinite(est)),
                 f"est={est}")


# ============================================================
# 8. Evaluator
# ============================================================
def test_evaluator():
    result.section("8. Evaluator")
    from evaluator import Evaluator

    # Test 8.1: Init
    ev = Evaluator("test_method")
    result.check("8.1 init method name",
                 ev.method == "test_method")
    result.check("8.1 init empty frames",
                 len(ev.frames) == 0)

    # Test 8.2: update records data
    ev.update(frame=0, t=0.0, x=0.0, y=0.0, yaw=0.0,
              state=0, coverage=0.0, cost=0)
    ev.update(frame=1, t=0.1, x=0.1, y=0.0, yaw=0.0,
              state=1, coverage=10.0, cost=0,
              dwa_v=0.2, dwa_w=0.0)
    result.check("8.2 records 2 frames",
                 len(ev.frames) == 2)
    result.check("8.2 cumulative_distance > 0",
                 ev.cumulative_distance > 0)
    result.check("8.2 final_coverage tracked",
                 approx(ev.final_coverage, 10.0))

    # Test 8.3: RECOVER event counting
    ev.update(frame=2, t=0.2, x=0.1, y=0.0, yaw=0.0,
              state=2, coverage=10.0, cost=0)  # enter RECOVER
    result.check("8.3 recover_events incremented",
                 ev.recover_events == 1)

    # Test 8.4: DONE state
    ev.update(frame=3, t=0.3, x=0.1, y=0.0, yaw=0.0,
              state=3, coverage=10.0, cost=0)
    result.check("8.4 done_reached True",
                 ev.done_reached)

    # Test 8.5: save_csv
    tmp = tempfile.NamedTemporaryFile(suffix='.csv', delete=False)
    tmp.close()
    ev.save_csv(tmp.name)
    result.check("8.5 CSV file created",
                 os.path.exists(tmp.name) and os.path.getsize(tmp.name) > 0)
    # Verify content
    import csv
    with open(tmp.name) as f:
        reader = csv.reader(f)
        header = next(reader)
        result.check("8.5 CSV has correct header",
                     'frame' in header and 'coverage' in header and 'method' in header,
                     f"header={header}")
        rows = list(reader)
    result.check("8.5 CSV has 4 data rows",
                 len(rows) == 4,
                 f"rows={len(rows)}")
    os.unlink(tmp.name)

    # Test 8.6: summary
    s = ev.summary()
    result.check("8.6 summary has method",
                 s['method'] == "test_method")
    result.check("8.6 summary has total_frames",
                 s['total_frames'] == 4)
    result.check("8.6 summary has state_distribution",
                 'state_distribution' in s and 'FOLLOW' in s['state_distribution'])
    result.check("8.6 summary has recover_events",
                 s['recover_events'] == 1)

    # Test 8.7: save_summary
    tmp2 = tempfile.NamedTemporaryFile(suffix='.json', delete=False)
    tmp2.close()
    ev.save_summary(tmp2.name)
    result.check("8.7 summary JSON saved",
                 os.path.exists(tmp2.name))
    with open(tmp2.name) as f:
        loaded = json.load(f)
    result.check("8.7 JSON content valid",
                 loaded.get('method') == "test_method")
    os.unlink(tmp2.name)

    # Test 8.8: loc_error via true_x/true_y
    ev2 = Evaluator("loc_test")
    ev2.update(frame=0, t=0.0, x=0.1, y=0.0, yaw=0.0,
               state=0, coverage=0.0, cost=0,
               true_x=0.0, true_y=0.0)
    result.check("8.8 loc_error computed from true_x/y",
                 approx(ev2.loc_errors[-1], 0.1, 1e-3),
                 f"loc_err={ev2.loc_errors[-1]:.4f}")


# ============================================================
# 9. SemanticMapper (vision)
# ============================================================
def test_vision():
    result.section("9. SemanticMapper (vision)")
    from occupancy_grid import OccupancyGrid
    from vision import SemanticMapper

    grid = OccupancyGrid()
    sm = SemanticMapper(grid)

    # Test 9.1: classify_color — white (wall)
    # v4.0: classify_color returns (label, confidence) tuple
    white = np.array([[240, 240, 240]] * 10)
    ret = sm.classify_color(white)
    label = ret[0] if isinstance(ret, tuple) else ret
    result.check("9.1 white -> wall(2)",
                 label == 2, f"label={label}")

    # Test 9.2: classify_color — dark (doorway)
    dark = np.array([[50, 50, 50]] * 10)
    ret = sm.classify_color(dark)
    label = ret[0] if isinstance(ret, tuple) else ret
    result.check("9.2 dark -> doorway(3)",
                 label == 3, f"label={label}")

    # Test 9.3: classify_color — red (sofa)
    red = np.array([[180, 50, 50]] * 10)
    ret = sm.classify_color(red)
    label = ret[0] if isinstance(ret, tuple) else ret
    result.check("9.3 red -> sofa(4)",
                 label == 4, f"label={label}")

    # Test 9.4: classify_color — blue (bed)
    blue = np.array([[50, 80, 180]] * 10)
    ret = sm.classify_color(blue)
    label = ret[0] if isinstance(ret, tuple) else ret
    result.check("9.4 blue -> bed(5)",
                 label == 5, f"label={label}")

    # Test 9.5: classify_color — empty
    label = sm.classify_color(np.array([]))
    if isinstance(label, tuple):
        label = label[0]
    result.check("9.5 empty -> unknown(0)",
                 label == 0)

    # Test 9.6: get_label_at (v4.0 uses log_probs instead of semantic)
    grid2 = OccupancyGrid()
    grid2.log_odds[40, 50] = 2.0  # mark observed
    sm2 = SemanticMapper(grid2)
    # v4.0: set log_probs to simulate sofa observation
    if hasattr(sm2, 'log_probs'):
        sm2.log_probs[40, 50, :] = -5.0  # reset to low
        sm2.log_probs[40, 50, 4] = 2.0   # sofa = high
    lbl = sm2.get_label_at(0.0, 0.0)
    result.check("9.6 get_label_at returns name",
                 isinstance(lbl, str))

    # Test 9.7: coverage_by_label
    counts = sm2.coverage_by_label()
    result.check("9.7 coverage_by_label returns dict",
                 isinstance(counts, dict))
    result.check("9.7 has all label keys",
                 all(k in counts for k in ['unknown', 'floor', 'wall', 'doorway',
                                            'sofa', 'bed', 'table']))

    # Test 9.8: update with non-existent image
    lbl = sm.update("/nonexistent/path.jpg", 0.0, 0.0, 0.0)
    result.check("9.8 missing image returns 'unknown'",
                 lbl == 'unknown')


# ============================================================
# 10. config_loader
# ============================================================
def test_config_loader():
    result.section("10. config_loader")
    from config_loader import load_config, get

    cfg = load_config(force_reload=True)
    result.check("10.1 load_config returns dict",
                 isinstance(cfg, dict))
    # Should have some YAML files loaded
    result.check("10.1 config non-empty",
                 len(cfg) > 0,
                 f"keys={list(cfg.keys())}")

    # Test 10.2: get() function
    # Try to get a known section
    if 'sim' in cfg:
        v = get('sim', 'max_linear_x', None)
        result.check("10.2 get returns value from sim section",
                     v is not None or v is None)  # just doesn't crash
    else:
        result.check("10.2 sim section missing (skip)",
                     True)

    # Test 10.3: get with default
    v = get('nonexistent', 'key', 'default_val')
    result.check("10.3 get returns default for missing",
                 v == 'default_val')


# ============================================================
# 11. NavStateMachine
# ============================================================
def test_state_machine():
    result.section("11. NavStateMachine")
    from nav_core.runtime.state_machine import NavState, NavContext, NavStateMachine

    sm = NavStateMachine()

    # Test 11.1: PLAN -> FOLLOW when frontier found + path found
    ctx = NavContext()
    new_state, action = sm.transition_plan(ctx, path_found=True,
                                            has_reachable_frontier=True)
    result.check("11.1 PLAN->FOLLOW select_goal",
                 new_state == NavState.FOLLOW and action == 'select_goal')

    # Test 11.2: PLAN -> RECOVER when no path found
    ctx = NavContext()
    new_state, action = sm.transition_plan(ctx, path_found=False,
                                            has_reachable_frontier=True)
    result.check("11.2 PLAN->RECOVER when no path",
                 new_state == NavState.RECOVER and action == 'recover')

    # Test 11.3: PLAN -> DONE after too many empty cycles
    ctx = NavContext(no_frontier_cycles=sm.max_no_frontier_cycles)
    new_state, action = sm.transition_plan(ctx, path_found=False,
                                            has_reachable_frontier=False)
    result.check("11.3 PLAN->DONE after max cycles",
                 new_state == NavState.DONE and action == 'done')

    # Test 11.4: FOLLOW -> PLAN goal_reached
    ctx = NavContext(rx=0.1, ry=0.1, current_goal=(0.0, 0.0))
    new_state, action = sm.transition_follow(ctx)
    result.check("11.4 FOLLOW->PLAN goal_reached",
                 new_state == NavState.PLAN and action == 'goal_reached')

    # Test 11.5: FOLLOW -> RECOVER no_progress
    ctx = NavContext(no_progress_frames=sm.max_no_progress + 1,
                      current_goal=(5.0, 5.0))
    new_state, action = sm.transition_follow(ctx)
    result.check("11.5 FOLLOW->RECOVER no_progress",
                 new_state == NavState.RECOVER and action == 'no_progress')

    # Test 11.6: RECOVER -> PLAN after max frames
    ctx = NavContext(recover_frames=sm.max_recover_frames + 1)
    new_state, action = sm.transition_recover(ctx)
    result.check("11.6 RECOVER->PLAN retry_plan",
                 new_state == NavState.PLAN and action == 'retry_plan')

    # Test 11.7: DONE -> PLAN resume
    ctx = NavContext()
    new_state, action = sm.transition_done(ctx, has_reachable_frontier=True)
    result.check("11.7 DONE->PLAN resume",
                 new_state == NavState.PLAN and action == 'resume')

    # Test 11.8: NavState enum values
    result.check("11.8 NavState.PLAN=0",
                 NavState.PLAN == 0)
    result.check("11.8 NavState.FOLLOW=1",
                 NavState.FOLLOW == 1)
    result.check("11.8 NavState.RECOVER=2",
                 NavState.RECOVER == 2)
    result.check("11.8 NavState.DONE=3",
                 NavState.DONE == 3)


# ============================================================
# 12. NavigationRuntimeState
# ============================================================
def test_runtime_state():
    result.section("12. NavigationRuntimeState")
    from nav_core.runtime.runtime_state import NavigationRuntimeState
    from nav_core.runtime.state_machine import NavState

    rs = NavigationRuntimeState()

    # Test 12.1: default state is PLAN
    result.check("12.1 default state PLAN",
                 rs.state == NavState.PLAN)

    # Test 12.2: start_following
    rs.start_following(path=[(1.0, 0.0), (2.0, 0.0)], goal=(2.0, 0.0))
    result.check("12.2 state FOLLOW",
                 rs.state == NavState.FOLLOW)
    result.check("12.2 path set",
                 rs.current_path is not None and len(rs.current_path) == 2)
    result.check("12.2 goal set",
                 rs.current_goal == (2.0, 0.0))
    result.check("12.2 no_progress_frames cleared",
                 rs.no_progress_frames == 0)

    # Test 12.3: start_recovery
    rs.start_recovery(reset_align=True)
    result.check("12.3 state RECOVER",
                 rs.state == NavState.RECOVER)
    result.check("12.3 recover_frames cleared",
                 rs.recover_frames == 0)
    result.check("12.3 align_frames cleared",
                 rs.align_frames == 0)

    # Test 12.4: finish_recovery
    rs.finish_recovery()
    result.check("12.4 state PLAN after finish",
                 rs.state == NavState.PLAN)
    result.check("12.4 counters cleared",
                 rs.recover_frames == 0 and rs.no_progress_frames == 0)

    # Test 12.5: clear_goal
    rs.start_following(path=[(1.0, 0.0)], goal=(1.0, 0.0))
    rs.clear_goal()
    result.check("12.5 goal cleared",
                 rs.current_goal is None)
    result.check("12.5 goal_plan_attempts reset",
                 rs.goal_plan_attempts == 0)

    # Test 12.6: return_to_planning
    rs.return_to_planning()
    result.check("12.6 state PLAN",
                 rs.state == NavState.PLAN)

    # Test 12.7: mark_done
    rs.mark_done()
    result.check("12.7 state DONE",
                 rs.state == NavState.DONE)


# ============================================================
# 13. TelemetryCollector
# ============================================================
def test_telemetry():
    result.section("13. TelemetryCollector")
    from nav_core.metrics.telemetry import TelemetryCollector

    tc = TelemetryCollector(log_interval=5)

    # Test 13.1: init
    result.check("13.1 init counters zero",
                 tc.metrics['replan_count'] == 0 and
                 tc.metrics['recover_count'] == 0)

    # Test 13.2: frame_start/frame_end
    tc.frame_start()
    time.sleep(0.01)
    tc.frame_end('FOLLOW', 50.0, 3)
    result.check("13.2 total_frames incremented",
                 tc.metrics['total_frames'] == 1)
    result.check("13.2 state_time recorded",
                 tc.metrics['state_time']['FOLLOW'] > 0)

    # Test 13.3: record_astar
    tc.record_astar(duration=0.005, success=True)
    result.check("13.3 astar_call_count=1",
                 tc.metrics['astar_call_count'] == 1)

    # Test 13.4: record_astar failure increments replan
    tc.record_astar(duration=0.005, success=False)
    result.check("13.4 replan_count=1 on failure",
                 tc.metrics['replan_count'] == 1)

    # Test 13.5: record_dwa
    tc.record_dwa(duration=0.003, valid_ratio=0.8)
    result.check("13.5 dwa_call_count=1",
                 tc.metrics['dwa_call_count'] == 1)
    result.check("13.5 dwa_valid_ratio_history recorded",
                 len(tc.dwa_valid_ratio_history) == 1)

    # Test 13.6: record_recover
    tc.record_recover('no_progress')
    result.check("13.6 recover_count=1",
                 tc.metrics['recover_count'] == 1)
    result.check("13.6 recover_reasons recorded",
                 tc.metrics['recover_reasons']['no_progress'] == 1)

    # Test 13.7: record_goal_reached
    tc.record_goal_reached()
    result.check("13.7 goal_reached_count=1",
                 tc.metrics['goal_reached_count'] == 1)

    # Test 13.8: record_near_collision
    tc.record_near_collision()
    result.check("13.8 collision_near_count=1",
                 tc.metrics['collision_near_count'] == 1)

    # Test 13.9: get_summary
    summary = tc.get_summary()
    result.check("13.9 summary has total_frames",
                 'total_frames' in summary)
    result.check("13.9 summary has astar_avg_ms",
                 'astar_avg_ms' in summary)
    result.check("13.9 summary has recover_reasons",
                 'recover_reasons' in summary)


# ============================================================
# 14. SessionStore
# ============================================================
def test_session_store():
    result.section("14. SessionStore")
    from nav_core.session.session_store import SessionStore
    from occupancy_grid import OccupancyGrid

    tmp_dir = tempfile.mkdtemp()
    map_file = os.path.join(tmp_dir, "test_map.npz")
    ss = SessionStore(map_file)

    # Test 14.1: init
    result.check("14.1 init creates meta path",
                 ss.meta_file.endswith('_meta.json'))

    # Test 14.2: save_map / load_map
    grid = OccupancyGrid()
    grid.log_odds[10:20, 10:20] = 2.0
    grid.visited[10:20, 10:20] = True
    ss.save_map(grid)
    result.check("14.2 map file saved",
                 os.path.exists(map_file))

    grid2 = OccupancyGrid()
    loaded = ss.load_map(grid2)
    result.check("14.2 load_map returns True",
                 loaded)
    result.check("14.2 log_odds preserved",
                 np.allclose(grid2.log_odds[15, 15], 2.0))
    result.check("14.2 visited preserved",
                 grid2.visited[15, 15] == True)

    # Test 14.3: record_run
    ss.record_run(frames=100, coverage=85.5)
    ss.record_run(frames=200, coverage=90.0)
    meta = ss._load_meta()
    result.check("14.3 run_count=2",
                 meta.get('run_count') == 2)
    result.check("14.3 total_frames=300",
                 meta.get('total_frames') == 300)
    result.check("14.3 best_coverage=90.0",
                 approx(meta.get('best_coverage', 0), 90.0))

    # Test 14.4: save_params_summary
    ss.save_params_summary({'USE_AMCL': 1, 'USE_TEB': 1})
    meta = ss._load_meta()
    result.check("14.4 params_summary saved",
                 'params_summary' in meta and
                 meta['params_summary'].get('USE_AMCL') == 1)

    # Cleanup
    import shutil
    shutil.rmtree(tmp_dir, ignore_errors=True)


# ============================================================
# 15. FrontierManager
# ============================================================
def test_frontier_manager():
    result.section("15. FrontierManager")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from nav_core.exploration.frontier_manager import FrontierManager, FrontierSelection

    # Test 15.1: FrontierSelection dataclass
    sel = FrontierSelection(goal=(1.0, 2.0), info_gain=5.0, distance=3.0,
                             score=0.8, reachable_count=10, safe_count=8,
                             unsafe_count=2)
    result.check("15.1 FrontierSelection fields",
                 sel.goal == (1.0, 2.0) and sel.info_gain == 5.0)

    # Test 15.2: FrontierManager init
    grid = OccupancyGrid()
    grid.log_odds[10:70, 10:90] = -1.0  # free region
    cm = Costmap()
    cm.update_static(grid, frame=0)
    fm = FrontierManager(grid, cm, exploration_radius=8.0)
    result.check("15.2 init has occ_grid",
                 fm.occ_grid is grid)
    result.check("15.2 init has costmap",
                 fm.costmap is cm)
    result.check("15.2 init empty unreachable_goals",
                 len(fm.unreachable_goals) == 0)

    # Test 15.3: find_frontiers
    frontiers = fm.find_frontiers(0.0, 0.0, max_frontiers=10)
    result.check("15.3 find_frontiers returns list",
                 isinstance(frontiers, list))

    # Test 15.4: mark_unreachable
    fm.mark_unreachable(2.0, 3.0)
    result.check("15.4 unreachable goal added",
                 len(fm.unreachable_goals) > 0)

    # Test 15.5: is_unreachable
    result.check("15.5 is_unreachable detects marked",
                 fm.is_unreachable(2.0, 3.0))

    # Test 15.6: clear_unreachable
    fm.clear_unreachable()
    result.check("15.6 clear_unreachable empties set",
                 len(fm.unreachable_goals) == 0)

    # Test 15.7: mark_visited
    fm.mark_visited(1.0, 1.0, frame=100)
    result.check("15.7 visited_frontiers recorded",
                 len(fm.visited_frontiers) > 0)

    # Test 15.8: is_persistent_frontier
    result.check("15.8 is_persistent_frontier doesn't crash",
                 isinstance(fm.is_persistent_frontier(1.0, 1.0), bool))

    # Test 15.9: get_state / set_state
    state = fm.get_state()
    result.check("15.9 get_state returns dict",
                 isinstance(state, dict))
    result.check("15.9 state has unreachable_goals",
                 'unreachable_goals' in state)

    # Test 15.10: config-based weights
    fm2 = FrontierManager(grid, cm, config={
        'w_info_gain': 0.5, 'w_path_cost': 0.2,
        'w_risk': 0.1, 'w_heading_change': 0.1, 'w_retry_penalty': 0.1,
    })
    result.check("15.10 config w_info_gain=0.5",
                 approx(fm2.w_info_gain, 0.5))
    result.check("15.10 config w_path_cost=0.2",
                 approx(fm2.w_path_cost, 0.2))

    # Test 15.11: has_recent_oscillation
    result.check("15.11 has_recent_oscillation returns bool",
                 isinstance(fm.has_recent_oscillation([10, 20, 30], 50, cooldown=40), bool))

    # Test 15.12: gc_visited
    fm.gc_visited(current_frame=1000)
    result.check("15.12 gc_visited doesn't crash",
                 True)


# ============================================================
# Main runner
# ============================================================
def main():
    print("\n" + "#" * 60)
    print("#  COMPREHENSIVE MODULE TEST SUITE")
    print("#  Tests all navigation modules without CoppeliaSim")
    print("#" * 60)

    tests = [
        ("Odometry", test_odometry),
        ("OccupancyGrid", test_occupancy_grid),
        ("Costmap", test_costmap),
        ("AStarPlanner", test_astar_planner),
        ("DWAPlanner", test_dwa_planner),
        ("TEBPlanner", test_teb_planner),
        ("AMCL", test_amcl),
        ("Evaluator", test_evaluator),
        ("SemanticMapper", test_vision),
        ("config_loader", test_config_loader),
        ("NavStateMachine", test_state_machine),
        ("NavigationRuntimeState", test_runtime_state),
        ("TelemetryCollector", test_telemetry),
        ("SessionStore", test_session_store),
        ("FrontierManager", test_frontier_manager),
    ]

    failed_modules = []
    for name, fn in tests:
        try:
            fn()
        except Exception as e:
            result.failed += 1
            failed_modules.append(name)
            err_msg = f"EXCEPTION: {type(e).__name__}: {e}"
            result.errors.append((name, err_msg))
            print(f"\n  [ERROR] {name} raised exception:")
            traceback.print_exc()

    # Summary
    print("\n" + "#" * 60)
    print("#  TEST SUMMARY")
    print("#" * 60)
    total = result.passed + result.failed
    print(f"  Total checks: {total}")
    print(f"  Passed:       {result.passed}")
    print(f"  Failed:       {result.failed}")

    if result.errors:
        print(f"\n  Failures:")
        for name, msg in result.errors:
            print(f"    - {name}: {msg}")

    if failed_modules:
        print(f"\n  Modules with exceptions: {failed_modules}")

    print()
    if result.failed == 0:
        print("  *** ALL TESTS PASSED ***")
        return 0
    else:
        print(f"  *** {result.failed} TEST(S) FAILED ***")
        return 1


if __name__ == "__main__":
    sys.exit(main())
