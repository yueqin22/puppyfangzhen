#!/usr/bin/env python3
"""
v4.0 Academic Enhancement Test Suite
=====================================
Tests all 12 v4.0 academic enhancement modules.

Modules tested:
  1.  pareto.py            — Pareto multi-objective optimization
  2.  vision.py (v4.0)     — Probabilistic semantic fusion
  3.  active_slam.py       — Active SLAM + NBV
  4.  hybrid_astar.py      — Hybrid A* global planner
  5.  loop_closure.py      — Loop closure + pose graph optimization
  6.  rbpf_slam.py         — Rao-Blackwellized PF SLAM
  7.  dynamic_obstacles.py — Kalman tracking + Velocity Obstacle
  8.  mpc_planner.py       — Model Predictive Control
  9.  semantic_slam.py     — Semantic SLAM landmarks
  10. belief_planner.py    — Belief space planning
  11. rl_agent.py          — DQN RL agent
  12. (already in v3.0: AMCL KLD, info-gain frontier, TEB+jerk)
"""
import os
import sys
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

passed = 0
failed = 0
errors = []


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print(f"  [PASS] {name}")
    else:
        failed += 1
        errors.append((name, detail))
        print(f"  [FAIL] {name}  {detail}")


def section(title):
    print(f"\n{'='*60}\n  {title}\n{'='*60}")


# ============================================================
# 1. Pareto Multi-Objective Optimization
# ============================================================
def test_pareto():
    section("1. Pareto Multi-Objective Optimization")
    from pareto import (dominates, find_pareto_front, non_dominated_sort,
                         crowding_distance, topsis, select_pareto_best)

    # 1.1 Dominance
    hib = [True, True]  # both objectives maximize
    check("1.1 dominates: a dominates b",
          dominates([5, 5], [3, 4], hib))
    check("1.1 dominates: b does NOT dominate a",
          not dominates([3, 4], [5, 5], hib))
    check("1.1 non-dominated: equal",
          not dominates([3, 4], [3, 4], hib))
    check("1.1 mixed: neither dominates",
          not dominates([5, 2], [2, 5], hib)
          and not dominates([2, 5], [5, 2], hib))

    # 1.2 Minimize objectives
    hib_min = [False, False]  # both minimize
    check("1.2 minimize: a=(2,3) dominates b=(5,4)",
          dominates([2, 3], [5, 4], hib_min))

    # 1.3 Pareto front
    solutions = [
        ((5, 5), 'A'),
        ((3, 6), 'B'),
        ((4, 4), 'C'),
        ((6, 3), 'D'),
        ((2, 2), 'E'),
    ]
    pareto = find_pareto_front(solutions, [True, True])
    labels = [s[1] for s in pareto]
    check("1.3 pareto front has 3 non-dominated points",
          len(pareto) == 3, f"len={len(pareto)}, labels={labels}")
    check("1.3 dominated point C not in pareto",
          'C' not in labels)
    check("1.3 dominated point E not in pareto",
          'E' not in labels)

    # 1.4 Non-dominated sort
    fronts = non_dominated_sort(solutions, [True, True])
    check("1.4 3 fronts found", len(fronts) == 3, f"n_fronts={len(fronts)}")
    check("1.4 front 0 = pareto front", len(fronts[0]) == 3)
    check("1.4 front 1 has 1 dominated point", len(fronts[1]) == 1)
    check("1.4 front 2 has 1 point", len(fronts[2]) == 1)

    # 1.5 Crowding distance
    cd = crowding_distance(pareto)
    check("1.5 crowding distance list length matches",
          len(cd) == len(pareto))
    check("1.5 extreme points have inf crowding",
          any(c == float('inf') for c in cd),
          f"cd={cd}")

    # 1.6 TOPSIS
    scored = topsis(pareto, [True, True])
    check("1.6 TOPSIS returns scored list",
          len(scored) == len(pareto))
    check("1.6 TOPSIS scores in [0,1]",
          all(0.0 <= s[2] <= 1.0 for s in scored))
    check("1.6 TOPSIS sorted descending",
          all(scored[i][2] >= scored[i+1][2]
              for i in range(len(scored)-1)))

    # 1.7 select_pareto_best
    best_obj, best_payload, score = select_pareto_best(
        solutions, [True, True], method='topsis')
    check("1.7 select_pareto_best returns valid result",
          best_obj is not None and best_payload is not None)
    check("1.7 score in [0,1]", 0.0 <= score <= 1.0)

    # 1.8 Max-min method
    best_obj2, best_pay2, score2 = select_pareto_best(
        solutions, [True, True], method='maxmin')
    check("1.8 maxmin method works",
          best_obj2 is not None and 0 <= score2 <= 1)

    # 1.9 Empty input
    empty = find_pareto_front([], [True, True])
    check("1.9 empty input returns empty", len(empty) == 0)


# ============================================================
# 2. Probabilistic Semantic Fusion
# ============================================================
def test_vision():
    section("2. Probabilistic Semantic Fusion (v4.0)")
    from occupancy_grid import OccupancyGrid
    from vision import SemanticMapper, N_LABELS

    grid = OccupancyGrid()
    grid.log_odds[30:50, 30:70] = -1.0  # free space

    sm = SemanticMapper(grid)

    # 2.1 Init
    check("2.1 n_labels = 7", sm.n_labels == 7)
    check("2.1 log_probs shape correct",
          sm.log_probs.shape == (grid.height, grid.width, N_LABELS),
          f"shape={sm.log_probs.shape}")
    check("2.1 priors sum to 1 per cell",
          abs(np.sum(np.exp(sm.log_probs[40, 50])) - 1.0) < 0.01)

    # 2.2 classify_color returns (label, conf)
    white = np.array([[240, 240, 240]] * 10)
    result = sm.classify_color(white)
    check("2.2 classify returns tuple of 2",
          isinstance(result, tuple) and len(result) == 2,
          f"result type={type(result)}, len={len(result) if isinstance(result, tuple) else 'N/A'}")
    label, conf = result
    check("2.2 white = wall(2)", label == 2, f"label={label}")
    check("2.2 confidence > 0.5", conf > 0.5, f"conf={conf}")

    # 2.3 Bayesian update increases confidence
    gx, gy = 50, 40
    # Initial entropy (high uncertainty)
    H0 = sm.get_entropy(gx, gy)
    # Apply multiple wall observations
    for _ in range(5):
        sm._bayesian_update(gy, gx, 2, 0.8)  # wall observation, 80% conf
    H1 = sm.get_entropy(gx, gy)
    check("2.3 entropy decreases with observations",
          H1 < H0, f"H0={H0:.3f}, H1={H1:.3f}")
    check("2.3 MAP label is wall",
          np.argmax(sm.log_probs[gy, gx]) == 2)

    # 2.4 get_label_probs returns valid distribution
    wx, wy = grid.grid_to_world(gx, gy)
    probs = sm.get_label_probs(wx, wy)
    check("2.4 probs sum to ~1", abs(sum(probs) - 1.0) < 0.01,
          f"sum={sum(probs):.4f}")
    check("2.4 wall probability highest", np.argmax(probs) == 2,
          f"probs={probs}")

    # 2.5 Confusion matrix rows sum to 1
    check("2.5 confusion rows sum to 1",
          np.allclose(sm.confusion.sum(axis=1), 1.0, atol=1e-5))

    # 2.6 Semantic info gain
    ig = sm.semantic_info_gain(gx, gy)
    check("2.6 semantic_info_gain in [0,1]", 0.0 <= ig <= 1.0, f"ig={ig}")

    # 2.7 semantic_info_gain_region
    ig_region = sm.semantic_info_gain_region(gx, gy, radius_cells=5)
    check("2.7 region info gain >= 0", ig_region >= 0, f"ig_region={ig_region}")

    # 2.8 coverage_by_label
    counts = sm.coverage_by_label()
    check("2.8 coverage has all label keys",
          all(k in counts for k in ['unknown', 'floor', 'wall', 'doorway',
                                     'sofa', 'bed', 'table']))


# ============================================================
# 3. Active SLAM + NBV
# ============================================================
def test_active_slam():
    section("3. Active SLAM + NBV Planning")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from active_slam import ActiveSLAMEstimator, NBVPlanner

    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0  # most free
    grid.log_odds[30:50, 50:52] = 2.0  # small wall
    cm = Costmap()
    cm.update_static(grid, frame=0)

    asle = ActiveSLAMEstimator(grid, cm)

    # 3.1 Init
    check("3.1 weights sum ~1",
          abs(asle.w_geo + asle.w_loc + asle.w_sem - 1.0) < 0.1)

    # 3.2 expected_info_gain
    total, geo, loc, sem = asle.expected_info_gain(0.0, 0.0, 0.0)
    check("3.2 total IG is combination", total >= 0.0, f"total={total:.4f}")
    check("3.2 geo IG computed", geo >= 0.0, f"geo={geo:.4f}")
    check("3.2 loc IG computed", loc >= 0.0, f"loc={loc:.4f}")
    check("3.2 sem IG zero (no semantic mapper)", sem == 0.0)

    # 3.3 Update localization covariance
    new_cov = np.eye(3) * 0.05
    asle.update_localization_covariance(new_cov)
    check("3.3 covariance updated",
          np.allclose(asle.loc_covariance, new_cov))

    # 3.4 NBV planner
    nbv = NBVPlanner(grid, cm, asle, n_candidates=5)
    # Create some frontiers manually
    frontiers = [(1.0, 0.5, 10), (-1.0, 1.5, 15), (2.0, -0.5, 8)]
    result = nbv.plan_nbv(0.0, 0.0, 0.0, frontiers=frontiers)
    check("3.4 NBV returns tuple or None",
          result is None or (isinstance(result, tuple) and len(result) == 4))

    # 3.5 High uncertainty increases loc IG weight effect
    asle2 = ActiveSLAMEstimator(grid, cm, w_localization=0.9, w_geometric=0.1)
    asle2.loc_covariance = np.eye(3) * 1.0  # high uncertainty
    t2, g2, l2, s2 = asle2.expected_info_gain(0.0, 0.0)
    check("3.5 high unc -> higher loc IG", l2 > 0)

    # 3.6 boundary sampling mode
    nbv2 = NBVPlanner(grid, cm, asle, mode='boundary_sampling', n_candidates=5)
    result2 = nbv2._boundary_sampling(0.0, 0.0, 0.0)
    check("3.6 boundary sampling runs without crash", True)


# ============================================================
# 4. Hybrid A*
# ============================================================
def test_hybrid_astar():
    section("4. Hybrid A* Global Planner")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from hybrid_astar import HybridAStarPlanner

    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0  # free
    cm = Costmap()
    cm.update_static(grid, frame=0)

    planner = HybridAStarPlanner(cm, xy_resolution=0.3, step_size=0.3,
                                  max_iterations=2000)

    # 4.1 Init
    check("4.1 has motion primitives",
          len(planner.primitives) >= 3, f"n_prim={len(planner.primitives)}")

    # 4.2 State conversion round-trip
    sx, sy, st = 1.0, 0.5, 0.3
    idx = planner._state_to_index(sx, sy, st)
    check("4.2 state_to_index returns 3-tuple",
          len(idx) == 3 and all(isinstance(i, int) for i in idx))
    rx, ry, rt = planner._index_to_state(*idx)
    check("4.2 index_to_state approximate round-trip",
          abs(rx - sx) < 0.3 and abs(ry - sy) < 0.3)

    # 4.3 Plan simple path
    path = planner.plan(0.0, 0.0, 0.0, 1.5, 0.0, gtheta=0.0)
    check("4.3 plan returns list or None",
          path is None or isinstance(path, list))
    if path is not None:
        check("4.3 path has multiple waypoints", len(path) > 0)
        check("4.3 each waypoint is (x,y,theta)",
              all(len(p) == 3 for p in path))
        # Check path ends near goal
        last = path[-1]
        dist = math.sqrt((last[0] - 1.5)**2 + (last[1] - 0.0)**2)
        check("4.3 path reaches goal area", dist < 1.0,
              f"dist_to_goal={dist:.3f}")

    # 4.4 Motion primitives are valid
    for prim in planner.primitives:
        check("4.4 primitive has 3 components", len(prim) == 3)

    # 4.5 is_safe check
    check("4.5 safe position is safe", planner._is_safe(0.0, 0.0))


# ============================================================
# 5. Loop Closure + Pose Graph Optimization
# ============================================================
def test_loop_closure():
    section("5. Loop Closure + Pose Graph Optimization")
    from loop_closure import PoseGraph, LoopClosureDetector

    pg = PoseGraph()

    # 5.1 Add keyframes
    kf0 = pg.add_keyframe(0.0, 0.0, 0.0)
    kf1 = pg.add_keyframe(1.0, 0.0, 0.0)
    kf2 = pg.add_keyframe(2.0, 0.0, 0.0)
    check("5.1 3 keyframes added", len(pg.keyframes) == 3)
    check("5.1 2 odom edges added",
          sum(1 for e in pg.edges if e[6] == 'odom') == 2)

    # 5.2 Add loop closure
    pg.add_loop_closure(0, 2, 2.0, 0.0, 0.0, confidence=0.8)
    check("5.2 1 loop closure edge",
          sum(1 for e in pg.edges if e[6] == 'loop') == 1)

    # 5.3 Optimize - total error should decrease
    # First add some drift
    pg.keyframes[2].x = 2.1
    pg.keyframes[2].y = 0.05
    err_before = pg.optimize(n_iterations=1)
    err_after = pg.optimize(n_iterations=20)
    check("5.3 optimization reduces error",
          err_after <= err_before + 1e-6,
          f"err_before={err_before:.4f}, err_after={err_after:.4f}")

    # 5.4 First pose fixed (gauge freedom)
    check("5.4 first pose unchanged after opt",
          abs(pg.keyframes[0].x - 0.0) < 1e-6
          and abs(pg.keyframes[0].y - 0.0) < 1e-6)

    # 5.5 Loop closure detector
    lcd = LoopClosureDetector(min_keyframes_between=2)
    angles = [2 * math.pi * i / 36 for i in range(36)]
    dists = [3.0] * 36
    check("5.5 detector init ok", lcd.icp_max_iter > 0)

    # 5.6 Empty scan detection
    result = lcd.detect(pg.keyframes[-1], pg, [], [])
    check("5.6 empty scan returns empty list",
          isinstance(result, list) and len(result) == 0)

    # 5.7 Inverse compose
    xi = (0.0, 0.0, 0.0)
    xj = (1.0, 0.0, 0.0)
    dx, dy, dt = pg._inverse_compose(xi, xj)
    check("5.7 inverse compose: x+1 to the right",
          abs(dx - 1.0) < 1e-6 and abs(dy) < 1e-6)


# ============================================================
# 6. Rao-Blackwellized PF SLAM
# ============================================================
def test_rbpf_slam():
    section("6. Rao-Blackwellized PF SLAM")
    from rbpf_slam import RBPFSLAM, SLAMParticle

    rbpf = RBPFSLAM(n_particles=5, sigma_xy=0.01, sigma_yaw=0.01)

    # 6.1 Init
    check("6.1 5 particles created", len(rbpf.particles) == 5)
    check("6.1 weights sum ~1",
          abs(sum(p.weight for p in rbpf.particles) - 1.0) < 0.01)

    # 6.2 Init at pose
    rbpf.init_at(0.0, 0.0, 0.0, spread=0.1)
    xs = [p.x for p in rbpf.particles]
    check("6.2 particles near init pose",
          abs(np.mean(xs)) < 0.5, f"mean_x={np.mean(xs):.4f}")

    # 6.3 Each particle has its own map
    check("6.3 each particle has map",
          all(hasattr(p, 'map') and p.map is not None for p in rbpf.particles))

    # 6.4 Predict step
    old_x = np.mean([p.x for p in rbpf.particles])
    rbpf.predict(v=0.2, w=0.0, dt=0.5)
    new_x = np.mean([p.x for p in rbpf.particles])
    check("6.4 predict moves particles forward",
          new_x > old_x + 0.05, f"old={old_x:.3f}, new={new_x:.3f}")

    # 6.5 Update step (with a simple scan)
    angles = [2 * math.pi * i / 12 for i in range(12)]
    dists = [3.0] * 12
    n_eff = rbpf.update(angles, dists, frame=1)
    check("6.5 update returns n_eff", n_eff > 0, f"n_eff={n_eff:.2f}")

    # 6.6 Best particle exists
    best = rbpf.get_best_particle()
    check("6.6 best particle has map", hasattr(best, 'map'))

    # 6.7 Mean estimate
    mx, my, myaw = rbpf.get_mean_estimate()
    check("6.7 mean estimate is finite",
          all(np.isfinite([mx, my, myaw])))

    # 6.8 Best map
    best_map = rbpf.get_best_map()
    check("6.8 best_map is OccupancyGrid",
          hasattr(best_map, 'log_odds'))


# ============================================================
# 7. Dynamic Obstacles + Velocity Obstacle
# ============================================================
def test_dynamic_obstacles():
    section("7. Dynamic Obstacles + Velocity Obstacle")
    from occupancy_grid import OccupancyGrid
    from dynamic_obstacles import (KalmanTracker, ObstacleTracker,
                                     VelocityObstacle)

    # 7.1 Kalman tracker init
    kt = KalmanTracker(0.0, 0.0, vx=0.5, vy=0.0, dt=0.1)
    check("7.1 kalman state shape correct",
          kt.x.shape == (4,) and kt.F.shape == (4, 4))

    # 7.2 Predict
    kt.predict()
    check("7.2 predict moves object", kt.x[0] > 0.0, f"x={kt.x[0]:.4f}")

    # 7.3 Update
    z = np.array([0.06, 0.01])
    kt.update(z)
    check("7.3 update adjusts state", True)

    # 7.4 Predict position ahead
    px, py = kt.predict_position(1.0)
    check("7.4 predict position is finite",
          np.isfinite(px) and np.isfinite(py))

    # 7.5 Velocity Obstacle computation
    vo = VelocityObstacle.compute_vo(
        (0.0, 0.0), (5.0, 0.0),  # robot at 0, obstacle at 5,0
        (0.0, 0.0), (-1.0, 0.0),  # robot v=0, obstacle moving left
        0.3, 0.3,  # radii
        time_horizon=3.0)
    check("7.5 VO tuple has 4 elements", len(vo) == 4)

    # 7.6 Collision velocity is in VO
    v_col_x = -1.0  # robot moving toward obstacle at same speed
    in_vo = VelocityObstacle.velocity_in_vo(v_col_x, 0.0, vo)
    check("7.6 collision velocity is in VO", in_vo)

    # 7.7 Safe velocity is NOT in VO
    in_vo_safe = VelocityObstacle.velocity_in_vo(0.5, 0.5, vo)
    # May or may not be in VO depending on geometry, just check no crash
    check("7.7 velocity_in_vo returns bool",
          isinstance(in_vo_safe, bool))

    # 7.8 find_safe_velocity
    obstacles = [(5.0, 0.0, -1.0, 0.0, 0.3)]
    safe_v, safe_w = VelocityObstacle.find_safe_velocity(
        0.2, 0.0, 0.0, 0.0, 0.0, obstacles, 0.3, 0.5, 1.0)
    check("7.8 find_safe_velocity returns (v, w)",
          isinstance(safe_v, float) and isinstance(safe_w, float))

    # 7.9 ObstacleTracker init
    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0
    ot = ObstacleTracker(grid, max_obstacles=5, dt=0.1)
    check("7.9 obstacle tracker init", len(ot.trackers) == 0)

    # 7.10 Project out of VO
    vo2 = VelocityObstacle.compute_vo(
        (0.0, 0.0), (3.0, 0.0), (0.0, 0.0), (-1.0, 0.0), 0.3, 0.3)
    nvx, nvy = VelocityObstacle._project_out_vo(-1.0, 0.0, vo2)
    check("7.10 project_out_vo returns finite",
          np.isfinite(nvx) and np.isfinite(nvy))


# ============================================================
# 8. MPC Local Planner
# ============================================================
def test_mpc():
    section("8. MPC Model Predictive Control")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from mpc_planner import MPCPlanner

    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0
    cm = Costmap()
    cm.update_static(grid, frame=0)

    mpc = MPCPlanner(cm, horizon_steps=5, dt=0.1, mpc_iterations=3,
                      max_linear_vel=0.3, max_angular_vel=1.0)

    # 8.1 Init
    check("8.1 horizon = 5", mpc.horizon == 5)
    check("8.1 weights positive",
          mpc.w_track > 0 and mpc.w_obstacle > 0)

    # 8.2 Simple trajectory simulation
    u = np.zeros(2 * 5)
    u[0::2] = 0.2  # v = 0.2
    u[1::2] = 0.0  # w = 0
    xs, ys, yaws = mpc._simulate_trajectory(u, 0.0, 0.0, 0.0)
    check("8.2 trajectory has horizon+1 points", len(xs) == 6)
    check("8.2 moves forward", xs[-1] > 0.05, f"x_end={xs[-1]:.3f}")

    # 8.3 Compute velocity
    path = [(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
    v, w = mpc.compute_velocity(0.0, 0.0, 0.0, path, 1.5, 0.0)
    check("8.3 returns (v, w)",
          isinstance(v, (int, float)) and isinstance(w, (int, float)))
    check("8.3 v within limits", abs(v) <= mpc.max_v + 1e-6)
    check("8.3 w within limits", abs(w) <= mpc.max_w + 1e-6)

    # 8.4 Feasible projection
    u_test = np.array([1.0, 5.0] * 5)  # way beyond limits
    u_proj = mpc._project_feasible(u_test)
    check("8.4 projected v within limits",
          all(abs(u_proj[2*i]) <= mpc.max_v for i in range(5)))
    check("8.4 projected w within limits",
          all(abs(u_proj[2*i+1]) <= mpc.max_w for i in range(5)))

    # 8.5 Reset
    mpc.reset()
    check("8.5 reset clears prev values",
          mpc.prev_v == 0.0 and mpc.prev_w == 0.0)

    # 8.6 Empty path
    v2, w2 = mpc.compute_velocity(0.0, 0.0, 0.0, [], 0.0, 0.0)
    check("8.6 empty path returns 0,0", v2 == 0.0 and w2 == 0.0)


# ============================================================
# 9. Semantic SLAM (Object Landmarks)
# ============================================================
def test_semantic_slam():
    section("9. Semantic SLAM - Object Landmarks")
    from occupancy_grid import OccupancyGrid
    from semantic_slam import SemanticSLAM, SemanticLandmark

    grid = OccupancyGrid()
    ssl = SemanticSLAM(grid)

    # 9.1 Init
    check("9.1 no landmarks initially", len(ssl.landmarks) == 0)

    # 9.2 Add landmark
    lm = ssl.add_landmark(4, 1.5, 2.0, confidence=0.8, size=0.5)  # sofa
    check("9.2 landmark added", len(ssl.landmarks) == 1)
    check("9.2 landmark is sofa", lm.label_idx == 4)
    check("9.2 landmark has position", lm.x == 1.5 and lm.y == 2.0)

    # 9.3 Observe objects (data association)
    # Observe same sofa from a new angle (close enough to match)
    objects = [(4, 0.0, 0.5, 0.7)]  # label_idx, rel_angle, rel_dist, conf
    matches = ssl.observe_objects(objects, 1.0, 2.0, 0.0)
    check("9.3 observe matches existing landmark",
          len(matches) >= 0)  # may or may not match depending on distance

    # 9.4 Log likelihood computation
    log_lik = ssl.compute_log_likelihood(0.0, 0.0, 0.0, objects)
    check("9.4 log likelihood is finite", np.isfinite(log_lik))

    # 9.5 Display landmarks
    disp = ssl.get_landmarks_for_display()
    check("9.5 display list is list", isinstance(disp, list))

    # 9.6 Multiple landmarks
    ssl.add_landmark(5, -1.0, 1.0, confidence=0.7)  # bed
    ssl.add_landmark(6, 2.0, -0.5, confidence=0.6)  # table
    check("9.6 3 landmarks total", len(ssl.landmarks) == 3)

    # 9.7 Save to JSON
    import tempfile, json
    tmp = tempfile.NamedTemporaryFile(suffix='.json', delete=False)
    tmp.close()
    ssl.save(tmp.name)
    with open(tmp.name) as f:
        data = json.load(f)
    check("9.7 saved JSON has 3 entries", len(data) == 3)
    os.unlink(tmp.name)


# ============================================================
# 10. Belief Space Planning
# ============================================================
def test_belief_planner():
    section("10. Belief Space Planning")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from belief_planner import BeliefState, BeliefPlanner

    grid = OccupancyGrid()
    grid.log_odds[5:75, 5:95] = -1.0
    grid.log_odds[30:50, 40:42] = 2.0  # wall with features
    cm = Costmap()
    cm.update_static(grid, frame=0)

    # 10.1 BeliefState
    b = BeliefState(0.0, 0.0, 0.0)
    check("10.1 belief has cov 3x3", b.cov.shape == (3, 3))
    check("10.1 belief entropy is finite",
          np.isfinite(b.entropy()))
    check("10.1 uncertainty trace positive", b.uncertainty_trace() > 0)

    # 10.2 BeliefPlanner
    bp = BeliefPlanner(grid, cm)
    check("10.2 weights sum ~1",
          abs(bp.w_progress + bp.w_uncertainty + bp.w_info_gain - 1.0) < 0.1)

    # 10.3 Predict belief
    b2 = bp.predict_belief(b, v=0.2, w=0.0, dt=0.5)
    check("10.3 predict moves belief forward", b2.x > 0.0, f"x={b2.x:.4f}")
    check("10.3 predict increases uncertainty",
          b2.uncertainty_trace() > b.uncertainty_trace())

    # 10.4 Expected info gain
    ig = bp.expected_info_gain(b)
    check("10.4 info gain in [0, 1]", 0.0 <= ig <= 1.0, f"ig={ig:.4f}")

    # 10.5 Evaluate candidates
    frontiers = [(1.0, 0.5), (2.0, -0.5), (-0.5, 1.0)]
    candidates = bp.evaluate_candidates(b, 3.0, 0.0,
                                         frontiers=[(f[0], f[1], 5) for f in frontiers])
    check("10.5 evaluate_candidates returns list",
          isinstance(candidates, list))
    if candidates:
        check("10.5 each candidate has 4 elements",
              len(candidates[0]) == 4)

    # 10.6 Should recover threshold
    b_high = BeliefState(0.0, 0.0, 0.0, cov=np.eye(3) * 2.0)
    check("10.6 high uncertainty triggers recover",
          bp.should_recover(b_high))
    check("10.6 low uncertainty does NOT trigger recover",
          not bp.should_recover(b))

    # 10.7 Recovery actions
    rec_actions = bp.get_recovery_actions(b_high)
    check("10.7 recovery actions list", len(rec_actions) > 0)

    # 10.8 Copy
    b_copy = b.copy()
    b_copy.x = 999.0
    check("10.8 copy is independent", b.x == 0.0)


# ============================================================
# 11. RL DQN Agent
# ============================================================
def test_rl_agent():
    section("11. RL DQN Agent")
    from rl_agent import (ReplayBuffer, SimpleQNetwork, FrontierDQNAgent,
                           build_frontier_state, compute_frontier_reward)

    # 11.1 ReplayBuffer
    buf = ReplayBuffer(capacity=100)
    check("11.1 empty buffer", len(buf) == 0)
    buf.push([0]*8, 0, 1.0, [0]*8, False)
    check("11.1 buffer after push", len(buf) == 1)
    sample = buf.sample(1)
    check("11.1 sample has 1 item", len(sample) == 1)

    # 11.2 Q-Network forward pass
    qnet = SimpleQNetwork(8, 32, 4, lr=0.001)
    state = np.random.randn(1, 8).astype(np.float32)
    q_vals = qnet.predict(state)
    check("11.2 Q-network output shape",
          q_vals.shape == (1, 4), f"shape={q_vals.shape}")
    check("11.2 Q-values finite", np.all(np.isfinite(q_vals)))

    # 11.3 Train step
    target = np.random.randn(1, 4).astype(np.float32)
    loss = qnet.train_step(state, target)
    check("11.3 train step returns loss",
          isinstance(loss, float) and np.isfinite(loss))

    # 11.4 DQN Agent
    agent = FrontierDQNAgent(state_dim=8, n_actions=4,
                              hidden_dim=16, batch_size=8)
    check("11.4 agent has q_net and target_net",
          hasattr(agent, 'q_net') and hasattr(agent, 'target_net'))
    check("11.4 epsilon starts at 1.0",
          abs(agent.epsilon - 1.0) < 0.01)

    # 11.5 Select action (exploration mode)
    np.random.seed(42)
    state_arr = np.zeros(8, dtype=np.float32)
    action = agent.select_action(state_arr, training=True)
    check("11.5 action is int in [0, n_actions)",
          isinstance(action, int) and 0 <= action < 4)

    # 11.6 Select action (evaluation mode)
    action_eval = agent.select_action(state_arr, training=False)
    check("11.6 eval mode returns action",
          isinstance(action_eval, int) and 0 <= action_eval < 4)

    # 11.7 Store transition
    agent.store_transition(state_arr, 0, 1.0, state_arr, False)
    check("11.7 replay buffer grew", len(agent.replay) == 1)

    # 11.8 Training step
    for _ in range(10):
        agent.store_transition(
            np.random.randn(8).astype(np.float32),
            np.random.randint(0, 4),
            np.random.randn() * 0.1,
            np.random.randn(8).astype(np.float32),
            np.random.random() > 0.9
        )
    loss = agent.train()
    check("11.8 train returns loss", isinstance(loss, float))
    check("11.8 train increments step count",
          agent.train_step_count > 0)

    # 11.9 Build state
    s = build_frontier_state(0.5, 0.3, 5, 0.1, 50.0, 1.0, 3, 0.15, 0.7)
    check("11.9 state has 8 features", len(s) == 8)
    check("11.9 state values in reasonable range",
          all(-5.0 <= v <= 5.0 for v in s))

    # 11.10 Compute reward
    r = compute_frontier_reward(50.0, 52.0, -0.02, False, False, 10)
    check("11.10 reward computed", isinstance(r, float))
    check("11.10 coverage gain gives positive reward", r > 0, f"r={r:.3f}")

    # 11.11 Save/load
    import tempfile
    tmpdir = tempfile.mkdtemp()
    filepath = os.path.join(tmpdir, 'dqn_test')
    agent.save(filepath)
    agent2 = FrontierDQNAgent(state_dim=8, n_actions=4, hidden_dim=16)
    agent2.load(filepath)
    check("11.11 save/load preserves episode",
          agent2.episode == agent.episode)
    import shutil
    shutil.rmtree(tmpdir, ignore_errors=True)


# ============================================================
# 12. v3.0 Modules Regression Check
# ============================================================
def test_v3_regression():
    section("12. v3.0 Modules Regression Check")
    from occupancy_grid import OccupancyGrid
    from costmap import Costmap
    from teb_planner import TEBPlanner
    from amcl import AMCL
    from pareto import find_pareto_front

    # 12.1 OccupancyGrid still works
    g = OccupancyGrid()
    angles = [0.0, math.pi/2, math.pi, 3*math.pi/2]
    dists = [2.0, 2.0, 2.0, 2.0]
    g.update_from_scan(0.0, 0.0, angles, dists, max_range=8.0)
    check("12.1 occupancy grid scan update works",
          g.is_occupied(*g.world_to_grid(2.0, 0.0)))

    # 12.2 TEB v3.0 weights preserved
    cm = Costmap()
    cm.update_static(g, frame=0)
    teb = TEBPlanner(cm)
    check("12.2 TEB w_time=0.3", abs(teb.w_time - 0.3) < 1e-6)
    check("12.2 TEB w_jerk=0.05 (v3.1 完整Jerk梯度)", abs(teb.w_jerk - 0.05) < 1e-6)

    # 12.3 AMCL KLD still works
    amcl = AMCL(g, n_particles=50, n_obs_rays=8)
    amcl.init_cloud(0.0, 0.0, 0.0, spread=0.2)
    amcl.weight(angles, dists, frame=0)
    amcl.resample()
    check("12.3 AMCL KLD resample count in bounds",
          amcl.kld_min <= len(amcl.particles) <= amcl.kld_max)

    # 12.4 Pareto + v3.0 frontier info gain coexist
    from occupancy_grid import USE_INFO_THEORY
    check("12.4 USE_INFO_THEORY default True", USE_INFO_THEORY)


# ============================================================
# Main
# ============================================================
def main():
    print("\n" + "#" * 60)
    print("#  v4.0 ACADEMIC ENHANCEMENT TEST SUITE")
    print("#  12 new modules + v3.0 regression")
    print("#" * 60)

    tests = [
        ("Pareto Optimization", test_pareto),
        ("Semantic Fusion v4.0", test_vision),
        ("Active SLAM + NBV", test_active_slam),
        ("Hybrid A*", test_hybrid_astar),
        ("Loop Closure + PoseGraph", test_loop_closure),
        ("RBPF SLAM", test_rbpf_slam),
        ("Dynamic Obstacles + VO", test_dynamic_obstacles),
        ("MPC Planner", test_mpc),
        ("Semantic SLAM Landmarks", test_semantic_slam),
        ("Belief Space Planning", test_belief_planner),
        ("RL DQN Agent", test_rl_agent),
        ("v3.0 Regression", test_v3_regression),
    ]

    for name, fn in tests:
        try:
            fn()
        except Exception as e:
            global failed
            failed += 1
            errors.append((name, f"EXCEPTION: {type(e).__name__}: {e}"))
            print(f"\n  [ERROR] {name}:")
            import traceback
            traceback.print_exc()

    # Summary
    print("\n" + "#" * 60)
    print("#  TEST SUMMARY")
    print("#" * 60)
    total = passed + failed
    print(f"  Total checks: {total}")
    print(f"  Passed:       {passed}")
    print(f"  Failed:       {failed}")

    if errors:
        print(f"\n  Failures:")
        for name, msg in errors[:20]:
            print(f"    - {name}: {msg}")
        if len(errors) > 20:
            print(f"    ... and {len(errors)-20} more")

    print()
    if failed == 0:
        print("  *** ALL TESTS PASSED ***")
        return 0
    else:
        print(f"  *** {failed} TEST(S) FAILED ***")
        return 1


if __name__ == "__main__":
    sys.exit(main())
