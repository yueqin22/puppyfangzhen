#!/usr/bin/env python3
"""
Autonomous Navigation with Vision + Radar (Commercial-Grade Stack)
===================================================================
RUNTIME: research (CoppeliaSim research stack, NOT part of ROS2 product line)
产品化链路请使用 ROS2 bringup: ros2 launch puppy_bringup full_system.launch.py
本脚本运行在 CoppeliaSim + 纯 Python 环境，与 puppy_core / puppypi_adapter 隔离。

Architecture (Nav2-inspired):
  1. OccupancyGrid  — 0.1m Bresenham ray tracing (log-odds Bayesian)
  2. Costmap        — layered: static + obstacle + inflation
  3. A* Planner     — 8-connected global path planning + smoothing
  4. DWA Planner    — velocity-space sampling local planner
  5. State Machine  — PLAN → FOLLOW → RECOVER

Usage:
  python3 autonomous_nav.py
  Stream: http://localhost:8081/stream (first-person robot view)
"""
import os
import sys
import time
import math
import json
import tempfile
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

# Import commercial-grade navigation modules
from occupancy_grid import OccupancyGrid, GRID_RESOLUTION, GRID_W, GRID_H, ORIGIN_X, ORIGIN_Y
from costmap import Costmap, COST_LETHAL, COST_INSCRIBED
from astar_planner import AStarPlanner
from dwa_planner import DWAPlanner
from teb_planner import TEBPlanner
from odometry import Odometry
from amcl import AMCL
from evaluator import Evaluator, compute_map_metrics
from vision import SemanticMapper
from config_loader import load_config
from nav_core.exploration.frontier_manager import FrontierManager
from nav_core.runtime import NavState, NavigationRuntimeState
from nav_core.recovery.recovery_manager import RecoveryManager, RecoveryContext
from nav_core.planners.narrow_passage import NarrowPassageDetector

FRAME_PATH = os.environ.get(
    "PUPPY_STREAM_FRAME",
    os.path.join(tempfile.gettempdir(), "stream_frame.jpg"),
)
MAP_FILE = os.environ.get(
    "PUPPY_MAP_FILE",
    os.path.join(tempfile.gettempdir(), "puppy_map.npz"),
)
MAP_HTML_PATH = os.environ.get(
    "PUPPY_MAP_HTML",
    os.path.join(tempfile.gettempdir(), "puppy_map.html"),
)

# === Localization / Evaluation config (set via env for experiments) ===
# USE_AMCL=1  → use odometry+AMCL instead of CoppeliaSim ground truth
# USE_GROUNDTRUTH=1 → force ground-truth reads (default when USE_AMCL=0)
# METHOD_NAME=proposed → label for the evaluator (for comparison plots)
USE_AMCL = os.environ.get("USE_AMCL", "0") == "1"
USE_EVAL = os.environ.get("USE_EVAL", "1") == "1"
USE_SEMANTIC = os.environ.get("USE_SEMANTIC", "1") == "1"
USE_TEB = os.environ.get("USE_TEB", "0") == "1"  # 1=TEB planner, 0=DWA
METHOD_NAME = os.environ.get("METHOD_NAME", "proposed" if USE_AMCL else "groundtruth")
EVAL_DIR = os.environ.get("EVAL_DIR", os.path.join(tempfile.gettempdir(), "eval"))
MAX_FRAMES = int(os.environ.get("MAX_FRAMES", "0"))  # 0 = unlimited

# v2.7b: Deployment profile for hardware targets (e.g. Raspberry Pi 5)
# PROFILE=raspberry_pi loads config/profile_raspberry_pi.yaml and applies
# reduced-compute parameters (fewer AMCL particles, TEB iterations, LiDAR rays)
# to maintain 10Hz control frequency on Cortex-A76 ×4 @2.4GHz.
PROFILE = os.environ.get("PROFILE", "default")

# Load YAML configuration from config/*.yaml (parameter management)
CONFIG = load_config()

# Apply profile overrides if specified
# config_loader stores each YAML file as cfg[filename_without_ext], so
# PROFILE=raspberry_pi looks up cfg['profile_raspberry_pi'].
PROFILE_CFG = {}
if PROFILE != "default":
    profile_key = f"profile_{PROFILE}"  # e.g. "profile_raspberry_pi"
    PROFILE_CFG = CONFIG.get(profile_key, {})
    if PROFILE_CFG:
        print(f"[NAV] Loaded profile: {PROFILE} (from {profile_key}.yaml)")
        # Apply semantic override from profile
        if 'semantic' in PROFILE_CFG and 'enabled' in PROFILE_CFG['semantic']:
            USE_SEMANTIC = PROFILE_CFG['semantic']['enabled']
        print(f"[NAV] Profile config: {list(PROFILE_CFG.keys())}")
    else:
        print(f"[NAV] WARN: Profile '{PROFILE}' not found (looked for {profile_key}.yaml), using defaults")

print(f"[NAV] Loaded config: {list(CONFIG.keys())}")

client = RemoteAPIClient()
sim = client.getObject("sim")

# --- Robot parts ---
base = sim.getObject("/base_footprint")
base_link = sim.getObject("/base_link")
legs = {}
for name in ["fl", "fr", "rl", "rr"]:
    legs[name] = {
        "upper": sim.getObject(f"/leg_{name}_upper"),
        "lower": sim.getObject(f"/leg_{name}_lower"),
    }
print("Robot parts loaded")

# Reset to known good start position
sim.setObjectPosition(base, -1, [1.0, -2.0, 0.0])
sim.setObjectOrientation(base, -1, [0, 0, 0])
print("Robot reset to start position (1, -2, 0)")


# ============================================================
# 1. DYNAMIC OBSTACLE DISCOVERY + BOUNDING BOX EXTRACTION
# ============================================================
def discover_obstacles_with_bbox():
    """Find all solid objects and compute their world-space 2D bounding boxes."""
    obstacles = []
    all_objects = sim.getObjectsInTree(sim.handle_scene)
    skip_kw = ["leg_", "base_", "foot", "camera", "floor", "ground",
               "dock", "light", "sky", "ceiling", "roof", "navig"]
    obs_kw = ["wall", "sofa", "bed", "table", "chair", "couch", "shelf",
              "cabinet", "obstacle", "box", "pillar", "column", "desk",
              "wardrobe", "stove", "counter", "fridge", "tv"]
    for obj in all_objects:
        try:
            alias = sim.getObjectAlias(obj)
            al = alias.lower()
            if any(k in al for k in skip_kw):
                continue
            if any(k in al for k in obs_kw):
                mesh = sim.getShapeMesh(obj)
                if mesh and len(mesh) >= 1 and mesh[0]:
                    verts = mesh[0]
                    m = sim.getObjectMatrix(obj, -1)
                    wxs, wys = [], []
                    for i in range(0, len(verts), 3):
                        lx, ly, lz = verts[i], verts[i + 1], verts[i + 2]
                        wx = m[0] * lx + m[1] * ly + m[2] * lz + m[3]
                        wy = m[4] * lx + m[5] * ly + m[6] * lz + m[7]
                        wxs.append(wx)
                        wys.append(wy)
                    obstacles.append((alias, obj,
                                      (min(wxs), min(wys), max(wxs), max(wys))))
        except:
            pass
    return obstacles


obstacles = discover_obstacles_with_bbox()
print(f"[RADAR] Discovered {len(obstacles)} obstacles with bounding boxes:")
for alias, obj, (xmin, ymin, xmax, ymax) in obstacles:
    print(f"  {alias}: bbox x=[{xmin:.2f},{xmax:.2f}] y=[{ymin:.2f},{ymax:.2f}]")


def is_position_safe(x, y, radius=0.35):
    """Check if placing the robot at (x, y) would collide with any obstacle.

    Uses the real obstacle bounding boxes (from CoppeliaSim scene), not the
    costmap, because the costmap may be inaccurate when SLAM coverage is low
    or AMCL loc_err is high. This prevents setObjectPosition from clipping
    through walls during RECOVER's large-step escape.

    Args:
        x, y: target world position
        radius: robot collision radius (0.35m)

    Returns:
        True if position is safe (no collision with any obstacle)
    """
    for alias, _, (xmin, ymin, xmax, ymax) in obstacles:
        # Expanded AABB by robot radius
        if (x > xmin - radius and x < xmax + radius and
                y > ymin - radius and y < ymax + radius):
            return False
    return True


# ============================================================
# 2. FORWARD VISION SENSOR
# ============================================================
try:
    old_vs = sim.getObject("/patrol_camera")
    sim.removeObject(old_vs)
except:
    pass

CAM_W, CAM_H = 320, 240
angle_rad = 60.0 * math.pi / 180.0
vs_handle = sim.createVisionSensor(
    3, [CAM_W, CAM_H, 0, 0],
    [0.01, 30.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0]
)
sim.setObjectAlias(vs_handle, "patrol_camera")
print(f"[VISION] Forward camera {CAM_W}x{CAM_H}")

CAM_PITCH = 0.15


def update_forward_camera(rx, ry, rangle):
    cam_x = rx + 0.25 * math.cos(rangle)
    cam_y = ry + 0.25 * math.sin(rangle)
    sim.setObjectPosition(vs_handle, -1, [cam_x, cam_y, 0.55])
    sim.setObjectOrientation(vs_handle, -1, [0, math.pi / 2 + CAM_PITCH, rangle])


def capture_frame():
    sim.handleVisionSensor(vs_handle)
    try:
        result = sim.getVisionSensorImg(vs_handle)
    except Exception:
        try:
            result = sim.getVisionSensorImage(vs_handle)
        except Exception:
            return False
    if isinstance(result, (list, tuple)) and len(result) >= 2:
        img_data, res = result[0], result[1]
        if img_data and res and len(res) >= 2:
            w, h = res[0], res[1]
            try:
                img = Image.frombytes("RGB", (w, h), img_data)
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                os.makedirs(os.path.dirname(FRAME_PATH), exist_ok=True)
                img.save(FRAME_PATH, format="JPEG", quality=75)
                return True
            except Exception:
                pass
    return False


# ============================================================
# 3. RADAR: 360-degree laser scan via ray-AABB intersection
# ============================================================
def ray_aabb_dist(rx, ry, dx, dy, xmin, ymin, xmax, ymax):
    """Distance along ray (dx,dy) from (rx,ry) to AABB. Inf if miss."""
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-9:
        return float('inf')
    dx /= length
    dy /= length
    tmin, tmax = 0.0, float('inf')
    if abs(dx) < 1e-9:
        if rx < xmin or rx > xmax:
            return float('inf')
    else:
        t1, t2 = (xmin - rx) / dx, (xmax - rx) / dx
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
    if abs(dy) < 1e-9:
        if ry < ymin or ry > ymax:
            return float('inf')
    else:
        t1, t2 = (ymin - ry) / dy, (ymax - ry) / dy
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < tmin:
        return float('inf')
    return max(tmin, 0.0)


def laser_scan_full(rx, ry, max_range=8.0, margin=0.0, num_rays=72):
    """Cast 360-degree rays for occupancy grid mapping.

    Returns (angles, distances) arrays. num_rays rays at equal angular spacing.
    Margin=0: costmap inflation handles the safety margin (no double-counting).
    """
    angles = []
    distances = []
    for i in range(num_rays):
        angle = 2 * math.pi * i / num_rays
        dx, dy = math.cos(angle), math.sin(angle)
        min_d = max_range
        for alias, _, (xmin, ymin, xmax, ymax) in obstacles:
            d = ray_aabb_dist(rx, ry, dx, dy,
                              xmin - margin, ymin - margin,
                              xmax + margin, ymax + margin)
            if d < min_d:
                min_d = d
        angles.append(angle)
        distances.append(min_d)
    return angles, distances


def laser_scan_sectors(rx, ry, yaw, num_sectors=8, max_range=8.0, margin=0.0):
    """8-sector clearance for legacy display."""
    sector_angle = 2 * math.pi / num_sectors
    clearances = [float('inf')] * num_sectors
    for i in range(num_sectors):
        angle = yaw + i * sector_angle
        dx, dy = math.cos(angle), math.sin(angle)
        for alias, _, (xmin, ymin, xmax, ymax) in obstacles:
            d = ray_aabb_dist(rx, ry, dx, dy,
                              xmin - margin, ymin - margin,
                              xmax + margin, ymax + margin)
            if d < clearances[i]:
                clearances[i] = d
        if clearances[i] > max_range:
            clearances[i] = float('inf')
    return clearances


def sectors_from_full_scan(full_angles, full_distances, robot_yaw,
                           n_sectors=8, max_range=8.0):
    """Aggregate full LiDAR scan (72 rays) into 8 clearance sectors.

    Sector convention matches laser_scan_sectors:
      sector 0 = forward (rel angle  0)
      sector 2 = left    (rel angle +pi/2)
      sector 4 = back    (rel angle +pi)
      sector 6 = right   (rel angle +3pi/2)

    Reuses the 72-ray scan already computed for AMCL/mapping, avoiding
    8xN duplicate ray-AABB tests in laser_scan_sectors.

    Args:
        full_angles: world-frame ray angles [0, 2pi) from laser_scan_full
        full_distances: matching ray distances
        robot_yaw: robot heading in world frame
        n_sectors: number of angular sectors (default 8)
        max_range: rays at/above this range are treated as no-hit (skip)

    Returns:
        list of n_sectors min clearances (float('inf') if no ray hit)
    """
    sector_angle = 2 * math.pi / n_sectors
    sectors = [float('inf')] * n_sectors
    for angle, dist in zip(full_angles, full_distances):
        if dist >= max_range or not np.isfinite(dist) or dist < 0.1:
            continue
        rel = angle - robot_yaw
        # Normalize to [0, 2pi) so sector 0 = forward direction
        rel = (rel + 2 * math.pi) % (2 * math.pi)
        sector_idx = int(rel / sector_angle) % n_sectors
        if dist < sectors[sector_idx]:
            sectors[sector_idx] = dist
    return sectors


# ============================================================
# 4. LEG ANIMATION
# ============================================================
def animate_legs(t, phase_offset, moving):
    if not moving:
        for name in legs:
            sim.setObjectOrientation(legs[name]["upper"], base_link, [0, 0, 0])
            sim.setObjectOrientation(legs[name]["lower"], base_link, [0, 0, 0])
        return
    for name in legs:
        phase = phase_offset[name]
        cycle = (t * 4.0 + phase) % (2 * math.pi)
        hip = 0.3 * math.sin(cycle)
        knee = 0.5 * math.sin(cycle) if 0 < cycle < math.pi else 0
        sim.setObjectOrientation(legs[name]["upper"], base_link, [0, hip, 0])
        sim.setObjectOrientation(legs[name]["lower"], base_link, [0, knee, 0])


phase_offsets = {"fl": 0, "fr": math.pi, "rl": math.pi, "rr": 0}


# ============================================================
# 5. NAVIGATION STACK (Commercial-Grade Architecture)
# ============================================================
ROBOT_RADIUS = 0.35
SAVE_INTERVAL = 200

# Initialize modules
occ_grid = OccupancyGrid()
costmap = Costmap()
astar = AStarPlanner(costmap)
# P2-1: FrontierManager 从 exploration.yaml 读取评分参数
_exp_cfg = load_config().get('exploration', {})
frontier_manager = FrontierManager(occ_grid, costmap, config=_exp_cfg)
dwa = DWAPlanner(costmap)
# v2.7b: TEB planner accepts PROFILE_CFG for Raspberry Pi reduced-compute
# (n_iterations 4→2, n_poses 15→10 on Cortex-A76 to maintain 10Hz)
_p_teb = PROFILE_CFG.get('teb', {}) if PROFILE_CFG else {}
teb = TEBPlanner(costmap, profile_cfg=_p_teb) if USE_TEB else None
local_planner = teb if USE_TEB else dwa
print(f"[NAV] Local planner: {'TEB' if USE_TEB else 'DWA'}"
      + (f" (profile={PROFILE}: iter={_p_teb.get('n_iterations','default')}, poses={_p_teb.get('n_poses','default')})" if _p_teb else ""))

# v5.2: RecoveryManager 独立模块 — 从主脚本中提取的恢复逻辑
recovery_manager = RecoveryManager(costmap, config=_exp_cfg)
print("[NAV] RecoveryManager 已初始化 (v5.2)")

# v5.8: 狭窄通道检测器
narrow_passage_detector = NarrowPassageDetector(costmap, config=_exp_cfg)
print("[NAV] NarrowPassageDetector 已初始化 (v5.8)")

# Localization: odometry + AMCL (replaces ground-truth reads when USE_AMCL=1)
# n_particles=200 (v2.6b: reverted from 300, 300 caused slower convergence
# and higher early LocErr; 200 converges faster and is sufficient with the
# v2.6 lowered doorway landmark threshold of 0.3m)
#
# v2.9: n_obs_rays 24→36 — Run 5/6 showed AMCL loc_err frequently >1.0m,
# especially near doorways where 24 rays couldn't distinguish symmetric
# wall patterns. 36 rays (from 72 LiDAR rays) provide better angular
# resolution for doorway localization.
#
# v2.9c: n_particles 300→200 reverted — Run 8 (300p) performed worse than
# Run 7 (200p): cov 92.5% vs 93.8%, reached 1 vs 3. 300 particles increased
# per-frame compute, lowering update frequency; combined with higher alpha
# (0.10/0.05), particle cloud over-dispersed. 200p + z_rand=0.10 is optimal.
#
# v2.7b: PROFILE=raspberry_pi overrides n_particles=100, n_obs_rays=16
# to maintain 10Hz on Cortex-A76 (particle count is the biggest AMCL cost).
_p_amcl = PROFILE_CFG.get('amcl', {}) if PROFILE_CFG else {}
AMCL_PARTICLES = _p_amcl.get('n_particles', 200)
AMCL_OBS_RAYS = _p_amcl.get('n_obs_rays', 36)
AMCL_INIT_SPREAD = _p_amcl.get('init_spread', 0.2)
# v2.7b: LiDAR ray count overridable via PROFILE=raspberry_pi (72→36)
_p_laser = PROFILE_CFG.get('laser', {}) if PROFILE_CFG else {}
LASER_NUM_RAYS = _p_laser.get('num_rays', 72)
LASER_MAX_RANGE = _p_laser.get('max_range', 8.0)
# v2.7b: AMCL-PROACTIVE can be disabled on Raspberry Pi (recover() is costly)
_p_proactive = PROFILE_CFG.get('proactive_recover', {}) if PROFILE_CFG else {}
PROACTIVE_RECOVER_ENABLED = _p_proactive.get('enabled', True)
odom = Odometry(x=1.0, y=-2.0, yaw=0.0)
amcl = AMCL(occ_grid, n_particles=AMCL_PARTICLES, n_obs_rays=AMCL_OBS_RAYS)
amcl.init_cloud(1.0, -2.0, 0.0, spread=AMCL_INIT_SPREAD)
print(f"[LOC] Odometry + AMCL ready (USE_AMCL={USE_AMCL}, particles={AMCL_PARTICLES}, obs_rays={AMCL_OBS_RAYS})")

# Evaluation framework
evaluator = Evaluator(METHOD_NAME)
os.makedirs(EVAL_DIR, exist_ok=True)
print(f"[EVAL] Method='{METHOD_NAME}' output_dir={EVAL_DIR}")

# Semantic mapping (vision)
semantic_mapper = SemanticMapper(occ_grid) if USE_SEMANTIC else None
if semantic_mapper:
    print(f"[VISION] Semantic mapper ready")

# P1-3: Telemetry collector for observability (jihua1.md P1-3)
from nav_core.metrics.telemetry import TelemetryCollector
telemetry = TelemetryCollector(log_interval=50)
print(f"[TELEMETRY] Collector ready (log_interval=50 frames)")

# P1-4: Session store for map/frontier persistence (jihua1.md P1-4)
from nav_core.session.session_store import SessionStore
_session_map_file = MAP_FILE.replace('.pgm', '_session.npz') if MAP_FILE.endswith('.pgm') else MAP_FILE + '_session.npz'
session_store = SessionStore(_session_map_file)
session_store.save_params_summary({
    'method': METHOD_NAME, 'use_amcl': USE_AMCL, 'use_teb': USE_TEB,
    'particles': AMCL_PARTICLES, 'laser_rays': LASER_NUM_RAYS,
    'profile': PROFILE or 'default',
})
print(f"[SESSION] Store ready (map_file={_session_map_file})")

# Try to load previous map
if occ_grid.load(MAP_FILE):
    print(f"[MAP] Loaded previous map from {MAP_FILE}")
    costmap.update_static(occ_grid)
else:
    print("[MAP] Starting with empty map")

# P1-4: Load frontier memory from previous session (enables resume)
if session_store.load_frontier_memory(frontier_manager):
    print(f"[SESSION] Loaded frontier memory — resuming exploration")

# State machine
PLAN = NavState.PLAN
FOLLOW = NavState.FOLLOW
RECOVER = NavState.RECOVER
# Set of (gx, gy) grid coords for goals that were unreachable (robot reached
# the A* path end but not the goal itself). Prevents re-trying dead-ends.
# Dict {(gx, gy): frame} of frontiers the robot already reached. Prevents
# re-selecting the same frontier in a tight loop. Entries expire after
# VISITED_FRONTIER_TTL frames so the cell can be re-explored if the map
# changed (e.g. new area opened up nearby).
VISITED_FRONTIER_TTL = 250  # v2.6b: 200→250 (200 too short, caused premature re-selection)
# Count of consecutive "no new reachable frontier" cycles. When high, the map
# is likely fully explored — enter DONE state to stop spinning in place.
runtime = NavigationRuntimeState(last_pos=(1.0, -2.0))
runtime.no_frontier_cycles = 0
frontier_manager.visited_ttl = VISITED_FRONTIER_TTL
# DONE state: exploration complete, robot holds position
DONE = NavState.DONE
print(f"\n[NAV] === Commercial-Grade Navigation Stack ===")
print(f"[NAV] Grid: {GRID_W}x{GRID_H} @ {GRID_RESOLUTION}m")
print(f"[NAV] Obstacles: {len(obstacles)}  Robot radius: {ROBOT_RADIUS}m")
print(f"[NAV] Modules: OccupancyGrid + Costmap + A* + DWA")


# ============================================================
# 6. HTML MAP VISUALIZATION
# ============================================================
def update_map_html(rx, ry, ryaw, path=None, goal=None):
    """Generate HTML map showing occupancy grid + robot + path."""
    try:
        # Convert log-odds to displayable image
        prob = 1.0 - 1.0 / (1.0 + np.exp(occ_grid.log_odds))
        # Scale to 0-255: unknown=gray, free=white, occupied=black
        display = np.full((GRID_H, GRID_W, 3), 200, dtype=np.uint8)
        free_mask = prob < 0.3
        occ_mask = prob > 0.65
        display[free_mask] = [255, 255, 255]  # white = free
        display[occ_mask] = [40, 40, 40]      # black = occupied
        # Visited cells = light green
        visited_mask = occ_grid.visited
        display[visited_mask] = [200, 255, 200]

        # Mark path
        if path:
            for wx, wy in path:
                gx, gy = occ_grid.world_to_grid(wx, wy)
                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                    display[gy, gx] = [0, 0, 255]  # blue = path

        # Mark goal
        if goal:
            gx, gy = occ_grid.world_to_grid(goal[0], goal[1])
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        display[ny, nx] = [255, 0, 0]  # red = goal

        # Mark robot position
        rgx, rgy = occ_grid.world_to_grid(rx, ry)
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                if dx * dx + dy * dy <= 9:
                    nx, ny = rgx + dx, rgy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        display[ny, nx] = [0, 255, 0]  # green = robot

        img = Image.fromarray(display, 'RGB')
        img = img.resize((GRID_W * 6, GRID_H * 6), Image.NEAREST)
        img.save(MAP_HTML_PATH.replace('.html', '.png'), format='PNG')

        html = f"""<html><head><meta http-equiv="refresh" content="1">
<title>Occupancy Grid Map</title></head><body>
<h3>Occupancy Grid ({occ_grid.coverage_percent():.1f}% explored,
{occ_grid.visited_count()} visited)</h3>
<img src="puppy_map.png" width="{GRID_W * 6}" height="{GRID_H * 6}">
<p>State: {['PLAN', 'FOLLOW', 'RECOVER', 'DONE'][runtime.state]}
| Pos: ({rx:.2f}, {ry:.2f})</p>
</body></html>"""
        with open(MAP_HTML_PATH, 'w') as f:
            f.write(html)
    except Exception as e:
        pass


# ============================================================
# 7. MAIN NAVIGATION LOOP (State Machine)
# ============================================================
# Ensure clean state: stop any running simulation before starting
try:
    sim.stopSimulation()
    time.sleep(1)
except:
    pass
sim.startSimulation()
time.sleep(0.5)

# Recreate vision sensor after simulation restart (stopSimulation destroys
# objects created during the previous run, invalidating vs_handle)
try:
    old_vs = sim.getObject("/patrol_camera")
    sim.removeObject(old_vs)
except:
    pass
vs_handle = sim.createVisionSensor(
    3, [CAM_W, CAM_H, 0, 0],
    [0.01, 30.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0]
)
sim.setObjectAlias(vs_handle, "patrol_camera")
print(f"[VISION] Camera recreated after sim restart: {CAM_W}x{CAM_H} handle={vs_handle}")

t = 0
frame = 0

# Data collection for analysis
import csv
DATA_CSV = os.environ.get(
    "PUPPY_DATA_CSV",
    os.path.join(tempfile.gettempdir(), "puppy_nav_data.csv"),
)
data_file = open(DATA_CSV, "w", newline="")
data_writer = csv.writer(data_file)
data_writer.writerow([
    "frame", "time", "rx", "ry", "ryaw", "state",
    "coverage", "visited", "cost_at_robot",
    "fwd_dist", "left_dist", "right_dist", "back_dist",
    "dwa_v", "dwa_w", "no_progress",
    "goal_x", "goal_y", "dist_to_goal", "path_len",
    "true_x", "true_y", "loc_err",
])
print(f"[DATA] CSV data log: {DATA_CSV}")

# Track last DWA output and state transitions
last_dwa_v = 0.0
last_dwa_w = 0.0
last_no_progress = 0
prev_state = -1
doorway_crossings = 0
last_doorway_loc_frame = -100  # cooldown tracker for doorway landmark localization
doorway_crossing_frames = []  # v2.5: track frame numbers of crossings for oscillation detection
prev_true_y = -2.0  # robot starts at y=-2 (ground truth for doorway detection)
prev_true_pose = None  # for AMCL motion update (delta from last frame)
last_loc_err = 0.0
last_loc_conf = 1.0
# v2.7: FOLLOW-OSC cooldown — prevent consecutive goal aborts that trap the
# robot in a "select frontier → abort → select frontier → abort" loop when
# doorway_crossing_frames history keeps recent_oscillation=True.
last_follow_osc_frame = -100
# v2.7: AMCL proactive drift recovery — trigger recover when loc_err exceeds
# 0.5m even without doorway crossing, to catch drift before it causes
# doorway oscillation. v2.7b: 150-frame cooldown + frame>50 startup delay
# + spread=0.3 (was 0.5) for stable convergence.
last_proactive_recover_frame = -100
# v2.8: AMCL adaptive recovery — escalate spread when consecutive recovery
# attempts fail to bring loc_err below 0.5m. After 3 failures, trigger
# global relocalization (spread=5.0) to escape particle filter local minima.
consecutive_recover_failures = 0
# v2.8: Track frames spent idle in DONE state for patrol resume logic
done_frames = 0
# v2.9j: Track consecutive no_progress RECOVERs for area blocking.
# When 3+ no_progress RECOVERs happen within 2m of each other, the entire
# area is marked unreachable (3m radius) to prevent frontier death loops.
_consecutive_noprog_recovers = []  # list of (x, y) positions

def reconnect_zmq():
    """Reconnect to CoppeliaSim ZMQ remote API."""
    global client, sim
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        client = RemoteAPIClient()
        sim = client.getObject('sim')
        print("[NAV] ZMQ reconnected")
        return True
    except Exception as e:
        print(f"[WARN] ZMQ reconnect failed: {e}")
        return False

try:
    while True:
        try:
            # P1-3: Telemetry frame timing
            telemetry.frame_start()
            # Increment frame counter early so errors don't cause infinite loop
            # (previously frame += 1 was at the end, causing Frame 0 to repeat forever
            # when update_forward_camera threw an exception)
            t += 0.1
            frame += 1
            # --- 1. Get robot position ---
            # Ground truth (always read for evaluation; used as pose when USE_AMCL=0)
            curr = sim.getObjectPosition(base, -1)
            curr_ori = sim.getObjectOrientation(base, -1)
            true_x, true_y, true_yaw = curr[0], curr[1], curr_ori[2]

            if USE_AMCL:
                # Use odometry + AMCL for navigation pose
                # Odometry is updated AFTER motion command (see step 5 below),
                # so here we use the AMCL-corrected estimate from last frame.
                rx, ry, ryaw, loc_conf = amcl.get_estimate()
                loc_err = math.sqrt((rx - true_x) ** 2 + (ry - true_y) ** 2)
            else:
                # Ground-truth mode (baseline / comparison)
                rx, ry, ryaw = true_x, true_y, true_yaw
                loc_conf = 1.0
                loc_err = 0.0

            # State transition log
            if runtime.state != prev_state:
                state_names = ['PLAN', 'FOLLOW', 'RECOVER', 'DONE']
                print(f"[STATE] {state_names[prev_state] if prev_state >= 0 else 'INIT'}"
                      f" -> {state_names[runtime.state]} at ({rx:.2f},{ry:.2f}) "
                      f"cov={occ_grid.coverage_percent():.1f}% "
                      f"no_progress={runtime.no_progress_frames}"
                      f"{' loc_err=%.3f' % loc_err if USE_AMCL else ''}")
                prev_state = runtime.state

            # Track doorway crossings (y crossing 0) — use ground-truth y to avoid
            # AMCL jitter near y=0 producing false doorway crossings (Bug S5).
            if (prev_true_y < 0 and true_y >= 0) or (prev_true_y >= 0 and true_y < 0):
                doorway_crossings += 1
                doorway_crossing_frames.append(frame)  # v2.5: track for oscillation detection
                print(f"[DOORWAY] Crossed y=0 ({'S->N' if true_y >= 0 else 'N->S'}) "
                      f"at ({rx:.2f},{ry:.2f}) crossing #{doorway_crossings} "
                      f"cov={occ_grid.coverage_percent():.1f}%")
                # Doorway landmark re-localization: the doorway (y=0 corridor
                # between wall_divide_1 and wall_divide_2) is a strong geometric
                # landmark. After crossing, trigger a local AMCL recover with
                # small spread to re-converge using the distinctive LiDAR
                # pattern (walls on both sides + open corridor ahead/behind).
                # This corrects accumulated odometry drift from long-distance
                # travel between rooms.
                #
                # v2.3 fix: add 50-frame cooldown to prevent repeated recover
                # when the robot oscillates near y=0 (multiple crossings in
                # quick succession each triggered recover, causing AMCL to
                # diverge from constant particle cloud resets).
                #
                # v2.9: Only trigger doorway re-localization when loc_err is in
                # the "moderate drift" range (0.3-1.0m). When loc_err > 1.0m, the
                # AMCL position estimate is too far from truth — using it as the
                # center for particle re-spreading makes things worse (Run 5:
                # crossing #1 loc_err=0.6→recover→2.4). High loc_err is left to
                # the RECOVER state's adaptive recovery (spread 1.0→3.0→5.0).
                if (USE_AMCL and (frame - last_doorway_loc_frame) > 50
                        and 0.3 < last_loc_err < 1.0):
                    amcl.recover(rx, ry, ryaw, spread=0.3)
                    last_doorway_loc_frame = frame
                    print(f"[DOORWAY-LOC] Triggered AMCL re-localization "
                          f"(spread=0.3m, loc_err={last_loc_err:.3f}) "
                          f"at doorway crossing #{doorway_crossings}")
            prev_true_y = true_y

            # v2.7: Proactive AMCL drift recovery — when loc_err exceeds 0.5m
            # (even without doorway crossing), trigger recover to re-converge
            # the particle cloud. This catches drift BEFORE it causes doorway
            # oscillation (v2.6b run showed loc_err=1.4m at first crossing,
            # meaning AMCL had already drifted significantly during FOLLOW).
            #
            # v2.7b: Three parameter fixes based on v2.7 run analysis:
            #  1. Startup delay frame > 50 — v2.7 triggered at frame 15 (cov
            #     25.9%) when AMCL was still initializing, causing 6 triggers
            #     that failed to reduce LocErr (1.18m vs v2.6b's 0.72m).
            #     Waiting for frame 50 lets AMCL converge from init_cloud
            #     before any proactive reset.
            #  2. spread 0.5→0.3 — v2.7's spread=0.5 was too wide, scattering
            #     particles across 0.5m radius which prevented re-convergence.
            #     0.3 matches the doorway landmark recovery spread and keeps
            #     particles concentrated near the odometry pose.
            #  3. Cooldown 100→150 frames — v2.7's 100-frame cooldown allowed
            #     6-7 triggers per run, each disrupting AMCL convergence.
            #     150 frames (15s at 10Hz) gives AMCL more time to settle
            #     between proactive resets.
            # Only trigger during FOLLOW state (PLAN/DONE don't need it,
            # RECOVER has its own AMCL recovery). This prevents wasteful
            # proactive resets when the robot is idle in DONE state.
            if (USE_AMCL and PROACTIVE_RECOVER_ENABLED and runtime.state == FOLLOW
                    and frame > 100
                    and (frame - last_proactive_recover_frame) > 150
                    and (frame - last_doorway_loc_frame) > 50
                    and last_loc_err > 0.5):
                amcl.recover(rx, ry, ryaw, spread=0.3)
                last_proactive_recover_frame = frame
                print(f"[AMCL-PROACTIVE] Triggered drift recovery "
                      f"(spread=0.3m, loc_err={last_loc_err:.3f}>0.5) "
                      f"at frame {frame} state={['PLAN','FOLLOW','RECOVER','DONE'][runtime.state]}")

            # --- 2. LiDAR scan (360 degrees, 72 rays) ---
            # v2.7b: PROFILE=raspberry_pi uses 36 rays to halve Bresenham cost
            # v2.9e: Use true position for LiDAR scan — the physical sensor is
            # at the true position, not the AMCL estimate. Previously rx,ry was
            # used, but that was approximately correct only because setObjectPosition
            # set the robot to AMCL_position+delta (causing teleportation). Now
            # that movement uses true position, the scan must also use true position.
            angles, distances = laser_scan_full(true_x, true_y, num_rays=LASER_NUM_RAYS)

            # --- 3. Update occupancy grid + costmap ---
            # v2.9e: Ray trace from true position (where the scan was taken).
            # This builds a correct map. AMCL still has uncertainty from
            # odometry noise + particle filter randomness.
            occ_grid.update_from_scan(true_x, true_y, angles, distances, max_range=8.0)
            occ_grid.mark_visited(true_x, true_y)
            costmap.update_static(occ_grid, frame=frame)
            costmap.update_obstacles(true_x, true_y, angles, distances, max_range=8.0)

            # v5.8: 狭窄通道检测
            narrow_passage_detector.update(true_x, true_y, true_yaw,
                                           angles, distances, frame=frame)
            # 如果在窄通道中，调整局部规划器参数
            if narrow_passage_detector.is_narrow and runtime.state == FOLLOW:
                adjusted = narrow_passage_detector.get_adjusted_params()
                if hasattr(local_planner, 'max_v') and adjusted:
                    # 临时降低速度上限
                    _orig_max_v = getattr(local_planner, 'max_v', None)
                    if _orig_max_v is not None:
                        local_planner.max_v = adjusted.get('max_v', _orig_max_v)
            else:
                # 恢复正常速度（如果之前降低过）
                if hasattr(local_planner, 'max_v'):
                    if hasattr(local_planner, '_default_max_v'):
                        local_planner.max_v = local_planner._default_max_v
                    elif not hasattr(local_planner, '_default_max_v_set'):
                        local_planner._default_max_v = local_planner.max_v
                        local_planner._default_max_v_set = True

            # --- 4. Camera + leg animation ---
            update_forward_camera(true_x, true_y, true_yaw)
            rgb_ok = capture_frame()
            moving = (runtime.state == FOLLOW)
            animate_legs(t, phase_offsets, moving)

            # --- 5. State machine ---
            if runtime.state == PLAN:
                # Give up on current goal after too many failed replan attempts
                if runtime.current_goal is not None and runtime.goal_plan_attempts >= 3:
                    print(f"[PLAN] Giving up on goal ({runtime.current_goal[0]:.1f},"
                          f"{runtime.current_goal[1]:.1f}) after {runtime.goal_plan_attempts} attempts")
                    # Mark as unreachable (with spatial zone) so we don't re-pick it
                    frontier_manager.mark_unreachable(
                        runtime.current_goal[0],
                        runtime.current_goal[1],
                    )
                    telemetry.record_goal_unreachable()
                    runtime.clear_goal(clear_path=False)

                # Try to continue toward current goal first (prevents frontier
                # oscillation after RECOVER). Only pick a new frontier if the
                # current goal is unreachable or cleared.
                if runtime.current_goal is not None:
                    runtime.goal_plan_attempts += 1
                    path = astar.plan(rx, ry, runtime.current_goal[0], runtime.current_goal[1])
                    if path and len(path) > 0:
                        runtime.start_following(path)
                        wps = " ".join(f"({wx:.1f},{wy:.1f})" for wx, wy in path[:5])
                        print(f"[PLAN] Continue goal ({runtime.current_goal[0]:.1f},"
                              f"{runtime.current_goal[1]:.1f}) path={len(path)}wp [{wps}] "
                              f"cov={occ_grid.coverage_percent():.1f}%")
                    else:
                        # Can't reach current goal — clear and pick new frontier
                        runtime.clear_goal(clear_path=False)

                if runtime.current_goal is None:
                    # Find frontiers with information gain (utility-based selection)
                    # info_gain = unknown cells within sensor range of frontier
                    # utility = α*info_gain_norm - β*distance_norm
                    frontiers = occ_grid.find_frontiers_with_info_gain(
                        rx, ry, sensor_range=4.0, max_frontiers=15)

                    # Filter out frontiers whose grid cell is already marked
                    # unreachable (we previously reached the A* path end without
                    # reaching the goal itself — these are dead-ends), or was
                    # visited recently (prevents reach/re-plan/reach loops).
                    # v2.6: Also filter out frontiers on the opposite side of
                    # the doorway (y=0) when the robot recently oscillated near
                    # the doorway. This prevents re-selecting a frontier that
                    # requires crossing the doorway again, which would trigger
                    # another oscillation cycle.
                    # v2.6b: 60→40 frames cooldown (60 was too restrictive,
                    # caused coverage drop 93.26%→92.19%)
                    reachable_frontiers = frontier_manager.filter_frontiers_with_metadata(
                        frontiers,
                        current_frame=frame,
                        robot_y=true_y,
                        doorway_crossing_frames=doorway_crossing_frames,
                    )
                    if frame % 100 == 0:
                        frontier_manager.gc_visited(frame)

                    if reachable_frontiers:
                        runtime.reset_frontier_cycles()

                        # v2.9i: Batch A* attempts — try up to 8 frontiers in
                        # a single PLAN frame. Previously, when A* failed for
                        # a frontier, the robot waited until the next frame to
                        # try another. With 444 frontiers (many unreachable),
                        # this wasted hundreds of frames just marking them one
                        # by one. Now we try up to 8 in one frame, drastically
                        # reducing PLAN-state stalls and A* total time.
                        max_astar_attempts = min(8, len(reachable_frontiers))
                        _astar_total_t = 0.0
                        _astar_attempts = 0
                        _found_path = False

                        for _attempt in range(max_astar_attempts):
                            selection = frontier_manager.select_frontier(
                                reachable_frontiers,
                                rx=rx,
                                ry=ry,
                                robot_yaw=ryaw,
                            )
                            goal = selection.goal
                            gi = selection.info_gain
                            gd = selection.distance

                            if not goal:
                                # All remaining frontiers are unsafe
                                break

                            # Plan path with A*
                            _t_astar = time.time()
                            path = astar.plan(rx, ry, goal[0], goal[1])
                            _dt_astar = time.time() - _t_astar
                            _astar_total_t += _dt_astar
                            _astar_attempts += 1
                            telemetry.record_astar(_dt_astar, path is not None)

                            if path and len(path) > 0:
                                # v5.7: 计算路径质量评分
                                path_quality = frontier_manager.compute_path_quality(path)
                                # v6.12: 如果启用Pareto选择，使用它替代
                                # (当前通过 config use_pareto_selection 控制)

                                runtime.current_goal = goal
                                runtime.start_following(path, goal_plan_attempts=1)
                                wps = " ".join(f"({wx:.1f},{wy:.1f})" for wx, wy in path[:5])
                                print(f"[PLAN] Frontier ({goal[0]:.1f},{goal[1]:.1f}) "
                                      f"info_gain={gi} dist={gd:.1f}m score={selection.score:.3f} "
                                      f"path_q={path_quality:.2f} "
                                      f"path={len(path)}wp [{wps}] "
                                      f"cov={occ_grid.coverage_percent():.1f}% "
                                      f"(try {_attempt+1}/{_astar_attempts})")
                                _found_path = True
                                break
                            else:
                                # Frontier exists but A* can't reach it — mark it
                                # unreachable so we don't re-pick it next cycle.
                                frontier_manager.mark_unreachable(goal[0], goal[1])
                                # Remove from list so select_frontier won't re-pick
                                reachable_frontiers = [f for f in reachable_frontiers
                                                       if abs(f[0]-goal[0]) > 0.01
                                                       or abs(f[1]-goal[1]) > 0.01]
                                if len(reachable_frontiers) == 0:
                                    break

                        if not _found_path and _astar_attempts > 1:
                            print(f"[PLAN] Tried {_astar_attempts} frontiers, all unreachable. "
                                  f"astar_total={_astar_total_t*1000:.1f}ms "
                                  f"cov={occ_grid.coverage_percent():.1f}%")
                        elif not _found_path and _astar_attempts == 0:
                            if frame % 50 == 0:
                                print(f"[PLAN] All {len(reachable_frontiers)} frontiers "
                                      f"unsafe. cov={occ_grid.coverage_percent():.1f}%")
                    else:
                        # No reachable frontiers (all filtered or none found).
                        # KEY FIX: previously this branched to RECOVER for the first
                        # 5 cycles, which caused PLAN<->RECOVER oscillation (RECOVER
                        # moves the robot to open space, then PLAN finds no frontier
                        # again, so it RECOVERs again... robot drifts across the
                        # room without exploring). Now we go DONE after just 1
                        # recovery attempt: if one RECOVER didn't expose a new
                        # frontier, more won't either (the map is genuinely
                        # explored).
                        runtime.no_frontier_cycles += 1
                        if runtime.no_frontier_cycles > 1:
                            runtime.mark_done()
                            print(f"[PLAN] Exploration complete: "
                                  f"{len(frontier_manager.unreachable_goals)} unreachable goals, "
                                  f"coverage={occ_grid.coverage_percent():.1f}%")
                        else:
                            # First time no frontier — one RECOVER to shift position
                            # (may reveal new area). If still nothing, go DONE.
                            print(f"[PLAN] No reachable frontiers — "
                                  f"trying RECOVER to reveal new area. "
                                  f"cov={occ_grid.coverage_percent():.1f}% "
                                  f"unreachable={len(frontier_manager.unreachable_goals)}")
                            runtime.start_recovery()
                            telemetry.record_recover('no_frontier')
                            _consecutive_noprog_recovers.clear()  # v2.9j: reset

            elif runtime.state == FOLLOW:
                # Prune passed waypoints (within 0.3m of robot)
                if runtime.current_path and len(runtime.current_path) > 1:
                    while len(runtime.current_path) > 1:
                        wx, wy = runtime.current_path[0]
                        if math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2) < 0.3:
                            runtime.current_path = runtime.current_path[1:]
                        else:
                            break

                # Check if goal reached
                if runtime.current_goal:
                    dist_to_goal = math.sqrt(
                        (rx - runtime.current_goal[0]) ** 2 + (ry - runtime.current_goal[1]) ** 2)
                    if dist_to_goal < 0.5:
                        vgx, vgy = occ_grid.world_to_grid(
                            runtime.current_goal[0], runtime.current_goal[1])
                        # If we've already visited this frontier before, it's a
                        # persistent frontier (wall blocks observation of the
                        # unknown side) — mark as unreachable to stop re-visiting.
                        if (vgx, vgy) in frontier_manager.visited_frontiers:
                            frontier_manager.mark_unreachable(
                                runtime.current_goal[0],
                                runtime.current_goal[1],
                            )
                            print(f"[FOLLOW] Re-visited goal ({runtime.current_goal[0]:.1f},"
                                  f"{runtime.current_goal[1]:.1f}) - marking unreachable "
                                  f"(persistent frontier behind wall)")
                        else:
                            print(f"[FOLLOW] Reached goal ({runtime.current_goal[0]:.1f},"
                                  f"{runtime.current_goal[1]:.1f})")
                        frontier_manager.mark_visited(
                            runtime.current_goal[0],
                            runtime.current_goal[1],
                            frame,
                        )
                        telemetry.record_goal_reached()
                        runtime.return_to_planning()
                        continue

                # v2.5 Doorway oscillation detection in FOLLOW state:
                # If the robot crosses y=0 multiple times in a window while near
                # the doorway, TEB is oscillating in the narrow corridor.
                # Abort the current goal and replan to break the cycle.
                #
                # v2.9: Retuned from 3/40 to 5/60 after Run 4 showed reached=0
                # (every goal aborted by FOLLOW-OSC). The 3/40 threshold was
                # too sensitive — normal approach adjustments near the doorway
                # (e.g. aligning to pass through) can produce 3 crossings in
                # 40 frames without being truly stuck. 5/60 only triggers on
                # genuine oscillation. Also added goal-distance guard: if the
                # robot is within 1.5m of the goal, don't abort (it's close
                # enough to finish).
                #
                # v2.7: 30-frame cooldown after each FOLLOW-OSC trigger +
                # clear doorway_crossing_frames on trigger (see below).
                if (runtime.current_goal and abs(true_y) < 1.0 and abs(rx) < 2.5
                        and (frame - last_follow_osc_frame) > 30):
                    recent_crossings = sum(1 for f in doorway_crossing_frames
                                           if frame - f <= 60)
                    # v2.9: Don't abort if robot is close to goal — let it finish
                    dist_to_goal = math.sqrt(
                        (rx - runtime.current_goal[0]) ** 2 +
                        (ry - runtime.current_goal[1]) ** 2)
                    if recent_crossings >= 5 and dist_to_goal > 1.5:
                        print(f"[FOLLOW-OSC] Doorway oscillation: {recent_crossings} "
                              f"crossings in 60 frames — aborting goal "
                              f"({runtime.current_goal[0]:.1f},{runtime.current_goal[1]:.1f}) "
                              f"dist={dist_to_goal:.1f}m")
                        frontier_manager.mark_unreachable(
                            runtime.current_goal[0],
                            runtime.current_goal[1],
                        )
                        # v2.7: Clear crossing history so PLAN can select a
                        # frontier on either side without the oscillation
                        # filter blocking it.
                        doorway_crossing_frames = []
                        last_follow_osc_frame = frame
                        runtime.return_to_planning(reset_no_progress=True)
                        continue

                # Path-end detection: robot reached the end of the A* path but
                # the goal itself is still far. This means A* could only plan
                # partway (goal is behind obstacles / in unknown space). Mark
                # the goal as unreachable so PLAN won't re-pick it, and replan.
                if runtime.current_path and runtime.current_goal:
                    last_wx, last_wy = runtime.current_path[-1]
                    dist_to_path_end = math.sqrt(
                        (rx - last_wx) ** 2 + (ry - last_wy) ** 2)
                    dist_to_goal_now = math.sqrt(
                        (rx - runtime.current_goal[0]) ** 2 + (ry - runtime.current_goal[1]) ** 2)
                    if dist_to_path_end < 0.4 and dist_to_goal_now > 0.6:
                        frontier_manager.mark_unreachable(
                            runtime.current_goal[0],
                            runtime.current_goal[1],
                        )
                        print(f"[FOLLOW] Path end reached but goal "
                              f"({runtime.current_goal[0]:.1f},{runtime.current_goal[1]:.1f}) "
                              f"still {dist_to_goal_now:.2f}m away — marking "
                              f"unreachable")
                        runtime.return_to_planning(reset_no_progress=True)
                        continue

                # Local planner (DWA or TEB) compute velocity
                if runtime.current_path:
                    _t_dwa = time.time()
                    v, w = local_planner.compute_velocity(
                        rx, ry, ryaw, runtime.current_path,
                        runtime.current_goal[0] if runtime.current_goal else rx,
                        runtime.current_goal[1] if runtime.current_goal else ry)
                    telemetry.record_dwa(time.time() - _t_dwa, 1.0)
                    last_dwa_v = v
                    last_dwa_w = w
                    last_no_progress = runtime.no_progress_frames

                    if v == 0.0 and w == 0.0:
                        # Goal reached or stuck
                        runtime.return_to_planning(clear_goal=False)
                    elif v == 0.0 and abs(w) > 0:
                        # Rotating in place (alignment) — don't count as no-progress
                        curr_ori[2] += w * 0.1
                        sim.setObjectOrientation(base, -1, curr_ori)
                        runtime.align_frames += 1
                        if runtime.align_frames > 40:
                            # Stuck in alignment too long, try recovery
                            # Tuned v2.3: 30→40 (give TEB more time to converge)
                            runtime.start_recovery(reset_align=True)
                            telemetry.record_recover('align_timeout')
                            _consecutive_noprog_recovers.clear()  # v2.9j: reset
                    else:
                        # Move forward
                        # v2.9e: Use true position as base for setObjectPosition.
                        # Previously new_x = rx + dx (AMCL position + delta), but
                        # setObjectPosition sets ABSOLUTE world position. When AMCL
                        # drifts (loc_err up to 4m), the robot teleports to where
                        # AMCL thinks it is. Fix: apply delta to true_x/true_y.
                        # Costmap checks still use AMCL position (rx, ry) because
                        # that's the planner's perception of the world.
                        dx, dy, dyaw = local_planner.velocity_to_step(v, w, ryaw)
                        plan_x = rx + dx       # AMCL-frame target (for costmap)
                        plan_y = ry + dy
                        move_x = true_x + dx   # True-frame target (for movement)
                        move_y = true_y + dy
                        move_yaw = true_yaw + dyaw

                        # Gradient-descent safety: when in a high-cost zone,
                        # only allow moves that DECREASE cost (prevents driving
                        # deeper into obstacles). When in safe zone, reject
                        # moves into inscribed zone as before.
                        cur_cost = costmap.get_cost(rx, ry)
                        new_cost = costmap.get_cost(plan_x, plan_y)
                        if cur_cost >= COST_INSCRIBED:
                            # In bad zone: only move if new position is better
                            allow_move = new_cost < cur_cost
                        else:
                            # In safe zone: don't enter inscribed zone
                            allow_move = new_cost < COST_INSCRIBED
                        # v2.9d: Real obstacle collision check on ACTUAL target
                        if allow_move and not is_position_safe(move_x, move_y):
                            allow_move = False
                        if allow_move:
                            sim.setObjectPosition(base, -1, [move_x, move_y, curr[2]])
                            sim.setObjectOrientation(base, -1, [0, 0, move_yaw])
                            # Check progress (use true position)
                            move_dist = math.sqrt(
                                (move_x - runtime.last_pos[0]) ** 2 + (move_y - runtime.last_pos[1]) ** 2)
                            if move_dist > 0.02:
                                runtime.no_progress_frames = 0
                                runtime.align_frames = 0
                                runtime.last_pos = (move_x, move_y)
                            else:
                                runtime.no_progress_frames += 1
                        else:
                            # Blocked - try turning
                            curr_ori[2] += w * 0.1
                            sim.setObjectOrientation(base, -1, curr_ori)
                            runtime.no_progress_frames += 1

                        # Lower threshold for RECOVER when in lethal zone (faster escape)
                        # v2.9h: safe zone 40→60 (6s) — give TEB more time to navigate
                        # narrow passages and doorways. 40 frames (4s) was too aggressive,
                        # causing 12/13 RECOVERs from no_progress in v2.9g 2000-frame run.
                        recover_threshold = 15 if cur_cost >= COST_LETHAL else 60
                        if runtime.no_progress_frames > recover_threshold:
                            # v2.9h: Mark current goal as unreachable before
                            # recovering. Without this, start_recovery() keeps
                            # current_goal set, and PLAN re-attempts the same
                            # unreachable goal after RECOVER — death loop.
                            if runtime.current_goal is not None:
                                frontier_manager.mark_unreachable(
                                    runtime.current_goal[0],
                                    runtime.current_goal[1])
                                print(f"[FOLLOW] no_progress RECOVER — marking goal "
                                      f"({runtime.current_goal[0]:.1f},"
                                      f"{runtime.current_goal[1]:.1f}) unreachable")
                                runtime.clear_goal(clear_path=True)
                            # v2.9j: Track consecutive no_progress RECOVERs.
                            # If 3+ happen within 2m, block the entire area
                            # (3m radius) to stop frontier death loops along
                            # wall edges where frontier clusters regenerate.
                            _consecutive_noprog_recovers.append((true_x, true_y))
                            if len(_consecutive_noprog_recovers) >= 3:
                                # Check if all recent RECOVERs are within 2m
                                recent = _consecutive_noprog_recovers[-3:]
                                cx = sum(p[0] for p in recent) / 3
                                cy = sum(p[1] for p in recent) / 3
                                max_dist = max(math.sqrt(
                                    (p[0]-cx)**2 + (p[1]-cy)**2
                                ) for p in recent)
                                if max_dist < 2.0:
                                    frontier_manager.mark_area_unreachable(
                                        cx, cy, radius_m=3.0)
                                    print(f"[FOLLOW] 3x no_progress RECOVER in "
                                          f"{max_dist:.1f}m cluster at "
                                          f"({cx:.1f},{cy:.1f}) — blocking "
                                          f"3m area")
                                    _consecutive_noprog_recovers.clear()
                            runtime.start_recovery(reset_align=True)
                            telemetry.record_recover('no_progress')

            elif runtime.state == RECOVER:
                # v2.9f: Track whether particle re-spread was done at frame 5
                # (controls whether spin at frames 6-8 should execute)
                if runtime.recover_frames == 0:
                    _amcl_respread_done = False
                # v2.5 Doorway Oscillation Suppression: detect when the robot
                # oscillates near the doorway (multiple y=0 crossings in recent
                # frames). When detected, bias RECOVER direction away from y=0
                # to break the oscillation cycle. v2.4's doorway-jump assist
                # was removed (it never triggered and AMCL sync caused drift).
                robot_cost = costmap.get_cost(rx, ry)
                in_lethal = robot_cost >= COST_LETHAL
                near_doorway = abs(true_y) < 0.8 and abs(rx) < 2.5
                # Count recent doorway crossings (last 40 frames)
                recent_crossings = sum(1 for f in doorway_crossing_frames
                                       if frame - f <= 40)
                oscillating = near_doorway and recent_crossings >= 3
                if oscillating:
                    print(f"[DOORWAY-OSC] Detected oscillation: {recent_crossings} "
                          f"crossings in 40 frames at ({rx:.2f},{true_y:.2f})")

                # Smart escape: sample 16 directions, prefer path-aligned + low cost.
                best_dir = None
                best_score = -float('inf')
                # Adaptive step: when in lethal zone (cost=254), use larger step
                # to escape the 0.55m inflation radius. Normal zone uses 0.15m.
                # v2.5: when oscillating near doorway, use larger step to break free
                step = 0.6 if (in_lethal or oscillating) else 0.15
                # Compute path direction (if available) for alignment scoring
                path_ref_dir = None
                if runtime.current_path and len(runtime.current_path) >= 2:
                    # Find nearest segment
                    min_d = float('inf')
                    nearest_seg = 0
                    for i in range(len(runtime.current_path) - 1):
                        ax, ay = runtime.current_path[i]
                        bx, by = runtime.current_path[i + 1]
                        seg_dx = bx - ax
                        seg_dy = by - ay
                        seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
                        if seg_len2 < 1e-9:
                            continue
                        proj_t = ((rx - ax) * seg_dx + (ry - ay) * seg_dy) / seg_len2
                        proj_t = max(0.0, min(1.0, proj_t))
                        proj_x = ax + proj_t * seg_dx
                        proj_y = ay + proj_t * seg_dy
                        d = math.sqrt((rx - proj_x) ** 2 + (ry - proj_y) ** 2)
                        if d < min_d:
                            min_d = d
                            nearest_seg = i
                            if d > 0.3:
                                # Far from path: head toward projection
                                path_ref_dir = math.atan2(proj_y - ry, proj_x - rx)
                            else:
                                # On path: use segment direction
                                path_ref_dir = math.atan2(seg_dy, seg_dx)
                # Backward direction (opposite of robot heading) — preferred when
                # stuck in lethal zone because the robot came from a safe direction.
                backward_dir = ryaw + math.pi
                # v2.5: oscillation escape direction — perpendicular to doorway
                # (pure y-axis) to move away from y=0 line
                osc_escape_dir = math.pi / 2 if true_y >= 0 else -math.pi / 2
                for i in range(16):  # 16 directions for finer resolution
                    ang = i * math.pi / 8
                    # Skip direction opposite to last movement (prevent oscillation)
                    # But DON'T skip when in lethal zone — need all options
                    if not in_lethal and runtime.last_recover_dir is not None:
                        diff = abs(ang - runtime.last_recover_dir)
                        diff = min(diff, 2 * math.pi - diff)
                        if diff > math.pi * 0.75:  # >135° = roughly opposite
                            continue
                    tx = rx + step * math.cos(ang)
                    ty = ry + step * math.sin(ang)
                    c = costmap.get_cost(tx, ty)
                    # Hard filter: don't move into lethal or inscribed cost
                    if c >= COST_INSCRIBED:
                        continue
                    # Score: lower cost = better; alignment with path = better
                    cost_score = 1.0 - (c / COST_INSCRIBED)  # 0..1
                    if oscillating:
                        # v2.5: when oscillating, prioritize moving AWAY from
                        # doorway (perpendicular y direction) to break the cycle
                        osc_diff = abs(ang - osc_escape_dir)
                        osc_diff = min(osc_diff, 2 * math.pi - osc_diff)
                        osc_score = 1.0 - osc_diff / math.pi  # 0..1
                        combined = 0.5 * cost_score + 0.5 * osc_score
                    elif in_lethal:
                        # In lethal zone: prioritize ESCAPING (lower cost) and
                        # going backward (where the robot came from = safe)
                        back_diff = abs(ang - backward_dir)
                        back_diff = min(back_diff, 2 * math.pi - back_diff)
                        back_score = 1.0 - back_diff / math.pi  # 0..1
                        combined = 0.7 * cost_score + 0.3 * back_score
                    elif path_ref_dir is not None:
                        ang_diff = abs(ang - path_ref_dir)
                        while ang_diff > math.pi:
                            ang_diff = 2 * math.pi - ang_diff
                        align_score = 1.0 - ang_diff / math.pi  # 0..1
                        combined = 0.4 * cost_score + 0.6 * align_score
                    else:
                        combined = cost_score
                    if combined > best_score:
                        best_score = combined
                        best_dir = ang

                if best_dir is not None:
                    # Move toward best-scoring direction (path-aligned + low cost)
                    # v2.9e: Use true position as base for setObjectPosition
                    # (same fix as FOLLOW state — prevents teleportation when
                    # AMCL drifts). Costmap already filtered at AMCL position
                    # during direction scoring above.
                    tx = true_x + step * math.cos(best_dir)
                    ty = true_y + step * math.sin(best_dir)
                    # v2.9d: Real obstacle collision check on actual target
                    if is_position_safe(tx, ty):
                        sim.setObjectPosition(base, -1, [tx, ty, curr[2]])
                        sim.setObjectOrientation(base, -1, [0, 0, best_dir])
                        runtime.last_recover_dir = best_dir
                    else:
                        # Target blocked by real obstacle — skip this move
                        runtime.no_progress_frames += 1
                elif in_lethal or oscillating:
                    # All 0.6m directions blocked — try progressively larger steps
                    # to escape deep inflation zones (e.g. stuck inside obstacle)
                    # v2.9d: Reduced max step from 1.5m to 0.6m and added real
                    # obstacle collision check. Previously 1.5m steps could clip
                    # through walls because setObjectPosition has no physics.
                    # v2.9e: Use true position as base for movement.
                    escaped = False
                    for big_step in [0.4, 0.5, 0.6]:
                        for i in range(16):
                            ang = i * math.pi / 8
                            # Costmap check at AMCL position (planner perception)
                            cost_tx = rx + big_step * math.cos(ang)
                            cost_ty = ry + big_step * math.sin(ang)
                            # Movement target at true position
                            move_tx = true_x + big_step * math.cos(ang)
                            move_ty = true_y + big_step * math.sin(ang)
                            if (costmap.get_cost(cost_tx, cost_ty) < COST_INSCRIBED and
                                    is_position_safe(move_tx, move_ty)):
                                sim.setObjectPosition(base, -1, [move_tx, move_ty, curr[2]])
                                sim.setObjectOrientation(base, -1, [0, 0, ang])
                                runtime.last_recover_dir = ang
                                print(f"[RECOVER] Escaped with step={big_step}m "
                                      f"dir={ang:.2f}rad")
                                escaped = True
                                break
                        if escaped:
                            break
                    if not escaped:
                        # All directions blocked even at 0.6m — rotate in place
                        curr_ori[2] += 0.5
                        sim.setObjectOrientation(base, -1, curr_ori)
                else:
                    # All directions blocked — rotate in place
                    curr_ori[2] += 0.5
                    sim.setObjectOrientation(base, -1, curr_ori)
                runtime.recover_frames += 1

                # AMCL recovery: re-spread particles at frame 5 regardless of
                # lethal zone status.
                # v2.9: Previously, particle re-spreading was skipped when
                # in_lethal=True. But Run 5 showed the robot stuck in lethal
                # zone for 8+ recoveries with loc_err=0.6m never improving —
                # because the robot's AMCL position estimate was wrong, it
                # thought it was in a lethal zone when it might not be.
                # Re-spreading particles helps AMCL converge to the true pose,
                # which may reveal the robot is actually in open space.
                if USE_AMCL and runtime.recover_frames == 5:
                    # v2.8: Only re-spread particles when AMCL actually needs
                    # it (loc_err > 0.5). Re-spreading when loc_err is already
                    # low destroys a good estimate and makes things worse.
                    if last_loc_err > 0.5:
                        # v2.8: Adaptive spread — escalate when consecutive
                        # recoveries fail to bring loc_err below 0.5m.
                        # 0 failures: 1.0m (local recovery)
                        # 1-2 failures: 3.0m (wider search)
                        # 3+ failures: 5.0m (global relocalization on 10x8m map)
                        if consecutive_recover_failures >= 3:
                            _recover_spread = 5.0
                            _recover_label = "GLOBAL"
                        elif consecutive_recover_failures >= 1:
                            _recover_spread = 3.0
                            _recover_label = "WIDE"
                        else:
                            _recover_spread = 1.0
                            _recover_label = "LOCAL"
                        # v2.9f: Ground-truth fallback — when AMCL is completely
                        # lost (loc_err > 5m AND 4+ consecutive failures), the
                        # particle cloud is spread around the WRONG position.
                        # Spreading 5m around a position that's 8m off covers a
                        # 10m radius — but on a 10x8m map, that's the entire map,
                        # and the likelihood field in unmapped areas is uniform,
                        # so particles can't converge. Fix: use true position as
                        # recovery center (simulates GPS/AprilTag landmark fix).
                        # This is equivalent to a real robot using additional
                        # sensors for global localization when AMCL fails.
                        if last_loc_err > 5.0 and consecutive_recover_failures >= 4:
                            print(f"[AMCL-RECOVER] CRITICAL: loc_err={last_loc_err:.1f}m "
                                  f"with {consecutive_recover_failures} failures — "
                                  f"using ground-truth position fix")
                            amcl.recover(true_x, true_y, true_yaw, spread=0.5)
                            odom.reset(true_x, true_y, true_yaw)  # sync odom too
                            consecutive_recover_failures = 0  # reset counter
                        else:
                            print(f"[AMCL-RECOVER] Stuck for 5 frames — re-localizing "
                                  f"({_recover_label}, spread={_recover_spread}). "
                                  f"loc_err={last_loc_err:.3f} conf={last_loc_conf:.3f} "
                                  f"consecutive_failures={consecutive_recover_failures}"
                                  f"{' [in_lethal]' if in_lethal else ''}")
                            amcl.recover(rx, ry, ryaw, spread=_recover_spread)
                        _amcl_respread_done = True
                    else:
                        print(f"[AMCL-RECOVER] loc_err={last_loc_err:.3f} already low — "
                              f"skipping particle re-spread AND spin")
                        _amcl_respread_done = False
                elif USE_AMCL and runtime.recover_frames in (6, 7, 8):
                    # v2.9f: Only spin if particles were re-spread at frame 5.
                    # Spinning without re-spread causes AMCL divergence: the
                    # fast 90°/frame rotation introduces odometry noise that
                    # disperses the (already good) particle cloud. Run 10
                    # showed loc_err jumping from 0.089→2.278 after a pointless
                    # spin when loc_err was already low.
                    if _amcl_respread_done:
                        # Spin in place 90° per frame to gather 360° LiDAR data
                        # for AMCL convergence (3 frames * 90° = full revolution)
                        curr_ori[2] += math.pi / 2
                        sim.setObjectOrientation(base, -1, curr_ori)
                        print(f"[AMCL-RECOVER] Spin frame {runtime.recover_frames}/8 "
                              f"yaw_delta=+90° new_yaw={curr_ori[2]:.2f}"
                              f"{' [in_lethal]' if in_lethal else ''}")
                elif USE_AMCL and in_lethal and runtime.recover_frames % 3 == 0:
                    print(f"[AMCL-RECOVER] In lethal zone (cost={robot_cost}), "
                          f"escaping first (frame {runtime.recover_frames})")

                if runtime.recover_frames > 10:
                    # Reset DWA velocity and try planning again
                    local_planner.reset()
                    runtime.finish_recovery()
                    # After recovery, force fresh AMCL estimate
                    if USE_AMCL:
                        rx, ry, ryaw, last_loc_conf = amcl.get_estimate()
                        last_loc_err = math.sqrt((rx - true_x) ** 2
                                                 + (ry - true_y) ** 2)
                        # v2.8: Track consecutive recovery failures to
                        # escalate spread on next attempt.
                        if last_loc_err < 0.5:
                            consecutive_recover_failures = 0
                        else:
                            consecutive_recover_failures += 1
                        # v6.2: RecoveryManager失败记忆
                        if last_loc_err > 0.5:
                            recovery_manager.memory.record_failure(
                                true_x, true_y, frame, 'loc_err_high')
                            # v5.10: 检查是否应该封锁区域
                            if recovery_manager.memory.should_block_area(
                                    true_x, true_y, frame):
                                frontier_manager.mark_unreachable_with_reason(
                                    true_x, true_y, reason='amcl_lost', radius_m=3.0)
                                print(f"[RECOVER-MEM] 封锁区域 ({true_x:.1f},{true_y:.1f}) "
                                      f"3m半径 — 连续失败")
                        print(f"[AMCL-RECOVER] Recovery done. "
                              f"new loc_err={last_loc_err:.3f} "
                              f"conf={last_loc_conf:.3f} "
                              f"failures={consecutive_recover_failures} "
                              f"mem={len(recovery_manager.memory.failure_history)}")
                        # v2.9h: Only clear unreachable goals when AMCL actually
                        # corrected a position error (loc_err was high). no_progress
                        # RECOVERs don't indicate wrong position — clearing would
                        # re-expose genuinely unreachable frontiers, causing the
                        # death loop seen in v2.9g (12 RECOVERs at y≈3.5).
                        if last_loc_err > 0.5 and len(frontier_manager.unreachable_goals) > 50:
                            print(f"[FRONTIER] Clearing {len(frontier_manager.unreachable_goals)} "
                                  f"unreachable goals (AMCL corrected loc_err={last_loc_err:.3f})")
                            frontier_manager.clear_unreachable()

            elif runtime.state == DONE:
                # Exploration complete: hold position, keep sensing to refine
                # the map but don't move. Legs idle.
                done_frames += 1
                if frame % 100 == 0:
                    print(f"[DONE] Exploration complete. "
                          f"Coverage={occ_grid.coverage_percent():.1f}% "
                          f"Visited={occ_grid.visited_count()} "
                          f"Unreachable={len(frontier_manager.unreachable_goals)} "
                          f"idle={done_frames}")
                # v2.6: Check for new frontiers every 100 frames (was 200).
                # Use info-gain frontiers for better re-exploration targets.
                # Also clear visited_frontiers to allow re-visiting if the
                # robot has been in DONE for a while (map may have changed).
                if frame % 100 == 0:
                    frontiers = occ_grid.find_frontiers_with_info_gain(
                        rx, ry, sensor_range=4.0, max_frontiers=10)
                    reachable = frontier_manager.filter_frontiers_with_metadata(
                        frontiers,
                        current_frame=frame,
                        robot_y=true_y,
                        doorway_crossing_frames=doorway_crossing_frames,
                    )
                    if reachable:
                        print(f"[DONE] New reachable frontier detected — "
                              f"resuming exploration")
                        done_frames = 0
                        runtime.return_to_planning(clear_goal=False, clear_path=False)
                        runtime.reset_frontier_cycles()
                    elif done_frames > 300:
                        # v2.8: Idle too long — clear all frontier memory and
                        # unreachable markings to force re-exploration. This
                        # prevents the robot from being permanently stuck in
                        # DONE when AMCL drift caused false unreachable marks.
                        print(f"[DONE] Idle {done_frames} frames — clearing frontier "
                              f"memory & unreachable marks, resuming patrol")
                        frontier_manager.visited_frontiers.clear()
                        frontier_manager.visited_count.clear()
                        frontier_manager.clear_unreachable()
                        done_frames = 0
                        consecutive_recover_failures = 0
                        runtime.return_to_planning(clear_goal=False, clear_path=False)
                        runtime.reset_frontier_cycles()

            # --- 5.5 Localization update (Odometry + AMCL) ---
            # Simulates real robot localization: odometry drifts, AMCL corrects
            # it using LiDAR observations on the occupancy grid.
            # v2.9f: AMCL now runs every frame (was every 2 frames). The old
            # code had a bug: prev_true_pose was updated every frame, but
            # odom.update() only ran on even frames — so the displacement from
            # odd frames was lost, causing AMCL to see half the actual motion.
            if USE_AMCL and prev_true_pose is not None:
                # Compute ground-truth displacement (for evaluation / reference)
                dx_true = true_x - prev_true_pose[0]
                dy_true = true_y - prev_true_pose[1]
                dyaw_true = true_yaw - prev_true_pose[2]
                while dyaw_true > math.pi:
                    dyaw_true -= 2 * math.pi
                while dyaw_true < -math.pi:
                    dyaw_true += 2 * math.pi
                # Derive commanded velocity from ground-truth displacement, then
                # let Odometry.update() apply its own noise model (sigma_v /
                # sigma_w) and integrate the odom pose. This unifies the noise
                # model and removes the duplicate manual noise (Bug S6).
                dt = 0.1
                v = math.sqrt(dx_true ** 2 + dy_true ** 2) / dt
                w = dyaw_true / dt
                odom_dx, odom_dy, odom_dyaw = odom.update(v, w, dt)
                # v2.9g: CRITICAL FIX — Convert world-frame LiDAR angles to
                # robot-frame before passing to AMCL.
                # laser_scan_full() returns world-frame angles (0..2pi) because
                # occ_grid and costmap use them directly (hit = rx + dist*cos(a)).
                # But AMCL's weight() treats the input as robot-frame and adds
                # pyaw (beam_angle = pyaw + obs_angle). Without conversion, when
                # true_yaw != 0, predicted hit points are rotated by pyaw,
                # causing AMCL to converge to wrong positions. This was the
                # ROOT CAUSE of v2.9f's loc_err explosion (0.365 -> 1.185m at
                # frame 109 when ryaw flipped from 0.01 to 3.11).
                amcl_angles = [(a - true_yaw) for a in angles]
                # AMCL: predict (motion) + weight (LiDAR) + resample
                amcl.update(odom_dx, odom_dy, odom_dyaw, amcl_angles, distances, frame=frame)
                # Update pose from AMCL estimate
                rx, ry, ryaw, last_loc_conf = amcl.get_estimate()
                last_loc_err = math.sqrt((rx - true_x) ** 2 + (ry - true_y) ** 2)
                # v2.9f: Odometry correction — when AMCL is very confident
                # (conf > 0.95, loc_err < 0.15m), sync odometry to AMCL
                # estimate. This prevents drift accumulation: without this,
                # odom error grows unbounded, and AMCL's motion model
                # prediction (which uses odom delta) becomes increasingly
                # wrong, causing particles to diverge. In a real robot this
                # is done by the EKF/UKF feedback loop; here we simulate it
                # by resetting odom to the AMCL-corrected pose.
                if last_loc_conf > 0.95 and last_loc_err < 0.15:
                    odom.reset(rx, ry, ryaw)
            prev_true_pose = (true_x, true_y, true_yaw)

            # --- 5.6 Semantic mapping (vision) ---
            if semantic_mapper and rgb_ok and frame % 5 == 0:
                semantic_mapper.update(FRAME_PATH, true_x, true_y, true_yaw)

            # Per-frame: compute sectors for CSV + adaptive checks
            # Reuse full_scan results to avoid 8xN duplicate ray-AABB tests
            sectors = sectors_from_full_scan(angles, distances, ryaw)

            # P1-3: Telemetry frame end
            telemetry.frame_end(
                ['PLAN', 'FOLLOW', 'RECOVER', 'DONE'][runtime.state],
                occ_grid.coverage_percent(),
                len(frontier_manager.unreachable_goals))

            # --- 6. Periodic tasks ---
            # (frame and t are incremented at the top of the loop now)

            # Save map periodically
            if frame % SAVE_INTERVAL == 0:
                occ_grid.save(MAP_FILE)
                # P1-4: Session persistence (map + frontier memory)
                session_store.save_map(occ_grid)
                session_store.save_frontier_memory(frontier_manager)

            # Update HTML map every 10 frames
            if frame % 10 == 0:
                update_map_html(rx, ry, ryaw, runtime.current_path, runtime.current_goal)

            # Write CSV data row every frame (rich dataset for analysis)
            cur_cost = costmap.get_cost(rx, ry)
            goal_x = runtime.current_goal[0] if runtime.current_goal else 0.0
            goal_y = runtime.current_goal[1] if runtime.current_goal else 0.0
            dist_to_goal = math.sqrt((rx - goal_x) ** 2 + (ry - goal_y) ** 2)
            path_len = len(runtime.current_path) if runtime.current_path else 0
            data_writer.writerow([
                frame, f"{t:.1f}", f"{rx:.3f}", f"{ry:.3f}", f"{ryaw:.3f}",
                runtime.state,
                f"{occ_grid.coverage_percent():.2f}",
                occ_grid.visited_count(),
                int(cur_cost),
                f"{sectors[0]:.2f}", f"{sectors[2]:.2f}",
                f"{sectors[6]:.2f}", f"{sectors[4]:.2f}",
                f"{last_dwa_v:.3f}", f"{last_dwa_w:.3f}", last_no_progress,
                f"{goal_x:.3f}", f"{goal_y:.3f}",
                f"{dist_to_goal:.2f}", path_len,
                f"{true_x:.3f}", f"{true_y:.3f}", f"{last_loc_err:.3f}",
            ])

            # v2.9j: Flush CSV every 500 frames (5s) so data survives crashes
            if frame % 500 == 0:
                data_file.flush()

            # Evaluation framework: record this frame
            if USE_EVAL:
                evaluator.doorway_crossings = doorway_crossings
                evaluator.update(
                    frame=frame, t=t, x=rx, y=ry, yaw=ryaw,
                    state=runtime.state, coverage=occ_grid.coverage_percent(),
                    cost=cur_cost,
                    loc_error=last_loc_err if USE_AMCL else -1.0,
                    loc_conf=last_loc_conf,
                    dwa_v=last_dwa_v, dwa_w=last_dwa_w,
                    no_progress=last_no_progress,
                    true_x=true_x if USE_AMCL else None,
                    true_y=true_y if USE_AMCL else None,
                )

            # Print status every 50 frames (enhanced with cost + dwa + loc)
            if frame % 50 == 0:
                state_names = ['PLAN', 'FOLLOW', 'RECOVER', 'DONE']
                loc_str = f" loc_err={last_loc_err:.3f} conf={last_loc_conf:.2f}" if USE_AMCL else ""
                sem_str = ""
                if semantic_mapper:
                    counts = semantic_mapper.coverage_by_label()
                    labeled = sum(v for k, v in counts.items() if k != 'unknown')
                    sem_str = f" sem={labeled}"
                print(f"[NAV] pos=({rx:.2f},{ry:.2f}) yaw={ryaw:.2f} "
                      f"state={state_names[runtime.state]} "
                      f"cov={occ_grid.coverage_percent():.1f}% "
                      f"vis={occ_grid.visited_count()} "
                      f"cost={cur_cost} v={last_dwa_v:.2f} w={last_dwa_w:.2f} "
                      f"np={last_no_progress} "
                      f"fwd={sectors[0]:.2f} rgb={'OK' if rgb_ok else 'X'}"
                      f"{loc_str}{sem_str}")

            # Check frame limit (for controlled experiments)
            if MAX_FRAMES > 0 and frame >= MAX_FRAMES:
                print(f"[NAV] MAX_FRAMES={MAX_FRAMES} reached, exiting gracefully")
                break

            time.sleep(0.1)
        except KeyboardInterrupt:
            print("[NAV] Interrupted by user")
            break
        except Exception as e:
            print(f"[WARN] Frame {frame} error: {e}")
            if isinstance(e, (ConnectionError, TimeoutError)):
                reconnect_zmq()
            time.sleep(0.2)
            continue

except KeyboardInterrupt:
    print("\nStopped by user")
except Exception as e:
    print(f"\nError: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("Stopping simulation...")
    try:
        data_file.close()
        print(f"[DATA] CSV closed: {DATA_CSV} ({frame} frames)")
    except:
        pass
    # Save evaluation results
    if USE_EVAL:
        try:
            eval_csv = os.path.join(EVAL_DIR, f"{METHOD_NAME}_data.csv")
            eval_json = os.path.join(EVAL_DIR, f"{METHOD_NAME}_summary.json")
            evaluator.save_csv(eval_csv)
            evaluator.save_summary(eval_json)
            print(f"[EVAL] Saved: {eval_csv}")
            print(f"[EVAL] Summary: {eval_json}")
            s = evaluator.summary()
            print(f"[EVAL] {s['method']}: cov={s['final_coverage_pct']}% "
                  f"dist={s['cumulative_distance_m']}m "
                  f"recover={s['recover_events']} "
                  f"doorway={s['doorway_crossings']} "
                  f"loc_err={s['mean_loc_error_m']}m")
            # === Map quality metrics (IoU, Hausdorff, entropy) ===
            try:
                map_metrics = compute_map_metrics(occ_grid, obstacles)
                map_json = os.path.join(EVAL_DIR, f"{METHOD_NAME}_map_quality.json")
                with open(map_json, 'w') as mf:
                    json.dump(map_metrics, mf, indent=2)
                print(f"[EVAL] Map quality: IoU_occ={map_metrics['iou_occupied']} "
                      f"IoU_free={map_metrics['iou_free']} "
                      f"Hausdorff={map_metrics['hausdorff_m']}m "
                      f"Entropy={map_metrics['map_entropy']} "
                      f"P={map_metrics['precision']} R={map_metrics['recall']}")
                print(f"[EVAL] Map quality saved: {map_json}")
            except Exception as e:
                print(f"[EVAL] Map quality error: {e}")
        except Exception as e:
            print(f"[EVAL] Save error: {e}")
    # Save semantic stats
    if semantic_mapper:
        try:
            counts = semantic_mapper.coverage_by_label()
            print(f"[VISION] Semantic labels: {counts}")
        except:
            pass
    try:
        occ_grid.save(MAP_FILE)
    except Exception as e:
        print(f"[WARN] Failed to save map: {e}")
    # P1-3: Print telemetry summary
    try:
        tel_summary = telemetry.get_summary()
        print(f"[TELEMETRY] Summary: frames={tel_summary['total_frames']} "
              f"replan={tel_summary['replan_count']} recover={tel_summary['recover_count']} "
              f"reached={tel_summary['goal_reached_count']} "
              f"astar_avg={tel_summary['astar_avg_ms']:.1f}ms "
              f"dwa_avg={tel_summary['dwa_avg_ms']:.1f}ms")
        if tel_summary.get('recover_reasons'):
            print(f"[TELEMETRY] Recover reasons: {tel_summary['recover_reasons']}")
    except Exception as e:
        print(f"[WARN] Telemetry summary error: {e}")
    # P1-4: Session record run + final save
    try:
        session_store.save_map(occ_grid)
        session_store.save_frontier_memory(frontier_manager)
        session_store.record_run(frame, occ_grid.coverage_percent())
        print(f"[SESSION] Saved (run_count updated, coverage={occ_grid.coverage_percent():.1f}%)")
    except Exception as e:
        print(f"[WARN] Session save error: {e}")
    try:
        sim.stopSimulation()
        time.sleep(1)
    except:
        pass
    try:
        sim.setObjectPosition(base, -1, [1.0, -2.0, 0.0])
        sim.setObjectOrientation(base, -1, [0, 0, 0])
        for name in legs:
            sim.setObjectOrientation(legs[name]["upper"], base_link, [0, 0, 0])
            sim.setObjectOrientation(legs[name]["lower"], base_link, [0, 0, 0])
    except:
        pass
    try:
        sim.removeObject(vs_handle)
    except:
        pass
    print(f"Final: coverage={occ_grid.coverage_percent():.1f}% "
          f"visited={occ_grid.visited_count()}")
    print(f"Map saved: {MAP_FILE}")
