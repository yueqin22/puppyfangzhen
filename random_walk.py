#!/usr/bin/env python3
"""
Random Walk Baseline
====================
Comparison baseline: robot wanders randomly without frontier-based
exploration or A*/DWA planning. Used to quantify how much the proposed
navigation stack improves exploration efficiency.

Strategy:
  - Pick a random heading
  - Move forward until blocked (cost >= INSCRIBED) or timeout
  - Pick a new random heading and repeat
  - Same sensors (LiDAR) and mapping (OccupancyGrid) as the proposed method

Run with:
  METHOD_NAME=random_walk python3 random_walk.py
"""
import os
import sys
import time
import math
import tempfile
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

from occupancy_grid import OccupancyGrid
from costmap import Costmap, COST_LETHAL, COST_INSCRIBED
from evaluator import Evaluator

MAP_FILE = os.environ.get("PUPPY_MAP_FILE",
                          os.path.join(tempfile.gettempdir(), "puppy_map_rw.npz"))
FRAME_PATH = os.environ.get("PUPPY_STREAM_FRAME",
                            os.path.join(tempfile.gettempdir(), "stream_frame.jpg"))
METHOD_NAME = os.environ.get("METHOD_NAME", "random_walk")
EVAL_DIR = os.environ.get("EVAL_DIR", os.path.join(tempfile.gettempdir(), "eval"))
MAX_FRAMES = int(os.environ.get("MAX_FRAMES", "1500"))

client = RemoteAPIClient()
sim = client.getObject("sim")
base = sim.getObject("/base_footprint")
print(f"[RW] Method={METHOD_NAME}  max_frames={MAX_FRAMES}")

# Reset robot and start simulation
sim.setObjectPosition(base, -1, [1.0, -2.0, 0.0])
sim.setObjectOrientation(base, -1, [0, 0, 0])
sim.startSimulation()
import time as _t
_t.sleep(2)  # let sim stabilize
print("[RW] Simulation started")


# === Obstacle discovery (same as autonomous_nav) ===
def discover_obstacles():
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


obstacles = discover_obstacles()
print(f"[RW] {len(obstacles)} obstacles")


def ray_aabb_dist(rx, ry, dx, dy, xmin, ymin, xmax, ymax):
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


def laser_scan(rx, ry, num_rays=72, max_range=8.0):
    angles, distances = [], []
    for i in range(num_rays):
        angle = 2 * math.pi * i / num_rays
        dx, dy = math.cos(angle), math.sin(angle)
        min_d = max_range
        for _, _, (xmin, ymin, xmax, ymax) in obstacles:
            d = ray_aabb_dist(rx, ry, dx, dy, xmin, ymin, xmax, ymax)
            if d < min_d:
                min_d = d
        angles.append(angle)
        distances.append(min_d)
    return angles, distances


# === Navigation modules ===
occ_grid = OccupancyGrid()
costmap = Costmap()
evaluator = Evaluator(METHOD_NAME)
os.makedirs(EVAL_DIR, exist_ok=True)

# Random walk state
current_heading = 0.0
step_size = 0.15
steps_in_heading = 0
max_steps_per_heading = 30  # change direction every ~3s
blocked_count = 0

print(f"[RW] Starting random walk...")
t = 0.0
frame = 0
prev_ry = -2.0

try:
    while frame < MAX_FRAMES:
        # Get position
        curr = sim.getObjectPosition(base, -1)
        curr_ori = sim.getObjectOrientation(base, -1)
        rx, ry, ryaw = curr[0], curr[1], curr_ori[2]

        # LiDAR + mapping
        angles, distances = laser_scan(rx, ry)
        occ_grid.update_from_scan(rx, ry, angles, distances, max_range=8.0)
        occ_grid.mark_visited(rx, ry)
        costmap.update_static(occ_grid)
        costmap.update_obstacles(rx, ry, angles, distances, max_range=8.0)

        # Track doorway crossings
        if (prev_ry < 0 and ry >= 0) or (prev_ry >= 0 and ry < 0):
            evaluator.doorway_crossings += 1
            print(f"[RW] Doorway crossing #{evaluator.doorway_crossings} "
                  f"at ({rx:.2f},{ry:.2f}) cov={occ_grid.coverage_percent():.1f}%")
        prev_ry = ry

        # Random walk: pick new heading periodically or when blocked
        cur_cost = costmap.get_cost(rx, ry)
        if steps_in_heading >= max_steps_per_heading or cur_cost >= COST_INSCRIBED:
            # Pick a new random heading (prefer lower-cost directions)
            best_ang = None
            best_cost = float('inf')
            for _ in range(8):
                ang = np.random.uniform(0, 2 * math.pi)
                tx = rx + step_size * math.cos(ang)
                ty = ry + step_size * math.sin(ang)
                c = costmap.get_cost(tx, ty)
                if c < best_cost and c < COST_INSCRIBED:
                    best_cost = c
                    best_ang = ang
            if best_ang is not None:
                current_heading = best_ang
            else:
                # All blocked — random heading
                current_heading = np.random.uniform(0, 2 * math.pi)
                blocked_count += 1
            steps_in_heading = 0

        # Move in current heading
        new_x = rx + step_size * math.cos(current_heading)
        new_y = ry + step_size * math.sin(current_heading)
        if costmap.get_cost(new_x, new_y) < COST_INSCRIBED:
            sim.setObjectPosition(base, -1, [new_x, new_y, curr[2]])
            sim.setObjectOrientation(base, -1, [0, 0, current_heading])
            steps_in_heading += 1
        else:
            # Blocked — force new heading next frame
            steps_in_heading = max_steps_per_heading

        # Evaluation record (state=1 for "moving", like FOLLOW)
        evaluator.update(
            frame=frame, t=t, x=rx, y=ry, yaw=ryaw,
            state=1,  # treat all as "FOLLOW" for comparison
            coverage=occ_grid.coverage_percent(),
            cost=cur_cost,
            dwa_v=step_size / 0.1, dwa_w=0.0,
        )

        if frame % 50 == 0:
            print(f"[RW] f={frame} pos=({rx:.2f},{ry:.2f}) "
                  f"cov={occ_grid.coverage_percent():.1f}% "
                  f"vis={occ_grid.visited_count()} cost={cur_cost} "
                  f"blocked={blocked_count}")

        t += 0.1
        frame += 1
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[RW] Stopped")
except Exception as e:
    print(f"[RW] Error: {e}")
    import traceback
    traceback.print_exc()
finally:
    # Save evaluation
    eval_csv = os.path.join(EVAL_DIR, f"{METHOD_NAME}_data.csv")
    eval_json = os.path.join(EVAL_DIR, f"{METHOD_NAME}_summary.json")
    evaluator.save_csv(eval_csv)
    evaluator.save_summary(eval_json)
    s = evaluator.summary()
    print(f"\n[RW] DONE: cov={s['final_coverage_pct']}% "
          f"dist={s['cumulative_distance_m']}m "
          f"doorway={s['doorway_crossings']} "
          f"blocked={blocked_count}")
    print(f"[RW] Saved: {eval_csv}")
    occ_grid.save(MAP_FILE)
    try:
        sim.stopSimulation()
    except:
        pass
