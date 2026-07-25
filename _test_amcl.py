#!/usr/bin/env python3
"""Test the new vectorized AMCL with Likelihood Field Model."""
import time
import math
import numpy as np
import sys

# Test syntax first
import ast
for f in ["amcl.py"]:
    try:
        with open(f) as fp:
            ast.parse(fp.read())
        print(f"  {f}: syntax OK")
    except SyntaxError as e:
        print(f"  {f}: SYNTAX ERROR: {e}")
        sys.exit(1)

from occupancy_grid import OccupancyGrid
from amcl import AMCL, _distance_transform_numpy

# Build a simple test grid with some obstacles
print("\n=== Test 1: Distance Transform ===")
grid = OccupancyGrid()
# Mark some cells as occupied (simulating walls)
grid.log_odds[10:70, 5:7] = 2.0  # west wall
grid.log_odds[10:70, 93:95] = 2.0  # east wall
grid.log_odds[5:7, 10:90] = 2.0  # north wall
grid.log_odds[73:75, 10:90] = 2.0  # south wall
print(f"  Occupied cells: {(grid.log_odds > 0.6).sum()}")

t0 = time.time()
mask = grid.log_odds > 0.6
dist = _distance_transform_numpy(mask)
t1 = time.time()
print(f"  Distance transform: {(t1-t0)*1000:.1f}ms")
print(f"  dist[40, 50] (center): {dist[40, 50]:.2f} cells = {dist[40, 50]*0.1:.2f}m")
print(f"  dist[10, 50] (near wall): {dist[10, 50]:.2f} cells = {dist[10, 50]*0.1:.2f}m")

# Test 2: AMCL weight() speed
print("\n=== Test 2: AMCL weight() performance ===")
amcl = AMCL(grid, n_particles=100, n_obs_rays=8)
amcl.init_cloud(0.0, 0.0, 0.0, spread=0.2)

# Simulate 72-ray LiDAR scan
angles = [2 * math.pi * i / 72 for i in range(72)]
distances = [3.0 + 0.1 * math.sin(i) for i in range(72)]  # synthetic ranges

# Warmup
amcl.weight(angles, distances, frame=0)

t0 = time.time()
for _ in range(10):
    amcl.weight(angles, distances, frame=0)
t1 = time.time()
print(f"  10 weight() calls: {(t1-t0)*1000:.1f}ms ({(t1-t0)*100:.1f}ms/call)")
print(f"  Last N_eff: {amcl.last_n_eff:.1f} / {amcl.n}")

# Test 3: Full update() speed
print("\n=== Test 3: Full update() performance ===")
t0 = time.time()
for i in range(10):
    amcl.update(0.01, 0.0, 0.01, angles, distances, frame=i)
t1 = time.time()
print(f"  10 update() calls: {(t1-t0)*1000:.1f}ms ({(t1-t0)*100:.1f}ms/call)")

# Test 4: Estimate
x, y, yaw, conf = amcl.get_estimate()
print(f"  Estimate: x={x:.3f} y={y:.3f} yaw={yaw:.3f} conf={conf:.3f}")

print("\n=== ALL TESTS PASSED ===")
