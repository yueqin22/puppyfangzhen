#!/usr/bin/env python3
"""Syntax check + quick TEB test."""
import ast
import sys

files = ["amcl.py", "evaluator.py", "occupancy_grid.py", "teb_planner.py",
         "autonomous_nav.py", "odometry.py", "vision.py", "random_walk.py"]
for f in files:
    try:
        with open(f) as fp:
            ast.parse(fp.read())
        print(f"  OK: {f}")
    except SyntaxError as e:
        print(f"  FAIL: {f} - {e}")
        sys.exit(1)

print("\n=== TEB Quick Test ===")
import time
import math
from occupancy_grid import OccupancyGrid
from costmap import Costmap
from teb_planner import TEBPlanner

grid = OccupancyGrid()
# Add walls
grid.log_odds[10:70, 5:7] = 2.0
grid.log_odds[10:70, 93:95] = 2.0
grid.log_odds[5:7, 10:90] = 2.0
grid.log_odds[73:75, 10:90] = 2.0

costmap = Costmap()
costmap.update_static(grid)

teb = TEBPlanner(costmap)

# Test path following
path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.5), (3.0, 1.0), (4.0, 1.5)]

t0 = time.time()
for _ in range(10):
    v, w = teb.compute_velocity(0.0, 0.0, 0.0, path, 4.0, 1.5)
t1 = time.time()
print(f"  10 TEB calls: {(t1-t0)*1000:.1f}ms ({(t1-t0)*100:.1f}ms/call)")
print(f"  Last command: v={v:.3f} w={w:.3f}")

# Test with goal reached
v, w = teb.compute_velocity(4.0, 1.5, 0.0, path, 4.0, 1.5)
print(f"  Goal reached: v={v} w={w}")

# Test with no path
v, w = teb.compute_velocity(0.0, 0.0, 0.0, [], 4.0, 0.0)
print(f"  No path: v={v:.3f} w={w:.3f}")

print("\n=== ALL TESTS PASSED ===")
