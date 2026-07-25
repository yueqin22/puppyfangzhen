#!/usr/bin/env python3
"""Syntax check for all navigation modules."""
import ast
import sys

files = [
    'amcl.py',
    'autonomous_nav.py',
    'astar_planner.py',
    'teb_planner.py',
    'costmap.py',
    'occupancy_grid.py',
    'evaluator.py',
    'odometry.py',
    'vision.py',
    'random_walk.py',
    'dwa_planner.py',
    'config_loader.py',
]

errors = 0
for f in files:
    try:
        with open(f, 'r', encoding='utf-8') as fp:
            ast.parse(fp.read())
        print(f"OK  {f}")
    except SyntaxError as e:
        print(f"ERR {f}: {e}")
        errors += 1

print(f"\n{len(files) - errors}/{len(files)} modules OK, {errors} errors")
sys.exit(0 if errors == 0 else 1)
