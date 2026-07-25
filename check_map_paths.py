#!/usr/bin/env python3
"""Check map for obstacles along patrol paths"""
import sys

# Read PGM map
with open('/home/veni/puppy_ws/src/puppy_nav/maps/home_map.pgm', 'rb') as f:
    # Read header
    magic = f.readline().strip()  # P5
    dims = f.readline().strip().split()
    width, height = int(dims[0]), int(dims[1])
    maxval = f.readline().strip()  # 255
    data = f.read()

print(f"Map: {width}x{height}, data size: {len(data)}")

origin_x, origin_y = -5.0, -4.0
resolution = 0.05

def world_to_pixel(x, y):
    px = int((x - origin_x) / resolution)
    # PGM row 0 = top = north (y_max), so flip y
    py = height - 1 - int((y - origin_y) / resolution)
    return px, py

def get_cost(x, y):
    px, py = world_to_pixel(x, y)
    if 0 <= px < width and 0 <= py < height:
        return data[py * width + px]
    return -1

# Check paths between waypoints
paths = [
    ("WP1->WP2", 2.0, -2.0, 0.0, -1.0),
    ("WP2->WP3", 0.0, -1.0, 0.0, 1.5),
    ("WP3->WP4", 0.0, 1.5, -2.5, 1.5),
    ("WP4->WP5", -2.5, 1.5, 0.0, 1.5),
    ("WP5->WP6", 0.0, 1.5, 3.0, 2.0),
    ("WP6->WP7", 3.0, 2.0, 0.0, -1.0),
    ("WP7->WP8", 0.0, -1.0, -0.5, -2.0),
]

for name, x1, y1, x2, y2 in paths:
    print(f"\n=== {name}: ({x1},{y1}) -> ({x2},{y2}) ===")
    # Sample along the path
    steps = 20
    blocked = []
    for i in range(steps + 1):
        t = i / steps
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        c = get_cost(x, y)
        if c > 50:  # Not free
            blocked.append((x, y, c))
    if blocked:
        print(f"  BLOCKED at {len(blocked)} points:")
        for x, y, c in blocked[:5]:
            print(f"    ({x:.2f}, {y:.2f}): cost={c}")
    else:
        print(f"  Path is CLEAR")

# Also check specific points
print("\n=== Waypoint costs ===")
waypoints = [
    ("WP1 客厅", 2.0, -2.0),
    ("WP2 门口南", 0.0, -1.0),
    ("WP3 走廊", 0.0, 1.5),
    ("WP4 卧室", -2.5, 1.5),
    ("WP5 走廊返", 0.0, 1.5),
    ("WP6 厨房", 3.0, 2.0),
    ("WP7 门口返", 0.0, -1.0),
    ("WP8 充电桩", -0.5, -2.0),
]
for name, x, y in waypoints:
    c = get_cost(x, y)
    status = "FREE" if c == 255 else "OCCUPIED" if c == 0 else f"UNKNOWN({c})"
    print(f"  {name} ({x},{y}): cost={c} [{status}]")
