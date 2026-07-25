#!/usr/bin/env python3
"""Check if waypoints are in free space on the map"""
import numpy as np

# Read PGM file manually (without PIL)
with open('home_map.pgm', 'rb') as f:
    header = f.readline().decode().strip()  # P5
    line = f.readline().decode().strip()
    while line.startswith('#'):
        line = f.readline().decode().strip()
    width, height = map(int, line.split())
    maxval = int(f.readline().decode().strip())
    data = f.read()

arr = np.array(list(data), dtype=np.uint8).reshape((height, width))
print(f'Map size: {width} x {height}')
print(f'Resolution: 0.05 m/pixel')
print(f'Origin: (-5.0, -4.0)')
print()

# Waypoints to check
waypoints = [
    ('LivingRoom', 2.0, -2.0),
    ('Hallway', 0.5, 1.5),
    ('Bedroom', -2.5, 1.5),
    ('Kitchen', 3.0, 2.0),
    ('Charger', -0.5, -2.0),
    ('RobotStart', 1.0, -2.0),
]

origin_x, origin_y = -5.0, -4.0
resolution = 0.05

for name, x, y in waypoints:
    px = int((x - origin_x) / resolution)
    py = int((y - origin_y) / resolution)
    # Check 7x7 area around waypoint (0.35m radius)
    r = 4
    region = arr[max(0,py-r):py+r+1, max(0,px-r):px+r+1]
    occupied = np.sum(region < 128)
    total = region.size
    status = 'FREE' if occupied == 0 else f'BLOCKED ({occupied}/{total} cells occupied)'
    print(f'{name} ({x:.1f}, {y:.1f}) -> pixel ({px}, {py}): {status}')
    if occupied > 0:
        print(f'  Region values: {region.flatten()[:15]}')

# Also check the path from hallway to bedroom
print()
print('=== Path check: Hallway (0.5, 1.5) to Bedroom (-2.5, 1.5) ===')
for x in np.arange(0.5, -2.6, -0.5):
    px = int((x - origin_x) / resolution)
    py = int((1.5 - origin_y) / resolution)
    val = arr[py, px]
    status = 'FREE' if val >= 128 else 'WALL'
    print(f'  ({x:.1f}, 1.5) -> pixel ({px}, {py}): val={val} {status}')

# Check the doorway at y=0
print()
print('=== Doorway check at y=0, x from -1.5 to 1.5 ===')
for x in np.arange(-1.5, 1.6, 0.25):
    px = int((x - origin_x) / resolution)
    py = int((0.0 - origin_y) / resolution)
    val = arr[py, px]
    status = 'FREE' if val >= 128 else 'WALL'
    print(f'  ({x:.2f}, 0.0) -> pixel ({px}, {py}): val={val} {status}')
