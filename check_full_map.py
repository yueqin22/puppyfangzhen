#!/usr/bin/env python3
"""Check entire map for issues"""
import numpy as np

with open('/home/veni/puppy_ws/src/puppy_nav/maps/home_map.pgm', 'rb') as f:
    line = f.readline()
    assert line == b'P5\n'
    line = f.readline()
    w, h = map(int, line.split())
    line = f.readline()
    assert line == b'255\n'
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

print(f"Map size: {w}x{h}")
print(f"Free cells: {np.sum(data == 255)}")
print(f"Occupied cells: {np.sum(data == 0)}")
print(f"Unknown cells: {np.sum((data > 0) & (data < 255))}")

# Print entire map (downsampled by 2)
print("\nFull map (# = occupied, . = free):")
for y in range(0, h, 2):
    row = ""
    for x in range(0, w, 2):
        v = data[y][x]
        if v == 0:
            row += "#"
        elif v == 255:
            row += "."
        else:
            row += "?"
    print(f"y={y:3d}: {row}")

# Check specific area around robot (1.9, -2.4)
ox, oy = -5.0, -4.0
res = 0.05
rx, ry = 1.9, -2.4
px = int((rx - ox) / res)
py = int((ry - oy) / res)
print(f"\nRobot at ({rx},{ry}) -> pixel ({px},{py})")
print(f"Pixel value: {data[py][px]} ({'OCCUPIED' if data[py][px]==0 else 'FREE' if data[py][px]==255 else 'UNKNOWN'})")

# Check 40x40 area around robot
print(f"\n40x40 area around robot:")
for y in range(py-20, py+20):
    row = ""
    for x in range(px-20, px+20):
        if 0 <= x < w and 0 <= y < h:
            v = data[y][x]
            if v == 0:
                row += "#"
            elif v == 255:
                row += "."
            else:
                row += "?"
        else:
            row += " "
    print(f"  {row}")
