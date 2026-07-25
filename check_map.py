#!/usr/bin/env python3
"""Check map around robot position"""
import numpy as np

with open('/home/veni/puppy_ws/src/puppy_nav/maps/home_map.pgm', 'rb') as f:
    line = f.readline()
    assert line == b'P5\n'
    line = f.readline()
    w, h = map(int, line.split())
    line = f.readline()
    assert line == b'255\n'
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

ox, oy = -5.0, -4.0
res = 0.05

# Check several positions
positions = [
    (1.0, -2.0, "spawn"),
    (1.9, -2.4, "current"),
    (2.0, -2.0, "wp1"),
    (0.0, -1.0, "wp2"),
    (0.0, 1.5, "wp3"),
]

for rx, ry, name in positions:
    px = int((rx - ox) / res)
    py = int((ry - oy) / res)
    print(f"\n{name} ({rx},{ry}) -> pixel ({px},{py}):")
    for y in range(py-5, py+6):
        row = ""
        for x in range(px-5, px+6):
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
