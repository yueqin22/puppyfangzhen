#!/usr/bin/env python3
"""检查PGM地图在问题位置的值"""
import numpy as np

# 读取PGM地图
with open('/home/veni/puppy_ws/src/puppy_nav/maps/home_map.pgm', 'rb') as f:
    magic = f.readline().strip()
    dims = f.readline().strip().split()
    maxval = f.readline().strip()
    w, h = int(dims[0]), int(dims[1])
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

origin_x, origin_y = -5.0, -4.0
res = 0.05

def world_to_pixel(x, y):
    px = int((x - origin_x) / res)
    py = int((y - origin_y) / res)
    return px, py

positions = [
    (2.0, -2.0, '航点1'),
    (3.0, 2.0, '航点6'),
    (1.0, -2.0, '机器人初始'),
    (0.0, -1.0, '航点2'),
    (0.0, 1.5, '航点3'),
]
for x, y, name in positions:
    px, py = world_to_pixel(x, y)
    val = data[py, px]
    area = data[max(0,py-3):py+4, max(0,px-3):px+4]
    print(f'{name} ({x:.1f},{y:.1f}) -> 像素({px},{py}) 值={val} 周围min={area.min()} max={area.max()}')

# 打印(2.0,-2.0)周围20x20
px, py = world_to_pixel(2.0, -2.0)
print(f'\n--- 航点1 (2.0,-2.0) 周围地图 ---')
for y in range(py+10, py-11, -1):
    line = ''
    for x in range(px-10, px+11):
        v = data[y, x]
        if x == px and y == py:
            line += 'R'
        elif v == 0:
            line += '#'
        elif v == 255:
            line += '.'
        else:
            line += '?'
    print(f'  y={y:3d}: {line}')

# 打印(3.0,2.0)周围20x20
px, py = world_to_pixel(3.0, 2.0)
print(f'\n--- 航点6 (3.0,2.0) 周围地图 ---')
for y in range(py+10, py-11, -1):
    line = ''
    for x in range(px-10, px+11):
        v = data[y, x]
        if x == px and y == py:
            line += 'R'
        elif v == 0:
            line += '#'
        elif v == 255:
            line += '.'
        else:
            line += '?'
    print(f'  y={y:3d}: {line}')
