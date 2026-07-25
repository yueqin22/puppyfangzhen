#!/usr/bin/env python3
"""检查PGM文件翻转后的位置 - PGM是top-down, ROS map是bottom-up"""
import numpy as np

pgm_path = '/home/veni/puppy_ws/src/puppy_nav/maps/home_map.pgm'
with open(pgm_path, 'rb') as f:
    magic = f.readline().strip()
    dims = f.readline().strip().split()
    maxval = f.readline().strip()
    w, h = int(dims[0]), int(dims[1])
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

print(f'PGM: {w}x{h}')

# PGM row 0 = top of image (y_max in world)
# ROS map row 0 = bottom of map (y_min = origin_y in world)
# So map row py corresponds to PGM row (h - 1 - py)

origin_x, origin_y = -5.0, -4.0
res = 0.05

positions = [
    (3.0, 2.0, '航点6'),
    (2.0, -2.0, '航点1'),
    (1.0, -2.0, '机器人初始'),
]

print('\n=== 对比: 直接读取 vs 翻转后读取 ===')
for wx, wy, name in positions:
    px = int((wx - origin_x) / res)
    py = int((wy - origin_y) / res)
    # 直接读取 (错误的方式)
    direct_val = data[py, px]
    # 翻转后读取 (map_server的方式)
    flipped_row = h - 1 - py
    flipped_val = data[flipped_row, px]
    print(f'{name} ({wx:.1f},{wy:.1f}):')
    print(f'  地图像素({px},{py}) -> 直接PGM行{py}: 值={direct_val}')
    print(f'  地图像素({px},{py}) -> 翻转PGM行{flipped_row}: 值={flipped_val}')

# 打印(3.0,2.0)翻转后的周围区域
px, py = int((3.0 - origin_x) / res), int((2.0 - origin_y) / res)
flipped_row = h - 1 - py
print(f'\n--- 航点6 (3.0,2.0) 翻转后PGM行{flipped_row}周围 ---')
for y in range(flipped_row+10, flipped_row-11, -1):
    if y < 0 or y >= h:
        continue
    line = ''
    for x in range(px-10, px+11):
        if x < 0 or x >= w:
            line += '?'
            continue
        v = data[y, x]
        if x == px and y == flipped_row:
            line += 'R'
        elif v == 0:
            line += '#'
        elif v == 255:
            line += '.'
        else:
            line += '?'
    print(f'  PGM行{y:3d}: {line}')

# 打印(2.0,-2.0)翻转后的周围区域
px, py = int((2.0 - origin_x) / res), int((-2.0 - origin_y) / res)
flipped_row = h - 1 - py
print(f'\n--- 航点1 (2.0,-2.0) 翻转后PGM行{flipped_row}周围 ---')
for y in range(flipped_row+10, flipped_row-11, -1):
    if y < 0 or y >= h:
        continue
    line = ''
    for x in range(px-10, px+11):
        if x < 0 or x >= w:
            line += '?'
            continue
        v = data[y, x]
        if x == px and y == flipped_row:
            line += 'R'
        elif v == 0:
            line += '#'
        elif v == 255:
            line += '.'
        else:
            line += '?'
    print(f'  PGM行{y:3d}: {line}')

# 打印整个地图的翻转视图（缩小版）
print(f'\n=== 整个PGM地图（翻转后=map_server视角）===')
print(f'    x: 0       50      100     150     199')
print(f'       (world x: -5.0   -2.5    0.0     2.5     5.0)')
for y in range(0, h, 4):  # 每4行打印一次
    line = ''
    for x in range(0, w, 2):  # 每2列打印一次
        v = data[y, x]
        if v == 0:
            line += '#'
        elif v == 255:
            line += '.'
        else:
            line += '?'
    world_y = origin_y + (h - 1 - y) * res
    print(f'  y={y:3d} (world y={world_y:5.2f}): {line}')
