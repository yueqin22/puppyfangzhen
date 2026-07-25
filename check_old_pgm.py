#!/usr/bin/env python3
"""检查旧的PGM文件"""
import numpy as np

# 检查旧的PGM文件
old_path = '/home/veni/puppy_ws/maps/home_map.pgm'
with open(old_path, 'rb') as f:
    magic = f.readline().strip()
    dims = f.readline().strip().split()
    maxval = f.readline().strip()
    w, h = int(dims[0]), int(dims[1])
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

print(f'旧PGM: {magic}, {w}x{h}, maxval={maxval}')
print(f'文件大小: {w*h + 50} bytes (approx)')

# 统计
obs = np.sum(data == 0)
free = np.sum(data == 255)
print(f'障碍={obs}, 自由={free}, 总={w*h}')

# 检查几个位置（假设原点(-5,-4), 分辨率0.05）
ox, oy, res = -5.0, -4.0, 0.05
positions = [
    (3.0, 2.0, '航点6'),
    (2.0, -2.0, '航点1'),
    (1.0, -2.0, '机器人初始'),
]
for wx, wy, name in positions:
    px = int((wx - ox) / res)
    py = int((wy - oy) / res)
    if 0 <= px < w and 0 <= py < h:
        val = data[py, px]
        print(f'{name} ({wx:.1f},{wy:.1f}) -> 像素({px},{py}) 值={val}')
    else:
        print(f'{name} ({wx:.1f},{wy:.1f}) -> 像素({px},{py}) 超出范围({w}x{h})')

# 打印(3.0,2.0)周围
px, py = int((3.0 - ox) / res), int((2.0 - oy) / res)
if 0 <= px < w and 0 <= py < h:
    print(f'\n--- 旧地图 (3.0,2.0) 周围20x20 ---')
    for y in range(min(h-1,py+10), max(0,py-11), -1):
        line = ''
        for x in range(max(0,px-10), min(w,px+11)):
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

# 也检查一下YAML文件
import os
yaml_path = '/home/veni/puppy_ws/maps/home_map.yaml'
if os.path.exists(yaml_path):
    print(f'\n--- 旧YAML文件 ---')
    with open(yaml_path, 'r') as f:
        print(f.read())
