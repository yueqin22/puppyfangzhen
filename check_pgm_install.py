#!/usr/bin/env python3
"""直接检查install目录中的PGM文件"""
import numpy as np

pgm_path = '/home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/maps/home_map.pgm'
with open(pgm_path, 'rb') as f:
    magic = f.readline().strip()
    dims = f.readline().strip().split()
    maxval = f.readline().strip()
    w, h = int(dims[0]), int(dims[1])
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

print(f'PGM: {magic}, {w}x{h}, maxval={maxval}')
print(f'文件总字节: {w*h + len(magic) + len(dims[0]) + len(dims[1]) + len(maxval) + 4}')

origin_x, origin_y = -5.0, -4.0
res = 0.05

# 检查map_server发布的位置
positions = [
    (159, 119, '航点6(map_server像素)'),
    (160, 120, '航点6(PGM像素)'),
    (139, 39, '航点1(map_server像素)'),
    (140, 40, '航点1(PGM像素)'),
]
for px, py, name in positions:
    val = data[py, px]
    wx = origin_x + px * res
    wy = origin_y + py * res
    print(f'{name}: 像素({px},{py}) -> 世界({wx:.2f},{wy:.2f}) 值={val}')

# 打印(159,119)周围20x20
print(f'\n--- 像素(159,119)周围20x20 ---')
for y in range(119+10, 119-11, -1):
    line = ''
    for x in range(159-10, 159+11):
        v = data[y, x]
        if x == 159 and y == 119:
            line += 'R'
        elif v == 0:
            line += '#'
        elif v == 255:
            line += '.'
        else:
            line += '?'
    print(f'  y={y:3d}: {line}')

# 统计整个地图的障碍物分布
obstacle_count = np.sum(data == 0)
free_count = np.sum(data == 255)
other_count = w * h - obstacle_count - free_count
print(f'\n地图统计: 障碍={obstacle_count}, 自由={free_count}, 其他={other_count}')

# 找到所有障碍物的边界
obstacle_pixels = np.where(data == 0)
if len(obstacle_pixels[0]) > 0:
    y_min, y_max = obstacle_pixels[0].min(), obstacle_pixels[0].max()
    x_min, x_max = obstacle_pixels[1].min(), obstacle_pixels[1].max()
    print(f'障碍物范围: x=[{x_min},{x_max}], y=[{y_min},{y_max}]')
    print(f'世界坐标: x=[{origin_x+x_min*res:.2f},{origin_x+x_max*res:.2f}], '
          f'y=[{origin_y+y_min*res:.2f},{origin_y+y_max*res:.2f}]')
