#!/usr/bin/env python3
"""生成与 small_room.world 匹配的地图文件 (5x5m 房间 + 障碍物)"""
import numpy as np
import struct
import os

# 地图参数
resolution = 0.05  # 5cm/pixel
room_size = 6.0    # 6m x 6m (留余量)
width = int(room_size / resolution)   # 120 pixels
height = int(room_size / resolution)  # 120 pixels

# 原点 (左下角在世界坐标中的位置)
origin_x = -3.0  # 地图中心在 (0,0)
origin_y = -3.0

# 创建地图 (255=自由空间, 0=障碍物)
grid = np.full((height, width), 255, dtype=np.uint8)

def world_to_pixel(x, y):
    px = int((x - origin_x) / resolution)
    py = int((y - origin_y) / resolution)
    return px, py

def draw_wall(grid, x1, y1, x2, y2, thickness=0.1):
    """画一条墙"""
    px1, py1 = world_to_pixel(x1, y1)
    px2, py2 = world_to_pixel(x2, y2)
    t = max(1, int(thickness / resolution))
    if px1 == px2:  # 垂直墙
        for y in range(min(py1,py2), max(py1,py2)+1):
            for dx in range(-t//2, t//2+1):
                xi = px1 + dx
                if 0 <= xi < width and 0 <= y < height:
                    grid[y][xi] = 0
    elif py1 == py2:  # 水平墙
        for x in range(min(px1,px2), max(px1,px2)+1):
            for dy in range(-t//2, t//2+1):
                yi = py1 + dy
                if 0 <= x < width and 0 <= yi < height:
                    grid[yi][x] = 0

def draw_box(grid, cx, cy, size_x, size_y):
    """画一个矩形障碍物"""
    px, py = world_to_pixel(cx, cy)
    sx = int(size_x / resolution / 2)
    sy = int(size_y / resolution / 2)
    for y in range(py-sy, py+sy+1):
        for x in range(px-sx, px+sx+1):
            if 0 <= x < width and 0 <= y < height:
                grid[y][x] = 0

# 画墙壁 (5x5m 房间, 墙在 ±2.5m)
draw_wall(grid, -2.5, -2.5, 2.5, -2.5)   # 南墙
draw_wall(grid, -2.5, 2.5, 2.5, 2.5)     # 北墙
draw_wall(grid, -2.5, -2.5, -2.5, 2.5)   # 西墙
draw_wall(grid, 2.5, -2.5, 2.5, 2.5)     # 东墙

# 画障碍物 (box_obstacle at 1.0, 1.0, size 0.5x0.5)
draw_box(grid, 1.0, 1.0, 0.5, 0.5)

# 添加一些家庭环境的家具
# 沙发 (2.0, -1.0)
draw_box(grid, 2.0, -1.0, 0.8, 0.4)
# 桌子 (-1.5, 1.5)
draw_box(grid, -1.5, 1.5, 0.6, 0.6)
# 椅子 (-1.0, -1.5)
draw_box(grid, -1.0, -1.5, 0.3, 0.3)

# 翻转 Y 轴 (PGM 格式: 第一行是顶部, 但地图坐标 Y 向上)
grid = np.flipud(grid)

# 写 PGM 文件 (P5 二进制格式)
pgm_path = os.path.join(os.path.dirname(__file__), 'home_map.pgm')
with open(pgm_path, 'wb') as f:
    header = f"P5\n{width} {height}\n255\n".encode()
    f.write(header)
    f.write(grid.tobytes())

print(f"PGM 地图已生成: {pgm_path}")
print(f"  尺寸: {width}x{height} pixels")
print(f"  分辨率: {resolution} m/pixel")
print(f"  覆盖范围: {room_size}m x {room_size}m")

# 写 YAML 文件
yaml_path = os.path.join(os.path.dirname(__file__), 'home_map.yaml')
yaml_content = f"""image: home_map.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open(yaml_path, 'w') as f:
    f.write(yaml_content)

print(f"YAML 配置已生成: {yaml_path}")
print(f"  原点: ({origin_x}, {origin_y})")
