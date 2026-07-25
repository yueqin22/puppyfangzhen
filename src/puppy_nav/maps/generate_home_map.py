#!/usr/bin/env python3
"""
为 home.world (10m x 8m) 生成匹配的导航地图

home.world 布局：
  - 外墙：x=±5, y=±4（10m x 8m）
  - 内墙1：y=0, x从-5到-1（客厅/卧室分隔）
  - 内墙2：y=0, x从1到5（客厅/厨房分隔）
  - 走廊墙：x=0, y从2.5到4 和 y从-0.5到0.5（留1m门洞 y=1~2）
  - 家具：沙发(3.5,-3)、茶几(3.5,-2)、电视柜(4.8,-2)
         床(-3.5,3)、衣柜(-4.8,1)、床头柜(-2.5,3.5)
         橱柜(4.8,3)、冰箱(3.5,3.5)、餐桌(2,2.5)
         盆栽(-1,-3)、椅子(1.5,1.5 / 2.5,1.5)
         充电桩(-1,-3)
"""
import struct

# 地图参数
resolution = 0.05  # 5cm/像素
width = int(10.0 / resolution)   # 200 像素
height = int(8.0 / resolution)   # 160 像素
origin_x = -5.0  # 地图原点 x（世界坐标）
origin_y = -4.0  # 地图原点 y（世界坐标）

# 墙体厚度（像素）
wall_thickness = 3

def world_to_pixel(x, y):
    """世界坐标转像素坐标"""
    px = int((x - origin_x) / resolution)
    py = int((y - origin_y) / resolution)
    return px, py

def in_bounds(px, py):
    return 0 <= px < width and 0 <= py < height

# 初始化地图为全白（自由空间）
grid = [[255 for _ in range(width)] for _ in range(height)]

def draw_wall(x1, y1, x2, y2, thickness=wall_thickness):
    """画一条墙体线段（世界坐标）"""
    px1, py1 = world_to_pixel(x1, y1)
    px2, py2 = world_to_pixel(x2, y2)

    # Bresenham 画线
    dx = abs(px2 - px1)
    dy = abs(py2 - py1)
    sx = 1 if px1 < px2 else -1
    sy = 1 if py1 < py2 else -1
    err = dx - dy

    while True:
        # 画粗线
        for ox in range(-thickness, thickness + 1):
            for oy in range(-thickness, thickness + 1):
                tx, ty = px1 + ox, py1 + oy
                if in_bounds(tx, ty):
                    grid[ty][tx] = 0  # 黑色=占据
        if px1 == px2 and py1 == py2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            px1 += sx
        if e2 < dx:
            err += dx
            py1 += sy

def draw_box_obstacle(cx, cy, size_x, size_y):
    """画一个矩形障碍物（世界坐标，中心点+尺寸）"""
    x1 = cx - size_x / 2
    x2 = cx + size_x / 2
    y1 = cy - size_y / 2
    y2 = cy + size_y / 2
    t = max(1, wall_thickness - 1)
    draw_wall(x1, y1, x2, y1, t)  # 下边
    draw_wall(x2, y1, x2, y2, t)  # 右边
    draw_wall(x2, y2, x1, y2, t)  # 上边
    draw_wall(x1, y2, x1, y1, t)  # 左边

def draw_cylinder_obstacle(cx, cy, radius):
    """画一个圆柱形障碍物（世界坐标）"""
    px, py = world_to_pixel(cx, cy)
    r = int(radius / resolution)
    for y in range(py - r - 1, py + r + 2):
        for x in range(px - r - 1, px + r + 2):
            if in_bounds(x, y):
                if (x - px) ** 2 + (y - py) ** 2 <= r ** 2:
                    grid[y][x] = 0

# ===== 外墙 =====
draw_wall(-5, -4, 5, -4)    # 南墙
draw_wall(-5, 4, 5, 4)      # 北墙
draw_wall(5, -4, 5, 4)      # 东墙
draw_wall(-5, -4, -5, 4)    # 西墙

# ===== 内墙 =====
# 客厅/卧室分隔 (y=0, x从-5到-2, 留4m门洞)
draw_wall(-5, 0, -2, 0)
# 客厅/厨房分隔 (y=0, x从2到5, 留4m门洞)
draw_wall(2, 0, 5, 0)
# 注: wall_hall_1 已移除，4m宽门洞足够通行

# ===== 家具（作为障碍物） =====
# 客厅
draw_box_obstacle(3.5, -3, 1.5, 0.6)    # 沙发
draw_box_obstacle(3.5, -2, 0.8, 0.5)    # 茶几
draw_box_obstacle(4.8, -2, 0.3, 2.0)    # 电视柜

# 卧室
draw_box_obstacle(-3.5, 3, 1.5, 2.0)    # 床
draw_box_obstacle(-4.8, 1, 0.5, 1.5)    # 衣柜
draw_box_obstacle(-2.5, 3.5, 0.4, 0.4)  # 床头柜

# 厨房
draw_box_obstacle(4.8, 3, 0.4, 3.0)     # 橱柜
draw_box_obstacle(3.5, 3.5, 0.7, 0.7)   # 冰箱
draw_cylinder_obstacle(2, 2.5, 0.4)      # 餐桌

# 其他
draw_cylinder_obstacle(-1, -3, 0.2)      # 盆栽
draw_box_obstacle(1.2, 3.0, 0.4, 0.4)   # 椅子1
draw_box_obstacle(2.8, 3.0, 0.4, 0.4)   # 椅子2

# 充电桩（小障碍物，机器人需要靠近但不碰撞）
draw_box_obstacle(-1.0, -3.0, 0.3, 0.2)

# ===== 写入 PGM 文件 =====
output_file = 'home_map.pgm'
with open(output_file, 'wb') as f:
    # PGM 头部 (P5 = binary PGM)
    header = f'P5\n{width} {height}\n255\n'
    f.write(header.encode('ascii'))
    # 像素数据（按行写入）
    # PGM格式: row 0 = 图像顶部 = 北（y_max）
    # grid[0] = 南墙（y_min），grid[height-1] = 北墙（y_max）
    # 所以需要反向写入，让 PGM row 0 = grid[height-1] = 北墙
    for y in range(height - 1, -1, -1):
        row = bytes(grid[y])
        f.write(row)

# ===== 写入 YAML 文件 =====
yaml_content = f"""image: home_map.pgm
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

with open('home_map.yaml', 'w') as f:
    f.write(yaml_content)

print(f"地图生成完成: {output_file}")
print(f"  尺寸: {width} x {height} 像素")
print(f"  分辨率: {resolution} m/像素")
print(f"  覆盖范围: {width * resolution}m x {height * resolution}m")
print(f"  原点: ({origin_x}, {origin_y})")
