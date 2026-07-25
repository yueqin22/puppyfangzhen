#!/usr/bin/env python3
"""检查地图门口区域是否通畅"""
import sys

pgm_file = sys.argv[1] if len(sys.argv) > 1 else 'home_map.pgm'

with open(pgm_file, 'rb') as f:
    header = f.readline().decode().strip()
    dims = f.readline().decode().strip().split()
    w, h = int(dims[0]), int(dims[1])
    maxval = f.readline().decode().strip()
    data = f.read()

resolution = 0.05
origin_x, origin_y = -5.0, -4.0

def get_pixel(x_world, y_world):
    px = int((x_world - origin_x) / resolution)
    py = int((y_world - origin_y) / resolution)
    if 0 <= px < w and 0 <= py < h:
        return data[py * w + px]
    return -1

print(f"地图: {w}x{h}, 分辨率: {resolution}")
print(f"原点: ({origin_x}, {origin_y})")
print()

# 检查门口区域 (y=0, x从-1.5到1.5)
print("=== 门口区域 (y=0附近) ===")
print("     " + "".join([f"{x:4.1f}" for x in [i*0.1-1.5 for i in range(31)]]))
for y_world in [-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15]:
    py = int((y_world - origin_y) / resolution)
    row = f"y={y_world:5.2f}: "
    for x_world in [i*0.1-1.5 for i in range(31)]:
        val = get_pixel(x_world, y_world)
        if val < 0:
            row += "  ? "
        elif val < 128:
            row += "  # "
        else:
            row += "  . "
    print(row)

print()
print("# = 障碍物, . = 自由空间, ? = 越界")
print()

# 检查关键路径点
print("=== 关键航点检查 ===")
waypoints = [
    (2.0, -2.0, "客厅"),
    (0.0, -1.0, "门口南"),
    (0.0, 0.0, "门口中心"),
    (0.0, 1.0, "门口北"),
    (0.0, 1.5, "走廊"),
    (-2.5, 1.5, "卧室"),
    (3.0, 2.0, "厨房"),
    (-0.5, -2.0, "充电桩"),
]
for x, y, name in waypoints:
    val = get_pixel(x, y)
    status = "障碍物!" if val < 128 else "自由空间"
    print(f"  {name:8s} ({x:5.1f}, {y:5.1f}): {status} (val={val})")
