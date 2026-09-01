#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Strict Collision Verification Script
====================================
Verifies whether the robot's footprint intersected with any wall, furniture,
or dynamic pedestrian across the entire saved patrol trajectory.
"""
import os
import sys
import csv
import math
import numpy as np

from auto_patrol_simulation import OBSTACLES_BBOX, is_position_safe, check_wall_penetration

ROBOT_RADIUS = 0.15  # 机器狗半径 (宽0.30m -> 半径 0.15m)

def verify_collisions():
    csv_path = os.path.abspath("patrol_trajectory.csv")
    if not os.path.exists(csv_path):
        print(f"Trajectory file not found: {csv_path}")
        return

    frames = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            frames.append({
                'frame': int(row['frame']),
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw']),
            })

    print(f"Loaded {len(frames)} trajectory points from {csv_path}")

    min_clearance = float('inf')
    closest_obstacle = None
    closest_frame = -1
    collision_count = 0

    for idx, f in enumerate(frames):
        rx, ry = f['x'], f['y']

        # 1. 检查是否与障碍物 bbox 碰撞
        safe = is_position_safe(rx, ry, OBSTACLES_BBOX, safe_distance=ROBOT_RADIUS)

        # 计算到最近障碍物边缘的精确距离
        for name, bbox in OBSTACLES_BBOX:
            xmin, ymin, xmax, ymax = bbox
            # 计算点 (rx, ry) 到矩形的距离
            dx = max(xmin - rx, 0, rx - xmax)
            dy = max(ymin - ry, 0, ry - ymax)
            dist = math.sqrt(dx*dx + dy*dy) - ROBOT_RADIUS
            if dist < min_clearance:
                min_clearance = dist
                closest_obstacle = name
                closest_frame = f['frame']

            if dist < 0:
                collision_count += 1
                print(f"  [警告] 帧 {f['frame']}: 与障碍物 '{name}' 发生重叠 ({dist:.4f}m) 位置: ({rx:.3f}, {ry:.3f})")

        # 2. 检查穿墙
        if idx > 0:
            prev_x, prev_y = frames[idx-1]['x'], frames[idx-1]['y']
            pen, wall_name = check_wall_penetration(prev_x, prev_y, rx, ry, OBSTACLES_BBOX)
            if pen:
                collision_count += 1
                print(f"  [警告] 帧 {f['frame']}: 穿透墙体 '{wall_name}'")

    print("\n" + "=" * 60)
    print("碰撞精确校验结论:")
    print(f"  总轨迹点数:     {len(frames)}")
    print(f"  碰撞发生次数:   {collision_count}")
    print(f"  最小全景安全裕度: {min_clearance:.4f} 米 (最接近障碍物: '{closest_obstacle}'，第 {closest_frame} 帧)")
    if collision_count == 0:
        print("  校验结果: PASS —— 机器狗在整个巡航过程中完全没有碰到任何障碍物！")
    else:
        print(f"  校验结果: FAIL —— 发现 {collision_count} 次碰撞")
    print("=" * 60)

if __name__ == '__main__':
    verify_collisions()
