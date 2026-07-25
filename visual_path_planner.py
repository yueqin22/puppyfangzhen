"""可视化层轻量级A*路径规划器 + TEB式轨迹平滑

替代visual_sim.py中的直线追踪，实现：
    目标点 → A*静态路径 → 轨迹平滑 → 路径跟随 → 局部避障

A*在栅格上搜索，考虑所有墙体和家具。
轨迹平滑用RDP简化 + 样条插值，类似TEB的弹性带效果。
"""
import math
import heapq
from typing import List, Tuple, Optional

import numpy as np

# 栅格参数
GRID_RES = 0.10  # 10cm精度
GRID_MARGIN = 0.35  # 障碍物膨胀半径（机器人半径0.25 + 余量0.10）

# 世界范围
WORLD_X_MIN = -5.5
WORLD_X_MAX = 5.5
WORLD_Y_MIN = -4.5
WORLD_Y_MAX = 4.5

GRID_W = int((WORLD_X_MAX - WORLD_X_MIN) / GRID_RES)
GRID_H = int((WORLD_Y_MAX - WORLD_Y_MIN) / GRID_RES)


def _world_to_grid(x, y):
    gx = int((x - WORLD_X_MIN) / GRID_RES)
    gy = int((y - WORLD_Y_MIN) / GRID_RES)
    return gx, gy


def _grid_to_world(gx, gy):
    x = WORLD_X_MIN + (gx + 0.5) * GRID_RES
    y = WORLD_Y_MIN + (gy + 0.5) * GRID_RES
    return x, y


def _build_static_grid(obstacles_bbox):
    """构建静态障碍物栅格地图"""
    grid = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
    for name, (xmin, ymin, xmax, ymax) in obstacles_bbox:
        # 膨胀
        ex_min = xmin - GRID_MARGIN
        ey_min = ymin - GRID_MARGIN
        ex_max = xmax + GRID_MARGIN
        ey_max = ymax + GRID_MARGIN
        gx0, gy0 = _world_to_grid(ex_min, ey_min)
        gx1, gy1 = _world_to_grid(ex_max, ey_max)
        gx0 = max(0, min(GRID_W - 1, gx0))
        gx1 = max(0, min(GRID_W - 1, gx1))
        gy0 = max(0, min(GRID_H - 1, gy0))
        gy1 = max(0, min(GRID_H - 1, gy1))
        grid[gy0:gy1 + 1, gx0:gx1 + 1] = 255
    return grid


def _is_safe_cell(gx, gy, grid):
    if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H:
        return False
    return grid[gy, gx] == 0


def _heuristic(gx, gy, tx, ty):
    return math.sqrt((gx - tx) ** 2 + (gy - ty) ** 2)


def astar_plan(start_x, start_y, goal_x, goal_y, obstacles_bbox,
               max_nodes=5000):
    """A*路径规划

    Returns:
        path: [(x, y), ...] 世界坐标路径点列表，失败返回None
    """
    grid = _build_static_grid(obstacles_bbox)

    sgx, sgy = _world_to_grid(start_x, start_y)
    tgx, tgy = _world_to_grid(goal_x, goal_y)

    # 目标不可达时找最近可通行格
    if not _is_safe_cell(tgx, tgy, grid):
        best_d = float('inf')
        best = None
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                nx, ny = tgx + dx, tgy + dy
                if _is_safe_cell(nx, ny, grid):
                    d = dx * dx + dy * dy
                    if d < best_d:
                        best_d = d
                        best = (nx, ny)
        if best is None:
            return None
        tgx, tgy = best

    # 起点不安全时也找最近可通行格
    if not _is_safe_cell(sgx, sgy, grid):
        best_d = float('inf')
        best = None
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                nx, ny = sgx + dx, sgy + dy
                if _is_safe_cell(nx, ny, grid):
                    d = dx * dx + dy * dy
                    if d < best_d:
                        best_d = d
                        best = (nx, ny)
        if best is None:
            return None
        sgx, sgy = best

    # A*搜索（8方向）
    open_heap = [(0.0, (sgx, sgy))]
    came_from = {(sgx, sgy): None}
    g_score = {(sgx, sgy): 0.0}
    nodes_expanded = 0

    neighbors_8 = [(-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414),
                   (0, -1, 1.0), (0, 1, 1.0),
                   (1, -1, 1.414), (1, 0, 1.0), (1, 1, 1.414)]

    while open_heap and nodes_expanded < max_nodes:
        _, current = heapq.heappop(open_heap)
        nodes_expanded += 1

        if current == (tgx, tgy):
            # 回溯
            path = []
            node = current
            while node is not None:
                wx, wy = _grid_to_world(*node)
                path.append((wx, wy))
                node = came_from[node]
            path.reverse()
            return path

        cgx, cgy = current
        for dx, dy, cost in neighbors_8:
            nx, ny = cgx + dx, cgy + dy
            if not _is_safe_cell(nx, ny, grid):
                continue
            # 对角线不能切角
            if dx != 0 and dy != 0:
                if not _is_safe_cell(cgx + dx, cgy, grid) or \
                   not _is_safe_cell(cgx, cgy + dy, grid):
                    continue
            new_g = g_score[current] + cost
            neighbor = (nx, ny)
            if neighbor not in g_score or new_g < g_score[neighbor]:
                g_score[neighbor] = new_g
                f = new_g + _heuristic(nx, ny, tgx, tgy)
                came_from[neighbor] = current
                heapq.heappush(open_heap, (f, neighbor))

    return None


def rdp_simplify(path, epsilon=0.15):
    """RDP (Ramer-Douglas-Peucker) 路径简化

    去除冗余路径点，只保留转向点。
    类似TEB中的弹性带收缩——减少不必要的曲折。
    """
    if len(path) < 3:
        return path

    def perpendicular_dist(pt, line_start, line_end):
        if line_start == line_end:
            return math.sqrt((pt[0] - line_start[0]) ** 2 + (pt[1] - line_start[1]) ** 2)
        dx = line_end[0] - line_start[0]
        dy = line_end[1] - line_start[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length < 1e-9:
            return math.sqrt((pt[0] - line_start[0]) ** 2 + (pt[1] - line_start[1]) ** 2)
        t = ((pt[0] - line_start[0]) * dx + (pt[1] - line_start[1]) * dy) / (length * length)
        t = max(0, min(1, t))
        proj_x = line_start[0] + t * dx
        proj_y = line_start[1] + t * dy
        return math.sqrt((pt[0] - proj_x) ** 2 + (pt[1] - proj_y) ** 2)

    def rdp_recursive(points, eps):
        if len(points) < 3:
            return points
        # 找最大距离点
        max_dist = 0
        max_idx = 0
        for i in range(1, len(points) - 1):
            d = perpendicular_dist(points[i], points[0], points[-1])
            if d > max_dist:
                max_dist = d
                max_idx = i
        if max_dist > eps:
            left = rdp_recursive(points[:max_idx + 1], eps)
            right = rdp_recursive(points[max_idx:], eps)
            return left[:-1] + right
        else:
            return [points[0], points[-1]]

    result = rdp_recursive(path, epsilon)
    return result


def smooth_path(path, window=3):
    """移动平均平滑路径（TEB式弹性带效果）

    对路径点做加权平均，使轨迹更平滑。
    """
    if len(path) < 3:
        return path

    smoothed = [path[0]]  # 保留起点
    for i in range(1, len(path) - 1):
        sx, sy = 0.0, 0.0
        count = 0
        for j in range(max(0, i - window), min(len(path), i + window + 1)):
            sx += path[j][0]
            sy += path[j][1]
            count += 1
        smoothed.append((sx / count, sy / count))
    smoothed.append(path[-1])  # 保留终点
    return smoothed


def plan_and_smooth(start_x, start_y, goal_x, goal_y, obstacles_bbox):
    """完整路径规划：A* → RDP简化 → 平滑

    Returns:
        (path, raw_path) 平滑后路径和原始A*路径
    """
    raw = astar_plan(start_x, start_y, goal_x, goal_y, obstacles_bbox)
    if raw is None or len(raw) < 2:
        return None, raw

    # RDP简化
    simplified = rdp_simplify(raw, epsilon=0.20)
    # 平滑
    smoothed = smooth_path(simplified, window=2)

    return smoothed, raw


class PathFollower:
    """路径跟随器

    沿平滑路径前进，返回当前目标航点。
    类似TEB的局部轨迹跟踪。
    """

    def __init__(self, lookahead=0.5):
        self.lookahead = lookahead  # 前瞻距离
        self.path = []
        self.current_idx = 0

    def set_path(self, path):
        self.path = path if path else []
        self.current_idx = 0

    def get_target(self, robot_x, robot_y):
        """获取当前跟随目标点"""
        if not self.path:
            return None

        # 找到离机器人最近的路径点
        min_d = float('inf')
        nearest_idx = self.current_idx
        for i in range(self.current_idx, min(len(self.path), self.current_idx + 20)):
            d = math.sqrt((robot_x - self.path[i][0]) ** 2 +
                          (robot_y - self.path[i][1]) ** 2)
            if d < min_d:
                min_d = d
                nearest_idx = i

        self.current_idx = nearest_idx

        # 前瞻：找lookahead距离处的航点
        target_idx = nearest_idx
        accumulated = 0.0
        for i in range(nearest_idx, len(self.path) - 1):
            seg_len = math.sqrt(
                (self.path[i + 1][0] - self.path[i][0]) ** 2 +
                (self.path[i + 1][1] - self.path[i][1]) ** 2)
            accumulated += seg_len
            if accumulated >= self.lookahead:
                target_idx = i + 1
                break
        else:
            target_idx = len(self.path) - 1

        return self.path[target_idx]

    def is_done(self, robot_x, robot_y, threshold=0.4):
        """是否到达路径终点"""
        if not self.path:
            return True
        end = self.path[-1]
        d = math.sqrt((robot_x - end[0]) ** 2 + (robot_y - end[1]) ** 2)
        return d < threshold

    def remaining_distance(self, robot_x, robot_y):
        """剩余路径距离"""
        if not self.path:
            return 0.0
        total = 0.0
        # 到当前航点
        if self.current_idx < len(self.path):
            total += math.sqrt(
                (robot_x - self.path[self.current_idx][0]) ** 2 +
                (robot_y - self.path[self.current_idx][1]) ** 2)
            # 后续路径
            for i in range(self.current_idx, len(self.path) - 1):
                total += math.sqrt(
                    (self.path[i + 1][0] - self.path[i][0]) ** 2 +
                    (self.path[i + 1][1] - self.path[i][1]) ** 2)
        return total
