#!/usr/bin/env python3
"""
D* Lite Incremental Path Planner (v4.0 Innovation)
=====================================================
增量式重规划算法，当环境变化（如动态障碍物移动）时，
无需从头重新规划，只需增量更新受影响的节点。

================================================================================
理论创新
================================================================================

1. 增量启发式搜索 (Incremental Heuristic Search)
--------------------------------------------------------------------------------
D* Lite 是 A* 的增量版本。当边的代价变化时，A* 需要重新搜索整个图，
而 D* Lite 只更新受影响的节点，复杂度从 O(n·log n) 降为 O(k·log n)，
其中 k 为受影响节点数。

关键思想:
    - 搜索从目标到起点反向进行（g 值表示到目标的距离）
    - 维护 rhs (right-hand side) 值: rhs(s) = min_{s'∈succ(s)} (c(s,s') + g(s'))
    - 一致性: g(s) == rhs(s) → 节点一致
    - 代价变化时，仅更新不一致节点的局部邻域

2. Lifelong Planning A* (LPA*) 基础
--------------------------------------------------------------------------------
D* Lite 基于 LPA* (Koenig & Likhachev 2004):
    - 维护两套值: g(s) 和 rhs(s)
    - Key(s) = [min(g(s), rhs(s)) + h(s_start, s); min(g(s), rhs(s))]
    - 优先队列按 Key 字典序排序
    - UpdateVertex: 当边代价变化时，重新计算 rhs 并更新队列

3. 反向搜索与机器人移动
--------------------------------------------------------------------------------
D* Lite 从目标向起点搜索。当机器人移动时:
    - 新起点 h(s) 变化，所有 Key 整体偏移
    - 用 km (key modifier) 偏移量避免全队列更新:
        Key(s) = [min(g(s), rhs(s)) + h(s_start, s) + km;
                  min(g(s), rhs(s))]

4. 代价变化处理
--------------------------------------------------------------------------------
当障碍物移动导致边 (s1, s2) 的代价从 c_old 变为 c_new:
    1. 更新 c(s1, s2) = c_new
    2. 对 s1 调用 UpdateVertex:
       - 若 s2 是 s1 的前驱: rhs(s1) = min(rhs(s1), c_new + g(s2))
       - 若 s2 不再是最优后继: 重新计算 rhs(s1) = min_{s'∈succ} (c + g(s'))
    3. 将 s1 重新加入优先队列

参考文献:
    - Koenig & Likhachev (2002) "D* Lite"
    - Koenig & Likhachev (2005) "Fast Replanning for Navigation in Unknown Terrain"
    - Stentz (1995) "The Focussed D* Algorithm for Real-Time Replanning"
"""

import os
import math
import heapq
import numpy as np
from costmap import Costmap, COST_INSCRIBED, COST_LETHAL, GRID_W, GRID_H
from occupancy_grid import GRID_RESOLUTION, ORIGIN_X, ORIGIN_Y

USE_DSTAR_LITE = os.environ.get("USE_DSTAR_LITE", "0") == "1"

# 代价变化阈值（超过此值才触发增量更新）
COST_CHANGE_THRESHOLD = 5


class DStarLitePlanner:
    """D* Lite 增量重规划器。

    当环境变化时，增量更新路径而非重新规划。

    Parameters
    ----------
    costmap : Costmap
        分层代价地图。
    max_nodes : int
        最大扩展节点数。
    """

    def __init__(self, costmap, max_nodes=3000):
        self.costmap = costmap
        self.max_nodes = max_nodes

        # 8-connected neighbors
        self.neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        # D* Lite 状态
        self.g = {}           # g 值
        self.rhs = {}         # rhs 值
        self.U = []            # 优先队列 (heap)
        self.km = 0            # key modifier

        # 起点和目标
        self.start = None
        self.goal = None
        self.last_start = None

        # 代价缓存 (记录上次代价，用于检测变化)
        self.edge_costs = {}   # (s1, s2) -> cost

        # 统计
        self.total_replans = 0
        self.incremental_updates = 0
        self.full_replans = 0
        self.planning_time = 0.0

    def _world_to_grid(self, x, y):
        """世界坐标 → 栅格坐标。"""
        gx = int((x - ORIGIN_X) / GRID_RESOLUTION)
        gy = int((y - ORIGIN_Y) / GRID_RESOLUTION)
        return max(0, min(GRID_W - 1, gx)), max(0, min(GRID_H - 1, gy))

    def _grid_to_world(self, gx, gy):
        """栅格坐标 → 世界坐标。"""
        return gx * GRID_RESOLUTION + ORIGIN_X, gy * GRID_RESOLUTION + ORIGIN_Y

    def _heuristic(self, s1, s2):
        """Octile 距离启发式。"""
        gx1, gy1 = s1
        gx2, gy2 = s2
        dx = abs(gx1 - gx2)
        dy = abs(gy1 - gy2)
        return (dx + dy) + (1.414 - 2) * min(dx, dy)

    def _cell_cost(self, gx, gy):
        """栅格通行代价。"""
        c = self.costmap.get_cost_grid(gx, gy)
        if c >= COST_LETHAL:
            return float('inf')
        return 1.0 + (c / 128.0) ** 2 * 12.0

    def _edge_cost(self, s1, s2):
        """计算边 (s1→s2) 的代价。"""
        c2 = self._cell_cost(*s2)
        if c2 == float('inf'):
            return float('inf')
        # 对角移动代价
        dx = abs(s1[0] - s2[0])
        dy = abs(s1[1] - s2[1])
        move_cost = 1.414 if (dx + dy == 2) else 1.0
        return move_cost * c2

    def _key(self, s):
        """计算节点 s 的 Key。"""
        g = self.g.get(s, float('inf'))
        r = self.rhs.get(s, float('inf'))
        min_gr = min(g, r)
        h = self._heuristic(self.start, s) if self.start else 0
        return (min_gr + h + self.km, min_gr)

    def _top_key(self):
        """获取优先队列顶部 Key。"""
        if not self.U:
            return (float('inf'), float('inf'))
        return self.U[0][0]

    def _pop(self):
        """弹出优先队列顶部。"""
        return heapq.heappop(self.U)[1]

    def _insert(self, s):
        """插入节点到优先队列。"""
        heapq.heappush(self.U, (self._key(s), s))

    def _remove(self, s):
        """从优先队列移除节点。"""
        self.U = [(k, node) for k, node in self.U if node != s]
        heapq.heapify(self.U)

    def _update_vertex(self, s):
        """更新节点 s 的 rhs 并调整优先队列。"""
        if s != self.goal:
            # rhs(s) = min_{s'∈succ(s)} (c(s,s') + g(s'))
            min_rhs = float('inf')
            for dx, dy, _ in self.neighbors:
                ns = (s[0] + dx, s[1] + dy)
                if not (0 <= ns[0] < GRID_W and 0 <= ns[1] < GRID_H):
                    continue
                edge_c = self._edge_cost(s, ns)
                if edge_c == float('inf'):
                    continue
                candidate = edge_c + self.g.get(ns, float('inf'))
                if candidate < min_rhs:
                    min_rhs = candidate
            self.rhs[s] = min_rhs

        # 从队列中移除 s
        self._remove(s)

        # 如果不一致 (g != rhs)，重新插入
        if self.g.get(s, float('inf')) != self.rhs.get(s, float('inf')):
            self._insert(s)

    def _compute_shortest_path(self):
        """主循环：扩展节点直到队列顶部 key >= start 的 key 且 start 一致。"""
        while self.U:
            top_k = self._top_key()
            start_k = self._key(self.start)
            if top_k >= start_k and \
               self.rhs.get(self.start, float('inf')) == self.g.get(self.start, float('inf')):
                break

            if len(self.U) > self.max_nodes:
                break

            s = self._pop()
            self.total_replans += 1

            old_k = top_k
            new_k = self._key(s)

            g_old = self.g.get(s, float('inf'))
            r_old = self.rhs.get(s, float('inf'))

            if g_old > r_old:
                # Overconsistent: 降低 g
                self.g[s] = self.rhs[s]
                # 更新前驱 (s 的邻居)
                for dx, dy, _ in self.neighbors:
                    ns = (s[0] + dx, s[1] + dy)
                    if 0 <= ns[0] < GRID_W and 0 <= ns[1] < GRID_H:
                        self._update_vertex(ns)
            else:
                # Underconsistent: 提高 g
                self.g[s] = float('inf')
                self._update_vertex(s)
                for dx, dy, _ in self.neighbors:
                    ns = (s[0] + dx, s[1] + dy)
                    if 0 <= ns[0] < GRID_W and 0 <= ns[1] < GRID_H:
                        self._update_vertex(ns)

    def initialize(self, start_x, start_y, goal_x, goal_y):
        """初始化 D* Lite（首次规划）。

        Parameters
        ----------
        start_x, start_y : float
            起点世界坐标。
        goal_x, goal_y : float
            目标世界坐标。
        """
        import time
        t0 = time.time()

        self.start = self._world_to_grid(start_x, start_y)
        self.goal = self._world_to_grid(goal_x, goal_y)

        self.g.clear()
        self.rhs.clear()
        self.U = []
        self.km = 0

        self.g[self.goal] = float('inf')
        self.rhs[self.goal] = 0
        self._insert(self.goal)

        self._compute_shortest_path()

        self.last_start = self.start
        self.full_replans += 1
        self.planning_time = time.time() - t0

    def replan_incremental(self, new_start_x, new_start_y, changed_cells=None):
        """增量重规划。

        当机器人移动或环境变化时调用，增量更新路径。

        Parameters
        ----------
        new_start_x, new_start_y : float
            新起点世界坐标。
        changed_cells : list of (gx, gy) or None
            代价发生变化的栅格列表。

        Returns
        -------
        path : list of (float, float) or None
            更新后的路径。
        """
        import time
        t0 = time.time()

        new_start = self._world_to_grid(new_start_x, new_start_y)

        # 更新 km (key modifier)
        if self.last_start is not None:
            self.km += self._heuristic(self.last_start, new_start)
        self.last_start = new_start
        self.start = new_start

        # 增量更新代价变化的节点
        if changed_cells:
            for cell in changed_cells:
                if 0 <= cell[0] < GRID_W and 0 <= cell[1] < GRID_H:
                    # 更新该节点及其邻居
                    self._update_vertex(cell)
                    for dx, dy, _ in self.neighbors:
                        ns = (cell[0] + dx, cell[1] + dy)
                        if 0 <= ns[0] < GRID_W and 0 <= ns[1] < GRID_H:
                            self._update_vertex(ns)
            self.incremental_updates += 1
        else:
            # 无代价变化，只需更新 start 相关
            self._update_vertex(self.start)

        # 重新计算最短路径
        self._compute_shortest_path()

        self.planning_time = time.time() - t0
        return self.extract_path()

    def extract_path(self):
        """从 g/rhs 值提取路径。"""
        if self.start is None or self.goal is None:
            return None

        path = []
        current = self.start
        visited = {current}

        max_iter = 5000
        while current != self.goal and max_iter > 0:
            max_iter -= 1

            # 找最小代价后继
            best_next = None
            best_cost = float('inf')

            for dx, dy, move_cost in self.neighbors:
                ns = (current[0] + dx, current[1] + dy)
                if not (0 <= ns[0] < GRID_W and 0 <= ns[1] < GRID_H):
                    continue
                edge_c = self._edge_cost(current, ns)
                if edge_c == float('inf'):
                    continue
                g_val = self.g.get(ns, float('inf'))
                total = edge_c + g_val
                if total < best_cost:
                    best_cost = total
                    best_next = ns

            if best_next is None or best_next in visited:
                break

            visited.add(best_next)
            current = best_next
            wx, wy = self._grid_to_world(*current)
            path.append((wx, wy))

        # 添加目标点
        wx, wy = self._grid_to_world(*self.goal)
        path.append((wx, wy))

        return path if path else None

    def plan(self, start_x, start_y, goal_x, goal_y):
        """统一接口：首次规划或增量重规划。"""
        if self.start is None or self.goal is None or \
           self.goal != self._world_to_grid(goal_x, goal_y):
            # 首次规划或目标变化 → 完全重规划
            self.initialize(start_x, start_y, goal_x, goal_y)
            return self.extract_path()
        else:
            # 增量重规划
            return self.replan_incremental(start_x, start_y)

    def detect_cost_changes(self, costmap_snapshot):
        """检测 costmap 中的代价变化。

        Parameters
        ----------
        costmap_snapshot : ndarray
            上次的 costmap 快照。

        Returns
        -------
        changed_cells : list of (gx, gy)
            代价变化的栅格列表。
        """
        current = self.costmap.cost
        if costmap_snapshot is None or costmap_snapshot.shape != current.shape:
            return []

        diff = np.abs(current.astype(int) - costmap_snapshot.astype(int))
        changed = np.argwhere(diff > COST_CHANGE_THRESHOLD)
        return [(int(gx), int(gy)) for gx, gy in changed]

    def get_statistics(self):
        """获取规划统计。"""
        return {
            'total_replans': self.total_replans,
            'full_replans': self.full_replans,
            'incremental_updates': self.incremental_updates,
            'planning_time': self.planning_time,
            'nodes_in_queue': len(self.U),
        }
