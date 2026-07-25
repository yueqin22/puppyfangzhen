"""Frontier Manager: detection, clustering, and scoring.

Upgraded from the basic frontier detection in OccupancyGrid to support:
  - Multi-factor scoring (info_gain, path_cost, risk, heading_change, retry_penalty)
  - Spatial clustering to avoid duplicate frontiers in adjacent cells
  - Unreachable goal tracking with spatial zones
  - Visited frontier memory with TTL

This implements the "research-grade exploration" recommendation from
the technical upgrade plan (gaijin1.md section 6).
"""
import math
import numpy as np
from collections import defaultdict
from dataclasses import dataclass
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION


@dataclass(frozen=True)
class FrontierSelection:
    """Selected frontier candidate plus bookkeeping metadata."""

    goal: tuple[float, float] | None
    info_gain: float = 0.0
    distance: float = 0.0
    score: float = 0.0
    reachable_count: int = 0
    safe_count: int = 0
    unsafe_count: int = 0


class FrontierManager:
    """Manages frontier detection, scoring, and memory for exploration."""

    def __init__(self, occ_grid, costmap, exploration_radius=8.0, config=None):
        self.occ_grid = occ_grid
        self.costmap = costmap
        self.exploration_radius = exploration_radius

        # Frontier memory (persisted via SessionStore)
        self.unreachable_goals = set()  # (gx, gy) grid cells
        self.visited_frontiers = {}     # (gx, gy) -> frame visited
        self.visited_count = defaultdict(int)  # (gx, gy) -> visit count

        # P2-1: 评分权重从 config 读取（exploration.yaml），默认值与原硬编码一致
        cfg = config or {}
        self.w_info_gain = cfg.get('w_info_gain', 0.30)
        self.w_path_cost = cfg.get('w_path_cost', 0.25)
        self.w_risk = cfg.get('w_risk', 0.15)
        self.w_heading_change = cfg.get('w_heading_change', 0.15)
        self.w_retry_penalty = cfg.get('w_retry_penalty', 0.15)

        # TTL for visited frontiers (frames)
        self.visited_ttl = cfg.get('visited_ttl', 300)
        # Spatial zone radius for marking unreachable (cells)
        self.unreachable_radius = cfg.get('unreachable_radius', 3)
        # Distance at which repeated visits become heavily penalized.
        self.revisit_penalty_radius = cfg.get('revisit_penalty_radius', 1.0)
        self.revisit_penalty_weight = cfg.get('revisit_penalty_weight', 0.35)
        # Cluster merge radius
        self.cluster_merge_radius = cfg.get('cluster_merge_radius', 0.5)

        # v5.7: 路径质量感知评分参数
        self.w_path_quality = cfg.get('w_path_quality', 0.10)
        self.corridor_width_threshold = cfg.get('corridor_width_threshold', 0.7)
        self.path_bend_penalty = cfg.get('path_bend_penalty', 0.05)

        # v5.9: 边界区域建模参数
        self.region_clustering_enabled = cfg.get('region_clustering_enabled', True)
        self.region_min_size = cfg.get('region_min_size', 3)

        # v5.10: 不可达区域增强 — 失败原因分类和差异化冷却
        self.failure_cooling = {
            'blocked': cfg.get('cooling_blocked', 500),    # 物理阻挡，长时间冷却
            'narrow': cfg.get('cooling_narrow', 300),      # 窄通道暂时不可达
            'amcl_lost': cfg.get('cooling_amcl_lost', 100), # AMCL丢失导致，可重试
            'oscillation': cfg.get('cooling_oscillation', 200), # 震荡导致
        }

        # v6.12: Pareto边界选择
        self.use_pareto_selection = cfg.get('use_pareto_selection', False)

        # v5.10: 失败原因记录 (gx, gy) -> (reason, frame)
        self._failure_reasons = {}

        # v5.9: 区域缓存
        self._region_cache = {}
        self._region_cache_frame = -1

    def find_frontiers(self, rx, ry, robot_yaw=0.0, max_frontiers=15):
        """Find frontier candidates, clustered and scored.

        Args:
            rx, ry: robot position
            robot_yaw: robot heading (for heading alignment scoring)
            max_frontiers: max number to return
        """
        raw_frontiers = self.occ_grid.find_frontiers(rx, ry, max_frontiers=max_frontiers * 3)
        if not raw_frontiers:
            return []

        # Cluster nearby frontiers (DBSCAN-like)
        clusters = self._cluster_frontiers(raw_frontiers)

        # Score and sort each cluster
        scored = []
        for cx, cy, size in clusters:
            score = self._score_frontier(rx, ry, cx, cy, size, robot_yaw)
            scored.append((cx, cy, size, score))

        scored.sort(key=lambda f: -f[3])  # descending by score
        return [(f[0], f[1], f[2]) for f in scored[:max_frontiers]]

    def _cluster_frontiers(self, frontiers, merge_radius=None):
        """Merge frontiers within merge_radius into cluster centroids."""
        if merge_radius is None:
            merge_radius = self.cluster_merge_radius
        if not frontiers:
            return []
        visited = [False] * len(frontiers)
        clusters = []
        for i, (fx, fy, size) in enumerate(frontiers):
            if visited[i]:
                continue
            cluster_points = [(fx, fy, size)]
            visited[i] = True
            for j in range(i + 1, len(frontiers)):
                if visited[j]:
                    continue
                jx, jy, jsize = frontiers[j]
                if math.sqrt((jx - fx)**2 + (jy - fy)**2) < merge_radius:
                    cluster_points.append((jx, jy, jsize))
                    visited[j] = True
            total_size = sum(p[2] for p in cluster_points)
            avg_x = sum(p[0] for p in cluster_points) / len(cluster_points)
            avg_y = sum(p[1] for p in cluster_points) / len(cluster_points)
            clusters.append((avg_x, avg_y, total_size))
        return clusters

    def _score_frontier(self, rx, ry, fx, fy, size, robot_yaw):
        """Multi-factor frontier score (higher = better).

        score = w1*info_gain - w2*path_cost - w3*risk - w4*heading_change - w5*retry_penalty
        """
        # 1. Information gain: larger frontier = more unknown cells to explore
        info_gain = min(1.0, size / 20.0)

        # 2. Path cost: distance to frontier (normalized)
        dist = math.sqrt((fx - rx)**2 + (fy - ry)**2)
        path_cost = min(1.0, dist / self.exploration_radius)

        # 3. Risk: costmap cost at frontier location
        cost = self.costmap.get_cost(fx, fy)
        risk = min(1.0, cost / 128.0)

        # 4. Heading change: how much robot needs to turn
        angle_to_frontier = math.atan2(fy - ry, fx - rx)
        heading_diff = abs(angle_to_frontier - robot_yaw)
        while heading_diff > math.pi:
            heading_diff = 2 * math.pi - heading_diff
        heading_change = heading_diff / math.pi

        # 5. Retry penalty: has this frontier failed before?
        gx, gy = self.occ_grid.world_to_grid(fx, fy)
        retry_penalty = 0.0
        if (gx, gy) in self.unreachable_goals:
            retry_penalty = 1.0
        elif (gx, gy) in self.visited_frontiers:
            retry_penalty = 0.5

        score = (self.w_info_gain * info_gain
                 - self.w_path_cost * path_cost
                 - self.w_risk * risk
                 - self.w_heading_change * heading_change
                 - self.w_retry_penalty * retry_penalty)
        return score

    def mark_unreachable(self, wx, wy):
        """Mark a goal and its spatial zone as unreachable."""
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        r = self.unreachable_radius
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx*dx + dy*dy <= r*r:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        self.unreachable_goals.add((nx, ny))

    def mark_area_unreachable(self, wx, wy, radius_m=3.0):
        """Mark a large area as unreachable (v2.9j).

        Used when consecutive no_progress RECOVERs happen in the same area,
        indicating the entire region is genuinely inaccessible (e.g. behind
        a wall with no doorway). The default 0.5m radius of mark_unreachable
        is too small to cover frontier clusters along a wall edge.

        Args:
            wx, wy: center of the area to block (world coordinates)
            radius_m: radius in meters (default 3.0m)
        """
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        r = int(radius_m / GRID_RESOLUTION)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx*dx + dy*dy <= r*r:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        self.unreachable_goals.add((nx, ny))

    def clear_unreachable(self):
        """Clear all unreachable goal markings.

        Called after AMCL recovery when position error was high, since
        goals marked unreachable with a wrong position estimate may
        actually be reachable from the corrected position.
        """
        self.unreachable_goals.clear()

    def mark_visited(self, wx, wy, frame):
        """Record that a frontier was visited at this frame."""
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        self.visited_count[(gx, gy)] += 1
        self.visited_frontiers[(gx, gy)] = frame

    def is_persistent_frontier(self, wx, wy):
        """Check if frontier was visited before (persistent frontier behind wall)."""
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        return (gx, gy) in self.visited_frontiers

    def is_unreachable(self, wx, wy):
        """Check if a world position is in an unreachable zone."""
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        return (gx, gy) in self.unreachable_goals

    def filter_reachable(self, frontiers, current_frame):
        """Filter out unreachable and recently-visited frontiers."""
        result = []
        for fx, fy, size in frontiers:
            if self.is_unreachable(fx, fy):
                continue
            gx, gy = self.occ_grid.world_to_grid(fx, fy)
            last_visit = self.visited_frontiers.get((gx, gy))
            if last_visit is not None and (current_frame - last_visit) < self.visited_ttl:
                continue
            result.append((fx, fy, size))
        return result

    def gc_visited(self, current_frame):
        """Garbage-collect expired visited_frontiers entries."""
        if len(self.visited_frontiers) > 50:
            expired = [k for k, fr in self.visited_frontiers.items()
                       if (current_frame - fr) > self.visited_ttl]
            for k in expired:
                del self.visited_frontiers[k]

    def has_recent_oscillation(self, doorway_crossing_frames, current_frame, cooldown=40):
        """Return True when doorway oscillation happened within cooldown frames."""
        return any(current_frame - f <= cooldown for f in doorway_crossing_frames[-5:])

    def score_frontiers(
            self,
            frontiers,
            rx,
            ry,
            robot_yaw=0.0,
            safe_cost_threshold=120):
        """Score frontier candidates with multi-factor utility.

        Input tuples are expected to be `(fx, fy, size, info_gain, dist)`.
        Returns tuples in the form:
          `(fx, fy, size, info_gain, dist, score, cost, safe)`
        sorted by descending score.
        """
        if not frontiers:
            return []

        max_info_gain = max((frontier[3] for frontier in frontiers), default=1.0)
        max_dist = max((frontier[4] for frontier in frontiers), default=1.0)
        if max_info_gain <= 0:
            max_info_gain = 1.0
        if max_dist <= 0:
            max_dist = 1.0

        scored = []
        for fx, fy, size, info_gain, dist in frontiers:
            cost = int(self.costmap.get_cost(fx, fy))
            safe = cost < safe_cost_threshold
            score = self._score_ranked_frontier(
                rx=rx,
                ry=ry,
                robot_yaw=robot_yaw,
                fx=fx,
                fy=fy,
                info_gain=info_gain,
                distance=dist,
                max_info_gain=max_info_gain,
                max_dist=max_dist,
                cost=cost,
            )
            scored.append((fx, fy, size, info_gain, dist, score, cost, safe))

        scored.sort(key=lambda frontier: (frontier[5], frontier[3], -frontier[6]), reverse=True)
        return scored

    def _score_ranked_frontier(
            self,
            rx,
            ry,
            robot_yaw,
            fx,
            fy,
            info_gain,
            distance,
            max_info_gain,
            max_dist,
            cost):
        """Score a frontier tuple generated by info-gain ranking."""
        info_gain_norm = info_gain / max_info_gain if max_info_gain > 0 else 0.0
        distance_norm = distance / max_dist if max_dist > 0 else 0.0
        risk = min(1.0, cost / 128.0)

        angle_to_frontier = math.atan2(fy - ry, fx - rx)
        heading_diff = abs(angle_to_frontier - robot_yaw)
        while heading_diff > math.pi:
            heading_diff = 2 * math.pi - heading_diff
        heading_change = heading_diff / math.pi

        gx, gy = self.occ_grid.world_to_grid(fx, fy)
        retry_penalty = 0.0
        if (gx, gy) in self.unreachable_goals:
            retry_penalty = 1.0
        elif (gx, gy) in self.visited_frontiers:
            retry_penalty = 0.5

        revisit_penalty = 0.0
        if self.visited_frontiers:
            revisit_penalty = self._nearest_visited_penalty(fx, fy)

        score = (
            self.w_info_gain * info_gain_norm
            - self.w_path_cost * distance_norm
            - self.w_risk * risk
            - self.w_heading_change * heading_change
            - self.w_retry_penalty * retry_penalty
            - self.revisit_penalty_weight * revisit_penalty
        )
        return score

    # ===== v5.7: 路径质量感知边界评分 =====

    def compute_path_quality(self, path):
        """v5.7: 计算路径质量评分。

        评估因素:
          - 路径长度 vs 直线距离（弯折程度）
          - 走廊宽度（沿途是否经过窄通道）
          - 路径平滑度（转弯次数）

        Args:
            path: [(x,y), ...] A*路径waypoint列表

        Returns:
            float: 路径质量评分 (0=差, 1=好)
        """
        if not path or len(path) < 2:
            return 0.5

        # 1. 弯折度 = 实际长度 / 直线距离
        total_len = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_len += math.sqrt(dx * dx + dy * dy)

        direct_dist = math.sqrt(
            (path[-1][0] - path[0][0]) ** 2 +
            (path[-1][1] - path[0][1]) ** 2)
        if direct_dist < 1e-6:
            return 0.5
        bend_ratio = total_len / direct_dist
        bend_penalty = min(1.0, max(0.0, 1.0 - (bend_ratio - 1.0) / 2.0))

        # 2. 走廊宽度评分
        min_corridor = float('inf')
        for wx, wy in path:
            # 沿路径采样每个点的局部空间
            clearance = self._estimate_local_clearance(wx, wy)
            min_corridor = min(min_corridor, clearance)
        corridor_score = min(1.0, min_corridor / self.corridor_width_threshold)

        # 3. 转弯次数（每转弯一次扣分）
        n_turns = 0
        for i in range(1, len(path) - 1):
            d1 = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            d2 = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            cross = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(cross) > 1e-6:
                n_turns += 1
        smoothness = max(0.0, 1.0 - n_turns * self.path_bend_penalty)

        # 综合
        return 0.4 * bend_penalty + 0.35 * corridor_score + 0.25 * smoothness

    def _estimate_local_clearance(self, wx, wy):
        """估计某点的局部空间（到最近障碍物的距离）"""
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        min_dist = 10.0
        for r in range(1, 6):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx * dx + dy * dy <= r * r:
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                            if self.occ_grid.is_occupied(nx, ny):
                                actual_dist = r * GRID_RESOLUTION
                                if actual_dist < min_dist:
                                    min_dist = actual_dist
            if min_dist < 10.0:
                break
        return min_dist

    # ===== v5.9: 边界区域建模 =====

    def cluster_frontiers_to_regions(self, frontiers, rx, ry):
        """v5.9: 将frontier聚类升级为区域(region)。

        每个region包含:
          - 主方向（质心朝向）
          - 入口点（最接近机器人的点）
          - 区域大小
          - 内部frontier列表

        Returns:
            list of dict: [{center, entry_point, size, direction, frontiers}]
        """
        if not frontiers:
            return []

        # DBSCAN风格聚类
        visited = set()
        regions = []
        merge_dist = self.cluster_merge_radius * 2  # 区域用更大的合并半径

        for i, f in enumerate(frontiers):
            if i in visited:
                continue
            cluster = [f]
            visited.add(i)
            # BFS扩展聚类
            queue = [i]
            while queue:
                j = queue.pop(0)
                fx, fy = frontiers[j][0], frontiers[j][1]
                for k in range(i + 1, len(frontiers)):
                    if k in visited:
                        continue
                    kx, ky = frontiers[k][0], frontiers[k][1]
                    dist = math.sqrt((fx - kx) ** 2 + (fy - ky) ** 2)
                    if dist < merge_dist:
                        visited.add(k)
                        cluster.append(frontiers[k])
                        queue.append(k)

            if len(cluster) >= self.region_min_size:
                # 计算区域属性
                total_size = sum(f[2] if len(f) > 2 else 1 for f in cluster)
                cx = sum(f[0] for f in cluster) / len(cluster)
                cy = sum(f[1] for f in cluster) / len(cluster)

                # 入口点：最接近机器人的frontier
                entry = min(cluster, key=lambda f: (f[0] - rx) ** 2 + (f[1] - ry) ** 2)

                # 主方向
                direction = math.atan2(cy - ry, cx - rx)

                regions.append({
                    'center': (cx, cy),
                    'entry_point': (entry[0], entry[1]),
                    'size': total_size,
                    'direction': direction,
                    'frontier_count': len(cluster),
                    'frontiers': cluster,
                })

        # 按区域大小排序
        regions.sort(key=lambda r: r['size'], reverse=True)
        return regions

    def select_region(self, frontiers, rx, ry, robot_yaw=0.0):
        """v5.9: 选择最佳区域而非单个frontier。

        Returns:
            FrontierSelection: 入口点作为goal
        """
        regions = self.cluster_frontiers_to_regions(frontiers, rx, ry)
        if not regions:
            # 回退到普通选择
            return self.select_frontier(frontiers, rx=rx, ry=ry, robot_yaw=robot_yaw)

        # 评分每个区域
        best_region = None
        best_score = -float('inf')
        for region in regions:
            ex, ey = region['entry_point']
            dist = math.sqrt((ex - rx) ** 2 + (ey - ry) ** 2)
            dist_norm = min(1.0, dist / self.exploration_radius)
            size_norm = min(1.0, region['size'] / 30.0)

            # 方向对齐
            dir_diff = abs(region['direction'] - robot_yaw)
            while dir_diff > math.pi:
                dir_diff = 2 * math.pi - dir_diff
            align = 1.0 - dir_diff / math.pi

            score = (self.w_info_gain * size_norm
                     - self.w_path_cost * dist_norm
                     + self.w_heading_change * align * 0.5)

            if score > best_score:
                best_score = score
                best_region = region

        if best_region:
            ex, ey = best_region['entry_point']
            return FrontierSelection(
                goal=(ex, ey),
                info_gain=best_region['size'],
                distance=math.sqrt((ex - rx) ** 2 + (ey - ry) ** 2),
                score=best_score,
                reachable_count=len(frontiers),
                safe_count=len(frontiers),
                unsafe_count=0,
            )
        return FrontierSelection(goal=None, reachable_count=len(frontiers))

    # ===== v5.10: 不可达区域增强 =====

    def mark_unreachable_with_reason(self, wx, wy, reason='blocked', radius_m=3.0):
        """v5.10: 标记不可达区域并记录失败原因。

        Args:
            wx, wy: 世界坐标
            reason: 失败原因 ('blocked'/'narrow'/'amcl_lost'/'oscillation')
            radius_m: 标记半径(米)
        """
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        r = int(radius_m / GRID_RESOLUTION)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        self.unreachable_goals.add((nx, ny))
                        self._failure_reasons[(nx, ny)] = (reason, 0)  # frame=0占位

    def get_failure_reason(self, wx, wy):
        """v5.10: 获取某个区域的失败原因"""
        gx, gy = self.occ_grid.world_to_grid(wx, wy)
        entry = self._failure_reasons.get((gx, gy))
        if entry:
            return entry[0]
        return None

    def filter_reachable_with_cooling(self, frontiers, current_frame):
        """v5.10: 基于失败原因的差异化冷却过滤。

        不同失败原因有不同的冷却时间：
          - blocked: 500帧（物理阻挡，长时间冷却）
          - narrow: 300帧（窄通道可能暂时不可达）
          - amcl_lost: 100帧（AMCL丢失，可较快重试）
          - oscillation: 200帧（震荡导致）
        """
        result = []
        for f in frontiers:
            fx, fy = f[0], f[1]
            if self.is_unreachable(fx, fy):
                # 检查是否过了冷却期
                gx, gy = self.occ_grid.world_to_grid(fx, fy)
                reason_entry = self._failure_reasons.get((gx, gy))
                if reason_entry:
                    reason, marked_frame = reason_entry
                    cooling = self.failure_cooling.get(reason, 300)
                    if current_frame - marked_frame > cooling:
                        # 过了冷却期，允许重试
                        result.append(f)
                # 没有失败原因的不可达点继续被过滤
            else:
                result.append(f)
        return result

    # ===== v6.12: Pareto边界选择 =====

    def select_frontier_pareto(self, frontiers, rx, ry, robot_yaw=0.0):
        """v6.12: 使用Pareto前沿进行边界选择。

        将frontier选择建模为多目标优化问题：
          - 最大化信息增益
          - 最小化路径代价
          - 最小化风险
          - 最小化朝向变化

        使用Pareto非支配排序找到前沿，然后用TOPSIS决策。
        """
        try:
            from pareto import find_pareto_front, select_pareto_best
        except ImportError:
            # 回退到普通选择
            return self.select_frontier(frontiers, rx=rx, ry=ry, robot_yaw=robot_yaw)

        if not frontiers:
            return FrontierSelection(goal=None, reachable_count=0)

        # 准备多目标向量
        solutions = []
        for f in frontiers:
            fx, fy = f[0], f[1]
            size = f[2] if len(f) > 2 else 1
            info_gain = f[3] if len(f) > 3 else size / 20.0
            dist = f[4] if len(f) > 4 else math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)
            cost = int(self.costmap.get_cost(fx, fy))

            angle_to_f = math.atan2(fy - ry, fx - rx)
            heading_diff = abs(angle_to_f - robot_yaw)
            while heading_diff > math.pi:
                heading_diff = 2 * math.pi - heading_diff

            # 目标: (信息增益↑, 距离↓, 风险↓, 朝向变化↓)
            objectives = (info_gain, -dist, -cost, -heading_diff)
            solutions.append((objectives, f))

        higher_is_better = [True, True, True, True]

        # 找到Pareto前沿
        pareto_front = find_pareto_front(solutions, higher_is_better)

        if not pareto_front:
            return self.select_frontier(frontiers, rx=rx, ry=ry, robot_yaw=robot_yaw)

        # 从Pareto前沿中用TOPSIS选择
        try:
            chosen = select_pareto_best(pareto_front, higher_is_better)
        except Exception:
            # 回退：选信息增益最大的
            chosen = max(pareto_front, key=lambda s: s[0][0])

        f = chosen[1]
        return FrontierSelection(
            goal=(f[0], f[1]),
            info_gain=f[3] if len(f) > 3 else 0.0,
            distance=f[4] if len(f) > 4 else 0.0,
            score=0.0,
            reachable_count=len(frontiers),
            safe_count=len(pareto_front),
            unsafe_count=len(frontiers) - len(pareto_front),
        )

    def _nearest_visited_penalty(self, fx, fy):
        """Penalty for selecting a frontier too close to a recently visited one."""
        nearest_dist = None
        for gx, gy in self.visited_frontiers:
            wx, wy = self._grid_to_world(gx, gy)
            dist = math.sqrt((fx - wx) ** 2 + (fy - wy) ** 2)
            if nearest_dist is None or dist < nearest_dist:
                nearest_dist = dist
        if nearest_dist is None or nearest_dist >= self.revisit_penalty_radius:
            return 0.0
        return 1.0 - (nearest_dist / self.revisit_penalty_radius)

    def _grid_to_world(self, gx, gy):
        """Convert grid coordinates to world, with fallback for light test doubles."""
        if hasattr(self.occ_grid, "grid_to_world"):
            return self.occ_grid.grid_to_world(gx, gy)
        return gx / 10.0, gy / 10.0

    def filter_frontiers_with_metadata(
            self,
            frontiers,
            current_frame,
            robot_y,
            doorway_crossing_frames,
            doorway_y_margin=0.3,
            oscillation_cooldown=40):
        """Filter frontiers using memory and doorway-oscillation heuristics.

        Accepts frontiers in either `(fx, fy, size)` or
        `(fx, fy, size, info_gain, dist)` format and returns the same tuple
        shape it was given.
        """
        result = []
        recent_oscillation = self.has_recent_oscillation(
            doorway_crossing_frames,
            current_frame,
            cooldown=oscillation_cooldown,
        )
        for frontier in frontiers:
            fx, fy = frontier[0], frontier[1]
            if self.is_unreachable(fx, fy):
                continue
            gx, gy = self.occ_grid.world_to_grid(fx, fy)
            last_visit = self.visited_frontiers.get((gx, gy))
            if last_visit is not None and (current_frame - last_visit) < self.visited_ttl:
                continue
            if recent_oscillation and abs(robot_y) > doorway_y_margin:
                if (robot_y > 0 and fy < -doorway_y_margin) or (
                        robot_y < 0 and fy > doorway_y_margin):
                    continue
            result.append(frontier)
        return result

    def select_frontier(
            self,
            frontiers,
            safe_cost_threshold=120,
            rx=None,
            ry=None,
            robot_yaw=0.0):
        """Select the best frontier, preferring safe cells over unsafe ones.

        Input frontier tuples are expected to be
        `(fx, fy, size, info_gain, dist)` and already sorted by utility.
        """
        if rx is not None and ry is not None:
            ranked_frontiers = self.score_frontiers(
                frontiers,
                rx=rx,
                ry=ry,
                robot_yaw=robot_yaw,
                safe_cost_threshold=safe_cost_threshold,
            )
        else:
            ranked_frontiers = []
            for fx, fy, size, info_gain, dist in frontiers:
                cost = self.costmap.get_cost(fx, fy)
                ranked_frontiers.append(
                    (fx, fy, size, info_gain, dist, 0.0, cost, cost < safe_cost_threshold)
                )

        safe_frontiers = [frontier for frontier in ranked_frontiers if frontier[7]]
        unsafe_frontiers = [frontier for frontier in ranked_frontiers if not frontier[7]]

        if safe_frontiers:
            chosen = safe_frontiers[0]
            return FrontierSelection(
                goal=(chosen[0], chosen[1]),
                info_gain=chosen[3],
                distance=chosen[4],
                score=chosen[5],
                reachable_count=len(ranked_frontiers),
                safe_count=len(safe_frontiers),
                unsafe_count=len(unsafe_frontiers),
            )

        if unsafe_frontiers:
            chosen = min(unsafe_frontiers, key=lambda frontier: frontier[6])
            return FrontierSelection(
                goal=(chosen[0], chosen[1]),
                info_gain=chosen[3],
                distance=chosen[4],
                score=chosen[5],
                reachable_count=len(ranked_frontiers),
                safe_count=0,
                unsafe_count=len(unsafe_frontiers),
            )

        return FrontierSelection(
            goal=None,
            reachable_count=len(ranked_frontiers),
            safe_count=0,
            unsafe_count=0,
        )

    def get_state(self):
        """Get serializable state for session persistence."""
        return {
            'unreachable_goals': list(self.unreachable_goals),
            'visited_frontiers': dict(self.visited_frontiers),
            'visited_count': dict(self.visited_count),
            'failure_reasons': {f"{k[0]},{k[1]}": v for k, v in self._failure_reasons.items()},
        }

    def set_state(self, state):
        """Restore state from session persistence."""
        self.unreachable_goals = set(tuple(x) for x in state.get('unreachable_goals', []))
        self.visited_frontiers = {tuple(k): v for k, v in state.get('visited_frontiers', {}).items()}
        self.visited_count = defaultdict(int, {tuple(k): v for k, v in state.get('visited_count', {}).items()})
        self._failure_reasons = {tuple(int(v) for v in k.split(',')): tuple(v)
                                 for k, v in state.get('failure_reasons', {}).items()}
