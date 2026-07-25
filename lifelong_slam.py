#!/usr/bin/env python3
"""
Lifelong SLAM - Continuous Map Updating (v4.0 Innovation)
============================================================
持续建图系统，自动检测环境变化并增量更新地图。

================================================================================
理论创新
================================================================================

1. Lifelong SLAM 问题
--------------------------------------------------------------------------------
传统 SLAM 假设环境静态，但真实环境是动态的:
    - 家具移动（椅子被推开）
    - 门开关
    - 临时障碍物出现/消失
    - 长期变化（装修）

Lifelong SLAM 的挑战:
    a) 长期累积漂移 (accumulated drift)
    b) 地图不一致 (新旧观测冲突)
    c) 内存增长 (所有历史观测)
    d) 变化检测 (如何区分真实变化和噪声)

2. 变化检测 (Change Detection)
--------------------------------------------------------------------------------
用对数似然比检验 (Log-Likelihood Ratio Test, LRT):

    Λ(s) = log [P(z_t | H_1) / P(z_t | H_0)]

    H_0: 静态假设 (cell 状态未变)
    H_1: 变化假设 (cell 状态已变)

对占据栅格:
    H_0: cell ~ Bernoulli(p_old)
    H_1: cell ~ Bernoulli(p_new)

    Λ = Σ_k [z_k · log(p_new/p_old) + (1-z_k) · log((1-p_new)/(1-p_old))]

    若 Λ > threshold → 检测到变化

贝叶斯变化检测:
    P(change | z_{1:t}) ∝ P(z_{1:t} | change) · P(change)

    使用 Beta-Bernoulli 共轭先验:
    先验: p ~ Beta(α, β)
    后验: p | z ~ Beta(α + Σz, β + n - Σz)

    变化检测: 若后验均值偏移 > δ → 变化

3. 增量地图更新
--------------------------------------------------------------------------------
不重建整个地图，而是:
    a) 检测变化区域 (change detection)
    b) 只更新变化区域的栅格
    c) 保留未变化区域的历史信息

    更新策略:
        new_belief(s) = α · observation(s) + (1-α) · history(s)
    其中 α 为遗忘因子 (forgetting factor)

4. 子地图管理 (Submap Management)
--------------------------------------------------------------------------------
将大地图分割为子地图:
    - 每个子地图独立维护
    - 变化只影响对应子地图
    - 过时子地图可整体替换
    - 减少计算量

子地图一致性:
    - 子地图间用位姿图约束
    - 检测子地图间不一致
    - 触发局部重优化

5. 记忆管理 (Memory Management)
--------------------------------------------------------------------------------
长期运行时，不能存储所有历史观测。

遗忘策略:
    a) 时间衰减: w(t) = exp(-λ·Δt)，近期观测权重高
    b) 空间选择性: 仅在变化区域保留历史
    c) 关键帧选择: 保留代表性观测

参考文献:
    - Konolige & Bowman (2009) "Towards Lifelong Visual Maps"
    - Rosen et al. (2002) "A hybrid navigation system for long-term
      operation in changing environments"
    - Tipaldi et al. (2013) "Lifelong Localization in Changing Dynamic
      Environments"
    - Krajník et al. (2014) "Life-long spatio-temporal exploration of
      dynamic environments"
"""

import os
import math
import time
import numpy as np
from collections import deque
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION

USE_LIFELONG_SLAM = os.environ.get("USE_LIFELONG_SLAM", "0") == "1"


class ChangeDetector:
    """环境变化检测器。

    用对数似然比和贝叶斯更新检测栅格状态变化。

    Parameters
    ----------
    threshold : float
        变化检测阈值 (对数似然比)。
    min_observations : int
        检测变化所需的最小观测次数。
    """

    def __init__(self, threshold=2.5, min_observations=5):
        self.threshold = threshold
        self.min_obs = min_observations

        # 栅格历史观测: (gx, gy) -> list of (occupancy, timestamp)
        self.observations = {}

        # 变化记录
        self.changes_detected = []

        # 统计
        self.total_detections = 0

    def observe(self, gx, gy, occupancy, timestamp=None):
        """记录栅格观测。

        Parameters
        ----------
        gx, gy : int
            栅格坐标。
        occupancy : float
            占据值 [0, 1] (1=占据, 0=空闲)。
        timestamp : float or None
            时间戳 (默认当前时间)。
        """
        if timestamp is None:
            timestamp = time.time()

        key = (gx, gy)
        if key not in self.observations:
            self.observations[key] = deque(maxlen=20)  # 保留最近20次观测

        self.observations[key].append((occupancy, timestamp))

    def detect_change(self, gx, gy):
        """检测栅格 (gx, gy) 是否发生变化。

        使用滑动窗口对数似然比:
            将观测分为前后两段，计算两段的均值差异。

        Parameters
        ----------
        gx, gy : int
            栅格坐标。

        Returns
        -------
        changed : bool
            是否检测到变化。
        log_likelihood_ratio : float
            对数似然比。
        """
        key = (gx, gy)
        obs = self.observations.get(key, [])

        if len(obs) < self.min_obs:
            return False, 0.0

        # 分为前后两段 (deque 不支持切片，转为 list)
        obs_list = list(obs)
        n = len(obs_list)
        mid = n // 2
        old_obs = [o[0] for o in obs_list[:mid]]
        new_obs = [o[0] for o in obs_list[mid:]]

        if not old_obs or not new_obs:
            return False, 0.0

        # 计算均值
        p_old = np.mean(old_obs)
        p_new = np.mean(new_obs)

        # 避免极端值
        eps = 0.01
        p_old = max(eps, min(1 - eps, p_old))
        p_new = max(eps, min(1 - eps, p_new))

        # 对数似然比
        llr = 0.0
        for z in new_obs:
            llr += z * math.log(p_new / p_old) + \
                   (1 - z) * math.log((1 - p_new) / (1 - p_old))

        changed = llr > self.threshold

        if changed:
            self.total_detections += 1
            self.changes_detected.append({
                'cell': (gx, gy),
                'llr': llr,
                'p_old': p_old,
                'p_new': p_new,
                'timestamp': time.time(),
            })

        return changed, llr

    def scan_for_changes(self, occupancy_grid):
        """扫描整个地图，检测所有变化区域。

        Parameters
        ----------
        occupancy_grid : OccupancyGrid
            当前占据栅格地图。

        Returns
        -------
        changed_cells : list of (gx, gy, change_type)
            变化的栅格列表，change_type 为 'appeared'/'disappeared'。
        """
        changed_cells = []

        # 只检查有历史观测的格子
        for (gx, gy), obs_list in self.observations.items():
            if len(obs_list) < self.min_obs:
                continue

            changed, llr = self.detect_change(gx, gy)
            if not changed:
                continue

            # 判断变化类型
            old_p = np.mean([o[0] for o in list(obs_list)[:len(obs_list)//2]])
            new_p = np.mean([o[0] for o in list(obs_list)[len(obs_list)//2:]])

            if old_p < 0.3 and new_p > 0.5:
                change_type = 'appeared'    # 新障碍物出现
            elif old_p > 0.5 and new_p < 0.3:
                change_type = 'disappeared'  # 障碍物消失
            else:
                continue  # 噪声变化

            changed_cells.append((gx, gy, change_type))

        return changed_cells


class SubmapManager:
    """子地图管理器。

    将大地图分割为子地图，支持局部更新。

    Parameters
    ----------
    submap_size : int
        子地图边长 (栅格数)。
    max_submaps : int
        最大子地图数 (用于内存管理)。
    """

    def __init__(self, submap_size=20, max_submaps=50):
        self.submap_size = submap_size
        self.max_submaps = max_submaps

        # 子地图: (sx, sy) -> ndarray
        self.submaps = {}
        # 子地图时间戳
        self.submap_timestamps = {}
        # 子地图版本 (更新次数)
        self.submap_versions = {}

    def get_submap_key(self, gx, gy):
        """获取栅格所属子地图的键。"""
        return (gx // self.submap_size, gy // self.submap_size)

    def get_or_create_submap(self, sx, sy):
        """获取或创建子地图。"""
        key = (sx, sy)
        if key not in self.submaps:
            self.submaps[key] = np.zeros((self.submap_size, self.submap_size))
            self.submap_timestamps[key] = time.time()
            self.submap_versions[key] = 0

            # 内存管理: 超过最大数量时删除最旧的
            if len(self.submaps) > self.max_submaps:
                oldest = min(self.submap_timestamps, key=self.submap_timestamps.get)
                del self.submaps[oldest]
                del self.submap_timestamps[oldest]
                del self.submap_versions[oldest]

        return self.submaps[key]

    def update_cell(self, gx, gy, value):
        """更新单个栅格 (在对应子地图中)。"""
        sx = gx // self.submap_size
        sy = gy // self.submap_size
        lx = gx % self.submap_size
        ly = gy % self.submap_size

        submap = self.get_or_create_submap(sx, sy)
        submap[lx, ly] = value
        self.submap_timestamps[(sx, sy)] = time.time()
        self.submap_versions[(sx, sy)] += 1

    def get_cell(self, gx, gy):
        """获取栅格值。"""
        sx = gx // self.submap_size
        sy = gy // self.submap_size
        key = (sx, sy)
        if key not in self.submaps:
            return 0.0  # 未知
        lx = gx % self.submap_size
        ly = gy % self.submap_size
        return self.submaps[key][lx, ly]

    def replace_submap(self, sx, sy, new_data):
        """整体替换子地图 (用于大幅变化)。"""
        key = (sx, sy)
        self.submaps[key] = new_data.copy()
        self.submap_timestamps[key] = time.time()
        self.submap_versions[key] += 1


class LifelongSLAM:
    """持续建图系统。

    集成变化检测、子地图管理、增量更新，
    实现长期运行中的地图自动维护。

    Parameters
    ----------
    occ_grid : OccupancyGrid
        原始占据栅格地图。
    forgetting_factor : float
        遗忘因子 α ∈ (0, 1]。α=1 完全保留历史。
    """

    def __init__(self, occ_grid, forgetting_factor=0.3):
        self.occ_grid = occ_grid
        self.alpha = forgetting_factor

        # 变化检测器
        self.change_detector = ChangeDetector(threshold=2.5, min_observations=5)

        # 子地图管理器
        self.submap_manager = SubmapManager(submap_size=20, max_submaps=50)

        # 历史占据地图快照 (用于变化检测)
        self.history_grid = np.zeros((GRID_W, GRID_H))

        # 运行统计
        self.total_updates = 0
        self.changes_applied = 0
        self.last_change_scan = 0
        self.last_scan_time = 0.0

        # 变化日志
        self.change_log = []

    def update(self, angles, distances, robot_x, robot_y, robot_yaw,
               timestamp=None):
        """持续建图更新。

        Parameters
        ----------
        angles : ndarray
            LiDAR 角度数组。
        distances : ndarray
            LiDAR 距离数组。
        robot_x, robot_y : float
            机器人位姿。
        robot_yaw : float
            机器人朝向。
        timestamp : float or None
            时间戳。

        Returns
        -------
        changed_cells : list
            本次更新检测到的变化栅格。
        """
        if timestamp is None:
            timestamp = time.time()

        self.total_updates += 1

        # 1. 记录观测到变化检测器
        for i, (angle, dist) in enumerate(zip(angles, distances)):
            if dist >= 7.9:  # 无效距离
                continue

            # 射线终点 (障碍物)
            wx = robot_x + dist * math.cos(angle + robot_yaw)
            wy = robot_y + dist * math.sin(angle + robot_yaw)
            gx = int((wx - self.occ_grid.origin_x) / GRID_RESOLUTION)
            gy = int((wy - self.occ_grid.origin_y) / GRID_RESOLUTION)

            if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                self.change_detector.observe(gx, gy, 1.0, timestamp)

            # 射线上的空闲格
            n_ray = max(1, int(dist / GRID_RESOLUTION))
            for j in range(n_ray):
                t = j / n_ray
                rx = robot_x + t * dist * math.cos(angle + robot_yaw)
                ry = robot_y + t * dist * math.sin(angle + robot_yaw)
                rgx = int((rx - self.occ_grid.origin_x) / GRID_RESOLUTION)
                rgy = int((ry - self.occ_grid.origin_y) / GRID_RESOLUTION)
                if 0 <= rgx < GRID_W and 0 <= rgy < GRID_H:
                    self.change_detector.observe(rgx, rgy, 0.0, timestamp)

        # 2. 周期性扫描变化 (每100帧扫描一次)
        if self.total_updates - self.last_change_scan >= 100:
            self.last_change_scan = self.total_updates
            t0 = time.time()

            changed_cells = self.change_detector.scan_for_changes(self.occ_grid)
            self.last_scan_time = time.time() - t0

            # 3. 应用变化到子地图
            for gx, gy, change_type in changed_cells:
                old_val = self.submap_manager.get_cell(gx, gy)
                if change_type == 'appeared':
                    new_val = 1.0
                else:  # disappeared
                    new_val = 0.0

                # 增量更新 (遗忘因子加权)
                updated = self.alpha * new_val + (1 - self.alpha) * old_val
                self.submap_manager.update_cell(gx, gy, updated)

                self.changes_applied += 1
                self.change_log.append({
                    'frame': self.total_updates,
                    'cell': (gx, gy),
                    'type': change_type,
                    'old': old_val,
                    'new': new_val,
                    'updated': updated,
                    'timestamp': timestamp,
                })

            return changed_cells

        return []

    def get_local_map(self, robot_x, robot_y, radius_m=5.0):
        """获取机器人周围的局部地图 (从子地图)。"""
        rgx = int((robot_x - self.occ_grid.origin_x) / GRID_RESOLUTION)
        rgy = int((robot_y - self.occ_grid.origin_y) / GRID_RESOLUTION)
        r_cells = int(radius_m / GRID_RESOLUTION)

        local = np.zeros((2 * r_cells, 2 * r_cells))
        for dx in range(-r_cells, r_cells):
            for dy in range(-r_cells, r_cells):
                gx = rgx + dx
                gy = rgy + dy
                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                    local[dx + r_cells, dy + r_cells] = \
                        self.submap_manager.get_cell(gx, gy)

        return local

    def get_statistics(self):
        """获取统计信息。"""
        return {
            'total_updates': self.total_updates,
            'changes_applied': self.changes_applied,
            'total_detections': self.change_detector.total_detections,
            'submaps_loaded': len(self.submap_manager.submaps),
            'last_scan_time': self.last_scan_time,
            'change_log_size': len(self.change_log),
        }

    def export_change_report(self):
        """导出变化报告 (用于分析)。"""
        if not self.change_log:
            return "无变化记录"

        appeared = sum(1 for c in self.change_log if c['type'] == 'appeared')
        disappeared = sum(1 for c in self.change_log if c['type'] == 'disappeared')

        report = f"""
Lifelong SLAM 变化报告
========================
总更新次数: {self.total_updates}
应用变化数: {self.changes_applied}
检测总数: {self.change_detector.total_detections}
  - 新增障碍物: {appeared}
  - 障碍物消失: {disappeared}
子地图数: {len(self.submap_manager.submaps)}
"""
        return report
