"""Active SLAM端到端集成 (v6.8)

将Active SLAM的信息增益驱动探索集成到导航主循环中。
Active SLAM不仅探索新空间，还主动选择能同时降低定位和地图不确定性的动作。

集成点：
  1. 在PLAN状态，用NBV替换或补充frontier选择
  2. 在FOLLOW状态，评估当前目标的信息增益
  3. 用信息增益调整frontier评分权重
"""
import os
import math
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass

USE_ACTIVE_SLAM = os.environ.get("USE_ACTIVE_SLAM", "0") == "1"


@dataclass
class ActiveSLAMDecision:
    """Active SLAM决策结果"""
    use_nbv: bool = False          # 是否使用NBV替代frontier
    nbv_goal: Optional[Tuple[float, float]] = None  # NBV目标点
    info_gain_boost: float = 0.0   # 信息增益加成(调整frontier评分)
    uncertainty_reduction: float = 0.0  # 预期不确定性降低
    reason: str = ""


class ActiveSLAMIntegrator:
    """Active SLAM集成器 (v6.8)

    在frontier探索框架中注入Active SLAM的决策能力。
    """

    def __init__(self, occ_grid, costmap, config=None, active_estimator=None):
        self.occ_grid = occ_grid
        self.costmap = costmap
        cfg = config or {}

        # 延迟导入ActiveSLAMEstimator
        if active_estimator is not None:
            self.estimator = active_estimator
        else:
            try:
                from active_slam import ActiveSLAMEstimator
                self.estimator = ActiveSLAMEstimator(occ_grid, costmap, cfg=cfg)
            except ImportError:
                self.estimator = None

        # 参数
        self.nbv_interval = cfg.get('nbv_interval', 100)  # NBV计算间隔
        self.nbv_candidates = cfg.get('nbv_candidates', 10)  # NBV候选数
        self.loc_uncertainty_threshold = cfg.get('loc_uncertainty_threshold', 0.5)
        self.info_gain_weight = cfg.get('info_gain_weight', 0.3)

        # 状态
        self._last_nbv_frame = 0
        self._current_nbv = None
        self._loc_uncertainty = 0.0

    def update_localization_uncertainty(self, covariance_trace):
        """更新定位不确定性估计"""
        self._loc_uncertainty = covariance_trace
        if self.estimator:
            self.estimator.loc_covariance = np.eye(3) * max(0.01, covariance_trace / 3)

    def should_compute_nbv(self, frame):
        """检查是否应该计算NBV"""
        if not USE_ACTIVE_SLAM or self.estimator is None:
            return False
        # 定位不确定性高时更频繁计算NBV
        interval = self.nbv_interval
        if self._loc_uncertainty > self.loc_uncertainty_threshold:
            interval = max(20, interval // 3)  # 不确定性高时3倍频率
        return (frame - self._last_nbv_frame) >= interval

    def compute_nbv(self, rx, ry, ryaw, frontiers, frame):
        """计算下一波最佳视角(NBV)

        在frontier候选中，结合信息增益和定位不确定性选择最佳观测点。

        Args:
            rx, ry, ryaw: 机器人当前位姿
            frontiers: frontier候选列表 [(fx, fy, size), ...]
            frame: 当前帧

        Returns:
            ActiveSLAMDecision: 决策结果
        """
        if not frontiers or self.estimator is None:
            return ActiveSLAMDecision(reason="无候选或estimator未初始化")

        self._last_nbv_frame = frame

        # 如果定位不确定性高，优先选择能降低不确定性的位置
        if self._loc_uncertainty > self.loc_uncertainty_threshold:
            # 找到信息增益最高的frontier
            best_frontier = None
            best_info_gain = -float('inf')

            for fx, fy, size in frontiers[:self.nbv_candidates]:
                info = self.estimator.compute_info_gain(fx, fy, ryaw)
                # 信息增益 = 几何 + 定位 + 语义
                total_gain = (self.estimator.w_geo * info.get('geometric', 0) +
                             self.estimator.w_loc * info.get('localization', 0) +
                             self.estimator.w_sem * info.get('semantic', 0))
                if total_gain > best_info_gain:
                    best_info_gain = total_gain
                    best_frontier = (fx, fy)

            if best_frontier:
                self._current_nbv = best_frontier
                return ActiveSLAMDecision(
                    use_nbv=True,
                    nbv_goal=best_frontier,
                    info_gain_boost=best_info_gain,
                    uncertainty_reduction=self._loc_uncertainty * 0.5,
                    reason=f"定位不确定性={self._loc_uncertainty:.3f}，"
                           f"选择信息增益最高的frontier",
                )

        # 正常情况：用信息增益调整frontier评分权重
        return ActiveSLAMDecision(
            use_nbv=False,
            info_gain_boost=self.info_gain_weight,
            reason="信息增益权重调整",
        )

    def adjust_frontier_score(self, fx, fy, base_score, robot_yaw, frame):
        """调整frontier评分，加入Active SLAM信息增益

        Args:
            fx, fy: frontier位置
            base_score: 原始评分
            robot_yaw: 机器人当前朝向（用于信息增益计算）
            frame: 当前帧

        Returns:
            float: 调整后的评分
        """
        if not self.enabled or self.estimator is None:
            return base_score
        try:
            info = self.estimator.compute_info_gain(fx, fy, robot_yaw)
            total_gain = (self.estimator.w_geo * info.get('geometric', 0) +
                         self.estimator.w_loc * info.get('localization', 0) +
                         self.estimator.w_sem * info.get('semantic', 0))
            return base_score + self.info_gain_weight * total_gain
        except Exception:
            return base_score

    def get_status(self):
        """获取状态摘要"""
        return {
            'enabled': USE_ACTIVE_SLAM and self.estimator is not None,
            'loc_uncertainty': self._loc_uncertainty,
            'current_nbv': self._current_nbv,
            'last_nbv_frame': self._last_nbv_frame,
        }
