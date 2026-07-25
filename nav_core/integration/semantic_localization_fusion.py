"""语义SLAM与定位融合 (v6.11)

将语义物体地标融合到AMCL定位中，通过物体观测修正定位协方差。

方法：
  1. 维护物体级语义地标地图
  2. 当相机检测到物体时，匹配到地图中的地标
  3. 用地标观测修正AMCL粒子权重
  4. 降低定位不确定性
"""
import os
import math
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass

USE_SEMANTIC_SLAM = os.environ.get("USE_SEMANTIC_SLAM", "0") == "1"


@dataclass
class SemanticObservation:
    """语义观测"""
    label: str          # 物体标签
    obs_x: float        # 观测到的物体位置x(机器人坐标系)
    obs_y: float
    distance: float     # 到物体的距离
    bearing: float       # 方位角
    confidence: float = 0.5  # 检测置信度


class SemanticLocalizationFusion:
    """语义SLAM定位融合器 (v6.11)

    用语义物体地标修正AMCL定位。
    """

    def __init__(self, occ_grid, config=None, semantic_slam=None, amcl=None):
        self.occ_grid = occ_grid
        cfg = config or {}
        self.amcl = amcl

        # 延迟导入
        if semantic_slam is not None:
            self.semantic_slam = semantic_slam
        else:
            try:
                from semantic_slam import SemanticSLAM
                self.semantic_slam = SemanticSLAM(occ_grid, cfg=cfg)
            except ImportError:
                self.semantic_slam = None

        # 参数
        self.fusion_interval = cfg.get('fusion_interval', 10)  # 融合间隔帧
        self.observation_threshold = cfg.get('observation_threshold', 0.3)
        self.position_correction_weight = cfg.get('correction_weight', 0.3)

        # 状态
        self._last_fusion_frame = 0
        self._total_corrections = 0

    def update(self, rx, ry, ryaw, semantic_observations, frame):
        """执行语义定位融合

        Args:
            rx, ry, ryaw: 当前AMCL估计
            semantic_observations: 语义观测列表
            frame: 当前帧

        Returns:
            (corrected_x, corrected_y, corrected_yaw, correction_applied)
        """
        if (not USE_SEMANTIC_SLAM or self.semantic_slam is None or
                self.amcl is None or not semantic_observations):
            return rx, ry, ryaw, False

        if (frame - self._last_fusion_frame) < self.fusion_interval:
            return rx, ry, ryaw, False

        self._last_fusion_frame = frame

        # 匹配观测到地图地标
        corrections = []
        for obs in semantic_observations:
            match = self._match_landmark(obs, rx, ry, ryaw)
            if match is not None:
                corrections.append(match)

        if not corrections:
            return rx, ry, ryaw, False

        # 计算加权平均修正
        total_weight = sum(c[2] for c in corrections)
        if total_weight < 1e-6:
            return rx, ry, ryaw, False

        dx = sum(c[0] * c[2] for c in corrections) / total_weight
        dy = sum(c[1] * c[2] for c in corrections) / total_weight

        # 部分修正避免跳变
        correction = self.position_correction_weight
        corrected_x = rx + dx * correction
        corrected_y = ry + dy * correction

        # 应用到AMCL（如果粒子云支持）
        if hasattr(self.amcl, 'apply_correction'):
            self.amcl.apply_correction(dx * correction, dy * correction)

        self._total_corrections += 1
        print(f"[SEM-FUSION] 应用语义修正: dx={dx*correction:.3f} "
              f"dy={dy*correction:.3f} ({len(corrections)}个地标匹配)")

        return corrected_x, corrected_y, ryaw, True

    def _match_landmark(self, obs: SemanticObservation, rx, ry, ryaw):
        """匹配观测到地图中的地标

        Returns:
            (dx, dy, weight) or None: 修正向量和权重
        """
        if self.semantic_slam is None:
            return None

        # 计算观测到的物体在世界坐标系中的位置
        world_x = rx + obs.distance * math.cos(ryaw + obs.bearing)
        world_y = ry + obs.distance * math.sin(ryaw + obs.bearing)

        # 在语义地图中查找最近的同类地标
        nearest = None
        min_dist = float('inf')

        if hasattr(self.semantic_slam, 'landmarks'):
            for landmark in self.semantic_slam.landmarks.values():
                if landmark.label_name != obs.label:
                    continue
                dist = math.sqrt((landmark.x - world_x)**2 + (landmark.y - world_y)**2)
                if dist < min_dist and dist < 1.0:  # 1米内匹配
                    min_dist = dist
                    nearest = landmark

        if nearest is None:
            return None

        # 计算修正向量
        expected_x = nearest.x
        expected_y = nearest.y
        dx = expected_x - world_x
        dy = expected_y - world_y

        # 权重 = 检测置信度 * (1 - 距离/阈值)
        weight = obs.confidence * max(0, 1.0 - min_dist)

        return (dx, dy, weight)

    def get_status(self):
        return {
            'enabled': USE_SEMANTIC_SLAM and self.semantic_slam is not None,
            'total_corrections': self._total_corrections,
            'landmarks': len(self.semantic_slam.landmarks) if self.semantic_slam else 0,
        }
