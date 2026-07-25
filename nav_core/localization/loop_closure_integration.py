"""回环检测集成 (v6.6)

将回环检测和位姿图优化集成到主SLAM流程中。
当检测到回环时，使用位姿图修正来纠正AMCL粒子云的累积漂移。

工作流程：
  1. 定期添加关键帧到PoseGraph
  2. 检测回环（基于位置距离 + 扫描匹配）
  3. 检测到回环后，运行Gauss-Newton优化
  4. 计算位姿修正量，应用到AMCL粒子云
"""
import os
import math
import time
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

USE_LOOP_CLOSURE = os.environ.get("USE_LOOP_CLOSURE", "0") == "1"


@dataclass
class LoopClosureResult:
    """回环检测结果"""
    detected: bool = False
    correction_x: float = 0.0
    correction_y: float = 0.0
    correction_yaw: float = 0.0
    confidence: float = 0.0
    keyframe_from: int = -1
    keyframe_to: int = -1


class LoopClosureIntegrator:
    """回环检测集成器 (v6.6)

    将loop_closure.py的PoseGraph集成到导航主循环中。
    """

    def __init__(self, occ_grid, config=None, pose_graph=None):
        self.occ_grid = occ_grid
        cfg = config or {}

        # 延迟导入，避免循环依赖
        if pose_graph is not None:
            self.pose_graph = pose_graph
        else:
            try:
                from loop_closure import PoseGraph
                self.pose_graph = PoseGraph()
            except ImportError:
                self.pose_graph = None

        # 参数
        self.keyframe_interval = cfg.get('keyframe_interval', 50)  # 每50帧添加关键帧
        self.loop_distance_threshold = cfg.get('loop_distance_threshold', 1.5)  # 回环距离阈值
        self.min_keyframes_for_loop = cfg.get('min_keyframes_for_loop', 10)  # 最少关键帧数
        self.optimization_interval = cfg.get('optimization_interval', 200)  # 优化间隔

        # 状态
        self._last_keyframe_frame = 0
        self._last_optimization_frame = 0
        self._total_loops = 0
        self._last_result = LoopClosureResult()

        # 累积修正量
        self._cumulative_correction = (0.0, 0.0, 0.0)

    def should_add_keyframe(self, frame):
        """检查是否应该添加关键帧"""
        return (frame - self._last_keyframe_frame) >= self.keyframe_interval

    def add_keyframe(self, x, y, yaw, scan, frame):
        """添加关键帧到位姿图

        Args:
            x, y, yaw: 机器人位姿
            scan: (angles, distances) LiDAR扫描数据
            frame: 当前帧
        """
        if self.pose_graph is None:
            return

        self.pose_graph.add_keyframe(x, y, yaw, scan)
        self._last_keyframe_frame = frame

    def detect_loop_closure(self, current_x, current_y, current_yaw, frame):
        """检测回环

        通过比较当前位置与历史关键帧的距离来检测回环候选。
        如果距离小于阈值且不是相邻关键帧，则认为是回环。

        Args:
            current_x, current_y: 当前机器人位置
            current_yaw: 当前机器人朝向
            frame: 当前帧

        Returns:
            LoopClosureResult: 检测结果
        """
        if self.pose_graph is None or len(self.pose_graph.keyframes) < self.min_keyframes_for_loop:
            return LoopClosureResult()

        # 查找最近的历史关键帧（排除最近的几个）
        best_match = None
        best_dist = float('inf')
        recent_skip = 5  # 跳过最近5个关键帧

        for i, kf in enumerate(self.pose_graph.keyframes[:-recent_skip]):
            dist = math.sqrt((kf.x - current_x)**2 + (kf.y - current_y)**2)
            if dist < self.loop_distance_threshold and dist < best_dist:
                best_dist = dist
                best_match = kf

        if best_match is None:
            self._last_result = LoopClosureResult()
            return self._last_result

        # 简化的扫描匹配（实际应用ICP）
        # 这里用距离作为置信度的近似
        confidence = max(0.0, 1.0 - best_dist / self.loop_distance_threshold)

        # 计算修正量（简化：当前位置与关键帧位置的差）
        correction_x = best_match.x - current_x
        correction_y = best_match.y - current_y
        correction_yaw = best_match.theta - current_yaw

        # 归一化角度
        while correction_yaw > math.pi:
            correction_yaw -= 2 * math.pi
        while correction_yaw < -math.pi:
            correction_yaw += 2 * math.pi

        self._last_result = LoopClosureResult(
            detected=True,
            correction_x=correction_x,
            correction_y=correction_y,
            correction_yaw=correction_yaw,
            confidence=confidence,
            keyframe_from=best_match.id,
            keyframe_to=self.pose_graph.next_id - 1,
        )

        return self._last_result

    def optimize_pose_graph(self, frame):
        """运行位姿图优化

        Returns:
            bool: 是否执行了优化
        """
        if self.pose_graph is None:
            return False
        if (frame - self._last_optimization_frame) < self.optimization_interval:
            return False
        if len(self.pose_graph.keyframes) < self.min_keyframes_for_loop:
            return False

        try:
            self.pose_graph.optimize()
            self._last_optimization_frame = frame
            return True
        except Exception as e:
            print(f"[LOOP] 位姿图优化失败: {e}")
            return False

    def apply_correction_to_amcl(self, amcl, frame):
        """将回环检测修正应用到AMCL粒子云

        当检测到回环时，根据修正量调整AMCL粒子分布。

        Args:
            amcl: AMCL实例
            frame: 当前帧

        Returns:
            bool: 是否应用了修正
        """
        if not self._last_result.detected:
            return False

        # 获取当前AMCL估计
        est_x, est_y, est_yaw, _ = amcl.get_estimate()

        # 计算修正后的位置
        corrected_x = est_x + self._last_result.correction_x * 0.5  # 部分修正避免跳变
        corrected_y = est_y + self._last_result.correction_y * 0.5
        corrected_yaw = est_yaw + self._last_result.correction_yaw * 0.5

        # 如果修正量较大，重新散布粒子
        correction_magnitude = math.sqrt(
            self._last_result.correction_x**2 + self._last_result.correction_y**2)

        if correction_magnitude > 0.3:
            spread = min(1.0, correction_magnitude)
            amcl.recover(corrected_x, corrected_y, corrected_yaw, spread=spread)
            print(f"[LOOP] 回环修正应用: dx={self._last_result.correction_x:.2f} "
                  f"dy={self._last_result.correction_y:.2f} spread={spread:.2f}")
            self._total_loops += 1
            return True

        return False

    @property
    def total_loops(self):
        return self._total_loops

    def get_status(self):
        """获取状态摘要"""
        return {
            'keyframes': len(self.pose_graph.keyframes) if self.pose_graph else 0,
            'total_loops': self._total_loops,
            'last_correction': (self._last_result.correction_x,
                               self._last_result.correction_y,
                               self._last_result.correction_yaw) if self._last_result.detected else None,
        }
