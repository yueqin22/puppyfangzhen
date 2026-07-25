"""狭窄通道模式 (v5.8)

检测机器人是否处于狭窄通道（如门口、走廊），
并自适应调整局部规划器参数以提高通过性。

检测方法：
  1. LiDAR左右两侧最小距离 < 阈值
  2. Costmap走廊宽度 < 机器人直径
  3. 连续帧确认（避免瞬时噪声）

参数调整：
  - 降低速度上限
  - 提高clearance权重
  - 缩小采样窗口
  - 启用特殊路径跟踪模式
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from collections import deque

from costmap import Costmap, COST_LETHAL, COST_INSCRIBED


@dataclass
class NarrowPassageParams:
    """狭窄通道模式参数"""
    # 检测参数
    narrow_threshold: float = 0.8      # 两侧距离小于此值判定为窄通道(米)
    narrow_min_frames: int = 5         # 连续帧确认
    corridor_width: float = 0.7        # 走廊宽度阈值(米)

    # 参数调整
    narrow_max_v: float = 0.15         # 窄通道最大线速度
    narrow_max_w: float = 0.8          # 窄通道最大角速度
    narrow_w_clearance: float = 0.30   # 窄通道clearance权重提高
    narrow_w_path: float = 0.25         # 路径跟踪权重
    narrow_goal_tolerance: float = 0.25  # 目标容差缩小
    narrow_inflation: float = 0.15      # 膨胀半径减小


class NarrowPassageDetector:
    """狭窄通道检测器 (v5.8)

    通过LiDAR数据和costmap分析检测狭窄通道，
    并在进入/退出时触发参数切换。
    """

    def __init__(self, costmap, config=None):
        self.costmap = costmap
        cfg = config or {}
        self.params = NarrowPassageParams(
            narrow_threshold=cfg.get('narrow_threshold', 0.8),
            narrow_min_frames=cfg.get('narrow_min_frames', 5),
            corridor_width=cfg.get('corridor_width', 0.7),
            narrow_max_v=cfg.get('narrow_max_v', 0.15),
            narrow_max_w=cfg.get('narrow_max_w', 0.8),
            narrow_w_clearance=cfg.get('narrow_w_clearance', 0.30),
            narrow_w_path=cfg.get('narrow_w_path', 0.25),
            narrow_goal_tolerance=cfg.get('narrow_goal_tolerance', 0.25),
            narrow_inflation=cfg.get('narrow_inflation', 0.15),
        )

        # 状态跟踪
        self._is_narrow = False
        self._narrow_frame_count = 0
        self._normal_frame_count = 0
        self._transition_history = deque(maxlen=20)  # 记录切换历史

        # 最新的通道宽度
        self._current_corridor_width = float('inf')

    def update(self, rx, ry, ryaw, angles, distances, frame=0):
        """每帧更新检测状态

        Args:
            rx, ry, ryaw: 机器人位置和朝向
            angles: LiDAR角度数组(世界坐标系)
            distances: LiDAR距离数组
            frame: 当前帧号

        Returns:
            is_narrow: bool — 是否处于狭窄通道
        """
        # 计算左右两侧最近障碍距离
        left_dist = self._min_lateral_distance(angles, distances, ryaw, side='left')
        right_dist = self._min_lateral_distance(angles, distances, ryaw, side='right')

        # 通道宽度 = 左 + 右
        corridor_width = left_dist + right_dist
        self._current_corridor_width = corridor_width

        # 检测条件：两侧都近 或 通道宽度小于阈值
        is_narrow_raw = (corridor_width < self.params.corridor_width or
                         (left_dist < self.params.narrow_threshold and
                          right_dist < self.params.narrow_threshold))

        # 连续帧确认（去抖动）
        if is_narrow_raw:
            self._narrow_frame_count += 1
            self._normal_frame_count = 0
        else:
            self._normal_frame_count += 1
            self._narrow_frame_count = 0

        # 状态切换
        prev_narrow = self._is_narrow
        if not self._is_narrow and self._narrow_frame_count >= self.params.narrow_min_frames:
            self._is_narrow = True
            self._transition_history.append((frame, 'enter'))
        elif self._is_narrow and self._normal_frame_count >= self.params.narrow_min_frames:
            self._is_narrow = False
            self._transition_history.append((frame, 'exit'))

        # 状态切换日志
        if prev_narrow != self._is_narrow:
            action = '进入' if self._is_narrow else '退出'
            width_str = f"通道宽度={corridor_width:.2f}m"
            print(f"[NARROW] {action}狭窄通道模式 (帧{frame}, {width_str})")

        return self._is_narrow

    def _min_lateral_distance(self, angles, distances, robot_yaw, side='left'):
        """计算机器人侧面最近障碍物距离

        Args:
            angles: LiDAR角度(世界坐标系)
            distances: LiDAR距离
            robot_yaw: 机器人朝向
            side: 'left' 或 'right'
        """
        target_angle = robot_yaw + (math.pi / 2 if side == 'left' else -math.pi / 2)
        min_dist = 10.0  # 默认远距离

        for i, world_angle in enumerate(angles):
            # 转换到机器人坐标系
            robot_angle = world_angle - robot_yaw
            while robot_angle > math.pi:
                robot_angle -= 2 * math.pi
            while robot_angle < -math.pi:
                robot_angle += 2 * math.pi

            # 左侧：角度在 [pi/4, 3pi/4] 范围
            # 右侧：角度在 [-3pi/4, -pi/4] 范围
            if side == 'left':
                if math.pi / 4 < robot_angle < 3 * math.pi / 4:
                    if distances[i] < min_dist:
                        min_dist = distances[i]
            else:
                if -3 * math.pi / 4 < robot_angle < -math.pi / 4:
                    if distances[i] < min_dist:
                        min_dist = distances[i]

        return min_dist

    def get_adjusted_params(self):
        """获取狭窄通道模式下的调整参数

        Returns:
            dict: 调整后的参数（如果不在窄通道，返回空dict）
        """
        if not self._is_narrow:
            return {}
        return {
            'max_v': self.params.narrow_max_v,
            'max_w': self.params.narrow_max_w,
            'w_clearance': self.params.narrow_w_clearance,
            'w_path': self.params.narrow_w_path,
            'goal_tolerance': self.params.narrow_goal_tolerance,
            'inflation_radius': self.params.narrow_inflation,
        }

    @property
    def is_narrow(self):
        return self._is_narrow

    @property
    def corridor_width(self):
        return self._current_corridor_width

    def get_transition_history(self):
        """获取进入/退出历史记录"""
        return list(self._transition_history)
