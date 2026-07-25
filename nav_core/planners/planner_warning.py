"""局部规划器协同预警 (v6.3)

监控局部规划器的健康状态，在规划失败导致RECOVER之前提前预警。

预警条件：
  1. 规划器连续输出零速度
  2. 规划器计算时间异常增长
  3. 速度震荡（频繁反向）
  4. 路径跟踪偏差持续增大

预警动作：
  - 通知主循环提前进入RECOVER
  - 建议切换规划器
  - 调整规划器参数
"""
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Callable


@dataclass
class WarningSignal:
    """预警信号"""
    level: str = 'none'  # 'none', 'low', 'medium', 'high'
    reason: str = ""
    suggestion: str = ""  # 'recover', 'switch_planner', 'adjust_params'
    frame: int = 0


class PlannerHealthMonitor:
    """局部规划器健康监控器 (v6.3)

    通过分析规划器的输出模式，提前发现潜在问题。
    """

    def __init__(self, config=None):
        cfg = config or {}

        # 预警阈值
        self.zero_vel_threshold = cfg.get('zero_vel_threshold', 5)  # 连续零速度帧
        self.oscillation_window = cfg.get('oscillation_window', 20)  # 震荡检测窗口
        self.oscillation_threshold = cfg.get('oscillation_threshold', 0.5)  # 方向反转频率
        self.time_spike_threshold = cfg.get('time_spike_ms', 50.0)  # 计算时间突增阈值
        self.deviation_threshold = cfg.get('deviation_threshold', 0.5)  # 路径偏差阈值

        # 历史数据
        self._velocity_history = deque(maxlen=self.oscillation_window)
        self._zero_vel_count = 0
        self._last_vel_sign = 0  # 上一帧速度方向
        self._direction_changes = 0
        self._compute_times = deque(maxlen=50)
        self._path_deviations = deque(maxlen=50)

        # 当前预警状态
        self._current_warning = WarningSignal()

        # 预警回调
        self._callback = None

    def update(self, v, w, compute_time_ms, path_deviation, frame):
        """每帧更新监控状态

        Args:
            v, w: 规划器输出的线速度和角速度
            compute_time_ms: 本次规划耗时(毫秒)
            path_deviation: 机器人到路径的偏差(米)
            frame: 当前帧号

        Returns:
            WarningSignal: 当前预警状态
        """
        # 1. 零速度检测
        is_zero = abs(v) < 1e-6 and abs(w) < 1e-6
        if is_zero:
            self._zero_vel_count += 1
        else:
            self._zero_vel_count = 0

        # 2. 速度震荡检测
        self._velocity_history.append((v, w))
        current_sign = 1 if v > 0 else (-1 if v < 0 else 0)
        if current_sign != 0 and self._last_vel_sign != 0 and current_sign != self._last_vel_sign:
            self._direction_changes += 1
        if current_sign != 0:
            self._last_vel_sign = current_sign

        # 方向反转频率
        history_len = len(self._velocity_history)
        if history_len > 0:
            reversal_rate = self._direction_changes / history_len
        else:
            reversal_rate = 0.0

        # 3. 计算时间突增
        self._compute_times.append(compute_time_ms)
        avg_time = sum(self._compute_times) / len(self._compute_times) if self._compute_times else 0
        time_spike = compute_time_ms > self.time_spike_threshold and compute_time_ms > avg_time * 2

        # 4. 路径偏差
        self._path_deviations.append(path_deviation)
        avg_deviation = (sum(self._path_deviations) / len(self._path_deviations)
                         if self._path_deviations else 0)
        high_deviation = avg_deviation > self.deviation_threshold

        # 综合判断预警级别
        if self._zero_vel_count >= self.zero_vel_threshold:
            self._current_warning = WarningSignal(
                level='high',
                reason=f"连续{self._zero_vel_count}帧零速度输出",
                suggestion='recover',
                frame=frame,
            )
        elif reversal_rate > self.oscillation_threshold:
            self._current_warning = WarningSignal(
                level='high',
                reason=f"速度震荡: 反转率{reversal_rate:.2f}",
                suggestion='switch_planner',
                frame=frame,
            )
        elif time_spike:
            self._current_warning = WarningSignal(
                level='medium',
                reason=f"计算时间突增: {compute_time_ms:.1f}ms (平均{avg_time:.1f}ms)",
                suggestion='adjust_params',
                frame=frame,
            )
        elif high_deviation:
            self._current_warning = WarningSignal(
                level='medium',
                reason=f"路径偏差持续偏高: {avg_deviation:.2f}m",
                suggestion='recover',
                frame=frame,
            )
        else:
            self._current_warning = WarningSignal(level='none', frame=frame)

        # 触发回调
        if self._current_warning.level in ('medium', 'high') and self._callback:
            self._callback(self._current_warning)

        return self._current_warning

    def set_callback(self, callback: Callable):
        """设置预警回调"""
        self._callback = callback

    def get_warning(self) -> WarningSignal:
        """获取当前预警状态"""
        return self._current_warning

    def reset(self):
        """重置监控状态"""
        self._velocity_history.clear()
        self._zero_vel_count = 0
        self._direction_changes = 0
        self._compute_times.clear()
        self._path_deviations.clear()
        self._current_warning = WarningSignal()
