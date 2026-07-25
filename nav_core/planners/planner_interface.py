"""统一局部规划器接口 (v6.4)

提供DWA/TEB/MPC三种局部规划器的统一抽象层，支持运行时动态切换。
当某个规划器连续失败时，自动切换到备选规划器。

功能：
  - PlannerInterface: 抽象基类定义统一接口
  - PlannerManager: 管理多规划器，支持动态切换和失败预警(v6.3)
  - PlannerType: 枚举标识规划器类型
"""
import os
import math
import time
from enum import IntEnum
from typing import Optional, Tuple, List
from dataclasses import dataclass, field

from costmap import Costmap, COST_LETHAL, COST_INSCRIBED


class PlannerType(IntEnum):
    """局部规划器类型"""
    DWA = 0
    TEB = 1
    MPC = 2


@dataclass
class PlannerStatus:
    """规划器运行状态"""
    planner_type: PlannerType = PlannerType.DWA
    consecutive_failures: int = 0
    total_calls: int = 0
    total_failures: int = 0
    avg_compute_time_ms: float = 0.0
    last_velocity: Tuple[float, float] = (0.0, 0.0)  # (v, w)
    warning: str = ""  # v6.3 预警消息


class PlannerInterface:
    """局部规划器统一接口 (v6.4)

    所有具体规划器(DWA/TEB/MPC)通过此接口被统一调用。
    """

    def compute_velocity(self, rx, ry, ryaw, path, goal, obstacles=None):
        """计算最优速度命令

        Args:
            rx, ry, ryaw: 机器人当前位置和朝向
            path: 全局路径 waypoint列表 [(x,y), ...]
            goal: 目标点 (x, y)
            obstacles: 可选的动态障碍物列表

        Returns:
            (v, w): 线速度和角速度
        """
        raise NotImplementedError

    def reset(self):
        """重置规划器状态"""
        pass

    def get_type(self) -> PlannerType:
        """返回规划器类型"""
        raise NotImplementedError


class PlannerManager:
    """多规划器管理器 (v6.4 + v6.3)

    管理多个局部规划器实例，支持：
      - 运行时动态切换规划器
      - v6.3: 失败预警和自动切换
      - 性能统计和健康监控

    切换策略：
      - 默认使用DWA（最稳定）
      - USE_TEB=1 时优先使用TEB
      - USE_MPC=1 时优先使用MPC
      - 连续3次失败自动降级到DWA
    """

    def __init__(self, costmap, config=None, dwa=None, teb=None, mpc=None):
        self.costmap = costmap
        cfg = config or {}

        # 存储规划器实例（可能为None表示未启用）
        self._planners = {}
        if dwa is not None:
            self._planners[PlannerType.DWA] = dwa
        if teb is not None:
            self._planners[PlannerType.TEB] = teb
        if mpc is not None:
            self._planners[PlannerType.MPC] = mpc

        # 默认规划器
        default_type_str = cfg.get('default_planner', 'dwa').lower()
        type_map = {'dwa': PlannerType.DWA, 'teb': PlannerType.TEB, 'mpc': PlannerType.MPC}
        self._default_type = type_map.get(default_type_str, PlannerType.DWA)
        if self._default_type not in self._planners:
            # 回退到第一个可用的规划器
            self._default_type = list(self._planners.keys())[0] if self._planners else PlannerType.DWA

        self._current_type = self._default_type

        # v6.3: 预警参数
        self.failure_threshold = cfg.get('failure_threshold', 3)  # 连续失败阈值
        self.recovery_cooldown = cfg.get('recovery_cooldown', 50)  # 切换冷却帧
        self._last_switch_frame = -self.recovery_cooldown

        # 状态跟踪
        self.status = PlannerStatus(planner_type=self._current_type)
        self._compute_times = []  # 滚动平均计算时间
        self._max_time_history = 100

        # v6.3: 预警回调（由外部设置）
        self._warning_callback = None

    def compute_velocity(self, rx, ry, ryaw, path, goal, frame=0, obstacles=None):
        """通过当前活跃规划器计算速度

        Returns:
            (v, w): 线速度和角速度
        """
        planner = self._planners.get(self._current_type)
        if planner is None:
            # 当前规划器不可用，切换
            self._fallback_switch(frame, "当前规划器不可用")
            planner = self._planners.get(self._current_type)
            if planner is None:
                return 0.0, 0.0

        self.status.total_calls += 1
        t_start = time.time()

        try:
            v, w = planner.compute_velocity(rx, ry, ryaw, path, goal, obstacles) \
                if obstacles is not None else \
                planner.compute_velocity(rx, ry, ryaw, path, goal)
            success = self._check_velocity_validity(v, w, rx, ry, ryaw, path)
        except Exception as e:
            v, w = 0.0, 0.0
            success = False
            self.status.warning = f"规划器异常: {e}"

        # 记录计算时间
        dt_ms = (time.time() - t_start) * 1000
        self._compute_times.append(dt_ms)
        if len(self._compute_times) > self._max_time_history:
            self._compute_times.pop(0)
        self.status.avg_compute_time_ms = sum(self._compute_times) / len(self._compute_times)

        # v6.3: 失败跟踪和预警
        if not success:
            self.status.consecutive_failures += 1
            self.status.total_failures += 1
            if self.status.consecutive_failures >= self.failure_threshold:
                self._trigger_warning(frame, f"{self._current_type.name}连续失败"
                                     f"{self.status.consecutive_failures}次")
                self._fallback_switch(frame, "连续失败触发降级")
        else:
            self.status.consecutive_failures = 0
            self.status.warning = ""

        self.status.last_velocity = (v, w)
        return v, w

    def _check_velocity_validity(self, v, w, rx, ry, ryaw, path):
        """检查规划器输出的速度是否有效（非零且不撞墙）

        Args:
            v, w: 规划器输出的线速度和角速度
            rx, ry: 机器人当前位置
            ryaw: 机器人当前朝向（用于碰撞预测方向）
            path: 当前路径
        """
        if abs(v) < 1e-6 and abs(w) < 1e-6:
            # 零速度可能表示规划失败
            if path and len(path) > 0:
                return False
            return True
        # 检查预测位置是否在致命区域（沿机器人实际朝向）
        dt = 0.1
        pred_x = rx + v * dt * math.cos(ryaw)
        pred_y = ry + v * dt * math.sin(ryaw)
        cost = self.costmap.get_cost(pred_x, pred_y)
        return cost < COST_LETHAL

    def _fallback_switch(self, frame, reason):
        """v6.3: 降级切换到备选规划器"""
        if frame - self._last_switch_frame < self.recovery_cooldown:
            return  # 冷却期内不切换

        # 优先级：TEB失败→DWA，MPC失败→DWA，DWA失败→TEB
        fallback_map = {
            PlannerType.TEB: PlannerType.DWA,
            PlannerType.MPC: PlannerType.DWA,
            PlannerType.DWA: PlannerType.TEB if PlannerType.TEB in self._planners else PlannerType.DWA,
        }
        new_type = fallback_map.get(self._current_type, PlannerType.DWA)
        if new_type in self._planners and new_type != self._current_type:
            old_name = self._current_type.name
            self._current_type = new_type
            self.status.planner_type = new_type
            self.status.consecutive_failures = 0
            self._last_switch_frame = frame
            print(f"[PLANNER] 切换 {old_name}→{new_type.name}: {reason}")

    def _trigger_warning(self, frame, message):
        """v6.3: 触发预警"""
        self.status.warning = message
        if self._warning_callback:
            self._warning_callback(frame, message)

    def set_warning_callback(self, callback):
        """设置预警回调函数

        Args:
            callback: function(frame: int, message: str)
        """
        self._warning_callback = callback

    def switch_planner(self, planner_type: PlannerType):
        """手动切换规划器"""
        if planner_type in self._planners:
            self._current_type = planner_type
            self.status.planner_type = planner_type
            self.status.consecutive_failures = 0
            return True
        return False

    def get_current_planner(self):
        """获取当前活跃的规划器实例"""
        return self._planners.get(self._current_type)

    def get_type(self) -> PlannerType:
        return self._current_type

    def reset(self):
        """重置所有规划器"""
        for planner in self._planners.values():
            planner.reset()
        self.status.consecutive_failures = 0
        self.status.warning = ""

    def get_status(self) -> PlannerStatus:
        return self.status
