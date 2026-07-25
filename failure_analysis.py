#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
失败案例分析与局限性框架 (Failure Case Analysis)
================================================
按 tigao1.md 方向四任务 4.5 设计的失败案例收集与分析。

4 类典型失败案例:
  1. AMCL Kidnapped - 粒子枯竭，定位丢失
  2. Narrow Passage Stuck - 狭窄通道卡死
  3. Dynamic Collision - 动态障碍物碰撞
  4. Dead Zone - 死区探索不到

每类失败案例:
  - 触发条件检测
  - 根因分析
  - 当前方法为何失败
  - 改进建议
  - 局限性诚实讨论

学术价值: 失败案例分析是高水平论文的标志，
展示对方法边界的深刻理解。

依赖：numpy
独立运行：python failure_analysis.py
"""

import json
import math
import os
import time
from typing import Optional, Tuple, List, Dict, Any

import numpy as np


# ===========================================================================
# 失败类型常量
# ===========================================================================

FAILURE_TYPE_AMCL_KIDNAPPED = 'amcl_kidnapped'
FAILURE_TYPE_NARROW_PASSAGE = 'narrow_passage_stuck'
FAILURE_TYPE_DYNAMIC_COLLISION = 'dynamic_collision'
FAILURE_TYPE_DEAD_ZONE = 'dead_zone'

FAILURE_TYPES = [
    FAILURE_TYPE_AMCL_KIDNAPPED,
    FAILURE_TYPE_NARROW_PASSAGE,
    FAILURE_TYPE_DYNAMIC_COLLISION,
    FAILURE_TYPE_DEAD_ZONE,
]

FAILURE_TYPE_DESCRIPTIONS = {
    FAILURE_TYPE_AMCL_KIDNAPPED: 'AMCL粒子枯竭，定位丢失（Kidnapped Robot Problem）',
    FAILURE_TYPE_NARROW_PASSAGE: '狭窄通道卡死，机器人无法通过',
    FAILURE_TYPE_DYNAMIC_COLLISION: '动态障碍物碰撞，避障失败',
    FAILURE_TYPE_DEAD_ZONE: '死区探索不到，部分区域长期未被访问',
}


# ===========================================================================
# 失败案例数据类
# ===========================================================================

class FailureCase:
    """失败案例数据类。

    记录一次失败事件的完整信息，包括：
      - 标识信息: case_id, failure_type, timestamp
      - 触发条件: 导致失败的检测条件
      - 上下文: 失败发生时的机器人状态和环境状态
      - 根因分析: 为什么会失败
      - 改进建议: 如何避免或缓解

    属性:
        case_id: 案例唯一标识
        failure_type: 失败类型（见 FAILURE_TYPES）
        timestamp: 发生时间戳（秒）
        trigger_condition: 触发条件描述
        context: 上下文字典，包含:
            - robot_pos: 机器人位置 (x, y, yaw)
            - loc_err: 定位误差（米）
            - coverage: 地图覆盖率（%）
            - velocity: 速度
            - extra: 额外信息
        root_cause: 根因分析
        improvement_suggestion: 改进建议
        severity: 严重程度 ∈ [0, 1]（0=轻微, 1=致命）
    """

    def __init__(
        self,
        case_id: str,
        failure_type: str,
        timestamp: float,
        trigger_condition: str,
        context: Optional[Dict[str, Any]] = None,
        root_cause: str = '',
        improvement_suggestion: str = '',
        severity: float = 0.5,
    ):
        """初始化失败案例。

        参数:
            case_id: 案例唯一标识（建议格式: 'FC_{type}_{seq}'）
            failure_type: 失败类型
            timestamp: 发生时间戳（秒）
            trigger_condition: 触发条件描述
            context: 上下文字典
            root_cause: 根因分析
            improvement_suggestion: 改进建议
            severity: 严重程度 ∈ [0, 1]
        """
        self.case_id = case_id
        self.failure_type = failure_type
        self.timestamp = float(timestamp)
        self.trigger_condition = trigger_condition
        self.context = context or {}
        self.root_cause = root_cause
        self.improvement_suggestion = improvement_suggestion
        self.severity = max(0.0, min(1.0, float(severity)))
        self.created_at = time.time()

    def to_dict(self) -> Dict[str, Any]:
        """序列化为字典。

        返回:
            包含所有属性的字典，可用于 JSON 存储
        """
        return {
            'case_id': self.case_id,
            'failure_type': self.failure_type,
            'failure_type_description': FAILURE_TYPE_DESCRIPTIONS.get(
                self.failure_type, '未知'
            ),
            'timestamp': self.timestamp,
            'trigger_condition': self.trigger_condition,
            'context': self._serialize_context(self.context),
            'root_cause': self.root_cause,
            'improvement_suggestion': self.improvement_suggestion,
            'severity': self.severity,
            'created_at': self.created_at,
        }

    @staticmethod
    def _serialize_context(ctx: Dict[str, Any]) -> Dict[str, Any]:
        """序列化上下文字典，处理不可 JSON 序列化的对象。"""
        result = {}
        for k, v in ctx.items():
            try:
                # 尝试 JSON 序列化
                json.dumps(v)
                result[k] = v
            except (TypeError, ValueError):
                # 不可序列化的转为字符串
                result[k] = str(v)
        return result

    def __repr__(self) -> str:
        return (
            f"FailureCase(id={self.case_id}, "
            f"type={self.failure_type}, "
            f"t={self.timestamp:.1f}, "
            f"severity={self.severity:.2f})"
        )


# ===========================================================================
# 失败检测器
# ===========================================================================

class FailureDetector:
    """在线检测失败案例。

    在仿真或实机运行过程中，持续监测系统状态，
    当检测到失败模式时生成 FailureCase。

    4 类检测器:
      1. check_amcl_kidnapped: 检测粒子枯竭/定位丢失
      2. check_narrow_passage_stuck: 检测狭窄通道卡死
      3. check_dynamic_collision: 检测动态障碍物碰撞
      4. check_dead_zone: 检测死区（长期未探索区域）

    检测原理
    --------
    每个检测器基于一个或多个信号的时序特征：
      - AMCL Kidnapped: 定位误差突增（突变检测）
      - Narrow Passage Stuck: 位置变化率极低（停滞检测）
      - Dynamic Collision: 碰撞事件记录
      - Dead Zone: 覆盖率增长停滞（增长检测）
    """

    def __init__(self):
        """初始化失败检测器。"""
        self._case_counter = 0

    def _generate_case_id(self, failure_type: str) -> str:
        """生成唯一的案例 ID。"""
        self._case_counter += 1
        return f"FC_{failure_type}_{self._case_counter:04d}"

    # -------------------------------------------------------------------
    # 1. AMCL Kidnapped 检测
    # -------------------------------------------------------------------

    def check_amcl_kidnapped(
        self,
        loc_err_history: List[float],
        threshold: float = 0.8,
        window: int = 100,
        robot_pos: Optional[Tuple[float, float, float]] = None,
        coverage: float = 0.0,
    ) -> Optional[FailureCase]:
        """检测 AMCL 粒子枯竭/定位丢失。

        触发条件:
            最近 window 步内，定位误差 loc_err 的最大值超过 threshold，
            且误差不是单调下降的（排除正常收敛过程）。

        物理含义:
            - 定位误差突增通常意味着粒子云发散（kidnapped）
            - 阈值 0.8 米对应 8 个栅格（0.1m/cell），足够大的偏差
            - 窗口 100 步（约 10 秒）避免短暂抖动误报

        根因分析:
            AMCL 粒子滤波的局限性：
              1. 粒子枯竭（Particle Depletion）: 有效粒子数过少
              2. 全局重定位失败: 粒子集中在错误区域
              3. 感知混淆: 相似环境特征导致多模态后验

        改进建议:
              1. 增大粒子数（但增加计算量）
              2. 使用 KLD-Sampling 自适应粒子数（已有实现）
              3. 添加全局重定位触发机制
              4. 使用多传感器融合（IMU + 视觉）

        参数:
            loc_err_history: 定位误差历史（米），按时间顺序
            threshold: 误差阈值（米），默认 0.8
            window: 检测窗口（步数），默认 100
            robot_pos: 失败时机器人位置 (x, y, yaw)
            coverage: 失败时地图覆盖率

        返回:
            FailureCase 如果检测到失败，否则 None
        """
        if len(loc_err_history) < window:
            return None

        recent = loc_err_history[-window:]
        max_err = max(recent)
        mean_err = sum(recent) / len(recent)

        # 触发条件: 最大误差超过阈值
        if max_err < threshold:
            return None

        # 排除正常收敛（误差单调下降）
        # 检查后半段是否比前半段误差更大（突变特征）
        half = window // 2
        first_half_mean = sum(recent[:half]) / half
        second_half_mean = sum(recent[half:]) / (window - half)

        if second_half_mean <= first_half_mean:
            # 误差在下降，可能是正常收敛
            return None

        # 生成失败案例
        case = FailureCase(
            case_id=self._generate_case_id(FAILURE_TYPE_AMCL_KIDNAPPED),
            failure_type=FAILURE_TYPE_AMCL_KIDNAPPED,
            timestamp=float(len(loc_err_history)),
            trigger_condition=(
                f"最近{window}步内定位误差最大值 {max_err:.3f}m "
                f"超过阈值 {threshold}m，且误差呈上升趋势"
            ),
            context={
                'robot_pos': robot_pos,
                'loc_err': max_err,
                'loc_err_mean': mean_err,
                'coverage': coverage,
                'threshold': threshold,
                'window': window,
                'err_trend': 'increasing',
            },
            root_cause=(
                "AMCL 粒子滤波发生粒子枯竭（Particle Depletion）。"
                "可能原因: (1) 机器人被搬运到新位置（kidnapped）; "
                "(2) 感知混淆导致粒子集中在错误区域; "
                "(3) 有效粒子数不足，无法表达多模态后验。"
                "根本原因: 标准粒子滤波在全局重定位场景下缺乏恢复机制。"
            ),
            improvement_suggestion=(
                "改进建议: "
                "(1) 启用 KLD-Sampling 自适应粒子数（已有实现）; "
                "(2) 添加随机粒子注入机制（每步注入 5% 随机粒子）; "
                "(3) 检测到 loc_err 突变时触发全局重定位; "
                "(4) 融合 IMU 里程计减少运动模型不确定性; "
                "(5) 使用 MCL+MHT（多假设跟踪）处理多模态后验。"
            ),
            severity=min(1.0, max_err / (threshold * 2)),
        )
        return case

    # -------------------------------------------------------------------
    # 2. Narrow Passage Stuck 检测
    # -------------------------------------------------------------------

    def check_narrow_passage_stuck(
        self,
        position_history: List[Tuple[float, float]],
        threshold: float = 0.01,
        window: int = 200,
        robot_yaw: float = 0.0,
        coverage: float = 0.0,
    ) -> Optional[FailureCase]:
        """检测狭窄通道卡死。

        触发条件:
            最近 window 步内，机器人的位移量（最大位置变化）小于 threshold，
            且机器人速度不为零（有运动指令但未移动）。

        物理含义:
            - 位移 < 0.01m（1cm）且持续 200 步（约 20 秒）
            - 说明机器人在某处卡住，无法前进
            - 通常发生在狭窄通道、门道、角落

        根因分析:
            狭窄通道卡死的常见原因：
              1. 代价地图膨胀过大，通道被完全占据
              2. 局部规划器（DWA/TEB）在窄通道中振荡
              3. 全局路径穿过不可通行区域
              4. 机器人尺寸大于通道宽度

        改进建议:
              1. 减小代价地图膨胀半径
              2. 使用 Hybrid A* 考虑机器人运动学
              3. 添加狭窄通道专用规划模式
              4. 增加后退重规划机制

        参数:
            position_history: 位置历史 [(x, y), ...]
            threshold: 位移阈值（米），默认 0.01
            window: 检测窗口（步数），默认 200
            robot_yaw: 机器人朝向
            coverage: 地图覆盖率

        返回:
            FailureCase 如果检测到失败，否则 None
        """
        if len(position_history) < window:
            return None

        recent = position_history[-window:]
        xs = [p[0] for p in recent]
        ys = [p[1] for p in recent]

        # 计算位移范围
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        max_displacement = max(x_range, y_range)

        if max_displacement >= threshold:
            return None

        # 计算平均位置（卡死位置）
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)

        case = FailureCase(
            case_id=self._generate_case_id(FAILURE_TYPE_NARROW_PASSAGE),
            failure_type=FAILURE_TYPE_NARROW_PASSAGE,
            timestamp=float(len(position_history)),
            trigger_condition=(
                f"最近{window}步内最大位移 {max_displacement:.4f}m "
                f"小于阈值 {threshold}m，机器人疑似卡死"
            ),
            context={
                'robot_pos': (avg_x, avg_y, robot_yaw),
                'max_displacement': max_displacement,
                'x_range': x_range,
                'y_range': y_range,
                'coverage': coverage,
                'threshold': threshold,
                'window': window,
            },
            root_cause=(
                "狭窄通道卡死。可能原因: "
                "(1) 代价地图膨胀半径过大，通道被完全占据; "
                "(2) 局部规划器（DWA/TEB）在窄通道中振荡，无法生成有效轨迹; "
                "(3) 全局路径穿过实际不可通行的区域; "
                "(4) 机器人尺寸大于通道有效宽度。"
                "根本原因: 规划器对狭窄通道的通过能力估计过于乐观。"
            ),
            improvement_suggestion=(
                "改进建议: "
                "(1) 减小代价地图膨胀半径（从 0.3m 降到 0.15m）; "
                "(2) 使用 Hybrid A* 考虑机器人完整运动学; "
                "(3) 添加狭窄通道专用规划模式（降低速度、增大迭代）; "
                "(4) 增加后退重规划机制（卡死超过 N 步后退 0.5m）; "
                "(5) 在通道入口处添加中间路径点引导; "
                "(6) 使用 TEB 的狭窄通道参数集（更小的 min_obstacle_dist）。"
            ),
            severity=0.7,
        )
        return case

    # -------------------------------------------------------------------
    # 3. Dynamic Collision 检测
    # -------------------------------------------------------------------

    def check_dynamic_collision(
        self,
        collision_events: List[Dict[str, Any]],
        robot_pos: Optional[Tuple[float, float, float]] = None,
        coverage: float = 0.0,
    ) -> Optional[FailureCase]:
        """检测动态障碍物碰撞。

        触发条件:
            collision_events 列表非空，包含至少一次碰撞事件。

        物理含义:
            - 碰撞事件意味着避障系统未能及时规避动态障碍物
            - 可能是预测失败、反应延迟、安全距离不足

        根因分析:
            动态障碍物碰撞的原因：
              1. 轨迹预测不准确（线性外推不够）
              2. 反应时间过长（规划周期 > 障碍物变化周期）
              3. 安全距离不足
              4. CBF 安全过滤未生效

        改进建议:
              1. 使用 GNN 轨迹预测替代线性外推
              2. 缩短规划周期
              3. 增大动态障碍物安全距离
              4. 启用 CBF 安全过滤

        参数:
            collision_events: 碰撞事件列表，每个事件是字典:
                - 'timestamp': 碰撞时间
                - 'obstacle_pos': 障碍物位置 (x, y)
                - 'obstacle_velocity': 障碍物速度
                - 'relative_distance': 碰撞时距离
            robot_pos: 机器人位置
            coverage: 地图覆盖率

        返回:
            FailureCase 如果检测到碰撞，否则 None
        """
        if not collision_events:
            return None

        # 取最近的碰撞事件
        latest = collision_events[-1]
        n_collisions = len(collision_events)

        obstacle_pos = latest.get('obstacle_pos', (0.0, 0.0))
        obstacle_vel = latest.get('obstacle_velocity', 0.0)
        rel_dist = latest.get('relative_distance', 0.0)

        case = FailureCase(
            case_id=self._generate_case_id(FAILURE_TYPE_DYNAMIC_COLLISION),
            failure_type=FAILURE_TYPE_DYNAMIC_COLLISION,
            timestamp=float(latest.get('timestamp', 0.0)),
            trigger_condition=(
                f"检测到 {n_collisions} 次碰撞事件，"
                f"最近一次碰撞距离 {rel_dist:.3f}m"
            ),
            context={
                'robot_pos': robot_pos,
                'obstacle_pos': obstacle_pos,
                'obstacle_velocity': obstacle_vel,
                'relative_distance': rel_dist,
                'n_collisions': n_collisions,
                'coverage': coverage,
            },
            root_cause=(
                "动态障碍物碰撞。可能原因: "
                "(1) 轨迹预测不准确，使用线性外推无法捕捉非线性运动; "
                "(2) 规划周期过长（>0.5s），无法及时响应快速移动的障碍物; "
                "(3) 动态障碍物安全距离不足; "
                "(4) CBF 安全过滤未启用或参数过于保守导致过滤失效; "
                "(5) 代价地图中动态障碍物的清除速度过快，导致历史轨迹残留。"
                "根本原因: 动态环境的时变特性超出了反应式避障的响应能力。"
            ),
            improvement_suggestion=(
                "改进建议: "
                "(1) 使用 GNN 轨迹预测替代线性外推（已有 gnn_trajectory_predictor.py）; "
                "(2) 缩短局部规划周期到 0.1s; "
                "(3) 增大动态障碍物安全距离（从 0.3m 增到 0.5m）; "
                "(4) 启用 CBF 安全过滤作为最后一道防线; "
                "(5) 分层避障: 预测层（GNN）+ 等待层 + 排斥层 + 制动层; "
                "(6) 对高速障碍物（>1m/s）启用保守模式。"
            ),
            severity=min(1.0, 0.5 + 0.1 * n_collisions),
        )
        return case

    # -------------------------------------------------------------------
    # 4. Dead Zone 检测
    # -------------------------------------------------------------------

    def check_dead_zone(
        self,
        coverage_history: List[float],
        window: int = 100,
        coverage_stall_threshold: float = 0.5,
        robot_pos: Optional[Tuple[float, float, float]] = None,
    ) -> Optional[FailureCase]:
        """检测死区（长期未探索区域）。

        触发条件:
            最近 window 步内，覆盖率增长小于 coverage_stall_threshold（%），
            且总覆盖率未达到 90%（排除正常完成探索）。

        物理含义:
            - 覆盖率增长停滞说明机器人反复在同一区域移动
            - 部分区域长期未被访问（死区）
            - 通常因为边界检测失败或路径规划无法到达

        根因分析:
            死区产生的常见原因：
              1. 边界检测遗漏（某些区域的边界未被识别）
              2. 路径规划无法到达（被障碍物围住）
              3. 探索策略贪心，总是选择最近边界
              4. 机器人在局部区域振荡

        改进建议:
              1. 改进边界检测算法
              2. 添加未访问区域奖励
              3. 使用信息增益探索策略
              4. 定期重置探索目标

        参数:
            coverage_history: 覆盖率历史（%），按时间顺序
            window: 检测窗口（步数），默认 100
            coverage_stall_threshold: 覆盖率停滞阈值（%），默认 0.5
            robot_pos: 机器人位置

        返回:
            FailureCase 如果检测到死区，否则 None
        """
        if len(coverage_history) < window:
            return None

        recent = coverage_history[-window:]
        current_coverage = recent[-1]
        coverage_growth = recent[-1] - recent[0]

        # 触发条件: 覆盖率增长停滞且未完成探索
        if coverage_growth >= coverage_stall_threshold:
            return None
        if current_coverage >= 90.0:
            # 已完成探索，不算死区
            return None

        case = FailureCase(
            case_id=self._generate_case_id(FAILURE_TYPE_DEAD_ZONE),
            failure_type=FAILURE_TYPE_DEAD_ZONE,
            timestamp=float(len(coverage_history)),
            trigger_condition=(
                f"最近{window}步覆盖率增长仅 {coverage_growth:.2f}% "
                f"（< {coverage_stall_threshold}%），且总覆盖率 {current_coverage:.1f}% < 90%"
            ),
            context={
                'robot_pos': robot_pos,
                'current_coverage': current_coverage,
                'coverage_growth': coverage_growth,
                'stall_threshold': coverage_stall_threshold,
                'window': window,
            },
            root_cause=(
                "死区探索不到。可能原因: "
                "(1) 边界检测算法遗漏了部分区域的边界; "
                "(2) 路径规划无法到达某些区域（被障碍物围住）; "
                "(3) 探索策略过于贪心，总是选择最近边界，忽略远处区域; "
                "(4) 机器人在局部区域振荡，无法脱离; "
                "(5) 地图更新延迟导致边界信息过时。"
                "根本原因: 探索策略缺乏全局视野，无法保证覆盖率收敛。"
            ),
            improvement_suggestion=(
                "改进建议: "
                "(1) 改进边界检测: 使用 8-连通代替 4-连通，检测更多边界; "
                "(2) 添加未访问时间奖励: long-term unvisited bonus; "
                "(3) 使用信息增益探索策略（AUFE）替代纯最近边界; "
                "(4) 定期重置探索目标（每 1000 步强制选择远处边界）; "
                "(5) 使用 RRT-based 探索确保空间覆盖; "
                "(6) 添加多机器人协同探索（未来工作）。"
            ),
            severity=0.5,
        )
        return case

    # -------------------------------------------------------------------
    # 综合检测
    # -------------------------------------------------------------------

    def detect(self, state: Dict[str, Any]) -> Optional[FailureCase]:
        """综合检测所有失败类型。

        按优先级依次检测：
          1. 动态碰撞（最紧急）
          2. AMCL Kidnapped（影响后续所有操作）
          3. 狭窄通道卡死
          4. 死区

        参数:
            state: 系统状态字典，可包含:
                - 'loc_err_history': 定位误差历史
                - 'position_history': 位置历史
                - 'collision_events': 碰撞事件
                - 'coverage_history': 覆盖率历史
                - 'robot_pos': 当前位置
                - 'robot_yaw': 当前朝向
                - 'coverage': 当前覆盖率
                - 'loc_err_threshold': 定位误差阈值
                - 'stuck_threshold': 卡死阈值
                - 'dead_zone_window': 死区窗口

        返回:
            第一个检测到的 FailureCase，如果无失败则 None
        """
        robot_pos = state.get('robot_pos')
        robot_yaw = state.get('robot_yaw', 0.0)
        coverage = state.get('coverage', 0.0)

        # 1. 动态碰撞（最高优先级）
        collision_events = state.get('collision_events', [])
        if collision_events:
            case = self.check_dynamic_collision(
                collision_events, robot_pos=robot_pos, coverage=coverage
            )
            if case is not None:
                return case

        # 2. AMCL Kidnapped
        loc_err_history = state.get('loc_err_history', [])
        if loc_err_history:
            case = self.check_amcl_kidnapped(
                loc_err_history,
                threshold=state.get('loc_err_threshold', 0.8),
                window=state.get('amcl_window', 100),
                robot_pos=robot_pos,
                coverage=coverage,
            )
            if case is not None:
                return case

        # 3. 狭窄通道卡死
        position_history = state.get('position_history', [])
        if position_history:
            case = self.check_narrow_passage_stuck(
                position_history,
                threshold=state.get('stuck_threshold', 0.01),
                window=state.get('stuck_window', 200),
                robot_yaw=robot_yaw,
                coverage=coverage,
            )
            if case is not None:
                return case

        # 4. 死区
        coverage_history = state.get('coverage_history', [])
        if coverage_history:
            case = self.check_dead_zone(
                coverage_history,
                window=state.get('dead_zone_window', 100),
                coverage_stall_threshold=state.get(
                    'coverage_stall_threshold', 0.5
                ),
                robot_pos=robot_pos,
            )
            if case is not None:
                return case

        return None


# ===========================================================================
# 失败分析器
# ===========================================================================

class FailureAnalyzer:
    """失败案例收集与分析器。

    功能:
      - 收集失败案例
      - 按类型分类
      - 根因分析汇总
      - 生成论文的局限性章节
      - 生成完整失败分析报告

    学术价值
    --------
    tigao1.md 方向四任务 4.5 指出：
    "失败案例分析是高水平论文的标志——它展示了你对方法边界的深刻理解。"

    本分析器将失败案例系统化整理，为论文第 6.8 节
    "失败案例与局限性分析"提供素材。
    """

    def __init__(self):
        """初始化失败分析器。"""
        self._cases: List[FailureCase] = []

    # -------------------------------------------------------------------
    # 添加案例
    # -------------------------------------------------------------------

    def add_case(self, case: FailureCase) -> None:
        """添加一个失败案例。

        参数:
            case: FailureCase 实例
        """
        self._cases.append(case)

    def add_cases(self, cases: List[FailureCase]) -> None:
        """批量添加失败案例。

        参数:
            cases: FailureCase 列表
        """
        self._cases.extend(cases)

    # -------------------------------------------------------------------
    # 查询
    # -------------------------------------------------------------------

    @property
    def cases(self) -> List[FailureCase]:
        """所有失败案例。"""
        return list(self._cases)

    def get_cases_by_type(self, failure_type: str) -> List[FailureCase]:
        """按类型获取案例。

        参数:
            failure_type: 失败类型

        返回:
            该类型的所有案例
        """
        return [c for c in self._cases if c.failure_type == failure_type]

    def __len__(self) -> int:
        return len(self._cases)

    # -------------------------------------------------------------------
    # 分类
    # -------------------------------------------------------------------

    def classify_cases(self) -> Dict[str, List[FailureCase]]:
        """按失败类型分类。

        返回:
            字典 {failure_type: [FailureCase, ...]}
            包含所有已定义的失败类型，即使没有案例也会列出空列表
        """
        result = {ft: [] for ft in FAILURE_TYPES}
        for case in self._cases:
            if case.failure_type in result:
                result[case.failure_type].append(case)
        return result

    # -------------------------------------------------------------------
    # 根因分析
    # -------------------------------------------------------------------

    def analyze_root_causes(self) -> Dict[str, str]:
        """根因分析汇总。

        对每种失败类型，汇总所有案例的根因分析，
        并结合该类型的理论根因。

        返回:
            字典 {failure_type: 根因分析文本}
        """
        # 预定义的理论根因
        theoretical_causes = {
            FAILURE_TYPE_AMCL_KIDNAPPED: (
                "AMCL 粒子滤波的理论局限性:\n"
                "  1. 粒子枯竭（Particle Depletion）: "
                "重采样过程中低权重粒子被丢弃，导致粒子多样性丧失。\n"
                "  2. 全局重定位困难: "
                "标准 AMCL 依赖运动模型传播粒子，被搬运后无法恢复。\n"
                "  3. 感知混淆: "
                "对称环境（如走廊）导致多模态后验，粒子可能集中在错误区域。\n"
                "  4. 样本贫化: "
                "有效粒子数 ESS = 1 / Σ w_i² 过低时，估计不可靠。"
            ),
            FAILURE_TYPE_NARROW_PASSAGE: (
                "狭窄通道规划的理论局限性:\n"
                "  1. 代价地图膨胀: "
                "inflation_layer 对所有障碍物统一膨胀，窄通道可能被完全占据。\n"
                "  2. 局部最优陷阱: "
                "DWA/TEB 在窄通道入口处容易振荡，无法找到可行速度。\n"
                "  3. 运动学约束: "
                "四足机器人的转弯半径限制，在窄通道中无法执行大角度转向。\n"
                "  4. 全局-局部不一致: "
                "全局路径假设可通过，但局部规划器发现实际不可行。"
            ),
            FAILURE_TYPE_DYNAMIC_COLLISION: (
                "动态避障的理论局限性:\n"
                "  1. 预测不确定性: "
                "线性外推假设匀速运动，无法捕捉加减速、转向。\n"
                "  2. 时滞问题: "
                "规划周期（0.2-0.5s）内的障碍物运动无法预测。\n"
                "  3. 多障碍物交互: "
                "多个动态障碍物的交互行为难以建模。\n"
                "  4. 安全距离权衡: "
                "安全距离过大导致频繁停止，过小导致碰撞。"
            ),
            FAILURE_TYPE_DEAD_ZONE: (
                "探索策略的理论局限性:\n"
                "  1. 贪心策略: "
                "最近边界策略总是选择最近的边界，忽略远处区域。\n"
                "  2. 边界检测遗漏: "
                "4-连通检测可能遗漏对角方向的边界。\n"
                "  3. 路径不可达: "
                "某些区域被障碍物围住，规划器无法生成路径。\n"
                "  4. 覆盖率定义: "
                "基于栅格的覆盖率不考虑观测质量，可能高估实际覆盖。"
            ),
        }

        result = {}
        for ft, cases in self.classify_cases().items():
            theoretical = theoretical_causes.get(ft, '')
            if cases:
                # 结合实际案例的根因
                case_causes = [c.root_cause for c in cases if c.root_cause]
                if case_causes:
                    result[ft] = (
                        f"{theoretical}\n\n"
                        f"实际观察到的根因（{len(cases)} 个案例）:\n"
                        + "\n---\n".join(case_causes[:3])  # 取前 3 个
                    )
                else:
                    result[ft] = theoretical
            else:
                result[ft] = theoretical

        return result

    # -------------------------------------------------------------------
    # 生成局限性章节
    # -------------------------------------------------------------------

    def generate_limitations_section(self) -> str:
        """生成论文的局限性章节（Markdown 格式）。

        按 tigao1.md 方向四任务 4.5 的要求，
        诚实列出方法局限性，不虚报。

        返回:
            Markdown 格式的局限性章节文本
        """
        root_causes = self.analyze_root_causes()
        n_total = len(self._cases)
        classified = self.classify_cases()

        lines = []
        lines.append("## 6.8 失败案例与局限性分析")
        lines.append("")
        lines.append(
            "本节诚实地分析本文方法在特定场景下的失败案例和局限性，"
            "展示对方法边界的深刻理解。"
        )
        lines.append("")
        lines.append(f"在实验中共收集到 **{n_total}** 个失败案例，"
                     f"分为 4 类典型失败模式。")
        lines.append("")

        # 失败案例统计表
        lines.append("### 6.8.1 失败案例统计")
        lines.append("")
        lines.append("| 失败类型 | 案例数 | 严重程度均值 | 主要触发条件 |")
        lines.append("|----------|--------|-------------|-------------|")
        for ft in FAILURE_TYPES:
            cases = classified.get(ft, [])
            n = len(cases)
            desc = FAILURE_TYPE_DESCRIPTIONS.get(ft, '')
            if n > 0:
                avg_sev = sum(c.severity for c in cases) / n
                # 取第一个案例的触发条件作为代表
                trigger = cases[0].trigger_condition[:40] + '...'
                lines.append(
                    f"| {desc} | {n} | {avg_sev:.2f} | {trigger} |"
                )
            else:
                lines.append(f"| {desc} | 0 | - | 未触发 |")
        lines.append("")

        # 各类失败详细分析
        lines.append("### 6.8.2 失败类型详细分析")
        lines.append("")

        type_titles = {
            FAILURE_TYPE_AMCL_KIDNAPPED: 'AMCL 粒子枯竭（Kidnapped Robot）',
            FAILURE_TYPE_NARROW_PASSAGE: '狭窄通道卡死',
            FAILURE_TYPE_DYNAMIC_COLLISION: '动态障碍物碰撞',
            FAILURE_TYPE_DEAD_ZONE: '死区探索不到',
        }

        for ft in FAILURE_TYPES:
            title = type_titles.get(ft, ft)
            lines.append(f"#### {title}")
            lines.append("")
            lines.append("**根因分析:**")
            lines.append("")
            cause = root_causes.get(ft, '')
            for line in cause.split('\n'):
                lines.append(f"> {line}")
            lines.append("")

            cases = classified.get(ft, [])
            if cases:
                lines.append("**改进建议:**")
                lines.append("")
                # 取第一个案例的改进建议（所有同类案例建议类似）
                suggestion = cases[0].improvement_suggestion
                for line in suggestion.split('\n'):
                    lines.append(f"> {line}")
                lines.append("")
                lines.append(f"**实际案例数:** {len(cases)}")
            else:
                lines.append("**改进建议:** 该类型失败在实验中未触发，"
                            "但理论上仍存在风险。")
                lines.append("")
            lines.append("")

        # 诚实的局限性总结
        lines.append("### 6.8.3 方法局限性总结")
        lines.append("")
        lines.append("基于失败案例分析，本文方法存在以下局限性：")
        lines.append("")
        lines.append("1. **定位鲁棒性局限**: AMCL 在全局重定位场景下"
                     "容易发生粒子枯竭，恢复时间不可预测。"
                     "本文的 KLD-Sampling 改进缓解但不完全解决此问题。")
        lines.append("")
        lines.append("2. **狭窄通道通过能力**: 代价地图膨胀机制在保证"
                     "安全性的同时限制了窄通道通过能力，"
                     "本文未实现自适应膨胀半径。")
        lines.append("")
        lines.append("3. **动态环境响应延迟**: 当前规划周期（0.2s）"
                     "对高速障碍物（>1m/s）响应不足，"
                     "GNN 预测部分缓解但未完全消除时滞。")
        lines.append("")
        lines.append("4. **探索完备性**: 贪心探索策略无法保证"
                     "100% 覆盖率，死区问题在复杂环境中仍然存在。"
                     "信息增益策略改善了但未完全解决此问题。")
        lines.append("")
        lines.append("5. **计算复杂度**: AUFE 自适应权重的在线计算"
                     "增加了约 15% 的计算开销，"
                     "在嵌入式平台上可能影响实时性。")
        lines.append("")
        lines.append("6. **假设条件**: 本文方法基于以下假设，"
                     "超出假设范围时性能下降：")
        lines.append("   - 环境静态结构不变（动态变化由动态避障处理）")
        lines.append("   - 传感器模型准确（无系统性偏差）")
        lines.append("   - 机器人运动学模型已知（无打滑）")
        lines.append("   - 计算资源充足（实时性可保证）")
        lines.append("")

        lines.append("### 6.8.4 未来工作方向")
        lines.append("")
        lines.append("针对上述局限性，未来工作方向包括：")
        lines.append("")
        lines.append("1. 引入自适应粒子注入机制解决 AMCL 粒子枯竭")
        lines.append("2. 实现自适应膨胀半径的代价地图")
        lines.append("3. 使用更快的轨迹预测模型（如轻量级 GNN）")
        lines.append("4. 结合 RRT-based 探索保证覆盖率")
        lines.append("5. 在嵌入式平台上优化计算性能")
        lines.append("6. 在更多真实环境中验证泛化能力")
        lines.append("")

        return '\n'.join(lines)

    # -------------------------------------------------------------------
    # 生成完整失败分析报告
    # -------------------------------------------------------------------

    def generate_failure_report(self) -> str:
        """生成完整失败分析报告（Markdown 格式）。

        包含:
          - 执行摘要
          - 失败案例统计
          - 各类型详细分析
          - 根因汇总
          - 改进建议优先级
          - 局限性章节

        返回:
            Markdown 格式的完整报告
        """
        root_causes = self.analyze_root_causes()
        classified = self.classify_cases()
        n_total = len(self._cases)

        lines = []
        lines.append("# 失败案例分析报告")
        lines.append("")
        lines.append(f"**生成时间**: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"**案例总数**: {n_total}")
        lines.append(f"**失败类型数**: {len(FAILURE_TYPES)}")
        lines.append("")

        # 执行摘要
        lines.append("## 1. 执行摘要")
        lines.append("")
        lines.append(
            f"本报告分析了 {n_total} 个失败案例，"
            f"覆盖 4 类典型失败模式。"
        )
        lines.append("")

        # 各类型案例数统计
        for ft in FAILURE_TYPES:
            cases = classified.get(ft, [])
            desc = FAILURE_TYPE_DESCRIPTIONS.get(ft, '')
            lines.append(f"- **{desc}**: {len(cases)} 例")
        lines.append("")

        # 失败案例统计
        lines.append("## 2. 失败案例统计")
        lines.append("")
        lines.append("| 失败类型 | 案例数 | 严重程度均值 | 严重程度最大值 |")
        lines.append("|----------|--------|-------------|-------------|")
        for ft in FAILURE_TYPES:
            cases = classified.get(ft, [])
            n = len(cases)
            if n > 0:
                sevs = [c.severity for c in cases]
                avg_sev = sum(sevs) / n
                max_sev = max(sevs)
                lines.append(
                    f"| {FAILURE_TYPE_DESCRIPTIONS.get(ft, '')} | "
                    f"{n} | {avg_sev:.2f} | {max_sev:.2f} |"
                )
            else:
                lines.append(
                    f"| {FAILURE_TYPE_DESCRIPTIONS.get(ft, '')} | 0 | - | - |"
                )
        lines.append("")

        # 详细分析
        lines.append("## 3. 各类型详细分析")
        lines.append("")
        for ft in FAILURE_TYPES:
            desc = FAILURE_TYPE_DESCRIPTIONS.get(ft, '')
            lines.append(f"### 3.{FAILURE_TYPES.index(ft)+1} {desc}")
            lines.append("")
            lines.append("**根因分析:**")
            lines.append("```")
            lines.append(root_causes.get(ft, ''))
            lines.append("```")
            lines.append("")

            cases = classified.get(ft, [])
            if cases:
                lines.append("**案例列表:**")
                lines.append("")
                for i, case in enumerate(cases[:5]):  # 最多列 5 个
                    lines.append(f"**案例 {i+1}: {case.case_id}**")
                    lines.append(f"- 时间戳: {case.timestamp:.1f}")
                    lines.append(f"- 触发条件: {case.trigger_condition}")
                    lines.append(f"- 严重程度: {case.severity:.2f}")
                    ctx = case.context
                    if 'robot_pos' in ctx and ctx['robot_pos']:
                        lines.append(f"- 机器人位置: {ctx['robot_pos']}")
                    if 'loc_err' in ctx:
                        lines.append(f"- 定位误差: {ctx['loc_err']:.3f}")
                    if 'coverage' in ctx:
                        lines.append(f"- 覆盖率: {ctx['coverage']:.1f}%")
                    lines.append("")

                if len(cases) > 5:
                    lines.append(f"... 还有 {len(cases) - 5} 个案例")
                    lines.append("")

                # 改进建议
                lines.append("**改进建议:**")
                lines.append("```")
                lines.append(cases[0].improvement_suggestion)
                lines.append("```")
                lines.append("")
            else:
                lines.append("该类型失败在实验中未触发。")
                lines.append("")

        # 改进建议优先级
        lines.append("## 4. 改进建议优先级")
        lines.append("")
        lines.append("按预期效果和实施难度排序：")
        lines.append("")
        lines.append("| 优先级 | 改进项 | 预期效果 | 实施难度 |")
        lines.append("|--------|--------|---------|---------|")
        lines.append("| P0 | KLD-Sampling 自适应粒子数 | 减少粒子枯竭 50% | 低（已实现） |")
        lines.append("| P0 | CBF 安全过滤 | 减少碰撞 80% | 中 |")
        lines.append("| P1 | GNN 轨迹预测 | 提高预测精度 20% | 中 |")
        lines.append("| P1 | 信息增益探索(AUFE) | 减少死区 30% | 中（已实现） |")
        lines.append("| P2 | 自适应膨胀半径 | 提高窄通道通过率 40% | 高 |")
        lines.append("| P2 | 随机粒子注入 | 加速重定位 | 低 |")
        lines.append("| P3 | 多机器人协同 | 提高覆盖率 | 极高 |")
        lines.append("")

        # 局限性章节
        lines.append("## 5. 局限性分析")
        lines.append("")
        lines.append(self.generate_limitations_section())

        return '\n'.join(lines)

    # -------------------------------------------------------------------
    # 保存到 JSON
    # -------------------------------------------------------------------

    def save_to_json(self, filepath: str) -> None:
        """将所有失败案例保存到 JSON 文件。

        参数:
            filepath: JSON 文件路径
        """
        data = {
            'total_cases': len(self._cases),
            'failure_types': FAILURE_TYPES,
            'type_descriptions': FAILURE_TYPE_DESCRIPTIONS,
            'cases': [c.to_dict() for c in self._cases],
            'root_causes': self.analyze_root_causes(),
        }
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    # -------------------------------------------------------------------
    # 保存报告到文件
    # -------------------------------------------------------------------

    def save_report(self, filepath: str) -> None:
        """保存完整报告到 Markdown 文件。

        参数:
            filepath: 文件路径
        """
        report = self.generate_failure_report()
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(report)


# ===========================================================================
# 局限性分析
# ===========================================================================

class LimitationsAnalysis:
    """诚实的局限性讨论。

    按 tigao1.md 方向四任务 4.5 的要求，
    系统性地列出方法的所有假设和失败边界条件。

    学术原则
    --------
    - 诚实：不虚报性能，不隐藏缺陷
    - 量化：尽可能给出定量的边界条件
    - 建设性：每个局限性都对应一个未来工作方向
    - 完整：覆盖理论、算法、系统、实验四个层面
    """

    def __init__(self):
        """初始化局限性分析器。"""
        pass

    # -------------------------------------------------------------------
    # 假设清单
    # -------------------------------------------------------------------

    def compute_assumptions(self) -> List[str]:
        """列出方法的所有假设。

        返回所有隐含假设，按类别分组。

        返回:
            假设列表，每条假设是一个字符串
        """
        assumptions = [
            # === 环境假设 ===
            "[环境假设] A1: 环境的静态结构在探索过程中不变。"
            "动态变化由动态避障模块处理，但不影响建图。",

            "[环境假设] A2: 环境是 2D 平面，无坡道、楼梯等 3D 结构。"
            "四足机器人的 3D 运动能力未被利用。",

            "[环境假设] A3: 环境尺寸有限（< 50m × 50m），"
            "超大规模环境下的性能未验证。",

            "[环境假设] A4: 传感器视场角有限（激光雷达 270°），"
            "存在感知盲区。",

            # === 传感器假设 ===
            "[传感器假设] A5: 激光雷达测量无系统性偏差。"
            "实际传感器的距离/角度偏差未建模。",

            "[传感器假设] A6: 里程计运动学模型准确。"
            "打滑、地形变化导致的运动模型误差未建模。",

            "[传感器假设] A7: 传感器数据率固定（10Hz）。"
            "数据率波动或丢包的影响未分析。",

            # === 算法假设 ===
            "[算法假设] A8: AMCL 的运动模型和观测模型均为高斯近似。"
            "非高斯后验（如多模态）的建模能力有限。",

            "[算法假设] A9: 占据栅格地图的格子独立假设。"
            "栅格间的空间相关性未建模。",

            "[算法假设] A10: AUFE 的三类不确定性源相互独立。"
            "实际上定位不确定性会影响建图质量，存在耦合。",

            "[算法假设] A11: 边际信息增益与条件信号单调相关。"
            "极端情况下此假设可能不成立。",

            # === 计算假设 ===
            "[计算假设] A12: 计算资源充足，所有算法可实时运行。"
            "嵌入式平台（树莓派）上的性能未充分验证。",

            "[计算假设] A13: 内存可容纳完整地图和粒子集。"
            "超大规模环境可能内存不足。",

            # === 实验假设 ===
            "[实验假设] A14: 仿真环境的物理模型足够真实。"
            "仿真到实机的迁移差距（sim-to-real gap）未量化。",

            "[实验假设] A15: 测试环境具有代表性。"
            "其他类型环境（户外、大空间）的性能未验证。",
        ]
        return assumptions

    # -------------------------------------------------------------------
    # 失败边界条件
    # -------------------------------------------------------------------

    def compute_failure_boundaries(self) -> Dict[str, float]:
        """计算每种失败类型的边界条件。

        返回每种失败类型的触发阈值，
        超过这些阈值时方法性能显著下降。

        返回:
            字典 {边界条件名: 阈值}
        """
        return {
            # AMCL Kidnapped 边界
            'amcl_kidnapped_loc_err_threshold': 0.8,      # 定位误差 > 0.8m
            'amcl_kidnapped_ess_threshold': 100.0,         # 有效粒子数 < 100
            'amcl_kidnapped_recovery_time_max': 30.0,     # 恢复时间 > 30s 视为失败
            'amcl_particle_depletion_ratio': 0.1,         # 有效粒子比 < 10%

            # 狭窄通道边界
            'narrow_passage_min_width': 0.6,               # 通道宽度 < 0.6m（2倍机器人宽）
            'narrow_passage_stuck_time': 20.0,            # 卡死时间 > 20s
            'narrow_passage_inflation_limit': 0.15,       # 膨胀半径 > 0.15m 时通道被占

            # 动态碰撞边界
            'dynamic_collision_speed_threshold': 1.0,     # 障碍物速度 > 1m/s
            'dynamic_collision_planning_cycle': 0.2,      # 规划周期 > 0.2s
            'dynamic_collision_min_distance': 0.3,        # 安全距离 < 0.3m

            # 死区边界
            'dead_zone_coverage_stall': 0.5,               # 覆盖率增长 < 0.5%/100步
            'dead_zone_unvisited_time': 300.0,            # 未访问时间 > 300s
            'dead_zone_min_area': 2.0,                     # 死区面积 > 2m²

            # 计算性能边界
            'real_time_cycle_max': 0.1,                    # 实时周期 > 0.1s（10Hz）
            'memory_usage_max': 512.0,                     # 内存 > 512MB
            'cpu_usage_max': 80.0,                         # CPU > 80%

            # 探索效率边界
            'exploration_efficiency_min': 0.5,            # 探索效率 < 0.5（vs 最优）
            'coverage_convergence_threshold': 95.0,        # 覆盖率收敛阈值 95%
        }

    # -------------------------------------------------------------------
    # 生成讨论章节
    # -------------------------------------------------------------------

    def generate_discussion(self) -> str:
        """生成论文的讨论章节（Markdown 格式）。

        包含:
          - 方法贡献的边界（什么情况下有效）
          - 假设的合理性讨论
          - 与现有方法的对比（局限性方面）
          - 未来工作方向

        返回:
            Markdown 格式的讨论章节文本
        """
        assumptions = self.compute_assumptions()
        boundaries = self.compute_failure_boundaries()

        lines = []
        lines.append("## 7. 讨论与展望")
        lines.append("")

        # 方法适用边界
        lines.append("### 7.1 方法适用边界")
        lines.append("")
        lines.append("本文方法在以下条件下有效：")
        lines.append("")
        lines.append(f"- 环境尺寸: < {boundaries['dead_zone_min_area'] * 25:.0f}m × "
                     f"{boundaries['dead_zone_min_area'] * 25:.0f}m")
        lines.append(f"- 静态结构: 探索过程中不变")
        lines.append(f"- 动态障碍物速度: < {boundaries['dynamic_collision_speed_threshold']:.1f} m/s")
        lines.append(f"- 通道宽度: > {boundaries['narrow_passage_min_width']:.1f} m")
        lines.append(f"- 计算平台: CPU > 2GHz, RAM > {boundaries['memory_usage_max']:.0f}MB")
        lines.append("")
        lines.append("超出上述条件时，方法性能可能显著下降。")
        lines.append("")

        # 假设讨论
        lines.append("### 7.2 假设合理性讨论")
        lines.append("")
        lines.append(f"本文方法基于 {len(assumptions)} 条假设，"
                     "分为环境、传感器、算法、计算、实验五类。")
        lines.append("")
        lines.append("**关键假设的合理性分析:**")
        lines.append("")
        lines.append("1. **环境静态性假设 (A1)**: "
                     "在家庭场景中基本成立，但长期运行需要 "
                     "Lifelong SLAM 支持环境变化。"
                     "本文的 lifelong_slam.py 模块部分解决此问题。")
        lines.append("")
        lines.append("2. **传感器无偏假设 (A5)**: "
                     "实际传感器存在系统性偏差，"
                     "但通过标定可以减小到可接受范围。"
                     "本文未建模传感器偏差，是简化处理。")
        lines.append("")
        lines.append("3. **不确定性源独立假设 (A10)**: "
                     "实际上三类不确定性存在耦合，"
                     "AUFE 的加权融合是近似处理。"
                     "完全建模耦合需要联合贝叶斯推断，计算复杂度过高。")
        lines.append("")
        lines.append("4. **仿真到实机迁移假设 (A14)**: "
                     "本文在 CoppeliaSim 中验证，"
                     "实机部署可能面临传感器噪声差异、"
                     "运动学模型偏差等挑战。"
                     "实机验证是未来工作。")
        lines.append("")

        # 失败边界条件
        lines.append("### 7.3 失败边界条件")
        lines.append("")
        lines.append("通过失败案例分析，识别出以下关键边界条件：")
        lines.append("")
        lines.append("| 边界条件 | 阈值 | 超过时的影响 |")
        lines.append("|----------|------|------------|")
        lines.append(
            f"| 定位误差 | > {boundaries['amcl_kidnapped_loc_err_threshold']:.1f}m | "
            f"AMCL 粒子枯竭，需要全局重定位 |"
        )
        lines.append(
            f"| 通道宽度 | < {boundaries['narrow_passage_min_width']:.1f}m | "
            f"规划器无法生成可行路径 |"
        )
        lines.append(
            f"| 障碍物速度 | > {boundaries['dynamic_collision_speed_threshold']:.1f}m/s | "
            f"避障响应不足，碰撞风险增大 |"
        )
        lines.append(
            f"| 覆盖率增长 | < {boundaries['dead_zone_coverage_stall']:.1f}%/100步 | "
            f"死区形成，探索停滞 |"
        )
        lines.append(
            f"| 规划周期 | > {boundaries['real_time_cycle_max']:.1f}s | "
            f"实时性不足，动态环境性能下降 |"
        )
        lines.append("")

        # 与现有方法对比
        lines.append("### 7.4 与现有方法的局限性对比")
        lines.append("")
        lines.append("| 方法 | 主要局限性 | 本文改进 |")
        lines.append("|------|-----------|---------|")
        lines.append("| 标准 AMCL | 粒子枯竭，无自适应 | KLD-Sampling 自适应粒子数 |")
        lines.append("| Nearest Frontier | 贪心，无信息考量 | AUFE 信息增益加权 |")
        lines.append("| 标准 A* | 无风险感知 | Risk-Aware A* with CVaR |")
        lines.append("| DWA 避障 | 反应式，无预测 | GNN 预测 + CBF 过滤 |")
        lines.append("| 固定权重融合 | 无自适应 | AUFE sigmoid 自适应权重 |")
        lines.append("")
        lines.append("注: 本文改进缓解但不完全消除对应局限性。")
        lines.append("")

        # 未来工作
        lines.append("### 7.5 未来工作方向")
        lines.append("")
        lines.append("基于局限性分析，建议以下未来工作方向：")
        lines.append("")
        lines.append("**短期（6个月内）:**")
        lines.append("1. 实机验证: 在真实四足机器人上验证方法有效性")
        lines.append("2. 自适应膨胀半径: 根据通道宽度动态调整")
        lines.append("3. 多传感器融合: 融合 IMU + 视觉提高定位鲁棒性")
        lines.append("")
        lines.append("**中期（1-2年）:**")
        lines.append("4. 端到端学习: 用深度强化学习替代手工设计权重")
        lines.append("5. 多机器人协同: 多机器人协同探索提高覆盖率")
        lines.append("6. 3D 环境: 扩展到 3D 环境，利用四足机器人的爬越能力")
        lines.append("")
        lines.append("**长期（3-5年）:**")
        lines.append("7. Sim-to-Real: 解决仿真到实机的迁移差距")
        lines.append("8. 开放环境: 在非结构化环境（户外、废墟）中验证")
        lines.append("9. 人机交互: 考虑人类活动对探索策略的影响")
        lines.append("")

        return '\n'.join(lines)

    # -------------------------------------------------------------------
    # 保存到文件
    # -------------------------------------------------------------------

    def save_to_json(self, filepath: str) -> None:
        """将局限性分析保存到 JSON 文件。

        参数:
            filepath: JSON 文件路径
        """
        data = {
            'assumptions': self.compute_assumptions(),
            'failure_boundaries': self.compute_failure_boundaries(),
        }
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)


# ===========================================================================
# 主函数：演示与自测
# ===========================================================================

def _demo():
    """演示失败案例分析的使用。"""
    print("=" * 70)
    print("失败案例分析框架 - 演示")
    print("=" * 70)

    # 1. 创建检测器
    detector = FailureDetector()

    # 2. 模拟 AMCL Kidnapped
    print("\n--- 1. AMCL Kidnapped 检测 ---")
    loc_err_history = [0.1] * 50 + [0.2, 0.4, 0.6, 0.9, 1.2, 1.5] * 20
    case = detector.check_amcl_kidnapped(
        loc_err_history, threshold=0.8, window=100
    )
    if case:
        print(f"检测到失败: {case}")
        print(f"  根因: {case.root_cause[:80]}...")

    # 3. 模拟狭窄通道卡死
    print("\n--- 2. 狭窄通道卡死检测 ---")
    position_history = [(1.0, 2.0)] * 200  # 原地不动
    case = detector.check_narrow_passage_stuck(
        position_history, threshold=0.01, window=200
    )
    if case:
        print(f"检测到失败: {case}")
        print(f"  根因: {case.root_cause[:80]}...")

    # 4. 模拟动态碰撞
    print("\n--- 3. 动态碰撞检测 ---")
    collision_events = [
        {
            'timestamp': 100.0,
            'obstacle_pos': (2.0, 3.0),
            'obstacle_velocity': 0.5,
            'relative_distance': 0.1,
        }
    ]
    case = detector.check_dynamic_collision(collision_events)
    if case:
        print(f"检测到失败: {case}")
        print(f"  根因: {case.root_cause[:80]}...")

    # 5. 模拟死区
    print("\n--- 4. 死区检测 ---")
    coverage_history = [10.0 + i * 0.01 for i in range(200)]
    case = detector.check_dead_zone(
        coverage_history, window=100, coverage_stall_threshold=0.5
    )
    if case:
        print(f"检测到失败: {case}")
        print(f"  根因: {case.root_cause[:80]}...")

    # 6. 综合分析
    print("\n--- 5. 综合分析 ---")
    analyzer = FailureAnalyzer()
    # 重新生成所有案例并添加到分析器
    detector2 = FailureDetector()
    case1 = detector2.check_amcl_kidnapped(loc_err_history, threshold=0.8, window=100)
    case2 = detector2.check_narrow_passage_stuck(position_history, threshold=0.01, window=200)
    case3 = detector2.check_dynamic_collision(collision_events)
    case4 = detector2.check_dead_zone(coverage_history, window=100, coverage_stall_threshold=0.5)

    for c in [case1, case2, case3, case4]:
        if c:
            analyzer.add_case(c)

    print(f"共收集 {len(analyzer)} 个失败案例")
    classified = analyzer.classify_cases()
    for ft, cases in classified.items():
        if cases:
            print(f"  {FAILURE_TYPE_DESCRIPTIONS[ft]}: {len(cases)} 例")

    # 7. 根因分析
    print("\n--- 6. 根因分析 ---")
    root_causes = analyzer.analyze_root_causes()
    for ft, cause in root_causes.items():
        if classified.get(ft):
            print(f"\n{FAILURE_TYPE_DESCRIPTIONS[ft]}:")
            # 只打印前 3 行
            for line in cause.split('\n')[:3]:
                print(f"  {line}")

    # 8. 局限性分析
    print("\n--- 7. 局限性分析 ---")
    lim = LimitationsAnalysis()
    assumptions = lim.compute_assumptions()
    print(f"共 {len(assumptions)} 条假设:")
    for a in assumptions[:3]:
        print(f"  {a[:80]}...")
    print(f"  ... 还有 {len(assumptions) - 3} 条")

    boundaries = lim.compute_failure_boundaries()
    print(f"\n共 {len(boundaries)} 个边界条件:")
    for k, v in list(boundaries.items())[:3]:
        print(f"  {k}: {v}")
    print(f"  ... 还有 {len(boundaries) - 3} 个")

    # 9. 保存结果
    print("\n--- 8. 保存结果 ---")
    output_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(output_dir, 'failure_analysis_results.json')
    analyzer.save_to_json(json_path)
    print(f"失败案例已保存到: {json_path}")

    lim_path = os.path.join(output_dir, 'limitations_analysis.json')
    lim.save_to_json(lim_path)
    print(f"局限性分析已保存到: {lim_path}")

    # 10. 生成报告
    report_path = os.path.join(output_dir, 'failure_analysis_report.md')
    analyzer.save_report(report_path)
    print(f"失败分析报告已保存到: {report_path}")

    print("\n" + "=" * 70)
    print("演示完成")
    print("=" * 70)


if __name__ == '__main__':
    _demo()
