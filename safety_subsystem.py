"""安全子系统模块

本模块实现了一个纯 Python 的机器人安全子系统，可在仿真环境和 ROS2 中复用。
主要功能包括：
    1. 碰撞概率计算：基于高斯模型评估轨迹与障碍物的碰撞风险
    2. 紧急制动响应：根据激光雷达和动态障碍物判断是否需要紧急制动
    3. 风险评估：综合考虑距离、速度、碰撞时间(TTC)等因素计算风险评分
    4. 心跳检测：监控各节点运行状态，检测超时节点
    5. 恢复策略：根据风险等级和故障类型给出恢复动作建议
    6. 安全状态机：管理 NORMAL → CAUTIOUS → WARNING → DANGER → EMERGENCY_STOP 状态转换
    7. 事件日志：记录安全事件并支持导出 CSV 报告

依赖：numpy（数值计算）、Python 标准库（math、time、dataclasses、enum、csv）
"""

import csv
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np


class SafetyState(IntEnum):
    """安全状态枚举

    表示安全子系统的状态机所处状态，状态严重程度递增。
    """

    NORMAL = 0          # 正常：无安全风险
    CAUTIOUS = 1        # 谨慎：存在轻微风险，需提高警觉
    WARNING = 2         # 警告：风险较高，需减速
    DANGER = 3          # 危险：风险很高，需立即响应
    EMERGENCY_STOP = 4  # 紧急停止：触发紧急制动


@dataclass
class RiskAssessment:
    """风险评估结果

    属性:
        score: 综合风险评分（0-100），数值越大风险越高
        factors: 各风险因子及其贡献分值
        recommendations: 针对当前风险给出的建议动作列表
    """

    score: float = 0.0
    factors: Dict[str, float] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)


class SafetySubsystem:
    """安全子系统

    提供机器人运行期间的安全监控与响应能力，包括碰撞概率评估、紧急制动、
    风险评分、心跳检测、恢复策略及安全状态机。所有方法均为纯 Python 实现，
    不依赖 ROS2，可在仿真和真实机器人系统中复用。
    """

    # 各安全状态对应的速度缩放系数
    _VELOCITY_SCALE: Dict[SafetyState, float] = {
        SafetyState.NORMAL: 1.0,
        SafetyState.CAUTIOUS: 0.7,
        SafetyState.WARNING: 0.4,
        SafetyState.DANGER: 0.1,
        SafetyState.EMERGENCY_STOP: 0.0,
    }

    def __init__(self, safety_distance: float = 1.0,
                 brake_threshold: float = 0.3,
                 danger_threshold: float = 0.5,
                 warning_threshold: float = 1.0,
                 cautious_threshold: float = 1.5) -> None:
        """初始化安全子系统

        参数:
            safety_distance: 高斯碰撞模型中的安全距离参数 σ（米）
            brake_threshold: 触发紧急制动的距离阈值（米）
            danger_threshold: 判定为危险等级的距离阈值（米）
            warning_threshold: 判定为警告等级的距离阈值（米）
            cautious_threshold: 判定为注意等级的距离阈值（米）
        """
        self.safety_distance = safety_distance
        self.brake_threshold = brake_threshold
        self.danger_threshold = danger_threshold
        self.warning_threshold = warning_threshold
        self.cautious_threshold = cautious_threshold

        # 安全状态机当前状态
        self._state: SafetyState = SafetyState.NORMAL

        # 心跳记录：{节点名: 最近一次心跳时间戳}
        self.heartbeats: Dict[str, float] = {}

        # 安全事件日志
        self.event_log: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------
    # 1. 碰撞概率计算
    # ------------------------------------------------------------------
    def compute_collision_probability(self, trajectory: List[Tuple[float, float, float]],
                                      obstacles: List[Tuple[float, ...]],
                                      robot_radius: float) -> float:
        """计算轨迹的碰撞概率

        对轨迹中每个点计算到最近障碍物的距离，使用高斯模型计算碰撞概率：
            P(collision) = exp(-d² / (2σ²))
        其中 d 为机器人与障碍物之间的最近距离，σ 为安全距离参数。
        返回整条轨迹上的最大碰撞概率（最坏情况）。

        参数:
            trajectory: 轨迹点列表 [(x, y, theta), ...]
            obstacles: 障碍物列表，支持两种格式：
                       圆形障碍物 [(x, y, radius), ...]
                       矩形障碍物 [(xmin, ymin, xmax, ymax), ...]
            robot_radius: 机器人外接圆半径（米）

        返回:
            碰撞概率（0-1），取轨迹上最大值
        """
        if not trajectory or not obstacles:
            return 0.0

        sigma = self.safety_distance
        if sigma <= 0:
            return 1.0

        # 将轨迹坐标转为数组以便向量化计算
        traj_arr = np.asarray([(p[0], p[1]) for p in trajectory], dtype=float)

        max_prob = 0.0

        for obs in obstacles:
            if len(obs) == 3:
                # 圆形障碍物 (x, y, radius)
                ox, oy, orad = obs
                # 轨迹点到障碍物中心的距离
                center = np.array([ox, oy], dtype=float)
                dist_to_center = np.linalg.norm(traj_arr - center, axis=1)
                # 减去障碍物半径和机器人半径
                dist = np.maximum(dist_to_center - orad - robot_radius, 0.0)
            elif len(obs) == 4:
                # 矩形障碍物 (xmin, ymin, xmax, ymax)
                xmin, ymin, xmax, ymax = obs
                # 计算点到矩形外部的最短距离（向量化）
                dx = np.maximum(np.maximum(xmin - traj_arr[:, 0], 0.0),
                                traj_arr[:, 0] - xmax)
                dy = np.maximum(np.maximum(ymin - traj_arr[:, 1], 0.0),
                                traj_arr[:, 1] - ymax)
                dist = np.sqrt(dx ** 2 + dy ** 2) - robot_radius
                dist = np.maximum(dist, 0.0)
            else:
                # 不支持的障碍物格式，跳过
                continue

            # 高斯模型计算每个轨迹点的碰撞概率
            probs = np.exp(-(dist ** 2) / (2.0 * sigma ** 2))
            point_max = float(np.max(probs))
            if point_max > max_prob:
                max_prob = point_max

        return max_prob

    # ------------------------------------------------------------------
    # 2. 紧急制动响应
    # ------------------------------------------------------------------
    def check_emergency_brake(self, robot_pose: Tuple[float, float, float],
                              lidar_scan: List[Tuple[float, float]],
                              dynamic_obstacles: List[Tuple[float, ...]]
                              ) -> Tuple[bool, str, int]:
        """检查是否需要紧急制动

        综合激光雷达扫描数据和动态障碍物，计算机器人到最近障碍物的距离，
        据此判断是否需要制动及当前风险等级。

        参数:
            robot_pose: 机器人位姿 (x, y, theta)
            lidar_scan: 激光雷达扫描数据，[(range, angle), ...]
                        range 为距离（米），angle 为相对于机器人航向的角度（弧度）
            dynamic_obstacles: 动态障碍物列表 [(x, y, ...), ...]，至少包含位置

        返回:
            (should_brake, brake_reason, risk_level)
            - should_brake: 是否需要紧急制动
            - brake_reason: 制动原因描述
            - risk_level: 风险等级 0=安全, 1=注意, 2=警告, 3=危险
        """
        robot_x, robot_y, robot_theta = robot_pose
        min_dist = float('inf')

        # 处理激光雷达扫描数据
        for scan in lidar_scan:
            if scan is None or len(scan) < 2:
                continue
            range_val, angle = scan[0], scan[1]
            # 过滤无效值（NaN、非正数、inf）
            if not isinstance(range_val, (int, float)):
                continue
            if math.isnan(range_val) or math.isinf(range_val) or range_val <= 0.0:
                continue
            if range_val < min_dist:
                min_dist = range_val

        # 处理动态障碍物（基于全局坐标距离）
        for obs in dynamic_obstacles:
            if obs is None or len(obs) < 2:
                continue
            ox, oy = obs[0], obs[1]
            dist = math.hypot(robot_x - ox, robot_y - oy)
            if dist < min_dist:
                min_dist = dist

        # 根据最近距离判定风险等级与制动决策
        if min_dist <= self.brake_threshold:
            return (True, "障碍物距离小于%.2fm，触发紧急制动" % self.brake_threshold, 3)
        elif min_dist <= self.danger_threshold:
            return (False, "障碍物距离小于%.2fm，进入危险区域" % self.danger_threshold, 3)
        elif min_dist <= self.warning_threshold:
            return (False, "障碍物距离小于%.2fm，需减速" % self.warning_threshold, 2)
        elif min_dist <= self.cautious_threshold:
            return (False, "障碍物距离小于%.2fm，需提高警觉" % self.cautious_threshold, 1)
        else:
            return (False, "安全", 0)

    # ------------------------------------------------------------------
    # 3. 风险评估
    # ------------------------------------------------------------------
    def assess_risk(self, robot_pose: Tuple[float, float, float],
                    velocity: Union[float, Tuple[float, float]],
                    obstacles: List[Tuple[float, ...]],
                    dynamic_obstacles: List[Tuple[float, ...]]
                    ) -> RiskAssessment:
        """评估当前综合风险

        综合考虑障碍物距离、相对速度、碰撞时间(TTC)及路径曲率等因素，
        计算一个 0-100 的风险评分，并给出建议。

        参数:
            robot_pose: 机器人位姿 (x, y, theta)
            velocity: 机器人速度，可为标量速率或 (vx, vy) 向量
            obstacles: 静态障碍物列表，支持圆形或矩形格式
            dynamic_obstacles: 动态障碍物列表 [(x, y, vx, vy, ...), ...]

        返回:
            RiskAssessment 对象，包含 score、factors、recommendations
        """
        robot_x, robot_y, robot_theta = robot_pose

        # 归一化机器人速度为标量与向量
        if isinstance(velocity, (tuple, list, np.ndarray)) and len(velocity) >= 2:
            v_vec = np.array([velocity[0], velocity[1]], dtype=float)
            speed = float(np.linalg.norm(v_vec))
        else:
            speed = float(velocity)
            v_vec = np.array([speed * math.cos(robot_theta),
                              speed * math.sin(robot_theta)], dtype=float)

        factors: Dict[str, float] = {}
        recommendations: List[str] = []
        score = 0.0

        # ---- 因子1：到静态障碍物的最近距离 ----
        min_static_dist = float('inf')
        for obs in obstacles:
            dist = self._distance_to_obstacle(robot_x, robot_y, obs)
            if dist < min_static_dist:
                min_static_dist = dist

        if min_static_dist == float('inf'):
            distance_score = 0.0
        elif min_static_dist <= 0:
            distance_score = 40.0
            recommendations.append("立即停止：已接触或穿透障碍物")
        elif min_static_dist <= self.danger_threshold:
            distance_score = 30.0
            recommendations.append("大幅减速并避让静态障碍物")
        elif min_static_dist <= self.warning_threshold:
            distance_score = 20.0
            recommendations.append("减速接近静态障碍物")
        elif min_static_dist <= self.cautious_threshold:
            distance_score = 10.0
            recommendations.append("保持警觉注意静态障碍物")
        else:
            distance_score = 0.0
        factors["distance_score"] = distance_score
        score += distance_score

        # ---- 因子2：相对速度与 TTC ----
        ttc_score = 0.0
        velocity_score = 0.0
        min_ttc = float('inf')

        for obs in dynamic_obstacles:
            if obs is None or len(obs) < 2:
                continue
            ox, oy = obs[0], obs[1]
            # 动态障碍物速度（若提供）
            if len(obs) >= 4:
                ovx, ovy = obs[2], obs[3]
            else:
                ovx, ovy = 0.0, 0.0

            rel_pos = np.array([ox - robot_x, oy - robot_y], dtype=float)
            rel_vel = np.array([ovx - v_vec[0], ovy - v_vec[1]], dtype=float)
            pos_dist = float(np.linalg.norm(rel_pos))

            if pos_dist == 0:
                ttc = 0.0
            else:
                # 仅当相对速度沿接近方向时才计算有效 TTC
                closing_rate = -float(np.dot(rel_vel, rel_pos) / pos_dist)
                if closing_rate > 0:
                    ttc = pos_dist / closing_rate
                else:
                    ttc = float('inf')

            if ttc < min_ttc:
                min_ttc = ttc

            # 速度因子：高速时风险增加
            rel_speed = float(np.linalg.norm(rel_vel))
            if rel_speed > 1.0:
                velocity_score = min(velocity_score + rel_speed * 5.0, 20.0)

        if min_ttc < 1.0:
            ttc_score = 30.0
            recommendations.append("TTC小于1秒，立即避让或制动")
        elif min_ttc < 2.0:
            ttc_score = 15.0
            recommendations.append("TTC较短，准备避让")
        elif min_ttc < 3.0:
            ttc_score = 5.0
        else:
            ttc_score = 0.0

        factors["ttc_score"] = ttc_score
        factors["velocity_score"] = velocity_score
        score += ttc_score + velocity_score

        # ---- 因子3：路径曲率（基于速度方向与航向的差异） ----
        if speed > 1e-3:
            heading_vec = np.array([math.cos(robot_theta), math.sin(robot_theta)])
            v_dir = v_vec / speed
            # 方向偏差角（弧度）
            cos_err = float(np.clip(np.dot(heading_vec, v_dir), -1.0, 1.0))
            heading_err = abs(math.acos(cos_err))
            # 偏差越大，曲率风险越高
            curvature_score = min(heading_err / math.pi * 10.0, 10.0)
        else:
            curvature_score = 0.0
        factors["curvature_score"] = curvature_score
        score += curvature_score

        # 限制总评分在 0-100
        score = float(max(0.0, min(100.0, score)))

        if not recommendations:
            if score < 20.0:
                recommendations.append("状态安全，可正常运行")
            else:
                recommendations.append("保持关注，按需调整速度")

        return RiskAssessment(score=score, factors=factors, recommendations=recommendations)

    def _distance_to_obstacle(self, px: float, py: float,
                              obs: Tuple[float, ...]) -> float:
        """计算点到障碍物的最短距离

        参数:
            px, py: 点坐标
            obs: 障碍物，支持 (x, y, radius) 或 (xmin, ymin, xmax, ymax)

        返回:
            距离值（米），无匹配则返回 inf
        """
        if obs is None:
            return float('inf')
        if len(obs) == 3:
            ox, oy, orad = obs
            return math.hypot(px - ox, py - oy) - orad
        elif len(obs) == 4:
            xmin, ymin, xmax, ymax = obs
            dx = max(xmin - px, 0.0, px - xmax)
            dy = max(ymin - py, 0.0, py - ymax)
            return math.hypot(dx, dy)
        return float('inf')

    # ------------------------------------------------------------------
    # 4. 心跳检测
    # ------------------------------------------------------------------
    def register_heartbeat(self, node_name: str) -> None:
        """注册一个需要监控的节点

        参数:
            node_name: 节点名称
        """
        self.heartbeats[node_name] = time.time()
        self.log_event("heartbeat_registered", {"node": node_name})

    def update_heartbeat(self, node_name: str) -> bool:
        """更新节点的心跳时间戳

        参数:
            node_name: 节点名称

        返回:
            True 表示更新成功，False 表示节点未注册
        """
        if node_name not in self.heartbeats:
            return False
        self.heartbeats[node_name] = time.time()
        return True

    def check_heartbeats(self, timeout: float = 1.0) -> List[str]:
        """检查所有节点心跳是否超时

        参数:
            timeout: 超时阈值（秒）

        返回:
            超时节点名称列表
        """
        now = time.time()
        timed_out = []
        for node_name, last_ts in self.heartbeats.items():
            if now - last_ts > timeout:
                timed_out.append(node_name)
        if timed_out:
            self.log_event("heartbeat_timeout",
                           {"nodes": timed_out, "timeout": timeout})
        return timed_out

    # ------------------------------------------------------------------
    # 5. 恢复策略
    # ------------------------------------------------------------------
    def get_recovery_strategy(self, risk_level: int,
                              failure_type: Optional[str] = None) -> Dict[str, Any]:
        """获取恢复策略

        根据风险等级和故障类型返回恢复动作建议。

        参数:
            risk_level: 风险等级 0-3
            failure_type: 故障类型，可选值：
                          'sensor_failure'（传感器故障）
                          'localization_loss'（定位丢失）
                          None（无特定故障）

        返回:
            {'action': str, 'params': dict}
        """
        # 故障类型优先处理
        if failure_type == 'sensor_failure':
            return {
                'action': 'use_backup_sensor',
                'params': {'source': 'backup', ' degrade': False}
            }
        if failure_type == 'localization_loss':
            return {
                'action': 'global_relocalization',
                'params': {'method': 'AMCL', 'reset': True}
            }

        # 按风险等级返回策略
        if risk_level >= 3:
            return {
                'action': 'emergency_stop',
                'params': {'brake': True, 'clear_costmap': True}
            }
        elif risk_level == 2:
            return {
                'action': 'slow_down_and_replan',
                'params': {'speed_scale': 0.3, 'replan': True}
            }
        elif risk_level == 1:
            return {
                'action': 'cautious_approach',
                'params': {'speed_scale': 0.6, 'monitor': True}
            }
        else:
            return {
                'action': 'continue',
                'params': {'speed_scale': 1.0}
            }

    # ------------------------------------------------------------------
    # 6. 安全状态机
    # ------------------------------------------------------------------
    def update(self, robot_pose: Tuple[float, float, float],
               lidar_scan: List[Tuple[float, float]],
               velocity: Union[float, Tuple[float, float]],
               obstacles: List[Tuple[float, ...]]) -> SafetyState:
        """安全子系统主循环更新

        根据当前机器人状态、感知数据更新安全状态机。

        参数:
            robot_pose: 机器人位姿 (x, y, theta)
            lidar_scan: 激光雷达扫描数据 [(range, angle), ...]
            velocity: 机器人速度（标量或向量）
            obstacles: 障碍物列表（同时作为静态和动态障碍物处理）

        返回:
            更新后的安全状态
        """
        should_brake, brake_reason, risk_level = self.check_emergency_brake(
            robot_pose, lidar_scan, obstacles)

        risk = self.assess_risk(robot_pose, velocity, obstacles, obstacles)

        prev_state = self._state

        # 状态转换逻辑
        if should_brake or risk_level == 3:
            self._state = SafetyState.EMERGENCY_STOP
            self.log_event("state_transition", {
                "from": prev_state.name,
                "to": self._state.name,
                "reason": brake_reason,
            })
        elif risk_level == 2:
            self._state = SafetyState.DANGER
        elif risk_level == 1 or risk.score >= 30.0:
            self._state = SafetyState.WARNING
        elif risk.score >= 15.0:
            self._state = SafetyState.CAUTIOUS
        else:
            self._state = SafetyState.NORMAL

        if self._state != prev_state and self._state != SafetyState.EMERGENCY_STOP:
            self.log_event("state_transition", {
                "from": prev_state.name,
                "to": self._state.name,
                "risk_score": risk.score,
            })

        return self._state

    def get_current_state(self) -> SafetyState:
        """获取当前安全状态

        返回:
            当前 SafetyState
        """
        return self._state

    def get_safe_velocity(self, desired_velocity: Union[float, Tuple[float, float]]
                          ) -> Union[float, Tuple[float, float]]:
        """根据当前安全状态缩放期望速度

        参数:
            desired_velocity: 期望速度（标量或 (vx, vy) 向量）

        返回:
            缩放后的安全速度
        """
        scale = self._VELOCITY_SCALE.get(self._state, 0.0)
        if isinstance(desired_velocity, (tuple, list, np.ndarray)):
            v = np.asarray(desired_velocity, dtype=float)
            return tuple((v * scale).tolist())
        return float(desired_velocity) * scale

    # ------------------------------------------------------------------
    # 7. 事件日志
    # ------------------------------------------------------------------
    def log_event(self, event_type: str, details: Dict[str, Any]) -> None:
        """记录安全事件

        参数:
            event_type: 事件类型
            details: 事件详情
        """
        self.event_log.append({
            'timestamp': time.time(),
            'event_type': event_type,
            'details': details,
        })

    def get_event_log(self) -> List[Dict[str, Any]]:
        """获取事件日志

        返回:
            事件日志列表
        """
        return self.event_log

    def export_report(self, filename: str) -> None:
        """导出安全报告到 CSV 文件

        参数:
            filename: 输出 CSV 文件名
        """
        if not self.event_log:
            return

        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'event_type', 'details'])
            for event in self.event_log:
                writer.writerow([
                    event['timestamp'],
                    event['event_type'],
                    str(event['details']),
                ])
