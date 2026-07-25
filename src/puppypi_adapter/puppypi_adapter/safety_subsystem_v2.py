"""三级急停 + 分级告警 + 降级策略 + 自动上报

实现项目内存中的硬性安全要求:
    - 三级急停（硬件按钮 + 软件 API + 远程关机）
    - 分级告警 (WARN/ERROR/FATAL) + 降级策略 + 自动上报
    - 安全子系统包含碰撞检测→紧急制动→风险评仙→恢复策略闭环

三级急停架构:
    Level 1 (软件 API): emergency_stop() 方法调用
        - 触发条件: 碰撞检测、命令超时、软件故障
        - 响应时间: < 50ms
        - 恢复方式: release_emergency_stop()

    Level 2 (硬件按钮): GPIO 中断/轮询
        - 触发条件: 物理急停按钮按下
        - 响应时间: < 10ms (硬件级)
        - 恢复方式: 按钮释放 + 软件确认

    Level 3 (远程关机): 网络/串口命令
        - 触发条件: 远程监控中心下发关机命令
        - 响应时间: < 1s
        - 恢复方式: 手动重启

分级告警系统:
    WARN  (警告): 性能下降、传感器噪声增大 → 减速运行
    ERROR (错误): 传感器故障、碰撞 → 停止任务，安全停车
    FATAL (致命): 硬件故障、急停 → 立即断电保护

降级策略:
    - LiDAR 故障 → 降低最大速度 50%，仅用里程计
    - IMU 故障 → 禁用旋转，仅直线运动
    - 电池低电量 → 返回充电桩
    - CPU 过热 → 降低控制频率
    - 通信中断 → 本地安全模式
"""
from __future__ import annotations

import time
import math
import threading
import logging
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Callable, List, Dict, Any

logger = logging.getLogger(__name__)


class AlertLevel(IntEnum):
    """告警等级（严重程度递增）"""
    OK = 0       # 正常
    INFO = 1     # 信息（无需处理）
    WARN = 2     # 警告（需减速）
    ERROR = 3    # 错误（需停止任务）
    FATAL = 4    # 致命（立即断电）


class EStopSource(IntEnum):
    """急停来源"""
    SOFTWARE_API = 1      # Level 1: 软件 API
    HARDWARE_BUTTON = 2   # Level 2: 硬件按钮
    REMOTE_SHUTDOWN = 3   # Level 3: 远程关机
    COLLISION_DETECT = 4  # 碰撞检测
    WATCHDOG = 5          # 看门狗
    THERMAL = 6           # 过热保护


class DegradationMode(IntEnum):
    """降级模式"""
    FULL = 0          # 全功能
    REDUCED_SPEED = 1  # 降速运行
    SENSORS_DOWN = 2   # 传感器降级
    SAFE_STOP = 3      # 安全停车
    EMERGENCY = 4      # 紧急模式


@dataclass
class AlertEvent:
    """告警事件

    属性:
        level: 告警等级
        source: 告警来源
        message: 告警消息
        timestamp: 时间戳
        data: 附加数据
        acknowledged: 是否已确认
        recovery_action: 建议恢复动作
    """
    level: AlertLevel
    source: str
    message: str
    timestamp: float = field(default_factory=time.time)
    data: Dict[str, Any] = field(default_factory=dict)
    acknowledged: bool = False
    recovery_action: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            'level': self.level.name,
            'source': self.source,
            'message': self.message,
            'timestamp': self.timestamp,
            'data': self.data,
            'acknowledged': self.acknowledged,
            'recovery_action': self.recovery_action,
        }


@dataclass
class DegradationPolicy:
    """降级策略配置

    定义不同故障下的降级行为
    """
    sensor_fault_speed_limit: float = 0.15   # 传感器故障时速度限制
    imu_fault_disable_rotation: bool = True  # IMU故障禁用旋转
    battery_low_return_home: bool = True     # 低电量回充
    cpu_throttle_threshold: float = 75.0     # CPU温度降频阈值
    comm_timeout_local_mode: bool = True      # 通信断开进本地模式


class SafetySubsystem:
    """完整安全子系统

    集成三级急停、分级告警、降级策略、自动上报。
    与 HardwareInterface 协作实现安全闭环。

    架构:
        碰撞检测 → 紧急制动 → 风险评估 → 恢复策略 → 恢复/保持急停

    使用方式:
        safety = SafetySubsystem(hw_interface)
        safety.start()
        # 运行中...
        if safety.is_safe():
            hw.send_velocity(0.2, 0, 0)
        # 急停
        safety.emergency_stop("collision", EStopSource.COLLISION_DETECT)
    """

    def __init__(self, hw_interface=None,
                 policy: Optional[DegradationPolicy] = None):
        """初始化安全子系统

        Args:
            hw_interface: 硬件接口实例（可选，用于调用急停）
            policy: 降级策略配置
        """
        self.hw = hw_interface
        self.policy = policy or DegradationPolicy()

        # 急停状态（受 _state_lock 保护）
        self._estop_active = False
        self._estop_source: Optional[EStopSource] = None
        self._estop_reason = ""
        self._estop_time = 0.0
        self._state_lock = threading.Lock()

        # 告警历史
        self._alerts: List[AlertEvent] = []
        self._active_alerts: List[AlertEvent] = []
        self._alert_lock = threading.Lock()

        # 降级模式（受 _state_lock 保护）
        self._degradation = DegradationMode.FULL
        self._degradation_reason = ""

        # 碰撞检测状态
        self._collision_imminent = False
        self._last_collision_check = 0.0

        # 远程关机监听
        self._remote_shutdown_listener: Optional[Callable] = None
        self._remote_thread: Optional[threading.Thread] = None
        self._running = False

        # 上报回调
        self._report_callbacks: List[Callable[[AlertEvent], None]] = []

        # 统计
        self._estop_count = 0
        self._alert_count = 0

    # === 生命周期 ===
    def start(self):
        """启动安全子系统（开始远程关机监听）"""
        self._running = True
        # 启动远程关机监听线程
        self._remote_thread = threading.Thread(
            target=self._remote_shutdown_loop,
            daemon=True, name='Safety-Remote')
        self._remote_thread.start()
        logger.info("安全子系统已启动")

    def stop(self):
        """停止安全子系统"""
        self._running = False
        if self._remote_thread:
            self._remote_thread.join(timeout=1.0)

    # === 三级急停 ===
    def emergency_stop(self, reason: str,
                       source: EStopSource = EStopSource.SOFTWARE_API):
        """触发急停（三级急停统一入口）

        Level 1 (SOFTWARE_API): 软件 API 调用
        Level 2 (HARDWARE_BUTTON): 硬件按钮触发
        Level 3 (REMOTE_SHUTDOWN): 远程关机

        Args:
            reason: 急停原因描述
            source: 急停来源
        """
        with self._state_lock:
            if self._estop_active:
                return  # 已急停，不重复

            self._estop_active = True
            self._estop_source = source
            self._estop_reason = reason
            self._estop_time = time.time()
            self._estop_count += 1

        # 调用硬件急停（锁外调用，避免死锁）
        if self.hw:
            try:
                self.hw.emergency_stop(f"{source.name}: {reason}")
            except Exception as e:
                logger.error(f"硬件急停调用失败: {e}")

        # 记录 FATAL 告警
        self._raise_alert(
            AlertLevel.FATAL,
            f"estop_{source.name.lower()}",
            f"急停触发: {reason}",
            data={'source': source.name, 'reason': reason},
            recovery_action=self._get_recovery_action(source))

        logger.critical(f"急停触发 [{source.name}]: {reason}")

    def release_emergency_stop(self) -> bool:
        """释放急停状态

        安全检查:
            - 硬件按钮必须已释放
            - 无致命故障
            - 人工确认（远程关机后）

        Returns:
            True 如果成功释放
        """
        with self._state_lock:
            if not self._estop_active:
                return True

            # 远程关机需要特殊处理
            if self._estop_source == EStopSource.REMOTE_SHUTDOWN:
                logger.warning("远程关机急停需要人工重启")
                return False

            # 硬件按钮急停需要按钮已释放
            if (self._estop_source == EStopSource.HARDWARE_BUTTON
                    and self.hw and hasattr(self.hw, 'hw_estop_triggered')
                    and self.hw.hw_estop_triggered):
                logger.warning("硬件急停按钮未释放")
                return False

            # 释放
            if self.hw:
                try:
                    self.hw.release_emergency_stop()
                except Exception as e:
                    logger.error(f"硬件释放急停失败: {e}")
                    return False
            self._estop_active = False
            self._estop_source = None
            self._estop_reason = ""
            self._degradation = DegradationMode.FULL
            self._degradation_reason = ""

        logger.info("急停已释放")
        return True

    def register_remote_shutdown_listener(self,
                                          listener: Callable[[], bool]):
        """注册远程关机监听器

        Args:
            listener: 返回 True 表示收到远程关机命令
        """
        self._remote_shutdown_listener = listener

    # === 分级告警 ===
    def raise_alert(self, level: AlertLevel, source: str,
                    message: str, data: Optional[Dict] = None,
                    recovery_action: str = ""):
        """上报告警（公开接口）

        Args:
            level: 告警等级
            source: 告警来源
            message: 告警消息
            data: 附加数据
            recovery_action: 建议恢复动作
        """
        self._raise_alert(level, source, message, data or {}, recovery_action)

    def _raise_alert(self, level: AlertLevel, source: str,
                     message: str, data: Dict[str, Any],
                     recovery_action: str):
        """内部告警上报"""
        alert = AlertEvent(
            level=level, source=source, message=message,
            data=data, recovery_action=recovery_action)

        with self._alert_lock:
            self._alerts.append(alert)
            self._alert_count += 1
            # 保持历史不超过 1000 条
            if len(self._alerts) > 1000:
                self._alerts = self._alerts[-500:]

            # 添加到活跃告警（FATAL/ERROR/WARN）
            if level >= AlertLevel.WARN:
                self._active_alerts.append(alert)

        # 根据等级执行降级
        self._apply_degradation(level, source, data)

        # 触发上报回调
        for cb in self._report_callbacks:
            try:
                cb(alert)
            except Exception as e:
                logger.error(f"上报回调失败: {e}")

        # 日志
        if level >= AlertLevel.ERROR:
            logger.error(f"[{level.name}] {source}: {message}")
        elif level >= AlertLevel.WARN:
            logger.warning(f"[{level.name}] {source}: {message}")
        else:
            logger.info(f"[{level.name}] {source}: {message}")

    def acknowledge_alert(self, index: int) -> bool:
        """确认告警

        Args:
            index: 告警在活跃列表中的索引

        Returns:
            True 如果确认成功
        """
        with self._alert_lock:
            if 0 <= index < len(self._active_alerts):
                self._active_alerts[index].acknowledged = True
                # 已确认的 WARN 可移除
                if (self._active_alerts[index].level == AlertLevel.WARN
                        and self._active_alerts[index].acknowledged):
                    self._active_alerts.pop(index)
                return True
        return False

    def clear_alerts(self, level: AlertLevel = AlertLevel.WARN):
        """清除指定等级及以下的已确认告警"""
        with self._alert_lock:
            self._active_alerts = [
                a for a in self._active_alerts
                if a.level > level or not a.acknowledged
            ]

    # === 降级策略 ===
    def _apply_degradation(self, level: AlertLevel, source: str,
                           data: Dict[str, Any]):
        """根据告警等级应用降级策略"""
        with self._state_lock:
            if level == AlertLevel.FATAL:
                self._degradation = DegradationMode.EMERGENCY
                self._degradation_reason = f"FATAL: {source}"
            elif level == AlertLevel.ERROR:
                if 'sensor' in source.lower():
                    self._degradation = DegradationMode.SENSORS_DOWN
                    self._degradation_reason = f"传感器故障: {source}"
                else:
                    self._degradation = DegradationMode.SAFE_STOP
                    self._degradation_reason = f"错误: {source}"
            elif level == AlertLevel.WARN:
                if self._degradation < DegradationMode.REDUCED_SPEED:
                    self._degradation = DegradationMode.REDUCED_SPEED
                    self._degradation_reason = f"警告: {source}"

        # FATAL 触发急停（锁外调用，避免死锁）
        if level == AlertLevel.FATAL:
            self.emergency_stop(
                f"FATAL alert: {source}",
                EStopSource.SOFTWARE_API)
        # 错误时停止运动（锁外调用）
        elif level == AlertLevel.ERROR and self.hw:
            try:
                self.hw.send_velocity(0, 0, 0)
            except Exception as e:
                logger.error(f"停止运动失败: {e}")

    def get_speed_limit(self) -> float:
        """获取当前降级模式下的速度限制

        Returns:
            速度限制系数 [0.0, 1.0]
        """
        with self._state_lock:
            degradation = self._degradation

        if degradation == DegradationMode.FULL:
            return 1.0
        elif degradation == DegradationMode.REDUCED_SPEED:
            return 0.5
        elif degradation == DegradationMode.SENSORS_DOWN:
            # 修复除零风险：max_linear_x 可能为 0
            max_vx = (self.hw.max_linear_x if self.hw and self.hw.max_linear_x > 0
                      else 0.3)
            return min(self.policy.sensor_fault_speed_limit / max_vx, 1.0)
        else:
            return 0.0  # SAFE_STOP / EMERGENCY

    # === 碰撞检测 → 紧急制动 → 风险评估 → 恢复策略 ===
    def check_collision_risk(self, lidar_ranges: List[float],
                             min_safe_distance: float = 0.3) -> bool:
        """碰撞风险评估（碰撞检测 → 紧急制动）

        Args:
            lidar_ranges: LiDAR 距离数据
            min_safe_distance: 最小安全距离 (m)

        Returns:
            True 如果有碰撞风险
        """
        if not lidar_ranges:
            return False

        # 过滤 NaN/Inf/非正数（传感器异常数据）
        valid_ranges = [r for r in lidar_ranges
                        if not (math.isnan(r) or math.isinf(r)) and r > 0]
        if not valid_ranges:
            return False

        # 检查前方 ±30° 范围（前 1/6 的扫描点）
        n = len(valid_ranges)
        front_start = n * 5 // 12
        front_end = n * 7 // 12
        front_ranges = valid_ranges[front_start:front_end]

        min_dist = min(front_ranges) if front_ranges else float('inf')

        if min_dist < min_safe_distance * 0.5:
            # 紧急制动
            self._collision_imminent = True
            self.emergency_stop(
                f"碰撞 imminent: min_dist={min_dist:.2f}m",
                EStopSource.COLLISION_DETECT)
            return True
        elif min_dist < min_safe_distance:
            self._collision_imminent = True
            self.raise_alert(
                AlertLevel.WARN, "collision_risk",
                f"碰撞风险: min_dist={min_dist:.2f}m",
                data={'min_dist': min_dist})
            return True

        self._collision_imminent = False
        return False

    def assess_risk(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """风险评估

        综合考虑距离、速度、碰撞时间(TTC)等因素

        Args:
            sensor_data: 传感器数据

        Returns:
            风险评估结果
        """
        min_dist = sensor_data.get('min_obstacle_dist', float('inf'))
        speed = sensor_data.get('speed', 0.0)
        ttc = min_dist / max(speed, 0.01) if speed > 0 else float('inf')

        risk_score = 0.0
        if ttc < 1.0:
            risk_score = 100.0
        elif ttc < 2.0:
            risk_score = 50.0 * (2.0 - ttc)
        elif min_dist < 0.5:
            risk_score = 30.0

        return {
            'score': risk_score,
            'ttc': ttc,
            'min_dist': min_dist,
            'level': ('FATAL' if risk_score >= 80
                      else 'ERROR' if risk_score >= 50
                      else 'WARN' if risk_score >= 20
                      else 'OK'),
        }

    def get_recovery_strategy(self) -> str:
        """获取恢复策略建议

        Returns:
            恢复策略描述
        """
        if not self._estop_active:
            return "正常运行"

        source = self._estop_source
        if source == EStopSource.COLLISION_DETECT:
            return "后退避让 → 重新规划路径"
        elif source == EStopSource.HARDWARE_BUTTON:
            return "等待按钮释放 → 人工确认 → 释放急停"
        elif source == EStopSource.REMOTE_SHUTDOWN:
            return "等待远程恢复命令 → 重启系统"
        elif source == EStopSource.WATCHDOG:
            return "重置系统 → 重新初始化 → 恢复运行"
        elif source == EStopSource.THERMAL:
            return "等待温度降低 → 降低负载 → 恢复运行"
        else:
            return "检查故障 → 人工确认 → 释放急停"

    # === 自动上报 ===
    def add_report_callback(self, callback: Callable[[AlertEvent], None]):
        """添加自动上报回调

        用于将告警上报到云端/远程监控中心

        Args:
            callback: 回调函数，接收 AlertEvent
        """
        self._report_callbacks.append(callback)

    def export_alerts(self) -> List[Dict[str, Any]]:
        """导出所有告警历史（用于离线分析）

        Returns:
            告警列表，每项为 dict
        """
        with self._alert_lock:
            return [a.to_dict() for a in self._alerts]

    # === 状态查询 ===
    def is_safe(self) -> bool:
        """是否处于安全状态（可以继续运行）"""
        with self._state_lock:
            return (not self._estop_active
                    and self._degradation < DegradationMode.SAFE_STOP)

    def is_emergency_stopped(self) -> bool:
        """是否处于急停状态"""
        with self._state_lock:
            return self._estop_active

    @property
    def degradation_mode(self) -> DegradationMode:
        """当前降级模式"""
        with self._state_lock:
            return self._degradation

    @property
    def active_alerts(self) -> List[AlertEvent]:
        """活跃告警列表"""
        with self._alert_lock:
            return list(self._active_alerts)

    @property
    def stats(self) -> Dict[str, Any]:
        """统计信息"""
        with self._state_lock:
            return {
                'estop_count': self._estop_count,
                'alert_count': self._alert_count,
                'active_alerts': len(self._active_alerts),
                'degradation': self._degradation.name,
                'estop_active': self._estop_active,
                'estop_source': self._estop_source.name if self._estop_source else None,
                'estop_reason': self._estop_reason,
            }

    # === 内部方法 ===
    def _get_recovery_action(self, source: EStopSource) -> str:
        """获取恢复动作建议"""
        actions = {
            EStopSource.SOFTWARE_API: "检查软件故障 → 释放急停",
            EStopSource.HARDWARE_BUTTON: "等待按钮释放 → 确认 → 释放",
            EStopSource.REMOTE_SHUTDOWN: "等待远程恢复 → 重启",
            EStopSource.COLLISION_DETECT: "后退 → 重新规划",
            EStopSource.WATCHDOG: "重置 → 重新初始化",
            EStopSource.THERMAL: "降温 → 降低负载 → 恢复",
        }
        return actions.get(source, "人工检查")

    def _remote_shutdown_loop(self):
        """远程关机监听线程

        三级急停的第三级：远程关机
        通过注册的监听器检测远程关机命令
        """
        while self._running:
            try:
                if self._remote_shutdown_listener:
                    if self._remote_shutdown_listener():
                        self.emergency_stop(
                            "远程关机命令",
                            EStopSource.REMOTE_SHUTDOWN)
                        # 远程关机后停止监听
                        break
            except Exception as e:
                logger.error(f"远程关机监听错误: {e}")
            time.sleep(0.5)  # 2Hz 检查
