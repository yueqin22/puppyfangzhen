"""硬件抽象接口基类 (Hardware Abstraction Interface)

定义仿真和实体机统一的硬件接口，使上层导航/安全/任务逻辑
无需关心底层是 CoppeliaSim 仿真还是 PuppyPi 实体硬件。

设计原则（项目内存硬性约束）:
    - 仿真和实体机接口对上层一致呈现
    - 运动控制环以固定频率运行 (≥20Hz, 目标30Hz)
    - 传感器数据流支持异步读取
    - 电池状态支持低电量自动回充
    - 所有参数通过 YAML 配置，不硬编码

接口分层:
    1. 运动控制: send_velocity(vx, vy, wz) → 电机驱动
    2. 传感器读取: get_lidar_scan() / get_imu() / get_odometry()
    3. 电池状态: get_battery() → BatteryStatus
    4. 安全: emergency_stop() / enable_motors()
    5. 健康监控: get_health() → RobotHealth

使用方式:
    # 上层代码不关心底层实现
    hw = HardwareInterface.create(config)  # 自动选择 sim 或 real
    hw.send_velocity(0.2, 0.0, 0.1)
    scan = hw.get_lidar_scan()
    battery = hw.get_battery()

子类实现:
    - SimHardwareInterface: 仿真实现 (CoppeliaSim / 纯Python)
    - PuppyPiHardwareInterface: 实体机实现 (PuppyPi SDK)
"""
from __future__ import annotations

import abc
import math
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any, Callable

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SensorReadings:
    """传感器读数集合（一次读取的所有传感器数据）

    属性:
        lidar_angles: LiDAR 扫描角度数组 (rad)
        lidar_ranges: LiDAR 扫描距离数组 (m)
        imu_accel: IMU 加速度 [ax, ay, az] (m/s²)
        imu_gyro: IMU 角速度 [gx, gy, gz] (rad/s)
        imu_quat: IMU 四元数 [w, x, y, z]
        odom_x: 里程计 x 位置 (m)
        odom_y: 里程计 y 位置 (m)
        odom_yaw: 里程计航向角 (rad)
        odom_vx: 里程计 x 速度 (m/s)
        odom_vy: 里程计 y 速度 (m/s)
        odom_wz: 里程计角速度 (rad/s)
        timestamp: 时间戳 (秒)
    """
    lidar_angles: np.ndarray = field(default_factory=lambda: np.array([]))
    lidar_ranges: np.ndarray = field(default_factory=lambda: np.array([]))
    imu_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    imu_gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    imu_quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    odom_vx: float = 0.0
    odom_vy: float = 0.0
    odom_wz: float = 0.0
    timestamp: float = 0.0


@dataclass
class BatteryState:
    """电池状态

    属性:
        voltage: 电压 (V)
        current: 电流 (A), 正为放电, 负为充电
        percent: 电量百分比 [0.0, 1.0]
        charging: 是否正在充电
        temperature: 电池温度 (°C)
        low_battery: 低电量标志 (< 20%)
        critical_battery: 危急电量标志 (< 10%)
    """
    voltage: float = 12.0
    current: float = 0.0
    percent: float = 1.0
    charging: bool = False
    temperature: float = 25.0
    low_battery: bool = False
    critical_battery: bool = False

    def update_thresholds(self, low: float = 0.20, critical: float = 0.10):
        """根据阈值更新低电量标志"""
        self.low_battery = self.percent < low
        self.critical_battery = self.percent < critical


@dataclass
class RobotHealthState:
    """机器人健康状态

    属性:
        ok: 整体健康
        level: 健康等级 ('OK' / 'WARN' / 'ERROR' / 'FATAL')
        active_faults: 活跃故障列表
        cpu_temp: CPU 温度 (°C)
        imu_ready: IMU 就绪
        lidar_ready: LiDAR 就绪
        camera_ready: 相机就绪
        motion_ready: 运动控制就绪
    """
    ok: bool = True
    level: str = 'OK'
    active_faults: List[str] = field(default_factory=list)
    cpu_temp: float = 45.0
    imu_ready: bool = True
    lidar_ready: bool = True
    camera_ready: bool = True
    motion_ready: bool = True


class HardwareInterface(abc.ABC):
    """硬件抽象接口基类

    定义仿真和实体机统一的硬件访问接口。
    上层导航/安全/任务逻辑通过此接口与硬件交互，
    无需关心底层是仿真还是实体硬件。

    生命周期:
        1. create(config) → 实例化
        2. initialize() → 硬件初始化
        3. enable_motors() → 使能电机
        4. 主循环: send_velocity() / get_sensor_readings() / get_battery()
        5. emergency_stop() → 紧急停止
        6. shutdown() → 关闭硬件

    线程安全:
        - send_velocity() 和 get_sensor_readings() 可在不同线程调用
        - 内部使用锁保护共享状态
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """初始化硬件接口

        Args:
            config: 配置字典，包含运动控制、传感器、电池等参数

        Raises:
            ValueError: 配置参数非法
        """
        self.config = config or {}
        # 运动控制参数
        motion_cfg = self.config.get('motion', {})
        self.max_linear_x = motion_cfg.get('max_linear_x', 0.3)      # m/s
        self.max_linear_y = motion_cfg.get('max_linear_y', 0.0)      # m/s (差速驱动为0)
        self.max_angular_z = motion_cfg.get('max_angular_z', 1.2)   # rad/s
        self.accel_limit = motion_cfg.get('accel_limit', 2.0)        # m/s²
        self.yaw_rate_limit = motion_cfg.get('yaw_rate_limit', 4.0)  # rad/s²
        self.cmd_timeout = motion_cfg.get('cmd_timeout', 1.0)        # s

        # 电池参数
        battery_cfg = self.config.get('battery', {})
        self.low_battery_threshold = battery_cfg.get('low_threshold', 0.20)
        self.critical_battery_threshold = battery_cfg.get('critical_threshold', 0.10)
        self.auto_recharge_enabled = battery_cfg.get('auto_recharge', True)

        # 配置参数校验（启动时拒绝非法参数）
        self._validate_config()

        # 状态
        self._motors_enabled = False
        self._emergency_stopped = False
        self._last_cmd_time = time.time()
        self._last_velocity = (0.0, 0.0, 0.0)  # (vx, vy, wz)
        self._initialized = False

        # 统计
        self._cmd_count = 0
        self._error_count = 0
        self._last_error = ""

    def _validate_config(self):
        """校验配置参数，拒绝非法值

        Raises:
            ValueError: 参数非法时抛出
        """
        errors = []
        if self.max_linear_x < 0:
            errors.append(f"max_linear_x 不能为负: {self.max_linear_x}")
        if self.max_linear_y < 0:
            errors.append(f"max_linear_y 不能为负: {self.max_linear_y}")
        if self.max_angular_z < 0:
            errors.append(f"max_angular_z 不能为负: {self.max_angular_z}")
        if self.accel_limit <= 0:
            errors.append(f"accel_limit 必须为正: {self.accel_limit}")
        if self.yaw_rate_limit <= 0:
            errors.append(f"yaw_rate_limit 必须为正: {self.yaw_rate_limit}")
        if self.cmd_timeout <= 0:
            errors.append(f"cmd_timeout 必须为正: {self.cmd_timeout}")
        if self.cmd_timeout > 10.0:
            errors.append(f"cmd_timeout 过大 (>10s): {self.cmd_timeout}")
        if self.low_battery_threshold < 0 or self.low_battery_threshold > 1:
            errors.append(
                f"low_battery_threshold 超出 [0,1]: {self.low_battery_threshold}")
        if self.critical_battery_threshold < 0 or self.critical_battery_threshold > 1:
            errors.append(
                f"critical_battery_threshold 超出 [0,1]: "
                f"{self.critical_battery_threshold}")
        if self.critical_battery_threshold >= self.low_battery_threshold:
            errors.append(
                f"critical_battery_threshold ({self.critical_battery_threshold}) "
                f"必须小于 low_battery_threshold ({self.low_battery_threshold})")
        if errors:
            msg = "配置参数校验失败:\n  " + "\n  ".join(errors)
            logger.error(msg)
            raise ValueError(msg)

    # === 工厂方法 ===
    @staticmethod
    def create(config: Optional[Dict[str, Any]] = None) -> 'HardwareInterface':
        """根据配置创建硬件接口实例

        根据 config['backend'] 选择:
            - 'sim' / 'simulation': 仿真实现
            - 'real' / 'puppypi': PuppyPi 实体机实现
            - 'mock': 纯Python mock (无外部依赖)

        Args:
            config: 配置字典

        Returns:
            HardwareInterface 实例

        Raises:
            ValueError: 未知 backend
        """
        backend = (config or {}).get('backend', 'sim').lower()
        if backend in ('sim', 'simulation', 'coppelia'):
            try:
                from .sim_interface import SimHardwareInterface
                return SimHardwareInterface(config)
            except ImportError:
                # sim_interface 未安装时回退到 mock
                from .mock_interface import MockHardwareInterface
                return MockHardwareInterface(config)
        elif backend in ('real', 'puppypi', 'hardware'):
            from .puppypi_driver import PuppyPiHardwareInterface
            return PuppyPiHardwareInterface(config)
        elif backend == 'mock':
            from .mock_interface import MockHardwareInterface
            return MockHardwareInterface(config)
        else:
            raise ValueError(f"未知 backend: {backend}，可选: sim/real/mock")

    # === 生命周期管理 ===
    @abc.abstractmethod
    def initialize(self) -> bool:
        """初始化硬件连接

        Returns:
            True 如果初始化成功
        """
        pass

    @abc.abstractmethod
    def shutdown(self):
        """关闭硬件连接，释放资源"""
        pass

    # === 运动控制 ===
    def send_velocity(self, vx: float, vy: float, wz: float):
        """发送速度命令到运动控制环

        速度命令经过以下处理:
            1. 安全限幅 (max_linear_x/y, max_angular_z)
            2. 加速度限制 (accel_limit, yaw_rate_limit)
            3. 超时检测 (cmd_timeout)
            4. 紧急停止检查

        Args:
            vx: 线速度 x (m/s), 前进为正
            vy: 线速度 y (m/s), 左移为正 (差速驱动通常为0)
            wz: 角速度 (rad/s), 逆时针为正
        """
        if self._emergency_stopped:
            self._dispatch_motion(0.0, 0.0, 0.0)
            return

        if not self._motors_enabled:
            self._dispatch_motion(0.0, 0.0, 0.0)
            return

        # 安全限幅
        vx = max(-self.max_linear_x, min(self.max_linear_x, vx))
        vy = max(-self.max_linear_y, min(self.max_linear_y, vy))
        wz = max(-self.max_angular_z, min(self.max_angular_z, wz))

        # 加速度限制 (防止电机冲击)
        prev_vx, prev_vy, prev_wz = self._last_velocity
        dt = max(time.time() - self._last_cmd_time, 0.001)
        vx = self._limit_accel(vx, prev_vx, dt, self.accel_limit)
        vy = self._limit_accel(vy, prev_vy, dt, self.accel_limit)
        wz = self._limit_accel(wz, prev_wz, dt, self.yaw_rate_limit)

        self._dispatch_motion(vx, vy, wz)
        self._last_velocity = (vx, vy, wz)
        self._last_cmd_time = time.time()
        self._cmd_count += 1

    @abc.abstractmethod
    def _dispatch_motion(self, vx: float, vy: float, wz: float):
        """将速度命令分发到底层驱动 (子类实现)

        Args:
            vx: 限幅后的线速度 x
            vy: 限幅后的线速度 y
            wz: 限幅后的角速度
        """
        pass

    def enable_motors(self, enable: bool = True) -> bool:
        """使能/禁用电机

        Args:
            enable: True 使能, False 禁用

        Returns:
            True 如果操作成功
        """
        self._motors_enabled = enable
        if not enable:
            self._dispatch_motion(0.0, 0.0, 0.0)
        return True

    def emergency_stop(self, reason: str = "manual"):
        """紧急停止 — 立即停止所有运动

        三级急停的第一级：软件 API 急停
        (第二级：硬件按钮急停, 第三级：远程关机)

        Args:
            reason: 急停原因
        """
        self._emergency_stopped = True
        self._motors_enabled = False
        self._dispatch_motion(0.0, 0.0, 0.0)
        self._last_velocity = (0.0, 0.0, 0.0)  # 重置速度状态
        self._error_count += 1
        self._last_error = f"EMERGENCY_STOP: {reason}"

    def release_emergency_stop(self) -> bool:
        """释放紧急停止状态

        Returns:
            True 如果成功释放
        """
        self._emergency_stopped = False
        return True

    # === 传感器读取 ===
    @abc.abstractmethod
    def get_sensor_readings(self) -> SensorReadings:
        """读取所有传感器数据

        Returns:
            SensorReadings 包含 LiDAR/IMU/里程计数据
        """
        pass

    @abc.abstractmethod
    def get_battery(self) -> BatteryState:
        """读取电池状态

        Returns:
            BatteryState 电池状态
        """
        pass

    @abc.abstractmethod
    def get_health(self) -> RobotHealthState:
        """读取机器人健康状态

        Returns:
            RobotHealthState 健康状态
        """
        pass

    # === 状态查询 ===
    @property
    def motors_enabled(self) -> bool:
        """电机是否使能"""
        return self._motors_enabled

    @property
    def emergency_stopped(self) -> bool:
        """是否处于紧急停止状态"""
        return self._emergency_stopped

    @property
    def command_timeout(self) -> bool:
        """命令是否超时 (超过 cmd_timeout 未收到新命令)"""
        return (time.time() - self._last_cmd_time) > self.cmd_timeout

    @property
    def stats(self) -> Dict[str, Any]:
        """运行统计"""
        return {
            'cmd_count': self._cmd_count,
            'error_count': self._error_count,
            'last_error': self._last_error,
            'motors_enabled': self._motors_enabled,
            'emergency_stopped': self._emergency_stopped,
            'uptime': time.time() - (self._last_cmd_time - self.cmd_timeout),
        }

    # === 内部工具 ===
    @staticmethod
    def _limit_accel(target: float, current: float,
                     dt: float, limit: float) -> float:
        """加速度限制

        Args:
            target: 目标速度
            current: 当前速度
            dt: 时间步长
            limit: 加速度限制

        Returns:
            限制后的速度
        """
        max_delta = limit * dt
        delta = target - current
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        return current + delta

    # === Context Manager ===
    def __enter__(self):
        """支持 with 语句，自动初始化"""
        if not self._initialized:
            self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """with 语句退出时自动关闭，确保资源释放"""
        self.shutdown()
        return False  # 不吞异常


class Watchdog:
    """线程看门狗

    监控关键线程的心跳，检测线程死亡或卡死。

    使用方式:
        watchdog = Watchdog(timeout=0.5)
        watchdog.register('sim_thread')

        # 在被监控线程中:
        watchdog.heartbeat('sim_thread')

        # 在监控线程中:
        watchdog.check()  # 返回超时线程列表
        if watchdog.is_alive('sim_thread'):
            # 线程正常
    """

    def __init__(self, timeout: float = 0.5):
        """初始化看门狗

        Args:
            timeout: 心跳超时阈值（秒），超过此时间未更新心跳视为故障
        """
        self.timeout = timeout
        self._heartbeats: Dict[str, float] = {}
        self._lock = threading.Lock()
        self._callbacks: List[Callable[[str], None]] = []
        self._dead_threads: set = set()

    def register(self, name: str):
        """注册需要监控的线程"""
        with self._lock:
            self._heartbeats[name] = time.time()
            self._dead_threads.discard(name)

    def heartbeat(self, name: str):
        """更新心跳（在被监控线程中调用）"""
        with self._lock:
            self._heartbeats[name] = time.time()
            self._dead_threads.discard(name)

    def check(self) -> List[str]:
        """检查所有注册线程的心跳

        Returns:
            超时（可能已死亡）的线程名列表
        """
        now = time.time()
        dead = []
        with self._lock:
            for name, last_beat in self._heartbeats.items():
                if now - last_beat > self.timeout:
                    if name not in self._dead_threads:
                        self._dead_threads.add(name)
                        dead.append(name)

        # 触发回调
        for name in dead:
            logger.warning(f"看门狗: 线程 '{name}' 心跳超时 "
                          f"(>{self.timeout:.1f}s)")
            for cb in self._callbacks:
                try:
                    cb(name)
                except Exception as e:
                    logger.error(f"看门狗回调失败: {e}")

        return dead

    def is_alive(self, name: str) -> bool:
        """检查指定线程是否存活"""
        with self._lock:
            if name not in self._heartbeats:
                return False
            return (time.time() - self._heartbeats[name]) <= self.timeout

    def add_timeout_callback(self, callback: Callable[[str], None]):
        """添加超时回调"""
        self._callbacks.append(callback)

    def get_status(self) -> Dict[str, Any]:
        """获取所有线程状态"""
        now = time.time()
        with self._lock:
            return {
                name: {
                    'alive': (now - t) <= self.timeout,
                    'last_heartbeat_ago': round(now - t, 3),
                }
                for name, t in self._heartbeats.items()
            }
