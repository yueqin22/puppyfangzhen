"""故障注入测试框架 (Fault Injection Testing)

实现项目内存中的硬性要求:
    - 故障注入测试 (模拟传感器故障、通信中断、CPU过载)

故障类型:
    1. 传感器故障:
       - LiDAR 超时/噪声/数据丢失
       - IMU 漂移/零偏/饱和
       - 相机模糊/曝光异常
       - 编码器跳变

    2. 通信中断:
       - DDS 通信延迟
       - 消息丢失
       - 节点崩溃

    3. CPU 过载:
       - 计算延迟激增
       - 内存泄漏模拟
       - 线程饥饿

    4. 环境干扰:
       - 光照突变
       - 地面材质变化
       - 玻璃表面反射

故障注入模型:
    - 持续时间: 瞬时(<100ms) / 短时(100ms-1s) / 持续(>1s)
    - 严重程度: 轻微 / 中等 / 严重
    - 触发方式: 定时 / 随机 / 事件驱动

使用方式:
    injector = FaultInjector(hw_interface)
    injector.inject(FaultType.LIDAR_TIMEOUT, duration=2.0)
    # 运行被测系统...
    injector.clear(FaultType.LIDAR_TIMEOUT)
"""
from __future__ import annotations

import time
import threading
import random
import logging
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Callable, List, Dict, Any

import numpy as np

logger = logging.getLogger(__name__)


class FaultType(IntEnum):
    """故障类型"""
    # 传感器故障
    LIDAR_TIMEOUT = 1        # LiDAR 超时
    LIDAR_NOISE = 2          # LiDAR 噪声增大
    LIDAR_DATA_LOSS = 3      # LiDAR 数据丢失
    IMU_DRIFT = 4            # IMU 零偏漂移
    IMU_SATURATION = 5       # IMU 饱和
    CAMERA_BLUR = 6          # 相机模糊
    ENCODER_JUMP = 7         # 编码器跳变

    # 通信故障
    COMM_LATENCY = 10        # 通信延迟
    COMM_PACKET_LOSS = 11    # 消息丢失
    NODE_CRASH = 12          # 节点崩溃

    # 性能故障
    CPU_OVERLOAD = 20        # CPU 过载
    MEMORY_LEAK = 21         # 内存泄漏
    THREAD_STARVATION = 22   # 线程饥饿

    # 环境干扰
    LIGHT_CHANGE = 30        # 光照突变
    GLASS_SURFACE = 31       # 玻璃表面
    NARROW_CORRIDOR = 32     # 窄通道


class FaultSeverity(IntEnum):
    """故障严重程度"""
    MINOR = 1     # 轻微：系统可正常运行，性能略降
    MODERATE = 2  # 中等：触发降级，但可继续运行
    SEVERE = 3    # 严重：触发急停或安全停车


@dataclass
class FaultInjection:
    """故障注入配置

    属性:
        fault_type: 故障类型
        severity: 严重程度
        duration: 持续时间 (秒)，None 表示持续直到清除
        start_delay: 启动延迟 (秒)
        parameters: 故障特定参数
    """
    fault_type: FaultType
    severity: FaultSeverity = FaultSeverity.MODERATE
    duration: Optional[float] = None  # None = 持续
    start_delay: float = 0.0
    parameters: Dict[str, Any] = field(default_factory=dict)

    # 运行时状态
    _start_time: float = field(default=0.0, repr=False)
    _active: bool = field(default=False, repr=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'fault_type': self.fault_type.name,
            'severity': self.severity.name,
            'duration': self.duration,
            'start_delay': self.start_delay,
            'parameters': self.parameters,
            'active': self._active,
        }


class FaultInjector:
    """故障注入测试框架

    向 HardwareInterface 注入各类故障，测试系统的鲁棒性。

    工作原理:
        1. 包装 HardwareInterface 的传感器读取方法
        2. 根据故障配置修改传感器数据
        3. 记录系统对故障的响应
        4. 评估恢复能力

    使用方式:
        hw = HardwareInterface.create(config)
        injector = FaultInjector(hw)

        # 注入 LiDAR 超时故障 2 秒
        injector.inject(FaultType.LIDAR_TIMEOUT, duration=2.0)

        # 注入 IMU 漂移
        injector.inject(FaultType.IMU_DRIFT, severity=FaultSeverity.SEVERE,
                        parameters={'drift_rate': 0.1})

        # 读取传感器（返回故障数据）
        readings = injector.get_sensor_readings()

        # 清除所有故障
        injector.clear_all()
    """

    def __init__(self, hw_interface=None):
        """初始化故障注入器

        Args:
            hw_interface: 硬件接口实例
        """
        self.hw = hw_interface
        self._faults: Dict[FaultType, FaultInjection] = {}
        self._lock = threading.Lock()

        # 故障效果状态
        self._lidar_noise_level = 0.0  # LiDAR 噪声标准差
        self._imu_drift_rate = 0.0     # IMU 漂移率 (rad/s)
        self._imu_bias = np.zeros(3)  # IMU 零偏
        self._encoder_offset = 0.0    # 编码器偏移
        self._comm_latency = 0.0      # 通信延迟 (秒)
        self._packet_loss_rate = 0.0   # 消息丢失率

        # 统计
        self._injection_count = 0
        self._detection_count = 0
        self._recovery_count = 0

        # 故障历史记录
        self._fault_history: List[Dict[str, Any]] = []

        # CPU 过载模拟线程
        self._cpu_stress_thread: Optional[threading.Thread] = None
        self._cpu_stress_running = False

    def inject(self, fault_type: FaultType,
               severity: FaultSeverity = FaultSeverity.MODERATE,
               duration: Optional[float] = None,
               start_delay: float = 0.0,
               **parameters):
        """注入故障

        Args:
            fault_type: 故障类型
            severity: 严重程度
            duration: 持续时间 (秒)，None 表示持续
            start_delay: 启动延迟 (秒)
            **parameters: 故障特定参数
        """
        fault = FaultInjection(
            fault_type=fault_type,
            severity=severity,
            duration=duration,
            start_delay=start_delay,
            parameters=parameters,
        )

        with self._lock:
            self._faults[fault_type] = fault
            self._injection_count += 1

            # 记录历史
            self._fault_history.append({
                'time': time.time(),
                'action': 'inject',
                'fault_type': fault_type.name,
                'severity': severity.name,
                'duration': duration,
                'parameters': parameters,
            })

        # 延迟启动
        if start_delay > 0:
            def _delayed_start():
                time.sleep(start_delay)
                self._activate_fault(fault)
            threading.Thread(target=_delayed_start, daemon=True).start()
        else:
            self._activate_fault(fault)

        logger.info(f"故障注入: {fault_type.name} (severity={severity.name}, "
                    f"duration={duration}s)")

    def clear(self, fault_type: FaultType):
        """清除指定故障"""
        with self._lock:
            if fault_type in self._faults:
                fault = self._faults.pop(fault_type)
                self._fault_history.append({
                    'time': time.time(),
                    'action': 'clear',
                    'fault_type': fault_type.name,
                })

        # 重置故障效果
        if fault_type == FaultType.LIDAR_NOISE:
            self._lidar_noise_level = 0.0
        elif fault_type == FaultType.IMU_DRIFT:
            self._imu_drift_rate = 0.0
            self._imu_bias = np.zeros(3)
        elif fault_type == FaultType.ENCODER_JUMP:
            self._encoder_offset = 0.0
        elif fault_type == FaultType.COMM_LATENCY:
            self._comm_latency = 0.0
        elif fault_type == FaultType.COMM_PACKET_LOSS:
            self._packet_loss_rate = 0.0
        elif fault_type == FaultType.CPU_OVERLOAD:
            self._stop_cpu_stress()

        logger.info(f"故障清除: {fault_type.name}")

    def clear_all(self):
        """清除所有故障"""
        with self._lock:
            faults = list(self._faults.keys())
        for ft in faults:
            self.clear(ft)

    def _activate_fault(self, fault: FaultInjection):
        """激活故障效果"""
        ft = fault.fault_type
        params = fault.parameters
        sev = fault.severity

        if ft == FaultType.LIDAR_NOISE:
            # LiDAR 噪声: 增加测距噪声
            base_noise = {FaultSeverity.MINOR: 0.05,
                         FaultSeverity.MODERATE: 0.2,
                         FaultSeverity.SEVERE: 0.5}
            self._lidar_noise_level = params.get('noise_std', base_noise.get(sev, 0.2))

        elif ft == FaultType.LIDAR_TIMEOUT:
            # LiDAR 超时: 返回空扫描
            pass  # 在 get_sensor_readings 中处理

        elif ft == FaultType.LIDAR_DATA_LOSS:
            # 数据丢失: 部分扫描点丢失
            pass  # 在 get_sensor_readings 中处理

        elif ft == FaultType.IMU_DRIFT:
            # IMU 漂移: 累积零偏
            base_drift = {FaultSeverity.MINOR: 0.01,
                         FaultSeverity.MODERATE: 0.05,
                         FaultSeverity.SEVERE: 0.2}
            self._imu_drift_rate = params.get('drift_rate', base_drift.get(sev, 0.05))

        elif ft == FaultType.IMU_SATURATION:
            # IMU 饱和: 固定值
            self._imu_bias = np.array([9.81, 0, 0])  # 模拟饱和

        elif ft == FaultType.ENCODER_JUMP:
            # 编码器跳变
            base_jump = {FaultSeverity.MINOR: 0.1,
                        FaultSeverity.MODERATE: 0.5,
                        FaultSeverity.SEVERE: 2.0}
            self._encoder_offset = params.get('offset', base_jump.get(sev, 0.5))

        elif ft == FaultType.COMM_LATENCY:
            # 通信延迟
            base_latency = {FaultSeverity.MINOR: 0.05,
                           FaultSeverity.MODERATE: 0.2,
                           FaultSeverity.SEVERE: 0.5}
            self._comm_latency = params.get('latency', base_latency.get(sev, 0.2))

        elif ft == FaultType.COMM_PACKET_LOSS:
            # 消息丢失
            base_loss = {FaultSeverity.MINOR: 0.1,
                        FaultSeverity.MODERATE: 0.3,
                        FaultSeverity.SEVERE: 0.6}
            self._packet_loss_rate = params.get('loss_rate', base_loss.get(sev, 0.3))

        elif ft == FaultType.CPU_OVERLOAD:
            # CPU 过载: 启动计算密集线程
            self._start_cpu_stress(params.get('load_percent', 80))

        # 设置激活状态
        fault._active = True
        fault._start_time = time.time()

        # 定时清除（如果有持续时间）
        if fault.duration is not None:
            def _auto_clear():
                time.sleep(fault.duration)
                self.clear(ft)
            threading.Thread(target=_auto_clear, daemon=True).start()

    # === 包装传感器读取 ===
    def get_sensor_readings(self) -> 'SensorReadings':
        """读取传感器数据（应用故障效果）

        Returns:
            SensorReadings（可能被故障修改）
        """
        # 通信延迟模拟
        if self._comm_latency > 0:
            time.sleep(self._comm_latency)

        # 消息丢失模拟
        if self._packet_loss_rate > 0:
            if random.random() < self._packet_loss_rate:
                # 返回上一次的数据（模拟丢失）
                return self._last_readings if hasattr(self, '_last_readings') else None

        # 读取真实数据
        if self.hw:
            readings = self.hw.get_sensor_readings()
        else:
            return None

        # 应用故障效果
        with self._lock:
            active_faults = [f for f in self._faults.values() if f._active]

        for fault in active_faults:
            ft = fault.fault_type

            if ft == FaultType.LIDAR_NOISE:
                # 加入高斯噪声
                readings.lidar_ranges = readings.lidar_ranges + np.random.normal(
                    0, self._lidar_noise_level, len(readings.lidar_ranges))
                readings.lidar_ranges = np.clip(
                    readings.lidar_ranges, 0, 100)

            elif ft == FaultType.LIDAR_TIMEOUT:
                # 超时: 返回空扫描
                readings.lidar_ranges = np.full(
                    len(readings.lidar_ranges), 0.0)

            elif ft == FaultType.LIDAR_DATA_LOSS:
                # 部分丢失: 随机置零
                loss_mask = np.random.random(
                    len(readings.lidar_ranges)) < 0.3
                readings.lidar_ranges[loss_mask] = 0.0

            elif ft == FaultType.IMU_DRIFT:
                # 累积漂移
                dt = time.time() - fault._start_time
                drift = self._imu_drift_rate * dt
                readings.imu_gyro = readings.imu_gyro + np.array(
                    [drift, drift * 0.5, drift * 0.3])

            elif ft == FaultType.IMU_SATURATION:
                # 饱和: 固定值
                readings.imu_accel = self._imu_bias

            elif ft == FaultType.ENCODER_JUMP:
                # 编码器跳变
                readings.odom_x += self._encoder_offset
                readings.odom_y += self._encoder_offset * 0.5

        self._last_readings = readings
        return readings

    def send_velocity(self, vx: float, vy: float, wz: float):
        """发送速度命令（应用通信故障）"""
        if self._comm_latency > 0:
            time.sleep(self._comm_latency)

        if self._packet_loss_rate > 0:
            if random.random() < self._packet_loss_rate:
                return  # 命令丢失

        if self.hw:
            self.hw.send_velocity(vx, vy, wz)

    # === CPU 过载模拟 ===
    def _start_cpu_stress(self, load_percent: int = 80):
        """启动 CPU 压力测试线程"""
        self._cpu_stress_running = True
        self._cpu_stress_thread = threading.Thread(
            target=self._cpu_stress_loop,
            args=(load_percent,),
            daemon=True, name='FaultInjector-CPU')
        self._cpu_stress_thread.start()

    def _stop_cpu_stress(self):
        """停止 CPU 压力测试"""
        self._cpu_stress_running = False
        if self._cpu_stress_thread:
            self._cpu_stress_thread.join(timeout=1.0)

    def _cpu_stress_loop(self, load_percent: int):
        """CPU 压力测试循环"""
        busy_time = load_percent / 1000.0  # ms
        idle_time = (100 - load_percent) / 1000.0
        while self._cpu_stress_running:
            t0 = time.time()
            while (time.time() - t0) < busy_time:
                # 空转消耗 CPU
                math.sqrt(12345.6789)
            time.sleep(idle_time)

    # === 统计与报告 ===
    def record_detection(self, fault_type: FaultType):
        """记录系统检测到故障"""
        self._detection_count += 1
        logger.info(f"系统检测到故障: {fault_type.name}")

    def record_recovery(self, fault_type: FaultType):
        """记录系统从故障恢复"""
        self._recovery_count += 1
        logger.info(f"系统从故障恢复: {fault_type.name}")

    def get_fault_history(self) -> List[Dict[str, Any]]:
        """获取故障历史记录"""
        return list(self._fault_history)

    def get_active_faults(self) -> List[FaultType]:
        """获取当前活跃故障列表"""
        with self._lock:
            return [ft for ft, f in self._faults.items() if f._active]

    @property
    def stats(self) -> Dict[str, Any]:
        """统计信息"""
        return {
            'injection_count': self._injection_count,
            'detection_count': self._detection_count,
            'recovery_count': self._recovery_count,
            'detection_rate': (self._detection_count / max(self._injection_count, 1)),
            'recovery_rate': (self._recovery_count / max(self._injection_count, 1)),
            'active_faults': [f.name for f in self.get_active_faults()],
        }

    def generate_report(self) -> Dict[str, Any]:
        """生成故障注入测试报告"""
        return {
            'summary': self.stats,
            'fault_history': self.get_fault_history(),
            'conclusion': self._evaluate_robustness(),
        }

    def _evaluate_robustness(self) -> str:
        """评估系统鲁棒性"""
        if self._injection_count == 0:
            return "无故障注入"
        det_rate = self._detection_count / self._injection_count
        rec_rate = self._recovery_count / self._injection_count
        if det_rate >= 0.9 and rec_rate >= 0.8:
            return "优秀：故障检测和恢复能力强"
        elif det_rate >= 0.7 and rec_rate >= 0.6:
            return "良好：基本能应对常见故障"
        elif det_rate >= 0.5:
            return "一般：部分故障无法检测"
        else:
            return "不足：故障检测能力弱"
