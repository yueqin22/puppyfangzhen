"""PuppyPi Adapter: platform-specific translation layer.

Translates generic motion intent into PuppyPi SDK commands, and
PuppyPi platform status into standard ROS2 messages.

硬件抽象层模块:
    - hardware_interface: 统一硬件接口基类 + 工厂方法
    - mock_interface: 纯Python仿真实现 (接口与实体机一致)
    - puppypi_driver: PuppyPi实体机驱动 (RPLIDAR/MPU6050/PCA9685)
    - safety_subsystem_v2: 三级急停 + 分级告警 + 降级策略
    - sensor_calibration: LiDAR-IMU外参 + 相机内参 + 里程计标定
    - fault_injection: 故障注入测试框架 (15种故障类型)
    - performance_monitor: 实时性能监控 + 压力测试 + Fuzz Testing
"""
__version__ = '1.0.0'

from .hardware_interface import HardwareInterface, SensorReadings, BatteryState, RobotHealthState, Watchdog
from .mock_interface import MockHardwareInterface
from .safety_subsystem_v2 import SafetySubsystem, AlertLevel, EStopSource, DegradationMode
from .sensor_calibration import SensorCalibrator, ExtrinsicCalibration, CameraIntrinsics, OdometryCalibration
from .fault_injection import FaultInjector, FaultType, FaultSeverity, FaultInjection
from .performance_monitor import PerformanceMonitor, StressTester, SensorFuzzer, FrameTiming

# puppypi_driver 仅在实体机上可用 (依赖 RPi.GPIO/smbus 等)
try:
    from .puppypi_driver import PuppyPiHardwareInterface
except ImportError:
    PuppyPiHardwareInterface = None

__all__ = [
    'HardwareInterface', 'SensorReadings', 'BatteryState', 'RobotHealthState', 'Watchdog',
    'MockHardwareInterface',
    'PuppyPiHardwareInterface',
    'SafetySubsystem', 'AlertLevel', 'EStopSource', 'DegradationMode',
    'SensorCalibrator', 'ExtrinsicCalibration', 'CameraIntrinsics', 'OdometryCalibration',
    'FaultInjector', 'FaultType', 'FaultSeverity', 'FaultInjection',
    'PerformanceMonitor', 'StressTester', 'SensorFuzzer', 'FrameTiming',
]
