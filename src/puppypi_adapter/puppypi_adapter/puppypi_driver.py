"""PuppyPi 实体机硬件驱动实现

将 HardwareInterface 接口映射到 PuppyPi SDK 调用。
上层导航逻辑通过此驱动与实体硬件交互。

硬件架构:
    - 主控: Raspberry Pi 4B (ARM Cortex-A72)
    - LiDAR: RPLIDAR A1/A2 (UART)
    - IMU: MPU6050 / BMI160 (I2C)
    - 电机驱动: PCA9685 PWM + 直流减速电机
    - 电池: 3S LiPo (11.1V nominal)
    - 相机: USB Camera (可选)

依赖:
    - puppypi_sdk (PuppyPi 官方 SDK)
    - pyserial (LiDAR 串口通信)
    - smbus2 (I2C 设备访问)
    - RPi.GPIO (树莓派 GPIO, 急停按钮)

线程模型:
    - 主线程: 导航算法 (通过接口调用)
    - 传感器线程: 后台读取 LiDAR/IMU (30Hz)
    - 电机线程: 运动控制环 (30Hz)
    - 监控线程: 电池/温度监控 (1Hz)
"""
from __future__ import annotations

import math
import time
import threading
import logging
from typing import Optional, Dict, Any, Tuple

import numpy as np

from .hardware_interface import (
    HardwareInterface, SensorReadings, BatteryState, RobotHealthState
)

logger = logging.getLogger(__name__)


class PuppyPiHardwareInterface(HardwareInterface):
    """PuppyPi 实体机硬件接口

    实现 HardwareInterface 接口，调用 PuppyPi SDK 与实体硬件交互。
    包含运动控制环、传感器数据流、电池状态读取。

    配置示例:
        config = {
            'backend': 'real',
            'motion': {
                'max_linear_x': 0.3, 'max_angular_z': 1.2,
                'accel_limit': 2.0, 'yaw_rate_limit': 4.0,
                'cmd_timeout': 1.0,
            },
            'sensors': {
                'lidar_port': '/dev/ttyUSB0', 'lidar_baud': 115200,
                'lidar_beams': 720, 'lidar_range': 12.0,
                'imu_i2c_addr': 0x68, 'imu_rate': 100,
            },
            'battery': {
                'adc_channel': 0, 'adc_vref': 3.3,
                'low_threshold': 0.20, 'critical_threshold': 0.10,
                'auto_recharge': True, 'nominal_voltage': 11.1,
                'cells': 3, 'full_voltage': 12.6, 'empty_voltage': 9.9,
            },
            'gpio': {
                'estop_button_pin': 18,  # BCM 编号
                'led_status_pin': 24,
            },
        }
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)

        # 传感器配置
        sensor_cfg = self.config.get('sensors', {})
        self.lidar_port = sensor_cfg.get('lidar_port', '/dev/ttyUSB0')
        self.lidar_baud = sensor_cfg.get('lidar_baud', 115200)
        self.lidar_beams = sensor_cfg.get('lidar_beams', 720)
        self.lidar_range = sensor_cfg.get('lidar_range', 12.0)
        self.imu_i2c_addr = sensor_cfg.get('imu_i2c_addr', 0x68)
        self.imu_rate = sensor_cfg.get('imu_rate', 100)

        # 电池配置
        battery_cfg = self.config.get('battery', {})
        self.battery_adc_channel = battery_cfg.get('adc_channel', 0)
        self.battery_adc_vref = battery_cfg.get('adc_vref', 3.3)
        self.battery_cells = battery_cfg.get('cells', 3)
        self.battery_full_voltage = battery_cfg.get('full_voltage', 12.6)
        self.battery_empty_voltage = battery_cfg.get('empty_voltage', 9.9)
        self.battery_nominal = battery_cfg.get('nominal_voltage', 11.1)

        # GPIO 配置（急停按钮、状态LED）
        gpio_cfg = self.config.get('gpio', {})
        self.estop_pin = gpio_cfg.get('estop_button_pin', 18)
        self.led_pin = gpio_cfg.get('led_status_pin', 24)

        # 硬件句柄
        self._sdk = None
        self._lidar = None
        self._imu = None
        self._adc = None
        self._gpio = None

        # 传感器数据缓存
        self._sensor_lock = threading.Lock()
        self._cached_lidar: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._cached_imu: Optional[dict] = None
        self._cached_odom: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # 线程控制
        self._running = False
        self._sensor_thread: Optional[threading.Thread] = None
        self._motor_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._estop_thread: Optional[threading.Thread] = None

        # 运动控制环状态
        self._target_velocity = (0.0, 0.0, 0.0)  # (vx, vy, wz)
        self._actual_velocity = (0.0, 0.0, 0.0)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        # 电池缓存
        self._battery = BatteryState()
        self._cpu_temp = 45.0

        # 急停按钮状态（硬件中断回调）
        self._hw_estop_triggered = False

    def initialize(self) -> bool:
        """初始化实体硬件

        顺序:
            1. 初始化 PuppyPi SDK (电机驱动)
            2. 初始化 LiDAR
            3. 初始化 IMU
            4. 初始化 ADC (电池读取)
            5. 初始化 GPIO (急停按钮)
            6. 启动后台线程
        """
        try:
            # 1. 初始化 PuppyPi SDK
            self._init_sdk()

            # 2. 初始化 LiDAR
            self._init_lidar()

            # 3. 初始化 IMU
            self._init_imu()

            # 4. 初始化 ADC (电池电压读取)
            self._init_adc()

            # 5. 初始化 GPIO (急停按钮 + LED)
            self._init_gpio()

            # ``real`` is an explicit deployment mode, not a best-effort
            # hardware probe. Synthetic sensor values invalidate evidence, so
            # fail before starting any control thread when hardware is absent.
            missing = []
            for name, handle in (
                    ('puppypi_sdk', self._sdk),
                    ('lidar', self._lidar),
                    ('imu', self._imu),
                    ('adc', self._adc),
                    ('gpio', self._gpio)):
                if handle is None:
                    missing.append(name)
            if missing:
                raise RuntimeError(
                    'real backend hardware preflight failed; missing: '
                    + ', '.join(missing))

            # 6. 启动后台线程
            self._running = True
            self._sensor_thread = threading.Thread(
                target=self._sensor_loop, daemon=True, name='PuppyPi-Sensor')
            self._motor_thread = threading.Thread(
                target=self._motor_loop, daemon=True, name='PuppyPi-Motor')
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop, daemon=True, name='PuppyPi-Monitor')
            self._estop_thread = threading.Thread(
                target=self._estop_poll_loop, daemon=True, name='PuppyPi-EStop')

            self._sensor_thread.start()
            self._motor_thread.start()
            self._monitor_thread.start()
            self._estop_thread.start()

            self._initialized = True
            logger.info("PuppyPi 硬件初始化完成")
            return True

        except Exception as e:
            logger.error(f"PuppyPi 硬件初始化失败: {e}")
            self._last_error = str(e)
            # Close partially initialized devices; returning False remains a
            # hard failure for the real backend and never implies mock mode.
            try:
                self.shutdown()
            except Exception as cleanup_error:  # noqa: BLE001
                logger.error(f"PuppyPi cleanup after failed init failed: {cleanup_error}")
            return False

    def shutdown(self):
        """关闭硬件，安全停止所有线程和设备"""
        self._running = False
        # 停止电机
        self._dispatch_motion(0.0, 0.0, 0.0)

        # 等待线程结束
        for t in [self._sensor_thread, self._motor_thread,
                  self._monitor_thread, self._estop_thread]:
            if t:
                t.join(timeout=1.0)

        # 关闭设备
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.disconnect()
            except Exception:
                pass
        if self._sdk:
            try:
                self._sdk.close()
            except Exception:
                pass

        self._initialized = False
        logger.info("PuppyPi 硬件已关闭")

    # === 硬件初始化子方法 ===
    def _init_sdk(self):
        """初始化 PuppyPi SDK (电机驱动)"""
        try:
            import puppypi_sdk as sdk
            self._sdk = sdk.PuppyPi()
            self._sdk.initialize()
            logger.info("PuppyPi SDK 初始化成功")
        except ImportError:
            logger.warning("puppypi_sdk 未安装，使用模拟模式")
            self._sdk = None
        except Exception as e:
            logger.error(f"PuppyPi SDK 初始化失败: {e}")
            raise

    def _init_lidar(self):
        """初始化 LiDAR (RPLIDAR)"""
        try:
            from rplidar import RPLidar
            self._lidar = RPLidar(self.lidar_port, baudrate=self.lidar_baud)
            self._lidar.connect()
            self._lidar.start_motor()
            info = self._lidar.get_info()
            logger.info(f"LiDAR 初始化成功: {info}")
        except ImportError:
            logger.warning("rplidar 库未安装，LiDAR 使用模拟模式")
            self._lidar = None
        except Exception as e:
            logger.error(f"LiDAR 初始化失败: {e}")
            self._lidar = None

    def _init_imu(self):
        """初始化 IMU (MPU6050 / BMI160)"""
        try:
            from smbus2 import SMBus
            bus = SMBus(1)  # I2C bus 1
            # MPU6050 初始化
            bus.write_byte_data(self.imu_i2c_addr, 0x6B, 0)  # 唤醒
            self._imu = {'bus': bus, 'addr': self.imu_i2c_addr}
            logger.info("IMU 初始化成功")
        except ImportError:
            logger.warning("smbus2 未安装，IMU 使用模拟模式")
            self._imu = None
        except Exception as e:
            logger.warning(f"IMU 初始化失败 (非x86可能正常): {e}")
            self._imu = None

    def _init_adc(self):
        """初始化 ADC (电池电压读取)"""
        try:
            import Adafruit_ADS1x15
            self._adc = Adafruit_ADS1x15.ADS1115()
            logger.info("ADC 初始化成功")
        except ImportError:
            logger.warning("Adafruit_ADS1x15 未安装，电池使用模拟模式")
            self._adc = None
        except Exception as e:
            logger.warning(f"ADC 初始化失败: {e}")
            self._adc = None

    def _init_gpio(self):
        """初始化 GPIO (急停按钮 + LED)"""
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.estop_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.led_pin, GPIO.OUT)
            self._gpio = GPIO
            logger.info("GPIO 初始化成功")
        except ImportError:
            logger.warning("RPi.GPIO 未安装 (非树莓派环境)")
            self._gpio = None
        except Exception as e:
            logger.warning(f"GPIO 初始化失败: {e}")
            self._gpio = None

    # === 运动控制实现 ===
    def _dispatch_motion(self, vx: float, vy: float, wz: float):
        """将速度命令发送到 PuppyPi SDK

        PuppyPi 为四足机器人，将 (vx, vy, wz) 转换为步态参数:
            - vx → 前后步幅
            - vy → 左右步幅
            - wz → 旋转步幅
        """
        with self._sensor_lock:
            self._target_velocity = (vx, vy, wz)

        if self._sdk:
            try:
                # PuppyPi SDK 调用 (实际 API 根据版本调整)
                method = getattr(self._sdk, 'set_velocity', None)
                if method is None:
                    method = getattr(self._sdk, 'move', None)
                if method is None:
                    raise AttributeError('PuppyPi SDK has no set_velocity or move')
                method(vx, vy, wz)
            except Exception as e:
                logger.error(f"电机控制失败: {e}")
                self._error_count += 1

    # === 传感器读取实现 ===
    def get_sensor_readings(self) -> SensorReadings:
        """读取实体传感器数据"""
        with self._sensor_lock:
            lidar_data = self._cached_lidar
            imu_data = self._cached_imu
            odom = self._cached_odom
            actual_vel = self._actual_velocity

        # LiDAR 数据
        if lidar_data:
            angles, ranges = lidar_data
        else:
            # 无 LiDAR 时返回空扫描
            angles = np.linspace(-math.pi, math.pi, self.lidar_beams, endpoint=False)
            ranges = np.full(self.lidar_beams, self.lidar_range)

        # IMU 数据
        if imu_data:
            imu_accel = np.array(imu_data.get('accel', [0, 0, 9.81]))
            imu_gyro = np.array(imu_data.get('gyro', [0, 0, 0]))
            imu_quat = np.array(imu_data.get('quat', [1, 0, 0, 0]))
        else:
            imu_accel = np.array([0, 0, 9.81])
            imu_gyro = np.zeros(3)
            imu_quat = np.array([1.0, 0, 0, 0])

        return SensorReadings(
            lidar_angles=angles,
            lidar_ranges=ranges,
            imu_accel=imu_accel,
            imu_gyro=imu_gyro,
            imu_quat=imu_quat,
            odom_x=odom[0], odom_y=odom[1], odom_yaw=odom[2],
            odom_vx=actual_vel[0], odom_vy=actual_vel[1], odom_wz=actual_vel[2],
            timestamp=time.time(),
        )

    def get_battery(self) -> BatteryState:
        """读取实体电池状态"""
        return self._battery

    def get_health(self) -> RobotHealthState:
        """读取实体健康状态"""
        faults = []
        level = 'OK'
        ok = True

        if self._emergency_stopped or self._hw_estop_triggered:
            level = 'FATAL'
            ok = False
            faults.append('EMERGENCY_STOP')
        if self._battery.critical_battery:
            level = 'ERROR'
            ok = False
            faults.append('CRITICAL_BATTERY')
        elif self._battery.low_battery:
            if level == 'OK':
                level = 'WARN'
            faults.append('LOW_BATTERY')
        if self._cpu_temp > 80:
            if level == 'OK':
                level = 'WARN'
            faults.append(f'CPU_HIGH_TEMP:{self._cpu_temp:.0f}C')

        return RobotHealthState(
            ok=ok, level=level, active_faults=faults,
            cpu_temp=self._cpu_temp,
            imu_ready=self._imu is not None,
            lidar_ready=self._lidar is not None,
            camera_ready=True,
            motion_ready=self._motors_enabled,
        )

    # === 后台线程 ===
    def _sensor_loop(self):
        """传感器读取线程 (30Hz)

        持续读取 LiDAR 和 IMU 数据并缓存
        """
        rate = 30.0  # Hz
        dt = 1.0 / rate
        while self._running:
            try:
                # LiDAR 读取
                if self._lidar:
                    scan = self._read_lidar_scan()
                    if scan:
                        with self._sensor_lock:
                            self._cached_lidar = scan

                # IMU 读取
                if self._imu:
                    imu_data = self._read_imu()
                    if imu_data:
                        with self._sensor_lock:
                            self._cached_imu = imu_data

            except Exception as e:
                logger.error(f"传感器读取错误: {e}")

            time.sleep(dt)

    def _motor_loop(self):
        """运动控制环线程 (30Hz)

        三级运动控制:
            1. 目标速度 → 实际速度 (加速度限制)
            2. 实际速度 → 里程计积分
            3. 速度命令 → SDK 调用
        """
        rate = 30.0  # Hz (≥20Hz 要求)
        dt = 1.0 / rate
        while self._running:
            try:
                with self._sensor_lock:
                    target_vx, target_vy, target_wz = self._target_velocity

                # 加速度限制 (平滑过渡)
                avx, avy, awz = self._actual_velocity
                avx = self._limit_accel(target_vx, avx, dt, self.accel_limit)
                avy = self._limit_accel(target_vy, avy, dt, self.accel_limit)
                awz = self._limit_accel(target_wz, awz, dt, self.yaw_rate_limit)

                # 里程计积分
                self._odom_x += avx * math.cos(self._odom_yaw) * dt
                self._odom_y += avx * math.sin(self._odom_yaw) * dt
                self._odom_yaw += awz * dt
                while self._odom_yaw > math.pi:
                    self._odom_yaw -= 2 * math.pi
                while self._odom_yaw < -math.pi:
                    self._odom_yaw += 2 * math.pi

                with self._sensor_lock:
                    self._actual_velocity = (avx, avy, awz)
                    self._cached_odom = (self._odom_x, self._odom_y, self._odom_yaw)

                # 命令超时检测
                if self.command_timeout and self._motors_enabled:
                    self._dispatch_motion(0.0, 0.0, 0.0)

            except Exception as e:
                logger.error(f"运动控制环错误: {e}")

            time.sleep(dt)

    def _monitor_loop(self):
        """监控线程 (1Hz)

        读取电池电压、CPU 温度
        """
        while self._running:
            try:
                self._read_battery()
                self._read_cpu_temp()
            except Exception as e:
                logger.error(f"监控错误: {e}")
            time.sleep(1.0)

    def _estop_poll_loop(self):
        """急停按钮轮询线程 (100Hz)

        三级急停的第二级：硬件按钮急停
        当按钮被按下时立即触发急停
        """
        if not self._gpio:
            return
        while self._running:
            try:
                # 按钮按下为 LOW (下拉)
                if self._gpio.input(self.estop_pin) == 0:
                    if not self._hw_estop_triggered:
                        self._hw_estop_triggered = True
                        self.emergency_stop("hardware_button")
                        logger.critical("硬件急停按钮触发!")
                else:
                    if self._hw_estop_triggered:
                        self._hw_estop_triggered = False
                        logger.info("硬件急停按钮释放")
            except Exception as e:
                logger.error(f"急停按钮检测错误: {e}")
            time.sleep(0.01)  # 100Hz

    # === 硬件读取子方法 ===
    def _read_lidar_scan(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """读取 LiDAR 扫描数据"""
        try:
            scans = list(self._lidar.iter_scans())
            if not scans:
                return None
            scan = scans[-1]  # 最新扫描
            # 转换为 (angles, ranges)
            quality, angle, distance = [], [], []
            for q, a, d in scan:
                quality.append(q)
                angle.append(a)
                distance.append(d / 1000.0)  # mm → m

            angles = np.array(angle)
            ranges = np.array(distance)
            return (angles, ranges)
        except Exception as e:
            logger.error(f"LiDAR 读取失败: {e}")
            return None

    def _read_imu(self) -> Optional[dict]:
        """读取 IMU 数据 (MPU6050)"""
        try:
            bus = self._imu['bus']
            addr = self._imu['addr']

            # 读取加速度 (6字节)
            accel_data = bus.read_i2c_block_data(addr, 0x3B, 6)
            ax = self._twos_complement(accel_data[0] << 8 | accel_data[1]) / 16384.0
            ay = self._twos_complement(accel_data[2] << 8 | accel_data[3]) / 16384.0
            az = self._twos_complement(accel_data[4] << 8 | accel_data[5]) / 16384.0

            # 读取陀螺仪 (6字节)
            gyro_data = bus.read_i2c_block_data(addr, 0x43, 6)
            gx = self._twos_complement(gyro_data[0] << 8 | gyro_data[1]) / 131.0
            gy = self._twos_complement(gyro_data[2] << 8 | gyro_data[3]) / 131.0
            gz = self._twos_complement(gyro_data[4] << 8 | gyro_data[5]) / 131.0

            # 转换为 SI 单位
            accel = np.array([ax * 9.81, ay * 9.81, az * 9.81])
            gyro = np.array([math.radians(gx), math.radians(gy), math.radians(gz)])

            return {
                'accel': accel,
                'gyro': gyro,
                'quat': np.array([1.0, 0, 0, 0]),  # 需互补滤波
            }
        except Exception as e:
            logger.debug(f"IMU 读取失败: {e}")
            return None

    def _read_battery(self):
        """读取电池电压 (通过 ADC)"""
        if not self._adc:
            # 模拟模式：缓慢消耗
            self._battery.percent = max(
                0.0, self._battery.percent - 0.0001)
            self._battery.update_thresholds(
                self.low_battery_threshold,
                self.critical_battery_threshold)
            return

        try:
            # ADC 读取 (12位, 0-3.3V)
            raw = self._adc.read_adc(self.battery_adc_channel, gain=1)
            # 分压电阻: 电池电压 → ADC 电压
            adc_voltage = raw * self.battery_adc_vref / 32767.0
            # 假设 1:3 分压 (R1=30kΩ, R2=10kΩ)
            battery_voltage = adc_voltage * 4.0

            # 电压 → 电量百分比 (线性近似)
            percent = ((battery_voltage - self.battery_empty_voltage) /
                       (self.battery_full_voltage - self.battery_empty_voltage))
            percent = max(0.0, min(1.0, percent))

            self._battery.voltage = battery_voltage
            self._battery.percent = percent
            self._battery.temperature = 25.0  # 需温度传感器
            self._battery.update_thresholds(
                self.low_battery_threshold,
                self.critical_battery_threshold)

            # 低电量自动回充
            if (self._battery.low_battery and self.auto_recharge_enabled
                    and not self._battery.charging):
                logger.warning(f"低电量 {percent*100:.0f}%，触发自动回充")

        except Exception as e:
            logger.error(f"电池读取失败: {e}")

    def _read_cpu_temp(self):
        """读取 CPU 温度 (树莓派)"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                self._cpu_temp = float(f.read().strip()) / 1000.0
        except Exception:
            self._cpu_temp = 45.0  # 默认值

    @staticmethod
    def _twos_complement(val: int) -> int:
        """16位二补码转换"""
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    @property
    def hw_estop_triggered(self) -> bool:
        """硬件急停按钮是否触发"""
        return self._hw_estop_triggered
