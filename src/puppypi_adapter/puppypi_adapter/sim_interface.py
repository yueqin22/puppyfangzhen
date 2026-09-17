"""仿真硬件接口 (Sim Hardware Interface)

P0-4 backend 语义: ``sim`` = Gazebo / UE / C++ 仿真的输入输出。

与 ``mock`` 的本质区别（这是本文件存在的唯一理由）：

======  ==================================================  ==========================
backend 动力学归属                                          传感器数据来源
======  ==================================================  ==========================
mock    本进程内的 Python 运动学假模型，不代表真实执行器     本进程合成（带噪声的假数据）
sim     外部仿真器（Gazebo/UE/C++），本接口只做命令转发      外部仿真器通过 ROS topic 注入
======  ==================================================  ==========================

诚实性约束（P0-4 硬性要求）:
    - **绝不合成传感器数据**。LiDAR/IMU/里程计/电池必须由外部仿真器注入；
      未注入时返回空/默认并告警，而不是生成"看起来合理"的假数据——后者会让
      上层误以为链路已通，是本项目历史上一再出现的问题。
    - 与 ``mock`` 实现**完全相同的接口契约**，上层代码无需区分二者。

使用方式:
    hw = SimHardwareInterface({'backend': 'sim', 'motion': {...}})
    hw.initialize()
    hw.enable_motors(True)
    hw.send_velocity(0.2, 0.0, 0.1)   # 转发给外部仿真器

    # 由面向仿真的节点把仿真器真实数据注入进来
    hw.set_sensor_readings(readings)
    hw.set_battery(battery_state)
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Dict, Any, Callable, List

from .hardware_interface import (
    HardwareInterface, SensorReadings, BatteryState, RobotHealthState
)

logger = logging.getLogger(__name__)


class SimHardwareInterface(HardwareInterface):
    """仿真硬件接口 — 命令转发 + 外部状态注入，不伪造数据

    配置示例:
        config = {
            'backend': 'sim',
            'motion': {'max_linear_x': 0.3, 'max_angular_z': 1.2,
                       'cmd_timeout': 1.0},
        }

    外部仿真器接入:
        1. 命令侧: 通过 ``set_command_sink(callable)`` 注册转发回调,
           签名为 ``sink(vx: float, vy: float, wz: float) -> None``。
           未注册时命令仅被记录（用于状态发布），不会静默丢弃。
        2. 状态侧: 通过 ``set_sensor_readings`` / ``set_battery`` /
           ``set_health`` 注入仿真器的真实读数。
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)

        # ---- 命令侧 ----
        self._command_sink: Optional[Callable[[float, float, float], None]] = None
        self._last_dispatched = (0.0, 0.0, 0.0)  # (vx, vy, wz)
        self._dispatch_errors: List[str] = []

        # ---- 状态侧（由外部仿真器注入）----
        self._sensors: Optional[SensorReadings] = None
        self._sensors_stamp: float = 0.0
        self._battery: Optional[BatteryState] = None
        self._health = RobotHealthState(
            # A sim adapter is only a transport boundary.  Until the
            # simulator injects sensor data, reporting OK would falsely claim
            # that LiDAR/IMU/camera are healthy.
            ok=False, level='WARN',
            imu_ready=False, lidar_ready=False,
            camera_ready=False, motion_ready=False,
            active_faults=['SIM_BACKEND_NO_DATA_INJECTED'],
        )
        # 数据新鲜度阈值: 超过此时间未注入视为仿真器失联
        self._stale_sec = float(
            self.config.get('sim', {}).get('stale_sec', 2.0))

        self._lock = threading.Lock()
        self._initialized = False
        self._sim_connected = False

    # ==== 外部仿真器接入点 ============================================
    def set_command_sink(self, sink: Optional[Callable[[float, float, float], None]]):
        """注册命令转发回调 (外部仿真器的执行器入口)"""
        with self._lock:
            self._command_sink = sink
            self._sim_connected = sink is not None
            self._health.motion_ready = sink is not None
            if sink is None:
                self._health.ok = False
                self._health.level = 'WARN'
                if 'SIM_BACKEND_NO_SINK' not in self._health.active_faults:
                    self._health.active_faults.append('SIM_BACKEND_NO_SINK')

    def set_sensor_readings(self, readings: SensorReadings):
        """注入外部仿真器的传感器读数"""
        with self._lock:
            self._sensors = readings
            self._sensors_stamp = time.time()
            self._health.imu_ready = True
            self._health.lidar_ready = True
            if self._sim_connected:
                self._health.ok = True
                self._health.level = 'OK'
                self._health.active_faults = []

    def set_battery(self, battery: BatteryState):
        """注入外部仿真器的电池状态"""
        with self._lock:
            self._battery = battery

    def set_health(self, health: RobotHealthState):
        """注入外部仿真器的健康状态"""
        with self._lock:
            self._health = health

    # ==== 生命周期 ==================================================
    def initialize(self) -> bool:
        """初始化仿真后端连接

        注意: 这里**不**建立与外部仿真器的实际连接（由 ROS 节点负责订阅/发布）。
        仅标记本接口就绪；真正的"仿真器已连通"以 set_command_sink /
        set_sensor_readings 是否被调用为准。
        """
        self._initialized = True
        logger.info(
            'SimHardwareInterface 就绪 (等待外部仿真器注入数据; '
            '未注入前 get_sensor_readings 返回空, 不合成假数据)')
        return True

    def shutdown(self):
        """关闭仿真后端"""
        with self._lock:
            self._initialized = False
            self._sim_connected = False
            self._command_sink = None
        # 关闭前确保不残留运动指令
        self._dispatch_motion(0.0, 0.0, 0.0)

    # ==== 运动控制 ==================================================
    def _dispatch_motion(self, vx: float, vy: float, wz: float):
        """将速度命令转发给外部仿真器

        未注册 sink 时仅记录命令（供状态发布读取），不抛异常、不伪造执行。
        """
        with self._lock:
            sink = self._command_sink
        self._last_dispatched = (vx, vy, wz)
        if sink is None:
            return
        try:
            sink(vx, vy, wz)
        except Exception as exc:  # 转发失败必须可见, 不能静默吞掉
            msg = f'sim command sink failed: {exc}'
            logger.error(msg)
            with self._lock:
                self._dispatch_errors.append(msg)
                self._error_count += 1
                self._last_error = msg
                self._health.motion_ready = False

    # ==== 传感器 / 电池 / 健康 =======================================
    def get_sensor_readings(self) -> SensorReadings:
        """返回外部仿真器注入的传感器读数

        未注入时返回**空**读数（不是合成数据），并周期性告警。
        """
        with self._lock:
            sensors = self._sensors
            stamp = self._sensors_stamp
        if sensors is None:
            logger.warning(
                'SimHardwareInterface: 尚未注入传感器数据 '
                '(外部仿真器未连通?), 返回空读数而非合成数据')
            return SensorReadings()
        if (time.time() - stamp) > self._stale_sec:
            logger.warning(
                'SimHardwareInterface: 传感器数据已过期 '
                f'(>{self._stale_sec}s 未更新), 仿真器可能失联')
        return sensors

    def get_battery(self) -> BatteryState:
        """返回外部仿真器注入的电池状态（未注入时返回默认值，非合成动态）"""
        with self._lock:
            if self._battery is None:
                return BatteryState()
            return self._battery

    def get_health(self) -> RobotHealthState:
        """返回健康状态，含"是否收到仿真器数据"的诚实标记"""
        with self._lock:
            stale = (self._sensors is not None
                     and (time.time() - self._sensors_stamp) > self._stale_sec)
            health = RobotHealthState(
                ok=self._health.ok and not stale,
                level=('WARN' if stale else self._health.level),
                active_faults=list(self._health.active_faults),
                cpu_temp=self._health.cpu_temp,
                imu_ready=self._health.imu_ready,
                lidar_ready=self._health.lidar_ready,
                camera_ready=self._health.camera_ready,
                motion_ready=self._health.motion_ready and self._sim_connected,
            )
            if stale:
                health.active_faults.append('SIM_DATA_STALE')
            return health

    # ==== 诊断 ======================================================
    @property
    def sim_connected(self) -> bool:
        """外部仿真器执行器入口是否已注册"""
        return self._sim_connected

    @property
    def last_dispatched(self):
        """最近一次转发的速度命令 (vx, vy, wz)"""
        return self._last_dispatched

    @property
    def stats(self) -> Dict[str, Any]:
        """运行统计（补充 sim 专属字段）"""
        base = super().stats
        base.update({
            'backend': 'sim',
            'sim_connected': self._sim_connected,
            'last_dispatched': list(self._last_dispatched),
            'dispatch_errors': len(self._dispatch_errors),
            'sensors_injected': self._sensors is not None,
        })
        return base
