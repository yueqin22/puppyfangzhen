"""仿真 Mock 硬件接口实现

提供纯 Python 的仿真硬件接口，无需 CoppeliaSim 或实体硬件。
用于开发测试和算法验证，接口与实体机完全一致。

特点:
    - 完全用 Python 模拟运动学和传感器
    - 支持动态障碍物和噪声注入
    - 接口与 PuppyPiHardwareInterface 完全一致
    - 用于 CI/CD 和无硬件环境开发

运动学模型:
    差速驱动运动学:
        x' = x + v·cos(θ)·dt
        y' = y + v·sin(θ)·dt
        θ' = θ + ω·dt
"""
from __future__ import annotations

import math
import time
import threading
import random
from typing import Optional, Dict, Any

import numpy as np

from .hardware_interface import (
    HardwareInterface, SensorReadings, BatteryState, RobotHealthState
)


class MockHardwareInterface(HardwareInterface):
    """纯 Python Mock 硬件接口

    模拟差速驱动机器人的运动学、传感器和电池。
    所有接口与实体机完全一致，用于无硬件环境开发。

    配置示例:
        config = {
            'backend': 'mock',
            'motion': {'max_linear_x': 0.3, 'max_angular_z': 1.2},
            'battery': {'low_threshold': 0.20, 'auto_recharge': True},
            'sensors': {
                'lidar_beams': 72, 'lidar_range': 8.0,
                'imu_noise': 0.01, 'odom_noise': 0.005,
            },
            'initial_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        }
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)

        # 传感器配置
        sensor_cfg = self.config.get('sensors', {})
        self.lidar_beams = sensor_cfg.get('lidar_beams', 72)
        self.lidar_range = sensor_cfg.get('lidar_range', 8.0)
        self.lidar_noise = sensor_cfg.get('lidar_noise', 0.02)  # m
        self.imu_noise = sensor_cfg.get('imu_noise', 0.01)     # rad/s
        self.odom_noise = sensor_cfg.get('odom_noise', 0.005)  # m

        # 初始位姿
        init_pose = self.config.get('initial_pose', {})
        self._x = init_pose.get('x', 0.0)
        self._y = init_pose.get('y', 0.0)
        self._yaw = init_pose.get('yaw', 0.0)

        # 速度状态
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        # 电池状态
        battery_cfg = self.config.get('battery', {})
        self._battery_percent = battery_cfg.get('initial_percent', 1.0)
        self._battery_voltage = battery_cfg.get('nominal_voltage', 12.0)
        self._battery_drain_rate = battery_cfg.get('drain_rate', 0.00002)  # per cmd
        self._battery = BatteryState(percent=self._battery_percent)

        # 仿真环境（障碍物列表，用于 LiDAR 模拟）
        self._obstacles: list = []  # [(xmin,ymin,xmax,ymax), ...]
        self._dynamic_obstacles: list = []  # [{x,y,r}, ...]

        # 线程安全
        self._lock = threading.Lock()
        self._running = False
        self._sim_thread: Optional[threading.Thread] = None
        self._sim_dt = 1.0 / 30.0  # 30Hz 仿真步长

        # 心跳（用于看门狗检测线程存活）
        self._heartbeat = 0.0
        self._sim_error_count = 0
        self._sim_last_error = ""

        # IMU 噪声累积
        self._imu_bias = np.zeros(3)
        self._odom_bias = np.zeros(3)

        # 统计
        self._sim_time = 0.0
        self._faults: list = []

    def initialize(self) -> bool:
        """初始化 Mock 硬件"""
        self._initialized = True
        self._running = True
        # 启动仿真线程
        self._sim_thread = threading.Thread(
            target=self._sim_loop, daemon=True, name='MockHW-Sim')
        self._sim_thread.start()
        return True

    def shutdown(self):
        """关闭 Mock 硬件"""
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=1.0)
        self._initialized = False

    def set_environment(self, obstacles: list, dynamic_obstacles: list = None):
        """设置仿真环境障碍物

        Args:
            obstacles: 静态障碍物列表 [(xmin,ymin,xmax,ymax), ...]
            dynamic_obstacles: 动态障碍物列表 [{x,y,r}, ...]
        """
        with self._lock:
            self._obstacles = list(obstacles)
            self._dynamic_obstacles = list(dynamic_obstacles or [])

    # === 运动控制实现 ===
    def _dispatch_motion(self, vx: float, vy: float, wz: float):
        """将速度命令分发到仿真运动学"""
        with self._lock:
            self._vx = vx
            self._vy = vy
            self._wz = wz
            # 电池消耗（移动时消耗更多）
            speed = math.sqrt(vx*vx + vy*vy + wz*wz*0.1)
            self._battery_percent = max(
                0.0, self._battery_percent - self._battery_drain_rate * (1.0 + speed))

    # === 传感器读取实现 ===
    def get_sensor_readings(self) -> SensorReadings:
        """读取仿真传感器数据"""
        with self._lock:
            x, y, yaw = self._x, self._y, self._yaw
            vx, vy, wz = self._vx, self._vy, self._wz
            # 复制障碍物列表，避免 raycast 期间被 set_environment 修改
            obstacles = list(self._obstacles)
            dynamic_obstacles = list(self._dynamic_obstacles)

        # LiDAR 扫描模拟
        angles = np.linspace(-math.pi, math.pi, self.lidar_beams, endpoint=False)
        ranges = np.full(self.lidar_beams, self.lidar_range)
        for i, angle in enumerate(angles):
            world_angle = angle + yaw
            dist = self._raycast(x, y, world_angle, obstacles, dynamic_obstacles)
            if dist is not None:
                # 加入高斯噪声
                dist += random.gauss(0, self.lidar_noise)
                ranges[i] = min(dist, self.lidar_range)

        # IMU 模拟（带噪声和漂移）
        imu_accel = np.array([vx * math.cos(yaw), vx * math.sin(yaw), 9.81])
        imu_gyro = np.array([
            random.gauss(0, self.imu_noise),
            random.gauss(0, self.imu_noise),
            wz + random.gauss(0, self.imu_noise),
        ])
        # 四元数 (yaw 旋转)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        imu_quat = np.array([cy, 0.0, 0.0, sy])

        # 里程计（带噪声）
        odom_x = x + random.gauss(0, self.odom_noise)
        odom_y = y + random.gauss(0, self.odom_noise)
        odom_yaw = yaw + random.gauss(0, self.odom_noise * 2)

        return SensorReadings(
            lidar_angles=angles,
            lidar_ranges=ranges,
            imu_accel=imu_accel,
            imu_gyro=imu_gyro,
            imu_quat=imu_quat,
            odom_x=odom_x, odom_y=odom_y, odom_yaw=odom_yaw,
            odom_vx=vx, odom_vy=vy, odom_wz=wz,
            timestamp=time.time(),
        )

    def get_battery(self) -> BatteryState:
        """读取仿真电池状态"""
        with self._lock:
            percent = self._battery_percent

        # 模拟电压（电量越低电压越低）
        voltage = self._battery_voltage * (0.85 + 0.15 * percent)
        # 模拟电流（移动时电流大）
        current = abs(self._vx) * 2.0 + abs(self._wz) * 0.5 + 0.3

        batt = BatteryState(
            voltage=voltage,
            current=current,
            percent=percent,
            charging=percent < 0.0,  # mock 不充电
            temperature=25.0 + current * 0.5,
        )
        batt.update_thresholds(
            self.low_battery_threshold,
            self.critical_battery_threshold)
        self._battery = batt
        return batt

    def get_health(self) -> RobotHealthState:
        """读取仿真健康状态"""
        with self._lock:
            faults = list(self._faults)
        level = 'OK'
        ok = True
        if self._emergency_stopped:
            level = 'FATAL'
            ok = False
            faults.append('EMERGENCY_STOP')
        elif self._battery.critical_battery:
            level = 'ERROR'
            ok = False
            faults.append('CRITICAL_BATTERY')
        elif self._battery.low_battery:
            level = 'WARN'
            faults.append('LOW_BATTERY')

        return RobotHealthState(
            ok=ok, level=level, active_faults=faults,
            cpu_temp=45.0 + random.uniform(-2, 2),
            imu_ready=True, lidar_ready=True,
            camera_ready=True, motion_ready=self._motors_enabled,
        )

    def inject_fault(self, fault: str):
        """注入故障（用于故障注入测试）

        Args:
            fault: 故障类型 ('sensor_timeout' / 'imu_drift' / 'battery_drain')
        """
        with self._lock:
            self._faults.append(fault)
        if fault == 'emergency_stop':
            self.emergency_stop("fault_injection")

    def clear_fault(self, fault: str):
        """清除注入的故障"""
        with self._lock:
            if fault in self._faults:
                self._faults.remove(fault)

    # === 仿真循环 ===
    def _sim_loop(self):
        """仿真主循环（30Hz）

        异常保护:
            - 单帧异常不会导致线程退出
            - 连续异常超过阈值时标记线程为故障状态
            - 通过心跳时间戳供看门狗检测线程存活
        """
        last_time = time.time()
        consecutive_errors = 0
        MAX_CONSECUTIVE_ERRORS = 10

        while self._running:
            try:
                now = time.time()
                dt = now - last_time
                last_time = now
                # 限制 dt 防止大跳变（如系统休眠后恢复）
                dt = min(dt, self._sim_dt * 5)

                with self._lock:
                    # 差速驱动运动学更新
                    if self._motors_enabled and not self._emergency_stopped:
                        self._x += self._vx * math.cos(self._yaw) * dt
                        self._y += self._vx * math.sin(self._yaw) * dt
                        self._yaw += self._wz * dt
                        # 归一化 yaw
                        while self._yaw > math.pi:
                            self._yaw -= 2 * math.pi
                        while self._yaw < -math.pi:
                            self._yaw += 2 * math.pi

                    self._sim_time += dt

                # 更新心跳（锁外更新，避免不必要的锁竞争）
                self._heartbeat = time.time()
                consecutive_errors = 0  # 重置错误计数

                # 精确睡眠以保持仿真频率
                sleep_time = self._sim_dt - (time.time() - now)
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except Exception as e:
                consecutive_errors += 1
                self._sim_error_count += 1
                self._sim_last_error = str(e)
                # 更新心跳（即使出错也更新，表示线程还活着）
                self._heartbeat = time.time()
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    # 连续异常过多，注入故障状态
                    with self._lock:
                        if 'SIM_THREAD_ERROR' not in self._faults:
                            self._faults.append('SIM_THREAD_ERROR')
                    # 不退出，继续重试
                # 短暂休眠避免错误时 CPU 空转
                time.sleep(0.01)

    def _raycast(self, ox: float, oy: float, angle: float,
                 obstacles: list = None, dynamic_obstacles: list = None
                 ) -> Optional[float]:
        """射线投射模拟 LiDAR

        Args:
            ox, oy: 机器人位置
            angle: 射线角度 (世界坐标系)
            obstacles: 静态障碍物列表 (传入避免锁竞争)
            dynamic_obstacles: 动态障碍物列表

        Returns:
            到障碍物的距离，无命中返回 None
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        min_dist = self.lidar_range

        # 使用传入的障碍物列表（避免访问共享状态）
        obstacles = obstacles if obstacles is not None else self._obstacles
        dynamic_obstacles = (dynamic_obstacles if dynamic_obstacles is not None
                              else self._dynamic_obstacles)

        # 静态障碍物 AABB 求交
        for obs in obstacles:
            if len(obs) < 4:
                continue
            xmin, ymin, xmax, ymax = obs[0], obs[1], obs[2], obs[3]
            dist = self._ray_aabb(ox, oy, dx, dy, xmin, ymin, xmax, ymax)
            if dist is not None and dist < min_dist:
                min_dist = dist

        # 动态障碍物圆形求交
        for obs in dynamic_obstacles:
            cx, cy, r = obs.get('x', 0), obs.get('y', 0), obs.get('r', 0.3)
            dist = self._ray_circle(ox, oy, dx, dy, cx, cy, r)
            if dist is not None and dist < min_dist:
                min_dist = dist

        return min_dist if min_dist < self.lidar_range else None

    @staticmethod
    def _ray_aabb(ox, oy, dx, dy, xmin, ymin, xmax, ymax) -> Optional[float]:
        """射线与 AABB 求交 (Slab 法)"""
        inv_dx = 1.0 / dx if abs(dx) > 1e-9 else float('inf')
        inv_dy = 1.0 / dy if abs(dy) > 1e-9 else float('inf')

        t1 = (xmin - ox) * inv_dx
        t2 = (xmax - ox) * inv_dx
        t3 = (ymin - oy) * inv_dy
        t4 = (ymax - oy) * inv_dy

        tmin = max(min(t1, t2), min(t3, t4))
        tmax = min(max(t1, t2), max(t3, t4))

        if tmax < 0 or tmin > tmax:
            return None
        if tmin < 0:
            return tmax if tmax > 0 else None
        return tmin

    @staticmethod
    def _ray_circle(ox, oy, dx, dy, cx, cy, r) -> Optional[float]:
        """射线与圆形求交"""
        ocx = ox - cx
        ocy = oy - cy
        b = ocx * dx + ocy * dy
        c = ocx * ocx + ocy * ocy - r * r
        disc = b * b - c
        if disc < 0:
            return None
        t = -b - math.sqrt(disc)
        if t > 0.1:
            return t
        t = -b + math.sqrt(disc)
        if t > 0.1:
            return t
        return None

    @property
    def pose(self) -> tuple:
        """获取当前位姿 (用于仿真验证)"""
        with self._lock:
            return (self._x, self._y, self._yaw)

    @property
    def sim_time(self) -> float:
        """仿真时间"""
        return self._sim_time
