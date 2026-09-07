#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
硬件适配层测试 (Hardware Adapter Tests)
========================================
测试 puppypi_adapter 的所有模块:
  1. hardware_interface: 硬件抽象基类
  2. mock_interface: 仿真 Mock 实现
  3. puppypi_driver: PuppyPi 实体机驱动
  4. safety_subsystem_v2: 三级急停+分级告警
  5. sensor_calibration: 传感器校准
  6. fault_injection: 故障注入
  7. performance_monitor: 性能监控+压力测试+Fuzz

运行:
  python -m pytest test_hardware_adapter.py -v
"""
import os
import sys
import math
import time
import json
import tempfile
import unittest
import numpy as np
from pathlib import Path

# 添加路径
PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT / "src" / "puppypi_adapter"))

from puppypi_adapter.hardware_interface import (
    HardwareInterface, SensorReadings, BatteryState, RobotHealthState, Watchdog
)
from puppypi_adapter.mock_interface import MockHardwareInterface
from puppypi_adapter.sim_interface import SimHardwareInterface
from puppypi_adapter.safety_subsystem_v2 import (
    SafetySubsystem, AlertLevel, EStopSource, DegradationMode,
    DegradationPolicy, AlertEvent
)
from puppypi_adapter.sensor_calibration import (
    SensorCalibrator, ExtrinsicCalibration, CameraIntrinsics,
    OdometryCalibration
)
from puppypi_adapter.fault_injection import (
    FaultInjector, FaultType, FaultSeverity, FaultInjection
)
from puppypi_adapter.performance_monitor import (
    PerformanceMonitor, StressTester, SensorFuzzer, FrameTiming
)
# puppypi_driver 需要 RPi.GPIO 等，在非树莓派环境跳过相关测试
try:
    from puppypi_adapter.puppypi_driver import PuppyPiHardwareInterface
    _HAS_PUPPYPI = True
except Exception:
    _HAS_PUPPYPI = False

try:
    import rclpy
    _HAS_RCLPY = True
except Exception:
    _HAS_RCLPY = False



class TestHardwareInterface(unittest.TestCase):
    """测试硬件抽象接口"""

    def test_create_mock(self):
        """测试工厂方法创建 Mock 接口"""
        config = {'backend': 'mock', 'motion': {'max_linear_x': 0.3}}
        hw = HardwareInterface.create(config)
        self.assertIsInstance(hw, MockHardwareInterface)
        self.assertEqual(hw.max_linear_x, 0.3)

    def test_create_sim(self):
        """测试创建仿真接口 (P0-4: sim 是独立后端, 不再是 mock 的别名)"""
        config = {'backend': 'sim'}
        hw = HardwareInterface.create(config)
        self.assertIsNotNone(hw)
        self.assertIsInstance(hw, SimHardwareInterface)
        # sim 后端禁止伪装成 mock
        self.assertNotIsInstance(hw, MockHardwareInterface)

    def test_sim_backend_does_not_fall_back_to_mock(self):
        """P0-4 关键回归: 请求 sim 绝不能静默得到 mock

        历史缺陷: sim_interface 缺失时 create() 用 try/except ImportError
        静默回退 mock, 导致"配置为仿真、实际无执行器"被长期掩盖。
        """
        hw = HardwareInterface.create({'backend': 'sim'})
        self.assertEqual(hw.stats.get('backend'), 'sim')

    def test_sim_interface_never_fabricates_sensor_data(self):
        """P0-4: sim 后端未注入数据时返回空读数, 不合成"合理"假数据"""
        hw = SimHardwareInterface({'backend': 'sim'})
        hw.initialize()
        readings = hw.get_sensor_readings()
        # 未注入 -> 空, 而不是带噪声的合成数据
        self.assertEqual(len(readings.lidar_ranges), 0)
        self.assertFalse(hw.get_health().lidar_ready)

    def test_sim_command_forwarding(self):
        """P0-4: sim 后端把速度命令转发给外部仿真器 sink"""
        received = []
        hw = SimHardwareInterface(
            {'backend': 'sim', 'motion': {'accel_limit': 100.0}})
        hw.initialize()
        hw.set_command_sink(lambda vx, vy, wz: received.append((vx, vy, wz)))
        hw.enable_motors(True)
        # 基类 send_velocity 带加速度限幅 (dt 下限 1ms), 需连续多帧才收敛到目标;
        # 这本身是正确行为, 因此按真实控制环连续发送而非期望单帧到位
        for _ in range(5):
            hw.send_velocity(0.2, 0.0, 0.1)
        self.assertGreaterEqual(len(received), 1)
        # 每一帧都转发给外部仿真器
        self.assertEqual(len(received), 5)
        # 末帧应收敛到目标速度
        self.assertAlmostEqual(received[-1][0], 0.2, places=3)
        self.assertTrue(hw.sim_connected)

    def test_default_backend_is_mock(self):
        """P0-4: 默认后端由 sim 改为 mock (诚实默认, 无外部执行器)"""
        hw = HardwareInterface.create({})
        self.assertIsInstance(hw, MockHardwareInterface)

    def test_real_backend_fails_fast_without_hardware(self):
        """P0-4 / Task 8: real 后端缺少硬件 SDK 时必须 fail-fast，严禁静默回退到 mock"""
        from puppypi_adapter.puppypi_driver import PuppyPiHardwareInterface
        hw = HardwareInterface.create({'backend': 'real'})
        self.assertIsInstance(hw, PuppyPiHardwareInterface)
        self.assertNotIsInstance(hw, MockHardwareInterface)
        # 在非树莓派或缺少硬件环境下，initialize 必须返回 False 且记录错误，不能静默假装成功
        success = hw.initialize()
        self.assertFalse(success)
        self.assertFalse(hw._initialized)
        self.assertIn('preflight failed', hw._last_error.lower())

    def test_unknown_backend_raises(self):
        """未知 backend 应抛出 ValueError"""
        with self.assertRaises(ValueError):
            HardwareInterface.create({'backend': 'unknown'})

    def test_velocity_limiting(self):
        """测试速度限幅"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        hw.enable_motors(True)
        # 设置超限速度
        hw.send_velocity(10.0, 0, 0)  # 远超 max_linear_x=0.3
        # 内部应该被限幅
        vx, vy, wz = hw._last_velocity
        self.assertLessEqual(abs(vx), 0.3 + 0.01)
        hw.shutdown()

    def test_accel_limit(self):
        """测试加速度限制"""
        hw = MockHardwareInterface({
            'backend': 'mock',
            'motion': {'accel_limit': 2.0, 'max_linear_x': 1.0}
        })
        hw.initialize()
        hw.enable_motors(True)
        # 从 0 加速到 1.0
        hw.send_velocity(1.0, 0, 0)
        vx1, _, _ = hw._last_velocity
        # 第一次应该被加速度限制（远小于 1.0）
        self.assertLess(vx1, 1.0)
        hw.shutdown()

    def test_emergency_stop(self):
        """测试紧急停止"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        hw.enable_motors(True)
        hw.send_velocity(0.2, 0, 0)
        # 急停
        hw.emergency_stop("test")
        self.assertTrue(hw.emergency_stopped)
        self.assertFalse(hw.motors_enabled)
        # 急停后再发送命令应该不执行（速度为 0）
        hw.send_velocity(0.2, 0, 0)
        vx, vy, wz = hw._last_velocity
        self.assertLess(abs(vx), 0.01)  # 允许加速度残差
        # 释放
        self.assertTrue(hw.release_emergency_stop())
        self.assertFalse(hw.emergency_stopped)
        hw.shutdown()

    def test_command_timeout(self):
        """测试命令超时检测"""
        hw = MockHardwareInterface({
            'backend': 'mock',
            'motion': {'cmd_timeout': 0.1}
        })
        hw.initialize()
        hw.enable_motors(True)
        hw.send_velocity(0.1, 0, 0)
        self.assertFalse(hw.command_timeout)
        # 等待超时
        time.sleep(0.15)
        self.assertTrue(hw.command_timeout)
        hw.shutdown()


class TestMockInterface(unittest.TestCase):
    """测试 Mock 硬件接口"""

    def setUp(self):
        self.hw = MockHardwareInterface({
            'backend': 'mock',
            'sensors': {'lidar_beams': 36, 'lidar_range': 8.0},
            'initial_pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.5},
        })
        self.hw.initialize()

    def tearDown(self):
        self.hw.shutdown()

    def test_initial_pose(self):
        """测试初始位姿"""
        x, y, yaw = self.hw.pose
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(yaw, 0.5)

    def test_sensor_readings(self):
        """测试传感器读取"""
        readings = self.hw.get_sensor_readings()
        self.assertIsInstance(readings, SensorReadings)
        self.assertEqual(len(readings.lidar_angles), 36)
        self.assertEqual(len(readings.lidar_ranges), 36)
        self.assertEqual(len(readings.imu_accel), 3)
        self.assertEqual(len(readings.imu_gyro), 3)

    def test_battery(self):
        """测试电池状态"""
        batt = self.hw.get_battery()
        self.assertIsInstance(batt, BatteryState)
        self.assertGreater(batt.percent, 0)
        self.assertGreater(batt.voltage, 0)

    def test_health(self):
        """测试健康状态"""
        health = self.hw.get_health()
        self.assertIsInstance(health, RobotHealthState)
        self.assertTrue(health.ok)
        self.assertEqual(health.level, 'OK')

    def test_motion_updates_pose(self):
        """测试运动更新位姿"""
        initial_x = self.hw.pose[0]
        self.hw.enable_motors(True)
        self.hw.send_velocity(0.2, 0, 0)
        time.sleep(0.2)  # 让仿真线程运行
        new_x = self.hw.pose[0]
        # 应该向前移动
        self.assertGreater(new_x, initial_x)

    def test_environment_obstacles(self):
        """测试环境障碍物设置"""
        self.hw.set_environment(
            obstacles=[(2.0, -1.0, 3.0, 1.0)],
            dynamic_obstacles=[{'x': 1.0, 'y': 1.0, 'r': 0.3}]
        )
        # 射线应该检测到障碍物
        readings = self.hw.get_sensor_readings()
        # 部分扫描点应该 < max_range
        self.assertTrue(np.any(readings.lidar_ranges < 8.0))

    def test_fault_injection(self):
        """测试故障注入接口"""
        self.hw.inject_fault('emergency_stop')
        self.assertTrue(self.hw.emergency_stopped)
        self.hw.clear_fault('emergency_stop')


class TestSafetySubsystem(unittest.TestCase):
    """测试安全子系统"""

    def setUp(self):
        self.hw = MockHardwareInterface({'backend': 'mock'})
        self.hw.initialize()
        self.safety = SafetySubsystem(self.hw)
        self.safety.start()

    def tearDown(self):
        self.safety.stop()
        self.hw.shutdown()

    def test_emergency_stop_software(self):
        """测试软件 API 急停 (Level 1)"""
        self.safety.emergency_stop("test", EStopSource.SOFTWARE_API)
        self.assertTrue(self.safety.is_emergency_stopped())
        self.assertTrue(self.hw.emergency_stopped)

    def test_emergency_stop_release(self):
        """测试急停释放"""
        self.safety.emergency_stop("test", EStopSource.SOFTWARE_API)
        self.assertTrue(self.safety.release_emergency_stop())
        self.assertFalse(self.safety.is_emergency_stopped())

    def test_remote_shutdown_no_release(self):
        """测试远程关机急停不可自动释放"""
        self.safety.emergency_stop("remote", EStopSource.REMOTE_SHUTDOWN)
        self.assertFalse(self.safety.release_emergency_stop())

    def test_alert_levels(self):
        """测试分级告警"""
        self.safety.raise_alert(AlertLevel.WARN, "test", "warning message")
        self.safety.raise_alert(AlertLevel.ERROR, "test", "error message")
        alerts = self.safety.active_alerts
        self.assertEqual(len(alerts), 2)

    def test_degradation_warn(self):
        """测试 WARN 降级"""
        self.safety.raise_alert(AlertLevel.WARN, "sensor", "noise")
        self.assertEqual(self.safety.degradation_mode, DegradationMode.REDUCED_SPEED)
        self.assertAlmostEqual(self.safety.get_speed_limit(), 0.5)

    def test_degradation_error(self):
        """测试 ERROR 降级"""
        self.safety.raise_alert(AlertLevel.ERROR, "motor", "fault")
        self.assertGreaterEqual(self.safety.degradation_mode, DegradationMode.SAFE_STOP)

    def test_degradation_fatal(self):
        """测试 FATAL 触发急停"""
        self.safety.raise_alert(AlertLevel.FATAL, "system", "crash")
        self.assertTrue(self.safety.is_emergency_stopped())

    def test_collision_risk_detection(self):
        """测试碰撞风险检测"""
        # 前方 0.1m 有障碍物
        ranges = [10.0] * 36
        ranges[17:19] = [0.1, 0.1]  # 前方
        result = self.safety.check_collision_risk(ranges, min_safe_distance=0.3)
        self.assertTrue(result)
        self.assertTrue(self.safety.is_emergency_stopped())

    def test_risk_assessment(self):
        """测试风险评估"""
        result = self.safety.assess_risk({
            'min_obstacle_dist': 0.5,
            'speed': 0.2,
        })
        self.assertIn('score', result)
        self.assertIn('ttc', result)
        self.assertGreater(result['ttc'], 0)

    def test_report_callback(self):
        """测试自动上报回调"""
        received = []
        self.safety.add_report_callback(lambda alert: received.append(alert))
        self.safety.raise_alert(AlertLevel.WARN, "test", "msg")
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].level, AlertLevel.WARN)

    def test_export_alerts(self):
        """测试告警导出"""
        self.safety.raise_alert(AlertLevel.WARN, "test1", "msg1")
        self.safety.raise_alert(AlertLevel.ERROR, "test2", "msg2")
        exported = self.safety.export_alerts()
        self.assertEqual(len(exported), 2)
        self.assertIn('level', exported[0])

    def test_stats(self):
        """测试统计"""
        self.safety.emergency_stop("test")
        stats = self.safety.stats
        self.assertEqual(stats['estop_count'], 1)
        self.assertTrue(stats['estop_active'])


class TestSensorCalibration(unittest.TestCase):
    """测试传感器校准"""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.calibrator = SensorCalibrator(self.tmpdir)

    def test_lidar_imu_static(self):
        """测试 LiDAR-IMU 静态标定"""
        # 模拟重力读数 (有 roll/pitch 偏差)
        g = np.array([0.5, 0.3, -9.5])
        result = self.calibrator.calibrate_lidar_imu_static(g, lidar_mount_angle=0.1)
        self.assertIsInstance(result, ExtrinsicCalibration)
        self.assertEqual(result.method, "static_gravity")
        # 旋转矩阵应该是有效的
        self.assertEqual(result.rotation.shape, (3, 3))
        # 行列式接近 1
        det = np.linalg.det(result.rotation)
        self.assertAlmostEqual(det, 1.0, places=2)

    def test_lidar_imu_handeye(self):
        """测试手眼标定"""
        # 生成模拟旋转序列
        rotations = []
        for i in range(5):
            angle = i * 0.1
            R = np.array([
                [math.cos(angle), -math.sin(angle), 0],
                [math.sin(angle), math.cos(angle), 0],
                [0, 0, 1]
            ])
            rotations.append(R)
        result = self.calibrator.calibrate_lidar_imu_handeye(rotations, rotations)
        self.assertIsInstance(result, ExtrinsicCalibration)
        self.assertEqual(result.method, "handeye_tsai_lenz")

    def test_camera_calibration(self):
        """测试相机标定"""
        # 生成模拟棋盘格图像
        images = [np.random.randint(0, 255, (480, 640), dtype=np.uint8)
                  for _ in range(5)]
        result = self.calibrator.calibrate_camera(images)
        self.assertIsInstance(result, CameraIntrinsics)
        self.assertGreater(result.fx, 0)
        self.assertGreater(result.fy, 0)

    def test_odometry_calibration(self):
        """测试里程计标定"""
        # 模拟编码器读数 (已知距离 1m)
        encoders = [(1400, 1410), (1395, 1405), (1410, 1400)]
        result = self.calibrator.calibrate_odometry(encoders, known_distance=1.0)
        self.assertIsInstance(result, OdometryCalibration)
        self.assertGreater(result.left_scale, 0)
        self.assertGreater(result.right_scale, 0)

    def test_save_load_calibration(self):
        """测试标定结果保存/加载"""
        # 先标定
        g = np.array([0, 0, -9.81])
        self.calibrator.calibrate_lidar_imu_static(g)
        # 保存
        self.calibrator.save_calibration("test")
        # 新实例加载
        calibrator2 = SensorCalibrator(self.tmpdir)
        self.assertTrue(calibrator2.load_calibration("test"))
        self.assertIsNotNone(calibrator2.extrinsic)

    def test_is_calibrated(self):
        """测试标定状态检查"""
        self.assertFalse(self.calibrator.is_calibrated())
        g = np.array([0, 0, -9.81])
        self.calibrator.calibrate_lidar_imu_static(g)
        # 只有 extrinsic，没有 intrinsics
        self.assertFalse(self.calibrator.is_calibrated())


class TestFaultInjection(unittest.TestCase):
    """测试故障注入"""

    def setUp(self):
        self.hw = MockHardwareInterface({'backend': 'mock'})
        self.hw.initialize()
        self.injector = FaultInjector(self.hw)

    def tearDown(self):
        self.injector.clear_all()
        self.hw.shutdown()

    def test_inject_lidar_noise(self):
        """测试 LiDAR 噪声注入"""
        self.injector.inject(FaultType.LIDAR_NOISE, severity=FaultSeverity.MODERATE)
        # 读取数据应该有噪声
        readings = self.injector.get_sensor_readings()
        self.assertIsNotNone(readings)
        self.assertIn(FaultType.LIDAR_NOISE, self.injector.get_active_faults())

    def test_inject_imu_drift(self):
        """测试 IMU 漂移注入"""
        self.injector.inject(FaultType.IMU_DRIFT, severity=FaultSeverity.SEVERE)
        time.sleep(0.1)  # 等待漂移累积
        readings = self.injector.get_sensor_readings()
        self.assertIsNotNone(readings)

    def test_inject_duration(self):
        """测试定时故障"""
        self.injector.inject(FaultType.LIDAR_NOISE, duration=0.3)
        self.assertIn(FaultType.LIDAR_NOISE, self.injector.get_active_faults())
        time.sleep(0.5)  # 等待超时
        self.assertNotIn(FaultType.LIDAR_NOISE, self.injector.get_active_faults())

    def test_clear_fault(self):
        """测试清除故障"""
        self.injector.inject(FaultType.IMU_DRIFT)
        self.injector.clear(FaultType.IMU_DRIFT)
        self.assertNotIn(FaultType.IMU_DRIFT, self.injector.get_active_faults())

    def test_multiple_faults(self):
        """测试多重故障"""
        self.injector.inject(FaultType.LIDAR_NOISE)
        self.injector.inject(FaultType.IMU_DRIFT)
        self.injector.inject(FaultType.ENCODER_JUMP)
        active = self.injector.get_active_faults()
        self.assertEqual(len(active), 3)

    def test_stats(self):
        """测试统计"""
        self.injector.inject(FaultType.LIDAR_NOISE)
        self.injector.record_detection(FaultType.LIDAR_NOISE)
        self.injector.record_recovery(FaultType.LIDAR_NOISE)
        stats = self.injector.stats
        self.assertEqual(stats['injection_count'], 1)
        self.assertEqual(stats['detection_count'], 1)
        self.assertEqual(stats['recovery_count'], 1)

    def test_report(self):
        """测试报告生成"""
        self.injector.inject(FaultType.LIDAR_NOISE)
        report = self.injector.generate_report()
        self.assertIn('summary', report)
        self.assertIn('fault_history', report)
        self.assertIn('conclusion', report)


class TestPerformanceMonitor(unittest.TestCase):
    """测试性能监控"""

    def test_frame_timing(self):
        """测试帧计时"""
        monitor = PerformanceMonitor(target_fps=30)
        monitor.start_frame()
        time.sleep(0.01)
        frame = monitor.end_frame()
        self.assertIsNotNone(frame)
        self.assertGreater(frame.duration, 0.005)
        self.assertLess(frame.duration, 0.1)

    def test_warning_threshold(self):
        """测试超时告警"""
        monitor = PerformanceMonitor(warning_threshold=0.01)
        monitor.start_frame()
        time.sleep(0.03)  # 超过 10ms
        frame = monitor.end_frame()
        self.assertTrue(frame.exceeded_warning)
        self.assertGreater(monitor.stats['warning_frames'], 0)

    def test_degradation(self):
        """测试降级"""
        monitor = PerformanceMonitor(target_fps=10)
        # 模拟低 FPS
        for i in range(30):
            monitor.start_frame()
            time.sleep(0.15)  # 150ms → ~6.7 FPS
            monitor.end_frame()
        # 应该降级
        self.assertTrue(monitor.is_degraded())
        self.assertLess(monitor.get_speed_factor(), 1.0)

    def test_timeout_callback(self):
        """测试超时回调"""
        monitor = PerformanceMonitor(warning_threshold=0.01)
        received = []
        monitor.add_timeout_callback(lambda f: received.append(f))
        monitor.start_frame()
        time.sleep(0.03)
        monitor.end_frame()
        self.assertEqual(len(received), 1)

    def test_stats(self):
        """测试统计"""
        monitor = PerformanceMonitor()
        for i in range(10):
            monitor.start_frame()
            time.sleep(0.005)
            monitor.end_frame()
        stats = monitor.stats
        self.assertEqual(stats['total_frames'], 10)
        self.assertGreater(stats['avg_fps'], 0)


class TestStressTester(unittest.TestCase):
    """测试压力测试器"""

    def test_short_run(self):
        """测试短时间运行"""
        tester = StressTester(duration_hours=0.001, cycle_interval=0.01)  # ~3.6s
        results = tester.run(lambda: None)
        self.assertIn('duration_hours', results)
        self.assertIn('total_cycles', results)
        self.assertGreater(results['total_cycles'], 0)

    def test_memory_leak_detection(self):
        """测试内存泄漏检测"""
        tester = StressTester(duration_hours=0.001, cycle_interval=0.01)
        results = tester.run(lambda: None)
        self.assertIn('memory_leak_detected', results)
        self.assertIn('memory_leak_rate_mb_per_hour', results)

    def test_verdict(self):
        """测试结论生成"""
        tester = StressTester(duration_hours=0.001, cycle_interval=0.01)
        results = tester.run(lambda: None)
        self.assertIn('verdict', results)


class TestSensorFuzzer(unittest.TestCase):
    """测试 Fuzz 测试"""

    def setUp(self):
        self.fuzzer = SensorFuzzer(noise_level=0.1, mutation_rate=0.1)
        self.fuzzer.start()

    def tearDown(self):
        self.fuzzer.stop()

    def test_fuzz_lidar(self):
        """测试 LiDAR Fuzz"""
        original = np.array([5.0, 3.0, 8.0, 2.0])
        fuzzed = self.fuzzer.fuzz_lidar(original)
        self.assertEqual(len(fuzzed), len(original))
        # 应该有变化（噪声或突变）
        self.assertFalse(np.array_equal(fuzzed, original))

    def test_fuzz_imu(self):
        """测试 IMU Fuzz"""
        accel = np.array([0.1, 0.2, 9.81])
        gyro = np.array([0.01, 0.02, 0.03])
        a_fuzzed, g_fuzzed = self.fuzzer.fuzz_imu(accel, gyro)
        self.assertEqual(len(a_fuzzed), 3)
        self.assertEqual(len(g_fuzzed), 3)

    def test_fuzz_odometry(self):
        """测试里程计 Fuzz"""
        x, y, yaw = self.fuzzer.fuzz_odometry(1.0, 2.0, 0.5)
        # 应该有变化
        self.assertTrue(abs(x - 1.0) > 0 or abs(y - 2.0) > 0 or abs(yaw - 0.5) > 0)

    def test_report(self):
        """测试报告"""
        for _ in range(100):
            self.fuzzer.fuzz_lidar(np.array([5.0, 3.0, 8.0]))
        report = self.fuzzer.generate_report()
        self.assertGreater(report['total_tests'], 0)
        self.assertIn('verdict', report)


class TestStabilityWatchdog(unittest.TestCase):
    """测试看门狗机制"""

    def test_watchdog_register_and_heartbeat(self):
        """测试注册和心跳"""
        wd = Watchdog(timeout=0.5)
        wd.register('test_thread')
        self.assertTrue(wd.is_alive('test_thread'))
        # 更新心跳
        wd.heartbeat('test_thread')
        self.assertTrue(wd.is_alive('test_thread'))

    def test_watchdog_timeout_detection(self):
        """测试超时检测"""
        wd = Watchdog(timeout=0.05)  # 50ms 超时
        wd.register('test_thread')
        # 等待超时
        time.sleep(0.1)
        self.assertFalse(wd.is_alive('test_thread'))
        # check 应返回超时的线程
        dead = wd.check()
        self.assertIn('test_thread', dead)

    def test_watchdog_callback(self):
        """测试超时回调"""
        wd = Watchdog(timeout=0.05)
        received = []
        wd.add_timeout_callback(lambda name: received.append(name))
        wd.register('test_thread')
        time.sleep(0.1)
        wd.check()
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0], 'test_thread')

    def test_watchdog_no_duplicate_callback(self):
        """测试不重复触发回调"""
        wd = Watchdog(timeout=0.05)
        received = []
        wd.add_timeout_callback(lambda name: received.append(name))
        wd.register('test_thread')
        time.sleep(0.1)
        wd.check()  # 第一次应触发
        wd.check()  # 第二次不应重复触发
        self.assertEqual(len(received), 1)

    def test_watchdog_status(self):
        """测试状态查询"""
        wd = Watchdog(timeout=0.5)
        wd.register('thread1')
        wd.register('thread2')
        status = wd.get_status()
        self.assertIn('thread1', status)
        self.assertIn('thread2', status)
        self.assertTrue(status['thread1']['alive'])

    def test_watchdog_unknown_thread(self):
        """测试未注册线程"""
        wd = Watchdog(timeout=0.5)
        self.assertFalse(wd.is_alive('unknown'))


class TestStabilityConfigValidation(unittest.TestCase):
    """测试配置参数校验"""

    def test_valid_config(self):
        """测试合法配置"""
        hw = MockHardwareInterface({
            'backend': 'mock',
            'motion': {'max_linear_x': 0.3, 'max_angular_z': 1.2,
                        'accel_limit': 2.0, 'cmd_timeout': 1.0},
            'battery': {'low_threshold': 0.2, 'critical_threshold': 0.1},
        })
        self.assertEqual(hw.max_linear_x, 0.3)

    def test_negative_speed_rejected(self):
        """测试负速度被拒绝"""
        with self.assertRaises(ValueError):
            MockHardwareInterface({
                'backend': 'mock',
                'motion': {'max_linear_x': -0.5},
            })

    def test_zero_accel_rejected(self):
        """测试零加速度被拒绝"""
        with self.assertRaises(ValueError):
            MockHardwareInterface({
                'backend': 'mock',
                'motion': {'accel_limit': 0},
            })

    def test_invalid_battery_threshold_rejected(self):
        """测试非法电池阈值被拒绝"""
        with self.assertRaises(ValueError):
            MockHardwareInterface({
                'backend': 'mock',
                'battery': {'low_threshold': 1.5, 'critical_threshold': 0.1},
            })

    def test_critical_ge_low_rejected(self):
        """测试 critical >= low 被拒绝"""
        with self.assertRaises(ValueError):
            MockHardwareInterface({
                'backend': 'mock',
                'battery': {'low_threshold': 0.1, 'critical_threshold': 0.2},
            })

    def test_negative_cmd_timeout_rejected(self):
        """测试负超时被拒绝"""
        with self.assertRaises(ValueError):
            MockHardwareInterface({
                'backend': 'mock',
                'motion': {'cmd_timeout': -1.0},
            })


class TestStabilityContextManager(unittest.TestCase):
    """测试 Context Manager 资源清理"""

    def test_context_manager_init_shutdown(self):
        """测试 with 语句自动初始化和关闭"""
        with MockHardwareInterface({'backend': 'mock'}) as hw:
            self.assertTrue(hw._initialized)
            hw.enable_motors(True)
            hw.send_velocity(0.1, 0, 0)
        # 退出后应该已关闭
        self.assertFalse(hw._initialized)

    def test_context_manager_exception_cleanup(self):
        """测试异常时资源仍被清理"""
        hw_instance = None
        try:
            with MockHardwareInterface({'backend': 'mock'}) as hw:
                hw_instance = hw
                raise RuntimeError("test exception")
        except RuntimeError:
            pass
        # 确保资源被清理
        self.assertIsNotNone(hw_instance)
        self.assertFalse(hw_instance._initialized)


class TestStabilityThreadSafety(unittest.TestCase):
    """测试线程安全"""

    def test_concurrent_send_velocity(self):
        """测试并发速度命令不崩溃"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        hw.enable_motors(True)
        import threading
        errors = []

        def worker():
            try:
                for _ in range(100):
                    hw.send_velocity(0.1, 0, 0.1)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(len(errors), 0)
        hw.shutdown()

    def test_concurrent_estop_release(self):
        """测试并发急停和释放"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        import threading
        errors = []

        def estopper():
            try:
                for _ in range(50):
                    hw.emergency_stop("test")
                    hw.release_emergency_stop()
            except Exception as e:
                errors.append(e)

        def velocity_sender():
            try:
                for _ in range(50):
                    hw.send_velocity(0.1, 0, 0)
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=estopper),
            threading.Thread(target=estopper),
            threading.Thread(target=velocity_sender),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(len(errors), 0)
        hw.shutdown()

    def test_concurrent_safety_alerts(self):
        """测试并发告警上报"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        safety = SafetySubsystem(hw)
        safety.start()
        import threading
        errors = []

        def alerter(level):
            try:
                for i in range(20):
                    safety.raise_alert(level, f"test_{i}", f"msg_{i}")
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=alerter, args=(AlertLevel.WARN,)),
            threading.Thread(target=alerter, args=(AlertLevel.ERROR,)),
            threading.Thread(target=alerter, args=(AlertLevel.INFO,)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(len(errors), 0)
        safety.stop()
        hw.shutdown()

    def test_safety_concurrent_estop_read(self):
        """测试急停状态并发读写"""
        safety = SafetySubsystem()
        import threading
        errors = []
        stop_flag = [False]

        def reader():
            try:
                while not stop_flag[0]:
                    _ = safety.is_emergency_stopped()
                    _ = safety.is_safe()
                    _ = safety.stats
                    _ = safety.degradation_mode
            except Exception as e:
                errors.append(e)

        def writer():
            try:
                for _ in range(100):
                    safety.emergency_stop("test")
                    safety.release_emergency_stop()
            except Exception as e:
                errors.append(e)
            stop_flag[0] = True

        t1 = threading.Thread(target=reader)
        t2 = threading.Thread(target=writer)
        t1.start()
        t2.start()
        t1.join(timeout=5)
        t2.join()
        self.assertEqual(len(errors), 0)


class TestStabilitySimLoopException(unittest.TestCase):
    """测试仿真循环异常保护"""

    def test_sim_loop_survives_error(self):
        """测试仿真线程在异常后仍存活"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        # 注入会导致 raycast 异常的障碍物
        hw.set_environment([("invalid", "data")])  # 非法格式
        # 读取传感器不应崩溃
        readings = hw.get_sensor_readings()
        self.assertIsNotNone(readings)
        # 仿真线程应该还活着（通过健康检查）
        health = hw.get_health()
        self.assertIsNotNone(health)
        hw.shutdown()

    def test_sim_loop_heartbeat_updates(self):
        """测试仿真心跳正常更新"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        time.sleep(0.1)  # 让仿真线程跑一会儿
        # 心跳应该已更新
        self.assertGreater(hw._heartbeat, 0)
        old_heartbeat = hw._heartbeat
        time.sleep(0.1)
        # 心跳应该变化（仿真线程在更新）
        self.assertGreater(hw._heartbeat, old_heartbeat)
        hw.shutdown()

    def test_collision_nan_filtering(self):
        """测试碰撞检测 NaN 过滤"""
        safety = SafetySubsystem()
        # 构造包含 NaN/Inf 的 LiDAR 数据（72 点）
        # 前方区域 (index 30-42) 中放入近距离障碍
        bad_ranges = [5.0] * 72
        bad_ranges[0] = float('nan')
        bad_ranges[10] = float('inf')
        bad_ranges[20] = -1.0
        bad_ranges[35] = 0.1  # 前方近距离障碍
        bad_ranges[60] = float('nan')
        # 不应崩溃，应正确过滤
        result = safety.check_collision_risk(bad_ranges, min_safe_distance=0.5)
        # 0.1 在前方范围内，应检测到碰撞风险
        self.assertTrue(result)

    def test_collision_all_nan(self):
        """测试全 NaN 数据不崩溃"""
        safety = SafetySubsystem()
        bad_ranges = [float('nan')] * 72
        result = safety.check_collision_risk(bad_ranges)
        self.assertFalse(result)  # 无有效数据，返回无风险

    def test_speed_limit_zero_max_velocity(self):
        """测试 max_linear_x=0 时的除零保护"""
        hw = MockHardwareInterface({'backend': 'mock'})
        hw.initialize()
        hw.max_linear_x = 0.0  # 模拟配置为0
        safety = SafetySubsystem(hw)
        # 触发 SENSORS_DOWN 降级
        safety.raise_alert(AlertLevel.ERROR, "sensor_lidar", "fault")
        # get_speed_limit 不应崩溃
        limit = safety.get_speed_limit()
        self.assertGreater(limit, 0)
        self.assertLessEqual(limit, 1.0)
        hw.shutdown()


@unittest.skipUnless(_HAS_RCLPY, "requires rclpy (ROS2 Python)")
class TestAdapterNodes(unittest.TestCase):
    """测试 StatusAdapterNode 与 ModeAdapterNode 节点封装"""

    @classmethod
    def setUpClass(cls):
        if _HAS_RCLPY and not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if _HAS_RCLPY and rclpy.ok():
            rclpy.shutdown()

    def test_status_adapter_12_dof(self):
        """测试 StatusAdapterNode 包含 12 DOF 关节视角"""
        from puppypi_adapter.status_adapter_node import StatusAdapterNode
        node = StatusAdapterNode()
        self.assertEqual(len(node.joint_names), 12)
        self.assertIn('FR_hip_yaw_joint', node.joint_names)
        self.assertIn('RL_knee_joint', node.joint_names)
        if node.hw:
            node.hw.shutdown()
        node.destroy_node()

    def test_status_adapter_fallback(self):
        """测试 use_sim=False 时的优雅降级"""
        from puppypi_adapter.status_adapter_node import StatusAdapterNode
        node = StatusAdapterNode()
        node.use_sim = False
        # 不引发未捕获的 ImportError
        node._poll_status()
        if node.hw:
            node.hw.shutdown()
        node.destroy_node()

    def test_mode_adapter_posture_commands(self):
        """测试 ModeAdapterNode 姿态命令接收与分发"""
        from puppypi_adapter.mode_adapter_node import ModeAdapterNode
        from std_msgs.msg import String, Bool
        node = ModeAdapterNode()
        # 发送 STAND 姿态
        msg = String()
        msg.data = 'STAND'
        node._on_posture_cmd(msg)
        self.assertEqual(node.current_posture, 'STAND')

        # 发送 FREEZE 姿态
        msg.data = 'FREEZE'
        node._on_posture_cmd(msg)
        self.assertEqual(node.current_posture, 'FREEZE')
        if node.hw:
            self.assertTrue(node.hw.emergency_stopped)

        # 发送 RECOVER 姿态
        msg.data = 'RECOVER'
        node._on_posture_cmd(msg)
        self.assertEqual(node.current_posture, 'RECOVER')
        if node.hw:
            self.assertFalse(node.hw.emergency_stopped)

        # 使能电机
        enable_msg = Bool()
        enable_msg.data = True
        node._on_enable(enable_msg)
        self.assertTrue(node.motion_enabled)

        if node.hw:
            node.hw.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)

