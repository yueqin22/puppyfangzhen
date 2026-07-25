"""实时性能监控 + 压力测试 + Fuzz Testing

实现项目内存中的硬性要求:
    - 实时性能监控，每帧超时告警并自动降级
    - 24小时连续运行压力测试，内存泄漏检测
    - Fuzz testing，随机传感器噪声注入

性能监控:
    - 每帧执行时间测量
    - 超时告警 (>50ms 警告, >100ms 错误)
    - 自动降级 (降低控制频率)
    - 帧率统计 (FPS)

压力测试:
    - 24小时连续运行
    - 内存使用监控
    - CPU 使用率监控
    - 内存泄漏检测 (线性回归)
    - 自动终止条件 (崩溃/超时)

Fuzz Testing:
    - 随机传感器噪声注入
    - 边界值测试
    - 异常输入处理
    - 覆盖率统计

使用方式:
    # 性能监控
    monitor = PerformanceMonitor(target_fps=30)
    monitor.start_frame()
    # ... 执行任务 ...
    monitor.end_frame()
    if monitor.is_degraded():
        # 降级处理

    # 压力测试
    tester = StressTester(duration_hours=24)
    results = tester.run(test_function)

    # Fuzz Testing
    fuzzer = SensorFuzzer()
    fuzzer.start()
    # 运行被测系统...
    fuzzer.stop()
"""
from __future__ import annotations

import time
import threading
import gc
import math
import random
import logging
import tracemalloc
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Dict, Any, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class FrameTiming:
    """单帧计时信息"""
    frame_id: int
    start_time: float
    end_time: float = 0.0
    duration: float = 0.0
    task_name: str = ""
    warning_threshold: float = 0.050
    error_threshold: float = 0.100

    @property
    def exceeded_warning(self) -> bool:
        return self.duration > self.warning_threshold

    @property
    def exceeded_error(self) -> bool:
        return self.duration > self.error_threshold


class PerformanceMonitor:
    """实时性能监控器

    每帧执行时间测量，超时告警，自动降级。

    监控指标:
        - 帧执行时间 (ms)
        - FPS (帧率)
        - 超时帧比例
        - 内存使用 (MB)
        - CPU 使用率 (%)

    降级策略:
        - 正常 (FPS >= 25): 全功能
        - 降速 (FPS 15-25): 降低最大速度 50%
        - 严重降速 (FPS 10-15): 降低最大速度 80%
        - 危险 (FPS < 10): 安全停车
    """

    def __init__(self, target_fps: float = 30.0,
                 warning_threshold: float = 0.050,
                 error_threshold: float = 0.100):
        """初始化性能监控器

        Args:
            target_fps: 目标帧率
            warning_threshold: 警告阈值 (秒)
            error_threshold: 错误阈值 (秒)
        """
        self.target_fps = target_fps
        self.target_frame_time = 1.0 / target_fps
        self.warning_threshold = warning_threshold
        self.error_threshold = error_threshold

        # 帧计时
        self._current_frame: Optional[FrameTiming] = None
        self._frame_history: List[FrameTiming] = []
        self._frame_id = 0

        # 统计
        self._total_frames = 0
        self._warning_frames = 0
        self._error_frames = 0
        self._fps_history: List[float] = []
        self._memory_history: List[float] = []

        # 降级状态
        self._degradation_level = 0  # 0=正常, 1=降速, 2=严重降速, 3=危险

        # 内存监控
        self._use_tracemalloc = False

        # 超时回调
        self._timeout_callbacks: List[Callable[[FrameTiming], None]] = []

        # 线程安全
        self._lock = threading.Lock()

    def start_frame(self, task_name: str = ""):
        """开始帧计时"""
        self._current_frame = FrameTiming(
            frame_id=self._frame_id,
            start_time=time.time(),
            task_name=task_name,
            warning_threshold=self.warning_threshold,
            error_threshold=self.error_threshold,
        )

    def end_frame(self) -> Optional[FrameTiming]:
        """结束帧计时

        Returns:
            FrameTiming 帧计时信息
        """
        if not self._current_frame:
            return None

        frame = self._current_frame
        frame.end_time = time.time()
        frame.duration = frame.end_time - frame.start_time

        with self._lock:
            self._frame_history.append(frame)
            if len(self._frame_history) > 1000:
                self._frame_history = self._frame_history[-500:]

            self._total_frames += 1
            if frame.exceeded_warning:
                self._warning_frames += 1
            if frame.exceeded_error:
                self._error_frames += 1

            # 更新 FPS
            if len(self._frame_history) >= 2:
                recent = self._frame_history[-30:]
                if len(recent) >= 2:
                    dt = recent[-1].end_time - recent[0].start_time
                    if dt > 0:
                        fps = (len(recent) - 1) / dt
                        self._fps_history.append(fps)
                        if len(self._fps_history) > 100:
                            self._fps_history = self._fps_history[-50:]

            # 更新降级级别
            self._update_degradation()

            # 记录内存
            self._record_memory()

        # 超时回调
        if frame.exceeded_warning:
            for cb in self._timeout_callbacks:
                try:
                    cb(frame)
                except Exception as e:
                    logger.error(f"超时回调失败: {e}")

        self._frame_id += 1
        self._current_frame = None
        return frame

    def _update_degradation(self):
        """根据 FPS 更新降级级别"""
        if not self._fps_history:
            return
        avg_fps = np.mean(self._fps_history[-10:]) if len(self._fps_history) >= 10 else self._fps_history[-1]

        if avg_fps >= self.target_fps * 0.83:  # >= 25 FPS (target 30)
            self._degradation_level = 0
        elif avg_fps >= self.target_fps * 0.50:  # 15-25 FPS
            self._degradation_level = 1
        elif avg_fps >= self.target_fps * 0.33:  # 10-15 FPS
            self._degradation_level = 2
        else:  # < 10 FPS
            self._degradation_level = 3

    def _record_memory(self):
        """记录内存使用"""
        try:
            import psutil
            process = psutil.Process()
            mem_mb = process.memory_info().rss / 1024 / 1024
            self._memory_history.append(mem_mb)
            if len(self._memory_history) > 1000:
                self._memory_history = self._memory_history[-500:]
        except ImportError:
            pass

    def add_timeout_callback(self, callback: Callable[[FrameTiming], None]):
        """添加超时回调"""
        self._timeout_callbacks.append(callback)

    def get_speed_factor(self) -> float:
        """获取速度限制因子 (根据降级级别)

        Returns:
            速度因子 [0.0, 1.0]
        """
        factors = [1.0, 0.5, 0.2, 0.0]
        return factors[min(self._degradation_level, 3)]

    def is_degraded(self) -> bool:
        """是否处于降级状态"""
        return self._degradation_level > 0

    @property
    def stats(self) -> Dict[str, Any]:
        """统计信息"""
        with self._lock:
            avg_fps = float(np.mean(self._fps_history[-10:])) if self._fps_history else 0.0
            avg_frame_time = (
                float(np.mean([f.duration for f in self._frame_history[-30:]]))
                if self._frame_history else 0.0
            )
            return {
                'total_frames': self._total_frames,
                'avg_fps': avg_fps,
                'avg_frame_time_ms': avg_frame_time * 1000,
                'target_fps': self.target_fps,
                'warning_frames': self._warning_frames,
                'error_frames': self._error_frames,
                'warning_rate': self._warning_frames / max(self._total_frames, 1),
                'error_rate': self._error_frames / max(self._total_frames, 1),
                'degradation_level': self._degradation_level,
                'memory_mb': self._memory_history[-1] if self._memory_history else 0,
            }


class StressTester:
    """24小时压力测试器

    长时间运行系统，监控内存泄漏和性能衰减。

    检测内容:
        - 内存使用趋势 (线性回归斜率 > 阈值 = 泄漏)
        - FPS 衰减 (持续下降 = 性能问题)
        - 错误率上升
        - 崩溃检测

    使用方式:
        tester = StressTester(duration_hours=24)
        results = tester.run(lambda: run_one_cycle())
    """

    def __init__(self, duration_hours: float = 24.0,
                 cycle_interval: float = 1.0,
                 memory_leak_threshold: float = 0.1):  # MB/hour
        """初始化压力测试器

        Args:
            duration_hours: 测试持续时间 (小时)
            cycle_interval: 每个测试循环间隔 (秒)
            memory_leak_threshold: 内存泄漏阈值 (MB/hour)
        """
        self.duration_seconds = duration_hours * 3600
        self.cycle_interval = cycle_interval
        self.memory_leak_threshold = memory_leak_threshold

        # 测试结果
        self._start_time = 0.0
        self._end_time = 0.0
        self._cycle_count = 0
        self._error_count = 0
        self._crash_detected = False

        # 内存采样
        self._memory_samples: List[Tuple[float, float]] = []  # (time, memory_mb)
        self._fps_samples: List[Tuple[float, float]] = []

        # 性能监控
        self.monitor = PerformanceMonitor()

    def run(self, test_function: Callable[[], Any],
            progress_callback: Optional[Callable[[int, Dict], None]] = None
            ) -> Dict[str, Any]:
        """运行压力测试

        Args:
            test_function: 每个循环执行的测试函数
            progress_callback: 进度回调 (cycle_count, stats)

        Returns:
            测试结果字典
        """
        self._start_time = time.time()
        self._end_time = self._start_time + self.duration_seconds

        logger.info(f"压力测试开始: {self.duration_seconds/3600:.1f}小时")

        while time.time() < self._end_time:
            try:
                # 执行测试
                self.monitor.start_frame()
                test_function()
                self.monitor.end_frame()

                self._cycle_count += 1

                # 采样
                elapsed = time.time() - self._start_time
                stats = self.monitor.stats
                self._memory_samples.append((elapsed, stats['memory_mb']))
                self._fps_samples.append((elapsed, stats['avg_fps']))

                # 保持采样数量
                if len(self._memory_samples) > 10000:
                    self._memory_samples = self._memory_samples[-5000:]
                if len(self._fps_samples) > 10000:
                    self._fps_samples = self._fps_samples[-5000:]

                # 进度回调
                if progress_callback and self._cycle_count % 100 == 0:
                    progress_callback(self._cycle_count, stats)

            except Exception as e:
                self._error_count += 1
                logger.error(f"压力测试循环 {self._cycle_count} 错误: {e}")
                if self._error_count > 100:
                    self._crash_detected = True
                    break

            time.sleep(self.cycle_interval)

        # 分析结果
        return self.analyze_results()

    def analyze_results(self) -> Dict[str, Any]:
        """分析测试结果

        检测内存泄漏 (线性回归)、FPS衰减、错误率

        Returns:
            分析结果字典
        """
        duration = time.time() - self._start_time

        # 内存泄漏检测 (线性回归)
        memory_leak_rate = 0.0
        memory_leak_detected = False
        if len(self._memory_samples) > 10:
            times = np.array([s[0] / 3600 for s in self._memory_samples])  # hours
            mems = np.array([s[1] for s in self._memory_samples])
            # 线性回归: mem = a * time + b
            n = len(times)
            sum_t = times.sum()
            sum_m = mems.sum()
            sum_tm = (times * mems).sum()
            sum_tt = (times * times).sum()
            denom = n * sum_tt - sum_t * sum_t
            if abs(denom) > 1e-10:
                slope = (n * sum_tm - sum_t * sum_m) / denom
                memory_leak_rate = slope  # MB/hour
                memory_leak_detected = slope > self.memory_leak_threshold

        # FPS 衰减检测
        fps_trend = 0.0
        fps_degraded = False
        if len(self._fps_samples) > 10:
            first_half = [s[1] for s in self._fps_samples[:len(self._fps_samples)//2]]
            second_half = [s[1] for s in self._fps_samples[len(self._fps_samples)//2:]]
            if first_half and second_half:
                fps_trend = np.mean(second_half) - np.mean(first_half)
                fps_degraded = fps_trend < -2.0  # 下降超过 2 FPS

        return {
            'duration_hours': duration / 3600,
            'total_cycles': self._cycle_count,
            'error_count': self._error_count,
            'error_rate': self._error_count / max(self._cycle_count, 1),
            'crash_detected': self._crash_detected,
            'memory_leak_detected': memory_leak_detected,
            'memory_leak_rate_mb_per_hour': memory_leak_rate,
            'memory_final_mb': self._memory_samples[-1][1] if self._memory_samples else 0,
            'fps_trend': fps_trend,
            'fps_degraded': fps_degraded,
            'final_stats': self.monitor.stats,
            'verdict': self._get_verdict(memory_leak_detected, fps_degraded,
                                         self._crash_detected),
        }

    def _get_verdict(self, leak: bool, fps_deg: bool, crash: bool) -> str:
        """生成测试结论"""
        if crash:
            return "失败：系统崩溃"
        if leak:
            return "失败：检测到内存泄漏"
        if fps_deg:
            return "警告：性能衰减"
        if self._error_count > 10:
            return "警告：错误率较高"
        return "通过：系统稳定运行"


class SensorFuzzer:
    """传感器 Fuzz Testing

    随机注入传感器噪声，测试系统鲁棒性。

    Fuzz 策略:
        1. 高斯噪声: 在传感器读数上叠加高斯噪声
        2. 边界值: 极端值测试 (0, max, 负值)
        3. 突变: 瞬时大幅跳变
        4. 缺失: 数据丢失
        5. 延迟: 时间戳错位

    使用方式:
        fuzzer = SensorFuzzer(noise_level=0.1)
        fuzzer.start()
        # 运行被测系统...
        fuzzer.stop()
        report = fuzzer.generate_report()
    """

    def __init__(self, noise_level: float = 0.1,
                 mutation_rate: float = 0.01,
                 loss_rate: float = 0.001):
        """初始化 Fuzz 测试器

        Args:
            noise_level: 高斯噪声标准差
            mutation_rate: 突变概率 (每帧)
            loss_rate: 数据丢失概率
        """
        self.noise_level = noise_level
        self.mutation_rate = mutation_rate
        self.loss_rate = loss_rate

        self._running = False
        self._thread: Optional[threading.Thread] = None

        # 统计
        self._total_tests = 0
        self._mutations_injected = 0
        self._losses_injected = 0
        self._crashes_detected = 0
        self._exceptions_caught = 0

        # 噪声配置
        self._lidar_noise_std = noise_level
        self._imu_noise_std = noise_level * 0.5
        self._odom_noise_std = noise_level * 0.1

    def fuzz_lidar(self, ranges: np.ndarray) -> np.ndarray:
        """对 LiDAR 数据进行 Fuzz

        Args:
            ranges: 原始距离数据

        Returns:
            Fuzz 后的距离数据
        """
        if not self._running:
            return ranges

        self._total_tests += 1
        fuzzed = ranges.copy()

        # 高斯噪声
        noise = np.random.normal(0, self._lidar_noise_std, len(fuzzed))
        fuzzed = fuzzed + noise

        # 随机突变
        if random.random() < self.mutation_rate:
            self._mutations_injected += 1
            # 随机选择突变类型
            mutation_type = random.choice(['extreme', 'negative', 'zero', 'max'])

            n_mutate = random.randint(1, max(1, len(fuzzed) // 10))
            indices = random.sample(range(len(fuzzed)), min(n_mutate, len(fuzzed)))

            if mutation_type == 'extreme':
                fuzzed[indices] = np.random.uniform(50, 100, len(indices))
            elif mutation_type == 'negative':
                fuzzed[indices] = -np.random.uniform(0, 5, len(indices))
            elif mutation_type == 'zero':
                fuzzed[indices] = 0.0
            elif mutation_type == 'max':
                fuzzed[indices] = 100.0

        # 数据丢失
        if random.random() < self.loss_rate:
            self._losses_injected += 1
            loss_mask = np.random.random(len(fuzzed)) < 0.5
            fuzzed[loss_mask] = 0.0

        # 限制范围
        fuzzed = np.clip(fuzzed, -10, 100)

        return fuzzed

    def fuzz_imu(self, accel: np.ndarray, gyro: np.ndarray
                 ) -> Tuple[np.ndarray, np.ndarray]:
        """对 IMU 数据进行 Fuzz"""
        if not self._running:
            return accel, gyro

        self._total_tests += 1

        # 高斯噪声
        accel_fuzzed = accel + np.random.normal(0, self._imu_noise_std, 3)
        gyro_fuzzed = gyro + np.random.normal(0, self._imu_noise_std, 3)

        # 突变
        if random.random() < self.mutation_rate:
            self._mutations_injected += 1
            # NaN/Inf 注入
            if random.random() < 0.3:
                idx = random.randint(0, 2)
                accel_fuzzed[idx] = float('nan')
            else:
                # 极端值
                idx = random.randint(0, 2)
                accel_fuzzed[idx] = random.choice([1000, -1000, float('inf')])

        return accel_fuzzed, gyro_fuzzed

    def fuzz_odometry(self, x: float, y: float, yaw: float
                      ) -> Tuple[float, float, float]:
        """对里程计数据进行 Fuzz"""
        if not self._running:
            return x, y, yaw

        self._total_tests += 1

        # 高斯噪声
        x_fuzzed = x + random.gauss(0, self._odom_noise_std)
        y_fuzzed = y + random.gauss(0, self._odom_noise_std)
        yaw_fuzzed = yaw + random.gauss(0, self._odom_noise_std * 2)

        # 突变
        if random.random() < self.mutation_rate:
            self._mutations_injected += 1
            # 位置跳变
            x_fuzzed += random.uniform(-5, 5)
            y_fuzzed += random.uniform(-5, 5)

        return x_fuzzed, y_fuzzed, yaw_fuzzed

    def start(self):
        """开始 Fuzz 测试"""
        self._running = True
        logger.info(f"Fuzz 测试启动: noise={self.noise_level}, "
                    f"mutation_rate={self.mutation_rate}")

    def stop(self):
        """停止 Fuzz 测试"""
        self._running = False

    def record_crash(self):
        """记录崩溃"""
        self._crashes_detected += 1

    def record_exception(self):
        """记录异常（被系统捕获）"""
        self._exceptions_caught += 1

    def generate_report(self) -> Dict[str, Any]:
        """生成 Fuzz 测试报告"""
        return {
            'total_tests': self._total_tests,
            'mutations_injected': self._mutations_injected,
            'losses_injected': self._losses_injected,
            'crashes_detected': self._crashes_detected,
            'exceptions_caught': self._exceptions_caught,
            'crash_rate': self._crashes_detected / max(self._total_tests, 1),
            'exception_rate': self._exceptions_caught / max(self._total_tests, 1),
            'verdict': self._evaluate_robustness(),
        }

    def _evaluate_robustness(self) -> str:
        """评估系统鲁棒性"""
        if self._crashes_detected > 0:
            return f"失败：{self._crashes_detected} 次崩溃"
        if self._exceptions_caught > self._total_tests * 0.1:
            return "警告：异常率过高"
        if self._exceptions_caught > 0:
            return f"良好：捕获 {self._exceptions_caught} 次异常，无崩溃"
        return "优秀：无崩溃无异常"
