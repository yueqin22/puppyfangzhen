"""性能基准测试 (v9.0) - P0 修复版

依据 guihua20260809.md 第 3.3 节"性能采样缺陷"修复:
  - P0-5: 修复 CPU 监控（在工作负载运行期间采样，而非之后）
  - 新增: 进程 CPU 平均值和峰值
  - 新增: 线程 CPU 占用
  - 新增: 内存峰值与长期增长趋势
  - 新增: P50/P95/P99 延迟
  - 新增: 仿真时间与真实时间比例
  - 新增: 单帧超时次数

运行方式: python test_performance.py
"""
import sys
import os
import time
import math
import threading
import psutil
import numpy as np
from dataclasses import dataclass, field
from typing import List

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics_schema import is_invalid_metric_value


@dataclass
class ModuleTiming:
    """模块计时结果"""
    name: str
    call_count: int = 0
    total_time_ms: float = 0.0
    avg_time_ms: float = 0.0
    max_time_ms: float = 0.0
    times: list = field(default_factory=list)

    def update(self, dt_ms):
        self.call_count += 1
        self.total_time_ms += dt_ms
        self.times.append(dt_ms)
        if dt_ms > self.max_time_ms:
            self.max_time_ms = dt_ms
        self.avg_time_ms = self.total_time_ms / self.call_count

    def percentile(self, p):
        if not self.times:
            return 0.0
        sorted_times = sorted(self.times)
        idx = int(len(sorted_times) * p / 100)
        return sorted_times[min(idx, len(sorted_times) - 1)]


class ResourceMonitor:
    """后台资源监控器。

    依据 P0-5: 在工作负载运行期间持续采样 CPU/内存，
    而非在工作负载结束后采样（旧版导致 cpu_avg=0.0）。
    """

    def __init__(self, interval: float = 0.1):
        self.interval = interval
        self._stop_event = threading.Event()
        self._thread: threading.Thread = None
        self.cpu_samples: List[float] = []
        self.mem_samples: List[float] = []
        self.thread_count_samples: List[int] = []
        self._process = psutil.Process(os.getpid())

    def start(self):
        """启动后台监控线程。"""
        self._stop_event.clear()
        # 首次调用 cpu_percent 建立基线（否则首次返回0）
        self._process.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """停止监控线程并等待结束。"""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                cpu = self._process.cpu_percent(interval=self.interval)
                mem = self._process.memory_info().rss / 1024 / 1024  # MB
                n_threads = self._process.num_threads()
                self.cpu_samples.append(cpu)
                self.mem_samples.append(mem)
                self.thread_count_samples.append(n_threads)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                break

    def summary(self) -> dict:
        """返回资源使用汇总。"""
        if not self.cpu_samples:
            return {
                "cpu_avg": 0.0, "cpu_max": 0.0,
                "mem_avg_mb": 0.0, "mem_max_mb": 0.0,
                "mem_growth_mb": 0.0,
                "thread_count_avg": 0, "thread_count_max": 0,
                "samples": 0,
            }
        return {
            "cpu_avg": float(np.mean(self.cpu_samples)),
            "cpu_max": float(max(self.cpu_samples)),
            "mem_avg_mb": float(np.mean(self.mem_samples)),
            "mem_max_mb": float(max(self.mem_samples)),
            # 内存增长趋势: 最后5个样本均值 - 前5个样本均值
            "mem_growth_mb": float(
                np.mean(self.mem_samples[-5:]) - np.mean(self.mem_samples[:5])
            ) if len(self.mem_samples) >= 10 else 0.0,
            "thread_count_avg": int(np.mean(self.thread_count_samples)),
            "thread_count_max": int(max(self.thread_count_samples)),
            "samples": len(self.cpu_samples),
            "cpu_samples": self.cpu_samples,
            "mem_samples": self.mem_samples,
        }


def measure_module_performance(n_frames=100):
    """测量各模块的计算性能。

    Returns:
        (timings_dict, monitor, wall_time_s, frame_timeouts)
    """
    timings = {}
    frame_timeouts = 0  # 单帧超时计数 (>50ms 视为超时)
    frame_threshold_ms = 50.0  # 20Hz 要求单帧 <50ms

    def _get(name):
        if name not in timings:
            timings[name] = ModuleTiming(name=name)
        return timings[name]

    try:
        from occupancy_grid import OccupancyGrid
        from costmap import Costmap
        from astar_planner import AStarPlanner
        from dwa_planner import DWAPlanner
        from amcl import AMCL
        from odometry import Odometry
        from nav_core.exploration.frontier_manager import FrontierManager
        from nav_core.recovery.recovery_manager import RecoveryManager, RecoveryContext

        occ = OccupancyGrid()
        cm = Costmap()
        astar = AStarPlanner(cm)
        dwa = DWAPlanner(cm)
        amcl = AMCL(occ, n_particles=200, n_obs_rays=36)
        odom = Odometry()
        fm = FrontierManager(occ, cm)
        rm = RecoveryManager(cm)
    except ImportError as e:
        print(f"模块导入失败: {e}")
        return timings, None, 0.0, 0, 0.0

    angles = np.linspace(-math.pi, math.pi, 180)
    distances = np.random.uniform(1.0, 8.0, 180)

    # 启动后台资源监控 (P0-5 修复)
    monitor = ResourceMonitor(interval=0.1)
    monitor.start()

    sim_time_total = 0.0  # 累计仿真时间（按 SIM_DT=0.0333 估算）
    sim_dt = 0.0333
    wall_start = time.perf_counter()

    for frame in range(n_frames):
        frame_start = time.perf_counter()
        rx = frame * 0.01
        ry = math.sin(frame * 0.1) * 0.5
        ryaw = frame * 0.05

        # OccupancyGrid更新
        t0 = time.perf_counter()
        occ.update_from_scan(rx, ry, angles, distances, max_range=8.0)
        _get('occupancy_grid').update((time.perf_counter() - t0) * 1000)

        # Costmap更新
        t0 = time.perf_counter()
        cm.update_static(occ, frame=frame)
        _get('costmap').update((time.perf_counter() - t0) * 1000)

        # AMCL更新
        t0 = time.perf_counter()
        amcl.update(0.01, 0.0, 0.05, angles, distances, frame=frame)
        _get('amcl').update((time.perf_counter() - t0) * 1000)

        # A*路径规划（每10帧）
        if frame % 10 == 0:
            t0 = time.perf_counter()
            try:
                astar.find_path(rx, ry, 5.0, 3.0)
            except Exception:
                pass
            _get('astar').update((time.perf_counter() - t0) * 1000)

        # DWA速度计算
        t0 = time.perf_counter()
        try:
            dwa.compute_velocity(rx, ry, ryaw,
                                [(0, 0), (1, 0), (2, 0)], (5, 3))
        except Exception:
            pass
        _get('dwa').update((time.perf_counter() - t0) * 1000)

        # RecoveryManager决策（每5帧）
        if frame % 5 == 0:
            t0 = time.perf_counter()
            ctx = RecoveryContext(
                rx=rx, ry=ry, ryaw=ryaw,
                true_x=rx, true_y=ry,
                frame=frame, recover_frames=0,
            )
            rm.decide(ctx)
            _get('recovery_manager').update((time.perf_counter() - t0) * 1000)

        # 单帧超时检测
        frame_ms = (time.perf_counter() - frame_start) * 1000
        if frame_ms > frame_threshold_ms:
            frame_timeouts += 1
        sim_time_total += sim_dt

    wall_time = time.perf_counter() - wall_start
    monitor.stop()

    return timings, monitor, wall_time, frame_timeouts, sim_time_total


def main():
    print("=" * 60)
    print("性能基准测试 (v9.0) - P0 修复版")
    print("=" * 60)

    n_frames = 100
    timings, monitor, wall_time, frame_timeouts, sim_time_total = \
        measure_module_performance(n_frames)
    if monitor is None:
        return 1

    # 模块性能
    print("\n--- 模块计算延迟 ---")
    print(f"\n{'模块':<25} {'P50(ms)':<10} {'P95(ms)':<10} {'P99(ms)':<10} "
          f"{'最大(ms)':<10} {'调用次数':<10}")
    print("-" * 75)
    for name, t in sorted(timings.items(),
                          key=lambda x: x[1].avg_time_ms, reverse=True):
        p50 = t.percentile(50)
        p95 = t.percentile(95)
        p99 = t.percentile(99)
        print(f"{name:<25} {p50:<10.3f} {p95:<10.3f} {p99:<10.3f} "
              f"{t.max_time_ms:<10.3f} {t.call_count:<10}")

    total_per_frame = sum(t.avg_time_ms for t in timings.values())
    estimated_fps = 1000 / total_per_frame if total_per_frame > 0 else 0
    print(f"\n每帧总计算时间: {total_per_frame:.2f}ms")
    print(f"估算帧率: {estimated_fps:.1f} FPS")

    # 仿真时间 vs 真实时间比例
    sim_real_ratio = sim_time_total / wall_time if wall_time > 0 else 0
    print(f"\n--- 时间统计 ---")
    print(f"真实耗时: {wall_time:.2f}s")
    print(f"仿真时间: {sim_time_total:.2f}s (dt={0.0333})")
    print(f"仿真/真实 比例: {sim_real_ratio:.3f}x")
    print(f"单帧超时次数 (>50ms): {frame_timeouts}/{n_frames}")

    # 系统资源 (P0-5 修复: 在工作负载期间采样)
    print("\n--- 系统资源使用 (工作负载期间采样) ---")
    if monitor is not None:
        resources = monitor.summary()
        print(f"CPU平均: {resources['cpu_avg']:.1f}%")
        print(f"CPU峰值: {resources['cpu_max']:.1f}%")
        print(f"内存平均: {resources['mem_avg_mb']:.1f} MB")
        print(f"内存峰值: {resources['mem_max_mb']:.1f} MB")
        print(f"内存增长: {resources['mem_growth_mb']:+.1f} MB")
        print(f"线程数: avg={resources['thread_count_avg']}, "
              f"max={resources['thread_count_max']}")
        print(f"采样数: {resources['samples']}")
    else:
        resources = {
            "cpu_avg": 0.0, "cpu_max": 0.0,
            "mem_avg_mb": 0.0, "mem_max_mb": 0.0,
            "mem_growth_mb": 0.0,
            "thread_count_avg": 0, "thread_count_max": 0,
            "samples": 0,
        }
        print("[警告] 资源监控未启动")

    # P50/P95/P99 全局延迟
    all_frame_times = []
    for t in timings.values():
        all_frame_times.extend(t.times)
    if all_frame_times:
        sorted_ft = sorted(all_frame_times)
        n = len(sorted_ft)

        def _pct(p):
            return sorted_ft[min(int(n * p / 100), n - 1)]

        global_p50 = _pct(50)
        global_p95 = _pct(95)
        global_p99 = _pct(99)
    else:
        global_p50 = global_p95 = global_p99 = 0.0

    print(f"\n--- 全局延迟统计 ---")
    print(f"P50: {global_p50:.3f}ms")
    print(f"P95: {global_p95:.3f}ms")
    print(f"P99: {global_p99:.3f}ms")

    # 保存结果
    results = {
        'module_timings': {name: {
            'avg_ms': t.avg_time_ms,
            'p50_ms': t.percentile(50),
            'p95_ms': t.percentile(95),
            'p99_ms': t.percentile(99),
            'max_ms': t.max_time_ms,
            'calls': t.call_count,
        } for name, t in timings.items()},
        'total_per_frame_ms': total_per_frame,
        'estimated_fps': estimated_fps,
        'global_latency': {
            'p50_ms': global_p50,
            'p95_ms': global_p95,
            'p99_ms': global_p99,
        },
        'timing': {
            'wall_time_s': wall_time,
            'sim_time_s': sim_time_total,
            'sim_real_ratio': sim_real_ratio,
            'frame_timeout_count': frame_timeouts,
            'frame_timeout_threshold_ms': 50.0,
        },
        'resources': {
            'cpu_avg': resources['cpu_avg'],
            'cpu_max': resources['cpu_max'],
            'mem_avg_mb': resources['mem_avg_mb'],
            'mem_max_mb': resources['mem_max_mb'],
            'mem_growth_mb': resources['mem_growth_mb'],
            'thread_count_avg': resources['thread_count_avg'],
            'thread_count_max': resources['thread_count_max'],
            'samples': resources['samples'],
        },
    }

    # P0-2: 清洗无效值
    from metrics_schema import sanitize_metrics_dict
    clean, missing = sanitize_metrics_dict(results.get('resources', {}))
    if missing:
        print(f"\n[警告] 资源指标中有无效值被清洗: {missing}")

    json_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'performance_results.json')
    import json
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\n结果已保存: {json_path}")

    print(f"\n{'='*60}")
    print("性能基准测试完成")
    print("=" * 60)
    return 0


if __name__ == '__main__':
    sys.exit(main())
