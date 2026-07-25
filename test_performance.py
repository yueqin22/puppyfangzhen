"""性能基准测试 (v8.7)

测量导航系统的运行时性能指标：
  - 帧率(FPS)
  - 各模块计算延迟
  - 内存使用
  - CPU使用率

运行方式: python test_performance.py
"""
import sys
import os
import time
import math
import psutil
import numpy as np
from dataclasses import dataclass, field
from typing import List
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


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


def measure_module_performance(n_frames=100):
    """测量各模块的计算性能

    Args:
        n_frames: 模拟帧数

    Returns:
        dict: 模块名 -> ModuleTiming
    """
    timings = {}

    def _get(name):
        """获取或创建ModuleTiming（name为必填参数，defaultdict无法自动构造）"""
        if name not in timings:
            timings[name] = ModuleTiming(name=name)
        return timings[name]

    # 初始化模块
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
        return timings

    # 模拟数据
    angles = np.linspace(-math.pi, math.pi, 180)
    distances = np.random.uniform(1.0, 8.0, 180)

    # 模拟运行
    for frame in range(n_frames):
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

    return timings


def measure_system_resources(duration=10):
    """测量系统资源使用

    Args:
        duration: 测量持续时间(秒)

    Returns:
        dict: 资源使用数据
    """
    process = psutil.Process(os.getpid())

    # CPU使用率（采样）
    cpu_samples = []
    mem_samples = []

    for _ in range(int(duration / 0.5)):
        cpu_samples.append(process.cpu_percent(interval=0.5))
        mem_samples.append(process.memory_info().rss / 1024 / 1024)  # MB

    return {
        'cpu_avg': sum(cpu_samples) / len(cpu_samples) if cpu_samples else 0,
        'cpu_max': max(cpu_samples) if cpu_samples else 0,
        'mem_avg_mb': sum(mem_samples) / len(mem_samples) if mem_samples else 0,
        'mem_max_mb': max(mem_samples) if mem_samples else 0,
        'cpu_samples': cpu_samples,
        'mem_samples': mem_samples,
    }


def main():
    print("=" * 60)
    print("性能基准测试 (v8.7)")
    print("=" * 60)

    # 模块性能测试
    print("\n--- 模块计算延迟 ---")
    n_frames = 100
    timings = measure_module_performance(n_frames)

    print(f"\n{'模块':<25} {'平均(ms)':<12} {'最大(ms)':<12} {'P95(ms)':<12} {'调用次数':<10}")
    print("-" * 71)
    for name, t in sorted(timings.items(), key=lambda x: x[1].avg_time_ms, reverse=True):
        p95 = t.percentile(95)
        print(f"{name:<25} {t.avg_time_ms:<12.3f} {t.max_time_ms:<12.3f} "
              f"{p95:<12.3f} {t.call_count:<10}")

    # 帧率估算
    total_per_frame = sum(t.avg_time_ms for t in timings.values())
    estimated_fps = 1000 / total_per_frame if total_per_frame > 0 else 0
    print(f"\n每帧总计算时间: {total_per_frame:.2f}ms")
    print(f"估算帧率: {estimated_fps:.1f} FPS")

    # 系统资源
    print("\n--- 系统资源使用 (5秒采样) ---")
    resources = measure_system_resources(5)
    print(f"CPU平均: {resources['cpu_avg']:.1f}%")
    print(f"CPU峰值: {resources['cpu_max']:.1f}%")
    print(f"内存平均: {resources['mem_avg_mb']:.1f} MB")
    print(f"内存峰值: {resources['mem_max_mb']:.1f} MB")

    # 保存结果
    results = {
        'module_timings': {name: {
            'avg_ms': t.avg_time_ms,
            'max_ms': t.max_time_ms,
            'p95_ms': t.percentile(95),
            'calls': t.call_count,
        } for name, t in timings.items()},
        'total_per_frame_ms': total_per_frame,
        'estimated_fps': estimated_fps,
        'resources': {
            'cpu_avg': resources['cpu_avg'],
            'cpu_max': resources['cpu_max'],
            'mem_avg_mb': resources['mem_avg_mb'],
            'mem_max_mb': resources['mem_max_mb'],
        },
    }

    json_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'performance_results.json')
    import json
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n结果已保存: {json_path}")

    print(f"\n{'='*60}")
    print("性能基准测试完成")
    print("=" * 60)
    return 0


if __name__ == '__main__':
    sys.exit(main())
