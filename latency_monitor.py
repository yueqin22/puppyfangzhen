"""导航模块延迟监控 (P1-3)

依据 guihua20260809.md P1 阶段要求:
  - 记录各模块每帧的计算延迟
  - 计算 P50/P95/P99 百分位
  - 检测单帧超时 (>50ms 视为超时, 20Hz 要求)
  - 支持导出 JSON 格式供实验分析使用

用法:
    monitor = LatencyMonitor()
    with monitor.track("amcl"):
        amcl.update(...)
    with monitor.track("astar"):
        astar.find_path(...)
    stats = monitor.summary()
"""
import time
import math
import json
import os
from collections import defaultdict
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class ModuleStats:
    """单个模块的延迟统计。"""
    name: str
    times_ms: List[float] = field(default_factory=list)
    timeout_count: int = 0
    timeout_threshold_ms: float = 50.0

    def record(self, dt_ms: float):
        self.times_ms.append(dt_ms)
        if dt_ms > self.timeout_threshold_ms:
            self.timeout_count += 1

    def percentile(self, p: float) -> float:
        if not self.times_ms:
            return 0.0
        sorted_t = sorted(self.times_ms)
        idx = int(len(sorted_t) * p / 100)
        return sorted_t[min(idx, len(sorted_t) - 1)]

    def summary(self) -> dict:
        n = len(self.times_ms)
        if n == 0:
            return {"name": self.name, "n": 0, "p50_ms": 0, "p95_ms": 0,
                    "p99_ms": 0, "max_ms": 0, "avg_ms": 0, "timeouts": 0}
        return {
            "name": self.name,
            "n": n,
            "avg_ms": sum(self.times_ms) / n,
            "p50_ms": self.percentile(50),
            "p95_ms": self.percentile(95),
            "p99_ms": self.percentile(99),
            "max_ms": max(self.times_ms),
            "timeouts": self.timeout_count,
        }


class LatencyMonitor:
    """导航循环延迟监控器。

    P1-3: 为导航循环中的每个模块提供 P50/P95/P99 延迟统计，
    并检测单帧超时。

    用法:
        monitor = LatencyMonitor()
        with monitor.track("amcl"):
            amcl.update(...)
        stats = monitor.summary()  # 获取所有模块的延迟统计
    """

    def __init__(self, timeout_threshold_ms: float = 50.0):
        self.timeout_threshold_ms = timeout_threshold_ms
        self._modules: Dict[str, ModuleStats] = {}
        self._frame_times: List[float] = []
        self._frame_start: Optional[float] = None

    @contextmanager
    def track(self, module_name: str):
        """上下文管理器: 记录单个模块的执行时间。

        with monitor.track("amcl"):
            amcl.update(...)
        """
        if module_name not in self._modules:
            self._modules[module_name] = ModuleStats(
                name=module_name,
                timeout_threshold_ms=self.timeout_threshold_ms,
            )
        t0 = time.perf_counter()
        yield
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self._modules[module_name].record(dt_ms)

    def begin_frame(self):
        """标记帧开始。"""
        self._frame_start = time.perf_counter()

    def end_frame(self):
        """标记帧结束，记录帧总耗时。"""
        if self._frame_start is not None:
            dt_ms = (time.perf_counter() - self._frame_start) * 1000.0
            self._frame_times.append(dt_ms)
            self._frame_start = None

    def summary(self) -> dict:
        """返回所有模块和帧的延迟统计。"""
        module_stats = {name: m.summary()
                        for name, m in self._modules.items()}
        frame_stats = self._frame_summary()
        return {
            "modules": module_stats,
            "frame": frame_stats,
            "timeout_threshold_ms": self.timeout_threshold_ms,
        }

    def _frame_summary(self) -> dict:
        if not self._frame_times:
            return {"n": 0, "avg_ms": 0, "p50_ms": 0,
                    "p95_ms": 0, "p99_ms": 0, "max_ms": 0,
                    "timeout_count": 0}
        sorted_ft = sorted(self._frame_times)
        n = len(sorted_ft)
        timeout_count = sum(1 for t in self._frame_times
                            if t > self.timeout_threshold_ms)

        def _pct(p):
            return sorted_ft[min(int(n * p / 100), n - 1)]

        return {
            "n": n,
            "avg_ms": sum(self._frame_times) / n,
            "p50_ms": _pct(50),
            "p95_ms": _pct(95),
            "p99_ms": _pct(99),
            "max_ms": max(self._frame_times),
            "timeout_count": timeout_count,
        }

    def save_json(self, path: str):
        """保存延迟统计到 JSON 文件。"""
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.summary(), f, indent=2, default=str)

    def print_summary(self):
        """打印延迟统计摘要。"""
        s = self.summary()
        print("\n=== 延迟统计 ===")
        print(f"{'模块':<20} {'调用数':>6} {'P50(ms)':>10} {'P95(ms)':>10} "
              f"{'P99(ms)':>10} {'最大(ms)':>10} {'超时':>6}")
        print("-" * 80)
        for name, m in s["modules"].items():
            print(f"{name:<20} {m['n']:>6} {m['p50_ms']:>10.3f} "
                  f"{m['p95_ms']:>10.3f} {m['p99_ms']:>10.3f} "
                  f"{m['max_ms']:>10.3f} {m['timeouts']:>6}")
        f = s["frame"]
        print(f"\n{'帧总耗时':<20} {f['n']:>6} {f['p50_ms']:>10.3f} "
              f"{f['p95_ms']:>10.3f} {f['p99_ms']:>10.3f} "
              f"{f['max_ms']:>10.3f} {f['timeout_count']:>6}")
        print(f"超时阈值: {self.timeout_threshold_ms}ms")
