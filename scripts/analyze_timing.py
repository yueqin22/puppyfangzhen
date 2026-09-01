#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析 sim_test --timing-csv 产出的每帧耗时 CSV，计算分位数与超阈值计数。
用法:
  python scripts/analyze_timing.py artifacts/cpp_36000_seed1_timing.csv [csv2 csv3 ...]

输出: 帧数, 均值, P50/P90/P95/P99/P99.9, 以及 >50ms / >100ms / >200ms 帧数与占比。
用于 jihua20260818 §25.5 P2 长稳回归: 验证单帧 P99 <= 50ms。
"""
import sys
import statistics


def analyze(path):
    ms = []
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline()
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            try:
                ms.append(float(parts[1]))
            except ValueError:
                continue
    if not ms:
        print(f"[SKIP] {path}: 无有效数据")
        return
    n = len(ms)
    ms_sorted = sorted(ms)
    mean = statistics.fmean(ms_sorted)

    def pct(p):
        # 线性插值分位数 (numpy 'linear' 等效)
        if n == 1:
            return ms_sorted[0]
        k = (n - 1) * (p / 100.0)
        lo = int(k)
        hi = min(lo + 1, n - 1)
        frac = k - lo
        return ms_sorted[lo] + (ms_sorted[hi] - ms_sorted[lo]) * frac

    p50 = pct(50)
    p90 = pct(90)
    p95 = pct(95)
    p99 = pct(99)
    p999 = pct(99.9)

    def count_gt(thr):
        c = sum(1 for x in ms_sorted if x > thr)
        return c, 100.0 * c / n

    c50, r50 = count_gt(50.0)
    c100, r100 = count_gt(100.0)
    c200, r200 = count_gt(200.0)

    print(f"=== {path} ===")
    print(f"  帧数 n          = {n}")
    print(f"  均值 mean       = {mean:.3f} ms")
    print(f"  P50             = {p50:.3f} ms")
    print(f"  P90             = {p90:.3f} ms")
    print(f"  P95             = {p95:.3f} ms")
    print(f"  P99             = {p99:.3f} ms   (P2 目标 <= 50ms)")
    print(f"  P99.9           = {p999:.3f} ms")
    print(f"  >50ms  : {c50:6d} 帧 ({r50:.4f}%)")
    print(f"  >100ms : {c100:6d} 帧 ({r100:.4f}%)")
    print(f"  >200ms : {c200:6d} 帧 ({r200:.4f}%)")
    print()


def main():
    if len(sys.argv) < 2:
        print("用法: python analyze_timing.py <timing_csv> [csv2 ...]")
        sys.exit(1)
    for p in sys.argv[1:]:
        analyze(p)


if __name__ == "__main__":
    main()
