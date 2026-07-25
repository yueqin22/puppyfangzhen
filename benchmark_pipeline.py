"""端到端基准实验流水线 (v8.6)

自动化运行对比实验并生成报告。
支持多种方法配置的自动对比，生成统计表格和图表。

使用方法:
  python benchmark_pipeline.py --methods dwa amcl teb amcl_teb
  python benchmark_pipeline.py --all  # 运行所有配置

输出:
  - benchmark_results/comparison_table.csv
  - benchmark_results/comparison_charts.png
  - benchmark_results/benchmark_report.md
"""
import sys
import os
import json
import time
import argparse
import subprocess
import csv
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional

METHOD_CONFIGS = {
    'dwa': {
        'USE_AMCL': '0', 'USE_TEB': '0', 'USE_EVAL': '1',
        'METHOD_NAME': 'baseline_dwa',
        'MAX_FRAMES': '600',
    },
    'amcl': {
        'USE_AMCL': '1', 'USE_TEB': '0', 'USE_EVAL': '1',
        'METHOD_NAME': 'amcl_only',
        'MAX_FRAMES': '600',
    },
    'teb': {
        'USE_AMCL': '0', 'USE_TEB': '1', 'USE_EVAL': '1',
        'METHOD_NAME': 'teb_only',
        'MAX_FRAMES': '600',
    },
    'amcl_teb': {
        'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
        'METHOD_NAME': 'proposed_amcl_teb',
        'MAX_FRAMES': '600',
    },
    'amcl_teb_kld': {
        'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
        'USE_KLD_SAMPLING': '1',
        'METHOD_NAME': 'proposed_full',
        'MAX_FRAMES': '600',
    },
}


@dataclass
class BenchmarkResult:
    """基准实验结果"""
    method: str
    coverage: float = 0.0
    avg_loc_err: float = 0.0
    max_loc_err: float = 0.0
    recover_count: int = 0
    path_length: float = 0.0
    time_to_80pct: float = 0.0
    time_to_90pct: float = 0.0
    map_iou: float = 0.0
    hausdorff: float = 0.0
    map_entropy: float = 0.0
    runtime_seconds: float = 0.0
    success: bool = False
    error: str = ""


def run_benchmark(method: str, config: dict, timeout: int = 300) -> BenchmarkResult:
    """运行单个基准实验"""
    result = BenchmarkResult(method=method)
    print(f"\n运行方法: {method}")

    env = os.environ.copy()
    env.update(config)

    start_time = time.time()
    try:
        proc = subprocess.run(
            [sys.executable, 'autonomous_nav.py'],
            env=env, timeout=timeout,
            capture_output=True, text=True,
            cwd=os.path.dirname(os.path.abspath(__file__)),
        )
        result.runtime_seconds = time.time() - start_time

        # 解析评估结果
        result = parse_benchmark_output(proc.stdout + proc.stderr, result)

        # 尝试读取评估JSON
        eval_dir = os.path.join(os.environ.get('TEMP', '/tmp'), 'eval')
        summary_path = os.path.join(eval_dir, f"{config['METHOD_NAME']}_summary.json")
        if os.path.exists(summary_path):
            with open(summary_path) as f:
                eval_data = json.load(f)
                result.coverage = eval_data.get('coverage', result.coverage)
                result.avg_loc_err = eval_data.get('avg_loc_err', result.avg_loc_err)
                result.max_loc_err = eval_data.get('max_loc_err', result.max_loc_err)
                result.recover_count = eval_data.get('recover_count', result.recover_count)
                result.path_length = eval_data.get('path_length', result.path_length)
                result.map_iou = eval_data.get('map_iou', result.map_iou)
                result.hausdorff = eval_data.get('hausdorff', result.hausdorff)
                result.map_entropy = eval_data.get('map_entropy', result.map_entropy)
                result.time_to_80pct = eval_data.get('time_to_80pct', result.time_to_80pct)
                result.time_to_90pct = eval_data.get('time_to_90pct', result.time_to_90pct)

        result.success = result.coverage > 50.0
        status = "PASS" if result.success else "FAIL"
        print(f"  [{status}] 覆盖率={result.coverage:.1f}% loc_err={result.avg_loc_err:.3f}m")

    except subprocess.TimeoutExpired:
        result.error = f"超时({timeout}s)"
        print(f"  [FAIL] 超时")
    except Exception as e:
        result.error = str(e)
        print(f"  [FAIL] {e}")

    return result


def parse_benchmark_output(output: str, result: BenchmarkResult) -> BenchmarkResult:
    """从输出中解析指标"""
    for line in output.split('\n'):
        if 'Coverage' in line and '%' in line:
            try:
                parts = line.split()
                for p in parts:
                    val = p.replace('%', '')
                    try:
                        v = float(val)
                        if v > 50:
                            result.coverage = max(result.coverage, v)
                    except ValueError:
                        pass
            except Exception:
                pass
        if 'RECOVER' in line and '[RECOVER]' in line:
            result.recover_count += 1
    return result


def generate_comparison_table(results: List[BenchmarkResult]) -> str:
    """生成CSV对比表"""
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'benchmark_results')
    os.makedirs(output_dir, exist_ok=True)

    csv_path = os.path.join(output_dir, 'comparison_table.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            '方法', '覆盖率(%)', '平均定位误差(m)', '最大定位误差(m)',
            'RECOVER次数', '路径长度(m)', '达80%时间(s)', '达90%时间(s)',
            '地图IoU', 'Hausdorff(m)', '地图熵', '运行时间(s)', '成功'
        ])
        for r in results:
            writer.writerow([
                r.method, f'{r.coverage:.1f}', f'{r.avg_loc_err:.3f}',
                f'{r.max_loc_err:.3f}', r.recover_count,
                f'{r.path_length:.1f}', f'{r.time_to_80pct:.1f}',
                f'{r.time_to_90pct:.1f}', f'{r.map_iou:.3f}',
                f'{r.hausdorff:.2f}', f'{r.map_entropy:.3f}',
                f'{r.runtime_seconds:.1f}', '是' if r.success else '否'
            ])

    return csv_path


def generate_report(results: List[BenchmarkResult], csv_path: str) -> str:
    """生成Markdown报告"""
    report = ["# 端到端基准实验报告 (v8.6)\n"]
    report.append(f"运行时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    report.append(f"方法数: {len(results)}\n")
    report.append("## 结果汇总\n")

    # 找到最佳方法
    best_coverage = max(results, key=lambda r: r.coverage) if results else None
    best_loc = min((r for r in results if r.success), key=lambda r: r.avg_loc_err, default=None)

    if best_coverage:
        report.append(f"- 最高覆盖率: **{best_coverage.method}** ({best_coverage.coverage:.1f}%)")
    if best_loc:
        report.append(f"- 最低定位误差: **{best_loc.method}** ({best_loc.avg_loc_err:.3f}m)")

    report.append(f"\n## 对比表\n")
    report.append(f"CSV文件: `{csv_path}`\n")

    report.append("\n## 详细结果\n")
    for r in results:
        status = "成功" if r.success else "失败"
        report.append(f"### {r.method} ({status})\n")
        report.append(f"- 覆盖率: {r.coverage:.1f}%")
        report.append(f"- 平均定位误差: {r.avg_loc_err:.3f}m")
        report.append(f"- 最大定位误差: {r.max_loc_err:.3f}m")
        report.append(f"- RECOVER次数: {r.recover_count}")
        report.append(f"- 地图IoU: {r.map_iou:.3f}")
        report.append(f"- Hausdorff距离: {r.hausdorff:.2f}m")
        if r.error:
            report.append(f"- 错误: {r.error}")
        report.append("")

    report_path = os.path.join(os.path.dirname(csv_path), 'benchmark_report.md')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(report))

    return report_path


def main():
    parser = argparse.ArgumentParser(description='端到端基准实验流水线')
    parser.add_argument('--methods', nargs='+', default=['dwa', 'amcl_teb'],
                       help='要测试的方法列表')
    parser.add_argument('--all', action='store_true', help='测试所有方法')
    parser.add_argument('--timeout', type=int, default=300, help='超时时间(秒)')
    args = parser.parse_args()

    if args.all:
        methods = list(METHOD_CONFIGS.keys())
    else:
        methods = args.methods

    print("=" * 60)
    print("端到端基准实验流水线 (v8.6)")
    print(f"方法: {methods}")
    print("=" * 60)

    results = []
    for method in methods:
        if method not in METHOD_CONFIGS:
            print(f"未知方法: {method}")
            continue
        result = run_benchmark(method, METHOD_CONFIGS[method], args.timeout)
        results.append(result)

    if not results:
        print("无有效结果")
        return 1

    # 生成报告
    csv_path = generate_comparison_table(results)
    report_path = generate_report(results, csv_path)

    print(f"\n{'='*60}")
    print(f"CSV对比表: {csv_path}")
    print(f"详细报告: {report_path}")
    print(f"{'='*60}")

    # 保存JSON
    json_path = os.path.join(os.path.dirname(csv_path), 'benchmark_results.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump([asdict(r) for r in results], f, indent=2, ensure_ascii=False)

    return 0


if __name__ == '__main__':
    sys.exit(main())
