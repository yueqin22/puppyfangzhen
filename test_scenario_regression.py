"""场景回归测试规范 (v8.4)

定义5类固定测试场景和统一指标表，确保不同版本之间的可比性。

场景列表:
  1. open_room: 空旷房间(无障碍物)
  2. narrow_doorway: 窄门场景(2个房间)
  3. furniture_maze: 家具迷宫
  4. dynamic_obstacles: 动态障碍物场景
  5. complex_indoor: 复杂室内(多房间+家具)

运行方式: python test_scenario_regression.py [scenario_name]
         不指定则运行所有场景
"""
import sys
import os
import json
import subprocess
import time
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional

SCENARIOS = {
    'open_room': {
        'description': '空旷房间(无障碍物)',
        'scene_file': 'scenes/open_room.ttt',
        'max_frames': 400,
        'target_coverage': 95.0,
        'env': {
            'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
            'METHOD_NAME': 'scenario_open_room',
            'MAX_FRAMES': '400',
        },
    },
    'narrow_doorway': {
        'description': '窄门场景(2个房间)',
        'scene_file': 'scenes/narrow_doorway.ttt',
        'max_frames': 600,
        'target_coverage': 90.0,
        'env': {
            'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
            'METHOD_NAME': 'scenario_narrow_doorway',
            'MAX_FRAMES': '600',
        },
    },
    'furniture_maze': {
        'description': '家具迷宫',
        'scene_file': 'scenes/furniture_maze.ttt',
        'max_frames': 800,
        'target_coverage': 85.0,
        'env': {
            'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
            'METHOD_NAME': 'scenario_furniture_maze',
            'MAX_FRAMES': '800',
        },
    },
    'dynamic_obstacles': {
        'description': '动态障碍物场景',
        'scene_file': 'scenes/dynamic_obstacles.ttt',
        'max_frames': 700,
        'target_coverage': 85.0,
        'env': {
            'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
            'USE_DYNAMIC_OBSTACLES': '1',
            'METHOD_NAME': 'scenario_dynamic',
            'MAX_FRAMES': '700',
        },
    },
    'complex_indoor': {
        'description': '复杂室内(多房间+家具)',
        'scene_file': 'scenes/complex_indoor.ttt',
        'max_frames': 1000,
        'target_coverage': 80.0,
        'env': {
            'USE_AMCL': '1', 'USE_TEB': '1', 'USE_EVAL': '1',
            'METHOD_NAME': 'scenario_complex',
            'MAX_FRAMES': '1000',
        },
    },
}


@dataclass
class ScenarioResult:
    """场景测试结果"""
    scenario: str
    description: str
    coverage: float = 0.0
    loc_error: float = 0.0
    max_loc_error: float = 0.0
    recover_count: int = 0
    path_length: float = 0.0
    time_to_90pct: float = 0.0
    passed: bool = False
    error: str = ""
    runtime_seconds: float = 0.0


def run_scenario(scenario_name: str, timeout: int = 300) -> ScenarioResult:
    """运行单个场景测试

    Args:
        scenario_name: 场景名称
        timeout: 超时时间(秒)

    Returns:
        ScenarioResult: 测试结果
    """
    scenario = SCENARIOS[scenario_name]
    result = ScenarioResult(
        scenario=scenario_name,
        description=scenario['description'],
    )

    print(f"\n{'='*60}")
    print(f"运行场景: {scenario_name} - {scenario['description']}")
    print(f"{'='*60}")

    # 设置环境变量
    env = os.environ.copy()
    env.update(scenario['env'])

    # 运行仿真
    start_time = time.time()
    try:
        proc = subprocess.run(
            [sys.executable, 'autonomous_nav.py'],
            env=env,
            timeout=timeout,
            capture_output=True,
            text=True,
            cwd=os.path.dirname(os.path.abspath(__file__)),
        )
        result.runtime_seconds = time.time() - start_time

        # 解析输出获取指标
        output = proc.stdout + proc.stderr
        result = parse_scenario_output(output, result)

        # 检查是否达到目标覆盖率
        if result.coverage >= scenario['target_coverage']:
            result.passed = True
            print(f"  PASS: 覆盖率={result.coverage:.1f}% >= {scenario['target_coverage']}%")
        else:
            result.passed = False
            print(f"  FAIL: 覆盖率={result.coverage:.1f}% < {scenario['target_coverage']}%")

    except subprocess.TimeoutExpired:
        result.error = f"超时({timeout}s)"
        result.passed = False
        print(f"  FAIL: 超时")
    except Exception as e:
        result.error = str(e)
        result.passed = False
        print(f"  FAIL: {e}")

    return result


def parse_scenario_output(output: str, result: ScenarioResult) -> ScenarioResult:
    """从仿真输出中解析指标"""
    for line in output.split('\n'):
        if 'Coverage' in line or 'coverage' in line:
            # 尝试提取覆盖率
            parts = line.split()
            for i, p in enumerate(parts):
                try:
                    val = float(p.replace('%', ''))
                    if 'coverage' in line.lower() and val > 50:
                        result.coverage = val
                except ValueError:
                    pass
        if 'loc_err' in line or 'loc_error' in line:
            parts = line.split('=')
            if len(parts) >= 2:
                try:
                    result.loc_error = float(parts[-1].split()[0])
                except ValueError:
                    pass
        if 'RECOVER' in line and 'count' in line.lower():
            # 计算RECOVER次数
            result.recover_count = output.count('[RECOVER]')
        if 'DONE' in line:
            # 最终状态
            pass

    return result


def generate_report(results: List[ScenarioResult]) -> str:
    """生成统一指标表

    Returns:
        str: Markdown格式的报告
    """
    report = ["# 场景回归测试报告 (v8.4)\n"]
    report.append(f"运行时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    report.append("## 指标汇总表\n")
    report.append("| 场景 | 描述 | 覆盖率(%) | 定位误差(m) | RECOVER次数 | 路径长度(m) | 运行时间(s) | 结果 |")
    report.append("|------|------|-----------|-------------|-------------|-------------|-------------|------|")

    for r in results:
        status = "PASS" if r.passed else "FAIL"
        report.append(
            f"| {r.scenario} | {r.description} | {r.coverage:.1f} | "
            f"{r.loc_error:.3f} | {r.recover_count} | "
            f"{r.path_length:.1f} | {r.runtime_seconds:.1f} | {status} |"
        )

    report.append("\n## 详细结果\n")
    for r in results:
        report.append(f"### {r.scenario}\n")
        report.append(f"- 覆盖率: {r.coverage:.1f}%")
        report.append(f"- 定位误差: {r.loc_error:.3f}m (最大: {r.max_loc_error:.3f}m)")
        report.append(f"- RECOVER次数: {r.recover_count}")
        report.append(f"- 运行时间: {r.runtime_seconds:.1f}s")
        if r.error:
            report.append(f"- 错误: {r.error}")
        report.append("")

    return '\n'.join(report)


def main():
    if len(sys.argv) > 1:
        scenario_name = sys.argv[1]
        if scenario_name not in SCENARIOS:
            print(f"未知场景: {scenario_name}")
            print(f"可用场景: {list(SCENARIOS.keys())}")
            return 1
        results = [run_scenario(scenario_name)]
    else:
        # 运行所有场景
        results = []
        for name in SCENARIOS:
            results.append(run_scenario(name))

    # 生成报告
    report = generate_report(results)
    report_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'scenario_regression_report.md')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report)
    print(f"\n报告已保存: {report_path}")

    # 保存JSON结果
    json_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'scenario_regression_results.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump([asdict(r) for r in results], f, indent=2, ensure_ascii=False)

    # 打印汇总
    print(f"\n{'='*60}")
    passed = sum(1 for r in results if r.passed)
    print(f"汇总: {passed}/{len(results)} 场景通过")
    print(f"{'='*60}")

    return 0 if all(r.passed for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
