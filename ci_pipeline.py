"""CI 流水线 (P2-3)

依据 guihua20260809.md P2 阶段要求:
  - 构建 C++ 测试
  - 运行 Python 单元测试
  - 运行回归测试
  - 验证指标 schema

用法:
    python ci_pipeline.py              # 全部阶段
    python ci_pipeline.py --skip-cpp   # 跳过C++构建
    python ci_pipeline.py --python-only  # 仅Python测试
"""
import sys
import os
import subprocess
import time
import argparse
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.resolve()


class CIPipeline:
    """CI 流水线执行器。"""

    def __init__(self, skip_cpp=False, skip_python=False):
        self.skip_cpp = skip_cpp
        self.skip_python = skip_python
        self.results = []
        self.start_time = time.time()

    def run_stage(self, name, command, cwd=None, timeout=300):
        """执行一个 CI 阶段。"""
        print(f"\n{'='*60}")
        print(f"  [CI] {name}")
        print(f"{'='*60}")
        t0 = time.time()
        try:
            result = subprocess.run(
                command, shell=True, cwd=cwd or PROJECT_ROOT,
                capture_output=True, text=True, timeout=timeout,
            )
            elapsed = time.time() - t0
            passed = result.returncode == 0
            self.results.append({
                "stage": name,
                "passed": passed,
                "elapsed_s": elapsed,
                "returncode": result.returncode,
            })
            # 输出最后20行
            lines = result.stdout.strip().split('\n')[-20:]
            for line in lines:
                print(f"  {line}")
            if not passed:
                err_lines = result.stderr.strip().split('\n')[-10:]
                for line in err_lines:
                    print(f"  [stderr] {line}")
            status = "PASS" if passed else "FAIL"
            print(f"  [{status}] {elapsed:.1f}s")
            return passed
        except subprocess.TimeoutExpired:
            elapsed = time.time() - t0
            self.results.append({
                "stage": name, "passed": False,
                "elapsed_s": elapsed, "returncode": -1,
            })
            print(f"  [TIMEOUT] {elapsed:.1f}s")
            return False

    def run_python_tests(self):
        """运行 Python 单元测试。"""
        if self.skip_python:
            return True

        test_files = [
            "test_metrics_schema.py",
            "test_regression_scenarios.py",
            "nav_core/test_frontier_manager.py",
            "nav_core/test_runtime_state.py",
            "nav_core/test_state_machine.py",
        ]
        all_passed = True
        for tf in test_files:
            path = PROJECT_ROOT / tf
            if not path.exists():
                print(f"  [SKIP] {tf} (文件不存在)")
                continue
            passed = self.run_stage(
                f"pytest: {tf}",
                f'python -m pytest "{tf}" -v --tb=short -q',
            )
            if not passed:
                all_passed = False
        return all_passed

    def run_cpp_tests(self):
        """运行 C++ ctest。"""
        if self.skip_cpp:
            return True

        bat_path = PROJECT_ROOT / "run_cpp_tests.bat"
        if not bat_path.exists():
            print("  [SKIP] run_cpp_tests.bat 不存在")
            return True

        return self.run_stage(
            "C++ ctest",
            str(bat_path),
            timeout=120,
        )

    def run_config_validation(self):
        """验证统一配置。"""
        return self.run_stage(
            "配置验证",
            "python unified_config.py",
        )

    def run_schema_validation(self):
        """验证指标 schema 自检。"""
        return self.run_stage(
            "Schema 自检",
            "python metrics_schema.py",
        )

    def run_all(self):
        """运行全部 CI 阶段。"""
        print("=" * 60)
        print("  CI Pipeline (P2-3)")
        print(f"  项目: {PROJECT_ROOT}")
        print(f"  跳过C++: {self.skip_cpp}, 跳过Python: {self.skip_python}")
        print("=" * 60)

        results = []
        results.append(("Schema自检", self.run_schema_validation()))
        results.append(("配置验证", self.run_config_validation()))
        results.append(("Python测试", self.run_python_tests()))
        results.append(("C++测试", self.run_cpp_tests()))

        # 汇总
        total_elapsed = time.time() - self.start_time
        print(f"\n{'='*60}")
        print(f"  CI 汇总 ({total_elapsed:.1f}s)")
        print(f"{'='*60}")
        all_passed = True
        for name, passed in results:
            status = "PASS" if passed else "FAIL"
            print(f"  [{status}] {name}")
            if not passed:
                all_passed = False

        n_pass = sum(1 for _, p in results if p)
        n_total = len(results)
        print(f"\n  {n_pass}/{n_total} 阶段通过")

        if all_passed:
            print("  [CI PASS] 全部通过")
        else:
            print("  [CI FAIL] 部分失败")

        return 0 if all_passed else 1


def main():
    parser = argparse.ArgumentParser(description="CI Pipeline (P2-3)")
    parser.add_argument("--skip-cpp", action="store_true",
                        help="跳过 C++ 构建/测试")
    parser.add_argument("--skip-python", action="store_true",
                        help="跳过 Python 测试")
    parser.add_argument("--python-only", action="store_true",
                        help="仅运行 Python 测试")
    args = parser.parse_args()

    skip_cpp = args.skip_cpp or args.python_only
    skip_python = args.skip_python

    pipeline = CIPipeline(skip_cpp=skip_cpp, skip_python=skip_python)
    return pipeline.run_all()


if __name__ == "__main__":
    sys.exit(main())
