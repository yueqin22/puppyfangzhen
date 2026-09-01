"""Unit test for Phase E Benchmark Suite."""

import pytest
import os
import json
import sys
from pathlib import Path

try:
    from puppy_minicpm_robot.scripts.benchmark_suite import BenchmarkRunner
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    from benchmark_suite import BenchmarkRunner


class TestBenchmarkSuite:
    def test_benchmark_suite_execution(self, tmp_path):
        out_json = tmp_path / "benchmark_test.json"
        runner = BenchmarkRunner(num_trials_per_scenario=2)
        res = runner.run_full_suite(output_json=str(out_json))

        assert res["benchmark_status"] == "PASSED"
        assert out_json.exists()

        with open(out_json, "r", encoding="utf-8") as f:
            data = json.load(f)

        assert "scenarios" in data
        assert "open_area" in data["scenarios"]
        assert "narrow_corridor" in data["scenarios"]
        assert "dynamic_obstacle" in data["scenarios"]
        assert "target_occlusion" in data["scenarios"]
