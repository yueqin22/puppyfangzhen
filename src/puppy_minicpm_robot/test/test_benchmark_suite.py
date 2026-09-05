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

    def test_suite_is_not_vacuously_gated(self):
        """The suite must run with the adapter allowed to move at all.

        Rationale: with mission_phase unset the adapter's phase gating zeroes every
        command. Scenarios 1-3 then fail for a reason unrelated to what they
        measure, and scenario 4 -- which asserts vx == wz == 0 -- PASSES however
        broken the occlusion handling is. That state hid for months and meant the
        suite carried zero regression signal. If someone drops the phase
        assignment, fail here with a message that names the cause instead of
        letting four scenarios go quietly meaningless again.
        """
        runner = BenchmarkRunner(num_trials_per_scenario=1)
        assert runner.adapter.gate_by_mission_phase is True
        assert runner.adapter.mission_phase in runner.adapter.tracking_phases, (
            "benchmark_adapter.mission_phase=%r is not a tracking phase %r -- "
            "every command is gated to zero, so scenarios 1-3 fail spuriously and "
            "scenario 4 passes vacuously"
            % (runner.adapter.mission_phase, list(runner.adapter.tracking_phases))
        )
