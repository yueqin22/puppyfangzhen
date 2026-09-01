"""Unit test for Preflight Checker."""

import pytest
import sys
from pathlib import Path

try:
    from puppy_minicpm_robot.scripts.preflight_check import PreflightChecker
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    from preflight_check import PreflightChecker


class TestPreflightCheck:
    def test_run_all_checks(self):
        checker = PreflightChecker()
        res = checker.run_all_checks()

        assert "pass_count" in res
        assert "warn_count" in res
        assert "fail_count" in res
        assert len(checker.results) >= 5
