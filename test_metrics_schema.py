"""Unit tests for metrics_schema module.

依据 guihua20260809.md 第 7.1 节"单元测试"要求，
为统一指标校验模块编写测试。
"""
import math
import json
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics_schema import (
    is_invalid_metric_value,
    sanitize_metric,
    sanitize_metrics_dict,
    TrialResult,
    create_invalid_result,
    aggregate_valid_results,
    compute_stats,
    get_results_dir,
    make_experiment_id,
    save_trial_config,
    load_trial_config,
    mark_demo_results,
    is_demo_result,
    UNIFIED_METRICS_SPEC,
)


# =====================================================================
# 1. 值校验测试
# =====================================================================
class TestIsInvalidMetricValue:
    """测试 is_invalid_metric_value 函数。"""

    def test_none_is_invalid(self):
        assert is_invalid_metric_value(None) is True

    def test_nan_is_invalid(self):
        assert is_invalid_metric_value(float("nan")) is True

    def test_pos_inf_is_invalid(self):
        assert is_invalid_metric_value(float("inf")) is True

    def test_neg_inf_is_invalid(self):
        assert is_invalid_metric_value(float("-inf")) is True

    def test_neg_one_is_invalid(self):
        """-1 是历史脚本用作'解析失败'的占位符，应判为无效。"""
        assert is_invalid_metric_value(-1) is True

    def test_zero_is_valid(self):
        assert is_invalid_metric_value(0) is False

    def test_positive_int_is_valid(self):
        assert is_invalid_metric_value(42) is False

    def test_positive_float_is_valid(self):
        assert is_invalid_metric_value(3.14) is False

    def test_negative_float_is_valid(self):
        """除 -1 外的负数（如 -0.5）应视为有效（可能是合理指标值）。"""
        assert is_invalid_metric_value(-0.5) is False

    def test_bool_is_valid(self):
        """bool 是 int 子类，但应视为有效。"""
        assert is_invalid_metric_value(True) is False
        assert is_invalid_metric_value(False) is False


class TestSanitizeMetric:
    """测试 sanitize_metric 函数。"""

    def test_valid_value_returned(self):
        assert sanitize_metric(3.14) == 3.14

    def test_invalid_returns_default(self):
        assert sanitize_metric(None, default=0.0) == 0.0
        assert sanitize_metric(-1, default=0.0) == 0.0
        assert sanitize_metric(float("nan"), default=-99.0) == -99.0

    def test_invalid_returns_none_by_default(self):
        assert sanitize_metric(None) is None


class TestSanitizeMetricsDict:
    """测试 sanitize_metrics_dict 函数。"""

    def test_filters_invalid_and_returns_missing(self):
        metrics = {"a": 1.0, "b": -1, "c": None, "d": float("inf"), "e": 2.5}
        clean, missing = sanitize_metrics_dict(metrics)
        assert clean == {"a": 1.0, "e": 2.5}
        assert set(missing) == {"b", "c", "d"}

    def test_keep_invalid_as_none(self):
        metrics = {"a": 1.0, "b": -1}
        clean, missing = sanitize_metrics_dict(metrics, drop_invalid=False)
        assert clean == {"a": 1.0, "b": None}
        assert missing == ["b"]


# =====================================================================
# 2. TrialResult 数据结构测试
# =====================================================================
class TestTrialResult:
    """测试 TrialResult 数据类。"""

    def test_valid_result(self):
        r = TrialResult(
            experiment_id="test_001",
            algorithm="VO",
            scenario="dynamic_crossing",
            seed=1,
            sim_dt=0.0333,
            frames=1000,
            success=True,
            metrics={"collision_events": 3.0},
        )
        assert r.status == "valid"
        assert r.success is True
        assert r.mode == "official"

    def test_to_dict_removes_empty_fields(self):
        r = TrialResult(
            experiment_id="test_002",
            algorithm="APF",
            scenario="scene01",
            seed=2,
            sim_dt=0.0333,
            frames=500,
            success=True,
            metrics={"collision_events": 0.0},
        )
        d = r.to_dict()
        # failure_reason 为 None 应被移除
        assert "failure_reason" not in d
        # missing_fields 为空应被移除
        assert "missing_fields" not in d
        assert "artifacts" not in d

    def test_to_dict_keeps_failure_fields(self):
        r = create_invalid_result(
            "test_003", "VO", "scene01", 3,
            "metric_parser_failed", ["arrived", "frames"])
        d = r.to_dict()
        assert d["status"] == "invalid"
        assert d["failure_reason"] == "metric_parser_failed"
        assert d["missing_fields"] == ["arrived", "frames"]


class TestCreateInvalidResult:
    """测试 create_invalid_result 函数。"""

    def test_creates_invalid_result(self):
        r = create_invalid_result(
            experiment_id="exp_001",
            algorithm="STVOC",
            scenario="narrow_passage",
            seed=42,
            failure_reason="subprocess_timeout",
            missing_fields=["collision_events", "frames"],
        )
        assert r.status == "invalid"
        assert r.success is False
        assert r.failure_reason == "subprocess_timeout"
        assert r.missing_fields == ["collision_events", "frames"]
        assert r.metrics == {}


# =====================================================================
# 3. 结果聚合测试
# =====================================================================
class TestAggregateValidResults:
    """测试 aggregate_valid_results 函数。"""

    def test_filters_invalid_results(self):
        results = [
            TrialResult("id1", "VO", "s", 1, 0.033, 100, True,
                        metrics={"collision_events": 3.0}),
            create_invalid_result("id2", "VO", "s", 2,
                                  "parse_failed", ["collision_events"]),
            TrialResult("id3", "APF", "s", 1, 0.033, 100, True,
                        metrics={"collision_events": -1}),  # 无效值
            TrialResult("id4", "VO", "s", 3, 0.033, 100, True,
                        metrics={"collision_events": 1.0}),
        ]
        values, skipped = aggregate_valid_results(results, "collision_events")
        assert values == [3.0, 1.0]
        assert len(skipped) == 2

    def test_empty_list(self):
        values, skipped = aggregate_valid_results([], "collision_events")
        assert values == []
        assert skipped == []


class TestComputeStats:
    """测试 compute_stats 函数。"""

    def test_basic_stats(self):
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        stats = compute_stats(values)
        assert stats["n"] == 5
        assert stats["mean"] == 3.0
        assert stats["min"] == 1.0
        assert stats["max"] == 5.0
        assert stats["p50"] == 3.0

    def test_empty_list(self):
        stats = compute_stats([])
        assert stats["n"] == 0
        assert stats["mean"] == 0.0

    def test_single_value(self):
        stats = compute_stats([42.0])
        assert stats["n"] == 1
        assert stats["mean"] == 42.0
        assert stats["std"] == 0.0


# =====================================================================
# 4. demo/official 隔离测试
# =====================================================================
class TestDemoOfficialIsolation:
    """测试 demo / official 结果隔离。"""

    def test_get_results_dir_demo(self):
        path = get_results_dir("demo")
        assert "demo" in path

    def test_get_results_dir_official(self):
        path = get_results_dir("official")
        assert "official" in path

    def test_mark_demo_results_adds_warning(self):
        summary = {"some_metric": 1.0}
        marked = mark_demo_results(summary)
        assert marked["mode"] == "demo"
        assert "warning" in marked
        assert "DEMO ONLY" in marked["warning"]

    def test_is_demo_result(self):
        assert is_demo_result({"mode": "demo"}) is True
        assert is_demo_result({"mode": "official"}) is False
        assert is_demo_result({}) is False


# =====================================================================
# 5. 实验 ID 和配置序列化测试
# =====================================================================
class TestExperimentIdAndConfig:
    """测试实验 ID 生成和配置序列化。"""

    def test_make_experiment_id_format(self):
        eid = make_experiment_id("dynamic_crossing", "VO", 3, "20260809")
        assert eid == "20260809_dynamic_crossing_VO_seed003"

    def test_make_experiment_id_sanitizes_algo_name(self):
        eid = make_experiment_id("scene01", "CBF(VO+CBF)", 1, "20260809")
        assert "(" not in eid
        assert ")" not in eid

    def test_save_and_load_trial_config(self, tmp_path):
        config = {"algorithm": "VO", "scenario": "test"}
        env_vars = {"USE_VO": "1"}
        path = save_trial_config(
            str(tmp_path), "test_exp_001", config, env_vars, seed=42)
        assert os.path.exists(path)

        loaded = load_trial_config(path)
        assert loaded["experiment_id"] == "test_exp_001"
        assert loaded["seed"] == 42
        assert loaded["config"] == config
        assert loaded["env_vars"] == env_vars


# =====================================================================
# 6. 统一指标规范测试
# =====================================================================
class TestUnifiedMetricsSpec:
    """测试 UNIFIED_METRICS_SPEC 定义。"""

    def test_contains_required_metrics(self):
        """依据 guihua20260809.md 第 5 节，必须包含以下指标。"""
        keys = [m[0] for m in UNIFIED_METRICS_SPEC]
        required = [
            "collision_events", "collision_frames", "near_miss_events",
            "minimum_clearance", "path_length", "goal_success_rate",
            "timeout_rate", "coverage", "localization_rmse",
            "planning_time_p50_ms", "planning_time_p95_ms",
            "planning_time_p99_ms", "progress_per_second",
            "cpu_time_per_frame_ms",
        ]
        for req in required:
            assert req in keys, f"缺少必需指标: {req}"

    def test_each_metric_has_4_fields(self):
        for entry in UNIFIED_METRICS_SPEC:
            assert len(entry) == 4
            key, label, higher_better, desc = entry
            assert isinstance(key, str)
            assert isinstance(label, str)
            assert isinstance(higher_better, bool)
            assert isinstance(desc, str)
