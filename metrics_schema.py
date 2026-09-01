"""统一实验指标定义与校验模块 (v1.0)

依据 guihua20260809.md 第 5 节"统一指标定义"实现：
  - 定义所有实验必须记录的统一指标
  - 禁止 -1 / None / NaN / Infinity 进入统计
  - 为解析失败的实验提供 status / failure_reason / missing_fields 字段
  - 提供 demo / official 结果隔离标记

该模块被 compare_algorithms.py / test_performance.py /
ablation_study_v3.py 共享，保证指标定义一致。
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Tuple


# =====================================================================
# 1. 统一指标规范
# =====================================================================
# (metric_key, label, higher_better, description)
UNIFIED_METRICS_SPEC: List[Tuple[str, str, bool, str]] = [
    ("collision_events",       "碰撞事件数",   False, "连续碰撞只计一次"),
    ("collision_frames",       "碰撞帧数",     False, "处于碰撞状态的帧数"),
    ("near_miss_events",       "近距事件数",   False, "一次接近事件只计一次"),
    ("minimum_clearance",      "最小安全距离", True,  "全程最小安全距离(m)"),
    ("path_length",            "路径长度",     True,  "实际路径长度(m)"),
    ("goal_success_rate",      "到达率",       True,  "到达目标比例[0,1]"),
    ("timeout_rate",           "超时率",       False, "超时比例[0,1]"),
    ("coverage",               "覆盖率",       True,  "地图覆盖率[0,1]"),
    ("localization_rmse",      "定位RMSE",     False, "定位误差 RMSE(m)"),
    ("planning_time_p50_ms",   "规划P50",      False, "规划耗时 P50(ms)"),
    ("planning_time_p95_ms",   "规划P95",      False, "规划耗时 P95(ms)"),
    ("planning_time_p99_ms",   "规划P99",      False, "规划耗时 P99(ms)"),
    ("progress_per_second",    "进展速率",     True,  "单位时间有效进展(m/s)"),
    ("cpu_time_per_frame_ms",  "单帧CPU",      False, "单帧计算成本(ms)"),
]

# 历史兼容字段（旧脚本产出，允许读入但标注为 legacy）
LEGACY_METRIC_KEYS = {
    "dyn_collisions", "wall_hits", "arrived", "total_points",
    "frames", "near_miss", "total_frames", "final_coverage",
    "time_to_80", "mean_loc_err", "max_loc_err", "p95_loc_err",
    "recover_count", "collision_count", "wall_penetration_count",
    "follow_pct", "recover_pct", "done_pct", "avg_speed",
}


# =====================================================================
# 2. 值校验
# =====================================================================
def is_invalid_metric_value(value: Any) -> bool:
    """判断一个指标值是否为无效值。

    依据 guihua20260809.md 第 2.3 节，禁止以下值进入统计：
      - None
      - NaN
      - Infinity (+inf / -inf)
      - -1 （历史脚本用作"解析失败"占位符）
      - 负数（除允许负值的指标外）
    """
    if value is None:
        return True
    if isinstance(value, bool):
        # bool 是 int 子类，单独处理避免误判
        return False
    if isinstance(value, (int, float)):
        if math.isnan(value):
            return True
        if math.isinf(value):
            return True
        if value == -1:
            return True
        return False
    return False


def sanitize_metric(value: Any, default: Optional[float] = None) -> Optional[float]:
    """清洗单个指标值，无效则返回 default。"""
    if is_invalid_metric_value(value):
        return default
    return float(value)


def sanitize_metrics_dict(
    metrics: Dict[str, Any],
    drop_invalid: bool = True,
) -> Tuple[Dict[str, float], List[str]]:
    """清洗指标字典，返回 (有效指标, 缺失字段列表)。

    Args:
        metrics: 原始指标字典
        drop_invalid: True 则丢弃无效字段；False 则保留为 None

    Returns:
        (clean_metrics, missing_fields)
    """
    clean: Dict[str, float] = {}
    missing: List[str] = []
    for key, value in metrics.items():
        if is_invalid_metric_value(value):
            missing.append(key)
            if not drop_invalid:
                clean[key] = None
        else:
            clean[key] = float(value)
    return clean, missing


# =====================================================================
# 3. 实验结果数据结构
# =====================================================================
@dataclass
class TrialResult:
    """单次实验的标准化结果。

    依据 guihua20260809.md 第 5 节统一 JSON schema。
    """
    experiment_id: str
    algorithm: str
    scenario: str
    seed: int
    sim_dt: float
    frames: int
    success: bool
    status: str = "valid"  # valid | invalid | partial | timeout
    failure_reason: Optional[str] = None
    missing_fields: List[str] = field(default_factory=list)
    metrics: Dict[str, float] = field(default_factory=dict)
    artifacts: Dict[str, str] = field(default_factory=dict)
    config: Dict[str, Any] = field(default_factory=dict)
    mode: str = "official"  # official | demo
    timestamp: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        # 移除 None 值的 failure_reason
        if d.get("failure_reason") is None:
            d.pop("failure_reason", None)
        if not d.get("missing_fields"):
            d.pop("missing_fields", None)
        if not d.get("artifacts"):
            d.pop("artifacts", None)
        return d


def create_invalid_result(
    experiment_id: str,
    algorithm: str,
    scenario: str,
    seed: int,
    failure_reason: str,
    missing_fields: List[str],
    frames: int = 0,
    sim_dt: float = 0.0333,
    mode: str = "official",
) -> TrialResult:
    """创建一个标记为 invalid 的实验结果。

    用于日志解析失败、实验未正常完成等情况，
    替代历史脚本中返回 -1/None/Infinity 的做法。
    """
    return TrialResult(
        experiment_id=experiment_id,
        algorithm=algorithm,
        scenario=scenario,
        seed=seed,
        sim_dt=sim_dt,
        frames=frames,
        success=False,
        status="invalid",
        failure_reason=failure_reason,
        missing_fields=missing_fields,
        metrics={},
        mode=mode,
    )


# =====================================================================
# 4. 结果聚合（过滤无效数据）
# =====================================================================
def aggregate_valid_results(
    results: List[TrialResult],
    metric_key: str,
) -> Tuple[List[float], List[str]]:
    """从 TrialResult 列表中提取某个指标的所有有效值。

    自动跳过 status=invalid 的结果和该指标缺失/无效的结果。
    依据 guihua20260809.md P0-2 要求。
    """
    values: List[float] = []
    skipped_algos: List[str] = []
    for r in results:
        if r.status == "invalid":
            skipped_algos.append(f"{r.algorithm}(invalid)")
            continue
        val = r.metrics.get(metric_key)
        if is_invalid_metric_value(val):
            skipped_algos.append(f"{r.algorithm}(missing:{metric_key})")
            continue
        values.append(float(val))
    return values, skipped_algos


def compute_stats(values: List[float]) -> Dict[str, float]:
    """计算均值、标准差、P50/P95/P99（不依赖 scipy）。"""
    if not values:
        return {"n": 0, "mean": 0.0, "std": 0.0,
                "p50": 0.0, "p95": 0.0, "p99": 0.0,
                "min": 0.0, "max": 0.0}
    n = len(values)
    sorted_v = sorted(values)
    mean = sum(values) / n
    if n > 1:
        var = sum((x - mean) ** 2 for x in values) / (n - 1)
        std = math.sqrt(var)
    else:
        std = 0.0

    def _pct(p: float) -> float:
        if n == 1:
            return sorted_v[0]
        idx = int(n * p / 100)
        idx = min(idx, n - 1)
        return sorted_v[idx]

    return {
        "n": n,
        "mean": mean,
        "std": std,
        "p50": _pct(50),
        "p95": _pct(95),
        "p99": _pct(99),
        "min": sorted_v[0],
        "max": sorted_v[-1],
    }


# =====================================================================
# 5. demo / official 结果隔离
# =====================================================================
def get_results_dir(mode: str = "official") -> str:
    """返回对应模式的结果目录。

    依据 guihua20260809.md 第 2.1 节：
      - demo_results: 只用于展示流程
      - official_results: 用于论文、报告和算法结论
    """
    base = os.path.dirname(os.path.abspath(__file__))
    if mode == "demo":
        return os.path.join(base, "experiment_results", "demo")
    return os.path.join(base, "experiment_results", "official")


def mark_demo_results(summary: Dict[str, Any]) -> Dict[str, Any]:
    """给结果摘要打上 demo 标记，防止误用为正式结论。"""
    summary["mode"] = "demo"
    summary["warning"] = (
        "DEMO ONLY - 此结果仅用于展示分析流程，"
        "不可作为论文或算法结论。正式结论需使用 official 模式重跑。"
    )
    return summary


def is_demo_result(summary: Dict[str, Any]) -> bool:
    """判断结果是否为 demo 模式。"""
    return summary.get("mode") == "demo"


# =====================================================================
# 6. 统一实验 ID 生成
# =====================================================================
def make_experiment_id(
    scenario: str,
    algorithm: str,
    seed: int,
    date_str: str = "",
) -> str:
    """生成标准化实验 ID。

    格式: {date}_{scenario}_{algorithm}_seed{seed:03d}
    例如: 20260809_dynamic_crossing_vo_seed003
    """
    if not date_str:
        import datetime
        date_str = datetime.datetime.now().strftime("%Y%m%d")
    algo_safe = algorithm.replace(" ", "_").replace("(", "").replace(")", "")
    return f"{date_str}_{scenario}_{algo_safe}_seed{seed:03d}"


# =====================================================================
# 7. 配置序列化（保证可复现）
# =====================================================================
def save_trial_config(
    results_dir: str,
    experiment_id: str,
    config: Dict[str, Any],
    env_vars: Dict[str, str],
    seed: int,
) -> str:
    """保存单次实验的完整配置，保证可复现。

    依据 guihua20260809.md P0-3 要求。
    """
    os.makedirs(results_dir, exist_ok=True)
    config_path = os.path.join(results_dir, f"{experiment_id}_config.json")
    full_config = {
        "experiment_id": experiment_id,
        "seed": seed,
        "config": config,
        "env_vars": env_vars,
        "python_version": os.sys.version,
        "platform": os.sys.platform,
    }
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(full_config, f, indent=2, ensure_ascii=False, default=str)
    return config_path


def load_trial_config(config_path: str) -> Dict[str, Any]:
    """加载实验配置。"""
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


# =====================================================================
# 8. 统一实验数据 Schema (P2-2)
# =====================================================================
# 所有实验输出 JSON 必须遵循此 schema，保证 C++/Python 产出可互操作。
EXPERIMENT_OUTPUT_SCHEMA = {
    "experiment_id": "str  — 唯一实验ID",
    "timestamp": "str  — ISO格式时间戳",
    "mode": "str  — official | demo",
    "scenario": "str  — 场景名称",
    "algorithm": "str  — 算法名称",
    "seed": "int  — 随机种子",
    "sim_dt": "float  — 仿真步长(s)",
    "frames": "int  — 总帧数",
    "success": "bool  — 实验是否成功完成",
    "status": "str  — valid | invalid | partial | timeout",
    "failure_reason": "str?  — 失败原因(status!=valid时必填)",
    "missing_fields": "list[str]?  — 缺失字段列表",
    "metrics": "dict[str, float]  — 指标字典(值必须通过校验)",
    "artifacts": "dict[str, str]?  — 产物文件路径",
    "config": "dict  — 完整实验配置(保证可复现)",
}


def validate_experiment_output(data: Dict[str, Any]) -> Tuple[bool, List[str]]:
    """验证实验输出 JSON 是否符合统一 schema。

    Returns:
        (is_valid, errors)
    """
    errors = []
    required = ["experiment_id", "timestamp", "mode", "scenario",
                "algorithm", "seed", "sim_dt", "frames", "success",
                "status", "metrics"]
    for field in required:
        if field not in data:
            errors.append(f"缺少必需字段: {field}")

    # 验证 mode 值
    if data.get("mode") not in ("official", "demo"):
        errors.append(f"mode 必须为 official 或 demo, 实际: {data.get('mode')}")

    # 验证 status 值
    if data.get("status") not in ("valid", "invalid", "partial", "timeout"):
        errors.append(f"status 值非法: {data.get('status')}")

    # 验证 metrics 中无无效值
    metrics = data.get("metrics", {})
    for k, v in metrics.items():
        if is_invalid_metric_value(v):
            errors.append(f"metrics.{k} 包含无效值: {v}")

    # 验证 failure_reason 在 status!=valid 时存在
    if data.get("status") != "valid" and not data.get("failure_reason"):
        errors.append("status 非 valid 时必须提供 failure_reason")

    return len(errors) == 0, errors


# =====================================================================
# 9. 自检
# =====================================================================
def _self_test() -> bool:
    """模块自检，验证校验逻辑正确。"""
    assert is_invalid_metric_value(None) is True
    assert is_invalid_metric_value(float("nan")) is True
    assert is_invalid_metric_value(float("inf")) is True
    assert is_invalid_metric_value(float("-inf")) is True
    assert is_invalid_metric_value(-1) is True
    assert is_invalid_metric_value(0) is False
    assert is_invalid_metric_value(3.14) is False
    assert is_invalid_metric_value(True) is False

    clean, missing = sanitize_metrics_dict({
        "a": 1.0, "b": -1, "c": None, "d": float("inf"), "e": 2.5
    })
    assert clean == {"a": 1.0, "e": 2.5}, f"clean={clean}"
    assert set(missing) == {"b", "c", "d"}, f"missing={missing}"

    r = create_invalid_result(
        "test", "vo", "scene01", 1,
        "metric_parser_failed", ["arrived", "total_points"])
    assert r.status == "invalid"
    assert r.success is False

    vals, skipped = aggregate_valid_results([
        TrialResult("id1", "vo", "s", 1, 0.033, 100, True,
                    metrics={"collision_events": 3.0}),
        r,  # invalid
        TrialResult("id2", "apf", "s", 1, 0.033, 100, True,
                    metrics={"collision_events": -1}),  # missing
    ], "collision_events")
    assert vals == [3.0], f"vals={vals}"
    assert len(skipped) == 2, f"skipped={skipped}"

    return True


if __name__ == "__main__":
    if _self_test():
        print("[metrics_schema] 自检通过")
    else:
        print("[metrics_schema] 自检失败!")
        raise SystemExit(1)
