#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
参数敏感性分析模块 (Parameter Sensitivity Analysis)
=================================================
按 tigao1.md 方向四任务 4.4 设计的参数敏感性分析。

对 4 个关键参数进行扫描:
  1. risk_lambda         - 风险厌恶系数
  2. aufe_boost          - AUFE 权重增幅
  3. dyn_emergency_dist  - 动态避障急停距离
  4. amcl_n_particles    - AMCL 粒子数

对每个参数:
  - 在合理范围内变化 (±50% 或离散值)
  - 记录性能指标 (覆盖率, 碰撞次数, 定位误差)
  - 计算敏感性系数 S = ΔPerformance / ΔParameter
  - 绘制性能-参数曲线

理论说明
--------
1. 参数敏感性分析 (Sensitivity Analysis)
   通过系统地变化某个输入参数，观察其对输出性能的影响，
   从而量化模型对参数的依赖程度。高敏感参数需要精细调优，
   低敏感参数则说明方法在该维度上具有鲁棒性。

2. 敏感性系数 (Sensitivity Coefficient)
   定义一阶敏感性系数:
       S = ΔPerformance / ΔParameter
   当 S 较大时，性能对该参数高度敏感；
   当 S 接近 0 时，性能对该参数鲁棒。
   归一化敏感性系数:
       S_norm = (ΔPerformance / Performance) / (ΔParameter / Parameter)
   可消除量纲影响，便于跨参数比较。

3. 鲁棒性评估
   若参数在 ±50% 范围内变化时性能波动 < 5%，则认为方法
   对该参数鲁棒；若波动 > 10%，则需要精细调优。

用法
----
    python parameter_sensitivity.py run       # 运行全部扫描
    python parameter_sensitivity.py analyze   # 仅分析已有结果
    python parameter_sensitivity.py all       # 运行 + 分析
    python parameter_sensitivity.py --demo    # 演示模式（模拟数据）
"""
import os
import sys
import csv
import json
import math
import subprocess
import argparse
import datetime
from pathlib import Path
from typing import List, Dict, Any, Tuple, Optional

import numpy as np

# === 全局配置 ===
N_TRIALS = 3            # 每个参数值的重复次数
MAX_FRAMES = 4000       # 单次实验帧数上限
RESULTS_DIR = Path("e:/puppyfangzhen/sensitivity_results")
NAV_SCRIPT = Path("e:/puppyfangzhen/autonomous_nav.py")

# 尝试使用非交互式后端
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _MPL_OK = True
except Exception as _mpl_err:  # pragma: no cover
    _MPL_OK = False
    _MPL_ERR = str(_mpl_err)

# 中文字体
_CN_FONT = None
if _MPL_OK:
    for _font in ["Microsoft YaHei", "SimHei", "SimSun",
                  "WenQuanYi Zen Hei", "Noto Sans CJK SC", "DejaVu Sans"]:
        try:
            from matplotlib.font_manager import FontProperties
            fp = FontProperties(family=_font)
            if fp.get_name() != _font and _font != "DejaVu Sans":
                continue
            _CN_FONT = _font
            plt.rcParams["font.sans-serif"] = [_font, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            break
        except Exception:
            continue
    if _CN_FONT is None:
        _CN_FONT = "DejaVu Sans"


# =====================================================================
# 第一部分：参数定义
# =====================================================================
# 4 个关键参数的扫描配置
# 每项: (param_key, label, default_value, values, env_var, physical_meaning)
PARAMETER_CONFIGS = [
    {
        "key": "risk_lambda",
        "label": "风险厌恶系数 λ",
        "default": 0.5,
        "values": [0.1, 0.25, 0.5, 0.75, 1.0, 1.5],
        "env_var": "RISK_LAMBDA",
        "physical_meaning": (
            "Risk-Aware A* 中的风险惩罚权重。λ 越大，规划器越"
            "保守（更倾向远离不确定区域），路径更长但更安全；"
            "λ 越小则更激进（贴近不确定区域），路径更短但定位"
            "误差风险增加。物理意义：对定位/感知不确定性的"
            "厌恶程度。合理范围 [0.1, 1.5]。"),
    },
    {
        "key": "aufe_boost",
        "label": "AUFE 权重增幅 α",
        "default": 1.0,
        "values": [0.5, 0.75, 1.0, 1.25, 1.5],
        "env_var": "AUFE_BOOST",
        "physical_meaning": (
            "AUFE 自适应权重更新的步长增幅因子。α=1 为标准"
            "Regret 最小化步长；α>1 加速权重收敛但可能震荡；"
            "α<1 减速更稳定但收敛慢。物理意义：在线学习率"
            "的缩放系数。合理范围 [0.5, 1.5]。"),
    },
    {
        "key": "dyn_emergency_dist",
        "label": "动态避障急停距离 d_em",
        "default": 0.3,
        "values": [0.15, 0.225, 0.3, 0.375, 0.45],
        "env_var": "DYN_EMERGENCY_DIST",
        "physical_meaning": (
            "动态障碍物紧急制动触发距离（米）。d_em 越大，"
            "机器人在更远处急停，安全性高但效率低（频繁停顿）；"
            "d_em 越小则更贴近障碍物才制动，效率高但碰撞风险"
            "增加。物理意义：安全与效率的权衡阈值。"
            "合理范围 [0.15, 0.45] 米。"),
    },
    {
        "key": "amcl_n_particles",
        "label": "AMCL 粒子数 N_p",
        "default": 500,
        "values": [100, 200, 500, 1000, 2000],
        "env_var": "AMCL_N_PARTICLES",
        "physical_meaning": (
            "AMCL 粒子滤波的粒子数量。N_p 越大定位精度越高、"
            "对绑架问题恢复能力越强，但计算开销线性增加；"
            "N_p 越小计算快但可能粒子枯竭。物理意义：定位"
            "后验分布的采样密度。合理范围 [100, 2000]。"),
    },
]


def get_metric_from_trial(trial: dict, metric: str) -> float:
    """从 trial 结果中提取指定指标值。

    Args:
        trial: 单次实验结果 dict
        metric: 指标名 ('coverage' / 'collision' / 'loc_err')

    Returns:
        指标值
    """
    mapping = {
        "coverage": "final_coverage",
        "collision": "collision_count",
        "loc_err": "mean_loc_err",
    }
    key = mapping.get(metric, metric)
    return trial.get(key, 0.0)


# =====================================================================
# 第二部分：参数扫描
# =====================================================================
class ParameterSweep:
    """单个参数的扫描实验。

    在固定其他参数的前提下，对单个参数在其取值范围内
    逐点扫描，记录性能指标，计算敏感性系数。

    Attributes:
        param_name: 参数名（如 'risk_lambda'）
        param_values: 参数取值列表
        default_value: 默认参数值（基准点）
        results: 扫描结果，结构为 [{value, trials, metrics_summary}]
    """

    def __init__(self, param_name: str, param_values: List,
                 default_value):
        """初始化参数扫描。

        Args:
            param_name: 参数名
            param_values: 参数取值列表
            default_value: 默认参数值（用于对比基准）
        """
        self.param_name = param_name
        self.param_values = list(param_values)
        self.default_value = default_value
        self.results: List[dict] = []
        # 查找参数配置
        self._config = None
        for cfg in PARAMETER_CONFIGS:
            if cfg["key"] == param_name:
                self._config = cfg
                break
        if self._config is None:
            raise ValueError(f"未知参数: {param_name}")

    def run_sweep(self, n_trials: int = 3) -> List[dict]:
        """在每个参数值上运行实验。

        对每个参数值，运行 n_trials 次实验并汇总指标。

        Args:
            n_trials: 每个参数值的重复次数

        Returns:
            扫描结果列表，每项包含:
                - value: 参数值
                - trials: 各 trial 的指标列表
                - metrics_summary: 指标均值±标准差
        """
        self.results = []
        env_var = self._config["env_var"]
        results_dir = RESULTS_DIR / self.param_name
        results_dir.mkdir(parents=True, exist_ok=True)

        for value in self.param_values:
            print(f"\n  [{self.param_name}] 扫描值: {value}")
            # 构造环境变量
            env = os.environ.copy()
            env[env_var] = str(value)
            env["MAX_FRAMES"] = str(MAX_FRAMES)
            env["METHOD_NAME"] = f"sweep_{self.param_name}_{value}"
            env["PYTHONUNBUFFERED"] = "1"

            trials = []
            for trial_id in range(1, n_trials + 1):
                trial_dir = (results_dir / f"val_{value}"
                             / f"trial_{trial_id}")
                trial_dir.mkdir(parents=True, exist_ok=True)
                csv_path = trial_dir / f"sweep_data.csv"
                env["EVAL_DIR"] = str(trial_dir)

                # 随机种子
                seed = hash(f"{self.param_name}_{value}_{trial_id}") % (2 ** 32)
                env["PYTHONHASHSEED"] = str(seed)
                env["NPY_SEED"] = str(seed)

                print(f"    trial {trial_id}/{n_trials} (seed={seed})")
                metrics = self._run_single_experiment(
                    env, csv_path, trial_dir)
                metrics["trial_id"] = trial_id
                metrics["param_value"] = value
                trials.append(metrics)

            # 汇总指标
            metrics_summary = self._summarize_trials(trials)
            self.results.append({
                "value": value,
                "trials": trials,
                "metrics_summary": metrics_summary,
            })
        return self.results

    def _run_single_experiment(self, env: dict, csv_path: Path,
                                trial_dir: Path) -> dict:
        """运行单次实验并加载指标。

        Args:
            env: 环境变量字典
            csv_path: CSV 输出路径
            trial_dir: trial 目录

        Returns:
            指标字典
        """
        cmd = [sys.executable, str(NAV_SCRIPT)]
        proc = subprocess.Popen(
            cmd, env=env, cwd=str(NAV_SCRIPT.parent),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True,
        )
        log_path = trial_dir / "run.log"
        with open(log_path, "w", encoding="utf-8") as log_f:
            for line in proc.stdout:
                log_f.write(line)
        proc.wait()
        metrics = self._load_metrics(csv_path)
        if metrics is None:
            metrics = self._empty_metrics()
        return metrics

    def _load_metrics(self, csv_path: Path) -> Optional[dict]:
        """从 CSV 加载指标。"""
        if not csv_path.exists():
            return None
        rows = []
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for r in reader:
                try:
                    loc_err_val = r.get("loc_err",
                                        r.get("loc_error", "0.0"))
                    rows.append({
                        "frame": int(r["frame"]),
                        "coverage": float(r["coverage"]),
                        "state": int(r["state"]),
                        "loc_err": float(loc_err_val),
                    })
                except (ValueError, KeyError):
                    continue
        if not rows:
            return None
        states = [r["state"] for r in rows]
        loc_errs = [r["loc_err"] for r in rows if r["loc_err"] > 0]
        coverages = [r["coverage"] for r in rows]
        time_to_80 = -1.0
        for r in rows:
            if r["coverage"] >= 80:
                time_to_80 = r["frame"] / 10.0
                break
        recover_count = sum(1 for i in range(1, len(states))
                            if states[i] == 2 and states[i - 1] != 2)
        # 碰撞/穿墙从日志解析
        collision_count = 0
        wall_penetration_count = 0
        log_path = csv_path.parent / "run.log"
        if log_path.exists():
            with open(log_path, "r", encoding="utf-8") as lf:
                for line in lf:
                    if "collision" in line.lower():
                        collision_count += 1
                    if "wall_penetration" in line.lower():
                        wall_penetration_count += 1
        return {
            "final_coverage": coverages[-1] if coverages else 0.0,
            "time_to_80": time_to_80,
            "mean_loc_err": (float(np.mean(loc_errs)) if loc_errs
                             else -1.0),
            "recover_count": recover_count,
            "collision_count": collision_count,
            "wall_penetration_count": wall_penetration_count,
            "total_frames": len(rows),
        }

    def _empty_metrics(self) -> dict:
        """空指标字典（实验失败兜底）。"""
        return {
            "final_coverage": 0.0,
            "time_to_80": -1.0,
            "mean_loc_err": -1.0,
            "recover_count": 0,
            "collision_count": 0,
            "wall_penetration_count": 0,
            "total_frames": 0,
        }

    def _summarize_trials(self, trials: List[dict]) -> dict:
        """汇总多次 trial 的指标（均值±标准差）。"""
        summary = {}
        for key in ["final_coverage", "time_to_80", "mean_loc_err",
                    "recover_count", "collision_count",
                    "wall_penetration_count"]:
            values = [t[key] for t in trials if t[key] >= 0]
            if values:
                summary[key] = {
                    "mean": float(np.mean(values)),
                    "std": (float(np.std(values, ddof=1))
                            if len(values) > 1 else 0.0),
                    "min": float(np.min(values)),
                    "max": float(np.max(values)),
                    "n": len(values),
                }
            else:
                summary[key] = {"mean": -1.0, "std": 0.0,
                                "min": -1.0, "max": -1.0, "n": 0}
        return summary

    def compute_sensitivity(self, metric: str = "coverage") -> float:
        """计算敏感性系数 S = ΔPerformance / ΔParameter。

        使用端点差分法：取参数最大值与最小值对应的性能差，
        除以参数差值。正值表示正相关，负值表示反相关。

        Args:
            metric: 指标名 ('coverage' / 'collision' / 'loc_err')

        Returns:
            敏感性系数 S。若结果不足则返回 0.0。
        """
        if len(self.results) < 2:
            return 0.0
        # 取首尾两点
        first = self.results[0]
        last = self.results[-1]
        param_diff = last["value"] - first["value"]
        if abs(param_diff) < 1e-12:
            return 0.0
        perf_key = {"coverage": "final_coverage",
                    "collision": "collision_count",
                    "loc_err": "mean_loc_err"}.get(metric, metric)
        perf_first = first["metrics_summary"].get(
            perf_key, {}).get("mean", 0.0)
        perf_last = last["metrics_summary"].get(
            perf_key, {}).get("mean", 0.0)
        perf_diff = perf_last - perf_first
        return float(perf_diff / param_diff)

    def compute_normalized_sensitivity(self,
                                        metric: str = "coverage") -> float:
        """计算归一化敏感性系数。

        S_norm = (ΔP / P) / (Δx / x)

        消除量纲影响，便于跨参数比较。

        Args:
            metric: 指标名

        Returns:
            归一化敏感性系数
        """
        if len(self.results) < 2:
            return 0.0
        first = self.results[0]
        last = self.results[-1]
        param_diff = last["value"] - first["value"]
        param_mid = (first["value"] + last["value"]) / 2.0
        if abs(param_diff) < 1e-12 or abs(param_mid) < 1e-12:
            return 0.0
        perf_key = {"coverage": "final_coverage",
                    "collision": "collision_count",
                    "loc_err": "mean_loc_err"}.get(metric, metric)
        perf_first = first["metrics_summary"].get(
            perf_key, {}).get("mean", 0.0)
        perf_last = last["metrics_summary"].get(
            perf_key, {}).get("mean", 0.0)
        perf_mid = (perf_first + perf_last) / 2.0
        if abs(perf_mid) < 1e-12:
            return 0.0
        return float(((perf_last - perf_first) / perf_mid) /
                     (param_diff / param_mid))

    def compute_robustness(self, metric: str = "coverage") -> float:
        """计算鲁棒性指标。

        定义为性能波动范围（max - min）/ 均值。
        值越小表示越鲁棒（<0.05 为鲁棒，>0.10 需调优）。

        Args:
            metric: 指标名

        Returns:
            鲁棒性指标（相对波动率）
        """
        if not self.results:
            return 1.0
        perf_key = {"coverage": "final_coverage",
                    "collision": "collision_count",
                    "loc_err": "mean_loc_err"}.get(metric, metric)
        values = [r["metrics_summary"].get(perf_key, {}).get("mean", 0.0)
                  for r in self.results]
        values = [v for v in values if v >= 0]
        if not values or abs(np.mean(values)) < 1e-12:
            return 1.0
        return float((max(values) - min(values)) / abs(np.mean(values)))


# =====================================================================
# 第三部分：报告生成
# =====================================================================
class SensitivityReport:
    """参数敏感性分析报告生成器。

    生成 markdown 报告与可视化曲线图。
    """

    @staticmethod
    def generate_report(results: dict) -> str:
        """生成 markdown 格式的敏感性分析报告。

        Args:
            results: 分析结果字典，应包含:
                - parameters: 各参数的扫描结果
                - sensitivity_ranking: 敏感性排序
                - timestamp: 生成时间

        Returns:
            markdown 报告字符串
        """
        lines = []
        lines.append("# 参数敏感性分析报告\n")
        lines.append(f"生成时间: {results.get('timestamp', 'N/A')}\n")
        lines.append(f"扫描参数数: {results.get('n_params', 'N/A')}\n")
        lines.append(f"每参数重复次数: {results.get('n_trials', 'N/A')}\n")

        # 敏感性排序
        ranking = results.get("sensitivity_ranking", [])
        if ranking:
            lines.append("\n## 1. 参数敏感性排序（按 |S_norm| 降序）\n")
            lines.append("| 排名 | 参数 | 归一化敏感度 | 鲁棒性 | 评价 |\n")
            lines.append("|---|---|---|---|---|\n")
            for rank, (name, s) in enumerate(ranking, 1):
                robust = results.get("robustness", {}).get(name, 1.0)
                if abs(s) > 0.5:
                    verdict = "高敏感，需精细调优"
                elif abs(s) > 0.1:
                    verdict = "中敏感"
                else:
                    verdict = "低敏感，鲁棒"
                lines.append(f"| {rank} | {name} | {s:.4f} | "
                             f"{robust:.4f} | {verdict} |\n")

        # 各参数详情
        parameters = results.get("parameters", {})
        for param_key, param_data in parameters.items():
            lines.append(f"\n## 2. 参数: {param_data.get('label', param_key)}\n")
            lines.append(f"**物理意义**: {param_data.get('physical_meaning', 'N/A')}\n")
            lines.append(f"**默认值**: {param_data.get('default_value', 'N/A')}\n")
            lines.append(f"**扫描范围**: {param_data.get('values', [])}\n\n")

            # 敏感性系数
            sens = param_data.get("sensitivity", {})
            lines.append("### 敏感性系数\n")
            lines.append("| 指标 | S (绝对) | S_norm (归一化) | 鲁棒性 |\n")
            lines.append("|---|---|---|---|\n")
            for metric, s_val in sens.items():
                s_norm = param_data.get(
                    "normalized_sensitivity", {}).get(metric, 0.0)
                rob = param_data.get(
                    "robustness", {}).get(metric, 0.0)
                lines.append(f"| {metric} | {s_val:.4f} | "
                             f"{s_norm:.4f} | {rob:.4f} |\n")

            # 性能-参数表
            sweep = param_data.get("sweep_results", [])
            if sweep:
                lines.append("\n### 性能-参数数据\n")
                lines.append("| 参数值 | 覆盖率(%) | 碰撞次数 | "
                             "定位误差(m) |\n")
                lines.append("|---|---|---|---|\n")
                for item in sweep:
                    val = item.get("value", "")
                    ms = item.get("metrics_summary", {})
                    cov = ms.get("final_coverage", {}).get("mean", 0)
                    col = ms.get("collision_count", {}).get("mean", 0)
                    loc = ms.get("mean_loc_err", {}).get("mean", 0)
                    lines.append(f"| {val} | {cov:.2f} | "
                                 f"{col:.2f} | {loc:.4f} |\n")

        lines.append("\n## 3. 结论\n")
        if ranking:
            most_sensitive = ranking[0][0] if ranking else "N/A"
            least_sensitive = ranking[-1][0] if ranking else "N/A"
            lines.append(f"- 最敏感参数: **{most_sensitive}**，"
                         "需要精细调优。\n")
            lines.append(f"- 最不敏感参数: **{least_sensitive}**，"
                         "方法在该维度上鲁棒。\n")
        lines.append("- 建议对高敏感参数进行网格搜索或贝叶斯优化，"
                     "低敏感参数可使用默认值。\n")
        return "".join(lines)

    @staticmethod
    def plot_sensitivity_curves(results: dict, output_dir: str):
        """绘制 4 个参数的敏感性曲线。

        为每个参数生成一张子图，横轴为参数值，纵轴为性能指标，
        绘制覆盖率、碰撞次数、定位误差三条曲线（归一化到 [0,1]）。

        Args:
            results: 分析结果字典
            output_dir: 图片输出目录
        """
        if not _MPL_OK:
            print(f"[WARN] matplotlib 不可用，跳过绘图: {_MPL_ERR}")
            return None
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        parameters = results.get("parameters", {})
        n_params = len(parameters)
        if n_params == 0:
            print("[WARN] 无参数数据可绘制")
            return None

        # 2x2 子图布局
        n_cols = 2
        n_rows = (n_params + 1) // 2
        fig, axes = plt.subplots(n_rows, n_cols,
                                 figsize=(12, 5 * n_rows))
        if n_params == 1:
            axes = np.array([[axes]])
        elif n_rows == 1:
            axes = axes.reshape(1, -1)
        elif n_cols == 1:
            axes = axes.reshape(-1, 1)

        for idx, (param_key, param_data) in enumerate(parameters.items()):
            row, col = idx // n_cols, idx % n_cols
            ax = axes[row][col]
            sweep = param_data.get("sweep_results", [])
            if not sweep:
                ax.set_title(f"{param_data.get('label', param_key)}\n(无数据)")
                continue

            # 提取数据
            param_values = [item["value"] for item in sweep]
            coverages = [item["metrics_summary"].get(
                "final_coverage", {}).get("mean", 0) for item in sweep]
            cov_stds = [item["metrics_summary"].get(
                "final_coverage", {}).get("std", 0) for item in sweep]
            collisions = [item["metrics_summary"].get(
                "collision_count", {}).get("mean", 0) for item in sweep]
            loc_errs = [item["metrics_summary"].get(
                "mean_loc_err", {}).get("mean", 0) for item in sweep]

            # 标记默认值
            default_val = param_data.get("default_value")

            # 左轴：覆盖率
            color_cov = "#2196F3"
            ax.set_xlabel(param_data.get("label", param_key), fontsize=11)
            ax.set_ylabel("覆盖率(%)", color=color_cov, fontsize=11)
            line1 = ax.errorbar(param_values, coverages,
                                yerr=cov_stds,
                                color=color_cov, marker="o",
                                linewidth=2, markersize=8,
                                label="覆盖率",
                                capsize=4, capthick=1)
            ax.tick_params(axis="y", labelcolor=color_cov)
            # 默认值竖线
            if default_val is not None and default_val in param_values:
                ax.axvline(default_val, color="gray",
                           linestyle="--", alpha=0.5,
                           label=f"默认值={default_val}")

            # 右轴：碰撞次数 + 定位误差（归一化）
            ax2 = ax.twinx()
            color_col = "#F44336"
            color_loc = "#4CAF50"
            # 归一化碰撞到 [0, max_col]
            if max(collisions) > 0:
                col_norm = [c / max(collisions) * 100
                            for c in collisions]
            else:
                col_norm = collisions
            # 归一化定位误差
            if max(loc_errs) > 0:
                loc_norm = [l / max(loc_errs) * 100
                            for l in loc_errs]
            else:
                loc_norm = loc_errs
            line2 = ax2.plot(param_values, col_norm,
                            color=color_col, marker="s",
                            linewidth=2, markersize=7,
                            linestyle="--", label="碰撞(归一化)")
            line3 = ax2.plot(param_values, loc_norm,
                            color=color_loc, marker="^",
                            linewidth=2, markersize=7,
                            linestyle=":", label="误差(归一化)")
            ax2.set_ylabel("碰撞/误差(归一化%)", fontsize=10)

            # 合并图例
            lines_all = [line1] + line2 + line3
            labels = [l.get_label() for l in lines_all]
            ax.legend(lines_all, labels, loc="best", fontsize=8)

            ax.set_title(param_data.get("label", param_key),
                         fontsize=12, fontweight="bold")
            ax.grid(True, alpha=0.3)

        # 隐藏多余子图
        for idx in range(n_params, n_rows * n_cols):
            row, col = idx // n_cols, idx % n_cols
            axes[row][col].set_visible(False)

        plt.suptitle("参数敏感性曲线", fontsize=14, fontweight="bold",
                     y=1.0)
        plt.tight_layout()
        output_path = output_dir / "sensitivity_curves.png"
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] 敏感性曲线已保存: {output_path}")
        return str(output_path)


# =====================================================================
# 第四部分：全局敏感性分析
# =====================================================================
class GlobalSensitivityAnalysis:
    """全局敏感性分析。

    对 4 个关键参数依次进行扫描，汇总结果并排序。

    Attributes:
        n_trials: 每个参数值的重复次数
        results: 全部分析结果
    """

    def __init__(self, n_trials: int = 3):
        """初始化全局敏感性分析。

        Args:
            n_trials: 每个参数值的重复次数
        """
        self.n_trials = n_trials
        self.results: dict = {}

    def run_all(self) -> dict:
        """对 4 个参数全部扫描。

        Returns:
            完整分析结果字典，包含各参数的扫描数据、
            敏感性系数、鲁棒性指标和排序。
        """
        print("\n" + "=" * 80)
        print("  参数敏感性分析（4 参数扫描）")
        print("=" * 80)

        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        parameters_result = {}
        sensitivity_scores = {}

        for cfg in PARAMETER_CONFIGS:
            print(f"\n{'─' * 60}")
            print(f"  扫描参数: {cfg['label']} ({cfg['key']})")
            print(f"  默认值: {cfg['default']}")
            print(f"  扫描值: {cfg['values']}")
            print(f"  物理意义: {cfg['physical_meaning']}")
            print(f"{'─' * 60}")

            sweep = ParameterSweep(
                cfg["key"], cfg["values"], cfg["default"])
            sweep_results = sweep.run_sweep(self.n_trials)

            # 计算各指标的敏感性
            sensitivity = {}
            normalized_sensitivity = {}
            robustness = {}
            for metric in ["coverage", "collision", "loc_err"]:
                sensitivity[metric] = sweep.compute_sensitivity(metric)
                normalized_sensitivity[metric] = (
                    sweep.compute_normalized_sensitivity(metric))
                robustness[metric] = sweep.compute_robustness(metric)

            # 主指标（覆盖率）的归一化敏感度用于排序
            primary_s = abs(normalized_sensitivity.get("coverage", 0))

            parameters_result[cfg["key"]] = {
                "label": cfg["label"],
                "default_value": cfg["default"],
                "values": cfg["values"],
                "physical_meaning": cfg["physical_meaning"],
                "sweep_results": sweep_results,
                "sensitivity": sensitivity,
                "normalized_sensitivity": normalized_sensitivity,
                "robustness": robustness,
                "primary_sensitivity": primary_s,
            }
            sensitivity_scores[cfg["key"]] = primary_s
            print(f"\n  归一化敏感度 (coverage): {primary_s:.4f}")
            print(f"  鲁棒性指标 (coverage): "
                  f"{robustness.get('coverage', 0):.4f}")

        # 排序
        ranking = self.rank_parameters()

        self.results = {
            "timestamp": datetime.datetime.now().isoformat(),
            "n_params": len(PARAMETER_CONFIGS),
            "n_trials": self.n_trials,
            "parameters": parameters_result,
            "sensitivity_ranking": ranking,
            "robustness": {k: v["robustness"].get("coverage", 0)
                           for k, v in parameters_result.items()},
        }

        # 保存 JSON
        self._save_results()
        return self.results

    def rank_parameters(self) -> List[Tuple[str, float]]:
        """按敏感度排序（降序）。

        Returns:
            (param_name, sensitivity) 元组列表，按 |S_norm| 降序
        """
        if not self.results:
            return []
        scores = []
        for key, data in self.results.get("parameters", {}).items():
            scores.append((key, data.get("primary_sensitivity", 0.0)))
        # 按绝对值降序
        scores.sort(key=lambda x: abs(x[1]), reverse=True)
        return scores

    def _save_results(self):
        """保存结果到 JSON 文件。"""
        output_path = RESULTS_DIR / "sensitivity_results.json"
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False,
                      default=_json_default)
        print(f"\n  结果已保存: {output_path}")

        # 生成 markdown 报告
        report = SensitivityReport.generate_report(self.results)
        report_path = RESULTS_DIR / "sensitivity_report.md"
        with open(report_path, "w", encoding="utf-8") as f:
            f.write(report)
        print(f"  报告已保存: {report_path}")

        # 绘制曲线
        SensitivityReport.plot_sensitivity_curves(
            self.results, str(RESULTS_DIR))


# =====================================================================
# 第五部分：模拟数据生成（demo 模式）
# =====================================================================
def _generate_synthetic_sweep(param_key: str, param_values: list,
                              default_value, n_trials: int,
                              rng=None) -> List[dict]:
    """为参数扫描生成模拟数据。

    根据参数的物理意义生成合理的性能曲线：
        - risk_lambda: 倒 U 型（中间最优）
        - aufe_boost: 缓慢上升后饱和
        - dyn_emergency_dist: 覆盖率随 d_em 增大略降，碰撞减少
        - amcl_n_particles: 对数增长（边际递减）

    Args:
        param_key: 参数名
        param_values: 参数取值列表
        default_value: 默认值
        n_trials: 重复次数
        rng: 随机数生成器

    Returns:
        扫描结果列表
    """
    if rng is None:
        rng = np.random.default_rng(
            seed=hash(param_key) % (2 ** 32))

    results = []
    for val in param_values:
        # 根据参数类型生成性能曲线
        if param_key == "risk_lambda":
            # 倒 U 型：λ=0.5 最优
            opt = 0.5
            cov = 94.0 - 8.0 * ((val - opt) / opt) ** 2
            col = max(0, 1.5 + 2.0 * abs(val - opt))
            loc_err = 0.08 + 0.1 * abs(val - opt)
        elif param_key == "aufe_boost":
            # 缓慢上升后饱和
            cov = 90.0 + 4.0 * (1 - math.exp(-val))
            col = max(0, 1.0 - 0.3 * val)
            loc_err = max(0.05, 0.15 - 0.05 * val)
        elif param_key == "dyn_emergency_dist":
            # d_em 增大：覆盖率略降，碰撞减少
            cov = 94.0 - 5.0 * (val - 0.3) / 0.3
            col = max(0, 3.0 - 6.0 * (val - 0.15))
            loc_err = 0.08
        elif param_key == "amcl_n_particles":
            # 对数增长
            cov = 88.0 + 6.0 * math.log10(val / 100)
            col = max(0, 1.5 - 0.5 * math.log10(val / 100))
            loc_err = max(0.02, 0.2 / math.log10(val / 10))
        else:
            cov = 90.0
            col = 1.0
            loc_err = 0.08

        trials = []
        for i in range(n_trials):
            noise = rng.normal(0, 1.0)
            trials.append({
                "final_coverage": float(np.clip(cov + noise, 40, 99.5)),
                "time_to_80": float(np.clip(
                    250 + (94 - cov) * 10 + rng.normal(0, 20), 60, 800)),
                "mean_loc_err": float(max(0.01, loc_err + rng.normal(0, 0.01))),
                "recover_count": int(rng.integers(0, 3)),
                "collision_count": max(0, int(col + rng.normal(0, 0.5))),
                "wall_penetration_count": int(rng.integers(0, 1)),
                "total_frames": MAX_FRAMES,
                "trial_id": i + 1,
                "param_value": val,
            })
        # 汇总
        summary = {}
        for key in ["final_coverage", "time_to_80", "mean_loc_err",
                    "recover_count", "collision_count",
                    "wall_penetration_count"]:
            values = [t[key] for t in trials if t[key] >= 0]
            if values:
                summary[key] = {
                    "mean": float(np.mean(values)),
                    "std": (float(np.std(values, ddof=1))
                            if len(values) > 1 else 0.0),
                    "min": float(np.min(values)),
                    "max": float(np.max(values)),
                    "n": len(values),
                }
        results.append({
            "value": val,
            "trials": trials,
            "metrics_summary": summary,
        })
    return results


def run_demo():
    """使用模拟数据演示参数敏感性分析。

    当 CoppeliaSim 不可用时，此模式可独立运行:
        python parameter_sensitivity.py --demo
    """
    print("\n" + "=" * 80)
    print("  参数敏感性分析 —— 演示模式")
    print("  使用模拟数据进行完整分析")
    print("=" * 80)

    rng = np.random.default_rng(seed=42)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    parameters_result = {}
    for cfg in PARAMETER_CONFIGS:
        print(f"\n  [模拟] {cfg['label']} ({cfg['key']})")
        sweep_results = _generate_synthetic_sweep(
            cfg["key"], cfg["values"], cfg["default"],
            N_TRIALS, rng=rng)

        # 构造 ParameterSweep 对象以计算敏感性
        sweep = ParameterSweep(
            cfg["key"], cfg["values"], cfg["default"])
        sweep.results = sweep_results

        sensitivity = {}
        normalized_sensitivity = {}
        robustness = {}
        for metric in ["coverage", "collision", "loc_err"]:
            sensitivity[metric] = sweep.compute_sensitivity(metric)
            normalized_sensitivity[metric] = (
                sweep.compute_normalized_sensitivity(metric))
            robustness[metric] = sweep.compute_robustness(metric)

        primary_s = abs(normalized_sensitivity.get("coverage", 0))
        parameters_result[cfg["key"]] = {
            "label": cfg["label"],
            "default_value": cfg["default"],
            "values": cfg["values"],
            "physical_meaning": cfg["physical_meaning"],
            "sweep_results": sweep_results,
            "sensitivity": sensitivity,
            "normalized_sensitivity": normalized_sensitivity,
            "robustness": robustness,
            "primary_sensitivity": primary_s,
        }
        print(f"    归一化敏感度: {primary_s:.4f}")
        print(f"    鲁棒性: {robustness.get('coverage', 0):.4f}")

    # 排序
    scores = [(k, v["primary_sensitivity"])
              for k, v in parameters_result.items()]
    scores.sort(key=lambda x: abs(x[1]), reverse=True)

    results = {
        "timestamp": datetime.datetime.now().isoformat(),
        "mode": "demo",
        "n_params": len(PARAMETER_CONFIGS),
        "n_trials": N_TRIALS,
        "parameters": parameters_result,
        "sensitivity_ranking": scores,
        "robustness": {k: v["robustness"].get("coverage", 0)
                       for k, v in parameters_result.items()},
    }

    # 保存
    output_path = RESULTS_DIR / "sensitivity_results_demo.json"
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False,
                  default=_json_default)
    print(f"\n  结果已保存: {output_path}")

    # 报告
    report = SensitivityReport.generate_report(results)
    report_path = RESULTS_DIR / "sensitivity_report_demo.md"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    print(f"  报告已保存: {report_path}")

    # 曲线
    SensitivityReport.plot_sensitivity_curves(
        results, str(RESULTS_DIR))

    # 打印汇总
    print("\n" + "=" * 80)
    print("  敏感性排序（降序）")
    print("=" * 80)
    for rank, (name, s) in enumerate(scores, 1):
        label = next((c["label"] for c in PARAMETER_CONFIGS
                      if c["key"] == name), name)
        rob = results["robustness"].get(name, 0)
        if abs(s) > 0.5:
            verdict = "高敏感，需精细调优"
        elif abs(s) > 0.1:
            verdict = "中敏感"
        else:
            verdict = "低敏感，鲁棒"
        print(f"  {rank}. {label}: |S_norm|={abs(s):.4f}, "
              f"鲁棒性={rob:.4f} → {verdict}")

    print("\n" + "=" * 80)
    print(f"  演示完成！结果保存在: {RESULTS_DIR}")
    print("=" * 80)
    for p in sorted(RESULTS_DIR.iterdir()):
        print(f"    - {p.name} ({p.stat().st_size} 字节)")
    return results


def _json_default(obj):
    """JSON 序列化兜底（处理 numpy 类型）。"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    raise TypeError(f"无法序列化: {type(obj)}")


# =====================================================================
# 第六部分：命令行入口
# =====================================================================
def run_all():
    """运行全部参数敏感性扫描。"""
    analyzer = GlobalSensitivityAnalysis(n_trials=N_TRIALS)
    analyzer.run_all()


def main():
    """命令行入口。"""
    global N_TRIALS
    parser = argparse.ArgumentParser(
        description="参数敏感性分析模块")
    parser.add_argument(
        "command", nargs="?", default="run",
        choices=["run", "analyze", "all"],
        help="命令: run=运行扫描, analyze=分析已有结果, all=运行+分析")
    parser.add_argument(
        "--demo", action="store_true",
        help="使用模拟数据演示敏感性分析")
    parser.add_argument(
        "--n-trials", type=int, default=N_TRIALS,
        help=f"每个参数值的重复次数（默认 {N_TRIALS}）")
    args = parser.parse_args()

    N_TRIALS = args.n_trials

    if args.demo:
        run_demo()
        return
    if args.command in ("run", "all"):
        run_all()
    elif args.command == "analyze":
        # 仅分析已有结果
        results_path = RESULTS_DIR / "sensitivity_results.json"
        if results_path.exists():
            with open(results_path, "r", encoding="utf-8") as f:
                results = json.load(f)
            report = SensitivityReport.generate_report(results)
            report_path = RESULTS_DIR / "sensitivity_report.md"
            with open(report_path, "w", encoding="utf-8") as f:
                f.write(report)
            print(f"报告已生成: {report_path}")
            SensitivityReport.plot_sensitivity_curves(
                results, str(RESULTS_DIR))
        else:
            print(f"未找到结果文件: {results_path}")
            print("请先运行 'python parameter_sensitivity.py run'")


if __name__ == "__main__":
    main()
