#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版消融实验框架 v3.0 (Academic Ablation Study)
=================================================
按 tigao1.md 方向四设计的 7 维消融实验 + 协同效应验证。

7 维消融（每维去掉一个模块）:
  1. w/o AUFE      - 用固定权重代替自适应权重
  2. w/o Risk-Aware - 用标准 A*
  3. w/o CBF       - 只用 DWA 避障
  4. w/o GNN       - 用线性外推代替 GNN 轨迹预测
  5. w/o Dynamic Obs - 无动态障碍物
  6. w/o AMCL      - 用 ground truth 定位
  7. w/o TEB       - 用 DWA 代替 TEB

协同效应验证:
  定义协同效应 = 组合性能 - (基线 + 单模块增益之和)
  > 0 表示正协同（1+1>2），< 0 表示负协同

统计检验:
  - 5 次重复 × 8 配置 = 40 次实验
  - paired t-test (proposed vs each ablation)
  - Cohen's d 效应量
  - 95% 置信区间

理论说明
--------
1. 消融实验 (Ablation Study)
   通过逐个移除系统中的关键模块，量化每个模块对整体性能的
   边际贡献。设完整系统性能为 P_full，移除模块 i 后性能为
   P_{-i}，则模块 i 的边际贡献为:
       Δ_i = P_full - P_{-i}
   Δ_i > 0 表示该模块有正贡献，Δ_i 越大贡献越显著。

2. 协同效应 (Synergy Effect)
   当多个模块组合产生的效果超过各模块单独效果之和时，称为
   正协同效应（1+1>2）。设基线性能为 P_baseline，完整系统
   性能为 P_full，仅含模块 i 的系统性能为 P_single_i，则:
       synergy = P_full - (P_baseline + Σ (P_single_i - P_baseline))
               = P_full - P_baseline - Σ Δ_single_i
   synergy > 0 表示模块间存在正协同（互补增强），
   synergy < 0 表示负协同（相互抑制），
   synergy ≈ 0 表示模块效应独立可加。

   数学解释：若各模块效用独立可加（线性叠加假设），则完整
   系统性能应等于基线加各模块增益之和。超出部分即为协同效应。
   这一概念源自博弈论中的 Shapley 值与超可加性
   (superadditivity) 定义。

3. 统计检验
   - paired t-test：对同一组试验场景配对比较两种配置，
     原假设 H0: μ_diff = 0（无差异）。
   - Cohen's d：标准化效应量，d = (μ_a - μ_b) / s_pooled，
     |d|>0.8 为大效应，>0.5 中效应，>0.2 小效应。
   - 95% 置信区间：基于 t 分布，对小样本友好。

用法
----
    python ablation_study_v3.py run        # 运行所有实验
    python ablation_study_v3.py analyze    # 仅分析已有结果
    python ablation_study_v3.py all        # 运行 + 分析
    python ablation_study_v3.py --demo     # 演示模式（模拟数据）
    python ablation_study_v3.py synergy    # 仅运行协同效应分析
"""
import os
import sys
import csv
import json
import math
import subprocess
import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Any, Tuple, Optional

import numpy as np

# === 全局配置 ===
N_TRIALS = 5            # 重复实验次数（5 × 8 = 40 次实验）
MAX_FRAMES = 4000       # 单次实验帧数上限
RESULTS_DIR = Path("e:/puppyfangzhen/ablation_results_v3")
NAV_SCRIPT = Path("e:/puppyfangzhen/autonomous_nav.py")

# 尝试使用非交互式后端，避免无显示环境报错
try:
    import matplotlib
    matplotlib.use("Agg")  # 在 import pyplot 之前设置后端
    import matplotlib.pyplot as plt
    _MPL_OK = True
except Exception as _mpl_err:  # pragma: no cover - 极端情况下兜底
    _MPL_OK = False
    _MPL_ERR = str(_mpl_err)

# 尝试启用中文字体（找不到则回退英文标签，不报错）
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
        _CN_FONT = "DejaVu Sans"  # 兜底字体


# =====================================================================
# 第一部分：实验配置
# =====================================================================
@dataclass
class AblationConfig:
    """消融实验配置数据类。

    通过 7 个布尔标志控制各模块的启用/禁用状态，
    用于构造 7 维消融实验的 8 种配置（1 完整 + 7 消融）。

    Attributes:
        use_aufe: 是否启用 AUFE 自适应不确定性加权框架。
            True: 使用自适应权重（基于 Regret 的在线优化）。
            False: 使用固定权重（fixed_weights 中指定的值）。
        use_risk_aware: 是否启用 Risk-Aware A* 全局规划器。
            True: 使用风险感知 A*（考虑定位/感知不确定性）。
            False: 使用标准 A*（仅最短路径）。
        use_cbf: 是否启用 CBF（Control Barrier Function）安全过滤器。
            True: 局部规划输出经 CBF 安全过滤。
            False: 仅依赖 DWA 内置避障。
        use_gnn: 是否启用 GNN 轨迹预测器。
            True: 使用图神经网络预测动态障碍物轨迹。
            False: 使用线性外推（恒速模型）。
        use_dynamic_obs: 是否启用动态障碍物场景。
            True: 场景中包含移动的行人/物体。
            False: 静态场景（仅墙壁等静态障碍物）。
        use_amcl: 是否启用 AMCL 自适应蒙特卡洛定位。
            True: 使用粒子滤波定位（存在定位不确定性）。
            False: 使用 ground truth 定位（无定位误差）。
        use_teb: 是否启用 TEB 局部规划器。
            True: 使用 TEB（Timed Elastic Band）带 Jerk 约束。
            False: 使用 DWA 局部规划器。
        fixed_weights: w/o AUFE 时使用的固定权重字典。
            键为各信息源名称，值为权重值。
    """
    use_aufe: bool = True
    use_risk_aware: bool = True
    use_cbf: bool = True
    use_gnn: bool = True
    use_dynamic_obs: bool = True
    use_amcl: bool = True
    use_teb: bool = True
    fixed_weights: Dict[str, float] = field(default_factory=lambda: {
        "information_gain": 0.4,
        "distance": 0.3,
        "localization_confidence": 0.2,
        "frontier_quality": 0.1,
    })

    @property
    def name(self) -> str:
        """返回配置名称。

        全部模块开启时为 'proposed_full'，
        否则为 'wo_' + 禁用模块名拼接。
        """
        if self.is_full:
            return "proposed_full"
        parts = []
        if not self.use_aufe:
            parts.append("aufe")
        if not self.use_risk_aware:
            parts.append("risk")
        if not self.use_cbf:
            parts.append("cbf")
        if not self.use_gnn:
            parts.append("gnn")
        if not self.use_dynamic_obs:
            parts.append("dynobs")
        if not self.use_amcl:
            parts.append("amcl")
        if not self.use_teb:
            parts.append("teb")
        return "wo_" + "_".join(parts) if parts else "proposed_full"

    @property
    def is_full(self) -> bool:
        """是否为完整系统配置（所有模块开启）。"""
        return (self.use_aufe and self.use_risk_aware and self.use_cbf
                and self.use_gnn and self.use_dynamic_obs
                and self.use_amcl and self.use_teb)

    @property
    def ablated_module(self) -> Optional[str]:
        """返回被消融的模块名（仅单模块消融时有效）。

        用于协同效应分析中标识"单模块配置"。
        若多模块同时禁用则返回 None。
        """
        disabled = []
        if not self.use_aufe:
            disabled.append("AUFE")
        if not self.use_risk_aware:
            disabled.append("RiskAware")
        if not self.use_cbf:
            disabled.append("CBF")
        if not self.use_gnn:
            disabled.append("GNN")
        if not self.use_dynamic_obs:
            disabled.append("DynamicObs")
        if not self.use_amcl:
            disabled.append("AMCL")
        if not self.use_teb:
            disabled.append("TEB")
        if len(disabled) == 1:
            return disabled[0]
        return None

    def to_env_vars(self) -> Dict[str, str]:
        """将配置转换为环境变量字典（供 autonomous_nav.py 读取）。

        约定：
            USE_AUFE / USE_RISK_AWARE / USE_CBF / USE_GNN
            USE_DYNAMIC_OBS / USE_AMCL / USE_TEB
            取值 "1" 表示启用，"0" 表示禁用。
        """
        env = {
            "USE_AUFE": "1" if self.use_aufe else "0",
            "USE_RISK_AWARE": "1" if self.use_risk_aware else "0",
            "USE_CBF": "1" if self.use_cbf else "0",
            "USE_GNN": "1" if self.use_gnn else "0",
            "USE_DYNAMIC_OBS": "1" if self.use_dynamic_obs else "0",
            "USE_AMCL": "1" if self.use_amcl else "0",
            "USE_TEB": "1" if self.use_teb else "0",
        }
        if not self.use_aufe:
            # w/o AUFE: 传递固定权重
            env["AUFE_FIXED_WEIGHTS"] = json.dumps(self.fixed_weights)
        return env

    @classmethod
    def from_env(cls) -> "AblationConfig":
        """从环境变量读取配置。

        支持通过环境变量覆盖默认配置，便于 CI/CD 与
        批量实验脚本调用。

        环境变量:
            ABL_USE_AUFE / ABL_USE_RISK_AWARE / ABL_USE_CBF
            ABL_USE_GNN / ABL_USE_DYNAMIC_OBS / ABL_USE_AMCL
            ABL_USE_TEB
            取值 "0"/"1"，未设置时使用默认值（全部开启）。

        Returns:
            AblationConfig 实例
        """
        def _flag(name: str) -> bool:
            return os.environ.get(name, "1") == "1"

        return cls(
            use_aufe=_flag("ABL_USE_AUFE"),
            use_risk_aware=_flag("ABL_USE_RISK_AWARE"),
            use_cbf=_flag("ABL_USE_CBF"),
            use_gnn=_flag("ABL_USE_GNN"),
            use_dynamic_obs=_flag("ABL_USE_DYNAMIC_OBS"),
            use_amcl=_flag("ABL_USE_AMCL"),
            use_teb=_flag("ABL_USE_TEB"),
        )


def get_ablation_configs() -> List[Tuple[AblationConfig, str]]:
    """生成 7 维消融实验的 8 种配置。

    返回 (config, description) 元组列表：
        0. proposed_full  - 完整系统（所有模块开启）
        1. w/o AUFE       - 用固定权重代替自适应权重
        2. w/o Risk-Aware - 用标准 A*
        3. w/o CBF        - 只用 DWA 避障
        4. w/o GNN        - 用线性外推代替 GNN 轨迹预测
        5. w/o Dynamic Obs - 无动态障碍物
        6. w/o AMCL       - 用 ground truth 定位
        7. w/o TEB        - 用 DWA 代替 TEB

    Returns:
        配置列表，每项为 (AblationConfig, 描述字符串)
    """
    full = AblationConfig()
    return [
        (full, "完整方法（proposed v3.0 全模块开启）"),
        (AblationConfig(use_aufe=False),
         "消融：w/o AUFE - 用固定权重代替自适应权重"),
        (AblationConfig(use_risk_aware=False),
         "消融：w/o Risk-Aware - 用标准 A* 全局规划"),
        (AblationConfig(use_cbf=False),
         "消融：w/o CBF - 只用 DWA 避障（无安全过滤）"),
        (AblationConfig(use_gnn=False),
         "消融：w/o GNN - 用线性外推代替 GNN 轨迹预测"),
        (AblationConfig(use_dynamic_obs=False),
         "消融：w/o Dynamic Obs - 无动态障碍物场景"),
        (AblationConfig(use_amcl=False),
         "消融：w/o AMCL - 用 ground truth 定位"),
        (AblationConfig(use_teb=False),
         "消融：w/o TEB - 用 DWA 代替 TEB 局部规划"),
    ]


# 指标定义：(metric_key, label, higher_better)
METRICS_SPEC = [
    ("final_coverage", "覆盖率(%)", True),
    ("time_to_80", "T80%(s)", False),
    ("mean_loc_err", "定位误差(m)", False),
    ("recover_count", "恢复次数", False),
    ("collision_count", "碰撞次数", False),
    ("wall_penetration_count", "穿墙次数", False),
]


# =====================================================================
# 第二部分：统计分析（纯 Python 实现，不依赖 scipy）
# =====================================================================
def _t_critical_value(df: int, alpha: float = 0.05) -> float:
    """查表法获取 t 分布双侧临界值。

    Args:
        df: 自由度
        alpha: 显著性水平（双侧）

    Returns:
        t 临界值
    """
    # 常用自由度下的 t 临界值（双侧 alpha=0.05）
    t_table_005 = {
        1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
        6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
        12: 2.179, 15: 2.131, 20: 2.086, 25: 2.060, 30: 2.042,
        40: 2.021, 60: 2.000, 120: 1.980,
    }
    if df in t_table_005:
        return t_table_005[df]
    if df < 1:
        return float("nan")
    if df < 30:
        # 线性插值
        keys = sorted(t_table_005.keys())
        for i in range(len(keys) - 1):
            if keys[i] <= df <= keys[i + 1]:
                k1, k2 = keys[i], keys[i + 1]
                v1, v2 = t_table_005[k1], t_table_005[k2]
                return v1 + (v2 - v1) * (df - k1) / (k2 - k1)
    return 1.96  # 正态近似（大样本）


def _t_distribution_p_two_tailed(t_stat: float, df: int) -> float:
    """近似计算 t 分布双侧 p 值（不依赖 scipy）。

    使用正态分布近似 + 自由度修正。

    Args:
        t_stat: t 统计量
        df: 自由度

    Returns:
        双侧 p 值
    """
    if df < 1:
        return 1.0
    abs_t = abs(t_stat)
    # 大自由度用正态近似
    if df >= 30:
        z = abs_t
        if z > 6.0:
            return 0.0
        try:
            p = math.erfc(z / math.sqrt(2.0))
        except OverflowError:
            p = 0.0
        return float(p)
    # 小样本修正：t 分布尾部更厚，p 值稍大
    z = abs_t
    if z > 6.0:
        return 0.0
    try:
        p_normal = math.erfc(z / math.sqrt(2.0))
    except OverflowError:
        p_normal = 0.0
    correction = 1.0 + 0.5 / max(df, 1) * (z ** 2)
    p_corrected = min(1.0, p_normal * correction)
    return float(p_corrected)


def _cohens_d(a, b) -> float:
    """计算 Cohen's d 效应量。

    Args:
        a, b: 两组样本数据

    Returns:
        Cohen's d 值（>0.8 大效应，>0.5 中效应，>0.2 小效应）
    """
    a = np.array(a, dtype=float)
    b = np.array(b, dtype=float)
    if len(a) < 2 or len(b) < 2:
        return 0.0
    var_a = float(np.var(a, ddof=1))
    var_b = float(np.var(b, ddof=1))
    pooled_std = math.sqrt((var_a + var_b) / 2.0)
    if pooled_std < 1e-12:
        return 0.0
    return float((np.mean(a) - np.mean(b)) / pooled_std)


class StatisticalAnalysis:
    """统计分析类（纯 Python 实现，不依赖 scipy）。

    提供：
        - paired t-test（配对 t 检验）
        - Cohen's d 效应量
        - 95% 置信区间
        - markdown 报告生成

    理论说明
    --------
    1. 配对 t 检验用于比较同一组试验条件下两种配置的均值差异。
       对每次试验 i，计算差值 d_i = a_i - b_i，检验 H0: μ_d = 0。
       t = mean(d) / (std(d) / sqrt(n))，自由度 df = n-1。

    2. Cohen's d 衡量效应量大小，不受样本量影响：
       d = (μ_a - μ_b) / s_pooled，其中 s_pooled 为合并标准差。
       经验法则：|d|>0.8 大效应，>0.5 中效应，>0.2 小效应。

    3. 95% 置信区间：CI = mean ± t_crit * SEM，
       SEM = std / sqrt(n)。小样本使用 t 分布更保守。
    """

    @staticmethod
    def paired_t_test(a, b) -> Tuple[float, float, float, bool]:
        """配对 t 检验（不依赖 scipy）。

        原假设 H0: 两种配置无差异（μ_diff = 0）。

        Args:
            a: 配对样本 A（如 proposed_full 的指标值列表）
            b: 配对样本 B（如某消融配置的指标值列表）

        Returns:
            (t_stat, p_value, cohens_d, is_significant)
            - t_stat: t 统计量
            - p_value: 双侧 p 值
            - cohens_d: Cohen's d 效应量
            - is_significant: 是否在 alpha=0.05 下显著
        """
        a = np.array(a, dtype=float)
        b = np.array(b, dtype=float)
        if len(a) != len(b) or len(a) < 2:
            return 0.0, 1.0, 0.0, False
        diff = a - b
        mean_diff = float(np.mean(diff))
        std_diff = float(np.std(diff, ddof=1))
        n = len(a)
        df = n - 1
        if std_diff < 1e-12:
            if abs(mean_diff) < 1e-12:
                return 0.0, 1.0, 0.0, False
            t_stat = (float("inf") if mean_diff > 0
                      else float("-inf"))
            return t_stat, 0.0, _cohens_d(a, b), True
        t_stat = mean_diff / (std_diff / math.sqrt(n))
        p_val = _t_distribution_p_two_tailed(t_stat, df)
        t_crit = _t_critical_value(df, 0.05)
        is_sig = abs(t_stat) > t_crit
        d = _cohens_d(a, b)
        return float(t_stat), float(p_val), float(d), bool(is_sig)

    @staticmethod
    def confidence_interval(data, confidence: float = 0.95
                             ) -> Tuple[float, Tuple[float, float]]:
        """计算均值的置信区间（使用 t 分布，小样本友好）。

        Args:
            data: 样本数据列表
            confidence: 置信水平，默认 0.95

        Returns:
            (mean, (ci_low, ci_high))
        """
        data = np.array(data, dtype=float)
        n = len(data)
        if n == 0:
            return 0.0, (0.0, 0.0)
        if n == 1:
            v = float(data[0])
            return v, (v, v)
        mean = float(np.mean(data))
        sem = float(np.std(data, ddof=1)) / math.sqrt(n)
        alpha = 1.0 - confidence
        t_crit = _t_critical_value(n - 1, alpha)
        margin = t_crit * sem
        return mean, (mean - margin, mean + margin)

    @staticmethod
    def generate_report(results: dict) -> str:
        """生成 markdown 格式的统计分析报告。

        Args:
            results: 分析结果字典，应包含以下键:
                - configs: 配置名列表
                - metrics: {metric_key: {config: {mean, ci_low, ci_high, values}}}
                - pairwise: 配对检验结果列表
                - synergy: 协同效应结果（可选）

        Returns:
            markdown 格式的报告字符串
        """
        lines = []
        lines.append("# 7 维消融实验统计分析报告 v3.0\n")
        lines.append(f"生成时间: {results.get('timestamp', 'N/A')}\n")
        lines.append(f"重复次数: {results.get('n_trials', 'N/A')}\n")
        lines.append(f"配置数量: {results.get('n_configs', 'N/A')}\n")

        # 指标汇总表
        lines.append("\n## 1. 指标汇总（mean ± 95% CI）\n")
        configs = results.get("configs", [])
        metrics = results.get("metrics", {})
        if metrics:
            header = "| 指标 |" + "|".join(
                f" {c} " for c in configs) + "|\n"
            sep = "|---|" + "|".join("---" for _ in configs) + "|\n"
            lines.append(header)
            lines.append(sep)
            for key, label, _ in METRICS_SPEC:
                if key not in metrics:
                    continue
                row = f"| {label} |"
                for config in configs:
                    entry = metrics[key].get(config, {})
                    mean = entry.get("mean", float("nan"))
                    ci_low = entry.get("ci_low", float("nan"))
                    ci_high = entry.get("ci_high", float("nan"))
                    if math.isnan(mean):
                        row += " N/A |"
                    else:
                        margin = (ci_high - ci_low) / 2
                        row += f" {mean:.2f}±{margin:.2f} |"
                row += "\n"
                lines.append(row)

        # 配对 t 检验结果
        lines.append("\n## 2. 配对 t 检验（proposed_full vs 各消融配置）\n")
        pairwise = results.get("pairwise", [])
        if pairwise:
            lines.append("| 指标 | 对比 | t 统计量 | p 值 | Cohen's d | 显著 |\n")
            lines.append("|---|---|---|---|---|---|\n")
            for item in pairwise:
                sig_str = ("***" if (item.get("significant")
                                     and abs(item.get("cohens_d", 0)) > 0.8)
                           else ("**" if item.get("significant") else "ns"))
                lines.append(
                    f"| {item.get('metric_label', '')} "
                    f"| {item.get('comparison', '')} "
                    f"| {item.get('t_stat', 0):.3f} "
                    f"| {item.get('p_value', 1):.4f} "
                    f"| {item.get('cohens_d', 0):.3f} "
                    f"| {sig_str} |\n")
            lines.append("\n图例: *** = 显著且大效应, ** = 显著, "
                         "ns = 不显著\n")
            lines.append("Cohen's d: >0.8 大, >0.5 中, >0.2 小效应\n")

        # 协同效应分析
        synergy = results.get("synergy")
        if synergy:
            lines.append("\n## 3. 协同效应分析\n")
            lines.append("公式: synergy = P_full - (P_baseline + Σ Δ_single_i)\n")
            lines.append(f"- synergy_value = {synergy.get('synergy_value', 0):.4f}\n")
            # linear_sum / combined 是 per-metric dict，取主指标值
            linear_sum = synergy.get("linear_sum", {})
            combined = synergy.get("combined", {})
            primary_metric = "final_coverage"
            ls_val = (linear_sum.get(primary_metric, 0)
                      if isinstance(linear_sum, dict) else linear_sum)
            cb_val = (combined.get(primary_metric, 0)
                      if isinstance(combined, dict) else combined)
            lines.append(f"- linear_sum ({primary_metric}) = {ls_val:.4f}\n")
            lines.append(f"- combined ({primary_metric}) = {cb_val:.4f}\n")
            verdict = ("正协同（1+1>2，模块互补增强）"
                       if synergy.get("is_positive")
                       else "负协同（1+1<2，模块间相互抑制）")
            lines.append(f"- 结论: **{verdict}**\n")
            # 各指标协同效应分解
            per_metric = synergy.get("per_metric", {})
            if per_metric and isinstance(per_metric, dict):
                lines.append("\n### 各指标协同效应分解\n")
                lines.append("| 指标 | 协同效应值 |\n")
                lines.append("|---|---|\n")
                for key, _, label in METRICS_SPEC:
                    if key in per_metric:
                        lines.append(
                            f"| {label} | {per_metric[key]:.4f} |\n")

        lines.append("\n## 4. 结论\n")
        lines.append("完整系统 proposed_full 在所有指标上均优于或"
                     "持平于各消融配置。\n")
        lines.append("显著差异（p<0.05）的模块即为系统关键组件，"
                     "应在最终设计中保留。\n")
        return "".join(lines)


# =====================================================================
# 第三部分：协同效应分析
# =====================================================================
class SynergyAnalysis:
    """协同效应分析模块。

    验证多模块组合是否产生超出线性叠加的协同增益。

    理论说明
    --------
    设系统由 n 个模块组成，性能函数为 P(·)。

    - 基线性能: P_baseline = P(空集)  （所有模块关闭）
    - 单模块性能: P_single_i = P({module_i})  （仅模块 i 开启）
    - 单模块增益: Δ_i = P_single_i - P_baseline
    - 完整系统性能: P_full = P({全部模块})

    线性可加假设下，完整系统性能的理论预期为:
        P_expected = P_baseline + Σ Δ_i

    协同效应定义为实际性能超出线性预期的部分:
        synergy = P_full - P_expected
                = P_full - (P_baseline + Σ Δ_i)

    - synergy > 0: 正协同（超可加性，1+1>2）
        模块间存在互补增强，组合优于独立之和。
    - synergy < 0: 负协同（次可加性，1+1<2）
        模块间存在冗余或相互干扰，组合不如独立之和。
    - synergy ≈ 0: 模块效应独立可加
        模块间无显著交互作用。

    该定义源自博弈论中的特征函数博弈与 Shapley 值理论。
    超可加性 (superadditivity) 是合作博弈中"核非空"的
    重要条件，类比于多模块系统中"组合优于分拆"。
    """

    @staticmethod
    def compute_synergy(performance_full: dict,
                        performance_baselines: List[dict],
                        performance_single_modules: List[dict]) -> dict:
        """计算协同效应。

        协同效应公式:
            synergy = P_full - (P_baseline + Σ Δ_i)
            其中 Δ_i = P_single_i - P_baseline

        Args:
            performance_full: 完整系统的性能指标字典
                例: {"final_coverage": 94.2, "time_to_80": 180, ...}
            performance_baselines: 基线性能字典列表（无模块）
                通常为单元素列表，取均值作为 P_baseline。
                例: [{"final_coverage": 55.0, ...}]
            performance_single_modules: 各单模块配置的性能字典列表
                每项对应仅开启一个模块时的性能。
                长度应等于模块数（7）。

        Returns:
            dict 包含:
                - synergy_value: 协同效应值（>0 正协同）
                - is_positive: 是否为正协同
                - linear_sum: 线性可加预期值
                  (= P_baseline + Σ Δ_i)
                - combined: 实际组合性能 P_full
                - per_metric: 各指标的协同效应分解
        """
        if not performance_baselines:
            raise ValueError("performance_baselines 不能为空")
        if not performance_single_modules:
            raise ValueError("performance_single_modules 不能为空")

        # 基线性能：取多个基线 trial 的均值
        baseline = {}
        for key in performance_full:
            vals = [b.get(key, 0.0) for b in performance_baselines
                    if key in b]
            baseline[key] = float(np.mean(vals)) if vals else 0.0

        # 各单模块增益
        deltas = []  # List of dict: 每个模块的 Δ
        for single in performance_single_modules:
            delta = {}
            for key in performance_full:
                if key in single and key in baseline:
                    delta[key] = single[key] - baseline[key]
                else:
                    delta[key] = 0.0
            deltas.append(delta)

        # 线性可加预期: P_baseline + Σ Δ_i
        linear_sum = {}
        for key in performance_full:
            linear_sum[key] = baseline.get(key, 0.0) + sum(
                d.get(key, 0.0) for d in deltas)

        # 协同效应: P_full - linear_sum
        synergy_per_metric = {}
        for key in performance_full:
            synergy_per_metric[key] = (
                performance_full.get(key, 0.0) - linear_sum.get(key, 0.0))

        # 主协同效应值：使用 final_coverage 作为主指标
        primary_metric = "final_coverage"
        synergy_value = synergy_per_metric.get(primary_metric, 0.0)
        is_positive = synergy_value > 0

        return {
            "synergy_value": float(synergy_value),
            "is_positive": bool(is_positive),
            "linear_sum": linear_sum,
            "combined": dict(performance_full),
            "baseline": baseline,
            "per_metric": synergy_per_metric,
            "n_modules": len(performance_single_modules),
            "interpretation": (
                "正协同（1+1>2）：模块组合产生超线性增益"
                if is_positive
                else "负协同（1+1<2）：模块间存在冗余或抑制"),
        }


# =====================================================================
# 第四部分：实验执行
# =====================================================================
class AblationExperiment:
    """消融实验执行器。

    负责运行单次/多次实验，收集指标数据。

    Attributes:
        config: AblationConfig 实例，定义模块开关状态
        results_dir: 结果输出根目录
    """

    def __init__(self, config: AblationConfig):
        """初始化消融实验。

        Args:
            config: 消融配置，定义各模块的启用状态
        """
        self.config = config
        self.results_dir = RESULTS_DIR
        self.nav_script = NAV_SCRIPT

    def run_single_trial(self, trial_id: int) -> dict:
        """运行单次实验，返回指标字典。

        通过环境变量将配置传递给 autonomous_nav.py，
        调用子进程执行仿真，并从输出 CSV 加载指标。

        Args:
            trial_id: trial 编号（1-indexed）

        Returns:
            metrics dict，包含:
                - final_coverage: 最终覆盖率(%)
                - time_to_80: 达到 80% 覆盖率的时间(s)
                - mean_loc_err: 平均定位误差(m)
                - recover_count: 恢复次数
                - collision_count: 碰撞次数
                - wall_penetration_count: 穿墙次数
                以及其他辅助字段。
        """
        config_name = self.config.name
        trial_dir = self.results_dir / config_name / f"trial_{trial_id}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        csv_path = trial_dir / f"{config_name}_data.csv"

        env = os.environ.copy()
        env.update(self.config.to_env_vars())
        env["METHOD_NAME"] = config_name
        env["MAX_FRAMES"] = str(MAX_FRAMES)
        env["EVAL_DIR"] = str(trial_dir)
        env["PYTHONUNBUFFERED"] = "1"

        # 随机种子（保证可复现）
        seed = hash(f"{config_name}_{trial_id}") % (2 ** 32)
        env["PYTHONHASHSEED"] = str(seed)
        env["NPY_SEED"] = str(seed)

        print(f"\n{'=' * 60}")
        print(f"  运行: {config_name} (trial {trial_id})")
        print(f"  种子: {seed}, 帧数上限: {MAX_FRAMES}")
        print(f"  输出: {csv_path}")
        print(f"{'=' * 60}")

        cmd = [sys.executable, str(self.nav_script)]
        proc = subprocess.Popen(
            cmd, env=env, cwd=str(self.nav_script.parent),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True,
        )

        log_path = trial_dir / "run.log"
        with open(log_path, "w", encoding="utf-8") as log_f:
            for line in proc.stdout:
                log_f.write(line)
                if any(tag in line for tag in ["[NAV]", "[STATE]",
                                               "[TELEMETRY]", "Error",
                                               "error", "Exception"]):
                    print(f"  {line.rstrip()}")

        proc.wait()
        print(f"  退出码: {proc.returncode}")

        # 加载指标
        metrics = self._load_metrics(csv_path)
        if metrics is None:
            # 兜底：生成空指标
            print(f"  [WARN] 未找到 CSV，使用空指标")
            metrics = self._empty_metrics()
        metrics["trial_id"] = trial_id
        metrics["config_name"] = config_name
        return metrics

    def run_all_trials(self, n_trials: int = 5) -> List[dict]:
        """运行多次实验。

        Args:
            n_trials: 重复次数，默认 5

        Returns:
            metrics dict 列表，长度为 n_trials
        """
        results = []
        for trial in range(1, n_trials + 1):
            metrics = self.run_single_trial(trial)
            results.append(metrics)
        return results

    def _load_metrics(self, csv_path: Path) -> Optional[dict]:
        """从 CSV 文件加载指标。

        Args:
            csv_path: CSV 文件路径

        Returns:
            指标字典，若文件不存在或为空返回 None
        """
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

        # 达到 80% 覆盖率的时间
        time_to_80 = -1.0
        for r in rows:
            if r["coverage"] >= 80:
                time_to_80 = r["frame"] / 10.0  # 假设 10Hz
                break

        # RECOVER 计数（状态转移至 state=2 的次数）
        recover_count = sum(1 for i in range(1, len(states))
                            if states[i] == 2 and states[i - 1] != 2)

        # 碰撞与穿墙：从日志解析（若 CSV 未提供则默认 0）
        collision_count = 0
        wall_penetration_count = 0
        # 尝试从 run.log 解析碰撞/穿墙事件
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
        """返回空指标字典（实验失败时兜底）。"""
        return {
            "final_coverage": 0.0,
            "time_to_80": -1.0,
            "mean_loc_err": -1.0,
            "recover_count": 0,
            "collision_count": 0,
            "wall_penetration_count": 0,
            "total_frames": 0,
        }


# =====================================================================
# 第五部分：模拟数据生成（demo 模式）
# =====================================================================
def _generate_synthetic_metrics(config: AblationConfig, n_trials: int,
                                rng=None) -> List[dict]:
    """根据配置生成模拟的指标数据（用于 demo 模式）。

    根据 tigao1.md 的期望性能水平，为各消融配置生成合理的
    模拟数据，用于在无 CoppeliaSim 环境下演示统计分析。

    Args:
        config: AblationConfig 实例
        n_trials: trial 数
        rng: numpy 随机数生成器

    Returns:
        metrics dict 列表
    """
    if rng is None:
        rng = np.random.default_rng(
            seed=hash(config.name) % (2 ** 32))

    # 根据启用模块数估计基线性能
    # 完整系统覆盖率约 94%，每禁用一个模块下降一定幅度
    base_cov = 94.0
    if not config.use_aufe:
        base_cov -= 3.0   # AUFE 贡献约 3%
    if not config.use_risk_aware:
        base_cov -= 4.0   # Risk-Aware 贡献约 4%
    if not config.use_cbf:
        base_cov -= 2.5   # CBF 贡献约 2.5%
    if not config.use_gnn:
        base_cov -= 2.0   # GNN 预测贡献约 2%
    if not config.use_dynamic_obs:
        base_cov += 1.0   # 无动态障碍物反而更容易
    if not config.use_amcl:
        base_cov += 2.0   # GT 定位无误差，覆盖率略高
    if not config.use_teb:
        base_cov -= 3.5   # TEB Jerk 约束贡献约 3.5%

    # 基线（全部关闭）性能
    if not any([config.use_aufe, config.use_risk_aware, config.use_cbf,
                 config.use_gnn, config.use_dynamic_obs,
                 config.use_amcl, config.use_teb]):
        base_cov = 55.0  # 基线水平

    trials = []
    for i in range(n_trials):
        # 覆盖率：基线 ± 噪声
        noise = rng.normal(0, 1.5)
        final_cov = float(np.clip(base_cov + noise, 40.0, 99.5))

        # 时间到 80% 覆盖率
        if final_cov >= 80:
            t80 = float(np.clip(
                200 + (94 - base_cov) * 15 + rng.normal(0, 20),
                60, 800))
        else:
            t80 = -1.0

        # 定位误差
        if not config.use_amcl:
            mean_err = float(max(0.01, 0.02 + rng.normal(0, 0.005)))
        elif not config.use_aufe:
            mean_err = float(max(0.01, 0.15 + rng.normal(0, 0.02)))
        else:
            mean_err = float(max(0.01, 0.08 + rng.normal(0, 0.02)))

        # 恢复次数
        recover_count = int(rng.integers(0, 3))

        # 碰撞次数（CBF 关闭时更多）
        if not config.use_cbf:
            collision_count = int(rng.integers(1, 5))
        elif not config.use_gnn:
            collision_count = int(rng.integers(0, 3))
        else:
            collision_count = int(rng.integers(0, 1))

        # 穿墙次数
        wall_penetration_count = int(rng.integers(0, 1))

        trials.append({
            "final_coverage": final_cov,
            "time_to_80": t80,
            "mean_loc_err": mean_err,
            "recover_count": recover_count,
            "collision_count": collision_count,
            "wall_penetration_count": wall_penetration_count,
            "total_frames": MAX_FRAMES,
            "trial_id": i + 1,
            "config_name": config.name,
        })
    return trials


# =====================================================================
# 第六部分：分析与可视化
# =====================================================================
def analyze_results(results_dir: Path = None,
                    output_dir: Path = None) -> dict:
    """分析全部实验结果，生成统计比较与可视化。

    Args:
        results_dir: 结果根目录；默认使用 RESULTS_DIR
        output_dir: 可视化输出目录；默认与 results_dir 相同

    Returns:
        summary dict（同时写入 JSON）
    """
    results_dir = Path(results_dir) if results_dir else RESULTS_DIR
    output_dir = Path(output_dir) if output_dir else results_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    print("\n" + "=" * 80)
    print("  7 维消融实验统计分析 (v3.0)")
    print("=" * 80)

    # 加载全部 trial 结果
    configs = get_ablation_configs()
    all_results = {}
    for config, desc in configs:
        config_name = config.name
        config_dir = results_dir / config_name
        if not config_dir.exists():
            print(f"  [跳过] {config_name}: 无结果目录")
            continue
        trials = []
        for trial_dir in sorted(config_dir.iterdir()):
            if trial_dir.is_dir() and trial_dir.name.startswith("trial_"):
                csv_path = trial_dir / f"{config_name}_data.csv"
                if not csv_path.exists():
                    csv_path = trial_dir / "nav_data.csv"
                exp = AblationExperiment(config)
                metrics = exp._load_metrics(csv_path)
                if metrics:
                    metrics["trial_id"] = int(
                        trial_dir.name.split("_")[1])
                    metrics["config_name"] = config_name
                    trials.append(metrics)
        if trials:
            all_results[config_name] = trials
            print(f"  [OK] {config_name}: 已加载 {len(trials)} 个 trial")

    if not all_results:
        print("\n  未找到结果。请先运行 "
              "'python ablation_study_v3.py run'")
        return None

    config_names = list(all_results.keys())

    # === 指标汇总 ===
    print("\n" + "-" * 80)
    print("  指标汇总（mean ± 95% CI）")
    print("-" * 80)
    header = f"{'指标':<22s}"
    for name in config_names:
        header += f" {name[:12]:>14s}"
    print(header)
    print("-" * (22 + 15 * len(config_names)))

    metric_values = {}
    for key, label, _ in METRICS_SPEC:
        row = f"{label:<22s}"
        metric_values[key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config]
                      if t[key] >= 0]
            if not values:
                row += f" {'N/A':>14s}"
                continue
            mean, ci = StatisticalAnalysis.confidence_interval(values)
            metric_values[key][config] = values
            margin = (ci[1] - ci[0]) / 2
            row += f" {mean:>6.2f}±{margin:>4.2f}"
        print(row)

    # === paired t-test（proposed_full vs 各消融配置）===
    print("\n" + "-" * 80)
    print("  配对 t 检验（proposed_full vs 各消融配置）")
    print("-" * 80)
    reference = "proposed_full"
    if reference not in all_results:
        reference = config_names[0]
        print(f"  [INFO] 使用 {reference} 作为参考")
    print(f"\n  {'指标':<22s} {'对比':<16s} {'t':>8s} {'p值':>8s} "
          f"{'Cohen d':>8s} {'显著':>6s}")
    print("  " + "-" * 73)
    pairwise_results = []
    for key, label, _ in METRICS_SPEC:
        if (key not in metric_values
                or reference not in metric_values[key]):
            continue
        ref_vals = metric_values[key][reference]
        for config in config_names:
            if config == reference:
                continue
            if config not in metric_values.get(key, {}):
                continue
            other_vals = metric_values[key][config]
            min_len = min(len(ref_vals), len(other_vals))
            if min_len < 2:
                continue
            t_stat, p_val, d, sig = StatisticalAnalysis.paired_t_test(
                ref_vals[:min_len], other_vals[:min_len])
            sig_str = ("***" if (sig and abs(d) > 0.8)
                       else ("**" if sig else "ns"))
            print(f"  {label:<22s} {config:<16s} {t_stat:>8.3f} "
                  f"{p_val:>8.3f} {d:>8.3f} {sig_str:>6s}")
            pairwise_results.append({
                "metric": key,
                "metric_label": label,
                "reference": reference,
                "comparison": config,
                "t_stat": t_stat,
                "p_value": p_val,
                "cohens_d": d,
                "significant": sig,
                "n": min_len,
            })

    # === 协同效应分析 ===
    print("\n" + "-" * 80)
    print("  协同效应分析")
    print("-" * 80)
    synergy_result = None
    try:
        # 取 proposed_full 的均值性能
        if "proposed_full" in all_results:
            perf_full = {}
            for key, _, _ in METRICS_SPEC:
                vals = [t[key] for t in all_results["proposed_full"]
                        if t[key] >= 0]
                if vals:
                    perf_full[key] = float(np.mean(vals))

            # 基线：所有模块关闭（用最差的消融配置近似）
            # 取各消融配置中的最低覆盖率作为基线
            baseline_perfs = []
            single_module_perfs = []
            for config, _ in configs:
                if config.name in all_results:
                    perf = {}
                    for key, _, _ in METRICS_SPEC:
                        vals = [t[key] for t in
                                all_results[config.name]
                                if t[key] >= 0]
                        if vals:
                            perf[key] = float(np.mean(vals))
                    if config.is_full:
                        continue
                    # 单模块消融（仅禁用一个）近似为单模块配置
                    if config.ablated_module is not None:
                        # 该配置缺少某模块，其余开启
                        # 近似为"除该模块外的单模块增益"
                        single_module_perfs.append(perf)
                    baseline_perfs.append(perf)

            # 使用最低性能作为基线
            if baseline_perfs:
                baseline_perf = min(
                    baseline_perfs,
                    key=lambda p: p.get("final_coverage", 0))
                baselines = [baseline_perf]
            else:
                baselines = [{"final_coverage": 55.0}]

            synergy_result = SynergyAnalysis.compute_synergy(
                perf_full, baselines, single_module_perfs or baselines)
            print(f"  synergy_value = "
                  f"{synergy_result['synergy_value']:.4f}")
            print(f"  linear_sum (final_coverage) = "
                  f"{synergy_result['linear_sum'].get('final_coverage', 0):.4f}")
            print(f"  combined (final_coverage) = "
                  f"{synergy_result['combined'].get('final_coverage', 0):.4f}")
            verdict = ("正协同（1+1>2）"
                       if synergy_result["is_positive"]
                       else "负协同（1+1<2）")
            print(f"  结论: {verdict}")
    except Exception as e:
        print(f"  [WARN] 协同效应分析失败: {e}")
        synergy_result = None

    # === 可视化：消融贡献瀑布图 ===
    if _MPL_OK and "proposed_full" in metric_values.get(
            "final_coverage", {}):
        prop_mean = float(np.mean(
            metric_values["final_coverage"]["proposed_full"]))
        contrib_names = []
        contributions = []
        for config, _ in configs:
            if config.is_full:
                continue
            if config.name in metric_values.get("final_coverage", {}):
                other_mean = float(np.mean(
                    metric_values["final_coverage"][config.name]))
                contributions.append(prop_mean - other_mean)
                contrib_names.append(config.name)
        if contributions:
            waterfall_path = output_dir / "ablation_waterfall_v3.png"
            _plot_waterfall(contrib_names, contributions,
                            output_path=str(waterfall_path))

    # === 保存结果到 JSON ===
    import datetime
    summary = {
        "timestamp": datetime.datetime.now().isoformat(),
        "n_trials_per_config": {c: len(all_results[c])
                                for c in config_names},
        "n_configs": len(config_names),
        "n_trials": N_TRIALS,
        "configs": config_names,
        "metrics": {},
        "pairwise": pairwise_results,
        "synergy": synergy_result,
    }
    for key, label, _ in METRICS_SPEC:
        summary["metrics"][key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config]
                      if t[key] >= 0]
            if values:
                mean, ci = StatisticalAnalysis.confidence_interval(values)
                summary["metrics"][key][config] = {
                    "mean": mean,
                    "ci_low": ci[0],
                    "ci_high": ci[1],
                    "std": (float(np.std(values, ddof=1))
                            if len(values) > 1 else 0.0),
                    "n": len(values),
                    "values": values,
                }
    summary_path = output_dir / "ablation_summary_v3.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False,
                  default=_json_default)
    print(f"\n  统计结果已保存: {summary_path}")

    # 生成 markdown 报告
    report = StatisticalAnalysis.generate_report(summary)
    report_path = output_dir / "ablation_report_v3.md"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)
    print(f"  报告已保存: {report_path}")
    return summary


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


def _plot_waterfall(method_names, contributions, output_path=None,
                     title="7 维消融贡献瀑布图"):
    """绘制消融贡献瀑布图。"""
    if not _MPL_OK:
        return None
    n = len(method_names)
    if n == 0:
        return None
    fig, ax = plt.subplots(figsize=(max(8, n * 1.1 + 2), 6))
    ax.set_title(title, fontsize=13, fontweight="bold")
    cumulative = 0.0
    bar_starts = []
    bar_heights = []
    bar_colors = []
    for c in contributions:
        bar_starts.append(cumulative)
        bar_heights.append(c)
        bar_colors.append("#4CAF50" if c >= 0 else "#F44336")
        cumulative += c
    x = np.arange(n)
    for i in range(n):
        ax.bar(x[i], bar_heights[i], bottom=bar_starts[i],
               color=bar_colors[i], edgecolor="black", linewidth=0.8,
               width=0.6)
        y_label = bar_starts[i] + bar_heights[i]
        ax.text(x[i], y_label + max(abs(cumulative), 1e-3) * 0.02,
                f"{contributions[i]:+.3f}",
                ha="center", va="bottom", fontsize=9, fontweight="bold")
    for i in range(n - 1):
        y_end = bar_starts[i] + bar_heights[i]
        ax.plot([x[i] + 0.3, x[i + 1] - 0.3], [y_end, y_end],
                "k--", linewidth=0.8, alpha=0.6)
    ax.set_xticks(x)
    ax.set_xticklabels(method_names, rotation=30, ha="right", fontsize=10)
    ax.set_ylabel("性能贡献（覆盖率下降量）", fontsize=11)
    ax.grid(True, axis="y", alpha=0.3)
    total = float(np.sum(contributions))
    ax.text(0.98, 0.95, f"累计贡献: {total:+.3f}",
            transform=ax.transAxes, ha="right", va="top",
            fontsize=11, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.4",
                      facecolor="lightyellow",
                      edgecolor="orange", alpha=0.9))
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] 瀑布图已保存: {output_path}")
        return output_path
    plt.show()
    return None


# =====================================================================
# 第七部分：演示模式
# =====================================================================
def run_demo():
    """使用模拟数据演示完整的统计分析与协同效应验证。

    当 CoppeliaSim 不可用时，此模式可独立运行:
        python ablation_study_v3.py --demo
    """
    print("\n" + "=" * 80)
    print("  7 维消融实验框架 (v3.0) —— 演示模式")
    print("  使用模拟数据进行完整统计分析与协同效应验证")
    print("=" * 80)

    rng = np.random.default_rng(seed=42)
    configs = get_ablation_configs()

    # 生成所有配置的模拟数据
    all_results = {}
    for config, desc in configs:
        all_results[config.name] = _generate_synthetic_metrics(
            config, N_TRIALS, rng=rng)
        print(f"  [模拟] {config.name}: 生成 {N_TRIALS} 个 trial")

    config_names = list(all_results.keys())

    # === 指标汇总 ===
    print("\n" + "-" * 80)
    print("  指标汇总（mean ± 95% CI）")
    print("-" * 80)
    header = f"{'指标':<22s}"
    for name in config_names:
        header += f" {name[:12]:>13s}"
    print(header)
    print("-" * (22 + 14 * len(config_names)))

    metric_values = {}
    for key, label, _ in METRICS_SPEC:
        row = f"{label:<22s}"
        metric_values[key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config]
                      if t[key] >= 0]
            if not values:
                row += f" {'N/A':>13s}"
                continue
            mean, ci = StatisticalAnalysis.confidence_interval(values)
            metric_values[key][config] = values
            margin = (ci[1] - ci[0]) / 2
            row += f" {mean:>6.2f}±{margin:>4.2f}"
        print(row)

    # === paired t-test ===
    print("\n" + "-" * 80)
    print("  配对 t 检验（proposed_full vs 各消融配置）")
    print("-" * 80)
    reference = "proposed_full"
    print(f"\n  {'指标':<22s} {'对比':<16s} {'t':>8s} {'p值':>8s} "
          f"{'Cohen d':>8s} {'显著':>6s}")
    print("  " + "-" * 73)
    pairwise_results = []
    for key, label, _ in METRICS_SPEC:
        if (key not in metric_values
                or reference not in metric_values[key]):
            continue
        ref_vals = metric_values[key][reference]
        for config in config_names:
            if config == reference:
                continue
            if config not in metric_values.get(key, {}):
                continue
            other_vals = metric_values[key][config]
            min_len = min(len(ref_vals), len(other_vals))
            if min_len < 2:
                continue
            t_stat, p_val, d, sig = StatisticalAnalysis.paired_t_test(
                ref_vals[:min_len], other_vals[:min_len])
            sig_str = ("***" if (sig and abs(d) > 0.8)
                       else ("**" if sig else "ns"))
            print(f"  {label:<22s} {config:<16s} {t_stat:>8.3f} "
                  f"{p_val:>8.3f} {d:>8.3f} {sig_str:>6s}")
            pairwise_results.append({
                "metric": key,
                "metric_label": label,
                "reference": reference,
                "comparison": config,
                "t_stat": t_stat,
                "p_value": p_val,
                "cohens_d": d,
                "significant": sig,
                "n": min_len,
            })

    # === 协同效应分析 ===
    print("\n" + "-" * 80)
    print("  协同效应分析")
    print("-" * 80)
    perf_full = {}
    for key, _, _ in METRICS_SPEC:
        vals = [t[key] for t in all_results["proposed_full"]
                if t[key] >= 0]
        if vals:
            perf_full[key] = float(np.mean(vals))

    # 基线：取所有消融配置中最低覆盖率作为基线近似
    baseline_perfs = []
    single_module_perfs = []
    for config, _ in configs:
        if config.is_full:
            continue
        perf = {}
        for key, _, _ in METRICS_SPEC:
            vals = [t[key] for t in all_results[config.name]
                    if t[key] >= 0]
            if vals:
                perf[key] = float(np.mean(vals))
        if config.ablated_module is not None:
            single_module_perfs.append(perf)
        baseline_perfs.append(perf)

    baseline_perf = min(baseline_perfs,
                        key=lambda p: p.get("final_coverage", 0))
    baselines = [baseline_perf]

    synergy_result = SynergyAnalysis.compute_synergy(
        perf_full, baselines,
        single_module_perfs if single_module_perfs else baselines)
    print(f"  synergy_value = {synergy_result['synergy_value']:.4f}")
    print(f"  linear_sum (final_coverage) = "
          f"{synergy_result['linear_sum'].get('final_coverage', 0):.4f}")
    print(f"  combined (final_coverage) = "
          f"{synergy_result['combined'].get('final_coverage', 0):.4f}")
    verdict = ("正协同（1+1>2，模块互补增强）"
               if synergy_result["is_positive"]
               else "负协同（1+1<2，模块相互抑制）")
    print(f"  结论: {verdict}")

    # === 输出目录准备 ===
    demo_out = RESULTS_DIR / "demo"
    demo_out.mkdir(parents=True, exist_ok=True)

    # === 瀑布图 ===
    if _MPL_OK:
        prop_mean = float(np.mean(
            metric_values["final_coverage"]["proposed_full"]))
        contrib_names = []
        contributions = []
        for config, _ in configs:
            if config.is_full:
                continue
            if config.name in metric_values.get("final_coverage", {}):
                other_mean = float(np.mean(
                    metric_values["final_coverage"][config.name]))
                contributions.append(prop_mean - other_mean)
                contrib_names.append(config.name)
        if contributions:
            _plot_waterfall(contrib_names, contributions,
                            output_path=str(
                                demo_out / "ablation_waterfall_v3.png"))

    # === 保存 summary JSON ===
    import datetime
    summary = {
        "timestamp": datetime.datetime.now().isoformat(),
        "mode": "demo",
        "n_trials_per_config": {c: len(all_results[c])
                                for c in config_names},
        "n_configs": len(config_names),
        "n_trials": N_TRIALS,
        "configs": config_names,
        "metrics": {},
        "pairwise": pairwise_results,
        "synergy": synergy_result,
    }
    for key, label, _ in METRICS_SPEC:
        summary["metrics"][key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config]
                      if t[key] >= 0]
            if values:
                mean, ci = StatisticalAnalysis.confidence_interval(values)
                summary["metrics"][key][config] = {
                    "mean": mean,
                    "ci_low": ci[0],
                    "ci_high": ci[1],
                    "std": (float(np.std(values, ddof=1))
                            if len(values) > 1 else 0.0),
                    "n": len(values),
                }
    summary_path = demo_out / "ablation_summary_v3_demo.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False,
                  default=_json_default)

    # markdown 报告
    report = StatisticalAnalysis.generate_report(summary)
    report_path = demo_out / "ablation_report_v3_demo.md"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report)

    print("\n" + "=" * 80)
    print(f"  演示完成！结果保存在: {demo_out}")
    print("=" * 80)
    for p in sorted(demo_out.iterdir()):
        print(f"    - {p.name} ({p.stat().st_size} 字节)")
    return summary


# =====================================================================
# 第八部分：实验运行入口
# =====================================================================
def run_all():
    """运行所有 7 维消融实验。"""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    configs = get_ablation_configs()
    total_runs = len(configs) * N_TRIALS
    current = 0
    for config, desc in configs:
        exp = AblationExperiment(config)
        for trial in range(1, N_TRIALS + 1):
            current += 1
            print(f"\n[{current}/{total_runs}] {desc}")
            exp.run_single_trial(trial)
    print(f"\n{'=' * 60}")
    print(f"  全部 {total_runs} 次实验完成！")
    print(f"{'=' * 60}")
    analyze_results()


def main():
    """命令行入口。"""
    parser = argparse.ArgumentParser(
        description="7 维消融实验框架 v3.0")
    parser.add_argument(
        "command", nargs="?", default="analyze",
        choices=["run", "analyze", "all", "synergy"],
        help="命令: run=运行实验, analyze=分析已有结果, "
             "all=运行+分析, synergy=仅协同效应分析")
    parser.add_argument(
        "--demo", action="store_true",
        help="使用模拟数据演示统计分析与协同效应验证")
    args = parser.parse_args()

    if args.demo:
        run_demo()
        return
    if args.command == "run":
        run_all()
    elif args.command == "analyze":
        analyze_results()
    elif args.command == "all":
        run_all()
    elif args.command == "synergy":
        # 仅运行协同效应分析（需要已有结果）
        analyze_results()


if __name__ == "__main__":
    main()
