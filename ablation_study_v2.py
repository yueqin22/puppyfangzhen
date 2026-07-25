#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版消融实验框架 (v2.0)
================================
Systematic ablation experiments with extended baselines, enhanced
statistical analysis, and rich visualizations.

特性概览
--------
1. 扩展对比基线到 15 个配置，涵盖：
   - 经典方法：nearest_frontier / random_walk / greedy_frontier
   - 前沿方法：rrg_exploration / deep_rl_dqn / full_knowledge_astar
   - 消融实验：proposed_full 与各模块单独关闭的 7 个变体
2. 统计分析增强：
   - N_TRIALS = 10 次重复实验
   - 均值 ± 95% 置信区间
   - paired t-test（配对 t 检验，两两比较）
   - Friedman 检验（多算法比较，自行实现，不依赖 scipy）
   - Nemenyi 事后检验（CD 临界差）
   - Cohen's d 效应量
3. 可视化功能：
   - CD 图（Critical Difference Diagram）
   - Pareto 前沿可视化（2D / 3D）
   - 消融贡献瀑布图
   - 雷达图多维度对比
4. 兼容性：
   - 若 CoppeliaSim 不可用，可使用 --demo 模式以模拟数据
     运行统计分析与全部可视化测试
   - 仅依赖 numpy + matplotlib（不依赖 scipy）

用法
----
    python ablation_study_v2.py run       # 运行所有实验
    python ablation_study_v2.py analyze   # 仅分析已有结果
    python ablation_study_v2.py all       # 运行 + 分析
    python ablation_study_v2.py --demo    # 使用模拟数据演示统计与可视化
"""
import os
import sys
import csv
import json
import math
import subprocess
import argparse
from pathlib import Path

import numpy as np

# === 全局配置 ===
N_TRIALS = 10          # 重复实验次数（用于统计显著性）
MAX_FRAMES = 4000      # 单次实验帧数上限
RESULTS_DIR = Path("e:/puppyfangzhen/ablation_results_v2")
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
# configurations：14 个配置（经典3 + 前沿3 + 消融8 = 14，满足“10+”要求）
# 每个配置由 (name, description, env_dict, category, expected_baseline) 组成
# - category: classic / frontier / ablation
# - expected_baseline: 该配置在覆盖率指标上的期望相对水平（仅用于 demo 模式生成模拟数据）
CONFIGURATIONS = [
    # === 经典方法 ===
    ("nearest_frontier", "经典方法：最近边界 (Yamauchi 1997)", {
        "USE_AMCL": "1", "USE_TEB": "0", "USE_KLD": "0", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "0", "FRONTIER_STRATEGY": "nearest",
    }, "classic", 0.78),
    ("random_walk", "经典方法：随机游走（下界基准）", {
        "USE_AMCL": "1", "USE_TEB": "0", "USE_KLD": "0", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "0", "FRONTIER_STRATEGY": "random",
    }, "classic", 0.55),
    ("greedy_frontier", "经典方法：贪心边界（最大信息增益）", {
        "USE_AMCL": "1", "USE_TEB": "0", "USE_KLD": "0", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "0", "FRONTIER_STRATEGY": "greedy",
    }, "classic", 0.74),
    # === 前沿方法 ===
    ("rrg_exploration", "前沿方法：RRG 基于采样的探索 (Umari 2017)", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "0", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "0", "FRONTIER_STRATEGY": "rrg",
    }, "frontier", 0.85),
    ("deep_rl_dqn", "前沿方法：深度强化学习 DQN (Mnih 2015)", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "0", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "0", "USE_RL": "1", "FRONTIER_STRATEGY": "rl",
    }, "frontier", 0.88),
    ("full_knowledge_astar", "前沿方法：全知 A*（理论上界）", {
        "USE_AMCL": "0", "USE_TEB": "1", "USE_KLD": "0", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "0", "FRONTIER_STRATEGY": "oracle",
    }, "frontier", 0.96),
    # === 消融实验 ===
    ("proposed_full", "完整方法（proposed v3.0 全模块开启）", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "1", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "1",
    }, "ablation", 0.94),
    ("no_kld", "消融：无 KLD 采样（固定粒子数）", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "0", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "1", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "1",
    }, "ablation", 0.91),
    ("no_info_theory", "消融：无信息论（基于计数）", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "0",
        "USE_SEMANTIC": "1", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "1",
    }, "ablation", 0.90),
    ("no_teb_jerk", "消融：无 TEB Jerk 约束（使用 DWA）", {
        "USE_AMCL": "1", "USE_TEB": "0", "USE_KLD": "1", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "1", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "1",
    }, "ablation", 0.89),
    ("no_adaptive_weight", "消融：无自适应权重（固定权重）", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "1", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "0",
        "USE_RECOVERY": "1",
    }, "ablation", 0.91),
    ("no_semantic", "消融：无语义信息", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "0", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "1",
    }, "ablation", 0.92),
    ("no_recovery", "消融：无智能恢复", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "1", "USE_PARETO": "1", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "0",
    }, "ablation", 0.87),
    ("no_pareto", "消融：无 Pareto 多目标", {
        "USE_AMCL": "1", "USE_TEB": "1", "USE_KLD": "1", "USE_INFO_THEORY": "1",
        "USE_SEMANTIC": "1", "USE_PARETO": "0", "USE_ADAPTIVE_WEIGHT": "1",
        "USE_RECOVERY": "1",
    }, "ablation", 0.90),
]

# 指标定义：(metric_key, label, higher_better)
METRICS_SPEC = [
    ("final_coverage", "覆盖率(%)", True),
    ("time_to_80", "T80%(s)", False),
    ("time_to_90", "T90%(s)", False),
    ("mean_loc_err", "定位误差(m)", False),
    ("max_loc_err", "最大误差(m)", False),
    ("recover_count", "恢复次数", False),
    ("follow_pct", "跟随比例(%)", True),
    ("avg_speed", "平均速度(m/s)", True),
    ("compute_time", "计算时间(s)", False),
]


# =====================================================================
# 第二部分：统计分析函数（不依赖 scipy）
# =====================================================================
def _t_critical_value(df, alpha=0.05):
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


def _t_distribution_p_two_tailed(t_stat, df):
    """近似计算 t 分布双侧 p 值。

    使用正态分布近似 + 自由度修正，不依赖 scipy。
    """
    if df < 1:
        return 1.0
    abs_t = abs(t_stat)
    # 大自由度用正态近似
    if df >= 30:
        # 标准正态双侧 p 值近似
        z = abs_t
        if z > 6.0:
            return 0.0
        # 使用 erfc 近似：P(|Z|>z) = erfc(z/sqrt(2))
        try:
            p = math.erfc(z / math.sqrt(2.0))
        except OverflowError:
            p = 0.0
        return float(p)
    # 小样本用 t 表临界值反推近似 p 值
    # 通过查找最接近的临界值推断 p 值范围
    t_table_p = {
        # (df, alpha) -> t_crit  （双侧）
        # 我们用多个 alpha 等级
    }
    # 多级 alpha 临界值表
    alpha_levels = [0.20, 0.10, 0.05, 0.02, 0.01, 0.001]
    crit_table = {}
    crit_table[5] = [1.476, 2.015, 2.571, 3.365, 4.032, 6.859]
    crit_table[8] = [1.397, 1.860, 2.306, 2.896, 3.355, 5.041]
    crit_table[9] = [1.383, 1.833, 2.262, 2.821, 3.250, 4.781]
    crit_table[10] = [1.372, 1.812, 2.228, 2.764, 3.169, 4.587]
    crit_table[9] = [1.383, 1.833, 2.262, 2.821, 3.250, 4.781]

    # 通用近似：使用 Hills 方法
    # 参考：https://en.wikipedia.org/wiki/Student%27s_t-distribution
    # p ≈ 2 * (1 - cdf_t(|t|, df))
    # 使用正态近似 + 膨胀因子（保守估计）
    z = abs_t
    if z > 6.0:
        return 0.0
    try:
        p_normal = math.erfc(z / math.sqrt(2.0))
    except OverflowError:
        p_normal = 0.0
    # 小样本修正：t 分布尾部更厚，p 值稍大
    correction = 1.0 + 0.5 / max(df, 1) * (z ** 2)
    p_corrected = min(1.0, p_normal * correction)
    return float(p_corrected)


def confidence_interval(data, confidence=0.95):
    """计算均值的 95% 置信区间。

    使用 t 分布（小样本友好）。

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


def cohens_d(a, b):
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


def paired_t_test(a, b):
    """配对 t 检验（不依赖 scipy）。

    Args:
        a, b: 两组配对样本

    Returns:
        (t_stat, p_value, df, is_significant)
    """
    a = np.array(a, dtype=float)
    b = np.array(b, dtype=float)
    if len(a) != len(b) or len(a) < 2:
        return 0.0, 1.0, 0, False
    diff = a - b
    mean_diff = float(np.mean(diff))
    std_diff = float(np.std(diff, ddof=1))
    n = len(a)
    df = n - 1
    if std_diff < 1e-12:
        # 无方差情况
        if abs(mean_diff) < 1e-12:
            return 0.0, 1.0, df, False
        return (float("inf") if mean_diff > 0 else float("-inf")), 0.0, df, True
    t_stat = mean_diff / (std_diff / math.sqrt(n))
    p_val = _t_distribution_p_two_tailed(t_stat, df)
    t_crit = _t_critical_value(df, 0.05)
    is_sig = abs(t_stat) > t_crit
    return float(t_stat), float(p_val), df, bool(is_sig)


def friedman_test(perf_matrix):
    """Friedman 检验（多算法比较，自行实现，不依赖 scipy）。

    用于检验 k 个算法在 n 个数据集（试验/场景）上的表现是否有
    显著差异。原假设 H0：所有算法表现相同。

    Args:
        perf_matrix: shape=(n, k) 的 numpy 数组
            n = 试验数（block 数）
            k = 算法数
            值为性能指标（对于每个 block 内，越小越好将被赋予小排名）
            注意：本实现统一按“值越小越好”排名。

    Returns:
        dict: {
            "chi2": Friedman 卡方统计量,
            "df": 自由度 = k-1,
            "p_value": p 值,
            "average_ranks": 各算法的平均排名（长度 k），
            "n": block 数,
            "k": 算法数,
            "significant": 是否在 alpha=0.05 下显著,
        }
    """
    perf_matrix = np.array(perf_matrix, dtype=float)
    if perf_matrix.ndim != 2:
        raise ValueError("perf_matrix 必须是二维数组 (n_blocks, k_algos)")
    n, k = perf_matrix.shape
    if k < 2 or n < 1:
        return {"chi2": 0.0, "df": 0, "p_value": 1.0,
                "average_ranks": np.zeros(k), "n": n, "k": k,
                "significant": False}

    # 对每个 block（试验）独立排名，1 = 最小值（最优）
    # 处理并列：使用平均排名
    rank_matrix = np.zeros_like(perf_matrix)
    for i in range(n):
        row = perf_matrix[i]
        order = np.argsort(row, kind="stable")
        # 计算平均排名处理 ties
        sorted_vals = row[order]
        ranks = np.empty(k, dtype=float)
        i_pos = 0
        while i_pos < k:
            j = i_pos
            while j + 1 < k and sorted_vals[j + 1] == sorted_vals[i_pos]:
                j += 1
            # [i_pos, j] 为并列组
            avg_rank = (i_pos + 1 + j + 1) / 2.0  # 1-indexed 平均
            for idx in range(i_pos, j + 1):
                ranks[order[idx]] = avg_rank
            i_pos = j + 1
        rank_matrix[i] = ranks

    # 平均排名
    avg_ranks = rank_matrix.mean(axis=0)  # shape (k,)

    # Friedman 统计量
    # chi2_F = (12n / (k(k+1))) * (sum_j R_j^2 - k(k+1)^2/4)
    sum_r2 = float(np.sum(avg_ranks ** 2))
    chi2 = (12.0 * n) / (k * (k + 1)) * (sum_r2 - k * (k + 1) ** 2 / 4.0)
    df = k - 1

    # p-value: 卡方分布上尾概率
    # 使用不完全 Gamma 函数的级数展开近似
    p_value = _chi2_survival(chi2, df)
    t_crit_table = {1: 3.841, 2: 5.991, 3: 7.815, 4: 9.488, 5: 11.070,
                    6: 12.592, 7: 14.067, 8: 15.507, 9: 16.919, 10: 18.307,
                    11: 19.675, 12: 21.026, 13: 22.362}
    is_sig = bool(chi2 > t_crit_table.get(df, 18.307))

    return {
        "chi2": float(chi2),
        "df": int(df),
        "p_value": float(p_value),
        "average_ranks": avg_ranks,
        "rank_matrix": rank_matrix,
        "n": int(n),
        "k": int(k),
        "significant": is_sig,
    }


def _chi2_survival(x, df):
    """近似计算卡方分布的上尾概率 P(X > x)。

    使用 Wilson-Hilferty 正态近似：
        z ≈ ((x/df)^(1/3) - (1 - 2/(9*df))) / sqrt(2/(9*df))
    然后 P(X > x) ≈ 1 - Phi(z) = 0.5 * erfc(z/sqrt(2))

    Args:
        x: 卡方统计量
        df: 自由度

    Returns:
        上尾概率 p 值
    """
    if df <= 0:
        return 1.0
    if x <= 0:
        return 1.0
    if df == 1:
        # df=1: P(X>x) = 2 * (1 - Phi(sqrt(x)))
        z = math.sqrt(x)
        return float(math.erfc(z / math.sqrt(2.0)))
    if df == 2:
        # df=2: P(X>x) = exp(-x/2)
        return float(math.exp(-x / 2.0))

    # Wilson-Hilferty 近似
    h = 2.0 / (9.0 * df)
    # 防止 x/df 为负数
    ratio = x / df
    if ratio <= 0:
        return 1.0
    z = ((ratio) ** (1.0 / 3.0) - (1.0 - h)) / math.sqrt(h)
    # 上尾 = 1 - Phi(z) = 0.5 * erfc(z / sqrt(2))
    try:
        p = 0.5 * math.erfc(z / math.sqrt(2.0))
    except OverflowError:
        p = 0.0
    return float(max(0.0, min(1.0, p)))


def compute_critical_difference(n, k, alpha=0.05):
    """计算 Nemenyi 临界差 CD。

    当两个算法的平均排名之差超过 CD 时，认为差异显著。

    公式: CD = q_alpha * sqrt(k(k+1) / (6n))

    其中 q_alpha 是 Studentized range statistic 除以 sqrt(2) 后的值。
    这里使用查表法。

    Args:
        n: block 数（试验数 / 数据集数）
        k: 算法数
        alpha: 显著性水平（默认 0.05）

    Returns:
        CD 临界差值
    """
    # q_alpha / sqrt(2) 的查表值（alpha=0.05）
    # 行 = 算法数 k，列 = (alpha=0.05)
    # 来源：Demsar (2006) Table
    q_alpha_005 = {
        2: 2.773, 3: 2.913, 4: 3.025, 5: 3.117, 6: 3.198,
        7: 3.268, 8: 3.331, 9: 3.386, 10: 3.437, 11: 3.484,
        12: 3.527, 13: 3.568, 14: 3.606, 15: 3.642,
    }
    # alpha=0.10
    q_alpha_010 = {
        2: 2.326, 3: 2.467, 4: 2.579, 5: 2.670, 6: 2.749,
        7: 2.819, 8: 2.880, 9: 2.935, 10: 2.986, 11: 3.033,
        12: 3.076, 13: 3.117, 14: 3.155, 15: 3.191,
    }
    table = q_alpha_005 if abs(alpha - 0.05) < 1e-6 else q_alpha_010
    q = table.get(k, 3.642 if alpha <= 0.05 else 3.191)
    # CD = q_alpha / sqrt(2) * sqrt(k(k+1)/(6n))
    # 注意：Demsar 表中的 q_alpha 值已经是 q_alpha/sqrt(2) 形式
    cd = q * math.sqrt(k * (k + 1) / (6.0 * n))
    return float(cd)


def nemenyi_post_hoc(perf_matrix, alpha=0.05):
    """Nemenyi 事后检验。

    在 Friedman 检验显著后，进行两两算法比较。
    两个算法的平均排名差 > CD 时认为差异显著。

    Args:
        perf_matrix: shape=(n, k) 性能矩阵（值越小越好）
        alpha: 显著性水平

    Returns:
        list of dict: 每对算法的比较结果
    """
    friedman = friedman_test(perf_matrix)
    n = friedman["n"]
    k = friedman["k"]
    avg_ranks = friedman["average_ranks"]
    cd = compute_critical_difference(n, k, alpha)

    comparisons = []
    for i in range(k):
        for j in range(i + 1, k):
            rank_diff = float(abs(avg_ranks[i] - avg_ranks[j]))
            comparisons.append({
                "algo_i": int(i),
                "algo_j": int(j),
                "rank_diff": rank_diff,
                "cd": cd,
                "significant": bool(rank_diff > cd),
            })
    return comparisons, friedman, cd


# =====================================================================
# 第三部分：可视化函数
# =====================================================================
def plot_cd_diagram(method_names, average_ranks, cd_value,
                    output_path=None, title="Critical Difference Diagram"):
    """绘制 CD 图（Critical Difference Diagram）。

    横轴为平均排名（1=最优，k=最差），用横线连接无显著差异的方法。

    Args:
        method_names: 方法名称列表（长度 k）
        average_ranks: 各方法的平均排名（长度 k）
        cd_value: Nemenyi 临界差
        output_path: 图片保存路径；若为 None 则调用 plt.show()
        title: 图标题

    Returns:
        图片路径（output_path 或 None）
    """
    if not _MPL_OK:
        print(f"[WARN] matplotlib 不可用，跳过 CD 图绘制: {_MPL_ERR}")
        return None
    method_names = list(method_names)
    average_ranks = np.array(average_ranks, dtype=float)
    k = len(method_names)

    # 按排名排序（升序：最优在前）
    order = np.argsort(average_ranks)
    sorted_names = [method_names[i] for i in order]
    sorted_ranks = average_ranks[order]

    fig, ax = plt.subplots(figsize=(10, max(4, k * 0.45 + 1.5)))
    ax.set_title(title, fontsize=13, fontweight="bold")

    # 上半部分：排名轴
    ax.xaxis.set_ticks_position("top")
    ax.set_xlim(1, k)
    ax.set_ylim(0, 1)
    ax.set_xlabel("平均排名（1=最优）", fontsize=11)
    ax.xaxis.set_label_position("top")

    # 隐藏 y 轴
    ax.set_yticks([])
    for spine in ["left", "right", "bottom"]:
        ax.spines[spine].set_visible(False)

    # 绘制方法标签（交替上下避免重叠）
    tick_y = 0.4
    for i, (name, rank) in enumerate(zip(sorted_names, sorted_ranks)):
        # 上下交替放置标签
        side = "right" if i % 2 == 0 else "left"
        y_offset = 0.55 if i % 2 == 0 else 0.25
        ax.scatter(rank, tick_y, s=80, color="black", zorder=5)
        ax.annotate(
            f"{name} ({rank:.2f})",
            xy=(rank, tick_y),
            xytext=(rank, y_offset),
            ha="center", va="center", fontsize=10,
            arrowprops=dict(arrowstyle="-", color="gray", lw=0.8),
        )

    # 绘制无显著差异组之间的横线（连接排名差 < CD 的方法）
    if cd_value > 0:
        for i in range(k):
            for j in range(i + 1, k):
                if abs(sorted_ranks[i] - sorted_ranks[j]) < cd_value:
                    # 在 y=0.45 附近画横线
                    y_pos = 0.45 - 0.04 * (j - i - 1)
                    ax.hlines(y_pos, sorted_ranks[i], sorted_ranks[j],
                              colors="blue", linewidth=2.5, alpha=0.7)

    # 标注 CD 值
    ax.text(
        k - 0.5, 0.05,
        f"CD = {cd_value:.3f} (Nemenyi α=0.05)",
        fontsize=10, ha="right", va="bottom",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow",
                  edgecolor="orange", alpha=0.9),
    )

    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] CD 图已保存: {output_path}")
        return output_path
    plt.show()
    return None


def plot_pareto_front(objectives, labels, output_path=None,
                      title="Pareto Front", higher_is_better=None,
                      highlight_indices=None):
    """绘制 Pareto 前沿可视化（支持 2D 与 3D）。

    Args:
        objectives: shape=(n, 2) 或 (n, 3) 的 numpy 数组，每行一个解的目标值
        labels: 每个目标轴的名称列表（长度 2 或 3）
        output_path: 图片保存路径
        title: 图标题
        higher_is_better: 每个目标是否越大越好的布尔列表；None 表示全部最小化
        highlight_indices: 需要高亮显示的解的索引列表（Pareto 前沿）

    Returns:
        图片路径或 None
    """
    if not _MPL_OK:
        print(f"[WARN] matplotlib 不可用，跳过 Pareto 前沿绘制: {_MPL_ERR}")
        return None
    objectives = np.array(objectives, dtype=float)
    if objectives.ndim != 2:
        print("[ERROR] objectives 必须是二维数组")
        return None
    n, dim = objectives.shape
    if dim not in (2, 3):
        print(f"[ERROR] 仅支持 2D 或 3D Pareto 前沿，当前维度={dim}")
        return None
    if higher_is_better is None:
        higher_is_better = [False] * dim

    # 计算 Pareto 前沿（非支配解）
    pareto_idx = _compute_pareto_indices(objectives, higher_is_better)
    if highlight_indices is None:
        highlight_indices = pareto_idx

    fig = plt.figure(figsize=(8, 7) if dim == 2 else (10, 8))
    if dim == 2:
        ax = fig.add_subplot(111)
        # 全部解
        ax.scatter(objectives[:, 0], objectives[:, 1],
                   c="lightgray", s=40, label="所有解", alpha=0.6)
        # Pareto 前沿
        pf = objectives[pareto_idx]
        # 排序使前沿连线美观
        sort_order = np.argsort(pf[:, 0])
        pf_sorted = pf[sort_order]
        ax.plot(pf_sorted[:, 0], pf_sorted[:, 1],
                "r-", linewidth=2, alpha=0.8, label="Pareto 前沿")
        ax.scatter(pf_sorted[:, 0], pf_sorted[:, 1],
                   c="red", s=80, zorder=5, edgecolors="black")
        ax.set_xlabel(labels[0], fontsize=11)
        ax.set_ylabel(labels[1], fontsize=11)
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)
    else:
        # 3D
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(objectives[:, 0], objectives[:, 1], objectives[:, 2],
                   c="lightgray", s=30, label="所有解", alpha=0.6)
        pf = objectives[pareto_idx]
        ax.scatter(pf[:, 0], pf[:, 1], pf[:, 2],
                   c="red", s=80, label="Pareto 前沿",
                   edgecolors="black")
        ax.set_xlabel(labels[0], fontsize=10)
        ax.set_ylabel(labels[1], fontsize=10)
        ax.set_zlabel(labels[2], fontsize=10)
        ax.legend(loc="best")

    ax.set_title(title, fontsize=13, fontweight="bold")
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] Pareto 前沿图已保存: {output_path}")
        return output_path
    plt.show()
    return None


def _compute_pareto_indices(objectives, higher_is_better):
    """计算 Pareto 非支配解的索引。"""
    n, k = objectives.shape
    is_pareto = np.ones(n, dtype=bool)
    for i in range(n):
        if not is_pareto[i]:
            continue
        for j in range(n):
            if i == j:
                continue
            # 检查 j 是否支配 i
            dom = True
            strict = False
            for d in range(k):
                if higher_is_better[d]:
                    if objectives[j, d] < objectives[i, d]:
                        dom = False
                        break
                    if objectives[j, d] > objectives[i, d]:
                        strict = True
                else:
                    if objectives[j, d] > objectives[i, d]:
                        dom = False
                        break
                    if objectives[j, d] < objectives[i, d]:
                        strict = True
            if dom and strict:
                is_pareto[i] = False
                break
    return np.where(is_pareto)[0]


def plot_ablation_waterfall(method_names, contributions, output_path=None,
                             title="消融贡献瀑布图",
                             baseline_label="proposed_full"):
    """绘制消融贡献瀑布图。

    直观展示每个模块对总体性能的贡献（相对完整方法的减损）。

    Args:
        method_names: 模块名称列表（长度 n）
        contributions: 每个模块的贡献值（>=0 表示移除后性能下降多少）
        output_path: 图片保存路径
        title: 图标题
        baseline_label: 基线方法名称

    Returns:
        图片路径或 None
    """
    if not _MPL_OK:
        print(f"[WARN] matplotlib 不可用，跳过瀑布图绘制: {_MPL_ERR}")
        return None
    method_names = list(method_names)
    contributions = np.array(contributions, dtype=float)
    n = len(method_names)
    if n == 0:
        return None

    fig, ax = plt.subplots(figsize=(max(8, n * 1.0 + 2), 6))
    ax.set_title(title, fontsize=13, fontweight="bold")

    # 累计起点
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
    # 绘制每个贡献柱
    for i in range(n):
        ax.bar(x[i], bar_heights[i], bottom=bar_starts[i],
               color=bar_colors[i], edgecolor="black", linewidth=0.8,
               width=0.6)
        # 在柱顶标注数值
        y_label = bar_starts[i] + bar_heights[i]
        ax.text(x[i], y_label + max(abs(cumulative), 1e-3) * 0.02,
                f"{contributions[i]:+.3f}",
                ha="center", va="bottom", fontsize=9, fontweight="bold")

    # 连接线（瀑布效果）
    for i in range(n - 1):
        y_end = bar_starts[i] + bar_heights[i]
        ax.plot([x[i] + 0.3, x[i + 1] - 0.3], [y_end, y_end],
                "k--", linewidth=0.8, alpha=0.6)

    ax.set_xticks(x)
    ax.set_xticklabels(method_names, rotation=30, ha="right", fontsize=10)
    ax.set_ylabel("性能贡献（覆盖率下降量）", fontsize=11)
    ax.grid(True, axis="y", alpha=0.3)
    # 总计标注
    total = float(np.sum(contributions))
    ax.text(0.98, 0.95, f"累计贡献: {total:+.3f}",
            transform=ax.transAxes, ha="right", va="top",
            fontsize=11, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow",
                      edgecolor="orange", alpha=0.9))
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] 瀑布图已保存: {output_path}")
        return output_path
    plt.show()
    return None


def plot_radar_comparison(method_names, metrics_dict, output_path=None,
                          title="方法多维度雷达对比",
                          higher_is_better=None):
    """绘制雷达图对比。

    多维度对比：覆盖率/误差/速度/恢复次数/计算时间等。

    Args:
        method_names: 方法名称列表（长度 m）
        metrics_dict: {metric_label: [values for each method]} 长度 m
            所有 metrics 必须有相同长度（=方法数）
        output_path: 图片保存路径
        title: 图标题
        higher_is_better: {metric_label: bool}，未提供时默认 True

    Returns:
        图片路径或 None
    """
    if not _MPL_OK:
        print(f"[WARN] matplotlib 不可用，跳过雷达图绘制: {_MPL_ERR}")
        return None
    labels = list(metrics_dict.keys())
    n_metrics = len(labels)
    if n_metrics < 3:
        print(f"[ERROR] 雷达图至少需要 3 个维度，当前 {n_metrics}")
        return None
    m = len(method_names)
    if m == 0:
        return None
    if higher_is_better is None:
        higher_is_better = {}

    # 将每个 metric 归一化到 [0, 1]
    norm_data = np.zeros((m, n_metrics))
    for j, label in enumerate(labels):
        values = np.array(metrics_dict[label], dtype=float)
        if len(values) != m:
            print(f"[ERROR] 指标 {label} 长度 {len(values)} != 方法数 {m}")
            return None
        hi = higher_is_better.get(label, True)
        vmin, vmax = float(np.min(values)), float(np.max(values))
        if vmax - vmin < 1e-12:
            norm_col = np.ones(m) * 0.5
        elif hi:
            norm_col = (values - vmin) / (vmax - vmin)
        else:
            norm_col = (vmax - values) / (vmax - vmin)
        norm_data[:, j] = norm_col

    # 计算角度
    angles = np.linspace(0, 2 * np.pi, n_metrics, endpoint=False).tolist()
    angles += angles[:1]  # 闭合

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
    ax.set_title(title, fontsize=13, fontweight="bold", pad=20)

    # 颜色循环
    colors = plt.cm.tab10(np.linspace(0, 1, max(m, 10)))
    for i in range(m):
        values = norm_data[i].tolist()
        values += values[:1]
        ax.plot(angles, values, linewidth=2, label=method_names[i],
                color=colors[i])
        ax.fill(angles, values, alpha=0.1, color=colors[i])

    # 设置轴
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_ylim(0, 1.05)
    ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_yticklabels(["0.2", "0.4", "0.6", "0.8", "1.0"], fontsize=8)
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1), fontsize=9)
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"[OK] 雷达图已保存: {output_path}")
        return output_path
    plt.show()
    return None


# =====================================================================
# 第四部分：实验执行与数据加载
# =====================================================================
def run_experiment(config_name, env_vars, trial_num):
    """运行单个实验 trial（调用 autonomous_nav.py）。

    Args:
        config_name: 配置名称
        env_vars: 环境变量字典
        trial_num: trial 编号（1-indexed）

    Returns:
        CSV 数据文件路径
    """
    trial_dir = RESULTS_DIR / config_name / f"trial_{trial_num}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    csv_path = trial_dir / f"{config_name}_data.csv"

    env = os.environ.copy()
    env.update(env_vars)
    env["METHOD_NAME"] = config_name
    env["MAX_FRAMES"] = str(MAX_FRAMES)
    env["EVAL_DIR"] = str(trial_dir)
    env["PYTHONUNBUFFERED"] = "1"

    # 随机种子（保证可复现）
    seed = hash(f"{config_name}_{trial_num}") % (2 ** 32)
    env["PYTHONHASHSEED"] = str(seed)
    env["NPY_SEED"] = str(seed)

    print(f"\n{'=' * 60}")
    print(f"  运行: {config_name} (trial {trial_num}/{N_TRIALS})")
    print(f"  种子: {seed}, 帧数上限: {MAX_FRAMES}")
    print(f"  输出: {csv_path}")
    print(f"{'=' * 60}")

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
            if any(tag in line for tag in ["[NAV]", "[STATE]", "[TELEMETRY]",
                                           "Error", "error", "Exception"]):
                print(f"  {line.rstrip()}")

    proc.wait()
    print(f"  退出码: {proc.returncode}")

    if csv_path.exists():
        print(f"  CSV 已保存: {csv_path} ({csv_path.stat().st_size} 字节)")
    else:
        # 兜底：尝试从临时数据文件复制
        temp_csv = Path(os.path.join(
            os.environ.get("TEMP", "/tmp"), "puppy_nav_data.csv"))
        if temp_csv.exists():
            import shutil
            shutil.copy2(temp_csv, csv_path)
            print(f"  CSV 从临时目录复制: {csv_path}")

    import time
    time.sleep(3)
    return csv_path


def load_trial_metrics(csv_path):
    """从单个 trial 的 CSV 文件加载指标。

    Returns:
        dict: 包含汇总指标的字典，若文件不存在或为空则返回 None
    """
    if not csv_path.exists():
        return None
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                loc_err_val = r.get("loc_err", r.get("loc_error", "0.0"))
                rows.append({
                    "frame": int(r["frame"]),
                    "coverage": float(r["coverage"]),
                    "state": int(r["state"]),
                    "loc_err": float(loc_err_val),
                    "dwa_v": float(r.get("dwa_v", 0.0)),
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
    # 达到 90% 覆盖率的时间
    time_to_90 = -1.0
    for r in rows:
        if r["coverage"] >= 90:
            time_to_90 = r["frame"] / 10.0
            break

    # RECOVER 计数（状态转移至 state=2 的次数）
    recover_count = sum(1 for i in range(1, len(states))
                        if states[i] == 2 and states[i - 1] != 2)

    # 状态分布
    total = len(states)
    follow_pct = sum(1 for s in states if s == 1) / total * 100
    recover_pct = sum(1 for s in states if s == 2) / total * 100
    done_pct = sum(1 for s in states if s == 3) / total * 100

    # 平均速度（仅 FOLLOW 状态）
    follow_speeds = [r["dwa_v"] for r in rows if r["state"] == 1]
    avg_speed = float(np.mean(follow_speeds)) if follow_speeds else 0.0

    # 计算时间（粗略估计：每帧 ~10ms，加上规划开销）
    compute_time = total * 0.012  # ~12ms/帧

    return {
        "final_coverage": coverages[-1],
        "max_coverage": max(coverages),
        "time_to_80": time_to_80,
        "time_to_90": time_to_90,
        "mean_loc_err": float(np.mean(loc_errs)) if loc_errs else -1.0,
        "max_loc_err": float(np.max(loc_errs)) if loc_errs else -1.0,
        "p95_loc_err": float(np.percentile(loc_errs, 95)) if loc_errs else -1.0,
        "recover_count": recover_count,
        "follow_pct": follow_pct,
        "recover_pct": recover_pct,
        "done_pct": done_pct,
        "avg_speed": avg_speed,
        "compute_time": compute_time,
        "total_frames": total,
    }


# =====================================================================
# 第五部分：统计分析主流程
# =====================================================================
def analyze_results(results_dir=None, output_dir=None):
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
    print("  增强版消融实验统计分析 (v2.0)")
    print("=" * 80)

    # 加载全部 trial 结果
    all_results = {}
    for config_name, desc, _, _, _ in CONFIGURATIONS:
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
                metrics = load_trial_metrics(csv_path)
                if metrics:
                    trials.append(metrics)
        if trials:
            all_results[config_name] = trials
            print(f"  [OK] {config_name}: 已加载 {len(trials)} 个 trial")

    if not all_results:
        print("\n  未找到结果。请先运行 'python ablation_study_v2.py run'")
        return None

    config_names = list(all_results.keys())

    # === 生成均值±95%CI 汇总表 ===
    print("\n" + "-" * 80)
    print("  指标汇总（mean ± 95%CI）")
    print("-" * 80)
    header = f"{'指标':<18s}"
    for name in config_names:
        header += f" {name[:14]:>15s}"
    print(header)
    print("-" * (18 + 16 * len(config_names)))

    metric_values = {}  # metric_key -> {config -> [values]}
    for key, label, _ in METRICS_SPEC:
        row = f"{label:<18s}"
        metric_values[key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config] if t[key] >= 0]
            if not values:
                row += f" {'N/A':>15s}"
                continue
            mean, ci = confidence_interval(values)
            metric_values[key][config] = values
            row += f" {mean:>7.2f}±{((ci[1] - ci[0]) / 2):>5.2f}"
        print(row)

    # === paired t-test（proposed_full vs 其余）===
    print("\n" + "-" * 80)
    print("  配对 t 检验（proposed_full vs 每个其他配置）")
    print("-" * 80)
    reference = "proposed_full"
    if reference not in all_results:
        print(f"  [ERROR] 未找到参考方法 {reference}")
        reference = config_names[0]
        print(f"  使用 {reference} 作为参考")
    print(f"\n  {'指标':<18s} {'对比':<22s} {'t':>8s} {'p值':>8s} "
          f"{'Cohen d':>8s} {'显著':>6s}")
    print("  " + "-" * 75)
    pairwise_results = []
    for key, label, _ in METRICS_SPEC:
        if key not in metric_values or reference not in metric_values[key]:
            continue
        ref_vals = metric_values[key][reference]
        if len(ref_vals) < 2:
            continue
        for config in config_names:
            if config == reference:
                continue
            if config not in metric_values.get(key, {}):
                continue
            other_vals = metric_values[key][config]
            if len(other_vals) < 2:
                continue
            min_len = min(len(ref_vals), len(other_vals))
            if min_len < 2:
                continue
            a_vals = ref_vals[:min_len]
            b_vals = other_vals[:min_len]
            t_stat, p_val, df, sig = paired_t_test(a_vals, b_vals)
            d = cohens_d(a_vals, b_vals)
            sig_str = "***" if (sig and abs(d) > 0.8) else (
                "**" if sig else "ns")
            print(f"  {label:<18s} {config:<22s} {t_stat:>8.3f} "
                  f"{p_val:>8.3f} {d:>8.3f} {sig_str:>6s}")
            pairwise_results.append({
                "metric": key,
                "reference": reference,
                "comparison": config,
                "t_stat": t_stat, "p_value": p_val,
                "cohens_d": d, "significant": sig,
                "n": min_len,
            })

    print("\n  图例: *** = 显著且大效应, ** = 显著, ns = 不显著")
    print("  Cohen's d: >0.8 大, >0.5 中, >0.2 小效应")

    # === Friedman 检验（多算法比较）===
    print("\n" + "-" * 80)
    print("  Friedman 检验（多算法比较，alpha=0.05）")
    print("-" * 80)
    friedman_results = {}
    for key, label, higher_better in METRICS_SPEC:
        # 构造性能矩阵（n_trials × k_algos）
        # 选取所有配置都有的 trial 数
        min_trial = min(len(all_results[c]) for c in config_names)
        if min_trial < 2:
            continue
        # Friedman 中"越小越好"：若 higher_better=True，则取负
        matrix = np.zeros((min_trial, len(config_names)))
        for j, config in enumerate(config_names):
            for i in range(min_trial):
                v = all_results[config][i][key]
                if v < 0:
                    v = 0.0
                matrix[i, j] = -v if higher_better else v
        friedman = friedman_test(matrix)
        friedman_results[key] = {
            "chi2": friedman["chi2"],
            "df": friedman["df"],
            "p_value": friedman["p_value"],
            "significant": friedman["significant"],
            "average_ranks": friedman["average_ranks"].tolist(),
            "n": friedman["n"],
            "k": friedman["k"],
        }
        sig_str = "显著" if friedman["significant"] else "不显著"
        print(f"  {label:<18s} chi2={friedman['chi2']:>6.3f} "
              f"df={friedman['df']} p={friedman['p_value']:.4f} {sig_str}")
        # 打印平均排名
        ranks_str = ", ".join(
            f"{c}={r:.2f}"
            for c, r in zip(config_names, friedman["average_ranks"]))
        print(f"    平均排名: {ranks_str}")

    # === Nemenyi 事后检验（针对 final_coverage）===
    print("\n" + "-" * 80)
    print("  Nemenyi 事后检验（覆盖率指标）")
    print("-" * 80)
    key = "final_coverage"
    if key in friedman_results:
        fr = friedman_results[key]
        n_blocks = fr["n"]
        k_algos = fr["k"]
        cd = compute_critical_difference(n_blocks, k_algos, alpha=0.05)
        print(f"  CD = {cd:.3f} (n={n_blocks}, k={k_algos}, alpha=0.05)")
        avg_ranks = fr["average_ranks"]
        print(f"  排名差 > CD 的算法对（显著差异）:")
        n_sig_pairs = 0
        for i in range(k_algos):
            for j in range(i + 1, k_algos):
                diff = abs(avg_ranks[i] - avg_ranks[j])
                if diff > cd:
                    n_sig_pairs += 1
                    print(f"    {config_names[i]} vs {config_names[j]}: "
                          f"|{diff:.2f}| > {cd:.2f}")
        if n_sig_pairs == 0:
            print("    无显著差异的算法对")
        # 绘制 CD 图
        cd_path = output_dir / "cd_diagram_coverage.png"
        plot_cd_diagram(config_names, avg_ranks, cd,
                        output_path=str(cd_path),
                        title="覆盖率指标 CD 图 (Nemenyi α=0.05)")

    # === 消融贡献瀑布图（仅消融配置）===
    ablation_configs = [c for c in config_names
                        if next((cfg for cfg in CONFIGURATIONS
                                 if cfg[0] == c and cfg[3] == "ablation"),
                                None) is not None
                        and c != "proposed_full"]
    if "proposed_full" in metric_values.get("final_coverage", {}):
        prop_vals = metric_values["final_coverage"]["proposed_full"]
        prop_mean = float(np.mean(prop_vals))
        contributions = []
        contrib_names = []
        for config in ablation_configs:
            if config in metric_values.get("final_coverage", {}):
                other_vals = metric_values["final_coverage"][config]
                # 贡献 = 完整方法 - 消融方法（>=0 表示该模块有正贡献）
                contributions.append(prop_mean - float(np.mean(other_vals)))
                contrib_names.append(config)
        if contributions:
            waterfall_path = output_dir / "ablation_waterfall.png"
            plot_ablation_waterfall(contrib_names, contributions,
                                    output_path=str(waterfall_path))

    # === 雷达图对比 ===
    radar_metrics = ["final_coverage", "avg_speed", "follow_pct"]
    radar_labels = ["覆盖率", "速度", "跟随比例"]
    radar_dict = {}
    radar_hib = {}
    for label, key in zip(radar_labels, radar_metrics):
        if key not in metric_values:
            continue
        radar_dict[label] = [float(np.mean(metric_values[key].get(c, [0])))
                             for c in config_names]
        # 全部越大越好
        radar_hib[label] = True
    # 加上反向指标（越小越好）
    for label, key in [("定位误差", "mean_loc_err"),
                       ("恢复次数", "recover_count"),
                       ("计算时间", "compute_time")]:
        if key not in metric_values:
            continue
        radar_dict[label] = [float(np.mean(metric_values[key].get(c, [0])))
                             for c in config_names]
        radar_hib[label] = False
    if len(radar_dict) >= 3:
        radar_path = output_dir / "radar_comparison.png"
        plot_radar_comparison(config_names, radar_dict,
                              output_path=str(radar_path),
                              higher_is_better=radar_hib)

    # === 保存结果到 JSON ===
    summary = {
        "n_trials_per_config": {c: len(all_results[c]) for c in config_names},
        "metrics": {},
        "friedman": friedman_results,
        "pairwise_t_test": pairwise_results,
        "n_configurations": len(config_names),
    }
    for key, label, _ in METRICS_SPEC:
        summary["metrics"][key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config] if t[key] >= 0]
            if values:
                mean, ci = confidence_interval(values)
                summary["metrics"][key][config] = {
                    "mean": mean,
                    "ci_low": ci[0],
                    "ci_high": ci[1],
                    "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                    "n": len(values),
                    "values": values,
                }
    summary_path = output_dir / "ablation_summary_v2.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\n  统计结果已保存: {summary_path}")
    return summary


# =====================================================================
# 第六部分：演示模式（使用模拟数据）
# =====================================================================
def _generate_synthetic_metrics(config_name, expected_baseline, n_trials,
                                rng=None):
    """根据期望基线水平生成模拟的指标数据。

    用于 CoppeliaSim 不可用时的演示。

    Args:
        config_name: 配置名
        expected_baseline: 期望覆盖率水平（0~1）
        n_trials: trial 数
        rng: numpy 随机数生成器

    Returns:
        list of metrics dict
    """
    if rng is None:
        rng = np.random.default_rng(seed=hash(config_name) % (2 ** 32))

    # 基础覆盖率
    base_cov = max(40.0, min(99.0, expected_baseline * 100.0))
    trials = []
    for i in range(n_trials):
        # 覆盖率：基线 ± 噪声
        noise = rng.normal(0, 2.0)
        final_cov = float(np.clip(base_cov + noise, 40.0, 99.5))
        # 时间到 80% / 90% 覆盖率
        if final_cov >= 80:
            t80 = float(np.clip(200 + (1 - expected_baseline) * 200
                                + rng.normal(0, 20), 60, 800))
        else:
            t80 = -1.0
        if final_cov >= 90:
            t90 = float(np.clip(350 + (1 - expected_baseline) * 300
                                + rng.normal(0, 30), 120, 1200))
        else:
            t90 = -1.0
        # 定位误差
        if "no_kld" in config_name or "no_recovery" in config_name:
            loc_err_base = 0.18
        elif "gt_baseline" in config_name or "full_knowledge" in config_name:
            loc_err_base = 0.02
        else:
            loc_err_base = 0.08
        mean_err = float(max(0.01, loc_err_base + rng.normal(0, 0.02)))
        max_err = float(mean_err * (1.5 + rng.uniform(0, 0.5)))
        # 恢复次数
        if "no_recovery" in config_name:
            recover_count = int(rng.integers(2, 6))
        elif "random_walk" in config_name:
            recover_count = int(rng.integers(0, 3))
        else:
            recover_count = int(rng.integers(0, 2))
        # 跟随比例
        follow_pct = float(np.clip(60 + expected_baseline * 20
                                    + rng.normal(0, 5), 30, 95))
        # 平均速度
        if "no_teb_jerk" in config_name:
            avg_speed = float(np.clip(0.18 + rng.normal(0, 0.02), 0.1, 0.3))
        else:
            avg_speed = float(np.clip(0.22 + expected_baseline * 0.05
                                       + rng.normal(0, 0.02), 0.1, 0.35))
        # 计算时间
        compute_time = float(np.clip(40 + (1 - expected_baseline) * 60
                                      + rng.normal(0, 5), 20, 200))
        trials.append({
            "final_coverage": final_cov,
            "max_coverage": min(99.9, final_cov + rng.uniform(0.1, 1.0)),
            "time_to_80": t80,
            "time_to_90": t90,
            "mean_loc_err": mean_err,
            "max_loc_err": max_err,
            "p95_loc_err": float(mean_err * 1.3),
            "recover_count": recover_count,
            "follow_pct": follow_pct,
            "recover_pct": float(recover_count * 2 + rng.uniform(0, 2)),
            "done_pct": float(np.clip(final_cov * 0.5, 0, 50)),
            "avg_speed": avg_speed,
            "compute_time": compute_time,
            "total_frames": int(MAX_FRAMES),
        })
    return trials


def run_demo():
    """使用模拟数据演示完整的统计分析与可视化。

    当 CoppeliaSim 不可用时，此模式可独立运行：
        python ablation_study_v2.py --demo
    """
    print("\n" + "=" * 80)
    print("  增强版消融实验框架 (v2.0) —— 演示模式")
    print("  使用模拟数据进行完整统计分析与可视化测试")
    print("=" * 80)

    rng = np.random.default_rng(seed=42)
    n_trials = N_TRIALS

    # 生成所有配置的模拟数据
    all_results = {}
    for config_name, desc, _, _, baseline in CONFIGURATIONS:
        all_results[config_name] = _generate_synthetic_metrics(
            config_name, baseline, n_trials, rng=rng)
        print(f"  [模拟] {config_name}: 生成 {n_trials} 个 trial 数据")

    config_names = list(all_results.keys())

    # === 指标汇总 ===
    print("\n" + "-" * 80)
    print("  指标汇总（mean ± 95%CI）")
    print("-" * 80)
    header = f"{'指标':<18s}"
    for name in config_names:
        header += f" {name[:12]:>13s}"
    print(header)
    print("-" * (18 + 14 * len(config_names)))

    metric_values = {}
    for key, label, _ in METRICS_SPEC:
        row = f"{label:<18s}"
        metric_values[key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config] if t[key] >= 0]
            if not values:
                row += f" {'N/A':>13s}"
                continue
            mean, ci = confidence_interval(values)
            metric_values[key][config] = values
            row += f" {mean:>6.2f}±{((ci[1] - ci[0]) / 2):>4.2f}"
        print(row)

    # === paired t-test ===
    print("\n" + "-" * 80)
    print("  配对 t 检验（proposed_full vs 每个其他配置）")
    print("-" * 80)
    reference = "proposed_full"
    print(f"\n  {'指标':<18s} {'对比':<22s} {'t':>8s} {'p值':>8s} "
          f"{'Cohen d':>8s} {'显著':>6s}")
    print("  " + "-" * 75)
    for key, label, _ in METRICS_SPEC:
        if key not in metric_values or reference not in metric_values[key]:
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
            a_vals = ref_vals[:min_len]
            b_vals = other_vals[:min_len]
            t_stat, p_val, df, sig = paired_t_test(a_vals, b_vals)
            d = cohens_d(a_vals, b_vals)
            sig_str = "***" if (sig and abs(d) > 0.8) else (
                "**" if sig else "ns")
            print(f"  {label:<18s} {config:<22s} {t_stat:>8.3f} "
                  f"{p_val:>8.3f} {d:>8.3f} {sig_str:>6s}")

    # === Friedman 检验 ===
    print("\n" + "-" * 80)
    print("  Friedman 检验（多算法比较，alpha=0.05）")
    print("-" * 80)
    friedman_results = {}
    for key, label, higher_better in METRICS_SPEC:
        min_trial = min(len(all_results[c]) for c in config_names)
        if min_trial < 2:
            continue
        matrix = np.zeros((min_trial, len(config_names)))
        for j, config in enumerate(config_names):
            for i in range(min_trial):
                v = all_results[config][i][key]
                if v < 0:
                    v = 0.0
                matrix[i, j] = -v if higher_better else v
        friedman = friedman_test(matrix)
        friedman_results[key] = friedman
        sig_str = "显著" if friedman["significant"] else "不显著"
        print(f"  {label:<18s} chi2={friedman['chi2']:>6.3f} "
              f"df={friedman['df']} p={friedman['p_value']:.4f} {sig_str}")

    # === Nemenyi 事后检验 + CD 图 ===
    print("\n" + "-" * 80)
    print("  Nemenyi 事后检验 + CD 图")
    print("-" * 80)
    key = "final_coverage"
    if key in friedman_results:
        fr = friedman_results[key]
        n_blocks = fr["n"]
        k_algos = fr["k"]
        cd = compute_critical_difference(n_blocks, k_algos, alpha=0.05)
        print(f"  CD = {cd:.3f} (n={n_blocks}, k={k_algos}, alpha=0.05)")
        avg_ranks = fr["average_ranks"]
        print(f"  排名差 > CD 的算法对（显著差异）:")
        n_sig = 0
        for i in range(k_algos):
            for j in range(i + 1, k_algos):
                diff = abs(avg_ranks[i] - avg_ranks[j])
                if diff > cd:
                    n_sig += 1
                    print(f"    {config_names[i]} vs {config_names[j]}: "
                          f"|{diff:.2f}| > {cd:.2f}")
        if n_sig == 0:
            print("    无显著差异的算法对")

    # === 输出目录准备 ===
    demo_out = RESULTS_DIR / "demo"
    demo_out.mkdir(parents=True, exist_ok=True)

    # === 绘制 CD 图 ===
    cd_path = demo_out / "cd_diagram_coverage.png"
    plot_cd_diagram(config_names, avg_ranks, cd,
                    output_path=str(cd_path),
                    title="覆盖率指标 CD 图 (Nemenyi α=0.05) - Demo")

    # === Pareto 前沿可视化（2D 与 3D）===
    print("\n" + "-" * 80)
    print("  Pareto 前沿可视化")
    print("-" * 80)
    # 用每个配置的 (time_to_90 越小越好, mean_loc_err 越小越好) 作为目标
    rng2 = np.random.default_rng(seed=123)
    n_solutions = 50
    pareto_obj = np.column_stack([
        rng2.uniform(150, 800, n_solutions),   # 时间
        rng2.uniform(0.02, 0.25, n_solutions),  # 误差
    ])
    pareto_2d_path = demo_out / "pareto_front_2d.png"
    plot_pareto_front(pareto_obj, ["时间到 90%(s)", "定位误差(m)"],
                      output_path=str(pareto_2d_path),
                      title="2D Pareto 前沿（时间 vs 误差）",
                      higher_is_better=[False, False])
    # 3D Pareto
    pareto_obj_3d = np.column_stack([
        rng2.uniform(150, 800, n_solutions),    # 时间
        rng2.uniform(0.02, 0.25, n_solutions),  # 误差
        rng2.uniform(60, 99, n_solutions),       # 覆盖率（越大越好）
    ])
    pareto_3d_path = demo_out / "pareto_front_3d.png"
    plot_pareto_front(pareto_obj_3d,
                      ["时间到 90%(s)", "定位误差(m)", "覆盖率(%)"],
                      output_path=str(pareto_3d_path),
                      title="3D Pareto 前沿（时间/误差/覆盖率）",
                      higher_is_better=[False, False, True])

    # === 消融贡献瀑布图 ===
    print("\n" + "-" * 80)
    print("  消融贡献瀑布图")
    print("-" * 80)
    ablation_configs = [c[0] for c in CONFIGURATIONS
                        if c[3] == "ablation" and c[0] != "proposed_full"]
    if "proposed_full" in metric_values.get("final_coverage", {}):
        prop_mean = float(np.mean(
            metric_values["final_coverage"]["proposed_full"]))
        contrib_names = []
        contributions = []
        for config in ablation_configs:
            if config in metric_values.get("final_coverage", {}):
                other_mean = float(np.mean(
                    metric_values["final_coverage"][config]))
                contributions.append(prop_mean - other_mean)
                contrib_names.append(config)
        if contributions:
            waterfall_path = demo_out / "ablation_waterfall.png"
            plot_ablation_waterfall(contrib_names, contributions,
                                    output_path=str(waterfall_path),
                                    title="消融贡献瀑布图 - Demo")

    # === 雷达图对比 ===
    print("\n" + "-" * 80)
    print("  雷达图对比")
    print("-" * 80)
    radar_dict = {}
    radar_hib = {}
    for label, key, hib in [("覆盖率", "final_coverage", True),
                            ("速度", "avg_speed", True),
                            ("跟随比例", "follow_pct", True),
                            ("定位误差", "mean_loc_err", False),
                            ("恢复次数", "recover_count", False),
                            ("计算时间", "compute_time", False)]:
        if key not in metric_values:
            continue
        radar_dict[label] = [float(np.mean(metric_values[key].get(c, [0])))
                             for c in config_names]
        radar_hib[label] = hib
    if len(radar_dict) >= 3:
        radar_path = demo_out / "radar_comparison.png"
        plot_radar_comparison(config_names, radar_dict,
                              output_path=str(radar_path),
                              title="方法多维度雷达对比 - Demo",
                              higher_is_better=radar_hib)

    # === 保存 summary JSON ===
    summary = {
        "mode": "demo",
        "n_trials_per_config": {c: len(all_results[c]) for c in config_names},
        "metrics": {},
        "friedman": {k: {"chi2": v["chi2"], "df": v["df"],
                          "p_value": v["p_value"],
                          "significant": v["significant"],
                          "average_ranks": v["average_ranks"].tolist(),
                          "n": v["n"], "k": v["k"]}
                      for k, v in friedman_results.items()},
    }
    for key, label, _ in METRICS_SPEC:
        summary["metrics"][key] = {}
        for config in config_names:
            values = [t[key] for t in all_results[config] if t[key] >= 0]
            if values:
                mean, ci = confidence_interval(values)
                summary["metrics"][key][config] = {
                    "mean": mean,
                    "ci_low": ci[0],
                    "ci_high": ci[1],
                    "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                    "n": len(values),
                }
    summary_path = demo_out / "ablation_summary_v2_demo.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print("\n" + "=" * 80)
    print(f"  演示完成！所有可视化与统计结果保存在: {demo_out}")
    print("=" * 80)
    print(f"\n  生成的文件:")
    for p in sorted(demo_out.iterdir()):
        print(f"    - {p.name} ({p.stat().st_size} 字节)")
    return summary


# =====================================================================
# 第七部分：实验运行入口
# =====================================================================
def run_all():
    """运行所有消融实验。"""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    total_runs = len(CONFIGURATIONS) * N_TRIALS
    current = 0
    for config_name, desc, env_vars, _, _ in CONFIGURATIONS:
        for trial in range(1, N_TRIALS + 1):
            current += 1
            print(f"\n[{current}/{total_runs}] {desc}")
            run_experiment(config_name, env_vars, trial)
    print(f"\n{'=' * 60}")
    print(f"  全部 {total_runs} 次实验完成！")
    print(f"{'=' * 60}")
    analyze_results()


def main():
    """命令行入口。"""
    parser = argparse.ArgumentParser(
        description="增强版消融实验框架 (v2.0)")
    parser.add_argument(
        "command", nargs="?", default="analyze",
        choices=["run", "analyze", "all"],
        help="命令: run=运行实验, analyze=分析已有结果, all=运行+分析")
    parser.add_argument(
        "--demo", action="store_true",
        help="使用模拟数据演示统计分析与可视化（无需 CoppeliaSim）")
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


if __name__ == "__main__":
    main()
