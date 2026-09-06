#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_experiment_tables.py — 自动生成论文实验矩阵 (jihua20260905 §9.1 / §9.3 / §14)
================================================================================
把仓库里"已经存在但散落"的证据自动汇总成可直接进论文的表格, 避免手工抄日志。

输入:
  ablation_results/ablation_summary.json   消融实验 (proposed_v3 vs 4 组消融, 3 trial)
  artifacts/cpp_*_planB.json               C++ 数值仿真多种子长稳结果

输出 (artifacts/<run_id>/):
  navigation_stability_table.md            多种子导航稳定性表 (均值±std / P50 / P95 / 最差)
  ablation_table.md                        消融对比表 (mean±std, 最优加粗)
  failure_cases.md                         失败案例表 (规划失败按原因、近距、卡死、最差种子)
  experiment_tables.json                   机器可读汇总, 供下游报告消费

设计约束:
  - 只依赖标准库 (环境无 PyYAML/numpy 也能跑)
  - 不重新跑实验, 只汇总已有 artifacts; 缺数据明确写 "NO DATA", 绝不编造
  - 每个表都写明"数据来自哪个文件", 可追溯

用法:
    python scripts/gen_experiment_tables.py
    python scripts/gen_experiment_tables.py --run-id experiments-20260906
"""
import os
import re
import sys
import json
import math
import glob
import argparse
import datetime
import subprocess

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

ART = os.path.join(_ROOT, "artifacts")

# ----------------------------------------------------------------------------
# 消融指标定义 (标签/方向), 与 ablation_study.py:317 保持一致
# ----------------------------------------------------------------------------
ABL_METRICS = [
    # (key, 中文名, 单位, higher_better, 分组)
    ("final_coverage", "最终覆盖率", "%", True, "建图/覆盖"),
    ("time_to_80", "覆盖80%耗时", "s", False, "建图/覆盖"),
    ("time_to_90", "覆盖90%耗时", "s", False, "建图/覆盖"),
    ("mean_loc_err", "平均定位误差", "m", False, "定位"),
    ("max_loc_err", "峰值定位误差", "m", False, "定位"),
    ("recover_count", "重定位次数", "次", False, "定位"),
    ("follow_pct", "跟踪跟随率", "%", True, "视觉跟踪"),
    ("avg_speed", "平均速度", "m/s", True, "运动"),
]

# 消融组含义 (来自 ablation_study.py:39-52 的 config 描述)
ABL_DESC = {
    "proposed_v3": "完整方案 v3.0: KLD-AMCL + TEB(时间+急动) + 信息论前沿",
    "no_kld": "消融: 固定粒子 AMCL (去掉 KLD 采样)",
    "no_info_theory": "消融: 计数前沿 (去掉香农熵)",
    "dwa_baseline": "消融: DWA 规划器 (去掉 TEB 时间最优+急动)",
    "gt_baseline": "消融: 真值定位 (去掉 AMCL)",
}

# 导航稳定性列: (metrics key, 中文名, 单位, higher_better)
NAV_COLS = [
    ("astar_rate_pct", "A*成功率", "%", True),
    ("amcl_avg_err_m", "平均定位误差", "m", False),
    ("amcl_max_err_m", "峰值定位误差", "m", False),
    ("stuck_ratio_pct", "卡死占比", "%", False),
    ("collisions", "碰撞数", "次", False),
    ("near_miss", "近距事件", "次", False),
    ("rooms_visited", "访问房间", "个", True),
    ("total_distance_m", "运动距离", "m", True),
    ("rounds_completed", "完成巡逻圈", "圈", True),
    ("avg_speed", "平均速度", "m/s", True),
]

# 规划失败原因 -> 中文解释 (来自 planning_detail)
FAIL_REASONS = {
    "fail_no_nearest_start": "起点在膨胀后不可达(需修正)",
    "fail_no_nearest_goal": "终点在膨胀后不可达(需修正)",
    "fail_no_path": "给定代价地图下无可行路径",
    "fail_timeout": "超出 max_planning_time",
    "fail_max_nodes": "超出 MAX_NODES 上限",
}


# ----------------------------------------------------------------------------
# 统计工具
# ----------------------------------------------------------------------------
def mean(xs):
    xs = [x for x in xs if isinstance(x, (int, float))]
    return sum(xs) / len(xs) if xs else None


def stdev(xs):
    xs = [x for x in xs if isinstance(x, (int, float))]
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def pct(xs, p):
    """线性插值分位数。"""
    xs = sorted(x for x in xs if isinstance(x, (int, float)))
    if not xs:
        return None
    if len(xs) == 1:
        return xs[0]
    k = (len(xs) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return xs[int(k)]
    return xs[f] + (xs[c] - xs[f]) * (k - f)


def worst(xs, higher_better):
    xs = [x for x in xs if isinstance(x, (int, float))]
    if not xs:
        return None
    return min(xs) if higher_better else max(xs)


def fmt(v, digits=3):
    if v is None:
        return "NO DATA"
    if isinstance(v, int) or (isinstance(v, float) and v == int(v)):
        return str(int(v))
    return ("%." + str(digits) + "f") % v


def fmt_ms(v, digits=3):
    """fmt 的 mean±std 形式。"""
    if v is None:
        return "NO DATA"
    return fmt(v, digits)


def git_sha():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=_ROOT,
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "unknown"


# ----------------------------------------------------------------------------
# 加载数据
# ----------------------------------------------------------------------------
def load_planb(prefix):
    """加载 artifacts/cpp_<prefix>_seed*_planB.json, 返回按 seed 排序的列表。"""
    out = []
    for f in glob.glob(os.path.join(ART, "cpp_%s_seed*_planB.json" % prefix)):
        m = re.search(r"seed(\d+)_planB\.json$", os.path.basename(f))
        try:
            d = json.load(open(f, encoding="utf-8"))
        except Exception:
            continue
        out.append((int(m.group(1)) if m else 0, os.path.basename(f), d))
    out.sort(key=lambda t: t[0])
    return out


def detect_prefixes():
    """自动检测存在的 planB 前缀, 如 cpp_36000 / cpp_108000。"""
    found = set()
    for f in glob.glob(os.path.join(ART, "cpp_*_seed*_planB.json")):
        m = re.search(r"cpp_(.+?)_seed\d+_planB\.json$", os.path.basename(f))
        if m:
            found.add(m.group(1))
    return sorted(found, key=lambda p: (len(p), p))


def ablation_quality(abl, variants):
    """数据质量告警 —— 防止把"不可支撑结论的消融"直接搬进论文。

    jihua20260905 §9.1 要求: 每个实验至少 5~10 次重复, 报告均值/标准差/P50/P95/最差。
    §2.3 要求: 不能只挑一次最好看的结果。
    """
    warns = []

    # 1) 重复次数
    for v in variants:
        n = (abl.get(v) or {}).get("n_trials", 0)
        if isinstance(n, int) and n < 5:
            warns.append(("重复次数不足",
                          "`%s` 仅 %d 次重复；§9.1 要求 5~10 次，"
                          "当前样本量不足以支撑显著性结论。" % (v, n)))

    # 2) 指标饱和 / 方差过大
    for key, label, unit, hb, grp in ABL_METRICS:
        seen = {}
        allv = []
        for v in variants:
            mm = ((abl.get(v) or {}).get("metrics") or {}).get(key)
            if mm and isinstance(mm.get("mean"), (int, float)):
                seen[v] = mm
                allv.extend([x for x in (mm.get("values") or [])
                             if isinstance(x, (int, float))])
        if not seen:
            continue
        if allv and len(set(round(x, 9) for x in allv)) == 1:
            warns.append(("指标饱和",
                          "`%s`（%s）在所有配置下取值恒为 %s，无区分度，"
                          "不应作为消融结论依据。" % (key, label, fmt(allv[0]))))
            continue
        for v, mm in seen.items():
            sd = mm.get("std") or 0.0
            mn = mm.get("mean") or 0.0
            if mn != 0 and abs(sd / mn) > 0.4:
                warns.append(("方差过大",
                              "`%s` 的 `%s` 相对标准差 %.0f%%（mean=%.3f，std=%.3f），"
                              "在 n=%s 时组间差异不可信。"
                              % (v, label, 100 * abs(sd / mn), mn, sd,
                                 (abl.get(v) or {}).get("n_trials", "?"))))

    return warns


def load_ablation(path):
    if not os.path.exists(path):
        return None
    return json.load(open(path, encoding="utf-8"))


# ----------------------------------------------------------------------------
# 表 1: 多种子导航稳定性
# ----------------------------------------------------------------------------
def build_nav_table(prefixes):
    lines = []
    lines.append("# 多种子导航稳定性表")
    lines.append("")
    lines.append("数据来源: `artifacts/cpp_<prefix>_seedN_planB.json` (C++ 数值仿真, Plan B 配置)。")
    lines.append("生成命令: `python scripts/gen_experiment_tables.py`  ")
    lines.append("Git SHA: `%s`" % git_sha())
    lines.append("")
    lines.append("说明: 每行为一个随机种子的独立长跑; 底部为该组的统计量。")
    lines.append("`A*成功率`、`访问房间`、`完成巡逻圈`、`运动距离`、`平均速度` 越高越好;")
    lines.append("`误差`、`卡死占比`、`碰撞数`、`近距事件` 越低越好。")
    lines.append("")

    stats_for_json = {}

    for prefix in prefixes:
        rows = load_planb(prefix)
        if not rows:
            continue
        frames = rows[0][2].get("frames")
        lines.append("## 组: `%s` (每种子 %s 帧, %d 个种子)" % (prefix, frames, len(rows)))
        lines.append("")
        header = "| 种子 | 状态 | " + " | ".join(c[1] + "(" + c[2] + ")" for c in NAV_COLS) + " |"
        lines.append(header)
        lines.append("|" + "---|" * (len(NAV_COLS) + 2))

        for seed, fname, d in rows:
            m = d.get("metrics", {})
            cells = ["%d" % seed, "`%s`" % d.get("status", "?")]
            for key, _label, _unit, _hb in NAV_COLS:
                v = m.get(key)
                if v is None:
                    cells.append("NO DATA")
                elif isinstance(v, float):
                    cells.append("%.3f" % v)
                else:
                    cells.append(str(v))
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")

        # 统计
        lines.append("**统计汇总**")
        lines.append("")
        lines.append("| 指标 | 单位 | 方向 | 均值 | 标准差 | P50 | P95 | 最差 |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for key, label, unit, hb in NAV_COLS:
            xs = [d.get("metrics", {}).get(key) for _s, _f, d in rows]
            xs = [x for x in xs if isinstance(x, (int, float))]
            if not xs:
                lines.append("| %s | %s | %s | NO DATA | - | - | - | - |" % (label, unit, "↑" if hb else "↓"))
                continue
            lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
                label, unit, "↑" if hb else "↓",
                fmt(mean(xs)), fmt(stdev(xs)), fmt(pct(xs, 0.50)),
                fmt(pct(xs, 0.95)), fmt(worst(xs, hb))))
            stats_for_json.setdefault(prefix, {})[key] = {
                "mean": mean(xs), "std": stdev(xs), "p50": pct(xs, 0.50),
                "p95": pct(xs, 0.95), "worst": worst(xs, hb),
                "n": len(xs), "higher_better": hb,
            }
        lines.append("")

    return "\n".join(lines), stats_for_json


# ----------------------------------------------------------------------------
# 表 2: 消融对比
# ----------------------------------------------------------------------------
def build_ablation_table(abl):
    lines = []
    lines.append("# 消融实验对比表")
    lines.append("")
    if abl is None:
        lines.append("> **NO DATA**: 未找到 `ablation_results/ablation_summary.json`。")
        lines.append("> 请先运行 `python ablation_study.py all` 生成消融结果。")
        lines.append("")
        return "\n".join(lines), None, []

    variants = [v for v in ("proposed_v3", "no_kld", "no_info_theory",
                            "dwa_baseline", "gt_baseline") if v in abl]
    order = [v for v in abl if v not in variants]
    variants += order

    lines.append("数据来源: `ablation_results/ablation_summary.json` (每配置 3 次重复)。")
    lines.append("生成命令: `python scripts/gen_experiment_tables.py`  ")
    lines.append("Git SHA: `%s`" % git_sha())
    lines.append("")

    # ---- 归属与范围: 本消融跑的是 research-only 的 CoppeliaSim/Python 栈 ----
    lines.append("## 归属与范围警告 (必读)")
    lines.append("")
    lines.append("> 本消融由 `ablation_study.py` 驱动 **`autonomous_nav.py`** 运行；")
    lines.append("> 该脚本自述为 *「RUNTIME: research (CoppeliaSim research stack, ")
    lines.append("> NOT part of ROS2 product line)」*。")
    lines.append(">")
    lines.append("> 因此本表属于 **research-only 证据**，与毕业设计主线")
    lines.append("> （C++ `puppy_nav_core` 数值仿真 / ROS2 Humble / UE5 bridge）")
    lines.append("> **不是同一条链路**。论文引用时必须标明来源链路，")
    lines.append("> 不得把它当作主线导航方案的验收结论。")
    lines.append("")
    lines.append("## 配置含义")
    lines.append("")
    lines.append("| 配置 | 含义 | 重复次数 |")
    lines.append("|---|---|---|")
    for v in variants:
        n = (abl.get(v) or {}).get("n_trials", "?")
        lines.append("| `%s` | %s | %s |" % (v, ABL_DESC.get(v, "—"), n))
    lines.append("")

    # 每个分组一张表: 行=配置, 列=指标(mean±std), 最优加粗
    groups = []
    for _k, _l, _u, _hb, g in ABL_METRICS:
        if g not in groups:
            groups.append(g)

    json_out = {}
    for g in groups:
        keys = [(k, l, u, hb) for k, l, u, hb, gg in ABL_METRICS if gg == g]
        lines.append("## %s" % g)
        lines.append("")
        lines.append("| 配置 | " + " | ".join("%s(%s)%s" % (l, u, "↑" if hb else "↓") for _k, l, u, hb in keys) + " |")
        lines.append("|" + "---|" * (len(keys) + 1))

        # 先找每列最优, 用于加粗
        best = {}
        for k, _l, _u, hb in keys:
            vals = {}
            for v in variants:
                mm = ((abl.get(v) or {}).get("metrics") or {}).get(k)
                if mm and isinstance(mm.get("mean"), (int, float)):
                    vals[v] = mm["mean"]
            if vals:
                target = max(vals.values()) if hb else min(vals.values())
                best[k] = [v for v in vals if abs(vals[v] - target) < 1e-12]

        for v in variants:
            cells = ["`%s`" % v]
            for k, _l, _u, _hb in keys:
                mm = ((abl.get(v) or {}).get("metrics") or {}).get(k)
                if not mm or not isinstance(mm.get("mean"), (int, float)):
                    cells.append("N/A")
                    json_out.setdefault(v, {})[k] = None
                    continue
                sd = mm.get("std", 0.0) or 0.0
                cell = "%.3f±%.3f" % (mm["mean"], sd)
                if k in best and v in best[k]:
                    cell = "**%s**" % cell
                cells.append(cell)
                json_out.setdefault(v, {})[k] = {
                    "mean": mm["mean"], "std": sd,
                    "values": mm.get("values", []),
                    "best": bool(k in best and v in best[k]),
                }
            lines.append("| " + " | ".join(cells) + " |")
        lines.append("")

    # --- 数据质量告警 (诚实边界, 先于结论) ---
    warns = ablation_quality(abl, variants)
    lines.append("## 数据质量告警")
    lines.append("")
    if not warns:
        lines.append("无。所有配置重复次数达标、指标具区分度且方差在可接受范围。")
    else:
        lines.append("> **这些告警必须先解决, 才能把本表写进论文的结论章节。**")
        lines.append("")
        lines.append("| 类别 | 说明 |")
        lines.append("|---|---|")
        for cat, msg in warns:
            lines.append("| %s | %s |" % (cat, msg))
    lines.append("")

    lines.append("> 加粗 = 该指标下的最优配置（**注意**: 加粗只表示数值最优，"
                 "在重复次数不足/方差过大的情况下**不代表显著差异**）。  ")
    lines.append("> `gt_baseline` 使用真值定位, 因此不含定位误差指标 (N/A), 它用于给出定位误差的"
                 "下界参考而非可部署方案。")
    lines.append("")

    return "\n".join(lines), json_out, warns


# ----------------------------------------------------------------------------
# 表 3: 失败案例
# ----------------------------------------------------------------------------
def build_failure_table(prefixes):
    lines = []
    lines.append("# 失败案例表")
    lines.append("")
    lines.append("数据来源: `artifacts/cpp_<prefix>_seedN_planB.json` 的 `metrics.planning_detail` "
                 "与 `metrics`。")
    lines.append("生成命令: `python scripts/gen_experiment_tables.py`  ")
    lines.append("Git SHA: `%s`" % git_sha())
    lines.append("")
    lines.append("原则: 只统计真实记录的失败, 不隐藏、不美化。"
                 "安全类失败(碰撞/近距)与任务类失败(规划/卡死)分开看, "
                 "防止\"机器人不动所以零碰撞\"的虚假安全结论。")
    lines.append("")

    json_out = {}

    for prefix in prefixes:
        rows = load_planb(prefix)
        if not rows:
            continue
        n = len(rows)
        lines.append("## 组: `%s` (%d 个种子合计)" % (prefix, n))
        lines.append("")

        # --- 规划失败按原因聚合 ---
        agg = {}
        for _s, _f, d in rows:
            pd = (d.get("metrics") or {}).get("planning_detail") or {}
            for rk in FAIL_REASONS:
                agg[rk] = agg.get(rk, 0) + int(pd.get(rk, 0) or 0)
        total_calls = sum(int((d.get("metrics") or {}).get("astar_calls", 0) or 0) for _s, _f, d in rows)
        total_fail = sum(agg.values())

        lines.append("### 规划失败构成")
        lines.append("")
        lines.append("| 失败原因 | 含义 | 合计次数 | 占比(占A*调用) |")
        lines.append("|---|---|---|---|")
        if total_fail == 0:
            lines.append("| (无) | 全部种子零规划失败 | 0 | 0 |")
        else:
            for rk in FAIL_REASONS:
                c = agg.get(rk, 0)
                share = (100.0 * c / total_calls) if total_calls else 0.0
                lines.append("| `%s` | %s | %d | %.3f%% |" % (rk, FAIL_REASONS[rk], c, share))
        lines.append("")
        lines.append(r"- A\* 总调用次数: **%d**" % total_calls)
        lines.append("- 规划失败总次数: **%d** (%.3f%% of A\\* calls)" %
                     (total_fail, (100.0 * total_fail / total_calls) if total_calls else 0.0))
        pf_persist = sum(int((d.get("metrics") or {}).get("planning_persistent_failures", 0) or 0)
                         for _s, _f, d in rows)
        lines.append("- 持续性失败(连续失败计为不可恢复): **%d**" % pf_persist)
        lines.append("")

        # --- 安全类与任务类 ---
        def s(key):
            return sum(int((d.get("metrics") or {}).get(key, 0) or 0) for _s, _f, d in rows)

        coll = s("collisions")
        nm = s("near_miss")
        stall = s("stall_events")
        lines.append("### 安全与任务失败计数")
        lines.append("")
        lines.append("| 类别 | 指标 | 合计 | 说明 |")
        lines.append("|---|---|---|---|")
        lines.append("| 安全 | 碰撞 | %d | 正式回归要求全 0 |" % coll)
        lines.append("| 安全 | 近距事件 | %d | 低于安全间距但未接触 |" % nm)
        lines.append("| 任务 | 卡死回合 | %d | 长时间无位移 |" % stall)
        lines.append("")

        # --- 最差种子 ---
        lines.append("### 最差种子 (逐指标)")
        lines.append("")
        lines.append("| 指标 | 最差值 | 出现种子 | 方向 |")
        lines.append("|---|---|---|---|")
        for key, label, unit, hb in NAV_COLS:
            vals = []
            for sd, _f, d in rows:
                v = (d.get("metrics") or {}).get(key)
                if isinstance(v, (int, float)):
                    vals.append((v, sd))
            if not vals:
                lines.append("| %s | NO DATA | - | %s |" % (label, "↑" if hb else "↓"))
                continue
            wv, wseed = max(vals, key=lambda t: t[0]) if not hb else min(vals, key=lambda t: t[0])
            lines.append("| %s | %s %s | seed %d | %s |" % (label, fmt(wv), unit, wseed, "↑" if hb else "↓"))
        lines.append("")

        json_out[prefix] = {
            "astar_calls": total_calls,
            "planning_fail_by_reason": agg,
            "planning_fail_total": total_fail,
            "planning_persistent_failures": pf_persist,
            "collisions": coll, "near_miss": nm, "stall_events": stall,
        }

    # --- 已知限制与解释 ---
    lines.append("## 失败案例解释 (诚实边界)")
    lines.append("")
    lines.append("- `fail_no_path` 是主要失败形态: 在规划半径 0.35m 膨胀后的代价地图上, "
                 "部分靠近墙/家具的临时目标点不可达。系统通过 `start_corrected_count` / "
                 "`goal_corrected_count` 自动修正后重试, 因此单次失败 **不等于** 任务失败。")
    lines.append(r"- 判断规划能力应看 **A\* 成功率均值/最差** 与 **持续性失败次数**, "
                 "而不是绝对失败次数; 持续性失败为 0 表示没有出现不可恢复的规划死锁。")
    lines.append("- 近距事件是安全余量被压缩的预警, 不等于碰撞; 但需要在论文中单独报告, "
                 "不能只报\"零碰撞\"。")
    lines.append("")

    return "\n".join(lines), json_out


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
def main():
    global ART
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", default="experiments-" + datetime.date.today().isoformat())
    ap.add_argument("--artifacts", default=ART)
    ap.add_argument("--ablation", default=os.path.join(_ROOT, "ablation_results", "ablation_summary.json"))
    ap.add_argument("--prefix", nargs="*", default=None,
                    help="planB 前缀, 默认自动检测 (如 36000 108000)")
    args = ap.parse_args()

    ART = args.artifacts

    prefixes = args.prefix or detect_prefixes()
    if not prefixes:
        print("[WARN] 未找到任何 cpp_*_planB.json, 导航表与失败表将为 NO DATA", file=sys.stderr)

    out_dir = os.path.join(ART, args.run_id)
    os.makedirs(out_dir, exist_ok=True)

    abl = load_ablation(args.ablation)

    nav_md, nav_stats = build_nav_table(prefixes)
    abl_md, abl_stats, abl_warns = build_ablation_table(abl)
    fail_md, fail_stats = build_failure_table(prefixes)

    for name, md in (("navigation_stability_table.md", nav_md),
                     ("ablation_table.md", abl_md),
                     ("failure_cases.md", fail_md)):
        with open(os.path.join(out_dir, name), "w", encoding="utf-8") as f:
            f.write(md)

    with open(os.path.join(out_dir, "experiment_tables.json"), "w", encoding="utf-8") as f:
        json.dump({
            "run_id": args.run_id,
            "generated_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "git_sha": git_sha(),
            "prefixes": prefixes,
            "navigation": nav_stats,
            "ablation": abl_stats,
            "failures": fail_stats,
            "ablation_quality_warnings": [
                {"category": c, "message": m} for c, m in abl_warns],
        }, f, indent=2, ensure_ascii=False)

    print("实验矩阵已生成 -> %s" % out_dir)
    print("  navigation_stability_table.md")
    print("  ablation_table.md")
    print("  failure_cases.md")
    print("  experiment_tables.json")
    if abl is None:
        print("  [WARN] 消融数据缺失, ablation_table.md 为 NO DATA")
    return 0


if __name__ == "__main__":
    sys.exit(main())
