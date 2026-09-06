#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_release_report.py — 一键生成发布验收报告 (jihua20260905.md §9.3)

聚合 artifacts/ 下 C++ 长稳 PlanB JSON + 每帧耗时 CSV, 输出:

    artifacts/<run_id>/
      environment.json      OS / 编译器 / Git SHA / 时间
      metadata.json         run_id / profile / 种子范围 / 数据来源
      config_hashes.json    scene/contract/geometry/params 的 SHA-256
      cpp_seed_<n>.json     逐种子原始指标 (复制自 artifacts/)
      summary.md            多 seed 均值/标准差/P50/P95/最差 + 验收门禁
      plots/                SVG 图表 (A*率/AMCL误差/stuck/房间/碰撞/时延/距离)

图表零依赖 (手写 SVG), 不需要 matplotlib。
若提供 --trace <csv>, 额外生成 trajectory.svg (真值 vs 估计) 与 error.svg (逐帧误差)。

用法:
  python scripts/gen_release_report.py --run-id release-<date> [--cpp-prefix cpp_36000 cpp_108000] [--trace trace.csv] [--profile release]
"""
import argparse
import csv
import datetime
import hashlib
import json
import os
import statistics
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ART = os.path.join(ROOT, "artifacts")

# 验收门禁 (jihua20260905.md §12, 当前场景 v6.0)
GATES = [
    # (key, 标签, 方向: 'min' 越高越好 / 'max' 越低越好, 阈值, 单位)
    ("astar_rate_pct", "A* 成功率", "min", 95.0, "%"),
    ("amcl_avg_err_m", "AMCL 平均误差", "max", 0.20, "m"),
    ("amcl_max_err_m", "AMCL 峰值误差", "max", 0.75, "m"),
    ("stuck_ratio_pct", "stuck 比例", "max", 20.0, "%"),
    ("collisions", "碰撞次数", "max", 0, ""),
    ("rooms_visited", "房间覆盖", "min", 4, ""),
    ("total_distance_m", "运动距离(1h)", "min", 10.0, "m"),
]


def sha256_file(p):
    if not os.path.exists(p):
        return None
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def git_commit():
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                             capture_output=True, text=True, timeout=15)
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:
        return None


def pctl(vals, q):
    if not vals:
        return 0.0
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    idx = q * (len(s) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)


def load_seed(prefix, seed):
    fp = os.path.join(ART, "%s_seed%d_planB.json" % (prefix, seed))
    if not os.path.exists(fp):
        return None
    d = json.load(open(fp, encoding="utf-8"))
    m = d.get("metrics", {})
    return {
        "seed": seed,
        "status": d.get("status"),
        "astar_rate_pct": m.get("astar_rate_pct"),
        "amcl_avg_err_m": m.get("amcl_avg_err_m"),
        "amcl_max_err_m": m.get("amcl_max_err_m"),
        "stuck_ratio_pct": m.get("stuck_ratio_pct"),
        "collisions": m.get("collisions"),
        "rooms_visited": m.get("rooms_visited"),
        "total_distance_m": m.get("total_distance_m"),
        "avg_speed": m.get("avg_speed"),
        "near_miss": m.get("near_miss"),
        "stall_events": m.get("stall_events"),
        "planning_failures": m.get("planning_failures"),
        "amcl_conf": m.get("amcl_conf"),
    }


def load_latency(prefix, seed):
    fp = os.path.join(ART, "%s_seed%d_planB.timing.csv" % (prefix, seed))
    if not os.path.exists(fp):
        return []
    ms = []
    with open(fp, newline="") as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if len(row) >= 2:
                try:
                    ms.append(float(row[1]))
                except ValueError:
                    pass
    return ms


# ---------------- SVG 图表 (零依赖) ----------------
def _bar_chart(title, seeds, values, ylabel, ref_lines=None, color="#2b6cb0"):
    """竖向条形图: values 与 seeds 等长。ref_lines: [(value, label, color)]。"""
    ref_lines = ref_lines or []
    W, H = 720, 360
    pad_l, pad_b = 48, 40
    plot_w, plot_h = W - pad_l - 16, H - 60 - pad_b
    n = len(seeds) or 1
    vmax = max([v for v in values if v is not None] + [1.0] + [r[0] for r in ref_lines])
    vmin = min([v for v in values if v is not None] + [0.0])
    if vmax == vmin:
        vmax = vmin + 1
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" font-family="sans-serif" font-size="11">' % (W, H)]
    parts.append('<rect width="%d" height="%d" fill="#fff"/>' % (W, H))
    parts.append('<text x="%d" y="22" font-size="13" font-weight="700" fill="#1f2933">%s</text>' % (pad_l, title))
    # y 轴
    for i in range(5):
        yv = vmin + (vmax - vmin) * i / 4
        y = 60 + plot_h * (1 - i / 4)
        parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#e2e6ea"/>' % (pad_l, y, W - 16, y))
        parts.append('<text x="%d" y="%.1f" text-anchor="end" fill="#5a6b7b">%.2f</text>' % (pad_l - 4, y + 3, yv))
    bw = plot_w / n * 0.7
    for i, (s, v) in enumerate(zip(seeds, values)):
        if v is None:
            continue
        x = pad_l + (plot_w / n) * i + (plot_w / n - bw) / 2
        yh = plot_h * (v - vmin) / (vmax - vmin)
        y = 60 + plot_h - yh
        parts.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s"/>' % (x, y, bw, yh, color))
        parts.append('<text x="%.1f" y="%.1f" text-anchor="middle" fill="#374151">%.2f</text>' % (x + bw / 2, y - 3, v))
        parts.append('<text x="%.1f" y="%d" text-anchor="middle" fill="#8a97a5">%d</text>' % (x + bw / 2, H - pad_b + 14, s))
    for rv, rlbl, rc in ref_lines:
        if rv < vmin or rv > vmax:
            continue
        y = 60 + plot_h * (1 - (rv - vmin) / (vmax - vmin))
        parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="%s" stroke-dasharray="5 4"/>' % (pad_l, y, W - 16, y, rc))
        parts.append('<text x="%d" y="%.1f" fill="%s">%s</text>' % (W - 14, y - 3, rc, rlbl))
    parts.append('<text x="%d" y="%d" fill="#5a6b7b">%s</text>' % (pad_l, H - 6, ylabel))
    parts.append('</svg>')
    return "\n".join(parts)


def _hist_chart(title, ms, p99, ylabel="帧耗时"):
    W, H = 720, 360
    if not ms:
        return '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d"><rect width="%d" height="%d" fill="#fff"/><text x="20" y="40" fill="#b7791f">无耗时数据</text></svg>' % (W, H, W, H)
    bins = 40
    mx = max(ms)
    step = (mx or 1) / bins
    counts = [0] * bins
    for v in ms:
        counts[min(bins - 1, int(v / step))] += 1
    cmax = max(counts) or 1
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" font-family="sans-serif" font-size="11">' % (W, H)]
    parts.append('<rect width="%d" height="%d" fill="#fff"/>' % (W, H))
    parts.append('<text x="20" y="22" font-size="13" font-weight="700" fill="#1f2933">%s (n=%d, mean=%.2fms, P99=%.2fms)</text>' % (title, len(ms), statistics.mean(ms), p99))
    pad_l, pad_b = 48, 36
    plot_w, plot_h = W - pad_l - 16, H - 56 - pad_b
    bw = plot_w / bins
    for i, c in enumerate(counts):
        x = pad_l + bw * i
        h = plot_h * c / cmax
        y = 56 + plot_h - h
        parts.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="#2b6cb0" opacity="0.85"/>' % (x, y, bw - 1, h))
    # P99 线
    if p99 <= mx:
        xp = pad_l + (p99 / (mx or 1)) * plot_w
        parts.append('<line x1="%.1f" y1="56" x2="%.1f" y2="%.1f" stroke="#c0392b" stroke-dasharray="5 4"/>' % (xp, xp, 56 + plot_h))
        parts.append('<text x="%.1f" y="52" fill="#c0392b">P99=%.1fms</text>' % (xp + 2, p99))
    parts.append('<text x="%d" y="%d" fill="#5a6b7b">%s (ms)</text>' % (pad_l, H - 6, ylabel))
    parts.append('</svg>')
    return "\n".join(parts)


def _traj_chart(title, true_pts, est_pts):
    W, H = 720, 420
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" font-family="sans-serif" font-size="11">' % (W, H)]
    parts.append('<rect width="%d" height="%d" fill="#fff"/>' % (W, H))
    parts.append('<text x="20" y="22" font-size="13" font-weight="700" fill="#1f2933">%s</text>' % title)
    if not true_pts:
        parts.append('<text x="20" y="60" fill="#b7791f">无 trace 数据</text></svg>')
        return "\n".join(parts)
    xs = [p[0] for p in true_pts] + [p[0] for p in est_pts]
    ys = [p[1] for p in true_pts] + [p[1] for p in est_pts]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    if maxx == minx:
        maxx += 1
    if maxy == miny:
        maxy += 1
    pad = 40
    def tx(x):
        return pad + (x - minx) / (maxx - minx) * (W - 2 * pad)
    def ty(y):
        return H - pad - (y - miny) / (maxy - miny) * (H - 2 * pad)

    def poly(pts, stroke, fill):
        d = " ".join("%.1f,%.1f" % (tx(x), ty(y)) for x, y in pts)
        return '<polyline points="%s" fill="none" stroke="%s" stroke-width="1.5"/>' % (d, stroke)
    parts.append(poly(true_pts, "#1a7f37"))
    parts.append(poly(est_pts, "#c0392b"))
    parts.append('<text x="%d" y="%d" fill="#1a7f37">— 真值</text>' % (pad, H - 12))
    parts.append('<text x="%d" y="%d" fill="#c0392b">— 估计(AMCL)</text>' % (pad + 70, H - 12))
    parts.append('</svg>')
    return "\n".join(parts)


def main():
    global ART
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", default="release-" + datetime.date.today().isoformat())
    ap.add_argument("--artifacts", default=ART)
    ap.add_argument("--cpp-prefix", nargs="*", default=None,
                    help="要聚合的 PlanB 前缀, 默认自动检测 cpp_108000 / cpp_36000")
    ap.add_argument("--profile", default="release")
    ap.add_argument("--trace", default=None, help="可选逐帧 trace CSV (frame,true_x,...,est_x,...) 用于轨迹/误差图")
    args = ap.parse_args()

    ART = args.artifacts

    # ---- 自动探测前缀 ----
    prefixes = args.cpp_prefix or []
    if not prefixes:
        import glob
        seen = set()
        for fp in glob.glob(os.path.join(ART, "cpp_*_seed1_planB.json")):
            pre = os.path.basename(fp).split("_seed1")[0]
            seen.add(pre)
        prefixes = sorted(seen)
    if not prefixes:
        print("[gen_release_report] 未找到任何 cpp_*_seedN_planB.json, 退出", file=sys.stderr)
        return 2

    out_dir = os.path.join(ART, args.run_id)
    os.makedirs(os.path.join(out_dir, "plots"), exist_ok=True)

    rows_all = {}
    for prefix in prefixes:
        rows = [load_seed(prefix, s) for s in range(1, 11)]
        rows = [r for r in rows if r]
        rows_all[prefix] = rows

    # ---- 聚合统计 ----
    def agg(key, direction):
        vals = [r[key] for r in sum(rows_all.values(), []) if isinstance(r.get(key), (int, float))]
        if not vals:
            return None
        return {
            "mean": statistics.mean(vals),
            "std": statistics.pstdev(vals),
            "p50": pctl(vals, 0.5),
            "p95": pctl(vals, 0.95),
            "min": min(vals),
            "max": max(vals),
            "worst": (min(vals) if direction == "min" else max(vals)),
            "n": len(vals),
        }

    stats = {k: agg(k, d) for k, _, d, _, _ in GATES}
    stats["total_distance_m"] = agg("total_distance_m", "min")
    stats["avg_speed"] = agg("avg_speed", "min")
    stats["near_miss"] = agg("near_miss", "max")
    stats["planning_failures"] = agg("planning_failures", "max")

    # ---- 门禁判定 ----
    gate_results = []
    for k, label, direction, thr, unit in GATES:
        st = stats.get(k)
        if not st:
            gate_results.append((label, direction, "NO DATA", thr, unit, False))
            continue
        ok = (st["worst"] >= thr) if direction == "min" else (st["worst"] <= thr)
        gate_results.append((label, direction, st["worst"], thr, unit, ok))
    passed = all(g[5] for g in gate_results)

    # ---- environment / metadata / hashes ----
    env = {
        "os": os.uname().sysname if hasattr(os, "uname") else "win32",
        "python": sys.version.split()[0],
        "git_commit": git_commit(),
        "generated_at": datetime.datetime.now().astimezone().isoformat(),
        "artifacts_source": [p + "_seed*_planB.json" for p in prefixes],
    }
    meta = {
        "run_id": args.run_id,
        "profile": args.profile,
        "prefixes": prefixes,
        "seeds_per_prefix": {p: len(rows_all[p]) for p in prefixes},
        "gates": [{"metric": g[0], "direction": g[1], "worst": g[2],
                   "threshold": g[3], "unit": g[4], "pass": g[5]} for g in gate_results],
        "overall_pass": passed,
    }
    hashes = {
        "scene_home.json": sha256_file(os.path.join(ROOT, "config", "scene_home.json")),
        "simulation_contract.yaml": sha256_file(os.path.join(ROOT, "config", "simulation_contract.yaml")),
        "geometry_spec.yaml": sha256_file(os.path.join(ROOT, "config", "geometry_spec.yaml")),
        "unified_params.yaml": sha256_file(os.path.join(ROOT, "config", "unified_params.yaml")),
        "motion_capability.yaml": sha256_file(os.path.join(ROOT, "config", "motion_capability.yaml")),
    }

    # ---- 复制逐种子 JSON ----
    copied = []
    for prefix in prefixes:
        for s in range(1, 11):
            src = os.path.join(ART, "%s_seed%d_planB.json" % (prefix, s))
            if os.path.exists(src):
                dst = os.path.join(out_dir, "%s_seed%d.json" % (prefix, s))
                import shutil
                shutil.copy2(src, dst)
                copied.append(os.path.basename(dst))

    # ---- 图表 ----
    all_seeds = []
    all_astar, all_avg, all_maxe, all_stuck, all_rooms, all_coll, all_dist = [], [], [], [], [], [], []
    for prefix in prefixes:
        for r in rows_all[prefix]:
            all_seeds.append(r["seed"])
            all_astar.append(r["astar_rate_pct"]); all_avg.append(r["amcl_avg_err_m"])
            all_maxe.append(r["amcl_max_err_m"]); all_stuck.append(r["stuck_ratio_pct"])
            all_rooms.append(r["rooms_visited"]); all_coll.append(r["collisions"])
            all_dist.append(r["total_distance_m"])

    plots = {
        "astar_rate_per_seed.svg": _bar_chart("A* 成功率 (per seed)", all_seeds, all_astar, "成功率 %", [(95, "门限95", "#1a7f37"), (90, "最差90", "#b7791f")]),
        "amcl_avg_per_seed.svg": _bar_chart("AMCL 平均误差 (per seed)", all_seeds, all_avg, "误差 m", [(0.20, "门限0.20", "#c0392b")], color="#1a4971"),
        "amcl_max_per_seed.svg": _bar_chart("AMCL 峰值误差 (per seed)", all_seeds, all_maxe, "误差 m", [(0.75, "门限0.75", "#c0392b")], color="#1a4971"),
        "stuck_per_seed.svg": _bar_chart("stuck 比例 (per seed)", all_seeds, all_stuck, "stuck %", [(20, "门限20", "#c0392b")], color="#9a5b00"),
        "rooms_per_seed.svg": _bar_chart("房间覆盖 (per seed)", all_seeds, all_rooms, "房间数", [(4, "门限4", "#1a7f37"), (5, "当前5", "#2b6cb0")], color="#2f855a"),
        "collisions_per_seed.svg": _bar_chart("碰撞次数 (per seed)", all_seeds, all_coll, "碰撞", [(0, "门限0", "#c0392b")], color="#2f855a"),
        "distance_per_seed.svg": _bar_chart("运动距离 (per seed)", all_seeds, all_dist, "距离 m", [(10, "门限10", "#1a7f37")], color="#2b6cb0"),
    }
    for name, svg in plots.items():
        with open(os.path.join(out_dir, "plots", name), "w", encoding="utf-8") as f:
            f.write(svg)

    # 帧时延直方图 (聚合所有前缀 seed1 的 timing)
    lat = []
    for prefix in prefixes:
        lat += load_latency(prefix, 1)
    if not lat:
        for prefix in prefixes:
            lat += load_latency(prefix, 1)
    p99v = pctl(sorted(lat), 0.99) if lat else 0
    with open(os.path.join(out_dir, "plots", "frame_latency_hist.svg"), "w", encoding="utf-8") as f:
        f.write(_hist_chart("单帧耗时分布 (seed1 聚合)", lat, p99v))

    # 可选轨迹/误差图
    if args.trace and os.path.exists(args.trace):
        tp, ep, errs = [], [], []
        with open(args.trace, newline="") as f:
            r = csv.reader(f)
            hdr = next(r, [])
            idx = {c: i for i, c in enumerate(hdr)}
            for row in r:
                try:
                    tx = float(row[idx["true_x"]]); ty = float(row[idx["true_y"]])
                    ex = float(row[idx["est_x"]]); ey = float(row[idx["est_y"]])
                    tp.append((tx, ty)); ep.append((ex, ey))
                    if "err" in idx:
                        errs.append(float(row[idx["err"]]))
                except (ValueError, KeyError, IndexError):
                    continue
        with open(os.path.join(out_dir, "plots", "trajectory.svg"), "w", encoding="utf-8") as f:
            f.write(_traj_chart("轨迹: 真值 vs AMCL 估计", tp, ep))
        if errs:
            with open(os.path.join(out_dir, "plots", "error_timeseries.svg"), "w", encoding="utf-8") as f:
                f.write(_hist_chart("逐帧定位误差分布", errs, pctl(sorted(errs), 0.99), "误差 m"))

    # ---- summary.md ----
    def fmtv(v, nd=3):
        if v is None:
            return "—"
        if isinstance(v, float):
            return ("%.2f" % v) if abs(v) >= 1 else ("%.3f" % v)
        return str(v)

    lines = [
        "# 发布验收报告 `%s`" % args.run_id,
        "",
        "- 生成时间: %s" % env["generated_at"],
        "- Git SHA: `%s`" % env["git_commit"],
        "- Profile: `%s`" % args.profile,
        "- 数据来源: %s" % ", ".join("`%s` ×%d seed" % (p, meta["seeds_per_prefix"][p]) for p in prefixes),
        "- **总判定: %s**" % ("✅ PASS" if passed else "❌ NEEDS WORK"),
        "",
        "## 验收门禁 (jihua20260905.md §12)",
        "",
        "| 指标 | 最差值 | 阈值 | 方向 | 结果 |",
        "|---|---|---|---|---|",
    ]
    for label, direction, worst, thr, unit, ok in gate_results:
        sym = "≥" if direction == "min" else "≤"
        lines.append("| %s | %s %s | %s %s | %s%s | %s |"
                     % (label, fmtv(worst), unit, thr, unit, sym, direction,
                        "PASS" if ok else "**FAIL**"))
    lines += ["", "## 多 seed 聚合统计", "",
              "| 指标 | mean | std | P50 | P95 | min | max |",
              "|---|---|---|---|---|---|---|"]
    labelmap = {
        "astar_rate_pct": "A* 成功率 %", "amcl_avg_err_m": "AMCL 平均误差 m",
        "amcl_max_err_m": "AMCL 峰值误差 m", "stuck_ratio_pct": "stuck %",
        "collisions": "碰撞", "rooms_visited": "房间覆盖",
        "total_distance_m": "运动距离 m", "avg_speed": "平均速度 m/s",
        "near_miss": "近距事件", "planning_failures": "规划失败",
    }
    for k in ["astar_rate_pct", "amcl_avg_err_m", "amcl_max_err_m", "stuck_ratio_pct",
              "collisions", "rooms_visited", "total_distance_m", "avg_speed",
              "near_miss", "planning_failures"]:
        st = stats.get(k)
        if not st:
            continue
        lines.append("| %s | %s | %s | %s | %s | %s | %s |"
                     % (labelmap.get(k, k), fmtv(st["mean"]), fmtv(st["std"]),
                        fmtv(st["p50"]), fmtv(st["p95"]), fmtv(st["min"]), fmtv(st["max"])))
    lines += ["", "## 逐种子原始指标", "",
              "见 `cpp_<prefix>_seed<n>.json` (%d 个文件已复制)。" % len(copied),
              "", "## 图表", "",
              "见 `plots/` (SVG, 浏览器直接打开): A*率 / AMCL误差 / stuck / 房间覆盖 / 碰撞 / 运动距离 / 单帧时延分布。",
              "若带 `--trace` 运行, 额外生成 trajectory.svg 与 error_timeseries.svg。"]
    with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    with open(os.path.join(out_dir, "environment.json"), "w", encoding="utf-8") as f:
        json.dump(env, f, indent=2, ensure_ascii=False)
    with open(os.path.join(out_dir, "metadata.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    with open(os.path.join(out_dir, "config_hashes.json"), "w", encoding="utf-8") as f:
        json.dump(hashes, f, indent=2, ensure_ascii=False)

    print("[gen_release_report] -> %s" % out_dir)
    print("  前缀: %s" % ", ".join("%s×%d" % (p, meta["seeds_per_prefix"][p]) for p in prefixes))
    print("  总判定: %s" % ("PASS" if passed else "NEEDS WORK"))
    for label, _, worst, thr, unit, ok in gate_results:
        print("    %s %s %s %s -> %s" % ("✓" if ok else "✗", label, fmtv(worst), unit, "PASS" if ok else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
