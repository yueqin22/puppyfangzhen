# -*- coding: utf-8 -*-
"""R28 estimator-bias analysis.

Answers two questions with the per-frame trace produced by nav_ue_bridge --trace:

  Q1  Is the published estimate (largest-cluster mean) worse than the plain
      full-cloud weighted mean?  If yes, get_cluster_estimate() is manufacturing
      the systematic offset and the fix is to stop using it on unimodal clouds.

  Q2  Does the scan-likelihood optimum sit at the ground-truth pose?  If yes,
      sensor and map agree and the fault is purely in estimate extraction.
      If it sits off GT, scan and map genuinely disagree.
"""
import csv
import math
import sys
from collections import Counter

PATH = sys.argv[1] if len(sys.argv) > 1 else r"E:\puppyfangzhen\trace_val.csv"


def fnum(row, key, default=0.0):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def main():
    with open(PATH, newline="", encoding="utf-8", errors="replace") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print("empty trace")
        return

    need = {"mean_x", "top_x", "top_share", "score_gt", "best_dx"}
    if not need.issubset(rows[0].keys()):
        print("trace lacks R28 columns; rebuild the bridge and rerun")
        print("columns:", list(rows[0].keys()))
        return

    print("rows = %d   duration = %.1fs" % (len(rows), fnum(rows[-1], "t")))

    # ---------- Q1: published (cluster) estimate vs full-cloud mean ----------
    pub_err, mean_err, top_err, sep = [], [], [], []
    for r in rows:
        tx, ty = fnum(r, "true_x"), fnum(r, "true_y")
        pub_err.append(math.hypot(fnum(r, "est_x") - tx, fnum(r, "est_y") - ty))
        mean_err.append(math.hypot(fnum(r, "mean_x") - tx, fnum(r, "mean_y") - ty))
        top_err.append(math.hypot(fnum(r, "top_x") - tx, fnum(r, "top_y") - ty))
        sep.append(math.hypot(fnum(r, "top_x") - fnum(r, "mean_x"),
                              fnum(r, "top_y") - fnum(r, "mean_y")))

    def stat(name, v):
        print("  %-22s avg=%.3f  max=%.3f  p50=%.3f" % (
            name, sum(v) / len(v), max(v), sorted(v)[len(v) // 2]))

    print("\n=== Q1  estimator error vs ground truth (m) ===")
    stat("published est", pub_err)
    stat("full-cloud mean", mean_err)
    stat("largest-cluster mean", top_err)
    stat("|cluster - cloud mean|", sep)

    better = sum(1 for a, b in zip(mean_err, pub_err) if b - a > 0.05)
    print("  full-cloud mean beats published by >5cm on %d/%d frames (%.1f%%)"
          % (better, len(rows), 100.0 * better / len(rows)))

    print("\n  per-10s buckets (published / cloud-mean):")
    for b in range(0, int(fnum(rows[-1], "t")) + 1, 10):
        sel = [i for i, r in enumerate(rows) if b <= fnum(r, "t") < b + 10]
        if not sel:
            continue
        p = sum(pub_err[i] for i in sel) / len(sel)
        m = sum(mean_err[i] for i in sel) / len(sel)
        s = sum(sep[i] for i in sel) / len(sel)
        print("    %3ds-%3ds  pub=%.3f  cloud=%.3f  gain=%+.3f  sep=%.3f"
              % (b, b + 10, p, m, p - m, s))

    # ---------- cloud shape: how truncated is the largest cluster? ----------
    shares = [fnum(r, "top_share") for r in rows]
    ncl = Counter(int(fnum(r, "nclusters")) for r in rows)
    print("\n=== cloud shape ===")
    print("  top_share  avg=%.3f  min=%.3f  frames<0.80: %d (%.1f%%)"
          % (sum(shares) / len(shares), min(shares),
             sum(1 for s in shares if s < 0.80),
             100.0 * sum(1 for s in shares if s < 0.80) / len(shares)))
    print("  cluster count histogram: %s"
          % ", ".join("%d:%d" % kv for kv in sorted(ncl.items())))

    # ---------- Q2: likelihood-field grid search ----------
    gs = [r for r in rows if fnum(r, "score_best") > 0.0]
    print("\n=== Q2  scan-likelihood grid search around GT (%d samples) ===" % len(gs))
    if gs:
        bdx = [fnum(r, "best_dx") for r in gs]
        bdy = [fnum(r, "best_dy") for r in gs]
        dist = [math.hypot(a, b) for a, b in zip(bdx, bdy)]
        at_gt = sum(1 for d in dist if d < 0.15)
        print("  best-offset from GT:  mean=(%+.3f, %+.3f)  |mean|=%.3f"
              % (sum(bdx) / len(bdx), sum(bdy) / len(bdy),
                 math.hypot(sum(bdx) / len(bdx), sum(bdy) / len(bdy))))
        print("  |best-offset| avg=%.3f m   optimum within 15cm of GT: %d/%d (%.1f%%)"
              % (sum(dist) / len(dist), at_gt, len(gs), 100.0 * at_gt / len(gs)))
        sgt = [fnum(r, "score_gt") for r in gs]
        ses = [fnum(r, "score_est") for r in gs]
        sbe = [fnum(r, "score_best") for r in gs]
        print("  score  gt=%.4g   est=%.4g   best=%.4g"
              % (sum(sgt) / len(sgt), sum(ses) / len(ses), sum(sbe) / len(sbe)))
        gt_wins = sum(1 for a, b in zip(sgt, ses) if a > b)
        print("  GT scores higher than published est on %d/%d samples (%.1f%%)"
              % (gt_wins, len(gs), 100.0 * gt_wins / len(gs)))

    # ---------- first frames ----------
    print("\n=== first 5 frames ===")
    for r in rows[:5]:
        print("  f=%-4s true=(%+.3f,%+.3f) est=(%+.3f,%+.3f) cloud=(%+.3f,%+.3f) "
              "top=(%+.3f,%+.3f) share=%.2f ncl=%s"
              % (r["frame"], fnum(r, "true_x"), fnum(r, "true_y"),
                 fnum(r, "est_x"), fnum(r, "est_y"),
                 fnum(r, "mean_x"), fnum(r, "mean_y"),
                 fnum(r, "top_x"), fnum(r, "top_y"),
                 fnum(r, "top_share"), r["nclusters"]))


main()
