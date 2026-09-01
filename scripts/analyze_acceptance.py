#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze a black-box UE validation run against the commercial acceptance
criteria in guihua20260812.md  (section 16).

Inputs (produced by run_ue_validation.py):
  - bridge_val.log   : contains one or more "SUMMARY:" lines
  - trace_val.csv    : per-frame diagnostic trace (optional)

Outputs:
  - prints a PASS/FAIL table vs the section-16 thresholds
  - writes artifacts/<run_id>/metrics.json when --out is given
"""
import argparse
import csv
import json
import math
import os
import re
import sys

# R33: the UE collision body is a capsule of this radius (cm -> m).
#   PuppyRobotPawn.cpp:136  CollisionCapsule->SetCapsuleRadius(20.0f)
#   PuppyRobotPawn.cpp      static constexpr float ROBOT_RADIUS_CM = 20.0f
# Keep in sync with the UE source; the out-of-band check below is worthless if
# it verifies a radius the simulator does not use.
BODY_RADIUS_M = 0.20

# Acceptance thresholds from plan section 16.
THRESH = {
    "collisions":         (0,    "==", "重大碰撞：0"),
    "ue_collision_events":(0,    "==", "碰撞事件（UE 侧）：0"),
    "astar_rate":         (99.0, ">=", "静态可达目标规划成功率 >= 99%"),
    "avg_err":            (0.25, "<=", "动态场景平均定位误差 <= 0.25m (静态 <=0.10)"),
    "stuck_ratio":        (10.0, "<=", "卡住比例 <= 10% (目标 <=5%)"),
    "boundary_frames":    (0,    "==", "越界事件：0"),
}

SUMMARY_RE = re.compile(
    r"SUMMARY:\s*collisions=(?P<collisions>\d+)\s+"
    r"ue_collision_events=(?P<ue_collision_events>\d+)\s+"
    r"astar_rate=(?P<astar_rate>[\d.]+)\s+"
    r"avg_err=(?P<avg_err>[\d.]+)\s+"
    r"max_err=(?P<max_err>[\d.]+)\s+"
    r"rooms=(?P<rooms>\d+)\s+"
    r"rounds=(?P<rounds>\d+)\s+"
    r"targets=(?P<targets>\d+)\s+"
    # R33: optional so pre-R33 logs still parse.
    r"(?:targets_abandoned=(?P<targets_abandoned>\d+)\s+)?"
    r"gt_distance=(?P<gt_distance>[\d.]+)\s+"
    r"motion_frames=(?P<motion_frames>\d+)\s+"
    r"stuck_frames=(?P<stuck_frames>\d+)\s+"
    r"stuck_ratio=(?P<stuck_ratio>[\d.]+)\s+"
    r"boundary_frames=(?P<boundary_frames>\d+)\s+"
    r"boundary_ratio=(?P<boundary_ratio>[\d.]+)\s+"
    r"confidence=(?P<confidence>[\d.]+)\s+"
    r"frames=(?P<frames>\d+)"
)


def parse_summaries(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = SUMMARY_RE.search(line)
            if m:
                out.append({k: (None if v is None
                                else float(v) if "." in v else int(v))
                            for k, v in m.groupdict().items()})
    return out


def audit_clearance(trace_path, scene_path):
    """Out-of-band contact audit.

    Recomputes the body-surface clearance for every logged ground-truth pose
    directly from the scene definition, using exact point-to-AABB Euclidean
    distance.  This deliberately shares NO code with the simulator: if the UE
    collision predicate is ever wrong again (it was -- it grew obstacle boxes by
    the robot radius on all four sides, i.e. a Minkowski sum with a square
    rather than a disc, over-reporting contact by up to sqrt(2)x at convex
    corners), this check is what catches it.

    Returns None when inputs are unavailable, else a dict.
    """
    if not (os.path.exists(trace_path) and os.path.exists(scene_path)):
        return None
    with open(scene_path, encoding="utf-8") as f:
        scene = json.load(f)
    boxes = [(o["name"], o["xmin"], o["ymin"], o["xmax"], o["ymax"])
             for o in scene.get("obstacles", [])]
    if not boxes:
        return None

    worst = (float("inf"), None, 0.0, 0.0, -1)
    contact_frames = 0
    total = 0
    with open(trace_path, newline="", encoding="utf-8", errors="replace") as f:
        for row in csv.DictReader(f):
            try:
                px = float(row["true_x"])
                py = float(row["true_y"])
                frame = int(row["frame"])
            except (KeyError, TypeError, ValueError):
                continue
            total += 1
            best = (float("inf"), None)
            for name, x0, y0, x1, y1 in boxes:
                dx = max(x0 - px, 0.0, px - x1)
                dy = max(y0 - py, 0.0, py - y1)
                d = math.hypot(dx, dy)
                if d < best[0]:
                    best = (d, name)
            clearance = best[0] - BODY_RADIUS_M
            if clearance < 0.0:
                contact_frames += 1
            if clearance < worst[0]:
                worst = (clearance, best[1], px, py, frame)

    if total == 0:
        return None
    return {
        "frames_audited": total,
        "body_radius_m": BODY_RADIUS_M,
        "min_body_clearance_m": round(worst[0], 4),
        "min_at": {"frame": worst[4], "x": round(worst[2], 3),
                   "y": round(worst[3], 3), "obstacle": worst[1]},
        "contact_frames": contact_frames,
        "contact_ratio_pct": round(100.0 * contact_frames / total, 3),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge-log", default="bridge_val.log")
    ap.add_argument("--trace", default="trace_val.csv")
    ap.add_argument("--run-id", default="latest")
    ap.add_argument("--out", default=None, help="write metrics.json to this dir")
    ap.add_argument("--scene", default="config/scene_home.json",
                    help="scene definition used for the out-of-band audit")
    args = ap.parse_args()

    summaries = parse_summaries(args.bridge_log)
    if not summaries:
        print("[FAIL] no SUMMARY line found in %s" % args.bridge_log)
        sys.exit(2)

    # The run's final summary is the one with the most frames.
    s = max(summaries, key=lambda d: d["frames"])

    print("=" * 64)
    print("BLACK-BOX ACCEPTANCE  (plan section 16)   run=%s" % args.run_id)
    print("=" * 64)
    results = []
    for key, (limit, op, desc) in THRESH.items():
        val = s[key]
        if op == "==":
            ok = (val == limit)
        elif op == ">=":
            ok = (val >= limit)
        elif op == "<=":
            ok = (val <= limit)
        else:
            ok = False
        results.append((key, val, limit, op, ok, desc))
        print("  [%s] %-22s = %s  (need %s %s)  %s"
              % ("PASS" if ok else "FAIL", key, val, op, limit, desc))

    # 目标完成率 = 到达数 /（到达数 + 判定不可达而放弃数）
    #
    # R33: the old formula was reached/(rounds*5).  Since rounds = floor(
    # reached/5), the denominator can never exceed the numerator, so the check
    # was vacuous -- it reported 113.3% on the r33 run and would have reported
    # >= 100% for any run whatsoever, including one that abandoned every target
    # after the first loop.  An acceptance check that cannot fail is worse than
    # no check: it manufactures confidence.  Completion is now measured against
    # what the robot actually attempted.
    abandoned = s.get("targets_abandoned")
    if abandoned is None:
        print("  [WARN] %-20s 该 SUMMARY 无 targets_abandoned 字段（R33 前的日志），"
              "完成率退化为不可失败的旧口径，仅供参考" % "target_completion")
        expected = s["rounds"] * 5
        completion = (100.0 * s["targets"] / expected) if expected > 0 else 0.0
        detail = "targets=%d / rounds*5=%d [旧口径]" % (int(s["targets"]), expected)
    else:
        attempted = s["targets"] + abandoned
        completion = (100.0 * s["targets"] / attempted) if attempted > 0 else 0.0
        detail = "reached=%d / attempted=%d (abandoned=%d)" % (
            int(s["targets"]), int(attempted), int(abandoned))
    ok_c = completion >= 95.0
    results.append(("target_completion", round(completion, 1), 95.0, ">=", ok_c,
                    "目标完成率 >= 95%"))
    print("  [%s] %-22s = %.1f%%  (%s)  %s"
          % ("PASS" if ok_c else "FAIL", "target_completion", completion,
             detail, "目标完成率 >= 95%"))

    # ---- out-of-band contact audit (does not trust the simulator's counter) --
    audit = audit_clearance(args.trace, args.scene)
    if audit is None:
        print("  [WARN] out-of-band clearance audit skipped "
              "(missing %s or %s)" % (args.trace, args.scene))
    else:
        ok_clr = audit["contact_frames"] == 0
        results.append(("min_body_clearance", audit["min_body_clearance_m"],
                        0.0, ">", ok_clr,
                        "带外核验：体表净空全程 > 0（机身半径 %.2fm）"
                        % audit["body_radius_m"]))
        print("  [%s] %-22s = %+.3f m  (need > 0, body r=%.2fm)  最紧于 f=%d "
              "(%.2f,%.2f) near=%s"
              % ("PASS" if ok_clr else "FAIL", "min_body_clearance",
                 audit["min_body_clearance_m"], audit["body_radius_m"],
                 audit["min_at"]["frame"], audit["min_at"]["x"],
                 audit["min_at"]["y"], audit["min_at"]["obstacle"]))
        print("       contact_frames=%d/%d (%.3f%%) — 独立于 UE 自报计数"
              % (audit["contact_frames"], audit["frames_audited"],
                 audit["contact_ratio_pct"]))

    passed = sum(1 for r in results if r[4])
    total = len(results)
    overall = passed == total
    print("-" * 64)
    print("  OVERALL: %d/%d checks passed  ->  %s"
          % (passed, total, "ACCEPT (commercial candidate)" if overall
             else "NEEDS WORK"))
    print("=" * 64)

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        payload = {
            "run_id": args.run_id,
            "summary": s,
            "target_completion_pct": round(completion, 1),
            "checks": [
                {"metric": r[0], "value": r[1], "limit": r[2],
                 "op": r[3], "pass": r[4], "desc": r[5]} for r in results
            ],
            "overall_pass": overall,
            "clearance_audit": audit,
        }
        with open(os.path.join(args.out, "metrics.json"), "w",
                  encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        print("  metrics.json -> %s" % os.path.join(args.out, "metrics.json"))

    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
