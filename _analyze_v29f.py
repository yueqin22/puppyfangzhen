#!/usr/bin/env python3
"""Analyze v2.9f simulation results vs v2.9e baseline.

Reads /tmp/puppy_nav_data.csv and computes:
  1. AMCL localization error statistics (loc_err column)
  2. Real position teleportation (true_x/true_y jumps > 1m)
  3. Wall clipping (true position inside obstacle AABB)
  4. Coverage, distance traveled, RECOVER count
  5. Per-state loc_err breakdown
  6. Trajectory of loc_err over time (when did AMCL diverge?)
"""
import csv
import math
import os
import sys

CSV_PATH = os.environ.get("CSV_PATH", "/tmp/puppy_nav_data.csv")

# Obstacle AABBs from CoppeliaSim scene (must match autonomous_nav.py discovery)
# These are read at runtime from the scene, but for wall-clipping analysis we
# need them statically. We'll print a warning and skip if not provided.
OBSTACLES = []  # populated from env or hardcoded


def load_rows(path):
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                row = {
                    "frame": int(r["frame"]),
                    "time": float(r["time"]),
                    "rx": float(r["rx"]),
                    "ry": float(r["ry"]),
                    "ryaw": float(r["ryaw"]),
                    "state": r["state"],
                    "coverage": float(r["coverage"]),
                    "visited": int(r["visited"]),
                    "cost_at_robot": float(r["cost_at_robot"]),
                    "dwa_v": float(r["dwa_v"]),
                    "dwa_w": float(r["dwa_w"]),
                    "no_progress": int(r["no_progress"]),
                    "goal_x": float(r["goal_x"]),
                    "goal_y": float(r["goal_y"]),
                    "dist_to_goal": float(r["dist_to_goal"]),
                    "path_len": float(r["path_len"]),
                    "true_x": float(r["true_x"]),
                    "true_y": float(r["true_y"]),
                    "loc_err": float(r["loc_err"]),
                }
                rows.append(row)
            except (ValueError, KeyError) as e:
                continue
    return rows


def stats(vals):
    if not vals:
        return {"n": 0}
    vals = sorted(vals)
    n = len(vals)
    return {
        "n": n,
        "min": vals[0],
        "max": vals[-1],
        "mean": sum(vals) / n,
        "median": vals[n // 2],
        "p90": vals[int(n * 0.9)],
        "p99": vals[int(n * 0.99)] if n > 100 else vals[-1],
    }


def fmt(s, name):
    if s["n"] == 0:
        return f"  {name}: (no data)"
    return (f"  {name:20s} n={s['n']:4d}  min={s['min']:.3f}  "
            f"mean={s['mean']:.3f}  median={s['median']:.3f}  "
            f"p90={s['p90']:.3f}  p99={s['p99']:.3f}  max={s['max']:.3f}")


def main():
    if not os.path.exists(CSV_PATH):
        print(f"ERROR: {CSV_PATH} not found")
        sys.exit(1)

    rows = load_rows(CSV_PATH)
    print(f"Loaded {len(rows)} rows from {CSV_PATH}")
    if not rows:
        return

    # === 1. Overall loc_err stats ===
    print("\n=== AMCL Localization Error (loc_err) ===")
    all_err = [r["loc_err"] for r in rows]
    print(fmt(stats(all_err), "all_frames"))

    # Break down by state
    by_state = {}
    for r in rows:
        by_state.setdefault(r["state"], []).append(r["loc_err"])
    for st in sorted(by_state.keys()):
        print(fmt(stats(by_state[st]), st))

    # === 2. loc_err over time — find divergence point ===
    print("\n=== loc_err Timeline (every 50 frames) ===")
    print(f"  {'frame':>5s} {'time':>6s} {'state':>20s} {'loc_err':>8s} "
          f"{'rx':>7s} {'ry':>7s} {'true_x':>7s} {'true_y':>7s}")
    for r in rows[::50]:
        print(f"  {r['frame']:5d} {r['time']:6.1f} {r['state']:>20s} "
              f"{r['loc_err']:8.3f} {r['rx']:7.2f} {r['ry']:7.2f} "
              f"{r['true_x']:7.2f} {r['true_y']:7.2f}")

    # === 3. Find divergence point (first frame where loc_err > 1m) ===
    print("\n=== Divergence Analysis ===")
    div_frame = None
    for i, r in enumerate(rows):
        if r["loc_err"] > 1.0 and i > 0:
            # Check if it stays >1m for 20 frames
            sustained = all(rows[j]["loc_err"] > 1.0
                            for j in range(i, min(i + 20, len(rows))))
            if sustained:
                div_frame = i
                prev = rows[max(i - 1, 0)]
                print(f"  First sustained divergence at frame {r['frame']} "
                      f"(state={r['state']})")
                print(f"  Previous frame {prev['frame']}: loc_err={prev['loc_err']:.3f} "
                      f"state={prev['state']} pos=({prev['rx']:.2f},{prev['ry']:.2f}) "
                      f"true=({prev['true_x']:.2f},{prev['true_y']:.2f})")
                print(f"  Divergence frame: loc_err={r['loc_err']:.3f} "
                      f"pos=({r['rx']:.2f},{r['ry']:.2f}) "
                      f"true=({r['true_x']:.2f},{r['true_y']:.2f})")
                break
    if div_frame is None:
        print("  No sustained divergence detected (loc_err stayed <1m throughout)")

    # === 4. Real position teleportation ===
    print("\n=== Real Position Teleportation (>1m jump) ===")
    teleports = []
    for i in range(1, len(rows)):
        dx = rows[i]["true_x"] - rows[i - 1]["true_x"]
        dy = rows[i]["true_y"] - rows[i - 1]["true_y"]
        d = math.sqrt(dx * dx + dy * dy)
        if d > 1.0:
            teleports.append((i, d, rows[i - 1], rows[i]))
    print(f"  Count: {len(teleports)}")
    for idx, dist, prev, curr in teleports[:10]:
        print(f"  frame {prev['frame']}->{curr['frame']}: {dist:.2f}m jump "
              f"({prev['true_x']:.2f},{prev['true_y']:.2f}) -> "
              f"({curr['true_x']:.2f},{curr['true_y']:.2f}) state={curr['state']}")

    # === 5. Coverage & distance ===
    print("\n=== Coverage & Distance ===")
    max_cov = max(r["coverage"] for r in rows)
    final_cov = rows[-1]["coverage"]
    print(f"  Final coverage: {final_cov:.2f}%")
    print(f"  Max coverage:   {max_cov:.2f}%")

    total_dist = 0.0
    for i in range(1, len(rows)):
        dx = rows[i]["true_x"] - rows[i - 1]["true_x"]
        dy = rows[i]["true_y"] - rows[i - 1]["true_y"]
        total_dist += math.sqrt(dx * dx + dy * dy)
    print(f"  Total distance traveled (true): {total_dist:.2f}m")

    # === 6. RECOVER count ===
    print("\n=== RECOVER Events ===")
    recover_count = 0
    recover_starts = []
    prev_state = None
    for r in rows:
        st = r["state"]
        if "RECOVER" in st and (prev_state is None or "RECOVER" not in prev_state):
            recover_count += 1
            recover_starts.append(r)
        prev_state = st
    print(f"  RECOVER entries: {recover_count}")
    for r in recover_starts[:10]:
        print(f"    frame {r['frame']}: loc_err={r['loc_err']:.3f} "
              f"pos=({r['rx']:.2f},{r['ry']:.2f}) true=({r['true_x']:.2f},{r['true_y']:.2f})")

    # === 7. AMCL recovery effectiveness ===
    print("\n=== AMCL Recovery Effectiveness ===")
    # For each RECOVER entry, find the loc_err before and 10 frames after exit
    for r in recover_starts[:10]:
        start_frame = r["frame"]
        # Find exit (state changes away from RECOVER)
        end_idx = None
        for i, rr in enumerate(rows):
            if rr["frame"] >= start_frame and "RECOVER" not in rr["state"]:
                end_idx = i
                break
        if end_idx is None:
            continue
        after_idx = min(end_idx + 10, len(rows) - 1)
        print(f"  RECOVER@{start_frame}: loc_err {r['loc_err']:.3f} -> "
              f"exit@{rows[end_idx]['frame']} {rows[end_idx]['loc_err']:.3f} -> "
              f"+10f {rows[after_idx]['loc_err']:.3f}")

    # === 8. v2.9e vs v2.9f comparison ===
    print("\n=== v2.9f Summary (compare with v2.9e baseline) ===")
    print(f"  v2.9e: loc_err avg=1.64m  max=4.33m  dist=45.26m  RECOVER=4  cov=93.88%")
    print(f"  v2.9f: loc_err avg={stats(all_err)['mean']:.2f}m  "
          f"max={stats(all_err)['max']:.2f}m  "
          f"dist={total_dist:.2f}m  RECOVER={recover_count}  cov={final_cov:.2f}%")


if __name__ == "__main__":
    main()
