# -*- coding: utf-8 -*-
"""R28 per-ray discrepancy analysis.

The grid search showed the scan's best-matching pose is never the true pose, so
the LiDAR and the occupancy grid disagree. This script says WHERE they disagree:

  diff = meas - expect
    diff << 0  : sensor sees something the map lacks (phantom obstacle)
    diff >> 0  : map has something the sensor cannot see (missed geometry)

For phantom rays we also check whether the measured hit point coincides with a
pedestrian (pedestrians are physical in UE but absent from the map).
"""
import csv
import math
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else r"E:\puppyfangzhen\trace_val.csv.rays.csv"
TOL = 0.30      # m, tolerance before calling a ray inconsistent
PED_TOL = 0.55  # m, hit point within this distance of a pedestrian = explained


def main():
    with open(PATH, newline="", encoding="utf-8", errors="replace") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print("empty ray dump")
        return

    frames = defaultdict(list)
    for r in rows:
        frames[int(r["frame"])].append({k: float(v) for k, v in r.items()})

    print("frames = %s   rays/frame = %d\n"
          % (sorted(frames), len(next(iter(frames.values())))))

    grand_short = grand_long = grand_ok = grand_ped = 0

    for f in sorted(frames):
        rs = frames[f]
        short = [r for r in rs if r["diff"] < -TOL]
        long_ = [r for r in rs if r["diff"] > TOL]
        ok = len(rs) - len(short) - len(long_)
        ped_expl = [r for r in short if 0.0 <= r["ped_dist"] <= PED_TOL]

        grand_short += len(short)
        grand_long += len(long_)
        grand_ok += ok
        grand_ped += len(ped_expl)

        rms = math.sqrt(sum(r["diff"] ** 2 for r in rs) / len(rs))
        print("frame %-5d ok=%2d  short=%2d (ped-explained %2d)  long=%2d  rms_diff=%.3f m"
              % (f, ok, len(short), len(ped_expl), len(long_), rms))

        # the worst offenders, with where they land in world coordinates
        worst = sorted(rs, key=lambda r: r["diff"])[:6]
        for r in worst:
            if r["diff"] >= -TOL:
                break
            tag = "PED" if 0.0 <= r["ped_dist"] <= PED_TOL else "???"
            print("    ray%-3d ang=%+6.1f deg  meas=%5.2f expect=%5.2f diff=%+5.2f "
                  "hit=(%+6.2f,%+6.2f) ped_d=%5.2f  %s"
                  % (r["ray"], math.degrees(r["angle_local"]), r["meas"],
                     r["expect"], r["diff"], r["hit_x"], r["hit_y"],
                     r["ped_dist"], tag))
        worst_long = sorted(rs, key=lambda r: -r["diff"])[:4]
        for r in worst_long:
            if r["diff"] <= TOL:
                break
            print("    ray%-3d ang=%+6.1f deg  meas=%5.2f expect=%5.2f diff=%+5.2f "
                  "MISSED-MAP-GEOMETRY"
                  % (r["ray"], math.degrees(r["angle_local"]), r["meas"],
                     r["expect"], r["diff"]))

    total = grand_ok + grand_short + grand_long
    print("\n=== totals over %d rays ===" % total)
    print("  consistent          : %4d (%.1f%%)" % (grand_ok, 100.0 * grand_ok / total))
    print("  phantom (too short) : %4d (%.1f%%)  of which pedestrian-explained %d"
          % (grand_short, 100.0 * grand_short / total, grand_ped))
    print("  missed  (too long)  : %4d (%.1f%%)" % (grand_long, 100.0 * grand_long / total))

    unexplained = grand_short - grand_ped
    if unexplained > 0:
        print("\n  %d phantom rays are NOT explained by pedestrians -> unmapped"
              " geometry in the UE level. Their hit points:" % unexplained)
        pts = [(r["hit_x"], r["hit_y"]) for f in sorted(frames) for r in frames[f]
               if r["diff"] < -TOL and not (0.0 <= r["ped_dist"] <= PED_TOL)]
        # coarse spatial histogram on a 0.5m grid to reveal clusters
        buckets = defaultdict(int)
        for x, y in pts:
            buckets[(round(x * 2) / 2, round(y * 2) / 2)] += 1
        for (bx, by), c in sorted(buckets.items(), key=lambda kv: -kv[1])[:15]:
            print("    (%+5.1f,%+5.1f)  x%d" % (bx, by, c))


main()
