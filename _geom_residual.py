#!/usr/bin/env python3
"""P0-02 geometry double-source forensics.

For every LiDAR ray in the dump, take the MEASURED hit point (what UE really
has) and compute its signed distance to the nearest JSON obstacle surface.

  residual ~ 0            -> UE and JSON agree (ray explained)
  residual small positive -> UE box is slightly bigger / offset (systematic)
  residual large positive -> UE has geometry JSON does not know about at all

Usage: python _geom_residual.py [rays.csv]
"""
import csv
import json
import math
import sys
from collections import Counter

RAYS = sys.argv[1] if len(sys.argv) > 1 else "trace_val.csv.rays.csv"
SCENE = "config/scene_home.json"


def load_boxes():
    j = json.load(open(SCENE, encoding="utf-8"))
    return [(o["name"], o["xmin"], o["ymin"], o["xmax"], o["ymax"]) for o in j["obstacles"]]


def dist_to_box(px, py, b):
    """Distance from point to axis-aligned box; 0 if inside."""
    _, x0, y0, x1, y1 = b
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def main():
    boxes = load_boxes()
    rows = list(csv.DictReader(open(RAYS)))
    print("rays in dump: %d   json boxes: %d" % (len(rows), len(boxes)))

    buckets = Counter()
    unexplained = []
    for r in rows:
        meas = float(r["meas"])
        maxr = 12.0
        if meas >= maxr - 1e-3:
            buckets["no-return(maxrange)"] += 1
            continue
        hx, hy = float(r["hit_x"]), float(r["hit_y"])
        best = min(((dist_to_box(hx, hy, b), b[0]) for b in boxes), key=lambda t: t[0])
        d, name = best
        if d <= 0.06:
            buckets["explained(<=6cm)"] += 1
        elif d <= 0.15:
            buckets["inflated(6-15cm)"] += 1
        elif d <= 0.35:
            buckets["offset(15-35cm)"] += 1
        else:
            buckets["UNMAPPED(>35cm)"] += 1
            unexplained.append((float(r["frame"]), int(r["ray"]), hx, hy, d, name, meas,
                                float(r["expect"])))

    print("\n--- residual classification ---")
    tot = sum(buckets.values())
    for k in ["explained(<=6cm)", "inflated(6-15cm)", "offset(15-35cm)",
              "UNMAPPED(>35cm)", "no-return(maxrange)"]:
        n = buckets.get(k, 0)
        print("  %-22s %4d  %5.1f%%" % (k, n, 100.0 * n / tot))

    print("\n--- UNMAPPED hit points (UE geometry absent from JSON) ---")
    print("  %d rays" % len(unexplained))
    # cluster on a 0.4m grid so wall segments show up as runs
    cl = {}
    for f, ray, hx, hy, d, name, meas, exp in unexplained:
        key = (round(hx / 0.4) * 0.4, round(hy / 0.4) * 0.4)
        cl.setdefault(key, []).append((f, ray, hx, hy, d, meas))
    print("  %d clusters (0.4m grid), sorted by hit count:" % len(cl))
    for key in sorted(cl, key=lambda k: -len(cl[k])):
        pts = cl[key]
        frames = sorted({int(p[0]) for p in pts})
        mind = min(p[5] for p in pts)
        print("    (%+5.1f,%+5.1f) n=%2d  frames=%-22s closest_meas=%.3f"
              % (key[0], key[1], len(pts), str(frames), mind))

    # Is the unmapped geometry static in WORLD frame (real walls) or does it
    # follow the robot (self-hit)?  Check per-frame spread of one cluster.
    print("\n--- verdict inputs ---")
    world_static = sum(1 for k, v in cl.items() if len({int(p[0]) for p in v}) >= 2)
    print("  clusters seen in >=2 different frames: %d / %d" % (world_static, len(cl)))
    print("  => world-static clusters mean REAL stale level geometry, not self-hit")


if __name__ == "__main__":
    main()
