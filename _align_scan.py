# -*- coding: utf-8 -*-
"""R28 scan-to-map alignment search (offline, no UE run needed).

The per-ray dump showed the measured ranges are roughly the RIGHT MAGNITUDE but at
the WRONG BEARINGS: the sensor reports a wall 0.4m away where the map expects open
space, and reports max-range where the map has a wall 0.4m away.

This script recovers, per dumped frame, the rigid transform (dx, dy, dyaw) that best
explains the measured scan against the map geometry. If dyaw is large and consistent,
the LiDAR frame is rotated with respect to the navigation frame -- a protocol bug,
not an AMCL tuning problem.
"""
import csv
import json
import math
import sys
from collections import defaultdict

RAYS = sys.argv[1] if len(sys.argv) > 1 else r"E:\puppyfangzhen\trace_val.csv.rays.csv"
SCENE = r"E:\puppyfangzhen\config\scene_home.json"
ZMAX = 8.0


def load_boxes():
    with open(SCENE, encoding="utf-8") as fh:
        js = json.load(fh)
    return [(o["xmin"], o["ymin"], o["xmax"], o["ymax"]) for o in js["obstacles"]]


BOXES = load_boxes()


def raycast(rx, ry, ang):
    dx, dy = math.cos(ang), math.sin(ang)
    best = ZMAX
    for (x0, y0, x1, y1) in BOXES:
        tmin, tmax = -1e18, 1e18
        if abs(dx) > 1e-9:
            t1, t2 = (x0 - rx) / dx, (x1 - rx) / dx
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
        elif rx < x0 or rx > x1:
            continue
        if abs(dy) > 1e-9:
            t1, t2 = (y0 - ry) / dy, (y1 - ry) / dy
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
        elif ry < y0 or ry > y1:
            continue
        te = max(0.0, tmin)
        if tmax >= te and te < best:
            best = te
    return best


def cost(rx, ry, yaw, rays):
    """Robust mean absolute range error, saturating at 1m so a few bad rays
    cannot dominate. Rays at max range on both sides count as agreement."""
    tot = 0.0
    for la, meas in rays:
        exp = raycast(rx, ry, yaw + la)
        if meas >= ZMAX - 0.05 and exp >= ZMAX - 0.05:
            continue
        tot += min(abs(meas - exp), 1.0)
    return tot / len(rays)


def main():
    with open(RAYS, newline="", encoding="utf-8", errors="replace") as fh:
        rows = list(csv.DictReader(fh))
    frames = defaultdict(list)
    for r in rows:
        frames[int(r["frame"])].append(r)

    print("%-7s %-22s %-9s %-28s %s" % ("frame", "gt pose", "cost@gt", "best (dx,dy,dyaw)", "cost@best"))
    print("-" * 96)
    yaws = []
    for f in sorted(frames):
        rs = frames[f]
        # recover the GT pose the dump was generated from
        r0 = rs[0]
        wa0 = float(r0["angle_world"])
        la0 = float(r0["angle_local"])
        gyaw = wa0 - la0
        gx = float(r0["hit_x"]) - float(r0["meas"]) * math.cos(wa0)
        gy = float(r0["hit_y"]) - float(r0["meas"]) * math.sin(wa0)
        rays = [(float(r["angle_local"]), float(r["meas"])) for r in rs]

        c_gt = cost(gx, gy, gyaw, rays)

        # 1) coarse yaw sweep at the GT position (2 deg steps)
        best = (1e18, 0.0)
        for i in range(-90, 90):
            dyaw = math.radians(i * 2.0)
            c = cost(gx, gy, gyaw + dyaw, rays)
            if c < best[0]:
                best = (c, dyaw)
        c_yaw, dyaw = best

        # 2) refine translation at the best yaw, then yaw again (2 passes)
        dx = dy = 0.0
        for _ in range(2):
            b = (c_yaw, dx, dy)
            for ix in range(-8, 9):
                for iy in range(-8, 9):
                    c = cost(gx + ix * 0.1, gy + iy * 0.1, gyaw + dyaw, rays)
                    if c < b[0]:
                        b = (c, ix * 0.1, iy * 0.1)
            c_yaw, dx, dy = b
            b2 = (c_yaw, dyaw)
            for i in range(-20, 21):
                cand = dyaw + math.radians(i * 1.0)
                c = cost(gx + dx, gy + dy, gyaw + cand, rays)
                if c < b2[0]:
                    b2 = (c, cand)
            c_yaw, dyaw = b2

        yaws.append(math.degrees(dyaw))
        print("%-7d (%+.2f,%+.2f,%+6.1fd)  %-9.3f (%+.2f,%+.2f,%+7.1fd)      %.3f"
              % (f, gx, gy, math.degrees(gyaw), c_gt, dx, dy, math.degrees(dyaw), c_yaw))

    print("\nrecovered yaw offset per frame (deg): %s"
          % ", ".join("%+.0f" % v for v in yaws))
    mean = sum(yaws) / len(yaws)
    print("mean %+.1f deg   spread %.1f deg" % (mean, max(yaws) - min(yaws)))
    print("\nInterpretation:")
    print("  |dyaw| small everywhere        -> no rotation bug; look elsewhere")
    print("  dyaw large and CONSTANT        -> fixed LiDAR mounting/index offset")
    print("  dyaw ~= -2*gt_yaw             -> yaw sign flip between UE and nav frames")


main()
