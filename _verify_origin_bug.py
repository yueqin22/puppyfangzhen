#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R28 root-cause verification.

Hypothesis: SpawnObstacleVisuals() spawns every obstacle visual at the WORLD
ORIGIN (0,0,0) instead of its intended centre, because a bare AActor has no
RootComponent so SpawnActor()'s transform is silently dropped.

Test: re-simulate the 72-beam scan by ray-casting the GT pose against boxes
that keep their correct SIZE but are all centred at the origin, with a Z-slab
filter for the scan plane. Compare against the measured scan.
"""
import csv
import json
import math

SCENE = r"E:\puppyfangzhen\config\scene_home.json"
RAYS = r"E:\puppyfangzhen\trace_val.csv.rays.csv"
TRACE = r"E:\puppyfangzhen\trace_val.csv"
MAXR = 8.0
SCAN_Z_CM = 35.0          # robot Z (15cm) + LiDAR elevation (+20cm)


def size_z_cm(name):
    """Mirror of the Sizez if/else ladder in PuppyRobotPawn.cpp."""
    n = name
    if 'wall' in n:            return 280.0
    if 'counter' in n:         return 90.0
    if 'bed' in n:             return 45.0
    if 'sofa' in n:            return 70.0
    if 'coffee_table' in n:    return 45.0
    if 'dining_table' in n:    return 75.0
    if 'chair' in n:           return 80.0
    if 'fridge' in n:          return 170.0
    if 'shower' in n or 'toilet' in n or 'sink' in n: return 80.0
    if 'bookshelf' in n:       return 180.0
    if 'wardrobe' in n:        return 200.0
    if 'desk' in n:            return 75.0
    if 'tv' in n:              return 50.0
    if 'nightstand' in n:      return 50.0
    if 'island' in n:          return 90.0
    return 60.0


def load_boxes():
    obs = json.load(open(SCENE))['obstacles']
    real, origin = [], []
    for o in obs:
        sx = max(0.01, o['xmax'] - o['xmin'])
        sy = max(0.01, o['ymax'] - o['ymin'])
        sz = size_z_cm(o['name'])
        real.append((o['xmin'], o['ymin'], o['xmax'], o['ymax'], o['name']))
        # Buggy placement: centred at origin, Z centre 0 -> top at sz/2 cm.
        if sz * 0.5 >= SCAN_Z_CM:      # only boxes tall enough to block the scan plane
            origin.append((-sx / 2, -sy / 2, sx / 2, sy / 2, o['name']))
    return real, origin


def raycast(px, py, ang, boxes):
    """Slab method; returns nearest hit distance along ang, or MAXR."""
    dx, dy = math.cos(ang), math.sin(ang)
    best = MAXR
    for (xmin, ymin, xmax, ymax, _n) in boxes:
        t0, t1 = 0.0, best
        for p, d, lo, hi in ((px, dx, xmin, xmax), (py, dy, ymin, ymax)):
            if abs(d) < 1e-9:
                if p < lo or p > hi:
                    t0 = t1 + 1.0
                    break
            else:
                ta, tb = (lo - p) / d, (hi - p) / d
                if ta > tb:
                    ta, tb = tb, ta
                t0 = max(t0, ta)
                t1 = min(t1, tb)
        if t0 <= t1 and t0 > 1e-6:
            best = min(best, t0)
    return best


def main():
    real, origin = load_boxes()
    print('boxes at correct pose: %d   boxes tall enough at origin: %d'
          % (len(real), len(origin)))
    print('  origin-stacked, scan-blocking:', ', '.join(b[4] for b in origin))
    print()

    gt = {}
    for r in csv.DictReader(open(TRACE)):
        gt[int(r['frame'])] = (float(r['true_x']), float(r['true_y']), float(r['true_yaw']))

    rows = list(csv.DictReader(open(RAYS)))
    frames = sorted({int(r['frame']) for r in rows})
    print('%-7s %-28s %-28s' % ('frame', 'model = REAL map', 'model = ALL AT ORIGIN'))
    print('%-7s %-28s %-28s' % ('', 'rms / #match(<0.1m)', 'rms / #match(<0.1m)'))
    tot_r = tot_o = 0.0
    for f in frames:
        if f not in gt:
            continue
        gx, gy, gyaw = gt[f]
        rs = [r for r in rows if int(r['frame']) == f]
        er = eo = 0.0
        mr = mo = 0
        for r in rs:
            meas = float(r['meas'])
            a = gyaw + float(r['angle_local'])
            dr = raycast(gx, gy, a, real)
            do = raycast(gx, gy, a, origin)
            er += (meas - dr) ** 2
            eo += (meas - do) ** 2
            if abs(meas - dr) < 0.1: mr += 1
            if abs(meas - do) < 0.1: mo += 1
        n = len(rs)
        er, eo = math.sqrt(er / n), math.sqrt(eo / n)
        tot_r += er; tot_o += eo
        print('%-7d %6.3f m / %2d of %d          %6.3f m / %2d of %d'
              % (f, er, mr, n, eo, mo, n))
    k = len(frames)
    print()
    print('MEAN rms:  real map = %.3f m     all-at-origin = %.3f m' % (tot_r / k, tot_o / k))


if __name__ == '__main__':
    main()
