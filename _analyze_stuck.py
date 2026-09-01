#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R28b: why is the dog stuck 71% of the time now that the walls are real?

Localisation is fixed (avg_err 0.123 m).  The new failure is motion:
collisions on 2913/3600 frames, stuck_ratio 71.4%, rounds=0.
This script localises *where* and *why* the robot jams.
"""
import csv
import json
import math
from collections import Counter

TRACE = r"E:\puppyfangzhen\trace_val.csv"
SCENE = r"E:\puppyfangzhen\config\scene_home.json"

rows = list(csv.DictReader(open(TRACE)))
scene = json.load(open(SCENE))
obs = scene['obstacles']
R_PLAN = scene['robot']['radius_planning']

F = lambda r, k: float(r[k])


def nearest_obstacle(x, y):
    """Signed clearance to the closest obstacle AABB (negative = inside)."""
    best, bname = 1e9, ''
    for o in obs:
        dx = max(o['xmin'] - x, 0.0, x - o['xmax'])
        dy = max(o['ymin'] - y, 0.0, y - o['ymax'])
        if dx == 0.0 and dy == 0.0:                    # inside
            d = -min(x - o['xmin'], o['xmax'] - x, y - o['ymin'], o['ymax'] - y)
        else:
            d = math.hypot(dx, dy)
        if d < best:
            best, bname = d, o['name']
    return best, bname


print('=== motion profile ===')
moved = 0
disp = []
for i in range(1, len(rows)):
    d = math.hypot(F(rows[i], 'true_x') - F(rows[i - 1], 'true_x'),
                   F(rows[i], 'true_y') - F(rows[i - 1], 'true_y'))
    disp.append(d)
    if d > 1e-4:
        moved += 1
print('  frames with any displacement: %d / %d (%.1f%%)'
      % (moved, len(disp), 100.0 * moved / len(disp)))
print('  total path length: %.2f m   net displacement: %.2f m'
      % (sum(disp),
         math.hypot(F(rows[-1], 'true_x') - F(rows[0], 'true_x'),
                    F(rows[-1], 'true_y') - F(rows[0], 'true_y'))))

print()
print('=== collision hot spots (frames where the collision counter increments) ===')
hot = Counter()
inside = 0
for i in range(1, len(rows)):
    dc = int(rows[i]['collisions']) - int(rows[i - 1]['collisions'])
    if dc > 0:
        x, y = F(rows[i], 'true_x'), F(rows[i], 'true_y')
        clr, nm = nearest_obstacle(x, y)
        hot[nm] += dc
        if clr < 0:
            inside += 1
for nm, c in hot.most_common(10):
    print('  %-18s %5d collision events' % (nm, c))
print('  frames where GT is INSIDE an obstacle AABB: %d' % inside)

print()
print('=== clearance distribution (GT vs nearest obstacle surface) ===')
clr_all = [nearest_obstacle(F(r, 'true_x'), F(r, 'true_y'))[0] for r in rows]
buckets = [(-9, 0), (0, .05), (.05, .10), (.10, .20), (.20, .30), (.30, .50), (.50, 9)]
for lo, hi in buckets:
    n = sum(1 for c in clr_all if lo <= c < hi)
    tag = ' <-- INSIDE geometry' if hi == 0 else (
        '  <-- closer than radius_planning=%.2f' % R_PLAN if hi <= R_PLAN else '')
    print('  clearance [%5.2f,%5.2f) : %5d frames (%4.1f%%)%s'
          % (lo, hi, n, 100.0 * n / len(rows), tag))
print('  min clearance over run: %.3f m' % min(clr_all))

print()
print('=== goal progression ===')
seen = []
for r in rows:
    g = int(r['goal_idx'])
    if not seen or seen[-1][0] != g:
        seen.append((g, int(r['frame']), F(r, 'goal_x'), F(r, 'goal_y')))
for g, f, gx, gy in seen:
    print('  frame %5d -> goal #%d (%.2f, %.2f)' % (f, g, gx, gy))

print()
print('=== longest stall segments (no displacement) ===')
segs, run_start = [], None
for i in range(1, len(rows)):
    d = disp[i - 1]
    if d <= 1e-4:
        if run_start is None:
            run_start = i
    else:
        if run_start is not None and i - run_start > 60:
            segs.append((run_start, i - 1))
        run_start = None
if run_start is not None:
    segs.append((run_start, len(rows) - 1))
segs.sort(key=lambda s: s[0] - s[1])
for a, b in segs[:8]:
    r = rows[a]
    clr, nm = nearest_obstacle(F(r, 'true_x'), F(r, 'true_y'))
    print('  frames %5d-%5d (%5.1fs)  at (%+.2f,%+.2f) goal#%s (%.2f,%.2f)  '
          'clearance=%.3f to [%s]  cmd=(%.2f,%.2f,%.2f) path_pts=%s'
          % (a, b, (b - a) / 30.0, F(r, 'true_x'), F(r, 'true_y'), r['goal_idx'],
             F(r, 'goal_x'), F(r, 'goal_y'), clr, nm,
             F(r, 'cmd_vx'), F(r, 'cmd_vy'), F(r, 'cmd_wz'), r['path_pts']))
