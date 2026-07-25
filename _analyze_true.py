#!/usr/bin/env python3
"""Analyze true position vs AMCL position for wall-clipping and teleportation."""
import csv
import math

OBSTACLES = [
    ('wall_south',    -5.00, -4.05,  5.00, -3.95),
    ('wall_north',    -5.00,  3.95,  5.00,  4.05),
    ('wall_west',     -5.05, -4.00, -4.95,  4.00),
    ('wall_east',      4.95, -4.00,  5.05,  4.00),
    ('wall_divide_1', -5.00, -0.05, -1.00,  0.05),
    ('wall_divide_2',  1.00, -0.05,  5.00,  0.05),
    ('wall_bedroom_1',-5.05,  0.00, -4.95,  4.00),
    ('wall_kitchen_1', 4.95,  0.00,  5.05,  4.00),
    ('sofa',           2.75, -3.30,  4.25, -2.70),
    ('bed',           -4.25,  2.00, -2.75,  4.00),
    ('dining_table',   1.60,  2.48,  2.40,  2.52),
]
R = 0.35


def check(x, y):
    hits = []
    for name, xmin, ymin, xmax, ymax in OBSTACLES:
        if (xmin - R) <= x <= (xmax + R) and (ymin - R) <= y <= (ymax + R):
            dx = min(abs(x - xmin), abs(x - xmax))
            dy = min(abs(y - ymin), abs(y - ymax))
            hits.append((name, min(dx, dy)))
            if xmin <= x <= xmax and ymin <= y <= ymax:
                hits.append((name + '_CENTER', min(dx, dy)))
    return hits


with open('/tmp/puppy_nav_data.csv') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

print('=== True Position Teleportation Events (>1m jump) ===')
print('frame    | from -> to                       | dist  | state_change       | loc_err')
print('-' * 95)
count = 0
for i in range(1, len(rows)):
    x1, y1 = float(rows[i - 1]['true_x']), float(rows[i - 1]['true_y'])
    x2, y2 = float(rows[i]['true_x']), float(rows[i]['true_y'])
    d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    if d > 1.0:
        count += 1
        f1, f2 = int(rows[i - 1]['frame']), int(rows[i]['frame'])
        s1 = rows[i - 1]['state'].replace('NavState.', '')
        s2 = rows[i]['state'].replace('NavState.', '')
        le = float(rows[i]['loc_err'])
        print('%4d->%-4d | (%6.2f,%6.2f) -> (%6.2f,%6.2f) | %5.2fm | %-10s->%-10s | %.2f'
              % (f1, f2, x1, y1, x2, y2, d, s1, s2, le))

print('\nTotal: %d teleportation events' % count)

# loc_err statistics
loc_errs = [float(r['loc_err']) for r in rows]
avg_le = sum(loc_errs) / len(loc_errs)
max_le = max(loc_errs)
gt1 = sum(1 for e in loc_errs if e > 1.0)
gt2 = sum(1 for e in loc_errs if e > 2.0)
gt3 = sum(1 for e in loc_errs if e > 3.0)
print('\n=== AMCL Localization Error Statistics ===')
print('Average loc_err: %.2fm' % avg_le)
print('Max loc_err: %.2fm' % max_le)
print('Frames with loc_err > 1.0m: %d/600 (%.1f%%)' % (gt1, gt1 / 6.0))
print('Frames with loc_err > 2.0m: %d/600 (%.1f%%)' % (gt2, gt2 / 6.0))
print('Frames with loc_err > 3.0m: %d/600 (%.1f%%)' % (gt3, gt3 / 6.0))

# RECOVER state teleportation analysis
print('\n=== RECOVER State Analysis ===')
recover_frames = [(i, r) for i, r in enumerate(rows)
                  if 'RECOVER' in r['state']]
print('RECOVER frames: %d/600' % len(recover_frames))
recover_teleports = 0
for i, r in recover_frames:
    if i > 0:
        x1, y1 = float(rows[i - 1]['true_x']), float(rows[i - 1]['true_y'])
        x2, y2 = float(r['true_x']), float(r['true_y'])
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if d > 1.0:
            recover_teleports += 1
print('RECOVER teleports (>1m): %d' % recover_teleports)
