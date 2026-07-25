#!/usr/bin/env python3
"""Long-run analysis: wall clipping, teleportation, AMCL drift over time."""
import csv
import math

rows = list(csv.DictReader(open('/tmp/puppy_nav_data.csv')))
for r in rows:
    for k in ['frame', 'time', 'rx', 'ry', 'ryaw', 'coverage', 'visited',
              'cost_at_robot', 'dwa_v', 'dwa_w', 'no_progress', 'goal_x',
              'goal_y', 'dist_to_goal', 'path_len', 'true_x', 'true_y', 'loc_err']:
        r[k] = float(r[k])
    r['frame'] = int(r['frame'])

n = len(rows)
print(f"=== Long-Run Analysis: {n} frames ({n/10:.0f}s = {n/600:.1f}min) ===\n")

# 1. loc_err over time (per 5000 frames)
print("=== 1. loc_err Timeline (per 5000 frames) ===")
for start in range(0, n, 5000):
    chunk = rows[start:start+5000]
    if not chunk: break
    errs = [r['loc_err'] for r in chunk]
    avg = sum(errs)/len(errs)
    mx = max(errs)
    p99 = sorted(errs)[int(len(errs)*0.99)]
    print(f"  frame {chunk[0]['frame']:6d}-{chunk[-1]['frame']:6d}: "
          f"avg={avg:.3f} p99={p99:.3f} max={mx:.3f}")

# 2. Teleportation check (true position jumps >1m)
print("\n=== 2. Real Position Teleportation (>1m jump) ===")
teleports = []
for i in range(1, n):
    dx = rows[i]['true_x'] - rows[i-1]['true_x']
    dy = rows[i]['true_y'] - rows[i-1]['true_y']
    dist = math.sqrt(dx*dx + dy*dy)
    if dist > 1.0:
        teleports.append((rows[i]['frame'], dist,
                          rows[i-1]['true_x'], rows[i-1]['true_y'],
                          rows[i]['true_x'], rows[i]['true_y']))
print(f"  Count: {len(teleports)}")
for f, d, x1, y1, x2, y2 in teleports[:10]:
    print(f"  frame {f}: jump {d:.2f}m ({x1:.1f},{y1:.1f}) -> ({x2:.1f},{y2:.1f})")

# 3. AMCL vs True position divergence
print("\n=== 3. AMCL Divergence (loc_err > 0.5m) ===")
divergences = [r for r in rows if r['loc_err'] > 0.5]
print(f"  Frames with loc_err > 0.5m: {len(divergences)} ({len(divergences)/n*100:.1f}%)")
divergences_1m = [r for r in rows if r['loc_err'] > 1.0]
print(f"  Frames with loc_err > 1.0m: {len(divergences_1m)} ({len(divergences_1m)/n*100:.1f}%)")

# 4. Wall clipping check (true position inside obstacle bbox)
# Load obstacles
import sys
sys.path.insert(0, '/mnt/e/puppyfangzhen')
# Hardcode known obstacle bboxes from sim
obstacles_bbox = [
    # These will be loaded from the log
]
# Parse obstacle bboxes from log
import re
with open('/tmp/v29j_long_run.log') as f:
    for line in f:
        m = re.match(r'.*bbox x=\[(-?\d+\.\d+),(-?\d+\.\d+)\] y=\[(-?\d+\.\d+),(-?\d+\.\d+)\]', line)
        if m:
            xmin, xmax, ymin, ymax = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
            obstacles_bbox.append((xmin, ymin, xmax, ymax))

print(f"\n=== 4. Wall Clipping (true position inside obstacle) ===")
print(f"  Loaded {len(obstacles_bbox)} obstacle bboxes")
wall_clips = []
radius = 0.25  # robot radius
for r in rows:
    for xmin, ymin, xmax, ymax in obstacles_bbox:
        if (r['true_x'] > xmin - radius and r['true_x'] < xmax + radius and
            r['true_y'] > ymin - radius and r['true_y'] < ymax + radius):
            wall_clips.append((r['frame'], r['true_x'], r['true_y']))
            break
print(f"  Wall clipping events: {len(wall_clips)}")
for f, x, y in wall_clips[:10]:
    print(f"  frame {f}: true pos ({x:.2f},{y:.2f}) inside obstacle")

# 5. Coverage and distance
print(f"\n=== 5. Coverage & Distance ===")
print(f"  Final coverage: {rows[-1]['coverage']:.2f}%")
print(f"  Max coverage: {max(r['coverage'] for r in rows):.2f}%")
total_dist = sum(math.sqrt(
    (rows[i]['true_x']-rows[i-1]['true_x'])**2 +
    (rows[i]['true_y']-rows[i-1]['true_y'])**2
) for i in range(1, n))
print(f"  Total distance: {total_dist:.1f}m")

# 6. RECOVER analysis
print(f"\n=== 6. RECOVER Analysis ===")
recovers = []
prev_state = None
for r in rows:
    if 'RECOVER' in r['state'] and (prev_state is None or 'RECOVER' not in prev_state):
        recovers.append(r)
    prev_state = r['state']
print(f"  RECOVER entries: {len(recovers)}")
if recovers:
    # RECOVER loc_err distribution
    r_errs = [r['loc_err'] for r in recovers]
    print(f"  loc_err at RECOVER: avg={sum(r_errs)/len(r_errs):.3f} "
          f"max={max(r_errs):.3f} min={min(r_errs):.3f}")
    # RECOVER by position
    print(f"  RECOVER positions (sampled):")
    for i in range(0, min(20, len(recovers)), max(1, len(recovers)//20)):
        r = recovers[i]
        print(f"    frame {r['frame']:6d}: ({r['true_x']:.1f},{r['true_y']:.1f}) "
              f"loc_err={r['loc_err']:.3f}")

# 7. State distribution
print(f"\n=== 7. State Distribution ===")
states = {}
for r in rows:
    st = r['state'].replace('NavState.', '')
    states[st] = states.get(st, 0) + 1
for st, cnt in sorted(states.items(), key=lambda x: -x[1]):
    print(f"  {st:12s}: {cnt:6d} ({cnt/n*100:.1f}%)")

# 8. Speed analysis
print(f"\n=== 8. Speed Analysis ===")
speeds = [math.sqrt(
    (rows[i]['true_x']-rows[i-1]['true_x'])**2 +
    (rows[i]['true_y']-rows[i-1]['true_y'])**2
) * 10 for i in range(1, n)]  # m/s
static_count = sum(1 for s in speeds if s < 0.05)
print(f"  Static frames (v<0.05): {static_count} ({static_count/len(speeds)*100:.1f}%)")
print(f"  Mean speed: {sum(speeds)/len(speeds):.3f} m/s")
print(f"  Max speed: {max(speeds):.3f} m/s")
