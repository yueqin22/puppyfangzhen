#!/usr/bin/env python3
"""Deep analysis of v2.9g 2000-frame run to find remaining issues.

Focus areas:
  1. Coverage stagnation (91.9% from frame 750 to 2000)
  2. Frequent RECOVER (13 times, 12 from no_progress)
  3. RECOVER causing loc_err spikes
  4. Frontier management (214 unreachable frontiers cleared)
  5. State distribution and time wasted
"""
import csv
import math

rows = list(csv.DictReader(open('/tmp/puppy_nav_data.csv')))
for r in rows:
    for k in ['frame', 'time', 'rx', 'ry', 'ryaw', 'coverage', 'visited',
              'cost_at_robot', 'dwa_v', 'dwa_w', 'no_progress', 'goal_x',
              'goal_y', 'dist_to_goal', 'path_len', 'true_x', 'true_y', 'loc_err']:
        r[k] = float(r[k])
    r['frame'] = int(r['frame'])

print(f"=== 1. Coverage Stagnation Analysis ===")
cov_changes = []
prev_cov = 0
for r in rows:
    if r['coverage'] > prev_cov + 0.5:
        cov_changes.append((r['frame'], r['coverage'], r['true_x'], r['true_y'], r['state']))
        prev_cov = r['coverage']
print(f"Coverage increased {len(cov_changes)} times (>0.5% jumps):")
for f, c, x, y, st in cov_changes[:5]:
    print(f"  frame {f:4d}: cov={c:.1f}% at ({x:.1f},{y:.1f}) state={st}")
print("  ...")
for f, c, x, y, st in cov_changes[-5:]:
    print(f"  frame {f:4d}: cov={c:.1f}% at ({x:.1f},{y:.1f}) state={st}")

# When did coverage stop growing?
last_growth = 0
for r in rows:
    if r['coverage'] > rows[last_growth]['coverage']:
        last_growth = r['frame']
print(f"\nLast coverage growth at frame {last_growth} (cov={rows[last_growth]['coverage']:.2f}%)")
print(f"Stagnation: {2000 - last_growth} frames ({(2000-last_growth)*0.1:.0f}s) with no new area")

print(f"\n=== 2. RECOVER Pattern Analysis ===")
recovers = []
prev_state = None
for r in rows:
    if 'RECOVER' in r['state'] and (prev_state is None or 'RECOVER' not in prev_state):
        recovers.append(r)
    prev_state = r['state']

print(f"Total RECOVER entries: {len(recovers)}")
no_prog_recovers = [r for r in recovers if r['no_progress'] >= 40]
print(f"  From no_progress (>=40): {len(no_prog_recovers)}")

# Check what happens before each no_progress RECOVER
print("\n  RECOVER triggers (no_progress before entry):")
for r in recovers:
    idx = r['frame'] - 1
    if idx < len(rows):
        prev = rows[idx]
        print(f"  frame {r['frame']:4d}: no_progress={r['no_progress']:.0f} "
              f"pos=({r['true_x']:.1f},{r['true_y']:.1f}) "
              f"goal=({r['goal_x']:.1f},{r['goal_y']:.1f}) "
              f"dist={r['dist_to_goal']:.1f}m "
              f"v={r['dwa_v']:.2f} w={r['dwa_w']:.2f} "
              f"cost={r['cost_at_robot']:.0f}")

print(f"\n=== 3. RECOVER loc_err Impact ===")
for r in recovers:
    f = r['frame']
    # Find 5 frames before and after RECOVER entry
    before = rows[max(f-6, 0)]['loc_err']
    during = r['loc_err']
    # Find RECOVER exit
    exit_idx = None
    for i, rr in enumerate(rows):
        if rr['frame'] > f and 'RECOVER' not in rr['state']:
            exit_idx = i
            break
    after = rows[min(exit_idx+5, len(rows)-1)]['loc_err'] if exit_idx else -1
    delta = after - before
    print(f"  RECOVER@{f:4d}: before={before:.3f} -> during={during:.3f} -> "
          f"after={after:.3f} (delta={delta:+.3f})")

print(f"\n=== 4. State Distribution ===")
state_counts = {}
for r in rows:
    st = r['state'].replace('NavState.', '')
    state_counts[st] = state_counts.get(st, 0) + 1
for st, cnt in sorted(state_counts.items(), key=lambda x: -x[1]):
    print(f"  {st:12s}: {cnt:4d} frames ({cnt/len(rows)*100:.1f}%)")

print(f"\n=== 5. Movement Analysis ===")
# Speed analysis
speeds = []
for i in range(1, len(rows)):
    dx = rows[i]['true_x'] - rows[i-1]['true_x']
    dy = rows[i]['true_y'] - rows[i-1]['true_y']
    speeds.append(math.sqrt(dx*dx + dy*dy) / 0.1)  # m/s
avg_speed = sum(speeds) / len(speeds)
moving = sum(1 for s in speeds if s > 0.1)
still = sum(1 for s in speeds if s < 0.05)
print(f"  Avg speed: {avg_speed:.3f} m/s")
print(f"  Moving (>0.1m/s): {moving} frames ({moving/len(speeds)*100:.1f}%)")
print(f"  Still (<0.05m/s): {still} frames ({still/len(speeds)*100:.1f}%)")

# Repeated positions (robot going in circles)
print(f"\n=== 6. Position Revisit Analysis ===")
# Grid cells visited
cell_visits = {}
for r in rows:
    gx = int(r['true_x'] * 2)  # 0.5m grid
    gy = int(r['true_y'] * 2)
    cell_visits[(gx, gy)] = cell_visits.get((gx, gy), 0) + 1
total_cells = len(cell_visits)
hotspots = sorted(cell_visits.items(), key=lambda x: -x[1])[:10]
print(f"  Unique 0.5m cells visited: {total_cells}")
print(f"  Top revisited cells:")
for (gx, gy), cnt in hotspots:
    print(f"    ({gx*0.5:.1f},{gy*0.5:.1f}): {cnt} frames ({cnt/len(rows)*100:.1f}%)")

print(f"\n=== 7. Goal Reach Analysis ===")
# Track goal changes
goals = []
prev_goal = None
for r in rows:
    g = (r['goal_x'], r['goal_y'])
    if g != prev_goal:
        goals.append((r['frame'], g, r['state']))
        prev_goal = g
print(f"  Total goal changes: {len(goals)}")
for f, g, st in goals[:15]:
    print(f"    frame {f:4d}: goal=({g[0]:.1f},{g[1]:.1f}) state={st}")

print(f"\n=== 8. loc_err vs Position (spatial analysis) ===")
# Where is loc_err highest?
high_err = sorted(rows, key=lambda r: -r['loc_err'])[:10]
print(f"  Top 10 highest loc_err frames:")
for r in high_err:
    print(f"    frame {r['frame']:4d}: loc_err={r['loc_err']:.3f} "
          f"pos=({r['true_x']:.1f},{r['true_y']:.1f}) yaw={r['ryaw']:.1f} "
          f"state={r['state']}")

print(f"\n=== SUMMARY OF ISSUES ===")
print(f"  1. Coverage stuck at 91.9% for {2000-last_growth} frames ({(2000-last_growth)*0.1:.0f}s)")
print(f"  2. {len(recovers)} RECOVER events ({len(no_prog_recovers)} from no_progress)")
print(f"  3. RECOVER causes avg loc_err increase")
print(f"  4. Robot is still {still/len(speeds)*100:.1f}% of the time")
print(f"  5. {len(goals)} goal changes — frequent replanning")
