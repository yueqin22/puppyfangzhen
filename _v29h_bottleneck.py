#!/usr/bin/env python3
"""Analyze v2.9h bottlenecks: unreachable goals, static frames, TEB behavior."""
import csv
import math

rows = list(csv.DictReader(open('/tmp/puppy_nav_data.csv')))
for r in rows:
    for k in ['frame', 'time', 'rx', 'ry', 'ryaw', 'coverage', 'visited',
              'cost_at_robot', 'dwa_v', 'dwa_w', 'no_progress', 'goal_x',
              'goal_y', 'dist_to_goal', 'path_len', 'true_x', 'true_y', 'loc_err']:
        r[k] = float(r[k])
    r['frame'] = int(r['frame'])

print("=== 1. Static Frames Analysis (dwa_v < 0.05) ===")
static_frames = [r for r in rows if r['dwa_v'] < 0.05]
moving_frames = [r for r in rows if r['dwa_v'] >= 0.05]
print(f"Static: {len(static_frames)} ({len(static_frames)/len(rows)*100:.1f}%)")
print(f"Moving: {len(moving_frames)} ({len(moving_frames)/len(rows)*100:.1f}%)")

# Static by state
static_by_state = {}
for r in static_frames:
    st = r['state'].replace('NavState.', '')
    static_by_state[st] = static_by_state.get(st, 0) + 1
print("Static frames by state:")
for st, cnt in sorted(static_by_state.items(), key=lambda x: -x[1]):
    print(f"  {st:12s}: {cnt:4d} ({cnt/len(static_frames)*100:.1f}%)")

# Static in FOLLOW: what's happening?
static_follow = [r for r in static_frames if 'FOLLOW' in r['state']]
if static_follow:
    print(f"\nStatic FOLLOW frames: {len(static_follow)}")
    # Check dwa_w during static FOLLOW (rotating in place?)
    rotating = sum(1 for r in static_follow if abs(r['dwa_w']) > 0.3)
    print(f"  Rotating (|w|>0.3): {rotating} ({rotating/len(static_follow)*100:.1f}%)")
    # Check cost at robot
    high_cost = sum(1 for r in static_follow if r['cost_at_robot'] > 50)
    print(f"  High cost (>50): {high_cost} ({high_cost/len(static_follow)*100:.1f}%)")
    # no_progress distribution
    np_vals = [r['no_progress'] for r in static_follow]
    print(f"  no_progress: min={min(np_vals):.0f} max={max(np_vals):.0f} "
          f"mean={sum(np_vals)/len(np_vals):.1f}")

print("\n=== 2. Goal Distribution & Unreachable Analysis ===")
# Track all goals selected
goals = []
prev_goal = None
for r in rows:
    g = (round(r['goal_x'], 1), round(r['goal_y'], 1))
    if g != prev_goal:
        goals.append((r['frame'], g, r['state'], r['true_x'], r['true_y']))
        prev_goal = g

print(f"Total unique goals (0.1m): {len(set(g[1] for g in goals))}")
print(f"Total goal changes: {len(goals)}")

# Goal clusters
goal_positions = {}
for f, g, st, rx, ry in goals:
    goal_positions[g] = goal_positions.get(g, 0) + 1
top_goals = sorted(goal_positions.items(), key=lambda x: -x[1])[:15]
print(f"\nTop 15 most-selected goal positions (0.1m grid):")
for (gx, gy), cnt in top_goals:
    print(f"  ({gx:.1f}, {gy:.1f}): selected {cnt} times")

print("\n=== 3. Coverage Growth Timeline ===")
prev_cov = 0
growth_events = []
for r in rows:
    if r['coverage'] > prev_cov + 0.1:
        growth_events.append((r['frame'], r['coverage'], r['true_x'], r['true_y']))
        prev_cov = r['coverage']
print(f"Coverage growth events: {len(growth_events)}")
# Show first and last 10
for f, c, x, y in growth_events[:10]:
    print(f"  frame {f:4d}: cov={c:.1f}% at ({x:.1f},{y:.1f})")
print("  ...")
for f, c, x, y in growth_events[-10:]:
    print(f"  frame {f:4d}: cov={c:.1f}% at ({x:.1f},{y:.1f})")

print("\n=== 4. TEB/DWA Velocity Profile ===")
# Speed distribution in FOLLOW
follow_rows = [r for r in rows if 'FOLLOW' in r['state']]
v_vals = [r['dwa_v'] for r in follow_rows]
w_vals = [abs(r['dwa_w']) for r in follow_rows]
print(f"FOLLOW frames: {len(follow_rows)}")
print(f"  Linear v: mean={sum(v_vals)/len(v_vals):.3f} "
      f"min={min(v_vals):.3f} max={max(v_vals):.3f}")
print(f"  Angular |w|: mean={sum(w_vals)/len(w_vals):.3f} "
      f"min={min(w_vals):.3f} max={max(w_vals):.3f}")
# Speed buckets
v_buckets = {'0.0-0.05': 0, '0.05-0.1': 0, '0.1-0.2': 0, '0.2-0.3': 0, '0.3+': 0}
for v in v_vals:
    if v < 0.05: v_buckets['0.0-0.05'] += 1
    elif v < 0.1: v_buckets['0.05-0.1'] += 1
    elif v < 0.2: v_buckets['0.1-0.2'] += 1
    elif v < 0.3: v_buckets['0.2-0.3'] += 1
    else: v_buckets['0.3+'] += 1
print("  Linear v distribution:")
for bucket, cnt in v_buckets.items():
    print(f"    {bucket}: {cnt} ({cnt/len(v_vals)*100:.1f}%)")

print("\n=== 5. Path Length Analysis ===")
path_lens = [r['path_len'] for r in rows if r['path_len'] > 0]
if path_lens:
    print(f"  Path length: mean={sum(path_lens)/len(path_lens):.1f} "
          f"min={min(path_lens):.1f} max={max(path_lens):.1f}")
    short_paths = sum(1 for p in path_lens if p < 2.0)
    print(f"  Short paths (<2m): {short_paths} ({short_paths/len(path_lens)*100:.1f}%)")

print("\n=== 6. RECOVER Impact on loc_err ===")
recovers = []
prev_state = None
for r in rows:
    if 'RECOVER' in r['state'] and (prev_state is None or 'RECOVER' not in prev_state):
        recovers.append(r)
    prev_state = r['state']

print(f"RECOVER events: {len(recovers)}")
for r in recovers:
    f = r['frame']
    before = rows[max(f-5, 0)]['loc_err']
    # Find exit
    after_idx = None
    for i, rr in enumerate(rows):
        if rr['frame'] > f and 'RECOVER' not in rr['state']:
            after_idx = i
            break
    after = rows[min(after_idx+5, len(rows)-1)]['loc_err'] if after_idx else -1
    print(f"  RECOVER@{f:4d}: {before:.3f} -> {r['loc_err']:.3f} -> {after:.3f} "
          f"pos=({r['true_x']:.1f},{r['true_y']:.1f}) no_prog={r['no_progress']:.0f}")

print("\n=== 7. Spatial Coverage Map (0.5m grid) ===")
grid = {}
for r in rows:
    gx = int(r['true_x'] * 2)
    gy = int(r['true_y'] * 2)
    grid[(gx, gy)] = grid.get((gx, gy), 0) + 1
# Print as text map
min_gx = min(k[0] for k in grid)
max_gx = max(k[0] for k in grid)
min_gy = min(k[1] for k in grid)
max_gy = max(k[1] for k in grid)
print(f"  Grid: x=[{min_gx*0.5:.1f},{max_gx*0.5:.1f}] y=[{min_gy*0.5:.1f},{max_gy*0.5:.1f}]")
print(f"  Unique cells: {len(grid)}")
# Show y=3.0-4.0 area (where robot gets stuck)
print("  Visits to y=3.0-4.0 area (north edge):")
north = {k: v for k, v in grid.items() if k[1] >= 6}
for k, v in sorted(north.items()):
    print(f"    ({k[0]*0.5:.1f},{k[1]*0.5:.1f}): {v} frames")
