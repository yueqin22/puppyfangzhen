import csv, json, math
from collections import defaultdict

rows = list(csv.DictReader(open('trace_val.csv')))
N = len(rows)
obs = json.load(open('config/scene_home.json'))['obstacles']

def min_gap(x, y):
    g = 1e9
    for o in obs:
        # distance from point (x,y)m to AABB in meters
        dx = max(o['xmin']/100.0 - x, 0, x - o['xmax']/100.0)
        dy = max(o['ymin']/100.0 - y, 0, y - o['ymax']/100.0)
        g = min(g, math.hypot(dx, dy))
    return g

# --- 1. error vs time ---
print("=== AMCL 误差随时间 (每600帧均值) ===")
for b in range(0, N, 600):
    seg = rows[b:b+600]
    if not seg: break
    e = [float(r['err']) for r in seg]
    print("  frames %4d-%4d  mean_err=%.3f  max_err=%.3f" % (b, b+len(seg)-1,
          sum(e)/len(e), max(e)))

# --- 2. gap to nearest obstacle ---
print("\n=== 狗到最近障碍物的间隙 (m) ===")
gaps = [min_gap(float(r['true_x']), float(r['true_y'])) for r in rows]
print("  mean_gap=%.3f  median=%.3f  min=%.3f" % (sum(gaps)/N, sorted(gaps)[N//2], min(gaps)))
for thr in (0.15, 0.20, 0.25, 0.30, 0.40):
    c = sum(1 for g in gaps if g < thr)
    print("  gap < %.2f m : %5d frames (%.1f%%)" % (thr, c, 100*c/N))

# --- 3. stuck runs ---
print("\n=== 卡死段 (连续 displacement 极小) ===")
xs = [float(r['true_x']) for r in rows]
ys = [float(r['true_y']) for r in rows]
disp = [math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(N-1)] + [0]
RUN = 6  # cm per frame ~ low movement threshold
runs = []
i = 0
while i < N:
    if disp[i] < RUN/100.0:
        j = i
        while j < N and disp[j] < RUN/100.0:
            j += 1
        runs.append((i, j, (xs[i]+xs[j-1])/2, (ys[i]+ys[j-1])/2))
        i = j
    else:
        i += 1
runs.sort(key=lambda r: r[1]-r[0], reverse=True)
print("  共 %d 段卡死, 总卡死帧=%d (%.1f%%)" % (len(runs), sum(r[1]-r[0] for r in runs), 100*sum(r[1]-r[0] for r in runs)/N))
for (a, b, mx, my) in runs[:6]:
    # goal during this run
    g = rows[(a+b)//2]
    print("    f%4d-%4d  len=%3d  中心=(%.2f,%.2f)  goal#%s (%.2f,%.2f)" % (
        a, b-1, b-a, mx, my, g['goal_idx'], float(g['goal_x']), float(g['goal_y'])))

# --- 4. LOS failures ---
print("\n=== 前瞻视线 (LOS) 失败率 ===")
have = [r for r in rows if int(r['la_idx']) >= 0]
naive_bad = sum(1 for r in have if int(r['la_naive_los']) == 0)
new_bad = sum(1 for r in have if int(r['la_los']) == 0)
print("  有路径帧=%d" % len(have))
print("  OLD(closest+3) 会穿墙: %d (%.1f%%)" % (naive_bad, 100*naive_bad/len(have)))
print("  NEW(LOS感知)   会穿墙: %d (%.1f%%)" % (new_bad, 100*new_bad/len(have)))

# --- 5. collisions per obstacle (attribute by nearest) ---
print("\n=== 碰撞热点 (按最近障碍归因) ===")
cnt = defaultdict(int)
for r in rows:
    if float(r['collisions']) > 0:
        # collisions is cumulative; detect increase
        pass
# use differenced collision events
prev = 0.0
events = 0
for r in rows:
    c = float(r['collisions'])
    if c > prev:
        events += 1
        g = min_gap(float(r['true_x']), float(r['true_y']))
        # find nearest obstacle name
        best = None; bg = 1e9
        for o in obs:
            dx = max(o['xmin']/100.0 - float(r['true_x']), 0, float(r['true_x']) - o['xmax']/100.0)
            dy = max(o['ymin']/100.0 - float(r['true_y']), 0, float(r['true_y']) - o['ymax']/100.0)
            d = math.hypot(dx, dy)
            if d < bg: bg = d; best = o['name']
        cnt[best] += 1
    prev = c
print("  总碰撞事件=%d" % events)
for k, v in sorted(cnt.items(), key=lambda x: -x[1])[:8]:
    print("    %-16s %d" % (k, v))
