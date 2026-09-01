# -*- coding: utf-8 -*-
"""
分析 R27 逐帧 CSV trace, 回答三个悬而未决的问题:
  Q1 定位误差(err)何时/何地产生? 是缓慢累积还是突发跳变?
  Q2 A* 超时/失败集中在哪些区段?
  Q3 机器人卡在某个目标时, 指令速度/路径/碰撞在做什么?
"""
import csv
import os
import sys
from collections import defaultdict, Counter

TRACE = sys.argv[1] if len(sys.argv) > 1 else r"E:\puppyfangzhen\trace_val.csv"


def load(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    'frame': int(r['frame']), 't': float(r['t']),
                    'tx': float(r['true_x']), 'ty': float(r['true_y']),
                    'tyaw': float(r['true_yaw']),
                    'ex': float(r['est_x']), 'ey': float(r['est_y']),
                    'eyaw': float(r['est_yaw']),
                    'err': float(r['err']), 'conf': float(r['conf']),
                    'gi': int(r['goal_idx']),
                    'gx': float(r['goal_x']), 'gy': float(r['goal_y']),
                    'vx': float(r['cmd_vx']), 'vy': float(r['cmd_vy']),
                    'wz': float(r['cmd_wz']),
                    'col': int(r['collisions']),
                    'ac': int(r['astar_calls']), 'aok': int(r['astar_ok']),
                    'esc': int(r['escape']), 'pp': int(r['path_pts']),
                    'room': r['room'].strip(),
                })
            except (ValueError, KeyError):
                continue
    return rows


def bar(v, vmax, width=26):
    if vmax <= 0:
        return ''
    n = int(round(width * v / vmax))
    return '#' * max(0, min(width, n))


def main():
    if not os.path.exists(TRACE):
        print("NO TRACE FILE:", TRACE)
        return
    rows = load(TRACE)
    if not rows:
        print("TRACE EMPTY")
        return

    print("=" * 78)
    print("R27 TRACE ANALYSIS   rows=%d  frames=%d..%d  (%.1fs)"
          % (len(rows), rows[0]['frame'], rows[-1]['frame'], rows[-1]['t']))
    print("=" * 78)

    # ---------- Q1a: err time series (10s buckets) ----------
    print("\n[Q1a] localization error over time (10s buckets)")
    print("  %-9s %-7s %-7s %-7s %-6s %-8s %s"
          % ("window", "avg", "max", "conf", "esc%", "gt_move", "err profile"))
    buckets = defaultdict(list)
    for r in rows:
        buckets[int(r['t'] // 10)].append(r)
    gmax = max(r['err'] for r in rows)
    for b in sorted(buckets):
        rs = buckets[b]
        avg = sum(x['err'] for x in rs) / len(rs)
        mx = max(x['err'] for x in rs)
        cf = sum(x['conf'] for x in rs) / len(rs)
        esc = 100.0 * sum(1 for x in rs if x['esc']) / len(rs)
        move = 0.0
        for i in range(1, len(rs)):
            move += ((rs[i]['tx'] - rs[i-1]['tx'])**2 +
                     (rs[i]['ty'] - rs[i-1]['ty'])**2) ** 0.5
        print("  %-9s %-7.3f %-7.3f %-7.3f %-6.0f %-8.2f %s"
              % ("%3ds-%3ds" % (b*10, b*10+10), avg, mx, cf, esc, move,
                 bar(avg, gmax)))

    # ---------- Q1b: err by room ----------
    print("\n[Q1b] error by room (where does it drift?)")
    byroom = defaultdict(list)
    for r in rows:
        byroom[r['room']].append(r['err'])
    for room, es in sorted(byroom.items(), key=lambda kv: -sum(kv[1])/len(kv[1])):
        print("  %-14s frames=%-5d avg_err=%-7.3f max=%-7.3f"
              % (room, len(es), sum(es)/len(es), max(es)))

    # ---------- Q1c: biggest jumps ----------
    print("\n[Q1c] largest single-frame estimate jumps (drift events)")
    jumps = []
    for i in range(1, len(rows)):
        a, b = rows[i-1], rows[i]
        d = ((b['ex']-a['ex'])**2 + (b['ey']-a['ey'])**2) ** 0.5
        gt = ((b['tx']-a['tx'])**2 + (b['ty']-a['ty'])**2) ** 0.5
        jumps.append((d, gt, b))
    jumps.sort(key=lambda x: -x[0])
    print("  %-7s %-7s %-7s %-8s %-8s %-16s %s"
          % ("f", "est_jmp", "gt_move", "err", "conf", "est", "room"))
    for d, gt, b in jumps[:12]:
        print("  %-7d %-7.3f %-7.3f %-8.3f %-8.3f (%6.2f,%6.2f) %s"
              % (b['frame'], d, gt, b['err'], b['conf'], b['ex'], b['ey'], b['room']))

    # ---------- Q1d: est vs gt bias ----------
    print("\n[Q1d] estimate bias (is est systematically offset?)")
    mdx = sum(r['ex']-r['tx'] for r in rows) / len(rows)
    mdy = sum(r['ey']-r['ty'] for r in rows) / len(rows)
    print("  mean(est-gt) = (%+.3f, %+.3f)   |bias| = %.3f m" % (mdx, mdy, (mdx*mdx+mdy*mdy)**0.5))
    # lag test: does est trail gt?
    best = None
    for lag in range(0, 46):
        tot, n = 0.0, 0
        for i in range(lag, len(rows)):
            tot += ((rows[i]['ex']-rows[i-lag]['tx'])**2 +
                    (rows[i]['ey']-rows[i-lag]['ty'])**2) ** 0.5
            n += 1
        if n:
            v = tot/n
            if best is None or v < best[1]:
                best = (lag, v)
    print("  best-fit lag = %d frames (%.2fs), err at that lag = %.3f m"
          % (best[0], best[0]/30.0, best[1]))
    print("  -> if lag>2 and err drops a lot, the estimate is TRAILING the robot")

    # ---------- Q2: A* activity ----------
    print("\n[Q2] A* activity per 10s (calls / ok / fail)")
    print("  %-9s %-7s %-7s %-7s %-8s %s" % ("window", "calls", "ok", "fail", "rate%", "note"))
    for b in sorted(buckets):
        rs = buckets[b]
        c0, c1 = rs[0]['ac'], rs[-1]['ac']
        o0, o1 = rs[0]['aok'], rs[-1]['aok']
        calls, ok = c1-c0, o1-o0
        fail = calls - ok
        rate = (100.0*ok/calls) if calls else 0.0
        note = 'FAILING' if calls and rate < 80 else ''
        print("  %-9s %-7d %-7d %-7d %-8.1f %s"
              % ("%3ds-%3ds" % (b*10, b*10+10), calls, ok, fail, rate, note))

    # ---------- Q3: per-goal behaviour ----------
    print("\n[Q3] time spent per goal (who blocks progress?)")
    seg = []
    cur = rows[0]['gi']
    start = 0
    for i, r in enumerate(rows):
        if r['gi'] != cur:
            seg.append((cur, start, i-1))
            cur = r['gi']
            start = i
    seg.append((cur, start, len(rows)-1))
    print("  %-5s %-13s %-8s %-8s %-8s %-8s %-8s %s"
          % ("goal", "frames", "secs", "gt_move", "avg_spd", "avg_err", "esc%", "collisions"))
    for gi, a, b in seg:
        rs = rows[a:b+1]
        secs = (rs[-1]['t'] - rs[0]['t'])
        move = 0.0
        for i in range(1, len(rs)):
            move += ((rs[i]['tx']-rs[i-1]['tx'])**2 + (rs[i]['ty']-rs[i-1]['ty'])**2) ** 0.5
        spd = sum((x['vx']**2+x['vy']**2)**0.5 for x in rs)/len(rs)
        aerr = sum(x['err'] for x in rs)/len(rs)
        esc = 100.0*sum(1 for x in rs if x['esc'])/len(rs)
        dcol = rs[-1]['col'] - rs[0]['col']
        flag = '  <-- STUCK' if (secs > 15 and move < 1.0) else ''
        print("  #%-4d %-13s %-8.1f %-8.2f %-8.3f %-8.3f %-8.0f %-6d%s"
              % (gi, "%d-%d" % (rs[0]['frame'], rs[-1]['frame']), secs, move,
                 spd, aerr, esc, dcol, flag))

    # ---------- Q3b: commanded vs actual motion ----------
    print("\n[Q3b] commanded speed vs actual displacement (is the dog being blocked?)")
    tot_cmd = tot_act = 0.0
    blocked = 0
    for i in range(1, len(rows)):
        a, b = rows[i-1], rows[i]
        c = (b['vx']**2 + b['vy']**2) ** 0.5 / 30.0     # per-frame commanded
        act = ((b['tx']-a['tx'])**2 + (b['ty']-a['ty'])**2) ** 0.5
        tot_cmd += c
        tot_act += act
        if c > 0.005 and act < 0.2 * c:
            blocked += 1
    print("  commanded path  = %.2f m" % tot_cmd)
    print("  actual  path    = %.2f m" % tot_act)
    print("  efficiency      = %.1f%%" % (100.0*tot_act/tot_cmd if tot_cmd else 0))
    print("  blocked frames  = %d / %d (%.1f%%)  [cmd>0 but moved <20%% of it]"
          % (blocked, len(rows)-1, 100.0*blocked/max(1, len(rows)-1)))

    # ---------- summary ----------
    print("\n" + "=" * 78)
    print("OVERALL: avg_err=%.3f max_err=%.3f  gt_path=%.2fm  collisions=%d  targets_seen=%d"
          % (sum(r['err'] for r in rows)/len(rows), gmax, tot_act,
             rows[-1]['col'], len(set(r['gi'] for r in rows))))
    print("rooms visited:", dict(Counter(r['room'] for r in rows)))
    print("=" * 78)


if __name__ == '__main__':
    main()
