#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mine trace_val.csv to localize collisions and stalls, and attribute each
collision to the control mode that CAUSED it.

The `escape` column is a mode code written by nav_ue_bridge.cpp:
    0 = normal planner following
    1 = escape burst (stall recovery override)
    2 = collision brake (reactive contact response)

Attribution note: the brake engages on the very frame a contact is reported,
so the mode *at* the collision frame is always 2 and tells us nothing.  To
find the cause we look at the mode in the frames immediately BEFORE the
contact (default: 3 frames back).
"""
import csv
import sys
from collections import Counter

LOOKBACK = 3
MODE_NAME = {0: "normal", 1: "escape", 2: "brake"}


def fnum(row, k, d=0.0):
    try:
        return float(row[k])
    except Exception:
        return d


def mode_of(row):
    return int(fnum(row, "escape"))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "trace_val.csv"
    with open(path, encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        cols = r.fieldnames
        rows = list(r)

    print("columns: %d, rows: %d" % (len(cols or []), len(rows)))
    if not rows:
        print("EMPTY TRACE")
        return 1

    # ---- mode occupancy ---------------------------------------------------
    modes = Counter(mode_of(x) for x in rows)
    print("\n=== CONTROL MODE OCCUPANCY ===")
    for m in sorted(modes):
        print("  %-7s %5d frames (%.1f%%)"
              % (MODE_NAME.get(m, m), modes[m], 100.0 * modes[m] / len(rows)))

    # ---- collisions + causal attribution ----------------------------------
    prev = 0
    events = []          # (idx, delta, room, cause_mode)
    for i, row in enumerate(rows):
        c = int(fnum(row, "collisions"))
        if c > prev:
            lo = max(0, i - LOOKBACK)
            pre = [mode_of(x) for x in rows[lo:i]]
            # Cause = the most "active" override running just before impact;
            # an escape in the run-up outranks a brake carried over from an
            # earlier hit, and no override at all means the planner did it.
            if 1 in pre:
                cause = 1
            elif 2 in pre:
                cause = 2
            else:
                cause = 0
            events.append((i, c - prev, row.get("room", "?"), cause))
            prev = c

    print("\n=== COLLISION EVENTS (count=%d) ===" % len(events))
    if events:
        by_cause = Counter(MODE_NAME.get(e[3], e[3]) for e in events)
        print("  attributed cause (mode in %d frames before impact):" % LOOKBACK)
        for k, v in by_cause.most_common():
            print("     %-7s %3d  (%.0f%%)" % (k, v, 100.0 * v / len(events)))
        print("  by room:", dict(Counter(e[2] for e in events).most_common()))
        print("  first 12 events:")
        for i, d, room, cause in events[:12]:
            row = rows[i]
            print("     f=%-5s room=%-11s est=(%6.2f,%6.2f) cmd=(%5.2f,%5.2f) "
                  "cause=%-6s path=%s"
                  % (row.get("frame", "?"), room,
                     fnum(row, "est_x"), fnum(row, "est_y"),
                     fnum(row, "cmd_vx"), fnum(row, "cmd_vy"),
                     MODE_NAME.get(cause, cause), row.get("path_pts", "?")))

    # ---- stalls (ground-truth motion, matching the bridge's own metric) ----
    stalls = []
    key = "true_x" if "true_x" in (cols or []) else "est_x"
    keyy = "true_y" if "true_y" in (cols or []) else "est_y"
    for i in range(1, len(rows)):
        dx = fnum(rows[i], key) - fnum(rows[i - 1], key)
        dy = fnum(rows[i], keyy) - fnum(rows[i - 1], keyy)
        if (dx * dx + dy * dy) < (0.005 * 0.005):
            stalls.append(i)

    print("\n=== STALL FRAMES (count=%d, %.1f%%) ===  [source=%s]"
          % (len(stalls), 100.0 * len(stalls) / len(rows), key))
    if stalls:
        print("  by room:", dict(Counter(rows[i].get("room", "?")
                                         for i in stalls).most_common()))
        print("  by mode:", {MODE_NAME.get(k, k): v for k, v in
                             Counter(mode_of(rows[i]) for i in stalls).items()})
        print("  with no usable path (path_pts<=1): %d"
              % sum(1 for i in stalls if int(fnum(rows[i], "path_pts")) <= 1))

        # longest continuous stall burst -- distinguishes a true wedge from
        # ordinary stutter.
        runs, cur = [], [stalls[0]]
        for a, b in zip(stalls, stalls[1:]):
            if b == a + 1:
                cur.append(b)
            else:
                runs.append(cur)
                cur = [b]
        runs.append(cur)
        runs.sort(key=len, reverse=True)
        print("  longest bursts (frames): %s"
              % [len(x) for x in runs[:8]])
        print("  bursts >=30 frames (>=1s): %d" % sum(1 for x in runs if len(x) >= 30))
        top = runs[0]
        print("  longest burst: f=%s..%s (%d frames = %.1fs) room=%s mode=%s"
              % (rows[top[0]].get("frame"), rows[top[-1]].get("frame"),
                 len(top), len(top) / 30.0, rows[top[0]].get("room"),
                 MODE_NAME.get(mode_of(rows[top[0]]))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
