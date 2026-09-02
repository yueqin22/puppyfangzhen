#!/usr/bin/env python3
"""Print a side-by-side comparison of the escape A/B arms (see run_escape_ab.sh).

Usage:  python3 escape_ab_report.py ab_off_safety.json ab_on_safety.json

A note on what these numbers can and cannot say: the recorder runs for a fixed
window that starts before the mission is dispatched, so a large share of every
arm is STANDBY (the adapter publishes safety status whether or not a mission is
running). STANDBY samples report the ambient clearance (~1.4 m) and never set
lidar_override, so they do not distort the minimum clearance or the block
counters -- but they do mean `samples` is not a measure of mission length. Use
`nav_samples` for that.
"""

import json
import sys


KEYS = (
    "samples",
    "nav_samples",
    "obstacle_distance_min",
    "obstacle_distance_p05",
    "obstacle_distance_median",
    "blocked_duration_max_s",
    "lidar_override_samples",
    "blocked_escalated_samples",
    "commanded_vy_nonzero",
    "commanded_vy_peak",
)


def load(path):
    """Accept either recorder's output.

    record_safety_live.py stores every reason under "reasons"; probe_escape.py
    keeps only "top_reasons" and calls its sample count "safety_samples".
    Tolerating both keeps one report usable for either experiment.
    """
    with open(path) as fh:
        d = json.load(fh)
    reasons = d.get("reasons") or d.get("top_reasons") or {}
    d["nav_samples"] = sum(c for r, c in reasons.items() if r.startswith("NAV_MOVE"))
    d.setdefault("samples", d.get("safety_samples"))
    d["_blocked_reasons"] = {
        r: c for r, c in reasons.items()
        if "BLOCK" in r or "SIDESTEP" in r or "LIDAR" in r
    }
    return d


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return "%.3f" % v
    return str(v)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    off = load(sys.argv[1])
    on = load(sys.argv[2])

    print("%-28s %14s %14s" % ("metric", "escape OFF", "escape ON"))
    print("-" * 60)
    for k in KEYS:
        print("%-28s %14s %14s" % (k, fmt(off.get(k)), fmt(on.get(k))))
    print("-" * 60)
    for label, d in (("OFF", off), ("ON", on)):
        br = d.get("_blocked_reasons") or {}
        print("%s blocked-side reasons: %s" % (
            label, ", ".join("%s x%d" % (r[:52], c) for r, c in
                             sorted(br.items(), key=lambda kv: -kv[1])) or "(none)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
