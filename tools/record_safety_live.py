#!/usr/bin/env python3
"""Record /minicpm_robot/safety_status at full publish rate for a whole run.

Why this exists
---------------
`verify_standby_live.sh` samples safety status with `ros2 topic echo --once`.
Each sample costs a full DDS discovery + subscribe + wait + teardown cycle, so a
mission that runs for 60-100 s yields only 4-5 samples. That is far too sparse
to characterise an obstacle encounter:

* a 6 s block showed up as exactly one sample, so "blocked_escalated=1" and
  "blocked_escalated=0" were one unlucky sample apart;
* a run where the base never got close to a wall looks identical to a run whose
  close encounter simply fell between samples;
* the reported `obstacle_distance min` is the minimum of ~4 numbers, not of the
  ~1500 the adapter actually published.

This recorder subscribes once and keeps every message, so the minimum
clearance, the block duration profile and the override-reason histogram are all
grounded in the full series.

Usage
-----
    python3 record_safety_live.py --duration 300 --out safety_run.json

Run it alongside (not inside) verify_standby_live.sh --mission: the adapter
publishes safety status at 20 Hz regardless of who is listening, so starting
this a few seconds late only loses the first few samples.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SafetyRecorder(Node):
    def __init__(self, topic):
        super().__init__("safety_status_recorder")
        self.samples = []
        self.first_ts = None
        self.create_subscription(String, topic, self._cb, 50)

    def _cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        now = time.time()
        if self.first_ts is None:
            self.first_ts = now
        self.samples.append((now - self.first_ts, data))


def summarize(samples):
    from collections import Counter

    reasons = Counter()
    dists = []
    durations = []
    vy = []
    escalated = 0
    blocked = 0
    navigating = 0
    for _, d in samples:
        reasons[d.get("active_override_reason", "?") or "?"] += 1
        try:
            dists.append(float(d.get("obstacle_distance", float("nan"))))
        except Exception:
            pass
        try:
            durations.append(float(d.get("blocked_duration_s", 0.0)))
        except Exception:
            pass
        try:
            vy.append(float(d.get("commanded_vy", 0.0)))
        except Exception:
            pass
        if d.get("blocked_escalated"):
            escalated += 1
        if d.get("lidar_override"):
            blocked += 1
        if d.get("navigating"):
            navigating += 1

    def pct(vals, q):
        if not vals:
            return None
        s = sorted(vals)
        return s[min(len(s) - 1, int(len(s) * q))]

    dists = [d for d in dists if d == d]
    return {
        "samples": len(samples),
        "wall_span_s": (samples[-1][0] - samples[0][0]) if len(samples) > 1 else 0.0,
        "reasons": dict(reasons.most_common()),
        "obstacle_distance_min": min(dists) if dists else None,
        "obstacle_distance_p05": pct(dists, 0.05),
        "obstacle_distance_median": pct(dists, 0.50),
        "blocked_duration_max_s": max(durations) if durations else 0.0,
        "lidar_override_samples": blocked,
        "blocked_escalated_samples": escalated,
        "navigating_samples": navigating,
        "commanded_vy_nonzero": sum(1 for v in vy if abs(v) > 1e-6),
        "commanded_vy_peak": max((abs(v) for v in vy), default=0.0),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/minicpm_robot/safety_status")
    ap.add_argument("--duration", type=float, default=300.0)
    ap.add_argument("--out", default="safety_run.json")
    args = ap.parse_args()

    rclpy.init()
    node = SafetyRecorder(args.topic)
    deadline = time.time() + args.duration
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        summary = summarize(node.samples)
        summary["topic"] = args.topic
        summary["duration_requested_s"] = args.duration
        with open(args.out, "w") as fh:
            json.dump(summary, fh, indent=2)
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
