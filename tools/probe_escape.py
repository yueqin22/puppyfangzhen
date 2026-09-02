#!/usr/bin/env python3
"""Deterministically trigger a navigation block and watch how it is handled.

Why this exists
---------------
The block that motivated the tangential escape (defect 13) is intermittent: it
depends on where SLAM drift happens to put the base that run, so the A/B in
run_escape_ab.sh produced one arm that never got close to a wall at all --
`escape OFF` finished with a 0.566 m minimum clearance and zero blocked samples,
which says nothing about the behaviour under test.

This probe stops waiting for luck. It reads the live scan, finds the narrowest
heading the base could try to drive along, and puts the navigation goal 2 m down
exactly that bearing. The base is then guaranteed to be vetoed by its own LiDAR
within a fraction of a second, so the two arms differ only in the parameter
under test:

    escape OFF (escape_clearance_factor=999.0) -> every tangent is rejected,
    the base stops dead, `blocked_duration_s` grows without bound and
    `blocked_escalated` latches on.

    escape ON (1.15) -> the base slides along the open tangent instead.

Neither arm can reach the goal (it is behind an obstacle by construction), so
both end in a failed mission. What matters is HOW they spend the interval:
blocked duration, escalation, whether any lateral command was issued, and
whether the base actually moved.

Usage (WSL, Gazebo + SLAM running, 4 adapter nodes launched):
    python3 probe_escape.py --duration 40 --out escape_probe.json
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# Must match track_cmd_adapter_node parameters (config/minicpm_robot.yaml).
ARC_DEG = 60.0
PERCENTILE = 20.0

GOAL_AHEAD_M = 2.0     # goal is deliberately beyond the obstacle
PUBLISH_HZ = 5.0


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    return ordered[max(0, min(len(ordered) - 1, idx))]


def arc_clearance(scan, heading_rad):
    """Adapter's clearance rule: percentile over a +/-ARC_DEG/2 arc."""
    half = math.radians(ARC_DEG / 2.0)
    inside = []
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        delta = abs(((angle - heading_rad + math.pi) % (2 * math.pi)) - math.pi)
        if delta <= half and r == r and r > 0.0:
            inside.append(r)
    return percentile(inside, PERCENTILE)


class EscapeProbe(Node):
    def __init__(self):
        super().__init__("escape_probe")
        self.scan = None
        self.pose = None          # [x, y, yaw] in map frame
        self.phase = None
        self.safety = []          # (t, dict)
        self.poses = []           # (t, [x, y, yaw])
        self.t0 = None

        self.create_subscription(LaserScan, "/scan", self._scan, 10)
        self.create_subscription(String, "/minicpm_robot/mission_status", self._status, 20)
        self.create_subscription(String, "/minicpm_robot/safety_status", self._safety, 50)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cmd_pub = self.create_publisher(String, "/mission/command", 10)

    def _scan(self, msg):
        self.scan = msg

    def _status(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self.phase = d.get("phase", self.phase)
        p = d.get("robot_pose")
        if p and len(p) >= 3:
            self.pose = [float(p[0]), float(p[1]), float(p[2])]
            if self.t0 is None:
                self.t0 = time.time()
            self.poses.append((time.time() - self.t0, list(self.pose)))

    def _safety(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if self.t0 is None:
            self.t0 = time.time()
        self.safety.append((time.time() - self.t0, d))

    def narrowest_heading(self):
        """Body-frame heading with the least clearance, and that clearance."""
        best = (float("inf"), 0.0)
        for deg in range(-180, 180, 2):
            h = math.radians(deg)
            c = arc_clearance(self.scan, h)
            if c < best[0]:
                best = (c, h)
        return best[1], best[0]

    def publish_goal(self, body_heading):
        x, y, yaw = self.pose
        wx = yaw + body_heading
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x + GOAL_AHEAD_M * math.cos(wx)
        msg.pose.position.y = y + GOAL_AHEAD_M * math.sin(wx)
        msg.pose.orientation.z = math.sin(wx / 2.0)
        msg.pose.orientation.w = math.cos(wx / 2.0)
        self.goal_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--out", default="escape_probe.json")
    args = ap.parse_args()

    rclpy.init()
    node = EscapeProbe()

    # 1. Dispatch a mission FIRST. mission_grounder publishes nothing at all
    #    until a mission is loaded, and /minicpm_robot/mission_status is where
    #    this probe gets the map-frame pose. Waiting for a pose before
    #    dispatching therefore deadlocks: the pose cannot exist yet.
    deadline = time.time() + 30.0
    while time.time() < deadline and node.cmd_pub.get_subscription_count() < 1:
        rclpy.spin_once(node, timeout_sec=0.2)
    cmd = String()
    cmd.data = "Go to living_room and inspect the area"
    node.cmd_pub.publish(cmd)
    print("mission dispatched    : %s" % cmd.data)

    # 2. Now wait for a scan and the map pose the mission status carries.
    deadline = time.time() + 45.0
    while time.time() < deadline and (node.scan is None or node.pose is None):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.scan is None or node.pose is None:
        print("NO DATA: scan=%s pose=%s -- is SLAM publishing a map->base transform?"
              % (node.scan is not None, node.pose is not None))
        rclpy.shutdown()
        return 2

    heading, clearance = node.narrowest_heading()
    start_pose = list(node.pose)
    print("start pose (map)      : %.3f, %.3f, yaw %.3f" % tuple(start_pose))
    print("narrowest heading     : %+.0f deg body, clearance %.3f m"
          % (math.degrees(heading), clearance))
    print("goal                  : %.2f m down that bearing (behind the obstacle)"
          % GOAL_AHEAD_M)

    # 3. Keep re-publishing our own goal: mission_grounder also publishes one for
    #    its zone, and the blocked goal has to be the live target.
    #    Drive the blocked goal for the whole window, recording as we go.
    end = time.time() + args.duration
    next_pub = 0.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now >= next_pub and node.pose is not None:
            node.publish_goal(heading)
            next_pub = now + 1.0 / PUBLISH_HZ

    # 4. Summarise.
    dists, durations, vys, blocked, esc, reasons = [], [], [], 0, 0, {}
    for _, d in node.safety:
        try:
            v = float(d.get("obstacle_distance", float("nan")))
            if v == v:
                dists.append(v)
        except Exception:
            pass
        try:
            durations.append(float(d.get("blocked_duration_s", 0.0)))
        except Exception:
            pass
        try:
            vys.append(float(d.get("commanded_vy", 0.0)))
        except Exception:
            pass
        if d.get("lidar_override"):
            blocked += 1
        if d.get("blocked_escalated"):
            esc += 1
        r = d.get("active_override_reason", "?") or "?"
        reasons[r] = reasons.get(r, 0) + 1

    travelled = 0.0
    if len(node.poses) > 1:
        (_, p0), (_, p1) = node.poses[0], node.poses[-1]
        travelled = math.hypot(p1[0] - p0[0], p1[1] - p0[1])

    summary = {
        "narrowest_clearance_at_start_m": round(clearance, 3),
        "safety_samples": len(node.safety),
        "final_phase": node.phase,
        "obstacle_distance_min": round(min(dists), 3) if dists else None,
        "blocked_duration_max_s": round(max(durations), 2) if durations else 0.0,
        "lidar_override_samples": blocked,
        "blocked_escalated_samples": esc,
        "commanded_vy_nonzero": sum(1 for v in vys if abs(v) > 1e-6),
        "commanded_vy_peak": round(max((abs(v) for v in vys), default=0.0), 3),
        "net_displacement_m": round(travelled, 3),
        "top_reasons": dict(sorted(reasons.items(), key=lambda kv: -kv[1])[:6]),
    }
    with open(args.out, "w") as fh:
        json.dump(summary, fh, indent=2)
    print(json.dumps(summary, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
