"""Probe LiDAR clearance in every heading, using the adapter's own algorithm.

Purpose: when a mission aborts with "blocked by obstacle", the interesting
question is WHAT the 0.34 m return is -- a real wall, the robot's own legs, or
the floor because the base has sunk. Those three look identical in a single
`obstacle_distance` number and lead to completely different fixes, so this
replays the adapter's clearance rule (+/-30 deg arc, 20th percentile) for a full
circle of headings and prints the result.

Usage (inside WSL, with Gazebo/SLAM already running):
    source /opt/ros/humble/setup.bash
    source ~/puppy_ws/install/setup.bash
    python3 /mnt/e/puppyfangzhen/tools/diagnose_clearance.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Must match track_cmd_adapter_node parameters (config/minicpm_robot.yaml).
ARC_DEG = 60.0
PERCENTILE = 20.0
STOP_DISTANCE = 0.35


def percentile(values, pct):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((pct / 100.0) * (len(ordered) - 1)))
    idx = max(0, min(len(ordered) - 1, idx))
    return ordered[idx]


def clearance(scan, heading_rad):
    """Clearance for a world-frame heading, per the adapter's arc rule."""
    half = math.radians(ARC_DEG / 2.0)
    inside = []
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        delta = abs(((angle - heading_rad + math.pi) % (2 * math.pi)) - math.pi)
        if delta <= half and r == r and r > 0.0:  # r == r filters NaN
            inside.append(r)
    return percentile(inside, PERCENTILE), len(inside)


class Probe(Node):
    def __init__(self):
        super().__init__("clearance_probe")
        self.scan = None
        self.yaw = None
        self.pos = None
        self.create_subscription(LaserScan, "/scan", self._scan, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)

    def _scan(self, msg):
        self.scan = msg

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)


def main():
    rclpy.init()
    node = Probe()
    deadline = time.time() + 25
    while time.time() < deadline and (node.scan is None or node.yaw is None):
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.scan is None or node.yaw is None:
        print("MISSING data: scan=%s odom=%s"
              % (node.scan is not None, node.yaw is not None))
        rclpy.shutdown()
        return

    scan = node.scan
    print("odom: x=%.3f y=%.3f z=%.3f  yaw=%.1f deg"
          % (node.pos[0], node.pos[1], node.pos[2], math.degrees(node.yaw)))
    print("scan: rays=%d  angle_min=%.1f deg  inc=%.2f deg  max=%.2f m"
          % (len(scan.ranges), math.degrees(scan.angle_min),
             math.degrees(scan.angle_increment), scan.range_max))
    print("rule: +/-%.0f deg arc, %.0fth percentile, stop below %.2f m"
          % (ARC_DEG / 2, PERCENTILE, STOP_DISTANCE))
    print()
    print("heading(deg world)  clearance(m)  rays   note")

    facing = math.degrees(node.yaw)
    open_headings = []
    for deg in range(0, 360, 15):
        c, n = clearance(scan, math.radians(deg))
        note = ""
        if abs(((facing - deg + 180) % 360) - 180) < 8:
            note = "<- base facing"
        if c >= STOP_DISTANCE:
            open_headings.append((deg, c))
            note += " [DRIVABLE]"
        print("      %3d              %.3f      %3d   %s" % (deg, c, n, note))

    print()
    if open_headings:
        best = max(open_headings, key=lambda kv: kv[1])
        print("drivable headings: %s" % ", ".join("%d deg (%.2f m)" % h
                                                  for h in open_headings))
        print("most open: %d deg at %.2f m -- base faces %.0f deg, so it must turn %.0f deg"
              % (best[0], best[1], facing,
                 abs(((best[0] - facing + 180) % 360) - 180)))
    else:
        print("NO heading has clearance >= %.2f m: every direction reads closer than the"
              % STOP_DISTANCE)
        print("stop distance. That is a self-return (legs/floor), not a wall -- no amount")
        print("of steering or re-planning will clear it.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
