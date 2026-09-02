"""Print the base pose together with the clearance the adapter would compute.

Stands in for a full mission when all that is needed is "where is the base and
what does its forward arc read". Used to test whether a low standby clearance is
a property of the sensor/robot or simply inherited from wherever the previous run
left the base -- Gazebo keeps the robot where it was when only the ROS nodes are
restarted.

Usage (WSL, sim running):
    python3 /mnt/e/puppyfangzhen/tools/probe_pose_clearance.py [label]
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

ARC_DEG = 60.0
PERCENTILE = 20.0
STOP_DISTANCE = 0.35


class Probe(Node):
    def __init__(self):
        super().__init__("pose_clearance_probe")
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
    label = sys.argv[1] if len(sys.argv) > 1 else ""
    rclpy.init()
    node = Probe()
    deadline = time.time() + 25
    while time.time() < deadline and (node.scan is None or node.yaw is None):
        rclpy.spin_once(node, timeout_sec=0.3)

    if node.scan is None or node.yaw is None:
        print("[%s] MISSING scan or odom" % label)
        rclpy.shutdown()
        return

    scan = node.scan
    n = len(scan.ranges)
    ranges = list(scan.ranges)
    near = sum(1 for r in ranges if r == r and scan.range_min < r < STOP_DISTANCE)

    half = int(round(math.radians(ARC_DEG / 2.0) / scan.angle_increment))
    centre_idx = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    window = []
    for k in range(-half, half + 1):
        r = ranges[(centre_idx + k) % n]
        if r == r and scan.range_min < r <= scan.range_max:
            window.append(r)
    window.sort()
    if window:
        idx = int(round(PERCENTILE / 100.0 * (len(window) - 1)))
        fwd = window[max(0, min(len(window) - 1, idx))]
    else:
        fwd = float("nan")

    print("[%s] pose=(%.3f, %.3f) yaw=%.1f deg | forward p%.0f=%.3f m | "
          "rays<%.2fm: %d/%d (%.0f%%) | %s"
          % (label, node.pos[0], node.pos[1], math.degrees(node.yaw),
             PERCENTILE, fwd, STOP_DISTANCE, near, n, 100.0 * near / n,
             "BELOW stop distance" if fwd < STOP_DISTANCE else "clear"))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
