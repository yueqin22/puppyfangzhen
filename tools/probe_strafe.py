"""Measure whether the simulated base can strafe (linear.y).

Rotation is demonstrably a no-op on this platform (~1.3 deg for a command worth
103 deg), so navigation is limited to the body x-axis -- which cannot reach a
goal that sits off to the side. Before accepting that, check whether the base can
move sideways instead: gait_controller carries a vy_ term, and a planar-move
plugin in Gazebo would honour linear.y directly.

If strafing works, a goal 74 deg off the nose is reachable without ever turning,
and the "can't reach a sideways goal" limitation disappears.

Usage (inside WSL, Gazebo running):
    source /opt/ros/humble/setup.bash
    source ~/puppy_ws/install/setup.bash
    python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py [vy] [seconds]
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class StrafeProbe(Node):
    def __init__(self):
        super().__init__("strafe_probe")
        self.pose = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def wait_odom(self, secs=20.0):
        end = time.time() + secs
        while time.time() < end and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.3)
        return self.pose is not None


def main():
    vy = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

    rclpy.init()
    node = StrafeProbe()
    if not node.wait_odom():
        print("no /odom received; is Gazebo running?")
        rclpy.shutdown()
        return

    x0, y0, yaw0 = node.pose
    print("start: x=%.3f y=%.3f yaw=%.2f deg" % (x0, y0, math.degrees(yaw0)))
    print("commanding pure linear.y = %.2f m/s for %.1f s" % (vy, duration))

    twist = Twist()
    twist.linear.y = vy
    start = time.time()
    while time.time() - start < duration:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    stop = time.time() + 1.0
    while time.time() < stop:
        rclpy.spin_once(node, timeout_sec=0.05)

    x1, y1, yaw1 = node.pose
    twist.linear.y = 0.0
    for _ in range(5):
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)

    dx, dy = x1 - x0, y1 - y0
    total = math.hypot(dx, dy)
    # Project the world displacement onto the body +y axis (yaw0 + 90 deg).
    body_y = math.degrees(yaw0) + 90.0
    bx, by = math.cos(math.radians(body_y)), math.sin(math.radians(body_y))
    lateral = dx * bx + dy * by
    ideal = vy * duration

    print("end  : x=%.3f y=%.3f yaw=%.2f deg" % (x1, y1, math.degrees(yaw1)))
    print("total displacement   = %.3f m" % total)
    print("lateral (body +y)    = %.3f m   (ideal %.3f m)" % (lateral, ideal))
    print("yaw drift            = %.2f deg" % math.degrees(
        ((yaw1 - yaw0 + math.pi) % (2 * math.pi)) - math.pi))

    if abs(lateral) >= 0.5 * abs(ideal):
        print("VERDICT: STRAFE WORKS (%.0f%% of ideal). A sideways goal is reachable"
              % (100.0 * lateral / ideal if ideal else 0.0))
        print("         without ever turning -- navigation should use linear.y.")
    else:
        print("VERDICT: strafe is NOT executed (%.0f%% of ideal)."
              % (100.0 * lateral / ideal if ideal else 0.0))
        print("         Translation is limited to the body x-axis only.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
