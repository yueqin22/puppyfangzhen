"""Measure whether the simulated base actually executes angular.z.

The navigation adapter had to give up on rotation and drive only along the body
x-axis, because an in-place turn reportedly produced < 1 deg of yaw. That claim
decides the whole navigation strategy, so this measures it directly: stop any
other /cmd_vel publisher, command a pure yaw rate for a few seconds, and report
how much the base actually turned.

Usage (inside WSL, Gazebo running):
    source /opt/ros/humble/setup.bash
    source ~/puppy_ws/install/setup.bash
    python3 /mnt/e/puppyfangzhen/tools/probe_turn.py [wz] [seconds]
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TurnProbe(Node):
    def __init__(self):
        super().__init__("turn_probe")
        self.yaw = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))

    def wait_odom(self, secs=20.0):
        end = time.time() + secs
        while time.time() < end and self.yaw is None:
            rclpy.spin_once(self, timeout_sec=0.3)
        return self.yaw is not None


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    rclpy.init()
    node = TurnProbe()
    if not node.wait_odom():
        print("no /odom received; is Gazebo running?")
        rclpy.shutdown()
        return

    start_yaw = node.yaw
    start_wall = time.time()
    print("start yaw = %.2f deg" % math.degrees(start_yaw))
    print("commanding pure angular.z = %.2f rad/s for %.1f s" % (wz, duration))

    twist = Twist()
    twist.angular.z = wz
    samples = []
    while time.time() - start_wall < duration:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
        samples.append(node.yaw)
        time.sleep(0.05)

    # Let it coast, then read the final yaw.
    stop = time.time() + 1.0
    while time.time() < stop:
        rclpy.spin_once(node, timeout_sec=0.05)

    end_yaw = node.yaw
    twist.angular.z = 0.0
    for _ in range(5):
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)

    delta = math.degrees(((end_yaw - start_yaw + math.pi) % (2 * math.pi)) - math.pi)
    expected = math.degrees(wz * duration)
    print("end yaw   = %.2f deg" % math.degrees(end_yaw))
    print("turned    = %.2f deg  (ideal for %.2f rad/s x %.1f s = %.2f deg)"
          % (delta, wz, duration, expected))
    if abs(delta) < 3.0:
        print("VERDICT: rotation is NOT executed (turned < 3 deg).")
        print("         Navigation can only translate along the body axis.")
    else:
        print("VERDICT: rotation IS executed (%.1f%% of ideal)."
              % (100.0 * delta / expected if expected else 0.0))
        print("         The adapter's turn-free navigation may be unnecessary.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
