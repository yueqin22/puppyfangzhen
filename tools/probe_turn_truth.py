"""Is angular.z ignored by the physics, or does /odom fail to report the rotation?

Every "the platform cannot turn" conclusion so far rests on /odom, which the
Gazebo planar-move plugin publishes itself (publish_odom:=true). If the plugin's
UpdateOdometry integrates only x/y and drops yaw, then the base may be spinning
merrily while /odom insists nothing happened -- and navigation was rewritten to
crab-walk sideways for nothing.

This probe reads both sources under the same command:
  - /odom                 (what the navigation stack actually consumes)
  - /gazebo/model_states  (ground truth straight out of the physics engine)

Usage (inside WSL, Gazebo running, no other /cmd_vel publisher):
    python3 /mnt/e/puppyfangzhen/tools/probe_turn_truth.py [wz] [seconds]
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class TruthProbe(Node):
    def __init__(self, want_model="puppy"):
        super().__init__("turn_truth_probe")
        self.want_model = want_model
        self.odom_yaw = None
        self.odom_xy = None
        self.truth_yaw = None
        self.truth_xy = None
        self.model_name = None
        self.names = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        # NOTE: the gazebo_ros_state plugin publishes these at the TOP LEVEL, not
        # under a /gazebo prefix. Subscribing to "/gazebo/model_states" yields
        # nothing at all and the probe then reports a misleading "no data".
        self.create_subscription(ModelStates, "/model_states", self._states, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _odom(self, msg):
        self.odom_yaw = yaw_of(msg.pose.pose.orientation)
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def _states(self, msg):
        self.names = list(msg.name)
        # Select the robot BY NAME. Taking "the first model that is not the
        # world" silently picks a wall: walls hold a yaw of exactly 0 forever,
        # which reads as conclusive "the base does not rotate" evidence while
        # proving nothing.
        idx = None
        for i, n in enumerate(msg.name):
            if n == self.want_model:
                idx = i
                break
        if idx is None:
            return
        self.model_name = msg.name[idx]
        self.truth_yaw = yaw_of(msg.pose[idx].orientation)
        p = msg.pose[idx].position
        self.truth_xy = (p.x, p.y)

    def wait_ready(self, secs=25.0):
        end = time.time() + secs
        while time.time() < end and (self.odom_yaw is None or self.truth_yaw is None):
            rclpy.spin_once(self, timeout_sec=0.3)
        return self.odom_yaw is not None and self.truth_yaw is not None


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    model = sys.argv[3] if len(sys.argv) > 3 else "puppy"

    rclpy.init()
    node = TruthProbe(model)
    if not node.wait_ready():
        print("missing data: odom=%s model_states=%s"
              % (node.odom_yaw is not None, node.truth_yaw is not None))
        print("models seen: %s" % (node.names,))
        rclpy.shutdown()
        return 1

    print("models: %s   (tracking '%s')" % (node.names, node.model_name))
    o0, t0 = node.odom_yaw, node.truth_yaw
    o0xy, t0xy = node.odom_xy, node.truth_xy
    print("start: odom yaw=%.2f deg  truth yaw=%.2f deg"
          % (math.degrees(o0), math.degrees(t0)))
    print("commanding pure angular.z = %.2f rad/s for %.1f s" % (wz, duration))

    twist = Twist()
    twist.angular.z = wz
    start = time.time()
    while time.time() - start < duration:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    stop = time.time() + 1.0
    while time.time() < stop:
        rclpy.spin_once(node, timeout_sec=0.05)

    twist.angular.z = 0.0
    for _ in range(5):
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)

    o1, t1 = node.odom_yaw, node.truth_yaw
    o1xy, t1xy = node.odom_xy, node.truth_xy

    def delta(a, b):
        return math.degrees(((b - a + math.pi) % (2 * math.pi)) - math.pi)

    ideal = math.degrees(wz * duration)
    print("end  : odom yaw=%.2f deg  truth yaw=%.2f deg"
          % (math.degrees(o1), math.degrees(t1)))
    print()
    print("yaw change   odom  = %8.2f deg" % delta(o0, o1))
    print("yaw change   truth = %8.2f deg" % delta(t0, t1))
    print("ideal               = %8.2f deg" % ideal)
    print()
    print("xy move      odom  = %.3f m" % math.hypot(o1xy[0] - o0xy[0], o1xy[1] - o0xy[1]))
    print("xy move      truth = %.3f m" % math.hypot(t1xy[0] - t0xy[0], t1xy[1] - t0xy[1]))
    print()

    dt, do = abs(delta(t0, t1)), abs(delta(o0, o1))
    if dt > 0.5 * abs(ideal) and do < 0.2 * abs(ideal):
        print("VERDICT: THE BASE DOES TURN -- /odom IS LYING about yaw.")
        print("         Navigation was rewritten to crab-walk because of a broken")
        print("         odometry source, not because the platform cannot rotate.")
    elif dt < 0.05 * abs(ideal):
        print("VERDICT: the base really does not rotate (ground truth agrees).")
        print("         The rotation is lost in physics, not in reporting.")
    else:
        print("VERDICT: partial -- truth %.0f%% of ideal, odom %.0f%%."
              % (100 * dt / abs(ideal), 100 * do / abs(ideal)))
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
