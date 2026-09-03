"""Can the simulated Puppy rotate at all, and is it standing on its feet?

Two questions behind "the platform cannot turn":

1. Does the physics permit yaw? A pure z-torque on base_link answers this. If the
   model yaws under an external wrench, contacts and joints are not the obstacle
   and the fault lies in how /cmd_vel is applied.
2. Is the robot actually standing on its feet? With use_planar_move:=true the
   ros2_control block is NOT included, so the 12 leg joints have no controller
   and are free. A free joint cannot hold the body up, so the base may simply be
   resting on its belly -- in which case "no yaw" is a friction problem, not a
   command problem.

Usage (inside WSL, Gazebo running):
    python3 /mnt/e/puppyfangzhen/tools/probe_yaw_capability.py [torque] [seconds]
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import ApplyLinkWrench
from geometry_msgs.msg import Wrench, Vector3


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class CapabilityProbe(Node):
    def __init__(self, model="puppy"):
        super().__init__("yaw_capability_probe")
        self.model = model
        self.truth_yaw = None
        self.truth_xy = None
        self.links = None
        self.create_subscription(ModelStates, "/model_states", self._states, 10)
        self.create_subscription(LinkStates, "/link_states", self._links, 10)
        self.wrench = self.create_client(ApplyLinkWrench, "/apply_link_wrench")
        self.pub = self.create_publisher(Wrench, "/unused_wrench_topic", 1)

    def _states(self, msg):
        for i, n in enumerate(msg.name):
            if n == self.model:
                self.truth_yaw = yaw_of(msg.pose[i].orientation)
                p = msg.pose[i].position
                self.truth_xy = (p.x, p.y)
                break

    def _links(self, msg):
        if self.links is not None:
            return
        out = []
        for i, n in enumerate(msg.name):
            if self.model in n:
                out.append((n, msg.pose[i].position.z))
        self.links = out

    def wait_ready(self, secs=25.0):
        end = time.time() + secs
        while time.time() < end and (self.truth_yaw is None or self.links is None):
            rclpy.spin_once(self, timeout_sec=0.3)
        return self.truth_yaw is not None and self.links is not None


def main():
    torque = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    rclpy.init()
    node = CapabilityProbe()
    if not node.wait_ready():
        print("missing data: model_states=%s link_states=%s"
              % (node.truth_yaw is not None, node.links is not None))
        rclpy.shutdown()
        return 1

    print("=== link heights (z, metres) ===")
    zs = sorted(node.links, key=lambda t: t[1])
    for n, z in zs[:6]:
        print("  %-40s z=%+.4f" % (n, z))
    print("  ...")
    for n, z in zs[-3:]:
        print("  %-40s z=%+.4f" % (n, z))
    lowest = zs[0]
    print("lowest link: %s at z=%+.4f" % lowest)
    print("  -> a foot near z=0 means it is standing on its legs;")
    print("     a torso near z=0 means the base is resting on its belly.")

    if not node.wrench.wait_for_service(timeout_sec=10.0):
        print("no /apply_link_wrench service")
        rclpy.shutdown()
        return 1

    y0 = node.truth_yaw
    print()
    print("=== applying z-torque %.2f N*m to %s::base_link for %.1f s ==="
          % (torque, node.model, duration))
    req = ApplyLinkWrench.Request()
    req.link_name = "%s::base_link" % node.model
    req.reference_frame = "world"
    req.wrench.torque = Vector3(x=0.0, y=0.0, z=torque)
    req.start_time.sec = 0
    req.start_time.nanosec = 0
    req.duration.sec = int(duration)
    req.duration.nanosec = int((duration - int(duration)) * 1e9)
    future = node.wrench.call_async(req)

    end = time.time() + duration + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        future.result()
    except Exception as exc:  # noqa: BLE001 - the call result is informational
        print("wrench call raised: %s" % exc)

    y1 = node.truth_yaw
    delta = math.degrees(((y1 - y0 + math.pi) % (2 * math.pi)) - math.pi)
    print("yaw change under external torque = %.2f deg" % delta)
    print()
    if abs(delta) > 5.0:
        print("VERDICT: physics CAN rotate the model. The base is free to yaw,")
        print("         so angular.z is being lost by whatever applies /cmd_vel,")
        print("         not by contacts or joints.")
    else:
        print("VERDICT: even an external torque barely yaws it (%.2f deg)." % delta)
        print("         Contacts/joints are resisting rotation; a different")
        print("         /cmd_vel applier would not help on its own.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
