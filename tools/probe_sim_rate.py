"""Measure the Gazebo real-time factor (RTF).

Every velocity measurement in this project divides a displacement by WALL-CLOCK
seconds. That is only valid while the simulation runs at 1.0x real time. If
gzserver is starved of CPU, 4 s of wall clock may be 1 s of simulated time, and a
perfectly working actuator reports 25% efficiency.

Observed symptom that prompted this: the same strafe command (linear.y = 0.1 for
4 s) reported 104%, then 55%, then 29% across runs with identical code. The code
did not change between those runs; the clock did.

Usage (inside WSL, Gazebo running):
    python3 /mnt/e/puppyfangzhen/tools/probe_sim_rate.py [seconds]
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rosgraph_msgs.msg import Clock


class RateProbe(Node):
    def __init__(self):
        super().__init__("sim_rate_probe")
        self.sim = None
        # Gazebo publishes /clock best-effort. A subscriber asking for RELIABLE
        # will never match a BEST_EFFORT publisher, so this silent failure looks
        # exactly like "no data". Asking for best-effort matches both.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        # NOTE: do not name this callback `_clock`. Node already carries a
        # `_clock` instance attribute (the ROSClock), and an instance attribute
        # shadows the method -- the subscription would hand the executor a
        # ROSClock object instead of a callable.
        self.create_subscription(Clock, "/clock", self._on_clock, qos)

    def _on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

    rclpy.init()
    node = RateProbe()

    end = time.time() + 20.0
    while time.time() < end and node.sim is None:
        rclpy.spin_once(node, timeout_sec=0.3)
    if node.sim is None:
        print("no /clock received -- is Gazebo publishing simulated time?")
        rclpy.shutdown()
        return 1

    s0 = node.sim
    w0 = time.time()
    while time.time() - w0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    s1 = node.sim
    w1 = time.time()

    dsim = s1 - s0
    dwall = w1 - w0
    rtf = dsim / dwall if dwall > 0 else 0.0
    print("wall clock elapsed : %.3f s" % dwall)
    print("sim clock elapsed  : %.3f s" % dsim)
    print("real-time factor   : %.3fx" % rtf)
    print()
    if rtf >= 0.95:
        print("VERDICT: simulation keeps up with real time.")
        print("         Displacement / wall-seconds is a valid velocity measure.")
    elif rtf >= 0.5:
        print("VERDICT: simulation runs at %.0f%% of real time. Velocity figures"
              % (100 * rtf))
        print("         measured against wall clock under-report by that factor.")
    else:
        print("VERDICT: simulation is starved (%.0f%% of real time). ANY velocity"
              % (100 * rtf))
        print("         or efficiency number taken against wall clock is junk.")
        print("         Divide commanded-by-actual ratios by this RTF before")
        print("         concluding that an actuator is broken.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
