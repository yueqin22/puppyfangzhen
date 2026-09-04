"""Measure how the simulated base actually responds to /cmd_vel, axis by axis.

WHY THIS TOOL EXISTS
--------------------
Every axis-response claim in this project used to be a single measurement
divided by WALL-CLOCK seconds. That is only sound while the simulation runs at
1.0x real time, and nothing checked it. The same strafe command (linear.y=0.1
for 4 s) reported 104%, then 55%, then 29% across runs with identical code,
which is the signature of a measurement problem, not a robot problem.

So this probe:
  1. Times itself against the SIMULATED clock (/clock). If gzserver is starved,
     the measured efficiency stays correct instead of collapsing.
  2. Reports the real-time factor alongside every result, so a slow sim is
     visible rather than silent.
  3. Sweeps all four axes in one run, so cross-axis comparisons share a single
     environmental condition.
  4. Watches /cmd_vel for messages it did not send. track_cmd_adapter publishes
     at 20 Hz and will happily overwrite a probe command; previously that looked
     exactly like "the robot ignored me".

WHO IS DRIVING
--------------
With use_planar_move:=true (the default in this repo) the base is moved by the
Gazebo planar-move plugin, NOT by the trot gait -- no gait_controller node
exists at all. Do not read these numbers as statements about trot_gait.cpp.

Usage (inside WSL, Gazebo running, nothing else publishing /cmd_vel):
    python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py --sweep
    python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py --sweep --strafe-gain 1.22
    python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py <vx> <vy> <wz> <seconds>

--strafe-gain G multiplies only the lateral command by G before sending, to
verify a platform-compensation gain closes the loop (ratio -> 100% at G = 1/eff).
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap_deg(a):
    return math.degrees((a + math.pi) % (2 * math.pi) - math.pi)


class AxisProbe(Node):
    def __init__(self, use_sim_time=True):
        # Simulated time is requested via a parameter override at construction.
        # Do NOT instead subscribe to /clock yourself: Gazebo publishes it at
        # ~1 kHz and rclpy's spin_once handles one callback per call, so the
        # flood starves every other subscription. That cost a full measurement
        # round before it was noticed.
        overrides = []
        if use_sim_time:
            overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        super().__init__("axis_probe", parameter_overrides=overrides)

        self.pose = None
        self.foreign = 0
        self.foreign_example = None
        # Every command this probe has ever published. Compared as a set rather
        # than against a single "expected" value, so a delayed callback can
        # never be mistaken for somebody else's message.
        self._ours = set()

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _on_cmd(self, msg):
        got = (round(msg.linear.x, 6), round(msg.linear.y, 6),
               round(msg.angular.z, 6))
        if got not in self._ours:
            self.foreign += 1
            if self.foreign_example is None:
                self.foreign_example = got

    def send(self, vx, vy, wz):
        self._ours.add((round(vx, 6), round(vy, 6), round(wz, 6)))
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz
        self.pub.publish(t)

    def now(self):
        # rclpy's clock is simulated time when use_sim_time is on, and system
        # time otherwise -- so one accessor covers both cases.
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_odom(self, secs=30.0):
        end = time.time() + secs
        while time.time() < end and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.pose is not None


def stop(node, secs=1.5):
    end = time.time() + secs
    while time.time() < end:
        node.send(0.0, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)


def measure(node, vx, vy, wz, duration, label, rate=100.0, vy_gain=1.0):
    # Command starvation is a real confounder: if the driver decays the base's
    # velocity between /cmd_vel messages, a probe publishing slowly measures its
    # own publish rate rather than the robot. It looks identical to "the robot
    # is dragging its legs", so it has to be ruled out by sweeping the rate.
    interval = 1.0 / rate if rate > 0 else 0.0
    stop(node, 1.5)
    x0, y0, yaw0 = node.pose

    node.foreign = 0
    node.foreign_example = None

    t0 = node.now()
    w0 = time.time()
    # Run until the CLOCK THE ROBOT SEES has advanced by `duration`, with a
    # wall-clock cap so a stalled world cannot hang the probe.
    sent = 0
    while True:
        node.send(vx, vy * vy_gain, wz)
        sent += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.now() - t0 >= duration:
            break
        if time.time() - w0 > duration * 5 + 10:
            print("  !! sim clock advanced only %.2fs in %.0fs wall -- aborting"
                  % (node.now() - t0, time.time() - w0))
            break
        if interval:
            time.sleep(interval)
    t1 = node.now()
    w1 = time.time()
    stop(node, 1.0)

    x1, y1, yaw1 = node.pose
    dsim = t1 - t0
    dwall = w1 - w0
    rtf = dsim / dwall if dwall > 0 else 0.0

    dx, dy = x1 - x0, y1 - y0
    dyaw = wrap_deg(yaw1 - yaw0)

    # Project world displacement onto the body axes at the START heading.
    c, s = math.cos(yaw0), math.sin(yaw0)
    fwd = dx * c + dy * s      # body +x
    lat = -dx * s + dy * c     # body +y

    print()
    print("--- %s ---" % label)
    print("  command       : vx=%+.3f vy=%+.3f wz=%+.3f for %.1f s (sim) @%.0fHz"
          % (vx, vy, wz, duration, sent / max(dwall, 1e-9)))
    print("  sim elapsed   : %.2f s   wall %.2f s   RTF %.3fx" % (dsim, dwall, rtf))
    print("  body forward  : %+.3f m   (ideal %+.3f)" % (fwd, vx * dsim))
    print("  body lateral  : %+.3f m   (ideal %+.3f)" % (lat, vy * dsim))
    print("  yaw change    : %+.2f deg (ideal %+.2f)"
          % (dyaw, math.degrees(wz * dsim)))
    if node.foreign:
        print("  !! %d foreign /cmd_vel messages seen (e.g. %s); another node is"
              % (node.foreign, node.foreign_example))
        print("     overwriting this probe -- results are meaningless.")
        return None

    return {"label": label, "dsim": dsim, "rtf": rtf, "fwd": fwd, "lat": lat,
            "dyaw": dyaw, "ideal_fwd": vx * dsim, "ideal_lat": vy * dsim,
            "ideal_yaw": math.degrees(wz * dsim)}


def main():
    args = sys.argv[1:]
    rclpy.init()
    node = AxisProbe(use_sim_time=True)
    if not node.wait_odom():
        print("no /odom received; is Gazebo running?")
        rclpy.shutdown()
        return 1

    # Ask for simulated time, but verify it actually ticks. A node with
    # use_sim_time set and no /clock publisher reports a frozen clock, which
    # would make every duration loop hit the safety cap.
    node.spin(2.0)
    t0 = node.now()
    node.spin(1.0)
    if node.now() - t0 < 0.5:
        print("WARNING: use_sim_time set but the clock is frozen (no /clock).")
        print("         Falling back to wall-clock timing; efficiency numbers")
        print("         are then only valid while RTF is 1.0 -- watch the RTF")
        print("         column in the summary.")
        node.destroy_node()
        node = AxisProbe(use_sim_time=False)
        if not node.wait_odom():
            print("lost /odom after re-init")
            rclpy.shutdown()
            return 1
    else:
        print("using simulated time (RTF reported per axis)")

    rate = 100.0
    sweep = False
    vy_gain = 1.0
    positional = []
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--sweep":
            sweep = True
        elif a == "--rate":
            i += 1
            rate = float(args[i]) if i < len(args) else rate
        elif a == "--strafe-gain":
            i += 1
            vy_gain = float(args[i]) if i < len(args) else vy_gain
        else:
            positional.append(a)
        i += 1

    if sweep:
        cases = [
            (0.1, 0.0, 0.0, 4.0, "FORWARD  vx=+0.1"),
            (0.0, 0.1, 0.0, 4.0, "STRAFE +y vy=+0.1"),
            (0.0, -0.1, 0.0, 4.0, "STRAFE -y vy=-0.1"),
            (0.0, 0.0, 0.3, 6.0, "TURN     wz=+0.3"),
        ]
    elif len(positional) == 4:
        cases = [(float(positional[0]), float(positional[1]),
                  float(positional[2]), float(positional[3]), "CUSTOM")]
    else:
        print("usage: probe_strafe.py [--sweep] [--rate HZ]")
        print("       probe_strafe.py <vx> <vy> <wz> <seconds> [--rate HZ]")
        rclpy.shutdown()
        return 2

    results = []
    for vx, vy, wz, secs, label in cases:
        r = measure(node, vx, vy, wz, secs, label, rate=rate, vy_gain=vy_gain)
        if r:
            results.append(r)

    print()
    print("================ SUMMARY ================")
    print("%-18s %8s %8s %8s %10s" % ("axis", "actual", "ideal", "ratio", "RTF"))
    for r in results:
        if abs(r["ideal_fwd"]) > 1e-9:
            key, act, ideal = "forward", r["fwd"], r["ideal_fwd"]
        elif abs(r["ideal_lat"]) > 1e-9:
            key, act, ideal = "lateral", r["lat"], r["ideal_lat"]
        else:
            key, act, ideal = "yaw(deg)", r["dyaw"], r["ideal_yaw"]
        ratio = act / ideal if abs(ideal) > 1e-9 else float("nan")
        print("%-18s %8.3f %8.3f %7.0f%% %10.3f"
              % (r["label"] + " " + key, act, ideal, 100 * ratio, r["rtf"]))
    print()
    print("Read the RTF column first: a ratio near 100% at RTF 1.0 means the axis")
    print("works. A low ratio at low RTF is a measurement artefact, not a defect.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
