"""Characterise the near returns that sit on the 0.35 m stop threshold.

Context: at rest the adapter's forward arc reads 0.27-0.28 m and 26% of all rays
are closer than the 0.35 m stop distance, so forward motion is permanently
marginal -- a mission can still complete, but only after a spurious
BLOCKED_ESCALATION. Three candidate sources look identical in the single
`obstacle_distance` number and need different fixes:

  * the robot's own body/legs  -> narrow angular clusters, stable in the BODY frame
  * the floor (laser pitched down / base low) -> broad, short-range, stable
  * a real nearby wall         -> broad cluster, stable in the WORLD frame

This probe separates them by measuring, for each ray, how near it is, how WIDE
the contiguous near clusters are in degrees, and how stable they are over time.
It also reports the percentile the forward arc would need in order to clear the
stop distance, which is the number any fix has to move.

Usage (WSL, sim already running):
    source /opt/ros/humble/setup.bash
    source ~/puppy_ws/install/setup.bash
    python3 /mnt/e/puppyfangzhen/tools/diagnose_selfreturns.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

ARC_DEG = 60.0
STOP_DISTANCE = 0.35
SECTOR_DEG = 10


class Probe(Node):
    def __init__(self):
        super().__init__("selfreturn_probe")
        self.scans = []
        self.create_subscription(LaserScan, "/scan", self._scan, 10)

    def _scan(self, msg):
        self.scans.append(msg)
        if len(self.scans) > 40:
            self.scans.pop(0)


def _runs(flags):
    """Contiguous True runs in a circular list -> (start_idx, length)."""
    n = len(flags)
    if n == 0:
        return []
    # rotate so a run never straddles the seam
    start = 0
    for i in range(n):
        if not flags[i]:
            start = i
            break
    else:
        return [(0, n)]
    rotated = [flags[(start + k) % n] for k in range(n)]
    runs, i = [], 0
    while i < n:
        if rotated[i]:
            j = i
            while j < n and rotated[j]:
                j += 1
            runs.append(((start + i) % n, j - i))
            i = j
        else:
            i += 1
    return runs


def main():
    rclpy.init()
    node = Probe()
    deadline = time.time() + 20
    while time.time() < deadline and len(node.scans) < 12:
        rclpy.spin_once(node, timeout_sec=0.3)

    if not node.scans:
        print("MISSING: no /scan received")
        rclpy.shutdown()
        return

    scan = node.scans[-1]
    ranges = list(scan.ranges)
    n = len(ranges)
    inc_deg = math.degrees(scan.angle_increment)
    print("scan: rays=%d  inc=%.2f deg  range=[%.2f, %.2f]  frames_collected=%d"
          % (n, inc_deg, scan.range_min, scan.range_max, len(node.scans)))

    near = [r == r and scan.range_min < r < STOP_DISTANCE for r in ranges]
    n_near = sum(near)
    print("rays closer than %.2f m: %d / %d (%.1f%%)"
          % (STOP_DISTANCE, n_near, n, 100.0 * n_near / n))

    # --- per-sector profile -------------------------------------------------
    print()
    print("sector(deg body)   median   min    max    near%%")
    per_sector = SECTOR_DEG / inc_deg
    for s in range(0, int(round(360 / SECTOR_DEG))):
        lo, hi = int(s * per_sector), int((s + 1) * per_sector)
        vals = [r for r in ranges[lo:hi] if r == r and r > 0.0]
        if not vals:
            continue
        near_cnt = sum(1 for r in vals if r < STOP_DISTANCE)
        print("  %3d .. %3d        %6.3f %6.3f %6.3f   %4.0f%%"
              % (s * SECTOR_DEG, (s + 1) * SECTOR_DEG,
                 sorted(vals)[len(vals) // 2], min(vals), max(vals),
                 100.0 * near_cnt / len(vals)))

    # --- near clusters ------------------------------------------------------
    runs = _runs(near)
    runs.sort(key=lambda kv: -kv[1])
    print()
    print("near clusters (sorted by angular width):")
    print("  width_deg  centre_deg   mean_range")
    for idx, length in runs[:12]:
        centre = math.degrees(scan.angle_min + (idx + length / 2.0) * scan.angle_increment)
        centre = ((centre + 180.0) % 360.0) - 180.0
        vals = [ranges[(idx + k) % n] for k in range(length)]
        vals = [v for v in vals if v == v and v > 0.0]
        print("   %6.1f     %7.1f      %6.3f"
              % (length * inc_deg, centre, sum(vals) / max(1, len(vals))))

    if runs:
        widest = runs[0][1] * inc_deg
        print()
        print("widest near cluster: %.1f deg" % widest)
        if widest <= 15.0:
            print("  -> narrow: consistent with the robot's own legs/body (self-return).")
        elif widest >= 60.0:
            print("  -> broad: consistent with a wall or the floor, not a leg.")
        else:
            print("  -> intermediate: check whether it tracks the body or the world.")

    # --- temporal stability -------------------------------------------------
    if len(node.scans) >= 8:
        diffs = []
        for a, b in zip(node.scans[-8:-1], node.scans[-8 + 1:]):
            for ra, rb in zip(a.ranges, b.ranges):
                if ra == ra and rb == rb:
                    diffs.append(abs(ra - rb))
        if diffs:
            print()
            print("frame-to-frame range change: mean=%.4f m  max=%.3f m"
                  % (sum(diffs) / len(diffs), max(diffs)))
            print("  (static geometry stays near 0 regardless of the source)")

    # --- what percentile would the forward arc need? ------------------------
    print()
    print("forward arc (0 deg body, +/-%.0f deg), adapter uses the 20th percentile:"
          % (ARC_DEG / 2))
    half = int(round(math.radians(ARC_DEG / 2.0) / scan.angle_increment))
    centre_idx = int(round((0.0 - scan.angle_min) / scan.angle_increment))
    window = []
    for k in range(-half, half + 1):
        r = ranges[(centre_idx + k) % n]
        if r == r and scan.range_min < r <= scan.range_max:
            window.append(r)
    window.sort()
    if window:
        for pct in (0, 10, 20, 30, 40, 50, 60, 80, 100):
            idx = int(round(pct / 100.0 * (len(window) - 1)))
            v = window[max(0, min(len(window) - 1, idx))]
            flag = "  <-- below stop distance" if v < STOP_DISTANCE else ""
            print("   p%-3d = %.3f m%s" % (pct, v, flag))
        needed = None
        for pct in range(0, 101):
            idx = int(round(pct / 100.0 * (len(window) - 1)))
            if window[idx] >= STOP_DISTANCE:
                needed = pct
                break
        if needed is None:
            print("   no percentile clears %.2f m: the arc really is obstructed"
                  % STOP_DISTANCE)
        else:
            print("   smallest percentile clearing %.2f m: p%d" % (STOP_DISTANCE, needed))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
