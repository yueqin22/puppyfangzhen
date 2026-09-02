"""Record every /cmd_vel frame for N seconds and report the lateral component.

Why this exists: the end-to-end run reached COMPLETED and the safety status showed
`vy=0.150`, but the verifier only managed 5 sparse /cmd_vel samples, all of them
linear.y == 0. That proves the adapter COMPUTED a strafe; it does not prove the
strafe was PUBLISHED to /cmd_vel. Given this project's history of "unit tests pass,
real run fails", the publish step is sampled directly here at the full 20 Hz rate
instead of inferred from the mission outcome.

Mission status is recorded on the same timer for the same reason: `ros2 node list`
and `topic info` under WSL routinely under-report discovery, so a run can look
like "no subscribers, nothing dispatched" while the mission is in fact running.
Correlating lateral frames with an actual phase trace removes that ambiguity.

Usage:  python3 probe_cmdvel_lateral.py [seconds] [output_json]
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelRecorder(Node):
    def __init__(self):
        super().__init__("cmdvel_recorder")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self._cb, 50)
        self.mission_sub = self.create_subscription(
            String, "/minicpm_robot/mission_status", self._mission_cb, 20
        )
        self.rows = []
        self.phase_seq = []
        self.last_phase = None
        self.mission_msg = ""
        self.mission_frames = 0

    def _cb(self, msg: Twist) -> None:
        self.rows.append((time.time(), msg.linear.x, msg.linear.y, msg.angular.z))

    def _mission_cb(self, msg: String) -> None:
        self.mission_frames += 1
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        phase = data.get("phase")
        if phase and phase != self.last_phase:
            self.phase_seq.append(phase)
            self.last_phase = phase
        self.mission_msg = data.get("message", "") or self.mission_msg


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
    out_path = sys.argv[2] if len(sys.argv) > 2 else ""

    rclpy.init()
    node = CmdVelRecorder()
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rows = node.rows
    elapsed = max(1e-6, time.time() - t0)
    vx_nz = sum(1 for r in rows if abs(r[1]) > 1e-6)
    vy_nz = sum(1 for r in rows if abs(r[2]) > 1e-6)
    wz_nz = sum(1 for r in rows if abs(r[3]) > 1e-6)
    vy_peak = max((abs(r[2]) for r in rows), default=0.0)
    vy_neg = min((r[2] for r in rows), default=0.0)
    vy_pos = max((r[2] for r in rows), default=0.0)

    report = {
        "duration_s": round(elapsed, 2),
        "frames": len(rows),
        "rate_hz": round(len(rows) / elapsed, 2),
        "linear_x_nonzero": vx_nz,
        "linear_y_nonzero": vy_nz,
        "angular_z_nonzero": wz_nz,
        "linear_y_peak_abs": round(vy_peak, 4),
        "linear_y_min": round(vy_neg, 4),
        "linear_y_max": round(vy_pos, 4),
        "mission_status_frames": node.mission_frames,
        "mission_phase_order": " -> ".join(node.phase_seq),
        "mission_final_phase": node.last_phase or "",
        "mission_message": node.mission_msg,
    }

    print(json.dumps(report, indent=2))
    dispatched = node.mission_frames > 0
    if not dispatched:
        print("VERDICT: INCONCLUSIVE -- no mission_status seen; nothing was dispatched,")
        print("         so this window says nothing about lateral motion.")
    elif vy_nz > 0:
        print("VERDICT: LATERAL PUBLISHED -- /cmd_vel carried non-zero linear.y")
    else:
        print("VERDICT: NO LATERAL -- linear.y stayed 0 for the whole window")

    if out_path:
        with open(out_path, "w") as fh:
            json.dump({"report": report, "frames": rows[-2000:]}, fh)

    return 0


if __name__ == "__main__":
    sys.exit(main())
