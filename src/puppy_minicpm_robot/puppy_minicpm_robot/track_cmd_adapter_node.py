"""Track Command Adapter Node for Puppy Robot MiniCPM-RobotTrack integration.

Receives tracking intentions, performs scaling, deadband suppression, EMA smoothing,
strict velocity clamping (vx<=0.15m/s, wz<=0.30rad/s), timeout watchdog, and LiDAR safety override.
"""

import time
import math
import json
from typing import Optional, Dict, Any, Tuple, List
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    Node = object

from .types import TrackIntent, SafetyStatus, TrackMode


class TrackCmdAdapterNode(Node):
    """ROS2 Node adapting high-level visual track intents to safe velocity commands."""

    def __init__(self, node_name: str = "track_cmd_adapter_node"):
        if HAS_RCLPY:
            super().__init__(node_name)
            self._declare_parameters()
            self._init_publishers_and_subscribers()
        else:
            self._init_mock_state()

        # Initialised here rather than inside the rclpy branch: _travel_clearance
        # reads it on every tick, including ticks before the first LaserScan
        # arrives. Leaving it undefined on the real-node path fails with
        # AttributeError at runtime while the mock path (used by the unit tests)
        # happens to define it -- the classic "all green, dead on hardware" trap.
        self.latest_scan = None

        self.last_intent: Optional[TrackIntent] = None
        self.last_intent_time = 0.0
        self.current_smoothed_vx = 0.0
        self.current_smoothed_vy = 0.0
        self.current_smoothed_wz = 0.0
        # Closest obstacle in the arc the robot drives into.
        # `latest_min_obstacle_dist` is the FORWARD arc (used when vx > 0);
        # `latest_rear_obstacle_dist` covers reversing. See scan_callback for
        # why these are arc-restricted at all.
        self.latest_min_obstacle_dist = 99.0
        self.latest_rear_obstacle_dist = 99.0
        self.safety_status = SafetyStatus()
        # Set when motion is demanded but continuously suppressed by the LiDAR
        # override; cleared as soon as the robot is free to move again.
        self.blocked_since: Optional[float] = None
        self._last_blocked_log = 0.0
        # Latest mission phase reported by mission_grounder_node. `None` means no
        # mission has been seen yet, in which case the base stands by idle (it
        # must NOT auto-follow a phantom target and ram the wall before a mission
        # is even dispatched). mission_grounder only publishes once a mission is
        # loaded, so this stays None until the first dispatch.
        self.mission_phase: Optional[str] = None
        # Navigation runtime state (populated from /goal_pose + mission_status).
        self.nav_goal: Optional[List[float]] = None   # [x, y] map frame
        self.has_nav_goal: bool = False
        self.robot_pose: Optional[List[float]] = None  # [x, y, yaw] map frame
        self.pose_frame: str = "odom"

    def _declare_parameters(self):
        self.declare_parameter("mode", "dry-run")
        self.declare_parameter("vx_positive_scale", 1.5)
        self.declare_parameter("vx_negative_scale", 2.0)
        self.declare_parameter("vx_positive_deadband", 0.03)
        self.declare_parameter("vx_negative_deadband", 0.0)
        self.declare_parameter("max_vx", 0.15)
        self.declare_parameter("wz_scale", 1.2)
        self.declare_parameter("deadband_wz", 0.0)
        self.declare_parameter("max_wz", 0.30)
        self.declare_parameter("yaw_boost", 1.0)
        self.declare_parameter("yaw_boost_threshold", 0.10)
        self.declare_parameter("yaw_boost_max", 0.30)
        self.declare_parameter("ema_alpha", 1.0)
        self.declare_parameter("cmd_timeout_ms", 350.0)
        self.declare_parameter("min_obstacle_distance", 0.35)
        self.declare_parameter("obstacle_arc_deg", 60.0)
        self.declare_parameter("obstacle_percentile", 20.0)
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("blocked_escalation_sec", 5.0)
        self.declare_parameter("blocked_log_period_sec", 5.0)
        self.declare_parameter("gate_by_mission_phase", True)
        self.declare_parameter(
            "tracking_phases", ["VISUAL_TRACKING"]
        )
        # Navigation controller (used when the mission gates the tracker but a
        # /goal_pose is available -- the adapter becomes the actuator).
        self.declare_parameter("nav_yaw_gain", 2.0)
        self.declare_parameter("nav_align_rad", 0.35)      # ~20 deg: within this, drive forward
        self.declare_parameter("nav_lin_gain", 0.5)        # vx = clamp(dist * gain, max_vx)
        self.declare_parameter("nav_arrival_m", 0.20)      # stop this far from the goal
        self.declare_parameter("nav_timeout_grace_s", 3.0) # hold if pose/goal unknown this long
        # Cap for the lateral (body +y) component. Measured on the live sim: the
        # gait ignores angular.z almost completely (a command worth 103 deg of yaw
        # turned the base 1.3 deg) but it DOES execute linear.y (0.379 m travelled
        # for a 0.400 m command, 95%). So a goal lying off the nose is reachable by
        # strafing rather than by waiting for a turn that will never happen.
        self.declare_parameter("max_vy", 0.15)
        # Escape-from-block behaviour. Stopping dead is NOT a recovery on this
        # platform: the only recovery the old code relied on was rotation, and
        # rotation does not work here (see max_vy above). So when the LiDAR vetoes
        # the commanded direction the base slides tangentially instead.
        self.declare_parameter("escape_speed", 0.12)          # m/s cap for the slide
        # A tangent is only accepted when its clearance exceeds the stop threshold
        # by this factor. Without the margin the base accepts a tangent that is
        # barely clear, immediately re-blocks, flips to the other tangent and
        # chatters in place instead of making progress.
        self.declare_parameter("escape_clearance_factor", 1.15)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.vx_positive_scale = self.get_parameter("vx_positive_scale").get_parameter_value().double_value
        self.vx_negative_scale = self.get_parameter("vx_negative_scale").get_parameter_value().double_value
        self.vx_positive_deadband = self.get_parameter("vx_positive_deadband").get_parameter_value().double_value
        self.vx_negative_deadband = self.get_parameter("vx_negative_deadband").get_parameter_value().double_value
        self.max_vx = self.get_parameter("max_vx").get_parameter_value().double_value
        self.wz_scale = self.get_parameter("wz_scale").get_parameter_value().double_value
        self.deadband_wz = self.get_parameter("deadband_wz").get_parameter_value().double_value
        self.max_wz = self.get_parameter("max_wz").get_parameter_value().double_value
        self.yaw_boost = self.get_parameter("yaw_boost").get_parameter_value().double_value
        self.yaw_boost_threshold = self.get_parameter("yaw_boost_threshold").get_parameter_value().double_value
        self.yaw_boost_max = self.get_parameter("yaw_boost_max").get_parameter_value().double_value
        self.ema_alpha = self.get_parameter("ema_alpha").get_parameter_value().double_value
        self.cmd_timeout_ms = self.get_parameter("cmd_timeout_ms").get_parameter_value().double_value
        self.min_obstacle_distance = self.get_parameter("min_obstacle_distance").get_parameter_value().double_value
        self.obstacle_arc_deg = self.get_parameter("obstacle_arc_deg").get_parameter_value().double_value
        self.obstacle_percentile = self.get_parameter("obstacle_percentile").get_parameter_value().double_value
        self.confidence_threshold = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.blocked_escalation_sec = self.get_parameter("blocked_escalation_sec").get_parameter_value().double_value
        self.blocked_log_period_sec = self.get_parameter("blocked_log_period_sec").get_parameter_value().double_value
        self.gate_by_mission_phase = self.get_parameter("gate_by_mission_phase").get_parameter_value().bool_value
        self.tracking_phases = list(
            self.get_parameter("tracking_phases").get_parameter_value().string_array_value
        )
        self.nav_yaw_gain = self.get_parameter("nav_yaw_gain").get_parameter_value().double_value
        self.nav_align_rad = self.get_parameter("nav_align_rad").get_parameter_value().double_value
        self.nav_lin_gain = self.get_parameter("nav_lin_gain").get_parameter_value().double_value
        self.nav_arrival_m = self.get_parameter("nav_arrival_m").get_parameter_value().double_value
        self.nav_timeout_grace_s = self.get_parameter("nav_timeout_grace_s").get_parameter_value().double_value
        self.max_vy = self.get_parameter("max_vy").get_parameter_value().double_value
        self.escape_speed = self.get_parameter("escape_speed").get_parameter_value().double_value
        self.escape_clearance_factor = self.get_parameter(
            "escape_clearance_factor").get_parameter_value().double_value

    def _init_mock_state(self):
        self.mode = "dry-run"
        self.vx_positive_scale = 1.5
        self.vx_negative_scale = 2.0
        self.vx_positive_deadband = 0.03
        self.vx_negative_deadband = 0.0
        self.max_vx = 0.15
        self.wz_scale = 1.2
        self.deadband_wz = 0.0
        self.max_wz = 0.30
        self.yaw_boost = 1.0
        self.yaw_boost_threshold = 0.10
        self.yaw_boost_max = 0.30
        self.ema_alpha = 1.0
        self.cmd_timeout_ms = 350.0
        self.min_obstacle_distance = 0.35
        self.obstacle_arc_deg = 60.0
        self.obstacle_percentile = 20.0
        self.confidence_threshold = 0.35
        self.publish_rate_hz = 20.0
        self.blocked_escalation_sec = 5.0
        self.blocked_log_period_sec = 5.0
        self.gate_by_mission_phase = True
        self.max_vy = 0.15
        self.tracking_phases = ["VISUAL_TRACKING"]
        self.nav_yaw_gain = 2.0
        self.nav_align_rad = 0.35
        self.nav_lin_gain = 0.5
        self.nav_arrival_m = 0.20
        self.nav_timeout_grace_s = 3.0
        self.escape_speed = 0.12
        self.escape_clearance_factor = 1.15

    def _init_publishers_and_subscribers(self):
        self.cmd_vel_safe_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.safety_pub = self.create_publisher(String, "/minicpm_robot/safety_status", 10)

        self.intent_sub = self.create_subscription(
            String, "/minicpm_robot/track_intent", self.intent_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.mission_sub = self.create_subscription(
            String, "/minicpm_robot/mission_status", self.mission_status_callback, 10
        )
        # Navigation goal from the mission layer (map frame). When the tracker is
        # gated and this is present, the adapter drives the base toward it.
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, 10
        )

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"TrackCmdAdapterNode running in mode={self.mode}, max_vx={self.max_vx}, max_wz={self.max_wz}"
        )

    def intent_callback(self, msg: 'String'):
        try:
            data = json.loads(msg.data)
            self.last_intent = TrackIntent(
                instruction=data.get("instruction", ""),
                target_detected=data.get("target_detected", False),
                confidence=float(data.get("confidence", 0.0)),
                dx=float(data.get("dx", 0.0)),
                dy=float(data.get("dy", 0.0)),
                dz=float(data.get("dz", 0.0)),
                yaw=float(data.get("yaw", 0.0)),
                raw_vx=float(data.get("raw_vx", 0.0)),
                raw_wz=float(data.get("raw_wz", 0.0)),
                scaled_vx=float(data.get("scaled_vx", 0.0)),
                scaled_wz=float(data.get("scaled_wz", 0.0)),
                latency_ms=float(data.get("latency_ms", 0.0)),
                bbox=data.get("bbox", [0.0, 0.0, 0.0, 0.0]),
                error_code=int(data.get("error_code", 0)),
                error_msg=data.get("error_msg", "")
            )
            self.last_intent_time = time.time()
        except Exception as e:
            if HAS_RCLPY:
                self.get_logger().warn(f"Failed to parse intent message: {e}")

    def mission_status_callback(self, msg: 'String'):
        """Track which mission phase we are in, to gate tracking authority."""
        try:
            data = json.loads(msg.data)
            self.mission_phase = data.get("phase")
            # Map-frame robot pose [x, y, yaw]; the frame the arrival verdict
            # uses. The adapter navigates against this, not against /odom, so the
            # goal (also map frame) and the current pose share a frame.
            rp = data.get("robot_pose")
            pf = data.get("pose_frame")
            if isinstance(rp, (list, tuple)) and len(rp) >= 2:
                self.robot_pose = [float(v) for v in rp[:3]]
                self.pose_frame = pf or self.pose_frame
        except Exception:
            pass

    def goal_pose_callback(self, msg: 'PoseStamped'):
        """Store the latest navigation goal (map frame) for the adapter to drive to."""
        if HAS_RCLPY and msg.header.frame_id and msg.header.frame_id != "map":
            # Goals are published in the map frame; warn but still use the x/y.
            self.get_logger().warn(
                f"/goal_pose frame is '{msg.header.frame_id}', expected 'map'"
            )
        p = msg.pose.position
        self.nav_goal = [float(p.x), float(p.y)]
        self.has_nav_goal = True

    def _is_phase_gated(self) -> bool:
        """True when the tracker must not drive the base.

        Tracking is only the authority during `tracking_phases` (VISUAL_TRACKING).
        Everywhere else -- including when no mission has been seen yet -- the
        tracker stays hands-off and the base holds still.

        The tracker used to command the base in every phase. Measured on the
        live sim: during NAVIGATE_TO_ZONE (goal [1.0, 0.0], i.e. behind the
        robot) the tracker still drove it forward at max_vx towards a phantom
        target at dx=+0.6, away from the goal, until the wall stopped it and the
        mission failed with a misleading "blocked by obstacle".

        And with no mission seen yet (mission_phase is None) the tracker
        auto-followed that same phantom target at startup and rammed the +x wall
        before any mission was ever dispatched. No-mission is now treated as a
        gate (standby), not as teleop-follow.
        """
        if not self.gate_by_mission_phase:
            return False
        if self.mission_phase is None:
            return True
        return self.mission_phase not in self.tracking_phases

    def scan_callback(self, msg: 'LaserScan'):
        """Update closest-obstacle distance, restricted to the arc we can drive into.

        A global `min()` over all 360 deg turns the robot into its own obstacle.
        On Puppy the body-mounted LiDAR sees all four legs; measured in an empty
        5x5 m room these are two arcs at +/-49.6..64.7 deg at ~0.12 m and two at
        +/-144.9..152.9 deg at ~0.20 m. Self-returns sit permanently below the
        0.35 m threshold, so the override latched on forever: the reported
        `obstacle_distance` was 0.10 m while the actual forward 60 deg clearance
        was 1.98-2.6 m, and the robot could never move forward at all.

        Only obstacles in the direction of travel can be hit, so the check is
        limited to a configurable arc (+/- obstacle_arc_deg/2) around straight
        ahead / straight behind.

        A plain min() inside that arc is still not enough: the rear legs
        (144.9..152.9 deg) reach into a +/-30 deg rear window, which would latch
        the override on while reversing. Self-returns are narrow (they cover
        ~10% of the arc) whereas a wall covers most of it, so the arc distance is
        taken as a percentile rather than the absolute minimum. At the default
        60 deg arc and 20th percentile this still detects anything wider than
        roughly 7 cm at the 0.35 m stop distance.
        """
        try:
            # Keep the scan so the obstacle arc can be aimed at the direction the
            # base is ACTUALLY travelling (see _travel_clearance). The precomputed
            # fore/aft values are only meaningful for pure fore/aft motion; when
            # strafing, judging a sideways move against the nose arc reports the
            # wrong clearance entirely.
            self.latest_scan = msg
            self.latest_min_obstacle_dist = self._arc_obstacle_distance(msg, 0.0)
            self.latest_rear_obstacle_dist = self._arc_obstacle_distance(msg, math.pi)
        except Exception:
            self.latest_min_obstacle_dist = 99.0
            self.latest_rear_obstacle_dist = 99.0

    def _arc_obstacle_distance(self, msg: 'LaserScan', centre_angle: float) -> float:
        """Robust closest-range within +/- obstacle_arc_deg/2 of `centre_angle`.

        Returns the `obstacle_percentile`-th percentile of the valid ranges in the
        arc, so a handful of narrow self-returns cannot masquerade as a wall.
        """
        ranges = msg.ranges
        n = len(ranges)
        if n == 0 or msg.angle_increment <= 0.0:
            return 99.0

        half = max(1, int(round(math.radians(self.obstacle_arc_deg / 2.0) / msg.angle_increment)))
        centre_idx = int(round((centre_angle - msg.angle_min) / msg.angle_increment))

        window = []
        for k in range(-half, half + 1):
            r = ranges[(centre_idx + k) % n]  # modulo handles the +/-180 deg wrap
            if math.isfinite(r) and msg.range_min < r <= msg.range_max:
                window.append(r)
        if not window:
            return 99.0

        window.sort()
        if self.obstacle_percentile <= 0.0:
            return window[0]
        idx = int(math.floor(len(window) * self.obstacle_percentile / 100.0))
        return window[min(idx, len(window) - 1)]

    def _travel_clearance(self, vx: float, vy: float) -> float:
        """Clearance in the direction the base is actually translating (body frame).

        Fore/aft motion keeps using the arcs precomputed in scan_callback, which
        is both cheaper and what the existing tests pin. Any command with a
        lateral component is judged against an arc centred on the commanded
        diagonal instead -- otherwise a strafe is vetoed by whatever sits in front
        of the nose, even when the path it is actually taking is wide open.
        """
        fore_aft = self.latest_rear_obstacle_dist if vx < 0.0 else self.latest_min_obstacle_dist
        if abs(vy) <= 1e-3 or self.latest_scan is None:
            return fore_aft
        if abs(vx) <= 1e-6 and abs(vy) <= 1e-6:
            return fore_aft
        return self._arc_obstacle_distance(self.latest_scan, math.atan2(vy, vx))

    def _escape_velocity(self, vx: float, vy: float) -> Tuple[float, float, float]:
        """Slide tangentially out of a block; returns (evx, evy, clearance).

        The previous response to a block was `vx = vy = 0` annotated "allow
        rotation". Rotation is not available on this platform: gait_controller
        accepts angular.z but does not execute it (0.3 rad/s for 6 s produced
        1.28 deg of yaw instead of 103 deg). Zeroing the translation is therefore
        not "stop and reorient", it is a deadlock -- the base pins itself against
        the obstacle until mission_grounder times the phase out and fails the
        mission. Measured in the live sim: a 6.0 s block at 0.33 m clearance,
        two seconds short of the 8 s abort.

        Strafing *is* available (95% of command), so recovery is a tangential
        slide. Both tangents are scored against the LiDAR arc they would actually
        travel through and the wider one wins; if neither is clear the base
        genuinely has nowhere to go, and standing still is the correct answer.
        """
        speed = math.hypot(vx, vy)
        if speed <= 1e-6:
            return 0.0, 0.0, 0.0

        ux, uy = vx / speed, vy / speed
        required = self.min_obstacle_distance * max(1.0, self.escape_clearance_factor)
        best_clear = 0.0
        best = None
        for tx, ty in ((-uy, ux), (uy, -ux)):  # left / right tangents
            clear = self._travel_clearance(tx, ty)
            if clear < required:
                continue
            if best is None or clear > best_clear:
                best_clear, best = clear, (tx, ty)

        if best is None:
            return 0.0, 0.0, 0.0

        tx, ty = best
        s = min(speed, self.escape_speed)
        evx = max(-self.max_vx, min(self.max_vx, s * tx))
        evy = max(-self.max_vy, min(self.max_vy, s * ty))
        return evx, evy, best_clear

    def compute_velocity(self, intent: Optional[TrackIntent], current_time: float) -> Tuple[float, float, SafetyStatus]:
        """Compute bounded, smoothed, and safety-verified linear/angular velocities."""
        status = SafetyStatus(
            is_safe=True,
            obstacle_distance=self.latest_min_obstacle_dist,
            lidar_override=False,
            timeout_stop=False,
            confidence_decay=False,
            active_override_reason="NONE"
        )

        # Lateral command. Stays 0 unless the navigation branch sets it: the
        # tracker's intent carries no lateral component, so tracker-driven motion
        # remains pure fore/aft + yaw exactly as before.
        target_vy = 0.0

        # 0. Mission phase gate. Tracking is only the authority in tracking_phases.
        # Everywhere else the tracker must not drive the base. Three sub-cases:
        #   * a /goal_pose is available -> the adapter becomes the actuator and
        #     drives toward it (navigating=True, NOT phase_gated);
        #   * no mission at all -> deliberate standby, hold still (standby=True);
        #   * mission running but not in a tracking phase -> voluntary yield
        #     (phase_gated=True).
        # The gate is checked first so the reported reason is unambiguous and the
        # hold is never mistaken for a LiDAR block.
        if self._is_phase_gated():
            if self._nav_target_available():
                target_vx, target_vy, target_wz, status = self._nav_target_velocity(status, current_time)
            elif self.mission_phase is None:
                # No mission has been dispatched: stand by idle. Do NOT auto-follow
                # a phantom target (the mock backend reports dx=+0.6 forever), or
                # the robot drives into the wall on startup.
                status.standby = True
                status.active_override_reason = (
                    "STANDBY (no active mission: tracker idle, base held)"
                )
                self.current_smoothed_vx = 0.0
                self.current_smoothed_wz = 0.0
                self.blocked_since = None
                return 0.0, 0.0, status
            else:
                status.phase_gated = True
                status.active_override_reason = (
                    f"PHASE_GATE (mission phase {self.mission_phase}: "
                    f"tracker yields to navigation)"
                )
                self.current_smoothed_vx = 0.0
                self.current_smoothed_wz = 0.0
                self.blocked_since = None
                return 0.0, 0.0, status
        else:
            # Tracker-driven (VISUAL_TRACKING, or no mission yet -> teleop follow).
            # 1. Timeout Check
            dt_ms = (current_time - self.last_intent_time) * 1000.0 if self.last_intent_time > 0 else 9999.0
            if intent is None or dt_ms > self.cmd_timeout_ms:
                status.is_safe = False
                status.timeout_stop = True
                status.active_override_reason = f"INTENT_TIMEOUT ({dt_ms:.1f}ms > {self.cmd_timeout_ms}ms)"
                self.current_smoothed_vx = 0.0
                self.current_smoothed_wz = 0.0
                return 0.0, 0.0, status

            # 2. Target Detection & Confidence Check
            if not intent.target_detected or intent.confidence < self.confidence_threshold:
                status.confidence_decay = True
                status.active_override_reason = f"LOW_CONFIDENCE ({intent.confidence:.2f} < {self.confidence_threshold})"
                self.current_smoothed_vx = 0.0
                self.current_smoothed_wz = 0.0
                return 0.0, 0.0, status

            # 3. Velocity Scaling from Intent Waypoints / Raw Velocities
            raw_vx = intent.dx * 0.5 if abs(intent.dx) > 0.01 else intent.raw_vx
            raw_wz = (intent.dy * 1.0 + intent.yaw * 0.8) if abs(intent.dy) > 0.01 else intent.raw_wz

            # Apply scaling and deadband for linear velocity
            if raw_vx > 0:
                vx = raw_vx * self.vx_positive_scale
                if vx < self.vx_positive_deadband:
                    vx = 0.0
            else:
                vx = raw_vx * self.vx_negative_scale
                if abs(vx) < self.vx_negative_deadband:
                    vx = 0.0

            # Apply scaling and boost for angular velocity
            wz = raw_wz * self.wz_scale
            if abs(wz) < self.deadband_wz:
                wz = 0.0
            elif abs(wz) > self.yaw_boost_threshold:
                wz = wz * self.yaw_boost

            # 4. Strict Clamp Limits
            target_vx = float(np.clip(vx, -self.max_vx, self.max_vx))
            target_wz = float(np.clip(wz, -self.max_wz, self.max_wz))

        # 5. EMA Filter (alpha=1.0 in default config -> identity)
        alpha = self.ema_alpha
        smoothed_vx = alpha * target_vx + (1.0 - alpha) * self.current_smoothed_vx
        smoothed_vy = alpha * target_vy + (1.0 - alpha) * self.current_smoothed_vy
        smoothed_wz = alpha * target_wz + (1.0 - alpha) * self.current_smoothed_wz
        self.current_smoothed_vx = smoothed_vx
        self.current_smoothed_vy = smoothed_vy
        self.current_smoothed_wz = smoothed_wz

        # 6. LiDAR Obstacle Safety Override (applies to BOTH tracker and navigation)
        # Judge against the arc we are actually translating into. That is not always
        # straight ahead: while strafing toward a goal that lies off the nose, the
        # relevant clearance is along that diagonal, and quoting the nose arc
        # instead is exactly what made a wide-open path look blocked.
        heading_dist = self._travel_clearance(smoothed_vx, smoothed_vy)
        status.obstacle_distance = heading_dist

        # `blocked` means: the controller wants to translate AND the LiDAR says no.
        # A robot wedged against a wall satisfies this on every single 20 Hz tick,
        # so it must be time-bounded and reported, not latched silently. When
        # navigating this is genuine (an actuator is physically obstructed); when
        # tracker-driven it means the tracker tried to ram a wall.
        blocked = (abs(smoothed_vx) > 0.0 or abs(smoothed_vy) > 0.0) \
            and heading_dist < self.min_obstacle_distance
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = current_time
            blocked_duration = max(0.0, current_time - self.blocked_since)
            status.blocked_duration_s = blocked_duration
            status.is_safe = False
            status.lidar_override = True

            # Recovery: slide tangentially around whatever is in the way. Only
            # while NAVIGATING -- a tracker that is blocked is following a target
            # it cannot reach, and standing off is the right answer there.
            escape_clear = 0.0
            escaping = False
            if status.navigating:
                evx, evy, escape_clear = self._escape_velocity(smoothed_vx, smoothed_vy)
                escaping = (abs(evx) > 1e-6 or abs(evy) > 1e-6)

            if escaping:
                smoothed_vx, smoothed_vy = evx, evy
                self.current_smoothed_vy = evy
                # Deliberately NOT escalated. Escalation is what mission_grounder
                # accumulates toward an abort, so flagging it here would fail a
                # mission that is actively working its way around the obstacle.
                # The block itself is still reported below via heading_dist, so a
                # run that spends its whole time grazing a wall stays visible.
                status.active_override_reason = (
                    f"LIDAR_SIDESTEP (blocked {blocked_duration:.1f}s at "
                    f"{heading_dist:.2f}m, sliding on {escape_clear:.2f}m clearance)"
                )
            else:
                # Nowhere to go: both tangents are blocked too (or the tracker is
                # driving, where standing off is the intended behaviour).
                if blocked_duration >= self.blocked_escalation_sec:
                    status.blocked_escalated = True
                    status.active_override_reason = (
                        f"BLOCKED_ESCALATION ({blocked_duration:.1f}s >= "
                        f"{self.blocked_escalation_sec}s, clearance {heading_dist:.2f}m)"
                    )
                    self._warn_blocked(status.active_override_reason, current_time)
                else:
                    status.active_override_reason = (
                        f"LIDAR_OBSTACLE_CLOSE ({heading_dist:.2f}m < "
                        f"{self.min_obstacle_distance}m)"
                    )

                smoothed_vx = 0.0
                smoothed_vy = 0.0
                self.current_smoothed_vy = 0.0
        else:
            # Free to move (or never asked to), so any previous block is over.
            self.blocked_since = None

        status.commanded_vy = smoothed_vy
        return smoothed_vx, smoothed_wz, status

    @staticmethod
    def _norm_angle(a: float) -> float:
        """Wrap an angle to (-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a <= -math.pi:
            a += 2.0 * math.pi
        return a

    def _nav_target_available(self) -> bool:
        """A navigation goal we can actually drive toward right now."""
        if not self.gate_by_mission_phase:
            return False
        if self.mission_phase not in ("NAVIGATE_TO_ZONE", "RETURN_HOME"):
            return False
        if not self.has_nav_goal or self.nav_goal is None:
            return False
        if self.robot_pose is None or len(self.robot_pose) < 2:
            return False
        if self.pose_frame != "map":
            # Only navigate against a map-frame pose; otherwise the alignment
            # between the goal (map) and the current pose cannot be trusted.
            return False
        return True

    def _nav_target_velocity(self, status: 'SafetyStatus', current_time: float) -> Tuple[float, float, 'SafetyStatus']:
        """Drive toward self.nav_goal using self.robot_pose (map frame).

        This sim's gait does NOT execute rotation from /cmd_vel.angular.z (an
        in-place turn command yields < 1 deg of yaw), so the robot cannot turn
        to face the goal. Instead we reach it the way a non-rotating platform
        does: express the goal in the robot body frame and drive FORWARD when it
        is ahead or BACKWARD when it is behind, along the body x-axis. The LiDAR
        override (run by the caller) still zeroes motion near walls.
        """
        status.navigating = True
        cx, cy = self.robot_pose[0], self.robot_pose[1]
        cyaw = self.robot_pose[2] if len(self.robot_pose) > 2 else 0.0
        gx, gy = self.nav_goal[0], self.nav_goal[1]
        dx = gx - cx
        dy = gy - cy
        dist = math.hypot(dx, dy)
        if dist < self.nav_arrival_m:
            status.active_override_reason = (
                f"NAV_ARRIVED dist={dist:.2f}m (deadband {self.nav_arrival_m:.2f}m)"
            )
            return 0.0, 0.0, 0.0, status

        # Goal in the robot body frame.
        c, s = math.cos(cyaw), math.sin(cyaw)
        bdx = c * dx + s * dy        # forward distance to the goal
        bdy = -s * dx + c * dy       # left distance to the goal
        bearing = math.atan2(bdy, bdx)

        # Drive along the body-frame BEARING of the goal rather than picking
        # forward-or-reverse from the sign of bdx.
        #
        # With fore/aft only, a goal lying off the nose is approached by driving
        # into whatever happens to be straight ahead. Measured live: the goal sat
        # 74 deg off the nose with 2.6-4.7 m clear along that bearing but only
        # 0.33 m clear straight ahead, so every tick was refused by the LiDAR and
        # the mission aborted as "blocked by obstacle" after 8 s with the base
        # displaced by 0.015 m. Strafing is executed by this gait (0.379 m for a
        # 0.400 m command), so split the speed across both body axes.
        speed = min(self.max_vx, dist * self.nav_lin_gain)
        vx = speed * (bdx / dist)
        vy = max(-self.max_vy, min(self.max_vy, speed * (bdy / dist)))

        # Rotation stays a gentle correction proportional to the lateral OFFSET,
        # not to the bearing angle. Keying it off the angle would command a full
        # 180 deg spin for a goal directly behind while simultaneously reversing
        # toward it -- self-contradictory on any platform that honours angular.z.
        # It remains a no-op on this sim, which ignores angular.z entirely.
        wz = max(-self.max_wz, min(self.max_wz, bdy * self.nav_yaw_gain))
        status.active_override_reason = (
            f"NAV_MOVE dist={dist:.2f}m bearing={math.degrees(bearing):.0f}deg "
            f"vx={vx:.3f} vy={vy:.3f}"
        )
        return vx, vy, wz, status

    def _warn_blocked(self, reason: str, now: float):
        """Throttled operator-visible warning for a sustained motion block."""
        if not HAS_RCLPY:
            return
        if now - self._last_blocked_log < self.blocked_log_period_sec:
            return
        self._last_blocked_log = now
        self.get_logger().warn(f"[Safety] {reason}")

    def timer_callback(self):
        now = time.time()
        vx, wz, status = self.compute_velocity(self.last_intent, now)
        self.safety_status = status

        twist_msg = Twist()
        twist_msg.linear.x = float(vx)
        # Lateral command, non-zero only while strafing toward a navigation goal.
        # The gait ignores angular.z on this platform but honours linear.y, so this
        # is how a goal lying off the nose is reached without turning.
        twist_msg.linear.y = float(status.commanded_vy)
        twist_msg.angular.z = float(wz)

        # In dry-run mode, cmd_vel_safe outputs commanded velocity for monitoring,
        # but live cmd_vel is strictly suppressed to 0.
        if HAS_RCLPY:
            if hasattr(self, 'cmd_vel_safe_pub'):
                self.cmd_vel_safe_pub.publish(twist_msg)

            if hasattr(self, 'cmd_vel_pub'):
                if self.mode in [TrackMode.SIM.value, TrackMode.LIVE_ARM.value]:
                    self.cmd_vel_pub.publish(twist_msg)
                else:
                    # Dry-run: safe zero output
                    zero_msg = Twist()
                    self.cmd_vel_pub.publish(zero_msg)

            if hasattr(self, 'safety_pub'):
                msg = String()
                msg.data = json.dumps(status.to_dict())
                self.safety_pub.publish(msg)


def main(args=None):
    if not HAS_RCLPY:
        print("rclpy is not available in current environment")
        return
    rclpy.init(args=args)
    node = TrackCmdAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
