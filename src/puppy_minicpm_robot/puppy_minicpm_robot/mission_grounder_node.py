"""Mission Grounder Node for Puppy Robot MiniCPM-RobotTrack integration.

Grounds natural language commands into multi-stage structured missions:
1. Parse Task -> 2. Global Navigation / Patrol -> 3. MiniCPM Visual Tracking -> 4. Inspect & Verify -> 5. Return Home
"""

import time
import math
import json
import re
from typing import Dict, Any, Optional, List, Tuple
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    Node = object

try:
    import tf2_ros
    from tf2_ros import TransformException
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False

from .types import VisualMission, MissionPhase, TrackIntent


DEFAULT_ZONE_REGISTRY: Dict[str, List[float]] = {
    "backyard": [2.0, -2.0, 0.0],
    "后院": [2.0, -2.0, 0.0],
    "living_room": [1.0, 0.0, 0.0],
    "客厅": [1.0, 0.0, 0.0],
    "corridor": [0.0, 1.5, 1.57],
    "走廊": [0.0, 1.5, 1.57],
    "kitchen": [-1.5, 1.0, 3.14],
    "厨房": [-1.5, 1.0, 3.14],
    "home": [0.0, 0.0, 0.0],
    "原点": [0.0, 0.0, 0.0],
    "充电站": [0.0, 0.0, 0.0],
}


class MissionGrounderNode(Node):
    """ROS2 Node grounding natural language instructions into actionable robot mission phases."""

    def __init__(self, node_name: str = "mission_grounder_node"):
        if HAS_RCLPY:
            super().__init__(node_name)
            self._init_tf_state()
            self._declare_parameters()
            self._init_publishers_and_subscribers()
        else:
            self._init_mock_state()
            self._init_tf_state()

        self.current_mission: Optional[VisualMission] = None
        self.phase_start_time = time.time()
        # `robot_pose` is the ODOM-FRAME pose (fallback). `map_pose` is the
        # MAP-FRAME pose from TF and takes precedence for navigation decisions.
        # Zone/return waypoints are map-frame and goals are published with
        # frame_id "map", so comparing them against an odom-frame position is a
        # frame mismatch. Measured on the live sim: map->odom was
        # [-0.152, -0.056] with a -3.27 deg yaw, putting the robot at
        # (2.057, -0.190) in map vs (2.213, -0.008) in odom -- 0.24 m apart,
        # i.e. 60% of the 0.4 m arrival radius.
        self.robot_pose = [0.0, 0.0, 0.0]
        self.map_pose: Optional[List[float]] = None
        self.zone_registry = dict(DEFAULT_ZONE_REGISTRY)
        # Latest safety report from the track adapter, plus how long the robot
        # has been continuously blocked by an obstacle (motion demanded, LiDAR
        # refusing). See `safety_callback`.
        self.latest_safety: Dict[str, Any] = {}
        self.blocked_since: Optional[float] = None
        # Deadline for the current navigation phase, armed on entry. Falls back
        # to the base timeout when a phase is entered without arming (e.g. tests
        # that assign `current_mission` directly).
        self.phase_timeout: float = self.navigation_timeout_sec

    def _init_tf_state(self):
        """Declare TF state BEFORE the listener is constructed.

        `_init_publishers_and_subscribers` creates the TransformListener bound to
        `self.tf_buffer`. These attributes used to be assigned later in
        `__init__`, which silently discarded that buffer: `update_map_pose()`
        then short-circuited on "tf2 unavailable" for every tick and the node
        navigated in the odom frame while comparing against map-frame goals --
        with no exception raised anywhere. Unit tests stayed green because they
        assign `map_pose` directly and never exercise the real lookup.
        """
        self.tf_buffer = None
        self.tf_listener = None
        # Consecutive TF failures and the last time one was logged (see
        # `_log_tf_failure`). Without this the node silently navigates in the
        # odom frame while the goals it compares against are map-frame.
        self._tf_failures = 0
        self._tf_last_log = 0.0

    def _declare_parameters(self):
        self.declare_parameter("default_zone", "backyard")
        self.declare_parameter("default_dwell_sec", 5.0)
        self.declare_parameter("tracking_timeout_sec", 20.0)
        # Base (floor) for the navigation deadline; the actual deadline is
        # distance-derived, see `_arm_phase_timeout`.
        self.declare_parameter("navigation_timeout_sec", 15.0)
        self.declare_parameter("return_timeout_sec", 15.0)
        self.declare_parameter("navigation_speed_mps", 0.10)
        self.declare_parameter("navigation_slack", 2.0)
        self.declare_parameter("blocked_abort_sec", 8.0)
        self.declare_parameter("arrival_radius_m", 0.4)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tf_lookup_timeout_sec", 0.05)

        self.default_zone = self.get_parameter("default_zone").get_parameter_value().string_value
        self.default_dwell_sec = self.get_parameter("default_dwell_sec").get_parameter_value().double_value
        self.tracking_timeout_sec = self.get_parameter("tracking_timeout_sec").get_parameter_value().double_value
        self.navigation_timeout_sec = self.get_parameter("navigation_timeout_sec").get_parameter_value().double_value
        self.return_timeout_sec = self.get_parameter("return_timeout_sec").get_parameter_value().double_value
        self.navigation_speed_mps = self.get_parameter("navigation_speed_mps").get_parameter_value().double_value
        self.navigation_slack = self.get_parameter("navigation_slack").get_parameter_value().double_value
        self.blocked_abort_sec = self.get_parameter("blocked_abort_sec").get_parameter_value().double_value
        self.arrival_radius_m = self.get_parameter("arrival_radius_m").get_parameter_value().double_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.tf_lookup_timeout_sec = self.get_parameter("tf_lookup_timeout_sec").get_parameter_value().double_value

    def _init_mock_state(self):
        self.default_zone = "backyard"
        self.default_dwell_sec = 5.0
        self.tracking_timeout_sec = 20.0
        self.navigation_timeout_sec = 15.0
        self.return_timeout_sec = 15.0
        self.navigation_speed_mps = 0.10
        self.navigation_slack = 2.0
        self.blocked_abort_sec = 8.0
        self.arrival_radius_m = 0.4
        self.map_frame = "map"
        self.base_frame = "base_footprint"
        self.tf_lookup_timeout_sec = 0.05

    def _init_publishers_and_subscribers(self):
        self.mission_status_pub = self.create_publisher(String, "/minicpm_robot/mission_status", 10)
        self.instruction_pub = self.create_publisher(String, "/minicpm_robot/instruction", 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.mission_cmd_sub = self.create_subscription(
            String, "/mission/command", self.mission_cmd_callback, 10
        )
        self.intent_sub = self.create_subscription(
            String, "/minicpm_robot/track_intent", self.intent_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.safety_sub = self.create_subscription(
            String, "/minicpm_robot/safety_status", self.safety_callback, 10
        )

        # TF listener for the map-frame robot pose (see `robot_pose` comment).
        if HAS_TF2:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("MissionGrounderNode started and ready for natural language tasks.")

    def parse_instruction(self, raw_instruction: str) -> VisualMission:
        """Parse natural language command into structured mission parameters."""
        raw_text = raw_instruction.strip()
        lower_text = raw_text.lower()

        # Handle cancellation
        if any(w in lower_text for w in ["cancel", "abort", "stop", "取消", "终止"]):
            return VisualMission(
                mission_id=f"mission_{int(time.time())}",
                raw_instruction=raw_text,
                phase=MissionPhase.ABORTED,
                message="Mission cancelled by operator"
            )

        # Detect zone
        matched_zone = self.default_zone
        for zone_key in self.zone_registry.keys():
            if zone_key in raw_text or zone_key in lower_text:
                matched_zone = zone_key
                break

        zone_pose = self.zone_registry.get(matched_zone, [0.0, 0.0, 0.0])

        # Detect target
        target_name = "person"
        if "可疑" in raw_text or "suspicious" in lower_text:
            target_name = "suspicious_item"
        elif "person" in lower_text or "人" in raw_text:
            target_name = "person"
        elif "box" in lower_text or "箱" in raw_text or "包" in raw_text:
            target_name = "package"

        # Detect dwell time
        dwell_match = re.search(r"(\d+)\s*(?:秒|s|sec|seconds)", lower_text)
        dwell_time = float(dwell_match.group(1)) if dwell_match else self.default_dwell_sec

        mission = VisualMission(
            mission_id=f"mission_{int(time.time())}",
            raw_instruction=raw_text,
            target_name=target_name,
            target_category="security_inspection" if "可疑" in raw_text else "general_tracking",
            zone_name=matched_zone,
            zone_pose=zone_pose,
            dwell_time_sec=dwell_time,
            return_pose=[0.0, 0.0, 0.0],
            phase=MissionPhase.NAVIGATE_TO_ZONE,
            message=f"Dispatched to {matched_zone} to track {target_name}"
        )
        return mission

    def mission_cmd_callback(self, msg: 'String'):
        raw_cmd = msg.data.strip()
        if not raw_cmd:
            return

        mission = self.parse_instruction(raw_cmd)
        self.current_mission = mission
        self.phase_start_time = time.time()
        self.blocked_since = None

        if HAS_RCLPY:
            self.get_logger().info(f"Loaded new mission: {mission.mission_id} -> {mission.phase.value}")

        if mission.phase == MissionPhase.NAVIGATE_TO_ZONE:
            self.dispatch_goal_pose(mission.zone_pose)
            self._arm_phase_timeout(mission.zone_pose)

    def dispatch_goal_pose(self, pose: List[float]):
        """Publish global goal pose for Nav2 / patrol."""
        if not HAS_RCLPY or not hasattr(self, 'goal_pose_pub'):
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = 0.0
        # Quaternion from yaw
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
        msg.pose.orientation.z = float(np.sin(yaw / 2.0))
        msg.pose.orientation.w = float(np.cos(yaw / 2.0))
        self.goal_pose_pub.publish(msg)

    def intent_callback(self, msg: 'String'):
        # Could track target detection events to advance state
        pass

    def odom_callback(self, msg: 'Odometry'):
        self.robot_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            0.0
        ]

    def safety_callback(self, msg: 'String'):
        """Track how long the robot has been continuously blocked by an obstacle.

        The adapter reports `blocked_escalated` once motion has been demanded but
        suppressed for longer than its escalation window, which filters out the
        momentary overrides caused by a person walking through the field of view.
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        self.latest_safety = data
        if data.get("lidar_override") and data.get("blocked_escalated"):
            if self.blocked_since is None:
                self.blocked_since = time.time()
        else:
            self.blocked_since = None

    def blocked_for(self, now: float) -> float:
        if self.blocked_since is None:
            return 0.0
        return max(0.0, now - self.blocked_since)

    def update_map_pose(self) -> bool:
        """Refresh the map-frame robot pose from TF. Returns True on success."""
        if not HAS_RCLPY or not HAS_TF2 or self.tf_buffer is None:
            self._log_tf_failure("tf2 unavailable")
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except Exception as exc:
            # Covers TransformException (frame missing / extrapolation) and any
            # tf2 backend error. Keep the last known pose until TF recovers.
            # The failure is reported rather than swallowed: a silent fallback to
            # the odom frame reintroduces the 0.24 m arrival error, and it is
            # invisible from the status topic alone.
            self._log_tf_failure(f"{type(exc).__name__}: {exc}")
            return False

        if self._tf_failures:
            self._tf_failures = 0
            if HAS_RCLPY:
                self.get_logger().info(
                    f"TF {self.map_frame}->{self.base_frame} recovered; "
                    "arrival judged in map frame again."
                )

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.map_pose = [t.x, t.y, yaw]
        return True

    def _log_tf_failure(self, detail: str):
        """Report TF failures on a backoff instead of dropping them silently.

        The lookup runs at the timer rate (5 Hz), so an un-throttled warning
        would flood the log; but hiding it entirely means the node quietly
        navigates in the wrong frame with no trace.
        """
        self._tf_failures += 1
        if not HAS_RCLPY:
            return
        # Log the 1st failure, then back off to at most one line every 10 s.
        if self._tf_failures == 1 or time.time() - self._tf_last_log >= 10.0:
            self._tf_last_log = time.time()
            self.get_logger().warn(
                f"TF {self.map_frame}->{self.base_frame} unavailable "
                f"({self._tf_failures} consecutive): {detail}. "
                "Falling back to the ODOM frame -- arrival radius is compromised."
            )

    def nav_pose(self) -> List[float]:
        """Pose to navigate with: map frame when available, else odom fallback."""
        return self.map_pose if self.map_pose is not None else self.robot_pose

    def nav_frame(self) -> str:
        """Frame `nav_pose()` is expressed in, for status reporting."""
        return self.map_frame if self.map_pose is not None else "odom"

    def _arm_phase_timeout(self, target_pose: List[float], base: Optional[float] = None):
        """Deadline for reaching `target_pose`, derived from distance and speed.

        A fixed deadline cannot serve a quadruped capped at max_vx = 0.15 m/s:
        crossing the 5x5 m test room takes 13-33 s, so a flat 10 s navigation
        timeout fails legitimate traversals. It used to be masked because
        hitting the timeout advanced the mission (i.e. counted as "arrived"),
        which is precisely the false-success bug fixed above; simply turning it
        into a failure would have replaced false success with false failure.

        The deadline is armed once on entry rather than recomputed per tick,
        otherwise it would shrink as the robot closes in and fire prematurely.
        """
        pose = self.nav_pose()
        dist = math.hypot(
            pose[0] - target_pose[0],
            pose[1] - target_pose[1],
        )
        speed = max(self.navigation_speed_mps, 1e-3)
        floor = self.navigation_timeout_sec if base is None else base
        self.phase_timeout = floor + (dist / speed) * self.navigation_slack

    def _no_actuator_hint(self) -> str:
        """Explain a navigation timeout when nothing was driving the base.

        `/goal_pose` is published for Nav2 / a patrol node. When no such node is
        running and the track_cmd_adapter is not itself navigating, the phase has
        no actuator at all, and now that the tracker yields during navigation
        phases the robot simply stands still. Without this hint the operator sees
        a bare timeout and has no idea the navigator is missing. If the adapter
        reports `navigating`, it IS the actuator, so no hint is needed (a timeout
        then means the robot was genuinely unable to reach the goal).
        """
        if not self.latest_safety.get("phase_gated"):
            return ""
        if self.latest_safety.get("navigating"):
            return ""
        return (
            " -- the tracker yielded to navigation (phase gate) and no node "
            "consumed /goal_pose, so nothing drove the base"
        )

    def _fail(self, message: str, now: float):
        self.current_mission.phase = MissionPhase.FAILED
        self.current_mission.message = message
        self.phase_start_time = now
        if HAS_RCLPY:
            self.get_logger().error(f"Mission {self.current_mission.mission_id} FAILED: {message}")

    def timer_callback(self):
        now = time.time()

        # Keep the map-frame pose warm even while idle. The lookup used to run
        # only once a mission was already active, so the TF buffer was still
        # cold on the first ticks after dispatch and arrival fell back to the
        # odom frame during exactly the window that matters most.
        self.update_map_pose()

        if not self.current_mission:
            return

        elapsed_in_phase = now - self.phase_start_time
        phase = self.current_mission.phase

        # Terminal phases are latched: keep republishing, stop re-evaluating.
        if phase in (MissionPhase.COMPLETED, MissionPhase.FAILED, MissionPhase.ABORTED):
            self._publish_mission_status()
            return

        blocked_for = self.blocked_for(now)
        # Authoritative pose for navigation; falls back to /odom when TF is down.
        pose = self.nav_pose()

        if phase == MissionPhase.NAVIGATE_TO_ZONE:
            # Check distance to zone goal
            dist = math.hypot(
                pose[0] - self.current_mission.zone_pose[0],
                pose[1] - self.current_mission.zone_pose[1]
            )
            if dist < self.arrival_radius_m:
                self.current_mission.phase = MissionPhase.VISUAL_TRACKING
                self.phase_start_time = now
                self.current_mission.message = f"Zone reached. Starting visual tracking for {self.current_mission.target_name}"
                # Update tracking instruction
                self._publish_instruction(f"Follow the {self.current_mission.target_name} ahead")
            elif blocked_for >= self.blocked_abort_sec:
                # The robot is physically wedged; waiting out the navigation
                # timeout would only delay the report.
                self._fail(
                    f"Navigation aborted: blocked by obstacle for {blocked_for:.1f}s "
                    f"(>= {self.blocked_abort_sec}s), still {dist:.2f}m from {self.current_mission.zone_name}",
                    now,
                )
            elif elapsed_in_phase > self.phase_timeout:
                # Previously this silently advanced to VISUAL_TRACKING, so a robot
                # that never left the start still "reached" its zone.
                self._fail(
                    f"Navigation to {self.current_mission.zone_name} timed out after "
                    f"{elapsed_in_phase:.1f}s (deadline {self.phase_timeout:.1f}s); "
                    f"still {dist:.2f}m away{self._no_actuator_hint()}",
                    now,
                )

        elif phase == MissionPhase.VISUAL_TRACKING:
            if elapsed_in_phase > self.tracking_timeout_sec:
                self.current_mission.phase = MissionPhase.INSPECT_VERIFY
                self.phase_start_time = now
                self.current_mission.message = "Visual tracking completed. Inspecting and verifying target."
                self._publish_instruction("Stop and inspect target")

        elif phase == MissionPhase.INSPECT_VERIFY:
            if elapsed_in_phase >= self.current_mission.dwell_time_sec:
                self.current_mission.phase = MissionPhase.RETURN_HOME
                self.phase_start_time = now
                self.current_mission.message = "Inspection complete. Returning to home base."
                self.dispatch_goal_pose(self.current_mission.return_pose)
                self._arm_phase_timeout(
                    self.current_mission.return_pose, base=self.return_timeout_sec
                )
                self._publish_instruction("Return to home")

        elif phase == MissionPhase.RETURN_HOME:
            dist = math.hypot(
                pose[0] - self.current_mission.return_pose[0],
                pose[1] - self.current_mission.return_pose[1]
            )
            if dist < self.arrival_radius_m:
                # Only an actual arrival counts as success.
                self.current_mission.phase = MissionPhase.COMPLETED
                self.phase_start_time = now
                self.current_mission.message = "Mission completed successfully."
            elif blocked_for >= self.blocked_abort_sec:
                self._fail(
                    f"Return home aborted: blocked by obstacle for {blocked_for:.1f}s "
                    f"(>= {self.blocked_abort_sec}s), still {dist:.2f}m from home",
                    now,
                )
            elif elapsed_in_phase > self.phase_timeout:
                # Previously reported "Mission completed successfully" even with
                # the robot stranded metres away from home.
                self._fail(
                    f"Return home timed out after {elapsed_in_phase:.1f}s "
                    f"(deadline {self.phase_timeout:.1f}s); "
                    f"still {dist:.2f}m from home{self._no_actuator_hint()}",
                    now,
                )

        self._publish_mission_status()

    def _publish_instruction(self, instruction: str):
        if not HAS_RCLPY or not hasattr(self, 'instruction_pub'):
            return
        msg = String()
        msg.data = instruction
        self.instruction_pub.publish(msg)

    def _sync_pose_report(self):
        """Record the pose arrival is judged against, and the frame it is in.

        Arrival used to be computed from /odom while goals were map-frame, and
        nothing exposed which one was in use. Operators need to see both the
        coordinates and their frame to trust an "arrived" verdict.
        """
        if self.current_mission is None:
            return
        self.current_mission.robot_pose = list(self.nav_pose())
        self.current_mission.pose_frame = self.nav_frame()

    def _publish_mission_status(self):
        if not HAS_RCLPY or not hasattr(self, 'mission_status_pub'):
            return
        if self.current_mission is None:
            return
        self._sync_pose_report()
        msg = String()
        msg.data = json.dumps(self.current_mission.to_dict())
        self.mission_status_pub.publish(msg)


def main(args=None):
    if not HAS_RCLPY:
        print("rclpy is not available in current environment")
        return
    rclpy.init(args=args)
    node = MissionGrounderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
