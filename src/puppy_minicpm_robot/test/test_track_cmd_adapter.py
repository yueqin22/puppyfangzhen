"""Unit tests for Track Command Adapter Node (safety, smoothing, scaling, and timeout)."""

import pytest
import time
from puppy_minicpm_robot.types import TrackIntent, TrackMode
from puppy_minicpm_robot.track_cmd_adapter_node import TrackCmdAdapterNode


class TestTrackCmdAdapter:
    @pytest.fixture
    def adapter(self):
        node = TrackCmdAdapterNode()
        node.mode = "dry-run"
        node.max_vx = 0.15
        node.max_wz = 0.30
        node.vx_positive_scale = 1.5
        node.wz_scale = 1.2
        node.vx_positive_deadband = 0.03
        node.cmd_timeout_ms = 350.0
        node.min_obstacle_distance = 0.35
        node.obstacle_arc_deg = 60.0
        node.obstacle_percentile = 20.0
        node.confidence_threshold = 0.35
        return node

    def test_velocity_clamping_and_scaling(self, adapter):
        # Large forward command should clamp to max_vx (0.15)
        intent = TrackIntent(
            instruction="Go fast forward",
            target_detected=True,
            confidence=0.9,
            dx=2.0,
            dy=0.0,
            raw_vx=1.0
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert vx <= 0.15
        assert vx > 0.0
        assert status.is_safe is True

    def test_deadband_suppression(self, adapter):
        # Tiny forward command below deadband should output 0.0
        intent = TrackIntent(
            instruction="Tiny creep",
            target_detected=True,
            confidence=0.9,
            dx=0.01,
            dy=0.0,
            raw_vx=0.01
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert vx == 0.0

    def test_timeout_stop(self, adapter):
        # Intent older than 350ms must trigger timeout stop
        intent = TrackIntent(
            instruction="Follow person",
            target_detected=True,
            confidence=0.9,
            dx=0.5,
            dy=0.0,
            raw_vx=0.1
        )
        t_past = time.time() - 0.500  # 500ms ago (> 350ms)
        adapter.last_intent_time = t_past

        vx, wz, status = adapter.compute_velocity(intent, time.time())
        assert vx == 0.0
        assert wz == 0.0
        assert status.timeout_stop is True
        assert status.is_safe is False

    def test_low_confidence_stop(self, adapter):
        intent = TrackIntent(
            instruction="Lost target",
            target_detected=True,
            confidence=0.20,  # < 0.35 threshold
            dx=0.5,
            dy=0.0,
            raw_vx=0.1
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert vx == 0.0
        assert wz == 0.0
        assert status.confidence_decay is True

    def test_lidar_obstacle_safety_override(self, adapter):
        intent = TrackIntent(
            instruction="Follow ahead",
            target_detected=True,
            confidence=0.9,
            dx=0.8,
            dy=0.0,
            raw_vx=0.15
        )
        now = time.time()
        adapter.last_intent_time = now
        adapter.latest_min_obstacle_dist = 0.25  # Closer than min_obstacle_distance (0.35m)

        vx, wz, status = adapter.compute_velocity(intent, now)
        # Forward speed must be cut to 0 due to obstacle!
        assert vx == 0.0
        assert status.lidar_override is True
        assert status.is_safe is False

    class _FakeScan:
        """Minimal stand-in for sensor_msgs/LaserScan.

        Matches the fields scan_callback actually reads, so these tests stay
        pure-Python and runnable without a ROS installation.
        """
        def __init__(self, ranges, range_min=0.1, range_max=12.0):
            import math
            self.ranges = ranges
            self.angle_min = -math.pi
            self.angle_max = math.pi
            self.angle_increment = 2.0 * math.pi / len(ranges)
            self.range_min = range_min
            self.range_max = range_max

    @classmethod
    def _scan_with_self_legs(cls, front_distance=2.0, rear_distance=2.0):
        """Synthetic 360 deg scan mirroring the real Gazebo measurement.

        Everything is open except four arcs at the angles where Puppy's own legs
        appear (+/-49.6..64.7 deg and +/-144.9..152.9 deg) at ~0.12 m.
        """
        import math

        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / 360.0
        ranges = []
        for i in range(360):
            ang = math.degrees(angle_min + i * angle_increment)
            a = abs(ang)
            if (49.6 <= a <= 64.7) or (144.9 <= a <= 152.9):
                ranges.append(0.12)            # the robot's own legs
            elif a <= 30.0:
                ranges.append(front_distance)  # straight ahead
            elif a >= 150.0:
                ranges.append(rear_distance)   # straight behind
            else:
                ranges.append(2.0)
        return cls._FakeScan(ranges)

    def test_own_legs_do_not_latch_safety_override(self, adapter):
        """Regression: self-returns must not block forward motion forever.

        A global min() over all 360 deg reported 0.12 m from the legs and latched
        the override on, so the robot never moved even with ~2 m of clearance.
        """
        adapter.obstacle_arc_deg = 60.0
        adapter.scan_callback(self._scan_with_self_legs(front_distance=2.0))

        # The forward arc is clear...
        assert adapter.latest_min_obstacle_dist == pytest.approx(2.0)
        # ...even though the closest return overall is a leg at 0.12 m.
        assert min(self._scan_with_self_legs().ranges) == pytest.approx(0.12)

        intent = TrackIntent(
            instruction="Follow ahead",
            target_detected=True,
            confidence=0.9,
            dx=0.8,
            dy=0.0,
            raw_vx=0.15
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert status.lidar_override is False
        assert status.is_safe is True
        assert vx > 0.0

    def test_real_front_obstacle_still_blocks(self, adapter):
        """The arc restriction must not weaken real collision avoidance."""
        adapter.obstacle_arc_deg = 60.0
        adapter.scan_callback(self._scan_with_self_legs(front_distance=0.20))

        assert adapter.latest_min_obstacle_dist == pytest.approx(0.20)

        intent = TrackIntent(
            instruction="Follow ahead",
            target_detected=True,
            confidence=0.9,
            dx=0.8,
            dy=0.0,
            raw_vx=0.15
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert vx == 0.0
        assert status.lidar_override is True
        assert status.is_safe is False

    def test_reversing_uses_rear_arc(self, adapter):
        """Reversing must be judged against the rear arc, not the front one.

        The rear legs (144.9..152.9 deg) poke into a +/-30 deg rear window, so a
        plain min() would latch the override on while backing up. The percentile
        estimate must ignore them.
        """
        adapter.obstacle_arc_deg = 60.0
        adapter.obstacle_percentile = 20.0
        adapter.scan_callback(self._scan_with_self_legs(front_distance=2.0, rear_distance=2.0))

        assert adapter.latest_min_obstacle_dist == pytest.approx(2.0)
        assert adapter.latest_rear_obstacle_dist == pytest.approx(2.0)

        intent = TrackIntent(
            instruction="Back up",
            target_detected=True,
            confidence=0.9,
            dx=-0.8,
            dy=0.0,
            raw_vx=-0.15
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert status.lidar_override is False
        assert vx < 0.0

    def test_real_rear_obstacle_still_blocks_reversing(self, adapter):
        """A genuine wall behind must still stop reverse motion."""
        adapter.obstacle_arc_deg = 60.0
        adapter.obstacle_percentile = 20.0
        adapter.scan_callback(self._scan_with_self_legs(front_distance=2.0, rear_distance=0.15))

        assert adapter.latest_rear_obstacle_dist == pytest.approx(0.15)

        intent = TrackIntent(
            instruction="Back up",
            target_detected=True,
            confidence=0.9,
            dx=-0.8,
            dy=0.0,
            raw_vx=-0.15
        )
        now = time.time()
        adapter.last_intent_time = now

        vx, wz, status = adapter.compute_velocity(intent, now)
        assert status.lidar_override is True
        assert vx == 0.0

    # ------------------------------------------------------------------
    # Blocked-detection regression (sustained vs. momentary LiDAR override)
    # ------------------------------------------------------------------

    @staticmethod
    def _forward_intent(dx=0.8):
        return TrackIntent(
            instruction="Follow ahead",
            target_detected=True,
            confidence=0.9,
            dx=dx,
            dy=0.0,
            raw_vx=0.15,
        )

    def test_momentary_override_does_not_escalate(self, adapter):
        """A single blocked tick is a transient yield, not a wedged robot."""
        adapter.obstacle_arc_deg = 60.0
        adapter.scan_callback(self._scan_with_self_legs(front_distance=0.20))

        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)

        assert vx == 0.0
        assert status.lidar_override is True
        assert status.blocked_escalated is False
        assert status.blocked_duration_s == pytest.approx(0.0)
        assert status.active_override_reason.startswith("LIDAR_OBSTACLE_CLOSE")

    def test_sustained_block_escalates_and_reports_duration(self, adapter):
        """Regression: a robot held at vx=0 forever must become visible.

        Previously `lidar_override` simply latched on with no duration and no
        consumer, so the robot sat against a wall while the mission layer happily
        reported progress.
        """
        adapter.obstacle_arc_deg = 60.0
        adapter.blocked_escalation_sec = 5.0
        adapter.scan_callback(self._scan_with_self_legs(front_distance=0.20))

        t0 = time.time()
        adapter.last_intent_time = t0

        # Blocked continuously for 8 s of wall-clock ticks.
        statuses = []
        for _ in range(4):
            t = t0 + 2.0
            adapter.last_intent_time = t
            adapter.current_smoothed_vx = 0.15  # tracker still demands forward
            _, _, status = adapter.compute_velocity(self._forward_intent(), t)
            statuses.append(status)
            t0 = t

        last = statuses[-1]
        assert last.blocked_duration_s == pytest.approx(6.0)
        assert last.blocked_escalated is True
        assert last.active_override_reason.startswith("BLOCKED_ESCALATION")
        # Escalation only appears once the window has elapsed.
        assert statuses[0].blocked_escalated is False

    def test_block_timer_clears_when_path_frees(self, adapter):
        """The block is an episode: it must reset, not accumulate for the run."""
        adapter.obstacle_arc_deg = 60.0
        adapter.blocked_escalation_sec = 5.0

        t_blocked = time.time()
        adapter.last_intent_time = t_blocked
        adapter.scan_callback(self._scan_with_self_legs(front_distance=0.20))
        adapter.compute_velocity(self._forward_intent(), t_blocked)
        assert adapter.blocked_since is not None

        # The obstacle moves away.
        t_clear = t_blocked + 3.0
        adapter.last_intent_time = t_clear
        adapter.scan_callback(self._scan_with_self_legs(front_distance=2.0))
        _, _, status = adapter.compute_velocity(self._forward_intent(), t_clear)

        assert adapter.blocked_since is None
        assert status.blocked_duration_s == 0.0
        assert status.blocked_escalated is False
        assert status.lidar_override is False

    # ------------------------------------------------------------------
    # Mission phase gating: the tracker must not drive outside VISUAL_TRACKING
    # ------------------------------------------------------------------

    def test_tracker_drives_when_no_mission_seen(self, adapter):
        """With no mission, tracking stays enabled (teleop-style following)."""
        adapter.gate_by_mission_phase = True
        adapter.tracking_phases = ["VISUAL_TRACKING"]
        adapter.mission_phase = None

        now = time.time()
        adapter.last_intent_time = now
        vx, _, status = adapter.compute_velocity(self._forward_intent(), now)

        assert vx > 0.0
        assert status.phase_gated is False

    def test_tracker_drives_during_visual_tracking(self, adapter):
        adapter.mission_phase = "VISUAL_TRACKING"
        now = time.time()
        adapter.last_intent_time = now
        vx, _, status = adapter.compute_velocity(self._forward_intent(), now)

        assert vx > 0.0
        assert status.phase_gated is False

    def test_tracker_yields_during_navigation(self, adapter):
        """Regression: the tracker drove the base away from the navigation goal.

        Measured on the live sim: during NAVIGATE_TO_ZONE with the zone at
        [1.0, 0.0] (behind the robot) the tracker still commanded max_vx forward
        towards a target at dx=+0.6 until the wall stopped it, and the mission
        then failed with a misleading "blocked by obstacle".
        """
        adapter.mission_phase = "NAVIGATE_TO_ZONE"
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)

        assert vx == 0.0
        assert wz == 0.0
        assert status.phase_gated is True
        assert "PHASE_GATE" in status.active_override_reason

    def test_tracker_yields_in_terminal_and_verify_phases(self, adapter):
        for phase in ("RETURN_HOME", "INSPECT_VERIFY", "COMPLETED", "FAILED", "ABORTED"):
            adapter.mission_phase = phase
            adapter.current_smoothed_vx = 0.15
            now = time.time()
            adapter.last_intent_time = now
            vx, _, status = adapter.compute_velocity(self._forward_intent(), now)
            assert vx == 0.0, phase
            assert status.phase_gated is True, phase

    def test_phase_gate_is_not_counted_as_a_lidar_block(self, adapter):
        """A gate is arbitration, not an obstruction.

        If it were reported as `lidar_override`, the mission layer would abort
        with "blocked by obstacle" and blame a wall for a missing navigator.
        """
        adapter.mission_phase = "NAVIGATE_TO_ZONE"
        adapter.blocked_since = time.time() - 30.0   # pretend a long block
        now = time.time()
        adapter.last_intent_time = now

        _, _, status = adapter.compute_velocity(self._forward_intent(), now)

        assert status.phase_gated is True
        assert status.lidar_override is False
        assert status.blocked_escalated is False
        assert adapter.blocked_since is None

    def test_gate_can_be_disabled_by_parameter(self, adapter):
        adapter.gate_by_mission_phase = False
        adapter.mission_phase = "NAVIGATE_TO_ZONE"
        now = time.time()
        adapter.last_intent_time = now
        vx, _, status = adapter.compute_velocity(self._forward_intent(), now)

        assert vx > 0.0
        assert status.phase_gated is False


    # ------------------------------------------------------------------
    # Navigation: when gated but a /goal_pose is present, the adapter drives
    # the base itself (it is the actuator) instead of just holding still.
    # ------------------------------------------------------------------

    def _arm_nav(self, adapter, robot_pose, nav_goal, phase="NAVIGATE_TO_ZONE"):
        adapter.gate_by_mission_phase = True
        adapter.tracking_phases = ["VISUAL_TRACKING"]
        adapter.mission_phase = phase
        adapter.pose_frame = "map"
        adapter.robot_pose = list(robot_pose)            # [x, y, yaw] map frame
        adapter.nav_goal = list(nav_goal)                # [x, y] map frame
        adapter.has_nav_goal = True
        adapter.nav_yaw_gain = 2.0
        adapter.nav_align_rad = 0.35
        adapter.nav_lin_gain = 0.5
        adapter.nav_arrival_m = 0.20
        adapter.min_obstacle_distance = 0.35
        adapter.latest_min_obstacle_dist = 99.0         # clear path by default
        adapter.latest_rear_obstacle_dist = 99.0
        adapter.current_smoothed_vx = 0.0
        adapter.current_smoothed_wz = 0.0
        adapter.blocked_since = None

    def test_navigates_forward_when_goal_is_ahead(self, adapter):
        """Gated + goal ahead -> drive forward, NOT phase_gated."""
        self._arm_nav(adapter, [0.0, 0.0, 0.0], [2.0, 0.0])
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.navigating is True
        assert status.phase_gated is False
        assert vx > 0.0
        assert abs(wz) < 1e-6

    def test_navigates_reverses_when_goal_is_behind(self, adapter):
        """Goal directly behind -> reverse toward it (vx<0); no turn needed.

        This sim's gait does not execute angular.z, so the robot cannot turn to
        face a behind goal; it reaches it by driving backward along its x-axis.
        """
        self._arm_nav(adapter, [0.0, 0.0, 0.0], [-2.0, 0.0])   # 180 deg behind
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.navigating is True
        assert vx < 0.0
        assert abs(wz) < 1e-6

    def test_navigates_drives_forward_when_laterally_offset(self, adapter):
        """Goal ahead but offset laterally -> drive forward (rotation unavailable)."""
        self._arm_nav(adapter, [0.0, 0.0, 0.0], [2.0, 1.0])
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.navigating is True
        assert vx > 0.0

    def test_navigates_holds_at_arrival_deadband(self, adapter):
        """Within nav_arrival_m -> stop, still flag navigating."""
        self._arm_nav(adapter, [1.0, 0.0, 0.0], [1.0, 0.05])
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.navigating is True
        assert vx == 0.0

    def test_gated_without_goal_still_yields(self, adapter):
        """Gated but NO goal -> voluntary yield (phase_gated), not navigating."""
        adapter.gate_by_mission_phase = True
        adapter.mission_phase = "NAVIGATE_TO_ZONE"
        adapter.has_nav_goal = False
        adapter.nav_goal = None
        adapter.robot_pose = [0.0, 0.0, 0.0]
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.phase_gated is True
        assert status.navigating is False
        assert vx == 0.0 and wz == 0.0

    def test_lidar_override_still_blocks_navigation(self, adapter):
        """Navigation forward must still respect the LiDAR wall stop."""
        self._arm_nav(adapter, [0.0, 0.0, 0.0], [2.0, 0.0])
        adapter.latest_min_obstacle_dist = 0.17      # wall 0.17m ahead
        now = time.time()
        adapter.last_intent_time = now
        vx, wz, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.navigating is True
        assert status.lidar_override is True
        assert vx == 0.0

    def test_navigating_is_distinct_from_phase_gate(self, adapter):
        """navigating must never be reported as a voluntary yield / lidar block."""
        self._arm_nav(adapter, [0.0, 0.0, 0.0], [2.0, 0.0])
        now = time.time()
        adapter.last_intent_time = now
        _, _, status = adapter.compute_velocity(self._forward_intent(), now)
        assert status.navigating is True
        assert status.phase_gated is False
        assert status.lidar_override is False
