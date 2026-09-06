# -*- coding: utf-8 -*-
"""Unit tests for Mission Grounder Node (natural language decomposition and state machine)."""

import pytest
from puppy_minicpm_robot.types import MissionPhase
from puppy_minicpm_robot.mission_grounder_node import MissionGrounderNode


class TestMissionGrounder:
    @pytest.fixture
    def grounder(self):
        return MissionGrounderNode()

    def test_parse_chinese_backyard_inspection(self, grounder):
        cmd = "巡视后院并检查是否有可疑物品"
        mission = grounder.parse_instruction(cmd)

        assert mission.phase == MissionPhase.NAVIGATE_TO_ZONE
        assert mission.zone_name in ["backyard", "后院"]
        assert mission.zone_pose == [2.0, -2.0, 0.0]
        assert mission.target_name == "suspicious_item"
        assert mission.target_category == "security_inspection"
        assert mission.return_pose == [0.0, 0.0, 0.0]

    def test_parse_english_corridor_person_tracking(self, grounder):
        cmd = "Follow the person in corridor for 8 seconds"
        mission = grounder.parse_instruction(cmd)

        assert mission.phase == MissionPhase.NAVIGATE_TO_ZONE
        assert mission.zone_name in ["corridor", "走廊"]
        assert mission.dwell_time_sec == 8.0
        assert mission.target_name == "person"

    def test_parse_cancellation_command(self, grounder):
        cmd = "紧急终止任务 cancel"
        mission = grounder.parse_instruction(cmd)

        assert mission.phase == MissionPhase.ABORTED
        assert "cancelled" in mission.message.lower()

    def test_state_machine_transition(self, grounder):
        cmd = "巡视客厅"
        mission = grounder.parse_instruction(cmd)
        grounder.current_mission = mission

        # Simulate arriving at living room
        grounder.robot_pose = [1.0, 0.0, 0.0]
        grounder.timer_callback()
        assert grounder.current_mission.phase == MissionPhase.VISUAL_TRACKING

    # ------------------------------------------------------------------
    # False-success regression: timeout must NOT masquerade as arrival
    # ------------------------------------------------------------------

    class _FakeString:
        """Stand-in for std_msgs/String (no ROS needed)."""
        def __init__(self, data):
            self.data = data

    @staticmethod
    def _safety_json(override=True, escalated=True):
        import json
        return TestMissionGrounder._FakeString(json.dumps({
            "is_safe": False,
            "obstacle_distance": 0.16,
            "lidar_override": override,
            "timeout_stop": False,
            "confidence_decay": False,
            "active_override_reason": "BLOCKED_ESCALATION (12.0s >= 5.0s, clearance 0.16m)",
            "blocked_duration_s": 12.0,
            "blocked_escalated": escalated,
        }))

    def test_navigation_timeout_fails_instead_of_advancing(self, grounder):
        """Regression: a timeout used to silently advance to VISUAL_TRACKING.

        The robot had never left the start, yet the mission claimed the zone was
        reached and moved on to tracking.
        """
        import time
        mission = grounder.parse_instruction("巡视客厅")   # zone at [1.0, 0.0]
        grounder.current_mission = mission
        grounder.robot_pose = [0.0, 0.0, 0.0]             # 1.0 m away, not arrived
        grounder.phase_start_time = time.time() - 20.0    # past navigation_timeout_sec

        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.FAILED
        assert "timed out" in grounder.current_mission.message

    def test_return_home_timeout_does_not_report_success(self, grounder):
        """Regression: RETURN_HOME declared 'completed successfully' on timeout.

        Measured on the real sim: the robot was stranded 2.21 m from home, held
        at vx=0 by the LiDAR override, and the mission still reported success.
        """
        import time
        mission = grounder.parse_instruction("巡视客厅")
        mission.phase = MissionPhase.RETURN_HOME
        grounder.current_mission = mission
        grounder.robot_pose = [2.21, 0.0, 0.0]            # 2.21 m from home
        grounder.phase_start_time = time.time() - 20.0

        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.FAILED
        assert "completed successfully" not in grounder.current_mission.message.lower()

    def test_return_home_actual_arrival_reports_success(self, grounder):
        """Genuine arrival must still complete the mission."""
        mission = grounder.parse_instruction("巡视客厅")
        mission.phase = MissionPhase.RETURN_HOME
        grounder.current_mission = mission
        grounder.robot_pose = [0.1, 0.0, 0.0]             # within arrival_radius_m

        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.COMPLETED
        assert "completed successfully" in grounder.current_mission.message.lower()

    def test_sustained_block_aborts_mission_early(self, grounder):
        """A wedged robot should abort on the block, not on the nav timeout."""
        import time
        mission = grounder.parse_instruction("巡视客厅")
        grounder.current_mission = mission
        grounder.robot_pose = [0.0, 0.0, 0.0]
        grounder.phase_start_time = time.time() - 2.0     # well inside the nav timeout

        grounder.safety_callback(self._safety_json())     # escalated + override
        grounder.blocked_since = time.time() - 20.0       # blocked for 20 s
        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.FAILED
        assert "blocked" in grounder.current_mission.message.lower()

    def test_blocked_episode_resets_when_path_frees(self, grounder):
        """Clearing the obstacle must clear the block timer."""
        import time
        grounder.safety_callback(self._safety_json(override=True, escalated=True))
        assert grounder.blocked_since is not None

        grounder.safety_callback(self._safety_json(override=False, escalated=False))
        assert grounder.blocked_since is None
        assert grounder.blocked_for(time.time()) == 0.0

    def test_terminal_phase_is_latched(self, grounder):
        """A failed mission must not be resurrected by later ticks."""
        mission = grounder.parse_instruction("巡视客厅")
        mission.phase = MissionPhase.FAILED
        mission.message = "Navigation to 客厅 timed out"
        grounder.current_mission = mission
        grounder.robot_pose = [1.0, 0.0, 0.0]             # now sitting on the goal

        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.FAILED

    def test_blocked_abort_precedes_navigation_timeout(self, grounder):
        """Ordering invariant: the blocked-abort branch must be reachable.

        Escalation is what unblocks the grounder's own timer, so
        blocked_abort_sec has to sit above the adapter's escalation window and
        below the phase timeouts. If it drifts above them the generic "timed
        out" branch fires first and the branch becomes dead code that also hides
        the more useful "blocked by obstacle" reason (observed with a 15 s abort
        against a 10 s timeout).
        """
        import time
        # Matches config/minicpm_robot.yaml and _init_mock_state defaults.
        assert grounder.blocked_abort_sec > 5.0            # > blocked_escalation_sec
        assert grounder.blocked_abort_sec < grounder.navigation_timeout_sec
        assert grounder.blocked_abort_sec < grounder.return_timeout_sec

        mission = grounder.parse_instruction("巡视客厅")
        grounder.current_mission = mission
        grounder.robot_pose = [0.0, 0.0, 0.0]
        # Elapsed past the abort window but still inside the navigation timeout,
        # so only the blocked branch can fire.
        grounder.phase_start_time = time.time() - (grounder.blocked_abort_sec + 1.0)
        grounder.safety_callback(self._safety_json())
        grounder.blocked_since = time.time() - (grounder.blocked_abort_sec + 1.0)

        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.FAILED
        assert "blocked by obstacle" in grounder.current_mission.message

    def test_navigation_deadline_scales_with_distance(self, grounder):
        """Regression: a flat deadline cannot serve a 0.15 m/s quadruped.

        Crossing the 5x5 m test room takes 13-33 s, so the original flat 10 s
        navigation timeout would fail traversals that are perfectly healthy.
        """
        grounder.robot_pose = [0.0, 0.0, 0.0]

        grounder._arm_phase_timeout([1.0, 0.0])
        near = grounder.phase_timeout
        grounder._arm_phase_timeout([5.0, 0.0])
        far = grounder.phase_timeout

        assert far > near
        assert near > 10.0                      # old flat timeout was unreachable
        assert near == pytest.approx(15.0 + (1.0 / 0.10) * 2.0)
        # Return home arms against its own base.
        grounder._arm_phase_timeout([0.0, 0.0], base=grounder.return_timeout_sec)
        assert grounder.phase_timeout == pytest.approx(grounder.return_timeout_sec)

    def test_deadline_armed_once_not_shrinking_per_tick(self, grounder):
        """The deadline must not collapse as the robot closes in on the goal."""
        import time
        mission = grounder.parse_instruction("巡视客厅")
        mission.phase = MissionPhase.RETURN_HOME
        grounder.current_mission = mission
        grounder.robot_pose = [5.0, 0.0, 0.0]
        grounder._arm_phase_timeout([0.0, 0.0], base=grounder.return_timeout_sec)
        armed = grounder.phase_timeout

        # Robot has almost arrived; the deadline stays as armed.
        grounder.robot_pose = [0.1, 0.0, 0.0]
        grounder.phase_start_time = time.time()
        grounder.timer_callback()

        assert grounder.current_mission.phase == MissionPhase.COMPLETED
        assert armed > grounder.return_timeout_sec

    # ------------------------------------------------------------------
    # Frame-consistency regression: goals are map-frame, /odom is odom-frame
    # ------------------------------------------------------------------

    def test_arrival_is_judged_in_map_frame(self, grounder):
        """Regression: /odom (odom frame) was compared against map-frame goals.

        Measured on the live sim: map->odom was [-0.152, -0.056] @ -3.27 deg,
        putting the robot at (2.057, -0.190) in map vs (2.213, -0.008) in odom --
        a 0.24 m disagreement, 60% of the 0.4 m arrival radius. The robot could be
        declared "arrived" while still 0.64 m short of the goal.
        """
        mission = grounder.parse_instruction("巡视客厅")     # zone at [1.0, 0.0]
        grounder.current_mission = mission

        # Odom-frame pose says we are still 0.5 m short (outside the 0.4 m radius).
        grounder.robot_pose = [1.5, 0.0, 0.0]
        # Map-frame pose (authoritative) says we have actually arrived.
        grounder.map_pose = [1.05, 0.0, 0.0]

        grounder.timer_callback()
        assert grounder.current_mission.phase == MissionPhase.VISUAL_TRACKING

    def test_map_frame_poses_can_prevent_false_arrival(self, grounder):
        """The converse: odom says arrived, map says not -> must NOT advance."""
        mission = grounder.parse_instruction("巡视客厅")     # zone at [1.0, 0.0]
        grounder.current_mission = mission

        grounder.robot_pose = [1.05, 0.0, 0.0]              # odom: arrived
        grounder.map_pose = [1.5, 0.0, 0.0]                 # map: still 0.5 m short

        grounder.timer_callback()
        assert grounder.current_mission.phase == MissionPhase.NAVIGATE_TO_ZONE

    def test_odom_pose_used_when_tf_unavailable(self, grounder):
        """Without TF the odom pose must still drive navigation (fallback)."""
        mission = grounder.parse_instruction("巡视客厅")
        grounder.current_mission = mission
        grounder.map_pose = None                            # TF down
        grounder.robot_pose = [1.05, 0.0, 0.0]              # within arrival radius

        assert grounder.nav_pose() == grounder.robot_pose
        grounder.timer_callback()
        assert grounder.current_mission.phase == MissionPhase.VISUAL_TRACKING

    def test_goal_pose_is_published_in_map_frame(self, grounder):
        """Goals and waypoints must live in the same frame."""
        assert grounder.map_frame == "map"
        grounder.parse_instruction("巡视客厅")
        # dispatch_goal_pose stamps frame_id from map_frame; assert the pairing.
        assert grounder.base_frame == "base_footprint"

    def test_status_reports_pose_and_frame_in_use(self, grounder):
        """The status must expose which pose and frame drove the arrival verdict.

        A silent fallback from map to odom reintroduces the 0.24 m arrival error
        that was measured on the live sim, so it has to be visible on the wire.
        """
        import json
        mission = grounder.parse_instruction("巡视客厅")
        grounder.current_mission = mission

        grounder.robot_pose = [2.213, -0.008, 0.0]   # odom-frame
        grounder.map_pose = [2.057, -0.190, 0.0]     # map-frame (authoritative)
        grounder._sync_pose_report()
        reported = json.loads(json.dumps(mission.to_dict()))

        assert reported["pose_frame"] == "map"
        assert reported["robot_pose"] == [2.057, -0.19, 0.0]

        # TF drops out -> the fallback must be reported, not hidden.
        grounder.map_pose = None
        grounder._sync_pose_report()
        reported = json.loads(json.dumps(mission.to_dict()))

        assert reported["pose_frame"] == "odom"
        assert reported["robot_pose"] == [2.213, -0.008, 0.0]

    def test_tf_buffer_survives_construction(self, grounder):
        """Regression: the TF buffer was wiped after the listener was built.

        `__init__` used to assign `self.tf_buffer = None` *after*
        `_init_publishers_and_subscribers()` had already created the
        TransformListener, so every `update_map_pose()` call short-circuited on
        "tf2 unavailable" and the node navigated in the odom frame forever --
        measured as `pose_frame=odom` with `robot_pose` byte-identical to /odom
        while TF was in fact perfectly healthy.

        The other frame tests could not catch this because they assign
        `map_pose` directly and never touch the real lookup.
        """
        from puppy_minicpm_robot.mission_grounder_node import HAS_RCLPY

        if not HAS_RCLPY:
            # 书面原因 (jihua20260905.md §12 "skipped 有书面原因"):
            # 本用例断言的是"TF listener 建好之后 buffer 没有被后续赋值清空",
            # 必须真的构造一个 TransformListener 才能复现; 在 Windows 无 ROS2
            # 环境下 rclpy 不可用, 该路径根本不会被走到。放行到 WSL 侧执行,
            # 见 docs/test_inventory.md。
            pytest.skip("needs a ROS environment to construct a TF listener")

        assert grounder.tf_buffer is not None, "TF buffer was discarded after init"
        assert grounder.tf_listener is not None

    def test_tf_failure_is_reported_not_swallowed(self, grounder):
        """A TF failure must be visible; silence is how the frame bug hid."""
        import time
        grounder._tf_failures = 0
        grounder._tf_last_log = 0.0
        grounder.tf_buffer = None

        assert grounder.update_map_pose() is False
        assert grounder._tf_failures == 1

        # Backoff: repeated failures must not flood the log every tick.
        grounder._tf_last_log = time.time()
        for _ in range(5):
            grounder.update_map_pose()
        assert grounder._tf_failures == 6

    def test_timeout_blames_missing_navigator_when_tracker_gated(self, grounder):
        """A timeout with no actuator must say so, not just report elapsed time.

        `/goal_pose` is published for Nav2 / a patrol node. With none running the
        navigation phase has nothing driving the base, so after the phase gate
        the robot just stands still. A bare "timed out" leaves the operator
        hunting for an obstacle that was never there.
        """
        import time
        grounder.latest_safety = {"phase_gated": True}
        assert "no node consumed /goal_pose" in grounder._no_actuator_hint()

        grounder.latest_safety = {"phase_gated": False}
        assert grounder._no_actuator_hint() == ""

        grounder.latest_safety = {}
        assert grounder._no_actuator_hint() == ""
