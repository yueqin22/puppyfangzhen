"""Navigation lateral (strafe) behaviour of TrackCmdAdapterNode.

Background, measured on the live sim (and re-measured -- the numbers move):

* Yaw is NOT reliably executed. The SAME 0.3 rad/s for 6 s (ideal 103 deg) turned
  the base 1 deg, then 17 deg, then 99 deg across three runs as the sim's
  real-time factor (RTF) went 0.85 -> 0.88 -> 1.01. So turning is not a hard
  limit but an UNREPRODUCIBLE one, and navigation must not depend on it.
* linear.y IS executed and is the more reliable axis: lateral travel measured
  82%, 97%, then 118% of the commanded 0.400 m over those same runs. Forward
  (linear.x) tracked 95-100% consistently. So an off-nose goal is reached by
  strafing, which always works, rather than by turning.

The limitation belongs to the planar-move stand-in, not to the trot gait: with
use_planar_move:=true the URDF loads libgazebo_ros_planar_move.so and no joint
controller runs (there is no gait_controller node at all). That plugin drives the
model with Model::SetLinearVel + Model::SetAngularVel -- the first is a valid
rigid translation, the second is not a valid rigid rotation for a 12-joint model,
so the free legs fight it and the net yaw swings with how the solver integrates
that fight at the current RTF. Fixing trot_gait.cpp would change nothing while
this plugin is in the loop.

Before this change navigation picked forward-or-reverse from the sign of the
body-frame x offset, so a goal lying off the nose was approached by driving into
whatever was straight ahead. Live failure: the goal sat 76 deg off the nose with
2.6-4.7 m clear along that bearing but only 0.33 m clear ahead, so every 20 Hz
tick was vetoed by the LiDAR, the base moved 0.015 m in 8 s, and the mission
aborted with a misleading "blocked by obstacle".
"""

import math
import time

import pytest

from puppy_minicpm_robot.types import TrackIntent
from puppy_minicpm_robot.track_cmd_adapter_node import TrackCmdAdapterNode


def _fake_scan(ranges, range_min=0.1, range_max=12.0):
    """Minimal stand-in for sensor_msgs/LaserScan (no ROS needed)."""
    scan = type("Scan", (), {})()
    scan.ranges = ranges
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2.0 * math.pi / len(ranges)
    scan.range_min = range_min
    scan.range_max = range_max
    return scan


def _scan_front_blocked_sides_open(front=0.33, elsewhere=2.9, n=360):
    """The live situation: wall right in front of the nose, open everywhere else.

    Mirrors the measured scan where the +/-30 deg nose arc read 0.33 m while the
    60-135 deg sector (where the goal actually lay) read 2.6-4.7 m.
    """
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        ang = math.degrees(-math.pi + i * inc)
        ranges.append(front if abs(ang) <= 30.0 else elsewhere)
    return _fake_scan(ranges)


def _scan_all_blocked(distance=0.2, n=360):
    return _fake_scan([distance] * n)


def _scan_one_tangent_open(front=0.2, blocked_side=0.2, open_side=2.9, default=2.9, n=360):
    """Nose walled off, one flank walled off, the other flank wide open."""
    inc = 2.0 * math.pi / n
    ranges = []
    for i in range(n):
        ang = math.degrees(-math.pi + i * inc)
        if abs(ang) <= 30.0:
            ranges.append(front)
        elif 60.0 <= ang <= 120.0:
            ranges.append(blocked_side)   # body +y flank
        elif -120.0 <= ang <= -60.0:
            ranges.append(open_side)      # body -y flank
        else:
            ranges.append(default)
    return _fake_scan(ranges)


class TestNavigationLateral:
    @pytest.fixture
    def adapter(self):
        node = TrackCmdAdapterNode()
        node.mode = "dry-run"
        node.gate_by_mission_phase = True
        node.max_vx = 0.15
        node.max_vy = 0.15
        node.max_wz = 0.30
        node.min_obstacle_distance = 0.35
        node.obstacle_arc_deg = 60.0
        node.obstacle_percentile = 20.0
        node.nav_lin_gain = 0.5
        node.nav_yaw_gain = 2.0
        node.nav_arrival_m = 0.20
        node.strafe_gain = 1.0
        return node

    def test_strafe_gain_is_applied_only_at_the_wire(self, adapter):
        """strafe_gain must scale linear.y but leave the navigation intent
        (commanded_vy) and forward/yaw untouched.

        The hover stand-in base tracks lateral commands at only ~82%, so the
        config sets strafe_gain ~= 1.22; this test locks the seam so a future
        change cannot move the gain into the clearance/escape math or drop it
        from the published message.
        """
        adapter.strafe_gain = 1.22
        # A pure strafe intent: commanded_vy = 0.10, vx = 0, wz = 0.
        twist = adapter._build_twist(0.0, 0.10, 0.0)
        # Published lateral is boosted; intent is unchanged.
        assert twist.linear.y == pytest.approx(0.122, abs=1e-9)
        assert twist.linear.x == pytest.approx(0.0, abs=1e-9)
        assert twist.angular.z == pytest.approx(0.0, abs=1e-9)
        # Gain of 1.0 must be a pass-through.
        adapter.strafe_gain = 1.0
        twist = adapter._build_twist(0.0, 0.10, 0.0)
        assert twist.linear.y == pytest.approx(0.10, abs=1e-9)
        # Forward is never scaled, even with a non-unity gain.
        adapter.strafe_gain = 1.22
        twist = adapter._build_twist(0.15, 0.0, 0.0)
        assert twist.linear.x == pytest.approx(0.15, abs=1e-9)
        assert twist.linear.y == pytest.approx(0.0, abs=1e-9)

    def _arm_navigation(self, adapter, pose, goal):
        adapter.mission_phase = "NAVIGATE_TO_ZONE"
        adapter.has_nav_goal = True
        adapter.nav_goal = goal
        adapter.robot_pose = pose
        adapter.pose_frame = "map"
        adapter.last_intent_time = time.time()

    def test_live_scenario_goal_76_deg_off_nose_is_reachable(self, adapter):
        """Regression for the live FAILED mission.

        Robot at (2.023, -1.794) yaw 0.756 rad in map, goal (1.0, 0.0): the goal
        is 2.07 m away at 76 deg off the nose. Straight ahead is a wall at 0.33 m;
        the bearing toward the goal is wide open. Fore/aft-only navigation vetoed
        itself here for 8 s; strafing must instead drive along the clear bearing.
        """
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.33, elsewhere=2.9))
        self._arm_navigation(adapter, (2.023, -1.794, 0.756), (1.0, 0.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert status.navigating is True
        # Motion is commanded at all -- the pre-fix behaviour was a hard 0 here.
        assert abs(vx) > 0.0 or abs(status.commanded_vy) > 0.0
        # ...and most of it is lateral, because the goal is 76 deg off the nose.
        assert abs(status.commanded_vy) > abs(vx)
        assert status.lidar_override is False
        assert status.obstacle_distance > adapter.min_obstacle_distance

    def test_clearance_is_measured_along_the_travel_direction(self, adapter):
        """A strafe must be judged on its own bearing, not on the nose arc."""
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.33, elsewhere=2.9))
        # Goal 90 deg to the left of a base facing +x: a pure lateral move.
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (0.0, 1.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert abs(vx) == pytest.approx(0.0, abs=1e-6)
        assert status.commanded_vy > 0.0
        assert status.obstacle_distance == pytest.approx(2.9)
        assert status.lidar_override is False

    def test_lateral_move_is_stopped_when_its_own_path_is_blocked(self, adapter):
        """Strafing does not grant immunity: a blocked lateral path still blocks."""
        adapter.scan_callback(_scan_all_blocked(0.2))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (0.0, 1.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert status.lidar_override is True
        assert status.is_safe is False
        assert status.commanded_vy == 0.0
        assert vx == 0.0

    # ------------------------------------------------------------------
    # Escape from a block (defect 13): stopping dead was a deadlock
    # ------------------------------------------------------------------

    def test_blocked_navigation_sidesteps_instead_of_deadlocking(self, adapter):
        """Regression for the live 6.0 s block at 0.33 m.

        The old code answered a block with vx = vy = 0 and the comment "allow
        rotation", but rotation is not available (0.3 rad/s for 6 s turned the
        base 1.28 deg). The base therefore pinned itself in place until
        mission_grounder aborted at 8 s -- only 2 s of margin in the live run.
        """
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.33, elsewhere=2.9))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert status.lidar_override is True          # still reported as blocked...
        assert abs(status.commanded_vy) > 0.0         # ...but no longer standing still
        assert status.active_override_reason.startswith("LIDAR_SIDESTEP")
        assert status.blocked_escalated is False
        # The graze stays visible: the blocked bearing is what is reported.
        assert status.obstacle_distance == pytest.approx(0.33)

    def test_sidestep_takes_the_open_tangent(self, adapter):
        """Both tangents are scored; the slide must use the one that is open."""
        adapter.scan_callback(_scan_one_tangent_open())
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        _, _, status = adapter.compute_velocity(None, time.time())

        # +y side is walled off at 0.20 m, -y side is open at 2.9 m.
        assert status.commanded_vy < 0.0

    def test_sustained_escape_is_not_reported_as_escalation(self, adapter):
        """A base working around an obstacle must not accumulate toward an abort.

        mission_grounder aborts on `lidar_override and blocked_escalated` held for
        8 s, so escalating here would fail a mission that is making progress.
        """
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.33, elsewhere=2.9))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        now = time.time()
        adapter.blocked_since = now - 20.0  # long past the 5 s escalation window
        adapter.last_intent_time = now
        _, _, status = adapter.compute_velocity(None, now)

        assert status.blocked_duration_s > adapter.blocked_escalation_sec
        assert status.blocked_escalated is False

    def test_nowhere_to_go_still_escalates(self, adapter):
        """With both tangents blocked, stopping and escalating is still correct."""
        adapter.scan_callback(_scan_all_blocked(0.2))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        now = time.time()
        adapter.blocked_since = now - 10.0
        adapter.last_intent_time = now
        _, _, status = adapter.compute_velocity(None, now)

        assert status.blocked_escalated is True
        assert status.active_override_reason.startswith("BLOCKED_ESCALATION")
        assert status.commanded_vy == 0.0

    def test_tracker_block_stops_instead_of_sidestepping(self, adapter):
        """Escape is navigation-only: a blocked follower should stand off.

        The tracker is pursuing a target it cannot reach, so sliding sideways
        would invent motion the operator never asked for.
        """
        adapter.mission_phase = "VISUAL_TRACKING"
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.2, elsewhere=2.9))
        adapter.last_intent_time = time.time()
        intent = TrackIntent(instruction="Follow ahead", target_detected=True,
                             confidence=0.9, dx=0.8, dy=0.0, raw_vx=0.15)

        vx, _, status = adapter.compute_velocity(intent, time.time())

        assert vx == 0.0
        assert status.commanded_vy == 0.0
        assert status.lidar_override is True

    def test_tracker_driving_never_commands_lateral_motion(self, adapter):
        """Strafe is a navigation-only tool; tracker following stays fore/aft."""
        adapter.mission_phase = "VISUAL_TRACKING"
        adapter.latest_min_obstacle_dist = 9.0
        adapter.latest_rear_obstacle_dist = 9.0
        adapter.last_intent_time = time.time()
        intent = TrackIntent(instruction="Follow ahead", target_detected=True,
                             confidence=0.9, dx=0.6, dy=0.0, raw_vx=0.1)

        vx, wz, status = adapter.compute_velocity(intent, time.time())

        assert vx > 0.0
        assert status.commanded_vy == 0.0

    def test_pure_fore_aft_still_uses_precomputed_arcs(self, adapter):
        """A goal straight ahead on a clear path keeps the original fore/aft behaviour."""
        adapter.scan_callback(_scan_front_blocked_sides_open(front=2.9, elsewhere=2.9))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert status.commanded_vy == pytest.approx(0.0, abs=1e-6)
        assert status.lidar_override is False
        assert vx > 0.0

    def test_first_tick_before_any_scan_does_not_explode(self, adapter):
        """The first compute_velocity can precede the first LaserScan.

        `_travel_clearance` reads `latest_scan`, which is only assigned by
        scan_callback. On the real-node path it therefore has to exist from
        construction, not from the mock-state branch the unit tests happen to
        take -- otherwise the node dies with AttributeError before it ever
        moves, while every test here stays green.
        """
        adapter.latest_scan = None
        adapter.latest_min_obstacle_dist = 99.0
        adapter.latest_rear_obstacle_dist = 99.0
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert math.isfinite(status.obstacle_distance)

    def test_arrival_commands_no_lateral_motion(self, adapter):
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.33, elsewhere=2.9))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (0.05, 0.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        assert "NAV_ARRIVED" in status.active_override_reason
        assert status.commanded_vy == 0.0
        assert vx == 0.0
