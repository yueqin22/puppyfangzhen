"""Navigation lateral (strafe) behaviour of TrackCmdAdapterNode.

Background, all measured on the live sim rather than assumed:

* The gait ignores angular.z: commanding 0.3 rad/s for 6 s should turn the base
  103 deg but turned it 1.3 deg. Navigation therefore cannot turn to face a goal.
* The gait DOES execute linear.y: 0.1 m/s for 4 s produced 0.379 m of lateral
  travel against an ideal 0.400 m (95%).

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
        return node

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
        """A goal straight ahead keeps the original fore/aft behaviour."""
        adapter.scan_callback(_scan_front_blocked_sides_open(front=0.33, elsewhere=2.9))
        self._arm_navigation(adapter, (0.0, 0.0, 0.0), (1.0, 0.0))

        vx, wz, status = adapter.compute_velocity(None, time.time())

        # Nose arc is blocked, so forward motion is refused exactly as before.
        assert status.commanded_vy == pytest.approx(0.0, abs=1e-6)
        assert status.lidar_override is True
        assert vx == 0.0

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
