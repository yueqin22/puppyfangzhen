#!/usr/bin/env python3
"""Minimal regression suite for puppy_core."""
import os

import pytest

rclpy = pytest.importorskip('rclpy', reason='rclpy not available')


class TestInterfaceContracts:
    def test_battery_status_fields(self):
        from puppy_interfaces.msg import BatteryStatus
        msg = BatteryStatus()
        for field in ['voltage', 'current', 'percent', 'charging',
                      'low_battery', 'critical_battery']:
            assert hasattr(msg, field), f'BatteryStatus missing {field}'

    def test_fall_event_fields(self):
        from puppy_interfaces.msg import FallEvent
        msg = FallEvent()
        for field in ['detected', 'direction', 'confidence', 'source']:
            assert hasattr(msg, field), f'FallEvent missing {field}'

    def test_security_event_fields(self):
        from puppy_interfaces.msg import SecurityEvent
        msg = SecurityEvent()
        for field in ['event_type', 'severity', 'source_node', 'message']:
            assert hasattr(msg, field), f'SecurityEvent missing {field}'

    def test_patrol_status_fields(self):
        from puppy_interfaces.msg import PatrolStatus
        msg = PatrolStatus()
        for field in ['state', 'current_waypoint_index', 'total_waypoints',
                      'completion_ratio', 'message']:
            assert hasattr(msg, field), f'PatrolStatus missing {field}'

    def test_robot_health_fields(self):
        from puppy_interfaces.msg import RobotHealth
        msg = RobotHealth()
        for field in ['ok', 'level', 'active_faults', 'cpu_temp',
                      'battery_percent', 'imu_ready', 'lidar_ready',
                      'camera_ready', 'motion_ready']:
            assert hasattr(msg, field), f'RobotHealth missing {field}'

    def test_nav2_action_available(self):
        from nav2_msgs.action import NavigateToPose
        goal = NavigateToPose.Goal()
        assert hasattr(goal, 'pose'), 'NavigateToPose.Goal missing pose'

    def test_custom_actions_exist(self):
        from puppy_interfaces.action import Dock, RecoverPosture, SitDown, StandUp
        assert Dock.Goal is not None
        assert StandUp.Goal is not None
        assert SitDown.Goal is not None
        assert RecoverPosture.Goal is not None


class TestLaunchSmoke:
    def test_core_bringup_parses(self):
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('puppy_core')
        except Exception:
            pytest.skip('puppy_core package not installed')
        launch_file = os.path.join(pkg_share, 'launch', 'core_bringup.launch.py')
        if not os.path.exists(launch_file):
            pytest.skip(f'{launch_file} not found')
        import importlib.util
        spec = importlib.util.spec_from_file_location('core_bringup', launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        ld = module.generate_launch_description()
        assert ld is not None

    def test_full_system_parses(self):
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('puppy_bringup')
        except Exception:
            pytest.skip('puppy_bringup package not installed')
        launch_file = os.path.join(pkg_share, 'launch', 'full_system.launch.py')
        if not os.path.exists(launch_file):
            pytest.skip(f'{launch_file} not found')
        import importlib.util
        spec = importlib.util.spec_from_file_location('full_system', launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        ld = module.generate_launch_description()
        assert ld is not None

    def test_adapter_bringup_parses(self):
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('puppypi_adapter')
        except Exception:
            pytest.skip('puppypi_adapter package not installed')
        launch_file = os.path.join(pkg_share, 'launch', 'adapter_bringup.launch.py')
        if not os.path.exists(launch_file):
            pytest.skip(f'{launch_file} not found')
        import importlib.util
        spec = importlib.util.spec_from_file_location('adapter_bringup', launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        ld = module.generate_launch_description()
        assert ld is not None


class TestScenarioBehavior:
    @pytest.fixture
    def mission_manager_node(self):
        from puppy_core.mission_manager import MissionManagerNode
        rclpy.init()
        node = MissionManagerNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_low_battery_triggers_docking(self, mission_manager_node):
        from puppy_interfaces.msg import BatteryStatus
        node = mission_manager_node
        node.current_mission = 'PATROL'
        msg = BatteryStatus()
        msg.low_battery = True
        msg.percent = 0.15
        node._on_battery(msg)
        assert node.current_mission == 'DOCKING'

    def test_security_critical_interrupts_mission(self, mission_manager_node):
        from puppy_interfaces.msg import SecurityEvent
        node = mission_manager_node
        node.current_mission = 'PATROL'
        msg = SecurityEvent()
        msg.severity = 'critical'
        msg.event_type = 'intrusion'
        node._on_security(msg)
        assert node.current_mission == 'IDLE'

    def test_capability_unavailable_rejects_patrol(self, mission_manager_node):
        node = mission_manager_node
        node.capabilities = {
            'patrol': {'enabled': True, 'available': False, 'reason': 'LIDAR_TIMEOUT'},
            'navigation': {'enabled': True, 'available': True, 'reason': ''},
        }
        node._start_patrol()
        assert node.current_mission == 'IDLE'
        assert 'rejected' in node.last_status_message

    def test_capability_unavailable_rejects_docking(self, mission_manager_node):
        node = mission_manager_node
        node.capabilities = {
            'docking': {'enabled': True, 'available': False, 'reason': 'motion not ready'},
        }
        node._start_docking()
        assert node.current_mission == 'IDLE'
        assert 'rejected' in node.last_status_message

    def test_navigation_unavailable_rejects_docking(self, mission_manager_node):
        node = mission_manager_node
        node.capabilities = {
            'docking': {'enabled': True, 'available': True, 'reason': ''},
            'navigation': {'enabled': True, 'available': False, 'reason': 'IMU_TIMEOUT'},
        }
        node._start_docking()
        assert node.current_mission == 'IDLE'
        assert 'rejected' in node.last_status_message

    def test_capability_available_allows_patrol(self, mission_manager_node):
        node = mission_manager_node
        node.capabilities = {
            'patrol': {'enabled': True, 'available': True, 'reason': ''},
            'navigation': {'enabled': True, 'available': True, 'reason': ''},
        }
        node._start_patrol()
        assert node.current_mission == 'PATROL'

    def test_mission_command_patrol(self, mission_manager_node):
        from std_msgs.msg import String
        node = mission_manager_node
        msg = String()
        msg.data = 'patrol'
        node._on_command(msg)
        assert node.current_mission in ('PATROL', 'IDLE')

    def test_mission_command_stop(self, mission_manager_node):
        from std_msgs.msg import String
        node = mission_manager_node
        node.current_mission = 'PATROL'
        msg = String()
        msg.data = 'stop'
        node._on_command(msg)
        assert node.current_mission == 'IDLE'

    def test_navigation_unavailable_rejects_manual_goal(self, mission_manager_node):
        from std_msgs.msg import String
        node = mission_manager_node
        node.current_mission = 'PATROL'
        node.capabilities = {
            'navigation': {'enabled': True, 'available': False, 'reason': 'planner offline'},
        }
        msg = String()
        msg.data = 'goto:1.0,2.0'
        node._on_command(msg)
        assert node.current_mission == 'PATROL'
        assert 'rejected' in node.last_status_message

    def test_invalid_manual_goal_is_rejected_without_cancelling(self, mission_manager_node):
        from std_msgs.msg import String
        node = mission_manager_node
        node.current_mission = 'PATROL'
        msg = String()
        msg.data = 'goto:not-a-number'
        node._on_command(msg)
        assert node.current_mission == 'PATROL'
        assert 'invalid coordinates' in node.last_status_message


class TestSafetyManagerBehavior:
    @pytest.fixture
    def safety_manager_node(self):
        from puppy_core.safety_manager import SafetyManagerNode
        rclpy.init()
        node = SafetyManagerNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_fall_detected_disables_motors(self, safety_manager_node):
        from puppy_interfaces.msg import FallEvent
        node = safety_manager_node
        node.motors_enabled = True
        msg = FallEvent()
        msg.detected = True
        msg.direction = 'forward'
        node._on_fall(msg)
        assert not node.motors_enabled

    def test_critical_battery_disables_motors(self, safety_manager_node):
        from puppy_interfaces.msg import BatteryStatus
        node = safety_manager_node
        node.motors_enabled = True
        msg = BatteryStatus()
        msg.critical_battery = True
        msg.percent = 0.08
        node._on_battery(msg)
        assert not node.motors_enabled

    def test_cmd_vel_safety_clamp(self, safety_manager_node):
        from geometry_msgs.msg import Twist
        node = safety_manager_node
        node.motors_enabled = True
        msg = Twist()
        msg.linear.x = 10.0
        msg.angular.z = 5.0
        published = []
        original = node.safe_cmd_pub.publish
        node.safe_cmd_pub.publish = published.append
        try:
            node._on_cmd_vel(msg)
        finally:
            node.safe_cmd_pub.publish = original
        assert len(published) == 1
        clamped = published[0]
        assert abs(clamped.linear.x) <= node.max_linear_x + 1e-6
        assert abs(clamped.angular.z) <= node.max_angular_z + 1e-6


class TestStateAggregatorTimeout:
    @pytest.fixture
    def aggregator_node(self):
        from puppy_core.robot_state_aggregator import RobotStateAggregatorNode
        rclpy.init()
        node = RobotStateAggregatorNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_offline_sensors_detected(self, aggregator_node):
        node = aggregator_node
        timeout_faults = node._check_timeouts()
        fault_names = ' '.join(timeout_faults)
        assert 'IMU_OFFLINE' in fault_names
        assert 'LIDAR_OFFLINE' in fault_names
        assert 'BATTERY_OFFLINE' in fault_names

    def test_imu_message_updates_timestamp(self, aggregator_node):
        from sensor_msgs.msg import Imu
        node = aggregator_node
        msg = Imu()
        node._on_imu(msg)
        assert node.last_seen['imu'] is not None
        timeout_faults = node._check_timeouts()
        assert 'IMU_OFFLINE' not in ' '.join(timeout_faults)

    def test_battery_percent_normalized_to_robot_health_percent(self, aggregator_node):
        from puppy_interfaces.msg import BatteryStatus
        node = aggregator_node
        published = []
        original = node.health_pub.publish
        node.health_pub.publish = published.append
        try:
            msg = BatteryStatus()
            msg.percent = 0.25
            node._on_battery(msg)
            node._publish_health()
        finally:
            node.health_pub.publish = original
        assert published
        assert abs(published[-1].battery_percent - 25.0) < 1e-6
