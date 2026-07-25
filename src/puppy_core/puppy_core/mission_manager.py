"""Mission Manager: high-level task orchestration over Nav2 primitives."""
import json
import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from puppy_core.config_utils import get_dock_target, get_patrol_route
from puppy_interfaces.msg import BatteryStatus, PatrolStatus, SecurityEvent


class MissionManagerNode(Node):
    """Orchestrates high-level robot missions."""

    def __init__(self):
        super().__init__('mission_manager')

        self.current_mission = 'IDLE'
        self.dock_target = get_dock_target({'x': 0.0, 'y': -2.0, 'yaw': 0.0})
        self.patrol_route = get_patrol_route()
        self.current_waypoint_index = -1
        self.goal_handle = None
        self.pending_after_nav = None
        self.last_status_message = 'idle'
        self.capabilities = {}

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(String, '/mission/command', self._on_command, 10)
        self.create_subscription(SecurityEvent, '/security/event', self._on_security, 10)
        self.create_subscription(BatteryStatus, '/battery_status', self._on_battery, 10)
        self.create_subscription(String, '/robot/capabilities', self._on_capabilities, 10)

        self.status_pub = self.create_publisher(PatrolStatus, '/mission/status', 10)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'Mission manager started with patrol route of {len(self.patrol_route)} waypoints'
        )

    def _on_capabilities(self, msg: String):
        try:
            self.capabilities = json.loads(msg.data)
        except Exception:
            pass

    def _is_capability_available(self, name: str) -> bool:
        cap = self.capabilities.get(name)
        if cap is None:
            return True
        return cap.get('enabled', True) and cap.get('available', True)

    def _on_command(self, msg: String):
        cmd = msg.data.lower()
        self.get_logger().info(f'Mission command: {cmd}')
        if cmd == 'patrol':
            self._start_patrol()
        elif cmd == 'dock':
            self._start_docking()
        elif cmd == 'stop':
            self._cancel_mission()
        elif cmd.startswith('goto:'):
            self._start_manual_navigation(cmd)

    def _start_manual_navigation(self, cmd: str):
        if not self._is_capability_available('navigation'):
            self.get_logger().warn('Navigation capability unavailable - manual goal rejected')
            self.last_status_message = 'manual goal rejected: navigation unavailable'
            return

        try:
            parts = cmd.split(':', 1)
            coords = [value.strip() for value in parts[1].split(',')]
            if len(coords) != 2:
                raise ValueError('expected x,y')
            goal_x = float(coords[0])
            goal_y = float(coords[1])
        except (IndexError, ValueError) as exc:
            self.get_logger().warn(f'Invalid goto command "{cmd}": {exc}')
            self.last_status_message = 'manual goal rejected: invalid coordinates'
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose action not available')
            self.last_status_message = 'manual goal rejected: navigation action unavailable'
            return

        self._cancel_mission()
        started = self._start_navigation(
            goal_x,
            goal_y,
            0.0,
            self._on_goto_result,
        )
        if started:
            self.current_mission = 'NAVIGATION'
            self.last_status_message = f'manual goal ({goal_x}, {goal_y})'

    def _on_security(self, msg: SecurityEvent):
        if msg.severity == 'critical':
            self.get_logger().warn(f'Security alert: {msg.event_type} - interrupting mission')
            self._cancel_mission()

    def _on_battery(self, msg: BatteryStatus):
        if msg.low_battery and self.current_mission == 'PATROL':
            self.get_logger().info('Low battery - returning to dock')
            self._cancel_mission()
            self._start_docking()

    def _start_navigation(self, x: float, y: float, yaw: float = 0.0, on_complete=None) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose action not available')
            self.last_status_message = 'navigation action unavailable'
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        self.pending_after_nav = on_complete
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_response)
        return True

    def _start_patrol(self):
        if not self.patrol_route:
            self.get_logger().error('No patrol route configured')
            self.last_status_message = 'no patrol route configured'
            return
        if not self._is_capability_available('patrol'):
            self.get_logger().warn('Patrol capability unavailable - patrol rejected')
            self.current_mission = 'IDLE'
            self.last_status_message = 'patrol rejected: capability unavailable'
            return
        if not self._is_capability_available('navigation'):
            self.get_logger().warn('Navigation capability unavailable - patrol rejected')
            self.current_mission = 'IDLE'
            self.last_status_message = 'patrol rejected: navigation unavailable'
            return

        self._cancel_mission()
        self.current_mission = 'PATROL'
        self.current_waypoint_index = 0
        self.last_status_message = 'patrol started'
        self._dispatch_patrol_waypoint()

    def _start_docking(self):
        if not self._is_capability_available('docking'):
            self.get_logger().warn('Docking capability unavailable - docking rejected')
            self.current_mission = 'IDLE'
            self.last_status_message = 'docking rejected: capability unavailable'
            return
        if not self._is_capability_available('navigation'):
            self.get_logger().warn('Navigation capability unavailable - docking rejected')
            self.current_mission = 'IDLE'
            self.last_status_message = 'docking rejected: navigation unavailable'
            return

        self._cancel_mission()
        self.current_mission = 'DOCKING'
        self.current_waypoint_index = -1
        self.last_status_message = 'returning to dock'
        started = self._start_navigation(
            self.dock_target['x'],
            self.dock_target['y'],
            self.dock_target.get('yaw', 0.0),
            self._on_dock_result,
        )
        if not started:
            self.current_mission = 'IDLE'

    def _cancel_mission(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Failed to cancel navigation goal: {exc}')
        self.goal_handle = None
        self.pending_after_nav = None
        self.current_waypoint_index = -1
        self.current_mission = 'IDLE'
        self.last_status_message = 'mission cancelled'
        self.get_logger().info('Mission cancelled')

    def _dispatch_patrol_waypoint(self):
        if self.current_mission != 'PATROL':
            return

        if self.current_waypoint_index >= len(self.patrol_route):
            self.get_logger().info('Patrol route complete - returning to dock')
            self._start_docking()
            return

        waypoint = self.patrol_route[self.current_waypoint_index]
        self.last_status_message = f'navigating to {waypoint.get("name", self.current_waypoint_index)}'
        self.get_logger().info(
            f'Patrol waypoint {self.current_waypoint_index + 1}/{len(self.patrol_route)}: '
            f'{waypoint.get("name", "waypoint")} ({waypoint["x"]:.2f}, {waypoint["y"]:.2f})'
        )
        started = self._start_navigation(
            waypoint['x'],
            waypoint['y'],
            waypoint.get('yaw', 0.0),
            self._on_patrol_waypoint_result,
        )
        if not started:
            self.current_mission = 'IDLE'

    def _on_nav_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.goal_handle = None
            self.get_logger().error('Navigation goal rejected')
            self.last_status_message = 'navigation goal rejected'
            self._handle_nav_completion(False)
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self.goal_handle = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Navigation result failed: {exc}')
            self.last_status_message = 'navigation result failed'
            self._handle_nav_completion(False)
            return

        success = result.status == GoalStatus.STATUS_SUCCEEDED
        if success:
            self.last_status_message = 'navigation goal reached'
        else:
            self.last_status_message = f'navigation failed ({result.status})'
            self.get_logger().warn(self.last_status_message)
        self._handle_nav_completion(success)

    def _handle_nav_completion(self, success: bool):
        callback = self.pending_after_nav
        self.pending_after_nav = None
        if callback is not None:
            callback(success)

    def _on_patrol_waypoint_result(self, success: bool):
        if self.current_mission != 'PATROL':
            return

        if not success:
            failed_waypoint = self.patrol_route[self.current_waypoint_index]
            self.get_logger().warn(
                f'Patrol failed at {failed_waypoint.get("name", self.current_waypoint_index)}; aborting patrol'
            )
            self.current_mission = 'IDLE'
            self.last_status_message = 'patrol aborted'
            return

        reached = self.patrol_route[self.current_waypoint_index]
        self.last_status_message = f'reached {reached.get("name", self.current_waypoint_index)}'
        self.current_waypoint_index += 1
        self._dispatch_patrol_waypoint()

    def _on_dock_result(self, success: bool):
        self.current_waypoint_index = -1
        self.current_mission = 'IDLE'
        if success:
            self.last_status_message = 'docked'
            self.get_logger().info('Dock target reached')
        else:
            self.last_status_message = 'dock navigation failed'
            self.get_logger().warn('Failed to reach dock target')

    def _on_goto_result(self, success: bool):
        self.current_mission = 'IDLE'
        self.last_status_message = 'manual goal reached' if success else 'manual goal failed'

    def _publish_status(self):
        msg = PatrolStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.current_mission
        msg.current_waypoint_index = self.current_waypoint_index
        msg.total_waypoints = len(self.patrol_route)
        if self.current_mission == 'PATROL' and self.patrol_route:
            msg.completion_ratio = min(
                1.0,
                max(0.0, float(self.current_waypoint_index) / float(len(self.patrol_route))),
            )
        elif self.current_mission == 'DOCKING':
            msg.completion_ratio = 1.0
        else:
            msg.completion_ratio = 0.0
        msg.message = self.last_status_message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
