"""Goal Dispatcher: routes named goals to the active navigation stack."""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

from puppy_core.config_utils import load_navigation_targets


class GoalDispatcherNode(Node):
    """Dispatches navigation goals to the active planner."""

    def __init__(self):
        super().__init__('goal_dispatcher')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(String, '/goal/named', self._on_named_goal, 10)
        self.create_subscription(PoseStamped, '/goal/pose', self._on_pose_goal, 10)

        self.named_goals = load_navigation_targets()

        self.get_logger().info(f'Goal dispatcher started with {len(self.named_goals)} named goals')

    def _on_named_goal(self, msg: String):
        """Handle named goal request."""
        name = msg.data.lower()
        if name not in self.named_goals:
            self.get_logger().warn(f'Unknown named goal: {name}')
            return
        goal = self.named_goals[name]
        self._send_goal(goal['x'], goal['y'], goal.get('yaw', 0.0))

    def _on_pose_goal(self, msg: PoseStamped):
        """Handle direct pose goal."""
        self._send_goal(
            msg.pose.position.x,
            msg.pose.position.y,
            0.0,
            frame_id=msg.header.frame_id or 'map',
        )

    def _send_goal(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map'):
        """Send goal to navigation action server."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Navigation action not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav_client.send_goal_async(goal)
        self.get_logger().info(f'Goal sent: ({x:.1f}, {y:.1f}, yaw={yaw:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
