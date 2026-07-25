"""Adapt legacy /fall_detected Bool topic into semantic FallEvent."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from puppy_interfaces.msg import FallEvent


class FallEventAdapterNode(Node):
    """Convert legacy Bool fall signal into puppy_interfaces/FallEvent."""

    def __init__(self):
        super().__init__('fall_event_adapter')

        self.pub = self.create_publisher(FallEvent, '/fall/event', 10)
        self.create_subscription(Bool, '/fall_detected', self._on_fall_detected, 10)

        self.get_logger().info('Fall event adapter started')

    def _on_fall_detected(self, msg: Bool):
        event = FallEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.detected = bool(msg.data)
        event.direction = 'unknown'
        event.confidence = 1.0 if msg.data else 0.0
        event.source = 'legacy_bool_adapter'
        self.pub.publish(event)


def main(args=None):
    rclpy.init(args=args)
    node = FallEventAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
