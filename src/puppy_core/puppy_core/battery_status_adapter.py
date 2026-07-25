"""Adapt raw BatteryState into semantic BatteryStatus for puppy_core."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from puppy_interfaces.msg import BatteryStatus


class BatteryStatusAdapterNode(Node):
    """Convert sensor_msgs/BatteryState into puppy_interfaces/BatteryStatus."""

    def __init__(self):
        super().__init__('battery_status_adapter')

        self.declare_parameter('low_battery_threshold', 0.20)
        self.declare_parameter('critical_battery_threshold', 0.10)
        self.low_battery_threshold = float(
            self.get_parameter('low_battery_threshold').value
        )
        self.critical_battery_threshold = float(
            self.get_parameter('critical_battery_threshold').value
        )

        self.pub = self.create_publisher(BatteryStatus, '/battery_status', 10)
        self.create_subscription(BatteryState, '/battery_state', self._on_battery, 10)

        self.get_logger().info('Battery status adapter started')

    def _on_battery(self, msg: BatteryState):
        status = BatteryStatus()
        status.header = msg.header
        status.voltage = float(msg.voltage)
        status.current = float(msg.current)
        status.percent = float(max(0.0, min(1.0, msg.percentage)))
        status.charging = msg.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_FULL,
        )
        status.low_battery = status.percent <= self.low_battery_threshold
        status.critical_battery = status.percent <= self.critical_battery_threshold
        self.pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
