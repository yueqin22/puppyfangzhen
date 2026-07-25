"""Fake Status Server: simulates PuppyPi battery and health status."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from puppy_interfaces.msg import RobotHealth


class FakeStatusServerNode(Node):
    """Simulates PuppyPi battery, joints, and health status."""

    def __init__(self):
        super().__init__('fake_status_server')

        self.battery_percent = 0.85
        self.battery_drain_rate = 0.0001  # per tick

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.health_pub = self.create_publisher(RobotHealth, '/platform/health', 10)

        # Status timer (20Hz)
        self.create_timer(0.05, self._publish_status)

        self.get_logger().info('Fake status server started (battery=85%)')

    def _publish_status(self):
        # Simulate battery drain
        self.battery_percent = max(0.0, self.battery_percent - self.battery_drain_rate)

        # Battery
        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = 12.0 * (0.8 + 0.2 * self.battery_percent)
        bs.current = -1.0
        bs.percentage = self.battery_percent
        bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(bs)

        # Joint states (8DOF)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f'leg{i}_{j}' for i in range(4) for j in ['hip', 'knee']]
        js.position = [0.0] * 8
        self.joint_pub.publish(js)

        # Health
        h = RobotHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.ok = self.battery_percent > 0.1
        h.level = 'OK' if h.ok else 'ERROR'
        h.battery_percent = self.battery_percent * 100
        h.imu_ready = True
        h.lidar_ready = True
        h.camera_ready = True
        h.motion_ready = True
        self.health_pub.publish(h)


def main(args=None):
    rclpy.init(args=args)
    node = FakeStatusServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
