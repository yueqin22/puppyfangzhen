"""Status Adapter Node: translates PuppyPi status to ROS2 messages.

Input: PuppyPi board feedback (motor, posture, errors)
Output: /platform/health, /joint_states, /battery_state (raw)
        /battery_status (semantic, optional)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState

from puppy_interfaces.msg import BatteryStatus, RobotHealth


class StatusAdapterNode(Node):
    """Translates PuppyPi platform status to standard ROS2 messages."""

    def __init__(self):
        super().__init__('status_adapter')

        self.declare_parameter('use_sim', True)
        self.declare_parameter('publish_raw_battery', True)
        self.declare_parameter('publish_semantic_battery', True)
        self.declare_parameter('battery_drain_per_tick', 0.0002)

        self.use_sim = bool(self.get_parameter('use_sim').value)
        self.publish_raw_battery = bool(
            self.get_parameter('publish_raw_battery').value
        )
        self.publish_semantic_battery = bool(
            self.get_parameter('publish_semantic_battery').value
        )
        self.battery_drain_per_tick = float(
            self.get_parameter('battery_drain_per_tick').value
        )

        self.battery_percent = 0.80
        self.sdk_connected = False
        self.joint_names = [f'leg{i}_{j}' for i in range(4) for j in ['hip', 'knee']]

        self.health_pub = self.create_publisher(RobotHealth, '/platform/health', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.battery_semantic_pub = self.create_publisher(BatteryStatus, '/battery_status', 10)

        self.create_timer(0.05, self._poll_status)

        self.puppypi = None
        if not self.use_sim:
            try:
                raise ImportError('PuppyPi SDK not yet integrated')
            except ImportError as exc:
                self.get_logger().fatal(
                    f'use_sim=False but PuppyPi SDK unavailable: {exc}. '
                    f'Set use_sim:=true for simulation.'
                )
                raise

        mode_tag = '[SIM]' if self.use_sim else '[HARDWARE]'
        self.get_logger().info(f'Status adapter started {mode_tag}')

    def _poll_status(self):
        """Poll PuppyPi for status and publish to ROS2."""
        self.battery_percent = max(0.0, self.battery_percent - self.battery_drain_per_tick)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [0.0] * 8
        js.velocity = [0.0] * 8
        js.effort = [0.0] * 8
        self.joint_pub.publish(js)

        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = 12.0 * (0.8 + 0.2 * self.battery_percent)
        bs.current = -0.8
        bs.percentage = self.battery_percent
        bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if self.publish_raw_battery:
            self.battery_pub.publish(bs)

        if self.publish_semantic_battery:
            sem = BatteryStatus()
            sem.header = bs.header
            sem.voltage = bs.voltage
            sem.current = bs.current
            sem.percent = self.battery_percent
            sem.charging = False
            sem.low_battery = self.battery_percent <= 0.20
            sem.critical_battery = self.battery_percent <= 0.10
            self.battery_semantic_pub.publish(sem)

        health = RobotHealth()
        health.header.stamp = bs.header.stamp
        health.ok = self.battery_percent > 0.10
        health.level = 'WARN' if not self.sdk_connected else ('OK' if health.ok else 'ERROR')
        health.active_faults = []
        if not self.sdk_connected:
            health.active_faults.append('STATUS_SOURCE_SIMULATED')
        if self.battery_percent <= 0.10:
            health.active_faults.append('BATTERY_CRITICAL')
        elif self.battery_percent <= 0.20:
            health.active_faults.append('BATTERY_LOW')
        health.cpu_temp = 0.0
        health.battery_percent = self.battery_percent * 100.0
        health.imu_ready = self.sdk_connected
        health.lidar_ready = self.sdk_connected
        health.camera_ready = self.sdk_connected
        health.motion_ready = True
        self.health_pub.publish(health)


def main(args=None):
    rclpy.init(args=args)
    node = StatusAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
