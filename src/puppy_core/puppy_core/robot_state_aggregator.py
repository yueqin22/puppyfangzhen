"""Robot State Aggregator: unified robot health from semantic topics."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

from puppy_interfaces.msg import BatteryStatus, FallEvent, PlatformMotionState, RobotHealth


_DEFAULT_TIMEOUTS = {
    'battery': 10.0,
    'motion': 5.0,
    'imu': 2.0,
    'lidar': 3.0,
}


class RobotStateAggregatorNode(Node):
    """Aggregates robot state from multiple sources into unified health."""

    def __init__(self):
        super().__init__('robot_state_aggregator')

        self.declare_parameter('battery_timeout', _DEFAULT_TIMEOUTS['battery'])
        self.declare_parameter('motion_timeout', _DEFAULT_TIMEOUTS['motion'])
        self.declare_parameter('imu_timeout', _DEFAULT_TIMEOUTS['imu'])
        self.declare_parameter('lidar_timeout', _DEFAULT_TIMEOUTS['lidar'])
        self.timeouts = {
            'battery': self.get_parameter('battery_timeout').value,
            'motion': self.get_parameter('motion_timeout').value,
            'imu': self.get_parameter('imu_timeout').value,
            'lidar': self.get_parameter('lidar_timeout').value,
        }

        self.battery_percent = 1.0
        self.imu_ready = False
        self.lidar_ready = False
        self.camera_ready = False
        self.motion_ready = False
        self.active_faults = []
        self.platform_health = None

        self.last_seen = {
            'battery': None,
            'motion': None,
            'imu': None,
            'lidar': None,
        }
        self._reported_timeout_faults = set()

        self.health_pub = self.create_publisher(RobotHealth, '/robot/health', 10)

        self.create_subscription(BatteryStatus, '/battery_status', self._on_battery, 10)
        self.create_subscription(PlatformMotionState, '/platform/motion_state', self._on_motion, 10)
        self.create_subscription(FallEvent, '/fall/event', self._on_fall, 10)
        self.create_subscription(RobotHealth, '/platform/health', self._on_platform_health, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        self.create_timer(0.5, self._publish_health)

        self.get_logger().info(f'Robot state aggregator started (timeouts: {self.timeouts})')

    def _on_battery(self, msg: BatteryStatus):
        self.battery_percent = float(max(0.0, min(1.0, msg.percent)))
        self.last_seen['battery'] = self.get_clock().now()

    def _on_motion(self, msg: PlatformMotionState):
        self.motion_ready = msg.controllable
        self.last_seen['motion'] = self.get_clock().now()

    def _on_imu(self, msg: Imu):
        self.last_seen['imu'] = self.get_clock().now()

    def _on_scan(self, msg: LaserScan):
        self.last_seen['lidar'] = self.get_clock().now()

    def _on_fall(self, msg: FallEvent):
        if msg.detected:
            if 'FALL_DETECTED' not in self.active_faults:
                self.active_faults.append('FALL_DETECTED')
        elif 'FALL_DETECTED' in self.active_faults:
            self.active_faults.remove('FALL_DETECTED')

    def _on_platform_health(self, msg: RobotHealth):
        self.platform_health = msg
        self.camera_ready = msg.camera_ready

    def _check_timeouts(self) -> list:
        now = self.get_clock().now()
        timeout_faults = []
        for source, last in self.last_seen.items():
            if last is None:
                timeout_faults.append(f'{source.upper()}_OFFLINE')
                continue
            elapsed = (now - last).nanoseconds / 1e9
            if elapsed > self.timeouts[source]:
                timeout_faults.append(f'{source.upper()}_TIMEOUT')

        for fault in timeout_faults:
            if fault not in self._reported_timeout_faults:
                self.get_logger().warn(f'Sensor timeout: {fault}')
                self._reported_timeout_faults.add(fault)

        recovered = self._reported_timeout_faults - set(timeout_faults)
        for fault in recovered:
            self.get_logger().info(f'Sensor recovered: {fault}')
        self._reported_timeout_faults = set(timeout_faults)

        return timeout_faults

    def _publish_health(self):
        msg = RobotHealth()
        msg.header.stamp = self.get_clock().now().to_msg()

        timeout_faults = self._check_timeouts()
        timeout_set = {fault.split('_')[0] for fault in timeout_faults}

        self.imu_ready = 'IMU' not in timeout_set and self.last_seen['imu'] is not None
        self.lidar_ready = 'LIDAR' not in timeout_set and self.last_seen['lidar'] is not None
        battery_ok = 'BATTERY' not in timeout_set
        if 'MOTION' in timeout_set:
            self.motion_ready = False

        active_faults = list(self.active_faults)
        if self.platform_health is not None:
            for fault in self.platform_health.active_faults:
                if fault not in active_faults:
                    active_faults.append(fault)
        for fault in timeout_faults:
            if fault not in active_faults:
                active_faults.append(fault)

        msg.cpu_temp = self.platform_health.cpu_temp if self.platform_health else 0.0
        msg.ok = len(active_faults) == 0 and battery_ok and self.battery_percent > 0.1
        msg.level = 'OK' if msg.ok else 'ERROR'
        msg.active_faults = active_faults
        msg.battery_percent = (self.battery_percent * 100.0) if battery_ok else 0.0
        msg.imu_ready = self.imu_ready
        msg.lidar_ready = self.lidar_ready
        msg.camera_ready = self.camera_ready
        msg.motion_ready = self.motion_ready
        self.health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
