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
        # ``backend`` is the explicit contract-facing selector.  Keep
        # ``use_sim`` for backwards-compatible launch files, but never infer
        # a real backend failure as permission to switch to mock.
        self.declare_parameter('backend', '')
        self.declare_parameter('allow_backend_fallback', False)
        self.declare_parameter('publish_raw_battery', True)
        self.declare_parameter('publish_semantic_battery', True)
        self.declare_parameter('battery_drain_per_tick', 0.0002)

        self.use_sim = bool(self.get_parameter('use_sim').value)
        configured_backend = str(self.get_parameter('backend').value or '').strip().lower()
        self.allow_backend_fallback = bool(
            self.get_parameter('allow_backend_fallback').value)
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
        self.joint_names = [
            'FR_hip_yaw_joint', 'FR_hip_pitch_joint', 'FR_knee_joint',
            'FL_hip_yaw_joint', 'FL_hip_pitch_joint', 'FL_knee_joint',
            'RR_hip_yaw_joint', 'RR_hip_pitch_joint', 'RR_knee_joint',
            'RL_hip_yaw_joint', 'RL_hip_pitch_joint', 'RL_knee_joint',
        ]

        self.health_pub = self.create_publisher(RobotHealth, '/platform/health', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.battery_semantic_pub = self.create_publisher(BatteryStatus, '/battery_status', 10)

        self.hw = None
        backend = configured_backend or ('sim' if self.use_sim else 'real')
        if backend not in ('mock', 'sim', 'real'):
            raise ValueError(
                f"Unsupported status adapter backend '{backend}'; expected mock, sim, or real")
        try:
            from .hardware_interface import HardwareInterface
            self.hw = HardwareInterface.create({
                'backend': backend,
                'allow_backend_fallback': self.allow_backend_fallback,
            })
            self.sdk_connected = bool(self.hw.initialize())
            if not self.sdk_connected:
                raise RuntimeError(
                    f'HardwareInterface ({backend}) initialization returned False')
        except Exception as exc:
            # A real/sim backend is an explicit deployment choice.  Falling
            # back to mock hides disconnected sensors and makes a safety
            # demonstration invalid, so fail-fast unless a caller explicitly
            # opts into fallback (and records that choice in its launch).
            if self.allow_backend_fallback:
                self.get_logger().warning(
                    f'HardwareInterface ({backend}) failed: {exc}; '
                    'explicit allow_backend_fallback=true -> using mock')
                self.hw = HardwareInterface.create({'backend': 'mock'})
                self.sdk_connected = bool(self.hw.initialize())
                self.backend_actual = 'mock'
            else:
                self.get_logger().fatal(
                    f'HardwareInterface ({backend}) unavailable: {exc}; '
                    'refusing silent mock fallback')
                raise

        self.backend_requested = backend
        self.backend_actual = getattr(self, 'backend_actual', backend)

        self.create_timer(0.05, self._poll_status)

        mode_tag = {'sim': '[SIM]', 'real': '[HARDWARE]', 'mock': '[MOCK]'}[
            self.backend_actual]
        self.get_logger().info(
            f'Status adapter started {mode_tag} '
            f'(backend_requested={self.backend_requested}, '
            f'backend_actual={self.backend_actual})')

    def _poll_status(self):
        """Poll PuppyPi for status and publish to ROS2."""
        if self.hw:
            readings = self.hw.get_sensor_readings()
            battery_state = self.hw.get_battery()
            health_state = self.hw.get_health()

            self.battery_percent = battery_state.percent
            battery_voltage = battery_state.voltage
            battery_current = battery_state.current
            is_charging = battery_state.charging
            cpu_temp = health_state.cpu_temp
            health_ok = health_state.ok
            health_level = health_state.level
            active_faults = health_state.active_faults
            imu_ready = health_state.imu_ready
            lidar_ready = health_state.lidar_ready
            camera_ready = health_state.camera_ready
            motion_ready = health_state.motion_ready
        else:
            self.battery_percent = max(0.0, self.battery_percent - self.battery_drain_per_tick)
            battery_voltage = 12.0 * (0.8 + 0.2 * self.battery_percent)
            battery_current = -0.8
            is_charging = False
            cpu_temp = 0.0
            health_ok = self.battery_percent > 0.10
            health_level = 'WARN' if not self.sdk_connected else ('OK' if health_ok else 'ERROR')
            active_faults = []
            if not self.sdk_connected:
                active_faults.append('STATUS_SOURCE_SIMULATED')
            if self.battery_percent <= 0.10:
                active_faults.append('BATTERY_CRITICAL')
            elif self.battery_percent <= 0.20:
                active_faults.append('BATTERY_LOW')
            imu_ready = self.sdk_connected
            lidar_ready = self.sdk_connected
            camera_ready = self.sdk_connected
            motion_ready = True

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [0.0] * 12
        js.velocity = [0.0] * 12
        js.effort = [0.0] * 12
        self.joint_pub.publish(js)

        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = battery_voltage
        bs.current = battery_current
        bs.percentage = self.battery_percent
        bs.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if is_charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        if self.publish_raw_battery:
            self.battery_pub.publish(bs)

        if self.publish_semantic_battery:
            sem = BatteryStatus()
            sem.header = bs.header
            sem.voltage = bs.voltage
            sem.current = bs.current
            sem.percent = self.battery_percent
            sem.charging = is_charging
            sem.low_battery = self.battery_percent <= 0.20
            sem.critical_battery = self.battery_percent <= 0.10
            self.battery_semantic_pub.publish(sem)

        health = RobotHealth()
        health.header.stamp = bs.header.stamp
        health.ok = health_ok
        health.level = health_level
        health.active_faults = active_faults
        health.cpu_temp = cpu_temp
        health.battery_percent = self.battery_percent * 100.0
        health.imu_ready = imu_ready
        health.lidar_ready = lidar_ready
        health.camera_ready = camera_ready
        health.motion_ready = motion_ready
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
