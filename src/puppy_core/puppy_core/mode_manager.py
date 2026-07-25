"""Mode Manager: robot mode state machine.

Manages robot operational modes with priority-based switching
(gaijin2.md section 6.2). FAULT and SAFE_STOP have highest priority
and can interrupt any other mode.

Modes:
  IDLE, READY, TELEOP, NAVIGATION, PATROL, SECURITY, DOCKING, SAFE_STOP, FAULT
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from puppy_interfaces.msg import RobotMode, BatteryStatus, FallEvent


# Mode priority (higher number = higher priority)
MODE_PRIORITY = {
    'IDLE': 0,
    'READY': 1,
    'TELEOP': 2,
    'NAVIGATION': 3,
    'PATROL': 4,
    'SECURITY': 5,
    'DOCKING': 6,
    'SAFE_STOP': 90,
    'FAULT': 100,
}

VALID_MODES = set(MODE_PRIORITY.keys())


class ModeManagerNode(Node):
    """Manages robot operational modes with safety constraints."""

    def __init__(self):
        super().__init__('mode_manager')

        self.current_mode = 'IDLE'
        self.previous_mode = 'IDLE'
        self.motion_enabled = False
        self.autonomy_enabled = False
        self._mode_lock = False  # True when in FAULT/SAFE_STOP

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.mode_pub = self.create_publisher(RobotMode, '/robot/mode', qos)

        # Subscribers
        self.create_subscription(BatteryStatus, '/battery_status', self._on_battery, 10)
        self.create_subscription(FallEvent, '/fall/event', self._on_fall, 10)
        self.create_subscription(String, '/robot/mode_request', self._on_mode_request, 10)

        # Mode broadcast timer (5Hz)
        self.create_timer(0.2, self._publish_mode)

        self.get_logger().info('Mode manager started (mode=IDLE)')

    def _on_battery(self, msg: BatteryStatus):
        """Handle battery status - trigger SAFE_STOP on critical battery."""
        if msg.critical_battery and self.current_mode != 'FAULT':
            self._switch_mode('SAFE_STOP', 'critical battery')
        elif msg.low_battery and self.current_mode == 'PATROL':
            # Interrupt patrol for docking
            self._switch_mode('DOCKING', 'low battery during patrol')

    def _on_fall(self, msg: FallEvent):
        """Handle fall event - immediately enter SAFE_STOP."""
        if msg.detected and self.current_mode != 'FAULT':
            self._switch_mode('SAFE_STOP', f'fall detected: {msg.direction}')

    def _on_mode_request(self, msg: String):
        """Handle external mode request."""
        requested = msg.data.upper()
        if requested not in VALID_MODES:
            self.get_logger().warn(f'Invalid mode request: {requested}')
            return
        self._switch_mode(requested, 'external request')

    def _switch_mode(self, new_mode: str, reason: str = ""):
        """Switch mode with priority checking."""
        if self._mode_lock and MODE_PRIORITY[new_mode] < MODE_PRIORITY['SAFE_STOP']:
            self.get_logger().warn(
                f'Mode switch to {new_mode} blocked (locked in {self.current_mode})')
            return

        self.previous_mode = self.current_mode
        self.current_mode = new_mode

        # Update enable flags
        self.motion_enabled = new_mode not in ('IDLE', 'SAFE_STOP', 'FAULT')
        self.autonomy_enabled = new_mode in ('NAVIGATION', 'PATROL', 'SECURITY', 'DOCKING')

        # Lock if entering SAFE_STOP or FAULT
        if new_mode in ('SAFE_STOP', 'FAULT'):
            self._mode_lock = True
        elif new_mode == 'READY':
            # Unlock when explicitly returning to READY
            self._mode_lock = False

        self.get_logger().info(
            f'Mode: {self.previous_mode} -> {new_mode} ({reason})')

    def _publish_mode(self):
        """Broadcast current mode."""
        msg = RobotMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_mode = self.current_mode
        msg.previous_mode = self.previous_mode
        msg.motion_enabled = self.motion_enabled
        msg.autonomy_enabled = self.autonomy_enabled
        msg.reason = ''
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
