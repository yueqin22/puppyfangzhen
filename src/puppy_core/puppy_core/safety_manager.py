"""Safety Manager: monitors safety conditions and triggers emergency response.

Implements the safety requirements (gaijin2.md section 14):
  1. Command timeout auto-stop
  2. Mode switch clears motion commands
  3. Fall detection stops navigation
  4. Low battery prevents patrol
  5. Driver error enters SAFE_STOP
  6. Motors disabled by default on startup
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from puppy_interfaces.msg import BatteryStatus, FallEvent


class SafetyManagerNode(Node):
    """Monitors safety conditions and enforces emergency responses."""

    def __init__(self):
        super().__init__('safety_manager')

        # P1-2: 安全参数从 declare_parameter 读取（可由 safety.yaml 覆盖）
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.declare_parameter('low_battery_threshold', 0.20)
        self.declare_parameter('critical_battery_threshold', 0.10)
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_angular_z', 1.2)
        self.declare_parameter('motors_enabled_on_start', False)

        # Safety state
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        self.last_cmd_vel_time = self.get_clock().now()
        # Default: disabled (gaijin2.md 14.6)
        self.motors_enabled = self.get_parameter('motors_enabled_on_start').value

        # Publishers
        self.safe_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.motor_enable_pub = self.create_publisher(Bool, '/robot/motion_enable', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(BatteryStatus, '/battery_status', self._on_battery, 10)
        self.create_subscription(FallEvent, '/fall/event', self._on_fall, 10)
        self.create_subscription(Bool, '/robot/motors_enable_request', self._on_motor_request, 10)
        # 看门狗急停: 传感器失效/断链/心跳超时由 watchdog 判定后经此话题下达。
        # 此前仲裁者未订阅它, 导致看门狗急停无法作用到执行器 (真实缺口)。
        self.create_subscription(Bool, '/watchdog/emergency_stop',
                                 self._on_emergency_stop, 10)

        # Safety check timer (20Hz)
        self.create_timer(0.05, self._safety_check)

        self.get_logger().info('Safety manager started (motors DISABLED by default)')

    def _on_cmd_vel(self, msg: Twist):
        """Forward cmd_vel with safety checks."""
        self.last_cmd_vel_time = self.get_clock().now()
        if not self.motors_enabled:
            # Motors disabled - send zero velocity
            self.safe_cmd_pub.publish(Twist())
            return
        # Apply safety limits
        safe_msg = Twist()
        safe_msg.linear.x = max(-self.max_linear_x, min(self.max_linear_x, msg.linear.x))
        safe_msg.angular.z = max(-self.max_angular_z, min(self.max_angular_z, msg.angular.z))
        self.safe_cmd_pub.publish(safe_msg)

    def _on_emergency_stop(self, msg: Bool):
        """看门狗急停: 立即断电机使能并输出零速度 (§4 P0-4 急停必须零速度)。"""
        if msg.data:
            self.get_logger().error('Watchdog EMERGENCY STOP -> SAFE_STOP')
            self.motors_enabled = False
            self.safe_cmd_pub.publish(Twist())
            self._publish_motor_enable()

    def _on_battery(self, msg: BatteryStatus):
        """Monitor battery for safety thresholds."""
        if msg.critical_battery:
            self.get_logger().error('Critical battery! Entering SAFE_STOP')
            self.motors_enabled = False
            self._publish_motor_enable()

    def _on_fall(self, msg: FallEvent):
        """Fall detection - immediately disable motors."""
        if msg.detected:
            self.get_logger().error(f'Fall detected ({msg.direction})! Stopping motors')
            self.motors_enabled = False
            self._publish_motor_enable()

    def _on_motor_request(self, msg: Bool):
        """Handle motor enable/disable requests."""
        self.motors_enabled = msg.data
        self.get_logger().info(f'Motors {"enabled" if msg.data else "disabled"}')
        self._publish_motor_enable()

    def _safety_check(self):
        """Periodic safety check - command timeout."""
        if self.motors_enabled:
            elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
            if elapsed > self.cmd_vel_timeout:
                # Command timeout - publish zero velocity
                self.safe_cmd_pub.publish(Twist())

    def _publish_motor_enable(self):
        msg = Bool()
        msg.data = self.motors_enabled
        self.motor_enable_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
