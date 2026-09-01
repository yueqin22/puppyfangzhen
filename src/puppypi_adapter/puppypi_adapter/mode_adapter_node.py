"""Mode Adapter Node: manages PuppyPi posture mode transitions.

Handles posture commands: STAND, SIT, LIE_DOWN, RECOVER, FREEZE
(gaijin2.md section 7.2)

P0-3: use_sim=True（默认）时只记录姿态命令不执行；
      use_sim=False 时尝试导入 PuppyPi SDK，失败则报错退出。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


class ModeAdapterNode(Node):
    """Manages PuppyPi posture and mode transitions."""

    POSTURES = ['STAND', 'SIT', 'LIE_DOWN', 'RECOVER', 'FREEZE']

    def __init__(self):
        super().__init__('mode_adapter')

        # P0-3: 仿真/真机模式开关
        self.declare_parameter('use_sim', True)
        self.use_sim = self.get_parameter('use_sim').value

        self.current_posture = 'UNKNOWN'
        self.motion_enabled = False

        # Publishers / Subscribers
        self.create_subscription(String, '/robot/posture_cmd', self._on_posture_cmd, 10)
        self.create_subscription(Bool, '/robot/motion_enable', self._on_enable, 10)

        # HardwareInterface handles both sim and real hardware abstraction
        self.hw = None
        self.sdk_connected = False
        backend = 'sim' if self.use_sim else 'real'
        try:
            from .hardware_interface import HardwareInterface
            self.hw = HardwareInterface.create({'backend': backend})
            self.sdk_connected = self.hw.initialize()
            if not self.sdk_connected:
                self.get_logger().warn(f'HardwareInterface ({backend}) initialization returned False, falling back to mock')
                self.hw = HardwareInterface.create({'backend': 'mock'})
                self.hw.initialize()
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to initialize HardwareInterface ({backend}): {exc}. Falling back to mock interface.'
            )
            try:
                from .hardware_interface import HardwareInterface
                self.hw = HardwareInterface.create({'backend': 'mock'})
                self.hw.initialize()
            except Exception as mock_exc:
                self.get_logger().error(f'Mock HardwareInterface initialization failed: {mock_exc}')
                self.hw = None

        mode_tag = '[SIM]' if self.use_sim else ('[HARDWARE]' if self.sdk_connected else '[FALLBACK-MOCK]')
        self.get_logger().info(f'Mode adapter started {mode_tag}')

    def _on_posture_cmd(self, msg: String):
        """Handle posture command."""
        posture = msg.data.upper()
        if posture not in self.POSTURES:
            self.get_logger().warn(f'Unknown posture: {posture}')
            return
        self.get_logger().info(f'Posture command: {posture}')
        self.current_posture = posture

        if self.hw:
            if posture == 'FREEZE':
                self.hw.emergency_stop('posture_cmd_freeze')
            elif posture == 'RECOVER':
                self.hw.release_emergency_stop()
                self.hw.enable_motors(True)
            elif posture == 'STAND':
                self.hw.enable_motors(True)
            elif posture in ('SIT', 'LIE_DOWN'):
                self.hw.send_velocity(0.0, 0.0, 0.0)

    def _on_enable(self, msg: Bool):
        """Handle motor enable/disable."""
        self.motion_enabled = msg.data
        self.get_logger().info(f'Motors {"enabled" if msg.data else "disabled"}')
        if self.hw:
            self.hw.enable_motors(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ModeAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
