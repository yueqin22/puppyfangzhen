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

        # P0-3: SDK 初始化
        self.puppypi = None
        if not self.use_sim:
            try:
                raise ImportError('PuppyPi SDK not yet integrated')
            except ImportError as exc:
                self.get_logger().fatal(
                    f'use_sim=False but PuppyPi SDK unavailable: {exc}. '
                    f'Set use_sim:=true for simulation.')
                raise

        mode_tag = '[SIM]' if self.use_sim else '[HARDWARE]'
        self.get_logger().info(f'Mode adapter started {mode_tag}')

    def _on_posture_cmd(self, msg: String):
        """Handle posture command."""
        posture = msg.data.upper()
        if posture not in self.POSTURES:
            self.get_logger().warn(f'Unknown posture: {posture}')
            return
        self.get_logger().info(f'Posture command: {posture}')
        # P0-3: 仿真模式只记录，真机模式调用 SDK
        self.current_posture = posture
        if not self.use_sim:
            # TODO: self.puppypi.set_posture(posture)
            pass

    def _on_enable(self, msg: Bool):
        """Handle motor enable/disable."""
        self.motion_enabled = msg.data
        self.get_logger().info(f'Motors {"enabled" if msg.data else "disabled"}')
        # P0-3: 仿真模式只记录，真机模式调用 SDK
        if not self.use_sim:
            # TODO: self.puppypi.enable_motors(msg.data)
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ModeAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
