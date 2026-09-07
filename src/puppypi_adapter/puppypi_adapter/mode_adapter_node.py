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
        self.declare_parameter('backend', '')
        self.use_sim = self.get_parameter('use_sim').value
        configured_backend = str(self.get_parameter('backend').value or '').strip().lower()

        self.current_posture = 'UNKNOWN'
        self.motion_enabled = False

        # Publishers / Subscribers
        self.create_subscription(String, '/robot/posture_cmd', self._on_posture_cmd, 10)
        self.create_subscription(Bool, '/robot/motion_enable', self._on_enable, 10)

        # HardwareInterface handles both sim and real hardware abstraction
        self.hw = None
        self.sdk_connected = False
        # P0-4: backend_actual 记录"实际生效"的后端, 绝不因回退而谎报 configured 后端
        # use_sim=True -> backend 'sim' (外部仿真器拥有动力学, 本节点只发命令与
        # 使能/急停; 传感器数据由仿真器经 ROS topic 注入, 见 sim_interface.py)。
        # 这与 'mock'(进程内自成体系的假动力学) 语义不同, 不得混用。
        self.backend_requested = configured_backend or ('sim' if self.use_sim else 'real')
        self.backend_actual = None

        try:
            from .hardware_interface import HardwareInterface
            self.hw = HardwareInterface.create({'backend': self.backend_requested})
            self.sdk_connected = self.hw.initialize()
            self.backend_actual = self.backend_requested
            if not self.sdk_connected:
                # P0-4: 初始化失败不再静默回退 mock —— 显式报错并退出, 避免
                # "配置 real 却跑在 mock 上" 的静默降级被误当作真实运行.
                raise RuntimeError(
                    f'HardwareInterface ({self.backend_requested}) initialize() returned False')
        except Exception as exc:
            self.get_logger().fatal(
                f'HardwareInterface ({self.backend_requested}) unavailable: {exc}. '
                f'P0-4 禁止静默回退 mock: 请修正 backend 配置或安装对应后端 '
                f'(sim 需 sim_interface, real 需 PuppyPi SDK 且预检通过).'
            )
            raise

        self.backend_actual = self.backend_requested
        mode_tag = ('[SIM]' if self.use_sim
                    else ('[HARDWARE]' if self.sdk_connected else '[UNKNOWN]'))
        self.get_logger().info(
            f'Mode adapter started {mode_tag} '
            f'(backend={self.backend_actual}, sdk_connected={self.sdk_connected})')

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
