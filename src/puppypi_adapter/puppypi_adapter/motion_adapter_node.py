"""Motion Adapter Node: translates /cmd_vel to PuppyPi motion commands.

State machine: INIT -> DISABLED -> READY -> EXECUTING -> RECOVERING -> SAFE_STOP -> FAULT
(gaijin2.md section 5.3)

Input:
  - /cmd_vel (geometry_msgs/Twist)
  - StandUp/SitDown/RecoverPosture actions

Output:
  - PuppyPi SDK motion commands (via puppypi_control client)
  - /platform/motion_state (PlatformMotionState)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from puppy_interfaces.msg import PlatformMotionState


class MotionState:
    INIT = 'INIT'
    DISABLED = 'DISABLED'
    READY = 'READY'
    EXECUTING = 'EXECUTING'
    RECOVERING = 'RECOVERING'
    SAFE_STOP = 'SAFE_STOP'
    FAULT = 'FAULT'


class MotionAdapterNode(Node):
    """Translates velocity commands to PuppyPi platform actions."""

    def __init__(self):
        super().__init__('motion_adapter')

        # P0-3: 仿真/真机模式开关
        # use_sim=True（默认）：不依赖 SDK，自动 INIT→READY，用于 mock/sim 链路
        # use_sim=False：尝试导入 PuppyPi SDK，失败则报错退出
        self.declare_parameter('use_sim', True)
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_linear_y', 0.3)
        self.declare_parameter('max_angular_z', 1.2)
        self.declare_parameter('cmd_timeout', 1.0)
        self.declare_parameter('accel_limit', 2.0)
        self.declare_parameter('yaw_rate_limit', 4.0)
        self.use_sim = self.get_parameter('use_sim').value

        # Platform state
        self.platform_state = MotionState.INIT
        self.standing = False
        self.moving = False
        self.controllable = False
        self.gait_mode = 'idle'

        # Velocity limits (gaijin2.md section 7.3)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.last_cmd_time = self.get_clock().now()

        # Smoothed velocity (for accel limiting)
        # P0-3: vy 与 vx 同等对待。此前这里只有 current_vx/current_wz,
        # 而 _on_cmd_vel 根本不读 msg.linear.y, 真机侧 send_velocity() 更是把
        # 第二个参数硬编码成 0.0 —— 导航发出的横移指令在整条 adapter 链上被
        # 静默丢弃。平台实测横移可兑现 82%~118% (见 config/motion_capability.yaml),
        # 脱困行为正依赖它, 所以这里必须能走通。
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.accel_limit = float(self.get_parameter('accel_limit').value)
        self.yaw_rate_limit = float(self.get_parameter('yaw_rate_limit').value)

        # Publishers
        self.state_pub = self.create_publisher(PlatformMotionState, '/platform/motion_state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/platform/cmd', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_safe', self._on_cmd_vel, 10)

        # State machine timer (20Hz)
        self.create_timer(0.05, self._update_state)

        # P0-3: SDK 初始化
        self.puppypi = None
        if not self.use_sim:
            try:
                # 尝试导入 PuppyPi SDK
                # from puppypi_control import PuppyPiClient
                # self.puppypi = PuppyPiClient(...)
                from .puppypi_driver import PuppyPiHardwareInterface
                self.puppypi = PuppyPiHardwareInterface({'backend': 'real'})
                if not self.puppypi.initialize():
                    raise RuntimeError('PuppyPi hardware initialization failed')
            except ImportError as exc:
                self.get_logger().fatal(
                    f'use_sim=False but PuppyPi SDK unavailable: {exc}. '
                    f'Set use_sim:=true for simulation.')
                raise

        mode_tag = '[SIM]' if self.use_sim else '[HARDWARE]'
        self.get_logger().info(f'Motion adapter started {mode_tag} (state=INIT)')

    def _on_cmd_vel(self, msg: Twist):
        """Handle incoming velocity command with safety processing."""
        if not self.controllable or not self.standing:
            return  # Ignore commands when not controllable

        self.last_cmd_time = self.get_clock().now()

        # Apply limits (gaijin2.md 7.3)
        target_vx = max(-self.max_linear_x, min(self.max_linear_x, msg.linear.x))
        target_vy = max(-self.max_linear_y, min(self.max_linear_y, msg.linear.y))
        target_wz = max(-self.max_angular_z, min(self.max_angular_z, msg.angular.z))

        # Smooth acceleration
        dt = 0.05
        max_dv = self.accel_limit * dt
        max_dw = self.yaw_rate_limit * dt
        self.current_vx = max(self.current_vx - max_dv,
                              min(self.current_vx + max_dv, target_vx))
        self.current_vy = max(self.current_vy - max_dv,
                              min(self.current_vy + max_dv, target_vy))
        self.current_wz = max(self.current_wz - max_dw,
                              min(self.current_wz + max_dw, target_wz))

        # Dead zone
        if abs(self.current_vx) < 0.01:
            self.current_vx = 0.0
        if abs(self.current_vy) < 0.01:
            self.current_vy = 0.0
        if abs(self.current_wz) < 0.05:
            self.current_wz = 0.0

        # P0-3: 仿真模式下只记录速度，真机模式下发送到 SDK
        if self.use_sim:
            # 仿真模式：速度已记录在 self.current_vx/vy/wz，供状态发布
            pass
        else:
            # 真机模式：发送到 PuppyPi SDK
            # 不再把 vy 硬编码成 0.0：那会让"导航发出横移、底盘毫无反应"这种
            # 静默失效无法被发现。上限由 max_linear_y 约束（契约单一真源）。
            if self.puppypi:
                self.puppypi.send_velocity(self.current_vx, self.current_vy,
                                           self.current_wz)

        self.moving = (abs(self.current_vx) > 0.01
                       or abs(self.current_vy) > 0.01
                       or abs(self.current_wz) > 0.05)
        if self.moving:
            self.platform_state = MotionState.EXECUTING

    def _update_state(self):
        """State machine update and timeout handling."""
        now = self.get_clock().now()

        # Command timeout -> stop
        if (self.controllable and self.standing and
                (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout):
            # P0-4: 命令超时 -> 三轴全部归零。漏掉 vy 会留下"超时后仍在横移"的
            # 安全缺口。
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0
            if self.platform_state == MotionState.EXECUTING:
                self.platform_state = MotionState.READY
                self.moving = False

        # P0-3: 状态机转换
        # 仿真模式：INIT → DISABLED → READY（自动，无需 SDK）
        # 真机模式：需要 SDK 确认 standing 后才到 READY
        if self.platform_state == MotionState.INIT:
            self.platform_state = MotionState.DISABLED
        elif self.platform_state == MotionState.DISABLED:
            if self.use_sim:
                # 仿真模式：自动进入 READY
                self.platform_state = MotionState.READY
                self.standing = True
                self.controllable = True
                self.gait_mode = 'stand'
            # 真机模式：等待 SDK 确认 standing（TODO: SDK 集成后实现）
        elif self.platform_state == MotionState.READY and self.use_sim:
            # 仿真模式：保持 standing 和 controllable
            self.standing = True
            self.controllable = True

        self._publish_state()

    def _publish_state(self):
        """Publish platform motion state."""
        msg = PlatformMotionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.standing = self.standing
        msg.moving = self.moving
        msg.controllable = self.controllable
        msg.gait_mode = self.gait_mode
        msg.linear_x = self.current_vx
        # 上报真实的横移速度。写死 0.0 会让 /platform/motion_state 与底盘实际
        # 状态不一致 —— 上层监控看到的永远是"没有横移", 脱困是否生效无从判断。
        msg.linear_y = self.current_vy
        msg.angular_z = self.current_wz
        msg.platform_state = self.platform_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
