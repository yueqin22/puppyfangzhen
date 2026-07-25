"""Fake Motion Server: simulates PuppyPi motion platform.

Responds to /cmd_vel and posture commands, simulates motion execution
with realistic delays and publishes PlatformMotionState.
"""
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from puppy_interfaces.msg import PlatformMotionState


class FakeMotionServerNode(Node):
    """Simulated PuppyPi motion platform."""

    def __init__(self):
        super().__init__('fake_motion_server')

        self.standing = False
        self.moving = False
        self.controllable = False
        self.current_posture = 'SIT'
        self.vx = 0.0
        self.wz = 0.0

        # Simulated position (for odom feedback)
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_yaw = 0.0

        # Publishers
        self.state_pub = self.create_publisher(PlatformMotionState, '/platform/motion_state', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_safe', self._on_cmd_vel, 10)
        self.create_subscription(String, '/robot/posture_cmd', self._on_posture, 10)

        # Sim timer (20Hz)
        self.create_timer(0.05, self._update_sim)

        # Auto-stand after 2 seconds (simulated startup)
        self.create_timer(2.0, self._auto_stand)

        self.get_logger().info('Fake motion server started (will auto-stand in 2s)')

    def _auto_stand(self):
        """Simulate automatic stand-up sequence."""
        if not self.standing:
            self.standing = True
            self.controllable = True
            self.current_posture = 'STAND'
            self.get_logger().info('Fake platform: STOOD UP, controllable=True')

    def _on_cmd_vel(self, msg: Twist):
        if not self.controllable:
            return
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.moving = abs(self.vx) > 0.01 or abs(self.wz) > 0.05

    def _on_posture(self, msg: String):
        posture = msg.data.upper()
        self.get_logger().info(f'Fake posture: {posture}')
        self.current_posture = posture
        if posture == 'STAND':
            self.standing = True
            self.controllable = True
        elif posture in ('SIT', 'LIE_DOWN', 'FREEZE'):
            self.standing = False
            self.controllable = False
            self.vx = 0.0
            self.wz = 0.0

    def _update_sim(self):
        """Simulate motion and publish state."""
        if self.controllable and self.moving:
            dt = 0.05
            self.sim_x += self.vx * math.cos(self.sim_yaw) * dt
            self.sim_y += self.vx * math.sin(self.sim_yaw) * dt
            self.sim_yaw += self.wz * dt
            # Normalize yaw
            while self.sim_yaw > math.pi:
                self.sim_yaw -= 2 * math.pi
            while self.sim_yaw < -math.pi:
                self.sim_yaw += 2 * math.pi

        msg = PlatformMotionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.standing = self.standing
        msg.moving = self.moving
        msg.controllable = self.controllable
        msg.gait_mode = 'trot' if self.moving else 'idle'
        msg.linear_x = self.vx
        msg.linear_y = 0.0
        msg.angular_z = self.wz
        msg.platform_state = 'EXECUTING' if self.moving else ('READY' if self.controllable else 'DISABLED')
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMotionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
