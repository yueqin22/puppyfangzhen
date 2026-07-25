"""Capability Registry Node: ROS2 node wrapper for CapabilityRegistry."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from puppy_core.capability_registry import CapabilityRegistry
from puppy_interfaces.msg import RobotHealth


class CapabilityRegistryNode(Node):
    """ROS2 node that exposes CapabilityRegistry via topic + service."""

    CAP_NAMES = (
        'navigation',
        'patrol',
        'security',
        'emotion',
        'fall_detection',
        'docking',
        'rgb_camera',
        'depth_camera',
        'lidar',
        'imu',
    )

    def __init__(self):
        super().__init__('capability_registry')

        self.registry = CapabilityRegistry()

        for cap in self.CAP_NAMES:
            self.declare_parameter(f'enable_{cap}', True)
            if not self.get_parameter(f'enable_{cap}').value:
                self.registry.set_enabled(cap, False, 'disabled by config')

        self.create_subscription(RobotHealth, '/robot/health', self._on_health, 10)

        self.cap_pub = self.create_publisher(String, '/robot/capabilities', 10)
        self.create_timer(1.0, self._publish_capabilities)

        self.create_service(Trigger, '/robot/capability_query', self._on_query)

        self.get_logger().info(
            f'Capability registry started (available: {sorted(self.registry.get_available())})'
        )

    def _on_health(self, msg: RobotHealth):
        if not msg.lidar_ready:
            self.registry.disable('lidar', 'LIDAR offline or timeout')
            self.registry.disable('patrol', 'patrol requires LIDAR')
        else:
            self.registry.enable('lidar')
            if self.registry.is_available('navigation'):
                self.registry.enable('patrol')

        if not msg.imu_ready:
            self.registry.disable('imu', 'IMU offline or timeout')
            self.registry.disable('navigation', 'navigation requires IMU')
        else:
            self.registry.enable('imu')
            self.registry.enable('navigation')

        if not msg.camera_ready:
            self.registry.disable('rgb_camera', 'camera offline')
            self.registry.disable('depth_camera', 'depth camera offline')
        else:
            self.registry.enable('rgb_camera')
            self.registry.enable('depth_camera')

        if not msg.motion_ready:
            self.registry.disable('docking', 'motion not controllable')
        else:
            self.registry.enable('docking')

    def _publish_capabilities(self):
        msg = String()
        msg.data = json.dumps(self.registry.get_status())
        self.cap_pub.publish(msg)

    def _on_query(self, request, response):
        response.success = True
        response.message = json.dumps(self.registry.get_status())
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CapabilityRegistryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
