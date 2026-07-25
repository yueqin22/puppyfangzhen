#!/usr/bin/env python3
"""Manual /clock publisher for Gazebo simulation.

Reads timestamps from /odom topic (published by Gazebo with sim time)
and republishes them to /clock so nodes with use_sim_time=true can
synchronize properly.

This is needed because gazebo_ros in this environment doesn't publish
/clock automatically.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        self.clock_pub = self.create_publisher(
            Clock, '/clock', qos_profile)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.last_time = None
        self.get_logger().info('Clock publisher started - reading from /odom')
    
    def odom_callback(self, msg):
        """Extract sim time from odometry message and publish to /clock."""
        clock_msg = Clock()
        clock_msg.clock = msg.header.stamp
        self.clock_pub.publish(clock_msg)
        self.last_time = msg.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
