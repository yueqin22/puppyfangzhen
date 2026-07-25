#!/usr/bin/env python3
"""
Puppy 机械狗 - 里程计 TF 发布器
订阅 /odom 话题，发布 odom -> base_footprint 变换
（CoppeliaSim 的 simROS2 不支持 tf2_msgs/msg/TFMessage，所以用此节点转发）
注意：simROS2 使用 best_effort QoS，需要匹配
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        # simROS2 使用 best_effort QoS，需要匹配
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        self.get_logger().info('里程计 TF 发布器已启动，订阅 /odom (best_effort QoS)')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # odom
        t.child_frame_id = msg.child_frame_id    # base_footprint
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
