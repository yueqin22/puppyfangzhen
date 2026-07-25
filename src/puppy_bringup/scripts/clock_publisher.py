#!/usr/bin/env python3
"""
时钟发布节点 - 从 /odom 时间戳发布 /clock
关键：simROS2 插件不支持 rosgraph_msgs/msg/Clock，需要用 Python 发布
use_sim_time=false（本节点用墙钟运行），从 /odom 获取仿真时间
注意：simROS2 使用 best_effort QoS，需要匹配
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        # use_sim_time=false，本节点用墙钟运行
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        # simROS2 使用 best_effort QoS，需要匹配
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        self.last_stamp = None
        self.get_logger().info('时钟发布节点已启动（从 /odom 发布 /clock，best_effort QoS）')

    def odom_callback(self, msg):
        # 从 /odom 获取仿真时间戳，发布 /clock
        self.last_stamp = msg.header.stamp
        clock_msg = Clock()
        clock_msg.clock = self.last_stamp
        self.clock_pub.publish(clock_msg)


def main():
    # use_sim_time=false，否则没有 /clock 时节点无法运行
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
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
