#!/usr/bin/env python3
"""设置 AMCL 初始位姿（使用仿真时间）"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
import time

def main():
    # 使用仿真时间，与 AMCL 保持一致
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('set_initial_pose')

    pub = node.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', 10)

    # 等待发布者和 /clock 就绪
    time.sleep(2.0)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = -2.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    # 发布几次确保 AMCL 收到
    for i in range(5):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.3)

    print(f"已设置初始位姿: x=1.0, y=-2.0, theta=0.0")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
