#!/usr/bin/env python3
"""发送 Nav2 导航目标"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import sys

def main():
    rclpy.init()
    node = Node('send_nav_goal')

    pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

    # 等待发布者就绪
    time.sleep(1.0)

    # 目标位置: (3.0, 0.0, 0.0) - 向前移动
    target_x = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    target_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    target_yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = target_x
    msg.pose.position.y = target_y
    msg.pose.position.z = 0.0
    # Yaw to quaternion
    import math
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(target_yaw / 2)
    msg.pose.orientation.w = math.cos(target_yaw / 2)

    # 发布几次确保收到
    for i in range(3):
        pub.publish(msg)
        time.sleep(0.3)

    print(f"已发送导航目标: x={target_x}, y={target_y}, yaw={target_yaw}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
