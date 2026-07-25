#!/usr/bin/env python3
"""Send a navigation goal with proper sim_time timestamp."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import sys
import math

def main():
    # Initialize with use_sim_time parameter
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('goal_sender')

    # Get goal coordinates from command line or use defaults
    goal_x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    goal_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    goal_yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    # Convert yaw to quaternion
    q_z = math.sin(goal_yaw / 2.0)
    q_w = math.cos(goal_yaw / 2.0)

    node.get_logger().info(f'Sending goal to ({goal_x}, {goal_y}, yaw={goal_yaw})...')

    # Create action client
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Wait for server
    node.get_logger().info('Waiting for action server...')
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error('Action server not available!')
        return

    # Create goal with proper sim_time timestamp
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = goal_x
    goal_msg.pose.pose.position.y = goal_y
    goal_msg.pose.pose.position.z = 0.0
    goal_msg.pose.pose.orientation.x = 0.0
    goal_msg.pose.pose.orientation.y = 0.0
    goal_msg.pose.pose.orientation.z = q_z
    goal_msg.pose.pose.orientation.w = q_w
    goal_msg.behavior_tree = ''

    node.get_logger().info(f'Goal timestamp: {goal_msg.pose.header.stamp.sec}.{goal_msg.pose.header.stamp.nanosec}')

    # Send goal
    send_goal_future = client.send_goal_async(goal_msg)

    # Spin until goal is accepted
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)

    if send_goal_future.result() is None:
        node.get_logger().error('Failed to send goal!')
        return

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected!')
        return

    node.get_logger().info('Goal accepted! Waiting for result...')

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)

    if result_future.result() is None:
        node.get_logger().error('Failed to get result!')
    else:
        result = result_future.result()
        node.get_logger().info(f'Goal finished with status: {result.status}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
