#!/usr/bin/env python3
"""
Automatic patrol script for Puppy robot.
Navigates to a series of waypoints in a loop.
Usage: python3 patrol.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time
import sys


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Patrol waypoints (x, y, yaw) - covering the 5x5 room
        # Map covers from -3.0 to 3.0, room walls at +-2.5
        self.waypoints = [
            (1.5, 0.0, 0.0),       # East center
            (1.5, 1.5, math.pi/2),  # Northeast corner
            (0.0, 1.5, math.pi),    # North center
            (-1.5, 1.5, -math.pi/2), # Northwest corner
            (-1.5, 0.0, math.pi),   # West center
            (-1.5, -1.5, -math.pi/2), # Southwest corner
            (0.0, -1.5, 0.0),       # South center
            (1.5, -1.5, math.pi/2),  # Southeast corner
        ]

        self.current_waypoint = 0
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Patrol node started. Waiting for action server...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Action server not available after 30s!')
            return

        self.get_logger().info(f'Action server ready! {len(self.waypoints)} waypoints defined.')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        """Send the next waypoint as a navigation goal."""
        idx = self.current_waypoint % len(self.waypoints)
        x, y, yaw = self.waypoints[idx]

        self.get_logger().info(f'--- Waypoint {idx+1}/{len(self.waypoints)}: ({x:.1f}, {y:.1f}, yaw={yaw:.2f}) ---')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.behavior_tree = ''

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Log navigation feedback."""
        pass  # Keep quiet to avoid spam

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Waypoint {self.current_waypoint + 1} rejected! Trying next...')
            self.current_waypoint += 1
            time.sleep(2.0)
            self.send_next_waypoint()
            return

        self.get_logger().info(f'Goal accepted (waypoint {self.current_waypoint + 1})')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status if result else 'unknown'
        self.get_logger().info(f'Waypoint {self.current_waypoint + 1} finished with status: {status}')

        # Move to next waypoint
        self.current_waypoint += 1

        # Small pause between waypoints
        time.sleep(1.0)
        self.send_next_waypoint()


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = PatrolNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Patrol stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
