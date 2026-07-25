#!/usr/bin/env python3
"""Check laser scan data"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class LaserChecker(Node):
    def __init__(self):
        super().__init__('laser_checker')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.get_logger().info('Waiting for laser scan...')

    def cb(self, msg):
        print(f"\nLaser scan:")
        print(f"  frame: {msg.header.frame_id}")
        print(f"  angle_min: {math.degrees(msg.angle_min):.1f}°")
        print(f"  angle_max: {math.degrees(msg.angle_max):.1f}°")
        print(f"  angle_inc: {math.degrees(msg.angle_increment):.3f}°")
        print(f"  range_min: {msg.range_min}m")
        print(f"  range_max: {msg.range_max}m")
        print(f"  num_readings: {len(msg.ranges)}")

        # Find closest obstacles in each direction
        n = len(msg.ranges)
        directions = [
            (0, "front (0°)"),
            (n//4, "left (90°)"),
            (n//2, "back (180°)"),
            (3*n//4, "right (270°)"),
        ]

        print(f"\nObstacles by direction:")
        for idx, name in directions:
            # Check a window of 10 readings around this direction
            min_range = float('inf')
            for i in range(max(0, idx-5), min(n, idx+5)):
                r = msg.ranges[i]
                if msg.range_min < r < msg.range_max:
                    min_range = min(min_range, r)
            if min_range == float('inf'):
                print(f"  {name}: no obstacle detected")
            else:
                print(f"  {name}: {min_range:.2f}m")

        # Check for very close obstacles (< 0.5m)
        close_count = 0
        for r in msg.ranges:
            if msg.range_min < r < 0.5:
                close_count += 1
        print(f"\nReadings < 0.5m: {close_count}/{n}")

        # Print all readings < 1m
        print(f"\nReadings < 1m:")
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < 1.0:
                angle = math.degrees(msg.angle_min + i * msg.angle_increment)
                print(f"  [{i:3d}] {angle:6.1f}°: {r:.3f}m")

rclpy.init()
node = LaserChecker()
start = time.time()
while rclpy.ok() and time.time() - start < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
    if hasattr(node, '_got_msg'):
        break
node.destroy_node()
rclpy.shutdown()
