#!/usr/bin/env python3
"""Check local costmap around robot position"""
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
import numpy as np
import time

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.costmap_data = None
        self.sub = self.create_subscription(
            Costmap, '/local_costmap/costmap_raw', self.cb, 10)
        self.get_logger().info('Waiting for local costmap...')

    def cb(self, msg):
        self.costmap_data = msg
        self.get_logger().info(
            f'Received costmap: {msg.metadata.size_x}x{msg.metadata.size_y}, '
            f'res={msg.metadata.resolution}, '
            f'origin=({msg.metadata.origin.position.x:.2f},{msg.metadata.origin.position.y:.2f})')

        # Convert data to numpy array
        data = np.array(msg.data, dtype=np.uint8).reshape(
            msg.metadata.size_y, msg.metadata.size_x)

        # Find robot position in costmap (center of rolling window)
        cx = msg.metadata.size_x // 2
        cy = msg.metadata.size_y // 2

        # Print 20x20 area around robot
        print(f"\nLocal costmap around robot (center {cx},{cy}):")
        print(f"Origin: ({msg.metadata.origin.position.x:.2f}, {msg.metadata.origin.position.y:.2f})")
        print(f"Legend: .=free(0) #=lethal(254) ~=inflated ~=med(1-253)")
        for y in range(cy-10, cy+11):
            row = ""
            for x in range(cx-10, cx+11):
                if 0 <= x < msg.metadata.size_x and 0 <= y < msg.metadata.size_y:
                    v = data[y][x]
                    if v == 0:
                        row += "."
                    elif v == 254:
                        row += "#"
                    elif v == 253:
                        row += "+"
                    elif v > 200:
                        row += "*"
                    elif v > 100:
                        row += "o"
                    elif v > 0:
                        row += "~"
                    else:
                        row += "."
                else:
                    row += " "
            print(f"  {row}")

        # Count obstacles
        lethal = np.sum(data == 254)
        inflated = np.sum((data > 0) & (data < 254))
        free = np.sum(data == 0)
        print(f"\nStats: free={free}, lethal={lethal}, inflated={inflated}")

        # Check robot's cell
        robot_cell = data[cy][cx]
        print(f"Robot cell value: {robot_cell} ({'LETHAL' if robot_cell==254 else 'FREE' if robot_cell==0 else 'INFLATED'})")

rclpy.init()
node = CostmapChecker()
start = time.time()
while rclpy.ok() and time.time() - start < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.costmap_data:
        break
node.destroy_node()
rclpy.shutdown()
