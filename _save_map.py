#!/usr/bin/env python3
"""直接订阅 /map 话题并保存为 pgm + yaml 文件"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import sys
import os

class MapSaver(Node):
    def __init__(self, filename):
        super().__init__('map_saver_py')
        self.filename = filename
        self.saved = False
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info(f'Waiting for map on /map topic...')

    def map_callback(self, msg):
        if self.saved:
            return
        self.saved = True
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')

        # Save PGM
        pgm_file = self.filename + '.pgm'
        with open(pgm_file, 'w') as f:
            f.write('P5\n')
            f.write(f'{msg.info.width} {msg.info.height}\n')
            f.write('255\n')
            for val in msg.data:
                if val == -1:
                    f.write(chr(205))
                elif val == 0:
                    f.write(chr(254))
                elif val == 100:
                    f.write(chr(0))
                else:
                    f.write(chr(254 - int(val * 2.54)))

        # Save YAML
        yaml_file = self.filename + '.yaml'
        with open(yaml_file, 'w') as f:
            f.write(f'image: {os.path.basename(pgm_file)}\n')
            f.write(f'resolution: {msg.info.resolution}\n')
            f.write(f'origin: [{msg.info.origin.position.x}, {msg.info.origin.position.y}, {msg.info.origin.position.z}]\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')
            f.write('negate: 0\n')

        self.get_logger().info(f'Map saved to {pgm_file} and {yaml_file}')
        raise SystemExit(0)

def main():
    rclpy.init()
    filename = sys.argv[1] if len(sys.argv) > 1 else '/tmp/map'
    node = MapSaver(filename)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
