#!/usr/bin/env python3
"""检查map_server发布的地图 vs PGM文件"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapChecker(Node):
    def __init__(self):
        super().__init__('map_checker')
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos)
        self.received = False
        self.get_logger().info('等待地图...')

    def map_cb(self, msg):
        if self.received:
            return
        self.received = True
        info = msg.info
        data = np.array(msg.data).reshape(info.height, info.width)
        print(f'地图: {info.width}x{info.height}, 分辨率={info.resolution}, '
              f'原点=({info.origin.position.x:.4f},{info.origin.position.y:.4f})')

        # 检查问题位置
        ox, oy = info.origin.position.x, info.origin.position.y
        res = info.resolution
        positions = [
            (2.0, -2.0, '航点1'),
            (3.0, 2.0, '航点6'),
            (1.0, -2.0, '机器人初始'),
        ]
        for wx, wy, name in positions:
            px = int((wx - ox) / res)
            py = int((wy - oy) / res)
            val = data[py, px]
            # OccupancyGrid: 0=FREE, 100=OCCUPIED, -1=UNKNOWN
            status = 'FREE' if val == 0 else ('OCCUPIED' if val == 100 else 'UNKNOWN')
            print(f'{name} ({wx:.1f},{wy:.1f}) -> 像素({px},{py}) 值={val} [{status}]')

        # 打印(2.0,-2.0)周围30x30
        px, py = int((2.0 - ox) / res), int((-2.0 - oy) / res)
        print(f'\n--- 航点1 (2.0,-2.0) 周围30x30 (像素{px},{py}) ---')
        for y in range(py+15, py-16, -1):
            if y < 0 or y >= info.height:
                continue
            line = ''
            for x in range(px-15, px+16):
                if x < 0 or x >= info.width:
                    line += '?'
                    continue
                v = data[y, x]
                if x == px and y == py:
                    line += 'R'
                elif v == 0:
                    line += '.'
                elif v == 100:
                    line += '#'
                elif v == -1:
                    line += '?'
                else:
                    line += '~'
            print(f'  y={y:3d}: {line}')

        # 打印(3.0,2.0)周围30x30
        px, py = int((3.0 - ox) / res), int((2.0 - oy) / res)
        print(f'\n--- 航点6 (3.0,2.0) 周围30x30 (像素{px},{py}) ---')
        for y in range(py+15, py-16, -1):
            if y < 0 or y >= info.height:
                continue
            line = ''
            for x in range(px-15, px+16):
                if x < 0 or x >= info.width:
                    line += '?'
                    continue
                v = data[y, x]
                if x == px and y == py:
                    line += 'R'
                elif v == 0:
                    line += '.'
                elif v == 100:
                    line += '#'
                elif v == -1:
                    line += '?'
                else:
                    line += '~'
            print(f'  y={y:3d}: {line}')

        rclpy.shutdown()

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = MapChecker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
