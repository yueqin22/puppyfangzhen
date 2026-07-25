#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from nav2_msgs.msg import Costmap
import sys

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.costmap = None
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub = self.create_subscription(
            Costmap, '/global_costmap/costmap_raw',
            self.cb, qos)
        self.timer = self.create_timer(1.0, self.check)
        self.count = 0

    def cb(self, msg):
        self.costmap = msg

    def check(self):
        self.count += 1
        if self.costmap is None:
            print('Waiting for costmap...')
            if self.count > 5:
                print('No costmap received!')
                sys.exit(1)
            return
        m = self.costmap
        res = m.metadata.resolution
        ox = m.metadata.origin.position.x
        oy = m.metadata.origin.position.y
        sx = m.metadata.size_x
        sy = m.metadata.size_y
        print(f'Costmap: {sx}x{sy}, res={res}, origin=({ox},{oy})')
        print(f'Update time: {m.metadata.update_time.sec}.{m.metadata.update_time.nanosec}')

        positions = [
            ('robot(0,1.5)', 0.0, 1.5),
            ('target(-2.5,1.5)', -2.5, 1.5),
            ('doorway(0,0)', 0.0, 0.0),
            ('living(2,-2)', 2.0, -2.0),
            ('kitchen(3,2)', 3.0, 2.0),
            ('bedroom(-3.5,2)', -3.5, 2.0),
        ]
        for name, wx, wy in positions:
            mx = int((wx - ox) / res)
            my = int((wy - oy) / res)
            if 0 <= mx < sx and 0 <= my < sy:
                idx = my * sx + mx
                cost = m.data[idx]
                label = 'FREE' if cost == 0 else ('OCCUPIED' if cost >= 254 else f'INFLATED({cost})')
                print(f'  {name}: cost={cost} [{label}]')
            else:
                print(f'  {name}: OUT OF BOUNDS')
        sys.exit(0)

def main():
    rclpy.init()
    node = CostmapChecker()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
