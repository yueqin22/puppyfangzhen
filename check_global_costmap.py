#!/usr/bin/env python3
"""
检查全局代价地图 - 使用 costmap_raw (nav2_msgs/Costmap)
诊断全局规划器失败问题
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav2_msgs.msg import Costmap
import tf2_ros
import math


class GlobalCostmapChecker(Node):
    def __init__(self):
        super().__init__('global_costmap_checker')

        self.costmap = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 使用 TRANSIENT_LOCAL QoS 订阅 costmap_raw
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Costmap, '/global_costmap/costmap_raw',
            self.costmap_cb, qos)

        self.waypoints = [
            (1.0, -2.0, '机器人初始位置'),
            (2.0, -2.0, '航点1: 客厅中心'),
            (0.0, -1.0, '航点2: 门口南侧'),
            (0.0, 1.5, '航点3: 走廊'),
            (-2.5, 1.5, '航点4: 卧室'),
            (3.0, 2.0, '航点6: 厨房'),
            (-0.5, -2.0, '航点8: 充电桩'),
        ]

        self.timer = self.create_timer(2.0, self.check)
        self.get_logger().info('全局代价地图检查器启动，等待代价地图...')

    def costmap_cb(self, msg):
        self.costmap = msg
        self.get_logger().info(
            f'收到全局代价地图: {msg.metadata.size_x}x{msg.metadata.size_y}, '
            f'分辨率={msg.metadata.resolution}, '
            f'原点=({msg.metadata.origin.position.x:.2f},'
            f'{msg.metadata.origin.position.y:.2f})')

    def world_to_map(self, wx, wy):
        if self.costmap is None:
            return None
        meta = self.costmap.metadata
        mx = int((wx - meta.origin.position.x) / meta.resolution)
        my = int((wy - meta.origin.position.y) / meta.resolution)
        if 0 <= mx < meta.size_x and 0 <= my < meta.size_y:
            return (mx, my)
        return None

    def get_cost(self, mx, my):
        if self.costmap is None:
            return -1
        meta = self.costmap.metadata
        idx = my * meta.size_x + mx
        if 0 <= idx < len(self.costmap.data):
            return self.costmap.data[idx]
        return -1

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f'获取机器人位置失败: {e}')
            return None

    def check(self):
        if self.costmap is None:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose:
            x, y, yaw = robot_pose
            cell = self.world_to_map(x, y)
            if cell:
                cost = self.get_cost(*cell)
                status = 'FREE' if cost == 0 else (
                    'INFLATED' if 0 < cost < 254 else 'LETHAL')
                self.get_logger().info(
                    f'机器人: ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°) '
                    f'-> 像素({cell[0]},{cell[1]}) 代价={cost} [{status}]')

        self.get_logger().info('--- 航点代价检查 ---')
        for wx, wy, name in self.waypoints:
            cell = self.world_to_map(wx, wy)
            if cell:
                cost = self.get_cost(*cell)
                status = 'FREE' if cost == 0 else (
                    'INFLATED' if 0 < cost < 254 else 'LETHAL')
                marker = 'OK' if cost == 0 else ('~' if cost < 254 else 'XX')
                self.get_logger().info(
                    f'  [{marker}] {name}: ({wx:.1f},{wy:.1f}) -> '
                    f'像素({cell[0]},{cell[1]}) 代价={cost} [{status}]')

        # 打印门口区域
        door_cell = self.world_to_map(0.0, 0.0)
        if door_cell:
            self.print_area(door_cell[0], door_cell[1], 20, '门口区域(0,0)')

        # 打印机器人周围
        if robot_pose:
            x, y, _ = robot_pose
            center = self.world_to_map(x, y)
            if center:
                self.print_area(center[0], center[1], 12, '机器人周围')

        print('\n' + '=' * 60 + '\n')

    def print_area(self, cx, cy, radius, title):
        if self.costmap is None:
            return
        meta = self.costmap.metadata
        print(f'\n--- {title} (中心像素: {cx},{cy}) ---')
        for my in range(cy + radius, cy - radius - 1, -1):
            if my < 0 or my >= meta.size_y:
                continue
            line = ''
            for mx in range(cx - radius, cx + radius + 1):
                if mx < 0 or mx >= meta.size_x:
                    line += '?'
                    continue
                cost = self.get_cost(mx, my)
                if mx == cx and my == cy:
                    line += 'R'
                elif cost == 0:
                    line += '.'
                elif cost < 254:
                    line += '~'
                elif cost == 254:
                    line += '#'
                else:
                    line += '?'
            print(f'  y={my:3d}: {line}')


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = GlobalCostmapChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
