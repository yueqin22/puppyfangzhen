#!/usr/bin/env python3
"""简单巡逻脚本 - 家庭环境版"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time
import math


class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def navigate_to(self, x, y, yaw=0.0, timeout=60):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"导航到 ({x}, {y})...")

        self.client.wait_for_server(timeout_sec=15.0)
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.result() or not future.result().accepted:
            self.get_logger().warn("目标被拒绝")
            return False

        result_future = future.result().get_result_async()
        start = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                self.get_logger().warn("超时!")
                return False

        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info("到达! (%.1fs)" % (time.time() - start))
            return True
        else:
            self.get_logger().info("状态=%d" % status)
            return False


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    bot = PatrolBot()

    # 家庭环境航点（home.world 10m×8m）
    waypoints = [
        (2.5, -2.0, 0.0),           # 客厅-沙发
        (4.0, -1.0, math.pi/2),     # 客厅-电视
        (0.0, 1.5, math.pi/2),      # 走廊
        (-3.0, 2.5, math.pi),       # 卧室-床
        (3.0, 2.5, 0.0),            # 厨房-餐桌
        (-1.0, -2.5, 0.0),          # 充电桩
    ]

    bot.get_logger().info("巡逻开始! %d 个航点" % len(waypoints))

    round_num = 1
    try:
        while rclpy.ok():
            bot.get_logger().info("--- 第 %d 轮 ---" % round_num)
            for i, (x, y, yaw) in enumerate(waypoints):
                bot.navigate_to(x, y, yaw)
                time.sleep(1)
            round_num += 1
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            bot.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
