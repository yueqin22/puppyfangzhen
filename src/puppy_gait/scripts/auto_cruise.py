#!/usr/bin/env python3
"""
Puppy 机械狗自动巡航脚本
在家庭环境中自动巡航，沿预设路径行走并建图

路径设计（家庭环境 home.world）：
  起点在客厅中央 (0, -2)
  巡航路线：客厅 -> 走廊 -> 卧室 -> 走廊 -> 厨房 -> 客厅
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoCruise(Node):
    def __init__(self):
        super().__init__('auto_cruise')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 巡航状态
        self.state = 0
        self.state_start_time = self.get_clock().now()

        # 控制周期 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('=== 自动巡航启动 ===')
        self.get_logger().info('巡航路线: 客厅 -> 走廊 -> 卧室 -> 走廊 -> 厨房 -> 客厅')

        # 巡航步骤定义
        # 每步: (duration_sec, linear_x, angular_z, description)
        self.steps = [
            # 客厅：原地旋转扫描
            (3.0, 0.0, 0.3, '客厅: 旋转扫描环境'),
            # 客厅：向前走到走廊入口
            (4.0, 0.2, 0.0, '客厅: 向前走向走廊'),
            # 走廊：左转向北
            (3.0, 0.0, 0.4, '走廊入口: 左转'),
            # 走廊：向北走到卧室门口
            (5.0, 0.2, 0.0, '走廊: 向北走'),
            # 卧室：进入卧室
            (3.0, 0.15, 0.0, '卧室: 进入'),
            # 卧室：旋转扫描
            (4.0, 0.0, 0.3, '卧室: 旋转扫描'),
            # 卧室：退出
            (3.0, -0.15, 0.0, '卧室: 后退退出'),
            # 走廊：继续向北到厨房
            (4.0, 0.2, 0.0, '走廊: 继续向北'),
            # 厨房：进入厨房
            (3.0, 0.15, 0.0, '厨房: 进入'),
            # 厨房：旋转扫描
            (4.0, 0.0, -0.3, '厨房: 旋转扫描'),
            # 厨房：退出
            (3.0, -0.15, 0.0, '厨房: 后退退出'),
            # 走廊：向南返回
            (8.0, 0.2, 0.0, '走廊: 向南返回'),
            # 客厅：右转
            (3.0, 0.0, -0.4, '客厅入口: 右转'),
            # 客厅：回到起点
            (4.0, 0.2, 0.0, '客厅: 回到起点'),
            # 客厅：最终旋转扫描
            (5.0, 0.0, 0.3, '客厅: 最终扫描'),
        ]

        self.current_step = 0
        self.get_logger().info(f'步骤 0/{len(self.steps)}: {self.steps[0][3]}')

    def timer_callback(self):
        if self.current_step >= len(self.steps):
            # 巡航完成，停止
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.get_logger().info('=== 自动巡航完成！地图已建好 ===')
            self.timer.cancel()
            return

        duration, linear_x, angular_z, desc = self.steps[self.current_step]
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

        if elapsed >= duration:
            # 进入下一步
            self.current_step += 1
            self.state_start_time = self.get_clock().now()
            if self.current_step < len(self.steps):
                self.get_logger().info(
                    f'步骤 {self.current_step}/{len(self.steps)}: '
                    f'{self.steps[self.current_step][3]}')
            # 停顿 0.5 秒
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # 发布速度命令
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = AutoCruise()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            twist = Twist()
            node.cmd_pub.publish(twist)
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
