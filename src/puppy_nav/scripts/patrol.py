#!/usr/bin/env python3
"""
Puppy 机械狗 - 巡逻触发器（thin client）

P1-1: 已退化为 mission_manager 的触发入口。
不再直接发 Nav2 goal，改为发布 /mission/command 触发 mission_manager 执行巡逻。
订阅 /mission/status 显示巡逻进度。

RUNTIME: demo (productization path uses mission_manager as authoritative orchestrator)
权威编排层: puppy_core/mission_manager.py

用法:
  ros2 run puppy_nav patrol.py            # 触发一次巡逻
  ros2 run puppy_nav patrol.py -- --stop  # 停止巡逻
  ros2 launch puppy_bringup full_system.launch.py patrol:=true
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from puppy_interfaces.msg import PatrolStatus


class PatrolTriggerNode(Node):
    """Thin client: triggers patrol via mission_manager and monitors status."""

    def __init__(self):
        super().__init__('patrol_trigger')
        self.command_pub = self.create_publisher(String, '/mission/command', 10)
        self.create_subscription(PatrolStatus, '/mission/status', self._on_status, 10)

        self._triggered = False
        # 等待 publisher 建立连接后发送命令
        self.create_timer(1.5, self._send_command)

    def _send_command(self):
        if self._triggered:
            return
        # 支持 --stop 参数停止巡逻
        cmd = 'stop' if '--stop' in sys.argv else 'patrol'
        msg = String()
        msg.data = cmd
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent mission command: {cmd}')
        self._triggered = True
        if cmd == 'stop':
            # 停止命令发送后即可退出
            self.create_timer(2.0, self._shutdown)

    def _on_status(self, msg: PatrolStatus):
        self.get_logger().info(
            f'[mission] state={msg.state} progress={msg.completion_ratio:.0%} '
            f'wp={msg.current_waypoint_index}/{msg.total_waypoints} {msg.message}')

    def _shutdown(self):
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
