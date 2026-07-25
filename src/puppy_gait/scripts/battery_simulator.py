#!/usr/bin/env python3
"""
Puppy 机械狗 - 电池仿真节点

功能：
  1. 模拟电池电量消耗（移动时消耗更快，空闲时慢消耗）
  2. 检测机器人是否在充电桩附近，自动充电
  3. 发布 /battery_state 话题（sensor_msgs/BatteryState）
  4. 低电量时发布警报

充电桩位置：(0.0, -2.0)，与 emergency_response.py / patrol.py 保持一致
仿真速率说明：
  所有速率均为仿真秒级百分比，适合测试：
  - 空闲消耗: 0.1%/秒（约17分钟耗尽）
  - 移动消耗: 0.3%/秒（约5.5分钟耗尽）
  - 充电速率: 1.0%/秒（约100秒充满到100%）
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

from puppy_interfaces.msg import BatteryStatus
from shared_targets import get_named_target


class BatterySimulator(Node):
    """电池仿真节点"""

    def __init__(self):
        super().__init__('battery_simulator')
        self.declare_parameter('publish_semantic_battery', True)
        self.publish_semantic_battery = bool(
            self.get_parameter('publish_semantic_battery').value
        )

        # ===== 仿真电池参数（直接使用每秒百分比，方便测试）=====
        self.drain_rate_idle = 0.001      # 空闲: 0.1%/秒 (100%/0.001 ≈ 1000秒 ≈ 17分钟)
        self.drain_rate_active = 0.003    # 移动: 0.3%/秒 (100%/0.003 ≈ 333秒 ≈ 5.5分钟)
        self.charge_rate = 0.01           # 充电: 1.0%/秒 (从0到100%约100秒)
        self.voltage_full = 12.6
        self.voltage_empty = 10.5
        self.low_battery_threshold = 0.2  # 低电量阈值 20%

        # ===== 充电桩位置 =====
        dock_target = get_named_target('dock', {'x': 0.0, 'y': -2.0, 'yaw': 0.0})
        self.dock_x = float(dock_target['x'])
        self.dock_y = float(dock_target['y'])
        self.dock_range = 1.0

        # ===== 状态 =====
        self.battery_level = 1.0
        self.is_charging = False
        self.is_moving = False
        self.robot_x = 1.0
        self.robot_y = -2.0
        self.last_position = None
        self.last_time = None
        self.low_battery_alerted = False
        self.last_logged_percent = -1

        # ===== 发布器 =====
        # P0-1: 发布语义消息 puppy_interfaces/BatteryStatus 到 /battery_status
        # 供 puppy_core（mode_manager/safety_manager/mission_manager）订阅
        # 同时保留 sensor_msgs/BatteryState 到 /battery_state 作为原始兼容 topic
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.battery_semantic_pub = self.create_publisher(BatteryStatus, '/battery_status', 10)
        self.low_battery_pub = self.create_publisher(Bool, '/low_battery_alert', 10)

        # ===== 订阅器 =====
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # ===== 定时器（5Hz 更新电池状态）=====
        self.timer = self.create_timer(0.2, self.update_battery)
        self.last_time = self.get_clock().now()

        self.get_logger().info('=' * 50)
        self.get_logger().info('  电池仿真节点启动（仿真时间模式）')
        self.get_logger().info(f'  空闲耗电: {self.drain_rate_idle*100:.2f}%/秒')
        self.get_logger().info(f'  移动耗电: {self.drain_rate_active*100:.2f}%/秒')
        self.get_logger().info(f'  充电速率: {self.charge_rate*100:.2f}%/秒')
        self.get_logger().info(f'  充电桩位置: ({self.dock_x}, {self.dock_y})')
        self.get_logger().info(f'  低电量阈值: {self.low_battery_threshold * 100}%')
        self.get_logger().info('=' * 50)

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        if self.last_position is not None:
            dx = self.robot_x - self.last_position[0]
            dy = self.robot_y - self.last_position[1]
            distance = math.sqrt(dx * dx + dy * dy)
            self.is_moving = distance > 0.005
        self.last_position = (self.robot_x, self.robot_y)

    def update_battery(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            return

        dist_to_dock = math.sqrt(
            (self.robot_x - self.dock_x) ** 2 +
            (self.robot_y - self.dock_y) ** 2
        )

        if dist_to_dock < self.dock_range:
            self.is_charging = True
            self.battery_level += self.charge_rate * dt
            self.battery_level = min(1.0, self.battery_level)
        else:
            self.is_charging = False
            drain = self.drain_rate_active if self.is_moving else self.drain_rate_idle
            self.battery_level -= drain * dt
            self.battery_level = max(0.0, self.battery_level)

        self.publish_battery_state()

        if self.battery_level <= self.low_battery_threshold:
            if not self.low_battery_alerted:
                self.get_logger().warn(
                    f'低电量警报！当前电量: {self.battery_level * 100:.1f}%')
                self.low_battery_pub.publish(Bool(data=True))
                self.low_battery_alerted = True
        else:
            if self.low_battery_alerted and self.battery_level > 0.3:
                self.get_logger().info(
                    f'电量恢复正常: {self.battery_level * 100:.1f}%')
                self.low_battery_pub.publish(Bool(data=False))
                self.low_battery_alerted = False

        current_percent = int(self.battery_level * 100) // 10 * 10
        if self.is_charging and self.battery_level < 1.0:
            if current_percent != self.last_logged_percent:
                self.last_logged_percent = current_percent
                self.get_logger().info(
                    f'充电中... 电量: {self.battery_level * 100:.1f}%')
        elif not self.is_charging and self.battery_level >= 1.0:
            self.last_logged_percent = 100

    def publish_battery_state(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = self.voltage_empty + (
            self.voltage_full - self.voltage_empty) * self.battery_level
        if self.is_charging:
            msg.current = 1.0
        else:
            msg.current = -(1.0 if self.is_moving else 0.3)
        msg.percentage = self.battery_level
        if self.battery_level >= 1.0 and self.is_charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif self.is_charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.battery_pub.publish(msg)

        # P0-1: 发布语义消息 BatteryStatus 到 /battery_status
        # 供 puppy_core 订阅（mode_manager/safety_manager/mission_manager）
        sem = BatteryStatus()
        sem.header = msg.header
        sem.voltage = msg.voltage
        sem.current = msg.current
        sem.percent = self.battery_level
        sem.charging = self.is_charging
        sem.low_battery = self.battery_level <= self.low_battery_threshold
        sem.critical_battery = self.battery_level <= 0.10
        if self.publish_semantic_battery:
            self.battery_semantic_pub.publish(sem)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
