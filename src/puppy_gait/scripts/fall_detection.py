#!/usr/bin/env python3
"""
Puppy 机械狗 - 老人跌倒检测节点（仿真版本）

功能：
  1. 订阅 /camera/depth_camera/image_raw 话题（模拟摄像头图像）
  2. 仿真人体姿态估计，检测老人是否跌倒
  3. 当检测到人体高度突然降低（跌倒特征）时发布警报
  4. 发布 /fall_detected 话题（Bool 类型，True 表示检测到跌倒）
  5. 发布 /fall_detection/status 话题（String 类型，状态信息）
  6. 提供 /fall_detection/trigger_test 服务手动触发测试跌倒

注意：这是仿真版本，默认不自动模拟跌倒事件，避免误报。
测试跌倒：ros2 service call /fall_detection/trigger_test std_srvs/srv/Trigger
测试恢复：等待机器人移动（仿真中高度会"恢复"），或重启节点
实际部署时需要接入真实的人体姿态估计模型。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from puppy_interfaces.msg import FallEvent


class FallDetection(Node):
    """老人跌倒检测节点（仿真版本）"""

    def __init__(self):
        super().__init__('fall_detection')

        self.declare_parameter('auto_simulate_falls', False)
        self.declare_parameter('sim_cycle_seconds', 60.0)
        self.auto_sim = self.get_parameter('auto_simulate_falls').value

        # P1-2: 跌倒检测阈值从参数读取（可由 features.yaml 覆盖）
        self.declare_parameter('normal_height', 1.7)
        self.declare_parameter('fallen_height', 0.3)
        self.declare_parameter('height_drop_threshold', 0.5)
        self.declare_parameter('fall_confirm_duration', 1.0)
        self.declare_parameter('alert_cooldown', 30.0)

        self.image_sub = self.create_subscription(
            Image, '/camera/depth_camera/image_raw', self.image_callback, 10)

        self.fall_pub = self.create_publisher(Bool, '/fall_detected', 10)
        # P0-1: 发布语义消息 FallEvent 到 /fall/event
        # 供 puppy_core（mode_manager/safety_manager）订阅
        # 保留 Bool /fall_detected 作为兼容 topic（security_node 订阅）
        self.fall_event_pub = self.create_publisher(FallEvent, '/fall/event', 10)
        self.status_pub = self.create_publisher(
            String, '/fall_detection/status', 10)

        self.normal_height = self.get_parameter('normal_height').value
        self.fallen_height = self.get_parameter('fallen_height').value
        self.height_drop_threshold = self.get_parameter('height_drop_threshold').value
        self.fall_confirm_duration = self.get_parameter('fall_confirm_duration').value
        self.alert_cooldown = self.get_parameter('alert_cooldown').value

        self.current_height = self.normal_height
        self.previous_height = self.normal_height
        self.fall_start_time = None
        self.last_alert_time = None
        self.is_fallen = False
        self.frame_count = 0
        self.sim_phase = 0.0

        self.test_srv = self.create_service(
            Trigger, '/fall_detection/trigger_test', self.trigger_test_callback)
        self.recover_srv = self.create_service(
            Trigger, '/fall_detection/trigger_recover', self.trigger_recover_callback)

        self.status_timer = self.create_timer(2.0, self.publish_status)

        if self.auto_sim:
            sim_cycle = self.get_parameter('sim_cycle_seconds').value
            self.sim_cycle_frames = int(sim_cycle / 0.1)
            self.sim_timer = self.create_timer(0.1, self.simulation_step)
            self.get_logger().info(
                f'  自动模拟跌倒已开启（周期 {sim_cycle}s）')
        else:
            self.get_logger().info('  自动模拟跌倒已关闭（默认模式）')
            self.get_logger().info(
                '  测试跌倒: ros2 service call /fall_detection/trigger_test std_srvs/srv/Trigger')

        self.get_logger().info('=' * 50)
        self.get_logger().info('  老人跌倒检测节点启动（仿真版本）')
        self.get_logger().info(f'  正常站立高度: {self.normal_height} m')
        self.get_logger().info(
            f'  跌倒判定高度变化阈值: {self.height_drop_threshold * 100}%')
        self.get_logger().info(
            '  发布话题: /fall_detected, /fall_detection/status')
        self.get_logger().info('=' * 50)

        self.publish_status_msg('节点启动，等待检测...')

    def simulation_step(self):
        if not self.auto_sim:
            return
        self.frame_count += 1
        self.previous_height = self.current_height
        self.sim_phase += 0.05

        cycle = self.frame_count % self.sim_cycle_frames
        fall_start = self.sim_cycle_frames // 2
        fall_end = fall_start + 50
        if fall_start <= cycle <= fall_end:
            if cycle == fall_start:
                self.get_logger().warn(
                    '仿真事件: 检测到人体高度突然降低（模拟跌倒）')
            self.current_height = (
                self.fallen_height + 0.05 * math.sin(self.sim_phase))
        else:
            self.current_height = (
                self.normal_height + 0.05 * math.sin(self.sim_phase))

        self.detect_fall()

    def trigger_test_callback(self, request, response):
        self.get_logger().warn('[测试] 手动触发跌倒事件！')
        self.previous_height = self.normal_height
        self.current_height = self.fallen_height
        self.trigger_fall_alert()
        response.success = True
        response.message = '跌倒测试事件已触发'
        return response

    def trigger_recover_callback(self, request, response):
        self.get_logger().info('[测试] 手动触发恢复事件！')
        self.current_height = self.normal_height
        self.is_fallen = False
        self.fall_start_time = None
        self.last_alert_time = self.get_clock().now()
        self.fall_pub.publish(Bool(data=False))
        self._publish_fall_event(detected=False, source='manual_recover')
        self.publish_status_msg('检测到老人已站起，跌倒状态解除')
        response.success = True
        response.message = '恢复事件已触发'
        return response

    def image_callback(self, msg):
        pass

    def detect_fall(self):
        now = self.get_clock().now()

        if self.previous_height > 0.01:
            height_change_ratio = (
                self.previous_height - self.current_height
            ) / self.previous_height
        else:
            height_change_ratio = 0.0

        if height_change_ratio > self.height_drop_threshold:
            if self.fall_start_time is None:
                self.fall_start_time = now
                self.publish_status_msg(
                    f'疑似跌倒：高度从 {self.previous_height:.2f}m '
                    f'降至 {self.current_height:.2f}m')

            elapsed = (now - self.fall_start_time).nanoseconds / 1e9
            if elapsed >= self.fall_confirm_duration and not self.is_fallen:
                self.trigger_fall_alert()
        else:
            if self.fall_start_time is not None and not self.is_fallen:
                self.publish_status_msg('高度恢复正常，取消疑似跌倒警报')
            self.fall_start_time = None

        if self.is_fallen and self.current_height > self.normal_height * 0.7:
            self.is_fallen = False
            self.fall_start_time = None
            self.last_alert_time = now
            self.publish_status_msg('检测到老人已站起，跌倒状态解除')
            self.fall_pub.publish(Bool(data=False))
            self._publish_fall_event(detected=False, source='auto_recover')

    def trigger_fall_alert(self):
        now = self.get_clock().now()

        if self.last_alert_time is not None:
            elapsed = (now - self.last_alert_time).nanoseconds / 1e9
            if elapsed < self.alert_cooldown:
                return

        self.is_fallen = True
        self.last_alert_time = now

        alert_msg = Bool()
        alert_msg.data = True
        self.fall_pub.publish(alert_msg)
        # P0-1: 同时发布语义消息 FallEvent
        confidence = min(1.0, max(0.0, (
            self.previous_height - self.current_height) / max(self.previous_height, 0.01)))
        self._publish_fall_event(
            detected=True, direction='down', confidence=confidence, source='height_drop')

        alert_text = (
            f'跌倒警报！检测到老人跌倒 '
            f'(当前高度: {self.current_height:.2f}m)')
        self.publish_status_msg(alert_text)
        self.get_logger().error(alert_text)

    def publish_status_msg(self, message):
        status = String()
        status.data = message
        self.status_pub.publish(status)

    def _publish_fall_event(self, detected, direction='', confidence=0.0, source='simulation'):
        """P0-1: 发布语义消息 FallEvent 到 /fall/event。

        供 puppy_core（mode_manager/safety_manager）订阅。与 Bool /fall_detected
        同时发布，保持向后兼容。
        """
        msg = FallEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.detected = detected
        msg.direction = direction
        msg.confidence = confidence
        msg.source = source
        self.fall_event_pub.publish(msg)

    def publish_status(self):
        state_str = '跌倒' if self.is_fallen else '正常'
        status_text = f'[状态: {state_str}] 当前高度: {self.current_height:.2f}m'
        self.publish_status_msg(status_text)


def main(args=None):
    rclpy.init(args=args)
    node = FallDetection()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.fall_pub.publish(Bool(data=False))
            node._publish_fall_event(detected=False, source='shutdown')
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
