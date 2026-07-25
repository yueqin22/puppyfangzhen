#!/usr/bin/env python3
"""
紧急联动响应节点
- 跌倒检测 → 机器人导航到跌倒位置 + 语音紧急呼叫
- 煤气泄漏 → 机器人导航到厨房确认 + 语音警报
- 入侵检测 → 机器人导航到入侵位置 + 语音警告
- 低电量 → 自动返回充电桩

联动策略:
  1. 接收紧急事件
  2. 取消当前导航任务
  3. 导航到事件位置（失败则重试）
  4. 语音播报紧急信息
  5. 等待事件解除
"""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus


EMERGENCY_LOCATIONS = {
    'fall':      {'x': 1.0, 'y': -2.0, 'yaw': 0.0, 'name': '客厅'},
    'gas_leak':  {'x': 3.0, 'y': 2.0, 'yaw': 0.0, 'name': '厨房'},
    'intrusion': {'x': 0.0, 'y': -1.0, 'yaw': math.pi / 2, 'name': '门口'},
    'charging':  {'x': 0.0, 'y': -2.0, 'yaw': math.pi, 'name': '充电桩'},
}


class EmergencyResponse(Node):
    def __init__(self):
        super().__init__('emergency_response')

        self.cb_group = ReentrantCallbackGroup()

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.cb_group)

        self.current_emergency = None
        self.emergency_active = False
        self.last_fall_state = False
        self.last_intrusion = False
        self.last_low_battery = False
        self.cooldown_timer = None
        self.retry_timer = None
        self.goal_handle = None
        self.pending_location = None
        self.retry_count = 0
        self.max_retries = 5

        self.create_subscription(Bool, '/fall_detected', self.fall_callback, 10)
        self.create_subscription(String, '/security/alert', self.security_callback, 10)
        self.create_subscription(Bool, '/security/intrusion', self.intrusion_callback, 10)
        self.create_subscription(Bool, '/low_battery_alert', self.battery_callback, 10)
        self.create_subscription(String, '/emergency/cancel', self.cancel_callback, 10)

        self.voice_pub = self.create_publisher(String, '/voice/tts', 10)
        self.status_pub = self.create_publisher(String, '/emergency/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(5.0, self.publish_status)

        self.get_logger().info('紧急联动响应节点已启动')
        self.get_logger().info('监听: /fall_detected, /security/alert, /security/intrusion, /low_battery_alert')

    def fall_callback(self, msg):
        if msg.data and not self.last_fall_state:
            self.get_logger().warn('!!! 紧急事件: 检测到跌倒 !!!')
            self.trigger_emergency('fall', '检测到主人跌倒！正在前往现场确认')
        self.last_fall_state = msg.data

    def security_callback(self, msg):
        alert_text = msg.data
        if '高级警报' in alert_text and '煤气' in alert_text:
            if not self.emergency_active:
                self.trigger_emergency('gas_leak', f'煤气泄漏警报！{alert_text}，正在前往厨房确认')
        elif '高级警报' in alert_text and ('入侵' in alert_text or '闯入' in alert_text):
            if not self.emergency_active:
                self.trigger_emergency('intrusion', f'入侵警报！{alert_text}，正在前往门口查看')

    def intrusion_callback(self, msg):
        if msg.data and not self.last_intrusion:
            if not self.emergency_active:
                self.trigger_emergency('intrusion', '检测到入侵！正在前往门口查看')
        self.last_intrusion = msg.data

    def battery_callback(self, msg):
        if msg.data and not self.last_low_battery:
            if not self.emergency_active:
                self.trigger_emergency('charging', '电量不足，正在自动返回充电桩')
        self.last_low_battery = msg.data

    def trigger_emergency(self, event_type, voice_msg):
        if self.emergency_active:
            self.get_logger().warn(f'已有紧急事件进行中，忽略新事件: {event_type}')
            return

        self.emergency_active = True
        self.current_emergency = event_type
        self.retry_count = 0
        location = EMERGENCY_LOCATIONS.get(event_type, EMERGENCY_LOCATIONS['fall'])
        self.pending_location = location

        self.get_logger().warn(f'===== 紧急响应启动: {event_type} → {location["name"]} =====')

        self.voice_pub.publish(String(data=voice_msg))

        self.navigate_to(location['x'], location['y'], location['yaw'])

    def navigate_to(self, x, y, yaw):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 服务器不可用，3秒后重试...')
            self.schedule_retry()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(
            f'发送导航目标: ({x:.1f}, {y:.1f}, yaw={yaw:.2f}) '
            f'[尝试 {self.retry_count + 1}/{self.max_retries}]')
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def schedule_retry(self):
        if self.retry_count >= self.max_retries:
            self.get_logger().error(f'导航重试{self.max_retries}次均失败，放弃紧急导航')
            self.emergency_active = False
            self.current_emergency = None
            self.pending_location = None
            return
        self.retry_count += 1
        self.get_logger().info(f'3秒后重试导航 (第{self.retry_count}次)...')
        self.retry_timer = self.create_timer(3.0, self._do_retry)

    def _do_retry(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        if self.pending_location is None or not self.emergency_active:
            return
        loc = self.pending_location
        self.navigate_to(loc['x'], loc['y'], loc['yaw'])

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('紧急导航目标被拒绝，可能Nav2未就绪')
            self.schedule_retry()
            return

        self.get_logger().info('紧急导航目标已接受')
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'已到达紧急事件位置: {self.current_emergency}')
            self.announce_arrival()
            self.cooldown_timer = self.create_timer(10.0, self.clear_emergency)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'导航中止(状态:{status})，执行到达处理')
            self.announce_arrival()
            self.cooldown_timer = self.create_timer(10.0, self.clear_emergency)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'导航已取消: {self.current_emergency}')
            self.emergency_active = False
            self.current_emergency = None
            self.pending_location = None
        else:
            self.get_logger().warn(f'导航失败，状态: {status}')
            self.schedule_retry()

    def announce_arrival(self):
        if self.current_emergency == 'fall':
            self.voice_pub.publish(String(
                data='主人，您还好吗？我已经到达现场，正在为您呼叫帮助'))
        elif self.current_emergency == 'gas_leak':
            self.voice_pub.publish(String(
                data='已到达厨房，请立即检查煤气阀门，打开窗户通风，不要使用明火'))
        elif self.current_emergency == 'intrusion':
            self.voice_pub.publish(String(
                data='已到达门口，未发现异常情况，请主人查看监控确认'))
        elif self.current_emergency == 'charging':
            self.voice_pub.publish(String(
                data='已到达充电桩，开始充电'))

    def clear_emergency(self):
        if self.cooldown_timer:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None
        self.get_logger().info(f'紧急事件已解除: {self.current_emergency}')
        self.voice_pub.publish(String(data='紧急事件已解除，恢复正常巡逻'))
        self.emergency_active = False
        self.current_emergency = None
        self.pending_location = None

    def cancel_callback(self, msg):
        self.get_logger().info(f'手动取消紧急事件: {msg.data}')
        if self.cooldown_timer:
            self.cooldown_timer.cancel()
            self.cooldown_timer = None
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        if self.goal_handle is not None:
            self.get_logger().info('取消进行中的导航目标')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.goal_handle = None
        self.emergency_active = False
        self.current_emergency = None
        self.pending_location = None
        self.cmd_vel_pub.publish(Twist())

    def cancel_done_callback(self, future):
        try:
            result = future.result()
            if len(result.goals_canceling) > 0:
                self.get_logger().info('导航目标已成功取消')
            else:
                self.get_logger().warn('导航目标取消失败')
        except Exception as e:
            self.get_logger().error(f'取消导航目标异常: {e}')

    def publish_status(self):
        status = {
            'active': self.emergency_active,
            'event': self.current_emergency or 'none',
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = EmergencyResponse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
