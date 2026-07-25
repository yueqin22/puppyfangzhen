#!/usr/bin/env python3
"""
Puppy 机械狗 - 智能家庭巡逻系统（legacy 状态机）

RUNTIME: demo (legacy, superseded by mission_manager as authoritative orchestrator)
权威编排层: puppy_core/mission_manager.py (IDLE/PATROL/DOCKING/NAVIGATION)
本脚本保留独立状态机（IDLE/PATROL/RETURN_CHARGE/CHARGING/EMERGENCY）用于
演示和对照测试，不进入产品化 bringup。正式巡逻请通过 mission_manager 触发。

功能：
  1. 自动巡逻（客厅→走廊→卧室→厨房→客厅）
  2. 巡逻中检测异常（跌倒、入侵）
  3. 巡逻完成后返回充电桩
  4. 低电量自动回充
  5. 情绪交互（播放语音、表情回应）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import math

from shared_targets import get_named_target


class PatrolMission(Node):
    """家庭巡逻任务节点"""

    def __init__(self):
        super().__init__('patrol_mission')

        # 导航客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 发布器
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, '/patrol/alert', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.emotion_pub = self.create_publisher(String, '/emotion/command', 10)

        # 订阅器
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.fall_sub = self.create_subscription(
            Bool, '/fall_detected', self.fall_callback, 10)
        self.emotion_sub = self.create_subscription(
            String, '/emotion/recognized', self.emotion_callback, 10)

        # 状态
        self.battery_level = 1.0  # 电量 0-1
        self.low_battery_threshold = 0.2  # 低电量阈值
        self.is_charging = False
        self.patrol_count = 0
        self.current_waypoint = 0
        self.is_navigating = False
        self.emergency_stop = False

        # 充电桩位置（起点）— 从 puppy_core/config/navigation_targets.yaml 加载
        self.charging_dock = get_named_target(
            'dock', {'x': 0.0, 'y': -2.0, 'yaw': 0.0})

        # P0-5: 巡逻路径点从 puppy_core/config/patrol_routes.yaml 加载
        # 替代原硬编码 waypoints，统一坐标来源
        from shared_targets import get_patrol_route
        configured_route = get_patrol_route('default', [])
        if configured_route:
            self.patrol_waypoints = configured_route
            self.get_logger().info(f'从配置加载巡逻路线: {len(self.patrol_waypoints)} 个航点')
        else:
            # 回退到硬编码（仅当配置缺失时）
            self.get_logger().warn('patrol_routes.yaml 未找到，使用硬编码航点')
            self.patrol_waypoints = [
                {'name': '客厅', 'x': 2.0, 'y': 0.0, 'yaw': 0.0},
                {'name': '走廊', 'x': 2.0, 'y': 2.0, 'yaw': 1.57},
                {'name': '卧室', 'x': 0.5, 'y': 3.5, 'yaw': 3.14},
                {'name': '厨房', 'x': -2.0, 'y': 2.0, 'yaw': -1.57},
                {'name': '客厅返回', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            ]

        # 定时器：每 30 秒巡逻一次
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.state = 'IDLE'  # IDLE, PATROL, RETURN_CHARGE, CHARGING, EMERGENCY
        self.state_start_time = self.get_clock().now()

        self.get_logger().info('=' * 50)
        self.get_logger().info('  Puppy 家庭巡逻系统启动')
        self.get_logger().info('  巡逻路线: 客厅→走廊→卧室→厨房→充电桩')
        self.get_logger().info(f'  低电量阈值: {self.low_battery_threshold * 100}%')
        self.get_logger().info('=' * 50)

        self.publish_status('系统启动，准备巡逻')

    def battery_callback(self, msg):
        """电池状态回调"""
        self.battery_level = msg.percentage
        if msg.power_supply_status == 1:  # CHARGING
            self.is_charging = True
        else:
            self.is_charging = False

    def fall_callback(self, msg):
        """跌倒检测回调"""
        if msg.data and self.state != 'EMERGENCY':
            self.emergency_stop = True
            self.state = 'EMERGENCY'
            self.state_start_time = self.get_clock().now()
            alert = String()
            alert.data = '跌倒检测警报！检测到老人跌倒！'
            self.alert_pub.publish(alert)
            self.publish_status('紧急：检测到跌倒！')
            self.get_logger().error(alert.data)

    def emotion_callback(self, msg):
        """表情识别回调"""
        emotion = msg.data
        self.get_logger().info(f'检测到情绪: {emotion}')
        # 根据情绪播放相应回应
        responses = {
            'happy': '主人看起来很开心，我也很高兴！',
            'sad': '主人看起来不开心，我来陪陪你吧',
            'angry': '主人在生气，我安静一会儿',
            'surprised': '主人惊讶了，发生什么事了？',
            'neutral': '主人状态正常',
        }
        response = responses.get(emotion, '主人在呢')
        emotion_cmd = String()
        emotion_cmd.data = f'speak:{response}'
        self.emotion_pub.publish(emotion_cmd)

    def publish_status(self, message):
        """发布状态"""
        status = String()
        status.data = f'[{self.state}] {message}'
        self.status_pub.publish(status)
        self.get_logger().info(status.data)

    def timer_callback(self):
        """主状态机"""
        # 紧急状态处理
        if self.state == 'EMERGENCY':
            self.handle_emergency()
            return

        # 低电量检查
        if (self.battery_level < self.low_battery_threshold
                and self.state not in ['RETURN_CHARGE', 'CHARGING']):
            self.state = 'RETURN_CHARGE'
            self.state_start_time = self.get_clock().now()
            self.publish_status(f'低电量({self.battery_level*100:.0f}%)，返回充电')
            self.navigate_to_charging_dock()
            return

        if self.state == 'IDLE':
            # 开始巡逻
            self.state = 'PATROL'
            self.current_waypoint = 0
            self.state_start_time = self.get_clock().now()
            self.publish_status(f'开始第 {self.patrol_count + 1} 次巡逻')
            self.navigate_to_waypoint(self.current_waypoint)

        elif self.state == 'PATROL':
            if not self.is_navigating:
                # 到达当前路径点，扫描环境
                wp = self.patrol_waypoints[self.current_waypoint]
                self.publish_status(f'到达 {wp["name"]}，扫描环境...')
                self.scan_environment()

                self.current_waypoint += 1
                if self.current_waypoint >= len(self.patrol_waypoints):
                    # 巡逻完成，返回充电
                    self.patrol_count += 1
                    self.state = 'RETURN_CHARGE'
                    self.publish_status(f'巡逻完成(第{self.patrol_count}次)，返回充电桩')
                    self.navigate_to_charging_dock()
                else:
                    # 前往下一个路径点
                    self.navigate_to_waypoint(self.current_waypoint)

        elif self.state == 'RETURN_CHARGE':
            if not self.is_navigating:
                self.state = 'CHARGING'
                self.state_start_time = self.get_clock().now()
                self.publish_status('已到达充电桩，开始充电')
                # 播放情绪交互
                emotion_cmd = String()
                emotion_cmd.data = 'speak:巡逻完成，我回来充电啦，主人辛苦了'
                self.emotion_pub.publish(emotion_cmd)

        elif self.state == 'CHARGING':
            # 模拟充电过程
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            if elapsed > 10.0:  # 充电 10 秒后继续巡逻
                self.battery_level = 1.0
                self.state = 'IDLE'
                self.publish_status('充电完成，准备下一次巡逻')

    def navigate_to_waypoint(self, index):
        """导航到路径点"""
        if index >= len(self.patrol_waypoints):
            return
        wp = self.patrol_waypoints[index]
        self.publish_status(f'导航到 {wp["name"]} ({wp["x"]}, {wp["y"]})')
        self.send_navigation_goal(wp['x'], wp['y'], wp['yaw'])

    def navigate_to_charging_dock(self):
        """返回充电桩"""
        self.publish_status('导航返回充电桩')
        self.send_navigation_goal(
            self.charging_dock['x'],
            self.charging_dock['y'],
            self.charging_dock['yaw'])

    def send_navigation_goal(self, x, y, yaw):
        """发送导航目标"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('导航服务器不可用，使用直接速度控制')
            self.direct_navigate(x, y, yaw)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Yaw 转四元数
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.is_navigating = True
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback)
        self._send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """导航目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝')
            self.is_navigating = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """导航结果"""
        result = future.result().result
        self.is_navigating = False
        if result:
            self.get_logger().info('导航完成')
        else:
            self.get_logger().warn('导航失败')

    def nav_feedback_callback(self, feedback_msg):
        """导航反馈"""
        pass

    def direct_navigate(self, x, y, yaw):
        """直接速度控制导航（备用方案）"""
        self.is_navigating = True
        # 简单的前进+旋转
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        duration = 3.0  # 每个点走 3 秒

        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        twist = Twist()
        self.cmd_pub.publish(twist)
        self.is_navigating = False

    def scan_environment(self):
        """扫描环境（原地旋转）"""
        twist = Twist()
        twist.angular.z = 0.5  # 旋转速度
        duration = 4.0  # 旋转 4 秒

        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        twist = Twist()
        self.cmd_pub.publish(twist)

    def handle_emergency(self):
        """紧急状态处理"""
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        # 紧急状态持续 30 秒后恢复正常
        if elapsed > 30.0:
            self.emergency_stop = False
            self.state = 'RETURN_CHARGE'
            self.publish_status('紧急状态解除，返回充电')
            self.navigate_to_charging_dock()


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = PatrolMission()

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
