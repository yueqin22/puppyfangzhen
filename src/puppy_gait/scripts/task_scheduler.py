#!/usr/bin/env python3
"""
定时任务调度节点
- 早安巡逻 (7:00): 打开窗帘，全屋巡逻，早安问候
- 午间检查 (12:00): 快速巡检，服药提醒
- 晚间安防 (22:00): 夜间模式，安防巡逻，关灯关窗帘

支持加速仿真模式: 每个真实分钟 = 1仿真小时
  即 7:00 → 运行后7分钟触发
     12:00 → 运行后12分钟触发
     22:00 → 运行后22分钟触发

也支持手动触发: /task_scheduler/cmd "morning" / "noon" / "night"
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')

        # ===== 参数 =====
        self.declare_parameter('accelerated_time', True)  # 加速时间模式
        self.declare_parameter('morning_hour', 7)
        self.declare_parameter('noon_hour', 12)
        self.declare_parameter('night_hour', 22)
        self.accelerated = self.get_parameter('accelerated_time').value
        self.morning_hour = self.get_parameter('morning_hour').value
        self.noon_hour = self.get_parameter('noon_hour').value
        self.night_hour = self.get_parameter('night_hour').value

        # ===== 状态 =====
        self.completed_tasks = set()  # 已完成的任务（防止重复触发）
        self.sim_day = 0  # 仿真天数
        self.last_sim_hour = -1

        # ===== 订阅 =====
        self.create_subscription(String, '/task_scheduler/cmd', self.cmd_callback, 10)

        # ===== 发布 =====
        self.event_pub = self.create_publisher(String, '/task_scheduler/event', 10)
        self.voice_pub = self.create_publisher(String, '/voice/tts', 10)

        # ===== 定时器 =====
        # 加速模式: 每分钟检查一次; 正常模式: 每分钟检查一次
        self.create_timer(10.0, self.check_schedule)  # 每10秒检查一次

        self.get_logger().info(
            f'定时任务调度节点已启动 (加速模式: {self.accelerated})')
        self.get_logger().info(
            f'计划: 早安{self.morning_hour}:00, 午间{self.noon_hour}:00, 晚间{self.night_hour}:00')

    def get_current_hour(self):
        """获取当前仿真小时
        加速模式: 运行时间(分钟) % 24 = 仿真小时
        正常模式: 实际时间小时
        """
        if self.accelerated:
            sim_time = self.get_clock().now()
            total_sec = sim_time.nanoseconds / 1e9
            # 每60秒 = 1仿真小时
            sim_hour = int((total_sec / 60.0) % 24)
            return sim_hour
        else:
            import time
            return time.localtime().tm_hour

    def check_schedule(self):
        """检查是否需要触发定时任务"""
        current_hour = self.get_current_hour()

        # 检测新的一天（小时从大变小）
        if current_hour < self.last_sim_hour:
            self.sim_day += 1
            self.completed_tasks.clear()
            self.get_logger().info(f'新的一天开始 (第{self.sim_day + 1}天)')

        self.last_sim_hour = current_hour

        # 生成任务标识（天+小时），防止重复触发
        task_base = f'day{self.sim_day}'

        # 早安巡逻
        if current_hour == self.morning_hour:
            task_id = f'{task_base}_morning'
            if task_id not in self.completed_tasks:
                self.completed_tasks.add(task_id)
                self.trigger_morning()

        # 午间检查
        elif current_hour == self.noon_hour:
            task_id = f'{task_base}_noon'
            if task_id not in self.completed_tasks:
                self.completed_tasks.add(task_id)
                self.trigger_noon()

        # 晚间安防
        elif current_hour == self.night_hour:
            task_id = f'{task_base}_night'
            if task_id not in self.completed_tasks:
                self.completed_tasks.add(task_id)
                self.trigger_night()

    def trigger_morning(self):
        """早安任务"""
        self.get_logger().info('===== 触发早安任务 =====')
        self.event_pub.publish(String(data='morning_patrol'))
        self.voice_pub.publish(String(
            data='早安主人！现在是早上7点，我来开启全屋巡逻。窗帘已打开，祝您一天好心情'))

    def trigger_noon(self):
        """午间任务"""
        self.get_logger().info('===== 触发午间任务 =====')
        self.event_pub.publish(String(data='noon_check'))
        self.voice_pub.publish(String(
            data='午间巡检时间到了。主人，别忘了吃午饭和服药哦，我来检查一下家里的安全状况'))

    def trigger_night(self):
        """晚间任务"""
        self.get_logger().info('===== 触发晚间安防任务 =====')
        self.event_pub.publish(String(data='night_patrol'))
        self.voice_pub.publish(String(
            data='晚间安防模式已启动。窗帘已关闭，灯光已调暗，我来做最后一次安全巡逻，主人晚安'))

    def cmd_callback(self, msg):
        """手动触发任务"""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到手动触发命令: {cmd}')

        if cmd in ('morning', '早安', '早上'):
            self.trigger_morning()
        elif cmd in ('noon', '午间', '中午'):
            self.trigger_noon()
        elif cmd in ('night', '晚间', '晚上', '晚安'):
            self.trigger_night()
        elif cmd == 'status':
            current_hour = self.get_current_hour()
            status = (f'当前仿真时间: 第{self.sim_day + 1}天 {current_hour}:00, '
                      f'已完成任务: {len(self.completed_tasks)}个')
            self.voice_pub.publish(String(data=status))
            self.get_logger().info(status)
        else:
            self.get_logger().warn(f'未知命令: {cmd}（支持: morning/noon/night/status）')


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = TaskScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
