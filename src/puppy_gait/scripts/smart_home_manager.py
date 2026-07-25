#!/usr/bin/env python3
"""
智能家居家电管理节点
- 根据机器人位置自动控制灯光（人到灯亮，人走灯灭）
- 支持手动命令控制家电
- 发布家电状态到 /home_appliance/status
- 语音播报到 /voice/tts

家电列表:
  客厅灯、卧室灯、厨房灯、走廊灯、门口灯
  客厅空调、卧室空调
  客厅窗帘、卧室窗帘
"""
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool


# ===== 房间区域定义 (x_min, x_max, y_min, y_max) =====
# 注意：区域之间共享边界线（不重叠），按dict顺序优先匹配
ROOM_AREAS = {
    '客厅':   (0.5, 4.5, -3.5, -0.3),
    '充电桩': (-1.5, 0.5, -3.5, -1.5),
    '门口':   (-0.5, 0.5, -1.5, 0.0),
    '走廊':   (-0.8, 0.8, 0.0, 2.5),
    '卧室':   (-4.5, -0.8, 0.0, 3.5),
    '厨房':   (0.8, 4.5, 0.0, 3.5),
}

# ===== 家电初始状态 =====
APPLIANCES = {
    'living_room_light':  {'name': '客厅灯', 'type': 'light', 'on': False, 'room': '客厅'},
    'bedroom_light':      {'name': '卧室灯', 'type': 'light', 'on': False, 'room': '卧室'},
    'kitchen_light':      {'name': '厨房灯', 'type': 'light', 'on': False, 'room': '厨房'},
    'hallway_light':      {'name': '走廊灯', 'type': 'light', 'on': False, 'room': '走廊'},
    'door_light':         {'name': '门口灯', 'type': 'light', 'on': False, 'room': '门口'},
    'charging_light':     {'name': '充电桩灯', 'type': 'light', 'on': False, 'room': '充电桩'},
    'living_room_ac':     {'name': '客厅空调', 'type': 'ac', 'on': False, 'room': '客厅', 'temp': 26},
    'bedroom_ac':         {'name': '卧室空调', 'type': 'ac', 'on': False, 'room': '卧室', 'temp': 25},
    'living_room_curtain':{'name': '客厅窗帘', 'type': 'curtain', 'on': False, 'room': '客厅'},
    'bedroom_curtain':    {'name': '卧室窗帘', 'type': 'curtain', 'on': False, 'room': '卧室'},
}


class SmartHomeManager(Node):
    def __init__(self):
        super().__init__('smart_home_manager')

        # ===== 参数 =====
        self.declare_parameter('auto_light', True)  # 自动灯控
        self.declare_parameter('light_off_delay', 10.0)  # 离开房间后关灯延迟(秒)
        self.auto_light = self.get_parameter('auto_light').value
        self.light_off_delay = self.get_parameter('light_off_delay').value

        # ===== 状态 =====
        self.current_room = '未知'
        self.last_room = '未知'
        self.room_leave_time = {}  # 房间离开时间（用于延迟关灯）
        self.night_mode = False    # 夜间模式

        # ===== 订阅 =====
        self.create_subscription(PoseStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(String, '/home_appliance/cmd', self.cmd_callback, 10)
        self.create_subscription(String, '/task_scheduler/event', self.task_event_callback, 10)
        self.create_subscription(Bool, '/low_battery_alert', self.battery_callback, 10)

        # ===== 发布 =====
        self.status_pub = self.create_publisher(String, '/home_appliance/status', 10)
        self.voice_pub = self.create_publisher(String, '/voice/tts', 10)
        self.light_pub = self.create_publisher(Bool, '/home_appliance/light_state', 10)

        # ===== 定时器 =====
        self.create_timer(2.0, self.publish_status)
        self.create_timer(1.0, self.check_light_timeout)

        self.get_logger().info('智能家居家电管理节点已启动')
        self.get_logger().info(f'自动灯控: {"开启" if self.auto_light else "关闭"}, 关灯延迟: {self.light_off_delay}秒')

    def pose_callback(self, msg):
        """根据机器人位置判断所在房间，控制灯光"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.current_room = self.get_room(x, y)

        if self.current_room != self.last_room:
            self.on_room_change(self.last_room, self.current_room)
            self.last_room = self.current_room

    def get_room(self, x, y):
        """根据坐标判断所在房间"""
        for room, (x_min, x_max, y_min, y_max) in ROOM_AREAS.items():
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return room
        return '未知'

    def on_room_change(self, old_room, new_room):
        """房间切换时的处理"""
        now = self.get_clock().now().nanoseconds / 1e9

        if new_room != '未知':
            # 人到灯亮
            if self.auto_light and not self.night_mode:
                self.turn_on_room_lights(new_room)
                self.voice_pub.publish(String(data=f'欢迎来到{new_room}，灯已为您打开'))
            elif self.auto_light and self.night_mode and new_room == '走廊':
                # 夜间模式下走廊灯作为夜灯始终开启
                self.turn_on_room_lights(new_room)
            elif new_room == '充电桩' and self.auto_light:
                # 充电桩区域始终开灯（充电指示灯）
                self.turn_on_room_lights(new_room)

        if old_room != '未知' and old_room != new_room:
            self.room_leave_time[old_room] = now
            # 不立即关灯，等待延迟（在 check_light_timeout 中处理）

    def turn_on_room_lights(self, room):
        """打开指定房间的所有灯"""
        for key, appliance in APPLIANCES.items():
            if appliance['room'] == room and appliance['type'] == 'light':
                if not appliance['on']:
                    appliance['on'] = True
                    self.get_logger().info(f'开启: {appliance["name"]}')
                    self.light_pub.publish(Bool(data=True))

    def turn_off_room_lights(self, room):
        """关闭指定房间的所有灯"""
        for key, appliance in APPLIANCES.items():
            if appliance['room'] == room and appliance['type'] == 'light':
                if appliance['on']:
                    appliance['on'] = False
                    self.get_logger().info(f'关闭: {appliance["name"]}')
                    self.light_pub.publish(Bool(data=False))

    def check_light_timeout(self):
        """检查是否需要关闭已离开房间的灯"""
        now = self.get_clock().now().nanoseconds / 1e9
        for room, leave_time in list(self.room_leave_time.items()):
            if room == self.current_room:
                continue  # 当前房间不关灯
            if now - leave_time >= self.light_off_delay:
                self.turn_off_room_lights(room)
                del self.room_leave_time[room]

    def cmd_callback(self, msg):
        """处理手动家电控制命令
        命令格式: "on:客厅灯" / "off:卧室空调" / "all_off" / "all_on"
        """
        cmd = msg.data.strip()
        self.get_logger().info(f'收到家电命令: {cmd}')

        if cmd == 'all_off':
            for appliance in APPLIANCES.values():
                appliance['on'] = False
            self.voice_pub.publish(String(data='所有家电已关闭'))
            return

        if cmd == 'all_on':
            for appliance in APPLIANCES.values():
                appliance['on'] = True
            self.voice_pub.publish(String(data='所有家电已开启'))
            return

        if cmd == 'night_mode':
            self.night_mode = True
            for key, appliance in APPLIANCES.items():
                if appliance['type'] == 'curtain':
                    appliance['on'] = False  # 拉上窗帘
                if appliance['type'] == 'ac':
                    appliance['on'] = False
            self.voice_pub.publish(String(data='夜间模式已开启，窗帘已关闭，空调已关闭'))
            return

        if cmd == 'day_mode':
            self.night_mode = False
            for key, appliance in APPLIANCES.items():
                if appliance['type'] == 'curtain':
                    appliance['on'] = True  # 打开窗帘
            self.voice_pub.publish(String(data='日间模式已开启，窗帘已打开'))
            return

        # 单个设备控制: on:设备名 / off:设备名
        try:
            action, name = cmd.split(':')
            found = False
            for key, appliance in APPLIANCES.items():
                if name in appliance['name'] or name == key:
                    appliance['on'] = (action == 'on')
                    if appliance['type'] == 'ac' and action == 'on':
                        self.voice_pub.publish(
                            String(data=f'{appliance["name"]}已开启，温度{appliance["temp"]}度'))
                    else:
                        state = '开启' if action == 'on' else '关闭'
                        self.voice_pub.publish(
                            String(data=f'{appliance["name"]}已{state}'))
                    found = True
                    break
            if not found:
                self.voice_pub.publish(String(data=f'未找到设备: {name}'))
        except ValueError:
            self.get_logger().warn(f'命令格式错误: {cmd}')

    def task_event_callback(self, msg):
        """接收定时任务事件"""
        event = msg.data
        self.get_logger().info(f'收到定时任务事件: {event}')

        if event == 'morning_patrol':
            # 早安：打开所有窗帘，客厅灯
            self.night_mode = False
            for key, appliance in APPLIANCES.items():
                if appliance['type'] == 'curtain':
                    appliance['on'] = True
            self.voice_pub.publish(String(data='早安！窗帘已打开，新的一天开始了'))

        elif event == 'night_patrol':
            # 晚安：关闭窗帘，关闭空调，只留走廊灯
            self.night_mode = True
            for key, appliance in APPLIANCES.items():
                if appliance['type'] == 'curtain':
                    appliance['on'] = False
                if appliance['type'] == 'ac':
                    appliance['on'] = False
                if appliance['type'] == 'light' and appliance['room'] != '走廊':
                    appliance['on'] = False
            self.voice_pub.publish(String(data='夜间安防模式已开启，窗帘已关闭，只保留走廊灯'))

        elif event == 'noon_check':
            # 午间：开启客厅空调
            APPLIANCES['living_room_ac']['on'] = True
            self.voice_pub.publish(String(data='午间检查，客厅空调已开启'))

    def battery_callback(self, msg):
        """低电量时关闭非必要家电"""
        if msg.data:
            for key, appliance in APPLIANCES.items():
                if appliance['type'] == 'ac':
                    appliance['on'] = False
            self.get_logger().info('低电量警报：已关闭空调等大功率设备')

    def publish_status(self):
        """发布家电状态 (JSON)"""
        status = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'current_room': self.current_room,
            'night_mode': self.night_mode,
            'appliances': {k: {'name': v['name'], 'type': v['type'],
                               'on': v['on'], 'room': v['room']}
                           for k, v in APPLIANCES.items()}
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = SmartHomeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
