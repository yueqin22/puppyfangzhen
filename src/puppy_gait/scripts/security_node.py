#!/usr/bin/env python3
"""
Puppy 机械狗 - 家庭安防节点（仿真版本）

功能：
  1. 订阅 /scan 话题（LaserScan），检测异常近距离物体（入侵检测）
  2. 订阅 /fall_detected 话题（Bool），接收跌倒警报并升级为安防警报
  3. 订阅 /patrol/status 话题（String），获取当前位置和巡逻状态
  4. 订阅 /gas_sensor 话题（Float32），接收煤气/烟雾传感器数据
  5. 仿真异常声音检测（定时随机模拟）
  6. 警报分级：低（提醒）、中（警告）、高（紧急）
  7. 警报记录和通知（语音播报 + 历史记录）
  8. 发布 /security/alert（String）- 安防警报
  9. 发布 /security/status（String）- 安防状态
  10. 发布 /security/intrusion（Bool）- 入侵检测标志

注意：这是仿真版本，使用简化逻辑模拟各项检测功能。
实际部署时需要接入真实传感器：
  - 激光雷达（已具备，需结合 SLAM 静态地图精确过滤已知家具）
  - 麦克风阵列 + 声音分类模型（检测玻璃破碎、喊叫、撬门等异常声音）
  - 煤气/烟雾传感器（MQ-2/MQ-7 等，通过 ADC 采集并发布到 /gas_sensor）
  - 可选：人体热释电传感器（PIR）、门窗磁感传感器、摄像头人体检测
"""

import math
import random
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Float32

from puppy_interfaces.msg import SecurityEvent, FallEvent, PatrolStatus


# ===== 警报级别定义 =====
ALERT_LEVEL_LOW = '低'      # 提醒级别
ALERT_LEVEL_MEDIUM = '中'   # 警告级别
ALERT_LEVEL_HIGH = '高'     # 紧急级别

# 警报级别优先级（用于比较严重程度）
LEVEL_PRIORITY = {
    '正常': 0,
    ALERT_LEVEL_LOW: 1,
    ALERT_LEVEL_MEDIUM: 2,
    ALERT_LEVEL_HIGH: 3,
}

# ===== 已知家具位置（仿真用） =====
# 格式：(角度起始, 角度结束, 距离) 单位：弧度，米
# 这些角度范围内的近距离物体被视为已知家具，不触发入侵警报
# 实际部署时应从 SLAM 地图自动提取家具位置
KNOWN_FURNITURE = [
    (-0.3, 0.3, 0.35),    # 正前方桌子
    (1.5, 1.8, 0.40),     # 右侧沙发
    (-1.8, -1.5, 0.38),   # 左侧柜子
]

# 家庭房间名称（用于从巡逻状态中解析当前位置）
ROOM_NAMES = ['客厅', '门口', '走廊', '卧室', '厨房', '充电桩']


class SecurityNode(Node):
    """家庭安防节点（仿真版本）

    整合激光雷达入侵检测、跌倒警报、煤气/烟雾检测和异常声音检测，
    提供分级警报、记录和通知功能。
    """

    def __init__(self):
        super().__init__('security_node')

        # ===== 订阅器 =====
        # 激光雷达数据，用于入侵检测
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # P0-1: 跌倒警报 — 优先订阅语义消息 FallEvent 到 /fall/event
        # 保留 Bool /fall_detected 订阅以兼容旧版 fall_detection（现已双发）
        self.fall_event_sub = self.create_subscription(
            FallEvent, '/fall/event', self.fall_event_callback, 10)
        self.fall_sub = self.create_subscription(
            Bool, '/fall_detected', self.fall_callback, 10)

        # P0-1: 巡逻状态 — 优先订阅语义消息 PatrolStatus 到 /mission/status
        # 保留 String /patrol/status 订阅以兼容旧版 patrol 脚本
        self.patrol_status_sub = self.create_subscription(
            PatrolStatus, '/mission/status', self.patrol_status_semantic_callback, 10)
        self.patrol_status_str_sub = self.create_subscription(
            String, '/patrol/status', self.patrol_status_callback, 10)

        # 煤气/烟雾传感器数据（仿真，Float32 表示浓度 ppm）
        self.gas_sub = self.create_subscription(
            Float32, '/gas_sensor', self.gas_callback, 10)

        # ===== 发布器 =====
        # 安防警报（分级警报文本）
        self.alert_pub = self.create_publisher(
            String, '/security/alert', 10)

        # P0-1: 语义消息 SecurityEvent 到 /security/event
        # 供 puppy_core（mission_manager）订阅，保留 String /security/alert 兼容
        self.security_event_pub = self.create_publisher(
            SecurityEvent, '/security/event', 10)

        # 安防状态（当前系统状态摘要）
        self.status_pub = self.create_publisher(
            String, '/security/status', 10)

        # 入侵检测标志（Bool: True 表示检测到入侵）
        self.intrusion_pub = self.create_publisher(
            Bool, '/security/intrusion', 10)

        # 语音播报（通知用户）
        self.tts_pub = self.create_publisher(String, '/voice/tts', 10)

        # ===== 安防参数 =====
        # 入侵检测距离阈值（米）：小于此距离的物体视为异常近距离
        self.intrusion_distance_threshold = 0.5
        # 入侵检测冷却时间（秒）：避免短时间内重复报警
        self.intrusion_cooldown = 5.0
        # 煤气警报冷却时间（秒）：独立于入侵冷却
        self.gas_alert_cooldown = 10.0
        # 煤气浓度报警阈值（ppm 仿真值）
        self.gas_warning_threshold = 200.0   # 警告级别
        self.gas_danger_threshold = 500.0    # 危险级别
        # 异常声音检测周期（秒）
        self.sound_check_period = 20.0
        # 异常声音触发概率（仿真用）
        self.sound_anomaly_probability = 0.15
        # 警报级别自动恢复时间（秒）：无新警报后恢复为正常
        self.alert_reset_duration = 30.0
        # 警报记录最大条数
        self.max_alert_records = 100

        # ===== 内部状态 =====
        # 当前位置（从巡逻状态解析）
        self.current_location = '未知'
        # 上次入侵警报时间
        self.last_intrusion_time = None
        # 上次煤气警报时间
        self.last_gas_alert_time = None
        # 当前煤气浓度（ppm）
        self.gas_concentration = 0.0
        # 上次收到外部煤气数据的时间（用于判断是否需要仿真）
        self.last_gas_data_time = None
        # 是否检测到入侵
        self.intrusion_detected = False
        # 上次跌倒状态（边缘检测）
        self.last_fall_state = False
        # 警报记录队列（最近 N 条）
        self.alert_records = deque(maxlen=self.max_alert_records)
        # 当前警报级别
        self.current_alert_level = '正常'
        # 是否收到过激光雷达数据
        self.scan_received = False
        # 仿真用：声音检测计数器
        self.sound_check_counter = 0

        # ===== 定时器 =====
        # 状态发布定时器（1Hz，定期发布当前安防状态）
        self.status_timer = self.create_timer(1.0, self.publish_status)
        # 异常声音仿真检测定时器
        self.sound_timer = self.create_timer(
            self.sound_check_period, self.simulate_sound_detection)
        # 煤气数据仿真定时器（当无外部数据时生成仿真数据）
        self.gas_sim_timer = self.create_timer(2.0, self.simulate_gas_data)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  家庭安防节点启动（仿真版本）')
        self.get_logger().info(
            f'  入侵检测距离阈值: {self.intrusion_distance_threshold} m')
        self.get_logger().info(
            f'  煤气报警阈值: 警告 {self.gas_warning_threshold} ppm / '
            f'危险 {self.gas_danger_threshold} ppm')
        self.get_logger().info(
            f'  异常声音检测周期: {self.sound_check_period} 秒')
        self.get_logger().info(
            '  订阅话题: /scan, /fall_detected, /patrol/status, /gas_sensor')
        self.get_logger().info(
            '  发布话题: /security/alert, /security/status, /security/intrusion')
        self.get_logger().info('  注意: 仿真版本，实际部署需接入真实传感器')
        self.get_logger().info('=' * 50)

        self.publish_status_msg('安防系统启动，开始监控...')

    # ===== 回调函数 =====

    def scan_callback(self, msg):
        """激光雷达数据回调

        检测异常近距离物体（<0.5m），过滤已知家具后触发入侵警报。
        实际部署时，应结合 SLAM 地图和家具位置数据库进行精确过滤。
        """
        self.scan_received = True
        now = self.get_clock().now()

        # 冷却期检查，避免短时间内重复报警
        if self.last_intrusion_time is not None:
            elapsed = (now - self.last_intrusion_time).nanoseconds / 1e9
            if elapsed < self.intrusion_cooldown:
                return

        # 遍历激光雷达扫描数据，查找异常近距离物体
        intrusion_points = []
        for i, distance in enumerate(msg.ranges):
            # 跳过无效数据（inf/nan/超出量程）
            if distance < msg.range_min or distance > msg.range_max:
                continue
            # 检查是否为异常近距离物体
            if distance < self.intrusion_distance_threshold:
                # 计算该点的角度
                angle = msg.angle_min + i * msg.angle_increment
                # 过滤已知家具
                if not self.is_known_furniture(angle, distance):
                    intrusion_points.append((angle, distance))

        if intrusion_points:
            # 找到异常入侵物体，触发警报
            self.trigger_intrusion_alert(intrusion_points)
        else:
            # 未检测到入侵，检查是否需要解除入侵状态
            self.check_intrusion_reset(now)

    def fall_callback(self, msg):
        """跌倒警报回调（边缘检测：仅在False→True时升级警报）

        接收跌倒检测节点的警报，升级为高级别安防警报。
        """
        if msg.data and not self.last_fall_state:
            self.trigger_alert(
                ALERT_LEVEL_HIGH,
                f'检测到老人跌倒！位置：{self.current_location}，请立即查看')
        self.last_fall_state = msg.data

    def fall_event_callback(self, msg: FallEvent):
        """P0-1: 语义消息 FallEvent 回调（与 fall_callback 逻辑一致）。

        优先使用此回调；fall_callback 仅为兼容旧版 Bool /fall_detected。
        """
        if msg.detected and not self.last_fall_state:
            self.trigger_alert(
                ALERT_LEVEL_HIGH,
                f'检测到老人跌倒！位置：{self.current_location}，请立即查看',
                event_type='fall_detected', source_node='fall_detection')
        self.last_fall_state = msg.detected

    def patrol_status_callback(self, msg):
        """巡逻状态回调（String 兼容版）

        从巡逻状态消息中解析当前位置信息。
        巡逻状态格式示例："[PATROL] 到达 客厅，扫描环境..."
        """
        status_text = msg.data
        for room in ROOM_NAMES:
            if room in status_text:
                self.current_location = room
                break

    def patrol_status_semantic_callback(self, msg: PatrolStatus):
        """P0-1: 语义消息 PatrolStatus 回调。

        从 mission_manager 发布的 PatrolStatus 中提取状态消息，
        解析当前位置。
        """
        status_text = msg.message
        for room in ROOM_NAMES:
            if room in status_text:
                self.current_location = room
                break

    def gas_callback(self, msg):
        """煤气/烟雾传感器数据回调

        接收煤气浓度数据（ppm），超过阈值时触发相应级别警报。
        仿真数据可由外部节点发布，或由 simulate_gas_data 生成。
        """
        self.gas_concentration = msg.data
        self.last_gas_data_time = self.get_clock().now()
        self.check_gas_level()

    # ===== 仿真功能 =====

    def simulate_gas_data(self):
        """仿真煤气/烟雾数据生成

        当没有外部节点发布 /gas_sensor 数据时，定时生成仿真数据。
        正常情况下浓度较低，偶尔模拟异常情况（如厨房燃气泄漏）。
        实际部署时应移除此函数，使用真实传感器数据。
        """
        # 如果近期收到外部数据（5 秒内），则不生成仿真数据
        if self.last_gas_data_time is not None:
            elapsed = (self.get_clock().now() -
                       self.last_gas_data_time).nanoseconds / 1e9
            if elapsed < 5.0:
                return

        base_concentration = 50.0  # 正常背景浓度

        # 5% 概率模拟煤气泄漏事件
        if random.random() < 0.05:
            # 模拟厨房燃气泄漏
            self.gas_concentration = random.uniform(
                self.gas_warning_threshold, self.gas_danger_threshold * 1.5)
            self.get_logger().info(
                f'[仿真] 模拟煤气泄漏事件，浓度: {self.gas_concentration:.1f} ppm')
        else:
            # 正常浓度波动
            self.gas_concentration = base_concentration + random.uniform(-10.0, 10.0)

        self.check_gas_level()

    def simulate_sound_detection(self):
        """仿真异常声音检测

        定时模拟声音检测，有一定概率检测到异常声音。
        实际部署时，应接入麦克风阵列和声音分类模型，检测：
          - 玻璃破碎声
          - 异常喊叫/哭声
          - 撬门/砸物声
          - 非正常时段的脚步声
        """
        self.sound_check_counter += 1

        # 按概率模拟异常声音检测
        if random.random() < self.sound_anomaly_probability:
            sound_types = [
                '玻璃破碎声',
                '异常喊叫声',
                '撬门声',
                '重物坠落声',
            ]
            sound_type = random.choice(sound_types)

            self.trigger_alert(
                ALERT_LEVEL_MEDIUM,
                f'检测到异常声音，可能有人闯入（声音类型：{sound_type}）')

    # ===== 检测逻辑 =====

    def is_known_furniture(self, angle, distance):
        """判断某点是否为已知家具

        通过角度和距离匹配已知家具位置，避免误报。

        参数:
            angle: 检测点的角度（弧度）
            distance: 检测点的距离（米）

        返回:
            True 表示是已知家具，False 表示不是
        """
        for furn_angle_min, furn_angle_max, furn_distance in KNOWN_FURNITURE:
            # 角度在家具角度范围内，且距离接近家具距离（容差 0.1m）
            if (furn_angle_min <= angle <= furn_angle_max
                    and abs(distance - furn_distance) < 0.1):
                return True
        return False

    def check_gas_level(self):
        """检查煤气浓度并触发相应级别警报"""
        now = self.get_clock().now()

        # 冷却期检查
        if self.last_gas_alert_time is not None:
            elapsed = (now - self.last_gas_alert_time).nanoseconds / 1e9
            if elapsed < self.gas_alert_cooldown:
                return

        if self.gas_concentration >= self.gas_danger_threshold:
            # 危险级别：煤气浓度严重超标
            self.last_gas_alert_time = now
            self.trigger_alert(
                ALERT_LEVEL_HIGH,
                f'烟雾浓度超标！请检查厨房（浓度: {self.gas_concentration:.1f} ppm）')
        elif self.gas_concentration >= self.gas_warning_threshold:
            # 警告级别：煤气浓度偏高
            self.last_gas_alert_time = now
            self.trigger_alert(
                ALERT_LEVEL_MEDIUM,
                f'烟雾浓度偏高，请注意通风（浓度: {self.gas_concentration:.1f} ppm）')

    def trigger_intrusion_alert(self, intrusion_points):
        """触发入侵警报

        参数:
            intrusion_points: 异常近距离物体列表，每项为 (角度, 距离)
        """
        now = self.get_clock().now()
        self.last_intrusion_time = now
        self.intrusion_detected = True

        # 发布入侵检测标志
        intrusion_msg = Bool()
        intrusion_msg.data = True
        self.intrusion_pub.publish(intrusion_msg)

        # 计算最近物体的距离
        min_distance = min(d for _, d in intrusion_points)
        # 计算物体方位（角度转方位描述）
        avg_angle = sum(a for a, _ in intrusion_points) / len(intrusion_points)
        direction = self.angle_to_direction(avg_angle)

        # 触发高级别警报
        self.trigger_alert(
            ALERT_LEVEL_HIGH,
            f'检测到异常入侵！位置：{self.current_location} '
            f'（距离: {min_distance:.2f}m，方位: {direction}）')

    def check_intrusion_reset(self, now):
        """检查是否需要解除入侵状态

        当持续一段时间未检测到入侵物体时，解除入侵标志。
        """
        if not self.intrusion_detected:
            return

        if self.last_intrusion_time is not None:
            elapsed = (now - self.last_intrusion_time).nanoseconds / 1e9
            # 超过冷却时间 2 倍后解除入侵状态
            if elapsed > self.intrusion_cooldown * 2:
                self.intrusion_detected = False
                intrusion_msg = Bool()
                intrusion_msg.data = False
                self.intrusion_pub.publish(intrusion_msg)
                self.publish_status_msg('入侵警报解除，环境恢复正常')

    def angle_to_direction(self, angle):
        """角度转方位描述

        参数:
            angle: 角度（弧度）

        返回:
            方位描述字符串（正前方/左前方/右前方/左侧/右侧/后方等）
        """
        angle_deg = math.degrees(angle)
        if -30 <= angle_deg <= 30:
            return '正前方'
        elif 30 < angle_deg <= 90:
            return '右前方'
        elif -90 <= angle_deg < -30:
            return '左前方'
        elif 90 < angle_deg <= 150:
            return '右后方'
        elif -150 <= angle_deg < -90:
            return '左后方'
        else:
            return '正后方'

    def trigger_alert(self, level, message, event_type='generic', source_node='security_node'):
        """触发分级警报

        统一的警报触发入口，负责：
          1. 发布警报文本到 /security/alert（兼容）
          2. P0-1: 发布语义消息 SecurityEvent 到 /security/event
          3. 记录警报到历史记录
          4. 发布语音通知到 /voice/tts
          5. 更新当前警报级别

        参数:
            level: 警报级别（ALERT_LEVEL_LOW / ALERT_LEVEL_MEDIUM / ALERT_LEVEL_HIGH）
            message: 警报消息文本
            event_type: 事件类型标识（fall_detected/intrusion/gas/sound/generic）
            source_node: 触发源节点名
        """
        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        # 构造警报消息（包含级别前缀）
        alert_text = f'[{level}级警报] {message}'

        # 发布警报（兼容 topic）
        alert_msg = String()
        alert_msg.data = alert_text
        self.alert_pub.publish(alert_msg)

        # P0-1: 发布语义消息 SecurityEvent 到 /security/event
        # 供 mission_manager 订阅，severity 映射: 高=critical / 中=warning / 低=info
        severity_map = {ALERT_LEVEL_HIGH: 'critical', ALERT_LEVEL_MEDIUM: 'warning', ALERT_LEVEL_LOW: 'info'}
        sem = SecurityEvent()
        sem.header.stamp = now.to_msg()
        sem.event_type = event_type
        sem.severity = severity_map.get(level, 'info')
        sem.source_node = source_node
        sem.message = message
        self.security_event_pub.publish(sem)

        # 记录警报到历史
        self.alert_records.append({
            'timestamp': timestamp,
            'level': level,
            'message': message,
            'location': self.current_location,
        })

        # 更新当前警报级别（取最高级别）
        if LEVEL_PRIORITY.get(level, 0) > LEVEL_PRIORITY.get(
                self.current_alert_level, 0):
            self.current_alert_level = level

        # 语音通知
        self.publish_tts(message)

        # 日志输出（根据级别使用不同日志方法）
        if level == ALERT_LEVEL_HIGH:
            self.get_logger().error(alert_text)
        elif level == ALERT_LEVEL_MEDIUM:
            self.get_logger().warn(alert_text)
        else:
            self.get_logger().info(alert_text)

    # ===== 状态发布 =====

    def publish_status_msg(self, message):
        """发布安防状态消息到 /security/status 话题"""
        status = String()
        status.data = message
        self.status_pub.publish(status)

    def publish_status(self):
        """定时发布当前安防状态（由定时器调用）

        包含警报级别、位置、煤气浓度、入侵状态和警报统计信息。
        同时检查是否需要自动恢复警报级别。
        """
        # 检查警报级别自动恢复
        self.check_alert_level_reset()

        # 统计各级别警报数量
        high_count = sum(
            1 for r in self.alert_records if r['level'] == ALERT_LEVEL_HIGH)
        medium_count = sum(
            1 for r in self.alert_records if r['level'] == ALERT_LEVEL_MEDIUM)
        low_count = sum(
            1 for r in self.alert_records if r['level'] == ALERT_LEVEL_LOW)

        status_text = (
            f'[警报级别: {self.current_alert_level}] '
            f'位置: {self.current_location} | '
            f'煤气: {self.gas_concentration:.1f} ppm | '
            f'入侵: {"是" if self.intrusion_detected else "否"} | '
            f'警报记录: 高{high_count}/中{medium_count}/低{low_count}')
        self.publish_status_msg(status_text)

    def check_alert_level_reset(self):
        """检查是否需要将警报级别恢复为正常

        当超过 alert_reset_duration 秒无新警报时，恢复为正常级别。
        """
        if not self.alert_records:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        last_alert_time = self.alert_records[-1]['timestamp']

        if current_time - last_alert_time > self.alert_reset_duration:
            if self.current_alert_level != '正常':
                self.current_alert_level = '正常'
                self.publish_status_msg('警报级别已恢复为正常')

    def publish_tts(self, text):
        """发布语音合成文本到 /voice/tts 话题

        参数:
            text: 待播报的文本内容
        """
        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = SecurityNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        # 退出前清除入侵标志
        try:
            intrusion_msg = Bool()
            intrusion_msg.data = False
            node.intrusion_pub.publish(intrusion_msg)
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
