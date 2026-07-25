#!/usr/bin/env python3
"""
Puppy 机械狗 - 情绪交互节点（仿真版本）

功能：
  1. 订阅 /emotion/command 话题，接收交互命令
     - "speak:文本"   : 通过语音合成播报文本
     - "detect_emotion": 触发一次表情识别
  2. 订阅 /camera/depth_camera/image_raw 话题，获取摄像头图像（用于表情识别）
  3. 仿真版表情识别：定时随机模拟识别到不同情绪
  4. 根据情绪生成温暖关怀的回应文本（中文）
  5. 发布识别到的情绪到 /emotion/recognized
  6. 发布情绪回应文本到 /emotion/response
  7. 发布语音合成文本到 /voice/tts
  8. 定时巡逻时主动问候老人

注意：这是仿真版本，使用随机/定时方式模拟表情识别结果。
实际部署时需要接入用户已训练好的表情识别模型，例如：
  - 基于 CNN 的面部表情分类模型（FER2013 / AffectNet 数据集）
  - MediaPipe FaceMesh + 自定义分类器
  - 预训练模型（如 DDRNet、MobileFaceNet）
并结合 cv_bridge 将 ROS Image 转换为模型输入张量，
将模型输出的情绪类别发布到 /emotion/recognized 话题。
"""

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# 情绪类型列表
EMOTIONS = ['happy', 'sad', 'angry', 'surprised', 'neutral']

# 情绪中文映射
EMOTION_CN = {
    'happy': '开心',
    'sad': '难过',
    'angry': '生气',
    'surprised': '惊讶',
    'neutral': '平静',
}

# 情绪回应文本库（温暖关怀风格，适合陪伴老人）
# 每种情绪提供多条候选回应，随机选择一条，避免重复单调
EMOTION_RESPONSES = {
    'happy': [
        '看到主人开心，我也很高兴！今天天气真不错呢',
        '主人笑起来真好看，希望您每天都这么开心',
        '哇，主人今天心情很好呀，要不要一起去散散步',
        '开心的情绪最养人了，主人要多笑笑哦',
    ],
    'sad': [
        '主人不要难过，有我陪着你呢，要不要我讲个笑话',
        '看主人心情不好，我就在这里安静地陪着你',
        '别难过啦，生活总有起起落落，明天会更好的',
        '主人要是觉得心里难受，可以跟我说说话，我一直在',
    ],
    'angry': [
        '主人在生气吗？深呼吸，放松一下，我安静陪你',
        '生气伤身体呀，主人消消气，我给您唱首歌好不好',
        '别气别气，有什么不开心的事，跟我说说就好了',
        '主人冷静一下，我就在这里陪着你，慢慢来',
    ],
    'surprised': [
        '哇，主人惊讶了！发生什么有趣的事了',
        '主人这是看到什么了？快跟我分享分享',
        '惊讶的表情真可爱，是什么让主人这么意外',
        '看来主人遇到了新鲜事，要不要告诉我呀',
    ],
    'neutral': [
        '主人好，我在这里陪你',
        '主人状态很平静呢，这样挺好的',
        '我在您身边，有什么需要随时叫我',
        '主人，要不要聊聊天，或者一起听听音乐',
    ],
}

# 定时巡逻问候语（主动关怀老人）
PATROL_GREETINGS = [
    '主人，我巡逻一圈回来啦，一切正常，您放心',
    '主人，家里我都看过了，很安全，您安心休息',
    '主人，该喝水啦，记得多喝水对身体好',
    '主人，天气凉了，记得加件衣服哦',
    '主人，您今天感觉怎么样？有没有哪里不舒服',
    '主人，我在这儿呢，有什么事尽管吩咐我',
    '主人，久坐不好，起来活动活动吧，我陪您走走',
]


class EmotionInteraction(Node):
    """情绪交互节点（仿真版本）

    负责表情识别（仿真）、情绪回应生成、语音播报和主动问候。
    """

    def __init__(self):
        super().__init__('emotion_interaction')

        # ===== 订阅器 =====
        # 接收交互命令，格式如 "speak:你好" 或 "detect_emotion"
        self.command_sub = self.create_subscription(
            String, '/emotion/command', self.command_callback, 10)

        # 订阅摄像头图像（用于表情识别，仿真版本不实际处理图像）
        self.image_sub = self.create_subscription(
            Image, '/camera/depth_camera/image_raw', self.image_callback, 10)

        # ===== 发布器 =====
        # 发布识别到的情绪（happy/sad/angry/surprised/neutral）
        self.recognized_pub = self.create_publisher(
            String, '/emotion/recognized', 10)

        # 发布情绪回应文本
        self.response_pub = self.create_publisher(
            String, '/emotion/response', 10)

        # 发布语音合成文本（供 TTS 节点播报）
        self.tts_pub = self.create_publisher(String, '/voice/tts', 10)

        # ===== 仿真参数 =====
        # 表情识别周期（秒）：每隔多久模拟识别一次情绪
        self.detect_period = 15.0
        # 主动问候周期（秒）：巡逻时每隔多久主动问候老人
        self.greeting_period = 60.0
        # 同一情绪连续回应冷却时间（秒）：避免短时间内重复回应
        self.response_cooldown = 5.0

        # ===== 内部状态 =====
        # 上次识别到的情绪
        self.last_emotion = 'neutral'
        # 上次发布回应的时间
        self.last_response_time = None
        # 接收到的图像帧计数
        self.frame_count = 0
        # 仿真用：情绪切换计数器
        self.detect_counter = 0

        # ===== 定时器 =====
        # 表情识别定时器（仿真版：定时模拟识别情绪）
        self.detect_timer = self.create_timer(
            self.detect_period, self.simulate_emotion_detection)

        # 主动问候定时器（巡逻时定时问候老人）
        self.greeting_timer = self.create_timer(
            self.greeting_period, self.proactive_greeting)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  情绪交互节点启动（仿真版本）')
        self.get_logger().info(f'  表情识别周期: {self.detect_period} 秒')
        self.get_logger().info(f'  主动问候周期: {self.greeting_period} 秒')
        self.get_logger().info('  订阅话题: /emotion/command, /camera/depth_camera/image_raw')
        self.get_logger().info(
            '  发布话题: /emotion/recognized, /emotion/response, /voice/tts')
        self.get_logger().info('  注意: 仿真版本，实际部署需接入真实表情识别模型')
        self.get_logger().info('=' * 50)

        # 启动时播报欢迎语
        self.publish_tts('情绪交互系统已启动，主人您好，我会一直陪着您')

    def image_callback(self, msg):
        """图像回调函数（仿真版本）

        在仿真中，我们不实际处理图像数据，仅统计帧数。
        实际部署时，此处应：
          1. 使用 cv_bridge 将 ROS Image 转换为 OpenCV 格式
          2. 运行人脸检测，截取人脸区域
          3. 输入表情识别模型，得到情绪类别和置信度
          4. 将识别结果发布到 /emotion/recognized
        """
        self.frame_count += 1

    def command_callback(self, msg):
        """命令回调函数

        支持的命令格式：
          - "speak:文本"     : 通过语音合成播报指定文本
          - "detect_emotion" : 立即触发一次表情识别
          - "greeting"       : 主动问候老人
        """
        command = msg.data.strip()
        if not command:
            return

        self.get_logger().info(f'收到命令: {command}')

        # 语音播报命令
        if command.startswith('speak:'):
            text = command[len('speak:'):]
            if text:
                self.publish_tts(text)
            return

        # 触发表情识别
        if command == 'detect_emotion':
            self.get_logger().info('手动触发表情识别')
            self.simulate_emotion_detection()
            return

        # 触发主动问候
        if command == 'greeting':
            self.get_logger().info('手动触发主动问候')
            self.proactive_greeting()
            return

        # 未知命令
        self.get_logger().warn(f'未知命令: {command}')

    def simulate_emotion_detection(self):
        """仿真版表情识别

        随机模拟识别到一种情绪，并生成对应的回应文本。
        为增加真实感，识别结果有一定概率保持上一次的情绪。
        实际部署时，此方法应替换为真实模型推理结果。
        """
        self.detect_counter += 1

        # 30% 概率保持上一次情绪，70% 概率随机切换
        # 模拟真实场景中情绪的连续性
        if random.random() < 0.3:
            emotion = self.last_emotion
        else:
            emotion = random.choice(EMOTIONS)

        self.last_emotion = emotion
        emotion_cn = EMOTION_CN.get(emotion, '未知')

        self.get_logger().info(
            f'[仿真] 第 {self.detect_counter} 次表情识别结果: '
            f'{emotion} ({emotion_cn})')

        # 发布识别到的情绪
        recognized_msg = String()
        recognized_msg.data = emotion
        self.recognized_pub.publish(recognized_msg)

        # 生成并发布情绪回应
        self.generate_emotion_response(emotion)

    def generate_emotion_response(self, emotion):
        """根据情绪生成回应文本

        根据识别到的情绪，从回应文本库中随机选择一条温暖关怀的文本，
        发布到 /emotion/response 和 /voice/tts 话题。

        参数:
            emotion: 情绪类别（happy/sad/angry/surprised/neutral）
        """
        now = self.get_clock().now()

        # 冷却期检查，避免短时间内重复回应
        if self.last_response_time is not None:
            elapsed = (now - self.last_response_time).nanoseconds / 1e9
            if elapsed < self.response_cooldown:
                self.get_logger().info(
                    f'回应冷却中（剩余 {self.response_cooldown - elapsed:.1f} 秒），跳过本次回应')
                return

        self.last_response_time = now

        # 从回应文本库中随机选择一条
        responses = EMOTION_RESPONSES.get(emotion, ['主人好，我在这里陪你'])
        response = random.choice(responses)

        # 发布情绪回应文本
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

        # 同时发布到语音合成话题进行播报
        self.publish_tts(response)

        self.get_logger().info(f'情绪回应: {response}')

    def proactive_greeting(self):
        """定时主动问候老人

        在巡逻过程中定时向老人发送问候语，提供情绪价值和陪伴感。
        """
        greeting = random.choice(PATROL_GREETINGS)

        # 发布回应文本
        response_msg = String()
        response_msg.data = greeting
        self.response_pub.publish(response_msg)

        # 发布到语音合成话题
        self.publish_tts(greeting)

        self.get_logger().info(f'主动问候: {greeting}')

    def publish_tts(self, text):
        """发布语音合成文本

        将文本发布到 /voice/tts 话题，供语音合成节点播报。

        参数:
            text: 待播报的文本内容
        """
        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = EmotionInteraction()

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
