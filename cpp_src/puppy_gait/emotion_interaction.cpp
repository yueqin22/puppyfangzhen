// 情绪交互节点实现
#include "puppy_gait/emotion_interaction.h"

#include <chrono>
#include <random>

namespace puppy_gait {

// 情绪类型列表
static const std::vector<std::string> kEmotions = {
    "happy", "sad", "angry", "surprised", "neutral"};

// 情绪中文映射
static std::string emotionToCn(const std::string &emotion) {
  if (emotion == "happy") return "开心";
  if (emotion == "sad") return "难过";
  if (emotion == "angry") return "生气";
  if (emotion == "surprised") return "惊讶";
  if (emotion == "neutral") return "平静";
  return "未知";
}

// 情绪回应文本库（温暖关怀风格，适合陪伴老人）
// 每种情绪提供多条候选回应，随机选择一条，避免重复单调
static const std::vector<std::vector<std::string>> kEmotionResponses = {
    // happy
    {
        "看到主人开心，我也很高兴！今天天气真不错呢",
        "主人笑起来真好看，希望您每天都这么开心",
        "哇，主人今天心情很好呀，要不要一起去散散步",
        "开心的情绪最养人了，主人要多笑笑哦",
    },
    // sad
    {
        "主人不要难过，有我陪着你呢，要不要我讲个笑话",
        "看主人心情不好，我就在这里安静地陪着你",
        "别难过啦，生活总有起起落落，明天会更好的",
        "主人要是觉得心里难受，可以跟我说说话，我一直在",
    },
    // angry
    {
        "主人在生气吗？深呼吸，放松一下，我安静陪你",
        "生气伤身体呀，主人消消气，我给您唱首歌好不好",
        "别气别气，有什么不开心的事，跟我说说就好了",
        "主人冷静一下，我就在这里陪着你，慢慢来",
    },
    // surprised
    {
        "哇，主人惊讶了！发生什么有趣的事了",
        "主人这是看到什么了？快跟我分享分享",
        "惊讶的表情真可爱，是什么让主人这么意外",
        "看来主人遇到了新鲜事，要不要告诉我呀",
    },
    // neutral
    {
        "主人好，我在这里陪你",
        "主人状态很平静呢，这样挺好的",
        "我在您身边，有什么需要随时叫我",
        "主人，要不要聊聊天，或者一起听听音乐",
    },
};

// 定时巡逻问候语（主动关怀老人）
static const std::vector<std::string> kPatrolGreetings = {
    "主人，我巡逻一圈回来啦，一切正常，您放心",
    "主人，家里我都看过了，很安全，您安心休息",
    "主人，该喝水啦，记得多喝水对身体好",
    "主人，天气凉了，记得加件衣服哦",
    "主人，您今天感觉怎么样？有没有哪里不舒服",
    "主人，我在这儿呢，有什么事尽管吩咐我",
    "主人，久坐不好，起来活动活动吧，我陪您走走",
};

double EmotionInteractionNode::randomProbability() {
  static thread_local std::mt19937 gen{std::random_device{}()};
  static thread_local std::uniform_real_distribution<double> dist{0.0, 1.0};
  return dist(gen);
}

size_t EmotionInteractionNode::randomIndex(size_t size) {
  static thread_local std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<size_t> dist{0, size - 1};
  return dist(gen);
}

EmotionInteractionNode::EmotionInteractionNode()
    : rclcpp::Node("emotion_interaction") {
  // ===== 订阅器 =====
  // 接收交互命令，格式如 "speak:你好" 或 "detect_emotion"
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/emotion/command", 10,
      std::bind(&EmotionInteractionNode::commandCallback, this,
                std::placeholders::_1));

  // 订阅摄像头图像（用于表情识别，仿真版本不实际处理图像）
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth_camera/image_raw", 10,
      std::bind(&EmotionInteractionNode::imageCallback, this,
                std::placeholders::_1));

  // ===== 发布器 =====
  // 发布识别到的情绪（happy/sad/angry/surprised/neutral）
  recognized_pub_ =
      this->create_publisher<std_msgs::msg::String>("/emotion/recognized", 10);
  // 发布情绪回应文本
  response_pub_ =
      this->create_publisher<std_msgs::msg::String>("/emotion/response", 10);
  // 发布语音合成文本（供 TTS 节点播报）
  tts_pub_ = this->create_publisher<std_msgs::msg::String>("/voice/tts", 10);

  // ===== 定时器 =====
  // 表情识别定时器（仿真版：定时模拟识别情绪）
  detect_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(detect_period_)),
      std::bind(&EmotionInteractionNode::simulateEmotionDetection, this));
  // 主动问候定时器（巡逻时定时问候老人）
  greeting_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(greeting_period_)),
      std::bind(&EmotionInteractionNode::proactiveGreeting, this));

  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");
  RCLCPP_INFO(this->get_logger(), "  情绪交互节点启动（仿真版本）");
  RCLCPP_INFO(this->get_logger(), "  表情识别周期: %.1f 秒", detect_period_);
  RCLCPP_INFO(this->get_logger(), "  主动问候周期: %.1f 秒", greeting_period_);
  RCLCPP_INFO(this->get_logger(),
              "  订阅话题: /emotion/command, /camera/depth_camera/image_raw");
  RCLCPP_INFO(this->get_logger(),
              "  发布话题: /emotion/recognized, /emotion/response, /voice/tts");
  RCLCPP_INFO(this->get_logger(),
              "  注意: 仿真版本，实际部署需接入真实表情识别模型");
  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");

  // 启动时播报欢迎语
  publishTts("情绪交互系统已启动，主人您好，我会一直陪着您");
}

void EmotionInteractionNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr /*msg*/) {
  // 图像回调函数（仿真版本）
  // 在仿真中，我们不实际处理图像数据，仅统计帧数。
  // 实际部署时，此处应：
  //   1. 使用 cv_bridge 将 ROS Image 转换为 OpenCV 格式
  //   2. 运行人脸检测，截取人脸区域
  //   3. 输入表情识别模型，得到情绪类别和置信度
  //   4. 将识别结果发布到 /emotion/recognized
  frame_count_++;
}

void EmotionInteractionNode::commandCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  // 命令回调函数
  // 支持的命令格式：
  //   - "speak:文本"     : 通过语音合成播报指定文本
  //   - "detect_emotion" : 立即触发一次表情识别
  //   - "greeting"       : 主动问候老人
  std::string command = msg->data;
  // 去除首尾空白
  auto start = command.find_first_not_of(" \t\r\n");
  auto end = command.find_last_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return;
  }
  command = command.substr(start, end - start + 1);

  RCLCPP_INFO(this->get_logger(), "收到命令: %s", command.c_str());

  // 语音播报命令
  const std::string speak_prefix = "speak:";
  if (command.rfind(speak_prefix, 0) == 0) {
    std::string text = command.substr(speak_prefix.size());
    if (!text.empty()) {
      publishTts(text);
    }
    return;
  }

  // 触发表情识别
  if (command == "detect_emotion") {
    RCLCPP_INFO(this->get_logger(), "手动触发表情识别");
    simulateEmotionDetection();
    return;
  }

  // 触发主动问候
  if (command == "greeting") {
    RCLCPP_INFO(this->get_logger(), "手动触发主动问候");
    proactiveGreeting();
    return;
  }

  // 未知命令
  RCLCPP_WARN(this->get_logger(), "未知命令: %s", command.c_str());
}

void EmotionInteractionNode::simulateEmotionDetection() {
  // 仿真版表情识别
  // 随机模拟识别到一种情绪，并生成对应的回应文本。
  // 为增加真实感，识别结果有一定概率保持上一次的情绪。
  // 实际部署时，此方法应替换为真实模型推理结果。
  detect_counter_++;

  // 30% 概率保持上一次情绪，70% 概率随机切换
  // 模拟真实场景中情绪的连续性
  std::string emotion;
  if (randomProbability() < 0.3) {
    emotion = last_emotion_;
  } else {
    emotion = kEmotions[randomIndex(kEmotions.size())];
  }

  last_emotion_ = emotion;
  std::string emotion_cn = emotionToCn(emotion);

  RCLCPP_INFO(this->get_logger(),
              "[仿真] 第 %d 次表情识别结果: %s (%s)", detect_counter_,
              emotion.c_str(), emotion_cn.c_str());

  // 发布识别到的情绪
  std_msgs::msg::String recognized_msg;
  recognized_msg.data = emotion;
  recognized_pub_->publish(recognized_msg);

  // 生成并发布情绪回应
  generateEmotionResponse(emotion);
}

void EmotionInteractionNode::generateEmotionResponse(const std::string &emotion) {
  // 根据情绪生成回应文本
  // 根据识别到的情绪，从回应文本库中随机选择一条温暖关怀的文本，
  // 发布到 /emotion/response 和 /voice/tts 话题。
  rclcpp::Time now = this->now();

  // 冷却期检查，避免短时间内重复回应
  if (last_response_time_.has_value()) {
    double elapsed = (now - last_response_time_.value()).seconds();
    if (elapsed < response_cooldown_) {
      RCLCPP_INFO(this->get_logger(),
                  "回应冷却中（剩余 %.1f 秒），跳过本次回应",
                  response_cooldown_ - elapsed);
      return;
    }
  }

  last_response_time_ = now;

  // 从回应文本库中随机选择一条
  size_t idx = 0;
  if (emotion == "happy") {
    idx = 0;
  } else if (emotion == "sad") {
    idx = 1;
  } else if (emotion == "angry") {
    idx = 2;
  } else if (emotion == "surprised") {
    idx = 3;
  } else {
    idx = 4;
  }
  const auto &responses = kEmotionResponses[idx];
  std::string response = responses[randomIndex(responses.size())];

  // 发布情绪回应文本
  std_msgs::msg::String response_msg;
  response_msg.data = response;
  response_pub_->publish(response_msg);

  // 同时发布到语音合成话题进行播报
  publishTts(response);

  RCLCPP_INFO(this->get_logger(), "情绪回应: %s", response.c_str());
}

void EmotionInteractionNode::proactiveGreeting() {
  // 定时主动问候老人
  // 在巡逻过程中定时向老人发送问候语，提供情绪价值和陪伴感。
  std::string greeting =
      kPatrolGreetings[randomIndex(kPatrolGreetings.size())];

  // 发布回应文本
  std_msgs::msg::String response_msg;
  response_msg.data = greeting;
  response_pub_->publish(response_msg);

  // 发布到语音合成话题
  publishTts(greeting);

  RCLCPP_INFO(this->get_logger(), "主动问候: %s", greeting.c_str());
}

void EmotionInteractionNode::publishTts(const std::string &text) {
  // 发布语音合成文本
  // 将文本发布到 /voice/tts 话题，供语音合成节点播报。
  std_msgs::msg::String tts_msg;
  tts_msg.data = text;
  tts_pub_->publish(tts_msg);
}

}  // namespace puppy_gait
