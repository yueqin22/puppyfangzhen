// Puppy 机械狗 - 情绪交互节点（C++ 仿真版本）
//
// 功能：
//   1. 订阅 /emotion/command 话题，接收交互命令
//      - "speak:文本"   : 通过语音合成播报文本
//      - "detect_emotion": 触发一次表情识别
//   2. 订阅 /camera/depth_camera/image_raw 话题，获取摄像头图像（用于表情识别）
//   3. 仿真版表情识别：定时随机模拟识别到不同情绪
//   4. 根据情绪生成温暖关怀的回应文本（中文）
//   5. 发布识别到的情绪到 /emotion/recognized
//   6. 发布情绪回应文本到 /emotion/response
//   7. 发布语音合成文本到 /voice/tts
//   8. 定时巡逻时主动问候老人
//
// 注意：这是仿真版本，使用随机/定时方式模拟表情识别结果。
// 实际部署时需要接入用户已训练好的表情识别模型，例如：
//   - 基于 CNN 的面部表情分类模型（FER2013 / AffectNet 数据集）
//   - MediaPipe FaceMesh + 自定义分类器
//   - 预训练模型（如 DDRNet、MobileFaceNet）
// 并结合 cv_bridge 将 ROS Image 转换为模型输入张量，
// 将模型输出的情绪类别发布到 /emotion/recognized 话题。
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace puppy_gait {

/// 情绪交互节点（仿真版本）
///
/// 负责表情识别（仿真）、情绪回应生成、语音播报和主动问候。
class EmotionInteractionNode : public rclcpp::Node {
 public:
  EmotionInteractionNode();
  ~EmotionInteractionNode() override = default;

 private:
  // 图像回调（仿真版本仅统计帧数）
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  // 命令回调：支持 speak:/detect_emotion/greeting
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  // 仿真版表情识别
  void simulateEmotionDetection();
  // 根据情绪生成回应文本
  void generateEmotionResponse(const std::string &emotion);
  // 定时主动问候老人
  void proactiveGreeting();
  // 发布语音合成文本
  void publishTts(const std::string &text);

  // 生成均匀分布的随机数 [0.0, 1.0)
  static double randomProbability();
  // 从列表中随机选择一项
  static size_t randomIndex(size_t size);

 private:
  // ===== 仿真参数 =====
  double detect_period_{15.0};       // 表情识别周期（秒）
  double greeting_period_{60.0};     // 主动问候周期（秒）
  double response_cooldown_{5.0};    // 连续回应冷却时间（秒）

  // ===== 内部状态 =====
  std::string last_emotion_{"neutral"};
  std::optional<rclcpp::Time> last_response_time_;
  long frame_count_{0};
  int detect_counter_{0};

  // ===== 订阅器 =====
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // ===== 发布器 =====
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recognized_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr detect_timer_;
  rclcpp::TimerBase::SharedPtr greeting_timer_;
};

}  // namespace puppy_gait
