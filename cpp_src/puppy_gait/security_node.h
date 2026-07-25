// Puppy 机械狗 - 家庭安防节点（C++ 仿真版本）
//
// 功能：
//   1. 订阅 /scan 话题（LaserScan），检测异常近距离物体（入侵检测）
//   2. 订阅 /fall_detected 话题（Bool），接收跌倒警报并升级为安防警报
//   3. 订阅 /patrol/status 话题（String），获取当前位置和巡逻状态
//   4. 订阅 /gas_sensor 话题（Float32），接收煤气/烟雾传感器数据
//   5. 仿真异常声音检测（定时随机模拟）
//   6. 警报分级：低（提醒）、中（警告）、高（紧急）
//   7. 警报记录和通知（语音播报 + 历史记录）
//   8. 发布 /security/alert（String）- 安防警报
//   9. 发布 /security/status（String）- 安防状态
//   10. 发布 /security/intrusion（Bool）- 入侵检测标志
//
// 注意：这是仿真版本，使用简化逻辑模拟各项检测功能。
// 实际部署时需要接入真实传感器：
//   - 激光雷达（已具备，需结合 SLAM 静态地图精确过滤已知家具）
//   - 麦克风阵列 + 声音分类模型（检测玻璃破碎、喊叫、撬门等异常声音）
//   - 煤气/烟雾传感器（MQ-2/MQ-7 等，通过 ADC 采集并发布到 /gas_sensor）
//   - 可选：人体热释电传感器（PIR）、门窗磁感传感器、摄像头人体检测
#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "puppy_interfaces/msg/security_event.hpp"
#include "puppy_interfaces/msg/fall_event.hpp"
#include "puppy_interfaces/msg/patrol_status.hpp"

namespace puppy_gait {

// ===== 警报级别定义 =====
// 提醒 / 警告 / 紧急
enum class AlertLevel {
  Normal = 0,
  Low = 1,
  Medium = 2,
  High = 3,
};

/// 家庭安防节点（仿真版本）
///
/// 整合激光雷达入侵检测、跌倒警报、煤气/烟雾检测和异常声音检测，
/// 提供分级警报、记录和通知功能。
class SecurityNode : public rclcpp::Node {
 public:
  SecurityNode();
  ~SecurityNode() override = default;

 private:
  // ===== 回调函数 =====
  // 激光雷达数据回调：检测异常近距离物体
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  // 跌倒警报回调（Bool 兼容版，边缘检测）
  void fallCallback(const std_msgs::msg::Bool::SharedPtr msg);
  // P0-1: 语义消息 FallEvent 回调
  void fallEventCallback(const puppy_interfaces::msg::FallEvent::SharedPtr msg);
  // 巡逻状态回调（String 兼容版）
  void patrolStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  // P0-1: 语义消息 PatrolStatus 回调
  void patrolStatusSemanticCallback(
      const puppy_interfaces::msg::PatrolStatus::SharedPtr msg);
  // 煤气/烟雾传感器数据回调
  void gasCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // ===== 仿真功能 =====
  // 仿真煤气/烟雾数据生成
  void simulateGasData();
  // 仿真异常声音检测
  void simulateSoundDetection();

  // ===== 检测逻辑 =====
  // 判断某点是否为已知家具
  bool isKnownFurniture(double angle, double distance) const;
  // 检查煤气浓度并触发相应级别警报
  void checkGasLevel();
  // 触发入侵警报
  void triggerIntrusionAlert(const std::vector<std::pair<double, double>> &points);
  // 检查是否需要解除入侵状态
  void checkIntrusionReset(const rclcpp::Time &now);
  // 角度转方位描述
  static std::string angleToDirection(double angle);
  // 触发分级警报（统一入口）
  void triggerAlert(AlertLevel level, const std::string &message,
                    const std::string &event_type = "generic",
                    const std::string &source_node = "security_node");

  // ===== 状态发布 =====
  void publishStatusMsg(const std::string &message);
  // 定时发布当前安防状态
  void publishStatus();
  // 检查是否需要将警报级别恢复为正常
  void checkAlertLevelReset();
  // 发布语音合成文本
  void publishTts(const std::string &text);

  // 警报级别转字符串
  static std::string alertLevelToString(AlertLevel level);
  // 字符串转警报级别
  static AlertLevel stringToAlertLevel(const std::string &s);

  // 生成均匀分布的随机数 [0.0, 1.0)
  static double randomProbability();
  // 从指定大小的列表中随机选择一个索引
  static size_t randomIndexStatic(size_t size);

 private:
  // ===== 订阅器 =====
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::FallEvent>::SharedPtr
      fall_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fall_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::PatrolStatus>::SharedPtr
      patrol_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr patrol_status_str_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gas_sub_;

  // ===== 发布器 =====
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Publisher<puppy_interfaces::msg::SecurityEvent>::SharedPtr
      security_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intrusion_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr sound_timer_;
  rclcpp::TimerBase::SharedPtr gas_sim_timer_;

  // ===== 安防参数 =====
  double intrusion_distance_threshold_{0.5};
  double intrusion_cooldown_{5.0};
  double gas_alert_cooldown_{10.0};
  double gas_warning_threshold_{200.0};
  double gas_danger_threshold_{500.0};
  double sound_check_period_{20.0};
  double sound_anomaly_probability_{0.15};
  double alert_reset_duration_{30.0};
  size_t max_alert_records_{100};

  // ===== 内部状态 =====
  std::string current_location_{"未知"};
  std::optional<rclcpp::Time> last_intrusion_time_;
  std::optional<rclcpp::Time> last_gas_alert_time_;
  double gas_concentration_{0.0};
  std::optional<rclcpp::Time> last_gas_data_time_;
  bool intrusion_detected_{false};
  bool last_fall_state_{false};
  AlertLevel current_alert_level_{AlertLevel::Normal};
  bool scan_received_{false};
  int sound_check_counter_{0};

  // 警报记录
  struct AlertRecord {
    double timestamp;
    AlertLevel level;
    std::string message;
    std::string location;
  };
  std::deque<AlertRecord> alert_records_;
};

}  // namespace puppy_gait
