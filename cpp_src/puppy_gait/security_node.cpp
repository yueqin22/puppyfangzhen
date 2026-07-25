// 家庭安防节点实现
#include "puppy_gait/security_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppy_gait {

// ===== 已知家具位置（仿真用）=====
// 格式：(角度起始, 角度结束, 距离) 单位：弧度，米
// 这些角度范围内的近距离物体被视为已知家具，不触发入侵警报
// 实际部署时应从 SLAM 地图自动提取家具位置
struct FurnitureSpec {
  double angle_min;
  double angle_max;
  double distance;
};

static const std::vector<FurnitureSpec> kKnownFurniture = {
    {-0.3, 0.3, 0.35},    // 正前方桌子
    {1.5, 1.8, 0.40},     // 右侧沙发
    {-1.8, -1.5, 0.38},   // 左侧柜子
};

// 家庭房间名称（用于从巡逻状态中解析当前位置）
static const std::vector<std::string> kRoomNames = {
    "客厅", "门口", "走廊", "卧室", "厨房", "充电桩"};

// 异常声音类型（仿真用）
static const std::vector<std::string> kSoundTypes = {
    "玻璃破碎声", "异常喊叫声", "撬门声", "重物坠落声"};

double SecurityNode::randomProbability() {
  static thread_local std::mt19937 gen{std::random_device{}()};
  static thread_local std::uniform_real_distribution<double> dist{0.0, 1.0};
  return dist(gen);
}

size_t SecurityNode::randomIndexStatic(size_t size) {
  if (size == 0) {
    return 0;
  }
  static thread_local std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<size_t> dist{0, size - 1};
  return dist(gen);
}

std::string SecurityNode::alertLevelToString(AlertLevel level) {
  switch (level) {
    case AlertLevel::Low:
      return "低";
    case AlertLevel::Medium:
      return "中";
    case AlertLevel::High:
      return "高";
    case AlertLevel::Normal:
    default:
      return "正常";
  }
}

AlertLevel SecurityNode::stringToAlertLevel(const std::string &s) {
  if (s == "高") return AlertLevel::High;
  if (s == "中") return AlertLevel::Medium;
  if (s == "低") return AlertLevel::Low;
  return AlertLevel::Normal;
}

SecurityNode::SecurityNode() : rclcpp::Node("security_node") {
  // ===== 订阅器 =====
  // 激光雷达数据，用于入侵检测
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SecurityNode::scanCallback, this, std::placeholders::_1));

  // P0-1: 跌倒警报 — 优先订阅语义消息 FallEvent 到 /fall/event
  // 保留 Bool /fall_detected 订阅以兼容旧版 fall_detection（现已双发）
  fall_event_sub_ =
      this->create_subscription<puppy_interfaces::msg::FallEvent>(
          "/fall/event", 10,
          std::bind(&SecurityNode::fallEventCallback, this,
                    std::placeholders::_1));
  fall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fall_detected", 10,
      std::bind(&SecurityNode::fallCallback, this, std::placeholders::_1));

  // P0-1: 巡逻状态 — 优先订阅语义消息 PatrolStatus 到 /mission/status
  // 保留 String /patrol/status 订阅以兼容旧版 patrol 脚本
  patrol_status_sub_ =
      this->create_subscription<puppy_interfaces::msg::PatrolStatus>(
          "/mission/status", 10,
          std::bind(&SecurityNode::patrolStatusSemanticCallback, this,
                    std::placeholders::_1));
  patrol_status_str_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/patrol/status", 10,
      std::bind(&SecurityNode::patrolStatusCallback, this,
                std::placeholders::_1));

  // 煤气/烟雾传感器数据（仿真，Float32 表示浓度 ppm）
  gas_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/gas_sensor", 10,
      std::bind(&SecurityNode::gasCallback, this, std::placeholders::_1));

  // ===== 发布器 =====
  // 安防警报（分级警报文本）
  alert_pub_ =
      this->create_publisher<std_msgs::msg::String>("/security/alert", 10);
  // P0-1: 语义消息 SecurityEvent 到 /security/event
  // 供 puppy_core（mission_manager）订阅，保留 String /security/alert 兼容
  security_event_pub_ =
      this->create_publisher<puppy_interfaces::msg::SecurityEvent>(
          "/security/event", 10);
  // 安防状态（当前系统状态摘要）
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/security/status", 10);
  // 入侵检测标志（Bool: True 表示检测到入侵）
  intrusion_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/security/intrusion", 10);
  // 语音播报（通知用户）
  tts_pub_ = this->create_publisher<std_msgs::msg::String>("/voice/tts", 10);

  // ===== 定时器 =====
  // 状态发布定时器（1Hz，定期发布当前安防状态）
  status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&SecurityNode::publishStatus, this));
  // 异常声音仿真检测定时器
  sound_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(sound_check_period_)),
      std::bind(&SecurityNode::simulateSoundDetection, this));
  // 煤气数据仿真定时器（当无外部数据时生成仿真数据）
  gas_sim_timer_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&SecurityNode::simulateGasData, this));

  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");
  RCLCPP_INFO(this->get_logger(), "  家庭安防节点启动（仿真版本）");
  RCLCPP_INFO(this->get_logger(), "  入侵检测距离阈值: %.2f m",
              intrusion_distance_threshold_);
  RCLCPP_INFO(this->get_logger(),
              "  煤气报警阈值: 警告 %.0f ppm / 危险 %.0f ppm",
              gas_warning_threshold_, gas_danger_threshold_);
  RCLCPP_INFO(this->get_logger(), "  异常声音检测周期: %.1f 秒",
              sound_check_period_);
  RCLCPP_INFO(this->get_logger(),
              "  订阅话题: /scan, /fall_detected, /patrol/status, /gas_sensor");
  RCLCPP_INFO(this->get_logger(),
              "  发布话题: /security/alert, /security/status, /security/intrusion");
  RCLCPP_INFO(this->get_logger(), "  注意: 仿真版本，实际部署需接入真实传感器");
  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");

  publishStatusMsg("安防系统启动，开始监控...");
}

// ===== 回调函数 =====

void SecurityNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 激光雷达数据回调
  // 检测异常近距离物体（<0.5m），过滤已知家具后触发入侵警报。
  // 实际部署时，应结合 SLAM 地图和家具位置数据库进行精确过滤。
  scan_received_ = true;
  rclcpp::Time now = this->get_clock()->now();

  // 冷却期检查，避免短时间内重复报警
  if (last_intrusion_time_.has_value()) {
    double elapsed = (now - last_intrusion_time_.value()).seconds();
    if (elapsed < intrusion_cooldown_) {
      return;
    }
  }

  // 遍历激光雷达扫描数据，查找异常近距离物体
  std::vector<std::pair<double, double>> intrusion_points;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double distance = msg->ranges[i];
    // 跳过无效数据（inf/nan/超出量程）
    if (distance < msg->range_min || distance > msg->range_max) {
      continue;
    }
    // 检查是否为异常近距离物体
    if (distance < intrusion_distance_threshold_) {
      // 计算该点的角度
      double angle = msg->angle_min +
                    static_cast<double>(i) * msg->angle_increment;
      // 过滤已知家具
      if (!isKnownFurniture(angle, distance)) {
        intrusion_points.emplace_back(angle, distance);
      }
    }
  }

  if (!intrusion_points.empty()) {
    // 找到异常入侵物体，触发警报
    triggerIntrusionAlert(intrusion_points);
  } else {
    // 未检测到入侵，检查是否需要解除入侵状态
    checkIntrusionReset(now);
  }
}

void SecurityNode::fallCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  // 跌倒警报回调（边缘检测：仅在 False→True 时升级警报）
  // 接收跌倒检测节点的警报，升级为高级别安防警报。
  if (msg->data && !last_fall_state_) {
    triggerAlert(AlertLevel::High,
                 "检测到老人跌倒！位置：" + current_location_ + "，请立即查看");
  }
  last_fall_state_ = msg->data;
}

void SecurityNode::fallEventCallback(
    const puppy_interfaces::msg::FallEvent::SharedPtr msg) {
  // P0-1: 语义消息 FallEvent 回调（与 fallCallback 逻辑一致）。
  // 优先使用此回调；fallCallback 仅为兼容旧版 Bool /fall_detected。
  if (msg->detected && !last_fall_state_) {
    triggerAlert(AlertLevel::High,
                 "检测到老人跌倒！位置：" + current_location_ + "，请立即查看",
                 "fall_detected", "fall_detection");
  }
  last_fall_state_ = msg->detected;
}

void SecurityNode::patrolStatusCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  // 巡逻状态回调（String 兼容版）
  // 从巡逻状态消息中解析当前位置信息。
  // 巡逻状态格式示例："[PATROL] 到达 客厅，扫描环境..."
  const std::string &status_text = msg->data;
  for (const auto &room : kRoomNames) {
    if (status_text.find(room) != std::string::npos) {
      current_location_ = room;
      break;
    }
  }
}

void SecurityNode::patrolStatusSemanticCallback(
    const puppy_interfaces::msg::PatrolStatus::SharedPtr msg) {
  // P0-1: 语义消息 PatrolStatus 回调。
  // 从 mission_manager 发布的 PatrolStatus 中提取状态消息，解析当前位置。
  const std::string &status_text = msg->message;
  for (const auto &room : kRoomNames) {
    if (status_text.find(room) != std::string::npos) {
      current_location_ = room;
      break;
    }
  }
}

void SecurityNode::gasCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  // 煤气/烟雾传感器数据回调
  // 接收煤气浓度数据（ppm），超过阈值时触发相应级别警报。
  // 仿真数据可由外部节点发布，或由 simulateGasData 生成。
  gas_concentration_ = msg->data;
  last_gas_data_time_ = this->get_clock()->now();
  checkGasLevel();
}

// ===== 仿真功能 =====

void SecurityNode::simulateGasData() {
  // 仿真煤气/烟雾数据生成
  // 当没有外部节点发布 /gas_sensor 数据时，定时生成仿真数据。
  // 正常情况下浓度较低，偶尔模拟异常情况（如厨房燃气泄漏）。
  // 实际部署时应移除此函数，使用真实传感器数据。
  if (last_gas_data_time_.has_value()) {
    double elapsed =
        (this->get_clock()->now() - last_gas_data_time_.value()).seconds();
    if (elapsed < 5.0) {
      return;
    }
  }

  double base_concentration = 50.0;  // 正常背景浓度

  // 5% 概率模拟煤气泄漏事件
  if (randomProbability() < 0.05) {
    // 模拟厨房燃气泄漏
    std::uniform_real_distribution<double> dist(
        gas_warning_threshold_, gas_danger_threshold_ * 1.5);
    static thread_local std::mt19937 gen{std::random_device{}()};
    gas_concentration_ = dist(gen);
    RCLCPP_INFO(this->get_logger(),
                "[仿真] 模拟煤气泄漏事件，浓度: %.1f ppm", gas_concentration_);
  } else {
    // 正常浓度波动
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    static thread_local std::mt19937 gen{std::random_device{}()};
    gas_concentration_ = base_concentration + dist(gen);
  }

  checkGasLevel();
}

void SecurityNode::simulateSoundDetection() {
  // 仿真异常声音检测
  // 定时模拟声音检测，有一定概率检测到异常声音。
  // 实际部署时，应接入麦克风阵列和声音分类模型，检测：
  //   - 玻璃破碎声
  //   - 异常喊叫/哭声
  //   - 撬门/砸物声
  //   - 非正常时段的脚步声
  sound_check_counter_++;

  // 按概率模拟异常声音检测
  if (randomProbability() < sound_anomaly_probability_) {
    const std::string &sound_type =
        kSoundTypes[randomIndexStatic(kSoundTypes.size())];
    triggerAlert(AlertLevel::Medium,
                 "检测到异常声音，可能有人闯入（声音类型：" + sound_type + "）");
  }
}

// ===== 检测逻辑 =====

bool SecurityNode::isKnownFurniture(double angle, double distance) const {
  // 判断某点是否为已知家具
  // 通过角度和距离匹配已知家具位置，避免误报。
  for (const auto &furn : kKnownFurniture) {
    // 角度在家具角度范围内，且距离接近家具距离（容差 0.1m）
    if (furn.angle_min <= angle && angle <= furn.angle_max &&
        std::abs(distance - furn.distance) < 0.1) {
      return true;
    }
  }
  return false;
}

void SecurityNode::checkGasLevel() {
  // 检查煤气浓度并触发相应级别警报
  rclcpp::Time now = this->get_clock()->now();

  // 冷却期检查
  if (last_gas_alert_time_.has_value()) {
    double elapsed = (now - last_gas_alert_time_.value()).seconds();
    if (elapsed < gas_alert_cooldown_) {
      return;
    }
  }

  if (gas_concentration_ >= gas_danger_threshold_) {
    // 危险级别：煤气浓度严重超标
    last_gas_alert_time_ = now;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "烟雾浓度超标！请检查厨房（浓度: %.1f ppm）",
                  gas_concentration_);
    triggerAlert(AlertLevel::High, buf);
  } else if (gas_concentration_ >= gas_warning_threshold_) {
    // 警告级别：煤气浓度偏高
    last_gas_alert_time_ = now;
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "烟雾浓度偏高，请注意通风（浓度: %.1f ppm）",
                  gas_concentration_);
    triggerAlert(AlertLevel::Medium, buf);
  }
}

void SecurityNode::triggerIntrusionAlert(
    const std::vector<std::pair<double, double>> &intrusion_points) {
  // 触发入侵警报
  rclcpp::Time now = this->get_clock()->now();
  last_intrusion_time_ = now;
  intrusion_detected_ = true;

  // 发布入侵检测标志
  std_msgs::msg::Bool intrusion_msg;
  intrusion_msg.data = true;
  intrusion_pub_->publish(intrusion_msg);

  // 计算最近物体的距离
  double min_distance = intrusion_points[0].second;
  double sum_angle = 0.0;
  for (const auto &p : intrusion_points) {
    min_distance = std::min(min_distance, p.second);
    sum_angle += p.first;
  }
  double avg_angle = sum_angle / static_cast<double>(intrusion_points.size());
  std::string direction = angleToDirection(avg_angle);

  // 触发高级别警报
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "检测到异常入侵！位置：%s （距离: %.2fm，方位: %s）",
                current_location_.c_str(), min_distance, direction.c_str());
  triggerAlert(AlertLevel::High, buf, "intrusion", "security_node");
}

void SecurityNode::checkIntrusionReset(const rclcpp::Time &now) {
  // 检查是否需要解除入侵状态
  // 当持续一段时间未检测到入侵物体时，解除入侵标志。
  if (!intrusion_detected_) {
    return;
  }

  if (last_intrusion_time_.has_value()) {
    double elapsed = (now - last_intrusion_time_.value()).seconds();
    // 超过冷却时间 2 倍后解除入侵状态
    if (elapsed > intrusion_cooldown_ * 2.0) {
      intrusion_detected_ = false;
      std_msgs::msg::Bool intrusion_msg;
      intrusion_msg.data = false;
      intrusion_pub_->publish(intrusion_msg);
      publishStatusMsg("入侵警报解除，环境恢复正常");
    }
  }
}

std::string SecurityNode::angleToDirection(double angle) {
  // 角度转方位描述
  double angle_deg = angle * 180.0 / M_PI;
  if (-30 <= angle_deg && angle_deg <= 30) {
    return "正前方";
  } else if (30 < angle_deg && angle_deg <= 90) {
    return "右前方";
  } else if (-90 <= angle_deg && angle_deg < -30) {
    return "左前方";
  } else if (90 < angle_deg && angle_deg <= 150) {
    return "右后方";
  } else if (-150 <= angle_deg && angle_deg < -90) {
    return "左后方";
  } else {
    return "正后方";
  }
}

void SecurityNode::triggerAlert(AlertLevel level, const std::string &message,
                                const std::string &event_type,
                                const std::string &source_node) {
  // 触发分级警报
  // 统一的警报触发入口，负责：
  //   1. 发布警报文本到 /security/alert（兼容）
  //   2. P0-1: 发布语义消息 SecurityEvent 到 /security/event
  //   3. 记录警报到历史记录
  //   4. 发布语音通知到 /voice/tts
  //   5. 更新当前警报级别
  rclcpp::Time now = this->get_clock()->now();
  double timestamp = now.seconds();

  // 构造警报消息（包含级别前缀）
  std::string level_str = alertLevelToString(level);
  std::string alert_text = "[" + level_str + "级警报] " + message;

  // 发布警报（兼容 topic）
  std_msgs::msg::String alert_msg;
  alert_msg.data = alert_text;
  alert_pub_->publish(alert_msg);

  // P0-1: 发布语义消息 SecurityEvent 到 /security/event
  // 供 mission_manager 订阅，severity 映射: 高=critical / 中=warning / 低=info
  puppy_interfaces::msg::SecurityEvent sem;
  sem.header.stamp = now;
  sem.event_type = event_type;
  if (level == AlertLevel::High) {
    sem.severity = "critical";
  } else if (level == AlertLevel::Medium) {
    sem.severity = "warning";
  } else {
    sem.severity = "info";
  }
  sem.source_node = source_node;
  sem.message = message;
  security_event_pub_->publish(sem);

  // 记录警报到历史
  alert_records_.push_back({timestamp, level, message, current_location_});
  while (alert_records_.size() > max_alert_records_) {
    alert_records_.pop_front();
  }

  // 更新当前警报级别（取最高级别）
  if (static_cast<int>(level) > static_cast<int>(current_alert_level_)) {
    current_alert_level_ = level;
  }

  // 语音通知
  publishTts(message);

  // 日志输出（根据级别使用不同日志方法）
  if (level == AlertLevel::High) {
    RCLCPP_ERROR(this->get_logger(), "%s", alert_text.c_str());
  } else if (level == AlertLevel::Medium) {
    RCLCPP_WARN(this->get_logger(), "%s", alert_text.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "%s", alert_text.c_str());
  }
}

// ===== 状态发布 =====

void SecurityNode::publishStatusMsg(const std::string &message) {
  // 发布安防状态消息到 /security/status 话题
  std_msgs::msg::String status;
  status.data = message;
  status_pub_->publish(status);
}

void SecurityNode::publishStatus() {
  // 定时发布当前安防状态（由定时器调用）
  // 包含警报级别、位置、煤气浓度、入侵状态和警报统计信息。
  // 同时检查是否需要自动恢复警报级别。
  checkAlertLevelReset();

  // 统计各级别警报数量
  int high_count = 0, medium_count = 0, low_count = 0;
  for (const auto &r : alert_records_) {
    if (r.level == AlertLevel::High) {
      high_count++;
    } else if (r.level == AlertLevel::Medium) {
      medium_count++;
    } else if (r.level == AlertLevel::Low) {
      low_count++;
    }
  }

  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "[警报级别: %s] 位置: %s | 煤气: %.1f ppm | 入侵: %s | "
                "警报记录: 高%d/中%d/低%d",
                alertLevelToString(current_alert_level_).c_str(),
                current_location_.c_str(), gas_concentration_,
                intrusion_detected_ ? "是" : "否", high_count, medium_count,
                low_count);
  publishStatusMsg(buf);
}

void SecurityNode::checkAlertLevelReset() {
  // 检查是否需要将警报级别恢复为正常
  // 当超过 alert_reset_duration 秒无新警报时，恢复为正常级别。
  if (alert_records_.empty()) {
    return;
  }

  double current_time = this->get_clock()->now().seconds();
  double last_alert_time = alert_records_.back().timestamp;

  if (current_time - last_alert_time > alert_reset_duration_) {
    if (current_alert_level_ != AlertLevel::Normal) {
      current_alert_level_ = AlertLevel::Normal;
      publishStatusMsg("警报级别已恢复为正常");
    }
  }
}

void SecurityNode::publishTts(const std::string &text) {
  // 发布语音合成文本到 /voice/tts 话题
  std_msgs::msg::String tts_msg;
  tts_msg.data = text;
  tts_pub_->publish(tts_msg);
}

}  // namespace puppy_gait
