// 智能家居家电管理节点（C++ 版本）
//
// - 根据机器人位置自动控制灯光（人到灯亮，人走灯灭）
// - 支持手动命令控制家电
// - 发布家电状态到 /home_appliance/status
// - 语音播报到 /voice/tts
//
// 家电列表:
//   客厅灯、卧室灯、厨房灯、走廊灯、门口灯
//   客厅空调、卧室空调
//   客厅窗帘、卧室窗帘
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace puppy_gait {

/// 家电描述
struct Appliance {
  std::string key;
  std::string name;
  std::string type;   // light / ac / curtain
  bool on{false};
  std::string room;
  int temp{0};        // 仅 ac 使用
};

/// 房间区域定义 (x_min, x_max, y_min, y_max)
struct RoomArea {
  std::string name;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

/// 智能家居家电管理节点
class SmartHomeManagerNode : public rclcpp::Node {
 public:
  SmartHomeManagerNode();
  ~SmartHomeManagerNode() override = default;

 private:
  // 根据机器人位置判断所在房间，控制灯光
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  // 处理手动家电控制命令
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg);
  // 接收定时任务事件
  void taskEventCallback(const std_msgs::msg::String::SharedPtr msg);
  // 低电量时关闭非必要家电
  void batteryCallback(const std_msgs::msg::Bool::SharedPtr msg);
  // 定时发布家电状态 (JSON)
  void publishStatus();
  // 检查是否需要关闭已离开房间的灯
  void checkLightTimeout();

  // 根据坐标判断所在房间
  std::string getRoom(double x, double y) const;
  // 房间切换时的处理
  void onRoomChange(const std::string &old_room, const std::string &new_room);
  // 打开指定房间的所有灯
  void turnOnRoomLights(const std::string &room);
  // 关闭指定房间的所有灯
  void turnOffRoomLights(const std::string &room);
  // 发布语音合成文本
  void publishVoice(const std::string &text);

 private:
  // ===== 参数 =====
  bool auto_light_{true};
  double light_off_delay_{10.0};

  // ===== 状态 =====
  std::string current_room_{"未知"};
  std::string last_room_{"未知"};
  std::map<std::string, double> room_leave_time_;  // 房间离开时间
  bool night_mode_{false};

  // 家电列表
  std::vector<Appliance> appliances_;

  // ===== 订阅 =====
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_event_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_sub_;

  // ===== 发布 =====
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_pub_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr light_timeout_timer_;
};

}  // namespace puppy_gait
