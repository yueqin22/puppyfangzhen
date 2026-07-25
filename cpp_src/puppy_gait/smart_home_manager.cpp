// 智能家居家电管理节点实现
#include "puppy_gait/smart_home_manager.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>

namespace puppy_gait {

// ===== 房间区域定义 (x_min, x_max, y_min, y_max) =====
// 注意：区域之间共享边界线（不重叠），按顺序优先匹配
static const std::vector<RoomArea> kRoomAreas = {
    {"客厅", 0.5, 4.5, -3.5, -0.3},
    {"充电桩", -1.5, 0.5, -3.5, -1.5},
    {"门口", -0.5, 0.5, -1.5, 0.0},
    {"走廊", -0.8, 0.8, 0.0, 2.5},
    {"卧室", -4.5, -0.8, 0.0, 3.5},
    {"厨房", 0.8, 4.5, 0.0, 3.5},
};

SmartHomeManagerNode::SmartHomeManagerNode()
    : rclcpp::Node("smart_home_manager") {
  // ===== 参数 =====
  this->declare_parameter("auto_light", true);
  this->declare_parameter("light_off_delay", 10.0);
  auto_light_ = this->get_parameter("auto_light").as_bool();
  light_off_delay_ = this->get_parameter("light_off_delay").as_double();

  // ===== 状态 =====
  current_room_ = "未知";
  last_room_ = "未知";
  night_mode_ = false;

  // ===== 家电初始状态 =====
  appliances_ = {
      {"living_room_light", "客厅灯", "light", false, "客厅", 0},
      {"bedroom_light", "卧室灯", "light", false, "卧室", 0},
      {"kitchen_light", "厨房灯", "light", false, "厨房", 0},
      {"hallway_light", "走廊灯", "light", false, "走廊", 0},
      {"door_light", "门口灯", "light", false, "门口", 0},
      {"charging_light", "充电桩灯", "light", false, "充电桩", 0},
      {"living_room_ac", "客厅空调", "ac", false, "客厅", 26},
      {"bedroom_ac", "卧室空调", "ac", false, "卧室", 25},
      {"living_room_curtain", "客厅窗帘", "curtain", false, "客厅", 0},
      {"bedroom_curtain", "卧室窗帘", "curtain", false, "卧室", 0},
  };

  // ===== 订阅 =====
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/amcl_pose", 10,
      std::bind(&SmartHomeManagerNode::poseCallback, this,
                std::placeholders::_1));
  cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/home_appliance/cmd", 10,
      std::bind(&SmartHomeManagerNode::cmdCallback, this,
                std::placeholders::_1));
  task_event_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/task_scheduler/event", 10,
      std::bind(&SmartHomeManagerNode::taskEventCallback, this,
                std::placeholders::_1));
  battery_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/low_battery_alert", 10,
      std::bind(&SmartHomeManagerNode::batteryCallback, this,
                std::placeholders::_1));

  // ===== 发布 =====
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/home_appliance/status", 10);
  voice_pub_ = this->create_publisher<std_msgs::msg::String>("/voice/tts", 10);
  light_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/home_appliance/light_state", 10);

  // ===== 定时器 =====
  status_timer_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&SmartHomeManagerNode::publishStatus, this));
  light_timeout_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SmartHomeManagerNode::checkLightTimeout, this));

  RCLCPP_INFO(this->get_logger(), "智能家居家电管理节点已启动");
  RCLCPP_INFO(this->get_logger(), "自动灯控: %s, 关灯延迟: %.1f秒",
              auto_light_ ? "开启" : "关闭", light_off_delay_);
}

void SmartHomeManagerNode::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // 根据机器人位置判断所在房间，控制灯光
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  current_room_ = getRoom(x, y);

  if (current_room_ != last_room_) {
    onRoomChange(last_room_, current_room_);
    last_room_ = current_room_;
  }
}

std::string SmartHomeManagerNode::getRoom(double x, double y) const {
  // 根据坐标判断所在房间
  for (const auto &area : kRoomAreas) {
    if (area.x_min <= x && x <= area.x_max && area.y_min <= y &&
        y <= area.y_max) {
      return area.name;
    }
  }
  return "未知";
}

void SmartHomeManagerNode::onRoomChange(const std::string &old_room,
                                       const std::string &new_room) {
  // 房间切换时的处理
  double now = this->get_clock()->now().seconds();

  if (new_room != "未知") {
    // 人到灯亮
    if (auto_light_ && !night_mode_) {
      turnOnRoomLights(new_room);
      publishVoice("欢迎来到" + new_room + "，灯已为您打开");
    } else if (auto_light_ && night_mode_ && new_room == "走廊") {
      // 夜间模式下走廊灯作为夜灯始终开启
      turnOnRoomLights(new_room);
    } else if (new_room == "充电桩" && auto_light_) {
      // 充电桩区域始终开灯（充电指示灯）
      turnOnRoomLights(new_room);
    }
  }

  if (old_room != "未知" && old_room != new_room) {
    room_leave_time_[old_room] = now;
    // 不立即关灯，等待延迟（在 checkLightTimeout 中处理）
  }
}

void SmartHomeManagerNode::turnOnRoomLights(const std::string &room) {
  // 打开指定房间的所有灯
  for (auto &appliance : appliances_) {
    if (appliance.room == room && appliance.type == "light") {
      if (!appliance.on) {
        appliance.on = true;
        RCLCPP_INFO(this->get_logger(), "开启: %s", appliance.name.c_str());
        std_msgs::msg::Bool msg;
        msg.data = true;
        light_pub_->publish(msg);
      }
    }
  }
}

void SmartHomeManagerNode::turnOffRoomLights(const std::string &room) {
  // 关闭指定房间的所有灯
  for (auto &appliance : appliances_) {
    if (appliance.room == room && appliance.type == "light") {
      if (appliance.on) {
        appliance.on = false;
        RCLCPP_INFO(this->get_logger(), "关闭: %s", appliance.name.c_str());
        std_msgs::msg::Bool msg;
        msg.data = false;
        light_pub_->publish(msg);
      }
    }
  }
}

void SmartHomeManagerNode::checkLightTimeout() {
  // 检查是否需要关闭已离开房间的灯
  double now = this->get_clock()->now().seconds();
  for (auto it = room_leave_time_.begin(); it != room_leave_time_.end();) {
    if (it->first == current_room_) {
      ++it;  // 当前房间不关灯
      continue;
    }
    if (now - it->second >= light_off_delay_) {
      turnOffRoomLights(it->first);
      it = room_leave_time_.erase(it);
    } else {
      ++it;
    }
  }
}

void SmartHomeManagerNode::cmdCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  // 处理手动家电控制命令
  // 命令格式: "on:客厅灯" / "off:卧室空调" / "all_off" / "all_on"
  std::string cmd = msg->data;
  RCLCPP_INFO(this->get_logger(), "收到家电命令: %s", cmd.c_str());

  if (cmd == "all_off") {
    for (auto &appliance : appliances_) {
      appliance.on = false;
    }
    publishVoice("所有家电已关闭");
    return;
  }

  if (cmd == "all_on") {
    for (auto &appliance : appliances_) {
      appliance.on = true;
    }
    publishVoice("所有家电已开启");
    return;
  }

  if (cmd == "night_mode") {
    night_mode_ = true;
    for (auto &appliance : appliances_) {
      if (appliance.type == "curtain") {
        appliance.on = false;  // 拉上窗帘
      }
      if (appliance.type == "ac") {
        appliance.on = false;
      }
    }
    publishVoice("夜间模式已开启，窗帘已关闭，空调已关闭");
    return;
  }

  if (cmd == "day_mode") {
    night_mode_ = false;
    for (auto &appliance : appliances_) {
      if (appliance.type == "curtain") {
        appliance.on = true;  // 打开窗帘
      }
    }
    publishVoice("日间模式已开启，窗帘已打开");
    return;
  }

  // 单个设备控制: on:设备名 / off:设备名
  size_t colon = cmd.find(':');
  if (colon == std::string::npos) {
    RCLCPP_WARN(this->get_logger(), "命令格式错误: %s", cmd.c_str());
    return;
  }
  std::string action = cmd.substr(0, colon);
  std::string name = cmd.substr(colon + 1);
  bool found = false;
  for (auto &appliance : appliances_) {
    if (appliance.name.find(name) != std::string::npos || name == appliance.key) {
      appliance.on = (action == "on");
      if (appliance.type == "ac" && action == "on") {
        publishVoice(appliance.name + "已开启，温度" +
                     std::to_string(appliance.temp) + "度");
      } else {
        std::string state = (action == "on") ? "开启" : "关闭";
        publishVoice(appliance.name + "已" + state);
      }
      found = true;
      break;
    }
  }
  if (!found) {
    publishVoice("未找到设备: " + name);
  }
}

void SmartHomeManagerNode::taskEventCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  // 接收定时任务事件
  const std::string &event = msg->data;
  RCLCPP_INFO(this->get_logger(), "收到定时任务事件: %s", event.c_str());

  if (event == "morning_patrol") {
    // 早安：打开所有窗帘，客厅灯
    night_mode_ = false;
    for (auto &appliance : appliances_) {
      if (appliance.type == "curtain") {
        appliance.on = true;
      }
    }
    publishVoice("早安！窗帘已打开，新的一天开始了");

  } else if (event == "night_patrol") {
    // 晚安：关闭窗帘，关闭空调，只留走廊灯
    night_mode_ = true;
    for (auto &appliance : appliances_) {
      if (appliance.type == "curtain") {
        appliance.on = false;
      }
      if (appliance.type == "ac") {
        appliance.on = false;
      }
      if (appliance.type == "light" && appliance.room != "走廊") {
        appliance.on = false;
      }
    }
    publishVoice("夜间安防模式已开启，窗帘已关闭，只保留走廊灯");

  } else if (event == "noon_check") {
    // 午间：开启客厅空调
    for (auto &appliance : appliances_) {
      if (appliance.key == "living_room_ac") {
        appliance.on = true;
      }
    }
    publishVoice("午间检查，客厅空调已开启");
  }
}

void SmartHomeManagerNode::batteryCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  // 低电量时关闭非必要家电
  if (msg->data) {
    for (auto &appliance : appliances_) {
      if (appliance.type == "ac") {
        appliance.on = false;
      }
    }
    RCLCPP_INFO(this->get_logger(), "低电量警报：已关闭空调等大功率设备");
  }
}

void SmartHomeManagerNode::publishStatus() {
  // 发布家电状态 (JSON)
  std::ostringstream oss;
  int32_t sec = static_cast<int32_t>(this->get_clock()->now().nanoseconds() / 1000000000LL);
  oss << "{\"timestamp\":" << sec
      << ",\"current_room\":\"" << current_room_ << "\""
      << ",\"night_mode\":" << (night_mode_ ? "true" : "false")
      << ",\"appliances\":{";
  bool first = true;
  for (const auto &a : appliances_) {
    if (!first) oss << ",";
    first = false;
    oss << "\"" << a.key << "\":{"
        << "\"name\":\"" << a.name << "\""
        << ",\"type\":\"" << a.type << "\""
        << ",\"on\":" << (a.on ? "true" : "false")
        << ",\"room\":\"" << a.room << "\"}";
  }
  oss << "}}";

  std_msgs::msg::String status;
  status.data = oss.str();
  status_pub_->publish(status);
}

void SmartHomeManagerNode::publishVoice(const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  voice_pub_->publish(msg);
}

}  // namespace puppy_gait
