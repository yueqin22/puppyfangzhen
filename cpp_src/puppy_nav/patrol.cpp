// 巡逻触发器（thin client）实现
//
// 通过 mission_manager 触发巡逻并监控状态
#include "puppy_nav/patrol.h"

#include <algorithm>
#include <chrono>

namespace puppy_nav {

PatrolTriggerNode::PatrolTriggerNode(const std::vector<std::string>& extra_args)
    : rclcpp::Node("patrol_trigger") {
  command_pub_ =
      this->create_publisher<std_msgs::msg::String>("/mission/command", 10);
  status_sub_ = this->create_subscription<puppy_interfaces::msg::PatrolStatus>(
      "/mission/status", 10,
      std::bind(&PatrolTriggerNode::onStatus, this, std::placeholders::_1));

  triggered_ = false;

  // 等价 Python: cmd = 'stop' if '--stop' in sys.argv else 'patrol'
  // 检测 --stop 参数
  stop_mode_ = false;
  for (const auto& arg : extra_args) {
    if (arg == "--stop") {
      stop_mode_ = true;
      break;
    }
  }

  // 等待 publisher 建立连接后发送命令（1.5 秒）
  send_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1500),
      std::bind(&PatrolTriggerNode::sendCommand, this));
}

void PatrolTriggerNode::sendCommand() {
  if (triggered_) {
    return;
  }
  // 支持 --stop 参数停止巡逻
  std::string cmd = stop_mode_ ? "stop" : "patrol";
  std_msgs::msg::String msg;
  msg.data = cmd;
  command_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Sent mission command: %s", cmd.c_str());
  triggered_ = true;

  // 等价 Python: if cmd == 'stop': self.create_timer(2.0, self._shutdown)
  if (cmd == "stop") {
    // 停止命令发送后即可退出
    send_timer_->cancel();
    shutdown_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&PatrolTriggerNode::shutdown, this));
  } else {
    send_timer_->cancel();
  }
}

void PatrolTriggerNode::onStatus(
    const puppy_interfaces::msg::PatrolStatus::SharedPtr msg) {
  // 等价 Python:
  //   f'[mission] state={msg.state} progress={msg.completion_ratio:.0%} '
  //   f'wp={msg.current_waypoint_index}/{msg.total_waypoints} {msg.message}'
  RCLCPP_INFO(this->get_logger(),
              "[mission] state=%s progress=%.0f%% wp=%d/%d %s",
              msg->state.c_str(),
              msg->completion_ratio * 100.0,
              msg->current_waypoint_index,
              msg->total_waypoints,
              msg->message.c_str());
}

void PatrolTriggerNode::shutdown() {
  // 等价 Python: raise SystemExit(0)
  // 在 C++ 中通过请求 shutdown 来退出 spin
  if (shutdown_timer_) {
    shutdown_timer_->cancel();
  }
  rclcpp::shutdown();
}

}  // namespace puppy_nav

// ===== 节点入口 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 收集命令行参数（等价 Python: sys.argv）
  std::vector<std::string> extra_args;
  for (int i = 1; i < argc; ++i) {
    extra_args.push_back(argv[i]);
  }

  rclcpp::spin(std::make_shared<puppy_nav::PatrolTriggerNode>(extra_args));
  rclcpp::shutdown();
  return 0;
}
