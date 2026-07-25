// 简单巡逻脚本 - 家庭环境版 实现
//
// 使用 Nav2 NavigateToPose action client 依次导航到预设航点，循环巡逻
#include "puppy_nav/simple_patrol.h"

#include <cmath>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppy_nav {

PatrolBot::PatrolBot() : rclcpp::Node("patrol_bot") {
  client_ = rclcpp_action::create_client<NavigateToPose>(this,
                                                         "/navigate_to_pose");
}

bool PatrolBot::navigateTo(double x, double y, double yaw, double timeout) {
  using namespace std::chrono_literals;

  // 等价 Python:
  //   goal_msg = NavigateToPose.Goal()
  //   goal_msg.pose.header.frame_id = 'map'
  //   goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
  //   goal_msg.pose.pose.position.x = float(x)
  //   goal_msg.pose.pose.position.y = float(y)
  //   goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
  //   goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  // 简化 yaw -> 四元数（仅 z 和 w 分量）
  goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(this->get_logger(), "导航到 (%.2f, %.2f)...", x, y);

  // 等价 Python: self.client.wait_for_server(timeout_sec=15.0)
  if (!client_->wait_for_action_server(15s)) {
    RCLCPP_WARN(this->get_logger(), "Action server 不可用");
    return false;
  }

  // 等价 Python: future = self.client.send_goal_async(goal_msg)
  auto goal_future = client_->async_send_goal(goal_msg);

  // 等价 Python: rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
  auto goal_status = goal_future.wait_for(10s);
  if (goal_status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "发送 goal 超时");
    return false;
  }

  auto goal_handle = goal_future.get();
  // 等价 Python: if not future.result() or not future.result().accepted
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "目标被拒绝");
    return false;
  }

  // 等价 Python: result_future = future.result().get_result_async()
  auto result_future = client_->async_get_result(goal_handle);
  auto start = std::chrono::steady_clock::now();

  // 等价 Python:
  //   while not result_future.done():
  //       rclpy.spin_once(self, timeout_sec=0.5)
  //       if time.time() - start > timeout: return False
  while (true) {
    auto status = result_future.wait_for(500ms);
    if (status == std::future_status::ready) {
      break;
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    if (elapsed > timeout) {
      RCLCPP_WARN(this->get_logger(), "超时!");
      return false;
    }
    if (!rclcpp::ok()) {
      return false;
    }
  }

  auto result = result_future.get();
  // 等价 Python: status = result_future.result().status; if status == 4: SUCCEEDED
  // 在 C++ 中，rclcpp_action::ResultCode::SUCCEEDED 对应 Python 的 status==4
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    RCLCPP_INFO(this->get_logger(), "到达! (%.1fs)", elapsed);
    return true;
  }
  RCLCPP_INFO(this->get_logger(), "状态=%d",
              static_cast<int>(result.code));
  return false;
}

}  // namespace puppy_nav

// ===== 节点入口 =====
// 等价 Python: main() 函数，包含 waypoints 和无限循环
int main(int argc, char** argv) {
  // 等价 Python: rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
  // 在 C++ 中通过命令行参数传递 use_sim_time
  rclcpp::init(argc, argv);
  auto bot = std::make_shared<puppy_nav::PatrolBot>();

  // 家庭环境航点（home.world 10m×8m）
  struct Waypoint {
    double x, y, yaw;
  };
  std::vector<Waypoint> waypoints = {
      {2.5, -2.0, 0.0},          // 客厅-沙发
      {4.0, -1.0, M_PI / 2},     // 客厅-电视
      {0.0, 1.5, M_PI / 2},      // 走廊
      {-3.0, 2.5, M_PI},         // 卧室-床
      {3.0, 2.5, 0.0},           // 厨房-餐桌
      {-1.0, -2.5, 0.0},         // 充电桩
  };

  RCLCPP_INFO(bot->get_logger(), "巡逻开始! %zu 个航点", waypoints.size());

  int round_num = 1;
  try {
    while (rclcpp::ok()) {
      RCLCPP_INFO(bot->get_logger(), "--- 第 %d 轮 ---", round_num);
      for (size_t i = 0; i < waypoints.size(); ++i) {
        if (!rclcpp::ok()) break;
        bot->navigateTo(waypoints[i].x, waypoints[i].y, waypoints[i].yaw);
        // 等价 Python: time.sleep(1)
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      round_num++;
    }
  } catch (const std::exception& e) {
    // 等价 Python: except (KeyboardInterrupt, Exception): pass
    RCLCPP_WARN(bot->get_logger(), "巡逻异常: %s", e.what());
  }

  try {
    bot.reset();
    rclcpp::shutdown();
  } catch (...) {
    // 等价 Python: except Exception: pass
  }
  return 0;
}
