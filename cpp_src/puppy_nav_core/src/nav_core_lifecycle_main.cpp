// nav_core_lifecycle_main.cpp — puppy_nav_core ROS2 Lifecycle 节点入口
// 编译 (ROS2 环境): colcon build --packages-select puppy_nav_core
// 运行: ros2 run puppy_nav_core nav_core_lifecycle_node
//
// Lifecycle 控制:
//   ros2 lifecycle set /puppy_nav_core configure
//   ros2 lifecycle set /puppy_nav_core activate
//   ros2 lifecycle set /puppy_nav_core deactivate
//   ros2 lifecycle set /puppy_nav_core cleanup
//
// 项目硬性约束:
//   "ROS2 nodes must implement full lifecycle management
//    (on_configure/on_activate/on_deactivate/on_cleanup)"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "puppy_nav_core/nav_core_lifecycle_node.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puppy_nav_core::NavCoreLifecycleNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
