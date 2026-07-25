cmake_minimum_required(VERSION 3.8)
project(puppy_core_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(puppy_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ROS_DEPS
  rclcpp
  nav2_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  action_msgs
  rclcpp_action
  std_srvs
  puppy_interfaces
  ament_index_cpp
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/NodeTarget.cmake")

set(UTILITY_SOURCES
  capability_registry.cpp
  config_utils.cpp
)

add_library(${PROJECT_NAME} STATIC ${UTILITY_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_ROOT}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen yaml-cpp)

add_ros_node_from_class(nav_core_node nav_core_node.cpp "puppy_core/nav_core_node.h" "puppy_core::NavCoreNode")
add_ros_node_from_class(capability_registry capability_registry_node.cpp "puppy_core/capability_registry_node.h" "puppy_core::CapabilityRegistryNode")
add_ros_node_from_class(watchdog watchdog.cpp "puppy_core/watchdog.h" "puppy_core::WatchdogNode")
add_ros_node_from_class(safety_manager safety_manager.cpp "puppy_core/safety_manager.h" "puppy_core::SafetyManagerNode")
add_ros_node_from_class(mode_manager mode_manager.cpp "puppy_core/mode_manager.h" "puppy_core::ModeManagerNode")
add_ros_node_from_class(mission_manager mission_manager.cpp "puppy_core/mission_manager.h" "puppy_core::MissionManagerNode")
add_ros_node_from_class(goal_dispatcher goal_dispatcher.cpp "puppy_core/goal_dispatcher.h" "puppy_core::GoalDispatcherNode")
add_ros_node_from_class(robot_state_aggregator robot_state_aggregator.cpp "puppy_core/robot_state_aggregator.h" "puppy_core::RobotStateAggregatorNode")
add_ros_node_from_class(battery_status_adapter battery_status_adapter.cpp "puppy_core/battery_status_adapter.h" "puppy_core::BatteryStatusAdapterNode")
add_ros_node_from_class(fall_event_adapter fall_event_adapter.cpp "puppy_core/fall_event_adapter.h" "puppy_core::FallEventAdapterNode")

add_executable(dock_detector dock_detector.cpp)
target_include_directories(dock_detector PRIVATE ${INCLUDE_ROOT})
ament_target_dependencies(dock_detector ${ROS_DEPS})

foreach(target
  nav_core_node capability_registry watchdog safety_manager mode_manager
  mission_manager goal_dispatcher robot_state_aggregator battery_status_adapter
  fall_event_adapter dock_detector)
  target_link_libraries(${target} ${PROJECT_NAME} Eigen3::Eigen yaml-cpp)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${ROS_DEPS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
