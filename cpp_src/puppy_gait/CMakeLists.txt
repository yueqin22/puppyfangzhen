cmake_minimum_required(VERSION 3.8)
project(puppy_gait_cpp)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(puppy_interfaces REQUIRED)

set(ROS_DEPS
  rclcpp
  nav2_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  rclcpp_action
  action_msgs
  std_srvs
  ament_index_cpp
  puppy_interfaces
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/NodeTarget.cmake")

add_library(${PROJECT_NAME} STATIC shared_targets.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_ROOT}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPS})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_ros_node_from_class(auto_cruise auto_cruise.cpp "puppy_gait/auto_cruise.h" "puppy_gait::AutoCruiseNode")
add_ros_node_from_class(battery_simulator battery_simulator.cpp "puppy_gait/battery_simulator.h" "puppy_gait::BatterySimulatorNode")
add_ros_node_from_class(emergency_response emergency_response.cpp "puppy_gait/emergency_response.h" "puppy_gait::EmergencyResponseNode")
add_ros_node_from_class(emotion_interaction emotion_interaction.cpp "puppy_gait/emotion_interaction.h" "puppy_gait::EmotionInteractionNode")
add_ros_node_from_class(fall_detection fall_detection.cpp "puppy_gait/fall_detection.h" "puppy_gait::FallDetectionNode")
add_ros_node_from_class(patrol_mission patrol_mission.cpp "puppy_gait/patrol_mission.h" "puppy_gait::PatrolMissionNode")
add_ros_node_from_class(security_node security_node.cpp "puppy_gait/security_node.h" "puppy_gait::SecurityNode")
add_ros_node_from_class(smart_home_manager smart_home_manager.cpp "puppy_gait/smart_home_manager.h" "puppy_gait::SmartHomeManagerNode")
add_ros_node_from_class(task_scheduler task_scheduler.cpp "puppy_gait/task_scheduler.h" "puppy_gait::TaskSchedulerNode")

foreach(target
  auto_cruise battery_simulator emergency_response emotion_interaction
  fall_detection patrol_mission security_node smart_home_manager task_scheduler)
  target_link_libraries(${target} ${PROJECT_NAME} yaml-cpp)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${ROS_DEPS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
