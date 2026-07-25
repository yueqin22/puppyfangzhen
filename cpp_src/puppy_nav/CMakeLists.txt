cmake_minimum_required(VERSION 3.8)
project(puppy_nav_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(puppy_interfaces REQUIRED)

set(ROS_DEPS
  rclcpp
  rclcpp_action
  action_msgs
  nav2_msgs
  std_msgs
  puppy_interfaces
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

foreach(target simple_patrol patrol)
  add_executable(${target} ${target}.cpp)
  target_include_directories(${target} PRIVATE ${INCLUDE_ROOT})
  ament_target_dependencies(${target} ${ROS_DEPS})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()

