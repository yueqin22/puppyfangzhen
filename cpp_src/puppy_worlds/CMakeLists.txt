cmake_minimum_required(VERSION 3.8)
project(puppy_worlds_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(ROS_DEPS
  rclcpp
  nav_msgs
  rosgraph_msgs
  builtin_interfaces
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_executable(clock_publisher clock_publisher.cpp)
target_include_directories(clock_publisher PRIVATE ${INCLUDE_ROOT})
ament_target_dependencies(clock_publisher ${ROS_DEPS})
install(TARGETS clock_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
