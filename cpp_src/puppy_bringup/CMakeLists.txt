cmake_minimum_required(VERSION 3.8)
project(puppy_bringup_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(ROS_DEPS
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  rosgraph_msgs
  builtin_interfaces
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/NodeTarget.cmake")

add_ros_node_from_class(clock_publisher clock_publisher.cpp "puppy_bringup/clock_publisher.h" "puppy_bringup::ClockPublisherNode")
add_ros_node_from_class(odom_tf_publisher odom_tf_publisher.cpp "puppy_bringup/odom_tf_publisher.h" "puppy_bringup::OdomTfPublisherNode")

set(BRINGUP_TARGETS clock_publisher odom_tf_publisher)

find_path(COPPELIASIM_ZMQ_INCLUDE_DIR
  NAMES coppeliaSimZMQ.hpp
  HINTS
    "$ENV{COPPELIASIM_ROOT}/programming/zmqRemoteApi/clients/cpp"
    "$ENV{COPPELIASIM_ROOT}/programming/zmqRemoteApi/clients/cpp/include"
)

if(COPPELIASIM_ZMQ_INCLUDE_DIR)
  add_ros_node_from_class(coppelia_bridge coppelia_bridge.cpp "puppy_bringup/coppelia_bridge.h" "puppy_bringup::CoppeliaBridgeNode")
  target_include_directories(coppelia_bridge PRIVATE "${COPPELIASIM_ZMQ_INCLUDE_DIR}")
  list(APPEND BRINGUP_TARGETS coppelia_bridge)
else()
  message(WARNING
    "coppeliaSimZMQ.hpp was not found; skipping coppelia_bridge. "
    "Set COPPELIASIM_ROOT or CMAKE_INCLUDE_PATH to build the CoppeliaSim bridge.")
endif()

foreach(target ${BRINGUP_TARGETS})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
