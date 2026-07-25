cmake_minimum_required(VERSION 3.8)
project(puppypi_mock_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(puppy_interfaces REQUIRED)

set(ROS_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
  puppy_interfaces
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_executable(fake_motion fake_motion_server.cpp)
add_executable(fake_status fake_status_server.cpp)
add_executable(fake_sensors fake_sensor_publishers.cpp)

foreach(target fake_motion fake_status fake_sensors)
  target_include_directories(${target} PRIVATE ${INCLUDE_ROOT})
  ament_target_dependencies(${target} ${ROS_DEPS})
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
