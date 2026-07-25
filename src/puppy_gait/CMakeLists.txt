cmake_minimum_required(VERSION 3.8)
project(puppy_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(gait_controller
  src/gait_controller.cpp
  src/trot_gait.cpp
  src/inverse_kinematics.cpp
)
target_include_directories(gait_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(gait_controller PUBLIC cxx_std_17)
ament_target_dependencies(gait_controller
  rclcpp geometry_msgs sensor_msgs trajectory_msgs std_msgs tf2 tf2_ros
)

install(TARGETS gait_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fall_detection.py
  scripts/security_node.py
  scripts/emotion_interaction.py
  scripts/auto_cruise.py
  scripts/patrol_mission.py
  scripts/battery_simulator.py
  scripts/smart_home_manager.py
  scripts/task_scheduler.py
  scripts/emergency_response.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
