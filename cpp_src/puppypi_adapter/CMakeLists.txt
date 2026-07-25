cmake_minimum_required(VERSION 3.8)
project(puppypi_adapter_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(puppy_interfaces REQUIRED)

set(ROS_DEPS
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  builtin_interfaces
  puppy_interfaces
)

set(INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/NodeTarget.cmake")

set(UTILITY_SOURCES
  fault_injection.cpp
  hardware_interface.cpp
  mock_interface.cpp
  performance_monitor.cpp
  puppypi_driver.cpp
  safety_subsystem_v2.cpp
  sensor_calibration.cpp
)

add_library(${PROJECT_NAME} STATIC ${UTILITY_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_ROOT}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPS})

add_executable(motion_adapter motion_adapter_node.cpp)
target_include_directories(motion_adapter PRIVATE ${INCLUDE_ROOT})
ament_target_dependencies(motion_adapter ${ROS_DEPS})
target_link_libraries(motion_adapter ${PROJECT_NAME})

add_executable(mode_adapter mode_adapter_node.cpp)
target_include_directories(mode_adapter PRIVATE ${INCLUDE_ROOT})
ament_target_dependencies(mode_adapter ${ROS_DEPS})
target_link_libraries(mode_adapter ${PROJECT_NAME})

add_ros_node_from_class(status_adapter status_adapter_node.cpp "puppypi_adapter/status_adapter_node.h" "puppypi_adapter::StatusAdapterNode")
target_link_libraries(status_adapter ${PROJECT_NAME})

foreach(target motion_adapter status_adapter mode_adapter)
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
