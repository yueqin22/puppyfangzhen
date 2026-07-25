cmake_minimum_required(VERSION 3.8)
project(puppy_nav_core)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/O2 /std:c++17 /EHsc /utf-8)
else()
    add_compile_options(-O2 -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# v3.2: 主算法库 (含 TEB header-only)
add_library(puppy_nav_core_cpp STATIC
    src/occupancy_grid.cpp
    src/costmap.cpp
    src/astar_planner.cpp
    src/amcl.cpp
)
target_include_directories(puppy_nav_core_cpp PUBLIC include)
target_compile_definitions(puppy_nav_core_cpp PRIVATE _CRT_SECURE_NO_WARNINGS)
ament_target_dependencies(puppy_nav_core_cpp rclcpp)

install(TARGETS puppy_nav_core_cpp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# v3.2: ROS2 Lifecycle 节点 (项目硬性约束: full lifecycle management)
add_executable(nav_core_lifecycle_node src/nav_core_lifecycle_main.cpp)
target_include_directories(nav_core_lifecycle_node PRIVATE include)
target_link_libraries(nav_core_lifecycle_node puppy_nav_core_cpp)
ament_target_dependencies(nav_core_lifecycle_node
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_srvs
)
install(TARGETS nav_core_lifecycle_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    add_executable(test_nav_core
        test/test_nav_core.cpp
    )
    target_include_directories(test_nav_core PRIVATE include)
    target_link_libraries(test_nav_core puppy_nav_core_cpp)

    add_executable(step_test
        test/step_test.cpp
    )
    target_include_directories(step_test PRIVATE include)
    target_link_libraries(step_test puppy_nav_core_cpp)

    add_executable(debug_test
        test/debug_test.cpp
    )
    target_include_directories(debug_test PRIVATE include)
    target_link_libraries(debug_test puppy_nav_core_cpp)

    # v3.2: TEB 单元测试
    add_executable(test_teb_planner
        test/test_teb_planner.cpp
    )
    target_include_directories(test_teb_planner PRIVATE include)
    target_link_libraries(test_teb_planner puppy_nav_core_cpp)
endif()

ament_package()
