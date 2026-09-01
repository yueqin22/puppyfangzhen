# 顶层 CMakeLists.txt — 机器狗自主导航仿真系统
# 仅构建 C++ 仿真器（sim_test），不依赖 ROS2
cmake_minimum_required(VERSION 3.8)
project(puppy_nav_sim CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认 Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 仿真器子目录
add_subdirectory(sim)

# 单元测试（可选）
option(BUILD_TESTING "Build unit tests" OFF)
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
