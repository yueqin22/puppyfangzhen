# CMakeLists.txt — nav_ue_bridge (UE↔Nav 桥接进程)
# ================================================================
# 构建独立桥接进程, 通过 TCP 与 UE 通信
# 依赖: puppy_nav_core (静态库) + nav_bridge.h (头文件)
cmake_minimum_required(VERSION 3.10)
project(nav_ue_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC: /utf-8 必须加——源文件含中文注释和 Unicode 箭头(→↔)，
#   不加会导致 code page 936 误解析 #pragma once 和 namespace，引发重定义/语法错误
#   NOMINMAX: winsock2.h 的 min/max 宏会干扰 std::min/std::max
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3 /EHsc /utf-8 /MT")
    add_compile_definitions(NOMINMAX)
endif()

# 包含路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../puppy_nav_core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sim
)

# 源文件
add_executable(nav_ue_bridge
    nav_ue_bridge.cpp
    ../puppy_nav_core/src/occupancy_grid.cpp
    ../puppy_nav_core/src/costmap.cpp
    ../puppy_nav_core/src/astar_planner.cpp
    ../puppy_nav_core/src/amcl.cpp
)

# Windows 链接库
if(WIN32)
    target_link_libraries(nav_ue_bridge ws2_32)
endif()

# 输出目录
set_target_properties(nav_ue_bridge PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Release
)

# M1.5: 几何等价性校验工具
add_executable(geometry_check
    geometry_check.cpp
    ../puppy_nav_core/src/occupancy_grid.cpp
    ../puppy_nav_core/src/costmap.cpp
    ../puppy_nav_core/src/astar_planner.cpp
    ../puppy_nav_core/src/amcl.cpp
)
set_target_properties(geometry_check PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/Release
)
