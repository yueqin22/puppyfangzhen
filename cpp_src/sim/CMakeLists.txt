cmake_minimum_required(VERSION 3.8)
project(sim_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 纯 C++ 仿真，无任何外部依赖
# 编译 main.cpp 为 sim_test 可执行文件
add_executable(sim_test main.cpp)
