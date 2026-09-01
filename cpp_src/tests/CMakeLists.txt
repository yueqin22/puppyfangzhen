cmake_minimum_required(VERSION 3.10)
project(nav_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

# tests/CMakeLists.txt — 单元测试构建
# 零第三方依赖，使用自研 test_framework.h
# 通过 ctest 运行: cmake --build build --config Release --target RUN_TESTS
#                  或: ctest --test-dir build -C Release --output-on-failure

# 导航核心源文件（与 sim/CMakeLists.txt 保持一致）
set(NAV_CORE_SOURCES
    ../puppy_nav_core/src/occupancy_grid.cpp
    ../puppy_nav_core/src/costmap.cpp
    ../puppy_nav_core/src/astar_planner.cpp
    ../puppy_nav_core/src/amcl.cpp
    ../puppy_nav_core/src/path_validator.cpp
)

# 公共编译选项
function(configure_test target)
    target_include_directories(${target} PRIVATE
        ../puppy_nav_core/include
        .
    )
    target_compile_options(${target} PRIVATE
        /O2 /std:c++17 /EHsc /utf-8 /MT
    )
    # NOMINMAX: 防止 <windows.h>/<winsock2.h> 把 min/max 定义为宏，
    # 与协议头中的 std::min 冲突（C2589）。与 bridge 构建保持一致。
    target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
endfunction()

# ---- OccupancyGrid / Costmap / A* 单元测试 ----
add_executable(test_occupancy_grid
    test_occupancy_grid.cpp
    ${NAV_CORE_SOURCES}
)
configure_test(test_occupancy_grid)
add_test(NAME occupancy_grid COMMAND test_occupancy_grid)

# ---- AMCL 单元测试 ----
add_executable(test_amcl
    test_amcl.cpp
    ${NAV_CORE_SOURCES}
)
configure_test(test_amcl)
add_test(NAME amcl COMMAND test_amcl)

# ---- R31b LiDAR 反应式安全层单元测试 ----
# 纯头文件实现（../sim/lidar_safety.h），无需 nav core 源文件。
# 这一层直接跑在 UE 真实交付路径上，此前完全没有白盒覆盖。
add_executable(test_lidar_safety
    test_lidar_safety.cpp
)
configure_test(test_lidar_safety)
add_test(NAME lidar_safety COMMAND test_lidar_safety)

# ---- R33 机身几何 / 安全阶梯单元测试 ----
# 锁定"圆盘 Minkowski 和 vs 方形 Minkowski 和"的判据差异（r32 的 4 次碰撞
# 全部是方形判据在凸角处的假阳性），以及
# 机身 0.20 < r_pass 0.27 < 规划 footprint 0.35 < 回路瓶颈 0.50 的单调阶梯。
add_executable(test_body_geometry
    test_body_geometry.cpp
)
configure_test(test_body_geometry)
add_test(NAME body_geometry COMMAND test_body_geometry)

# ---- P0-03 协议 v2 故障注入 / 状态机白盒测试 ----
# 覆盖握手、版本不符拒绝、序列号校验、接收超时、断线、半包、帧往返、
# SIM_END/SHUTDOWN_ACK 往返。需要 Winsock。
add_executable(test_protocol
    test_protocol.cpp
)
configure_test(test_protocol)
target_include_directories(test_protocol PRIVATE ../bridge)
if(WIN32)
    target_link_libraries(test_protocol ws2_32)
endif()
add_test(NAME protocol COMMAND test_protocol)

# ---- 模块一 统一配置加载器 白盒测试 (C++ 侧) ----
# 覆盖: 内联契约解析 / 一致性校验(合法0错;坏footprint;坏inflation)
#       / 扁平快照 / SHA-256 确定性 / 加载真实契约并与 Python 侧字段对齐。
# 零第三方依赖 (自带最小 YAML 解析器, 与 Python config_loader 对齐)。
add_executable(test_config_contract
    test_config_contract.cpp
)
configure_test(test_config_contract)
target_include_directories(test_config_contract PRIVATE ../common)
add_test(NAME config_contract COMMAND test_config_contract)

# ---- 模块二 PathValidator 白盒测试 (A*/Costmap footprint 安全) ----
# 覆盖 §6.6 白盒场景 (空/单点/起终点障碍/越界/偏目标/中心安footprint撞/
#       连线穿墙/净空不足/未知区/旋转角点/采样稳定/净空上报) + §6.5 窄通道矩阵
#       + §6.1 根因修复 (A* 对窄门返回的"成功"路径被验证器拦下)。
add_executable(test_path_validator
    test_path_validator.cpp
    ${NAV_CORE_SOURCES}
)
configure_test(test_path_validator)
add_test(NAME path_validator COMMAND test_path_validator)
