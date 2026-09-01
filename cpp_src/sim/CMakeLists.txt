# sim/CMakeLists.txt — C++ 仿真器构建
# 纯 C++17，零第三方依赖
# 编译选项与原 build.bat (cl.exe) 完全对齐

# 导航核心源文件
set(NAV_CORE_SOURCES
    ../puppy_nav_core/src/occupancy_grid.cpp
    ../puppy_nav_core/src/costmap.cpp
    ../puppy_nav_core/src/astar_planner.cpp
    ../puppy_nav_core/src/amcl.cpp
    ../puppy_nav_core/src/path_validator.cpp
)

# 仿真器主程序
add_executable(sim_test
    main.cpp
    ${NAV_CORE_SOURCES}
)

target_include_directories(sim_test PRIVATE
    ../puppy_nav_core/include
    .
)

# MSVC 编译选项（与原 build.bat cl.exe 命令完全一致）
# /utf-8 是关键：不加会导致 amcl.cpp 中 UTF-8 注释使编译器误解析命名空间
# /std:c++17 必须显式传：CMake 的 CXX_STANDARD 对 VS 生成器不自动加标志
target_compile_options(sim_test PRIVATE
    /O2 /std:c++17 /EHsc /utf-8 /MT
)
target_compile_definitions(sim_test PRIVATE _CRT_SECURE_NO_WARNINGS)

# ---- 集成测试（可选）----
option(BUILD_INTEGRATION_TESTS "Build integration tests" OFF)
if(BUILD_INTEGRATION_TESTS)
    add_executable(astar_corridor_test
        astar_corridor_test.cpp
        ${NAV_CORE_SOURCES}
    )
    target_include_directories(astar_corridor_test PRIVATE
        ../puppy_nav_core/include
        .
    )
    target_compile_options(astar_corridor_test PRIVATE /O2 /EHsc /utf-8 /MT)
    target_compile_definitions(astar_corridor_test PRIVATE _CRT_SECURE_NO_WARNINGS)

    add_executable(nav_integration_test
        nav_integration_test.cpp
        ${NAV_CORE_SOURCES}
    )
    target_include_directories(nav_integration_test PRIVATE
        ../puppy_nav_core/include
        ../common
        .
    )
    target_compile_options(nav_integration_test PRIVATE /O2 /EHsc /utf-8 /MT)
    target_compile_definitions(nav_integration_test PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
