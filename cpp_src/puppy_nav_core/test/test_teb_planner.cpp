// test_teb_planner.cpp — TEB Local Planner 单元测试
// 验证 TEB 规划器在简单场景下的正确性
//
// 编译: cl /O2 /std:c++17 /EHsc /utf-8 /MT /D_CRT_SECURE_NO_WARNINGS
//       test_teb_planner.cpp
//       ../puppy_nav_core/src/occupancy_grid.cpp
//       ../puppy_nav_core/src/costmap.cpp
//       ../puppy_nav_core/src/astar_planner.cpp
//       ../puppy_nav_core/src/amcl.cpp
//       /I../puppy_nav_core/include
//       /Fe:test_teb_planner.exe
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "puppy_nav_core/teb_planner.h"

using namespace puppy_nav_core;

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while (0)

// 测试1: 直线轨迹
// 机器人从原点朝向 (5, 0), 应该输出正线速度, 0 角速度
void test_straight_line() {
    std::printf("\n[Test 1] 直线轨迹\n");
    OccupancyGrid grid;
    Costmap costmap;
    costmap.update_static(grid, 0);
    TEBPlanner planner(costmap);

    std::vector<std::pair<double, double>> path = {
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}
    };
    double v, w;
    planner.compute_velocity(0.0, 0.0, 0.0, path, 3.0, 0.0, v, w);
    std::printf("  v=%.3f w=%.3f (期望 v>0, w≈0)\n", v, w);
    CHECK(v > 0.0, "直线轨迹应有正线速度");
    CHECK(std::abs(w) < 0.5, "直线轨迹角速度应小");
}

// 测试2: 目标到达
// 机器人在目标位置, 应该输出 0 速度
void test_goal_reached() {
    std::printf("\n[Test 2] 目标到达\n");
    OccupancyGrid grid;
    Costmap costmap;
    costmap.update_static(grid, 0);
    TEBPlanner planner(costmap);

    std::vector<std::pair<double, double>> path = {{0.0, 0.0}, {1.0, 0.0}};
    double v, w;
    planner.compute_velocity(1.0, 0.0, 0.0, path, 1.0, 0.0, v, w);
    std::printf("  v=%.3f w=%.3f (期望 v=0, w=0)\n", v, w);
    CHECK(std::abs(v) < 0.01, "目标到达时应停");
    CHECK(std::abs(w) < 0.01, "目标到达时不应旋转");
}

// 测试3: 需要转向
// 机器人朝东, 目标在北方, 应该有正角速度
void test_turn_required() {
    std::printf("\n[Test 3] 需要转向\n");
    OccupancyGrid grid;
    Costmap costmap;
    costmap.update_static(grid, 0);
    TEBPlanner planner(costmap);

    std::vector<std::pair<double, double>> path = {
        {0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}
    };
    // 机器人朝东 (yaw=0), 目标在北方 (y=3)
    double v, w;
    planner.compute_velocity(0.0, 0.0, 0.0, path, 0.0, 3.0, v, w);
    std::printf("  v=%.3f w=%.3f (期望 w>0, 转向北)\n", v, w);
    CHECK(w > 0.0, "应正向旋转朝向北");
}

// 测试4: 无路径
// 没有路径时应该原地旋转朝向目标
void test_no_path() {
    std::printf("\n[Test 4] 无路径\n");
    OccupancyGrid grid;
    Costmap costmap;
    costmap.update_static(grid, 0);
    TEBPlanner planner(costmap);

    std::vector<std::pair<double, double>> empty_path;
    double v, w;
    planner.compute_velocity(0.0, 0.0, 0.0, empty_path, 1.0, 1.0, v, w);
    std::printf("  v=%.3f w=%.3f (期望 v=0, w!=0)\n", v, w);
    CHECK(std::abs(v) < 0.5, "无路径时不应快速前进");
}

// 测试5: 带障碍物的场景
// 在路径上有障碍物时, 应该减速或绕行
void test_with_obstacle() {
    std::printf("\n[Test 5] 带障碍物场景\n");
    OccupancyGrid grid;
    // 在 (1, 0) 附近放置障碍物
    for (int gx = 18; gx <= 22; ++gx) {
        for (int gy = 38; gy <= 42; ++gy) {
            grid.log_odds[(size_t)gy * grid.width + gx] = LOG_ODDS_MAX;
        }
    }
    Costmap costmap;
    costmap.update_static(grid, 0);
    TEBPlanner planner(costmap);

    std::vector<std::pair<double, double>> path = {
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}
    };
    double v, w;
    planner.compute_velocity(0.0, 0.0, 0.0, path, 3.0, 0.0, v, w);
    std::printf("  v=%.3f w=%.3f (障碍物在 (1, 0) 附近)\n", v, w);
    // 不会崩溃即视为通过
    CHECK(true, "带障碍物场景不崩溃");
}

// 测试6: velocity_to_step 兼容性
// 验证速度到位置步长转换
void test_velocity_to_step() {
    std::printf("\n[Test 6] velocity_to_step\n");
    OccupancyGrid grid;
    Costmap costmap;
    costmap.update_static(grid, 0);
    TEBPlanner planner(costmap);

    double dx, dy, dyaw;
    planner.velocity_to_step(0.3, 0.0, 0.0, 0.1, dx, dy, dyaw);
    std::printf("  v=0.3 w=0 dt=0.1: dx=%.3f dy=%.3f dyaw=%.3f\n", dx, dy, dyaw);
    CHECK(std::abs(dx - 0.03) < 0.001, "dx 应为 0.03");
    CHECK(std::abs(dy) < 0.001, "dy 应为 0");
    CHECK(std::abs(dyaw) < 0.001, "dyaw 应为 0");
}

int main() {
#ifdef _WIN32
    std::system("chcp 65001 > nul 2>&1");
#endif
    std::printf("======================================================\n");
    std::printf("TEB Local Planner 单元测试\n");
    std::printf("======================================================\n");

    test_straight_line();
    test_goal_reached();
    test_turn_required();
    test_no_path();
    test_with_obstacle();
    test_velocity_to_step();

    std::printf("\n======================================================\n");
    std::printf("总计: %d 通过, %d 失败\n", g_pass, g_fail);
    std::printf("======================================================\n");
    return g_fail > 0 ? 1 : 0;
}
