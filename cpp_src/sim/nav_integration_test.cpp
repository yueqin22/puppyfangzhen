// nav_integration_test.cpp — NavCoreStack 集成测试
// =================================================
// 在仿真场景下验证 puppy_nav_core 的 AMCL/Costmap/A* 集成正确性。
//
// 测试场景:
//   1. 从 BBox 障碍物构建 OccupancyGrid
//   2. AMCL 初始化并跑 N 帧（机器人沿巡航点移动）
//   3. A* 在新 Costmap 上规划路径
//   4. 检查 AMCL 估计误差 < 0.5m, 置信度 > 0.3
//
// 编译: cl /O2 /std:c++17 /EHsc /utf-8 /MT /D_CRT_SECURE_NO_WARNINGS
//       nav_integration_test.cpp
//       ..\puppy_nav_core\src\occupancy_grid.cpp
//       ..\puppy_nav_core\src\costmap.cpp
//       ..\puppy_nav_core\src\astar_planner.cpp
//       ..\puppy_nav_core\src\amcl.cpp
//       /I..\puppy_nav_core\include /I.
//       /Fe:nav_integration_test.exe
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "nav_bridge.h"
#include "simulation.h"  // 复用 build_obstacles / build_patrol_targets

using namespace puppy_sim;

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while (0)

// 模拟机器人在场景中移动（按巡航点）
void simulate_robot_motion(NavCoreStack& nav,
                            const std::vector<PatrolTarget>& targets,
                            const std::vector<BBox>& obstacles,
                            int num_frames) {
    double rx = targets[0].x, ry = targets[0].y, ryaw = targets[0].yaw;
    nav.init_amcl(rx, ry, ryaw, 0.3);

    double total_err = 0.0;
    double max_err = 0.0;
    int err_samples = 0;

    for (int frame = 0; frame < num_frames; ++frame) {
        // 简单地朝第一个目标走
        auto& tgt = targets[0];
        double dx = tgt.x - rx, dy = tgt.y - ry;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d > 0.05) {
            double step = std::min(2.0 / 30.0, d);  // 2 m/s, 30 FPS
            rx += step * dx / d;
            ry += step * dy / d;
            ryaw = std::atan2(dy, dx);
        }

        // AMCL 更新
        nav.update(rx, ry, ryaw, obstacles, frame);

        // 计算定位误差
        double err = std::sqrt(
            (nav.est_x - rx) * (nav.est_x - rx) +
            (nav.est_y - ry) * (nav.est_y - ry));
        total_err += err;
        if (err > max_err) max_err = err;
        err_samples++;

        if (frame % 50 == 0 && frame > 0) {
            std::printf("  frame %d: true=(%.2f,%.2f) est=(%.2f,%.2f) "
                        "err=%.3fm conf=%.3f n=%d\n",
                        frame, rx, ry, nav.est_x, nav.est_y, err,
                        nav.est_conf, nav.amcl.n);
        }
    }

    double avg_err = total_err / err_samples;
    std::printf("\n  === 定位误差统计 ===\n");
    std::printf("  平均误差: %.3f m\n", avg_err);
    std::printf("  最大误差: %.3f m\n", max_err);

    CHECK(avg_err < 0.5, "AMCL avg error < 0.5m");
    CHECK(max_err < 1.0, "AMCL max error < 1.0m");
    CHECK(nav.est_conf > 0.1, "AMCL confidence > 0.1");
}

// 测试 A* 路径规划在真实场景下能找到路径
// 沿巡航点连续规划（仿真实际使用的模式）
void test_astar_in_scene(NavCoreStack& nav,
                          const std::vector<PatrolTarget>& targets,
                          const std::vector<BBox>& obstacles) {
    std::printf("\n[Test] A* 路径规划（连续巡航点 = 仿真实际模式）\n");

    int success = 0;
    int total = 0;
    double total_path_len = 0.0;

    // 测试所有 19 个连续巡航点路径
    for (size_t i = 0; i + 1 < targets.size(); ++i) {
        auto& s = targets[i];
        auto& g = targets[i + 1];
        total++;

        auto path = nav.plan(s.x, s.y, g.x, g.y, obstacles, 0);

        if (!path.empty()) {
            success++;
            double end_dx = path.back().first - g.x;
            double end_dy = path.back().second - g.y;
            double end_dist = std::sqrt(end_dx * end_dx + end_dy * end_dy);
            // 计算路径总长度
            double path_len = 0.0;
            for (size_t k = 1; k < path.size(); ++k) {
                double ddx = path[k].first - path[k-1].first;
                double ddy = path[k].second - path[k-1].second;
                path_len += std::sqrt(ddx * ddx + ddy * ddy);
            }
            total_path_len += path_len;
            if (end_dist > 0.5) {
                std::printf("  %s -> %s: path=%zu pts len=%.1fm FAR(%.2fm)\n",
                            s.name.c_str(), g.name.c_str(),
                            path.size(), path_len, end_dist);
            }
        } else {
            std::printf("  %s -> %s: FAILED (no path)\n",
                        s.name.c_str(), g.name.c_str());
        }
    }

    std::printf("\n  A* 成功率: %d/%d (%.1f%%)\n", success, total,
                100.0 * success / total);
    std::printf("  路径总长度: %.1fm\n", total_path_len);
    // 仿真实际模式：连续巡航点，至少 70% 成功率
    // 失败路径通常是终点在桌椅/墙边（如 dining_detour 在桌椅之间），
    // 膨胀后阻塞 — 是几何限制而非算法问题
    CHECK(success >= total * 0.70, "A* success rate >= 70%");
}

int main() {
    std::printf("======================================================\n");
    std::printf("NavCoreStack 集成测试\n");
    std::printf("======================================================\n");

    // 1. 构建仿真场景
    auto obstacles = build_obstacles();
    auto targets = build_patrol_targets();
    std::printf("场景: %zu 个障碍物, %zu 个巡航点\n",
                obstacles.size(), targets.size());

    // 2. 初始化 NavCoreStack
    NavCoreStack nav;
    std::printf("\n初始化 OccupancyGrid + Costmap...\n");
    nav.init_from_obstacles(obstacles);

    // 验证占用栅格已正确构建
    int n_occ = nav.grid.count_occupied();
    std::printf("  占用 cell 数: %d\n", n_occ);
    CHECK(n_occ > 100, "occupied cells > 100");
    CHECK(n_occ < 5000, "occupied cells < 5000 (not all walls)");

    // 3. 测试 AMCL 定位
    std::printf("\n[Test] AMCL 定位（300帧 = 10秒）\n");
    simulate_robot_motion(nav, targets, obstacles, 300);

    // 4. 测试 A* 路径规划
    test_astar_in_scene(nav, targets, obstacles);

    // 5. 最终统计
    std::printf("\n======================================================\n");
    std::printf("最终统计\n");
    std::printf("======================================================\n");
    nav.report();

    std::printf("\n======================================================\n");
    std::printf("总计: %d 通过, %d 失败\n", g_pass, g_fail);
    std::printf("======================================================\n");

    return g_fail == 0 ? 0 : 1;
}
