// puppy_nav_core 综合验证测试
// ============================
// 验证 C++ 实现的 OccupancyGrid / Costmap / AStarPlanner / AMCL 功能正确性
// 通过与 Python 版本逻辑等价的测试用例确认转换无误
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/amcl.h"

using namespace puppy_nav_core;

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while (0)

static void mark_free_rect(OccupancyGrid& grid,
                           double x0, double y0,
                           double x1, double y1) {
    for (double x = x0; x <= x1; x += GRID_RESOLUTION) {
        for (double y = y0; y <= y1; y += GRID_RESOLUTION) {
            for (int i = 0; i < 2; ++i) {
                grid.update_free(x, y);
            }
        }
    }
}

// ===== 测试1: OccupancyGrid 基本功能 =====
void test_occupancy_grid_basic() {
    std::printf("\n[Test 1] OccupancyGrid 基本功能\n");
    OccupancyGrid grid;
    
    // 初始状态：所有 log_odds = 0
    CHECK(std::abs(grid.mean_log_odds()) < 1e-6, "initial mean_log_odds = 0");
    CHECK(grid.count_occupied() == 0, "initial count_occupied = 0");
    
    // 坐标转换：原点 (-5, -4) → grid (0, 0)
    int gx, gy;
    grid.world_to_grid(-5.0, -4.0, gx, gy);
    CHECK(gx == 0 && gy == 0, "world_to_grid origin");
    
    // grid (0,0) → world (-4.95, -3.95) (cell center)
    double wx, wy;
    grid.grid_to_world(0, 0, wx, wy);
    CHECK(std::abs(wx - (-4.95)) < 1e-6, "grid_to_world x");
    CHECK(std::abs(wy - (-3.95)) < 1e-6, "grid_to_world y");
    
    // 更新两束 LiDAR 光线（命中 3m 处的墙）
    // 注意：单次 MISS 更新只让 log_odds 到 -0.4，但 is_free 阈值是 < -0.5
    // 需要至少 2 次观测才能确认 cell 为 free（匹配 Python 行为）
    grid.update_lidar_beam(0.0, 0.0, 0.0, 3.0, 8.0);
    grid.update_lidar_beam(0.0, 0.0, 0.0, 3.0, 8.0);
    CHECK(grid.count_occupied() >= 1, "lidar beam creates occupied cell");
    
    // 命中点应该是 occupied
    int hgx, hgy;
    grid.world_to_grid(3.0, 0.0, hgx, hgy);
    CHECK(grid.is_occupied(hgx, hgy), "hit point is occupied");
    
    // 光线沿途应该是 free（多次观测后 log_odds < -0.5）
    int mgx, mgy;
    grid.world_to_grid(1.0, 0.0, mgx, mgy);
    CHECK(grid.is_free(mgx, mgy), "mid-point is free");
    
    std::printf("  occupied=%d, mean_lo=%.3f\n", grid.count_occupied(), grid.mean_log_odds());
}

// ===== 测试2: Costmap 膨胀 =====
void test_costmap_inflation() {
    std::printf("\n[Test 2] Costmap 膨胀与查询\n");
    OccupancyGrid grid;
    Costmap costmap;
    
    // 放一堵墙在 (3, 0)
    grid.update_lidar_beam(0.0, 0.0, 0.0, 3.0, 8.0);
    
    // 更新静态层
    costmap.update_static(grid, 0);
    
    // 墙体位置应该是 lethal
    uint8_t wall_cost = costmap.get_cost(3.0, 0.0);
    CHECK(wall_cost >= COST_LETHAL - 1, "wall is lethal");
    
    // 远离墙的位置应该是 free
    uint8_t free_cost = costmap.get_cost(0.0, 0.0);
    CHECK(free_cost < COST_INSCRIBED, "far from wall is safe");
    
    // 接近墙的位置应该有膨胀代价（高于 free 但低于 lethal）
    uint8_t near_cost = costmap.get_cost(2.5, 0.0);
    CHECK(near_cost > free_cost, "near wall has inflation cost");
    CHECK(near_cost < COST_LETHAL, "near wall not lethal");
    
    std::printf("  wall_cost=%u, free_cost=%u, near_cost=%u\n",
                wall_cost, free_cost, near_cost);
}

// ===== 测试3: A* 路径规划 =====
void test_astar_planner() {
    std::printf("\n[Test 3] A* 路径规划\n");
    OccupancyGrid grid;
    Costmap costmap;
    
    // 步骤1: 先用 360° LiDAR 扫描标记机器人周围的自由区域
    // 否则未观测的 cell 为 UNKNOWN，A* 拒绝穿越（设计意图）
    // 扫描半径需覆盖目标 (4,0) 距离 4m，设为 5m 确保目标 cell 已观测
    for (int i = 0; i < 360; i += 5) {
        double ang = i * M_PI / 180.0;
        grid.update_lidar_beam(0.0, 0.0, ang, 5.0, 8.0);
    }
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = -25; i <= 25; ++i) {
            double y = i * 0.02;
            double ang = std::atan2(y, 4.5);
            double dist = std::sqrt(4.5 * 4.5 + y * y);
            grid.update_lidar_beam(0.0, 0.0, ang, dist, 8.0);
        }
    }
    mark_free_rect(grid, -0.2, -0.45, 4.2, 0.45);
    
    // 步骤2: 创建一堵墙挡在 x=2，y=[-1, 1] 之间，留 y=[-0.2, 0.2] 缺口
    for (double y = -1.0; y <= 1.0; y += 0.1) {
        if (std::abs(y) > 0.2) {  // 留 0.4m 缺口
            grid.update_lidar_beam(0.0, 0.0, std::atan2(y, 2.0),
                                   std::sqrt(4.0 + y*y), 8.0);
        }
    }
    costmap.update_static(grid, 0);
    
    AStarPlanner planner(costmap);
    
    // 规划从 (0, 0) 到 (4, 0) 的路径
    auto path = planner.plan(0.0, 0.0, 4.0, 0.0);
    
    CHECK(!path.empty(), "A* finds a path");
    CHECK(path.size() >= 1, "path has at least one waypoint");
    
    // 防御性检查：仅在非空时访问 back()
    if (!path.empty()) {
        // 终点应该在目标附近
        auto& end = path.back();
        double dist_to_goal = std::sqrt((end.first - 4.0)*(end.first - 4.0) +
                                         (end.second - 0.0)*(end.second - 0.0));
        CHECK(dist_to_goal < 0.5, "path ends near goal");
        std::printf("  path size=%zu, end=(%.2f, %.2f), dist_to_goal=%.2f\n",
                    path.size(), end.first, end.second, dist_to_goal);
    } else {
        std::printf("  path is empty (失败)\n");
    }
    
    // 测试无路径情况（目标在墙后且完全封闭）
    // (跳过 — 实际场景中很难完全封闭)
}

// ===== 测试4: AMCL 基本定位 =====
void test_amcl_localization() {
    std::printf("\n[Test 4] AMCL 粒子滤波定位\n");
    OccupancyGrid grid;
    
    // 建一个简单的房间：四周有墙
    // 墙1: x=-2, 墙2: x=2, 墙3: y=-2, 墙4: y=2
    grid.update_lidar_beam(0.0, 0.0, 0.0, 2.0, 8.0);      // 东墙
    grid.update_lidar_beam(0.0, 0.0, M_PI, 2.0, 8.0);       // 西墙
    grid.update_lidar_beam(0.0, 0.0, M_PI/2, 2.0, 8.0);    // 北墙
    grid.update_lidar_beam(0.0, 0.0, -M_PI/2, 2.0, 8.0);   // 南墙
    
    AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 200, 0.05, 0.99);
    
    // 初始化粒子云在 (0.1, 0.1, 0) 附近（故意偏一点）
    amcl.init_cloud(0.1, 0.1, 0.0, 0.3);
    
    auto [x, y, yaw, conf] = amcl.get_estimate();
    CHECK(std::abs(x - 0.1) < 0.5, "initial x near 0.1");
    CHECK(std::abs(y - 0.1) < 0.5, "initial y near 0.1");
    CHECK(conf > 0.0, "confidence > 0");
    
    std::printf("  初始估计: x=%.3f, y=%.3f, yaw=%.3f, conf=%.3f, n=%d\n",
                x, y, yaw, conf, amcl.n);
    
    // 模拟5次 update（无运动，仅观测）
    // 观测：4面墙各 2m 远
    std::vector<double> angles = {-M_PI/2, 0.0, M_PI/2, M_PI};
    std::vector<double> distances = {2.0, 2.0, 2.0, 2.0};
    
    for (int i = 0; i < 5; ++i) {
        auto [px, py, pyaw, pconf] = amcl.update(0, 0, 0, angles, distances, i);
        std::printf("  frame %d: x=%.3f, y=%.3f, yaw=%.3f, conf=%.3f, n=%d, n_eff=%.1f\n",
                    i, px, py, pyaw, pconf, amcl.n, amcl.last_n_eff);
    }
    
    auto [fx, fy, fyaw, fconf] = amcl.get_estimate();
    // 5帧观测后，估计应该收敛到 (0, 0) 附近（因为观测对应机器人在原点）
    CHECK(std::abs(fx) < 0.5, "final x near 0");
    CHECK(std::abs(fy) < 0.5, "final y near 0");
    CHECK(fconf > 0.1, "final confidence reasonable");
    
    std::printf("  最终估计: x=%.3f, y=%.3f, yaw=%.3f, conf=%.3f, n=%d\n",
                fx, fy, fyaw, fconf, amcl.n);
}

// ===== 测试5: KLD 采样 =====
void test_kld_sampling() {
    std::printf("\n[Test 5] KLD 采样 Wilson-Hilferty\n");
    OccupancyGrid grid;
    AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 500, 0.05, 0.99);
    
    // 通过反射访问私有 kld_sample_size 不可行，这里通过 resample 间接测试
    // 初始化一个分散的粒子云，然后 resample 看粒子数是否合理
    amcl.init_cloud(0.0, 0.0, 0.0, 5.0);
    
    // 设置均匀权重
    amcl.last_n_eff = 1.0;  // 强制触发 resample
    
    int old_n = amcl.n;
    amcl.resample();
    int new_n = amcl.n;
    
    CHECK(new_n >= 50, "KLD: n >= kld_min");
    CHECK(new_n <= 500, "KLD: n <= kld_max");
    
    std::printf("  分散云 resample: old_n=%d, new_n=%d\n", old_n, new_n);
    
    // 集中云（粒子都在一起）
    amcl.init_cloud(0.0, 0.0, 0.0, 0.1);
    amcl.last_n_eff = 1.0;
    amcl.resample();
    int concentrated_n = amcl.n;
    
    std::printf("  集中云 resample: n=%d (应该比分散云少)\n", concentrated_n);
    // 注意：由于随机性，这个测试只验证范围，不严格验证集中<分散
}

// ===== 测试6: 性能基准 =====
void test_performance() {
    std::printf("\n[Test 6] 性能基准（C++ vs Python 预期）\n");
    OccupancyGrid grid;
    
    // 建立复杂环境
    for (int i = 0; i < 360; i += 5) {
        double ang = i * M_PI / 180.0;
        double dist = 1.5 + 0.5 * std::sin(5.0 * ang);
        grid.update_lidar_beam(0.0, 0.0, ang, dist, 8.0);
    }
    
    Costmap costmap;
    
    // 测量 costmap 更新耗时
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 1000; ++i) {
        costmap.update_static(grid, i);
    }
    auto t1 = std::chrono::steady_clock::now();
    double costmap_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / 1000.0;
    
    // 测量 A* 规划耗时
    AStarPlanner planner(costmap);
    auto t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i) {
        planner.plan(-1.0, -1.0, 1.0, 1.0);
    }
    auto t3 = std::chrono::steady_clock::now();
    double astar_ms = std::chrono::duration<double, std::milli>(t3 - t2).count() / 100.0;
    
    // 测量 AMCL 更新耗时
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.3);
    std::vector<double> angles, distances;
    for (int i = 0; i < 360; i += 10) {
        angles.push_back(i * M_PI / 180.0);
        distances.push_back(1.5 + 0.5 * std::sin(5.0 * i * M_PI / 180.0));
    }
    auto t4 = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i) {
        amcl.update(0.001, 0.001, 0.001, angles, distances, i);
    }
    auto t5 = std::chrono::steady_clock::now();
    double amcl_ms = std::chrono::duration<double, std::milli>(t5 - t4).count() / 100.0;
    
    std::printf("  Costmap update:   %.3f ms/frame (Python ~5-10ms)\n", costmap_ms);
    std::printf("  A* planning:      %.3f ms/plan    (Python ~20-50ms)\n", astar_ms);
    std::printf("  AMCL update:      %.3f ms/frame (Python ~3-5ms)\n", amcl_ms);
    
    CHECK(costmap_ms < 5.0, "costmap < 5ms (faster than Python)");
    CHECK(astar_ms < 20.0, "A* < 20ms (faster than Python)");
    CHECK(amcl_ms < 5.0, "AMCL < 5ms (meets <30ms constraint)");
    
    std::printf("\n  性能约束验证:\n");
    std::printf("    AMCL < 30ms/frame: %s\n", amcl_ms < 30.0 ? "PASS" : "FAIL");
    std::printf("    控制环 ≥20Hz:     %s\n", costmap_ms + amcl_ms < 50.0 ? "PASS" : "FAIL");
}

int main() {
    std::printf("======================================================\n");
    std::printf("puppy_nav_core C++ 综合验证测试\n");
    std::printf("======================================================\n");
    
    test_occupancy_grid_basic();
    test_costmap_inflation();
    test_astar_planner();
    test_amcl_localization();
    test_kld_sampling();
    test_performance();
    
    std::printf("\n======================================================\n");
    std::printf("总计: %d 通过, %d 失败\n", g_pass, g_fail);
    std::printf("======================================================\n");
    
    return g_fail == 0 ? 0 : 1;
}
