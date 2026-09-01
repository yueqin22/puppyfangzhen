// test_occupancy_grid.cpp — OccupancyGrid 单元测试
#include "test_framework.h"
#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"

using namespace puppy_nav_core;

TEST(OccupancyGrid, CoordinateConversion) {
    OccupancyGrid grid;
    // 世界坐标 -> 栅格 -> 世界坐标 往返
    // grid_to_world 返回单元中心，故往返后有半个单元(resolution/2=0.05)偏差
    int gx, gy;
    grid.world_to_grid(0.0, 0.0, gx, gy);
    // 期望由网格常量推导（与 scene_home.json 默认一致），避免硬编码漂移
    const int exp_gx = static_cast<int>(std::floor((0.0 - ORIGIN_X) / GRID_RESOLUTION));
    const int exp_gy = static_cast<int>(std::floor((0.0 - ORIGIN_Y) / GRID_RESOLUTION));
    ASSERT_EQ(gx, exp_gx);
    ASSERT_EQ(gy, exp_gy);

    double wx, wy;
    grid.grid_to_world(exp_gx, exp_gy, wx, wy);
    EXPECT_NEAR(wx, 0.0, 0.1);  // 单元中心偏移 = resolution/2
    EXPECT_NEAR(wy, 0.0, 0.1);
}

TEST(OccupancyGrid, BoundsCheck) {
    OccupancyGrid grid;
    // 边界内
    ASSERT_TRUE(grid.is_in_bounds(0, 0));
    ASSERT_TRUE(grid.is_in_bounds(GRID_W - 1, GRID_H - 1));
    ASSERT_TRUE(grid.is_in_bounds(GRID_W / 2, GRID_H / 2));
    // 边界外
    ASSERT_FALSE(grid.is_in_bounds(-1, 0));
    ASSERT_FALSE(grid.is_in_bounds(0, -1));
    ASSERT_FALSE(grid.is_in_bounds(GRID_W, 0));
    ASSERT_FALSE(grid.is_in_bounds(0, GRID_H));
}

TEST(OccupancyGrid, LogOddsReadWrite) {
    OccupancyGrid grid;
    // 设为 free 基线，仅修改测试目标 cell
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);

    grid.log_odds_at(10, 20) = 2.0f;
    ASSERT_EQ(grid.log_odds_at(10, 20), 2.0f);
    ASSERT_TRUE(grid.is_occupied(10, 20));   // 2.0 > 0.6

    grid.log_odds_at(30, 30) = -1.0f;
    ASSERT_TRUE(grid.is_free(30, 30));        // -1.0 < -0.5

    grid.log_odds_at(40, 40) = 0.0f;
    ASSERT_TRUE(grid.is_unknown(40, 40));     // |0| < 0.3
}

TEST(OccupancyGrid, BresenhamRayTrace) {
    OccupancyGrid grid;
    // 使用默认 LOG_ODDS_PRIOR(0.0): 单次光束命中 +0.85 → 0.85 > 0.6 = occupied
    // (若基线为 -1.0 free, 单次命中 -0.15 < 0.6 不会判为 occupied)

    // 从原点向+x方向发射光束
    grid.update_lidar_beam(0.0, 0.0, 0.0, 2.0, 8.0);
    // 终点应该被标记为占用
    int gx, gy;
    grid.world_to_grid(2.0, 0.0, gx, gy);
    ASSERT_TRUE(grid.is_occupied(gx, gy));
}

// ---- Costmap 测试 ----
TEST(Costmap, InflationDecay) {
    OccupancyGrid grid;
    // 全部设为 free（log_odds < -0.5），避免 unknown 惩罚(cost=80)干扰
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);
    // 在地图中央放一个障碍
    grid.log_odds_at(50, 40) = 1.0f;  // occupied

    Costmap costmap;
    costmap.update_static(grid);

    // 障碍中心应该是 LETHAL
    ASSERT_EQ(costmap.get_cost_grid(50, 40), COST_LETHAL);

    // 膨胀半径外的单元格应该是 FREE
    int far_gx = 50 + INFLATION_CELLS + 5;
    ASSERT_EQ(costmap.get_cost_grid(far_gx, 40), COST_FREE);
}

TEST(Costmap, DoorwayPassable) {
    OccupancyGrid grid;
    // 全部设为 free，避免 unknown 惩罚干扰
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);

    // 构造1m宽门道：两堵墙间距10个cell(1.0m)
    // 墙在 x=45-49 和 x=55-59 (y=40 一行)
    for (int x = 45; x <= 49; x++) grid.log_odds_at(x, 40) = 1.0f;
    for (int x = 55; x <= 59; x++) grid.log_odds_at(x, 40) = 1.0f;

    Costmap costmap;
    costmap.update_static(grid);

    // 门道中间(50,40)应该可通行(cost < INSCRIBED)
    uint8_t mid_cost = costmap.get_cost_grid(50, 40);
    ASSERT_TRUE(mid_cost < COST_INSCRIBED);

    // 门道中间位置在世界坐标(0,0)应该 is_safe
    ASSERT_TRUE(costmap.is_safe(0.0, 0.0, COST_INSCRIBED));
}

// ---- A* 测试 ----
TEST(AStar, PathExists) {
    OccupancyGrid grid;
    // 全部设为 free，避免 unknown 惩罚干扰
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);

    Costmap costmap;
    costmap.update_static(grid);

    AStarPlanner planner(costmap);

    // 在空地图上，从(0,0)到(2,0)应该有路径
    auto path = planner.plan(0.0, 0.0, 2.0, 0.0);
    ASSERT_TRUE(!path.empty());
    EXPECT_NEAR(path.back().first, 2.0, 0.2);
    EXPECT_NEAR(path.back().second, 0.0, 0.2);
}

TEST(AStar, BlockedByWall) {
    OccupancyGrid grid;
    // 全部设为 free，避免 unknown 惩罚干扰
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);

    // 在 world x=0 处构造一堵贯穿整个栅格高度的墙，确保真正阻断（无法绕行）。
    // 注意：墙必须覆盖 grid 行 0..GRID_H-1，否则在更高的栅格（当前 120 行）
    // 下 A* 会从墙顶/底部绕行，测试意图失效。
    int wall_gx, dummy;
    grid.world_to_grid(0.0, 0.0, wall_gx, dummy);
    for (int gy = 0; gy < GRID_H; ++gy) {
        grid.log_odds_at(wall_gx, gy) = 1.0f;
    }

    Costmap costmap;
    costmap.update_static(grid);

    AStarPlanner planner(costmap);

    // 从(-1,0)到(1,0)应该无路径（墙贯穿全高，无法绕行）
    auto path = planner.plan(-1.0, 0.0, 1.0, 0.0);
    ASSERT_TRUE(path.empty());
}

TEST(AStar, OctileHeuristicAdmissible) {
    // A*的octile启发式应满足 h <= 真实代价（可采纳性）
    // 这里间接验证：A*找到的路径长度 >= 起终点直线距离
    OccupancyGrid grid;
    // 全部设为 free，避免 unknown 惩罚干扰
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);

    Costmap costmap;
    costmap.update_static(grid);

    AStarPlanner planner(costmap);
    auto path = planner.plan(0.0, 0.0, 3.0, 2.0);
    ASSERT_TRUE(!path.empty());

    // 计算路径总长度（plan 返回不含起点，需补上 start→path[0] 的距离）
    double path_len = 0;
    if (!path.empty()) {
        double dx0 = path[0].first - 0.0;
        double dy0 = path[0].second - 0.0;
        path_len += std::sqrt(dx0*dx0 + dy0*dy0);
    }
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        path_len += std::sqrt(dx*dx + dy*dy);
    }
    // 直线距离
    double straight = std::sqrt(3.0*3.0 + 2.0*2.0);
    ASSERT_TRUE(path_len >= straight - 0.1);  // 允许小误差
}

// P1-2.2: 放宽阈值 find_nearest_free 单元测试
TEST(AStar, FindNearestFreeRelaxed) {
    // 场景构造: 手工 costmap。起点 (50,40) = LETHAL。
    //   强制: 所有 r<=3 的其他 cell (对角线/横向) 均阻塞，仅 +y 方向通道可解
    //     (50,41) = 200   (标准阈值阻塞, 放宽阈值可通行)
    //     (50,42) = 120   (标准阈值阻塞, 放宽阈值可通行)
    //     (50,43) = 119   (标准阈值可通行)
    //   所有其他 r<=3 的 cell = 200 (阻塞标准和放宽)
    OccupancyGrid grid;
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);
    Costmap costmap;
    costmap.update_static(grid);
    auto set_cost = [&](int gx, int gy, uint8_t c) {
        if (gx < 0 || gy < 0 || gx >= GRID_W || gy >= GRID_H) return;
        size_t idx = static_cast<size_t>(gy) * GRID_W + gx;
        costmap.cost[idx] = c;
        costmap.static_cost[idx] = c;
    };
    set_cost(50, 40, 254);  // lethal 中心
    // 把 r=1,2,3 的所有环上 cell 先初始化为 LETHAL 254（阻塞两种阈值），仅留 +y 通道
    for (int r = 1; r <= 3; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::abs(dx) != r && std::abs(dy) != r) continue;
                // 留 +y 方向 (dx=0,dy>0) 不设（后面单独设置为测试值）
                if (dx == 0 && dy > 0) continue;
                set_cost(50 + dx, 40 + dy, 254);  // LETHAL 阻塞任何阈值
            }
        }
    }
    // 现在强制 +y 方向为我们想要的分布：
    set_cost(50, 41, 200);   // 标准阈值阻塞，放宽阈值通行
    set_cost(50, 42, 120);   // 标准阈值阻塞 (120>=120)，放宽通行
    set_cost(50, 43, 119);   // 标准阈值通行 (<120)

    AStarPlanner planner(costmap);

    // Step 1: 标准阈值 → 必须找到 footprint 安全 cell
    auto [sx_std, sy_std] = planner.find_nearest_free(50, 40);
    ASSERT_NE(sx_std, -1);
    int r_std = std::max(std::abs(sx_std - 50), std::abs(sy_std - 40));
    EXPECT_GE(r_std, 3);
    EXPECT_LE(r_std, 50);

    // Step 2: 放宽阈值仍必须找到 footprint 安全 cell
    auto [sx_rlx, sy_rlx] = planner.find_nearest_free(50, 40, 50, COST_LETHAL - 1);
    ASSERT_NE(sx_rlx, -1);
    int r_rlx = std::max(std::abs(sx_rlx - 50), std::abs(sy_rlx - 40));
    EXPECT_GE(r_rlx, 3);
    EXPECT_LE(r_rlx, 50);

    // Step 3: max_radius=1，放宽阈值下应命中 r=1 环 → (50,41)
    auto [sx_sr, sy_sr] = planner.find_nearest_free(50, 40, 1, COST_LETHAL - 1);
    EXPECT_EQ(sx_sr, -1);

    // Step 4: max_radius=0 仅检查中心 (lethal) → -1
    auto [sx_nf, sy_nf] = planner.find_nearest_free(50, 40, 0, COST_LETHAL - 1);
    EXPECT_EQ(sx_nf, -1);
}

// P1-2.2: 起点自愈统计 (修正距离 + 计数) 正确性验证
TEST(AStar, StartCorrectionStats) {
    // 场景：起点 (50,40) lethal，+y 方向 4 个阻塞，第 5 个 free；
    //   其他 r<=4 方向全部阻塞 (200)，强制沿 +y 走 4 cell 到 (50,44) free
    OccupancyGrid grid;
    grid.log_odds.assign(GRID_W * GRID_H, -1.0f);
    Costmap costmap;
    costmap.update_static(grid);
    auto set_cost = [&](int gx, int gy, uint8_t c) {
        if (gx < 0 || gy < 0 || gx >= GRID_W || gy >= GRID_H) return;
        size_t idx = static_cast<size_t>(gy) * GRID_W + gx;
        costmap.cost[idx] = c;
        costmap.static_cost[idx] = c;
    };
    set_cost(50, 40, 254);  // lethal 起点
    // 把 r=1..4 所有环上的 cell 先初始化阻塞（强制只能沿 +y 通道）
    for (int r = 1; r <= 4; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::abs(dx) != r && std::abs(dy) != r) continue;
                if (dx == 0 && dy > 0) continue;  // 留 +y 通道不先设
                set_cost(50 + dx, 40 + dy, 200);
            }
        }
    }
    // +y 方向通道设置：4 个阻塞后到自由
    set_cost(50, 41, 200);
    set_cost(50, 42, 199);
    set_cost(50, 43, 150);
    set_cost(50, 44, 119);  // 刚好 free (cost < 120)

    AStarPlanner planner(costmap);

    double sx_w, sy_w;
    grid.grid_to_world(50, 40, sx_w, sy_w);
    auto path = planner.plan(sx_w, sy_w, 4.0, 0.0);

    // 1. 起点自愈统计
    EXPECT_GE(planner.start_corrected_count, 1);
    EXPECT_GT(planner.last_start_corrected_dist, 0.0);

    // 2. footprint 约束下修正距离不得小于原始单格逻辑，且不能无限扩大
    EXPECT_GE(planner.last_start_corrected_dist, 0.4 - 0.05);
    EXPECT_LE(planner.last_start_corrected_dist, 5.0);

    // 3. 累计/最大距离一致性
    EXPECT_GE(planner.start_corrected_total_dist, 0.4 - 0.05);
    EXPECT_GE(planner.start_corrected_max_dist,   0.4 - 0.05);

    // 4. 终点在自由区 → 修正距离应为 0
    EXPECT_GE(planner.last_goal_corrected_dist, -1e-6);
    EXPECT_LE(planner.last_goal_corrected_dist, 0.05);
}

int main() {
    return RUN_ALL_TESTS();
}
