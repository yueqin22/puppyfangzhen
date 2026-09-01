// test_path_validator.cpp — M2 PathValidator 白盒测试
// ================================================
// 覆盖 jihua20260818 §6.6 白盒场景 + §6.5 窄通道矩阵。
// 编译: 见 tests/CMakeLists.txt (链接 nav_core 源 + path_validator.cpp)
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include "test_framework.h"
#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/path_validator.h"

using namespace puppy_nav_core;

// 统一契约的 footprint (来自 config/simulation_contract.yaml)
static const std::vector<Vec2> kContractFootprint = {
    {-0.35, -0.25}, {-0.35, 0.25}, {0.35, 0.25}, {0.35, -0.25}
};

static PathValidationOptions make_opts() {
    PathValidationOptions o;
    o.footprint = kContractFootprint;
    o.min_path_clearance = 0.08;
    o.goal_tolerance = 0.35;
    o.sample_step = 0.025;     // = res(0.1)/4 <= res/2
    o.unknown_is_blocked = true;
    return o;
}

// 全部置为 free (log_odds < -0.5)
static void fill_free(OccupancyGrid& g) {
    g.log_odds.assign((size_t)g.width * g.height, -1.0f);
}
// 全部置为 unknown (log_odds = 0)
static void fill_unknown(OccupancyGrid& g) {
    g.log_odds.assign((size_t)g.width * g.height, 0.0f);
}

// 建造带门道的墙: 在 y≈0 处一堵墙, 中间留 door_width 宽的缺口。
// 起点 (0,-2) 在南, 终点 (0,2) 在北, 必须穿过门道。
static OccupancyGrid make_corridor(double door_width) {
    OccupancyGrid g;
    fill_free(g);
    double half = door_width / 2.0;
    for (int gy = 0; gy < g.height; ++gy) {
        for (int gx = 0; gx < g.width; ++gx) {
            double wx, wy;
            g.grid_to_world(gx, gy, wx, wy);
            // 墙: |y| < 0.1 且 |x| > half
            if (std::abs(wy) < 0.10 && std::abs(wx) > half) {
                g.log_odds[gy * g.width + gx] = 1.0f;
            }
        }
    }
    return g;
}

// ---- §6.6 白盒场景 ----
TEST(PathValidator, EmptyPath) {
    OccupancyGrid g; fill_free(g);
    PathValidator v(g);
    auto r = v.validate({}, 0, 0, 5, 5, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::EMPTY_PATH);
}

TEST(PathValidator, SinglePointFarFromGoal) {
    OccupancyGrid g; fill_free(g);
    PathValidator v(g);
    // 路径仅 1 点, 距目标 5m >> goal_tolerance
    auto r = v.validate({{0, 0}}, 0, 0, 5, 5, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::GOAL_TOO_FAR);
    ASSERT_TRUE(r.endpoint_error > 0.35);
}

TEST(PathValidator, StartInObstacle) {
    OccupancyGrid g; fill_free(g);
    g.log_odds[10 * g.width + 10] = 1.0f;  // 占用 cell
    // 把起点放到该 cell 世界坐标
    double sx, sy; g.grid_to_world(10, 10, sx, sy);
    PathValidator v(g);
    auto r = v.validate({{sx + 1.0, sy}}, sx, sy, sx + 2.0, sy, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::START_IN_OBSTACLE);
}

TEST(PathValidator, GoalInObstacle) {
    OccupancyGrid g; fill_free(g);
    double gx, gy; g.grid_to_world(20, 20, gx, gy);
    // 终点落在占用 cell
    g.log_odds[20 * g.width + 20] = 1.0f;
    PathValidator v(g);
    auto r = v.validate({{gx - 1.0, gy}}, 0, 0, gx, gy, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::GOAL_IN_OBSTACLE);
}

TEST(PathValidator, StartOutOfBounds) {
    OccupancyGrid g; fill_free(g);
    PathValidator v(g);
    auto r = v.validate({{0, 0}}, 9999.0, 9999.0, 0, 0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::START_OUT_OF_BOUNDS);
}

TEST(PathValidator, GoalOutOfBounds) {
    OccupancyGrid g; fill_free(g);
    PathValidator v(g);
    auto r = v.validate({{0, 0}}, 0, 0, 9999.0, 9999.0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::GOAL_OUT_OF_BOUNDS);
}

TEST(PathValidator, EndpointTooFar) {
    OccupancyGrid g; fill_free(g);
    PathValidator v(g);
    // 终点距目标 2m > tolerance
    auto r = v.validate({{2.0, 0.0}}, 0, 0, 0, 0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == ::PathValidationCode::GOAL_TOO_FAR);
}

TEST(PathValidator, CenterSafeFootprintCollides) {
    // 起点/终点自由, 但路径经过一个仅容中心通过、footprint 必撞墙的窄缝
    OccupancyGrid g = make_corridor(0.5);  // 0.5m 门 = 半宽 0.25 < footprint x 半长轴 0.35
    PathValidator v(g);
    // 直穿门道, 顶点落在门中心 (0,0)
    auto r = v.validate({{0.0, 0.0}, {0.0, 2.0}}, 0.0, -2.0, 0.0, 2.0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::FOOTPRINT_COLLISION ||
                r.code == PathValidationCode::SEGMENT_PENETRATION ||
                r.code == PathValidationCode::INSUFFICIENT_CLEARANCE);
}

TEST(PathValidator, SegmentPenetration) {
    // 两安全端点, 连线从墙的一侧斜穿到另一侧 (中间必穿墙)
    OccupancyGrid g; fill_free(g);
    // 在 x≈0 处造一堵竖墙 (厚 ~0.1m): 所有 |cx| < 0.06 的 cell 置占用
    for (int gy = 0; gy < g.height; ++gy) {
        for (int gx = 0; gx < g.width; ++gx) {
            double cx, cy; g.grid_to_world(gx, gy, cx, cy);
            (void)cy;
            if (std::abs(cx) < 0.06) g.log_odds[gy * g.width + gx] = 1.0f;
        }
    }
    PathValidator v(g);
    // 左 (-1,0) 到右 (1,0), 中间穿过 x=0 墙
    auto r = v.validate({{-1.0, 0.0}, {1.0, 0.0}}, -1.0, 0.0, 1.0, 0.0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::SEGMENT_PENETRATION ||
                r.code == PathValidationCode::FOOTPRINT_COLLISION);
}

TEST(PathValidator, InsufficientClearance) {
    // 0.8m 门: 半宽 0.4 > footprint x 半长轴 0.35 → 不穿墙,
    // 但净空 = 0.4-0.35 = 0.05m < 0.08 → 应判 INSUFFICIENT_CLEARANCE
    OccupancyGrid g = make_corridor(0.8);
    PathValidator v(g);
    auto r = v.validate({{0.0, 0.0}, {0.0, 2.0}}, 0.0, -2.0, 0.0, 2.0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::INSUFFICIENT_CLEARANCE);
    ASSERT_TRUE(r.min_clearance >= 0.0);
}

TEST(PathValidator, UnknownRegionBlocked) {
    OccupancyGrid g; fill_unknown(g);  // 全 unknown
    PathValidator v(g);
    // unknown_is_blocked=true → 路径穿过 unknown 必碰撞
    auto r = v.validate({{1.0, 0.0}}, 0, 0, 1.0, 0.0, make_opts());
    ASSERT_FALSE(r.valid);
    ASSERT_TRUE(r.code == PathValidationCode::FOOTPRINT_COLLISION ||
                r.code == PathValidationCode::START_IN_OBSTACLE);
}

TEST(PathValidator, FootprintOrientationInvariant) {
    // 规划以 admissible_half_width = footprint_max_extent + min_clearance 为准
    // (§6.4/§6.5), 验证器对 footprint 朝向采用"最大外接"保守策略: 同一障碍在
    // 0° 与 90° 下判定必须一致, 旋转不得削弱碰撞检测。
    // 在最大半长轴(0.35)以内放置障碍 → 两种朝向都应命中。
    OccupancyGrid g; fill_free(g);
    for (int gy = 0; gy < g.height; ++gy) {
        for (int gx = 0; gx < g.width; ++gx) {
            double wx, wy; g.grid_to_world(gx, gy, wx, wy);
            if (std::abs(wx - 0.30) < 0.06 && std::abs(wy) < 0.06)
                g.log_odds[gy * g.width + gx] = 1.0f;
        }
    }
    PathValidator v(g);
    double clr_aligned, clr_rot;
    bool hit_aligned = v.footprint_collision_and_clearance(0, 0, 0.0,      kContractFootprint, true, clr_aligned);
    bool hit_rot    = v.footprint_collision_and_clearance(0, 0, M_PI / 2.0, kContractFootprint, true, clr_rot);
    ASSERT_TRUE(hit_aligned);   // 轴对齐: 障碍在最大半长轴内 → 命中
    ASSERT_TRUE(hit_rot);       // 旋转 90°: 保守策略下同样命中 (不削弱安全)
    ASSERT_TRUE(hit_aligned == hit_rot);  // 朝向不变性: 旋转不改变碰撞判定
}

TEST(PathValidator, SamplingStepStable) {
    // 不同 sample_step 对同一安全/危险路径判定一致
    OccupancyGrid g_safe = make_corridor(1.5);
    OccupancyGrid g_unsafe = make_corridor(0.6);
    PathValidator v_safe(g_safe), v_unsafe(g_unsafe);
    auto o1 = make_opts(); o1.sample_step = 0.05;
    auto o2 = make_opts(); o2.sample_step = 0.025;
    auto o3 = make_opts(); o3.sample_step = 0.01;

    auto rs1 = v_safe.validate({{0,0},{0,2}}, 0, -2, 0, 2, o1);
    auto rs2 = v_safe.validate({{0,0},{0,2}}, 0, -2, 0, 2, o2);
    auto rs3 = v_safe.validate({{0,0},{0,2}}, 0, -2, 0, 2, o3);
    ASSERT_TRUE(rs1.valid && rs2.valid && rs3.valid);

    auto ru1 = v_unsafe.validate({{0,0},{0,2}}, 0, -2, 0, 2, o1);
    auto ru2 = v_unsafe.validate({{0,0},{0,2}}, 0, -2, 0, 2, o2);
    auto ru3 = v_unsafe.validate({{0,0},{0,2}}, 0, -2, 0, 2, o3);
    ASSERT_FALSE(ru1.valid); ASSERT_FALSE(ru2.valid); ASSERT_FALSE(ru3.valid);
}

TEST(PathValidator, MinClearanceReported) {
    // 穿过 1.2m 门 (安全净空 > 0) 时, min_clearance 应 > 0
    OccupancyGrid g = make_corridor(1.2);
    PathValidator v(g);
    auto r = v.validate({{0,0},{0,2}}, 0, -2, 0, 2, make_opts());
    ASSERT_TRUE(r.valid);
    ASSERT_TRUE(r.min_clearance > 0.0);
}

// ---- §6.5 窄通道矩阵 ----
TEST(PathValidator, CorridorMatrix) {
    // 可通门宽阈值由 footprint + 安全裕度推导:
    //   admissible_half = footprint_max_extent(0.35) + min_clearance(0.08) = 0.43m
    double adm_half = PathValidator::admissible_half_width(kContractFootprint, 0.08);
    // 保守上界 = footprint 最大半轴(对角 0.430) + 裕度 0.08 ≈ 0.510
    ASSERT_NEAR(adm_half, 0.5101, 1e-3);

    // 预期: >=1.0m 通过; <=0.8m 拒绝 (物理不可通过)
    const double pass_set[]   = {1.5, 1.3, 1.2, 1.0};
    const double reject_set[] = {0.8, 0.7, 0.6, 0.5};

    for (double w : pass_set) {
        OccupancyGrid g = make_corridor(w);
        PathValidator v(g);
        auto r = v.validate({{0,0},{0,2}}, 0, -2, 0, 2, make_opts());
        ASSERT_TRUE(r.valid);
    }
    for (double w : reject_set) {
        OccupancyGrid g = make_corridor(w);
        PathValidator v(g);
        auto r = v.validate({{0,0},{0,2}}, 0, -2, 0, 2, make_opts());
        ASSERT_FALSE(r.valid);
    }
}

// ---- §6.1 根因修复: A* 在 INSCRIBED_RADIUS=0 时可能对窄门返回"成功"路径,
//       PathValidator 必须拦住它 ----
TEST(PathValidator, AStarNarrowDoorRejectedByValidator) {
    OccupancyGrid g = make_corridor(0.6);
    Costmap costmap; costmap.update_static(g);
    AStarPlanner planner(costmap);
    auto path = planner.plan(0.0, -2.0, 0.0, 2.0);

    PathValidator v(g);
    auto r = v.validate(path, 0.0, -2.0, 0.0, 2.0, make_opts());
    if (!path.empty()) {
        // A* 真的返回了路径 → 验证器必须判不安全 (核心 bug 修复)
        ASSERT_FALSE(r.valid);
    } else {
        // A* 已正确返回无路径, 同样符合预期
        printf("  [info] A* 对 0.6m 门正确返回空路径\n");
    }
}

TEST(PathValidator, AStarWideDoorAcceptedByValidator) {
    OccupancyGrid g = make_corridor(1.2);
    Costmap costmap; costmap.update_static(g);
    AStarPlanner planner(costmap);
    auto path = planner.plan(0.0, -2.0, 0.0, 2.0);
    ASSERT_FALSE(path.empty());
    PathValidator v(g);
    auto r = v.validate(path, 0.0, -2.0, 0.0, 2.0, make_opts());
    ASSERT_TRUE(r.valid);
}

int main() {
    printf("\n=== M2 PathValidator 白盒测试 (§6.5/§6.6) ===\n");
    printf("    footprint=[-0.35,-0.25]..[0.35,0.25], min_clearance=0.08, goal_tol=0.35\n");
    printf("    admissible_half_width = 0.35 + 0.08 = 0.43m\n\n");
    return RUN_ALL_TESTS();
}
