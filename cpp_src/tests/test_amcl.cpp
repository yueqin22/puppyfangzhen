// test_amcl.cpp — AMCL 单元测试
// 覆盖: 聚类估计、限幅平滑(MAX_STEP=0.3m)、scan_score 归一化 [0,1]、
//       predict 运动模型、KLD 粒子数范围、resample 稳定性
//
// 测试策略: AMCL 的 particles_/weights_ 为私有，通过公共 API 驱动:
//   - init_cloud(x,y,yaw,spread) 初始化粒子云（不重置 cluster_prev_ 状态）
//   - get_cluster_estimate() 返回 (x,y,yaw,conf)
//   - get_particles() const 返回粒子只读视图
//   - _last_obs_likelihood 公开成员反映 scan_score
//   - predict(dx,dy,dyaw) 移动粒子
#include "test_framework.h"
#include "puppy_nav_core/amcl.h"
#include "puppy_nav_core/occupancy_grid.h"

#include <cmath>
#include <vector>

using namespace puppy_nav_core;

// ---- 测试辅助: 构造一个带障碍物的简单地图 ----
// 在地图中央放一面墙，用于 likelihood field 构建
static OccupancyGrid make_test_grid() {
    OccupancyGrid grid;
    grid.log_odds.assign(GRID_W * GRID_H, 0.0f);
    // 在 x=2.0 处放一列占用（墙），长度 2m
    int wall_gx, gy1, gy2;
    grid.world_to_grid(2.0, -1.0, wall_gx, gy1);
    grid.world_to_grid(2.0, 1.0, wall_gx, gy2);
    for (int gy = gy1; gy <= gy2; gy++) {
        grid.log_odds_at(wall_gx, gy) = 2.0f;
    }
    // 在 x=-2.0 处放另一面墙
    grid.world_to_grid(-2.0, -1.0, wall_gx, gy1);
    grid.world_to_grid(-2.0, 1.0, wall_gx, gy2);
    for (int gy = gy1; gy <= gy2; gy++) {
        grid.log_odds_at(wall_gx, gy) = 2.0f;
    }
    return grid;
}

// ---- 测试辅助: 生成模拟 LiDAR 扫描 ----
// 机器人位于原点，前方 2m 有墙 → 应在 angle=0 处返回 ~2.0m
static std::pair<std::vector<double>, std::vector<double>>
make_scan_towards_wall() {
    std::vector<double> angles, dists;
    // 72 束，覆盖 [0, 2π)
    int N = 72;
    for (int i = 0; i < N; i++) {
        double a = (double)i * 2.0 * M_PI / N;
        angles.push_back(a);
        // 前方(±0.1rad)命中 2m 墙，其余打到 max_range(8.0)
        double da = std::atan2(std::sin(a), std::cos(a));
        if (std::abs(da) < 0.15) {
            dists.push_back(2.0);
        } else {
            dists.push_back(8.0);  // z_max = 未命中
        }
    }
    return {angles, dists};
}

// ============================================================
// 1. init_cloud + get_cluster_estimate: 单簇场景估计正确
// ============================================================
TEST(AMCL, InitCloudSingleCluster) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.set_seed(42);

    amcl.init_cloud(0.0, 0.0, 0.0, 0.1);

    auto [x, y, yaw, conf] = amcl.get_cluster_estimate();
    // 估计应接近原点（spread=0.1，粒子集中在 0.1m 内）
    ASSERT_NEAR(x, 0.0, 0.2);
    ASSERT_NEAR(y, 0.0, 0.2);
    // 置信度应在 [0, 1]
    ASSERT_TRUE(conf >= 0.0);
    ASSERT_TRUE(conf <= 1.0);
}

// ============================================================
// 2. 限幅平滑 MAX_STEP=0.3m: 簇中心跳变 >0.3m 时被钳制
// ============================================================
// 原理: init_cloud 不重置 cluster_prev_x_/y_ 状态。
//   第一次 get_cluster_estimate() 设置 cluster_prev=(0,0)。
//   第二次 init_cloud(2.0,0,...) 把粒子移到 (2,0)，
//   但 cluster_prev 仍为 (0,0)，限幅使估计最多移动 0.3m → (~0.3, 0)。
TEST(AMCL, ClusterLimitStepClamps) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.set_seed(42);

    // 第一帧: 粒子在 (0,0)，设置 cluster_prev=(0,0)
    amcl.init_cloud(0.0, 0.0, 0.0, 0.05);
    auto [x1, y1, yaw1, conf1] = amcl.get_cluster_estimate();
    ASSERT_NEAR(x1, 0.0, 0.15);

    // 第二帧: 粒子移到 (2,0)，但 cluster_prev=(0,0)，
    // 限幅 MAX_STEP=0.3 应将估计钳制到 ~(0.3, 0)
    amcl.init_cloud(2.0, 0.0, 0.0, 0.05);
    auto [x2, y2, yaw2, conf2] = amcl.get_cluster_estimate();
    // 估计 x 应在 [0, 0.3+容差] 之间，远小于 2.0
    ASSERT_TRUE(x2 <= 0.35);
    ASSERT_TRUE(x2 >= 0.0);
}

// ============================================================
// 3. 限幅平滑: 正常小移动（<0.3m）不受钳制
// ============================================================
TEST(AMCL, ClusterNormalStepNotClamped) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.set_seed(42);

    amcl.init_cloud(0.0, 0.0, 0.0, 0.05);
    auto [x1, y1, yaw1, conf1] = amcl.get_cluster_estimate();

    // 移动 0.1m（< MAX_STEP=0.3），不应被钳制
    amcl.init_cloud(0.1, 0.0, 0.0, 0.05);
    auto [x2, y2, yaw2, conf2] = amcl.get_cluster_estimate();
    ASSERT_NEAR(x2, 0.1, 0.1);
}

// ============================================================
// 4. scan_score 归一化: _last_obs_likelihood 在合理范围
// ============================================================
// 调用 weight() 后 _last_obs_likelihood 应 > 0，
// 且 get_cluster_estimate 的 confidence ∈ [0, 1]
TEST(AMCL, ScanScoreNormalized) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.1);

    auto [angles, dists] = make_scan_towards_wall();
    amcl.weight(angles, dists, 0);

    // _last_obs_likelihood 是 exp(best_log_lik / n_beams)，应 > 0
    ASSERT_TRUE(amcl._last_obs_likelihood > 0.0);
    // 归一化后的 scan_score 上限 = z_hit + z_rand/z_max = 0.9 + 0.0125 = 0.9125
    // raw 值不应超过此上限（归一化前）
    ASSERT_TRUE(amcl._last_obs_likelihood <= 1.0);

    // 置信度 ∈ [0, 1]
    auto [x, y, yaw, conf] = amcl.get_cluster_estimate();
    ASSERT_TRUE(conf >= 0.0);
    ASSERT_TRUE(conf <= 1.0);
}

// ============================================================
// 5. predict 运动模型: 粒子整体移动
// ============================================================
TEST(AMCL, PredictMovesParticles) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 100);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.01);  // 极小 spread

    // 记录移动前粒子均值
    const auto& p_before = amcl.get_particles();
    double mean_x_before = 0;
    for (const auto& p : p_before) mean_x_before += p[0];
    mean_x_before /= p_before.size();

    // 移动 dx=1.0
    amcl.predict(1.0, 0.0, 0.0);

    const auto& p_after = amcl.get_particles();
    double mean_x_after = 0;
    for (const auto& p : p_after) mean_x_after += p[0];
    mean_x_after /= p_after.size();

    // 均值应移动约 1.0m（噪声存在，容差 0.2）
    ASSERT_NEAR(mean_x_after - mean_x_before, 1.0, 0.2);
}

// ============================================================
// 6. predict 噪声: 移动后粒子有分散
// ============================================================
TEST(AMCL, PredictAddsNoise) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 100);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.001);  // 几乎点状

    // 移动前粒子 x 标准差 ~ 0
    const auto& p0 = amcl.get_particles();
    double var0 = 0;
    for (const auto& p : p0) var0 += p[0] * p[0];
    var0 = var0 / p0.size();

    // 大角度移动引入噪声（alpha1*|dyaw|）
    amcl.predict(0.0, 0.0, 3.14);

    const auto& p1 = amcl.get_particles();
    double var1 = 0;
    for (const auto& p : p1) var1 += p[0] * p[0];
    var1 = var1 / p1.size();

    // 噪声使方差增大
    ASSERT_TRUE(var1 > var0);
}

// ============================================================
// 7. KLD 粒子数范围: n_active ∈ [kld_min, kld_max]
// ============================================================
TEST(AMCL, KLDParticleCountRange) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.1);

    auto [angles, dists] = make_scan_towards_wall();
    amcl.update(0.1, 0.0, 0.0, angles, dists, 1);

    // KLD 采样后 n_active 应在 [50, 500]
    ASSERT_TRUE(amcl.n_active >= 50);
    ASSERT_TRUE(amcl.n_active <= 500);
}

// ============================================================
// 8. resample 不崩溃且保持粒子数
// ============================================================
TEST(AMCL, ResampleStable) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 100);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.3);

    auto [angles, dists] = make_scan_towards_wall();
    amcl.weight(angles, dists, 0);
    amcl.resample();

    // 粒子数不变
    ASSERT_EQ((int)amcl.get_particles().size(), 100);
}

// ============================================================
// 9. get_estimate 加权均值与 get_cluster_estimate 一致性（单簇）
// ============================================================
// 单簇场景下，加权均值和聚类估计应接近
TEST(AMCL, EstimateConsistencySingleCluster) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 200, 0.45, 8.0, 36, 50, 200);
    amcl.set_seed(42);
    amcl.init_cloud(1.0, 0.5, 0.3, 0.1);

    // 重置 cluster 状态（新构造的 AMCL cluster_initialized_=false）
    auto [xc, yc, yawc, confc] = amcl.get_cluster_estimate();
    auto [xe, ye, yawe, confe] = amcl.get_estimate();

    // 单簇场景两者应接近
    EXPECT_NEAR(xc, xe, 0.15);
    EXPECT_NEAR(yc, ye, 0.15);
}

// ============================================================
// 10. recover 分散粒子后估计仍在合理范围
// ============================================================
TEST(AMCL, RecoverDisperses) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.1);

    // recover 后粒子应分散（spread=2.0）
    amcl.recover(0.0, 0.0, 0.0, 2.0);

    const auto& p = amcl.get_particles();
    // 计算粒子云的标准差，recover 后应较大
    double mean_x = 0;
    for (const auto& pt : p) mean_x += pt[0];
    mean_x /= p.size();
    double var_x = 0;
    for (const auto& pt : p) var_x += (pt[0] - mean_x) * (pt[0] - mean_x);
    var_x /= p.size();
    double std_x = std::sqrt(var_x);

    // spread=2.0 → 标准差应 > 0.5
    ASSERT_TRUE(std_x > 0.5);
}

// ============================================================
// 11. 完整 update 循环不崩溃，返回有效位姿
// ============================================================
TEST(AMCL, UpdateCycleValid) {
    OccupancyGrid grid = make_test_grid();
    AMCL amcl(grid, 200, 0.45, 8.0, 36, 50, 200);
    amcl.set_seed(42);
    amcl.init_cloud(0.0, 0.0, 0.0, 0.2);

    auto [angles, dists] = make_scan_towards_wall();

    // 连续 10 帧 update
    for (int frame = 1; frame <= 10; frame++) {
        auto [x, y, yaw, conf] = amcl.update(0.05, 0.0, 0.0, angles, dists, frame);
        ASSERT_TRUE(conf >= 0.0);
        ASSERT_TRUE(conf <= 1.0);
        // 位姿应在地图范围内 [-5, 5] x [-4, 4]
        ASSERT_TRUE(x >= -5.0 && x <= 5.0);
        ASSERT_TRUE(y >= -4.0 && y <= 4.0);
    }
}

int main() {
    return RUN_ALL_TESTS();
}
