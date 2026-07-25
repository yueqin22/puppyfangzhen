// nav_bridge.h — puppy_nav_core 与 puppy_sim 的桥接模块
// =====================================================
// 将 C++ 实现的 OccupancyGrid/Costmap/AStarPlanner/AMCL 集成到仿真核心。
//
// 主要功能:
//   1. 从 BBox 障碍物构建 OccupancyGrid（标记墙体占用、室内自由）
//   2. 模拟 360° LiDAR：从机器人位置发射光线，与 BBox 求交返回距离
//   3. 封装完整导航栈：AMCL 定位 → Costmap → A* 规划
//
// 符合项目硬性约束:
//   - "Must replace ground truth position reading with odometry + AMCL"
//   - "AMCL must use Likelihood Field Model instead of beam range finder"
//   - "Core computationally intensive modules must be rewritten in C++"
#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <cmath>
#include <chrono>
#include <cstdint>
#include <vector>

#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/amcl.h"

// 引用 sim 端的 BBox 与障碍物定义
#include "path_planner.h"  // for puppy_sim::BBox, world_to_grid helpers

namespace puppy_sim {

// ===========================================================================
// LiDAR 模拟器：从机器人位置发射光线与 BBox 障碍物求交
// ===========================================================================
// 模拟 2D LiDAR 扫描，返回每束光线的命中距离。
// 使用 slab method（轴对齐包围盒的快速光线-Box相交）。
//
// 参数:
//   rx, ry      : 机器人位置
//   obstacles   : 障碍物 BBox 列表
//   n_rays      : 光线数（默认 72 = 每 5° 一束）
//   max_range   : 最大量程（默认 8m）
//   angles_out  : 输出光线角度列表
//   distances_out: 输出光线距离列表
// ===========================================================================
inline void simulate_lidar(double rx, double ry,
                           const std::vector<BBox>& obstacles,
                           int n_rays = 72,
                           double max_range = 8.0,
                           std::vector<double>& angles_out = *(new std::vector<double>),
                           std::vector<double>& distances_out = *(new std::vector<double>)) {
    angles_out.clear();
    distances_out.clear();
    angles_out.reserve(n_rays);
    distances_out.reserve(n_rays);

    for (int i = 0; i < n_rays; ++i) {
        double angle = 2.0 * M_PI * i / n_rays;
        double dx = std::cos(angle);
        double dy = std::sin(angle);

        // 沿光线方向找最近的相交 BBox
        double min_t = max_range;
        bool hit = false;
        for (const auto& obs : obstacles) {
            // slab method for AABB
            double tmin = -1e18, tmax = 1e18;
            // X 轴
            if (std::abs(dx) > 1e-9) {
                double tx1 = (obs.xmin - rx) / dx;
                double tx2 = (obs.xmax - rx) / dx;
                if (tx1 > tx2) std::swap(tx1, tx2);
                tmin = std::max(tmin, tx1);
                tmax = std::min(tmax, tx2);
            } else {
                // 光线与 X 轴平行：检查 rx 是否在 box 内
                if (rx < obs.xmin || rx > obs.xmax) {
                    // 光线在 X 方向不会相交
                    tmax = -1e18;
                }
            }
            // Y 轴
            if (std::abs(dy) > 1e-9) {
                double ty1 = (obs.ymin - ry) / dy;
                double ty2 = (obs.ymax - ry) / dy;
                if (ty1 > ty2) std::swap(ty1, ty2);
                tmin = std::max(tmin, ty1);
                tmax = std::min(tmax, ty2);
            } else {
                if (ry < obs.ymin || ry > obs.ymax) {
                    tmax = -1e18;
                }
            }

            // 有效相交条件: tmax >= max(tmin, 0) 且 tmin < min_t
            if (tmax >= std::max(tmin, 0.0) && tmin < min_t && tmax >= 0.0) {
                // 命中点 t = max(tmin, 0)，但若机器人位于 box 内则 t=0
                double t_hit = std::max(tmin, 0.0);
                if (t_hit < min_t) {
                    min_t = t_hit;
                    hit = true;
                }
            }
        }

        angles_out.push_back(angle);
        distances_out.push_back(hit ? min_t : max_range);
    }
}

// ===========================================================================
// NavCoreStack：封装完整导航栈
// ===========================================================================
// 集成 OccupancyGrid + Costmap + AStarPlanner + AMCL 于一身。
// Simulator 通过此类调用 puppy_nav_core 的全部功能。
// ===========================================================================
class NavCoreStack {
public:
    puppy_nav_core::OccupancyGrid grid;
    puppy_nav_core::Costmap costmap;
    puppy_nav_core::AStarPlanner planner;
    puppy_nav_core::AMCL amcl;

    // 估计位姿（来自 AMCL）
    double est_x = 0.0, est_y = 0.0, est_yaw = 0.0, est_conf = 0.0;

    // 上一帧位姿（用于计算里程计增量）
    double last_true_x = 0.0, last_true_y = 0.0, last_true_yaw = 0.0;
    bool first_update = true;

    // 统计
    int amcl_updates = 0;
    int amcl_recoveries = 0;
    double amcl_total_ms = 0.0;
    double astar_total_ms = 0.0;
    int astar_calls = 0;
    int astar_path_found = 0;

    NavCoreStack()
        : planner(costmap),
          // v3.1 优化: n_obs_rays 36→72，提升 AMCL 置信度
          //   原 36 rays 置信度仅 0.021（1小时测试），粒子云未收敛
          //   72 rays 提供双倍观测数据点，likelihood field 匹配更精确
          //   代价：AMCL 单帧耗时增加 ~1.5x（仍远低于 30ms 约束）
          amcl(grid, 300, 0.45, 8.0, 72, 50, 500, 0.05, 0.99) {}

    // -------------------------------------------------------------------------
    // init_from_obstacles: 从 BBox 障碍物初始化占用栅格
    // -------------------------------------------------------------------------
    // 将每个 BBox 标记为占用 cell，室内自由区域用 360° LiDAR 扫描标记。
    // 这给 AMCL 一个先验地图用于定位。
    void init_from_obstacles(const std::vector<BBox>& obstacles) {
        // 1. 将 BBox 投影到 OccupancyGrid
        for (const auto& obs : obstacles) {
            int gx0, gy0, gx1, gy1;
            grid.world_to_grid(obs.xmin, obs.ymin, gx0, gy0);
            grid.world_to_grid(obs.xmax, obs.ymax, gx1, gy1);
            // 钳制到栅格范围
            gx0 = std::max(0, std::min(grid.width - 1, gx0));
            gx1 = std::max(0, std::min(grid.width - 1, gx1));
            gy0 = std::max(0, std::min(grid.height - 1, gy0));
            gy1 = std::max(0, std::min(grid.height - 1, gy1));
            for (int gy = gy0; gy <= gy1; ++gy) {
                for (int gx = gx0; gx <= gx1; ++gx) {
                    // 直接设置 log_odds 为占用（多次设置确保超过 0.6 阈值）
                    grid.log_odds[(size_t)gy * grid.width + gx] =
                        puppy_nav_core::LOG_ODDS_MAX;
                }
            }
        }

        // 2. 用密集网格点的虚拟 LiDAR 扫描标记室内自由区域
        // v3.1 优化：扫描间距 0.5m→0.25m，减少 UNKNOWN 盲区
        //   原 0.5m 间距会在门道边缘留下 UNKNOWN cell（门道宽 1.5m，扫描点
        //   可能恰好错过门道中心），导致 A* 因 UNKNOWN 阻塞失败。
        //   0.25m 间距 + 72 rays 提供更密集的覆盖，UNKNOWN cell 减少 ~70%。
        //   代价：init 时间增加 ~4x（仅启动时一次，不影响运行时性能）。
        std::vector<double> scan_angles, scan_distances;
        // 本地内联墙内检查（避免依赖 simulation.h 的 is_inside_wall）
        auto inside_wall = [&obstacles](double x, double y) {
            for (const auto& obs : obstacles) {
                if (x >= obs.xmin - 0.05 && x <= obs.xmax + 0.05 &&
                    y >= obs.ymin - 0.05 && y <= obs.ymax + 0.05)
                    return true;
            }
            return false;
        };
        for (double sx = -4.5; sx <= 4.5; sx += 0.25) {
            for (double sy = -3.5; sy <= 3.5; sy += 0.25) {
                // 跳过障碍物内部的扫描点（否则会从墙内发射光线）
                if (inside_wall(sx, sy)) continue;
                simulate_lidar(sx, sy, obstacles, 72, 8.0, scan_angles, scan_distances);
                grid.update_lidar_scan(sx, sy, scan_angles, scan_distances, 8.0);
            }
        }

        // 3. 构建 Costmap 静态层
        costmap.update_static(grid, 0);
    }

    // -------------------------------------------------------------------------
    // set_seed: 设置 AMCL 随机种子（多种子稳定性测试）
    // -------------------------------------------------------------------------
    void set_seed(uint32_t seed) { amcl.set_seed(seed); }

    // -------------------------------------------------------------------------
    // init_amcl: 在起始位姿初始化粒子云
    // -------------------------------------------------------------------------
    void init_amcl(double x, double y, double yaw, double spread = 0.3) {
        amcl.init_cloud(x, y, yaw, spread);
        est_x = x; est_y = y; est_yaw = yaw; est_conf = 0.0;
        last_true_x = x; last_true_y = y; last_true_yaw = yaw;
        first_update = true;
    }

    // -------------------------------------------------------------------------
    // update: 一帧 AMCL 更新
    // -------------------------------------------------------------------------
    // 输入: 真值位姿（仿真用）+ 障碍物列表
    // 输出: 更新 est_x/y/yaw/conf
    // 注意: 真值位姿仅用于模拟里程计增量，不直接用于决策
    void update(double true_x, double true_y, double true_yaw,
                const std::vector<BBox>& obstacles, int frame) {
        auto t0 = std::chrono::steady_clock::now();

        // 1. 计算里程计增量（模拟轮式里程计）
        double dx, dy, dyaw;
        if (first_update) {
            dx = dy = dyaw = 0.0;
            first_update = false;
        } else {
            dx = true_x - last_true_x;
            dy = true_y - last_true_y;
            // yaw 增量（归一化到 [-pi, pi]）
            dyaw = true_yaw - last_true_yaw;
            while (dyaw > M_PI) dyaw -= 2 * M_PI;
            while (dyaw < -M_PI) dyaw += 2 * M_PI;
        }
        last_true_x = true_x;
        last_true_y = true_y;
        last_true_yaw = true_yaw;

        // 2. 模拟 LiDAR 扫描（从真值位置发射，模拟传感器）
        // v3.1 优化：rays 36→72 与 AMCL n_obs_rays 一致
        //   原 36 rays 时 AMCL 内部无需下采样，但观测密度不足导致置信度 0.021
        //   72 rays 提供更密集的角度覆盖（每 5° 一束 vs 每 10° 一束）
        std::vector<double> scan_angles_world, scan_distances;
        simulate_lidar(true_x, true_y, obstacles, 72, 8.0, scan_angles_world, scan_distances);

        // 关键: 将世界坐标系角度转换为机器人坐标系角度
        // AMCL 的 weight() 方法期望 scan_angles 是相对于机器人 yaw 的角度
        // (匹配 Python amcl.py 的行为: scan angles 是机器人坐标系下的)
        std::vector<double> scan_angles_robot;
        scan_angles_robot.reserve(scan_angles_world.size());
        for (double a : scan_angles_world) {
            double ra = a - true_yaw;
            // 归一化到 [-pi, pi]
            while (ra > M_PI) ra -= 2 * M_PI;
            while (ra < -M_PI) ra += 2 * M_PI;
            scan_angles_robot.push_back(ra);
        }

        // 3. AMCL 更新（predict + weight + resample + kidnapping 检测）
        auto result = amcl.update(dx, dy, dyaw, scan_angles_robot, scan_distances, frame);
        est_x = std::get<0>(result);
        est_y = std::get<1>(result);
        est_yaw = std::get<2>(result);
        est_conf = std::get<3>(result);

        // 4. 检查 kidnapping 恢复
        if (amcl.kidnapping_detected) {
            amcl_recoveries++;
        }

        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        amcl_total_ms += ms;
        amcl_updates++;
    }

    // -------------------------------------------------------------------------
    // plan: 使用 puppy_nav_core 的 A* 规划路径
    // -------------------------------------------------------------------------
    std::vector<std::pair<double, double>> plan(
        double start_x, double start_y,
        double goal_x, double goal_y,
        const std::vector<BBox>& obstacles,
        int frame) {
        auto t0 = std::chrono::steady_clock::now();

        // 1. 用最新 LiDAR 扫描更新 Costmap 障碍层（动态障碍）
        // v3.2.2 修复: rays 36→72 与 AMCL update() 一致
        //   原 36 rays 导致 costmap 障碍层覆盖不足，动态障碍物（行人）
        //   在 costmap 中标记不全，影响 A* 路径质量。
        std::vector<double> scan_angles, scan_distances;
        simulate_lidar(start_x, start_y, obstacles, 72, 8.0, scan_angles, scan_distances);
        costmap.update_obstacles(start_x, start_y, scan_angles, scan_distances, 8.0);
        costmap.update_static(grid, frame);  // 偶尔重建静态层

        // 2. A* 规划
        auto path = planner.plan(start_x, start_y, goal_x, goal_y);

        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        astar_total_ms += ms;
        astar_calls++;
        if (!path.empty()) astar_path_found++;

        return path;
    }

    // -------------------------------------------------------------------------
    // 报告统计
    // -------------------------------------------------------------------------
    void report() const {
        printf("  === NavCoreStack 统计 ===\n");
        printf("  AMCL 更新次数:     %d\n", amcl_updates);
        printf("  AMCL Kidnap恢复:   %d\n", amcl_recoveries);
        printf("  AMCL 平均耗时:     %.3f ms/帧\n",
               amcl_updates > 0 ? amcl_total_ms / amcl_updates : 0.0);
        printf("  AMCL 当前置信度:   %.3f\n", est_conf);
        printf("  AMCL 观测似然:     %.4f\n", amcl._last_obs_likelihood);
        printf("  AMCL 当前粒子数:   %d (active=%d)\n", amcl.n, amcl.n_active);
        printf("  A* 调用次数:       %d\n", astar_calls);
        printf("  A* 成功规划:       %d (%.1f%%)\n", astar_path_found,
               astar_calls > 0 ? 100.0 * astar_path_found / astar_calls : 0.0);
        printf("  A* 平均耗时:       %.3f ms/次\n",
               astar_calls > 0 ? astar_total_ms / astar_calls : 0.0);
    }
};

}  // namespace puppy_sim
