// Occupancy Grid Map — C++ 实现
// ===============================
// 2D 占用栅格地图，使用 Bresenham 光线追踪进行 LiDAR 建图。
// 分辨率：0.1m/cell（商用级，原始代码为 1m）。
//
// 数值约定：
//   log_odds > 0.6   = 占用（观测到障碍）
//   log_odds < -0.5  = 空闲（观测到空地）
//   |log_odds| < 0.3 = 未知（尚未观测）
//
// 基于 Gmapping 风格的占用栅格建图 + 逆传感器模型（log-odds 贝叶斯更新）。
#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <cstdint>

namespace puppy_nav_core {

// 栅格地图配置
constexpr int GRID_W = 100;              // 10m / 0.1m = 100 cells 宽
constexpr int GRID_H = 80;               // 8m / 0.1m = 80 cells 高
constexpr double GRID_RESOLUTION = 0.1;  // 米/单元
constexpr double ORIGIN_X = -5.0;        // grid (0,0) 对应的世界 x
constexpr double ORIGIN_Y = -4.0;        // grid (0,0) 对应的世界 y

// Log-odds 界限（用于稳定的贝叶斯建图）
constexpr float LOG_ODDS_MIN = -2.0f;
constexpr float LOG_ODDS_MAX = 3.5f;
constexpr float LOG_ODDS_HIT = 0.85f;    // 占用观测
constexpr float LOG_ODDS_MISS = -0.4f;   // 空闲观测
constexpr float LOG_ODDS_PRIOR = 0.0f;   // 未知

class OccupancyGrid {
public:
    std::vector<float> log_odds;    // (H*W) row-major, size = GRID_H * GRID_W
    std::vector<bool> visited;       // (H*W) row-major
    int width{GRID_W}, height{GRID_H};
    double resolution{GRID_RESOLUTION};
    double origin_x{ORIGIN_X}, origin_y{ORIGIN_Y};

    OccupancyGrid();

    // --- 坐标转换 ---
    void world_to_grid(double x, double y, int& gx, int& gy) const;
    void grid_to_world(int gx, int gy, double& x, double& y) const;

    // 直接索引访问（index = gy * GRID_W + gx）
    float& log_odds_at(int gx, int gy);
    float log_odds_at(int gx, int gy) const;
    bool is_in_bounds(int gx, int gy) const;
    bool is_occupied(int gx, int gy) const;  // log_odds > 0.6（边界外视为墙）
    bool is_unknown(int gx, int gy) const;    // |log_odds| < 0.3
    bool is_free(int gx, int gy) const;       // log_odds < -0.5（匹配 Python）

    // --- Bresenham 光线追踪（用于 LiDAR 更新）---
    void ray_trace(double x0, double y0, double x1, double y1);
    void update_occupied(double wx, double wy);
    void update_free(double wx, double wy);

    // --- 批量更新：一束 LiDAR 光线 ---
    void update_lidar_beam(double rx, double ry, double angle,
                           double distance, double max_range);
    void update_lidar_scan(double rx, double ry,
                           const std::vector<double>& angles,
                           const std::vector<double>& distances,
                           double max_range = 8.0);

    // --- 统计 ---
    int count_occupied() const;
    double mean_log_odds() const;
};

}  // namespace puppy_nav_core
