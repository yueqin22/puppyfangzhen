// Occupancy Grid Map — 实现
// 对应 Python: occupancy_grid.py
// 完整保留 Python 算法逻辑（log-odds 贝叶斯更新 + Bresenham 光线追踪）。
#include "puppy_nav_core/occupancy_grid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>

namespace puppy_nav_core {

OccupancyGrid::OccupancyGrid() {
    log_odds.assign(static_cast<size_t>(GRID_H) * GRID_W, LOG_ODDS_PRIOR);
    visited.assign(static_cast<size_t>(GRID_H) * GRID_W, false);
}

// --- 坐标转换 ---
void OccupancyGrid::world_to_grid(double x, double y, int& gx, int& gy) const {
    // Python 使用 math.floor（区别于 Costmap.world_to_grid 的 int() 截断）
    gx = static_cast<int>(std::floor((x - origin_x) / resolution));
    gy = static_cast<int>(std::floor((y - origin_y) / resolution));
}

void OccupancyGrid::grid_to_world(int gx, int gy, double& x, double& y) const {
    // 返回单元中心
    x = origin_x + (gx + 0.5) * resolution;
    y = origin_y + (gy + 0.5) * resolution;
}

// --- 索引访问 ---
float& OccupancyGrid::log_odds_at(int gx, int gy) {
    return log_odds[static_cast<size_t>(gy) * GRID_W + gx];
}

float OccupancyGrid::log_odds_at(int gx, int gy) const {
    return log_odds[static_cast<size_t>(gy) * GRID_W + gx];
}

bool OccupancyGrid::is_in_bounds(int gx, int gy) const {
    return gx >= 0 && gx < width && gy >= 0 && gy < height;
}

bool OccupancyGrid::is_occupied(int gx, int gy) const {
    // Python: 边界外返回 True（视为墙）
    if (!is_in_bounds(gx, gy)) return true;
    return log_odds[static_cast<size_t>(gy) * GRID_W + gx] > 0.6f;
}

bool OccupancyGrid::is_unknown(int gx, int gy) const {
    if (!is_in_bounds(gx, gy)) return false;
    return std::abs(log_odds[static_cast<size_t>(gy) * GRID_W + gx]) < 0.3f;
}

bool OccupancyGrid::is_free(int gx, int gy) const {
    // Python is_free 使用 < -0.5（与 find_frontiers 的 free_mask 一致）
    if (!is_in_bounds(gx, gy)) return false;
    return log_odds[static_cast<size_t>(gy) * GRID_W + gx] < -0.5f;
}

// --- Bresenham 光线追踪 ---
void OccupancyGrid::ray_trace(double x0, double y0, double x1, double y1) {
    // 从世界坐标 (x0,y0) 到 (x1,y1) 追踪一条 Bresenham 线，
    // 沿线所有单元标记为 FREE（log-odds MISS 更新，钳制到 LOG_ODDS_MIN）。
    // 出界即停止（匹配 Python update_from_scan 的 in_bounds 中断行为）。
    int gx0, gy0, gx1, gy1;
    world_to_grid(x0, y0, gx0, gy0);
    world_to_grid(x1, y1, gx1, gy1);

    int dx = std::abs(gx1 - gx0);
    int dy = std::abs(gy1 - gy0);
    int sx = (gx0 < gx1) ? 1 : -1;
    int sy = (gy0 < gy1) ? 1 : -1;
    int err = dx - dy;
    int gx = gx0, gy = gy0;
    while (true) {
        if (!is_in_bounds(gx, gy)) break;
        float& lo = log_odds[static_cast<size_t>(gy) * GRID_W + gx];
        lo = std::max(lo + LOG_ODDS_MISS, LOG_ODDS_MIN);
        if (gx == gx1 && gy == gy1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; gx += sx; }
        if (e2 < dx)  { err += dx; gy += sy; }
    }
}

void OccupancyGrid::update_occupied(double wx, double wy) {
    int gx, gy;
    world_to_grid(wx, wy, gx, gy);
    if (!is_in_bounds(gx, gy)) return;
    float& lo = log_odds[static_cast<size_t>(gy) * GRID_W + gx];
    lo = std::min(lo + LOG_ODDS_HIT, LOG_ODDS_MAX);
}

void OccupancyGrid::update_free(double wx, double wy) {
    int gx, gy;
    world_to_grid(wx, wy, gx, gy);
    if (!is_in_bounds(gx, gy)) return;
    float& lo = log_odds[static_cast<size_t>(gy) * GRID_W + gx];
    lo = std::max(lo + LOG_ODDS_MISS, LOG_ODDS_MIN);
}

// --- LiDAR 批量更新 ---
void OccupancyGrid::update_lidar_beam(double rx, double ry, double angle,
                                      double distance, double max_range) {
    // 对应 Python update_from_scan 中单条光线的逻辑：
    //   - 沿光线（机器人 -> 命中点）的单元标记为 FREE
    //   - 命中点单元标记为 OCCUPIED（若命中真实障碍）
    //   - 若光线达到 max_range（无命中），所有单元均为 FREE
    // 使用 log-odds 更新以获得概率稳定性。
    int rgx, rgy;
    world_to_grid(rx, ry, rgx, rgy);
    if (!is_in_bounds(rgx, rgy)) return;

    double dist = distance;
    bool hit_is_obstacle;
    if (dist >= max_range || !std::isfinite(dist)) {
        dist = max_range;
        hit_is_obstacle = false;
    } else {
        hit_is_obstacle = true;
    }

    double hx = rx + dist * std::cos(angle);
    double hy = ry + dist * std::sin(angle);
    int hgx, hgy;
    world_to_grid(hx, hy, hgx, hgy);

    // Bresenham 从机器人到命中点
    int dx = std::abs(hgx - rgx);
    int dy = std::abs(hgy - rgy);
    int sx = (rgx < hgx) ? 1 : -1;
    int sy = (rgy < hgy) ? 1 : -1;
    int err = dx - dy;
    int gx = rgx, gy = rgy;
    while (true) {
        if (!is_in_bounds(gx, gy)) break;
        float& lo = log_odds[static_cast<size_t>(gy) * GRID_W + gx];
        if (gx == hgx && gy == hgy && hit_is_obstacle) {
            lo = std::min(lo + LOG_ODDS_HIT, LOG_ODDS_MAX);
        } else {
            lo = std::max(lo + LOG_ODDS_MISS, LOG_ODDS_MIN);
        }
        if (gx == hgx && gy == hgy) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; gx += sx; }
        if (e2 < dx)  { err += dx; gy += sy; }
    }
}

void OccupancyGrid::update_lidar_scan(double rx, double ry,
                                      const std::vector<double>& angles,
                                      const std::vector<double>& distances,
                                      double max_range) {
    size_t n = std::min(angles.size(), distances.size());
    for (size_t i = 0; i < n; ++i) {
        update_lidar_beam(rx, ry, angles[i], distances[i], max_range);
    }
}

// --- 统计 ---
int OccupancyGrid::count_occupied() const {
    int cnt = 0;
    size_t total = static_cast<size_t>(GRID_H) * GRID_W;
    for (size_t i = 0; i < total; ++i) {
        if (log_odds[i] > 0.6f) ++cnt;
    }
    return cnt;
}

double OccupancyGrid::mean_log_odds() const {
    double sum = 0.0;
    size_t total = static_cast<size_t>(GRID_H) * GRID_W;
    for (size_t i = 0; i < total; ++i) {
        sum += log_odds[i];
    }
    return sum / static_cast<double>(total);
}

}  // namespace puppy_nav_core
