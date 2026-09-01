// Layered Costmap — 实现
// 对应 Python: costmap.py
// 完整保留 Python 算法逻辑（膨胀核预计算、未知单元处理、动态层衰减）。
#include "puppy_nav_core/costmap.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>

namespace puppy_nav_core {

Costmap::Costmap() {
    size_t total = static_cast<size_t>(GRID_H) * GRID_W;
    cost.assign(total, COST_FREE);
    static_cost.assign(total, COST_FREE);
    obstacle_cost.assign(total, COST_FREE);
    unknown_mask.assign(total, false);
    build_inflation_kernel();
}

// --- 膨胀核预计算 ---
// 对应 Python _build_inflation_kernel：为每个相对单元距离预计算膨胀代价。
void Costmap::build_inflation_kernel() {
    int size = 2 * INFLATION_CELLS + 1;
    kernel_size_ = size;
    inflation_kernel_.assign(static_cast<size_t>(size) * size, 0.0f);
    int c = INFLATION_CELLS;  // 核中心
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            int dx = j - c;
            int dy = i - c;
            double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution;
            float val = 0.0f;
            if (dist <= 0.0) {
                // 中心单元 = lethal
                val = static_cast<float>(COST_LETHAL);
            } else if (dist <= INSCRIBED_RADIUS) {
                // 内切半径内 = inscribed（机器人必然碰撞）
                val = static_cast<float>(COST_INSCRIBED);
            } else if (dist <= INFLATION_RADIUS) {
                // 膨胀区间 = 指数衰减
                double factor = std::exp(-COST_SCALE_FACTOR * (dist - INSCRIBED_RADIUS));
                val = static_cast<float>(COST_INSCRIBED * factor);
            } else {
                val = 0.0f;
            }
            inflation_kernel_[static_cast<size_t>(i) * size + j] = val;
        }
    }
}

// --- 膨胀 ---
// 对应 Python _inflate：用预计算核膨胀二值障碍 mask。
// 返回 float 数组（已 clip 到 [0, 254]），调用方按需转 uint8。
std::vector<float> Costmap::inflate(const std::vector<bool>& obstacle_mask) const {
    std::vector<float> inflated(static_cast<size_t>(GRID_H) * GRID_W, 0.0f);
    int half = INFLATION_CELLS;
    int size = kernel_size_;
    // 对每个障碍单元，将其核窗口叠加（取最大值）到 inflated
    for (int gy = 0; gy < GRID_H; ++gy) {
        for (int gx = 0; gx < GRID_W; ++gx) {
            if (!obstacle_mask[static_cast<size_t>(gy) * GRID_W + gx]) continue;
            // 计算合法的目标窗口与对应的核窗口
            int y0 = std::max(0, gy - half);
            int y1 = std::min(GRID_H, gy + half + 1);
            int x0 = std::max(0, gx - half);
            int x1 = std::min(GRID_W, gx + half + 1);
            int ky0 = half - (gy - y0);
            int ky1 = half + (y1 - gy);
            int kx0 = half - (gx - x0);
            int kx1 = half + (x1 - gx);
            // inflated[y0:y1, x0:x1] = max(inflated, kernel[ky0:ky1, kx0:kx1])
            for (int i = 0; i < (y1 - y0); ++i) {
                for (int j = 0; j < (x1 - x0); ++j) {
                    size_t dst = static_cast<size_t>(y0 + i) * GRID_W + (x0 + j);
                    size_t k = static_cast<size_t>(ky0 + i) * size + (kx0 + j);
                    if (inflation_kernel_[k] > inflated[dst]) {
                        inflated[dst] = inflation_kernel_[k];
                    }
                }
            }
        }
    }
    // clip 到 [0, 254]（匹配 Python np.clip(inflated, 0, 254)）
    for (auto& v : inflated) {
        if (v < 0.0f) v = 0.0f;
        else if (v > 254.0f) v = 254.0f;
    }
    return inflated;
}

// --- 静态层更新 ---
// 对应 Python update_static：从 OccupancyGrid 更新静态层（带脏标志优化）。
void Costmap::update_static(const OccupancyGrid& grid, int frame) {
    size_t total = static_cast<size_t>(GRID_H) * GRID_W;

    // 统计 occupied 单元
    std::vector<bool> occupied(total, false);
    int occupied_count = 0;
    for (size_t i = 0; i < total; ++i) {
        if (grid.log_odds[i] > 0.6f) {
            occupied[i] = true;
            ++occupied_count;
        }
    }

    // 脏标志判断：仅当 (1) 首次/显式脏、(2) occupied 数显著变化、
    // (3) 时间间隔到期 时重建（避免每帧全量重建）
    int diff = occupied_count - last_occupied_count_;
    bool count_changed = (diff < 0 ? -diff : diff) > 3;
    bool time_elapsed = (frame - last_static_update_frame_) >= 2;
    if (!(static_dirty_ || count_changed || time_elapsed)) {
        return;  // 跳过重建，当前 static_cost 仍然有效
    }

    std::vector<uint8_t> static_layer(total, COST_FREE);
    // occupied 单元 = lethal
    for (size_t i = 0; i < total; ++i) {
        if (occupied[i]) static_layer[i] = COST_LETHAL;
    }
    // 膨胀 occupied 障碍（仅在已知墙壁周围创建膨胀梯度）
    std::vector<float> inflated = inflate(occupied);
    // unknown 单元 = 中等惩罚(80)，防止 A* 穿过未映射墙壁。
    // 在膨胀之后应用，避免 unknown 单元被膨胀到 lethal。
    // cost 80 < INSCRIBED(128)，仍可通行但比 free(0) 强烈抑制。
    for (size_t i = 0; i < total; ++i) {
        if (std::abs(grid.log_odds[i]) < 0.3f) {
            unknown_mask[i] = true;
            static_layer[i] = 80;
        } else {
            unknown_mask[i] = false;
        }
    }
    // 合并：取膨胀障碍与 unknown 惩罚的最大值
    // (Python: np.maximum(inflated, static)，inflated 已 clip 并转 uint8)
    for (size_t i = 0; i < total; ++i) {
        uint8_t inf_val = static_cast<uint8_t>(inflated[i]);  // 已 clip 到 [0,254]
        uint8_t s = static_layer[i];
        static_cost[i] = (inf_val > s) ? inf_val : s;
    }

    // ===== R19i BUG#4 正确修正: 仅在 Costmap 边缘标记 LETHAL(避免A*计划到地图外), =====
    //   同时 UE ClampToHomeBounds 设置为 costmap 边界 - 2cm = 永不超出地图.
    //   R19g的5 cells (=0.5m) 太大了 = 把室内0.5m通道变成墙(导致R19i机器人沿外圈走不进门道!).
    //   1 cell (=0.1m)才是正确的: 仅最外1格=地图真正边缘 (x∈[-5,-4.9]和[4.9,5.0], y同理).
    //   效果: A*不规划到地图真实边缘外, UE侧clamp (-4.98,4.98)(-3.98,3.98)对齐.
    {
        constexpr int MARGIN = 1;  // 0.1m = 最外1格 (NOT 5格!)
        for (int gy = 0; gy < GRID_H; ++gy) {
            for (int gx = 0; gx < GRID_W; ++gx) {
                if (gx < MARGIN || gx >= (GRID_W - MARGIN) ||
                    gy < MARGIN || gy >= (GRID_H - MARGIN)) {
                    static_cost[static_cast<size_t>(gy) * GRID_W + gx] = COST_LETHAL;
                }
            }
        }
    }
    merge();
    last_occupied_count_ = occupied_count;
    last_static_update_frame_ = frame;
    static_dirty_ = false;
}

// --- 动态障碍层更新 ---
// 对应 Python update_obstacles：从实时 LiDAR 扫描更新动态障碍层。
void Costmap::update_obstacles(double rx, double ry,
                               const std::vector<double>& angles,
                               const std::vector<double>& distances,
                               double max_range) {
    size_t total = static_cast<size_t>(GRID_H) * GRID_W;
    // 衰减先前障碍（动态层是短期的）：max(cost-10, 0)
    // (Python: np.maximum(obstacle_cost.astype(int16)-10, 0).astype(uint8))
    for (size_t i = 0; i < total; ++i) {
        int v = static_cast<int>(obstacle_cost[i]) - 10;
        obstacle_cost[i] = (v < 0) ? 0 : static_cast<uint8_t>(v);
    }

    // 标记当前 LiDAR 命中为障碍
    std::vector<bool> obstacle_mask(total, false);
    size_t n = std::min(angles.size(), distances.size());
    for (size_t i = 0; i < n; ++i) {
        double dist = distances[i];
        double angle = angles[i];
        if (dist >= max_range || !std::isfinite(dist) || dist < 0.1) {
            continue;
        }
        double hx = rx + dist * std::cos(angle);
        double hy = ry + dist * std::sin(angle);
        int gx, gy;
        world_to_grid(hx, hy, gx, gy);
        if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
            obstacle_mask[static_cast<size_t>(gy) * GRID_W + gx] = true;
        }
    }

    // v3.2.2 修复: 动态障碍用软代价(100)而非膨胀硬阻塞
    //   原行为: 膨胀(INFLATION_RADIUS=0.15m)后 COST_INSCRIBED(128) >= PLAN_BLOCKED(110)
    //   导致行人在窄门道处膨胀区切断通路，A* 失败率 18% (nopath)。
    //   新行为: 动态障碍命中点设 cost=100 (低于 PLAN_BLOCKED=110)，
    //   A* 视为高代价但可通行，始终能找到路径但尽量避开行人。
    //   静态障碍(墙)仍用膨胀，保证不穿墙。
    //   cell_cost() 中 cost=100 → 1 + (100/110)^2 * 12 = 10.9，约为自由cell的11倍。
    for (size_t i = 0; i < total; ++i) {
        if (obstacle_mask[i]) {
            obstacle_cost[i] = 100;  // 软代价，低于 PLAN_BLOCKED(110)
        }
    }
    merge();
}

// --- 合并所有层 ---
// 对应 Python _merge：主代价 = max(静态层, 障碍层)
void Costmap::merge() {
    size_t total = static_cast<size_t>(GRID_H) * GRID_W;
    for (size_t i = 0; i < total; ++i) {
        cost[i] = (static_cost[i] > obstacle_cost[i]) ? static_cost[i]
                                                       : obstacle_cost[i];
    }
}

// --- 代价查询 ---
uint8_t Costmap::get_cost(double x, double y) const {
    // Python: int() 截断（非 floor）
    int gx = static_cast<int>((x - origin_x) / resolution);
    int gy = static_cast<int>((y - origin_y) / resolution);
    if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
        return cost[static_cast<size_t>(gy) * GRID_W + gx];
    }
    return COST_LETHAL;  // 边界外 = lethal
}

uint8_t Costmap::get_cost_grid(int gx, int gy) const {
    if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
        return cost[static_cast<size_t>(gy) * GRID_W + gx];
    }
    return COST_LETHAL;
}

bool Costmap::is_unknown_grid(int gx, int gy) const {
    if (gx >= 0 && gx < GRID_W && gy >= 0 && gy < GRID_H) {
        return unknown_mask[static_cast<size_t>(gy) * GRID_W + gx];
    }
    return false;
}

bool Costmap::is_lethal(double x, double y) const {
    // 位置在障碍内或过于靠近障碍
    return get_cost(x, y) >= COST_INSCRIBED;
}

bool Costmap::is_safe(double x, double y, uint8_t threshold) const {
    // 位置代价低于阈值（可通行）
    return get_cost(x, y) < threshold;
}

// --- 坐标转换 ---
void Costmap::world_to_grid(double x, double y, int& gx, int& gy) const {
    // Python Costmap.world_to_grid 使用 int()（截断向零），
    // 区别于 OccupancyGrid.world_to_grid 的 floor。
    gx = static_cast<int>((x - origin_x) / resolution);
    gy = static_cast<int>((y - origin_y) / resolution);
}

void Costmap::grid_to_world(int gx, int gy, double& x, double& y) const {
    x = origin_x + (gx + 0.5) * resolution;
    y = origin_y + (gy + 0.5) * resolution;
}

}  // namespace puppy_nav_core
