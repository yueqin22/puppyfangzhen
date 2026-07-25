// Layered Costmap — C++ 实现
// ===========================
// Nav2 风格的分层代价地图，用于路径规划与避障。
//
// 分层：
//   1. 静态层  — 来自 OccupancyGrid（已知障碍）
//   2. 障碍层  — 来自实时 LiDAR（动态障碍）
//   3. 膨胀层  — 按机器人半径扩张障碍（代价梯度）
//
// 代价值（Nav2 约定）：
//   0       = 空闲
//   1-127   = 膨胀梯度（远离障碍时代价递减）
//   128     = 内切半径（机器人必然碰撞）
//   254     = lethal（障碍）
//   255     = 无信息
#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <cstdint>

#include "puppy_nav_core/occupancy_grid.h"

namespace puppy_nav_core {

// 代价常量（Nav2 约定）
constexpr uint8_t COST_FREE = 0;
constexpr uint8_t COST_INSCRIBED = 128;
constexpr uint8_t COST_LETHAL = 254;
constexpr uint8_t COST_UNKNOWN = 255;

// 机器人参数
constexpr double ROBOT_RADIUS = 0.35;       // 实际机器人半径，用于碰撞检测
// v3.2.2 修复: INSCRIBED_RADIUS 0.25→0.0
//   原 0.25m > INFLATION_RADIUS(0.15m) 是逻辑 bug——内切半径不应大于膨胀半径。
//   导致 3x3 膨胀核全在 INSCRIBED 区(128≥PLAN_BLOCKED=110)，A* 被膨胀区硬阻断。
//   设为 0.0 让膨胀核只有 LETHAL 中心 + 指数衰减区(最大 100<110 可通行)，
//   cell_cost 二次惩罚仍让墙边代价高(10.9 vs 自由 1.0)，A* 会尽量避开。
constexpr double INSCRIBED_RADIUS = 0.0;   // 内切半径(0=仅LETHAL阻塞,衰减区可通行)
// NOTE: INSCRIBED_RADIUS < ROBOT_RADIUS 故意为之。门道宽 2m，但 wall_divide
// 厚度(0.1m) + 膨胀使得可通行走廊变窄。若 INSCRIBED_RADIUS=0.35，机器人在
// (1.35,-0.40) 处距 wall_divide_2(y=[-0.05,0.05]) 正好 0.35m，命中
// COST_INSCRIBED=128，导致 DWA 拒绝该位置的所有轨迹（碰撞检测
// cost >= COST_INSCRIBED），机器人卡住且 RECOVER 将其推回开阔区 → 门道振荡。
// 使用 0.25m 给机器人足够的门道通行间隙，同时保留安全裕度（CoppeliaSim
// 用位置控制，无真实物理碰撞）。
// v3.0: INFLATION_RADIUS 0.55→0.15 — 项目硬性约束要求 0.15m 以穿过 1m 门道。
//   仿真场景门道宽 1.5m，墙厚 0.3m，原 0.55m 膨胀使门道可通行宽度仅 0.4m，
//   A* 无法在卧室/书房/卫生间之间找到路径。0.15m 给门道留 1.2m 可通行宽度。
constexpr double INFLATION_RADIUS = 0.15;   // 膨胀距离（项目硬性约束：0.15m 穿 1m 门道）
constexpr int INFLATION_CELLS = static_cast<int>(INFLATION_RADIUS / GRID_RESOLUTION);
constexpr int INSCRIBED_CELLS = static_cast<int>(INSCRIBED_RADIUS / GRID_RESOLUTION);
constexpr double COST_SCALE_FACTOR = 2.5;   // 越大衰减越陡

class Costmap {
public:
    int width{GRID_W}, height{GRID_H};
    double resolution{GRID_RESOLUTION};
    double origin_x{ORIGIN_X}, origin_y{ORIGIN_Y};

    std::vector<uint8_t> cost;              // (H*W) row-major，主代价地图（合并层）
    std::vector<uint8_t> static_cost;       // 静态层（来自占用栅格）
    std::vector<uint8_t> obstacle_cost;     // 动态障碍层（来自实时 LiDAR）
    std::vector<bool> unknown_mask;         // 未被 LiDAR 观测的单元

    Costmap();

    // Python 接口对应
    void update_static(const OccupancyGrid& grid, int frame = 0);
    void update_obstacles(double rx, double ry,
                          const std::vector<double>& angles,
                          const std::vector<double>& distances,
                          double max_range = 8.0);

    uint8_t get_cost(double x, double y) const;
    uint8_t get_cost_grid(int gx, int gy) const;
    bool is_unknown_grid(int gx, int gy) const;
    bool is_lethal(double x, double y) const;
    bool is_safe(double x, double y, uint8_t threshold = 60) const;

    void world_to_grid(double x, double y, int& gx, int& gy) const;
    void grid_to_world(int gx, int gy, double& x, double& y) const;

private:
    std::vector<float> inflation_kernel_;   // 预计算膨胀核
    int kernel_size_{2 * INFLATION_CELLS + 1};
    int last_occupied_count_{0};
    int last_static_update_frame_{-100};
    bool static_dirty_{true};

    void build_inflation_kernel();
    std::vector<float> inflate(const std::vector<bool>& obstacle_mask) const;
    void merge();
};

}  // namespace puppy_nav_core
