#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <utility>
#include "puppy_nav_core/costmap.h"

namespace puppy_nav_core {

// 规划阈值：A* 将 cost >= 110 的 cell 视为阻塞。
// 此值低于 COST_INSCRIBED(128)，使 A* 避免通过膨胀高但仍可通行的狭窄缺口。
constexpr int PLAN_BLOCKED = 110;

// A* 放弃前最大扩展节点数（防止大网格上的失控搜索）
// v3.2.2 修复: 2000→8000 — 栅格 100x80=8000 cells，原 2000 只能探索 25%，
//   导致长距离规划（dock→bedroom1）未到达目标就放弃，A* 成功率从 95.6% 降到 76.7%。
//   8000 允许探索全栅格，实测单次 A* 平均 0.6ms（远低于 100ms 超时），
//   即使满 8000 节点也仅 ~5ms。
constexpr int MAX_NODES = 8000;

// A* 超时阈值（100ms）
constexpr double ASTAR_TIMEOUT_SEC = 0.1;

class AStarPlanner {
public:
    explicit AStarPlanner(const Costmap& costmap);

    // 主入口：规划路径
    // 返回路径点列表 (wx, wy)，不含起点，含终点；失败返回空 vector
    std::vector<std::pair<double, double>> plan(
        double start_x, double start_y,
        double goal_x, double goal_y);

private:
    const Costmap& costmap_;

    // 8 连通邻居：(dx, dy, move_cost)
    struct Neighbor { int dx, dy; double cost; };
    std::vector<Neighbor> neighbors_;

    double heuristic(int gx, int gy, int tx, int ty) const;
    double cell_cost(int gx, int gy) const;
    bool is_traversable(int gx, int gy) const;
    // v3.2.2 修复: max_radius 10→50 — 原 10 cells (1m) 在密集障碍区
    //   (餐厅/储物间)找不到自由 cell，返回 {-1,-1} 导致 A* 失败。
    //   50 cells (5m) 覆盖半个栅格，足以从任何墙角找到最近自由 cell。
    std::pair<int, int> find_nearest_free(int gx, int gy, int max_radius = 50) const;

    // 路径平滑
    std::vector<std::pair<double, double>> smooth_path(
        const std::vector<std::pair<double, double>>& path) const;
    bool has_line_of_sight(
        const std::pair<double, double>& p1,
        const std::pair<double, double>& p2) const;

    // 网格坐标编码为唯一 key：gx * 10000 + gy
    static long long cell_key(int gx, int gy) {
        return static_cast<long long>(gx) * 10000 + gy;
    }
};

}  // namespace puppy_nav_core
