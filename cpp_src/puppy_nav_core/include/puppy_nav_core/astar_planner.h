#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <utility>
#include "puppy_nav_core/costmap.h"

namespace puppy_nav_core {

// 规划阈值：A* 将 cost >= 120 的 cell 视为阻塞。
// v3.2.10: 110→120 — 原阈值 110 在门道处阻塞过多 cell。
//   膨胀核在 dist=0.05m 处 cost=113, dist=0.04m 处 cost=117。
//   阈值 110 让这些 cell 不可通行，门道有效宽度减少 0.04m/侧。
//   120 允许 dist>=0.03m 的 cell 可通行，门道多 0.04m 宽度。
//   cell_cost 二次惩罚仍让墙边代价高(cost 117 → 1+(117/120)^2*12=12.4)，
//   A* 会尽量避开但不会硬阻断，提升门道穿越成功率。
constexpr int PLAN_BLOCKED = 120;

// A* 放弃前最大扩展节点数（防止大网格上的失控搜索）
// v3.2.2: 2000→8000（对应当时 100x80=8000 cells 栅格，允许探索全栅格）。
// 2026-08-20 修复(P0 规划失败根因): 栅格已扩充为 160x120=19200 cells，
//   但 MAX_NODES 未同步更新，导致搜索最多只能探索 ~42% 自由空间，
//   长距离规划(dock→bedroom1 等)在抵达目标前耗尽节点预算而放弃 ——
//   表现为 planning_failures 占比 ~67%、astar_rate ~32%（fail_no_path=0 说明路径本存在）。
//   修正: 提升至 30000（约 1.5× 栅格，含三次重试余量）；
//   历史数据 8000 节点≈5ms，线性外推 30000≈19ms，远低于 100ms 超时，安全。
constexpr int MAX_NODES = 30000;  // 网格 160x120=19200 cells；30000≈1.5x 含三次重试余量

// A* 超时阈值（100ms）
constexpr double ASTAR_TIMEOUT_SEC = 0.1;  // 计划 §13: max_planning_time_ms=100

class AStarPlanner {
public:
    explicit AStarPlanner(const Costmap& costmap);

    // 主入口：规划路径
    // 返回路径点列表 (wx, wy)，不含起点，含终点；失败返回空 vector
    std::vector<std::pair<double, double>> plan(
        double start_x, double start_y,
        double goal_x, double goal_y);

    // v3.2.10: 放宽阈值重试 — 标准 plan() 失败时调用
    //   用 COST_LETHAL(254) 作为阻塞阈值，允许穿越高膨胀区
    //   仅在标准阈值找不到路径时使用，避免常态走墙边
    std::vector<std::pair<double, double>> plan_relaxed(
        double start_x, double start_y,
        double goal_x, double goal_y);

    // P1-2.2: 纯静态层 fallback — 前两级(plan/plan_relaxed)都失败时的终极重试
    //   忽略动态障碍层(obstacle_cost)，仅用静态层(static_cost)规划 + 放宽阈值
    //   场景：动态障碍（行人）100%阻塞物理通路，但静态层（墙/门）是通的
    //   路径给出后仍由 DWA/跟踪控制做避障，只提升规划成功率
    std::vector<std::pair<double, double>> plan_static_only_fallback(
        double start_x, double start_y,
        double goal_x, double goal_y);

    // v3.2.10: 放宽模式重试统计
    int relaxed_calls   = 0;
    int relaxed_success = 0;

    // P1-2.2: 纯静态层 fallback 统计（动态障碍100%阻塞物理通路时的终极重试）
    int static_only_calls   = 0;
    int static_only_success = 0;

    // P1-2.2: 起点/终点嵌墙自愈统计（A*成功率 seed6 97.2% -> >=99%）
    //   start_corrected_count:       起点发生修正的次数（吸附到最近自由 cell）
    //   goal_corrected_count:        终点发生修正的次数
    //   start_corrected_total_dist:  起点修正累计距离（米，用于统计漂移程度）
    //   start_corrected_max_dist:    单次起点修正最大距离
    //   start_corrected_warn_count:  起点修正距离 >0.5m 的 WARN 次数（与 P1-2.1 尖峰交叉验证）
    int start_corrected_count = 0;
    int goal_corrected_count  = 0;
    double start_corrected_total_dist = 0.0;
    double start_corrected_max_dist   = 0.0;
    int start_corrected_warn_count    = 0;
    // 最近一次修正距离（米，<0 = 未修正），用于 nav_bridge.h 即时打印 WARN
    double last_start_corrected_dist = -1.0;
    double last_goal_corrected_dist  = -1.0;

private:
    // v3.2.10: 内部规划实现（由 plan/plan_relaxed 调用，使用 current_threshold_）
    std::vector<std::pair<double, double>> plan_impl(
        double start_x, double start_y,
        double goal_x, double goal_y);

    // P1-2.2: 纯静态层模式 — true 时 is_traversable/find_nearest_free 只看 static_cost
    //   用于 plan_static_only_fallback()
    bool static_only_mode_ = false;

private:
    const Costmap& costmap_;

    // 8 连通邻居：(dx, dy, move_cost)
    struct Neighbor { int dx, dy; double cost; };
    std::vector<Neighbor> neighbors_;

    // v3.2.10: 当前规划阈值（plan=PLAN_BLOCKED, plan_relaxed=COST_LETHAL-1）
    int current_threshold_ = PLAN_BLOCKED;

    // R26a: 起点宽容区中心（世界坐标）。plan_impl() 每次规划前设为当前起点；
    //   is_footprint_traversable() 在距该点 START_RELIEF_RADIUS 内改用
    //   PHYSICAL_RADIUS 判定，使机器人能从"合法但被 A* 判致命"的贴墙位姿走出。
    double relief_cx_ = 0.0;
    double relief_cy_ = 0.0;
    bool   relief_active_ = false;

    double heuristic(int gx, int gy, int tx, int ty) const;
    double cell_cost(int gx, int gy) const;
    bool is_traversable(int gx, int gy) const;
    bool is_footprint_traversable(int gx, int gy, int threshold) const;

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

public:
    // v3.2.5: A* 失败原因统计（公开用于诊断）
    //   fail_no_nearest_start: 起点找不到自由 cell（机器人嵌入障碍）
    //   fail_no_nearest_goal:  终点找不到自由 cell（目标在墙内）
    //   fail_timeout:          100ms 超时
    //   fail_max_nodes:        超过 MAX_NODES 上限
    //   fail_no_path:          开放列表耗尽仍无路径（真无路）
    int fail_no_nearest_start = 0;
    int fail_no_nearest_goal = 0;
    int fail_timeout = 0;
    int fail_max_nodes = 0;
    int fail_no_path = 0;
    // 最后一次失败的起终点（用于定位场景）
    double last_fail_sx = 0, last_fail_sy = 0;
    double last_fail_gx = 0, last_fail_gy = 0;
    int last_fail_reason = 0;  // 1=start, 2=goal, 3=timeout, 4=max_nodes, 5=no_path

    // P1-2.2: 暴露 find_nearest_free 便于单元测试与扩展
    //   max_radius: 搜索半径（单位 cell），默认 50 (5m)
    //   threshold_overload: -1 使用 current_threshold_，否则自定义阻塞阈值（如 253 放宽）
    std::pair<int, int> find_nearest_free(int gx, int gy, int max_radius = 50,
                                          int threshold_overload = -1) const;
};

}  // namespace puppy_nav_core
