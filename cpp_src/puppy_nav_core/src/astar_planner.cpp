#include "puppy_nav_core/astar_planner.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>
#include <cstdint>

namespace puppy_nav_core {

namespace {
// 开放节点：用于优先队列（替换 Python 的 heapq 元组 (f, counter, (x, y))）
struct OpenNode {
    double f;        // f = g + h
    int counter;     // 绑定计数器（同 f 值时按插入顺序出队）
    int gx, gy;      // 网格坐标
};

// 最小堆比较器：f 小者优先，相同 f 按 counter 小者优先
// （std::priority_queue 中返回 true 表示 a 优先级低于 b）
struct OpenNodeGreater {
    bool operator()(const OpenNode& a, const OpenNode& b) const {
        if (a.f != b.f) return a.f > b.f;
        return a.counter > b.counter;
    }
};
}  // namespace

AStarPlanner::AStarPlanner(const Costmap& costmap) : costmap_(costmap) {
    // 8 连通邻居：(dx, dy, move_cost)
    neighbors_ = {
        {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
        {1, 1, 1.414}, {1, -1, 1.414}, {-1, 1, 1.414}, {-1, -1, 1.414},
    };
}

double AStarPlanner::heuristic(int gx, int gy, int tx, int ty) const {
    // Octile 距离（8 连通网格的 admissible 启发）
    int dx = std::abs(gx - tx);
    int dy = std::abs(gy - ty);
    return (dx + dy) + (1.414 - 2) * std::min(dx, dy);
}

double AStarPlanner::cell_cost(int gx, int gy) const {
    // 获取 cell 的通行代价（越高越昂贵）。
    //
    // 使用二次惩罚，使高代价 cell（障碍物附近的狭窄缺口）
    // 被强烈抑制。这使 A* 偏好绕开狭窄通道的较长路径，
    // 而非 DWA 无法跟随的捷径。
    //
    // v2.3 调优：增大惩罚系数 (8.0->12.0->30.0) 使路径更远离墙壁/家具，
    //   避免跟随器在定位误差下刮碰 coffee_table 等障碍的拐角(§16 验收残留缺陷)
    // 减少机器人卡在膨胀区导致的 RECOVER 触发。
    //
    // v3.1 优化：UNKNOWN cell 软惩罚（不再硬阻塞）
    //   原行为: is_traversable() 对 UNKNOWN 返回 false，导致 A* 成功率 46.1%
    //   1 小时测试中大量路径因 UNKNOWN cell 阻塞而失败。
    //   新行为: UNKNOWN cell 视为可通行但代价 5.0（比自由 cell 高 5x），
    //   A* 会优先选已观测自由 cell，仅在无路时才穿越 UNKNOWN 区域。
    //   has_line_of_sight() 仍拒绝 UNKNOWN（避免平滑阶段抄近路）。
    uint8_t c = costmap_.get_cost_grid(gx, gy);
    if (c >= current_threshold_) {
        return std::numeric_limits<double>::infinity();  // 阻塞
    }
    // UNKNOWN cell 软惩罚：基础代价 5.0（仍可通行但优先级低）
    if (costmap_.is_unknown_grid(gx, gy)) {
        return 5.0;
    }
    // 二次惩罚：free=1, cost 50->2.7, cost 100->11.0, cost 109->13.0
    double cf = static_cast<double>(c);
    double pb = static_cast<double>(PLAN_BLOCKED);
    return 1.0 + (cf / pb) * (cf / pb) * 30.0;
}

bool AStarPlanner::is_traversable(int gx, int gy) const {
    // cell 是否可进入（代价低于阈值）。
    // v3.2.10: 使用 current_threshold_ 支持放宽模式
    // P1-2.2: static_only_mode_ 为 true 时忽略动态障碍层，只看静态层 static_cost
    return is_footprint_traversable(gx, gy, current_threshold_);
}

bool AStarPlanner::is_footprint_traversable(int gx, int gy, int threshold) const {
    if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) {
        return false;
    }

    // R26a: 起点邻域宽容 —— 在距规划起点 START_RELIEF_RADIUS 内改用 PHYSICAL_RADIUS。
    //   机器人物理半径 0.20m，但默认 footprint 判定用 0.35m，中间 15cm 是
    //   "物理合法却被 A* 判致命"的带。贴墙位姿落入该带会使起点嵌墙，
    //   旧逻辑强拽起点 0.7m 导致路径与真身脱节 → 零速死锁。
    //   这里只放宽起点附近，让机器人能走出该带；离开邻域立刻恢复严格判定。
    double eff_radius = ROBOT_RADIUS;
    if (relief_active_) {
        double wx, wy;
        costmap_.grid_to_world(gx, gy, wx, wy);
        double ddx = wx - relief_cx_;
        double ddy = wy - relief_cy_;
        if (ddx * ddx + ddy * ddy <= START_RELIEF_RADIUS * START_RELIEF_RADIUS) {
            eff_radius = PHYSICAL_RADIUS;
        }
    }

    const int radius_cells = static_cast<int>(std::ceil(eff_radius / GRID_RESOLUTION));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (std::sqrt(static_cast<double>(dx * dx + dy * dy)) * GRID_RESOLUTION > eff_radius) {
                continue;
            }
            int nx = gx + dx;
            int ny = gy + dy;
            if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) {
                return false;
            }
            if (nx == gx && ny == gy) {
                uint8_t center_cost = static_only_mode_
                    ? costmap_.static_cost[static_cast<size_t>(ny) * GRID_W + nx]
                    : costmap_.get_cost_grid(nx, ny);
                if (center_cost >= threshold) {
                    return false;
                }
            }
            if (costmap_.static_cost[static_cast<size_t>(ny) * GRID_W + nx] >= COST_LETHAL) {
                return false;
            }
        }
    }
    return true;
}

std::pair<int, int> AStarPlanner::find_nearest_free(int gx, int gy, int max_radius,
                                                     int threshold_overload) const {
    // 使用方形环壳 BFS 寻找离 (gx, gy) 最近的可通行 cell。
    // P1-2.2:
    //   1) 修复 r < max_radius -> r <= max_radius（原逻辑漏搜了最外一圈）
    //   2) 支持 threshold_overload，允许自定义阻塞阈值（放宽阈值重试时用）
    //   3) static_only_mode_ 时忽略 dynamic obstacle，只看 static_cost（纯静态层 fallback）
    //   4) 先检查中心 cell，再按 r=1..max_radius 逐圈扩展
    auto traversable = [&](int x, int y) -> bool {
        if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return false;
        uint8_t c;
        if (static_only_mode_) {
            c = costmap_.static_cost[static_cast<size_t>(y) * GRID_W + x];
        } else {
            c = costmap_.get_cost_grid(x, y);
        }
        int thresh = (threshold_overload >= 0) ? threshold_overload : current_threshold_;
        return is_footprint_traversable(x, y, thresh);
    };

    if (traversable(gx, gy)) {
        return {gx, gy};
    }
    // P1-2.2: 原 r < max_radius 漏搜最外一圈，现改为 <=
    for (int r = 1; r <= max_radius; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::abs(dx) != r && std::abs(dy) != r) {
                    continue;  // 只检查当前半径 r 的方形环壳边缘
                }
                int nx = gx + dx;
                int ny = gy + dy;
                if (traversable(nx, ny)) {
                    return {nx, ny};
                }
            }
        }
    }
    return {-1, -1};  // 范围内未找到
}

std::vector<std::pair<double, double>> AStarPlanner::plan(
    double start_x, double start_y,
    double goal_x, double goal_y) {
    // v3.2.10: 标准 plan — 用 PLAN_BLOCKED 阈值
    current_threshold_ = PLAN_BLOCKED;
    return plan_impl(start_x, start_y, goal_x, goal_y);
}

std::vector<std::pair<double, double>> AStarPlanner::plan_relaxed(
    double start_x, double start_y,
    double goal_x, double goal_y) {
    // v3.2.10: 放宽模式 — 用 COST_LETHAL-1 阈值，允许穿越高膨胀区
    //   仅在标准 plan() 失败时调用
    current_threshold_ = COST_LETHAL - 1;  // 253 — 仅 LETHAL(254) 阻塞
    relaxed_calls++;
    auto path = plan_impl(start_x, start_y, goal_x, goal_y);
    if (!path.empty()) relaxed_success++;
    return path;
}

std::vector<std::pair<double, double>> AStarPlanner::plan_static_only_fallback(
    double start_x, double start_y,
    double goal_x, double goal_y) {
    // P1-2.2: 纯静态层 fallback 终极重试
    //   解决场景：动态障碍（行人）完全阻塞物理通路，导致前两级 plan/plan_relaxed 真无路径
    //   方法：启用 static_only_mode_（只看静态层 static_cost，忽略 obstacle_cost）
    //         同时用放宽阈值 (COST_LETHAL-1=253)
    //   安全性：路径给出后仍由下游 DWA/跟踪控制做动态避障，仅用于提升 A* 成功率
    static_only_mode_ = true;
    current_threshold_ = COST_LETHAL - 1;
    static_only_calls++;
    auto path = plan_impl(start_x, start_y, goal_x, goal_y);
    static_only_mode_ = false;  // 务必还原（成员变量重复使用）
    if (!path.empty()) static_only_success++;
    return path;
}

std::vector<std::pair<double, double>> AStarPlanner::plan_impl(
    double start_x, double start_y,
    double goal_x, double goal_y) {
    // 从 start 规划到 goal 的路径（世界坐标）。
    // 返回 (wx, wy) 路径点列表（不含起点，含终点），
    // 失败返回空 vector（对应 Python 的 None）。

    auto start_time = std::chrono::steady_clock::now();
    int iterations = 0;

    // P1-2.2: 重置"上次修正距离"标志（<0 = 未修正），一旦发生修正即写入距离
    last_start_corrected_dist = -1.0;
    last_goal_corrected_dist  = -1.0;

    // R26a: 激活起点邻域 footprint 宽容（详见 costmap.h 的 START_RELIEF_RADIUS 注释）。
    //   必须在任何 is_footprint_traversable/find_nearest_free 调用之前设置，
    //   这样起点嵌墙自愈逻辑本身也能享受宽容，从而基本不再触发强拽。
    relief_cx_ = start_x;
    relief_cy_ = start_y;
    relief_active_ = true;
    // RAII：plan_impl 有多个 return 分支，用守卫确保离开时一定关闭宽容，
    //   避免宽容态泄漏到 DWA / 后续碰撞检查等共用 planner 的调用。
    struct ReliefGuard {
        bool* flag;
        ~ReliefGuard() { *flag = false; }
    } relief_guard{&relief_active_};

    int sgx, sgy;
    int ggx, ggy;
    costmap_.world_to_grid(start_x, start_y, sgx, sgy);
    costmap_.world_to_grid(goal_x, goal_y, ggx, ggy);

    // ------------------------------------------------------------------
    // P1-2.2: 起点/终点嵌墙自愈
    //   Step 1: 标准阈值 (PLAN_BLOCKED=120) 搜索最近自由 cell
    //   Step 2: 失败则放宽阈值 (COST_LETHAL-1=253，仅致命壁阻塞) 重试
    //   统计: 修正次数、修正距离、>0.5m 警告数
    // ------------------------------------------------------------------
    auto resolve_start_goal = [&](
        int gx, int gy, bool is_start,
        int& out_gx, int& out_gy) -> bool {

        // Step 1: 标准阈值
        std::pair<int, int> free_cell = find_nearest_free(gx, gy);
        if (free_cell.first == -1) {
            // Step 2: 放宽阈值（仅 LETHAL(254) 视为阻塞）
            free_cell = find_nearest_free(gx, gy, 50, COST_LETHAL - 1);
        }
        if (free_cell.first == -1) {
            if (is_start) { fail_no_nearest_start++; last_fail_reason = 1; }
            else          { fail_no_nearest_goal++;  last_fail_reason = 2; }
            last_fail_sx = start_x; last_fail_sy = start_y;
            last_fail_gx = goal_x;  last_fail_gy = goal_y;
            return false;
        }
        out_gx = free_cell.first;
        out_gy = free_cell.second;

        // 计算修正距离（世界坐标）
        if (out_gx != gx || out_gy != gy) {
            double orig_wx, orig_wy, new_wx, new_wy;
            costmap_.grid_to_world(gx,    gy,    orig_wx, orig_wy);
            costmap_.grid_to_world(out_gx, out_gy, new_wx,  new_wy);
            double dx = new_wx - orig_wx;
            double dy = new_wy - orig_wy;
            double dist_m = std::sqrt(dx*dx + dy*dy);
            if (is_start) {
                start_corrected_count++;
                start_corrected_total_dist += dist_m;
                if (dist_m > start_corrected_max_dist) {
                    start_corrected_max_dist = dist_m;
                }
                if (dist_m > 0.5) {
                    start_corrected_warn_count++;
                }
                last_start_corrected_dist = dist_m;
            } else {
                goal_corrected_count++;
                last_goal_corrected_dist = dist_m;
            }
        } else {
            if (is_start) last_start_corrected_dist = 0.0;
            else          last_goal_corrected_dist  = 0.0;
        }
        return true;
    };

    if (!resolve_start_goal(sgx, sgy, /*is_start=*/true,  sgx, sgy)) return {};
    if (!resolve_start_goal(ggx, ggy, /*is_start=*/false, ggx, ggy)) return {};

    if (sgx == ggx && sgy == ggy) {
        return {{goal_x, goal_y}};
    }

    // A* 搜索
    std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeGreater> open_heap;
    int counter = 0;  // 堆的绑定计数器
    std::unordered_map<long long, double> g_score;
    std::unordered_map<long long, std::pair<int, int>> came_from;
    std::unordered_set<long long> visited;

    g_score[cell_key(sgx, sgy)] = 0.0;
    double h_start = heuristic(sgx, sgy, ggx, ggy);
    open_heap.push(OpenNode{h_start, counter, sgx, sgy});

    while (!open_heap.empty()) {
        // 100ms 超时检查
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;
        if (elapsed.count() > ASTAR_TIMEOUT_SEC) {
            fail_timeout++;
            last_fail_sx = start_x; last_fail_sy = start_y;
            last_fail_gx = goal_x;  last_fail_gy = goal_y;
            last_fail_reason = 3;
            return {};
        }
        ++iterations;
        if (iterations > MAX_NODES) {
            fail_max_nodes++;
            last_fail_sx = start_x; last_fail_sy = start_y;
            last_fail_gx = goal_x;  last_fail_gy = goal_y;
            last_fail_reason = 4;
            return {};
        }

        OpenNode top = open_heap.top();
        open_heap.pop();
        int cx = top.gx;
        int cy = top.gy;

        long long ck = cell_key(cx, cy);
        if (visited.count(ck)) {
            continue;  // 已扩展过，跳过（惰性删除）
        }
        visited.insert(ck);

        if (cx == ggx && cy == ggy) {
            // 重建路径
            std::vector<std::pair<double, double>> path;
            std::pair<int, int> cur = {cx, cy};
            while (true) {
                double wx, wy;
                costmap_.grid_to_world(cur.first, cur.second, wx, wy);
                path.push_back({wx, wy});
                auto it = came_from.find(cell_key(cur.first, cur.second));
                if (it == came_from.end()) {
                    break;  // 到达起点（came_from 中无此节点）
                }
                cur = it->second;
            }
            std::reverse(path.begin(), path.end());
            // 平滑路径
            path = smooth_path(path);
            // 排除起点（对应 Python 的 path[1:]）
            if (path.empty()) {
                return {};
            }
            // v3.2.18 修复路径塌缩：直线短路径 [start, goal]（2 点）经 path.begin()+1
            // 会塌缩为单点 [goal]，被下游 simulation.h 的 size()>=2 门控误判为失败 →
            // 机器人停滞并每帧重规划（§25.4 问题1 的诱因之一）。
            // 当排除起点后不足 2 点时，保留完整 [start, goal]，避免塌缩。
            if (path.size() == 2) {
                return path;  // 已是 [start, goal]，无需排除起点
            }
            return std::vector<std::pair<double, double>>(path.begin() + 1, path.end());
        }

        double base_cost = g_score[ck];
        for (const Neighbor& nb : neighbors_) {
            int nx = cx + nb.dx;
            int ny = cy + nb.dy;
            if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) {
                continue;
            }
            long long nk = cell_key(nx, ny);
            if (visited.count(nk)) {
                continue;
            }
            if (!is_traversable(nx, ny)) {
                continue;
            }
            double cc = cell_cost(nx, ny);
            if (cc == std::numeric_limits<double>::infinity()) {
                continue;
            }
            // 对角线移动：检查两个正交 cell 是否都未阻塞（不切角）
            if (nb.dx != 0 && nb.dy != 0) {
                if (!is_traversable(cx + nb.dx, cy) ||
                    !is_traversable(cx, cy + nb.dy)) {
                    continue;
                }
            }
            double new_g = base_cost + nb.cost * cc;
            auto it = g_score.find(nk);
            if (it == g_score.end() || new_g < it->second) {
                g_score[nk] = new_g;
                came_from[nk] = {cx, cy};
                double h = heuristic(nx, ny, ggx, ggy);
                ++counter;
                open_heap.push(OpenNode{new_g + h, counter, nx, ny});
            }
        }
    }

    // 开放列表耗尽仍无路径
    fail_no_path++;
    last_fail_sx = start_x; last_fail_sy = start_y;
    last_fail_gx = goal_x;  last_fail_gy = goal_y;
    last_fail_reason = 5;
    return {};
}

std::vector<std::pair<double, double>> AStarPlanner::smooth_path(
    const std::vector<std::pair<double, double>>& path) const {
    // 通过删除冗余路径点平滑路径（视线测试）。
    //
    // 使用 Bresenham 检查两个路径点之间是否有清晰视线
    // （所有 cell 可通行）。若有，则移除中间路径点。
    if (path.size() <= 2) {
        return path;
    }

    std::vector<std::pair<double, double>> smoothed;
    smoothed.push_back(path[0]);
    size_t i = 0;
    while (i < path.size() - 1) {
        // 寻找从 i 出发能直达的最远路径点
        size_t j = path.size() - 1;
        while (j > i + 1) {
            if (has_line_of_sight(path[i], path[j])) {
                break;
            }
            --j;
        }
        smoothed.push_back(path[j]);
        i = j;
    }

    return smoothed;
}

bool AStarPlanner::has_line_of_sight(
    const std::pair<double, double>& p1,
    const std::pair<double, double>& p2) const {
    // 检查 p1 到 p2 的连线是否清晰（所有 cell 可通行）。
    //
    // 不通过未知 cell 取捷径 — 防止路径平滑穿过未测绘的墙。
    int x0, y0, x1, y1;
    costmap_.world_to_grid(p1.first, p1.second, x0, y0);
    costmap_.world_to_grid(p2.first, p2.second, x1, y1);
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    while (true) {
        if (!is_traversable(x, y)) {
            return false;
        }
        // 不通过未知 cell 取捷径（未测绘区域）
        if (costmap_.is_unknown_grid(x, y)) {
            return false;
        }
        if (x == x1 && y == y1) {
            return true;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

}  // namespace puppy_nav_core
