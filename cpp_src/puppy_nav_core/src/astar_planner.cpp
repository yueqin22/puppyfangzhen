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
    // v2.3 调优：增大惩罚系数 (8.0->12.0) 使路径更远离墙壁，
    // 减少机器人卡在膨胀区导致的 RECOVER 触发。
    //
    // v3.1 优化：UNKNOWN cell 软惩罚（不再硬阻塞）
    //   原行为: is_traversable() 对 UNKNOWN 返回 false，导致 A* 成功率 46.1%
    //   1 小时测试中大量路径因 UNKNOWN cell 阻塞而失败。
    //   新行为: UNKNOWN cell 视为可通行但代价 5.0（比自由 cell 高 5x），
    //   A* 会优先选已观测自由 cell，仅在无路时才穿越 UNKNOWN 区域。
    //   has_line_of_sight() 仍拒绝 UNKNOWN（避免平滑阶段抄近路）。
    uint8_t c = costmap_.get_cost_grid(gx, gy);
    if (c >= PLAN_BLOCKED) {
        return std::numeric_limits<double>::infinity();  // 阻塞
    }
    // UNKNOWN cell 软惩罚：基础代价 5.0（仍可通行但优先级低）
    if (costmap_.is_unknown_grid(gx, gy)) {
        return 5.0;
    }
    // 二次惩罚：free=1, cost 50->2.7, cost 100->11.0, cost 109->13.0
    double cf = static_cast<double>(c);
    double pb = static_cast<double>(PLAN_BLOCKED);
    return 1.0 + (cf / pb) * (cf / pb) * 12.0;
}

bool AStarPlanner::is_traversable(int gx, int gy) const {
    // cell 是否可进入（代价低于阈值）。
    //
    // v3.1 优化：UNKNOWN cell 不再硬阻塞（cell_cost 中加软惩罚 5.0）
    //   原行为: UNKNOWN cell 直接返回 false，A* 成功率 46.1%
    //   新行为: UNKNOWN cell 视为可通行，cell_cost 返回 5.0 高代价
    //   1 小时测试中 UNKNOWN cell 主要在门道边缘（init_from_obstacles
    //   0.5m 扫描间距导致的盲区），软惩罚让 A* 优先选已知自由 cell
    //   但仍允许穿越门道盲区，大幅提升成功率。
    //
    // 阻塞条件（保留）:
    //   - cost >= PLAN_BLOCKED（实际障碍/膨胀致命区）
    uint8_t c = costmap_.get_cost_grid(gx, gy);
    if (c >= PLAN_BLOCKED) {
        return false;
    }
    return true;
}

std::pair<int, int> AStarPlanner::find_nearest_free(int gx, int gy, int max_radius) const {
    // 使用 BFS 寻找离 (gx, gy) 最近的可通行 cell。
    if (is_traversable(gx, gy)) {
        return {gx, gy};
    }
    for (int r = 1; r < max_radius; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::abs(dx) != r && std::abs(dy) != r) {
                    continue;  // 只检查当前半径 r 的环形边缘
                }
                int nx = gx + dx;
                int ny = gy + dy;
                if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                    if (is_traversable(nx, ny)) {
                        return {nx, ny};
                    }
                }
            }
        }
    }
    return {-1, -1};  // 未找到（对应 Python 的 None, None）
}

std::vector<std::pair<double, double>> AStarPlanner::plan(
    double start_x, double start_y,
    double goal_x, double goal_y) {
    // 从 start 规划到 goal 的路径（世界坐标）。
    // 返回 (wx, wy) 路径点列表（不含起点，含终点），
    // 失败返回空 vector（对应 Python 的 None）。

    auto start_time = std::chrono::steady_clock::now();
    int iterations = 0;

    int sgx, sgy;
    int ggx, ggy;
    costmap_.world_to_grid(start_x, start_y, sgx, sgy);
    costmap_.world_to_grid(goal_x, goal_y, ggx, ggy);

    // 起点/终点若阻塞则吸附到最近自由 cell
    std::pair<int, int> s = find_nearest_free(sgx, sgy);
    if (s.first == -1) {
        return {};
    }
    sgx = s.first;
    sgy = s.second;

    std::pair<int, int> g = find_nearest_free(ggx, ggy);
    if (g.first == -1) {
        return {};
    }
    ggx = g.first;
    ggy = g.second;

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
            return {};
        }
        ++iterations;
        if (iterations > MAX_NODES) {
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

    return {};  // 未找到路径
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
