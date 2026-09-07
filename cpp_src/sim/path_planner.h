// path_planner.h — A*路径规划 + RDP简化 + 平滑（C++版）
// 从Python visual_path_planner.py 转换
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <functional>
#include <cstdint>

namespace puppy_sim {

struct BBox {
    double xmin, ymin, xmax, ymax;
    BBox(double x1, double y1, double x2, double y2)
        : xmin(x1), ymin(y1), xmax(x2), ymax(y2) {}
};

// 栅格参数
const double GRID_RES = 0.10;
const double GRID_MARGIN = 0.35;
const double WORLD_X_MIN = -5.5, WORLD_X_MAX = 5.5;
const double WORLD_Y_MIN = -4.5, WORLD_Y_MAX = 4.5;
const int G_W = (int)((WORLD_X_MAX - WORLD_X_MIN) / GRID_RES);
const int G_H = (int)((WORLD_Y_MAX - WORLD_Y_MIN) / GRID_RES);

inline void world_to_grid(double x, double y, int& gx, int& gy) {
    gx = (int)((x - WORLD_X_MIN) / GRID_RES);
    gy = (int)((y - WORLD_Y_MIN) / GRID_RES);
}

inline void grid_to_world(int gx, int gy, double& x, double& y) {
    x = WORLD_X_MIN + (gx + 0.5) * GRID_RES;
    y = WORLD_Y_MIN + (gy + 0.5) * GRID_RES;
}

// 构建静态障碍物栅格
inline std::vector<std::vector<uint8_t>> build_grid(const std::vector<BBox>& obstacles) {
    std::vector<std::vector<uint8_t>> grid(G_H, std::vector<uint8_t>(G_W, 0));
    for (auto& obs : obstacles) {
        int gx0, gy0, gx1, gy1;
        world_to_grid(obs.xmin - GRID_MARGIN, obs.ymin - GRID_MARGIN, gx0, gy0);
        world_to_grid(obs.xmax + GRID_MARGIN, obs.ymax + GRID_MARGIN, gx1, gy1);
        gx0 = std::max(0, std::min(G_W-1, gx0));
        gx1 = std::max(0, std::min(G_W-1, gx1));
        gy0 = std::max(0, std::min(G_H-1, gy0));
        gy1 = std::max(0, std::min(G_H-1, gy1));
        for (int y = gy0; y <= gy1; y++)
            for (int x = gx0; x <= gx1; x++)
                grid[y][x] = 255;
    }
    return grid;
}

inline bool is_safe_cell(int gx, int gy, const std::vector<std::vector<uint8_t>>& grid) {
    if (gx < 0 || gx >= G_W || gy < 0 || gy >= G_H) return false;
    return grid[gy][gx] == 0;
}

// A*路径规划
inline std::vector<std::pair<double,double>> astar_plan(
        double sx, double sy, double tx, double ty,
        const std::vector<BBox>& obstacles, int max_nodes = 5000) {

    auto grid = build_grid(obstacles);
    int sgx, sgy, tgx, tgy;
    world_to_grid(sx, sy, sgx, sgy);
    world_to_grid(tx, ty, tgx, tgy);

    // 目标不可达时找最近可通行格
    if (!is_safe_cell(tgx, tgy, grid)) {
        int best_d = 999999, bx = tgx, by = tgy;
        for (int dx = -5; dx <= 5; dx++)
            for (int dy = -5; dy <= 5; dy++) {
                int nx = tgx+dx, ny = tgy+dy;
                if (is_safe_cell(nx, ny, grid) && dx*dx+dy*dy < best_d) {
                    best_d = dx*dx+dy*dy; bx = nx; by = ny;
                }
            }
        tgx = bx; tgy = by;
    }
    if (!is_safe_cell(sgx, sgy, grid)) {
        int best_d = 999999, bx = sgx, by = sgy;
        for (int dx = -5; dx <= 5; dx++)
            for (int dy = -5; dy <= 5; dy++) {
                int nx = sgx+dx, ny = sgy+dy;
                if (is_safe_cell(nx, ny, grid) && dx*dx+dy*dy < best_d) {
                    best_d = dx*dx+dy*dy; bx = nx; by = ny;
                }
            }
        sgx = bx; sgy = by;
    }

    // A*搜索
    using Node = std::pair<int, int>;  // (gx, gy)
    using FNode = std::pair<double, Node>;
    std::priority_queue<FNode, std::vector<FNode>, std::greater<FNode>> open;
    std::vector<Node> came_from;
    std::vector<double> g_score;

    auto idx = [sgx, sgy](int x, int y) {
        return (y - sgy + 50) * 200 + (x - sgx + 50);
    };
    // 简化：用unordered_map
    std::unordered_map<long long, Node> cf;
    std::unordered_map<long long, double> gs;

    long long start_key = (long long)sgx * 10000 + sgy;
    gs[start_key] = 0;
    open.push({0, {sgx, sgy}});

    int dx8[] = {-1,-1,-1,0,0,1,1,1};
    int dy8[] = {-1,0,1,-1,1,-1,0,1};
    double cost8[] = {1.414,1,1.414,1,1,1.414,1,1.414};

    int expanded = 0;
    while (!open.empty() && expanded < max_nodes) {
        auto [f, cur] = open.top(); open.pop();
        expanded++;
        if (cur.first == tgx && cur.second == tgy) {
            // 回溯
            std::vector<std::pair<double,double>> path;
            Node n = cur;
            while (true) {
                double wx, wy;
                grid_to_world(n.first, n.second, wx, wy);
                path.push_back({wx, wy});
                long long key = (long long)n.first * 10000 + n.second;
                if (cf.find(key) == cf.end()) break;
                n = cf[key];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (int i = 0; i < 8; i++) {
            int nx = cur.first + dx8[i];
            int ny = cur.second + dy8[i];
            if (!is_safe_cell(nx, ny, grid)) continue;
            // 对角线不切角
            if (dx8[i] != 0 && dy8[i] != 0) {
                if (!is_safe_cell(cur.first + dx8[i], cur.second, grid) ||
                    !is_safe_cell(cur.first, cur.second + dy8[i], grid)) continue;
            }
            long long key = (long long)nx * 10000 + ny;
            double new_g = gs[(long long)cur.first*10000+cur.second] + cost8[i];
            if (gs.find(key) == gs.end() || new_g < gs[key]) {
                gs[key] = new_g;
                double h = std::sqrt((double)(nx-tgx)*(nx-tgx) + (double)(ny-tgy)*(ny-tgy));
                open.push({new_g + h, {nx, ny}});
                cf[key] = cur;
            }
        }
    }
    return {};  // 失败
}

// RDP简化
inline std::vector<std::pair<double,double>> rdp_simplify(
        const std::vector<std::pair<double,double>>& path, double eps = 0.20) {
    if (path.size() < 3) return path;

    auto perp_dist = [](const std::pair<double,double>& p,
                        const std::pair<double,double>& a,
                        const std::pair<double,double>& b) -> double {
        double dx = b.first - a.first, dy = b.second - a.second;
        double len = std::sqrt(dx*dx + dy*dy);
        if (len < 1e-9) return std::sqrt((p.first-a.first)*(p.first-a.first) + (p.second-a.second)*(p.second-a.second));
        double t = ((p.first-a.first)*dx + (p.second-a.second)*dy) / (len*len);
        t = std::max(0.0, std::min(1.0, t));
        double px = a.first + t*dx, py = a.second + t*dy;
        return std::sqrt((p.first-px)*(p.first-px) + (p.second-py)*(p.second-py));
    };

    std::function<std::vector<std::pair<double,double>>(std::vector<std::pair<double,double>>, double)> rdp =
        [&](std::vector<std::pair<double,double>> pts, double e) -> std::vector<std::pair<double,double>> {
        if (pts.size() < 3) return pts;
        double max_d = 0; size_t max_i = 0;
        for (size_t i = 1; i < pts.size()-1; i++) {
            double d = perp_dist(pts[i], pts[0], pts.back());
            if (d > max_d) { max_d = d; max_i = i; }
        }
        if (max_d > e) {
            auto left = rdp(std::vector<std::pair<double,double>>(pts.begin(), pts.begin()+max_i+1), e);
            auto right = rdp(std::vector<std::pair<double,double>>(pts.begin()+max_i, pts.end()), e);
            left.pop_back();
            left.insert(left.end(), right.begin(), right.end());
            return left;
        }
        return {pts[0], pts.back()};
    };

    return rdp(path, eps);
}

// 移动平均平滑
inline std::vector<std::pair<double,double>> smooth_path(
        const std::vector<std::pair<double,double>>& path, int window = 2) {
    if (path.size() < 3) return path;
    std::vector<std::pair<double,double>> out;
    out.push_back(path[0]);
    for (size_t i = 1; i < path.size()-1; i++) {
        double sx = 0, sy = 0; int cnt = 0;
        for (int j = std::max(0, (int)i-window); j < std::min((int)path.size(), (int)i+window+1); j++) {
            sx += path[j].first; sy += path[j].second; cnt++;
        }
        out.push_back({sx/cnt, sy/cnt});
    }
    out.push_back(path.back());
    return out;
}

// 完整路径规划：A* → RDP → 平滑
inline std::vector<std::pair<double,double>> plan_and_smooth(
        double sx, double sy, double tx, double ty,
        const std::vector<BBox>& obstacles) {
    auto raw = astar_plan(sx, sy, tx, ty, obstacles);
    if (raw.size() < 2) return {};
    auto simplified = rdp_simplify(raw, 0.20);
    return smooth_path(simplified, 2);
}

// 路径跟随器
class PathFollower {
public:
    std::vector<std::pair<double,double>> path;
    size_t current_idx = 0;
    double lookahead;

    PathFollower(double la = 0.6) : lookahead(la) {}

    void set_path(const std::vector<std::pair<double,double>>& p) {
        path = p; current_idx = 0;
    }

    std::pair<double,double> get_target(double rx, double ry) {
        if (path.empty()) return {0, 0};
        // 找最近路径点
        double min_d = 1e9; size_t nearest = current_idx;
        for (size_t i = current_idx; i < std::min(path.size(), current_idx+20); i++) {
            double d = std::sqrt((rx-path[i].first)*(rx-path[i].first) + (ry-path[i].second)*(ry-path[i].second));
            if (d < min_d) { min_d = d; nearest = i; }
        }
        current_idx = nearest;
        // 前瞻
        double accum = 0;
        for (size_t i = nearest; i < path.size()-1; i++) {
            accum += std::sqrt((path[i+1].first-path[i].first)*(path[i+1].first-path[i].first) +
                               (path[i+1].second-path[i].second)*(path[i+1].second-path[i].second));
            if (accum >= lookahead) return path[i+1];
        }
        return path.back();
    }

    bool is_done(double rx, double ry, double threshold = 0.4) {
        if (path.empty()) return true;
        auto& end = path.back();
        return std::sqrt((rx-end.first)*(rx-end.first) + (ry-end.second)*(ry-end.second)) < threshold;
    }
};

} // namespace puppy_sim
