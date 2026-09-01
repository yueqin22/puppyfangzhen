// PathValidator 实现 — 见 path_validator.h 说明
#include "puppy_nav_core/path_validator.h"

namespace puppy_nav_core {

// ---------------------------------------------------------------------------
// 几何辅助
// ---------------------------------------------------------------------------
Vec2 PathValidator::world_to_local(double wx, double wy,
                                    double cx, double cy, double theta) {
    double dx = wx - cx;
    double dy = wy - cy;
    double c = std::cos(-theta);
    double s = std::sin(-theta);
    return Vec2{ dx * c - dy * s, dx * s + dy * c };
}

bool PathValidator::point_in_polygon(double lx, double ly,
                                      const std::vector<Vec2>& poly) {
    // 射线法: 从点向右发射, 统计与边相交次数
    bool inside = false;
    size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly[i].x, yi = poly[i].y;
        double xj = poly[j].x, yj = poly[j].y;
        bool intersect = ((yi > ly) != (yj > ly)) &&
            (lx < (xj - xi) * (ly - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

double PathValidator::footprint_max_extent(const std::vector<Vec2>& footprint) {
    double m = 0.0;
    for (const auto& p : footprint) {
        double d = std::hypot(p.x, p.y);
        if (d > m) m = d;
    }
    return m;
}

double PathValidator::admissible_half_width(const std::vector<Vec2>& footprint,
                                             double min_path_clearance) {
    return footprint_max_extent(footprint) + min_path_clearance;
}

void PathValidator::footprint_cells(double cx, double cy, double theta,
                                     const std::vector<Vec2>& footprint,
                                     std::vector<std::pair<int,int>>& out_cells) const {
    out_cells.clear();
    if (footprint.empty()) return;

    // 1) 计算旋转后 footprint 的包围盒 (世界坐标)
    double minx =  std::numeric_limits<double>::infinity();
    double maxx = -std::numeric_limits<double>::infinity();
    double miny =  std::numeric_limits<double>::infinity();
    double maxy = -std::numeric_limits<double>::infinity();
    for (const auto& p : footprint) {
        double wx = cx + (p.x * std::cos(theta) - p.y * std::sin(theta));
        double wy = cy + (p.x * std::sin(theta) + p.y * std::cos(theta));
        if (wx < minx) minx = wx; if (wx > maxx) maxx = wx;
        if (wy < miny) miny = wy; if (wy > maxy) maxy = wy;
    }

    // 2) 包围盒内逐 cell 中心变换到局部, 点在多边形内则收集
    int gx0, gy0, gx1, gy1;
    grid_.world_to_grid(minx, miny, gx0, gy0);
    grid_.world_to_grid(maxx, maxy, gx1, gy1);
    // 扩展 1 格以防浮点边界漏 cell
    gx0--; gy0--; gx1++; gy1++;
    // 半格膨胀: 把 cell 中心或其四邻半格点落入多边形的 cell 都视为覆盖,
    //   避免 footprint 边界格被 point_in_polygon 判为外部而漏检 (M2 修复:
    //   0.6m 门边界墙格原被漏判, 导致不安全路径通过)。膨胀使验证更保守(安全)。
    double h = 0.5 * grid_.resolution;
    for (int gy = gy0; gy <= gy1; ++gy) {
        for (int gx = gx0; gx <= gx1; ++gx) {
            double wcx, wcy;
            grid_.grid_to_world(gx, gy, wcx, wcy);
            bool inside =
                point_in_polygon(wcx, wcy, footprint) ||
                point_in_polygon(wcx - h, wcy, footprint) ||
                point_in_polygon(wcx + h, wcy, footprint) ||
                point_in_polygon(wcx, wcy - h, footprint) ||
                point_in_polygon(wcx, wcy + h, footprint);
            if (inside) {
                out_cells.emplace_back(gx, gy);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 净空计算
// ---------------------------------------------------------------------------
double PathValidator::cell_clearance_cells(int gx, int gy, int cap_cells) const {
    const int W = grid_.width, H = grid_.height;
    if (gx < 0 || gy < 0 || gx >= W || gy >= H) return 0.0;  // 越界视为 0 净空
    if (grid_.is_occupied(gx, gy)) return 0.0;                // 自身占用
    // 由近及远环形搜索最近 occupied cell (精确欧氏)
    for (int r = 1; r <= cap_cells; ++r) {
        bool found = false;
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::abs(dx) != r && std::abs(dy) != r) continue;  // 仅环
                int nx = gx + dx, ny = gy + dy;
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                if (grid_.is_occupied(nx, ny)) { found = true; break; }
            }
            if (found) break;
        }
        if (found) return static_cast<double>(r);
    }
    return static_cast<double>(cap_cells);  // 超出上限: 视为"足够远"
}

double PathValidator::clearance_m(double x, double y) const {
    int gx, gy;
    grid_.world_to_grid(x, y, gx, gy);
    int ring = cell_clearance_cells(gx, gy);
    // 边界修正: ring 是到占用 cell 中心的 Chebyshev 格数,
    //   真实到障碍边界距离 ≈ (ring - 0.5) * resolution, 否则高估半格 (0.05m),
    //   导致 min_path_clearance(0.08m) 这种亚半格阈值被穿透。
    return (ring == 0) ? 0.0 : (ring - 0.5) * grid_.resolution;
}

bool PathValidator::footprint_collision_and_clearance(
        double x, double y, double theta,
        const std::vector<Vec2>& footprint,
        bool unknown_is_blocked,
        double& out_clearance_m) const {
    std::vector<std::pair<int,int>> cells;
    footprint_cells(x, y, theta, footprint, cells);

    const int W = grid_.width, H = grid_.height;
    double min_clr = std::numeric_limits<double>::infinity();

    for (const auto& c : cells) {
        int gx = c.first, gy = c.second;
        if (gx < 0 || gy < 0 || gx >= W || gy >= H) {
            out_clearance_m = 0.0;
            return true;  // 越界 = 碰撞
        }
        if (grid_.is_occupied(gx, gy)) {
            out_clearance_m = 0.0;
            return true;  // footprint 穿墙
        }
        if (unknown_is_blocked && grid_.is_unknown(gx, gy)) {
            out_clearance_m = 0.0;
            return true;  // 未知按占用处理
        }
        double clr = cell_clearance_cells(gx, gy);
        // 边界修正 (同 clearance_m): 到障碍边界 ≈ (ring-0.5)*res
        clr = (clr == 0) ? 0.0 : (clr - 0.5) * grid_.resolution;
        if (clr < min_clr) min_clr = clr;
    }
    out_clearance_m = (min_clr == std::numeric_limits<double>::infinity())
                          ? (30.0 * grid_.resolution) : min_clr;
    return false;
}

// ---------------------------------------------------------------------------
// 主验证流程 (§6.2 十项条件)
// ---------------------------------------------------------------------------
PathValidationResult PathValidator::validate(
        const std::vector<std::pair<double, double>>& path,
        double start_x, double start_y,
        double goal_x,  double goal_y,
        const PathValidationOptions& opt) const {
    PathValidationResult r;
    r.valid = false;
    r.min_clearance = std::numeric_limits<double>::infinity();
    r.collision_index = -1;

    const double res = grid_.resolution;

    // 校验 sample_step
    double step = opt.sample_step;
    if (step <= 0.0 || step > res / 2.0) step = res / 2.0;

    // 条件 1: 起点有效且不在 lethal 区域
    {
        int gx, gy;
        grid_.world_to_grid(start_x, start_y, gx, gy);
        if (gx < 0 || gy < 0 || gx >= grid_.width || gy >= grid_.height) {
            r.code = PathValidationCode::START_OUT_OF_BOUNDS;
            r.reason = to_string(r.code);
            return r;
        }
        if (grid_.is_occupied(gx, gy)) {
            r.code = PathValidationCode::START_IN_OBSTACLE;
            r.reason = to_string(r.code);
            return r;
        }
        if (opt.unknown_is_blocked && grid_.is_unknown(gx, gy)) {
            r.code = PathValidationCode::START_IN_OBSTACLE;
            r.reason = to_string(r.code);
            return r;
        }
    }

    // 条件 2(目标合法性, §6.7): 导航目标本身不得越界 / 落在占用或未知区。
    //   目标在地图外或障碍内应快速失败 (而非仅看路径终点)。
    {
        int ggx, ggy;
        grid_.world_to_grid(goal_x, goal_y, ggx, ggy);
        if (ggx < 0 || ggy < 0 || ggx >= grid_.width || ggy >= grid_.height) {
            r.code = PathValidationCode::GOAL_OUT_OF_BOUNDS;
            r.reason = to_string(r.code);
            return r;
        }
        if (grid_.is_occupied(ggx, ggy)) {
            r.code = PathValidationCode::GOAL_IN_OBSTACLE;
            r.reason = to_string(r.code);
            return r;
        }
        if (opt.unknown_is_blocked && grid_.is_unknown(ggx, ggy)) {
            r.code = PathValidationCode::GOAL_IN_OBSTACLE;
            r.reason = to_string(r.code);
            return r;
        }
    }

    // 条件 3: 路径点数量 >= 2 (含起点的完整路径至少 2 点)
    // 注: A* 输出不含起点, 完整路径 = [start] + path, 故要求 path.size() >= 1
    if (path.empty()) {
        r.code = PathValidationCode::EMPTY_PATH;
        r.reason = to_string(r.code);
        return r;
    }

    // 构造完整路径 [start, ...path]
    std::vector<std::pair<double, double>> full;
    full.reserve(path.size() + 1);
    full.emplace_back(start_x, start_y);
    for (const auto& p : path) full.push_back(p);

    // 条件 2: 终点有效且不在 lethal/unknown
    {
        double gx_w = full.back().first, gy_w = full.back().second;
        int gx, gy;
        grid_.world_to_grid(gx_w, gy_w, gx, gy);
        if (gx < 0 || gy < 0 || gx >= grid_.width || gy >= grid_.height) {
            r.code = PathValidationCode::GOAL_OUT_OF_BOUNDS;
            r.reason = to_string(r.code);
            return r;
        }
        if (grid_.is_occupied(gx, gy)) {
            r.code = PathValidationCode::GOAL_IN_OBSTACLE;
            r.reason = to_string(r.code);
            return r;
        }
        if (opt.unknown_is_blocked && grid_.is_unknown(gx, gy)) {
            r.code = PathValidationCode::GOAL_IN_OBSTACLE;
            r.reason = to_string(r.code);
            return r;
        }
    }

    // 条件 4: 路径终点距目标 < goal_tolerance
    {
        double ex = full.back().first - goal_x;
        double ey = full.back().second - goal_y;
        r.endpoint_error = std::hypot(ex, ey);
        if (r.endpoint_error > opt.goal_tolerance) {
            r.code = PathValidationCode::GOAL_TOO_FAR;
            r.reason = to_string(r.code);
            return r;
        }
    }

    // 条件 5: 路径长度 > 起终点直线距离下限
    {
        double straight = std::hypot(goal_x - start_x, goal_y - start_y);
        double plen = 0.0;
        for (size_t i = 1; i < full.size(); ++i) {
            plen += std::hypot(full[i].first - full[i-1].first,
                               full[i].second - full[i-1].second);
        }
        if (straight > 1e-9 && plen < opt.min_path_length_ratio * straight) {
            r.code = PathValidationCode::PATH_TOO_SHORT;
            r.reason = to_string(r.code);
            return r;
        }
    }

    // 条件 6 & 7 & 8: 逐采样点 footprint 碰撞 / 连线穿墙 / 净空
    //   在完整路径相邻点间按 step 插值, 计算每点朝向 (沿路径切线)
    int sample_idx = 0;
    for (size_t i = 0; i < full.size(); ++i) {
        double cx = full[i].first, cy = full[i].second;
        double theta;
        if (i + 1 < full.size()) {
            theta = std::atan2(full[i+1].second - cy, full[i+1].first - cx);
        } else {
            theta = std::atan2(cy - full[i-1].second, cx - full[i-1].first);
        }
        double clr;
        if (footprint_collision_and_clearance(cx, cy, theta,
                                              opt.footprint,
                                              opt.unknown_is_blocked, clr)) {
            r.code = PathValidationCode::FOOTPRINT_COLLISION;
            r.reason = to_string(r.code);
            r.collision_index = sample_idx;
            r.checked_samples = sample_idx + 1;
            if (r.min_clearance == std::numeric_limits<double>::infinity())
                r.min_clearance = 0.0;
            return r;
        }
        if (clr < r.min_clearance) r.min_clearance = clr;

        // 与下一点的插值
        if (i + 1 < full.size()) {
            double nx = full[i+1].first, ny = full[i+1].second;
            double seg = std::hypot(nx - cx, ny - cy);
            int nsub = static_cast<int>(std::ceil(seg / step));
            if (nsub < 1) nsub = 1;
            for (int s = 1; s < nsub; ++s) {
                double t = static_cast<double>(s) / nsub;
                double ix = cx + (nx - cx) * t;
                double iy = cy + (ny - cy) * t;
                double ith = std::atan2(ny - cy, nx - cx);
                double iclr;
                if (footprint_collision_and_clearance(ix, iy, ith,
                                                      opt.footprint,
                                                      opt.unknown_is_blocked, iclr)) {
                    r.code = PathValidationCode::SEGMENT_PENETRATION;
                    r.reason = to_string(r.code);
                    r.collision_index = sample_idx + s;
                    r.checked_samples = sample_idx + s + 1;
                    if (r.min_clearance == std::numeric_limits<double>::infinity())
                        r.min_clearance = 0.0;
                    return r;
                }
                if (iclr < r.min_clearance) r.min_clearance = iclr;
                ++sample_idx;
            }
        }
        ++sample_idx;
    }
    r.checked_samples = sample_idx;

    // 条件 8: 最小净空 >= min_path_clearance
    if (r.min_clearance < opt.min_path_clearance) {
        r.code = PathValidationCode::INSUFFICIENT_CLEARANCE;
        r.reason = to_string(r.code);
        r.valid = false;
        return r;
    }

    // 全部通过
    r.valid = true;
    r.code = PathValidationCode::OK;
    r.reason = to_string(r.code);
    return r;
}

}  // namespace puppy_nav_core
