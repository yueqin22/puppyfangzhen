// PathValidator — A*/Costmap 路径安全验证器
// ============================================
// 修复根因 (jihua20260818 §6.1): A* 只检查路径中心点 / 用代价阈值，
//   可能返回一条"非空但机器人真实 footprint 无法通过"的路径。
//
// 本验证器对规划结果做**连续 footprint 碰撞检查**:
//   - 在相邻路径点之间按 sample_step(<=resolution/2) 插值采样;
//   - 每个采样位姿把机器人 footprint 多边形旋转 + 平移后栅格化;
//   - 用 Minkowski 和 (footprint ⊕ 障碍) 判定: 任一 footprint cell 落在
//     occupied / unknown(若 unknown_is_blocked) / 越界 → 碰撞;
//   - 计算 footprint 到最近障碍的净空 (clearance)。
//
// 规划成功必须同时满足 §6.2 的 10 项条件，任一不满足返回明确 reason。
//
// 阈值来源: 统一由调用方从 config/simulation_contract.yaml 注入
//   (robot.footprint / planning.min_path_clearance / planning.goal_tolerance / map.resolution)。
#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <string>
#include <cmath>
#include <utility>
#include <limits>

#include "puppy_nav_core/occupancy_grid.h"

namespace puppy_nav_core {

// 2D 点 (机器人局部或世界坐标)
struct Vec2 {
    double x = 0.0, y = 0.0;
};

// 验证失败原因 (机器可解析, 见 §16.4 / §23.12)
enum class PathValidationCode {
    OK = 0,
    EMPTY_PATH,            // 路径为空
    SINGLE_POINT,          // 只有一个点且终点偏离目标 > 容差
    START_IN_OBSTACLE,     // 起点落在占用/未知区域
    GOAL_IN_OBSTACLE,      // 终点落在占用/未知区域
    START_OUT_OF_BOUNDS,   // 起点越界
    GOAL_OUT_OF_BOUNDS,    // 终点越界
    GOAL_TOO_FAR,          // 路径终点距目标 > goal_tolerance
    PATH_TOO_SHORT,        // 路径长度 < 起终点直线距离下限
    FOOTPRINT_COLLISION,   // footprint 穿墙 (含越界/未知)
    SEGMENT_PENETRATION,   // 相邻点安全但连线穿障碍 (采样发现)
    INSUFFICIENT_CLEARANCE // 净空 < min_path_clearance
};

inline const char* to_string(PathValidationCode c) {
    switch (c) {
        case PathValidationCode::OK:                  return "OK";
        case PathValidationCode::EMPTY_PATH:          return "EMPTY_PATH";
        case PathValidationCode::SINGLE_POINT:        return "SINGLE_POINT";
        case PathValidationCode::START_IN_OBSTACLE:   return "START_IN_OBSTACLE";
        case PathValidationCode::GOAL_IN_OBSTACLE:    return "GOAL_IN_OBSTACLE";
        case PathValidationCode::START_OUT_OF_BOUNDS: return "START_OUT_OF_BOUNDS";
        case PathValidationCode::GOAL_OUT_OF_BOUNDS:  return "GOAL_OUT_OF_BOUNDS";
        case PathValidationCode::GOAL_TOO_FAR:        return "GOAL_TOO_FAR";
        case PathValidationCode::PATH_TOO_SHORT:      return "PATH_TOO_SHORT";
        case PathValidationCode::FOOTPRINT_COLLISION: return "FOOTPRINT_COLLISION";
        case PathValidationCode::SEGMENT_PENETRATION: return "SEGMENT_PENETRATION";
        case PathValidationCode::INSUFFICIENT_CLEARANCE: return "INSUFFICIENT_CLEARANCE";
    }
    return "UNKNOWN";
}

// 验证选项 (全部来自统一配置, 禁止硬编码默认值)
struct PathValidationOptions {
    std::vector<Vec2> footprint;                 // 机器人局部坐标系多边形 (建议 CCW)
    double min_path_clearance = 0.08;           // planning.min_path_clearance
    double goal_tolerance     = 0.35;           // planning.goal_tolerance
    double sample_step        = 0.025;          // <= map.resolution / 2
    bool   unknown_is_blocked = true;           // map.unknown_is_blocked
    double min_path_length_ratio = 0.5;         // 路径长度 >= 直线距离 * 该比例
};

// 验证结果 (对应 §6.3 返回结构)
struct PathValidationResult {
    bool   valid = false;
    PathValidationCode code = PathValidationCode::EMPTY_PATH;
    std::string reason = to_string(PathValidationCode::EMPTY_PATH);
    double min_clearance   = 0.0;   // 全程最小净空 (m)
    double endpoint_error  = 0.0;   // 路径终点距目标距离 (m)
    int    collision_index = -1;    // 首个碰撞采样点索引
    int    checked_samples = 0;     // 实际采样点数
};

class PathValidator {
public:
    explicit PathValidator(const OccupancyGrid& grid) : grid_(grid) {}

    // 主入口: 校验一条规划路径。
    //   path: A* 输出 (不含起点, 含终点)
    //   start_*: 机器人真实起点 (用于起点合法性 + 首段朝向)
    //   goal_*: 目标点 (用于 endpoint_error)
    PathValidationResult validate(
        const std::vector<std::pair<double, double>>& path,
        double start_x, double start_y,
        double goal_x,  double goal_y,
        const PathValidationOptions& opt) const;

    // --- 白盒可测的底层原语 ---

    // 计算某位姿 footprint 的碰撞与最小净空。
    // 返回值: 是否发生碰撞 (occupied / unknown / 越界)。
    // 通过 out_clearance_m 输出 footprint 到最近 occupied 的净空 (m)。
    bool footprint_collision_and_clearance(
        double x, double y, double theta,
        const std::vector<Vec2>& footprint,
        bool unknown_is_blocked,
        double& out_clearance_m) const;

    // footprint 半长轴 (局部坐标到原点最大距离), 用于由 footprint 推导可通门宽。
    static double footprint_max_extent(const std::vector<Vec2>& footprint);

    // 由 footprint + 安全裕度推导可通过的**最小门道半宽** (m):
    //   admissible_half = footprint_max_extent + min_path_clearance
    static double admissible_half_width(const std::vector<Vec2>& footprint,
                                        double min_path_clearance);

    // 给定网格, 返回某 cell 到最近 occupied cell 的欧氏距离 (cell 数)。
    // 该 cell 本身 occupied → 返回 0。窗口半径上限 cap_cells (默认 30 ≈ 3m)。
    double cell_clearance_cells(int gx, int gy, int cap_cells = 30) const;

    // 世界坐标下 (x,y) 的净空 (m)。
    double clearance_m(double x, double y) const;

private:
    const OccupancyGrid& grid_;

    // 把世界点变换到 footprint 局部坐标。
    static Vec2 world_to_local(double wx, double wy, double cx, double cy, double theta);

    // 点在多边形内 (射线法)。
    static bool point_in_polygon(double lx, double ly, const std::vector<Vec2>& poly);

    // 收集某位姿 footprint 覆盖的网格 cell (栅格化)。
    void footprint_cells(double cx, double cy, double theta,
                          const std::vector<Vec2>& footprint,
                          std::vector<std::pair<int,int>>& out_cells) const;
};

}  // namespace puppy_nav_core
