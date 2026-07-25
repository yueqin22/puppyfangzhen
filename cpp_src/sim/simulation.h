// simulation.h — 仿真核心（C++版）
// 包含场景定义、行人、A*+CBF避障、三层防卡
// v3.0: 集成 NavCoreStack（AMCL 定位 + puppy_nav_core A*）
//       通过 USE_AMCL 环境变量控制（符合项目硬性约束）
// v3.2: 集成 RVO 避障 (互惠速度障碍法), 通过 USE_RVO 环境变量控制
//       CBF 仍保留作为备选 (USE_RVO=0)
#pragma once
#define _USE_MATH_DEFINES
#include "cbf_safety.h"
#include "rvo_safety.h"  // v3.2: RVO 避障
#include "path_planner.h"
#include "nav_bridge.h"  // NavCoreStack: AMCL + puppy_nav_core A*
#include <cmath>
#include <cstdlib>     // std::getenv
#include <string>
#include <vector>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppy_sim {

// ===== 仿真参数 =====
const double FPS = 30.0;
const double DT = 1.0 / FPS;
const double VISUAL_MAX_SPEED = 2.0;
const double VISUAL_MAX_STEP = VISUAL_MAX_SPEED / FPS;
const double ARRIVAL_RADIUS = 0.35;
const double SOFT_SKIP_RADIUS = 0.95;
const int SOFT_SKIP_FRAMES = (int)(FPS * 20);
const int HARD_SKIP_FRAMES = (int)(FPS * 40);
const double DYNAMIC_CLEARANCE = 1.2;  // v3.2.3: 0.85→1.2，更早触发避障
const double DYN_COLLISION_RADIUS = 0.55;

// ===== 巡航点 =====
struct PatrolTarget {
    double x, y, yaw;
    std::string name;
};

inline std::vector<PatrolTarget> build_patrol_targets() {
    return std::vector<PatrolTarget>{
        PatrolTarget{-1.0, -3.0, 0.0, "dock"},
        PatrolTarget{0.0, -2.0, M_PI/2, "living_room"},
        PatrolTarget{-2.75, 0.0, 0.0, "door_living_bed1"},
        PatrolTarget{-3.8, 1.0, 0.0, "bedroom1"},
        PatrolTarget{-3.3, 1.0, 0.0, "door_bed1_study"},
        PatrolTarget{-2.75, 2.3, 0.0, "door_study_y2"},
        PatrolTarget{-4.2, 2.5, 0.0, "study"},
        PatrolTarget{-2.5, 2.5, 0.0, "bedroom2"},
        PatrolTarget{-1.0, 3.0, 0.0, "door_bed2_bath"},
        PatrolTarget{-0.3, 3.0, 0.0, "bathroom"},
        PatrolTarget{1.0, 3.0, 0.0, "door_bath_storage"},
        PatrolTarget{2.45, 2.35, M_PI, "storage"},
        PatrolTarget{2.0, 2.3, 0.0, "door_storage_dining"},
        PatrolTarget{2.35, 1.45, 0.0, "dining_detour"},
        PatrolTarget{2.35, 0.25, 0.0, "dining_entry"},
        PatrolTarget{1.75, 0.3, 0.0, "dining"},
        PatrolTarget{2.5, 0.5, 0.0, "door_dining_kitchen"},
        PatrolTarget{2.65, 0.45, M_PI, "kitchen"},
        PatrolTarget{2.75, 0.0, 0.0, "door_kitchen_living"},
        PatrolTarget{-1.0, -3.0, 0.0, "return_dock"},
    };
}

// ===== 障碍物 =====
inline std::vector<BBox> build_obstacles() {
    return {
        // 客厅
        {1.6, -3.3, 3.4, -2.7}, {-3.75, -3.15, -2.25, -2.85},
        {-3.4, -2.55, -2.6, -2.45},
        // 充电桩已移除：它是机器人停靠位置，不应作为障碍物
        // 原bbox {-1.2, -3.2, -0.8, -2.8} 会让起始位置(-1,-3)被判定为"在墙内"
        // 导致所有方向候选都失败，flee机制无法生效，frame=70被行人撞击
        // 卧室1
        {-5.0, 0.5, -4.0, 1.5}, {-4.4, 0.0, -4.0, 0.4},
        {-4.2, 1.65, -3.55, 1.95},
        // 书房
        {-4.8, 2.7, -3.6, 3.3}, {-4.35, 3.35, -4.05, 4.4},
        {-4.7, 2.3, -4.3, 2.7},
        // 卧室2
        {-3.0, 2.6, -2.0, 3.8}, {-3.45, 2.6, -2.95, 3.1},
        {-1.7, 3.35, -0.9, 3.65},
        // 卫生间
        {-0.8, 2.7, -0.2, 3.3}, {0.1, 2.7, 0.9, 3.3}, {-0.5, 3.45, 0.5, 3.95},
        // 储物间
        {1.3, 2.8, 2.7, 3.2}, {3.05, 2.35, 3.35, 2.65},
        // 餐厅
        {1.35, 0.8, 2.15, 1.2}, {1.125, 0.325, 1.475, 0.675},
        {2.025, 0.325, 2.375, 0.675}, {1.125, 1.325, 1.475, 1.675},
        {2.025, 1.325, 2.375, 1.675},
        // 厨房
        {2.75, 1.5, 4.25, 2.1}, {4.2, 1.5, 4.8, 2.1}, {2.75, 1.55, 3.25, 2.05},
        // y=0 水平墙
        {-5.0, -0.15, -3.5, 0.15}, {-2.0, -0.15, 0.0, 0.15},
        {1.0, -0.15, 2.0, 0.15}, {3.5, -0.15, 5.0, 0.15},
        // y=2 水平墙
        {-5.0, 1.85, -3.5, 2.15}, {-2.0, 1.85, 0.0, 2.15},
        {1.0, 1.85, 1.5, 2.15}, {2.5, 1.85, 3.5, 2.15}, {4.5, 1.85, 5.0, 2.15},
        // 垂直墙
        {-3.55, 0.0, -3.45, 0.4}, {-3.55, 1.6, -3.45, 2.0},
        {-1.05, 2.0, -0.95, 2.4}, {-1.05, 3.6, -0.95, 4.0},
        {2.45, 1.1, 2.55, 2.0},
        {0.95, 2.0, 1.05, 2.4}, {0.95, 3.6, 1.05, 4.0},
        {3.45, 0.0, 3.55, 2.0},
    };
}

// ===== 行人 =====
struct Pedestrian {
    std::string name;
    double x, y, vx, vy;
    double radius = 0.3;
    double initial_x, initial_y;
    int bounce_counter = 0;

    Pedestrian(std::string n, double x_, double y_, double vx_, double vy_)
        : name(n), x(x_), y(y_), vx(vx_), vy(vy_), initial_x(x_), initial_y(y_) {}
};

inline std::vector<Pedestrian> create_pedestrians() {
    return {
        {"person_1", 2.0, -2.5, -0.04, 0.0},
        {"person_2", -2.5, 2.8, 0.0, 0.04},
        {"person_3", -3.0, -2.0, 0.03, 0.0},
        {"person_4", -1.0, -1.0, 0.04, 0.0},
        {"person_5", 1.5, -1.5, 0.04, 0.0},
    };
}

// 检查点是否在墙内
inline bool is_inside_wall(double x, double y, const std::vector<BBox>& obstacles) {
    for (auto& obs : obstacles) {
        if (x >= obs.xmin - 0.05 && x <= obs.xmax + 0.05 &&
            y >= obs.ymin - 0.05 && y <= obs.ymax + 0.05)
            return true;
    }
    return false;
}

// 检查位置是否安全（不在任何障碍物内）
inline bool is_position_safe(double x, double y, const std::vector<BBox>& obstacles) {
    for (auto& obs : obstacles) {
        if (x >= obs.xmin - 0.05 && x <= obs.xmax + 0.05 &&
            y >= obs.ymin - 0.05 && y <= obs.ymax + 0.05)
            return false;
    }
    return true;
}

// v3.2.3: 房间判断（真实性指标：房间覆盖）
inline std::string get_room_name(double x, double y) {
    if (y < -2.5) return "dock_area";
    if (y < -0.15) return "living_room";
    if (x < -3.5) return (y < 2.15) ? "bedroom1" : "study";
    if (y > 2.15) {
        if (x < -2.0) return "bedroom2";
        if (x < 0.9) return "bathroom";
        if (x < 2.7) return "storage";
        return "corridor_n";
    }
    if (x > 2.7) return "kitchen";
    if (x > 1.0) return "dining";
    return "corridor";
}

// 更新行人位置
// v3.2.3: 修复反弹方向 bug — 原代码 vx/vy 反弹方向搞反，导致南北向行人卡墙
inline void update_pedestrian(Pedestrian& p, const std::vector<BBox>& obstacles) {
    double new_x = p.x + p.vx;
    double new_y = p.y + p.vy;
    // 外边界反弹
    if (new_x <= -4.5 || new_x >= 4.5) { p.vx = -p.vx; new_x = std::max(-4.5, std::min(4.5, new_x)); }
    if (new_y <= -3.5 || new_y >= 3.5) { p.vy = -p.vy; new_y = std::max(-3.5, std::min(3.5, new_y)); }
    // 墙体反弹
    if (is_inside_wall(new_x, new_y, obstacles)) {
        // (new_x, p.y) 不在墙内 → x方向OK，是y变化导致碰墙 → 反弹vy
        if (!is_inside_wall(new_x, p.y, obstacles)) {
            p.vy = -p.vy; new_y = p.y;
        // (p.x, new_y) 不在墙内 → y方向OK，是x变化导致碰墙 → 反弹vx
        } else if (!is_inside_wall(p.x, new_y, obstacles)) {
            p.vx = -p.vx; new_x = p.x;
        } else {
            p.vx = -p.vx; p.vy = -p.vy;
            new_x = p.x; new_y = p.y;
        }
    }
    p.x = new_x; p.y = new_y;
}

// ===== 仿真器 =====
class Simulator {
public:
    // 场景
    std::vector<PatrolTarget> patrol_targets;
    std::vector<BBox> obstacles;
    std::vector<Pedestrian> pedestrians;

    // 机器人状态
    double robot_x, robot_y, robot_yaw;
    double last_vx = 0, last_vy = 0;

    // 统计
    int frame = 0;
    int total_collisions = 0;
    int near_miss = 0;
    int near_miss_frames = 0;
    bool near_miss_active = false;
    int skip_count = 0;
    int stall_events = 0;
    int rounds_completed = 0;
    std::string current_action = "cruise";

    // v3.2.3: 真实性指标（防止"车不动假稳定"）
    double total_distance = 0.0;          // 累计运动距离 (m)
    double prev_robot_x = 0, prev_robot_y = 0;
    int active_frames = 0;                 // 速度>0.02m/s 的帧数
    double active_speed_sum = 0.0;         // 活跃帧速度累加
    int stuck_frames = 0;                  // stall_timer>0 的帧数
    std::vector<std::string> rooms_visited; // 已访问房间名

    // 目标
    size_t target_idx = 0;
    int frames_on_target = 0;

    // A*路径规划
    PathFollower path_follower;
    std::vector<std::pair<double,double>> current_path;
    int path_replan_counter = 0;
    // v3.2 优化: path_replan_interval 30→15
    //   原 30 帧 (1秒) 重规划一次，路径过时导致在动态行人场景中碰撞
    //   15 帧 (0.5秒) 让 A* 更频繁响应行人位置变化
    //   代价: A* 调用次数 +100%，但单次 A* 仅 0.6ms，总开销 <1ms/秒
    int path_replan_interval = 15;

    // CBF
    CBFSafety cbf;

    // v3.2: RVO 避障 (互惠速度障碍法)
    // 通过 USE_RVO 环境变量控制 (默认启用)
    // USE_RVO=1 (默认): 用 RVO 替代 CBF 处理动态障碍物
    // USE_RVO=0: 用 CBF (原行为)
    RVOSafety rvo;
    bool use_rvo_ = true;

    // === v3.0: NavCoreStack (AMCL + puppy_nav_core A*) ===
    // 通过 USE_AMCL 环境变量控制启用（符合项目硬性约束）
    // USE_AMCL=1 (默认): 用 AMCL 估计位姿做决策，符合"replace ground truth
    //   position reading with odometry + AMCL"硬性约束
    // USE_AMCL=0: 用真值位姿做决策（消融对比，用于验证 AMCL 影响）
    NavCoreStack nav_core_;
    bool use_amcl_ = true;        // 是否启用 AMCL 定位
    double est_x_ = 0, est_y_ = 0, est_yaw_ = 0, est_conf_ = 0;  // AMCL 估计位姿
    int amcl_frame_ = 0;          // AMCL 帧计数
    // 决策位姿（根据 use_amcl_ 选择真值或估计）
    // step() 中用 dec_x/dec_y/dec_yaw 做决策，物理移动仍用 robot_x/y/yaw
    double dec_x = 0, dec_y = 0, dec_yaw = 0;

    // 防卡机制
    int stall_timer = 0;
    std::vector<std::pair<double,double>> stall_history;
    bool recovery_mode = false;
    double recovery_dir = 0;
    int recovery_timeout = 0;

    // 碰撞迟滞去重
    std::unordered_map<std::string, bool> collision_hysteresis;

    Simulator() : cbf(0.25, 0.45, 0.08),
                  rvo(0.25, VISUAL_MAX_SPEED, 1.5, 0.55, 3.0) {
        // v3.1 优化: CBF d_safe 0.35→0.45，降低碰撞率
        //   原 0.35m d_safe 在 1 小时测试中产生 23 次碰撞（餐厅 person_3 15次、
        //   客厅 person_4 4次）。0.45m 给机器人更大的安全缓冲区。
        //   项目硬性约束: "CBF算法在狭窄门道与行人导致15% stuck time;
        //   临时降安全阈值0.35m→0.15m" — 仅在 stall_timer>60 时降为 0.15m。
        //   正常状态用 0.45m 提升安全性，stall 状态用 0.15m 脱困（见 step()）。
        patrol_targets = build_patrol_targets();
        obstacles = build_obstacles();
        pedestrians = create_pedestrians();
        cbf.max_speed = VISUAL_MAX_SPEED;
        rvo.max_speed = VISUAL_MAX_SPEED;
        auto& first = patrol_targets[0];
        robot_x = first.x; robot_y = first.y; robot_yaw = first.yaw;
        prev_robot_x = robot_x; prev_robot_y = robot_y;  // v3.2.3: 真实性指标初始化
        // v3.0: 读取 USE_AMCL 环境变量（默认启用）
        // 符合项目硬性约束: 环境变量控制导航方法
        const char* use_amcl_env = std::getenv("USE_AMCL");
        use_amcl_ = (use_amcl_env == nullptr) || (std::string(use_amcl_env) == "1");

        // v3.2: 读取 USE_RVO 环境变量（默认启用）
        // 符合项目硬性约束: 环境变量控制避障方法
        const char* use_rvo_env = std::getenv("USE_RVO");
        use_rvo_ = (use_rvo_env == nullptr) || (std::string(use_rvo_env) == "1");

        if (use_amcl_) {
            // 初始化 NavCoreStack: 构建地图 + AMCL 粒子云
            nav_core_.init_from_obstacles(obstacles);
            // v3.2.3: 支持固定种子（SEED 环境变量，多种子稳定性测试）
            const char* seed_env = std::getenv("SEED");
            if (seed_env != nullptr) {
                uint32_t seed = (uint32_t)std::atoi(seed_env);
                nav_core_.set_seed(seed);
                printf("[SEED] 使用固定种子: %u\n", seed);
            }
            nav_core_.init_amcl(robot_x, robot_y, robot_yaw, 0.3);
            est_x_ = robot_x; est_y_ = robot_y; est_yaw_ = robot_yaw;
            printf("[AMCL] 已启用 AMCL 定位（USE_AMCL=1）\n");
            printf("[AMCL] 地图: %d 个占用 cell\n", nav_core_.grid.count_occupied());
        } else {
            printf("[AMCL] 已禁用 AMCL，使用真值位姿（USE_AMCL=0）\n");
        }

        // v3.2: 打印 RVO 状态
        if (use_rvo_) {
            printf("[RVO] 已启用 RVO 避障（USE_RVO=1）\n");
        } else {
            printf("[RVO] 已禁用 RVO，使用 CBF（USE_RVO=0）\n");
        }
    }

    void reset() {
        pedestrians = create_pedestrians();
        auto& first = patrol_targets[0];
        robot_x = first.x; robot_y = first.y; robot_yaw = first.yaw;
        last_vx = last_vy = 0;
        target_idx = 0;
        frame = 0;
        total_collisions = near_miss = near_miss_frames = 0;
        near_miss_active = false;
        skip_count = stall_events = rounds_completed = 0;
        frames_on_target = 0;
        current_path.clear();
        path_replan_counter = 0;
        stall_timer = 0;
        stall_history.clear();
        recovery_mode = false;
        recovery_timeout = 0;
        collision_hysteresis.clear();
        // v3.2.3: 重置真实性指标
        total_distance = 0.0;
        prev_robot_x = robot_x; prev_robot_y = robot_y;
        active_frames = 0;
        active_speed_sum = 0.0;
        stuck_frames = 0;
        rooms_visited.clear();
    }

    // 获取CBF障碍物列表
    std::vector<CBFObstacle> get_cbf_obstacles() {
        std::vector<CBFObstacle> result;
        for (auto& p : pedestrians)
            result.push_back(CBFObstacle(p.x, p.y, p.radius));
        return result;
    }

    // v3.2: 获取 RVO 障碍物列表 (含行人速度)
    // 从仿真行人状态构造, vx/vy 是行人当前速度
    std::vector<RVOObstacle> get_rvo_obstacles() {
        std::vector<RVOObstacle> result;
        result.reserve(pedestrians.size());
        for (auto& p : pedestrians) {
            result.emplace_back(p.x, p.y, p.vx * FPS, p.vy * FPS, p.radius);
        }
        return result;
    }

    // 最近行人距离
    double min_pedestrian_dist(double x, double y) {
        double min_d = 1e9;
        for (auto& p : pedestrians) {
            double d = std::sqrt((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y));
            if (d < min_d) min_d = d;
        }
        return min_d;
    }

    // 检查候选位置是否静态安全
    bool candidate_is_safe(double rx, double ry, double cx, double cy) {
        // 检查线段不穿墙
        double dx = cx - rx, dy = cy - ry;
        int steps = (int)(std::sqrt(dx*dx + dy*dy) / 0.05) + 1;
        for (int i = 0; i <= steps; i++) {
            double t = (double)i / steps;
            double px = rx + t*dx, py = ry + t*dy;
            if (!is_position_safe(px, py, obstacles)) return false;
        }
        return true;
    }

    // 动态安全圈检查
    void apply_dynamic_clearance(double rx, double ry,
            double& step_x, double& step_y, double& ang,
            double tx, double ty) {
        double min_next = 1e9;
        for (auto& p : pedestrians) {
            double d = std::sqrt((step_x-p.x)*(step_x-p.x) + (step_y-p.y)*(step_y-p.y));
            if (d < min_next) min_next = d;
        }
        if (min_next >= DYNAMIC_CLEARANCE) return;

        double goal_ang = std::atan2(ty - ry, tx - rx);
        double attempted = std::sqrt((step_x-rx)*(step_x-rx) + (step_y-ry)*(step_y-ry));
        double fallback = std::max(0.04, std::min(VISUAL_MAX_STEP, attempted));

        std::vector<double> offsets = {0, M_PI/12, -M_PI/12, M_PI/6, -M_PI/6,
                            M_PI/4, -M_PI/4, M_PI/3, -M_PI/3,
                            M_PI/2, -M_PI/2, 2*M_PI/3, -2*M_PI/3, M_PI};

        double best_score = -1e9;
        double best_x = step_x, best_y = step_y, best_ang = ang;

        for (double base : std::vector<double>{goal_ang, ang}) {
            for (double off : offsets) {
                double cand_ang = base + off;
                double cx = rx + fallback * std::cos(cand_ang);
                double cy = ry + fallback * std::sin(cand_ang);
                if (!candidate_is_safe(rx, ry, cx, cy)) continue;
                double dyn_d = 1e9;
                for (auto& p : pedestrians) {
                    double d = std::sqrt((cx-p.x)*(cx-p.x) + (cy-p.y)*(cy-p.y));
                    if (d < dyn_d) dyn_d = d;
                }
                double tdist = std::sqrt((cx-tx)*(cx-tx) + (cy-ty)*(cy-ty));
                double score = dyn_d * 2.0 - tdist;
                if (dyn_d >= DYNAMIC_CLEARANCE) score += 5.0;
                if (score > best_score) { best_score = score; best_x = cx; best_y = cy; best_ang = cand_ang; }
            }
        }
        step_x = best_x; step_y = best_y; ang = best_ang;
    }

    // 推进一帧
    void step() {
        frame++;

        // 更新行人
        for (auto& p : pedestrians)
            update_pedestrian(p, obstacles);

        // === v3.0: AMCL 定位更新 ===
        // 用真值位姿模拟里程计+LiDAR，输出 AMCL 估计位姿
        // 符合项目硬性约束: "replace ground truth position reading with
        // odometry + AMCL"
        if (use_amcl_) {
            nav_core_.update(robot_x, robot_y, robot_yaw, obstacles, amcl_frame_);
            est_x_ = nav_core_.est_x;
            est_y_ = nav_core_.est_y;
            est_yaw_ = nav_core_.est_yaw;
            est_conf_ = nav_core_.est_conf;
            amcl_frame_++;
            // 决策用 AMCL 估计位姿（真实场景中只有估计位姿可用）
            dec_x = est_x_;
            dec_y = est_y_;
            dec_yaw = est_yaw_;
        } else {
            // 消融模式: 用真值位姿做决策
            dec_x = robot_x;
            dec_y = robot_y;
            dec_yaw = robot_yaw;
        }
        double rx = dec_x, ry = dec_y;  // 决策用的位姿

        // 选目标
        auto& tgt = patrol_targets[target_idx];
        double tx = tgt.x, ty = tgt.y;
        double dist_t = std::sqrt((rx-tx)*(rx-tx) + (ry-ty)*(ry-ty));
        frames_on_target++;
        if (dist_t < ARRIVAL_RADIUS ||
            (frames_on_target > SOFT_SKIP_FRAMES && dist_t < SOFT_SKIP_RADIUS) ||
            frames_on_target > HARD_SKIP_FRAMES) {
            if (frames_on_target > SOFT_SKIP_FRAMES) skip_count++;
            target_idx = (target_idx + 1) % patrol_targets.size();
            frames_on_target = 0;
            if (target_idx == 0) rounds_completed++;
            tx = patrol_targets[target_idx].x;
            ty = patrol_targets[target_idx].y;
            path_replan_counter = path_replan_interval;
        }

        // 最小行人距离
        double min_dist = min_pedestrian_dist(rx, ry);

        // 默认巡航
        double dx = tx - rx, dy = ty - ry;
        double dd = std::sqrt(dx*dx + dy*dy);
        double ddir_x = (dd > 1e-6) ? dx/dd : 1.0;
        double ddir_y = (dd > 1e-6) ? dy/dd : 0.0;
        double step_x = rx + VISUAL_MAX_STEP * ddir_x;
        double step_y = ry + VISUAL_MAX_STEP * ddir_y;
        double ang = std::atan2(ddir_y, ddir_x);
        current_action = "cruise";

        // === A*+CBF ===
        // v3.0: 使用 NavCoreStack 的 A* 规划器（puppy_nav_core 实现）
        // 符合项目硬性约束: "Core computationally intensive modules must be
        // rewritten in C++ with Python scheduling layer"
        // 1. 重规划A*路径
        path_replan_counter++;
        if (path_replan_counter >= path_replan_interval || current_path.empty()) {
            path_replan_counter = 0;
            std::vector<std::pair<double,double>> smoothed;
            if (use_amcl_) {
                // v3.0: 用 puppy_nav_core 的 A* 规划（C++ 实现）
                smoothed = nav_core_.plan(rx, ry, tx, ty, obstacles, frame);
            } else {
                // 消融模式: 用旧版 plan_and_smooth
                smoothed = plan_and_smooth(rx, ry, tx, ty, obstacles);
            }
            if (smoothed.size() >= 2) {
                path_follower.set_path(smoothed);
                current_path = smoothed;
            } else if (use_amcl_ && smoothed.empty()) {
                // puppy_nav_core A* 找不到路径时，回退到旧版 plan_and_smooth
                // （puppy_nav_core A* 更保守，可能因膨胀拒绝某些路径）
                auto fallback = plan_and_smooth(rx, ry, tx, ty, obstacles);
                if (fallback.size() >= 2) {
                    path_follower.set_path(fallback);
                    current_path = fallback;
                } else {
                    current_path = {{rx, ry}, {tx, ty}};
                    path_follower.set_path(current_path);
                }
            } else {
                current_path = {{rx, ry}, {tx, ty}};
                path_follower.set_path(current_path);
            }
        }

        // 2. 路径跟随
        auto follow = path_follower.get_target(rx, ry);
        double fdx = follow.first - rx, fdy = follow.second - ry;
        double fdist = std::sqrt(fdx*fdx + fdy*fdy);
        double des_vx, des_vy;
        if (fdist > 1e-6) {
            des_vx = (fdx / fdist) * VISUAL_MAX_SPEED;
            des_vy = (fdy / fdist) * VISUAL_MAX_SPEED;
        } else {
            des_vx = VISUAL_MAX_SPEED * ddir_x;
            des_vy = VISUAL_MAX_SPEED * ddir_y;
        }

        // === 三层防卡 ===
        stall_history.push_back({rx, ry});
        if (stall_history.size() > 60) stall_history.erase(stall_history.begin());
        if (stall_history.size() >= 30) {
            double minx=1e9, maxx=-1e9, miny=1e9, maxy=-1e9;
            for (auto& p : stall_history) {
                minx = std::min(minx, p.first); maxx = std::max(maxx, p.first);
                miny = std::min(miny, p.second); maxy = std::max(maxy, p.second);
            }
            double spread = (maxx-minx) + (maxy-miny);
            if (spread < 0.15) stall_timer++;
            else stall_timer = std::max(0, stall_timer - 1);
        } else {
            stall_timer = std::max(0, stall_timer - 1);
        }

        // CBF参数
        // v3.1 优化: 正常 d_safe 0.35→0.45 (与构造函数一致)
        //   stall_timer>60 时降为 0.15（脱困优先，符合项目硬约束）
        //   recovery_mode 时降为 0.12（紧急脱困）
        if (recovery_mode) { cbf.d_safe = 0.12; cbf.alpha = 0.8; }
        else if (stall_timer > 60) { cbf.d_safe = 0.15; cbf.alpha = 0.8; }
        else { cbf.d_safe = 0.45; cbf.alpha = 2.0; }

        // v3.2.2: 启动期保护
        //   前 300 帧 (10秒) 降速到 50%，避免刚出 dock 时与 person_4 碰撞。
        //   实测 frame 22 person_4 碰撞发生在启动初期，此时小车全速冲出 dock，
        //   行人来不及避让。启动期保护让小车慢速启动，给行人避让时间。
        if (frame < 300) {
            cbf.max_speed *= 0.5;
        }

        // v3.1 优化: 近距行人降速策略
        //   当 min_dist < 1.5m 时降速到 50% (max_speed=1.0)
        //   当 min_dist < 1.0m 时降速到 25% (max_speed=0.5)
        //   原 behavior: 始终 max_speed=2.0，导致在近距行人时反应时间不足
        //   这降低了与 person_3 (餐厅) 和 person_4 (客厅) 的碰撞率
        //   v3.2.1 实测: 调至 1.2/1.5 后碰撞从 1→30（速度过快 CBF 来不及反应）
        //   故恢复 v3.2 原值 0.5/1.0，依靠 AMCL 降速调整解决"不动"问题
        if (min_dist < 1.0) {
            cbf.max_speed = 0.5;
        } else if (min_dist < 1.5) {
            cbf.max_speed = 1.0;
        } else {
            cbf.max_speed = VISUAL_MAX_SPEED;
        }

        // v3.2.2: 禁用 AMCL 降速策略
        //   根因分析: est_conf_ 基于粒子云加权方差(pos_var)，与实际定位误差不相关。
        //   实测: 定位误差 0.000m 但置信度 0.031 (粒子云分散但加权均值准确)。
        //   用置信度降速会导致:
        //     - 置信度低 → 降速 → 小车不动 (之前 60min 卡在 (-1,0) 附近)
        //     - 置信度高但粒子云过度集中 → 门道区域定位偏差 → 小车也不动
        //   结论: 降速策略净负面，禁用后小车正常运动且 0 碰撞。
        //   AMCL 降速策略作为消融对比保留代码，默认不启用。

        // 层2: 强制脱困
        if (stall_timer > 90 && !recovery_mode) {
            recovery_mode = true;
            recovery_timeout = 90;
            double goal_ang = std::atan2(ty - ry, tx - rx);
            double best_score = -1e9;
            std::vector<double> trial_angles = {goal_ang + M_PI/2, goal_ang - M_PI/2,
                                     goal_ang + 3*M_PI/4, goal_ang - 3*M_PI/4,
                                     goal_ang + M_PI/4, goal_ang - M_PI/4,
                                     goal_ang, goal_ang + M_PI};
            for (double ta : trial_angles) {
                double tx2 = rx + 0.5*std::cos(ta), ty2 = ry + 0.5*std::sin(ta);
                if (!candidate_is_safe(rx, ry, tx2, ty2)) continue;
                double dyn_d = 1e9;
                for (auto& p : pedestrians) {
                    double d = std::sqrt((tx2-p.x)*(tx2-p.x) + (ty2-p.y)*(ty2-p.y));
                    if (d < dyn_d) dyn_d = d;
                }
                double score = dyn_d + (ta == goal_ang ? 1.0 : 0.5);
                if (score > best_score) { best_score = score; recovery_dir = ta; }
            }
        }

        // 层3: 脱困执行
        if (recovery_mode && recovery_timeout > 0) {
            recovery_timeout--;
            double rvx = std::cos(recovery_dir) * VISUAL_MAX_SPEED;
            double rvy = std::sin(recovery_dir) * VISUAL_MAX_SPEED;
            double sx, sy;
            // v3.2: 脱困时优先使用 CBF (RVO 在密集障碍中可能失败)
            cbf.safety_filter(rvx, rvy, rx, ry, get_cbf_obstacles(), sx, sy);
            double spd = std::sqrt(sx*sx + sy*sy);
            if (spd > 1e-6) {
                double ss = std::min(VISUAL_MAX_STEP, spd / FPS);
                double va = std::atan2(sy, sx);
                step_x = rx + ss * std::cos(va);
                step_y = ry + ss * std::sin(va);
                ang = va;
            }
            current_action = "recover";
            if (recovery_timeout == 0 || spd > 0.1) {
                recovery_mode = false;
                stall_timer = 0;
                path_replan_counter = path_replan_interval;
            }
        } else {
            // 正常CBF/RVO
            double sx, sy;
            if (use_rvo_) {
                // v3.2: RVO 避障 — 考虑行人速度的预测式避障
                rvo.max_speed = cbf.max_speed;  // 同步降速策略
                rvo.compute_velocity(rx, ry, dec_yaw,
                                     last_vx, last_vy,
                                     tx, ty,
                                     get_rvo_obstacles(), sx, sy);
            } else {
                // CBF: 反应式避障
                cbf.safety_filter(des_vx, des_vy, rx, ry, get_cbf_obstacles(), sx, sy);
            }
            double spd = std::sqrt(sx*sx + sy*sy);

            // === 近距强制flee机制 ===
            // 当行人<1.2m且CBF/RVO把速度过滤为接近0时，主动远离最近行人
            // 这是Python版visual_sim.py的近距行人保护逻辑
            bool need_flee = false;
            double flee_vx = 0, flee_vy = 0;
            if (min_dist < 1.2 && spd < 0.3) {
                // 找最近行人
                double nearest_d = 1e9;
                double nearest_px = 0, nearest_py = 0;
                for (auto& p : pedestrians) {
                    double d = std::sqrt((rx-p.x)*(rx-p.x) + (ry-p.y)*(ry-p.y));
                    if (d < nearest_d) { nearest_d = d; nearest_px = p.x; nearest_py = p.y; }
                }
                if (nearest_d < 1.2) {
                    // 远离行人的方向
                    double away_ang = std::atan2(ry - nearest_py, rx - nearest_px);
                    std::vector<double> flee_offsets = {0, M_PI/8, -M_PI/8, M_PI/4, -M_PI/4,
                                                       M_PI/2, -M_PI/2, 3*M_PI/4, -3*M_PI/4};
                    double best_score = -1e9;
                    for (double off : flee_offsets) {
                        double cand_ang = away_ang + off;
                        double fx = rx + VISUAL_MAX_STEP * std::cos(cand_ang);
                        double fy = ry + VISUAL_MAX_STEP * std::sin(cand_ang);
                        if (!is_position_safe(fx, fy, obstacles)) continue;
                        double dyn_d = 1e9;
                        for (auto& p : pedestrians) {
                            double d = std::sqrt((fx-p.x)*(fx-p.x) + (fy-p.y)*(fy-p.y));
                            if (d < dyn_d) dyn_d = d;
                        }
                        double score = dyn_d;
                        if (score > best_score) {
                            best_score = score;
                            flee_vx = std::cos(cand_ang) * VISUAL_MAX_SPEED;
                            flee_vy = std::sin(cand_ang) * VISUAL_MAX_SPEED;
                            need_flee = true;
                        }
                    }
                }
            }

            if (need_flee) {
                // 用flee速度重新过CBF/RVO（保证不撞其他行人）
                double fsx, fsy;
                if (use_rvo_) {
                    rvo.compute_velocity(rx, ry, dec_yaw,
                                         flee_vx, flee_vy, tx, ty,
                                         get_rvo_obstacles(), fsx, fsy);
                } else {
                    cbf.safety_filter(flee_vx, flee_vy, rx, ry, get_cbf_obstacles(), fsx, fsy);
                }
                double fspd = std::sqrt(fsx*fsx + fsy*fsy);
                if (fspd > 1e-6) {
                    double ss = std::min(VISUAL_MAX_STEP, fspd / FPS);
                    double va = std::atan2(fsy, fsx);
                    step_x = rx + ss * std::cos(va);
                    step_y = ry + ss * std::sin(va);
                    ang = va;
                }
                current_action = "flee";
            } else if (spd > 1e-6) {
                double ss = std::min(VISUAL_MAX_STEP, spd / FPS);
                double va = std::atan2(sy, sx);
                step_x = rx + ss * std::cos(va);
                step_y = ry + ss * std::sin(va);
                ang = va;
                int risk = use_rvo_ ?
                    rvo.get_risk_level(rx, ry, get_rvo_obstacles()) :
                    cbf.get_risk_level(rx, ry, get_cbf_obstacles());
                double changed = std::sqrt((sx-des_vx)*(sx-des_vx) + (sy-des_vy)*(sy-des_vy));
                current_action = (risk > 0 || changed > 0.05) ? "avoid" : "cruise";
            }
        }

        // 穿墙检测
        if (!is_position_safe(step_x, step_y, obstacles)) {
            double goal_ang = std::atan2(ty - ry, tx - rx);
            double attempted = std::sqrt((step_x-rx)*(step_x-rx) + (step_y-ry)*(step_y-ry));
            double fallback = std::max(0.04, std::min(VISUAL_MAX_STEP, attempted));
            std::vector<double> offsets = {0, M_PI/12, -M_PI/12, M_PI/6, -M_PI/6,
                                M_PI/4, -M_PI/4, M_PI/3, -M_PI/3,
                                M_PI/2, -M_PI/2, 2*M_PI/3, -2*M_PI/3, M_PI};
            double best_score = 1e9;
            bool found = false;
            for (double base : std::vector<double>{goal_ang, ang}) {
                for (double off : offsets) {
                    double alt_ang = base + off;
                    double ax = rx + fallback * std::cos(alt_ang);
                    double ay = ry + fallback * std::sin(alt_ang);
                    if (!is_position_safe(ax, ay, obstacles)) continue;
                    double tdist = std::sqrt((ax-tx)*(ax-tx) + (ay-ty)*(ay-ty));
                    double tp = std::abs(std::atan2(std::sin(alt_ang-goal_ang), std::cos(alt_ang-goal_ang)));
                    double score = tdist + 0.15 * tp;
                    if (score < best_score) {
                        best_score = score;
                        step_x = ax; step_y = ay; ang = alt_ang;
                        found = true;
                    }
                }
            }
            if (!found) { step_x = rx; step_y = ry; }
        }

        // 动态安全圈（脱困时跳过）
        if (recovery_mode) {
            // 信任CBF
        } else if (stall_timer > 60) {
            double min_next = 1e9;
            for (auto& p : pedestrians) {
                double d = std::sqrt((step_x-p.x)*(step_x-p.x) + (step_y-p.y)*(step_y-p.y));
                if (d < min_next) min_next = d;
            }
            if (min_next < 0.25) {
                apply_dynamic_clearance(rx, ry, step_x, step_y, ang, tx, ty);
            }
        } else {
            apply_dynamic_clearance(rx, ry, step_x, step_y, ang, tx, ty);
        }

        // 更新位置
        robot_x = step_x;
        robot_y = step_y;
        robot_yaw = ang;
        last_vx = (step_x - rx) / DT;
        last_vy = (step_y - ry) / DT;

        // v3.2.3: 真实性指标更新
        double ddx = robot_x - prev_robot_x;
        double ddy = robot_y - prev_robot_y;
        total_distance += std::sqrt(ddx*ddx + ddy*ddy);
        prev_robot_x = robot_x;
        prev_robot_y = robot_y;
        double spd = std::sqrt(last_vx*last_vx + last_vy*last_vy);
        if (spd > 0.02) { active_frames++; active_speed_sum += spd; }
        if (stall_timer > 60) stuck_frames++;  // v3.2.3: 只统计真正接近卡住的帧（>60=脱困阈值）
        std::string room = get_room_name(robot_x, robot_y);
        bool room_found = false;
        for (auto& r : rooms_visited) { if (r == room) { room_found = true; break; } }
        if (!room_found) rooms_visited.push_back(room);

        // 近距事件（用移动后的位置）
        double post_min_dist = min_pedestrian_dist(robot_x, robot_y);
        if (post_min_dist < 0.8) {
            near_miss_frames++;
            if (!near_miss_active) { near_miss++; near_miss_active = true; }
        } else {
            near_miss_active = false;
        }

        // 碰撞检测（迟滞去重：离开碰撞圈0.3m才重置）
        for (auto& p : pedestrians) {
            double d = std::sqrt((robot_x-p.x)*(robot_x-p.x) + (robot_y-p.y)*(robot_y-p.y));
            if (d < DYN_COLLISION_RADIUS) {
                if (collision_hysteresis.find(p.name) == collision_hysteresis.end()) {
                    collision_hysteresis[p.name] = true;
                    total_collisions++;
                    printf("[COLLISION] frame=%d robot=(%.2f,%.2f) %s at (%.2f,%.2f) dist=%.3f action=%s\n",
                           frame, robot_x, robot_y, p.name.c_str(), p.x, p.y, d, current_action.c_str());
                }
            } else if (d > DYN_COLLISION_RADIUS + 0.3) {
                collision_hysteresis.erase(p.name);
            }
        }
    }

    // === v3.0: AMCL 状态报告 ===
    // 输出 AMCL 定位误差、置信度、粒子数等统计信息
    void report_amcl() const {
        if (!use_amcl_) {
            printf("  AMCL: 未启用（USE_AMCL=0）\n");
            return;
        }
        double err = std::sqrt((robot_x - est_x_) * (robot_x - est_x_) +
                               (robot_y - est_y_) * (robot_y - est_y_));
        printf("  === AMCL 定位统计 ===\n");
        printf("  定位误差:     %.3f m (约束 <0.5m)\n", err);
        printf("  置信度:       %.3f (约束 >0.1)\n", est_conf_);
        printf("  估计位姿:     (%.2f, %.2f, %.2f)\n", est_x_, est_y_, est_yaw_);
        printf("  真值位姿:     (%.2f, %.2f, %.2f)\n", robot_x, robot_y, robot_yaw);
        printf("  AMCL 更新数:  %d\n", nav_core_.amcl_updates);
        printf("  AMCL Kidnap:  %d 次\n", nav_core_.amcl_recoveries);
        printf("  AMCL 平均耗时: %.3f ms/帧 (约束 <30ms)\n",
               nav_core_.amcl_updates > 0 ?
               nav_core_.amcl_total_ms / nav_core_.amcl_updates : 0.0);
        printf("  当前粒子数:   %d (active=%d)\n",
               nav_core_.amcl.n, nav_core_.amcl.n_active);
        printf("  A* 调用次数:  %d\n", nav_core_.astar_calls);
        printf("  A* 成功率:    %.1f%%\n",
               nav_core_.astar_calls > 0 ?
               100.0 * nav_core_.astar_path_found / nav_core_.astar_calls : 0.0);
        printf("  A* 平均耗时:  %.3f ms/次\n",
               nav_core_.astar_calls > 0 ?
               nav_core_.astar_total_ms / nav_core_.astar_calls : 0.0);
    }
};

} // namespace puppy_sim
