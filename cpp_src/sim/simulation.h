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
#include "sim_params.h"  // v3.2.18i+: YAML 参数配置化 (任务 P1-2.4)
#include "mini_json.h"   // M1.2: JSON 场景加载
#include "scene_types.h" // M1.2: PatrolTarget/Pedestrian 定义 (打破循环依赖)
#include "scene_loader.h" // M1.2: 从 scene_home.json 加载场景
#include <cmath>
#include <cstdlib>     // std::getenv
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

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
const double DYNAMIC_CLEARANCE = 1.5;  // v3.2.6: 1.2→1.5，更早触发避障（多行人包围防护）
const double DYN_COLLISION_RADIUS = 0.55;

// ===== 巡航点 =====
// PatrolTarget 定义已移至 scene_types.h (M1.2)

inline std::vector<PatrolTarget> build_patrol_targets() {
    // P1-2+P1-3: 巡逻点定义
    //   R19n测试: 拉开间距后A*放宽重试从2→42次,rounds=0→不可接受
    //   回退到原始坐标(已验证R19k 287圈稳定),仅保留删除#19重复点
    return std::vector<PatrolTarget>{
        PatrolTarget{-1.0, -3.0, 0.0, "dock"},
        PatrolTarget{0.0, -2.0, M_PI/2, "living_room"},
        PatrolTarget{-2.75, 0.0, 0.0, "door_living_bed1"},
        PatrolTarget{-3.8, 1.0, 0.0, "bedroom1"},
        PatrolTarget{-3.3, 1.0, 0.0, "door_bed1_study"},
        PatrolTarget{-2.75, 2.3, 0.0, "door_study_y2"},
        PatrolTarget{-4.0, 2.4, 0.0, "study"},
        PatrolTarget{-2.5, 2.4, 0.0, "bedroom2"},
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
        // P1-2 FIX: 删除原#19 return_dock=(-1.0,-3.0),与#0重复→rounds虚计数
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
// Pedestrian 定义已移至 scene_types.h (M1.2)

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
//
// R26b (2026-08-13): 间隙由 0.05m 提升到机器人物理半径 0.20m。
//   修复的不一致：整条链原本有三个互不相同的半径口径 ——
//     0.05m  这里（bridge 安全判定 / ESCAPE 方向扫描 / 直线兜底截断）
//     0.20m  UE 刚体碰撞（PuppyRobotPawn.cpp: ROBOT_RADIUS_CM = 20）
//     0.35m  A* footprint（costmap.h: ROBOT_RADIUS）
//   后果：ESCAPE 的 16 方向扫描按 5cm 判定，认定正北 clear=1.20m 安全，
//     于是把机器人一路推向北墙贴死；UE 在 20cm 处拦住，而 A* 认为 35cm 内
//     全是致命区，规划起点必然嵌墙 → 强拽起点 0.7m → 路径与真身脱节 → 死锁。
//   现在统一为物理半径，使"bridge 认为安全的位置"与"UE 物理允许的位置"一致；
//   余下 0.20m→0.35m 的裕度差由 R26a 的 A* 起点宽容机制吸收。
constexpr double SAFE_CLEARANCE = 0.20;

inline bool is_position_safe(double x, double y, const std::vector<BBox>& obstacles) {
    for (auto& obs : obstacles) {
        if (x >= obs.xmin - SAFE_CLEARANCE && x <= obs.xmax + SAFE_CLEARANCE &&
            y >= obs.ymin - SAFE_CLEARANCE && y <= obs.ymax + SAFE_CLEARANCE)
            return false;
    }
    return true;
}

// 房间判断（真实性指标：房间覆盖）
//
// R26b: 同步 config/scene_home.json v6.0 的 rooms[] 判据。
//   旧实现是上一代 8 房间布局（bedroom1/study/bedroom2/storage/corridor_n/dining），
//   与当前 4 房间公寓完全不符，导致 SUMMARY 里的 rooms 覆盖指标失真。
//   当前布局：内墙 y=0.8~1.0，竖直隔断 x=0.4~0.6 分隔卧室(NW)与卫生间(NE)。
inline std::string get_room_name(double x, double y) {
    if (y > 1.0) {                        // 北半区：卧室 / 卫生间
        if (x < -0.6) return "bedroom";
        if (x >  0.6) return "bathroom";
        return "hallway";                 // 竖直隔断附近
    }
    if (y < 0.8) {                        // 南半区：客厅 / 厨房（开放式）
        if (x > 0.9) return "kitchen";
        return "living_room";
    }
    return "hallway";                     // y∈[0.8,1.0] 内墙带
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
    // v3.2.18i+: 集中管理的可调参数 (任务 P1-2.4)
    //   优先级: 命令行 > 环境变量 > YAML > 默认值
    //   默认构造用 SimParams 默认值 (= v3.2.18i 基线)，保证向后兼容
    SimParams params_;

    // 派生参数 (从 params_ 计算，避免重复运算)
    double visual_max_step_;  // = params_.visual_max_speed / FPS

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
    int stall_events = 0;                   // 卡死事件(episode)计数: stall_timer 由 0 首次进入 >0 计一次
    int persistent_plan_failures_ = 0;   // §25.7 方案B: 仅计"持续(卡死)规划失败"(plan失败且机器人卡死)
    bool in_persistent_plan_fail_ = false; // 同一卡死片段仅计一次, 避免每帧重复计数
    int rounds_completed = 0;
    int goals_reached = 0;                  // P0-2: 正常到达目标计数 (不含跳过的不可达目标)
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

    // v3.2.18: 重规划退避状态（解决 §25.4 问题1：A* 连续失败→单帧 250~390ms 飙升）
    int replan_backoff_ = 0;             // 退避冷却剩余帧数（>0 时本帧不重规划）
    int consecutive_plan_failures_ = 0;  // 连续规划失败计数（达阈值切换目标）

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
    // v3.2.9: AMCL 平均误差追踪（避免只看快照值误判）
    double amcl_err_sum_ = 0.0;   // 累计定位误差
    double amcl_err_max_ = 0.0;   // 最大定位误差
    int amcl_err_samples_ = 0;    // 误差采样数
    // 决策位姿（根据 use_amcl_ 选择真值或估计）
    // step() 中用 dec_x/dec_y/dec_yaw 做决策，物理移动仍用 robot_x/y/yaw
    double dec_x = 0, dec_y = 0, dec_yaw = 0;
    // P1-2.1: 决策位姿速率限制状态
    double prev_dec_x_ratelimit_ = 0, prev_dec_y_ratelimit_ = 0;
    bool dec_initialized_ = false;

    // v3.2.14: IMU+LiDAR 互补滤波融合
    //   原理: IMU高频(30Hz)预测提供短期精确位姿增量，AMCL低频全局校正
    //   门道穿越时 AMCL 置信度低，alpha 降低，更依赖 IMU 预测，抑制 max_err 尖峰
    //   公式: x_fused = alpha * x_amcl + (1-alpha) * x_imu
    //   alpha = 0.3 + 0.5 * conf（conf=0→0.3, conf=1→0.8）
    struct IMUFusion {
        double x = 0, y = 0, yaw = 0;      // 融合位姿
        double vx = 0, vy = 0, vyaw = 0;   // 速度（从里程计）
        bool initialized = false;
        double prev_rx = 0, prev_ry = 0, prev_yaw = 0;  // 上一帧真值（用于计算速度）

        void predict(double dt) {
            if (!initialized) return;
            // IMU 积分: 用速度预测位姿
            x += vx * dt;
            y += vy * dt;
            yaw += vyaw * dt;
            yaw = std::atan2(std::sin(yaw), std::cos(yaw));
        }

        void correct(double amcl_x, double amcl_y, double amcl_yaw, double conf) {
            if (!initialized) {
                x = amcl_x; y = amcl_y; yaw = amcl_yaw;
                initialized = true;
                return;
            }
            // alpha 根据 AMCL 置信度动态调整
            // 高置信度(conf>0.8): alpha=0.7，信任AMCL
            // 低置信度(conf<0.3): alpha=0.3，信任IMU
            double alpha = 0.3 + 0.5 * std::max(0.0, std::min(1.0, conf));

            // v3.2.14a: 限幅 AMCL 跳变 — 防止 AMCL 突然跳变拉偏融合位姿
            //   门道穿越时 AMCL 可能跳变 2m+，即使 alpha=0.3 也会拉偏 0.6m
            //   限制 AMCL 校正量到 MAX_CORRECT (0.3m/帧，与v3.2.11限幅一致)
            //   v3.2.14a 调参: 0.5m→0.3m，seed4 avg_err 0.578m→预期<0.5m
            const double MAX_CORRECT = 0.3;
            double dx_amcl = amcl_x - x;
            double dy_amcl = amcl_y - y;
            double d_amcl = std::sqrt(dx_amcl*dx_amcl + dy_amcl*dy_amcl);
            double scale = 1.0;
            if (d_amcl > MAX_CORRECT) {
                scale = MAX_CORRECT / d_amcl;
            }

            x = x + alpha * dx_amcl * scale;
            y = y + alpha * dy_amcl * scale;

            // yaw 用角度插值
            double dyaw = amcl_yaw - yaw;
            dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
            yaw = std::atan2(std::sin(yaw + alpha * dyaw), std::cos(yaw + alpha * dyaw));
        }

        void update_velocity(double rx, double ry, double ryaw, double dt) {
            if (!initialized) {
                prev_rx = rx; prev_ry = ry; prev_yaw = ryaw;
                return;
            }
            vx = (rx - prev_rx) / dt;
            vy = (ry - prev_ry) / dt;
            double dyaw = ryaw - prev_yaw;
            dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
            vyaw = dyaw / dt;
            prev_rx = rx; prev_ry = ry; prev_yaw = ryaw;
        }
    } imu_fusion_;
    bool use_imu_fusion_ = true;  // 通过 USE_IMU_FUSION 环境变量控制

    // v3.2.9: 门道 landmark 恢复机制
    //   原理: 机器人穿越 y=0/y=2 门道时，粒子云容易分裂（门道两侧墙结构相似），
    //   导致定位误差尖峰（max_err 0.9-1.6m）。检测到穿越时触发小 spread 恢复，
    //   让粒子云在当前估计附近重新收敛。
    //   项目记忆: "Doorway landmarks (y=0 crossings) with 50-frame cooldown
    //   improve AMCL localization accuracy by triggering recover(spread=0.3)"
    int doorway_cooldown_ = 0;      // 冷却计数器（帧）
    double prev_dec_y_ = 0;         // 上一帧决策 y（用于检测穿越）
    static const int DOORWAY_COOLDOWN = 50;  // 50帧冷却（~1.7秒）
    static constexpr double DOORWAY_Y_THRESH = 0.3;  // 门道检测阈值（|y| < 0.3 或 |y-2| < 0.3）

    // 防卡机制
    int stall_timer = 0;
    std::vector<std::pair<double,double>> stall_history;
    bool recovery_mode = false;
    double recovery_dir = 0;
    int recovery_timeout = 0;

    // 碰撞迟滞去重
    std::unordered_map<std::string, bool> collision_hysteresis;

    // v3.2.17: 碰撞诊断环形缓冲区（记录最近30帧，碰撞时打印）
    struct CollisionDiagFrame {
        int frame = 0;
        double rx = 0, ry = 0;        // 真值位姿
        double dx = 0, dy = 0;        // 决策位姿
        double conf = 0;              // AMCL置信度
        double spd = 0;               // 机器人速度
        double min_ped_d = 1e9;       // 最近行人距离
        std::string action;           // 当前动作
        double ped_x = 0, ped_y = 0, ped_vx = 0, ped_vy = 0;  // 最近行人
    };
    std::vector<CollisionDiagFrame> collision_diag_buf_;
    static const int COLLISION_DIAG_SIZE = 30;

    // v3.2.15: 房间访问奖励机制
    //   记录每个房间的访问次数，跳点时优先选择未访问/少访问的房间
    //   目标: 提升房间覆盖率 8→9+，同时保持轮次效率
    std::unordered_map<std::string, int> room_visit_count_;
    // v3.2.15a: 跳过目标历史，避免反复跳到同一个无法到达的目标
    //   当目标被跳过时加入此列表，选择新目标时排除
    //   正常到达目标或完成一轮时清空
    std::vector<size_t> skipped_targets_;
    // v3.2.15b: 房间跳过计数，避免反复尝试不可达房间
    //   某房间被跳过>=3次后标记为不可达，不再选择该房间的巡航点
    //   不在一轮结束时清空（保持不可达记录）
    std::unordered_map<std::string, int> room_skip_count_;
    static const int ROOM_MAX_SKIP = 1;  // 房间最大跳过次数（v3.2.15c: 3→1，避免反复尝试不可达房间）
    // v3.2.15d: 不可达目标集合（简化方案）
    //   目标被跳过一次即加入此集合，永久排除（直到正常到达该目标时移除）
    //   替代复杂的skipped_targets_和room_skip_count_逻辑
    std::unordered_set<size_t> unreachable_targets_;

    Simulator() : cbf(0.25, 0.45, 0.08),
                  rvo(0.25, VISUAL_MAX_SPEED, 1.5, 0.55, 3.0) {
        // v3.2.18i+: 默认构造，从环境变量加载参数 (向后兼容)
        params_.apply_env_overrides();
        init_simulator();
    }

    // v3.2.18i+: 从 SimParams 构造 (任务 P1-2.4)
    //   参数已包含 YAML 加载 + 环境变量覆盖，由调用方完成
    //   注意: rvo time_horizon=1.5s 是固定值(非可调参数), 与 dynamic_clearance
    //   的 1.5m 数值相同但语义不同, 不可混用
    explicit Simulator(const SimParams& params)
        : params_(params),
          cbf(0.25, 0.45, 0.08),
          rvo(0.25, params.visual_max_speed, 1.5,
              params.dyn_collision_radius, 3.0) {
        init_simulator();
    }

    // v3.2.18i+: 仿真器初始化 (构造函数共用逻辑)
    //   将原构造函数的环境变量读取改为从 params_ 读取
    void init_simulator() {
        visual_max_step_ = params_.visual_max_speed / FPS;

        // v3.1 优化: CBF d_safe 0.35→0.45，降低碰撞率
        //   原 0.35m d_safe 在 1 小时测试中产生 23 次碰撞（餐厅 person_3 15次、
        //   客厅 person_4 4次）。0.45m 给机器人更大的安全缓冲区。
        //   项目硬性约束: "CBF算法在狭窄门道与行人导致15% stuck time;
        //   临时降安全阈值0.35m→0.15m" — 仅在 stall_timer>60 时降为 0.15m。
        //   正常状态用 0.45m 提升安全性，stall 状态用 0.15m 脱困（见 step()）。
        patrol_targets = build_patrol_targets();
        obstacles = build_obstacles();
        pedestrians = create_pedestrians();

        // M1.2: 支持 JSON 场景加载 (环境变量 SCENE_JSON 指定路径)
        //   设置 SCENE_JSON=config/scene_home.json 后，场景数据从 JSON 加载
        //   未设置时使用硬编码值（保持向后兼容）
        const char* json_path = std::getenv("SCENE_JSON");
        if (json_path && *json_path) {
            FILE* f = fopen(json_path, "rb");
            if (f) {
                fclose(f);
                try {
                    patrol_targets = scene_loader::load_patrol_targets(json_path);
                    obstacles = scene_loader::load_obstacles(json_path);
                    pedestrians = scene_loader::load_pedestrians(json_path);
                    printf("[SCENE] 从 JSON 加载: %s (%zu障碍/%zu巡逻点/%zu行人)\n",
                           json_path, obstacles.size(), patrol_targets.size(), pedestrians.size());
                } catch (const std::exception& e) {
                    printf("[SCENE] JSON 加载失败(%s), 使用硬编码: %s\n", json_path, e.what());
                    patrol_targets = build_patrol_targets();
                    obstacles = build_obstacles();
                    pedestrians = create_pedestrians();
                }
            }
        }
        cbf.max_speed = params_.visual_max_speed;
        rvo.max_speed = params_.visual_max_speed;
        auto& first = patrol_targets[0];
        robot_x = first.x; robot_y = first.y; robot_yaw = first.yaw;
        prev_robot_x = robot_x; prev_robot_y = robot_y;  // v3.2.3: 真实性指标初始化

        // v3.2.18i+: 从 params_ 读取导航开关 (原为环境变量直接读取)
        use_amcl_ = params_.use_amcl;
        use_rvo_ = params_.use_rvo;
        use_imu_fusion_ = params_.use_imu_fusion;

        if (use_amcl_) {
            // 初始化 NavCoreStack: 构建地图 + AMCL 粒子云
            nav_core_.init_from_obstacles(obstacles);
            // v3.2.3: 支持固定种子（SEED 环境变量，多种子稳定性测试）
            if (params_.seed > 0) {
                nav_core_.set_seed((uint32_t)params_.seed);
                printf("[SEED] 使用固定种子: %u\n", (uint32_t)params_.seed);
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

        // v3.2.14: IMU 融合状态
        if (use_imu_fusion_ && use_amcl_) {
            printf("[IMU] 已启用 IMU+LiDAR 互补滤波融合（USE_IMU_FUSION=1）\n");
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
        skip_count = stall_events = rounds_completed = goals_reached = 0;
        frames_on_target = 0;
        current_path.clear();
        path_replan_counter = 0;
        replan_backoff_ = 0;
        consecutive_plan_failures_ = 0;
        stall_timer = 0;
        persistent_plan_failures_ = 0;
        in_persistent_plan_fail_ = false;
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
        room_visit_count_.clear();  // v3.2.15: 重置房间访问计数
        skipped_targets_.clear();   // v3.2.15a: 重置跳过历史
        room_skip_count_.clear();   // v3.2.15b: 重置房间跳过计数
        unreachable_targets_.clear(); // v3.2.15d: 重置不可达目标集合
        // v3.2.9: 重置 AMCL 误差追踪
        amcl_err_sum_ = 0.0;
        amcl_err_max_ = 0.0;
        amcl_err_samples_ = 0;
        doorway_cooldown_ = 0;
        prev_dec_y_ = 0;
        // P1-2.1: 重置决策位姿速率限制
        dec_initialized_ = false;
        // v3.2.14: 重置 IMU 融合状态
        imu_fusion_.initialized = false;
        imu_fusion_.vx = imu_fusion_.vy = imu_fusion_.vyaw = 0;
    }

    // v3.2.15: 房间访问奖励 - 跳点时选择最优下一个目标
    //   策略: 优先未访问房间 → 少访问房间 → 顺序下一个
    //   过滤 corridor/corridor_n（非真正房间），距离限制 8m 避免跳太远
    //   v3.2.15a: 排除已跳过的目标（避免循环跳点）
    size_t select_target_by_room_reward(double rx, double ry) {
        const double MAX_JUMP_DIST = 8.0;  // 最大跳跃距离(m)
        const int LOW_VISIT_THRESH = 2;    // 少访问阈值

        auto is_real_room = [](const std::string& room) {
            return room != "corridor" && room != "corridor_n";
        };

        auto dist_to = [&](size_t idx) {
            double dx = rx - patrol_targets[idx].x;
            double dy = ry - patrol_targets[idx].y;
            return std::sqrt(dx*dx + dy*dy);
        };

        // v3.2.15d: 排除不可达目标（简化方案）
        auto is_unreachable = [&](size_t idx) {
            return unreachable_targets_.count(idx) > 0;
        };

        // 1. 收集未访问和少访问的真实房间巡航点
        std::vector<size_t> unvisited, low_visit;
        for (size_t i = 0; i < patrol_targets.size(); i++) {
            if (is_unreachable(i)) continue;  // v3.2.15d: 排除不可达目标
            std::string room = get_room_name(patrol_targets[i].x, patrol_targets[i].y);
            if (!is_real_room(room)) continue;
            if (dist_to(i) > MAX_JUMP_DIST) continue;

            int visits = room_visit_count_.count(room) ? room_visit_count_[room] : 0;
            if (visits == 0) {
                unvisited.push_back(i);
            } else if (visits <= LOW_VISIT_THRESH) {
                low_visit.push_back(i);
            }
        }

        // 2. 优先未访问房间中最近的
        if (!unvisited.empty()) {
            size_t best = unvisited[0];
            double best_d = dist_to(best);
            for (size_t i = 1; i < unvisited.size(); i++) {
                double d = dist_to(unvisited[i]);
                if (d < best_d) { best = unvisited[i]; best_d = d; }
            }
            printf("[ROOM_REWARD] frame=%d 跳到未访问房间: %s (idx=%zu, dist=%.2fm)\n",
                   frame, get_room_name(patrol_targets[best].x, patrol_targets[best].y).c_str(), best, best_d);
            return best;
        }

        // 3. 次选少访问房间中最近的
        if (!low_visit.empty()) {
            size_t best = low_visit[0];
            double best_d = dist_to(best);
            for (size_t i = 1; i < low_visit.size(); i++) {
                double d = dist_to(low_visit[i]);
                if (d < best_d) { best = low_visit[i]; best_d = d; }
            }
            return best;
        }

        // 4. 都访问过了或都不可达，按顺序选下一个（排除不可达的）
        for (size_t offset = 1; offset <= patrol_targets.size(); offset++) {
            size_t next = (target_idx + offset) % patrol_targets.size();
            if (!is_unreachable(next)) return next;
        }
        // 全部不可达（极端情况），强制顺序下一个
        return (target_idx + 1) % patrol_targets.size();
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
        if (min_next >= params_.dynamic_clearance) return;

        double goal_ang = std::atan2(ty - ry, tx - rx);
        double attempted = std::sqrt((step_x-rx)*(step_x-rx) + (step_y-ry)*(step_y-ry));
        double fallback = std::max(0.04, std::min(visual_max_step_, attempted));

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
                if (dyn_d >= params_.dynamic_clearance) score += 5.0;
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

            // v3.2.14: IMU+LiDAR 互补滤波融合
            // 1. IMU predict: 用速度积分预测位姿
            // 2. AMCL correct: 用AMCL估计校正，alpha根据置信度动态调整
            if (use_imu_fusion_) {
                imu_fusion_.update_velocity(robot_x, robot_y, robot_yaw, DT);
                imu_fusion_.predict(DT);
                imu_fusion_.correct(est_x_, est_y_, est_yaw_, est_conf_);
                dec_x = imu_fusion_.x;
                dec_y = imu_fusion_.y;
                dec_yaw = imu_fusion_.yaw;
            } else {
                dec_x = est_x_;
                dec_y = est_y_;
                dec_yaw = est_yaw_;
            }

            // P1-2.1: 决策位姿速率限制已移除 — 会导致 seed1 碰撞(避障不及)
            //   改为降低 IMU 融合 alpha（见下方 correct 调用）

            // v3.2.9: 追踪平均/最大定位误差（每 30 帧采样一次降低开销）
            if (amcl_frame_ % 30 == 0) {
                double err = std::sqrt((robot_x - dec_x) * (robot_x - dec_x) +
                                       (robot_y - dec_y) * (robot_y - dec_y));
                amcl_err_sum_ += err;
                if (err > amcl_err_max_) amcl_err_max_ = err;
                amcl_err_samples_++;
                // P1-2.1: 尖峰事件日志 — err>1m 时输出结构化日志
                //   对比 dec_x(融合) vs est_x_(AMCL原始) 的偏差，定位尖峰来源
                if (err > 1.0) {
                    double amcl_err = std::sqrt((robot_x - est_x_) * (robot_x - est_x_) +
                                                (robot_y - est_y_) * (robot_y - est_y_));
                    std::printf("EVT,spike,frame=%d,err=%.3f,amcl_err=%.3f,conf=%.3f,dec=(%.2f,%.2f),est=(%.2f,%.2f),true=(%.2f,%.2f)\n",
                                amcl_frame_, err, amcl_err, est_conf_,
                                dec_x, dec_y, est_x_, est_y_, robot_x, robot_y);
                    std::fflush(stdout);
                }
            }
            // 决策用融合后的位姿（或纯AMCL估计位姿）
            // dec_x/dec_y/dec_yaw 已在上方设置
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
        // v3.2.15d: 到达判断用真实位姿（避免AMCL估计误差导致错误到达判断）
        //   原bug: AMCL估计位姿离目标<0.35m但真实位姿离目标很远，
        //   导致错误判断到达目标，但room_visit_count_未更新，形成循环
        double dist_t = std::sqrt((robot_x-tx)*(robot_x-tx) + (robot_y-ty)*(robot_y-ty));
        frames_on_target++;

        // v3.2.16: 跳点超时（回退到20秒，45秒超时导致门道徘徊→AMCL漂移→碰撞）
        if (dist_t < params_.arrival_radius ||
            (frames_on_target > params_.soft_skip_frames && dist_t < params_.soft_skip_radius) ||
            frames_on_target > params_.hard_skip_frames) {
            bool was_skipped = (frames_on_target > params_.soft_skip_frames);
            if (was_skipped) {
                skip_count++;
                // v3.2.15d: 标记为不可达目标，永久排除（直到正常到达）
                unreachable_targets_.insert(target_idx);
            }
            // v3.2.15a: 不在正常到达时清空跳过历史（避免循环跳点）
            //   只在完成一轮时清空，确保一轮内不重复选择无法到达的目标
            if (was_skipped) {
                target_idx = select_target_by_room_reward(rx, ry);
            } else {
                // v3.2.15d: 正常到达目标，从不可达集合中移除（目标可达）
                unreachable_targets_.erase(target_idx);
                target_idx = (target_idx + 1) % patrol_targets.size();
                goals_reached++;  // P0-2: 正常到达一个目标即计一次
            }
            frames_on_target = 0;
            if (target_idx == 0) {
                rounds_completed++;
                // v3.2.16: 永久排除不可达目标（回退）
                //   本轮排除+下一轮重试导致每轮都尝试study/bedroom2
                //   累计门道徘徊→AMCL粒子云发散→max_err=11.393m→1碰撞
                //   永久排除是安全选择，房间覆盖8/10可接受
            }
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
        double step_x = rx + visual_max_step_ * ddir_x;
        double step_y = ry + visual_max_step_ * ddir_y;
        double ang = std::atan2(ddir_y, ddir_x);
        current_action = "cruise";

        // === A*+CBF ===
        // v3.0: 使用 NavCoreStack 的 A* 规划器（puppy_nav_core 实现）
        // 符合项目硬性约束: "Core computationally intensive modules must be
        // rewritten in C++ with Python scheduling layer"
        // 1. 重规划A*路径
        path_replan_counter++;
        // v3.2.18: 重规划退避（解决 §25.4 问题1：A* 连续失败导致单帧 250~390ms 飙升）
        //   原逻辑：规划失败后 current_path 被清空 → 下一帧 current_path.empty() 为真
        //   → 每帧重新调用完整 A* + 放宽重试 + 静态层回退 + legacy plan_and_smooth，
        //   动态行人封堵/窄通道暂不可达时单帧耗时飙到 250~390ms，P99 退化。
        //   新逻辑：失败后立即进入指数退避冷却(封顶 10 帧)，冷却期间不调用任何 A*，
        //   原地观察/制动；连续失败达 3 次切换目标(SKIP_TARGET)，复用既有跳点逻辑。
        bool plan_due = (path_replan_counter >= path_replan_interval) || current_path.empty();
        if (replan_backoff_ > 0) {
            replan_backoff_--;
            plan_due = false;  // 退避冷却中：本帧不重规划
        }
        if (plan_due) {
            path_replan_counter = 0;
            std::vector<std::pair<double,double>> smoothed;
            if (use_amcl_) {
                // v3.0: 用 puppy_nav_core 的 A* 规划（C++ 实现）
                smoothed = nav_core_.plan(rx, ry, tx, ty, obstacles, frame);
            } else {
                // 消融模式: 用旧版 plan_and_smooth
                smoothed = plan_and_smooth(rx, ry, tx, ty, obstacles);
            }
            bool have_path = (smoothed.size() >= 2);
            if (!have_path && use_amcl_ && smoothed.empty()) {
                // puppy_nav_core A* 找不到路径时，回退到旧版 plan_and_smooth
                // （puppy_nav_core A* 更保守，可能因膨胀拒绝某些路径）
                auto fallback = plan_and_smooth(rx, ry, tx, ty, obstacles);
                if (fallback.size() >= 2) {
                    smoothed = fallback;
                    have_path = true;
                }
            }
            if (have_path) {
                path_follower.set_path(smoothed);
                current_path = smoothed;
                consecutive_plan_failures_ = 0;
                replan_backoff_ = 0;
                in_persistent_plan_fail_ = false;  // §25.7 方案B: 规划成功→结束卡死片段
            } else {
                // 规划失败：指数退避，避免每帧无节制重规划导致单帧耗时飙升
                consecutive_plan_failures_++;
                current_path.clear();
                path_follower.set_path({});
                current_action = "wait_plan";
                replan_backoff_ = std::min(10, 2 + 2 * consecutive_plan_failures_);
                // 连续失败达阈值：切换目标（SKIP_TARGET），复用既有跳点逻辑
                if (consecutive_plan_failures_ >= 3) {
                    unreachable_targets_.insert(target_idx);
                    size_t next_idx = select_target_by_room_reward(rx, ry);
                    target_idx = next_idx;
                    frames_on_target = 0;
                    tx = patrol_targets[target_idx].x;
                    ty = patrol_targets[target_idx].y;
                    consecutive_plan_failures_ = 0;
                    replan_backoff_ = 0;
                    path_replan_counter = path_replan_interval;  // 下一帧为新目标重规划
                }
                // §25.7 方案B(用户确认): 指标精化 —— 仅当规划失败且机器人处于卡死状态
                // (stall_timer>=阈值, 与 escape 行为一致)才计为"持续规划失败"; 瞬时(已恢复)
                // 封堵失败不计。同一卡死片段用 in_persistent_plan_fail_ 去重, 仅计一次。
                // 原始 NavCoreStack::planning_failures 仍保留用于诊断(见 main.cpp 报告)。
                const int kStallPersistentThreshold = 40;  // stall_timer>40 即触发脱困参数(escape)
                if (stall_timer >= kStallPersistentThreshold) {
                    if (!in_persistent_plan_fail_) {
                        persistent_plan_failures_++;
                        in_persistent_plan_fail_ = true;
                    }
                } else {
                    in_persistent_plan_fail_ = false;
                }
            }
        }

        // 2. 路径跟随
        auto follow = path_follower.get_target(rx, ry);
        double fdx = follow.first - rx, fdy = follow.second - ry;
        double fdist = std::sqrt(fdx*fdx + fdy*fdy);
        double des_vx, des_vy;
        if (current_path.size() >= 2 && fdist > 1e-6) {
            des_vx = (fdx / fdist) * params_.visual_max_speed;
            des_vy = (fdy / fdist) * params_.visual_max_speed;
        } else {
            des_vx = 0.0;
            des_vy = 0.0;
            step_x = rx;
            step_y = ry;
            current_action = "wait_plan";
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
            // v3.2.4: 0.15→0.10，降低 stall 敏感度
            //   原 0.15m 把正常减速避让（行人通过、转弯）也计为 stall，
            //   导致 stuck_ratio 15%。0.10m 只检测真正卡住的情况。
            if (spread < 0.10) {
                if (stall_timer == 0) stall_events++;  // 新一次卡死片段(stall episode)开始计数
                stall_timer++;
            } else stall_timer = std::max(0, stall_timer - 1);
        } else {
            stall_timer = std::max(0, stall_timer - 1);
        }

        // CBF参数
        // v3.1 优化: 正常 d_safe 0.35→0.45 (与构造函数一致)
        //   stall_timer>60 时降为 0.15（脱困优先，符合项目硬约束）
        //   recovery_mode 时降为 0.12（紧急脱困）
        // v3.2.5: 60→40，缩短降安全阈值响应时间
        //   原 60 帧 (2秒) 才降 d_safe 到 0.15m，期间机器人持续减速导致 stuck 帧累积。
        //   40 帧 (1.3秒) 提前触发脱困参数，让机器人在被行人短暂阻挡后更快恢复运动。
        //   配合 v3.2.4 的 spread<0.10 检测，整体 stuck_ratio 目标 <10%。
        if (recovery_mode) { cbf.d_safe = 0.12; cbf.alpha = 0.8; }
        else if (stall_timer > 40) { cbf.d_safe = 0.15; cbf.alpha = 0.8; }
        else { cbf.d_safe = 0.45; cbf.alpha = 2.0; }

        // v3.2.2: 启动期保护
        //   前 300 帧 (10秒) 降速到 50%，避免刚出 dock 时与 person_4 碰撞。
        //   实测 frame 22 person_4 碰撞发生在启动初期，此时小车全速冲出 dock，
        //   行人来不及避让。启动期保护让小车慢速启动，给行人避让时间。
        if (frame < 300) {
            cbf.max_speed *= 0.5;
        }

        // v3.1 优化: 近距行人降速策略 (基于预测式行人位置)
        // v3.2.7: 改用预测式距离 — 行人 0.5 秒后的位置 + 当前位置取最小值
        //   原问题: min_dist 基于当前行人位置，但行人有速度，等距离 < 1.0m 时
        //   降速已太晚（行人 0.04 m/frame * 15 frame = 0.6m 移动距离）。
        //   seed=4 碰撞分析: person_3 在 frame 8234 以 0.04m/frame 接近，
        //   当前距离 1.1m (不降速)，15 帧后距离 0.5m 触发碰撞。
        //   修复: 用 0.5 秒后预测位置 + 当前位置取最小值作为 effective_dist。
        //   等效于"看到行人接近时提前降速"，给 CBF/RVO 更多反应时间。
        double predict_horizon = 0.5;  // 0.5 秒预测
        double min_pred_dist = 1e9;
        int n_close_peds = 0;  // v3.2.7: 2m 内行人数（多行人包围检测）
        for (auto& p : pedestrians) {
            double cur_d = std::sqrt((robot_x-p.x)*(robot_x-p.x) + (robot_y-p.y)*(robot_y-p.y));
            // 预测位置: 行人速度 * FPS * horizon
            double pred_x = p.x + p.vx * FPS * predict_horizon;
            double pred_y = p.y + p.vy * FPS * predict_horizon;
            double pred_d = std::sqrt((robot_x-pred_x)*(robot_x-pred_x) + (robot_y-pred_y)*(robot_y-pred_y));
            double eff_d = std::min(cur_d, pred_d);
            if (eff_d < min_pred_dist) min_pred_dist = eff_d;
            if (cur_d < 2.0) n_close_peds++;
        }
        // v3.2.7: 平滑降速 — 用 effective 预测距离做线性插值
        //   eff_d >= 1.8: 全速 2.0
        //   eff_d <= 0.8: 最低 0.4 (而非 0.5，给 CBF 更多裕度)
        //   中间: 线性插值，避免阶梯式跳变
        double max_spd;
        if (min_pred_dist >= 1.8) {
            max_spd = params_.visual_max_speed;
        } else if (min_pred_dist <= 0.8) {
            max_spd = 0.4;
        } else {
            double t = (min_pred_dist - 0.8) / (1.8 - 0.8);  // 0..1
            max_spd = 0.4 + t * (params_.visual_max_speed - 0.4);
        }
        // v3.2.7: 多行人包围场景额外降速（2+ 行人在 2m 内 → 再降 30%）
        //   原因: 多行人接近时 CBF 单独处理每个障碍，可能"躲一个撞另一个"
        //   seed=4 碰撞日志: frame 15234 两个行人同时 dist 0.6/0.8 触发碰撞。
        //   多行人场景下额外降速让 CBF 有更大缓冲区选择安全方向。
        if (n_close_peds >= 2) max_spd *= 0.7;
        cbf.max_speed = max_spd;

        // 层2: 强制脱困
        // v3.2.5: 90→60，提前触发恢复模式
        //   原 90 帧 (3秒) 才进入 recovery_mode，期间机器人完全不动，stuck 帧快速累积。
        //   60 帧 (2秒) 配合 stall_timer>40 的降阈值，形成"减速→脱困参数→强制恢复"的3级响应。
        if (stall_timer > 60 && !recovery_mode) {
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
            double rvx = std::cos(recovery_dir) * params_.visual_max_speed;
            double rvy = std::sin(recovery_dir) * params_.visual_max_speed;
            double sx, sy;
            // v3.2: 脱困时优先使用 CBF (RVO 在密集障碍中可能失败)
            cbf.safety_filter(rvx, rvy, rx, ry, get_cbf_obstacles(), sx, sy);
            double spd = std::sqrt(sx*sx + sy*sy);
            if (spd > 1e-6) {
                double ss = std::min(visual_max_step_, spd / FPS);
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
            // v3.2.6: 使用真值位姿做避障计算（行人位置是真值，避障也应基于真值）
            //   原代码传 rx,ry（估计位姿），当 AMCL 偏差大时 CBF 计算的行人距离不准。
            double sx, sy;
            if (use_rvo_) {
                // v3.2: RVO 避障 — 考虑行人速度的预测式避障
                rvo.max_speed = cbf.max_speed;  // 同步降速策略
                rvo.compute_velocity(robot_x, robot_y, dec_yaw,
                                     last_vx, last_vy,
                                     tx, ty,
                                     get_rvo_obstacles(), sx, sy);
            } else {
                // CBF: 反应式避障
                cbf.safety_filter(des_vx, des_vy, robot_x, robot_y, get_cbf_obstacles(), sx, sy);
            }
            double spd = std::sqrt(sx*sx + sy*sy);

            // === 近距强制flee机制 ===
            // v3.2.7: 改进 flee 触发条件 — 用预测式距离 + 更早触发
            //   原: min_dist < 1.2 && spd < 0.3 (距离已近 + 几乎停下才触发)
            //   新: min_pred_dist < 1.4 && spd < 0.5 (预测接近 + 速度较低就触发)
            //   等效于"看到行人会接近且自己速度不高就先 flee"，避免等到 spd<0.3 才反应
            // v3.2.6: 改进 flee 逻辑 — 基于真值位姿 + 多行人包围处理
            //   原问题: flee 基于 AMCL 估计位姿 (rx,ry)，当估计偏差大时 flee 方向错误。
            //   且 flee 只远离最近行人，多行人同时接近时远离一个靠近另一个。
            //   碰撞日志: seed=9 frame=20223 两个行人同时 dist=0.543 触发碰撞。
            //   修复: flee 方向基于真值位姿 + 远离所有近距行人的合力方向。
            bool need_flee = false;
            double flee_vx = 0, flee_vy = 0;
            double flee_speed = params_.visual_max_speed;  // v3.2.17: flee 速度（近距离降速）
            // v3.2.7: 用预测式 min_dist (与降速逻辑一致)
            double flee_check_dist = min_pred_dist;
            if (flee_check_dist < 1.4 && spd < 0.5) {
                // v3.2.6: 用真值位姿计算 flee（碰撞检测基于真值，flee 也应基于真值）
                double true_min_dist = 1e9;
                // v3.2.18e: 保存最近行人信息用于追逐检测（修复 is_chasing 永远 false 的 bug）
                //   原代码用 d < true_min_dist 重新查找，但 true_min_dist 已是最小值，条件永远 false
                double nearest_ped_vx = 0, nearest_ped_vy = 0;
                double nearest_ped_px = 0, nearest_ped_py = 0;
                for (auto& p : pedestrians) {
                    double d = std::sqrt((robot_x-p.x)*(robot_x-p.x) + (robot_y-p.y)*(robot_y-p.y));
                    if (d < true_min_dist) {
                        true_min_dist = d;
                        nearest_ped_vx = p.vx; nearest_ped_vy = p.vy;
                        nearest_ped_px = p.x; nearest_ped_py = p.y;
                    }
                }
                if (true_min_dist < 1.4) {
                    // v3.2.6: 计算远离所有近距行人的合力方向
                    // v3.2.17: 用预测式行人位置（0.3秒后）计算 away 方向
                    //   原问题: 用当前位置计算 away，当行人移动时 away 方向与行人运动平行，
                    //   导致机器人"平行逃逸"无法拉开距离，最终碰撞 (seed10 frame=28758)
                    //   修复: 用 0.3s 后预测位置计算 away，让 flee 方向避开行人运动趋势
                    //   v3.2.18i+: 参数化预测时间 (任务 P1-2.4)
                    double sum_fx = 0, sum_fy = 0;
                    int near_count = 0;
                    for (auto& p : pedestrians) {
                        double pred_px = p.x + p.vx * FPS * params_.prediction_short_time;
                        double pred_py = p.y + p.vy * FPS * params_.prediction_short_time;
                        double d = std::sqrt((robot_x-pred_px)*(robot_x-pred_px) + (robot_y-pred_py)*(robot_y-pred_py));
                        if (d < 1.5) {
                            double w = (d > 0.01) ? 1.0 / d : 100.0;
                            sum_fx += w * (robot_x - pred_px) / std::max(d, 0.01);
                            sum_fy += w * (robot_y - pred_py) / std::max(d, 0.01);
                            near_count++;
                        }
                    }
                    double away_ang;
                    if (near_count > 0 && (std::abs(sum_fx) > 1e-6 || std::abs(sum_fy) > 1e-6)) {
                        away_ang = std::atan2(sum_fy, sum_fx);
                    } else {
                        double nearest_d = 1e9, npx = 0, npy = 0;
                        for (auto& p : pedestrians) {
                            double d = std::sqrt((robot_x-p.x)*(robot_x-p.x) + (robot_y-p.y)*(robot_y-p.y));
                            if (d < nearest_d) { nearest_d = d; npx = p.x; npy = p.y; }
                        }
                        away_ang = std::atan2(robot_y - npy, robot_x - npx);
                    }
                    // v3.2.18d: 追逐检测 — 行人朝机器人移动时使用1.0s长期预测
                    //   非追逐场景保持v3.2.18b原始0.3s行为（避免seed3/6/10回归）
                    //   v3.2.18d修正: 原用away_ang检测追逐，但多行人场景away_ang偏离实际flee方向
                    //   改为检测行人是否朝机器人移动（vel·(robot-ped)>0.3）
                    //   v3.2.18e: 修复 is_chasing 永远 false 的 bug — 原代码用 d < true_min_dist
                    //   重新查找行人，但 true_min_dist 已是最小值，条件永远 false。
                    //   现在直接使用上面保存的 nearest_ped_vx/vy/px/py
                    // v3.2.18g: 区分强追逐(dot>0.8)和弱追逐(0.3<dot<=0.8)
                    //   强追逐: flee_speed=1.5 + 混合预测（修复seed8平行追逐碰撞）
                    //   弱追逐: 仅flee_speed=1.5（避免seed6/10的1.0s预测回归）
                    //   v3.2.18i+: 参数化追逐阈值 (任务 P1-2.4)
                    double ped_spd = std::sqrt(nearest_ped_vx*nearest_ped_vx + nearest_ped_vy*nearest_ped_vy);
                    bool is_chasing = false;
                    bool is_strong_chasing = false;
                    if (ped_spd > 0.01) {
                        double to_rx = robot_x - nearest_ped_px;
                        double to_ry = robot_y - nearest_ped_py;
                        double to_rd = std::sqrt(to_rx*to_rx + to_ry*to_ry);
                        if (to_rd > 0.01) {
                            double dot = (nearest_ped_vx * to_rx + nearest_ped_vy * to_ry) / (ped_spd * to_rd);
                            if (dot > params_.chasing_dot_threshold) is_chasing = true;
                            if (dot > params_.strong_chasing_dot_threshold) is_strong_chasing = true;
                        }
                    }
                    // v3.2.17: 近距离 flee 速度
                    //   多行人场景: 0.5m/s 精确机动（v3.2.17d 1.5m/s 导致 seed2/6 回归）
                    //   单行人场景: 1.2m/s 快速逃脱（行人速度1.2m/s，需匹配）
                    //   v3.2.18c: 保持 v3.2.17c 速度策略，仅修复 fallback 方向选择
                    //   （追逐速度提升到2.0会导致AMCL长期退化 avg_err 0.084→0.417m）
                    //   v3.2.18i: 回退flee_speed=1.5（v3.2.18d/h中1.5导致seed1/5碰撞）
                    //   仅用混合预测解决方向问题，速度保持1.2匹配行人
                    //   v3.2.18i+: 参数化 (任务 P1-2.4)
                    if (near_count >= params_.flee_multi_count) flee_speed = params_.flee_speed_multi;
                    else if (true_min_dist < params_.flee_speed_near_thresh) flee_speed = params_.flee_speed_close;
                    else if (true_min_dist < params_.flee_speed_med_thresh) flee_speed = params_.flee_speed_medium;
                    else flee_speed = params_.visual_max_speed;
                    // v3.2.17: 扩展 flee 搜索方向 9→16，更密集覆盖
                    std::vector<double> flee_offsets = {0, M_PI/8, -M_PI/8, M_PI/4, -M_PI/4,
                                                       M_PI/2, -M_PI/2, 3*M_PI/4, -3*M_PI/4,
                                                       M_PI/6, -M_PI/6, M_PI/3, -M_PI/3,
                                                       5*M_PI/6, -5*M_PI/6, M_PI};
                    double best_score = -1e9;
                    double best_fallback_score = -1e9;  // v3.2.17: 备选最佳（包围时用）
                    double best_fallback_vx = 0, best_fallback_vy = 0;
                    for (double off : flee_offsets) {
                        double cand_ang = away_ang + off;
                        // v3.2.6: 基于真值位姿检查安全性
                        double fx = robot_x + visual_max_step_ * std::cos(cand_ang);
                        double fy = robot_y + visual_max_step_ * std::sin(cand_ang);
                        if (!is_position_safe(fx, fy, obstacles)) continue;
                        // v3.2.17: 评分用预测式行人位置（0.3秒后）+ 当前位置取最小值
                        //   v3.2.18i+: 参数化预测时间 (任务 P1-2.4)
                        double dyn_d = 1e9;
                        for (auto& p : pedestrians) {
                            double cur_d = std::sqrt((fx-p.x)*(fx-p.x) + (fy-p.y)*(fy-p.y));
                            double pred_px = p.x + p.vx * FPS * params_.prediction_short_time;
                            double pred_py = p.y + p.vy * FPS * params_.prediction_short_time;
                            double pred_d = std::sqrt((fx-pred_px)*(fx-pred_px) + (fy-pred_py)*(fy-pred_py));
                            double eff_d = std::min(cur_d, pred_d);
                            if (eff_d < dyn_d) dyn_d = eff_d;
                        }
                        // v3.2.7: 拒绝 flee 后仍距行人 < 0.7m 的方向
                        if (dyn_d >= 0.7) {
                            double score = dyn_d;
                            if (score > best_score) {
                                best_score = score;
                                flee_vx = std::cos(cand_ang) * flee_speed;
                                flee_vy = std::sin(cand_ang) * flee_speed;
                                need_flee = true;
                            }
                        }
                        // v3.2.18h: fallback 评分 — 仅单行人强追逐用混合预测
                        //   v3.2.18e: 所有追逐场景用混合预测 → seed6/10回归
                        //   v3.2.18f: 完全不用混合预测 → seed8碰撞回归
                        //   v3.2.18g: 强追逐(dot>0.8)用混合预测 → seed8修复但seed3碰撞
                        //   seed3碰撞根因: 多行人夹击(person1+x,person2-x)时混合预测选不佳方向
                        //   v3.2.18h: 仅near_count<2且强追逐时用混合预测
                        //   seed8: 单行人强追逐→混合预测→垂直逃离→0碰撞
                        //   seed3: 多行人夹击→保持0.3s→避免混合预测方向错误
                        //   v3.2.18i+: 参数化预测时间 (任务 P1-2.4)
                        double fb_score;
                        if (is_strong_chasing && near_count < 2) {
                            double long_d = 1e9;
                            for (auto& p : pedestrians) {
                                double lppx = p.x + p.vx * FPS * params_.prediction_long_time;
                                double lppy = p.y + p.vy * FPS * params_.prediction_long_time;
                                double lrfx = robot_x + std::cos(cand_ang) * flee_speed * params_.prediction_long_time;
                                double lrfy = robot_y + std::sin(cand_ang) * flee_speed * params_.prediction_long_time;
                                double ld = std::sqrt((lrfx-lppx)*(lrfx-lppx) + (lrfy-lppy)*(lrfy-lppy));
                                if (ld < long_d) long_d = ld;
                            }
                            fb_score = 0.5 * dyn_d + 0.5 * long_d;
                        } else {
                            fb_score = dyn_d;
                        }
                        if (fb_score > best_fallback_score) {
                            best_fallback_score = fb_score;
                            best_fallback_vx = std::cos(cand_ang) * flee_speed;
                            best_fallback_vy = std::sin(cand_ang) * flee_speed;
                        }
                    }
                    // v3.2.17: 被包围时不再停止，选择最佳可用方向
                    //   原逻辑: 所有方向<0.7m→停止→被移动行人撞击
                    //   新逻辑: 选择 1.0s 预测距离最大的方向逃离，移动比静止更安全
                    if (!need_flee && best_fallback_score > 0) {
                        flee_vx = best_fallback_vx;
                        flee_vy = best_fallback_vy;
                        need_flee = true;
                    }
                }
            }

            if (need_flee) {
                // v3.2.17: flee 时用临时目标引导 RVO，而非绕过 RVO
                //   原问题1: RVO compute_velocity 忽略 flee 方向，总是朝原目标计算
                //   原问题2: 完全绕过 RVO 导致多行人场景无法考虑所有行人避障
                //   修复: 设置临时目标 = 机器人 + flee方向 * 2.0m，让 RVO 朝 flee 方向
                //   计算速度，同时 RVO 自然考虑所有行人的速度障碍
                double flee_ang = std::atan2(flee_vy, flee_vx);
                double temp_tx = rx + std::cos(flee_ang) * 2.0;
                double temp_ty = ry + std::sin(flee_ang) * 2.0;
                double fsx, fsy;
                if (use_rvo_) {
                    // v3.2.17: flee 时用 flee_speed 覆盖 RVO max_speed（不取 min）
                    //   原取 min(orig_max, flee_speed) 导致 cbf.max_speed=0.4 时机器人
                    //   只能以0.4m/s逃离，被1.2m/s行人追上。flee 必须快于行人才能逃脱
                    double orig_max = rvo.max_speed;
                    rvo.max_speed = flee_speed;
                    rvo.compute_velocity(rx, ry, dec_yaw,
                                         flee_vx, flee_vy, temp_tx, temp_ty,
                                         get_rvo_obstacles(), fsx, fsy);
                    rvo.max_speed = orig_max;  // 恢复（下帧会重新设置）
                } else {
                    cbf.safety_filter(flee_vx, flee_vy, rx, ry, get_cbf_obstacles(), fsx, fsy);
                }
                double fspd = std::sqrt(fsx*fsx + fsy*fsy);
                if (fspd > 1e-6) {
                    double ss = std::min(visual_max_step_, fspd / FPS);
                    double va = std::atan2(fsy, fsx);
                    step_x = rx + ss * std::cos(va);
                    step_y = ry + ss * std::sin(va);
                    ang = va;
                }
                current_action = "flee";
            } else if (spd > 1e-6) {
                double ss = std::min(visual_max_step_, spd / FPS);
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
        // v3.2.5: 关键修复 — 同时检查估计位姿和真值位姿的安全性
        //   原代码只检查 step_x, step_y（基于估计位姿 rx,ry）。
        //   当 AMCL 漂移到墙内时，所有替代方向都从 rx,ry 出发也会被判为不安全，
        //   导致机器人完全卡住（stall_timer 持续增长，stuck_ratio 28%）。
        //   修复: 先检查估计位姿帧的 step，若不安全则尝试从真值位姿出发的方向，
        //   最后将 delta 转换回估计帧保持一致性。
        {
            double true_step_x = robot_x + (step_x - rx);
            double true_step_y = robot_y + (step_y - ry);
            bool est_unsafe = !is_position_safe(step_x, step_y, obstacles);
            bool true_unsafe = !is_position_safe(true_step_x, true_step_y, obstacles);
            if (est_unsafe || true_unsafe) {
                double goal_ang = std::atan2(ty - ry, tx - rx);
                double attempted = std::sqrt((step_x-rx)*(step_x-rx) + (step_y-ry)*(step_y-ry));
                double fallback = std::max(0.04, std::min(visual_max_step_, attempted));
                std::vector<double> offsets = {0, M_PI/12, -M_PI/12, M_PI/6, -M_PI/6,
                                    M_PI/4, -M_PI/4, M_PI/3, -M_PI/3,
                                    M_PI/2, -M_PI/2, 2*M_PI/3, -2*M_PI/3, M_PI};
                double best_score = 1e9;
                bool found = false;
                // v3.2.5: 从真值位姿出发尝试方向（真值位姿不会在墙内）
                for (double base : std::vector<double>{goal_ang, ang}) {
                    for (double off : offsets) {
                        double alt_ang = base + off;
                        double ax = robot_x + fallback * std::cos(alt_ang);
                        double ay = robot_y + fallback * std::sin(alt_ang);
                        if (!is_position_safe(ax, ay, obstacles)) continue;
                        double tdist = std::sqrt((ax-tx)*(ax-tx) + (ay-ty)*(ay-ty));
                        double tp = std::abs(std::atan2(std::sin(alt_ang-goal_ang), std::cos(alt_ang-goal_ang)));
                        double score = tdist + 0.15 * tp;
                        if (score < best_score) {
                            best_score = score;
                            // 转换回估计帧: step = rx + (true_pos - robot)
                            step_x = rx + (ax - robot_x);
                            step_y = ry + (ay - robot_y);
                            ang = alt_ang;
                            found = true;
                        }
                    }
                }
                if (!found) {
                    // 所有方向都不安全，保持原位（基于真值位姿）
                    step_x = rx; step_y = ry;
                }
            }
        }

        // 动态安全圈（脱困时跳过）
        // v3.2.5: 同步 stall_timer>40 阈值
        // v3.2.5: 使用真值位姿 robot_x, robot_y 做动态避障（行人位置是真值）
        if (recovery_mode) {
            // 信任CBF
        } else if (stall_timer > 40) {
            double min_next = 1e9;
            double true_step_x = robot_x + (step_x - rx);
            double true_step_y = robot_y + (step_y - ry);
            for (auto& p : pedestrians) {
                double d = std::sqrt((true_step_x-p.x)*(true_step_x-p.x) + (true_step_y-p.y)*(true_step_y-p.y));
                if (d < min_next) min_next = d;
            }
            if (min_next < 0.25) {
                apply_dynamic_clearance(robot_x, robot_y, true_step_x, true_step_y, ang, tx, ty);
                // 转换回估计帧
                step_x = rx + (true_step_x - robot_x);
                step_y = ry + (true_step_y - robot_y);
            }
        } else {
            double true_step_x = robot_x + (step_x - rx);
            double true_step_y = robot_y + (step_y - ry);
            apply_dynamic_clearance(robot_x, robot_y, true_step_x, true_step_y, ang, tx, ty);
            // 转换回估计帧
            step_x = rx + (true_step_x - robot_x);
            step_y = ry + (true_step_y - robot_y);
        }

        // 更新位置
        // v3.2.5: 最终安全防护 — 如果新位置不安全（穿墙/嵌入障碍），回退到原位置
        //   之前的 wall check 已尝试替代方向，但若所有方向都失败会保留默认 step（朝目标）。
        //   此处作为最后防线，确保机器人永远不会嵌入障碍物。
        //
        // v3.2.5 关键修复: step_x/step_y 是基于 AMCL 估计位姿 (rx, ry) 计算的，
        //   但物理移动必须基于真值位姿 (robot_x, robot_y)。当 AMCL 估计漂移到墙内时，
        //   机器人会跟着穿墙。修复方法：将 step 的 DELTA 应用到真值位姿，
        //   并用真值位姿做碰撞检测。
        double delta_x = step_x - rx;  // 决策计算的位移
        double delta_y = step_y - ry;
        double true_step_x = robot_x + delta_x;  // 应用到真值位姿
        double true_step_y = robot_y + delta_y;
        if (!is_position_safe(true_step_x, true_step_y, obstacles)) {
            // 尝试更小的步长（基于真值位姿）
            double small_step = 0.02;
            double smag = std::sqrt(delta_x*delta_x + delta_y*delta_y);
            if (smag > 1e-6) {
                double ux = delta_x / smag, uy = delta_y / smag;
                double alt_x = robot_x + small_step * ux;
                double alt_y = robot_y + small_step * uy;
                if (is_position_safe(alt_x, alt_y, obstacles)) {
                    true_step_x = alt_x; true_step_y = alt_y;
                } else {
                    true_step_x = robot_x; true_step_y = robot_y;  // 完全回退
                }
            } else {
                true_step_x = robot_x; true_step_y = robot_y;
            }
        }
        double prev_true_x = robot_x, prev_true_y = robot_y;  // 保存真值位姿用于速度计算
        robot_x = true_step_x;
        robot_y = true_step_y;
        robot_yaw = ang;
        // 速度基于实际位移（安全防护可能改变了 step）
        last_vx = (robot_x - prev_true_x) / DT;
        last_vy = (robot_y - prev_true_y) / DT;

        // v3.2.3: 真实性指标更新
        double ddx = robot_x - prev_robot_x;
        double ddy = robot_y - prev_robot_y;
        total_distance += std::sqrt(ddx*ddx + ddy*ddy);
        prev_robot_x = robot_x;
        prev_robot_y = robot_y;
        double spd = std::sqrt(last_vx*last_vx + last_vy*last_vy);
        if (spd > 0.02) { active_frames++; active_speed_sum += spd; }
        // v3.2.5: 同步 >40 阈值，统计真正接近卡住的帧
        if (stall_timer > 40) stuck_frames++;
        std::string room = get_room_name(robot_x, robot_y);
        bool room_found = false;
        for (auto& r : rooms_visited) { if (r == room) { room_found = true; break; } }
        if (!room_found) rooms_visited.push_back(room);
        room_visit_count_[room]++;  // v3.2.15: 更新房间访问计数

        // 近距事件（用移动后的位置）
        double post_min_dist = min_pedestrian_dist(robot_x, robot_y);
        if (post_min_dist < 0.8) {
            near_miss_frames++;
            if (!near_miss_active) { near_miss++; near_miss_active = true; }
        } else {
            near_miss_active = false;
        }

        // v3.2.17: 记录碰撞诊断帧（环形缓冲区）
        {
            CollisionDiagFrame df;
            df.frame = frame;
            df.rx = robot_x; df.ry = robot_y;
            df.dx = dec_x; df.dy = dec_y;
            df.conf = est_conf_;
            df.spd = std::sqrt(last_vx*last_vx + last_vy*last_vy);
            df.action = current_action;
            // 找最近行人
            double nearest_d = 1e9;
            for (auto& p : pedestrians) {
                double d = std::sqrt((robot_x-p.x)*(robot_x-p.x) + (robot_y-p.y)*(robot_y-p.y));
                if (d < nearest_d) {
                    nearest_d = d;
                    df.ped_x = p.x; df.ped_y = p.y;
                    df.ped_vx = p.vx; df.ped_vy = p.vy;
                }
            }
            df.min_ped_d = nearest_d;
            collision_diag_buf_.push_back(df);
            if ((int)collision_diag_buf_.size() > COLLISION_DIAG_SIZE) {
                collision_diag_buf_.erase(collision_diag_buf_.begin());
            }
        }

        // 碰撞检测（迟滞去重：离开碰撞圈0.3m才重置）
        for (auto& p : pedestrians) {
            double d = std::sqrt((robot_x-p.x)*(robot_x-p.x) + (robot_y-p.y)*(robot_y-p.y));
            if (d < params_.dyn_collision_radius) {
                if (collision_hysteresis.find(p.name) == collision_hysteresis.end()) {
                    collision_hysteresis[p.name] = true;
                    total_collisions++;
                    printf("[COLLISION] frame=%d robot=(%.2f,%.2f) %s at (%.2f,%.2f) dist=%.3f action=%s\n",
                           frame, robot_x, robot_y, p.name.c_str(), p.x, p.y, d, current_action.c_str());
                    // v3.2.17: 打印碰撞前30帧诊断信息
                    printf("[COLLISION_DIAG] === 碰撞前 %d 帧回溯 ===\n", (int)collision_diag_buf_.size());
                    for (auto& df : collision_diag_buf_) {
                        double ped_rel_x = df.ped_x - df.rx;
                        double ped_rel_y = df.ped_y - df.ry;
                        double closing = -(ped_rel_x * df.ped_vx + ped_rel_y * df.ped_vy) /
                                         std::max(0.01, std::sqrt(ped_rel_x*ped_rel_x + ped_rel_y*ped_rel_y));
                        printf("[COLLISION_DIAG] f=%d true=(%.2f,%.2f) dec=(%.2f,%.2f) conf=%.2f spd=%.2f min_d=%.2f ped=(%.2f,%.2f) pv=(%.3f,%.3f) close=%.3f act=%s\n",
                               df.frame, df.rx, df.ry, df.dx, df.dy, df.conf, df.spd, df.min_ped_d,
                               df.ped_x, df.ped_y, df.ped_vx, df.ped_vy, closing, df.action.c_str());
                    }
                    printf("[COLLISION_DIAG] === 回溯结束 ===\n");
                    collision_diag_buf_.clear();  // 清空避免重复打印
                }
            } else if (d > params_.dyn_collision_radius + 0.3) {
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
        // v3.2.14: 快照误差用融合后的决策位姿（dec_x/dec_y）
        double err = std::sqrt((robot_x - dec_x) * (robot_x - dec_x) +
                               (robot_y - dec_y) * (robot_y - dec_y));
        // v3.2.9: 输出平均/最大误差，避免只看快照误判
        double avg_err = (amcl_err_samples_ > 0) ? amcl_err_sum_ / amcl_err_samples_ : 0.0;
        printf("  === AMCL 定位统计 ===\n");
        printf("  定位误差(快照): %.3f m (约束 <0.5m)\n", err);
        printf("  定位误差(平均): %.3f m (最大 %.3f m, %d 采样)\n", avg_err, amcl_err_max_, amcl_err_samples_);
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
        // v3.2.5: A* 失败原因诊断
        int total_fails = nav_core_.astar_calls - nav_core_.astar_path_found;
        if (total_fails > 0) {
            printf("  A* 失败诊断 (%d 次):\n", total_fails);
            printf("    起点嵌墙:    %d\n", nav_core_.planner.fail_no_nearest_start);
            printf("    终点嵌墙:    %d\n", nav_core_.planner.fail_no_nearest_goal);
            printf("    超时(>100ms): %d\n", nav_core_.planner.fail_timeout);
            printf("    节点超限:    %d\n", nav_core_.planner.fail_max_nodes);
            printf("    真无路径:    %d\n", nav_core_.planner.fail_no_path);
            printf("    最后失败: start=(%.2f,%.2f) goal=(%.2f,%.2f) reason=%d\n",
                   nav_core_.planner.last_fail_sx, nav_core_.planner.last_fail_sy,
                   nav_core_.planner.last_fail_gx, nav_core_.planner.last_fail_gy,
                   nav_core_.planner.last_fail_reason);
        }
    }
};

} // namespace puppy_sim
