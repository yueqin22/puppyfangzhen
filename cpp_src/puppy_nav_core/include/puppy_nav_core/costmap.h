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
//
// R29d（2026-08-15）：0.35 → 0.42。这不是"实际机器人半径"，而是**规划用安全
//   footprint 半径** = 机身半径 + 定位误差裕度。原值 0.35 只给了 0.05m 裕度，
//   远小于实测定位误差，导致规划器认为安全的位姿实际已使机身嵌入家具。
//
// 实证（3600 帧 UE 验收，250 个"指令 >=0.20m/s 但零位移"的真卡住帧）：
//   真实净空中位数 0.245m  <  机身半径 0.30m  → 98% 的帧机身实际嵌入障碍
//   估计净空中位数 0.406m                     → 规划器仅 6% 的帧察觉贴障碍
//   该处定位误差 中位 0.159m / 最大 0.346m；最近障碍 86% 为 coffee_table
//   结论：裕度(0.05m) < 定位误差(0.16m)，安全边界从原理上守不住。
//
// 为何取 0.42 而非更大：0.42 是保持 5/5 巡逻目标可达的几何上限
//   （footprint 0.35→2184 自由格 5/5；0.40/0.42→1957 格 5/5；0.45→1796 格 4/5 丢目标）。
//
// 残留风险（场景缺陷，非算法缺陷，见 docs/）：0.42 只能给 0.12m 裕度，仍低于
//   实测误差中位 0.159m。若要覆盖规范 §16.4 允许的 0.25m 动态定位误差，需
//   0.30+0.25=0.55m 净空，而该场景在 0.55m 下仅 4/5 目标可达 —— 即
//   scene_home.json 的 coffee_table 过道在规范允许的定位误差预算下不具备
//   零接触可导航性，需移动家具而非继续调参。
constexpr double ROBOT_RADIUS = 0.35;       // 规划安全 footprint：机身0.30 + 保守裕度0.05
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
// R29d 根因修复（2026-08-15）：INFLATION_RADIUS 0.15 → 0.35
//
// 缺陷：INFLATION_CELLS = (int)(0.15/0.1) = 1，膨胀核只有 3x3，即障碍外仅 1 格
//   (0.10m) 带惩罚代价，再往外一律为 0。于是 A* 认为"贴着家具边缘 0.10m 走"
//   与"走空间中线"几乎等价（无梯度可言），大量路径贴边生成；而机身半径 0.30m，
//   贴边行走必然刮碰。
// 实证（3600 帧 UE 验收 trace）：72 次接触中 70 次发生在质心距家具 AABB
//   0.29~0.42m（中位 0.31m），0 次在 AABB 内部 —— 与机身半径 0.30m 精确吻合，
//   证明碰撞源于路径贴边而非定位穿模。
// 修复：膨胀半径取一个机身半径 0.35m（≥PHYSICAL_RADIUS 0.30m），梯度跨 3 格：
//   距障碍 0.1m → 代价 99.7 → A* 步代价 9.28x 自由格
//   距障碍 0.2m → 代价 77.6 → 步代价 6.02x
//   距障碍 0.3m → 代价 60.5 → 步代价 4.05x
//   距障碍 >0.35m → 代价 0   → 步代价 1.0x
//   （步代价公式见 astar_planner.cpp: 1.0 + (c/PLAN_BLOCKED)^2 * 12.0）
// 为何不会重现"0.55m 堵死门道"：因 INSCRIBED_RADIUS=0，衰减区最大代价 99.7
//   仍 < PLAN_BLOCKED=120，故门道**不会被硬阻断**，只是代价升高；窄处仍可通行，
//   宽处 A* 会优先走中线。这正是当年 0.55m 失败的原因（那时膨胀区落入
//   INSCRIBED=128 ≥ 阈值 → 硬阻断），机制已不同。
constexpr double INFLATION_RADIUS = 0.35;   // 膨胀距离 = 一个机身半径（仅代价梯度，不硬阻断）
constexpr int INFLATION_CELLS = static_cast<int>(INFLATION_RADIUS / GRID_RESOLUTION);
constexpr int INSCRIBED_CELLS = static_cast<int>(INSCRIBED_RADIUS / GRID_RESOLUTION);
constexpr double COST_SCALE_FACTOR = 2.5;   // 越大衰减越陡

// R26a (2026-08-13): 起点宽容半径 —— 解决"合法位姿被判致命"死锁
//
// 问题：is_footprint_traversable() 用 ROBOT_RADIUS(0.35m) 圆盘扫 LETHAL 格，
//   即机器人质心距障碍 <0.35m 一律判不可通行；但 UE 物理刚体半径为 0.30m
//   (PuppyRobotPawn.cpp:71 SetCapsuleRadius(30.0f)，UE 单位 cm)，机器人可以合法
//   停在 0.30~0.35m。一旦落入这 5cm 宽的"合法但 A* 视为致命"带，规划起点即嵌墙，
//   旧逻辑用 find_nearest_free 把起点强拽 avg 0.714m / max 0.906m（161 次 >0.5m），
//   于是路径起点偏离机器人真身 0.7m，跟随器朝错方向走 → 零速 → 触发 ESCAPE
//   → 被盲推进墙体（实测 A* start=(-0.44,3.48) 落在北墙 y[3.3,3.5] 内）→ 死锁。
//
// 解法（ROS global_planner 同类做法）：不搬起点，而是在起点邻域放宽 footprint。
//   距规划起点 START_RELIEF_RADIUS 内用 PHYSICAL_RADIUS 判定，让机器人能从
//   真实位置"走出"膨胀带；超出该邻域立即恢复 ROBOT_RADIUS 严格判定，安全裕度不变。
// R29d（2026-08-15）曾把本值从 0.20 改为 0.30，理由是"UE 碰撞体为
//   SetCapsuleRadius(30.0f)"。R33（2026-08-14 复核）证实该理由是错的：
//   UE 侧确实存在 ROBOT_RADIUS_CM 常量（PuppyRobotPawn.cpp:1969 = 20.0f），
//   且碰撞胶囊为 PuppyRobotPawn.cpp:136 `SetCapsuleRadius(20.0f)`，
//   故真实机身半径 = 0.20m，不是 0.30m。
//   R29d 引为证据的"70/72 次接触发生在质心距 AABB 0.29~0.42m"其实不是机身外缘
//   命中，而是旧碰撞判据把 AABB 四向各膨胀 R 做轴对齐方形判定（= 与正方形取
//   Minkowski 和）在凸角处的假阳性：方形判据在角点的触及半径为 R·√2 = 0.283m，
//   而非 R = 0.20m。R33 已将该判据改为精确点到 AABB 欧氏距离（与圆盘取
//   Minkowski 和），凸角假阳性消除。
// 本值保留 0.30 而不下调到 0.20：PHYSICAL_RADIUS 仅用于起点宽容判定，取值大于
//   真实机身 = 判定更严格 = 更保守，下调反而会放宽。改动它需重跑起点死锁回归。
constexpr double PHYSICAL_RADIUS = 0.30;      // 保守值；真实机身半径为 0.20m（见上）
constexpr double START_RELIEF_RADIUS = 0.60;  // 起点宽容作用范围（仅影响起点附近）

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
