// rvo_safety.h — RVO 互惠速度障碍法 (C++ 版)
// =================================================
// 从 Python rvo_avoidance.py 转换，完全等效。
//
// 算法 (van den Berg et al. ICRA 2008):
//   1. 对每个动态障碍物 B 构造 RVO 锥:
//      - 轴线: 机器人指向 B 的方向
//      - 半角: asin(combined_radius / dist)
//      - 锥顶点: (v_A + v_B) / 2 (RVO 关键改进, VO 锥顶点在 v_B)
//   2. 在速度空间采样找最优安全速度:
//      - 优先选择 preferred_vel (朝目标方向)
//      - 跳过任何 RVO 锥内的速度
//      - 若所有方向都被 RVO 覆盖，选最大边缘的方向
//   3. 速度平滑 (EMA 滤波)
//
// 与 CBF 的区别:
//   - CBF 是反应式 (距离 < d_safe 才过滤), RVO 是预测式 (基于 TTC)
//   - CBF 假设障碍物静止, RVO 考虑障碍物速度
//   - RVO 假设对方也会避让 (互惠), 适合多人场景
//
// 接口兼容 CBFSafety:
//   - safety_filter(u_des_x, u_des_y, x, y, obstacles, &safe_x, &safe_y)
//   - 但需要障碍物速度信息 (通过 RVOObstacle 扩展)
//
// 项目硬性约束:
//   - "动态障碍避障能力不足, 门道区域行人避碰问题突出"
//   - 项目记忆: RVO 是 CBF 的可行替代 (498 vs 3 碰撞, 3.4min vs 5.6min/小时)
#pragma once
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <algorithm>
#include <cmath>
#include <vector>

namespace puppy_sim {

// RVO 障碍物 (带速度)
struct RVOObstacle {
    double cx, cy;       // 中心位置
    double vx, vy;       // 速度 (m/s)
    double radius;       // 半径
    RVOObstacle(double x, double y, double vx_, double vy_, double r)
        : cx(x), cy(y), vx(vx_), vy(vy_), radius(r) {}
};

class RVOSafety {
public:
    double robot_radius;
    double max_speed;
    double time_horizon;     // 预测时间范围 (秒)
    double safe_distance;    // 近距保护触发距离
    double neighbor_dist;    // 只考虑此距离内的行人
    int num_samples;         // 速度空间采样数

    // 平滑: 上一帧速度
    double last_vx = 0.0;
    double last_vy = 0.0;

    // 统计
    int compute_count = 0;
    int cone_count = 0;
    int all_in_rvo_count = 0;
    int near_miss_count = 0;

    RVOSafety(double robot_radius_ = 0.25,
              double max_speed_ = 2.0,
              double time_horizon_ = 1.5,
              double safe_distance_ = 0.55,
              double neighbor_dist_ = 3.0)
        : robot_radius(robot_radius_), max_speed(max_speed_),
          time_horizon(time_horizon_), safe_distance(safe_distance_),
          neighbor_dist(neighbor_dist_), num_samples(36) {}

    // 兼容 CBFSafety 接口 (假设障碍物静止)
    // 注意: 若使用此接口，RVO 退化为 VO (无互惠)
    // 推荐: 使用 compute_velocity 传入障碍物速度
    void safety_filter(double u_des_x, double u_des_y,
                       double x, double y,
                       const std::vector<CBFObstacle>& obstacles,
                       double& safe_x, double& safe_y) const {
        // 转换 CBFObstacle -> RVOObstacle (假设静止)
        std::vector<RVOObstacle> rvo_obs;
        rvo_obs.reserve(obstacles.size());
        for (const auto& obs : obstacles) {
            rvo_obs.emplace_back(obs.cx, obs.cy, 0.0, 0.0, obs.radius);
        }
        // 调用核心算法
        double tx = x + u_des_x, ty = y + u_des_y;  // 伪目标
        compute_velocity(x, y, 0.0, u_des_x, u_des_y, tx, ty, rvo_obs,
                         safe_x, safe_y);
    }

    // 核心 RVO 计算
    // 输入: 机器人位姿+速度, 目标位置, 障碍物列表 (含速度)
    // 输出: 安全速度 (safe_vx, safe_vy)
    void compute_velocity(double robot_x, double robot_y, double robot_yaw,
                          double robot_vx, double robot_vy,
                          double target_x, double target_y,
                          const std::vector<RVOObstacle>& obstacles,
                          double& out_vx, double& out_vy) const {
        // 1. 计算 preferred velocity (朝目标方向, 距离近时减速)
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double dist_to_target = std::sqrt(dx * dx + dy * dy);
        double pref_vx, pref_vy;
        if (dist_to_target > 1e-6) {
            double speed_scale = std::min(1.0, dist_to_target / 0.5);
            pref_vx = (dx / dist_to_target) * max_speed * speed_scale;
            pref_vy = (dy / dist_to_target) * max_speed * speed_scale;
        } else {
            pref_vx = max_speed;
            pref_vy = 0.0;
        }

        // 2. 对每个障碍物构造 RVO 锥
        struct RVOCones {
            double apex_vx, apex_vy;
            double axis_angle;
            double half_angle;
        };
        std::vector<RVOCones> cones;
        double min_obs_dist = 1e9;
        double ttc_min = 1e9;

        for (const auto& obs : obstacles) {
            double rel_x = obs.cx - robot_x;
            double rel_y = obs.cy - robot_y;
            double dist = std::sqrt(rel_x * rel_x + rel_y * rel_y);
            if (dist < min_obs_dist) min_obs_dist = dist;
            if (dist < 1e-6 || dist > neighbor_dist) continue;

            double combined_r = robot_radius + obs.radius;
            double half_angle;
            if (dist <= combined_r) {
                half_angle = M_PI;
            } else {
                half_angle = std::asin(std::min(1.0, combined_r / dist));
            }
            double axis_angle = std::atan2(rel_y, rel_x);

            // RVO: 锥顶点在 (v_A + v_B) / 2
            double apex_vx = (robot_vx + obs.vx) / 2.0;
            double apex_vy = (robot_vy + obs.vy) / 2.0;

            cones.push_back({apex_vx, apex_vy, axis_angle, half_angle});

            // TTC 计算
            double rel_vel_x = robot_vx - obs.vx;
            double rel_vel_y = robot_vy - obs.vy;
            double rel_speed_along = (dist > 1e-6) ?
                (rel_vel_x * rel_x + rel_vel_y * rel_y) / dist : 0.0;
            if (rel_speed_along > 0 && dist > combined_r) {
                double t = (dist - combined_r) / rel_speed_along;
                if (t < ttc_min && t < time_horizon) ttc_min = t;
            }
        }

        // 3. 在速度空间采样找最优安全速度
        struct Candidate { double angle; };
        std::vector<Candidate> candidates;
        candidates.reserve(num_samples + 2);
        for (int i = 0; i < num_samples; ++i) {
            candidates.push_back({2.0 * M_PI * i / num_samples});
        }
        // preferred 方向
        candidates.push_back({std::atan2(pref_vy, pref_vx)});
        // 当前速度方向
        if (std::abs(robot_vx) > 1e-6 || std::abs(robot_vy) > 1e-6) {
            candidates.push_back({std::atan2(robot_vy, robot_vx)});
        }

        double best_vx = 0, best_vy = 0;
        double best_cost = 1e9;
        bool found = false;

        for (const auto& c : candidates) {
            double vel_x = std::cos(c.angle) * max_speed;
            double vel_y = std::sin(c.angle) * max_speed;

            // 检查是否在任何 RVO 锥内
            bool in_any_rvo = false;
            for (const auto& cone : cones) {
                double rel_v_x = vel_x - cone.apex_vx;
                double rel_v_y = vel_y - cone.apex_vy;
                double rel_v_norm = std::sqrt(rel_v_x * rel_v_x + rel_v_y * rel_v_y);
                if (rel_v_norm < 1e-6) continue;

                double rel_angle = std::atan2(rel_v_y, rel_v_x);
                double angle_diff = rel_angle - cone.axis_angle;
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                if (std::abs(angle_diff) < cone.half_angle) {
                    in_any_rvo = true;
                    break;
                }
            }
            if (in_any_rvo) continue;

            // 代价: 与 preferred_vel 的距离
            double cost = (vel_x - pref_vx) * (vel_x - pref_vx) +
                         (vel_y - pref_vy) * (vel_y - pref_vy);
            if (cost < best_cost) {
                best_cost = cost;
                best_vx = vel_x;
                best_vy = vel_y;
                found = true;
            }
        }

        // 4. 所有方向都在 RVO 内 — 选最大边缘方向
        if (!found && !cones.empty()) {
            double best_margin = -1e9;
            double best_angle = 0.0;
            for (int i = 0; i < 72; ++i) {
                double angle = 2.0 * M_PI * i / 72;
                double vel_x = std::cos(angle) * max_speed * 0.5;
                double vel_y = std::sin(angle) * max_speed * 0.5;

                double min_margin = 1e9;
                for (const auto& cone : cones) {
                    double rel_v_x = vel_x - cone.apex_vx;
                    double rel_v_y = vel_y - cone.apex_vy;
                    double rel_v_norm = std::sqrt(rel_v_x * rel_v_x + rel_v_y * rel_v_y);
                    double margin;
                    if (rel_v_norm < 1e-6) {
                        margin = 0.0;
                    } else {
                        double rel_angle = std::atan2(rel_v_y, rel_v_x);
                        double angle_diff = rel_angle - cone.axis_angle;
                        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
                        margin = std::abs(angle_diff) - cone.half_angle;
                    }
                    if (margin < min_margin) min_margin = margin;
                }
                if (min_margin > best_margin) {
                    best_margin = min_margin;
                    best_angle = angle;
                }
            }
            best_vx = std::cos(best_angle) * max_speed * 0.3;
            best_vy = std::sin(best_angle) * max_speed * 0.3;
        }

        if (!found && cones.empty()) {
            best_vx = pref_vx;
            best_vy = pref_vy;
        }

        // 5. 速度平滑 (EMA 滤波, alpha=0.6)
        double alpha = 0.6;
        double smooth_vx = alpha * best_vx + (1.0 - alpha) * last_vx;
        double smooth_vy = alpha * best_vy + (1.0 - alpha) * last_vy;
        double speed = std::sqrt(smooth_vx * smooth_vx + smooth_vy * smooth_vy);
        if (speed > max_speed) {
            smooth_vx = smooth_vx * (max_speed / speed);
            smooth_vy = smooth_vy * (max_speed / speed);
        }

        out_vx = smooth_vx;
        out_vy = smooth_vy;
    }

    // 最近障碍物距离
    double min_distance(double x, double y,
                       const std::vector<RVOObstacle>& obstacles) const {
        if (obstacles.empty()) return 1e9;
        double min_d = 1e9;
        for (const auto& obs : obstacles) {
            double dx = x - obs.cx, dy = y - obs.cy;
            double d = std::sqrt(dx * dx + dy * dy) - robot_radius - obs.radius;
            if (d < min_d) min_d = d;
        }
        return min_d;
    }

    // 风险等级 (0=安全, 1=近距, 2=危险, 3=碰撞)
    int get_risk_level(double x, double y,
                       const std::vector<RVOObstacle>& obstacles) const {
        if (obstacles.empty()) return 0;
        double min_d = 1e9;
        for (const auto& obs : obstacles) {
            double dx = x - obs.cx, dy = y - obs.cy;
            double d = std::sqrt(dx * dx + dy * dy) - robot_radius - obs.radius;
            if (d < min_d) min_d = d;
        }
        if (min_d > safe_distance) return 0;
        if (min_d > 0) return 1;
        if (min_d > -0.08) return 2;
        return 3;
    }
};

}  // namespace puppy_sim
