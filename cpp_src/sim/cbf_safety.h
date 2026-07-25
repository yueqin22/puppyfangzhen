// cbf_safety.h — CBF安全过滤器（C++版）
// 从Python cbf_safety.py 转换，完全等效
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

namespace puppy_sim {

// 障碍物（圆形/点）
struct CBFObstacle {
    double cx, cy;  // 中心坐标
    double radius;  // 半径
    CBFObstacle(double x, double y, double r) : cx(x), cy(y), radius(r) {}
};

class CBFSafety {
public:
    double robot_radius;
    double d_safe;
    double d_critical;
    double alpha;
    double max_speed;

    CBFSafety(double robot_radius_ = 0.25, double d_safe_ = 0.35, double d_critical_ = 0.08)
        : robot_radius(robot_radius_), d_safe(d_safe_), d_critical(d_critical_),
          alpha(2.0), max_speed(1.5) {}

    // 计算点到障碍物的有效距离（表面到表面）
    double compute_distance(double x, double y, const CBFObstacle& obs) const {
        double dx = x - obs.cx, dy = y - obs.cy;
        double dist = std::sqrt(dx*dx + dy*dy);
        return dist - robot_radius - obs.radius;
    }

    // CBF值: h = dist - d_safe
    double compute_cbf_value(double x, double y, const CBFObstacle& obs) const {
        return compute_distance(x, y, obs) - d_safe;
    }

    // CBF梯度
    void compute_gradient(double x, double y, const CBFObstacle& obs,
                          double& gh_dx, double& gh_dy) const {
        double dx = x - obs.cx, dy = y - obs.cy;
        double dist = std::sqrt(dx*dx + dy*dy);
        double total = dist + robot_radius + obs.radius;
        if (total < 1e-10) { gh_dx = 0; gh_dy = 0; return; }
        gh_dx = dx / total;
        gh_dy = dy / total;
    }

    // 最近障碍物距离
    double min_distance(double x, double y, const std::vector<CBFObstacle>& obstacles) const {
        if (obstacles.empty()) return 1e9;
        double min_d = 1e9;
        for (auto& obs : obstacles) {
            double d = compute_distance(x, y, obs);
            if (d < min_d) min_d = d;
        }
        return min_d;
    }

    // 风险等级
    int get_risk_level(double x, double y, const std::vector<CBFObstacle>& obstacles) const {
        if (obstacles.empty()) return 0;
        double h_min = 1e9;
        for (auto& obs : obstacles) {
            double h = compute_cbf_value(x, y, obs);
            if (h < h_min) h_min = h;
        }
        if (h_min > d_safe) return 0;
        if (h_min > 0) return 1;
        if (h_min > -d_critical) return 2;
        return 3;
    }

    // QP安全过滤器（投影法）
    // 输入期望速度(u_des_x, u_des_y)，返回安全速度(safe_x, safe_y)
    void safety_filter(double u_des_x, double u_des_y, double x, double y,
                      const std::vector<CBFObstacle>& obstacles,
                      double& safe_x, double& safe_y) const {
        safe_x = u_des_x;
        safe_y = u_des_y;

        if (obstacles.empty()) return;

        std::vector<double> A_x, A_y, b;
        for (auto& obs : obstacles) {
            double h_val = compute_distance(x, y, obs);
            double h_cbf = h_val - d_safe;
            if (h_cbf >= d_safe) continue;  // 约束无效

            double gh_dx, gh_dy;
            compute_gradient(x, y, obs, gh_dx, gh_dy);
            A_x.push_back(gh_dx);
            A_y.push_back(gh_dy);
            b.push_back(-alpha * h_cbf);
        }

        if (A_x.empty()) return;

        // 投影迭代法
        for (int iter = 0; iter < 50; iter++) {
            bool all_ok = true;
            for (size_t i = 0; i < A_x.size(); i++) {
                double dot = A_x[i]*safe_x + A_y[i]*safe_y;
                if (dot < b[i] - 1e-10) {
                    all_ok = false;
                    double a_norm_sq = A_x[i]*A_x[i] + A_y[i]*A_y[i];
                    if (a_norm_sq < 1e-12) continue;
                    double correction = (b[i] - dot) / a_norm_sq;
                    safe_x += correction * A_x[i];
                    safe_y += correction * A_y[i];
                }
            }
            if (all_ok) break;
        }
    }
};

} // namespace puppy_sim
