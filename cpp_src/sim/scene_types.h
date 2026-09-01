// scene_types.h — 场景数据类型定义 (M1.2)
// ================================================================
// 从 simulation.h 抽取 PatrolTarget/Pedestrian 定义，
// 供 simulation.h 和 scene_loader.h 共同引用，打破循环依赖。
// BBox 定义保留在 path_planner.h 中。
#pragma once
#include <string>

namespace puppy_sim {

struct PatrolTarget {
    double x, y, yaw;
    std::string name;
};

struct Pedestrian {
    std::string name;
    double x, y, vx, vy;
    double radius = 0.3;
    double initial_x, initial_y;
    int bounce_counter = 0;

    Pedestrian(std::string n, double x_, double y_, double vx_, double vy_)
        : name(n), x(x_), y(y_), vx(vx_), vy(vy_), initial_x(x_), initial_y(y_) {}
};

} // namespace puppy_sim
