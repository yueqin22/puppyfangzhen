// scene_loader.h — 从 scene_home.json 加载场景数据 (UE-PLAN M1.2)
// ================================================================
// 提供与 simulation.h 中 build_obstacles/build_patrol_targets/create_pedestrians
// 相同的接口，但数据从 JSON 加载，实现场景配置化。
//
// 用法:
//   auto obstacles = scene_loader::load_obstacles("config/scene_home.json");
//   auto patrols   = scene_loader::load_patrol_targets("config/scene_home.json");
//   auto peds      = scene_loader::load_pedestrians("config/scene_home.json");
#pragma once
#include "mini_json.h"
#include "scene_types.h"  // PatrolTarget, Pedestrian (puppy_sim namespace)
#include "path_planner.h"  // BBox (puppy_sim namespace)
#include <string>
#include <vector>
#include <cstdio>

using puppy_sim::BBox;
using puppy_sim::PatrolTarget;
using puppy_sim::Pedestrian;

namespace scene_loader {

inline std::string find_scene_json() {
    // 按优先级搜索 scene_home.json
    const char* candidates[] = {
        "config/scene_home.json",
        "../config/scene_home.json",
        "../../config/scene_home.json",
        "../../../config/scene_home.json",
        nullptr
    };
    for (int i = 0; candidates[i]; ++i) {
        FILE* f = fopen(candidates[i], "rb");
        if (f) { fclose(f); return candidates[i]; }
    }
    return "";
}

inline std::vector<BBox> load_obstacles(const std::string& path) {
    auto root = mini_json::parse_file(path);
    std::vector<BBox> result;
    const auto& arr = root["obstacles"];
    for (size_t i = 0; i < arr.size(); ++i) {
        const auto& o = arr[i];
        result.push_back(BBox{
            o["xmin"].as_double(),
            o["ymin"].as_double(),
            o["xmax"].as_double(),
            o["ymax"].as_double()
        });
    }
    return result;
}

inline std::vector<PatrolTarget> load_patrol_targets(const std::string& path) {
    auto root = mini_json::parse_file(path);
    std::vector<PatrolTarget> result;
    const auto& arr = root["patrol_targets"];
    for (size_t i = 0; i < arr.size(); ++i) {
        const auto& p = arr[i];
        result.push_back(PatrolTarget{
            p["x"].as_double(),
            p["y"].as_double(),
            p["yaw"].as_double(),
            p["name"].as_string()
        });
    }
    return result;
}

inline std::vector<Pedestrian> load_pedestrians(const std::string& path) {
    auto root = mini_json::parse_file(path);
    std::vector<Pedestrian> result;
    const auto& arr = root["pedestrians"];
    for (size_t i = 0; i < arr.size(); ++i) {
        const auto& p = arr[i];
        Pedestrian ped(
            p["name"].as_string(),
            p["x"].as_double(),
            p["y"].as_double(),
            p["vx"].as_double(),
            p["vy"].as_double()
        );
        ped.radius = p["radius"].as_double(0.3);
        result.push_back(ped);
    }
    return result;
}

// ===== 传感器/AMCL/网格参数 =====
struct SceneConfig {
    // grid
    int grid_w = 100, grid_h = 80;
    double resolution = 0.1;
    double origin_x = -5.0, origin_y = -4.0;

    // sim
    int fps = 30;
    double dt = 1.0 / 30.0;
    double max_speed = 2.0;
    double arrival_radius = 0.35;
    double soft_skip_radius = 0.95;
    int soft_skip_frames = 600;
    int hard_skip_frames = 1200;

    // lidar
    int n_rays = 72;
    double max_range = 8.0;

    // amcl
    int n_particles = 300;
    double sigma_obs = 0.45;
    double z_max = 8.0;
    int n_obs_rays = 72;
    int kld_min = 50;
    int kld_max = 500;

    // robot
    double radius_planning = 0.25;
    double radius_cbf = 0.25;
    double cbf_d_safe = 0.35;
    double cbf_alpha = 2.0;
    double init_x = -1.0, init_y = -3.0, init_yaw = 0.0;
};

inline SceneConfig load_config(const std::string& path) {
    auto root = mini_json::parse_file(path);
    SceneConfig c;

    const auto& g = root["grid"];
    c.grid_w = g["width"].as_int(100);
    c.grid_h = g["height"].as_int(80);
    c.resolution = g["resolution"].as_double(0.1);
    c.origin_x = g["origin_x"].as_double(-5.0);
    c.origin_y = g["origin_y"].as_double(-4.0);

    const auto& s = root["sim"];
    c.fps = s["fps"].as_int(30);
    c.dt = s["dt"].as_double(1.0 / 30.0);
    c.max_speed = s["visual_max_speed"].as_double(2.0);
    c.arrival_radius = s["arrival_radius"].as_double(0.35);
    c.soft_skip_radius = s["soft_skip_radius"].as_double(0.95);
    c.soft_skip_frames = s["soft_skip_frames"].as_int(600);
    c.hard_skip_frames = s["hard_skip_frames"].as_int(1200);

    const auto& l = root["lidar"];
    c.n_rays = l["n_rays"].as_int(72);
    c.max_range = l["max_range"].as_double(8.0);

    const auto& a = root["amcl"];
    c.n_particles = a["n_particles"].as_int(300);
    c.sigma_obs = a["sigma_obs"].as_double(0.45);
    c.z_max = a["z_max"].as_double(8.0);
    c.n_obs_rays = a["n_obs_rays"].as_int(72);
    c.kld_min = a["kld_min"].as_int(50);
    c.kld_max = a["kld_max"].as_int(500);

    const auto& r = root["robot"];
    c.radius_planning = r["radius_planning"].as_double(0.35);
    c.radius_cbf = r["radius_cbf"].as_double(0.25);
    c.cbf_d_safe = r["cbf_d_safe"].as_double(0.35);
    c.cbf_alpha = r["cbf_alpha"].as_double(2.0);
    c.init_x = r["initial_pose"]["x"].as_double(-1.0);
    c.init_y = r["initial_pose"]["y"].as_double(-3.0);
    c.init_yaw = r["initial_pose"]["yaw"].as_double(0.0);

    return c;
}

} // namespace scene_loader
