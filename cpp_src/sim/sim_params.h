// sim_params.h — Simulation parameters loaded from YAML config
// ==============================================================
// Task P1-2.4: Centralize all tunable parameters into one struct,
// loaded from config/sim.yaml. Environment variables act as overrides.
// Priority: command-line > environment variable > YAML > default
//
// Usage:
//   SimParams params;
//   params.load_yaml("config/sim.yaml");
//   params.apply_env_overrides();
//   Simulator sim(params);
#pragma once
#include "mini_yaml.h"
#include <cstdlib>
#include <string>

namespace puppy_sim {

struct SimParams {
    // ---- Navigation switches (were env vars) ----
    bool use_amcl = true;         // USE_AMCL (default: 1)
    bool use_rvo = true;          // USE_RVO (default: 1)
    bool use_imu_fusion = true;   // USE_IMU_FUSION (default: 1)
    int  seed = 0;                // SEED (0 = random)

    // ---- Robot motion ----
    double visual_max_speed = 2.0;    // m/s, max robot speed
    double arrival_radius = 0.35;     // m, goal arrival threshold
    double soft_skip_radius = 0.95;   // m, soft skip threshold
    int    soft_skip_frames = 600;    // frames (~20s at 30fps)
    int    hard_skip_frames = 1200;   // frames (~40s at 30fps)

    // ---- Dynamic obstacle avoidance ----
    double dynamic_clearance = 1.5;       // m, dynamic obstacle detection radius
    double dyn_collision_radius = 0.55;   // m, collision radius for pedestrians

    // ---- Flee behavior (v3.2.18i tuned) ----
    double flee_speed_close = 1.2;    // m/s, flee speed when pedestrian < 0.7m
    double flee_speed_medium = 0.8;   // m/s, flee speed when pedestrian < 1.0m
    double flee_speed_multi = 0.5;    // m/s, flee speed when 2+ pedestrians near
    double flee_speed_near_thresh = 0.7;   // m, threshold for close flee speed
    double flee_speed_med_thresh = 1.0;    // m, threshold for medium flee speed
    int    flee_multi_count = 2;           // pedestrian count for multi-flee

    // ---- Chasing detection (v3.2.18i) ----
    double chasing_dot_threshold = 0.3;        // dot product for chasing detection
    double strong_chasing_dot_threshold = 0.8; // dot product for strong chasing
    double prediction_short_time = 0.3;        // seconds, short-term prediction
    double prediction_long_time = 1.0;         // seconds, long-term prediction

    // ---- AMCL parameters ----
    int    amcl_n_particles = 300;
    double amcl_sigma_obs = 0.45;
    double amcl_z_max = 8.0;
    int    amcl_n_obs_rays = 72;
    int    kld_min = 50;
    int    kld_max = 500;

    // ---- Load from YAML file ----
    void load_yaml(const std::string& path) {
        mini_yaml::Config cfg = mini_yaml::parse_file(path);
        if (cfg.values_.empty()) return;  // file not found or empty

        // Navigation
        use_amcl = cfg.get_bool("navigation.use_amcl", use_amcl);
        use_rvo = cfg.get_bool("navigation.use_rvo", use_rvo);
        use_imu_fusion = cfg.get_bool("navigation.use_imu_fusion", use_imu_fusion);
        seed = cfg.get_int("navigation.seed", seed);

        // Robot motion
        visual_max_speed = cfg.get_double("robot.visual_max_speed", visual_max_speed);
        arrival_radius = cfg.get_double("robot.arrival_radius", arrival_radius);
        soft_skip_radius = cfg.get_double("robot.soft_skip_radius", soft_skip_radius);
        soft_skip_frames = cfg.get_int("robot.soft_skip_frames", soft_skip_frames);
        hard_skip_frames = cfg.get_int("robot.hard_skip_frames", hard_skip_frames);

        // Dynamic obstacles
        dynamic_clearance = cfg.get_double("obstacles.dynamic_clearance", dynamic_clearance);
        dyn_collision_radius = cfg.get_double("obstacles.collision_radius", dyn_collision_radius);

        // Flee behavior
        flee_speed_close = cfg.get_double("flee.speed_close", flee_speed_close);
        flee_speed_medium = cfg.get_double("flee.speed_medium", flee_speed_medium);
        flee_speed_multi = cfg.get_double("flee.speed_multi", flee_speed_multi);
        flee_speed_near_thresh = cfg.get_double("flee.near_thresh", flee_speed_near_thresh);
        flee_speed_med_thresh = cfg.get_double("flee.medium_thresh", flee_speed_med_thresh);
        flee_multi_count = cfg.get_int("flee.multi_count", flee_multi_count);

        // Chasing detection
        chasing_dot_threshold = cfg.get_double("chasing.dot_threshold", chasing_dot_threshold);
        strong_chasing_dot_threshold = cfg.get_double("chasing.strong_dot_threshold", strong_chasing_dot_threshold);
        prediction_short_time = cfg.get_double("chasing.prediction_short", prediction_short_time);
        prediction_long_time = cfg.get_double("chasing.prediction_long", prediction_long_time);

        // AMCL
        amcl_n_particles = cfg.get_int("amcl.n_particles", amcl_n_particles);
        amcl_sigma_obs = cfg.get_double("amcl.sigma_obs", amcl_sigma_obs);
        amcl_z_max = cfg.get_double("amcl.z_max", amcl_z_max);
        amcl_n_obs_rays = cfg.get_int("amcl.n_obs_rays", amcl_n_obs_rays);
        kld_min = cfg.get_int("amcl.kld_min", kld_min);
        kld_max = cfg.get_int("amcl.kld_max", kld_max);
    }

    // ---- Apply environment variable overrides ----
    // Priority: env > YAML > default
    void apply_env_overrides() {
        const char* e;
        if ((e = std::getenv("USE_AMCL")) != nullptr)
            use_amcl = (std::string(e) == "1");
        if ((e = std::getenv("USE_RVO")) != nullptr)
            use_rvo = (std::string(e) == "1");
        if ((e = std::getenv("USE_IMU_FUSION")) != nullptr)
            use_imu_fusion = (std::string(e) == "1");
        if ((e = std::getenv("SEED")) != nullptr)
            seed = std::atoi(e);
    }

    // ---- Print loaded configuration ----
    void print() const {
        printf("[CFG] use_amcl=%d use_rvo=%d use_imu=%d seed=%d\n",
               use_amcl, use_rvo, use_imu_fusion, seed);
        printf("[CFG] max_speed=%.1f arrival=%.2f skip=%d/%d\n",
               visual_max_speed, arrival_radius, soft_skip_frames, hard_skip_frames);
        printf("[CFG] flee: close=%.1f med=%.1f multi=%.1f thresh=%.1f/%.1f\n",
               flee_speed_close, flee_speed_medium, flee_speed_multi,
               flee_speed_near_thresh, flee_speed_med_thresh);
        printf("[CFG] chasing: dot=%.1f/%.1f pred=%.1f/%.1f\n",
               chasing_dot_threshold, strong_chasing_dot_threshold,
               prediction_short_time, prediction_long_time);
        printf("[CFG] amcl: n=%d sigma=%.2f rays=%d kld=%d-%d\n",
               amcl_n_particles, amcl_sigma_obs, amcl_n_obs_rays, kld_min, kld_max);
    }
};

} // namespace puppy_sim
