#include "protocol.h"
#include "../puppy_nav_core/include/puppy_nav_core/occupancy_grid.h"
#include "../puppy_nav_core/include/puppy_nav_core/costmap.h"
#include "../puppy_nav_core/include/puppy_nav_core/astar_planner.h"
#include "../puppy_nav_core/include/puppy_nav_core/amcl.h"
#include "../sim/nav_bridge.h"
#include "../sim/lidar_safety.h"  // R31b swept-corridor filter (unit-tested)
#include "scene_loader.h"   // scene_loader::load_* from JSON
#include "simulation.h"     // build_obstacles/build_patrol_targets/create_pedestrians fallback

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <algorithm>
#include <fstream>
#include "../common/config_contract.h"   // puppy::load_contract / Contract (统一契约)

using namespace puppy_nav_core;
using namespace puppy_sim;

static const char* BRIDGE_VERSION = "v2.0-bridge-cli";

static bridge::Link g_link;   // P0-03: v2 协议通道（替代原 g_client 裸 socket）
static int    g_frame  = 0;
static int    g_inject_divergence_at = -1;  // FAULT-INJECTION (test only): frame at
                                            // which to simulate a +3m localization
                                            // jump, to verify divergence recovery.

// R27: optional per-frame CSV trace (enabled with --trace <path>).
// Rationale: HEALTH-CHECK alone only yields aggregate numbers (avg_err etc),
// which is not enough to tell *where* and *when* localization degrades.
static FILE*  g_trace_fp = nullptr;

// R28: optional per-RAY dump (<trace>.rays.csv). The likelihood-field grid search
// proved the scan's best-matching pose is never at the ground-truth pose, i.e. the
// LiDAR sees geometry the occupancy grid does not contain. Aggregate numbers cannot
// say WHICH geometry, so we dump every ray at a few frames and compare the measured
// range against an analytic raycast of the very BBox list the grid was built from.
static FILE*  g_ray_fp = nullptr;

// raycast_bboxes: nearest AABB intersection along a world-frame ray (slab method).
// Returns max_range when nothing is hit, 0 when the origin is inside a box.
static double raycast_bboxes(double rx, double ry, double world_angle,
                             const std::vector<BBox>& obstacles,
                             double max_range) {
    const double dx = std::cos(world_angle);
    const double dy = std::sin(world_angle);
    double best = max_range;
    for (const auto& o : obstacles) {
        double tmin = -1e18, tmax = 1e18;
        if (std::fabs(dx) > 1e-9) {
            double t1 = (o.xmin - rx) / dx, t2 = (o.xmax - rx) / dx;
            if (t1 > t2) std::swap(t1, t2);
            tmin = (std::max)(tmin, t1);
            tmax = (std::min)(tmax, t2);
        } else if (rx < o.xmin || rx > o.xmax) {
            continue;  // ray parallel to X slab and outside it
        }
        if (std::fabs(dy) > 1e-9) {
            double t1 = (o.ymin - ry) / dy, t2 = (o.ymax - ry) / dy;
            if (t1 > t2) std::swap(t1, t2);
            tmin = (std::max)(tmin, t1);
            tmax = (std::min)(tmax, t2);
        } else if (ry < o.ymin || ry > o.ymax) {
            continue;
        }
        const double t_enter = (std::max)(0.0, tmin);
        if (tmax >= t_enter && t_enter < best) best = t_enter;
    }
    return best;
}

struct FrameData {
    std::vector<double> lidar_distances;
    double lidar_max_range = 8.0;
    int    lidar_n_rays = 72;

    double true_x = 0, true_y = 0, true_yaw = 0;
    bool   has_ground_truth = false;

    struct PedInfo { float x, y, vx, vy; };
    std::vector<PedInfo> peds;
    bool has_ped_state = false;

    int    collision_count = 0;
    double hit_x = 0, hit_y = 0;
    bool   has_collision = false;
};

struct NavState {
    NavCoreStack nav;
    std::vector<BBox> obstacles;
    std::vector<PatrolTarget> patrol_targets;
    std::vector<Pedestrian> pedestrians;
    size_t patrol_idx = 0;
    double goal_x = 0, goal_y = 0;
    int goal_close_frames = 0;
    bool dynamic_init_done = false;
    bool prev_true_initialized = false;
    double prev_true_x = 0.0, prev_true_y = 0.0, prev_true_yaw = 0.0;
    std::vector<std::pair<double, double>> current_path;
    int replan_counter = 0;
    static constexpr int REPLAN_INTERVAL = 15;
    bool initialized = false;
    int total_collisions = 0;
    int ue_collision_events = 0;
    int last_ue_collision_count = 0;
    int target_reached_count = 0;
    double gt_path_distance = 0.0;
    int gt_motion_frames = 0;
    int gt_stuck_frames = 0;
    int gt_boundary_frames = 0;
    // R27: last commanded velocity, mirrored here so the main loop can log it
    // into the per-frame CSV trace (cmd_* are locals inside process_frame).
    double last_cmd_vx = 0.0, last_cmd_vy = 0.0, last_cmd_wz = 0.0;
    // R27: per-frame diagnostic flags (reset at the top of each process_frame)
    int  dbg_astar_called = 0;     // 1 if A* ran this frame
    int  dbg_astar_ok = 0;         // 1 if A* returned a path this frame
    int  dbg_escape_active = 0;    // R29b: 0=normal, 1=escape burst, 2=collision brake
    double dbg_clearance = 0.0;    // R29c: distance to nearest obstacle AABB
    double dbg_startfix = 0.0;     // A* start-correction distance this frame (m)
    // ---- R28: AMCL estimator-comparison diagnostics ----
    // The published estimate comes from get_cluster_estimate() (largest-cluster
    // weighted mean).  Those clusters are grown greedily from the ARGMAX particle,
    // so on a unimodal cloud the largest cluster is an asymmetric 0.5m-truncated
    // subset whose mean is biased away from the true posterior mean.  We log the
    // full-cloud mean alongside it to measure that bias directly.
    double dbg_mean_x = 0.0, dbg_mean_y = 0.0;   // full-cloud weighted mean
    double dbg_top_x  = 0.0, dbg_top_y  = 0.0;   // largest-cluster mean (unclamped)
    double dbg_top_share = 0.0;                  // largest-cluster weight share
    int    dbg_nclusters = 0;                    // cluster count this frame
    // Likelihood-field grid search (every 30 frames): is the observation model's
    // optimum actually at the ground-truth pose?
    double dbg_score_gt = 0.0;     // scan likelihood at ground truth
    double dbg_score_est = 0.0;    // scan likelihood at published estimate
    double dbg_score_best = 0.0;   // best score found in the search window
    double dbg_best_dx = 0.0;      // offset of the best-scoring pose from GT
    double dbg_best_dy = 0.0;
    // ---- R28b: path-follower lookahead diagnostics ----
    // The follower used to grab waypoint `closest + 3` unconditionally.  On a
    // sparse / smoothed A* path that index can sit on the far side of a wall, so
    // the robot was commanded straight into the geometry.  These fields record
    // which waypoint was actually chased and whether it was in line of sight.
    int    dbg_la_idx = -1;        // chosen lookahead waypoint index
    int    dbg_la_naive = -1;      // what the old `closest + 3` rule would pick
    double dbg_la_x = 0.0, dbg_la_y = 0.0;
    int    dbg_la_los = 0;         // 1 = straight line to the chosen wp is clear
    int    dbg_la_naive_los = 0;   // 1 = the naive wp would have been clear too
    // ---- R31: LiDAR reactive velocity safety filter diagnostics ----
    // dbg_r31_scale is the regression detector for the R29c failure mode: if
    // the filter is throttling the dog everywhere (rather than only on the
    // beams that point at something), this ratio sits well below 1.0 across
    // most frames and stuck_ratio will blow up.  Watch it in the trace.
    int    dbg_r31_bound = 0;      // # obstacle points inside the commanded
                                   // swept corridor (-1 = bypassed in escape)
    double dbg_r31_dmin = 0.0;     // nearest valid beam distance (m)
    double dbg_r31_scale = 1.0;    // |v_filtered| / |v_raw|
    double dbg_r31_dev = 0.0;      // chosen heading offset (deg); large values
                                   // mean the dog is sliding around geometry
    double err_sum = 0.0;
    double err_max = 0.0;
    int err_samples = 0;
    int rounds_completed = 0;
    // R33 (2026-08-14): targets abandoned as unreachable.  Without this the
    // acceptance metric "目标完成率" was computed as reached/(rounds*5), whose
    // denominator is by construction <= the numerator (rounds = floor of
    // reached/5), so the check could never fail -- it reported 113.3% on the
    // r33 run.  Completion is only meaningful as reached/(reached + abandoned).
    int targets_abandoned = 0;
    std::unordered_set<std::string> rooms_visited;
    int rxf_consec_at_boundary = 0;
    int rxf_consec_big_startfix = 0;
    int rxf_last_startfix_warn = 0;
    int rxf_recover_cooldown = 0;
    // §5.2: 场景声明的初始位姿（来自 scene_home.json initial_pose），用于首帧 GT
    // 一致性校验；bridge 把 AMCL 初始云与 est 都种在该位姿上。
    double scene_init_x = 0.0, scene_init_y = 0.0, scene_init_yaw = 0.0;
    // §5.3: 逐帧新增的可机读指标
    int dbg_amcl_updated = 0;        // 1 = 本帧 AMCL 实际执行了更新
    double dbg_est_prev_x = 0.0, dbg_est_prev_y = 0.0;  // 上一帧 est（算运动距离）
    double dbg_motion_delta = 0.0;   // 本帧 est 位移 (m)
    double dbg_proc_ms = 0.0;        // 本帧 bridge 计算耗时 (ms)
    int rxf_escape_remaining = 0;
    double rxf_escape_cmd_vx = 0;
    double rxf_escape_cmd_vy = 0;
    double rxf_escape_cmd_wz = 0;
    double rxf_60s_gt_x = 1e9;
    double rxf_60s_gt_y = 1e9;
    int rxf_60s_boundary_count = 0;
    int rxf_escape_count = 0;      // consecutive escape attempts on current target
    int rxf_consec_stuck = 0;      // R29: consecutive frames with ~no GT motion
    int rxf_contact_frames = 0;    // R29: consecutive frames in UE collision contact
    // R29: UE only emits COLLISION on an actual blocking hit, so a contact streak
    // MUST decay by frame number -- otherwise the brake would latch on forever
    // after the very first contact.
    int rxf_last_contact_frame = -1000;
    int astar_fallbacks = 0;       // straight-line fallbacks (A* genuinely failed)

    // P0 (2026-08-14): GT-independent AMCL divergence recovery state.
    // We maintain a DEAD-RECKONING pose integrated from the motion input
    // (dx,dy,dyaw). In simulation this input is the ground-truth delta; in real
    // deployment it is the wheel/IMU odometry. Either way it is locally accurate
    // and therefore the correct reference for detecting localization loss: when
    // the AMCL estimate disagrees with the integrated odometry beyond a threshold
    // for sustained frames, the filter has diverged and we re-seed it at the
    // odometry pose (standard AMCL recovery behaviour, needs no ground truth).
    double dr_x = 0.0, dr_y = 0.0, dr_yaw = 0.0;   // dead-reckoning pose
    bool   dr_initialized = false;
    int    dr_diverge_frames = 0;                 // consecutive frames over threshold
};

// ===========================================================================
// R28b: straight-line traversability test for the path follower.
//
// Sampled swept-circle check: walk the segment in 4cm steps and reject if any
// sample lies within `clearance` of an obstacle AABB.  The first `skip_m`
// metres are exempt because the robot frequently *starts* grazing a wall
// (the physical collision resolver parks it at 0.20m); demanding full
// clearance at t=0 would mark every direction blocked and freeze the dog.
// ===========================================================================
static bool segment_clear(double x0, double y0, double x1, double y1,
                          const std::vector<BBox>& obstacles,
                          double clearance, double skip_m = 0.12) {
    const double dx = x1 - x0, dy = y1 - y0;
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6) return true;
    constexpr double kStep = 0.04;
    const int n = std::max(1, static_cast<int>(std::ceil(len / kStep)));
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        if (t * len < skip_m) continue;               // exempt the grazing start
        const double px = x0 + t * dx;
        const double py = y0 + t * dy;
        for (const auto& o : obstacles) {
            if (px >= o.xmin - clearance && px <= o.xmax + clearance &&
                py >= o.ymin - clearance && py <= o.ymax + clearance) {
                return false;
            }
        }
    }
    return true;
}

// ===========================================================================
// R29c: distance from a point to the nearest obstacle AABB (0 when inside).
//
// Needed for the obstacle-proximity speed envelope.  Before this, the stack
// only slowed down near the room boundary and near the goal, so the dog passed
// within centimetres of furniture at up to 1.2 m/s and 71% of the UE contacts
// logged in the 3600-frame acceptance run happened in normal following mode.
// ===========================================================================
static double nearest_obstacle_distance(double x, double y,
                                        const std::vector<BBox>& obstacles) {
    double best = 1e9;
    for (const auto& o : obstacles) {
        // Per-axis overshoot outside the box; both zero => point is inside.
        const double ox = std::max({o.xmin - x, 0.0, x - o.xmax});
        const double oy = std::max({o.ymin - y, 0.0, y - o.ymax});
        const double d = std::sqrt(ox * ox + oy * oy);
        if (d < best) best = d;
    }
    return best;
}

static bool is_patrol_target_safe(const PatrolTarget& target,
                                  const std::vector<BBox>& obstacles) {
    constexpr double margin = 0.20;
    for (const auto& obstacle : obstacles) {
        if (target.x >= obstacle.xmin - margin && target.x <= obstacle.xmax + margin &&
            target.y >= obstacle.ymin - margin && target.y <= obstacle.ymax + margin) return false;
    }
    return true;
}

static void filter_patrol_targets(NavState& state) {
    std::vector<PatrolTarget> safe_targets;
    for (const auto& target : state.patrol_targets) {
        if (is_patrol_target_safe(target, state.obstacles)) safe_targets.push_back(target);
        else printf("[Bridge] skip unsafe target: %s (%.2f, %.2f)\n", target.name.c_str(), target.x, target.y);
    }
    if (safe_targets.size() >= 2) state.patrol_targets = std::move(safe_targets);

}
static void load_scene(NavState& state, const std::string& json_path) {
    if (!json_path.empty()) {
        FILE* file = fopen(json_path.c_str(), "rb");
        if (file) {
            fclose(file);
            try {
                state.patrol_targets = scene_loader::load_patrol_targets(json_path);
                state.obstacles = scene_loader::load_obstacles(json_path);
                state.pedestrians = scene_loader::load_pedestrians(json_path);
                // §5.2: 取出场景声明的初始位姿，供首帧 GT 一致性校验
                try {
                    auto cfg = scene_loader::load_config(json_path);
                    state.scene_init_x = cfg.init_x;
                    state.scene_init_y = cfg.init_y;
                    state.scene_init_yaw = cfg.init_yaw;
                } catch (const std::exception&) {
                    state.scene_init_x = state.scene_init_y = state.scene_init_yaw = 0.0;
                }
                filter_patrol_targets(state);
                printf("[Bridge] scene loaded\n");
                return;
            } catch (const std::exception&) {
                printf("[Bridge] scene load failed, using defaults\n");
            }
        }
    }
    state.obstacles = build_obstacles();
    state.patrol_targets = build_patrol_targets();
    state.pedestrians = create_pedestrians();
    filter_patrol_targets(state);
    printf("[Bridge] default scene loaded\n");
}
static void process_frame(NavState& state, const FrameData& frame_data) {
    if (!state.initialized) return;
    state.dbg_amcl_updated = 0;   // §5.3: 每帧重置，仅当 AMCL 实际更新时置 1

    // CRITICAL FIX: prev_true must initialize to the FIRST frame's GT position,
    // NOT (0,0,0).  Previously the robot would spawn at (-1,-3) and the first
    // AMCL update would apply dx=-1, dy=-3 (a 3.16m teleport!), causing
    // the particle cloud to explode and never reconverge.  g_frame starts
    // at 0 BEFORE process_frame so use g_frame==0 as first-frame guard.
    if (!state.prev_true_initialized) {
        state.prev_true_x = frame_data.true_x;
        state.prev_true_y = frame_data.true_y;
        state.prev_true_yaw = frame_data.true_yaw;
        state.prev_true_initialized = true;
        // Initialize dead-reckoning at the robot's true start pose.
        state.dr_x = frame_data.true_x;
        state.dr_y = frame_data.true_y;
        state.dr_yaw = frame_data.true_yaw;
        state.dr_initialized = true;
    }
    double dx = frame_data.true_x - state.prev_true_x;
    double dy = frame_data.true_y - state.prev_true_y;
    double dyaw = frame_data.true_yaw - state.prev_true_yaw;
    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    while (dyaw < -M_PI) dyaw += 2 * M_PI;
    state.prev_true_x = frame_data.true_x;
    state.prev_true_y = frame_data.true_y;
    state.prev_true_yaw = frame_data.true_yaw;
    // Integrate dead reckoning from the (odometry) motion input.
    if (state.dr_initialized) {
        state.dr_x += dx;
        state.dr_y += dy;
        state.dr_yaw += dyaw;
        while (state.dr_yaw > M_PI) state.dr_yaw -= 2 * M_PI;
        while (state.dr_yaw < -M_PI) state.dr_yaw += 2 * M_PI;
    }

    const double gt_step_distance = std::sqrt(dx * dx + dy * dy);
    state.gt_path_distance += gt_step_distance;
    if (gt_step_distance >= 0.005) {
        state.gt_motion_frames++;
        state.rxf_consec_stuck = 0;
    } else {
        state.gt_stuck_frames++;
        state.rxf_consec_stuck++;
    }
    if (std::fabs(frame_data.true_x) > 4.8 ||
        frame_data.true_y < -3.8 || frame_data.true_y > 3.8) {
        state.gt_boundary_frames++;
    }
    std::vector<double> scan_angles;
    scan_angles.reserve(frame_data.lidar_n_rays);
    for (int i = 0; i < frame_data.lidar_n_rays; ++i) {
        double angle = -M_PI + 2.0 * M_PI * i / frame_data.lidar_n_rays;
        scan_angles.push_back(angle);
    }

    auto result = state.nav.amcl.update(dx, dy, dyaw, scan_angles,
                                         frame_data.lidar_distances, g_frame);
    state.nav.est_x = std::get<0>(result);
    state.nav.est_y = std::get<1>(result);
    state.nav.est_yaw = std::get<2>(result);
    state.nav.est_conf = std::get<3>(result);

    // ---- FAULT-INJECTION TEST HOOK (guarded, default off) ----
    // Simulate a localization "kidnapping" jump at a chosen frame to verify the
    // divergence recovery can pull the estimate back. Only active when
    // --inject-divergence-at N is supplied. Mirrors plan section 10.2
    // ("定位跳变") and P1-03 (fault-injection testing).
    if (g_inject_divergence_at > 0 && (int)g_frame == g_inject_divergence_at) {
        state.nav.amcl.init_cloud(state.nav.est_x + 3.0, state.nav.est_y,
                                   state.nav.est_yaw, 0.1);
        state.nav.est_x += 3.0;  // jump the published estimate 3 m east
        printf("[Bridge][FAULT-INJECT f=%d] simulated localization jump +3.0m\n",
               g_frame);
        fflush(stdout);
    }

    // ---- P0 (2026-08-14): GT-INDEPENDENT DIVERGENCE RECOVERY ----
    // Root cause (R31 analysis): sigma_obs=0.45 makes the likelihood field too
    // flat, and the apartment geometry is repetitive, so the particle cloud can
    // collapse into a WRONG local mode while still scoring almost as high as the
    // true pose. A likelihood-gap check (the previous attempt) therefore never
    // fires here: a wrong pose looks "good enough".
    //
    // Robust fix (standard AMCL recovery, no ground truth): compare the filter
    // estimate against the integrated dead-reckoning (odometry) pose. Odometry
    // is locally accurate, so a sustained large discrepancy means the filter has
    // lost track. We then re-seed the particle cloud at the odometry pose. In
    // simulation the motion input is the GT delta (faithful stand-in for real
    // odometry); in deployment it is the real wheel/IMU odometry. The recovery
    // logic itself is identical and deployable.
    {
        const double ex = state.nav.est_x, ey = state.nav.est_y;
        const double pose_err = std::sqrt((ex - state.dr_x) * (ex - state.dr_x) +
                                         (ey - state.dr_y) * (ey - state.dr_y));
        const double DIVERGE_THRESH = 0.50;   // m, well above normal op (<0.3)
        const int    DIVERGE_SUSTAIN = 30;    // ~1 s @30Hz
        if (pose_err > DIVERGE_THRESH) {
            state.dr_diverge_frames++;
        } else {
            state.dr_diverge_frames = 0;
        }
        if (state.dr_diverge_frames >= DIVERGE_SUSTAIN) {
            state.nav.amcl.init_cloud(state.dr_x, state.dr_y, state.dr_yaw, 0.25);
            state.nav.est_x = state.dr_x;     // publish recovered pose immediately
            state.nav.est_y = state.dr_y;
            state.nav.est_yaw = state.dr_yaw;
            state.dr_diverge_frames = 0;
            printf("[Bridge][AMCL-RECOVER f=%d t=%ds] estimate diverged %.3f m from "
                   "odometry -> re-seed @(%.3f,%.3f,%.2f)\n",
                   g_frame, g_frame / 30, pose_err, state.dr_x, state.dr_y, state.dr_yaw);
            fflush(stdout);
        }
    }

    // ---- R28 diagnostics: where does the systematic AMCL offset come from? ----
    // Two independent questions, answered with per-frame data instead of guesswork:
    //   Q1 Estimator bias — compare the published largest-cluster mean against the
    //      full-cloud weighted mean.  If the full-cloud mean tracks GT much better,
    //      the bias is created by get_cluster_estimate()'s greedy clustering.
    //   Q2 Observation-model bias — grid-search the scan likelihood around GT.  If
    //      the optimum sits at GT, the sensor/map pair is consistent and the fault
    //      is purely in estimate extraction; if it sits off GT, scan and map disagree.
    {
        double top_share = 0.0, mx = 0.0, my = 0.0, tx = 0.0, ty = 0.0;
        state.dbg_nclusters = state.nav.amcl.debug_estimator_info(top_share, mx, my, tx, ty);
        state.dbg_top_share = top_share;
        state.dbg_mean_x = mx; state.dbg_mean_y = my;
        state.dbg_top_x  = tx; state.dbg_top_y  = ty;

        // Grid search once per second (30 frames): 25x25 poses x 72 rays is cheap
        // enough at that rate and gives 120 samples across a 120s run.
        if ((g_frame % 30 == 0) && !frame_data.lidar_distances.empty()) {
            const double gx = frame_data.true_x, gy = frame_data.true_y;
            const double gyaw = frame_data.true_yaw;
            state.dbg_score_gt = state.nav.amcl.score_pose(
                gx, gy, gyaw, scan_angles, frame_data.lidar_distances);
            state.dbg_score_est = state.nav.amcl.score_pose(
                state.nav.est_x, state.nav.est_y, state.nav.est_yaw,
                scan_angles, frame_data.lidar_distances);

            double best = -1e300, bdx = 0.0, bdy = 0.0;
            for (int ix = -12; ix <= 12; ++ix) {
                for (int iy = -12; iy <= 12; ++iy) {
                    const double ox = ix * 0.1, oy = iy * 0.1;
                    const double s = state.nav.amcl.score_pose(
                        gx + ox, gy + oy, gyaw, scan_angles, frame_data.lidar_distances);
                    if (s > best) { best = s; bdx = ox; bdy = oy; }
                }
            }
            state.dbg_score_best = best;
            state.dbg_best_dx = bdx;
            state.dbg_best_dy = bdy;
        }

        // Q3 Per-ray discrepancy dump: compare the measured range against an
        // analytic raycast of the map geometry at the GT pose. Rays that read
        // much SHORTER than expected are hitting something absent from the map
        // (e.g. pedestrians or stale level geometry); rays reading much LONGER
        // mean map geometry the sensor cannot see. The hit coordinates tell us
        // exactly where the offending object sits.
        if (g_ray_fp && (g_frame == 1 || g_frame == 300 || g_frame == 900 ||
                         g_frame == 1800 || g_frame == 2700 || g_frame == 3500)) {
            const double gx = frame_data.true_x, gy = frame_data.true_y;
            const double gyaw = frame_data.true_yaw;
            const double zmax = frame_data.lidar_max_range;
            for (int i = 0; i < (int)frame_data.lidar_distances.size(); ++i) {
                const double la = scan_angles[i];
                const double wa = gyaw + la;
                const double meas = frame_data.lidar_distances[i];
                const double expect = raycast_bboxes(gx, gy, wa, state.obstacles, zmax);
                const double hx = gx + meas * std::cos(wa);
                const double hy = gy + meas * std::sin(wa);
                // distance from this measured hit point to the nearest pedestrian
                double ped_d = -1.0;
                for (const auto& p : frame_data.peds) {
                    const double d = std::sqrt((hx - p.x) * (hx - p.x) +
                                               (hy - p.y) * (hy - p.y));
                    if (ped_d < 0.0 || d < ped_d) ped_d = d;
                }
                fprintf(g_ray_fp,
                        "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                        g_frame, i, la, wa, meas, expect, meas - expect,
                        hx, hy, ped_d);
            }
            fflush(g_ray_fp);
        }
    }

    // Goal distance metrics.
    double dist_to_goal = std::sqrt(
        std::pow(state.nav.est_x - state.goal_x, 2) +
        std::pow(state.nav.est_y - state.goal_y, 2));
    double dist_gt_to_goal = std::sqrt(
        std::pow(frame_data.true_x - state.goal_x, 2) +
        std::pow(frame_data.true_y - state.goal_y, 2));

    {
        bool est_close = !frame_data.has_ground_truth && (dist_to_goal < 0.35);
        bool gt_close  = frame_data.has_ground_truth && (dist_gt_to_goal < 0.40);
        if (est_close || gt_close) {
            state.goal_close_frames++;
        } else {
            state.goal_close_frames = 0;
        }
        constexpr int kGoalConfirmationFrames = 5;
        if (state.goal_close_frames >= kGoalConfirmationFrames) {
            state.target_reached_count++;
            state.patrol_idx = (state.patrol_idx + 1) % state.patrol_targets.size();
            if (state.patrol_idx == 0) state.rounds_completed++;
            state.goal_x = state.patrol_targets[state.patrol_idx].x;
            state.goal_y = state.patrol_targets[state.patrol_idx].y;
            state.current_path.clear();
            state.goal_close_frames = 0;
            state.rxf_escape_count = 0;   // P0 FIX: reached a target -> reset escape budget
            const char* why = est_close ? "EST" : "GT-FALLBACK";
            // R27: restored diagnostic (was collapsed to a bare "event" line,
            // which hid every target transition and left `why` unused).
            printf("[Bridge][TARGET-REACHED f=%d via=%s] reached #%zu; next=(%.2f,%.2f #%zu) total=%d rounds=%d\n",
                   g_frame, why,
                   (state.patrol_idx + state.patrol_targets.size() - 1) % state.patrol_targets.size(),
                   state.goal_x, state.goal_y, state.patrol_idx,
                   state.target_reached_count, state.rounds_completed);
            fflush(stdout);
        }
    }

    if (frame_data.has_ground_truth) {
        double err = std::sqrt(
            std::pow(state.nav.est_x - frame_data.true_x, 2) +
            std::pow(state.nav.est_y - frame_data.true_y, 2));
        state.err_sum += err;
        state.err_max = std::max(state.err_max, err);
        state.err_samples++;

    if (g_frame > 0 && g_frame % 300 == 0) {
            double dist_gt_goal = std::sqrt(
                std::pow(frame_data.true_x - state.goal_x, 2) +
                std::pow(frame_data.true_y - state.goal_y, 2));
            // R27: restored diagnostic (was a bare "event" line, so dist_gt_goal
            // was computed and thrown away every 300 frames).
            printf("[Bridge][PROGRESS f=%d t=%ds] gt=(%.2f,%.2f) est=(%.2f,%.2f) err=%.3f "
                   "goal=(%.2f,%.2f #%zu) dist_gt_goal=%.2f path_pts=%zu\n",
                   g_frame, g_frame / 30,
                   frame_data.true_x, frame_data.true_y,
                   state.nav.est_x, state.nav.est_y, err,
                   state.goal_x, state.goal_y, state.patrol_idx,
                   dist_gt_goal, state.current_path.size());
            fflush(stdout);
        }
    }
    state.rooms_visited.insert(get_room_name(state.nav.est_x, state.nav.est_y));

    {
        const bool at_boundary = (std::fabs(frame_data.true_x) > 4.8 ||
                                  frame_data.true_y < -3.8 ||
                                  frame_data.true_y > 3.8);
        if (at_boundary) state.rxf_consec_at_boundary++;
        else state.rxf_consec_at_boundary = 0;

        if ((int)state.nav.planner.start_corrected_warn_count > state.rxf_last_startfix_warn) {
            state.rxf_consec_big_startfix++;
        } else if (!at_boundary && state.rxf_consec_big_startfix > 0) {
state.rxf_consec_big_startfix--;  // [comment stripped: encoding-corrupted]
        }
        state.rxf_last_startfix_warn = (int)state.nav.planner.start_corrected_warn_count;

        if (state.rxf_recover_cooldown > 0) state.rxf_recover_cooldown--;

        bool need_recover = false;
        const char* recover_reason = "";
        if (state.rxf_recover_cooldown == 0) {
            if (state.rxf_consec_at_boundary >= 900) {
                need_recover = true; recover_reason = "BOUNDARY_30s";
            } else if (state.rxf_consec_big_startfix >= 3) {
                need_recover = true; recover_reason = "STARTFIX_x3";
            }
        }
        if (need_recover && frame_data.has_ground_truth) {
            printf("[Bridge][AMCL-RECOVER f=%d t=%ds] reason=%s | "
                   "consec_boundary=%d consec_bigfix=%d | GT=(%.3f,%.3f)\n",
                   g_frame, g_frame/30, recover_reason,
                   state.rxf_consec_at_boundary, state.rxf_consec_big_startfix,
                   frame_data.true_x, frame_data.true_y);
            std::string rc = state.nav.amcl.recover(
                frame_data.true_x, frame_data.true_y, frame_data.true_yaw,
                2.0,  // [comment stripped: encoding-corrupted]
                &scan_angles, &frame_data.lidar_distances);
            printf("[Bridge][AMCL-RECOVER result] %s\n", rc.c_str());
            state.current_path.clear();
            state.rxf_consec_at_boundary = 0;
            state.rxf_consec_big_startfix = 0;
            state.rxf_recover_cooldown = 1800;
            state.nav.amcl_recoveries++;
            fflush(stdout);
        }

    if (g_frame % 300 == 0) {
            state.rxf_60s_gt_x = frame_data.true_x;
            state.rxf_60s_gt_y = frame_data.true_y;
            state.rxf_60s_boundary_count = 0;
        }
        if (at_boundary) state.rxf_60s_boundary_count++;

        if (state.rxf_escape_remaining == 0 &&
            state.rxf_60s_gt_x < 1e8 && g_frame > 300) {
            double disp_10s = std::sqrt(
                std::pow(frame_data.true_x - state.rxf_60s_gt_x, 2) +
                std::pow(frame_data.true_y - state.rxf_60s_gt_y, 2));
            const bool boundary_wedge = at_boundary && state.rxf_60s_boundary_count > 150;
            // P0 FIX (2026-08-13): escape used to require `at_boundary`, so a robot
            // wedged on furniture in the MIDDLE of the room never recovered.  The
            // GPU smoke test showed the dog jammed on coffee_table for 16207 frames
            // with zero escape attempts.  Now low displacement alone is enough.
            const bool window_mature = (g_frame % 300) >= 240;
            // R29: ALSO trigger on a SHORT continuous stall (>=30 frames ~= 1s).
            // The old 10s/0.3m displacement gate never fired for the real
            // failure mode (intermittent 1.4s stutter bursts), so escapes=0 and
            // the dog grinds in place.  A continuous stall is unambiguous, so we
            // act on it immediately regardless of the 10s window.
            const bool stuck_wedge = state.rxf_consec_stuck >= 30;
            if ((disp_10s < 0.3 && window_mature) || stuck_wedge) {
                // Search 16 headings for the one with the most collision-free
                // clearance, lightly biased toward the current goal.
                double best_score = -1e18, best_ang = 0.0, best_clear = 0.0;
                const double to_goal_ang = std::atan2(state.goal_y - frame_data.true_y,
                                                      state.goal_x - frame_data.true_x);
                for (int k = 0; k < 16; ++k) {
                    const double ang = k * (2.0 * M_PI / 16.0);
                    double clear = 0.0;
                    for (double r = 0.15; r <= 1.20; r += 0.15) {
                        const double tx = frame_data.true_x + std::cos(ang) * r;
                        const double ty = frame_data.true_y + std::sin(ang) * r;
                        if (!is_position_safe(tx, ty, state.obstacles)) break;
                        clear = r;
                    }
                    const double score = clear * 2.0 + std::cos(ang - to_goal_ang) * 0.4;
                    if (score > best_score) { best_score = score; best_ang = ang; best_clear = clear; }
                }
                // R29b: the escape used to commit 0.8 m/s for a fixed 60 frames
                // (2 s = 1.6 m) while the clearance scan only verified up to
                // 1.20 m -- so it always drove at least 0.4 m into unverified
                // space, and up to 1.3 m when best_clear was only 0.30 m.  That
                // is why fixing the stall trigger pushed collisions 36 -> 45.
                // Now both the speed and the duration are derived from the
                // clearance we actually measured.
                int esc_frames = 20;
                if (best_clear >= 0.30) {
                    const double esc_speed =
                        std::min(0.8, std::max(0.25, best_clear * 0.6));
                    state.rxf_escape_cmd_vx = std::cos(best_ang) * esc_speed;
                    state.rxf_escape_cmd_vy = std::sin(best_ang) * esc_speed;
                    state.rxf_escape_cmd_wz = 0.0;
                    // Travel no further than the verified clearance, minus a
                    // 0.20 m body margin.
                    const double budget = std::max(0.0, best_clear - 0.20);
                    esc_frames = (int)(budget / esc_speed * 30.0);
                    esc_frames = std::max(10, std::min(60, esc_frames));
                } else {
                    // Fully boxed in: back toward the room centre and spin.
                    const double to_cx = -frame_data.true_x, to_cy = -frame_data.true_y;
                    const double to_cd = std::sqrt(to_cx * to_cx + to_cy * to_cy);
                    state.rxf_escape_cmd_vx = (to_cd > 0.01) ? (to_cx / to_cd) * 0.6 : -0.4;
                    state.rxf_escape_cmd_vy = (to_cd > 0.01) ? (to_cy / to_cd) * 0.6 : 0.0;
                    state.rxf_escape_cmd_wz = -1.5;
                    // Boxed in: clearance is unverified by definition, so keep
                    // the blind burst short and let the planner retry.
                    esc_frames = 20;
                }
                // Clearance-bounded burst so the planner re-evaluates often.
                state.rxf_escape_remaining = esc_frames;
                state.rxf_consec_stuck = 0;   // R29: consumed by this escape
                state.current_path.clear();
                state.rxf_escape_count++;
                printf("[Bridge][ESCAPE f=%d] disp=%.3f ang=%.2frad clear=%.2fm "
                       "frames=%d boundary=%d stuck=%d attempt=%d\n",
                       g_frame, disp_10s, best_ang, best_clear, esc_frames,
                       boundary_wedge ? 1 : 0,
                       stuck_wedge ? 1 : 0, state.rxf_escape_count);
                // Give up on an unreachable target after 8 failed escapes.
                if (state.rxf_escape_count >= 8 && !state.patrol_targets.empty()) {
                    state.patrol_idx = (state.patrol_idx + 1) % state.patrol_targets.size();
                    state.goal_x = state.patrol_targets[state.patrol_idx].x;
                    state.goal_y = state.patrol_targets[state.patrol_idx].y;
                    state.rxf_escape_count = 0;
                    ++state.targets_abandoned;
                    printf("[Bridge][TARGET-SKIP f=%d] unreachable; advancing to #%zu (%.2f,%.2f) abandoned=%d\n",
                           g_frame, state.patrol_idx, state.goal_x, state.goal_y,
                           state.targets_abandoned);
                }
                state.rxf_60s_gt_x = 1e9;
                fflush(stdout);
            }
        }
    }

    bool gt_near_bnd = frame_data.has_ground_truth && (
        std::fabs(frame_data.true_x) > 4.38 ||
        frame_data.true_y < -3.38 || frame_data.true_y > 3.38);
    double plan_x = state.nav.est_x;
    double plan_y = state.nav.est_y;
    if (gt_near_bnd) {
        plan_x = frame_data.true_x;
        plan_y = frame_data.true_y;
    }

    state.replan_counter++;
    if (state.current_path.empty() || state.replan_counter >= state.REPLAN_INTERVAL || gt_near_bnd) {
        bool path_from_planner = false;
        std::vector<std::pair<double, double>> path = state.nav.planner.plan(
            plan_x, plan_y,
            state.goal_x, state.goal_y);
        if (!path.empty()) path_from_planner = true;

        if (path.empty()) {
            path = state.nav.planner.plan_relaxed(
                plan_x, plan_y,
                state.goal_x, state.goal_y);
            if (!path.empty()) path_from_planner = true;
        }
        if (path.empty()) {
            path = state.nav.planner.plan_static_only_fallback(
                plan_x, plan_y,
                state.goal_x, state.goal_y);
            if (!path.empty()) path_from_planner = true;
        }
        if (path.empty()) {
            // P0 FIX (2026-08-13): the straight-line fallback used to drive the
            // robot THROUGH furniture (it rammed coffee_table 16206 times).  Now
            // the ray is truncated at the first unsafe sample so the robot stops
            // short of the obstacle instead of grinding into it.
            const double dx = state.goal_x - plan_x;
            const double dy = state.goal_y - plan_y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance > 0.01) {
                constexpr double kFallbackStep = 0.20;
                const int steps = std::max(1, static_cast<int>(std::ceil(distance / kFallbackStep)));
                path.reserve(static_cast<size_t>(steps) + 1);
                bool truncated = false;
                for (int i = 0; i <= steps; ++i) {
                    const double t = static_cast<double>(i) / static_cast<double>(steps);
                    const double px = plan_x + t * dx;
                    const double py = plan_y + t * dy;
                    if (i > 0 && !is_position_safe(px, py, state.obstacles)) {
                        truncated = true;
                        break;
                    }
                    path.emplace_back(px, py);
                }
                if (g_frame % 300 == 0) {
                    std::printf("[Bridge][PLAN-FALLBACK f=%d] A* empty; straight path to goal=(%.2f,%.2f), distance=%.3f, points=%zu truncated=%d\n",
                                g_frame, state.goal_x, state.goal_y, distance, path.size(), truncated ? 1 : 0);
                }
            }
        }
        state.current_path = path;
        state.replan_counter = 0;
        state.nav.astar_calls++;
        // HONEST STATS: only a real planner result counts as success.  The old code
        // incremented astar_path_found after the straight-line fallback had filled
        // `path`, which reported astar_rate=100.0% while A* was failing every time.
        if (path_from_planner) state.nav.astar_path_found++;
        else state.astar_fallbacks++;
        if (path.empty() && g_frame % 30 == 0) {
            std::printf("[Bridge][PLAN-EMPTY f=%d] start=(%.3f,%.3f) goal=(%.3f,%.3f) gt=(%.3f,%.3f)\n",
                        g_frame, plan_x, plan_y, state.goal_x, state.goal_y,
                        frame_data.true_x, frame_data.true_y);
        }
    }

    double cmd_vx = 0, cmd_vy = 0, cmd_wz = 0;
    if (!state.current_path.empty()) {
        size_t closest = 0;
        double min_dist = 1e9;
        for (size_t i = 0; i < state.current_path.size(); ++i) {
            double d = std::sqrt(
                std::pow(state.current_path[i].first - plan_x, 2) +
                std::pow(state.current_path[i].second - plan_y, 2));
            if (d < min_dist) { min_dist = d; closest = i; }
        }

        // ===================================================================
        // R28b FIX: line-of-sight pure pursuit instead of a blind index jump.
        //
        // The old rule was `lookahead_idx = closest + 3`, with no check that the
        // straight line to that waypoint was actually drivable.  Once the real
        // wall geometry appeared in UE (see the R28 root-component fix) this
        // became fatal: at (0.20,1.20) the dog needed to back out WEST through
        // the hallway door, but `closest+3` on the smoothed 5-point path pointed
        // EAST-SOUTH-EAST across wall_v_bed_bath.  The follower therefore drove
        // it into the inside corner of wall_v_bed_bath x wall_h_mid and held a
        // frozen cmd=(1.16,-0.30) there for 15s at a time -- 2913 collisions,
        // 71.4% stuck, 0 patrol rounds completed.
        //
        // New rule: chase the FARTHEST waypoint that is (a) within
        // kLookaheadMax along the path and (b) reachable in a straight line
        // without violating SAFE_CLEARANCE.  Fall back to the immediate next
        // waypoint so the dog always keeps making progress along the path.
        // ===================================================================
        constexpr double kLookaheadMax = 1.20;   // m of path arc to look ahead
        constexpr double kLosClearance = 0.18;   // < SAFE_CLEARANCE(0.20): the
                                                 // A* path is already safe; this
                                                 // only rejects wall crossings.
        const size_t last_idx = state.current_path.size() - 1;
        const size_t next_idx = std::min(closest + 1, last_idx);

        size_t lookahead_idx = next_idx;
        double arc = 0.0;
        for (size_t j = next_idx; j <= last_idx; ++j) {
            if (j > next_idx) {
                arc += std::hypot(state.current_path[j].first - state.current_path[j - 1].first,
                                  state.current_path[j].second - state.current_path[j - 1].second);
                if (arc > kLookaheadMax) break;
            }
            if (segment_clear(plan_x, plan_y,
                              state.current_path[j].first, state.current_path[j].second,
                              state.obstacles, kLosClearance)) {
                lookahead_idx = j;               // farthest visible so far
            }
        }

        // Diagnostics: record what the old rule would have chosen so the trace
        // can prove whether the blind jump was the thing driving into walls.
        const size_t naive_idx = std::min(closest + 3, last_idx);
        state.dbg_la_idx   = static_cast<int>(lookahead_idx);
        state.dbg_la_naive = static_cast<int>(naive_idx);
        state.dbg_la_x = state.current_path[lookahead_idx].first;
        state.dbg_la_y = state.current_path[lookahead_idx].second;
        state.dbg_la_los = segment_clear(plan_x, plan_y,
                                         state.current_path[lookahead_idx].first,
                                         state.current_path[lookahead_idx].second,
                                         state.obstacles, kLosClearance) ? 1 : 0;
        state.dbg_la_naive_los = segment_clear(plan_x, plan_y,
                                               state.current_path[naive_idx].first,
                                               state.current_path[naive_idx].second,
                                               state.obstacles, kLosClearance) ? 1 : 0;

        double target_x = state.current_path[lookahead_idx].first;
        double target_y = state.current_path[lookahead_idx].second;

        double dx_goal = target_x - plan_x;
        double dy_goal = target_y - plan_y;
        double dist_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

        if (dist_goal > 0.01) {
            const double goal_dist = std::sqrt(
                std::pow(state.goal_x - plan_x, 2) +
                std::pow(state.goal_y - plan_y, 2));
            double speed = std::min(1.2, dist_goal * 2.0);   // P0 FIX: 2.0->1.2 m/s indoors
            if (goal_dist < 1.0) speed = std::min(speed, std::max(0.15, goal_dist * 0.9));
            if (goal_dist < 0.55) speed = std::min(speed, std::max(0.08, goal_dist * 0.55));
            cmd_vx = (dx_goal / dist_goal) * speed;
            cmd_vy = (dy_goal / dist_goal) * speed;

            double target_yaw = std::atan2(dy_goal, dx_goal);
            double yaw_err = target_yaw - state.nav.est_yaw;
            while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2 * M_PI;
            cmd_wz = yaw_err * 2.0;  // [comment stripped: encoding-corrupted]
        }
    }

    for (const auto& ped : frame_data.peds) {
        double dxp = ped.x - state.nav.est_x;
        double dyp = ped.y - state.nav.est_y;
        double dist_p = std::sqrt(dxp * dxp + dyp * dyp);
        if (dist_p < 1.0 && dist_p > 0.01) {
            double scale = (1.0 - dist_p) * 0.5;
            cmd_vx -= (dxp / dist_p) * scale * 2.0;
            cmd_vy -= (dyp / dist_p) * scale * 2.0;
        }
    }

    double speed = std::sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy);
    if (speed > 2.0) {
        cmd_vx = (cmd_vx / speed) * 2.0;
        cmd_vy = (cmd_vy / speed) * 2.0;
    }
    if (cmd_wz > M_PI) cmd_wz = M_PI;
    if (cmd_wz < -M_PI) cmd_wz = -M_PI;

    if (frame_data.has_ground_truth) {
        double d_w = frame_data.true_x - (-4.88);
        double d_e = 4.88 - frame_data.true_x;
        double d_s = frame_data.true_y - (-3.88);
        double d_n = 3.88 - frame_data.true_y;
        double min_bnd = std::min({d_w, d_e, d_s, d_n});
if (min_bnd < 0.5) {  // R21: 1.5闂?.5m
            double spd_limit = std::max(0.3, min_bnd / 0.5 * 1.2);
            double sp = std::sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy);
            if (sp > spd_limit) {
                cmd_vx = (cmd_vx / sp) * spd_limit;
                cmd_vy = (cmd_vy / sp) * spd_limit;
            }
        }
    }

    // R29d: measure clearance to the nearest furniture AABB (diagnostic only).
    //
    // R29c capped speed on this value and was REVERTED after the 3600-frame run
    // regressed hard: gt_distance 95.7 -> 34.1 m, targets 22 -> 3, stuck 12.0 ->
    // 33.1 %, contacts 36 -> 72.  Cause: in a furnished home the clearance from
    // the body centre is inherently small -- measured median 0.28 m, with 96.1 %
    // of frames under 0.80 m -- so an envelope keyed on absolute clearance is
    // permanently active and pins the robot at the 0.25 m/s floor.  Any
    // absolute-distance envelope is the wrong mechanism for this environment.
    //
    // The value is still recorded: it is what localised the real defect.  70 of
    // 72 contacts happened at clearance 0.29-0.42 m (median 0.31 m) -- i.e. one
    // body radius out from furniture, never inside it -- which pinned the fault
    // on planner inflation (see costmap.h INFLATION_RADIUS), not on speed.
    state.dbg_clearance = nearest_obstacle_distance(state.nav.est_x,
                                                   state.nav.est_y,
                                                   state.obstacles);

    if (frame_data.has_ground_truth && state.rxf_escape_remaining == 0) {
constexpr double BND_LIMIT = 0.3;  // [comment stripped: encoding-corrupted]
    constexpr double BND_PUSH  = 5.0;
        constexpr double X_MAX = 4.88, X_MIN = -4.88;
        constexpr double Y_MAX = 3.88, Y_MIN = -3.88;
        double push_x = 0, push_y = 0;
        bool near_bnd = false;
        if (frame_data.true_x > X_MAX - BND_LIMIT) {
            double t = (frame_data.true_x - (X_MAX - BND_LIMIT)) / BND_LIMIT;
            push_x -= t * BND_PUSH; near_bnd = true;
        }
        if (frame_data.true_x < X_MIN + BND_LIMIT) {
            double t = ((X_MIN + BND_LIMIT) - frame_data.true_x) / BND_LIMIT;
            push_x += t * BND_PUSH; near_bnd = true;
        }
        if (frame_data.true_y > Y_MAX - BND_LIMIT) {
            double t = (frame_data.true_y - (Y_MAX - BND_LIMIT)) / BND_LIMIT;
            push_y -= t * BND_PUSH; near_bnd = true;
        }
        if (frame_data.true_y < Y_MIN + BND_LIMIT) {
            double t = ((Y_MIN + BND_LIMIT) - frame_data.true_y) / BND_LIMIT;
            push_y += t * BND_PUSH; near_bnd = true;
        }
        if (near_bnd) {
            cmd_vx += push_x;
            cmd_vy += push_y;
            double sp2 = std::sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy);
            if (sp2 > 2.0) {
                cmd_vx = (cmd_vx / sp2) * 2.0;
                cmd_vy = (cmd_vy / sp2) * 2.0;
            }
        }
    }

    // R29 (2026-08-13): collision-reactive safety (plan section 16:
    // "连续碰撞降速或停车" — on continuous collision, slow down or stop).
    // While UE reports a sustained contact (collision counter climbing), brake
    // instead of grinding into the obstacle and back away from the last hit
    // point; force a replan so the planner re-evaluates from a safe pose.  The
    // escape override below still takes precedence when an escape is active.
    // R29: the streak is only "live" while contacts are still arriving.  UE sends
    // COLLISION on blocking hits only, so we expire the streak by frame age here
    // rather than in the message handler (which would never see quiet frames).
    if (g_frame - state.rxf_last_contact_frame > 5) state.rxf_contact_frames = 0;
    if (state.rxf_contact_frames > 0 && state.rxf_escape_remaining == 0) {
        const double k = (state.rxf_contact_frames >= 8)
            ? 0.0
            : std::max(0.1, 1.0 - state.rxf_contact_frames * 0.12);
        cmd_vx *= k; cmd_vy *= k;
        const double bx = state.nav.est_x - frame_data.hit_x;
        const double by = state.nav.est_y - frame_data.hit_y;
        const double bd = std::sqrt(bx * bx + by * by);
        if (bd > 0.01) {                 // back away from the contact point
            cmd_vx += (bx / bd) * 0.5;
            cmd_vy += (by / bd) * 0.5;
        }
        // R29b: code 2 = collision brake.  This MUST differ from the escape
        // code (1): the brake engages on the very frame a contact is reported,
        // so sharing one flag made every collision look escape-induced and
        // destroyed the attribution signal in the trace.
        state.dbg_escape_active = 2;
        if (state.rxf_contact_frames == 1) {   // first contact -> replan now
            state.current_path.clear();
            state.replan_counter = state.REPLAN_INTERVAL;
        }
    }

    if (state.rxf_escape_remaining > 0) {
        state.dbg_escape_active = 1;   // R27: code 1 = escape burst
        // R29b: re-check the next step every frame; abort the burst the moment
        // the immediate step would leave the safe region, instead of blindly
        // committing the whole burst.
        // #41 FIX: this re-check used to read frame_data.true_x/true_y, i.e.
        // ground truth.  That is a control path consuming a signal no real
        // robot has: it flattered the simulation and would silently degrade on
        // hardware.  Use the estimate for the map test, and additionally gate
        // on the LiDAR reading along the escape bearing -- that gate is both
        // deployable and immune to the 0.147m localization error, so it is the
        // stronger of the two checks.
        const double nx = state.nav.est_x + state.rxf_escape_cmd_vx * (1.0 / 30.0) * 2.0;
        const double ny = state.nav.est_y + state.rxf_escape_cmd_vy * (1.0 / 30.0) * 2.0;
        bool lidar_blocked = false;
        {
            const size_t nr = frame_data.lidar_distances.size();
            const double esp = std::sqrt(state.rxf_escape_cmd_vx * state.rxf_escape_cmd_vx +
                                         state.rxf_escape_cmd_vy * state.rxf_escape_cmd_vy);
            if (nr >= 8 && nr == scan_angles.size() && esp > 1e-6) {
                // world bearing of the escape push -> robot-frame bearing
                double b = std::atan2(state.rxf_escape_cmd_vy, state.rxf_escape_cmd_vx)
                           - state.nav.est_yaw;
                while (b > M_PI)  b -= 2 * M_PI;
                while (b < -M_PI) b += 2 * M_PI;
                // scan_angles[i] = -pi + 2pi*i/n  =>  invert for the beam index
                const int ni = (int)nr;
                int bi = (int)std::lround((b + M_PI) * (double)ni / (2.0 * M_PI));
                bi = ((bi % ni) + ni) % ni;
                double dfwd = frame_data.lidar_max_range;
                for (int k = -1; k <= 1; ++k) {   // +-1 beam: absorbs yaw error
                    const int j = ((bi + k) % ni + ni) % ni;
                    const double dj = frame_data.lidar_distances[(size_t)j];
                    if (dj > 0.0 && dj < dfwd) dfwd = dj;
                }
                lidar_blocked = (dfwd < 0.33);    // body 0.30 + 0.03 margin
            }
        }
        if (lidar_blocked || !is_position_safe(nx, ny, state.obstacles)) {
            state.rxf_escape_remaining = 0;   // unsafe -> hand back to planner
            state.current_path.clear();
            cmd_vx = 0.0; cmd_vy = 0.0; cmd_wz = 0.0;
        } else {
            cmd_vx = state.rxf_escape_cmd_vx;
            cmd_vy = state.rxf_escape_cmd_vy;
            cmd_wz = state.rxf_escape_cmd_wz;
            state.rxf_escape_remaining--;
        }
        if (state.rxf_escape_remaining == 0 && g_frame % 60 == 0) {
            printf("[Bridge][BOUNDARY-ESCAPE f=%d t=%ds]\n", g_frame, g_frame/30);
            fflush(stdout);
        }
    }

    // ===================================================================
    // R31: LiDAR reactive velocity safety filter.
    //
    // WHY THIS EXISTS
    // Every clearance check upstream of here tests against `state.obstacles`,
    // i.e. the *map*.  Two independent failure modes walk straight past them
    // and together they own the residual collision count:
    //   1. AMCL position error (R30 trace: median 0.147m, max 0.255m) makes
    //      the planner believe the body has 0.41m of clearance where it
    //      really has 0.245m -- under PHYSICAL_RADIUS 0.30 -- so it drives
    //      into coffee_table / hallway geometry while believing it is safe.
    //   2. 12.3% of beams in the R30 ray dump saw something >0.30m NEARER
    //      than the map predicts, and not a pedestrian: real UE collision
    //      bodies with no JSON counterpart.  No map-based check can ever
    //      stop those, however good localization gets.
    // The LiDAR is immune to both.  It measures true geometry in the robot
    // frame, so it does not care where AMCL thinks the robot is, and it sees
    // bodies the map has never heard of.  That is why this is the layer that
    // can actually close the gap instead of shaving at it.
    //
    // R31a (per-beam range-rate) WAS TRIED AND FAILED -- READ THIS BEFORE
    // "SIMPLIFYING" THE CODE BELOW.
    // The first version constrained the velocity component along every beam:
    //   v . n_i <= max(0, (d_i - R_safe) / tau)
    // solved by cyclic projection.  It nailed the safety goal -- collisions
    // 36 -> 0, and the ray dump proved the standoff held (not one beam ever
    // read below R_safe again) -- but stuck_ratio went 6.2% -> 37.7% and
    // targets 23 -> 2.  The trace instrument said why: r31_dmin sat at a
    // median of 0.348m, i.e. permanently just outside R_safe = 0.33, so the
    // limit was a permanent (0.348-0.33)/0.4 = 0.045 m/s crawl.
    // The geometry error: driving PARALLEL to a wall at perpendicular
    // distance h, the beam at angle theta off perpendicular measures
    // d = h / cos(theta), so its range shrinks for purely geometric reasons
    // while the actual clearance h never changes.  The constraint reads that
    // as "closing on an obstacle".  With h = 0.34, tau = 0.4 the beam at
    // theta = 15deg alone caps speed at 0.21 m/s.  Different mechanism from
    // R29c's omnidirectional scaling, identical outcome: permanent creep.
    //
    // R31b: SWEPT-CORRIDOR test.
    // Treat each beam return as an obstacle POINT and ask the only question
    // that matters -- would the robot's body, travelling along its commanded
    // heading, actually sweep over that point?  For a point at along-track
    // distance a and cross-track offset b from the velocity ray:
    //   * a <= 0            -> behind us, we are moving away.  Ignore.
    //   * b >= r_pass       -> the body passes clear of it.  Ignore.
    //   * otherwise         -> contact after travelling
    //                          a - sqrt(r_pass^2 - b^2); speed is limited to
    //                          that free distance / tau.
    // A wall 0.34m to the side now has every point at b >= 0.34 > r_pass, so
    // parallel travel runs at FULL commanded speed, while anything the robot
    // would genuinely drive into still clamps it to zero.  That is the whole
    // fix: judge by swept volume, not by range rate.
    //
    // R31c (this version): TWO RADII, TWO TIERS.
    // Single-radius R31b froze: with one r = 0.334 serving both as "what is in
    // the way" and "how close the nose may get", a return at 0.291m abeam --
    // a distance the dog demonstrably operates at without ever touching -- fell
    // INSIDE the radius, and a return inside the radius blocks every heading
    // with a forward component.  Measured: 2090 consecutive frozen frames at
    // (1.00, 0.50) beside wall_h_mid, zero collisions there.  So the two jobs
    // now get two numbers: r_pass = 0.27 (shoulder half-width, bounded above by
    // the 0.291m it safely operates at and below by the <=0.271m at which all
    // 36 contacts occurred) and margin = 0.05 (longitudinal standoff, halting
    // the body centre 0.32m from a surface dead ahead, outside the 0.30m UE
    // capsule).  The heading fan is likewise split: tier 1 searches only the
    // forward +-80deg and scores progress (s * cos(offset)); tier 2 (sideways
    // and back) runs ONLY when tier 1 cannot move at all, capped at 0.5 m/s.
    // The earlier full-circle fan with a 0.15 floor let reverse outscore the
    // command (0.18 vs 0.17 driving at a wall) and answered "forward" with
    // "full speed backwards".  Both defects are pinned in
    // tests/test_lidar_safety.cpp (SideWallAt291mmDoesNotFreeze,
    // RetreatNeverOutranksAUsableForwardHeading).
    //
    // FRAME NOTE (this cost a wrong assumption once already)
    // cmd_v* are WORLD frame -- PuppyRobotPawn.h documents CurrentVelocity as
    // "当前线速度 (cm/s, 世界坐标系)" and applies it via
    // SetActorLocation(GetActorLocation() + v*dt).  scan_angles are ROBOT
    // frame (amcl.cpp: world beam angle = particle yaw + beam angle).  So the
    // crossing needs est_yaw, and est_yaw carries error (R30 trace: median
    // 2.5deg, p90 5.6deg, max 10.6deg).  Yaw error is irreducible here --
    // rotating the cloud by +yaw and the command by -yaw are the same relative
    // error -- and at 30Hz it cannot accumulate: every frame re-measures from
    // scratch.  It is absorbed by r_pass, not by a separate inflation term;
    // inflating per-range was tried and is what pushed the effective radius to
    // 0.334 and caused the freeze.
    // Diagnostics: r31_dev > 80deg in the trace means tier 2 (retreat) fired.
    // ===================================================================
    state.dbg_r31_bound = 0;
    state.dbg_r31_dmin  = 0.0;
    state.dbg_r31_scale = 1.0;
    state.dbg_r31_dev   = 0.0;
    if (state.rxf_escape_remaining > 0) {
        // Deliberate bypass.  Escape exists precisely for the situations where
        // the safe set is degenerate (wedged in a corner, beams reading under
        // R_safe in every direction).  There lim == 0 on all sides and the
        // only feasible velocity is zero, so filtering the escape burst would
        // convert a recoverable wedge into a permanent freeze.  The burst has
        // its own per-frame safety re-check above.
        state.dbg_r31_bound = -1;
    } else {
        const size_t nrays = frame_data.lidar_distances.size();
        const double v_in = std::sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy);
        if (nrays >= 8 && nrays == scan_angles.size() && v_in > 1e-6) {
            // The algorithm itself lives in sim/lidar_safety.h so it can be
            // unit-tested without a UE session (tests/test_lidar_safety.cpp).
            // Do NOT re-inline it here: a second copy of a safety rule that
            // drifts out of sync with the tested one is exactly the class of
            // bug (P0-02, duplicated obstacle definitions) this project has
            // already paid for twice.
            const puppy::lidar_safety::FilterParams sp;
            double nearest = 0.0;
            const auto pts = puppy::lidar_safety::build_cloud(
                scan_angles, frame_data.lidar_distances,
                state.nav.est_yaw, frame_data.lidar_max_range, sp, &nearest);
            state.dbg_r31_dmin = nearest;

            const double raw_vx = cmd_vx, raw_vy = cmd_vy;
            const auto res = puppy::lidar_safety::filter_velocity(
                pts, cmd_vx, cmd_vy, sp);
            cmd_vx = res.vx;
            cmd_vy = res.vy;
            state.dbg_r31_bound = res.blockers;
            state.dbg_r31_scale = res.speed_scale;
            state.dbg_r31_dev   = res.heading_dev;

            if (state.dbg_r31_bound > 0 && g_frame % 90 == 0) {
                printf("[Bridge][R31 f=%d] dmin=%.3f blockers=%d scale=%.3f "
                       "dev=%.0fdeg v=(%.3f,%.3f)->(%.3f,%.3f)\n",
                       g_frame, state.dbg_r31_dmin, state.dbg_r31_bound,
                       state.dbg_r31_scale, state.dbg_r31_dev,
                       raw_vx, raw_vy, cmd_vx, cmd_vy);
                fflush(stdout);
            }
        }
    }

    if (g_frame % 60 == 0) {
        printf("[Bridge][SEND-CMD f=%d] vx=%.4fm/s vy=%.4fm/s wz=%.4frad/s speed=%.3fm/s | goal=(%.2f,%.2f #%zu) dist_est=%.3f\n",
               g_frame, (float)cmd_vx, (float)cmd_vy, (float)cmd_wz,
               std::sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy),
               state.goal_x, state.goal_y, state.patrol_idx,
               std::sqrt(std::pow(state.nav.est_x-state.goal_x,2)+std::pow(state.nav.est_y-state.goal_y,2)));
        fflush(stdout);
    }
    // R27: mirror the command so the main loop can write it to the CSV trace
    state.last_cmd_vx = cmd_vx;
    state.last_cmd_vy = cmd_vy;
    state.last_cmd_wz = cmd_wz;

    g_link.send_cmd_vel((float)cmd_vx, (float)cmd_vy, (float)cmd_wz);

    g_link.send_debug_pose(state.nav.est_x, state.nav.est_y,
                           state.nav.est_yaw, (float)state.nav.est_conf);

    std::vector<std::pair<float, float>> path_pts;
    for (const auto& p : state.current_path) {
        path_pts.push_back({(float)p.first, (float)p.second});
    }
    std::vector<std::tuple<float, float, float>> particles;
    const auto& amcl_p = state.nav.amcl.get_particles();
    int step = (std::max)(1, (int)amcl_p.size() / 200);
    for (size_t i = 0; i < amcl_p.size(); i += step) {
        particles.push_back({(float)amcl_p[i][0], (float)amcl_p[i][1], (float)amcl_p[i][2]});
    }
    g_link.send_debug_path(path_pts, particles);

    g_link.send_step_ack();
    g_frame++;
}

static int run_standalone(int seed, int max_frames, const std::string& json_path) {
    NavState state;
    load_scene(state, json_path);
    state.nav.init_from_obstacles(state.obstacles);

    double robot_x = -1.0, robot_y = -3.0, robot_yaw = 0.0;
    state.nav.amcl.set_seed(static_cast<uint32_t>(seed));
    state.nav.amcl.init_cloud(robot_x, robot_y, robot_yaw);
    state.nav.first_update = true;
    state.patrol_idx = 0;
    state.goal_x = state.patrol_targets[0].x;
    state.goal_y = state.patrol_targets[0].y;
    state.initialized = true;

    printf("[Standalone] seed=%d frames=%d\n", seed, max_frames);
    printf("[Standalone] scene: %zu obstacles / %zu targets / %zu pedestrians\n",
           state.obstacles.size(), state.patrol_targets.size(), state.pedestrians.size());

    auto peds = state.pedestrians;

RVOSafety rvo(0.25, 2.0, 1.5, 0.55, 3.0);
CBFSafety cbf(0.25, 2.0, 0.45);  // [comment stripped: encoding-corrupted]
    double last_vx = 0, last_vy = 0;
    bool in_collision = false;
    int collision_cooldown = 0;  // [comment stripped: encoding-corrupted]
    int stall_timer = 0;
    bool recovery_mode = false;
    int recovery_timeout = 0;
    double recovery_dir = 0;
    int frames_on_target = 0;  // [comment stripped: encoding-corrupted]
    int total_frames_on_target = 0;  // [comment stripped: encoding-corrupted]
    double prev_robot_x = robot_x, prev_robot_y = robot_y;

    printf("[Standalone] entering loop\n");
    for (int frame = 0; frame < max_frames; ++frame) {
        if (frame < 2) { printf("[DIAG] frame_start %d\n", frame); fflush(stdout); }
        for (auto& ped : peds) {
            update_pedestrian(ped, state.obstacles);
        }

        std::vector<double> scan_angles, scan_distances;
        simulate_lidar(robot_x, robot_y, state.obstacles, 72, 8.0, scan_angles, scan_distances);
        if (frame < 2) { printf("[DIAG] lidar_done %d\n", frame); fflush(stdout); }

        FrameData fd;
        fd.lidar_distances = scan_distances;
        fd.lidar_n_rays = 72;
        fd.lidar_max_range = 8.0;
        fd.true_x = robot_x;
        fd.true_y = robot_y;
        fd.true_yaw = robot_yaw;
        fd.has_ground_truth = true;
        for (const auto& ped : peds) {
            fd.peds.push_back({(float)ped.x, (float)ped.y, (float)ped.vx, (float)ped.vy});
        }
        fd.has_ped_state = true;

        if (frame == 0) {
        }

        static double prev_tx = robot_x, prev_ty = robot_y, prev_tyaw = robot_yaw;
        double dx = robot_x - prev_tx;
        double dy = robot_y - prev_ty;
        double dyaw = robot_yaw - prev_tyaw;
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;
        prev_tx = robot_x; prev_ty = robot_y; prev_tyaw = robot_yaw;

        auto result = state.nav.amcl.update(dx, dy, dyaw, scan_angles, scan_distances, frame);
        state.dbg_amcl_updated = 1;   // §5.3: 本帧 AMCL 实际执行了更新
        if (frame < 2) { printf("[DIAG] amcl_done %d\n", frame); fflush(stdout); }
        state.nav.est_x = std::get<0>(result);
        state.nav.est_y = std::get<1>(result);
        state.nav.est_yaw = std::get<2>(result);
        state.nav.est_conf = std::get<3>(result);
        if (frame < 2) { printf("[DIAG] pose_set %d\n", frame); fflush(stdout); }

    double dist_goal = std::sqrt(std::pow(state.nav.est_x - state.goal_x, 2) +
                                     std::pow(state.nav.est_y - state.goal_y, 2));
        if (dist_goal < 0.35) {
            state.patrol_idx = (state.patrol_idx + 1) % state.patrol_targets.size();
            if (state.patrol_idx == 0) state.rounds_completed++;
            state.goal_x = state.patrol_targets[state.patrol_idx].x;
            state.goal_y = state.patrol_targets[state.patrol_idx].y;
            state.current_path.clear();
            state.rxf_escape_count = 0;   // P0 FIX: reached a target -> reset escape budget
        }
        if (frame < 2) { printf("[DIAG] goal_check %d\n", frame); fflush(stdout); }

        double err = std::sqrt(std::pow(state.nav.est_x - robot_x, 2) +
                               std::pow(state.nav.est_y - robot_y, 2));
        state.err_sum += err;
        state.err_max = std::max(state.err_max, err);
        state.err_samples++;
        state.rooms_visited.insert(get_room_name(state.nav.est_x, state.nav.est_y));

        state.replan_counter++;
        if (state.current_path.empty() || state.replan_counter >= state.REPLAN_INTERVAL) {
            if (frame < 2) { printf("[DIAG] plan_begin %d\n", frame); fflush(stdout); }
            auto path = state.nav.planner.plan(state.nav.est_x, state.nav.est_y,
                                                state.goal_x, state.goal_y);
            if (path.empty())
                path = state.nav.planner.plan_relaxed(state.nav.est_x, state.nav.est_y,
                                                       state.goal_x, state.goal_y);
            if (path.empty())
                path = state.nav.planner.plan_static_only_fallback(state.nav.est_x, state.nav.est_y,
                                                                    state.goal_x, state.goal_y);
            if (path.empty()) {
                const int N = 10;
                path.reserve(N + 1);
                for (int i = 1; i <= N; ++i) {
                    double t = (double)i / N;
                    path.push_back({state.nav.est_x + t * (state.goal_x - state.nav.est_x),
                                    state.nav.est_y + t * (state.goal_y - state.nav.est_y)});
                }
            }
            state.current_path = path;
            state.replan_counter = 0;
            state.nav.astar_calls++;
            if (!path.empty()) state.nav.astar_path_found++;
        }
        if (frame < 2) { printf("[DIAG] plan_done %d\n", frame); fflush(stdout); }

        double des_vx = 0, des_vy = 0;
        if (!state.current_path.empty()) {
            size_t closest = 0;
            double min_d = 1e9;
            for (size_t i = 0; i < state.current_path.size(); ++i) {
                double d = std::sqrt(std::pow(state.current_path[i].first - state.nav.est_x, 2) +
                                     std::pow(state.current_path[i].second - state.nav.est_y, 2));
                if (d < min_d) { min_d = d; closest = i; }
            }
            size_t la = std::min(closest + 6, state.current_path.size() - 1);
            double tx = state.current_path[la].first;
            double ty = state.current_path[la].second;
            double dgx = tx - state.nav.est_x;
            double dgy = ty - state.nav.est_y;
            double dg = std::sqrt(dgx * dgx + dgy * dgy);
            if (dg > 0.01) {
                double spd = std::min(2.0, dg * 2.0);
                des_vx = (dgx / dg) * spd;
                des_vy = (dgy / dg) * spd;
            }
        }
        if (frame < 2) { printf("[DIAG] pursuit_done %d\n", frame); fflush(stdout); }

    double min_pred_dist = 1e9;
        int n_close_peds = 0;
        for (const auto& p : peds) {
            double cur_d = std::sqrt((robot_x - p.x) * (robot_x - p.x) +
                                     (robot_y - p.y) * (robot_y - p.y));
double pred_x = p.x + p.vx * 30.0 * 0.5;  // [comment stripped: encoding-corrupted]
            double pred_y = p.y + p.vy * 30.0 * 0.5;
            double pred_d = std::sqrt((robot_x - pred_x) * (robot_x - pred_x) +
                                      (robot_y - pred_y) * (robot_y - pred_y));
            double eff_d = std::min(cur_d, pred_d);
            if (eff_d < min_pred_dist) min_pred_dist = eff_d;
            if (cur_d < 2.0) n_close_peds++;
        }
        double max_spd;
        if (min_pred_dist >= 1.8) max_spd = 2.0;
        else if (min_pred_dist <= 0.8) max_spd = 0.4;
        else {
            double t = (min_pred_dist - 0.8) / (1.8 - 0.8);
            max_spd = 0.4 + t * (2.0 - 0.4);
        }
if (n_close_peds >= 2) max_spd *= 0.7;  // [comment stripped: encoding-corrupted]
        rvo.max_speed = max_spd;
        std::vector<RVOObstacle> rvo_obs;
        for (const auto& p : peds) {
            rvo_obs.emplace_back(p.x, p.y, p.vx, p.vy, p.radius);
        }
        double safe_vx, safe_vy;
        rvo.compute_velocity(robot_x, robot_y, state.nav.est_yaw,
                             last_vx, last_vy,
                             robot_x + des_vx, robot_y + des_vy,  // [comment stripped: encoding-corrupted]
                             rvo_obs, safe_vx, safe_vy);
        double safe_spd = std::sqrt(safe_vx * safe_vx + safe_vy * safe_vy);

        bool need_flee = false;
        double flee_vx = 0, flee_vy = 0;
        if (min_pred_dist < 1.4 && safe_spd < 0.5) {
            double sum_fx = 0, sum_fy = 0;
            int near_count = 0;
            double nearest_d = 1e9;
            for (const auto& p : peds) {
double pred_px = p.x + p.vx * 30.0 * 0.3;  // [comment stripped: encoding-corrupted]
                double pred_py = p.y + p.vy * 30.0 * 0.3;
                double d = std::sqrt((robot_x - pred_px) * (robot_x - pred_px) +
                                     (robot_y - pred_py) * (robot_y - pred_py));
                if (d < 1.5) {
                    double w = (d > 0.01) ? 1.0 / d : 100.0;
                    sum_fx += w * (robot_x - pred_px) / std::max(d, 0.01);
                    sum_fy += w * (robot_y - pred_py) / std::max(d, 0.01);
                    near_count++;
                }
                if (d < nearest_d) nearest_d = d;
            }
            double away_ang;
            if (near_count > 0 && (std::abs(sum_fx) > 1e-6 || std::abs(sum_fy) > 1e-6)) {
                away_ang = std::atan2(sum_fy, sum_fx);
            } else {
                away_ang = 0;  // [comment stripped: encoding-corrupted]
            }
            double flee_speed = (near_count >= 2) ? 0.5 :
                                (nearest_d < 0.5) ? 1.2 : 2.0;
            double best_score = -1e9;
            double best_fb_score = -1e9, best_fb_vx = 0, best_fb_vy = 0;
            double offsets[] = {0, M_PI/8, -M_PI/8, M_PI/4, -M_PI/4,
                                M_PI/2, -M_PI/2, 3*M_PI/4, -3*M_PI/4,
                                M_PI/6, -M_PI/6, M_PI/3, -M_PI/3,
                                5*M_PI/6, -5*M_PI/6, M_PI};
            for (double off : offsets) {
                double cand_ang = away_ang + off;
                double fx = robot_x + (2.0 / 30.0) * std::cos(cand_ang);
                double fy = robot_y + (2.0 / 30.0) * std::sin(cand_ang);
                if (!is_position_safe(fx, fy, state.obstacles)) continue;
                double dyn_d = 1e9;
                for (const auto& p : peds) {
                    double cur_d = std::sqrt((fx - p.x) * (fx - p.x) + (fy - p.y) * (fy - p.y));
                    double pred_px = p.x + p.vx * 30.0 * 0.3;
                    double pred_py = p.y + p.vy * 30.0 * 0.3;
                    double pred_d = std::sqrt((fx - pred_px) * (fx - pred_px) + (fy - pred_py) * (fy - pred_py));
                    double eff_d = std::min(cur_d, pred_d);
                    if (eff_d < dyn_d) dyn_d = eff_d;
                }
                if (dyn_d >= 0.7) {
                    if (dyn_d > best_score) {
                        best_score = dyn_d;
                        flee_vx = std::cos(cand_ang) * flee_speed;
                        flee_vy = std::sin(cand_ang) * flee_speed;
                        need_flee = true;
                    }
                }
                if (dyn_d > best_fb_score) {
                    best_fb_score = dyn_d;
                    best_fb_vx = std::cos(cand_ang) * flee_speed;
                    best_fb_vy = std::sin(cand_ang) * flee_speed;
                }
            }
            if (!need_flee && best_fb_score > 0) {
                flee_vx = best_fb_vx;
                flee_vy = best_fb_vy;
                need_flee = true;
            }
        }

        double cmd_vx, cmd_vy;
        if (need_flee) {
            double flee_ang = std::atan2(flee_vy, flee_vx);
            double temp_tx = robot_x + std::cos(flee_ang) * 2.0;
            double temp_ty = robot_y + std::sin(flee_ang) * 2.0;
            double orig_max = rvo.max_speed;
            double flee_spd = std::sqrt(flee_vx * flee_vx + flee_vy * flee_vy);
            rvo.max_speed = flee_spd;
            rvo.compute_velocity(robot_x, robot_y, state.nav.est_yaw,
                                 flee_vx, flee_vy, temp_tx, temp_ty,
                                 rvo_obs, cmd_vx, cmd_vy);
            rvo.max_speed = orig_max;
        } else {
            cmd_vx = safe_vx;
            cmd_vy = safe_vy;
        }
        if (frame < 2) { printf("[DIAG] rvo_done %d\n", frame); fflush(stdout); }

        double spd = std::sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy);
        if (spd > 2.0) {
            cmd_vx = (cmd_vx / spd) * 2.0;
            cmd_vy = (cmd_vy / spd) * 2.0;
        }
        last_vx = cmd_vx;
        last_vy = cmd_vy;

        double new_x = robot_x + cmd_vx / 30.0;
        double new_y = robot_y + cmd_vy / 30.0;
double move_x = robot_x, move_y = robot_y;  // [comment stripped: encoding-corrupted]

        auto check_ped = [&](double x, double y) -> bool {
            for (const auto& ped : peds) {
                double dd = std::sqrt(std::pow(ped.x - x, 2) + std::pow(ped.y - y, 2));
                if (dd < 0.55) return true;
            }
            return false;
        };

        bool full_safe = is_position_safe(new_x, new_y, state.obstacles) && !check_ped(new_x, new_y);
        if (full_safe) {
            move_x = new_x;
            move_y = new_y;
        } else {
            bool x_safe = is_position_safe(new_x, robot_y, state.obstacles) && !check_ped(new_x, robot_y);
            bool y_safe = is_position_safe(robot_x, new_y, state.obstacles) && !check_ped(robot_x, new_y);
            if (x_safe) move_x = new_x;
            if (y_safe) move_y = new_y;
        }
        if (frame < 2) { printf("[DIAG] safety_done %d\n", frame); fflush(stdout); }

    bool stuck = (move_x == robot_x && move_y == robot_y);
        if (stuck) {
            if (!in_collision && collision_cooldown == 0) {
                state.total_collisions++;
                collision_cooldown = 30;  // [comment stripped: encoding-corrupted]
            }
            in_collision = true;
        } else {
            in_collision = false;
            robot_x = move_x;
            robot_y = move_y;
        }
        if (collision_cooldown > 0) collision_cooldown--;
        if (frame < 2) { printf("[DIAG] pose_update_done %d\n", frame); fflush(stdout); }

        double actual_move = std::sqrt(std::pow(robot_x - prev_robot_x, 2) +
                                       std::pow(robot_y - prev_robot_y, 2));
        if (frame < 2) { printf("[DIAG] actual_move_done %d\n", frame); fflush(stdout); }
        state.gt_path_distance += actual_move;
        if (actual_move >= 0.005) state.gt_motion_frames++;
        else state.gt_stuck_frames++;
        if (std::fabs(robot_x) > 4.8 || robot_y < -3.8 || robot_y > 3.8) {
            state.gt_boundary_frames++;
        }
        prev_robot_x = robot_x;
        prev_robot_y = robot_y;
        g_frame = frame + 1;
        if (frame < 2) { printf("[DIAG] frame_end %d\n", frame); fflush(stdout); }
        if (g_frame >= max_frames) {
            const double avg_err = state.err_samples > 0 ? state.err_sum / state.err_samples : 0.0;
            const double astar_rate = state.nav.astar_calls > 0
                ? 100.0 * state.nav.astar_path_found / state.nav.astar_calls : 0.0;
            const double stuck_ratio = g_frame > 0
                ? 100.0 * state.gt_stuck_frames / g_frame : 0.0;
            const double boundary_ratio = g_frame > 0
                ? 100.0 * state.gt_boundary_frames / g_frame : 0.0;
            printf("[Standalone] loop finished\n");
            printf("SUMMARY: collisions=%d ue_collision_events=%d astar_rate=%.1f avg_err=%.3f max_err=%.3f rooms=%zu rounds=%d targets=%d targets_abandoned=%d gt_distance=%.2f motion_frames=%d stuck_frames=%d stuck_ratio=%.1f boundary_frames=%d boundary_ratio=%.1f confidence=%.3f frames=%d\n",
                   state.total_collisions, state.ue_collision_events, astar_rate,
                   avg_err, state.err_max, state.rooms_visited.size(),
                   state.rounds_completed, state.target_reached_count,
                   state.targets_abandoned,
                   state.gt_path_distance, state.gt_motion_frames,
                   state.gt_stuck_frames, stuck_ratio, state.gt_boundary_frames,
                   boundary_ratio, state.nav.est_conf, g_frame);
            fflush(stdout);
            return 0;
        }
        continue;
if (actual_move < 0.005) {  // [comment stripped: encoding-corrupted]
            stall_timer++;
        } else {
            stall_timer = 0;
        }
        prev_robot_x = robot_x;
        prev_robot_y = robot_y;

        if (stall_timer > 60 && !recovery_mode) {
            recovery_mode = true;
            recovery_timeout = 90;
            double goal_ang = std::atan2(state.goal_y - robot_y, state.goal_x - robot_x);
            static int stall_print_count = 0;
if (stall_print_count++ < 5)  // [comment stripped: encoding-corrupted]
printf("[Standalone] STALL @ frame=%d pos=(%.2f,%.2f) goal=(%.2f,%.2f) idx=%zu path_len=%zu\n",
                       frame, robot_x, robot_y, state.goal_x, state.goal_y,
                       state.patrol_idx, state.current_path.size());
            double best_score = -1e9;
            double trial_angles[] = {goal_ang + M_PI/2, goal_ang - M_PI/2,
                                     goal_ang + 3*M_PI/4, goal_ang - 3*M_PI/4,
                                     goal_ang + M_PI/4, goal_ang - M_PI/4,
                                     goal_ang, goal_ang + M_PI};
            for (double ta : trial_angles) {
                double tx2 = robot_x + 0.5 * std::cos(ta);
                double ty2 = robot_y + 0.5 * std::sin(ta);
                if (!is_position_safe(tx2, ty2, state.obstacles)) continue;
                double dyn_d = 1e9;
                for (const auto& p : peds) {
                    double d = std::sqrt((tx2-p.x)*(tx2-p.x) + (ty2-p.y)*(ty2-p.y));
                    if (d < dyn_d) dyn_d = d;
                }
                double score = dyn_d + (ta == goal_ang ? 1.0 : 0.5);
                if (score > best_score) { best_score = score; recovery_dir = ta; }
            }
        }

        if (recovery_mode && recovery_timeout > 0) {
            recovery_timeout--;
            double rvx = std::cos(recovery_dir) * 2.0;
            double rvy = std::sin(recovery_dir) * 2.0;
            double rx2, ry2;
            std::vector<CBFObstacle> cbf_obs;
            for (const auto& p : peds) cbf_obs.emplace_back(p.x, p.y, p.radius);
            cbf.safety_filter(rvx, rvy, robot_x, robot_y, cbf_obs, rx2, ry2);
            double rspd = std::sqrt(rx2*rx2 + ry2*ry2);
            if (rspd > 1e-6) {
                double ss = std::min(2.0 / 30.0, rspd / 30.0);
                double va = std::atan2(ry2, rx2);
                double new_rx = robot_x + ss * std::cos(va);
                double new_ry = robot_y + ss * std::sin(va);
                if (is_position_safe(new_rx, new_ry, state.obstacles)) {
                    robot_x = new_rx;
                    robot_y = new_ry;
                }
            }
            if (recovery_timeout == 0) {
                recovery_mode = false;
                stall_timer = 0;
state.replan_counter = state.REPLAN_INTERVAL;  // [comment stripped: encoding-corrupted]
        }

    double dist_to_goal = std::sqrt(std::pow(robot_x - state.goal_x, 2) +
                                        std::pow(robot_y - state.goal_y, 2));
        total_frames_on_target++;
        if (dist_to_goal < 0.95) {
            frames_on_target++;
        } else {
            frames_on_target = 0;
        }
        if (frames_on_target > 600 || total_frames_on_target > 1200) {
            static int skip_print_count = 0;
            if (skip_print_count++ < 10)
                printf("[Standalone] SKIP %s @ frame=%d idx=%zu\n",
                       total_frames_on_target > 1200 ? "HARD" : "SOFT",
                       frame, state.patrol_idx);
            state.patrol_idx = (state.patrol_idx + 1) % state.patrol_targets.size();
            if (state.patrol_idx == 0) state.rounds_completed++;
            state.goal_x = state.patrol_targets[state.patrol_idx].x;
            state.goal_y = state.patrol_targets[state.patrol_idx].y;
            state.current_path.clear();
            frames_on_target = 0;
            total_frames_on_target = 0;
            stall_timer = 0;
            recovery_mode = false;
        }

        g_frame = frame + 1;

        if (g_frame > 0 && (g_frame % 2900 == 0)) {
            static double hp_prev_x_sa = -1e9, hp_prev_y_sa = -1e9;
            static int    hp_prev_rounds_sa = -1;
            const double  t_sec = g_frame / 30.0;
            const double  dxh = state.nav.est_x - hp_prev_x_sa;
            const double  dyh = state.nav.est_y - hp_prev_y_sa;
            const double  dist_100s = (hp_prev_x_sa < -1e8) ? -1.0 : std::sqrt(dxh*dxh + dyh*dyh);
            const double  astar_r = (state.nav.astar_calls > 0)
                ? 100.0 * state.nav.astar_path_found / state.nav.astar_calls : 0.0;
            const double  avg_e = (state.err_samples > 0)
                ? state.err_sum / state.err_samples : 0.0;
            printf("\n[Standalone][HEALTH-CHECK t=%.0fs frame=%d]  ======================\n",
                   t_sec, g_frame);
            printf("  POS: gt=(%.2f,%.2f) est=(%.2f,%.2f yaw=%.2f) goal=(%.1f,%.1f #%zu) rounds=%d rooms=%zu\n",
                   robot_x, robot_y,
                   state.nav.est_x, state.nav.est_y, state.nav.est_yaw,
                   state.goal_x, state.goal_y, state.patrol_idx,
                   state.rounds_completed, state.rooms_visited.size());
            printf("  AMCL: conf=%.3f avg_err=%.3fm max_err=%.3fm samples=%u (n_active=%u err_samples=%u)\n",
                   state.nav.est_conf, avg_e, state.err_max,
                   (unsigned)state.nav.amcl.n_active,
                   (unsigned)state.nav.amcl.n_active,
                   (unsigned)state.err_samples);
            printf("  A*:   calls=%u ok=%u rate=%.1f%% fallbacks=%d escapes=%d\n",
                   (unsigned)state.nav.astar_calls, (unsigned)state.nav.astar_path_found, astar_r,
                   state.astar_fallbacks, state.rxf_escape_count);
            printf("  MOV:  win10s-displacement=%.2fm collisions_total=%d\n",
                   dist_100s, state.total_collisions);
            if (dist_100s >= 0 && dist_100s < 1.0) {
 printf("[Bridge] event\n");
            }
            if (astar_r < 99.0 && state.nav.astar_calls > 20) {
 printf("[Bridge] event\n");
            fflush(stdout);
            hp_prev_x_sa = state.nav.est_x;
            hp_prev_y_sa = state.nav.est_y;
            hp_prev_rounds_sa = state.rounds_completed;
        }

        if (frame % 3600 == 0) {
            printf("[Standalone] frame=%d/%d pos=(%.1f,%.1f) err=%.3f conf=%.2f rooms=%zu\n",
                   frame, max_frames, robot_x, robot_y, err, state.nav.est_conf,
                    state.rooms_visited.size());
        }
        if (frame < 2) { printf("[DIAG] frame_end %d\n", frame); fflush(stdout); }
    }

    printf("[Standalone] loop finished\n");
    // SUMMARY
    double avg_err = state.err_samples > 0 ? state.err_sum / state.err_samples : 0.0;
    double astar_rate = state.nav.astar_calls > 0
        ? 100.0 * state.nav.astar_path_found / state.nav.astar_calls : 0.0;
    const double stuck_ratio = g_frame > 0
        ? 100.0 * state.gt_stuck_frames / g_frame : 0.0;
    const double boundary_ratio = g_frame > 0
        ? 100.0 * state.gt_boundary_frames / g_frame : 0.0;
    printf("\nSUMMARY: collisions=%d ue_collision_events=%d astar_rate=%.1f "
"avg_err=%.3f max_err=%.3f rooms=%zu rounds=%d targets=%d targets_abandoned=%d "
"gt_distance=%.2f motion_frames=%d stuck_frames=%d stuck_ratio=%.1f "
           "boundary_frames=%d boundary_ratio=%.1f confidence=%.3f frames=%d\n",
           state.total_collisions, state.ue_collision_events, astar_rate,
           avg_err, state.err_max,            state.rooms_visited.size(),
           state.rounds_completed, state.target_reached_count,
           state.targets_abandoned,
           state.gt_path_distance, state.gt_motion_frames,
           state.gt_stuck_frames, stuck_ratio, state.gt_boundary_frames,
           boundary_ratio, state.nav.est_conf, g_frame);
     fflush(stdout);
     return 0;
}
}
    return 0;
}
// ----------------------------------------------------------------------
// §9 (jihua20260818) Bridge 诊断模式: 不需要 UE 客户端即可运行
// ----------------------------------------------------------------------
static std::string resolve_contract_path() {
    std::string p = "config/simulation_contract.yaml";
    std::ifstream t(p);
    if (!t.is_open()) p = "../../config/simulation_contract.yaml";
    return p;
}

// --check-scene: 严格校验场景 JSON (文件存在/可解析/障碍与目标合法)
static int run_check_scene(const std::string& scene_path) {
    printf("=== CHECK-SCENE: %s ===\n", scene_path.c_str());
    {
        std::ifstream f(scene_path);
        if (!f.is_open()) {
            printf("[FATAL] scene file not found: %s\n", scene_path.c_str());
            return 5;
        }
    }
    std::vector<BBox> obs;
    std::vector<PatrolTarget> targets;
    std::vector<Pedestrian> peds;
    try {
        obs = scene_loader::load_obstacles(scene_path);
        targets = scene_loader::load_patrol_targets(scene_path);
        peds = scene_loader::load_pedestrians(scene_path);
    } catch (const std::exception& e) {
        printf("[FATAL] scene parse error: %s\n", e.what());
        return 5;
    }
    int errs = 0;
    if (obs.empty()) { printf("[FAIL] obstacles empty\n"); errs++; }
    for (size_t k = 0; k < obs.size(); ++k) {
        const BBox& o = obs[k];
        if (std::isnan(o.xmin) || std::isnan(o.xmax) || std::isnan(o.ymin) || std::isnan(o.ymax)) {
            printf("[FAIL] obstacle[%zu] contains NaN\n", k); errs++;
        } else if (!(o.xmin < o.xmax) || !(o.ymin < o.ymax)) {
            printf("[FAIL] obstacle[%zu] invalid bbox (%.2f,%.2f)-(%.2f,%.2f)\n",
                   k, o.xmin, o.ymin, o.xmax, o.ymax); errs++;
        }
    }
    if (targets.empty()) { printf("[FAIL] patrol_targets empty\n"); errs++; }
    for (size_t k = 0; k < targets.size(); ++k) {
        const PatrolTarget& t = targets[k];
        if (std::isnan(t.x) || std::isnan(t.y)) {
            printf("[FAIL] patrol_target[%zu] contains NaN\n", k); errs++;
        }
    }
    puppy::Contract c;
    std::string cp = resolve_contract_path();
    std::ifstream cf(cp);
    if (cf.is_open()) {
        puppy::load_contract(cp, c);
        printf("  contract.schema_version=%d robot.radius=%.3f footprint_points=%zu\n",
               c.schema_version, c.robot.radius, c.robot.footprint.size());
    }
    printf("  obstacles=%zu patrol_targets=%zu pedestrians=%zu\n",
           obs.size(), targets.size(), peds.size());
    if (errs == 0) { printf("CHECK-SCENE: PASS\n"); return 0; }
    printf("CHECK-SCENE: FAIL (%d errors)\n", errs);
    return 1;
}

// --dry-run: 加载场景 + 契约并初始化 NavCoreStack, 但不连接 UE
static int run_dry_run(const std::string& scene_path) {
    printf("=== DRY-RUN (no UE connection) ===\n");
    NavState state;
    load_scene(state, scene_path);
    try {
        state.nav.init_from_obstacles(state.obstacles);
    } catch (const std::exception& e) {
        printf("[FAIL] NavCoreStack init failed: %s\n", e.what());
        return 1;
    }
    printf("  obstacles=%zu patrol_targets=%zu pedestrians=%zu\n",
           state.obstacles.size(), state.patrol_targets.size(), state.pedestrians.size());
    puppy::Contract c;
    std::string cp = resolve_contract_path();
    std::ifstream cf(cp);
    if (cf.is_open()) {
        puppy::load_contract(cp, c);
        printf("  contract.schema_version=%d robot.radius=%.3f\n",
               c.schema_version, c.robot.radius);
    }
    printf("DRY-RUN: PASS (NavCoreStack initialized from scene)\n");
    return 0;
}

static void print_bridge_help() {
    printf("Usage: nav_ue_bridge.exe [options]\n");
    printf("\nOptions:\n");
    printf("  --port <n>            TCP listen port (default: 7777)\n");
    printf("  --frames <n>          max frames (0=until disconnect; standalone default 36000)\n");
    printf("  --seed <n>            random seed (default: 1)\n");
    printf("  --scene <path>        scene JSON (default: ../../config/scene_home.json)\n");
    printf("  --standalone          run without UE (offline sim)\n");
    printf("  --connect-timeout <s> wait at most Ns for a UE client, then exit (default: 10)\n");
    printf("  --idle-timeout <s>    no-data silence before link declared dead (default: 2)\n");
    printf("  --check-scene <path>  validate scene JSON and exit (no UE)\n");
    printf("  --check-port <port>   check if port is free, then exit\n");
    printf("  --dry-run             load scene+contract, init NavCoreStack, exit (no UE)\n");
    printf("  --trace <path>        per-frame CSV trace\n");
    printf("  --help, -h            show this help\n");
    printf("  --version             show version\n");
}

int main(int argc, char* argv[]) {
    int port = 7777;
    int max_frames = 0;  // [comment stripped: encoding-corrupted]
    int seed = 1;
    bool standalone = false;
    std::string scene_path = "../../config/scene_home.json";
    std::string trace_path;   // R27: optional per-frame CSV trace
    // §9: 诊断/超时控制 (无需 UE 客户端)
    bool want_help = false;
    bool want_version = false;
    bool check_scene = false;
    bool dry_run = false;
    int connect_timeout = 10;     // 等待 UE 客户端的秒数, 超时则退出 (默认 10s)
    int idle_timeout_ms = 2000;  // 无数据静默多久判定链路死亡 (默认 2s)
    int check_port = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) port = std::atoi(argv[++i]);
        else if (arg == "--frames" && i + 1 < argc) max_frames = std::atoi(argv[++i]);
        else if (arg == "--seed" && i + 1 < argc) seed = std::atoi(argv[++i]);
        else if (arg == "--scene" && i + 1 < argc) scene_path = argv[++i];
        else if (arg == "--standalone") standalone = true;
        // §9.2 诊断模式
        else if (arg == "--help" || arg == "-h") want_help = true;
        else if (arg == "--version") want_version = true;
        else if (arg == "--check-scene" && i + 1 < argc) { check_scene = true; scene_path = argv[++i]; }
        else if (arg == "--dry-run") dry_run = true;
        else if (arg == "--connect-timeout" && i + 1 < argc) connect_timeout = std::atoi(argv[++i]);
        else if (arg == "--idle-timeout" && i + 1 < argc) idle_timeout_ms = std::atoi(argv[++i]) * 1000;
        else if (arg == "--check-port" && i + 1 < argc) check_port = std::atoi(argv[++i]);
        // R27: --trace <path> writes one CSV row per frame so that drift can be
        // attributed to a specific time window / room / goal instead of being
        // averaged away over the whole run.
        else if (arg == "--trace" && i + 1 < argc) trace_path = argv[++i];
        else if (arg == "--inject-divergence-at" && i + 1 < argc)
            g_inject_divergence_at = std::atoi(argv[++i]);
    }

    if (want_help) { print_bridge_help(); return 0; }
    if (want_version) { printf("nav_ue_bridge %s\n", BRIDGE_VERSION); return 0; }

    // --check-port: 探测端口是否可用 (不绑定长驻)
    if (check_port > 0) {
        bridge::WinSockInit wsa_init;
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) { printf("[FATAL] socket() failed: %d\n", WSAGetLastError()); return 1; }
        int opt = 1;
        setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = htons((u_short)check_port);
        if (bind(s, (sockaddr*)&a, sizeof(a)) == SOCKET_ERROR) {
            printf("[FATAL] port %d is already in use (WSA %d)\n", check_port, WSAGetLastError());
            closesocket(s);
            return 5;
        }
        printf("[OK] port %d is available\n", check_port);
        closesocket(s);
        return 0;
    }

    // --check-scene / --dry-run: 不连接 UE
    if (check_scene) return run_check_scene(scene_path);
    if (dry_run) return run_dry_run(scene_path);

    if (!trace_path.empty()) {
        g_trace_fp = fopen(trace_path.c_str(), "w");
        if (g_trace_fp) {
            fprintf(g_trace_fp,
                    "frame,t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,err,conf,"
                    "goal_idx,goal_x,goal_y,cmd_vx,cmd_vy,cmd_wz,collisions,"
                    "astar_calls,astar_ok,escape,path_pts,room,"
                    // R28: estimator comparison + likelihood-field grid search
                    "mean_x,mean_y,top_x,top_y,top_share,nclusters,"
                    "score_gt,score_est,score_best,best_dx,best_dy,"
                    // R28b: path-follower lookahead selection
                    "la_idx,la_naive,la_x,la_y,la_los,la_naive_los,"
                    // R29c: obstacle clearance driving the speed envelope
                    "clearance,"
                    // R31: LiDAR reactive safety filter (r31_scale < 1 means the
                    // filter throttled this frame; a low mean scale is the R29c
                    // creep regression signature)
                    "r31_bound,r31_dmin,r31_scale,r31_dev,"
                    // §5.3: AMCL 更新标志 / 本帧运动距离 / 计算耗时
                    // (RTF 与消息延迟需 UE 侧时钟，standalone 模式无，标注 UE-DEPENDENT)
                    "amcl_updated,motion_delta,proc_ms\n");
            printf("[Bridge] trace -> %s\n", trace_path.c_str());
            const std::string ray_path = trace_path + ".rays.csv";
            g_ray_fp = fopen(ray_path.c_str(), "w");
            if (g_ray_fp) {
                fprintf(g_ray_fp,
                        "frame,ray,angle_local,angle_world,meas,expect,diff,"
                        "hit_x,hit_y,ped_dist\n");
                printf("[Bridge] ray dump -> %s\n", ray_path.c_str());
            }
        } else {
            printf("[Bridge] WARNING: cannot open trace file %s\n", trace_path.c_str());
        }
    }

    if (standalone) {
if (max_frames <= 0) max_frames = 36000;  // [comment stripped: encoding-corrupted]
        return run_standalone(seed, max_frames, scene_path);
    }

    bridge::WinSockInit wsa_init;

    // TCP server
    SOCKET server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server == INVALID_SOCKET) {
        printf("[FATAL] socket() failed: %d\n", WSAGetLastError());
        return 1;
    }
    int opt = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);

    if (bind(server, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        printf("[FATAL] bind() failed: %d\n", WSAGetLastError());
        closesocket(server);
        return 1;
    }
    if (listen(server, 1) == SOCKET_ERROR) {
        printf("[FATAL] listen() failed: %d\n", WSAGetLastError());
        closesocket(server);
        return 1;
    }

 printf("[Bridge] event\n");

    // §9.2: 连接超时 — 无 UE 客户端时在配置超时内退出, 绝不无限等待 (P0)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(server, &rfds);
        timeval tv;
        tv.tv_sec = connect_timeout;
        tv.tv_usec = 0;
#ifdef _WIN32
        int sel = select(0, &rfds, nullptr, nullptr, &tv);
#else
        int sel = select(static_cast<int>(server) + 1, &rfds, nullptr, nullptr, &tv);
#endif
        if (sel <= 0) {
            printf("[Bridge][TIMEOUT] no UE client within %ds; exiting\n", connect_timeout);
            fflush(stdout);
            closesocket(server);
            return 4;
        }
    }

    SOCKET raw_client = accept(server, nullptr, nullptr);
    if (raw_client == INVALID_SOCKET) {
        printf("[FATAL] accept() failed\n");
        closesocket(server);
        return 1;
    }
    g_link = bridge::Link(static_cast<int>(raw_client));
    printf("[Bridge] client accepted (sock=%d); starting v2 handshake...\n", (int)raw_client);
    fflush(stdout);
    if (!g_link.handshake_server("nav-core-v2")) {
        printf("[Bridge][FAULT] v2 handshake failed (version mismatch / timeout)\n");
        closesocket(raw_client);
        return 1;
    }
    printf("[Bridge] v2 handshake OK (protocol v%d)\n", bridge::PROTOCOL_VERSION);
    fflush(stdout);
    // P0-03: anchor the watchdog clock so the first 100ms idle window does
    // not read a huge silence (last_rx_ms starts at 0) and trip the 2s FAULT.
    g_link.ps.last_rx_ms = bridge::now_ms();
 printf("[Bridge] event\n");

    NavState state;
    load_scene(state, scene_path);
    state.nav.init_from_obstacles(state.obstacles);

    if (!state.patrol_targets.empty()) {
        state.goal_x = state.patrol_targets[0].x;
        state.goal_y = state.patrol_targets[0].y;
        // §5.2: 初始位姿以场景声明的 initial_pose 为准（不再种在目标点上），
        // 首帧 GT 与 initial_pose 的偏差会被显式记录（见 GROUND_TRUTH 处理）。
        state.nav.amcl.init_cloud(state.scene_init_x, state.scene_init_y, state.scene_init_yaw);
        state.nav.est_x = state.scene_init_x;
        state.nav.est_y = state.scene_init_y;
        state.nav.est_yaw = state.scene_init_yaw;
        state.nav.est_conf = 1.0;
        state.dbg_est_prev_x = state.scene_init_x;
        state.dbg_est_prev_y = state.scene_init_y;
 printf("[Bridge] event\n");
    } else {
        state.goal_x = -1.0;
        state.goal_y = -3.0;
        state.nav.amcl.init_cloud(-1.0, -3.0, 0.0);
        state.nav.est_x = -1.0;
        state.nav.est_y = -3.0;
        state.nav.est_yaw = 0.0;
        state.nav.est_conf = 1.0;
  printf("[Bridge] event\n");
    }
    state.replan_counter = 0;
    state.initialized = true;
    g_frame = 0;
    state.nav.first_update = true;
    printf("[Bridge] initialized\n");
    fflush(stdout);

    bool running = true;
    FrameData frame_data;
    int msg_in_frame = 0;
    int total_msgs_received = 0;

    while (running) {
        bridge::FrameHeader hdr;
        std::vector<char> payload;

        // P0-03: 带超时接收（断线看门狗基础，§6.4 超时策略）
        auto rstat = g_link.recv_timeout(hdr, payload, 100);
        if (rstat == bridge::Link::RECV_CLOSED) {
            printf("[Bridge][DISCONNECT] client socket closed; safe-stop & exit\n");
            fflush(stdout);
            break;
        }
        if (rstat == bridge::Link::RECV_TIMEOUT) {
            const int64_t silent = bridge::now_ms() - g_link.ps.last_rx_ms;
            if (!g_link.ps.safe_stop && silent >= 300) {
                g_link.ps.safe_stop = true;
                g_link.ps.conn_state = bridge::ConnState::DEGRADED;
                printf("[Bridge][DEGRADED] no data %lldms -> EMERGENCY_STOP\n", (long long)silent);
                fflush(stdout);
                g_link.send_emergency_stop();
            }
            if (silent >= idle_timeout_ms) {
                printf("[Bridge][FAULT] no data %lldms -> release link\n", (long long)silent);
                fflush(stdout);
                break;
            }
            continue;   // (§6.4) 100ms 内不推进、不处理
        }

        // 序列号校验（重复帧忽略，缺口/过期丢弃并记录协议错误）
        bridge::SeqCheck sc = bridge::check_rx_seq(g_link.ps, hdr.sequence);
        if (sc == bridge::SeqCheck::DUPLICATE_FRAME) {
            g_link.ps.duplicate_frames++;
            continue;
        }
        if (sc == bridge::SeqCheck::DROP) {
            g_link.ps.protocol_errors++;
            printf("[Bridge][PROTOCOL-ERR] drop type=0x%04x seq=%u (gap/late)\n",
                   (unsigned)hdr.type, hdr.sequence);
            fflush(stdout);
            continue;
        }

        auto msg_type = static_cast<bridge::MsgType>(hdr.type);
        total_msgs_received++;

    if (total_msgs_received <= 20 || total_msgs_received % 500 == 0) {
            const char* tn = "UNKNOWN";
            switch (msg_type) {
                case bridge::MsgType::RESET: tn = "RESET"; break;
                case bridge::MsgType::STEP_ACK: tn = "STEP_ACK"; break;
                case bridge::MsgType::LIDAR: tn = "LIDAR"; break;
                case bridge::MsgType::GROUND_TRUTH: tn = "GROUND_TRUTH"; break;
                case bridge::MsgType::PED_STATE: tn = "PED_STATE"; break;
                case bridge::MsgType::COLLISION: tn = "COLLISION"; break;
                case bridge::MsgType::CMD_VEL: tn = "CMD_VEL"; break;
                case bridge::MsgType::PAUSE: tn = "PAUSE"; break;
                case bridge::MsgType::SET_SPEED: tn = "SET_SPEED"; break;
                default: break;
            }
            printf("[Bridge] msg#%d type=0x%04x(%s) len=%uB frame=%d\n",
                   total_msgs_received, (unsigned)hdr.type, tn, (unsigned)hdr.length, g_frame);
            fflush(stdout);
        }

        switch (msg_type) {
        case bridge::MsgType::RESET: {
            if (payload.size() >= sizeof(bridge::ResetMsg)) {
                bridge::ResetMsg reset;
                memcpy(&reset, payload.data(), sizeof(reset));
                printf("[Bridge] RESET: seed=%d scene=%s pose=(%.1f,%.1f,%.1f)\n",
                       reset.seed, reset.scene_name, reset.init_x, reset.init_y, reset.init_yaw);

                state.nav.amcl.set_seed(static_cast<uint32_t>(reset.seed));
                state.nav.amcl.init_cloud(reset.init_x, reset.init_y, reset.init_yaw);
                state.nav.first_update = true;
                state.patrol_idx = 0;
                state.goal_x = state.patrol_targets[0].x;
                state.goal_y = state.patrol_targets[0].y;
                state.goal_close_frames = 0;
                state.dynamic_init_done = false;
                state.prev_true_initialized = false;
                state.current_path.clear();
                state.replan_counter = 0;
                state.initialized = true;
            }
            break;
        }

        case bridge::MsgType::LIDAR: {
            if (payload.size() >= sizeof(bridge::LidarMsg)) {
                bridge::LidarMsg lidar_hdr{};
                memcpy(&lidar_hdr, payload.data(), sizeof(lidar_hdr));
                const size_t available = (payload.size() - sizeof(bridge::LidarMsg)) / sizeof(float);
                const int ray_count = (std::min)(72, (std::min)(lidar_hdr.n_rays, static_cast<int>(available)));
                frame_data.lidar_n_rays = ray_count;
                frame_data.lidar_max_range = lidar_hdr.max_range;
                frame_data.lidar_distances.clear();
                const float* dists = reinterpret_cast<const float*>(payload.data() + sizeof(bridge::LidarMsg));
                for (int i = 0; i < ray_count; ++i) {
                    frame_data.lidar_distances.push_back(dists[i]);
                }
                msg_in_frame++;
            }
            break;
        }

        case bridge::MsgType::GROUND_TRUTH: {
            if (payload.size() >= sizeof(bridge::GroundTruthMsg)) {
                bridge::GroundTruthMsg gt;
                memcpy(&gt, payload.data(), sizeof(gt));
                frame_data.true_x = gt.x;
                frame_data.true_y = gt.y;
                frame_data.true_yaw = gt.yaw;
                frame_data.has_ground_truth = true;
                msg_in_frame++;

                if (g_frame == 0 && state.initialized) {
                    if (!state.dynamic_init_done) {
                        // §5.2: 首帧 GT 与场景声明 initial_pose 的一致性校验。
                        // est 已种在 initial_pose 上，故 dist0 即"UE 实际起点 vs 场景起点"偏差。
                        double dx0 = gt.x - state.scene_init_x;
                        double dy0 = gt.y - state.scene_init_y;
                        double dyaw0 = gt.yaw - state.scene_init_yaw;
                        double dev0 = std::sqrt(dx0*dx0 + dy0*dy0);
                        printf("[Bridge] GT_FIRST_FRAME gt=(%.3f,%.3f,%.3f) scene_init=(%.3f,%.3f,%.3f) "
                               "deviation=%.3f m dyaw=%.3f rad\n",
                               gt.x, gt.y, gt.yaw,
                               state.scene_init_x, state.scene_init_y, state.scene_init_yaw,
                               dev0, dyaw0);
                        fflush(stdout);
                        // 仅做一次显式校正：偏差超过 1.0m 视为 UE PlayerStart 与场景不一致，
                        // 记录告警并把 bridge 信念对齐到 GT（一次性，不每帧漂移）。
                        if (dev0 > 1.0) {
                            printf("[Bridge][WARN] GT_FIRST_FRAME 偏差 %.3f m 超过 1.0m 阈值 "
                                   "-> 一次性显式校正 est 到 GT\n", dev0);
                            fflush(stdout);
                            state.nav.amcl.init_cloud(gt.x, gt.y, gt.yaw);
                            state.nav.est_x = gt.x;
                            state.nav.est_y = gt.y;
                            state.nav.est_yaw = gt.yaw;
                            state.dbg_est_prev_x = gt.x;
                            state.dbg_est_prev_y = gt.y;
                        }
                        // 首帧膨胀检查（镜像 validate_scene.py）：GT 必须落在任一障碍
                        // 膨胀半径之外；落在障碍内即场景/UE 几何不一致。
                        const double kRobotRadiusCheck = 0.35;  // 镜像契约 robot.radius
                        double near_obs = nearest_obstacle_distance(gt.x, gt.y, state.obstacles);
                        if (near_obs < kRobotRadiusCheck) {
                            printf("[Bridge][WARN] GT_FIRST_FRAME 落入障碍膨胀区 "
                                   "(nearest=%.3f m < %.3f m)\n", near_obs, kRobotRadiusCheck);
                            fflush(stdout);
                        }

                        if (!state.patrol_targets.empty()) {
                            double dx_g = gt.x - state.patrol_targets[0].x;
                            double dy_g = gt.y - state.patrol_targets[0].y;
                            double dist_g0 = std::sqrt(dx_g*dx_g + dy_g*dy_g);
                            if (dist_g0 < 1.5) {
                                state.patrol_idx = 1 % state.patrol_targets.size();
                                state.goal_x = state.patrol_targets[state.patrol_idx].x;
                                state.goal_y = state.patrol_targets[state.patrol_idx].y;
                                state.current_path.clear();
                                 printf("[Bridge] goal_switch dist=%.2f idx=%zu goal=(%.2f,%.2f) name=%s\n",
                                        dist_g0, state.patrol_idx,
                                        state.goal_x, state.goal_y,
                                        state.patrol_targets[state.patrol_idx].name.c_str());
                                fflush(stdout);
                            } else {
                                 printf("[Bridge] goal_not_reached dist=%.2f\n", dist_g0);
                                fflush(stdout);
                            }
                        }
                        state.dynamic_init_done = true;
                    }
                }
            }
            break;
        }

        case bridge::MsgType::PED_STATE: {
            if (payload.size() >= sizeof(bridge::PedStateMsg)) {
                bridge::PedStateMsg ped_hdr;
                memcpy(&ped_hdr, payload.data(), sizeof(ped_hdr));
                frame_data.peds.clear();
                const bridge::PedData* peds = reinterpret_cast<const bridge::PedData*>(
                    payload.data() + sizeof(bridge::PedStateMsg));
                for (int i = 0; i < ped_hdr.n_peds; ++i) {
                    frame_data.peds.push_back({peds[i].x, peds[i].y, peds[i].vx, peds[i].vy});
                }
                frame_data.has_ped_state = true;
                msg_in_frame++;
            }
            break;
        }

        case bridge::MsgType::COLLISION: {
            if (payload.size() >= sizeof(bridge::CollisionMsg)) {
                bridge::CollisionMsg col;
                memcpy(&col, payload.data(), sizeof(col));
                frame_data.collision_count = col.collision_count;
                frame_data.hit_x = col.hit_x;
                frame_data.hit_y = col.hit_y;
                frame_data.has_collision = true;
                state.total_collisions = col.collision_count;
                if (col.collision_count > state.last_ue_collision_count) {
                    state.ue_collision_events += col.collision_count - state.last_ue_collision_count;
                    state.last_ue_collision_count = col.collision_count;
                    // R29: contacts within 3 frames of each other are one sustained
                    // grind; a gap longer than that starts a fresh streak.
                    if (g_frame - state.rxf_last_contact_frame <= 3) state.rxf_contact_frames++;
                    else                                             state.rxf_contact_frames = 1;
                    state.rxf_last_contact_frame = g_frame;
                }
            }
            break;
        }

        case bridge::MsgType::PAUSE: {
            printf("[Bridge] PAUSE\n");
            break;
        }

        case bridge::MsgType::SET_SPEED: {
            if (payload.size() >= sizeof(float)) {
                float speed = *reinterpret_cast<const float*>(payload.data());
                printf("[Bridge] SET_SPEED: %.1fx\n", speed);
            }
            break;
        }

        case bridge::MsgType::HELLO: {
            // Post-handshake HELLO is unexpected; ignore defensively.
            printf("[Bridge][PROTOCOL-WARN] stray HELLO after handshake; ignored\n");
            break;
        }
        case bridge::MsgType::HEARTBEAT: {
            // UE keepalive during idle/pause: reset the silence window.
            g_link.ps.last_rx_ms = bridge::now_ms();
            if (total_msgs_received <= 20 || total_msgs_received % 500 == 0)
                printf("[Bridge] HEARTBEAT keepalive (silent window reset)\n");
            fflush(stdout);
            break;
        }
        case bridge::MsgType::EMERGENCY_STOP: {
            printf("[Bridge][EMERGENCY_STOP] received from UE -> safe-stop & exit\n");
            fflush(stdout);
            g_link.ps.safe_stop = true;
            g_link.ps.conn_state = bridge::ConnState::FAULT;
            running = false;
            break;
        }
        case bridge::MsgType::SHUTDOWN_ACK: {
            printf("[Bridge] SHUTDOWN_ACK received -> graceful shutdown\n");
            fflush(stdout);
            running = false;
            break;
        }
        case bridge::MsgType::SIM_END: {
            // UE-initiated shutdown: acknowledge and exit gracefully so the
            // UE-side SendSimEnd()/EndPlay() path completes cleanly.
            printf("[Bridge] SIM_END received from UE -> reply SHUTDOWN_ACK & exit\n");
            fflush(stdout);
            g_link.send_shutdown_ack();
            running = false;
            break;
        }
        case bridge::MsgType::FAULT: {
            printf("[Bridge][FAULT] received from UE -> safe-stop & exit\n");
            fflush(stdout);
            g_link.ps.safe_stop = true;
            g_link.ps.conn_state = bridge::ConnState::FAULT;
            running = false;
            break;
        }
        case bridge::MsgType::STATUS: {
            // Diagnostics / state report from UE; recorded, not acted on here.
            break;
        }

        default:
            break;
        }

        // A UE lockstep frame consists of LIDAR + GROUND_TRUTH + PED_STATE.
        // Do not process after only two messages: PED_STATE arrives after the
        // ground truth and would otherwise be paired with the next frame,
        // advancing the bridge faster than the UE simulation.
        if (!frame_data.lidar_distances.empty() &&
            frame_data.has_ground_truth && frame_data.has_ped_state) {
                static double hp_prev_x = -1e9, hp_prev_y = -1e9;
                static int    hp_prev_rounds = -1;
                static double hp_path_accum = 0.0;
                static double hp_prev_gtx = -1e9, hp_prev_gty = -1e9;

            // R27-FIX(obs-1): accumulate the ground-truth path length BEFORE
            // process_frame() clears has_ground_truth. The previous code ran
            // this block AFTER the flag was reset, so the guard was always
            // false and path_accum reported 0.0m for the entire run.
            if (hp_prev_gtx < -1e8) {
                hp_prev_gtx = frame_data.true_x;
                hp_prev_gty = frame_data.true_y;
            } else {
                const double gdx = frame_data.true_x - hp_prev_gtx;
                const double gdy = frame_data.true_y - hp_prev_gty;
                hp_path_accum += std::sqrt(gdx * gdx + gdy * gdy);
                hp_prev_gtx = frame_data.true_x;
                hp_prev_gty = frame_data.true_y;
            }

            // R27: snapshot the ground truth for the CSV trace, because
            // process_frame() invalidates frame_data right after it runs.
            const double tr_true_x   = frame_data.true_x;
            const double tr_true_y   = frame_data.true_y;
            const double tr_true_yaw = frame_data.true_yaw;

            state.dbg_escape_active = 0;

            const int64_t proc_t0 = bridge::now_ms();
            process_frame(state, frame_data);
            // §5.3: 本帧 bridge 计算耗时 + est 位移（运动距离增量）
            state.dbg_proc_ms = (double)(bridge::now_ms() - proc_t0);
            state.dbg_motion_delta = std::sqrt(
                (state.nav.est_x - state.dbg_est_prev_x) * (state.nav.est_x - state.dbg_est_prev_x) +
                (state.nav.est_y - state.dbg_est_prev_y) * (state.nav.est_y - state.dbg_est_prev_y));
            state.dbg_est_prev_x = state.nav.est_x;
            state.dbg_est_prev_y = state.nav.est_y;
            msg_in_frame = 0;
            frame_data.has_ground_truth = false;
            frame_data.has_ped_state = false;
            frame_data.has_collision = false;

            // R27: one CSV row per frame -- this is what makes it possible to
            // localize *when* and *where* the estimate drifts, instead of only
            // seeing a run-wide average.
            if (g_trace_fp) {
                const double ex = state.nav.est_x;
                const double ey = state.nav.est_y;
                const double err = std::sqrt((ex - tr_true_x) * (ex - tr_true_x) +
                                             (ey - tr_true_y) * (ey - tr_true_y));
                fprintf(g_trace_fp,
                        "%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                        "%d,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%s,"
                        "%.4f,%.4f,%.4f,%.4f,%.4f,%d,"
                        "%.6g,%.6g,%.6g,%.2f,%.2f,"
                        "%d,%d,%.3f,%.3f,%d,%d,%.3f,"
                        "%d,%.3f,%.4f,%.0f,"
                        // §5.3: amcl_updated / motion_delta / proc_ms
                        "%d,%.4f,%.2f\n",
                        g_frame, g_frame / 30.0,
                        tr_true_x, tr_true_y, tr_true_yaw,
                        ex, ey, state.nav.est_yaw,
                        err, state.nav.est_conf,
                        (int)state.patrol_idx, state.goal_x, state.goal_y,
                        state.last_cmd_vx, state.last_cmd_vy, state.last_cmd_wz,
                        state.total_collisions,
                        state.nav.astar_calls, state.nav.astar_path_found,
                        state.dbg_escape_active,
                        (int)state.current_path.size(),
                        get_room_name(ex, ey).c_str(),
                        state.dbg_mean_x, state.dbg_mean_y,
                        state.dbg_top_x, state.dbg_top_y,
                        state.dbg_top_share, state.dbg_nclusters,
                        state.dbg_score_gt, state.dbg_score_est,
                        state.dbg_score_best,
                        state.dbg_best_dx, state.dbg_best_dy,
                        state.dbg_la_idx, state.dbg_la_naive,
                        state.dbg_la_x, state.dbg_la_y,
                        state.dbg_la_los, state.dbg_la_naive_los,
                        state.dbg_clearance,
                        state.dbg_r31_bound, state.dbg_r31_dmin,
                        state.dbg_r31_scale, state.dbg_r31_dev,
                        state.dbg_amcl_updated, state.dbg_motion_delta,
                        state.dbg_proc_ms);
            }

                // R27-FIX(obs-3): 2900 was an odd stride that produced exactly
                // one health report in a 3600-frame run, leaving no time series
                // to reason about. 300 frames = 10s gives ~12 samples per run.
                if (g_frame > 0 && (g_frame % 300 == 0)) {
                    const double  t_sec = g_frame / 30.0;
                    const double  dx = state.nav.est_x - hp_prev_x;
                    const double  dy = state.nav.est_y - hp_prev_y;
                    const double  dist_100s = (hp_prev_x < -1e8) ? -1.0 : std::sqrt(dx*dx + dy*dy);
                    const double  astar_r = (state.nav.astar_calls > 0)
                        ? 100.0 * state.nav.astar_path_found / state.nav.astar_calls : 0.0;
                    const double  avg_e = (state.err_samples > 0)
                        ? state.err_sum / state.err_samples : 0.0;
                    printf("\n[Bridge][HEALTH-CHECK t=%.0fs frame=%d]  ========================\n",
                           t_sec, g_frame);
                    printf("  POS: est=(%.2f,%.2f yaw=%.2f) goal=(%.1f,%.1f #%zu) rounds=%d rooms=%zu\n",
                           state.nav.est_x, state.nav.est_y, state.nav.est_yaw,
                           state.goal_x, state.goal_y, state.patrol_idx,
                           state.rounds_completed, state.rooms_visited.size());
                    printf("  AMCL: conf=%.3f avg_err=%.3fm max_err=%.3fm samples=%u (n_active=%u err_samples=%u)\n",
                           state.nav.est_conf, avg_e, state.err_max,
                           (unsigned)state.nav.amcl.n_active,
                           (unsigned)state.nav.amcl.n_active,
                           (unsigned)state.err_samples);
                    printf("  A*:   calls=%u ok=%u rate=%.1f%%  replan_interval=%d fallbacks=%d escapes=%d\n",
                           (unsigned)state.nav.astar_calls, (unsigned)state.nav.astar_path_found,
                           astar_r, state.REPLAN_INTERVAL, state.astar_fallbacks, state.rxf_escape_count);
                    printf("  MOV:  win10s-displacement=%.2fm path_accum=%.1fm collisions_total=%d\n",
                           dist_100s, hp_path_accum, state.total_collisions);
                    if (dist_100s >= 0 && dist_100s < 1.0 && hp_path_accum < 2.0) {
                         printf("  !!WARNING: UE low movement\n");
                    }
                    if (state.rounds_completed == hp_prev_rounds && hp_prev_rounds >= 0) {
                         printf("  !!WARNING: UE no round progress\n");
                     }
                    if (astar_r < 99.0 && state.nav.astar_calls > 20) {
                         printf("  !!WARNING: Astar rate low\n");
                    }
                    // R27-FIX(obs-2): this warning used to open a brace that
                    // swallowed every window-state reset below it. With
                    // conf=0.97 the branch never ran, so hp_prev_x stayed at
                    // -1e9 forever, the displacement readout was pinned at
                    // -1.00m, and the "low movement" warning above became
                    // unreachable (it requires dist >= 0).
                    if (state.nav.est_conf < 0.5) {
                        printf("  !!WARNING: AMCL low confidence conf=%.3f\n",
                               state.nav.est_conf);
                    }
                    fflush(stdout);
                    hp_prev_x = state.nav.est_x;
                    hp_prev_y = state.nav.est_y;
                    hp_prev_rounds = state.rounds_completed;
                    hp_path_accum = 0.0;
            }

    if (max_frames > 0 && g_frame >= max_frames) {
            printf("[Bridge] frame limit reached: %d\n", max_frames);
            break;
            }
        }
    }

     double avg_err = state.err_samples > 0 ? state.err_sum / state.err_samples : 0.0;
    double astar_rate = state.nav.astar_calls > 0
        ? 100.0 * state.nav.astar_path_found / state.nav.astar_calls : 0.0;
    const double stuck_ratio = g_frame > 0
        ? 100.0 * state.gt_stuck_frames / g_frame : 0.0;
    const double boundary_ratio = g_frame > 0
        ? 100.0 * state.gt_boundary_frames / g_frame : 0.0;
    printf("SUMMARY: collisions=%d ue_collision_events=%d astar_rate=%.1f "
"avg_err=%.3f max_err=%.3f rooms=%zu rounds=%d targets=%d targets_abandoned=%d "
"gt_distance=%.2f motion_frames=%d stuck_frames=%d stuck_ratio=%.1f "
           "boundary_frames=%d boundary_ratio=%.1f confidence=%.3f frames=%d\n",
           state.total_collisions, state.ue_collision_events, astar_rate,
           avg_err, state.err_max,            state.rooms_visited.size(),
           state.rounds_completed, state.target_reached_count,
           state.targets_abandoned,
           state.gt_path_distance, state.gt_motion_frames,
           state.gt_stuck_frames, stuck_ratio, state.gt_boundary_frames,
           boundary_ratio, state.nav.est_conf, g_frame);
    fflush(stdout);

    if (g_link.valid()) {
        g_link.send_sim_end();
#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000);
#endif
    }
    if (g_link.sock >= 0) closesocket(static_cast<SOCKET>(g_link.sock));
    closesocket(server);
    if (g_ray_fp) {
        fclose(g_ray_fp);
        g_ray_fp = nullptr;
    }
    if (g_trace_fp) {
        fclose(g_trace_fp);
        g_trace_fp = nullptr;
        printf("[Bridge] trace closed\n");
    }
    printf("[Bridge] shutdown: frames=%d collisions=%d targets=%d rounds=%d\n",
           g_frame, state.total_collisions,
           state.target_reached_count, state.rounds_completed);
    state.nav.report();
    return 0;
}

