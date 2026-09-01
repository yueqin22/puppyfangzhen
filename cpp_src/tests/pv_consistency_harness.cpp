// pv_consistency_harness.cpp — M2 §6.3 Python/C++ field-level diff harness.
// Builds a set of scenarios, runs the C++ PathValidator, and writes the SAME
// inputs (grid + path + options) together with the C++ result to a JSON file.
// The Python test (nav_core/validation/test_consistency.py) reads this file,
// reconstructs the identical grid, runs the Python validator, and field-level
// diffs its output against cpp_result.
//
// Build (Windows / MSVC, no cmd.exe):
//   source scripts/msvc_env.sh
//   cl.exe /O2 /std:c++17 /EHsc /utf-8 /MT /D_CRT_SECURE_NO_WARNINGS /DNOMINMAX
//     /Icpp_src/puppy_nav_core/include /Icpp_src/tests
//     pv_consistency_harness.cpp occupancy_grid.cpp costmap.cpp astar_planner.cpp
//     amcl.cpp path_validator.cpp
//     /Fe build_pv/pv_consistency_harness.exe
// Run:  build_pv/pv_consistency_harness.exe build_pv/pv_consistency.json
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <utility>
#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/path_validator.h"

using namespace puppy_nav_core;

static const std::vector<Vec2> kFP = {
    {-0.35, -0.25}, {-0.35, 0.25}, {0.35, 0.25}, {0.35, -0.25}
};

static void fill_free(OccupancyGrid& g) {
    g.log_odds.assign((size_t)g.width * g.height, -1.0f);
}
static void fill_unknown(OccupancyGrid& g) {
    g.log_odds.assign((size_t)g.width * g.height, 0.0f);
}
static OccupancyGrid make_corridor(double dw) {
    OccupancyGrid g; fill_free(g);
    double half = dw / 2.0;
    for (int gy = 0; gy < g.height; ++gy)
        for (int gx = 0; gx < g.width; ++gx) {
            double wx, wy; g.grid_to_world(gx, gy, wx, wy);
            if (std::abs(wy) < 0.10 && std::abs(wx) > half)
                g.log_odds[(size_t)gy * g.width + gx] = 1.0f;
        }
    return g;
}
static PathValidationOptions make_opts() {
    PathValidationOptions o;
    o.footprint = kFP;
    o.min_path_clearance = 0.08;
    o.goal_tolerance = 0.35;
    o.sample_step = 0.025;
    o.unknown_is_blocked = true;
    return o;
}

static void emit_grid(FILE* f, const OccupancyGrid& g) {
    fprintf(f, "\"grid\":{\"width\":%d,\"height\":%d,\"resolution\":%g,"
                "\"origin_x\":%g,\"origin_y\":%g,\"log_odds\":[",
            g.width, g.height, g.resolution, g.origin_x, g.origin_y);
    for (size_t i = 0; i < g.log_odds.size(); ++i) {
        if (i) fputc(',', f);
        fprintf(f, "%.3f", g.log_odds[i]);
    }
    fprintf(f, "]}");
}
static void emit_path(FILE* f, const std::vector<std::pair<double, double>>& p) {
    fprintf(f, "[");
    for (size_t i = 0; i < p.size(); ++i) {
        if (i) fprintf(f, ",");
        fprintf(f, "[%.6f,%.6f]", p[i].first, p[i].second);
    }
    fprintf(f, "]");
}
static void emit_opts(FILE* f, const PathValidationOptions& o) {
    fprintf(f, "\"options\":{\"min_path_clearance\":%g,\"goal_tolerance\":%g,"
                "\"sample_step\":%g,\"unknown_is_blocked\":%s,"
                "\"min_path_length_ratio\":%g,\"footprint\":[",
            o.min_path_clearance, o.goal_tolerance, o.sample_step,
            o.unknown_is_blocked ? "true" : "false", o.min_path_length_ratio);
    for (size_t i = 0; i < o.footprint.size(); ++i) {
        if (i) fprintf(f, ",");
        fprintf(f, "[%.6f,%.6f]", o.footprint[i].x, o.footprint[i].y);
    }
    fprintf(f, "]}");
}

struct Scenario {
    std::string name;
    OccupancyGrid g;
    std::vector<std::pair<double, double>> path;
    double sx, sy, gx, gy;
};

int main(int argc, char** argv) {
    const char* out = (argc > 1) ? argv[1] : "pv_consistency.json";
    FILE* f = fopen(out, "w");
    if (!f) { fprintf(stderr, "cannot open %s\n", out); return 2; }

    auto opts = make_opts();
    std::vector<Scenario> scs;

    // 1) empty path
    { Scenario s; s.name = "empty_path"; s.g = OccupancyGrid(); fill_free(s.g);
      s.path = {}; s.sx = 0; s.sy = 0; s.gx = 5; s.gy = 5; scs.push_back(s); }
    // 2) single point far from goal
    { Scenario s; s.name = "single_point_far"; s.g = OccupancyGrid(); fill_free(s.g);
      s.path = {{0, 0}}; s.sx = 0; s.sy = 0; s.gx = 5; s.gy = 5; scs.push_back(s); }
    // 3) start in obstacle
    { Scenario s; s.name = "start_in_obstacle"; s.g = OccupancyGrid(); fill_free(s.g);
      s.g.log_odds[10 * s.g.width + 10] = 1.0f;
      double bx, by; s.g.grid_to_world(10, 10, bx, by);
      s.path = {{bx + 1.0, by}}; s.sx = bx; s.sy = by; s.gx = bx + 2.0; s.gy = by;
      scs.push_back(s); }
    // 4) goal in obstacle
    { Scenario s; s.name = "goal_in_obstacle"; s.g = OccupancyGrid(); fill_free(s.g);
      double bx, by; s.g.grid_to_world(20, 20, bx, by);
      s.g.log_odds[20 * s.g.width + 20] = 1.0f;
      s.path = {{bx - 1.0, by}}; s.sx = 0; s.sy = 0; s.gx = bx; s.gy = by;
      scs.push_back(s); }
    // 5) start out of bounds
    { Scenario s; s.name = "start_out_of_bounds"; s.g = OccupancyGrid(); fill_free(s.g);
      s.path = {{0, 0}}; s.sx = 9999; s.sy = 9999; s.gx = 0; s.gy = 0; scs.push_back(s); }
    // 6) goal out of bounds
    { Scenario s; s.name = "goal_out_of_bounds"; s.g = OccupancyGrid(); fill_free(s.g);
      s.path = {{0, 0}}; s.sx = 0; s.sy = 0; s.gx = 9999; s.gy = 9999; scs.push_back(s); }
    // 7) corridor pass (1.2 m)
    { Scenario s; s.name = "corridor_pass_1_2"; s.g = make_corridor(1.2);
      s.path = {{0, 0}, {0, 2}}; s.sx = 0; s.sy = -2; s.gx = 0; s.gy = 2; scs.push_back(s); }
    // 8) corridor reject (0.6 m) — A* root-cause: footprint cannot pass
    { Scenario s; s.name = "corridor_reject_0_6"; s.g = make_corridor(0.6);
      s.path = {{0, 0}, {0, 2}}; s.sx = 0; s.sy = -2; s.gx = 0; s.gy = 2; scs.push_back(s); }
    // 9) corridor reject (0.8 m) — insufficient clearance
    { Scenario s; s.name = "corridor_reject_0_8"; s.g = make_corridor(0.8);
      s.path = {{0, 0}, {0, 2}}; s.sx = 0; s.sy = -2; s.gx = 0; s.gy = 2; scs.push_back(s); }
    // 10) unknown region blocked
    { Scenario s; s.name = "unknown_region_blocked"; s.g = OccupancyGrid(); fill_unknown(s.g);
      s.path = {{1, 0}}; s.sx = 0; s.sy = 0; s.gx = 1; s.gy = 0; scs.push_back(s); }
    // 11) A* narrow door (0.6 m): validate whatever A* returns
    { Scenario s; s.name = "astar_narrow_door_0_6"; s.g = make_corridor(0.6);
      Costmap cm; cm.update_static(s.g);
      AStarPlanner planner(cm);
      s.path = planner.plan(0.0, -2.0, 0.0, 2.0);
      s.sx = 0; s.sy = -2; s.gx = 0; s.gy = 2; scs.push_back(s); }

    fprintf(f, "{\"scenarios\":[");
    for (size_t i = 0; i < scs.size(); ++i) {
        auto& s = scs[i];
        PathValidator v(s.g);
        auto r = v.validate(s.path, s.sx, s.sy, s.gx, s.gy, opts);
        if (i) fprintf(f, ",\n");
        fprintf(f, "{\"name\":\"%s\",", s.name.c_str());
        emit_grid(f, s.g);
        fprintf(f, ",\"start\":[%.6f,%.6f],\"goal\":[%.6f,%.6f],\"path\":",
                s.sx, s.sy, s.gx, s.gy);
        emit_path(f, s.path);
        fprintf(f, ",");
        emit_opts(f, opts);
        char mc_buf[32];
        if (std::isfinite(r.min_clearance))
            snprintf(mc_buf, sizeof(mc_buf), "%.6f", r.min_clearance);
        else
            snprintf(mc_buf, sizeof(mc_buf), "null");
        fprintf(f, ",\"cpp_result\":{\"valid\":%s,\"code\":%d,"
                    "\"min_clearance\":%s,\"endpoint_error\":%.6f"
                    ",\"collision_index\":%d,\"checked_samples\":%d}}",
                r.valid ? "true" : "false", (int)r.code,
                mc_buf, r.endpoint_error,
                r.collision_index, r.checked_samples);
    }
    fprintf(f, "]}");
    fclose(f);
    printf("wrote %s with %d scenarios\n", out, (int)scs.size());
    return 0;
}
