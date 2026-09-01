// geometry_check.cpp -- M1.5: 几何等价性校验
// ================================================================
// 在场景中生成 500 个随机采样位姿, 对每个位姿做 72 射线 LiDAR 扫描,
// 输出 JSON 格式结果供 UE 端对比 (均值差 < 5cm, 最大差 < 20cm).
//
// 用法:
//   geometry_check.exe [--n 500] [--seed 42] [--scene path] [--json output.json]
//
// 输出:
//   stdout: SUMMARY 行 (供流水线解析)
//   --json: 完整 500 点 x 72 射线扫描数据 (供 UE 端逐点对比)
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <random>
#include <algorithm>

#include "simulation.h"      // build_obstacles, is_position_safe
#include "nav_bridge.h"      // simulate_lidar
#include "scene_loader.h"    // JSON 场景加载
#include "mini_json.h"       // JSON 输出

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 简单 JSON 输出 (避免依赖外部库)
static void write_json(const std::string& path,
                       const std::vector<std::tuple<double,double,double>>& samples,
                       const std::vector<std::vector<double>>& scans) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { printf("[ERROR] 无法写入: %s\n", path.c_str()); return; }

    fprintf(f, "{\"n_samples\":%zu,\"n_rays\":72,\"samples\":[\n", samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        double x, y, yaw;
        std::tie(x, y, yaw) = samples[i];
        fprintf(f, "  {\"x\":%.4f,\"y\":%.4f,\"yaw\":%.4f,\"distances\":[", x, y, yaw);
        for (size_t j = 0; j < scans[i].size(); ++j) {
            fprintf(f, "%.4f", scans[i][j]);
            if (j + 1 < scans[i].size()) fprintf(f, ",");
        }
        fprintf(f, "]}");
        if (i + 1 < samples.size()) fprintf(f, ",");
        fprintf(f, "\n");
    }
    fprintf(f, "]}\n");
    fclose(f);
    printf("[GeometryCheck] JSON 已写入: %s\n", path.c_str());
}

int main(int argc, char* argv[]) {
    int n_samples = 500;
    int seed = 42;
    std::string scene_path = "../../config/scene_home.json";
    std::string json_path = "";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--n" && i + 1 < argc) n_samples = std::atoi(argv[++i]);
        else if (arg == "--seed" && i + 1 < argc) seed = std::atoi(argv[++i]);
        else if (arg == "--scene" && i + 1 < argc) scene_path = argv[++i];
        else if (arg == "--json" && i + 1 < argc) json_path = argv[++i];
    }

    // 加载场景
    auto obstacles = scene_loader::load_obstacles(scene_path);
    if (obstacles.empty()) {
        printf("[ERROR] 障碍物为空, 场景加载失败: %s\n", scene_path.c_str());
        return 1;
    }
    printf("[GeometryCheck] 场景: %zu 障碍物, %d 采样点, seed=%d\n",
           obstacles.size(), n_samples, seed);

    // 生成随机采样位姿 (在场景内, 不在墙内)
    std::mt19937 rng(static_cast<uint32_t>(seed));
    std::uniform_real_distribution<> dist_x(-4.5, 4.5);
    std::uniform_real_distribution<> dist_y(-3.5, 3.5);
    std::uniform_real_distribution<> dist_yaw(-M_PI, M_PI);

    std::vector<std::tuple<double, double, double>> samples;
    std::vector<std::vector<double>> scans;

    int attempts = 0;
    while ((int)samples.size() < n_samples && attempts < n_samples * 20) {
        double x = dist_x(rng);
        double y = dist_y(rng);
        double yaw = dist_yaw(rng);
        attempts++;

        if (!is_position_safe(x, y, obstacles)) continue;

        // 做 72 射线 LiDAR 扫描
        std::vector<double> angles, distances;
        simulate_lidar(x, y, obstacles, 72, 8.0, angles, distances);

        samples.emplace_back(x, y, yaw);
        scans.push_back(distances);
    }

    printf("[GeometryCheck] 生成 %zu/%d 有效采样点 (%d 次尝试)\n",
           samples.size(), n_samples, attempts);

    // 计算统计信息
    double dist_sum = 0, dist_max = 0, dist_min = 1e9;
    int hit_count = 0, total_rays = 0;
    for (size_t i = 0; i < scans.size(); ++i) {
        for (double d : scans[i]) {
            dist_sum += d;
            dist_max = std::max(dist_max, d);
            dist_min = std::min(dist_min, d);
            if (d < 8.0) hit_count++;  // 未命中 max_range
            total_rays++;
        }
    }
    double dist_mean = total_rays > 0 ? dist_sum / total_rays : 0;
    double hit_rate = total_rays > 0 ? 100.0 * hit_count / total_rays : 0;

    printf("[GeometryCheck] 距离统计: mean=%.3fm max=%.3fm min=%.3fm hit_rate=%.1f%%\n",
           dist_mean, dist_max, dist_min, hit_rate);

    // 输出 JSON (可选)
    if (!json_path.empty()) {
        write_json(json_path, samples, scans);
    }

    // SUMMARY 行 (供流水线解析)
    printf("SUMMARY: n_samples=%zu mean_dist=%.4f max_dist=%.4f min_dist=%.4f hit_rate=%.1f\n",
           samples.size(), dist_mean, dist_max, dist_min, hit_rate);

    return 0;
}
