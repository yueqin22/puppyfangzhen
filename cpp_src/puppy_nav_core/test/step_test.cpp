// 分段运行完整测试，找出崩溃位置
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/amcl.h"

using namespace puppy_nav_core;

int main() {
    std::printf("=== T1: occupancy_grid ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        grid.update_lidar_beam(0.0, 0.0, 0.0, 3.0, 8.0);
        int hgx, hgy; grid.world_to_grid(3.0, 0.0, hgx, hgy);
        std::printf("  occupied=%d, hit_occupied=%d\n", grid.count_occupied(), grid.is_occupied(hgx, hgy));
    }
    
    std::printf("=== T2: costmap ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        for (double x = 0.0; x <= 4.2; x += GRID_RESOLUTION) {
            for (int i = 0; i < 2; ++i) {
                grid.update_free(x, 0.0);
            }
        }
        Costmap costmap;
        costmap.update_static(grid, 0);
        std::printf("  wall=%u, far=%u\n", costmap.get_cost(3.0, 0.0), costmap.get_cost(0.0, 0.0));
    }
    
    std::printf("=== T3: A* (single beam) ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        grid.update_lidar_beam(0.0, 0.0, 0.0, 3.0, 8.0);
        Costmap costmap;
        costmap.update_static(grid, 0);
        AStarPlanner planner(costmap);
        auto path = planner.plan(0.0, 0.0, 4.0, 0.0);
        std::printf("  path.size=%zu\n", path.size());
    }
    
    std::printf("=== T4: A* (wall+gap) ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        // 建墙带缺口
        for (double y = -1.0; y <= 1.0; y += 0.1) {
            if (std::abs(y) > 0.2) {
                double dist = std::sqrt(4.0 + y*y);
                grid.update_lidar_beam(0.0, 0.0, std::atan2(y, 2.0), dist, 8.0);
            }
        }
        Costmap costmap;
        costmap.update_static(grid, 0);
        AStarPlanner planner(costmap);
        auto path = planner.plan(0.0, 0.0, 4.0, 0.0);
        std::printf("  path.size=%zu\n", path.size());
        if (!path.empty()) {
            std::printf("  end=(%.2f, %.2f)\n", path.back().first, path.back().second);
        }
    }
    
    std::printf("=== T5: AMCL ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        grid.update_lidar_beam(0.0, 0.0, 0.0, 2.0, 8.0);
        grid.update_lidar_beam(0.0, 0.0, M_PI, 2.0, 8.0);
        grid.update_lidar_beam(0.0, 0.0, M_PI/2, 2.0, 8.0);
        grid.update_lidar_beam(0.0, 0.0, -M_PI/2, 2.0, 8.0);
        
        AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 200, 0.05, 0.99);
        amcl.init_cloud(0.1, 0.1, 0.0, 0.3);
        
        std::vector<double> angles = {-M_PI/2, 0.0, M_PI/2, M_PI};
        std::vector<double> distances = {2.0, 2.0, 2.0, 2.0};
        
        for (int i = 0; i < 5; ++i) {
            auto [px, py, pyaw, pconf] = amcl.update(0, 0, 0, angles, distances, i);
            std::printf("  f%d: x=%.3f y=%.3f conf=%.3f n=%d n_eff=%.1f\n",
                        i, px, py, pconf, amcl.n, amcl.last_n_eff);
        }
    }
    
    std::printf("=== T6: KLD ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 500, 0.05, 0.99);
        amcl.init_cloud(0.0, 0.0, 0.0, 5.0);
        amcl.last_n_eff = 1.0;
        amcl.resample();
        std::printf("  spread cloud: n=%d\n", amcl.n);
        
        amcl.init_cloud(0.0, 0.0, 0.0, 0.1);
        amcl.last_n_eff = 1.0;
        amcl.resample();
        std::printf("  concentrated cloud: n=%d\n", amcl.n);
    }
    
    std::printf("=== T7: performance ===\n"); std::fflush(stdout);
    {
        OccupancyGrid grid;
        for (int i = 0; i < 360; i += 5) {
            double ang = i * M_PI / 180.0;
            double dist = 1.5 + 0.5 * std::sin(5.0 * ang);
            grid.update_lidar_beam(0.0, 0.0, ang, dist, 8.0);
        }
        Costmap costmap;
        
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 1000; ++i) costmap.update_static(grid, i);
        auto t1 = std::chrono::steady_clock::now();
        double costmap_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / 1000.0;
        
        AStarPlanner planner(costmap);
        auto t2 = std::chrono::steady_clock::now();
        for (int i = 0; i < 100; ++i) planner.plan(-1.0, -1.0, 1.0, 1.0);
        auto t3 = std::chrono::steady_clock::now();
        double astar_ms = std::chrono::duration<double, std::milli>(t3 - t2).count() / 100.0;
        
        AMCL amcl(grid, 300, 0.45, 8.0, 36, 50, 500);
        amcl.init_cloud(0.0, 0.0, 0.0, 0.3);
        std::vector<double> angles, distances;
        for (int i = 0; i < 360; i += 10) {
            angles.push_back(i * M_PI / 180.0);
            distances.push_back(1.5 + 0.5 * std::sin(5.0 * i * M_PI / 180.0));
        }
        auto t4 = std::chrono::steady_clock::now();
        for (int i = 0; i < 100; ++i) amcl.update(0.001, 0.001, 0.001, angles, distances, i);
        auto t5 = std::chrono::steady_clock::now();
        double amcl_ms = std::chrono::duration<double, std::milli>(t5 - t4).count() / 100.0;
        
        std::printf("  costmap=%.3fms, A*=%.3fms, AMCL=%.3fms\n", costmap_ms, astar_ms, amcl_ms);
    }
    
    std::printf("\nALL DONE\n");
    return 0;
}
