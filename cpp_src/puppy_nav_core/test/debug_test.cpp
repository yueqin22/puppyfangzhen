// 调试版测试 — 逐个调用找出崩溃位置
#include <cstdio>
#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/amcl.h"

using namespace puppy_nav_core;

int main() {
    std::printf("STEP 1: 创建 OccupancyGrid\n"); std::fflush(stdout);
    OccupancyGrid grid;
    std::printf("STEP 1 OK: log_odds.size=%zu\n", grid.log_odds.size()); std::fflush(stdout);
    
    std::printf("STEP 2: update_lidar_beam\n"); std::fflush(stdout);
    grid.update_lidar_beam(0.0, 0.0, 0.0, 3.0, 8.0);
    std::printf("STEP 2 OK: occupied=%d\n", grid.count_occupied()); std::fflush(stdout);
    
    std::printf("STEP 3: 创建 Costmap\n"); std::fflush(stdout);
    Costmap costmap;
    std::printf("STEP 3 OK: cost.size=%zu\n", costmap.cost.size()); std::fflush(stdout);
    
    std::printf("STEP 4: costmap.update_static\n"); std::fflush(stdout);
    costmap.update_static(grid, 0);
    std::printf("STEP 4 OK\n"); std::fflush(stdout);
    
    std::printf("STEP 5: A* plan\n"); std::fflush(stdout);
    AStarPlanner planner(costmap);
    auto path = planner.plan(0.0, 0.0, 4.0, 0.0);
    std::printf("STEP 5 OK: path.size=%zu\n", path.size()); std::fflush(stdout);
    
    std::printf("STEP 6: 创建 AMCL\n"); std::fflush(stdout);
    AMCL amcl(grid, 100, 0.45, 8.0, 36, 50, 200, 0.05, 0.99);
    std::printf("STEP 6 OK: n=%d\n", amcl.n); std::fflush(stdout);
    
    std::printf("STEP 7: amcl.init_cloud\n"); std::fflush(stdout);
    amcl.init_cloud(0.1, 0.1, 0.0, 0.3);
    std::printf("STEP 7 OK\n"); std::fflush(stdout);
    
    std::printf("STEP 8: amcl.get_estimate\n"); std::fflush(stdout);
    auto [x, y, yaw, conf] = amcl.get_estimate();
    std::printf("STEP 8 OK: x=%.3f y=%.3f conf=%.3f\n", x, y, conf); std::fflush(stdout);
    
    std::printf("STEP 9: amcl.update\n"); std::fflush(stdout);
    std::vector<double> angles = {-M_PI/2, 0.0, M_PI/2, M_PI};
    std::vector<double> distances = {2.0, 2.0, 2.0, 2.0};
    auto [px, py, pyaw, pconf] = amcl.update(0, 0, 0, angles, distances, 0);
    std::printf("STEP 9 OK: x=%.3f y=%.3f conf=%.3f\n", px, py, pconf); std::fflush(stdout);
    
    std::printf("\nALL STEPS PASSED\n");
    return 0;
}
