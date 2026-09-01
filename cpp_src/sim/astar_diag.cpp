// astar_diag.cpp — 诊断新 A* 在 SHARED 场景下的"真无路径"断连根因。
// 对每条相邻巡逻边调用 NavCoreStack::plan；对第一条失败边，打印 static_cost
// 栅格的 ASCII 地图 + 起点/终点 + 从起点 BFS 可达区，定位阻断墙。
#include <cstdio>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <algorithm>
#include "simulation.h"
#include "nav_bridge.h"
#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"

using namespace puppy_sim;
using namespace puppy_nav_core;

static const int RCELLS = (int)std::ceil(ROBOT_RADIUS / GRID_RESOLUTION); // ROBOT_RADIUS/RES

// 复刻 is_footprint_traversable：中心代价<阈值 且 footprint 内无 LETHAL。
// combined=true 用 get_cost_grid（含动态层），false 用 static_cost（仅静态）。
bool traversable(const Costmap& cm, int gx, int gy, bool combined) {
    if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) return false;
    uint8_t center = combined ? cm.get_cost_grid(gx, gy) : cm.static_cost[(size_t)gy * GRID_W + gx];
    if (center >= 120) return false;
    for (int dy = -RCELLS; dy <= RCELLS; ++dy)
        for (int dx = -RCELLS; dx <= RCELLS; ++dx) {
            if (std::sqrt(dx*dx+dy*dy)*GRID_RESOLUTION > 0.35) continue;
            int nx = gx+dx, ny = gy+dy;
            if (nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) return false;
            if (cm.static_cost[(size_t)ny*GRID_W+nx] >= 254) return false;
        }
    return true;
}

int main() {
    auto obstacles = build_obstacles();
    auto targets = build_patrol_targets();
    NavCoreStack nav;
    nav.init_from_obstacles(obstacles);

    int first_fail = -1;
    double fsx, fsy, fgx, fgy;
    int f_idx = -1;
    for (size_t i = 0; i + 1 < targets.size(); ++i) {
        double sx = targets[i].x, sy = targets[i].y;
        double gx = targets[i+1].x, gy = targets[i+1].y;
        int b_nns=nav.planner.fail_no_nearest_start, b_nng=nav.planner.fail_no_nearest_goal,
            b_np=nav.planner.fail_no_path, b_to=nav.planner.fail_timeout, b_mn=nav.planner.fail_max_nodes;
        auto p = nav.plan(sx, sy, gx, gy, obstacles, 0);
        bool ok = p.size() >= 2;
        printf("[%2zu->%2zu] %-18s -> %-18s : %s\n", i, i+1,
               targets[i].name.c_str(), targets[i+1].name.c_str(),
               ok ? "OK" : "FAIL");
        if (!ok && first_fail < 0) {
            first_fail = (int)i; fsx=sx; fsy=sy; fgx=gx; fgy=gy;
            f_idx = (int)i;
            printf("    -> 增量失败: no_nearest_start=%d no_nearest_goal=%d no_path=%d timeout=%d max_nodes=%d\n",
                   nav.planner.fail_no_nearest_start-b_nns, nav.planner.fail_no_nearest_goal-b_nng,
                   nav.planner.fail_no_path-b_np, nav.planner.fail_timeout-b_to, nav.planner.fail_max_nodes-b_mn);
        }
    }
    if (first_fail < 0) { printf("ALL EDGES OK\n"); return 0; }

    printf("\n=== 第一条失败边 %d->%d : (%.2f,%.2f)->(%.2f,%.2f) ===\n",
           first_fail, first_fail+1, fsx, fsy, fgx, fgy);

    // 用静态层做连通性分析（update_static 已在 plan 内调用；再确保一次）
    nav.costmap.update_static(nav.grid, 0);

    auto w2g = [](double x, double y, int& gx, int& gy){
        gx = (int)((x - ORIGIN_X)/GRID_RESOLUTION);
        gy = (int)((y - ORIGIN_Y)/GRID_RESOLUTION);
    };
    int sgx,sgy,ggx,ggy;
    w2g(fsx,fsy,sgx,sgy); w2g(fgx,fgy,ggx,ggy);

    // 起点/终点吸附（最近可通行格，用 combined 判定，与标准 plan 一致）
    auto snap = [&](int& gx, int& gy){
        if (traversable(nav.costmap, gx, gy, true)) return;
        for (int r=1; r<=20; ++r)
            for (int dy=-r; dy<=r; ++dy)
                for (int dx=-r; dx<=r; ++dx){
                    int nx=gx+dx, ny=gy+dy;
                    if (traversable(nav.costmap,nx,ny,true)){gx=nx;gy=ny;return;}
                }
    };
    int ssgx=sgx,ssgy=sgy,gggx=ggx,gggy=ggy;
    snap(ssgx,ssgy); snap(gggx,gggy);

    // BFS 从吸附起点（combined = A* 实际看到的代价图）
    std::vector<char> reach((size_t)GRID_W*GRID_H, 0);
    std::queue<std::pair<int,int>> q;
    q.push({ssgx,ssgy}); reach[(size_t)ssgy*GRID_W+ssgx]=1;
    int dx8[]={-1,-1,-1,0,0,1,1,1}, dy8[]={-1,0,1,-1,1,-1,0,1};
    while(!q.empty()){
        auto[cx,cy]=q.front(); q.pop();
        for(int k=0;k<8;++k){
            int nx=cx+dx8[k], ny=cy+dy8[k];
            if(nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
            if(reach[(size_t)ny*GRID_W+nx]) continue;
            if(!traversable(nav.costmap,nx,ny,true)) continue;
            reach[(size_t)ny*GRID_W+nx]=1; q.push({nx,ny});
        }
    }
    bool connected = reach[(size_t)gggy*GRID_W+gggx]==1;
    printf("吸附起点格(%d,%d) 吸附终点格(%d,%d) combined连通=%s\n",
           ssgx,ssgy,gggx,gggy, connected?"YES":"NO");

    // 仅静态层连通性对比
    std::vector<char> reachS((size_t)GRID_W*GRID_H, 0);
    std::queue<std::pair<int,int>> qS;
    qS.push({ssgx,ssgy}); reachS[(size_t)ssgy*GRID_W+ssgx]=1;
    while(!qS.empty()){
        auto[cx,cy]=qS.front(); qS.pop();
        for(int k=0;k<8;++k){
            int nx=cx+dx8[k], ny=cy+dy8[k];
            if(nx<0||nx>=GRID_W||ny<0||ny>=GRID_H) continue;
            if(reachS[(size_t)ny*GRID_W+nx]) continue;
            if(!traversable(nav.costmap,nx,ny,false)) continue;
            reachS[(size_t)ny*GRID_W+nx]=1; qS.push({nx,ny});
        }
    }
    printf("仅静态层连通=%s\n", reachS[(size_t)gggy*GRID_W+gggx]?"YES":"NO");

    // 打印聚焦区域 ASCII（覆盖起点->终点，留边距）
    int minx=std::min(ssgx,gggx)-8, maxx=std::max(ssgx,gggx)+8;
    int miny=std::min(ssgy,gggy)-8, maxy=std::max(ssgy,gggy)+8;
    minx=std::max(0,minx); maxx=std::min(GRID_W-1,maxx);
    miny=std::max(0,miny); maxy=std::min(GRID_H-1,maxy);
    printf("地图范围 x[%d,%d] y[%d,%d] (y 向上)\n", minx,maxx,miny,maxy);
    for (int y=maxy; y>=miny; --y){
        for (int x=minx; x<=maxx; ++x){
            char c;
            uint8_t sc = nav.costmap.static_cost[(size_t)y*GRID_W+x];
            if (sc >= 254) c='#';
            else if (sc >= 120) c='+';
            else if (x==ssgx&&y==ssgy) c='S';
            else if (x==gggx&&y==gggy) c='G';
            else if (reach[(size_t)y*GRID_W+x]) c='o';
            else if (traversable(nav.costmap,x,y,true)) c='.';
            else c=' ';
            putchar(c);
        }
        putchar('\n');
    }
    printf("图例: #=LETHAL墙 +=膨胀阻挡(>=120) .=可通行 o=起点可达 S=起点 G=终点 空格=不可通行\n");
    return 0;
}
