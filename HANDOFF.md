# 机器狗自主导航仿真 - 交接文档

## 1. 项目概述

基于 CoppeliaSim 的四足机器狗自主导航仿真系统，采用 Nav2 启发的商用级导航架构。

- **仿真环境**: CoppeliaSim (Linux版，运行于 WSL2 Ubuntu-22.04 + Xvfb 软件渲染)
- **场景**: 10m × 8m 室内家居环境（客厅 + 卧室 + 厨房），含墙、门、沙发、床、餐桌
- **机器人**: Puppy 四足机器狗，半径 0.35m
- **通信**: ZMQ Remote API (`coppeliasim_zmqremoteapi_client`)

## 2. 核心架构

```
LiDAR扫描(360°/72射线) → OccupancyGrid → Costmap(分层) → A*(全局) → DWA(局部) → 状态机
```

### 状态机: PLAN → FOLLOW → RECOVER → DONE

| 状态 | 功能 |
|------|------|
| PLAN | 前沿检测 + A*全局规划 + 不可达goal过滤 |
| FOLLOW | 路径剪枝 + DWA局部规划 + 进度检测 |
| RECOVER | 8方向最低cost逃逸 + 防逆向振荡 |
| DONE | 探索完成，保持位置，定期检查新前沿 |

## 3. 核心文件（5个）

| 文件 | 功能 | 行数 |
|------|------|------|
| [occupancy_grid.py](occupancy_grid.py) | 占据栅格地图 (0.1m, log-odds贝叶斯, Bresenham射线) | ~200 |
| [costmap.py](costmap.py) | 分层costmap (静态+障碍+膨胀), unknown_mask, COST_INSCRIBED=128 | ~250 |
| [astar_planner.py](astar_planner.py) | A* 8连通 + octile启发式 + 路径平滑 + PLAN_BLOCKED=110 | ~214 |
| [dwa_planner.py](dwa_planner.py) | DWA 两阶段(ALIGN+DRIVE) + 5项评分 + 顺序航点跟踪 | ~340 |
| [autonomous_nav.py](autonomous_nav.py) | 主导航脚本，集成所有模块 + 状态机 | ~700 |

### 主入口
```bash
python3 autonomous_nav.py
```

## 4. 关键参数

### DWA 参数 ([dwa_planner.py:26-59](dwa_planner.py#L26))
```python
max_v = 0.3        # 最大线速度 m/s
max_w = 1.2        # 最大角速度 rad/s
horizon = 2.0      # 预测时域 s
v_samples = 9      # 线速度采样数
w_samples = 15     # 角速度采样数
ALIGN阈值 = 0.5    # rad，超过则原地旋转对准
goal_tolerance = 0.3  # m
```

### DWA 评分权重 ([dwa_planner.py:48-52](dwa_planner.py#L48))
```python
w_heading = 0.30    # 朝向goal
w_clearance = 0.15  # 障碍距离(cost 0-49满分, 50-127线性递减)
w_velocity = 0.10   # 前进速度
w_goal = 0.35       # 接近goal进度(主导项)
w_path = 0.10       # A*路径跟踪(cross-track error)
```

### A* 参数 ([astar_planner.py:23](astar_planner.py#L23))
```python
PLAN_BLOCKED = 110  # cost≥110视为障碍(低于COST_INSCRIBED=128，避开窄缝)
# _cell_cost: 二次惩罚 1.0 + (c/110)^2 * 8.0
# _is_traversable: 禁止穿越unknown cells
```

### 导航参数 ([autonomous_nav.py](autonomous_nav.py))
```python
ROBOT_RADIUS = 0.35
SAVE_INTERVAL = 200           # 地图保存间隔(帧)
goal_tolerance = 0.5          # FOLLOW状态goal到达判定
no_progress_frames > 30       # 触发RECOVER
recover_frames > 10           # 回到PLAN
VISITED_FRONTIER_TTL = 300    # 已访问frontier屏蔽时长(帧)
no_frontier_cycles > 5        # 进入DONE状态
unreachable半径 = 3 cells     # 空间扩展标记
```

## 5. 已解决的关键问题（按重要性）

### P0 - 致命问题
1. **A*穿墙规划** — unknown cell cost惩罚(80) + 禁止穿越unknown + 路径平滑禁止穿越unknown
2. **DWA穿墙** — _get_local_goal改为顺序航点跟踪（非最近航点）
3. **velocity_to_step不使用当前yaw** — 改用midpoint heading `ryaw + w*dt*0.5`
4. **goal不可达死循环** — path终点检测 + unreachable_goals(带空间扩展)

### P1 - 严重振荡
5. **沙发南侧窄区域振荡** — PLAN_BLOCKED=110 + 二次cost惩罚
6. **门口穿越振荡** — w_clearance 0.3→0.15, ALIGN阈值 0.35→0.5, clearance归一化
7. **前沿切换振荡** — 目标持久化(goal_plan_attempts) + visited_frontiers(TTL)
8. **持久前沿重复访问** — 二次访问同一frontier标记unreachable

### P2 - 效率优化
9. **DWA path-following** — cross-track error评分，窄通道更稳定
10. **DONE状态** — 所有前沿不可达时停止，避免无意义移动
11. **RECOVER防逆向** — 跳过>135°的反向，避免振荡

## 6. 场景障碍物布局

```
wall_south:    x=[-5,5]    y=[-4.05,-3.95]   南墙
wall_north:    x=[-5,5]    y=[3.95,4.05]     北墙
wall_west:     x=[-5.05,-4.95] y=[-4,4]      西墙
wall_east:     x=[4.95,5.05] y=[-4,4]        东墙
wall_divide_1: x=[-5,-1]   y=[-0.05,0.05]    隔墙西段(门口在x=[-1,1])
wall_divide_2: x=[1,5]     y=[-0.05,0.05]    隔墙东段
sofa:          x=[2.75,4.25] y=[-3.30,-2.70] 沙发(南侧)
bed:           x=[-4.25,-2.75] y=[2.00,4.00] 床(卧室)
dining_table:  x=[1.60,2.40] y=[2.47,2.53]   餐桌(厨房)

机器人起点: (1.0, -2.0, 0.0) 朝向东
```

## 7. 运行环境配置

### 启动顺序
```bash
# 1. Windows端: 启动WSL
wsl -d Ubuntu-22.04

# 2. WSL内: 启动Xvfb虚拟显示
Xvfb :99 -screen 0 1280x720x24 -nolisten tcp &

# 3. WSL内: 启动CoppeliaSim(加载场景)
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
export LD_LIBRARY_PATH=/home/veni/CoppeliaSim
cd /home/veni/CoppeliaSim
./coppeliaSim /home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt &

# 4. WSL内: 启动MJPG流服务器(第一人称视角)
cd /home/veni && python3 mjpeg_file_server.py &

# 5. WSL内: 运行导航(在项目目录)
cd /mnt/e/puppyfangzhen
DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 python3 -u autonomous_nav.py
```

### 流媒体
- 第一人称视角: `http://localhost:8081/stream`
- 地图HTML: `/tmp/puppy_map.html`
- 地图数据: `/tmp/puppy_map.npz` (可删除重置地图)

## 8. 当前性能指标

| 指标 | 数值 |
|------|------|
| 最终coverage | ~95% |
| visited cells | 1364+ |
| 不可达区域数 | ~8个(沙发南侧、东北角等) |
| 门口穿越 | 成功(南侧↔北侧) |
| 探索状态 | 能正确进入DONE |

## 9. 已知限制与待改进

### 已知限制
1. **地图不持久化unreachable_goals** — 每次重启会重新尝试已标记的不可达frontier（但有visited_frontiers兜底）
2. **门口偶尔振荡** — DWA在门口窄通道中valid轨迹有限，path-following score缓解但未完全解决
3. **前沿聚类粗糙** — find_frontiers返回的frontier可能在相邻grid cell，空间扩展(r=3)覆盖大部分情况

### 可改进方向
1. **持久化unreachable_goals** — 保存到map文件，避免重启后重复尝试
2. **frontier聚类** — 对物理上接近的frontier做DBSCAN聚类，选代表点
3. **DWA窄通道模式** — 检测窄通道时切换参数(降低w_clearance，提高w_path)
4. **TEB Planner** — 替换DWA为TEB(Time Elastic Band)，窄通道表现更好
5. **多机器人协同** — 扩展为多机器人探索
6. **3D导航** — 支持高度变化(楼梯、斜坡)
7. **视觉SLAM** — 替换占据栅格为ORB-SLAM3，提升建图精度
8. **动态障碍物** — 添加行人/移动物体处理

## 10. 调试技巧

### 快速重置
```bash
# 删除地图重新开始
rm /tmp/puppy_map.npz
# 机器人会从0% coverage开始探索
```

### 查看地图
- 浏览器打开 `/tmp/puppy_map.html`（每10帧自动刷新）
- 绿点=机器人，蓝线=路径，红点=goal，白=free，黑=occupied，灰=unknown

### 日志解读
```
[PLAN] Frontier (x,y) path=Nwp [...] cov=XX%   # 选择前沿并规划
[FOLLOW] Reached goal (x,y)                      # 到达goal
[FOLLOW] Path end reached but goal still Xm away — marking unreachable  # 不可达
[PLAN] Giving up on goal (x,y) after 3 attempts  # 放弃(3次失败)
[FOLLOW] Re-visited goal — marking unreachable   # 持久前沿
[PLAN] Exploration complete                       # 进入DONE
[NAV] pos=(x,y) yaw=θ state=S cov=C% vis=V       # 状态输出(每50帧)
```

### 常见问题
| 现象 | 原因 | 解决 |
|------|------|------|
| `ModuleNotFoundError: coppeliasim_zmqremoteapi_client` | Windows端运行 | 必须在WSL运行 |
| 无输出 | Python缓冲 | 用 `python3 -u` |
| 连接失败 | CoppeliaSim未启动 | 检查 `ps aux \| grep coppelia` |
| 机器人不动 | 仿真未开始 | 确认CoppeliaSim场景已加载并运行 |
| 振荡 | DWA参数 | 调整w_clearance/w_path权重 |

## 11. 文件清单

### 核心导航（5个，必须）
- `occupancy_grid.py` — 占据栅格
- `costmap.py` — 分层costmap
- `astar_planner.py` — A*全局规划
- `dwa_planner.py` — DWA局部规划
- `autonomous_nav.py` — 主导航脚本

### 辅助工具
- `mjpeg_file_server.py` — MJPG流服务器
- `build_coppelia_scene.py` — 场景构建
- `reset_robot.py` — 机器人重置
- `stop_sim.py` — 停止仿真
- `diag_*.py` / `check_*.py` / `test_*.py` — 诊断/测试脚本

### 数据文件
- `/tmp/puppy_map.npz` — 保存的地图(log-odds + visited)
- `/tmp/puppy_map.html` — 地图可视化
- `/tmp/stream_frame.jpg` — 第一人称视角帧

---

**最后更新**: 2026-07-03
**Coverage**: ~95%
**状态**: 可正常运行，探索效率良好，能正确处理不可达区域
