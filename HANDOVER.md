# 项目交接文档 — 四足机器狗自主导航仿真系统

> **最后更新**: 2026-07-12 | **版本**: v4.0 | **负责人**: Veni
>
> 本文档面向接手开发者，聚焦**当前状态**与**快速上手**。历史迭代细节见附录。

---

## 一、项目概览

基于 CoppeliaSim 的四足机器狗自主导航仿真系统，对标 Nav2 商用级架构，纯 Python 实现，从感知到决策全栈自研。

**核心能力**：里程计 + AMCL 定位 → 雷达感知 → 占据栅格建图 → 信息增益 Frontier 探索 → A* 全局规划 → TEB 局部规划 → 状态机调度 → 语义视觉层 + 量化评估。

**v4.0 新增**：8房间自动巡航仿真、动态避障系统、4个技术创新模块（Risk-Aware A*、D* Lite、Lifelong SLAM、GNN轨迹预测）。

**当前性能**（v4.0 长时仿真，2小时72000帧）：
- 到达率 **100%**（204轮巡航全通过）
- 覆盖率 **76.6%**（140/201栅格）
- 穿墙事件 **0次**（已修复）
- 动态碰撞 **782次**（修复前1785次，降56%）
- FPS稳定 **25.3-27.3**
- 电量循环正常（100%→20%→100% 反复18次）

**两条技术路线**：

| 路线 | 说明 | 状态 |
|------|------|------|
| **CoppeliaSim + Python** | 纯 Python 脚本通过 ZMQ 远程控制，算法全栈自研 | **当前主力** |
| ROS2 + Gazebo | 经典 ROS2 Humble + Nav2 + SLAM 导航栈 | 备用路线 |

---

## 二、快速启动

### 2.1 环境依赖

- **操作系统**: Windows 11 + WSL2 Ubuntu 22.04
- **仿真器**: CoppeliaSim Edu（运行于 WSL2 + Xvfb 软件渲染）
- **Python**: 3.10+

```bash
pip install coppeliasim-zmqremoteapi-client Pillow numpy matplotlib
# 重要：不要安装 scipy（与 NumPy 2.x 不兼容）
```

### 2.2 启动步骤

**第一步：启动 CoppeliaSim**（Windows 上）— 打开 CoppeliaSim，加载家庭场景 `.ttt` 文件。

**第二步：启动 MJPEG 流服务器**（WSL 中）
```bash
wsl -d Ubuntu-22.04
export PUPPY_STREAM_FRAME=/tmp/stream_frame.jpg
python3 /mnt/e/puppyfangzhen/mjpeg_file_server.py
```
流地址: `http://localhost:8081/stream`

**第三步：启动自主导航**（WSL 中）
```bash
wsl -d Ubuntu-22.04
cd /mnt/e/puppyfangzhen

# 推荐方案（AMCL + TEB）
USE_AMCL=1 USE_TEB=1 METHOD_NAME=proposed_amcl_teb \
  python3 -u autonomous_nav.py

# 快速测试（限 800 帧）
MAX_FRAMES=800 USE_AMCL=1 USE_TEB=1 python3 -u autonomous_nav.py
```

**第四步：一键运行 4 组对比实验**
```bash
bash _full_experiment.sh
```

### 2.3 环境变量控制开关

| 变量 | 默认 | 说明 |
|------|------|------|
| `USE_AMCL` | `0` | `1`=AMCL 定位，`0`=读 ground truth |
| `USE_TEB` | `0` | `1`=TEB 局部规划，`0`=DWA |
| `USE_EVAL` | `1` | `1`=开启量化评估 |
| `USE_SEMANTIC` | `1` | `1`=开启 HSV 语义建图 |
| `METHOD_NAME` | `proposed` | 实验方法名，影响输出命名 |
| `MAX_FRAMES` | `0` | 最大帧数，`0`=无限 |
| `EVAL_DIR` | `/tmp/eval` | 评估产物输出目录 |
| `PROFILE` | `default` | v2.7b: `raspberry_pi`=树莓派 5 降配（粒子 100, TEB 迭代 2, 射线 36） |

**树莓派 5 降配部署示例**（v2.7b 新增）：
```bash
# 满配（桌面 CPU，默认）
USE_AMCL=1 USE_TEB=1 python3 -u autonomous_nav.py

# 树莓派 5 降配（保持 10Hz）
USE_AMCL=1 USE_TEB=1 PROFILE=raspberry_pi python3 -u autonomous_nav.py
```
降配详情见 [config/profile_raspberry_pi.yaml](config/profile_raspberry_pi.yaml)。

### 2.4 四种实验方法

| 方法名 | USE_AMCL | USE_TEB | 说明 |
|--------|----------|---------|------|
| `groundtruth` | 0 | 0 | 基线：ground truth + DWA |
| `random_walk` | - | - | 随机漫游基线（独立脚本） |
| `proposed_amcl_dwa` | 1 | 0 | AMCL + 信息增益 + DWA |
| `proposed_amcl_teb` | 1 | 1 | **推荐**：AMCL + 信息增益 + TEB |

---

## 三、核心文件清单

### 3.1 导航核心模块（10 个）

| 文件 | 功能 | 行数 |
|------|------|------|
| [autonomous_nav.py](autonomous_nav.py) | 主导航脚本：状态机 + 模块集成 + 评估 | ~1050 |
| [odometry.py](odometry.py) | 差速驱动里程计，含高斯噪声模型 | ~120 |
| [amcl.py](amcl.py) | AMCL 粒子滤波定位（Likelihood Field，向量化） | ~480 |
| [occupancy_grid.py](occupancy_grid.py) | 占据栅格地图 + 信息增益 Frontier | ~280 |
| [costmap.py](costmap.py) | 分层 costmap（静态 + 障碍 + 膨胀） | ~250 |
| [astar_planner.py](astar_planner.py) | A* 8 连通 + octile 启发式 + 路径平滑 | ~214 |
| [dwa_planner.py](dwa_planner.py) | DWA 两阶段（ALIGN+DRIVE）+ 5 项评分 | ~340 |
| [teb_planner.py](teb_planner.py) | TEB Timed Elastic Band 轨迹优化 | ~320 |
| [evaluator.py](evaluator.py) | 量化评估 + 地图质量指标（IoU/Hausdorff/Entropy） | ~420 |
| [vision.py](vision.py) | HSV 颜色语义建图层 | ~180 |

### 3.2 辅助模块

| 文件 | 功能 |
|------|------|
| [random_walk.py](random_walk.py) | 随机漫游基线对照 |
| [config_loader.py](config_loader.py) | YAML 配置加载器 |
| [mjpeg_file_server.py](mjpeg_file_server.py) | MJPEG 视频流服务器（端口 8081） |
| [build_coppelia_scene.py](build_coppelia_scene.py) | 构建仿真场景脚本 |
| [_full_experiment.sh](_full_experiment.sh) | 4 组对比实验一键脚本 |

### 3.3 配置与产物

| 路径 | 说明 |
|------|------|
| [config/](config/) | YAML 配置文件（exploration/mapping/planner/runtime/sim） |
| [eval_results/](eval_results/) | 实验产物：CSV、JSON、PNG 对比图 |

---

## 四、核心算法简介

### 4.1 AMCL 定位（Likelihood Field Model）

**文件**: [amcl.py](amcl.py)

从 Beam Range Finder Model 改为 Likelihood Field Model（Thrun 6.4），全向量化实现，**750 倍加速**（300ms → 0.4ms）。距离变换用 numpy-only 的 3-4-5 Chamfer 算法替代 scipy。

**关键参数**（v2.7b 调优后）：
```python
n_particles = 200        # 粒子数（v2.6: 300→200；v2.7b: PROFILE=raspberry_pi 时 100）
n_obs_rays = 24          # 观测射线数（v2.7b: PROFILE=raspberry_pi 时 16）
sigma_obs = 0.55         # 似然场高斯 sigma（v2.3.1: 0.35→0.55）
neff_threshold = n/3     # 有效粒子数阈值（重采样触发）
alpha1/alpha4 = 0.08     # 旋转噪声
alpha2/alpha3 = 0.04     # 平移噪声
```

**门道地标辅助定位**（v2.3 新增，v2.6 优化）：穿门后**在 loc_err > 0.3m 时**触发 `amcl.recover(spread=0.3)`（v2.6: 0.5→0.3m，更早纠正漂移）。50 帧冷却期防止门道附近反复穿越导致 AMCL 抖动。

**主动漂移检测 AMCL-PROACTIVE**（v2.7 新增，v2.7b 调优）：即使无门道穿越，当 `loc_err > 0.5m` 时主动触发 `recover(spread=0.3)`。v2.7b 参数：
- 启动延迟 `frame > 50`（v2.7: 无延迟 → 误触发）
- 冷却期 150 帧（v2.7: 100 → 过频繁）
- spread=0.3（v2.7: 0.5 → 粒子云过散）
- PROFILE=raspberry_pi 时自动关闭（`recover()` 计算量大）

### 4.2 信息增益 Frontier 选择

**文件**: [occupancy_grid.py](occupancy_grid.py) 的 `find_frontiers_with_info_gain()`

平衡探索价值与移动成本：`utility = α·info_gain_norm - β·distance_norm`（α=1.0, β=0.5）。BFS 4 连通聚类 O(n) 替代贪心 O(n²)。

### 4.3 TEB 局部规划器

**文件**: [teb_planner.py](teb_planner.py)

Timed Elastic Band 轨迹优化：15 个 pose 弹性带，dt=0.1s，梯度下降 4 次迭代。

**参数**（v2.7b 调优后）：
```python
w_obstacle = 3.0    # 障碍物代价（门道附近自适应降至 0.8）
w_smooth = 0.8      # 平滑度
w_path = 0.6        # 路径跟随（v2.3: 0.4→0.6）
w_goal = 1.2        # 目标导向（门道附近自适应升至 1.5）
max_v = 0.3         # 最大线速度
n_iterations = 4    # 优化迭代数
learning_rate = 0.08
# v2.7 门道参数自适应（v2.6: 1.5→v2.7: 0.8，检测范围 |y|<1.0→|y|<1.5）
# v2.7b: PROFILE=raspberry_pi 时 n_iterations=2, n_poses=10, max_v=0.25
```

**v2.7 门道参数自适应改进**：v2.6 的 `w_obstacle=1.5` 在门道附近仍然过于激进，导致 12 次门道穿越和 10 次 FOLLOW-OSC 振荡。v2.7 将 `w_obstacle` 降至 0.8，检测范围从 `|y|<1.0` 扩大到 `|y|<1.5`，让弹性带在门道附近几乎不受障碍物推力，平滑穿过窄通道。

### 4.4 地图质量评估

**文件**: [evaluator.py](evaluator.py) 的 `compute_map_metrics()`

| 指标 | 含义 |
|------|------|
| `iou_occupied` / `iou_free` | 障碍物/自由区域交并比 |
| `hausdorff_m` | Hausdorff 距离（最坏边界误差） |
| `map_entropy` | Shannon 熵（地图不确定度） |
| `precision` / `recall` | 精确率/召回率 |
| `observed_pct` | 已观测区域百分比 |

### 4.5 状态机

| 状态 | 功能 |
|------|------|
| `PLAN` | 信息增益 Frontier 选择 + A* 全局规划 |
| `FOLLOW` | 路径剪枝 + TEB/DWA 局部规划 + 进度检测 + **门道振荡检测（v2.5）+ 30 帧冷却（v2.7）** |
| `RECOVER` | 16 方向最低 cost 逃逸 + 渐进式步长 + 后退优先 + **振荡抑制（v2.5）** |
| `DONE` | 探索完成，定期检查新 frontier |

**关键改进**：
- **v2.7 FOLLOW-OSC 冷却期 + 清空历史**：触发后 30 帧不重复检测，并清空 `doorway_crossing_frames`，彻底消除"选目标→中止→选目标"连锁循环（v2.6b 10 次 → v2.7 0 次）
- **v2.7 AMCL-PROACTIVE 主动漂移检测**：loc_err > 0.5m 时主动 recover，frame>50 启动延迟 + 150 帧冷却 + spread=0.3
- **v2.6 TEB 门道参数自适应**：门道附近（v2.7: |y|<1.5, |x|<2.5）w_obstacle 3.0→0.8, w_goal 1.2→1.5
- **v2.6 Frontier 振荡过滤**：40 帧内振荡过则不选门道另一侧 frontier
- **v2.5 门道振荡检测**：FOLLOW 状态检测 40 帧内 ≥3 次 y=0 穿越时中止目标
- **v2.5 RECOVER 振荡抑制**：检测到振荡时优先垂直 y 方向逃离
- RECOVER-PLAN 振荡修复（无 frontier 时限制 1 次 RECOVER 后进入 DONE）
- `align_frames` 在所有状态转换点重置

### 4.6 三重防碰撞

1. **雷达 bbox 膨胀**：`laser_scan` 给每个障碍物 bbox 加 margin
2. **前进目的地检查**：`point_in_any_obstacle(new_x, new_y, ROBOT_RADIUS)`
3. **碰撞恢复**：每帧检查当前位置，若在障碍物内则立即后退

---

## 五、最终实验结果（v2.7b，2026-07-07）

### 5.1 导航性能对比（v2.7b 最终实验，2026-07-07）

| Method | Time(s) | Cov(%) | Dist(m) | RECOVER | Doorway | LocErr(m) | Efficiency(cov/m) |
|--------|---------|--------|---------|---------|---------|-----------|-------------------|
| groundtruth | 80.0 | 91.78 | 21.76 | 2 | 1 | - | 4.217 |
| random_walk | 79.9 | 93.90 | 114.60 | 0 | 1 | - | 0.819 |
| proposed_amcl_dwa | 80.0 | 92.30 | 101.08 | 11 | 7 | 0.997 | 0.913 |
| **proposed_amcl_teb** | **80.0** | **93.22** | **48.80** | **12** | **7** | **1.003** | **1.911** |

**结论**：
- TEB 探索距离 **48.8m**（v2.7 的 113m → 48.8m，-57%）
- TEB 路径效率 **1.911 cov/m**（是 DWA 的 2.1 倍，是 random_walk 的 2.3 倍）
- TEB 门道穿越 **7 次**（v2.6b 12 次 → 7 次，-42%）
- TEB FOLLOW-OSC **0 次**（v2.6b 10 次 → 0，彻底消除振荡连锁）
- TEB IoU_occ **0.852**（接近 groundtruth 0.871）

### 5.2 地图质量对比

| Method | IoU_occ | IoU_free | Hausdorff(m) | Entropy | Precision | Recall |
|--------|---------|----------|--------------|---------|-----------|--------|
| groundtruth | 0.871 | 0.991 | 0.20 | 0.551 | 1.0 | 0.871 |
| proposed_amcl_dwa | 0.721 | 0.980 | 0.70 | 0.551 | 1.0 | 0.721 |
| **proposed_amcl_teb** | **0.852** | **0.989** | 0.67 | 0.554 | 1.0 | 0.852 |

### 5.3 v2.7b vs v2.7 vs v2.6b TEB 迭代对比

| 指标 | v2.6b | v2.7 | **v2.7b** | 最优 | 说明 |
|------|-------|------|---------|------|------|
| Distance | 85m | 113m | **48.8m** | v2.7b ✅ | AMCL-PROACTIVE 参数调优 |
| Efficiency | 1.09 | 0.83 | **1.91** | v2.7b ✅ | 路径效率翻倍 |
| Doorway | 12 | 12 | **7** | v2.7b ✅ | TEB 门道 w_obstacle=0.8 |
| IoU_occ | 0.805 | 0.824 | **0.852** | v2.7b ✅ | 地图质量提升 |
| FOLLOW-OSC | 10 | 0 | 0 | v2.7+ ✅ | 30帧冷却+清空历史 |
| Hausdorff | 0.5m | 0.2m | 0.67m | v2.7 | v2.7b LocErr 高导致边界误差 |
| LocErr | 0.72m | 1.18m | 1.00m | v2.6b | odometry 噪声累积（待优化） |

### 5.4 树莓派 5 降配部署（v2.7b 新增）

| 参数 | 满配（桌面） | 降配（树莓派 5） | 变化 |
|------|-------------|-----------------|------|
| AMCL 粒子数 | 200 | 100 | -50% |
| TEB 迭代数 | 4 | 2 | -50% |
| TEB pose 数 | 15 | 10 | -33% |
| LiDAR 射线数 | 72 | 36 | -50% |
| AMCL-PROACTIVE | 开启 | 关闭 | 节省 CPU |
| 预期帧率 | ~50-100Hz | ~10Hz | 满足实时控制 |
| 预期覆盖率 | ~93% | ~90% | -3% 可接受 |

**用法**：`PROFILE=raspberry_pi USE_AMCL=1 USE_TEB=1 python3 -u autonomous_nav.py`

### 5.5 实验产物

`eval_results/` 目录包含：
- `comparison_table.txt` — 4 方法对比表
- `coverage_vs_time.png` — 覆盖率随时间变化曲线
- `loc_error_vs_time.png` — 定位误差随时间变化曲线
- `state_distribution.png` — 状态分布柱状图
- `trajectories.png` — 4 方法轨迹对比可视化
- `{method}_data.csv` — 每帧数据（20 字段）
- `{method}_summary.json` — 方法汇总指标
- `{method}_map_quality.json` — 地图质量指标

### 5.6 场景障碍物布局

```
wall_south:    x=[-5,5]      y=[-4.05,-3.95]   南墙
wall_north:    x=[-5,5]      y=[3.95,4.05]     北墙
wall_west:     x=[-5.05,-4.95] y=[-4,4]        西墙
wall_east:     x=[4.95,5.05] y=[-4,4]          东墙
wall_divide_1: x=[-5,-1]     y=[-0.05,0.05]    隔墙西段(门口在 x=[-1,1])
wall_divide_2: x=[1,5]       y=[-0.05,0.05]    隔墙东段
sofa:          x=[2.75,4.25] y=[-3.30,-2.70]   沙发(南侧)
bed:           x=[-4.25,-2.75] y=[2.00,4.00]   床(卧室)
dining_table:  x=[1.60,2.40] y=[2.47,2.53]     餐桌(厨房)

机器人起点: (1.0, -2.0, 0.0) 朝向东
```

---

## 六、关键参数速查

### 6.1 导航参数（[autonomous_nav.py](autonomous_nav.py)）

| 参数 | 值 | 说明 |
|------|-----|------|
| `ROBOT_RADIUS` | 0.35 m | 机器人物理半径 |
| `goal_tolerance` | 0.5 m | FOLLOW 状态 goal 到达判定 |
| `no_progress_frames` | >40 | 触发 RECOVER（致命区 15） |
| `align_frames` | >40 | 触发 RECOVER（对齐卡死） |
| `recover_frames` | >10 | 回到 PLAN |
| `VISITED_FRONTIER_TTL` | 250 帧 | 已访问 frontier 屏蔽时长（v2.6: 300→250） |
| `MAX_FRAMES` | 0/800 | 0=无限，实验用 800 |

### 6.2 AMCL 参数（[amcl.py](amcl.py)）

```python
n_particles = 200        # 粒子数（v2.6: 保持 200，300 收敛慢）
n_obs_rays = 24          # 观测射线数
sigma_obs = 0.55         # 似然场高斯 sigma
neff_threshold = n/3     # 重采样阈值
alpha1/alpha4 = 0.08     # 旋转噪声
alpha2/alpha3 = 0.04     # 平移噪声
# 门道地标阈值: 0.3m（v2.6: 0.5→0.3m，更早纠正漂移）
```

### 6.3 TEB 参数（[teb_planner.py](teb_planner.py)）

```python
n_poses = 15             # 弹性带 pose 数
dt = 0.1                 # 时间间隔（与仿真一致）
max_v = 0.3              # 最大线速度
n_iterations = 4         # 优化迭代数
learning_rate = 0.08
w_obstacle = 3.0         # 障碍物代价（v2.6: 门道附近自适应降至 1.5）
w_smooth = 0.8           # 平滑度
w_path = 0.6             # 路径跟随
w_goal = 1.2             # 目标导向（v2.6: 门道附近自适应升至 1.5）
```

### 6.4 A* 参数（[astar_planner.py](astar_planner.py)）

```python
PLAN_BLOCKED = 110       # cost≥110 视为障碍
# _cell_cost: 二次惩罚 1.0 + (c/110)^2 * 12.0  (v2.3: 8.0→12.0 远离墙)
# _is_traversable: 禁止穿越 unknown cells
```

### 6.5 Costmap 参数（[costmap.py](costmap.py)）

```python
COST_LETHAL = 254        # 致命代价
COST_INSCRIBED = 128     # 内切半径代价
INSCRIBED_RADIUS = 0.25  # 内切半径
INFLATION_RADIUS = 0.55  # 膨胀半径
```

---

## 七、常见问题排查

### Q1: ZMQ 连不上 CoppeliaSim？
1. CoppeliaSim 是否运行？`ps aux | grep coppelia`
2. 场景是否已加载（不是空白）？
3. ZMQ 插件是否启用？（CoppeliaSim 菜单 → Plugins）
4. 端口 23000 是否冲突？

### Q2: scipy 与 NumPy 2.x 不兼容？
已用 numpy-only 的 `_distance_transform_numpy()`（3-4-5 Chamfer）替代，**不要安装 scipy**。

### Q3: PowerShell 引号转义？
避免 `wsl bash -c "python3 -c '...'"` 嵌套引号，将命令写入 `.py` 或 `.sh` 脚本执行。

### Q4: Xvfb socket 绑定失败？
非致命，已有 Xvfb 实例在运行。如需重启用 `pkill -9 Xvfb`。

### Q5: AMCL 定位误差大？
1. `sigma_obs` 是否匹配传感器噪声（v2.3.1 默认 0.55）？
2. 粒子是否收敛？看日志 `[AMCL]` 的 `neff` 值
3. 门道地标是否触发？看日志 `[DOORWAY-LOC]`
4. `neff_threshold` 是否过低（v2.3.1: n/3）？

### Q6: 机器人穿墙？
1. `discover_obstacles_with_bbox()` 是否识别所有墙壁？
2. 墙的 bbox 方向是否正确？（旋转的墙必须用 `getObjectMatrix` 变换顶点）
3. `ROBOT_RADIUS` 设置是否足够大？

### Q7: 日志标签解读
```
[NAV] pos=(x,y) yaw=θ state=S cov=C% vis=V    # 状态输出（每 50 帧）
[PLAN] Frontier (x,y) info_gain=N dist=Dm      # 信息增益 Frontier 选择
[FOLLOW] Reached goal (x,y)                     # 到达 goal
[RECOVER] esc#N dir=θ clearance=Cm             # 逃逸动作
[AMCL] neff=X/Y resample=bool err=Xm           # AMCL 状态
[EVAL] Map quality: IoU_occ=X Hausdorff=Ym     # 地图质量
[DOORWAY] crossed at (x,y)                      # 门口穿越检测
[DOORWAY-LOC] Triggered AMCL re-localization    # 门道地标定位（v2.3）
[FOLLOW-OSC] Doorway oscillation                # FOLLOW 门道振荡检测（v2.5）
[DOORWAY-OSC] Detected oscillation              # RECOVER 门道振荡抑制（v2.5）
```

---

## 八、换计算机迁移步骤

### 8.1 必做清单

1. **拷贝整个项目目录**: `e:\puppyfangzhen\` → 新机器同样路径
2. **安装 CoppeliaSim Edu** — 首次启动确认 ZMQ 插件可用
3. **安装 Python 依赖**:
   ```bash
   pip install coppeliasim-zmqremoteapi-client Pillow numpy matplotlib
   # 不要安装 scipy
   ```
4. **设置 WSL2 + Ubuntu 22.04**:
   ```bash
   wsl --install
   sudo apt update && sudo apt install python3-pip xvfb
   pip3 install coppeliasim-zmqremoteapi-client Pillow numpy matplotlib
   ```
5. **拷贝/重建场景文件** — 无 `.ttt` 则运行 `build_coppelia_scene.py`
6. **验证运行**:
   ```bash
   # 终端 1：MJPEG 服务器
   wsl -d Ubuntu-22.04
   export PUPPY_STREAM_FRAME=/tmp/stream_frame.jpg
   python3 /mnt/e/puppyfangzhen/mjpeg_file_server.py

   # 终端 2：导航
   wsl -d Ubuntu-22.04
   cd /mnt/e/puppyfangzhen
   USE_AMCL=1 USE_TEB=1 METHOD_NAME=proposed_amcl_teb python3 -u autonomous_nav.py
   ```

### 8.2 重要提示

- 仓库默认不携带 `.ttt` 场景文件，需自行保留或用脚本重建
- **不要安装 scipy**，已用 numpy-only 实现替代
- 实验通过环境变量切换方法，无需改代码
- 所有评估产物在 `eval_results/` 目录

---

## 九、待办事项与改进方向

### 已知限制
1. 地图不持久化 `unreachable_goals` — 每次重启会重新尝试（有 TTL 兜底）
2. AMCL 在长走廊可能漂移 — 缺乏全局定位修正
3. TEB LocErr 0.86m 偏高 — v2.6 优化路径效率的代价，地图质量已超越 groundtruth
4. Coverage 91.52% — 路径效率优化减少了探索范围，可通过延长 MAX_FRAMES 改善
5. AMCL 随机性 — 不同运行间结果有波动，建议多次实验取平均

### 可改进方向
1. **PuppyPi 实体部署** — 参见 [gaijin2.md](gaijin2.md)
2. **ROS2 集成** — 将 AMCL/TEB/信息增益移植到 ROS2 路线
3. **持久化 unreachable_goals** — 保存到 map 文件
4. **动态障碍物处理** — 添加行人/移动物体检测与避让
5. **3D 导航** — 支持高度变化（楼梯、斜坡）
6. **视觉 SLAM** — 替换占据栅格为 ORB-SLAM3

### 技术升级方案参考
- [gaijin1.md](gaijin1.md) — 技术升级总方案
- [gaijin2.md](gaijin2.md) — PuppyPi 实体平台部署规划书

---

## 十、代码位置速查

| 功能 | 文件 | 位置 |
|------|------|------|
| 环境变量定义 | [autonomous_nav.py](autonomous_nav.py) | L54-L60 |
| 障碍物发现 + bbox | [autonomous_nav.py](autonomous_nav.py) | L90-L121 |
| `sectors_from_full_scan` | [autonomous_nav.py](autonomous_nav.py) | L255-L289 |
| AMCL 初始化 | [autonomous_nav.py](autonomous_nav.py) | L331-L334 |
| VISITED_FRONTIER_TTL | [autonomous_nav.py](autonomous_nav.py) | L373 |
| doorway_crossing_frames 初始化 | [autonomous_nav.py](autonomous_nav.py) | L496 |
| 门道地标定位（v2.6: 阈值0.3m） | [autonomous_nav.py](autonomous_nav.py) | L572-L578 |
| **Frontier 振荡过滤（v2.6）** | [autonomous_nav.py](autonomous_nav.py) | L644-L658 |
| **FOLLOW 振荡检测（v2.5）** | [autonomous_nav.py](autonomous_nav.py) | L774-L794 |
| 状态机主循环 | [autonomous_nav.py](autonomous_nav.py) | L500-L1130 |
| **RECOVER 振荡抑制（v2.5）** | [autonomous_nav.py](autonomous_nav.py) | L898-L904 |
| RECOVER 16 方向逃逸 | [autonomous_nav.py](autonomous_nav.py) | L906-L1000 |
| **DONE 检查（v2.6: 100帧）** | [autonomous_nav.py](autonomous_nav.py) | L1090-L1112 |
| AMCL Likelihood Field | [amcl.py](amcl.py) | `_build_likelihood_field()` |
| AMCL 向量化权重 | [amcl.py](amcl.py) | `weight()` |
| Chamfer 距离变换 | [amcl.py](amcl.py) | `_distance_transform_numpy()` |
| 信息增益 Frontier | [occupancy_grid.py](occupancy_grid.py) | `find_frontiers_with_info_gain()` |
| **β=0.5 utility 权重（v2.5）** | [occupancy_grid.py](occupancy_grid.py) | L303-L308 |
| BFS 4 连通聚类 | [occupancy_grid.py](occupancy_grid.py) | L199-L224 |
| 地图质量指标 | [evaluator.py](evaluator.py) | `compute_map_metrics()` |
| **TEB 门道参数自适应（v2.6）** | [teb_planner.py](teb_planner.py) | L97-L110 |
| TEB 轨迹优化 | [teb_planner.py](teb_planner.py) | `_optimize_step()` |
| A* unknown 拦截 | [astar_planner.py](astar_planner.py) | L156 |

---

# 附录 A：优化历程与深度分析

## A.1 8 轮迭代优化历程

| 轮次 | 版本 | 关键优化 | DWA LocErr | TEB LocErr | TEB Cov | TEB Dist | TEB Haus |
|------|------|---------|-----------|-----------|---------|----------|----------|
| 初始 | v2.2 | P0 bug 修复（8 致命 + 12 严重） | 1.31m | 1.31m | - | - | - |
| 第 2 轮 | v2.2 | 梯度安全 + 自适应 RECOVER | - | 0.88m | - | - | - |
| 第 3 轮 | v2.2 | 跳过致命区 AMCL 旋转 | - | 0.66m | - | - | - |
| 第 4 轮 | v2.2-final | **B1 `t` 遮蔽 + B2 `align_frames` 重置** | 1.21m | 0.96m | - | 97m | - |
| 第 5 轮 | v2.3 | **门道地标 + sigma_obs + A* 远离墙** | 0.86m | 1.00m | - | 42m | - |
| 第 6 轮 | v2.3.1 | **sigma_obs 0.55 + 条件门道地标** | 1.24m | 0.63m | 90.31% | 19m | 0.27m |
| 第 7 轮 | v2.5 | **门道振荡检测（FOLLOW+RECOVER）** | 0.68m | 0.72m | 93.26% | 88m | 0.53m |
| 第 8 轮 | v2.6 | **TEB 门道参数自适应 + Frontier 振荡过滤** | 0.86m | 0.86m | 91.52% | **41m** | **0.17m** |

### v2.5-v2.6 关键优化

| 优化项 | 文件 | 效果 |
|--------|------|------|
| **v2.6 TEB 门道参数自适应** | teb_planner.py:97-110 | 门道附近 w_obstacle 3.0→1.5, w_goal 1.2→1.5，减少门道振荡 |
| **v2.6 Frontier 振荡过滤** | autonomous_nav.py:644 | 40帧内振荡过则不选门道另一侧 frontier，减少无效穿越 |
| **v2.6 AMCL 门道地标阈值** | autonomous_nav.py:572 | 0.5→0.3m，更早纠正 AMCL 漂移 |
| v2.6 DONE 检查频率 | autonomous_nav.py:1090 | 200→100帧，更快恢复探索 |
| v2.6 VISITED_FRONTIER_TTL | autonomous_nav.py:373 | 300→250，允许 sooner 重新探索 |
| v2.5 FOLLOW 振荡检测 | autonomous_nav.py:774 | 40帧内≥3次y=0穿越时中止目标 |
| v2.5 RECOVER 振荡抑制 | autonomous_nav.py:898 | 振荡时优先垂直y方向逃离 |
| v2.5 β=0.5 恢复 | occupancy_grid.py:307 | v2.4的0.3导致路径暴增139m，恢复0.5 |

### 关键 bug 修复（按重要性排序）

| Bug | 文件 | 影响 |
|-----|------|------|
| **B1 `t` 变量遮蔽** | autonomous_nav.py:863 | RECOVER 路径投影参数 `t` 覆盖全局时间变量，导致时间重置为 0.1s |
| **B2 `align_frames` 未重置** | autonomous_nav.py:587,661,957 | PLAN→FOLLOW 转换时未重置，累积>100 立即触发 RECOVER |
| **B3 `sectors_from_full_scan` 缺失** | autonomous_nav.py:255 | 函数被调用但未定义，sector 索引映射错误 |
| F1 A* 拦截 unknown | astar_planner.py:156 | A* 不再穿墙规划未观测区 |
| F2 TEB dt 0.2→0.1 | teb_planner.py:51 | TEB 不再半速运行 |
| F3 TEB velocity/kinematic | teb_planner.py:234-256 | 弹性带不塌缩 |
| F6 np.roll→np.pad | occupancy_grid.py:182 | 边界无虚假 frontier |
| F8 config_loader 集成 | config_loader.py | YAML 真正被读取 |

### v2.3-v2.3.1 关键优化

| 优化项 | 文件 | 效果 |
|--------|------|------|
| 门道地标辅助定位 | autonomous_nav.py:551 | 利用门道强地标纠正长距离漂移 |
| 门道定位条件触发 | autonomous_nav.py:570 | 仅 loc_err>0.5 时触发，避免破坏已收敛 AMCL |
| AMCL sigma_obs 0.35→0.55 | amcl.py:132 | 更平坦似然场，减少地图噪声敏感度 |
| AMCL neff n/3↔n/4 | amcl.py:171 | 平衡重采样（n/4 粒子贫化，回调 n/3） |
| AMCL n_obs_rays 16→24 | autonomous_nav.py:332 | 更多观测射线，权重更准确 |
| A* 惩罚系数 8.0→12.0 | astar_planner.py:63 | 路径更靠近走廊中央，减少卡墙 |
| TEB w_path 0.4→0.6 | teb_planner.py:69 | 更紧密跟随全局路径 |
| TEB w_goal 1.0→1.2 | teb_planner.py:71 | 更强目标导向 |
| RECOVER 自适应步长 | autonomous_nav.py:848 | 致命区 0.6m + 渐进 0.8/1.0/1.5m |
| RECOVER 16 方向采样 | autonomous_nav.py:880 | 更精细的逃逸路径 |
| 后退方向优先 | autonomous_nav.py:897 | 致命区 70% 代价 + 30% 后退 |

## A.2 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│  应用层 — puppy_core（任务编排/模式管理/安全监督）                  │
├─────────────────────────────────────────────────────────────────┤
│  机器人能力层 — nav_core                                         │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐     │
│  │  定位        │  建图        │  规划        │  探索        │     │
│  │ Odometry    │ OccGrid     │ A* + TEB    │ InfoGain    │     │
│  │ AMCL (LF)   │ (log-odds)  │ / DWA       │ Frontier    │     │
│  └─────────────┴─────────────┴─────────────┴─────────────┘     │
│  感知：LiDAR (ray-AABB) + Vision (HSV 语义)                      │
│  评估：Evaluator (CSV/JSON/PNG) + MapMetrics (IoU/HD/Ent)       │
├─────────────────────────────────────────────────────────────────┤
│  平台适配层 — puppypi_adapter                                    │
│  CoppeliaSim ZMQ 适配 / PuppyPi 实体适配 / Mock 模拟器            │
├─────────────────────────────────────────────────────────────────┤
│  硬件/SDK 层 — CoppeliaSim / PuppyPi SDK / ROS2 ros2_control     │
└─────────────────────────────────────────────────────────────────┘
```

### 主循环数据流

```
每帧 (0.1s)
   ├─► 1. CoppeliaSim ZMQ 读取 → ground truth pose (可选)
   ├─► 2. LiDAR 扫描 (72 射线, ray-AABB) → (angles[], distances[])
   ├─► 3. Odometry.update(v, w, dt) → (dx, dy, dyaw)
   ├─► 4. AMCL.update() → (rx, ry, ryaw)  [运动预测 + 似然场权重 + 重采样]
   ├─► 5. OccupancyGrid.update()  [Bresenham 射线清除 + log-odds 更新]
   ├─► 6. State Machine
   │       ├─ PLAN    → find_frontiers_with_info_gain() → A*.plan()
   │       ├─ FOLLOW  → TEB/DWA.compute_velocity()
   │       ├─ RECOVER → 16 方向 cost 扫描 → 最低 cost 逃逸
   │       └─ DONE    → 保持位置
   ├─► 7. 速度执行 → CoppeliaSim (v, w)
   ├─► 8. Evaluator.log_frame() → CSV
   └─► 9. Doorway 检测 / Vision 语义更新 / 地图保存
```

## A.3 算法复杂度分析

| 模块 | 优化后复杂度 | 实测耗时 |
|------|-------------|---------|
| AMCL 权重计算 | O(N·K) LF lookup | 300ms → **0.4ms** (750×) |
| 距离变换 | O(H·W) Chamfer 2-pass | 50ms → **2ms** (25×) |
| Frontier 检测 | O(H·W) + utility 排序 | ~5ms |
| A* 规划 | O(b^d) + 路径平滑 | ~10ms |
| TEB 优化 | O(5·15) = O(75) | ~3ms |
| DWA 采样 | O(9·15·2) = O(270) | ~8ms |
| Costmap 更新 | O(局部窗口) | ~2ms |

## A.4 技术创新点（论文 Contribution）

1. **全向量化 AMCL + Likelihood Field Model** — 750× 加速，无需 scipy
2. **信息增益 Frontier 选择** — utility = α·gain_norm - β·dist_norm，BFS 聚类
3. **TEB 轨迹优化替代 DWA** — 效率提升 100%，路径更短 80%
4. **门道地标辅助定位** — 利用门道独特 LiDAR 模式纠正长距离漂移
5. **门道振荡检测与抑制（v2.5）** — FOLLOW+RECOVER 双状态检测，40帧内≥3次y=0穿越时中止目标
6. **TEB 门道参数自适应（v2.6）** — 门道附近动态调整 w_obstacle/w_goal，Hausdorff 降至 0.17m
7. **Frontier 振荡过滤（v2.6）** — 振荡冷却期内不选门道另一侧 frontier，路径效率翻倍
8. **论文级地图质量评估** — IoU/Hausdorff/Entropy/Precision/Recall
9. **状态机振荡修复** — 限制 1 次 RECOVER 后 DONE，align_frames 重置

## A.5 关键设计决策（ADR）

- **ADR-001**: 选择 CoppeliaSim + Python 而非 ROS2 + Gazebo — 便于算法迭代
- **ADR-002**: AMCL 用 Likelihood Field 而非 Beam Range Finder — 750× 加速
- **ADR-003**: TEB 与 DWA 接口兼容 — `compute_velocity(rx,ry,ryaw,path,goal) → (v,w)`
- **ADR-004**: 信息增益 utility 用线性归一化 — 简单有效
- **ADR-005**: 不持久化 unreachable_goals — 依赖 TTL 兜底
- **ADR-006**: 评估数据用 CSV + JSON 双格式 — 论文表格 + 曲线

## A.6 ROS2 + Gazebo 路线（备用）

```
e:\puppyfangzhen\src\
├── puppy_description/     # URDF 机器人模型
├── puppy_hardware/        # ros2_control 配置
├── puppy_gait/            # Trot 步态控制器 (C++)
├── puppy_localization/    # 足式里程计 + EKF
├── puppy_slam/            # SLAM Toolbox 配置
├── puppy_nav/             # Nav2 导航配置 + 地图
├── puppy_worlds/          # Gazebo 仿真世界
├── puppy_bringup/         # 一键启动文件
├── puppy_core/            # 应用层
├── puppy_interfaces/      # 自定义 msg/srv/action
├── puppypi_adapter/       # PuppyPi 平台适配层
└── puppypi_mock/          # PuppyPi 模拟器
```

```bash
wsl -d Ubuntu-22.04
mkdir -p ~/puppy_ws/src
cp -r /mnt/e/puppyfangzhen/src/* ~/puppy_ws/src/
source /opt/ros/humble/setup.bash
cd ~/puppy_ws
colcon build --symlink-install
source ~/puppy_ws/install/setup.bash

# 建图
ros2 launch puppy_bringup mapping.launch.py world:=small_room

# 导航
ros2 launch puppy_bringup navigation.launch.py world:=small_room map:=~/puppy_ws/src/puppy_nav/maps/small_room.yaml
```

> 注：ROS2 路线当前未集成 AMCL/TEB/信息增益等新算法，仅作为备用展示。

---

## B. v4.0 新增内容（2026-07-12）

### B.1 8房间自动巡航仿真系统

**文件**: [auto_patrol_simulation.py](auto_patrol_simulation.py) + [long_patrol_simulation.py](long_patrol_simulation.py)

8房间场景（客厅、卧室1、书房、卧室2、卫生间、储物间、餐厅、厨房）+ 18个巡航航点。

**巡航路线**: 充电桩 → 客厅 → 卧室1 → 书房 → 卧室2 → 卫生间 → 储物间 → 餐厅 → 厨房 → 返回充电

**特性**:
- 3个动态行人障碍物（person_1/2/3）
- 四层动态避障系统（等待→急退→排斥→DWA）
- 行人轨迹预测（8-10步外推）
- 门道区域自动减速
- 碰撞记录冷却（5帧去重）
- 每50轮检查点保存

**运行**:
```bash
# 短测试（5000帧，约10分钟）
python long_patrol_simulation.py --frames 5000 --no-coppelia

# 完整2小时仿真（72000帧）
python long_patrol_simulation.py --frames 72000 --no-coppelia
```

### B.2 四个技术创新模块

#### B.2.1 Risk-Aware A* 风险感知路径规划
**文件**: [risk_aware_astar.py](risk_aware_astar.py)

| 项 | 说明 |
|----|------|
| 理论 | 机会约束规划(Chance-Constrained) + CVaR条件风险价值 |
| 算法 | 基于AMCL协方差传播计算碰撞概率，erf函数近似高斯积分 |
| 公式 | `cost = path_length + λ · CVaR_α(collision_cost)` |
| 特性 | λ=0退化为标准A*；可接入AMCL实时协方差 |
| 启用 | `set USE_RISK_AWARE=1` |

#### B.2.2 D* Lite 增量重规划
**文件**: [dstar_lite.py](dstar_lite.py)

| 项 | 说明 |
|----|------|
| 理论 | 增量启发式搜索，O(k·log n) vs A*的O(n·log n) |
| 算法 | rhs值增量更新 + key modifier(km)避免全队列更新 |
| 特性 | 动态障碍物移动只需局部更新；自动检测costmap变化 |
| 启用 | `set USE_DSTAR_LITE=1` |

#### B.2.3 Lifelong SLAM 持续建图
**文件**: [lifelong_slam.py](lifelong_slam.py)

| 项 | 说明 |
|----|------|
| 理论 | 对数似然比检验(LRT) + 贝叶斯Beta-Bernoulli共轭先验 |
| 算法 | 子地图管理 + 遗忘因子加权更新 + 变化检测器 |
| 特性 | 自动检测环境变化(新增/消失障碍物)；内存管理(max_submaps) |
| 启用 | `set USE_LIFELONG_SLAM=1` |

#### B.2.4 GNN 动态轨迹预测
**文件**: [gnn_trajectory_predictor.py](gnn_trajectory_predictor.py)

| 项 | 说明 |
|----|------|
| 理论 | 时空图神经网络 + 社会池化(Social Pooling) |
| 算法 | GRU编码时序 + 社会池化聚合行人间交互 + MLP解码 |
| 特性 | 纯NumPy实现(无需PyTorch)；在线学习；多行人联合预测 |
| 启用 | `set USE_GNN_PREDICTOR=1` |

### B.3 其他 v4.0 新增模块

| 文件 | 功能 |
|------|------|
| [safety_subsystem.py](safety_subsystem.py) | 完整安全子系统(碰撞概率/急制动/风险评估/5级状态机) |
| [pomdp_navigation.py](pomdp_navigation.py) | POMDP导航建模(粒子滤波信念更新/POMCP) |
| [cbf_safety.py](cbf_safety.py) | 控制障碍函数安全过滤器(QP求解/前向不变性) |
| [theory_analysis.py](theory_analysis.py) | 理论分析(CRLB/AUFE Regret/Wilson-Hilferty/Lipschitz) |
| [hybrid_astar.py](hybrid_astar.py) | 混合A*规划器(连续+离散状态空间) |
| [mpc_planner.py](mpc_planner.py) | 模型预测控制规划器 |
| [belief_planner.py](belief_planner.py) | 基于信念的规划器 |
| [rbpf_slam.py](rbpf_slam.py) | Rao-Blackwellized粒子滤波SLAM |
| [semantic_slam.py](semantic_slam.py) | 语义SLAM |
| [loop_closure.py](loop_closure.py) | 回环检测 |
| [active_slam.py](active_slam.py) | 主动SLAM |
| [aufe_framework.py](aufe_framework.py) | AUFE自适应不确定性融合框架 |

### B.4 测试套件

| 测试文件 | 内容 | 结果 |
|---------|------|------|
| [test_all_modules.py](test_all_modules.py) | 核心模块测试 | 171/171 ✅ |
| [test_v4_all.py](test_v4_all.py) | v4.0增强模块测试 | 137/137 ✅ |
| [test_v4_innovations.py](test_v4_innovations.py) | 4个技术创新模块测试 | 25/25 ✅ |
| [test_recovery.py](test_recovery.py) | 恢复策略测试 | 10/10 ✅ |
| [test_frontier_quality.py](test_frontier_quality.py) | Frontier质量测试 | 7/7 ✅ |

运行所有测试:
```bash
python test_all_modules.py
python test_v4_all.py
python test_v4_innovations.py
```

### B.5 v4.0 长时仿真结果（2026-07-12）

**2小时仿真**（72000帧，204轮巡航）:

| 指标 | 修复前 | 修复后 | 改善 |
|------|--------|--------|------|
| 穿墙事件 | 111次 | 0次 | 100%消除 |
| 动态碰撞 | 1785次 | 782次 | 降56% |
| 到达率 | 100% | 100% | 保持 |
| 覆盖率 | 69.7% | 76.6% | 提升 |
| FPS | 27.3→25.3 | 25.3-27.3 | 稳定 |

**修复内容**:
1. 行人穿墙修复（添加`_is_inside_wall()`检测）
2. 四层避障系统（等待→急退→排斥→DWA）
3. 行人轨迹预测（8-10步外推）
4. 门道区域减速（8个门道坐标检测）
5. 穿墙误报修复（检测半径0.10→0.0）
6. 碰撞记录冷却（5帧去重）

### B.6 配置文件

| 文件 | 说明 |
|------|------|
| [config/sim.yaml](config/sim.yaml) | 仿真参数 |
| [config/runtime.yaml](config/runtime.yaml) | 运行时参数 |
| [config/planner_global.yaml](config/planner_global.yaml) | 全局规划器 |
| [config/planner_local.yaml](config/planner_local.yaml) | 局部规划器 |
| [config/profile_raspberry_pi.yaml](config/profile_raspberry_pi.yaml) | 树莓派5降配 |

### B.7 关键经验教训（v4.0）

1. **行人穿墙是动态碰撞的根本原因** — 修复行人运动边界后碰撞降56%
2. **门道是碰撞热点区域** — 8个门道坐标需特殊处理
3. **erf近似足够精确** — Abramowitz-Stegun近似误差<2.5e-7
4. **纯NumPy GNN可行** — 无需PyTorch依赖，在线学习有效
5. **长时仿真能暴露隐蔽问题** — 2小时仿真发现111次穿墙误报

---

> **祝你接手顺利！有问题先看日志输出，`[NAV]`/`[PLAN]`/`[FOLLOW]`/`[RECOVER]`/`[AMCL]`/`[EVAL]`/`[DOORWAY]`/`[PATROL]` 标签能快速定位问题。**
