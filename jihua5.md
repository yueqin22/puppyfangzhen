# Puppy 仿真核心与 C++ 化优化计划 v5.0

> 文档版本：v5.0  
> 更新日期：2026-07-25  
> 依据文档：`交接5.md`  
> 适用范围：`cpp_src/sim`、`cpp_src/puppy_nav_core`、`src/*` ROS2 工作区、工程化整理与实验验证  
> 核心目标：在 v3.2.2 已达到“可运动 + 0 碰撞 + A* 93%”的基础上，提高稳定性、可复现性、C++ 工程化程度和论文可信度。

---

## 一、当前状态判断

### 1.1 已达成成果

根据 `交接5.md`，当前 v3.2.2 已完成 6 轮关键优化：

| 模块 | 当前状态 | 结果 |
|------|----------|------|
| AMCL | 修复随机粒子注入过散、置信度系数过敏 | 置信度从 0.042 提升到 0.491 |
| A* | 提升搜索节点数、扩大 nearest-free 半径、修复膨胀核阻塞 | 成功率达到 93.0% |
| Costmap | 动态障碍改为软代价 | 行人不再硬切断通路 |
| Simulation | 启动期降速、行人位置调整 | 1 小时 0 碰撞 |
| 验证 | collision/wall/process 全 PASS | 小车确认为真实运动，不是假稳定 |

### 1.2 主要剩余问题

| 优先级 | 问题 | 风险 |
|--------|------|------|
| P0 | 行人反弹逻辑 vx/vy 方向错误 | 当前靠调整行人方向规避，场景泛化差 |
| P0 | 0 碰撞只有 1 小时单次验证 | 随机性下可能不稳定，论文证据不够强 |
| P1 | AMCL 置信度与定位误差不一致 | 影响定位质量评价与自适应策略可信度 |
| P1 | KLD-Sampling 粒子数长期不收敛 | 算法声称与实际表现之间有缺口 |
| P1 | Python/C++ 双实现并存 | 运行路径不清晰，后续维护容易改错地方 |
| P2 | 工程产物、实验结果、源码混杂 | 交付、复现、版本管理困难 |
| P2 | launch 使用固定延迟 | 不同机器上启动稳定性不够 |

---

## 二、总体优化路线

本阶段建议采用“先稳仿真，再固化 C++，最后补论文证据”的路线：

```text
P0 稳定性闭环
  修复行人反弹 bug
  多随机种子压力测试
  固定验收指标

P1 算法可信度
  改进 AMCL 置信度
  重新审视 KLD-Sampling
  加入真实性诊断指标

P2 C++ 工程化
  明确 cpp_src 与 src 的迁移关系
  将运行时节点逐步迁移到 ament_cmake/rclcpp
  Python 退为实验和诊断层

P3 复现实验
  12 小时压力测试
  多场景/多种子/多消融对比
  生成论文表格和图
```

---

## 三、P0：仿真稳定性闭环

### 3.1 修复行人反弹逻辑

**目标**：让行人可以自然进行南北/东西双方向运动，而不是靠“全部改成东西向”规避 bug。

**涉及文件**：

- `cpp_src/sim/simulation.h`

**计划改动**：

当前逻辑中，`new_x,p.y` 检查失败时表示 x 方向撞墙，应反弹 `vx`；`p.x,new_y` 检查失败时表示 y 方向撞墙，应反弹 `vy`。需要把 `交接5.md` 中记录的 vx/vy 反弹方向修正。

**验收标准**：

- 南北向行人不会卡在水平墙边；
- 东西向行人不会卡在竖直墙边；
- 运行 1 小时，`collision_count == 0`；
- A* 成功率保持 `>= 90%`；
- 记录行人轨迹，确认不是“行人不动换稳定”。

### 3.2 建立多种子稳定性测试

**目标**：避免单次 1 小时 0 碰撞带来的偶然性。

**涉及文件**：

- `cpp_src/sim/main.cpp`
- `cpp_src/sim/simulation.h`
- 新增：`cpp_src/sim/run_stability_suite.ps1`

**计划改动**：

- 给仿真入口增加随机种子参数；
- 固定输出 JSON/CSV 指标；
- 一次运行 10 个 seed，每个 seed 至少 1 小时；
- 汇总最差值、均值、方差。

**验收标准**：

| 指标 | 要求 |
|------|------|
| seed 数 | >= 10 |
| 每 seed 时长 | >= 1 小时 |
| 碰撞次数 | 全部为 0 |
| wall penetration | 全部为 0 |
| A* 成功率 | 最低 >= 85%，均值 >= 90% |
| 进程存活 | 全部 true |

### 3.3 加入“真实性指标”

**目标**：把“0 碰撞但车不动”的问题永久挡住。

**新增指标**：

| 指标 | 含义 | 建议阈值 |
|------|------|----------|
| total_distance | 小车累计运动距离 | 1 小时 >= 10m |
| unique_rooms_visited | 访问房间数 | >= 3 |
| goal_completion_count | 完成目标数 | >= 1 |
| average_speed_when_active | 活跃段平均速度 | > 0.02m/s |
| stuck_frames_ratio | 卡住帧比例 | < 20% |

**验收标准**：

所有实验报告必须同时输出安全指标和运动真实性指标。

---

## 四、P1：AMCL 与 A* 可信度提升

### 4.1 重构 AMCL 置信度计算

**问题**：当前置信度 0.491，但定位误差 0.000m，说明 `pos_var` 型置信度不能稳定代表真实定位质量。

**建议方案**：

| 方案 | 内容 | 风险 |
|------|------|------|
| A | 使用 N_eff / N 作为置信度组成项 | 容易受 z_rand 影响 |
| B | 使用粒子加权方差 + scan likelihood 双指标 | 更符合定位质量 |
| C | 输出 confidence 与 loc_error 两条线，不再强行等价 | 最稳妥 |

**推荐实现**：

```text
confidence = 0.5 * normalized_scan_score
           + 0.3 * normalized_neff
           + 0.2 * normalized_particle_compactness
```

同时保留 `loc_error` 作为仿真专用真值指标，论文中明确区分“估计置信度”和“真值误差”。

**验收标准**：

- 置信度与定位误差趋势大体一致；
- 置信度不再被用于直接降速；
- 低置信度只触发重定位/恢复，不直接让机器人停住。

### 4.2 重新审视 KLD-Sampling

**问题**：粒子数 500 全 active，KLD 自适应没有真正发挥作用。

**优化方向**：

- 检查 bin 划分是否过细或过粗；
- 检查 z_rand 是否让权重过度均匀；
- 添加粒子数随时间变化曲线；
- 比较 fixed-500、KLD、KLD+权重锐化三组。

**验收标准**：

| 指标 | 要求 |
|------|------|
| 粒子数曲线 | 能随场景不确定性变化 |
| 定位误差 | 不劣于 fixed-500 |
| AMCL 耗时 | 均值 < 30ms |
| kidnapped recovery | 能恢复，不飞出地图 |

### 4.3 A* 窄通道鲁棒性测试

**目标**：确认 `INSCRIBED_RADIUS=0.0` 不是只对当前地图有效。

**测试场景**：

- 1.2m 门道；
- 1.0m 门道；
- 0.8m 门道；
- 动态障碍横穿门口；
- 目标点落在障碍附近。

**验收标准**：

- 合理门宽下 A* 不被膨胀区硬阻断；
- 过窄门道能明确失败，不产生穿墙路径；
- A* 平均耗时仍 < 100ms。

---

## 五、P2：C++ 化工程路线

### 5.1 明确源码分层

建议统一为：

| 目录 | 定位 |
|------|------|
| `src/` | ROS2 工作区权威源码 |
| `cpp_src/` | C++ 迁移参考区，逐步迁入 `src/` |
| 根目录 Python | 实验/论文/分析脚本 |
| `backup_*` | 历史备份，不参与构建 |

### 5.2 优先迁移顺序

| 顺序 | 模块 | 理由 |
|------|------|------|
| 1 | `puppy_nav_core` | 算法核心已是 C++，最适合作为主线 |
| 2 | `puppypi_adapter` | 接口清楚，适合做 C++ ROS2 包模板 |
| 3 | `puppy_core` | mission/safety/mode 是运行时核心 |
| 4 | `puppy_gait` | 已有 C++ gait_controller，可继续补齐行为节点 |
| 5 | bringup launch | launch 可继续 Python，但默认节点切到 C++ |

### 5.3 第一阶段交付：C++ adapter 包

**目标**：把 `cpp_src/puppypi_adapter` 晋升为 `src/puppypi_adapter_cpp`，不破坏现有 Python 包。

**任务**：

- 拷贝 C++ 源码到 `src/puppypi_adapter_cpp`；
- 修正 `CMakeLists.txt` include 路径；
- 保持可执行名：`motion_adapter`、`mode_adapter`、`status_adapter`；
- 新增 `adapter_bringup_cpp.launch.py`；
- 在 README 标注 C++ 版本启动方式。

**验收标准**：

```bash
cd "$PUPPY_WS"
colcon build --symlink-install --packages-select puppypi_adapter_cpp
ros2 launch puppypi_adapter_cpp adapter_bringup_cpp.launch.py use_sim:=true
```

能够构建并启动，不影响原 `puppypi_adapter` Python 包。

### 5.4 第二阶段交付：C++ core 包

**目标**：将 `mission_manager`、`safety_manager`、`goal_dispatcher` 迁移为 C++，形成完整控制面。

**验收标准**：

- topic/service/action 名称与 Python 版保持兼容；
- 原 Python 回归测试等价迁移为 C++ 或 launch smoke test；
- `full_system_cpp.launch.py` 默认走 C++ core + C++ adapter。

---

## 六、P3：工程化与复现优化

### 6.1 版本管理与清理

当前目录不是 git 仓库，且缓存/日志/截图/编译产物混在源码中。建议：

- 初始化 git；
- 使用 `.gitignore` 排除产物；
- 清理根目录截图、日志、`.obj/.exe`、`__pycache__`；
- 把长期实验结果统一放入 `experiment_results/`；
- 把论文图统一放入 `theory_figures/` 或 `docs/figures/`。

### 6.2 构建脚本统一

建议保留三类脚本：

| 脚本 | 用途 |
|------|------|
| `scripts/sync_to_wsl.sh` | Windows 源码同步到 WSL 工作区 |
| `scripts/build_ros2.sh` | colcon build |
| `scripts/run_cpp_stability.ps1` | Windows C++ 仿真压力测试 |

其余 `_fix_*`、`_check_*`、`sim_run*.log` 类临时脚本和日志建议归档。

### 6.3 启动逻辑去固定延迟

当前 launch 中存在 `TimerAction(15s/18s/20s/40s)`。建议：

- Nav2 lifecycle active 后再启动上层节点；
- `/tf`、`/scan`、`/map` 可用后再启动 RViz/mission；
- Coppelia bridge 发布 `/clock` 后再启动 use_sim_time 节点。

验收标准：不同机器上启动成功率更稳定，不靠手动重启。

---

## 七、P4：论文与实验增强

### 7.1 12 小时压力测试

根据 `交接5.md` 建议，运行：

```powershell
.\sim_test.exe 1296000 180000
```

但建议先完成多 seed 1 小时测试，再跑 12 小时。12 小时测试至少输出：

- 碰撞数；
- 穿墙数；
- A* 成功率；
- AMCL 平均耗时；
- A* 平均耗时；
- 运动距离；
- 房间覆盖；
- stuck ratio；
- 内存/进程存活状态。

### 7.2 消融实验补齐

建议保留以下对比：

| 实验 | 对比项 |
|------|--------|
| AMCL | fixed particles vs KLD vs KLD+confidence |
| Costmap | dynamic lethal vs dynamic soft-cost |
| A* | old inflation vs fixed inflation |
| Safety | no startup guard vs startup guard |
| Simulation | single seed vs multi seed |

### 7.3 论文叙事建议

本阶段论文主线建议聚焦为：

> 面向动态室内环境的四足机器人安全自主导航：基于软代价动态障碍、鲁棒 A* 膨胀建模与 AMCL 不确定性诊断的 C++ 仿真验证。

这样能把 v3.2.2 的真实成果串起来，而不是堆算法。

---

## 八、推荐执行顺序

### 第 1 天：修复与验证

- 修复行人反弹 vx/vy bug；
- 增加 seed 参数；
- 跑 3 个 seed 的 30 分钟冒烟测试；
- 确认不会回到“车不动假稳定”。

### 第 2 天：多 seed 稳定性

- 跑 10 个 seed，每个 1 小时；
- 生成稳定性汇总表；
- 若有碰撞，定位到具体 frame/房间/行人。

### 第 3 天：AMCL 指标重构

- 输出 N_eff、scan score、particle variance；
- 新 confidence 不参与降速；
- 对比旧 confidence 与 loc_error。

### 第 4-5 天：C++ 包晋升

- 晋升 `puppypi_adapter_cpp` 到 `src/`；
- 补 C++ launch；
- 保持 Python 包可回退；
- 写构建说明。

### 第 6-7 天：论文证据整理

- 整理表格和图；
- 更新实验结果；
- 写限制与失败案例；
- 准备 12 小时压力测试。

---

## 九、最终验收标准

| 类别 | 验收项 | 标准 |
|------|--------|------|
| 安全 | 碰撞 | 多 seed 1 小时全部 0 |
| 安全 | 穿墙 | 全部 0 |
| 规划 | A* 成功率 | 均值 >= 90%，最低 >= 85% |
| 定位 | AMCL 误差 | 均值 < 0.5m |
| 性能 | A* 耗时 | 均值 < 100ms |
| 性能 | AMCL 耗时 | 均值 < 30ms |
| 真实性 | 运动距离 | 1 小时 >= 10m |
| 真实性 | 房间覆盖 | >= 3 个房间 |
| 工程 | C++ adapter | 可独立 colcon build |
| 工程 | 文档 | README + 交接 + 实验表一致 |

---

## 十、结论

v3.2.2 已经解决了最关键的问题：不是靠不运动换 0 碰撞，而是在真实运动下达到 0 碰撞和 A* 93%。下一阶段不要急着继续堆新算法，重点应放在：

1. 修掉已知仿真 bug；
2. 用多 seed 和长时测试证明稳定性；
3. 把 C++ 实现晋升为主运行路径；
4. 把实验指标变成可复现证据；
5. 让论文主线从“功能很多”收敛为“安全、稳定、可验证的 C++ 自主导航系统”。
