# 机器狗自主导航仿真系统 — 工程交接文档

**日期**: 2026-08-01
**版本**: v3.2.18i
**状态**: 10种子1小时测试全部零碰撞，核心指标全部达标

---

## 1. 项目概述

基于 CoppeliaSim + 自研 C++ 仿真器的四足机器狗自主导航系统，实现建图、定位、路径规划、动态避障的完整闭环。

- **机器人**: Puppy 四足机器狗，半径 0.35m
- **场景**: 10m × 8m 室内家居（客厅/卧室/书房/厨房/卫生间/储物间/餐厅），含5个动态行人
- **定位**: AMCL 自适应蒙特卡洛定位（KLD-Sampling + 聚类估计 + IMU融合）
- **规划**: A* 全局规划 + 路径跟随 + RVO动态避障
- **仿真**: 30 FPS，1小时等效 = 108000帧

---

## 2. 工程结构

```
e:\puppyfangzhen\
├── cpp_src/                    # ★ C++ 核心代码（当前主开发）
│   ├── puppy_nav_core/         #   导航核心库
│   │   ├── include/puppy_nav_core/
│   │   │   ├── amcl.h          #     AMCL 定位（聚类估计+IMU融合+MHT+ICP）
│   │   │   ├── astar_planner.h #     A* 路径规划
│   │   │   ├── costmap.h       #     分层代价地图
│   │   │   └── occupancy_grid.h#     占据栅格地图
│   │   └── src/                #   实现文件
│   ├── sim/                    #   C++ 仿真器（★ 主要验证平台）
│   │   ├── simulation.h        #     仿真核心（场景/行人/flee避障/碰撞诊断）
│   │   ├── nav_bridge.h        #     导航栈桥接（AMCL+A*+路径跟随）
│   │   ├── main.cpp            #     仿真入口
│   │   ├── cbf_safety.h        #     CBF 安全过滤器
│   │   ├── rvo_safety.h        #     RVO 互惠速度障碍避障
│   │   ├── path_planner.h      #     路径跟随器
│   │   ├── build.bat           #     编译脚本
│   │   └── batch_test.ps1      #     10种子批量验证脚本
│   ├── puppy_core/             #   应用层（ROS2节点）
│   ├── puppy_gait/             #   步态控制
│   ├── puppy_interfaces/       #   ROS2 消息/服务定义
│   ├── puppypi_adapter/        #   硬件适配层
│   └── puppypi_mock/           #   仿真接口
├── src/                        # ROS2 Python 包（旧版，部分保留）
│   ├── puppy_bringup/          #   系统启动
│   ├── puppy_core/             #   核心逻辑
│   ├── puppy_nav/              #   导航
│   ├── puppy_slam/             #   SLAM
│   └── puppy_description/      #   URDF 模型
├── nav_core/                   # Python 导航核心（旧版）
├── config/                     # YAML 配置文件
├── maps/                       # 地图文件
└── docs/                       # 文档
```

---

## 3. 技术架构

### 四层架构

```
应用层 (puppy_core)        — 任务调度/模式管理/安全监控
    ↓
导航能力层 (nav_core)      — AMCL定位/A*规划/避障/探索
    ↓
平台适配层 (puppypi_adapter)— 运动控制/传感器数据/电池状态
    ↓
硬件/SDK层                  — PuppyPi SDK / CoppeliaSim
```

### 仿真数据流

```
LiDAR扫描(72射线) → OccupancyGrid → Costmap(分层) → A*(全局规划)
                                                          ↓
真值位姿 → AMCL粒子滤波 → 聚类估计 → IMU融合 → 决策位姿 → 路径跟随 → RVO避障 → flee → 物理移动
                    ↑                                    ↓
              碰撞检测 ← 真值位姿 ← ← ← ← ← ← ← ← ← ← ←
```

---

## 4. 核心算法

### 4.1 AMCL 定位（KLD-Sampling + 聚类估计 + IMU融合）

**文件**: [amcl.cpp](file:///e:/puppyfangzhen/cpp_src/puppy_nav_core/src/amcl.cpp)

- **KLD-Sampling**: Wilson-Hilferty 近似自适应粒子数（收敛时50-66，分散时500）
- **聚类估计** (`get_cluster_estimate()`): 贪心距离聚类(0.5m阈值)→取最大簇加权均值，解决粒子云分裂时均值落墙内问题
- **限幅平滑**: MAX_STEP=0.3m/帧，防止簇切换时估计位姿跳变
- **簇中心边界验证**: 仅选择地图边界内(`is_in_bounds`)的粒子簇
- **IMU+LiDAR互补滤波**: IMU高频(30Hz)预测+AMCL低频全局校正，alpha=0.3+0.5*conf动态调整，MAX_CORRECT=0.3m限幅
- **scan_score归一化**: 跳过max_range光束，归一化到[0,1]，置信度从0.602提升到0.683+

### 4.2 A* 路径规划

**文件**: [astar_planner.cpp](file:///e:/puppyfangzhen/cpp_src/puppy_nav_core/src/astar_planner.cpp)

- 8连通 + octile启发式
- UNKNOWN cell 软惩罚(5.0)而非硬阻塞
- 路径平滑 + 不可达目标过滤
- 成功率: 97.2-100%

### 4.3 动态避障（flee + RVO + CBF）

**文件**: [simulation.h](file:///e:/puppyfangzhen/cpp_src/sim/simulation.h) (flee逻辑), [rvo_safety.h](file:///e:/puppyfangzhen/cpp_src/sim/rvo_safety.h)

- **预测式行人避让**: 0.3秒后预测位置计算flee方向，平滑线性插值降速
- **flee速度策略**: 多行人0.5m/s，单行人近距离1.2m/s，中距离0.8m/s
- **追逐检测** (v3.2.18i): `dot = vel·(robot-ped)/(|vel|·|robot-ped|)`
  - 弱追逐(dot>0.3): flee_speed=1.2
  - 强追逐(dot>0.8)且单行人(near_count<2): 混合预测(0.5*0.3s + 0.5*1.0s)偏好垂直逃离
- **RVO协同**: flee时设置临时目标引导RVO，flee_speed覆盖RVO max_speed
- **碰撞诊断**: 环形缓冲区记录最近30帧，碰撞时打印回溯

### 4.4 代价地图

**文件**: [costmap.h](file:///e:/puppyfangzhen/cpp_src/puppy_nav_core/include/puppy_nav_core/costmap.h)

- 静态层 + 障碍层 + 膨胀层
- ROBOT_RADIUS=0.35m, INFLATION_RADIUS=0.15m（通过1m门道的关键）
- GRID_W=100, GRID_H=80, GRID_RESOLUTION=0.1m

---

## 5. 构建与运行

### 5.1 C++ 仿真器编译（主要验证平台）

```powershell
cd e:\puppyfangzhen\cpp_src\sim
.\build.bat
```

编译依赖：Visual Studio 2022 (MSVC 14.44)，`build.bat` 自动调用 `vcvars64.bat`。

### 5.2 单种子测试

```powershell
$env:USE_AMCL="1"
$env:USE_IMU_FUSION="1"
$env:USE_ICP="0"
.\sim_test.exe 108000 54000 8    # 1小时测试, seed=8
```

参数: `[帧数] [报告间隔] [seed]`

### 5.3 10种子批量验证

```powershell
.\batch_test.ps1
```

或直接命令行循环（更可靠）：

```powershell
$env:USE_AMCL="1"; $env:USE_IMU_FUSION="1"; $env:USE_ICP="0"
for ($seed=1; $seed -le 10; $seed++) {
    $out = & .\sim_test.exe 108000 54000 $seed 2>&1
    # ... 提取指标
}
```

### 5.4 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `USE_AMCL` | 1 | 1=AMCL定位, 0=真值位姿(消融) |
| `USE_IMU_FUSION` | 1 | 1=IMU融合, 0=纯AMCL |
| `USE_ICP` | 0 | 1=ICP扫描匹配(实验性,已禁用) |
| `USE_RVO` | 1 | 1=RVO避障, 0=CBF |
| `SEED` | 0 | 0=随机, >0=固定种子 |

### 5.5 ROS2 系统启动（WSL2）

```bash
# WSL2 Ubuntu-22.04 + ROS2 Humble
source /opt/ros/humble/setup.bash
cd ~/puppy_ws && colcon build --symlink-install
source install/setup.bash

# 建图
ros2 launch puppy_bringup mapping.launch.py world:=small_room

# 导航
ros2 launch puppy_bringup navigation.launch.py
```

---

## 6. 最新验证结果（v3.2.18i）

### 10种子1小时测试 — 全部零碰撞

| Seed | 碰撞 | A*成功率 | 平均误差 | 置信度 | 房间 | 轮次 |
|------|------|---------|---------|--------|------|------|
| 1    | **0** | 99.9% | 0.146m | 0.976 | 8 | 10 |
| 2    | **0** | 100.0% | 0.064m | 0.802 | 8 | 4 |
| 3    | **0** | 99.8% | 0.146m | 0.863 | 9 | 22 |
| 4    | **0** | 99.5% | 0.080m | 0.817 | 8 | 5 |
| 5    | **0** | 99.5% | 0.080m | 0.867 | 8 | 13 |
| 6    | **0** | 97.2% | 0.089m | 0.975 | 8 | 10 |
| 7    | **0** | 99.9% | 0.092m | 0.783 | 8 | 5 |
| 8    | **0** | 99.4% | 0.113m | 0.815 | 8 | 9 |
| 9    | **0** | 99.9% | 0.071m | 0.979 | 8 | 13 |
| 10   | **0** | 100.0% | 0.161m | 0.825 | 8 | 5 |

### 约束达标情况

| 指标 | 约束 | 实测 | 状态 |
|------|------|------|------|
| 碰撞次数 | 0 | 0/10种子 | ✅ |
| A*成功率 | ≥90% | 97.2-100% | ✅ |
| 平均定位误差 | <0.5m | 0.064-0.161m | ✅ |
| 置信度 | >0.1 | 0.783-0.979 | ✅ |
| 房间覆盖 | ≥3 | 8-9 | ✅ |
| 卡住帧比例 | <20% | 0% | ✅ |
| AMCL耗时 | <30ms | ~3ms/帧 | ✅ |
| 穿墙 | 0 | 0 | ✅ |

---

## 7. 优化历程与关键教训

### v3.2.7 → v3.2.18i 核心突破路径

| 版本 | 核心改进 | 结果 |
|------|---------|------|
| v3.2.7 | 预测式行人避让+平滑降速 | 10种子10分钟0碰撞 |
| v3.2.8 | AMCL置信度优化(跳过max_range+归一化) | 置信度0.602→0.683 |
| v3.2.11 | 聚类AMCL估计+限幅平滑 | avg_err 0.391→0.087m, A* 90→97% |
| v3.2.14a | IMU+LiDAR互补滤波(限幅0.3m) | max_err 2.7→2.4m, 0碰撞 |
| v3.2.18b | 簇中心边界验证(is_in_bounds) | seed6 A* 81→97%, 9/10零碰撞 |
| **v3.2.18i** | **is_chasing修复+强追逐混合预测** | **10/10零碰撞** |

### 关要教训

1. **AMCL定位精度与规划稳定性必须平衡**: 减小sigma_obs(0.45→0.30)导致A*从89%暴跌到41%，粒子过度收敛→估计漂移→规划失败
2. **聚类估计优于MHT**: v3.2.13 MHT多假设跟踪在门道对称区域累积似然不可靠，简单"取最大簇+限幅平滑"更稳定
3. **ICP需反馈粒子云**: ICP直接修改est_x/y但不反馈到AMCL粒子云导致状态不一致，max_err恶化
4. **flee速度不能盲目提升**: flee_speed=1.5帮助seed8逃脱但导致seed1/5碰撞，速度过快反应时间不足
5. **混合预测需条件触发**: 1.0s长期预测在行人弹墙后不可靠，仅强追逐(dot>0.8)单行人场景使用
6. **is_chasing检测bug**: 用`d < true_min_dist`重新查找行人永远为false（true_min_dist已是最小值），导致追逐检测失效

---

## 8. 已知限制

1. **max_err尖峰**: 门道穿越时粒子云短暂分裂，max_err可达2-8m，但50帧内自动恢复
2. **seed 6 A* 97.2%**: 仍低于99%，剩余失败全为"真无路径"（AMCL估计漂移导致起点落墙内）
3. **IMU速度来源**: 仿真中从真值位姿计算，实际系统需用里程计
4. **ICP已禁用**: 需将校正量反馈到粒子云才能工作，保留代码供未来改进
5. **MHT已禁用**: 保留代码但不调用，简单聚类估计更稳定

---

## 9. 待办事项与未来方向

### 短期
- [ ] 固化Visual Studio编译环境（设置系统级INCLUDE/LIB路径）
- [ ] 24小时连续运行压力测试
- [ ] 内存泄漏检测（长时间运行）

### 中期
- [ ] ICP扫描匹配改进（校正量反馈到粒子云）
- [ ] seed 6 A*成功率提升（分析剩余3%失败根因）
- [ ] TEB Local Planner集成（替代路径跟随器）
- [ ] ROS2 Lifecycle完整实现（on_configure/on_activate/on_deactivate/on_cleanup）

### 长期
- [ ] 真机部署与100+小时故障-free验证
- [ ] 多机通信（DDS分离导航算法与运动控制）
- [ ] OTA固件/算法包更新
- [ ] ISO 13482 / IEC 61508认证

---

## 10. 关键文件索引

### C++ 核心代码
| 文件 | 功能 |
|------|------|
| [simulation.h](file:///e:/puppyfangzhen/cpp_src/sim/simulation.h) | 仿真核心（场景/行人/flee/碰撞诊断） |
| [amcl.cpp](file:///e:/puppyfangzhen/cpp_src/puppy_nav_core/src/amcl.cpp) | AMCL定位（聚类+IMU融合+MHT+ICP） |
| [astar_planner.cpp](file:///e:/puppyfangzhen/cpp_src/puppy_nav_core/src/astar_planner.cpp) | A*路径规划 |
| [nav_bridge.h](file:///e:/puppyfangzhen/cpp_src/sim/nav_bridge.h) | 导航栈桥接 |
| [main.cpp](file:///e:/puppyfangzhen/cpp_src/sim/main.cpp) | 仿真入口 |
| [rvo_safety.h](file:///e:/puppyfangzhen/cpp_src/sim/rvo_safety.h) | RVO避障 |
| [cbf_safety.h](file:///e:/puppyfangzhen/cpp_src/sim/cbf_safety.h) | CBF安全过滤 |

### 配置文件
| 文件 | 内容 |
|------|------|
| [config/runtime.yaml](file:///e:/puppyfangzhen/config/runtime.yaml) | 运行时参数 |
| [config/sim.yaml](file:///e:/puppyfangzhen/config/sim.yaml) | 仿真环境配置 |
| [config/planner_global.yaml](file:///e:/puppyfangzhen/config/planner_global.yaml) | 全局规划参数 |
| [config/planner_local.yaml](file:///e:/puppyfangzhen/config/planner_local.yaml) | 局部规划参数 |

### 测试结果
| 文件 | 内容 |
|------|------|
| [v3218i_10seed.txt](file:///e:/puppyfangzhen/cpp_src/sim/v3218i_10seed.txt) | 最新10种子验证结果 |
| [batch_test.ps1](file:///e:/puppyfangzhen/cpp_src/sim/batch_test.ps1) | 批量验证脚本 |

### 文档
| 文件 | 内容 |
|------|------|
| [README.md](file:///e:/puppyfangzhen/README.md) | 环境配置与快速开始 |
| [HANDOFF.md](file:///e:/puppyfangzhen/HANDOFF.md) | 早期交接文档 |
| [HANDOFF_CXX_NAV_CORE.md](file:///e:/puppyfangzhen/HANDOFF_CXX_NAV_CORE.md) | C++导航核心交接 |
| [docs/cpp_migration_plan.md](file:///e:/puppyfangzhen/docs/cpp_migration_plan.md) | C++迁移计划 |

---

## 11. 环境与依赖

### 开发环境
- **OS**: Windows 11
- **编译器**: Visual Studio 2022 (MSVC 14.44.35207)
- **C++标准**: C++17
- **编译选项**: `/O2 /std:c++17 /EHsc /utf-8 /MT /D_CRT_SECURE_NO_WARNINGS`

### 运行环境（ROS2）
- **OS**: WSL2 Ubuntu-22.04
- **ROS2**: Humble Hawksbill
- **仿真**: CoppeliaSim / Gazebo Classic 11
- **Python**: 3.10 (诊断/分析工具)

### 核心依赖
- 无第三方C++库（纯标准库实现）
- Python: numpy（不使用scipy，兼容性约束）

---

*本交接文档基于 v3.2.18i 版本，10种子1小时测试全部零碰撞的稳定状态编写。*
