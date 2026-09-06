# Puppy SLAM 仿真 - 使用说明

## 环境信息

- **操作系统**: Windows 11 + WSL2 Ubuntu 22.04
- **ROS2**: Humble Hawksbill
- **仿真**: Gazebo Classic 11
- **工作空间**: 默认 `~/puppy_ws` (WSL内) / `/mnt/e/puppyfangzhen` (Windows E盘挂载路径)
- **提示**: `e:\puppyfangzhen` 不是 git 仓库，交接时建议额外保留压缩包或版本快照
- **工程方向**: 运行时 ROS2 节点逐步收敛到 C++，Python 主要保留为实验、分析、地图生成和诊断工具。

建议在 WSL 中先设置两个路径变量，后续命令都基于这两个变量：

```bash
export PUPPY_SRC=/mnt/e/puppyfangzhen
export PUPPY_WS=~/puppy_ws
```

## 先说明白一件事

- Windows 侧源码目录挂载到 WSL 后通常是 `$PUPPY_SRC`，方便你在当前系统里看和改文件。
- 真正运行 ROS2、Gazebo、RViz 的位置是 **WSL Ubuntu** 里的 `$PUPPY_WS`。
- 所以“Windows 上看不到仿真窗口”通常不是因为项目没启动，而是因为：
  1. 还没有进入 WSL 运行；
  2. 代码还没有同步到 `$PUPPY_WS/src/` 并编译；
  3. WSLg 或 VcXsrv 没把 Linux GUI 转发到 Windows。
- Gazebo/RViz 打开后会作为 **Windows 窗口显示出来**，但它们本质上是 WSL 里的 Linux GUI 程序，不会显示在当前 PowerShell 终端里。
- 在当前机器上，WSLg 下的 Gazebo 窗口可能显示为 **`Gazebo (Ubuntu-22.04)`**，宿主进程是 `msrdc.exe`，不一定能在 Windows 任务管理器里直接看到 `gzclient.exe` 这样的标题。

## 快速开始

### 1. 在 Windows 中打开 WSL

```powershell
wsl -d Ubuntu-22.04
```

看到 Ubuntu 提示符后，再继续下面步骤。

### 2. 准备 WSL 工作空间并同步代码

```bash
export PUPPY_SRC=/mnt/e/puppyfangzhen
export PUPPY_WS=~/puppy_ws
mkdir -p "$PUPPY_WS/src"
cp -r "$PUPPY_SRC/src/"* "$PUPPY_WS/src/"
```

如果你之前已经同步过代码，这一步可以跳过。

### 3. 设置环境并编译

```bash
source /opt/ros/humble/setup.bash
export PATH=$HOME/.local/bin:$PATH
cd "$PUPPY_WS"
colcon build --symlink-install
source "$PUPPY_WS/install/setup.bash"
```

### 4. 验证 URDF 模型（在 RViz 中查看）

```bash
ros2 launch puppy_description display.launch.py
```
这会打开 RViz2，你可以看到机器人模型并用 GUI 滑块控制关节。

### 5. 启动 Gazebo 仿真 + 建图

```bash
# 使用小房间场景建图
ros2 launch puppy_bringup mapping.launch.py world:=small_room

# 使用办公室场景建图
ros2 launch puppy_bringup mapping.launch.py world:=office
```

启动后会打开：
- Gazebo 仿真环境（含机器人）
- RViz2（显示激光、地图，默认开启）

默认参数为 `gui:=false teleop:=false`，更适合首次在新机器上验证。
如果你需要 Gazebo GUI 或键盘控制，请显式开启：

```bash
ros2 launch puppy_bringup mapping.launch.py world:=small_room gui:=true teleop:=true
```

**键盘控制机器人移动：**
- `i` - 前进
- `,` - 后退
- `j` - 左转
- `l` - 右转
- `k` - 停止

如果要用键盘控制，请确保系统里有 `xterm`：

```bash
sudo apt install xterm
```

### 6. 保存地图

建图完成后，在另一个 WSL 终端执行：

```bash
source /opt/ros/humble/setup.bash
source "$PUPPY_WS/install/setup.bash"
ros2 run nav2_map_server map_saver_cli -f "$PUPPY_WS/src/puppy_nav/maps/small_room"
```

### 7. 启动自主导航

```bash
# 先把地图复制到 puppy_nav/maps 目录
ros2 launch puppy_bringup navigation.launch.py world:=small_room map:="$PUPPY_WS/src/puppy_nav/maps/small_room.yaml"
```

在 RViz2 中：
1. 点击 "2D Pose Estimate" 设置初始位置
2. 点击 "2D Goal Pose" 设置目标位置
3. 机器人会自动规划路径并导航

## 已安装的 ROS2 包

| 包名 | 说明 |
|------|------|
| ros-humble-desktop | ROS2 桌面完整版 |
| ros-humble-gazebo-ros-pkgs | Gazebo 仿真集成 |
| ros-humble-ros2-control | 关节控制框架 |
| ros-humble-gazebo-ros2-control | Gazebo ros2_control 插件 |
| ros-humble-cartographer | Google Cartographer SLAM |
| ros-humble-slam-toolbox | SLAM Toolbox |
| ros-humble-navigation2 | Nav2 导航栈 |
| ros-humble-nav2-bringup | Nav2 启动文件 |
| ros-humble-robot-localization | EKF 定位融合 |

## 工作空间包结构

```
$PUPPY_WS/src/
├── puppy_description/     # URDF 机器人模型
├── puppy_hardware/        # ros2_control 配置
├── puppy_gait/            # Trot 步态控制器 (C++)
├── puppy_localization/    # 足式里程计 + EKF
├── puppy_slam/            # SLAM 配置 (slam_toolbox)
├── puppy_nav/             # Nav2 导航配置
├── puppy_worlds/          # Gazebo 仿真世界
└── puppy_bringup/         # 一键启动文件
```

## 编译工作空间

如果修改了代码，需要重新编译：

```bash
# 在 WSL 中
source /opt/ros/humble/setup.bash
export PATH=$HOME/.local/bin:$PATH
cd "$PUPPY_WS"
colcon build --symlink-install
```

## 从 E 盘同步代码到 WSL

如果你在 Windows 上修改了 E 盘的源码：

```bash
export PUPPY_SRC=/mnt/e/puppyfangzhen
export PUPPY_WS=~/puppy_ws
cp -r "$PUPPY_SRC/src/"* "$PUPPY_WS/src/"
cd "$PUPPY_WS"
colcon build --symlink-install
```

建议先清理旧目录里同名包，再同步，避免保留陈旧文件。

## 项目清理

仓库根目录下保留了不少仿真截图、日志、编译产物和缓存。新加的清理脚本默认只预览，不会删除文件：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\clean_artifacts.ps1
```

确认列表无误后再执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\clean_artifacts.ps1 -Apply
```

在 WSL/Linux 中可以使用：

```bash
bash scripts/clean_artifacts.sh
bash scripts/clean_artifacts.sh --apply
```

默认不会清理 `experiment_results/`、`ablation_results*/`、`sensitivity_results/`、`eval_*` 和 `screenshots/` 等结果目录。如需一起清理，PowerShell 加 `-IncludeResults`，bash 加 `--include-results`。

## C++ 化方向

项目目标是把运行时 ROS2 节点逐步收敛到 C++：

- `src/` 作为当前 ROS2 工作空间的权威源码目录；
- `cpp_src/` 作为 C++ 迁移与参考实现目录；
- Python 保留给实验算法、数据分析、地图生成、快速诊断；
- launch/config 包可以继续使用 ROS2 常见的 Python launch 文件；
- 每迁移一个节点，都要保持 topic/service/action 接口兼容，并补一条 smoke test。

更详细的迁移顺序见 `docs/cpp_migration_plan.md`。

## 运行 Profile 与验收门禁 (jihua20260905.md §11 / §12)

`config/unified_params.yaml` 顶部 `profile:` 选择激活档, `profiles:` 定义四套语义
(单一真源: 阈值一律来自 `simulation_contract.yaml` / `motion_capability.yaml`, 不在此重复写死):

| profile | 用途 | 验收级别 | 默认帧数 | 后端 | 强制单一真源 |
|---|---|---|---|---|---|
| `dev` | 本地开发冒烟 | SMOKE | 300 | cpp_nav_core | 否 |
| `research` | 消融/论文多种子 | REGRESSION | 36000 | cpp_nav_core | 否 |
| `release` | 发布验收(严格, 不可静默回退) | RELEASE | 108000 | cpp_nav_core | **是** |
| `demo` | 答辩/视频(稳定优先) | INTEGRATION | 9000 | ue_bridge | 否 |

- 切换档: 改 `config/unified_params.yaml` 顶部 `profile:` 字段。
- 校验激活档合法: `python scripts/check_profile.py` (非法退出非零)。
- 发布门禁: `python scripts/check_profile.py --assert-release` 仅 `release` 且
  `enforce_single_source=true` 时通过。
- 文档一致性(§13 风险#8): `python scripts/check_docs_consistency.py` 扫描旧房间数/旧半径/Jazzy 混写。
- 一键发布报告(§9.3): `python scripts/gen_release_report.py --run-id <id>` 聚合
  `artifacts/cpp_*_seedN_planB.json` → `artifacts/<id>/` (summary.md + plots/)。
- 全量回归: `bash scripts/run_regression.sh` (已接入上述门禁, 15/15 ALL GREEN)。

## 常见问题

### Q: Gazebo 打不开 / GUI 不显示？
A: 先确认你是在 **WSL Ubuntu** 里启动，而不是直接在 Windows PowerShell 里启动。

1. 在 Windows PowerShell 中运行：
```powershell
wsl --update
```
2. 进入 WSL 后再运行 ROS2 启动命令。
3. 如果用的是 VcXsrv，先双击 `e:\puppyfangzhen\start_vcxsrv.bat`，再在 WSL 中执行：
```bash
source "$PUPPY_SRC/_use_vcxsrv.sh"
```
4. 如果窗口还是没出来，检查 Windows 任务栏右下角是否有 VcXsrv 的 X 图标，或者确认 WSLg 是否正常。
5. 如果 WSLg 已经正常，但你以为“没有 Gazebo”，请在任务栏或 Alt+Tab 里找 **`Gazebo (Ubuntu-22.04)`** 这个窗口。

### Q: `mapping.launch.py` 为什么没弹出键盘控制窗口？
A: 现在默认 `teleop:=false`，是为了降低首次启动失败率。需要时手动开：

```bash
ros2 launch puppy_bringup mapping.launch.py world:=small_room teleop:=true
```

### Q: CoppeliaSim 路线的 `.ttt` 场景文件在哪？
A: 仓库默认不提交 `.ttt` 二进制场景。你可以：

1. 保留已有的 `home_coppelia.ttt`
2. 或者在打开 CoppeliaSim 后运行：
```bash
python3 "$PUPPY_SRC/build_coppelia_scene.py"
```

### Q: 我在 `e:\puppyfangzhen` 里已经看到代码了，为什么还是看不到仿真？
A: 因为这里是 Windows 侧源码目录，不是 ROS2 的实际运行环境。你需要：

1. `wsl -d Ubuntu-22.04`
2. 把 `$PUPPY_SRC/src/*` 同步到 `$PUPPY_WS/src/`
3. 在 `$PUPPY_WS` 里 `colcon build`
4. 再从 WSL 启动 `ros2 launch ...`

### Q: 内存不足（5.8GB）？
A: Gazebo + RViz 比较吃内存。可以：
1. 关闭 RViz 中的不必要的可视化
2. 降低 Gazebo 物理更新频率
3. 增加虚拟内存（页面文件）

### Q: 步态控制器让机器人摔倒？
A: 调整 `puppy_gait/config/gait_params.yaml`：
- 降低 `step_length`（步长）
- 降低 `gait_period`（步频加快）
- 增加 `body_height`（降低重心）

### Q: SLAM 建图效果差？
A: 
1. 确保机器人走得慢一点（`cmd_vel` 的 vx 设小）
2. 检查激光数据：`ros2 topic echo /scan`
3. 调整 `mapper_params_online_async.yaml` 中的 `resolution`

## MiniCPM-RobotTrack 视觉跟踪与导航系统

根据 `20260828.md` 实施规划，系统新增 `puppy_minicpm_robot` 模块，将 OpenBMB MiniCPM-RobotTrack 视觉语言大模型与 Puppy 底盘及 Nav2 导航链路深度集成：

### 1. 核心节点与职责
- `vision_bridge_node`：统一仿真、USB/RealSense 相机与 Unitree Go2 VideoClient 的图像流输入，提供标准 `/camera/color/image_raw`（默认 384x384 裁剪与缩放）。
- `minicpm_track_node`：负责模型推理客户端/引擎管理，接收自然语言任务指令，输出航向建议、3D Waypoint 及意图置信度。
- `track_cmd_adapter_node`：速度映射、死区抑制、EMA滤波、失联 350ms 超时停车以及 `/scan` 激光雷达防撞安全覆盖（严格限制 `vx<=0.15m/s, wz<=0.30rad/s`）。
- `mission_grounder_node`：将自然语言任务（如“巡视后院并检查是否有可疑物品”）解耦为结构化任务阶段（导航至区域 ➔ 视觉跟踪 ➔ 驻留检查 ➔ 自动返航）。

### 2. 离线推断与阶段 A 验证
```bash
python src/puppy_minicpm_robot/scripts/offline_inference_demo.py --instruction "Follow the person ahead" --output logs/offline_inference_result.json
```

### 3. Go2 实机与硬件预检诊断
```bash
python src/puppy_minicpm_robot/scripts/preflight_check.py
```

### 4. 仿真与实机启动
```bash
# 仿真环境（支持 dry-run / sim 模式）
ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py mode:=dry-run

# Go2 实机部署（默认 dry-run 安全模式）
ros2 launch puppy_minicpm_robot minicpm_robot_go2.launch.py mode:=dry-run
```

