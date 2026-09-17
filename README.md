# Puppy Fang 仿真与导航平台

面向四足机器人（Puppy/PuppyPi）的 ROS 2 导航、SLAM、运动控制与仿真研究项目。仓库同时包含 Python 原型、C++ 核心迁移、Gazebo 仿真、UE5/C++ bridge，以及 PuppyPi/MiniCPM-RobotTrack 适配代码。

> 当前仓库是研究与开发中的工作区快照。部分功能依赖 ROS 2、Gazebo、UE5、CoppeliaSim 或真实硬件，不能仅靠普通 Python 环境完整运行。

## 项目结构

```text
src/                 ROS 2 工作空间源码（主要运行入口）
cpp_src/             C++ 核心、仿真器、bridge 与 C++ 测试
config/              统一参数与 profile 配置
scripts/             构建、回归、配置检查和复现实验脚本
tests/               辅助测试与研究验证
docs/                接口、迁移计划、测试清单和答辩资料
maps/                地图资源
tools/               诊断与实验工具
```

主要 ROS 2 包位于 `src/`：`puppy_bringup`、`puppy_description`、`puppy_nav`、`puppy_slam`、`puppy_core`、`puppypi_adapter` 和 `puppy_minicpm_robot` 等。

## 环境要求

- Ubuntu 22.04（推荐通过 WSL2 使用）
- ROS 2 Humble
- Python 3.10+
- `colcon`、CMake、GCC（C++ 构建时需要）
- Gazebo Classic 11（仿真场景需要）
- 可选：UE5、CoppeliaSim、PuppyPi/Go2 硬件 SDK

## 快速开始（ROS 2）

```bash
source /opt/ros/humble/setup.bash
export PUPPY_SRC=/mnt/e/puppyfangzhen   # 按实际仓库路径修改
export PUPPY_WS=~/puppy_ws
mkdir -p "$PUPPY_WS/src"
cp -a "$PUPPY_SRC/src/." "$PUPPY_WS/src/"
cd "$PUPPY_WS"
colcon build --symlink-install
source install/setup.bash
```

常用启动入口：

```bash
ros2 launch puppy_bringup mock_system.launch.py
ros2 launch puppy_bringup full_system.launch.py
ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py mode:=dry-run
```

真实硬件启动前请先执行硬件预检：

```bash
python src/puppy_minicpm_robot/scripts/preflight_check.py
ros2 launch puppypi_adapter adapter_bringup.launch.py \
  params_file:=src/puppypi_adapter/config/adapter_params.yaml
```

## 验证与开发

```bash
python scripts/run_python_smoke.py
python scripts/check_profile.py
python scripts/check_config_consistency.py
pytest -q
bash scripts/run_regression.sh
```

`dev`、`research`、`release`、`demo` profile 在 `config/unified_params.yaml` 中定义。发布前建议执行：

```bash
python scripts/check_profile.py --assert-release
python scripts/check_docs_consistency.py
```

## 运行模式

| 模式 | 适用场景 |
| --- | --- |
| `mock + core` | 接口契约、状态机、CI 与快速单元测试 |
| `Gazebo + Nav2 + core` | SLAM、路径规划、传感器和闭环导航 |
| `C++/UE5 bridge` | C++ 基准、回归实验和三维可视化 |
| `PuppyPi real + core` | 真实硬件测试（必须先通过预检） |

## 结果与生成文件

日志、构建目录、缓存、截图、实验结果和第三方/硬件目录默认由 `.gitignore` 排除。请不要把密钥、设备配置或大体积二进制文件提交到公开仓库。

```bash
bash scripts/clean_artifacts.sh
# 确认列表无误后再执行：
bash scripts/clean_artifacts.sh --apply
```

## 文档索引

- [接口契约](INTERFACES.md)
- [状态机](STATE_MACHINE.md)
- [C++ 迁移计划](docs/cpp_migration_plan.md)
- [测试清单](docs/test_inventory.md)
- [项目交接记录](HANDOFF_FINAL.md)

## 贡献说明

提交改动前请说明运行环境和验证命令。涉及 ROS 2 topic/service/action 或配置契约的改动，请同步更新 `INTERFACES.md` 和对应测试。

## 许可证

仓库当前未声明统一开源许可证。公开发布前请在根目录补充 `LICENSE`，并确认 `third_party/`、模型和硬件 SDK 的授权条款。
