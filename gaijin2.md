# PuppyPi 实体平台部署与软件重构规划书

## 1. 文档目的

本文档用于指导程序员把当前项目从“仿真中的四足导航与服务机器人原型”，逐步演进成“可部署在 PuppyPi 机器狗平台上的真实系统”。

这不是一份泛泛的想法记录，而是一份面向开发实施的规划书。目标是让程序员看完之后，能够直接开始拆包、定义接口、安排开发顺序，并且清楚哪些地方是可复用的，哪些地方必须重写。

本文档重点回答五个问题：

1. 当前仓库哪些能力可以保留，哪些必须重构。
2. PuppyPi 平台和当前仿真平台之间的关键差异在哪里。
3. 应该如何设计软件架构，才能为以后上 PuppyPi 做准备。
4. 每个包、节点、接口、消息、Action、配置文件分别该承担什么职责。
5. 开发应该按什么阶段推进，如何验收，如何降低实机风险。

---

## 2. 核心结论

### 2.1 不能直接“移植”，必须做“平台适配式重构”

当前项目的核心价值在于：

- 已经形成了完整导航链路
- 已经有 ROS2 系统骨架
- 已经有步态控制与服务机器人节点雏形
- 已经具备仿真验证基础

但它还不能直接上 PuppyPi，原因不是“代码不够多”，而是**平台假设不一致**。

最关键的一条是：

- 当前仓库的机器人模型按 **12 关节、每腿 3 自由度** 来组织
- PuppyPi 实体平台是 **8DOF 联动腿结构**

这意味着：

1. 现有 `puppy_gait` 的关节输出接口不能原样驱动 PuppyPi。
2. 现有 URDF、控制器配置、关节轨迹接口不能直接照搬。
3. 现有 `leg_odometry.cpp` 也不能作为真实里程计方案直接使用。
4. 现有导航与任务层可以保留思路，但底盘执行层必须抽象成平台适配层。

### 2.2 正确方向不是“把所有代码搬上 PuppyPi”，而是“把项目拆成两个部分”

后续系统应分成两大层：

1. **平台无关层**
   负责导航、任务、状态机、传感器融合、地图、巡逻、安全策略、行为编排。

2. **PuppyPi 平台适配层**
   负责 PuppyPi 的底盘控制、步态调用、传感器驱动、板载 SDK 适配、电池与安全状态、底层动作执行。

程序员的真正任务不是“改几个 launch 文件”，而是建立这两个层之间稳定的接口。

### 2.3 最重要的工程目标

为未来部署到 PuppyPi 做准备，最关键的不是先把所有功能都跑起来，而是先建立下面四件事：

1. **平台抽象接口**
2. **底盘控制适配层**
3. **真实传感器输入链**
4. **可控的真机 bringup 和安全机制**

如果这四件事做对了，后续导航、巡逻、安防、交互功能都可以逐步挂上去。

### 2.4 在真正开发前，必须先确认的 PuppyPi 平台信息

程序员正式开工前，必须先和硬件负责人或采购负责人确认下面这些信息。没有这些信息，很多接口只能先按“适配层占位”写，不能直接写死。

必须确认：

1. PuppyPi 的具体型号与硬件版本
2. 官方控制接口是 Python SDK、串口协议、还是板载服务
3. 是否支持连续速度控制
4. 是否支持姿态控制与恢复动作
5. 是否能读到关节状态、IMU、电量、动作模式
6. 是否自带摄像头、IMU、超声、麦克风等
7. LiDAR 是否需要外接
8. 主控板是树莓派、Jetson，还是厂商定制控制器
9. 系统镜像是否允许安装 ROS2 Humble
10. 底层是否已有现成 ROS/ROS2 话题桥接

结论：

在没有确认这些事实之前，程序员**可以先做平台无关层和适配层接口**，但不要把 PuppyPi 具体调用方式写死在上层逻辑里。

---

## 3. 当前仓库与 PuppyPi 的差异分析

这一节非常重要，程序员必须先理解“断层点”在哪里。

## 3.1 运动学结构差异

当前仓库中：

- `src/puppy_gait/src/gait_controller.cpp` 生成 12 个关节目标
- `src/puppy_hardware/config/puppy_controllers.yaml` 也是 12 关节配置
- `src/puppy_description/urdf` 也是典型 12 关节四足结构

而 PuppyPi 平台是消费级成品机器狗，底层通常并不是你现在这种“标准 12 关节 ROS 控制器直驱模型”。

这意味着：

- 当前 `gait_controller` 不能直接作为 PuppyPi 的底层执行器
- 当前 `joint_trajectory_controller` 不能作为 PuppyPi 的唯一真实控制入口
- 当前 `ros2_control` 配置只能保留思路，不能假设它已经适配 PuppyPi

结论：

**步态层必须从“直接输出 12 关节轨迹”改成“输出平台无关的机体运动意图”，再由 PuppyPi 适配器转换成平台动作命令。**

## 3.2 真实执行链差异

当前仓库里，执行层存在明显仿真假设：

- `autonomous_nav.py` 会直接写位姿
- `coppelia_bridge.py` 会积分 `/cmd_vel` 后直接改 base 位姿

但 PuppyPi 真机不可能接受这种控制方式。

真机必须变成：

- 上层输出速度意图或动作意图
- PuppyPi 平台适配层调用其 SDK / 控制接口
- 真实机体运动后，再由传感器和状态估计回传姿态与位置

所以：

**真机部署时 `/cmd_vel -> setObjectPosition` 这种思路必须彻底消失。**

## 3.3 里程计与定位差异

当前 `src/puppy_localization/src/leg_odometry.cpp` 的本质是：

- 订阅 `/cmd_vel`
- 再结合 IMU yaw 做积分
- 输出 `/odom`

这在仿真验证时可以接受，但在 PuppyPi 真机上不够可靠，原因是：

1. `/cmd_vel` 是意图，不是真实运动结果
2. PuppyPi 是腿式平台，存在打滑、失足、地面接触不稳定
3. 真实里程计必须基于传感器反馈，而不是基于“我希望它怎么动”

因此真机至少要升级为以下任一方案：

1. **IMU + 腿部相位/步态估计的里程计**
2. **IMU + 视觉里程计**
3. **IMU + 激光定位 / AMCL**
4. **IMU + 足端接触估计 + 激光建图定位**

结论：

`leg_odometry.cpp` 当前只能保留为占位实现，不能视为真机可用方案。

## 3.4 传感器输入差异

当前仿真系统已经假设有：

- `/scan`
- `/imu/data`
- `/camera/rgb/image_raw`
- `/camera/depth/image_raw`

但 PuppyPi 真机是否自带这些传感器、接口是否一致、带宽是否够、驱动是否稳定，都要单独设计。

因此后续代码不能再默认“传感器一定存在”，而必须引入：

- 设备发现
- 启动前检查
- topic readiness 检查
- 模块降级运行

## 3.5 启动方式差异

当前 bringup 大量依赖：

- `TimerAction`
- 固定延迟

这对仿真能用，但真机不稳。

PuppyPi 真机部署必须改成：

- 设备是否在线
- 驱动是否 ready
- IMU 是否有数据
- 激光是否有数据
- 底盘控制接口是否 active
- 安全状态是否解除锁定

才能进入导航或巡逻。

---

## 4. 面向 PuppyPi 的总体软件架构

## 4.1 总体原则

未来系统建议采用四层结构：

1. **Application Layer**
2. **Robot Capability Layer**
3. **Platform Adapter Layer**
4. **Hardware / SDK Layer**

### 第一层：Application Layer

负责业务逻辑：

- 巡逻
- 安防
- 情绪交互
- 回充
- 任务调度
- 模式切换

### 第二层：Robot Capability Layer

负责机器人通用能力：

- 导航
- 定位
- 地图
- 目标点执行
- 状态估计
- 感知事件
- 安全约束

这一层必须尽可能平台无关。

### 第三层：Platform Adapter Layer

这是最关键的一层，专门处理 PuppyPi：

- PuppyPi 底盘命令适配
- PuppyPi 传感器消息适配
- PuppyPi 电量 / 错误 / 状态适配
- PuppyPi 动作模式切换

### 第四层：Hardware / SDK Layer

负责与 PuppyPi 官方控制库、驱动程序、串口板、相机、雷达等真实设备通信。

---

## 4.2 推荐包结构

建议不要继续把所有逻辑塞在现有包里，而是新增如下结构。

### 保留并重构的包

- `puppy_nav`
- `puppy_localization`
- `puppy_bringup`
- `puppy_gait`

### 新增的包

- `puppy_core`
- `puppy_interfaces`
- `puppypi_adapter`
- `puppypi_bringup`
- `puppypi_sensors`
- `puppypi_control`
- `puppypi_diagnostics`
- `puppypi_description`
- `puppypi_mock`

### 4.3 推荐的目录结构

建议未来代码组织成下面这样，便于程序员拆模块、并行开发和后期维护：

```text
src/
  puppy_interfaces/
    msg/
    srv/
    action/
  puppy_core/
    puppy_core/
      mission_manager.py
      mode_manager.py
      safety_manager.py
      goal_dispatcher.py
      robot_state_aggregator.py
      event_hub.py
    launch/
    config/
  puppy_nav/
    launch/
    config/
    scripts/
  puppy_localization/
    launch/
    config/
    src/
  puppypi_adapter/
    puppypi_adapter/
      motion_adapter_node.py
      status_adapter_node.py
      mode_adapter_node.py
      dock_adapter_node.py
    launch/
    config/
  puppypi_control/
    puppypi_control/
      puppypi_client.py
      motion_primitives.py
      velocity_bridge.py
      posture_controller.py
      gait_mode_selector.py
    test/
  puppypi_sensors/
    puppypi_sensors/
      imu_driver_node.py
      lidar_driver_node.py
      camera_driver_node.py
      battery_driver_node.py
      mic_driver_node.py
    launch/
    config/
  puppypi_bringup/
    launch/
      hardware_bringup.launch.py
      navigation_bringup.launch.py
      full_service_bringup.launch.py
    config/
  puppypi_description/
    urdf/
    meshes/
    rviz/
  puppypi_diagnostics/
    puppypi_diagnostics/
      doctor.py
      checks/
    launch/
  puppypi_mock/
    puppypi_mock/
      fake_motion_server.py
      fake_status_server.py
      fake_sensor_publishers.py
```

---

## 5. 每个包应该做什么

下面这一节是给程序员直接拆包用的。

## 5.1 `puppy_interfaces`

作用：统一定义项目内部的消息、服务、Action。

这个包必须先做，因为后续所有模块都要依赖它。

建议定义：

### Msg

- `RobotMode.msg`
- `RobotHealth.msg`
- `BatteryStatus.msg`
- `PatrolStatus.msg`
- `SecurityEvent.msg`
- `FallEvent.msg`
- `DockStatus.msg`
- `PlatformMotionState.msg`

建议字段如下。

#### `RobotMode.msg`

```text
std_msgs/Header header
string current_mode
string previous_mode
bool motion_enabled
bool autonomy_enabled
string reason
```

#### `RobotHealth.msg`

```text
std_msgs/Header header
bool ok
string level          # OK/WARN/ERROR/FATAL
string[] active_faults
float32 cpu_temp
float32 battery_percent
bool imu_ready
bool lidar_ready
bool camera_ready
bool motion_ready
```

#### `BatteryStatus.msg`

```text
std_msgs/Header header
float32 voltage
float32 current
float32 percent
bool charging
bool low_battery
bool critical_battery
```

#### `PatrolStatus.msg`

```text
std_msgs/Header header
string state
int32 current_waypoint_index
int32 total_waypoints
float32 completion_ratio
string message
```

#### `SecurityEvent.msg`

```text
std_msgs/Header header
string event_type     # intrusion / gas / abnormal_sound / manual_alarm
string severity       # info / warn / critical
string source_node
geometry_msgs/PoseStamped pose
string message
```

#### `FallEvent.msg`

```text
std_msgs/Header header
bool detected
string direction
float32 confidence
string source
```

#### `DockStatus.msg`

```text
std_msgs/Header header
bool dock_visible
bool aligned
bool charging
float32 distance_estimate
string stage
```

#### `PlatformMotionState.msg`

```text
std_msgs/Header header
bool standing
bool moving
bool controllable
string gait_mode
float32 linear_x
float32 linear_y
float32 angular_z
string platform_state
```

### Srv

- `SetRobotMode.srv`
- `ClearFault.srv`
- `EnableMotors.srv`
- `DisableMotors.srv`
- `StartMapping.srv`
- `StopMapping.srv`

建议字段：

#### `SetRobotMode.srv`

```text
string mode
string reason
---
bool success
string message
```

#### `EnableMotors.srv`

```text
bool enable
---
bool success
string message
```

### Action

- `NavigateToPose.action`
- `PatrolRoute.action`
- `Dock.action`
- `StandUp.action`
- `SitDown.action`
- `RecoverPosture.action`

建议最少定义以下语义。

#### `PatrolRoute.action`

Goal:

```text
geometry_msgs/PoseStamped[] waypoints
bool loop
float32 pause_seconds
```

Result:

```text
bool success
int32 reached_waypoints
string message
```

Feedback:

```text
int32 current_waypoint_index
float32 distance_to_waypoint
string state
```

#### `Dock.action`

Goal:

```text
string dock_id
bool force
```

Result:

```text
bool success
bool charging
string message
```

Feedback:

```text
string stage
float32 distance_estimate
bool dock_visible
```

设计原则：

1. 不要再大量依赖 `std_msgs/String` 表意。
2. 所有跨模块动作都要结构化。
3. 任务必须可反馈、可取消、可失败返回原因。

## 5.2 `puppy_core`

作用：承载平台无关的机器人能力层。

建议包含：

- `mission_manager.py`
- `mode_manager.py`
- `safety_manager.py`
- `goal_dispatcher.py`
- `event_hub.py`
- `robot_state_aggregator.py`

职责：

1. 统一管理机器人当前模式
2. 汇总导航状态、底盘状态、电池状态、传感器状态
3. 对上提供任务级接口
4. 对下调用导航 Action、底盘 Action、回充 Action

这层不应该知道 PuppyPi SDK 的细节。

建议再补一个 `capability_registry.py`，用于统一声明当前系统能力开关，例如：

- 是否支持导航
- 是否支持回充
- 是否支持 RGB-D
- 是否支持语音交互

这样以后在真机能力不完整时，可以优雅降级，而不是直接崩。

## 5.3 `puppypi_adapter`

作用：这是整个真机落地的核心包。

职责：

1. 把通用层的运动意图翻译成 PuppyPi 可执行指令
2. 把 PuppyPi 平台状态转换成 ROS2 标准话题与内部接口消息
3. 管理底盘可用状态、姿态模式、错误恢复

建议拆成以下节点：

### `puppypi_motion_adapter_node`

输入：

- `/cmd_vel` 或内部导航运动意图
- `StandUp.action`
- `SitDown.action`
- `RecoverPosture.action`

输出：

- 发送给 PuppyPi 底盘控制 SDK 的动作命令
- `/platform/motion_state`

内部状态机建议：

- INIT
- DISABLED
- READY
- EXECUTING
- RECOVERING
- SAFE_STOP
- FAULT

### `puppypi_status_adapter_node`

输入：

- PuppyPi 板载反馈
- 电机状态
- 姿态状态
- 错误状态

输出：

- `/robot/health`
- `/joint_states`（如果能提供）
- `/platform/faults`
- `/battery_state`

### `puppypi_mode_adapter_node`

负责 PuppyPi 上的姿态模式切换，例如：

- 站立
- 卧倒
- 恢复
- 禁止运动
- 巡航使能

注意：

PuppyPi 很可能不是“连续速度控制平台优先”，而是“动作模式 + 指令封装平台优先”。这一层就是把这件事消化掉。

## 5.3.1 `puppypi_mock`

作用：给上层程序员脱离真机开发提供 mock 环境。

必须实现：

1. 假底盘动作服务
2. 假平台状态反馈
3. 假电池变化
4. 假传感器 ready 状态

这样 `puppy_core`、任务系统、模式机、安全逻辑都可以在没有 PuppyPi 真机时先开发和联调。

## 5.4 `puppypi_sensors`

作用：统一封装 PuppyPi 真机传感器。

建议拆成：

- `imu_driver_node`
- `lidar_driver_node`
- `camera_driver_node`
- `mic_driver_node`
- `battery_driver_node`
- `touch_driver_node`（如果 PuppyPi 有）

要求：

1. 所有传感器必须发布到统一命名的 ROS2 话题
2. 命名尽量与当前仿真话题一致
3. 每个节点都要带健康状态输出

推荐目标话题：

- `/scan`
- `/imu/data`
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`
- `/battery_state`

如果 PuppyPi 没有深度相机，就明确降级为：

- `/camera/color/image_raw`

并让视觉模块支持 RGB-only 模式。

建议所有传感器节点都统一发布一个配套健康话题，例如：

- `/sensor_status/imu`
- `/sensor_status/lidar`
- `/sensor_status/camera`

字段至少包含：

- `ready`
- `hz`
- `last_msg_age_ms`
- `device_name`
- `error_message`

## 5.5 `puppypi_control`

作用：面向 PuppyPi 的底盘控制逻辑封装。

这是“适配器之下、SDK 之上”的一层，建议独立出来，不要把 SDK 调用散落在各个业务节点里。

建议内部模块：

- `puppypi_client.py`
- `motion_primitives.py`
- `velocity_bridge.py`
- `posture_controller.py`
- `gait_mode_selector.py`

职责：

1. 封装 PuppyPi SDK / 串口协议 / Python API
2. 统一暴露：
   - `set_velocity(vx, vy, wz)`
   - `stand()`
   - `sit()`
   - `stop()`
   - `recover()`
3. 对外屏蔽具体平台细节

## 5.6 `puppypi_diagnostics`

作用：真机运行时诊断。

建议包含：

- `device_checker.py`
- `topic_checker.py`
- `latency_checker.py`
- `power_checker.py`
- `thermal_checker.py`

输出：

- `/diagnostics`
- `/robot/health`

并提供统一命令行入口：

- `ros2 run puppypi_diagnostics doctor --full`
- `ros2 run puppypi_diagnostics doctor --motion`
- `ros2 run puppypi_diagnostics doctor --sensors`

## 5.7 `puppypi_description`

作用：维护 PuppyPi 真实机体的 URDF/Xacro。

注意：

这个包不能继续复用当前 12 关节描述作为“真实机体定义”，最多只能参考。

应包含：

- PuppyPi 真实 link/joint 结构
- 真实传感器外参
- 真实 base_link / imu_link / laser_link / camera_link 关系

目标是让：

- TF 树正确
- 传感器外参正确
- RViz 中真实模型可视化正确

---

## 6. 推荐的节点图与职责划分

## 6.1 顶层节点关系

建议真机运行时的主链路如下：

1. 传感器节点发布
   - `/scan`
   - `/imu/data`
   - `/camera/...`
   - `/battery_state`

2. 定位节点运行
   - 激光定位 / SLAM / AMCL
   - EKF
   - odom 发布

3. PuppyPi 运动适配节点运行
   - 接收 `/cmd_vel`
   - 调用 PuppyPi SDK
   - 输出平台状态

4. Nav2 或自定义导航运行
   - 接收 odom/scan/map
   - 输出速度意图

5. `puppy_core` 任务层运行
   - 统一调用导航和底盘
   - 管理巡逻、安防、回充、模式

## 6.3 建议统一的话题表

程序员开发时，尽量收敛到下面这套话题，不要每个节点自己发明命名。

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop / core | motion_adapter | 通用速度意图 |
| `/scan` | `sensor_msgs/LaserScan` | lidar_driver | localization/nav | 激光数据 |
| `/imu/data` | `sensor_msgs/Imu` | imu_driver | localization | IMU 数据 |
| `/battery_state` | `sensor_msgs/BatteryState` | battery_driver/status_adapter | core/safety | 电池状态 |
| `/robot/mode` | `puppy_interfaces/RobotMode` | mode_manager | 全局 | 模式广播 |
| `/robot/health` | `puppy_interfaces/RobotHealth` | state_aggregator | 全局 | 健康状态 |
| `/platform/motion_state` | `puppy_interfaces/PlatformMotionState` | motion_adapter | core | 平台运动状态 |
| `/security/event` | `puppy_interfaces/SecurityEvent` | security_node | core | 安防事件 |
| `/fall/event` | `puppy_interfaces/FallEvent` | fall_detection | core | 跌倒事件 |
| `/dock/status` | `puppy_interfaces/DockStatus` | dock_adapter | core | 充电桩状态 |

## 6.2 节点级职责

### `robot_state_aggregator`

订阅：

- `/battery_state`
- `/diagnostics`
- `/platform/motion_state`
- `/amcl_pose` 或 `/odom`
- `/robot/fall_event`

发布：

- `/robot/state`
- `/robot/health`

### `mode_manager`

管理模式：

- IDLE
- READY
- TELEOP
- NAVIGATION
- PATROL
- SECURITY
- DOCKING
- SAFE_STOP
- FAULT

要求：

1. 模式切换必须有条件检查
2. FAULT 和 SAFE_STOP 优先级最高
3. 任意时候电量过低或跌倒都能中断当前任务

### `mission_manager`

职责：

- 接收高层任务
- 调用导航 Action
- 调用底盘动作 Action
- 协调巡逻、回充、警报处理

不能直接操作 PuppyPi SDK，必须通过适配层。

---

## 7. 控制接口设计

这一部分是程序员最容易做错的地方。

## 7.1 统一“运动意图接口”

为了避免所有上层模块都直接依赖 PuppyPi 特定控制方式，建议定义一个统一运动接口：

### 输入接口

- `/cmd_vel`：通用移动意图
- `/robot/posture_cmd`：姿态动作命令
- `/robot/motion_enable`：使能/失能

### 输出接口

- `/platform/motion_state`
- `/platform/cmd_result`

`puppypi_motion_adapter_node` 负责把这些通用接口翻译成 PuppyPi 平台动作。

## 7.2 姿态动作接口

建议定义：

- `STAND`
- `SIT`
- `LIE_DOWN`
- `RECOVER`
- `FREEZE`

不要把这些动作散在不同节点里硬编码。

## 7.3 速度命令的适配策略

PuppyPi 平台不一定天然适合连续线速度角速度控制，所以适配层需要做以下事情：

1. 限幅
2. 平滑
3. 死区处理
4. 安全刹停
5. 指令超时清零

建议参数化：

- `max_linear_x`
- `max_linear_y`
- `max_angular_z`
- `cmd_timeout`
- `accel_limit`
- `yaw_rate_limit`

## 7.4 真机动作安全策略

适配层必须内建下面几条：

1. 若未站立，禁止接收导航速度命令
2. 若检测到跌倒，立即停止并进入恢复流程
3. 若低电量，拒绝进入巡逻模式
4. 若 SDK 无响应，进入 SAFE_STOP

### 7.5 建议的控制回路频率

程序员实现时建议统一频率约束，避免系统各层随意设置：

- 传感器驱动：按设备原生频率
- 运动适配输出：20Hz 到 50Hz
- 平台状态回读：20Hz
- 模式机：10Hz
- 任务管理：5Hz 到 10Hz
- diagnostics：1Hz 到 2Hz

这样既不会让 PuppyPi 平台负载过高，也方便排查时序问题。

---

## 8. 定位与导航方案建议

## 8.1 真机定位优先方案

建议优先采用：

### 第一阶段

- IMU + 2D LiDAR + AMCL / SLAM Toolbox

理由：

1. 相比腿式里程计更容易落地
2. 对 PuppyPi 平台侵入小
3. 更适合先把导航跑通

### 第二阶段

- IMU + 激光 + 视觉辅助

### 第三阶段

- 加入更可靠的腿式里程估计

## 8.2 暂不建议把当前 `leg_odometry.cpp` 直接当真机定位方案

因为它本质依赖 `/cmd_vel`，而不是依赖真实机体反馈。

程序员应该把它改造成以下形式之一：

### 方案 A：占位型 odom 节点

只在没有更好来源时提供最基础 odom，并明确标记为低可信。

### 方案 B：平台反馈型 odom 节点

如果 PuppyPi SDK 能返回步态/速度估计，则使用平台反馈生成 `/odom`。

### 方案 C：融合型 odom 节点

融合：

- IMU
- 平台速度估计
- 激光定位结果

输出更稳定 odom。

## 8.3 地图与导航建议

建议真机优先使用 ROS2 路线，不建议继续依赖 `autonomous_nav.py` 作为真机主入口。

原因：

1. 真机更需要标准化 lifecycle
2. 真机更需要可控的 TF、clock、传感器话题
3. 真机更需要可维护的 bringup

因此真机推荐路线是：

- `puppypi_bringup`
- `puppy_localization`
- `puppy_nav`
- `puppy_core`
- `puppypi_adapter`

而不是 `autonomous_nav.py` 直连。

### 8.4 导航参数建议单独维护 PuppyPi 版本

不要直接沿用仿真参数。

建议新增：

- `nav2_params_puppypi.yaml`
- `ekf_puppypi.yaml`

至少需要单独调的参数包括：

1. 最大速度
2. 最小原地旋转角速度
3. inflation 半径
4. footprint
5. 恢复行为
6. 激光更新频率与超时
7. 定位初始协方差

---

## 9. 任务系统设计

为了给未来功能扩展留空间，必须从现在开始把任务系统正规化。

## 9.1 顶层任务类型

建议定义以下任务：

1. `GoToPose`
2. `PatrolRoute`
3. `ReturnToDock`
4. `SecurityCheck`
5. `RespondToFall`
6. `SpeakMessage`
7. `FollowPerson`（未来可选）

## 9.2 任务优先级

优先级建议如下：

1. `SAFE_STOP`
2. `RespondToFall`
3. `ReturnToDock`（低电）
4. `SecurityCheck`
5. `GoToPose`
6. `PatrolRoute`
7. `Idle behaviors`

## 9.3 行为编排

推荐用状态机先落地，后续如有需要再升级为行为树。

第一版不要一开始就追求复杂 BT 图，而是先做稳定状态机：

- IDLE
- READY
- NAVIGATING
- PATROLLING
- DOCKING
- RECOVERING
- SAFE_STOP
- FAULT

---

## 10. 真机 Bringup 设计

## 10.1 建议拆分为三级 bringup

### Level 1: Hardware Bringup

只起：

- PuppyPi 平台控制适配
- IMU
- LiDAR
- Camera
- Battery
- Diagnostics

目标：

- 确认传感器工作
- 确认底盘控制工作
- 确认 TF 初始正确

### Level 2: Navigation Bringup

在 Level 1 基础上，再起：

- localization
- map server / slam toolbox
- Nav2

目标：

- 真机能完成定位与点到点导航

### Level 3: Full Service Bringup

在 Level 2 基础上，再起：

- mission_manager
- patrol
- security
- emotion_interaction
- docking manager

目标：

- 真机完成业务闭环

## 10.2 启动顺序原则

必须从“时间驱动”改成“就绪驱动”。

推荐顺序：

1. 平台控制适配层 ready
2. IMU ready
3. LiDAR ready
4. TF 基础树 ready
5. localization ready
6. Nav2 active
7. mission_manager active

每一步都要有 readiness check。

### 10.3 建议的 readiness check 条件

程序员实现 bringup 检查脚本时，可直接按下面条件：

#### `hardware_ready`

- PuppyPi 控制 SDK 可连接
- `/imu/data` 2 秒内持续更新
- `/battery_state` 可读

#### `sensor_ready`

- `/scan` 频率稳定
- 相机节点已启动
- TF 中存在 `base_link -> imu_link`

#### `localization_ready`

- EKF active
- `/odom` 持续更新
- `/tf` 中存在 `odom -> base_link`

#### `navigation_ready`

- Nav2 lifecycle 全部 active
- 可接收导航目标
- 初始位姿已设置

#### `service_ready`

- mission manager active
- safety manager active
- 模式机处于 READY

---

## 11. 配置体系设计

为了让程序员好维护，必须把配置文件标准化。

建议目录：

- `config/platform/puppypi.yaml`
- `config/platform/motion_limits.yaml`
- `config/platform/sensors.yaml`
- `config/nav/nav2_params_puppypi.yaml`
- `config/localization/ekf_puppypi.yaml`
- `config/core/modes.yaml`
- `config/core/safety.yaml`

## 11.1 `puppypi.yaml`

内容建议：

- 平台名称
- SDK 连接方式
- 控制频率
- 是否支持连续速度控制
- 是否支持姿态恢复
- 电量阈值

## 11.2 `motion_limits.yaml`

- 最大前进速度
- 最大横移速度（若不支持则置零）
- 最大转速
- 最大加速度
- 超时停止时长

## 11.3 `safety.yaml`

- 低电量阈值
- 危险低电量阈值
- 跌倒检测触发策略
- 故障进入 SAFE_STOP 条件
- 恢复重试次数

### 11.4 建议增加 `features.yaml`

用于控制某些功能是否启用：

- `enable_navigation`
- `enable_patrol`
- `enable_security`
- `enable_emotion`
- `enable_fall_detection`
- `enable_docking`

这样在不同 PuppyPi 配置版本下可以快速裁剪功能。

---

## 12. 程序员开发任务拆解

下面这一部分可以直接作为开发拆解依据。

## 阶段 A：抽象层搭建

### 目标

把现有工程从“和仿真耦合”改成“能接不同底盘平台”。

### 任务

1. 新建 `puppy_interfaces`
2. 新建 `puppy_core`
3. 把现有字符串型事件改成结构化消息/Action
4. 抽象机器人模式机和任务管理器

### 验收

- 所有上层模块不再直接依赖 PuppyPi SDK
- 核心任务流可以只依赖接口跑 mock

代码产出：

- `puppy_interfaces`
- `puppy_core`
- `puppypi_mock`

## 阶段 B：PuppyPi 底盘适配

### 目标

建立 PuppyPi 平台控制入口。

### 任务

1. 新建 `puppypi_control`
2. 新建 `puppypi_adapter`
3. 实现：
   - stand
   - sit
   - stop
   - recover
   - set_velocity
4. 加入指令超时和限幅

### 验收

- 可以通过 ROS2 接口控制 PuppyPi 站立、停止、移动
- 断开控制命令后能自动停车

代码产出：

- `puppypi_control`
- `puppypi_adapter`
- `hardware_bringup.launch.py`

## 阶段 C：传感器适配

### 目标

让真机传感器输入与现有导航系统对齐。

### 任务

1. 新建 `puppypi_sensors`
2. 打通 `/scan`
3. 打通 `/imu/data`
4. 打通 `/camera/...`
5. 打通 `/battery_state`

### 验收

- RViz 中可见点云/激光/图像/TF
- 传感器掉线能被 diagnostics 检测

代码产出：

- `puppypi_sensors`
- `puppypi_description`
- `puppypi_diagnostics`

## 阶段 D：定位导航

### 目标

让 PuppyPi 真机具备可控导航能力。

### 任务

1. 配置 `puppypi_description`
2. 修正 TF
3. 配置 `ekf_puppypi.yaml`
4. 配置 `nav2_params_puppypi.yaml`
5. 起 AMCL 或 SLAM Toolbox

### 验收

- 真机能完成建图
- 真机能完成定点导航
- 导航过程中不会因为底盘姿态异常直接崩掉

代码产出：

- `navigation_bringup.launch.py`
- `nav2_params_puppypi.yaml`
- `ekf_puppypi.yaml`

## 阶段 E：任务层上线

### 目标

让巡逻、安防、回充、恢复等功能进入同一个任务系统。

### 任务

1. `mission_manager`
2. `mode_manager`
3. `safety_manager`
4. `dock_manager`
5. 巡逻 Action 封装

### 验收

- 低电时自动中断巡逻并返航
- 跌倒时自动进入恢复流程
- 可从 READY / PATROL / DOCKING / FAULT 模式间切换

代码产出：

- `full_service_bringup.launch.py`
- `dock_manager.py`
- `mission_manager.py`
- `safety_manager.py`

---

## 13. 最需要程序员立刻修改的现有代码

这部分是直接点名。

## 13.1 必须降级为“参考实现”的模块

以下模块不能再当真机主实现：

- `autonomous_nav.py`
- `src/puppy_bringup/scripts/coppelia_bridge.py`
- `src/puppy_localization/src/leg_odometry.cpp`

原因：

- 强仿真假设
- 位姿写入式控制
- `/cmd_vel` 伪里程计

## 13.2 可以保留思想、但必须改接口的模块

- `src/puppy_gait/src/gait_controller.cpp`
- `src/puppy_nav/*`
- `src/puppy_gait/scripts/task_scheduler.py`
- `src/puppy_gait/scripts/security_node.py`
- `src/puppy_gait/scripts/emotion_interaction.py`

改造方向：

1. 不再直接假设 12 关节
2. 不再直接假设字符串消息足够表达任务
3. 不再把仿真控制逻辑和业务逻辑绑死

### 13.3 建议优先冻结、不再继续往里堆逻辑的旧文件

为了避免团队成员继续把新逻辑写进旧仿真脚本里，建议明确冻结以下文件，只允许做 bugfix，不允许继续承担新功能：

- `autonomous_nav.py`
- `src/puppy_bringup/launch/coppelia_system.launch.py`
- `src/puppy_bringup/launch/full_system.launch.py`

新开发统一进入：

- `puppy_core/*`
- `puppypi_*/*`

---

## 14. 真机安全设计要求

程序员在设计时必须把安全当成第一优先级。

必须具备：

1. 指令超时自动停车
2. 模式切换时清空运动指令
3. 跌倒后禁止继续导航
4. 低电量禁止继续巡逻
5. 驱动异常进入 SAFE_STOP
6. 启动时默认电机不上使能，待人工确认

如果 PuppyPi SDK 支持，还应加入：

- 姿态异常锁定
- 电机过热报警
- 控制链心跳检测

---

## 15. 建议的交付物清单

程序员最终交付不应只是“代码能跑”，而应包括：

1. 包结构与源码
2. 接口定义文档
3. `README_puppypi.md`
4. 真机启动说明
5. 配置文件说明
6. 已知问题清单
7. 诊断脚本
8. 演示脚本

另外必须提供三套启动方式：

1. `hardware_bringup`
2. `navigation_bringup`
3. `full_service_bringup`

### 15.1 还应交付的工程文档

建议再加四份配套文档：

1. `INTERFACES.md`
   - 列出所有 topic / service / action / 参数

2. `STATE_MACHINE.md`
   - 列出所有模式与切换条件

3. `PUPPYPI_ADAPTER_NOTES.md`
   - 记录 SDK 限制、已知坑、平台注意事项

4. `TEST_PLAN.md`
   - 记录真机验证用例与通过标准

### 15.2 建议的验收用例

程序员交付时建议按以下用例验收：

1. 上电后 60 秒内完成 `hardware_ready`
2. 人工发送站立命令，机器人可进入站立状态
3. 发送 `/cmd_vel`，机器人可低速前进、停止
4. 激光与 IMU 可在 RViz 中正确显示
5. 机器人可接收 1 个导航点并移动到目标附近
6. 电池低于阈值时自动进入 RETURN_TO_DOCK 模式
7. 模拟跌倒事件后，系统进入 SAFE_STOP 或恢复流程
8. 任一关键传感器断开后，`/robot/health` 变为 WARN 或 ERROR

---

## 16. 最终建议

如果要为以后上 PuppyPi 做准备，程序员应该遵循一个基本原则：

**不要把 PuppyPi 当成当前仿真狗的“实体版”，而要把它当成一个需要专门适配的底盘平台。**

这句话非常关键。

因为从软件工程角度看，真正应该沉淀的是：

- 通用导航能力
- 通用任务能力
- 通用安全机制
- 通用状态管理

而不是把这些能力绑死在某一种仿真执行模型上。

所以这份规划书的本质目标是：

1. 保住你现在已有的算法和系统积累
2. 用平台适配层把 PuppyPi 接进来
3. 让未来无论是继续用 PuppyPi，还是换别的实体四足平台，都不用重写上层系统

如果照这个方向做，最终得到的不会只是“PuppyPi 能跑”，而是“一套可以接入 PuppyPi 的机器人软件平台”。
