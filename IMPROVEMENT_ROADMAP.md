# PuppyPi 项目改进路线图

更新时间：2026-07-06

这份文档把当前仓库的主要问题整理成一份可执行的改进清单。目标不是继续堆功能，而是先把项目从“能跑实验 / 有产品化骨架”推进到“接口统一 / 链路闭环 / 可持续维护”。

## 1. 现状判断

当前仓库实际上并行存在两条路线：

1. `CoppeliaSim + 纯 Python` 研究主线
2. `ROS2 + puppy_core + puppypi_adapter` 产品化骨架

当前成熟度判断：

- 研究原型能力：较强
- ROS2 架构方向：正确
- 真机 PuppyPi 接入完成度：较低
- 自动化回归能力：较弱

最关键的现实问题不是“算法还不够多”，而是：

- 接口契约没有统一
- 同一能力存在多套控制路径
- 真机适配层仍以占位实现为主
- launch、action、topic、service 还没有收敛成一条权威运行链路

## 2. 总体改造原则

后续改造建议遵守下面四条原则：

1. 先统一接口，再补功能。
2. 只保留一个权威任务编排层。
3. 仿真链和真机链共享同一套上层契约。
4. 用自动化测试把契约固定下来。

推荐的边界划分：

- 原始设备/仿真层：标准 ROS 消息
  - 例如 `sensor_msgs/BatteryState`、`sensor_msgs/JointState`、`sensor_msgs/LaserScan`
- 适配/聚合层：`puppy_interfaces` 语义消息
  - 例如 `BatteryStatus`、`FallEvent`、`PlatformMotionState`、`RobotHealth`
- 任务/模式/安全层：只消费语义消息，不直接依赖底层原始 topic
- 导航动作层：短期内以 Nav2 标准 action 为唯一运行时导航接口

## 3. P0：必须先解决的问题

这些问题不修，系统即使“能起节点”，也很难形成可靠闭环。

### P0-1 统一电池、跌倒、平台状态接口契约

**问题**

- `puppy_core` 多处订阅 `puppy_interfaces/BatteryStatus`
- `puppypi_adapter` 和仿真脚本实际发布的是 `sensor_msgs/BatteryState`
- 跌倒、安全、健康状态也分别存在“原始 topic”和“上层语义 topic”混用

**涉及文件**

- `src/puppy_core/puppy_core/mode_manager.py`
- `src/puppy_core/puppy_core/safety_manager.py`
- `src/puppy_core/puppy_core/mission_manager.py`
- `src/puppy_core/puppy_core/robot_state_aggregator.py`
- `src/puppypi_adapter/puppypi_adapter/status_adapter_node.py`
- `src/puppy_gait/scripts/battery_simulator.py`
- `src/puppy_gait/scripts/fall_detection.py`
- `src/puppy_gait/scripts/security_node.py`

**建议做法**

- 明确一条统一规则：
  - 底层保留原始标准消息
  - 上层只使用 `puppy_interfaces` 语义消息
- 由 adapter 或 aggregator 负责把 `BatteryState` 转成 `BatteryStatus`
- `fall_detection` 输出统一改成 `FallEvent`
- `security_node` 输出统一改成 `SecurityEvent`

**验收标准**

- 所有 `puppy_core` 节点不再直接依赖 `sensor_msgs/BatteryState`
- 低电量回充、跌倒停机、安全事件中断都能在同一条语义链路上触发
- mock、Gazebo、Coppelia、真机四种来源都能映射到同一套上层消息

### P0-2 修正 topic / service / action 语义混用

**问题**

- `mode_adapter_node.py` 把 `EnableMotors.srv` 当作订阅消息使用
- `safety_manager.py` 实际发布的是 `Bool`
- 导航同时存在：
  - 自定义 `puppy_interfaces/NavigateToPose`
  - Nav2 标准 `nav2_msgs/NavigateToPose`
- `mission_manager.py` 依赖 `patrol_route` 和 `dock` action client，但仓库内没有对应 server 实现

**涉及文件**

- `src/puppypi_adapter/puppypi_adapter/mode_adapter_node.py`
- `src/puppy_core/puppy_core/safety_manager.py`
- `src/puppy_core/puppy_core/mission_manager.py`
- `src/puppy_core/puppy_core/goal_dispatcher.py`
- `src/puppy_interfaces/action/NavigateToPose.action`
- `src/puppy_interfaces/action/PatrolRoute.action`
- `src/puppy_interfaces/action/Dock.action`
- `src/puppy_nav/scripts/patrol.py`
- `src/puppy_gait/scripts/patrol_mission.py`

**建议做法**

- 电机使能采用单一语义：
  - 请求：service，例如 `EnableMotors.srv`
  - 状态：topic，例如 `/robot/motion_enable` 或 `/robot/motors_enabled`
- 导航运行时先统一为 Nav2 标准 action
- 暂时不要让运行链路同时依赖自定义和标准两个导航 action
- `PatrolRoute`、`Dock` 要么补 server，要么从当前运行链路移除，避免“有 client 无 server”

**验收标准**

- 电机使能链路从请求到执行只有一套协议
- 所有正在使用的 action 都能找到真实 server
- 运行中的巡逻、手动目标、回充目标不再混用两种导航动作定义

### P0-3 补齐真实 PuppyPi 适配层，至少达到“诚实可用”

**问题**

- `motion_adapter` 的 SDK 初始化、速度下发、状态回读还是 TODO
- `status_adapter` 还在发布占位 joint/battery 数据
- `mode_adapter` 的姿态切换和电机控制也没有真正落地

**涉及文件**

- `src/puppypi_adapter/puppypi_adapter/motion_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/status_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/mode_adapter_node.py`
- `src/puppypi_adapter/config/adapter_params.yaml`

**建议做法**

- 抽一个统一的 PuppyPi SDK client 封装层
- `motion_adapter` 负责：
  - 速度限幅
  - 超时停机
  - SDK 下发
  - 平台状态回读
- `status_adapter` 负责：
  - 电池
  - 关节
  - 故障
  - 原始健康状态
- `mode_adapter` 负责：
  - 站立 / 坐下 / 恢复动作
  - 电机使能
- 如果短期接不上真 SDK，就在 launch 和文档里明确标记“未完成，不属于当前支持路径”

**验收标准**

- `motion_adapter` 能从 `INIT` 进入 `READY`
- `cmd_vel_safe` 能真实驱动平台移动并在超时后停下
- `status_adapter` 不再发布占位常量数据
- 真机链路和 mock 链路对上层表现一致

### P0-4 收敛一条权威 bringup 路径

**问题**

- 目前有多套入口：
  - Gazebo 导航
  - Gazebo 全系统
  - Coppelia 全系统
  - mock bringup
  - core bringup
  - adapter bringup
- 这些入口没有收敛到统一的上层运行协议
- `goal_dispatcher` 存在，但没有进入 `setup.py` 的可执行入口，也没有进核心 bringup
- `adapter_bringup.launch.py` 里参数路径使用相对路径，不够稳

**涉及文件**

- `src/puppy_bringup/launch/full_system.launch.py`
- `src/puppy_bringup/launch/navigation.launch.py`
- `src/puppy_bringup/launch/coppelia_system.launch.py`
- `src/puppy_core/launch/core_bringup.launch.py`
- `src/puppypi_adapter/launch/adapter_bringup.launch.py`
- `src/puppy_core/setup.py`

**建议做法**

- 定义四条明确支持的入口：
  1. `mock + core`
  2. `Gazebo + Nav2 + core`
  3. `Coppelia research stack`
  4. `PuppyPi real hardware + core`
- 每个入口都写清“包含哪些节点、对哪些接口负责”
- `goal_dispatcher` 若保留，应加入安装入口并纳入对应 bringup
- 所有 launch 的配置文件路径都改为 package share 路径

**验收标准**

- 每个官方入口都能在干净工作空间中启动
- 启动后不存在“节点起来了但关键 server/topic 缺失”的情况
- 文档中的入口和实际可运行入口完全一致

### P0-5 统一坐标、命名目标和关键环境约定

**问题**

- 充电桩坐标在不同脚本中不一致
- `/amcl_pose` 的消息类型假设与标准 AMCL 不一致
- waypoint、dock、房间名、named goal 仍有多处硬编码

**涉及文件**

- `src/puppy_gait/scripts/patrol_mission.py`
- `src/puppy_gait/scripts/battery_simulator.py`
- `src/puppy_nav/scripts/patrol.py`
- `src/puppy_core/puppy_core/goal_dispatcher.py`
- `src/puppy_core/config/*.yaml`

**建议做法**

- 所有命名目标统一挪到 YAML
- 充电桩坐标只保留一个来源
- `amcl_pose` 相关订阅统一按标准消息类型处理
- 巡逻 waypoint、dock、房间坐标统一通过配置注入

**验收标准**

- 低电量返航和“到达充电桩开始充电”能使用同一套坐标
- 命名目标、巡逻点、回充点在所有入口下语义一致
- 新场景切换时不需要到多份脚本里手改坐标

## 4. P1：重要但可分阶段完成的问题

### P1-1 收拢状态机，只保留一个权威任务编排层

当前至少有这些地方在决定“机器人下一步干什么”：

- `autonomous_nav.py`
- `src/puppy_nav/scripts/patrol.py`
- `src/puppy_gait/scripts/patrol_mission.py`
- `src/puppy_core/puppy_core/mission_manager.py`

**建议**

- 如果目标是产品化链路，建议以 `mission_manager + mode_manager + safety_manager` 为唯一权威控制面
- 其余脚本退化为：
  - demo
  - test client
  - research-only runtime

**验收标准**

- 正式 bringup 下只有一套任务优先级和中断逻辑
- 巡逻、回充、跌倒、安全事件都由同一个编排层裁决

### P1-2 把硬编码参数迁出脚本

当前硬编码较多的内容包括：

- waypoint
- 房间名
- dock 坐标
- 电池阈值
- 行为定时器
- 安全阈值

**建议**

- 把运行参数统一收口到：
  - `src/puppy_core/config/`
  - `src/puppypi_adapter/config/`
  - `src/puppy_nav/config/`
- 研究原型侧 `config/*.yaml` 与 ROS2 配置尽量对齐命名

**验收标准**

- 改场景时主要改配置，而不是改 Python 源码
- 仿真和真机共享尽可能多的参数名

### P1-3 把健康状态聚合做实

`robot_state_aggregator.py` 目前更像骨架，IMU、LiDAR、Camera 就绪状态没有真正接入。

**建议**

- 为关键 topic 引入超时检测和时间戳检查
- `RobotHealth` 要能真实反映：
  - 电池是否可用
  - 运动是否可控
  - IMU / LiDAR / Camera 是否在线
  - 当前是否存在 active fault

**验收标准**

- 拔掉一个关键传感器或停掉一个关键节点时，`RobotHealth` 能在限定时间内反映异常

### P1-4 把 `CapabilityRegistry` 真正接进系统

`capability_registry.py` 方向是对的，但目前没有进入主运行路径。

**建议**

- 在 bringup 或 core 初始化阶段加载 capability
- 让 mission/safety/feature 开关真实依赖 capability 状态

**验收标准**

- 关闭某能力时，上层不是直接报错，而是优雅降级

### P1-5 增加最小可回归测试集

当前大量 `check_*.py` / `check_*.sh` 更偏诊断工具，不是稳定回归测试。

**建议**

- 增加三类测试：
  1. 接口契约测试
  2. launch smoke test
  3. 关键场景行为测试
- 推荐优先覆盖：
  - 低电量回充
  - 跌倒触发安全停机
  - 手动发送导航目标
  - 巡逻失败后的重试 / 中断

**验收标准**

- 每次改接口或 launch，都能自动发现破坏性回归

## 5. P2：中长期优化方向

### P2-1 合并研究栈与 ROS2 栈的算法复用

`autonomous_nav.py` 这一套算法已经有价值，后续可以考虑拆成可复用库或 ROS2 node，而不是长期停留在单脚本运行态。

方向包括：

- 把 A* / DWA / TEB / AMCL / frontier 管理抽成可复用模块
- 与 `nav_core/` 目录下的状态机和探索模块逐步合流

### P2-2 建立统一评估流水线

当前 `eval_results/` 很有价值，但更像手工实验产物。

建议后续补：

- 固定场景基线
- 自动保存 summary / csv / plot
- 统一 mock / Gazebo / Coppelia 的对比口径

### P2-3 加强硬件安全和运维能力

后续真机阶段可以继续补：

- watchdog
- 心跳监控
- rosbag 自动录制
- 故障后安全姿态恢复
- docking 感知闭环
- QoS 和网络容错

## 6. 推荐执行顺序

建议按下面顺序推进，不建议跳着做：

### 第一批：接口收口

- P0-1
- P0-2
- P0-5

目标：把消息、坐标、动作协议统一下来。

### 第二批：补齐适配层

- P0-3
- P0-4

目标：让 mock / sim / hardware 都能沿同一条上层接口跑通。

### 第三批：收拢控制面

- P1-1
- P1-2
- P1-3
- P1-4

目标：把系统从“多个 demo 并存”收拢到一个清晰的产品化控制面。

### 第四批：回归与演进

- P1-5
- P2-1
- P2-2
- P2-3

目标：让后续改动不再靠手工排障兜底。

## 7. 完成定义

当下面这些条件都成立时，可以认为项目进入下一阶段：

1. core 层只依赖一套稳定的语义接口。
2. 官方 launch 入口都能在干净环境下启动。
3. mock、Gazebo、Coppelia、真机四条链路对上层表现一致。
4. 巡逻、导航、跌倒、低电量回充都有自动化验证。
5. 真机适配层不再以 TODO 和占位数据为主。

## 8. 下一步建议

如果继续往下做，最推荐的落地方式不是同时改很多地方，而是分两个短迭代：

1. 先做“接口统一 + launch 收口”
2. 再做“adapter 落地 + 任务编排收敛”

这样改动的风险最可控，也最容易在每一轮结束后看到系统状态实实在在变好。
