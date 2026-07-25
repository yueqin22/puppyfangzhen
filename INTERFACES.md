# 接口文档 (INTERFACES.md)

本文档列出项目中所有 ROS2 topic / service / action / 参数接口，对应 gaijin2.md 第15.1节要求的交付文档。

---

## 1. 话题 (Topics)

### 1.1 运动控制

| 话题 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|------|------|--------|--------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop / mission_manager | safety_manager | 按需 | 通用速度意图 |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | safety_manager | motion_adapter | 按需 | 安全限幅后的速度 |
| `/platform/cmd` | `geometry_msgs/Twist` | motion_adapter | PuppyPi SDK | 30Hz | 平台最终执行命令 |

### 1.2 传感器

| 话题 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|------|------|--------|--------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | lidar_driver / fake_sensors | localization / nav | 20Hz | 2D激光扫描 |
| `/imu/data` | `sensor_msgs/Imu` | imu_driver / fake_sensors | localization / ekf | 100Hz | IMU数据 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | camera_driver / fake_sensors | vision nodes | 30Hz | RGB图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | camera_driver | vision nodes | 30Hz | 深度图像 |
| `/joint_states` | `sensor_msgs/JointState` | status_adapter / fake_status | tf / rviz | 20Hz | 关节状态 |

### 1.3 机器人状态

| 话题 | 类型 | 发布者 | 订阅者 | 频率 | 说明 |
|------|------|--------|--------|------|------|
| `/robot/mode` | `puppy_interfaces/RobotMode` | mode_manager | 全局 | 5Hz | 当前运行模式 |
| `/robot/health` | `puppy_interfaces/RobotHealth` | state_aggregator | 全局 | 2Hz | 健康状态聚合 |
| `/platform/motion_state` | `puppy_interfaces/PlatformMotionState` | motion_adapter | core / safety | 20Hz | 平台运动状态 |
| `/platform/health` | `puppy_interfaces/RobotHealth` | status_adapter | state_aggregator | 20Hz | 平台原始健康 |
| `/battery_state` | `sensor_msgs/BatteryState` | status_adapter / fake_status | safety / core | 20Hz | 电池状态 |

### 1.4 事件与任务

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/security/event` | `puppy_interfaces/SecurityEvent` | security_node | mission_manager | 安防事件 |
| `/fall/event` | `puppy_interfaces/FallEvent` | fall_detection | mode_manager / safety | 跌倒事件 |
| `/dock/status` | `puppy_interfaces/DockStatus` | dock_adapter | mission_manager | 充电桩状态 |
| `/mission/status` | `puppy_interfaces/PatrolStatus` | mission_manager | 全局 | 任务进度 |
| `/mission/command` | `std_msgs/String` | 外部 | mission_manager | 任务命令 |

### 1.5 控制命令

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/robot/mode_request` | `std_msgs/String` | 外部 | mode_manager | 模式切换请求 |
| `/robot/posture_cmd` | `std_msgs/String` | 外部 | mode_adapter | 姿态命令(STAND/SIT/...) |
| `/robot/motors_enable_request` | `std_msgs/Bool` | 外部 | safety_manager | 电机使能请求 |
| `/robot/motion_enable` | `std_msgs/Bool` | safety_manager | motion_adapter | 运动使能 |

### 1.6 导航目标

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/goal/pose` | `geometry_msgs/PoseStamped` | 外部 | goal_dispatcher | 直接位姿目标 |
| `/goal/named` | `std_msgs/String` | 外部 | goal_dispatcher | 命名目标(dock/bedroom/...) |

---

## 2. 服务 (Services)

| 服务 | 类型 | 服务端 | 服务客户端 | 说明 |
|------|------|--------|-----------|------|
| `/robot/set_mode` | `puppy_interfaces/SetRobotMode` | mode_manager | 全局 | 设置运行模式 |
| `/robot/clear_fault` | `puppy_interfaces/ClearFault` | mode_manager | 全局 | 清除故障 |
| `/robot/enable_motors` | `puppy_interfaces/EnableMotors` | safety_manager | 全局 | 使能/失能电机 |
| `/mapping/start` | `puppy_interfaces/StartMapping` | slam_manager | mission_manager | 开始建图 |
| `/mapping/stop` | `puppy_interfaces/StopMapping` | slam_manager | mission_manager | 停止建图 |

---

## 3. 动作 (Actions)

| Action | Goal | Result | Feedback | 服务端 | 说明 |
|--------|------|--------|----------|--------|------|
| `navigate_to_pose` | PoseStamped, timeout | success, final_pose, distance | distance, time, state | Nav2 / nav_core | 点到点导航 |
| `patrol_route` | waypoints[], loop, pause | success, reached_wp | current_idx, distance, state | patrol_server | 路线巡逻 |
| `dock` | dock_id, force | success, charging | stage, distance, visible | dock_adapter | 自动回充 |
| `stand_up` | - | success, message | stage | mode_adapter | 站立 |
| `sit_down` | - | success, message | stage | mode_adapter | 坐下 |
| `recover_posture` | direction | success, message | stage | mode_adapter | 姿态恢复 |

---

## 4. 参数 (Parameters)

### 4.1 运动限制 (safety_manager)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `cmd_vel_timeout` | float | 1.0 | 命令超时秒数 |
| `max_linear_x` | float | 0.3 | 最大线速度 m/s |
| `max_angular_z` | float | 1.2 | 最大角速度 rad/s |
| `motors_enabled_on_start` | bool | false | 启动时电机默认状态 |

### 4.2 电池阈值 (safety_manager)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `low_battery_threshold` | float | 0.20 | 低电量阈值 |
| `critical_battery_threshold` | float | 0.10 | 危险电量阈值 |

### 4.3 平台适配 (motion_adapter)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_linear_x` | float | 0.3 | 平台最大线速度 |
| `max_angular_z` | float | 1.2 | 平台最大角速度 |
| `accel_limit` | float | 2.0 | 加速度限制 |
| `yaw_rate_limit` | float | 4.0 | 角加速度限制 |
| `cmd_timeout` | float | 1.0 | 命令超时 |
| `dead_zone_linear` | float | 0.01 | 线速度死区 |
| `dead_zone_angular` | float | 0.05 | 角速度死区 |

---

## 5. TF 树

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── imu_link
                ├── laser_link
                ├── camera_link
                │    └── camera_color_optical_frame
                └── leg_{fl,fr,rl,rr}_upper
                     └── leg_{fl,fr,rl,rr}_lower
```

---

## 6. 命名目标 (Named Goals)

goal_dispatcher 支持的命名目标（可在 config 中扩展）：

| 名称 | 坐标 | 说明 |
|------|------|------|
| `dock` | (0.0, 0.0) | 充电桩 |
| `bedroom` | (-3.5, 3.0) | 卧室 |
| `kitchen` | (2.0, 2.5) | 厨房 |
| `living_room` | (0.0, -2.0) | 客厅 |

---

## 7. 配置文件

| 文件 | 说明 |
|------|------|
| `config/sim.yaml` | 仿真环境配置 |
| `config/mapping.yaml` | 建图参数 |
| `config/planner_global.yaml` | A*全局规划参数 |
| `config/planner_local.yaml` | DWA局部规划参数 |
| `config/exploration.yaml` | 探索策略参数 |
| `config/runtime.yaml` | 运行时参数 |
| `src/puppy_core/config/modes.yaml` | 机器人模式定义 |
| `src/puppy_core/config/safety.yaml` | 安全阈值 |
| `src/puppy_core/config/features.yaml` | 功能开关 |
| `src/puppypi_adapter/config/adapter_params.yaml` | 平台适配参数 |
