# Puppy 机械狗 SLAM 仿真项目计划

> 版本：v1.0  
> 目标：在 Gazebo 仿真环境中构建一个完整的 Puppy 四足机器人 SLAM（同步定位与地图构建）系统，实现自主定位、建图与导航。

---

## 一、项目概述

### 1.1 项目目标

- **核心目标**：基于 ROS2 + Gazebo 搭建 Puppy 四足机器人的 SLAM 仿真系统
- **功能清单**：
  1. 四足机器人 URDF / SDF 模型构建与关节控制
  2. 多种传感器仿真（2D LiDAR、RGB-D 相机、IMU、轮式/足式里程计）
  3. 多种 SLAM 算法集成（Cartographer、SLAM Toolbox、RTAB-Map）
  4. 基于 Nav2 的路径规划与自主导航
  5. 仿真环境（多场景：室内房间、走廊、迷宫）
  6. 评估指标：建图精度、定位误差、导航成功率

### 1.2 技术选型

| 类别 | 选型 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 22.04 LTS | ROS2 Humble 官方支持 |
| ROS 版本 | ROS2 Humble Hawksbill | 长期支持版，生态成熟 |
| 物理仿真 | Gazebo Fortress (Harmonic 可选) | 与 ROS2 深度集成 |
| 机器人建模 | URDF + Xacro | 参数化模型描述 |
| 关节控制 | `ros2_control` + `gazebo_ros2_control` | 标准控制接口 |
| 足式步态 | 自研 trot 步态控制器 | 基础足式运动 |
| 2D SLAM | Cartographer / slam-toolbox | 主流 2D LiDAR SLAM |
| 3D SLAM | RTAB-Map | RGB-D 视觉 SLAM |
| 导航框架 | Nav2 (Navigation2) | ROS2 标准导航栈 |
| 语言 | C++ + Python | 算法层 C++，脚本层 Python |
| 可视化 | RViz2 | 调参与可视化 |

---

## 二、系统架构设计

### 2.1 总体架构图（数据流）

```
┌─────────────────────────────────────────────────────────────────┐
│                       Gazebo 仿真环境                            │
│  ┌──────────────┐   ┌──────────────┐   ┌───────────────────┐   │
│  │  世界场景     │   │  Puppy 机器人 │   │  物理引擎(ODE)    │   │
│  │ (world模型)  │   │  URDF/SDF    │   │  碰撞/动力学       │   │
│  └──────┬───────┘   └──────┬───────┘   └─────────┬─────────┘   │
└─────────┼──────────────────┼──────────────────────┼─────────────┘
          │                  │                      │
          ▼                  ▼                      ▼
  ┌───────────────┐  ┌───────────────┐    ┌───────────────────┐
  │  激光数据     │  │  RGB-D 图像   │    │ 关节状态 / IMU    │
  │ /scan         │  │ /camera/image  │    │ /joint_states     │
  │ /range        │  │ /camera/depth  │    │ /imu/data         │
  └───────┬───────┘  └───────┬───────┘    └─────────┬─────────┘
          │                  │                      │
          ▼                  ▼                      ▼
  ┌───────────────────────────────────────────────────────────┐
  │                    SLAM 节点组                              │
  │  ┌──────────────────┐  ┌────────────┐  ┌───────────────┐   │
  │  │ Cartographer     │  │ slam-toolbox│  │ RTAB-Map      │   │
  │  │ (2D LiDAR)      │  │ (2D LiDAR) │  │ (RGB-D+IMU)    │   │
  │  └─────────┬────────┘  └─────┬──────┘  └────────┬──────┘   │
  └──────────┼──────────────────┼──────────────────┼───────────┘
             │  /map            │  /map            │  /map
             │  /tf             │  /tf             │  /tf
             └──────────────────┼──────────────────┘
                                ▼
                    ┌───────────────────────┐
                    │  Nav2 导航栈           │
                    │  - AMCL 定位(备选)     │
                    │  - 全局规划器(A*)      │
                    │  - 局部规划器(TEB/DWB) │
                    │  - 代价地图            │
                    │  - 行为树(BehaviorTree) │
                    └──────────┬────────────┘
                               │ /cmd_vel (期望速度)
                               ▼
                    ┌───────────────────────┐
                    │  步态控制器 (Trot)    │
                    │  - 逆运动学(IK)       │
                    │  - 关节轨迹生成        │
                    │  - ros2_control 接口  │
                    └──────────┬────────────┘
                               │ 关节命令
                               ▼
                    ┌───────────────────────┐
                    │  Gazebo 物理仿真 ↺     │ ← 闭环
                    └───────────────────────┘
```

### 2.2 计算图（ROS2 Nodes & Topics）

| Node | 发布 Topic | 订阅 Topic | 说明 |
|------|-----------|-----------|------|
| `gazebo` | `/scan`, `/imu/data`, `/joint_states`, `/camera/*`, `/tf_static` | `/cmd_vel`, `/joint_cmd` | 仿真与传感器发布 |
| `robot_state_publisher` | `/robot_description`, `/tf` | `/joint_states` | 发布机器人 TF 树 |
| `slam_toolbox` | `/map`, `/tf` (map→odom) | `/scan`, `/tf` (odom→base_link) | 2D SLAM 建图 |
| `cartographer_node` | `/map`, `/tf` | `/scan`, `/imu/data` | Google Cartographer |
| `rtabmap` | `/map`, `/tf`, `/octomap` | `/camera/rgb`, `/camera/depth`, `/imu` | 3D 视觉 SLAM |
| `nav2_controller_server` | `/cmd_vel` | `/tf`, `/map`, `/goal_pose` | 局部路径跟踪 |
| `nav2_planner_server` | `/plan` | `/map`, `/tf`, `/goal_pose` | 全局路径规划 |
| `gait_controller` | `/joint_trajectory` | `/cmd_vel`, `/joint_states` | 足式步态控制 |
| `teleop_twist_keyboard` | `/cmd_vel` | - | 键盘遥控 |

### 2.3 TF 坐标系树

```
map (SLAM 发布)
 └── odom (里程计或 SLAM 提供)
      └── base_link (机器人基座)
           ├── base_footprint (Nav2 必需)
           ├── imu_link
           ├── laser_link
           ├── camera_link
           │    └── camera_rgb_frame
           │    └── camera_depth_frame
           ├── leg_FR_hip
           │    └── leg_FR_thigh
           │         └── leg_FR_calf
           ├── leg_FL_hip ...
           ├── leg_RR_hip ...
           └── leg_RL_hip ...
```

---

## 三、Puppy 机器人模型设计

### 3.1 机械结构参数（参考 Unitree Go1 / A1 简化版）

| 参数 | 数值 | 说明 |
|------|------|------|
| 躯干尺寸 (长×宽×高) | 0.40m × 0.22m × 0.12m | 主体尺寸 |
| 总质量 | ~12 kg | 包含电池、传感器 |
| 腿结构 | 每条腿 3 自由度（hip_yaw / hip_pitch / knee） | 共 12 关节 |
| 大腿长度 L1 | 0.20 m | |
| 小腿长度 L2 | 0.20 m | |
| 站立高度 | ~0.28 m | |
| 关节力矩限制 | ±20 N·m | |
| 最大关节速度 | ±30 rad/s | |

### 3.2 URDF / Xacro 建模计划

**文件结构：**
```
puppy_description/
├── urdf/
│   ├── puppy.urdf.xacro          # 主文件，include 各部件
│   ├── macros.xacro              # 关节宏定义
│   ├── materials.xacro           # 颜色/材质
│   ├── torso.xacro               # 躯干 link
│   ├── leg.xacro                 # 单腿宏（被 4 条腿实例化）
│   ├── sensors.xacro             # 相机 / 激光 / IMU 插件
│   └── gazebo_control.xacro      # gazebo_ros2_control 配置
├── meshes/
│   ├── torso.stl                 # (可选) 美观模型
│   └── leg.stl
└── config/
    └── joint_names.yaml
```

**关键实现细节：**
- 躯干使用 `box` collision，`mesh` visual（可选）
- 4 条腿分别命名：`FR` (前右), `FL` (前左), `RR` (后右), `RL` (后左)
- 每条腿 3 个 revolute joint：`hip_yaw` → `hip_pitch` → `knee`
- 使用 `xacro:macro name="leg" params="prefix side"` 实例化
- 坐标系约定：`x` 朝前，`y` 朝左，`z` 朝上（REP-103）

### 3.3 传感器仿真配置

**1) 2D 激光雷达（LiDAR）— 用于 2D SLAM**

```xml
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_sensor">
    <ray>
      <scan>
        <horizontal><samples>720</samples><min_angle>-3.14</min_angle><max_angle>3.14</max_angle></horizontal>
      </scan>
      <range><min>0.1</min><max>10.0</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0.0</mean><stddev>0.02</stddev></noise>
    </ray>
    <plugin name="gazebo_ros_ray" filename="libgazebo_ros_ray_sensor.so">
      <ros><argument>~/out:=scan</argument></ros>
      <frame_name>laser_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

- 10m 量程，360° 扫描，0.02m 高斯噪声
- 发布话题：`/scan` (sensor_msgs/LaserScan)

**2) RGB-D 深度相机 — 用于 3D SLAM / RTAB-Map**

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="kinect">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>640</width><height>480</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>5.0</far></clip>
    </camera>
    <plugin filename="libgazebo_ros_camera.so" name="camera_plugin">
      <ros><argument>~/image:=/camera/rgb/image_raw</argument></ros>
      <camera_name>kinect</camera_name>
    </plugin>
  </sensor>
</gazebo>
```

- 640×480 分辨率，FOV 60°，5m 深度范围

**3) IMU — 用于 Cartographer / RTAB-Map 辅助**

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <imu><angular_velocity><x><noise><type>gaussian</type><stddev>0.001</stddev></noise></x>...</imu>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros><argument>~/out:=imu/data</argument></ros>
    </plugin>
  </sensor>
</gazebo>
```

### 3.4 关节控制器（ros2_control）

**文件：** `puppy_hardware/config/puppy_controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - FR_hip_yaw_joint
      - FR_hip_pitch_joint
      - FR_knee_joint
      - FL_hip_yaw_joint
      - FL_hip_pitch_joint
      - FL_knee_joint
      - RR_hip_yaw_joint
      - RR_hip_pitch_joint
      - RR_knee_joint
      - RL_hip_yaw_joint
      - RL_hip_pitch_joint
      - RL_knee_joint
    interfaces: ["position"]
    state_interfaces: ["position", "velocity"]
    command_interfaces: ["position"]
```

---

## 四、足式运动控制（步态控制器）

### 4.1 步态选择：对角小跑（Trot Gait）

**为什么选 Trot？**
- 最常见、最稳定的四足运动步态
- 对角腿（FR+RL / FL+RR）同步摆动
- 相移 50%（半步）
- 控制相对简单，适合初期实现

**Trot 步态核心参数：**

| 参数 | 符号 | 默认值 | 可调范围 |
|------|------|--------|---------|
| 步态周期 | T | 0.5 s | 0.3 – 0.8 |
| 占空比 | D | 0.5 | 0.4 – 0.6 |
| 步长 | S | 0.08 m | 0 – 0.15 |
| 抬腿高度 | H | 0.05 m | 0.02 – 0.10 |
| 身体高度 | h_body | 0.28 m | 0.25 – 0.30 |

### 4.2 步态控制器节点设计

**节点名：** `gait_controller`（C++ 实现，package: `puppy_gait`）

**输入输出：**

| 类型 | Topic | Message | 说明 |
|------|-------|---------|------|
| Sub | `/cmd_vel` | `geometry_msgs/Twist` | 期望线速度 (vx, vy)、角速度 (wz) |
| Sub | `/joint_states` | `sensor_msgs/JointState` | 当前关节角度 |
| Pub | `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 目标关节轨迹 |
| 或直接发布到 | `/joint_trajectory_controller/joint_trajectory` | | |

**内部状态机：**

```
STAND  ←───┐
  │        │
  ▼        │
TROT  ─────┘
  │
  ▼
TURN (集成在 TROT 中处理 wz)
```

### 4.3 逆运动学（IK）推导

**每条腿的 2-DoF IK（hip_yaw 单独处理）：**

在矢状面（sagittal plane）内，已知足端目标位置 (x, z)（相对于 hip 关节）：

```
r = sqrt(x² + z²)
cos(θ2) = (L1² + L2² - r²) / (2·L1·L2)
θ2 = acos(cos(θ2)) - π       （膝盖角，负值表示前屈）
θ1 = atan2(x, -z) - atan2(L2·sin(θ2), L1 + L2·cos(θ2))   （髋关节 pitch）
```

> 关键细节：**θ2 必须取负值**（膝盖前屈），否则腿会反折。

### 4.4 足端轨迹生成（摆动相 / 支撑相）

**摆动相（Swing, t ∈ [0, T·(1-D)]）：**

```
x(t) = S/2 - S·(t / T_swing)            （线性后退 → 前移）
z(t) = -h_body + H·sin(π·t / T_swing)   （正弦抬足）
```

**支撑相（Stance, t ∈ [0, T·D]）：**

```
x(t) = -S/2 + S·(t / T_stance)          （匀速后推，推动身体前进）
z(t) = -h_body                           （保持接地高度）
```

**相移分配（Trot）：**
- FR, RL 腿：相位 0
- FL, RR 腿：相位 T/2

### 4.5 转向与侧向运动处理

**原地旋转（cmd_vel.angular.z）：**
- 左右腿产生相反的步长：`S_left = S0 + K_rot · wz`，`S_right = S0 - K_rot · wz`
- 同时足端产生侧向位移（hip_yaw 关节微调）

**侧向移动（cmd_vel.linear.y）：**
- 通过 hip_yaw 关节产生左右对称的侧向足端轨迹

---

## 五、SLAM 方案设计

### 5.1 方案一：Cartographer（Google 出品，推荐首选）

**适用场景：** 2D LiDAR + IMU，室内环境

**关键文件：** `puppy_slam/config/cartographer_2d.lua`

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
```

配套 launch 文件：`puppy_slam/launch/cartographer.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('puppy_slam')
    lua = os.path.join(pkg, 'config', 'cartographer_2d.lua')
    return LaunchDescription([
        Node(package='cartographer_ros', executable='cartographer_node',
             arguments=['-configuration_directory', os.path.dirname(lua),
                        '-configuration_basename', 'cartographer_2d.lua'],
             remappings=[('scan', '/scan'), ('imu', '/imu/data')]),
        Node(package='cartographer_ros', executable='occupancy_grid_node',
             parameters=[{'resolution': 0.05}, {'publish_period_sec': 1.0}]),
    ])
```

### 5.2 方案二：slam-toolbox（Nav2 官方推荐 2D SLAM）

**适用场景：** 2D LiDAR + 里程计，轻量级

**关键文件：** `puppy_slam/config/mapper_params_online_async.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    plugin: slam_toolbox/AsyncSlamToolbox
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping
    resolution: 0.05
    max_laser_range: 10.0
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.5
    do_loop_closing: true
    loop_match_min_measurements: 10
    transform_publish_period: 0.02
    map_update_interval: 5.0
```

### 5.3 方案三：RTAB-Map（RGB-D 视觉 SLAM）

**适用场景：** RGB-D 相机，需要 3D 地图

**关键参数：**
```yaml
rtabmap:
  ros__parameters:
    frame_id: base_link
    subscribe_depth: true
    subscribe_rgb: true
    subscribe_imu: true
    rgbd_cameras: 1
    odom_frame_id: odom
    Grid/FromDepth: true
    Grid/Size: 0.05
    Grid/RayTracing: true
    Grid/3D: false
```

### 5.4 里程计来源（足式里程计）

由于足式机器人没有真正的轮式里程计，需要自己发布 odom→base_link 的 TF：

**推荐方案：robot_localization (EKF) 融合**

```yaml
ekf_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /leg_odom
    odom0_config: [true, true, false, false, false, true,
                   true, false, false, false, false, false, false, false, false]
    imu0: /imu/data
    imu0_config: [false, false, false, false, false, true,
                  false, false, false, false, false, true, true, false, false]
```

---

## 六、Nav2 导航方案

### 6.1 Nav2 核心配置文件

**文件：** `puppy_nav/config/nav2_params.yaml`（节选）

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: base_footprint
    odom_frame_id: odom
    global_frame_id: map
    laser_model_type: likelihood_field
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    update_min_d: 0.1
    update_min_a: 0.2
    transform_tolerance: 0.5

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.3
      min_vel_x: 0.0
      max_vel_theta: 1.0
      acc_lim_x: 0.5
      acc_lim_theta: 1.0

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["voxel_layer", "inflation_layer"]

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ["navigate_to_pose"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator/navigate_to_pose"
      bt_xml_filename: "navigate_to_pose_w_replanning_and_recovery.xml"
```

### 6.2 启动文件

`puppy_nav/launch/navigation.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('puppy_nav')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg, 'maps', 'office.yaml')
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server',
             parameters=[{'yaml_filename': map_file}, {'use_sim_time': True}]),
        Node(package='nav2_amcl', executable='amcl', parameters=[params]),
        Node(package='nav2_controller', executable='controller_server', parameters=[params]),
        Node(package='nav2_planner', executable='planner_server', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', parameters=[params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             parameters=[{'use_sim_time': True}, {'autostart': True},
                         {'node_names': ['map_server', 'amcl', 'controller_server',
                                         'planner_server', 'behavior_server',
                                         'bt_navigator']}]),
    ])
```

---

## 七、仿真世界设计

### 7.1 三种场景

| 场景 | 尺寸 | 复杂度 | 用途 |
|------|------|--------|------|
| 小房间 | 5m × 5m | 简单，无障碍物 | 调试步态、基础建图 |
| 办公室 | 20m × 15m | 中等，含家具墙柱 | 日常 SLAM + 导航 |
| 迷宫 | 30m × 30m | 高，复杂墙、回环 | 回环检测压力测试 |

### 7.2 Gazebo world 文件模板

`puppy_worlds/worlds/office.world`（节选）

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="office">
    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background></scene>

    <!-- 地面 -->
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>
      </link></model>

    <!-- 墙体 -->
    <model name="wall_1"><static>true</static><pose>-10 0 1.25 0 0 0</pose>
      <link name="link"><collision><geometry><box><size>0.2 20 2.5</size></box></geometry></collision>
        <visual><geometry><box><size>0.2 20 2.5</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual></link></model>

    <!-- 机器人 -->
    <include><uri>model://puppy</uri><pose>0 0 0.3 0 0 0</pose></include>
  </world>
</sdf>
```

---

## 八、完整文件目录结构

```
puppyfangzhen/                         # 工作空间根目录
├── jihua.md                            # 本计划书
│
├── src/
│   ├── puppy_description/              # 机器人 URDF 模型
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── urdf/ (puppy.urdf.xacro, macros, torso, leg, sensors, gazebo_control)
│   │   ├── meshes/ (.stl)
│   │   ├── config/joint_names.yaml
│   │   └── launch/display.launch.py
│   │
│   ├── puppy_hardware/                 # ros2_control 硬件接口
│   │   ├── package.xml / CMakeLists.txt
│   │   └── config/puppy_controllers.yaml
│   │
│   ├── puppy_gait/                     # 足式步态控制器 (核心)
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── include/puppy_gait/ (gait_controller.hpp, trot_gait.hpp, inverse_kinematics.hpp, state_machine.hpp)
│   │   ├── src/ (.cpp 实现)
│   │   ├── config/gait_params.yaml
│   │   └── launch/gait_controller.launch.py
│   │
│   ├── puppy_localization/             # 里程计 + 融合
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── src/leg_odometry.cpp
│   │   ├── config/ekf.yaml
│   │   └── launch/localization.launch.py
│   │
│   ├── puppy_slam/                     # SLAM 配置 (三种方案)
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── config/ (cartographer_2d.lua, mapper_params_online_async.yaml, rtabmap.yaml)
│   │   ├── maps/ (.pgm, .yaml)
│   │   └── launch/ (cartographer, slam_toolbox, rtabmap).launch.py
│   │
│   ├── puppy_nav/                      # Nav2 导航
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── config/nav2_params.yaml
│   │   ├── maps/
│   │   └── launch/navigation.launch.py
│   │
│   ├── puppy_worlds/                   # Gazebo 仿真世界
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── worlds/ (small_room, office, maze).world
│   │   ├── models/puppy/ (model.config, model.sdf)
│   │   └── launch/simulation.launch.py
│   │
│   └── puppy_bringup/                  # 一键启动
│       ├── package.xml / CMakeLists.txt
│       ├── launch/ (mapping.launch.py, navigation.launch.py)
│       └── rviz/ (mapping.rviz, navigation.rviz)
│
├── scripts/
│   ├── install_deps.sh
│   └── evaluate_slam.py
│
├── docs/
│   ├── BUILDING.md
│   ├── RUNNING.md
│   └── TROUBLESHOOTING.md
│
└── colcon.meta
```

---

## 九、实现步骤（按阶段拆解任务）

### 阶段 0：环境搭建（预计 1 天）

**任务 0.1** 安装基础环境
- 安装 Ubuntu 22.04（物理机 / WSL2 / 虚拟机）
- 安装 ROS2 Humble：`sudo apt install ros-humble-desktop`
- 安装 Gazebo Fortress：`sudo apt install ros-humble-gazebo-ros-pkgs`
- 安装 ros2_control：`sudo apt install ros-humble-ros2-control ros-humble-gazebo-ros2-control`
- 安装 Cartographer：`sudo apt install ros-humble-cartographer ros-humble-cartographer-ros`
- 安装 slam-toolbox：`sudo apt install ros-humble-slam-toolbox`
- 安装 RTAB-Map：`sudo apt install ros-humble-rtabmap ros-humble-rtabmap-ros`
- 安装 Nav2：`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- 安装 robot_localization：`sudo apt install ros-humble-robot-localization`
- 创建 colcon 工作空间

**任务 0.2** 验证 ROS2 / Gazebo 工作正常

### 阶段 1：机器人建模（预计 2-3 天）

**任务 1.1** 创建 `puppy_description` 包
- 编写各 xacro 部件（躯干、腿、传感器、gazebo_control）
- 主文件 include 所有部件

**任务 1.2** RViz2 可视化验证
- `display.launch.py` + `joint_state_publisher_gui`

**任务 1.3** 生成 Gazebo SDF model
- `xacro → gz sdf -p → model.sdf`

### 阶段 2：关节控制 + 步态（预计 3-5 天）

**任务 2.1** 配置 puppy_hardware 控制器
- 编写 `puppy_controllers.yaml`
- 验证：`ros2 control list_hardware_components`

**任务 2.2** 实现逆运动学 `inverse_kinematics.cpp`
- 函数：`solveIK(x, y, z) → (yaw, pitch, knee)`
- 单元测试（FK 反推验证）

**任务 2.3** 实现 Trot 步态 `trot_gait.cpp`
- 类 `TrotGait`：参数 period, duty_cycle, step_length, lift_height
- `getFootPosition(leg_id, t)` → 足端 (x, y, z)
- 相移：FR/RL=0，FL/RR=T/2

**任务 2.4** 实现步态控制器节点 `gait_controller.cpp`
- 订阅 `/cmd_vel` → 内部时间推进
- 4 条腿足端位置 → IK → 关节角度 → 发布 JointTrajectory

**验证：** 站立稳定 → 直线走 → 旋转 → 1 分钟不摔倒

### 阶段 3：里程计（预计 1 天）

**任务 3.1** `leg_odometry.cpp` 发布 odom TF
- 积分 cmd_vel（基础版），或融合 IMU（进阶版）
- 发布 `/odom` 和 `odom → base_link` TF

**任务 3.2** 配置 robot_localization EKF

### 阶段 4：SLAM 建图（预计 2-3 天）

**任务 4.1** Cartographer 集成 + 建图 + 保存地图
**任务 4.2** slam-toolbox 集成 + 对比地图质量
**任务 4.3** RTAB-Map 3D 建图（可选）

### 阶段 5：Nav2 导航（预计 2-3 天）

**任务 5.1** 编写 `nav2_params.yaml`（确保 base_footprint TF 存在）
**任务 5.2** 编写 `navigation.launch.py`（map_server + AMCL + planner + controller + bt_navigator + lifecycle_manager）
**任务 5.3** RViz2 中点击 Nav Goal 验证 → 避障测试

### 阶段 6：自动化评估（可选，预计 2 天）

**任务 6.1** 编写 `evaluate_slam.py`，对比 SLAM 轨迹 vs Gazebo 真值，输出 ATE/RPE
**任务 6.2** 多场景多参数批量测试 → 生成对比报告

---

## 十、关键 launch 文件（总览）

### 一键建图

```bash
ros2 launch puppy_bringup mapping.launch.py world:=office
# 启动内容：Gazebo + spawn robot + controller_manager + gait_controller
#           + Cartographer (或 slam_toolbox) + teleop + RViz2
```

### 一键导航

```bash
ros2 launch puppy_bringup navigation.launch.py map:=office
# 启动内容：Gazebo + spawn robot + gait_controller
#           + map_server + AMCL + Nav2 planner/controller + RViz2
```

---

## 十一、关键难点与应对策略

| 难点 | 风险 | 应对策略 |
|------|------|---------|
| 足式机器人仿真不稳定（摔倒、抖动） | 高 | ① PID 关节控制；② 加入关节阻尼；③ 先站立再逐步加速；④ 足端加 contact sensor |
| 足式里程计误差大 | 中 | ① 融合 IMU（robot_localization）；② 依赖 SLAM 的 map→odom 修正 |
| Gazebo 传感器 plugin 异常 | 中 | ① `ros2 topic hz /scan` 检查；② `use_sim_time: true`；③ 版本匹配 |
| Nav2 TF 不完整 | 中 | ① `view_frames` 检查；② 确保 base_footprint 存在 |
| SLAM 回环闭合差 | 中 | ① 调整 loop_match 参数；② 确保激光频率；③ 路径设计包含多次回环 |

---

## 十二、里程碑与时间规划

| 里程碑 | 内容 | 预计时间 | 验收标准 |
|--------|------|---------|---------|
| M1 | 环境搭建 + 基础包跑通 | 1 天 | turtlesim + gazebo 正常 |
| M2 | URDF 模型在 RViz 中显示 | 2 天 | 模型完整，关节可手动运动 |
| M3 | Gazebo 中机器人稳定站立 | 1 天 | 不掉落，不剧烈抖动 |
| M4 | 遥控走路 + 转向 | 2 天 | 键盘控制机器人自主移动 |
| M5 | 里程计发布 odom TF | 1 天 | 方形轨迹误差 < 5% |
| M6 | Cartographer 建图成功 | 2 天 | 地图闭合、保存成功 |
| M7 | slam-toolbox 对比测试 | 1 天 | 两套地图均可加载 |
| M8 | Nav2 自主导航 | 2 天 | RViz 点击可达目标点 |
| M9 | 多场景测试 + 调优 | 2 天 | 3 种地图均能建图 + 导航 |
| M10 | 评估报告 / 文档 | 1 天 | ATE/RPE 数据 + 文档齐全 |
| **合计** | | **~15 工作日** | |

---

## 十三、下一步行动建议

1. **优先执行**：阶段 0（环境）→ 阶段 1（建模），先有模型才能谈 SLAM
2. **并行思路**：`puppy_gait` 可以与 `puppy_description` 并行开发，前者不依赖 Gazebo
3. **先跑最小闭环**：先用"虚拟差速轮"替代足式步态（即直接把 Puppy 当差速机器人走），先跑通 SLAM + Nav2 全流程；再把差速替换成真正的步态控制器
4. **调试工具清单**：`ros2 topic echo`、`ros2 topic hz`、`ros2 run tf2_tools view_frames`、`rviz2` TF 可视化

---

## 附录 A：常用命令速查

```bash
# 编译
cd ~/puppy_ws
colcon build --symlink-install --packages-select puppy_description puppy_gait puppy_slam puppy_nav puppy_bringup
source install/setup.bash

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看话题频率
ros2 topic hz /scan
ros2 topic hz /imu/data

# 手动发速度命令测试
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 或
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# 保存地图 (slam-toolbox)
ros2 run nav2_map_server map_saver_cli -f ~/puppy_ws/maps/office

# 保存地图 (Cartographer)
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'map.pbstream'}"
```

---

*本文档持续更新，每个模块实现后应补充具体的代码片段与调试记录。*
