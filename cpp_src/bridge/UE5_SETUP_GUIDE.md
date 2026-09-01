# UE5 仿真环境搭建指南 (PuppyNav)

本指南说明如何从零搭建 UE5 仿真环境, 与 `nav_ue_bridge.exe` 通过 TCP 协议进行
30Hz lockstep 联合仿真。完成本指南后, 你将获得一个可运行的 UE5 场景, 包含
机器人 Pawn、360 度 LiDAR、动态行人, 以及 AMCL/A* 调试可视化。

> **前置条件**
> - Unreal Engine 5.3 (通过 Epic Games Launcher 安装)
> - Visual Studio 2022 (含 "使用 C++ 的游戏开发" 工作负载)
> - Windows 10/11 x64
> - 已编译 `nav_ue_bridge.exe` (运行 `build_bridge.bat`)

---

## 目录

1. [创建 UE5 工程](#1-创建-ue5-工程)
2. [导入 PuppyNav 模块](#2-导入-puppynav-模块)
3. [场景生成](#3-场景生成)
4. [蓝图配置](#4-蓝图配置)
5. [行人配置](#5-行人配置)
6. [运行仿真](#6-运行仿真)
7. [调试可视化](#7-调试可视化)
8. [常见问题](#8-常见问题)

---

## 1. 创建 UE5 工程

1. 打开 **Epic Games Launcher** -> **Unreal Engine** -> **库** -> 确认已安装
   **5.3.x** 版本。
2. 启动 UE5, 选择 **Games** 类别, 模板选 **Blank** (空白)。
3. 工程设置:
   - **Project Location**: `D:\puppy_ue`
   - **Project Name**: `puppy_ue`
   - **Project Type**: 勾选 **C++** (必须, 否则无法编译模块)
   - **Target Platform**: Desktop
   - **Quality Preset**: Maximum
   - **Starter Content**: 取消勾选 (不需要)
4. 点击 **Create** 创建工程。UE 会自动生成 Visual Studio 解决方案并打开编辑器。
5. 关闭 UE 编辑器 (后续导入模块后需重新生成工程)。

工程目录结构应如下:

```
D:\puppy_ue\
  puppy_ue.uproject
  Source\
    puppy_ue\
      puppy_ue.Build.cs
      puppy_ue.cpp
      puppy_ue.h
      puppy_ueGameModeBase.h
      puppy_ueGameModeBase.cpp
```

---

## 2. 导入 PuppyNav 模块

### 2.1 复制源码文件

将 `ue_modules/` 下的全部 9 个文件复制到工程的 `Source/puppy_ue/` 目录:

```
源 (e:\puppyfangzhen\cpp_src\bridge\ue_modules\)
  PuppyNav.Build.cs
  PuppyRobotPawn.h        PuppyRobotPawn.cpp
  PuppyTcpServer.h        PuppyTcpServer.cpp
  PuppyLiDARComponent.h   PuppyLiDARComponent.cpp
  PuppyPedestrianActor.h  PuppyPedestrianActor.cpp

目标 (D:\puppy_ue\Source\puppy_ue\)
```

用 PowerShell 复制 (在工程目录下执行):

```powershell
$src = "e:\puppyfangzhen\cpp_src\bridge\ue_modules"
$dst = "D:\puppy_ue\Source\puppy_ue"
Copy-Item "$src\*" -Destination $dst -Recurse -Force
```

复制后目录结构:

```
D:\puppy_ue\Source\puppy_ue\
  puppy_ue.Build.cs          (原有)
  PuppyNav.Build.cs          (新增 -- 模块构建规则)
  PuppyRobotPawn.h/.cpp      (新增)
  PuppyTcpServer.h/.cpp      (新增)
  PuppyLiDARComponent.h/.cpp (新增)
  PuppyPedestrianActor.h/.cpp(新增)
```

### 2.2 修改 .uproject 添加模块声明

用文本编辑器打开 `D:\puppy_ue\puppy_ue.uproject`, 在 `"Modules"` 数组中添加
PuppyNav 模块。修改后内容如下:

```json
{
  "FileVersion": 3,
  "EngineAssociation": "5.3",
  "Category": "",
  "Description": "",
  "Modules": [
    {
      "Name": "puppy_ue",
      "Type": "Runtime",
      "LoadingPhase": "Default",
      "AdditionalDependencies": [
        "Engine"
      ]
    },
    {
      "Name": "PuppyNav",
      "Type": "Runtime",
      "LoadingPhase": "Default"
    }
  ]
}
```

> **注意**: `"EngineAssociation"` 应为你安装的 UE5.3 版本号, 如 `"5.3"` 或
> `"5.3.2"` 等, 保持原值即可。

### 2.3 模块依赖说明

`PuppyNav.Build.cs` 声明了以下模块依赖:

| 依赖模块 | 用途 |
|---------|------|
| Core / CoreUObject / Engine | UE 核心基础 |
| InputCore | 输入系统 |
| NavigationSystem | 导航网格 (预留) |
| AIModule | AI 导航 (预留) |
| Sockets | TCP socket 通信 |
| Networking | 网络层支持 |

这些模块均为 UE5 内置模块, 无需额外安装。

### 2.4 重新生成工程文件并编译

1. 右键 `puppy_ue.uproject` -> **Generate Visual Studio project files**。
2. 双击 `puppy_ue.sln` 打开 Visual Studio 2022。
3. 配置选 **Development Editor** | **Win64**。
4. 菜单 **生成** -> **生成解决方案** (Ctrl+Shift+B)。
5. 编译成功后, 双击 `puppy_ue.uproject` 打开 UE 编辑器。

> **验证**: 在 UE 编辑器中, 打开 **Content Browser** -> 点击 **Settings**
> (齿轮图标) -> 勾选 **Show C++ Classes**。应能看到 `PuppyNav` 分组下有
> `PuppyRobotPawn`、`PuppyPedestrianActor` 两个 C++ 类。

---

## 3. 场景生成

场景由 `ue_scene_builder.py` 从 `scene_home.json` 自动生成, 包含地板、
障碍物 (墙体)、巡逻点标记、行人 Actor、光照和 PlayerStart。

### 3.1 启用 Python 脚本插件

1. UE 编辑器菜单 **Edit** -> **Plugins**。
2. 搜索 **Python Editor Script Plugin**, 勾选 **Enabled**。
3. 搜索 **Editor Scripting Utilities**, 确认已启用 (通常默认开启)。
4. 重启 UE 编辑器使插件生效。

### 3.2 准备场景 JSON 文件

确保 `scene_home.json` 文件存在。该文件定义了场景的网格、障碍物、行人、
巡逻点等。如果项目中已有此文件, 记录其完整路径, 例如:

```
e:\puppyfangzhen\config\scene_home.json
```

JSON 文件的关键结构:

```json
{
  "grid": { "width": 100, "height": 80, "resolution": 0.1 },
  "obstacles": [
    { "name": "wall_north", "xmin": -5.0, "ymin": 3.8, "xmax": 5.0, "ymax": 4.0 }
  ],
  "patrol_targets": [
    { "name": "room_a", "x": 3.0, "y": 2.0 }
  ],
  "pedestrians": [
    { "name": "ped_0", "x": 1.0, "y": 0.5, "vx": 0.03, "vy": 0.02, "radius": 0.3 }
  ],
  "robot": {
    "initial_pose": { "x": -1.0, "y": -3.0, "yaw": 0.0 }
  }
}
```

### 3.3 运行场景生成脚本

1. UE 编辑器中, 创建一个新空关卡: **File** -> **New Level** ->
   选择 **Empty Level** -> 保存为 `HomeMap`。
2. 打开 **Output Log** 面板 (菜单 **Window** -> **Developer Tools** ->
   **Output Log**)。
3. 在 Output Log 左下角下拉框切换到 **Python** 模式。
4. 执行以下命令 (路径根据实际情况修改):

```python
py e:/puppyfangzhen/cpp_src/bridge/ue_scene_builder.py --json e:/puppyfangzhen/config/scene_home.json
```

脚本会依次完成:
- 创建地板 (10m x 8m, 厚度 10cm)
- 为每个 obstacle 创建 BSP Brush 立方体 (高 2.5m)
- 为每个 patrol_target 创建标记点 Actor (球体组件)
- 为每个 pedestrian 创建 `PuppyPedestrianActor` (含速度和半径)
- 创建 DirectionalLight + SkyLight
- 创建 PlayerStart (位置来自 robot.initial_pose)
- 保存当前关卡

> **坐标映射**: JSON 坐标单位为米, UE 单位为厘米。脚本自动乘以 100 转换。
> 坐标系: JSON 的 (x, y) 映射到 UE 的 (x, y, z=0), 不翻转。

5. 检查 Output Log 中的输出, 确认无错误:

```
[SceneBuilder] 地板: 10.0m x 8.0m
[SceneBuilder] 障碍物: N/N
[SceneBuilder] 巡逻点: N/N
[SceneBuilder] 行人: N/N
[SceneBuilder] 光照+PlayerStart 完成
[SceneBuilder] 场景构建完成! 关卡已保存。
```

6. 将此关卡设为默认关卡: **Edit** -> **Project Settings** ->
   **Maps & Modes** -> **Game Default Map** 选 `HomeMap`。

---

## 4. 蓝图配置

### 4.1 创建 BP_PuppyRobotPawn 蓝图

1. **Content Browser** 中, 右键 -> **Blueprint Class**。
2. 搜索并选择 **PuppyRobotPawn** (C++ 类, 在 PuppyNav 分组下)。
3. 命名为 `BP_PuppyRobotPawn`, 双击打开蓝图编辑器。

### 4.2 确认子组件

`APuppyRobotPawn` 的 C++ 构造函数已自动创建以下子组件, 蓝图中无需手动添加:

| 组件 | 类型 | 说明 |
|------|------|------|
| CollisionCapsule | UCapsuleComponent | 根组件, 半径 30cm, 用于碰撞检测 |
| LiDARComp | UPuppyLiDARComponent | 360 度 LiDAR (72 射线, 量程 8m) |
| TcpComp | UPuppyTcpServer | TCP 通信组件 |

在蓝图编辑器的 **Components** 面板中确认这三个组件存在。如果缺失, 说明
C++ 编译未成功或模块未正确加载, 请回到第 2 步检查。

### 4.3 配置 TCP 端口

1. 在 Components 面板选中 **TcpComp**。
2. 在 **Details** 面板中, 找到 **Network** 分类。
3. 设置 **Port** = `7777` (默认值, 与 `nav_ue_bridge.exe --port 7777` 一致)。

### 4.4 配置 LiDAR 参数 (可选)

选中 **LiDARComp**, 在 Details 面板可调整:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| NumRays | 72 | 射线数 (72 = 每 5 度一束) |
| MaxRange | 800.0 | 最大量程 (cm), 8m |
| AngleMin | -3.14159 | 起始角度 (弧度) |
| AngleMax | 3.14159 | 终止角度 (弧度) |

> 保持默认值即可, 这些参数与 `nav_ue_bridge.exe` 的协议预期一致。

### 4.5 配置机器人参数 (可选)

选中蓝图根节点 (BP_PuppyRobotPawn), 在 Details 面板的 **Robot** 分类:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| MoveSpeed | 200.0 | 线速度上限 (cm/s = 2 m/s) |
| RotateSpeed | 180.0 | 角速度上限 (deg/s) |

### 4.6 设置 GameMode

1. **Content Browser** 右键 -> **Blueprint Class** -> **Game Mode Base** ->
   命名 `BP_PuppyGameMode`。
2. 打开 `BP_PuppyGameMode`, 设置 **Default Pawn Class** =
   `BP_PuppyRobotPawn`。
3. **Project Settings** -> **Maps & Modes** -> **Global Default Game Mode** =
   `BP_PuppyGameMode`。

### 4.7 编译并保存蓝图

点击蓝图编辑器左上角 **Compile**, 然后 **Save**。

---

## 5. 行人配置

### 5.1 行人蓝图 (自动生成)

`ue_scene_builder.py` 会自动使用 `APuppyPedestrianActor` (C++ 类) 直接
spawn 行人, 无需手动创建蓝图。脚本的 `build_pedestrians()` 函数通过以下
方式加载类:

```python
ped_class = unreal.load_class(None, "/Script/PuppyNav.PuppyPedestrianActor")
```

并设置每个行人的属性:

```python
actor.set_editor_property("velocity", unreal.Vector2D(vx * 100.0, vy * 100.0))
actor.set_editor_property("radius", radius * 100.0)
```

### 5.2 创建 BP_PuppyPedestrianActor (推荐, 用于自定义外观)

如果需要为行人添加网格体或材质, 可创建蓝图:

1. **Content Browser** 右键 -> **Blueprint Class** -> 选
   **PuppyPedestrianActor** -> 命名 `BP_PuppyPedestrianActor`。
2. 打开蓝图, 在 CapsuleComp 下添加 **Static Mesh Component** (可选圆柱体
   或角色模型)。
3. 编译保存。

> **注意**: 如果创建了 `BP_PuppyPedestrianActor`, 需修改
> `ue_scene_builder.py` 中的类加载路径:
> ```python
> ped_class = unreal.load_class(None, "/Game/BP_PuppyPedestrianActor.BP_PuppyPedestrianActor")
> ```

### 5.3 行人行为说明

`APuppyPedestrianActor` 的行为与 C++ 仿真保持一致:

- 匀速直线运动 (速度 3-4 cm/s, 即 0.03-0.04 m/s)
- 碰墙反弹 (通过 `IsInsideWall` 检测, 翻转速度分量)
- 每帧 Tick 中自动更新位置
- 支持 RESET 时恢复初始位置和速度

行人状态会在 `APuppyRobotPawn::SendSensorData()` 中自动采集, 通过
`PED_STATE` 消息发送给 `nav_ue_bridge.exe`。

### 5.4 验证行人配置

PIE (Play In Editor) 后, 在场景中应看到圆柱体行人自动移动并在碰墙时反弹。
Output Log 中应出现:

```
[PuppyTcpServer] Listening on port 7777
[PuppyTcpServer] Client connected
```

---

## 6. 运行仿真

### 6.1 仿真启动顺序

仿真采用 30Hz lockstep 机制: UE 发送一帧传感数据 -> Nav 处理 -> Nav 回发
指令 -> UE 推进一帧物理。

**启动顺序**: 先启动 `nav_ue_bridge.exe`, 再在 UE 中 PIE。

### 6.2 步骤一: 启动 nav_ue_bridge

打开命令提示符 (cmd 或 PowerShell):

```powershell
cd e:\puppyfangzhen\cpp_src\bridge\build
.\nav_ue_bridge.exe --port 7777 --scene ..\..\config\scene_home.json
```

命令行参数说明:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| --port | 7777 | TCP 端口, 需与 UE 的 TcpComp.Port 一致 |
| --frames | 0 | 最大帧数 (0 = 无限) |
| --seed | (随机) | 随机种子 |
| --scene | (无) | 场景 JSON 文件路径 |
| --standalone | (关闭) | 无 UE 自测模式 |

bridge 启动后会等待 UE 连接 (输出等待连接日志)。

### 6.3 步骤二: 在 UE 中 PIE

1. 切换到 UE 编辑器, 确认当前打开的关卡为 `HomeMap`。
2. 点击工具栏 **Play** 按钮 (或按 Alt+P) 开始 PIE。
3. UE 启动后, `PuppyTcpServer` 在 BeginPlay 中创建监听 socket,
   `nav_ue_bridge` 连接成功后开始 lockstep 仿真。

### 6.4 lockstep 数据流

每帧 (30Hz) 的数据流如下:

```
UE5                              nav_ue_bridge
  |                                    |
  |--- LIDAR (72 距离值) ------------->|
  |--- GROUND_TRUTH (x, y, yaw) ----->|
  |--- PED_STATE (N 个行人) ---------->|
  |                                    | (AMCL 更新 + A* 规划 + CBF)
  |<--- CMD_VEL (vx, vy, wz) ----------|
  |<--- DEBUG_POSE (估计位姿+置信度) --|  (可选)
  |<--- DEBUG_PATH (路径+粒子云) ------|  (可选)
  |<--- STEP_ACK ----------------------|
  |                                    |
  |  (UE 推进一帧物理)                   |
```

> **单位换算**: UE 内部使用厘米 (cm), 协议使用米 (m)。
> - 位置: UE cm -> 协议 m (除以 100)
> - 速度: UE cm/s -> 协议 m/s (除以 100)
> - 角速度: UE deg/s -> 协议 rad/s (乘以 pi/180)
> - LiDAR 距离: UE cm -> 协议 m (除以 100)
> - 偏航角: UE 度 -> 协议弧度 (乘以 pi/180)
>
> 这些换算已在 `PuppyTcpServer.cpp` 和 `PuppyRobotPawn.cpp` 中自动处理。

### 6.5 停止仿真

- 停止 UE: 按 **Esc** 或点击 **Stop** 退出 PIE。
- 停止 bridge: 在命令行窗口按 **Ctrl+C**。
- 连接断开时 bridge 会自动退出, UE 会检测到 socket 错误并日志记录。

---

## 7. 调试可视化

`nav_ue_bridge.exe` 会发送两类调试消息, 用于在 UE 中可视化导航算法的
内部状态:

| 消息类型 | 值 | 内容 |
|---------|-----|------|
| DEBUG_POSE | 0x0021 | AMCL 估计位姿 (est_x, est_y, est_yaw) + 置信度 |
| DEBUG_PATH | 0x0022 | A* 路径折线 (最多 256 点) + AMCL 粒子云 (最多 200 点) |

### 7.1 当前状态

`PuppyTcpServer.cpp` 的 `HandleMessage()` 目前收到 DEBUG_POSE 和
DEBUG_PATH 时仅记录日志, 未进行可视化绘制。要启用可视化, 需添加调试绘制
代码。

### 7.2 扩展 PuppyTcpServer 添加调试委托

在 `PuppyTcpServer.h` 中添加委托声明和缓存成员:

```cpp
// AMCL 估计位姿数据 (从 DEBUG_POSE 解析)
USTRUCT(BlueprintType)
struct FPuppyDebugPose
{
    GENERATED_BODY()
    UPROPERTY(BlueprintReadOnly) double EstX = 0.0;
    UPROPERTY(BlueprintReadOnly) double EstY = 0.0;
    UPROPERTY(BlueprintReadOnly) double EstYaw = 0.0;
    UPROPERTY(BlueprintReadOnly) float  Confidence = 0.0f;
};

// A* 路径 + 粒子云 (从 DEBUG_PATH 解析)
USTRUCT(BlueprintType)
struct FPuppyDebugPath
{
    GENERATED_BODY()
    UPROPERTY(BlueprintReadOnly) TArray<FVector2D> PathPoints;
    UPROPERTY(BlueprintReadOnly) TArray<FVector>   Particles; // x, y, yaw(弧度)
};

// 收到 DEBUG_POSE 时触发
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnDebugPoseReceived, const FPuppyDebugPose&, DebugPose);
// 收到 DEBUG_PATH 时触发
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnDebugPathReceived, const FPuppyDebugPath&, DebugPath);
```

在类中添加:

```cpp
UPROPERTY(BlueprintAssignable, Category = "Debug")
FOnDebugPoseReceived OnDebugPoseReceived;

UPROPERTY(BlueprintAssignable, Category = "Debug")
FOnDebugPathReceived OnDebugPathReceived;
```

### 7.3 在 HandleMessage 中解析并广播

在 `PuppyTcpServer.cpp` 的 `HandleMessage()` 中替换 DEBUG_POSE 和
DEBUG_PATH 分支:

```cpp
else if (MsgType == PuppyMsgType::DEBUG_POSE)
{
    if (Data == nullptr || Len < 28) return;  // 3xdouble + float = 28 字节

    FPuppyDebugPose Pose;
    FMemory::Memcpy(&Pose.EstX,       Data,      8);
    FMemory::Memcpy(&Pose.EstY,       Data + 8,  8);
    FMemory::Memcpy(&Pose.EstYaw,     Data + 16, 8);
    FMemory::Memcpy(&Pose.Confidence, Data + 24, 4);

    // 协议单位是米, 转换为 UE 厘米用于绘制
    Pose.EstX *= 100.0;
    Pose.EstY *= 100.0;

    OnDebugPoseReceived.Broadcast(Pose);
}
else if (MsgType == PuppyMsgType::DEBUG_PATH)
{
    if (Data == nullptr || Len < 8) return;

    int32 NPath = 0, NPart = 0;
    FMemory::Memcpy(&NPath, Data, 4);
    FMemory::Memcpy(&NPart, Data + 4, 4);

    FPuppyDebugPath DebugPath;
    DebugPath.PathPoints.SetNum(NPath);
    DebugPath.Particles.SetNum(NPart);

    uint32 Offset = 8;
    for (int32 i = 0; i < NPath; ++i)
    {
        float X = 0.0f, Y = 0.0f;
        FMemory::Memcpy(&X, Data + Offset, 4); Offset += 4;
        FMemory::Memcpy(&Y, Data + Offset, 4); Offset += 4;
        DebugPath.PathPoints[i] = FVector2D(X * 100.0f, Y * 100.0f); // m -> cm
    }
    for (int32 i = 0; i < NPart; ++i)
    {
        float X = 0.0f, Y = 0.0f, Yaw = 0.0f;
        FMemory::Memcpy(&X,   Data + Offset, 4); Offset += 4;
        FMemory::Memcpy(&Y,   Data + Offset, 4); Offset += 4;
        FMemory::Memcpy(&Yaw, Data + Offset, 4); Offset += 4;
        DebugPath.Particles[i] = FVector(X * 100.0f, Y * 100.0f, Yaw);
    }

    OnDebugPathReceived.Broadcast(DebugPath);
}
```

### 7.4 在 PuppyRobotPawn 中绘制调试

在 `PuppyRobotPawn.h` 中添加回调声明:

```cpp
UFUNCTION()
void HandleDebugPose(const FPuppyDebugPose& DebugPose);
UFUNCTION()
void HandleDebugPath(const FPuppyDebugPath& DebugPath);
```

在 `BeginPlay()` 中绑定委托 (与 CMD_VEL 绑定并列):

```cpp
TcpComp->OnDebugPoseReceived.AddDynamic(this, &APuppyRobotPawn::HandleDebugPose);
TcpComp->OnDebugPathReceived.AddDynamic(this, &APuppyRobotPawn::HandleDebugPath);
```

在 `PuppyRobotPawn.cpp` 中实现绘制:

```cpp
void APuppyRobotPawn::HandleDebugPose(const FPuppyDebugPose& DebugPose)
{
    UWorld* World = GetWorld();
    if (!World) return;

    // AMCL 估计位姿: 绿色球体 (偏移 z=50 显示在地面上方)
    FVector EstLoc(DebugPose.EstX, DebugPose.EstY, 50.0f);
    DrawDebugSphere(World, EstLoc, 25.0f, 12, FColor::Green, false, 0.1f, 0, 2.0f);

    // 朝向指示线 (红色)
    float YawDeg = FMath::RadiansToDegrees(DebugPose.EstYaw);
    FVector Forward = FRotator(0.0f, YawDeg, 0.0f).Vector() * 60.0f;
    DrawDebugLine(World, EstLoc, EstLoc + Forward, FColor::Red, false, 0.1f, 0, 3.0f);

    // 置信度文字
    FString Text = FString::Printf(TEXT("conf=%.2f"), DebugPose.Confidence);
    DrawDebugString(World, EstLoc + FVector(0,0,40), Text, nullptr, FColor::Yellow, 0.1f, false, 1.0f);
}

void APuppyRobotPawn::HandleDebugPath(const FPuppyDebugPath& DebugPath)
{
    UWorld* World = GetWorld();
    if (!World) return;

    // A* 路径: 青色折线
    const TArray<FVector2D>& Points = DebugPath.PathPoints;
    for (int32 i = 0; i + 1 < Points.Num(); ++i)
    {
        FVector A(Points[i].X,   Points[i].Y,   10.0f);
        FVector B(Points[i+1].X, Points[i+1].Y, 10.0f);
        DrawDebugLine(World, A, B, FColor::Cyan, false, 0.1f, 0, 3.0f);
    }

    // AMCL 粒子云: 黄色小点
    for (const FVector& P : DebugPath.Particles)
    {
        FVector Loc(P.X, P.Y, 20.0f);
        DrawDebugPoint(World, Loc, 5.0f, FColor::Yellow, false, 0.1f);
    }
}
```

### 7.5 调试可视化颜色约定

| 可视元素 | 颜色 | 说明 |
|---------|------|------|
| AMCL 估计位姿 | 绿色球体 | 估计的机器人位置 |
| 朝向指示 | 红色线段 | 估计的朝向 (yaw) |
| 置信度文字 | 黄色文字 | AMCL 置信度 [0, 1] |
| A* 路径 | 青色折线 | 规划的全局路径 |
| 粒子云 | 黄色点集 | AMCL 粒子采样分布 |

### 7.6 启用调试消息

`nav_ue_bridge.exe` 默认每帧发送 DEBUG_POSE 和 DEBUG_PATH。如果未收到,
检查 bridge 是否正常运行, 以及 Output Log 中是否有
`[PuppyTcpServer] DEBUG_POSE received` 等日志。

---

## 8. 常见问题

### 8.1 编译错误

**问题: 找不到 "Sockets" 或 "Networking" 模块**

```
ERROR: Cannot find module 'Sockets'
```

解决: 确认 `PuppyNav.Build.cs` 中 `PublicDependencyModuleNames` 包含
`"Sockets"` 和 `"Networking"`。这两个是 UE5 内置模块, 但需在 Build.cs
显式声明。重新生成工程文件后重新编译。

**问题: generated.h 文件找不到**

```
Fatal error: 'PuppyRobotPawn.generated.h' file not found
```

解决: UE 的 UnrealHeaderTool 需要先生成 generated 文件。步骤:
1. 关闭 UE 编辑器。
2. 右键 `.uproject` -> **Generate Visual Studio project files**。
3. 在 Visual Studio 中 **生成解决方案**。
4. 编译成功后再打开 UE 编辑器。

**问题: LNK2019 无法解析的外部符号**

```
error LNK2019: unresolved external symbol "public: void __cdecl UPuppyLiDARComponent::GetScanData..."
```

解决: 确认所有 `.cpp` 文件都已复制到 `Source/puppy_ue/` 目录。检查
Visual Studio 解决方案资源管理器中是否包含这些 .cpp 文件。如果没有,
重新生成工程文件。

**问题: 编译时 PCH 相关错误**

```
Fatal error: 'CoreMinimal.h' file not found
```

解决: `PuppyNav.Build.cs` 中已设置 `PCHUsage = UseExplicitOrSharedPCHs`。
确保模块依赖中包含 `"Core"`。如果问题持续, 尝试清理中间文件:
删除 `D:\puppy_ue\Intermediate\` 目录后重新生成。

### 8.2 连接失败

**问题: nav_ue_bridge 启动后一直等待连接**

```
[bridge] Waiting for UE connection on port 7777...
```

解决:
1. 确认 UE 已进入 PIE 模式 (Play In Editor)。
2. 确认 `BP_PuppyRobotPawn` 的 TcpComp **Port** 设置为 7777。
3. 确认防火墙未阻止 localhost TCP 7777 端口。
4. 检查 UE Output Log 是否有
   `[PuppyTcpServer] Listening on port 7777` 日志。
5. 如果端口被占用, 修改 TcpComp.Port 和 `--port` 参数为其他值
   (如 8888), 保持两边一致。

**问题: UE 显示 "Failed to create listen socket on port 7777**

```
[PuppyTcpServer] Failed to create listen socket on port 7777
```

解决: 端口 7777 已被占用 (可能是上次 PIE 未正常退出)。方法:
1. 退出所有 UE 实例。
2. 在 PowerShell 中查找并结束占用进程:
   ```powershell
   Get-NetTCPConnection -LocalPort 7777 | Select-Object OwningProcess
   Stop-Process -Id <PID>
   ```
3. 或更换端口号。

**问题: 连接建立后立即断开**

解决:
1. 确认 `nav_ue_bridge.exe` 版本与 UE 模块版本匹配 (协议版本一致)。
2. 检查 `protocol.h` 中的 `MsgType` 枚举值与
   `PuppyTcpServer.h` 中的 `PuppyMsgType` 常量是否完全一致。
3. 查看 bridge 命令行输出和 UE Output Log 的错误信息。

### 8.3 场景生成问题

**问题: Python 脚本报错 "No module named unreal"**

```
ImportError: No module named 'unreal'
```

解决: 脚本必须在 UE 编辑器的 Python 控制台中运行, 不能在外部 Python
环境中运行。确认:
1. 已启用 **Python Editor Script Plugin**。
2. 在 Output Log 中切换到 **Python** 模式执行命令。

**问题: BSP Brush 未生成或位置错误**

解决:
1. 确认 `scene_home.json` 中的坐标值合理 (单位为米)。
2. 障碍物的 xmin/xmax/ymin/ymax 应满足 xmax > xmin, ymax > ymin。
3. 脚本中障碍物高度固定为 2.5m (250cm)。
4. BSP Brush 需要 **Geometry Editing** 模式才能在编辑器中看到, 但
   LineTrace 射线检测不依赖此模式。

**问题: 行人生成为空 Actor (无运动行为)**

```
[WARN] PuppyPedestrianActor 未加载, 使用 fallback Actor for ped_0
```

解决: 说明 `unreal.load_class(None, "/Script/PuppyNav.PuppyPedestrianActor")`
加载失败。检查:
1. PuppyNav 模块是否编译成功 (见第 2 步)。
2. UE 编辑器是否重启过 (启用插件后需重启)。
3. 在 Content Browser 的 C++ Classes 中确认 `PuppyPedestrianActor` 存在。

### 8.4 帧率与性能优化

**问题: PIE 帧率低于 30 FPS, 仿真卡顿**

lockstep 机制要求 UE 以 30Hz 稳定运行。优化建议:

1. **降低渲染负载**:
   - **Project Settings** -> **Engine** -> **Rendering**:
     关闭 **Anti-Aliasing** (设为 None) 或使用 FXAA (不用 TSR/TAA)。
     关闭 **Motion Blur**。
     关闭 **Contact Shadows**。
   - **Shadow**: 设为 **Low** 或关闭可移动光源阴影。

2. **限制 LiDAR 射线检测频率**:
   LiDAR 每帧执行 72 次 LineTrace, 是主要 CPU 开销。如果帧率不足,
   可在 `PuppyLiDARComponent` 中降低 `NumRays` (如 36), 但需同步
   修改 `nav_ue_bridge.exe` 的预期射线数。

3. **减少行人数量**:
   每个行人每帧执行墙碰撞检测。默认 5 个行人, 如需更多注意性能。

4. **使用固定帧率**:
   在 `Project Settings` -> **Engine** -> **General Settings** ->
   **Framerate** 中:
   - 勾选 **Use Fixed Frame Rate**。
   - 设置 **Fixed Frame Rate** = 30。

5. **打包运行** (脱离编辑器开销):
   - **File** -> **Package Project** -> **Windows** -> 打包到本地目录。
   - 打包后的独立进程性能优于编辑器 PIE。

6. **命令行性能分析**:
   PIE 时按 **~** 键打开控制台, 输入:
   ```
   stat unit          # 查看各线程耗时
   stat fps           # 查看帧率
   stat gpu           # 查看 GPU 耗时
   profilegpu         # 详细 GPU profile
   ```

**问题: lockstep 导致仿真比实时慢**

lockstep 机制下, 如果 UE 单帧处理时间超过 33ms (30Hz), 实际仿真频率
会低于 30Hz, 但仿真时间仍正确 (由 DeltaTime 控制)。这是预期行为, 不会
影响导航算法的正确性, 仅影响视觉流畅度。

### 8.5 数据异常

**问题: LiDAR 数据全部为 max_range (8m)**

说明射线未命中任何障碍物。检查:
1. 场景中是否有 BSP Brush 障碍物 (运行 `ue_scene_builder.py` 后检查)。
2. 障碍物是否与机器人在同一高度。LiDAR 射线在 z=0 平面发射, 障碍物
   高度为 0-2.5m, 应能命中。
3. 障碍物的 Collision 是否启用。BSP Brush 默认有碰撞。

**问题: 机器人不动或移动方向错误**

检查:
1. `nav_ue_bridge.exe` 是否正常发送 CMD_VEL (查看 bridge 控制台输出)。
2. UE Output Log 中是否有
   `[PuppyTcpServer] CMD_VEL vx=... vy=... wz=...` 日志。
3. 单位换算: bridge 发送 m/s, UE 转换为 cm/s (乘以 100)。
4. 机器人 CollisionCapsule 是否卡在墙内 (初始位置可能在障碍物内部)。

**问题: 碰撞计数不断增长**

如果机器人卡在墙角反复碰撞, CollisionCount 会快速增长。检查:
1. 初始位姿是否在开放区域 (不在障碍物内)。
2. `MoveSpeed` 是否过大导致穿墙 (默认 200cm/s = 2m/s, 通常安全)。
3. 机器人半径 (CollisionCapsule 半径 30cm) 是否与场景尺度匹配。

---

## 附录: 快速检查清单

完成所有步骤后, 逐项确认:

- [ ] UE5 5.3 已安装, VS2022 已配置 C++ 游戏开发工作负载
- [ ] `D:\puppy_ue` 工程创建为 C++ 空白工程
- [ ] 9 个 PuppyNav 源文件已复制到 `Source/puppy_ue/`
- [ ] `puppy_ue.uproject` 已添加 PuppyNav 模块声明
- [ ] VS2022 编译成功 (Development Editor | Win64)
- [ ] UE 编辑器中 C++ Classes 可见 PuppyRobotPawn 和 PuppyPedestrianActor
- [ ] Python Editor Script Plugin 已启用
- [ ] `ue_scene_builder.py` 已成功生成场景 (Output Log 无错误)
- [ ] `BP_PuppyRobotPawn` 蓝图已创建, TcpComp.Port = 7777
- [ ] GameMode 已配置, Default Pawn Class = BP_PuppyRobotPawn
- [ ] `nav_ue_bridge.exe` 已编译 (build_bridge.bat 执行成功)
- [ ] 先启动 `nav_ue_bridge.exe --port 7777`, 再 UE PIE
- [ ] UE Output Log 显示 `[PuppyTcpServer] Client connected`
- [ ] 机器人在场景中移动, 行人自动巡逻, LiDAR 射线命中障碍物

---

## 附录: 文件路径速查

| 文件 | 路径 |
|------|------|
| 工程目录 | `D:\puppy_ue\` |
| uproject | `D:\puppy_ue\puppy_ue.uproject` |
| 模块源码 | `D:\puppy_ue\Source\puppy_ue\PuppyNav.Build.cs` |
| 场景脚本 | `e:\puppyfangzhen\cpp_src\bridge\ue_scene_builder.py` |
| 协议文档 | `e:\puppyfangzhen\cpp_src\bridge\protocol.md` |
| 协议头文件 | `e:\puppyfangzhen\cpp_src\bridge\protocol.h` |
| Bridge 源码 | `e:\puppyfangzhen\cpp_src\bridge\nav_ue_bridge.cpp` |
| Bridge 可执行 | `e:\puppyfangzhen\cpp_src\bridge\build\nav_ue_bridge.exe` |
| 构建脚本 | `e:\puppyfangzhen\cpp_src\bridge\build_bridge.bat` |
| 场景配置 | `e:\puppyfangzhen\config\scene_home.json` |
| UE 模块原始源码 | `e:\puppyfangzhen\cpp_src\bridge\ue_modules\` |
