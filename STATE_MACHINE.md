# 状态机文档 (STATE_MACHINE.md)

本文档描述系统中所有状态机及其切换条件，对应 gaijin2.md 第15.1节要求的交付文档。

---

## 1. 导航状态机 (NavStateMachine)

位置：`nav_core/runtime/state_machine.py`

控制探索导航的底层状态循环。

```
        ┌─────────────────────────────────┐
        │                                 │
        ▼                                 │
    ┌────────┐  path found   ┌────────┐   │
    │  PLAN  │ ───────────►  │ FOLLOW │   │
    └────────┘               └────────┘   │
        │                        │        │
        │ no frontier            │ no     │
        │ (cycle > 5)            │ progress│
        ▼                        ▼        │
    ┌────────┐  10 frames   ┌────────┐    │
    │  DONE  │ ◄──────────  │ RECOVER │ ───┘
    └────────┘              └────────┘
        │ new frontier
        ▼
    ┌────────┐
    │  PLAN  │
    └────────┘
```

### 状态定义

| 状态 | 值 | 说明 |
|------|---|------|
| `PLAN` | 0 | 前沿检测 + A*全局规划 |
| `FOLLOW` | 1 | DWA局部规划 + 路径跟踪 |
| `RECOVER` | 2 | 8方向采样最低cost逃逸 |
| `DONE` | 3 | 探索完成，保持位置 |

### 转换条件

| 从 | 到 | 条件 |
|----|----|----|
| PLAN | FOLLOW | 找到可达frontier且A*规划成功 |
| PLAN | RECOVER | 有frontier但A*规划失败 |
| PLAN | RECOVER | 无可达frontier (cycle <= 5) |
| PLAN | DONE | 无可达frontier (cycle > 5) |
| FOLLOW | PLAN | 到达goal (dist < 0.5m) |
| FOLLOW | PLAN | 到达path终点但goal仍远 (path_end < 0.4m, goal > 0.6m) |
| FOLLOW | RECOVER | 无进展 (no_progress > 30帧) |
| RECOVER | PLAN | 恢复完成 (10帧后) |
| DONE | PLAN | 检测到新的可达frontier |

### FOLLOW子状态

FOLLOW状态内部由DWA控制：
- **ALIGN**: heading_err > 0.5 rad → 原地旋转对准
- **DRIVE**: heading_err <= 0.5 rad → 采样速度空间前进

---

## 2. 机器人模式状态机 (ModeManager)

位置：`puppy_core/mode_manager.py`

控制机器人整体运行模式，优先级仲裁。

### 模式定义（按优先级升序）

| 模式 | 优先级 | motion_enabled | autonomy_enabled | 说明 |
|------|--------|---------------|------------------|------|
| `IDLE` | 0 | false | false | 空闲 |
| `READY` | 1 | true | false | 就绪 |
| `TELEOP` | 2 | true | false | 遥控 |
| `NAVIGATION` | 3 | true | true | 自主导航 |
| `PATROL` | 4 | true | true | 巡逻 |
| `SECURITY` | 5 | true | true | 安防 |
| `DOCKING` | 6 | true | true | 回充 |
| `SAFE_STOP` | 90 | false | false | 安全停止(锁定) |
| `FAULT` | 100 | false | false | 故障(锁定) |

### 转换规则

```
IDLE ────► READY ◄───► TELEOP
              │
              ├────► NAVIGATION ◄───► SAFE_STOP
              ├────► PATROL ────────► DOCKING
              ├────► SECURITY
              └────► DOCKING
                       │
SAFE_STOP ────► READY (解锁)
FAULT ────► READY (清除故障后)
```

### 自动触发

| 条件 | 触发模式 |
|------|---------|
| 电量 < 10% | → SAFE_STOP |
| 电量 < 20% 且在PATROL | → DOCKING |
| 检测到跌倒 | → SAFE_STOP |
| 外部请求 | → 请求的模式(受优先级限制) |

### 锁定机制

进入 `SAFE_STOP` 或 `FAULT` 时锁定，只能通过显式切换到 `READY` 解锁。低优先级模式请求在锁定状态下被拒绝。

---

## 3. 平台运动状态机 (MotionAdapter)

位置：`puppypi_adapter/motion_adapter_node.py`

控制PuppyPi底盘运动平台的状态。

```
INIT ──► DISABLED ──► READY ◄──► EXECUTING
                          │
                          ├──► RECOVERING ──► READY
                          │
                          └──► SAFE_STOP ──► FAULT
```

| 状态 | 说明 |
|------|------|
| `INIT` | 初始化中，SDK连接 |
| `DISABLED` | 电机未使能 |
| `READY` | 站立就绪，可接收命令 |
| `EXECUTING` | 正在执行运动命令 |
| `RECOVERING` | 姿态恢复中 |
| `SAFE_STOP` | 安全停止 |
| `FAULT` | 平台故障 |

### 安全约束

1. 未站立时拒绝 `/cmd_vel`
2. 命令超时(1s)自动停车
3. 速度限幅 (max 0.3 m/s, 1.2 rad/s)
4. 加速度限制 (2.0 m/s², 4.0 rad/s²)
5. 死区过滤 (0.01 m/s, 0.05 rad/s)

---

## 4. 任务优先级 (MissionManager)

位置：`puppy_core/mission_manager.py`

### 任务类型与优先级

| 优先级 | 任务 | 说明 |
|--------|------|------|
| 1 (最高) | SAFE_STOP | 安全停止 |
| 2 | RespondToFall | 跌倒响应 |
| 3 | ReturnToDock | 低电返航 |
| 4 | SecurityCheck | 安防检查 |
| 5 | GoToPose | 点到点导航 |
| 6 | PatrolRoute | 路线巡逻 |
| 7 (最低) | Idle | 空闲行为 |

### 仲裁规则

- 高优先级任务可中断低优先级任务
- 同优先级任务排队执行
- SAFE_STOP 和 FALL 立即中断所有任务
- 低电量自动中断 PATROL 并触发 ReturnToDock

---

## 5. 探索Frontier管理 (FrontierManager)

位置：`nav_core/exploration/frontier_manager.py`

### Frontier评分模型

```
score = w1 * info_gain      # 信息增益(frontier大小)
      - w2 * path_cost       # 路径长度代价
      - w3 * risk            # 到达风险(costmap cost)
      - w4 * heading_change  # 朝向切换成本
      - w5 * retry_penalty   # 历史失败惩罚
```

默认权重：0.30, 0.25, 0.15, 0.15, 0.15

### Frontier记忆机制

| 机制 | 说明 |
|------|------|
| `unreachable_goals` | 不可达目标(带3格半径空间扩展)，永久屏蔽 |
| `visited_frontiers` | 已访问frontier，TTL=300帧后可重新选择 |
| `visited_count` | 访问次数，>=2次标记为persistent frontier→unreachable |

### 不可达判定流程

```
1. A*规划 → path终点
2. 机器人到达path终点 (dist < 0.4m)
3. 但goal仍远 (dist > 0.6m)
4. → 标记goal为unreachable (带空间扩展)
5. → 回到PLAN寻找新frontier
```

### 持久frontier判定

```
1. 机器人到达goal (dist < 0.5m)
2. 该goal之前被访问过 (在visited_frontiers中)
3. → 标记为persistent frontier → unreachable
4. (墙后无法观测的unknown区域，到达无法消除frontier)
```
