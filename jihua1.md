# Puppy 机器人项目二次开发实施计划

> 文档目的：把当前仓库的改进方向整理成一份程序员可直接执行的开发计划。  
> 文档定位：不是毕业设计介绍，不是泛泛路线图，而是面向实现、联调、验收和回归测试的任务书。  
> 适用范围：当前 `e:\puppyfangzhen` 仓库，覆盖研究型 CoppeliaSim Python 主线、`nav_core` 共享算法层、ROS2/`puppy_core`/`puppypi_adapter` 产品化主线。

---

## 1. 先说结论

当前项目已经不再是“只有一个能跑起来的演示脚本”，而是已经形成了三层雏形：

1. 研究型导航主线  
   入口是 `autonomous_nav.py`，依托 CoppeliaSim 做快速算法迭代。
2. 可复用导航内核  
   入口是 `nav_core/`，已经开始承接运行时状态、frontier 选择、会话持久化、telemetry 等公共能力。
3. ROS2 产品化主线  
   入口是 `src/puppy_core`、`src/puppypi_adapter`、`src/puppy_bringup`，目标是把上层任务、模式、安全、适配层做成真正可部署的系统。

这三个部分现在已经有了骨架，但还没有完全收束成“一个清晰、可维护、可验证”的体系。  
接下来最重要的不是再堆几个新功能，而是把已有成果整理成：

1. 算法更稳定
2. 模块边界更清楚
3. 参数配置更统一
4. 回归测试更可靠
5. ROS2 和研究主线之间的关系更明确

一句话概括：  
**下一阶段的核心任务，是把“能跑的原型”提升成“可以持续开发、持续验证、持续优化的导航平台”。**

---

## 2. 当前代码现状分析

### 2.1 已经完成或基本完成的部分

#### 2.1.1 研究主线闭环已经具备

当前 `autonomous_nav.py` 已经具备完整导航闭环能力，主要包括：

- 占据栅格建图
- costmap 构建
- A* 全局规划
- DWA / TEB 局部规划
- AMCL 定位
- frontier 探索
- RECOVER 状态
- HTML / PNG / CSV / JSON 评估输出

这意味着后续优化不是从零开始，而是在一个已经成型的系统上继续做结构升级和算法加强。

#### 2.1.2 `nav_core` 已经开始承担共享能力

当前 `nav_core` 下已经有以下模块：

- `nav_core/exploration/frontier_manager.py`
- `nav_core/runtime/runtime_state.py`
- `nav_core/runtime/state_machine.py`
- `nav_core/session/session_store.py`
- `nav_core/metrics/telemetry.py`

其中已经落地的关键点包括：

- `NavigationRuntimeState` 已从 `autonomous_nav.py` 中抽出
- `FrontierManager` 已升级为带评分、记忆、TTL、不可达区域管理的版本
- `SessionStore` 已具备地图和 frontier memory 持久化能力
- `TelemetryCollector` 已定义结构化指标采集框架

这说明项目已经走在“统一内核”的正确方向上，只是还没有完全接线完毕。

#### 2.1.3 frontier 选择算法已完成一轮真正升级

当前 `nav_core/exploration/frontier_manager.py` 不再只是“按近距离或简单 utility 选点”，而是已经有多因子评分：

- 信息增益
- 距离代价
- 风险代价
- 朝向变化代价
- 不可达惩罚
- 已访问邻域惩罚

当前 `select_frontier(...)` 已可接受：

- `rx`
- `ry`
- `robot_yaw`

并输出包含 `score` 的 `FrontierSelection`。

这部分是本阶段已经落地的算法升级成果，后续文档中会把它视为“已完成基础”，下一步是在其上继续做 recovery 和 path-quality aware 的升级。

#### 2.1.4 `puppy_core` / `puppypi_adapter` 已经不是空壳

这条线目前已经有可运行骨架：

- `mission_manager.py`
- `goal_dispatcher.py`
- `mode_manager.py`
- `safety_manager.py`
- `robot_state_aggregator.py`
- `battery_status_adapter.py`
- `motion_adapter_node.py`
- `status_adapter_node.py`
- `mode_adapter_node.py`

已经形成了以下系统方向：

- 任务层
- 模式管理层
- 安全层
- 语义状态聚合层
- 平台适配层

但要注意：

- 这条线**已有架构基础**
- 但很多节点仍然是**占位实现、仿真实现、未接入真实 SDK 的骨架**

所以文档里必须明确：  
**可以复用的是框架，不是所有实现都已经达到实机可用等级。**

---

### 2.2 当前系统存在的关键问题

#### 2.2.1 `autonomous_nav.py` 仍然过于庞大

虽然已经抽出 `NavigationRuntimeState` 和 `FrontierManager`，但 `autonomous_nav.py` 仍然承担了太多职责：

- CoppeliaSim 连接
- 仿真对象发现
- 视觉传感器创建
- 雷达建模
- 地图维护
- AMCL 更新
- 状态机执行
- frontier 选点
- DWA/TEB 调度
- recover 行为
- HTML 地图渲染
- CSV 评估写出
- 实验 profile 切换

这会导致两个问题：

1. 后续算法改动容易牵动运行时逻辑
2. 很多行为很难做单元测试或回归测试

所以后续必须继续拆。

#### 2.2.2 `nav_core` 已有模块，但尚未完全接入主循环

目前存在“模块已经写出来，但主入口还没完全使用”的情况，例如：

- `SessionStore` 已存在，但研究主线里还没有形成完整接入闭环
- `TelemetryCollector` 已存在，但研究主线日志与指标仍以旧式散落逻辑为主
- `NavStateMachine` 已存在，但主状态切换仍有不少逻辑留在 `autonomous_nav.py`

这意味着当前仓库已经有“正确方向的中间成果”，下一阶段需要做的不是重写，而是**接线、收口、消除双轨逻辑**。

#### 2.2.3 RECOVER 仍然是当前算法上的最大可提升点

frontier 选择已经做了一轮升级，但 RECOVER 仍主要依赖：

- 方向枚举
- 代价比较
- 大步逃逸
- 原地旋转

它能工作，但还不够“聪明”。  
当前最值得继续做算法升级的地方，不是 frontier，而是：

1. recovery 目标选择
2. recovery 短程规划
3. recovery 成败记忆
4. recovery 与局部规划器的协同

#### 2.2.4 研究主线与 ROS2 产品线的边界还不够清晰

当前存在两条并行路线：

1. `autonomous_nav.py` 为核心的研究主线
2. `src/puppy_*` 为核心的 ROS2 主线

这是合理的，但目前仓库里仍然容易让开发者混淆：

- 哪条线是“研究实验入口”
- 哪条线是“正式 bringup 入口”
- 哪些模块只服务于 research
- 哪些模块必须沉淀到共享层

这需要在计划和实施中明确：

- 研究主线保留用于快速迭代
- 共享算法统一沉淀到 `nav_core`
- ROS2 主线只消费共享能力，不复制研究逻辑

#### 2.2.5 参数已经开始 YAML 化，但还不彻底

当前已有：

- `config/exploration.yaml`
- `config/mapping.yaml`
- `config/planner_global.yaml`
- `config/planner_local.yaml`
- `config/runtime.yaml`
- `config/sim.yaml`
- `config/profile_raspberry_pi.yaml`

这是很好的基础。  
但仍存在问题：

- 部分参数仍硬编码在 `autonomous_nav.py`
- profile 覆盖还不统一
- ROS2 参数和 research 参数还没有统一口径
- 参数变更后还缺少自动化验收链路

---

## 3. 当前代码成熟度分级

为了方便程序员判断哪里是“可以直接扩展”的，哪里是“先要重构”的，先做一个成熟度分级。

### 3.1 A 级：可直接复用并继续扩展

这些模块已经具备明确职责和较好的结构，后续可以继续增强：

- `nav_core/exploration/frontier_manager.py`
- `nav_core/runtime/runtime_state.py`
- `nav_core/runtime/state_machine.py`
- `nav_core/session/session_store.py`
- `nav_core/metrics/telemetry.py`
- `src/puppy_core/puppy_core/mission_manager.py`
- `src/puppy_core/puppy_core/robot_state_aggregator.py`
- `src/puppy_core/puppy_core/safety_manager.py`

### 3.2 B 级：结构方向正确，但实现还需要补强

- `autonomous_nav.py`
- `src/puppypi_adapter/puppypi_adapter/motion_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/status_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/mode_adapter_node.py`
- `src/puppy_core/puppy_core/goal_dispatcher.py`
- `src/puppy_core/puppy_core/mode_manager.py`

这些模块不需要推倒重来，但需要继续做：

- 边界收缩
- 真实实现替换占位逻辑
- 与配置和测试体系接通

### 3.3 C 级：可以保留为参考，但不适合作为长期核心入口

- `autonomous_nav.py` 里与仿真强耦合的对象初始化部分
- `src/puppy_bringup/scripts/coppelia_bridge.py`
- `src/puppy_localization/src/leg_odometry.cpp`

这些部分并不是没用，而是：

- 对研究很有用
- 对快速演示很有用
- 但不适合继续承担“统一平台核心实现”的角色

---

## 4. 本轮开发总体目标

本轮开发建议定义为以下四个目标：

### 目标 1：继续提升探索与恢复算法

重点是：

- frontier 打分继续增强
- RECOVER 从“逃逸动作”升级为“短程决策模块”
- 局部规划器与探索器协同增强

### 目标 2：继续把研究主线逻辑沉淀到 `nav_core`

重点是：

- 继续缩小 `autonomous_nav.py`
- 提高可测性
- 为 ROS2 主线复用做准备

### 目标 3：把 ROS2 / `puppy_core` / `puppypi_adapter` 主线从“骨架可跑”推进到“联调可用”

重点是：

- 统一 topic / action / service 契约
- 明确 bringup 路径
- 补齐 adapter 的真实职责

### 目标 4：建立更像工程系统的回归验证机制

重点是：

- 单元测试
- 场景回归测试
- profile 验证
- 指标对比

---

## 5. 验收指标

本轮开发不建议只用“能不能跑”做验收，而应使用一组量化指标。

### 5.1 研究主线指标

至少记录以下指标：

- 覆盖率 `coverage`
- 总路径长度 `distance`
- frontier 切换次数
- replan 次数
- recover 次数
- recover 成功率
- 局部规划失败率
- 门口穿越次数
- 平均规划耗时
- 平均控制周期耗时
- AMCL 平均定位误差

### 5.2 ROS2 主线指标

至少记录以下工程指标：

- bringup 成功率
- 关键节点启动成功率
- 关键 topic 就绪时间
- command timeout 是否生效
- 低电量降级是否生效
- 跌倒事件是否触发 SAFE_STOP
- 导航 action 是否可用

### 5.3 代码与测试指标

- `nav_core` 单元测试通过
- `puppy_core` 回归测试通过
- `compileall` / `py_compile` 通过
- 关键 launch 可成功解析
- 新增模块必须配最少一类测试

---

## 6. 分阶段实施路线

本轮建议拆成 6 个阶段推进。

---

## 7. 阶段 P0：文档与边界收口

### 7.1 目标

在开始继续写代码之前，先把边界定死，避免后续开发继续出现“双轨实现”和“重复造轮子”。

### 7.2 主要任务

1. 明确研究主线入口
2. 明确 ROS2 主线入口
3. 明确 `nav_core` 的职责边界
4. 明确哪些旧脚本只允许 bugfix，不再继续堆业务逻辑

### 7.3 涉及文件

- `jihua1.md`
- `HANDOVER.md`
- `HANDOFF.md`
- `IMPROVEMENT_ROADMAP.md`
- `gaijin1.md`
- `gaijin2.md`

### 7.4 实施要求

程序员需要统一认知：

- `autonomous_nav.py` 是 research runtime
- `src/puppy_bringup/launch/*.launch.py` 是 ROS2 系统入口
- `nav_core` 是共享算法层
- `puppy_core` 是任务与模式层
- `puppypi_adapter` 是平台适配层

### 7.5 验收标准

- 团队内部对各层职责描述一致
- 后续新增功能不再直接塞进不合适的入口文件

---

## 8. 阶段 P1：继续拆分 `autonomous_nav.py`

### 8.1 目标

把研究主线中的运行逻辑进一步外提到 `nav_core`，让 `autonomous_nav.py` 逐步只承担：

- research runtime 组装
- CoppeliaSim 适配
- 调试输出

### 8.2 当前已完成基础

已完成：

- `NavigationRuntimeState`
- `FrontierManager`

待继续做：

- state machine 纯逻辑继续外提
- recovery 逻辑模块化
- telemetry 接线
- session 持久化接线

### 8.3 具体任务

#### 任务 P1-1：把 PLAN/FOLLOW/RECOVER 的状态迁移判定继续压到 `NavStateMachine`

涉及文件：

- `autonomous_nav.py`
- `nav_core/runtime/state_machine.py`
- `nav_core/test_state_machine.py`

实现步骤：

1. 清点 `autonomous_nav.py` 中所有状态切换分支
2. 把纯决策条件迁移到 `NavStateMachine`
3. 保留 runtime 里只做环境调用与副作用处理
4. 为每个 transition 分支补测试

风险：

- 状态切换和副作用耦合严重，拆分时容易漏掉计数器清零

验收标准：

- `autonomous_nav.py` 中状态切换条件显著减少
- `NavStateMachine` 能独立覆盖 PLAN/FOLLOW/RECOVER/DONE 转移

建议测试：

- 到达 goal
- path 末端但 goal 仍远
- 无 frontier
- no progress
- recover 超时返回 PLAN

#### 任务 P1-2：把 recovery 行为提炼为独立模块

涉及文件：

- 新增建议：`nav_core/runtime/recovery_manager.py`
- `autonomous_nav.py`
- `nav_core/test_recovery_manager.py`

实现步骤：

1. 抽取 recovery 输入输出接口
2. 让 recovery 模块只接受：
   - 当前 pose
   - 当前 path
   - 当前 costmap
   - oscillation 标志
   - lethal 标志
3. 输出：
   - 恢复动作建议
   - 恢复原因
   - 是否需要 replan

风险：

- 当前 recovery 中混有直接 `sim.setObjectPosition` 和 `setObjectOrientation`

验收标准：

- recovery 选方向的核心评分不再写在主循环里
- 可对 recovery 逻辑单独做单元测试

#### 任务 P1-3：接入 `TelemetryCollector`

涉及文件：

- `autonomous_nav.py`
- `nav_core/metrics/telemetry.py`

实现步骤：

1. 在主循环 frame 开始和结束处接线
2. 在 A* / DWA / TEB 调用周围采集耗时
3. 在 recover 触发处记录 recover reason
4. 在 frontier 数量变化处记录 frontier_count

风险：

- 不要让 telemetry 反向污染业务逻辑

验收标准：

- 运行时定期打印统一 telemetry 摘要
- `get_summary()` 输出能写入实验结果

#### 任务 P1-4：接入 `SessionStore`

涉及文件：

- `autonomous_nav.py`
- `nav_core/session/session_store.py`

实现步骤：

1. 启动时加载 map
2. 启动时加载 frontier memory
3. 定时保存 map
4. 定时保存 frontier memory
5. 结束时记录 run statistics

风险：

- JSON 序列化 frontier memory 时键值转换容易出错

验收标准：

- 中断后重启可恢复地图和 frontier 记忆
- `meta_file` 中能看到 run_count、best_coverage、params_summary

---

## 9. 阶段 P2：继续升级 frontier 与探索算法

### 9.1 目标

在现有 multi-factor frontier scoring 基础上，继续提升选点质量，减少无效探索和重复恢复。

### 9.2 当前已完成基础

已完成：

- info_gain
- distance
- risk
- heading_change
- unreachable penalty
- revisit penalty

### 9.3 具体任务

#### 任务 P2-1：把 frontier 打分参数从硬编码改为完整配置驱动

涉及文件：

- `nav_core/exploration/frontier_manager.py`
- `config/exploration.yaml`
- `config_loader.py` 或相应注入层
- `autonomous_nav.py`

实现步骤：

1. 读取 `exploration.w_*`
2. 读取 TTL、unreachable radius、revisit 半径
3. 在主入口注入 `FrontierManager`
4. 移除散落常量

风险：

- 当前 research runtime 和 `FrontierManager` 构造函数之间还没有完整参数注入链

验收标准：

- 改 YAML 不改代码即可影响 frontier 选择行为

#### 任务 P2-2：引入“路径质量感知”的 frontier 评分

当前问题：

现在 frontier 评分仍更多依赖欧氏距离和当前位置局部信息，缺少“到达该 frontier 的路径是否好走”的判断。

建议增加因子：

- A* 路径长度
- 最小走廊宽度
- 路径平均代价值
- 路径弯折次数

涉及文件：

- `nav_core/exploration/frontier_manager.py`
- `astar_planner.py` 或后续抽象的 path evaluator
- 新增建议：`nav_core/planners/path_metrics.py`

实现步骤：

1. 为候选 frontier 生成轻量路径质量估计
2. 定义 path_quality_score
3. 并入 frontier 总分
4. 做 A/B 对比实验

风险：

- 候选 frontier 多时，逐一跑完整 A* 代价高

建议做法：

- 第一版先做启发式近似
- 第二版再做更精确的 path metrics

验收标准：

- 减少“看起来近但实际上很难走”的 frontier 被选中

#### 任务 P2-3：升级 frontier 聚类

当前问题：

已有 `_cluster_frontiers(...)`，但还是偏简单合并，无法充分表达 frontier region。

建议升级方向：

- region 级 frontier
- cluster centroid + representative point
- cluster width / shape / orientation

涉及文件：

- `nav_core/exploration/frontier_manager.py`
- `occupancy_grid.py`

实现步骤：

1. 把 frontier cell 提升为 frontier cluster
2. 为 cluster 计算：
   - centroid
   - cell_count
   - principal direction
   - representative entry point
3. 选点时优先选代表点而不是几何中心

验收标准：

- 相邻 frontier 不再频繁重复切换
- 大片连续 frontier 只形成少量稳定候选

#### 任务 P2-4：加强“不可达区域建模”

当前问题：

`unreachable_goals` 以圆形扩展区为主，表达能力仍偏弱。

建议升级：

- 区域级不可达
- 失败原因分类
- 失败冷却时间
- 与地图版本绑定

涉及文件：

- `nav_core/exploration/frontier_manager.py`
- `nav_core/session/session_store.py`

实现步骤：

1. 增加 failure metadata
2. 区分：
   - path not found
   - local planner failed
   - repeated oscillation
   - lethal zone trap
3. 让不同失败原因使用不同冷却策略

验收标准：

- 不再只靠“简单标黑一片区域”避免重复失败

---

## 10. 阶段 P3：重点升级 RECOVER 算法

### 10.1 目标

把 recovery 从“逃逸动作集合”提升为“短程恢复决策器”。

### 10.2 为什么这是当前最值得做的算法点

frontier 选择已经明显提升，但系统仍会在这些场景受限：

- 门口摇摆
- 贴障碍震荡
- path 已失效但局部规划仍不断尝试
- 进入 inflation 深区后逃逸效率低

这些问题本质上都和 recover 的策略上限有关。

### 10.3 具体任务

#### 任务 P3-1：给 recover 增加“恢复目标点”概念

当前 recover 多是方向和步长，不是目标点规划。

建议改成：

- 先选一个短程恢复目标点
- 再执行短路径或短轨迹

涉及文件：

- 新增建议：`nav_core/runtime/recovery_manager.py`
- `autonomous_nav.py`

实现步骤：

1. 在机器人附近采样候选恢复点
2. 依据以下因子打分：
   - 当前 cost
   - 到目标点路径通畅性
   - 与原路径的重接难度
   - 与障碍物距离
   - 是否远离 oscillation 区
3. 选出最优恢复目标

验收标准：

- recover 结果从“转一下、蹭一下”变成“明确逃到某个更可恢复的位置”

#### 任务 P3-2：增加短程 mini-plan

建议做法：

- recovery 不是直接给速度
- 而是生成一个 0.5m~1.5m 的 mini path

涉及文件：

- `nav_core/runtime/recovery_manager.py`
- `astar_planner.py` 或局部短规划模块

实现步骤：

1. 用低分辨率或局部窗口生成 mini path
2. 由局部规划器执行 mini path
3. 执行完成后再回主路径规划

验收标准：

- recover 的行为更连贯
- 原地反复转向和试探减少

#### 任务 P3-3：增加 recover 失败记忆

建议记录：

- 恢复点历史
- 失败方向历史
- 同一位置 recover 次数

涉及文件：

- `NavigationRuntimeState`
- `SessionStore`
- `RecoveryManager`

验收标准：

- 同一 trap 区域中 recovery 不再反复试同一种动作

#### 任务 P3-4：局部规划器和 recover 协同

建议：

- 当 DWA/TEB 在某些模式下持续失败时，不要等完全卡死再 recover
- 引入“局部失败预警”

可用信号：

- valid trajectory ratio
- local cost surge
- path deviation spike
- repeated heading flip

验收标准：

- recover 更早触发，但总触发次数不盲目上涨

---

## 11. 阶段 P4：升级局部规划与模式感知

### 11.1 目标

进一步提升 DWA/TEB 在狭窄通道、门口、近障碍区域的表现。

### 11.2 具体任务

#### 任务 P4-1：真正落地 narrow passage mode

当前 `config/planner_local.yaml` 中已经有：

- `narrow_passage_mode`
- `narrow_w_clearance`
- `narrow_w_path`

说明系统已经为这件事预留了配置位，但逻辑还没有真正收口。

涉及文件：

- `dwa_planner.py`
- `teb_planner.py`
- `config/planner_local.yaml`
- `autonomous_nav.py`

实现步骤：

1. 定义狭窄通道检测规则
2. 进入窄通道后切换局部规划参数
3. 离开窄通道后恢复默认参数

验收标准：

- 门口 oscillation 明显减少
- path adherence 明显提升

#### 任务 P4-2：补充局部规划器可观测指标

建议记录：

- valid ratio
- best trajectory score
- clearance min
- heading score
- goal score
- path score

涉及文件：

- `dwa_planner.py`
- `teb_planner.py`
- `nav_core/metrics/telemetry.py`

验收标准：

- 局部规划失败的原因能从指标上看出来

#### 任务 P4-3：定义 DWA / TEB 对照实验基线

目标：

让“算法换了之后到底变好了没有”可以有统一结论。

实现要求：

- 固定地图
- 固定起点
- 固定随机种子
- 固定帧数上限

输出指标：

- 覆盖率
- 距离
- 恢复次数
- 失败次数
- 门口穿越
- 平均耗时

---

## 12. 阶段 P5：完善 ROS2 产品主线

### 12.1 目标

把 `puppy_core + puppypi_adapter + bringup` 从“可运行骨架”推进到“工程上能联调”的状态。

### 12.2 具体任务

#### 任务 P5-1：统一 topic / action / service 契约

当前现状：

- 基本方向是对的
- 但仍有原始 topic、语义 topic、兼容层 topic 混用情况

涉及文件：

- `src/puppy_interfaces/*`
- `src/puppy_core/puppy_core/*`
- `src/puppypi_adapter/puppypi_adapter/*`

实施要求：

明确三层概念：

1. 原始标准消息层  
   例如：
   - `sensor_msgs/BatteryState`
   - `sensor_msgs/Imu`
   - `sensor_msgs/LaserScan`

2. 语义消息层  
   例如：
   - `BatteryStatus`
   - `RobotHealth`
   - `PlatformMotionState`
   - `FallEvent`

3. 任务与控制接口层  
   例如：
   - `/mission/command`
   - `/goal/named`
   - `/goal/pose`
   - `/robot/mode`
   - `/robot/motion_enable`

验收标准：

- 上层业务不直接依赖底层原始硬件细节
- 同类语义只有一套主接口，不再多路并存

#### 任务 P5-2：补齐 `puppypi_adapter` 的真实职责

当前现状：

- `motion_adapter_node.py`、`status_adapter_node.py`、`mode_adapter_node.py` 结构已具备
- 但很多逻辑仍是：
  - `use_sim=True`
  - 模拟数据
  - TODO 标记

涉及文件：

- `src/puppypi_adapter/puppypi_adapter/motion_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/status_adapter_node.py`
- `src/puppypi_adapter/puppypi_adapter/mode_adapter_node.py`
- `src/puppypi_adapter/config/adapter_params.yaml`

实施步骤：

1. 抽 SDK client 封装
2. 把真实下发和真实回读都压进 adapter
3. 保持 `use_sim` / `use_hardware` 两套模式共存
4. 不允许上层直接触碰 SDK

验收标准：

- 上层接口不变
- 底层从 mock 切到真机时不需要改任务层代码

#### 任务 P5-3：bringup 路径收口

建议明确四类入口：

1. 研究主线入口
2. mock 产品线入口
3. 仿真产品线入口
4. 实机产品线入口

涉及文件：

- `src/puppy_bringup/launch/*`
- `src/puppypi_mock/launch/mock_bringup.launch.py`
- `src/puppypi_adapter/launch/adapter_bringup.launch.py`
- `src/puppy_core/launch/core_bringup.launch.py`

实施要求：

- 每个入口说明依赖哪些节点
- 参数路径统一用 package share
- use_sim_time 统一贯通

验收标准：

- 新人能按文档一条命令起对系统
- 不再依赖“猜时间差”的启动方式

#### 任务 P5-4：加强 `robot_state_aggregator`

当前已经有 timeout 检测基础，这很好。  
下一步建议补：

- camera timeout
- nav action readiness
- mode consistency
- motion enable consistency

涉及文件：

- `src/puppy_core/puppy_core/robot_state_aggregator.py`

验收标准：

- `RobotHealth` 能真正作为全局健康状态来源

#### 任务 P5-5：加强 `mission_manager`

当前 `mission_manager.py` 已具备：

- patrol
- dock
- goto
- low battery 中断
- security 中断
- capability 检查

下一步建议补：

- 任务优先级
- 明确任务恢复策略
- 失败重试策略
- dock server 真正闭环

验收标准：

- 任务层不只是“发一个 Nav2 goal”，而是开始具备真正的任务调度语义

---

## 13. 阶段 P6：测试与回归体系完善

### 13.1 目标

把当前的“手工调试经验”沉淀成系统化测试。

### 13.2 当前已有测试基础

已有：

- `nav_core/test_frontier_manager.py`
- `nav_core/test_runtime_state.py`
- `nav_core/test_state_machine.py`
- `src/puppy_core/test/test_regression.py`

这说明项目已经有可扩展测试土壤，不需要从零开始搭。

### 13.3 具体任务

#### 任务 P6-1：为 recovery 新模块补单元测试

建议测试：

- lethal zone escape
- oscillation escape
- backward preference
- path-aligned recovery
- repeated failure penalty

#### 任务 P6-2：为 frontier path-quality 打分补测试

建议测试：

- 两个 info_gain 相近但路径质量不同的 frontier
- 已访问邻域惩罚是否生效
- heading penalty 是否生效

#### 任务 P6-3：补 research runtime smoke test

建议最少做：

- import 成功
- 关键配置可加载
- 主状态机对象可初始化

由于 CoppeliaSim 依赖较重，不要求完整仿真自动化，但至少要让“代码级错误”尽早暴露。

#### 任务 P6-4：补场景回归测试规范

建议定义固定场景：

1. 开阔场景
2. 门口场景
3. 多房间场景
4. 狭窄走廊场景
5. 障碍陷阱场景

每次改动输出统一表格：

- coverage
- distance
- recover
- replans
- average planning time
- doorway crossing

#### 任务 P6-5：补 ROS2 launch smoke 回归

建议覆盖：

- `core_bringup.launch.py`
- `adapter_bringup.launch.py`
- `full_system.launch.py`

验收标准：

- 关键 launch 至少能解析
- 关键节点入口不会因改名或参数问题失效

---

## 14. 优先级清单

为了避免程序员同时改太多地方，下面给出建议优先级。

### P0：必须先做

1. 文档边界收口
2. `autonomous_nav.py` 继续拆分
3. `TelemetryCollector` 接线
4. `SessionStore` 接线
5. `FrontierManager` 参数配置化

### P1：优先投入

1. RecoveryManager 抽离
2. 路径质量感知 frontier 评分
3. narrow passage mode 真正落地
4. `puppypi_adapter` 真正接入职责补强
5. bringup 路径收口

### P2：第二批推进

1. frontier region 建模
2. recover mini-plan
3. 任务恢复与优先级增强
4. 更强的 health 聚合
5. 场景回归自动化

### P3：中长期

1. JPS / Hybrid A* 对照
2. 更强的 TEB 参数自适应
3. 更真实的实机 adapter
4. 更完整的离线 benchmark 流水线

---

## 15. 建议的开发顺序

建议按照下面顺序推进，而不是并行乱改。

### 第 1 批

- P1-1 状态机纯化
- P1-3 telemetry 接线
- P1-4 session 接线
- P2-1 frontier 参数配置化

目标：

先把“结构与观测”打牢。

### 第 2 批

- P1-2 RecoveryManager 抽离
- P3-1 recover 目标点
- P3-2 recover mini-plan

目标：

先攻最有价值的算法短板。

### 第 3 批

- P2-2 路径质量感知 frontier
- P2-3 frontier region 聚类
- P4-1 narrow passage mode

目标：

继续提升探索效率和局部稳定性。

### 第 4 批

- P5-1 接口统一
- P5-2 adapter 补强
- P5-3 bringup 收口
- P5-4 health 聚合增强

目标：

让产品主线更像正式系统。

### 第 5 批

- P6 系列测试建设

目标：

把经验固化成回归能力。

---

## 16. 给程序员的具体执行要求

### 16.1 写代码前必须做的事

1. 先读本文件
2. 读 `nav_core` 当前测试
3. 读 `autonomous_nav.py` 当前主循环
4. 明确是改 research 线还是 ROS2 线

### 16.2 改代码时的约束

1. 新能力优先落在 `nav_core`，不要继续把算法塞回 `autonomous_nav.py`
2. ROS2 上层不要直接依赖底层 SDK
3. 配置优先用 YAML，不要继续散落硬编码
4. 新增行为必须至少补一个测试或 smoke 验证
5. 改 frontier / recovery 后必须跑基准实验

### 16.3 每项任务交付时必须包含

1. 改动文件清单
2. 实现说明
3. 风险说明
4. 测试结果
5. 对比指标

---

## 17. 建议的每周里程碑

### 里程碑 M1：结构收口

完成：

- telemetry 接线
- session 接线
- frontier 参数配置化
- 状态机继续抽离

验收：

- 主循环更瘦
- 指标更完整
- 可恢复上次地图和 frontier 状态

### 里程碑 M2：恢复算法升级

完成：

- RecoveryManager
- recover 目标点
- mini-plan

验收：

- recover 更稳定
- 门口/贴障场景下无效震荡减少

### 里程碑 M3：探索算法升级

完成：

- path-quality aware frontier score
- frontier region 聚类
- 狭窄通道模式

验收：

- 路径更短
- replan / recover 减少
- 覆盖率不下降

### 里程碑 M4：ROS2 主线强化

完成：

- adapter 职责补齐
- bringup 收口
- health 聚合增强
- mission 管理增强

验收：

- mock / sim / hardware 三条链路口径一致

### 里程碑 M5：回归体系落地

完成：

- recovery 测试
- frontier 测试
- launch smoke
- 场景对比表

验收：

- 之后每次算法升级都有稳定比较依据

---

## 18. 最终目标定义

当下面这些条件都满足时，可以认为项目进入了下一阶段：

1. `autonomous_nav.py` 不再承担大段核心决策逻辑
2. `nav_core` 真正成为 research 与 ROS2 共享的算法层
3. frontier 和 recover 都进入“可独立测试、可独立优化”的状态
4. `puppy_core` 与 `puppypi_adapter` 的边界稳定
5. bringup 路径清晰
6. 改算法后可以用统一指标做对比

---

## 19. 最后建议

如果只能继续做三件最值的事，建议优先做这三件：

1. 继续拆 `autonomous_nav.py`，把 telemetry / session / recovery 模块真正接进 `nav_core`
2. 把 RECOVER 升级成短程恢复决策器
3. 建立统一的场景回归与指标对比

这三件事的共同价值是：

- 不只是“修 bug”
- 不只是“再加功能”
- 而是在同时抬高项目的算法上限、工程上限和后续可维护性

对程序员来说，这份计划最重要的执行原则只有一句：

**以后每加一项能力，都尽量让它成为可复用模块、可配置能力、可测试单元，而不是主脚本里的又一段分支逻辑。**
