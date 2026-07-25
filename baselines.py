#!/usr/bin/env python3
"""
Standard Exploration Baselines (标准探索基线)
============================================
提供3个标准基线方法用于与proposed方法对比：
  1. RandomWalkBaseline - 随机游走（下界基准）
  2. NearestFrontierBaseline - 最近边界探索 (Yamauchi 1997)
  3. GreedyInformationBaseline - 贪心信息增益探索 (Bourgault 2002)

这些基线与 proposed 方法使用相同的传感器/地图接口，
仅探索策略不同，便于公平对比。

参考文献:
  - Yamauchi (1997) "A frontier-based approach for autonomous exploration"
  - Bourgault et al. (2002) "Information based adaptive robotic exploration"
"""
import math
import numpy as np

from occupancy_grid import OccupancyGrid


class BaselineStrategy:
    """基线探索策略基类。

    定义所有基线方法的统一接口 ``select_goal``，使不同探索策略可在
    相同的评估框架下公平对比。子类必须实现该方法。

    设计原则:
      - 与 proposed 方法共享相同的 OccupancyGrid / Costmap 接口
      - 不依赖 CoppeliaSim 或任何仿真器（纯 Python 策略层）
      - 每次调用 select_goal 返回一个 (goal_x, goal_y) 目标点
      - 策略可维护内部状态（如当前目标、卡死计数等）

    理论背景:
      自主探索的核心问题是 "下一步去哪里"。不同策略在信息利用效率
      和计算复杂度上有不同的 trade-off：
        - 随机策略无需任何地图推理，但效率最低（下界）
        - 边界策略利用已观测/未观测的边界，效率中等
        - 信息论策略显式建模信息增益，效率较高但计算昂贵
        - Oracle 策略假设已知全局最优，给出理论上界
    """

    def select_goal(self, occ_grid, robot_pos, costmap=None):
        """选择下一个探索目标点。

        Args:
            occ_grid: OccupancyGrid 实例，包含当前地图状态
                （log_odds 概率栅格、visited 访问标记、find_frontiers 边界检测）
            robot_pos: (x, y) 元组，机器人当前世界坐标
            costmap: 可选 Costmap 实例，用于代价感知的目标筛选。
                若为 None，策略仅依赖 occ_grid。

        Returns:
            (goal_x, goal_y) 元组，世界坐标系下的目标点。
            若无可探索目标，返回机器人当前位置（原地等待）。
        """
        raise NotImplementedError(
            "Subclasses must implement select_goal()")


class RandomWalkBaseline(BaselineStrategy):
    """随机游走探索基线（下界基准）。

    策略:
      - 在地图已知空闲区域内随机选取目标点
      - 机器人到达目标或被卡住时，随机选取新目标
      - 早期阶段（无已知空闲区域）在机器人附近随机方向选点

    理论说明:
      随机游走是最朴素的探索策略，不使用任何 frontier 检测或信息论
      推理。其期望覆盖率增长率为 O(sqrt(t))（二维随机游走的扩散特性），
      远低于 frontier-based 方法的 O(t) 线性增长。

      在评估中作为 **下界基准**（lower bound），用于量化 proposed
      方法相对于无策略探索的提升幅度。若 proposed 方法不显著优于
      随机游走，则说明其探索策略无实质价值。

    卡死检测:
      维护上一帧机器人位置，若连续多帧位移小于阈值，则判定为卡死，
      强制选取新随机目标。这模拟了 "撞墙后转向" 的随机游走行为。

    参考文献:
      - 对比基准标准做法，见 Thrun et al. (2005) "Probabilistic Robotics",
        Chapter 17 "Robotic Exploration".
    """

    def __init__(self, goal_tolerance=0.5, stuck_threshold=0.05,
                 stuck_patience=10, min_goal_dist=1.0, search_radius=4.0):
        """初始化随机游走策略。

        Args:
            goal_tolerance: 目标到达判定距离（米）
            stuck_threshold: 判定卡死的单帧最小位移（米）
            stuck_patience: 连续卡死多少帧后强制换目标
            min_goal_dist: 新目标距机器人的最小距离（米），避免原地打转
            search_radius: 早期无空闲区域时，在机器人附近搜索的半径（米）
        """
        self._goal_tolerance = goal_tolerance
        self._stuck_threshold = stuck_threshold
        self._stuck_patience = stuck_patience
        self._min_goal_dist = min_goal_dist
        self._search_radius = search_radius
        # 内部状态
        self._current_goal = None
        self._last_robot_pos = None
        self._stuck_count = 0
        self._rng = np.random.default_rng()

    def select_goal(self, occ_grid, robot_pos, costmap=None):
        rx, ry = robot_pos

        # 判断是否需要选取新目标
        need_new = self._should_pick_new_goal(rx, ry)

        # 更新卡死检测状态
        self._update_stuck_state(rx, ry)

        if need_new:
            self._current_goal = self._pick_random_free_goal(
                occ_grid, rx, ry, costmap)

        # 兜底：若仍无目标，返回机器人当前位置
        if self._current_goal is None:
            return (rx, ry)
        return self._current_goal

    def _should_pick_new_goal(self, rx, ry):
        """判断是否需要选取新目标（到达或卡死）。"""
        if self._current_goal is None:
            return True
        gx, gy = self._current_goal
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < self._goal_tolerance:
            return True
        if self._stuck_count >= self._stuck_patience:
            return True
        return False

    def _update_stuck_state(self, rx, ry):
        """更新卡死计数器。"""
        if self._last_robot_pos is not None:
            dx = rx - self._last_robot_pos[0]
            dy = ry - self._last_robot_pos[1]
            moved = math.hypot(dx, dy)
            if moved < self._stuck_threshold:
                self._stuck_count += 1
            else:
                self._stuck_count = 0
        self._last_robot_pos = (rx, ry)

    def _pick_random_free_goal(self, occ_grid, rx, ry, costmap=None):
        """在已知空闲区域内随机选取目标点。

        优先从 occ_grid 的已知空闲区域（log_odds < -0.5）中随机采样；
        若无空闲区域（探索早期），在机器人附近 search_radius 内随机选点；
        选取后会检查 costmap 代价，避免选到障碍物内。

        Returns:
            (goal_x, goal_y) 元组，或 None（无法选取时）
        """
        # 空闲区域掩码：log_odds < -0.5 表示观测到的空闲
        free_mask = occ_grid.log_odds < -0.5
        free_y, free_x = np.where(free_mask)

        if len(free_x) > 0:
            # 从已知空闲区域中随机采样
            idx = self._rng.integers(0, len(free_x))
            gx, gy = int(free_x[idx]), int(free_y[idx])
            wx, wy = occ_grid.grid_to_world(gx, gy)
            # 确保目标距机器人有最小距离，避免原地目标
            if math.hypot(wx - rx, wy - ry) < self._min_goal_dist:
                # 尝试多次采样
                for _ in range(10):
                    idx = self._rng.integers(0, len(free_x))
                    gx, gy = int(free_x[idx]), int(free_y[idx])
                    wx, wy = occ_grid.grid_to_world(gx, gy)
                    if math.hypot(wx - rx, wy - ry) >= self._min_goal_dist:
                        break
            # costmap 可达性检查
            if costmap is not None:
                if costmap.get_cost(wx, wy) >= 128:  # COST_INSCRIBED
                    # 不可达，随机选另一个
                    idx = self._rng.integers(0, len(free_x))
                    gx, gy = int(free_x[idx]), int(free_y[idx])
                    wx, wy = occ_grid.grid_to_world(gx, gy)
            return (wx, wy)

        # 无空闲区域：在机器人附近随机方向选点
        angle = self._rng.uniform(0, 2 * math.pi)
        dist = self._rng.uniform(0.5, self._search_radius)
        wx = rx + dist * math.cos(angle)
        wy = ry + dist * math.sin(angle)
        return (wx, wy)


class NearestFrontierBaseline(BaselineStrategy):
    """最近边界探索基线 (Yamauchi 1997)。

    策略:
      - 调用 ``occ_grid.find_frontiers(rx, ry)`` 检测边界
      - 选择距离机器人最近的 frontier 质心作为目标
      - 到达后选择下一个最近边界，直至无边界可探索

    理论说明:
      Frontier-based exploration 是自主探索的经典方法。Frontier 定义为
      已知空闲区域与未知区域的边界 —— 到达 frontier 意味着机器人将
      传感器视野扩展到未知区域，从而增长地图覆盖。

      "最近边界" 策略贪心地选择欧氏距离最近的 frontier，优点是：
        - 计算简单（O(n) frontier 检测 + 排序）
        - 实现直观，无超参数
      缺点是：
        - 不考虑 frontier 的大小（信息量）
        - 欧氏距离 ≠ 实际路径距离，可能选到不可达的 frontier
        - 容易在多个相近 frontier 间反复横跳

      尽管如此， nearest-frontier 仍是公认的 **强基线**，许多现代方法
      都以此为改进起点。

    参考文献:
      - Yamauchi, B. (1997). "A frontier-based approach for autonomous
        exploration." In Proceedings of the 1997 IEEE International
        Symposium on Computational Intelligence in Robotics and
        Automation (CIRA), pp. 146-151.
      - Yamauchi, B. (1998). "Frontier-based exploration using multiple
        robots." In Proceedings of the second international conference
        on Autonomous agents, pp. 47-53.
    """

    def __init__(self, goal_tolerance=0.5):
        """初始化最近边界策略。

        Args:
            goal_tolerance: 目标到达判定距离（米），用于决定何时切换目标
        """
        self._goal_tolerance = goal_tolerance
        self._current_goal = None

    def select_goal(self, occ_grid, robot_pos, costmap=None):
        rx, ry = robot_pos

        # 若有当前目标且未到达，保持目标（避免反复横跳）
        if self._current_goal is not None:
            gx, gy = self._current_goal
            if math.hypot(gx - rx, gy - ry) >= self._goal_tolerance:
                return self._current_goal

        # 检测 frontier（已按距离排序）
        frontiers = occ_grid.find_frontiers(rx, ry, max_frontiers=20)

        if frontiers:
            # frontiers[0] = (wx, wy, cluster_size)，选最近的
            wx, wy, _size = frontiers[0]
            self._current_goal = (wx, wy)
            return self._current_goal

        # 无 frontier：地图已探索完毕或刚开始，返回机器人位置
        self._current_goal = None
        return (rx, ry)


class GreedyInformationBaseline(BaselineStrategy):
    """贪心信息增益探索基线 (Bourgault 2002)。

    策略:
      - 对每个 frontier，计算其预期信息增益（基于 Shannon 熵）
      - 用 utility = α·info_gain - β·distance 作为评分
      - 选择 utility 最大的 frontier 作为目标

    理论说明:
      信息论探索方法将建图视为 "不确定性消除" 过程。每个栅格的占用
      概率 p 对应一个 Shannon 熵：

          H(p) = -p·log2(p) - (1-p)·log2(1-p)

      - 未知区域 p=0.5，H=1 bit（最大不确定性）
      - 已知空闲/占用 p≈0 或 1，H≈0（已确定）

      到达 frontier 后，传感器观测会消除其感知范围内的不确定性，
      因此 frontier 的信息增益 ≈ 其感知范围内未知/不确定区域的熵之和。

      Bourgault 等人提出的 utility 函数平衡了信息增益和移动代价：

          utility = α · I_norm - β · d_norm

      其中 I_norm 和 d_norm 分别是归一化的信息增益和距离。α、β 是
      权重超参数，反映 "探索价值 vs 旅行成本" 的 trade-off。

      与 nearest-frontier 相比，greedy-info 更倾向于访问信息丰富的
      大 frontier（如大房间入口），而非最近的小 frontier，因此通常
      能更快增长覆盖率。但计算复杂度更高（每个 frontier 需计算局部熵）。

    实现说明:
      - 直接使用 ``occ_grid.log_odds`` 计算 Shannon 熵（不依赖
        proposed 方法的 ``find_frontiers_with_info_gain``，保证独立性）
      - frontier 候选由 ``occ_grid.find_frontiers`` 提供（已聚类去噪）
      - 默认 α=1.0, β=0.5（Bourgault 2002 推荐配置）

    参考文献:
      - Bourgault, F., Makarenko, A. A., Williams, S. B., Grocholsky, B.,
        & Durrant-Whyte, H. F. (2002). "Information based adaptive robotic
        exploration." In Proceedings of the 2002 IEEE/RSJ International
        Conference on Intelligent Robots and Systems (IROS), Vol. 1,
        pp. 525-530.
      - Shannon, C. E. (1948). "A mathematical theory of communication."
        Bell System Technical Journal, 27(3), 379-423.
    """

    def __init__(self, alpha=1.0, beta=0.5, sensor_range=4.0,
                 goal_tolerance=0.5):
        """初始化贪心信息增益策略。

        Args:
            alpha: 信息增益权重 α（探索价值）
            beta: 距离代价权重 β（旅行成本）
            sensor_range: 传感器感知范围（米），用于估计信息增益
            goal_tolerance: 目标到达判定距离（米）
        """
        self._alpha = alpha
        self._beta = beta
        self._sensor_range = sensor_range
        self._goal_tolerance = goal_tolerance
        self._current_goal = None

    def select_goal(self, occ_grid, robot_pos, costmap=None):
        rx, ry = robot_pos

        # 若有当前目标且未到达，保持目标
        if self._current_goal is not None:
            gx, gy = self._current_goal
            if math.hypot(gx - rx, gy - ry) >= self._goal_tolerance:
                return self._current_goal

        # 获取 frontier 候选（已按距离排序）
        frontiers = occ_grid.find_frontiers(rx, ry, max_frontiers=20)

        if not frontiers:
            self._current_goal = None
            return (rx, ry)

        # 预计算整张栅格的 Shannon 熵
        # p = 1 / (1 + exp(-log_odds))，裁剪以保证数值稳定
        lo = np.clip(occ_grid.log_odds, -10.0, 10.0)
        p = 1.0 / (1.0 + np.exp(-lo))
        eps = 1e-12
        p_safe = np.clip(p, eps, 1.0 - eps)
        cell_entropy = (
            -p_safe * np.log2(p_safe) - (1.0 - p_safe) * np.log2(1.0 - p_safe)
        )

        sensor_cells = int(self._sensor_range / occ_grid.resolution)

        # 计算每个 frontier 的信息增益和距离
        gains = []
        for wx, wy, _size in frontiers:
            fgx, fgy = occ_grid.world_to_grid(wx, wy)
            # 感知范围内的 bounding box
            x0 = max(0, fgx - sensor_cells)
            x1 = min(occ_grid.width, fgx + sensor_cells + 1)
            y0 = max(0, fgy - sensor_cells)
            y1 = min(occ_grid.height, fgy + sensor_cells + 1)
            # 圆形感知掩码
            yy, xx = np.mgrid[y0:y1, x0:x1]
            mask = (xx - fgx) ** 2 + (yy - fgy) ** 2 <= sensor_cells ** 2
            local_entropy = cell_entropy[y0:y1, x0:x1]
            info_gain = float(np.sum(local_entropy * mask))
            dist = math.hypot(wx - rx, wy - ry)
            gains.append((info_gain, dist, wx, wy))

        # 归一化并计算 utility
        max_gain = max((g[0] for g in gains), default=1.0)
        max_dist = max((g[1] for g in gains), default=1.0)
        if max_gain <= 0.0:
            max_gain = 1.0
        if max_dist <= 0.0:
            max_dist = 1.0

        best_utility = -float('inf')
        best_goal = None
        for info_gain, dist, wx, wy in gains:
            gain_norm = info_gain / max_gain
            dist_norm = dist / max_dist
            utility = self._alpha * gain_norm - self._beta * dist_norm
            if utility > best_utility:
                best_utility = utility
                best_goal = (wx, wy)

        if best_goal is None:
            best_goal = (rx, ry)

        self._current_goal = best_goal
        return best_goal


class OracleBaseline(BaselineStrategy):
    """全知最优探索基线（理论上界）。

    策略:
      - 假设已知全局地图（ground truth）
      - 直接选择最远的未访问区域作为目标
      - 理论上给出探索效率的上界

    理论说明:
      Oracle 基线是一种 **理想化策略**，假设机器人拥有环境的完整先验
      知识。它贪心地选择 "最远未访问点" 作为目标，使得每次移动都
      最大化空间覆盖的增长。

      在实际系统中 Oracle 不可达（因为未知区域本身就是探索目标），
      但它提供了探索效率的 **理论上界**（upper bound）：
        - proposed 方法的覆盖率曲线应渐近趋近 Oracle
        - 若 proposed 显著低于 Oracle，说明仍有改进空间
        - 若 proposed 接近 Oracle，说明策略已接近最优

    实现说明:
      由于本基线与 proposed 方法共享同一 OccupancyGrid 接口（无独立
      ground truth），此处采用以下近似：
        - 使用 ``occ_grid.visited`` 标记判断 "已访问"
        - 使用 ``occ_grid.log_odds`` 判断 "未占用"（free 或 unknown）
        - 在所有 "未访问且未占用" 的栅格中，选取距机器人最远者
      这近似了 "去最远未探索区域" 的 Oracle 行为。

      注意：真正的 Oracle 会使用环境的真实 free space（含未知但实际
      可通行区域），而本近似受限于已观测信息，因此实际性能略低于
      理论上界。但作为对比基准已足够。

    参考文献:
      - 上界基准的标准做法，见 Hollinger et al. (2014) "Autonomous
        robotic information gathering for ocean monitoring."
        Journal of Field Robotics, 31(5), 733-753.
    """

    def __init__(self, goal_tolerance=0.5, sample_stride=3):
        """初始化 Oracle 策略。

        Args:
            goal_tolerance: 目标到达判定距离（米）
            sample_stride: 栅格采样步长（每隔 N 个栅格采样一次，加速）
        """
        self._goal_tolerance = goal_tolerance
        self._sample_stride = sample_stride
        self._current_goal = None

    def select_goal(self, occ_grid, robot_pos, costmap=None):
        rx, ry = robot_pos

        # 若有当前目标且未到达，保持目标
        if self._current_goal is not None:
            gx, gy = self._current_goal
            if math.hypot(gx - rx, gy - ry) >= self._goal_tolerance:
                return self._current_goal

        # 未占用掩码：free 或 unknown（非 occupied）
        # log_odds > 0.6 视为 occupied（与 OccupancyGrid.is_occupied 一致）
        non_occupied = occ_grid.log_odds <= 0.6
        # 未访问掩码
        unvisited = ~occ_grid.visited
        # 候选目标：未占用且未访问
        candidate_mask = non_occupied & unvisited

        if not candidate_mask.any():
            # 全部已访问或占用 —— 探索完成
            self._current_goal = None
            return (rx, ry)

        # 采样以加速（每隔 stride 个栅格取一个候选）
        cand_y, cand_x = np.where(candidate_mask)
        if self._sample_stride > 1 and len(cand_x) > 1000:
            cand_x = cand_x[::self._sample_stride]
            cand_y = cand_y[::self._sample_stride]

        # 计算各候选到机器人的栅格距离
        robot_gx, robot_gy = occ_grid.world_to_grid(rx, ry)
        dists = np.sqrt(
            (cand_x.astype(np.float64) - robot_gx) ** 2
            + (cand_y.astype(np.float64) - robot_gy) ** 2
        )

        # 选最远候选
        idx = int(np.argmax(dists))
        gx, gy = int(cand_x[idx]), int(cand_y[idx])
        wx, wy = occ_grid.grid_to_world(gx, gy)
        self._current_goal = (wx, wy)
        return self._current_goal


class BaselineFactory:
    """基线策略工厂。

    通过名称字符串创建对应的基线策略实例，便于在实验配置中
    通过 YAML 指定基线方法。

    用法:
        strategy = BaselineFactory.create('nearest_frontier')
        goal = strategy.select_goal(occ_grid, robot_pos)
    """

    # 名称 -> 类的映射
    _REGISTRY = {
        'random_walk': RandomWalkBaseline,
        'nearest_frontier': NearestFrontierBaseline,
        'greedy_info': GreedyInformationBaseline,
        'oracle': OracleBaseline,
    }

    @classmethod
    def create(cls, name):
        """根据名称创建基线策略实例。

        Args:
            name: 基线名称，可选值：
                - 'random_walk'      : 随机游走（下界）
                - 'nearest_frontier' : 最近边界 (Yamauchi 1997)
                - 'greedy_info'      : 贪心信息增益 (Bourgault 2002)
                - 'oracle'           : 全知最优（上界）

        Returns:
            BaselineStrategy 子类实例

        Raises:
            ValueError: 未知基线名称
        """
        if name not in cls._REGISTRY:
            valid = ', '.join(sorted(cls._REGISTRY.keys()))
            raise ValueError(
                f"Unknown baseline '{name}'. Valid options: {valid}")
        return cls._REGISTRY[name]()

    @classmethod
    def available_names(cls):
        """返回所有可用的基线名称列表。"""
        return sorted(cls._REGISTRY.keys())


if __name__ == '__main__':
    # 简单烟雾测试：验证各基线可正常实例化和调用
    print("[baselines] Smoke test started")
    grid = OccupancyGrid()
    # 模拟一些观测：在机器人周围标记一小片 free 区域
    for i in range(20, 40):
        for j in range(20, 40):
            grid.log_odds[j, i] = -1.0  # free
    # 模拟一些 unknown（默认 log_odds=0 即 unknown）
    robot = (0.0, 0.0)

    for name in BaselineFactory.available_names():
        strat = BaselineFactory.create(name)
        goal = strat.select_goal(grid, robot)
        print(f"  {name:20s} -> goal=({goal[0]:.2f}, {goal[1]:.2f})")
    print("[baselines] Smoke test passed")
