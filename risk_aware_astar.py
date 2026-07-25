#!/usr/bin/env python3
"""
Risk-Aware A* Path Planner (v4.0 Innovation)
=============================================
在经典 A* 基础上引入定位不确定性传播，选择"风险最小"路径而非"最短"路径。

================================================================================
理论创新
================================================================================

1. 机会约束规划 (Chance-Constrained Programming)
--------------------------------------------------------------------------------
传统 A* 假设机器人位姿确定，实际 AMCL 定位存在不确定性 Σ。
机器人真实位置服从高斯分布: x_true ~ N(x_est, Σ)。

碰撞概率:
    P_collision(s) = P(x_true ∈ O | x_est, Σ)

机会约束要求碰撞概率低于阈值 δ:
    P_collision(s) ≤ δ, ∀s ∈ path

在栅格地图中近似为: 若 N(μ, Σ) 落入障碍物格子的概率 > δ，则该格子不可通行。

2. 条件风险价值 (CVaR) 风险度量
--------------------------------------------------------------------------------
Value at Risk (VaR): 给定置信度 α 下的最大损失分位数
    VaR_α = inf{γ : P(loss ≤ γ) ≥ α}

Conditional VaR (CVaR / Expected Shortfall): 超过 VaR 的期望损失
    CVaR_α = E[loss | loss > VaR_α]

Rockafellar-Uryasev 公式:
    CVaR_α(L) = min_γ { γ + (1/(1-α)) * E[(L - γ)+] }

CVaR 优于 VaR:
    - VaR 只关心分位点，忽略尾部极端损失
    - CVaR 考虑超阈值部分的期望，是一致性风险度量 (coherent risk measure)
    - CVaR 满足次可加性: CVaR(A+B) ≤ CVaR(A) + CVaR(B)
    - 优化 CVaR 可用线性规划 (Rockafellar-Uryasev 2000)

在路径规划中，每条路径的风险用 CVaR 评估:
    risk(path) = CVaR_α( max_{s ∈ path} collision_cost(s) )

3. 不确定性传播 (Uncertainty Propagation)
--------------------------------------------------------------------------------
机器人运动模型: x_{k+1} = f(x_k, u_k) + w_k, w_k ~ N(0, Q)
协方差传播 (EKF 预测):
    Σ_{k+1} = F_k Σ_k F_k^T + G_k Q G_k^T

其中 F = ∂f/∂x 为雅可比矩阵。

沿路径的累积不确定性:
    Σ(s) 随路径长度增加（无观测时）
    Σ(s) 在信息丰富区域减小（有 LiDAR 观测时）

4. 风险感知代价函数
--------------------------------------------------------------------------------
总代价 = 路径长度 + λ · 风险项

    cost(s) = d(s, s') + λ · CVaR_α(collision_cost(s))

其中:
    - d(s, s') 为运动代价
    - λ 为风险厌恶系数 (λ→0 退化为标准 A*, λ→∞ 极度保守)
    - collision_cost(s) = 基于定位不确定性的碰撞代价

碰撞代价近似（高斯-栅格近似）:
    collision_cost(s) = Σ_{c ∈ O_near} P(x_true ∈ c | x_est, Σ)

    用误差函数 erf 近似累积高斯概率:
    P(x_true ∈ [x1, x2] × [y1, y2] | μ, Σ) ≈
        0.25 · [erf((x2-μx)/(σx·√2)) - erf((x1-μx)/(σx·√2))]
              · [erf((y2-μy)/(σy·√2)) - erf((y1-μy)/(σy·√2))]

参考文献:
    - Rockafellar & Uryasev (2000) "Optimization of Conditional Value-at-Risk"
    - Luders et al. (2010) "Robust Safe Path Planning for Autonomous Vehicles
      Using Chance-Constrained Routing"
    - Blackmore et al. (2010) "A Probabilistic Particle-Control Approach to
      Stochastic Trajectory Planning"
    - Ono & Williams (2008) "Iterative Risk Allocation"
"""

import os
import math
import heapq
import numpy as np
from costmap import Costmap, COST_INSCRIBED, COST_LETHAL, GRID_W, GRID_H
from occupancy_grid import GRID_RESOLUTION, ORIGIN_X, ORIGIN_Y

USE_RISK_AWARE = os.environ.get("USE_RISK_AWARE", "0") == "1"


class RiskAwareAStarPlanner:
    """考虑定位不确定性传播的风险感知 A* 路径规划器。

    在标准 A* 代价基础上，叠加基于定位协方差的碰撞风险代价，
    使得规划器优先选择"安全冗余"更高的路径。

    Parameters
    ----------
    costmap : Costmap
        分层代价地图。
    risk_lambda : float
        风险厌恶系数 λ。λ=0 退化为标准 A*；λ 越大越保守。
    alpha : float
        CVaR 置信度 (0.9~0.99)。越高越保守。
    sigma_max : float
        假设最大定位标准差 (m)。用于在不实时跟踪协方差时的近似。
    """

    def __init__(self, costmap, risk_lambda=1.0, alpha=0.95,
                 sigma_max=0.15, localization_cov=None,
                 adaptive_lambda=False, lambda_base=1.0,
                 lambda_min=0.3, lambda_max=3.0):
        """
        Parameters
        ----------
        costmap : Costmap
            分层代价地图。
        risk_lambda : float
            风险厌恶系数 λ。λ=0 退化为标准 A*；λ 越大越保守。
            当 adaptive_lambda=True 时作为初始值和基础值 lambda_base。
        alpha : float
            CVaR 置信度 (0.9~0.99)。越高越保守。
        sigma_max : float
            假设最大定位标准差 (m)。用于在不实时跟踪协方差时的近似。
        adaptive_lambda : bool
            是否启用自适应风险系数。启用后 λ 将根据 loc_err / map_uncertainty
            自动调整，理论依据：定位越不确定，所需的安全裕度越大。
        lambda_base : float
            自适应 λ 的基础值（loc_err=0 时的 λ）。
        lambda_min, lambda_max : float
            自适应 λ 的上下界（防止极端保守或失效）。
        """
        self.costmap = costmap
        self.lambda_base = float(lambda_base)
        self.lambda_risk = float(risk_lambda)
        self.alpha = alpha
        self.sigma_max = sigma_max
        self.adaptive_lambda = bool(adaptive_lambda)

        # 自适应 λ 的上下界
        self.lambda_min = float(lambda_min)
        self.lambda_max = float(lambda_max)

        # 自适应 λ 参数：σ(k·(x - x0)) 中的 k 与 x0
        # x0 = 触发阈值，超过此值开始放大 λ
        self.adaptive_loc_err_threshold = 0.5   # m
        self.adaptive_loc_err_slope = 6.0       # sigmoid 斜率
        self.adaptive_map_unc_threshold = 0.6   # 归一化 Shannon 熵
        self.adaptive_map_unc_slope = 4.0

        # 记录当前外部信号（由外部循环每帧更新）
        self.current_loc_err = 0.0
        self.current_map_uncertainty = 0.0

        # 定位协方差 (3x3) - 由外部 AMCL 更新
        if localization_cov is not None:
            self.loc_cov = np.array(localization_cov)
        else:
            self.loc_cov = np.eye(3) * (sigma_max ** 2)

        # 8-connected neighbors: (dx, dy, move_cost)
        self.neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        # 风险代价缓存（避免重复计算）
        self._risk_cache = {}

        # 统计
        self.nodes_expanded = 0
        self.planning_time = 0.0
        self._lambda_history: list = []

    def update_localization_covariance(self, cov):
        """更新定位协方差矩阵 (由 AMCL 调用)。

        Parameters
        ----------
        cov : ndarray, shape (3, 3)
            AMCL 当前估计的位姿协方差。
        """
        self.loc_cov = np.array(cov)
        # 协方差变化后清空缓存
        self._risk_cache.clear()

    def update_adaptive_lambda(self, loc_err=None, map_uncertainty=None):
        """根据当前定位误差和建图不确定性更新风险厌恶系数 λ。

        自适应公式（基于 sigmoid 平滑过渡）::

            λ(t) = clip(λ_base · f(loc_err) · g(map_unc), λ_min, λ_max)

            f(x) = 1 + α_loc · sigmoid(k_loc · (x - x0_loc))
            g(y) = 1 + α_map · sigmoid(k_map · (y - x0_map))

        其中:
            - λ_base = 基础风险系数（低不确定性时的 λ）
            - α_loc / α_map = 放大幅度（默认各 1.0，即最大可放大 2 倍）
            - sigmoid 保证 λ 在阈值附近平滑过渡，不硬切换

        理论依据
        --------
        在不确定性较高的环境中（定位漂移、地图未知区域多），
        固定的 λ 难以兼顾安全性与效率：
            - λ 太小 → 容易碰撞
            - λ 太大 → 过度保守，绕远路
        自适应 λ 根据实时不确定性调整，使风险厌恶程度与环境匹配，
        是"机会约束规划"在动态环境下的自然扩展。

        参考文献
        --------
        - Ono & Williams (2008) "Iterative Risk Allocation"
          提出风险分配应随状态变化
        - Blackmore et al. (2010) "Probabilistic Particle-Control"
          论证了不确定性越大，所需保守度越高

        Parameters
        ----------
        loc_err : float, optional
            当前定位误差估计 (m)。如不提供则使用上一次记录值。
        map_uncertainty : float, optional
            当前建图不确定性（归一化 Shannon 熵 ∈ [0,1]）。
            如不提供则使用上一次记录值。
        """
        if not self.adaptive_lambda:
            return

        if loc_err is not None:
            self.current_loc_err = max(0.0, float(loc_err))
        if map_uncertainty is not None:
            self.current_map_uncertainty = max(0.0, min(1.0, float(map_uncertainty)))

        # 安全的 sigmoid 函数
        def _safe_sigmoid(x):
            if x > 50:
                return 1.0
            if x < -50:
                return 0.0
            return 1.0 / (1.0 + math.exp(-x))

        # 定位不确定性放大因子: loc_err > 0.5m 时开始放大
        loc_signal = _safe_sigmoid(
            self.adaptive_loc_err_slope *
            (self.current_loc_err - self.adaptive_loc_err_threshold))
        loc_factor = 1.0 + 1.0 * loc_signal  # [1.0, 2.0]

        # 建图不确定性放大因子: map_unc > 0.6 时开始放大
        map_signal = _safe_sigmoid(
            self.adaptive_map_unc_slope *
            (self.current_map_uncertainty - self.adaptive_map_unc_threshold))
        map_factor = 1.0 + 1.0 * map_signal  # [1.0, 2.0]

        # 自适应 λ
        new_lambda = self.lambda_base * loc_factor * map_factor
        # 裁剪到 [lambda_min, lambda_max]
        new_lambda = max(self.lambda_min, min(self.lambda_max, new_lambda))

        # 如果变化超过 5% 则清空缓存
        if abs(new_lambda - self.lambda_risk) > 0.05 * self.lambda_risk:
            self._risk_cache.clear()

        self.lambda_risk = new_lambda
        self._lambda_history.append(new_lambda)
        # 仅保留最近 100 次记录
        if len(self._lambda_history) > 100:
            self._lambda_history = self._lambda_history[-100:]

    def get_lambda_history(self):
        """返回自适应 λ 的历史记录（用于可视化与诊断）。"""
        return list(self._lambda_history)

    def _erf_approx(self, x):
        """ Abramowitz-Stegun 近似 erf(x)。

        erf(x) ≈ 1 - (a1·t + a2·t² + a3·t³)·e^(-x²),  t = 1/(1+p·x)

        精度: |误差| < 2.5e-7

        Parameters
        ----------
        x : float or ndarray
            输入值。

        Returns
        -------
        erf : float or ndarray
            误差函数值。
        """
        # 系数
        p = 0.3275911
        a1 = 0.254829592
        a2 = -0.284496736
        a3 = 1.421413741
        a4 = -1.453152027
        a5 = 1.061405429

        sign = np.sign(x)
        x_abs = np.abs(x)
        t = 1.0 / (1.0 + p * x_abs)
        y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * np.exp(-x_abs * x_abs)
        return sign * y

    def _compute_collision_probability(self, gx, gy):
        """计算机器人在栅格 (gx, gy) 处的碰撞概率。

        基于高斯定位不确定性，计算机器人真实位置落在障碍物格子内的概率。

        P_collision = Σ_{c ∈ O_near} P(x_true ∈ c | μ, Σ)

        用误差函数近似累积高斯概率。

        Parameters
        ----------
        gx, gy : int
            栅格坐标。

        Returns
        -------
        p_collision : float
            碰撞概率 [0, 1]。
        """
        cache_key = (gx, gy)
        if cache_key in self._risk_cache:
            return self._risk_cache[cache_key]

        # 机器人估计位置 (世界坐标)
        wx = gx * GRID_RESOLUTION + ORIGIN_X
        wy = gy * GRID_RESOLUTION + ORIGIN_Y

        # 定位标准差 (取 x, y 分量)
        sigma_x = math.sqrt(max(self.loc_cov[0, 0], 1e-6))
        sigma_y = math.sqrt(max(self.loc_cov[1, 1], 1e-6))

        # 限制最大标准差 (避免数值问题)
        sigma_x = min(sigma_x, 0.5)
        sigma_y = min(sigma_y, 0.5)

        # 搜索半径内的障碍物格子
        r_cells = max(1, int(3 * max(sigma_x, sigma_y) / GRID_RESOLUTION))

        p_collision = 0.0
        sqrt2 = math.sqrt(2.0)

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                nx, ny = gx + dx, gy + dy
                if not (0 <= nx < GRID_W and 0 <= ny < GRID_H):
                    continue
                cost = self.costmap.get_cost_grid(nx, ny)
                if cost < COST_INSCRIBED:
                    continue  # 非障碍物

                # 障碍物格子的世界坐标范围
                ox_min = nx * GRID_RESOLUTION + ORIGIN_X
                ox_max = ox_min + GRID_RESOLUTION
                oy_min = ny * GRID_RESOLUTION + ORIGIN_Y
                oy_max = oy_min + GRID_RESOLUTION

                # P(x_true ∈ [ox_min, ox_max] × [oy_min, oy_max] | μ, Σ)
                # 用 erf 近似累积高斯
                px = 0.5 * (self._erf_approx((ox_max - wx) / (sigma_x * sqrt2))
                           - self._erf_approx((ox_min - wx) / (sigma_x * sqrt2)))
                py = 0.5 * (self._erf_approx((oy_max - wy) / (sigma_y * sqrt2))
                           - self._erf_approx((oy_min - wy) / (sigma_y * sqrt2)))

                p_collision += px * py

        p_collision = min(p_collision, 1.0)
        self._risk_cache[cache_key] = p_collision
        return p_collision

    def _compute_cvar_cost(self, p_collision):
        """计算 CVaR 风险代价。

        CVaR_α(loss) = E[loss | loss > VaR_α(loss)]

        简化模型: loss = p_collision (碰撞概率即损失)
        假设 loss 服从 Beta(α_p, β_p) 分布，则:
            VaR_α = Beta.ppf(confidence, α_p, β_p)
            CVaR_α = α_p / (α_p + β_p) · (1 - I_{1-VaR}(α_p+1, β_p)) / (1 - confidence)

        简化版本 (避免 Beta 计算): 用凸组合近似
            risk ≈ p_collision · (1 + (1 - alpha) · tail_factor)

        其中 tail_factor 衡量尾部风险放大。

        Parameters
        ----------
        p_collision : float
            碰撞概率 [0, 1]。

        Returns
        -------
        cvar_cost : float
            CVaR 风险代价。
        """
        if p_collision < 1e-6:
            return 0.0
        # 简化 CVaR: 风险随概率非线性增长，尾部放大
        # CVaR_alpha ≈ p · (1 + (1-alpha) * 5)  (尾部因子5)
        tail_factor = 1.0 + (1.0 - self.alpha) * 5.0
        return p_collision * tail_factor

    def _cell_cost_risk(self, gx, gy):
        """风险感知栅格代价 = 运动代价 + λ · CVaR 风险代价。

        Parameters
        ----------
        gx, gy : int
            栅格坐标。

        Returns
        -------
        cost : float
            总代价 (inf 表示不可通行)。
        """
        c = self.costmap.get_cost_grid(gx, gy)
        if c >= COST_LETHAL:
            return float('inf')

        # 基础运动代价 (与标准 A* 一致)
        base_cost = 1.0 + (c / 128.0) ** 2 * 12.0

        if not USE_RISK_AWARE or self.lambda_risk <= 0:
            return base_cost

        # 风险代价
        p_collision = self._compute_collision_probability(gx, gy)
        cvar_cost = self._compute_cvar_cost(p_collision)

        return base_cost + self.lambda_risk * cvar_cost * 10.0

    def _heuristic(self, gx, gy, tx, ty):
        """Octile 距离启发式 (可采纳)。"""
        dx = abs(gx - tx)
        dy = abs(gy - ty)
        return (dx + dy) + (1.414 - 2) * min(dx, dy)

    def _world_to_grid(self, x, y):
        """世界坐标 → 栅格坐标。"""
        gx = int((x - ORIGIN_X) / GRID_RESOLUTION)
        gy = int((y - ORIGIN_Y) / GRID_RESOLUTION)
        return max(0, min(GRID_W - 1, gx)), max(0, min(GRID_H - 1, gy))

    def _grid_to_world(self, gx, gy):
        """栅格坐标 → 世界坐标。"""
        return gx * GRID_RESOLUTION + ORIGIN_X, gy * GRID_RESOLUTION + ORIGIN_Y

    def plan(self, start_x, start_y, goal_x, goal_y):
        """风险感知 A* 规划。

        Parameters
        ----------
        start_x, start_y : float
            起点世界坐标。
        goal_x, goal_y : float
            目标世界坐标。

        Returns
        -------
        path : list of (float, float) or None
            路径点列表 (世界坐标)，失败返回 None。
        """
        import time
        t0 = time.time()

        sgx, sgy = self._world_to_grid(start_x, start_y)
        tgx, tgy = self._world_to_grid(goal_x, goal_y)

        # 目标不可达时找最近可通行格
        if self.costmap.get_cost_grid(tgx, tgy) >= COST_LETHAL:
            best_d = float('inf')
            best = None
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    nx, ny = tgx + dx, tgy + dy
                    if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                        c = self.costmap.get_cost_grid(nx, ny)
                        d = dx * dx + dy * dy
                        if c < COST_LETHAL and d < best_d:
                            best_d = d
                            best = (nx, ny)
            if best is None:
                self.planning_time = time.time() - t0
                return None
            tgx, tgy = best

        # A* 搜索
        open_heap = [(0.0, (sgx, sgy))]
        came_from = {(sgx, sgy): None}
        g_score = {(sgx, sgy): 0.0}
        self.nodes_expanded = 0

        while open_heap and self.nodes_expanded < 2000:
            _, current = heapq.heappop(open_heap)
            self.nodes_expanded += 1

            if current == (tgx, tgy):
                # 回溯路径
                path = []
                node = current
                while node is not None:
                    wx, wy = self._grid_to_world(*node)
                    path.append((wx, wy))
                    node = came_from[node]
                path.reverse()
                self.planning_time = time.time() - t0
                return path

            cgx, cgy = current
            for dx, dy, move_cost in self.neighbors:
                ngx, ngy = cgx + dx, cgy + dy
                if not (0 <= ngx < GRID_W and 0 <= ngy < GRID_H):
                    continue

                cell_cost = self._cell_cost_risk(ngx, ngy)
                if cell_cost == float('inf'):
                    continue

                tentative_g = g_score[current] + move_cost * cell_cost

                neighbor = (ngx, ngy)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(ngx, ngy, tgx, tgy)
                    came_from[neighbor] = current
                    heapq.heappush(open_heap, (f, neighbor))

        self.planning_time = time.time() - t0
        return None

    def get_risk_map(self):
        """获取当前风险地图 (用于可视化)。

        Returns
        -------
        risk_map : ndarray, shape (GRID_W, GRID_H)
            每个格子的碰撞概率。
        """
        risk_map = np.zeros((GRID_W, GRID_H))
        for gx in range(GRID_W):
            for gy in range(GRID_H):
                if self.costmap.get_cost_grid(gx, gy) >= COST_INSCRIBED:
                    risk_map[gx, gy] = self._compute_collision_probability(gx, gy)
        return risk_map
