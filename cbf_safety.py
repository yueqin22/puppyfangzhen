"""
控制障碍函数(CBF)安全模块

用控制障碍函数(Control Barrier Function)保证导航安全性。

================================================================================
理论贡献
================================================================================

1. CBF 前向不变性理论 (Forward Invariance)
--------------------------------------------------------------------------------
对障碍物集合 {O_i}，定义控制障碍函数(CBF):
    h_i(x) = dist(x, O_i) - d_safe

安全集合定义为:
    C = { x : h_i(x) >= 0,  ∀i }

前向不变性 (Forward Invariance) 定理:
    若存在控制 u 使得对所有 i:
        L_f h_i(x) + L_g h_i(x) * u >= -α * h_i(x)
    即  ḣ_i(x) >= -α * h_i(x)
    则安全集合 C 是前向不变的——
    即若 x(0) ∈ C，则 x(t) ∈ C, ∀t >= 0。

证明思路:
    由 ḣ_i >= -α * h_i 和比较引理 (Comparison Lemma):
        h_i(x(t)) >= h_i(x(0)) * e^{-α*t} >= 0  (当 h_i(x(0)) >= 0)
    因此 h_i 始终非负，系统始终安全。

其中:
    L_f h_i = ∇h_i · f(x)    (李导数，漂移项)
    L_g h_i = ∇h_i · g(x)    (控制方向上的导数)
    α > 0 为安全裕度参数

2. QP 安全过滤器可行性条件
--------------------------------------------------------------------------------
QP (Quadratic Program) 安全过滤器:
    min_{u}  ||u - u_desired||²
    s.t.     L_g h_i(x) * u >= -L_f h_i(x) - α * h_i(x),  ∀i

可行性条件:
    - 必须存在 u 使得所有CBF约束同时满足
    - 若系统在安全集合内部 (h_i > 0, ∀i)，QP 几乎总有解
    - 若接近边界，可能需要松弛约束（增大 α 或引入松弛变量）
    - 松弛形式:
        min_{u, δ}  ||u - u_desired||² + M * δ²
        s.t.        L_g h_i(x) * u >= -L_f h_i(x) - α * h_i(x) - δ,  ∀i
                    δ >= 0
      其中 M >> 1 为大惩罚系数，保证优先满足安全性

    - 当 L_g h_i 行满秩时，约束矩阵 A = [L_g h_1; ...; L_g h_n] 行满秩
      约束可行的充要条件是冲突约束不在同一方向

3. 与 TEB 局部规划器的集成方式
--------------------------------------------------------------------------------
TEB (Timed Elastic Band) 生成局部轨迹，CBF 作为安全过滤器:

    TEB 输出 u_desired → CBF安全过滤 → u_safe → 执行

集成方式:
    a) 后处理式 (Post-processing):
       TEB生成期望速度 v_des, ω_des
       CBF对 (v_des, ω_des) 做QP投影得到安全 (v_safe, ω_safe)
       优点: 不修改TEB内部，即插即用

    b) 约束嵌入式 (Embedded):
       将CBF约束作为硬约束加入TEB的优化问题
       TEB QP 问题中增加约束: h_i(x_k) >= 0 对所有时间步 k
       优点: 更优的轨迹，但修改TEB内部

    c) 混合式 (Hybrid):
       TB正常规划，CBF仅在危险时介入
       根据 risk_level 决定是否激活安全过滤器
       平衡安全性与机动性

================================================================================
数学公式
================================================================================
- CBF定义:
      h_i(x) = dist(x, O_i) - d_safe

- 安全条件:
      h_i(x) >= 0,  ∀i

- QP安全过滤器:
      u_safe = argmin_u ||u - u_des||²
      s.t.   ∇h_i · f(x,u) + α * h_i(x) >= 0,  ∀i

- 碰撞时间 (TTC):
      TTC = h_min / |v_rel|  (最近CBF值除以相对速度)
"""

import numpy as np


class CBFSafety:
    """
    控制障碍函数(CBF)安全过滤器

    对每个障碍物定义CBF h_i(x) = dist(x, O_i) - d_safe，
    通过QP求解在所有安全约束下最接近期望控制的安全控制。

    Parameters
    ----------
    robot_radius : float
        机器人半径 (m)。
    d_safe : float
        安全距离阈值 (m)，h_i >= 0 时安全。
    d_critical : float
        临界距离阈值 (m)，低于此距离触发紧急制动。

    Attributes
    ----------
    alpha : float
        CBF安全裕度参数 α > 0，控制收敛到边界的速度。
    max_speed : float
        最大线速度上限。
    """

    def __init__(self, robot_radius, d_safe, d_critical):
        """
        初始化CBF安全过滤器。

        Parameters
        ----------
        robot_radius : float
            机器人半径 (m)。
        d_safe : float
            安全距离 (m)。
        d_critical : float
            临界距离 (m)。
        """
        self.robot_radius = robot_radius
        self.d_safe = d_safe
        self.d_critical = d_critical

        # CBF安全裕度参数
        self.alpha = 2.0  # α > 0，越大越保守
        # 最大速度（用于TTC和自适应安全距离）
        self.max_speed = 1.5  # m/s
        # QP松弛惩罚系数
        self._slack_penalty = 1000.0

    def _point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """
        计算点 (px, py) 到线段 ((x1,y1) -> (x2,y2)) 的距离。

        Parameters
        ----------
        px, py : float
            点坐标。
        x1, y1, x2, y2 : float
            线段端点坐标。

        Returns
        -------
        dist : float
            最短距离。
        """
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq < 1e-12:
            # 退化为点
            return np.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        # 投影参数 t ∈ [0, 1]
        t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))

        # 投影点
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy

        return np.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)

    def _compute_distance_to_obstacle(self, x, y, obstacle):
        """
        计算点到单个障碍物的距离（减去机器人半径和障碍物半径）。

        障碍物格式:
            - 圆形: {'type': 'circle', 'center': (cx, cy), 'radius': r}
            - 线段: {'type': 'segment', 'p1': (x1,y1), 'p2': (x2,y2), 'radius': r}
            - 点:   {'type': 'point', 'position': (cx, cy), 'radius': r}

        Parameters
        ----------
        x, y : float
            机器人位置。
        obstacle : dict
            障碍物描述。

        Returns
        -------
        dist : float
            有效距离（表面到表面）。
        obs_radius : float
            障碍物等效半径。
        cx, cy : float
            最近点坐标（用于梯度计算）。
        """
        obs_type = obstacle.get('type', 'circle')

        if obs_type == 'circle' or obs_type == 'point':
            if obs_type == 'circle':
                cx, cy = obstacle['center']
                r = obstacle.get('radius', 0.0)
            else:
                cx, cy = obstacle['position']
                r = 0.0
            dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
            effective_dist = dist - self.robot_radius - r
            return effective_dist, r, cx, cy

        elif obs_type == 'segment':
            x1, y1 = obstacle['p1']
            x2, y2 = obstacle['p2']
            r = obstacle.get('radius', 0.0)
            dist = self._point_to_segment_distance(x, y, x1, y1, x2, y2)
            effective_dist = dist - self.robot_radius - r
            return effective_dist, r, x, y  # 最近点需要单独计算

        else:
            raise ValueError(f"未知障碍物类型: {obs_type}")

    def compute_cbf(self, x, y, obstacles):
        """
        计算所有障碍物的CBF值。

        h_i(x) = dist(x, O_i) - d_safe

        Parameters
        ----------
        x, y : float
            机器人位置。
        obstacles : list of dict
            障碍物列表。

        Returns
        -------
        h_values : list of float
            每个障碍物的CBF值。h_i >= 0 表示安全。
        """
        h_values = []
        for obs in obstacles:
            dist, _, _, _ = self._compute_distance_to_obstacle(x, y, obs)
            h = dist - self.d_safe
            h_values.append(h)
        return h_values

    def compute_cbf_gradient(self, x, y, obstacle):
        """
        计算CBF对位置的梯度 ∇h = (∂h/∂x, ∂h/∂y)。

        h_i(x,y) = dist(x, y, O_i) - d_safe
        ∇h_i = (∂dist/∂x, ∂dist/∂y) = (x - cx, y - cy) / dist

        梯度方向为远离障碍物方向。

        Parameters
        ----------
        x, y : float
            机器人位置。
        obstacle : dict
            单个障碍物。

        Returns
        -------
        grad : tuple (dh/dx, dh/dy)
            CBF梯度。
        """
        dist, _, cx, cy = self._compute_distance_to_obstacle(x, y, obstacle)

        if dist < 1e-10:
            # 避免除零，返回远离方向
            dx = x - cx
            dy = y - cy
            norm = np.sqrt(dx ** 2 + dy ** 2)
            if norm < 1e-10:
                return (0.0, 0.0)
            return (dx / norm, dy / norm)

        dh_dx = (x - cx) / (dist + self.robot_radius +
                            obstacle.get('radius', 0.0))
        dh_dy = (y - cy) / (dist + self.robot_radius +
                            obstacle.get('radius', 0.0))
        return (dh_dx, dh_dy)

    def is_safe(self, x, y, obstacles):
        """
        检查当前位置是否安全（所有CBF > 0）。

        Parameters
        ----------
        x, y : float
            机器人位置。
        obstacles : list of dict
            障碍物列表。

        Returns
        -------
        safe : bool
            True 如果所有 h_i > 0。
        """
        h_values = self.compute_cbf(x, y, obstacles)
        return all(h > 0 for h in h_values)

    def compute_minimum_distance(self, x, y, obstacles):
        """
        计算到最近障碍物的距离。

        Parameters
        ----------
        x, y : float
            机器人位置。
        obstacles : list of dict
            障碍物列表。

        Returns
        -------
        min_dist : float
            到最近障碍物的有效距离（表面到表面）。
        """
        if not obstacles:
            return float('inf')

        min_dist = float('inf')
        for obs in obstacles:
            dist, _, _, _ = self._compute_distance_to_obstacle(x, y, obs)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def compute_ttc(self, velocity, x, y, obstacles):
        """
        计算碰撞时间 (Time To Collision)。

        TTC = h_min / |v_rel|
        其中 h_min 为最小CBF值，v_rel 为接近障碍物的相对速度。

        Parameters
        ----------
        velocity : array-like, shape (2,)
            机器人速度 (vx, vy)。
        x, y : float
            机器人位置。
        obstacles : list of dict
            障碍物列表。

        Returns
        -------
        ttc : float
            碰撞时间 (s)。若远离障碍物或无障碍物返回 inf。
        """
        if not obstacles:
            return float('inf')

        vx, vy = velocity[0], velocity[1]
        speed = np.sqrt(vx ** 2 + vy ** 2)

        if speed < 1e-6:
            return float('inf')

        min_ttc = float('inf')
        for obs in obstacles:
            dist, _, cx, cy = self._compute_distance_to_obstacle(x, y, obs)
            # 计算朝向障碍物的速度分量
            if dist < 1e-10:
                return 0.0
            # 障碍物方向单位向量
            dir_x = (cx - x) / (dist + self.robot_radius +
                                obs.get('radius', 0.0))
            dir_y = (cy - y) / (dist + self.robot_radius +
                                obs.get('radius', 0.0))
            # 接近速度（正为接近）
            approach_speed = -(vx * dir_x + vy * dir_y)

            if approach_speed > 1e-6:
                ttc = dist / approach_speed
                if ttc < min_ttc:
                    min_ttc = ttc

        return min_ttc if min_ttc < float('inf') else float('inf')

    def get_risk_level(self, x, y, obstacles):
        """
        基于CBF值评估风险等级。

        风险等级:
            0 (安全):     h_min > d_safe
            1 (低风险):   0 < h_min <= d_safe
            2 (中风险):   -d_critical < h_min <= 0
            3 (高风险):   h_min <= -d_critical

        Parameters
        ----------
        x, y : float
            机器人位置。
        obstacles : list of dict
            障碍物列表。

        Returns
        -------
        risk_level : int
            风险等级 0-3。
        """
        if not obstacles:
            return 0

        h_values = self.compute_cbf(x, y, obstacles)
        h_min = min(h_values)

        if h_min > self.d_safe:
            return 0  # 安全
        elif h_min > 0:
            return 1  # 低风险
        elif h_min > -self.d_critical:
            return 2  # 中风险
        else:
            return 3  # 高风险

    def adaptive_safety_margin(self, velocity, obstacles):
        """
        根据速度动态调整安全距离。

        速度越快，所需安全距离越大:
            d_safe_adaptive = d_safe + k * ||v||²
        其中 k 为制动距离系数，与最大减速度相关。

        Parameters
        ----------
        velocity : array-like, shape (2,)
            机器人速度 (vx, vy)。
        obstacles : list of dict
            障碍物列表（用于获取最近障碍物方向）。

        Returns
        -------
        d_safe_adaptive : float
            自适应安全距离。
        """
        speed = np.sqrt(velocity[0] ** 2 + velocity[1] ** 2)
        # 制动距离系数: d = v²/(2*a_max)
        a_max = 2.0  # 最大减速度 m/s²
        braking_dist = speed ** 2 / (2.0 * a_max)
        d_safe_adaptive = self.d_safe + braking_dist
        return d_safe_adaptive

    def _solve_qp_simple(self, u_desired, A_constr, b_constr):
        """
        简单QP求解器（投影法）。

        求解:
            min ||u - u_desired||²
            s.t. A_constr @ u >= b_constr

        用投影迭代法：若期望控制违反约束，投影到可行域。
        对于2D控制输入(v, ω)，约束为线性半空间。

        Parameters
        ----------
        u_desired : ndarray, shape (2,)
            期望控制 (v_des, ω_des)。
        A_constr : ndarray, shape (m, 2)
            约束矩阵，每行为 [a1, a2]。
        b_constr : ndarray, shape (m,)
            约束右端项。

        Returns
        -------
        u_safe : ndarray, shape (2,)
            安全控制。
        """
        u = u_desired.copy()
        n_constraints = len(A_constr)

        if n_constraints == 0:
            return u

        # 检查约束是否满足
        for _ in range(50):  # 最大迭代次数
            violations = []
            for i in range(n_constraints):
                if A_constr[i] @ u < b_constr[i] - 1e-10:
                    violations.append(i)

            if not violations:
                break  # 所有约束满足

            # 对违反的约束做投影
            for i in violations:
                a = A_constr[i]
                b = b_constr[i]
                residual = a @ u - b
                a_norm_sq = a @ a
                if a_norm_sq < 1e-12:
                    continue
                # 投影到半空间 a·u >= b
                u = u + ((b - a @ u) / a_norm_sq) * a

        return u

    def safety_filter(self, u_desired, x, y, obstacles):
        """
        QP安全过滤器。

        在所有CBF约束下，找到最接近期望控制的安全控制:
            min ||u - u_desired||²
            s.t. ∇h_i · f(x,u) + α * h_i(x) >= 0,  ∀i

        对控制仿射系统 f(x,u) = f(x) + g(x)*u，约束化为:
            ∇h_i · g(x) * u >= -∇h_i · f(x) - α * h_i(x)

        用简单投影法求解。

        Parameters
        ----------
        u_desired : array-like, shape (2,)
            期望控制 (v_des, ω_des)。
        x, y : float
            机器人位置。
        obstacles : list of dict
            障碍物列表。

        Returns
        -------
        u_safe : ndarray, shape (2,)
            安全控制 (v_safe, ω_safe)。
        """
        u_des = np.array(u_desired, dtype=float)

        if not obstacles:
            return u_des

        # 收集CBF约束
        constraints_A = []
        constraints_b = []

        for obs in obstacles:
            h_val, _, _, _ = self._compute_distance_to_obstacle(x, y, obs)
            h_cbf = h_val - self.d_safe  # h_i(x)

            # 计算梯度 ∇h_i
            gh_dx, gh_dy = self.compute_cbf_gradient(x, y, obs)

            # 对控制仿射系统 g(x) 的简化模型:
            # 假设 u = (v, ω)，机器人在 (x,y,θ) 处
            # f(x,u) = [v*cos(θ), v*sin(θ), ω]
            # ∇h · f(x,u) = gh_dx * v * cos(θ) + gh_dy * v * sin(θ)
            # 约束: ∇h · f(x,u) + α * h_i >= 0
            # => v * (gh_dx * cos(θ) + gh_dy * sin(θ)) >= -α * h_i
            #
            # 这里简化为位置约束（不考虑θ），用梯度方向作为约束方向:
            #   ∇h · u_pos >= -α * h_i
            # 其中 u_pos 为期望移动方向
            #
            # 实际约束: 对线速度v施加约束
            #   (∂h/∂x) * v >= -α * h_i  （简化，假设朝向与梯度对齐）

            # 构造约束: A_i * u >= b_i
            # A_i = [gh_dx, gh_dy] (梯度方向作为约束方向)
            # b_i = -α * h_cbf
            # 这要求控制速度在梯度方向上的投影足够大
            A_row = np.array([gh_dx, gh_dy])
            b_val = -self.alpha * h_cbf

            # 只有约束有效时（h_cbf < 某阈值）才添加
            if h_cbf < self.d_safe:
                constraints_A.append(A_row)
                constraints_b.append(b_val)

        if not constraints_A:
            # 无有效约束，直接返回期望控制
            return u_des

        A_constr = np.array(constraints_A)
        b_constr = np.array(constraints_b)

        # 求解QP
        u_safe = self._solve_qp_simple(u_des, A_constr, b_constr)

        return u_safe
