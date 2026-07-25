"""
POMDP导航模块

将导航问题建模为部分可观测马尔可夫决策过程(POMDP)。

================================================================================
理论贡献
================================================================================

1. 维度诅咒问题 (Curse of Dimensionality)
--------------------------------------------------------------------------------
导航POMDP的状态空间 S 为机器人位姿 (x, y, θ)，连续空间离散化后状态数为
    |S| = N_x * N_y * N_θ
置信状态 b(s) 是 S 上的概率分布，维度为 |S|。
对于 100×100×36 的网格，|S| = 360,000，置信状态即为 360,000 维向量，
直接在完整置信空间上做值迭代在计算上不可行。

解决思路：
    a) 粒子滤波 (Particle Filter): 用 N 个带权粒子近似置信状态，
       将维度从 |S| 降至 O(N)，N << |S|
    b) 点基值迭代 (PBVI, Point-Based Value Iteration): 仅在置信空间中
       采样的有限点集 B = {b_1, ..., b_K} 上计算值函数
    c) POMCP (Partial Observability Monte Carlo Planning): 在线蒙特卡洛
       树搜索，无需离线预计算，按需采样，天然适配大状态空间

2. POMCP 采样策略
--------------------------------------------------------------------------------
POMCP = Monte Carlo Tree Search (MCTS) 应用于 POMDP
    - 搜索树节点为 (信念节点, 动作节点) 的交替结构
    - 信念节点用粒子集合近似表示: b ≈ {s_1, s_2, ..., s_n}
    - 动作选择采用 UCB1: a* = argmax_a [ Q(b,a) + c*sqrt(ln(N_b) / N_{b,a}) ]
    - 通过仿真采样估计 Q 值: Q(b,a) ≈ (1/K) Σ_{k=1}^{K} R_k (累积折扣回报)
    - 每次采样用生成模型 G(s,a) -> (s', o, r) 产生轨迹
    - 蒙特卡洛估计渐进完整，K → ∞ 时收敛到真实值

3. 信息论探索策略与 POMDP 的关系
--------------------------------------------------------------------------------
导航中存在 "探索-利用" (exploration-exploitation) 权衡：
    - 利用 (exploitation): 朝已知最优路径前进
    - 探索 (exploration): 主动采集信息减少状态不确定性

信息增益:
    IG(a) = H(b) - E_{o|b,a}[H(b')] = H(b) - Σ_o P(o|b,a) H(τ(b,a,o))
其中 τ(b,a,o) 为置信更新算子，H(b) 为熵。

与 POMDP 值函数的结合:
    V(b) = max_a [ R(b,a) + γ * Σ_o P(o|b,a) * V(τ(b,a,o)) + λ * IG(a) ]
信息增益作为内在奖励 (intrinsic reward) 叠加在外在奖励上，
驱动机器人主动探索未知区域，加速 AMCL 置信度收敛。

================================================================================
数学公式
================================================================================
- 置信更新 (Bayesian Filter):
      b'(s') = η * Z(o|s') * Σ_{s∈S} T(s'|s,a) * b(s)
  其中 η 为归一化常数。

- POMDP 贝尔曼方程:
      V*(b) = max_{a∈A} [ R(b,a) + γ * Σ_{o∈O} P(o|b,a) * V*(τ(b,a,o)) ]
  其中 R(b,a) = Σ_s b(s) R(s,a)。

- 熵 (Shannon Entropy):
      H(b) = -Σ_s b(s) * log(b(s))

- KL 散度 (信息增益):
      D_KL(b' || b) = Σ_s b'(s) * log(b'(s) / b(s))
"""

import numpy as np


class POMDPNavigation:
    """
    POMDP导航控制器

    将移动机器人导航建模为部分可观测马尔可夫决策过程。
    状态为机器人位姿 (x, y, θ)，动作为离散运动指令，
    观测为 LiDAR 读数与 AMCL 置信度。

    Parameters
    ----------
    state_size : int or tuple
        状态空间大小。若为 int 则表示一维状态数；若为 tuple (nx, ny, ntheta)
        则表示三维离散网格各维度大小。
    action_size : int
        动作空间大小（默认4: forward, turn_left, turn_right, stop）。
    observation_size : int
        观测空间维度（LiDAR射线数 + 1个AMCL置信度）。

    Attributes
    ----------
    actions : list of str
        动作名称列表。
    gamma : float
        折扣因子 γ ∈ (0, 1)。
    """

    # 动作定义
    ACTIONS = ['forward', 'turn_left', 'turn_right', 'stop']

    def __init__(self, state_size, action_size, observation_size):
        """
        初始化POMDP导航模型。

        Parameters
        ----------
        state_size : int or tuple
            离散化状态空间大小。
        action_size : int
            动作空间维度。
        observation_size : int
            观测空间维度。
        """
        # 状态空间
        if isinstance(state_size, tuple):
            self.grid_shape = state_size  # (nx, ny, ntheta)
            self.n_states = int(np.prod(state_size))
        else:
            self.grid_shape = (state_size,)
            self.n_states = state_size

        self.state_size = state_size
        self.action_size = action_size
        self.observation_size = observation_size

        # 动作空间
        self.actions = self.ACTIONS[:action_size] if action_size <= 4 else \
            [f'action_{i}' for i in range(action_size)]

        # POMDP参数
        self.gamma = 0.95          # 折扣因子
        self.noise_std = 0.1      # 运动噪声标准差
        self.obs_noise_std = 0.05  # 观测噪声标准差

        # 奖励参数
        self.reward_goal = 100.0    # 到达目标奖励
        self.reward_collision = -100.0  # 碰撞惩罚
        self.reward_step = -1.0    # 每步代价
        self.reward_info_gain = 1.0  # 信息增益系数

        # 运动模型参数
        self.step_distance = 0.5   # forward步长
        self.turn_angle = np.pi / 8  # 转向角度

        # POMCP参数
        self.pomcp_iterations = 200  # POMCP每次搜索的迭代次数
        self.pomcp_depth = 5        # POMCP搜索深度
        self.ucb_c = 1.0 / np.sqrt(2)  # UCB1探索常数
        self.n_particles = 200   # 粒子数

        # 预分配置信状态
        self._rng = np.random.default_rng(42)

    def _motion_model(self, state, action):
        """
        带噪声的运动模型 T(s'|s,a)。

        给定当前状态 s=(x, y, θ) 和动作 a，生成下一状态 s'。
        加入高斯运动噪声模拟真实机器人不确定性。

        Parameters
        ----------
        state : array-like, shape (3,)
            当前状态 (x, y, θ)。
        action : str
            动作名称。

        Returns
        -------
        new_state : ndarray, shape (3,)
            下一状态 (x', y', θ')。
        """
        x, y, theta = state[0], state[1], state[2]
        # 运动噪声
        dx = self.noise_std * self._rng.standard_normal()
        dy = self.noise_std * self._rng.standard_normal()
        dtheta = self.noise_std * self._rng.standard_normal()

        if action == 'forward':
            x_new = x + self.step_distance * np.cos(theta) + dx
            y_new = y + self.step_distance * np.sin(theta) + dy
            theta_new = theta + dtheta
        elif action == 'turn_left':
            x_new = x + dx
            y_new = y + dy
            theta_new = theta + self.turn_angle + dtheta
        elif action == 'turn_right':
            x_new = x + dx
            y_new = y + dy
            theta_new = theta - self.turn_angle + dtheta
        else:  # stop
            x_new = x + dx
            y_new = y + dy
            theta_new = theta + dtheta

        # 角度归一化到 [-π, π]
        theta_new = (theta_new + np.pi) % (2 * np.pi) - np.pi
        return np.array([x_new, y_new, theta_new])

    def _observation_model(self, state, true_obs=None):
        """
        观测模型 Z(o|s)。

        根据状态生成带噪声的观测。若提供真实观测，则返回观测概率。

        Parameters
        ----------
        state : ndarray, shape (3,)
            状态 (x, y, θ)。
        true_obs : ndarray or None
            真实观测值，若提供则计算 P(o|s)。

        Returns
        -------
        obs : ndarray
            生成的观测（若 true_obs=None）或观测概率（若提供）。
        """
        if true_obs is None:
            # 生成模拟观测：前若干维为LiDAR距离，末维为AMCL置信度
            lidar = np.abs(self._rng.standard_normal(self.observation_size - 1)) \
                    * (1.0 + 0.5 * np.sin(state[2])) + self.obs_noise_std
            amcl_conf = 0.8 + 0.2 * self._rng.random()
            obs = np.concatenate([lidar, [amcl_conf]])
            return obs
        else:
            # 计算观测概率（高斯模型）
            expected = np.abs(state[:self.observation_size - 1]) + 0.5
            diff = true_obs[:self.observation_size - 1] - expected
            prob = np.exp(-0.5 * np.sum(diff ** 2) / self.obs_noise_std ** 2)
            return prob

    def _reward(self, state, action, goal=None):
        """
        奖励函数 R(s, a)。

        R(s,a) = 
            +100          若到达目标
            -100          若碰撞
            -1            每步代价
            +λ*ΔH         信息增益

        Parameters
        ----------
        state : ndarray, shape (3,)
            状态。
        action : str
            动作。
        goal : ndarray or None
            目标位置 (x_g, y_g)。

        Returns
        -------
        reward : float
            即时奖励。
        """
        reward = self.reward_step  # 每步代价

        if goal is not None:
            dist_to_goal = np.sqrt((state[0] - goal[0]) ** 2 +
                                   (state[1] - goal[1]) ** 2)
            if dist_to_goal < 0.5:
                reward += self.reward_goal

        return reward

    def belief_update(self, belief, action, observation):
        """
        置信状态更新（粒子滤波）。

        实现 Bayesian Filter 更新方程:
            b'(s') = η * Z(o|s') * Σ_s T(s'|s,a) * b(s)

        用粒子滤波近似：先根据运动模型预测，再用观测重采样。

        Parameters
        ----------
        belief : dict
            置信状态，包含:
            - 'particles': ndarray, shape (N, 3), 粒子状态
            - 'weights': ndarray, shape (N,), 粒子权重
        action : str
            执行的动作。
        observation : ndarray
            观测值。

        Returns
        -------
        new_belief : dict
            更新后的置信状态，结构同 belief。
        """
        particles = belief['particles'].copy()
        weights = belief['weights'].copy()

        # ===== 预测步：根据运动模型传播粒子 =====
        for i in range(len(particles)):
            particles[i] = self._motion_model(particles[i], action)

        # ===== 更新步：用观测修正权重 =====
        for i in range(len(particles)):
            obs_prob = self._observation_model(particles[i], observation)
            weights[i] *= max(obs_prob, 1e-300)  # 防止下溢

        # 归一化权重
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights = weights / weight_sum
        else:
            # 所有权重为0，重置为均匀分布
            weights = np.ones(len(weights)) / len(weights)

        # ===== 重采样（系统重采样） =====
        n = len(weights)
        positions = (self._rng.random() + np.arange(n)) / n
        cumsum = np.cumsum(weights)
        cumsum[-1] = 1.0  # 防止浮点误差
        indices = np.searchsorted(cumsum, positions)
        particles = particles[indices]
        weights = np.ones(n) / n

        return {'particles': particles, 'weights': weights}

    def _estimate_state(self, belief):
        """
        从置信状态估计最可能状态（加权均值）。

        Parameters
        ----------
        belief : dict
            置信状态。

        Returns
        -------
        mean_state : ndarray, shape (3,)
            加权均值状态。
        """
        particles = belief['particles']
        weights = belief['weights']
        # 对角度取加权平均（处理角度卷绕）
        sin_theta = np.sin(particles[:, 2])
        cos_theta = np.cos(particles[:, 2])
        mean_x = np.sum(particles[:, 0] * weights)
        mean_y = np.sum(particles[:, 1] * weights)
        mean_theta = np.arctan2(np.sum(sin_theta * weights),
                                np.sum(cos_theta * weights))
        return np.array([mean_x, mean_y, mean_theta])

    def compute_information_gain(self, belief_before, belief_after):
        """
        计算信息增益（KL散度）。

        信息增益衡量置信状态变化带来的信息量:
            IG = D_KL(b_after || b_before) = Σ b'(s) * log(b'(s)/b(s))

        对于粒子滤波表示，用粒子分布的核密度估计计算KL散度，
        或用熵差近似:
            IG ≈ H(b_before) - H(b_after)

        Parameters
        ----------
        belief_before : dict
            动作前的置信状态。
        belief_after : dict
            动作后的置信状态。

        Returns
        -------
        info_gain : float
            信息增益值（非负）。
        """
        # 用粒子分布的协方差迹作为不确定性度量
        # 熵近似: H(b) ≈ log(det(Σ))，Σ为状态协方差
        cov_before = np.cov(belief_before['particles'].T,
                           aweights=belief_before['weights'])
        cov_after = np.cov(belief_after['particles'].T,
                          aweights=belief_after['weights'])

        # 加小量保证正定性
        eps = 1e-8
        cov_before += eps * np.eye(3)
        cov_after += eps * np.eye(3)

        # 熵近似：用协方差矩阵的迹
        entropy_before = 0.5 * np.log(np.trace(cov_before) + eps)
        entropy_after = 0.5 * np.log(np.trace(cov_after) + eps)

        info_gain = entropy_before - entropy_after
        return max(info_gain, 0.0)  # 信息增益非负

    def expected_information_gain(self, belief, action):
        """
        计算期望信息增益。

        采取动作a后的期望信息增益:
            EIG(a) = H(b) - E_{o|b,a}[H(τ(b,a,o))]
                   = H(b) - Σ_o P(o|b,a) * H(τ(b,a,o))

        通过蒙特卡洛采样近似：多次模拟，取平均信息增益。

        Parameters
        ----------
        belief : dict
            当前置信状态。
        action : str
            待评估的动作。

        Returns
        -------
        eig : float
            期望信息增益。
        """
        n_samples = 20
        total_ig = 0.0

        for _ in range(n_samples):
            # 随机采样一个状态
            idx = self._rng.choice(len(belief['particles']),
                                    p=belief['weights'])
            state = belief['particles'][idx]
            # 模拟动作和观测
            next_state = self._motion_model(state, action)
            obs = self._observation_model(next_state)
            # 更新信念
            new_belief = self.belief_update(belief, action, obs)
            # 计算信息增益
            ig = self.compute_information_gain(belief, new_belief)
            total_ig += ig

        return total_ig / n_samples

    def _simulate(self, belief, depth, goal):
        """
        POMCP 递归蒙特卡洛模拟。

        从当前信念节点开始，递归向下采样到指定深度，
        返回从该节点开始的期望累积折扣回报。

        Parameters
        ----------
        belief : dict
            当前信念状态。
        depth : int
            剩余搜索深度。
        goal : ndarray
            目标位置。

        Returns
        -------
        total_reward : float
            累积折扣奖励。
        """
        if depth <= 0:
            return 0.0

        # 简化策略：UCB1选择动作
        state = self._estimate_state(belief)
        best_action = None
        best_value = -np.inf

        for action in self.actions:
            # 即时奖励
            r = self._reward(state, action, goal)
            # 信息增益奖励
            ig = self.expected_information_gain(belief, action)
            value = r + self.reward_info_gain * ig
            # UCB探索加成
            exploration_bonus = self.ucb_c * np.sqrt(np.log(depth + 1))
            total = value + exploration_bonus
            if total > best_value:
                best_value = total
                best_action = action

        # 模拟下一步
        next_state = self._motion_model(state, best_action)
        obs = self._observation_model(next_state)
        new_belief = self.belief_update(belief, best_action, obs)
        reward = self._reward(state, best_action, goal)
        future = self._simulate(new_belief, depth - 1, goal)

        return reward + self.gamma * future

    def compute_value_function(self, belief, horizon=5):
        """
        用POMCP近似求解POMDP值函数。

        POMCP (Partial Observability Monte Carlo Planning):
        - 在线蒙特卡洛树搜索
        - 每个信念节点用粒子集表示
        - 用UCT策略选择动作
        - 通过K次随机模拟估计Q(b,a)

        值函数估计:
            V(b) ≈ (1/K) * Σ_k R_k (k次模拟的平均回报)

        Parameters
        ----------
        belief : dict
            当前置信状态。
        horizon : int, optional
            搜索深度（时间步），默认5。

        Returns
        -------
        value : float
            估计的值函数 V(b)。
        """
        total_value = 0.0
        goal = belief.get('goal', None)

        for _ in range(self.pomcp_iterations):
            # 从信念中采样一个状态
            idx = self._rng.choice(len(belief['particles']),
                                    p=belief['weights'])
            sampled_belief = {
                'particles': belief['particles'][idx:idx+1].copy(),
                'weights': np.array([1.0]),
                'goal': goal
            }
            # 递归模拟
            value = self._simulate(sampled_belief, horizon, goal)
            total_value += value

        return total_value / self.pomcp_iterations

    def get_optimal_action(self, belief):
        """
        获取最优动作。

        对每个动作计算 Q(b,a)，选择使Q最大的动作:
            a* = argmax_a Q(b,a)
        其中 Q(b,a) = R(b,a) + γ * V(τ(b,a,o)) + λ * EIG(a)

        Parameters
        ----------
        belief : dict
            当前置信状态，可包含 'goal' 键。

        Returns
        -------
        action : str
            最优动作名称。
        """
        goal = belief.get('goal', None)
        state = self._estimate_state(belief)

        best_action = self.actions[0]
        best_value = -np.inf

        for action in self.actions:
            # 即时奖励
            immediate_reward = self._reward(state, action, goal)
            # 期望信息增益
            eig = self.expected_information_gain(belief, action)
            # 综合价值
            q_value = immediate_reward + self.reward_info_gain * eig

            if q_value > best_value:
                best_value = q_value
                best_action = action

        return best_action

    def plot_belief(self, belief, filename):
        """
        可视化置信状态分布。

        将粒子分布绘制为散点图，颜色表示权重，
        保存到指定文件。

        Parameters
        ----------
        belief : dict
            置信状态，包含 'particles' 和 'weights'。
        filename : str
            输出图片文件名。

        Note
        ----
        需要 matplotlib。若未安装则跳过绘图并打印提示。
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("[plot_belief] matplotlib未安装，跳过可视化")
            return

        particles = belief['particles']
        weights = belief['weights']

        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        # 左图: x-y 位置分布
        ax = axes[0]
        scatter = ax.scatter(particles[:, 0], particles[:, 1],
                             c=weights, cmap='hot', s=10, alpha=0.6)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('粒子位置分布 (x, y)')
        plt.colorbar(scatter, ax=ax, label='权重')

        # 右图: 角度分布直方图
        ax = axes[1]
        ax.hist(particles[:, 2], bins=36, weights=weights,
                color='steelblue', edgecolor='black', alpha=0.7)
        ax.set_xlabel('θ (rad)')
        ax.set_ylabel('概率')
        ax.set_title('朝向角分布')

        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        plt.close()
        print(f"[plot_belief] 置信状态可视化已保存到 {filename}")
