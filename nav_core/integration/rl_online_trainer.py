"""RL-DQN在线训练集成 (v6.10)

在探索过程中实时学习frontier选择策略，逐步用RL替代手工评分。
采用ε-greedy策略平衡探索和利用。

训练目标：
  1. 状态: (coverage, loc_err, frontier_count, robot_cost, distance_to_goal)
  2. 动作: 选择哪个frontier（离散动作空间，对应排名后的frontier列表）
  3. 奖励: 覆盖率增量 - 路径代价 - 恢复惩罚
  4. 在线学习: 每个FOLLOW→PLAN周期收集一次经验
"""
import os
import math
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass

USE_RL_AGENT = os.environ.get("USE_RL_AGENT", "0") == "1"
RL_TRAIN = os.environ.get("RL_TRAIN", "0") == "1"


@dataclass
class RLState:
    """RL状态向量"""
    coverage: float = 0.0
    loc_error: float = 0.0
    frontier_count: int = 0
    robot_cost: int = 0
    distance_to_goal: float = 0.0
    no_progress: int = 0

    def to_vector(self):
        """转换为神经网络输入向量"""
        return np.array([
            self.coverage / 100.0,
            min(self.loc_error / 2.0, 1.0),
            min(self.frontier_count / 100.0, 1.0),
            min(self.robot_cost / 254.0, 1.0),
            min(self.distance_to_goal / 10.0, 1.0),
            min(self.no_progress / 30.0, 1.0),
        ], dtype=np.float32)


class RLOnlineTrainer:
    """RL在线训练器 (v6.10)

    在导航过程中收集经验，训练DQN选择frontier。
    """

    def __init__(self, config=None, rl_agent=None):
        cfg = config or {}

        # 延迟导入DQN agent
        if rl_agent is not None:
            self.agent = rl_agent
        else:
            try:
                from rl_agent import DQNAgent
                self.agent = DQNAgent(cfg=cfg)
            except ImportError:
                self.agent = None

        # 参数
        self.epsilon_start = cfg.get('epsilon_start', 1.0)
        self.epsilon_end = cfg.get('epsilon_end', 0.1)
        self.epsilon_decay = cfg.get('epsilon_decay', 0.999)
        self.train_interval = cfg.get('train_interval', 10)  # 训练间隔
        self.reward_coverage_weight = cfg.get('reward_coverage', 1.0)
        self.reward_distance_weight = cfg.get('reward_distance', -0.1)
        self.reward_recover_weight = cfg.get('reward_recover', -0.5)

        # 状态跟踪
        self._epsilon = self.epsilon_start
        self._last_state = None
        self._last_action = -1
        self._last_coverage = 0.0
        self._train_step = 0
        self._total_episodes = 0

    def select_frontier(self, frontiers, state: RLState, frame):
        """使用ε-greedy选择frontier

        Args:
            frontiers: 已排序的frontier候选列表 [(fx, fy, ...), ...]
            state: 当前RL状态
            frame: 当前帧

        Returns:
            int: 选择的frontier索引
        """
        if not frontiers or self.agent is None:
            return 0

        n_actions = min(len(frontiers), 10)  # 最多10个动作

        # ε-greedy
        if RL_TRAIN and np.random.random() < self._epsilon:
            # 探索：随机选择（偏向前面的frontier）
            weights = np.array([1.0 / (i + 1) for i in range(n_actions)])
            weights /= weights.sum()
            action = np.random.choice(n_actions, p=weights)
        else:
            # 利用：DQN选择
            state_vec = state.to_vector()
            q_values = self.agent.predict(state_vec)
            action = int(np.argmax(q_values[:n_actions]))

        # 记录状态转换
        if RL_TRAIN and self._last_state is not None:
            self._record_transition(state, action)

        self._last_state = state
        self._last_action = action
        self._last_coverage = state.coverage

        # ε衰减
        if RL_TRAIN:
            self._epsilon = max(self.epsilon_end, self._epsilon * self.epsilon_decay)

        return action

    def _record_transition(self, new_state: RLState, new_action: int):
        """记录状态转换和奖励"""
        if self.agent is None:
            return

        # 计算奖励
        coverage_delta = new_state.coverage - self._last_coverage
        reward = (self.reward_coverage_weight * coverage_delta +
                  self.reward_distance_weight * min(new_state.distance_to_goal / 10.0, 1.0) +
                  (self.reward_recover_weight if new_state.no_progress > 30 else 0))

        # 存储经验
        done = new_state.coverage > 95.0
        self.agent.store_experience(
            self._last_state.to_vector(),
            self._last_action,
            reward,
            new_state.to_vector(),
            done,
        )

        if done:
            self._total_episodes += 1

    def train_step(self, frame):
        """执行一步训练

        Returns:
            bool: 是否执行了训练
        """
        if not RL_TRAIN or self.agent is None:
            return False
        if frame % self.train_interval != 0:
            return False
        if len(self.agent) < 32:  # 经验不足
            return False

        loss = self.agent.train()
        self._train_step += 1

        if self._train_step % 100 == 0:
            print(f"[RL] 训练步={self._train_step} ε={self._epsilon:.3f} "
                  f"loss={loss:.4f} episodes={self._total_episodes}")

        return True

    def save_model(self, path):
        """保存训练模型"""
        if self.agent is not None:
            self.agent.save(path)

    def load_model(self, path):
        """加载训练模型"""
        if self.agent is not None:
            self.agent.load(path)

    def get_status(self):
        return {
            'enabled': USE_RL_AGENT and self.agent is not None,
            'training': RL_TRAIN,
            'epsilon': self._epsilon,
            'train_steps': self._train_step,
            'episodes': self._total_episodes,
            'buffer_size': len(self.agent) if self.agent else 0,
        }
