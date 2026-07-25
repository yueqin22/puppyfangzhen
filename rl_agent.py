"""
Reinforcement Learning Assisted Decision Making (v4.0)
=======================================================
Deep Q-Network (DQN) agent that assists high-level decision making
in the navigation system.

Instead of replacing the entire navigation stack with end-to-end RL,
we use RL for specific high-level decisions where traditional methods
are hard to tune:

  1. Frontier selection: which frontier to explore next?
  2. Recovery trigger: should we initiate AMCL recovery?
  3. Planner selection: TEB vs DWA depending on situation?

The agent observes a state vector and outputs a discrete action.
Uses a simple fully-connected Q-network with experience replay
and target network (vanilla DQN).

Training happens during simulation runs (online learning), and the
learned policy can be saved/loaded.

References:
  - Mnih et al. (2015) "Human-level control through deep reinforcement
    learning" (DQN)
  - Tai et al. (2017) "Virtual-to-real deep reinforcement learning:
    continuous control of mobile robots for mapless navigation"
  - Zhu et al. (2017) "Target-driven visual navigation in indoor scenes
    using deep reinforcement learning"
  - Kober et al. (2013) "Reinforcement learning in robotics: A survey"
"""
import os
import math
import json
import random
import numpy as np
from collections import deque

USE_RL = os.environ.get("USE_RL", "0") == "1"
RL_TRAIN = os.environ.get("RL_TRAIN", "0") == "1"


class ReplayBuffer:
    """Experience replay buffer for DQN."""

    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        return random.sample(self.buffer, min(batch_size, len(self.buffer)))

    def __len__(self):
        return len(self.buffer)


class SimpleQNetwork:
    """Simple fully-connected Q-network.

    Uses numpy only (no PyTorch/TensorFlow dependency) to keep the
    project lightweight. Implements forward pass and manual backprop
    for a small MLP.
    """

    def __init__(self, input_dim, hidden_dim, output_dim, lr=0.001):
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim
        self.lr = lr

        # Weights: input -> hidden -> output
        scale1 = math.sqrt(2.0 / input_dim)
        scale2 = math.sqrt(2.0 / hidden_dim)
        self.W1 = np.random.randn(input_dim, hidden_dim).astype(np.float32) * scale1
        self.b1 = np.zeros(hidden_dim, dtype=np.float32)
        self.W2 = np.random.randn(hidden_dim, output_dim).astype(np.float32) * scale2
        self.b2 = np.zeros(output_dim, dtype=np.float32)

    def forward(self, x):
        """Forward pass: Q(s, a) for all actions."""
        self._z1 = x @ self.W1 + self.b1
        self._a1 = np.maximum(0, self._z1)  # ReLU
        self._z2 = self._a1 @ self.W2 + self.b2
        return self._z2

    def predict(self, x):
        """Predict Q-values (no training)."""
        z1 = x @ self.W1 + self.b1
        a1 = np.maximum(0, z1)
        z2 = a1 @ self.W2 + self.b2
        return z2

    def train_step(self, x, target):
        """One training step with MSE loss.

        Args:
            x: input state (batch_size, input_dim)
            target: target Q-values (batch_size, output_dim)

        Returns:
            loss value
        """
        # Forward
        q_values = self.forward(x)

        # Loss: MSE
        error = q_values - target
        loss = np.mean(error ** 2)

        # Backward
        batch_size = x.shape[0]
        d_z2 = 2 * error / batch_size  # (batch, output)
        d_W2 = self._a1.T @ d_z2
        d_b2 = np.sum(d_z2, axis=0)

        d_a1 = d_z2 @ self.W2.T
        d_z1 = d_a1 * (self._z1 > 0).astype(np.float32)  # ReLU gradient
        d_W1 = x.T @ d_z1
        d_b1 = np.sum(d_z1, axis=0)

        # SGD update
        self.W1 -= self.lr * d_W1
        self.b1 -= self.lr * d_b1
        self.W2 -= self.lr * d_W2
        self.b2 -= self.lr * d_b2

        return float(loss)

    def save(self, filepath):
        """Save weights to npz."""
        np.savez(filepath, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2)

    def load(self, filepath):
        """Load weights from npz."""
        data = np.load(filepath)
        self.W1 = data['W1']
        self.b1 = data['b1']
        self.W2 = data['W2']
        self.b2 = data['b2']


class FrontierDQNAgent:
    """DQN agent for frontier selection (high-level exploration decision).

    State features (normalized):
      - Robot position (x, y)
      - Number of frontiers
      - Localization uncertainty (trace of covariance)
      - Coverage percentage
      - Distance to nearest frontier
      - Number of recovered times

    Actions (discrete):
      0: Select nearest frontier
      1: Select highest info-gain frontier
      2: Select farthest frontier (explore new areas)
      3: Perform AMCL recovery (if uncertain)
    """

    def __init__(self, state_dim=8, n_actions=4, cfg=None,
                 hidden_dim=64, learning_rate=0.001, gamma=0.95,
                 epsilon_start=1.0, epsilon_min=0.1, epsilon_decay=0.995,
                 batch_size=32, buffer_size=5000):
        cfg = cfg or {}
        self.state_dim = state_dim
        self.n_actions = n_actions

        hidden_dim = cfg.get('hidden_dim', hidden_dim)
        lr = cfg.get('learning_rate', learning_rate)
        self.gamma = cfg.get('gamma', gamma)
        self.epsilon = cfg.get('epsilon_start', epsilon_start)
        self.epsilon_min = cfg.get('epsilon_min', epsilon_min)
        self.epsilon_decay = cfg.get('epsilon_decay', epsilon_decay)
        self.batch_size = cfg.get('batch_size', batch_size)
        self.target_update_freq = cfg.get('target_update_freq', 100)

        self.q_net = SimpleQNetwork(state_dim, hidden_dim, n_actions, lr)
        self.target_net = SimpleQNetwork(state_dim, hidden_dim, n_actions, lr)
        self._sync_target()

        self.replay = ReplayBuffer(cfg.get('buffer_size', buffer_size))
        self.train_step_count = 0
        self.total_reward = 0.0
        self.episode = 0

    def _sync_target(self):
        """Copy weights from Q-network to target network."""
        self.target_net.W1 = self.q_net.W1.copy()
        self.target_net.b1 = self.q_net.b1.copy()
        self.target_net.W2 = self.q_net.W2.copy()
        self.target_net.b2 = self.q_net.b2.copy()

    def select_action(self, state, training=True):
        """Select action using epsilon-greedy policy.

        Args:
            state: numpy array of state features
            training: if True, use epsilon-greedy; else greedy

        Returns:
            action index (int)
        """
        state = np.array(state, dtype=np.float32).reshape(1, -1)

        if training and random.random() < self.epsilon:
            return random.randint(0, self.n_actions - 1)

        q_values = self.q_net.predict(state)[0]
        return int(np.argmax(q_values))

    def store_transition(self, state, action, reward, next_state, done):
        """Store experience in replay buffer."""
        self.replay.push(
            np.array(state, dtype=np.float32),
            action,
            float(reward),
            np.array(next_state, dtype=np.float32),
            done
        )
        self.total_reward += reward

    def train(self):
        """Train on one batch from replay buffer."""
        if len(self.replay) < self.batch_size:
            return 0.0

        batch = self.replay.sample(self.batch_size)
        states = np.array([b[0] for b in batch])
        actions = np.array([b[1] for b in batch], dtype=np.int32)
        rewards = np.array([b[2] for b in batch], dtype=np.float32)
        next_states = np.array([b[3] for b in batch])
        dones = np.array([b[4] for b in batch], dtype=np.float32)

        # Current Q values
        current_q = self.q_net.predict(states)

        # Target Q values
        next_q = self.target_net.predict(next_states)
        max_next_q = np.max(next_q, axis=1)
        target_q = current_q.copy()
        for i in range(len(batch)):
            target_q[i, actions[i]] = rewards[i] + self.gamma * max_next_q[i] * (1 - dones[i])

        # Train step
        loss = self.q_net.train_step(states, target_q)

        self.train_step_count += 1
        if self.train_step_count % self.target_update_freq == 0:
            self._sync_target()

        # Decay epsilon
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

        return loss

    def reset_episode(self):
        """Reset per-episode stats."""
        self.episode += 1
        self.total_reward = 0.0

    def save(self, filepath):
        """Save model and training state."""
        self.q_net.save(filepath + '_q.npz')
        self.target_net.save(filepath + '_target.npz')
        meta = {
            'epsilon': self.epsilon,
            'train_step_count': self.train_step_count,
            'episode': self.episode,
            'state_dim': self.state_dim,
            'n_actions': self.n_actions,
        }
        with open(filepath + '_meta.json', 'w') as f:
            json.dump(meta, f, indent=2)

    def load(self, filepath):
        """Load model and training state."""
        self.q_net.load(filepath + '_q.npz')
        self.target_net.load(filepath + '_target.npz')
        meta_path = filepath + '_meta.json'
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
            self.epsilon = meta.get('epsilon', self.epsilon)
            self.train_step_count = meta.get('train_step_count', 0)
            self.episode = meta.get('episode', 0)


def build_frontier_state(rx, ry, n_frontiers, loc_uncertainty,
                          coverage, nearest_dist, recover_count,
                          avg_speed, follow_ratio):
    """Build state vector for frontier selection agent.

    Returns normalized state array.
    """
    state = np.array([
        rx / 10.0,           # x position (normalized by map size)
        ry / 10.0,           # y position
        min(1.0, n_frontiers / 20.0),  # number of frontiers
        min(1.0, loc_uncertainty / 0.5),  # localization uncertainty
        coverage / 100.0,    # coverage percentage
        min(1.0, nearest_dist / 8.0),  # distance to nearest frontier
        min(1.0, recover_count / 20.0),  # recovery count
        min(1.0, avg_speed / 0.3),  # average speed
    ], dtype=np.float32)
    return state


def compute_frontier_reward(prev_coverage, new_coverage, loc_err_change,
                              collision, goal_reached, frame_delta=1):
    """Compute reward for frontier selection.

    Reward components:
      + coverage increase (positive)
      - localization error increase (negative)
      - collision penalty (large negative)
      + goal reached bonus
      - small time penalty
    """
    reward = 0.0
    reward += (new_coverage - prev_coverage) * 10.0  # coverage gain
    reward -= loc_err_change * 5.0  # loc error increase bad, decrease good
    reward -= 0.01 * frame_delta  # small time penalty
    if collision:
        reward -= 10.0
    if goal_reached:
        reward += 5.0
    return reward
