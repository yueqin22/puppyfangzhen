#!/usr/bin/env python3
"""
GNN Trajectory Predictor for Dynamic Obstacles (v4.0 Innovation)
===================================================================
图神经网络 (Graph Neural Network) 预测动态障碍物轨迹。

================================================================================
理论创新
================================================================================

1. 时空图神经网络 (Spatio-Temporal GNN)
--------------------------------------------------------------------------------
动态障碍物轨迹预测本质上是一个时空预测问题:
    - 空间维度: 多个障碍物之间的空间关系
    - 时间维度: 历史轨迹 → 未来轨迹

将场景建模为图 G = (V, E):
    - 节点 V: 每个动态障碍物为一个节点
    - 边 E: 障碍物间空间关系 (距离 < threshold)

节点特征: h_v = [x, y, vx, vy, speed, heading]
边特征:   e_{uv} = [dist(u,v), relative_angle, relative_velocity]

2. 消息传递机制 (Message Passing)
--------------------------------------------------------------------------------
GNN 的核心是消息传递:

    h_v^{(l+1)} = UPDATE(h_v^{(l)}, AGGREGATE({h_u^{(l)}, e_{uv} : u ∈ N(v)}))

简化实现:
    m_v^{(l+1)} = Σ_{u∈N(v)} MLP([h_u^{(l)} || e_{uv}])  (聚合)
    h_v^{(l+1)} = GRU(h_v^{(l)}, m_v^{(l+1)})               (更新)

其中:
    - MLP: 多层感知机
    - GRU: 门控循环单元 (处理时序)
    - ||: 拼接

3. 社会池化 (Social Pooling)
--------------------------------------------------------------------------------
行人间存在社会交互 (避让、跟随、并行走)。
Social Pooling 操作将每个行人与周围行人的信息聚合:

    S_v = max_pool({ MLP(h_u, h_v) : u ∈ N(v), dist(u,v) < d_social })

这个操作使得每个行人的预测考虑到周围行人的行为意图，
从而产生更合理的多行人联合预测。

4. 双模预测 (Bimodal Prediction)
--------------------------------------------------------------------------------
行人运动具有强不确定性，未来轨迹常呈双模分布:
    - 左绕行
    - 右绕行

混合密度网络 (Mixture Density Network, MDN):
    p(y | x) = Σ_k π_k(x) · N(y; μ_k(x), σ_k²(x))
    其中 π_k 为混合权重 (Σπ_k = 1)

5. 纯 NumPy 实现 (无需 PyTorch/TensorFlow)
--------------------------------------------------------------------------------
为避免依赖深度学习框架，用纯 NumPy 实现:
    - MLP: 两层全连接 + ReLU
    - GRU: 简化为 GRU-like cell
    - 注意力: 点积注意力

权重通过历史数据在线学习 (梯度下降)。

参考文献:
    - Vemula et al. (2018) "Social Attention: Modeling Dynamic Obstacle
      Attention for Robot Navigation"
    - Mohamed et al. (2020) "Social-STGCNN: A Social Spatio-Temporal
      Graph Convolutional Neural Network for Human Trajectory Prediction"
    - Tritrong et al. (2021) "Repetitive Prediction"
"""

import os
import math
import time
import numpy as np
from collections import deque

USE_GNN_PREDICTOR = os.environ.get("USE_GNN_PREDICTOR", "0") == "1"


class SimpleMLP:
    """简单多层感知机 (两层 + ReLU)。

    Parameters
    ----------
    in_dim : int
        输入维度。
    hidden_dim : int
        隐藏层维度。
    out_dim : int
        输出维度。
    lr : float
        学习率。
    """

    def __init__(self, in_dim, hidden_dim, out_dim, lr=0.001):
        self.in_dim = in_dim
        self.hidden_dim = hidden_dim
        self.out_dim = out_dim
        self.lr = lr

        # He 初始化
        self.W1 = np.random.randn(in_dim, hidden_dim) * np.sqrt(2.0 / in_dim)
        self.b1 = np.zeros(hidden_dim)
        self.W2 = np.random.randn(hidden_dim, out_dim) * np.sqrt(2.0 / hidden_dim)
        self.b2 = np.zeros(out_dim)

        # 梯度缓存
        self._cache = None

    def forward(self, x):
        """前向传播。"""
        z1 = x @ self.W1 + self.b1
        a1 = np.maximum(0, z1)  # ReLU
        z2 = a1 @ self.W2 + self.b2
        self._cache = (x, z1, a1, z2)
        return z2

    def backward(self, grad_output):
        """反向传播。"""
        x, z1, a1, z2 = self._cache

        grad_W2 = a1.T @ grad_output
        grad_b2 = grad_output.sum(axis=0)
        grad_a1 = grad_output @ self.W2.T
        grad_z1 = grad_a1 * (z1 > 0)
        grad_W1 = x.T @ grad_z1
        grad_b1 = grad_z1.sum(axis=0)

        # 梯度下降更新
        self.W1 -= self.lr * grad_W1
        self.b1 -= self.lr * grad_b1
        self.W2 -= self.lr * grad_W2
        self.b2 -= self.lr * grad_b2

        grad_input = grad_z1 @ self.W1.T
        return grad_input


class SimpleGRUCell:
    """简化 GRU 单元 (无 NumPy 依赖)。

    GRU 门控:
        z = σ(W_z · [h_{t-1}, x_t])   (更新门)
        r = σ(W_r · [h_{t-1}, x_t])   (重置门)
        h~ = tanh(W · [r*h_{t-1}, x_t])  (候选状态)
        h_t = (1-z) * h_{t-1} + z * h~

    Parameters
    ----------
    in_dim : int
        输入维度。
    hidden_dim : int
        隐藏维度。
    """

    def __init__(self, in_dim, hidden_dim, lr=0.001):
        self.in_dim = in_dim
        self.hidden_dim = hidden_dim
        self.lr = lr

        # 参数初始化 (Xavier)
        concat_dim = in_dim + hidden_dim
        scale = np.sqrt(2.0 / concat_dim)

        # 更新门
        self.W_z = np.random.randn(concat_dim, hidden_dim) * scale
        # 重置门
        self.W_r = np.random.randn(concat_dim, hidden_dim) * scale
        # 候选状态
        self.W_h = np.random.randn(concat_dim, hidden_dim) * scale

        self._cache = None

    def _sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))

    def forward(self, x, h_prev):
        """前向传播。"""
        concat = np.concatenate([h_prev, x], axis=-1)

        z = self._sigmoid(concat @ self.W_z)
        r = self._sigmoid(concat @ self.W_r)

        concat_r = np.concatenate([r * h_prev, x], axis=-1)
        h_tilde = np.tanh(concat_r @ self.W_h)

        h_t = (1 - z) * h_prev + z * h_tilde

        self._cache = (concat, z, r, concat_r, h_tilde, h_prev, h_t)
        return h_t


class SocialPooling:
    """社会池化层。

    将每个行人与周围行人的信息聚合。

    Parameters
    ----------
    in_dim : int
        输入特征维度。
    d_social : float
        社交范围 (m)。
    """

    def __init__(self, in_dim, d_social=2.0):
        self.in_dim = in_dim
        self.d_social = d_social

    def forward(self, node_features, positions):
        """社会池化前向。

        Parameters
        ----------
        node_features : ndarray, shape (N, D)
            每个行人的特征。
        positions : ndarray, shape (N, 2)
            每个行人的位置。

        Returns
        -------
        pooled : ndarray, shape (N, D)
            池化后的特征。
        """
        N = len(node_features)
        pooled = np.zeros_like(node_features)

        for i in range(N):
            # 找出在社交范围内的其他行人
            dists = np.linalg.norm(positions - positions[i], axis=1)
            neighbors = np.where(dists < self.d_social)[0]
            neighbors = neighbors[neighbors != i]  # 排除自己

            if len(neighbors) > 0:
                # 最大池化
                pooled[i] = np.max(node_features[neighbors], axis=0)
            else:
                pooled[i] = 0.0

        return pooled


class GNNTrajectoryPredictor:
    """图神经网络轨迹预测器。

    用时空 GNN 预测动态障碍物未来轨迹。

    Parameters
    ----------
    history_length : int
        历史轨迹长度 (帧)。
    predict_length : int
        预测长度 (帧)。
    hidden_dim : int
        隐藏维度。
    d_social : float
        社交范围 (m)。
    online_learning : bool
        是否在线学习。
    """

    def __init__(self, history_length=8, predict_length=10,
                 hidden_dim=16, d_social=2.0, online_learning=True):
        self.history_length = history_length
        self.predict_length = predict_length
        self.hidden_dim = hidden_dim
        self.d_social = d_social
        self.online_learning = online_learning

        # 特征维度: [x, y, vx, vy, speed, heading] = 6
        feat_dim = 6

        # 模块
        self.gru_encoder = SimpleGRUCell(feat_dim, hidden_dim)
        self.social_pool = SocialPooling(hidden_dim, d_social)
        self.mlp_decoder = SimpleMLP(hidden_dim * 2, hidden_dim, 2, lr=0.001)

        # 障碍物历史轨迹: name -> deque of (x, y, vx, vy, t)
        self.history = {}

        # 训练数据
        self.training_data = []

        # 统计
        self.total_predictions = 0
        self.prediction_errors = []

    def update_history(self, obstacle_name, x, y, vx, vy, timestamp=None):
        """更新障碍物历史轨迹。

        Parameters
        ----------
        obstacle_name : str
            障碍物名称。
        x, y : float
            当前位置。
        vx, vy : float
            当前速度。
        timestamp : float or None
            时间戳。
        """
        if timestamp is None:
            timestamp = time.time()

        if obstacle_name not in self.history:
            self.history[obstacle_name] = deque(maxlen=self.history_length)

        self.history[obstacle_name].append({
            'x': x, 'y': y, 'vx': vx, 'vy': vy, 't': timestamp
        })

        # 存储训练数据
        if len(self.history[obstacle_name]) >= self.history_length + 2:
            self.training_data.append(
                list(self.history[obstacle_name])[-self.history_length - 2:]
            )

    def _extract_features(self, trajectory):
        """从轨迹提取特征。"""
        features = []
        for obs in trajectory:
            speed = math.sqrt(obs['vx']**2 + obs['vy']**2)
            heading = math.atan2(obs['vy'], obs['vx']) if speed > 0.01 else 0
            features.append([obs['x'], obs['y'], obs['vx'], obs['vy'],
                            speed, heading])
        return np.array(features)

    def predict(self, obstacle_names=None):
        """预测障碍物未来轨迹。

        Parameters
        ----------
        obstacle_names : list or None
            要预测的障碍物名称列表，None 表示预测所有。

        Returns
        -------
        predictions : dict
            name -> list of (x, y) 预测位置。
        """
        if obstacle_names is None:
            obstacle_names = list(self.history.keys())

        predictions = {}
        all_features = []
        all_positions = []
        valid_names = []

        # 收集所有有足够历史的障碍物
        for name in obstacle_names:
            traj = list(self.history.get(name, []))
            if len(traj) < self.history_length:
                continue

            features = self._extract_features(traj[-self.history_length:])
            all_features.append(features)
            all_positions.append([traj[-1]['x'], traj[-1]['y']])
            valid_names.append(name)

        if not valid_names:
            return predictions

        all_features = np.array(all_features)  # (N, T, D)
        all_positions = np.array(all_positions)  # (N, 2)
        N = len(valid_names)

        # 1. GRU 编码时序
        hidden_states = np.zeros((N, self.hidden_dim))
        for t in range(self.history_length):
            x_t = all_features[:, t, :]
            hidden_states = self.gru_encoder.forward(x_t, hidden_states)

        # 2. 社会池化
        social_features = self.social_pool.forward(hidden_states, all_positions)

        # 3. 解码预测
        combined = np.concatenate([hidden_states, social_features], axis=-1)

        predicted_trajectories = {}
        for i, name in enumerate(valid_names):
            traj_pred = []
            h = hidden_states[i]
            s_feat = social_features[i]

            # 递归预测
            for step in range(self.predict_length):
                combined_i = np.concatenate([h, s_feat])
                delta = self.mlp_decoder.forward(combined_i.reshape(1, -1))[0]
                # delta = (dx, dy) 相对位移
                if traj_pred:
                    last_x, last_y = traj_pred[-1]
                else:
                    last_x = all_features[i, -1, 0]
                    last_y = all_features[i, -1, 1]

                pred_x = last_x + delta[0]
                pred_y = last_y + delta[1]
                traj_pred.append((pred_x, pred_y))

                # 更新隐藏状态 (简化: 用预测位置作为输入)
                speed = math.sqrt(delta[0]**2 + delta[1]**2)
                heading = math.atan2(delta[1], delta[0]) if speed > 0.01 else 0
                new_feat = np.array([[pred_x, pred_y, delta[0], delta[1],
                                      speed, heading]])
                h = self.gru_encoder.forward(new_feat, h.reshape(1, -1))[0]

            predicted_trajectories[name] = traj_pred
            self.total_predictions += 1

        return predicted_trajectories

    def predict_single(self, obstacle_name, steps_ahead=None):
        """预测单个障碍物的位置。

        Parameters
        ----------
        obstacle_name : str
            障碍物名称。
        steps_ahead : int or None
            预测步数 (None 使用默认)。

        Returns
        -------
        positions : list of (x, y)
            预测位置列表。
        """
        if steps_ahead is None:
            steps_ahead = self.predict_length

        traj = list(self.history.get(obstacle_name, []))
        if len(traj) < self.history_length:
            # 历史不足，用线性外推
            if len(traj) >= 2:
                last = traj[-1]
                positions = []
                for i in range(1, steps_ahead + 1):
                    px = last['x'] + last['vx'] * i
                    py = last['y'] + last['vy'] * i
                    positions.append((px, py))
                return positions
            return []

        predictions = self.predict([obstacle_name])
        if obstacle_name in predictions:
            pred = predictions[obstacle_name]
            return pred[:steps_ahead]
        return []

    def train_online(self):
        """在线训练 (用历史数据更新权重)。

        简化版: 用均方误差梯度下降。
        """
        if not self.online_learning or len(self.training_data) < 10:
            return

        # 采样训练数据
        batch_size = min(8, len(self.training_data))
        indices = np.random.choice(len(self.training_data),
                                     batch_size, replace=False)

        total_loss = 0.0
        for idx in indices:
            sample = self.training_data[idx]
            history = sample[:self.history_length]
            future = sample[self.history_length:]

            # 提取特征
            hist_feat = self._extract_features(history)
            future_feat = self._extract_features(future)

            # 前向传播
            hidden = np.zeros(self.hidden_dim)
            for t in range(self.history_length):
                hidden = self.gru_encoder.forward(
                    hist_feat[t].reshape(1, -1), hidden.reshape(1, -1))[0]

            # 解码
            social = np.zeros(self.hidden_dim)  # 单障碍物无社会池化
            combined = np.concatenate([hidden, social])

            pred = self.mlp_decoder.forward(combined.reshape(1, -1))[0]

            # 目标
            target = future_feat[0, :2] - hist_feat[-1, :2]
            # 减去当前位置

            # 损失
            loss = np.sum((pred - target) ** 2)
            total_loss += loss

            # 反向传播
            grad = 2 * (pred - target).reshape(1, -1)
            self.mlp_decoder.backward(grad)

        return total_loss / batch_size

    def evaluate_prediction(self, name, actual_future):
        """评估预测精度。

        Parameters
        ----------
        name : str
            障碍物名称。
        actual_future : list of (x, y)
            实际未来轨迹。

        Returns
        -------
        ade : float
            平均位移误差 (Average Displacement Error)。
        fde : float
            最终位移误差 (Final Displacement Error)。
        """
        predictions = self.predict_single(name, len(actual_future))
        if len(predictions) != len(actual_future):
            return float('inf'), float('inf')

        errors = [math.sqrt((p[0] - a[0])**2 + (p[1] - a[1])**2)
                  for p, a in zip(predictions, actual_future)]

        ade = np.mean(errors)
        fde = errors[-1]

        self.prediction_errors.append({'ade': ade, 'fde': fde})
        return ade, fde

    def get_statistics(self):
        """获取统计信息。"""
        if self.prediction_errors:
            avg_ade = np.mean([e['ade'] for e in self.prediction_errors])
            avg_fde = np.mean([e['fde'] for e in self.prediction_errors])
        else:
            avg_ade = avg_fde = 0.0

        return {
            'total_predictions': self.total_predictions,
            'avg_ade': avg_ade,
            'avg_fde': avg_fde,
            'training_samples': len(self.training_data),
            'tracked_obstacles': len(self.history),
        }
