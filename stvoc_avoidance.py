"""STVOC: Spatio-Temporal Velocity Obstacle Cone 时空速度障碍锥避障法

创新方法：基于时空轨迹预测和速度障碍锥的预防式动态碰撞避免。

核心思想：
    传统避障方法（排斥力、DWA、急退）都是反应式的——等到行人很近才触发。
    STVOC 通过多步轨迹预测提前3秒识别碰撞风险，在速度空间中搜索
    避开所有行人预测轨迹锥体的最优速度，实现预防式避障。

五大组件：
    1. 多步轨迹预测器 (TrajectoryPredictor)
       - 基于行人速度+加速度估计，预测未来N步的完整轨迹
       - 支持线性运动和随机运动两种模式

    2. 时空冲突图 (SpatioTemporalConflictGraph)
       - 将机器人规划路径与行人预测轨迹构建时空冲突图
       - 计算每个时空点的冲突概率
       - 找出最早冲突时间点和位置

    3. 速度障碍锥 (VelocityObstacleCone)
       - 基于冲突点，计算每个行人的速度障碍锥
       - 在速度空间中搜索避开所有锥体的最优速度
       - 选择既避开行人又最接近目标方向的速度

    4. 热点记忆 (HotspotMemory)
       - 记录历史碰撞/近距离规避的位置
       - 使用指数衰减的热度图
       - 经过热点区域时自动降速+扩大预测范围

    5. TTC博弈等待决策 (TTCGameDecision)
       - 计算机器人和行人到达冲突点的时间
       - 基于TTC比较决定等待还是通过
       - 避免不必要的等待和危险抢行

作者: PuppyNav Team
版本: 1.0
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from collections import deque

import numpy as np


# ====================================================================
# 数据结构
# ====================================================================

@dataclass
class PredictedPoint:
    """预测轨迹点"""
    x: float
    y: float
    t: float  # 时间偏移（秒）


@dataclass
class ConflictEvent:
    """时空冲突事件"""
    position: Tuple[float, float]       # 冲突位置
    time: float                          # 冲突时间（秒）
    obstacle_name: str                   # 障碍物名称
    severity: float                      # 严重程度 [0, 1]
    robot_speed_at_conflict: float       # 机器人到达冲突点时的速度
    obstacle_speed_at_conflict: float    # 障碍物到达冲突点时的速度


@dataclass
class VelocityObstacle:
    """速度障碍锥

    表示某个行人在速度空间中造成的禁止区域。
    机器人速度如果落入此锥体，将在未来与该行人碰撞。
    """
    apex: Tuple[float, float]            # 锥顶（行人速度向量）
    left_dir: Tuple[float, float]         # 锥左边界方向（单位向量）
    right_dir: Tuple[float, float]        # 锥右边界方向（单位向量）
    obstacle_name: str
    distance: float                       # 行人到机器人的距离


@dataclass
class AvoidanceCommand:
    """避障指令"""
    vx: float              # 推荐x方向线速度（世界坐标系）
    wz: float              # 推荐角速度
    action: str            # 动作类型: 'cruise'/'avoid'/'wait'/'flee'
    reason: str             # 原因描述
    confidence: float      # 置信度 [0, 1]
    conflict_detected: bool  # 是否检测到冲突
    time_to_conflict: float   # 距冲突时间（秒），无冲突为 inf
    vy: float = 0.0        # 推荐y方向线速度（世界坐标系，用于flee精确方向）


# ====================================================================
# 1. 多步轨迹预测器
# ====================================================================

class TrajectoryPredictor:
    """多步轨迹预测器

    基于行人的当前位置、速度和运动模式，预测未来N步的轨迹。

    预测模型:
        - 线性运动: position(t) = p0 + v*t + 0.5*a*t²
        - 随机运动: 使用 Ornstein-Uhlenbeck 过程模拟随机游走
        - 混合模型: 前1/3时间用线性，后2/3逐渐增加不确定性

    不确定性增长:
        sigma(t) = sigma0 * sqrt(t)  (布朗运动模型)
        预测轨迹形成一个"锥形"的不确定区域
    """

    def __init__(self, horizon: float = 3.0, dt: float = 1/30):
        """初始化轨迹预测器

        Args:
            horizon: 预测时间范围（秒），默认3秒
            dt: 时间步长（秒），默认1/30秒
        """
        self.horizon = horizon
        self.dt = dt
        self.num_steps = int(horizon / dt)
        # 基础不确定性（每秒）
        self.sigma0 = 0.15  # m/√s
        # 加速度估计历史
        self._velocity_history: Dict[str, deque] = {}

    def predict(self, obs_x: float, obs_y: float,
                obs_vx: float, obs_vy: float,
                pattern: str = 'linear',
                name: str = '',
                ) -> List[PredictedPoint]:
        """预测行人未来轨迹

        Args:
            obs_x, obs_y: 行人当前位置
            obs_vx, obs_vy: 行人当前速度
            pattern: 运动模式 ('linear' / 'random')
            name: 行人名称（用于跟踪速度历史）

        Returns:
            预测轨迹点列表，长度 num_steps
        """
        trajectory = []
        # 估计加速度（基于速度历史）
        ax, ay = self._estimate_acceleration(name, obs_vx, obs_vy)

        for i in range(1, self.num_steps + 1):
            t = i * self.dt
            # 线性预测 + 加速度修正
            px = obs_x + obs_vx * t + 0.5 * ax * t * t
            py = obs_y + obs_vy * t + 0.5 * ay * t * t

            # 随机运动模式：增加随机扰动
            if pattern == 'random':
                # 不确定性随时间增长
                sigma = self.sigma0 * math.sqrt(t)
                # 用确定性种子保证同一帧内一致
                noise_x = sigma * math.sin(i * 0.7) * 0.5
                noise_y = sigma * math.cos(i * 1.3) * 0.5
                px += noise_x
                py += noise_y

            # 边界限制（避免预测到墙外）
            trajectory.append(PredictedPoint(x=px, y=py, t=t))

        return trajectory

    def _estimate_acceleration(self, name: str,
                                vx: float, vy: float) -> Tuple[float, float]:
        """基于速度历史估计加速度

        使用最近5帧的速度差分估计加速度。
        """
        if name not in self._velocity_history:
            self._velocity_history[name] = deque(maxlen=5)

        history = self._velocity_history[name]
        history.append((vx, vy))

        if len(history) < 2:
            return (0.0, 0.0)

        # 最近两帧的速度差
        old_vx, old_vy = history[0]
        new_vx, new_vy = history[-1]
        n = len(history) - 1
        if n == 0:
            return (0.0, 0.0)

        ax = (new_vx - old_vx) / (n * self.dt)
        ay = (new_vy - old_vy) / (n * self.dt)

        # 限幅，避免异常加速度
        max_a = 1.0  # m/s²
        ax = max(-max_a, min(max_a, ax))
        ay = max(-max_a, min(max_a, ay))

        return (ax, ay)

    def get_uncertainty(self, t: float) -> float:
        """获取时间 t 处的位置不确定性（标准差）

        Args:
            t: 时间偏移（秒）

        Returns:
            不确定性（米）
        """
        return self.sigma0 * math.sqrt(max(t, 0.0))


# ====================================================================
# 2. 时空冲突图
# ====================================================================

class SpatioTemporalConflictGraph:
    """时空冲突图

    将机器人规划路径与所有行人预测轨迹构建时空冲突图，
    检测未来是否会发生碰撞，并找出最早的冲突事件。

    冲突判定:
        对于每个时间步 t，检查机器人位置与行人预测位置的距离。
        如果距离 < safe_distance + uncertainty(t)，则标记为冲突。
    """

    def __init__(self, safe_distance: float = 0.55,
                 robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30):
        """初始化冲突图

        Args:
            safe_distance: 基础安全距离（米）
            robot_radius: 机器人半径
            obstacle_radius: 障碍物半径
        """
        self.base_safe_dist = safe_distance
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius

    def detect_conflicts(self,
                         robot_path: List[Tuple[float, float, float]],
                         obstacles: List[Tuple[str, List[PredictedPoint],
                                               float, float]],
                         ) -> List[ConflictEvent]:
        """检测所有时空冲突

        Args:
            robot_path: 机器人未来路径 [(x, y, t), ...]
            obstacles: 行人预测数据列表 [(name, trajectory, vx, vy), ...]

        Returns:
            冲突事件列表（按时间排序）
        """
        conflicts = []

        for name, trajectory, obs_vx, obs_vy in obstacles:
            for i, (rx, ry, rt) in enumerate(robot_path):
                if i >= len(trajectory):
                    break

                pp = trajectory[i]
                dist = math.sqrt((rx - pp.x) ** 2 + (ry - pp.y) ** 2)

                # 不确定性随时间增长
                uncertainty = self.sigma_growth(pp.t)
                effective_safe = (self.base_safe_dist
                                   + uncertainty * 0.5)

                if dist < effective_safe:
                    # 计算严重程度：距离越近越严重
                    severity = 1.0 - (dist / effective_safe)
                    severity = max(0.0, min(1.0, severity))

                    # 计算冲突时的速度
                    robot_speed = self._estimate_speed_at(robot_path, i)
                    obs_speed = math.sqrt(obs_vx ** 2 + obs_vy ** 2)

                    conflicts.append(ConflictEvent(
                        position=((rx + pp.x) / 2, (ry + pp.y) / 2),
                        time=rt,
                        obstacle_name=name,
                        severity=severity,
                        robot_speed_at_conflict=robot_speed,
                        obstacle_speed_at_conflict=obs_speed,
                    ))
                    break  # 每个行人只记录最早的冲突

        # 按时间排序
        conflicts.sort(key=lambda c: c.time)
        return conflicts

    def sigma_growth(self, t: float) -> float:
        """不确定性随时间增长

        使用平方根模型：sigma(t) = sigma0 * sqrt(t)
        """
        return 0.15 * math.sqrt(max(t, 0.0))

    def _estimate_speed_at(self, path: List[Tuple[float, float, float]],
                           index: int) -> float:
        """估计机器人在路径某点的速度"""
        if index == 0 or len(path) < 2:
            return 0.0
        prev = path[index - 1]
        curr = path[index]
        dt = curr[2] - prev[2]
        if dt <= 0:
            return 0.0
        dx = curr[0] - prev[0]
        dy = curr[1] - prev[1]
        return math.sqrt(dx ** 2 + dy ** 2) / dt

    def get_earliest_conflict(self, conflicts: List[ConflictEvent]
                               ) -> Optional[ConflictEvent]:
        """获取最早的冲突事件"""
        if not conflicts:
            return None
        return min(conflicts, key=lambda c: c.time)


# ====================================================================
# 3. 速度障碍锥
# ====================================================================

class VelocityObstacleCone:
    """速度障碍锥 (Velocity Obstacle)

    对于每个行人，在机器人速度空间中构建一个锥形禁止区域。
    如果机器人速度落入此锥体，将在未来与该行人碰撞。

    VO 的构建:
        1. 计算行人相对机器人的位置向量 r = (obstacle - robot) / |r|
        2. 计算锥的半角 alpha = arcsin((R_robot + R_obs) / |r|)
        3. 锥的轴线方向为 r，半角为 alpha
        4. 锥顶为行人速度向量（相对速度空间）

    速度选择:
        在速度空间中采样候选速度，选择不落入任何锥体且
        最接近目标方向的速度。
    """

    def __init__(self, robot_radius: float = 0.25,
                 obstacle_radius: float = 0.30):
        """初始化速度障碍锥计算器

        Args:
            robot_radius: 机器人半径
            obstacle_radius: 障碍物半径
        """
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.combined_radius = robot_radius + obstacle_radius

    def compute_vo(self, robot_x: float, robot_y: float,
                    obs_x: float, obs_y: float,
                    obs_vx: float, obs_vy: float,
                    ) -> Optional[VelocityObstacle]:
        """计算单个行人的速度障碍锥

        Args:
            robot_x, robot_y: 机器人位置
            obs_x, obs_y: 行人位置
            obs_vx, obs_vy: 行人速度

        Returns:
            速度障碍锥，距离太远返回 None
        """
        # 相对位置向量
        dx = obs_x - robot_x
        dy = obs_y - robot_y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        # 太远不构成威胁
        if dist > 5.0 or dist < 1e-6:
            return None

        # 锥的半角
        if dist < self.combined_radius:
            # 已经碰撞，锥覆盖整个空间
            alpha = math.pi
        else:
            alpha = math.asin(self.combined_radius / dist)

        # 锥的轴线方向（从机器人指向行人）
        axis_dir = (dx / dist, dy / dist)

        # 锥的左右边界方向（旋转 ±alpha）
        cos_a = math.cos(alpha)
        sin_a = math.sin(alpha)
        left_dir = (axis_dir[0] * cos_a - axis_dir[1] * sin_a,
                    axis_dir[0] * sin_a + axis_dir[1] * cos_a)
        right_dir = (axis_dir[0] * cos_a + axis_dir[1] * sin_a,
                     -axis_dir[0] * sin_a + axis_dir[1] * cos_a)

        return VelocityObstacle(
            apex=(obs_vx, obs_vy),
            left_dir=left_dir,
            right_dir=right_dir,
            obstacle_name='',
            distance=dist,
        )

    def is_velocity_safe(self, vx: float, vy: float,
                          vo: VelocityObstacle) -> bool:
        """检查给定速度是否在速度障碍锥外（安全）

        速度 v 相对于行人的相对速度为 v_rel = v - v_obs。
        如果 v_rel 的方向不在锥体内，则安全。

        Args:
            vx, vy: 候选速度
            vo: 速度障碍锥

        Returns:
            True 如果速度安全
        """
        # 相对速度
        rvx = vx - vo.apex[0]
        rvy = vy - vo.apex[1]

        # 检查相对速度方向是否在锥体内
        # 使用叉积判断方向
        cross_left = (rvx * vo.left_dir[1] - rvy * vo.left_dir[0])
        cross_right = (rvx * vo.right_dir[1] - rvy * vo.right_dir[0])

        # 如果相对速度为零，认为不安全
        if abs(rvx) < 1e-6 and abs(rvy) < 1e-6:
            return False

        # 相对速度方向在锥体左侧或右侧之外则安全
        # 具体判断取决于锥体的朝向
        # 简化：如果相对速度与轴线方向的夹角 > alpha，则安全
        axis_x = (vo.left_dir[0] + vo.right_dir[0]) / 2
        axis_y = (vo.left_dir[1] + vo.right_dir[1]) / 2
        axis_norm = math.sqrt(axis_x ** 2 + axis_y ** 2)
        if axis_norm < 1e-6:
            return False

        rv_norm = math.sqrt(rvx ** 2 + rvy ** 2)
        if rv_norm < 1e-6:
            return False

        cos_angle = (rvx * axis_x + rvy * axis_y) / (rv_norm * axis_norm)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.acos(cos_angle)

        # 锥的半角
        alpha = math.acos(max(-1.0, min(1.0,
            (vo.left_dir[0] * axis_x + vo.left_dir[1] * axis_y) / axis_norm)))

        return angle > alpha

    def find_optimal_velocity(self, robot_x: float, robot_y: float,
                               target_x: float, target_y: float,
                               obstacles: List[Tuple[str, float, float,
                                                     float, float]],
                               max_speed: float = 0.3,
                               num_samples: int = 36,
                               ) -> Tuple[float, float, str]:
        """在速度空间中搜索最优避障速度

        Args:
            robot_x, robot_y: 机器人位置
            target_x, target_y: 目标位置
            obstacles: 行人列表 [(name, ox, oy, ovx, ovy), ...]
            max_speed: 最大速度
            num_samples: 速度方向采样数

        Returns:
            (vx, vy, reason) 最优速度和原因
        """
        # 目标方向
        target_ang = math.atan2(target_y - robot_y, target_x - robot_x)

        # 计算所有行人的 VO
        vos = []
        for name, ox, oy, ovx, ovy in obstacles:
            vo = self.compute_vo(robot_x, robot_y, ox, oy, ovx, ovy)
            if vo is not None:
                vo.obstacle_name = name
                vos.append(vo)

        if not vos:
            # 无威胁，直接朝目标
            vx = max_speed * math.cos(target_ang)
            vy = max_speed * math.sin(target_ang)
            return (vx, vy, 'cruise_no_threat')

        # 在速度空间中采样候选速度
        best_vx = 0.0
        best_vy = 0.0
        best_score = -float('inf')
        best_reason = 'no_safe_velocity'

        for i in range(num_samples):
            # 采样角度
            ang = 2 * math.pi * i / num_samples
            # 采样速度大小（多个档位）
            for speed_ratio in [1.0, 0.75, 0.5, 0.25]:
                speed = max_speed * speed_ratio
                vx = speed * math.cos(ang)
                vy = speed * math.sin(ang)

                # 检查是否避开所有 VO
                safe = True
                min_dist = float('inf')
                for vo in vos:
                    if not self.is_velocity_safe(vx, vy, vo):
                        safe = False
                        break
                    min_dist = min(min_dist, vo.distance)

                if not safe:
                    continue

                # 计算评分：朝目标方向 + 远离障碍物
                # 评分 = 朝向目标 * w1 + 速度大小 * w2 + 最小距离 * w3
                w1, w2, w3 = 1.0, 0.3, 0.2
                dir_score = math.cos(ang - target_ang)  # [-1, 1]
                speed_score = speed_ratio
                dist_score = min(min_dist / 5.0, 1.0)
                score = w1 * dir_score + w2 * speed_score + w3 * dist_score

                if score > best_score:
                    best_score = score
                    best_vx = vx
                    best_vy = vy
                    best_reason = 'avoid'

        if best_score == -float('inf'):
            # 所有方向都不安全，原地停止
            return (0.0, 0.0, 'emergency_stop_all_blocked')

        return (best_vx, best_vy, best_reason)


# ====================================================================
# 4. 热点记忆
# ====================================================================

class HotspotMemory:
    """碰撞热点记忆

    记录历史碰撞和近距离规避的位置，构建热度图。
    经过高热度区域时自动提高警惕等级。

    热度衰减模型:
        每个热点有初始热度1.0，随时间指数衰减
        hot(t) = hot0 * exp(-t / tau)
        tau = 300秒（5分钟半衰期）

    空间扩散:
        热点的影响范围 = base_radius + hot * expand_radius
        越热的热点影响范围越大
    """

    def __init__(self, decay_tau: float = 300.0,
                 base_radius: float = 0.5,
                 expand_radius: float = 1.0,
                 grid_resolution: float = 0.5,
                 map_size: Tuple[float, float, float, float] = (-6, -4, 6, 4)):
        """初始化热点记忆

        Args:
            decay_tau: 热度衰减时间常数（秒）
            base_radius: 基础影响半径
            expand_radius: 最大扩展半径
            grid_resolution: 热度图分辨率
            map_size: 地图范围 (xmin, ymin, xmax, ymax)
        """
        self.decay_tau = decay_tau
        self.base_radius = base_radius
        self.expand_radius = expand_radius
        self.grid_resolution = grid_resolution
        self.map_bounds = map_size

        # 热点列表: [(x, y, hot, timestamp), ...]
        self._hotspots: List[List[float]] = []
        # 热度图网格
        gx = int((map_size[2] - map_size[0]) / grid_resolution)
        gy = int((map_size[3] - map_size[1]) / grid_resolution)
        self._heat_grid = np.zeros((gy, gx))
        self._grid_origin = (map_size[0], map_size[1])

        # 统计
        self.total_events = 0
        self.near_miss_events = 0

    def record_event(self, x: float, y: float,
                     severity: float = 1.0,
                     is_near_miss: bool = False,
                     timestamp: Optional[float] = None):
        """记录一个碰撞/近距离规避事件

        Args:
            x, y: 事件位置
            severity: 严重程度 [0, 1]
            is_near_miss: 是否为近距离规避
            timestamp: 事件时间戳（默认当前时间）
        """
        if timestamp is None:
            timestamp = time.time()

        self._hotspots.append([x, y, severity, timestamp])
        self.total_events += 1
        if is_near_miss:
            self.near_miss_events += 1

        # 更新热度图
        self._update_heat_grid(x, y, severity)

    def _update_heat_grid(self, x: float, y: float, severity: float):
        """更新热度图"""
        cx = int((x - self._grid_origin[0]) / self.grid_resolution)
        cy = int((y - self._grid_origin[1]) / self.grid_resolution)

        if 0 <= cy < self._heat_grid.shape[0] and 0 <= cx < self._heat_grid.shape[1]:
            self._heat_grid[cy, cx] += severity

    def get_heat_level(self, x: float, y: float,
                       current_time: Optional[float] = None
                       ) -> float:
        """获取某位置的热度等级

        Args:
            x, y: 查询位置
            current_time: 当前时间（默认 time.time()）

        Returns:
            热度等级 [0, 1]，0 表示无历史碰撞
        """
        if current_time is None:
            current_time = time.time()

        total_heat = 0.0

        for hs in self._hotspots:
            hx, hy, hot0, ts = hs
            # 距离衰减
            dist = math.sqrt((x - hx) ** 2 + (y - hy) ** 2)
            influence_radius = self.base_radius + hot0 * self.expand_radius
            if dist > influence_radius:
                continue

            # 时间衰减
            dt = current_time - ts
            time_decay = math.exp(-dt / self.decay_tau)

            # 距离衰减（高斯）
            spatial_decay = math.exp(-(dist ** 2) / (2 * (influence_radius / 2) ** 2))

            total_heat += hot0 * time_decay * spatial_decay

        return min(total_heat, 1.0)

    def get_alertness_level(self, x: float, y: float) -> Tuple[float, float, float]:
        """获取某位置的警惕等级

        根据热度等级返回相应的参数调整建议

        Args:
            x, y: 查询位置

        Returns:
            (speed_limit_factor, predict_horizon_multiplier, alert_level)
            - speed_limit_factor: 速度限制系数 [0.3, 1.0]
            - predict_horizon_multiplier: 预测范围倍数 [1.0, 2.0]
            - alert_level: 警惕等级 [0, 1]
        """
        heat = self.get_heat_level(x, y)

        # 热度越高，速度限制越大，预测范围越大
        speed_factor = 1.0 - heat * 0.7  # 最低降到 0.3 倍速
        horizon_mult = 1.0 + heat * 1.0  # 最高 2 倍预测范围

        return (speed_factor, horizon_mult, heat)

    def decay(self, current_time: Optional[float] = None):
        """衰减所有热点（清理过期数据）

        Args:
            current_time: 当前时间
        """
        if current_time is None:
            current_time = time.time()

        # 移除热度衰减到很小的热点
        self._hotspots = [
            hs for hs in self._hotspots
            if hs[2] * math.exp(-(current_time - hs[3]) / self.decay_tau) > 0.01
        ]

        # 衰减热度图
        self._heat_grid *= 0.99  # 缓慢衰减

    def get_hotspots(self) -> List[Tuple[float, float, float]]:
        """获取当前活跃的热点列表

        Returns:
            [(x, y, heat_level), ...]
        """
        result = []
        for hs in self._hotspots:
            x, y, hot0, ts = hs
            dt = time.time() - ts
            heat = hot0 * math.exp(-dt / self.decay_tau)
            if heat > 0.05:
                result.append((x, y, heat))
        return result

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        return {
            'total_events': self.total_events,
            'near_miss_events': self.near_miss_events,
            'active_hotspots': len(self._hotspots),
            'max_heat': float(np.max(self._heat_grid)) if self._heat_grid.size > 0 else 0.0,
        }


# ====================================================================
# 5. TTC博弈等待决策
# ====================================================================

class TTCGameDecision:
    """TTC博弈等待决策

    基于碰撞时间 (Time To Collision) 的博弈论决策器。
    决定机器人应该等待、通过还是避让。

    决策逻辑:
        - 计算机器人和行人到达冲突点的时间 (TTC_robot, TTC_person)
        - 如果 TTC_person < TTC_robot (行人先到): 机器人等待
        - 如果 TTC_robot < TTC_person * 0.8 (机器人明显先到): 加速通过
        - 如果两者接近: 根据博弈论选择让步（保守策略，等待）
        - 如果 TTC < emergency_threshold: 紧急避让

    博弈论分析:
        - 纯策略纳什均衡: (通过, 等待) 或 (等待, 通过)
        - 保守策略: 当不确定时选择等待（损失更小）
        - 混合策略: 当双方TTC接近时，以概率p选择等待
    """

    def __init__(self, emergency_threshold: float = 1.0,
                 wait_threshold: float = 2.0,
                 pass_ratio: float = 0.8):
        """初始化TTC博弈决策器

        Args:
            emergency_threshold: 紧急避让TTC阈值（秒）
            wait_threshold: 等待TTC阈值（秒）
            pass_ratio: 机器人通过的速度比阈值
        """
        self.emergency_threshold = emergency_threshold
        self.wait_threshold = wait_threshold
        self.pass_ratio = pass_ratio

    def compute_ttc(self, robot_x: float, robot_y: float,
                    robot_vx: float, robot_vy: float,
                    obs_x: float, obs_y: float,
                    obs_vx: float, obs_vy: float,
                    safe_distance: float = 0.55) -> float:
        """计算碰撞时间 (TTC)

        Args:
            robot_x, robot_y: 机器人位置
            robot_vx, robot_vy: 机器人速度
            obs_x, obs_y: 行人位置
            obs_vx, obs_vy: 行人速度
            safe_distance: 安全距离

        Returns:
            TTC（秒），无碰撞风险返回 inf
        """
        # 相对位置和速度
        rx = obs_x - robot_x
        ry = obs_y - robot_y
        rvx = obs_vx - robot_vx
        rvy = obs_vy - robot_vy

        # 相对距离
        dist = math.sqrt(rx ** 2 + ry ** 2)
        if dist < safe_distance:
            return 0.0  # 已经碰撞

        # 相对速度沿接近方向的分量
        if dist < 1e-6:
            return float('inf')

        # 接近速率 = -dot(rel_vel, rel_pos) / |rel_pos|
        closing_rate = -(rvx * rx + rvy * ry) / dist

        if closing_rate <= 0:
            return float('inf')  # 在远离

        # TTC = (dist - safe_distance) / closing_rate
        ttc = (dist - safe_distance) / closing_rate
        return max(ttc, 0.0)

    def decide(self, ttc: float,
               robot_can_pass: bool = True,
               obstacle_approaching: bool = True,
               ) -> Tuple[str, float, str]:
        """TTC博弈决策

        Args:
            ttc: 碰撞时间（秒）
            robot_can_pass: 机器人是否有足够空间通过
            obstacle_approaching: 行人是否在接近

        Returns:
            (action, speed_factor, reason)
            action: 'pass' / 'wait' / 'flee'
            speed_factor: 速度系数 [0, 1.5]
            reason: 决策原因
        """
        if ttc == float('inf') or ttc > self.wait_threshold:
            # 无碰撞风险或很远
            return ('pass', 1.0, 'no_collision_risk')

        if ttc < self.emergency_threshold:
            # 紧急避让
            return ('flee', 0.0, f'emergency_ttc={ttc:.2f}s')

        # 在 wait_threshold 和 emergency_threshold 之间
        if not obstacle_approaching:
            # 行人在远离，可以安全通过
            return ('pass', 1.0, 'obstacle_receding')

        if not robot_can_pass:
            # 没有足够空间通过，等待
            return ('wait', 0.0, 'no_passage_space')

        # 博弈论决策：当 TTC 在中间区间时
        # 使用保守策略：偏向等待（损失更小）
        # 但如果 TTC 较大（接近 wait_threshold），可以减速通过

        if ttc > self.wait_threshold * 0.7:
            # TTC 较大，减速通过
            speed_factor = 0.5
            return ('pass', speed_factor, f'slow_pass_ttc={ttc:.2f}s')
        else:
            # TTC 较小，保守等待
            return ('wait', 0.0, f'conservative_wait_ttc={ttc:.2f}s')


# ====================================================================
# STVOC 主控制器
# ====================================================================

class STVOCController:
    """STVOC 时空速度障碍锥避障控制器

    整合五大组件，提供统一的避障决策接口。

    使用方式:
        controller = STVOCController()

        # 每帧调用
        cmd = controller.compute_avoidance(
            robot_x, robot_y, robot_yaw,
            robot_vx, robot_vy,
            target_x, target_y,
            obstacles=[(name, ox, oy, ovx, ovy, pattern), ...],
        )
        # cmd.vx, cmd.wz 为推荐速度
    """

    def __init__(self, dt: float = 1/30,
                 max_speed: float = 0.3,
                 max_angular: float = 1.2,
                 safe_distance: float = 0.55,
                 horizon: float = 3.0):
        """初始化 STVOC 控制器

        Args:
            dt: 控制周期（秒）
            max_speed: 最大线速度（m/s）
            max_angular: 最大角速度
            safe_distance: 安全距离
            horizon: 预测时间范围（秒）
        """
        self.dt = dt
        self.max_speed = max_speed
        self.max_angular = max_angular
        self.safe_distance = safe_distance

        # 五大组件
        # v5.3: horizon 作为构造参数，仿真传0.5s（max_speed=6.0），
        # 测试用默认3.0s（max_speed=0.3）
        self.predictor = TrajectoryPredictor(horizon=horizon, dt=dt)
        self.conflict_graph = SpatioTemporalConflictGraph(
            safe_distance=safe_distance)
        self.vo_cone = VelocityObstacleCone(
            robot_radius=0.25, obstacle_radius=0.30)
        self.hotspot_memory = HotspotMemory()
        self.ttc_decision = TTCGameDecision()

        # 状态
        self._frame_count = 0
        self._last_action = 'cruise'
        self._wait_count = 0
        self._max_wait = 60  # 最大等待帧数
        self._bottleneck_max_wait = 180  # 瓶颈区域最大等待帧数(6秒)
        self._gap_crossing_active = False  # 是否正在执行间隙穿越
        self._gap_crossing_frames = 0     # 间隙穿越已执行帧数
        self._flee_cooldown = 0           # flee冷却帧数（避免持续flee导致卡住）

        # 统计
        self.stats = {
            'total_decisions': 0,
            'avoid_count': 0,
            'wait_count': 0,
            'flee_count': 0,
            'cruise_count': 0,
            'near_miss_recorded': 0,
            'gap_crossing_count': 0,
        }

    def compute_avoidance(self,
                           robot_x: float, robot_y: float,
                           robot_yaw: float,
                           robot_vx: float, robot_vy: float,
                           target_x: float, target_y: float,
                           obstacles: List[Tuple[str, float, float,
                                                 float, float, str]],
                           ) -> AvoidanceCommand:
        """计算避障指令

        Args:
            robot_x, robot_y: 机器人位置
            robot_yaw: 机器人航向
            robot_vx, robot_vy: 机器人当前速度
            target_x, target_y: 目标位置
            obstacles: 行人列表 [(name, ox, oy, ovx, ovy, pattern), ...]

        Returns:
            避障指令
        """
        self._frame_count += 1
        self.stats['total_decisions'] += 1
        current_time = time.time()

        # === 步骤0a: 热点查询（提前到近距检测之前）===
        speed_factor, horizon_mult, alert = \
            self.hotspot_memory.get_alertness_level(robot_x, robot_y)

        # === 步骤0b: 前瞻近距行人保护（最高优先级）===
        # 创新：预测行人下一帧位置，提前一帧反应
        # 固定阈值0.7m > 碰撞半径0.55m + 行人单步0.04m的余量
        if self._flee_cooldown > 0:
            self._flee_cooldown -= 1
        flee_threshold = 0.7

        # 收集所有近距行人，计算合力flee方向
        flee_x_total = 0.0
        flee_y_total = 0.0
        min_obs_dist = float('inf')
        nearest_info = None
        has_near_obs = False

        for name, ox, oy, ovx, ovy, _ in obstacles:
            # 预测行人下一帧位置（前瞻）
            # v5.3: ovx/ovy 是 m/s，需乘以 dt 得到位移
            next_x = ox + ovx * self.dt
            next_y = oy + ovy * self.dt
            # 当前距离和预测距离
            d = math.sqrt((robot_x - ox)**2 + (robot_y - oy)**2)
            d_next = math.sqrt((robot_x - next_x)**2 + (robot_y - next_y)**2)
            # 取较近的（前瞻检测）
            d_eff = min(d, d_next)
            if d_eff < min_obs_dist:
                min_obs_dist = d_eff
                nearest_info = (name, ox, oy, next_x, next_y)

            # 在阈值内的行人贡献flee方向
            if d_eff < flee_threshold:
                has_near_obs = True
                # 远离行人预测位置（而非当前位置）
                away_dx = robot_x - next_x
                away_dy = robot_y - next_y
                away_norm = math.sqrt(away_dx**2 + away_dy**2)
                if away_norm < 1e-6:
                    td = math.atan2(target_y - robot_y, target_x - robot_x)
                    away_dx = math.cos(td)
                    away_dy = math.sin(td)
                    away_norm = 1.0
                away_dx /= away_norm
                away_dy /= away_norm

                # 纯远离行人预测位置（加权）
                weight = (flee_threshold - d_eff) / flee_threshold
                flee_x_total += weight * away_dx
                flee_y_total += weight * away_dy

        if has_near_obs:
            # 归一化合力方向
            flee_norm = math.sqrt(flee_x_total**2 + flee_y_total**2)
            if flee_norm < 1e-6 and nearest_info is not None:
                # fallback：直接远离最近行人预测位置
                _, _, _, nx, ny = nearest_info
                flee_x_total = robot_x - nx
                flee_y_total = robot_y - ny
                flee_norm = math.sqrt(flee_x_total**2 + flee_y_total**2)
                if flee_norm < 1e-6:
                    td = math.atan2(target_y - robot_y, target_x - robot_x)
                    flee_x_total = math.cos(td)
                    flee_y_total = math.sin(td)
                    flee_norm = 1.0
            flee_x_total /= flee_norm
            flee_y_total /= flee_norm

            # flee方向已含垂直分量（绕开行人路径），直接使用
            flee_speed = self.max_speed * 0.8
            vx = flee_speed * flee_x_total
            vy = flee_speed * flee_y_total
            target_ang = math.atan2(vy, vx)
            wz = self._angle_to_angular(target_ang, robot_yaw)
            self.stats['flee_count'] += 1
            # 不冷却——flee是安全关键操作，必须持续触发
            # 记录到热点
            if nearest_info is not None:
                _, ox, oy, _, _ = nearest_info
                self.hotspot_memory.record_event(
                    ox, oy, severity=0.8, is_near_miss=True)
            return AvoidanceCommand(
                vx=vx, wz=wz, action='flee',
                reason=f'proximity_flee_dist={min_obs_dist:.2f}m_alert={alert:.2f}',
                confidence=0.9,
                conflict_detected=True,
                time_to_conflict=0.0,
                vy=vy,
            )

        # === 步骤1: 调整预测范围（热点已在步骤0a查询）===
        original_horizon = self.predictor.horizon
        self.predictor.horizon = original_horizon * horizon_mult
        self.predictor.num_steps = int(self.predictor.horizon / self.dt)

        # === 步骤2: 多步轨迹预测 ===
        predicted_obstacles = []
        for name, ox, oy, ovx, ovy, pattern in obstacles:
            traj = self.predictor.predict(ox, oy, ovx, ovy, pattern, name)
            predicted_obstacles.append((name, traj, ovx, ovy))

        # === 步骤3: 构建机器人未来路径（朝目标转向的曲线预测）===
        # 创新：不再沿当前航向直线外推，而是模拟机器人朝目标转向的真实轨迹
        # 这样能提前检测到机器人转向后才会发生的碰撞
        robot_path = []
        effective_speed = self.max_speed * speed_factor
        target_dir = math.atan2(target_y - robot_y, target_x - robot_x)
        sim_yaw = robot_yaw
        sim_x, sim_y = robot_x, robot_y
        for i in range(1, self.predictor.num_steps + 1):
            t = i * self.dt
            # 逐步转向目标方向（模拟真实转向约束）
            yaw_diff = math.atan2(
                math.sin(target_dir - sim_yaw),
                math.cos(target_dir - sim_yaw))
            turn = max(-self.max_angular * self.dt,
                       min(self.max_angular * self.dt, yaw_diff))
            sim_yaw += turn
            sim_x += effective_speed * math.cos(sim_yaw) * self.dt
            sim_y += effective_speed * math.sin(sim_yaw) * self.dt
            robot_path.append((sim_x, sim_y, t))

        # === 步骤4: 时空冲突检测 ===
        conflicts = self.conflict_graph.detect_conflicts(
            robot_path, predicted_obstacles)

        # === 步骤5: TTC博弈决策 ===
        if not conflicts:
            # 无冲突，正常巡航
            self.stats['cruise_count'] += 1
            self._last_action = 'cruise'
            self._wait_count = 0
            # 恢复原始预测范围
            self.predictor.horizon = original_horizon
            self.predictor.num_steps = int(original_horizon / self.dt)

            vx = effective_speed * math.cos(
                math.atan2(target_y - robot_y, target_x - robot_x))
            vy = effective_speed * math.sin(
                math.atan2(target_y - robot_y, target_x - robot_x))
            wz = 0.0

            return AvoidanceCommand(
                vx=vx, wz=wz, action='cruise',
                reason='no_conflict',
                confidence=1.0,
                conflict_detected=False,
                time_to_conflict=float('inf'),
                vy=vy,
            )

        # 有冲突，获取最早的冲突时间（来自时空冲突图）
        # 冲突图基于机器人曲线轨迹+行人预测轨迹，比相对速度TTC更准确
        earliest = conflicts[0]
        ttc = earliest.time

        # === 创新点: 时间间隙穿越 (Temporal Gap Crossing) ===
        # v5.3修正: 阈值根据 horizon=0.5s 调整
        # 关键修复：cruise阈值从0.3s提高到0.5s（=horizon）
        # 确保所有检测到的冲突都触发避障，而非忽略0.3~0.5s的冲突
        # 1. 冲突在0.5秒后 → 不可能（horizon=0.5s）
        # 2. 冲突在0.5秒内 + 行人近(<1.2m) → flee
        # 3. 冲突在0.5秒内 + TTC极短(<0.1s) → 紧急避让
        # 4. 冲突在0.5秒内 + 无间隙 → 等待间隙出现
        if ttc > 0.5:
            # 不可能触发（horizon限制），但保留作为安全网
            action = 'cruise'
            speed_mult = 1.0
            reason = f'conflict_far_ttc={ttc:.2f}s'
            self._wait_count = 0
            vx = effective_speed * math.cos(target_dir)
            vy = effective_speed * math.sin(target_dir)
            wz = self._angle_to_angular(target_dir, robot_yaw)
        else:
            # ttc <= 0.5: 冲突在预测范围内，必须处理
            # === 近距行人保护 ===
            # v5.3: 阈值从0.7m提高到1.2m
            # v5.3b: 阈值从1.2m提高到1.5m
            # 问题: wait状态下机器人静止，但行人会继续走向机器人
            # 0.7m太近——行人以1.2m/s移动时0.7m只需0.58秒就碰撞
            # 1.2m给机器人足够时间flee远离
            # 1.5m: 行人1.2m/s时1.25秒才到达，flee一步0.30m可拉开到安全距离
            min_obs_dist = float('inf')
            for _, ox, oy, _, _, _ in obstacles:
                d = math.sqrt((robot_x - ox)**2 + (robot_y - oy)**2)
                if d < min_obs_dist:
                    min_obs_dist = d

            if min_obs_dist < 1.5:
                # 行人近，必须主动远离而非等待
                action = 'flee'
                speed_mult = 0.0
                reason = f'proximity_flee_dist={min_obs_dist:.2f}m'
                self._wait_count = 0
            elif (gap_time := self._find_temporal_gap(
                    robot_path, predicted_obstacles,
                    min_gap_duration=0.3)) is not None and gap_time < 0.1:
                # 间隙就在当前，快速通过
                action = 'avoid'
                speed_mult = 1.0
                reason = f'gap_crossing_now_gap={gap_time:.2f}s'
                self._gap_crossing_active = True
                self._gap_crossing_frames = 0
                self.stats['gap_crossing_count'] += 1
                self._wait_count = 0
            elif ttc < 0.1:
                # 极近距紧急避让（冲突图已检测到即将碰撞）
                # 注意: 使用冲突图ttc而非compute_ttc的min_ttc
                # 因为冲突图基于机器人曲线轨迹+行人预测轨迹，更准确
                action = 'flee'
                speed_mult = 0.0
                reason = f'emergency_ttc={ttc:.2f}s'
                self._wait_count = 0
            else:
                # 等待间隙出现——即使TTC较短也等待，避免盲目穿越瓶颈
                action = 'wait'
                speed_mult = 0.0
                reason = f'waiting_for_gap_ttc={ttc:.2f}s'
                self._wait_count += 1
                self.stats['wait_count'] += 1
                if self._wait_count > self._bottleneck_max_wait:
                    # 等待超过阈值，强制通过（降速）
                    action = 'avoid'
                    speed_mult = 0.5
                    reason = 'wait_timeout_force_cross'

        # === 步骤6: 速度障碍锥最优速度选择 ===
        if action == 'wait':
            vx, vy = 0.0, 0.0
            wz = 0.0
        elif action == 'flee':
            # 紧急避让：直接远离最近行人（不走目标方向，优先保命）
            # 找最近行人
            nearest_x, nearest_y = 0.0, 0.0
            nearest_d2 = float('inf')
            for _, ox, oy, _, _, _ in obstacles:
                d2 = (robot_x - ox)**2 + (robot_y - oy)**2
                if d2 < nearest_d2:
                    nearest_d2 = d2
                    nearest_x = ox
                    nearest_y = oy
            # 远离方向 = 机器人 - 行人
            away_dx = robot_x - nearest_x
            away_dy = robot_y - nearest_y
            away_norm = math.sqrt(away_dx**2 + away_dy**2)
            if away_norm < 1e-6:
                # 行人在正上方，向目标方向逃
                away_dx = math.cos(target_dir)
                away_dy = math.sin(target_dir)
                away_norm = 1.0
            flee_speed = self.max_speed * 0.7
            vx = flee_speed * (away_dx / away_norm)
            vy = flee_speed * (away_dy / away_norm)
            target_ang = math.atan2(vy, vx)
            wz = self._angle_to_angular(target_ang, robot_yaw)
            self.stats['flee_count'] += 1
        elif action == 'cruise':
            # 冲突尚远，已在上方计算速度，直接使用
            self.stats['cruise_count'] += 1
        else:
            # 避让：速度障碍锥找最优速度
            vo_obstacles = [(name, ox, oy, ovx, ovy)
                           for name, ox, oy, ovx, ovy, _ in obstacles]
            vx, vy, vo_reason = self.vo_cone.find_optimal_velocity(
                robot_x, robot_y, target_x, target_y,
                vo_obstacles,
                max_speed=self.max_speed * speed_mult * speed_factor)
            target_ang = math.atan2(vy, vx)
            wz = self._angle_to_angular(target_ang, robot_yaw)
            self.stats['avoid_count'] += 1

        # 记录近距规避到热点记忆（帮助未来通过此区域时提前减速）
        self.hotspot_memory.record_event(
            earliest.position[0], earliest.position[1],
            severity=earliest.severity * 0.5, is_near_miss=True)
        self.stats['near_miss_recorded'] += 1

        self._last_action = action

        # 恢复原始预测范围
        self.predictor.horizon = original_horizon
        self.predictor.num_steps = int(original_horizon / self.dt)

        return AvoidanceCommand(
            vx=vx, wz=wz, action=action,
            reason=f'{reason}_conflict_at_t={ttc:.2f}s',
            confidence=1.0 - earliest.severity * 0.5,
            conflict_detected=True,
            time_to_conflict=ttc,
            vy=vy,
        )

    def _find_temporal_gap(self, robot_path, predicted_obstacles,
                           min_gap_duration: float = 2.0) -> Optional[float]:
        """寻找时间间隙：行人轨迹中机器人可以安全穿越的时间窗口

        创新点：当机器人必须穿越行人路径时（如门道、走廊交叉），
        不再盲目等待固定时间，而是分析行人预测轨迹，找到
        连续 min_gap_duration 秒的清空窗口，在窗口开始时快速通过。

        Args:
            robot_path: 机器人预测路径 [(x, y, t), ...]
            predicted_obstacles: 行人预测数据 [(name, traj, vx, vy), ...]
            min_gap_duration: 最小清空持续时间（秒）

        Returns:
            间隙开始时间（秒），无间隙返回 None
        """
        if not robot_path or not predicted_obstacles:
            return 0.0

        # 对每个时间步，检查是否所有行人都远离机器人路径
        gap_start = None
        gap_length = 0.0
        dt = robot_path[0][2] if len(robot_path) > 0 else self.dt

        for i, (rx, ry, rt) in enumerate(robot_path):
            all_clear = True
            for name, traj, _, _ in predicted_obstacles:
                if i >= len(traj):
                    continue
                pp = traj[i]
                dist = math.sqrt((rx - pp.x) ** 2 + (ry - pp.y) ** 2)
                # 安全距离 + 不确定性余量
                uncertainty = 0.15 * math.sqrt(max(rt, 0.0))
                effective_safe = self.safe_distance + 0.3 + uncertainty
                if dist < effective_safe:
                    all_clear = False
                    break

            if all_clear:
                if gap_start is None:
                    gap_start = rt
                    gap_length = dt
                else:
                    gap_length += dt
                if gap_length >= min_gap_duration:
                    return gap_start
            else:
                gap_start = None
                gap_length = 0.0

        return None

    def _angle_to_angular(self, target_ang: float,
                          current_yaw: float) -> float:
        """将角度差转换为角速度"""
        diff = target_ang - current_yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        # 比例控制，限幅
        wz = diff * 5.0  # 增益
        return max(-self.max_angular, min(self.max_angular, wz))

    def record_collision(self, x: float, y: float,
                         severity: float = 1.0):
        """记录碰撞事件到热点记忆"""
        self.hotspot_memory.record_event(x, y, severity, is_near_miss=False)

    def record_near_miss(self, x: float, y: float,
                         severity: float = 0.5):
        """记录近距离规避事件到热点记忆"""
        self.hotspot_memory.record_event(x, y, severity, is_near_miss=True)

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        stats = self.stats.copy()
        stats['hotspot'] = self.hotspot_memory.get_stats()
        stats['frame_count'] = self._frame_count
        return stats

    def decay_hotspots(self):
        """衰减热点记忆（定期调用）"""
        self.hotspot_memory.decay()
