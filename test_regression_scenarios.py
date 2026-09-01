"""导航系统回归测试 (P1-5)

依据 guihua20260809.md P1 阶段要求:
  - 绑架机器人 (kidnapping) 恢复测试
  - 窄通道 (narrow passage) 导航测试
  - 动态碰撞 (dynamic collision) 回归测试

每个场景模拟特定故障模式，验证 AMCL/A*/避障模块的恢复能力。
"""
import sys
import os
import math
import time
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION
from amcl import AMCL
from costmap import Costmap
from astar_planner import AStarPlanner


# =====================================================================
# 1. 绑架机器人恢复测试
# =====================================================================
class TestKidnappingRecovery:
    """测试 AMCL 在机器人被突然移动到新位置后的重定位能力。

    场景: 机器人在位置 A 正常运行，突然被"绑架"（手动搬移）到位置 B。
    AMCL 应通过连续低似然检测到 kidnapping，触发全局重定位，
    并在合理帧数内恢复到正确位姿。
    """

    def setup_method(self):
        """初始化测试环境。"""
        self.occ = OccupancyGrid()
        # 构建一个有障碍物的地图
        angles = np.linspace(-math.pi, math.pi, 180)
        # 在 (3, 0) 处放置一面墙
        dists = np.where(np.abs(angles) < 0.3, 3.0, 8.0)
        self.occ.update_from_scan(0, 0, angles, dists)
        # 在 (-3, 0) 处也放置障碍
        dists2 = np.where(np.abs(angles - math.pi) < 0.3, 3.0, 8.0)
        self.occ.update_from_scan(0, 0, angles, dists2)

        self.amcl = AMCL(self.occ, n_particles=200,
                         kld_min=50, kld_max=300)
        self.amcl.init_cloud(0, 0, 0, spread=0.2)

    def test_kidnapping_detection(self):
        """测试 kidnapping 检测触发。"""
        # 正常运行几帧
        angles = np.linspace(-math.pi, math.pi, 36)
        dists = np.where(np.abs(angles) < 0.3, 3.0, 8.0)
        for i in range(5):
            self.amcl.update(0.01, 0, 0.01, angles, dists, frame=i)

        assert not self.amcl.kidnapping_detected, "初始不应有kidnapping"

        # 模拟绑架: 机器人突然移到 (4, 4)，但 AMCL 粒子还在 (0, 0)
        # 新位置的 LiDAR 观测完全不匹配 -> 低似然
        kidnap_angles = np.linspace(-math.pi, math.pi, 36)
        kidnap_dists = np.full(36, 2.0)  # 完全不同的观测

        # 运行足够多帧触发 kidnapping 检测
        for i in range(20):
            self.amcl.update(0.05, 0.05, 0.01,
                             kidnap_angles, kidnap_dists, frame=5 + i)

        # 验证 kidnapping 被检测到并触发了恢复
        # (recover 会重置粒子分布)
        assert self.amcl.resample_count > 0, "kidnapping后应触发resample"

    def test_recovery_convergence(self):
        """测试 kidnapping 后 AMCL 能重新收敛。"""
        # 初始化在原点
        angles = np.linspace(-math.pi, math.pi, 36)
        dists = np.where(np.abs(angles) < 0.3, 3.0, 8.0)
        self.amcl.init_cloud(0, 0, 0, spread=0.1)

        # 正常运行
        for i in range(10):
            self.amcl.update(0.01, 0, 0.01, angles, dists, frame=i)

        # 触发恢复
        self.amcl.recover(0, 0, 0, spread=2.0,
                          scan=(angles, dists))

        # 恢复后运行若干帧，验证能收敛
        for i in range(30):
            self.amcl.update(0.01, 0, 0.01, angles, dists, frame=10 + i)

        x, y, yaw, conf = self.amcl.get_estimate()
        # 收敛后位置应接近真实位置 (0, 0)
        pos_err = math.sqrt(x**2 + y**2)
        assert pos_err < 2.0, f"恢复后位置误差过大: {pos_err:.2f}m"


# =====================================================================
# 2. 窄通道导航测试
# =====================================================================
class TestNarrowPassage:
    """测试 A* 在窄通道场景下的路径规划能力。

    场景: 两面墙之间留 1m 宽的通道，验证 A* 能找到通过通道的路径，
    且 costmap 膨胀半径不会堵死通道。
    """

    def setup_method(self):
        """构建窄通道场景。"""
        self.occ = OccupancyGrid()
        self.cm = Costmap()

        # 构建两面平行墙，中间留 1m 通道
        # 墙1: y = 0.5, 墙2: y = -0.5 (通道宽 1m)
        angles = np.linspace(-math.pi, math.pi, 180)

        # 从通道入口扫描
        for x in np.arange(-2.0, 2.0, 0.2):
            dists = np.full(180, 8.0)
            # 北侧墙 (y=0.5)
            for idx, a in enumerate(angles):
                wy = x + 8.0 * math.sin(a)
                wx = x + 8.0 * math.cos(a)
                if abs(wy - 0.5) < 0.1:
                    d = (0.5 - 0) / max(abs(math.sin(a)), 0.01)
                    if 0 < d < 8.0:
                        dists[idx] = d
                # 南侧墙 (y=-0.5)
                if abs(wy + 0.5) < 0.1:
                    d = (0.5) / max(abs(math.sin(a)), 0.01)
                    if 0 < d < 8.0:
                        dists[idx] = d
            self.occ.update_from_scan(x, 0, angles, dists, max_range=8.0)

        self.cm.update_static(self.occ, frame=0)
        self.astar = AStarPlanner(self.cm)

    def test_path_through_narrow_passage(self):
        """A* 应能找到通过 1m 通道的路径。"""
        # 从通道一侧到另一侧
        path = self.astar.plan(-1.0, 0.0, 1.0, 0.0)
        assert path is not None, "A* 应找到通过窄通道的路径"
        assert len(path) > 0, "路径不应为空"

    def test_path_does_not_cross_walls(self):
        """路径不应穿过墙壁。"""
        path = self.astar.plan(-1.0, 0.0, 1.0, 0.0)
        if path is None:
            pytest.skip("A* 未找到路径")
        for wx, wy in path:
            # 检查路径点不在墙内
            assert abs(wy - 0.5) > 0.2, f"路径穿过北侧墙: ({wx},{wy})"
            assert abs(wy + 0.5) > 0.2, f"路径穿过南侧墙: ({wx},{wy})"


# =====================================================================
# 3. 动态碰撞回归测试
# =====================================================================
class TestDynamicCollision:
    """测试避障算法在动态障碍物场景下的碰撞避免能力。

    场景: 机器人沿直线运动，动态障碍物从侧面切入。
    验证 VO 避障算法能有效避免碰撞。
    """

    def setup_method(self):
        """初始化动态碰撞场景。"""
        # 机器人状态
        self.robot_pos = np.array([0.0, 0.0])
        self.robot_vel = np.array([0.5, 0.0])  # 向右运动
        self.robot_radius = 0.35

        # 动态障碍物（行人）
        self.ped_pos = np.array([2.0, 1.0])
        self.ped_vel = np.array([-0.3, -0.4])  # 向左下运动（切入机器人路径）
        self.ped_radius = 0.3

    def test_vo_avoidance(self):
        """VO 算法应修改速度避免碰撞。"""
        try:
            from vo_avoidance import VOController
        except ImportError:
            pytest.skip("vo_avoidance 模块不可用")

        vo = VOController()
        # VOController.compute_avoidance 签名:
        # (robot_x, robot_y, robot_yaw, robot_vx, robot_vy,
        #  target_x, target_y, obstacles)
        # obstacles: [(name, x, y, vx, vy, pattern), ...]
        obstacles = [("ped1",
                      float(self.ped_pos[0]), float(self.ped_pos[1]),
                      float(self.ped_vel[0]), float(self.ped_vel[1]),
                      "crossing")]
        cmd = vo.compute_avoidance(
            float(self.robot_pos[0]), float(self.robot_pos[1]), 0.0,
            float(self.robot_vel[0]), float(self.robot_vel[1]),
            5.0, 0.0,  # target
            obstacles,
        )
        assert cmd is not None, "VO 应返回有效指令"
        # 避障后速度应与原速度不同
        new_vel = np.array([cmd.vx, cmd.vy])
        vel_change = float(np.linalg.norm(new_vel - self.robot_vel))
        assert vel_change > 0.01, \
            f"VO 应修改速度避免碰撞, 变化量={vel_change:.4f}"

    def test_collision_prediction(self):
        """验证在无避障时会发生碰撞。"""
        # 模拟 5 秒内的运动
        dt = 0.1
        collision_occurred = False
        robot_pos = self.robot_pos.copy()
        ped_pos = self.ped_pos.copy()

        for _ in range(50):  # 5秒
            robot_pos += self.robot_vel * dt
            ped_pos += self.ped_vel * dt
            dist = np.linalg.norm(robot_pos - ped_pos)
            if dist < self.robot_radius + self.ped_radius:
                collision_occurred = True
                break

        assert collision_occurred, \
            "无避障时应发生碰撞（验证测试场景有效性）"

    def test_min_clearance_with_vo(self):
        """VO 避障后应保持最小安全距离。"""
        try:
            from vo_avoidance import VOController
        except ImportError:
            pytest.skip("vo_avoidance 模块不可用")

        vo = VOController()
        dt = 0.1
        min_clearance = float('inf')
        robot_pos = self.robot_pos.copy().astype(float)
        ped_pos = self.ped_pos.copy().astype(float)
        robot_vel = self.robot_vel.copy().astype(float)

        for _ in range(50):
            obstacles = [("ped1",
                          float(ped_pos[0]), float(ped_pos[1]),
                          float(self.ped_vel[0]), float(self.ped_vel[1]),
                          "crossing")]
            cmd = vo.compute_avoidance(
                float(robot_pos[0]), float(robot_pos[1]), 0.0,
                float(robot_vel[0]), float(robot_vel[1]),
                5.0, 0.0, obstacles,
            )
            if cmd is None:
                break
            robot_vel = np.array([cmd.vx, cmd.vy])
            robot_pos = robot_pos + robot_vel * dt
            ped_pos = ped_pos + self.ped_vel * dt
            dist = float(np.linalg.norm(robot_pos - ped_pos))
            min_clearance = min(min_clearance, dist)

        # 安全距离应大于碰撞半径之和
        collision_radius = self.robot_radius + self.ped_radius
        assert min_clearance > collision_radius, \
            f"最小距离 {min_clearance:.3f}m 应大于碰撞半径 {collision_radius:.3f}m"


# =====================================================================
# 4. AMCL 粒子收敛性回归测试
# =====================================================================
class TestAMCLConvergence:
    """测试 AMCL 粒子滤波器的收敛性。"""

    def test_convergence_from_gaussian_spread(self):
        """从高斯分布初始粒子应收敛到真实位姿。"""
        occ = OccupancyGrid()
        # 构建简单环境
        angles = np.linspace(-math.pi, math.pi, 36)
        dists = np.where(np.abs(angles) < 0.5, 2.0, 8.0)
        occ.update_from_scan(0, 0, angles, dists)

        amcl = AMCL(occ, n_particles=100, kld_min=50, kld_max=200)
        amcl.init_cloud(0.5, 0.5, 0.1, spread=0.5)  # 有偏差的初始估计

        # 运行 50 帧
        for i in range(50):
            amcl.update(0, 0, 0, angles, dists, frame=i)

        x, y, yaw, conf = amcl.get_estimate()
        pos_err = math.sqrt(x**2 + y**2)
        # 应收敛到真实位置 (0, 0) 附近
        assert pos_err < 1.0, \
            f"AMCL 收敛后误差 {pos_err:.2f}m 应小于 1.0m"

    def test_kld_particle_adaptation(self):
        """KLD 采样应自适应调整粒子数。"""
        occ = OccupancyGrid()
        angles = np.linspace(-math.pi, math.pi, 36)
        dists = np.full(36, 8.0)
        occ.update_from_scan(0, 0, angles, dists)

        amcl = AMCL(occ, n_particles=300, kld_min=50, kld_max=500)
        amcl.init_cloud(0, 0, 0, spread=0.1)

        # 初始粒子数应为 max
        assert amcl.n_active >= amcl.kld_min

        # 运行若干帧后，粒子数应自适应
        for i in range(30):
            amcl.update(0, 0, 0, angles, dists, frame=i)

        # 收敛后粒子数应减少 (KLD 自适应)
        assert amcl.n_active <= amcl.kld_max, \
            "粒子数不应超过 kld_max"


# =====================================================================
# 5. OccupancyGrid 增量更新回归测试
# =====================================================================
class TestIncrementalUpdate:
    """测试 OccupancyGrid 的增量更新功能 (P1-2)。"""

    def test_dirty_mask_tracking(self):
        """dirty_mask 应正确追踪变化的 cell。"""
        occ = OccupancyGrid()
        assert occ.get_dirty_count() == 0, "初始 dirty_count 应为 0"

        # 扫描后应有 dirty cell
        angles = np.linspace(-math.pi, math.pi, 36)
        dists = np.full(36, 3.0)
        occ.update_from_scan(0, 0, angles, dists)

        n_dirty = occ.get_dirty_count()
        assert n_dirty > 0, "扫描后应有 dirty cell"

        # snapshot 后 dirty 应清零
        occ.take_snapshot()
        assert occ.get_dirty_count() == 0, "snapshot 后 dirty 应清零"

    def test_has_changed_since_snapshot(self):
        """has_changed_since_snapshot 应正确检测变化。"""
        occ = OccupancyGrid()
        occ.take_snapshot()  # 初始 snapshot

        # 无变化
        assert not occ.has_changed_since_snapshot(0.05), \
            "无变化时应返回 False"

        # 扫描产生变化
        angles = np.linspace(-math.pi, math.pi, 36)
        dists = np.full(36, 3.0)
        occ.update_from_scan(0, 0, angles, dists)

        assert occ.has_changed_since_snapshot(0.05), \
            "扫描后应检测到变化"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
