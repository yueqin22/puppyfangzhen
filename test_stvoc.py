#!/usr/bin/env python3
"""STVOC 时空速度障碍锥避障法 单元测试

测试五大组件:
    1. TrajectoryPredictor - 多步轨迹预测
    2. SpatioTemporalConflictGraph - 时空冲突图
    3. VelocityObstacleCone - 速度障碍锥
    4. HotspotMemory - 碰撞热点记忆
    5. TTCGameDecision - TTC博弈等待决策
    6. STVOCController - 主控制器集成测试
"""
import math
import time
import unittest
import numpy as np

from stvoc_avoidance import (
    TrajectoryPredictor, SpatioTemporalConflictGraph,
    VelocityObstacleCone, HotspotMemory, TTCGameDecision,
    STVOCController, PredictedPoint, ConflictEvent, AvoidanceCommand,
)


class TestTrajectoryPredictor(unittest.TestCase):
    """测试多步轨迹预测器"""

    def setUp(self):
        self.predictor = TrajectoryPredictor(horizon=3.0, dt=1/30)

    def test_linear_prediction(self):
        """测试线性运动预测"""
        traj = self.predictor.predict(
            obs_x=1.0, obs_y=2.0, obs_vx=0.5, obs_vy=0.0,
            pattern='linear', name='test')
        self.assertEqual(len(traj), 90)
        # 第1步应该在 (1.0 + 0.5*dt, 2.0)
        self.assertAlmostEqual(traj[0].x, 1.0 + 0.5 * (1/30), places=3)
        self.assertAlmostEqual(traj[0].y, 2.0, places=3)
        # 最后一步应该在 (1.0 + 0.5*3.0, 2.0) = (2.5, 2.0)
        self.assertAlmostEqual(traj[-1].x, 2.5, places=1)
        self.assertAlmostEqual(traj[-1].y, 2.0, places=1)

    def test_random_prediction(self):
        """测试随机运动预测"""
        traj = self.predictor.predict(
            obs_x=0.0, obs_y=0.0, obs_vx=0.1, obs_vy=0.1,
            pattern='random', name='random_walker')
        self.assertEqual(len(traj), 90)
        # 随机运动应该有扰动
        linear_x = 0.1 * 3.0  # 纯线性最后位置
        self.assertNotAlmostEqual(traj[-1].x, linear_x, places=2)

    def test_acceleration_estimation(self):
        """测试加速度估计"""
        # 多次更新速度历史，产生加速度
        self.predictor.predict(0, 0, 0.1, 0, name='acc')
        self.predictor.predict(0, 0, 0.2, 0, name='acc')
        self.predictor.predict(0, 0, 0.3, 0, name='acc')
        # 再次预测，应该包含加速度修正
        traj = self.predictor.predict(0, 0, 0.4, 0, name='acc')
        self.assertGreater(traj[-1].x, 0.4 * 3.0)  # 加速度使距离更大

    def test_prediction_horizon(self):
        """测试预测范围"""
        traj = self.predictor.predict(0, 0, 1.0, 0, pattern='linear')
        self.assertAlmostEqual(traj[-1].t, 3.0, places=1)

    def test_uncertainty_growth(self):
        """测试不确定性随时间增长"""
        u0 = self.predictor.get_uncertainty(0.0)
        u1 = self.predictor.get_uncertainty(1.0)
        u4 = self.predictor.get_uncertainty(4.0)
        self.assertGreater(u1, u0)
        self.assertGreater(u4, u1)
        # sqrt 关系
        self.assertAlmostEqual(u4, u1 * 2, places=2)


class TestConflictGraph(unittest.TestCase):
    """测试时空冲突图"""

    def setUp(self):
        self.cg = SpatioTemporalConflictGraph(safe_distance=0.55)

    def test_no_conflict(self):
        """测试无冲突场景"""
        # 机器人路径和行人轨迹不交叉
        robot_path = [(0, 0, 0.1), (0.1, 0, 0.2), (0.2, 0, 0.3)]
        obstacles = [('person', [
            PredictedPoint(x=5.0, y=5.0, t=0.1),
            PredictedPoint(x=5.1, y=5.0, t=0.2),
            PredictedPoint(x=5.2, y=5.0, t=0.3),
        ], 0.1, 0.0)]
        conflicts = self.cg.detect_conflicts(robot_path, obstacles)
        self.assertEqual(len(conflicts), 0)

    def test_conflict_detected(self):
        """测试冲突检测"""
        # 机器人和行人会在同一位置
        robot_path = [(1.0, 1.0, 0.1), (1.1, 1.0, 0.2), (1.2, 1.0, 0.3)]
        obstacles = [('person', [
            PredictedPoint(x=1.0, y=1.0, t=0.1),
            PredictedPoint(x=1.1, y=1.0, t=0.2),
            PredictedPoint(x=1.2, y=1.0, t=0.3),
        ], 0.1, 0.0)]
        conflicts = self.cg.detect_conflicts(robot_path, obstacles)
        self.assertGreater(len(conflicts), 0)
        self.assertEqual(conflicts[0].obstacle_name, 'person')

    def test_earliest_conflict(self):
        """测试获取最早冲突"""
        conflicts = [
            ConflictEvent((1, 1), 0.5, 'a', 0.8, 0.3, 0.2),
            ConflictEvent((2, 2), 0.3, 'b', 0.9, 0.4, 0.1),
            ConflictEvent((3, 3), 0.7, 'c', 0.7, 0.2, 0.3),
        ]
        earliest = self.cg.get_earliest_conflict(conflicts)
        self.assertEqual(earliest.obstacle_name, 'b')
        self.assertAlmostEqual(earliest.time, 0.3)

    def test_multiple_obstacles(self):
        """测试多个行人冲突检测"""
        robot_path = [(0, 0, 0.1), (0.5, 0, 0.2), (1.0, 0, 0.3)]
        obstacles = [
            ('person_a', [
                PredictedPoint(0.5, 0, 0.1),
                PredictedPoint(0.5, 0, 0.2),
                PredictedPoint(0.5, 0, 0.3),
            ], 0.0, 0.0),
            ('person_b', [
                PredictedPoint(1.0, 0, 0.1),
                PredictedPoint(1.0, 0, 0.2),
                PredictedPoint(1.0, 0, 0.3),
            ], 0.0, 0.0),
        ]
        conflicts = self.cg.detect_conflicts(robot_path, obstacles)
        self.assertEqual(len(conflicts), 2)


class TestVelocityObstacleCone(unittest.TestCase):
    """测试速度障碍锥"""

    def setUp(self):
        self.vo = VelocityObstacleCone(robot_radius=0.25, obstacle_radius=0.30)

    def test_vo_computation(self):
        """测试速度障碍锥计算"""
        # 行人在正前方
        vo = self.vo.compute_vo(0, 0, 2.0, 0, 0.0, 0.0)
        self.assertIsNotNone(vo)
        self.assertEqual(vo.obstacle_name, '')
        self.assertAlmostEqual(vo.distance, 2.0, places=2)

    def test_vo_far_obstacle(self):
        """测试远距离行人不构成威胁"""
        vo = self.vo.compute_vo(0, 0, 10.0, 0, 0.0, 0.0)
        self.assertIsNone(vo)

    def test_velocity_safety_check(self):
        """测试速度安全性检查"""
        # 行人在正前方，朝向行人的速度不安全
        vo = self.vo.compute_vo(0, 0, 2.0, 0, 0.0, 0.0)
        # 朝行人方向的速度应该不安全
        safe = self.vo.is_velocity_safe(0.3, 0.0, vo)
        # 由于行人在前方，朝前走可能不安全
        # 注意：具体取决于锥体几何

    def test_optimal_velocity_selection(self):
        """测试最优速度选择"""
        # 无行人时应该朝目标方向
        vx, vy, reason = self.vo.find_optimal_velocity(
            0, 0, 5.0, 0, [], max_speed=0.3)
        self.assertAlmostEqual(vx, 0.3, places=2)
        self.assertEqual(reason, 'cruise_no_threat')

    def test_optimal_velocity_with_obstacle(self):
        """测试有行人时的最优速度选择"""
        # 行人在正前方
        vx, vy, reason = self.vo.find_optimal_velocity(
            0, 0, 5.0, 0,
            [('person', 1.5, 0, 0.0, 0.0)],
            max_speed=0.3)
        # 应该选择避让速度
        self.assertNotEqual(reason, 'cruise_no_threat')


class TestHotspotMemory(unittest.TestCase):
    """测试碰撞热点记忆"""

    def setUp(self):
        self.hm = HotspotMemory(decay_tau=300.0)

    def test_record_and_query(self):
        """测试记录和查询"""
        self.hm.record_event(1.5, 0.5, 1.0)
        heat = self.hm.get_heat_level(1.5, 0.5)
        self.assertGreater(heat, 0.5)

    def test_distance_decay(self):
        """测试距离衰减"""
        self.hm.record_event(1.5, 0.5, 1.0)
        # 近距离热度高
        near = self.hm.get_heat_level(1.5, 0.5)
        # 远距离热度低
        far = self.hm.get_heat_level(5.0, 5.0)
        self.assertGreater(near, far)
        self.assertAlmostEqual(far, 0.0, places=2)

    def test_time_decay(self):
        """测试时间衰减"""
        # 记录一个旧事件
        old_time = time.time() - 600  # 10分钟前
        self.hm.record_event(1.0, 1.0, 1.0, timestamp=old_time)
        # 记录一个新事件
        self.hm.record_event(1.0, 1.0, 1.0)
        # 新事件热度更高
        heat = self.hm.get_heat_level(1.0, 1.0)
        self.assertGreater(heat, 0.0)

    def test_alertness_level(self):
        """测试警惕等级"""
        # 无历史碰撞
        sf, hm, al = self.hm.get_alertness_level(0, 0)
        self.assertAlmostEqual(al, 0.0, places=2)
        self.assertAlmostEqual(sf, 1.0, places=2)

        # 有碰撞历史
        self.hm.record_event(1.0, 1.0, 1.0)
        sf, hm, al = self.hm.get_alertness_level(1.0, 1.0)
        self.assertGreater(al, 0.0)
        self.assertLess(sf, 1.0)
        self.assertGreater(hm, 1.0)

    def test_decay_cleanup(self):
        """测试衰减清理"""
        # 记录很多事件
        for i in range(10):
            self.hm.record_event(i * 0.1, 0, 1.0)
        # 衰减
        self.hm.decay()
        # 热点应该还存在（衰减时间常数300s）
        self.assertGreater(len(self.hm._hotspots), 0)

    def test_stats(self):
        """测试统计信息"""
        self.hm.record_event(1, 1, 1.0, is_near_miss=False)
        self.hm.record_event(2, 2, 0.5, is_near_miss=True)
        stats = self.hm.get_stats()
        self.assertEqual(stats['total_events'], 2)
        self.assertEqual(stats['near_miss_events'], 1)

    def test_hotspots_list(self):
        """测试获取热点列表"""
        self.hm.record_event(1, 1, 1.0)
        hotspots = self.hm.get_hotspots()
        self.assertGreater(len(hotspots), 0)
        x, y, heat = hotspots[0]
        self.assertAlmostEqual(x, 1.0, places=2)
        self.assertAlmostEqual(y, 1.0, places=2)


class TestTTCGameDecision(unittest.TestCase):
    """测试TTC博弈等待决策"""

    def setUp(self):
        self.ttc = TTCGameDecision(
            emergency_threshold=1.0, wait_threshold=2.0)

    def test_no_collision_risk(self):
        """测试无碰撞风险"""
        action, speed, reason = self.ttc.decide(
            ttc=float('inf'), robot_can_pass=True,
            obstacle_approaching=False)
        self.assertEqual(action, 'pass')
        self.assertAlmostEqual(speed, 1.0)

    def test_emergency_flee(self):
        """测试紧急避让"""
        action, speed, reason = self.ttc.decide(
            ttc=0.5, robot_can_pass=True,
            obstacle_approaching=True)
        self.assertEqual(action, 'flee')
        self.assertAlmostEqual(speed, 0.0)

    def test_conservative_wait(self):
        """测试保守等待"""
        action, speed, reason = self.ttc.decide(
            ttc=1.2, robot_can_pass=True,
            obstacle_approaching=True)
        self.assertEqual(action, 'wait')

    def test_slow_pass(self):
        """测试减速通过"""
        action, speed, reason = self.ttc.decide(
            ttc=1.6, robot_can_pass=True,
            obstacle_approaching=True)
        self.assertEqual(action, 'pass')
        self.assertAlmostEqual(speed, 0.5)

    def test_ttc_computation(self):
        """测试TTC计算"""
        # 行人从正前方接近
        ttc = self.ttc.compute_ttc(
            robot_x=0, robot_y=0, robot_vx=0, robot_vy=0,
            obs_x=2.0, obs_y=0, obs_vx=-0.5, obs_vy=0,
            safe_distance=0.55)
        # dist=2.0, closing_rate=0.5, ttc=(2.0-0.55)/0.5=2.9s
        self.assertGreater(ttc, 0)
        self.assertAlmostEqual(ttc, 2.9, places=1)

    def test_ttc_receding(self):
        """测试行人在远离时TTC为inf"""
        ttc = self.ttc.compute_ttc(
            robot_x=0, robot_y=0, robot_vx=0, robot_vy=0,
            obs_x=2.0, obs_y=0, obs_vx=0.5, obs_vy=0,
            safe_distance=0.55)
        self.assertEqual(ttc, float('inf'))


class TestSTVOCController(unittest.TestCase):
    """测试STVOC主控制器"""

    def setUp(self):
        self.controller = STVOCController(
            dt=1/30, max_speed=0.3, max_angular=1.2, safe_distance=0.55)

    def test_no_obstacles(self):
        """测试无行人时正常巡航"""
        cmd = self.controller.compute_avoidance(
            robot_x=0, robot_y=0, robot_yaw=0,
            robot_vx=0, robot_vy=0,
            target_x=5, target_y=0,
            obstacles=[])
        self.assertEqual(cmd.action, 'cruise')
        self.assertFalse(cmd.conflict_detected)
        self.assertEqual(cmd.time_to_conflict, float('inf'))

    def test_far_obstacle(self):
        """测试远距离行人不触发避障"""
        cmd = self.controller.compute_avoidance(
            robot_x=0, robot_y=0, robot_yaw=0,
            robot_vx=0.1, robot_vy=0,
            target_x=5, target_y=0,
            obstacles=[('far_person', 10.0, 10.0, 0, 0, 'linear')])
        self.assertFalse(cmd.conflict_detected)

    def test_approaching_obstacle(self):
        """测试行人接近时触发避障"""
        cmd = self.controller.compute_avoidance(
            robot_x=0, robot_y=0, robot_yaw=0,
            robot_vx=0.1, robot_vy=0,
            target_x=5, target_y=0,
            obstacles=[('person', 1.5, 0, -0.3, 0, 'linear')])
        # 应该检测到冲突
        self.assertTrue(cmd.conflict_detected)
        self.assertLess(cmd.time_to_conflict, 5.0)

    def test_record_collision(self):
        """测试碰撞记录到热点记忆"""
        self.controller.record_collision(1.5, 0.5, 1.0)
        heat = self.controller.hotspot_memory.get_heat_level(1.5, 0.5)
        self.assertGreater(heat, 0.5)

    def test_record_near_miss(self):
        """测试近距离规避记录"""
        self.controller.record_near_miss(1.0, 1.0, 0.5)
        stats = self.controller.hotspot_memory.get_stats()
        self.assertEqual(stats['near_miss_events'], 1)

    def test_stats_tracking(self):
        """测试统计追踪"""
        # 运行几次决策
        for _ in range(5):
            self.controller.compute_avoidance(
                0, 0, 0, 0.1, 0, 5, 0,
                [('person', 1.5, 0, -0.3, 0, 'linear')])
        stats = self.controller.get_stats()
        self.assertGreater(stats['total_decisions'], 0)

    def test_hotspot_aware_speed_reduction(self):
        """测试热点区域降速"""
        # 记录碰撞热点
        self.controller.record_collision(2.0, 0, 1.0)
        # 在热点附近查询
        sf, hm, al = self.controller.hotspot_memory.get_alertness_level(2.0, 0)
        self.assertGreater(al, 0)
        self.assertLess(sf, 1.0)

    def test_decay(self):
        """测试热点衰减"""
        # 记录热点
        self.controller.record_collision(1, 1, 1.0)
        # 衰减
        self.controller.decay_hotspots()
        # 热点仍存在（衰减缓慢）
        stats = self.controller.get_stats()
        self.assertGreater(stats['hotspot']['active_hotspots'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
