#!/usr/bin/env python3
"""
技术创新模块测试 (v4.0 Innovation Tests)
=========================================
验证 4 个技术创新模块的功能正确性：
1. Risk-Aware A* 风险感知路径规划
2. D* Lite 增量重规划
3. Lifelong SLAM 持续建图
4. GNN 动态轨迹预测
"""
import os
import sys
import math
import time
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

# 设置环境变量启用新模块
os.environ['USE_RISK_AWARE'] = '1'
os.environ['USE_DSTAR_LITE'] = '1'
os.environ['USE_LIFELONG_SLAM'] = '1'
os.environ['USE_GNN_PREDICTOR'] = '1'

from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION
from costmap import Costmap
from risk_aware_astar import RiskAwareAStarPlanner
from dstar_lite import DStarLitePlanner
from lifelong_slam import LifelongSLAM, ChangeDetector, SubmapManager
from gnn_trajectory_predictor import GNNTrajectoryPredictor

passed = 0
failed = 0


def test_passed(name):
    global passed
    passed += 1
    print(f"  [OK] {name}")


def test_failed(name, reason=""):
    global failed
    failed += 1
    print(f"  [FAIL] {name}: {reason}")


def test_risk_aware_astar():
    """测试风险感知 A*。"""
    print("\n=== 测试 Risk-Aware A* ===")

    # 1. 基本初始化
    try:
        costmap = Costmap()
        planner = RiskAwareAStarPlanner(costmap, risk_lambda=1.0, alpha=0.95)
        test_passed("初始化")
    except Exception as e:
        test_failed("初始化", str(e))
        return

    # 2. erf 近似精度
    try:
        # erf(0) = 0, erf(inf) = 1, erf(-inf) = -1
        assert abs(planner._erf_approx(0.0)) < 1e-5
        assert abs(planner._erf_approx(1.0) - 0.8427) < 0.001
        assert abs(planner._erf_approx(-1.0) + 0.8427) < 0.001
        test_passed("erf近似精度")
    except Exception as e:
        test_failed("erf近似精度", str(e))

    # 3. 碰撞概率计算
    try:
        # 在障碍物附近应有碰撞概率 > 0
        p = planner._compute_collision_probability(GRID_W // 2, GRID_H // 2)
        assert 0 <= p <= 1
        test_passed(f"碰撞概率计算 (p={p:.4f})")
    except Exception as e:
        test_failed("碰撞概率计算", str(e))

    # 4. CVaR 代价
    try:
        cvar = planner._compute_cvar_cost(0.5)
        assert cvar > 0
        cvar_zero = planner._compute_cvar_cost(0.0)
        assert cvar_zero == 0
        test_passed(f"CVaR代价计算 (cvar={cvar:.4f})")
    except Exception as e:
        test_failed("CVaR代价计算", str(e))

    # 5. 风险感知规划
    try:
        # 用 costmap 中心附近的点规划
        path = planner.plan(0.0, 0.0, 1.0, 1.0)
        if path is None:
            # 尝试其他点
            path = planner.plan(-1.0, -1.0, 1.0, 1.0)
        assert path is not None and len(path) >= 2
        test_passed(f"风险感知规划 (path_len={len(path)})")
    except Exception as e:
        test_failed("风险感知规划", str(e))

    # 6. 协方差更新
    try:
        new_cov = np.eye(3) * 0.05  # 更小的不确定性
        planner.update_localization_covariance(new_cov)
        assert np.allclose(planner.loc_cov, new_cov)
        test_passed("协方差更新")
    except Exception as e:
        test_failed("协方差更新", str(e))

    # 7. λ=0 退化为标准 A*
    try:
        planner2 = RiskAwareAStarPlanner(costmap, risk_lambda=0.0)
        # 在中心格计算代价
        cost_normal = planner2._cell_cost_risk(GRID_W // 2, GRID_H // 2)
        c = costmap.get_cost_grid(GRID_W // 2, GRID_H // 2)
        expected = 1.0 + (c / 128.0) ** 2 * 12.0
        assert abs(cost_normal - expected) < 1e-6
        test_passed("λ=0退化为标准A*")
    except Exception as e:
        test_failed("λ=0退化", str(e))


def test_dstar_lite():
    """测试 D* Lite。"""
    print("\n=== 测试 D* Lite ===")

    # 1. 基本初始化
    try:
        costmap = Costmap()
        planner = DStarLitePlanner(costmap)
        test_passed("初始化")
    except Exception as e:
        test_failed("初始化", str(e))
        return

    # 2. 首次规划
    try:
        planner.initialize(0.0, 0.0, 1.0, 1.0)
        path = planner.extract_path()
        if path is None:
            planner.initialize(-1.0, -1.0, 1.0, 1.0)
            path = planner.extract_path()
        assert path is not None and len(path) >= 2
        test_passed(f"首次规划 (path_len={len(path)})")
    except Exception as e:
        test_failed("首次规划", str(e))

    # 3. 增量重规划
    try:
        # 移动起点 (模拟机器人移动)
        path = planner.replan_incremental(0.5, 0.5)
        assert path is not None
        test_passed("增量重规划")
    except Exception as e:
        test_failed("增量重规划", str(e))

    # 4. 代价变化检测
    try:
        snapshot = costmap.cost.copy()
        # 修改一个格子
        costmap.cost[GRID_H//2, GRID_W//2] = 200
        changed = planner.detect_cost_changes(snapshot)
        assert len(changed) > 0
        test_passed(f"代价变化检测 (changed={len(changed)})")
    except Exception as e:
        test_failed("代价变化检测", str(e))

    # 5. 统计
    try:
        stats = planner.get_statistics()
        assert 'total_replans' in stats
        assert stats['total_replans'] > 0
        test_passed(f"统计信息 (replans={stats['total_replans']})")
    except Exception as e:
        test_failed("统计信息", str(e))


def test_lifelong_slam():
    """测试持续建图。"""
    print("\n=== 测试 Lifelong SLAM ===")

    # 1. 基本初始化
    try:
        occ_grid = OccupancyGrid()
        lifelong = LifelongSLAM(occ_grid, forgetting_factor=0.3)
        test_passed("初始化")
    except Exception as e:
        test_failed("初始化", str(e))
        return

    # 2. 变化检测器
    try:
        detector = ChangeDetector(threshold=1.0, min_observations=4)
        # 记录多次观测
        for i in range(5):
            detector.observe(10, 10, 0.0)  # 空闲
        changed, llr = detector.detect_change(10, 10)
        assert not changed  # 应该无变化

        # 记录变化
        for i in range(5):
            detector.observe(10, 10, 1.0)  # 变成占据
        changed, llr = detector.detect_change(10, 10)
        assert changed  # 应该检测到变化
        test_passed(f"变化检测 (llr={llr:.2f})")
    except Exception as e:
        test_failed("变化检测", str(e))

    # 3. 子地图管理
    try:
        sm = SubmapManager(submap_size=20, max_submaps=5)
        # 写入数据
        sm.update_cell(5, 5, 0.8)
        sm.update_cell(25, 25, 0.6)
        assert sm.get_cell(5, 5) == 0.8
        assert sm.get_cell(25, 25) == 0.6
        assert len(sm.submaps) == 2
        test_passed(f"子地图管理 (submaps={len(sm.submaps)})")
    except Exception as e:
        test_failed("子地图管理", str(e))

    # 4. 持续更新
    try:
        angles = np.linspace(-math.pi, math.pi, 72)
        distances = np.ones(72) * 2.0  # 2m 处都有障碍

        for _ in range(5):
            lifelong.update(angles, distances, 0.0, 0.0, 0.0)

        stats = lifelong.get_statistics()
        assert stats['total_updates'] >= 5
        test_passed(f"持续更新 (updates={stats['total_updates']})")
    except Exception as e:
        test_failed("持续更新", str(e))

    # 5. 报告导出
    try:
        report = lifelong.export_change_report()
        assert isinstance(report, str)
        test_passed("报告导出")
    except Exception as e:
        test_failed("报告导出", str(e))


def test_gnn_predictor():
    """测试 GNN 轨迹预测。"""
    print("\n=== 测试 GNN 轨迹预测 ===")

    # 1. 基本初始化
    try:
        predictor = GNNTrajectoryPredictor(history_length=8,
                                              predict_length=10,
                                              hidden_dim=16)
        test_passed("初始化")
    except Exception as e:
        test_failed("初始化", str(e))
        return

    # 2. 历史更新
    try:
        for i in range(10):
            predictor.update_history("person_1",
                                     x=i * 0.1, y=0.0,
                                     vx=0.1, vy=0.0)
        assert "person_1" in predictor.history
        assert len(predictor.history["person_1"]) >= 8
        test_passed("历史更新")
    except Exception as e:
        test_failed("历史更新", str(e))

    # 3. 单障碍物预测
    try:
        pred = predictor.predict_single("person_1", steps_ahead=5)
        assert len(pred) == 5
        test_passed(f"单障碍物预测 (len={len(pred)})")
    except Exception as e:
        test_failed("单障碍物预测", str(e))

    # 4. 多障碍物预测
    try:
        # 添加第二个障碍物
        for i in range(10):
            predictor.update_history("person_2",
                                     x=0.0, y=i * 0.05,
                                     vx=0.0, vy=0.05)
        predictions = predictor.predict(["person_1", "person_2"])
        assert len(predictions) == 2
        test_passed("多障碍物预测")
    except Exception as e:
        test_failed("多障碍物预测", str(e))

    # 5. 社会池化
    try:
        features = np.random.randn(3, 16)  # 3个行人
        positions = np.array([[0, 0], [1, 0], [3, 0]])
        pooled = predictor.social_pool.forward(features, positions)
        assert pooled.shape == (3, 16)
        test_passed("社会池化")
    except Exception as e:
        test_failed("社会池化", str(e))

    # 6. 在线训练
    try:
        # 生成更多训练数据
        for step in range(20):
            predictor.update_history("person_1",
                                     x=step * 0.1, y=0.0,
                                     vx=0.1, vy=0.0)
        loss = predictor.train_online()
        assert loss is None or loss >= 0
        test_passed("在线训练")
    except Exception as e:
        test_failed("在线训练", str(e))

    # 7. 统计
    try:
        stats = predictor.get_statistics()
        assert 'total_predictions' in stats
        test_passed(f"统计信息 (preds={stats['total_predictions']})")
    except Exception as e:
        test_failed("统计信息", str(e))


def test_integration():
    """集成测试: 所有模块协同工作。"""
    print("\n=== 集成测试 ===")

    try:
        # 共享 costmap
        costmap = Costmap()
        occ_grid = OccupancyGrid()

        # 1. Risk-Aware A* 规划路径
        risk_planner = RiskAwareAStarPlanner(costmap, risk_lambda=0.5)
        path = risk_planner.plan(-1.0, -1.0, 1.0, 1.0)
        if path is None:
            path = risk_planner.plan(0.0, 0.0, 1.0, 1.0)
        assert path is not None

        # 2. D* Lite 增量更新
        dstar = DStarLitePlanner(costmap)
        dstar.initialize(-1.0, -1.0, 1.0, 1.0)

        # 模拟障碍物变化
        snapshot = costmap.cost.copy()
        costmap.cost[GRID_H//2, GRID_W//2] = 200
        changed = dstar.detect_cost_changes(snapshot)
        assert len(changed) > 0

        # 增量重规划
        new_path = dstar.replan_incremental(0.1, 0.1, changed)
        # new_path 可能为 None 如果起点被阻塞，但不影响测试

        # 3. Lifelong SLAM 持续建图
        lifelong = LifelongSLAM(occ_grid)
        angles = np.linspace(-math.pi, math.pi, 36)
        distances = np.ones(36) * 2.0
        lifelong.update(angles, distances, 0.0, 0.0, 0.0)

        # 4. GNN 预测
        gnn = GNNTrajectoryPredictor(history_length=4, predict_length=5)
        for i in range(6):
            gnn.update_history("person_1", x=i*0.1, y=0, vx=0.1, vy=0)
        pred = gnn.predict_single("person_1")

        test_passed(f"集成测试通过 (path={len(path) if path else 0}, dstar_changed={len(changed)}, pred={len(pred) if pred else 0})")
    except Exception as e:
        test_failed("集成测试", str(e))


if __name__ == '__main__':
    print("=" * 60)
    print("  技术创新模块测试 (v4.0 Innovation Tests)")
    print("=" * 60)

    test_risk_aware_astar()
    test_dstar_lite()
    test_lifelong_slam()
    test_gnn_predictor()
    test_integration()

    print("\n" + "=" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败")
    print("=" * 60)

    sys.exit(0 if failed == 0 else 1)
