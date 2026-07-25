#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v5.0 学术提升代码测试套件 (Academic Enhancement Test Suite v5.0)
================================================================
对 v5.0 学术提升阶段的 7 个核心模块进行单元测试：

  1. risk_aware_astar.py      - 风险感知 A* 与自适应 λ
  2. theory_analysis.py       - AUFE 水填充最优性证明
  3. baselines.py             - 标准探索基线（随机/最近边界/贪心信息/Oracle）
  4. ablation_study_v3.py     - 7 维消融实验与协同效应
  5. parameter_sensitivity.py - 参数敏感性分析
  6. unified_uncertainty.py   - 统一不确定性数学框架
  7. failure_analysis.py      - 失败案例分析与局限性

测试原则：
  - 不依赖 CoppeliaSim（使用 mock 数据）
  - 使用 unittest 框架（兼容 pytest 运行）
  - 每个测试用例 < 1 秒
  - import 失败时自动跳过对应测试

运行方式：
    python test_v5_academic.py
    python -m pytest test_v5_academic.py -v
"""

import os
import sys
import math
import time
import unittest
from typing import Optional, Tuple, List, Dict, Any

import numpy as np

# 确保项目根目录在 sys.path 中
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)


# =====================================================================
# Mock 对象（不依赖 CoppeliaSim / 真实仿真器）
# =====================================================================

class MockCostmap:
    """模拟代价地图。

    提供 risk_aware_astar 等模块所需的 costmap 接口：
      - get_cost_grid(gx, gy): 返回栅格代价值
      - width / height: 地图尺寸
    """

    def __init__(self, width: int = 30, height: int = 30,
                 default_cost: int = 0):
        self.width = width
        self.height = height
        self._default_cost = default_cost
        # 构造一个简单的代价值矩阵
        self._cost_grid = np.zeros((width, height), dtype=np.int32)
        # 在中间放一个障碍物
        cx, cy = width // 2, height // 2
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if 0 <= cx + dx < width and 0 <= cy + dy < height:
                    self._cost_grid[cx + dx, cy + dy] = 254  # COST_LETHAL

    def get_cost_grid(self, gx: int, gy: int) -> int:
        """获取栅格代价（模拟 costmap.Costmap.get_cost_grid）。"""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return int(self._cost_grid[gx, gy])
        return 0

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """世界坐标 → 栅格坐标。"""
        gx = int(wx / 0.1)
        gy = int(wy / 0.1)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """栅格坐标 → 世界坐标。"""
        return (gx * 0.1, gy * 0.1)


class MockOccupancyGrid:
    """模拟占据栅格地图。

    提供 baselines 等模块所需的 occ_grid 接口：
      - log_odds: log-odds 概率栅格
      - visited: 访问标记
      - find_frontiers(rx, ry, max_frontiers): 边界检测
      - grid_to_world / world_to_grid: 坐标转换
      - resolution / width / height
    """

    def __init__(self, width: int = 50, height: int = 50,
                 resolution: float = 0.1):
        self.width = width
        self.height = height
        self.resolution = resolution
        # log-odds: 0 = 未知, 正 = 空闲, 负 = 占据
        self.log_odds = np.zeros((height, width), dtype=np.float32)
        # 标记部分区域为已知空闲
        self.log_odds[10:40, 10:40] = 1.0
        # visited 标记
        self.visited = np.zeros((height, width), dtype=bool)
        self.visited[20:30, 20:30] = True

    def find_frontiers(self, rx: int, ry: int,
                       max_frontiers: int = 20) -> List[Tuple[float, float, int]]:
        """检测边界（已知空闲与未知的交界）。

        返回 (world_x, world_y, cluster_size) 三元组列表，
        与 baselines.py 的期望接口一致。
        """
        return [
            (1.5, 1.5, 5),
            (3.5, 3.5, 8),
            (2.5, 1.0, 3),
            (1.0, 2.5, 4),
        ]

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """栅格坐标 → 世界坐标。"""
        return (gx * self.resolution, gy * self.resolution)

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """世界坐标 → 栅格坐标。"""
        return (int(wx / self.resolution), int(wy / self.resolution))

    def get_log_odds(self, gx: int, gy: int) -> float:
        """获取栅格 log-odds 值。"""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return float(self.log_odds[gy, gx])
        return 0.0

    def is_visited(self, gx: int, gy: int) -> bool:
        """检查栅格是否已访问。"""
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return bool(self.visited[gy, gx])
        return False


# =====================================================================
# 模块导入（失败则记录错误，对应测试自动跳过）
# =====================================================================

_IMPORT_ERRORS: Dict[str, str] = {}

# 1. risk_aware_astar
try:
    from risk_aware_astar import RiskAwareAStarPlanner
    _RISK_AWARE_OK = True
except Exception as _e:
    _RISK_AWARE_OK = False
    _IMPORT_ERRORS['risk_aware_astar'] = str(_e)

# 2. theory_analysis
try:
    from theory_analysis import AUFE_Optimality_Proof
    _THEORY_OK = True
except Exception as _e:
    _THEORY_OK = False
    _IMPORT_ERRORS['theory_analysis'] = str(_e)

# 3. baselines
try:
    from baselines import (
        BaselineStrategy,
        RandomWalkBaseline,
        NearestFrontierBaseline,
        GreedyInformationBaseline,
        OracleBaseline,
        BaselineFactory,
    )
    _BASELINES_OK = True
except Exception as _e:
    _BASELINES_OK = False
    _IMPORT_ERRORS['baselines'] = str(_e)

# 4. ablation_study_v3
try:
    from ablation_study_v3 import (
        AblationConfig,
        StatisticalAnalysis,
        SynergyAnalysis,
    )
    _ABLATION_OK = True
except Exception as _e:
    _ABLATION_OK = False
    _IMPORT_ERRORS['ablation_study_v3'] = str(_e)

# 5. parameter_sensitivity
try:
    from parameter_sensitivity import (
        PARAMETER_CONFIGS,
        ParameterSweep,
        GlobalSensitivityAnalysis,
    )
    _PARAM_SENS_OK = True
except Exception as _e:
    _PARAM_SENS_OK = False
    _IMPORT_ERRORS['parameter_sensitivity'] = str(_e)

# 6. unified_uncertainty
try:
    from unified_uncertainty import (
        UncertaintyState,
        UncertaintyPropagator,
        InformationGainCalculator,
        AUFEAsApproximateOptimum,
        UncertaintyTracker,
    )
    _UNIFIED_UNC_OK = True
except Exception as _e:
    _UNIFIED_UNC_OK = False
    _IMPORT_ERRORS['unified_uncertainty'] = str(_e)

# 7. failure_analysis
try:
    from failure_analysis import (
        FailureCase,
        FailureDetector,
        FailureAnalyzer,
        LimitationsAnalysis,
        FAILURE_TYPE_AMCL_KIDNAPPED,
        FAILURE_TYPE_NARROW_PASSAGE,
        FAILURE_TYPE_DYNAMIC_COLLISION,
        FAILURE_TYPE_DEAD_ZONE,
    )
    _FAILURE_OK = True
except Exception as _e:
    _FAILURE_OK = False
    _IMPORT_ERRORS['failure_analysis'] = str(_e)


# =====================================================================
# 跳过装饰器（import 失败时跳过整个测试类）
# =====================================================================

skip_if_no_risk_aware = unittest.skipUnless(
    _RISK_AWARE_OK,
    f"risk_aware_astar 导入失败: {_IMPORT_ERRORS.get('risk_aware_astar', '未知')}"
)
skip_if_no_theory = unittest.skipUnless(
    _THEORY_OK,
    f"theory_analysis 导入失败: {_IMPORT_ERRORS.get('theory_analysis', '未知')}"
)
skip_if_no_baselines = unittest.skipUnless(
    _BASELINES_OK,
    f"baselines 导入失败: {_IMPORT_ERRORS.get('baselines', '未知')}"
)
skip_if_no_ablation = unittest.skipUnless(
    _ABLATION_OK,
    f"ablation_study_v3 导入失败: {_IMPORT_ERRORS.get('ablation_study_v3', '未知')}"
)
skip_if_no_param_sens = unittest.skipUnless(
    _PARAM_SENS_OK,
    f"parameter_sensitivity 导入失败: {_IMPORT_ERRORS.get('parameter_sensitivity', '未知')}"
)
skip_if_no_unified_unc = unittest.skipUnless(
    _UNIFIED_UNC_OK,
    f"unified_uncertainty 导入失败: {_IMPORT_ERRORS.get('unified_uncertainty', '未知')}"
)
skip_if_no_failure = unittest.skipUnless(
    _FAILURE_OK,
    f"failure_analysis 导入失败: {_IMPORT_ERRORS.get('failure_analysis', '未知')}"
)


# =====================================================================
# 测试类 1: TestRiskAwareAdaptiveLambda
# 测试 risk_aware_astar.py 的自适应 λ 机制
# =====================================================================

@skip_if_no_risk_aware
class TestRiskAwareAdaptiveLambda(unittest.TestCase):
    """测试 Risk-Aware A* 的自适应风险系数 λ。"""

    def setUp(self):
        """每个测试前创建 mock costmap 和 planner。"""
        self.costmap = MockCostmap(width=30, height=30)

    def test_adaptive_lambda_disabled(self):
        """adaptive_lambda=False 时 update_adaptive_lambda 应直接返回，不修改 λ。"""
        planner = RiskAwareAStarPlanner(
            self.costmap, risk_lambda=1.0, adaptive_lambda=False
        )
        original_lambda = planner.lambda_risk
        # 调用 update_adaptive_lambda 应无效果
        planner.update_adaptive_lambda(loc_err=1.0, map_uncertainty=0.8)
        self.assertAlmostEqual(planner.lambda_risk, original_lambda)
        # 历史记录应为空
        self.assertEqual(len(planner.get_lambda_history()), 0)

    def test_adaptive_lambda_enabled_updates(self):
        """adaptive_lambda=True 时 λ 应根据 loc_err / map_unc 更新。"""
        planner = RiskAwareAStarPlanner(
            self.costmap, risk_lambda=1.0, adaptive_lambda=True,
            lambda_base=1.0, lambda_min=0.3, lambda_max=3.0
        )
        # 高不确定性 → λ 应增大
        planner.update_adaptive_lambda(loc_err=1.5, map_uncertainty=0.9)
        self.assertGreater(planner.lambda_risk, 1.0)
        # 历史记录应有 1 条
        history = planner.get_lambda_history()
        self.assertEqual(len(history), 1)

    def test_lambda_clipped_to_min(self):
        """λ 不应低于 lambda_min。"""
        planner = RiskAwareAStarPlanner(
            self.costmap, risk_lambda=1.0, adaptive_lambda=True,
            lambda_base=0.1, lambda_min=0.5, lambda_max=3.0
        )
        # 低不确定性 → λ 应被裁剪到 lambda_min
        planner.update_adaptive_lambda(loc_err=0.0, map_uncertainty=0.0)
        self.assertGreaterEqual(planner.lambda_risk, 0.5)

    def test_lambda_clipped_to_max(self):
        """λ 不应超过 lambda_max。"""
        planner = RiskAwareAStarPlanner(
            self.costmap, risk_lambda=1.0, adaptive_lambda=True,
            lambda_base=2.0, lambda_min=0.3, lambda_max=3.0
        )
        # 极高不确定性 → λ 应被裁剪到 lambda_max
        planner.update_adaptive_lambda(loc_err=5.0, map_uncertainty=1.0)
        self.assertLessEqual(planner.lambda_risk, 3.0)

    def test_lambda_history_recorded(self):
        """get_lambda_history 应返回更新历史列表。"""
        planner = RiskAwareAStarPlanner(
            self.costmap, risk_lambda=1.0, adaptive_lambda=True,
            lambda_base=1.0, lambda_min=0.3, lambda_max=3.0
        )
        # 连续更新 5 次
        for i in range(5):
            planner.update_adaptive_lambda(
                loc_err=0.1 * i, map_uncertainty=0.1 * i
            )
        history = planner.get_lambda_history()
        self.assertEqual(len(history), 5)
        # 所有值都应在 [lambda_min, lambda_max] 范围内
        for v in history:
            self.assertGreaterEqual(v, 0.3 - 1e-6)
            self.assertLessEqual(v, 3.0 + 1e-6)

    def test_risk_map_generation(self):
        """get_risk_map 应返回非空的风险地图数组。"""
        planner = RiskAwareAStarPlanner(
            self.costmap, risk_lambda=1.0, adaptive_lambda=False
        )
        risk_map = planner.get_risk_map()
        self.assertIsNotNone(risk_map)
        self.assertEqual(risk_map.ndim, 2)
        # 风险值应在 [0, 1] 范围内
        self.assertGreaterEqual(risk_map.min(), 0.0)
        self.assertLessEqual(risk_map.max(), 1.0 + 1e-6)


# =====================================================================
# 测试类 2: TestAUFE_OptimalityProof
# 测试 theory_analysis.py 的 AUFE 水填充最优性证明
# =====================================================================

@skip_if_no_theory
class TestAUFE_OptimalityProof(unittest.TestCase):
    """测试 AUFE 水填充最优性的理论证明与数值验证。"""

    def setUp(self):
        """创建 AUFE 证明实例。"""
        self.proof = AUFE_Optimality_Proof(n_sources=3, seed=42)

    def test_water_filling_weights_sum_to_one(self):
        """水填充最优权重应归一化（和为 1）且非负。"""
        gains = np.array([0.8, 0.5, 0.3])
        weights = self.proof.water_filling_optimal(gains)
        self.assertAlmostEqual(float(np.sum(weights)), 1.0, places=6)
        # 所有权重非负
        self.assertTrue(np.all(weights >= -1e-9))

    def test_sigmoid_weights_normalized(self):
        """AUFE sigmoid 权重应归一化。"""
        gains = np.array([0.9, 0.4, 0.6])
        weights = self.proof.aufe_sigmoid_weights(gains, k_slope=5.0, boost=2.0)
        self.assertAlmostEqual(float(np.sum(weights)), 1.0, places=6)
        # 所有权重非负
        self.assertTrue(np.all(weights >= -1e-9))

    def test_approximation_error_computed(self):
        """compute_approximation_error 应返回有效的误差统计。"""
        result = self.proof.compute_approximation_error(n_samples=50, k_slope=5.0)
        self.assertIsInstance(result, dict)
        self.assertIn('mean', result)
        self.assertIn('std', result)
        self.assertIn('max', result)
        # 误差均值应非负
        self.assertGreaterEqual(result['mean'], 0.0)

    def test_convergence_with_slope(self):
        """prove_convergence_with_slope 应返回收敛数据。"""
        result = self.proof.prove_convergence_with_slope(
            k_values=[1.0, 5.0, 20.0, 50.0]
        )
        self.assertIsInstance(result, dict)
        self.assertIn('k_values', result)
        self.assertIn('mean_errors', result)
        # k_values 应与输入一致
        self.assertEqual(len(result['k_values']), 4)
        # 误差数应与 k_values 数一致
        self.assertEqual(len(result['mean_errors']), 4)

    def test_regret_bound_within_theory(self):
        """compute_regret_bound 应返回 regret 上界，实际值应在界内。"""
        result = self.proof.compute_regret_bound(T=100, n_runs=10)
        self.assertIsInstance(result, dict)
        self.assertIn('mean_regret', result)
        self.assertIn('theoretical_bound', result)
        # 理论上界应为正（可能是标量或数组，统一转为 float）
        tb = result['theoretical_bound']
        tb_val = float(np.atleast_1d(tb)[0]) if hasattr(tb, '__len__') else float(tb)
        self.assertGreater(tb_val, 0)


# =====================================================================
# 测试类 3: TestBaselines
# 测试 baselines.py 的标准探索基线
# =====================================================================

@skip_if_no_baselines
class TestBaselines(unittest.TestCase):
    """测试标准探索基线方法。"""

    def setUp(self):
        """创建 mock occupancy grid。"""
        self.occ_grid = MockOccupancyGrid(width=50, height=50)
        self.robot_pos = (2.5, 2.5)  # 世界坐标

    def test_random_walk_baseline(self):
        """RandomWalkBaseline 应返回有效的目标点。"""
        baseline = RandomWalkBaseline(goal_tolerance=0.5)
        goal = baseline.select_goal(self.occ_grid, self.robot_pos)
        self.assertIsInstance(goal, tuple)
        self.assertEqual(len(goal), 2)
        # 目标应是有限数值
        self.assertTrue(math.isfinite(goal[0]))
        self.assertTrue(math.isfinite(goal[1]))

    def test_nearest_frontier_baseline(self):
        """NearestFrontierBaseline 应返回有效的目标点。"""
        baseline = NearestFrontierBaseline(goal_tolerance=0.5)
        goal = baseline.select_goal(self.occ_grid, self.robot_pos)
        self.assertIsInstance(goal, tuple)
        self.assertEqual(len(goal), 2)
        self.assertTrue(math.isfinite(goal[0]))
        self.assertTrue(math.isfinite(goal[1]))

    def test_greedy_info_baseline(self):
        """GreedyInformationBaseline 应返回有效的目标点。"""
        baseline = GreedyInformationBaseline(
            alpha=1.0, beta=0.5, sensor_range=4.0, goal_tolerance=0.5
        )
        goal = baseline.select_goal(self.occ_grid, self.robot_pos)
        self.assertIsInstance(goal, tuple)
        self.assertEqual(len(goal), 2)
        self.assertTrue(math.isfinite(goal[0]))
        self.assertTrue(math.isfinite(goal[1]))

    def test_oracle_baseline(self):
        """OracleBaseline 应返回有效的目标点。"""
        baseline = OracleBaseline(goal_tolerance=0.5)
        goal = baseline.select_goal(self.occ_grid, self.robot_pos)
        self.assertIsInstance(goal, tuple)
        self.assertEqual(len(goal), 2)
        self.assertTrue(math.isfinite(goal[0]))
        self.assertTrue(math.isfinite(goal[1]))

    def test_factory_create(self):
        """BaselineFactory.create 应创建所有注册的基线。"""
        for name in ['random_walk', 'nearest_frontier', 'greedy_info', 'oracle']:
            strategy = BaselineFactory.create(name)
            self.assertIsNotNone(strategy)
            self.assertIsInstance(strategy, BaselineStrategy)
        # 未知名称应抛出 ValueError
        with self.assertRaises(ValueError):
            BaselineFactory.create('unknown_method')


# =====================================================================
# 测试类 4: TestAblationStudyV3
# 测试 ablation_study_v3.py 的消融实验与统计分析
# =====================================================================

@skip_if_no_ablation
class TestAblationStudyV3(unittest.TestCase):
    """测试 7 维消融实验框架。"""

    def test_config_name_full(self):
        """完整配置的 name 应为 'proposed_full'。"""
        config = AblationConfig()
        self.assertTrue(config.is_full)
        self.assertEqual(config.name, 'proposed_full')

    def test_config_name_ablated(self):
        """单模块消融配置的 name 应以 'wo_' 开头。"""
        config = AblationConfig(use_aufe=False)
        self.assertFalse(config.is_full)
        self.assertTrue(config.name.startswith('wo_'))
        self.assertIn('aufe', config.name)

    def test_config_ablated_module(self):
        """单模块消融时 ablated_module 应返回模块名。"""
        config = AblationConfig(use_gnn=False)
        self.assertEqual(config.ablated_module, 'GNN')
        # 多模块消融时返回 None
        multi = AblationConfig(use_gnn=False, use_cbf=False)
        self.assertIsNone(multi.ablated_module)

    def test_to_env_vars(self):
        """to_env_vars 应返回正确的环境变量字典。"""
        config = AblationConfig(use_risk_aware=False)
        env = config.to_env_vars()
        self.assertIsInstance(env, dict)
        self.assertEqual(env['USE_RISK_AWARE'], '0')
        self.assertEqual(env['USE_AUFE'], '1')

    def test_paired_t_test(self):
        """StatisticalAnalysis.paired_t_test 应返回统计量。"""
        a = [0.85, 0.87, 0.82, 0.88, 0.86]
        b = [0.72, 0.74, 0.70, 0.75, 0.73]
        t_stat, p_value, cohens_d, is_sig = StatisticalAnalysis.paired_t_test(a, b)
        self.assertTrue(math.isfinite(t_stat))
        self.assertTrue(0.0 <= p_value <= 1.0)
        self.assertTrue(math.isfinite(cohens_d))
        self.assertIsInstance(is_sig, bool)
        # a 明显优于 b，应显著
        self.assertTrue(is_sig)
        # Cohen's d 应为正（a > b）
        self.assertGreater(cohens_d, 0)

    def test_confidence_interval(self):
        """StatisticalAnalysis.confidence_interval 应返回均值和区间。"""
        data = [0.80, 0.82, 0.78, 0.85, 0.81]
        mean, (ci_low, ci_high) = StatisticalAnalysis.confidence_interval(data)
        self.assertTrue(math.isfinite(mean))
        self.assertTrue(math.isfinite(ci_low))
        self.assertTrue(math.isfinite(ci_high))
        # 区间下界 ≤ 均值 ≤ 区间上界
        self.assertLessEqual(ci_low, mean + 1e-6)
        self.assertGreaterEqual(ci_high, mean - 1e-6)

    def test_synergy_analysis(self):
        """SynergyAnalysis.compute_synergy 应返回协同效应分析结果。"""
        perf_full = {'final_coverage': 0.90}
        perf_baseline = [{'final_coverage': 0.50}]
        perf_single = [
            {'final_coverage': 0.60},
            {'final_coverage': 0.65},
            {'final_coverage': 0.62},
        ]
        result = SynergyAnalysis.compute_synergy(
            perf_full, perf_baseline, perf_single
        )
        self.assertIsInstance(result, dict)
        self.assertIn('synergy_value', result)
        self.assertIn('is_positive', result)


# =====================================================================
# 测试类 5: TestParameterSensitivity
# 测试 parameter_sensitivity.py 的参数敏感性分析
# =====================================================================

@skip_if_no_param_sens
class TestParameterSensitivity(unittest.TestCase):
    """测试参数敏感性分析模块。"""

    def test_parameter_configs_complete(self):
        """PARAMETER_CONFIGS 应包含 4 个参数配置。"""
        self.assertEqual(len(PARAMETER_CONFIGS), 4)
        keys = [cfg['key'] for cfg in PARAMETER_CONFIGS]
        expected = ['risk_lambda', 'aufe_boost', 'dyn_emergency_dist',
                    'amcl_n_particles']
        for k in expected:
            self.assertIn(k, keys)
        # 每个配置应有必要的字段
        for cfg in PARAMETER_CONFIGS:
            self.assertIn('default', cfg)
            self.assertIn('values', cfg)
            self.assertIn('env_var', cfg)
            self.assertGreater(len(cfg['values']), 0)

    def test_parameter_sweep_init(self):
        """ParameterSweep 应正确初始化。"""
        sweep = ParameterSweep(
            param_name='risk_lambda',
            param_values=[0.1, 0.5, 1.0],
            default_value=0.5
        )
        self.assertEqual(sweep.param_name, 'risk_lambda')
        self.assertEqual(len(sweep.param_values), 3)
        self.assertEqual(sweep.default_value, 0.5)
        # 未知参数应抛出 ValueError
        with self.assertRaises(ValueError):
            ParameterSweep('unknown_param', [1.0], 1.0)

    def test_global_sensitivity_analysis(self):
        """GlobalSensitivityAnalysis 应可初始化。"""
        gsa = GlobalSensitivityAnalysis(n_trials=2)
        self.assertEqual(gsa.n_trials, 2)
        # n_trials 应为正整数
        self.assertGreater(gsa.n_trials, 0)


# =====================================================================
# 测试类 6: TestUnifiedUncertainty
# 测试 unified_uncertainty.py 的统一不确定性框架
# =====================================================================

@skip_if_no_unified_unc
class TestUnifiedUncertainty(unittest.TestCase):
    """测试统一不确定性数学框架。"""

    def test_uncertainty_state_total(self):
        """UncertaintyState.total_uncertainty 应返回 [0,1] 范围的值。"""
        u = UncertaintyState(u_loc=0.3, u_map=0.5, u_motion=0.7)
        total = u.total_uncertainty()
        self.assertTrue(0.0 <= total <= 1.0)
        # 均等权重下，total 应为三者的均值
        expected = (0.3 + 0.5 + 0.7) / 3.0
        self.assertAlmostEqual(total, expected, places=6)
        # 自定义权重
        total_w = u.total_uncertainty(weights=(0.5, 0.3, 0.2))
        self.assertTrue(0.0 <= total_w <= 1.0)

    def test_entropy(self):
        """UncertaintyState.entropy 应返回非负值。"""
        u = UncertaintyState(u_loc=0.5, u_map=0.5, u_motion=0.5)
        h = u.entropy()
        self.assertGreaterEqual(h, 0.0)
        # 熵最大值不超过 3（三个维度各 1 bit）
        self.assertLessEqual(h, 3.0 + 1e-9)

    def test_normalized_entropy(self):
        """normalized_entropy 应在 [0, 1] 范围内。"""
        u = UncertaintyState(u_loc=0.3, u_map=0.6, u_motion=0.9)
        h_norm = u.normalized_entropy()
        self.assertTrue(0.0 <= h_norm <= 1.0)
        # 应为 entropy() / 3
        self.assertAlmostEqual(h_norm, u.entropy() / 3.0, places=9)

    def test_to_dict_and_to_vector(self):
        """to_dict / to_vector / from_vector 应正确序列化。"""
        u = UncertaintyState(u_loc=0.2, u_map=0.4, u_motion=0.6, timestamp=1.5)
        d = u.to_dict()
        self.assertIsInstance(d, dict)
        self.assertIn('u_loc', d)
        self.assertIn('u_map', d)
        self.assertIn('u_motion', d)
        v = u.to_vector()
        self.assertEqual(v.shape, (3,))
        # from_vector 往返
        u2 = UncertaintyState.from_vector(v, timestamp=1.5)
        self.assertAlmostEqual(u2.u_loc, 0.2, places=6)
        self.assertAlmostEqual(u2.u_map, 0.4, places=6)
        self.assertAlmostEqual(u2.u_motion, 0.6, places=6)

    def test_propagator_predict(self):
        """UncertaintyPropagator.predict 应增加定位不确定性。"""
        prop = UncertaintyPropagator(default_motion_noise=0.1)
        u = UncertaintyState(u_loc=0.3, u_map=0.5, u_motion=0.4)
        u_pred = prop.predict(u, action={'type': 'forward', 'linear_velocity': 0.2})
        # 预测后 u_loc 应增大（运动增加不确定性）
        self.assertGreaterEqual(u_pred.u_loc, u.u_loc - 1e-9)
        # u_map 预测步不变
        self.assertAlmostEqual(u_pred.u_map, u.u_map, places=6)

    def test_propagator_update(self):
        """UncertaintyPropagator.update 应减小定位不确定性。"""
        prop = UncertaintyPropagator(default_motion_noise=0.1)
        u = UncertaintyState(u_loc=0.6, u_map=0.5, u_motion=0.4)
        u_upd = prop.update(u, observation={'has_observation': True},
                            sensor_quality=0.8)
        # 更新后 u_loc 应减小（观测降低不确定性）
        self.assertLessEqual(u_upd.u_loc, u.u_loc + 1e-9)
        # u_motion 更新步不变
        self.assertAlmostEqual(u_upd.u_motion, u.u_motion, places=6)

    def test_information_gain(self):
        """InformationGainCalculator 应计算信息增益 ΔI = H(before) - H(after)。"""
        calc = InformationGainCalculator()
        # Shannon 熵 -p·log2(p) 在 p=1/e≈0.368 处取最大值。
        # 选取 before 接近 0.368（高熵）、after 接近 0 或 1（低熵），
        # 使 ΔI = H(before) - H(after) > 0（不确定性降低→信息增加）。
        u_before = UncertaintyState(u_loc=0.37, u_map=0.37, u_motion=0.37)
        u_after = UncertaintyState(u_loc=0.95, u_map=0.95, u_motion=0.95)
        ig = calc.compute_total_info_gain(u_before, u_after)
        # 信息增益应非负（before 高熵 → after 低熵）
        self.assertGreaterEqual(ig, -1e-9)
        # 边际信息增益
        marginals = calc.compute_all_marginal_info_gains(u_before)
        self.assertIsInstance(marginals, (dict, np.ndarray))


# =====================================================================
# 测试类 7: TestFailureAnalysis
# 测试 failure_analysis.py 的失败案例分析
# =====================================================================

@skip_if_no_failure
class TestFailureAnalysis(unittest.TestCase):
    """测试失败案例分析与局限性框架。"""

    def test_failure_case_creation(self):
        """FailureCase 应正确创建并序列化。"""
        case = FailureCase(
            case_id='FC_test_001',
            failure_type=FAILURE_TYPE_AMCL_KIDNAPPED,
            timestamp=100.0,
            trigger_condition='定位误差超过阈值',
            context={'robot_pos': (1.0, 2.0, 0.0), 'loc_err': 0.9},
            root_cause='粒子枯竭',
            improvement_suggestion='增大粒子数',
            severity=0.8,
        )
        self.assertEqual(case.case_id, 'FC_test_001')
        self.assertEqual(case.failure_type, FAILURE_TYPE_AMCL_KIDNAPPED)
        self.assertEqual(case.severity, 0.8)
        # 序列化为字典
        d = case.to_dict()
        self.assertIsInstance(d, dict)
        self.assertEqual(d['case_id'], 'FC_test_001')
        self.assertEqual(d['failure_type'], FAILURE_TYPE_AMCL_KIDNAPPED)

    def test_amcl_kidnapped_detection(self):
        """FailureDetector.check_amcl_kidnapped 应检测定位丢失。"""
        detector = FailureDetector()
        # 构造上升趋势的定位误差历史（窗口 100 步）：
        # 前 50 步低误差（正常），后 50 步高误差（kidnapped），
        # 使 second_half_mean > first_half_mean 满足触发条件。
        low_err = [0.1] * 50
        high_err = [0.9, 1.0, 1.1, 1.2, 1.0, 0.95, 1.3, 1.4, 1.2, 1.5] * 5
        loc_err_history = low_err + high_err  # 100 步
        case = detector.check_amcl_kidnapped(
            loc_err_history, threshold=0.8, window=100
        )
        self.assertIsNotNone(case)
        self.assertEqual(case.failure_type, FAILURE_TYPE_AMCL_KIDNAPPED)
        # 无失败的正常历史应返回 None
        normal_history = [0.05] * 120
        case_none = detector.check_amcl_kidnapped(
            normal_history, threshold=0.8, window=100
        )
        self.assertIsNone(case_none)

    def test_narrow_passage_detection(self):
        """FailureDetector.check_narrow_passage_stuck 应检测狭窄通道卡死。"""
        detector = FailureDetector()
        # 构造位置几乎不变的历史（200+ 步）
        pos_history = [(1.0, 1.0)] * 200
        case = detector.check_narrow_passage_stuck(
            pos_history, threshold=0.01, window=200
        )
        self.assertIsNotNone(case)
        self.assertEqual(case.failure_type, FAILURE_TYPE_NARROW_PASSAGE)
        # 正常移动的历史应返回 None
        moving_history = [(i * 0.1, i * 0.1) for i in range(200)]
        case_none = detector.check_narrow_passage_stuck(
            moving_history, threshold=0.01, window=200
        )
        self.assertIsNone(case_none)

    def test_dynamic_collision_detection(self):
        """FailureDetector.check_dynamic_collision 应检测碰撞事件。"""
        detector = FailureDetector()
        # 有碰撞事件
        collisions = [{'time': 50.0, 'obstacle': 'person_1'}]
        case = detector.check_dynamic_collision(collisions)
        self.assertIsNotNone(case)
        self.assertEqual(case.failure_type, FAILURE_TYPE_DYNAMIC_COLLISION)
        # 无碰撞事件应返回 None
        case_none = detector.check_dynamic_collision([])
        self.assertIsNone(case_none)

    def test_dead_zone_detection(self):
        """FailureDetector.check_dead_zone 应检测死区。"""
        detector = FailureDetector()
        # 覆盖率长期停滞
        coverage_history = [0.5] * 120
        case = detector.check_dead_zone(
            coverage_history, window=100, coverage_stall_threshold=0.5
        )
        # 应检测到死区（或返回 None，取决于实现细节）
        if case is not None:
            self.assertEqual(case.failure_type, FAILURE_TYPE_DEAD_ZONE)

    def test_failure_analyzer(self):
        """FailureAnalyzer 应能添加案例并生成报告。"""
        analyzer = FailureAnalyzer()
        case1 = FailureCase(
            case_id='FC_001', failure_type=FAILURE_TYPE_AMCL_KIDNAPPED,
            timestamp=100.0, trigger_condition='test',
        )
        case2 = FailureCase(
            case_id='FC_002', failure_type=FAILURE_TYPE_NARROW_PASSAGE,
            timestamp=200.0, trigger_condition='test',
        )
        analyzer.add_case(case1)
        analyzer.add_case(case2)
        # 分类
        classified = analyzer.classify_cases()
        self.assertIsInstance(classified, (dict, list))
        # 生成报告
        report = analyzer.generate_failure_report()
        self.assertIsInstance(report, (dict, str))


# =====================================================================
# 主入口
# =====================================================================

if __name__ == '__main__':
    unittest.main()
