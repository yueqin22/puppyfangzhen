"""RecoveryManager单元测试 (v8.1)

测试恢复逻辑的各种场景：
  - lethal_zone: 机器人在致命区域中的恢复
  - oscillation: 门口震荡的恢复
  - path_aligned: 有路径参考时的恢复
  - mini_plan: Mini-Plan逃逸路径
  - amcl_respread: AMCL粒子重散布
  - failure_memory: 恢复失败记忆和区域封锁
  - belief_space: 信念空间恢复触发

运行方式: python test_recovery.py
"""
import sys
import os
import math
import numpy as np

# 确保项目根目录在路径中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from costmap import Costmap, COST_LETHAL, COST_INSCRIBED
from nav_core.recovery.recovery_manager import (
    RecoveryManager, RecoveryContext, RecoveryResult,
    RecoveryMemory, MiniPlan
)


def make_costmap():
    """创建测试用costmap"""
    cm = Costmap()
    return cm


def test_lethal_zone_escape():
    """测试致命区域逃逸"""
    print("\n=== Test: lethal_zone_escape ===")
    cm = make_costmap()
    rm = RecoveryManager(cm)

    # 设置机器人位置在致命区域
    ctx = RecoveryContext(
        rx=1.0, ry=1.0, ryaw=0.0,
        true_x=1.0, true_y=1.0,
        frame=0, recover_frames=0,
        robot_cost=250, in_lethal=True,
    )
    result = rm.decide(ctx)
    assert result.action == 'move', f"Expected 'move', got '{result.action}'"
    assert result.move_x != ctx.true_x or result.move_y != ctx.true_y, "Should move"
    print(f"  PASS: 逃逸到 ({result.move_x:.2f}, {result.move_y:.2f})")


def test_oscillation_escape():
    """测试门口震荡恢复"""
    print("\n=== Test: oscillation_escape ===")
    cm = make_costmap()
    rm = RecoveryManager(cm)

    ctx = RecoveryContext(
        rx=0.0, ry=0.1, ryaw=0.0,
        true_x=0.0, true_y=0.1,
        frame=0, recover_frames=0,
        near_doorway=True, oscillating=True,
    )
    result = rm.decide(ctx)
    assert result.action in ('move', 'rotate'), f"Expected 'move' or 'rotate', got '{result.action}'"
    print(f"  PASS: 震荡恢复动作: {result.action}")


def test_path_aligned_recovery():
    """测试路径对齐恢复"""
    print("\n=== Test: path_aligned_recovery ===")
    cm = make_costmap()
    rm = RecoveryManager(cm)

    # 提供一条路径
    path = [(0, 0), (0.5, 0), (1.0, 0), (1.5, 0)]
    ctx = RecoveryContext(
        rx=0.2, ry=0.3, ryaw=0.0,
        true_x=0.2, true_y=0.3,
        frame=0, recover_frames=0,
        current_path=path,
    )
    result = rm.decide(ctx)
    assert result.action in ('move', 'rotate'), f"Expected 'move' or 'rotate', got '{result.action}'"
    print(f"  PASS: 路径对齐恢复: {result.action} -> ({result.move_x:.2f}, {result.move_y:.2f})")


def test_amcl_respread():
    """测试AMCL粒子重散布"""
    print("\n=== Test: amcl_respread ===")
    cm = make_costmap()
    rm = RecoveryManager(cm)

    # 第5帧应触发AMCL重散布
    ctx = RecoveryContext(
        rx=1.0, ry=1.0, ryaw=0.0,
        true_x=1.0, true_y=1.0,
        frame=5, recover_frames=5,
        loc_err=0.8, loc_conf=0.3,
    )
    result = rm.decide(ctx)
    assert result.action == 'spread_particles', \
        f"Expected 'spread_particles', got '{result.action}'"
    assert result.spread_label in ('LOCAL', 'WIDE', 'GLOBAL'), \
        f"Unexpected label: {result.spread_label}"
    print(f"  PASS: AMCL重散布 {result.spread_label} spread={result.spread_radius}")


def test_critical_gt_recovery():
    """测试临界情况使用真值恢复"""
    print("\n=== Test: critical_gt_recovery ===")
    cm = make_costmap()
    rm = RecoveryManager(cm)

    ctx = RecoveryContext(
        rx=1.0, ry=1.0, ryaw=0.0,
        true_x=5.0, true_y=3.0,
        frame=5, recover_frames=5,
        loc_err=6.0, loc_conf=0.1,
        consecutive_recover_failures=4,
    )
    result = rm.decide(ctx)
    assert result.action == 'spread_particles', \
        f"Expected 'spread_particles', got '{result.action}'"
    assert result.spread_label == 'CRITICAL_GT', \
        f"Expected 'CRITICAL_GT', got '{result.spread_label}'"
    # 应使用真值位置
    assert abs(result.spread_x - 5.0) < 0.01, "Should use true_x"
    assert abs(result.spread_y - 3.0) < 0.01, "Should use true_y"
    print(f"  PASS: 临界恢复使用真值 ({result.spread_x:.1f}, {result.spread_y:.1f})")


def test_failure_memory():
    """测试恢复失败记忆"""
    print("\n=== Test: failure_memory ===")
    memory = RecoveryMemory(cluster_radius=2.0)

    # 记录3次失败
    for f in range(10, 40, 10):
        memory.record_failure(1.0, 1.0, f, 'no_progress')

    # 应检测到重复失败
    count = memory.count_recent_failures(1.0, 1.0, 40, window=100)
    assert count == 3, f"Expected 3 failures, got {count}"
    assert memory.should_block_area(1.0, 1.0, 40), "Should block area"

    # 扩散升级
    spread, label = memory.escalate_spread(3)
    assert spread == 5.0 and label == 'GLOBAL', f"Expected GLOBAL, got {label}"

    spread, label = memory.escalate_spread(1)
    assert spread == 3.0 and label == 'WIDE', f"Expected WIDE, got {label}"

    spread, label = memory.escalate_spread(0)
    assert spread == 1.0 and label == 'LOCAL', f"Expected LOCAL, got {label}"
    print("  PASS: 失败记忆正确")


def test_mini_plan():
    """测试Mini-Plan逃逸路径"""
    print("\n=== Test: mini_plan ===")
    cm = make_costmap()
    mp = MiniPlan(cm, max_steps=5, step_size=0.3)

    # 从开放位置找逃逸路径
    path = mp.find_escape_path(0, 0)
    # 在空costmap中应该能找到路径
    assert path is not None, "Should find escape path in open space"
    print(f"  PASS: 找到逃逸路径 ({len(path)}步)")


def test_belief_space_recovery():
    """测试信念空间恢复"""
    print("\n=== Test: belief_space_recovery ===")
    cm = make_costmap()
    rm = RecoveryManager(cm, config={
        'belief_enabled': True,
        'belief_conf_threshold': 0.5,
        'belief_cov_threshold': 0.5,
    })

    # 低置信度应触发信念恢复
    ctx = RecoveryContext(
        rx=1.0, ry=1.0, ryaw=0.0,
        true_x=1.0, true_y=1.0,
        frame=5, recover_frames=5,
        loc_err=0.3, loc_conf=0.2,
        loc_covariance_trace=1.0,
    )
    result = rm.decide(ctx)
    assert result.action == 'spread_particles', \
        f"Expected 'spread_particles', got '{result.action}'"
    assert result.spread_label == 'BELIEF', \
        f"Expected 'BELIEF', got '{result.spread_label}'"
    print(f"  PASS: 信念恢复 {result.spread_label} spread={result.spread_radius}")


def test_recovery_finish():
    """测试恢复完成"""
    print("\n=== Test: recovery_finish ===")
    cm = make_costmap()
    rm = RecoveryManager(cm, config={'max_recover_frames': 10})

    # 超过最大帧数应结束
    ctx = RecoveryContext(
        rx=1.0, ry=1.0, ryaw=0.0,
        true_x=1.0, true_y=1.0,
        frame=20, recover_frames=11,
        loc_err=0.1, loc_conf=0.95,
    )
    result = rm.decide(ctx)
    assert result.action == 'finish', f"Expected 'finish', got '{result.action}'"
    assert result.should_finish, "Should finish"
    print(f"  PASS: 恢复完成 (loc_err={ctx.loc_err:.3f})")


def test_recovery_finish_with_failure():
    """测试恢复完成但失败记录"""
    print("\n=== Test: recovery_finish_with_failure ===")
    cm = make_costmap()
    rm = RecoveryManager(cm, config={'max_recover_frames': 10})

    # loc_err仍高应记录失败
    ctx = RecoveryContext(
        rx=1.0, ry=1.0, ryaw=0.0,
        true_x=1.0, true_y=1.0,
        frame=20, recover_frames=11,
        loc_err=0.8, loc_conf=0.3,
    )
    result = rm.decide(ctx)
    assert result.action in ('finish', 'block_area'), \
        f"Expected 'finish' or 'block_area', got '{result.action}'"
    print(f"  PASS: 恢复完成(失败) action={result.action}")


def main():
    """运行所有测试"""
    print("=" * 60)
    print("RecoveryManager 单元测试 (v8.1)")
    print("=" * 60)

    tests = [
        test_lethal_zone_escape,
        test_oscillation_escape,
        test_path_aligned_recovery,
        test_amcl_respread,
        test_critical_gt_recovery,
        test_failure_memory,
        test_mini_plan,
        test_belief_space_recovery,
        test_recovery_finish,
        test_recovery_finish_with_failure,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  FAIL: {e}")
            failed += 1

    print("\n" + "=" * 60)
    print(f"结果: {passed} passed, {failed} failed, {passed + failed} total")
    print("=" * 60)
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
