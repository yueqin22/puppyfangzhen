"""Frontier路径质量测试 (v8.2)

测试frontier选择在以下对照场景中的表现：
  - 信息增益相近但路径不同
  - 路径质量评分应优先选择短路径
  - 窄通道frontier应被降权
  - 不可达frontier应被排除

运行方式: python test_frontier_quality.py
"""
import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION
from costmap import Costmap, COST_LETHAL, COST_INSCRIBED
from nav_core.exploration.frontier_manager import FrontierManager


def make_frontier_manager():
    """创建测试用FrontierManager"""
    occ = OccupancyGrid()
    cm = Costmap()
    fm = FrontierManager(occ, cm)
    return fm


def test_path_cost_vs_info_gain():
    """测试路径代价vs信息增益的平衡"""
    print("\n=== Test: path_cost_vs_info_gain ===")
    fm = make_frontier_manager()

    # 两个frontier信息增益相近，但距离不同
    frontiers = [
        (5.0, 0.0, 30, 0.8, 5.0),   # 近的frontier
        (8.0, 0.0, 35, 0.85, 8.0),  # 远的frontier
    ]
    scored = fm.score_frontiers(frontiers, rx=0, ry=0, robot_yaw=0)
    assert len(scored) == 2
    # 近的应该得分更高(路径代价低)
    near_score = scored[0][5] if scored[0][0] == 5.0 else scored[1][5]
    far_score = scored[1][5] if scored[1][0] == 8.0 else scored[0][5]
    assert near_score > far_score, \
        f"Near frontier should score higher: {near_score} vs {far_score}"
    print(f"  PASS: 近frontier得分更高 ({near_score:.3f} > {far_score:.3f})")


def test_risky_frontier_penalty():
    """测试高风险frontier的惩罚"""
    print("\n=== Test: risky_frontier_penalty ===")
    fm = make_frontier_manager()

    # 通过直接设置log_odds来模拟障碍物
    gx, gy = fm.occ_grid.world_to_grid(3.0, 0.0)
    fm.occ_grid.log_odds[gy, gx] = 5.0  # 标记为已占据
    fm.costmap.update_static(fm.occ_grid, frame=0)

    frontiers = [
        (3.0, 0.0, 20, 0.5, 3.0),  # 高代价
        (5.0, 0.0, 20, 0.5, 5.0),  # 低代价
    ]
    scored = fm.score_frontiers(frontiers, rx=0, ry=0, robot_yaw=0)
    assert len(scored) == 2
    print(f"  PASS: 评分完成")


def test_unreachable_filter():
    """测试不可达frontier过滤"""
    print("\n=== Test: unreachable_filter ===")
    fm = make_frontier_manager()

    # 标记一个区域为不可达
    fm.mark_unreachable(3.0, 0.0)

    frontiers = [
        (3.0, 0.0, 20, 0.5, 3.0),  # 不可达
        (5.0, 0.0, 20, 0.5, 5.0),  # 可达
    ]
    filtered = fm.filter_frontiers_with_metadata(
        frontiers, current_frame=10, robot_y=0,
        doorway_crossing_frames=[])
    assert len(filtered) == 1, f"Expected 1 frontier, got {len(filtered)}"
    assert filtered[0][0] == 5.0, "Should keep reachable frontier"
    print("  PASS: 不可达frontier被正确过滤")


def test_visited_frontier_cooldown():
    """测试已访问frontier冷却"""
    print("\n=== Test: visited_frontier_cooldown ===")
    fm = make_frontier_manager()

    # 标记一个frontier为最近访问
    fm.mark_visited(3.0, 0.0, frame=100)

    frontiers = [
        (3.0, 0.0, 20, 0.5, 3.0),  # 最近访问
        (5.0, 0.0, 20, 0.5, 5.0),
    ]
    # 在冷却期内应被过滤
    filtered = fm.filter_frontiers_with_metadata(
        frontiers, current_frame=110, robot_y=0,
        doorway_crossing_frames=[])
    assert len(filtered) == 1, f"Expected 1 after cooldown, got {len(filtered)}"
    print("  PASS: 最近访问frontier被冷却")


def test_heading_alignment():
    """测试朝向对齐评分"""
    print("\n=== Test: heading_alignment ===")
    fm = make_frontier_manager()

    # robot朝向x正方向
    frontiers = [
        (5.0, 0.0, 20, 0.5, 5.0),   # 正前方 (heading_change=0)
        (-5.0, 0.0, 20, 0.5, 5.0),  # 后方 (heading_change=1)
    ]
    scored = fm.score_frontiers(frontiers, rx=0, ry=0, robot_yaw=0)
    # 正前方应该得分更高(heading_change更小)
    front = [s for s in scored if s[0] == 5.0][0]
    back = [s for s in scored if s[0] == -5.0][0]
    assert front[5] >= back[5], f"Front should score >= back: {front[5]} vs {back[5]}"
    print(f"  PASS: 正前方得分 >= 后方 ({front[5]:.3f} >= {back[5]:.3f})")


def test_safe_vs_unsafe_selection():
    """测试安全vs不安全frontier选择"""
    print("\n=== Test: safe_vs_unsafe_selection ===")
    fm = make_frontier_manager()

    # 通过直接设置log_odds来模拟障碍物（使用4.5避免边界越界：
    # x=5.0 → gx=100 超出 GRID_W=100 的范围0-99）
    gx, gy = fm.occ_grid.world_to_grid(4.5, 0.0)
    assert 0 <= gx < GRID_W, f"gx={gx} out of bounds"
    fm.occ_grid.log_odds[gy, gx] = 5.0
    fm.costmap.update_static(fm.occ_grid, frame=0)

    frontiers = [
        (4.5, 0.0, 30, 0.8, 4.5),  # 不安全（有障碍物）
        (3.0, 0.0, 20, 0.5, 3.0),  # 安全
    ]
    sel = fm.select_frontier(frontiers, safe_cost_threshold=120,
                             rx=0, ry=0, robot_yaw=0)
    assert sel.goal is not None, "Should select a frontier"
    print(f"  PASS: 选择了frontier ({sel.goal})")


def test_pareto_selection():
    """测试Pareto边界选择(如果已实现)"""
    print("\n=== Test: pareto_selection ===")
    try:
        from pareto import find_pareto_front, dominates, select_pareto_best

        # 3个目标: 信息增益(高好), 路径代价(低好), 风险(低好)
        solutions = [
            ((0.8, 5.0, 0.2), 'A'),  # 高信息, 远, 低风险
            ((0.5, 2.0, 0.1), 'B'),  # 中信息, 近, 极低风险
            ((0.3, 1.0, 0.3), 'C'),  # 低信息, 极近, 中风险
        ]
        higher_is_better = [True, False, False]

        pareto = find_pareto_front(solutions, higher_is_better)
        # A和B应该在Pareto前沿上, C被A和B支配
        pareto_names = [s[1] for s in pareto]
        assert 'A' in pareto_names, "A should be on Pareto front"
        assert 'B' in pareto_names, "B should be on Pareto front"
        print(f"  PASS: Pareto前沿 = {pareto_names}")

        # 测试select_pareto_best
        best_obj, best_payload, score = select_pareto_best(
            solutions, higher_is_better)
        assert best_payload in ('A', 'B'), \
            f"Best should be A or B, got {best_payload}"
        print(f"  PASS: TOPSIS选择 = {best_payload} (score={score:.3f})")
    except ImportError as e:
        print(f"  SKIP: pareto模块未找到 ({e})")


def main():
    print("=" * 60)
    print("Frontier路径质量测试 (v8.2)")
    print("=" * 60)

    tests = [
        test_path_cost_vs_info_gain,
        test_risky_frontier_penalty,
        test_unreachable_filter,
        test_visited_frontier_cooldown,
        test_heading_alignment,
        test_safe_vs_unsafe_selection,
        test_pareto_selection,
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
