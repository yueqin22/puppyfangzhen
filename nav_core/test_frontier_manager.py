"""Unit tests for nav_core.exploration.frontier_manager."""
from nav_core.exploration.frontier_manager import FrontierManager


class _DummyOccGrid:
    def world_to_grid(self, x, y):
        return int(round(x * 10)), int(round(y * 10))


class _DummyCostmap:
    def __init__(self, costs=None):
        self.costs = costs or {}

    def get_cost(self, x, y):
        return self.costs.get((x, y), 0)


def _manager(costs=None):
    mgr = FrontierManager(_DummyOccGrid(), _DummyCostmap(costs))
    mgr.visited_ttl = 10
    return mgr


def test_filter_frontiers_drops_unreachable_and_recently_visited():
    mgr = _manager()
    mgr.unreachable_goals.add((10, 10))
    mgr.visited_frontiers[(20, 20)] = 95

    filtered = mgr.filter_frontiers_with_metadata(
        [
            (1.0, 1.0, 5, 0.8, 1.0),
            (2.0, 2.0, 5, 0.7, 2.0),
            (3.0, 3.0, 5, 0.6, 3.0),
        ],
        current_frame=100,
        robot_y=0.0,
        doorway_crossing_frames=[],
    )

    assert filtered == [(3.0, 3.0, 5, 0.6, 3.0)]


def test_filter_frontiers_blocks_cross_doorway_targets_during_recent_oscillation():
    mgr = _manager()

    filtered = mgr.filter_frontiers_with_metadata(
        [
            (1.0, -1.0, 5, 0.8, 1.0),
            (1.0, 1.0, 5, 0.7, 1.1),
        ],
        current_frame=100,
        robot_y=1.0,
        doorway_crossing_frames=[65, 75, 90],
    )

    assert filtered == [(1.0, 1.0, 5, 0.7, 1.1)]


def test_select_frontier_prefers_safe_candidate():
    mgr = _manager({
        (1.0, 1.0): 150,
        (2.0, 2.0): 40,
    })

    selection = mgr.select_frontier([
        (1.0, 1.0, 5, 0.9, 1.0),
        (2.0, 2.0, 5, 0.8, 1.2),
    ])

    assert selection.goal == (2.0, 2.0)
    assert selection.safe_count == 1
    assert selection.unsafe_count == 1


def test_select_frontier_prefers_better_heading_when_scores_available():
    mgr = _manager({
        (3.0, 0.0): 10,
        (0.0, 3.0): 10,
    })

    selection = mgr.select_frontier(
        [
            (0.0, 3.0, 5, 10, 3.0),
            (3.0, 0.0, 5, 10, 3.0),
        ],
        rx=0.0,
        ry=0.0,
        robot_yaw=0.0,
    )

    assert selection.goal == (3.0, 0.0)
    assert selection.score > 0


def test_select_frontier_falls_back_to_lowest_cost_unsafe_candidate():
    mgr = _manager({
        (1.0, 1.0): 180,
        (2.0, 2.0): 140,
    })

    selection = mgr.select_frontier([
        (1.0, 1.0, 5, 0.9, 1.0),
        (2.0, 2.0, 5, 0.8, 1.2),
    ])

    assert selection.goal == (2.0, 2.0)
    assert selection.safe_count == 0
    assert selection.unsafe_count == 2


def test_select_frontier_penalizes_recently_visited_neighbors():
    mgr = _manager({
        (1.0, 1.0): 20,
        (4.0, 4.0): 20,
    })
    mgr.visited_frontiers[(10, 10)] = 95
    mgr.revisit_penalty_radius = 1.5

    selection = mgr.select_frontier(
        [
            (1.0, 1.0, 5, 10, 1.4),
            (4.0, 4.0, 5, 8, 5.6),
        ],
        rx=0.0,
        ry=0.0,
        robot_yaw=0.0,
    )

    assert selection.goal == (4.0, 4.0)
