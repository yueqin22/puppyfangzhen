"""Unit tests for nav_core.runtime.runtime_state."""
from nav_core.runtime import NavigationRuntimeState, NavState


def test_start_following_sets_state_and_resets_counters():
    runtime = NavigationRuntimeState(
        no_progress_frames=9,
        align_frames=4,
        current_goal=(9.0, 9.0),
    )

    runtime.start_following(
        path=[(1.0, 1.0), (2.0, 2.0)],
        goal=(3.0, 3.0),
        goal_plan_attempts=2,
    )

    assert runtime.state == NavState.FOLLOW
    assert runtime.current_path == [(1.0, 1.0), (2.0, 2.0)]
    assert runtime.current_goal == (3.0, 3.0)
    assert runtime.goal_plan_attempts == 2
    assert runtime.no_progress_frames == 0
    assert runtime.align_frames == 0


def test_return_to_planning_clears_goal_and_optional_progress():
    runtime = NavigationRuntimeState(
        state=NavState.FOLLOW,
        current_path=[(1.0, 1.0)],
        current_goal=(2.0, 2.0),
        goal_plan_attempts=3,
        no_progress_frames=7,
    )

    runtime.return_to_planning(reset_no_progress=True)

    assert runtime.state == NavState.PLAN
    assert runtime.current_path is None
    assert runtime.current_goal is None
    assert runtime.goal_plan_attempts == 0
    assert runtime.no_progress_frames == 0


def test_start_and_finish_recovery_manage_transient_state():
    runtime = NavigationRuntimeState(
        state=NavState.FOLLOW,
        align_frames=5,
        no_progress_frames=4,
        recover_frames=2,
        last_recover_dir=1.2,
    )

    runtime.start_recovery(reset_align=True)
    assert runtime.state == NavState.RECOVER
    assert runtime.recover_frames == 0
    assert runtime.align_frames == 0

    runtime.no_progress_frames = 8
    runtime.align_frames = 3
    runtime.last_recover_dir = 2.4
    runtime.finish_recovery()

    assert runtime.state == NavState.PLAN
    assert runtime.recover_frames == 0
    assert runtime.no_progress_frames == 0
    assert runtime.align_frames == 0
    assert runtime.last_recover_dir is None
