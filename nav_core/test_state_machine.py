"""Unit tests for nav_core.runtime.state_machine."""
from nav_core.runtime.state_machine import NavContext, NavState, NavStateMachine


def test_plan_resets_no_frontier_cycles_when_goal_selected():
    sm = NavStateMachine()
    ctx = NavContext(no_frontier_cycles=3)

    new_state, action = sm.transition_plan(
        ctx,
        path_found=True,
        has_reachable_frontier=True,
    )

    assert new_state == NavState.FOLLOW
    assert action == 'select_goal'
    assert ctx.no_frontier_cycles == 0


def test_plan_reaches_done_after_too_many_empty_cycles():
    sm = NavStateMachine()
    ctx = NavContext(no_frontier_cycles=sm.max_no_frontier_cycles)

    new_state, action = sm.transition_plan(
        ctx,
        path_found=False,
        has_reachable_frontier=False,
    )

    assert new_state == NavState.DONE
    assert action == 'done'


def test_follow_reports_goal_reached_inside_tolerance():
    sm = NavStateMachine()
    ctx = NavContext(
        rx=0.1,
        ry=0.1,
        current_goal=(0.0, 0.0),
    )

    new_state, action = sm.transition_follow(ctx)

    assert new_state == NavState.PLAN
    assert action == 'goal_reached'


def test_done_resumes_when_new_frontier_appears():
    sm = NavStateMachine()
    ctx = NavContext()

    new_state, action = sm.transition_done(ctx, has_reachable_frontier=True)

    assert new_state == NavState.PLAN
    assert action == 'resume'
