"""Navigation State Machine: PLAN -> FOLLOW -> RECOVER -> DONE.

Abstracts the state machine logic from autonomous_nav.py so it can be
shared between runtimes. The actual sensor/actuator integration is
handled by the runtime adapter.

v5.6: NavStateMachine 纯化 — 增加完整的状态转换条件和上下文管理，
使状态决策逻辑完全独立于autonomous_nav.py。
"""
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Callable
import math


class NavState(IntEnum):
    """Navigation state machine states."""
    PLAN = 0
    FOLLOW = 1
    RECOVER = 2
    DONE = 3
    DEGRADED = 4  # 强制降级状态：连续恢复失败或死锁后进入


@dataclass
class NavContext:
    """Context passed to state machine for state transitions.

    Encapsulates all the data the state machine needs to make decisions,
    without coupling to any specific runtime (CoppeliaSim or ROS2).

    v5.6: 扩展了上下文字段，包含所有状态转换所需的信息。
    """
    rx: float = 0.0
    ry: float = 0.0
    ryaw: float = 0.0
    frame: int = 0
    coverage: float = 0.0
    visited_count: int = 0
    current_path: Optional[List[Tuple[float, float]]] = None
    current_goal: Optional[Tuple[float, float]] = None
    goal_plan_attempts: int = 0
    no_progress_frames: int = 0
    recover_frames: int = 0
    no_frontier_cycles: int = 0
    # v5.6 扩展字段
    target_coverage: float = 95.0
    max_frames: int = 600
    doorway_crossing_frames: List[int] = field(default_factory=list)
    has_path: bool = False
    has_reachable_frontier: bool = False


class NavStateMachine:
    """Pure state machine logic for navigation.

    This class only handles state transitions and decision logic.
    The actual planning, sensing, and actuation are delegated to
    callbacks provided by the runtime adapter.

    v5.6: 状态机纯化 — 所有状态转换条件都在这里定义，
    autonomous_nav.py 只负责执行动作，不再内联状态判断逻辑。
    """

    def __init__(self):
        self.state = NavState.PLAN
        self.goal_tolerance = 0.5
        self.path_end_threshold = 0.4
        self.goal_far_threshold = 0.6
        self.max_goal_attempts = 3
        self.max_no_progress = 30
        self.max_recover_frames = 10
        self.max_no_frontier_cycles = 5
        # v5.6: 门震荡参数
        self.doorway_osc_threshold = 3
        self.doorway_osc_window = 40

    def transition_plan(self, ctx: NavContext, path_found: bool, has_reachable_frontier: bool):
        """Handle PLAN state transitions.

        Returns: (new_state, action) where action is one of:
          'select_goal', 'recover', 'done', 'continue_goal', 'give_up_goal'
        """
        # Give up on current goal after too many failed attempts
        if ctx.current_goal is not None and ctx.goal_plan_attempts >= self.max_goal_attempts:
            return NavState.PLAN, 'give_up_goal'

        if ctx.current_goal is None:
            if has_reachable_frontier and path_found:
                ctx.no_frontier_cycles = 0
                return NavState.FOLLOW, 'select_goal'
            elif has_reachable_frontier and not path_found:
                ctx.no_frontier_cycles = 0
                return NavState.RECOVER, 'recover'
            else:  # no reachable frontier
                ctx.no_frontier_cycles += 1
                if ctx.no_frontier_cycles > self.max_no_frontier_cycles:
                    return NavState.DONE, 'done'
                return NavState.RECOVER, 'recover'
        else:
            # Continue with existing goal
            if path_found:
                ctx.no_frontier_cycles = 0
                return NavState.FOLLOW, 'continue_goal'
            else:
                return NavState.RECOVER, 'recover'

    def transition_follow(self, ctx: NavContext):
        """Handle FOLLOW state transitions.

        Returns: (new_state, action) where action is one of:
          'goal_reached', 'path_end_unreachable', 'no_progress', 'continue'
        """
        # Check goal reached
        if ctx.current_goal:
            dist_to_goal = math.sqrt(
                (ctx.rx - ctx.current_goal[0])**2 +
                (ctx.ry - ctx.current_goal[1])**2)
            if dist_to_goal < self.goal_tolerance:
                return NavState.PLAN, 'goal_reached'

        # Check path end reached but goal still far
        if ctx.current_path and ctx.current_goal:
            last_wx, last_wy = ctx.current_path[-1]
            dist_to_path_end = math.sqrt(
                (ctx.rx - last_wx)**2 + (ctx.ry - last_wy)**2)
            dist_to_goal = math.sqrt(
                (ctx.rx - ctx.current_goal[0])**2 +
                (ctx.ry - ctx.current_goal[1])**2)
            if dist_to_path_end < self.path_end_threshold and dist_to_goal > self.goal_far_threshold:
                return NavState.PLAN, 'path_end_unreachable'

        # Check no progress
        if ctx.no_progress_frames > self.max_no_progress:
            return NavState.RECOVER, 'no_progress'

        # v5.6: 检查门震荡
        if ctx.doorway_crossing_frames:
            recent = sum(1 for f in ctx.doorway_crossing_frames
                        if ctx.frame - f <= self.doorway_osc_window)
            if recent >= self.doorway_osc_threshold:
                return NavState.RECOVER, 'doorway_oscillation'

        return NavState.FOLLOW, 'continue'

    def transition_recover(self, ctx: NavContext):
        """Handle RECOVER state transitions.

        Returns: (new_state, action)
        """
        if ctx.recover_frames > self.max_recover_frames:
            return NavState.PLAN, 'retry_plan'
        return NavState.RECOVER, 'continue'

    def transition_done(self, ctx: NavContext, has_reachable_frontier: bool):
        """Handle DONE state transitions."""
        if has_reachable_frontier:
            return NavState.PLAN, 'resume'
        return NavState.DONE, 'continue'

    def should_stop(self, ctx: NavContext) -> bool:
        """v5.6: 检查是否应该停止探索"""
        if ctx.coverage >= ctx.target_coverage:
            return True
        if ctx.frame >= ctx.max_frames:
            return True
        return False

    def get_state_name(self) -> str:
        """获取当前状态名称"""
        return self.state.name
