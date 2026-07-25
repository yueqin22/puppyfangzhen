"""Mutable runtime state for the research navigation loop."""
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from collections import deque

from nav_core.runtime.state_machine import NavState

Waypoint = Tuple[float, float]
Path = List[Waypoint]


@dataclass
class NavigationRuntimeState:
    """Holds loop state and common transition-side resets."""

    state: NavState = NavState.PLAN
    current_path: Optional[Path] = None
    current_goal: Optional[Waypoint] = None
    recover_frames: int = 0
    no_progress_frames: int = 0
    align_frames: int = 0
    goal_plan_attempts: int = 0
    last_pos: Waypoint = (0.0, 0.0)
    last_recover_dir: Optional[float] = None
    no_frontier_cycles: int = 0

    # 状态超时机制：每个状态的最大停留时间（秒）
    state_start_time: dict = field(default_factory=dict)
    state_timeout: dict = field(default_factory=lambda: {
        'PLAN': 10.0, 'FOLLOW': 30.0, 'RECOVER': 15.0
    })

    # Watchdog监控：导航循环心跳检测
    last_update_time: float = 0.0
    watchdog_timeout: float = 5.0

    # 死锁检测：记录最近20次状态转换序列
    state_history: deque = field(default_factory=lambda: deque(maxlen=20))

    def __post_init__(self) -> None:
        """初始化状态计时，记录初始状态的进入时间。"""
        self.state_start_time[self.state.name] = time.time()
        self.state_history.append(self.state.name)

    def _enter_state(self, new_state: NavState) -> None:
        """记录状态转换并更新计时。

        Args:
            new_state: 要进入的新状态。
        """
        self.state = new_state
        # 记录状态转换序列（deque自动保留最近20条）
        self.state_history.append(new_state.name)
        # 重置新状态的进入时间
        self.state_start_time[new_state.name] = time.time()

    def start_following(
            self,
            path: Path,
            goal: Optional[Waypoint] = None,
            goal_plan_attempts: Optional[int] = None) -> None:
        """Enter FOLLOW with a fresh path and cleared progress counters."""
        self.current_path = path
        if goal is not None:
            self.current_goal = goal
        if goal_plan_attempts is not None:
            self.goal_plan_attempts = goal_plan_attempts
        self._enter_state(NavState.FOLLOW)
        self.no_progress_frames = 0
        self.align_frames = 0

    def clear_goal(self, clear_path: bool = True) -> None:
        """Drop the active goal and its planning attempt counter."""
        if clear_path:
            self.current_path = None
        self.current_goal = None
        self.goal_plan_attempts = 0

    def return_to_planning(
            self,
            clear_goal: bool = True,
            clear_path: bool = True,
            reset_no_progress: bool = False) -> None:
        """Enter PLAN and optionally clear navigation progress."""
        self._enter_state(NavState.PLAN)
        if clear_goal:
            self.clear_goal(clear_path=clear_path)
        elif clear_path:
            self.current_path = None
        if reset_no_progress:
            self.no_progress_frames = 0

    def start_recovery(self, reset_align: bool = False) -> None:
        """Enter RECOVER from planning or following."""
        self._enter_state(NavState.RECOVER)
        self.recover_frames = 0
        if reset_align:
            self.align_frames = 0

    def finish_recovery(self) -> None:
        """Return from recovery with cleared transient counters."""
        self._enter_state(NavState.PLAN)
        self.recover_frames = 0
        self.no_progress_frames = 0
        self.align_frames = 0
        self.last_recover_dir = None

    def reset_frontier_cycles(self) -> None:
        self.no_frontier_cycles = 0

    def mark_done(self) -> None:
        self._enter_state(NavState.DONE)

    def check_state_timeout(self) -> bool:
        """检查当前状态是否超时，超时则自动转换状态。

        各状态超时阈值由 state_timeout 定义：
          - PLAN 超时 → 转换到 RECOVER
          - FOLLOW 超时 → 转换到 RECOVER
          - RECOVER 超时 → 转换到 DEGRADED

        Returns:
            True 表示当前状态已超时并执行了状态转换。
        """
        state_name = self.state.name
        timeout = self.state_timeout.get(state_name)
        if timeout is None:
            return False
        start = self.state_start_time.get(state_name)
        if start is None:
            return False
        if time.time() - start <= timeout:
            return False
        # 超时，执行状态转换
        if self.state == NavState.RECOVER:
            self._enter_state(NavState.DEGRADED)
        elif self.state == NavState.DEGRADED:
            # DEGRADED 状态不自动恢复，保持降级
            return True
        else:
            # PLAN / FOLLOW 超时 → 进入 RECOVER
            self._enter_state(NavState.RECOVER)
        return True

    def update_watchdog(self) -> None:
        """更新watchdog时间戳。

        应在每帧导航循环开始时调用，用于检测导航循环是否卡死。
        """
        self.last_update_time = time.time()

    def watchdog_check(self) -> bool:
        """检查导航循环是否正常运行。

        如果超过 watchdog_timeout 秒没有调用 update_watchdog()，
        返回 False 表示导航可能卡死。

        Returns:
            True 表示正常运行，False 表示可能卡死。
        """
        if self.last_update_time == 0.0:
            return True  # 未初始化，不报错
        elapsed = time.time() - self.last_update_time
        return elapsed <= self.watchdog_timeout

    def detect_deadlock(self) -> bool:
        """检测状态机是否陷入死循环。

        分析最近20次状态转换序列，如果某个状态转换对
        （如 PLAN→RECOVER→PLAN）重复出现超过3次，
        返回 True 表示检测到死锁，并强制进入 DEGRADED 状态。

        检测逻辑：检查最近9个状态点是否形成 A↔B 交替模式
        （即4次完整的 A→B→A 循环，超过3次）。

        Returns:
            True 表示检测到死锁并已强制进入 DEGRADED 状态。
        """
        if len(self.state_history) < 9:
            return False
        recent = list(self.state_history)[-9:]
        a, b = recent[0], recent[1]
        if a == b:
            return False
        # 验证是否为 A↔B 交替模式
        for i, s in enumerate(recent):
            expected = a if i % 2 == 0 else b
            if s != expected:
                return False
        # 检测到死锁，强制进入 DEGRADED 状态
        self._enter_state(NavState.DEGRADED)
        return True
