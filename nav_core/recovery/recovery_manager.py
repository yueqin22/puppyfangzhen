"""Recovery Manager: independent module for robot recovery decisions.

Extracted from autonomous_nav.py (v5.2) to make recovery logic testable
and extensible. Implements:
  - v5.2: Smart escape direction selection (16-direction sampling)
  - v6.1: Recovery target point + Mini-Plan (short escape path planning)
  - v6.2: Recovery failure memory (history tracking, repeat-failure penalty)
  - v6.9: Belief-space recovery trigger (uncertainty threshold based decision)

The RecoveryManager is pure logic — it does NOT call CoppeliaSim APIs directly.
It returns a RecoveryResult describing what action the caller should take
(move, rotate, spread particles, etc.), and the caller executes it.

Dependencies:
  - costmap.Costmap (for cost queries)
  - nav_core.runtime.state_machine.NavState (for state constants)
  - belief_planner.BeliefState (optional, for v6.9 belief-space triggers)
"""
import math
import time
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, Callable
from collections import defaultdict

from costmap import Costmap, COST_LETHAL, COST_INSCRIBED


@dataclass
class RecoveryContext:
    """All data the RecoveryManager needs to make a decision.

    Encapsulates robot state, sensor data, and loop context without
    coupling to CoppeliaSim or any specific runtime.
    """
    rx: float = 0.0          # AMCL-estimated robot x
    ry: float = 0.0          # AMCL-estimated robot y
    ryaw: float = 0.0        # robot heading
    true_x: float = 0.0      # ground-truth x (for evaluation)
    true_y: float = 0.0
    true_yaw: float = 0.0
    frame: int = 0
    recover_frames: int = 0  # frames spent in current RECOVER cycle
    loc_err: float = 0.0     # AMCL localization error
    loc_conf: float = 1.0    # AMCL confidence
    robot_cost: int = 0      # costmap cost at robot position
    in_lethal: bool = False
    near_doorway: bool = False
    oscillating: bool = False
    current_path: Optional[List[Tuple[float, float]]] = None
    current_goal: Optional[Tuple[float, float]] = None
    last_recover_dir: Optional[float] = None
    doorway_crossing_frames: List[int] = field(default_factory=list)
    consecutive_recover_failures: int = 0
    loc_covariance_trace: float = 0.0  # for v6.9 belief-space trigger
    degraded: bool = False  # DEGRADED 状态标记：强制降级后置True


@dataclass
class RecoveryResult:
    """Action the caller should execute after RecoveryManager decides.

    The caller (autonomous_nav.py) reads these fields and performs the
    actual CoppeliaSim API calls.
    """
    action: str = 'continue'  # 'move', 'rotate', 'spread_particles',
                              # 'spin', 'finish', 'continue', 'block_area'
    move_x: float = 0.0       # target world x for 'move'
    move_y: float = 0.0       # target world y for 'move'
    move_yaw: float = 0.0     # target heading for 'move'
    rotate_delta: float = 0.0 # rotation delta for 'rotate' (radians)
    spread_x: float = 0.0     # AMCL spread center x
    spread_y: float = 0.0
    spread_yaw: float = 0.0
    spread_radius: float = 1.0
    spread_label: str = ''    # 'LOCAL', 'WIDE', 'GLOBAL'
    block_area_x: float = 0.0 # area to mark unreachable (v6.2)
    block_area_y: float = 0.0
    block_area_radius: float = 0.0
    message: str = ''         # log message
    should_finish: bool = False  # transition back to PLAN


class RecoveryMemory:
    """v6.2: Recovery failure memory.

    Tracks recovery attempt history to:
      1. Detect repeated failures in the same area (spatial clustering)
      2. Escalate spread radius for persistent failures
      3. Block areas with chronic failure (3+ consecutive no_progress)
    """

    def __init__(self, cluster_radius=2.0, max_history=50):
        self.cluster_radius = cluster_radius
        self.max_history = max_history
        self.failure_history = []  # list of (x, y, frame, reason)
        self.area_blocks = {}      # (grid_x, grid_y) -> block_frame

    def record_failure(self, x, y, frame, reason='no_progress'):
        """Record a recovery failure at (x, y) with reason."""
        self.failure_history.append((x, y, frame, reason))
        if len(self.failure_history) > self.max_history:
            self.failure_history.pop(0)

    def count_recent_failures(self, x, y, frame, window=100):
        """Count failures near (x, y) within recent frames."""
        count = 0
        for fx, fy, fr, _ in self.failure_history:
            dist = math.sqrt((fx - x)**2 + (fy - y)**2)
            if dist < self.cluster_radius and (frame - fr) < window:
                count += 1
        return count

    def should_block_area(self, x, y, frame, threshold=3):
        """Check if an area should be blocked due to repeated failures."""
        return self.count_recent_failures(x, y, frame) >= threshold

    def escalate_spread(self, consecutive_failures):
        """v6.1/v6.2: Escalate AMCL spread based on failure count.

        Returns: (spread_radius, label)
        """
        if consecutive_failures >= 3:
            return 5.0, 'GLOBAL'
        elif consecutive_failures >= 1:
            return 3.0, 'WIDE'
        else:
            return 1.0, 'LOCAL'

    def get_state(self):
        """Serializable state for persistence."""
        return {
            'failure_history': list(self.failure_history),
            'area_blocks': {f"{k[0]},{k[1]}": v for k, v in self.area_blocks.items()},
        }

    def set_state(self, state):
        """Restore state from persistence."""
        self.failure_history = [tuple(x) for x in state.get('failure_history', [])]
        self.area_blocks = {tuple(int(v) for v in k.split(',')): v
                            for k, v in state.get('area_blocks', {}).items()}


class MiniPlan:
    """v6.1: Short-range escape plan.

    When the robot is stuck, instead of random direction sampling, plan
    a short multi-step path to escape the stuck situation. Uses a simple
    breadth-first search in the costmap to find the nearest safe cell.
    """

    def __init__(self, costmap, max_steps=5, step_size=0.3):
        self.costmap = costmap
        self.max_steps = max_steps
        self.step_size = step_size

    def find_escape_path(self, sx, sy, goal_x=None, goal_y=None):
        """Find a short escape path from (sx, sy) to a safe cell.

        Uses BFS with 8-connected neighbors, limited depth.
        Returns: list of (x, y) waypoints, or None if no path found.
        """
        from collections import deque
        queue = deque()
        queue.append((sx, sy, []))
        visited = set()
        visited.add((round(sx, 1), round(sy, 1)))

        for _ in range(200):  # limit iterations
            if not queue:
                break
            cx, cy, path = queue.popleft()
            cost = self.costmap.get_cost(cx, cy)
            if cost < 50 and len(path) > 0:
                # Found a safe cell
                return path
            if len(path) >= self.max_steps:
                continue
            for i in range(8):
                ang = i * math.pi / 4
                nx = cx + self.step_size * math.cos(ang)
                ny = cy + self.step_size * math.sin(ang)
                key = (round(nx, 1), round(ny, 1))
                if key in visited:
                    continue
                ncost = self.costmap.get_cost(nx, ny)
                if ncost >= COST_INSCRIBED:
                    continue
                visited.add(key)
                new_path = path + [(nx, ny)]
                # Prioritize toward goal if given
                if goal_x is not None and goal_y is not None:
                    # Check if this direction is toward goal
                    to_goal = math.atan2(goal_y - sy, goal_x - sx)
                    dir_to_n = math.atan2(ny - sy, nx - sx)
                    if abs(dir_to_n - to_goal) < math.pi / 2:
                        queue.appendleft((nx, ny, new_path))  # prioritize
                    else:
                        queue.append((nx, ny, new_path))
                else:
                    queue.append((nx, ny, new_path))
        return None


class RecoveryManager:
    """Independent recovery decision maker (v5.2).

    Encapsulates all RECOVER state logic that was previously inline in
    autonomous_nav.py. The caller provides a RecoveryContext each frame
    and receives a RecoveryResult describing what to do.

    Features:
      - v5.2: 16-direction smart escape with path alignment
      - v6.1: Mini-Plan escape path when all directions blocked
      - v6.2: Failure memory + area blocking + spread escalation
      - v6.9: Belief-space trigger (loc_conf < threshold → force spread)
    """

    def __init__(self, costmap, config=None):
        self.costmap = costmap
        cfg = config or {}

        # Escape parameters
        self.normal_step = cfg.get('normal_step', 0.15)
        self.escape_step = cfg.get('escape_step', 0.6)
        self.max_step = cfg.get('max_step', 0.6)
        self.n_directions = cfg.get('n_directions', 16)

        # AMCL recovery parameters
        self.spread_trigger_frame = cfg.get('spread_trigger_frame', 5)
        self.max_recover_frames = cfg.get('max_recover_frames', 10)
        self.loc_err_threshold = cfg.get('loc_err_threshold', 0.5)
        self.critical_loc_err = cfg.get('critical_loc_err', 5.0)

        # v6.9: Belief-space recovery
        self.belief_conf_threshold = cfg.get('belief_conf_threshold', 0.5)
        self.belief_cov_threshold = cfg.get('belief_cov_threshold', 0.5)
        self.belief_enabled = cfg.get('belief_enabled', False)

        # v6.2: Failure memory
        self.memory = RecoveryMemory(
            cluster_radius=cfg.get('memory_cluster_radius', 2.0),
            max_history=cfg.get('memory_max_history', 50),
        )

        # v6.1: Mini-Plan
        self.mini_plan = MiniPlan(
            costmap,
            max_steps=cfg.get('mini_plan_max_steps', 5),
            step_size=cfg.get('mini_plan_step_size', 0.3),
        )

        # State tracking
        self._amcl_respread_done = False

        # 强制降级机制：连续恢复失败超过阈值后不再尝试恢复
        self.max_consecutive_failures = cfg.get('max_consecutive_failures', 5)

        # 恢复策略超时（秒）：超时后自动切换到下一个策略
        self.strategy_timeout = {
            'spread': cfg.get('spread_timeout', 2.0),
            'rotate': cfg.get('rotate_timeout', 5.0),
        }
        # 策略计时跟踪
        self._strategy_start_time = {}
        self._exhausted_strategies = set()

    def decide(self, ctx: RecoveryContext) -> RecoveryResult:
        """Main entry: decide recovery action for this frame.

        Args:
            ctx: RecoveryContext with current robot state and sensor data.

        Returns:
            RecoveryResult describing the action to take.
        """
        # 强制降级：连续恢复失败超过阈值，不再尝试恢复
        if ctx.consecutive_recover_failures > self.max_consecutive_failures:
            ctx.degraded = True
            return RecoveryResult(
                action='degraded',
                message=f"DEGRADED: consecutive_recover_failures="
                        f"{ctx.consecutive_recover_failures} 超过 "
                        f"max={self.max_consecutive_failures} — "
                        f"请求上层降低速度或人工干预",
            )

        if ctx.recover_frames == 0:
            self._amcl_respread_done = False
            self._strategy_start_time = {}
            self._exhausted_strategies = set()

        # Phase 0: Check if recovery should finish (超过最大帧数)
        if ctx.recover_frames > self.max_recover_frames:
            return self._finish_recovery(ctx)

        # Phase 1: AMCL particle recovery at frame 5 (优先于escape)
        # 受策略超时限制：spread超时后跳过此阶段
        if (ctx.recover_frames == self.spread_trigger_frame
                and self._is_strategy_available('spread')):
            return self._amcl_recovery(ctx)

        # Phase 2: Spin for AMCL convergence (frames 6-8)
        # 受策略超时限制：rotate超时后跳过此阶段
        if (ctx.recover_frames in (6, 7, 8)
                and self._amcl_respread_done
                and self._is_strategy_available('rotate')):
            return RecoveryResult(
                action='spin',
                rotate_delta=math.pi / 2,
                message=f"Spin frame {ctx.recover_frames}/8 "
                        f"yaw_delta=+90°"
                        f"{' [in_lethal]' if ctx.in_lethal else ''}",
            )

        # Phase 3: Try smart escape (direction sampling)
        escape_result = self._smart_escape(ctx)
        if escape_result is not None:
            return escape_result

        # Phase 4: Lethal zone escape
        if ctx.in_lethal and ctx.recover_frames % 3 == 0:
            return RecoveryResult(
                action='continue',
                message=f"In lethal zone (cost={ctx.robot_cost}), "
                        f"escaping first (frame {ctx.recover_frames})",
            )

        return RecoveryResult(action='continue')

    def _smart_escape(self, ctx: RecoveryContext) -> Optional[RecoveryResult]:
        """v5.2: 16-direction smart escape with path alignment."""
        step = self.escape_step if (ctx.in_lethal or ctx.oscillating) else self.normal_step
        osc_escape_dir = math.pi / 2 if ctx.true_y >= 0 else -math.pi / 2
        backward_dir = ctx.ryaw + math.pi

        # Compute path reference direction
        path_ref_dir = self._compute_path_ref_dir(ctx)

        best_dir = None
        best_score = -float('inf')

        for i in range(self.n_directions):
            ang = i * 2 * math.pi / self.n_directions
            # Skip opposite of last movement (not in lethal)
            if not ctx.in_lethal and ctx.last_recover_dir is not None:
                diff = abs(ang - ctx.last_recover_dir)
                diff = min(diff, 2 * math.pi - diff)
                if diff > math.pi * 0.75:
                    continue
            tx = ctx.rx + step * math.cos(ang)
            ty = ctx.ry + step * math.sin(ang)
            c = self.costmap.get_cost(tx, ty)
            if c >= COST_INSCRIBED:
                continue
            cost_score = 1.0 - (c / COST_INSCRIBED)
            if ctx.oscillating:
                osc_diff = abs(ang - osc_escape_dir)
                osc_diff = min(osc_diff, 2 * math.pi - osc_diff)
                osc_score = 1.0 - osc_diff / math.pi
                combined = 0.5 * cost_score + 0.5 * osc_score
            elif ctx.in_lethal:
                back_diff = abs(ang - backward_dir)
                back_diff = min(back_diff, 2 * math.pi - back_diff)
                back_score = 1.0 - back_diff / math.pi
                combined = 0.7 * cost_score + 0.3 * back_score
            elif path_ref_dir is not None:
                ang_diff = abs(ang - path_ref_dir)
                while ang_diff > math.pi:
                    ang_diff = 2 * math.pi - ang_diff
                align_score = 1.0 - ang_diff / math.pi
                combined = 0.4 * cost_score + 0.6 * align_score
            else:
                combined = cost_score
            if combined > best_score:
                best_score = combined
                best_dir = ang

        if best_dir is not None:
            # Use true position for movement (prevents teleportation when AMCL drifts)
            tx = ctx.true_x + step * math.cos(best_dir)
            ty = ctx.true_y + step * math.sin(best_dir)
            return RecoveryResult(
                action='move',
                move_x=tx,
                move_y=ty,
                move_yaw=best_dir,
                message=f"Escape dir={best_dir:.2f}rad step={step:.2f}m score={best_score:.2f}",
            )

        # v6.1: All directions blocked — try Mini-Plan escape
        return self._mini_plan_escape(ctx)

    def _mini_plan_escape(self, ctx: RecoveryContext) -> Optional[RecoveryResult]:
        """v6.1: Use Mini-Plan to find a multi-step escape path."""
        if not (ctx.in_lethal or ctx.oscillating):
            # Not stuck enough for mini-plan, just rotate
            return RecoveryResult(
                action='rotate',
                rotate_delta=0.5,
                message="All directions blocked — rotating in place",
            )

        goal_x = ctx.current_goal[0] if ctx.current_goal else None
        goal_y = ctx.current_goal[1] if ctx.current_goal else None
        escape_path = self.mini_plan.find_escape_path(
            ctx.true_x, ctx.true_y, goal_x, goal_y)

        if escape_path and len(escape_path) > 0:
            first_wp = escape_path[0]
            return RecoveryResult(
                action='move',
                move_x=first_wp[0],
                move_y=first_wp[1],
                move_yaw=math.atan2(first_wp[1] - ctx.true_y,
                                    first_wp[0] - ctx.true_x),
                message=f"Mini-Plan escape: {len(escape_path)} steps to safe cell",
            )

        # Try progressively larger steps (original fallback)
        for big_step in [0.4, 0.5, self.max_step]:
            for i in range(self.n_directions):
                ang = i * 2 * math.pi / self.n_directions
                cost_tx = ctx.rx + big_step * math.cos(ang)
                cost_ty = ctx.ry + big_step * math.sin(ang)
                if self.costmap.get_cost(cost_tx, cost_ty) < COST_INSCRIBED:
                    move_tx = ctx.true_x + big_step * math.cos(ang)
                    move_ty = ctx.true_y + big_step * math.sin(ang)
                    return RecoveryResult(
                        action='move',
                        move_x=move_tx,
                        move_y=move_ty,
                        move_yaw=ang,
                        message=f"Escaped with step={big_step}m dir={ang:.2f}rad",
                    )

        # All blocked — rotate in place
        return RecoveryResult(
            action='rotate',
            rotate_delta=0.5,
            message="All directions blocked — rotating in place",
        )

    def _compute_path_ref_dir(self, ctx: RecoveryContext) -> Optional[float]:
        """Compute path reference direction from current path."""
        if not ctx.current_path or len(ctx.current_path) < 2:
            return None
        min_d = float('inf')
        path_ref_dir = None
        for i in range(len(ctx.current_path) - 1):
            ax, ay = ctx.current_path[i]
            bx, by = ctx.current_path[i + 1]
            seg_dx = bx - ax
            seg_dy = by - ay
            seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len2 < 1e-9:
                continue
            proj_t = ((ctx.rx - ax) * seg_dx + (ctx.ry - ay) * seg_dy) / seg_len2
            proj_t = max(0.0, min(1.0, proj_t))
            proj_x = ax + proj_t * seg_dx
            proj_y = ay + proj_t * seg_dy
            d = math.sqrt((ctx.rx - proj_x) ** 2 + (ctx.ry - proj_y) ** 2)
            if d < min_d:
                min_d = d
                if d > 0.3:
                    path_ref_dir = math.atan2(proj_y - ctx.ry, proj_x - ctx.rx)
                else:
                    path_ref_dir = math.atan2(seg_dy, seg_dx)
        return path_ref_dir

    def _amcl_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        """v5.2 + v6.2: AMCL particle re-spread with escalation."""
        if ctx.loc_err <= self.loc_err_threshold and not self._should_trigger_belief(ctx):
            self._amcl_respread_done = False
            return RecoveryResult(
                action='continue',
                message=f"loc_err={ctx.loc_err:.3f} already low — "
                        f"skipping particle re-spread AND spin",
            )

        # v6.9: Belief-space trigger
        if self._should_trigger_belief(ctx):
            return self._belief_space_recovery(ctx)

        # v6.2: Escalate spread based on failure memory
        spread, label = self.memory.escalate_spread(ctx.consecutive_recover_failures)

        # v6.2: Critical ground-truth fallback
        if ctx.loc_err > self.critical_loc_err and ctx.consecutive_recover_failures >= 4:
            return RecoveryResult(
                action='spread_particles',
                spread_x=ctx.true_x,
                spread_y=ctx.true_y,
                spread_yaw=ctx.true_yaw,
                spread_radius=0.5,
                spread_label='CRITICAL_GT',
                message=f"CRITICAL: loc_err={ctx.loc_err:.1f}m with "
                        f"{ctx.consecutive_recover_failures} failures — "
                        f"using ground-truth position fix",
            )

        self._amcl_respread_done = True
        return RecoveryResult(
            action='spread_particles',
            spread_x=ctx.rx,
            spread_y=ctx.ry,
            spread_yaw=ctx.ryaw,
            spread_radius=spread,
            spread_label=label,
            message=f"Stuck for {self.spread_trigger_frame} frames — re-localizing "
                    f"({label}, spread={spread}). "
                    f"loc_err={ctx.loc_err:.3f} conf={ctx.loc_conf:.3f} "
                    f"failures={ctx.consecutive_recover_failures}"
                    f"{' [in_lethal]' if ctx.in_lethal else ''}",
        )

    def _should_trigger_belief(self, ctx: RecoveryContext) -> bool:
        """v6.9: Check if belief-space recovery should trigger."""
        if not self.belief_enabled:
            return False
        return (ctx.loc_conf < self.belief_conf_threshold or
                ctx.loc_covariance_trace > self.belief_cov_threshold)

    def _belief_space_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        """v6.9: Belief-space recovery — larger spread for high uncertainty."""
        spread = 5.0 if ctx.loc_conf < 0.3 else 3.0
        self._amcl_respread_done = True
        return RecoveryResult(
            action='spread_particles',
            spread_x=ctx.true_x,
            spread_y=ctx.true_y,
            spread_yaw=ctx.true_yaw,
            spread_radius=spread,
            spread_label='BELIEF',
            message=f"Belief-space recovery: conf={ctx.loc_conf:.3f} "
                    f"cov_trace={ctx.loc_covariance_trace:.3f} "
                    f"→ spread={spread}m",
        )

    def _finish_recovery(self, ctx: RecoveryContext) -> RecoveryResult:
        """Finish recovery and transition back to PLAN."""
        result = RecoveryResult(
            action='finish',
            should_finish=True,
            message="Recovery done",
        )

        # v6.2: Record failure if loc_err still high
        if ctx.loc_err > self.loc_err_threshold:
            self.memory.record_failure(
                ctx.true_x, ctx.true_y, ctx.frame, 'loc_err_high')
            result.message += f". loc_err={ctx.loc_err:.3f} still high — recorded failure"
        else:
            result.message += f". loc_err={ctx.loc_err:.3f} conf={ctx.loc_conf:.3f}"

        # v6.2: Check if area should be blocked
        if self.memory.should_block_area(ctx.true_x, ctx.true_y, ctx.frame):
            result.action = 'block_area'
            result.block_area_x = ctx.true_x
            result.block_area_y = ctx.true_y
            result.block_area_radius = 3.0
            result.message += f" — blocking 3m area at ({ctx.true_x:.1f},{ctx.true_y:.1f})"

        return result

    def _is_strategy_available(self, strategy_name: str) -> bool:
        """检查恢复策略是否可用（未超时）。

        首次调用时记录开始时间，后续调用检查是否超过
        strategy_timeout 中定义的最大尝试时间。超时后标记为
        耗尽，不再尝试该策略，自动切换到下一个策略。

        Args:
            strategy_name: 策略名称，如 'spread', 'rotate'。

        Returns:
            True 表示策略仍可用，False 表示已超时应切换策略。
        """
        if strategy_name in self._exhausted_strategies:
            return False
        if strategy_name not in self.strategy_timeout:
            return True
        now = time.time()
        if strategy_name not in self._strategy_start_time:
            self._strategy_start_time[strategy_name] = now
            return True
        elapsed = now - self._strategy_start_time[strategy_name]
        if elapsed > self.strategy_timeout[strategy_name]:
            self._exhausted_strategies.add(strategy_name)
            return False
        return True

    def reset(self):
        """Reset transient state for a new recovery cycle."""
        self._amcl_respread_done = False
        self._strategy_start_time = {}
        self._exhausted_strategies = set()

    def get_state(self):
        """Get serializable state for persistence."""
        return {'memory': self.memory.get_state()}

    def set_state(self, state):
        """Restore state from persistence."""
        if 'memory' in state:
            self.memory.set_state(state['memory'])
