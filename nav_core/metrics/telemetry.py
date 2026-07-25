"""Telemetry Collector: structured logging and performance metrics.

Implements the observability recommendations (gaijin1.md section 7.2):
  - Coverage curve
  - Frontier count
  - DWA valid trajectory ratio
  - Replanning count
  - Recover trigger count and reasons
  - Average planning time
  - Collision/near-collision count

All metrics are collected in a structured way for offline analysis and
regression testing.
"""
import time
from collections import defaultdict, deque


class TelemetryCollector:
    """Collects navigation performance metrics for observability."""

    def __init__(self, log_interval=50):
        self.log_interval = log_interval
        self.metrics = {
            # Counters
            'replan_count': 0,
            'recover_count': 0,
            'goal_reached_count': 0,
            'goal_unreachable_count': 0,
            'collision_near_count': 0,
            'total_frames': 0,
            # Timing
            'astar_total_time': 0.0,
            'astar_call_count': 0,
            'dwa_total_time': 0.0,
            'dwa_call_count': 0,
            # State distribution
            'state_time': defaultdict(float),
            # Recover reasons
            'recover_reasons': defaultdict(int),
        }
        # Time series for plotting
        self.coverage_history = deque(maxlen=10000)
        self.frontier_count_history = deque(maxlen=10000)
        self.dwa_valid_ratio_history = deque(maxlen=10000)

        self._last_log_time = time.time()
        self._frame_start_time = time.time()

    def frame_start(self):
        """Mark the start of a frame for timing."""
        self._frame_start_time = time.time()

    def frame_end(self, state_name, coverage, frontier_count):
        """Record end-of-frame metrics."""
        self.metrics['total_frames'] += 1
        frame_time = time.time() - self._frame_start_time
        self.metrics['state_time'][state_name] += frame_time

        self.coverage_history.append(coverage)
        self.frontier_count_history.append(frontier_count)

        # Periodic logging
        if self.metrics['total_frames'] % self.log_interval == 0:
            self._log_summary()

    def record_astar(self, duration, success):
        """Record A* planner call."""
        self.metrics['astar_total_time'] += duration
        self.metrics['astar_call_count'] += 1
        if not success:
            self.metrics['replan_count'] += 1

    def record_dwa(self, duration, valid_ratio):
        """Record DWA planner call."""
        self.metrics['dwa_total_time'] += duration
        self.metrics['dwa_call_count'] += 1
        self.dwa_valid_ratio_history.append(valid_ratio)

    def record_recover(self, reason):
        """Record RECOVER state entry."""
        self.metrics['recover_count'] += 1
        self.metrics['recover_reasons'][reason] += 1

    def record_goal_reached(self):
        self.metrics['goal_reached_count'] += 1

    def record_goal_unreachable(self):
        self.metrics['goal_unreachable_count'] += 1

    def record_near_collision(self):
        self.metrics['collision_near_count'] += 1

    def _log_summary(self):
        """Print periodic metrics summary."""
        m = self.metrics
        frames = m['total_frames']
        if frames == 0:
            return
        astar_avg = (m['astar_total_time'] / m['astar_call_count'] * 1000
                     if m['astar_call_count'] > 0 else 0)
        dwa_avg = (m['dwa_total_time'] / m['dwa_call_count'] * 1000
                   if m['dwa_call_count'] > 0 else 0)
        cov = self.coverage_history[-1] if self.coverage_history else 0
        fc = self.frontier_count_history[-1] if self.frontier_count_history else 0
        print(f"[TELEMETRY] frames={frames} cov={cov:.1f}% frontiers={fc} "
              f"replan={m['replan_count']} recover={m['recover_count']} "
              f"reached={m['goal_reached_count']} unreachable={m['goal_unreachable_count']} "
              f"astar_avg={astar_avg:.1f}ms dwa_avg={dwa_avg:.1f}ms")

    def get_summary(self):
        """Get summary dict for session recording."""
        m = self.metrics
        return {
            'total_frames': m['total_frames'],
            'replan_count': m['replan_count'],
            'recover_count': m['recover_count'],
            'goal_reached_count': m['goal_reached_count'],
            'goal_unreachable_count': m['goal_unreachable_count'],
            'collision_near_count': m['collision_near_count'],
            'astar_avg_ms': (m['astar_total_time'] / m['astar_call_count'] * 1000
                             if m['astar_call_count'] > 0 else 0),
            'dwa_avg_ms': (m['dwa_total_time'] / m['dwa_call_count'] * 1000
                           if m['dwa_call_count'] > 0 else 0),
            'recover_reasons': dict(m['recover_reasons']),
            'state_time': dict(m['state_time']),
        }
