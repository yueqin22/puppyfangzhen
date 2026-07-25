#!/usr/bin/env python3
"""
Navigation Evaluator
====================
Quantitative evaluation framework for autonomous navigation.
Records time-series metrics during a run, computes summary statistics,
and generates comparison plots across multiple runs (e.g. proposed method
vs random walk vs frontier-only baseline).

Metrics:
  - Coverage (%) over time — exploration efficiency
  - Cumulative path length (m)
  - RECOVER trigger count — robustness
  - Doorway crossings — exploration completeness
  - Localization error (m) — AMCL accuracy
  - State distribution — navigation stability

Usage:
  ev = Evaluator("my_method")
  ev.update(frame, t, rx, ry, ryaw, state, coverage, ...)
  ev.save_csv()
  Evaluator.compare_and_plot(["my_method", "random"], "/tmp/eval")
"""
import os
import csv
import math
import json
import numpy as np


class Evaluator:
    """Records metrics and computes statistics for one navigation run."""

    STATE_NAMES = ['PLAN', 'FOLLOW', 'RECOVER', 'DONE']

    def __init__(self, method_name="proposed"):
        self.method = method_name
        self.frames = []        # frame number
        self.times = []         # simulation time (s)
        self.xs = []            # robot x
        self.ys = []            # robot y
        self.yaws = []          # robot yaw
        self.states = []        # state int
        self.coverages = []     # coverage %
        self.costs = []         # cost at robot
        self.loc_errors = []    # AMCL position error (m), -1 if no AMCL
        self.loc_confs = []     # AMCL confidence [0,1]
        self.dwa_vs = []
        self.dwa_ws = []
        self.no_progress = []
        # Events
        self.recover_events = 0      # count of RECOVER entries
        self.doorway_crossings = 0   # count of y=0 crossings
        self.goal_reached = 0        # count of goals reached
        self.goal_failed = 0         # count of goals given up
        # Path
        self.cumulative_distance = 0.0
        self.cum_dists = []    # per-frame cumulative distance (exact)
        self.last_xy = None
        # Final stats
        self.final_coverage = 0.0
        self.final_time = 0.0
        self.done_reached = False

    def update(self, frame, t, x, y, yaw, state, coverage, cost,
               loc_error=-1.0, loc_conf=1.0,
               dwa_v=0.0, dwa_w=0.0, no_progress=0,
               true_x=None, true_y=None):
        """Record one frame of data."""
        self.frames.append(frame)
        self.times.append(t)
        self.xs.append(x)
        self.ys.append(y)
        self.yaws.append(yaw)
        self.states.append(state)
        self.coverages.append(coverage)
        self.costs.append(cost)
        self.loc_errors.append(loc_error)
        self.loc_confs.append(loc_conf)
        self.dwa_vs.append(dwa_v)
        self.dwa_ws.append(dwa_w)
        self.no_progress.append(no_progress)

        # Cumulative path length
        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            self.cumulative_distance += math.sqrt(dx * dx + dy * dy)
        self.last_xy = (x, y)
        self.cum_dists.append(self.cumulative_distance)

        # State transition counts
        if state == 2 and len(self.states) >= 2 and self.states[-2] != 2:
            self.recover_events += 1
        if state == 3:
            self.done_reached = True

        # Localization error (if ground truth provided)
        if true_x is not None and true_y is not None:
            err = math.sqrt((x - true_x) ** 2 + (y - true_y) ** 2)
            # Override the loc_errors entry just appended
            self.loc_errors[-1] = err

        self.final_coverage = coverage
        self.final_time = t

    def save_csv(self, filepath):
        """Save raw time-series to CSV."""
        with open(filepath, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['frame', 'time', 'x', 'y', 'yaw', 'state',
                        'coverage', 'cost', 'loc_error', 'loc_conf',
                        'dwa_v', 'dwa_w', 'no_progress',
                        'cum_dist', 'method'])
            for i in range(len(self.frames)):
                w.writerow([
                    self.frames[i], f"{self.times[i]:.1f}",
                    f"{self.xs[i]:.3f}", f"{self.ys[i]:.3f}",
                    f"{self.yaws[i]:.3f}", self.states[i],
                    f"{self.coverages[i]:.2f}", self.costs[i],
                    f"{self.loc_errors[i]:.4f}", f"{self.loc_confs[i]:.3f}",
                    f"{self.dwa_vs[i]:.3f}", f"{self.dwa_ws[i]:.3f}",
                    self.no_progress[i],
                    # Cumulative distance at this frame
                    f"{self._cum_dist_at(i):.3f}",
                    self.method,
                ])

    def _cum_dist_at(self, idx):
        """Exact cumulative distance up to frame idx."""
        if 0 <= idx < len(self.cum_dists):
            return self.cum_dists[idx]
        elif self.cum_dists:
            return self.cum_dists[-1]
        return 0.0

    def summary(self):
        """Return summary statistics dict."""
        states = np.array(self.states)
        state_counts = {i: int(np.sum(states == i)) for i in range(4)}
        total = len(states) if len(states) > 0 else 1

        # Time to reach 50%, 80%, 90% coverage
        def time_to_coverage(target):
            for i, c in enumerate(self.coverages):
                if c >= target:
                    return self.times[i]
            return -1.0  # never reached

        # Mean localization error (only where AMCL was used)
        loc_errs = [e for e in self.loc_errors if e >= 0]
        mean_loc_err = float(np.mean(loc_errs)) if loc_errs else -1.0
        max_loc_err = float(np.max(loc_errs)) if loc_errs else -1.0

        return {
            'method': self.method,
            'total_frames': len(self.frames),
            'total_time_s': self.final_time,
            'final_coverage_pct': round(self.final_coverage, 2),
            'cumulative_distance_m': round(self.cumulative_distance, 2),
            'recover_events': self.recover_events,
            'doorway_crossings': self.doorway_crossings,
            'goal_reached': self.goal_reached,
            'goal_failed': self.goal_failed,
            'done_reached': self.done_reached,
            'state_distribution': {
                self.STATE_NAMES[k]: state_counts[k] for k in range(4)
            },
            'recover_pct': round(state_counts[2] * 100.0 / total, 1),
            'follow_pct': round(state_counts[1] * 100.0 / total, 1),
            'time_to_50pct': time_to_coverage(50),
            'time_to_80pct': time_to_coverage(80),
            'time_to_90pct': time_to_coverage(90),
            'mean_loc_error_m': round(mean_loc_err, 4),
            'max_loc_error_m': round(max_loc_err, 4),
            'exploration_efficiency': round(
                self.final_coverage / max(self.cumulative_distance, 0.01), 3),
        }

    def save_summary(self, filepath):
        """Save summary JSON."""
        with open(filepath, 'w') as f:
            json.dump(self.summary(), f, indent=2)

    @staticmethod
    def compare_and_plot(method_names, output_dir):
        """Load CSV results from multiple methods and plot comparison.

        Each method's CSV should be at <output_dir>/<method>_data.csv.
        Generates:
          - coverage_vs_time.png
          - loc_error_vs_time.png
          - state_distribution.png
          - comparison_table.txt
        """
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print("[EVAL] matplotlib not available, skipping plots")
            return

        os.makedirs(output_dir, exist_ok=True)
        summaries = []

        # Load each method's data
        all_data = {}
        for name in method_names:
            csv_path = os.path.join(output_dir, f"{name}_data.csv")
            if not os.path.exists(csv_path):
                print(f"[EVAL] Missing {csv_path}, skipping {name}")
                continue
            data = np.genfromtxt(csv_path, delimiter=',', names=True,
                                 dtype=None, encoding='utf-8')
            all_data[name] = data
            # Load summary
            json_path = os.path.join(output_dir, f"{name}_summary.json")
            if os.path.exists(json_path):
                with open(json_path) as f:
                    summaries.append(json.load(f))

        if not all_data:
            print("[EVAL] No data to plot")
            return

        # --- Plot 1: Coverage vs Time ---
        fig, ax = plt.subplots(figsize=(10, 6))
        for name, data in all_data.items():
            ax.plot(data['time'], data['coverage'], label=name, linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Coverage (%)')
        ax.set_title('Exploration Coverage Over Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 100)
        fig.savefig(os.path.join(output_dir, 'coverage_vs_time.png'), dpi=100)
        plt.close(fig)

        # --- Plot 2: Localization Error vs Time ---
        fig, ax = plt.subplots(figsize=(10, 6))
        for name, data in all_data.items():
            errs = data['loc_error']
            mask = errs >= 0
            if mask.any():
                ax.plot(data['time'][mask], errs[mask],
                        label=name, linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Localization Error (m)')
        ax.set_title('AMCL Localization Error vs Ground Truth')
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.savefig(os.path.join(output_dir, 'loc_error_vs_time.png'), dpi=100)
        plt.close(fig)

        # --- Plot 3: State Distribution (stacked bar) ---
        fig, ax = plt.subplots(figsize=(10, 6))
        x_pos = np.arange(len(all_data))
        bottom = np.zeros(len(all_data))
        colors = ['#4CAF50', '#2196F3', '#FF9800', '#9E9E9E']
        labels = ['PLAN', 'FOLLOW', 'RECOVER', 'DONE']
        for s in range(4):
            vals = []
            for name, data in all_data.items():
                states = data['state']
                total = len(states)
                pct = np.sum(states == s) * 100.0 / max(total, 1)
                vals.append(pct)
            ax.bar(x_pos, vals, bottom=bottom, label=labels[s], color=colors[s])
            bottom += vals
        ax.set_xticks(x_pos)
        ax.set_xticklabels(list(all_data.keys()))
        ax.set_ylabel('Time (%)')
        ax.set_title('State Machine Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        fig.savefig(os.path.join(output_dir, 'state_distribution.png'), dpi=100)
        plt.close(fig)

        # --- Plot 4: Trajectory ---
        fig, ax = plt.subplots(figsize=(10, 8))
        for name, data in all_data.items():
            ax.plot(data['x'], data['y'], label=name, linewidth=1.5, alpha=0.8)
            # Mark start
            ax.plot(data['x'][0], data['y'][0], 'go', markersize=10)
            # Mark end
            ax.plot(data['x'][-1], data['y'][-1], 'rx', markersize=10)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Robot Trajectories')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.axhline(0, color='gray', linestyle='--', alpha=0.5, label='doorway')
        fig.savefig(os.path.join(output_dir, 'trajectories.png'), dpi=100)
        plt.close(fig)

        # --- Comparison table ---
        table_path = os.path.join(output_dir, 'comparison_table.txt')
        with open(table_path, 'w') as f:
            f.write("=" * 80 + "\n")
            f.write("NAVIGATION METHODS COMPARISON TABLE\n")
            f.write("=" * 80 + "\n\n")
            headers = ['Method', 'Time(s)', 'Cov(%)', 'Dist(m)',
                       'RECOVER', 'Doorway', 'LocErr(m)', 'Efficiency']
            f.write(f"{headers[0]:<15} {headers[1]:<8} {headers[2]:<8} "
                    f"{headers[3]:<8} {headers[4]:<8} {headers[5]:<8} "
                    f"{headers[6]:<10} {headers[7]:<10}\n")
            f.write("-" * 80 + "\n")
            for s in summaries:
                f.write(f"{s['method']:<15} {s['total_time_s']:<8.1f} "
                        f"{s['final_coverage_pct']:<8.2f} "
                        f"{s['cumulative_distance_m']:<8.2f} "
                        f"{s['recover_events']:<8} "
                        f"{s['doorway_crossings']:<8} "
                        f"{s['mean_loc_error_m']:<10.4f} "
                        f"{s['exploration_efficiency']:<10.3f}\n")
            f.write("\n")
            # State distribution
            f.write("State Distribution (%):\n")
            f.write(f"{'Method':<15} {'PLAN':<8} {'FOLLOW':<8} "
                    f"{'RECOVER':<8} {'DONE':<8}\n")
            for s in summaries:
                sd = s['state_distribution']
                f.write(f"{s['method']:<15} "
                        f"{sd.get('PLAN',0):<8} {sd.get('FOLLOW',0):<8} "
                        f"{sd.get('RECOVER',0):<8} {sd.get('DONE',0):<8}\n")

        print(f"[EVAL] Plots and table saved to {output_dir}")
        return table_path


# ============================================================
# MAP QUALITY METRICS
# ============================================================
# These functions compare the robot-built occupancy grid against a
# ground-truth grid built from the known obstacle layout, providing
# objective measures of mapping accuracy:
#   - IoU (Intersection over Union): overlap quality of occupied cells
#   - Hausdorff distance: worst-case boundary error
#   - Map entropy: information-theoretic uncertainty of the grid
# ============================================================

def build_ground_truth_grid(obstacles, grid_w=100, grid_h=80,
                            resolution=0.1, origin_x=-5.0, origin_y=-4.0):
    """Build a ground-truth occupancy grid from the known obstacle list.

    Args:
        obstacles: list of (alias, handle, (xmin, ymin, xmax, ymax)) tuples
            (as returned by discover_obstacles_with_bbox in autonomous_nav.py)
        grid_w, grid_h, resolution, origin_x, origin_y: grid parameters
            (must match occupancy_grid.py constants)

    Returns:
        (H, W) uint8 array: 1 = occupied, 0 = free
    """
    gt = np.zeros((grid_h, grid_w), dtype=np.uint8)
    for alias, _, (xmin, ymin, xmax, ymax) in obstacles:
        # Convert world bbox to grid indices
        gx0 = max(0, int((xmin - origin_x) / resolution))
        gx1 = min(grid_w, int((xmax - origin_x) / resolution) + 1)
        gy0 = max(0, int((ymin - origin_y) / resolution))
        gy1 = min(grid_h, int((ymax - origin_y) / resolution) + 1)
        if gx0 < gx1 and gy0 < gy1:
            gt[gy0:gy1, gx0:gx1] = 1
    return gt


def compute_map_metrics(occ_grid, obstacles):
    """Compute map quality metrics: IoU, Hausdorff, entropy.

    Compares the robot-built occupancy grid (occ_grid.log_odds) against
    a ground-truth grid built from the known obstacle list.

    Args:
        occ_grid: OccupancyGrid instance built by the robot
        obstacles: list of (alias, handle, bbox) tuples

    Returns:
        dict with keys:
            - iou_occupied: IoU of occupied cells (0-1, higher=better)
            - iou_free: IoU of free cells (0-1, higher=better)
            - hausdorff_m: Hausdorff distance in meters (lower=better)
            - map_entropy: Shannon entropy of occupancy probabilities (bits)
                           (lower=more certain)
            - observed_pct: percentage of cells observed (coverage)
            - precision: of cells marked occupied, how many are truly occupied
            - recall: of truly occupied cells, how many were detected
    """
    from occupancy_grid import GRID_W, GRID_H, GRID_RESOLUTION, ORIGIN_X, ORIGIN_Y

    # Build ground truth
    gt = build_ground_truth_grid(obstacles, GRID_W, GRID_H,
                                 GRID_RESOLUTION, ORIGIN_X, ORIGIN_Y)

    # Robot's observed occupancy: occupied if log_odds > 0.6
    obs_occupied = (occ_grid.log_odds > 0.6).astype(np.uint8)
    obs_free = (occ_grid.log_odds < -0.5).astype(np.uint8)
    obs_unknown = (np.abs(occ_grid.log_odds) < 0.3).astype(np.uint8)

    # Ground truth free = not occupied AND in bounds
    gt_free = (1 - gt).astype(np.uint8)

    # === IoU ===
    # IoU of occupied cells: |obs ∩ gt| / |obs ∪ gt|
    # Only consider cells that have been observed (not unknown)
    observed_mask = 1 - obs_unknown  # cells the robot has seen
    obs_occ_observed = obs_occupied & observed_mask
    intersection_occ = np.sum(obs_occ_observed & gt)
    union_occ = np.sum((obs_occ_observed | (gt > 0)) & observed_mask)
    iou_occupied = float(intersection_occ) / max(union_occ, 1)

    # IoU of free cells: |obs_free ∩ gt_free| / |obs_free ∪ gt_free|
    obs_free_observed = obs_free & observed_mask
    intersection_free = np.sum(obs_free_observed & gt_free)
    union_free = np.sum((obs_free_observed | (gt == 0)) & observed_mask)
    iou_free = float(intersection_free) / max(union_free, 1)

    # === Precision / Recall (for occupied cells) ===
    true_positive = int(np.sum(obs_occ_observed & gt))
    false_positive = int(np.sum(obs_occ_observed & (1 - gt)))
    false_negative = int(np.sum((1 - obs_occ_observed) & gt & observed_mask))
    precision = float(true_positive) / max(true_positive + false_positive, 1)
    recall = float(true_positive) / max(true_positive + false_negative, 1)

    # === Hausdorff distance ===
    # Distance between the set of occupied cells in observed vs ground truth
    # We use a simplified version: for each observed occupied cell, find the
    # nearest ground-truth occupied cell, and vice versa. Take the max.
    obs_pts = np.argwhere(obs_occ_observed > 0)  # (N, 2) array of (y, x)
    gt_pts = np.argwhere((gt > 0) & observed_mask)  # (M, 2)
    if len(obs_pts) > 0 and len(gt_pts) > 0:
        # Compute pairwise min distances using a distance transform on gt
        # This is O(N+M) instead of O(N*M)
        from amcl import _distance_transform_numpy
        gt_mask = gt > 0
        dist_to_gt = _distance_transform_numpy(gt_mask)  # (H, W) in cells

        # For each observed occupied cell, distance to nearest gt cell
        d_obs_to_gt = dist_to_gt[obs_pts[:, 0], obs_pts[:, 1]]
        max_d_obs_to_gt = float(d_obs_to_gt.max()) if len(d_obs_to_gt) > 0 else 0.0

        # For each gt cell, distance to nearest observed occupied cell
        obs_mask = obs_occ_observed > 0
        dist_to_obs = _distance_transform_numpy(obs_mask)
        d_gt_to_obs = dist_to_obs[gt_pts[:, 0], gt_pts[:, 1]]
        max_d_gt_to_obs = float(d_gt_to_obs.max()) if len(d_gt_to_obs) > 0 else 0.0

        # Hausdorff = max of the two directional distances
        hausdorff_cells = max(max_d_obs_to_gt, max_d_gt_to_obs)
        hausdorff_m = hausdorff_cells * GRID_RESOLUTION
    else:
        hausdorff_m = float('inf')

    # === Map entropy ===
    # Shannon entropy of occupancy probabilities
    # H = -Σ p log(p) - (1-p) log(1-p)
    # Only for observed cells (unobserved cells are uniform 0.5 → max entropy)
    lo = occ_grid.log_odds
    # Convert log-odds to probability
    # p = 1 / (1 + exp(-lo))
    # Clamp to avoid numerical issues
    lo_clamped = np.clip(lo, -10, 10)
    p = 1.0 / (1.0 + np.exp(-lo_clamped))
    # Entropy per cell: H(p) = -p*log(p) - (1-p)*log(1-p)
    # Use safe log
    eps = 1e-12
    p_safe = np.clip(p, eps, 1 - eps)
    entropy_per_cell = -p_safe * np.log2(p_safe) - (1 - p_safe) * np.log2(1 - p_safe)
    # Mean entropy over all cells (lower = more certain map)
    map_entropy = float(entropy_per_cell.mean())

    # Observed percentage
    observed_pct = float(np.sum(observed_mask)) / (GRID_W * GRID_H) * 100.0

    return {
        'iou_occupied': round(iou_occupied, 4),
        'iou_free': round(iou_free, 4),
        'hausdorff_m': round(hausdorff_m, 4),
        'map_entropy': round(map_entropy, 4),
        'observed_pct': round(observed_pct, 2),
        'precision': round(precision, 4),
        'recall': round(recall, 4),
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
    }
