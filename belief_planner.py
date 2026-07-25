"""
Belief Space Planning (v4.0)
==============================
Plans in belief space (state + uncertainty) rather than deterministic state.

The belief state b = (x, y, theta, Sigma) includes both the mean pose and
its uncertainty (covariance). The planner considers how actions affect both
the expected state and the expected uncertainty, choosing actions that
optimize a combination of task progress and uncertainty reduction.

Implementation uses an approximate, computationally tractable approach:
  - Belief state = mean pose + 3x3 covariance
  - Prediction: linearized uncertainty propagation (EKF-style)
  - Observation: expected information gain from LiDAR scan
  - Planning: lookahead tree with belief states (simplified to sampling)

References:
  - Thrun et al. (2005) "Probabilistic Robotics" (Chapter 15: Partially
    Observable Markov Decision Processes)
  - Platt et al. (2010) "Belief Space Planning assuming Maximum
    Likelihood Observations"
  - van den Berg et al. (2012) "LQG-MP: Optimized Path Planning for
    Robots with Motion Uncertainty and Imperfect State Information"
"""
import os
import math
import numpy as np

USE_BELIEF_PLANNING = os.environ.get("USE_BELIEF_PLANNING", "0") == "1"


class BeliefState:
    """Belief state: mean pose + covariance."""

    __slots__ = ['x', 'y', 'theta', 'cov']

    def __init__(self, x, y, theta, cov=None):
        self.x = x
        self.y = y
        self.theta = theta
        if cov is None:
            self.cov = np.eye(3) * 0.01
        else:
            self.cov = np.array(cov)

    def entropy(self):
        """Shannon entropy of the Gaussian belief (proportional to log det)."""
        # H = 0.5 * log((2*pi*e)^n * det(Sigma))
        det = np.linalg.det(self.cov)
        if det <= 0:
            return float('inf')
        return 0.5 * math.log((2 * math.pi * math.e) ** 3 * det)

    def uncertainty_trace(self):
        """Trace of covariance (sum of variances)."""
        return float(np.trace(self.cov))

    def copy(self):
        return BeliefState(self.x, self.y, self.theta, self.cov.copy())


class BeliefPlanner:
    """Belief space planner for exploration and navigation.

    Evaluates candidate actions by simulating their effect on both the
    expected state and the expected uncertainty.

    The utility function balances:
      U = w_progress * task_progress
        - w_uncertainty * expected_uncertainty
        + w_info_gain * expected_info_gain
    """

    def __init__(self, occ_grid, costmap, semantic_mapper=None, cfg=None):
        self.occ_grid = occ_grid
        self.costmap = costmap
        self.semantic_mapper = semantic_mapper
        cfg = cfg or {}

        # Weights
        self.w_progress = cfg.get('w_progress', 0.4)
        self.w_uncertainty = cfg.get('w_uncertainty', 0.3)
        self.w_info_gain = cfg.get('w_info_gain', 0.3)

        # Motion model uncertainty
        self.motion_noise_v = cfg.get('motion_noise_v', 0.05)  # per m traveled
        self.motion_noise_w = cfg.get('motion_noise_w', 0.1)  # per rad turned

        # Observation model
        self.sensor_range = cfg.get('sensor_range_m', 8.0)
        self.sensor_range_cells = int(self.sensor_range / self.occ_grid.resolution)

        # Planning parameters
        self.n_candidates = cfg.get('n_belief_candidates', 8)
        self.lookahead_dist = cfg.get('belief_lookahead_m', 2.0)

    def predict_belief(self, belief, v, w, dt):
        """Predict belief state after applying (v, w) for dt seconds.

        Uses linearized uncertainty propagation (EKF prediction step).
        """
        new_b = belief.copy()

        # Mean update (same as standard motion model)
        if abs(w) > 1e-6:
            R = v / w
            new_b.x += -R * math.sin(belief.theta) + R * math.sin(belief.theta + w * dt)
            new_b.y += R * math.cos(belief.theta) - R * math.cos(belief.theta + w * dt)
            new_b.theta += w * dt
        else:
            new_b.x += v * dt * math.cos(belief.theta)
            new_b.y += v * dt * math.sin(belief.theta)

        # Normalize theta
        while new_b.theta > math.pi:
            new_b.theta -= 2 * math.pi
        while new_b.theta < -math.pi:
            new_b.theta += 2 * math.pi

        # Covariance prediction (linearized Jacobian)
        # F = df/dx (3x3 Jacobian of motion model w.r.t. state)
        theta = belief.theta
        F = np.eye(3)
        if abs(w) > 1e-6:
            R = v / w
            F[0, 2] = -R * math.cos(theta) + R * math.cos(theta + w * dt)
            F[1, 2] = -R * math.sin(theta) + R * math.sin(theta + w * dt)
        else:
            F[0, 2] = -v * dt * math.sin(theta)
            F[1, 2] = v * dt * math.cos(theta)

        # Process noise (from control uncertainty)
        dist = v * dt
        ang = w * dt
        Q = np.diag([
            (self.motion_noise_v * abs(dist))**2 + 0.001,
            (self.motion_noise_v * abs(dist))**2 + 0.001,
            (self.motion_noise_w * abs(ang) + 0.01)**2
        ])

        new_b.cov = F @ belief.cov @ F.T + Q
        return new_b

    def expected_info_gain(self, belief):
        """Expected information gain from a LiDAR scan at this belief state.

        Approximated by:
          IG ≈ H(prior) - E[H(posterior)]
        where the expected posterior covariance depends on how many
        distinct geometric features are visible from this position.

        We use the number of occupied cells in view as a proxy for the
        information content of the observation.
        """
        gx, gy = self.occ_grid.world_to_grid(belief.x, belief.y)
        r = self.sensor_range_cells

        # Count occupied and unknown cells in view
        n_occ = 0
        n_unknown = 0
        total = 0

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx*dx + dy*dy > r*r:
                    continue
                cx, cy = gx + dx, gy + dy
                if not (0 <= cx < self.occ_grid.width and 0 <= cy < self.occ_grid.height):
                    continue
                total += 1
                if self.occ_grid.is_occupied(cx, cy):
                    n_occ += 1
                elif self.occ_grid.is_unknown(cx, cy):
                    n_unknown += 1

        # More occupied cells = more features to localize against = more IG
        # More unknown cells = more mapping info gain
        feature_density = n_occ / max(total, 1)
        unknown_density = n_unknown / max(total, 1)

        # Expected info gain (normalized)
        geo_ig = 0.6 * feature_density + 0.4 * unknown_density

        # Semantic info gain
        sem_ig = 0.0
        if self.semantic_mapper:
            sem_gain = self.semantic_mapper.semantic_info_gain_region(
                gx, gy, radius_cells=r // 2)
            sem_ig = sem_gain / (math.pi * (r//2)**2)

        total_ig = 0.7 * geo_ig + 0.3 * sem_ig
        return total_ig

    def evaluate_candidates(self, belief, goal_x, goal_y, frontiers=None):
        """Evaluate candidate actions and select the best one.

        Args:
            belief: current BeliefState
            goal_x, goal_y: target goal
            frontiers: optional list of frontier candidates

        Returns:
            list of (dx, dy, utility, predicted_belief) sorted by utility
        """
        candidates = []

        if frontiers:
            # Evaluate each frontier as a candidate target
            for f in frontiers[:self.n_candidates]:
                fx, fy = f[0], f[1]
                dx = fx - belief.x
                dy = fy - belief.y
                dist = math.sqrt(dx**2 + dy**2)
                if dist < 0.1:
                    continue

                # Predict what the belief would be at the frontier
                # (simplified: assume we move straight there)
                pred_belief = self._predict_along_path(belief, fx, fy, dist)

                # Expected info gain at the frontier
                ig = self.expected_info_gain(pred_belief)

                # Progress toward goal
                dist_to_goal = math.sqrt((goal_x - fx)**2 + (goal_y - fy)**2)
                start_dist = math.sqrt((goal_x - belief.x)**2 + (goal_y - belief.y)**2)
                progress = max(0, (start_dist - dist_to_goal) / max(start_dist, 0.1))

                # Uncertainty cost
                unc = pred_belief.uncertainty_trace()
                unc_norm = min(1.0, unc / 1.0)

                utility = (self.w_progress * progress
                           - self.w_uncertainty * unc_norm
                           + self.w_info_gain * ig)

                candidates.append((fx, fy, utility, pred_belief))

        if not candidates:
            # Generate heading-based candidates
            for i in range(self.n_candidates):
                angle = -math.pi + 2 * math.pi * i / self.n_candidates
                target_x = belief.x + self.lookahead_dist * math.cos(angle)
                target_y = belief.y + self.lookahead_dist * math.sin(angle)

                if not self.costmap.is_safe(target_x, target_y):
                    continue

                pred_belief = self._predict_along_path(
                    belief, target_x, target_y, self.lookahead_dist)

                ig = self.expected_info_gain(pred_belief)

                dist_to_goal = math.sqrt(
                    (goal_x - target_x)**2 + (goal_y - target_y)**2)
                start_dist = math.sqrt(
                    (goal_x - belief.x)**2 + (goal_y - belief.y)**2)
                progress = max(0, (start_dist - dist_to_goal) / max(start_dist, 0.1))

                unc = pred_belief.uncertainty_trace()
                unc_norm = min(1.0, unc / 1.0)

                utility = (self.w_progress * progress
                           - self.w_uncertainty * unc_norm
                           + self.w_info_gain * ig)

                candidates.append((target_x, target_y, utility, pred_belief))

        candidates.sort(key=lambda c: -c[2])
        return candidates

    def _predict_along_path(self, belief, target_x, target_y, distance):
        """Simplified belief prediction along a straight path to target.

        Uses a single-step prediction with the total distance traveled.
        """
        if distance < 0.01:
            return belief.copy()

        angle = math.atan2(target_y - belief.y, target_x - belief.x)
        heading_diff = angle - belief.theta
        while heading_diff > math.pi:
            heading_diff -= 2 * math.pi
        while heading_diff < -math.pi:
            heading_diff += 2 * math.pi

        # Simulate: turn + move + turn
        v = 0.2  # assumed speed
        w = 0.5  # assumed turn rate

        # Turn to face target
        dt_turn = abs(heading_diff) / w if w > 0 else 0
        b1 = self.predict_belief(belief, 0.0,
                                  np.sign(heading_diff) * w if abs(heading_diff) > 0.01 else 0.0,
                                  dt_turn)

        # Move straight
        dt_move = distance / v if v > 0 else 0
        b2 = self.predict_belief(b1, v, 0.0, dt_move)

        return b2

    def should_recover(self, belief):
        """Determine if belief uncertainty is too high and recovery is needed."""
        unc = belief.uncertainty_trace()
        return unc > 1.0  # threshold: if total variance > 1 m^2

    def get_recovery_actions(self, belief):
        """Suggest recovery actions to reduce uncertainty.

        Returns list of candidate actions that maximize information gain,
        e.g., rotating in place to get more scan data, or moving toward
        areas with more landmarks.
        """
        candidates = []
        # Rotate in place
        for i in range(8):
            angle = belief.theta + 2 * math.pi * i / 8
            # Expected IG from rotating to this angle
            # (simplified: same position, different heading)
            rot_b = belief.copy()
            rot_b.theta = angle
            ig = self.expected_info_gain(rot_b)
            candidates.append((belief.x, belief.y, ig, rot_b))

        candidates.sort(key=lambda c: -c[2])
        return candidates
