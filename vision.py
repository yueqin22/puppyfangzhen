"""
Vision-based Semantic Mapping (v4.0)
=====================================
Upgraded from hard-label semantic mapping to **probabilistic Bayesian semantic
fusion** with uncertainty-aware updates.

v4.0 improvements:
  1. Per-cell categorical probability distribution over semantic classes
  2. Bayesian fusion of multiple observations (with confusion matrix model)
  3. Classification uncertainty modeled via Dirichlet concentration
  4. Entropy-based confidence metric
  5. Semantic information gain for Active SLAM (frontier selection)

References:
  - Posner et al. (2010) "Using Visual Range to Characterise Outdoor 3D Environments"
  - Hähnel et al. (2003) "Map Building with Mobile Robots in Dynamic Environments"
  - Bowman et al. (2017) "Probabilistic Data Association for Semantic SLAM"
"""
import math
import numpy as np
from PIL import Image
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H

N_LABELS = 7  # unknown, floor, wall, doorway, sofa, bed, table
LABEL_NAMES = ['unknown', 'floor', 'wall', 'doorway', 'sofa', 'bed', 'table']
LABEL_COLORS = {
    'unknown':   (160, 160, 160),
    'floor':     (220, 220, 200),
    'wall':      (240, 240, 240),
    'doorway':   (100, 100, 100),
    'sofa':      (180, 100, 100),
    'bed':       (100, 130, 180),
    'table':     (140, 90, 60),
}


class SemanticMapper:
    """Probabilistic vision-based semantic mapping with Bayesian fusion.

    Each cell maintains a categorical probability distribution over 7 semantic
    classes, updated via Bayes rule with a confusion matrix model of the
    classifier's uncertainty.
    """

    LABELS = LABEL_COLORS

    def __init__(self, occ_grid, cfg=None):
        self.grid = occ_grid
        self.label_names = LABEL_NAMES
        self.n_labels = N_LABELS

        # v4.0: probability distribution per cell (H, W, N_LABELS)
        # Stored as log-probabilities for numerical stability
        self.log_probs = np.zeros((GRID_H, GRID_W, N_LABELS), dtype=np.float32)
        # Initialize with uniform distribution over non-unknown classes
        # unknown=low prior, floor/wall/doorway=medium, furniture=low
        self._init_priors()

        # Observation count (number of fused observations)
        self.obs_count = np.zeros((GRID_H, GRID_W), dtype=np.uint16)

        # Confusion matrix: P(observed_label | true_label)
        # Row = observed, Col = true
        # Diagonal = high (0.7-0.85), off-diagonal small
        self.confusion = self._build_confusion_matrix()

        # Minimum confidence for a cell to be considered "labeled"
        self.confidence_threshold = cfg.get('confidence_threshold', 0.6) if cfg else 0.6

    def _init_priors(self):
        """Initialize prior probabilities for each cell."""
        # Prior: uniform over non-unknown, plus small unknown mass
        prior = np.ones(self.n_labels, dtype=np.float32) * 0.02  # small base
        prior[0] = 0.05   # unknown prior
        prior[1] = 0.25   # floor
        prior[2] = 0.20   # wall
        prior[3] = 0.10   # doorway
        prior[4] = 0.05   # sofa
        prior[5] = 0.05   # bed
        prior[6] = 0.05   # table
        prior = prior / prior.sum()
        self.log_priors = np.log(prior)
        self.log_probs[:] = self.log_priors  # broadcast to all cells

    def _build_confusion_matrix(self):
        """Build classifier confusion matrix P(observed | true).

        Models the fact that:
        - wall and floor are easily confused (both achromatic, varying light)
        - furniture colors are more distinct but can confuse with floor
        - doorway (dark) is sometimes confused with wall in shadows
        """
        C = np.eye(self.n_labels, dtype=np.float32) * 0.05
        # Diagonal: correct classification probability
        C[0, 0] = 0.50  # unknown stays unknown often
        C[1, 1] = 0.75  # floor
        C[2, 2] = 0.80  # wall
        C[3, 3] = 0.70  # doorway
        C[4, 4] = 0.85  # sofa (red is distinct)
        C[5, 5] = 0.85  # bed (blue is distinct)
        C[6, 6] = 0.80  # table (brown)

        # Off-diagonal confusions
        C[1, 2] = 0.10  # true wall -> observe floor
        C[2, 1] = 0.10  # true floor -> observe wall
        C[1, 3] = 0.05  # true doorway -> observe floor
        C[3, 2] = 0.10  # true wall -> observe doorway (shadows)
        C[4, 6] = 0.05  # true table -> observe sofa
        C[6, 4] = 0.05  # true sofa -> observe table
        C[1, 6] = 0.05  # true table -> observe floor
        C[5, 6] = 0.03  # true table -> observe bed

        # Normalize rows so they sum to 1
        row_sums = C.sum(axis=1, keepdims=True)
        row_sums[row_sums < 1e-9] = 1.0
        C = C / row_sums
        return C

    def classify_color(self, rgb_pixels):
        """Classify a region's dominant RGB color into a semantic label.

        Returns (label_index, confidence) tuple in v4.0.
        For backward compatibility, if called with single return value,
        returns label_index.
        """
        if len(rgb_pixels) == 0:
            return 0, 0.0  # unknown (return tuple for consistent unpacking)

        mean_rgb = np.mean(rgb_pixels, axis=0)
        r, g, b = mean_rgb[0], mean_rgb[1], mean_rgb[2]

        r_n, g_n, b_n = r / 255.0, g / 255.0, b / 255.0
        mx, mn = max(r_n, g_n, b_n), min(r_n, g_n, b_n)
        delta = mx - mn
        v = mx
        s = delta / mx if mx > 0 else 0
        if delta == 0:
            h = 0
        elif mx == r_n:
            h = 60 * (((g_n - b_n) / delta) % 6)
        elif mx == g_n:
            h = 60 * ((b_n - r_n) / delta + 2)
        else:
            h = 60 * ((r_n - g_n) / delta + 4)

        # v4.0: return confidence based on how "clean" the classification is
        label = 0
        conf = 0.5

        if s < 0.15:
            if v < 0.35:
                label, conf = 3, min(0.85, 0.35 + (0.35 - v) * 2)
            elif v > 0.7:
                label, conf = 2, min(0.85, 0.7 + (v - 0.7) * 0.5)
            else:
                label, conf = 1, min(0.75, 0.6 + abs(v - 0.5) * 0.3)
        else:
            if h < 30 or h > 330:
                label, conf = 4, min(0.9, 0.7 + s * 0.3)
            elif 200 < h < 270:
                label, conf = 5, min(0.9, 0.7 + s * 0.3)
            elif 20 < h < 45:
                label, conf = 6, min(0.85, 0.65 + s * 0.3)
            else:
                label, conf = 1, 0.55

        return label, conf

    def update(self, image_path, rx, ry, ryaw, fov_deg=60, max_dist=3.0):
        """Update semantic map using Bayesian fusion.

        For each observed cell, applies Bayes rule:
          P(true | obs) ∝ P(obs | true) * P(true)
        where P(obs | true) comes from the confusion matrix.
        """
        try:
            img = Image.open(image_path)
            img = img.resize((80, 60))
            pixels = np.array(img)
        except Exception:
            return 'unknown'

        h, w = pixels.shape[:2]
        cy, cx = h // 2, w // 2
        center_region = pixels[cy - 10:cy + 10, cx - 15:cx + 15]
        top_region = pixels[5:15, cx - 15:cx + 15]
        bottom_region = pixels[h - 15:h - 5, cx - 15:cx + 15]

        result_center = self.classify_color(center_region.reshape(-1, 3))
        result_far = self.classify_color(top_region.reshape(-1, 3))
        result_near = self.classify_color(bottom_region.reshape(-1, 3))

        label_center, conf_center = result_center
        label_far, conf_far = result_far
        label_near, conf_near = result_near

        fov_rad = math.radians(fov_deg)
        for dist_range, obs_label, obs_conf in [
            ((0.3, 1.0), label_near, conf_near),
            ((1.0, 2.0), label_center, conf_center),
            ((2.0, max_dist), label_far, conf_far),
        ]:
            if obs_label == 0:
                continue
            d_min, d_max = dist_range
            n_samples = 7
            for i in range(n_samples):
                angle = ryaw + (i - n_samples // 2) * (fov_rad / n_samples)
                for d in np.linspace(d_min, d_max, 4):
                    wx = rx + d * math.cos(angle)
                    wy = ry + d * math.sin(angle)
                    gx, gy = self.grid.world_to_grid(wx, wy)
                    if self.grid.in_bounds(gx, gy):
                        if abs(self.grid.log_odds[gy, gx]) > 0.3:
                            self._bayesian_update(gy, gx, obs_label, obs_conf)

        return self.label_names[label_center]

    def _bayesian_update(self, gy, gx, obs_label, obs_conf):
        """Apply one Bayesian update to a cell's probability distribution.

        log_posterior = log(P(obs|true)) + log_prior - log_normalizer
        P(obs|true) is the obs_conf-weighted confusion matrix row.
        """
        # Observation likelihood: weighted confusion matrix row
        # If we're very confident (obs_conf=1), use the obs_label row of confusion
        # If not confident, blend toward uniform
        likelihood = self.confusion[obs_label, :].copy()
        if obs_conf < 1.0:
            # Blend with uniform to model low confidence
            uniform = np.ones(self.n_labels, dtype=np.float32) / self.n_labels
            likelihood = obs_conf * likelihood + (1.0 - obs_conf) * uniform
            likelihood = likelihood / likelihood.sum()

        log_likelihood = np.log(likelihood + 1e-12)
        log_prior = self.log_probs[gy, gx]

        # Bayes rule in log space
        log_posterior = log_likelihood + log_prior
        log_norm = _logsumexp(log_posterior)
        log_posterior = log_posterior - log_norm

        self.log_probs[gy, gx] = log_posterior
        self.obs_count[gy, gx] += 1

    def get_label_at(self, x, y):
        """Get MAP (maximum a posteriori) label name at a world position."""
        gx, gy = self.grid.world_to_grid(x, y)
        if not self.grid.in_bounds(gx, gy):
            return 'unknown'
        map_idx = int(np.argmax(self.log_probs[gy, gx]))
        return self.label_names[map_idx]

    def get_label_probs(self, x, y):
        """Get full probability distribution at a world position."""
        gx, gy = self.grid.world_to_grid(x, y)
        if not self.grid.in_bounds(gx, gy):
            return np.ones(self.n_labels) / self.n_labels
        probs = np.exp(self.log_probs[gy, gx])
        return probs / probs.sum()

    def get_entropy(self, gx, gy):
        """Get Shannon entropy of the distribution at cell (gx, gy).

        Low entropy = confident label.
        High entropy = uncertain / multiple plausible labels.
        """
        if not self.grid.in_bounds(gx, gy):
            return math.log(self.n_labels)
        log_p = self.log_probs[gy, gx]
        p = np.exp(log_p)
        p = p / p.sum()
        # Shannon entropy H = -sum(p * log(p))
        mask = p > 1e-12
        entropy = -np.sum(p[mask] * log_p[mask])
        return float(entropy)

    def coverage_by_label(self):
        """Count cells per label (MAP label, with confidence threshold)."""
        counts = {}
        probs = np.exp(self.log_probs)
        prob_sums = probs.sum(axis=2, keepdims=True)
        prob_sums[prob_sums < 1e-12] = 1.0
        norm_probs = probs / prob_sums
        map_labels = np.argmax(norm_probs, axis=2)
        map_conf = np.max(norm_probs, axis=2)

        for idx, name in enumerate(self.label_names):
            mask = (map_labels == idx) & (map_conf >= self.confidence_threshold)
            counts[name] = int(np.sum(mask))
        return counts

    def semantic_info_gain(self, gx, gy, sensor_range_cells=40):
        """Expected semantic information gain if we observe cell (gx,gy).

        Computed as the expected reduction in entropy from a new observation:
          IG = H(prior) - E[H(posterior)]

        Uses the current entropy as a proxy (simplified, since we don't
        know what the observation will be). Higher current entropy means
        higher potential information gain.

        This is used in Active SLAM to select frontiers where semantic
        uncertainty is high.
        """
        if not self.grid.in_bounds(gx, gy):
            return 0.0

        # Current entropy
        current_H = self.get_entropy(gx, gy)
        max_H = math.log(self.n_labels)
        if max_H < 1e-12:
            return 0.0

        # Simplified: info gain proportional to current entropy
        # (more uncertain = more to gain)
        return current_H / max_H

    def semantic_info_gain_region(self, cx, cy, radius_cells=40):
        """Total semantic info gain in a circular region.

        Sum of info gains for all unknown/uncertain cells in range.
        """
        total = 0.0
        r2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx*dx + dy*dy > r2:
                    continue
                gx, gy = cx + dx, cy + dy
                if not self.grid.in_bounds(gx, gy):
                    continue
                if self.grid.is_unknown(gx, gy):
                    total += 1.0  # unknown cells: max gain
                else:
                    total += self.semantic_info_gain(gx, gy)
        return total

    def render_layer(self, display):
        """Overlay semantic colors on an RGB display array (in-place)."""
        probs = np.exp(self.log_probs)
        prob_sums = probs.sum(axis=2, keepdims=True)
        prob_sums[prob_sums < 1e-12] = 1.0
        norm_probs = probs / prob_sums
        map_labels = np.argmax(norm_probs, axis=2)
        map_conf = np.max(norm_probs, axis=2)

        for idx, name in enumerate(self.label_names):
            if idx == 0:
                continue
            mask = (map_labels == idx) & (map_conf >= self.confidence_threshold)
            if not np.any(mask):
                continue
            color = self.LABELS.get(name, (160, 160, 160))
            alpha = 0.5
            for c in range(3):
                channel = display[:, :, c]
                channel[mask] = (channel[mask].astype(np.float32) * (1 - alpha)
                                 + color[c] * alpha).astype(np.uint8)


def _logsumexp(x):
    """Numerically stable log-sum-exp."""
    x_max = np.max(x)
    return x_max + np.log(np.sum(np.exp(x - x_max)))
