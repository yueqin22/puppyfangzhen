"""
Semantic SLAM - Object-Level Landmarks (v4.0)
================================================
Enhances AMCL localization with semantic object landmarks.

In addition to LiDAR scan matching, the robot can use detected objects
(sofa, bed, table, doorway) as semantic landmarks to constrain its pose
estimate. Each object has a known class and approximate position, providing
additional constraints that improve localization accuracy, especially in
geometrically ambiguous environments.

The semantic observation model:
  P(z_semantic | x, m) = product over detected objects of P(object_i | x, m)

Where each object observation likelihood is based on:
  1. Expected object position from the semantic map
  2. Observed object position from camera
  3. Class consistency (should observe the right type of object)

References:
  - Bowman et al. (2017) "Probabilistic Data Association for Semantic SLAM"
  - Cadena et al. (2016) "Past, Present, and Future of Simultaneous
    Localization and Mapping: Towards the Robust-Perception Age"
  - Doherty et al. (2019) "Dual Quaternion Based Visual-Inertial-Semantic
    SLAM"
"""
import os
import math
import numpy as np
from vision import N_LABELS, LABEL_NAMES

USE_SEMANTIC_SLAM = os.environ.get("USE_SEMANTIC_SLAM", "0") == "1"


class SemanticLandmark:
    """A semantic landmark in the map (object with known class + position)."""

    def __init__(self, lid, label_idx, x, y, confidence=0.5, size=0.5):
        self.id = lid
        self.label_idx = label_idx
        self.x = x
        self.y = y
        self.confidence = confidence
        self.size = size  # approximate size (radius)
        self.observations = 1  # how many times observed

    @property
    def label_name(self):
        return LABEL_NAMES[self.label_idx] if self.label_idx < len(LABEL_NAMES) else 'unknown'


class SemanticSLAM:
    """Semantic SLAM: object-level landmark mapping and localization.

    Maintains a landmark map and provides semantic observation likelihood
    for AMCL particles.
    """

    def __init__(self, occ_grid, cfg=None):
        self.occ_grid = occ_grid
        cfg = cfg or {}

        self.landmarks = []  # list of SemanticLandmark
        self.next_id = 0

        # Observation model parameters
        self.sigma_pos = cfg.get('semantic_sigma_pos', 0.5)  # position uncertainty
        self.sigma_class = cfg.get('semantic_sigma_class', 0.3)  # class confusion
        self.association_dist = cfg.get('association_distance', 1.5)
        self.confidence_threshold = cfg.get('confidence_threshold', 0.4)
        self.min_observations = cfg.get('min_observations', 2)

        # Data association: max distance for matching observed object to landmark
        self.max_assoc_dist = cfg.get('max_association_dist', 1.5)

    def add_landmark(self, label_idx, x, y, confidence=0.5, size=0.5):
        """Add a new semantic landmark."""
        lm = SemanticLandmark(self.next_id, label_idx, x, y, confidence, size)
        self.next_id += 1
        self.landmarks.append(lm)
        return lm

    def observe_objects(self, objects, robot_x, robot_y, robot_yaw,
                         camera_fov_deg=60, max_range=5.0):
        """Process a set of observed objects from the camera.

        Args:
            objects: list of (label_idx, rel_angle, rel_dist, conf) tuples
                     relative to robot's current pose
            robot_x, robot_y, robot_yaw: current robot pose
            camera_fov_deg: camera field of view
            max_range: max detection range

        Returns:
            list of (landmark_id, label_idx, world_x, world_y) matched landmarks
        """
        matches = []

        for (label_idx, rel_angle, rel_dist, conf) in objects:
            if conf < self.confidence_threshold:
                continue

            # Convert to world coordinates
            world_angle = robot_yaw + rel_angle
            world_x = robot_x + rel_dist * math.cos(world_angle)
            world_y = robot_y + rel_dist * math.sin(world_angle)

            # Find nearest landmark of same class
            best_lm = None
            best_dist = float('inf')

            for lm in self.landmarks:
                if lm.label_idx != label_idx:
                    continue
                d = math.sqrt((lm.x - world_x)**2 + (lm.y - world_y)**2)
                if d < best_dist and d < self.max_assoc_dist:
                    best_dist = d
                    best_lm = lm

            if best_lm is not None:
                # Update existing landmark (running average)
                best_lm.observations += 1
                alpha = 1.0 / best_lm.observations
                best_lm.x = best_lm.x + alpha * (world_x - best_lm.x)
                best_lm.y = best_lm.y + alpha * (world_y - best_lm.y)
                best_lm.confidence = min(0.95, best_lm.confidence + 0.05)
                matches.append((best_lm.id, label_idx, best_lm.x, best_lm.y))
            else:
                # Create new landmark
                new_lm = self.add_landmark(label_idx, world_x, world_y, conf)
                matches.append((new_lm.id, label_idx, world_x, world_y))

        return matches

    def compute_log_likelihood(self, particle_x, particle_y, particle_yaw,
                               observed_objects):
        """Compute log-likelihood of semantic observations given a particle pose.

        For each observed object, find the most likely corresponding landmark
        and compute the likelihood based on position and class agreement.

        Args:
            particle_x, particle_y, particle_yaw: particle pose
            observed_objects: list of (label_idx, rel_angle, rel_dist, conf)

        Returns:
            log-likelihood scalar
        """
        if not observed_objects or not self.landmarks:
            return 0.0

        log_lik = 0.0
        sigma_pos_sq = self.sigma_pos ** 2

        for (label_idx, rel_angle, rel_dist, conf) in observed_objects:
            if conf < self.confidence_threshold:
                continue

            # Predicted world position from particle's perspective
            pred_angle = particle_yaw + rel_angle
            pred_x = particle_x + rel_dist * math.cos(pred_angle)
            pred_y = particle_y + rel_dist * math.sin(pred_angle)

            # Find best matching landmark
            best_log = -float('inf')
            for lm in self.landmarks:
                if lm.confidence < 0.3:
                    continue
                # Position agreement
                dx = pred_x - lm.x
                dy = pred_y - lm.y
                dist_sq = dx**2 + dy**2
                pos_log = -dist_sq / (2 * sigma_pos_sq) - math.log(2 * math.pi * sigma_pos_sq)

                # Class agreement (same class = high, different = low)
                if lm.label_idx == label_idx:
                    class_log = math.log(1.0 - self.sigma_class + 1e-9)
                else:
                    class_log = math.log(self.sigma_class / (N_LABELS - 1) + 1e-9)

                total = pos_log + class_log + math.log(lm.confidence)
                if total > best_log:
                    best_log = total

            if best_log > -float('inf'):
                log_lik += best_log * conf

        return log_lik

    def get_landmarks_for_display(self):
        """Return landmarks for visualization."""
        return [(lm.x, lm.y, lm.label_idx, lm.confidence)
                for lm in self.landmarks
                if lm.confidence >= self.confidence_threshold
                and lm.observations >= self.min_observations]

    def save(self, filepath):
        """Save landmark map to JSON."""
        import json
        data = [{
            'id': lm.id,
            'label': LABEL_NAMES[lm.label_idx] if lm.label_idx < len(LABEL_NAMES) else 'unknown',
            'label_idx': lm.label_idx,
            'x': lm.x,
            'y': lm.y,
            'confidence': lm.confidence,
            'size': lm.size,
            'observations': lm.observations,
        } for lm in self.landmarks]
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
