#!/usr/bin/env python3
"""
Wheel Odometry (Dead Reckoning)
================================
Differential-drive dead-reckoning by integrating velocity commands.
Used to replace CoppeliaSim ground-truth position reads with a realistic
odometry source that has drift — the input to AMCL.

Model:
    x'   = x + v*cos(yaw)*dt
    y'   = y + v*sin(yaw)*dt
    yaw' = yaw + w*dt

Noise (simulates wheel slip, uneven floor):
    v_noisy = v * (1 + N(0, sigma_v))
    w_noisy = w * (1 + N(0, sigma_w))

The noise is what makes localization non-trivial — pure dead reckoning
drifts ~1m per 10m traveled, so AMCL correction is required.
"""
import math
import numpy as np


class Odometry:
    """Differential-drive wheel odometry with realistic drift."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0,
                 sigma_v=0.02, sigma_w=0.03):
        """Initialize odometry at pose (x, y, yaw).

        Args:
            sigma_v: linear velocity noise (2% of v)
            sigma_w: angular velocity noise (3% of w)
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w
        # Total distance traveled (for drift analysis)
        self.total_distance = 0.0
        self.total_rotation = 0.0
        # Last commanded velocity (for external inspection)
        self.last_v = 0.0
        self.last_w = 0.0

    def reset(self, x, y, yaw):
        """Reset to a known pose (e.g. at start)."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.total_distance = 0.0
        self.total_rotation = 0.0

    def update(self, v, w, dt):
        """Integrate velocity command over dt seconds.

        Adds multiplicative Gaussian noise to simulate real wheels.
        Returns the displacement (dx, dy, dyaw) for AMCL motion update.
        """
        # Add noise (multiplicative — faster motion = more absolute error)
        v_noisy = v * (1.0 + np.random.normal(0, self.sigma_v))
        w_noisy = w * (1.0 + np.random.normal(0, self.sigma_w))

        # Integrate (exact arc for small dt)
        old_x, old_y, old_yaw = self.x, self.y, self.yaw
        self.x += v_noisy * math.cos(self.yaw) * dt
        self.y += v_noisy * math.sin(self.yaw) * dt
        self.yaw += w_noisy * dt
        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Track totals
        self.total_distance += abs(v_noisy) * dt
        self.total_rotation += abs(w_noisy) * dt
        self.last_v = v_noisy
        self.last_w = w_noisy

        # Return relative motion (for AMCL sample motion model)
        dx = self.x - old_x
        dy = self.y - old_y
        dyaw = self.yaw - old_yaw
        return dx, dy, dyaw

    def get_pose(self):
        """Return current estimated pose (x, y, yaw)."""
        return self.x, self.y, self.yaw

    def drift_report(self, true_x, true_y, true_yaw):
        """Compute drift error vs ground truth (for analysis)."""
        pos_err = math.sqrt((self.x - true_x) ** 2 + (self.y - true_y) ** 2)
        yaw_err = abs(self.yaw - true_yaw)
        while yaw_err > math.pi:
            yaw_err = 2 * math.pi - yaw_err
        return {
            'pos_error_m': pos_err,
            'yaw_error_rad': yaw_err,
            'distance_traveled_m': self.total_distance,
            'drift_ratio': pos_err / max(self.total_distance, 0.01),
        }
