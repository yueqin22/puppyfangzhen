"""
Rao-Blackwellized Particle Filter SLAM (v4.0)
===============================================
FastSLAM-style SLAM where each particle carries its own occupancy grid map.

The filter estimates the joint posterior over robot trajectories and maps:
  p(x_{1:t}, m | z_{1:t}, u_{1:t})

Using Rao-Blackwellization:
  p(x_{1:t}, m | z, u) = p(m | x_{1:t}, z) * p(x_{1:t} | z, u)

Each particle has:
  - A trajectory estimate (like standard particle filter)
  - Its own occupancy grid map

References:
  - Montemerlo et al. (2002) "FastSLAM: A Factored Solution to the
    Simultaneous Localization and Mapping Problem"
  - Grisetti et al. (2007) "Improved Techniques for Grid Mapping with
    Rao-Blackwellized Particle Filters" (GMapping)
  - Murphy (2000) "Bayesian Map Learning in Dynamic Environments"
"""
import os
import math
import numpy as np
from occupancy_grid import OccupancyGrid, GRID_W, GRID_H, GRID_RESOLUTION

USE_RBPF_SLAM = os.environ.get("USE_RBPF_SLAM", "0") == "1"


class SLAMParticle:
    """Single particle in the RBPF SLAM filter."""

    def __init__(self, n_particles=0):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.weight = 0.0
        self.map = OccupancyGrid()
        self.weight_log = 0.0  # log weight for numerical stability


class RBPFSLAM:
    """Rao-Blackwellized Particle Filter for SLAM.

    Each particle maintains its own occupancy grid. The filter is updated
    with motion model (predict) and observation model (weight + map update).

    Note: Full RBPF SLAM is memory-intensive (N_particles * map_size).
    For a 100x80 grid with 50 particles, this is ~50 * 8000 = 400k cells
    for log_odds alone, which is manageable.
    """

    def __init__(self, n_particles=30, sigma_xy=0.05, sigma_yaw=0.05,
                 resample_threshold=0.5):
        self.n = n_particles
        self.sigma_xy = sigma_xy
        self.sigma_yaw = sigma_yaw
        self.resample_threshold = resample_threshold

        self.particles = []
        for _ in range(n_particles):
            self.particles.append(SLAMParticle())

        # Initialize weights uniformly
        for p in self.particles:
            p.weight = 1.0 / n_particles
            p.weight_log = math.log(p.weight)

    def init_at(self, x, y, yaw, spread=0.5):
        """Initialize all particles around a starting pose."""
        for p in self.particles:
            p.x = x + np.random.normal(0, spread * 0.3)
            p.y = y + np.random.normal(0, spread * 0.3)
            p.yaw = yaw + np.random.normal(0, spread * 0.5)
            p.weight = 1.0 / self.n
            p.weight_log = math.log(p.weight)

    def predict(self, v, w, dt):
        """Motion update: apply odometry with noise to each particle."""
        for p in self.particles:
            # Add noise to controls
            v_noisy = v + np.random.normal(0, self.sigma_xy)
            w_noisy = w + np.random.normal(0, self.sigma_yaw)

            # Simple differential-drive motion model
            if abs(w_noisy) > 1e-6:
                R = v_noisy / w_noisy
                p.x += -R * math.sin(p.yaw) + R * math.sin(p.yaw + w_noisy * dt)
                p.y += R * math.cos(p.yaw) - R * math.cos(p.yaw + w_noisy * dt)
                p.yaw += w_noisy * dt
            else:
                p.x += v_noisy * dt * math.cos(p.yaw)
                p.y += v_noisy * dt * math.sin(p.yaw)

            # Normalize yaw
            while p.yaw > math.pi:
                p.yaw -= 2 * math.pi
            while p.yaw < -math.pi:
                p.yaw += 2 * math.pi

    def update(self, angles, distances, frame=0):
        """Observation update: weight particles and update their maps."""
        # For each particle:
        # 1. Compute observation likelihood
        # 2. Update the particle's map with the scan
        # 3. Update weight
        for p in self.particles:
            # Compute likelihood by comparing scan to particle's map
            log_likelihood = self._scan_likelihood(p, angles, distances)

            # Update map
            p.map.update_from_scan(p.x, p.y, angles, distances, max_range=8.0)

            # Update weight (log domain)
            p.weight_log += log_likelihood

        # Normalize weights
        max_log_w = max(p.weight_log for p in self.particles)
        sum_w = 0.0
        for p in self.particles:
            p.weight = math.exp(p.weight_log - max_log_w)
            sum_w += p.weight

        if sum_w < 1e-30:
            # Reset weights
            for p in self.particles:
                p.weight = 1.0 / self.n
                p.weight_log = math.log(p.weight)
        else:
            for p in self.particles:
                p.weight /= sum_w
                p.weight_log = math.log(p.weight)

        # Compute effective sample size
        n_eff = 1.0 / sum(p.weight ** 2 for p in self.particles)

        # Resample if needed
        if n_eff < self.resample_threshold * self.n:
            self._resample()

        return n_eff

    def _scan_likelihood(self, particle, angles, distances, max_range=8.0):
        """Compute log-likelihood of a scan given a particle's map.

        Uses a simplified likelihood field model:
        - For each beam, find distance to nearest obstacle in map
        - Likelihood ∝ exp(-d^2 / 2σ^2)
        """
        sigma_hit = 0.2
        log_likelihood = 0.0

        # Pre-compute distance transform of the map (approximate)
        # For efficiency, we use a simple lookup approach
        grid = particle.map

        for angle, dist in zip(angles, distances):
            if dist >= max_range or dist < 0.1:
                continue  # skip max-range or invalid readings

            # Expected hit position in world
            wx = particle.x + dist * math.cos(particle.yaw + angle)
            wy = particle.y + dist * math.sin(particle.yaw + angle)
            gx, gy = grid.world_to_grid(wx, wy)

            if not grid.in_bounds(gx, gy):
                log_likelihood += math.log(0.01)  # low probability
                continue

            # Check if cell is occupied
            if grid.is_occupied(gx, gy):
                log_likelihood += math.log(0.9)  # high probability
            elif grid.is_unknown(gx, gy):
                log_likelihood += math.log(0.3)  # medium
            else:
                # Free space: check nearby cells
                nearest_occ_dist = self._nearest_occupied_dist(grid, gx, gy)
                if nearest_occ_dist < 0.5:
                    log_likelihood += math.log(0.6)
                else:
                    log_likelihood += math.log(0.1)

        return log_likelihood

    def _nearest_occupied_dist(self, grid, gx, gy, max_radius=10):
        """Approximate distance to nearest occupied cell (grid units)."""
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if grid.in_bounds(nx, ny) and grid.is_occupied(nx, ny):
                        return r * GRID_RESOLUTION
        return max_radius * GRID_RESOLUTION

    def _resample(self):
        """Systematic resampling of particles."""
        # Compute cumulative weights
        cumsum = [0.0]
        for p in self.particles:
            cumsum.append(cumsum[-1] + p.weight)

        # Systematic resampling
        new_particles = []
        step = 1.0 / self.n
        r = np.random.uniform(0, step)
        j = 1
        for i in range(self.n):
            u = r + i * step
            while j < len(cumsum) and cumsum[j] < u:
                j += 1
            # Copy particle j-1
            src = self.particles[max(0, min(j - 1, self.n - 1))]
            new_p = SLAMParticle()
            new_p.x = src.x
            new_p.y = src.y
            new_p.yaw = src.yaw
            new_p.weight = 1.0 / self.n
            new_p.weight_log = math.log(new_p.weight)
            # Copy the map (this is the expensive part)
            new_p.map.log_odds = src.map.log_odds.copy()
            new_p.map.visited = src.map.visited.copy()
            new_particles.append(new_p)

        self.particles = new_particles

    def get_best_particle(self):
        """Return the particle with highest weight."""
        best = max(self.particles, key=lambda p: p.weight)
        return best

    def get_mean_estimate(self):
        """Return weighted mean pose estimate."""
        mx = sum(p.weight * p.x for p in self.particles)
        my = sum(p.weight * p.y for p in self.particles)

        # Circular mean for yaw
        sin_sum = sum(p.weight * math.sin(p.yaw) for p in self.particles)
        cos_sum = sum(p.weight * math.cos(p.yaw) for p in self.particles)
        myaw = math.atan2(sin_sum, cos_sum)

        return mx, my, myaw

    def get_best_map(self):
        """Return the map of the best particle."""
        return self.get_best_particle().map
