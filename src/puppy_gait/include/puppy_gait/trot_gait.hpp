#pragma once

#include <array>
#include <cmath>
#include "puppy_gait/inverse_kinematics.hpp"

namespace puppy_gait {

// Trot gait parameters
struct GaitParams {
    double period;        // gait cycle period [s]
    double duty_factor;   // stance phase fraction [0..1]
    double step_length;   // forward step length [m]
    double step_height;   // foot lift height [m]
    double body_height;   // standing height [m]
};

// Leg identifiers
enum class LegId {
    FR = 0,  // Front Right
    FL = 1,  // Front Left
    RR = 2,  // Rear Right
    RL = 3   // Rear Left
};

// Default trot gait parameters
constexpr GaitParams DEFAULT_GAIT = {
    0.5,   // 0.5s per cycle
    0.5,   // 50% stance, 50% swing
    0.08,  // 8cm step
    0.05,  // 5cm lift
    0.28   // 28cm standing height
};

/**
 * Trot gait generator.
 * Diagonal legs (FR+RL, FL+RR) move in phase.
 * Produces foot positions for all 4 legs at a given phase.
 */
class TrotGait {
public:
    explicit TrotGait(const GaitParams& params = DEFAULT_GAIT);

    /**
     * Get foot position for a specific leg at time t.
     * @param leg   Leg identifier
     * @param t     Time within gait cycle [0..period]
     * @param vx    Forward velocity command [m/s]
     * @param vy    Lateral velocity command [m/s]
     * @param wz    Angular velocity command [rad/s]
     * @return Foot position relative to hip joint
     */
    FootPosition getFootPosition(LegId leg, double t,
                                  double vx = 0.0,
                                  double vy = 0.0,
                                  double wz = 0.0) const;

    /**
     * Get all 4 foot positions at time t.
     */
    std::array<FootPosition, 4> getAllFootPositions(double t,
                                                      double vx = 0.0,
                                                      double vy = 0.0,
                                                      double wz = 0.0) const;

    /**
     * Get standing pose (all feet at default position).
     */
    std::array<FootPosition, 4> getStandingPose() const;

    void setParams(const GaitParams& params) { params_ = params; }
    const GaitParams& getParams() const { return params_; }

private:
    GaitParams params_;

    // Phase offset for each leg [0..1]
    // Trot: FR=0, FL=0.5, RR=0.5, RL=0
    double getPhaseOffset(LegId leg) const;

    // Generate foot trajectory for a single leg at normalized phase [0..1]
    FootPosition generateTrajectory(double phase, double step_x, double step_y) const;
};

}  // namespace puppy_gait
