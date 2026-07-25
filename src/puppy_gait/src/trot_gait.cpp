#include "puppy_gait/trot_gait.hpp"

namespace puppy_gait {

TrotGait::TrotGait(const GaitParams& params)
    : params_(params) {}

double TrotGait::getPhaseOffset(LegId leg) const {
    // Trot: diagonal pairs in phase
    // FR + RL = phase 0
    // FL + RR = phase 0.5
    switch (leg) {
        case LegId::FR: return 0.0;
        case LegId::FL: return 0.5;
        case LegId::RR: return 0.5;
        case LegId::RL: return 0.0;
        default: return 0.0;
    }
}

FootPosition TrotGait::generateTrajectory(double phase, double step_x, double step_y) const {
    FootPosition foot;

    double duty = params_.duty_factor;

    if (phase < duty) {
        // Stance phase: foot moves backward (pushes body forward)
        double stance_progress = phase / duty;  // [0..1]

        // Linear backward motion
        foot.x = step_x / 2.0 - step_x * stance_progress;
        foot.y = step_y / 2.0 - step_y * stance_progress;
        foot.z = -params_.body_height;
    } else {
        // Swing phase: foot lifts and moves forward
        double swing_progress = (phase - duty) / (1.0 - duty);  // [0..1]

        // Forward motion (linear)
        foot.x = -step_x / 2.0 + step_x * swing_progress;
        foot.y = -step_y / 2.0 + step_y * swing_progress;

        // Vertical lift (sinusoidal)
        foot.z = -params_.body_height + params_.step_height * std::sin(M_PI * swing_progress);
    }

    return foot;
}

FootPosition TrotGait::getFootPosition(LegId leg, double t,
                                         double vx, double vy, double wz) const {
    // Normalize time to [0..1] phase
    double phase = std::fmod(t / params_.period, 1.0);
    if (phase < 0) phase += 1.0;

    // Add leg-specific phase offset
    double leg_phase = std::fmod(phase + getPhaseOffset(leg), 1.0);

    // Compute step length from velocity command
    // step = velocity * period (distance per cycle)
    double step_x = vx * params_.period;

    // Lateral and rotational steps depend on leg position
    double step_y = 0.0;

    // For rotation (wz), left and right legs have opposite lateral steps
    // This creates a turning motion
    bool is_right = (leg == LegId::FR || leg == LegId::RR);
    double rot_step = wz * params_.period * 0.1;  // scaled rotation

    if (is_right) {
        step_y = -vy * params_.period - rot_step;
    } else {
        step_y = vy * params_.period + rot_step;
    }

    return generateTrajectory(leg_phase, step_x, step_y);
}

std::array<FootPosition, 4> TrotGait::getAllFootPositions(double t,
                                                            double vx, double vy,
                                                            double wz) const {
    return {
        getFootPosition(LegId::FR, t, vx, vy, wz),
        getFootPosition(LegId::FL, t, vx, vy, wz),
        getFootPosition(LegId::RR, t, vx, vy, wz),
        getFootPosition(LegId::RL, t, vx, vy, wz),
    };
}

std::array<FootPosition, 4> TrotGait::getStandingPose() const {
    FootPosition stand;
    stand.x = 0.0;
    stand.y = 0.0;
    stand.z = -params_.body_height;

    return {stand, stand, stand, stand};
}

}  // namespace puppy_gait
