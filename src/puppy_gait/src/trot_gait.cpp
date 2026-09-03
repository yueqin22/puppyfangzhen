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

// Hip offset of a leg in the body frame, REP-103 (+x forward, +y left).
// Keep in sync with the xacro that builds the robot: FR/RR sit at negative y
// (right side) and FR/FL at positive x (front).
static void hipOffset(LegId leg, double half_length, double half_width,
                      double& x, double& y) {
    const bool front = (leg == LegId::FR || leg == LegId::FL);
    const bool right = (leg == LegId::FR || leg == LegId::RR);
    x = front ? +half_length : -half_length;
    y = right ? -half_width : +half_width;
}

FootPosition TrotGait::getFootPosition(LegId leg, double t,
                                         double vx, double vy, double wz) const {
    // Normalize time to [0..1] phase
    double phase = std::fmod(t / params_.period, 1.0);
    if (phase < 0) phase += 1.0;

    // Add leg-specific phase offset
    double leg_phase = std::fmod(phase + getPhaseOffset(leg), 1.0);

    // Where this leg's hip sits in the body frame.
    double x_i = 0.0, y_i = 0.0;
    hipOffset(leg, params_.half_length, params_.half_width, x_i, y_i);

    // Rigid-body decomposition.
    //
    // A foot planted at body offset (x_i, y_i) has world velocity
    //     V_i = (vx - wz*y_i, vy + wz*x_i)
    // and staying planted means sweeping it by the negative of that, measured
    // in the body frame. generateTrajectory sweeps the foot by (-step_x,
    // -step_y) over a stance lasting duty_factor of one cycle, so
    //     step = V_i * period * duty_factor
    //
    // Two consequences, both of which the previous code got wrong:
    //
    //   * step_x must depend on y_i (LEFT/RIGHT). The fore/aft component of a
    //     turn is -wz*y_i, so the left and right legs must take different
    //     stride lengths. The old code used vx*period for every leg, so a turn
    //     command produced no fore/aft differentiation at all.
    //
    //   * step_y must depend on x_i (FRONT/REAR) for the rotational term, and
    //     must have NO leg dependence at all for the plain vy term. The old
    //     code flipped both by left/right, which is itself a rotation: a vy
    //     command yawed on the spot instead of strafing, and a wz command had
    //     its front and rear contributions cancel.
    //
    // The duty_factor term is not cosmetic. A pair is only in stance for that
    // fraction of the cycle, so omitting it makes the base travel 1/duty times
    // the commanded speed -- 2x at the default duty of 0.5.
    //
    // Both claims are checked offline by tools/check_gait_kinematics.py, which
    // solves for the rigid motion implied by the four foot paths and reports
    // the residual when the legs disagree.
    const double stance_time = params_.period * params_.duty_factor;
    const double step_x = (vx - wz * y_i) * stance_time;
    const double step_y = (vy + wz * x_i) * stance_time;

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
