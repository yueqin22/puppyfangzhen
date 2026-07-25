#include "puppy_gait/inverse_kinematics.hpp"
#include <stdexcept>
#include <algorithm>

namespace puppy_gait {

InverseKinematics::InverseKinematics(const LegParams& params)
    : params_(params) {
    if (params_.L1 <= 0.0 || params_.L2 <= 0.0) {
        throw std::invalid_argument("Leg lengths must be positive");
    }
}

JointAngles InverseKinematics::solve(const FootPosition& target) const {
    JointAngles result;

    // hip_yaw: rotation around z-axis for lateral (y) displacement
    // Project foot onto x-z plane first
    double r_yz = std::sqrt(target.x * target.x + target.z * target.z);
    result.hip_yaw = std::atan2(target.y, r_yz);

    // In the sagittal plane (x-z), solve 2-link IK
    double x_plane = r_yz;  // distance in the sagittal plane
    double z_plane = target.z;

    // Distance from hip to foot in sagittal plane
    double r = std::sqrt(x_plane * x_plane + z_plane * z_plane);

    // Clamp r to reachable range
    double r_max = params_.L1 + params_.L2 - 0.001;
    double r_min = std::abs(params_.L1 - params_.L2) + 0.001;
    r = std::clamp(r, r_min, r_max);

    // Knee angle using law of cosines
    // cos(knee) = (L1^2 + L2^2 - r^2) / (2 * L1 * L2)
    double cos_knee = (params_.L1 * params_.L1 + params_.L2 * params_.L2 - r * r)
                      / (2.0 * params_.L1 * params_.L2);
    cos_knee = std::clamp(cos_knee, -1.0, 1.0);

    // Knee bends backward (negative angle for forward-facing knee)
    result.knee = -(M_PI - std::acos(cos_knee));

    // Hip pitch angle
    // Angle of the foot from hip in sagittal plane
    double alpha = std::atan2(x_plane, -z_plane);  // measured from -z axis

    // Angle of knee bisector
    double beta = std::atan2(params_.L2 * std::sin(-result.knee),
                             params_.L1 + params_.L2 * std::cos(-result.knee));

    result.hip_pitch = alpha - beta;

    return result;
}

FootPosition InverseKinematics::forward(const JointAngles& angles) const {
    FootPosition result;

    // In sagittal plane (before yaw rotation)
    double x_sag = params_.L1 * std::sin(angles.hip_pitch)
                 + params_.L2 * std::sin(angles.hip_pitch + angles.knee);
    double z_sag = -(params_.L1 * std::cos(angles.hip_pitch)
                   + params_.L2 * std::cos(angles.hip_pitch + angles.knee));

    // Apply hip yaw rotation (around z-axis)
    result.x = x_sag * std::cos(angles.hip_yaw);
    result.y = x_sag * std::sin(angles.hip_yaw);
    result.z = z_sag;

    return result;
}

}  // namespace puppy_gait
