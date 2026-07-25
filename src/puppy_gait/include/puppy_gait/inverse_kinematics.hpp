#pragma once

#include <array>
#include <cmath>

namespace puppy_gait {

// Robot leg parameters
struct LegParams {
    double L1;  // thigh length
    double L2;  // calf length
};

// Joint angles for one leg
struct JointAngles {
    double hip_yaw;    // rotation around z
    double hip_pitch;  // rotation around y (thigh)
    double knee;       // rotation around y (calf)
};

// 3D foot position relative to hip joint
struct FootPosition {
    double x;  // forward
    double y;  // lateral
    double z;  // vertical (negative = below hip)
};

/**
 * Inverse kinematics solver for a 3-DoF quadruped leg.
 * hip_yaw rotates around z-axis (lateral movement).
 * hip_pitch and knee operate in the sagittal (x-z) plane.
 *
 * Coordinate convention:
 *   x = forward, y = lateral, z = up
 *   Foot z is negative (below the hip).
 */
class InverseKinematics {
public:
    explicit InverseKinematics(const LegParams& params = {0.20, 0.20});

    /**
     * Solve IK for a given foot position.
     * @param target  Foot position relative to hip joint
     * @return Joint angles (hip_yaw, hip_pitch, knee)
     */
    JointAngles solve(const FootPosition& target) const;

    /**
     * Forward kinematics: compute foot position from joint angles.
     * Used for verification.
     */
    FootPosition forward(const JointAngles& angles) const;

private:
    LegParams params_;
};

}  // namespace puppy_gait
