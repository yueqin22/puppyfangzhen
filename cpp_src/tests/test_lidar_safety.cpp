// test_lidar_safety.cpp -- white-box tests for the reactive LiDAR safety layer
// ===========================================================================
// This layer is safety-critical and sits on the real UE delivery path, so its
// geometry is pinned here with cases that can be verified analytically by hand.
//
// Three historical regressions are encoded as tests.  Each one cost a full
// acceptance run to find, so if the filter is ever "simplified" they must fail:
//
//  1. R31a -- per-beam range-rate limit.  Passed every safety test and still
//     failed acceptance, because travelling PARALLEL to a wall shortens the
//     off-perpendicular beams for purely geometric reasons and the limiter read
//     that as closing on the wall (stuck 6.2% -> 37.7%, targets 23 -> 2).
//     Guarded by WallParallelTravelIsNotSlowed / NarrowCorridorAllowsFullSpeed.
//
//  2. R31b single-radius -- one "safe radius" served both as the shoulder
//     half-width and as the braking standoff.  With r = 0.334 the dog stalled
//     2090 consecutive frames at (1.00, 0.50) beside wall_h_mid: its nearest
//     return sat at 0.291 m (never once a collision there), but 0.291 < 0.334
//     put the return inside the radius, and a return inside the radius blocks
//     EVERY heading with a forward component.  Permanent freeze from a state
//     that was never dangerous.  Guarded by SideWallAt291mmDoesNotFreeze.
//
//  3. R31b full-circle fan -- the heading search scored retreat headings on the
//     same footing as the command (score = s * max(cos, 0.15)).  Driving at a
//     wall, forward scored 0.17 and straight-backwards scored 0.18, so the
//     robot answered "go forward" with "full speed in reverse".  Guarded by
//     RetreatNeverOutranksAUsableForwardHeading.
//
// Numbers below are derived from the two-radius contract:
//   r_pass = 0.27  shoulder half-width  (what counts as being in the way)
//   margin = 0.05  longitudinal standoff (how close the nose may get)
//   tau    = 0.35  braking horizon      (free distance -> speed)
// so a surface dead ahead brings the body centre to rest at r_pass + margin =
// 0.32 m, which is outside the 0.30 m UE collision capsule.
// ===========================================================================

#include "test_framework.h"
#include "../sim/lidar_safety.h"

#include <cmath>
#include <vector>

using namespace puppy::lidar_safety;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A synthetic 72-beam scan (5deg spacing), matching the UE LiDAR config.
struct Scan {
    std::vector<double> angles;
    std::vector<double> ranges;
    double max_range = 8.0;
    Scan() {
        for (int i = 0; i < 72; ++i) {
            angles.push_back(-kPi + 2.0 * kPi * i / 72.0);
            ranges.push_back(8.0);           // no hit everywhere by default
        }
    }
    // Ray-cast an infinite line  n.p = c  (n unit) into the scan.
    void add_line(double nx, double ny, double c) {
        for (std::size_t i = 0; i < angles.size(); ++i) {
            const double dx = std::cos(angles[i]);
            const double dy = std::sin(angles[i]);
            const double den = nx * dx + ny * dy;
            if (std::fabs(den) < 1e-9) continue;
            const double t = c / den;
            if (t > 0.0 && t < ranges[i]) ranges[i] = t;
        }
    }
};

FilterParams default_params() { return FilterParams(); }

double speed_of(const FilterResult& r) {
    return std::sqrt(r.vx * r.vx + r.vy * r.vy);
}

// The bridge/analyzer call a frame "stuck" below 0.005 m of GT travel per tick
// at 30 Hz, i.e. below 0.15 m/s.  Anything the filter emits as "moving" has to
// clear that bar or acceptance counts it as stuck.
constexpr double kStuckSpeed = 0.15;

}  // namespace

// --- cloud construction ----------------------------------------------------

// A dropout (0) must never be read as "surface at 0m": that would clamp every
// heading to zero and freeze the robot outright.  No-hit beams sit at
// max_range and must also be dropped.
TEST(LidarSafety, DropoutsAndNoHitsExcluded) {
    Scan s;
    s.ranges[0] = 0.0;       // dropout
    s.ranges[1] = -1.0;      // garbage
    s.ranges[2] = 8.0;       // no hit
    s.ranges[3] = 0.90;      // real return
    double nearest = 0.0;
    auto pts = build_cloud(s.angles, s.ranges, 0.0, s.max_range,
                           default_params(), &nearest);
    ASSERT_EQ((int)pts.size(), 1);
    EXPECT_NEAR(nearest, 0.90, 1e-9);
}

// Mismatched array lengths must fail closed (empty cloud) rather than read out
// of bounds.
TEST(LidarSafety, MismatchedArraysYieldEmptyCloud) {
    std::vector<double> a{0.0, 0.1, 0.2};
    std::vector<double> r{1.0, 1.0};
    auto pts = build_cloud(a, r, 0.0, 8.0, default_params(), nullptr);
    ASSERT_EQ((int)pts.size(), 0);
}

// Returns past the binding horizon are dropped from the cloud (they could not
// constrain any command anyway) but must STILL be reported as `nearest`, which
// is a diagnostic channel, not a control input.  Reporting only bindable points
// would make the trace lie about how close the robot really was.
TEST(LidarSafety, FarReturnsDroppedFromCloudButStillReported) {
    FilterParams p = default_params();
    std::vector<double> a{0.0, 0.0};
    std::vector<double> r{1.90, 0.60};       // 1.90 > point_range = 1.50
    double nearest = -1.0;
    auto pts = build_cloud(a, r, 0.0, 8.0, p, &nearest);
    ASSERT_EQ((int)pts.size(), 1);
    EXPECT_NEAR(pts[0].x, 0.60, 1e-9);
    EXPECT_NEAR(nearest, 0.60, 1e-9);
}

// --- corridor geometry -----------------------------------------------------

// Point straight ahead: the body may travel (d - r_pass) before its shoulder
// touches, less the standoff margin, in tau seconds.  Verified by hand.
TEST(LidarSafety, HeadOnPointLimitIsDistanceMinusShoulderAndMargin) {
    FilterParams p = default_params();
    std::vector<SafetyPoint> pts{{1.0, 0.0}};
    const double s = corridor_speed(pts, 1.0, 0.0, p, nullptr);
    EXPECT_NEAR(s, (1.0 - p.r_pass - p.margin) / p.tau, 1e-9);
}

// THE two-radius contract, stated as a distance: a surface dead ahead brings
// the body centre to a halt at r_pass + margin, and that has to sit outside the
// UE collision capsule (30 cm radius) or the filter is not actually preventing
// contact.
TEST(LidarSafety, BodyHaltsOutsideTheCollisionCapsule) {
    FilterParams p = default_params();
    const double standoff = p.r_pass + p.margin;
    EXPECT_GT(standoff, 0.30);                       // outside the capsule

    std::vector<SafetyPoint> at_standoff{{standoff, 0.0}};
    EXPECT_NEAR(corridor_speed(at_standoff, 1.0, 0.0, p, nullptr), 0.0, 1e-12);

    std::vector<SafetyPoint> just_beyond{{standoff + 0.10, 0.0}};
    EXPECT_NEAR(corridor_speed(just_beyond, 1.0, 0.0, p, nullptr),
                0.10 / p.tau, 1e-9);
}

// A point behind us cannot be hit by driving forward.
TEST(LidarSafety, PointBehindDoesNotConstrain) {
    FilterParams p = default_params();
    std::vector<SafetyPoint> pts{{-0.35, 0.0}};
    int blockers = -1;
    const double s = corridor_speed(pts, 1.0, 0.0, p, &blockers);
    EXPECT_GT(s, 1e8);           // unconstrained
    EXPECT_EQ(blockers, 0);
}

// The shoulder half-width is a hard boundary: a return one centimetre outside
// it passes by and constrains nothing.  This is the property the freeze fix
// depends on, so it is pinned right at the edge rather than comfortably away.
TEST(LidarSafety, ReturnJustOutsideTheShoulderDoesNotConstrain) {
    FilterParams p = default_params();
    std::vector<SafetyPoint> outside{{0.50, p.r_pass + 0.01}};
    int blockers = -1;
    EXPECT_GT(corridor_speed(outside, 1.0, 0.0, p, &blockers), 1e8);
    EXPECT_EQ(blockers, 0);

    std::vector<SafetyPoint> inside{{0.50, p.r_pass - 0.01}};
    blockers = -1;
    EXPECT_LT(corridor_speed(inside, 1.0, 0.0, p, &blockers), 1e8);
    EXPECT_EQ(blockers, 1);
}

// Already inside the shoulder and still driving at it -> zero speed, never a
// negative or NaN limit.
TEST(LidarSafety, PointAlreadyInsideShoulderGivesZeroSpeed) {
    FilterParams p = default_params();
    std::vector<SafetyPoint> pts{{0.20, 0.0}};
    const double s = corridor_speed(pts, 1.0, 0.0, p, nullptr);
    EXPECT_NEAR(s, 0.0, 1e-12);
    EXPECT_GE(s, 0.0);
}

// ...but escaping from it is still allowed at full speed.
TEST(LidarSafety, EscapingAnEmbeddedPointIsUnconstrained) {
    FilterParams p = default_params();
    std::vector<SafetyPoint> pts{{0.20, 0.0}};
    const double s = corridor_speed(pts, -1.0, 0.0, p, nullptr);
    EXPECT_GT(s, 1e8);
}

// --- regression 1: the R31a range-rate failure -----------------------------

// A wall 0.40m to the left, driving parallel to it: every wall point sits at
// cross-track 0.40 > r_pass, so the corridor is clear and the command must pass
// through UNCHANGED.  The per-beam range-rate filter capped this at ~0.2 m/s
// and produced 37.7% stuck.
TEST(LidarSafety, WallParallelTravelIsNotSlowed) {
    Scan s;
    s.add_line(0.0, 1.0, 0.40);          // wall at y = +0.40
    auto pts = build_cloud(s.angles, s.ranges, 0.0, s.max_range,
                           default_params(), nullptr);
    ASSERT_TRUE(pts.size() > 0);         // the wall IS visible to the sensor
    const auto out = filter_velocity(pts, 1.20, 0.0, default_params());
    EXPECT_NEAR(out.speed_scale, 1.0, 1e-6);
    EXPECT_NEAR(out.heading_dev, 0.0, 1e-9);
    EXPECT_NEAR(out.vx, 1.20, 1e-6);
    EXPECT_NEAR(out.vy, 0.0, 1e-6);
}

// A corridor 0.72m wide (walls at +-0.36) still permits full-speed travel down
// its length.  This is the case that decides whether the robot can cross the
// apartment at all; the tightest real bottleneck on the patrol loop measures
// 0.475 m of clearance, comfortably wider than this test.
TEST(LidarSafety, NarrowCorridorAllowsFullSpeedAlongIt) {
    Scan s;
    s.add_line(0.0,  1.0, 0.36);
    s.add_line(0.0, -1.0, 0.36);
    auto pts = build_cloud(s.angles, s.ranges, 0.0, s.max_range,
                           default_params(), nullptr);
    const auto out = filter_velocity(pts, 1.20, 0.0, default_params());
    EXPECT_NEAR(out.speed_scale, 1.0, 1e-6);
    EXPECT_NEAR(out.heading_dev, 0.0, 1e-9);
}

// Same wall, now driving INTO it.  Proves the two tests above are not passing
// simply because the filter is inert.
//
// The assertion is on the APPROACH RATE, not the total speed: deflecting to
// slide along the wall at speed is the desired answer, so clamping |v| would be
// the wrong requirement.  Analytically, for a wall at h with the ray meeting
// the inflated surface at (h - r_pass)/u_n, the approach rate is
//   s * u_n = (h - r_pass - margin*u_n)/tau  <=  (0.40-0.27)/0.35 = 0.371,
// and with the v_in cap the true maximum over the fan is 0.332 m/s.
TEST(LidarSafety, DrivingIntoTheWallHasItsApproachRateClamped) {
    Scan s;
    s.add_line(0.0, 1.0, 0.40);
    auto pts = build_cloud(s.angles, s.ranges, 0.0, s.max_range,
                           default_params(), nullptr);
    const auto out = filter_velocity(pts, 0.0, 1.20, default_params());
    EXPECT_GT(out.blockers, 0);          // it saw the wall in its corridor
    EXPECT_LT(out.vy, 0.40);             // approach rate crushed from 1.20
    EXPECT_LT(out.speed_scale, 1.0);     // and it is not passing through inert
}

// --- regression 2: the single-radius freeze --------------------------------

// The exact measured freeze state.  Nearest return 0.291 m abeam, which the
// robot demonstrably operates at without contact (2090 frames, zero
// collisions).  It must keep moving -- and specifically must clear the 0.15 m/s
// bar below which acceptance counts the frame as stuck.
TEST(LidarSafety, SideWallAt291mmDoesNotFreeze) {
    Scan s;
    s.add_line(0.0, 1.0, 0.291);
    auto pts = build_cloud(s.angles, s.ranges, 0.0, s.max_range,
                           default_params(), nullptr);
    ASSERT_TRUE(pts.size() > 0);
    const auto out = filter_velocity(pts, 1.20, 0.0, default_params());
    EXPECT_GT(speed_of(out), kStuckSpeed);
    EXPECT_NEAR(out.speed_scale, 1.0, 1e-6);   // 0.291 > r_pass: fully clear
    EXPECT_FALSE(out.unwedged);                // and no retreat was needed
}

// One rung tighter, INSIDE the shoulder, where tier 1 genuinely has no forward
// solution (a return inside r_pass blocks the whole forward half-plane).  The
// old design froze here forever.  The retreat tier must produce real motion,
// capped and gentle, and must flag itself so the trace shows what happened.
TEST(LidarSafety, WedgedRobotRetreatsInsteadOfFreezing) {
    FilterParams p = default_params();
    std::vector<SafetyPoint> pts{{0.20, 0.0}};   // inside r_pass, dead ahead
    const auto out = filter_velocity(pts, 1.20, 0.0, p);
    EXPECT_TRUE(out.unwedged);
    EXPECT_GT(speed_of(out), kStuckSpeed);
    EXPECT_LE(speed_of(out), p.unwedge_speed + 1e-9);
    EXPECT_GT(std::fabs(out.heading_dev), 80.0);   // it left the forward fan
}

// --- regression 3: the retreat that outranked the command ------------------

// Driving at a wall with open space behind: the answer must be a forward-ish
// slide, NEVER a reversal.  The full-circle fan with a 0.15 floor answered this
// with full speed in reverse, because backwards had more room.  Retreat is
// strictly subordinate: it may only run when tier 1 cannot move at all.
TEST(LidarSafety, RetreatNeverOutranksAUsableForwardHeading) {
    Scan s;
    s.add_line(0.0, 1.0, 0.40);          // wall ahead of a +Y command
    auto pts = build_cloud(s.angles, s.ranges, 0.0, s.max_range,
                           default_params(), nullptr);
    const auto out = filter_velocity(pts, 0.0, 1.20, default_params());
    EXPECT_FALSE(out.unwedged);                    // tier 1 solved it
    EXPECT_GT(out.vy, 0.0);                        // still making progress
    EXPECT_LE(std::fabs(out.heading_dev), 80.0);   // inside the forward fan
}

// --- heading search --------------------------------------------------------

// An isolated obstacle dead ahead should be steered around rather than stalled
// in front of: the filter must keep meaningful speed and pick a nonzero
// heading offset.
TEST(LidarSafety, SlidesAroundAnIsolatedObstacle) {
    FilterParams p = default_params();
    // A small cluster of returns straight ahead at 0.55m, ~0.2m wide.
    std::vector<SafetyPoint> pts;
    for (double y = -0.10; y <= 0.101; y += 0.05) {
        pts.push_back({0.55, y});
    }
    const auto out = filter_velocity(pts, 1.20, 0.0, p);
    EXPECT_GT(std::fabs(out.heading_dev), 1.0);   // it turned
    EXPECT_GT(out.speed_scale, 0.5);              // and kept real speed
    EXPECT_FALSE(out.unwedged);                   // without retreating
}

// When the command is already clear, the fan must not gratuitously deflect it:
// offset 0 wins all ties.  Otherwise the robot would wander off the A* path.
TEST(LidarSafety, KeepsCommandedHeadingWhenClear) {
    std::vector<SafetyPoint> pts;   // empty world
    const auto out = filter_velocity(pts, 0.8, -0.6, default_params());
    EXPECT_NEAR(out.heading_dev, 0.0, 1e-9);
    EXPECT_NEAR(out.vx, 0.8, 1e-9);
    EXPECT_NEAR(out.vy, -0.6, 1e-9);
    EXPECT_NEAR(out.speed_scale, 1.0, 1e-9);
}

// The filter must never speed the robot up beyond what was commanded.
TEST(LidarSafety, NeverExceedsCommandedSpeed) {
    std::vector<SafetyPoint> pts{{2.0, 0.0}};   // far away
    const auto out = filter_velocity(pts, 0.30, 0.0, default_params());
    EXPECT_LE(out.speed_scale, 1.0 + 1e-9);
    EXPECT_NEAR(out.vx, 0.30, 1e-9);
}

// A zero command must stay zero (and must not divide by zero).
TEST(LidarSafety, ZeroCommandStaysZero) {
    std::vector<SafetyPoint> pts{{0.35, 0.0}};
    const auto out = filter_velocity(pts, 0.0, 0.0, default_params());
    EXPECT_NEAR(out.vx, 0.0, 1e-12);
    EXPECT_NEAR(out.vy, 0.0, 1e-12);
}

// Fully enclosed by geometry: every heading is blocked, including the retreat
// fan, so the output must be zero -- and finite.  This is the state the escape
// burst exists to break, and the reason the bridge bypasses this filter while a
// burst is running.
TEST(LidarSafety, FullyWedgedYieldsZeroNotNaN) {
    std::vector<SafetyPoint> pts;
    for (int i = 0; i < 72; ++i) {
        const double a = -kPi + 2.0 * kPi * i / 72.0;
        pts.push_back({0.25 * std::cos(a), 0.25 * std::sin(a)});
    }
    const auto out = filter_velocity(pts, 1.20, 0.0, default_params());
    EXPECT_NEAR(out.speed_scale, 0.0, 1e-9);
    EXPECT_NEAR(speed_of(out), 0.0, 1e-12);
    EXPECT_FALSE(out.unwedged);          // nowhere to unwedge to
    EXPECT_TRUE(std::isfinite(out.vx));
    EXPECT_TRUE(std::isfinite(out.vy));
}

// Rotating the scan by the estimated yaw must rotate the constraint with it:
// the same wall expressed in a rotated robot frame must produce the same
// world-frame decision.  This pins the frame convention that the bridge relies
// on (world-frame command vs robot-frame scan angles).
TEST(LidarSafety, YawRotationIsAppliedToTheCloud) {
    Scan s;
    s.add_line(0.0, 1.0, 0.40);      // wall at robot-frame y = +0.40
    const double yaw = kPi / 2.0;    // robot facing world +Y
    auto pts = build_cloud(s.angles, s.ranges, yaw, s.max_range,
                           default_params(), nullptr);
    // After rotating by +90deg the wall lies along world x = -0.40, so driving
    // toward world -X must have its approach rate clamped while +X stays free.
    const auto blocked  = filter_velocity(pts, -1.20, 0.0, default_params());
    const auto free_dir = filter_velocity(pts,  1.20, 0.0, default_params());
    EXPECT_GT(blocked.blockers, 0);
    EXPECT_LT(-blocked.vx, 0.40);                  // approaching slowly
    EXPECT_NEAR(free_dir.speed_scale, 1.0, 1e-6);  // leaving at full speed
    EXPECT_EQ(free_dir.blockers, 0);
}

int main() {
    return RUN_ALL_TESTS();
}
