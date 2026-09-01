// ============================================================================
// test_body_geometry.cpp -- R33 (2026-08-14)
//
// Pins the two geometric facts that the UE collision checker got wrong, and the
// ordering invariant that keeps the whole stack safe.
//
// BACKGROUND
// The UE-side contact predicate (PuppyRobotPawn.cpp IsInsideHomeObstacle) used
// to grow each obstacle AABB by the robot radius R on all four sides
// independently:
//
//     x in [xmin-R, xmax+R]  AND  y in [ymin-R, ymax+R]
//
// That is the Minkowski sum of the box with a SQUARE of half-width R, i.e. it
// models a 2R x 2R AXIS-ALIGNED SQUARE robot.  The actual body is a capsule --
// a DISC of radius R in plan view (CollisionCapsule->SetCapsuleRadius(20.0f)).
// The correct forbidden region for a disc centre is the Minkowski sum with a
// DISC: the box grown by R with ROUNDED corners, i.e.
//
//     dist_point_to_AABB(p, box) < R
//
// The two agree along faces and diverge at convex corners, where the square
// test reaches R*sqrt(2) instead of R -- a 41.4% over-reach.  Every one of the
// 4 "collisions" in the r32 acceptance run was a door-frame convex corner with
// 6.9 cm of real body clearance.
//
// These tests are deliberately self-contained (they re-implement both
// predicates) so they document the distinction rather than merely exercising
// whichever version currently ships.
// ============================================================================
#include "test_framework.h"

#include <cmath>
#include <cstddef>

#include "../sim/lidar_safety.h"

namespace {

struct Box {
    double xmin, ymin, xmax, ymax;
};

// The scene's real door-frame post: scene_home.json "wall_h_mid".
constexpr Box kWallHMid{0.30, 0.80, 1.50, 1.00};

// PuppyRobotPawn.cpp:136 SetCapsuleRadius(20.0f) -> 0.20 m in plan view.
constexpr double kBodyRadius = 0.20;

// A* footprint used by is_footprint_traversable (costmap.h ROBOT_RADIUS).
constexpr double kPlannerFootprint = 0.35;

// Widest-path bottleneck of the 5-target patrol loop in scene_home.json,
// computed by max-min Dijkstra over the exact clearance field.  Any inflation
// radius at or above this value disconnects the loop.
constexpr double kLoopBottleneck = 0.50;

// The predicate that shipped before R33: Minkowski sum with a square.
bool inside_square(double px, double py, const Box& b, double r) {
    return px >= b.xmin - r && px <= b.xmax + r &&
           py >= b.ymin - r && py <= b.ymax + r;
}

// The predicate R33 installed: Minkowski sum with a disc.
bool inside_disc(double px, double py, const Box& b, double r) {
    const double dx = std::fmax(std::fmax(b.xmin - px, 0.0), px - b.xmax);
    const double dy = std::fmax(std::fmax(b.ymin - py, 0.0), py - b.ymax);
    return dx * dx + dy * dy < r * r;
}

double dist_to_box(double px, double py, const Box& b) {
    const double dx = std::fmax(std::fmax(b.xmin - px, 0.0), px - b.xmax);
    const double dy = std::fmax(std::fmax(b.ymin - py, 0.0), py - b.ymax);
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

// The three positions UE reported as collisions in the r32 run.  All sit
// diagonally off a door-frame corner with the body still clear of the wall.
//
// NOTE on precision: the BBOX-COLLIDE log line formats coordinates with %.0f,
// so these are whole centimetres recovered from the log, not the exact poses.
// Event #4's y lands on 0.60, which is precisely the square test's boundary
// (ymin - R = 0.80 - 0.20, which in IEEE-754 double is 0.6000000000000001), so
// the square predicate's verdict on it depends on the lost digits.  The
// physical claim -- the body was clear -- is unaffected and is asserted for all
// three; the square-flag assertion is only made where the logged precision
// determines it.
TEST(BodyGeometry, R32ReportedCollisionsWereCornerArtifacts) {
    struct Event { double x, y; bool square_flag_determined; };
    const Event kEvents[3] = {
        {1.69, 0.61, true},
        {0.11, 0.61, true},
        {1.68, 0.60, false},   // on the square boundary at logged precision
    };
    for (const Event& e : kEvents) {
        const double gap = dist_to_box(e.x, e.y, kWallHMid);
        // The body was demonstrably not touching: >= 6.5 cm of air.
        EXPECT_GT(gap, kBodyRadius + 0.065);
        // The disc predicate -- the correct one -- clears all three.
        EXPECT_FALSE(inside_disc(e.x, e.y, kWallHMid, kBodyRadius));
        // The square predicate flagged them, which is why they were reported.
        if (e.square_flag_determined) {
            EXPECT_TRUE(inside_square(e.x, e.y, kWallHMid, kBodyRadius));
        }
    }
}

// The two predicates must agree on a face approach -- the divergence is a
// corner-only effect, so a face test cannot detect the bug and a face
// regression must not be masked by this suite.
TEST(BodyGeometry, PredicatesAgreeOnFaceApproach) {
    const double x = 0.90;  // squarely below the middle of the wall's span
    // Just outside: 1 mm of clearance.
    EXPECT_FALSE(inside_disc(x, kWallHMid.ymin - kBodyRadius - 0.001,
                             kWallHMid, kBodyRadius));
    EXPECT_FALSE(inside_square(x, kWallHMid.ymin - kBodyRadius - 0.001,
                               kWallHMid, kBodyRadius));
    // Just inside: 1 mm of interpenetration.
    EXPECT_TRUE(inside_disc(x, kWallHMid.ymin - kBodyRadius + 0.001,
                            kWallHMid, kBodyRadius));
    EXPECT_TRUE(inside_square(x, kWallHMid.ymin - kBodyRadius + 0.001,
                              kWallHMid, kBodyRadius));
}

// The disc predicate must still catch every real interpenetration -- R33 is a
// correction, not a relaxation.  Sweep the full 360 deg around a corner at a
// radius just inside the body and require a hit at every angle.
TEST(BodyGeometry, DiscPredicateCatchesEveryRealPenetration) {
    const double cx = kWallHMid.xmax;  // convex corner (1.50, 0.80)
    const double cy = kWallHMid.ymin;
    const double kPi = 3.14159265358979323846;
    for (int deg = 0; deg < 360; ++deg) {
        const double a = deg * kPi / 180.0;
        const double r = kBodyRadius - 0.002;  // 2 mm of overlap
        const double px = cx + r * std::cos(a);
        const double py = cy + r * std::sin(a);
        EXPECT_TRUE(inside_disc(px, py, kWallHMid, kBodyRadius));
    }
    // And the corner over-reach of the square test is exactly R*(sqrt(2)-1).
    const double diag = kBodyRadius / std::sqrt(2.0);
    EXPECT_TRUE(inside_square(cx + diag * 1.4, cy - diag * 1.4, kWallHMid,
                              kBodyRadius));
    EXPECT_FALSE(inside_disc(cx + diag * 1.4, cy - diag * 1.4, kWallHMid,
                             kBodyRadius));
}

// The safety ladder.  Each layer must be strictly more conservative than the
// one it protects, and the most conservative layer must still fit through the
// scene.  Violating any link either lets the body touch geometry (too loose) or
// freezes the robot in a doorway (too tight) -- both have happened.
TEST(BodyGeometry, SafetyLadderIsMonotone) {
    const puppy::lidar_safety::FilterParams p;
    // Reactive filter must bind before the body can touch.
    EXPECT_GT(p.r_pass, kBodyRadius);
    // ...with a usable buffer, not a rounding error.
    EXPECT_GT(p.r_pass - kBodyRadius, 0.05);
    // Longitudinal standoff sits outside the shoulder half-width.
    EXPECT_GT(p.r_pass + p.margin, p.r_pass);
    // The planner must be more conservative than the reactive layer, so the
    // filter only ever engages on tracking error, never on nominal paths.
    EXPECT_GT(kPlannerFootprint, p.r_pass);
    // ...and must still clear the tightest bottleneck on the patrol loop.
    EXPECT_LT(kPlannerFootprint, kLoopBottleneck);
}

int main() {
    return RUN_ALL_TESTS();
}
