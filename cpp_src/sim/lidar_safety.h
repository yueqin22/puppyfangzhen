#pragma once
// ============================================================================
// lidar_safety.h -- reactive safety layer: swept-corridor velocity filter
//
// WHAT THIS IS FOR
// Everything upstream in the navigation stack reasons about the *map*.  The
// map is accurate (measured: beam hit points land on the JSON obstacle
// surfaces, median error ~0.01-0.03 m), so the map is not the problem --
// LOCALIZATION is.  AMCL error runs to 0.255 m worst case, so the controller
// can believe it has 0.35 m of clearance while the body actually has 0.10 m.
// Measured on the R30 baseline: all 36 UE contacts happened at a GT map
// clearance of 0.201-0.271 m, and none above that.  A LiDAR filter is immune
// to the estimator: it measures true geometry relative to the body, so it does
// not care where AMCL thinks the robot is.
//
// WHY A SWEPT-CORRIDOR TEST AND NOT A PER-BEAM RANGE-RATE LIMIT
// The obvious formulation -- constrain the velocity component along each beam
// by v . n_i <= (d_i - R)/tau -- was implemented, measured, and rejected.  It
// achieved the safety goal (collisions 36 -> 0) but destroyed mobility (stuck
// 6.2% -> 37.7%, targets 23 -> 2).  Reason: travelling PARALLEL to a wall at
// perpendicular distance h, the beam theta off perpendicular reads d =
// h/cos(theta).  Its range shrinks for purely geometric reasons while the real
// clearance h is constant, and the constraint misreads that as closing on an
// obstacle.  The swept-corridor test asks the only question that matters:
// would the body, travelling along this heading, actually sweep over that
// point?  Parallel travel is then unrestricted.
//
// WHY TWO RADII AND NOT ONE  (this is the fix for the R31b freeze)
// A single "safe radius" has to serve two contradictory jobs at once:
//   (a) deciding WHICH returns are in the way -- a shoulder half-width.  Too
//       large and a wall the robot is legitimately sliding past counts as an
//       obstruction.
//   (b) deciding HOW CLOSE the front may get -- a braking standoff.  Too
//       small and the robot stops with its nose already in the furniture.
// Making one number do both is what froze the robot.  Measured: with a single
// r = 0.334 the dog stalled for 2090 consecutive frames at (1.00, 0.50) beside
// wall_h_mid.  Its nearest return sat at 0.291 m -- a perfectly safe distance,
// it never once collided there -- but 0.291 < 0.334 put the return INSIDE the
// radius, and a point inside the radius blocks EVERY heading with a forward
// component (b <= |p| < r always holds, so free_d < 0 in the whole forward
// half-plane).  Total, permanent freeze from a state that was never dangerous.
// So the two jobs get two numbers:
//   r_pass  = 0.27  shoulder half-width.  Upper-bounded by the clearance the
//                   robot demonstrably operates at without contact (0.291 m
//                   measured, 2090 frames, zero collisions); lower-bounded by
//                   the clearance at which contact does occur (<= 0.271 m
//                   measured over all 36 events).  0.27 is the one value that
//                   fits between them.
//   margin  = 0.05  longitudinal standoff, so the body centre halts 0.32 m
//                   from a surface dead ahead -- comfortably outside the
//                   0.30 m collision capsule.
// Passability check: the tightest bottleneck on the patrol loop is 0.475 m of
// clearance (widest-path analysis over the scene), so r_pass = 0.27 leaves
// 0.205 m of slack in the worst doorway.
//
// FRAME CONVENTION -- READ BEFORE USE
// Points and headings passed in must be in the SAME frame, robot-centred.  In
// the UE bridge the command velocity is world frame and the scan angles are
// robot frame, so the caller rotates beams by the estimated yaw before calling
// in.  Yaw error is irreducible here (rotating the cloud by +yaw and rotating
// the command by -yaw are the same relative error), and at 30 Hz it does not
// accumulate: every frame re-measures from scratch.
// ============================================================================

#include <cmath>
#include <cstddef>
#include <vector>

namespace puppy {
namespace lidar_safety {

// A single obstacle return, robot-centred.
struct SafetyPoint {
    double x = 0.0;
    double y = 0.0;
};

struct FilterParams {
    // Shoulder half-width: returns farther than this from the velocity ray are
    // passed to one side and cannot obstruct.  See the header comment for why
    // this is NOT the collision radius.
    double r_pass = 0.27;
    // Longitudinal standoff subtracted from the free distance, so the body
    // stops r_pass + margin = 0.32 m short of a surface dead ahead.
    double margin = 0.05;
    // Braking horizon: free distance is divided by this to get a speed cap.
    double tau = 0.35;
    // Returns beyond this cannot bind (free_d/tau would exceed any command).
    double point_range = 1.50;
    // Tier-1 speed at or below which the robot is considered wedged and the
    // retreat search is allowed to run.
    double unwedge_eps = 0.02;
    // Speed cap for the retreat search.  The retreat heading is corridor-
    // verified, so this is prudence rather than safety: it keeps the unwedge
    // gentle and lets the planner re-evaluate quickly.
    double unwedge_speed = 0.50;
};

struct FilterResult {
    double vx = 0.0;
    double vy = 0.0;
    double speed_scale = 1.0;   // |v_out| / |v_in|
    double heading_dev = 0.0;   // chosen heading offset, degrees
    int    blockers = 0;        // returns inside the ORIGINAL commanded corridor
    double nearest = 0.0;       // nearest valid return (m), 0 if none
    bool   unwedged = false;    // true if the retreat fallback produced v_out
};

// Build the robot-centred obstacle cloud from one scan.
//
// `angles` are beam bearings in the robot frame and `ranges` the matching
// distances; `yaw` rotates them into the frame the velocity command lives in.
// Invalid returns are dropped rather than clamped: a 0 reading treated as
// "surface at 0 m" would block every heading and freeze the robot, and no-hit
// beams legitimately come back at max_range.
inline std::vector<SafetyPoint> build_cloud(const std::vector<double>& angles,
                                            const std::vector<double>& ranges,
                                            double yaw,
                                            double max_range,
                                            const FilterParams& p,
                                            double* nearest_out = nullptr) {
    std::vector<SafetyPoint> pts;
    if (angles.size() != ranges.size()) {
        if (nearest_out) *nearest_out = 0.0;
        return pts;
    }
    pts.reserve(ranges.size());
    double nearest = 0.0;
    bool have_nearest = false;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const double d = ranges[i];
        if (!(d > 0.0)) continue;                       // dropout
        if (d >= max_range - 1e-3) continue;            // no hit
        if (!have_nearest || d < nearest) { nearest = d; have_nearest = true; }
        if (d > p.point_range) continue;                // cannot bind
        const double th = yaw + angles[i];
        pts.push_back({ d * std::cos(th), d * std::sin(th) });
    }
    if (nearest_out) *nearest_out = have_nearest ? nearest : 0.0;
    return pts;
}

// Largest speed along the unit heading (ux,uy) that keeps the swept body clear.
//
// For a point at along-track distance a and cross-track offset b:
//   a <= 0        -> behind us, we are moving away from it.  Ignore.
//   |b| >= r_pass -> it passes by the shoulder.  Ignore.
//   otherwise     -> contact after travelling a - sqrt(r_pass^2 - b^2), from
//                    which the standoff margin is withheld.
// Returns a very large value when the corridor is entirely clear.
inline double corridor_speed(const std::vector<SafetyPoint>& pts,
                             double ux, double uy,
                             const FilterParams& p,
                             int* blockers = nullptr) {
    double free_min = 1e9;
    if (blockers) *blockers = 0;
    const double r2 = p.r_pass * p.r_pass;
    for (const auto& pt : pts) {
        const double a = pt.x * ux + pt.y * uy;
        if (a <= 0.0) continue;
        const double cx = pt.x - a * ux;
        const double cy = pt.y - a * uy;
        const double b2 = cx * cx + cy * cy;
        if (b2 >= r2) continue;
        if (blockers) (*blockers)++;
        const double free_d = a - std::sqrt(r2 - b2);
        if (free_d < free_min) free_min = free_d;
    }
    if (free_min >= 1e9) return 1e9;
    const double usable = free_min - p.margin;
    return (usable > 0.0 ? usable : 0.0) / p.tau;
}

// Heading offsets searched in the forward half-plane (tier 1).  Offset 0 is
// evaluated first and ties are never taken, so the commanded heading wins
// whenever it is just as good: the fan engages only when it strictly helps,
// and the robot does not wander off the A* path for no reason.
inline const double* forward_offsets(std::size_t* n) {
    static const double kOff[] = { 0.0, 12.0, -12.0, 24.0, -24.0, 36.0, -36.0,
                                   50.0, -50.0, 65.0, -65.0, 80.0, -80.0 };
    *n = sizeof(kOff) / sizeof(kOff[0]);
    return kOff;
}

// Additional offsets searched ONLY when tier 1 cannot move at all (tier 2).
// These point sideways and backwards, which is the only way out of a state
// where a return has landed inside r_pass: such a return blocks the entire
// forward half-plane, so without reach beyond 90deg the robot is frozen for
// good.  Kept strictly subordinate to tier 1 -- if it competed on equal terms
// the robot would veer away from every wall it tried to approach and would
// never reach a goal next to one.
inline const double* retreat_offsets(std::size_t* n) {
    static const double kOff[] = { 90.0, -90.0, 105.0, -105.0, 120.0, -120.0,
                                   145.0, -145.0, 180.0 };
    *n = sizeof(kOff) / sizeof(kOff[0]);
    return kOff;
}

// Filter one velocity command against one scan cloud.
inline FilterResult filter_velocity(const std::vector<SafetyPoint>& pts,
                                     double vx, double vy,
                                     const FilterParams& p) {
    FilterResult out;
    const double v_in = std::sqrt(vx * vx + vy * vy);
    if (v_in <= 1e-6) {
        out.vx = vx; out.vy = vy;
        return out;
    }
    const double ux0 = vx / v_in;
    const double uy0 = vy / v_in;
    corridor_speed(pts, ux0, uy0, p, &out.blockers);

    const double kDeg = 3.14159265358979323846 / 180.0;
    auto rotate = [&](double off_deg, double* ux, double* uy) {
        const double a = off_deg * kDeg;
        const double ca = std::cos(a), sa = std::sin(a);
        *ux = ux0 * ca - uy0 * sa;
        *uy = ux0 * sa + uy0 * ca;
        return ca;
    };

    // ---- Tier 1: make progress along the commanded direction --------------
    std::size_t n_fwd = 0;
    const double* fwd = forward_offsets(&n_fwd);
    double best_score = -1.0, best_s = 0.0;
    for (std::size_t i = 0; i < n_fwd; ++i) {
        double ux, uy;
        const double ca = rotate(fwd[i], &ux, &uy);
        double s = corridor_speed(pts, ux, uy, p);
        if (s > v_in) s = v_in;
        const double score = s * ca;
        if (score > best_score + 1e-9) {
            best_score = score;
            best_s = s;
            out.vx = ux * s;
            out.vy = uy * s;
            out.heading_dev = fwd[i];
        }
    }
    if (best_s > p.unwedge_eps) {
        out.speed_scale = std::sqrt(out.vx * out.vx + out.vy * out.vy) / v_in;
        return out;
    }

    // ---- Tier 2: wedged.  Any motion beats none. --------------------------
    // Score on raw free speed, not on progress toward the goal: the point is
    // to leave a state the planner cannot see, and the corridor test has
    // already verified whichever heading we pick.
    std::size_t n_ret = 0;
    const double* ret = retreat_offsets(&n_ret);
    const double cap = (p.unwedge_speed < v_in) ? p.unwedge_speed : v_in;
    double best_ret = 0.0;
    for (std::size_t i = 0; i < n_ret; ++i) {
        double ux, uy;
        rotate(ret[i], &ux, &uy);
        double s = corridor_speed(pts, ux, uy, p);
        if (s > cap) s = cap;
        if (s > best_ret + 1e-9) {
            best_ret = s;
            out.vx = ux * s;
            out.vy = uy * s;
            out.heading_dev = ret[i];
            out.unwedged = true;
        }
    }
    if (!out.unwedged) {          // fully enclosed: hold still, finite
        out.vx = 0.0; out.vy = 0.0; out.heading_dev = 0.0;
    }
    out.speed_scale = std::sqrt(out.vx * out.vx + out.vy * out.vy) / v_in;
    return out;
}

}  // namespace lidar_safety
}  // namespace puppy
