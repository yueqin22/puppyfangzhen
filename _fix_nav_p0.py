#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Byte-level patcher for the two P0 navigation bugs found in the 2026-08-13
GPU smoke test (dog wedged on coffee_table: 16206 collisions / 99.9% stuck).

Both target files contain large amounts of GBK/UTF-8 mojibake comments, so we
operate on raw bytes and never decode the whole file.

FIX 1 (UE  PuppyRobotPawn.cpp) : per-axis sliding instead of whole-move revert
FIX 2 (Bridge nav_ue_bridge.cpp): generic stuck-escape (not boundary-only)
FIX 3 (Bridge nav_ue_bridge.cpp): honest A* stats (planner vs straight-line)
FIX 4 (Bridge nav_ue_bridge.cpp): collision-truncated fallback + speed cap
"""
import re
import shutil
import sys

UE_FILE = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"
BR_FILE = r"E:\puppyfangzhen\cpp_src\bridge\nav_ue_bridge.cpp"


def load(path):
    with open(path, "rb") as f:
        return f.read()


def save(path, data):
    shutil.copyfile(path, path + ".bak_p0fix")
    with open(path, "wb") as f:
        f.write(data)


def sub_once(data, pattern, repl, label):
    """Replace exactly one regex match, asserting the count."""
    rx = re.compile(pattern, re.DOTALL)
    found = rx.findall(data)
    if len(found) != 1:
        print("  !! %-22s expected 1 match, got %d -> SKIPPED" % (label, len(found)))
        return data, False
    data = rx.sub(lambda m: repl, data, count=1)
    print("  ok %-22s patched" % label)
    return data, True


# ===========================================================================
# FIX 1 - UE: per-axis sliding collision resolution
# ===========================================================================
UE_NEW = rb'''if (bInObstacleAfter && !bInObstacleBefore)
    {
        // R23 SLIDE FIX (2026-08-13): per-axis sliding instead of whole-move revert.
        // BUG: any contact reverted the ENTIRE delta, so a robot pressed against a
        // furniture face could never slide along it -> permanent wedge.  Observed in
        // the GPU smoke test: 16206 collisions, 99.9% stuck frames, the dog rammed
        // coffee_table at 2 m/s for 387 s and never reached patrol target #1.
        // FIX: retry the move on each axis separately (mirrors the standalone
        // simulator's x_safe / y_safe logic) and only revert fully when both axes
        // are blocked.  This lets the robot graze furniture and slide free.
        const FVector SlideX(LocAfter.X,  LocBefore.Y, LocAfter.Z);
        const FVector SlideY(LocBefore.X, LocAfter.Y,  LocAfter.Z);
        const bool bSlideXFree = !IsInsideHomeObstacle(SlideX.X, SlideX.Y);
        const bool bSlideYFree = !IsInsideHomeObstacle(SlideY.X, SlideY.Y);

        FVector ResolvedLoc = LocBefore;
        const TCHAR* ResolveMode = TEXT("REVERT");
        bool bSlid = false;
        if (bSlideXFree && bSlideYFree)
        {
            const double GainX = FMath::Abs(SlideX.X - LocBefore.X);
            const double GainY = FMath::Abs(SlideY.Y - LocBefore.Y);
            if (GainX >= GainY) { ResolvedLoc = SlideX; ResolveMode = TEXT("SLIDE-X"); }
            else                { ResolvedLoc = SlideY; ResolveMode = TEXT("SLIDE-Y"); }
            bSlid = true;
        }
        else if (bSlideXFree) { ResolvedLoc = SlideX; ResolveMode = TEXT("SLIDE-X"); bSlid = true; }
        else if (bSlideYFree) { ResolvedLoc = SlideY; ResolveMode = TEXT("SLIDE-Y"); bSlid = true; }

        bBboxCollision = true;

        SetActorLocation(ResolvedLoc, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

        ++CollisionCount;

        if (TcpComp != nullptr && ObsAfter != nullptr)
        {
            const float Bx = (ObsAfter->Xmin + ObsAfter->Xmax) * 0.5f;
            const float By = (ObsAfter->Ymin + ObsAfter->Ymax) * 0.5f;
            TcpComp->SendCollision(CollisionCount, 0.01 * Bx, 0.01 * By);
        }

        // Throttle: the old code logged every frame and produced 16k log lines.
        static double LastBboxLogT = 0.0;
        const double NowBboxT = FPlatformTime::Seconds();
        if (NowBboxT - LastBboxLogT > 2.0)
        {
            LastBboxLogT = NowBboxT;
            UE_LOG(LogTemp, Warning,
                   TEXT("[PuppyRobotPawn] BBOX-COLLIDE #%d: obs=[%s] (%.0f,%.0f)->(%.0f,%.0f) resolve=%s slid=%d"),
                   CollisionCount,
                   ObsAfter ? ObsAfter->Name : TEXT("?"),
                   LocBefore.X, LocBefore.Y, LocAfter.X, LocAfter.Y,
                   ResolveMode, bSlid ? 1 : 0);
        }

        LocAfter = ResolvedLoc;'''


# ===========================================================================
# FIX 3 + 4 - Bridge: honest A* stats + collision-truncated fallback
# ===========================================================================
BR_PLAN_NEW = rb'''bool path_from_planner = false;
        std::vector<std::pair<double, double>> path = state.nav.planner.plan(
            plan_x, plan_y,
            state.goal_x, state.goal_y);
        if (!path.empty()) path_from_planner = true;

        if (path.empty()) {
            path = state.nav.planner.plan_relaxed(
                plan_x, plan_y,
                state.goal_x, state.goal_y);
            if (!path.empty()) path_from_planner = true;
        }
        if (path.empty()) {
            path = state.nav.planner.plan_static_only_fallback(
                plan_x, plan_y,
                state.goal_x, state.goal_y);
            if (!path.empty()) path_from_planner = true;
        }
        if (path.empty()) {
            // P0 FIX (2026-08-13): the straight-line fallback used to drive the
            // robot THROUGH furniture (it rammed coffee_table 16206 times).  Now
            // the ray is truncated at the first unsafe sample so the robot stops
            // short of the obstacle instead of grinding into it.
            const double dx = state.goal_x - plan_x;
            const double dy = state.goal_y - plan_y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance > 0.01) {
                constexpr double kFallbackStep = 0.20;
                const int steps = std::max(1, static_cast<int>(std::ceil(distance / kFallbackStep)));
                path.reserve(static_cast<size_t>(steps) + 1);
                bool truncated = false;
                for (int i = 0; i <= steps; ++i) {
                    const double t = static_cast<double>(i) / static_cast<double>(steps);
                    const double px = plan_x + t * dx;
                    const double py = plan_y + t * dy;
                    if (i > 0 && !is_position_safe(px, py, state.obstacles)) {
                        truncated = true;
                        break;
                    }
                    path.emplace_back(px, py);
                }
                if (g_frame % 300 == 0) {
                    std::printf("[Bridge][PLAN-FALLBACK f=%d] A* empty; straight path to goal=(%.2f,%.2f), distance=%.3f, points=%zu truncated=%d\\n",
                                g_frame, state.goal_x, state.goal_y, distance, path.size(), truncated ? 1 : 0);
                }
            }
        }
        state.current_path = path;
        state.replan_counter = 0;
        state.nav.astar_calls++;
        // HONEST STATS: only a real planner result counts as success.  The old code
        // incremented astar_path_found after the straight-line fallback had filled
        // `path`, which reported astar_rate=100.0% while A* was failing every time.
        if (path_from_planner) state.nav.astar_path_found++;
        else state.astar_fallbacks++;'''


# ===========================================================================
# FIX 2 - Bridge: generic stuck-escape with safe-direction search
# ===========================================================================
BR_ESCAPE_NEW = rb'''const bool boundary_wedge = at_boundary && state.rxf_60s_boundary_count > 150;
            // P0 FIX (2026-08-13): escape used to require `at_boundary`, so a robot
            // wedged on furniture in the MIDDLE of the room never recovered.  The
            // GPU smoke test showed the dog jammed on coffee_table for 16207 frames
            // with zero escape attempts.  Now low displacement alone is enough.
            const bool window_mature = (g_frame % 300) >= 240;
            if (disp_10s < 0.3 && window_mature && (boundary_wedge || true)) {
                // Search 16 headings for the one with the most collision-free
                // clearance, lightly biased toward the current goal.
                double best_score = -1e18, best_ang = 0.0, best_clear = 0.0;
                const double to_goal_ang = std::atan2(state.goal_y - frame_data.true_y,
                                                      state.goal_x - frame_data.true_x);
                for (int k = 0; k < 16; ++k) {
                    const double ang = k * (2.0 * M_PI / 16.0);
                    double clear = 0.0;
                    for (double r = 0.15; r <= 1.20; r += 0.15) {
                        const double tx = frame_data.true_x + std::cos(ang) * r;
                        const double ty = frame_data.true_y + std::sin(ang) * r;
                        if (!is_position_safe(tx, ty, state.obstacles)) break;
                        clear = r;
                    }
                    const double score = clear * 2.0 + std::cos(ang - to_goal_ang) * 0.4;
                    if (score > best_score) { best_score = score; best_ang = ang; best_clear = clear; }
                }
                if (best_clear >= 0.30) {
                    state.rxf_escape_cmd_vx = std::cos(best_ang) * 0.8;
                    state.rxf_escape_cmd_vy = std::sin(best_ang) * 0.8;
                    state.rxf_escape_cmd_wz = 0.0;
                } else {
                    // Fully boxed in: back toward the room centre and spin.
                    const double to_cx = -frame_data.true_x, to_cy = -frame_data.true_y;
                    const double to_cd = std::sqrt(to_cx * to_cx + to_cy * to_cy);
                    state.rxf_escape_cmd_vx = (to_cd > 0.01) ? (to_cx / to_cd) * 0.6 : -0.4;
                    state.rxf_escape_cmd_vy = (to_cd > 0.01) ? (to_cy / to_cd) * 0.6 : 0.0;
                    state.rxf_escape_cmd_wz = -1.5;
                }
                // Short bursts (2 s) so the planner re-evaluates often.
                state.rxf_escape_remaining = 60;
                state.current_path.clear();
                state.rxf_escape_count++;
                printf("[Bridge][ESCAPE f=%d] disp=%.3f ang=%.2frad clear=%.2fm boundary=%d attempt=%d\\n",
                       g_frame, disp_10s, best_ang, best_clear, boundary_wedge ? 1 : 0,
                       state.rxf_escape_count);
                // Give up on an unreachable target after 8 failed escapes.
                if (state.rxf_escape_count >= 8 && !state.patrol_targets.empty()) {
                    state.patrol_idx = (state.patrol_idx + 1) % state.patrol_targets.size();
                    state.goal_x = state.patrol_targets[state.patrol_idx].x;
                    state.goal_y = state.patrol_targets[state.patrol_idx].y;
                    state.rxf_escape_count = 0;
                    printf("[Bridge][TARGET-SKIP f=%d] unreachable; advancing to #%zu (%.2f,%.2f)\\n",
                           g_frame, state.patrol_idx, state.goal_x, state.goal_y);
                }
                state.rxf_60s_gt_x = 1e9;
                fflush(stdout);
            }'''


def main():
    total_ok = 0

    # ---------------- UE ----------------
    print("[FIX 1] UE per-axis sliding  ->", UE_FILE)
    ue = load(UE_FILE)
    orig_ue = ue
    ue, ok = sub_once(
        ue,
        rb'if \(bInObstacleAfter && !bInObstacleBefore\).*?LocAfter = LocBefore;',
        UE_NEW,
        "ue.slide_collision",
    )
    total_ok += ok
    if ue != orig_ue:
        save(UE_FILE, ue)
        print("  -> written (backup .bak_p0fix)")

    # ---------------- Bridge ----------------
    print("\n[FIX 2/3/4] Bridge  ->", BR_FILE)
    br = load(BR_FILE)
    orig_br = br

    # new NavState fields
    br, ok = sub_once(
        br,
        rb'int rxf_60s_boundary_count = 0;',
        rb'''int rxf_60s_boundary_count = 0;
    int rxf_escape_count = 0;      // consecutive escape attempts on current target
    int astar_fallbacks = 0;       // straight-line fallbacks (A* genuinely failed)''',
        "br.navstate_fields",
    )
    total_ok += ok

    # honest stats + truncated fallback
    br, ok = sub_once(
        br,
        rb'std::vector<std::pair<double, double>> path = state\.nav\.planner\.plan\(.*?if \(!path\.empty\(\)\) state\.nav\.astar_path_found\+\+;',
        BR_PLAN_NEW,
        "br.plan_block",
    )
    total_ok += ok

    # generic stuck escape
    br, ok = sub_once(
        br,
        rb'if \(disp_10s < 0\.3 && at_boundary && state\.rxf_60s_boundary_count > 150\) \{.*?state\.rxf_60s_gt_x = 1e9;\s*\n\s*fflush\(stdout\);\s*\n\s*\}',
        BR_ESCAPE_NEW,
        "br.generic_escape",
    )
    total_ok += ok

    # reset escape counter whenever a patrol target is actually reached
    br, ok = sub_once(
        br,
        rb'state\.goal_x = state\.patrol_targets\[state\.patrol_idx\]\.x;\n            state\.goal_y = state\.patrol_targets\[state\.patrol_idx\]\.y;\n            state\.current_path\.clear\(\);',
        rb'''state.goal_x = state.patrol_targets[state.patrol_idx].x;
            state.goal_y = state.patrol_targets[state.patrol_idx].y;
            state.current_path.clear();
            state.rxf_escape_count = 0;   // reached a target -> reset escape budget''',
        "br.reset_escape_on_goal",
    )
    total_ok += ok

    # indoor speed cap 2.0 -> 1.2 m/s
    br, ok = sub_once(
        br,
        rb'double speed = std::min\(2\.0, dist_goal \* 2\.0\);\n            if \(goal_dist < 1\.0\)',
        rb'''double speed = std::min(1.2, dist_goal * 2.0);   // P0 FIX: 2.0->1.2 m/s indoors
            if (goal_dist < 1.0)''',
        "br.speed_cap",
    )
    total_ok += ok

    # surface fallback count in the health check
    br, ok = sub_once(
        br,
        rb'"  A\*:   calls=%d ok=%d rate=%\.1f%%  replan_interval=%d\\\\n"',
        rb'"  A*:   calls=%d ok=%d rate=%.1f%%  replan_interval=%d fallbacks=%d escapes=%d\\n"',
        "br.healthcheck_fmt",
    )
    total_ok += ok

    if br != orig_br:
        save(BR_FILE, br)
        print("  -> written (backup .bak_p0fix)")

    print("\n=== %d patches applied ===" % total_ok)
    return 0


if __name__ == "__main__":
    sys.exit(main())
