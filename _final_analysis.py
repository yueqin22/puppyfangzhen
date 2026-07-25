#!/usr/bin/env python3
"""Final comprehensive analysis for 3-hour long-run simulation."""
import csv
import math

CSV_PATH = r"C:\Users\Administrator\AppData\Local\Temp\puppy_nav_data.csv"

OBSTACLES = [
    ("wall_south",    -5.00, -4.05,  5.00, -3.95),
    ("wall_north",    -5.00,  3.95,  5.00,  4.05),
    ("wall_west",     -5.05, -4.00, -4.95,  4.00),
    ("wall_east",      4.95, -4.00,  5.05,  4.00),
    ("wall_divide_1", -5.00, -0.05, -1.00,  0.05),
    ("wall_divide_2",  1.00, -0.05,  5.00,  0.05),
    ("sofa",           2.75, -3.30,  4.25, -2.70),
    ("bed",           -4.25,  2.00, -2.75,  4.00),
    ("dining_table",   1.60,  2.47,  2.40,  2.53),
]
ROBOT_RADIUS = 0.35

def load_csv():
    rows = []
    with open(CSV_PATH, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append({
                    "frame": int(r["frame"]),
                    "time": float(r["time"]),
                    "rx": float(r["rx"]),
                    "ry": float(r["ry"]),
                    "ryaw": float(r["ryaw"]),
                    "state": int(r["state"]),
                    "coverage": float(r["coverage"]),
                    "dwa_v": float(r["dwa_v"]),
                    "dwa_w": float(r["dwa_w"]),
                    "true_x": float(r.get("true_x", r["rx"])),
                    "true_y": float(r.get("true_y", r["ry"])),
                    "loc_err": float(r.get("loc_err", 0.0)),
                })
            except (ValueError, KeyError):
                continue
    return rows

def is_inside_obstacle(x, y):
    for name, xmin, ymin, xmax, ymax in OBSTACLES:
        if x > xmin and x < xmax and y > ymin and y < ymax:
            return name
    return None

def time_segment_analysis(rows, segment_size=10000):
    """Break data into segments and compute stats for each."""
    print("=" * 90)
    print(f"{'Segment':>10s} {'Frames':>7s} {'Time(min)':>9s} {'loc_err':>8s} {'max':>6s} {'P95':>6s} "
          f"{'>0.5m':>5s} {'Wall':>5s} {'Jump':>5s} {'DONE%':>6s} {'FOLLOW%':>8s}")
    print("-" * 90)

    total = len(rows)
    state_names = {0: "PLAN", 1: "FOLLOW", 2: "RECOVER", 3: "DONE"}

    for start in range(0, total, segment_size):
        segment = rows[start:start + segment_size]
        if not segment:
            break

        errs = [r["loc_err"] for r in segment if r["loc_err"] > 0]
        errs_sorted = sorted(errs)

        # Wall-through check
        wall_count = sum(1 for r in segment if is_inside_obstacle(r["true_x"], r["true_y"]))

        # Position jumps
        jump_count = 0
        for i in range(1, len(segment)):
            dx = segment[i]["true_x"] - segment[i-1]["true_x"]
            dy = segment[i]["true_y"] - segment[i-1]["true_y"]
            if math.sqrt(dx*dx + dy*dy) > 1.0:
                jump_count += 1

        # State distribution
        done_count = sum(1 for r in segment if r["state"] == 3)
        follow_count = sum(1 for r in segment if r["state"] == 1)
        seg_size = len(segment)

        avg_err = sum(errs) / len(errs) if errs else 0
        max_err = max(errs) if errs else 0
        p95 = errs_sorted[int(len(errs_sorted) * 0.95)] if len(errs_sorted) > 0 else 0
        over_05 = sum(1 for e in errs if e > 0.5)

        time_min = segment[-1]["time"] / 60.0

        seg_label = f"{start//segment_size + 1}"
        print(f"{seg_label:>10s} {seg_size:>7d} {time_min:>9.1f} {avg_err:>8.3f} {max_err:>6.3f} "
              f"{p95:>6.3f} {over_05:>5d} {wall_count:>5d} {jump_count:>5d} "
              f"{100*done_count/seg_size:>6.1f} {100*follow_count/seg_size:>8.1f}")

    print("=" * 90)

def near_wall_analysis(rows):
    """Analyze which obstacles the robot got close to."""
    print("\n=== Near-Wall Analysis (true pos within 0.35m of obstacle) ===")
    obstacle_counts = {}
    for r in rows:
        x, y = r["true_x"], r["true_y"]
        for name, xmin, ymin, xmax, ymax in OBSTACLES:
            if (x > xmin - ROBOT_RADIUS and x < xmax + ROBOT_RADIUS and
                    y > ymin - ROBOT_RADIUS and y < ymax + ROBOT_RADIUS):
                inside = (x > xmin and x < xmax and y > ymin and y < ymax)
                if not inside:  # near but not inside
                    obstacle_counts[name] = obstacle_counts.get(name, 0) + 1

    if obstacle_counts:
        for name, count in sorted(obstacle_counts.items(), key=lambda x: -x[1]):
            print(f"  {name:20s}: {count:5d} frames")
    else:
        print("  No near-wall events")

def loc_err_timeline(rows, interval=5000):
    """Show loc_err at regular intervals."""
    print(f"\n=== Loc_err Timeline (every {interval} frames) ===")
    print(f"{'Frame':>8s} {'Time(min)':>9s} {'loc_err':>8s} {'state':>8s} {'pos(x,y)':>15s}")
    print("-" * 55)
    for r in rows[::interval]:
        state_name = {0: "PLAN", 1: "FOLLOW", 2: "RECOVER", 3: "DONE"}.get(r["state"], "?")
        print(f"{r['frame']:>8d} {r['time']/60:>9.1f} {r['loc_err']:>8.3f} "
              f"{state_name:>8s} ({r['true_x']:>5.2f},{r['true_y']:>5.2f})")

def speed_analysis(rows):
    """Analyze robot speed distribution."""
    print(f"\n=== Speed Analysis ===")
    speeds = []
    for i in range(1, len(rows)):
        dx = rows[i]["true_x"] - rows[i-1]["true_x"]
        dy = rows[i]["true_y"] - rows[i-1]["true_y"]
        dt = rows[i]["time"] - rows[i-1]["time"]
        if dt > 0:
            speed = math.sqrt(dx*dx + dy*dy) / dt
            speeds.append((speed, rows[i]["state"]))

    if not speeds:
        return

    all_speeds = [s[0] for s in speeds]
    moving_speeds = [s[0] for s in speeds if s[1] == 1]  # FOLLOW state

    print(f"  All frames:  avg={sum(all_speeds)/len(all_speeds):.3f} m/s  "
          f"max={max(all_speeds):.3f} m/s")
    if moving_speeds:
        print(f"  FOLLOW only: avg={sum(moving_speeds)/len(moving_speeds):.3f} m/s  "
              f"max={max(moving_speeds):.3f} m/s")

    # High speed events (>0.5 m/s = 0.05m/frame at 10Hz)
    high_speed = [(s, st) for s, st in speeds if s > 0.5]
    print(f"  High speed (>0.5 m/s): {len(high_speed)} frames")
    if high_speed:
        state_counts = {}
        for _, st in high_speed:
            state_name = {0: "PLAN", 1: "FOLLOW", 2: "RECOVER", 3: "DONE"}.get(st, "?")
            state_counts[state_name] = state_counts.get(state_name, 0) + 1
        for s, c in sorted(state_counts.items(), key=lambda x: -x[1]):
            print(f"    {s:10s}: {c} frames")

def main():
    rows = load_csv()
    if not rows:
        print("No data found!")
        return

    print("=" * 90)
    print(f"  FINAL ANALYSIS: {len(rows)} frames, {rows[-1]['time']/60:.1f} minutes "
          f"({rows[-1]['time']/3600:.2f} hours)")
    print(f"  Frame range: {rows[0]['frame']} - {rows[-1]['frame']}")
    print("=" * 90)

    # Time segment analysis
    time_segment_analysis(rows, segment_size=10000)

    # Near-wall analysis
    near_wall_analysis(rows)

    # Loc_err timeline
    loc_err_timeline(rows, interval=5000)

    # Speed analysis
    speed_analysis(rows)

    # Summary
    print("\n" + "=" * 90)
    print("  SUMMARY")
    print("=" * 90)

    wall_throughs = sum(1 for r in rows if is_inside_obstacle(r["true_x"], r["true_y"]))
    jumps = 0
    for i in range(1, len(rows)):
        dx = rows[i]["true_x"] - rows[i-1]["true_x"]
        dy = rows[i]["true_y"] - rows[i-1]["true_y"]
        if math.sqrt(dx*dx + dy*dy) > 1.0:
            jumps += 1

    amcl_inside = sum(1 for r in rows if is_inside_obstacle(r["rx"], r["ry"]))

    errs = [r["loc_err"] for r in rows if r["loc_err"] > 0]

    print(f"  Total frames:        {len(rows):>8d} ({rows[-1]['time']/3600:.2f} hours)")
    print(f"  Wall-through events: {wall_throughs:>8d}")
    print(f"  Position jumps:      {jumps:>8d}")
    print(f"  AMCL false loc:      {amcl_inside:>8d}")
    print(f"  loc_err avg:         {sum(errs)/len(errs):>8.3f} m")
    print(f"  loc_err max:         {max(errs):>8.3f} m")
    print(f"  loc_err > 0.5m:      {sum(1 for e in errs if e > 0.5):>8d} frames ({100*sum(1 for e in errs if e > 0.5)/len(errs):.4f}%)")
    print(f"  loc_err > 1.0m:      {sum(1 for e in errs if e > 1.0):>8d} frames")
    print(f"  Coverage:            {rows[-1]['coverage']:>8.2f}%")
    print("=" * 90)

if __name__ == "__main__":
    main()
