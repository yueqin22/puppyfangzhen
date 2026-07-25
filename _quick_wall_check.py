#!/usr/bin/env python3
"""Quick wall-through and position jump analysis for long-run simulation."""
import csv
import math
import sys

CSV_PATH = r"C:\Users\Administrator\AppData\Local\Temp\puppy_nav_data.csv"

# Obstacle bounding boxes (from CoppeliaSim simulation log)
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

def check_position_jumps(rows, threshold=1.0):
    """Check for position jumps > threshold meters between consecutive frames."""
    jumps = []
    for i in range(1, len(rows)):
        dx = rows[i]["true_x"] - rows[i-1]["true_x"]
        dy = rows[i]["true_y"] - rows[i-1]["true_y"]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > threshold:
            jumps.append({
                "frame": rows[i]["frame"],
                "from": (rows[i-1]["true_x"], rows[i-1]["true_y"]),
                "to": (rows[i]["true_x"], rows[i]["true_y"]),
                "dist": dist,
                "state": rows[i]["state"],
            })
    return jumps

def check_amcl_jumps(rows, threshold=1.0):
    """Check for AMCL position estimate jumps."""
    jumps = []
    for i in range(1, len(rows)):
        dx = rows[i]["rx"] - rows[i-1]["rx"]
        dy = rows[i]["ry"] - rows[i-1]["ry"]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > threshold:
            jumps.append({
                "frame": rows[i]["frame"],
                "from": (rows[i-1]["rx"], rows[i-1]["ry"]),
                "to": (rows[i]["rx"], rows[i]["ry"]),
                "dist": dist,
                "state": rows[i]["state"],
                "true_pos": (rows[i]["true_x"], rows[i]["true_y"]),
            })
    return jumps

def check_loc_err_stats(rows):
    """Compute loc_err statistics."""
    errs = [r["loc_err"] for r in rows if r["loc_err"] > 0]
    if not errs:
        return None
    errs.sort()
    n = len(errs)
    return {
        "count": n,
        "avg": sum(errs) / n,
        "min": errs[0],
        "max": errs[-1],
        "median": errs[n // 2],
        "p95": errs[int(n * 0.95)],
        "p99": errs[int(n * 0.99)],
        "over_05m": sum(1 for e in errs if e > 0.5),
        "over_10m": sum(1 for e in errs if e > 1.0),
    }

def check_state_distribution(rows):
    """Count frames in each state."""
    states = {}
    state_names = {0: "PLAN", 1: "FOLLOW", 2: "RECOVER", 3: "DONE"}
    for r in rows:
        s = r["state"]
        states[s] = states.get(s, 0) + 1
    return {state_names.get(k, f"S{k}"): v for k, v in sorted(states.items())}

def check_wall_through(rows):
    """Check if true position is inside any obstacle bounding box (expanded by robot radius)."""
    violations = []
    for r in rows:
        x, y = r["true_x"], r["true_y"]
        for name, xmin, ymin, xmax, ymax in OBSTACLES:
            # Expand bbox by robot radius
            if (x > xmin - ROBOT_RADIUS and x < xmax + ROBOT_RADIUS and
                    y > ymin - ROBOT_RADIUS and y < ymax + ROBOT_RADIUS):
                # Check if actually inside (not just near corner)
                inside = (x > xmin and x < xmax and y > ymin and y < ymax)
                violations.append({
                    "frame": r["frame"],
                    "pos": (x, y),
                    "obstacle": name,
                    "bbox": (xmin, ymin, xmax, ymax),
                    "inside": inside,
                    "state": r["state"],
                    "loc_err": r["loc_err"],
                })
    return violations

def check_amcl_wall_through(rows):
    """Check if AMCL position estimate is inside any obstacle (false localization)."""
    violations = []
    for r in rows:
        x, y = r["rx"], r["ry"]
        for name, xmin, ymin, xmax, ymax in OBSTACLES:
            if (x > xmin and x < xmax and y > ymin and y < ymax):
                violations.append({
                    "frame": r["frame"],
                    "amcl_pos": (x, y),
                    "true_pos": (r["true_x"], r["true_y"]),
                    "obstacle": name,
                    "state": r["state"],
                    "loc_err": r["loc_err"],
                })
    return violations

def main():
    rows = load_csv()
    if not rows:
        print("No data found!")
        return

    print(f"=== Quick Analysis: {len(rows)} frames (frame {rows[0]['frame']} - {rows[-1]['frame']}) ===")
    print()

    # State distribution
    states = check_state_distribution(rows)
    print("State Distribution:")
    total = sum(states.values())
    for s, count in sorted(states.items()):
        print(f"  {s:10s}: {count:6d} ({100*count/total:.1f}%)")
    print()

    # Wall-through check (true position)
    wall_violations = check_wall_through(rows)
    true_inside = [v for v in wall_violations if v["inside"]]
    print(f"Wall-through (true pos INSIDE obstacle): {len(true_inside)}")
    for v in true_inside[:10]:
        print(f"  Frame {v['frame']}: pos=({v['pos'][0]:.2f},{v['pos'][1]:.2f}) "
              f"inside {v['obstacle']} bbox={v['bbox']} state={v['state']} loc_err={v['loc_err']:.3f}")
    if len(true_inside) > 10:
        print(f"  ... and {len(true_inside)-10} more")

    near_wall = [v for v in wall_violations if not v["inside"]]
    print(f"Near-wall (true pos within {ROBOT_RADIUS}m of obstacle): {len(near_wall)}")
    print()

    # AMCL wall-through (AMCL estimate inside obstacle but true pos not)
    amcl_wall = check_amcl_wall_through(rows)
    print(f"AMCL estimate inside obstacle (false localization): {len(amcl_wall)}")
    for v in amcl_wall[:10]:
        print(f"  Frame {v['frame']}: amcl=({v['amcl_pos'][0]:.2f},{v['amcl_pos'][1]:.2f}) "
              f"true=({v['true_pos'][0]:.2f},{v['true_pos'][1]:.2f}) "
              f"obstacle={v['obstacle']} loc_err={v['loc_err']:.3f}")
    if len(amcl_wall) > 10:
        print(f"  ... and {len(amcl_wall)-10} more")
    print()

    # Position jumps (true position)
    jumps = check_position_jumps(rows, threshold=1.0)
    print(f"True Position Jumps (>1.0m): {len(jumps)}")
    for j in jumps[:10]:
        print(f"  Frame {j['frame']}: ({j['from'][0]:.2f},{j['from'][1]:.2f}) -> "
              f"({j['to'][0]:.2f},{j['to'][1]:.2f}) dist={j['dist']:.2f}m state={j['state']}")
    if len(jumps) > 10:
        print(f"  ... and {len(jumps)-10} more")
    print()

    # AMCL position jumps
    amcl_jumps = check_amcl_jumps(rows, threshold=1.0)
    print(f"AMCL Position Jumps (>1.0m): {len(amcl_jumps)}")
    for j in amcl_jumps[:10]:
        print(f"  Frame {j['frame']}: ({j['from'][0]:.2f},{j['from'][1]:.2f}) -> "
              f"({j['to'][0]:.2f},{j['to'][1]:.2f}) dist={j['dist']:.2f}m "
              f"state={j['state']} true=({j['true_pos'][0]:.2f},{j['true_pos'][1]:.2f})")
    if len(amcl_jumps) > 10:
        print(f"  ... and {len(amcl_jumps)-10} more")
    print()

    # loc_err stats
    loc_stats = check_loc_err_stats(rows)
    if loc_stats:
        print("Loc_err Statistics:")
        print(f"  Count:  {loc_stats['count']}")
        print(f"  Avg:    {loc_stats['avg']:.3f}m")
        print(f"  Median: {loc_stats['median']:.3f}m")
        print(f"  Min:    {loc_stats['min']:.3f}m")
        print(f"  Max:    {loc_stats['max']:.3f}m")
        print(f"  P95:    {loc_stats['p95']:.3f}m")
        print(f"  P99:    {loc_stats['p99']:.3f}m")
        print(f"  >0.5m:  {loc_stats['over_05m']} frames")
        print(f"  >1.0m:  {loc_stats['over_10m']} frames")
    print()

    # Coverage
    covs = [r["coverage"] for r in rows]
    print(f"Coverage: min={min(covs):.2f}% max={max(covs):.2f}% last={covs[-1]:.2f}%")

if __name__ == "__main__":
    main()
