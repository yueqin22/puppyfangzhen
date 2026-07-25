"""Analyze robot path for wall-clipping (穿墙) detection.

Reads puppy_nav_data.csv, checks if any robot position falls inside
wall/obstacle bounding boxes, and generates a path visualization.
"""
import csv
import math
import os
import tempfile

# Obstacle bounding boxes from build_coppelia_scene.py
OBSTACLES = [
    ("wall_south",   -5.00, -4.05,  5.00, -3.95),
    ("wall_north",   -5.00,  3.95,  5.00,  4.05),
    ("wall_west",    -5.05, -4.00, -4.95,  4.00),
    ("wall_east",     4.95, -4.00,  5.05,  4.00),
    # Divide walls (gap between x=-1 and x=1 is the doorway)
    ("wall_divide_1",-5.00, -0.05, -1.00,  0.05),
    ("wall_divide_2", 1.00, -0.05,  5.00,  0.05),
    ("wall_bedroom_1",-5.05, 0.00, -4.95,  4.00),
    ("wall_kitchen_1",4.95, 0.00,  5.05,  4.00),
    # Furniture
    ("sofa",          2.75, -3.30,  4.25, -2.70),
    ("bed",          -4.25,  2.00, -2.75,  4.00),
    ("dining_table",  1.60,  2.48,  2.40,  2.52),
]

# Robot radius for collision check
ROBOT_RADIUS = 0.35


def point_in_box(x, y, xmin, ymin, xmax, ymax, margin=0):
    """Check if point (x,y) is inside the box (with optional margin)."""
    return (xmin - margin) <= x <= (xmax + margin) and \
           (ymin - margin) <= y <= (ymax + margin)


def check_wall_clipping(rx, ry):
    """Check if robot position is inside any obstacle (accounting for robot radius).

    Returns list of (obstacle_name, penetration_depth) for each collision.
    """
    collisions = []
    for name, xmin, ymin, xmax, ymax in OBSTACLES:
        # Check if robot center is inside obstacle (expanded by robot radius)
        if point_in_box(rx, ry, xmin, ymin, xmax, ymax, margin=ROBOT_RADIUS):
            # Calculate penetration depth
            dx = min(abs(rx - xmin), abs(rx - xmax))
            dy = min(abs(ry - ymin), abs(abs(ry - ymax)))
            depth = min(dx, dy)
            collisions.append((name, depth))

            # Also check if center is inside the obstacle itself
            if point_in_box(rx, ry, xmin, ymin, xmax, ymax):
                collisions.append((name + "_CENTER", depth))

    return collisions


def analyze_csv(csv_path):
    """Read CSV and analyze path for wall-clipping."""
    positions = []
    collisions = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                frame = int(row['frame'])
                rx = float(row['rx'])
                ry = float(row['ry'])
                state = row.get('state', '')
                positions.append((frame, rx, ry, state))

                # Check for wall clipping
                hits = check_wall_clipping(rx, ry)
                for obs_name, depth in hits:
                    collisions.append((frame, rx, ry, obs_name, depth, state))
            except (ValueError, KeyError):
                continue

    return positions, collisions


def generate_html_map(positions, collisions, output_path):
    """Generate an HTML visualization of the robot path with collision markers."""
    # SVG dimensions
    MAP_W, MAP_H = 10.0, 8.0  # meters
    SCALE = 60  # pixels per meter
    SVG_W = int(MAP_W * SCALE) + 100
    SVG_H = int(MAP_H * SCALE) + 100
    OFFSET_X = 50
    OFFSET_Y = 50

    def to_svg(x, y):
        """Convert world coordinates to SVG coordinates."""
        sx = OFFSET_X + (x + 5) * SCALE  # x: [-5,5] -> [0,10]
        sy = SVG_H - OFFSET_Y - (y + 4) * SCALE  # y: [-4,4] -> [0,8], flipped
        return sx, sy

    html = f"""<!DOCTYPE html>
<html>
<head>
<title>Robot Path Analysis — Wall Clipping Check</title>
<style>
body {{ font-family: monospace; background: #1a1a2e; color: #eee; padding: 20px; }}
h1 {{ color: #e94560; }}
.info {{ background: #16213e; padding: 15px; border-radius: 8px; margin: 10px 0; }}
.collision {{ color: #ff4444; font-weight: bold; }}
.safe {{ color: #44ff44; }}
svg {{ border: 2px solid #333; border-radius: 4px; }}
.legend {{ display: inline-block; margin-right: 20px; }}
</style>
</head>
<body>
<h1>Robot Path Analysis</h1>
<div class="info">
<p><b>Total positions:</b> {len(positions)}</p>
<p><b>Wall-clipping incidents:</b> <span class="{'collision' if collisions else 'safe'}">{len(collisions)}</span></p>
"""

    if collisions:
        html += "<p><b>Collision details:</b></p><ul>"
        # Group collisions by obstacle
        by_obstacle = {}
        for frame, rx, ry, obs, depth, state in collisions:
            if obs not in by_obstacle:
                by_obstacle[obs] = []
            by_obstacle[obs].append((frame, rx, ry, depth, state))

        for obs, incidents in by_obstacle.items():
            html += f"<li class='collision'>{obs}: {len(incidents)} frames"
            # Show first 3 examples
            for f, x, y, d, s in incidents[:3]:
                html += f"<br>  frame={f} pos=({x:.2f},{y:.2f}) depth={d:.2f}m state={s}"
            if len(incidents) > 3:
                html += f"<br>  ... and {len(incidents)-3} more"
            html += "</li>"
        html += "</ul>"
    else:
        html += "<p class='safe'>No wall-clipping detected! Robot stays within valid space.</p>"

    html += f"""
</div>
<div class="info">
<span class="legend">🟢 Path</span>
<span class="legend">🔴 Collision</span>
<span class="legend">⬜ Obstacle</span>
<span class="legend">🟡 Start</span>
<span class="legend">🔵 End</span>
</div>
<svg width="{SVG_W}" height="{SVG_H}" style="background: #0f0f23">
"""

    # Draw obstacles
    for name, xmin, ymin, xmax, ymax in OBSTACLES:
        sx, sy = to_svg(xmin, ymax)  # top-left in SVG
        w = (xmax - xmin) * SCALE
        h = (ymax - ymin) * SCALE
        color = "#555" if "wall" in name else "#8B4513"
        html += f'<rect x="{sx:.0f}" y="{sy:.0f}" width="{w:.0f}" height="{h:.0f}" fill="{color}" stroke="#999" stroke-width="1"/>\n'
        # Label
        lx, ly = to_svg((xmin+xmax)/2, (ymin+ymax)/2)
        html += f'<text x="{lx:.0f}" y="{ly:.0f}" fill="#aaa" font-size="9" text-anchor="middle">{name}</text>\n'

    # Draw robot path (subsample for large datasets)
    step = max(1, len(positions) // 500)
    path_points = []
    for i in range(0, len(positions), step):
        frame, rx, ry, state = positions[i]
        sx, sy = to_svg(rx, ry)
        path_points.append(f"{sx:.1f},{sy:.1f}")

    if path_points:
        html += f'<polyline points="{" ".join(path_points)}" fill="none" stroke="#00ff88" stroke-width="2" opacity="0.7"/>\n'

    # Mark collision points
    collision_frames = set()
    for frame, rx, ry, obs, depth, state in collisions:
        if frame not in collision_frames:
            collision_frames.add(frame)
            sx, sy = to_svg(rx, ry)
            html += f'<circle cx="{sx:.0f}" cy="{sy:.0f}" r="5" fill="#ff3333" stroke="#fff" stroke-width="1"/>\n'

    # Mark start and end
    if positions:
        frame0, x0, y0, s0 = positions[0]
        sx, sy = to_svg(x0, y0)
        html += f'<circle cx="{sx:.0f}" cy="{sy:.0f}" r="6" fill="#ffdd00" stroke="#fff" stroke-width="2"/>\n'
        html += f'<text x="{sx+10:.0f}" y="{sy:.0f}" fill="#ffdd00" font-size="10">START f={frame0}</text>\n'

        frameN, xN, yN, sN = positions[-1]
        sx, sy = to_svg(xN, yN)
        html += f'<circle cx="{sx:.0f}" cy="{sy:.0f}" r="6" fill="#3399ff" stroke="#fff" stroke-width="2"/>\n'
        html += f'<text x="{sx+10:.0f}" y="{sy:.0f}" fill="#3399ff" font-size="10">END f={frameN} ({sN})</text>\n'

    html += """</svg>
</body>
</html>"""

    with open(output_path, 'w') as f:
        f.write(html)
    print(f"HTML map saved to: {output_path}")


def main():
    csv_path = os.path.join(tempfile.gettempdir(), "puppy_nav_data.csv")
    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return

    print(f"Analyzing: {csv_path}")
    positions, collisions = analyze_csv(csv_path)

    print(f"\n=== Path Analysis ===")
    print(f"Total positions: {len(positions)}")
    print(f"Wall-clipping incidents: {len(collisions)}")

    if collisions:
        print("\n=== Collision Details ===")
        by_obstacle = {}
        for frame, rx, ry, obs, depth, state in collisions:
            if obs not in by_obstacle:
                by_obstacle[obs] = []
            by_obstacle[obs].append((frame, rx, ry, depth, state))

        for obs, incidents in by_obstacle.items():
            print(f"\n{obs}: {len(incidents)} frames")
            for f, x, y, d, s in incidents[:5]:
                print(f"  frame={f} pos=({x:.2f},{y:.2f}) depth={d:.2f}m state={s}")
            if len(incidents) > 5:
                print(f"  ... and {len(incidents)-5} more")
    else:
        print("\n✅ No wall-clipping detected! Robot stays within valid space.")

    # Also check for "teleportation" (large jumps between consecutive frames)
    print("\n=== Teleportation Check ===")
    teleports = 0
    for i in range(1, len(positions)):
        f1, x1, y1, s1 = positions[i-1]
        f2, x2, y2, s2 = positions[i]
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        if dist > 1.0:  # More than 1m in one frame = teleport
            teleports += 1
            if teleports <= 5:
                print(f"  frame {f1}->{f2}: ({x1:.2f},{y1:.2f})->({x2:.2f},{y2:.2f}) dist={dist:.2f}m state={s1}->{s2}")
    if teleports > 5:
        print(f"  ... total {teleports} teleportation events")
    if teleports == 0:
        print("  ✅ No teleportation detected. Path is smooth.")

    # Generate HTML visualization
    output_path = os.path.join(tempfile.gettempdir(), "puppy_path_analysis.html")
    generate_html_map(positions, collisions, output_path)
    print(f"\nOpen in browser: file:///{output_path.replace(os.sep, '/')}")


if __name__ == "__main__":
    main()
