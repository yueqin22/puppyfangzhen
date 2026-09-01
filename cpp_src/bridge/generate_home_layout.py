"""
Generate a SPACIOUS MODERN OPEN-PLAN apartment layout for PuppyPi simulation.
v5.5 - FIXED ENTRY TRAP + Robot-navigation-friendly with DOUBLE DOOR WAYPOINTS:
       - FIXED: Entry/foyer 3-sided trap (sofa_chaise + shoe_cabinet + side_table_e)
       - Shortened L-sofa chaise to not block entry, removed obstructive side table
       - Moved shoe cabinet east to clear 1.0m+ entry corridor
       - Added return_west waypoint to guide robot through CENTER of living room
       - All doorways have double waypoints (outside + inside)
       - Fully open kitchen, generous 1.5m+ clearance on all main paths
       16m x 12m open living/dining/kitchen, 3 enclosed north rooms
"""
import json, math

WALL_OUTER = 0.20
WALL_INNER = 0.12
OW = WALL_OUTER / 2  # 0.10
IW = WALL_INNER / 2  # 0.06

# ============================================================
# COORDINATE SPACE - 16m x 12m (v5.4 NAVIGATION-FRIENDLY)
# ============================================================
# X: -8.0 to +8.0 (16m wide)
# Y: -6.0 to +6.0 (12m tall)
#
# Wall centerlines:
#   Outer:    x=-8.0, x=8.0, y=-6.0, y=6.0 (thickness 0.20)
#   h1:       y=2.2 (thickness 0.12, from 2.14 to 2.26) - north rooms divider
#   v_mb_b2:  x=-3.0 (thickness 0.12, from -3.06 to -2.94) - master/study divider
#   v_b2_ba:  x=0.5  (thickness 0.12, from 0.44 to 0.56) - study/bath divider
#   v_kw_up:  x=4.2  (thickness 0.12, from 4.14 to 4.26) - bath east/kitchen west (UPPER only)
#
# Room layout (north of h1):
#   Master bedroom:   -7.8 < x < -3.06, 2.26 < y < 5.8
#   Second bedroom/study: -2.94 < x < 0.44, 2.26 < y < 5.8
#   Bathroom:         0.56 < x < 4.14, 2.26 < y < 5.8
# Open plan south:   -7.8 < x < 7.8, -5.8 < y < 2.14 (FULLY OPEN)
# Kitchen (east):    4.26 < x < 7.8, -5.8 < y < 5.8 (open to dining/living)

walls = [
    # ===== OUTER WALLS =====
    {"name": "wall_south_w1",  "xmin": -8.0, "ymin": -6.0, "xmax": -0.6, "ymax": -5.80},
    {"name": "wall_south_w2",  "xmin":  0.6, "ymin": -6.0, "xmax":  8.0, "ymax": -5.80},
    {"name": "wall_north",     "xmin": -8.0, "ymin":  5.80, "xmax":  8.0, "ymax":  6.0},
    {"name": "wall_west",      "xmin": -8.0, "ymin": -6.0, "xmax": -7.80, "ymax":  6.0},
    {"name": "wall_east",      "xmin":  7.80,"ymin": -6.0, "xmax":  8.0, "ymax":  6.0},

    # ===== h1: NORTH ROOMS DIVIDER at y=2.2 =====
    {"name": "wall_h1_w1",     "xmin": -7.80, "ymin": 2.14, "xmax": -4.30, "ymax": 2.26},
    # Master door gap: x=-4.30 to x=-3.10 (1.2m wide)
    {"name": "wall_h1_w1b",    "xmin": -3.10, "ymin": 2.14, "xmax": -2.94, "ymax": 2.26},
    {"name": "wall_h1_w2",     "xmin": -2.94, "ymin": 2.14, "xmax": -0.80, "ymax": 2.26},
    # Study door gap: x=-0.80 to x=0.40 (1.2m)
    {"name": "wall_h1_w2b",    "xmin":  0.40, "ymin": 2.14, "xmax":  0.56, "ymax": 2.26},
    {"name": "wall_h1_w3",     "xmin":  0.56, "ymin": 2.14, "xmax":  1.60, "ymax": 2.26},
    # Bathroom door gap: x=1.60 to x=2.80 (1.2m)
    {"name": "wall_h1_w3b",    "xmin":  2.80, "ymin": 2.14, "xmax":  4.14, "ymax": 2.26},
    # NOTE: No wall between x=4.14 and 7.80 south of h1 - kitchen is fully open!

    # ===== VERTICAL DIVIDERS (north of h1 only!) =====
    {"name": "wall_v_mb_b2",   "xmin": -3.06, "ymin": 2.14, "xmax": -2.94, "ymax": 5.80},
    {"name": "wall_v_b2_ba",   "xmin":  0.44, "ymin": 2.14, "xmax":  0.56, "ymax": 5.80},
    {"name": "wall_v_kw_up",   "xmin":  4.14, "ymin": 2.14, "xmax":  4.26, "ymax": 5.80},
]

# ============================================================
# FURNITURE - arranged for robot navigation (NO corridor blocking!)
# ============================================================
furniture = [
    # ===== LIVING ROOM (SW, south open area) =====
    # v5.5: Main sofa SHORTENED to not extend past entry door (door at x=-0.6~0.6)
    {"name": "sofa_main",      "xmin": -4.50, "ymin": -5.70, "xmax": -0.80, "ymax": -4.70},
    # v5.5: Chaise SHORTENED - only extends 1.2m north from sofa, does NOT block entry
    {"name": "sofa_chaise",    "xmin": -0.80, "ymin": -4.70, "xmax":  0.00, "ymax": -3.50},
    {"name": "coffee_table",   "xmin": -2.50, "ymin": -3.80, "xmax": -0.50, "ymax": -2.80},
    # TV console against h1 wall, BETWEEN master door (ends x=-3.10) and study door (starts x=-0.80)
    {"name": "tv_console",     "xmin": -2.40, "ymin":  1.85, "xmax": -1.20, "ymax":  2.10},
    {"name": "tv_set",         "xmin": -1.90, "ymin":  1.90, "xmax": -1.70, "ymax":  2.10},
    # v5.5: Shoe cabinet MOVED EAST to x=1.2+ (0.6m from door edge at x=0.6), 1.0m+ entry clearance
    {"name": "shoe_cabinet",   "xmin":  1.20, "ymin": -5.70, "xmax":  2.20, "ymax": -5.00},
    {"name": "bookshelf_w",    "xmin": -7.75, "ymin": -4.50, "xmax": -7.10, "ymax": -1.00},
    {"name": "plant_sw",       "xmin": -7.00, "ymin": -5.60, "xmax": -6.20, "ymax": -4.80},
    {"name": "plant_tv",       "xmin": -6.50, "ymin":  1.20, "xmax": -5.90, "ymax":  1.80},
    # v5.5: REMOVED side_table_e - was creating the north wall of the entry trap
    {"name": "armchair",       "xmin": -6.00, "ymin": -3.50, "xmax": -4.80, "ymax": -2.50},
    {"name": "floor_lamp",     "xmin": -6.30, "ymin": -3.00, "xmax": -6.00, "ymax": -2.50},

    # ===== DESK on WEST wall (against west wall, not blocking) =====
    {"name": "desk",           "xmin": -7.60, "ymin": -0.50, "xmax": -6.20, "ymax":  0.10},
    {"name": "desk_chair",     "xmin": -7.20, "ymin": -1.20, "xmax": -6.50, "ymax": -0.55},

    # ===== WASHBASIN (outside bathroom, wet/dry separation) - against h1 wall, east side =====
    # Leave 0.4m clearance from bathroom door (ends at x=2.8)
    {"name": "washbasin",      "xmin":  3.20, "ymin":  1.60, "xmax":  4.00, "ymax":  2.10},
    {"name": "mirror",         "xmin":  3.40, "ymin":  2.10, "xmax":  3.80, "ymax":  2.14},

    # ===== WARDROBE in master bedroom (against NORTH wall) =====
    {"name": "wardrobe_mb",    "xmin": -4.50, "ymin":  4.80, "xmax": -3.20, "ymax":  5.70},

    # ===== DINING AREA (center-east, moved SOUTH with LOTS of clearance) =====
    {"name": "dining_table",   "xmin":  1.80, "ymin": -2.50, "xmax":  3.40, "ymax": -1.60},
    {"name": "chair_n1",       "xmin":  2.10, "ymin": -1.50, "xmax":  2.50, "ymax": -1.20},
    {"name": "chair_n2",       "xmin":  2.80, "ymin": -1.50, "xmax":  3.20, "ymax": -1.20},
    {"name": "chair_s1",       "xmin":  2.10, "ymin": -2.90, "xmax":  2.50, "ymax": -2.55},
    {"name": "chair_s2",       "xmin":  2.80, "ymin": -2.90, "xmax":  3.20, "ymax": -2.55},
    {"name": "sideboard",      "xmin":  2.40, "ymin": -5.70, "xmax":  3.90, "ymax": -4.80},
    {"name": "hutch",          "xmin":  7.00, "ymin": -4.50, "xmax":  7.75, "ymax": -3.50},

    # ===== KITCHEN (east side, FULLY OPEN to dining - no wall!) =====
    {"name": "counter_n",      "xmin":  6.70, "ymin":  3.00, "xmax":  7.75, "ymax":  5.70},
    {"name": "counter_e",      "xmin":  6.70, "ymin": -2.00, "xmax":  7.75, "ymax":  2.10},
    {"name": "stove",          "xmin":  6.80, "ymin":  1.00, "xmax":  7.70, "ymax":  1.80},
    {"name": "kitchen_sink",   "xmin":  6.80, "ymin": -0.50, "xmax":  7.70, "ymax":  0.30},
    {"name": "kitchen_island", "xmin":  5.80, "ymin":  0.00, "xmax":  6.60, "ymax":  1.00},
    {"name": "fridge",         "xmin":  5.50, "ymin":  4.80, "xmax":  6.40, "ymax":  5.70},
    {"name": "pantry",         "xmin":  4.40, "ymin":  4.80, "xmax":  5.40, "ymax":  5.70},
    {"name": "microwave",      "xmin":  6.80, "ymin": -1.80, "xmax":  7.50, "ymax": -1.20},
    {"name": "plant_kitchen",  "xmin":  7.20, "ymin": -5.60, "xmax":  7.75, "ymax": -5.00},

    # ===== MASTER BEDROOM (NW, north of h1) - furniture against walls, CLEAR entrance! =====
    {"name": "master_bed",     "xmin": -6.80, "ymin":  3.50, "xmax": -4.60, "ymax":  5.20},
    {"name": "mb_pillows",     "xmin": -6.80, "ymin":  4.80, "xmax": -4.60, "ymax":  5.20},
    {"name": "nightstand_w",   "xmin": -7.70, "ymin":  4.00, "xmax": -7.00, "ymax":  4.60},
    {"name": "nightstand_e",   "xmin": -4.50, "ymin":  4.00, "xmax": -3.80, "ymax":  4.60},
    # Dresser against WEST wall (north section, well away from door)
    {"name": "dresser_mb",     "xmin": -7.70, "ymin":  2.40, "xmax": -6.20, "ymax":  3.00},
    # Armchair in NE corner of master bedroom (NOT blocking door at x=-4.3~-3.1!)
    {"name": "mb_chair",       "xmin": -4.00, "ymin":  4.50, "xmax": -3.20, "ymax":  5.30},

    # ===== SECOND BEDROOM / STUDY (N-center) - furniture against walls, CLEAR entrance! =====
    {"name": "single_bed",     "xmin": -2.70, "ymin":  3.80, "xmax": -1.00, "ymax":  5.20},
    {"name": "sb_pillow",      "xmin": -2.70, "ymin":  4.80, "xmax": -1.00, "ymax":  5.20},
    # Desk against NORTH wall (east side), chair tucked under
    {"name": "desk_b2",        "xmin": -0.50, "ymin":  4.50, "xmax":  0.40, "ymax":  5.20},
    {"name": "chair_b2",       "xmin": -0.30, "ymin":  3.90, "xmax":  0.20, "ymax":  4.45},
    # Bookshelf + wardrobe stacked against WEST wall (south to north)
    {"name": "bookshelf_b2",   "xmin": -2.90, "ymin":  2.40, "xmax": -2.00, "ymax":  3.80},
    {"name": "wardrobe_b2",    "xmin": -2.90, "ymin":  3.90, "xmax": -2.00, "ymax":  5.60},
    {"name": "sb_nightstand",  "xmin": -1.00, "ymin":  4.50, "xmax": -0.40, "ymax":  5.10},

    # ===== BATHROOM (NE, north of h1) - CLEAR entrance from door at x=1.6~2.8 =====
    # Toilet/bidet along WEST wall (south section, near divider)
    {"name": "toilet",         "xmin":  0.70, "ymin":  2.40, "xmax":  1.30, "ymax":  3.20},
    {"name": "bidet",          "xmin":  1.30, "ymin":  2.40, "xmax":  1.80, "ymax":  3.00},
    {"name": "laundry",        "xmin":  0.70, "ymin":  3.40, "xmax":  1.40, "ymax":  4.20},
    # Bathtub along EAST wall (north section)
    {"name": "bathtub",        "xmin":  3.00, "ymin":  3.80, "xmax":  4.00, "ymax":  5.60},
    # Sink along EAST wall (south section, near door but not blocking)
    {"name": "bath_sink",      "xmin":  3.00, "ymin":  2.40, "xmax":  4.00, "ymax":  3.20},
    {"name": "bath_cabinet",   "xmin":  1.80, "ymin":  4.50, "xmax":  2.80, "ymax":  5.50},
]

all_obstacles = walls + furniture

# ============================================================
# SAFETY CHECK
# ============================================================
X_MIN, X_MAX = -7.80, 7.80
Y_MIN, Y_MAX = -5.80, 5.80
MARGIN = 0.30
DOOR_MIN = 0.90

def is_point_safe(x, y, obstacles, margin=MARGIN):
    if x < X_MIN + margin or x > X_MAX - margin or y < Y_MIN + margin or y > Y_MAX - margin:
        return False, f"out of bounds ({x:.2f},{y:.2f})"
    for obs in obstacles:
        if (x >= obs["xmin"] - margin and x <= obs["xmax"] + margin and
            y >= obs["ymin"] - margin and y <= obs["ymax"] + margin):
            return False, f"too close to {obs['name']}"
    return True, "safe"

def find_safe_point(center_x, center_y, obstacles, margin=0.25, search_radius=3.0, step=0.05):
    safe, reason = is_point_safe(center_x, center_y, obstacles, margin)
    if safe:
        return (round(center_x, 2), round(center_y, 2))
    for r in [i * step for i in range(1, int(search_radius/step) + 1)]:
        for angle_deg in range(0, 360, 5):
            angle = math.radians(angle_deg)
            px = center_x + r * math.cos(angle)
            py = center_y + r * math.sin(angle)
            safe, reason = is_point_safe(px, py, obstacles, margin)
            if safe:
                return (round(px, 2), round(py, 2))
    return None

# Room identification
def is_in_room(x, y, room_name):
    if room_name == "dock":
        # Dock is in the entry/foyer area, near the south entrance door
        return y < -4.0 and -0.5 < x < 1.5
    elif room_name in ("living_room_w", "living_room_c", "living_room_e", "hallway"):
        return y < 2.14 and x < 4.14 and y > -5.8
    elif room_name == "dining":
        return y < 0 and y > -4.0 and 0.0 < x < 5.0
    elif room_name == "kitchen":
        return x > 4.26 and y < 5.8 and y > -5.8
    elif room_name == "master_bed":
        return x < -3.06 and y > 2.26 and y < 5.8
    elif room_name == "study":
        return -2.94 < x < 0.44 and y > 2.26 and y < 5.8
    elif room_name == "bathroom":
        return 0.56 < x < 4.14 and y > 2.26 and y < 5.8
    return False

# ============================================================
# PATROL TARGETS - v5.5: DOUBLE WAYPOINTS + FIXED RETURN PATH
# ============================================================
# Route: dock(foyer) → living → hallway hub → [enter→visit→exit for each room] → kitchen → dining → return_south → dock
# v5.5: Dock moved to entry/foyer (0.5,-5.0), return path goes along south side
desired_targets = [
    # --- Entry/foyer dock near the south entrance door ---
    ("dock",             0.5,  -5.0,  0.0),
    ("living_room_w",   -5.5,  -2.0,  1.5708),
    ("living_room_c",   -2.0,  -2.0,  0.0),
    ("living_room_e",    0.0,  -3.0,  3.14159),
    # --- Central Hallway (hub) ---
    ("hallway",         -1.0,   0.8,  0.0),
    # --- Master bedroom: enter → visit → exit back to hallway ---
    ("door_mb_out",     -3.7,   1.8,  1.5708, True),
    ("door_mb_in",      -3.5,   2.6,  1.5708, True),
    ("master_bed",      -5.7,   4.0,  3.14159),
    ("door_mb_out",     -3.7,   1.8,  1.5708, True),   # exit
    # --- Study: enter → visit → exit back to hallway ---
    ("door_study_out",  -0.2,   1.8,  1.5708, True),
    ("door_study_in",    0.1,   2.6,  1.5708, True),
    ("study",           -1.5,   3.3,  3.14159),
    ("door_study_out",  -0.2,   1.8,  1.5708, True),   # exit
    # --- Bathroom: enter → visit → exit back to hallway ---
    ("door_bath_out",    2.2,   1.8,  1.5708, True),
    ("door_bath_in",     2.5,   2.6,  1.5708, True),
    ("bathroom",         2.2,   4.0,  3.14159),
    ("door_bath_out",    2.2,   1.8,  1.5708, True),   # exit
    # --- Kitchen (fully open, no door needed) and dining ---
    ("kitchen",          6.0,  -0.5,  3.14159),
    ("dining",           4.0,  -1.0,  1.5708),
    # Return waypoint: south of dining area, guides robot along south corridor back to foyer
    ("return_south",     3.5,  -4.0,  3.14159, True),
]

patrol_targets = []
print("=" * 70)
print("FINDING SAFE PATROL POINTS (v5.5 - fixed entry trap, return_west waypoint)")
print("=" * 70)
all_pass = True
for entry in desired_targets:
    name, cx, cy, yaw = entry[0], entry[1], entry[2], entry[3]
    is_waypoint = entry[4] if len(entry) > 4 else False
    margin = 0.15 if is_waypoint else 0.25
    pt = find_safe_point(cx, cy, all_obstacles, margin=margin, search_radius=1.5 if is_waypoint else 3.0)
    if pt:
        px, py = pt
        safe, reason = is_point_safe(px, py, all_obstacles, margin=0.12 if is_waypoint else 0.20)
        in_room = is_in_room(px, py, name) if not is_waypoint else True
        # For "in" waypoints, verify they are in the correct room
        if name.endswith("_in"):
            if "mb" in name:
                in_room = is_in_room(px, py, "master_bed")
            elif "study" in name:
                in_room = is_in_room(px, py, "study")
            elif "bath" in name:
                in_room = is_in_room(px, py, "bathroom")
        status = "PASS" if (safe and in_room) else "FAIL"
        if not (safe and in_room):
            all_pass = False
        wp_note = " [WAYPOINT]" if is_waypoint else ""
        room_note = "" if (in_room or is_waypoint) else f" [WRONG ROOM! at ({px:.2f},{py:.2f})]"
        if not safe:
            room_note = f" [UNSAFE: {reason}]"
        print(f"  [{status}] {name:20s} -> ({px:6.2f}, {py:6.2f}) yaw={yaw:.2f}{wp_note}{room_note}")
        pt_entry = {"name": name, "x": px, "y": py, "yaw": yaw}
        if is_waypoint:
            pt_entry["waypoint"] = True
        patrol_targets.append(pt_entry)
    else:
        all_pass = False
        print(f"  [FAIL] {name:20s} -> NO SAFE POINT near ({cx:.2f},{cy:.2f})")

INIT_X = patrol_targets[0]["x"] if patrol_targets else 0.0
INIT_Y = patrol_targets[0]["y"] if patrol_targets else -4.5
print(f"\n  Robot start (charging dock): ({INIT_X:.2f}, {INIT_Y:.2f})")
safe, reason = is_point_safe(INIT_X, INIT_Y, all_obstacles, margin=0.15)
print(f"  [{'PASS' if safe else 'FAIL'}] Start position: {reason}")

# ============================================================
# PEDESTRIANS - positioned in open areas, not blocking paths or doors
# ============================================================
pedestrians = [
    {"name": "person_1", "x": -2.0, "y": -2.0, "vx": -0.012, "vy":  0.008, "radius": 0.30},
    {"name": "person_2", "x":  2.5, "y": -3.5, "vx":  0.010, "vy": -0.006, "radius": 0.30},
    {"name": "person_3", "x": -6.0, "y": -1.5, "vx":  0.012, "vy": -0.008, "radius": 0.30},
    {"name": "person_4", "x":  5.5, "y": -2.5, "vx": -0.008, "vy":  0.008, "radius": 0.30},
    {"name": "person_5", "x":  5.5, "y":  3.5, "vx":  0.008, "vy": -0.012, "radius": 0.30},
    {"name": "person_6", "x": -1.5, "y":  0.0, "vx":  0.008, "vy": -0.008, "radius": 0.30},
]

print("\n" + "=" * 70)
print("PEDESTRIAN START POSITIONS")
print("=" * 70)
for p in pedestrians:
    safe, reason = is_point_safe(p["x"], p["y"], all_obstacles, margin=0.25)
    if not safe: all_pass = False
    print(f"  [{'PASS' if safe else 'FAIL'}] {p['name']:10s} at ({p['x']:5.2f},{p['y']:5.2f}): {reason}")

# ============================================================
# DOORWAY WIDTH CHECK
# ============================================================
print("\n" + "=" * 70)
print("DOORWAY WIDTH VERIFICATION")
print("=" * 70)
doors = [
    ("Entry door (south)",       -0.60,  0.60),
    ("Master bedroom door",      -4.30, -3.10),
    ("Study door",               -0.80,  0.40),
    ("Bathroom door",             1.60,  2.80),
    ("Kitchen-dining opening",    4.26,  7.80),
]
for name, x1, x2 in doors:
    w = abs(x2 - x1)
    ok = w >= DOOR_MIN
    if not ok: all_pass = False
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:32s}: {w:.2f}m (min {DOOR_MIN:.2f}m)")

# Check key circulation paths
print("\n  Circulation spot checks:")
corridor_checks = [
    ("West wall walkway",      -6.50, -1.5),
    ("Center open area",        0.00, -1.0),
    ("Kitchen west passage",    5.00, -0.5),
    ("Kitchen south entry",     5.50, -2.5),
    ("Hallway central",        -1.50,  0.8),
    ("Dining area west",        0.50, -2.0),
    ("South walkway",          -2.00, -4.2),
    ("Entry/foyer (door in)",   0.00, -5.0),
    ("Entry east passage",      0.80, -4.5),
    ("Return path (south)",      4.50, -3.5),
    ("Return path (west)",      -1.50, -4.0),
    ("Master bedroom inside",  -5.50,  3.0),
    ("Bathroom inside",         3.00,  3.5),
    ("Study inside",           -1.50,  3.5),
    ("Master door outside",    -3.70,  1.8),
    ("Master door inside",     -3.70,  2.6),
    ("Study door inside",      -0.20,  2.7),
    ("Bath door inside",        2.20,  2.7),
]
for name, cx, cy in corridor_checks:
    safe, reason = is_point_safe(cx, cy, all_obstacles, margin=0.30)
    ok = safe
    if not ok: all_pass = False
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:28s} at ({cx:5.2f},{cy:5.2f}): {reason}")

print("\n" + "=" * 70)
if all_pass:
    print("ALL CHECKS PASSED - v5.5 ROBOT-NAVIGATION-FRIENDLY layout ready!")
else:
    print("SOME CHECKS FAILED - review above")
print("=" * 70)

# ============================================================
# GENERATE JSON
# ============================================================
scene = {
    "version": "5.5",
    "description": "Robot-navigation-friendly open-plan apartment v5.5 - 16m x 12m, fixed entry trap, double waypoints, return_west guide",
    "source": "v5.5 - fixed 3-sided entry trap, shortened L-sofa, removed side table, added return_west waypoint",
    "grid": {
        "width": 160, "height": 120, "resolution": 0.1,
        "origin_x": -8.0, "origin_y": -6.0,
        "comment": "16m x 12m, 0.1m/cell"
    },
    "sim": {
        "fps": 30, "dt": 0.033333, "visual_max_speed": 2.0,
        "visual_max_step": 0.066667, "arrival_radius": 0.45,
        "soft_skip_radius": 1.0, "soft_skip_frames": 500,
        "hard_skip_frames": 1000, "dynamic_clearance": 1.0,
        "dyn_collision_radius": 0.45
    },
    "robot": {
        "radius_planning": 0.30, "radius_cbf": 0.25,
        "cbf_d_safe": 0.30, "cbf_d_critical": 0.08,
        "cbf_alpha": 2.0, "cbf_max_speed": 1.5,
        "initial_pose": {"x": INIT_X, "y": INIT_Y, "yaw": 0.0}
    },
    "lidar": {
        "n_rays": 72, "max_range": 12.0,
        "angle_min": -3.14159, "angle_max": 3.14159,
        "comment": "72 rays = 5 degree spacing, full 360 degrees, 12m range"
    },
    "amcl": {
        "n_particles": 300, "sigma_obs": 0.45, "z_max": 12.0,
        "n_obs_rays": 72, "kld_min": 50, "kld_max": 500,
        "kidnap_recover_spread": 3.0
    },
    "obstacles": all_obstacles,
    "patrol_targets": patrol_targets,
    "pedestrians": pedestrians,
    "rooms": [
        {"name": "foyer",        "test": "y < -4.0 and x > -1.5 and x < 2.5"},
        {"name": "living_room",  "test": "y < 2.14 and x < 4.14 and y > -5.8"},
        {"name": "dining",       "test": "y < 0 and y > -4.0 and x > 0.0 and x < 5.0"},
        {"name": "kitchen",      "test": "x > 4.26"},
        {"name": "master_bed",   "test": "x < -3.06 and y > 2.26"},
        {"name": "study",        "test": "x > -2.94 and x < 0.44 and y > 2.26"},
        {"name": "bathroom",     "test": "x > 0.56 and x < 4.14 and y > 2.26"},
        {"name": "hallway",      "test": "else"}
    ],
    "path_follower": {
        "lookahead": 0.6, "search_window": 20, "arrival_threshold": 0.45
    }
}

json_path = r"e:\puppyfangzhen\config\scene_home.json"
with open(json_path, 'w', encoding='utf-8') as f:
    json.dump(scene, f, indent=2, ensure_ascii=False)
print(f"\nScene JSON written: {json_path}")
print(f"  Obstacles: {len(all_obstacles)} ({len(walls)} walls, {len(furniture)} furniture)")
print(f"  Patrol targets: {len(patrol_targets)}")
print(f"  Pedestrians: {len(pedestrians)}")
print(f"  Space: 16m x 12m (X: -8.0 to 8.0, Y: -6.0 to 6.0)")

# ============================================================
# GENERATE C++ ARRAYS
# ============================================================
print("\n" + "=" * 70)
print("C++ HOME_OBSTACLES ARRAY")
print("=" * 70)
print("static const FHomeObstacle HOME_OBSTACLES[] = {")
for obs in all_obstacles:
    name_padded = obs["name"].ljust(24)
    xmin = obs["xmin"] * 100
    ymin = obs["ymin"] * 100
    xmax = obs["xmax"] * 100
    ymax = obs["ymax"] * 100
    print(f'    {{ TEXT("{name_padded}"), {xmin:7.1f}f,{ymin:7.1f}f,{xmax:7.1f}f,{ymax:7.1f}f }},')
print("};")
print("static const int NUM_HOME_OBSTACLES = UE_ARRAY_COUNT(HOME_OBSTACLES);")

print("\n" + "=" * 70)
print("C++ PEDESTRIAN CONFIG")
print("=" * 70)
print("    const FPedCfg Cfgs[6] = {")
for p in pedestrians:
    print(f'        {{ TEXT("{p["name"]}"), {p["x"]:6.2f}f, {p["y"]:6.2f}f, {p["vx"]:5.2f}f, {p["vy"]:5.2f}f }},')
print("    };")

print("\n" + "=" * 70)
print("INITIAL POSITION CONSTANTS")
print("=" * 70)
print(f"    constexpr double  INIT_X_M  = {INIT_X:.2f};")
print(f"    constexpr double  INIT_Y_M  = {INIT_Y:.2f};")

cpp_ref_path = r"e:\puppyfangzhen\cpp_src\bridge\home_obstacles_array_v51.h"
with open(cpp_ref_path, 'w') as f:
    f.write("// Auto-generated by generate_home_layout.py v5.5\n")
    f.write("// Robot-navigation-friendly open-plan 16m x 12m apartment\n")
    f.write("// Fixed entry trap, double waypoints per door, return_west guide\n\n")
    f.write("static const FHomeObstacle HOME_OBSTACLES[] = {\n")
    for obs in all_obstacles:
        name_padded = obs["name"].ljust(24)
        xmin = obs["xmin"] * 100
        ymin = obs["ymin"] * 100
        xmax = obs["xmax"] * 100
        ymax = obs["ymax"] * 100
        f.write(f'    {{ TEXT("{name_padded}"), {xmin:7.1f}f,{ymin:7.1f}f,{xmax:7.1f}f,{ymax:7.1f}f }},\n')
    f.write("};\n")
    f.write("static const int NUM_HOME_OBSTACLES = UE_ARRAY_COUNT(HOME_OBSTACLES);\n")
    f.write("\n// Pedestrians (6, in open areas)\n")
    f.write("const FPedCfg Cfgs[6] = {\n")
    for p in pedestrians:
        f.write(f'    {{ TEXT("{p["name"]}"), {p["x"]:6.2f}f, {p["y"]:6.2f}f, {p["vx"]:5.2f}f, {p["vy"]:5.2f}f }},\n')
    f.write("};\n")
    f.write(f"\n// Initial position (charging dock)\n")
    f.write(f"constexpr double INIT_X_M = {INIT_X:.2f};\n")
    f.write(f"constexpr double INIT_Y_M = {INIT_Y:.2f};\n")
print(f"\nC++ reference saved: {cpp_ref_path}")
