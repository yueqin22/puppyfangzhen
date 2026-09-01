"""
Quick 2D visualization of the home layout v5.1 - Spacious 16m x 12m open plan
"""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

with open(r"e:\puppyfangzhen\config\scene_home.json", "r", encoding="utf-8") as f:
    scene = json.load(f)

obstacles = scene["obstacles"]
patrol = scene["patrol_targets"]
peds = scene["pedestrians"]

fig, ax = plt.subplots(1, 1, figsize=(20, 15))
fig.patch.set_facecolor('#1a1a2e')
ax.set_facecolor('#16213e')

rooms = {
    "ENTRY/FOYER": (0.0, -5.0),
    "LIVING ROOM": (-3.0, -3.0),
    "DINING AREA": (2.0, -0.8),
    "MASTER BED": (-5.5, 3.8),
    "STUDY/GUEST": (-1.3, 4.0),
    "BATHROOM": (2.3, 4.0),
    "KITCHEN": (6.0, 0.0),
    "HALLWAY": (-1.5, 1.2),
}

wall_color = '#5c677d'
outer_wall_color = '#7d8597'
furn_colors = {
    'sofa': '#4a90d9', 'coffee': '#6b8e23', 'tv': '#2d3436',
    'shoe': '#8b4513', 'bookshelf': '#6c5ce7', 'side_table': '#d4a574',
    'plant': '#00b894', 'dining': '#fdcb6e', 'chair': '#e17055',
    'sideboard': '#636e72', 'bed': '#e84393', 'pillow': '#fd79a8',
    'nightstand': '#6c5ce7', 'wardrobe': '#2d3436', 'dresser': '#636e72',
    'bench': '#d4a574', 'armchair': '#00b894', 'lamp': '#ffeaa7',
    'single_bed': '#00b894', 'desk': '#0984e3', 'chair_b': '#74b9ff',
    'toilet': '#dfe6e9', 'sink': '#b2bec3', 'bathtub': '#81ecec',
    'shelf': '#fab1a0', 'counter': '#e17055', 'stove': '#d63031',
    'sink_k': '#74b9ff', 'fridge': '#dfe6e9', 'hutch': '#b2bec3',
    'pantry': '#636e72', 'microwave': '#636e72', 'island': '#fdcb6e',
    'mirror': '#dfe6e9', 'bidet': '#b2bec3', 'laundry': '#b2bec3',
    'washbasin': '#81ecec',
}

def get_furn_color(name):
    if 'wall' in name:
        if name.startswith('wall_south') or name.startswith('wall_north') or name.startswith('wall_west') or name.startswith('wall_east'):
            return outer_wall_color
        return wall_color
    if 'sofa' in name: return furn_colors['sofa']
    if 'coffee' in name: return furn_colors['coffee']
    if 'tv' in name: return furn_colors['tv']
    if 'shoe' in name: return furn_colors['shoe']
    if 'bookshelf' in name: return furn_colors['bookshelf']
    if 'side_table' in name: return furn_colors['side_table']
    if 'plant' in name: return furn_colors['plant']
    if 'dining_table' in name: return furn_colors['dining']
    if 'chair_n' in name or 'chair_s' in name or 'chair_w' in name or 'chair_e' in name: return furn_colors['chair']
    if 'chair_b' in name: return furn_colors['chair_b']
    if 'sideboard' in name: return furn_colors['sideboard']
    if 'hutch' in name: return furn_colors['hutch']
    if 'master_bed' in name or 'mb_pillows' in name: return furn_colors['bed']
    if 'pillow' in name and 'mb' not in name: return furn_colors['pillow']
    if 'nightstand' in name: return furn_colors['nightstand']
    if 'wardrobe' in name: return furn_colors['wardrobe']
    if 'dresser' in name: return furn_colors['dresser']
    if 'bed_bench' in name: return furn_colors['bench']
    if 'mb_chair' in name: return furn_colors['armchair']
    if 'floor_lamp' in name: return furn_colors['lamp']
    if 'armchair' in name: return furn_colors['armchair']
    if 'single_bed' in name or 'sb_pillow' in name: return furn_colors['single_bed']
    if 'desk' in name: return furn_colors['desk']
    if 'desk_chair' in name: return furn_colors['chair_b']
    if 'toilet' in name: return furn_colors['toilet']
    if 'bidet' in name: return furn_colors['bidet']
    if 'laundry' in name: return furn_colors['laundry']
    if 'bath_sink' in name or 'kitchen_sink' in name or 'sink' in name: return furn_colors['sink_k']
    if 'bathtub' in name: return furn_colors['bathtub']
    if 'shelf' in name or 'bookshelf' in name: return furn_colors['shelf']
    if 'counter' in name: return furn_colors['counter']
    if 'stove' in name: return furn_colors['stove']
    if 'fridge' in name: return furn_colors['fridge']
    if 'pantry' in name: return furn_colors['pantry']
    if 'microwave' in name: return furn_colors['microwave']
    if 'kitchen_island' in name: return furn_colors['island']
    if 'mirror' in name: return furn_colors['mirror']
    if 'washbasin' in name: return furn_colors['washbasin']
    if 'sb_nightstand' in name: return furn_colors['nightstand']
    if 'wardrobe_b2' in name: return furn_colors['wardrobe']
    return '#95a5a6'

for obs in obstacles:
    x = obs["xmin"]
    y = obs["ymin"]
    w = obs["xmax"] - obs["xmin"]
    h = obs["ymax"] - obs["ymin"]
    color = get_furn_color(obs["name"])
    alpha = 0.9 if 'wall' in obs["name"] else 0.85
    ec = '#333' if 'wall' in obs["name"] else '#222'
    lw = 2.5 if 'wall' in obs["name"] else 0.8
    zorder = 2 if 'wall' in obs["name"] else 3
    rect = patches.FancyBboxPatch((x, y), w, h, 
                                   boxstyle="round,pad=0.03",
                                   facecolor=color, edgecolor=ec, linewidth=lw, alpha=alpha, zorder=zorder)
    ax.add_patch(rect)

# Draw floor areas as subtle colored regions
floor_areas = [
    # Living/dining open area (south of h1)
    {'x1': -7.8, 'y1': -5.8, 'x2': 4.14, 'y2': 2.14, 'color': '#2d3436', 'alpha': 0.08},
    # Master bed (northwest)
    {'x1': -7.8, 'y1': 2.26, 'x2': -3.06, 'y2': 5.8, 'color': '#6c5ce7', 'alpha': 0.1},
    # Study/guest (north-center)
    {'x1': -2.94, 'y1': 2.26, 'x2': 0.44, 'y2': 5.8, 'color': '#00b894', 'alpha': 0.1},
    # Bathroom (northeast)
    {'x1': 0.56, 'y1': 2.26, 'x2': 4.14, 'y2': 5.8, 'color': '#81ecec', 'alpha': 0.1},
    # Kitchen (east, full height)
    {'x1': 4.26, 'y1': -5.8, 'x2': 7.8, 'y2': 5.8, 'color': '#e17055', 'alpha': 0.08},
]
for fa in floor_areas:
    rect = patches.Rectangle((fa['x1'], fa['y1']), fa['x2']-fa['x1'], fa['y2']-fa['y1'],
                              facecolor=fa['color'], alpha=fa['alpha'], edgecolor='none', zorder=1)
    ax.add_patch(rect)

# Patrol points and path
pt_coords = []
for i, pt in enumerate(patrol):
    x, y = pt["x"], pt["y"]
    pt_coords.append((x, y))
    color = '#ffeaa7' if pt["name"] == "dock" else '#ff7675'
    ax.plot(x, y, 'o', color=color, markersize=12, zorder=6, markeredgecolor='white', markeredgewidth=2)
    short_name = pt["name"].replace("living_room_", "LR-").replace("hallway", "HALL")
    ax.annotate(f'{i+1}.{short_name}', (x, y), textcoords="offset points", xytext=(10, 10),
                fontsize=7, color='white', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='#2d3436', alpha=0.9, edgecolor='none'))

if pt_coords:
    px = [p[0] for p in pt_coords] + [pt_coords[0][0]]
    py = [p[1] for p in pt_coords] + [pt_coords[0][1]]
    ax.plot(px, py, '--', color='#74b9ff', linewidth=2.5, alpha=0.6, zorder=4)

for p in peds:
    ax.plot(p["x"], p["y"], 's', color='#fd79a8', markersize=11, zorder=7,
            markeredgecolor='white', markeredgewidth=1.5)
    ax.annotate(p["name"], (p["x"], p["y"]), textcoords="offset points", xytext=(0, -18),
                fontsize=6.5, color='#fd79a8', ha='center', fontweight='bold')

for name, (x, y) in rooms.items():
    ax.text(x, y, name, fontsize=11, color='#dfe6e9', ha='center', va='center',
            fontweight='bold', alpha=0.5, family='monospace',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#2d3436', alpha=0.3, edgecolor='none'))

INIT_X = scene["robot"]["initial_pose"]["x"]
INIT_Y = scene["robot"]["initial_pose"]["y"]
ax.plot(INIT_X, INIT_Y, '*', color='#00cec9', markersize=24, zorder=8,
        markeredgecolor='white', markeredgewidth=2.5)
ax.annotate('ROBOT START\n(DOCK)', (INIT_X, INIT_Y), textcoords="offset points", xytext=(0, -28),
            fontsize=8, color='#00cec9', ha='center', fontweight='bold', family='monospace')

ax.set_xlim(-8.5, 8.5)
ax.set_ylim(-6.5, 6.5)
ax.set_aspect('equal')
ax.set_xlabel('X (meters)', color='white', fontsize=13)
ax.set_ylabel('Y (meters)', color='white', fontsize=13)
ax.set_title('PuppyPi Home Layout v5.1 - Spacious Open Plan Apartment (16m × 12m)', 
             color='white', fontsize=16, fontweight='bold', pad=20)
ax.tick_params(colors='white', labelsize=10)
ax.grid(True, alpha=0.1, color='white')
for spine in ax.spines.values():
    spine.set_color('#5c677d')

# Legend
legend_items = [
    ('Outer Wall', outer_wall_color, 's'),
    ('Inner Wall', wall_color, 's'),
    ('Sofa/Lounge', furn_colors['sofa'], 's'),
    ('Bed', furn_colors['bed'], 's'),
    ('Dining/Table', furn_colors['dining'], 's'),
    ('Kitchen', furn_colors['counter'], 's'),
    ('Bathroom', furn_colors['bathtub'], 's'),
    ('Patrol Point', '#ff7675', 'o'),
    ('Dock/Charger', '#ffeaa7', 'o'),
    ('Pedestrian', '#fd79a8', 's'),
    ('Robot Start', '#00cec9', '*'),
]
for i, (label, color, marker) in enumerate(legend_items):
    y_pos = 5.5 - i * 0.5
    ms = 10 if marker != '*' else 14
    ax.plot(-8.0, y_pos, marker, color=color, markersize=ms if marker != '*' else 14, 
            markeredgecolor='white', markeredgewidth=0.5)
    ax.text(-7.3, y_pos, label, fontsize=9, color='#dfe6e9', va='center', family='monospace')

# North arrow
ax.annotate('N', xy=(7.0, 5.2), fontsize=16, color='white', ha='center', fontweight='bold', family='monospace')
ax.annotate('', xy=(7.0, 5.8), xytext=(7.0, 4.6),
            arrowprops=dict(arrowstyle='->', color='white', lw=3))

# Scale bar
ax.plot([5.0, 7.0], [-6.0, -6.0], '-', color='white', lw=5, solid_capstyle='butt')
ax.text(6.0, -6.3, '2 meters', fontsize=10, color='white', ha='center', family='monospace')

# Door markers (cyan)
door_markings = [
    # Entry door (south wall, x=-0.6 to 0.6)
    (-0.6, -6.0, 1.2, 0.2, 'ENTRY'),
    # Master bedroom door (h1, x=-4.3 to -3.1)
    (-4.3, 2.0, 1.2, 0.2, 'MB'),
    # Second bedroom door (h1, x=-0.8 to 0.4)
    (-0.8, 2.0, 1.2, 0.2, 'B2'),
    # Bathroom door (h1, x=1.6 to 2.8)
    (1.6, 2.0, 1.2, 0.2, 'BA'),
    # Kitchen pass-through (open from 4.26 to 7.8) - no door needed, mark with arc
]
for dx, dy, dw, dh, label in door_markings:
    rect = patches.FancyBboxPatch((dx, dy), dw, dh, boxstyle="round,pad=0.02",
                                   facecolor='#00cec9', edgecolor='#00cec9', 
                                   alpha=0.9, zorder=5, linewidth=0)
    ax.add_patch(rect)
    if label == 'ENTRY':
        ax.text(0.0, -5.7, 'MAIN ENTRY', fontsize=8, color='#00cec9', ha='center', fontweight='bold', family='monospace')

# Kitchen open pass indicator
ax.annotate('OPEN TO KITCHEN', xy=(6.0, 2.0), fontsize=9, color='#00cec9', ha='center', 
            fontweight='bold', family='monospace', alpha=0.8,
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#2d3436', alpha=0.7, edgecolor='#00cec9'))

plt.tight_layout()
out_path = r"e:\puppyfangzhen\screenshots\layout_v51_2d.png"
plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f"Layout v5.1 saved to: {out_path}")
plt.close()
