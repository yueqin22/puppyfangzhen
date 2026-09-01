"""
Apply v4.2 home layout to PuppyRobotPawn.cpp:
1. Replace HOME_OBSTACLES array (55 items - 20 walls + 35 furniture)
2. Update INIT_X_M to 0.05, INIT_Y_M to -2.98
3. Replace pedestrian Cfgs array
4. Output patched file to E: drive for manual copy (due to D: permission)
"""
import os
import re
import shutil

src_cpp = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"
out_path = r"E:\puppyfangzhen\cpp_src\bridge\PuppyRobotPawn_patched_v42.cpp"

with open(src_cpp, 'r', encoding='utf-8') as f:
    content = f.read()
print(f"Read source: {src_cpp} ({len(content)} bytes)")

# 1. Replace HOME_OBSTACLES array
new_obstacles = """static const FHomeObstacle HOME_OBSTACLES[] = {
    { TEXT("wall_south_w1           "),  -500.0f, -400.0f,  -45.0f, -380.0f },
    { TEXT("wall_south_w2           "),    45.0f, -400.0f,  500.0f, -380.0f },
    { TEXT("wall_north              "),  -500.0f,  380.0f,  500.0f,  400.0f },
    { TEXT("wall_west               "),  -500.0f, -400.0f, -480.0f,  400.0f },
    { TEXT("wall_east               "),   480.0f, -400.0f,  500.0f,  400.0f },
    { TEXT("wall_h1_w1              "),  -480.0f,   -6.0f, -370.0f,    6.0f },
    { TEXT("wall_h1_w1b             "),  -280.0f,   -6.0f, -266.0f,    6.0f },
    { TEXT("wall_v_mbr_e            "),  -266.0f,   -6.0f, -254.0f,  380.0f },
    { TEXT("wall_h1_w2              "),  -254.0f,   -6.0f,  -80.0f,    6.0f },
    { TEXT("wall_h1_w3              "),    20.0f,   -6.0f,  200.0f,    6.0f },
    { TEXT("wall_h1_w4              "),   294.0f,   -6.0f,  480.0f,    6.0f },
    { TEXT("wall_v_kitchen_w        "),   294.0f,   -6.0f,  306.0f,  380.0f },
    { TEXT("wall_h2_w1              "),  -254.0f,  214.0f, -150.0f,  226.0f },
    { TEXT("wall_h2_w2              "),   -60.0f,  214.0f,   -6.0f,  226.0f },
    { TEXT("wall_v_bed2_bath        "),    -6.0f,  214.0f,    6.0f,  380.0f },
    { TEXT("wall_h2_w3              "),     6.0f,  214.0f,   50.0f,  226.0f },
    { TEXT("wall_h2_w4              "),   140.0f,  214.0f,  164.0f,  226.0f },
    { TEXT("wall_v_bath_stor        "),   164.0f,  214.0f,  176.0f,  380.0f },
    { TEXT("wall_h2_w5              "),   176.0f,  214.0f,  200.0f,  226.0f },
    { TEXT("wall_h2_w6              "),   288.0f,  214.0f,  294.0f,  226.0f },
    { TEXT("sofa_3seat              "),    80.0f, -370.0f,  300.0f, -310.0f },
    { TEXT("sofa_chaise             "),   240.0f, -370.0f,  310.0f, -220.0f },
    { TEXT("coffee_table            "),   120.0f, -230.0f,  220.0f, -170.0f },
    { TEXT("tv_console              "),  -300.0f,  -55.0f, -100.0f,  -25.0f },
    { TEXT("tv_set                  "),  -240.0f,  -95.0f, -160.0f,  -55.0f },
    { TEXT("shoe_cabinet            "),   -80.0f, -375.0f,   30.0f, -330.0f },
    { TEXT("bookshelf_w             "),  -475.0f, -280.0f, -430.0f,  -80.0f },
    { TEXT("side_table              "),   315.0f, -300.0f,  355.0f, -260.0f },
    { TEXT("plant_sw                "),  -455.0f, -370.0f, -405.0f, -320.0f },
    { TEXT("dining_table            "),   330.0f, -180.0f,  430.0f, -100.0f },
    { TEXT("chair_n                 "),   360.0f,  -90.0f,  400.0f,  -65.0f },
    { TEXT("chair_s                 "),   360.0f, -215.0f,  400.0f, -190.0f },
    { TEXT("chair_w                 "),   310.0f, -155.0f,  330.0f, -120.0f },
    { TEXT("chair_e                 "),   430.0f, -155.0f,  460.0f, -120.0f },
    { TEXT("sideboard               "),   430.0f, -300.0f,  475.0f, -200.0f },
    { TEXT("master_bed              "),  -470.0f,  120.0f, -320.0f,  250.0f },
    { TEXT("mb_pillows              "),  -470.0f,  220.0f, -320.0f,  250.0f },
    { TEXT("nightstand_s            "),  -470.0f,   50.0f, -420.0f,   95.0f },
    { TEXT("nightstand_n            "),  -470.0f,  270.0f, -420.0f,  315.0f },
    { TEXT("wardrobe                "),  -460.0f,  315.0f, -280.0f,  370.0f },
    { TEXT("dresser_mb              "),  -272.0f,   30.0f, -260.0f,  120.0f },
    { TEXT("single_bed              "),  -240.0f,  300.0f, -110.0f,  370.0f },
    { TEXT("desk                    "),   -30.0f,  250.0f,   -6.0f,  320.0f },
    { TEXT("office_chair            "),   -75.0f,  265.0f,  -35.0f,  305.0f },
    { TEXT("bookshelf_b2            "),  -254.0f,  240.0f, -240.0f,  290.0f },
    { TEXT("toilet                  "),    15.0f,  235.0f,   55.0f,  285.0f },
    { TEXT("sink                    "),   120.0f,  240.0f,  165.0f,  290.0f },
    { TEXT("bathtub                 "),    60.0f,  310.0f,  165.0f,  370.0f },
    { TEXT("shelf_n                 "),   185.0f,  330.0f,  290.0f,  370.0f },
    { TEXT("shelf_e                 "),   260.0f,  240.0f,  294.0f,  320.0f },
    { TEXT("counter_n               "),   315.0f,  300.0f,  470.0f,  360.0f },
    { TEXT("counter_e               "),   420.0f,   20.0f,  475.0f,  295.0f },
    { TEXT("stove                   "),   330.0f,  310.0f,  380.0f,  350.0f },
    { TEXT("kitchen_sink            "),   425.0f,  200.0f,  470.0f,  250.0f },
    { TEXT("fridge                  "),   315.0f,   20.0f,  370.0f,  100.0f },
};"""

pattern = r'static const FHomeObstacle HOME_OBSTACLES\[\] = \{[^;]*?\};'
match = re.search(pattern, content, re.DOTALL)
if match:
    old_array = match.group(0)
    content = content.replace(old_array, new_obstacles)
    print("[OK] Replaced HOME_OBSTACLES array (55 items)")
else:
    print("[ERROR] Could not find HOME_OBSTACLES array!")

# 2. Update INIT_X_M and INIT_Y_M
# Try to find existing patterns
for pattern, replacement in [
    (r'constexpr double\s+INIT_X_M\s*=\s*[-\d.]+;', 'constexpr double  INIT_X_M  = 0.05;'),
    (r'constexpr double\s+INIT_Y_M\s*=\s*[-\d.]+;', 'constexpr double  INIT_Y_M  = -2.98;'),
]:
    content = re.sub(pattern, replacement, content)
print("[OK] Updated INIT_X_M=0.05, INIT_Y_M=-2.98")

# 3. Replace pedestrian Cfgs array
new_pedestrians = """    const FPedCfg Cfgs[5] = {
        { TEXT("person_1"),   0.50f,  -1.50f, -0.02f,  0.01f },
        { TEXT("person_2"),  -1.00f,   1.30f,  0.01f,  0.02f },
        { TEXT("person_3"),  -3.20f,  -2.00f,  0.02f,  0.00f },
        { TEXT("person_4"),   0.00f,  -1.50f,  0.02f, -0.01f },
        { TEXT("person_5"),   3.50f,   1.50f, -0.02f,  0.02f },
    };"""

ped_pattern = r'const FPedCfg Cfgs\[5\] = \{[^;]*?\};'
ped_match = re.search(ped_pattern, content, re.DOTALL)
if ped_match:
    old_peds = ped_match.group(0)
    content = content.replace(old_peds, new_pedestrians)
    print("[OK] Replaced pedestrian Cfgs array")
else:
    print("[WARN] Could not find pedestrian Cfgs array via regex, trying alternate...")
    # Try multi-line pattern with possible whitespace
    ped_pattern2 = r'const FPedCfg Cfgs\[5\]\s*=\s*\{.*?\};'
    ped_match2 = re.search(ped_pattern2, content, re.DOTALL)
    if ped_match2:
        content = content.replace(ped_match2.group(0), new_pedestrians)
        print("[OK] Replaced pedestrian Cfgs array (alt pattern)")
    else:
        print("[ERROR] Could not find pedestrian Cfgs array!")

# Write output to E: drive
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, 'w', encoding='utf-8') as f:
    f.write(content)
print(f"\n[OK] Patched file written to: {out_path}")
print(f"     File size: {len(content)} bytes")

# Verify
print("\n=== VERIFICATION ===")
checks = [
    ("wall_south_w1 (new south wall)", 'wall_south_w1' in content),
    ("wall_north (full outer wall)", '"wall_north' in content),
    ("wall_east (full outer wall)", '"wall_east' in content),
    ("sofa_3seat (L-sofa main)", '"sofa_3seat' in content),
    ("dining_table (SE area)", '"dining_table' in content),
    ("bathtub (bathroom)", '"bathtub' in content),
    ("fridge (kitchen)", '"fridge' in content),
    ("INIT_X_M = 0.05", 'INIT_X_M  = 0.05' in content),
    ("INIT_Y_M = -2.98", 'INIT_Y_M  = -2.98' in content),
    ("person_1 at (0.50, -1.50)", '0.50f,  -1.50f' in content),
    ("person_5 at (3.50, 1.50)", '3.50f,   1.50f' in content),
    ("NUM_HOME_OBSTACLES present", 'NUM_HOME_OBSTACLES' in content),
]
all_ok = True
for name, result in checks:
    print(f"  [{'PASS' if result else 'FAIL'}] {name}")
    if not result:
        all_ok = False

if all_ok:
    print("\n*** ALL PATCHES APPLIED SUCCESSFULLY ***")
    print(f"*** Copy {out_path} to D:\\puppy_ue\\Source\\PuppyNav\\PuppyRobotPawn.cpp ***")
else:
    print("\n*** SOME CHECKS FAILED - review needed ***")
