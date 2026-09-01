"""
Read PuppyRobotPawn.cpp from D:, apply patches, write the patched version to E:
Then we can try copying it back.
"""
import re

src_path = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"
dst_path = r"e:\puppyfangzhen\cpp_src\bridge\PuppyRobotPawn_patched_v2.cpp"

with open(src_path, 'r', encoding='utf-8') as f:
    content = f.read()

print(f"Read {len(content)} chars from {src_path}")

# 1. Update INIT_Y_M from -2.7 to -3.0
old_init = "constexpr double  INIT_Y_M  = -2.7;   // scene_home.robot.initial_pose.y"
new_init = "constexpr double  INIT_Y_M  = -3.0;   // scene_home.robot.initial_pose.y"
if old_init in content:
    content = content.replace(old_init, new_init)
    print("[OK] Updated INIT_Y_M to -3.0")
else:
    print("[WARN] INIT_Y_M exact match not found")
    content = re.sub(
        r'constexpr double\s+INIT_Y_M\s*=\s*-2\.7;',
        'constexpr double  INIT_Y_M  = -3.0;   // scene_home.robot.initial_pose.y',
        content
    )
    print("[OK] Updated INIT_Y_M via regex")

# 2. Replace HOME_OBSTACLES array
new_obstacles = """static const FHomeObstacle HOME_OBSTACLES[] = {
    { TEXT("wall_y0_w1              "), -500.0f, -15.0f,-350.0f,  15.0f },
    { TEXT("wall_y0_w2              "), -200.0f, -15.0f,   0.0f,  15.0f },
    { TEXT("wall_y0_w3              "),  100.0f, -15.0f, 200.0f,  15.0f },
    { TEXT("wall_y0_w4              "),  350.0f, -15.0f, 500.0f,  15.0f },
    { TEXT("wall_y2_w1              "), -500.0f, 185.0f,-350.0f, 215.0f },
    { TEXT("wall_y2_w2              "), -200.0f, 185.0f,   0.0f, 215.0f },
    { TEXT("wall_y2_w3              "),  100.0f, 185.0f, 150.0f, 215.0f },
    { TEXT("wall_y2_w4              "),  250.0f, 185.0f, 350.0f, 215.0f },
    { TEXT("wall_y2_w5              "),  450.0f, 185.0f, 500.0f, 215.0f },
    { TEXT("wall_v_bed1_door_bot    "), -355.0f,   0.0f,-345.0f,  40.0f },
    { TEXT("wall_v_bed1_door_top    "), -355.0f, 160.0f,-345.0f, 200.0f },
    { TEXT("wall_v_bed2_bath_bot    "), -105.0f, 200.0f, -95.0f, 240.0f },
    { TEXT("wall_v_bed2_bath_top    "), -105.0f, 360.0f, -95.0f, 400.0f },
    { TEXT("wall_v_bath_store_bot   "),   95.0f, 200.0f, 105.0f, 240.0f },
    { TEXT("wall_v_bath_store_top   "),   95.0f, 360.0f, 105.0f, 400.0f },
    { TEXT("wall_v_dining_kitchen   "),  245.0f, 110.0f, 255.0f, 200.0f },
    { TEXT("wall_v_kitchen_east     "),  345.0f,   0.0f, 355.0f, 200.0f },
    { TEXT("sofa                    "),  180.0f,-335.0f, 340.0f,-285.0f },
    { TEXT("tv_console              "), -360.0f,-330.0f,-220.0f,-295.0f },
    { TEXT("side_table              "),   50.0f,-320.0f, 100.0f,-280.0f },
    { TEXT("bookshelf_w             "), -490.0f,-260.0f,-450.0f,-120.0f },
    { TEXT("shoe_cabinet            "), -490.0f,-330.0f,-430.0f,-290.0f },
    { TEXT("master_bed              "), -495.0f,  45.0f,-405.0f, 150.0f },
    { TEXT("nightstand_mb           "), -495.0f,   5.0f,-455.0f,  40.0f },
    { TEXT("wardrobe                "), -470.0f, 175.0f,-360.0f, 210.0f },
    { TEXT("desk                    "), -470.0f, 300.0f,-370.0f, 340.0f },
    { TEXT("bookshelf_nw            "), -490.0f, 355.0f,-430.0f, 390.0f },
    { TEXT("filing_cabinet          "), -490.0f, 225.0f,-450.0f, 265.0f },
    { TEXT("single_bed              "), -305.0f, 275.0f,-210.0f, 380.0f },
    { TEXT("nightstand_2            "), -345.0f, 290.0f,-305.0f, 325.0f },
    { TEXT("dresser                 "), -190.0f, 350.0f,-110.0f, 390.0f },
    { TEXT("bathtub                 "),  -60.0f, 350.0f,  60.0f, 390.0f },
    { TEXT("toilet                  "),  -85.0f, 255.0f, -35.0f, 300.0f },
    { TEXT("sink                    "),   25.0f, 255.0f,  90.0f, 300.0f },
    { TEXT("shelves_n               "),  130.0f, 350.0f, 310.0f, 390.0f },
    { TEXT("shelves_e               "),  320.0f, 270.0f, 355.0f, 390.0f },
    { TEXT("cabinet_s               "),  115.0f, 225.0f, 165.0f, 265.0f },
    { TEXT("dining_table            "),  120.0f,  85.0f, 200.0f, 135.0f },
    { TEXT("chair_n                 "),  145.0f, 140.0f, 175.0f, 165.0f },
    { TEXT("chair_s                 "),  145.0f,  55.0f, 175.0f,  80.0f },
    { TEXT("chair_w                 "),  100.0f,  95.0f, 115.0f, 125.0f },
    { TEXT("chair_e                 "),  205.0f,  95.0f, 220.0f, 125.0f },
    { TEXT("sideboard               "),  110.0f,  20.0f, 190.0f,  50.0f },
    { TEXT("counter_n               "),  265.0f, 165.0f, 430.0f, 210.0f },
    { TEXT("counter_e               "),  430.0f,  30.0f, 490.0f, 210.0f },
    { TEXT("fridge                  "),  265.0f, 110.0f, 310.0f, 160.0f },
};"""

pattern = r'static const FHomeObstacle HOME_OBSTACLES\[\] = \{[^;]*?\};'
match = re.search(pattern, content, re.DOTALL)
if match:
    content = content[:match.start()] + new_obstacles + content[match.end():]
    print(f"[OK] Replaced HOME_OBSTACLES array")
else:
    print("[ERROR] Could not find HOME_OBSTACLES array!")

# 3. Replace pedestrian Cfgs array
new_peds = """    const FPedCfg Cfgs[5] = {

        { TEXT("person_1"),  2.50f, -1.50f, -0.03f,  0.00f },

        { TEXT("person_2"), -1.50f,  1.00f,  0.00f,  0.03f },

        { TEXT("person_3"), -3.00f, -1.00f,  0.02f,  0.00f },

        { TEXT("person_4"),  0.00f, -2.50f,  0.03f,  0.00f },

        { TEXT("person_5"),  0.50f,  0.80f, -0.02f,  0.02f }

    };"""

ped_pattern = r'const FPedCfg Cfgs\[5\] = \{[^;]*?\};'
ped_match = re.search(ped_pattern, content, re.DOTALL)
if ped_match:
    content = content[:ped_match.start()] + new_peds + content[ped_match.end():]
    print("[OK] Replaced pedestrian Cfgs array")
else:
    print("[ERROR] Could not find pedestrian Cfgs array!")

# Write patched file to E: drive
with open(dst_path, 'w', encoding='utf-8') as f:
    f.write(content)
print(f"\nPatched file written to: {dst_path}")
print(f"File size: {len(content)} chars")

# Verify
checks = [
    ("INIT_Y_M = -3.0", "INIT_Y_M  = -3.0" in content),
    ("wall_y0_w1 present", "wall_y0_w1" in content),
    ("sofa present", '"sofa' in content),
    ("Old wall_north boundary removed", 'wall_north        "' not in content),
    ("person_5 at (0.50, 0.80)", '0.50f,  0.80f' in content),
    ("fridge present", '"fridge' in content),
    ("bathtub present", '"bathtub' in content),
    ("46 obstacle entries", content.count('TEXT("') - content.count('TEXT("person_') == 46),
]
print("\n=== VERIFICATION ===")
all_ok = True
for name, result in checks:
    print(f"  [{'PASS' if result else 'FAIL'}] {name}")
    if not result:
        all_ok = False

if all_ok:
    print("\n*** PATCHED FILE READY ON E: DRIVE ***")
else:
    print("\n*** SOME CHECKS FAILED ***")
