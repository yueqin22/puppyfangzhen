"""
Fix script: update pedestrian positions and add SpawnObstacleVisuals() call in BeginPlay.
Run after patch_home_layout.py if those two items failed.
"""
import sys, os, shutil

CPP_TARGET = r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"

def safe_write(target_path, content_bytes):
    tmp = r"e:\puppyfangzhen\tmp_fix_out.tmp"
    with open(tmp, 'wb') as f:
        f.write(content_bytes)
    try:
        if os.path.exists(target_path):
            os.remove(target_path)
        shutil.copy2(tmp, target_path)
        os.remove(tmp)
        return True
    except Exception as e:
        print(f"  Copy failed: {e}")
        return False

def read_file(path):
    with open(path, 'rb') as f:
        raw = f.read()
    for enc in ['utf-8', 'utf-8-sig', 'gbk', 'latin-1']:
        try:
            return raw.decode(enc), enc
        except:
            continue
    return raw.decode('latin-1'), 'latin-1'

def write_file(path, text, enc='utf-8'):
    return safe_write(path, text.encode(enc))

# Read with binary to avoid encoding issues, then decode carefully
with open(CPP_TARGET, 'rb') as f:
    raw = f.read()

content = raw.decode('utf-8', errors='replace')
enc = 'utf-8'

changes = 0

# Fix 1: Update pedestrian positions - find the block by searching for the exact pattern
old_ped_lines = [
    '        { TEXT("person_1"),  2.0f, -2.5f, -0.04f,  0.00f },',
    '        { TEXT("person_2"), -2.5f,  2.8f,  0.00f,  0.04f },',
    '        { TEXT("person_3"), -3.0f, -2.0f,  0.03f,  0.00f },',
    '        { TEXT("person_4"), -1.0f, -1.0f,  0.04f,  0.00f },',
    '        { TEXT("person_5"),  1.5f, -1.5f,  0.04f,  0.00f }',
]
new_ped_lines = [
    '        { TEXT("person_1"), -0.50f, -2.50f, -0.03f,  0.00f },',
    '        { TEXT("person_2"),  1.20f, -0.90f,  0.00f,  0.03f },',
    '        { TEXT("person_3"),  3.50f,  0.50f,  0.02f,  0.00f },',
    '        { TEXT("person_4"), -3.50f,  3.00f,  0.00f, -0.02f },',
    '        { TEXT("person_5"), -0.80f,  2.50f,  0.02f,  0.00f }',
]

for old, new in zip(old_ped_lines, new_ped_lines):
    if old in content:
        content = content.replace(old, new, 1)
        print(f"  Updated: {old.strip()[:40]}...")
        changes += 1
    elif new in content:
        print(f"  Already updated: {new.strip()[:40]}...")
    else:
        print(f"  WARNING: Could not find: {old.strip()[:50]}")

# Fix 2: Add SpawnObstacleVisuals() call in BeginPlay
# Find the call to SpawnPedestriansFromScene() that is followed by a closing brace (end of BeginPlay)
spawn_call = "    SpawnPedestriansFromScene();"
if "SpawnObstacleVisuals();" not in content:
    # Find the LAST occurrence of SpawnPedestriansFromScene(); (the one in BeginPlay, not the function def)
    # We look for "    SpawnPedestriansFromScene();\n}" pattern
    idx = content.rfind(spawn_call + "\n}")
    if idx >= 0:
        insert_pos = idx + len(spawn_call)
        content = content[:insert_pos] + "\n    // Spawn 3D obstacle visuals from HOME_OBSTACLES so LiDAR can see them.\n    SpawnObstacleVisuals();" + content[insert_pos:]
        print("  Added SpawnObstacleVisuals() call after SpawnPedestriansFromScene() in BeginPlay")
        changes += 1
    else:
        print("  WARNING: Could not find SpawnPedestriansFromScene() call location")
else:
    print("  SpawnObstacleVisuals() call already present")

# Write back
if write_file(CPP_TARGET, content, enc):
    print(f"\nWritten {len(content)} chars with {changes} changes")
else:
    print("ERROR: Failed to write file")
    sys.exit(1)

# Verify
print("\n=== Verification ===")
checks = [
    ("person_1.*-0.50f.*-2.50f", "person_1 at living room"),
    ("person_2.*1.20f.*-0.90f", "person_2 at dining"),
    ("person_3.*3.50f.*0.50f", "person_3 at kitchen"),
    ("person_4.*-3.50f.*3.00f", "person_4 at master bedroom"),
    ("person_5.*-0.80f.*2.50f", "person_5 at bedroom2"),
    ("SpawnObstacleVisuals();", "SpawnObstacleVisuals() call present"),
]
import re
for pattern, desc in checks:
    if re.search(pattern, content):
        print(f"  OK: {desc}")
    else:
        print(f"  FAIL: {desc}")
