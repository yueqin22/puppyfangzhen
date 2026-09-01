"""
Fix compilation errors in PuppyRobotPawn_patched.cpp:
1. Line endings -> CRLF (Windows)
2. GetActorLabel() -> GetName() (editor-only API)
3. Move SpawnObstacleVisuals() after HOME_OBSTACLES definition (forward reference issue)
"""
import os
import re

src = r'e:\puppyfangzhen\PuppyRobotPawn_patched.cpp'
dst = r'e:\puppyfangzhen\PuppyRobotPawn_patched_fixed.cpp'

with open(src, 'rb') as f:
    raw = f.read()

# Decode and normalize line endings to CRLF
text = raw.decode('utf-8-sig')  # handle BOM if present
text = text.replace('\r\n', '\n').replace('\r', '\n')  # normalize to LF first

# Fix 1: Replace GetActorLabel() with GetName()
# GetActorLabel is editor-only; GetName() works in all builds
text = text.replace('A->GetActorLabel()', 'A->GetName()')

# Fix 2: Move SpawnObstacleVisuals function to after HOME_OBSTACLES definition
# Find the SpawnObstacleVisuals function definition
func_start_marker = 'void APuppyRobotPawn::SpawnObstacleVisuals()'
func_end_marker = 'void APuppyRobotPawn::SpawnPedestriansFromScene()'

func_start = text.find(func_start_marker)
func_end = text.find(func_end_marker)

if func_start == -1 or func_end == -1:
    print(f"ERROR: Could not find function markers. start={func_start}, end={func_end}")
    exit(1)

# Extract the SpawnObstacleVisuals function
spawn_func = text[func_start:func_end].rstrip() + '\n\n'

# Remove the function from its current position
# Also remove the extra blank lines left behind
text_before_func = text[:func_start].rstrip() + '\n\n\n'
text_after_func = text[func_end:]
text = text_before_func + text_after_func

# Now find the position after NUM_HOME_OBSTACLES definition
# Insert SpawnObstacleVisuals after NUM_HOME_OBSTACLES line
num_marker = 'static const int NUM_HOME_OBSTACLES = UE_ARRAY_COUNT(HOME_OBSTACLES);'
num_pos = text.find(num_marker)
if num_pos == -1:
    print("ERROR: Could not find NUM_HOME_OBSTACLES")
    exit(1)

# Find end of this line
line_end = text.find('\n', num_pos)
if line_end == -1:
    line_end = len(text)

# Insert the function after NUM_HOME_OBSTACLES (with a blank line)
insert_pos = line_end + 1
text = text[:insert_pos] + '\n' + spawn_func + text[insert_pos:]

# Fix 3: Also need to move the FHomeObstacle struct before SpawnObstacleVisuals
# Actually, let's check - FHomeObstacle is defined right before HOME_OBSTACLES
# Since SpawnObstacleVisuals uses FHomeObstacle, it needs to see the struct definition too.
# But since we're moving the function after HOME_OBSTACLES (which is after FHomeObstacle),
# the struct will already be defined. Good.

# Convert to CRLF
text = text.replace('\n', '\r\n')

# Write the fixed file
with open(dst, 'wb') as f:
    f.write(text.encode('utf-8'))

print(f"Fixed file written to {dst}")
print(f"Size: {os.path.getsize(dst)} bytes")

# Verify fixes
with open(dst, 'rb') as f:
    verify = f.read().decode('utf-8')

has_crlf = '\r\n' in verify
has_getactorlabel = 'GetActorLabel()' in verify
func_pos = verify.find(func_start_marker)
obs_pos = verify.find('static const FHomeObstacle HOME_OBSTACLES[]')
num_pos_v = verify.find(num_marker)
func_after_obs = func_pos > num_pos_v

print(f"CRLF line endings: {has_crlf}")
print(f"GetActorLabel remaining: {has_getactorlabel}")
print(f"Func position: {func_pos}, HOME_OBSTACLES position: {obs_pos}, NUM_HOME_OBSTACLES position: {num_pos_v}")
print(f"Func defined after NUM_HOME_OBSTACLES: {func_after_obs}")
