"""Fix line endings to CRLF and prepare final files for UE project."""
import os

# Fix cpp file
with open(r'e:\puppyfangzhen\PuppyRobotPawn_patched_fixed.cpp', 'rb') as f:
    cpp = f.read()
cpp_text = cpp.decode('utf-8')
cpp_text = cpp_text.replace('\r\n', '\n').replace('\r', '\n')
cpp_text = cpp_text.replace('\n', '\r\n')
cpp_fixed = cpp_text.encode('utf-8')

# Fix header file
with open(r'e:\puppyfangzhen\PuppyRobotPawn_patched.h', 'rb') as f:
    h = f.read()
h_text = h.decode('utf-8')
h_text = h_text.replace('\r\n', '\n').replace('\r', '\n')
h_text = h_text.replace('\n', '\r\n')
h_fixed = h_text.encode('utf-8')

# Write final files
with open(r'e:\puppyfangzhen\PuppyRobotPawn_final.cpp', 'wb') as f:
    f.write(cpp_fixed)
with open(r'e:\puppyfangzhen\PuppyRobotPawn_final.h', 'wb') as f:
    f.write(h_fixed)

print(f'CPP: {len(cpp_fixed)} bytes')
print(f'H:   {len(h_fixed)} bytes')

# Verify no GetActorLabel/SetActorLabel outside #if WITH_EDITOR
import re
# Find lines with GetActorLabel/SetActorLabel not inside WITH_EDITOR blocks
lines = cpp_text.split('\r\n')
in_editor_block = False
issues = []
for i, line in enumerate(lines, 1):
    if '#if WITH_EDITOR' in line:
        in_editor_block = True
    if '#endif' in line and in_editor_block:
        in_editor_block = False
    if ('GetActorLabel' in line or 'SetActorLabel' in line) and not in_editor_block:
        issues.append(f'Line {i}: {line.strip()}')
if issues:
    print('WARNING: Editor-only API used outside WITH_EDITOR:')
    for iss in issues:
        print(f'  {iss}')
else:
    print('No editor-only API issues found.')
