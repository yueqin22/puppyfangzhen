"""Copy final patched files to UE project using Python shutil."""
import shutil
import os

src_cpp = r'e:\puppyfangzhen\PuppyRobotPawn_final.cpp'
src_h = r'e:\puppyfangzhen\PuppyRobotPawn_final.h'
dst_cpp = r'D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp'
dst_h = r'D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.h'

for src, dst in [(src_cpp, dst_cpp), (src_h, dst_h)]:
    try:
        # Try direct copy with shutil
        shutil.copy2(src, dst)
        sz = os.path.getsize(dst)
        print(f'OK: {dst} ({sz} bytes)')
    except Exception as e:
        print(f'shutil failed: {e}')
        # Fallback: read and write manually
        with open(src, 'rb') as f:
            data = f.read()
        with open(dst, 'wb') as f:
            f.write(data)
        sz = os.path.getsize(dst)
        print(f'OK (fallback): {dst} ({sz} bytes)')

# Verify
for dst in [dst_cpp, dst_h]:
    if os.path.exists(dst):
        print(f'Verified: {dst} - {os.path.getsize(dst)} bytes')
    else:
        print(f'ERROR: {dst} not found!')
