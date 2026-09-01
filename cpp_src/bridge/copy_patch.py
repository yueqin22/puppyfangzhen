"""
Copy patched files to UE project using temp-file-then-replace method.
"""
import os, sys, shutil, tempfile

pairs = [
    (r'e:\puppyfangzhen\PuppyRobotPawn_patched.cpp', r'D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp'),
    (r'e:\puppyfangzhen\PuppyRobotPawn_patched.h', r'D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.h'),
]

for src, dst in pairs:
    print(f"\nProcessing: {os.path.basename(dst)}")
    print(f"  Source: {src} ({os.path.getsize(src)} bytes)")

    if not os.path.exists(src):
        print(f"  ERROR: Source not found!")
        continue

    # Method 1: Write to temp in same dir, then os.replace
    dst_dir = os.path.dirname(dst)
    try:
        with open(src, 'rb') as f:
            data = f.read()

        # Create temp file in the target directory
        fd, tmp_path = tempfile.mkstemp(dir=dst_dir, prefix='_patch_', suffix='.tmp')
        try:
            os.write(fd, data)
            os.close(fd)
            # Replace original
            os.replace(tmp_path, dst)
            print(f"  SUCCESS via tempfile+replace: {os.path.getsize(dst)} bytes")
        except Exception as e1:
            os.close(fd) if not getattr(os, 'O_CLOEXEC', False) else None
            try:
                os.unlink(tmp_path)
            except:
                pass
            print(f"  Method 1 failed: {e1}")

            # Method 2: shutil.copy2
            try:
                shutil.copy2(src, dst)
                print(f"  SUCCESS via shutil.copy2: {os.path.getsize(dst)} bytes")
            except Exception as e2:
                print(f"  Method 2 failed: {e2}")

                # Method 3: Direct write
                try:
                    with open(dst, 'wb') as f:
                        f.write(data)
                    print(f"  SUCCESS via direct write: {os.path.getsize(dst)} bytes")
                except Exception as e3:
                    print(f"  Method 3 failed: {e3}")
    except Exception as e:
        print(f"  ERROR reading source: {e}")

print("\nDone.")
