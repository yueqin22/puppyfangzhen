#!/usr/bin/env python3
import sys, os
from PIL import Image
p = "/tmp/stream_frame.jpg"
if not os.path.exists(p):
    print("NO_FRAME_FILE")
    sys.exit(1)
st = os.stat(p)
print(f"size_bytes={st.st_size} mtime={int(st.st_mtime)}")
try:
    img = Image.open(p)
    ex = img.getextrema()
    print(f"img_size={img.size} extrema={ex}")
    # Determine if all black
    def is_black(e):
        if isinstance(e, tuple):
            return e[0] == 0 and e[1] == 0
        return e == 0
    if all(is_black(e) for e in ex):
        print("VERDICT: ALL_BLACK")
    else:
        print("VERDICT: HAS_CONTENT")
except Exception as e:
    print(f"ERROR: {e}")
