from PIL import Image
img = Image.open('/tmp/snap_test.jpg')
print(f"Size: {img.size}, Mode: {img.mode}")
px = list(img.getdata())
vals = [sum(p)//3 for p in px[::500]]
import statistics
print(f"Avg brightness: {statistics.mean(vals):.1f}")
print(f"Min/Max: {min(vals)}/{max(vals)}")
print(f"Unique colors (sampled): {len(set(px[::200]))}")
print(f"Sample colors: {list(set(px[::500]))[:5]}")
