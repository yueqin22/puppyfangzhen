from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject("sim")
vs = sim.getObject("/patrol_camera")
print(f"Vision sensor handle: {vs}")
sim.setObjectPosition(vs, -1, [3.0, -3.0, 3.0])
sim.setObjectOrientation(vs, -1, [-2.4, -0.4, 2.8])
ret = sim.handleVisionSensor(vs)
print(f"handleVisionSensor returned: {ret}")
img, res = sim.getVisionSensorImage(vs)
print(f"res: {res}")
print(f"img type: {type(img)}, len: {len(img) if img else 0}")
if img:
    print(f"first 10 bytes: {img[:10]}")
    # Try saving as image
    from PIL import Image
    import io
    w, h = res[0], res[1]
    print(f"Image size: {w}x{h}")
    image = Image.frombytes("RGB", (w, h), img)
    image = image.transpose(Image.FLIP_TOP_BOTTOM)
    image.save("/tmp/vs_test.jpg", format="JPEG", quality=80)
    print("Saved /tmp/vs_test.jpg")
    import os
    print(f"File size: {os.path.getsize('/tmp/vs_test.jpg')} bytes")
