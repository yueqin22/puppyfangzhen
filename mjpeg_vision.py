#!/usr/bin/env python3
"""
MJPEG Stream Server using CoppeliaSim Vision Sensor.
This bypasses X11 screen capture entirely - renders directly via CoppeliaSim's
internal rendering pipeline, which works even in headless/software-rendered environments.
"""
import time, math, io, threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

# Connect to CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject("sim")

# Get robot handle
base = sim.getObject("/base_footprint")

# Create a vision sensor if it doesn't exist
# Try to find existing vision sensor first - if found, remove it (to recreate with correct options)
vs_handle = None
try:
    old_vs = sim.getObject("/patrol_camera")
    sim.removeObject(old_vs)
    print(f"Removed old vision sensor: {old_vs}")
except:
    pass

if vs_handle is None:
    # Create a new vision sensor
    # Per docs: intParams has 4 elements, floatParams has 11 elements
    # options bit 1 (2) = perspective mode
    try:
        angle_rad = 60.0 * math.pi / 180.0
        vs_handle = sim.createVisionSensor(
            3,  # options: bit0=explicitly handled, bit1=perspective mode
            [640, 480, 0, 0],  # intParams: res_x, res_y, reserved, reserved
            [0.01, 50.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0]  # floatParams: near, far, angle, size_x, res, res, r, g, b, res, res
        )
        sim.setObjectAlias(vs_handle, "patrol_camera")
        print(f"Created vision sensor: handle={vs_handle}")
    except Exception as e:
        print(f"createVisionSensor failed: {e}")

if vs_handle is None:
    print("ERROR: No vision sensor available!")
    exit(1)

# Set vision sensor properties
try:
    sim.setObjectPosition(vs_handle, sim.handle_world, [3.0, -3.0, 3.0])
    # Point camera at robot
    sim.setObjectOrientation(vs_handle, sim.handle_world, [-2.4, -0.4, 2.8])
except Exception as e:
    print(f"Set position failed: {e}")

print(f"Vision sensor ready: {vs_handle}")

# Shared state for camera tracking
current_frame = None
frame_lock = threading.Lock()
running = True

def update_camera_and_capture():
    """Continuously update camera position and capture frames"""
    global current_frame
    while running:
        try:
            # Get robot position and orientation
            rpos = sim.getObjectPosition(base, -1)
            rori = sim.getObjectOrientation(base, -1)
            
            # Camera follows robot from behind and above
            cam_dist = 4.0
            cam_height = 3.5
            cam_x = rpos[0] - math.cos(rori[2]) * cam_dist
            cam_y = rpos[1] - math.sin(rori[2]) * cam_dist
            
            sim.setObjectPosition(vs_handle, sim.handle_world, [cam_x, cam_y, cam_height])
            
            # Look at robot
            dx = rpos[0] - cam_x
            dy = rpos[1] - cam_y
            dz = rpos[2] - cam_height
            yaw = math.atan2(dy, dx)
            pitch = math.atan2(-dz, math.sqrt(dx*dx + dy*dy))
            sim.setObjectOrientation(vs_handle, sim.handle_world, [pitch, 0, yaw])
            
            # Trigger vision sensor rendering
            sim.handleVisionSensor(vs_handle)
            
            # Get the image - try getVisionSensorImg (newer API) first
            try:
                result = sim.getVisionSensorImg(vs_handle)
            except Exception:
                result = sim.getVisionSensorImage(vs_handle)
            
            # Handle variable return value length
            if isinstance(result, (list, tuple)) and len(result) >= 2:
                img_data = result[0]
                res = result[1]
            else:
                img_data = None
                res = None
            
            # Convert to JPEG
            if img_data and res and len(res) >= 2:
                w, h = res[0], res[1]
                # img_data is raw RGB bytes
                img = Image.frombytes("RGB", (w, h), img_data)
                # Flip vertically (CoppeliaSim image is bottom-up)
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                jpeg_buf = io.BytesIO()
                img.save(jpeg_buf, format="JPEG", quality=75)
                frame = jpeg_buf.getvalue()
                
                with frame_lock:
                    current_frame = frame
            
            time.sleep(0.1)  # 10 FPS
            
        except Exception as e:
            print(f"Capture error: {e}")
            time.sleep(1)

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            
            while running:
                with frame_lock:
                    frame = current_frame
                
                if frame:
                    try:
                        self.send_header("Content-type", "image/jpeg")
                        self.send_header("Content-length", str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n--frame\r\n")
                    except:
                        break
                time.sleep(0.1)
        elif self.path == "/snapshot":
            with frame_lock:
                frame = current_frame
            if frame:
                self.send_response(200)
                self.send_header("Content-type", "image/jpeg")
                self.send_header("Content-length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(b"No frame available")
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress logs

# Start capture thread
capture_thread = threading.Thread(target=update_camera_and_capture, daemon=True)
capture_thread.start()

# Start HTTP server
PORT = 8081
server = HTTPServer(("0.0.0.0", PORT), MJPEGHandler)
print(f"MJPEG Stream Server running on http://localhost:{PORT}/stream")
print(f"Snapshot URL: http://localhost:{PORT}/snapshot")
try:
    server.serve_forever()
except KeyboardInterrupt:
    running = False
    print("Server stopped")
