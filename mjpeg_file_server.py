#!/usr/bin/env python3
"""
Lightweight MJPEG Stream Server - reads frames from a shared JPEG file.
No ZMQ connection needed - just serves the file written by navigation scripts.
"""
import os
import tempfile
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

FRAME_PATH = os.environ.get(
    "PUPPY_STREAM_FRAME",
    os.path.join(tempfile.gettempdir(), "stream_frame.jpg"),
)
PORT = int(os.environ.get("PUPPY_STREAM_PORT", "8081"))

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_mtime = 0
            while True:
                try:
                    if os.path.exists(FRAME_PATH):
                        mtime = os.path.getmtime(FRAME_PATH)
                        if mtime != last_mtime:
                            with open(FRAME_PATH, "rb") as f:
                                frame = f.read()
                            if frame and len(frame) > 100:
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                                self.wfile.write(frame)
                                self.wfile.write(b"\r\n")
                                self.wfile.flush()
                                last_mtime = mtime
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    pass
                time.sleep(0.1)
        elif self.path == "/snapshot":
            try:
                if os.path.exists(FRAME_PATH):
                    with open(FRAME_PATH, "rb") as f:
                        frame = f.read()
                    if frame and len(frame) > 100:
                        self.send_response(200)
                        self.send_header("Content-type", "image/jpeg")
                        self.send_header("Content-length", str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        return
            except:
                pass
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b"No frame available")
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

server = ThreadingHTTPServer(("0.0.0.0", PORT), MJPEGHandler)
print(f"MJPEG Stream Server on http://localhost:{PORT}/stream")
print(f"Snapshot: http://localhost:{PORT}/snapshot")
try:
    server.serve_forever()
except KeyboardInterrupt:
    print("Server stopped")
