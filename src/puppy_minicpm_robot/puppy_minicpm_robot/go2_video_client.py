"""Unitree Go2 Video Client wrapper for MiniCPM-RobotTrack.

Supports Unitree SDK2 VideoClient or standard OpenCV video streams / image capture.
"""

import time
import threading
from typing import Optional, Tuple
import numpy as np

from .inference_engine import crop_and_resize_image


class Go2VideoClient:
    """Interface to Unitree Go2 front camera."""

    def __init__(
        self,
        network_interface: str = "enP8p1s0",
        camera_ip: str = "192.168.123.161",
        target_size: int = 384,
        timeout_sec: float = 3.0
    ):
        self.network_interface = network_interface
        self.camera_ip = camera_ip
        self.target_size = target_size
        self.timeout_sec = timeout_sec

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._last_frame_time: float = 0.0
        self._is_running = False
        self._client = None
        self._init_client()

    def _init_client(self):
        """Try importing and initializing unitree_sdk2py VideoClient if present."""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.video.video_client import VideoClient
            ChannelFactoryInitialize(0, self.network_interface)
            self._client = VideoClient()
            self._client.SetTimeout(self.timeout_sec)
            self._client.Init()
            self._is_running = True
        except Exception:
            self._client = None
            self._is_running = False

    def get_frame(self) -> Tuple[bool, Optional[np.ndarray], float]:
        """Fetch latest fresh RGB frame."""
        now = time.time()
        if self._client is not None:
            try:
                code, data = self._client.GetImageSample()
                if code == 0 and len(data) > 0:
                    import cv2
                    np_arr = np.frombuffer(data, dtype=np.uint8)
                    bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    processed = crop_and_resize_image(rgb, self.target_size)
                    with self._lock:
                        self._latest_frame = processed
                        self._last_frame_time = now
                    return True, processed, now
            except Exception:
                pass

        with self._lock:
            if self._latest_frame is not None and (now - self._last_frame_time < self.timeout_sec):
                return True, self._latest_frame, self._last_frame_time

        return False, None, 0.0

    def set_mock_frame(self, frame: np.ndarray):
        """Set mock frame for testing."""
        with self._lock:
            self._latest_frame = crop_and_resize_image(frame, self.target_size)
            self._last_frame_time = time.time()
