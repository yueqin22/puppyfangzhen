"""Inference engine abstraction for MiniCPM-RobotTrack."""

import abc
import json
import math
import time
import urllib.request
import urllib.error
from typing import Dict, Any, Optional, Tuple, List
import numpy as np

from .types import TrackIntent, InferenceState, TrackMode, WaypointStrategy


# --------------------------------------------------------------------------
# OpenCV availability probe (resolved ONCE, then cached)
# --------------------------------------------------------------------------
# Why this exists: OpenCV may be installed but *unusable* -- e.g. the distro
# package python3-opencv (built against numpy 1.x) while pip has installed
# numpy 2.x. In that case `import cv2` succeeds but the first call raises
# `AttributeError: _ARRAY_API not found`, and -- worse -- the failing import
# makes numpy print a multi-line diagnostic plus a traceback straight to
# stderr on EVERY attempt. Probing per frame therefore floods the launch log
# (hundreds of MB over a long run) and burns CPU, even when a numpy fallback
# path is in place. Probe once, cache the outcome, never probe again.
_CV2 = None
_CV2_PROBED = False


def _get_cv2():
    """Return a *usable* cv2 module, or None if OpenCV is missing/broken.

    The result is cached after the first call so the (noisy, expensive)
    failure path can happen at most once per process.
    """
    global _CV2, _CV2_PROBED
    if not _CV2_PROBED:
        _CV2_PROBED = True
        try:
            import cv2 as _cv2
            # Importing is not enough: verify it actually works. A cv2 built
            # against numpy 1.x under numpy 2.x imports fine and only blows
            # up on first use.
            _cv2.resize(np.zeros((4, 4, 3), dtype=np.uint8), (2, 2))
            _CV2 = _cv2
        except Exception:
            _CV2 = None
    return _CV2


def crop_and_resize_image(
    image: np.ndarray,
    target_size: int = 384,
    mode: str = "center_crop_height"
) -> np.ndarray:
    """Preprocess image to standard 384x384 input matching MiniCPM-RobotTrack upstream."""
    if image is None or image.size == 0:
        return np.zeros((target_size, target_size, 3), dtype=np.uint8)

    h, w = image.shape[:2]
    if h == target_size and w == target_size:
        return image

    if mode == "center_crop_height":
        if w > h:
            offset = (w - h) // 2
            cropped = image[:, offset:offset + h]
        elif h > w:
            offset = (h - w) // 2
            cropped = image[offset:offset + w, :]
        else:
            cropped = image
    else:
        cropped = image

    # Resize via cv2 when usable, otherwise pure-numpy nearest neighbour.
    # NOTE: cv2 is probed through the cached helper -- never imported inline
    # here, or a broken install would emit numpy's stderr diagnostic on every
    # single frame. See _get_cv2() for the full rationale.
    cv2 = _get_cv2()
    if cv2 is not None:
        try:
            resized = cv2.resize(
                cropped, (target_size, target_size), interpolation=cv2.INTER_LINEAR
            )
            return resized
        except Exception:
            # cv2 became unusable at runtime; fall through to numpy path.
            pass

    row_indices = (np.linspace(0, cropped.shape[0] - 1, target_size)).astype(int)
    col_indices = (np.linspace(0, cropped.shape[1] - 1, target_size)).astype(int)
    resized = cropped[np.ix_(row_indices, col_indices)]

    return resized


class BaseInferenceEngine(abc.ABC):
    """Abstract base class for MiniCPM-RobotTrack inference backends."""

    def __init__(self, waypoint_strategy: WaypointStrategy = WaypointStrategy.FIRST):
        self.waypoint_strategy = waypoint_strategy
        self.state = InferenceState()
        self._latencies: List[float] = []

    @abc.abstractmethod
    def infer(self, image: np.ndarray, instruction: str) -> TrackIntent:
        """Run single-frame inference with instruction and return structured TrackIntent."""
        pass

    def record_latency(self, latency_ms: float):
        self._latencies.append(latency_ms)
        if len(self._latencies) > 100:
            self._latencies.pop(0)
        self.state.avg_latency_ms = float(np.mean(self._latencies))
        self.state.p95_latency_ms = float(np.percentile(self._latencies, 95))
        self.state.fps = 1000.0 / max(self.state.avg_latency_ms, 1.0)
        self.state.frames_processed += 1


class MockInferenceEngine(BaseInferenceEngine):
    """Deterministic and synthetic inference engine for tests, CI, and dry-run simulation."""

    def __init__(
        self,
        waypoint_strategy: WaypointStrategy = WaypointStrategy.FIRST,
        simulated_latency_ms: float = 25.0,
        default_confidence: float = 0.85
    ):
        super().__init__(waypoint_strategy)
        self.simulated_latency_ms = simulated_latency_ms
        self.default_confidence = default_confidence
        self.state.is_ready = True
        self.state.backend = "mock"
        self.state.model_name = "MiniCPM-RobotTrack-Mock"

    def infer(self, image: np.ndarray, instruction: str) -> TrackIntent:
        t0 = time.time()
        instruction_clean = (instruction or "").strip().lower()

        if not instruction_clean:
            return TrackIntent(
                instruction="",
                target_detected=False,
                confidence=0.0,
                error_code=3,
                error_msg="Empty instruction"
            )

        # Generate responsive mock waypoints based on instruction keywords
        target_detected = True
        confidence = self.default_confidence
        dx = 0.60
        dy = 0.0
        dz = 0.0
        yaw = 0.0

        if "stop" in instruction_clean or "halt" in instruction_clean or "wait" in instruction_clean:
            dx = 0.0
            dy = 0.0
            confidence = 0.95
        elif "left" in instruction_clean:
            dy = 0.35
            yaw = 0.25
        elif "right" in instruction_clean:
            dy = -0.35
            yaw = -0.25
        elif "back" in instruction_clean or "reverse" in instruction_clean:
            dx = -0.30
        elif "lost" in instruction_clean or "none" in instruction_clean:
            target_detected = False
            confidence = 0.10
            dx = 0.0

        # Waypoint strategy handling
        if self.waypoint_strategy == WaypointStrategy.TWO_STEP:
            dx = dx * 0.9
            dy = dy * 0.9
        elif self.waypoint_strategy == WaypointStrategy.DX4_DW1:
            dx = dx * 1.1

        # Derive raw velocities
        raw_vx = float(np.clip(dx * 0.5, -0.15, 0.15)) if target_detected else 0.0
        raw_wz = float(np.clip((dy * 1.0 + yaw * 0.8), -0.30, 0.30)) if target_detected else 0.0

        latency_ms = self.simulated_latency_ms
        self.record_latency(latency_ms)

        return TrackIntent(
            instruction=instruction,
            target_detected=target_detected,
            confidence=confidence,
            dx=float(dx),
            dy=float(dy),
            dz=float(dz),
            yaw=float(yaw),
            raw_vx=raw_vx,
            raw_wz=raw_wz,
            scaled_vx=raw_vx,
            scaled_wz=raw_wz,
            latency_ms=latency_ms,
            bbox=[0.2, 0.3, 0.8, 0.7] if target_detected else [0.0, 0.0, 0.0, 0.0],
            error_code=0 if target_detected else 3,
            error_msg="" if target_detected else "Target not detected"
        )


class HTTPClientInferenceEngine(BaseInferenceEngine):
    """HTTP client connecting to upstream http_minicpm_robot_track_server.py."""

    def __init__(
        self,
        server_url: str = "http://127.0.0.1:5801",
        timeout_sec: float = 0.5,
        waypoint_strategy: WaypointStrategy = WaypointStrategy.FIRST
    ):
        super().__init__(waypoint_strategy)
        self.server_url = server_url.rstrip("/")
        self.timeout_sec = timeout_sec
        self.state.backend = "http_client"
        self.state.model_name = "MiniCPM-RobotTrack-HTTP"
        self.check_health()

    def check_health(self) -> bool:
        """Check if upstream inference server is reachable and healthy."""
        try:
            req = urllib.request.Request(f"{self.server_url}/health", method="GET")
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                if resp.status == 200:
                    self.state.is_ready = True
                    self.state.last_error = ""
                    return True
        except Exception as e:
            self.state.is_ready = False
            self.state.last_error = f"Server health check failed: {e}"
        return False

    def infer(self, image: np.ndarray, instruction: str) -> TrackIntent:
        t0 = time.time()
        if image is None or image.size == 0:
            return TrackIntent(
                instruction=instruction,
                target_detected=False,
                error_code=1,
                error_msg="No input image frame"
            )

        processed_img = crop_and_resize_image(image, 384)

        try:
            import cv2
            _, encoded_jpeg = cv2.imencode(".jpg", processed_img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            import base64
            img_b64 = base64.b64encode(encoded_jpeg).decode("ascii")
        except Exception:
            # Simple hex payload fallback
            img_b64 = processed_img.tobytes().hex()

        payload = {
            "instruction": instruction,
            "image": img_b64,
            "waypoint_strategy": self.waypoint_strategy.value,
            "timestamp": time.time()
        }

        data_bytes = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            f"{self.server_url}/predict",
            data=data_bytes,
            headers={"Content-Type": "application/json"},
            method="POST"
        )

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                result = json.loads(resp.read().decode("utf-8"))
                latency_ms = (time.time() - t0) * 1000.0
                self.record_latency(latency_ms)
                self.state.is_ready = True
                self.state.consecutive_errors = 0

                return TrackIntent(
                    instruction=instruction,
                    target_detected=bool(result.get("target_detected", True)),
                    confidence=float(result.get("confidence", 0.9)),
                    dx=float(result.get("dx", 0.0)),
                    dy=float(result.get("dy", 0.0)),
                    dz=float(result.get("dz", 0.0)),
                    yaw=float(result.get("yaw", 0.0)),
                    raw_vx=float(result.get("vx", 0.0)),
                    raw_wz=float(result.get("wz", 0.0)),
                    scaled_vx=float(result.get("vx", 0.0)),
                    scaled_wz=float(result.get("wz", 0.0)),
                    latency_ms=latency_ms,
                    bbox=result.get("bbox", [0.0, 0.0, 0.0, 0.0]),
                    error_code=0,
                    error_msg=""
                )
        except Exception as e:
            latency_ms = (time.time() - t0) * 1000.0
            self.state.consecutive_errors += 1
            self.state.last_error = str(e)
            return TrackIntent(
                instruction=instruction,
                target_detected=False,
                confidence=0.0,
                latency_ms=latency_ms,
                error_code=4,
                error_msg=f"HTTP infer error: {e}"
            )


def create_inference_engine(
    backend: str = "mock",
    waypoint_strategy: WaypointStrategy = WaypointStrategy.FIRST,
    server_url: str = "http://127.0.0.1:5801",
    timeout_sec: float = 0.5
) -> BaseInferenceEngine:
    """Factory function for inference engines."""
    if backend == "http":
        return HTTPClientInferenceEngine(
            server_url=server_url,
            timeout_sec=timeout_sec,
            waypoint_strategy=waypoint_strategy
        )
    else:
        return MockInferenceEngine(waypoint_strategy=waypoint_strategy)
