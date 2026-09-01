#!/usr/bin/env python3
"""Dataset Player & Camera Stream Simulator for MiniCPM-RobotTrack.

Plays back images or sequences at fixed framerates, with timestamp sync and latency measurement.
"""

import argparse
import time
import os
import sys
from pathlib import Path
import numpy as np
from typing import Optional, Tuple, List, Dict

# Add parent path
current_dir = Path(__file__).resolve().parent
pkg_root = current_dir.parent
sys.path.insert(0, str(pkg_root))
sys.path.insert(0, str(pkg_root.parent))

from puppy_minicpm_robot.inference_engine import crop_and_resize_image


class DatasetPlayer:
    def __init__(
        self,
        dataset_dir: Optional[str] = None,
        target_fps: float = 10.0,
        target_size: int = 384,
        loop: bool = True
    ):
        self.dataset_dir = dataset_dir
        self.target_fps = target_fps
        self.target_size = target_size
        self.loop = loop
        self.frames = []
        self.current_idx = 0
        self._load_dataset()

    def _load_dataset(self):
        if self.dataset_dir and os.path.isdir(self.dataset_dir):
            valid_exts = {".jpg", ".jpeg", ".png", ".bmp"}
            files = sorted([
                os.path.join(self.dataset_dir, f)
                for f in os.listdir(self.dataset_dir)
                if os.path.splitext(f)[1].lower() in valid_exts
            ])
            try:
                import cv2
                for f in files:
                    bgr = cv2.imread(f)
                    if bgr is not None:
                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                        self.frames.append(crop_and_resize_image(rgb, self.target_size))
            except Exception:
                pass

        if not self.frames:
            # Generate 5 synthetic benchmark frames
            for i in range(5):
                img = np.ones((self.target_size, self.target_size, 3), dtype=np.uint8) * 160
                # Moving synthetic box
                x1 = int((i + 1) * 45)
                img[100:250, x1:x1+80] = [40, 180, 220]
                self.frames.append(img)

    def get_next_frame(self) -> Tuple[np.ndarray, int, float]:
        """Return next frame, frame index, and timestamp."""
        if not self.frames:
            dummy = np.zeros((self.target_size, self.target_size, 3), dtype=np.uint8)
            return dummy, 0, time.time()

        frame = self.frames[self.current_idx]
        idx = self.current_idx
        now = time.time()

        self.current_idx += 1
        if self.current_idx >= len(self.frames):
            if self.loop:
                self.current_idx = 0
            else:
                self.current_idx = len(self.frames) - 1

        return frame, idx, now


def main():
    parser = argparse.ArgumentParser(description="MiniCPM-RobotTrack Dataset Player")
    parser.add_argument("--dataset", type=str, default=None, help="Directory containing images")
    parser.add_argument("--fps", type=float, default=10.0, help="Playback frame rate")
    parser.add_argument("--count", type=int, default=10, help="Number of frames to play")
    args = parser.parse_args()

    player = DatasetPlayer(dataset_dir=args.dataset, target_fps=args.fps)
    print(f"Dataset player initialized with {len(player.frames)} frames at {args.fps} FPS")

    delay = 1.0 / max(args.fps, 1.0)
    for i in range(args.count):
        t0 = time.time()
        frame, idx, ts = player.get_next_frame()
        print(f"Frame #{i:03d} (dataset idx: {idx}): shape={frame.shape}, ts={ts:.3f}")
        elapsed = time.time() - t0
        time.sleep(max(0.0, delay - elapsed))


if __name__ == "__main__":
    from typing import Optional, Tuple
    main()
