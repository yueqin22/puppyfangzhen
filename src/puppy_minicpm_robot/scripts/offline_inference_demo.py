#!/usr/bin/env python3
"""Offline Inference Demo for MiniCPM-RobotTrack (Phase A Exit Criteria).

Performs single-frame inference with a text instruction and outputs structured JSON.
Usage:
    python offline_inference_demo.py --instruction "Follow the person ahead" --output logs/offline_result.json
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path
import numpy as np

# Ensure package and parent are in path
current_dir = Path(__file__).resolve().parent
pkg_root = current_dir.parent
sys.path.insert(0, str(pkg_root))
sys.path.insert(0, str(pkg_root.parent))

from puppy_minicpm_robot.types import WaypointStrategy, TrackIntent
from puppy_minicpm_robot.inference_engine import (
    create_inference_engine,
    crop_and_resize_image
)


def load_or_create_image(image_path: str = None) -> np.ndarray:
    """Load image from disk or generate a synthetic test image."""
    if image_path and os.path.exists(image_path):
        try:
            import cv2
            img = cv2.imread(image_path)
            if img is not None:
                # Convert BGR to RGB
                return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"[WARN] Failed to read {image_path} with cv2 ({e}), falling back to synthetic image.")

    # Generate synthetic 640x480 RGB image with a person-like mock silhouette
    img = np.ones((480, 640, 3), dtype=np.uint8) * 210
    # Add ground floor
    img[320:, :] = [140, 150, 160]
    # Add person box
    img[120:380, 270:370] = [40, 70, 180]  # Blue figure
    img[90:120, 295:345] = [220, 180, 140]  # Head
    return img


def run_offline_inference(
    image_path: str = None,
    instruction: str = "Follow the person ahead",
    backend: str = "mock",
    server_url: str = "http://127.0.0.1:5801",
    waypoint_strategy: str = "first",
    output_path: str = "logs/offline_inference_result.json"
) -> dict:
    print(f"=== MiniCPM-RobotTrack Offline Inference Demo ===")
    print(f"Instruction : '{instruction}'")
    print(f"Backend     : {backend}")
    print(f"Strategy    : {waypoint_strategy}")

    # 1. Load image
    img = load_or_create_image(image_path)
    print(f"Image shape : {img.shape}")

    # 2. Initialize engine
    strategy = WaypointStrategy(waypoint_strategy)
    engine = create_inference_engine(backend=backend, waypoint_strategy=strategy, server_url=server_url)

    # 3. Perform inference
    t0 = time.time()
    intent: TrackIntent = engine.infer(img, instruction)
    wall_latency_ms = (time.time() - t0) * 1000.0

    result_dict = {
        "status": "SUCCESS" if intent.error_code == 0 else "DEGRADED",
        "wall_latency_ms": round(wall_latency_ms, 2),
        "intent": intent.to_dict(),
        "metadata": {
            "backend": backend,
            "waypoint_strategy": waypoint_strategy,
            "image_shape": list(img.shape),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        }
    }

    # 4. Save structured output
    if output_path:
        out_file = Path(output_path)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        with open(out_file, "w", encoding="utf-8") as f:
            json.dump(result_dict, f, indent=2, ensure_ascii=False)
        print(f"Saved result: {out_file}")

    print("\n--- Inference Result ---")
    print(f"Target Detected : {intent.target_detected} (Confidence: {intent.confidence:.2f})")
    print(f"3D Waypoint     : dx={intent.dx:.3f}m, dy={intent.dy:.3f}m, dz={intent.dz:.3f}m, yaw={intent.yaw:.3f}rad")
    print(f"Suggested Speed : vx={intent.scaled_vx:.3f} m/s, wz={intent.scaled_wz:.3f} rad/s")
    print(f"Model Latency   : {intent.latency_ms:.1f} ms")
    print(f"Error Status    : code={intent.error_code} ({intent.error_msg or 'OK'})")
    print("=================================================")

    return result_dict


def main():
    parser = argparse.ArgumentParser(description="MiniCPM-RobotTrack Offline Inference Demo")
    parser.add_argument("--image", type=str, default=None, help="Path to input image file")
    parser.add_argument("--instruction", type=str, default="Follow the person ahead", help="Tracking prompt")
    parser.add_argument("--backend", type=str, default="mock", choices=["mock", "http"], help="Inference backend")
    parser.add_argument("--server-url", type=str, default="http://127.0.0.1:5801", help="Server URL for HTTP backend")
    parser.add_argument("--strategy", type=str, default="first", choices=["first", "two-step", "dx4-dw1"], help="Waypoint strategy")
    parser.add_argument("--output", type=str, default="logs/offline_inference_result.json", help="Path to save result JSON")

    args = parser.parse_args()
    run_offline_inference(
        image_path=args.image,
        instruction=args.instruction,
        backend=args.backend,
        server_url=args.server_url,
        waypoint_strategy=args.strategy,
        output_path=args.output
    )


if __name__ == "__main__":
    main()
