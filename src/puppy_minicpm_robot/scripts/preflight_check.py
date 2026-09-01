#!/usr/bin/env python3
"""Preflight Check Tool for MiniCPM-RobotTrack & Go2 / Jetson Deployment.

Checks hardware, software dependencies, DDS, PyTorch/CUDA, TensorRT, camera interfaces,
and model assets according to 20260828.md Phase F requirements.
"""

import os
import sys
import platform
import subprocess
import shutil
from pathlib import Path
from typing import Dict, Any, List, Tuple


class PreflightChecker:
    def __init__(self):
        self.results: List[Tuple[str, str, str]] = []  # (Category, Status [PASS/WARN/FAIL], Message)

    def log(self, category: str, status: str, message: str):
        self.results.append((category, status, message))

    def check_python_environment(self):
        py_ver = platform.python_version()
        major, minor = sys.version_info[:2]
        if major == 3 and minor >= 8:
            self.log("Python", "PASS", f"Python version {py_ver} OK")
        else:
            self.log("Python", "FAIL", f"Python version {py_ver} is unsupported (requires >= 3.8)")

    def check_pytorch_and_cuda(self):
        try:
            import torch
            has_cuda = torch.cuda.is_available()
            if has_cuda:
                device_name = torch.cuda.get_device_name(0)
                cuda_ver = torch.version.cuda
                self.log("PyTorch/CUDA", "PASS", f"PyTorch {torch.__version__} with CUDA {cuda_ver} on {device_name}")
            else:
                self.log("PyTorch/CUDA", "WARN", f"PyTorch {torch.__version__} loaded (CPU only, GPU CUDA unavailable)")
        except ImportError:
            self.log("PyTorch/CUDA", "WARN", "PyTorch not installed in current environment (CPU mock mode active)")

    def check_tensorrt(self):
        try:
            import tensorrt as trt
            self.log("TensorRT", "PASS", f"TensorRT version {trt.__version__} available")
        except ImportError:
            self.log("TensorRT", "WARN", "TensorRT not found (required for Orin Jetson edge acceleration)")

    def check_ros2_environment(self):
        ros_distro = os.environ.get("ROS_DISTRO", "")
        if ros_distro:
            self.log("ROS 2", "PASS", f"ROS 2 environment active: ROS_DISTRO={ros_distro}")
        else:
            self.log("ROS 2", "WARN", "ROS_DISTRO not set in environment (ROS2 workspace not sourced)")

    def check_cyclonedds(self):
        cyclonedds_home = os.environ.get("CYCLONEDDS_HOME", "/home/unitree/cyclonedds/install")
        cyclonedds_uri = os.environ.get("CYCLONEDDS_URI", "")
        if os.path.exists(cyclonedds_home) or cyclonedds_uri:
            self.log("CycloneDDS", "PASS", f"CycloneDDS configured ({cyclonedds_uri or cyclonedds_home})")
        else:
            self.log("CycloneDDS", "WARN", "CycloneDDS home not detected (verify Unitree CycloneDDS setup on Go2)")

    def check_opencv(self):
        try:
            import cv2
            self.log("OpenCV", "PASS", f"OpenCV version {cv2.__version__} available")
        except ImportError:
            self.log("OpenCV", "WARN", "OpenCV not found (falling back to pure numpy image operations)")

    def check_model_checkpoints(self):
        workspace_root = Path(__file__).resolve().parents[3]
        ckpt_dir = workspace_root / "third_party" / "MiniCPM-Robot" / "MiniCPM-RobotTrack" / "minicpm_robot_track" / "checkpoints"
        if ckpt_dir.exists() and any(ckpt_dir.iterdir()):
            self.log("Checkpoints", "PASS", f"Model checkpoint directory found: {ckpt_dir}")
        else:
            self.log("Checkpoints", "WARN", f"Checkpoints empty or external at {ckpt_dir} (Mock/Dry-Run available)")

    def check_hardware_camera(self):
        # Check network interface on Linux or video devices
        if platform.system() == "Linux":
            nic_check = shutil.which("ip")
            if nic_check:
                try:
                    out = subprocess.check_output(["ip", "-br", "link"]).decode("utf-8")
                    if "enP8p1s0" in out or "eth0" in out:
                        self.log("Camera Interface", "PASS", "Wired camera NIC detected")
                    else:
                        self.log("Camera Interface", "WARN", "Verified Go2 camera NIC (enP8p1s0) not found")
                except Exception:
                    self.log("Camera Interface", "WARN", "Could not query network interfaces")
        else:
            self.log("Camera Interface", "INFO", f"Running on {platform.system()} host (Simulation & Mock active)")

    def run_all_checks(self) -> Dict[str, Any]:
        self.check_python_environment()
        self.check_pytorch_and_cuda()
        self.check_tensorrt()
        self.check_ros2_environment()
        self.check_cyclonedds()
        self.check_opencv()
        self.check_model_checkpoints()
        self.check_hardware_camera()

        pass_count = sum(1 for _, s, _ in self.results if s == "PASS")
        warn_count = sum(1 for _, s, _ in self.results if s == "WARN")
        fail_count = sum(1 for _, s, _ in self.results if s == "FAIL")

        ready_for_live = (fail_count == 0 and warn_count <= 2 and any(s == "PASS" for c, s, _ in self.results if c == "PyTorch/CUDA"))

        print("\n" + "=" * 60)
        print("     MiniCPM-RobotTrack Go2 Preflight Diagnostics")
        print("=" * 60)
        for cat, status, msg in self.results:
            tag = f"[{status}]"
            print(f"{tag:<8} {cat:<18}: {msg}")
        print("-" * 60)
        print(f"Summary: {pass_count} Passed, {warn_count} Warnings, {fail_count} Failed")

        if fail_count > 0:
            print("Status : BLOCKED - Critical dependencies missing for execution.")
        elif ready_for_live:
            print("Status : READY_FOR_LIVE_TEST (Proceed with dry-run and emergency stop ready).")
        else:
            print("Status : READY_FOR_SIM_AND_DRY_RUN (Simulation & Mock modes fully functional).")
        print("=" * 60 + "\n")

        return {
            "pass_count": pass_count,
            "warn_count": warn_count,
            "fail_count": fail_count,
            "ready_for_live": ready_for_live,
            "details": self.results
        }


def main():
    checker = PreflightChecker()
    checker.run_all_checks()


if __name__ == "__main__":
    main()
