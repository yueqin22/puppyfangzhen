"""Unit test for Phase A offline inference demo."""

import pytest
import os
import json
import sys
from pathlib import Path

try:
    from puppy_minicpm_robot.scripts.offline_inference_demo import run_offline_inference
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    from offline_inference_demo import run_offline_inference


class TestOfflineInferenceDemo:
    def test_run_offline_inference_success(self, tmp_path):
        out_json = tmp_path / "test_offline.json"
        res = run_offline_inference(
            image_path=None,
            instruction="Follow the person ahead",
            backend="mock",
            output_path=str(out_json)
        )

        assert res["status"] == "SUCCESS"
        assert out_json.exists()

        with open(out_json, "r", encoding="utf-8") as f:
            data = json.load(f)

        assert data["status"] == "SUCCESS"
        assert "intent" in data
        assert data["intent"]["target_detected"] is True
        assert data["intent"]["scaled_vx"] > 0.0
