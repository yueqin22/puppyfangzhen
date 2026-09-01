"""Unit tests for Vision Bridge and image preprocessing."""

import pytest
import numpy as np

from puppy_minicpm_robot.inference_engine import crop_and_resize_image
from puppy_minicpm_robot.vision_bridge_node import VisionBridgeNode


class TestVisionBridgePreprocessing:
    def test_crop_and_resize_landscape(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[100:200, 100:200] = 255
        out = crop_and_resize_image(img, target_size=384, mode="center_crop_height")
        assert out.shape == (384, 384, 3)

    def test_crop_and_resize_portrait(self):
        img = np.zeros((640, 480, 3), dtype=np.uint8)
        out = crop_and_resize_image(img, target_size=384, mode="center_crop_height")
        assert out.shape == (384, 384, 3)

    def test_crop_and_resize_square(self):
        img = np.ones((512, 512, 3), dtype=np.uint8) * 128
        out = crop_and_resize_image(img, target_size=384)
        assert out.shape == (384, 384, 3)

    def test_crop_and_resize_empty_or_none(self):
        out_none = crop_and_resize_image(None, target_size=384)
        assert out_none.shape == (384, 384, 3)
        assert np.all(out_none == 0)

        out_empty = crop_and_resize_image(np.array([]), target_size=384)
        assert out_empty.shape == (384, 384, 3)


class TestVisionBridgeNode:
    def test_synthetic_frame_generation(self):
        node = VisionBridgeNode()
        frame = node.generate_synthetic_frame()
        assert frame is not None
        assert frame.shape == (384, 384, 3)
        assert frame.dtype == np.uint8
        # Ensure image has non-zero content
        assert np.sum(frame) > 0

    def test_mock_state_defaults(self):
        node = VisionBridgeNode()
        assert node.output_width == 384
        assert node.output_height == 384
        assert node.publish_fps == 10.0
        assert node.camera_source == "synthetic"
