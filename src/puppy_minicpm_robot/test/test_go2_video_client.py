"""Unit test for Go2 Video Client."""

import pytest
import numpy as np
from puppy_minicpm_robot.go2_video_client import Go2VideoClient


class TestGo2VideoClient:
    def test_mock_frame_set_and_get(self):
        client = Go2VideoClient(network_interface="dummy", target_size=384, timeout_sec=2.0)
        mock_img = np.ones((480, 640, 3), dtype=np.uint8) * 200

        client.set_mock_frame(mock_img)
        ok, frame, ts = client.get_frame()

        assert ok is True
        assert frame is not None
        assert frame.shape == (384, 384, 3)
        assert ts > 0.0
