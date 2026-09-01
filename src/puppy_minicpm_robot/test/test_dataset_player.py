"""Unit test for Dataset Player."""

import pytest
import sys
import numpy as np
from pathlib import Path

try:
    from puppy_minicpm_robot.scripts.dataset_player import DatasetPlayer
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    from dataset_player import DatasetPlayer


class TestDatasetPlayer:
    def test_synthetic_player_loop(self):
        player = DatasetPlayer(dataset_dir=None, target_fps=10.0, target_size=384, loop=True)
        assert len(player.frames) > 0

        frame1, idx1, ts1 = player.get_next_frame()
        assert frame1.shape == (384, 384, 3)
        assert idx1 == 0
        assert ts1 > 0.0

        frame2, idx2, ts2 = player.get_next_frame()
        assert idx2 == 1
