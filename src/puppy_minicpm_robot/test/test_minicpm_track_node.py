"""Unit tests for MiniCPM-RobotTrack inference node and mock backend."""

import pytest
import numpy as np
from puppy_minicpm_robot.types import WaypointStrategy
from puppy_minicpm_robot.inference_engine import MockInferenceEngine
from puppy_minicpm_robot.minicpm_track_node import MiniCPMTrackNode


class TestMockInferenceEngine:
    def test_forward_person_following(self):
        engine = MockInferenceEngine(waypoint_strategy=WaypointStrategy.FIRST)
        dummy_img = np.zeros((384, 384, 3), dtype=np.uint8)
        intent = engine.infer(dummy_img, "Follow the person ahead")

        assert intent.target_detected is True
        assert intent.confidence >= 0.8
        assert intent.dx > 0.0
        assert intent.scaled_vx > 0.0
        assert intent.error_code == 0

    def test_left_turn_intent(self):
        engine = MockInferenceEngine(waypoint_strategy=WaypointStrategy.FIRST)
        dummy_img = np.zeros((384, 384, 3), dtype=np.uint8)
        intent = engine.infer(dummy_img, "Turn left and follow")

        assert intent.dy > 0.0
        assert intent.scaled_wz > 0.0

    def test_stop_intent(self):
        engine = MockInferenceEngine()
        dummy_img = np.zeros((384, 384, 3), dtype=np.uint8)
        intent = engine.infer(dummy_img, "Stop moving")

        assert intent.dx == 0.0
        assert intent.dy == 0.0
        assert intent.scaled_vx == 0.0

    def test_target_lost_intent(self):
        engine = MockInferenceEngine()
        dummy_img = np.zeros((384, 384, 3), dtype=np.uint8)
        intent = engine.infer(dummy_img, "target lost")

        assert intent.target_detected is False
        assert intent.confidence <= 0.2
        assert intent.error_code != 0


class TestMiniCPMTrackNode:
    def test_node_process_frame(self):
        node = MiniCPMTrackNode()
        dummy_img = np.zeros((384, 384, 3), dtype=np.uint8)
        intent = node.process_frame(dummy_img, "Follow person")

        assert intent is not None
        assert intent.target_detected is True
