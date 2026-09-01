"""MiniCPM-RobotTrack Inference Node for Puppy Robot.

Subscribes to camera frames and natural language instructions, executes inference
using MiniCPM-RobotTrack backend, and publishes structured tracking intentions.
"""

import time
import json
from typing import Optional
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    Node = object

from .types import TrackIntent, InferenceState, TrackMode, WaypointStrategy
from .inference_engine import create_inference_engine, BaseInferenceEngine, crop_and_resize_image


class MiniCPMTrackNode(Node):
    """ROS2 Node executing MiniCPM-RobotTrack visual tracking model."""

    def __init__(self, node_name: str = "minicpm_track_node"):
        if HAS_RCLPY:
            super().__init__(node_name)
            self._declare_parameters()
            self._init_publishers_and_subscribers()
        else:
            self._init_mock_state()

        self.engine: BaseInferenceEngine = create_inference_engine(
            backend=self.backend,
            waypoint_strategy=WaypointStrategy(self.waypoint_strategy),
            server_url=self.server_url,
            timeout_sec=self.inference_timeout_sec
        )

        self.latest_frame: Optional[np.ndarray] = None
        self.last_frame_timestamp = 0.0
        self.last_intent: Optional[TrackIntent] = None
        self.is_active = True

    def _declare_parameters(self):
        self.declare_parameter("mode", "dry-run")
        self.declare_parameter("backend", "mock")
        self.declare_parameter("server_url", "http://127.0.0.1:5801")
        self.declare_parameter("instruction", "Follow the person ahead")
        self.declare_parameter("waypoint_strategy", "first")
        self.declare_parameter("inference_rate_hz", 10.0)
        self.declare_parameter("inference_timeout_sec", 0.5)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.backend = self.get_parameter("backend").get_parameter_value().string_value
        self.server_url = self.get_parameter("server_url").get_parameter_value().string_value
        self.instruction = self.get_parameter("instruction").get_parameter_value().string_value
        self.waypoint_strategy = self.get_parameter("waypoint_strategy").get_parameter_value().string_value
        self.inference_rate_hz = self.get_parameter("inference_rate_hz").get_parameter_value().double_value
        self.inference_timeout_sec = self.get_parameter("inference_timeout_sec").get_parameter_value().double_value

    def _init_mock_state(self):
        self.mode = "dry-run"
        self.backend = "mock"
        self.server_url = "http://127.0.0.1:5801"
        self.instruction = "Follow the person ahead"
        self.waypoint_strategy = "first"
        self.inference_rate_hz = 10.0
        self.inference_timeout_sec = 0.5

    def _init_publishers_and_subscribers(self):
        self.intent_pub = self.create_publisher(String, "/minicpm_robot/track_intent", 10)
        self.state_pub = self.create_publisher(String, "/minicpm_robot/inference_state", 10)

        self.image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, "/minicpm_robot/instruction", self.instruction_callback, 10
        )

        period = 1.0 / max(self.inference_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"MiniCPMTrackNode started: mode={self.mode}, backend={self.backend}, rate={self.inference_rate_hz}Hz"
        )

    def instruction_callback(self, msg: 'String'):
        new_instruction = msg.data.strip()
        if new_instruction and new_instruction != self.instruction:
            self.instruction = new_instruction
            if HAS_RCLPY:
                self.get_logger().info(f"Updated tracking instruction: '{self.instruction}'")

    def image_callback(self, msg: 'Image'):
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ["rgb8", "bgr8"]:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            elif msg.encoding == "mono8":
                gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                img = np.stack([gray, gray, gray], axis=-1)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))[:, :, :3]

            self.latest_frame = img
            self.last_frame_timestamp = time.time()
        except Exception as e:
            if HAS_RCLPY:
                self.get_logger().warn(f"Error parsing image: {e}")

    def timer_callback(self):
        if not self.is_active:
            return

        # Check frame freshness (if older than 500ms, mark stale)
        now = time.time()
        if self.latest_frame is None or (now - self.last_frame_timestamp > 0.5):
            intent = TrackIntent(
                instruction=self.instruction,
                target_detected=False,
                confidence=0.0,
                error_code=1,
                error_msg="No fresh image frame received"
            )
        else:
            intent = self.engine.infer(self.latest_frame, self.instruction)

        self.last_intent = intent
        self.publish_intent(intent)
        self.publish_state()

    def process_frame(self, frame: np.ndarray, instruction: Optional[str] = None) -> TrackIntent:
        """Direct frame processing helper for testing and headless scripts."""
        inst = instruction if instruction is not None else self.instruction
        return self.engine.infer(frame, inst)

    def publish_intent(self, intent: TrackIntent):
        if not HAS_RCLPY or not hasattr(self, 'intent_pub'):
            return
        msg = String()
        msg.data = json.dumps(intent.to_dict())
        self.intent_pub.publish(msg)

    def publish_state(self):
        if not HAS_RCLPY or not hasattr(self, 'state_pub'):
            return
        state = self.engine.state
        state.mode = TrackMode(self.mode)
        msg = String()
        msg.data = json.dumps(state.to_dict())
        self.state_pub.publish(msg)


def main(args=None):
    if not HAS_RCLPY:
        print("rclpy is not available in current environment")
        return
    rclpy.init(args=args)
    node = MiniCPMTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
