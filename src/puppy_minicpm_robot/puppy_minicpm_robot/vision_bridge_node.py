"""Vision Bridge Node for Puppy Robot MiniCPM-RobotTrack integration.

Unifies input from simulation, USB/RealSense camera, Go2 VideoClient, and synthetic sources,
publishing standard /camera/color/image_raw and /camera/color/camera_info.
"""

import time
import json
from array import array
from typing import Optional, Tuple
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String, Header
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    Node = object  # Dummy base class for non-ROS test environments

from .types import CameraSource
from .inference_engine import crop_and_resize_image


class VisionBridgeNode(Node):
    """ROS2 Node that manages camera input sources and publishes unified RGB frames."""

    def __init__(self, node_name: str = "vision_bridge_node"):
        if HAS_RCLPY:
            super().__init__(node_name)
            self._declare_parameters()
            self._init_publishers_and_subscribers()
        else:
            self._init_mock_state()

        self.frames_received = 0
        self.frames_published = 0
        self.last_frame_time = time.time()
        self.latest_cv_image: Optional[np.ndarray] = None
        self.is_streaming = True
        # Set by image_callback when a genuinely new frame arrives; cleared
        # once that frame has been published. Used so the timer does not
        # re-publish the same image over and over (see timer_callback).
        self._has_new_frame = False

    def _declare_parameters(self):
        self.declare_parameter("camera_source", "synthetic")
        self.declare_parameter("publish_fps", 10.0)
        self.declare_parameter("output_width", 384)
        self.declare_parameter("output_height", 384)
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.camera_source = self.get_parameter("camera_source").get_parameter_value().string_value
        self.publish_fps = self.get_parameter("publish_fps").get_parameter_value().double_value
        self.output_width = self.get_parameter("output_width").get_parameter_value().integer_value
        self.output_height = self.get_parameter("output_height").get_parameter_value().integer_value
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

    def _init_mock_state(self):
        self.camera_source = "synthetic"
        self.publish_fps = 10.0
        self.output_width = 384
        self.output_height = 384
        self.input_topic = "/camera/color/image_raw"
        self.frame_id = "camera_color_optical_frame"

    def _init_publishers_and_subscribers(self):
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
        self.status_pub = self.create_publisher(String, "/vision_bridge/status", 10)

        # If source is sim/realsense and remapping is used, subscribe to incoming topic
        if self.camera_source in ["sim", "realsense", "external"]:
            self.image_sub = self.create_subscription(
                Image, self.input_topic, self.image_callback, 10
            )

        # Timer for publishing (especially for synthetic/go2/file sources)
        period = 1.0 / max(self.publish_fps, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"VisionBridgeNode initialized with source={self.camera_source} at {self.publish_fps} FPS"
        )

    def generate_synthetic_frame(self) -> np.ndarray:
        """Generate procedural synthetic frame for closed-loop testing without hardware."""
        img = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        # Background gradient
        t = time.time()
        c = int((np.sin(t) + 1.0) * 40 + 20)
        img[:, :] = [c, c + 20, c + 40]

        # Draw moving target block
        x_center = int((np.sin(t * 1.5) * 0.3 + 0.5) * self.output_width)
        y_center = int(self.output_height * 0.5)
        half_box = 40
        x1 = max(0, x_center - half_box)
        x2 = min(self.output_width, x_center + half_box)
        y1 = max(0, y_center - half_box)
        y2 = min(self.output_height, y_center + half_box)
        img[y1:y2, x1:x2] = [30, 200, 240]  # Yellow/Orange target
        return img

    def image_callback(self, msg: 'Image'):
        """Handle incoming ROS2 Image message."""
        self.frames_received += 1
        self.last_frame_time = time.time()
        # Convert ROS Image to numpy array
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ["rgb8", "bgr8"]:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            elif msg.encoding == "mono8":
                gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                img = np.stack([gray, gray, gray], axis=-1)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))[:, :, :3]

            processed = crop_and_resize_image(img, self.output_width)
            self.latest_cv_image = processed
            self._has_new_frame = True
        except Exception as e:
            if HAS_RCLPY:
                self.get_logger().warn(f"Failed to process image frame: {e}")

    def timer_callback(self):
        """Timer callback for frame generation and publication."""
        if not self.is_streaming:
            return

        if self.camera_source == "synthetic":
            img = self.generate_synthetic_frame()
            self.latest_cv_image = img
            self.publish_frame(img)
        elif self.latest_cv_image is not None and self._has_new_frame:
            # Publish ONLY when a genuinely new camera frame has arrived.
            #
            # The timer runs at publish_fps (10 Hz by default) while a Gazebo
            # camera typically delivers only ~2-3 Hz. Publishing the cached
            # frame on every tick therefore re-sent the SAME image ~4x per
            # second: ~4.4 MB/s of redundant DDS traffic for a 384x384 RGB
            # frame, plus a matching CameraInfo. That burned CPU for no new
            # information (measured ~76% of a core on a loaded 4-core box)
            # and inflated end-to-end latency. Subscribers still receive
            # every real frame exactly once, on the first tick after arrival.
            self.publish_frame(self.latest_cv_image)
            self._has_new_frame = False

        self._publish_status()

    def publish_frame(self, frame: np.ndarray):
        """Publish numpy frame as ROS2 Image and CameraInfo."""
        if not HAS_RCLPY or not hasattr(self, 'image_pub'):
            return

        h, w, c = frame.shape
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.frame_id
        img_msg.height = h
        img_msg.width = w
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = 0
        img_msg.step = w * c
        # PERFORMANCE: hand the payload over as array.array('B'), never bytes.
        #
        # The rosidl-generated setter for a `uint8[]` field only fast-paths
        # array.array; anything else (including bytes/bytearray) falls into a
        # __debug__ branch that runs TWO full Python-level generator passes --
        #   all(isinstance(v, int) for v in value)
        #   all(0 <= val < 256 for val in value)
        # -- over every single byte. For a 384x384 RGB frame that is 442,368
        # elements per publish: ~180 ms of pure interpreter loop per frame,
        # which pinned this node at ~100% of a core even though the actual
        # image work (resize) costs ~2 ms.
        #
        # array.array.frombytes() does the same conversion at C level, so the
        # setter short-circuits on the isinstance() check and returns.
        contiguous = np.ascontiguousarray(frame)
        payload = array("B")
        payload.frombytes(contiguous.tobytes())
        img_msg.data = payload

        self.image_pub.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.height = h
        info_msg.width = w
        info_msg.distortion_model = "plumb_bob"
        info_msg.k = [384.0, 0.0, 192.0, 0.0, 384.0, 192.0, 0.0, 0.0, 1.0]
        self.camera_info_pub.publish(info_msg)

        self.frames_published += 1

    def _publish_status(self):
        if not HAS_RCLPY or not hasattr(self, 'status_pub'):
            return
        status = {
            "camera_source": self.camera_source,
            "frames_published": self.frames_published,
            "frames_received": self.frames_received,
            "is_streaming": self.is_streaming,
            "age_ms": round((time.time() - self.last_frame_time) * 1000.0, 1)
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    if not HAS_RCLPY:
        print("rclpy is not available in current environment")
        return
    rclpy.init(args=args)
    node = VisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
