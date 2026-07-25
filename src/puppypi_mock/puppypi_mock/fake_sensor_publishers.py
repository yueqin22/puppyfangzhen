"""Fake Sensor Publishers: simulates IMU, LiDAR, camera for testing."""
import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Header


class FakeSensorPublishersNode(Node):
    """Simulates sensor data for hardware-free development."""

    def __init__(self):
        super().__init__('fake_sensors')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)

        # Sensor timers at realistic frequencies
        self.create_timer(0.01, self._publish_imu)    # 100Hz
        self.create_timer(0.05, self._publish_scan)   # 20Hz
        self.create_timer(0.033, self._publish_camera) # 30Hz

        self.get_logger().info('Fake sensors started (IMU=100Hz, LiDAR=20Hz, Camera=30Hz)')

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = 1.0
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        self.imu_pub.publish(msg)

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / 72
        msg.time_increment = 0.001
        msg.scan_time = 0.05
        msg.range_min = 0.1
        msg.range_max = 8.0
        # Simulate 72 rays with random distances
        msg.ranges = [random.uniform(1.0, 5.0) for _ in range(72)]
        self.scan_pub.publish(msg)

    def _publish_camera(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = 240
        msg.width = 320
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 320 * 3
        msg.data = bytes([128] * (240 * 320 * 3))  # gray image
        self.camera_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublishersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
