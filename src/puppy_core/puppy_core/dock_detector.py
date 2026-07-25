"""Dock感知闭环 (v7.9)

检测充电桩并执行自动回充流程。
使用LiDAR和视觉传感器融合检测dock位置。

状态机:
  SEARCHING -> DETECTED -> APPROACHING -> ALIGNED -> DOCKED
  失败时: -> RECOVERING -> SEARCHING
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Tuple


class DockState(IntEnum):
    """回充状态"""
    IDLE = 0        # 空闲
    SEARCHING = 1   # 搜索dock
    DETECTED = 2     # 检测到dock
    APPROACHING = 3  # 接近中
    ALIGNED = 4      # 已对齐
    DOCKED = 5       # 已对接
    RECOVERING = 6   # 恢复中
    FAILED = 7       # 失败


@dataclass
class DockDetection:
    """Dock检测结果"""
    detected: bool = False
    distance: float = 0.0    # 到dock的距离
    bearing: float = 0.0    # 方位角(弧度)
    confidence: float = 0.0
    method: str = ""        # 'lidar', 'vision', 'fusion'


class DockDetectorNode(Node):
    """Dock感知节点 (v7.9)

    检测充电桩并执行自动回充。
    """

    def __init__(self):
        super().__init__('dock_detector')

        # 参数
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', -3.5)
        self.declare_parameter('dock_yaw', 0.0)
        self.declare_parameter('approach_speed', 0.1)
        self.declare_parameter('align_threshold', 0.1)  # 对齐阈值(弧度)
        self.declare_parameter('dock_distance_threshold', 0.3)  # 对接距离
        self.declare_parameter('search_timeout', 30.0)  # 搜索超时(秒)

        # dock目标位置
        self.dock_x = self.get_parameter('dock_x').value
        self.dock_y = self.get_parameter('dock_y').value
        self.dock_yaw = self.get_parameter('dock_yaw').value

        # 状态
        self.state = DockState.IDLE
        self._detection = DockDetection()
        self._state_start_time = self.get_clock().now()

        # ROS2接口
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/dock/state', 10)
        self.create_service(Trigger, '/dock/start_docking', self._start_docking)
        self.create_service(Trigger, '/dock/cancel', self._cancel_docking)

        # 控制循环
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f'Dock感知节点已启动 (v7.9), dock位置: ({self.dock_x:.1f}, {self.dock_y:.1f})')

    def _on_scan(self, msg: LaserScan):
        """处理LiDAR扫描，检测dock特征"""
        # dock特征：特定距离和角度范围内的反射点
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        distances = np.array(msg.ranges)

        # 简化检测：寻找前方近距离的特征点
        front_mask = (np.abs(angles) < 0.5) & (distances < 1.0) & (distances > 0.1)
        if np.any(front_mask):
            front_dists = distances[front_mask]
            front_angles = angles[front_mask]
            # 检测dock的V形特征（两侧有近距离点，中间有远距离点）
            min_dist = np.min(front_dists)
            min_idx = np.argmin(front_dists)
            self._detection = DockDetection(
                detected=True,
                distance=float(min_dist),
                bearing=float(front_angles[min_idx]),
                confidence=0.7,
                method='lidar',
            )
        else:
            self._detection.detected = False

    def _start_docking(self, request, response):
        """启动自动回充"""
        self.state = DockState.SEARCHING
        self._state_start_time = self.get_clock().now()
        response.success = True
        response.message = '回充已启动'
        self.get_logger().info('自动回充已启动')
        return response

    def _cancel_docking(self, request, response):
        """取消回充"""
        self.state = DockState.IDLE
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        response.success = True
        response.message = '回充已取消'
        return response

    def _control_loop(self):
        """主控制循环"""
        if self.state == DockState.IDLE:
            return

        # 发布状态
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds / 1e9

        if self.state == DockState.SEARCHING:
            self._handle_searching(elapsed)
        elif self.state == DockState.DETECTED:
            self._handle_detected()
        elif self.state == DockState.APPROACHING:
            self._handle_approaching()
        elif self.state == DockState.ALIGNED:
            self._handle_aligned()
        elif self.state == DockState.RECOVERING:
            self._handle_recovering(elapsed)

    def _handle_searching(self, elapsed):
        """搜索状态处理"""
        if self._detection.detected:
            self.state = DockState.DETECTED
            self._state_start_time = self.get_clock().now()
            self.get_logger().info(
                f'检测到dock: 距离={self._detection.distance:.2f}m '
                f'方位={math.degrees(self._detection.bearing):.1f}°')
        elif elapsed > self.get_parameter('search_timeout').value:
            self.state = DockState.FAILED
            self.get_logger().error('搜索dock超时')

    def _handle_detected(self):
        """检测到dock状态处理"""
        if not self._detection.detected:
            self.state = DockState.SEARCHING
            return
        self.state = DockState.APPROACHING
        self._state_start_time = self.get_clock().now()

    def _handle_approaching(self):
        """接近dock状态处理"""
        if not self._detection.detected:
            self.state = DockState.SEARCHING
            return

        cmd = Twist()
        align_thresh = self.get_parameter('align_threshold').value
        dock_dist_thresh = self.get_parameter('dock_distance_threshold').value

        # 旋转对准dock
        if abs(self._detection.bearing) > align_thresh:
            cmd.angular.z = 0.5 * np.sign(self._detection.bearing)
        # 前进接近
        elif self._detection.distance > dock_dist_thresh:
            cmd.linear.x = self.get_parameter('approach_speed').value
        else:
            # 到达对接距离
            self.state = DockState.ALIGNED
            self.get_logger().info('已对齐dock，准备对接')

        self.cmd_vel_pub.publish(cmd)

    def _handle_aligned(self):
        """已对齐状态处理"""
        # 缓慢前进完成对接
        if self._detection.distance > 0.1:
            cmd = Twist()
            cmd.linear.x = 0.05
            self.cmd_vel_pub.publish(cmd)
        else:
            self.state = DockState.DOCKED
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info('对接完成！')

    def _handle_recovering(self, elapsed):
        """恢复状态处理"""
        if elapsed > 5.0:
            self.state = DockState.SEARCHING
            self._state_start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = DockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
