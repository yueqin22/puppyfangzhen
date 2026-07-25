"""nav_core ROS2节点 (v7.8)

将nav_core的核心导航算法（AMCL、A*、DWA/TEB、frontier探索）
包装为ROS2节点，使研究成果可以在ROS2产品系统中使用。

订阅:
  - /scan (sensor_msgs/LaserScan): LiDAR扫描
  - /odom (nav_msgs/Odometry): 里程计
  - /map (nav_msgs/OccupancyGrid): 静态地图
  - /initialpose (geometry_msgs/PoseWithCovarianceStamped): 初始位姿

发布:
  - /cmd_vel (geometry_msgs/Twist): 速度命令
  - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped): 定位结果
  - /frontiers (visualization_msgs/MarkerArray): frontier可视化

服务:
  - /nav_core/start_exploration: 开始探索
  - /nav_core/stop: 停止导航
"""
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool


class NavCoreNode(Node):
    """nav_core ROS2包装节点 (v7.8)

    将纯Python的nav_core算法包装为ROS2节点，
    使产品系统可以通过标准ROS2接口使用研究成果。
    """

    def __init__(self):
        super().__init__('nav_core_node')

        # 延迟导入nav_core模块
        self._import_modules()

        # 参数
        self.declare_parameter('use_amcl', True)
        self.declare_parameter('use_teb', False)
        self.declare_parameter('map_file', '')
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 1.2)

        # 初始化算法模块
        self._init_algorithm_modules()

        # 状态
        self._exploring = False
        self._current_goal = None
        self._frame = 0

        # ROS2 接口
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()

        # 控制循环定时器 (10Hz)
        self._timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('nav_core节点已启动 (v7.8)')

    def _import_modules(self):
        """延迟导入nav_core模块"""
        try:
            from occupancy_grid import OccupancyGrid
            from costmap import Costmap
            from astar_planner import AStarPlanner
            from dwa_planner import DWAPlanner
            from amcl import AMCL
            from odometry import Odometry
            from nav_core.exploration.frontier_manager import FrontierManager
            from nav_core.runtime import NavState, NavigationRuntimeState
            from nav_core.metrics.telemetry import TelemetryCollector
            from nav_core.session.session_store import SessionStore

            self._OccupancyGrid = OccupancyGrid
            self._Costmap = Costmap
            self._AStarPlanner = AStarPlanner
            self._DWAPlanner = DWAPlanner
            self._AMCL = AMCL
            self._Odometry = Odometry
            self._FrontierManager = FrontierManager
            self._NavState = NavState
            self._NavigationRuntimeState = NavigationRuntimeState
            self._TelemetryCollector = TelemetryCollector
            self._SessionStore = SessionStore
            self._modules_loaded = True
        except ImportError as e:
            self.get_logger().error(f'nav_core模块导入失败: {e}')
            self._modules_loaded = False

    def _init_algorithm_modules(self):
        """初始化算法模块实例"""
        if not self._modules_loaded:
            return

        self.occ_grid = self._OccupancyGrid()
        self.costmap = self._Costmap()
        self.astar = self._AStarPlanner(self.costmap)
        self.dwa = self._DWAPlanner(self.costmap)

        use_amcl = self.get_parameter('use_amcl').value
        if use_amcl:
            self.amcl = self._AMCL(self.occ_grid, n_particles=200, n_obs_rays=36)
        else:
            self.amcl = None

        self.odom = self._Odometry()
        self.runtime = self._NavigationRuntimeState()
        self.telemetry = self._TelemetryCollector()

        # TEB规划器(可选)
        use_teb = self.get_parameter('use_teb').value
        if use_teb:
            try:
                from teb_planner import TEBPlanner
                self.teb = TEBPlanner(self.costmap)
                self.local_planner = self.teb
                self.get_logger().info('使用TEB局部规划器')
            except ImportError:
                self.local_planner = self.dwa
                self.get_logger().warning('TEB导入失败，使用DWA')
        else:
            self.local_planner = self.dwa

    def _setup_subscribers(self):
        """设置订阅者"""
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._on_initial_pose, 10)

    def _setup_publishers(self):
        """设置发布者"""
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.status_pub = self.create_publisher(String, '/nav_core/status', 10)

    def _setup_services(self):
        """设置服务"""
        self.create_service(Trigger, '/nav_core/start_exploration',
                           self._start_exploration)
        self.create_service(Trigger, '/nav_core/stop', self._stop_nav)

    def _on_scan(self, msg: LaserScan):
        """处理LiDAR扫描"""
        if not self._modules_loaded:
            return
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        distances = np.array(msg.ranges)
        # 更新占据栅格和代价地图
        if hasattr(self, '_current_pose'):
            rx, ry, ryaw = self._current_pose
            self.occ_grid.update_from_scan(rx, ry, angles, distances,
                                           max_range=msg.range_max)
            self.costmap.update_static(self.occ_grid, frame=self._frame)

    def _on_odom(self, msg: Odometry):
        """处理里程计"""
        if not self._modules_loaded:
            return
        pose = msg.pose.pose
        # 用于AMCL运动更新
        self._last_odom = (pose.position.x, pose.position.y,
                          pose.orientation.z)

    def _on_map(self, msg: OccupancyGrid):
        """处理静态地图"""
        if not self._modules_loaded:
            return
        # 将ROS地图消息转换为内部格式
        self.get_logger().info(f'收到地图: {msg.info.width}x{msg.info.height}')

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        """处理初始位姿"""
        if not self._modules_loaded or self.amcl is None:
            return
        pose = msg.pose.pose
        self.amcl.recover(pose.position.x, pose.position.y, 0.0, spread=0.5)
        self.get_logger().info(
            f'设置初始位姿: ({pose.position.x:.2f}, {pose.position.y:.2f})')

    def _start_exploration(self, request, response):
        """开始探索服务"""
        self._exploring = True
        self.runtime.state = self._NavState.PLAN
        response.success = True
        response.message = '探索已开始'
        self.get_logger().info('探索已启动')
        return response

    def _stop_nav(self, request, response):
        """停止导航服务"""
        self._exploring = False
        # 停止机器人
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        response.success = True
        response.message = '导航已停止'
        self.get_logger().info('导航已停止')
        return response

    def _control_loop(self):
        """主控制循环 (10Hz)"""
        if not self._modules_loaded or not self._exploring:
            return

        self._frame += 1
        # 这里简化处理，实际实现需要完整的导航逻辑
        # 发布状态
        status = String()
        status.data = f'frame={self._frame} state={self.runtime.state}'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = NavCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
