#!/usr/bin/env python3
"""
CoppeliaSim ZMQ 桥接节点
完全绕过 simROS2 插件，通过 ZMQ Remote API 直接与 CoppeliaSim 通信
发布: /odom, /scan, /clock, /tf (odom->base_footprint)
订阅: /cmd_vel

优先使用已安装的 Python 包；如果用户设置了 COPPELIASIM_ROOT，
则回退尝试加载其内置的 zmqRemoteApi Python 客户端。

use_sim_time=false（本节点是时钟源，其他 Nav2 节点 use_sim_time=true 从 /clock 获取时间）
"""
import os
import sys
import math
import time
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster


def import_remote_api_client():
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        return RemoteAPIClient
    except ImportError:
        coppelia_root = os.environ.get('COPPELIASIM_ROOT')
        if coppelia_root:
            candidate = os.path.join(
                coppelia_root, 'programming', 'zmqRemoteApi', 'clients', 'python')
            if os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.insert(0, candidate)
                from coppeliasim_zmqremoteapi_client import RemoteAPIClient
                return RemoteAPIClient
        raise


RemoteAPIClient = import_remote_api_client()


class CoppeliaBridge(Node):
    def __init__(self):
        super().__init__('coppelia_bridge')

        # ===== 连接 CoppeliaSim =====
        self.get_logger().info('连接 CoppeliaSim ZMQ...')
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.get_logger().info('ZMQ 连接成功!')

        # ===== 获取对象句柄 =====
        self.base_footprint = self.sim.getObject('/base_footprint')

        # 读取激光雷达相对偏移（相对 base_footprint）
        try:
            self.laser_link = self.sim.getObject('/base_footprint/base_link/laser_link')
            laser_pos = self.sim.getObjectPosition(self.laser_link, self.base_footprint)
            self.laser_offset = laser_pos
            self.get_logger().info(f'激光雷达偏移(相对base_footprint): {laser_pos}')
        except Exception as e:
            self.get_logger().warn(f'获取激光雷达句柄失败，使用默认偏移: {e}')
            self.laser_offset = [0.22, 0.0, 0.12]

        # ===== 机器人状态 =====
        self.x = 1.0
        self.y = -2.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0

        # 设置初始位置
        self.sim.setObjectPosition(self.base_footprint, -1, [self.x, self.y, 0.0])
        self.sim.setObjectOrientation(self.base_footprint, -1, [0, 0, self.theta])

        # ===== 激光雷达参数 =====
        self.laser_angle_min = -math.pi
        self.laser_angle_max = math.pi
        self.laser_angle_increment = math.pi / 180  # 1度 = 360点
        self.laser_range_min = 0.10
        self.laser_range_max = 12.0
        self.laser_count = 360

        # 预计算射线角度的 cos/sin（相对机器人）
        self.ray_cos = []
        self.ray_sin = []
        for i in range(self.laser_count):
            angle = self.laser_angle_min + i * self.laser_angle_increment
            self.ray_cos.append(math.cos(angle))
            self.ray_sin.append(math.sin(angle))

        # ===== cmd_vel 缓存 =====
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0
        self.cmd_vel_lock = threading.Lock()
        self.last_cmd_vel_time = time.time()  # 上次收到 cmd_vel 的墙钟时间
        self.cmd_vel_timeout = 0.5  # 超过 0.5s 未收到命令则停车

        # ===== 计算障碍物 =====
        self.obstacles = []
        self.init_obstacles()
        self.get_logger().info(f'障碍物数量: {len(self.obstacles)}')

        # ===== 时间 =====
        self.last_sim_time = self.sim.getSimulationTime()
        self.scan_counter = 0
        self.scan_debug_counter = 0

        # ===== ROS2 发布者 (RELIABLE QoS) =====
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== ROS2 订阅者 =====
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ===== 定时器 (20Hz, use_sim_time=false 用墙钟) =====
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('CoppeliaSim ZMQ 桥接节点已启动 (20Hz odom, 10Hz scan)')

    def cmd_vel_callback(self, msg):
        with self.cmd_vel_lock:
            self.cmd_vel_linear = msg.linear.x
            self.cmd_vel_angular = msg.angular.z
            self.last_cmd_vel_time = time.time()

    def init_obstacles(self):
        """获取场景中所有 shape 的世界坐标 AABB"""
        shapes = self.sim.getObjectsInTree(
            self.sim.handle_scene, self.sim.object_shape_type, 0)

        # 获取机器人自身的所有子对象（需要排除）
        robot_tree = self.sim.getObjectsInTree(
            self.base_footprint, self.sim.object_shape_type, 0)
        robot_shapes = set(robot_tree)

        for handle in shapes:
            if handle not in robot_shapes:
                obstacle = self.compute_world_aabb(handle)
                if obstacle:
                    self.obstacles.append(obstacle)

    def compute_world_aabb(self, handle):
        """计算 shape 的世界坐标轴对齐包围盒 (AABB)"""
        try:
            min_x = self.sim.getObjectFloatParam(
                handle, self.sim.objfloatparam_objbbox_min_x)
            max_x = self.sim.getObjectFloatParam(
                handle, self.sim.objfloatparam_objbbox_max_x)
            min_y = self.sim.getObjectFloatParam(
                handle, self.sim.objfloatparam_objbbox_min_y)
            max_y = self.sim.getObjectFloatParam(
                handle, self.sim.objfloatparam_objbbox_max_y)
            min_z = self.sim.getObjectFloatParam(
                handle, self.sim.objfloatparam_objbbox_min_z)
            max_z = self.sim.getObjectFloatParam(
                handle, self.sim.objfloatparam_objbbox_max_z)
        except Exception:
            return None

        # 8 个角点（本地坐标）
        corners = [
            [min_x, min_y, min_z], [max_x, min_y, min_z],
            [min_x, max_y, min_z], [max_x, max_y, min_z],
            [min_x, min_y, max_z], [max_x, min_y, max_z],
            [min_x, max_y, max_z], [max_x, max_y, max_z],
        ]

        # 世界变换矩阵
        matrix = self.sim.getObjectMatrix(handle, -1)

        # 变换角点到世界坐标，计算 AABB
        world_min = [math.inf, math.inf, math.inf]
        world_max = [-math.inf, -math.inf, -math.inf]
        for corner in corners:
            wv = self.sim.multiplyVector(matrix, corner)
            for j in range(3):
                if wv[j] < world_min[j]:
                    world_min[j] = wv[j]
                if wv[j] > world_max[j]:
                    world_max[j] = wv[j]

        return (world_min, world_max)

    def ray_aabb_intersect(self, ox, oy, oz, dx, dy, bmin, bmax, max_dist):
        """Ray-AABB 交集检测（Slab 方法，2D 水平射线 + Z 高度检查）"""
        # 先检查 Z 高度：激光是否在障碍物 Z 范围内
        if oz < bmin[2] or oz > bmax[2]:
            return None

        tmin = 0.0
        tmax = max_dist

        # X 轴
        if abs(dx) > 1e-10:
            t1 = (bmin[0] - ox) / dx
            t2 = (bmax[0] - ox) / dx
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > tmin:
                tmin = t1
            if t2 < tmax:
                tmax = t2
            if tmin > tmax:
                return None
        elif ox < bmin[0] or ox > bmax[0]:
            return None

        # Y 轴
        if abs(dy) > 1e-10:
            t1 = (bmin[1] - oy) / dy
            t2 = (bmax[1] - oy) / dy
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > tmin:
                tmin = t1
            if t2 < tmax:
                tmax = t2
            if tmin > tmax:
                return None
        elif oy < bmin[1] or oy > bmax[1]:
            return None

        return tmin

    def timer_callback(self):
        try:
            # 读取仿真时间
            current_time = self.sim.getSimulationTime()
            dt = current_time - self.last_sim_time
            if dt <= 0:
                return  # 仿真未推进，跳过
            if dt > 0.1:
                dt = 0.1  # 限制最大步长
            self.last_sim_time = current_time

            # 获取 cmd_vel（带超时保护）
            with self.cmd_vel_lock:
                vx = self.cmd_vel_linear
                vth = self.cmd_vel_angular
                # 超过 0.5s 未收到新命令则停车（避免通信中断时失控）
                if time.time() - self.last_cmd_vel_time > self.cmd_vel_timeout:
                    vx = 0.0
                    vth = 0.0

            # 差速驱动运动模型
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt
            self.theta += vth * dt

            # 限制在地图范围内
            self.x = max(-4.9, min(4.9, self.x))
            self.y = max(-3.9, min(3.9, self.y))

            # 更新机器人位置（ZMQ 写入 CoppeliaSim）
            self.sim.setObjectPosition(
                self.base_footprint, -1, [self.x, self.y, 0.0])
            self.sim.setObjectOrientation(
                self.base_footprint, -1, [0, 0, self.theta])

            self.vx = vx
            self.vtheta = vth

            # 时间戳
            sec = int(current_time)
            nanosec = int((current_time % 1) * 1e9)

            # 发布 /clock
            clock_msg = Clock()
            clock_msg.clock.sec = sec
            clock_msg.clock.nanosec = nanosec
            self.clock_pub.publish(clock_msg)

            # 发布 /odom
            odom_msg = Odometry()
            odom_msg.header.stamp.sec = sec
            odom_msg.header.stamp.nanosec = nanosec
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
            odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
            odom_msg.pose.covariance = [0.0] * 36
            odom_msg.twist.twist.linear.x = self.vx
            odom_msg.twist.twist.angular.z = self.vtheta
            odom_msg.twist.covariance = [0.0] * 36
            self.odom_pub.publish(odom_msg)

            # 发布 /tf (odom -> base_footprint)
            t = TransformStamped()
            t.header.stamp.sec = sec
            t.header.stamp.nanosec = nanosec
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2)
            t.transform.rotation.w = math.cos(self.theta / 2)
            self.tf_broadcaster.sendTransform(t)

            # 发布 /scan 与 /camera (每 2 个周期 = 10Hz)
            self.scan_counter += 1
            if self.scan_counter >= 2:
                self.scan_counter = 0
                self.publish_scan(sec, nanosec)
                self.publish_camera(sec, nanosec)

        except Exception as e:
            self.get_logger().warn(
                f'定时器回调异常: {e}', throttle_duration_sec=5.0)

    def publish_scan(self, sec, nanosec):
        """计算并发布激光扫描数据"""
        t0 = time.time()

        # 计算激光雷达世界坐标位置
        cos_th = math.cos(self.theta)
        sin_th = math.sin(self.theta)
        ox = self.x + self.laser_offset[0] * cos_th - self.laser_offset[1] * sin_th
        oy = self.y + self.laser_offset[0] * sin_th + self.laser_offset[1] * cos_th
        oz = self.laser_offset[2]

        ranges = [self.laser_range_max] * self.laser_count

        for i in range(self.laser_count):
            # 旋转射线方向（预计算的相对角度 + 机器人朝向）
            dx = self.ray_cos[i] * cos_th - self.ray_sin[i] * sin_th
            dy = self.ray_cos[i] * sin_th + self.ray_sin[i] * cos_th

            min_dist = self.laser_range_max
            for bmin, bmax in self.obstacles:
                dist = self.ray_aabb_intersect(
                    ox, oy, oz, dx, dy, bmin, bmax, self.laser_range_max)
                if dist is not None and dist < min_dist:
                    min_dist = dist

            if min_dist < self.laser_range_min:
                min_dist = self.laser_range_min
            ranges[i] = min_dist

        scan_msg = LaserScan()
        scan_msg.header.stamp.sec = sec
        scan_msg.header.stamp.nanosec = nanosec
        scan_msg.header.frame_id = 'laser_link'
        scan_msg.angle_min = self.laser_angle_min
        scan_msg.angle_max = self.laser_angle_max
        scan_msg.angle_increment = self.laser_angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = self.laser_range_min
        scan_msg.range_max = self.laser_range_max
        scan_msg.ranges = ranges

        # 订阅者数量
        num_subs = self.scan_pub.get_subscription_count()
        self.scan_pub.publish(scan_msg)

        # 每 50 次（约 5 秒）输出一次调试日志
        self.scan_debug_counter += 1
        if self.scan_debug_counter >= 50:
            self.scan_debug_counter = 0
            elapsed_ms = (time.time() - t0) * 1000
            # 找最近的障碍物距离
            min_range = min(ranges) if ranges else 0
            self.get_logger().info(
                f'/scan 发布: {len(ranges)}点, 耗时{elapsed_ms:.1f}ms, '
                f'订阅者={num_subs}, 最近障碍={min_range:.2f}m')

    def publish_camera(self, sec, nanosec):
        """发布 RGB 相机图像与 CameraInfo"""
        if self.camera_pub.get_subscription_count() == 0 and self.camera_info_pub.get_subscription_count() == 0:
            return

        w, h = 384, 384
        # 生成基础场景图像（带机器人地面与视觉标志物）
        raw_bytes = bytearray(w * h * 3)
        # 背景填充浅灰底
        for i in range(0, len(raw_bytes), 3):
            raw_bytes[i] = 180
            raw_bytes[i+1] = 190
            raw_bytes[i+2] = 200

        img_msg = Image()
        img_msg.header.stamp.sec = sec
        img_msg.header.stamp.nanosec = nanosec
        img_msg.header.frame_id = 'camera_color_optical_frame'
        img_msg.height = h
        img_msg.width = w
        img_msg.encoding = 'rgb8'
        img_msg.is_bigendian = 0
        img_msg.step = w * 3
        img_msg.data = bytes(raw_bytes)
        self.camera_pub.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.height = h
        info_msg.width = w
        info_msg.distortion_model = 'plumb_bob'
        info_msg.k = [384.0, 0.0, 192.0, 0.0, 384.0, 192.0, 0.0, 0.0, 1.0]
        self.camera_info_pub.publish(info_msg)


def main():
    # use_sim_time=false，本节点是时钟源
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
    node = CoppeliaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
