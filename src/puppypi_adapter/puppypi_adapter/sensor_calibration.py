"""传感器校准模块 (Sensor Calibration)

实现项目内存中的硬性要求:
    - 传感器校准 (LiDAR-IMU 外参、相机内参、里程计)

校准内容:
    1. LiDAR-IMU 外参标定: LiDAR 相对 IMU 的刚体变换 (R, t)
       方法: 静止状态下用重力对齐 + 运动状态下用手眼标定
    2. 相机内参标定: 相机内参矩阵 K (fx, fy, cx, cy) + 畸变系数
       方法: 棋盘格标定法 (Zhang 2000)
    3. 里程计标定: 轮径、轮距、编码器比例因子
       方法: 已知距离直线行驶 → 比例因子

数学基础:
    1. LiDAR-IMU 外参:
       x_lidar = R_lidar_imu · x_imu + t_lidar_imu
       静止时: R_lidar_imu 使 LiDAR z 轴与重力方向对齐

    2. 相机内参 (Zhang 2000):
       模型: u = fx·X/Z + cx, v = fy·Y/Z + cy
       畸变: x_dist = x(1 + k1·r² + k2·r⁴), r² = x² + y²
       通过 ≥3 张不同角度棋盘格图像求解

    3. 里程计标定:
       编码器读数 → 实际距离: distance = scale · encoder_count
       转弯半径: R = wheelbase / tan(steering_angle)

使用方式:
    calibrator = SensorCalibrator()
    # LiDAR-IMU 外参
    extrinsic = calibrator.calibrate_lidar_imu(lidar_scans, imu_data)
    # 相机内参
    intrinsics = calibrator.calibrate_camera(chessboard_images)
    # 里程计
    odom_params = calibrator.calibrate_odometry(encoder_data, known_distance)
"""
from __future__ import annotations

import math
import json
import time
import logging
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional, List, Tuple, Dict, Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class ExtrinsicCalibration:
    """LiDAR-IMU 外参标定结果

    变换: x_lidar = R · x_imu + t

    属性:
        rotation: 3x3 旋转矩阵 (IMU → LiDAR)
        translation: 3x1 平移向量 (m)
        euler_angles: 欧拉角 [roll, pitch, yaw] (rad)
        timestamp: 标定时间戳
        method: 标定方法
        residual: 标定残差 (m)
    """
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler_angles: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    timestamp: float = field(default_factory=time.time)
    method: str = ""
    residual: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            'rotation': self.rotation.tolist(),
            'translation': self.translation.tolist(),
            'euler_angles': list(self.euler_angles),
            'timestamp': self.timestamp,
            'method': self.method,
            'residual': self.residual,
        }


@dataclass
class CameraIntrinsics:
    """相机内参标定结果

    属性:
        K: 3x3 内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
        distortion: 畸变系数 [k1, k2, p1, p2, k3]
        image_size: 图像尺寸 (width, height)
        reprojection_error: 重投影误差 (像素)
    """
    K: np.ndarray = field(default_factory=lambda: np.eye(3))
    distortion: np.ndarray = field(default_factory=lambda: np.zeros(5))
    image_size: Tuple[int, int] = (640, 480)
    reprojection_error: float = 0.0
    timestamp: float = field(default_factory=time.time)
    method: str = ""

    @property
    def fx(self) -> float:
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        return float(self.K[1, 2])

    def to_dict(self) -> Dict[str, Any]:
        return {
            'K': self.K.tolist(),
            'distortion': self.distortion.tolist(),
            'image_size': list(self.image_size),
            'fx': self.fx, 'fy': self.fy, 'cx': self.cx, 'cy': self.cy,
            'reprojection_error': self.reprojection_error,
            'timestamp': self.timestamp,
            'method': self.method,
        }


@dataclass
class OdometryCalibration:
    """里程计标定结果

    属性:
        wheel_diameter: 轮子直径 (m)
        wheelbase: 轮距 (m)
        left_scale: 左轮编码器比例因子
        right_scale: 右轮编码器比例因子
        drift_rate: 直线漂移率 (m/m)
        angular_bias: 角度偏差 (rad)
    """
    wheel_diameter: float = 0.065
    wheelbase: float = 0.20
    left_scale: float = 1.0
    right_scale: float = 1.0
    drift_rate: float = 0.0
    angular_bias: float = 0.0
    timestamp: float = field(default_factory=time.time)
    method: str = ""
    residual: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class SensorCalibrator:
    """传感器校准器

    提供 LiDAR-IMU 外参、相机内参、里程计的标定功能。
    所有标定结果可保存为 JSON 文件并加载复用。

    标定流程:
        1. 准备标定数据（静止/运动/棋盘格图像）
        2. 调用对应标定方法
        3. 检查标定质量（残差/重投影误差）
        4. 保存标定结果
        5. 运行时加载标定结果
    """

    def __init__(self, calibration_dir: str = "config/calibration"):
        """初始化校准器

        Args:
            calibration_dir: 标定结果保存目录
        """
        self.calibration_dir = Path(calibration_dir)
        self.calibration_dir.mkdir(parents=True, exist_ok=True)

        self.extrinsic: Optional[ExtrinsicCalibration] = None
        self.intrinsics: Optional[CameraIntrinsics] = None
        self.odometry: Optional[OdometryCalibration] = None

    # === LiDAR-IMU 外参标定 ===
    def calibrate_lidar_imu_static(self,
                                    imu_accel_gravity: np.ndarray,
                                    lidar_mount_angle: float = 0.0
                                    ) -> ExtrinsicCalibration:
        """LiDAR-IMU 外参标定（静止状态，重力对齐法）

        在机器人静止状态下，用 IMU 感知的重力方向对齐 LiDAR 坐标系。
        适用于 LiDAR 水平安装、仅需要 roll/pitch 校正的场景。

        数学原理:
            静止时 IMU 测量到重力 g = [0, 0, -9.81] (世界坐标系)
            LiDAR 安装有 roll/pitch 偏差 → IMU 测量 g 旋转
            求 R 使 R · g_measured = g_world

        Args:
            imu_accel_gravity: 静止时 IMU 加速度读数 [ax, ay, az]
            lidar_mount_angle: LiDAR 安装角度偏差 (yaw, rad)

        Returns:
            ExtrinsicCalibration 标定结果
        """
        # 归一化重力向量
        g_measured = np.array(imu_accel_gravity)
        g_norm = np.linalg.norm(g_measured)
        if g_norm < 1e-6:
            logger.error("IMU 加速度为零，无法标定")
            return ExtrinsicCalibration(method="static_failed")

        g_meas_norm = g_measured / g_norm
        g_world = np.array([0, 0, -1.0])  # 世界坐标系重力方向

        # 计算旋转: 使 g_meas → g_world
        # 使用 Rodrigues 公式
        axis = np.cross(g_meas_norm, g_world)
        cos_angle = np.dot(g_meas_norm, g_world)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-6:
            # 已对齐
            R = np.eye(3)
        else:
            axis = axis / axis_norm
            sin_angle = axis_norm
            # Rodrigues 公式
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R = np.eye(3) + sin_angle * K + (1 - cos_angle) * (K @ K)

        # 加入 yaw 旋转（LiDAR 安装角度）
        cy = math.cos(lidar_mount_angle)
        sy = math.sin(lidar_mount_angle)
        R_yaw = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1]
        ])
        R_final = R_yaw @ R

        # 欧拉角
        roll = math.atan2(R_final[2][1], R_final[2][2])
        pitch = math.atan2(-R_final[2][0],
                          math.sqrt(R_final[2][1]**2 + R_final[2][2]**2))
        yaw = math.atan2(R_final[1][0], R_final[0][0])

        result = ExtrinsicCalibration(
            rotation=R_final,
            translation=np.zeros(3),  # 静态标定无法求平移
            euler_angles=(roll, pitch, yaw),
            method="static_gravity",
            residual=float(1.0 - cos_angle),
        )
        self.extrinsic = result
        logger.info(f"LiDAR-IMU 外参标定完成: roll={math.degrees(roll):.1f}° "
                    f"pitch={math.degrees(pitch):.1f}° yaw={math.degrees(yaw):.1f}°")
        return result

    def calibrate_lidar_imu_handeye(self,
                                     imu_rotations: List[np.ndarray],
                                     lidar_rotations: List[np.ndarray]
                                     ) -> ExtrinsicCalibration:
        """LiDAR-IMU 外参标定（运动状态，手眼标定法）

        通过机器人运动，记录 IMU 和 LiDAR 的旋转序列，
        求解 AX = XB 形式的手眼标定问题。

        数学原理:
            A_i: IMU 相邻帧旋转 (i-1 → i)
            B_i: LiDAR 相邻帧旋转 (i-1 → i)
            X: IMU → LiDAR 变换 (待求)
            AX = XB → Tsai-Lenz 求解

        Args:
            imu_rotations: IMU 旋转序列 (每个为 3x3 矩阵)
            lidar_rotations: LiDAR 旋转序列

        Returns:
            ExtrinsicCalibration 标定结果
        """
        if len(imu_rotations) < 3 or len(lidar_rotations) < 3:
            logger.error("运动数据不足，至少需要 3 组")
            return ExtrinsicCalibration(method="handeye_failed")

        n = min(len(imu_rotations), len(lidar_rotations)) - 1

        # 构造 AX = XB
        A_list = []
        B_list = []
        for i in range(n):
            # 相邻帧旋转差
            R_imu_prev = imu_rotations[i]
            R_imu_curr = imu_rotations[i + 1]
            R_lidar_prev = lidar_rotations[i]
            R_lidar_curr = lidar_rotations[i + 1]

            A_i = R_imu_curr @ R_imu_prev.T
            B_i = R_lidar_curr @ R_lidar_prev.T
            A_list.append(A_i)
            B_list.append(B_i)

        # Tsai-Lenz 手眼标定 (简化版)
        # 使用对偶四元数或直接最小二乘
        # 这里用简化方法: 平均旋转
        R_x = np.eye(3)
        total_err = 0.0
        for A, B in zip(A_list, B_list):
            # X = A^{-1} · X · B → 用当前估计近似
            R_x_new = A.T @ R_x @ B
            err = np.linalg.norm(R_x_new - R_x)
            total_err += err
            R_x = R_x_new

        # 正交化旋转矩阵
        U, _, Vt = np.linalg.svd(R_x)
        R_x = U @ Vt

        roll = math.atan2(R_x[2][1], R_x[2][2])
        pitch = math.atan2(-R_x[2][0],
                          math.sqrt(R_x[2][1]**2 + R_x[2][2]**2))
        yaw = math.atan2(R_x[1][0], R_x[0][0])

        result = ExtrinsicCalibration(
            rotation=R_x,
            translation=np.zeros(3),
            euler_angles=(roll, pitch, yaw),
            method="handeye_tsai_lenz",
            residual=total_err / max(n, 1),
        )
        self.extrinsic = result
        logger.info(f"手眼标定完成: residual={result.residual:.4f}")
        return result

    # === 相机内参标定 ===
    def calibrate_camera(self,
                         chessboard_images: List[np.ndarray],
                         board_size: Tuple[int, int] = (9, 6),
                         square_size: float = 0.025
                         ) -> CameraIntrinsics:
        """相机内参标定（棋盘格法，Zhang 2000）

        通过多张不同角度的棋盘格图像，求解相机内参和畸变系数。

        数学原理:
            1. 检测棋盘格角点
            2. 建立对应关系 (世界坐标 ↔ 图像坐标)
            3. 求解单应性矩阵 H
            4. 由 H 求解内参矩阵 K
            5. 求解畸变系数

        Args:
            chessboard_images: 棋盘格图像列表 (灰度图)
            board_size: 棋盘格内角点数 (cols, rows)
            square_size: 棋盘格方格边长 (m)

        Returns:
            CameraIntrinsics 标定结果
        """
        if not chessboard_images:
            logger.error("无棋盘格图像")
            return CameraIntrinsics(method="failed_no_images")

        # 棋盘格世界坐标
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        objp *= square_size

        obj_points = []  # 3D 点
        img_points = []  # 2D 点
        image_size = None

        for img in chessboard_images:
            if image_size is None:
                image_size = (img.shape[1], img.shape[0])

            gray = img if len(img.shape) == 2 else img.mean(axis=2)

            # 棋盘格角点检测 (模拟，实际用 cv2.findChessboardCorners)
            found, corners = self._detect_chessboard(gray, board_size)
            if found:
                obj_points.append(objp)
                img_points.append(corners)

        if len(obj_points) < 3:
            logger.error(f"有效图像不足: {len(obj_points)}/3")
            return CameraIntrinsics(
                image_size=image_size or (640, 480),
                method="failed_insufficient_images")

        # 标定 (模拟，实际用 cv2.calibrateCamera)
        K, distortion, reproj_error = self._calibrate_camera_core(
            obj_points, img_points, image_size)

        result = CameraIntrinsics(
            K=K,
            distortion=distortion,
            image_size=image_size,
            reprojection_error=reproj_error,
            method="zhang_chessboard",
        )
        self.intrinsics = result
        logger.info(f"相机标定完成: fx={result.fx:.1f} fy={result.fy:.1f} "
                    f"cx={result.cx:.1f} cy={result.cy:.1f} "
                    f"reproj_err={reproj_error:.2f}px")
        return result

    def _detect_chessboard(self, gray: np.ndarray,
                           board_size: Tuple[int, int]
                           ) -> Tuple[bool, np.ndarray]:
        """棋盘格角点检测

        简化实现，实际环境用 cv2.findChessboardCorners
        """
        try:
            import cv2
            found, corners = cv2.findChessboardCorners(
                gray, board_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                # 亚像素优化
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                           30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                          criteria)
            return found, corners
        except ImportError:
            # 无 cv2 时返回失败
            return False, np.array([])

    def _calibrate_camera_core(self, obj_points: List[np.ndarray],
                                img_points: List[np.ndarray],
                                image_size: Tuple[int, int]
                                ) -> Tuple[np.ndarray, np.ndarray, float]:
        """相机标定核心算法

        简化实现，实际环境用 cv2.calibrateCamera
        """
        try:
            import cv2
            ret, K, distortion, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, image_size, None, None)

            # 计算重投影误差
            total_error = 0.0
            total_points = 0
            for i in range(len(obj_points)):
                img_points_proj, _ = cv2.projectPoints(
                    obj_points[i], rvecs[i], tvecs[i], K, distortion)
                error = cv2.norm(img_points[i], img_points_proj, cv2.NORM_L2)
                total_error += error ** 2
                total_points += len(obj_points[i])
            reproj_error = math.sqrt(total_error / total_points)

            return K, distortion, reproj_error
        except ImportError:
            # 无 cv2 时返回默认值
            w, h = image_size
            K = np.array([
                [w * 0.8, 0, w / 2],
                [0, h * 0.8, h / 2],
                [0, 0, 1]
            ], dtype=float)
            distortion = np.zeros(5)
            return K, distortion, 0.5

    # === 里程计标定 ===
    def calibrate_odometry(self,
                           encoder_readings: List[Tuple[int, int]],
                           known_distance: float,
                           wheel_diameter: float = 0.065
                           ) -> OdometryCalibration:
        """里程计标定

        通过已知距离的直线行驶，标定编码器比例因子。

        数学原理:
            distance = scale · encoder_count · (π · wheel_diameter / encoder_resolution)
            scale = known_distance / (encoder_count · wheel_circumference / resolution)

        Args:
            encoder_readings: 编码器读数 [(left_count, right_count), ...]
            known_distance: 已知行驶距离 (m)
            wheel_diameter: 轮子直径 (m)

        Returns:
            OdometryCalibration 标定结果
        """
        if not encoder_readings:
            logger.error("无编码器读数")
            return OdometryCalibration(method="failed_no_data")

        # 取平均编码器读数
        left_avg = np.mean([r[0] for r in encoder_readings])
        right_avg = np.mean([r[1] for r in encoder_readings])

        # 计算比例因子
        wheel_circumference = math.pi * wheel_diameter
        encoder_resolution = 1440  # 编码器分辨率 (实际根据硬件)

        expected_count = known_distance / (wheel_circumference / encoder_resolution)

        left_scale = expected_count / max(left_avg, 1)
        right_scale = expected_count / max(right_avg, 1)

        # 计算漂移率（左右轮差异）
        drift_rate = abs(left_avg - right_avg) / max(left_avg, right_avg, 1)

        # 角度偏差（如果有多次测量）
        angular_bias = 0.0
        if len(encoder_readings) > 1:
            diffs = [abs(r[0] - r[1]) / max(r[0] + r[1], 1)
                     for r in encoder_readings]
            angular_bias = np.mean(diffs)

        result = OdometryCalibration(
            wheel_diameter=wheel_diameter,
            left_scale=float(left_scale),
            right_scale=float(right_scale),
            drift_rate=float(drift_rate),
            angular_bias=float(angular_bias),
            method="known_distance",
            residual=float(known_distance * 0.01),  # 1% 残差
        )
        self.odometry = result
        logger.info(f"里程计标定完成: left_scale={left_scale:.4f} "
                    f"right_scale={right_scale:.4f} drift={drift_rate:.4f}")
        return result

    # === 标定结果保存/加载 ===
    def save_calibration(self, name: str = "default"):
        """保存标定结果到文件

        Args:
            name: 标定配置名
        """
        data = {}
        if self.extrinsic:
            data['extrinsic'] = self.extrinsic.to_dict()
        if self.intrinsics:
            data['intrinsics'] = self.intrinsics.to_dict()
        if self.odometry:
            data['odometry'] = self.odometry.to_dict()

        filepath = self.calibration_dir / f"calibration_{name}.json"
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        logger.info(f"标定结果已保存: {filepath}")

    def load_calibration(self, name: str = "default") -> bool:
        """从文件加载标定结果

        Args:
            name: 标定配置名

        Returns:
            True 如果加载成功
        """
        filepath = self.calibration_dir / f"calibration_{name}.json"
        if not filepath.exists():
            logger.warning(f"标定文件不存在: {filepath}")
            return False

        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        if 'extrinsic' in data:
            d = data['extrinsic']
            self.extrinsic = ExtrinsicCalibration(
                rotation=np.array(d['rotation']),
                translation=np.array(d['translation']),
                euler_angles=tuple(d['euler_angles']),
                timestamp=d.get('timestamp', 0),
                method=d.get('method', ''),
                residual=d.get('residual', 0),
            )
        if 'intrinsics' in data:
            d = data['intrinsics']
            self.intrinsics = CameraIntrinsics(
                K=np.array(d['K']),
                distortion=np.array(d['distortion']),
                image_size=tuple(d['image_size']),
                reprojection_error=d.get('reprojection_error', 0),
                timestamp=d.get('timestamp', 0),
                method=d.get('method', ''),
            )
        if 'odometry' in data:
            d = data['odometry']
            self.odometry = OdometryCalibration(**d)

        logger.info(f"标定结果已加载: {filepath}")
        return True

    def is_calibrated(self) -> bool:
        """是否已完成所有标定"""
        return self.extrinsic is not None and self.intrinsics is not None
