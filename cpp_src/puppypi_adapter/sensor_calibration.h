// 传感器校准模块 (Sensor Calibration)
//
// 实现项目内存中的硬性要求:
//   - 传感器校准 (LiDAR-IMU 外参、相机内参、里程计)
//
// 校准内容:
//   1. LiDAR-IMU 外参标定: LiDAR 相对 IMU 的刚体变换 (R, t)
//      方法: 静止状态下用重力对齐 + 运动状态下用手眼标定
//   2. 相机内参标定: 相机内参矩阵 K (fx, fy, cx, cy) + 畸变系数
//      方法: 棋盘格标定法 (Zhang 2000)
//   3. 里程计标定: 轮径、轮距、编码器比例因子
//      方法: 已知距离直线行驶 → 比例因子
//
// 注意: 当前 C++ 版本不依赖 OpenCV/numpy，提供核心算法框架。
//       相机标定的角点检测与单应性求解在 .cpp 中以占位实现给出，
//       实际部署时可集成 OpenCV (cv::calibrateCamera)。
//
// 对应 Python: puppypi_adapter/sensor_calibration.py
// 纯工具类（非 ROS2 节点），不包含 main()
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace puppypi_adapter {

/// 3x3 矩阵（行主序）
using Matrix3 = std::array<std::array<double, 3>, 3>;
/// 3x1 向量
using Vector3 = std::array<double, 3>;
/// 5 维向量（畸变系数）
using Vector5 = std::array<double, 5>;

/// LiDAR-IMU 外参标定结果（对应 Python: ExtrinsicCalibration）
struct ExtrinsicCalibration {
    Matrix3 rotation;            // 3x3 旋转矩阵 (IMU → LiDAR)
    Vector3 translation;         // 3x1 平移向量 (m)
    Vector3 euler_angles;        // [roll, pitch, yaw] (rad)
    double timestamp{0.0};
    std::string method;
    double residual{0.0};

    ExtrinsicCalibration();
};

/// 相机内参标定结果（对应 Python: CameraIntrinsics）
struct CameraIntrinsics {
    Matrix3 K;                   // 内参矩阵
    Vector5 distortion;           // 畸变系数 [k1, k2, p1, p2, k3]
    std::array<int, 2> image_size;  // (width, height)
    double reprojection_error{0.0};
    double timestamp{0.0};
    std::string method;

    CameraIntrinsics();

    double fx() const { return K[0][0]; }
    double fy() const { return K[1][1]; }
    double cx() const { return K[0][2]; }
    double cy() const { return K[1][2]; }
};

/// 里程计标定结果（对应 Python: OdometryCalibration）
struct OdometryCalibration {
    double wheel_diameter{0.065};  // 轮子直径 (m)
    double wheelbase{0.20};        // 轮距 (m)
    double left_scale{1.0};        // 左轮编码器比例因子
    double right_scale{1.0};       // 右轮编码器比例因子
    double drift_rate{0.0};        // 直线漂移率 (m/m)
    double angular_bias{0.0};       // 角度偏差 (rad)
    double timestamp{0.0};
    std::string method;
    double residual{0.0};
};

/// 传感器校准器（对应 Python: SensorCalibrator）
///
/// 提供 LiDAR-IMU 外参、相机内参、里程计的标定功能。
/// 所有标定结果可保存为 JSON 文件并加载复用。
class SensorCalibrator {
public:
    /// 构造函数
    /// @param calibration_dir 标定结果保存目录
    explicit SensorCalibrator(const std::string& calibration_dir =
                                  "config/calibration");

    // === LiDAR-IMU 外参标定 ===
    /// 静止状态外参标定（重力对齐法）
    /// @param imu_accel_gravity 静止时 IMU 加速度读数 [ax, ay, az]
    /// @param lidar_mount_angle LiDAR 安装角度偏差 (yaw, rad)
    ExtrinsicCalibration calibrate_lidar_imu_static(
        const Vector3& imu_accel_gravity,
        double lidar_mount_angle = 0.0);

    /// 运动状态外参标定（手眼标定法，Tsai-Lenz 简化版）
    /// @param imu_rotations IMU 旋转序列（每个为 3x3 矩阵）
    /// @param lidar_rotations LiDAR 旋转序列
    ExtrinsicCalibration calibrate_lidar_imu_handeye(
        const std::vector<Matrix3>& imu_rotations,
        const std::vector<Matrix3>& lidar_rotations);

    // === 相机内参标定 ===
    /// 相机内参标定（棋盘格法，Zhang 2000）
    /// @param image_width 图像宽度
    /// @param image_height 图像高度
    /// @param board_cols 棋盘格内角点列数
    /// @param board_rows 棋盘格内角点行数
    /// @param square_size 棋盘格方格边长 (m)
    /// @param num_images 用于标定的图像数（占位用，实际角点未检测）
    /// @return 标定结果（无 OpenCV 时返回估算默认值）
    CameraIntrinsics calibrate_camera(
        int image_width, int image_height,
        int board_cols = 9, int board_rows = 6,
        double square_size = 0.025,
        int num_images = 0);

    // === 里程计标定 ===
    /// 里程计标定
    /// @param encoder_readings 编码器读数 [(left_count, right_count), ...]
    /// @param known_distance 已知行驶距离 (m)
    /// @param wheel_diameter 轮子直径 (m)
    OdometryCalibration calibrate_odometry(
        const std::vector<std::pair<int, int>>& encoder_readings,
        double known_distance,
        double wheel_diameter = 0.065);

    // === 标定结果保存/加载 ===
    /// 保存标定结果到文件 (calibration_<name>.json)
    /// @param name 标定配置名
    void save_calibration(const std::string& name = "default");
    /// 从文件加载标定结果
    /// @return true 如果加载成功
    bool load_calibration(const std::string& name = "default");

    /// 是否已完成所有标定
    bool is_calibrated() const {
        return extrinsic_set_ && intrinsics_set_;
    }

    // === 标定结果访问 ===
    const ExtrinsicCalibration* extrinsic() const {
        return extrinsic_set_ ? &extrinsic_ : nullptr;
    }
    const CameraIntrinsics* intrinsics() const {
        return intrinsics_set_ ? &intrinsics_ : nullptr;
    }
    const OdometryCalibration* odometry() const {
        return odometry_set_ ? &odometry_ : nullptr;
    }

private:
    std::string calibration_dir_;
    ExtrinsicCalibration extrinsic_;
    bool extrinsic_set_{false};
    CameraIntrinsics intrinsics_;
    bool intrinsics_set_{false};
    OdometryCalibration odometry_;
    bool odometry_set_{false};
};

}  // namespace puppypi_adapter
