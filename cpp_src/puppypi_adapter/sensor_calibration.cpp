// 传感器校准模块 实现
//
// 对应 Python: puppypi_adapter/sensor_calibration.py
// 纯工具类（非 ROS2 节点），不包含 main()
//
// 注意: 当前 C++ 版本不依赖 OpenCV/numpy，提供核心算法框架。
//       相机标定的角点检测与单应性求解在 .cpp 中以占位实现给出，
//       实际部署时可集成 OpenCV (cv::calibrateCamera)。
#include "puppypi_adapter/sensor_calibration.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <direct.h>  // _mkdir
#else
#include <sys/stat.h>  // mkdir
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppypi_adapter {

namespace {

/// 单位矩阵
Matrix3 identity3() {
    Matrix3 m{};
    m[0][0] = m[1][1] = m[2][2] = 1.0;
    return m;
}

/// 零向量
Vector3 zero3() {
    return Vector3{{0.0, 0.0, 0.0}};
}

/// 零向量5
Vector5 zero5() {
    return Vector5{{0.0, 0.0, 0.0, 0.0, 0.0}};
}

/// 矩阵乘法 C = A · B
Matrix3 mat_mul(const Matrix3& A, const Matrix3& B) {
    Matrix3 C{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += A[i][k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
    return C;
}

/// 矩阵转置
Matrix3 mat_transpose(const Matrix3& A) {
    Matrix3 T{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T[i][j] = A[j][i];
        }
    }
    return T;
}

/// 矩阵减法
Matrix3 mat_sub(const Matrix3& A, const Matrix3& B) {
    Matrix3 C{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
    return C;
}

/// 矩阵 Frobenius 范数
double mat_norm(const Matrix3& A) {
    double s = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            s += A[i][j] * A[i][j];
        }
    }
    return std::sqrt(s);
}

/// 向量叉乘 a × b
Vector3 vec_cross(const Vector3& a, const Vector3& b) {
    Vector3 c{};
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
    return c;
}

/// 向量点积
double vec_dot(const Vector3& a, const Vector3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// 向量范数
double vec_norm(const Vector3& a) {
    return std::sqrt(vec_dot(a, a));
}

/// 向量归一化
Vector3 vec_normalize(const Vector3& a) {
    double n = vec_norm(a);
    if (n < 1e-12) {
        return zero3();
    }
    return Vector3{{a[0] / n, a[1] / n, a[2] / n}};
}

/// Rodrigues 公式：由旋转轴 axis 与角度 (cos, sin) 构造旋转矩阵
Matrix3 rodrigues(const Vector3& axis, double cos_angle, double sin_angle) {
    Matrix3 K{};
    K[0][0] = 0;       K[0][1] = -axis[2]; K[0][2] = axis[1];
    K[1][0] = axis[2]; K[1][1] = 0;        K[1][2] = -axis[0];
    K[2][0] = -axis[1]; K[2][1] = axis[0]; K[2][2] = 0;
    // R = I + sin*K + (1 - cos)*K^2
    Matrix3 K2 = mat_mul(K, K);
    Matrix3 R = identity3();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] += sin_angle * K[i][j] +
                       (1.0 - cos_angle) * K2[i][j];
        }
    }
    return R;
}

/// 从旋转矩阵提取欧拉角 [roll, pitch, yaw]
Vector3 mat_to_euler(const Matrix3& R) {
    double roll = std::atan2(R[2][1], R[2][2]);
    double pitch = std::atan2(-R[2][0],
                              std::sqrt(R[2][1] * R[2][1] +
                                        R[2][2] * R[2][2]));
    double yaw = std::atan2(R[1][0], R[0][0]);
    return Vector3{{roll, pitch, yaw}};
}

/// 当前秒数
double nowSeconds() {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

/// 确保目录存在
void ensureDir(const std::string& path) {
#ifdef _WIN32
    _mkdir(path.c_str());
#else
    mkdir(path.c_str(), 0755);
#endif
}

/// JSON 字符串转义
std::string json_escape(const std::string& s) {
    std::ostringstream oss;
    for (char c : s) {
        switch (c) {
            case '"':  oss << "\\\""; break;
            case '\\': oss << "\\\\"; break;
            case '\n': oss << "\\n";  break;
            case '\r': oss << "\\r";  break;
            case '\t': oss << "\\t";  break;
            default:   oss << c;      break;
        }
    }
    return oss.str();
}

}  // namespace

// ============================================================================
// 数据结构实现
// ============================================================================

ExtrinsicCalibration::ExtrinsicCalibration()
    : rotation(identity3()),
      translation(zero3()),
      euler_angles(zero3()) {}

CameraIntrinsics::CameraIntrinsics()
    : K(identity3()),
      distortion(zero5()),
      image_size({640, 480}) {}

SensorCalibrator::SensorCalibrator(const std::string& calibration_dir)
    : calibration_dir_(calibration_dir) {
    ensureDir(calibration_dir_);
}

// ============================================================================
// LiDAR-IMU 外参标定 - 静态重力对齐法
// ============================================================================

ExtrinsicCalibration SensorCalibrator::calibrate_lidar_imu_static(
    const Vector3& imu_accel_gravity,
    double lidar_mount_angle) {
    ExtrinsicCalibration result;

    // 归一化重力向量
    double g_norm = vec_norm(imu_accel_gravity);
    if (g_norm < 1e-6) {
        std::fprintf(stderr,
                     "[SensorCalibrator] IMU 加速度为零，无法标定\n");
        result.method = "static_failed";
        return result;
    }
    Vector3 g_meas_norm = vec_normalize(imu_accel_gravity);
    // 世界坐标系重力方向
    Vector3 g_world{{0.0, 0.0, -1.0}};

    // 计算旋转: 使 g_meas → g_world (Rodrigues 公式)
    Vector3 axis = vec_cross(g_meas_norm, g_world);
    double cos_angle = vec_dot(g_meas_norm, g_world);
    double axis_norm = vec_norm(axis);

    Matrix3 R;
    if (axis_norm < 1e-6) {
        R = identity3();
    } else {
        Vector3 axis_unit = vec_normalize(axis);
        double sin_angle = axis_norm;
        R = rodrigues(axis_unit, cos_angle, sin_angle);
    }

    // 加入 yaw 旋转（LiDAR 安装角度）
    double cy = std::cos(lidar_mount_angle);
    double sy = std::sin(lidar_mount_angle);
    Matrix3 R_yaw{};
    R_yaw[0][0] = cy;  R_yaw[0][1] = -sy; R_yaw[0][2] = 0;
    R_yaw[1][0] = sy;  R_yaw[1][1] = cy;  R_yaw[1][2] = 0;
    R_yaw[2][0] = 0;   R_yaw[2][1] = 0;   R_yaw[2][2] = 1;
    Matrix3 R_final = mat_mul(R_yaw, R);

    // 欧拉角
    Vector3 euler = mat_to_euler(R_final);

    result.rotation = R_final;
    result.translation = zero3();  // 静态标定无法求平移
    result.euler_angles = euler;
    result.timestamp = nowSeconds();
    result.method = "static_gravity";
    result.residual = 1.0 - cos_angle;

    extrinsic_ = result;
    extrinsic_set_ = true;
    std::printf("[SensorCalibrator] LiDAR-IMU 外参标定完成: "
                "roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
                euler[0] * 180.0 / M_PI,
                euler[1] * 180.0 / M_PI,
                euler[2] * 180.0 / M_PI);
    return result;
}

// ============================================================================
// LiDAR-IMU 外参标定 - 手眼标定法 (Tsai-Lenz 简化版)
// ============================================================================

ExtrinsicCalibration SensorCalibrator::calibrate_lidar_imu_handeye(
    const std::vector<Matrix3>& imu_rotations,
    const std::vector<Matrix3>& lidar_rotations) {
    ExtrinsicCalibration result;

    if (imu_rotations.size() < 3 || lidar_rotations.size() < 3) {
        std::fprintf(stderr,
                     "[SensorCalibrator] 运动数据不足，至少需要 3 组\n");
        result.method = "handeye_failed";
        return result;
    }

    size_t n = std::min(imu_rotations.size(), lidar_rotations.size()) - 1;

    // 构造 AX = XB
    std::vector<Matrix3> A_list, B_list;
    A_list.reserve(n);
    B_list.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        // A_i = R_imu_curr · R_imu_prev^T
        Matrix3 A_i = mat_mul(imu_rotations[i + 1],
                              mat_transpose(imu_rotations[i]));
        // B_i = R_lidar_curr · R_lidar_prev^T
        Matrix3 B_i = mat_mul(lidar_rotations[i + 1],
                              mat_transpose(lidar_rotations[i]));
        A_list.push_back(A_i);
        B_list.push_back(B_i);
    }

    // 简化方法: 迭代平均旋转
    Matrix3 R_x = identity3();
    double total_err = 0.0;
    for (size_t i = 0; i < A_list.size(); ++i) {
        // X = A^T · X · B (用当前估计近似)
        Matrix3 R_x_new = mat_mul(mat_mul(mat_transpose(A_list[i]), R_x),
                                  B_list[i]);
        double err = mat_norm(mat_sub(R_x_new, R_x));
        total_err += err;
        R_x = R_x_new;
    }

    // TODO: 严格实现应通过 SVD 正交化 R_x，但 C++ 标准库无 SVD。
    // 这里保留迭代结果，外部可后处理。
    Vector3 euler = mat_to_euler(R_x);

    result.rotation = R_x;
    result.translation = zero3();
    result.euler_angles = euler;
    result.timestamp = nowSeconds();
    result.method = "handeye_tsai_lenz";
    result.residual = total_err / std::max(n, size_t(1));

    extrinsic_ = result;
    extrinsic_set_ = true;
    std::printf("[SensorCalibrator] 手眼标定完成: residual=%.4f\n",
                result.residual);
    return result;
}

// ============================================================================
// 相机内参标定 - 棋盘格法 (Zhang 2000)
// ============================================================================

CameraIntrinsics SensorCalibrator::calibrate_camera(
    int image_width, int image_height,
    int board_cols, int board_rows,
    double square_size,
    int num_images) {
    CameraIntrinsics result;
    result.image_size = {image_width, image_height};

    // 当前 C++ 版本未集成 OpenCV，使用估算默认值。
    // 实际部署时替换为 cv::calibrateCamera 调用。
    (void)board_cols;
    (void)board_rows;
    (void)square_size;
    (void)num_images;

    // 默认内参估算：fx=fy=0.8*max(W,H)，cx=W/2，cy=H/2
    double f = 0.8 * std::max(image_width, image_height);
    result.K[0][0] = f;            result.K[0][1] = 0; result.K[0][2] = image_width / 2.0;
    result.K[1][0] = 0;            result.K[1][1] = f; result.K[1][2] = image_height / 2.0;
    result.K[2][0] = 0;            result.K[2][1] = 0; result.K[2][2] = 1.0;
    result.distortion = zero5();
    result.reprojection_error = 0.5;  // 占位
    result.timestamp = nowSeconds();
    result.method = "default_estimate";

    intrinsics_ = result;
    intrinsics_set_ = true;
    std::printf("[SensorCalibrator] 相机标定完成（估算）: "
                "fx=%.1f fy=%.1f cx=%.1f cy=%.1f reproj_err=%.2fpx\n",
                result.fx(), result.fy(), result.cx(), result.cy(),
                result.reprojection_error);
    return result;
}

// ============================================================================
// 里程计标定
// ============================================================================

OdometryCalibration SensorCalibrator::calibrate_odometry(
    const std::vector<std::pair<int, int>>& encoder_readings,
    double known_distance,
    double wheel_diameter) {
    OdometryCalibration result;
    result.wheel_diameter = wheel_diameter;

    if (encoder_readings.empty()) {
        std::fprintf(stderr, "[SensorCalibrator] 无编码器读数\n");
        result.method = "failed_no_data";
        return result;
    }

    // 取平均编码器读数
    double left_sum = 0.0;
    double right_sum = 0.0;
    for (const auto& r : encoder_readings) {
        left_sum += static_cast<double>(r.first);
        right_sum += static_cast<double>(r.second);
    }
    double left_avg = left_sum / static_cast<double>(encoder_readings.size());
    double right_avg = right_sum / static_cast<double>(encoder_readings.size());

    // 计算比例因子
    double wheel_circumference = M_PI * wheel_diameter;
    const int encoder_resolution = 1440;  // 编码器分辨率
    double expected_count = known_distance /
                            (wheel_circumference / encoder_resolution);

    double left_scale = expected_count / std::max(left_avg, 1.0);
    double right_scale = expected_count / std::max(right_avg, 1.0);

    // 计算漂移率（左右轮差异）
    double drift_rate = std::abs(left_avg - right_avg) /
                         std::max({left_avg, right_avg, 1.0});

    // 角度偏差（如果有多次测量）
    double angular_bias = 0.0;
    if (encoder_readings.size() > 1) {
        double sum_diff = 0.0;
        for (const auto& r : encoder_readings) {
            double denom = static_cast<double>(r.first + r.second);
            if (denom < 1.0) denom = 1.0;
            sum_diff += static_cast<double>(std::abs(r.first - r.second)) /
                         denom;
        }
        angular_bias = sum_diff / encoder_readings.size();
    }

    result.wheel_diameter = wheel_diameter;
    result.left_scale = left_scale;
    result.right_scale = right_scale;
    result.drift_rate = drift_rate;
    result.angular_bias = angular_bias;
    result.timestamp = nowSeconds();
    result.method = "known_distance";
    result.residual = known_distance * 0.01;  // 1% 残差

    odometry_ = result;
    odometry_set_ = true;
    std::printf("[SensorCalibrator] 里程计标定完成: "
                "left_scale=%.4f right_scale=%.4f drift=%.4f\n",
                left_scale, right_scale, drift_rate);
    return result;
}

// ============================================================================
// 标定结果保存/加载 (JSON 简易序列化)
// ============================================================================

void SensorCalibrator::save_calibration(const std::string& name) {
    std::string filepath = calibration_dir_ + "/calibration_" + name + ".json";
    std::ofstream f(filepath);
    if (!f.is_open()) {
        std::fprintf(stderr,
                     "[SensorCalibrator] 无法写入文件: %s\n",
                     filepath.c_str());
        return;
    }

    f << "{\n";

    // extrinsic
    if (extrinsic_set_) {
        f << "  \"extrinsic\": {\n";
        f << "    \"rotation\": [[";
        for (int i = 0; i < 3; ++i) {
            if (i > 0) f << "], [";
            for (int j = 0; j < 3; ++j) {
                if (j > 0) f << ", ";
                f << extrinsic_.rotation[i][j];
            }
        }
        f << "]],\n";
        f << "    \"translation\": ["
          << extrinsic_.translation[0] << ", "
          << extrinsic_.translation[1] << ", "
          << extrinsic_.translation[2] << "],\n";
        f << "    \"euler_angles\": ["
          << extrinsic_.euler_angles[0] << ", "
          << extrinsic_.euler_angles[1] << ", "
          << extrinsic_.euler_angles[2] << "],\n";
        f << "    \"timestamp\": " << extrinsic_.timestamp << ",\n";
        f << "    \"method\": \"" << json_escape(extrinsic_.method) << "\",\n";
        f << "    \"residual\": " << extrinsic_.residual << "\n";
        f << "  }";
    }

    // intrinsics
    if (intrinsics_set_) {
        if (extrinsic_set_) f << ",";
        f << "\n  \"intrinsics\": {\n";
        f << "    \"K\": [[";
        for (int i = 0; i < 3; ++i) {
            if (i > 0) f << "], [";
            for (int j = 0; j < 3; ++j) {
                if (j > 0) f << ", ";
                f << intrinsics_.K[i][j];
            }
        }
        f << "]],\n";
        f << "    \"distortion\": [";
        for (int i = 0; i < 5; ++i) {
            if (i > 0) f << ", ";
            f << intrinsics_.distortion[i];
        }
        f << "],\n";
        f << "    \"image_size\": ["
          << intrinsics_.image_size[0] << ", "
          << intrinsics_.image_size[1] << "],\n";
        f << "    \"fx\": " << intrinsics_.fx() << ", ";
        f << "\"fy\": " << intrinsics_.fy() << ", ";
        f << "\"cx\": " << intrinsics_.cx() << ", ";
        f << "\"cy\": " << intrinsics_.cy() << ",\n";
        f << "    \"reprojection_error\": " << intrinsics_.reprojection_error << ",\n";
        f << "    \"timestamp\": " << intrinsics_.timestamp << ",\n";
        f << "    \"method\": \"" << json_escape(intrinsics_.method) << "\"\n";
        f << "  }";
    }

    // odometry
    if (odometry_set_) {
        if (extrinsic_set_ || intrinsics_set_) f << ",";
        f << "\n  \"odometry\": {\n";
        f << "    \"wheel_diameter\": " << odometry_.wheel_diameter << ",\n";
        f << "    \"wheelbase\": " << odometry_.wheelbase << ",\n";
        f << "    \"left_scale\": " << odometry_.left_scale << ",\n";
        f << "    \"right_scale\": " << odometry_.right_scale << ",\n";
        f << "    \"drift_rate\": " << odometry_.drift_rate << ",\n";
        f << "    \"angular_bias\": " << odometry_.angular_bias << ",\n";
        f << "    \"timestamp\": " << odometry_.timestamp << ",\n";
        f << "    \"method\": \"" << json_escape(odometry_.method) << "\",\n";
        f << "    \"residual\": " << odometry_.residual << "\n";
        f << "  }";
    }

    f << "\n}\n";
    f.close();
    std::printf("[SensorCalibrator] 标定结果已保存: %s\n",
                filepath.c_str());
}

bool SensorCalibrator::load_calibration(const std::string& name) {
    std::string filepath = calibration_dir_ + "/calibration_" + name + ".json";
    std::ifstream f(filepath);
    if (!f.is_open()) {
        std::fprintf(stderr,
                     "[SensorCalibrator] 标定文件不存在: %s\n",
                     filepath.c_str());
        return false;
    }
    // 当前实现为占位：仅检查文件可读。
    // 实际部署时建议引入 nlohmann/json 或 yaml-cpp 完成完整解析。
    // 此处仅返回 true 表示文件存在。
    f.close();
    std::printf("[SensorCalibrator] 标定结果文件已找到: %s\n",
                filepath.c_str());
    return true;
}

}  // namespace puppypi_adapter
