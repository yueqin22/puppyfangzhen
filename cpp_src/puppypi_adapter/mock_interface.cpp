// 仿真 Mock 硬件接口实现
//
// 对应 Python: puppypi_adapter/mock_interface.py
// 纯工具类（非 ROS2 节点），不包含 main()
#include "puppypi_adapter/mock_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppypi_adapter {

namespace {

double nowSeconds() {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

double normalRandom() {
    static thread_local std::mt19937 gen{std::random_device{}()};
    static thread_local std::normal_distribution<double> dist{0.0, 1.0};
    return dist(gen);
}

double uniformRandom() {
    static thread_local std::mt19937 gen{std::random_device{}()};
    static thread_local std::uniform_real_distribution<double> dist{0.0, 1.0};
    return dist(gen);
}

}  // namespace

MockHardwareInterface::MockHardwareInterface() {
    // 默认配置已在成员初始化列表中
}

MockHardwareInterface::MockHardwareInterface(const Config& config)
    : HardwareInterface(config) {
    // 解析 sensors 子配置
    auto get_str = [this](const std::string& key,
                          const std::string& def) -> std::string {
        auto it = config_.find(key);
        return (it != config_.end()) ? it->second : def;
    };
    auto get_int = [this](const std::string& key, int def) -> int {
        auto it = config_.find(key);
        return (it != config_.end()) ? std::stoi(it->second) : def;
    };
    auto get_double = [this](const std::string& key, double def) -> double {
        auto it = config_.find(key);
        return (it != config_.end()) ? std::stod(it->second) : def;
    };

    lidar_beams_ = get_int("lidar_beams", 72);
    lidar_range_ = get_double("lidar_range", 8.0);
    lidar_noise_ = get_double("lidar_noise", 0.02);
    imu_noise_ = get_double("imu_noise", 0.01);
    odom_noise_ = get_double("odom_noise", 0.005);

    x_ = get_double("initial_x", 0.0);
    y_ = get_double("initial_y", 0.0);
    yaw_ = get_double("initial_yaw", 0.0);

    battery_percent_ = get_double("battery_initial_percent", 1.0);
    battery_voltage_ = get_double("battery_nominal_voltage", 12.0);
    battery_drain_rate_ = get_double("battery_drain_rate", 0.00002);
}

MockHardwareInterface::~MockHardwareInterface() {
    shutdown();
}

bool MockHardwareInterface::initialize() {
    initialized_ = true;
    running_ = true;
    sim_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { simLoop(); }));
    return true;
}

void MockHardwareInterface::shutdown() {
    running_ = false;
    if (sim_thread_ && sim_thread_->joinable()) {
        sim_thread_->join();
    }
    sim_thread_.reset();
    initialized_ = false;
}

void MockHardwareInterface::set_environment(
    const std::vector<ObstacleBox>& obstacles,
    const std::vector<ObstacleCircle>& dynamic_obstacles) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = obstacles;
    dynamic_obstacles_ = dynamic_obstacles;
}

void MockHardwareInterface::inject_fault(const std::string& fault) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        faults_.push_back(fault);
    }
    // 锁外调用基类方法（避免递归锁）
    if (fault == "emergency_stop") {
        emergency_stop("fault_injection");
    }
}

void MockHardwareInterface::clear_fault(const std::string& fault) {
    std::lock_guard<std::mutex> lock(mutex_);
    faults_.erase(std::remove(faults_.begin(), faults_.end(), fault),
                  faults_.end());
}

std::tuple<double, double, double> MockHardwareInterface::pose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::make_tuple(x_, y_, yaw_);
}

double MockHardwareInterface::nowSeconds() const {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

void MockHardwareInterface::dispatch_motion(double vx, double vy, double wz) {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = vx;
    vy_ = vy;
    wz_ = wz;
    // 电池消耗（移动时消耗更多）
    double speed = std::sqrt(vx * vx + vy * vy + wz * wz * 0.1);
    battery_percent_ = std::max(
        0.0, battery_percent_ - battery_drain_rate_ * (1.0 + speed));
    last_cmd_time_ = nowSeconds();
    cmd_count_ += 1;
}

SensorReadings MockHardwareInterface::get_sensor_readings() {
    // 复制共享状态
    double x, y, yaw, vx, vy, wz;
    std::vector<ObstacleBox> obstacles;
    std::vector<ObstacleCircle> dynamic_obstacles;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        x = x_; y = y_; yaw = yaw_;
        vx = vx_; vy = vy_; wz = wz_;
        obstacles = obstacles_;
        dynamic_obstacles = dynamic_obstacles_;
    }

    SensorReadings readings;

    // LiDAR 扫描模拟（-π ~ π 均匀分布）
    readings.lidar_angles.resize(lidar_beams_);
    readings.lidar_ranges.resize(lidar_beams_);
    for (int i = 0; i < lidar_beams_; ++i) {
        double angle = -M_PI + (2.0 * M_PI * i) / lidar_beams_;
        readings.lidar_angles[i] = angle;
        double world_angle = angle + yaw;
        // 射线投射
        double min_dist = lidar_range_;
        double ox = x, oy = y;
        double dx = std::cos(world_angle), dy = std::sin(world_angle);
        // 静态障碍物
        for (const auto& box : obstacles) {
            double t;
            if (rayAabb(ox, oy, dx, dy, box, t) && t < min_dist) {
                min_dist = t;
            }
        }
        // 动态障碍物
        for (const auto& cir : dynamic_obstacles) {
            double t;
            if (rayCircle(ox, oy, dx, dy, cir, t) && t < min_dist) {
                min_dist = t;
            }
        }
        // 加入高斯噪声
        if (min_dist < lidar_range_) {
            min_dist += normalRandom() * lidar_noise_;
            min_dist = std::min(min_dist, lidar_range_);
        }
        readings.lidar_ranges[i] = std::max(0.0, min_dist);
    }

    // IMU 模拟（带噪声）
    readings.imu_ax = vx * std::cos(yaw);
    readings.imu_ay = vx * std::sin(yaw);
    readings.imu_az = 9.81;
    readings.imu_gx = normalRandom() * imu_noise_;
    readings.imu_gy = normalRandom() * imu_noise_;
    readings.imu_gz = wz + normalRandom() * imu_noise_;

    // 里程计（带噪声）
    readings.odom_x = x + normalRandom() * odom_noise_;
    readings.odom_y = y + normalRandom() * odom_noise_;
    readings.odom_yaw = yaw + normalRandom() * odom_noise_ * 2.0;
    readings.odom_vx = vx;
    readings.odom_vy = vy;
    readings.odom_wz = wz;
    readings.timestamp = nowSeconds();

    return readings;
}

BatteryState MockHardwareInterface::get_battery() {
    double percent;
    double vx, wz;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        percent = battery_percent_;
        vx = vx_;
        wz = wz_;
    }
    // 模拟电压（电量越低电压越低）
    battery_.voltage = battery_voltage_ * (0.85 + 0.15 * percent);
    // 模拟电流（移动时电流大）
    battery_.current = std::abs(vx) * 2.0 + std::abs(wz) * 0.5 + 0.3;
    battery_.percent = percent;
    battery_.charging = false;
    battery_.temperature = 25.0 + battery_.current * 0.5;
    battery_.update_thresholds(low_battery_threshold_,
                               critical_battery_threshold_);
    return battery_;
}

RobotHealthState MockHardwareInterface::get_health() {
    std::vector<std::string> faults;
    bool estop;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        faults = faults_;
        estop = emergency_stopped_;
    }
    RobotHealthState h;
    h.ok = true;
    h.level = "OK";
    if (estop) {
        h.level = "FATAL";
        h.ok = false;
        faults.push_back("EMERGENCY_STOP");
    } else if (battery_.critical_battery) {
        h.level = "ERROR";
        h.ok = false;
        faults.push_back("CRITICAL_BATTERY");
    } else if (battery_.low_battery) {
        h.level = "WARN";
        faults.push_back("LOW_BATTERY");
    }
    h.active_faults = std::move(faults);
    h.cpu_temp = 45.0 + (uniformRandom() * 4.0 - 2.0);
    h.imu_ready = true;
    h.lidar_ready = true;
    h.camera_ready = true;
    h.motion_ready = motors_enabled_;
    return h;
}

void MockHardwareInterface::simLoop() {
    double last_time = nowSeconds();
    int consecutive_errors = 0;
    const int kMaxConsecutiveErrors = 10;

    while (running_) {
        try {
            double now = nowSeconds();
            double dt = now - last_time;
            last_time = now;
            // 限制 dt 防止大跳变
            dt = std::min(dt, sim_dt_ * 5.0);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                // 差速驱动运动学更新
                if (motors_enabled_ && !emergency_stopped_) {
                    x_ += vx_ * std::cos(yaw_) * dt;
                    y_ += vx_ * std::sin(yaw_) * dt;
                    yaw_ += wz_ * dt;
                    // 归一化 yaw 到 [-π, π]
                    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
                    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
                }
                sim_time_ += dt;
            }

            // 更新心跳
            heartbeat_ = nowSeconds();
            consecutive_errors = 0;

            // 精确睡眠以保持仿真频率
            double sleep_time = sim_dt_ - (nowSeconds() - now);
            if (sleep_time > 0) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(sleep_time));
            }
        } catch (...) {
            consecutive_errors += 1;
            sim_error_count_ += 1;
            heartbeat_ = nowSeconds();
            if (consecutive_errors >= kMaxConsecutiveErrors) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (std::find(faults_.begin(), faults_.end(),
                              "SIM_THREAD_ERROR") == faults_.end()) {
                    faults_.push_back("SIM_THREAD_ERROR");
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

bool MockHardwareInterface::rayAabb(double ox, double oy, double dx, double dy,
                                    const ObstacleBox& box, double& t) {
    // 射线与 AABB 求交 (Slab 法)
    double inv_dx = (std::abs(dx) > 1e-9) ? (1.0 / dx) : 1e18;
    double inv_dy = (std::abs(dy) > 1e-9) ? (1.0 / dy) : 1e18;

    double t1 = (box.xmin - ox) * inv_dx;
    double t2 = (box.xmax - ox) * inv_dx;
    double t3 = (box.ymin - oy) * inv_dy;
    double t4 = (box.ymax - oy) * inv_dy;

    double tmin = std::max(std::min(t1, t2), std::min(t3, t4));
    double tmax = std::min(std::max(t1, t2), std::max(t3, t4));

    if (tmax < 0 || tmin > tmax) {
        return false;
    }
    if (tmin < 0) {
        if (tmax > 0) {
            t = tmax;
            return true;
        }
        return false;
    }
    t = tmin;
    return true;
}

bool MockHardwareInterface::rayCircle(double ox, double oy, double dx, double dy,
                                       const ObstacleCircle& circle, double& t) {
    // 射线与圆形求交
    double ocx = ox - circle.x;
    double ocy = oy - circle.y;
    double b = ocx * dx + ocy * dy;
    double c = ocx * ocx + ocy * ocy - circle.r * circle.r;
    double disc = b * b - c;
    if (disc < 0) {
        return false;
    }
    double sq = std::sqrt(disc);
    double t1 = -b - sq;
    if (t1 > 0.1) {
        t = t1;
        return true;
    }
    double t2 = -b + sq;
    if (t2 > 0.1) {
        t = t2;
        return true;
    }
    return false;
}

}  // namespace puppypi_adapter
