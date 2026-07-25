// PuppyPi 实体机硬件驱动实现
//
// 对应 Python: puppypi_adapter/puppypi_driver.py
// 纯工具类（非 ROS2 节点），不包含 main()
//
// 注意：当前 C++ 版本未集成实际 SDK（与 Python 版一致，使用模拟模式回退）。
//       硬件依赖（rplidar, smbus2, RPi.GPIO）通过条件编译宏在 Linux 平台上启用。
#include "puppypi_adapter/puppypi_driver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#ifdef __linux__
#include <unistd.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppypi_adapter {

namespace {

double nowSeconds() {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

}  // namespace

PuppyPiHardwareInterface::PuppyPiHardwareInterface() = default;

PuppyPiHardwareInterface::PuppyPiHardwareInterface(const Config& config)
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

    lidar_port_ = get_str("lidar_port", "/dev/ttyUSB0");
    lidar_baud_ = get_int("lidar_baud", 115200);
    lidar_beams_ = get_int("lidar_beams", 720);
    lidar_range_ = get_double("lidar_range", 12.0);
    imu_i2c_addr_ = get_int("imu_i2c_addr", 0x68);
    imu_rate_ = get_int("imu_rate", 100);

    battery_adc_channel_ = get_int("adc_channel", 0);
    battery_adc_vref_ = get_double("adc_vref", 3.3);
    battery_cells_ = get_int("cells", 3);
    battery_full_voltage_ = get_double("full_voltage", 12.6);
    battery_empty_voltage_ = get_double("empty_voltage", 9.9);
    battery_nominal_ = get_double("nominal_voltage", 11.1);

    estop_pin_ = get_int("estop_button_pin", 18);
    led_pin_ = get_int("led_status_pin", 24);

    // 初始化 IMU 缓存为默认值
    cached_imu_accel_[0] = 0.0;
    cached_imu_accel_[1] = 0.0;
    cached_imu_accel_[2] = 9.81;
}

PuppyPiHardwareInterface::~PuppyPiHardwareInterface() {
    shutdown();
}

bool PuppyPiHardwareInterface::initialize() {
    // 顺序: SDK → LiDAR → IMU → ADC → GPIO → 后台线程
    initSdk();
    initLidar();
    initImu();
    initAdc();
    initGpio();

    running_ = true;
    sensor_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { sensorLoop(); }));
    motor_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { motorLoop(); }));
    monitor_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { monitorLoop(); }));
    estop_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { estopPollLoop(); }));

    initialized_ = true;
    std::printf("[PuppyPiDriver] 硬件初始化完成\n");
    return true;
}

void PuppyPiHardwareInterface::shutdown() {
    running_ = false;
    // 停止电机
    dispatch_motion(0.0, 0.0, 0.0);

    // 等待线程结束
    auto join_thread = [](std::unique_ptr<std::thread>& t) {
        if (t && t->joinable()) {
            t->join();
        }
        t.reset();
    };
    join_thread(sensor_thread_);
    join_thread(motor_thread_);
    join_thread(monitor_thread_);
    join_thread(estop_thread_);

    // 关闭设备（实际句柄为空时不执行任何操作）
    sdk_handle_ = nullptr;
    lidar_handle_ = nullptr;
    imu_handle_ = nullptr;
    adc_handle_ = nullptr;
    gpio_handle_ = nullptr;

    initialized_ = false;
    std::printf("[PuppyPiDriver] 硬件已关闭\n");
}

void PuppyPiHardwareInterface::initSdk() {
    // TODO: 集成 puppypi_sdk
    // 当前未集成，使用模拟模式
    sdk_handle_ = nullptr;
    std::printf("[PuppyPiDriver] puppypi_sdk 未安装，使用模拟模式\n");
}

void PuppyPiHardwareInterface::initLidar() {
    // TODO: 集成 rplidar
    lidar_handle_ = nullptr;
    std::printf("[PuppyPiDriver] rplidar 库未安装，LiDAR 使用模拟模式\n");
}

void PuppyPiHardwareInterface::initImu() {
    // TODO: 集成 smbus2 + MPU6050
    imu_handle_ = nullptr;
    std::printf("[PuppyPiDriver] smbus2 未安装，IMU 使用模拟模式\n");
}

void PuppyPiHardwareInterface::initAdc() {
    // TODO: 集成 Adafruit_ADS1x15
    adc_handle_ = nullptr;
    std::printf("[PuppyPiDriver] Adafruit_ADS1x15 未安装，电池使用模拟模式\n");
}

void PuppyPiHardwareInterface::initGpio() {
    // TODO: 集成 RPi.GPIO
    gpio_handle_ = nullptr;
    std::printf("[PuppyPiDriver] RPi.GPIO 未安装 (非树莓派环境)\n");
}

void PuppyPiHardwareInterface::dispatch_motion(double vx, double vy, double wz) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_velocity_[0] = vx;
        target_velocity_[1] = vy;
        target_velocity_[2] = wz;
        last_cmd_time_ = nowSeconds();
        cmd_count_ += 1;
    }
    // TODO: 实际 SDK 调用 sdk_handle_->set_velocity(vx, vy, wz)
}

SensorReadings PuppyPiHardwareInterface::get_sensor_readings() {
    SensorReadings readings;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // LiDAR 数据
        if (has_cached_lidar_) {
            readings.lidar_angles = cached_lidar_angles_;
            readings.lidar_ranges = cached_lidar_ranges_;
        } else {
            // 无 LiDAR 时返回空扫描（满量程）
            readings.lidar_angles.resize(lidar_beams_);
            readings.lidar_ranges.resize(lidar_beams_);
            for (int i = 0; i < lidar_beams_; ++i) {
                double angle = -M_PI + (2.0 * M_PI * i) / lidar_beams_;
                readings.lidar_angles[i] = angle;
                readings.lidar_ranges[i] = lidar_range_;
            }
        }
        // IMU 数据
        readings.imu_ax = cached_imu_accel_[0];
        readings.imu_ay = cached_imu_accel_[1];
        readings.imu_az = cached_imu_accel_[2];
        readings.imu_gx = cached_imu_gyro_[0];
        readings.imu_gy = cached_imu_gyro_[1];
        readings.imu_gz = cached_imu_gyro_[2];
        // 里程计
        readings.odom_x = cached_odom_x_;
        readings.odom_y = cached_odom_y_;
        readings.odom_yaw = cached_odom_yaw_;
        readings.odom_vx = actual_velocity_[0];
        readings.odom_vy = actual_velocity_[1];
        readings.odom_wz = actual_velocity_[2];
    }
    readings.timestamp = nowSeconds();
    return readings;
}

BatteryState PuppyPiHardwareInterface::get_battery() {
    std::lock_guard<std::mutex> lock(mutex_);
    return battery_;
}

RobotHealthState PuppyPiHardwareInterface::get_health() {
    std::vector<std::string> faults;
    std::string level = "OK";
    bool ok = true;

    bool estop = emergency_stopped_ || hw_estop_triggered_;
    bool crit = battery_.critical_battery;
    bool low = battery_.low_battery;
    bool motors_en = motors_enabled_;
    bool has_imu = (imu_handle_ != nullptr);
    bool has_lidar = (lidar_handle_ != nullptr);

    if (estop) {
        level = "FATAL";
        ok = false;
        faults.emplace_back("EMERGENCY_STOP");
    }
    if (crit) {
        level = "ERROR";
        ok = false;
        faults.emplace_back("CRITICAL_BATTERY");
    } else if (low) {
        if (level == "OK") {
            level = "WARN";
        }
        faults.emplace_back("LOW_BATTERY");
    }
    if (cpu_temp_ > 80.0) {
        if (level == "OK") {
            level = "WARN";
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "CPU_HIGH_TEMP:%.0fC", cpu_temp_);
        faults.emplace_back(buf);
    }

    RobotHealthState h;
    h.ok = ok;
    h.level = std::move(level);
    h.active_faults = std::move(faults);
    h.cpu_temp = cpu_temp_;
    h.imu_ready = has_imu;
    h.lidar_ready = has_lidar;
    h.camera_ready = true;
    h.motion_ready = motors_en;
    return h;
}

void PuppyPiHardwareInterface::sensorLoop() {
    // 30Hz 传感器读取
    const double rate = 30.0;
    const double dt = 1.0 / rate;
    while (running_) {
        try {
            // LiDAR 读取
            if (lidar_handle_) {
                if (readLidarScan()) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    has_cached_lidar_ = true;
                }
            }
            // IMU 读取
            if (imu_handle_) {
                readImu();
            }
        } catch (...) {
            // 忽略单次读取错误
        }
        std::this_thread::sleep_for(
            std::chrono::duration<double>(dt));
    }
}

void PuppyPiHardwareInterface::motorLoop() {
    // 30Hz 运动控制环（≥20Hz 要求）
    const double rate = 30.0;
    const double dt = 1.0 / rate;
    while (running_) {
        try {
            double target[3];
            {
                std::lock_guard<std::mutex> lock(mutex_);
                target[0] = target_velocity_[0];
                target[1] = target_velocity_[1];
                target[2] = target_velocity_[2];
            }

            // 加速度限制 (平滑过渡)
            double avx = actual_velocity_[0];
            double avy = actual_velocity_[1];
            double awz = actual_velocity_[2];
            avx = limit_accel(target[0], avx, dt, accel_limit_);
            avy = limit_accel(target[1], avy, dt, accel_limit_);
            awz = limit_accel(target[2], awz, dt, yaw_rate_limit_);

            // 里程计积分
            cached_odom_x_ += avx * std::cos(cached_odom_yaw_) * dt;
            cached_odom_y_ += avx * std::sin(cached_odom_yaw_) * dt;
            cached_odom_yaw_ += awz * dt;
            // 归一化 yaw 到 [-π, π]
            while (cached_odom_yaw_ > M_PI) cached_odom_yaw_ -= 2.0 * M_PI;
            while (cached_odom_yaw_ < -M_PI) cached_odom_yaw_ += 2.0 * M_PI;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                actual_velocity_[0] = avx;
                actual_velocity_[1] = avy;
                actual_velocity_[2] = awz;
            }

            // 命令超时检测
            if (command_timeout() && motors_enabled_) {
                dispatch_motion(0.0, 0.0, 0.0);
            }
        } catch (...) {
            // 忽略单次控制环错误
        }
        std::this_thread::sleep_for(
            std::chrono::duration<double>(dt));
    }
}

void PuppyPiHardwareInterface::monitorLoop() {
    // 1Hz 监控
    while (running_) {
        try {
            readBattery();
            readCpuTemp();
        } catch (...) {
            // 忽略监控错误
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void PuppyPiHardwareInterface::estopPollLoop() {
    // 100Hz 急停按钮轮询
    // 仅在 GPIO 可用时运行；当前未集成 GPIO，直接返回
    if (!gpio_handle_) {
        return;
    }
    while (running_) {
        try {
            // TODO: 实际 GPIO 读取
            // if (gpio_input(estop_pin_) == 0) { hw_estop_triggered_ = true;
            //   emergency_stop("hardware_button"); }
        } catch (...) {
            // 忽略急停检测错误
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool PuppyPiHardwareInterface::readLidarScan() {
    // TODO: 集成 rplidar.iter_scans()
    // 当前未集成，返回失败
    return false;
}

bool PuppyPiHardwareInterface::readImu() {
    // TODO: 集成 smbus2 + MPU6050
    // 当前未集成，返回失败
    return false;
}

void PuppyPiHardwareInterface::readBattery() {
    if (!adc_handle_) {
        // 模拟模式：缓慢消耗
        std::lock_guard<std::mutex> lock(mutex_);
        battery_.percent = std::max(0.0, battery_.percent - 0.0001);
        battery_.update_thresholds(low_battery_threshold_,
                                    critical_battery_threshold_);
        return;
    }
    // TODO: 实际 ADC 读取
    // double raw = adc_handle_->read_adc(battery_adc_channel_, gain=1);
    // double adc_voltage = raw * battery_adc_vref_ / 32767.0;
    // double battery_voltage = adc_voltage * 4.0;  // 1:3 分压
    // double percent = (battery_voltage - battery_empty_voltage_) /
    //                  (battery_full_voltage_ - battery_empty_voltage_);
}

void PuppyPiHardwareInterface::readCpuTemp() {
    // 树莓派: /sys/class/thermal/thermal_zone0/temp (单位: 毫摄氏度)
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (f.is_open()) {
        std::string line;
        std::getline(f, line);
        if (!line.empty()) {
            try {
                cpu_temp_ = std::stod(line) / 1000.0;
                return;
            } catch (...) {
                // 解析失败，使用默认值
            }
        }
    }
    cpu_temp_ = 45.0;  // 默认值
}

int16_t PuppyPiHardwareInterface::twosComplement(uint16_t val) {
    // 16 位二补码转换
    if (val >= 0x8000) {
        return -static_cast<int16_t>((65535 - val) + 1);
    }
    return static_cast<int16_t>(val);
}

double PuppyPiHardwareInterface::nowSeconds() {
    return ::puppypi_adapter::nowSeconds();
}

}  // namespace puppypi_adapter
