// 故障注入测试框架 实现
//
// 对应 Python: puppypi_adapter/fault_injection.py
// 纯工具类（非 ROS2 节点），不包含 main()
#include "puppypi_adapter/fault_injection.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <sstream>
#include <utility>

namespace puppypi_adapter {

namespace {

/// 获取当前时间（秒，单调时钟）
double nowSeconds() {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

/// 标准正态分布随机数（线程本地引擎）
double normalRandom() {
    static thread_local std::mt19937 gen{std::random_device{}()};
    static thread_local std::normal_distribution<double> dist{0.0, 1.0};
    return dist(gen);
}

/// [0,1) 均匀随机数
double uniformRandom() {
    static thread_local std::mt19937 gen{std::random_device{}()};
    static thread_local std::uniform_real_distribution<double> dist{0.0, 1.0};
    return dist(gen);
}

}  // namespace

FaultInjector::FaultInjector(HardwareInterface* hw_interface)
    : hw_(hw_interface) {}

FaultInjector::~FaultInjector() {
    stopCpuStress();
}

const char* FaultInjector::faultTypeName(FaultType t) {
    switch (t) {
        case FaultType::LIDAR_TIMEOUT:      return "LIDAR_TIMEOUT";
        case FaultType::LIDAR_NOISE:        return "LIDAR_NOISE";
        case FaultType::LIDAR_DATA_LOSS:    return "LIDAR_DATA_LOSS";
        case FaultType::IMU_DRIFT:          return "IMU_DRIFT";
        case FaultType::IMU_SATURATION:     return "IMU_SATURATION";
        case FaultType::CAMERA_BLUR:        return "CAMERA_BLUR";
        case FaultType::ENCODER_JUMP:       return "ENCODER_JUMP";
        case FaultType::COMM_LATENCY:       return "COMM_LATENCY";
        case FaultType::COMM_PACKET_LOSS:   return "COMM_PACKET_LOSS";
        case FaultType::NODE_CRASH:         return "NODE_CRASH";
        case FaultType::CPU_OVERLOAD:       return "CPU_OVERLOAD";
        case FaultType::MEMORY_LEAK:        return "MEMORY_LEAK";
        case FaultType::THREAD_STARVATION:  return "THREAD_STARVATION";
        case FaultType::LIGHT_CHANGE:       return "LIGHT_CHANGE";
        case FaultType::GLASS_SURFACE:      return "GLASS_SURFACE";
        case FaultType::NARROW_CORRIDOR:    return "NARROW_CORRIDOR";
    }
    return "UNKNOWN";
}

const char* FaultInjector::severityName(FaultSeverity s) {
    switch (s) {
        case FaultSeverity::MINOR:    return "MINOR";
        case FaultSeverity::MODERATE: return "MODERATE";
        case FaultSeverity::SEVERE:    return "SEVERE";
    }
    return "UNKNOWN";
}

double FaultInjector::resolveParam(
    const std::map<std::string, double>& params,
    const std::string& key,
    FaultSeverity sev,
    double minor_v, double moderate_v, double severe_v) const {
    auto it = params.find(key);
    if (it != params.end()) {
        return it->second;
    }
    switch (sev) {
        case FaultSeverity::MINOR:    return minor_v;
        case FaultSeverity::MODERATE: return moderate_v;
        case FaultSeverity::SEVERE:    return severe_v;
    }
    return moderate_v;
}

void FaultInjector::inject(FaultType fault_type,
                           FaultSeverity severity,
                           double duration,
                           double start_delay,
                           const std::map<std::string, double>& params) {
    FaultInjection fault;
    fault.fault_type = fault_type;
    fault.severity = severity;
    fault.duration = duration;
    fault.start_delay = start_delay;
    fault.parameters = params;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        faults_[static_cast<int>(fault_type)] = fault;
        injection_count_ += 1;

        FaultHistoryEntry entry;
        entry.time = nowSeconds();
        entry.action = "inject";
        entry.fault_type = faultTypeName(fault_type);
        entry.severity = severityName(severity);
        entry.duration = duration;
        fault_history_.push_back(std::move(entry));
    }

    // 延迟启动
    if (start_delay > 0.0) {
        // 简化：在调用线程中阻塞 sleep（避免引入额外线程管理）
        std::this_thread::sleep_for(
            std::chrono::duration<double>(start_delay));
    }
    // 激活故障
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = faults_.find(static_cast<int>(fault_type));
        if (it != faults_.end()) {
            activateFault(it->second);
        }
    }

    // 定时清除（如果有持续时间）
    if (duration > 0.0) {
        // 简化：启动一个守护线程在 duration 秒后清除
        std::thread([this, fault_type, duration]() {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(duration));
            this->clear(fault_type);
        }).detach();
    }

    std::printf("[FaultInjector] 注入: %s (severity=%s, duration=%.2fs)\n",
                faultTypeName(fault_type), severityName(severity), duration);
}

void FaultInjector::clear(FaultType fault_type) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = faults_.find(static_cast<int>(fault_type));
        if (it == faults_.end()) {
            return;
        }
        faults_.erase(it);

        FaultHistoryEntry entry;
        entry.time = nowSeconds();
        entry.action = "clear";
        entry.fault_type = faultTypeName(fault_type);
        fault_history_.push_back(std::move(entry));
    }

    // 重置故障效果
    switch (fault_type) {
        case FaultType::LIDAR_NOISE:
            lidar_noise_level_ = 0.0;
            break;
        case FaultType::IMU_DRIFT:
            imu_drift_rate_ = 0.0;
            imu_bias_[0] = imu_bias_[1] = imu_bias_[2] = 0.0;
            break;
        case FaultType::ENCODER_JUMP:
            encoder_offset_ = 0.0;
            break;
        case FaultType::COMM_LATENCY:
            comm_latency_ = 0.0;
            break;
        case FaultType::COMM_PACKET_LOSS:
            packet_loss_rate_ = 0.0;
            break;
        case FaultType::CPU_OVERLOAD:
            stopCpuStress();
            break;
        default:
            break;
    }

    std::printf("[FaultInjector] 清除: %s\n", faultTypeName(fault_type));
}

void FaultInjector::clearAll() {
    std::vector<FaultType> to_clear;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        to_clear.reserve(faults_.size());
        for (const auto& kv : faults_) {
            to_clear.push_back(static_cast<FaultType>(kv.first));
        }
    }
    for (auto ft : to_clear) {
        clear(ft);
    }
}

void FaultInjector::activateFault(FaultInjection& fault) {
    const auto ft = fault.fault_type;
    const auto sev = fault.severity;
    const auto& params = fault.parameters;

    switch (ft) {
        case FaultType::LIDAR_NOISE:
            // LiDAR 噪声: 增加测距噪声
            lidar_noise_level_ = resolveParam(params, "noise_std", sev,
                                              0.05, 0.2, 0.5);
            break;
        case FaultType::LIDAR_TIMEOUT:
        case FaultType::LIDAR_DATA_LOSS:
            // 在 getSensorReadings 中处理
            break;
        case FaultType::IMU_DRIFT:
            // IMU 漂移: 累积零偏
            imu_drift_rate_ = resolveParam(params, "drift_rate", sev,
                                            0.01, 0.05, 0.2);
            break;
        case FaultType::IMU_SATURATION:
            // IMU 饱和: 固定值
            imu_bias_[0] = 9.81;
            imu_bias_[1] = 0.0;
            imu_bias_[2] = 0.0;
            break;
        case FaultType::ENCODER_JUMP:
            // 编码器跳变
            encoder_offset_ = resolveParam(params, "offset", sev,
                                            0.1, 0.5, 2.0);
            break;
        case FaultType::COMM_LATENCY:
            // 通信延迟
            comm_latency_ = resolveParam(params, "latency", sev,
                                          0.05, 0.2, 0.5);
            break;
        case FaultType::COMM_PACKET_LOSS:
            // 消息丢失
            packet_loss_rate_ = resolveParam(params, "loss_rate", sev,
                                             0.1, 0.3, 0.6);
            break;
        case FaultType::CPU_OVERLOAD: {
            // CPU 过载: 启动计算密集线程
            int load_percent = 80;
            auto it = params.find("load_percent");
            if (it != params.end()) {
                load_percent = static_cast<int>(it->second);
            }
            startCpuStress(load_percent);
            break;
        }
        default:
            // 其他故障类型暂未实现具体效果
            break;
    }

    fault.active = true;
    fault.start_time = nowSeconds();
}

bool FaultInjector::getSensorReadings(SensorReadings& out) {
    // 通信延迟模拟
    if (comm_latency_ > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(comm_latency_));
    }

    // 消息丢失模拟
    if (packet_loss_rate_ > 0.0) {
        if (uniformRandom() < packet_loss_rate_) {
            // 返回上一次的数据（模拟丢失）
            if (has_last_readings_) {
                out = last_readings_;
                return true;
            }
            return false;
        }
    }

    // 读取真实数据
    if (!hw_) {
        return false;
    }
    out = hw_->get_sensor_readings();

    // 应用故障效果
    std::vector<FaultInjection> active_faults;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& kv : faults_) {
            if (kv.second.active) {
                active_faults.push_back(kv.second);
            }
        }
    }

    for (const auto& fault : active_faults) {
        switch (fault.fault_type) {
            case FaultType::LIDAR_NOISE: {
                // 加入高斯噪声并裁剪到 [0, 100]
                for (auto& r : out.lidar_ranges) {
                    r = std::clamp(r + normalRandom() * lidar_noise_level_,
                                   0.0, 100.0);
                }
                break;
            }
            case FaultType::LIDAR_TIMEOUT: {
                // 超时: 返回全零扫描
                std::fill(out.lidar_ranges.begin(),
                          out.lidar_ranges.end(), 0.0);
                break;
            }
            case FaultType::LIDAR_DATA_LOSS: {
                // 部分丢失: 随机置零
                for (auto& r : out.lidar_ranges) {
                    if (uniformRandom() < 0.3) {
                        r = 0.0;
                    }
                }
                break;
            }
            case FaultType::IMU_DRIFT: {
                // 累积漂移
                double dt = nowSeconds() - fault.start_time;
                double drift = imu_drift_rate_ * dt;
                out.imu_gx += drift;
                out.imu_gy += drift * 0.5;
                out.imu_gz += drift * 0.3;
                break;
            }
            case FaultType::IMU_SATURATION: {
                // 饱和: 固定值
                out.imu_ax = imu_bias_[0];
                out.imu_ay = imu_bias_[1];
                out.imu_az = imu_bias_[2];
                break;
            }
            case FaultType::ENCODER_JUMP: {
                // 编码器跳变
                out.odom_x += encoder_offset_;
                out.odom_y += encoder_offset_ * 0.5;
                break;
            }
            default:
                break;
        }
    }

    last_readings_ = out;
    has_last_readings_ = true;
    return true;
}

void FaultInjector::sendVelocity(double vx, double vy, double wz) {
    // 应用通信故障
    if (comm_latency_ > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(comm_latency_));
    }
    if (packet_loss_rate_ > 0.0 && uniformRandom() < packet_loss_rate_) {
        return;  // 命令丢失
    }
    if (hw_) {
        hw_->send_velocity(vx, vy, wz);
    }
}

void FaultInjector::recordDetection(FaultType /*fault_type*/) {
    std::lock_guard<std::mutex> lock(mutex_);
    detection_count_ += 1;
}

void FaultInjector::recordRecovery(FaultType /*fault_type*/) {
    std::lock_guard<std::mutex> lock(mutex_);
    recovery_count_ += 1;
}

std::vector<FaultHistoryEntry> FaultInjector::getFaultHistory() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_history_;
}

std::vector<FaultType> FaultInjector::getActiveFaults() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<FaultType> result;
    result.reserve(faults_.size());
    for (const auto& kv : faults_) {
        if (kv.second.active) {
            result.push_back(static_cast<FaultType>(kv.first));
        }
    }
    return result;
}

FaultInjector::Stats FaultInjector::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats s;
    s.injection_count = injection_count_;
    s.detection_count = detection_count_;
    s.recovery_count = recovery_count_;
    s.detection_rate = static_cast<double>(detection_count_) /
                       std::max(injection_count_, 1L);
    s.recovery_rate = static_cast<double>(recovery_count_) /
                      std::max(injection_count_, 1L);
    for (const auto& kv : faults_) {
        if (kv.second.active) {
            s.active_faults.push_back(faultTypeName(
                static_cast<FaultType>(kv.first)));
        }
    }
    return s;
}

std::string FaultInjector::evaluateRobustness() const {
    // 与 Python 版一致：根据检测率与恢复率分级
    long inj = injection_count_;
    long det = detection_count_;
    long rec = recovery_count_;
    if (inj == 0) {
        return "无故障注入";
    }
    double det_rate = static_cast<double>(det) / inj;
    double rec_rate = static_cast<double>(rec) / inj;
    if (det_rate >= 0.9 && rec_rate >= 0.8) {
        return "优秀：故障检测和恢复能力强";
    }
    if (det_rate >= 0.7 && rec_rate >= 0.6) {
        return "良好：基本能应对常见故障";
    }
    if (det_rate >= 0.5) {
        return "一般：部分故障无法检测";
    }
    return "不足：故障检测能力弱";
}

FaultInjector::Report FaultInjector::generateReport() const {
    Report r;
    r.summary = getStats();
    r.fault_history = getFaultHistory();
    r.conclusion = evaluateRobustness();
    return r;
}

void FaultInjector::startCpuStress(int load_percent) {
    stopCpuStress();
    cpu_stress_running_ = true;
    cpu_stress_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this, load_percent]() {
            cpuStressLoop(load_percent);
        }));
}

void FaultInjector::stopCpuStress() {
    cpu_stress_running_ = false;
    if (cpu_stress_thread_ && cpu_stress_thread_->joinable()) {
        cpu_stress_thread_->join();
    }
    cpu_stress_thread_.reset();
}

void FaultInjector::cpuStressLoop(int load_percent) {
    // 简化的 CPU 压力循环：busy_time 内做 sqrt 计算，剩余时间休眠
    double busy_time = load_percent / 1000.0;       // ms
    double idle_time = (100 - load_percent) / 1000.0;
    while (cpu_stress_running_) {
        auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - t0).count() < busy_time) {
            // 空转消耗 CPU
            volatile double v = std::sqrt(12345.6789);
            (void)v;
        }
        std::this_thread::sleep_for(
            std::chrono::duration<double>(idle_time));
    }
}

}  // namespace puppypi_adapter
