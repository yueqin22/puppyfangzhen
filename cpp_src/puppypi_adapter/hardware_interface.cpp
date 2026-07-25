// 硬件抽象接口基类 实现
//
// 对应 Python: puppypi_adapter/hardware_interface.py
// 纯工具类（非 ROS2 节点），不包含 main()
//
// 注意：此文件实现 hardware_interface.h 中声明的方法。
// 由于 mock_interface.h / puppypi_driver.h 都继承自 HardwareInterface，
// 这些方法的实现是构建系统的必要依赖。
#include "puppypi_adapter/hardware_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>

// 引入具体子类以支持工厂方法 create()
#include "puppypi_adapter/mock_interface.h"
#include "puppypi_adapter/puppypi_driver.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppypi_adapter {

namespace {

/// 当前秒数（与 Python time.time() 语义一致）
double nowSeconds() {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

/// 将字符串转为小写（用于 backend 配置比较）
std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) {
                       return static_cast<char>(std::tolower(c));
                   });
    return out;
}

}  // namespace

// ============================================================================
// HardwareInterface 实现
// ============================================================================

HardwareInterface::HardwareInterface() {
    // 默认配置已在成员声明中给出
    last_cmd_time_ = nowSeconds();
}

HardwareInterface::HardwareInterface(const Config& config)
    : config_(config) {
    // 解析 motion 子配置（C++ 中采用扁平 key: motion_max_linear_x 等）
    auto get_double = [this](const std::string& key, double def) -> double {
        auto it = config_.find(key);
        return (it != config_.end()) ? std::stod(it->second) : def;
    };

    max_linear_x_ = get_double("motion_max_linear_x", 0.3);
    max_linear_y_ = get_double("motion_max_linear_y", 0.0);
    max_angular_z_ = get_double("motion_max_angular_z", 1.2);
    accel_limit_ = get_double("motion_accel_limit", 2.0);
    yaw_rate_limit_ = get_double("motion_yaw_rate_limit", 4.0);
    cmd_timeout_ = get_double("motion_cmd_timeout", 1.0);

    // 解析 battery 子配置
    low_battery_threshold_ = get_double("battery_low_threshold", 0.20);
    critical_battery_threshold_ = get_double("battery_critical_threshold", 0.10);
    auto it_ar = config_.find("battery_auto_recharge");
    if (it_ar != config_.end()) {
        auto_recharge_enabled_ = (it_ar->second == "true" || it_ar->second == "1");
    }

    // 配置参数校验（启动时拒绝非法参数）
    validate_config();

    // 状态
    last_cmd_time_ = nowSeconds();
}

void HardwareInterface::validate_config() const {
    // 与 Python 版 _validate_config 一致：拒绝非法参数
    std::vector<std::string> errors;
    if (max_linear_x_ < 0) {
        errors.push_back("max_linear_x 不能为负: " +
                          std::to_string(max_linear_x_));
    }
    if (max_linear_y_ < 0) {
        errors.push_back("max_linear_y 不能为负: " +
                          std::to_string(max_linear_y_));
    }
    if (max_angular_z_ < 0) {
        errors.push_back("max_angular_z 不能为负: " +
                          std::to_string(max_angular_z_));
    }
    if (accel_limit_ <= 0) {
        errors.push_back("accel_limit 必须为正: " +
                          std::to_string(accel_limit_));
    }
    if (yaw_rate_limit_ <= 0) {
        errors.push_back("yaw_rate_limit 必须为正: " +
                          std::to_string(yaw_rate_limit_));
    }
    if (cmd_timeout_ <= 0) {
        errors.push_back("cmd_timeout 必须为正: " +
                          std::to_string(cmd_timeout_));
    }
    if (cmd_timeout_ > 10.0) {
        errors.push_back("cmd_timeout 过大 (>10s): " +
                          std::to_string(cmd_timeout_));
    }
    if (low_battery_threshold_ < 0 || low_battery_threshold_ > 1) {
        errors.push_back("low_battery_threshold 超出 [0,1]: " +
                          std::to_string(low_battery_threshold_));
    }
    if (critical_battery_threshold_ < 0 || critical_battery_threshold_ > 1) {
        errors.push_back("critical_battery_threshold 超出 [0,1]: " +
                          std::to_string(critical_battery_threshold_));
    }
    if (critical_battery_threshold_ >= low_battery_threshold_) {
        errors.push_back("critical_battery_threshold (" +
                          std::to_string(critical_battery_threshold_) +
                          ") 必须小于 low_battery_threshold (" +
                          std::to_string(low_battery_threshold_) + ")");
    }
    if (!errors.empty()) {
        std::string msg = "配置参数校验失败:\n  ";
        for (size_t i = 0; i < errors.size(); ++i) {
            if (i > 0) msg += "\n  ";
            msg += errors[i];
        }
        std::fprintf(stderr, "[HardwareInterface] %s\n", msg.c_str());
        throw std::invalid_argument(msg);
    }
}

std::unique_ptr<HardwareInterface> HardwareInterface::create(
    const Config& config) {
    // 根据 config["backend"] 选择实例:
    //   "sim"/"simulation"/"coppelia"/"mock" → MockHardwareInterface
    //   "real"/"puppypi"/"hardware" → PuppyPiHardwareInterface
    // 默认为 "sim"
    std::string backend = "sim";
    auto it = config.find("backend");
    if (it != config.end()) {
        backend = toLower(it->second);
    }

    if (backend == "real" || backend == "puppypi" || backend == "hardware") {
        return std::make_unique<PuppyPiHardwareInterface>(config);
    }
    // sim / simulation / coppelia / mock / 默认 都使用 MockHardwareInterface
    // (sim_interface 在 C++ 版本中未实现，与 Python 回退到 mock 的行为一致)
    return std::make_unique<MockHardwareInterface>(config);
}

void HardwareInterface::send_velocity(double vx, double vy, double wz) {
    if (emergency_stopped_) {
        dispatch_motion(0.0, 0.0, 0.0);
        return;
    }
    if (!motors_enabled_) {
        dispatch_motion(0.0, 0.0, 0.0);
        return;
    }

    // 安全限幅
    vx = std::clamp(vx, -max_linear_x_, max_linear_x_);
    vy = std::clamp(vy, -max_linear_y_, max_linear_y_);
    wz = std::clamp(wz, -max_angular_z_, max_angular_z_);

    // 加速度限制 (防止电机冲击)
    double prev_vx = std::get<0>(last_velocity_);
    double prev_vy = std::get<1>(last_velocity_);
    double prev_wz = std::get<2>(last_velocity_);
    double dt = std::max(nowSeconds() - last_cmd_time_, 0.001);
    vx = limit_accel(vx, prev_vx, dt, accel_limit_);
    vy = limit_accel(vy, prev_vy, dt, accel_limit_);
    wz = limit_accel(wz, prev_wz, dt, yaw_rate_limit_);

    dispatch_motion(vx, vy, wz);
    last_velocity_ = std::make_tuple(vx, vy, wz);
    last_cmd_time_ = nowSeconds();
    cmd_count_ += 1;
}

bool HardwareInterface::enable_motors(bool enable) {
    motors_enabled_ = enable;
    if (!enable) {
        dispatch_motion(0.0, 0.0, 0.0);
    }
    return true;
}

void HardwareInterface::emergency_stop(const std::string& reason) {
    emergency_stopped_ = true;
    motors_enabled_ = false;
    dispatch_motion(0.0, 0.0, 0.0);
    // 重置速度状态
    last_velocity_ = std::make_tuple(0.0, 0.0, 0.0);
    error_count_ += 1;
    last_error_ = "EMERGENCY_STOP: " + reason;
}

bool HardwareInterface::release_emergency_stop() {
    emergency_stopped_ = false;
    return true;
}

bool HardwareInterface::command_timeout() const {
    return (nowSeconds() - last_cmd_time_) > cmd_timeout_;
}

std::unordered_map<std::string, std::string> HardwareInterface::stats() const {
    std::unordered_map<std::string, std::string> s;
    s["cmd_count"] = std::to_string(cmd_count_);
    s["error_count"] = std::to_string(error_count_);
    s["last_error"] = last_error_;
    s["motors_enabled"] = motors_enabled_ ? "true" : "false";
    s["emergency_stopped"] = emergency_stopped_ ? "true" : "false";
    s["uptime"] = std::to_string(nowSeconds() - (last_cmd_time_ - cmd_timeout_));
    return s;
}

double HardwareInterface::limit_accel(double target, double current,
                                        double dt, double limit) {
    // 与 Python _limit_accel 一致
    double max_delta = limit * dt;
    double delta = target - current;
    if (std::abs(delta) > max_delta) {
        delta = std::copysign(max_delta, delta);
    }
    return current + delta;
}

// ============================================================================
// Watchdog 实现
// ============================================================================

Watchdog::Watchdog(double timeout)
    : timeout_(timeout) {}

void Watchdog::register_thread(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    heartbeats_[name] = nowSeconds();
    dead_threads_[name] = false;
}

void Watchdog::heartbeat(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    heartbeats_[name] = nowSeconds();
    dead_threads_[name] = false;
}

std::vector<std::string> Watchdog::check() {
    double now = nowSeconds();
    std::vector<std::string> dead;
    std::vector<TimeoutCallback> cbs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& kv : heartbeats_) {
            if (now - kv.second > timeout_) {
                auto it = dead_threads_.find(kv.first);
                if (it == dead_threads_.end() || !it->second) {
                    dead_threads_[kv.first] = true;
                    dead.push_back(kv.first);
                }
            }
        }
        cbs = callbacks_;
    }

    // 触发回调（锁外执行，避免回调中再次获取锁造成死锁）
    for (const auto& name : dead) {
        std::printf("[Watchdog] 线程 '%s' 心跳超时 (>%gs)\n",
                    name.c_str(), timeout_);
        for (const auto& cb : cbs) {
            try {
                cb(name);
            } catch (...) {
                std::fprintf(stderr, "[Watchdog] 回调失败\n");
            }
        }
    }
    return dead;
}

bool Watchdog::is_alive(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = heartbeats_.find(name);
    if (it == heartbeats_.end()) {
        return false;
    }
    return (nowSeconds() - it->second) <= timeout_;
}

void Watchdog::add_timeout_callback(TimeoutCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_.push_back(std::move(callback));
}

}  // namespace puppypi_adapter
