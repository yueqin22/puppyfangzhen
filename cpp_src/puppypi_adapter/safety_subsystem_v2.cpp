// 三级急停 + 分级告警 + 降级策略 + 自动上报 实现
//
// 对应 Python: puppypi_adapter/safety_subsystem_v2.py
// 纯工具类（非 ROS2 节点），不包含 main()
#include "puppypi_adapter/safety_subsystem_v2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

namespace puppypi_adapter {

namespace {

double nowSeconds() {
    auto t = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

}  // namespace

SafetySubsystem::SafetySubsystem(HardwareInterface* hw_interface,
                                   DegradationPolicy policy)
    : hw_(hw_interface),
      policy_(policy) {}

SafetySubsystem::~SafetySubsystem() {
    stop();
}

double SafetySubsystem::nowSeconds() {
    return ::puppypi_adapter::nowSeconds();
}

const char* SafetySubsystem::alert_level_name(AlertLevel l) {
    switch (l) {
        case AlertLevel::OK:     return "OK";
        case AlertLevel::INFO:   return "INFO";
        case AlertLevel::WARN:   return "WARN";
        case AlertLevel::ERROR:  return "ERROR";
        case AlertLevel::FATAL:  return "FATAL";
    }
    return "UNKNOWN";
}

const char* SafetySubsystem::estop_source_name(EStopSource s) {
    switch (s) {
        case EStopSource::SOFTWARE_API:      return "SOFTWARE_API";
        case EStopSource::HARDWARE_BUTTON:    return "HARDWARE_BUTTON";
        case EStopSource::REMOTE_SHUTDOWN:    return "REMOTE_SHUTDOWN";
        case EStopSource::COLLISION_DETECT:   return "COLLISION_DETECT";
        case EStopSource::WATCHDOG:           return "WATCHDOG";
        case EStopSource::THERMAL:            return "THERMAL";
    }
    return "UNKNOWN";
}

const char* SafetySubsystem::degradation_name(DegradationMode m) {
    switch (m) {
        case DegradationMode::FULL:           return "FULL";
        case DegradationMode::REDUCED_SPEED:   return "REDUCED_SPEED";
        case DegradationMode::SENSORS_DOWN:   return "SENSORS_DOWN";
        case DegradationMode::SAFE_STOP:      return "SAFE_STOP";
        case DegradationMode::EMERGENCY:      return "EMERGENCY";
    }
    return "UNKNOWN";
}

void SafetySubsystem::start() {
    running_ = true;
    remote_thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { remote_shutdown_loop(); }));
    std::printf("[SafetySubsystem] 安全子系统已启动\n");
}

void SafetySubsystem::stop() {
    running_ = false;
    if (remote_thread_ && remote_thread_->joinable()) {
        remote_thread_->join();
    }
    remote_thread_.reset();
}

void SafetySubsystem::emergency_stop(const std::string& reason,
                                       EStopSource source) {
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        if (estop_active_) {
            return;  // 已急停，不重复
        }
        estop_active_ = true;
        estop_source_ = source;
        estop_reason_ = reason;
        estop_time_ = nowSeconds();
        estop_count_ += 1;
    }

    // 调用硬件急停（锁外调用，避免死锁）
    if (hw_) {
        try {
            std::string combined = std::string(estop_source_name(source)) +
                                   ": " + reason;
            hw_->emergency_stop(combined);
        } catch (...) {
            std::fprintf(stderr,
                         "[SafetySubsystem] 硬件急停调用失败\n");
        }
    }

    // 记录 FATAL 告警
    std::unordered_map<std::string, std::string> data;
    data["source"] = estop_source_name(source);
    data["reason"] = reason;
    raise_alert_internal(AlertLevel::FATAL,
                          std::string("estop_") +
                              estop_source_name(source),
                          std::string("急停触发: ") + reason,
                          data,
                          get_recovery_action(source));

    std::printf("[SafetySubsystem] 急停触发 [%s]: %s\n",
                estop_source_name(source), reason.c_str());
}

bool SafetySubsystem::release_emergency_stop() {
    // 先在锁内读取状态
    EStopSource src;
    bool active;
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        if (!estop_active_) {
            return true;
        }
        src = estop_source_;
        active = estop_active_;
    }

    // 远程关机需要特殊处理
    if (src == EStopSource::REMOTE_SHUTDOWN) {
        std::printf("[SafetySubsystem] 远程关机急停需要人工重启\n");
        return false;
    }

    // 硬件按钮急停需要按钮已释放
    // 注意：PuppyPiHardwareInterface::hw_estop_triggered() 是公开方法
    // 此处保守处理：仅检查通过 HardwareInterface 基类不可访问的 hw_estop_triggered
    // 由于 C++ 多态下基类无此方法，此处省略具体检查（与 Python 版略有不同）
    // TODO: 若需严格检查，需通过 dynamic_cast<PuppyPiHardwareInterface*>

    // 释放硬件急停
    if (hw_) {
        try {
            if (!hw_->release_emergency_stop()) {
                std::fprintf(stderr,
                             "[SafetySubsystem] 硬件释放急停失败\n");
                return false;
            }
        } catch (...) {
            std::fprintf(stderr,
                         "[SafetySubsystem] 硬件释放急停失败\n");
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(state_lock_);
        estop_active_ = false;
        estop_source_ = EStopSource::SOFTWARE_API;
        estop_reason_.clear();
        degradation_ = DegradationMode::FULL;
        degradation_reason_.clear();
    }

    std::printf("[SafetySubsystem] 急停已释放\n");
    (void)active;
    return true;
}

void SafetySubsystem::register_remote_shutdown_listener(
    RemoteShutdownListener listener) {
    std::lock_guard<std::mutex> lock(state_lock_);
    remote_listener_ = std::move(listener);
}

void SafetySubsystem::raise_alert(
    AlertLevel level,
    const std::string& source,
    const std::string& message,
    const std::unordered_map<std::string, std::string>& data,
    const std::string& recovery_action) {
    raise_alert_internal(level, source, message, data, recovery_action);
}

void SafetySubsystem::raise_alert_internal(
    AlertLevel level,
    const std::string& source,
    const std::string& message,
    const std::unordered_map<std::string, std::string>& data,
    const std::string& recovery_action) {
    AlertEvent alert;
    alert.level = level;
    alert.source = source;
    alert.message = message;
    alert.timestamp = nowSeconds();
    alert.data = data;
    alert.recovery_action = recovery_action;

    // 收集回调副本（锁内）然后在锁外调用
    std::vector<ReportCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(alert_lock_);
        alerts_.push_back(alert);
        alert_count_ += 1;
        // 保持历史不超过 1000 条
        if (alerts_.size() > 1000) {
            alerts_.erase(
                alerts_.begin(),
                alerts_.begin() + (alerts_.size() - 500));
        }
        // 添加到活跃告警（FATAL/ERROR/WARN）
        if (static_cast<int>(level) >= static_cast<int>(AlertLevel::WARN)) {
            active_alerts_.push_back(alert);
        }
        callbacks = report_callbacks_;
    }

    // 根据等级执行降级
    apply_degradation(level, source, data);

    // 触发上报回调（锁外调用，避免回调中再次获取锁造成死锁）
    for (const auto& cb : callbacks) {
        try {
            cb(alert);
        } catch (...) {
            // 忽略回调异常
        }
    }

    // 日志
    if (static_cast<int>(level) >= static_cast<int>(AlertLevel::ERROR)) {
        std::fprintf(stderr, "[%s] %s: %s\n",
                     alert_level_name(level), source.c_str(),
                     message.c_str());
    } else if (static_cast<int>(level) >=
               static_cast<int>(AlertLevel::WARN)) {
        std::printf("[%s] %s: %s\n",
                    alert_level_name(level), source.c_str(),
                    message.c_str());
    } else {
        std::printf("[%s] %s: %s\n",
                    alert_level_name(level), source.c_str(),
                    message.c_str());
    }
}

bool SafetySubsystem::acknowledge_alert(int index) {
    std::lock_guard<std::mutex> lock(alert_lock_);
    if (index < 0 || index >= static_cast<int>(active_alerts_.size())) {
        return false;
    }
    active_alerts_[index].acknowledged = true;
    // 已确认的 WARN 可移除
    if (active_alerts_[index].level == AlertLevel::WARN &&
        active_alerts_[index].acknowledged) {
        active_alerts_.erase(active_alerts_.begin() + index);
    }
    return true;
}

void SafetySubsystem::clear_alerts(AlertLevel level) {
    std::lock_guard<std::mutex> lock(alert_lock_);
    int threshold = static_cast<int>(level);
    active_alerts_.erase(
        std::remove_if(active_alerts_.begin(), active_alerts_.end(),
                       [threshold](const AlertEvent& a) {
                           return static_cast<int>(a.level) <= threshold &&
                                  a.acknowledged;
                       }),
        active_alerts_.end());
}

void SafetySubsystem::apply_degradation(
    AlertLevel level,
    const std::string& source,
    const std::unordered_map<std::string, std::string>& /*data*/) {
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        if (level == AlertLevel::FATAL) {
            degradation_ = DegradationMode::EMERGENCY;
            degradation_reason_ = std::string("FATAL: ") + source;
        } else if (level == AlertLevel::ERROR) {
            // 判断是否为传感器故障（source 中包含 "sensor"）
            std::string source_lower = source;
            std::transform(source_lower.begin(), source_lower.end(),
                           source_lower.begin(),
                           [](unsigned char c) {
                               return static_cast<char>(std::tolower(c));
                           });
            if (source_lower.find("sensor") != std::string::npos) {
                degradation_ = DegradationMode::SENSORS_DOWN;
                degradation_reason_ = std::string("传感器故障: ") + source;
            } else {
                degradation_ = DegradationMode::SAFE_STOP;
                degradation_reason_ = std::string("错误: ") + source;
            }
        } else if (level == AlertLevel::WARN) {
            if (static_cast<int>(degradation_) <
                static_cast<int>(DegradationMode::REDUCED_SPEED)) {
                degradation_ = DegradationMode::REDUCED_SPEED;
                degradation_reason_ = std::string("警告: ") + source;
            }
        }
    }

    // FATAL 触发急停（锁外调用，避免死锁）
    if (level == AlertLevel::FATAL) {
        emergency_stop(std::string("FATAL alert: ") + source,
                       EStopSource::SOFTWARE_API);
    }
    // 错误时停止运动（锁外调用）
    else if (level == AlertLevel::ERROR && hw_) {
        try {
            hw_->send_velocity(0.0, 0.0, 0.0);
        } catch (...) {
            std::fprintf(stderr,
                         "[SafetySubsystem] 停止运动失败\n");
        }
    }
}

double SafetySubsystem::get_speed_limit() const {
    DegradationMode mode = degradation_mode();
    if (mode == DegradationMode::FULL) {
        return 1.0;
    }
    if (mode == DegradationMode::REDUCED_SPEED) {
        return 0.5;
    }
    if (mode == DegradationMode::SENSORS_DOWN) {
        // 修复除零风险：max_linear_x 可能为 0
        double max_vx = 0.3;
        if (hw_ && hw_->max_linear_x() > 0) {
            max_vx = hw_->max_linear_x();
        }
        return std::min(policy_.sensor_fault_speed_limit / max_vx, 1.0);
    }
    return 0.0;  // SAFE_STOP / EMERGENCY
}

bool SafetySubsystem::check_collision_risk(
    const std::vector<double>& lidar_ranges,
    double min_safe_distance) {
    if (lidar_ranges.empty()) {
        return false;
    }

    // 过滤 NaN/Inf/非正数（传感器异常数据）
    std::vector<double> valid;
    valid.reserve(lidar_ranges.size());
    for (double r : lidar_ranges) {
        if (!std::isnan(r) && !std::isinf(r) && r > 0) {
            valid.push_back(r);
        }
    }
    if (valid.empty()) {
        return false;
    }

    // 检查前方 ±30° 范围（前 1/6 的扫描点）
    size_t n = valid.size();
    size_t front_start = n * 5 / 12;
    size_t front_end = n * 7 / 12;
    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = front_start; i < front_end && i < n; ++i) {
        if (valid[i] < min_dist) {
            min_dist = valid[i];
        }
    }

    if (min_dist < min_safe_distance * 0.5) {
        // 紧急制动
        collision_imminent_ = true;
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "碰撞 imminent: min_dist=%.2fm", min_dist);
        emergency_stop(buf, EStopSource::COLLISION_DETECT);
        return true;
    }
    if (min_dist < min_safe_distance) {
        collision_imminent_ = true;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "碰撞风险: min_dist=%.2fm",
                      min_dist);
        std::unordered_map<std::string, std::string> data;
        data["min_dist"] = std::to_string(min_dist);
        raise_alert(AlertLevel::WARN, "collision_risk", buf, data);
        return true;
    }

    collision_imminent_ = false;
    return false;
}

SafetySubsystem::RiskAssessment SafetySubsystem::assess_risk(
    double min_obstacle_dist, double speed) const {
    RiskAssessment r;
    r.min_dist = min_obstacle_dist;
    r.ttc = (speed > 0) ? (min_obstacle_dist / std::max(speed, 0.01))
                        : std::numeric_limits<double>::infinity();
    if (r.ttc < 1.0) {
        r.score = 100.0;
    } else if (r.ttc < 2.0) {
        r.score = 50.0 * (2.0 - r.ttc);
    } else if (min_obstacle_dist < 0.5) {
        r.score = 30.0;
    }

    if (r.score >= 80) {
        r.level = "FATAL";
    } else if (r.score >= 50) {
        r.level = "ERROR";
    } else if (r.score >= 20) {
        r.level = "WARN";
    } else {
        r.level = "OK";
    }
    return r;
}

std::string SafetySubsystem::get_recovery_strategy() const {
    EStopSource src;
    bool active;
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        active = estop_active_;
        src = estop_source_;
    }
    if (!active) {
        return "正常运行";
    }
    switch (src) {
        case EStopSource::COLLISION_DETECT:
            return "后退避让 → 重新规划路径";
        case EStopSource::HARDWARE_BUTTON:
            return "等待按钮释放 → 人工确认 → 释放急停";
        case EStopSource::REMOTE_SHUTDOWN:
            return "等待远程恢复命令 → 重启系统";
        case EStopSource::WATCHDOG:
            return "重置系统 → 重新初始化 → 恢复运行";
        case EStopSource::THERMAL:
            return "等待温度降低 → 降低负载 → 恢复运行";
        case EStopSource::SOFTWARE_API:
            return "检查软件故障 → 释放急停";
    }
    return "检查故障 → 人工确认 → 释放急停";
}

void SafetySubsystem::add_report_callback(ReportCallback callback) {
    std::lock_guard<std::mutex> lock(alert_lock_);
    report_callbacks_.push_back(std::move(callback));
}

std::vector<AlertEvent> SafetySubsystem::export_alerts() const {
    std::lock_guard<std::mutex> lock(alert_lock_);
    return alerts_;
}

bool SafetySubsystem::is_safe() const {
    std::lock_guard<std::mutex> lock(state_lock_);
    return !estop_active_ &&
           static_cast<int>(degradation_) <
               static_cast<int>(DegradationMode::SAFE_STOP);
}

bool SafetySubsystem::is_emergency_stopped() const {
    std::lock_guard<std::mutex> lock(state_lock_);
    return estop_active_;
}

DegradationMode SafetySubsystem::degradation_mode() const {
    std::lock_guard<std::mutex> lock(state_lock_);
    return degradation_;
}

std::vector<AlertEvent> SafetySubsystem::active_alerts() const {
    std::lock_guard<std::mutex> lock(alert_lock_);
    return active_alerts_;
}

SafetySubsystem::Stats SafetySubsystem::get_stats() const {
    Stats s;
    std::lock_guard<std::mutex> slock(state_lock_);
    s.estop_count = estop_count_;
    s.estop_active = estop_active_;
    s.estop_source = estop_active_ ? estop_source_name(estop_source_) : "";
    s.estop_reason = estop_reason_;
    s.degradation = degradation_name(degradation_);
    s.alert_count = alert_count_;
    {
        std::lock_guard<std::mutex> alock(alert_lock_);
        s.active_alerts = active_alerts_.size();
    }
    return s;
}

std::string SafetySubsystem::get_recovery_action(EStopSource source) {
    switch (source) {
        case EStopSource::SOFTWARE_API:      return "检查软件故障 → 释放急停";
        case EStopSource::HARDWARE_BUTTON:    return "等待按钮释放 → 确认 → 释放";
        case EStopSource::REMOTE_SHUTDOWN:    return "等待远程恢复 → 重启";
        case EStopSource::COLLISION_DETECT:   return "后退 → 重新规划";
        case EStopSource::WATCHDOG:           return "重置 → 重新初始化";
        case EStopSource::THERMAL:            return "降温 → 降低负载 → 恢复";
    }
    return "人工检查";
}

void SafetySubsystem::remote_shutdown_loop() {
    // 远程关机监听线程（2Hz 检查）
    while (running_) {
        try {
            RemoteShutdownListener listener;
            {
                std::lock_guard<std::mutex> lock(state_lock_);
                listener = remote_listener_;
            }
            if (listener && listener()) {
                emergency_stop("远程关机命令",
                               EStopSource::REMOTE_SHUTDOWN);
                // 远程关机后停止监听
                break;
            }
        } catch (...) {
            // 忽略监听错误
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

}  // namespace puppypi_adapter
