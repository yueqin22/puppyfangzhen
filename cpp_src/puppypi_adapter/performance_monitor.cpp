// 实时性能监控 + 压力测试 + Fuzz Testing 实现
//
// 对应 Python: puppypi_adapter/performance_monitor.py
// 纯工具类（非 ROS2 节点），不包含 main()
#include "puppypi_adapter/performance_monitor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <utility>

#ifdef __linux__
#include <sys/resource.h>
#include <unistd.h>
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

// ============================================================================
// PerformanceMonitor
// ============================================================================

PerformanceMonitor::PerformanceMonitor(double target_fps,
                                       double warning_threshold,
                                       double error_threshold)
    : target_fps_(target_fps),
      target_frame_time_(1.0 / target_fps),
      warning_threshold_(warning_threshold),
      error_threshold_(error_threshold) {}

PerformanceMonitor::~PerformanceMonitor() = default;

double PerformanceMonitor::nowSeconds() {
    return ::puppypi_adapter::nowSeconds();
}

double PerformanceMonitor::getProcessMemoryMB() {
#ifdef __linux__
    // 读取 /proc/self/statm 中的 RSS（页数）
    std::ifstream f("/proc/self/statm");
    if (!f.is_open()) {
        return 0.0;
    }
    long total_pages = 0;
    long rss_pages = 0;
    f >> total_pages >> rss_pages;
    long page_size = sysconf(_SC_PAGESIZE);
    return static_cast<double>(rss_pages * page_size) / (1024.0 * 1024.0);
#else
    // 非 Linux 平台返回 0（与 Python 版无 psutil 时行为一致）
    return 0.0;
#endif
}

void PerformanceMonitor::startFrame(const std::string& task_name) {
    current_frame_.frame_id = frame_id_;
    current_frame_.start_time = nowSeconds();
    current_frame_.task_name = task_name;
    current_frame_.warning_threshold = warning_threshold_;
    current_frame_.error_threshold = error_threshold_;
    current_frame_.duration = 0.0;
    current_frame_.end_time = 0.0;
    has_current_frame_ = true;
}

bool PerformanceMonitor::endFrame(FrameTiming& out) {
    if (!has_current_frame_) {
        return false;
    }
    FrameTiming frame = current_frame_;
    frame.end_time = nowSeconds();
    frame.duration = frame.end_time - frame.start_time;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_history_.push_back(frame);
        if (frame_history_.size() > 1000) {
            // 保留最近 500 帧
            frame_history_.erase(
                frame_history_.begin(),
                frame_history_.begin() + (frame_history_.size() - 500));
        }

        total_frames_ += 1;
        if (frame.exceeded_warning()) {
            warning_frames_ += 1;
        }
        if (frame.exceeded_error()) {
            error_frames_ += 1;
        }

        // 更新 FPS（最近 30 帧）
        if (frame_history_.size() >= 2) {
            size_t start_idx = (frame_history_.size() >= 30)
                                    ? (frame_history_.size() - 30)
                                    : 0;
            const auto& first = frame_history_[start_idx];
            const auto& last = frame_history_.back();
            double dt = last.end_time - first.start_time;
            if (dt > 0) {
                double fps = static_cast<double>(frame_history_.size() - start_idx - 1) / dt;
                fps_history_.push_back(fps);
                if (fps_history_.size() > 100) {
                    fps_history_.erase(
                        fps_history_.begin(),
                        fps_history_.begin() + (fps_history_.size() - 50));
                }
            }
        }

        updateDegradation();
        recordMemory();
    }

    // 超时回调（锁外执行，避免回调中再次获取锁造成死锁）
    if (frame.exceeded_warning()) {
        for (const auto& cb : timeout_callbacks_) {
            try {
                cb(frame);
            } catch (...) {
                // 忽略回调异常
            }
        }
    }

    frame_id_ += 1;
    has_current_frame_ = false;
    out = frame;
    return true;
}

void PerformanceMonitor::updateDegradation() {
    if (fps_history_.empty()) {
        return;
    }
    double avg_fps;
    if (fps_history_.size() >= 10) {
        // 取最近 10 个的均值
        double sum = 0.0;
        size_t start = fps_history_.size() - 10;
        for (size_t i = start; i < fps_history_.size(); ++i) {
            sum += fps_history_[i];
        }
        avg_fps = sum / 10.0;
    } else {
        avg_fps = fps_history_.back();
    }

    if (avg_fps >= target_fps_ * 0.83) {       // >= 25 FPS (target 30)
        degradation_level_ = 0;
    } else if (avg_fps >= target_fps_ * 0.50) { // 15-25 FPS
        degradation_level_ = 1;
    } else if (avg_fps >= target_fps_ * 0.33) { // 10-15 FPS
        degradation_level_ = 2;
    } else {                                     // < 10 FPS
        degradation_level_ = 3;
    }
}

void PerformanceMonitor::recordMemory() {
    double mem_mb = getProcessMemoryMB();
    if (mem_mb > 0.0) {
        memory_history_.push_back(mem_mb);
        if (memory_history_.size() > 1000) {
            memory_history_.erase(
                memory_history_.begin(),
                memory_history_.begin() + (memory_history_.size() - 500));
        }
    }
}

void PerformanceMonitor::addTimeoutCallback(TimeoutCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_callbacks_.push_back(std::move(callback));
}

double PerformanceMonitor::getSpeedFactor() const {
    // 0=1.0, 1=0.5, 2=0.2, 3=0.0
    static const double factors[4] = {1.0, 0.5, 0.2, 0.0};
    int level = degradation_level_;
    if (level < 0) level = 0;
    if (level > 3) level = 3;
    return factors[level];
}

PerformanceMonitor::Stats PerformanceMonitor::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats s;
    s.total_frames = total_frames_;
    if (!fps_history_.empty()) {
        size_t start = (fps_history_.size() >= 10)
                            ? (fps_history_.size() - 10)
                            : 0;
        double sum = 0.0;
        size_t n = fps_history_.size() - start;
        for (size_t i = start; i < fps_history_.size(); ++i) {
            sum += fps_history_[i];
        }
        s.avg_fps = (n > 0) ? (sum / static_cast<double>(n)) : 0.0;
    } else {
        s.avg_fps = 0.0;
    }
    if (!frame_history_.empty()) {
        size_t start = (frame_history_.size() >= 30)
                            ? (frame_history_.size() - 30)
                            : 0;
        double sum = 0.0;
        size_t n = frame_history_.size() - start;
        for (size_t i = start; i < frame_history_.size(); ++i) {
            sum += frame_history_[i].duration;
        }
        s.avg_frame_time_ms = (n > 0) ? (sum / static_cast<double>(n) * 1000.0)
                                       : 0.0;
    } else {
        s.avg_frame_time_ms = 0.0;
    }
    s.target_fps = target_fps_;
    s.warning_frames = warning_frames_;
    s.error_frames = error_frames_;
    s.warning_rate = static_cast<double>(warning_frames_) /
                      std::max(total_frames_, 1L);
    s.error_rate = static_cast<double>(error_frames_) /
                    std::max(total_frames_, 1L);
    s.degradation_level = degradation_level_;
    s.memory_mb = memory_history_.empty() ? 0.0 : memory_history_.back();
    return s;
}

// ============================================================================
// StressTester
// ============================================================================

StressTester::StressTester(double duration_hours,
                           double cycle_interval,
                           double memory_leak_threshold)
    : duration_seconds_(duration_hours * 3600.0),
      cycle_interval_(cycle_interval),
      memory_leak_threshold_(memory_leak_threshold) {}

StressTester::~StressTester() = default;

StressTester::Report StressTester::run(TestFunction test_function,
                                        ProgressCallback progress_callback) {
    start_time_ = nowSeconds();
    double end_time = start_time_ + duration_seconds_;

    std::printf("[StressTester] 压力测试开始: %.1f 小时\n",
                duration_seconds_ / 3600.0);

    while (nowSeconds() < end_time) {
        try {
            monitor_.startFrame();
            test_function();
            FrameTiming ft;
            monitor_.endFrame(ft);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                cycle_count_ += 1;
                double elapsed = nowSeconds() - start_time_;
                PerformanceMonitor::Stats stats = monitor_.getStats();
                memory_samples_.emplace_back(elapsed, stats.memory_mb);
                fps_samples_.emplace_back(elapsed, stats.avg_fps);

                // 保持采样数量
                if (memory_samples_.size() > 10000) {
                    memory_samples_.erase(
                        memory_samples_.begin(),
                        memory_samples_.begin() +
                            (memory_samples_.size() - 5000));
                }
                if (fps_samples_.size() > 10000) {
                    fps_samples_.erase(
                        fps_samples_.begin(),
                        fps_samples_.begin() +
                            (fps_samples_.size() - 5000));
                }
            }

            // 进度回调
            if (progress_callback && cycle_count_ % 100 == 0) {
                PerformanceMonitor::Stats stats = monitor_.getStats();
                progress_callback(static_cast<int>(cycle_count_), stats);
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            error_count_ += 1;
            if (error_count_ > 100) {
                crash_detected_ = true;
                break;
            }
        }

        std::this_thread::sleep_for(
            std::chrono::duration<double>(cycle_interval_));
    }

    return analyzeResults();
}

StressTester::Report StressTester::analyzeResults() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Report r;
    double duration = nowSeconds() - start_time_;
    r.duration_hours = duration / 3600.0;
    r.total_cycles = cycle_count_;
    r.error_count = error_count_;
    r.error_rate = static_cast<double>(error_count_) /
                    std::max(cycle_count_, 1L);
    r.crash_detected = crash_detected_;

    // 内存泄漏检测 (线性回归: mem = a * time + b)
    r.memory_leak_detected = false;
    r.memory_leak_rate_mb_per_hour = 0.0;
    if (memory_samples_.size() > 10) {
        double n = static_cast<double>(memory_samples_.size());
        double sum_t = 0.0, sum_m = 0.0, sum_tm = 0.0, sum_tt = 0.0;
        for (const auto& s : memory_samples_) {
            double t = s.first / 3600.0;  // hours
            double m = s.second;
            sum_t += t;
            sum_m += m;
            sum_tm += t * m;
            sum_tt += t * t;
        }
        double denom = n * sum_tt - sum_t * sum_t;
        if (std::abs(denom) > 1e-10) {
            double slope = (n * sum_tm - sum_t * sum_m) / denom;
            r.memory_leak_rate_mb_per_hour = slope;
            r.memory_leak_detected = slope > memory_leak_threshold_;
        }
    }
    r.memory_final_mb = memory_samples_.empty()
                            ? 0.0
                            : memory_samples_.back().second;

    // FPS 衰减检测（前后半段均值差）
    r.fps_trend = 0.0;
    r.fps_degraded = false;
    if (fps_samples_.size() > 10) {
        size_t mid = fps_samples_.size() / 2;
        double first_sum = 0.0;
        for (size_t i = 0; i < mid; ++i) first_sum += fps_samples_[i].second;
        double second_sum = 0.0;
        for (size_t i = mid; i < fps_samples_.size(); ++i)
            second_sum += fps_samples_[i].second;
        double first_avg = (mid > 0) ? first_sum / mid : 0.0;
        double second_avg = (fps_samples_.size() - mid > 0)
                                ? second_sum / (fps_samples_.size() - mid)
                                : 0.0;
        r.fps_trend = second_avg - first_avg;
        r.fps_degraded = r.fps_trend < -2.0;  // 下降超过 2 FPS
    }

    r.final_stats = monitor_.getStats();
    r.verdict = getVerdict(r.memory_leak_detected, r.fps_degraded,
                             r.crash_detected);
    return r;
}

std::string StressTester::getVerdict(bool leak, bool fps_deg,
                                       bool crash) const {
    if (crash) {
        return "失败：系统崩溃";
    }
    if (leak) {
        return "失败：检测到内存泄漏";
    }
    if (fps_deg) {
        return "警告：性能衰减";
    }
    if (error_count_ > 10) {
        return "警告：错误率较高";
    }
    return "通过：系统稳定运行";
}

// ============================================================================
// SensorFuzzer
// ============================================================================

SensorFuzzer::SensorFuzzer(double noise_level,
                             double mutation_rate,
                             double loss_rate)
    : noise_level_(noise_level),
      mutation_rate_(mutation_rate),
      loss_rate_(loss_rate),
      lidar_noise_std_(noise_level),
      imu_noise_std_(noise_level * 0.5),
      odom_noise_std_(noise_level * 0.1) {}

SensorFuzzer::~SensorFuzzer() = default;

double SensorFuzzer::nowSeconds() {
    return ::puppypi_adapter::nowSeconds();
}

void SensorFuzzer::fuzzLidar(std::vector<double>& ranges) {
    if (!running_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        total_tests_ += 1;
    }
    if (ranges.empty()) {
        return;
    }

    // 高斯噪声
    for (auto& r : ranges) {
        r += normalRandom() * lidar_noise_std_;
    }

    // 随机突变
    if (uniformRandom() < mutation_rate_) {
        std::lock_guard<std::mutex> lock(mutex_);
        mutations_injected_ += 1;
        // 随机选择突变类型
        int mutation_type = static_cast<int>(uniformRandom() * 4.0);
        size_t n_mutate = static_cast<size_t>(
            uniformRandom() * static_cast<double>(
                std::max<size_t>(ranges.size() / 10, 1)) + 1.0);
        n_mutate = std::min(n_mutate, ranges.size());
        for (size_t i = 0; i < n_mutate; ++i) {
            size_t idx = static_cast<size_t>(
                uniformRandom() * ranges.size());
            if (idx >= ranges.size()) idx = ranges.size() - 1;
            switch (mutation_type) {
                case 0:  // extreme
                    ranges[idx] = 50.0 + uniformRandom() * 50.0;
                    break;
                case 1:  // negative
                    ranges[idx] = -uniformRandom() * 5.0;
                    break;
                case 2:  // zero
                    ranges[idx] = 0.0;
                    break;
                case 3:  // max
                    ranges[idx] = 100.0;
                    break;
            }
        }
    }

    // 数据丢失
    if (uniformRandom() < loss_rate_) {
        std::lock_guard<std::mutex> lock(mutex_);
        losses_injected_ += 1;
        for (auto& r : ranges) {
            if (uniformRandom() < 0.5) {
                r = 0.0;
            }
        }
    }

    // 限制范围
    for (auto& r : ranges) {
        r = std::clamp(r, -10.0, 100.0);
    }
}

void SensorFuzzer::fuzzImu(double& ax, double& ay, double& az,
                            double& gx, double& gy, double& gz) {
    if (!running_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        total_tests_ += 1;
    }

    // 高斯噪声
    ax += normalRandom() * imu_noise_std_;
    ay += normalRandom() * imu_noise_std_;
    az += normalRandom() * imu_noise_std_;
    gx += normalRandom() * imu_noise_std_;
    gy += normalRandom() * imu_noise_std_;
    gz += normalRandom() * imu_noise_std_;

    // 突变
    if (uniformRandom() < mutation_rate_) {
        std::lock_guard<std::mutex> lock(mutex_);
        mutations_injected_ += 1;
        int idx = static_cast<int>(uniformRandom() * 3.0);
        if (idx == 0) ax = 1000.0;
        else if (idx == 1) ay = -1000.0;
        else az = 1000.0;
    }
}

void SensorFuzzer::fuzzOdometry(double& x, double& y, double& yaw) {
    if (!running_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        total_tests_ += 1;
    }

    // 高斯噪声
    x += normalRandom() * odom_noise_std_;
    y += normalRandom() * odom_noise_std_;
    yaw += normalRandom() * odom_noise_std_ * 2.0;

    // 突变（位置跳变）
    if (uniformRandom() < mutation_rate_) {
        std::lock_guard<std::mutex> lock(mutex_);
        mutations_injected_ += 1;
        x += (uniformRandom() - 0.5) * 10.0;
        y += (uniformRandom() - 0.5) * 10.0;
    }
}

void SensorFuzzer::recordCrash() {
    std::lock_guard<std::mutex> lock(mutex_);
    crashes_detected_ += 1;
}

void SensorFuzzer::recordException() {
    std::lock_guard<std::mutex> lock(mutex_);
    exceptions_caught_ += 1;
}

std::string SensorFuzzer::evaluateRobustness() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (crashes_detected_ > 0) {
        std::ostringstream oss;
        oss << "失败：" << crashes_detected_ << " 次崩溃";
        return oss.str();
    }
    if (exceptions_caught_ > total_tests_ / 10) {
        return "警告：异常率过高";
    }
    if (exceptions_caught_ > 0) {
        std::ostringstream oss;
        oss << "良好：捕获 " << exceptions_caught_ << " 次异常，无崩溃";
        return oss.str();
    }
    return "优秀：无崩溃无异常";
}

SensorFuzzer::Report SensorFuzzer::generateReport() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Report r;
    r.total_tests = total_tests_;
    r.mutations_injected = mutations_injected_;
    r.losses_injected = losses_injected_;
    r.crashes_detected = crashes_detected_;
    r.exceptions_caught = exceptions_caught_;
    r.crash_rate = static_cast<double>(crashes_detected_) /
                    std::max(total_tests_, 1L);
    r.exception_rate = static_cast<double>(exceptions_caught_) /
                        std::max(total_tests_, 1L);
    // 评估鲁棒性（需要再获取锁，但已经持锁，所以直接内联实现）
    if (crashes_detected_ > 0) {
        std::ostringstream oss;
        oss << "失败：" << crashes_detected_ << " 次崩溃";
        r.verdict = oss.str();
    } else if (exceptions_caught_ > total_tests_ / 10) {
        r.verdict = "警告：异常率过高";
    } else if (exceptions_caught_ > 0) {
        std::ostringstream oss;
        oss << "良好：捕获 " << exceptions_caught_ << " 次异常，无崩溃";
        r.verdict = oss.str();
    } else {
        r.verdict = "优秀：无崩溃无异常";
    }
    return r;
}

}  // namespace puppypi_adapter
