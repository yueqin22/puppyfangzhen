// 实时性能监控 + 压力测试 + Fuzz Testing
//
// 实现项目内存中的硬性要求:
//   - 实时性能监控，每帧超时告警并自动降级
//   - 24小时连续运行压力测试，内存泄漏检测
//   - Fuzz testing，随机传感器噪声注入
//
// 性能监控:
//   - 每帧执行时间测量
//   - 超时告警 (>50ms 警告, >100ms 错误)
//   - 自动降级 (降低控制频率)
//   - 帧率统计 (FPS)
//
// 压力测试:
//   - 24小时连续运行
//   - 内存使用监控 (RSS)
//   - 内存泄漏检测 (线性回归)
//
// Fuzz Testing:
//   - 随机传感器噪声注入
//   - 边界值测试
//   - 异常输入处理
//
// 对应 Python: puppypi_adapter/performance_monitor.py
// 纯工具类（非 ROS2 节点），不包含 main()
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace puppypi_adapter {

/// 单帧计时信息（对应 Python: @dataclass FrameTiming）
struct FrameTiming {
    long frame_id{0};
    double start_time{0.0};
    double end_time{0.0};
    double duration{0.0};
    std::string task_name;
    double warning_threshold{0.050};
    double error_threshold{0.100};

    bool exceeded_warning() const { return duration > warning_threshold; }
    bool exceeded_error() const { return duration > error_threshold; }
};

/// 实时性能监控器（对应 Python: PerformanceMonitor）
///
/// 监控指标:
///   - 帧执行时间 (ms)
///   - FPS (帧率)
///   - 超时帧比例
///   - 内存使用 (MB)
///
/// 降级策略:
///   - 正常 (FPS >= 25): 全功能
///   - 降速 (FPS 15-25): 降低最大速度 50%
///   - 严重降速 (FPS 10-15): 降低最大速度 80%
///   - 危险 (FPS < 10): 安全停车
class PerformanceMonitor {
public:
    using TimeoutCallback = std::function<void(const FrameTiming&)>;

    PerformanceMonitor(double target_fps = 30.0,
                       double warning_threshold = 0.050,
                       double error_threshold = 0.100);
    ~PerformanceMonitor();

    PerformanceMonitor(const PerformanceMonitor&) = delete;
    PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;

    /// 开始帧计时
    void startFrame(const std::string& task_name = "");

    /// 结束帧计时
    /// @return 帧计时信息（无活跃帧时返回 false）
    bool endFrame(FrameTiming& out);

    /// 添加超时回调
    void addTimeoutCallback(TimeoutCallback callback);

    /// 获取速度限制因子 [0.0, 1.0]
    double getSpeedFactor() const;

    /// 是否处于降级状态
    bool isDegraded() const { return degradation_level_ > 0; }

    /// 统计信息
    struct Stats {
        long total_frames{0};
        double avg_fps{0.0};
        double avg_frame_time_ms{0.0};
        double target_fps{0.0};
        long warning_frames{0};
        long error_frames{0};
        double warning_rate{0.0};
        double error_rate{0.0};
        int degradation_level{0};
        double memory_mb{0.0};
    };
    Stats getStats() const;

private:
    /// 根据 FPS 更新降级级别
    void updateDegradation();
    /// 记录当前进程内存使用（MB）
    void recordMemory();

    /// 获取当前秒数
    static double nowSeconds();
    /// 获取当前进程 RSS（MB）
    static double getProcessMemoryMB();

private:
    double target_fps_;
    double target_frame_time_;
    double warning_threshold_;
    double error_threshold_;

    bool has_current_frame_{false};
    FrameTiming current_frame_;
    std::vector<FrameTiming> frame_history_;
    long frame_id_{0};

    long total_frames_{0};
    long warning_frames_{0};
    long error_frames_{0};
    std::vector<double> fps_history_;
    std::vector<double> memory_history_;

    int degradation_level_{0};  // 0=正常, 1=降速, 2=严重降速, 3=危险

    std::vector<TimeoutCallback> timeout_callbacks_;
    mutable std::mutex mutex_;
};

/// 24 小时压力测试器（对应 Python: StressTester）
///
/// 长时间运行系统，监控内存泄漏和性能衰减。
class StressTester {
public:
    using TestFunction = std::function<void()>;
    using ProgressCallback =
        std::function<void(int /*cycle*/, const PerformanceMonitor::Stats&)>;

    StressTester(double duration_hours = 24.0,
                 double cycle_interval = 1.0,
                 double memory_leak_threshold = 0.1);  // MB/hour
    ~StressTester();

    StressTester(const StressTester&) = delete;
    StressTester& operator=(const StressTester&) = delete;

    /// 运行压力测试
    /// @param test_function 每个循环执行的测试函数
    /// @param progress_callback 进度回调（可为 nullptr）
    /// @return 测试结果（Report）
    struct Report {
        double duration_hours{0.0};
        long total_cycles{0};
        long error_count{0};
        double error_rate{0.0};
        bool crash_detected{false};
        bool memory_leak_detected{false};
        double memory_leak_rate_mb_per_hour{0.0};
        double memory_final_mb{0.0};
        double fps_trend{0.0};
        bool fps_degraded{false};
        PerformanceMonitor::Stats final_stats;
        std::string verdict;
    };
    Report run(TestFunction test_function, ProgressCallback progress_callback);

private:
    /// 分析测试结果（检测内存泄漏、FPS 衰减、错误率）
    Report analyzeResults() const;
    /// 生成测试结论
    std::string getVerdict(bool leak, bool fps_deg, bool crash) const;

private:
    double duration_seconds_;
    double cycle_interval_;
    double memory_leak_threshold_;

    double start_time_{0.0};
    long cycle_count_{0};
    long error_count_{0};
    bool crash_detected_{false};

    // 内存与 FPS 采样：(time, value)
    std::vector<std::pair<double, double>> memory_samples_;
    std::vector<std::pair<double, double>> fps_samples_;

    PerformanceMonitor monitor_;
    mutable std::mutex mutex_;
};

/// 传感器 Fuzz Testing（对应 Python: SensorFuzzer）
///
/// 随机注入传感器噪声，测试系统鲁棒性。
class SensorFuzzer {
public:
    SensorFuzzer(double noise_level = 0.1,
                 double mutation_rate = 0.01,
                 double loss_rate = 0.001);
    ~SensorFuzzer();

    SensorFuzzer(const SensorFuzzer&) = delete;
    SensorFuzzer& operator=(const SensorFuzzer&) = delete;

    /// 对 LiDAR 数据进行 Fuzz
    void fuzzLidar(std::vector<double>& ranges);
    /// 对 IMU 数据进行 Fuzz
    /// @param ax,ay,az 加速度（输入输出）
    /// @param gx,gy,gz 陀螺仪（输入输出）
    void fuzzImu(double& ax, double& ay, double& az,
                 double& gx, double& gy, double& gz);
    /// 对里程计数据进行 Fuzz
    void fuzzOdometry(double& x, double& y, double& yaw);

    /// 开始 Fuzz 测试
    void start() { running_ = true; }
    /// 停止 Fuzz 测试
    void stop() { running_ = false; }

    /// 记录崩溃
    void recordCrash();
    /// 记录异常（被系统捕获）
    void recordException();

    /// 生成 Fuzz 测试报告
    struct Report {
        long total_tests{0};
        long mutations_injected{0};
        long losses_injected{0};
        long crashes_detected{0};
        long exceptions_caught{0};
        double crash_rate{0.0};
        double exception_rate{0.0};
        std::string verdict;
    };
    Report generateReport() const;

private:
    /// 评估系统鲁棒性
    std::string evaluateRobustness() const;
    /// 获取当前秒数
    static double nowSeconds();

private:
    double noise_level_;
    double mutation_rate_;
    double loss_rate_;

    std::atomic<bool> running_{false};

    long total_tests_{0};
    long mutations_injected_{0};
    long losses_injected_{0};
    long crashes_detected_{0};
    long exceptions_caught_{0};

    double lidar_noise_std_;
    double imu_noise_std_;
    double odom_noise_std_;

    mutable std::mutex mutex_;
};

}  // namespace puppypi_adapter
