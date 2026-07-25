// 故障注入测试框架 (Fault Injection Testing)
//
// 实现项目内存中的硬性要求:
//   - 故障注入测试 (模拟传感器故障、通信中断、CPU过载)
//
// 故障类型:
//   1. 传感器故障: LiDAR 超时/噪声/数据丢失, IMU 漂移/饱和, 相机模糊, 编码器跳变
//   2. 通信中断: DDS 延迟, 消息丢失, 节点崩溃
//   3. CPU 过载: 计算延迟, 内存泄漏, 线程饥饿
//   4. 环境干扰: 光照突变, 玻璃表面, 窄通道
//
// 故障注入模型:
//   - 持续时间: 瞬时(<100ms) / 短时(100ms-1s) / 持续(>1s)
//   - 严重程度: 轻微 / 中等 / 严重
//   - 触发方式: 定时 / 随机 / 事件驱动
//
// 对应 Python: puppypi_adapter/fault_injection.py
// 纯工具类（非 ROS2 节点），不包含 main()
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "puppypi_adapter/hardware_interface.h"

namespace puppypi_adapter {

/// 故障类型（对应 Python: FaultType IntEnum）
enum class FaultType : int {
    // 传感器故障
    LIDAR_TIMEOUT = 1,        // LiDAR 超时
    LIDAR_NOISE = 2,          // LiDAR 噪声增大
    LIDAR_DATA_LOSS = 3,      // LiDAR 数据丢失
    IMU_DRIFT = 4,            // IMU 零偏漂移
    IMU_SATURATION = 5,       // IMU 饱和
    CAMERA_BLUR = 6,          // 相机模糊
    ENCODER_JUMP = 7,         // 编码器跳变
    // 通信故障
    COMM_LATENCY = 10,        // 通信延迟
    COMM_PACKET_LOSS = 11,    // 消息丢失
    NODE_CRASH = 12,          // 节点崩溃
    // 性能故障
    CPU_OVERLOAD = 20,        // CPU 过载
    MEMORY_LEAK = 21,         // 内存泄漏
    THREAD_STARVATION = 22,   // 线程饥饿
    // 环境干扰
    LIGHT_CHANGE = 30,        // 光照突变
    GLASS_SURFACE = 31,       // 玻璃表面
    NARROW_CORRIDOR = 32,     // 窄通道
};

/// 故障严重程度（对应 Python: FaultSeverity IntEnum）
enum class FaultSeverity : int {
    MINOR = 1,     // 轻微：系统可正常运行，性能略降
    MODERATE = 2,  // 中等：触发降级，但可继续运行
    SEVERE = 3,    // 严重：触发急停或安全停车
};

/// 故障注入配置（对应 Python: @dataclass FaultInjection）
struct FaultInjection {
    FaultType fault_type{FaultType::LIDAR_TIMEOUT};
    FaultSeverity severity{FaultSeverity::MODERATE};
    double duration{0.0};          // 0 表示持续（对应 Python None）
    double start_delay{0.0};
    std::map<std::string, double> parameters;  // 故障特定参数

    // 运行时状态
    double start_time{0.0};
    bool active{false};
};

/// 故障历史记录条目
struct FaultHistoryEntry {
    double time{0.0};
    std::string action;       // "inject" / "clear"
    std::string fault_type;   // 故障类型名称
    std::string severity;     // 严重程度名称
    double duration{0.0};
};

/// 故障注入器（对应 Python: FaultInjector）
///
/// 向 HardwareInterface 注入各类故障，测试系统的鲁棒性。
/// 包装硬件接口的传感器读取与速度发送方法，应用故障效果。
class FaultInjector {
public:
    /// 构造函数
    /// @param hw_interface 硬件接口实例（可为 nullptr 用于纯逻辑测试）
    explicit FaultInjector(HardwareInterface* hw_interface = nullptr);
    ~FaultInjector();

    FaultInjector(const FaultInjector&) = delete;
    FaultInjector& operator=(const FaultInjector&) = delete;

    /// 注入故障
    /// @param fault_type 故障类型
    /// @param severity 严重程度
    /// @param duration 持续时间(秒)，0 表示持续
    /// @param start_delay 启动延迟(秒)
    /// @param params 故障特定参数（例如 noise_std, drift_rate 等）
    void inject(FaultType fault_type,
                FaultSeverity severity = FaultSeverity::MODERATE,
                double duration = 0.0,
                double start_delay = 0.0,
                const std::map<std::string, double>& params = {});

    /// 清除指定故障
    void clear(FaultType fault_type);

    /// 清除所有故障
    void clearAll();

    /// 读取传感器数据（应用故障效果）
    /// @param out 输出参数，写入故障后的传感器读数
    /// @return true 如果读取成功（硬件接口存在且未丢包）
    bool getSensorReadings(SensorReadings& out);

    /// 发送速度命令（应用通信故障）
    void sendVelocity(double vx, double vy, double wz);

    /// 记录系统检测到故障
    void recordDetection(FaultType fault_type);

    /// 记录系统从故障恢复
    void recordRecovery(FaultType fault_type);

    /// 获取故障历史记录
    std::vector<FaultHistoryEntry> getFaultHistory() const;

    /// 获取当前活跃故障列表
    std::vector<FaultType> getActiveFaults() const;

    /// 统计信息
    struct Stats {
        long injection_count{0};
        long detection_count{0};
        long recovery_count{0};
        double detection_rate{0.0};
        double recovery_rate{0.0};
        std::vector<std::string> active_faults;
    };
    Stats getStats() const;

    /// 生成故障注入测试报告（包含评估结论）
    struct Report {
        Stats summary;
        std::vector<FaultHistoryEntry> fault_history;
        std::string conclusion;
    };
    Report generateReport() const;

private:
    /// 激活故障效果
    void activateFault(FaultInjection& fault);

    /// 根据故障严重程度与参数解析具体参数值
    double resolveParam(const std::map<std::string, double>& params,
                        const std::string& key,
                        FaultSeverity sev,
                        double minor_v, double moderate_v, double severe_v) const;

    /// 启动 CPU 压力测试线程
    void startCpuStress(int load_percent);
    /// 停止 CPU 压力测试线程
    void stopCpuStress();
    /// CPU 压力测试循环
    void cpuStressLoop(int load_percent);

    /// 评估系统鲁棒性
    std::string evaluateRobustness() const;

    /// 故障类型名称（用于日志与历史记录）
    static const char* faultTypeName(FaultType t);
    /// 严重程度名称
    static const char* severityName(FaultSeverity s);

private:
    HardwareInterface* hw_;  // 非拥有指针
    mutable std::mutex mutex_;
    std::unordered_map<int, FaultInjection> faults_;  // key = static_cast<int>(FaultType)

    // 故障效果状态
    double lidar_noise_level_{0.0};     // LiDAR 噪声标准差
    double imu_drift_rate_{0.0};        // IMU 漂移率 (rad/s)
    double imu_bias_[3]{0.0, 0.0, 0.0}; // IMU 零偏
    double encoder_offset_{0.0};        // 编码器偏移
    double comm_latency_{0.0};          // 通信延迟 (秒)
    double packet_loss_rate_{0.0};       // 消息丢失率

    // 统计
    long injection_count_{0};
    long detection_count_{0};
    long recovery_count_{0};

    // 故障历史
    std::vector<FaultHistoryEntry> fault_history_;

    // 最近一次传感器读数（用于模拟丢包时返回上次数据）
    SensorReadings last_readings_;
    bool has_last_readings_{false};

    // CPU 过载模拟线程
    std::unique_ptr<std::thread> cpu_stress_thread_;
    std::atomic<bool> cpu_stress_running_{false};
};

}  // namespace puppypi_adapter
