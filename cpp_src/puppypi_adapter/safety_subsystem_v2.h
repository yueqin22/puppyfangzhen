// 三级急停 + 分级告警 + 降级策略 + 自动上报
//
// 实现项目内存中的硬性安全要求:
//   - 三级急停（硬件按钮 + 软件 API + 远程关机）
//   - 分级告警 (WARN/ERROR/FATAL) + 降级策略 + 自动上报
//   - 安全子系统: 碰撞检测 → 紧急制动 → 风险评估 → 恢复策略闭环
//
// 三级急停架构:
//   Level 1 (软件 API): emergency_stop() 调用
//     - 触发: 碰撞检测、命令超时、软件故障
//     - 响应时间: < 50ms
//     - 恢复: release_emergency_stop()
//   Level 2 (硬件按钮): GPIO 中断/轮询
//     - 触发: 物理急停按钮按下
//     - 响应时间: < 10ms (硬件级)
//     - 恢复: 按钮释放 + 软件确认
//   Level 3 (远程关机): 网络/串口命令
//     - 触发: 远程监控中心下发关机命令
//     - 响应时间: < 1s
//     - 恢复: 手动重启
//
// 降级策略:
//   - LiDAR 故障 → 降低最大速度 50%，仅用里程计
//   - IMU 故障 → 禁用旋转，仅直线运动
//   - 电池低电量 → 返回充电桩
//   - CPU 过热 → 降低控制频率
//   - 通信中断 → 本地安全模式
//
// 对应 Python: puppypi_adapter/safety_subsystem_v2.py
// 纯工具类（非 ROS2 节点），不包含 main()
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "puppypi_adapter/hardware_interface.h"

namespace puppypi_adapter {

/// 告警等级（对应 Python: AlertLevel IntEnum，严重程度递增）
enum class AlertLevel : int {
    OK = 0,        // 正常
    INFO = 1,      // 信息（无需处理）
    WARN = 2,      // 警告（需减速）
    ERROR = 3,     // 错误（需停止任务）
    FATAL = 4,     // 致命（立即断电）
};

/// 急停来源（对应 Python: EStopSource IntEnum）
enum class EStopSource : int {
    SOFTWARE_API = 1,       // Level 1: 软件 API
    HARDWARE_BUTTON = 2,    // Level 2: 硬件按钮
    REMOTE_SHUTDOWN = 3,    // Level 3: 远程关机
    COLLISION_DETECT = 4,   // 碰撞检测
    WATCHDOG = 5,           // 看门狗
    THERMAL = 6,            // 过热保护
};

/// 降级模式（对应 Python: DegradationMode IntEnum）
enum class DegradationMode : int {
    FULL = 0,            // 全功能
    REDUCED_SPEED = 1,   // 降速运行
    SENSORS_DOWN = 2,    // 传感器降级
    SAFE_STOP = 3,       // 安全停车
    EMERGENCY = 4,       // 紧急模式
};

/// 告警事件（对应 Python: @dataclass AlertEvent）
struct AlertEvent {
    AlertLevel level{AlertLevel::OK};
    std::string source;
    std::string message;
    double timestamp{0.0};
    std::unordered_map<std::string, std::string> data;
    bool acknowledged{false};
    std::string recovery_action;
};

/// 降级策略配置（对应 Python: @dataclass DegradationPolicy）
struct DegradationPolicy {
    double sensor_fault_speed_limit{0.15};   // 传感器故障时速度限制
    bool imu_fault_disable_rotation{true};   // IMU故障禁用旋转
    bool battery_low_return_home{true};      // 低电量回充
    double cpu_throttle_threshold{75.0};      // CPU温度降频阈值
    bool comm_timeout_local_mode{true};      // 通信断开进本地模式
};

/// 完整安全子系统（对应 Python: SafetySubsystem）
///
/// 集成三级急停、分级告警、降级策略、自动上报。
/// 与 HardwareInterface 协作实现安全闭环。
class SafetySubsystem {
public:
    using ReportCallback = std::function<void(const AlertEvent&)>;
    using RemoteShutdownListener = std::function<bool()>;

    SafetySubsystem(HardwareInterface* hw_interface = nullptr,
                    DegradationPolicy policy = DegradationPolicy{});
    ~SafetySubsystem();

    SafetySubsystem(const SafetySubsystem&) = delete;
    SafetySubsystem& operator=(const SafetySubsystem&) = delete;

    // === 生命周期 ===
    /// 启动安全子系统（开始远程关机监听）
    void start();
    /// 停止安全子系统
    void stop();

    // === 三级急停 ===
    /// 触发急停（三级急停统一入口）
    /// @param reason 急停原因描述
    /// @param source 急停来源
    void emergency_stop(const std::string& reason,
                        EStopSource source = EStopSource::SOFTWARE_API);
    /// 释放急停状态
    /// @return true 如果成功释放
    bool release_emergency_stop();
    /// 注册远程关机监听器
    /// @param listener 返回 true 表示收到远程关机命令
    void register_remote_shutdown_listener(RemoteShutdownListener listener);

    // === 分级告警 ===
    /// 上报告警（公开接口）
    void raise_alert(AlertLevel level,
                     const std::string& source,
                     const std::string& message,
                     const std::unordered_map<std::string, std::string>& data = {},
                     const std::string& recovery_action = "");
    /// 确认告警
    /// @param index 告警在活跃列表中的索引
    /// @return true 如果确认成功
    bool acknowledge_alert(int index);
    /// 清除指定等级及以下的已确认告警
    void clear_alerts(AlertLevel level = AlertLevel::WARN);

    // === 碰撞检测 → 紧急制动 → 风险评估 → 恢复策略 ===
    /// 碰撞风险评估（碰撞检测 → 紧急制动）
    /// @param lidar_ranges LiDAR 距离数据
    /// @param min_safe_distance 最小安全距离 (m)
    /// @return true 如果有碰撞风险
    bool check_collision_risk(const std::vector<double>& lidar_ranges,
                              double min_safe_distance = 0.3);

    /// 风险评估结果
    struct RiskAssessment {
        double score{0.0};
        double ttc{0.0};        // time-to-collision (秒)
        double min_dist{0.0};
        std::string level;      // "OK" / "WARN" / "ERROR" / "FATAL"
    };
    /// 综合风险评估
    /// @param min_obstacle_dist 最近障碍物距离
    /// @param speed 当前速度
    RiskAssessment assess_risk(double min_obstacle_dist, double speed) const;

    /// 获取恢复策略建议
    std::string get_recovery_strategy() const;

    // === 自动上报 ===
    /// 添加自动上报回调
    void add_report_callback(ReportCallback callback);
    /// 导出所有告警历史（用于离线分析）
    std::vector<AlertEvent> export_alerts() const;

    // === 状态查询 ===
    /// 是否处于安全状态（可以继续运行）
    bool is_safe() const;
    /// 是否处于急停状态
    bool is_emergency_stopped() const;
    /// 当前降级模式
    DegradationMode degradation_mode() const;
    /// 活跃告警列表（副本）
    std::vector<AlertEvent> active_alerts() const;
    /// 当前降级模式下的速度限制 [0.0, 1.0]
    double get_speed_limit() const;

    /// 统计信息
    struct Stats {
        long estop_count{0};
        long alert_count{0};
        size_t active_alerts{0};
        std::string degradation;        // 降级模式名称
        bool estop_active{false};
        std::string estop_source;       // 急停来源名称 (空表示无)
        std::string estop_reason;
    };
    Stats get_stats() const;

private:
    /// 内部告警上报
    void raise_alert_internal(AlertLevel level,
                              const std::string& source,
                              const std::string& message,
                              const std::unordered_map<std::string, std::string>& data,
                              const std::string& recovery_action);
    /// 根据告警等级应用降级策略
    void apply_degradation(AlertLevel level,
                           const std::string& source,
                           const std::unordered_map<std::string, std::string>& data);
    /// 获取恢复动作建议
    static std::string get_recovery_action(EStopSource source);
    /// 远程关机监听线程
    void remote_shutdown_loop();

    /// 告警等级名称
    static const char* alert_level_name(AlertLevel l);
    /// 急停来源名称
    static const char* estop_source_name(EStopSource s);
    /// 降级模式名称
    static const char* degradation_name(DegradationMode m);

    /// 当前秒数
    static double nowSeconds();

private:
    HardwareInterface* hw_;  // 非拥有指针
    DegradationPolicy policy_;

    // 急停状态（受 state_lock_ 保护）
    bool estop_active_{false};
    EStopSource estop_source_{EStopSource::SOFTWARE_API};
    std::string estop_reason_;
    double estop_time_{0.0};
    mutable std::mutex state_lock_;

    // 告警历史（受 alert_lock_ 保护）
    std::vector<AlertEvent> alerts_;
    std::vector<AlertEvent> active_alerts_;
    mutable std::mutex alert_lock_;

    // 降级模式（受 state_lock_ 保护）
    DegradationMode degradation_{DegradationMode::FULL};
    std::string degradation_reason_;

    // 碰撞检测状态
    bool collision_imminent_{false};

    // 远程关机监听
    RemoteShutdownListener remote_listener_;
    std::unique_ptr<std::thread> remote_thread_;
    std::atomic<bool> running_{false};

    // 上报回调（受 alert_lock_ 保护）
    std::vector<ReportCallback> report_callbacks_;

    // 统计（受 state_lock_ 保护）
    long estop_count_{0};
    long alert_count_{0};
};

}  // namespace puppypi_adapter
