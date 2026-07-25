// 硬件抽象接口基类 (Hardware Abstraction Interface)
//
// 定义仿真和实体机统一的硬件接口，使上层导航/安全/任务逻辑
// 无需关心底层是 CoppeliaSim 仿真还是 PuppyPi 实体硬件。
//
// 设计原则（项目内存硬性约束）:
//   - 仿真和实体机接口对上层一致呈现
//   - 运动控制环以固定频率运行 (≥20Hz, 目标30Hz)
//   - 传感器数据流支持异步读取
//   - 电池状态支持低电量自动回充
//   - 所有参数通过 YAML 配置，不硬编码
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace puppypi_adapter {

/// 传感器读数集合
struct SensorReadings {
    std::vector<double> lidar_angles;
    std::vector<double> lidar_ranges;
    double imu_ax{0.0}, imu_ay{0.0}, imu_az{0.0};
    double imu_gx{0.0}, imu_gy{0.0}, imu_gz{0.0};
    double odom_x{0.0}, odom_y{0.0}, odom_yaw{0.0};
    double odom_vx{0.0}, odom_vy{0.0}, odom_wz{0.0};
    double timestamp{0.0};
};

/// 电池状态
struct BatteryState {
    double voltage{12.0};
    double current{0.0};
    double percent{1.0};
    bool charging{false};
    double temperature{25.0};
    bool low_battery{false};
    bool critical_battery{false};

    void update_thresholds(double low = 0.20, double critical = 0.10) {
        low_battery = percent < low;
        critical_battery = percent < critical;
    }
};

/// 机器人健康状态
struct RobotHealthState {
    bool ok{true};
    std::string level{"OK"};
    std::vector<std::string> active_faults;
    double cpu_temp{45.0};
    bool imu_ready{true};
    bool lidar_ready{true};
    bool camera_ready{true};
    bool motion_ready{true};
};

using Config = std::unordered_map<std::string, std::string>;

/// 硬件抽象接口基类
class HardwareInterface {
public:
    HardwareInterface();
    explicit HardwareInterface(const Config& config);
    virtual ~HardwareInterface() = default;

    HardwareInterface(const HardwareInterface&) = delete;
    HardwareInterface& operator=(const HardwareInterface&) = delete;

    static std::unique_ptr<HardwareInterface> create(const Config& config = {});

    virtual bool initialize() = 0;
    virtual void shutdown() = 0;

    void send_velocity(double vx, double vy, double wz);
    virtual void dispatch_motion(double vx, double vy, double wz) = 0;
    bool enable_motors(bool enable = true);
    void emergency_stop(const std::string& reason = "manual");
    bool release_emergency_stop();

    virtual SensorReadings get_sensor_readings() = 0;
    virtual BatteryState get_battery() = 0;
    virtual RobotHealthState get_health() = 0;

    bool motors_enabled() const { return motors_enabled_; }
    bool emergency_stopped() const { return emergency_stopped_; }
    bool command_timeout() const;
    std::unordered_map<std::string, std::string> stats() const;
    static double limit_accel(double target, double current, double dt, double limit);
    double max_linear_x() const { return max_linear_x_; }

protected:
    void validate_config() const;

protected:
    Config config_;
    double max_linear_x_{0.3};
    double max_linear_y_{0.0};
    double max_angular_z_{1.2};
    double accel_limit_{2.0};
    double yaw_rate_limit_{4.0};
    double cmd_timeout_{1.0};
    double low_battery_threshold_{0.20};
    double critical_battery_threshold_{0.10};
    bool auto_recharge_enabled_{true};
    bool motors_enabled_{false};
    bool emergency_stopped_{false};
    double last_cmd_time_{0.0};
    std::tuple<double, double, double> last_velocity_{0.0, 0.0, 0.0};
    bool initialized_{false};
    long cmd_count_{0};
    long error_count_{0};
    std::string last_error_;
    mutable std::mutex mutex_;
};

/// 线程看门狗
class Watchdog {
public:
    using TimeoutCallback = std::function<void(const std::string&)>;
    explicit Watchdog(double timeout = 0.5);
    void register_thread(const std::string& name);
    void heartbeat(const std::string& name);
    std::vector<std::string> check();
    bool is_alive(const std::string& name) const;
    void add_timeout_callback(TimeoutCallback callback);
private:
    double timeout_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, double> heartbeats_;
    std::unordered_map<std::string, bool> dead_threads_;
    std::vector<TimeoutCallback> callbacks_;
};

}  // namespace puppypi_adapter
