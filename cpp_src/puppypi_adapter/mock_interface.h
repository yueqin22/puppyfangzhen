// 仿真 Mock 硬件接口实现
//
// 提供纯 C++ 的仿真硬件接口，无需 CoppeliaSim 或实体硬件。
// 用于开发测试和算法验证，接口与实体机完全一致。
//
// 运动学模型（差速驱动）:
//   x' = x + v·cos(θ)·dt
//   y' = y + v·sin(θ)·dt
//   θ' = θ + ω·dt
//
// 对应 Python: puppypi_adapter/mock_interface.py
// 纯工具类（非 ROS2 节点），不包含 main()
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "puppypi_adapter/hardware_interface.h"

namespace puppypi_adapter {

/// 矩形障碍物 (xmin, ymin, xmax, ymax)
struct ObstacleBox {
    double xmin{0.0};
    double ymin{0.0};
    double xmax{0.0};
    double ymax{0.0};
};

/// 圆形动态障碍物
struct ObstacleCircle {
    double x{0.0};
    double y{0.0};
    double r{0.3};
};

/// Mock 硬件接口（对应 Python: MockHardwareInterface）
///
/// 模拟差速驱动机器人的运动学、传感器和电池。
/// 所有接口与实体机完全一致，用于无硬件环境开发。
class MockHardwareInterface : public HardwareInterface {
public:
    MockHardwareInterface();
    explicit MockHardwareInterface(const Config& config);
    ~MockHardwareInterface() override;

    MockHardwareInterface(const MockHardwareInterface&) = delete;
    MockHardwareInterface& operator=(const MockHardwareInterface&) = delete;

    // === HardwareInterface 实现 ===
    bool initialize() override;
    void shutdown() override;
    void dispatch_motion(double vx, double vy, double wz) override;
    SensorReadings get_sensor_readings() override;
    BatteryState get_battery() override;
    RobotHealthState get_health() override;

    // === 仿真环境配置 ===
    /// 设置仿真环境障碍物
    /// @param obstacles 静态障碍物列表
    /// @param dynamic_obstacles 动态障碍物列表
    void set_environment(const std::vector<ObstacleBox>& obstacles,
                         const std::vector<ObstacleCircle>& dynamic_obstacles = {});

    /// 获取当前位姿 (x, y, yaw)
    std::tuple<double, double, double> pose() const;

    /// 获取仿真时间
    double sim_time() const { return sim_time_; }

    // === 故障注入（用于测试）===
    void inject_fault(const std::string& fault);
    void clear_fault(const std::string& fault);

private:
    /// 仿真主循环（30Hz）
    void simLoop();

    /// 射线与 AABB 求交 (Slab 法)
    static bool rayAabb(double ox, double oy, double dx, double dy,
                        const ObstacleBox& box, double& t);

    /// 射线与圆形求交
    static bool rayCircle(double ox, double oy, double dx, double dy,
                          const ObstacleCircle& circle, double& t);

    /// 当前秒数（仿真时间轴）
    double nowSeconds() const;

private:
    // 传感器配置
    int lidar_beams_{72};
    double lidar_range_{8.0};
    double lidar_noise_{0.02};  // m
    double imu_noise_{0.01};     // rad/s
    double odom_noise_{0.005};   // m

    // 位姿
    double x_{0.0}, y_{0.0}, yaw_{0.0};
    // 速度
    double vx_{0.0}, vy_{0.0}, wz_{0.0};

    // 电池
    double battery_percent_{1.0};
    double battery_voltage_{12.0};
    double battery_drain_rate_{0.00002};  // 每次命令
    BatteryState battery_;

    // 仿真环境
    std::vector<ObstacleBox> obstacles_;
    std::vector<ObstacleCircle> dynamic_obstacles_;

    // 线程控制
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> sim_thread_;
    double sim_dt_{1.0 / 30.0};  // 30Hz 仿真步长

    // 心跳与错误统计
    double heartbeat_{0.0};
    int sim_error_count_{0};

    // 仿真时间
    double sim_time_{0.0};

    // 故障列表
    std::vector<std::string> faults_;
};

}  // namespace puppypi_adapter
