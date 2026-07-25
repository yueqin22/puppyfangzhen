// PuppyPi 实体机硬件驱动实现
//
// 将 HardwareInterface 接口映射到 PuppyPi SDK 调用。
// 上层导航逻辑通过此驱动与实体硬件交互。
//
// 硬件架构:
//   - 主控: Raspberry Pi 4B (ARM Cortex-A72)
//   - LiDAR: RPLIDAR A1/A2 (UART)
//   - IMU: MPU6050 / BMI160 (I2C)
//   - 电机驱动: PCA9685 PWM + 直流减速电机
//   - 电池: 3S LiPo (11.1V nominal)
//   - 相机: USB Camera (可选)
//
// 线程模型:
//   - 主线程: 导航算法 (通过接口调用)
//   - 传感器线程: 后台读取 LiDAR/IMU (30Hz)
//   - 电机线程: 运动控制环 (30Hz)
//   - 监控线程: 电池/温度监控 (1Hz)
//   - 急停线程: GPIO 急停按钮轮询 (100Hz)
//
// 注意：当前 C++ 版本未集成实际 SDK（与 Python 版一致，使用模拟模式回退）。
//       硬件依赖（rplidar, smbus2, RPi.GPIO）通过条件编译宏在 Linux 平台上启用。
//
// 对应 Python: puppypi_adapter/puppypi_driver.py
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

/// PuppyPi 实体机硬件接口（对应 Python: PuppyPiHardwareInterface）
///
/// 实现 HardwareInterface 接口，调用 PuppyPi SDK 与实体硬件交互。
/// 包含运动控制环、传感器数据流、电池状态读取。
class PuppyPiHardwareInterface : public HardwareInterface {
public:
    PuppyPiHardwareInterface();
    explicit PuppyPiHardwareInterface(const Config& config);
    ~PuppyPiHardwareInterface() override;

    PuppyPiHardwareInterface(const PuppyPiHardwareInterface&) = delete;
    PuppyPiHardwareInterface& operator=(const PuppyPiHardwareInterface&) = delete;

    // === HardwareInterface 实现 ===
    bool initialize() override;
    void shutdown() override;
    void dispatch_motion(double vx, double vy, double wz) override;
    SensorReadings get_sensor_readings() override;
    BatteryState get_battery() override;
    RobotHealthState get_health() override;

    /// 硬件急停按钮是否触发
    bool hw_estop_triggered() const { return hw_estop_triggered_; }

private:
    // === 硬件初始化子方法（每个失败时回退到模拟模式）===
    void initSdk();
    void initLidar();
    void initImu();
    void initAdc();
    void initGpio();

    // === 后台线程 ===
    void sensorLoop();     // 30Hz 传感器读取
    void motorLoop();      // 30Hz 运动控制环
    void monitorLoop();    // 1Hz 电池/CPU温度监控
    void estopPollLoop();  // 100Hz 急停按钮轮询

    // === 硬件读取子方法 ===
    /// 读取 LiDAR 扫描数据
    /// @return true 读取成功，结果存入缓存
    bool readLidarScan();
    /// 读取 IMU 数据
    /// @return true 读取成功，结果存入缓存
    bool readImu();
    /// 读取电池电压（通过 ADC 或模拟模式）
    void readBattery();
    /// 读取 CPU 温度（树莓派 /sys/class/thermal/...）
    void readCpuTemp();

    /// 16 位二补码转换
    static int16_t twosComplement(uint16_t val);

    /// 当前秒数
    static double nowSeconds();

private:
    // 传感器配置
    std::string lidar_port_{"/dev/ttyUSB0"};
    int lidar_baud_{115200};
    int lidar_beams_{720};
    double lidar_range_{12.0};
    int imu_i2c_addr_{0x68};
    int imu_rate_{100};

    // 电池配置
    int battery_adc_channel_{0};
    double battery_adc_vref_{3.3};
    int battery_cells_{3};
    double battery_full_voltage_{12.6};
    double battery_empty_voltage_{9.9};
    double battery_nominal_{11.1};

    // GPIO 配置
    int estop_pin_{18};
    int led_pin_{24};

    // 硬件句柄（实际 SDK 句柄未在头文件暴露，使用 void* 占位）
    // 在 C++ 版本中，硬件 SDK 集成前这些都保持 nullptr
    void* sdk_handle_{nullptr};   // PuppyPi SDK
    void* lidar_handle_{nullptr};  // RPLidar
    void* imu_handle_{nullptr};    // I2C SMBus
    void* adc_handle_{nullptr};    // ADS1115
    void* gpio_handle_{nullptr};   // RPi.GPIO

    // 传感器数据缓存
    std::vector<double> cached_lidar_angles_;
    std::vector<double> cached_lidar_ranges_;
    bool has_cached_lidar_{false};
    double cached_imu_accel_[3]{0.0, 0.0, 9.81};
    double cached_imu_gyro_[3]{0.0, 0.0, 0.0};
    double cached_odom_x_{0.0};
    double cached_odom_y_{0.0};
    double cached_odom_yaw_{0.0};
    double actual_velocity_[3]{0.0, 0.0, 0.0};

    // 线程控制
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> sensor_thread_;
    std::unique_ptr<std::thread> motor_thread_;
    std::unique_ptr<std::thread> monitor_thread_;
    std::unique_ptr<std::thread> estop_thread_;

    // 运动控制环状态
    double target_velocity_[3]{0.0, 0.0, 0.0};

    // 电池缓存与 CPU 温度
    BatteryState battery_;
    double cpu_temp_{45.0};

    // 急停按钮状态
    std::atomic<bool> hw_estop_triggered_{false};
};

}  // namespace puppypi_adapter
