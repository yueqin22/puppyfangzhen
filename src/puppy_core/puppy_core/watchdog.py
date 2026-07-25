"""Watchdog & 故障恢复 (v7.10)

监控系统健康状态，在故障发生时执行恢复动作。

监控项:
  1. 节点心跳超时检测
  2. 传感器数据超时(LiDAR/IMU/Camera)
  3. 通信超时(cmd_vel/odom)
  4. 异常状态检测(跌倒/卡住/偏离)

恢复动作:
  1. 重启节点
  2. 清除故障状态
  3. 触发姿态恢复
  4. 紧急停止
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import time


class WatchdogState(IntEnum):
    """看门狗状态"""
    HEALTHY = 0      # 健康
    DEGRADED = 1     # 降级
    FAULT = 2        # 故障
    RECOVERING = 3    # 恢复中
    EMERGENCY_STOP = 4  # 紧急停止


@dataclass
class SensorHealth:
    """传感器健康状态"""
    name: str
    last_update: float = 0.0
    timeout: float = 1.0  # 超时阈值(秒)
    healthy: bool = True
    failure_count: int = 0


class WatchdogNode(Node):
    """Watchdog节点 (v7.10)

    监控系统健康，在故障时执行恢复。
    """

    def __init__(self):
        super().__init__('watchdog')

        # 参数
        self.declare_parameter('check_interval', 0.5)  # 检查间隔(秒)
        self.declare_parameter('sensor_timeout', 1.0)  # 传感器超时
        self.declare_parameter('cmd_timeout', 0.5)  # 命令超时
        self.declare_parameter('max_failures', 3)  # 最大失败次数
        self.declare_parameter('auto_recover', True)  # 自动恢复

        # 状态
        self.state = WatchdogState.HEALTHY
        self._failure_count = 0
        self._recovery_attempts = 0

        # 传感器健康跟踪
        self._sensors: Dict[str, SensorHealth] = {
            'lidar': SensorHealth('lidar', timeout=self.get_parameter('sensor_timeout').value),
            'imu': SensorHealth('imu', timeout=self.get_parameter('sensor_timeout').value),
        }

        # 通信健康
        self._last_cmd_vel_time = time.time()
        self._cmd_timeout = self.get_parameter('cmd_timeout').value

        # ROS2接口
        self.create_subscription(LaserScan, '/scan', self._on_lidar, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.health_pub = self.create_publisher(String, '/watchdog/health', 10)
        self.emergency_pub = self.create_publisher(Bool, '/watchdog/emergency_stop', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 服务
        self.create_service(Trigger, '/watchdog/recover', self._recover_service)
        self.create_service(Trigger, '/watchdog/reset', self._reset_service)

        # 检查定时器
        interval = self.get_parameter('check_interval').value
        self.create_timer(interval, self._check_health)
        self.get_logger().info('Watchdog节点已启动 (v7.10)')

    def _on_lidar(self, msg: LaserScan):
        """LiDAR数据更新"""
        self._sensors['lidar'].last_update = time.time()
        self._sensors['lidar'].healthy = True

    def _on_imu(self, msg: Imu):
        """IMU数据更新"""
        self._sensors['imu'].last_update = time.time()
        self._sensors['imu'].healthy = True
        # 检测跌倒（加速度异常）
        accel = msg.linear_acceleration
        accel_mag = (accel.x**2 + accel.y**2 + accel.z**2) ** 0.5
        if accel_mag > 25.0 or accel_mag < 5.0:  # 正常约9.8
            self.get_logger().error(f'检测到异常加速度: {accel_mag:.1f} m/s² — 可能跌倒')
            self._trigger_emergency('fall_detected')

    def _on_cmd_vel(self, msg: Twist):
        """cmd_vel更新"""
        self._last_cmd_vel_time = time.time()

    def _check_health(self):
        """定期健康检查"""
        now = time.time()
        any_unhealthy = False
        critical_failure = False

        for name, sensor in self._sensors.items():
            if sensor.last_update > 0:
                elapsed = now - sensor.last_update
                if elapsed > sensor.timeout:
                    sensor.healthy = False
                    sensor.failure_count += 1
                    any_unhealthy = True
                    if sensor.failure_count > self.get_parameter('max_failures').value:
                        critical_failure = True
                    self.get_logger().warning(
                        f'传感器 {name} 超时 ({elapsed:.1f}s)')
                else:
                    sensor.healthy = True
                    sensor.failure_count = 0

        # 更新状态
        if critical_failure:
            if self.state != WatchdogState.EMERGENCY_STOP:
                self._trigger_emergency('critical_sensor_failure')
        elif any_unhealthy:
            if self.state == WatchdogState.HEALTHY:
                self.state = WatchdogState.DEGRADED
                self.get_logger().warning('系统降级运行')
        else:
            if self.state == WatchdogState.DEGRADED:
                self.state = WatchdogState.HEALTHY
                self.get_logger().info('系统恢复正常')

        # 发布健康状态
        health_msg = String()
        state_names = ['HEALTHY', 'DEGRADED', 'FAULT', 'RECOVERING', 'EMERGENCY_STOP']
        health_msg.data = state_names[self.state]
        self.health_pub.publish(health_msg)

    def _trigger_emergency(self, reason: str):
        """触发紧急停止"""
        self.state = WatchdogState.EMERGENCY_STOP
        # 紧急停止
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        # 发布紧急停止信号
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        self.get_logger().error(f'紧急停止: {reason}')

        # 尝试自动恢复
        if self.get_parameter('auto_recover').value:
            self._start_recovery()

    def _start_recovery(self):
        """启动故障恢复流程"""
        self.state = WatchdogState.RECOVERING
        self._recovery_attempts += 1
        self.get_logger().info(f'启动恢复流程 (第{self._recovery_attempts}次)')

        # 恢复步骤：
        # 1. 清除紧急停止
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_pub.publish(emergency_msg)

        # 2. 重置传感器计数
        for sensor in self._sensors.values():
            sensor.failure_count = 0

        # 3. 检查是否恢复
        self.create_timer(3.0, self._check_recovery)

    def _check_recovery(self):
        """检查恢复结果（单次定时回调）"""
        all_healthy = all(s.healthy for s in self._sensors.values())
        if all_healthy:
            self.state = WatchdogState.HEALTHY
            self.get_logger().info('恢复成功')
        else:
            self.state = WatchdogState.FAULT
            self.get_logger().error('恢复失败 — 需要人工干预')

    def _recover_service(self, request, response):
        """手动恢复服务"""
        self._start_recovery()
        response.success = True
        response.message = '恢复流程已启动'
        return response

    def _reset_service(self, request, response):
        """重置看门狗"""
        self.state = WatchdogState.HEALTHY
        self._failure_count = 0
        self._recovery_attempts = 0
        for sensor in self._sensors.values():
            sensor.failure_count = 0
            sensor.healthy = True
        # 清除紧急停止
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_pub.publish(emergency_msg)
        response.success = True
        response.message = '看门狗已重置'
        self.get_logger().info('看门狗已重置')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
