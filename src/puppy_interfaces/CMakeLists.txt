cmake_minimum_required(VERSION 3.8)
project(puppy_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotMode.msg"
  "msg/RobotHealth.msg"
  "msg/BatteryStatus.msg"
  "msg/PatrolStatus.msg"
  "msg/SecurityEvent.msg"
  "msg/FallEvent.msg"
  "msg/DockStatus.msg"
  "msg/PlatformMotionState.msg"
  "srv/SetRobotMode.srv"
  "srv/ClearFault.srv"
  "srv/EnableMotors.srv"
  "srv/StartMapping.srv"
  "srv/StopMapping.srv"
  # P0-2: 移除冗余的 NavigateToPose.action 和 PatrolRoute.action
  # 导航统一使用 Nav2 标准 nav2_msgs/NavigateToPose
  # 巡逻由 mission_manager 用 Nav2 NavigateToPose 逐 waypoint 实现
  "action/Dock.action"
  "action/StandUp.action"
  "action/SitDown.action"
  "action/RecoverPosture.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_package()
