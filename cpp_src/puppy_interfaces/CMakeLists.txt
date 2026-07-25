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
  "action/Dock.action"
  "action/StandUp.action"
  "action/SitDown.action"
  "action/RecoverPosture.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_package()
