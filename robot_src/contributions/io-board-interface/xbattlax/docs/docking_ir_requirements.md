# Docking, IR Homing, and Obstacle Camera Requirements

Status: draft requirements captured from maintainer feedback on July 18, 2026.

Source: [`makerspet/oomwoo-io-board#1`](https://github.com/makerspet/oomwoo-io-board/pull/1#issuecomment-5010609602).

## Maintainer-requested next work

The requested direction is:

1. Add an obstacle camera to the front of the robot URDF with about 130 degrees
   field of view.
2. Add a dock to Gazebo simulation with an IR emitter for homing.
3. Add two front IR sensors separated by a baffle for final approach homing.
4. Add one left and one right IR sensor to detect the dock's IR emitter when the
   dock location is unknown.
5. Implement docking in two stages:
   - Nav2 drives to a known pose just in front of the dock.
   - Final docking runs without Nav2, using the two front IR homing sensors.
6. Implement undocking.
7. Evaluate Nav2 docking server (`opennav_docking`) before deciding whether to
   use it or keep a smaller custom final-approach controller.

## Proposed sensor ownership

| Signal | Simulation owner | Hardware owner | ROS2 exposure |
|---|---|---|---|
| Front obstacle camera | URDF/Gazebo camera plugin | CPU camera interface | Standard image + camera info topics. |
| Dock IR emitter | Dock world/model plugin | Dock hardware | Not a robot MCU output; treated as environment signal. |
| Front-left IR homing sensor | Gazebo sensor/plugin | MCU ADC/digital input | Normalized IR intensity or future custom dock sensor message. |
| Front-right IR homing sensor | Gazebo sensor/plugin | MCU ADC/digital input | Normalized IR intensity or future custom dock sensor message. |
| Left dock-search IR sensor | Gazebo sensor/plugin | MCU ADC/digital input | Normalized IR intensity or future custom dock sensor message. |
| Right dock-search IR sensor | Gazebo sensor/plugin | MCU ADC/digital input | Normalized IR intensity or future custom dock sensor message. |
| Charge contacts | Gazebo contact/battery plugin | Power/charger circuit + MCU telemetry | Dock-present and charging-active flags. |

## Proposed ROS2 topics

Start with simple topics until the real sensor behavior is measured:

| Topic | Type | Meaning |
|---|---|---|
| `/camera/obstacle/image_raw` | `sensor_msgs/msg/Image` | Front obstacle camera image. |
| `/camera/obstacle/camera_info` | `sensor_msgs/msg/CameraInfo` | 130 degree front camera calibration. |
| `/oomwoo/dock_ir/front_left` | `std_msgs/msg/Float32` | Normalized IR beacon strength, 0.0-1.0. |
| `/oomwoo/dock_ir/front_right` | `std_msgs/msg/Float32` | Normalized IR beacon strength, 0.0-1.0. |
| `/oomwoo/dock_ir/search_left` | `std_msgs/msg/Float32` | Left-side dock beacon search strength, 0.0-1.0. |
| `/oomwoo/dock_ir/search_right` | `std_msgs/msg/Float32` | Right-side dock beacon search strength, 0.0-1.0. |
| `/oomwoo/dock/state` | `std_msgs/msg/String` initially | JSON state: dock visible, aligned, contact made, charging active. |

If the bridge later publishes a custom `oomwoo_msgs/DockIr` message, it should
include raw readings, filtered readings, thresholded booleans, saturation flags,
and the sensor layout revision.

## Docking state machine

| State | Controller | Exit condition |
|---|---|---|
| `navigate_to_predock` | Nav2 | Robot reaches a known pose in front of the dock. |
| `search_for_beacon` | Dock controller | Side/search IR sensors detect the dock emitter. |
| `final_ir_align` | Dock controller, no Nav2 | Front-left/front-right readings are balanced and increasing. |
| `contact_and_charge` | Dock controller + MCU/power telemetry | Charge contacts report docked and charging active. |
| `docked` | Job/dock-cycle module | Recharge/service complete or user command. |
| `undocking` | Dock controller, no Nav2 until clear | Robot backs out to a known clear pose, then hands back to Nav2. |

## CPU/MCU interface impact

- The MCU should stream dock IR readings or thresholded bits in telemetry.
- The MCU should report dock-present and charging-active separately.
- The CPU-side docking controller may command slow final-approach velocities, but
  the MCU must still stop on bumper, cliff, wheel-drop, e-stop, or heartbeat
  timeout.
- Final approach should use smaller max velocity and shorter command duration
  than normal `/cmd_vel` navigation.
- The simulated dock should publish enough data to test the exact ROS2 bridge
  contract before the real dock hardware exists.
