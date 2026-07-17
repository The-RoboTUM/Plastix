# GripperX

GripperX is an outdoor litter-collecting mobile robot. It combines a 4-wheel
independent steering/drive chassis (4WIS/4WID, a.k.a. swerve) with a robotic
arm for picking up litter, running [ROS 2 Jazzy](https://docs.ros.org/) on a
Raspberry Pi 5.

## Stack at a glance

- **Drive/steering:** 4x independent-steering, independent-drive wheel modules
  (swerve kinematics), with per-wheel motor control handled by an ESP32
  running micro-ROS firmware.
- **Arm:** LeRobot SO-ARM100-based gripper arm.
- **Compute:** Raspberry Pi 5 (ROS 2 Jazzy, Ubuntu) for high-level control,
  ESP32 (micro-ROS) for motor-level control.
- **Autonomy:** Gazebo simulation, localization (EKF, laser odometry, SLAM,
  AMCL), and Nav2 for path planning and navigation.

## Repository layout

| Path | Content |
|---|---|
| [`Software/ros2/`](Software/ros2/README.md) | ROS 2 Jazzy colcon workspace (packages, maps, scripts) — used on both the laptop (simulation) and the Pi (real robot) |
| [`Software/microros/`](Software/microros/README.md) | ESP32 micro-ROS firmware for motor control |
| [`Software/pi_env/`](Software/pi_env/README.md) | Reference snapshot of the Raspberry Pi's system environment (systemd units, udev rules, package lists) |
| [`documentation/`](documentation/README.md) | Electrical/wiring schematics |

See [`Software/README.md`](Software/README.md) for a short index of the
software folders, and the READMEs linked above for details on each.
