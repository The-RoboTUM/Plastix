# ros2

Standard ROS 2 Jazzy colcon workspace. Source packages live under `src/`;
building with `colcon` generates `build/`, `install/`, and `log/`, which are
not committed to the repository.

## Packages (`src/`)

### Robot model and drive chain

| Package | Description |
|---|---|
| `gripperx_geometry` | **The single source of truth for the robot's geometry.** Wheel base, track, radii and contact offsets are declared once in `config/geometry.yaml` and consumed by dozens of sites across parameter YAMLs, xacro, Python defaults, C++ initialisers and test constants. **Never edit a geometric value anywhere else** — edit the yaml, run `python3 -m gripperx_geometry.generate --sync-constants`, then `colcon test --packages-select gripperx_geometry`, which names every site that drifted. |
| `gripperx_description` | Robot model: URDF/Xacro, meshes, RViz display configuration |
| `gripperx_swerve_controller` | **The active drive path.** 4WIS4WID swerve kinematics as a `ros2_control` controller: consumes `/cmd_vel`, enforces the per-wheel steering windows, and writes the 4 steering-position + 4 wheel-velocity command interfaces. Also holds stall detection and the per-wheel velocity regulator. |
| `gripperx_control` | Steering servo driver (`steer_servo_node`) and its calibration tool, LiDAR power relay (`lidar_power_node`), firmware mock, and the `ros2_control` configuration (`ros2_controllers.yaml`, `steer_servo.yaml`) that the controller above reads. Also still contains the superseded `swerve_cmd_node` / `joint_command_bridge` pair, which no launch file starts. |
| `gripperx_hardware_interface` | `ros2_control` hardware interface bridging `/hw/joint_commands` and `/hw/joint_states` to the ESP32, plus the command watchdog |
| `gripperx_control_msgs` | Swerve and per-wheel telemetry messages (`SwerveIntentEcho`, `WheelStallState`, `WheelVelocityReport`) |

### Autonomy and perception

| Package | Description |
|---|---|
| `gripperx_localization` | State and pose estimation (EKF, wheel odometry, laser odometry, SLAM, AMCL) and an odometry divergence monitor |
| `gripperx_planning` | Nav2 configuration: planner, controller, behaviour trees, costmaps. Data only — it installs no executables. |
| `gripperx_behaviors` | Nav2 recovery plugin: a bounded pure-lateral "crab walk" escape |
| `gripperx_sensors` | LiDAR/IMU/GPS mock publishers, and `scan_range_filter`, which republishes `/scan_raw` as `/scan` with the robot's self-returns removed |

### External interface

| Package | Description |
|---|---|
| `gripperx_external` | Link to the external Octopus system over rosbridge: goal ingress, validation, geodesy, the arming gate (`SetArming`) and robot telemetry |
| `gripperx_external_msgs` | Message and service definitions for that link |

### Arm, teleop, bringup, tooling

| Package | Description |
|---|---|
| `gripperx_arm` | LeRobot SO-ARM100 action server for the gripper arm |
| `gripperx_arm_msgs` | Action interface definitions for the arm |
| `gripperx_teleop` | Teleop input multiplexer and keyboard teleop |
| `gripperx_bringup` | Real-robot bringup launches (URDF + `ros2_control` + swerve, sensors, localization, autonomy) |
| `gripperx_gazebo` | Gazebo Sim bringup (worlds, robot spawn, sensor bridge) |
| `gripperx_debug` | Logging and offline plotting tools for debugging |

## Vendored third-party packages

The following upstream projects are vendored in-tree under `src/`:

| Package | Purpose |
|---|---|
| `csm` | Canonical Scan Matcher — 2D laser scan matching library (ROS wrapper of Andrea Censi's CSM) |
| `ldlidar_stl_ros2` | LDRobot LiDAR ROS 2 driver |
| `rf2o_laser_odometry` | 2D odometry estimation from planar laser scans |
| `ros2_laser_scan_matcher` | Laser-scan-based odometry, built on `csm` |

## Other folders

- `maps/` — saved occupancy grid maps used for localization.
- `scripts/` — dependency installation and workspace utility scripts.
- `tools/` — standalone diagnostic scripts for the real robot, run with `python3` rather than
  `ros2 run`. **Eight of the ten command the drive, including the ones that expect no visible
  motion**, so they are subject to the project's motion-approval rule: no movement of drivetrain,
  steering or arm without an explicit user approval, per test. `tools/README.md` states per script
  what it commands and which open point it belongs to — read it before running anything there.
