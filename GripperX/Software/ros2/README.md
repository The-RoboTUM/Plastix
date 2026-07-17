# ros2

Standard ROS 2 Jazzy colcon workspace. Source packages live under `src/`;
building with `colcon` generates `build/`, `install/`, and `log/`, which are
not committed to the repository.

## Packages (`src/`)

| Package | Description |
|---|---|
| `gripperx_description` | Robot model: URDF/Xacro, meshes, RViz display configuration |
| `gripperx_control` | Swerve kinematics and `ros2_control` configuration |
| `gripperx_gazebo` | Gazebo Sim bringup (world, robot spawn, sensor bridge) |
| `gripperx_bringup` | Real-robot bringup launches (URDF + `ros2_control` + swerve, sensors, localization, autonomy) |
| `gripperx_localization` | State and pose estimation (EKF, laser odometry, SLAM, AMCL) |
| `gripperx_planning` | Nav2-based motion planning and navigation |
| `gripperx_sensors` | LiDAR, IMU, and GPS driver nodes (mock and real) |
| `gripperx_hardware_interface` | `ros2_control` hardware interface bridging `/hw/joint_commands` and `/hw/joint_states` |
| `gripperx_arm` | LeRobot SO-ARM100 action server for the gripper arm |
| `gripperx_arm_msgs` | Action interface definitions for the arm |
| `gripperx_teleop` | Teleop input multiplexer and keyboard teleop |
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
