# gripperx_bringup

Launch files for the real robot: the motion stack (`ros2_control`, swerve controller, steering,
arm, LiDAR, IMU, teleop mux), the full autonomy stack on top of it, a bench teleop launch and a
legacy mapping launch.

For: anyone who starts or changes the robot's software stack. Status: 2026-11-17. No nodes of its
own; every launch here only wires other packages together.

Short forms: SR-1 (safety requirement 1 — no motion without explicit approval per test; a bringup
start or restart counts as motion), SR-14 (safety requirement 14 — controller activation must not
move an actuator), IMU (inertial sensor, BNO085), EKF (extended Kalman filter), SLAM (simultaneous
localization and mapping), twin (the Gazebo simulation).

**Starting or restarting the real stack makes the drive path commandable and leaves the steering
limp at an unknown angle: it needs an explicit approval for that test (SR-1).** On the robot the
stack is run by systemd (`documentation/DEPLOYMENT.md`, script
`Software/pi_env/systemd/scripts/gripperx-bringup.sh`, which runs
`real_robot.launch.py use_mock_firmware:=false use_lidar:=true arm_home_on_startup:=false`).

## Launch files

| Launch | What it starts |
|---|---|
| `real_robot.launch.py` | The motion stack. See below. This is what the systemd bringup runs. |
| `bringup.launch.py` | Backward-compatible alias for `real_robot.launch.py`. |
| `real_autonomy.launch.py` | `real_robot` (always with the real LD06 LiDAR, camera off) plus `gripperx_sensors/sensors.launch.py`, `gripperx_localization/localization.launch.py` and optionally `gripperx_planning/navigation.launch.py`. |
| `teleop_real.launch.py` | Bench teleop: `teleop_joint_commands_node` (`/cmd_vel` to `/hw/joint_commands`), `steer_servo_node`, optionally `arm_action_server` and a local keyboard or joystick teleop. The micro-ROS agent is **not** started here: it is the `gripperx-agent` systemd service. |
| `mapping.launch.py` | **Legacy.** `rf2o_laser_odometry` plus `slam_toolbox` (`config/slam_toolbox.yaml`) and no EKF. The `gripperx-mapping` service no longer uses it: it runs `gripperx_localization/launch/localization.launch.py`. Do not use it for Nav2 (nothing would publish `/odometry/filtered`). |

### What `real_robot.launch.py` starts

| Node | Package | Note |
|---|---|---|
| `arm_action_server`, `arm_joint_state_relay` | `gripperx_arm` | arm node respawns; loads `arm_poses.yaml`, `arm_safety_filter.yaml` and the shared `tf_freshness.yaml` |
| `LD06`, `scan_range_filter` | `ldlidar_stl_ros2`, `gripperx_sensors` | driver publishes `/scan_raw`, the filter publishes `/scan`; both respawn; only with `use_lidar` |
| `lidar_power_node` | `gripperx_control` | LiDAR power relay; only with `use_lidar` |
| `bno085_imu` | `gripperx_sensors` | only with `use_imu` |
| `steer_servo_node` | `gripperx_control` | steering servos |
| `teleop_mux` | `gripperx_teleop` | default mode `keyboard` |
| `robot_state_publisher` (via `gripperx_description/launch/load_urdf.launch.py`) | | URDF from `gripperx_v1.urdf.xacro` |
| `ros2_control_node` | `controller_manager` | hardware plugin `gripperx_hardware_interface/GripperXInterface` through the URDF; config `gripperx_control/config/ros2_controllers.yaml` |
| spawners | | `joint_state_broadcaster` after the control node starts, then `swerve_controller` after the broadcaster spawner exits |
| `drive_fault_nav_guard` | `gripperx_control` | cancels Nav2 goals while the whole-drive fault is active; respawns |
| `hw_firmware_mock` | `gripperx_control` | only with `use_mock_firmware:=true` (desk only) |
| RViz | `gripperx_description` | only with `use_rviz` |

The micro-ROS agent (link to the ESP32) is the separate `gripperx-agent` systemd service.

| Launch argument (`real_robot.launch.py`) | Default | Meaning |
|---|---|---|
| `use_sim_time` | `false` | Keep false on hardware. |
| `use_mock_firmware` | `false` | Run `hw_firmware_mock` instead of the ESP32 (desk only). Defaults to false so a bare manual launch can never start the mock next to the real ESP32. |
| `use_lidar` | `true` | LD06 driver, filter, power node and the URDF LiDAR. |
| `lidar_stamp_source` | `mid_sweep` | How the LD06 stamps `/scan`: `mid_sweep`, `first_ray` or `publish` (old loop-tick stamp). In the first two, a scan with bad per-point stamps is dropped, not re-stamped. |
| `use_camera` | `false` | Camera link in the URDF only; the camera driver is the separate `gripperx-camera` service. |
| `use_imu` | `true` | Start the BNO085 driver; false on a bench without the sensor. |
| `arm_home_on_startup` | `false` | Permanently false by user decision: the arm node respawns, so true would mean unattended arm motion. This argument overrides the YAML value. |
| `use_rviz` | `false` | |

`real_autonomy.launch.py` arguments: `use_sim_time`, `use_mock_firmware`, `enable_sensors`,
`enable_localization`, `enable_navigation` (default false), `enable_laser_odometry` (default false),
`fuse_laser_odometry` (default false), `enable_gps`, `enable_slam`, `enable_saved_map_localization`,
`map_yaml_file`, `use_rviz`. `use_mock_sensors:=true` is refused at launch: the real LiDAR always
runs, and a mock `/scan` on the same topic and frame would be indistinguishable from it.

## Examples

```bash
# Motion stack only (what systemd runs)
ros2 launch gripperx_bringup real_robot.launch.py use_mock_firmware:=false

# Full stack with SLAM and Nav2
ros2 launch gripperx_bringup real_autonomy.launch.py \
  enable_slam:=true enable_saved_map_localization:=false enable_navigation:=true
```

Simulation: `ros2 launch gripperx_gazebo simulation.launch.py` (see `gripperx_gazebo`).

## Configuration

`config/slam_toolbox.yaml` is used only by the legacy `mapping.launch.py`. Everything else is read
from the launched packages' own config directories (`gripperx_control`, `gripperx_arm`,
`gripperx_localization`, `gripperx_planning`); this package holds no tuning values.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_bringup
```

The package only installs launch files and config (`CMakeLists.txt` runs the lint hooks, no
tests). The wiring between the autostart, localization and Nav2 is checked by
`gripperx_localization/test/check_autonomy_launch_wiring.py`. After a deploy, verify the install timestamps on the robot: the
systemd services run the installed copy, not the git checkout.
