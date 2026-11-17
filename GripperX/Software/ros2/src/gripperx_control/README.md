# gripperx_control

Steering-servo driver, LiDAR power control, the whole-drive-fault Nav2 guard and the
`ros2_control` configuration for the drive chain.

For: anyone who runs or changes the drive and steering path on the robot. Status: 2026-11-17.

Short forms: SR-18 (safety requirement 18 — whole-drive fault on a missing wheel encoder or a
persistent stall), SR-1
(safety requirement 1 — no motion without explicit approval per test).

> **The swerve kinematics are not in this package.** `SwerveController` in
> `gripperx_swerve_controller` replaced the former command-node chain. The old nodes and
> `launch/control.launch.py` are gone from the tree; `config/swerve_cmd.yaml` is a leftover kept
> only for provenance. Do not wire new work to it. Likewise
> `src/gripperx_control/swerve_controller.py` (a Python class for a paper's non-linear kinematic
> tracking controller) is unused legacy: nothing in the workspace imports it and it is not an
> executable.

## Executables

| Executable | Role |
|---|---|
| `steer_servo_node` | Drives the four Feetech steering servos on the Pi's own USB serial bus. Subscribes `/hw/joint_commands` (steering half), `/teleop/direct_steer` (timed override, `direct_timeout_sec`) and `/teleop/active_mode`; publishes the measured steering on `/hw/steer_states`. Config: `config/steer_servo.yaml`. |
| `lidar_power_node` | GPIO relay for LiDAR power. Services `/lidar/set_power` (`std_srvs/SetBool`) and `/lidar/power_cycle` (`std_srvs/Trigger`); publishes `/lidar/power_state` (`std_msgs/Bool`, latched). Starts on. Config: `config/lidar_power.yaml`. |
| `drive_fault_nav_guard` | Cancels every Nav2 goal (`action_names` parameter, default `/navigate_to_pose`, `/navigate_through_poses`, `/follow_path`, `/backup`, `/drive_on_heading`, `/crab_walk`) while `/swerve_controller/drive_fault` is active, so the fault reads as "goal cancelled" instead of a long failing recovery. It never publishes a velocity, and after an operator clear it only stops cancelling: it never resumes a goal. |
| `hw_firmware_mock` | Stands in for the ESP32 on the bench (including simulated encoders). Desk only; it refuses to publish when another `/hw/joint_states` writer exists. |
| `teleop_joint_commands_node` | Direct joint-command teleop path to `/hw/joint_commands` (`teleop_real.launch.py`). Config: `config/teleop_joint_commands.yaml`. |
| `steer_servo_calibrate` | Offline CLI tool, **not a ROS node**. Moves the servos by hand with torque off and prints a block to paste into `steer_servo.yaml`. |

Python modules shared with other packages: `swerve_kinematic_model` and `steering_limits` (used by
`gripperx_teleop` for its manoeuvre guard), `sts_servo_bus`.

## Launch

`launch/teleop.launch.py` starts `teleop_joint_commands_node` and `teleop_twist_keyboard` for a
bench test. The real stack is started from `gripperx_bringup` (`real_robot.launch.py`), which also
starts `steer_servo_node`, `lidar_power_node`, `drive_fault_nav_guard` (with respawn) and
`ros2_control`.

## Configuration owned here but consumed elsewhere

| File | Holds |
|---|---|
| `config/ros2_controllers.yaml` | read by `controller_manager`; carries the `swerve_controller` parameters (stall detection, whole-drive fault, regulator, alignment gate, topic names) and the geometry parameters, which are **generated** from `gripperx_geometry/config/geometry.yaml` (do not edit them here) |
| `config/swerve_controller.sim.yaml` | the twin's layer on top of it |
| `config/steer_servo.yaml` | per-wheel steering centres, limits and signs; produced by `steer_servo_calibrate` |

## Steering geometry

Steering travel is asymmetric and per wheel (outward and inward limits differ, mirrored between
the left and right side); the numbers live in `config/steer_servo.yaml` and are explained in
`docs/STEERING_LIMITS.md` (read its status note first: its sections (a)-(c) describe a retired
symmetric model). Keyboard A/D drives all four wheels from one value with a counter-rotating
pattern, so its usable envelope is the inward limit, not the outward one. Steering mechanical play
is present and characterised; treat a commanded steering angle as an intent, not a measurement.
The measurement is `/hw/steer_states`.

## Whole-drive fault (SR-18) in short

`swerve_controller` latches `/swerve_controller/drive_fault` and holds all four wheels at zero
when a wheel is driven without a confirmed encoder, a stall persists, or the encoder feedback
link is lost (`FEEDBACK_LOST`). The fault is also kept in a marker file under `/run/gripperx/`,
which the controller re-adopts on every load. That marker is cleared only by a deliberate stack
start (`systemctl start`/`restart`, the Mode R shutdown-button restart, `gripperx-stack-stop
--mode=restart`); systemd's automatic `Restart=on-failure` or a `ros2_control_node` respawn keeps
it, so the drive stays stopped (`documentation/DEPLOYMENT.md` §7.1). STALL_* reasons can also be cleared by an operator through
`/swerve_controller/clear_drive_fault`. Message and service definitions:
`gripperx_control_msgs/README.md`. Thresholds and windows: the `drive_fault_*`, `stall_*` and
`encoder_confirm_*` keys in `config/ros2_controllers.yaml`. A runtime `ros2 param set` of the
stall parameters is ignored (observed on the robot); `wheel_regulator_enabled` is the
documented runtime switch.

## Known defects

- **`REGULATOR_OFF_STALE_FEEDBACK` while driving** was recorded 2026-08-21 (drive feedback older
  than its own bound on a nominally 30 Hz link). Whether it still occurs is TO-VERIFY (not
  re-measured for this page).
- **Steering mechanical play** is present and out of scope by user decision.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_control_msgs gripperx_control
colcon test --packages-select gripperx_control && colcon test-result --verbose
```

The tests are desk tests (`test_drive_fault_nav_guard.py`, `test_hw_firmware_mock.py`, the
`check_*.py` scripts); none moves hardware. Restarting the bringup counts as motion (SR-1).
