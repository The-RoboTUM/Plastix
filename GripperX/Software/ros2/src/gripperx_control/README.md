# gripperx_control

Steering-servo driver, LiDAR power control, and the `ros2_control` configuration
for the drive chain.

> **The swerve kinematics are no longer here.** `SwerveController` in
> `gripperx_swerve_controller` replaced `swerve_cmd_node` + `joint_command_bridge`
> and the two controllers they fed. Both nodes and `launch/control.launch.py` are
> still on disk but **no launch file starts them**; `sim_steer_bridge` is likewise
> out of the active path. Do not wire new work to them.

## Active nodes

| Executable | Role |
|---|---|
| `steer_servo_node` | Drives the four Feetech steering servos on the Pi's own USB serial bus. Subscribes `/hw/joint_commands` (steering half), `/teleop/direct_steer` (timed override, `direct_timeout_sec` 0.5 s) and `/teleop/active_mode`; publishes the real steering measurement on `/hw/steer_states`. Config: `config/steer_servo.yaml`. |
| `lidar_power_node` | GPIO relay for LiDAR power. Services `/lidar/set_power` (`std_srvs/SetBool`) and `/lidar/power_cycle` (`std_srvs/Trigger`); publishes `/lidar/power_state` (`std_msgs/Bool`, latched). Starts ON. Config: `config/lidar_power.yaml`. |
| `hw_firmware_mock` | Stands in for the ESP32 on the bench. Runs on its declared defaults. |
| `steer_servo_calibrate` | Offline CLI tool, **not a ROS node**. Moves the servos by hand with torque off and prints a block to paste into `steer_servo.yaml`. |
| `teleop_joint_commands_node` | Direct joint-command teleop path. Config: `config/teleop_joint_commands.yaml`. |

## Configuration owned here but consumed elsewhere

- `config/ros2_controllers.yaml` — read by `controller_manager`; carries the
  `swerve_controller` parameters and the robot geometry (`a = 0.180`,
  `b = 0.110`, `wheel_radius = 0.070`).
- `config/swerve_controller.sim.yaml` — the twin's layer on top of it.

`config/swerve_cmd.yaml` and `config/joint_command_bridge*.yaml` belong to the
retired nodes and are kept only for provenance.

## Steering geometry

Steering travel is **asymmetric and per wheel**: −100/+35° on FL and BR, +100/−35°
on FR and BL. See `docs/STEERING_LIMITS.md`. Keyboard A/D drives all four from one
value with a counter-rotating pattern, so its usable envelope is the inward limit
(35°), not the outward one.

## Known defects

- **`REGULATOR_OFF_STALE_FEEDBACK` in every driving run of 2026-08-21 (open,
  P1, not diagnosed).** The drive feedback reads older than its own 0.2 s bound on
  a nominally 30 Hz link. It is reported as an observation, not a cause; a true
  standstill reports the same status legitimately, so the finding is specifically
  that it appears *while driving*. The status codes are documented in
  `gripperx_control_msgs/msg/WheelVelocityReport.msg`.
- **Steering mechanical play.** Present and characterised; explicitly out of scope
  by user decision. It limits the accuracy of any pose the steering is commanded
  into, so treat commanded steering angle as an intent, not a measurement — the
  measurement is `/hw/steer_states`.
