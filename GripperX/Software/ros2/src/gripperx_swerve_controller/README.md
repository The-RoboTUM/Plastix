# gripperx_swerve_controller

4WIS4WID swerve kinematics as a **`ros2_control` controller** (`SwerveController`,
exported through pluginlib as `gripperx_swerve_controller/SwerveController`).

This is the active drive path on **both** the real robot and the twin. It replaced
`swerve_cmd_node` + `joint_command_bridge` and the two controllers they fed
(`steering_position_controller`, `wheel_velocity_controller`). Those nodes were deleted from
`gripperx_control` (a leftover config file remains); do not wire new work to them.

For: anyone who changes or debugs the drive and steering control. Status: 2026-11-17.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| subscribe | `/cmd_vel` | `geometry_msgs/Twist` |
| subscribe | `/teleop/direct_steer` | `std_msgs/Float64MultiArray` (timed override, arbitration point A2) |
| subscribe | `/teleop/active_mode` | `std_msgs/String` |
| subscribe | `/hw/wheel_feedback_valid` | `std_msgs/Int32MultiArray` (latched, encoder provenance) |
| publish | `/swerve_controller/intent_echo` | `gripperx_control_msgs/SwerveIntentEcho` |
| publish | `/swerve_controller/wheel_velocities` | `gripperx_control_msgs/WheelVelocityReport` |
| publish | `/swerve_controller/stall_state` | `gripperx_control_msgs/WheelStallState` (latched, edge-triggered) |
| publish | `/swerve_controller/drive_fault` | `gripperx_control_msgs/DriveFault` (latched, edge-triggered) |
| command interfaces | 4 steering position + 4 wheel velocity | joint order **FL, FR, BL, BR** throughout |

`intent_echo` is published from inside the `controller_manager` update loop, not
from a callback, so a wedged executor shows up to the watchdog as *divergence*
rather than as silence.

**Whole-drive fault** (user decision 2026-10-16, `drive_fault_monitor.hpp`): a
wheel driven without LIVE encoder provenance, or a persistent stall (escalated
HWR-30a — hardware rework requirement 30a, tier-1 drive-motor stall cut-off),
forces all four wheel commands to exactly 0.0 until the **stack restarts** — no
command, controller switch or controller re-load clears it.
`gripperx_control/drive_fault_nav_guard` cancels Nav2 goals while it is active;
the web teleop shows it as a standing alarm.

**Operator clear** (SR-18 amendment 2026-11-09): the `STALL_*` reasons
(`STALL_RETRIP`, `STALL_TRIP_COUNT`, `STALL_MULTI_WHEEL`, `STALL_LATCHED_HELD`)
can also be cleared through the service `/swerve_controller/clear_drive_fault`
(`gripperx_control_msgs/srv/ClearDriveFault`) or the web teleop's clear button;
`ENCODER_MISSING` and `FEEDBACK_LOST` stay restart-only. The clear is applied by
the update loop, removes the tmpfs marker first (if that fails, nothing is
cleared), releases the tier-1 latches and re-arms every detector. **It is a
motion release:** a drive command that is still held drives off at once.
Goals cancelled by the fault are not resumed.

## Configuration

Parameters come from `gripperx_control/config/ros2_controllers.yaml`; the twin
layers `gripperx_control/config/swerve_controller.sim.yaml` on top. The geometry parameters
(`a`, `b`, wheel radius) are generated from
`gripperx_geometry/config/geometry.yaml`: do not restate or edit them here or in the config.
Pure rotation uses the swerve spin pose, a steering angle derived from `a` and `b` (the
arctangent of their ratio), outward on every wheel.

## Launched by

`gripperx_bringup/launch/real_robot.launch.py` (real) and
`gripperx_gazebo/launch/spawn_robot.launch.py` (twin), both via the
`controller_manager` spawner. Never spawn it alongside
`steering_position_controller` / `wheel_velocity_controller` — they claim the same
interfaces.

## Known defects

- **~5.9 % drivetrain rotation shortfall (open).** A commanded rotation is
  delivered at roughly 94.1 %. A 360° command lands at ~335–345°. This consumes
  the Nav2 yaw tolerance on turns beyond ~90°, and it is why a spin test result of
  exactly 360° would indicate a *second* error cancelling this one rather than a
  correct machine. Cause not established.
- **The per-wheel velocity regulator.** The code default of `wheel_regulator_enabled` is off, but
  `gripperx_control/config/ros2_controllers.yaml` sets it to true, so the deployed machine runs
  with the regulator on (a user decision recorded against FR-14, functional requirement 14 — the
  per-wheel regulator; re-acceptance of its extended report contract is still pending). It is
  the only runtime-switchable regulator parameter (`ros2 param set /swerve_controller
  wheel_regulator_enabled false` returns to open loop). Keep in mind why it is observable: with
  it on, a mechanically degraded wheel is driven harder instead of showing up as a velocity
  deviation, so read `correction` and the regulator status in `WheelVelocityReport`
  (`gripperx_control_msgs`).

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_control_msgs gripperx_swerve_controller
colcon test --packages-select gripperx_swerve_controller && colcon test-result --verbose
```

C++ (GoogleTest) desk tests for the kinematics, steering limits, stall detector, drive fault and
its clear service, the regulator and the alignment gate; none moves hardware.
