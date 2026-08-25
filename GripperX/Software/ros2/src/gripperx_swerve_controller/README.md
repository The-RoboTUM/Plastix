# gripperx_swerve_controller

4WIS4WID swerve kinematics as a **`ros2_control` controller** (`SwerveController`,
exported through pluginlib as `gripperx_swerve_controller/SwerveController`).

This is the active drive path on **both** the real robot and the twin. It replaced
`swerve_cmd_node` + `joint_command_bridge` and the two controllers they fed
(`steering_position_controller`, `wheel_velocity_controller`). Those nodes still
exist on disk in `gripperx_control`, but no launch file starts them — do not wire
new work to them.

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
| command interfaces | 4 steering position + 4 wheel velocity | joint order **FL, FR, BL, BR** throughout |

`intent_echo` is published from inside the `controller_manager` update loop, not
from a callback, so a wedged executor shows up to the watchdog as *divergence*
rather than as silence.

## Configuration

Parameters come from `gripperx_control/config/ros2_controllers.yaml`; the twin
layers `gripperx_control/config/swerve_controller.sim.yaml` on top. Geometry in
use: `a = 0.180`, `b = 0.110`, `wheel_radius = 0.070`. Pure rotation uses the
swerve spin pose, `atan2(a, b) = 58.57°` outward on every wheel.

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
- **The per-wheel velocity regulator is OFF by default** and must stay off unless
  a change is deliberate: with it on, a mechanically degraded wheel is silently
  driven harder instead of showing up as a velocity deviation. See
  `gripperx_control_msgs/msg/WheelVelocityReport.msg` for the incident that made
  this a requirement.
