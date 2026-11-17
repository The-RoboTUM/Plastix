# gripperx_control_msgs

Message and service definitions of the drive command chain: the swerve controller's intent
echo, the per-wheel velocity report, the stall state, the whole-drive fault and its operator
clear service. No nodes; interfaces only.

For: anyone who reads drive diagnostics or changes the controller. Status: 2026-11-17. The
`.msg`/`.srv` files carry long field comments with the reasoning and the requirement
references; read them for detail, this page is only the map.

Short forms: SR-18 (safety requirement 18 — whole-drive fault on a missing encoder or a
persistent stall), HWR-30a (hardware rework requirement 30a — per-motor stall cut-off, tier
1), FR-11 / FR-14 (functional requirements 11 and 14 — wheel velocity reporting and the
per-wheel regulator), SR-11 (safety requirement 11 — a watchdog input must not be the topic
it polices).

## Definitions

| File | Published or served on | By | Purpose |
|---|---|---|---|
| `msg/SwerveIntentEcho.msg` | `/swerve_controller/intent_echo` | `swerve_controller` (from its update loop) | The `/cmd_vel` the controller last consumed plus a message counter. The hardware interface watchdog compares it with `/cmd_vel` to tell silence from divergence (SR-11). |
| `msg/WheelVelocityReport.msg` | `/swerve_controller/wheel_velocities` | `swerve_controller` | Per wheel: commanded and measured velocity, stall state, regulator correction/status, steering alignment gate status, derived manoeuvre. Joint order FL, FR, BL, BR. |
| `msg/WheelStallState.msg` | `/swerve_controller/stall_state` (latched) | `swerve_controller` | Per-wheel tier-1 stall latch, armed flags, trip counts. Edge-triggered. |
| `msg/DriveFault.msg` | `/swerve_controller/drive_fault` (latched) | `swerve_controller` | Whole-drive fault (SR-18): active flag, reason code, `fault_id`, tripping wheel, evidence, and the operator-clear bookkeeping. |
| `srv/ClearDriveFault.srv` | `/swerve_controller/clear_drive_fault` | `swerve_controller` | Operator clear of a latched fault. Request: `requester`, `expected_reason`, `expected_fault_id`. Response: `success`, `reason`, `message`. |

Topic and service names are the controller's defaults, set in
`gripperx_control/config/ros2_controllers.yaml` (`intent_echo_topic`, `stall_state_topic`,
`drive_fault_topic`; the report and clear-service names are defaults in
`gripperx_swerve_controller/include/gripperx_swerve_controller/swerve_controller.hpp`).

Consumers (by package dependency): `gripperx_hardware_interface` (intent echo),
`gripperx_control` (`drive_fault_nav_guard`, which cancels Nav2 goals while a fault is
active), `gripperx_teleop` (web UI fault banner and clear button).

## Things worth knowing

- **Drive fault reasons** (`DriveFault.msg`): `REASON_NONE` 0, `ENCODER_MISSING` 1,
  `STALL_RETRIP` 2, `STALL_TRIP_COUNT` 3, `STALL_MULTI_WHEEL` 4, `STALL_LATCHED_HELD` 5,
  `FEEDBACK_LOST` 6. The STALL_* reasons mean "commanded, encoder not moving", which cannot
  be told apart from a silent encoder; they are not proof of a mechanical stall.
- **Clearing.** A restart of the stack clears every reason. The service clears only
  codes 2 to 5; `ENCODER_MISSING` and `FEEDBACK_LOST` are refused and need a restart. A clear
  is a motion release: a drive command that is still held makes the robot drive off at once.
  Passing `expected_fault_id` makes the service refuse a fault the caller did not review.
- **Enum mirrors.** The regulator-status and alignment-status constants in
  `WheelVelocityReport.msg` mirror enums in `gripperx_swerve_controller` and are
  `static_assert`ed against them; change both or neither.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_control_msgs
```

Interface-only package: no tests of its own. The controller's tests
(`colcon test --packages-select gripperx_swerve_controller`) exercise the messages. After a
definition change rebuild `gripperx_swerve_controller`, `gripperx_hardware_interface`,
`gripperx_control` and `gripperx_teleop`.
