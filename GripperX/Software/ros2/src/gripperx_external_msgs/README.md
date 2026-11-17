# gripperx_external_msgs

Typed interfaces of the external-goal link (the Octopus litter-detection system, connected
over a rosbridge WebSocket). They are internal to GripperX: the wire format on the link itself
is JSON, spoken only by `octopus_link_node` in `gripperx_external`. No nodes; interfaces only.

For: anyone who reads the gateway's status or telemetry, or changes the link. Status:
2026-11-17. The definitions carry long comments with the reasoning and requirement references;
this page is the map.

Short forms: SR-1 (safety requirement 1 — no motion without explicit approval), SR-15
(safety requirement 15 — external command authority, the arming gate), FR-12 (functional
requirement 12 — external litter goals from the Octopus and telemetry outward), Octopus
(the external detection/dashboard system), datum (origin of the shared flat-earth conversion).

## Definitions

All topics below are relative to the namespace `/gripperx/external`.

| File | Topic / service | Direction | Purpose |
|---|---|---|---|
| `msg/GeodeticDatum.msg` | `datum` (latched) | link to gateway | Origin of the flat-earth conversion shared with the Octopus; flags for fallback and bootstrap values. Dispatch on a bootstrap datum is refused. |
| `msg/ExternalLinkStatus.msg` | `link_status` (latched) | link to gateway | Link health: connected, reconnects, message age, open-target count, datum validity, and the Octopus frame-transform status (`frame_*` fields). |
| `msg/ExternalGoal.msg` | `goal` | link to gateway | One goal as received (lat/lon, confidence, datum) plus its resolution into a map position and a robot standing pose. Published only when goal ingress is enabled. |
| `msg/ExternalTargetList.msg` | `targets` (latched) | link to gateway | The whole target set as one message, for the preview. Not for dispatch. |
| `msg/ExternalTarget.msg` | (element of the list) | | One entry: id, lat/lon, source metres, confidence, collected, is_goal, last_seen. |
| `msg/ExternalGoalStatus.msg` | `goal_status` | gateway out | Lifecycle of one goal (`STATE_RECEIVED` to `STATE_DONE`, `STATE_REJECTED`, `STATE_FAILED`, `STATE_BLACKLISTED`, ...) with machine-readable `reason`, attempts and `acknowledged`. |
| `msg/ArmingState.msg` | `arming_state` (latched) | gateway out | Layer 1 of the two-layer authority gate: armed flag, `seconds_remaining` (monotonic; reason with this, not with `expires_at`), the last disarm trigger (codes 0 to 10) and whether auto-pick is available. |
| `msg/RobotTelemetry.msg` | `telemetry` | gateway to link | Typed telemetry: map and line-frame pose, WGS84 position, odometry, navigation and arming state, teleop mode, link health, goal counters, blacklist, battery. Every block carries its own `*_valid` flag and reason. |
| `srv/SetArming.srv` | `set_arming` | operator to gateway | Arm for an explicit duration, or disarm. Request: `arm`, `duration_sec`, `requested_by`. Response: `success`, `message`, `state`. |

`set_arming` is a plain local ROS service and is deliberately absent from the rosbridge
services list, so the external system cannot arm the motion chain (SR-1, SR-15).

Used by: `gripperx_external` (`octopus_link_node`, `goal_gateway_node`, tests),
`gripperx_arm` (reads `ArmingState` for leader mode), `gripperx_teleop` (arming client and
status display), and the web UI.

## Things worth knowing

- **Disarm triggers** (`ArmingState.last_disarm_trigger`): `NONE` 0, `OPERATOR` 1, `TIMEOUT` 2,
  `LINK_LOST` 3, `MODE_CHANGE` 4 (includes the Space E-stop), `NAV2_UNAVAILABLE` 5,
  `NODE_SHUTDOWN` 6, `EXCESSIVE_ABORTS` 7, `CLOCK_STALLED` 8, `CLOCK_JUMPED_BACK` 9,
  `PICK_CONFIRM` 10 (the gateway disarms itself before an aimed pick so the operator's next
  arming approves exactly that pick; not an error).
- **Battery** in `RobotTelemetry` is reported as unavailable with reason
  `NO_SENSOR_INSTALLED` (no measurement chain on the robot).
- **Octopus ids** restart at 1 when their node restarts. `blacklisted_ids` survives a link
  reconnect and is dropped only on evidence of an Octopus id-space reset.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_external_msgs
```

Interface-only package: no tests of its own; the checks in `gripperx_external/test/` use the
messages. After a definition change rebuild `gripperx_external`, `gripperx_arm`,
`gripperx_teleop` and, on the robot, restart the link nodes.
