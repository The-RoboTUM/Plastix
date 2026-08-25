# gripperx_external

The robot's interface to the **Octopus**, an external litter-detection system that
supplies GPS goals over a rosbridge WebSocket. This package is the only place that
speaks the external wire format.

Two executables, both in the namespace `/gripperx/external`:

- **`octopus_link_node`** — transport only. Drains the WebSocket, converts JSON to
  typed messages and back. Makes no decisions.
- **`goal_gateway_node`** — all the judgement: validation, geodesy, grasp-pose
  resolution, the arming gate, preview, telemetry, and the gated dispatch to Nav2
  and the arm.

## Octopus wire topics

| Topic | Type | Direction |
|---|---|---|
| `/octopus/fake_eve_gps_start` | `sensor_msgs/NavSatFix` | in — the geodetic datum |
| `/octopus/trash_goal` | `sensor_msgs/NavSatFix` | in — exactly one current goal |
| `/octopus/trash_gps` | `std_msgs/String` (JSON) | in — full target set, preview only |
| `/octopus/flight_camera_transform/status` | `std_msgs/String` (JSON) | in — their transform health |
| `/octopus/trash_goal_done` | `std_msgs/String` | out — finished target id |
| `/octopus/devices/gripperx/status` | `std_msgs/String` (JSON) | out — robot telemetry |

## Local interface (relative to `/gripperx/external`)

Link → gateway: `datum`, `link_status`, and — only with `goal_ingress_enabled` —
`goal`, `targets`. Gateway → link: `telemetry`, `goal_done`.
Gateway publishes `status`, `arming_state`, `preview_markers` and `/diagnostics`;
it subscribes `/odometry/filtered`, `/global_costmap/costmap` and
`/teleop/active_mode`, and holds action clients for `/navigate_to_pose` and
`/pick_plastic`.

## The arming gate — read this before using the package

Authority is two-layer and **the external system can reach neither layer**.

1. **`set_arming`** (`gripperx_external_msgs/SetArming`) is the one and only way
   into the armed state. It is a plain ROS service on the robot's own domain and
   is **deliberately absent from the rosbridge `services_glob`** — if the Octopus
   could call it, the Octopus could arm the motion chain it is gated by. Arming
   always carries an explicit duration; there is no indefinite arm, and the window
   is measured on a **monotonic** clock. Reason with
   `ArmingState.seconds_remaining`, not with `expires_at`, which is only a
   projection.
2. **`teleop_mux`** must be in `autonomous` before Nav2 output reaches `/cmd_vel`
   at all. It stays operator-owned. The gateway *observes* `/teleop/active_mode`
   and never publishes it.

Ten disarm triggers are reported in `ArmingState.last_disarm_trigger`, including
link loss, clock stall and clock-jumped-backwards as separate codes.

## Running it

```bash
ros2 launch gripperx_external octopus_link.launch.py
```

Config: `config/octopus_link_real.yaml` (real) / `config/octopus_link_twin.yaml`
(twin). Defaults are deliberately conservative — `goal_ingress_enabled: false`,
`allow_arm: false`, `dry_run: true` — so a fresh bringup previews and dispatches
nothing.

The rclpy-free modules (`octopus_protocol`, `geodesy`, `grasp`, `validation`,
`arming`) are tested with plain `python3` and nothing running: see `test/check_*.py`.

## Known gaps

- Several safety-relevant parameters ship as `TO-VERIFY` rather than as guessed
  numbers: the geofence bounds, the grasp offset and tolerance, and
  `datum_jump_warn_m`. Goals do not resolve to a standing pose until the grasp
  offset is measured.
- **Battery telemetry is permanently unavailable** with reason
  `NO_SENSOR_INSTALLED` — there is no measurement chain on this robot. The field
  is reported honestly rather than as a plausible zero.
- Octopus target ids restart at 1 when their node restarts, so the local blacklist
  is only valid within one link session.
