# gripperx_external

The robot's interface to the **Octopus**, an external litter-detection system that supplies GPS
goals over a rosbridge WebSocket. This package is the only place that speaks the external wire
format. Its interfaces are in `gripperx_external_msgs`.

For: operators and developers of the external-goal path. Status: 2026-11-17. What has been
accepted on the robot is recorded in the requirements document, not here.

Short forms: SR-1 (safety requirement 1 — no motion without explicit approval per test), SR-15
(safety requirement 15 — external command authority, the arming gate), SR-16 (safety
requirement 16 — scoped exception to SR-1 for the automatic pick while armed), FR-12 (functional
requirement 12 — external litter goals and telemetry), FR-16 (adaptive grasp, "aimed pick").

Three executables, all in the namespace `/gripperx/external`:

- **`octopus_link_node`** — transport only. Drains the WebSocket, converts JSON to typed messages
  and back. Makes no decisions.
- **`goal_gateway_node`** — all the judgement: validation, geodesy, grasp-pose resolution,
  correlation of the goal with the Octopus' target list, the arming gate, preview, telemetry, and
  the gated dispatch to Nav2 and the arm (`/navigate_to_pose`, `/pick_plastic`).
- **`line_calibration_node`** — the shared reference line with the Octopus: the operator clicks
  frame post A, then post B, in RViz ("Publish Point", fixed frame `map`); it publishes
  `map -> octopus_line`, a latched status and markers, and logs the measured length L to enter on
  the drone side. **The gateway refuses every goal until this is done, at every start and after
  every SLAM restart.** It is **not** started by `octopus_link.launch.py` but by
  `gripperx_localization/launch/localization.launch.py` (`enable_line_calibration:=true` by
  default), so it lives and dies with slam_toolbox; it loads this package's
  `octopus_link_<env>.yaml`. The geofence is derived from the calibration: the square of side L
  around the line midpoint. Convention: `documentation/OCTOPUS_LINE_CALIBRATION.md`.
Pure modules (no ROS): `octopus_protocol`, `geodesy`, `line_frame`, `grasp`, `validation`,
  `arming`, `correlation`, `clock_rate`, `domain_guard`, `diagnostics`, `rosbridge_client`.

## Launch

```bash
ros2 launch gripperx_external octopus_link.launch.py [env:=twin|real] [goal_ingress:=true] \
    [dry_run:=false] [url:=ws://...] [use_sim_time:=...] [rviz:=true]
```

| Argument | Default | Meaning |
|---|---|---|
| `env` | `twin` | Which config to load: `config/octopus_link_twin.yaml` or `config/octopus_link_real.yaml`. The twin needs `ROS_DOMAIN_ID=220`, the real robot `20`; a mismatch is fatal at startup. |
| `goal_ingress` | `false` | `false`: the external goal topics are not subscribed at all. `true`: goals are validated and previewed, and can be dispatched only after an explicit arming. The launch default is `false`, but the robot's systemd script passes `goal_ingress:=true`. |
| `dry_run` | `true` | A second, independent block on dispatch. Setting it false arms nothing. The launch default is `true`, but the robot's systemd script passes `dry_run:=false`: on the robot only the arming gate (and the teleop mux) remain as blocks. |
| `url` | from config | WebSocket address of the Octopus. |
| `use_sim_time` | true for the twin, false for real | |
| `rviz` | `false` | Opens RViz with the goal markers. |

There is deliberately **no** `arm` or `allow_arm` launch argument: no parameter, argument,
environment variable or config may start the system armed. On the robot the node pair is started
by the `gripperx-external` systemd service (script `Software/pi_env/systemd/scripts/gripperx-external.sh`);
restarting it is a motion-relevant action (SR-1).

The gateway also loads `gripperx_arm_msgs/config/tf_freshness.yaml` (`max_tf_age_sec`, one bound
shared with the arm). Without it the gateway refuses to start.

## Octopus wire topics (exact names)

| Topic | Type | Direction |
|---|---|---|
| `/octopus/fake_eve_gps_start` | `sensor_msgs/NavSatFix` | in: the geodetic datum |
| `/octopus/trash_goal` | `sensor_msgs/NavSatFix` | in: exactly one current goal |
| `/octopus/trash_gps` | `std_msgs/String` (JSON) | in: full target set, preview and correlation |
| `/octopus/flight_camera_transform/status` | `std_msgs/String` (JSON) | in: their transform health |
| `/octopus/trash_goal_done` | `std_msgs/String` | out: finished target id |
| `/octopus/devices/gripperx/status` | `std_msgs/String` (JSON) | out: robot telemetry |

## Local interface (relative to `/gripperx/external`)

Link to gateway: `datum`, `link_status`, and, only with `goal_ingress_enabled`, `goal`, `targets`.
Gateway to link: `telemetry`, `goal_done`. The link node can also publish `telemetry_json`
(debug).
Calibration to gateway: `line_calibration` (latched JSON); the calibration node also publishes
`line_calibration_markers`, `/tf` (`map -> octopus_line`) and serves `line_calibration_node/reset`
(`std_srvs/Trigger`).
Gateway publishes `goal_status`, `arming_state`, `preview_markers` and `/diagnostics`. It serves
`set_arming` (`gripperx_external_msgs/SetArming`) and `clear_blacklist` (`std_srvs/Trigger`).
It subscribes `/odometry/filtered`, `/global_costmap/costmap`, `/teleop/active_mode`, `/arm/state`,
`/arm/stop_epoch` and `/clock`, and holds action clients for `/navigate_to_pose` and
`/pick_plastic`.

## The arming gate: read this before using the package

Authority is two-layer and **the external system can reach neither layer**.

1. **`set_arming`** is the one and only way into the armed state. It is a plain ROS service on
   the robot's own domain and is **deliberately absent from the rosbridge `services_glob`**.
   Arming always carries an explicit duration; there is no indefinite arm, and the window is
   measured on a **monotonic** clock. Reason with `ArmingState.seconds_remaining`, not with
   `expires_at`. Where an arrival can actuate the arm, each arming carries SR-16's assertion that
   the arm's workspace is clear.
2. **`teleop_mux`** (package `gripperx_teleop`) must be in `autonomous` before Nav2 output reaches
   `/cmd_vel`. It stays operator-owned. The gateway *observes* `/teleop/active_mode` and never
   publishes it.

Ten disarm triggers (codes 1 to 10) are reported in `ArmingState.last_disarm_trigger`, including
link loss, clock stall, clock-jumped-backwards and `PICK_CONFIRM`. A pick that ends `STOPPED` (the
Space E-stop) disarms.

## Pick modes and gates (`grasp.aimed_pick`, startup-only)

| Mode | Behaviour |
|---|---|
| `off` (code default) | The fixed pick sequence (FR-3). |
| `confirm_by_rearm` | After the arrival gates the arm previews the aimed pick; if it passes the gateway disarms and records one pending pick; the operator's next arming approves exactly that pick. |
| `automatic` | The aimed pick runs on arrival, under the arming gate. Allowed outside the simulation since the 2026-11-16 user decision; startup warning; assumptions are marked TO-VERIFY in the config. |

Other gates between arrival and pick, each configured in the YAML: arm at home (`/arm/state`,
`dispatch_requires_arm_home`), localisation settle (`grasp.localisation_settle_*`), correlation of
the dispatched goal with the Octopus target list (`goal_match_tolerance_m`, `max_target_list_age_sec`),
an occlusion latch for a target that disappears from the list at the standoff pose
(`occlusion_latch_enabled`) and the list-loss hold (`correlation_list_loss_hold_sec`: an empty or
stale list is tolerated for that long if no grasp was requested; it never allows a pick on a stale
list and never an acknowledgement). A re-aim in place before the pick exists
(`grasp.reaim_*`); it is off in the real config.

## Configuration

`config/octopus_link_real.yaml` and `config/octopus_link_twin.yaml` (the keys are grouped per
node; comments in the files hold the decisions and their dates). The file and launch-argument
defaults are conservative (`goal_ingress_enabled: false`, `dry_run: true`), but they are **not**
what runs on the robot: the systemd script starts the link with `goal_ingress:=true` and
`dry_run:=false`, so on the robot an armed arrival dispatches; the arming gate and the teleop mux
are the remaining blocks. Differences between the files: both set `allow_arm: true` (it only
permits a later explicit arming call); the real file additionally sets `auto_pick` and
`grasp.aimed_pick` to values that let an armed arrival actuate the arm (user decisions in the
file), the twin keeps `auto_pick: false` and `grasp.aimed_pick: "off"`. `allow_arm` and the grasp mode are startup-only
(the parameter callback refuses a runtime change). Grasp geometry (`grasp.offset_x_m`,
`grasp.tolerance_m`) is in the YAML; do not restate it elsewhere. Robot geometry: 
`gripperx_geometry/config/geometry.yaml`.

## Known gaps

- Several safety-relevant values are marked TO-VERIFY in the config rather than measured: see the
  `grasp.*` block, the geofence bounds and `datum_jump_warn_m` in the YAML.
- **Battery telemetry is permanently unavailable** (`NO_SENSOR_INSTALLED`): there is no
  measurement chain on this robot.
- Octopus target ids restart at 1 when their node restarts. The local blacklist survives a link
  reconnect and is dropped only on evidence of an Octopus id-space reset (the gateway compares the
  identity of the incoming target list; `id_space_resets` in the telemetry counts the drops).

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_external_msgs gripperx_arm_msgs gripperx_external
colcon test --packages-select gripperx_external && colcon test-result --verbose
```

The pure modules can also be tested with plain `python3 test/check_*.py` and nothing running.
Tests use mock rosbridge and motion servers; none moves hardware.
