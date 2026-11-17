# gripperx_teleop

Operator input and the mode arbitration in front of `/cmd_vel`: a mux on the robot, a terminal
keyboard teleop and a browser teleop on the laptop.

For: operators and developers of the manual-drive and arm-key path. Status: 2026-11-17.

Short forms: SR-1 (safety requirement 1 — no motion without explicit approval per test), SR-3
(safety requirement 3 — dead-man: input that stops arriving stops the robot), SR-18 (safety
requirement 18 — whole-drive fault, operator clear), FR-17 (functional requirement 17 — leader-arm
teleoperation), D17 (the two-stage yaw-rate alarm of the localization work), P / O / I / J / U / L
(the pick, open-gripper, arm-home, leader-mode, arm-gate and disarm-gate keys).

| Executable | Where | Role |
|---|---|---|
| `teleop_mux_node` | Pi | Multiplexes three input sources onto `/cmd_vel` and publishes the active mode. |
| `keyboard_teleop_node` | laptop | Terminal keyboard source. |
| `web_teleop_node` | laptop | Browser UI built on the keyboard node (same keys, same publishers). Run **either** this or the keyboard node, never both: they publish the same `cmd_vel` and would race. |

## teleop_mux_node: the arbitration point

Subscribes `/teleop/{keyboard,controller,autonomous}/cmd_vel` and `/teleop/set_mode`; publishes
`/cmd_vel` and `/teleop/active_mode` (re-published every tick at `publish_rate_hz`, **not**
latched).

`/teleop/set_mode` accepts exactly `keyboard`, `controller`, `autonomous`; anything else is
warned about and ignored. **Nav2 output does not reach `/cmd_vel` unless the mux is in
`autonomous`**: this is layer 2 of the authority gate that `gripperx_external` is built on.
Switching into `keyboard` publishes an immediate zero twist. If the active source goes quiet for
`cmd_timeout_sec` a zero twist goes out. In `keyboard` mode the full planar twist is forwarded
(`keyboard_pass_manoeuvre_axes`); the other two modes forward the twist verbatim. The default mode
is `keyboard` (`initial_mode`).

Config: `config/teleop_mux.yaml`. Launch: `launch/teleop_mux.launch.py`; the bringup starts the
node itself (`gripperx_bringup/launch/real_robot.launch.py`).

## Keys (keyboard node; the web page draws the same keys)

| Key | Action |
|---|---|
| `W` / `S` | forward / backward (dead-man: drives only while held) |
| `A` / `D` | steer left / right (cumulative, stays put when released; legacy `/teleop/direct_steer` route) |
| Arrow `left` / `right` | crab walk sideways (dead-man, IK (inverse kinematics) route through `/cmd_vel`) |
| Arrow `up` / `down` | steer the crab's direction of travel while crabbing |
| `0` / `9` | spin in place clockwise / counter-clockwise (dead-man) |
| `Space` | emergency stop: zero twist, wheels straight, mode back to `keyboard`; also `/arm/stop` (moving arm goes limp), leaves leader mode, cancels a running pick |
| `K` / `G` | mode `keyboard` / `autonomous` |
| `P` / `O` / `I` | pick sequence / open gripper / arm home |
| `U` / `L` | arm the external authority gate for a fixed window / disarm it (service `/gripperx/external/set_arming`) |
| `J` | leader mode (FR-17): off, then J asks the leader arm to align itself to the follower and then enables leader mode (the leader arm moves); J again cancels or switches it off. While leader mode is on, drive keys are blocked |
| `Q` / `Ctrl+C` | quit |

Two routes leave the node: `A`/`D` use the legacy direct-steer topic; crab and spin go through
`/cmd_vel` so they get the per-wheel steering windows and the module flip. A transition guard
(`manoeuvre.py`) withholds traction until the modules are measurably in the new pose, judged on
`/hw/steer_states` or on a worst-case timeout. The pose maths comes from `gripperx_control`
(kinematic model and steering limits) and the geometry parameters in `config/keyboard_teleop.yaml`
are **generated** from `gripperx_geometry/config/geometry.yaml`: do not edit them there.

## Interfaces used by the keyboard/web node (names as in code)

| Dir | Name | Type |
|---|---|---|
| out | `/teleop/keyboard/cmd_vel` (set by parameter), `/teleop/direct_steer`, `/teleop/set_mode`, `/arm/command`, `/arm/stop`, `/arm/leader/enable`, `/arm/leader/align`, `/teleop/manoeuvre` (latched status line) | twist / steer array / string / empty / bool |
| in | `/hw/steer_states` (best effort: it crosses the Wi-Fi hop), `/arm/leader/enable`, `/arm/leader/align_state`, `/arm/stop_epoch` | array / bool / string / int64 |
| client | action `pick_plastic` (`gripperx_arm_msgs/action/PickPlastic`, carries the current stop epoch), service `/gripperx/external/set_arming` | |

`web_teleop_node` additionally subscribes `/teleop/active_mode`, `/joint_states`,
`/gripperx/external/arming_state`, `/arm/leader/wall_state`, `/swerve_controller/drive_fault`,
`/localization/yaw_alarm` and the front camera's compressed image topic (`camera_image_topic`,
default `/camera/front/image_raw/compressed`); it is a client of
`/swerve_controller/clear_drive_fault` and of the camera node's parameter services (profile).

## Browser UI (`web_teleop_node`)

```bash
ros2 launch gripperx_teleop web_teleop.launch.py [open_browser:=true] [web_host:=...] [web_port:=...]
# then open http://localhost:8080/
```

| Launch argument | Default | Meaning |
|---|---|---|
| `web_host` | `127.0.0.1` | Bind address. **`0.0.0.0` puts a live drive interface on the network with no password.** |
| `web_port` | `8080` | |
| `open_browser` | `false` | |
| `client_timeout_sec` | `0.5` | Page silence before the robot is stopped (dead-man, SR-3). |
| `crab_speed_m_s`, `spin_speed_rad_s` | see launch file | Manoeuvre speeds (also arguments of `laptop_teleop.launch.py`). |
| `use_steer_feedback` | `true` | Arm manoeuvres on measured steering instead of a timeout. |

The page sends the complete set of held keys with every beat, so a lost beat can never leave a
key "stuck". One session holds control at a time (a control lease; a silent holder lapses to a
free seat; only a visible, focused page auto-claims a free seat). The page shows the key deck,
robot view (commanded versus measured wheel angles), readouts, an event log, a quit dialog, and:

- **Drive-fault banner and clear button** (SR-18): shows `/swerve_controller/drive_fault` and,
  for the clearable reasons only, a button with a confirmation dialog that sends the shown fault
  identity to `/swerve_controller/clear_drive_fault`. A clear is a motion release.
- **Yaw-rate alarm banner** (D17): a non-latched warning and a latched error from
  `/localization/yaw_alarm`; the latched error stays until the stack restarts.
- **Leader mode** state, alignment state and the leader's virtual-wall state.
- **Camera tile**: stream under the robot view, default off, with a view/detail toggle; frames
  are requested from the camera node only while a viewer is connected.

Details and routes: `docs/TELEOP_WEB_UI.md`.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_teleop
colcon test --packages-select gripperx_teleop && colcon test-result --verbose
```

The tests are desk tests (several use a headless browser); none moves hardware. Manoeuvre
checks: `test/check_manoeuvres.py`, `test/check_crab_steering.py`.

## Stale documents inside this package

`docs/TELEOP_MANOEUVRES.md` still carries figures from the retired geometry (spin pose, spin
rate) and an older key mapping (arrow keys for spin; the code uses `0` / `9`). Treat the code and
`config/` as authoritative until that file is revised.
