# gripperx_teleop

Operator input and the mode arbitration in front of `/cmd_vel`.

| Executable | Role |
|---|---|
| `teleop_mux_node` | Multiplexes three input sources onto `/cmd_vel` and publishes the active mode. Runs on the Pi. |
| `keyboard_teleop_node` | Keyboard source. Runs on the laptop. |

## teleop_mux_node — the arbitration point

Subscribes `/teleop/{keyboard,controller,autonomous}/cmd_vel` and `/teleop/set_mode`;
publishes `/cmd_vel` and `/teleop/active_mode` (re-published every tick at
`publish_rate_hz`, default 20 Hz, **not** latched).

`/teleop/set_mode` accepts exactly `keyboard`, `controller`, `autonomous`;
anything else is warned about and ignored. **Nav2 output does not reach `/cmd_vel`
unless the mux is in `autonomous`** — this is layer 2 of the authority gate that
`gripperx_external` is built on. Switching *into* `keyboard` publishes an
immediate zero twist. If the active source goes quiet for `cmd_timeout_sec`
(default 0.5 s) a zero twist goes out.

In `keyboard` mode the full planar twist is forwarded by default
(`keyboard_pass_manoeuvre_axes: true`); the other two modes forward the twist
verbatim.

Config: `config/teleop_mux.yaml`. Launch: `launch/teleop_mux.launch.py`.

## keyboard_teleop_node

Publishes `/teleop/keyboard/cmd_vel`, `/teleop/direct_steer`, `/teleop/set_mode`,
`/arm/command`, and `/teleop/manoeuvre` (latched status line). Subscribes
`/hw/steer_states` (BEST_EFFORT — it crosses the wifi hop). Action client on
`pick_plastic`; **service client on `/gripperx/external/set_arming`** (keys `U`
arm for 120 s, `L` disarm).

Two routes leave this node: `A`/`D` steer through the legacy
`/teleop/direct_steer` pattern, while the arrow-key crab and spin manoeuvres go
the IK route through `/cmd_vel` so they get the per-wheel steering windows and the
module flip. A transition guard withholds traction until the modules are
measurably in pose.

`Space` is the emergency stop: zero twist, wheels straightened, mode forced back
to `keyboard`.

Launch: `launch/laptop_teleop.launch.py`.

## Known defect

**The manoeuvre transition guard uses retired geometry.** This node builds its
kinematic model from `a = 0.203 / b = 0.16556` and therefore predicts a spin pose
of **50.80°**, while `swerve_controller` — using the active `a = 0.180 / b = 0.110`
— commands **58.57°**. The 7.90° gap exceeds the guard's 6.00° tolerance, so on a
spin the guard does not see the pose it is waiting for and arms on its timeout
fallback instead of on the measured angles. A fix exists on a separate branch and
is deliberately not merged before the 2026-08-25 deploy. Note that
`test/check_manoeuvres.py` still passes: it is self-consistent with the retired
constant, so it is green over a number the machine no longer uses.

See `docs/TELEOP_MANOEUVRES.md` — whose twist table carries the same retired
figures, and a spin rate of 0.60 rad/s that is now 0.55.
