# Crab walk and in-place spin on the arrow keys (FR-7)

> Status 2026-08-13. Implemented in the working tree, **not yet deployed to the
> Pi and not yet driven**. FR-7 in the internal `REQUIREMENTS` is still `draft` /
> P4-nice-to-have and explicitly says "not a task for now" — the status needs to
> catch up with the user's request before this is accepted.

## Keys

| Key | Manoeuvre | Twist published |
|---|---|---|
| `W` / `S` | forward / backward (unchanged) | `linear.x = ±0.50 m/s` |
| `A` / `D` | steer, cumulative (unchanged) | pattern on `/teleop/direct_steer` |
| `←` | crab left | `linear.y = +0.25 m/s` |
| `→` | crab right | `linear.y = -0.25 m/s` |
| `↑` | spin clockwise | `angular.z = -0.60 rad/s` |
| `↓` | spin counter-clockwise | `angular.z = +0.60 rad/s` |
| `Space` | emergency stop, every manoeuvre (SR-2) | zero + straighten + keyboard mode |

The arrows are **dead-man keys** exactly like `W`/`S` (SR-3): they act only while
the key repeats, and releasing ends the manoeuvre within `drive_hold_sec` and
returns the wheels to straight ahead. Pressing `W`/`S`/`A`/`D` cancels an active
manoeuvre; pressing another arrow switches to it. Only one manoeuvre can ever be
active — the most recent press wins.

`↑ = clockwise` is the mapping the user specified. It is deliberately not the
"up = forward" reading; the banner says so.

## Two routes out of one node

```
A/D  ──► /teleop/direct_steer ─────────────────────────► steer_servo_node
             (fixed pattern, overrides /hw/joint_commands for 0.5 s)

arrows ─► /teleop/keyboard/cmd_vel ─► teleop_mux ─► /cmd_vel ─► swerve_cmd_node
             (linear.y / angular.z)                              │ per-wheel windows
                                                                 ▼
                                             joint_command_bridge ─► ros2_control
                                                     ─► /hw/joint_commands ─► servos
```

Crab and spin need wheel poses a fixed per-wheel pattern cannot express — all
four at ∓90° and ∓50.8° respectively — and they need the calibrated per-wheel
steering windows (`FL[-100,+30] FR[-30,+100] BL[-30,+100] BR[-100,+30]`) so no
wheel is silently clamped out of the pose. The direct-steer route has none of
that. So the arrows take the IK route, where the limits, the ±180° module flip
and the per-wheel speed split all come for free.

The legacy direct-steer route is **bypassed, not removed**: `A`/`D` still use
it unchanged, and the `/hw/joint_commands` double-publisher question is
untouched by this change.

Two things had to give way:

1. **`teleop_mux` zeroed `angular.z`** (and never forwarded `linear.y`) in
   keyboard mode. New parameter `keyboard_pass_manoeuvre_axes` (default `true`)
   forwards the full planar twist. Behaviour-neutral for the existing keys: in
   cornering mode the keyboard node publishes `linear.x` only.
2. **`steer_servo_node`'s direct-steer override wins** over
   `/hw/joint_commands` for `direct_timeout_sec = 0.5 s` after the last
   message. While a manoeuvre is active the keyboard node therefore **stops
   publishing `/teleop/direct_steer`** so the override lapses and the IK path
   actually owns the servos.

## Transition guard

Cornering, crab and spin park the wheels in completely different poses.
Switching swings modules through up to 140°, roughly 0.3–0.5 s per 90°.
Commanding drive during that slew moves the robot in a direction nobody asked
for and scrubs the tyres. `manoeuvre.TransitionGuard` prevents it:

| State | What is published | Leaves when |
|---|---|---|
| `releasing` | zero twist | `direct_release_sec` (0.7 s) elapsed — the override is gone |
| `aligning` | the **target pose** at `manoeuvre_pose_scale` (2 %) | every wheel within `align_tolerance_rad` (6°) of the target, measured on `/hw/steer_states` |
| `armed` | the full twist | manoeuvre changes |

* **Feedback, not a fixed delay.** The node subscribes to `/hw/steer_states`
  (`BEST_EFFORT`, depth 1 — it crosses the unstable hotspot link, NFR-8) and
  compares against the pose it predicts `swerve_cmd_node` will command. That
  prediction is not a second formula: it calls the same `inverse_kinematics()`
  + `resolve_wheel_targets()` with the same calibrated windows, which is why
  `gripperx_teleop` now depends on `gripperx_control`.
* **Fallback.** If the feedback never arrives, the guard arms after
  `align_timeout_sec` (1.5 s) and says so on the status line
  (`NO STEER FEEDBACK, armed on timeout, pose NOT confirmed`). Refusing to
  ever move with no diagnosis would be worse; arming silently would be worse
  still.
* **Switching while moving stops first.** The drive is withheld from the
  instant the new manoeuvre is requested, so there is no "keep driving in the
  old direction while the wheels turn" window.
* **Not zero traction, 2 %.** The chain has no steer-only command — a module's
  angle and speed both come out of one twist. But the IK's
  `delta_i = atan2(vy_i, vx_i)` is *invariant* under positive scaling of the
  whole twist, so a twist scaled to 2 % commands **exactly** the target pose at
  2 % of the wheel speed (5 mm/s for crab, vs 250 mm/s armed). The clean fix
  would be a real steer-only path — either `steer_alignment_min_scale: 0.0` in
  `swerve_cmd.yaml` (affects cornering too) or an explicit flag through the
  chain. Both are bigger decisions than this change.
* **Known cosmetic artifact in `releasing`.** `direct_release_sec` (0.7 s) is
  longer than `steer_servo_node`'s `direct_timeout_sec` (0.5 s) on purpose, so
  the override is definitely gone before the pose is commanded. In the 0.2 s
  between the two, `/hw/joint_commands` already rules while the node is still
  publishing a **zero** twist — i.e. the wheels are briefly commanded straight.
  This is the most conservative thing to command, costs at most ~0.2 s of
  no-traction servo travel, and is on the way to the target pose anyway. A
  single stray arrow tap (no auto-repeat) never gets past `releasing` at all,
  so it produces that twitch and nothing else — no drive, no pose change.
  Shortening `direct_release_sec` reduces the twitch but narrows the margin on
  a flaky link; the guard is feedback-based, so a too-long release only delays
  arming.
* **Untouched when unused.** The guard starts `armed` in `cornering`. If no
  arrow is ever pressed it never transitions, so plain `W/S/A/D` driving is
  identical to before — including on a laptop that never sees
  `/hw/steer_states`.

## Operator feedback

The manoeuvre is no longer inferable from which key is held, and arrow-left
first swinging all four wheels 90° is alarming unannounced. So:

* a status line is written to the raw terminal on every state change,
  e.g. `>> CRAB LEFT (sideways, +vy) | aligning - wheels moving into pose, drive withheld`;
* the same string is published on `/teleop/manoeuvre` (`std_msgs/String`,
  `TRANSIENT_LOCAL` so a late subscriber sees the current state);
* the banner lists the arrows and warns that the robot moves only once the
  wheels are in pose.

The status line deliberately carries no live angle error — it changes every
tick during a slew and would scroll the terminal at the publish rate.

## Twist table (verified, `test/check_manoeuvres.py`)

Model order FL, BL, BR, FR. All angles inside their window; the limiter passes
every one through unchanged (`status: ok`).

| Manoeuvre | twist | FL | BL | BR | FR | max wheel speed |
|---|---|---|---|---|---|---|
| crab left | `vy=+0.25` | −90.00 | +90.00 | −90.00 | +90.00 | 4.81 rad/s |
| crab right | `vy=−0.25` | −90.00 | +90.00 | −90.00 | +90.00 | 4.81 rad/s |
| spin CW | `ω=−0.60` | −50.80 | +50.80 | −50.80 | +50.80 | 3.02 rad/s |
| spin CCW | `ω=+0.60` | −50.80 | +50.80 | −50.80 | +50.80 | 3.02 rad/s |

Note that crab left and crab right share **one** pose and differ only in the
sign of the wheel speeds, so switching between them needs no slew at all. Crab
reaches ∓90° because `resolve_wheel_targets` takes the ±180° flip on FL and BR,
whose windows exclude the naive +90°; those two wheels then drive backwards.
Spin needs 50.80° **outward** on all four, 49° inside the 100° outward limit.

## Dependency: `enable_point_turn` must stay `false`

`swerve_cmd.yaml` sets `enable_point_turn: false`, so pure rotation goes through
the **swerve** spin (wheels on the rotation tangent at ∓50.8°, zero scrub) — the
pose this guard waits for. Setting it back to `true` would route any
`|ω| ≥ point_turn_omega_threshold` (0.35 rad/s; arrow spin publishes 0.60) into
`_compute_point_turn`, which holds the wheels **straight** and skid-turns
instead. The guard would then wait for a 50.8° pose that is never commanded,
arm on the timeout, and the robot would tank-turn. If point turn is ever
re-enabled, `spin_speed_rad_s` and this guard need revisiting together.

## Verification

```bash
# kinematics + guard logic, no ROS
python3 src/gripperx_teleop/test/check_manoeuvres.py

# real nodes, real DDS, key press -> /cmd_vel; pins itself to domain 221 +
# localhost discovery so it can never reach the robot (SR-8)
source install/setup.bash
python3 src/gripperx_teleop/test/check_teleop_manoeuvre_path.py
```

## Deployment (pending)

`teleop_mux` runs on the Pi as part of bringup. Until the new
`gripperx_teleop` is deployed and `teleop_mux` restarted, the arrow keys do
nothing on the real robot — the mux still zeroes the axes. Restarting
`gripperx-bringup.service` is itself movement (arm home + steering centering)
and needs explicit user approval per SR-1.
