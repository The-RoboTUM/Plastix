# Crab walk and in-place spin on the arrow keys (FR-7)

> Written 2026-08-13, brought in line with the code 2026-11-17. The manoeuvres are
> implemented in `gripperx_teleop` (FR-7, the crab-walk and spin requirement). The
> description of the guard below has not been re-driven on the robot since the
> drive stack moved to `swerve_controller`; treat timings and behaviour as
> derived from the code, not freshly measured.

## Keys

| Key | Manoeuvre | Twist published |
|---|---|---|
| `W` / `S` | forward / backward (unchanged) | `linear.x = ±0.50 m/s` |
| `A` / `D` | steer, cumulative (unchanged) | pattern on `/teleop/direct_steer` |
| `←` | crab left | `linear.y = +0.25 m/s` |
| `→` | crab right | `linear.y = -0.25 m/s` |
| `↑` / `↓` | while crabbing: steer the crab's direction of travel | modifier of an active crab, does nothing otherwise |
| `0` | spin clockwise | `angular.z = -spin_speed_rad_s` |
| `9` | spin counter-clockwise | `angular.z = +spin_speed_rad_s` |

`spin_speed_rad_s` is a node parameter: its default in `keyboard_teleop_node.py` is
0.55 rad/s, but `launch/laptop_teleop.launch.py` and `launch/web_teleop.launch.py`
both pass 1.4 rad/s (the reasoning, a stiction bound, is in the comment of
`laptop_teleop.launch.py`), so a normally launched teleop spins at the launch value.
| `Space` | emergency stop, every manoeuvre (SR-2) | zero + straighten + keyboard mode |

The arrows are **dead-man keys** exactly like `W`/`S` (SR-3): they act only while
the key repeats, and releasing ends the manoeuvre within `drive_hold_sec` and
returns the wheels to straight ahead. Pressing `W`/`S`/`A`/`D` cancels an active
manoeuvre; pressing another arrow switches to it. Only one manoeuvre can ever be
active — the most recent press wins.

Spin was first put on `↑`/`↓` (2026-08); it is on `0`/`9` now, because the up/down
arrows steer a crab's direction of travel.

## Two routes out of one node

```
A/D  ──► /teleop/direct_steer ─────────────────────────► steer_servo_node
             (fixed pattern, overrides /hw/joint_commands for 0.5 s)

arrows, 0/9 ─► /teleop/keyboard/cmd_vel ─► teleop_mux ─► /cmd_vel
                 (linear.x/y, angular.z)                 │
                                                          ▼
                              swerve_controller (ros2_control controller, package
                              gripperx_swerve_controller): inverse kinematics
                              and the per-wheel steering windows
```

(`swerve_cmd_node` and `joint_command_bridge`, which this page described in
2026-08, no longer exist; `swerve_controller` replaced them.)

Crab and spin need wheel poses a fixed per-wheel pattern cannot express, and
they need the calibrated per-wheel steering windows (the live values are in
`gripperx_control/config/ros2_controllers.yaml`; spin pose angle follows from
`gripperx_geometry/config/geometry.yaml`) so no wheel is silently clamped out
of the pose. The direct-steer route has none of that. So the arrows and `0`/`9`
take the inverse-kinematics route, where the limits, the ±180° module flip and
the per-wheel speed split all come for free.

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
  compares against the pose it predicts the drive controller will command. That
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
  would be a real steer-only path in the drive controller. That is a bigger
  decision than this change.
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

## Wheel poses

The poses (all angles inside their steering windows, the limiter passes them
unchanged) are computed by `gripperx_teleop/manoeuvre.py` from the same inverse
kinematics as the controller; `test/check_manoeuvres.py` checks them. The
numbers are not repeated here: the spin pose follows from the robot geometry
(`Software/ros2/src/gripperx_geometry/config/geometry.yaml`), the windows are
in `gripperx_control/config/ros2_controllers.yaml`.

Crab left and crab right (without crab steering) share **one** pose and differ
only in the sign of the wheel speeds, so switching between them needs no slew at
all. The crab pose is reached through the ±180° flip that `resolve_wheel_targets`
takes on the wheels whose windows exclude the naive angle; those wheels then
drive backwards. Spin needs the same outward angle on all four wheels.

## Dependency: point turn must stay off

The active drive configuration has pure rotation go through the **swerve**
spin (wheels on the rotation tangent, zero scrub) — the pose this guard waits
for. `enable_point_turn` is not ported to `swerve_controller` and is false in the
legacy `gripperx_control/config/swerve_cmd.yaml`
(`gripperx_control/config/ros2_controllers.yaml` states this, section "NOT
PORTED"). A skid turn with the wheels held straight would make the guard wait for
a pose that is never commanded, arm on the timeout, and the robot would
tank-turn. If a point-turn mode is ever re-enabled, `spin_speed_rad_s` and this
guard need revisiting together.

## Verification

```bash
# kinematics + guard logic, no ROS
python3 src/gripperx_teleop/test/check_manoeuvres.py

# real nodes, real DDS, key press -> /cmd_vel; pins itself to domain 221 +
# localhost discovery so it can never reach the robot (SR-8)
source install/setup.bash
python3 src/gripperx_teleop/test/check_teleop_manoeuvre_path.py
```

## Deployment

`teleop_mux` runs on the Pi as part of bringup; the keyboard node runs on the
laptop. A change to `gripperx_teleop` reaches the robot only through the deploy
procedure in `documentation/DEPLOYMENT.md` §0. Restarting `gripperx-bringup.service`
is still motion-relevant and needs explicit user approval per SR-1 (the
motor-enable process rule): it makes the drive path commandable again and leaves
the steering limp at an unknown angle. It does **not** home the arm
(`arm_home_on_startup` is permanently false) and does **not** centre the steering.
