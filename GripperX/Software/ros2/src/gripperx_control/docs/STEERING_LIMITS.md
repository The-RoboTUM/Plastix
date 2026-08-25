# Steering angle limits (Task #22, parts a–c)

> **Status 2026-08-13 — read the last two sections first.** Sections (a)–(c)
> below describe the historical **symmetric** model (one `steering_angle_limit`
> for all four wheels, both directions) and are kept for the reasoning trail.
> They are **superseded**: the steering range was calibrated on the machine and
> is asymmetric *and* per wheel. Stage A moved `steer_servo_node` onto the
> per-direction schema, stage B moved the kinematics, URDF and teleop onto it.
> Where (a)–(c) name concrete defaults, those numbers are stale.

## Problem

Beyond a certain steering angle, the robot "drives into itself" (wheel
or servo bracket touches the chassis frame). The concrete limit angle
will be **determined later experimentally together with the user** (see
`STEERING_LIMITS_TEST.md`). This document prepares the software.

## (a) Where do limits currently apply?

Three places, in this order along the signal path:

1. **`keyboard_teleop_node.py` (runs on the laptop, NOT on the Pi!)**
   The parameter `steer_limit_rad` (default *then* `0.785` rad ≈ 45°; today
   `radians(35)`, see stage B) limits how
   far the accumulated A/D steering angle can build up before it is
   published on `/teleop/direct_steer`. Purely client-side, no
   hardware reference.

2. **`swerve_cmd_node.py` (Pi, IK path)**
   Parameter `steering_angle_limit` (default `0.7854` rad; **removed** in
   stage B, replaced by the per-wheel window). Limits the
   target steering angles computed by the inverse kinematics before they
   are forwarded as a `JointState`. Applies **only** to the
   autonomy/`cmd_vel` path — the direct-steer path (keyboard) bypasses
   `swerve_cmd_node` entirely (see `STEER_DIFFERENTIAL.md`).

3. **`steer_servo_node.py` (Pi, hardware level) — the place that
   actually matters.**
   Parameter `steering_angle_limit_deg` (default `45.0`°). Applied via
   `_clamp_angle()` / `_angle_to_counts()` / `_counts_to_angle()` to
   **both** incoming paths: `/hw/joint_commands` (from
   `swerve_cmd_node` via `joint_command_bridge`) **and**
   `/teleop/direct_steer` (keyboard bypass). This is the only point at
   which both paths meet again — hence the right place for a robust limit.

   Additionally hard-limited physically by the calibration values
   `counts_plus_90` / `counts_minus_90` (see `steer_servo.yaml`,
   `steer_servo_calibrate.py`): `calibrated_angle_to_counts()` always
   clamps the result at the end to `[min(counts_minus_90, counts_plus_90),
   max(...)]`. These numbers are not servo manufacturer values for "90°",
   but the actually traversed safe end positions from manual calibration
   (comment in the calibration script: "move it by hand to the safe
   end stops — without hitting the chassis"). **A software limit can
   never exceed this physical boundary**, regardless of the configured
   `steering_angle_limit_deg` — this is already a built-in safety
   property, not a new one.

   The servo calibration itself is NOT hard-fixed at ±90°, but the safety
   endpoints chosen during the last calibration run (2026-07-01, see
   comment in `steer_servo.yaml`), currently consistent with `±45°`.

**Conclusion (of Task #22, superseded by stage B):** `steer_servo_node.py` is
the only place that covers both paths (IK and direct steer) together, so the
changes for (b)/(c) applied exclusively there.

That conclusion does not survive the asymmetric range. A clamp at the servo
node is a *last line of defence*, not a plan: whatever it clamps away is a
wheel that no longer points where the kinematics put it, and nothing upstream
learns about it. Since 2026-08-13 the same window is therefore **also** known
upstream — in `swerve_cmd_node` (which plans inside it instead of being
clamped), in `keyboard_teleop_node` (which cannot offer the operator an angle
that would be clamped), in the URDF, and in `sim_steer_bridge` /
`teleop_joint_commands_node`. `config/steer_servo.yaml` stays the single source
of truth for the numbers; the other places mirror it and log the window they
resolved at startup, so a mismatch is visible.

## (b) Implementation: parameterizable, cross-path limit

Already present (unchanged): `steering_angle_limit_deg`, applied to
all four servos, on both paths. No code gap here — the actual
extension is (c).

## (c) Front servos may optionally steer further

New parameters in `steer_servo_node.py` / `gripperx_control/config/steer_servo.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `steering_angle_limit_deg` | `45.0` | Base limit, always applies to the REAR servos, and to the FRONT ones as long as the extension below is off. |
| `enable_front_extended_steering` | `false` | Switch. Off = exactly the previous behavior (a single limit for all four). |
| `front_extended_steering_limit_deg` | `45.0` | Limit for the FRONT servos when the extension is active. Ignored (with a warning) if smaller than `steering_angle_limit_deg` — the extension should never be a reduction. |

Implementation: new method `_limit_rad_for(joint_index)` selects the
appropriate limit per joint (`FRONT_JOINT_INDICES = (0, 1)` =
`f_left_steer`, `f_right_steer`). `_clamp_angle()`, `_angle_to_counts()`,
and `_counts_to_angle()` now accept `joint_index` and use this
limit — both as a clamp and as the scaling reference for the
count conversion. All callers (`_on_commands`, `_on_direct_steer`,
`_on_timer`) were adjusted accordingly. This means the extension applies
identically on both the IK path AND the direct-steer path.

### Important: calibration, not just a software limit

`calibrated_angle_to_counts()` scales linearly between `center` and
`counts_plus_90`/`counts_minus_90` via the ratio
`angle_rad / limit_rad` and then hard-clamps the result to the
calibrated count range. This means:

- If `front_extended_steering_limit_deg` is increased **without**
  recalibrating the `counts_plus_90`/`counts_minus_90` values for FL/FR,
  **nothing dangerous** happens — the servo cannot mathematically go
  beyond the already-calibrated (old, safe) end position, because the
  clamp at the end of `calibrated_angle_to_counts()` prevents this. It
  would simply be non-functional (larger angle values would map to the
  same old end position, just with a "less correct" linear ratio in
  between).
- To actually steer further, the calibration values for FL/FR must be
  recorded again (`steer_servo_calibrate.py`, `calibrate` mode,
  `--limit-deg <new front value>` only for the two front servos; keep the
  lines for BL/BR unchanged from the previous calibration). This is
  deliberate — calibration is the step where a human, with torque OFF,
  manually feels out the safe mechanical stop (see
  `STEERING_LIMITS_TEST.md`).

## Kinematic caveat (OP-2 in the internal `REQUIREMENTS`)

Steering the front further on one side only (front-extended active,
angle beyond the rear limit) leaves the symmetric 4WS geometry that the
`direct_steer` formula in the keyboard teleop relies on (`[angle, angle,
-angle, -angle]`). This can cause scrubbing if the front angles no longer
match the actual state of the rear ones. **This option therefore stays
off by default** and should only be tried after explicit confirmation by
the user (see `OP-2`) and with caution on a robot on a stand.

There is a positive interaction with Task #21 (`STEER_DIFFERENTIAL.md`):
the steering differential there reads the actual four servo angles from
`/hw/steer_states` and reconstructs a *consistent* omega from them via
`model.forward_kinematics_body()` — this continues to work unchanged even
with asymmetric front/rear angles (the function does not assume symmetry,
it is the pseudo-inverse for arbitrary four angles). Should the
front-extended option be activated, the differential tends to mitigate
the scrubbing caused by the asymmetry, but does not replace the
underlying kinematic issue.

## Behavioral neutrality

With the defaults (`enable_front_extended_steering: false`,
`front_extended_steering_limit_deg: 45.0` = `steering_angle_limit_deg`),
the behavior is exactly identical to the previous code — no
deployment risk until the user deliberately sets the values after
the experiment (`STEERING_LIMITS_TEST.md`).

## Deployment on the Pi (NOT executed yet — remote session without
Pi access)

Same procedure as in `STEER_DIFFERENTIAL.md`:

```bash
ssh ubuntu@gripperx-1.local   # with a retry loop
source /opt/ros/jazzy/setup.bash
cd ~/ws
colcon build --packages-select gripperx_control
```

`gripperx_control` is normally (not symlink-) installed; `colcon build`
is sufficient, no manual copying of two paths is needed (that only
affects `gripperx_arm`). Afterwards: **no automatic service restart** of
`gripperx-bringup.service` (triggers an arm home run + steering servo
centering) without explicit user approval.

After activating/changing the limits via parameter override for testing:

```bash
ros2 param set /steer_servo_node steering_angle_limit_deg <new_value>
ros2 param set /steer_servo_node enable_front_extended_steering true
ros2 param set /steer_servo_node front_extended_steering_limit_deg <new_front_value>
```

Afterwards, mirror the change in the places that carry the same window:
`gripperx_control/config/swerve_cmd.yaml` and
`config/teleop_joint_commands.yaml`
(`steering_outward_limit_deg` / `steering_inward_limit_deg` /
`steering_outward_sign`), the URDF joint limits in
`gripperx_description/urdf/gripperx_v1.core.xacro`, and — laptop-side —
`keyboard_teleop_node`'s `steer_outward_limit_rad` / `steer_inward_limit_rad` /
`steer_outward_sign`. Otherwise the kinematics plans for angles the servo node
clamps away, which is the exact failure mode stage B removed.
(`swerve_cmd.yaml steering_angle_limit` no longer exists — a single symmetric
number cannot express this range.)

---

## Update 2026-08-13 — per-direction limits (outward / inward)

The measured mechanical range is **asymmetric**: every wheel swings
**100° outward** (away from the chassis) and only **30° inward**
(user measurement 2026-08-13; raised to **35° inward** 2026-08-17 — a user
estimate, TO-VERIFY, not a new measurement, see `steer_servo.yaml` for the
honesty caveat). Everything above this section describes the
older **symmetric** model, which cannot express that: a single
`steering_angle_limit_deg` of 30 throws away 70° of outward travel, one of
100 turns a commanded −30° into ~9° of real steering.

### New config schema (`config/steer_servo.yaml`)

| Key | Meaning |
|---|---|
| `counts_outward_limit` | raw counts recorded while wheel *i* was held at its outward limit (joint order FL, FR, BL, BR) |
| `counts_inward_limit` | ditto for the inward limit |
| `steering_outward_limit_deg` | angle those outward counts belong to (100.0) |
| `steering_inward_limit_deg` | angle those inward counts belong to (35.0, raised from 30.0 on 2026-08-17) |
| `steering_outward_sign` | per wheel: which **sign of the joint angle** is physically outward (+1/−1) |

All four count/limit keys must be present together — `steer_servo_node`
raises on a half-configured schema instead of silently mis-scaling. While
they are absent, the node keeps using the legacy symmetric keys
(`counts_plus_90`/`counts_minus_90` at ±`steering_angle_limit_deg`) with
bit-identical behaviour to before.

`counts_plus_90`/`counts_minus_90` keep their misleading legacy names (a
rename would break the fallback for configs already deployed); the new
keys are the ones that say what they contain.

### Sign convention (measured 2026-08-13, URDF-checked)

**Definition.** A wheel steers *outward* when the tyre's front swings **away
from the vehicle body laterally**, *inward* when it swings towards it.

**Joint-angle sign.** Uniform for all four steering joints: the URDF macro
`steer_joint` emits `<axis xyz="0 0 1"/>` with `rpy="0 0 0"`, all four joints
have `parent="chassis_link"`, and `chassis_link` sits on `base_link` with
`rpy="0 0 0"`. So every steering joint turns about `base_link`'s +Z —
REP-103 counterclockwise seen from above, no mirroring in the model.
`swerve_kinematic_model.inverse_kinematics()` uses
`delta_i = atan2(vy_i, vx_i)` for every wheel, and `swerve_cmd_node` only
reorders (model order FL, BL, BR, FR ↔ joint order FL, FR, BL, BR) — there
is no per-wheel sign factor anywhere in the steering path. So **+angle
points a wheel towards the robot's left (+y)**, for every wheel.

**One would therefore conclude** outward = +y on the left wheels and −y on the
right ones, `[+1, -1, +1, -1]`. **That conclusion is wrong and was refuted on
the machine** (2026-08-13). The measured value is:

    steering_outward_sign: [-1, +1, +1, -1]     # FL, FR, BL, BR (MEASURED)

Method: all four servos were driven 15° in the measured outward tick direction
and the resulting pose inspected. The wheels lined up tangentially for an
in-place spin — exactly the pattern the kinematics produces for pure rotation
(FL −58.6, FR +58.6, BL +58.6, BR −58.6, each wheel line normalised mod 180).
The magnitude read 50.7 until 2026-08-21; that came from the retired
`b = 0.16556` geometry and understated the pose by about 8°. Measured 58.57 in
the twin, and `atan2(a, b)` on the active `a = 0.180 / b = 0.110` gives the same.
Outward and the spin pose share the pattern (−, +, +, −), so a spin turns every
wheel **outward** and uses 58.6° of the 100° available.

Why the URDF reading misleads: the wheel hangs on a purely lateral lever arm
off the king pin (`*_wheel_offset_xyz`, y = ±0.072 m), so turning the joint
swings the wheel fore/aft around the pin rather than in/out. "Outward" is about
where the wheel body ends up, not about toe. The two readings agree on the rear
pair and contradict each other on the front pair.

Sanity check to re-run if these numbers are ever touched: under `[+1,-1,+1,-1]`
the spin pose needs 58.6° **inward** on three wheels, exceeding the 35° inward
limit — i.e. in-place spin would be impossible. It is asserted in
`gripperx_control/test/check_steering_limits.py`.

In robot-frame angles the reachable range is **−100/+35° on FL and BR and
+100/−35° on FR and BL** — asymmetric *and* per wheel, which is why the limits
are resolved per joint rather than as one signed pair.

**Measured on the machine** (2026-08-13, each wheel turned outward by hand
with torque off, watching which id moved and in which tick direction — no
commanded motion): outward makes the raw count **fall on FL and BR** and
**rise on FR and BL**. That is *diagonal*, not per-side, consistent with the
corner servos being mounted mirrored both left/right and front/rear. It is
ground truth, not derivable from the code, and is carried as
`steering_outward_tick_direction`.

Combining the two with the measured outward signs: `counts_outward_limit` comes
out **below** `center_counts` on FL and BR and **above** it on FR and BL, and
after calibration **FL and FR have `counts_at_pos_limit < center_counts`**
(mirrored mount) — the case `calibrated_counts_to_rad_asym()` handles with a
direction-aware branch instead of the legacy `counts >= center` test.

The same measurement corrected the id map: in joint order FL, FR, BL, BR the
ids are **[13, 14, 11, 12]**, not the committed `[11, 14, 12, 13]` (a
3-cycle: 11 FL→BL, 12 BL→BR, 13 BR→FL, 14 stays FR). See the warning in
`steer_servo.yaml` — `servo_ids` must not be corrected on its own, because
the count arrays are index-aligned with the old order and `center_counts[i]`
is the zero of wheel *i*'s angle↔counts model. Since **OP-20 Option B**
(`center_on_startup`, default `false`) a mis-paired block no longer *commands*
a jump at the next bringup — `_hold_all_servos()` runs only with
`center_on_startup: true` — but it still silently skews every command and
every state readout for the affected wheels.

`steer_servo_calibrate.py calibrate` states the expected outward direction
per wheel, asks the operator to confirm it (a `n` flips that wheel's sign and
says so), and checks every recorded outward endpoint against the measured
tick direction — which is what catches a wheel/id mix-up.

Getting a sign wrong is **not** a mechanical hazard: the conversion stays
hard-clamped to the count window between the two recorded end positions
(`calibrated_counts_bounds()`), both of which were physically reached during
calibration. The wheel would take the opposite direction to the one the
kinematics intends — wrong geometry, not an overload.

### How the front extension composes

`enable_front_extended_steering` / `front_extended_steering_limit_deg`
(Task #22c) still work unchanged in the symmetric fallback. With the
per-direction model they only raise the **outward** limit of FL/FR
(`max(outward, front_extended)`); inward is the self-collision-critical
side (SR-6) and is never extended. The feature is largely **redundant**
now — the measured range is identical on all four wheels — and it still
carries the old caveat: raising a limit without recalibrating the endpoint
counts for that direction only under-scales it (harmless, but useless).

## Stage B 2026-08-13 — the asymmetry in kinematics, URDF and teleop

Stage A gave `steer_servo_node` the per-direction window. Stage B removed the
symmetric numbers upstream of it, so nothing plans a pose the servo node would
have to clamp.

### Where the window now lives

| place | keys | role |
|---|---|---|
| `config/steer_servo.yaml` | `counts_*_limit`, `steering_{outward,inward}_limit_deg`, `steering_outward_sign` | **source of truth** (calibrated), last line of defence |
| `config/swerve_cmd.yaml` | `steering_{outward,inward}_limit_deg`, `steering_outward_sign` | IK plans inside the window |
| `config/teleop_joint_commands.yaml` | same three keys | legacy direct-to-`/hw/joint_commands` path |
| `keyboard_teleop_node` | `steer_{outward,inward}_limit_rad`, `steer_outward_sign`, `steer_limit_rad` | operator cannot request a clamped angle |
| `gripperx_v1.core.xacro` | `steer_outward_limit`, `steer_inward_limit` | model + digital twin joint limits |
| `sim_steer_bridge` | same three keys | twin's counterpart to the servo clamp |

`gripperx_control/steering_limits.py` holds the shared representation
(`SteeringLimits`), the limit-aware module-solution search, and the twist
limiter. It imports no ROS, so it is checkable without a robot:
`python3 src/gripperx_control/test/check_steering_limits.py`.

### What happens to a pose that does not fit

Not a per-wheel clamp — that is precisely the silent failure being removed.
Instead, in `limit_twist_to_steering_range()`:

1. **Limit-aware module choice.** Each wheel has two equivalent solutions,
   `(δ, +v)` and `(δ+180°, −v)`. The reachable one is picked (nearest-to-current
   only decides between two reachable ones). This alone makes crab walk
   reachable: `+90°` does not fit FL/BR, `−90°` with reversed wheel direction
   does, and the resulting pose turns all four wheels outward.
2. **Reduce |ω| only.** Scaling the *whole* twist is provably useless —
   `atan2(vy_i, vx_i)` is invariant under uniform scaling (the mistake the old
   `_scale_twist_to_steer_limit` made). What rotates the wheel angles is the
   ratio ω/(vx, vy): shrinking it pushes the instantaneous centre outward, so
   the manoeuvre keeps its geometry and only becomes a wider turn. Bisection,
   16 steps; each wheel angle is monotone in ω, so this converges on the real
   boundary and is conservative where the ±180° flip splits the reachable set.
3. **Reject** if even ω = 0 does not fit. Then the requested *direction of
   travel* is unreachable (a steep diagonal — FR-8, P4 draft). Honouring it
   partially would mean driving somewhere nobody asked for, so the node holds
   the steering and commands zero drive. Logged as an error.

Every intervention is logged (throttled 2 s). Silence means the pose was taken
as asked.

### Effect on the standard manoeuvres

Measured by `check_steering_limits.py` (geometry a = 0.203, b = 0.16556):

| manoeuvre | requested per wheel | outcome |
|---|---|---|
| pure forward / reverse | 0° | untouched |
| in-place spin | FL −50.8, BL +50.8, BR −50.8, FR +50.8 | **untouched**, outward on all four, 49° of margin |
| crab left (`vy` only) | ±90° | reachable via the module flip, outward on all four |
| corner, vx 0.30, ω 0.40 | max 19.2° | untouched |
| corner, vx 0.30, ω 1.00 | max 56.5° (would be clamped to 30° on two wheels) | ω → 0.58 rad/s, radius 0.52 m instead of 0.30 m, all four inside |
| diagonal 45° | 45° on all four | rejected (unreachable direction of travel) |

The binding constraint in ordinary cornering is the 30° inward limit on the two
wheels that steer into the turn, which caps curvature at roughly
ω ≤ vx·tan30° / (a + b·tan30°) ≈ 1.93 · vx rad/s (minimum radius ≈ 0.52 m).
The 100° outward travel is what makes the tangential in-place spin and crab
possible; it is real travel, but only on the wheels whose outward side a given
pose uses. Raising a single symmetric limit to 100 would be wrong in exactly
the way the old symmetric model was.

### Keyboard teleop

A/D drives all four wheels from one value with the counter-rotating pattern
`[+1, +1, −1, −1]`, so every steering direction puts two wheels on their inward
side: the usable envelope is exactly the inward limit (35°, raised from 30°
2026-08-17 — see the honesty caveat in `steer_servo.yaml`).
`steer_limit_rad` therefore defaults to `radians(35)` (was 0.785 = 45°, of
which the servo node silently discarded 15° on two wheels). The node derives
that bound from the window at startup and caps `steer_limit_rad` at it, with an
error log — an operator can ask for less, never for more.
