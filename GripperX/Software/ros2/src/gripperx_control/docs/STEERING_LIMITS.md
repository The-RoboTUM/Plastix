# Steering angle limits (Task #22, parts a–c)

## Problem

Beyond a certain steering angle, the robot "drives into itself" (wheel
or servo bracket touches the chassis frame). The concrete limit angle
will be **determined later experimentally together with the user** (see
`STEERING_LIMITS_TEST.md`). This document prepares the software.

## (a) Where do limits currently apply?

Three places, in this order along the signal path:

1. **`keyboard_teleop_node.py` (runs on the laptop, NOT on the Pi!)**
   The parameter `steer_limit_rad` (default `0.785` rad ≈ 45°) limits how
   far the accumulated A/D steering angle can build up before it is
   published on `/teleop/direct_steer`. Purely client-side, no
   hardware reference.

2. **`swerve_cmd_node.py` (Pi, IK path)**
   Parameter `steering_angle_limit` (default `0.7854` rad). Limits the
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

**Conclusion:** `steer_servo_node.py` is the only place that covers both
paths (IK and direct steer) together. The changes for (b)/(c) therefore
apply exclusively there — no duplication of the limit logic in
`swerve_cmd_node.py` or `keyboard_teleop_node.py`.

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

## Kinematic caveat (OP-2 in `REQUIREMENTS.md`)

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

Afterwards, be sure to also update `gripperx_control/config/swerve_cmd.yaml`
(`steering_angle_limit`, in rad) and `keyboard_teleop_node`'s
`steer_limit_rad` (laptop-side) to the same new value, so that the IK
does not plan for angles that would be clamped down anyway (not a
safety issue, just unnecessary `steer_alignment_scale` braking).
