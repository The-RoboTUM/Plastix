# Test plan: determine steering angle limit experimentally (Task #22d)

Goal: find the largest steering angle at which the wheel/servo bracket
does **not yet** run into the chassis/wheel well — separately for front
and rear, in case they differ (preparation for the optional
front-extended feature from `STEERING_LIMITS.md`).

## Safety framework (binding)

- The robot must be **on a stand** (all four wheels spinning freely, no
  ground contact) — any binding/hitting a stop must show up purely from
  turning the steering servos, without the drive running at the same time.
- **Motion approval is obtained for EACH step individually**, not as a
  blanket approval for the whole session — per the safety rule (incident
  2026-07-06). After a rejection by the user, do not retry on your own.
- No `gripperx-bringup.service` restart during the test series (triggers
  an arm home run + steering servo centering) — the node is already
  running, parameters are changed live via `ros2 param set`.
- Torque stays ON (unlike the calibration procedure in
  `steer_servo_calibrate.py`, which moves things by hand with torque
  OFF) — here the point is precisely to check whether the *motorized*
  stop is problematic, not the hand-moved one.
- Abort immediately (spacebar = EMERGENCY STOP in the keyboard teleop) on
  any audible/visible binding, scraping, or contact.

## Preparation

1. Put the robot on a stand, all four wheels free.
2. `gripperx-bringup` is already running (do not restart). Teleop mode
   `keyboard` (default after bringup).
3. A second person / camera to observe the wheel-to-chassis clearance
   during the test (line of sight to all four wheel wells, not just one).
4. Have `journalctl -u gripperx-*.service` ready, but **no**
   `ros2` CLI diagnostics (topic echo, node info) during/shortly after
   bringup phases — DDS overrun risk. For the actual test series (Pi
   long since booted, no bringup transition), `ros2 topic pub` /
   `ros2 param set` is uncritical.

## Procedure: stepwise angles, front/rear separated

Testing is done directly via `/teleop/direct_steer`
(`std_msgs/Float64MultiArray`, order `[FL, FR, BL, BR]`, values in
rad), NOT via the keyboard — so that exactly reproducible angles are
driven without ramp-up via A/D, and to test the front/rear axle
separately (the keyboard always couples both in opposite directions,
see `keyboard_teleop_node.py`).

For each step: **obtain approval → send command → observe result →
servos back to 0 → next step.**

### Step A — front only, in 5° steps

For `deg` in `10, 15, 20, ..., possibly beyond 45 if unremarkable`:

```bash
python3 -c "import math; print(math.radians($deg))"   # compute rad value
ros2 topic pub -1 /teleop/direct_steer std_msgs/msg/Float64MultiArray \
  "{data: [<rad>, <rad>, 0.0, 0.0]}"
```

Observe: contact/scraping at FL, FR? Back to `{data: [0,0,0,0]}`.
As soon as an angle is noticeable (contact, audible binding, noticeably
increased servo current/juddering): note the last unremarkable value as
`front_limit_deg_safe`, abort the series.

### Step B — rear only, in 5° steps

Analogous, with `{data: [0.0, 0.0, <rad>, <rad>]}` (note the sign: the
keyboard drives the rear opposite to the front, `-angle` — for this
isolated test the sign does not matter, it is about the magnitude;
optionally also repeat with the reversed sign if the mechanics are not
symmetric).

Result: `rear_limit_deg_safe`.

### Step C — realistic combination (front+rear opposed)

With the previously unremarkable values from A/B combined, the way the
keyboard actually drives it (`[δ, δ, -δ, -δ]`), in 5° steps up to the
smaller of the two individual limits:

```bash
ros2 topic pub -1 /teleop/direct_steer std_msgs/msg/Float64MultiArray \
  "{data: [<rad>, <rad>, -<rad>, -<rad>]}"
```

Reason: binding can turn out differently for a combined front+rear
position than for an isolated axle (e.g. if the wheel and fender come
closer together for certain combinations). Result:
`combined_limit_deg_safe`.

### Step D (optional) — check the front-extended hypothesis

If Step A shows a significantly larger safe value at the front than at
the rear (`front_limit_deg_safe > rear_limit_deg_safe`), additionally
test with the rear fixed at `rear_limit_deg_safe` and the front
stepped up further:

```bash
ros2 topic pub -1 /teleop/direct_steer std_msgs/msg/Float64MultiArray \
  "{data: [<rad_front>, <rad_front>, <rad_rear_safe>, -<rad_rear_safe>]}"
```

This is exactly the scenario `enable_front_extended_steering` is
prepared for (see `STEERING_LIMITS.md`).

## Evaluation / adoption into configuration

After the test series, with the three/four determined values:

1. Apply a safety margin (e.g. 1–2° below the last unremarkable value as
   the new limit, do not push all the way to the boundary).
2. `gripperx_control/config/steer_servo.yaml`:
   `steering_angle_limit_deg` = `rear_limit_deg_safe` (or
   `combined_limit_deg_safe`, whichever is more conservative).
3. If front-extended is desired (Step D confirmed):
   `enable_front_extended_steering: true`,
   `front_extended_steering_limit_deg` = the determined front value —
   **and** recalibration of `counts_plus_90`/`counts_minus_90` for FL/FR
   is required (see "Important: calibration, not just a software limit"
   in `STEERING_LIMITS.md`) — this test plan only determines the
   *angle*, not the servo counts for it.
4. Mirror the new window (`steering_outward_limit_deg`,
   `steering_inward_limit_deg`, `steering_outward_sign`) into
   `gripperx_control/config/swerve_cmd.yaml` and
   `config/teleop_joint_commands.yaml`, into the URDF
   (`gripperx_description/urdf/gripperx_v1.core.xacro`, properties
   `steer_outward_limit`/`steer_inward_limit`) and — laptop-side — into
   `keyboard_teleop_node` (`steer_outward_limit_rad`, `steer_inward_limit_rad`,
   `steer_outward_sign`; `steer_limit_rad` is the operator cap and is
   additionally capped at what the window allows). Single symmetric values no
   longer exist anywhere; see the stage-B section of `STEERING_LIMITS.md`. Then
   re-run `python3 src/gripperx_control/test/check_steering_limits.py`.
5. Record the result in the journal (`gripperx-journal`) and in
   `HANDOVER.md`, including the three/four measured values and whether
   front-extended was activated.

## Not part of this test

- Driving behavior under load/ground contact (tested on a stand — real
  friction/traction can show binding differently; possibly a follow-up
  test on the ground with careful, short driving maneuvers after this
  test, again with approval per step).
- The wheel-speed differentiation described in Task #21 — this is
  independent of this angle limit test.
