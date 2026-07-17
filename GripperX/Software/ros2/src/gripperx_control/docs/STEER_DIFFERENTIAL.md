# Steering differential from servo feedback (Task #21)

## Problem

When steering, the drive wheels should turn at different speeds
(inner wheel slower, outer wheel faster), matched to the
actual steering angle — otherwise the drivetrain binds mechanically.

So far (user observation): in keyboard teleop all four wheels turn
at exactly the same speed, regardless of the steering angle.

**Relation to `REQUIREMENTS.md`:** FR-4 / OP-1. OP-1 actually recommends
first only *measuring* whether the IK-inherent differential is
sufficient in real operation (option A), before implementing a
dedicated, servo-position-dependent correction (option B) — to avoid
duplicate logic. This session was tasked with implementing option B
directly (the assignment explicitly names the keyboard bypass as the
cause, which pure IK measurement on the vehicle could not fix, because
the IK is not called at all on this path). The double-application
concern from OP-1 is addressed here via the `steer_diff_omega_gate`
(see below): the new logic only kicks in exactly where the IK, lacking
a real `omega` command, cannot differentiate at all.

## Cause

Two signal paths exist in parallel and know nothing of each other:

- **Steering (keyboard):** `keyboard_teleop_node` publishes directly on
  `/teleop/direct_steer` → `steer_servo_node` drives the servos. The
  `swerve_cmd_node` (with its inverse kinematics) is completely
  bypassed here.
- **Drive (keyboard):** `keyboard_teleop_node` only publishes `linear.x`
  on `/teleop/keyboard/cmd_vel`. `teleop_mux_node` deliberately sets
  `angular.z = 0` in keyboard mode (code comment: "steering via
  direct_steer"). `swerve_cmd_node` thus receives `/cmd_vel` with
  `omega=0` and computes, via
  `FourWIS4WIDKinematicModel.inverse_kinematics()`, exactly the same
  speed for all four wheels — the model knows nothing of the actual
  servo angle.

In the autonomy path (Nav2 → `/cmd_vel` with `omega != 0`), the problem
does not occur: there, `inverse_kinematics()` receives the real `omega`
and already differentiates the wheel speeds correctly.

Since Fix 6 (`d68d14c`), `steer_servo_node` continuously publishes the
ACTUAL steering angles on `/hw/steer_states` — regardless of whether
they came via `direct_steer` or via the IK chain. `swerve_cmd_node`
already consumes this topic for computing the target steering angles
(`_read_model_steering_angles`), but so far not for the wheel speed.

## Solution

New method `_apply_steer_feedback_differential()` in
`gripperx_control/swerve_cmd_node.py`, called right after the existing
IK computation (`_compute_direct_ik` / `_compute_tracking_control`),
**only** in the normal driving branch (not for point-turn, which has
its own, already correct differentiation):

1. **Gate against double application:** if `|desired_body_twist.omega| >
   steer_diff_omega_gate` (default 0.05 rad/s), nothing is changed — the
   IK has already computed the differentiation from the real command in
   that case (autonomy path). This is the central mechanism that
   prevents this additional logic from duplicating the existing IK
   differentiation.
2. **Reconstructing omega from feedback:** using the ACTUAL steering
   angles from `/hw/steer_states` and an assumed uniform nominal speed
   (average of the currently computed, undifferentiated wheel speeds),
   `model.forward_kinematics_body()` is called — the same function
   otherwise used in the model to convert wheel states back to a body
   twist. This yields a plausible `omega_estimate` that reflects the
   actual steering geometry.
3. **Low-pass filter** (`steer_diff_time_constant_sec`, default 0.3 s) on
   `omega_estimate` for smooth transitions (no jumps from servo noise
   or fast steering movements).
4. **The same IK again, no second algorithm:** with
   `(nominal_vx, 0, omega_estimate)`, `model.inverse_kinematics()` is
   called again — the same function the autonomy path also uses. The
   result replaces the previously uniform wheel speeds. This keeps the
   entire logic a single coherent application of the Lee 2015 model
   instead of a second, hand-rolled Ackermann formula.
5. **Conservative limiting:** every new wheel speed is clamped to
   `[min_ratio, max_ratio] * nominal speed` (default
   0.5…1.5) — a safety net against noisy/faulty servo feedback, no
   wheel can be strongly over- or under-driven by an outlier.
6. Below `steer_diff_min_speed_mps` (default 0.03 m/s) the
   differentiation stays off (standstill/noise) and the filter is
   reset, so that no stale value carries over the next time the robot
   moves off.

## Parameters (`gripperx_control/config/swerve_cmd.yaml`, node `swerve_cmd_node`)

| Parameter | Default | Meaning |
|---|---|---|
| `enable_steer_feedback_differential` | `false` | Feature switch. Off = exactly the previous behavior. |
| `steer_diff_omega_gate` | `0.05` rad/s | Above this commanded omega, the additional logic stays off (autonomy path). |
| `steer_diff_min_speed_mps` | `0.03` | Below this, no differentiation (standstill/noise). |
| `steer_diff_time_constant_sec` | `0.3` | Low-pass time constant for smooth transitions. |
| `steer_diff_max_omega` | `1.5` rad/s | Clamp on the reconstructed omega (safety net). |
| `steer_diff_min_ratio` | `0.5` | Lower bound relative to the nominal speed (inner wheel). |
| `steer_diff_max_ratio` | `1.5` | Upper bound relative to the nominal speed (outer wheel). |

The default is behaviorally neutral throughout
(`enable_steer_feedback_differential: false`) — until deliberately
activated, no behavior change, no deployment risk.

## Limits / deliberately not addressed

- Only affects `swerve_cmd_node`, not `teleop_joint_commands_node.py`
  (simulation/test path, not wired up in real operation on the Pi — see
  `gripperx_control/launch/*.launch.py`). (Its dead-code sibling `teleop_hw.py`,
  originally also excluded here, was deleted 2026-07-15 as unreferenced.)
- Assumes `/hw/steer_states` is fresh (`steer_states_timeout_sec`,
  an already existing parameter). Without fresh feedback, the fallback
  to `/joint_states` kicks in anyway — but that only holds the
  command echo, not the actual angle; in that case the differentiation
  inevitably becomes less accurate (not a new failure mode, an existing
  limitation).

## Deployment on the Pi (NOT executed yet — remote session without
Pi access)

`gripperx_control` is installed on the Pi as a Python package, **no
symlink install**. After merging to the Pi:

```bash
ssh ubuntu@gripperx-1.local   # with a retry loop, see safety rules
source /opt/ros/jazzy/setup.bash
cd ~/ws
colcon build --packages-select gripperx_control
```

`colcon build --packages-select gripperx_control` automatically updates both
install paths for `swerve_cmd_node.py` (unlike `gripperx_arm`, which the trap
documentation in `REQUIREMENTS.md`/agent rules explicitly mentions —
`gripperx_control` uses the normal setuptools install, not the gripperx_arm special
case). Still, verify after the build:

```bash
diff <(python3 -c "import gripperx_control.swerve_cmd_node as m; print(m.__file__)" 2>/dev/null) /dev/null  # check path
```

Afterwards: **no automatic service restart** (per the safety rules this
would trigger an arm home run + steering servo centering). To make the
change effective, `gripperx-bringup.service` must be restarted — only
with explicit user approval.

Activation afterwards can be tested via parameter override (no code
redeploy needed for parameter tests):

```bash
ros2 param set /swerve_cmd_node enable_steer_feedback_differential true
```

(Only test the live parameter set with motion approval — the drive
wheels then react immediately to the steering angle feedback.)
