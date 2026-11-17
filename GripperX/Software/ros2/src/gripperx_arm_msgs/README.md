# gripperx_arm_msgs

Interface definitions for the arm: the pick action, the arm state and the leader-mode
diagnostics, plus the one shared TF (coordinate-transform tree) freshness bound used by both the arm node and the goal
gateway. No nodes; interfaces only.

For: anyone who sends or reads these interfaces. Status: 2026-11-17. The behaviour behind
them is described in `gripperx_arm/README.md` (server side) and `gripperx_external/README.md`
(gateway client). The `.action` and `.msg` files carry the full field comments; this page is
a map, not a copy.

## Definitions

| File | Kind | Used for |
|---|---|---|
| `action/PickPlastic.action` | action | Goal: `execute`, `stop_epoch`, and the aimed-pick fields `aimed`, `target` (`geometry_msgs/PointStamped`), `preview`, `check_target_shift`, `preview_target_in_base`, `max_target_shift_m`. Result: `success`, `message`, machine-readable `reason`, `target_in_base`. Feedback: `progress`, `current_step`. |
| `msg/ArmState.msg` | message | The arm node's own state (`known`, `idle`, `at_home`, `gripper_at_home`, `torque_on`, `owner`). The goal gateway dispatches no navigation unless the newest message says idle and at home and is fresh. |
| `msg/LeaderFollowDebug.msg` | message | Per-tick leader-mode diagnostics (setpoint, target, measured, phase times, outcome). Read-only; nothing subscribes to it for control. |
| `config/tf_freshness.yaml` | parameter file | `max_tf_age_sec`, one bound for the goal gateway and the arm. Loaded by both nodes in every launch path; there is no code default, so without the file the gateway refuses to start and the arm refuses every aimed pick. |

Provided by: `arm_action_server` in `gripperx_arm` (action `pick_plastic`, topics
`/arm/state`, `/arm/leader/follow_debug`). Consumed by: `goal_gateway_node` and
`keyboard_teleop_node` (action client), `gripperx_arm` and `gripperx_external` (the YAML).

## Things worth knowing

- **Stop epoch.** A `PickPlastic` goal must carry the last value the sender saw on
  `/arm/stop_epoch`; a stale or unfilled (0) epoch is refused and nothing moves. This is
  how a Space press always wins over a pick.
- **Aimed pick results.** `reason` is empty on success (`PREVIEW_OK` for a passed preview).
  Refusals before any motion: `OUT_OF_REACH`, `ENVELOPE_REJECTED`, `NO_PATH`, `STALE_TF`,
  `BASE_MOVING`, `STEERING_UNKNOWN`, `INVALID_TARGET`, `AIMED_UNAVAILABLE`,
  `TARGET_SHIFTED`; during motion: `BASE_MOVED_DURING_PICK`; otherwise `NO_PATH`, `REFUSED`,
  `STOPPED`, `CANCELED`, `HOLDING`, `ERROR`.
- **`max_tf_age_sec`** is startup-only in both nodes. Its comment in the file records it as
  TO-VERIFY (a starting value accepted by the user on 2026-11-05, not yet confirmed on the
  robot). After a change restart both nodes.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_arm_msgs
```

Interface-only package: there are no tests. After a change to a definition rebuild every
package that depends on it (`gripperx_arm`, `gripperx_external`, `gripperx_teleop`) and, on the
robot, redeploy them together.
