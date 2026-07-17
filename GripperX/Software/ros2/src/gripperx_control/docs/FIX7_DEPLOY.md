# Lower publishing rates / merge bridge (Fix 7, NFR-1, Task #11)

## Context

`REQUIREMENTS.md` NFR-1 (status **open** at the time of this preparation —
see "Open points" below): lower publishing rates and merge
`joint_command_bridge` with `swerve_cmd_node` to reduce DDS load and
CPU on the Pi. Background: `controller_manager` overruns
(`write()` 64–319 ms instead of <33 ms, Task #14/NFR-2) — every additional
CPU/DDS load on the Pi is a suspected contributing factor.

This session ran **remotely without Pi access** — everything here is
committed locally, **nothing deployed/tested**.

## Inventory: chain topology (production path)

In production, `gripperx-bringup.service` → `gripperx_bringup real_robot.launch.py`
→ `gripperx_control control.launch.py` runs:

```
keyboard_teleop_node (laptop)
  → /teleop/keyboard/cmd_vel, /teleop/direct_steer
teleop_mux_node (Pi)
  → /cmd_vel  (+ /teleop/active_mode)
swerve_cmd_node (Pi, control_timer)
  → /swerve_cmd_joint_states
joint_command_bridge (Pi, purely reactive, no timer of its own)
  → /steering_position_controller/commands, /wheel_velocity_controller/commands
ros2_control_node / controller_manager (update_rate=30, Task #14/NFR-2)
  → GripperXInterface::write() → /hw/joint_commands
steer_servo_node (Pi, _on_timer, blocking serial I/O)
  ← /hw/joint_commands   → servos
  → /hw/steer_states  (feedback, consumed by swerve_cmd_node)
ESP32 firmware (micro-ROS, outside this repo)
  → /hw/joint_states  (consumed by GripperXInterface::read(), state_timeout_sec=2.0s)
```

`teleop_real.launch.py` / `teleop_joint_commands_node.py` (100 Hz) is a
**legacy/test path**, not used by `gripperx-bringup.service` —
deliberately left untouched so as not to unnecessarily expand this fix package.

## Rate table (current → target)

| Component | Current | Target | Rationale |
|---|---|---|---|
| `keyboard_teleop_node` (laptop) → `/teleop/keyboard/cmd_vel`, `/teleop/direct_steer` | 20 Hz (launch param in `laptop_teleop.launch.py`; the node's 50 Hz default is not used there) | **unchanged, 20 Hz** | Runs over the unstable hotspot network to the Pi — already reduced. Consumer timeouts (`direct_timeout_sec`/`command_timeout_sec` 0.5 s each, `drive_hold_sec` 0.6 s) only require ≥6 Hz; lowering further would noticeably worsen the teleop driving feel without meaningfully saving CPU/DDS on the Pi (small Twist/Float64MultiArray messages). |
| `teleop_mux_node` → `/cmd_vel`, `/teleop/active_mode` | 20 Hz | **unchanged, 20 Hz** | Runs on the Pi, but is pure forwarding with no kinematics compute load; `cmd_timeout_sec`=0.5 s would only need ≥6 Hz. No relevant CPU lever, poor risk/benefit ratio. |
| `swerve_cmd_node` `control_timer` → `/swerve_cmd_joint_states` (+ optionally direct controller commands, see merge) | 50 Hz | **30 Hz** | The downstream consumer `ros2_control` only processes at `controller_manager.update_rate: 30` anyway (`ros2_controllers.yaml`) — 50 Hz already over-supplies it today with no benefit. Margin at 30 Hz (33 ms period) against all dependent time constants: `steer_diff_time_constant_sec` 0.3 s → factor ~9; `cmd_vel_timeout_sec`/`steer_states_timeout_sec` 0.5 s → factor ~15; `GripperXInterface.command_timeout_sec` (watchdog, cbd713b) 0.5 s → factor ~15. Kinematics compute load (IK + differential + point-turn) drops by 40%. |
| `joint_command_bridge` (separate process, reactive — rate = current input rate of `swerve_cmd_node`) | effectively 50 Hz (current), **its own DDS participant + 1 hop** | Rate automatically drops with `swerve_cmd_node` to 30 Hz; **the node itself can optionally be removed** (see merge assessment) | No timer of its own — its cadence follows exactly that of `swerve_cmd_node`. The merge does not primarily save rate, but an entire DDS process + one serialization/hop per cycle. |
| `steer_servo_node` `_on_timer` (blocking serial writes/reads, up to 4 servos) → `/hw/steer_states` | 50 Hz | **30 Hz** | Suspected biggest real CPU/timing cost factor on the Pi (blocking serial I/O per cycle, no Python async). Per the analysis above, the consumer `swerve_cmd_node` only needs ≥6–10 Hz of fresh feedback; 30 Hz leaves a factor of ~15 margin to `steer_states_timeout_sec`/`direct_timeout_sec`/`command_timeout_sec` (0.5 s each) and a factor of ~9 to the differential low-pass (0.3 s). `/hw/joint_commands` only arrives via `GripperXInterface::write()` at `update_rate=30` anyway — 50 Hz polling was already over-supply here. |
| `GripperXInterface::write()` → `/hw/joint_commands` | 30 Hz (`controller_manager.update_rate`) | **unchanged** | Separate NFR (#14/NFR-2) — the *overruns* of this cycle are the symptom, not the rate itself; safeguarded by the watchdog (cbd713b). Not part of this fix. |
| `/hw/joint_states` (ESP32 firmware, micro-ROS) | per the task, ~10 Hz (firmware source not in this repo) | **unchanged** | Outside the software repo; `state_timeout_sec=2.0 s` in `GripperXInterface` already tolerates this with a factor ≥6 margin (at an assumed 10 Hz even a factor of 20). No change possible from this repo. |
| `teleop_joint_commands_node` (legacy, `teleop_real.launch.py`, NOT in the production bringup) | 100 Hz | **unchanged (out of scope)** | Not used by `gripperx-bringup.service`/`real_robot.launch.py`. Documented as a finding, not changed. |

**Rule of thumb kept:** every changed rate stays at a factor of ≥9 above the
strictest dependent timeout/time constant (0.3 s differential low-pass),
factor ≥15 above all 0.5 s timeouts (watchdog, direct steer, steer states,
cmd_vel).

**Trade-off to review:** the NFR-1 acceptance criterion mentions "chain
latency stays within control-loop requirements (50 Hz control)". Interpreted
here as the *latency budget* requirement (not every individual rate has to
stay at exactly 50 Hz) — 30 Hz adds at most ~20 ms of chain latency
(20 ms → 33 ms period), which is not perceptible against the 300–500 ms
timeout budgets and the vehicle speed (≤0.5 m/s teleop). Since NFR-1
currently has status "open", this interpretation should be explicitly
confirmed at the next requirements clarification (`gripperx-requirements`)
before final acceptance.

## Bridge merge: assessment and decision

**Decision: implemented, but OFF by default** (launch argument
`use_integrated_bridge`, default `false` = exactly the previous behavior,
`joint_command_bridge` remains the default and fallback path).

**Why the merge is clean:**
- `joint_command_bridge.command_callback()` (`gripperx_control/joint_command_bridge.py`)
  does nothing more than a name reordering + multiplier: it reads
  `JointState.name/position/velocity` (model order FL, BL, BR, FR) and
  writes two `Float64MultiArray` in controller order (FL, FR, BL, BR).
- `swerve_cmd_node.control_timer_callback()` already has `steering_positions`
  and `wheel_angular_speeds` in exactly the model order
  (`MODEL_STEERING_JOINTS`/`MODEL_DRIVE_JOINTS`) *before* they are packed
  into the `JointState` message — identical data basis, no information
  loss, no second computation.
- The currently deployed `wheel_command_multipliers` value is `[1,1,1,1]`
  (`joint_command_bridge.yaml`, overriding the node code default
  `[1,-1,1,-1]`) — no hidden sign logic that could be lost in the merge.

**Implementation:** new method `SwerveCmdNode._publish_bridge_commands()`
(`gripperx_control/swerve_cmd_node.py`) — an inline equivalent of the bridge
mapping, called right after the existing `/swerve_cmd_joint_states` publish
(which **always** stays active, even with the merge — see compatibility
below). New parameters (all with the same defaults as
`joint_command_bridge.yaml`): `enable_integrated_bridge` (default `false`),
`steering_command_topic`, `wheel_command_topic`, `steering_joint_names`,
`wheel_joint_names`, `wheel_command_multipliers`.

`gripperx_control/launch/control.launch.py`: new launch argument
`use_integrated_bridge` (default `false`). `true` → `enable_integrated_bridge`
is passed through to `swerve_cmd_node`, and the separate `joint_command_bridge`
node is **not** started via `UnlessCondition` (one fewer DDS node and one
fewer hop per cycle). `false` (default) → exactly the previous behavior,
both nodes as usual.

**Why it is not enabled by default right away:** no technical risk is
apparent, but it is unnecessary to deploy the merge *and* the rate change
simultaneously as the default without being able to distinguish between the
two effects (overrun frequency) — see verification below. Enabling it is a
separate, low-risk follow-up step (one launch argument) once the rate change
has been verified on its own.

## Compatibility with recent changes (checked, not broken)

- **Watchdog (`cbd713b`, `GripperXInterface`):** subscribes to `wheel_command_topic`
  (default `/wheel_velocity_controller/commands`) as a liveness signal. This
  topic keeps being published in both modes (bridge separate *or*
  integrated) with identical content and semantics — only the cadence drops
  from 50→30 Hz, which is uncritical given the ~15x margin to the 0.5 s
  timeout. No watchdog parameter changed.
- **Steering differential + limits (`fc1c8fe`):** `swerve_cmd_node` still
  reads `/hw/steer_states` for `_read_model_steering_angles()` and the
  optional differential path; `steer_servo_node`'s extended limits
  (`enable_front_extended_steering`) are independent of `control_rate_hz`.
  Both features remain with their previous defaults (off) unchanged.
- **Fix 6 (`d68d14c`):** `steer_servo_node._publish_state_only()` (publishing
  `/hw/steer_states` even without a received `/hw/joint_commands`) is part of
  `_on_timer()` and remains unchanged — only the timer period (50→30 Hz)
  changes, not the path itself.

## Changed files

- `gripperx_control/config/swerve_cmd.yaml` — `control_rate_hz: 50.0 → 30.0`;
  new bridge merge parameters (default off).
- `gripperx_control/config/steer_servo.yaml` — `control_rate_hz: 50.0 → 30.0`.
- `gripperx_control/src/gripperx_control/swerve_cmd_node.py` — new parameters,
  `_publish_bridge_commands()`, call in `control_timer_callback()`.
- `gripperx_control/launch/control.launch.py` — new launch argument
  `use_integrated_bridge` (default `false`), `UnlessCondition` on the
  `joint_command_bridge` node.
- `gripperx_control/docs/FIX7_DEPLOY.md` — this file.

`joint_command_bridge.py`/`.yaml` **unchanged** — remains a fully
functional fallback.

## Deployment on the Pi (NOT executed yet — remote session without Pi access)

**Dual-path trap `gripperx_control`:** per `docs/HANDOVER.md` (verified during
the Task #17 deployment), `gripperx_control` has **the same dual-path pattern as
`gripperx_arm`** (site-packages copy + entry point separate, no
`--symlink-install`). *Note on a contradiction:* `gripperx_control/docs/STEER_DIFFERENTIAL.md`
claims the opposite ("normal setuptools install, not a special case") —
per the verified finding from Task #17, this is very likely outdated/wrong.
For this deployment, the HANDOVER.md statement is treated as authoritative;
to be safe, **check both paths**:

```bash
ssh ubuntu@gripperx-1.local   # with a retry loop (hotspot unstable)
source /opt/ros/jazzy/setup.bash
cd ~/ws
colcon build --packages-select gripperx_control
# To be safe, explicitly verify both install paths:
diff src/gripperx_control/src/gripperx_control/swerve_cmd_node.py \
     install/gripperx_control/lib/python3.12/site-packages/gripperx_control/swerve_cmd_node.py
diff src/gripperx_control/config/swerve_cmd.yaml \
     install/gripperx_control/share/gripperx_control/config/swerve_cmd.yaml
diff src/gripperx_control/config/steer_servo.yaml \
     install/gripperx_control/share/gripperx_control/config/steer_servo.yaml
diff src/gripperx_control/launch/control.launch.py \
     install/gripperx_control/share/gripperx_control/launch/control.launch.py
```

If `colcon build` does not automatically update the copies (as is the case
for `gripperx_arm`), update them by hand — see the `gripperx_arm` procedure in the
agent rules, analogously for `gripperx_control`.

**No automatic service restart** — per the safety rules this would trigger
an arm home run + steering servo centering. Only with explicit user
approval:

```bash
sudo systemctl restart gripperx-bringup.service
```

## Verification (after deployment + approval)

1. **Before** the restart: capture/count
   `journalctl -u gripperx-bringup.service --since "-15min"`
   (baseline overrun frequency, `write()` >33 ms).
2. Restart only with approval (see above).
3. **Afterwards:** `journalctl -u gripperx-bringup.service -f` (no `ros2` CLI
   during/shortly after boot, see safety rules) — compare overrun frequency
   over a comparable time window against the baseline. Expectation:
   noticeably reduced, since `steer_servo_node`'s blocking serial cycles and
   `swerve_cmd_node`'s kinematics cycles run 40% less often — no guarantee,
   since Task #14/NFR-2 (overrun cause within the `controller_manager`
   itself) may persist independently of this.
4. `ros2 topic hz /hw/steer_states` / `/swerve_cmd_joint_states` /
   `/wheel_velocity_controller/commands` NOT during boot/bringup, only in
   stable operation, briefly — to confirm ~30 Hz instead of ~50 Hz.
5. **Driving test (only with explicit user approval, robot on a stand):** a
   short keyboard teleop test — steering behavior/responsiveness
   subjectively unchanged? Log `/hw/joint_commands` alongside it (see the
   HANDOVER.md rule).
6. Verify the merge separately (optional, its own step): bringup with
   `ros2 launch gripperx_bringup real_robot.launch.py use_mock_firmware:=false
   use_lidar:=true` — note this is just the `control.launch.py` part; in
   practice via the service start with an additional launch argument;
   `gripperx-bringup.sh` would need to be extended with
   `use_integrated_bridge:=true` for that (**not** done in this session,
   since NOT deployed) — `ros2 node list` should then NOT show
   `joint_command_bridge` anymore, but
   `/steering_position_controller/commands` and
   `/wheel_velocity_controller/commands` should still be flowing
   (`ros2 topic hz`, not during boot).

## Open points / conflicts

- **NFR-1 status "open"** at the time of this preparation (`REQUIREMENTS.md`) —
  this implementation was prepared offline on an explicit assignment
  (analogous to Fix 8/Task #12). NFR-1 should be regularly clarified before
  deployment, especially the latency interpretation (see the rate table
  above).
- **Dual-path contradiction** between `docs/HANDOVER.md` (gripperx_control has
  the dual-path trap, verified) and `gripperx_control/docs/STEER_DIFFERENTIAL.md`
  (claims the opposite) — should be cleaned up at the next documentation
  maintenance pass (`gripperx-specification`).
- The change is **not deployed and not tested** — build/node start/
  behavior verification are still pending.
