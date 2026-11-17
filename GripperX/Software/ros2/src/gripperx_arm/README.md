# gripperx_arm

The follower arm of the robot: a 6-servo SO101-type arm (five limb joints plus the gripper,
Feetech servos on one serial bus). The package holds the node that is the **only writer to
that bus**, the safety filter every arm setpoint must pass, the planner, the aimed-grasp
(adaptive pick) code and a set of operator and calibration tools.

For: anyone who runs, tests or changes the arm. Status: 2026-11-17. Which behaviours are
accepted on the robot is recorded in the requirements document, not here; this page only
describes what the code does.

Short forms used below: SR-17 (safety requirement 17 — arm safety envelope), SR-16 (safety
requirement 16 — scoped exception to the motion-approval rule for the automatic pick), SR-1
(safety requirement 1 — no motion without explicit approval per test), FR-3 (functional
requirement 3 — fixed grip sequence), FR-15 (arm kinematic model), FR-16 (adaptive grasp),
FR-17 (leader-arm teleoperation), P (pick key / pick action), I (return-home key), J
(leader-mode key), L1/L2/L3 (safety layers: servo EEPROM (non-volatile servo memory) limits / URDF joint limits /
runtime filter).

## Executables

| Executable (`ros2 run gripperx_arm ...`) | Role |
|---|---|
| `arm_action_server` | The arm node. Single writer to `/dev/arm_servo` (1 Mbaud). Serves the pick action, `/arm/command`, leader mode, stop handling, the safety filter and the planner. |
| `arm_joint_state_relay` | Converts raw servo angles on `/arm/joint_states` into URDF joint angles on `/joint_states` using `config/arm_calibration.yaml`. Read-only; warns while `calibration_verified` is false. |
| `arm_pose_teach` | Hand-teach a pose: torque off, move by hand, prints a YAML block for `arm_poses.yaml`. Needs the bus, so stop `arm_action_server` first. `--watch` is read-only. |
| `arm_eeprom_limits` | Computes the L1 (servo EEPROM) angle limits from the URDF limits, the calibration and the margins in `arm_safety_filter.yaml`; writes them only on request. Moves nothing but changes servo memory: SR-1 applies. |
| `arm_wrap_offset` | Plans (dry run unless `--write`) the per-servo offset that moves the 0/4095 tick wrap out of a joint's reachable range. Input: the leader's `record_limits` output. |
| `arm_poses_to_rad` | Prints the taught tick poses as angles relative to home, for review. |
| `arm_aimed_pick` | Operator test tool: sends **one** aimed pick to a target (default frame `base_link`, `--frame map`). It moves the arm and the gripper, asks for a typed confirmation, and needs an explicit approval for that test (SR-1). |
| `arm_aimed_region` | Offline: prints the aimed pick's allowed grasp region and a standoff proposal from the active model and configuration. Moves nothing, opens no port. |

Offline generators (run with `python3 -m gripperx_arm.<module>`, not installed as nodes):
`generate_acm` writes `config/arm_allowed_collisions.yaml` from the robot description;
`stop_protected` writes `config/arm_stop_protected.yaml` (link pairs the mechanical stops
already keep apart). Both files are generated: do not edit them by hand.

Other modules (pure Python, unit-tested): `kinematics` (URDF-driven kinematics, FR-15),
`safety_filter` (SR-17), `planner` (direct, joint-ordered detour and RRT-Connect (randomised path search) paths),
`exec_steps` (path to whole servo ticks), `aimed_grasp` (grasp candidates, region),
`tf_intake` (bounded TF (coordinate-transform) reader for the aimed pick), `leader_band` (leader command band and
edge slowdown), `servo_bus`, `poses`, `convex_distance`, `mesh_distance`.

## Launch

`launch/arm.launch.py` starts `arm_action_server` with `arm_poses.yaml`,
`arm_safety_filter.yaml` and the shared `tf_freshness.yaml` (from `gripperx_arm_msgs`).

| Launch argument | Default | Meaning |
|---|---|---|
| `home_on_startup` | `false` | Move the arm to home when the node starts. Kept false on purpose: the node is respawned by the bringup, so true would mean unattended arm motion at unpredictable moments. In `real_robot.launch.py` its own `arm_home_on_startup` argument overrides the YAML value. |

In the real bringup the node and the relay are started by
`gripperx_bringup/launch/real_robot.launch.py`.

## Interfaces (names as in code)

Action server: `pick_plastic` (`gripperx_arm_msgs/action/PickPlastic`). Fixed pick (FR-3) or,
with `aimed: true`, an aimed pick at a target (x, y); `preview: true` runs every refusal
check without moving. A goal must carry the current stop epoch (see `/arm/stop_epoch`),
otherwise it is refused.

| Dir | Topic | Type | Note |
|---|---|---|---|
| in | `/arm/command` | `std_msgs/String` | `go_home` (planned, checked), `open_gripper` (not blocked by the filter); anything else is warned about and ignored |
| in | `/arm/stop` | `std_msgs/Empty` | Space: a moving arm goes limp (torque off), a stationary arm is left alone; raises the stop epoch |
| in | `/arm/leader/enable`, `/arm/leader/active` | `std_msgs/Bool` | leader mode control and heartbeat |
| in | `/arm/leader/joint_states` | `sensor_msgs/JointState` | leader arm stream (all six joints) |
| in | `/arm/leader/calibration_verified` | `std_msgs/Bool` | name set by `leader.calibration_verified_topic` |
| in | `/gripperx/external/arming_state` | `gripperx_external_msgs/ArmingState` | name set by `leader.arming_state_topic` |
| in | `/hw/steer_states` | `std_msgs/Float64MultiArray` | steering angles for the steering envelope and the aimed pick's checks (`safety_filter.steer_states_topic`) |
| in | `/robot_description` | `std_msgs/String` | model for the filter |
| in | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | aimed pick only (raw prefilter, see `tf_intake`) |
| out | `/arm/joint_states` | `sensor_msgs/JointState` | raw servo angles, 2 Hz |
| out | `/arm/state` | `gripperx_arm_msgs/ArmState` | latched; idle / at home / torque / owner, read by the goal gateway |
| out | `/arm/stop_epoch` | `std_msgs/Int64` | latched; raised on every Space |
| out | `/arm/safety_filter/status` | `std_msgs/String` | latched |
| out | `/arm/leader/state` | `std_msgs/String` | latched |
| out | `/arm/leader/follow_debug` | `gripperx_arm_msgs/LeaderFollowDebug` | per tick while leader mode is active (`leader.follow_debug_enabled`) |
| out | `/arm/aimed/region` | `std_msgs/String` | latched JSON of the allowed grasp region (`aimed.region_topic`) |

`arm_joint_state_relay` publishes `/joint_states` from `/arm/joint_states`.

## Key behaviour

- **One writer, one safety chain (SR-17).** Every arm setpoint passes the filter in
  `gripperx_arm.safety_filter`. Without the filter, its configuration or the model, P and I
  are refused. Paths are planned and validated in whole servo ticks before the first write;
  no path means no motion. Gripper-only moves are evaluated and logged but not blocked
  (the joint limit still applies).
- **Automatic versus operator motion.** While `planner.automatic_paths` is `direct_only`
  (the default), automatically triggered motions (return home after an abort, re-plan in a
  running pick, a pick started by the goal gateway) take the direct path or none; detours
  are reserved for operator P and I.
- **Stops.** Space (`/arm/stop`) makes a moving arm limp; every other abort returns home on a
  planned path, or holds and reports if none exists. A refusal after motion started holds at
  the measured position and reports it.
- **Aimed pick (FR-16).** The target is transformed once, at the start, into `base_link`
  (the TF must be fresh within `max_tf_age_sec`, one bound shared with the goal gateway
  through `gripperx_arm_msgs/config/tf_freshness.yaml`). The gripper is vertical, the wrist
  roll is searched; the approach ends above the grasp pose and descends vertically
  (`aimed.pregrasp_height_m`, `descent_step_m`, `descent_max_lateral_m`). Refusal reasons are
  listed in `PickPlastic.action`. The base is watched during the whole pick
  (`BASE_MOVED_DURING_PICK` returns the arm home).
- **Leader mode (FR-17).** The six-servo leader arm (package `gripperx_leader`, on the
  laptop) drives the follower, gripper included. Setpoints stay a margin inside the joint
  limits (`leader_band`) and slow down toward the band edges; one rejected setpoint is not
  written but does not end leader mode. A closing-side gripper block does not end leader
  mode.
- **Startup.** The arm stays where it was left; nothing may assume it is at home.

## Configuration (one home per fact)

| File | Holds |
|---|---|
| `config/arm_poses.yaml` | node-level settings: `home_on_startup`, taught `grip` pose, gripper open/close. `home` is not here: it is the calibration reference (`zero_ticks`) |
| `config/arm_calibration.yaml` | tick to joint-angle calibration, `zero_ticks`, `calibration_verified` |
| `config/arm_safety_filter.yaml` | SR-17 filter, planner, abort/hold/stop/bus behaviour, EEPROM margins, `arm_state`, `leader.*`, `aimed.*` |
| `config/arm_allowed_collisions.yaml`, `config/arm_stop_protected.yaml` | generated collision-pair lists |
| `../gripperx_arm_msgs/config/tf_freshness.yaml` | `max_tf_age_sec`, shared with the goal gateway; startup-only, restart both nodes after a change |
| `../gripperx_geometry/config/geometry.yaml` | joint limits, boxes and every geometric value (never restate them here) |

Speed, margin and torque values are tuned on the robot and recorded with their user
decisions in the YAML comments. Values marked TO-VERIFY there are placeholders, not safety
margins.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_arm_msgs gripperx_arm
colcon test --packages-select gripperx_arm && colcon test-result --verbose
```

Several node tests use their own `ROS_DOMAIN_ID` (set in `CMakeLists.txt`) with discovery
off, so they do not touch a running robot's network. They use a fake servo bus; no test moves
hardware. Any run against the real arm needs an approval under SR-1.
