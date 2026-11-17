# gripperx_localization

EKF, wheel odometry, laser odom, SLAM, AMCL, and **RViz** (a single launch).

For: anyone who runs or changes localization and the odometry cross-checks. Status: 2026-11-17.
Nav2 is in **`gripperx_planning`**.

Short forms: EKF (extended Kalman filter, `robot_localization`), AMCL (adaptive Monte Carlo
localization on a saved map), SLAM (simultaneous localization and mapping, `slam_toolbox`), IMU
(inertial sensor; the BNO085 on the robot), D9 / D17 (user decisions on the yaw-rate cross-check
and its two-stage alarm, recorded in `config/localization.yaml`).

## Simulation (terminal 2)

With Gazebo already running:

```bash
source install/setup.bash
ros2 launch gripperx_localization localization.launch.py
```

By default: map (`arena_map.yaml`), AMCL, laser odom, EKF, and RViz with `/map`.

The AMCL initial pose (`x=2`, `y=0`) must match the sim spawn (`simulation.launch.py`). If the robot appears **outside the map**, align both or use **2D Pose Estimate** in RViz.

RViz runs in the same launch to view map, laser, pose, and send a **2D Goal** to Nav2 (T3). Without the window: `use_rviz:=false`.

## Real robot

Same, with `use_sim_time:=false`.

## Modes

| Mode | Flags |
|------|--------|
| Map + AMCL | `enable_saved_map_localization:=true` |
| SLAM | `enable_slam:=true`, `enable_saved_map_localization:=false` |
| EKF only | both `false` |

## Nodes

`localization_input_node` (turns `/joint_states` into `/wheel/odom`, and
republishes the BNO085 stream from `/imu/bno085/data` on `/imu/data/filtered` with the
standstill gyro bias removed and implausible samples dropped; since step 2.4 the
EKF's `imu0` fuses its yaw rate only (index 11), on the robot and the twin alike; the gyro-bias
guard publishes its live state as JSON on `/localization/gyro_bias_state`, latched, parameters
`gyro_bias_*` in `config/localization.yaml`),
`ground_truth_odom_bridge` (twin only), and
`odom_divergence_monitor`, which compares two odometry sources and reports on
`/diagnostics`. Offline analysis tools (not part of the running stack): `yaw_compare` (per-phase
yaw agreement table from a bag), `imu_replay` and `imu_replay_report` (replay a bag through the
fusion with `launch/imu_replay.launch.py`, on an isolated DDS domain).

`localization.launch.py` also starts `line_calibration_node` from `gripperx_external` (argument
`enable_line_calibration`, default true), so the Octopus reference-line calibration lives and dies
with slam_toolbox. Main launch arguments: `use_sim_time` (default true: pass `false` on the
robot), `enable_saved_map_localization` (default true), `enable_slam`, `enable_laser_odometry`
(default true), `fuse_laser_odometry` (default false, see below), `enable_gps`, `map_yaml_file`,
`use_rviz`. On the robot the `gripperx-mapping` service starts the SLAM variant.

### Yaw-rate channel of `odom_divergence_monitor` (step 2.6)

Second channel of the same node, **diagnostic only** (`/diagnostics`, status
"localization: yaw-rate cross-check"): it commands nothing, stops nothing and feeds
nothing back into the EKF. It compares the encoder yaw rate (`/wheel/odom`
`twist.angular.z`) with the gyro (`/imu/data/filtered`, rotated into the body z
axis through TF (coordinate-transform tree), because the IMU is mounted upside down).

- **Two stages (D17, 2026-11-13).** WARNING: the 1 s means differ by more than an absolute
  tolerance plus a relative term, for a sustain time; not latched. ERROR: the mean over a longer
  window of the excess beyond the relative term (clipped at zero) exceeds the absolute tolerance;
  **latched until the stack restarts**. Window lengths, tolerances and gains (the D9 tolerance and
  the D17 stages; all provisional, TO-VERIFY) live only in `config/localization.yaml`, section
  `odom_divergence_monitor`; they are not restated here. The latched state is published as JSON
  on `/localization/yaw_alarm` (fields include `latched`, `live_state`, `warning`, episode counts)
  and shown by the web teleop banner (`gripperx_teleop`).
- **Which `|rate|`:** the **gyro** yaw rate scales the relative term. The relative term covers
  the encoder's proportional over-report in spins. Scaling by the gyro, not the encoder, gives a
  gyro stuck low the tightest tolerance and stops a runaway encoder from widening its own.
- **Every episode is seen:** counted (`episodes`) and logged when it happens; the status
  stays at `DIVERGED_HELD` (ERROR) for `alarm_hold_sec` after it, so the 0.5 s tick
  cannot miss a short one.
- **Gaps:** a window with less than `min_window_fill` of data is not judged and the
  sustain counter is *frozen* (an alarm in progress is neither confirmed nor erased,
  and an active `DIVERGED` stays red). When more than `gap_report_fraction` of the last
  `gap_report_window_sec` is unjudgeable the state is `INTERMITTENT` (WARN), e.g. a
  partly failing IMU.
- **IMU silent (`IMU_SILENT`, WARN):** no accepted gyro sample for
  `imu_silence_timeout_sec`. Today the EKF then falls back to the encoder yaw rate
  silently. Also reported when the IMU never appears after `startup_grace_sec`, and
  when `localization_input_node` rejects every sample (see below).
  Its **return** is counted (`imu_returns`, `last_silence_sec`), logged, and the status
  is held at WARN for `return_hold_sec` whatever the sub-state; the averaging window is
  refilled after it. `WHEEL_SILENT`/`BOTH_SILENT` likewise. A gyro that first appears
  after the grace period (absent from boot, or the TF mount arrived late) is counted
  apart (`imu_first_appearances`, message "APPEARED") and does not raise `imu_returns`.
  **Known limit:** a stall of the monitor process itself (e.g. SIGSTOP for 4 s) looks like
  a silence and a return and is counted as one.
- **Time:** window and silence use arrival time on the node clock (`use_sim_time`
  honoured, the grace period starts at the first valid clock), not header stamps; stamps
  only drop stale (`max_sample_age_sec`) and out-of-order samples. A backward step, or a
  forward jump the monotonic clock did not make (`clock_step_tolerance_sec`; chrony on the
  RTC-less Pi), restarts the channel instead of reading as a silence plus a return.
- **Cannot see:**
  - a gyro stuck at zero (or any constant inside the tolerance) while the robot stands or
    turns slower than `yaw_abs_tol_dps`;
  - a gyro **frozen at a level the plausibility filter passes** (up to 3 rad/s) at standstill:
    the alarm fires within seconds and then **self-clears** about 30 s later
    (`gyro_bias_min_sec`), because the bias estimator learns the frozen value as bias and
    the conditioned output reads 0 again (shown by test; only the episode count/log and the
    held status remain);
  - a **real** rate above 3 rad/s: `localization_input_node` rejects the sample, the topic
    goes quiet and this channel says `IMU_SILENT` while the IMU is alive (inferred from the
    config note, not injected);
  - which source is wrong; errors below the tolerance. **Slip detection is `TO-VERIFY`:** no
    recording with known slip exists, only "quiet when clean" is shown.
- All values: `config/localization.yaml`, section `odom_divergence_monitor`.
  Tests: `test/check_yaw_divergence.py [bag_dir]` (logic + replay; reads that config; without a
  bag directory the replay part does NOT run and says so) and `test/check_yaw_divergence_node.py`
  (the node with rclpy, ~45 s, domain 99 on the laptop).

Laser odometry is computed and published on `/laser/odom` but is deliberately
**not** fused (`fuse_laser_odometry` stays false), so the monitor has an
independent second opinion at zero risk to the estimate.

## Known defects

- **OP-39 (open point 39) — `/wheel/odom` disagreed with the wheels by two orders of magnitude
  (2026-08-21).** The mechanism is explained and the fix is in the tree; the requirements entry
  still lists hardware verification as open, so treat `/odometry/filtered` as provisional and
  cross-check it against `odom_divergence_monitor`. The IMU is now connected and fused (yaw rate
  only), so wheel odometry is no longer the EKF's only yaw source.
- **The EKF lagged rotation badly before 2026-08-21.** Yaw and yaw-rate process
  noise were raised (0.08 → 0.15 and 0.05 → 0.50) after measurement: the filter
  had needed about six seconds to accept a rotation its own input reported from
  the first cycle, then kept publishing yaw rate after the wheels had stopped.
  That lag is what made the map lurch and snap back during turns. If turning
  behaviour regresses, check these two values first.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_localization
colcon test --packages-select gripperx_localization && colcon test-result --verbose
```

Desk tests only (`test/test_*.py` under pytest, `test/check_*.py` as plain scripts); none moves
hardware. The node test `check_yaw_divergence_node.py` needs rclpy and its own DDS domain.
