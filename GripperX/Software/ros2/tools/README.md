# tools — diagnostic scripts for the real robot

Standalone `rclpy` scripts. They are **not** a ROS package: run them with `python3`, not
`ros2 run`. Each is self-contained and each states its purpose, its method and its caveats in its
own docstring — read that before running it, the docstrings carry the reasoning.

```bash
export ROS_DOMAIN_ID=20                 # real robot; 220 for the twin
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
python3 Software/ros2/tools/watch_wheels.py
```

The three environment variables are **not optional**. Without them a tool sees an empty graph and
reports nothing rather than failing — several of these scripts once produced confident results off a
stationary or two-wheel robot before guards were added. The checks are also code.

---

## ⚠ SR-1 — eight of these ten command the drive

**SR-1 applies: no movement of drivetrain, steering or arm without explicit user approval, per test.**
That includes every script in the "commands the drive" group below, *including the ones that expect
no visible motion*. `floor_check.py` deliberately commands below the breakaway threshold and
`bench_check.py` deliberately chooses an angular rate below it — **the drive is still commanded**,
and "no motion expected" is a prediction, not a guarantee.

Read the table before you run anything.

| Script | Commands | Purpose |
|---|---|---|
| `watch_wheels.py` | **nothing — read-only** | Turn each wheel **by hand** and watch which encoder counts. Separates "the wheel is not driven" from "the encoder is not counting". The safest place to start on an unfamiliar robot |
| `steer_straight.py` | **steering only** (`/teleop/direct_steer`); publishes nothing on `/cmd_vel` | Steer all four wheels to 0° and report. The override is freshness-gated (0.50 s), so it must be republished — one participant does both the publishing and the measuring |
| `odom_check.py` | **drive** (`Twist`) | Does wheel odometry decode a straight drive as a straight drive? |
| `odom_check2.py` | **drive** (`Twist`) | **Preferred over `odom_check.py`.** Captures what the wheels did, what `/joint_states` carries and what odometry says in **one** run, so the three cannot disagree about which run they describe |
| `floor_check.py` | **drive** (`Twist`), below the breakaway threshold | Commands below `stall_min_command_rad_s` and verifies every wheel reports `REGULATOR_OFF_BELOW_FLOOR=7` with a correction of exactly 0.0 |
| `bench_check.py` | **drive + steering**, angular rate below breakaway | Checks the commanded contact-point correction against its design without the robot travelling. Catches the one failure mode that would make the correction worse than nothing: applying it *before* the ±180° module fold |
| `a13_disable_while_driving.py` | **drive** (`Twist`) | Disables the wheel regulator **while driving** — the acceptance criterion for the runtime switch, which every earlier disable avoided by happening on blocks or after the run |
| `stopping_distance.py` | **drive** (`Twist`) | Commands a steady speed, cuts it with an exact-zero twist (what Nav2 does at a goal, i.e. what the Octopus link actually experiences), and reads the encoder position delta until standstill. Writes every sample incrementally, never buffers a run |
| `yaw_shortfall.py` | **drive + steering** | The yaw-rate shortfall measurement for the spin half of `OP-29` |
| `crab_yaw.py` | **drive + steering** | Was written to test whether the diagonal wheel fold produces a yaw couple in crab. **Its hypothesis has since been REFUTED** — see below |

## Which open point each one belongs to

| Script | Open point |
|---|---|
| `odom_check.py`, `odom_check2.py` | `OP-39` — `/wheel/odom` disagreed with the wheels by two orders of magnitude (measurement of 2026-08-21, cause not established). **Record the live `drive_joint_multipliers` value with every run** — it changed from `[1,1,-1,-1]` to `[1,1,1,1]` on 2026-08-21, and a bag that does not say which was live is not comparable to earlier ones |
| `yaw_shortfall.py`, `bench_check.py` | `OP-29`, spin half — repaired and confirmed on hardware; the remaining ~5.9 % is a separate drivetrain shortfall |
| `crab_yaw.py` | `OP-29`, crab half. **The hypothesis this script tests is refuted.** The diagonal fold is innocent and so is `manoeuvre.py`; the cause is mechanical play in the steering drive train (`HWR-41` / `OP-H15`), which the user placed out of scope. **Do not re-run it expecting an answer** — it is kept because the method is sound and the negative result is evidence |
| `floor_check.py`, `a13_disable_while_driving.py` | `FR-14` — wheel regulator, items 12 and A13 |
| `stopping_distance.py` | The Octopus link's arrival behaviour (`FR-12`) |
| `watch_wheels.py`, `steer_straight.py` | None — general instruments |

## Caveat on `stopping_distance.py`

Three throwaway patch scripts (`patch_yaw.py`, `patch_stopping.py`, `patch_status.py`) existed
alongside these and rewrote sibling scripts **in place** at a hard-coded absolute path. They are
deliberately **not** part of this directory. One consequence survives them: the committed state of
`stopping_distance.py` may or may not already include the patches they applied (an `HWR-30a` latch
abort, alternating direction, absolute distance). Check its content against what you expect before
trusting a run.
