# gripperx_localization

EKF, wheel odometry, laser odom, SLAM, AMCL, and **RViz** (a single launch).

Nav2 → **`gripperx_planning`**.

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

`localization_input_node` (fuses `/joint_states` + `/imu/data` into `/wheel/odom`
and `/imu/data/filtered`), `ground_truth_odom_bridge` (twin only), and
`odom_divergence_monitor`, which compares two odometry sources and reports on
`/diagnostics`.

Laser odometry is computed and published on `/laser/odom` but is deliberately
**not** fused (`fuse_laser_odometry` stays false), so the monitor has an
independent second opinion at zero risk to the estimate.

## Known defects

- **`OP-39` — `/wheel/odom` disagreed with the wheels by two orders of magnitude
  (2026-08-21).** It is the EKF's `odom0` and, with no IMU currently connected, its
  single motion source. The mechanism is fully explained and **the fix is already
  in the tree**; what remains is verification on hardware, scheduled 2026-08-25.
  Until that verification lands, treat `/odometry/filtered` as provisional and
  cross-check it against `odom_divergence_monitor` rather than trusting it alone.
  *(The `OP-39` entry itself lives on the unmerged `Theo-req-audit` branch and
  becomes resolvable in `REQUIREMENTS` when that branch merges.)*
- **The EKF lagged rotation badly before 2026-08-21.** Yaw and yaw-rate process
  noise were raised (0.08 → 0.15 and 0.05 → 0.50) after measurement: the filter
  had needed about six seconds to accept a rotation its own input reported from
  the first cycle, then kept publishing yaw rate after the wheels had stopped.
  That lag is what made the map lurch and snap back during turns. If turning
  behaviour regresses, check these two values first.
