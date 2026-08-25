# gripperx_planning

Nav2: planner, holonomic controller (DWB), behaviors, BT navigator.

**Does not** include EKF, SLAM, or AMCL — launch **`gripperx_localization`** first.

## Simulation (terminal 3, after Gazebo + localization)

```bash
source install/setup.bash
ros2 launch gripperx_planning navigation.launch.py use_sim_time:=true
```

Nav2 publishes `/cmd_vel`, which reaches the drive chain through `teleop_mux`
(**only** while it is in `autonomous` mode) and is consumed by
`swerve_controller` from `gripperx_swerve_controller`. There is no
`swerve_cmd_node` in the path any more.

## Goal in RViz

**2D Goal Pose** (RViz already open from `gripperx_localization/localization.launch.py`).

## Real robot

```bash
ros2 launch gripperx_planning navigation.launch.py use_sim_time:=false
```

(with localization and sensors running).

Config: `config/nav2.yaml`. Behavior tree (replan + recovery, [Nav2 walkthrough](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html#navigate-to-pose-with-replanning-and-recovery)): `config/navigate_to_pose_w_replanning_and_recovery.xml` — used by default in `navigation.launch.py`. Contract: `INTERFACE.md`.

## Recovery behaviours

`config/nav2.yaml` loads four: `drive_on_heading`, `backup`, `crab_walk`, `wait`.
`crab_walk` is this project's own plugin (`gripperx_behaviors`) — a bounded
pure-lateral escape. Both behaviour trees invoke it. It is the least
field-exercised part of the stack; if autonomy misbehaves during recovery, suspect
it first.

## Known defects

- **The ~5.9 % drivetrain rotation shortfall consumes the yaw tolerance** on turns
  beyond roughly 90°. Goal checkers tuned as if rotation were delivered in full
  will time out on large heading changes. See
  `gripperx_swerve_controller/README.md`.
- Nav2 has been verified in the twin far more than on the robot. Treat twin
  success as necessary, not sufficient — and note the twin's own load defect in
  `gripperx_gazebo/README.md`.
