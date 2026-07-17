# gripperx_planning

Nav2: planner, holonomic controller (DWB), behaviors, BT navigator.

**Does not** include EKF, SLAM, or AMCL — launch **`gripperx_localization`** first.

## Simulation (terminal 3, after Gazebo + localization)

```bash
source install/setup.bash
ros2 launch gripperx_planning navigation.launch.py use_sim_time:=true
```

Nav2 publishes `/cmd_vel` → `swerve_cmd_node` (started in `gripperx_gazebo`).

## Goal in RViz

**2D Goal Pose** (RViz already open from `gripperx_localization/localization.launch.py`).

## Real robot

```bash
ros2 launch gripperx_planning navigation.launch.py use_sim_time:=false
```

(with localization and sensors running).

Config: `config/nav2.yaml`. Behavior tree (replan + recovery, [Nav2 walkthrough](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html#navigate-to-pose-with-replanning-and-recovery)): `config/navigate_to_pose_w_replanning_and_recovery.xml` — used by default in `navigation.launch.py`. Contract: `INTERFACE.md`.
