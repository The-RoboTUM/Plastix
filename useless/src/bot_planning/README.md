# bot_planning

Nav2: planner, controller holonómico (DWB), behaviors, BT navigator.

**No** incluye EKF, SLAM ni AMCL — lanza **`bot_localization`** antes.

## Simulación (terminal 3, tras Gazebo + localización)

```bash
source install/setup.bash
ros2 launch bot_planning navigation.launch.py use_sim_time:=true
```

Nav2 publica `/cmd_vel` → `swerve_cmd_node` (arrancado en `bot_gazebo`).

## Objetivo en RViz

**2D Goal Pose** (RViz ya abierto desde `bot_localization/localization.launch.py`).

## Robot real

```bash
ros2 launch bot_planning navigation.launch.py use_sim_time:=false
```

(con localización y sensores en marcha).

Config: `config/nav2.yaml`. Behavior tree (replan + recovery, [Nav2 walkthrough](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html#navigate-to-pose-with-replanning-and-recovery)): `config/navigate_to_pose_w_replanning_and_recovery.xml` — usado por defecto en `navigation.launch.py`. Contrato: `INTERFACE.md`.
