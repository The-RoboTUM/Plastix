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
