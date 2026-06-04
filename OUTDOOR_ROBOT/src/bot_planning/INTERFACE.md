# Contract between `bot_localization` and `bot_planning`

Packages are split for parallel development. **`bot_gazebo` only runs the simulator** (robot + sensors + control); start localization and planning separately.

## `bot_gazebo` owns

| Responsibility | Main outputs |
|----------------|--------------|
| Gazebo world + robot spawn | `/joint_states`, `/imu/data`, `/scan`, `/camera/*` |
| Swerve command path | `/cmd_vel` → joint commands |

**Launch:** `ros2 launch bot_gazebo simulation.launch.py`

## `bot_localization` owns

| Responsibility | Main outputs |
|----------------|--------------|
| Wheel odometry from joint states | `/wheel/odom` |
| EKF sensor fusion | `/odometry/filtered`, TF `odom` → `base_footprint` |
| Laser odometry (optional) | `/laser/odom` |
| Mapping (optional) | `/map`, TF `map` → `odom` via **slam_toolbox** |
| Saved-map pose (optional) | `/map`, TF `map` → `odom` via **AMCL** + `map_server` |

**Launch:** `ros2 launch bot_localization localization.launch.py`

## `bot_planning` owns

| Responsibility | Main outputs |
|----------------|--------------|
| Nav2 planner, controller, recovery | `/cmd_vel` from Nav2 |

**Requires:** localization running (map + `map`→`odom` + `/odometry/filtered` + `/scan`).

**Does not use directly:** `/laser/odom` or `ros2_laser_scan_matcher` — laser odometry is started only in `bot_localization` (`enable_laser_odometry`) and fused into `/odometry/filtered` by the EKF.

**Launch:** `ros2 launch bot_planning navigation.launch.py use_sim_time:=true`

## Typical simulation (4 terminals)

```bash
# 1 — sim
ros2 launch bot_gazebo simulation.launch.py

# 2 — localization (navigate in saved map)
ros2 launch bot_localization localization.launch.py \
  enable_slam:=false enable_saved_map_localization:=true enable_laser_odometry:=true \
  map_yaml_file:=$(pwd)/maps/arena_map.yaml

# 3 — planning
ros2 launch bot_planning navigation.launch.py use_sim_time:=true

# 4 — RViz (optional)
ros2 launch bot_localization localization.launch.py  # RViz included by default
```
