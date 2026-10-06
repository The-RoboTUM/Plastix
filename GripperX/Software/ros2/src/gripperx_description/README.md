# gripperx_description

The robot model: URDF/Xacro, meshes and the RViz display configuration. No nodes.

## Two entry files, and the difference matters

| File | Use |
|---|---|
| `urdf/gripperx_v1.urdf.xacro` | **Real robot.** `ros2_control` is included only under `enable_ros2_control`, with plugin `gripperx_hardware_interface/GripperXInterface`. |
| `urdf/gripperx_v1.gazebo.xacro` | **Twin.** Always includes `ros2_control` as `gz_ros2_control/GazeboSimSystem`, adds the Gazebo IMU on `/imu/data`, and adds the odometry-publisher plugin that produces `/ground_truth/odom` with zero noise. |

Both pull the same macro libraries:

- `gripperx_v1.core.xacro` — the chassis: `base_footprint`, `base_link`,
  `chassis_link`, four steer links, four wheel links, `arm_stowed_link`,
  `imu_link`, and the geometry constants (`wheel_radius` 0.070, track 0.2174,
  axles at x ±0.1809, steer limits 1.745329 rad outward / 0.610865 rad inward).
- `gripperx_v1.lidar.xacro` — `lidar_link` and, in the twin, a ray sensor limited
  to ±160° so it reproduces the real LD06's ±20° blind rear wedge.
- `gripperx_v1.camera.xacro` — `camera_link` / `camera_optical_link`, optional
  Gazebo camera.
- `gripperx_v1.ros2_control.xacro` — the 4 position-commanded steer joints and 4
  velocity-commanded wheel joints, plus the `/hw/*` topic parameters.

## Launch

| Launch | What it does |
|---|---|
| `load_urdf.launch.py` | Runs `xacro` and starts `robot_state_publisher`. The workhorse everything else includes. Default `urdf_file` is the **gazebo** entry and default `use_sim_time` is **true**. |
| `rviz.launch.py` | Real-robot URDF + optional `joint_state_publisher_gui` + RViz on `rviz/display.rviz`. |
| `display.launch.py` | Backward-compatible alias for `rviz.launch.py`. |

```bash
ros2 launch gripperx_description display.launch.py
```

## Assets

`meshes/` holds 11 STL files, referenced as
`package://gripperx_description/meshes`. `rviz/display.rviz` is the only RViz
config.

`scripts/decimate_meshes.py` is **not installed** by `CMakeLists.txt` and is not a
ROS node — it is a standalone offline tool requiring `fast_simplification`, which
is not declared as a dependency.

## Housekeeping

`package.xml` still carries placeholder `TODO` values for description, license and
maintainer.
