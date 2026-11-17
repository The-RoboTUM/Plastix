# gripperx_gazebo

Gazebo sim (the digital twin): world, spawn, `ros2_control`, sensor bridge, the swerve controller
and the teleop mux, with launch files for sim-only, sim plus SLAM mapping and sim plus Nav2.

For: developers who test control, localization and navigation on the desk. Status: 2026-11-17.
The twin is for geometry, topic wiring and control logic; it does not transfer load-sensitive
results (see "Known defect" below).

Short forms: DT-n (digital-twin work items in the requirements), EKF (extended Kalman filter, the
`robot_localization` fusion), IMU (inertial sensor), SLAM (simultaneous localization and mapping).

**Does not** launch localization, AMCL, EKF, or Nav2 — that goes in
`gripperx_localization` and `gripperx_planning`.

## Launch files

| Launch | What it starts |
|---|---|
| `simulation.launch.py` | Sim only (gz-sim, spawn, controllers, `teleop_mux`); optional `use_lidar`, `use_camera`, `use_rviz`, `world_file`, `headless`, `initial_mode`. |
| `sim_mapping.launch.py` | Sim plus `slam_toolbox` (async mapping), ground-truth odometry passed through as TF (coordinate transform) `odom->base_footprint`. |
| `sim_navigation.launch.py` | One launch for the integrated stack: sim plus localization plus Nav2, with `use_sim_time:=true` throughout. Arguments: `odom_source` (`ground_truth` default, or `ekf`), `localization` (`slam` default, or `amcl` with `map_yaml_file`), `enable_laser_odometry`, `fuse_laser_odometry`, `world_file`, `headless`, spawn pose arguments. |
| `simulate_robot.launch.py`, `spawn_robot.launch.py` | Building blocks the above include (gz bridge, spawn, controller spawners, mux). |

`teleop_mux` starts in `keyboard` mode (`initial_mode`); Nav2 output reaches `/cmd_vel` only after
`/teleop/set_mode` is set to `autonomous`, as on the real robot. The sim IMU appears as
`/imu/bno085/data`, the same name as the real BNO085 driver, so one localization configuration
serves both (`config/gz_bridge.yaml`). In the twin the EKF runs the robot's own filter
configuration; its IMU noise is white only (no bias or drift), so IMU-robustness results do not
transfer.

## Prerequisites (Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/install_jazzy_sim_deps.sh
```

## Worlds

- `worlds/testworld_v1.world.sdf` — **default**. A freely invented, 12x12 m
  Nav2 tuning world: enclosed outer walls, a central north-south corridor
  with a narrow section (chicane, ~0.9 m remaining width), four rooms
  (NW/SW/NE/SE) with door openings, scattered columns/boxes/table.
- `worlds/empty.world.sdf` — empty world (for teleop/physics tests).
- `worlds/testworld_simple.world.sdf` — reduced version of the tuning world.
- `worlds/demo_area_2m5.world.sdf` — 2.5 m demo area.

## Current 3-terminal set

Real servo steering in the sim (`sim_steer_bridge`, see below) plus SLAM mapping,
in one Gazebo GUI window with a standalone RViz. Every terminal starts with the
isolated sim environment — never the plain `~/.bashrc` env (a dedicated
`ROS_DOMAIN_ID` keeps the simulation isolated from the real robot's domain):

```bash
source Software/ros2/scripts/sim_env.sh
```

Around every run: check `ros2 node list` for leftovers before starting, and do a
PID-exact teardown (individual `kill -9`, then `ros2 daemon stop`) after — a plain
Ctrl-C is not sufficient (orphaned sim nodes are the
usual survivors).

**Terminal 1 — sim + GUI + SLAM:**

```bash
source Software/ros2/scripts/sim_env.sh
ros2 launch gripperx_gazebo sim_mapping.launch.py headless:=false
```

`headless:=false` starts gz-sim WITHOUT `-s` (server+GUI in one process) — needs a
desktop session/`$DISPLAY`. This also starts `slam_toolbox` (async, `mapping`
mode).

**Terminal 2 — RViz standalone (NOT via `use_rviz:=true`):**

```bash
source Software/ros2/scripts/sim_env.sh
rviz2 -d install/gripperx_localization/share/gripperx_localization/rviz/localization.rviz
```

RViz started via a launch argument (`use_rviz:=true`) does **not reliably
start** — a known flakiness issue. Standalone in its own terminal is the
current accepted practice.

**Terminal 3 — teleop:**

```bash
source Software/ros2/scripts/sim_env.sh
ros2 launch gripperx_teleop laptop_teleop.launch.py
```

Do **not** set `publish_steer_cmd_vel:=true` on `keyboard_teleop_node` — its
default (`false`) is required here, and the sim mux runs with `keyboard_pass_angular_z` false like
the real robot. Enabling that option would feed the steering angle back in as `cmd_vel.angular.z`
through the swerve inverse kinematics: double steering, with the wheels fighting each other.
(`sim_steer_bridge` is retired from the active path; arbitration of the steering now lives in
`swerve_controller`, one implementation shared by real robot and sim. Comments in
`launch/simulate_robot.launch.py` still mention the bridge in one place.)

Keys: **W/S** drive forward/backward, **A/D** steer, **Space** emergency stop (stop + straight ahead +
back to keyboard mode).

Save the map once mapping looks complete:

```bash
source Software/ros2/scripts/sim_env.sh
ros2 run nav2_map_server map_saver_cli -f <path>/<name> --ros-args -p save_map_timeout:=10.0
```

## Older/manual variants

- Sim only, no SLAM (headless, default world):
  ```bash
  source install/setup.bash
  ros2 launch gripperx_gazebo simulation.launch.py
  ```
  Optional: `use_lidar:=false`, `use_camera:=false`, `use_rviz:=true`,
  `world_file:=<path>`, `headless:=false`.
- `sim_mapping.launch.py` deliberately does not use the full EKF chain from
  `gripperx_localization/launch/localization.launch.py` — instead, ground-truth
  odometry is passed through directly as TF `odom->base_footprint` via
  `ground_truth_odom_bridge` (`gripperx_localization`).
- `scripts/sim_env_dt4.sh` predates `Software/ros2/scripts/sim_env.sh` and is now superseded
  by it for normal use.
- Full stack (localization + Nav2) once `ros-jazzy-robot-localization` is installed:

  | Terminal | Package | Launch |
  |----------|---------|--------|
  | 2 | `gripperx_localization` | `localization.launch.py` (+ RViz by default) |
  | 3 | `gripperx_planning` | `navigation.launch.py` |

See [`../gripperx_localization/README.md`](../gripperx_localization/README.md).

## Known defect — the twin is not weight-true

**The twin has been standing 40 % lopsided in every run ever made.** Total
simulated mass is 3.72 kg of bare CAD shell: `chassis_link` carries 2.27 kg with
no battery, Pi, motors or electronics, and `arm_stowed_link` and `imu_link` weigh
one gram each. The real mass and centre of gravity are not documented anywhere.
There is also not one friction or contact parameter in the whole workspace — no
`mu`, `mu2`, `slip`, `kp/kd`, no `<surface>` — so every tyre is Gazebo default on
a smooth cylinder.

Consequence: **any load-sensitive twin result is tainted.** Traction, tipping,
slip, motor effort and anything derived from them do not transfer to the robot.
Geometry, topic wiring and control logic do transfer, and that is what the twin is
for.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_gazebo
```

The package has no tests of its own; the twin is exercised by running the launch files above and
by the checks in `gripperx_localization` and `gripperx_planning`. A sim run moves only simulated
hardware, but always run it in the isolated sim environment (`Software/ros2/scripts/sim_env.sh`), never in the
robot's `ROS_DOMAIN_ID`.
