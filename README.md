# ITQ ROS 2 workspace

This repository is a [ROS 2](https://docs.ros.org/) workspace. Packages live under `src/`. After building with `colcon`, outputs appear under `build/`, `install/`, and `log/` (do not commit those as source code).

The sections below describe **what belongs in each package** so the stack stays clear: one concern per package, with `robot_bringup` wiring everything together.

---

## `robot_description`

**Role:** Canonical physical description of the robot for simulation, visualization, kinematics, and tools that consume a robot model.

**Typical contents**

- **URDF and/or Xacro** (`urdf/`, sometimes `robots/`): joints, links, inertial tags, gazebo tags if needed.
- **Meshes and assets** (`meshes/`): STL/DAE and textures referenced by the URDF/Xacro.
- **RViz / model-oriented resources** (`rviz/`): robot-centric displays (optional; can live here or in bringup).

**Usually avoid here**

- Long-running navigation or control nodes (those belong in other packages).

**Dependencies**

- Often depended on by `robot_bringup`, `robot_controller`, `robot_mapping`, and `robot_localization` via `find_package()` or launch file references to shared Xacro.

---

## `robot_controller`

**Role:** Motion control stack: [`ros2_control`](https://control.ros.org/) configuration, controllers, and (if applicable) hardware or simulation interfaces.

**Typical contents**

- **Hardware or mock interfaces** (`src/`): C++ plugins for custom hardware/bringup bridges when you are not using an off-the-shelf package-only setup.
- **Controller configuration** (`config/`): YAML for `joint_trajectory_controller`, `diff_drive_controller`, `imu_sensor_broadcaster`, etc.
- **Launch files that only start controllers** (`launch/`): optional small launches included from bringup.

**Usually avoid here**

- Full-stack “start the whole robot” launches (prefer `robot_bringup`).
- URDF/Xacro source files (keep those in `robot_description`).

---

## `robot_localization`

**Role:** Fuse sensor and odometry data into a consistent world- or map-frame estimate (often using [`robot_localization`](http://wiki.ros.org/robot_localization) EKF/UKF or similar).

**Typical contents**

- **Filter parameters** (`config/`): YAML for IMU integration, GPS (if used), odometry inputs, frames (`odom`, `base_link`, etc.).
- **Launches** (`launch/`): start the estimator with the right remaps.

**Usually avoid here**

- SLAM occupancy grid builds and lifelong mapping (those go in `robot_mapping`).

**Dependencies**

- Commonly relies on sensor topics produced after the robot is driving; coordinate frame names must match what `robot_description` and TF publish.

---

## `robot_mapping`

**Role:** Build and maintain maps: SLAM, occupancy grids, and mapping-specific parameters.

**Typical contents**

- **Mapper configuration** (`config/`): params for Slam Toolbox, Cartographer, ORB-SLAM wrappers, etc.
- **Mapping launches** (`launch/`): online mapping, map saving, playback from bags if needed.

**Usually avoid here**

- EKF fusion config (unless a launch here needs to *call into* localization; clearer to keep YAML in `robot_localization`).

---

## `robot_bringup`

**Role:** Entry point for operators: launches that combine description, controllers, localization, navigation/mapping stacks, RViz, and simulation (Gazebo/Ignition) if you use it.

**Typical contents**

- **Top-level launch files** (`launch/`): e.g. `bringup.launch.py` that includes other packages’ launches with consistent args (`use_sim_time`, namespaces).
- **Environment-specific overrides** (`config/`): only when something is genuinely “how we start the full system,” not duplicated controller or mapper YAML owned elsewhere.

**Design guideline**

- Keep `robot_bringup` thin: orchestration and defaults, not the only place holding URDF or low-level controller parameters.

---

## sensors

**Role:** All Sensor nodes with a single launch file that would startup all sensor publishers. The Sensor Nodes include Camera, GPS, Wheel Encoders, IMU, Lidar and anything else that the team decides.

**Typical contents** 

- ** ** (`launch/`): e.g. `bringup.launch.py` that includes other packages’ launches with consistent args (`use_sim_time`, namespaces).
- **

**Design guideline**

- Keep `sensors` thin: orchestration and defaults, not the only place holding URDF or low-level controller parameters.


---

## Layout on disk

All of these packages are **siblings** under `src/`. `robot_bringup` is not a folder that contains the others; it is the same kind of ROS package as the rest:

```text
src/
├── robot_bringup/
├── robot_controller/
├── robot_description/
├── robot_localization/
└── robot_mapping/
└── sensors/
```

## What `robot_bringup` does at runtime

When you run a top-level launch from `robot_bringup`, it usually **includes** launch files from the other packages (for example via `IncludeLaunchDescription` and each package’s `share/` install path). That is an orchestration relationship, not a directory hierarchy:

- **robot_description** — models, meshes  
- **robot_controller** — `ros2_control`, controllers  
- **robot_localization** — state estimation  
- **robot_mapping** — SLAM / maps when needed  

Declare in each `package.xml` only what that package actually needs to build or install; `robot_bringup` adds `exec_depend` (or similar) on the packages whose launches or resources it references.

---

## Build

```bash
cd /home/celeste/itq_ros2
source /opt/ros/<DISTRO>/setup.bash   # replace <DISTRO> with your ROS 2 distro, e.g. jazzy
colcon build
source install/setup.bash
```

Replace the path above with your clone location if different.
