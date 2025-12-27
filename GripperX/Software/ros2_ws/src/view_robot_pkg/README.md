# view_robot_pkg
## Visualize SolidWorks-based URDF versions in RViz (ROS 2)

This ROS 2 package is used to visualize and compare multiple URDF versions of the same robot, exported from SolidWorks, in RViz2.  
It is intended as a small URDF validation workspace during mechanical design iterations.

The launch files start:
- robot_state_publisher (publishes TF from the URDF)
- joint_state_publisher_gui (interactive joint sliders)
- rviz2 (preconfigured view)
 --- 
## How to Use

Build the workspace (from the workspace root):
```bash
colcon build --symlink-install
source install/setup.bash
```
Launch one of the versions:
```bash
ros2 launch view_robot_pkg view_v1.launch
ros2 launch view_robot_pkg view_v2.launch
ros2 launch view_robot_pkg view_v3.launch
```
---
## URDF versions

- ### Version 1: Chassis + wheels (rotation only)
    - **Visible**: chassis and wheels

    - **Joints**: only continuous joints at the wheel rotation axes

- ### Version 2: Full suspension geometry (rigid)

    - **Visible**: chassis, wheels, suspension geometry

    - **Joints**: same as Version 1

    - **Suspension** is rigidly attached to the chassis (visual only)

- ### Version 3: Suspension geometry + linear wheel travel

    - **Visible**: same as Version 2

    - **Joints**: continuous wheel rotation joints ***plus*** an additional prismatic joint in z-direction at each wheel
---
## Repository structure (conceptual)
```text
view_robot_pkg/
├── launch/
│   ├── view_v1.launch
│   ├── view_v2.launch
│   └── view_v3.launch
├── urdf/
│   ├── v1_chassis_wheels.urdf
│   ├── v2_full_suspension_rigid.urdf
│   └── v3_suspension_with_linear_wheel.urdf
├── meshes/
│   ├── v1_chassis_wheels/
│   ├── v2_full_suspension_rigid/
│   └── v3_suspension_with_linear_wheel/
├── rviz/
│   └── default_view.rviz
├── resource/
│   └── view_robot_pkg
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---
## Dependencies
```bash
sudo apt install ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-robot-state-publisher
```
---
## Related YouTube Video
#### Explains how to convert SolidWorks to URDF for ROS2 & How to use this package.
#### Link: https://www.youtube.com/watch?v=JdZJP3tGcA4&feature=youtu.be

