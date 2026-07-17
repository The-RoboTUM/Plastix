# gripperx_bringup

Launches for the real robot.

## Teleop (recommended for getting started)

```bash
ros2 launch gripperx_bringup teleop_real.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # another terminal
```

## Layers

| Launch | What it starts |
|---|---|
| `teleop_real.launch.py` | micro-ROS agent + bridge `/cmd_vel` → `/hw/joint_commands` |
| `real_robot.launch.py` | URDF, ros2_control, swerve, firmware mock |
| `real_autonomy.launch.py` | real_robot + sensors + localization + optional Nav2 |

## Typical modes (advanced)

```bash
# Base robot + ros2_control
ros2 launch gripperx_bringup real_robot.launch.py use_mock_firmware:=false

# Full stack + Nav2
ros2 launch gripperx_bringup real_autonomy.launch.py \\
  use_mock_firmware:=false use_mock_sensors:=false \\
  enable_navigation:=true use_rviz:=true
```

Simulation: `ros2 launch gripperx_gazebo simulation.launch.py`
