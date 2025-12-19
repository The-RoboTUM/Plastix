# Octopus ROS2 Workspace

Multi-robot garbage collection coordination system.

## Quick Start

```bash
# Build
cd octopus_ws
colcon build

# Source
source install/setup.bash

# Run brain nodes
ros2 launch octopus_package octopus_main_launch.py

# Run a robot (in separate terminal)
ros2 run robot_package robot_controller --ros-args -p robot_name:=eve
ros2 run robot_package robot_controller --ros-args -p robot_name:=robby
ros2 run robot_package robot_controller --ros-args -p robot_name:=gripperx
ros2 run robot_package robot_controller --ros-args -p robot_name:=sharx
```

## Packages

| Package | Owner | Description |
|---------|-------|-------------|
| `octopus_msgs` | Brain Team | Message/service/action definitions |
| `octopus_package` | Brain Team | Central brain nodes |
| `robot_package` | All Robot Teams | Robot controller template |
| `perception_package` | Eve (Drone) Team | Image processing nodes |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/octopus/commands` | `String` | Commands from brain |
| `/octopus/robot_status` | `RobotStatus` | Status from robots |
| `/rtk_gps/locations` | `Location` | GPS data |
| `/task_db/active_tasks` | `TaskStatus` | Active tasks |
| `/drone/image/detection` | `String` | Drone detections |

## Services

| Service | Type |
|---------|------|
| `/octopus/get_robot_status` | `GetRobotStatus` |
| `/{robot_name}/get_status` | `GetRobotStatus` |
| `/task_db/create` | `CreateTask` |
| `/location_db/query` | `LocationQuery` |

## ROS2 Versions

# Test it with:
- ROS2 Humble
- ROS2 Jazzy

