# PlastiX

Multi-robot garbage collection system.

## Teams

| Team | Robot | Package |
|------|-------|---------|
| Eve (Drone) | Aerial drone | `robot_package` + `perception_package` |
| Robby | Small item collector | `robot_package` |
| GripperX | Large item collector | `robot_package` |
| SharX | Water collector | `robot_package` |
| Brain (Octopus) | Coordination | `octopus_package` |

## Quick Start

```bash
cd octopus_ws
colcon build
source install/setup.bash
ros2 launch octopus_package octopus_main_launch.py
```

## Structure

```
octopus_ws/
└── src/
    ├── octopus_msgs/       # Message definitions
    ├── octopus_package/    # Brain nodes
    ├── robot_package/      # Robot template
    └── perception_package/ # Drone perception
```

See [`octopus_ws/README.md`](octopus_ws/README.md) for details.
