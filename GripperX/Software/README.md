# Software

Software components for GripperX, split into three independent parts.

| Folder | Content |
|---|---|
| [`ros2/`](ros2/README.md) | ROS 2 Jazzy colcon workspace — all ROS packages, maps, and utility scripts. Used both for the Gazebo simulation (laptop) and the real robot (Raspberry Pi 5). |
| [`microros/`](microros/README.md) | ESP32 firmware (PlatformIO project) running micro-ROS, bridging ROS 2 and the drivetrain motors. |
| [`pi_env/`](pi_env/README.md) | Reference snapshot of the Raspberry Pi's system environment (systemd units, udev rules, network/OS/package info) — not a workspace, kept for reproducing a fresh Pi setup. |
