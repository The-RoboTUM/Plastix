# ESP32 firmware (micro-ROS)

PlatformIO firmware for GripperX's ESP32. Publishes `/hw/joint_states` and listens on `/hw/joint_commands` via micro-ROS serial.

Contract compatible with `gripperx_hardware_interface` and `teleop_joint_commands_node` in the ROS 2 workspace.

## Requirements

- PlatformIO (`~/.platformio/penv/bin/pio`)
- ESP32 connected on `/dev/ttyUSB0` (adjustable in `platformio.ini`)
- ROS 2 **Jazzy** (`board_microros_distro = jazzy`)

## Flashing

```bash
sudo chmod 666 /dev/ttyUSB0
cd microros/firmware
pio run -e esp32dev -t upload
```

If you changed the ROS distro, clean micro-ROS first:

```bash
pio run --target clean_microros
pio run -e esp32dev -t upload
```

**Do not open the PlatformIO serial monitor while the agent is running.**

## Topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/hw/joint_commands` | `std_msgs/Float64MultiArray` | ROS 2 → ESP32 | ~100 Hz |
| `/hw/joint_states` | `std_msgs/Float64MultiArray` | ESP32 → ROS 2 | 100 Hz |

Firmware node: `/gripperx_firmware`

### Array layout (8 values)

| Index | Joint | Unit | ESP32 |
|--------|-------|--------|-------|
| 0–3 | steering | rad | ignored — steering servos are driven directly by the Pi, not the ESP32; echoed as 0.0 in `/hw/joint_states` |
| 4 | `f_leftwheel` | rad/s | motor 1 |
| 5 | `f_rightwheel` | rad/s | motor 2 |
| 6 | `b_leftwheel` | rad/s | motor 3 |
| 7 | `b_rightwheel` | rad/s | motor 4 |

## Manual test (without teleop)

With the agent running:

```bash
ros2 topic pub -r 10 /hw/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]}"

ros2 topic echo /hw/joint_states
```

See also: [`../TROUBLESHOOTING.md`](../TROUBLESHOOTING.md)
