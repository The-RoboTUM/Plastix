# micro-ROS (ESP32 firmware)

ESP32 firmware bridging ROS 2 and the drivetrain motor hardware via
**micro-ROS over serial**.

## Structure

```text
microros/
├── firmware/                # PlatformIO project (ESP32-S3 firmware)
│   ├── src/main.cpp
│   ├── src/motor_controller.cpp
│   ├── src/quad_encoder.cpp
│   ├── include/motor_controller.hpp
│   ├── include/quad_encoder.hpp
│   └── platformio.ini       # envs: esp32-s3 (production), pintest (bench)
├── TROUBLESHOOTING.md
└── README.md                # this file
```

## Firmware topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/hw/joint_commands` | `std_msgs/Float64MultiArray` | Pi → ESP32 |
| `/hw/joint_states` | `std_msgs/Float64MultiArray` | ESP32 → Pi |

The two arrays are **different lengths** — do not assume symmetry:

- `/hw/joint_commands` — **8** values: 4 steer positions, 4 wheel velocities.
  The ESP32 consumes only 4–7; the steering servos hang on the Pi's USB bus.
- `/hw/joint_states` — **16** values: 0–3 reserved and **always zero** from this
  firmware (the ESP32 has no steering sensor — real steering feedback is
  `/hw/steer_states` from `steer_servo_node`), 4–7 wheel velocities, 8–11 wheel
  positions from the encoders, 12–15 one `EncoderStatus` provenance code per
  wheel saying whether 4–7 / 8–11 are measured or an echo of the command
  (FR-11 items 5/6).

Corrected 2026-08-24 against `firmware/src/main.cpp` (`NUM_CMD_JOINTS 8`,
`NUM_STATE_JOINTS 16`); the normative contract is
[`gripperx_hardware_interface/INTERFACE.md`](../ros2/src/gripperx_hardware_interface/INTERFACE.md).
Anything appended must go **after** index 15 — the Pi keys its length guards on
8 / 12 / 16 and reads a shorter message as "provenance unknown", never as valid.

## Troubleshooting

[`TROUBLESHOOTING.md`](TROUBLESHOOTING.md)
