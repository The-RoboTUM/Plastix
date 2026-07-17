# micro-ROS (ESP32 firmware)

ESP32 firmware bridging ROS 2 and the drivetrain motor hardware via
**micro-ROS over serial**.

## Structure

```text
microros/
├── firmware/                # PlatformIO project (ESP32 firmware)
│   ├── src/main.cpp
│   ├── src/motor_controller.cpp
│   ├── include/motor_controller.hpp
│   └── platformio.ini
├── TROUBLESHOOTING.md
└── README.md                # this file
```

## Firmware topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/hw/joint_commands` | `std_msgs/Float64MultiArray` | Pi → ESP32 |
| `/hw/joint_states` | `std_msgs/Float64MultiArray` | ESP32 → Pi |

Both topics carry an 8-value array (steering + drive per wheel). Details of
the array layout: [`firmware/README.md`](firmware/README.md).

## Troubleshooting

[`TROUBLESHOOTING.md`](TROUBLESHOOTING.md)
