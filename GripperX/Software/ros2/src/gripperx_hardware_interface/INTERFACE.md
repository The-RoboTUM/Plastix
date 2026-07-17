# Hardware bridge contract (Pi ros2_control ↔ ESP32 firmware)

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/hw/joint_commands` | `std_msgs/Float64MultiArray` | Pi → firmware | ~100 Hz |
| `/hw/joint_states` | `std_msgs/Float64MultiArray` | firmware → Pi | 50–100 Hz |

## Array layout (8 values, fixed order)

### `/hw/joint_commands`

| Index | Joint | Unit | Meaning |
|---|---|---|---|
| 0 | `f_left_steer` | rad | steering setpoint |
| 1 | `f_right_steer` | rad | steering setpoint |
| 2 | `b_leftsteer` | rad | steering setpoint |
| 3 | `b_rightsteer` | rad | steering setpoint |
| 4 | `f_leftwheel` | rad/s | wheel velocity setpoint (PID on ESP32) |
| 5 | `f_rightwheel` | rad/s | wheel velocity setpoint |
| 6 | `b_leftwheel` | rad/s | wheel velocity setpoint |
| 7 | `b_rightwheel` | rad/s | wheel velocity setpoint |

### `/hw/joint_states`

| Index | Joint | Unit | Meaning |
|---|---|---|---|
| 0–3 | steering joints | rad | measured steering angle |
| 4–7 | wheel joints | rad/s | measured wheel velocity |

Wheel joint positions are not required for the current swerve stack.

## Bench testing

Launch with `use_mock_firmware:=true` (default) to run `hw_firmware_mock` instead of ESP32/micro-ROS.

When micro-ROS is ready on the ESP32, publish/subscribe the same topics and set `use_mock_firmware:=false`.
