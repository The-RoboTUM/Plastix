# Hardware bridge contract (Pi ros2_control ↔ ESP32 firmware)

## Topics

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/hw/joint_commands` | `std_msgs/Float64MultiArray` | Pi → firmware | ~100 Hz |
| `/hw/joint_states` | `std_msgs/Float64MultiArray` | firmware → Pi | **30 Hz** (measured 29.999 Hz on hardware 2026-08-20; was 50–100 Hz here, which was never true — the firmware published a measured 8.5 Hz until the 2026-08-20 rate rebuild) |
| `/hw/steer_states` | `std_msgs/Float64MultiArray` | `steer_servo_node` → `read()` | 30 Hz |
| `/hw/steer_states_valid` | `std_msgs/Bool` | `GripperXInterface` → health readout | on change, latched |
| `/hw/wheel_feedback_valid` | `std_msgs/Int32MultiArray` | `GripperXInterface` → health readout | on change, latched |

## Array layout (fixed order)

`/hw/joint_commands` is always 8 values. `/hw/joint_states` is **at least** 8 values;
firmware with encoder feedback appends 4 wheel positions, and firmware with the
provenance block appends 4 more codes after those (see below). `read()` rejects anything
shorter than 8 and ignores anything beyond 16, so old and new publishers interoperate.

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
| 0–3 | steering joints | — | **reserved, always 0.0 — read() ignores them** |
| 4–7 | wheel joints | rad/s | measured wheel velocity |
| 8–11 | wheel joints | rad | accumulated wheel position (encoder firmware only) |
| 12–15 | wheel joints | code | provenance of 4–7 / 8–11 (provenance firmware only) |

Wheel order is the same in every block: FL, FR, BL, BR.

**Indices 0–3 are reserved and always 0.0 (FR-10).** The steering servos hang off the Pi,
not the ESP32, so the firmware has nothing to measure and structurally never will —
anything it could put there would be an echo of the command, which is what FR-2 rejects as
feedback. `read()` therefore **ignores** these four values entirely; it does not copy them
into any state interface. `hw_firmware_mock` publishes zeros here as well (it used to
model a first-order lag, which made the bench better-behaved than the machine and hid this
exact divergence).

### `/hw/steer_states` — the steering measurement (FR-10)

4 values in rad, joint order FL, FR, BL, BR (`f_left_steer`, `f_right_steer`,
`b_leftsteer`, `b_rightsteer`) — identical to the block order above, no remapping.
Published by `steer_servo_node` at `control_rate_hz: 30.0` from the servos' own position
readback. `GripperXInterface::read()` merges it into the four steering **position** state
interfaces.

* **QoS (subscriber, explicit):** `KEEP_LAST(10)`, `RELIABLE`, `VOLATILE` — matched to the
  rclpy default publisher profile in `steer_servo_node`. Not `SystemDefaultsQoS()`, which
  resolves to `BEST_EFFORT` under `rmw_fastrtps` on this stack.
* **Freshness:** `steer_states_timeout_sec` (default 0.5 s, carried over from
  `swerve_cmd_node`).
* **Stale or absent:** the last valid measurement is **held** — never replaced by 0.0 —
  a throttled `ERROR` is logged, and `/hw/steer_states_valid` goes `false`. `read()` still
  returns `OK`: losing the steering readback must not deactivate the hardware component
  and take the whole drive path with it.
* The steering **velocity** state interface is always 0.0 on real hardware;
  `steer_servo_node` reports angle only.

**Indices 8–11 are optional.** Firmware without encoder feedback publishes 8 values and
`read()` leaves the wheel position state untouched; wheel odometry then stays flat and a
throttled **`ERROR`** is logged. Firmware with encoder feedback (HWR-10) publishes 12, and
`read()` fills the wheel position state interfaces, which is what
`gripperx_localization/localization_input_node` needs for `/wheel/odom`.
The scaling of these values depends on `COUNTS_PER_OUTPUT_REV` in
`Software/microros/firmware/include/motor_controller.hpp`.

### Indices 12–15 — wheel feedback provenance (FR-11 items 5/6)

Index 4–7 is **either a real encoder measurement or the commanded velocity handed
straight back** (`MotorController::getRPM()` falls back to the target when the encoder is
not running), and nothing in the value itself distinguishes the two. A closed loop built
on an echo would regulate against its own setpoint with an error of identically zero and
look flawless. Indices 12–15 are the missing bit, one code per wheel:

| Code | Name | Measurement? | Meaning |
|---|---|---|---|
| `-1` | `UNKNOWN` | no | never sent by firmware — what a message shorter than 16 decodes to |
| `0` | `NO_ENCODER` | no | no encoder attached to this wheel; 4–7 is the command echoed back |
| `1` | `INIT_FAILED` | no | encoder attached, PCNT rejected its configuration; 4–7 is the echo |
| `2` | `LIVE_UNCONFIRMED` | yes | PCNT configured and running, no count change seen since boot |
| `3` | `LIVE` | yes | counts have moved — the decoder is provably working |

The order is **monotone in confidence**: `code >= 2` is the measurement test. Do not
renumber.

`LIVE_UNCONFIRMED → LIVE` promotes on the **first observed count change since boot** — no
count threshold, no time window. There is **no downgrade path**: a downgrade would have to
mean "commanded to move but not counting", which needs a command/count comparison and
thresholds that have not been measured, and it belongs to HWR-30a. Without it, a
stationary robot would report dead encoders every time it stood still.

**Length guard.** A message shorter than 16 maps to `UNKNOWN` — *provenance not known* —
never to a valid code, exactly like the 8-vs-12 guard above. Silence is not an assurance.
An 8-value message additionally means the publisher has no encoders at all, so 4–7 is the
command echoed back; that is logged as an `ERROR` naming the cause, not as a warning about
odometry.

**`/hw/wheel_feedback_valid`** (`std_msgs/Int32MultiArray`, 4 values, FL FR BL BR,
`KEEP_LAST(1)`/`RELIABLE`/`TRANSIENT_LOCAL`) carries these codes and is republished on
change only. It is a **topic, not a state interface**, on purpose: `GazeboSimSystem`
exports no such interface, so a controller claiming one would fail to activate in sim and
fork real from sim (§3.1.6).

**The wheel velocity is NOT held when its provenance degrades** — unlike the steering
angle, which *is* held. A held steering angle still describes where the wheel points; a
held non-zero velocity asserts that a possibly stationary robot is still moving and would
feed that fiction into odometry. The value is passed through as sent and marked
not-a-measurement instead; acting on that mark is the consumer's job (FR-11 item 6).

`hw_firmware_mock` reports `NO_ENCODER` on all four wheels, because it **is** an echo by
construction — which makes it the standing negative test for this path.

## Bench testing

Launch with `use_mock_firmware:=true` (default) to run `hw_firmware_mock` instead of ESP32/micro-ROS.

When micro-ROS is ready on the ESP32, publish/subscribe the same topics and set `use_mock_firmware:=false`.
