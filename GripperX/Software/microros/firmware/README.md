# ESP32 firmware (micro-ROS)

PlatformIO firmware for GripperX's ESP32-S3. Publishes `/hw/joint_states` and listens on `/hw/joint_commands` via micro-ROS serial.

Contract compatible with `gripperx_hardware_interface` and `teleop_joint_commands_node` in the ROS 2 workspace.

## `Theo-pi-merge` is NOT an open firmware divergence — settled 2026-08-24

The rescue commit `82c13c2` ("commit the working tree as deployed") produced the branch
`Theo-pi-merge` (tip `530fc94`), and its `motor_controller.hpp` differs from `Theo` by 42
lines. That looked like an unresolved firmware fork and was carried as a blocker. It is not
one, and nothing needs merging:

- **Those 42 lines are 0 lines of code.** `git diff -w Theo Theo-pi-merge -- Software/microros/`
  with comment lines removed is EMPTY across the whole firmware tree. Both states compile to
  the same binary, so the flashed board matches `Theo` whichever source you start from.
- The divergence is one comment block, and **`Theo` holds the newer one**. The Pi side still
  carries the pre-2026-08-20 text; `Theo` replaced it with the measured version (safety
  finding F-43, the breakaway/drop-out ladder, "THE MACHINE CANNOT CREEP").
- `main.cpp`, `motor_controller.cpp`, `quad_encoder.cpp`, `pin_test.cpp` and this README are
  byte-identical between the two. The large per-file counts reported earlier were measured
  against the older ancestor `1fd9a02`, not between the branches.
- `git cherry Theo Theo-pi-merge` lists exactly one commit not already patch-equivalent in
  `Theo`: `82c13c2` itself. Outside the firmware it brings mostly `.bak-premerge` /
  `.bak-predemo` backup files, plus older copies of files `Theo` has since moved on.

**Do not merge the branch to "recover" the firmware** — it would only reintroduce the
superseded comment. The same holds for `gripperx_localization/config/localization.yaml`, the
one file that looked like real loss: the Pi copy still claims "one (a, b) pair cannot serve
both directions", which `Theo` disproved on 2026-08-21 with the (a, b, h) triple.
Branch tip recorded here as `530fc94` so it can be deleted without losing the reference.

## Requirements

- PlatformIO (`~/.platformio/penv/bin/pio`)
- Board **YD-ESP32-S3-N16R8**, connected on `/dev/ttyACM0` via the board's COM USB-C port
  (CH343 bridge). On the Pi the udev rule `pi_env/udev/99-gripperx.rules` gives it the
  stable name `/dev/esp32`; `platformio.ini` uses the raw `/dev/ttyACM0`.
- ROS 2 **Jazzy** (`board_microros_distro = jazzy`)

## Environments

| Env | Sources | Purpose |
|---|---|---|
| `esp32-s3` | everything except `pin_test.cpp` | production firmware (micro-ROS, encoders) |
| `pintest` | only `pin_test.cpp` | bench instrument, no micro-ROS |

There is no `esp32dev` env — that was the retired ESP32-WROOM-32 board.

## Flashing (production)

```bash
sudo chmod 666 /dev/ttyACM0
cd Software/microros/firmware
pio run -e esp32-s3 -t upload
```

If you changed the ROS distro, clean micro-ROS first:

```bash
pio run --target clean_microros
pio run -e esp32-s3 -t upload
```

**Do not open the PlatformIO serial monitor while the agent is running.**
On the Pi, stop the agent before flashing: `docker stop mros_agent`.

## Flashing (bench instrument)

`src/pin_test.cpp` verifies each drive pin and all four encoder pairs at the driver
header, with the MDD10A disconnected. No micro-ROS, no Pi needed.

```bash
pio run -e pintest -t upload
pio device monitor -e pintest
```

Serial commands @115200: `l` pin table · `<n> on|off` single pin · `s` stop all ·
`r` reset encoder counters · `m <FL|FR|BL|BR> <f|r> [duty]` drive one wheel.

`m` asserts PWM **and** DIR together and **will spin a motor** if the driver is connected
and powered — only with the wheel free-standing and with explicit approval. Everything
boots stopped, and an active pin auto-stops after 4 s.

Encoder counts print continuously, so a hand-turned wheel can be checked without driving
anything. This is also how `COUNTS_PER_OUTPUT_REV` gets measured — see
`documentation/ENCODER_FEEDBACK.md` §4.

## Topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/hw/joint_commands` | `std_msgs/Float64MultiArray` | ROS 2 → ESP32 | **30 Hz** — set on the Pi by `controller_manager` `update_rate` (`gripperx_control/config/ros2_controllers.yaml` L3); the firmware applies each command in `cmdCb()` as it arrives and does not rate-limit it |
| `/hw/joint_states` | `std_msgs/Float64MultiArray` | ESP32 → ROS 2 | **30 Hz** — `main.cpp` `STATES_PUBLISH_US 33333`, scheduled on `micros()` with a fixed phase |

Both rows read **100 Hz** before 2026-08-20 and neither was ever true. `/hw/joint_states`
was `STATES_PUBLISH_MS 100` (nominal 10 Hz) and **measured 8.72 Hz / 114–121 ms**, because
`rclc_executor_spin_some()` was allowed to sleep 100 ms in the same loop; `/hw/joint_commands`
has been driven by the Pi's 30 Hz `update_rate` throughout.

The 30 Hz state rate is bounded by the 115200 8N1 link, not chosen for convenience —
the link budget (75.0 % occupancy at 30 Hz, 25 pp margin, and why 40/50 Hz were rejected)
is written out at `STATES_PUBLISH_US` in `src/main.cpp`.

**The publish rate is not the measurement rate.** Encoder sampling is decoupled from
publishing: `MotorController::sampleEncoder()` runs on every loop iteration, self-throttled
to 200 Hz, and the velocity is a first difference of the PCNT accumulator over a sliding
**100 ms** window with `micros()` timing. Rationale and the noise/delay arithmetic are in
`include/motor_controller.hpp`.

Firmware node: `/gripperx_firmware`

### Array layout

`/hw/joint_commands` (ROS 2 → ESP32) — **8 values**:

| Index | Joint | Unit | ESP32 |
|--------|-------|--------|-------|
| 0–3 | steering | rad | ignored — steering servos are driven directly by the Pi, not the ESP32 |
| 4 | `f_leftwheel` | rad/s | motor 1 (FL) |
| 5 | `f_rightwheel` | rad/s | motor 2 (FR) |
| 6 | `b_leftwheel` | rad/s | motor 3 (BL) |
| 7 | `b_rightwheel` | rad/s | motor 4 (BR) |

`/hw/joint_states` (ESP32 → ROS 2) — **16 values**. Indices 0–7 are the original
contract consumed by `gripperx_hardware_interface::read()` (size check is `>= 8`,
so the appended blocks are backward-compatible); indices 8–11 are the **real
encoder positions** (HWR-10); indices 12–15 are the per-wheel **provenance** codes
(`EncoderStatus`, FR-11 items 5/6) that say whether 4–7 is a measurement or an echo
of the command. The Pi keys its length guards on 8 / 12 / 16, so anything appended
later must go **after** index 15:

| Index | Joint | Unit | ESP32 |
|--------|-------|--------|-------|
| 0–3 | steering | rad | not sensed on the ESP32 → echoed as 0.0 |
| 4–7 | FL/FR/BL/BR wheel | rad/s | **measured** wheel velocity (was synthetic) |
| 8–11 | FL/FR/BL/BR wheel | rad | **measured** wheel position (HWR-10) |
| 12–15 | FL/FR/BL/BR wheel | `EncoderStatus` code | **provenance** of 4–7 and 8–11: `0` no encoder · `1` PCNT init failed · `2` running, no count seen yet · `3` live (FR-11 items 5/6) |

Wheel velocity and position are derived from 4× hardware PCNT x4 quadrature
decoders (encoders on GPIO 8/9, 10/11, 12/13, 14/21). The counts-per-revolution
scaling (`ENCODER_CPR_PER_CHANNEL`, `GEAR_RATIO` in `include/motor_controller.hpp`)
was **measured** on the assembled robot 2026-08-13 (3200 counts per output revolution,
= 16 × 4 × 50), and the per-wheel encoder direction signs were confirmed in the same
session — see the comments there. Re-measure only if motors or encoders are replaced.

## Manual test (without teleop)

With the agent running:

```bash
ros2 topic pub -r 10 /hw/joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]}"

ros2 topic echo /hw/joint_states
```

See also: [`../TROUBLESHOOTING.md`](../TROUBLESHOOTING.md)
