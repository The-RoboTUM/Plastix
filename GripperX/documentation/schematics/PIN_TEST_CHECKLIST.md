# GripperX — Full Pin Test Checklist (Pi 5 + ESP32-S3)

**Status: WORKING CHECKLIST — walked through step by step with the user.** Part A1/A2 (motor PWM/DIR
+ encoders, all 4 wheels) PASSED at the bench 2026-07-27/28, technical verification only — see result
notes inline below. A3 and Part B remain open/not done.
Scope: bench, bare `YD-ESP32-S3-N16R8` devkit + Raspberry Pi 5. Verifies every planned pin/interface
**before** the HWR-8 rewire builds the harness. Method: **functional, incl. movement** — every motor
run needs the user's **explicit per-test approval** (relayed approval never counts).

Source of truth: `documentation/schematics/WIRING_PLAN.md` (§1 ESP32-S3, §2 Pi 5). Where this and the
plan disagree, the plan wins.

Mark each item: **PASS / FAIL / N/A / DEFERRED** + measured value or note.

Not covered this round (user decision): Status display SPI (GPIO38/39/40, HWR-21), BATT_SENSE (GPIO2).

---

## 0. Pre-flight — safety gates (once, BEFORE any movement)

| # | Gate | State |
|---|---|---|
| 0.1 | **Production stack isolated.** With the S3 on `/dev/ttyACM0` (≠ `/dev/esp32`) no running service grabs it. Before any Pi-commanded motor run **and** before deploying the udev fix: stop `gripperx-agent` + `gripperx-bringup` (mapping/navigation as needed). User go/no-go. | ☐ |
| 0.2 | **Inline fuse** in the 12 V motor supply feed (PSU has **no current limiting**). | ☐ |
| 0.3 | **Common ground** (OP-H9): ESP GND ↔ MDD10A GND(s) ↔ encoder-supply GND ↔ 12 V motor-supply GND — confirm by continuity **before** power-on. | ☐ |
| 0.4 | Test motor **mechanically secured / free-spinning**, wheel off the ground. | ☐ |
| 0.5 | **Silkscreen check** (WIRING_PLAN §7 Q1): each `IOxx` label = the planned GPIO; GPIO33–37 (PSRAM) never probed. | ☐ |
| 0.6 | **Pin-test firmware flashed** (Agent 1): boots STOPPED, drives exactly one pin per serial command. | ☐ |

---

## A. ESP32-S3 (`esp32-drive`) GPIO — WIRING_PLAN §1

### A1. Motor PWM / DIR outputs

Wheels: **FL = 4/5**, **FR = 6/7**, **BL = 15/16**, **BR = 17/18** (PWM/DIR). Firmware pin map already
matches (`main.cpp:16-23`).

| Wheel | PWM | DIR | Signal check (driver disconnected, meter/scope) | Idle-LOW (R30–R37) | **[GATE]** Functional run (approval per channel) |
|---|---|---|---|---|---|
| FL | 4 | 5 | **PASS** (see method note) | **PASS** — no boot-time spin observed | **PASS** — both directions, moderate speed, no cross-talk |
| FR | 6 | 7 | **PASS** (see method note) | **PASS** — no boot-time spin observed | **PASS** — both directions, moderate speed, no cross-talk (after fixing a physical PWM/DIR lead cross at the driver, see note) |
| BL | 15 | 16 | **PASS** (see method note) | **PASS** — no boot-time spin observed | **PASS** — both directions, moderate speed, no cross-talk |
| BR | 17 | 18 | **PASS** (see method note) | **PASS** — no boot-time spin observed | **PASS** — both directions, moderate speed, no cross-talk |

Rules: one MDD10A channel + test motor at a time; common GND verified; short run, low duty, watch
current. **Explicit user approval required before each functional run.**

**Result 2026-07-27/28 (bench, `pin_test.cpp`, env `pintest`):** All 4 wheels PASS for both PWM/DIR
and encoders (see A2 below) — technical verification only, final acceptance is the user's.
- **Signal-check method actually used:** not the originally planned disconnected-driver meter/scope
  probe, but **single-pin drive** — commanding only the PWM pin (`analogWrite`, motor spins at fixed
  low duty) or only the DIR pin (`digitalWrite`, motor stays still, no PWM) in isolation. This proved
  reliable for identifying which physical lead is PWM vs. DIR without relying on wire color.
- **Direction convention confirmed:** LEFT side (FL/BL) — DIR HIGH = physical **backward**, encoder
  count increases on backward. RIGHT side (FR/BR) — DIR HIGH = physical **forward**. Confirms the
  left/right mirroring assumed in the drivetrain design.
- **Command-chain audit:** no software pin swap found (firmware pin map matches WIRING_PLAN §1.1
  exactly). The audit traced a **physical PWM/DIR lead cross at the FR driver connection** — found
  and corrected during the bench session.
- Both Cytron MDD10A driver units confirmed working.
- Boot-safe early-LOW init held throughout (all 8 PWM/DIR pins, R30–R37 pull-downs) — no boot-time
  motor spin observed → **SR-7 structurally confirmed at bench** (final production confirmation still
  pending full reassembly).
- **Perfboard status:** Perfboard #1 (encoder + motor control wiring used for this bench test) works.
  Perfboard #2 is not yet built.
- **Still open / not covered by this PASS:** encoder counts-per-rev not calibrated (odometry scaling
  TODO, see ASBUILT.md); A3 and Part B below; bench PSU has **no current limiting** (gate 0.2 —
  inline fuse still to be added).

### A2. Encoder inputs (needs encoder-capable firmware — Agent 3 / PCNT in pin-test fw)

Wheels: **FL = 8/9**, **FR = 10/11**, **BL = 12/13**, **BR = 14/21** (A/B). PCNT via GPIO matrix.

| Wheel | A | B | Vcc = 3.3 V (NOT 5 V) | Hand-turn → A/B count, correct direction (movement-free) | **[GATE]** Under drive: counts track command |
|---|---|---|---|---|---|
| FL | 8 | 9 | **PASS** — 3.3 V push-pull, no pull-ups/downs | **PASS** | **PASS** — counts track command, no cross-talk |
| FR | 10 | 11 | **PASS** — 3.3 V push-pull, no pull-ups/downs | **PASS** (initially read 0, traced to the same physical lead cross fixed above) | **PASS** — counts track command, no cross-talk |
| BL | 12 | 13 | **PASS** — 3.3 V push-pull, no pull-ups/downs | **PASS** | **PASS** — counts track command, no cross-talk |
| BR | 14 | 21 | **PASS** — 3.3 V push-pull, no pull-ups/downs | **PASS** | **PASS** — counts track command, no cross-talk |

**Result 2026-07-27/28:** All 4 encoder pairs PASS, both directions, moderate speed, no cross-talk
between wheels. Encoders are 3.3 V push-pull Hall outputs — confirmed no pull-ups/pull-downs fitted
or needed. **Not yet done:** counts-per-rev calibration (odometry scaling `COUNTS_PER_OUTPUT_REV` in
`motor_controller.hpp` remains the unconfirmed nominal value 2200 = 11 CPR × 4 × 50:1 gear ratio).
**Firmware/board note:** the in-repo production firmware (`main.cpp` + `motor_controller.cpp` +
`quad_encoder.cpp`) now contains real PCNT encoder feedback, but this bench PASS was run with the
separate `pin_test.cpp` bench instrument, not that firmware. After the bench session the board was
restored to the **pre-encoder production firmware** (open-loop, no PCNT) — the encoder-feedback
firmware exists in-repo but has not been deployed/flashed to the board.

### A3. Reserved / transport pins — verify only, do NOT lay signals on them

| # | Pins | Check | State |
|---|---|---|---|
| A3.1 | UART0 **43/44** | Pi link via COM USB-C (Option A) → micro-ROS enumerates | ☐ |
| A3.2 | **19/20** | native USB / JTAG — left free (debug) | ☐ |
| A3.3 | Strapping **0/3/45/46** | untouched by any signal | ☐ |
| A3.4 | Flash **26–32**, PSRAM **33–37** | excluded — not probed | ☐ |

---

## B. Raspberry Pi 5 interfaces — WIRING_PLAN §2

Pi owns no motor/encoder GPIO — verify device enumeration/function. Many bench items may be absent →
mark N/A.

| # | Interface | Check | State |
|---|---|---|---|
| B1 | **LD06** `/dev/lidar` → `ttyAMA0` @230400 (GPIO14/15, pins 8/10) | symlink present, `/scan` publishes | ☐ |
| B2 | **IMU BNO085** i2c1 (GPIO2/3, pins 3/5) | `i2cdetect -y 1` → 0x4A/0x4B *(N/A if not fitted)* | ☐ |
| B3 | **LIDAR_EN** GPIO23 (pin 16), default ON | *(only if switch HW present; else DEFERRED)* | ☐ |
| B4 | **USB topology** | esp32-drive / steering_servo / arm_servo / 2× camera enumerate; udev symlinks resolve *(absent devices → N/A)* | ☐ |
| B5 | **micro-ROS end-to-end** (after Agent 2 udev fix) | S3 binds as `/dev/esp32`, agent connects, `/hw/joint_states` ↔ `/hw/joint_commands` flow — **only with production stack isolated (gate 0.1)** | ☐ |

---

## Walkthrough order

1. **Now, movement-free & parallel to the SW agents:** Pi checks B1–B4, ESP signal-level A1 (signal
   column) + A3.
2. **After Agent 1 (pin-test fw):** A1 functional runs (per-channel approval gate).
3. **After Agent 3 (encoder fw) / PCNT in pin-test fw:** A2 encoder tests.
4. **After Agent 2 (udev) + stack isolation:** B5 end-to-end.
