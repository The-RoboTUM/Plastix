# GripperX — As-built Specification

**Status:** component inventory 2026-07-13; **partially updated since — see the correction notice below.**
**Responsible (survey):** Agent `gripperx-specification` (strictly read-only)

> ## ⚠️ CORRECTION NOTICE — 2026-08-24. READ THIS BEFORE THE BANNER BELOW.
> **The robot is NOT disassembled.** It was reassembled and has been **driving since 2026-08-19**, with
> hardware runs on 2026-08-19, 2026-08-20 and 2026-08-21. The banner below describes the **2026-07-13
> rework** and was never retired; it is kept as the record of that period, **not as a description of the
> machine today.**
>
> **This file is a mix of vintages and must be read as one.** §1.1-§1.5 carry updates as late as
> 2026-08-18, while **§1.6 (power supply) was never updated after 2026-07-13.** Four claims of that
> vintage were corrected on 2026-08-24 and each is marked inline: the **driver regrouping** (§1.2), the
> **actuator/drive switching level** (§1.6), the **main fuse rating** (§1.6, §2), and this banner.
>
> ⚠️ **Where this file and `documentation/schematics/WIRING_PLAN.md` disagree, WIRING_PLAN wins for
> wiring and topology** — it was traced element-by-element from the schematic on **2026-08-12**, which is
> a month later than this file's inventory date. **Items neither document can settle from the desk are
> listed in the internal requirements audit of 2026-08-24, §3.8, as checks for the machine.**

> ## ⚠️ STATUS AS OF 2026-07-13 — SUPERSEDED, KEPT AS THE RECORD OF THE REWORK PERIOD
> The robot has been **completely disassembled since the course change on 2026-07-09** (chassis stiffening,
> power supply, wiring/electronics build, encoder repair — see Section 10 (`HWR-*`/`HWA-*`) of the
> internal requirements document, tracked internally and not part of this repository, for the goal,
> requirements and open decisions of the rework; formerly a separate hardware-rework document,
> merged in on 2026-07-14).
> This file records **only the COMPONENT inventory verified by the disassembly on 2026-07-13**
> (what is physically installed/present). It describes **NO currently valid wiring/topology/pin assignment**
> — the chassis is being rebuilt, the electronics/cable build is being redone on perfboard. A
> **final, wiring-accurate as-built version will follow only after the rebuild**, once wiring diagrams
> (`documentation/schematics/`) and pin mapping on the new ESP32-S3 board have been verified.

---

## 0. This file replaces the desktop legacy `~/Desktop/GripperX_Asbuilt.md`

The old desktop file (last updated 2026-07-01, created **without physical disassembly inspection**) contained
several assumptions that were disproved as **incorrect** by the disassembly on 2026-07-13. The
internal requirements document (Section 7 `OP-H*` / Section 10) documents these corrections;
they are adopted here for the as-built specification:

| Old assumption (legacy as-built / setup plan) | Real (verified at disassembly 2026-07-13) |
|---|---|
| 2× Cytron **MDD20A** as motor driver | **1× Cytron MDD10A Rev2.0 + 1× DBH-12V** (two different drivers, neither with an encoder input) |
| Battery **3S LiPo** (setup plan assumption) | at disassembly: **4S LiFePO4**, DREAMDASH, 12 V / 10 Ah / 128 Wh, `AKCFAH0347` — **meanwhile (user 2026-07-13) replaced by MHPOWOS 12 V 20 Ah** (20 A continuous / 28 A pulse), see 1.6 |
| ESP32 on a "**matching** YD-ESP32-S3 adapter" | **WROOM-32 module on an S3-44P adapter — board/adapter mismatch** (WROOM-32 is not an S3, pinout/terminal labels don't match); as a procurement decision the user ordered the matching `YD-ESP32-S3-N16R8` |

The old file was regarded as the reference up to this point; from now on **this file** (`documentation/ASBUILT.md`) is the
canonical as-built source for GripperX. In case of contradiction between the desktop legacy and the
current internal requirements document (Section 10), that document applies.

---

## 1. Component inventory (verified at disassembly 2026-07-13, unless noted otherwise)

### 1.1 Onboard computer / boards
- **Raspberry Pi 5, 8 GB RAM**, Ubuntu 24.04 arm64, ROS2 Jazzy, hostname `GripperX-1`. *(Source: legacy as-built, PM)*
- **YD-ESP32-S3-N16R8** (ESP32-S3-WROOM-1, 44-pin, 16 MB flash / 8 MB octal PSRAM, native USB, 4×
  hardware PCNT), on-board USB-UART bridge (CH343/CH9102-family chip) for the micro-ROS serial link,
  drive control. **Installed and bench pin-tested 2026-07-27/28** (motor PWM/DIR + quadrature
  encoders confirmed on all 4 wheels — see §1.2). Supersedes the retired ESP32-WROOM-32-on-mismatched-
  S3-adapter combination described in §0 above.
- Seeed XIAO Bus Servo Driver Board **V1.0** for the arm (USB-C→Pi).
- Waveshare Bus Servo Adapter (A) for the steering (USB).
- **Pi cooling:** currently only one fan, **no heat spreaders/heatsinks** on the processors.

### 1.2 Drivetrain (4WID)
- 4× DC gear motor **Pololu 37D SYJ GB37-50** with **quadrature Hall encoder** (2 channels A/B, 3.3–5 V).
- **Motor drivers: 2× Cytron MDD10A Rev2.0** (dual-channel, 10 A/channel, 30 A peak, PWM+DIR control) —
  **both units confirmed working at the bench pin test 2026-07-27/28**. Current physical grouping:
  **FRONT driver (`A3`) = FL + FR, REAR driver (`A4`) = BL + BR** (⚠️ **CORRECTED 2026-08-24** — this
  line previously still read *"left pair (FL/BL) on one driver, right pair (FR/BR) on the other"*, the
  pre-2026-07-29 LEFT/RIGHT grouping, which the correction directly below already contradicted in the
  same paragraph). **⚠️ CORRECTED 2026-08-24 — THIS REGROUPING IS INSTALLED, not planned.** It reads *"planned change
  (user decision 2026-07-29, housing redesign; **not yet installed**)"* below; `documentation/schematics/WIRING_PLAN.md` §7 records
  it as **AS-BUILT 2026-08-12, confirmed against the harness by the user** — `A3` (FRONT) `FL` ch 1 /
  `FR` ch 2 as planned, and `A4` (REAR) built with **`BR` ch 1 / `BL` ch 2**, i.e. the channel order
  swapped against the plan (electrically irrelevant — the MDD10A's channels are identical and each wheel
  keeps its own ESP32 PWM/DIR pair; no pin-map and no firmware change). *Original text, superseded:*
  **~~Planned change (user decision 2026-07-29, housing redesign; not yet installed):~~** regroup to
  **FRONT driver = FL+FR / REAR driver = BL+BR** — harness/perfboard + documentation change only, ESP32 per-wheel pin map (§1.1) and firmware
  direction signs unchanged (WIRING_PLAN §0.1/§7, REQUIREMENTS OP-H9/HWA-2/HWA-11). The previously installed **DBH-12V has
  been physically removed** (OP-H9; formerly documented here as a second, mismatched driver — see §0
  history table). Neither MDD10A channel reads encoders — encoders go straight to the ESP32-S3 for
  evaluation via hardware PCNT.
- **Pin mapping — CONFIRMED at bench pin test 2026-07-27/28** (ESP32-S3,
  `Software/microros/firmware/src/main.cpp`, matches `documentation/schematics/WIRING_PLAN.md` §1.1 exactly; supersedes the
  retired legacy WROOM-32 map below):

  | Wheel | PWM | DIR | Encoder A | Encoder B |
  |---|---|---|---|---|
  | FL | GPIO4 | GPIO5 | GPIO8 | GPIO9 |
  | FR | GPIO6 | GPIO7 | GPIO10 | GPIO11 |
  | BL | GPIO15 | GPIO16 | GPIO12 | GPIO13 |
  | BR | GPIO17 | GPIO18 | GPIO14 | GPIO21 |

  None of these are ESP32-S3 strapping pins (S3 strapping set = GPIO0/3/45/46, none used for drive) —
  the legacy WROOM-32 strapping-pin problem below no longer applies (different chip). Boot-safe
  early-LOW init on all 8 PWM/DIR outputs (R30–R37 pull-downs) held through the bench session — no
  boot-time motor spin observed → **SR-7 structurally confirmed at bench** (final production
  confirmation still pending full reassembly).

  **Bench pin-test verification 2026-07-27/28** (`documentation/schematics/PIN_TEST_CHECKLIST.md`
  Part A1/A2; bench instrument `Software/microros/firmware/src/pin_test.cpp`, PlatformIO env
  `pintest`; technical verification only, final acceptance is the user's):
  - All 4 wheels: motor PWM/DIR **and** quadrature encoders confirmed functional, both directions,
    moderate speed, no cross-talk between channels.
  - **Direction convention confirmed:** LEFT side (FL/BL) — DIR HIGH = physical **backward**, encoder
    count increases on backward. RIGHT side (FR/BR) — DIR HIGH = physical **forward**. Confirms the
    left/right mirroring assumed in the drivetrain design.
    **SUPERSEDED for the encoder half (re-measured 2026-08-13 on the assembled robot):** all four
    count directions are inverted against this entry — forward now counts **up** on FL/BL and **down**
    on FR/BR. Uniform inversion = a consistently swapped A/B pair in the rebuilt harness; miswired
    encoder cables were found and corrected in that session. The DIR↔physical half is untested since
    the rebuild. Details and the resulting `ENC_DIR_*` rule in `ENCODER_FEEDBACK.md`.
  - **Safe-start pull-downs R30–R37 VERIFIED 2026-08-13, all 8 pass.** Tested with the actuator branch
    off by driving each MDD10A input pin to high-Z from the ESP32 — the exact state that exists for a
    few ms during an ESP32 reset, before `setup()` pulls the drive pins LOW. Every pin read LOW while
    floating **and** while fighting the ESP32's internal ~45 kΩ pull-up (0/5 samples high in both
    cases), which puts them firmly in the 10 kΩ class rather than being a mere leakage path. Measured
    in software via the `pd yes` command in the `pintest` firmware, so the reading covers the whole
    signal path including the connector, at the driver input itself. Until this test the safe-start
    rested entirely on an unverified assumption.
  - **`COUNTS_PER_OUTPUT_REV` measured 2026-08-13: 3200** (FL 3202.1 / BL 3198.9 over 10 hand-turned
    output revolutions each, 0.1 % apart). Factors exactly as 16 pulses/channel × 4 × 50:1, so the
    encoder is 64 counts per motor revolution and the nominal 50:1 gearbox is confirmed. The previous
    2200 came from a placeholder 11 pulses/channel; dividing real counts by too small a number
    inflates the result, so reported speed and distance would have been ~45 % **too high** (one true
    revolution = 3200 counts, reported as 3200/2200 = 1.45 rev). No drive test has run on the rebuilt
    robot, so no recorded odometry or map is affected.
  - **Command-chain audit:** no software pin swap found (firmware pin map matches the plan exactly).
    The audit traced a **physical PWM/DIR lead cross at the FR driver connection**, found and
    corrected during the bench session. **Single-pin drive** (drive only PWM → motor spins at fixed
    low duty; drive only DIR → motor stays still) proved the reliable method for identifying PWM vs.
    DIR conductors without relying on wire color.
  - Encoders: 3.3 V push-pull Hall outputs, no pull-ups/pull-downs fitted or needed (confirmed).
  - **Firmware/board state:** the in-repo production firmware (`main.cpp` + `motor_controller.cpp` +
    `quad_encoder.cpp`) now contains real PCNT-based encoder feedback (`getRPM()` returns measured,
    not commanded, RPM) — but this is **source only, not yet deployed**. The bench PASS above was run
    with the separate `pin_test.cpp` bench instrument; afterwards the physical board was **restored
    to the pre-encoder production firmware** (open-loop, no PCNT). The encoder-feedback firmware has
    not been flashed to the board.
    **SUPERSEDED 2026-08-18 — the encoder firmware IS deployed and has been running in production
    since 2026-08-17:** the "restored to pre-encoder" state above described the board only for the
    interval between the 2026-07-27/28 bench session and the next firmware change; it is not the
    current state. On **2026-08-17** ("Fix firmware ROS domain and reflash" session, which also moved
    the DDS participant into domain 20 — see the direction-convention entry above) the board was
    reflashed with the encoder firmware and verified live: `ros2 topic echo /hw/joint_states --once`
    returned **12 values** (the pre-encoder firmware publishes 8; the 12-value layout and the PCNT
    encoder plumbing were introduced together in commit `0b2c7ba`), at ~8.72 Hz. It stayed flashed and
    in use through the 2026-08-18 ground tests: wheel positions on the running robot read
    `-5.79, 5.13, -18.87, 17.85 rad` — accumulated counts an open-loop firmware cannot produce — and a
    same-day steering-differential ground test (2026-08-18 10:54) recorded an encoder-derived
    outer/inner wheel-speed ratio of **1.644** while driving, which requires live encoder feedback to
    exist at all. Evidence: the archived session journal of 2026-08-17 (reflash + verification) and of
    2026-08-18 (10:54 differential measurement) — tracked internally, not in this repository.
    **Later on 2026-08-18** (concurrent with this correction, not yet journaled at the time of writing)
    the board was reportedly reflashed again, extending `hw/joint_states` to **16 values** with a
    per-wheel encoder-status provenance block (FR-11 items 5/6): `[0-3]` steering, reserved and always
    zero from this firmware; `[4-7]` wheel velocity; `[8-11]` wheel position; `[12-15]` one
    `EncoderStatus` code per wheel (FL, FR, BL, BR order) — `0`=NoEncoder, `1`=InitFailed,
    `2`=LiveUnconfirmed, `3`=Live (`Software/microros/firmware/include/motor_controller.hpp`,
    commit `2dd405e`, already present in this worktree's source tree). **Deployment VERIFIED live on
    the robot 2026-08-18**, actuator power off: after the reflash `/hw/joint_states` carried 16 values,
    initially `[…, 2.0, 2.0, 3.0, 3.0]` (FL/FR `LiveUnconfirmed`, BL/BR already `Live`) and `[3,3,3,3]`
    after all four wheels had been turned by hand — the `LiveUnconfirmed → Live` promotion observed on
    real hardware. `/hw/wheel_feedback_valid` (latched, `GripperXInterface`) reported the same codes.
    Wheel↔index mapping is closed by construction rather than by measurement for FL/FR: the encoder
    pin defines (`main.cpp` `ENC_FL_A 8` … `ENC_BR_B 21`) are character-identical to
    `documentation/schematics/WIRING_PLAN.md` §91-98, and the measured half (BL→index 10/14, BR→11/15) matches. **Not
    verified:** `InitFailed` (not producible without a PCNT rejection), and the encoder *count
    directions* under this firmware — `ENC_DIR_*` are all `1`, and the harness A/B swap corrected on
    2026-08-13 is the precedent for why that is worth re-checking.
  - **Still open (as of the 2026-07-27/28 bench session):** encoder counts-per-rev not calibrated at
    the bench — odometry scaling (`COUNTS_PER_OUTPUT_REV` in `motor_controller.hpp`) was, at that
    time, the unconfirmed nominal value 2200 (11 CPR × 4 × 50:1 gear ratio), flagged `TODO(HWR-10)` in
    source. **RESOLVED 2026-08-13** — see the measured-3200 entry above; the source no longer carries
    the TODO (`motor_controller.hpp` now reads "measured 2026-08-13" at the constant definition).
    Still genuinely open from the bench session: Part A3 (transport/reserved pins) and Part B (Pi
    interfaces) of the checklist not yet run; bench PSU has **no current limiting** (inline fuse still
    to be added, checklist gate 0.2).
  - **Perfboard build status:** Perfboard #1 (encoder + motor control wiring used for this bench
    test) is built and confirmed working. Perfboard #2 is **not yet built**.
- **Legacy WROOM-32 pin map + strapping-pin problem — RETIRED, historical record only** (described the
  removed WROOM-32-on-mismatched-adapter setup, does not apply to the confirmed ESP32-S3 wiring
  above; last verified 2026-06-29/07-01, before disassembly):

  | Motor | PWM | DIR | inv | Note |
  |---|---|---|---|---|
  | FL | GPIO19 | GPIO13 | true | — |
  | BL | GPIO18 | GPIO21 | true | — |
  | BR | GPIO16 | GPIO26 | false | GPIO16 = ESP32(-WROOM-32) strapping pin |
  | FR | GPIO5 | GPIO17 | false | GPIO5 = ESP32(-WROOM-32) strapping pin |

  FR-PWM (GPIO5) and BR-PWM (GPIO16) were ESP32-WROOM-32 strapping pins — pull-ups drove them HIGH on
  reset, causing FR/BR to run uncontrolled for ~500 ms after every reset. This is an artifact of the
  removed WROOM-32 board and its different strapping-pin set; it does not carry over to the ESP32-S3.
- **Known anomalies (measured on the software side, before disassembly, WROOM-32/DBH-12V setup —
  historical, not reproduced on the current ESP32-S3 + 2× MDD10A setup):** the FR wheel sometimes did
  not turn at all; the left motors ran about 20% faster than the right, and additionally faster in
  reverse than forward — plausible contributing cause at the time: the two mismatched drivers (MDD10A
  vs. DBH-12V).

### 1.3 Steering (4WIS)
- 4× **Feetech ST3215** bus servos, serial via Waveshare USB adapter, ttyACM0 @ 1 Mbps.
- **Calibration** (`protocol_end=0`, reference baseline before disassembly):

  | Servo | ID | center_counts | +90° counts | −90° counts |
  |---|---|---|---|---|
  | FL | 11 | 1785 | 2809 | 761 |
  | FR | 14 | 1147 | 2171 | 123 |
  | BL | 12 | 1104 | 2128 | 80 |
  | BR | 13 | 1544 | 2568 | 520 |

  These values apply to the **previous** mechanical setup (spring linkage). After the chassis rework
  (rigid direct linkage), a **recalibration is mandatory** — the numbers are documented here as
  historical reference, not as currently valid.
- **Sporadic kernel panic on Pi boot** with the servo USB plugged in: occurred occasionally
  in earlier sessions, has not been reproduced several times since; a causal connection with the servo USB driver
  is **not established**.

### 1.4 Arm
- LeRobot **SO101**, 6 DOF, 6× **Feetech STS3215 (12 V variant)**, IDs 1–6, Seeed XIAO board V1.0, 1 Mbps.
- Servos completely ignore the MOVE_TIME/GOAL_SPEED registers → speed is produced via a software ramp
  (linear interpolation, ~30 steps/s), not via firmware timing. **Generalized troubleshooting lesson
  (Feetech STS/SCS servos in general, not only this arm):** before investing time in register tuning,
  first check whether MOVE_TIME/GOAL_SPEED have any effect at all — ideally by comparison with a
  known-working servo (e.g. the steering ST3215s in §1.3, where the same register logic does work).
  Some firmware variants simply ignore these registers; a software ramp is the robust fallback
  whenever they do. Relevant again for any future Feetech servo work (steering servos, a replacement
  arm, etc.), not just this incident.
- **Arm calibration table** (raw servo counts, verified 2026-07-01, before disassembly — the arm is
  not mechanically part of the hardware rework, but the power supply/wiring changes do affect it, so
  a re-check after the rebuild is still pending):

  | Joint | Approach (GRIP_POS) | Home (HOME_POS) | Gripper open | Gripper closed |
  |---|---|---|---|---|
  | 1 | 2106 | 2069 | – | – |
  | 2 | 3036 | 963 | – | – |
  | 3 | 1513 | 3077 | – | – |
  | 4 | 2934 | 1031 | – | – |
  | 5 | 2113 | 2042 | – | – |
  | 6 | 2652 | 1356 (closed) | 2656 | 1497 |

  Home position = gripper closed (J6=1356) — holds litter while returning. `POS_TOLERANCE=80`; the
  arm reaches home within 10–20 ticks.
- **Grip-sequence timing — contradiction between sources, flagged not resolved:** the calibration
  snapshot above (2026-07-01) records approach 2000 ms → grip_close 800 ms → return_home
  2500 ms + 500 ms. the internal requirements document FR-3 documents later, final production timings (after the user
  twice requested the arm be slowed further): startup home 7600 ms, approach 6000 ms, grip_close
  3600 ms, return_home/go_home 6400 ms, open_gripper 3600 ms. the internal requirements document FR-3 should be
  treated as authoritative for **timings** — the 2026-07-01 values above look like a
  pre-final-slowdown snapshot — but this is stated here rather than silently corrected, since this
  file's role is to record verified as-built values, not adjudicate between sources. The
  **positions** in the table above are not affected by this timing discrepancy.
- Grip sequence `pick_plastic` verified at 12 V before disassembly.

### 1.5 Sensors
- **LiDAR LD06** on Pi GPIO UART (ttyAMA0), so far with a separate 5 V supply + GND bridge to the Pi.
- **IMU: BNO085 9-DoF (STEMMA QT / I2C) is present** — not yet connected. Architecture decision
  (2026-07-13): connection via **Pi I2C**, not the ESP32.
- **GNSS/GPS:** no module installed; planned as a later requirement, HW preparation (space/connector/
  power) is part of the MVP rework, connection also planned via the Pi.
- **Cameras:** currently **none** installed (the user's wish for a gripper camera and optionally a front camera is
  new and part of the rework, see §10 of the internal requirements document).

### 1.6 Power supply
- **Battery (CHOSEN / swapped, user 2026-07-13):** **MHPOWOS 12 V 20 Ah LiFePO4**, manufacturer spec
  **20 A continuous discharge current / 28 A pulse current (≥3 s)**, ~256 Wh — **actual stock, once procured**. Physical dimensions of the
  delivered pack still to be **measured** (battery compartment to be adapted, see §10 of the internal requirements document HWR-29).
  - **Replaces:** the previous **DREAMDASH 12 V (12.8 V) / 10 Ah / 128 Wh, `AKCFAH0347`, 4S LiFePO4** — **removed**;
    it was the documented brown-out cause (only 10 A continuous current, Ri ≤ 70 mΩ; see §10 of the internal requirements document HWR-1/HWA-1).
- **DC-DC converter:** **szwengao**, output 5 V / 10 A, input 12 V/24 V.
- **Automotive relay JD1912** (typ. ~40 A SPDT, 12 V) — installed as a switching element. **⚠️ CORRECTED
  2026-08-24: the split into two switching paths is BUILT, not merely "agreed as the target
  architecture".** Two relays are in place — **`K1` (actuators) and `K2` (drive)** — and `K2` is
  **cascaded behind `K1`**, so drive power exists only while the actuator path is also closed.
  See `documentation/schematics/WIRING_PLAN.md` §9 / §9.2. *Original wording, superseded:* ~~split into two separate switching
  paths ("actuators overall" vs. "drive motors only") **is agreed as the target architecture**.~~
- **Fuse.** ⚠️ **CORRECTED 2026-08-24: the tiered fusing is DRAWN and its values are settled, and the main
  fuse is `F1` 25 A — not the "~30 A" this file projected.** Per `documentation/schematics/WIRING_PLAN.md` §9.3 (as-drawn
  2026-08-12): **`F1` 25 A MIDI/ANS slow** (changed from 30 A on 2026-08-12), `F2` ~20 A actuator branch,
  **`F3` 5 A on `+5V_LOGIC` and `F4` 3 A on `+5V_SENS`** — both on the **5 V output** side of `T1`,
  downstream of the converter rather than on its 12 V input.

  ⚠️ **RE-CORRECTED 2026-08-25, and the 2026-08-24 correction is withdrawn.** That pass read the
  `~7.5-10 A` / `~3-5 A` `[TBD HWA-1]` ranges out of
  `documentation/schematics/WIRING_PLAN.md` §9.3 and the `.qet` element labels, and concluded that the
  `F3` 5 A / `F4` 3 A written here were wrong. **It had it backwards.** 5 A and 3 A are a **user
  decision of 2026-08-13** which explicitly dropped the `[TBD HWA-1]` flags, with the placement
  corrected the same day and a stated rationale: `T1` is a szwengao WG-1224S rated 5 V / 10 A, so
  `F3` 5 A + `F4` 3 A = 8 A fits under the converter, and the USB hub sits on 12 V so the Pi does not
  carry its peripherals. The internal requirements record has said since that date that it is the
  §9.3 **table** that lags and needs correcting on the schematics side — which has not happened yet,
  so **§9.3 and the `.qet` labels still show the superseded ranges**. Where they and this file
  disagree on `F3`/`F4`, this file carries the decision.

  **Four further branch fuses are drawn but still carry placeholder `F?` designators** (Servo
  ~7.5 A?, DRIVE ~20 A, F-DRIVE ~10 A, B-DRIVE ~10 A).
  ⚠️ **`F1` is drawn at 25 A but the fitted part is a non-MIDI interim** — the MIDI-25 A procurement is
  still open (internal REQUIREMENTS `HWA-1`). *Original text, superseded:* ~~**Fuse (old status quo):** 20 A.
  **Will be replaced in the rework** with new, tiered fusing matching the MHPOWOS 20 Ah (main fuse ~30 A
  slow-blow + fuses per branch; preliminary list see internal REQUIREMENTS §10 HWR-4, final values after
  conductor cross-section analysis HWA-1).~~
- ### ⚠️ SWITCHING ARCHITECTURE — CORRECTED 2026-08-24. THE SUPERSEDED TEXT WAS SAFETY-RELEVANT.
  **The sentence struck below said the robot has no separate actuator/drive cut-off. That has been FALSE
  since 2026-08-12. It is corrected here rather than quietly deleted, because this file is exported and a
  reader who acted on it would have believed the machine had no way to kill actuator power short of the
  master switch.**

  **What is built, per `documentation/schematics/WIRING_PLAN.md` §9 — status *as-drawn*, traced element-by-element from the
  schematic on 2026-08-12** (a month later than this file's 2026-07-13 inventory, which is why WIRING_PLAN
  wins on topology):

  - **`Q1` MASTER** → USB hub, `T1` DC/DC (`+5V_LOGIC` / `+5V_SENS`), and `COIL_SUP`
  - **`S1` E-stop (NC)** in the `COIL_SUP` feed → drops **both** relay coils, so all motion stops **while
    the Pi, ESP32 and sensors stay powered** — that is the deliberate `HWR-7` scope: diagnostics and the
    status display survive the stop
  - **`Q3`** → `K1` coil (**actuators**) · **`Q4`** → `K2` coil (**drive**), `K2` cascaded behind `K1`
  - Both relays are **NO** types with coils on the **switched** side, so a failed-open switch, a broken
    coil wire or a blown `F1` all default to **actuators OFF** — de-energised *is* the safe state

  **internal REQUIREMENTS `OP-H2` already records `HWR-7` as aligned with the coil-side switching actually
  built**, so the requirements side was current and only this file lagged.

  ⚠️ **ONE THING IS NOT SETTLED AND IS DELIBERATELY NOT RESOLVED HERE.** The user confirmed on 2026-08-24
  that the machine carries **four operator switches — `Logik` · `Aktor` · `Drive` · `Haupt`**.
  **`documentation/schematics/WIRING_PLAN.md` §9 documents THREE** (`Q1` = Haupt/master, `Q3` = Aktor, `Q4` = Drive) plus the `S1`
  E-stop; **there is no `Q2`, and no separate `Logik` switch appears anywhere in the traced architecture.**

  ⚠️ **NARROWED 2026-08-24 by direct inspection of the `.qet`** (internal hardware audit 2026-08-24, §2).
  One of the three possibilities above is now excluded: **the trace did not miss anything.** The drawing
  the internal QET wiring drawing contains exactly **three `switch.elmt` instances**
  (`Q1`, `Q3`, `Q4`) plus **one `estop.elmt`** (`S1`) — there is no fourth switch element, labelled or
  unlabelled, and no `Q2` or `Q5` designator anywhere in the file. `documentation/schematics/WIRING_PLAN.md` §9 is therefore an
  **accurate** rendering of the drawing; it is the **drawing** that is one operator element short of the
  machine. What remains open is only **which physical switch the fourth one is and where it sits
  electrically** — that is a check for the machine, recorded in
  the internal requirements audit of 2026-08-24 (§3.8) and the internal hardware audit of the same day (Q1). **Nothing about
  that fourth switch's function or position is asserted here.**

  *Original text, superseded 2026-08-24:* ~~**A main switch for the overall system is present** (complete
  stop possible at any time). A **separate switching level just for the actuators/drive motors is
  missing** — this is the subject of the planned tiered switch architecture in the rework (see
  internal REQUIREMENTS §10, HWR-7).~~

---

## 2. Open / still to be verified

These points are deliberately **not guessed** — they remain open until a measurement, a datasheet, or a
physical check is available:

- **Battery dimensions (newly open):** The **fundamental electrical question is closed** — battery chosen =
  **MHPOWOS 12 V 20 Ah** (20 A continuous / 28 A pulse). What remains open is only **measuring the dimensions (L×W×H) on the delivered pack**,
  so the battery compartment can be adapted (§10 of the internal requirements document HWR-29; MHPOWOS presumably larger than
  the old 10 Ah cell).
- **LiDAR connector type** (both ends): no matching 4-pin connector-to-jumper cable currently available,
  exact connector type unknown.
- **Chassis linkage/geometry:** the exact construction of the (currently removed) spring linkage as well as
  wheel geometry (wheel radius, wheelbase, track) have not been finally measured — only reference values from the code are known.
- **ESP GPIO mapping (new board):** **RESOLVED** — confirmed at the bench pin test 2026-07-27/28 (§1.2
  above), matches `documentation/schematics/WIRING_PLAN.md` §1.1. Remaining open items moved to §1.2: encoder counts-per-rev
  calibration, checklist Part A3 (transport/reserved pins) + Part B (Pi interfaces) not yet run, PSU
  current limiting.
- **Final fuse values + conductor cross-sections:** ~~With the battery choice (MHPOWOS 20 Ah), the fusing is being rebuilt
  (main fuse ~30 A slow-blow + fuses per branch, see internal REQUIREMENTS §10 HWR-4)~~ — ⚠️ **CORRECTED
  2026-08-24, re-corrected 2026-08-25: every branch fuse VALUE is settled — `F1` 25 A, `F2` ~20 A,
  `F3` 5 A, `F4` 3 A. The "~30 A" in the struck text is superseded; the `F3` 5 A / `F4` 3 A this
  bullet has carried all along are the user's decision of 2026-08-13 and are RIGHT (see §1.6 — the
  2026-08-24 pass briefly declared them wrong against a stale §9.3 table). What remains open is the
  four placeholder `F?` designators, the MIDI-25 A procurement for `F1`, and the §9.5(5) selectivity
  review.** The **final values** and
  the **conductor cross-sections per circuit** are to be finalized after the consumer load budget (HWA-1) — until then the
  fuse values are **preliminary**. The old 20 A main fuse will be replaced.
- **Component selection switching architecture + current buffer:** switches/E-stop/contactors and the Pi/ESP decoupling exist so far
  only at the block level; concrete models/ratings are still to be selected (§10 of the internal requirements document HWA-10) — after that the
  wiring diagram will be updated from block to component level.

---

*Sources: this file is now the primary source for the disassembly findings 2026-07-13; the hardware
rework requirements, analysis and open points live in Section 10 (`HWR-*`/`HWA-*`) and Section 7
(`OP-H*`) of the internal requirements document — tracked internally, not in this repository, and
formerly a separate hardware-rework document merged in on 2026-07-14. Further sources, also
internal: the pin mappings and the servo/arm calibration values recorded before the disassembly,
and the superseded legacy as-built version of 2026-07-01.*
