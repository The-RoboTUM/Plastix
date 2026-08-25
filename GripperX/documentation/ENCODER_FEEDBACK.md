# Encoder Feedback — Implementation & Design Record (HWR-10)

**Purpose of this document:** this record preserves *what is actually in the repo and on the
board* for the wheel-encoder feedback path, so the working implementation is not lost or
re-derived from scratch, and so a future implementer does not reintroduce a defect that was
already found and fixed once. It is descriptive, not aspirational — every claim below is tied
to a file:line in the current working tree (`Software/microros/firmware/`,
`Software/ros2/src/gripperx_hardware_interface/`) or to a dated, sourced measurement in
the internal requirements document (tracked internally, not in this repository).

**This revision (2026-08-20) is a full pass, not an incremental one.** The document previously
described a 12-value state layout, an `updateEncoder()` function called once per publish, and
an unmeasured, placeholder counts-per-revolution constant. All three of those are now wrong —
not stale in detail, but describing a contract the firmware no longer has. Every section below
has been re-verified against the tree; superseded claims are kept, marked, and explained rather
than silently deleted, per project convention.

## Status at a glance (2026-08-20)

- **Production firmware (env `esp32-s3`) is flashed and hardware-verified.** This supersedes the
  document's previous headline claim that only the bench instrument (`pin_test.cpp`) had been
  run. Two commits since then put the production build on the board and exercised it:
  `84d9a16` (2026-08-19, FR-11 items 5/6 — the provenance block) and the 2026-08-20 rate work
  (`4bd6060` decoupling sampling from publishing, `bab29d3` raising the feedforward gain). See
  §1.2/§1.3 for what each changed and the hardware evidence cited in
  the internal requirements document FR-11.
- **`COUNTS_PER_OUTPUT_REV` is measured, not a placeholder.** It is **3200**
  (`ENCODER_CPR_PER_CHANNEL 16.0 × 4 × GEAR_RATIO 50.0`), measured 2026-08-13 by hand-rolling two
  wheels exactly 10 output revolutions on the assembled robot, and independently confirmed to
  about 1 % by three later hand-rolling measurements taken 2026-08-19/2026-08-20 for an unrelated
  purpose (the `wheel_radius` calibration and a 10-revolution check). See §1.2.
- **Per-wheel `dirSign` is wired into the four `attachEncoder()` calls.** The document previously
  flagged this as outstanding (§4 item 2 below). `main.cpp` now passes `ENC_DIR_FL/FR/BL/BR`
  explicitly, all confirmed `+1` on the assembled robot 2026-08-13. See §1.3.
- **The sampling/publish contract was reworked 2026-08-20 and the function that used to do this
  was renamed.** `updateEncoder()` — called once per publish, over whatever jittery interval the
  publish loop happened to produce — is gone. It is now `sampleEncoder()`, called from every
  `loop()` iteration and self-throttled to 200 Hz; the publish loop only reads the result. The
  rename is deliberate (source comment, `motor_controller.hpp:128-130`): a caller that kept
  calling something named `updateEncoder()` once per publish would silently reintroduce the
  defect this change fixed. See §1.2.
- **`/hw/joint_states` is 16 values, not 12.** `[0-3]` reserved/zero (steering — the ESP32 has
  no steering sensor), `[4-7]` wheel velocity, `[8-11]` wheel position, `[12-15]` per-wheel
  **provenance** (`EncoderStatus`) — new since the document was last written, and now the
  mechanism that lets a consumer tell a real measurement from an echo of the command. See
  §1.2/§1.3 and the new §1.4.
- **§4's TODO list is done.** All four items are closed; see §4 for what closed each one and the
  commit/measurement behind it.

---

## 1. Production firmware integration (HWR-10)

Applies to PlatformIO env `esp32-s3` (`Software/microros/firmware/platformio.ini`,
`build_src_filter = +<*> -<pin_test.cpp>` — i.e. everything except the bench instrument).

### 1.1 `QuadEncoder` — HW PCNT x4 quadrature decoder

Files: `Software/microros/firmware/include/quad_encoder.hpp`,
`Software/microros/firmware/src/quad_encoder.cpp`.

One `QuadEncoder` instance drives **one PCNT unit** end-to-end for one wheel's A/B pair. The
ESP32-S3 has exactly 4 PCNT units (`PCNT_UNIT_0..3`), one per drive wheel
(`quad_encoder.hpp:10`). PCNT is reached through the GPIO matrix, so encoder pins are not
restricted to any particular GPIO set (`quad_encoder.hpp:12-15`).

**`begin(unit, pinA, pinB)`** (`quad_encoder.cpp:9-72`):
- Both encoder pins configured plain `INPUT`, **no pull-ups** — the encoders are 3.3 V
  push-pull Hall outputs (`quad_encoder.cpp:13-15`).
- **Channel/control setup for x4 decode**, one PCNT unit with two channels:
  - Channel 0: `pulse_gpio_num = pinA`, `ctrl_gpio_num = pinB`
    (`quad_encoder.cpp:18-29`). `pos_mode = PCNT_COUNT_INC` (rising edge of A → +1),
    `neg_mode = PCNT_COUNT_DEC` (falling edge of A → −1); `lctrl_mode = PCNT_MODE_KEEP` (B
    low → keep direction), `hctrl_mode = PCNT_MODE_REVERSE` (B high → reverse direction).
  - Channel 1: `pulse_gpio_num = pinB`, `ctrl_gpio_num = pinA`
    (`quad_encoder.cpp:32-43`) — mirror image of channel 0 (counts both edges of B, A is the
    control line), which is what completes the count to x4 (both edges of both channels).
  - Both channels share `counter_h_lim = kHighLimit = 30000`,
    `counter_l_lim = kLowLimit = -30000` (`quad_encoder.hpp:49-50`,
    `quad_encoder.cpp:27-28,41-42`) — symmetric software limits well inside the int16 counter
    range.
- **Glitch filter:** `pcnt_set_filter_value(unit_, 1000)` + `pcnt_filter_enable(unit_)`
  (`quad_encoder.cpp:47-48`) — ~12.5 µs at 80 MHz APB clock, against PWM/EMI-induced bounce;
  same value as the bench instrument (§2).
- **64-bit overflow accumulation:** `PCNT_EVT_H_LIM` / `PCNT_EVT_L_LIM` are enabled
  (`quad_encoder.cpp:52-53`); the shared PCNT ISR service is installed once process-wide
  (`g_isr_service_installed` guard, `quad_encoder.cpp:3-7,58-64`) and each unit registers its
  own handler (`pcnt_isr_handler_add`, `quad_encoder.cpp:65`). `overflowIsr()`
  (`quad_encoder.cpp:90-96`) adds `±kHighLimit`/`kLowLimit` to a `volatile int64_t overflow_`
  each time the 16-bit hardware counter hits a limit and auto-resets — this is what lets
  `count()` return a 64-bit, non-wrapping accumulated position across an arbitrary number of
  wheel revolutions instead of wrapping at ~30000 counts.
- **Race-safe read:** `count()` (`quad_encoder.cpp:74-88`) re-samples `overflow_` before and
  after the (non-atomic) hardware-counter read and loops until the two samples agree,
  rejecting the race where an overflow ISR fires between the two reads (which would otherwise
  produce a spurious ±`kHighLimit` spike in the derived velocity).
- **Checked return values, added in `84d9a16` (2026-08-19).** Every one of `begin()`'s eleven
  `esp_err_t`-returning calls is now checked; on the **first** failure the function returns
  `false` immediately, **before `pcnt_counter_resume()`**, so a half-configured unit can never
  count (`quad_encoder.cpp:29,43,47-48,52-53,55-56,62,65,69`). This is what makes
  `EncoderStatus::InitFailed` (§1.4) a real, reachable state rather than a value nothing can
  produce: previously every `esp_err_t` was discarded, `begin()` returned `void`, and a
  rejected PCNT configuration produced a silently dead decoder that `MotorController` still
  reported as initialised — the count stayed 0, the derived RPM stayed 0, and nothing said the
  number was not a measurement. That was the actual, hardware-observed failure mode this
  change closed (D14 in the internal requirements document, see §1.4).

Uses the legacy ESP-IDF PCNT API (`driver/pcnt.h`) — deprecated but still present in the
arduino-esp32 3.x / IDF 5.x core; deprecation warnings are expected and harmless
(`quad_encoder.hpp:17-19`).

### 1.2 `MotorController` — encoder-derived velocity, position and provenance

Files: `Software/microros/firmware/include/motor_controller.hpp`,
`Software/microros/firmware/src/motor_controller.cpp`.

- **`attachEncoder(pcnt_unit_t unit, uint8_t encPinA, uint8_t encPinB, int8_t dirSign = 1)`**
  (`motor_controller.hpp:122`, `motor_controller.cpp:40-46`) — stores the unit/pins/sign and
  sets `hasEncoder_ = true`. Must be called *before* `begin()`. `dirSign` (±1) aligns the
  measured sign with the motor **command** frame (a positive `setTargetRPM` should read back
  as positive measured RPM); it is a wiring fact that can only be settled at the bench, and
  defaults to `+1` if not passed explicitly (`motor_controller.hpp:116-122`). **As of `main.cpp`
  it IS passed explicitly at every call site** — see §1.3; the default only matters for a
  caller that omits the argument.
- **`begin()`** (`motor_controller.cpp:10-38`) additionally starts the PCNT decoder exactly
  once, guarded by `encInited_` so repeated `begin()` calls (micro-ROS reconnects) do not
  re-init the encoder and position keeps accumulating across reconnects
  (`motor_controller.cpp:19-34`). Encoder pins are physically separate from the motor
  PWM/DIR pins, so starting the decoder here never drives a motor. On success it resets the
  sample ring and sets status to `LiveUnconfirmed` (never straight to `Live` — see §1.4); on
  failure it sets `InitFailed` and does not touch the ring
  (`motor_controller.cpp:20-33`).
- **`sampleEncoder()`** (`motor_controller.cpp:48-107`) — **this is `updateEncoder()` renamed**,
  and the rename is deliberate, not cosmetic (`motor_controller.hpp:124-130`): *"It was called
  `updateEncoder()` while that coupling existed; the rename is deliberate, so no caller keeps
  the old once-per-publish contract by accident."* Contract, verified against the source:
  - **Call it from every `loop()` iteration.** The function self-throttles: it reads
    `micros()`, computes time since the last stored sample, and returns immediately (a few
    microseconds) if less than `ENC_SAMPLE_INTERVAL_US` (5000 µs → **200 Hz**) has elapsed
    (`motor_controller.cpp:51-55`, `motor_controller.hpp:72`). It must **not** be tied to the
    state-publish cycle — that coupling was the defect this change fixed (see below).
  - **Ring buffer, not a single last-reading.** A per-wheel `EncSample{ int64_t count;
    uint32_t tUs; }` ring of `ENC_SAMPLE_SLOTS = 32` entries
    (`motor_controller.hpp:74,169-176`) stores every accepted sample's absolute count and
    `micros()` timestamp.
  - **Position** is derived from the **absolute** accumulated count on every accepted sample
    (not integrated deltas, so no integration drift), pre-multiplied by `encDirSign_` into the
    motor command frame: `positionRad_ = (encDirSign_ * count / COUNTS_PER_OUTPUT_REV) * 2π`
    (`motor_controller.cpp:86`).
  - **Velocity** is a **first difference over a sliding window of at least
    `ENC_WINDOW_US` (100000 µs → 100 ms)**, walking back through the ring to the oldest sample
    still within that window (or the oldest available one, if the ring does not yet reach that
    far back — e.g. the first ~100 ms after `begin()` or right after a gap reset), so the window
    grows into its nominal length instead of producing a spike
    (`motor_controller.cpp:94-101`). `dt` is **measured in microseconds** via `micros()` between
    the two samples used, not assumed — an irregular sample cadence changes the window
    *length*, never the correctness of the estimate (`motor_controller.cpp:98,104-106`).
    `measuredRPM_ = revs * 60000000.0 / dt`, `revs` computed the same way as position (absolute
    count delta over `COUNTS_PER_OUTPUT_REV`, signed by `encDirSign_`).
  - **`LiveUnconfirmed → Live` promotion** happens here, on the first observed count change
    since `begin()` — see §1.4.
  - **Gap handling:** if the time since the last sample exceeds `ENC_MAX_GAP_US` (250000 µs —
    e.g. a micro-ROS reconnect during which `loop()` was not sampling), the whole ring history
    is discarded and `measuredRPM_` reset to 0, because every stored sample would otherwise be
    treated as a window boundary spreading the gap's counts over a window that never ran
    (`motor_controller.cpp:71-77`, `motor_controller.hpp:75-78`).
  - **Why this rework happened, in the source's own words** (`motor_controller.hpp:36-71`):
    under the old once-per-publish contract, a "measurement" existed only every 114-121 ms
    (the jittery old publish period, see §1.3) and its `dt` came from `millis()` — 1 ms
    quantisation over a 115 ms window is 0.87 % velocity error from the clock alone, versus
    ~0.001 % for `micros()` over 100 ms. The new **100 ms** window is a deliberate trade,
    justified with numbers in the header rather than assumed: count quantisation (one count =
    2π/3200 rad) contributes 0.46 % of the 4.286 rad/s reference speed at 100 ms — worse than a
    naively shorter window would suggest is available, because a 33 ms window would be 1.4 %,
    *noisier* than the estimator it replaced. Combining the two error terms in quadrature: old
    0.041 rad/s (0.96 %) at up to 178 ms total latency, new 0.0196 rad/s (0.46 %) at up to 83 ms
    — **the count term alone got 15 % worse** (shorter window than the old jittery one), stated
    honestly rather than only citing the terms that improved; removing the `dt` term more than
    pays for it, because that term scaled *with* the reading rather than being a fixed floor.

### 1.3 `main.cpp` — pin map, attach, state layout and the publish scheduler

File: `Software/microros/firmware/src/main.cpp`.

- **Encoder pin map** (`main.cpp:28-35`), one PCNT unit per wheel — unchanged since the
  original bench work:

  | Wheel | A pin | B pin | PCNT unit |
  |---|---|---|---|
  | FL | IO8 | IO9 | `PCNT_UNIT_0` |
  | FR | IO10 | IO11 | `PCNT_UNIT_1` |
  | BL | IO12 | IO13 | `PCNT_UNIT_2` |
  | BR | IO14 | IO21 | `PCNT_UNIT_3` |

  Matches `documentation/schematics/WIRING_PLAN.md` §1.1/§1.3/§5 and the bench pin map
  (§2, §3).
- **Per-wheel `dirSign`, now wired in** (`main.cpp:70-73`): `ENC_DIR_FL/FR/BL/BR`, each `1`.
  **This closes what the document previously listed as open (old §4 item 2).** The comment
  block above these defines (`main.cpp:37-69`) records the bench procedure and the 2026-08-13
  confirmation on the assembled robot: each wheel rolled forward *by hand* gave the physical
  direction ↔ count-direction mapping (FL/BL forward = counts up, FR/BR forward = counts down);
  each wheel was then driven with `DIR HIGH` and read back, giving `DIR HIGH` = FL/BL backward,
  FR/BR forward. Since `applyPwmDir()` sets `DIR LOW` for a positive command — the opposite of
  that test — a positive command turns every wheel in the direction that counts up, hence
  `dirSign = +1` on all four. The comment explicitly notes this chain is **entirely in the motor
  frame** and independent of `ROBOT_FRAME_WHEEL_SIGN` (below) and of the URDF.
  It also explicitly flags that the bench entry recorded in this document's own §3 claimed the
  *opposite* count directions — **superseded**, because miswired encoder cables were found and
  corrected in the session that produced this measurement; see §3.
- **Attach in `startMotors()`** (`main.cpp:261-269`):
  ```cpp
  motor_fl.attachEncoder(PCNT_UNIT_0, ENC_FL_A, ENC_FL_B, ENC_DIR_FL);
  motor_fr.attachEncoder(PCNT_UNIT_1, ENC_FR_A, ENC_FR_B, ENC_DIR_FR);
  motor_bl.attachEncoder(PCNT_UNIT_2, ENC_BL_A, ENC_BL_B, ENC_DIR_BL);
  motor_br.attachEncoder(PCNT_UNIT_3, ENC_BR_A, ENC_BR_B, ENC_DIR_BR);
  motor_fl.begin(); motor_bl.begin(); motor_br.begin(); motor_fr.begin();
  ```
  All four calls now pass a `dirSign` argument explicitly — this document previously said none
  of them did (superseded, see above).
- **`hw/joint_states` layout — 16 values, not 12** (`main.cpp:97-133`):
  ```
  #define NUM_STATE_JOINTS 16
  IDX_FL=4, IDX_FR=5, IDX_BL=6, IDX_BR=7                       // wheel velocity (rad/s)
  IDX_FL_POS=8, IDX_FR_POS=9, IDX_BL_POS=10, IDX_BR_POS=11     // wheel position (rad)
  IDX_FL_ENC=12, IDX_FR_ENC=13, IDX_BL_ENC=14, IDX_BR_ENC=15   // EncoderStatus provenance (new)
  ```
  Indices 0-3 are **reserved and structurally always zero** — not a gap to be filled later. The
  ESP32 has no steering sensor; the real steering measurement is `/hw/steer_states`, published
  by `steer_servo_node` on the Pi and merged by `gripperx_hardware_interface` (FR-10). The only
  value this firmware could ever place in 0-3 would be an echo of the steering command, which is
  exactly what FR-2 rejects as feedback (`main.cpp:97-108`). Indices 4-7 keep the pre-existing
  8-value minimum contract that the Pi `gripperx_hardware_interface::read()` still accepts as a
  floor (size check `>= 8`; §1.4/Pi side below). Indices 8-11 (position) and 12-15 (provenance)
  are strictly appended, so an older or shorter publisher continues to interoperate.
- **Encoder sampling runs every loop iteration, decoupled from publishing** (`main.cpp:380-383`):
  ```cpp
  if (motors_ok) {
      motor_fl.sampleEncoder(); motor_fr.sampleEncoder();
      motor_bl.sampleEncoder(); motor_br.sampleEncoder();
  }
  ```
  This call sits directly in `loop()`, ahead of the publish-deadline check, and runs on **every**
  pass through `loop()` (self-throttled internally to 200 Hz, per §1.2) — not only when a
  publish is due.
- **Publish scheduler, fixed phase on `micros()`** (`main.cpp:169-181,391-402`):
  `STATES_PUBLISH_US = 33333` (**30 Hz**), chosen to match the Pi's `controller_manager`
  `update_rate` (`gripperx_control/config/ros2_controllers.yaml` L3) so `read()` sees a fresh
  frame per control cycle. The deadline advances by exactly one period rather than being
  re-based on the current time, so the publish interval cannot accumulate the loop's overshoot
  the way the old `STATES_PUBLISH_MS 100` (nominal 10 Hz) did; a missed cycle re-bases the
  phase instead of catching up, so a stall cannot produce a burst of frames onto a link that is
  already the binding constraint. `EXEC_SPIN_MS = 5` bounds how long `rclc_executor_spin_some`
  may sit idle, so publishing and sampling are no longer starved by a 100 ms sleep inside the
  same loop as the old scheduler was.
  - **Measured: 29.999 Hz** on hardware (the internal requirements document FR-11 rate block; min 0.022 s,
    max 0.044 s, std dev 0.0024 s — `/hw/joint_states` itself; `/joint_states` downstream also
    measured 29.999 Hz).
  - **Old rate — the two figures are BOTH correct; they are two separate measurement runs.**
    `main.cpp:139` and `Software/microros/firmware/README.md:71` cite **8.72 Hz / 114-121 ms**,
    which is the archived measurement of **2026-08-17**. the internal requirements document's FR-11 rate
    block cites **8.496 Hz / 117-124 ms, std dev 0.0020 s**, which is the fresh baseline taken on
    **2026-08-20 immediately before flashing**, so that the improvement was measured against a
    same-day number rather than an archived one. Same defect, same firmware, different day and
    different run. The ~3 % gap is ordinary run-to-run variation of a rate that was never stable
    by construction — it was whatever the 100 ms `spin_some` and the once-per-second ping
    happened to produce on that boot. Neither source is wrong and neither needs correcting;
    quote the date along with the number.
  - **Link budget** (worked out in full at the `STATES_PUBLISH_US` definition,
    `main.cpp:144-168`): 115200 8N1 = 11520 B/s. One 16-value state frame (with XRCE/serial
    framing and a +10 % byte-stuffing allowance) ≈ 175.8 B ≈ 15.26 ms; one 8-value command
    frame ≈ 105.4 B ≈ 9.15 ms. Treating the (full-duplex) UART as if it were shared, worst case:
    states 45.8 % + commands 27.5 % + a 1 Hz ping ≈ 1.7 % = **75.0 % occupancy at 30 Hz, 25 pp
    margin**. 40 Hz would be 90.2 % (9.8 pp margin); 50 Hz is not feasible (105.5 %). The margin
    is kept deliberately wide because one budget line is unquantified: the publisher QoS is
    `RELIABLE`, and XRCE-level acknowledgement traffic has never been measured. Raising the baud
    rate is the real headroom but is a coordinated firmware + `gripperx-agent.sh` change, not
    part of this work.
- **Publish block, per-wheel values** (`main.cpp:404-430`):
  - Velocity, `IDX_FL/FR/BL/BR`: `ROBOT_FRAME_WHEEL_SIGN * rpmToRad(±getRPM())`, FL/BL negated
    for the physical mirroring, same convention as `cmdCb()`.
  - Position, `IDX_*_POS`: same FL/BL mirroring and the same `ROBOT_FRAME_WHEEL_SIGN`, so
    integrated position runs with the velocities rather than against them.
  - Provenance, `IDX_*_ENC`: `(double)(uint8_t)getEncoderStatus()` — **no sign and no frame
    conversion applied**, deliberately, because a status code is not a physical quantity;
    multiplying it by `ROBOT_FRAME_WHEEL_SIGN` would turn `Live` (3) into `-3`.
  - If `motors_ok` is false the whole 16-value array stays at the zero fill applied earlier in
    the loop, which decodes as `EncoderStatus::NoEncoder` on every wheel — the one code the zero
    fill can produce is the one that claims the least (`main.cpp:404,431-433`).

### 1.4 `EncoderStatus` — per-wheel provenance (FR-11 items 5/6)

**New since this document was last written**, and now load-bearing: HWR-30a's stall detector
arms only on it, and the Pi-side hardware interface republishes it as a latched safety-relevant
topic.

`getRPM()` (`motor_controller.cpp:179-185`) is, and always has been:
```cpp
return encInited_ ? measuredRPM_ : targetRPM_;
```
i.e. it is either a real measurement or a verbatim echo of the last commanded value, and until
this mechanism existed nothing in the data said which — a plausible, setpoint-tracking value is
what *both* branches look like. `EncoderStatus` (`motor_controller.hpp:101-106`) is the missing
bit, one code per wheel, published on `hw/joint_states[12..15]` in FL, FR, BL, BR order:

| Value | Name | Measurement? | Meaning |
|---|---|---|---|
| 0 | `NoEncoder` | no | `attachEncoder()` was never called for this wheel |
| 1 | `InitFailed` | no | encoder attached, but the PCNT unit rejected its configuration |
| 2 | `LiveUnconfirmed` | **yes, but see below** | PCNT configured and running, no count change seen since boot |
| 3 | `Live` | yes | counts have actually moved — the decoder is provably working |

- **The order is deliberately monotone in confidence.** `NoEncoder` is `0` so that a zero-filled
  state array — or any older/foreign publisher that never sends this block — degrades to "not a
  measurement" rather than to "valid" (`motor_controller.hpp:99-100`). Consumers may rely on the
  ordering (`>= LiveUnconfirmed` as the general measurement test); do not renumber.
- **Promotion `LiveUnconfirmed → Live` is one-way and unconditional on the first observed count
  change** (`motor_controller.cpp:59-69`) — no threshold, no time window. There is deliberately
  **no downgrade path**: a downgrade would need to distinguish "commanded to move but not
  counting" from "genuinely stationary", which requires comparing the command against the count
  over thresholds nobody has measured, and doing that here would make a stationary robot report
  a dead encoder every time it stood still. That detection is a different mechanism entirely —
  HWR-30a (below) — not a job for `EncoderStatus` itself.
- **`LiveUnconfirmed` is `>= LiveUnconfirmed` and therefore counts as "measured" by the general
  test — but it is NOT usable as evidence that an encoder works.** It means only *"configured and
  running, no count change seen yet"*, which is bit-identical to the actual fault a stall
  detector needs to catch (a dead encoder reporting a plausible, unchanging value). This is why
  **HWR-30a's stall detector arms strictly on `>= Live`, not `>= LiveUnconfirmed`** — deliberately
  *stricter* than FR-11's own measurement test
  (`Software/ros2/src/gripperx_swerve_controller/src/stall_detector.cpp:170-179`, comment: *"The
  test is `>= kStallProvenanceLive`, i.e. LIVE only — STRICTER than FR-11's `>= LIVE_UNCONFIRMED`
  measurement test. LIVE_UNCONFIRMED means 'begin() succeeded and no count change has been seen
  yet', which is indistinguishable from the very fault this detector looks for. Arming on it
  would make the detector trip on its own uncertainty."*). Any other consumer that needs to
  distinguish "an encoder that has actually proven itself" from "an encoder that merely
  initialised" needs the same `>= Live` test, not `>= LiveUnconfirmed`.
- **This is testable without motor power.** The encoders are 3.3 V push-pull Hall outputs wired
  **directly to the ESP32, bypassing the motor drivers entirely**
  (`documentation/schematics/WIRING_PLAN.md` §1.3: *"Encoders are not routed through the
  drivers (straight to the ESP32, GND on the ESP32 (`A1`) ground node ...) and are unaffected"*).
  Turning a wheel by hand with the driver unpowered promotes it `LiveUnconfirmed → Live`; this
  was verified on hardware 2026-08-19 (`[2,2,3,3]` observed mid-test → `[3,3,3,3]` after all
  four wheels were turned by hand). **This same property is what let a real hardware fault be
  diagnosed cleanly on 2026-08-20 (OP-H14):** when driver `A3` (front) produced no drive on
  either channel, FL and FR still promoted to `Live` when turned by hand — because the encoders
  never depend on the driver being alive — which is what let the fault be attributed to *"no
  drive"* rather than *"no feedback"* and correctly localised to the shared `A3` supply cable
  rather than to four independently-suspect wheels.

**On the Pi side, `/hw/wheel_feedback_valid` is LATCHED — a volatile subscriber gets nothing.**
`GripperXInterface` republishes the four provenance codes on `/hw/wheel_feedback_valid`
(`std_msgs/Int32MultiArray`, FL FR BL BR) with QoS `KeepLast(1)` + `RELIABLE` +
**`TRANSIENT_LOCAL`**
(`Software/ros2/src/gripperx_hardware_interface/src/gripperx_interface.cpp:344-352`), and only on
change. A subscriber created with a volatile (the default) profile connects but never receives
anything until the next change — which, once the four wheels reach `Live` and stay there, may be
never for the lifetime of that subscriber. **This is reported here because it has bitten twice in
practice**, not as a hypothetical: any node that needs to know wheel-feedback provenance —
including a freshly (re)started one — must subscribe with `TRANSIENT_LOCAL` durability to receive
the latched state, exactly as `GripperXInterface` publishes it
(`Software/ros2/src/gripperx_hardware_interface/INTERFACE.md:111-112`). *This could not be
independently corroborated from a second incident record inside this worktree at the time of
writing — recorded on instruction, and worth a WIRING_PLAN/INTERFACE.md cross-check if a third
occurrence turns up.*

**On the Pi side, `read()` consumes all three appended blocks** —
`Software/ros2/src/gripperx_hardware_interface/src/gripperx_interface.cpp:622-692` — length-gated
the same way as the firmware documents its own contract: `>= 8` accepted at all (below that,
`ERROR` and return); `>= 12` (`kNumStateValuesWithWheelPositions`) required to fill the wheel
position state interfaces, else they are left at their previous value rather than forced to zero
(so a mid-run firmware downgrade cannot produce a position jump); `>= 16`
(`kNumStateValuesWithProvenance`) required to decode the provenance block, else all four wheels
are reported `UNKNOWN` (`-1` — negative on purpose, distinct from any code the firmware itself
sends). **This closes what the document previously listed as open (old §4 item 3).**

---

## 2. Bench instrument that proved the approach (`pin_test.cpp`)

File: `Software/microros/firmware/src/pin_test.cpp` (PlatformIO env `pintest`,
`build_src_filter = +<pin_test.cpp> -<main.cpp> -<motor_controller.cpp>` — no micro-ROS, no
`MotorController`/`QuadEncoder` classes; a self-contained hand-rolled PCNT setup that mirrors
the production logic for verification purposes). **Unchanged in substance since this document
was last written** — the only commit touching this file since (`03c00d3`) added an unrelated
`pd` serial command to verify pull-down resistors, which shifted the line numbers below but not
the encoder logic itself.

`setupPcntPair(pcnt_unit_t unit, uint8_t pinA, uint8_t pinB)` (`pin_test.cpp:399-430`):
- Same two-channel x4-decode scheme as `QuadEncoder::begin()`: channel 0
  pulse=A/ctrl=B/`INC` on rising A/`DEC` on falling A/`KEEP` on B-low/`REVERSE` on B-high
  (`pin_test.cpp:399-410`); channel 1 pulse=B/ctrl=A, mirrored controls
  (`pin_test.cpp:412-423`).
- **Difference from production:** uses the full int16 hardware limits directly
  (`counter_h_lim = 32767`, `counter_l_lim = -32768`, `pin_test.cpp:408-409,421-422`) with
  **no overflow ISR / 64-bit accumulation** — `printEncoderCounts()`
  (`pin_test.cpp:436-447`) reads the raw 16-bit `pcnt_get_counter_value()` directly each
  print cycle and just derives a fwd/rev indicator from the delta. This is adequate for a
  bench probe (bounded hand-turn / short driven runs) but would wrap silently on a long
  continuous run — the production `QuadEncoder` class's overflow handling (§1.1) is the fix
  for that, not present here.
- Same glitch filter value (1000, ~12.5 µs @ 80 MHz APB) as production
  (`pin_test.cpp:425-428`, comment explicitly notes it matches).
- Pin map used for the bench test (`pin_test.cpp:117-121`): FL 8/9, FR 10/11, BL 12/13,
  BR 14/21 → `PCNT_UNIT_0..3` — identical to the production map in `main.cpp` (§1.3).
- Encoder pins configured plain `INPUT`, no pull-ups (`pin_test.cpp:457-460`), consistent
  with the 3.3 V push-pull encoder supply.
- The instrument also exercises drive PWM/DIR pins independently (`l/on/off`, `m <wheel>
  <f|r> [duty]`, `s`/stop commands), the new `pd` pull-down check, and prints live encoder
  counts + direction every `PRINT_INTERVAL_MS` (200 ms, `pin_test.cpp:124,481-486`), which is
  how the hand-turn + driven encoder reads were performed and observed live on the serial
  monitor — including the 2026-08-13, 2026-08-19 and 2026-08-20 measurements cited throughout
  this document.

---

## 3. Bench-confirmed facts (2026-07-28, superseded/extended below)

Recorded here for traceability; the authoritative, longer-form record is
the internal requirements document HWR-10, FR-11 (items 5/6 and the drive-feedback-rate block), and
OP-H3/OP-H9/HWR-8/HWR-9.

- All 4 encoders (FL/FR/BL/BR) read correct quadrature counts in both directions of rotation,
  with no cross-talk between channels, on the pin map in §1.3/§2.
- No pull-ups/pull-downs needed or used; encoders confirmed **3.3 V push-pull** outputs,
  directly compatible with the ESP32-S3's non-5V-tolerant GPIOs.
- **Per-wheel direction convention — the encoder half below is SUPERSEDED, see the
  2026-08-13 block that follows. Kept for the record, do not use it.**
  - **LEFT (FL, BL):** DIR pin HIGH → physical **backward** rotation; encoder count
    increasing (positive) corresponds to **backward**.
  - **RIGHT (FR, BR):** DIR pin HIGH → physical **forward** rotation; encoder count
    increasing (positive) corresponds to **forward**.
  - This is consistent with (and confirms the correctness of) the firmware's existing
    left-side command mirroring in `main.cpp` (`cmdCb()` negates FL/BL, and the same negation
    is applied to velocity/position in the publish loop — §1.3).
- This resolves the wiring-polarity unknown that `motor_controller.hpp` originally flagged as
  "BENCH-CONFIRM per wheel" — the per-wheel `dirSign` is now bench-known **and, since
  2026-08-13, wired into `main.cpp`'s `attachEncoder()` calls** (§1.3 — the document
  previously said this transcription step was still outstanding; it is not, see §4).

### Re-measured 2026-08-13 on the assembled robot — all four encoder directions INVERTED

Measured with the `pintest` firmware by rolling each wheel forward by hand with torque
off; nothing was commanded. Encoder→wheel mapping came out matching the firmware labels
on all four (worth checking, since the steering ids turned out to be a 3-cycle against
their config and the rear driver channels are documented as swapped).

| wheel | forward rotation counts | old entry above |
|---|---|---|
| FL | **increasing** | decreasing (said increasing = backward) |
| FR | **decreasing** | increasing |
| BL | **increasing** | decreasing |
| BR | **decreasing** | increasing |

All four are inverted against the bench entry — uniformly, which is the signature of a
consistently swapped A/B pair in the rebuilt harness (swapping A and B reverses the
counting direction of a quadrature decoder). Miswired encoder cables were in fact found
and corrected during this session, so the table above describes the corrected state.

**Consequence, as it stood at the time:** the `dirSign` values this section previously declared
"bench-known" were not directly usable without a companion motor-lead check, because the
measurement only fixes one half of the chain (physical rotation ↔ count direction) and the
other half (whether `DIR LOW` drives a wheel forward or backward) is a separate motor-lead fact.
**This has since been resolved and closed** — `main.cpp`'s comment block (§1.3) records the
completed two-step bench procedure (hand-roll to get rotation↔count, then drive with `DIR HIGH`
to get DIR↔rotation) and its result, `dirSign = +1` on all four wheels, confirmed 2026-08-13. The
open question this section originally ended on is answered; see §1.3, not here, for the current
value and its derivation.

**Counts-per-revolution — measured the same session, 2026-08-13, and independently confirmed
since.** See §1.2 for the full number (`COUNTS_PER_OUTPUT_REV = 3200`, `= 16 × 4 × 50`) and the
~1 % independent confirmation from later, unrelated hand-rolling measurements
(2026-08-19/2026-08-20). This document previously (§4 item 1, below) listed the constant as an
unmeasured placeholder (`ENCODER_CPR_PER_CHANNEL = 11.0`, `COUNTS_PER_OUTPUT_REV = 2200`);
trusting that placeholder would have **overstated** reported speed and distance by about 45 %,
because dividing real counts by too small a number inflates the result — one true wheel
revolution (3200 real counts) would have been reported as 3200/2200 ≈ 1.45 revolutions.

---

## 4. Open / TODO — all four items closed

This section is kept, per project convention, rather than deleted now that its contents are
done — each item below states what closed it and where the evidence is.

1. **Counts-per-revolution calibration (real odometry scaling). CLOSED.** Measured 2026-08-13:
   two wheels hand-rolled exactly 10 output revolutions with the `pintest` firmware counting —
   FL 32021 counts / 10 rev = 3202.1, BL 31989 / 10 = 3198.9, mean 3200.5, the two readings
   0.1 % apart and bracketing **3200**, which factors exactly as `16 × 4 × 50`
   (`ENCODER_CPR_PER_CHANNEL = 16.0`, `GEAR_RATIO = 50.0`, `motor_controller.hpp:8-34`).
   **Independently confirmed to about 1 %** by three later hand-rolling measurements taken for
   an unrelated purpose (the `wheel_radius` calibration, the internal requirements document): encoder
   deltas read **−0.86 %** and **−1.14 %** against nominal on 2026-08-19, and **−0.51 %** on a
   2026-08-20 ten-revolution check. No corrected value has been derived from this small,
   consistent shortfall — the constant stays `3200` — but it is recorded as a real, repeated
   observation rather than left unmentioned.
2. **Per-wheel `dirSign` not yet wired in. CLOSED.** `main.cpp`'s four `attachEncoder()` calls
   now pass `ENC_DIR_FL/FR/BL/BR` explicitly (all `+1`, confirmed 2026-08-13 on the assembled
   robot by the two-step bench procedure recorded at `main.cpp:37-69` and summarised in §1.3).
3. **Pi-side `gripperx_hardware_interface::read()` does not consume the new position indices.
   CLOSED.** `read()` now fills `hw_positions_` for wheel joints from indices 8-11 when the
   message is long enough (`gripperx_interface.cpp:662-664`), and decodes indices 12-15 into
   per-wheel provenance (`gripperx_interface.cpp:634-638`) — see §1.4.
4. **Production node integration untested as a unit. CLOSED.** The production firmware
   (`env:esp32-s3`) has been flashed and run with micro-ROS active on the assembled robot:
   `84d9a16` (2026-08-19) flashed and hardware-verified the provenance block (16 values arriving,
   `LiveUnconfirmed → Live` promotion observed on real hardware, latched-topic behaviour
   verified against late subscribers); the 2026-08-20 rate work (`4bd6060`, `bab29d3`) flashed
   and hardware-verified the decoupled 200 Hz sampling / 30 Hz publish contract (measured
   29.999 Hz, see §1.3). See the internal requirements document FR-11 for the full acceptance record.

---

## References

- Requirement: the internal requirements document HWR-10 ("Encoder wiring (on ESP32, quadrature) +
  encoder odometry"), FR-11 (per-wheel velocity feedback, items 5/6 provenance, and the
  2026-08-20 drive-feedback-rate block with the link budget and estimator arithmetic), and the
  related OP-H3/OP-H9/HWR-8/HWR-9/HWR-30/HWR-30a/D14 entries.
- Wiring: `documentation/schematics/WIRING_PLAN.md` §1.1/§1.3/§5 (encoder pin map, 3.3 V
  supply, no level shifting, encoders wired directly to the ESP32 bypassing the motor drivers).
- Pi-side contract: `Software/ros2/src/gripperx_hardware_interface/INTERFACE.md` and
  `Software/ros2/src/gripperx_hardware_interface/src/gripperx_interface.cpp`.
- Firmware contract: `Software/microros/firmware/README.md` (kept consistent with this
  document as of 2026-08-20).
- Origin of the PCNT decode scheme: `Hardware_Test/bench_tests/encoder_test/encoder_test.ino`
  (original ESP32-WROOM-32 sketch, ported to the S3's 4 PCNT units).
