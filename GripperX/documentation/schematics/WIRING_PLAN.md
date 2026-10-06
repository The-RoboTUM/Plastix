# GripperX — Target Wiring Plan (Soll-Verkabelung)

**Status: PLAN — pin-assignment target for the HWR-8 rewire at reassembly.**
**User decisions 2026-07-19 (see §7):** Pi-link = Option A (UART bridge) · LIDAR_EN default ON ·
RGB LED kept (GPIO48 conditional spare only) · board silkscreen/PSRAM verify at bench.
This document defines the intended (`Soll`) pin assignment for every required connection on the
`esp32-drive` board and the Raspberry Pi 5. The KiCad schematic and the QET wiring/terminal
diagram — the graphical twin of this plan — are maintained as source files in the team's internal
working repository, not in this one; a rendered PDF of the QET diagram is published as the
Electronics Overview wiki asset. Both will be updated to follow this plan; where they disagree,
**this plan is authoritative** until the drawings are brought into line.

**This plan supersedes all legacy pin notes**, in particular:
- the HWR-8 note *"FR-PWM GPIO5→GPIO22, BR-PWM GPIO16→GPIO4"* — that is **legacy ESP32-WROOM-32
  numbering**. **GPIO22–25 do not exist on the ESP32-S3.** The rewire targets are resolved to real
  S3 GPIOs below (see §1 and §4).
- the legacy verified WROOM-32 map (slalom test): FL PWM19/DIR13, BL PWM18/DIR21, BR PWM16/DIR26,
  FR PWM5/DIR17. None of these carries over unchanged to the S3.
- the 2026-07-17 bench sketches, which ran on a **WROOM-32 DevKit** using GPIO25/26 (motor) and
  GPIO34/35 (encoder). Those bench pins are **WROOM-32 pins and are invalid on the S3-N16R8**
  (GPIO26 = flash, GPIO34/35 = octal-PSRAM on the R8). The bench proved the encoders, the PWM+DIR
  control path and the PCNT quadrature decode work; it did **not** define the S3 pin map. This plan
  does.

**Board under this plan:** `YD-ESP32-S3-N16R8` (ESP32-S3-WROOM-1, 8 MB octal PSRAM, native USB,
4× HW PCNT). **Not** the old WROOM-32/44P-adapter combination (that mismatch is retired — ASBUILT
§1.1). The YD-ESP32-S3 dev board carries the **GPIO number directly on the silkscreen** (e.g.
`IO4`), so the old "44P adapter" label confusion no longer applies — but physically re-verify each
silkscreen label against this table before soldering (see Open Questions).

**Sources:** the team's internal requirements document (§10 HWR-5/8/10/21/22/23/25/26, §7
OP-H1…H12 — tracked internally, not in this repository); `documentation/ASBUILT.md`;
`Hardware_Test/hw_test/hw_test.ino`,
`Hardware_Test/bench_tests/encoder_test/encoder_test.ino`,
`Hardware_Test/bench_tests/motor_driver_test/motor_driver_test.ino`;
`Software/microros/firmware/src/main.cpp` + `platformio.ini`;
the internal KiCad symbol map (source file, not exported to this repository).
Pinout facts verified against the ESP32-S3 pinout ([Random Nerd Tutorials](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/),
[mculab N16R8 forensics](https://esp32-s3.mculab.workers.dev/index-en)).

---

## 0. Key decisions up front

1. **Drivers = 2× Cytron MDD10A, PWM+DIR (Scheme A)** — DBH-12V removed (OP-H9). Each MDD10A is
   dual-channel; driver-to-wheel grouping is **FRONT/REAR** (user decision 2026-07-29, housing
   redesign): **FRONT driver (MDD10A #1) = FL + FR**, **REAR driver (MDD10A #2) = BL + BR** (was
   LEFT = FL+BL / RIGHT = FR+BR). This is a harness/perfboard grouping only and does **not** change
   the ESP32 per-wheel pin map (§1) — each wheel keeps its PWM/DIR GPIOs and the firmware direction
   signs are unchanged. Per channel: **1 PWM + 1 DIR**. → 4 PWM + 4 DIR total.
2. **Encoders straight to the ESP32, quadrature, 4× HW PCNT, 3.3 V logic** (HWR-10, OP-H3). ESP32-S3
   GPIOs are **NOT 5 V-tolerant** — encoder Vcc = **3.3 V** (bench-confirmed 2026-07-17). → 4×(A+B) =
   8 input lines.
3. **Common-ground rule (mandatory, bench lesson 2026-07-17):** ESP32 GND, both driver GNDs, encoder
   supply GND and the 12 V motor-supply GND must **all** be tied together — otherwise PWM/DIR and
   encoder A/B levels are undefined. This is a wiring requirement, not a pin choice, but it belongs
   on the schematic.
4. **Pi link (micro-ROS transport):** the current firmware uses `board_microros_transport = serial`
   with `ARDUINO_USB_MODE=0` / `ARDUINO_USB_CDC_ON_BOOT=0` (`platformio.ini`) → `Serial` = **UART0
   via the on-board USB-UART bridge**. On the S3 that is UART0 = **GPIO43/44** (internal to the
   bridge; the Pi connects to the board's **"COM"/UART USB-C** port). **Recommendation: keep this
   transport** (least change, matches working firmware). Then **GPIO19/20 (native USB / USB-JTAG)
   are free** and are reserved for debug / as the alternative transport. If native USB CDC is chosen
   instead (alternative), GPIO19/20 are consumed and **GPIO43/44 become free**. Exactly one of the
   two pairs is consumed by the Pi link — see §3.
5. **Status display driven by `esp32-drive` over SPI** (HWR-21, OP-H6) — display changed from I2C to
   **SPI (Sharp Memory LCD `LS027B7DH01`, 2.7", sunlight-readable)**, 3 pins: **DISP_MOSI/SI GPIO38,
   DISP_SCLK GPIO39, DISP_CS GPIO40** (CS active-high on this display). Reallocated 2026-07-20 (was I2C
   SDA/SCL on GPIO38/39).
6. **IMU (BNO085) and GNSS live on the Pi, not the ESP32** (HWR-25/26) — this keeps the ESP32 budget
   relaxed; they appear only in the Pi table (§2).

---

## 1. ESP32-S3 (`esp32-drive`) — pin assignment

**Legend:** *fixed* = dictated by hardware, no free choice. `IOxx` = the board silkscreen label
(= the S3 GPIO number on the YD-ESP32-S3 board). Every **primary** pin below is unique — zero
overlap. Alternatives are drawn from the spare pool (§1.2); collisions are stated explicitly.

### 1.1 Function table

| Function | Primary GPIO | Board label | Alternative GPIO | Rationale / constraints |
|---|---|---|---|---|
| FL motor PWM | **GPIO4** | IO4 | GPIO1 | LEDC-capable; ADC1, low & well broken out; not strapping. Alt GPIO1 = free spare. |
| FL motor DIR | **GPIO5** | IO5 | GPIO42 | Plain GPIO out; grouped next to FL_PWM. Alt GPIO42 = free spare. |
| FR motor PWM | **GPIO6** | IO6 | GPIO2 | LEDC-capable; not strapping. Alt GPIO2 = free spare. |
| FR motor DIR | **GPIO7** | IO7 | GPIO47 | Plain GPIO out. Alt GPIO47 = free spare. |
| BL motor PWM | **GPIO15** | IO15 | GPIO40 | LEDC-capable; not strapping. Alt GPIO40 = free spare. |
| BL motor DIR | **GPIO16** | IO16 | GPIO48 | Plain GPIO out. **Alt GPIO48 = onboard RGB LED pin — usable only if the WS2812 LED is not used; verify.** |
| BR motor PWM | **GPIO17** | IO17 | GPIO41 | LEDC-capable; not strapping. Alt GPIO41 = free spare. |
| BR motor DIR | **GPIO18** | IO18 | GPIO19 | Plain GPIO out. **Alt GPIO19 = native-USB D-; free only if the Pi link uses the UART bridge (the recommended default) — then usable.** |
| FL encoder A | **GPIO8** | IO8 | GPIO47 | PCNT via GPIO matrix (any GPIO). Grouped per motor. Alt GPIO47 shared with FR_DIR alt — only one relocatable at a time. |
| FL encoder B | **GPIO9** | IO9 | GPIO20 | PCNT. **Alt GPIO20 = native-USB D+; free only with the UART-bridge transport.** |
| FR encoder A | **GPIO10** | IO10 | GPIO43 | PCNT. **Alt GPIO43 = UART0 TX; free only if the Pi link uses native USB (i.e. NOT the recommended default).** |
| FR encoder B | **GPIO11** | IO11 | GPIO44 | PCNT. **Alt GPIO44 = UART0 RX; free only if the Pi link uses native USB.** |
| BL encoder A | **GPIO12** | IO12 | GPIO1 | PCNT. Alt GPIO1 shared with FL_PWM alt — only one at a time. |
| BL encoder B | **GPIO13** | IO13 | GPIO2 | PCNT. Alt GPIO2 shared with FR_PWM alt — only one at a time. |
| BR encoder A | **GPIO14** | IO14 | GPIO40 | PCNT. Alt GPIO40 shared with BL_PWM alt — only one at a time. |
| BR encoder B | **GPIO21** | IO21 | GPIO41 | PCNT. Alt GPIO41 shared with BR_PWM alt — only one at a time. |
| Status display MOSI/SI (SPI) | **GPIO38** | IO38 | GPIO42 | SPI data to Sharp Memory LCD (HWR-21). Was I2C SDA. Alt GPIO42 shared with FL_DIR alt. |
| Status display SCLK (SPI) | **GPIO39** | IO39 | GPIO47 | SPI clock. Was I2C SCL. Alt GPIO47 shared — only one at a time. |
| Status display CS/SCS (SPI) | **GPIO40** | IO40 | GPIO41 | SPI chip-select, active-high on LS027B7DH01. NEW pin (SPI needs CS; I2C did not). Was a guaranteed-free spare. |
| **BATT_SENSE** (battery-voltage ADC) | **GPIO2** | IO2 | GPIO1 | ADC1_CH1. Reads the 5:1 sensor-module output (12 dB atten, eFuse cal). DECIDED+BUILT 2026-07-21. Was a guaranteed-free spare. |
| **Pi link (micro-ROS)** | **GPIO43/44 (UART0)** *fixed* | COM/UART USB-C | GPIO19/20 (native USB) | See §3. Recommended: UART bridge (43/44). Not user-wired — internal to the on-board bridge; the Pi plugs into the board's COM USB-C. |
| *(reserve)* relay/actuator ctrl | GPIO47 *(if free)* | IO47 | — | Optional (HWR-8 budget line). Harmless low-rate output; only if not consumed as an encoder/display alt. |

Pins consumed by primaries: **2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38, 39, 40**
(20 signal pins) + UART0 43/44 for the link. Still within budget (HWR-8: ≈19–23 expected).
*CORRECTED 2026-08-12:* the count previously read 18 pins and omitted **GPIO40** (DISP_CS, added with the
display's I2C→SPI change 2026-07-20) and **GPIO2** (BATT_SENSE, built 2026-07-21) — both are listed as
primaries in §1.1 and as consumed in §1.2, so only this summary line was stale.

### 1.2 Spare pool (for alternatives)

- **Guaranteed-free spares (no primary use):** GPIO1, GPIO41, GPIO42, GPIO47.
  (**GPIO40 consumed by DISP_CS** — display I2C→SPI 2026-07-20; **GPIO2 consumed by BATT_SENSE**
  (ADC1_CH1) — battery monitoring 2026-07-21. **GPIO1 (ADC1_CH0) is the last free ADC1 pin** and is kept
  as reserve — one fewer fault-relocation spare than before.)
- **Conditional spares:**
  - GPIO19, GPIO20 — free **only if** the Pi link uses the UART bridge (the recommended default).
  - GPIO43, GPIO44 — free **only if** the Pi link uses native USB (the alternative transport).
  - GPIO48 — free **only if** the onboard RGB LED (WS2812) is not driven.
- **Honest limitation (user is aware):** there are only 6 guaranteed-free spares for 18 functions, so
  several alternatives above **share** a spare pin. A shared spare can absorb **only one** failed
  primary at a time; a second simultaneous failure on a function pointing at the same spare must take
  a different free pin. Encoders are the most flexible (PCNT reaches any GPIO through the matrix), so
  in practice reassign a broken encoder line to whichever spare is still free.

### 1.3 Driver-channel ↔ motor ↔ ESP32-pin mapping (FRONT/REAR grouping)

Consequence of the 2026-07-29 front/rear driver decision (§0.1 item 1). The ESP32 PWM/DIR pins are
**per-wheel and unchanged** (§1.1); only *which driver channel* each wheel's PWM/DIR pair and motor
leads land on has changed. This is the textual twin of the routed QET wiring diagram (`A3`/`A4`
drivers, `X-DRV` terminals).

| Driver | Channel | ESP32 PWM / DIR | Motor |
|---|---|---|---|
| **A3 — MDD10A #1 FRONT** | ch 1 | GPIO4 / GPIO5 | M1 — FL |
| | ch 2 | GPIO6 / GPIO7 | M3 — FR |
| **A4 — MDD10A #2 REAR** | ch 1 | GPIO17 / GPIO18 | M4 — BR |
| | ch 2 | GPIO15 / GPIO16 | M2 — BL |

**AS-BUILT 2026-08-12 (user, rear driver):** on **A4 (REAR)** the channel order is **swapped against the
original plan** — **BR on ch 1, BL on ch 2** (was BL ch1 / BR ch2). The table above reflects the build.
Electrically irrelevant: the MDD10A's two channels are identical, and each wheel keeps its **own** ESP32
PWM/DIR pair, which simply lands on the other channel's input header. **No pin-map change, no firmware
change.** **A3 (FRONT) confirmed as-built by the user 2026-08-12: as planned, FL ch 1 / FR ch 2.** The
whole table above is therefore verified against the harness.

**Delta vs. the former LEFT/RIGHT grouping:** **FR and BL swap drivers** (FR #2→#1, BL #1→#2); FL
stays on #1, BR stays on #2. Each motor's ESP32 PWM/DIR pair **and** its M+/M− leads follow it to the
new driver channel. Encoders are **not** routed through the drivers (straight to the ESP32, GND on the
**ESP32 (`A1`) ground node** per §12 deviation 4 — *not* on a separate `X-ENC` strip) and
are unaffected. Firmware `main.cpp` is unchanged.

---

## 2. Raspberry Pi 5 — pin assignment

Header GPIOs are on the RP1; the peripheral functions below are the standard Pi 5 assignments.

| Function | Primary | Header pin | Alternative | Fixed? | Rationale / constraints |
|---|---|---|---|---|---|
| LiDAR UART TX (LD06) | **GPIO14 / TXD (ttyAMA0)** | pin 8 | uart3 GPIO4 (TXD) via dtoverlay | **fixed** (primary header UART) | LD06 already on `ttyAMA0` (ASBUILT §1.5). PL011. Alt needs a `dtoverlay=uart3` + device re-map. |
| LiDAR UART RX (LD06) | **GPIO15 / RXD (ttyAMA0)** | pin 10 | uart3 GPIO5 (RXD) via dtoverlay | **fixed** | Pair of the above. |
| IMU I2C SDA (BNO085) | **GPIO2 / SDA1 (i2c1)** | pin 3 | i2c-gpio bit-bang or i2c3 (GPIO4/5) via dtoverlay | **fixed** (primary I2C, has onboard 1.8 kΩ pull-ups) | HWR-25, STEMMA-QT. BNO085 default addr 0x4A/0x4B. Onboard pull-ups suit the IMU. |
| IMU I2C SCL (BNO085) | **GPIO3 / SCL1 (i2c1)** | pin 5 | i2c3 (GPIO4/5) via dtoverlay | **fixed** | Pair of the above. |
| **LIDAR_EN** power switch (HWR-23) | **GPIO23** | pin 16 | **GPIO24** (pin 18) | no | Drives a high-side MOSFET/relay for the switchable LiDAR branch. Both are plain, unused header GPIOs clear of UART/I2C/SPI0 defaults. Idle state must be defined (see Open Q3). |
| GNSS reserve UART (HWR-26) | **GPIO4/5 (uart3)** | pins 7 / 29 | **USB-serial GNSS via the powered hub** | no | Reserve only (use = stage 2). Alt route (USB) avoids header UART contention and keeps GNSS platform-agnostic (HWR-18). Keep both options open in the layout. |
| **Clean-shutdown / restart button** signal (HWR-40) | **GPIO17** | **pin 11** | none declared (see §2.1 for the exclusion set) | no | **Bench-verified 2026-08-19** — plain GPIO, no strapping/boot function, clear of `i2c1` / `ttyAMA0` / SPI0. Idle **HIGH**, pressed **LOW**. Module **GND = pin 9**, **VCC = pin 1 = 3.3 V — never 5 V**. Keeps GPIO24 (pin 18) free as the LIDAR_EN alternative. Detail + evidence: §2.1. |
| USB hub uplink (HWR-5/22) | **1× Pi USB 3.0 port** | USB-A | the other Pi USB 3.0 port | no | Powered hub, **data uplink only**; hub supply from the regulated 5 V rail, not Pi USB. Downstream: drive ESP32, steering-servo adapter, arm board, cam-gripper, cam-front. |

**USB topology (HWR-5, HWR-22):** externally-powered USB hub → uplink to one Pi USB 3.0 port.
Downstream devices: `esp32-drive` (COM port), steering-servo adapter (ttyACM), arm XIAO board
(ttyACM), gripper camera (UVC/MJPEG), front camera (UVC/MJPEG). Cameras must be MJPEG/H.264
(no uncompressed YUYV); distribute across both USB buses. udev symlinks (`/dev/esp32`,
`/dev/steering_servo`, `/dev/arm_servo`) stay by serial/port path, unchanged by the hub.


### 2.1 Clean-shutdown / restart button (HWR-40) — Pi pin assignment, BENCH-VERIFIED 2026-08-19

Everything in this subsection was **established on the machine on 2026-08-19** (multimeter on the module,
read-only GPIO reads on the Pi). Nothing here is inferred from a datasheet or estimated. Requirement-level
text stays in the internal requirements document, §10.3 **HWR-40** (tracked internally, not in this
repository); this is the wiring/pin authority for it.

#### Connections (3-pin button module → Pi 40-pin header)

| Module lead | Pi header pin | Net | Note |
|---|---|---|---|
| **Signal / OUT** | **pin 11** | **GPIO17** (BCM 17) | Idle **HIGH**, pressed **LOW**. |
| **GND** | **pin 9** | GND | Adjacent to pin 11 — signal and its return sit in the same header corner, so the pair can be run as one short twisted lead. |
| **VCC** | **pin 1** | **+3.3 V** | **3.3 V ONLY.** See the 5 V prohibition below. |

#### Why GPIO17 (the exclusion set that produced it)

Every other candidate on the header is either taken or reserved by this plan (§2 table) or by the Pi's own
config:

- **GPIO2 / GPIO3** (pins 3 / 5) — IMU `i2c1`, marked **fixed**.
- **GPIO14 / GPIO15** (pins 8 / 10) — LD06 UART `ttyAMA0`, marked **fixed**.
- **GPIO23** (pin 16) — `LIDAR_EN` (HWR-23), with **GPIO24** (pin 18) as its **declared alternative**;
  taking GPIO24 for the button would consume that fallback, so it is deliberately left free.
- **GPIO4 / GPIO5** (pins 7 / 29) — GNSS reserve UART (HWR-26).
- **GPIO7–GPIO11** — SPI0. `/boot/firmware/config.txt` on the Pi carries **`dtparam=spi=on`**, so SPI0 is
  enabled and these pins are in use, not spare.

**GPIO17** is a plain GPIO with **no strapping or boot function**, clear of I2C, UART and SPI, and it leaves
the LIDAR_EN alternative intact.

*Name-collision warning (not a conflict):* **`GPIO17` also appears in §1.1 as `IO17` = BR motor PWM on the
`esp32-drive` ESP32-S3.** Those are **two different chips**. When quoting a pin, always say whether it is a
**Pi** GPIO or an **ESP32-S3** GPIO — this plan holds both maps.

#### 5 V is PROHIBITED on this module (record, do not re-litigate)

Pi GPIOs are **not 5 V tolerant**. A 3-pin button module typically drives its signal pin **from its own VCC**,
so wiring VCC to 5 V (pin 2/4) would put **5 V onto GPIO17** and can destroy the RP1 input. Therefore:
**VCC = pin 1 (3.3 V)**. If a module is ever chosen that **requires** 5 V, it MUST NOT be connected directly —
it needs a **level shifter** on the signal line. Carry this into HWA-10 when the button component is selected.

#### Polarity and pull-ups

- **Idle HIGH, pressed LOW** — measured by the user with a multimeter and independently confirmed by reading
  the line on the Pi.
- The **module carries its own pull-up**. The Pi-side pull-up bias (`LINE_REQ_FLAG_BIAS_PULL_UP`) pulls in the
  **same direction**, so enabling it is **not** a conflict; it only makes the idle level defined if the module
  is ever unplugged. There is no pull-up fight and no need for an external resistor.

#### Kernel / GPIO facts (verified read-only on the Pi)

- Header chip = **`/dev/gpiochip4`**, label **`pinctrl-rp1`**, **54 lines**, group **`dialout`** — the `ubuntu`
  user can read/drive it **without root**. This is **not** `gpiochip0` (a different bank).
- **Line offset = BCM number**, so the button is **line 17** on that chip. This is exactly the pattern
  `Software/ros2/src/gripperx_control/config/lidar_power.yaml` already uses for GPIO23
  (`gpio_chip_path: /dev/gpiochip4`, `gpio_line_offset: 23`) — the button trigger should follow the same
  chip-path + offset + consumer-name convention rather than inventing a second one.
- At the time of the check, **line 17 read `used=False, consumer=None`** (free, nobody claims it) while line 23
  read `consumer='gripperx_lidar_power'` — confirming both that the pin is available and that the reading
  method resolves consumers correctly.
- Installed tooling on the Pi: **libgpiod 1.6.3** with the **v1 Python bindings** (`python3-libgpiod`);
  **no `gpiod` CLI tools** are installed (`gpioinfo`/`gpioget` are not available — use the Python bindings).
  `LINE_REQ_FLAG_BIAS_PULL_UP` **is** supported by this version.

#### Debounce evidence — result and its honest limit

Bench read, 2026-08-19: **20 s of sampling at 1 kHz** captured a clean **1→0 / 0→1 pair** with a **230 ms
press**, and **3 edges across 2 presses** — i.e. one transition of the second press fell outside the sampling
window. **No observable contact bounce**: bounce would appear as several same-direction edges within a few
milliseconds, and none were recorded. A follow-up read confirmed a **stable idle HIGH**.

**Limits of that evidence, stated so nobody over-reads it:** 1 kHz sampling **cannot see sub-millisecond
bounce**, and only **two presses** landed in the window. The result is "no bounce at the millisecond scale
over two presses", **not** "this button does not bounce". Debounce handling in the trigger is therefore still
warranted; it is just not carrying a measured problem.

#### Trigger mechanism: userspace daemon — `dtoverlay=gpio-shutdown` is SUPERSEDED

**Do not add a `dtoverlay=gpio-shutdown` line to `/boot/firmware/config.txt`.** It was considered on
2026-08-19 and **superseded the same session** by a user requirement: **two functions on this one button.**

**Press timing — DECIDED 2026-08-19 (user). These are the values the daemon implements; §2.1 is their
single source of truth** (HWR-40 states the *requirement*, this table states the *numbers* — do not restate
them elsewhere):

| Press duration (press → release) | Action |
|---|---|
| **< 1 s** | **Clean restart of the ROS2 processes.** The **Pi stays up.** |
| **1 s … 3 s — DEAD ZONE** | **NOTHING HAPPENS.** The press is discarded. |
| **≥ 3 s** | **Clean shutdown and halt** of the OS. |

**The dead zone is deliberate, not a gap in the spec.** Without it, one badly-timed press sits on the
boundary between "restart the stack" and "halt the machine", and the operator cannot know which they will
get. Discarding the ambiguous band means **a surprise halt is impossible**; the cost is that a press
released between 1 s and 3 s does nothing at all, which the operator must repeat. That trade was chosen
deliberately. **A daemon that maps the dead zone onto either action is a defect.**

The overlay emits a **single `KEY_POWER` event** and **cannot measure press duration**, so it can
distinguish **none** of the three bands above — it cannot tell a restart press from a halt press, and it
cannot discard the dead zone. The trigger is therefore a **small non-ROS userspace daemon reading GPIO17** on
`/dev/gpiochip4` line 17 — consistent with HWR-40's decided *"Pi GPIO evaluated by a small independent
systemd unit"* signalling path. **The daemon is not designed here** (another agent implements it); this
subsection only fixes the pin assignment, and that assignment matches a **daemon-based** trigger, not the
overlay.

**Cost of dropping the overlay — recorded honestly, not glossed over.** The kernel-level overlay would have
worked **even with userspace wedged**, which is precisely the robustness HWR-40 criterion 1 argues for. A
userspace daemon does not inherit that property. What is on the other side of the trade:
- the daemon is a **tiny process with no ROS and no DDS**, so it does not share the dependency chain that
  actually wedges (micro-ROS / DDS / `controller_manager`);
- it runs with **`Restart=always`**;
- the **Pi's own power button remains the fallback** if the daemon itself is dead.

This is a **weaker** guarantee than the overlay's, accepted in exchange for the two-function control the user
requires. It is a real regression in the wedged case and should be treated as such in any later review.

#### DECLINED 2026-08-19 (user): no dedicated feedback LED at the button

A dedicated indicator LED next to the button — to show that a press was registered and which action is
running — was proposed and **declined by the user**: the operating instructions are considered clear enough.
**Do not re-propose it.** No LED is to be wired, no header pin or 5 V budget is reserved for one.

*Consequence, recorded so the decision is understood rather than just obeyed:* at the machine there is now
**no local feedback on the button itself**. In particular a press discarded in the **1 s … 3 s dead zone**
is **indistinguishable from a dead button** — nothing happens in both cases, and the operator learns the
difference only by the robot's later behaviour. The **only remaining path** to HWR-40's criterion 2
(Pi-independent indication of what is happening and when it is safe to remove power) is therefore
**HWR-21**, the `esp32-drive`-driven status display — which **is not built**.

#### Open: can a GPIO **wake** the Pi 5 from halt? — UNVERIFIED

On the **Pi 4**, GPIO3 could wake the board from halt. The **Pi 5** has its **own power button** and the **J2
header**, and whether a header GPIO can wake it **has not been verified**. **Do not assume it can.** Until
verified, re-powering after a halt is **`Q1`** or the **Pi's own button** — which is also what HWR-40 assumes
(manual power removal, no latch). No pin is reserved for a wake function on the strength of an unverified
capability.


---

## 3. Pi link transport — the two mutually exclusive options

| Option | ESP32 pins used | Frees | Firmware setting |
|---|---|---|---|
| **A — UART bridge (recommended default)** | UART0 GPIO43/44 (internal to bridge) | GPIO19/20 (native USB) → debug/spares | current `platformio.ini` (`ARDUINO_USB_MODE=0`, `CDC_ON_BOOT=0`, `board_microros_transport=serial`) |
| **B — Native USB CDC (alternative)** | GPIO19/20 | GPIO43/44 (UART0) → spares | `ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1` |

**Recommendation: Option A.** It matches the firmware that already works and keeps the very useful
USB-JTAG on GPIO19/20 available for debugging. Option B is more robust against UART-bridge quirks and
gives one extra clean USB endpoint, but changes the boot/enumeration behaviour and consumes 19/20.
The choice is bench-decidable; the alternative-pin columns in §1.1 flag which spares each option
unlocks. **Do not route motor PWM/DIR onto either 19/20 or 43/44** so the transport stays switchable.

---

## 4. Constraints & exclusions (ESP32-S3-N16R8)

**Never used for motor PWM/DIR or any timing-critical signal:**
- **Strapping pins GPIO0, GPIO3, GPIO45, GPIO46** — this is the structural fix for the old ~500 ms
  uncontrolled-motor-run after reset (SR-7): under the legacy WROOM wiring the offenders were the
  *WROOM* strapping pins GPIO5/GPIO16 carrying FR/BR PWM. On the S3 the strapping set is different
  (0/3/45/46); **no motor line touches it in this plan.** GPIO45/46 additionally have fixed boot
  pulls; GPIO0 is the BOOT button. If ever used, only for a harmless static function, stated
  explicitly (none are used here).

**Excluded entirely (not available on the N16R8):**
- **GPIO26–32** — SPI0/1 flash.
- **GPIO33–37** — **octal PSRAM** on the R8 module (task note flagged 35–37; GPIO33/34 are also
  octal-PSRAM lines on octal parts, so this plan excludes the full 33–37 range to be safe). Verify on
  the delivered board (see Open Q1).

**Reserved (do not use for motor/encoder signals, to keep the transport switchable):**
- **GPIO19/GPIO20** — native USB D-/D+ / USB-JTAG (Pi-link Option B; debug otherwise).
- **GPIO43/GPIO44** — UART0 (Pi-link Option A, the recommended default).

**Special:**
- **GPIO48** — onboard RGB LED (WS2812) on the YD board; only usable as a spare if the LED is unused.
- ESP32-S3 has **no input-only GPIOs** (unlike the classic ESP32) — encoders are on normal
  bidirectional pins. Encoder inputs are **3.3 V only** (not 5 V-tolerant).

---

## 5. Changes vs. legacy wiring (what gets rewired at reassembly)

| Signal | Legacy (WROOM-32, slalom test) | Target (this plan, S3) | Note |
|---|---|---|---|
| FL PWM / DIR | GPIO19 / GPIO13 | **GPIO4 / GPIO5** | new board, new pins |
| BL PWM / DIR | GPIO18 / GPIO21 | **GPIO15 / GPIO16** | new pins |
| BR PWM / DIR | **GPIO16 (strapping)** / GPIO26 | **GPIO17 / GPIO18** | **strapping pin eliminated** |
| FR PWM / DIR | **GPIO5 (strapping)** / GPIO17 | **GPIO6 / GPIO7** | **strapping pin eliminated; supersedes the "→GPIO22" note (GPIO22 does not exist on S3)** |
| Encoders A/B ×4 | not wired (open-loop) | **GPIO8/9, 10/11, 12/13, 14/21** | new — 8 lines, 4× HW PCNT, 3.3 V |
| Status display SPI | none | **GPIO38 (MOSI/SI) / GPIO39 (SCLK) / GPIO40 (CS)** | new (HWR-21); Sharp Memory LCD LS027B7DH01, SPI (was planned I2C, changed 2026-07-20) |
| Pi link | UART0 GPIO1/3 via CP2102 (WROOM) | **UART0 GPIO43/44 via on-board bridge (S3)** | no user wiring — internal; Pi plugs into COM USB-C |

**Net effect:** every motor line moves off strapping pins → SR-7 / HWR-8 resolved structurally;
encoder odometry becomes wireable (HWR-10); the status display gets its bus (HWR-21).

---

## 6. Recommended requirements change (not applied in this run)

**HWR-8** currently reads *"Strapping-pin rewire (FR-PWM→GPIO22, BR-PWM→GPIO4)…"*. Proposed update:
- Drop the literal `GPIO22 / GPIO4` targets — **GPIO22 does not exist on the ESP32-S3** and GPIO4 is
  re-used as FL_PWM in the new map. Replace with: *"rewire all motor PWM/DIR off the S3 strapping
  set per `documentation/schematics/WIRING_PLAN.md` §1/§5 (FL 4/5, FR 6/7, BL 15/16, BR 17/18)."*
- Note that the firmware `#define FR_REWIRE_GPIO22 / BR_REWIRE_GPIO4` blocks in `main.cpp` are legacy
  WROOM logic and must be replaced by the S3 pin map when the firmware is ported to the S3 board.
- Cross-reference this plan as the pin source of truth for the HWR-15 schematic.

---

## 7. Open questions

1. **Board silkscreen / PSRAM range (physical verify before soldering):** confirm on the delivered
   `YD-ESP32-S3-N16R8` that GPIO33–37 are truly unavailable (octal PSRAM) and that every primary pin
   in §1.1 is broken out with a matching `IOxx` silkscreen label. *Recommendation:* a quick blink/scan
   of each planned pin before final wiring. **Options:** (a) accept the plan as-is and verify at
   bench [recommended]; (b) request a photo of both pin headers now so the map can be pre-checked.
   **DECIDED 2026-07-19: (a) — verify at bench.**
2. **Pi-link transport choice — Option A (UART bridge) vs. B (native USB)?** *Recommendation:*
   **Option A** (keep the working serial-over-bridge transport; frees GPIO19/20 for JTAG/debug).
   Choose B only if the UART bridge proves unreliable at bench. **Options:** (a) A [recommended];
   (b) B; (c) decide at bench — plan already reserves both pairs.
   **DECIDED 2026-07-19: (a) — Option A, UART bridge; GPIO19/20 stay free for debug.**
3. **LIDAR_EN idle/default level (HWR-23):** should the LiDAR branch power **on** or **off** by
   default at Pi boot? *Recommendation:* **default ON** (LiDAR available after boot) via a pull that
   holds the high-side switch enabled, with software able to cut it — matches NFR-9 (drive without
   LiDAR) as an on-demand action, not a boot state. **Options:** (a) default ON, SW can disable
   [recommended]; (b) default OFF, SW must enable (safer inrush, but LiDAR silent until commanded).
   **DECIDED 2026-07-19: (a) — default ON, software can disable.**
4. **RGB LED (GPIO48) as a spare?** May the onboard WS2812 be sacrificed to free GPIO48 as a spare?
   *Recommendation:* **keep the LED** (useful as an at-a-glance ESP heartbeat, complements HWR-21) and
   treat GPIO48 as a *conditional* spare only. **Options:** (a) keep LED [recommended]; (b) free
   GPIO48 for wiring.
   **DECIDED 2026-07-19: (a) — LED kept; GPIO48 remains a conditional spare.**
5. **Front/rear driver-to-wheel grouping (OP-H9/HWA-2) — DECIDED (user 2026-07-29, housing redesign):**
   **FRONT driver = FL + FR**, **REAR driver = BL + BR** (was tendency LEFT = FL+BL / RIGHT = FR+BR).
   The regrouping fits the redesigned housing better. This does **not** change the ESP32 pin map (§1;
   PWM/DIR are per-wheel regardless) nor the firmware direction signs — only which driver channel each
   wheel lands on (harness/perfboard). *Remaining:* ~~physical install +~~ user acceptance of the
   replacement MDD10A (HWA-11). ⚠️ **CORRECTED 2026-08-24:** the **physical install is done** — §1.3
   records the grouping as **AS-BUILT 2026-08-12, verified against the harness by the user** (with `A4`'s
   channel order swapped against the plan). Only the **HWA-11 user acceptance** of the replacement
   MDD10A unit is still open. The same stale "install still open" wording sits in §12.3 row 2 and in
   internal REQUIREMENTS OP-H9/HWA-11.

---

## 8. Conductor cross-sections — decisions (user, 2026-07-20)

Feeds **HWA-1** (conductor cross-sections per circuit) and **HWR-16** (current-matched cross-sections /
color code). Based on the `gripperx-schematics` cross-section sizing analysis (2026-07-20) against the
KiCad power sheet (fuses F1–F4) and the MHPOWOS 12 V 20 Ah battery (20 A cont / 28 A pulse). The heavy
power side keeps the sizing from that analysis; the decisions below settle the two branches the user
chose to close now (servos, drive-motor tails) and record the follow-on items.

### 8.1 Servo wiring (steering + arm) — CLOSED / accepted as-is

- **Steering bus (4× ST3215, 12 V) and arm (6× STS3215, 12 V): DONE.** The **factory-supplied cables
  (~jumper gauge) stay**; no further cross-section work needed. The servo strands from the sizing table
  are therefore **"as-is accepted", no longer open.** (Currents are low: ~2.7 A/servo stall assumed,
  well within the factory lead; the actuator branch stays on **F2 ~20–25 A**.)

### 8.2 Drive motors (4× Pololu GB37-50) → MDD10A — DECISION = Option B

- **Factory motor lead:** ~20 cm, **presumed 24 AWG (not verified).** Planned total motor↔driver run
  ~50 cm.
- **Decision (Option B):** **keep the 20 cm factory stub + add the remaining ~30 cm as a proper
  extension in `0.5 mm² / AWG 20`.** Result: total drop **~0.27 V (~2.3 %) @ 5 A stall**, and the AWG
  uncertainty of the stub becomes irrelevant (the thick extension dominates the run). **No thicker cable
  directly at the motors is required.**
- This *supersedes* the sizing-table suggestion of 1.5 mm² per motor tail (that value was mechanical/
  worst-case; Option B is the accepted build). Per-motor drive strand = **0.5 mm² / AWG 20 extension +
  factory stub.**

### 8.3 Heavy cross-sections — UNCHANGED (focus stays here)

The thick-conductor focus stays on the **power distribution** and the **Pi 5 logic rail**, unchanged
from the sizing analysis:

| Strand | Cross-section |
|---|---|
| Battery → main distribution (`+12V`, main feed, **F1 25 A slow** as drawn 2026-08-12; §9.3) | **6 mm² / ≈AWG 9–10** |
| Main → actuator/drive node and `+12V_DRV` driver trunk (F2 ~20–25 A) | **4 mm² / ≈AWG 11** |
| `+5V_LOGIC` → Pi 5 (+ ESP32) — **voltage-drop-critical**, keep short | **1.5 mm² / ≈AWG 16** |

Star-ground returns per HWR-4 sized ≥ their live conductor (main GND = 6 mm²); common-ground
ESP↔drivers mandatory (HWA-2).

### 8.4 Follow-on / open points (from these decisions)

1. **Firmware stall / over-current cut-off for the drive motors (open point, normal priority).** The
   thin `0.5 mm²/AWG 20` motor lead is fine for transient stall (~5 A) but **cable sizing does NOT cover
   a sustained stall** (continuous-blockage heating). Add a firmware stall/over-current trip (encoder-
   blockage detection, HWR-10, is the natural hook) to shed the motor on a held stall. *Kabelauslegung
   selbst deckt Dauer-Stall nicht ab.*
2. **Per-drive-motor short-circuit pre-fuse (nice-to-have, LOW priority, optional).** `F2 ~20–25 A` is
   too slow to protect the thin motor lead in a dead-short case, and the MDD10A's own protection is
   coarse/slow. A small per-motor fuse would close that gap. **Explicitly optional / low-prio — the user
   deliberately accepts the factory state as the baseline.**

### 8.5 Open measurement (non-blocking note)

- **Actual GB37-50 stall-current measurement** and **AWG verification of the factory stub** are still
  outstanding. The **5 A stall / 24 AWG** figures are documented/presumed, **not measured** — confirm at
  bench (does not block the Option B build above).

---

## 9. Operator / switching architecture (RESTRUCTURED by the user 2026-08-12)

**Status: as-drawn. Traced and verified element-by-element from the QET wiring/terminal diagram
(internal source file, not in this repository) on 2026-08-12** (electrical trace *through* terminal
blocks, not conductor-only). Supersedes the 2026-07-21 coil-side-only architecture.

> ⚠️ **COMPLETENESS CAVEAT — added 2026-08-24. The trace below is accurate; the drawing it traces is not
> complete.** Re-verified against the `.qet` on 2026-08-24: the file holds exactly **three
> `switch.elmt` instances** (`Q1`, `Q3`, `Q4`) and **one `estop.elmt`** (`S1`), and every conductor path
> asserted in this section checks out end-to-end. **But the user confirmed on 2026-08-24 that the machine
> carries FOUR operator switches — `Haupt` (master) · `Logik` · `Aktor` · `Drive` — plus the E-stop.**
> The **`Logik` switch is not drawn at all**: it is an *undrawn element*, not a gap in this prose. There
> is no `Q2` and no `Q5` in the file (the string `Q5` appears once, inside a free-text annotation reading
> *"MDD10A driver-to-wheel (OP-H9/sec.7 Q5, DECIDED 2026-07-29)"* — that is a reference to **§7 open
> question 5** of this document, not a component designator; `Q6` is the BSS138 `LIDAR_EN` driver MOSFET,
> not an operator switch).
> **Consequence: until the `Logik` switch is drawn, §9 must not be read as the complete operator
> topology.** Where the `Logik` switch sits electrically is unknown from the desk and is **not guessed
> here** — see the internal hardware audit of 2026-08-24 (Q1) and the robot-day list.

```
G1 ──F1 (25 A MIDI/ANS slow) ──┬── X1:15 ── B6 batt-sense              ] PERMANENTLY LIVE
                               ├── X1:13 ── F2 (~20 A) ── K1.COM(30)   ] bypasses Q1
                               │
                               └── Q1 MASTER ──┬── A5 USB hub (12 V)
                                               ├── X1:14 ── T1 DC/DC ──┬── F3 → +5V_LOGIC (Pi, ESP, C1)
                                               │                       └── F4 → +5V_SENS (LCD, K3→LiDAR)
                                               └── X1:11 ── S1 NC (E-stop) ── COIL_SUP (X1:5)
                                                                          ├── X1:17 ── Q3 → K1 coil
                                                                          └──────────── Q4 → K2 coil

K1.87(NO) ──┬── F? Servo (~7.5 A?) ── X1:6 "+12V_ACT" ── A7 steering (M11-M14) + A8 arm (M5-M10)
            └── F? DRIVE  (~20 A)  ── K2.COM(30)
K2.87(NO) ── X1:7 +12V_DRV ──┬── F? F-DRIVE (~10 A) ── A3 MDD10A #1 FRONT (FL+FR)
                             └── F? B-DRIVE (~10 A) ── A4 MDD10A #2 REAR  (BL+BR)
```

**Exactly three things stay live with `Q1` open:** `B6` (batt-sense), `F2`, and the `K1.COM` contact.
Everything else is dead, because `COIL_SUP` is fed from the **switched** side — opening `Q1` drops both
relay coils, so `K1` and `K2` release and `+12V_ACT` / `+12V_DRV` go dead.

### 9.1 Why this is sound (do NOT "correct" it back)

1. **`Q1` switches the whole robot off while carrying only the logic current.** It feeds the DC/DC and the
   USB hub, not the actuator branch: the WG-1224S is 50 W ≈ **4.2 A at 12 V**, plus the hub — instead of
   the ~20–25 A actuator/drive branch. This is **pilot / control-circuit switching**: contact wear,
   arcing and switch voltage drop all stay off the heavy path, which is why a cheap rocker remains
   adequate. The heavy current still flows only through the JD1912 **contacts**.
2. **The fail-safe direction is correct.** Both relays are **NO** types and their coils sit on the
   **switched** side. A failed-open switch, a broken coil wire, a released contact or a blown `F1` all
   default to **actuators OFF**. Nothing has to actively hold the actuators off — de-energised *is* safe.
3. **The E-stop keeps its HWR-7 scope.** `S1` (NC) sits in the `COIL_SUP` feed, so pressing it drops
   `K1` + `K2` — all motion off — while **Pi, ESP32 and sensors stay powered** from the logic rail
   (`+5V_LOGIC` / `+5V_SENS` are upstream of `COIL_SUP`, fed directly from `Q1` via `T1`). That is the
   entire point of the revised scope: **status display and diagnostics survive the stop.**
4. **`B6` ahead of the switch reads the true battery terminal voltage**, including load-induced sag,
   rather than a value already dropped across `Q1`. (See open decision 3 — this benefit is smaller than
   it looks, and it costs standby current.)

### 9.2 `K2` cascaded behind `K1` — intentional

**`K2.COM` is fed from the `K1.NO` node (through `F? DRIVE ~20 A`), *not* directly from `F2`.** The drive
path is therefore a **subset** of the actuator path: **drive power exists only when `K1` is also closed.**
**Consequence: the actuator switch (and `K1`) is an upstream dependency of any drive movement** — motor
switch ON alone does nothing while the actuator switch is OFF. Confirmed by the user 2026-08-10, re-verified
in the drawing 2026-08-12. Do not read it as an error and do not "correct" it.

### 9.3 Branch fusing (as-drawn)

| Fuse | Rating as drawn | Protects | Status |
|---|---|---|---|
| `F1` | **25 A** MIDI/ANS slow | whole battery feed | **CHANGED 2026-08-12** (was 30 A) — see open decision 5 |
| `F2` | **~20 A** | actuator branch to `K1.COM` | **CHANGED 2026-08-12** (was `~20-25 A [TBD HWA-1]`; the TBD flag was dropped) |
| `F3` | ~7.5–10 A `[TBD HWA-1]` | **5 V OUTPUT side** (`T1` → Pi), alongside `F4` | **PLACEMENT CORRECTED 2026-08-13** — this table previously put `F3` on the 12 V input, which is the **fallback** two-converter design, not the built one (REQUIREMENTS HWR-2 caution note). Rating still `[TBD HWA-1]`. |
| `F4` | ~3–5 A `[TBD HWA-1]` | `+5V_SENS` (LCD, `K3`→LiDAR) | unchanged |
| `F? Servo` | **~7.5 A?** | steering + arm servo chains via `X1:6` | **NEW 2026-08-12** — designator and value both open |
| `F? DRIVE` | **~20 A** | feed from `K1.NO` to `K2.COM` | **NEW 2026-08-12** — designator open |
| `F? F-DRIVE` | **~10 A** | `A3` MDD10A #1 FRONT | **NEW 2026-08-12** — designator open |
| `F? B-DRIVE` | **~10 A** | `A4` MDD10A #2 REAR | **NEW 2026-08-12** — designator open |

**Naming note:** `X1:6` is still labelled `+12V_ACT`, but it now sits **downstream of `F? Servo`**, so as
drawn that label denotes the **fused servo branch only**, not the whole actuator rail. The `K1.NO` node
itself is unnamed. Likewise `X1:1 "+12V"` now sits on the **switched** side, while the permanently-live
pre-`Q1` bar has no named terminal at all.

### 9.4 Unchanged from the previous revision

- **`K1` = the existing JD1912** (actuator path → steering + arm); **`K2` = a second 40 A JD1912**
  (drive-motors-only path, rail `+12V_DRV`).
- **Full galvanic isolation** for storage/transport = **unplug the Anderson SB50**. The operator switches
  do **not** isolate the battery — and with this architecture `B6`, `F2` and `K1.COM` remain live even
  with `Q1` open, which makes the SB50 the only true isolation point.

### 9.5 OPEN DECISIONS — recorded as open, not settled

1. **~~Reverse-polarity protection absent~~ — CLOSED 2026-08-13 (user): `D1` is NOT required.** *Rationale (user):* the battery connector is **mechanically keyed** and can only be inserted one way, so a polarity reversal cannot occur in service; a protection element against an event the mechanics already prevent adds cost, a part and a failure point without reducing risk. **The recommendation below to re-add `D1` is therefore SUPERSEDED** — the drawing's state matches the decision and is no longer a defect. Residual boundary cases (miswired charger, different-polarity replacement pack, bench supply) are tracked under **HWA-10 part C**, which is closed. Historical record of the finding follows.
   ~~`D1` (VS-40CPQ060 crowbar
   Schottky) has been **deleted from the drawing entirely** — the element is gone *and* the
   `schottky.elmt` definition has been removed from the project collection (0 instances remain). So the
   robot as drawn has **no reverse-polarity protection in any switch position**, and a reversed battery
   is limited only by `F1`. `F2` and the `K1` contact would tolerate a reversal; **`B6` may not**, and it
   is on the permanently-live bar. **Recommendation: re-add `D1` on the pre-`Q1` bar directly after
   `F1`** — there it covers **every** branch in **every** switch position, the crowbar action through
   `F1` is unchanged, and it costs one wire. That is where it sat before the restructure.~~
   *(Superseded — see the decision above. `BESTELLLISTE.md` Charge 2 Nr. 13 no longer needs ordering.)*
2. **`Q1` now cuts power to a running Linux system.** It used to feed only the coils; it now feeds the Pi
   as well, so flipping it is a **hard power-off on a Pi 5 that may be writing ROS2 logs** — filesystem /
   SD-card corruption is the realistic failure mode, not a hypothetical one. Options, ascending in effort:
   (a) journald to RAM plus a read-only rootfs; (b) a documented "shut down in software, *then* `Q1`"
   procedure; (c) a soft-shutdown latch that removes power only after the OS is down. **Needs a
   deliberate decision.** **STATUS 2026-08-13: raised as `HWR-40` (must, MVP, draft) in the team's
   internal requirements document** (tracked internally, not in this repository), which decided
   option (b) — power removal stays **manual**, so no latch exists yet. See §9.6 and HWR-40 itself;
   this entry stays open until HWR-40 is implemented.
3. **`B6` quiescent current is unknown — `TO-VERIFY (nameplate or measurement)`. Do not guess it.** The
   trade-off: a purely resistive divider (~0.2 mA) is ≈**0.7 %/month** on the 20 Ah pack and entirely
   acceptable; an **active** module drawing several mA could reach **5–10 %/month**, and deep-discharging
   LiFePO4 is to be avoided. Note that the benefit of sensing *ahead* of the switch is small: **the ESP32
   that reads `B6` sits behind `Q1`**, so with `Q1` open nobody is listening anyway. Sensing pre-switch
   only avoids `Q1`'s contact drop (~0.1 V at logic current, **<1 % of 12 V**, and firmware-correctable),
   and the battery's load-induced sag is visible either way. **If the standby draw turns out non-trivial,
   moving `B6` behind `Q1` is the better trade** — 0.1 V of accuracy against the entire standby drain.
4. **Logic and coil supply now energise simultaneously — scenario S6.** Closing `Q1` boots the ESP32 and
   raises `COIL_SUP` in the *same instant*. If `Q3`/`Q4` were left closed, `K1`/`K2` pull in **during the
   ESP32 boot**, while its GPIOs are still high-Z. It **is** covered: `R30`–`R37` hold all eight PWM/DIR
   inputs low passively, and PWM low means the MDD10A output stage is off. But the **entire margin now
   rests on those eight resistors, with no procedural backstop.** Therefore: **the pull-downs are
   safety-relevant, not cosmetic** — they must not be value-engineered away, and any rework that touches
   the MDD10A control harness must re-verify them. **VERIFIED 2026-08-13: all eight pull-downs measured
   and passing** (`pin_test.cpp` `pd` command; ASBUILT updated) — so the margin is confirmed present, but
   the structural point stands: it is the *only* safe-start mechanism. **Operating rule: leave `Q3`/`Q4` OPEN at rest.** The
   deferred **HWR-7 safety re-audit should cover this scenario explicitly.**
5. **Fuse selectivity and the two changed ratings need review (`HWA-1`).** Three concerns, none of which
   I will resolve by guessing:
   - **`F1` 25 A over `F2` ~20 A leaves only ~5 A of discrimination.** A hard actuator fault may clear
     `F1` (killing the whole robot) instead of only `F2`.
   - **`F? DRIVE ~20 A` sits in series under `F2` ~20 A — identical ratings, so there is no
     discrimination at all** between them; a drive-branch fault could clear either one.
   - **`F? Servo ~7.5 A?`** carries the user's own question mark. For reference, the documented estimates
     are **~11 A stall for the 4 steering servos alone** (§12.4, 4×~2.7 A) plus **~8 A realistic / ~16 A
     theoretical for the 6 arm servos** (§12.4). 7.5 A may well be right for *realistic sequential*
     motion but looks low against stall. **Needs the HWA-1 load budget, not a guess.**
   - Also: the four new fuses carry placeholder designators **`F?`**, which is not DIN EN 81346
     conformant. They need real numbers (`F5`–`F8` are free) once the values are settled.
6. **~~BESTELLLISTE orders a 30 A `F1`~~ — RESOLVED 2026-08-13.** Charge 1 Nr. 6 (30 A) is struck as the
   wrong value and superseded by **Charge 2 Nr. 22** (MIDI 25 A slow + holder), which is still `offen`:
   a 25 A fuse in **non-MIDI form** is currently fitted as a stopgap, so the position is **not** closed.
   Note `F1` 25 A sits **below the pack's 28 A pulse rating** — a legitimate pulse could clear it; that
   part of decision 5 remains open. The four new
   branch fuses are probably covered by the JOREST ATO set already on the list (Charge 1 Nr. 8), but the
   count/ratings should be re-checked once decision 5 lands.

### 9.6 Clean-shutdown button — now `HWR-40` (user request 2026-08-12)

**RAISED 2026-08-13 as `HWR-40` (must, MVP, draft) in the team's internal requirements document
§10.3** (tracked internally, not in this repository) — this section is no longer the open TODO it
was written as; **HWR-40 is the authoritative text** and this stays as the
electrical-side view. HWR-40 decided that power removal remains **manual**, so **no power latch exists**
in it as it stands, and implementing one must not be recorded as implementing HWR-40. No component has
been selected (HWA-10).

> **SUPERSEDED 2026-08-19 — READ §2.1 BEFORE WIRING ANYTHING FROM THIS SECTION.** The sentence below
> (*"no part numbers, currents or **GPIO assignments** invented"*) was true when written and is **no longer
> true for the GPIO assignment**: the button's **signal pin is now assigned and bench-verified** — **Pi
> `GPIO17`, header pin 11**, GND pin 9, VCC pin 1 = **3.3 V only**, idle HIGH / pressed LOW. See **§2.1**,
> which is the **authoritative pin assignment**. The rest of the sentence still holds: **no part numbers and
> no currents** are fixed here (component selection remains open under **HWA-10**).

~~Recorded below at requirement level: behaviour and intent only, **no part
numbers, currents or GPIO assignments invented.**~~ *(Superseded for the GPIO assignment — see the box above
and §2.1. Retained as the historical record of this section's original scope.)*

**Intent.** A dedicated operator button that shuts the robot down **cleanly**: pressing it triggers an
**orderly OS shutdown** ~~and only **then** removes power~~ — **corrected 2026-08-19: the button never removes
power at all.** It halts the OS; **the operator then removes power at `Q1`** (decision (ii), manual — see the
third bullet below). ~~The effect is that **`Q1` stops being the normal way to switch the robot off**~~ —
**more precisely:** `Q1` is still operated on **every** shutdown, but it is no longer the thing that *shuts
the machine down*; it only **de-energises an already-halted system**, and it remains the
**hard / emergency disconnect** for the case where the OS is already hung. That is what resolves open
decision 2 from the operator side.

**Behaviour to specify** *(written 2026-08-12 as a wish list — annotated 2026-08-19 against what has
since been decided; these are **targets**, and where a target is not met today the bullet says so):*
- One deliberate operator action requests shutdown (distinct from the E-stop, which must stay a pure
  motion cut — the two must not be confused under stress). **Still true, but narrowed 2026-08-19:** the
  control now carries **two** functions, so the *shutdown* request is specifically the **long press
  (≥ 3 s)**; a short press (< 1 s) restarts the ROS2 processes and leaves the Pi up, and the **1 s … 3 s
  band is deliberately dead**. Values and rationale: **§2.1**. The E-stop distinction is unchanged and
  remains load-bearing.
- ~~The system signals progress and, crucially, **when it is safe to remove power**.~~
  > **NOT MET TODAY — this is a target, not a description of the machine (2026-08-19).** It is HWR-40
  > **criterion 2**, and **nothing on the robot currently provides it**: **HWR-21** (the Pi-independent
  > status display driven by `esp32-drive`) **is not built**, and a **dedicated feedback LED at the button
  > was declined by the user on 2026-08-19** (§2.1). So there is at present **no machine-side indication of
  > shutdown progress and no "safe to remove power" signal** — and, because the button has no local
  > feedback either, a press discarded in the dead zone is indistinguishable from a dead button. **HWR-21 is
  > the only planned path to closing this.** No substitute is specified here, and none may be improvised
  > into the wiring: read this bullet as an open gap, not as a feature.
- ~~Power is removed only after the OS has halted — either by an operator prompted by that signal, or
  automatically by a latch.~~
  > **SUPERSEDED 2026-08-13 (decision (ii)) — power removal is MANUAL. There is no latch.** What actually
  > holds: after the OS has halted, the **operator removes power at `Q1`**, prompted by the "safe to remove
  > power" indication **once HWR-21 exists** (see the bullet above — today that prompt does not exist).
  > **No automatic latch is part of HWR-40**, and HWR-40's own text states that implementing one **must not
  > be recorded as implementing HWR-40**. An automatic latch remains a **possible later iteration** only —
  > it would be a new single point of failure in the very path that powers the Pi, which is why it was
  > rejected for now. *(The one part of the old text still true: `esp32-drive` **would be** the natural
  > latch host **if** a latch is ever added, since it stays alive while the Pi goes down — see the coupling
  > bullet below.)*

**Couplings to existing requirements:**
- **HWR-7 (switching levels / E-stop scope):** the new button is a *third* level alongside E-stop
  (motion off, logic alive) and `Q1` (everything off). The level hierarchy must stay unambiguous.
- **HWR-21 / OP-H6 (Pi-independent status display, `A6` driven by `esp32-drive`):** the display is the
  natural place to show shutdown progress and the **"safe to switch off"** state, precisely because it
  does not depend on the Pi being up. **`HWR-21` IS NOT BUILT (as of 2026-08-19)** — this is where that
  indication *is to* live, not where it lives today; see the struck second bullet above.
- **ESP32 survives an actuator cut-off** (powered from `+5V_LOGIC` via a VBUS-cut hub link, HWR-2/§0):
  it is still alive while the Pi is going down and **after it has halted**. **That property is unchanged and
  stays load-bearing — but only for HWR-21**, the Pi-independent status display: `esp32-drive` is what can
  still show *"shutdown running"* / *"safe to remove power"* at a moment when the Pi by definition cannot.
  It is also the natural host **if** an automatic power latch is ever added (none is required today —
  HWR-40 decided power removal stays **manual**).
  > **SUPERSEDED 2026-08-19 — the ESP is NOT the host of the shutdown-request signal.** ~~this makes
  > `esp32-drive` the natural host for both the shutdown-request signal and any power latch~~ — the trigger
  > was **decided on 2026-08-13 to be a Pi GPIO**, evaluated by a small independent systemd-managed daemon,
  > precisely so the halt path does **not** depend on the micro-ROS link. The pin is assigned in **§2.1**
  > (**Pi `GPIO17` / header pin 11**). **Do not wire the button to the ESP32.** Wiring it to the ESP would
  > route the shutdown request through exactly the link that is dead in the case the button exists for.
- **Open decision 2** above (hard power-off risk) is the problem this TODO exists to close.

**Action: DONE 2026-08-13** — raised as `HWR-40`. Remaining work lives in HWR-40 and HWA-10 (component
selection), not here. **Electrical/pin work since then lives in §2.1** (pin assignment, polarity, 3.3 V rule,
press timing) — this section is history plus the HWR-21 coupling, **not** a wiring instruction.

Authoritative electrical drawing: the QET wiring/terminal diagram (as-drawn wiring / terminal
level; internal source file maintained in the team's working repository, not exported here — a
rendered PDF is published as the Electronics Overview wiki asset). The internal KiCad sheet for
power distribution still shows the **pre-2026-08-12** architecture and is **out of date** with
respect to this section.

## 10. Protection / resistor pass (2026-07-21)

> ⚠️ **WHICH DRAWING? — clarified 2026-08-24.** The `Where` column below names **KiCad sheets**
> (`power_distribution`, `drivetrain`, `logic_sensors`), so "ADDED" in this table means *added to the
> KiCad schematic*. **§9.6 states that the KiCad `power_distribution.kicad_sch` sheet still shows the
> pre-2026-08-12 architecture and is out of date**, and the authoritative electrical drawing is the
> `.qet`. Verified against the `.qet` on 2026-08-24: **it contains zero diode elements** — the flyback
> diodes `D2`/`D3`/`D4` across the `K1`/`K2`/`K3` coils are **NOT in the authoritative drawing**, whatever
> their state in KiCad. Rows are annotated below where the two drawings differ. Do not read this table as
> a statement about the `.qet` or about the machine.

Added to the schematic (or flagged where a value/decision is genuinely open):

| Item | Where | Status |
|---|---|---|
| Flyback diode across **K1** coil | power_distribution (D2) — **KiCad only** | ADDED in KiCad. ⚠️ **NOT PRESENT in the `.qet` (verified 2026-08-24: 0 diode elements in the file).** Fitted state unknown from the desk; the part is on `BESTELLLISTE.md` Charge 2 Nr. 12 with no order status. |
| Flyback diode across **K2** coil | power_distribution (D3) — **KiCad only** | ADDED in KiCad. ⚠️ **NOT PRESENT in the `.qet` (verified 2026-08-24: 0 diode elements in the file).** Fitted state unknown from the desk; the part is on `BESTELLLISTE.md` Charge 2 Nr. 12 with no order status. |
| Flyback diode across **LiDAR relay K3** coil | power_distribution (D4) — **KiCad only** | ADDED in KiCad. ⚠️ **NOT PRESENT in the `.qet` (verified 2026-08-24: 0 diode elements in the file).** Fitted state unknown from the desk; the part is on `BESTELLLISTE.md` Charge 2 Nr. 12 with no order status. |
| **LiDAR GPIO driver** (GPIO23/LIDAR_EN): BSS138 + **100 Ω gate series (R2)** + **10 kΩ gate pull-down (R3)** | power_distribution (Q6/R2/R3) | ADDED (keeps LiDAR OFF while Pi GPIO high-Z at boot) |
| **Pull-downs ~10 kΩ on all 8 MDD10A control inputs** (PWM/DIR) | drivetrain (R30–R37) | ADDED (defined-OFF while ESP32 boots/floats) |
| **Reverse-polarity crowbar** Schottky + fuse | power_distribution (D1 + F1) | **NO LONGER PRESENT in the `.qet` (2026-08-12)** — `D1` was deleted in the user's restructure, element *and* `schottky.elmt` definition. The robot as drawn has **no reverse-polarity protection**. See §9.5 open decision 1, which is **CLOSED 2026-08-13 (user): `D1` is NOT required** — the drawing's state matches the decision and is no longer a defect; the parenthetical recommendation to re-add it is **superseded**. ⚠️ **CORRECTED 2026-08-24:** this row previously ended *"Still on the order list (BESTELLLISTE Charge 2 Nr. 13)"* — `BESTELLLISTE.md` Nr. 13 has been marked **`❌ entfällt`** since 2026-08-13. |
| **Pre-charge resistor at K1** (was: limits inrush into bulk elkos C2/C3) | power_distribution (note) | **OMITTED (deliberate, user 2026-07-21; justification REWRITTEN 2026-08-10)** — the original reason ("inrush into the bulk elkos C2/C3 is tolerated") **no longer holds: `C2`/`C3` are deferred out of build iteration 1** (§12.5), so there is **no bulk capacitance at `K1` left to charge** and the inrush concern **lapses**. `K1` switches the servo/arm load directly, which the JD1912 contacts handle. **Revisit this omission if `C2`/`C3` return.** Retrofit anyway if contact erosion appears. |
| **I2C pull-ups** SDA/SCL (BNO085) | logic_sensors (note) | VERIFY on breakout — do NOT add (STEMMA-QT board + Pi i2c1 already have pull-ups) |
| **Encoder output pull-ups** (4× GB37-50) | drivetrain (note) | **NONE (confirmed, user 2026-07-21)** — push-pull Hall outputs → no pull-ups needed |
| **Battery-voltage ADC divider** | power_distribution (note) | ⚠️ **CORRECTED 2026-08-24 — DECIDED AND BUILT, not an open option.** `BATT_SENSE` on **GPIO2 (ADC1_CH1)** is listed as a **primary** in §1.1 and marked *"DECIDED+BUILT 2026-07-21"*; the `.qet` carries `B6` (25→5 V sense module), `R40` (~1 k series) and `C10` (~100 nF filter). *Superseded text:* ~~**OPTION, FEASIBLE** — GPIO1 and GPIO2 are free (spare pool)… User decision on whether to fit it~~ — **GPIO2 is no longer free**, it is consumed. Analysis retained in §11. **Residual defect:** `netlist_check.py` C-06 reports `C10` wired on **one pin only** in the `.qet`. |
| **Indicator-LED series resistors** | — | N/A — no discrete status LEDs on the schematic (ESP32-S3 onboard WS2812 is self-contained) |

**Common-ground requirement (decided, OP-H9):** ESP32 GND ↔ both MDD10A GND ↔ encoder-supply GND ↔ 12 V
motor-supply GND all tied to the one star-ground net — belongs on the schematic, mandatory for defined
PWM/DIR and encoder levels.

## 11. Battery-voltage monitoring — ESP32-S3 ADC feasibility (analysis 2026-07-21)

**Question:** can an ADC pin be freed on the `esp32-drive` (ESP32-S3-N16R8) for battery-voltage sensing,
or is the map too tight?

**ADC banks on the S3:** ADC1 = GPIO1–GPIO10 (usable with WiFi active); ADC2 = GPIO11–GPIO20
(**excluded** — ADC2 is unreliable/blocked while WiFi is active, per the task). So only ADC1 (GPIO1–10)
is a candidate.

**ADC1 pin occupancy in the current map (§1.1):**

| GPIO | ADC1 ch | Current use | Free for ADC? |
|---|---|---|---|
| GPIO1 | CH0 | spare (guaranteed-free) | **YES** |
| GPIO2 | CH1 | spare (guaranteed-free) | **YES** |
| GPIO3 | CH2 | none, but **strapping pin** | no — a divider would pull the strap at boot (avoid) |
| GPIO4 | CH3 | FL_PWM | no |
| GPIO5 | CH4 | FL_DIR | no |
| GPIO6 | CH5 | FR_PWM | no |
| GPIO7 | CH6 | FR_DIR | no |
| GPIO8 | CH7 | FL_ENC_A | no |
| GPIO9 | CH8 | FL_ENC_B | no |
| GPIO10 | CH9 | FR_ENC_A | no |

**Result: it is NOT tight — two clean ADC1 pins are free: GPIO1 (CH0) and GPIO2 (CH1).** No primary
function has to move. **Recommendation: use GPIO2 (ADC1_CH1)** for the battery divider and keep GPIO1 as
the last general spare. **Cost:** GPIO1/GPIO2 also serve as *alternative* (fault-relocation) spares for a
couple of motor/encoder lines (§1.1/§1.2); consuming one leaves one fewer relocation spare — no active
function is displaced. GPIO3 (the only other ADC1 pin) is a **strapping pin** and must not carry a divider,
so leave it unused.

**DECIDED + BUILT (user go 2026-07-21) — GPIO2 (ADC1_CH1) = BATT_SENSE.** Implemented on the
`logic_sensors` sheet (B6 / R40 / C10) and GPIO2 is now a primary in §1.1 (no longer a spare).

Implementation (uses the user's existing **"25 V→5 V" sensor module = classic 5:1 divider**, typ.
R_top = 30 kΩ / R_bottom = 7.5 kΩ — represented as one external module `B6`, not discrete parts):
- **VIN = +12 V rail (post-F1)**; module **GND = common star ground**.
- **OUT → ~1 kΩ series (R40) → GPIO2 (ADC1_CH1)**; **~100 nF filter cap (C10) at the ADC node → GND**.
- **Scaling 5:1:** 14.6 V ≙ **2.92 V**, 15.0 V ≙ **3.00 V** at the ADC (< 3.3 V OK). **Do not exceed
  ~16.5 V input** (≈3.3 V at the pin). Divider current ≈ 0.29 mA (negligible on 20 Ah).
- **Firmware:** ADC1_CH1, **12 dB attenuation + eFuse calibration**, average a few samples.
- **GPIO1 (ADC1_CH0)** remains the last free ADC1 pin (reserve) — this leaves one fewer fault-relocation
  spare. GPIO3 (the only other ADC1 pin) stays unused (strapping pin).

---

## 12. Star-ground (GND) distribution scheme — TBD / PARTLY SUPERSEDED (2026-07-24, revised 2026-08-10)

**Status: TBD / PARTLY SUPERSEDED — this section is NOT an approved target state.** Parts of the scheme
as originally written have since been overruled by explicit decisions (listed immediately below), so it
must not be read as a build instruction. What stays valid and useful is the **tree in §12.2** and the
**allocation table in §12.3** as **reference material** for branch assignment and cross-section
reasoning. It feeds **HWR-4** (defined star ground), **HWR-16** (matched cross-sections), and the
**common-ground rule** (OP-H9 / §10 / §0.3).

**Decided deviations from the scheme as written — do NOT "correct" these back:**
1. **USB hub GND stays on `X-PWR`, not on `X-SEN`** (decided 2026-08-10). The hub runs on **12 V**, so
   its return is power-side. See §12.3 row 21 — including the wiring constraint that `X-SEN:3` must not
   be bridged into the `X-SEN` comb bar.
2. **Cameras get no second supply** — they are bus-powered by the 12 V-fed hub. See §12.3 rows 22/23.
3. **`C2`/`C3`/`C4` are deferred out of build iteration 1** — see §12.5 for the capacitor scope actually
   being built.
4. **NO `X-ENC` strip. Encoder returns AND the MDD10A pull-down returns go to the `esp32-drive` (`A1`)
   ground node** (decided 2026-08-10). **Reasoning (user):** the encoder supply `+3V3` is **sourced from
   `A1`**, so supply and return pair up at the same reference — which is the point of FLAG B, reached
   without a dedicated strip. **As built 2026-08-12** the returns land on the `N-ENC-A` / `N-ENC-B`
   collector nodes, which reference the ESP32 ground node directly — **no dedicated collector terminals**.
   Interim terminals `X-CPU:10`–`X-CPU:14` were added on 2026-08-10 and then **removed by the user in the
   GUI round-trip as unnecessary**; the older target `X-CPU:4` stays retired. The `X-CPU` numbering
   therefore ends at `:9` — see the terminal-numbering note under §12.3. **`X-ENC` is intentionally absent — its
   absence is no longer a defect.**
5. **NO `X-STE` / `X-ARM` strips** (decided 2026-08-10). The two bus-servo chains stay landed **directly
   on `X-AKT:3` (steering, via `A7`) and `X-AKT:4` (arm, via `A8`)**. Physically there is **one** GND feed
   per daisy-chain, so a dedicated strip buys nothing. Those two terminals were relabelled in the `.qet`
   from `-> X-STE` / `-> X-ARM` to name the chain they actually collect (they were pointing at strips that
   will never exist). This also **closes §12.7 question 4**.
6. **The `X-ENC`/`X-STE`/`X-ARM` boxes in the §12.2 tree and their rows in §12.4 are NOT build targets** —
   see the inline markers there.

**Implementation state in the `.qet`** (internal QET wiring/terminal diagram source file, not in
this repository, as of 2026-08-10): the strips **`X-GND`, `X-AKT`, `X-DRV`, `X-LOG`, `X-CPU`,
`X-SEN`, `X-PWR` exist and carry their trunk conductors**. Per deviations 4/5 there is **no
`X-ENC`, `X-STE` or `X-ARM`**.

**✅ RESOLVED 2026-08-12 — the ground loop is GONE (verified).** For the record, the defect was: the
encoder and pull-down returns were **cross-bonded between power ground and signal ground** via a bridge
`N-GND1 ↔ X-DRV:1`, giving a second logic↔power path that bypassed `X-GND` — a ground loop, a **§12.2 rule
1** violation and a **FLAG B** violation (not a missing connection).

The user's restructure removed `N-GND1`. **Verified independently on 2026-08-12** by electrical trace
*through* terminal blocks: all four encoder-cable grounds (`M1` FL, `M2` BL, `M3` FR, `M4` BR) now reach
**only** the `esp32-drive` (`A1`) / `X-CPU` reference, with **zero** power-ground members reachable while
`X-GND` is excluded; and both `A3`/`A4` MDD10A drivers still reach `X-GND`. `netlist_check.py` **C-08 is
clear.**

The requirement itself is unchanged and now **satisfied**: encoder GND must not sit on the motor/driver
power ground, and it no longer does.

### 12.1 Problem being fixed

The current `.qet` has **48 GND conductors** on one net (`GND 6 mm²`) fanning off a **single** main GND
terminal. That is unmanageable graphically and electrically poor: motor PWM return currents share
ground segments with encoder/sensor signal returns → **common-impedance coupling** (noise on the very
signals — encoder A/B, ADC, I2C — that must stay quiet). The fix is a **hierarchical star ground**:
few thick *trunk* conductors from the main star point to *sub-distribution* terminal strips, with each
load's return landing on its nearest strip.

### 12.2 Topology (tree)

```
X-GND  (main star point = battery minus / single reference bar; DC/DC GND bonds here)
├── X-AKT   Actuator power-GND collector           [trunk 4 mm²]
│   ├── X-DRV   drive-motor drivers (2× MDD10A B-)  [4 mm²]
│   ├── X-STE   steering servos (4× ST3215)         [2.5 mm²]   << NOT BUILT (deviation 5):
│   │                                                              chain GND lands on X-AKT:3
│   └── X-ARM   arm servos (6× STS3215)             [2.5 mm²]   << NOT BUILT (deviation 5):
│                                                                  chain GND lands on X-AKT:4
├── X-LOG   Logic/signal-GND collector             [trunk 2.5 mm²]
│   ├── X-CPU   Pi 5 + esp32-drive                  [1.5 mm²]
│   │             (no dedicated collector terminals - removed by the user 2026-08-12)
│   │             encoder + pull-down GND arrive via the N-ENC-A/N-ENC-B collectors (deviation 4)
│   ├── X-SEN   sensors / LiDAR / cameras            [1.5 mm²]  (hub GND -> X-PWR, deviation 1;
│   │                                                            cameras via hub, deviation 2)
│   └── X-ENC   4× drive-motor encoders             [0.5 mm² / AWG20]  << NOT BUILT (deviation 4):
│                                                       encoder GND goes to the ESP32 A1 ground node
└── X-PWR   Main harness (DC/DC bond, relay coils)  [4 mm² DC/DC strap + 0.5 mm² coil returns]
            + X-SEN:3 USB hub GND (12 V-fed, deviation 1)
```

**The three `<< NOT BUILT` branches are kept only to document the reasoning that led to the decision.**
The signal-vs-power separation they were invented for is still required — for the encoders it is now
achieved by referencing them to the ESP32 ground node instead of a separate strip.

**Hard rules honored:**
1. **Pure star.** The power-GND subtree (`X-AKT`) and the signal-GND subtree (`X-LOG`) share **no**
   conductor segment — they meet **only** at `X-GND`.
2. **Single bridge** between logic-GND and power-GND = `X-GND` itself. The non-isolated DC/DC
   (szwengao 5 V/10 A) internally commons its 12 V-input GND and 5 V-output GND; that bond is placed
   **at/next to `X-GND`** (on `X-PWR`) with a short heavy strap, so it does **not** create a second
   meeting point. No ground loops.
3. **Returns sized ≥ their live conductor** (HWR-4 / §8.3): main GND = 6 mm², actuator trunk = 4 mm²,
   logic trunks match the 5 V feeds.

### 12.3 Full allocation table — every current GND consumer → its strip

| # | GND consumer (as-built) | Strip | Live rail | Stub cross-section | Notes / flags |
|---|---|---|---|---|---|
| 1 | MDD10A #1 (FRONT) power GND (B−) — drives FL, FR | **X-DRV** | +12V_DRV (K2) | driver lug | Motor current returns **through** the driver, not the motors directly. FRONT/REAR grouping per user decision 2026-07-29. |
| 2 | MDD10A #2 (REAR) power GND (B−) — drives BR (ch 1), BL (ch 2) | **X-DRV** | +12V_DRV (K2) | driver lug | FRONT/REAR driver-to-wheel grouping (OP-H9/HWA-11); **rear channel order as-built 2026-08-12, see §1.3**; ~~physical install/~~user acceptance still open (⚠️ **CORRECTED 2026-08-24:** the install is as-built per §1.3; only HWA-11 acceptance remains). |
| 3–6 | 4× drive-motor tails (FL/BL/FR/BR, M±) | via driver | — | 0.5 mm²/AWG20 (§8.2) | Land on driver MA/MB, **not** on a GND strip. |
| 7 | ST3215 FL (ID11) power GND | **`X-AKT:3`** *(no `X-STE` strip — deviation 5)* | +12V_ACT (K1) | factory jumper (§8.1) | **REVISED 2026-08-10.** Steering servos daisy-chain P+G+data, so physically there is **one** chain-GND feed; it lands **directly on `X-AKT:3`** (via the `A7` Waveshare adapter). Rows 7–10 are traceability only — they are **not** four separate returns. Already wired this way in the `.qet`. |
| 8 | ST3215 FR (ID14) power GND | via the chain → **`X-AKT:3`** | +12V_ACT (K1) | factory jumper | |
| 9 | ST3215 BL (ID12) power GND | via the chain → **`X-AKT:3`** | +12V_ACT (K1) | factory jumper | |
| 10 | ST3215 BR (ID13) power GND | via the chain → **`X-AKT:3`** | +12V_ACT (K1) | factory jumper | |
| 11–16 | 6× STS3215 arm (J1–J6) power GND | **`X-AKT:4`** *(no `X-ARM` strip — deviation 5)* | +12V_ACT (K1) | factory jumper (§8.1) | **REVISED 2026-08-10.** Daisy-chained via the `A8` XIAO bus board → **one** chain-GND feed, landing **directly on `X-AKT:4`**. Already wired this way in the `.qet`. |
| 17 | Raspberry Pi 5 GND | **X-CPU** | +5V_LOGIC | 1.5 mm² | Clean logic branch (HWR-2). |
| 18 | esp32-drive (YD-ESP32-S3) GND | **X-CPU** | +5V_LOGIC | 1.5 mm² | **Reference node for encoders + BATT_SENSE + PWM/DIR** — see §12.5. |
| 19 | MDD10A pull-downs R30–R37 GND (8×) | **ESP32 (`A1`) ground node**, via the `N-ENC-A`/`N-ENC-B` collectors | — | 0.5 mm² | **REVISED 2026-08-10.** Hold PWM/DIR low at boot; **must reference `esp32-drive` (`A1`) GND, not power ground** — that requirement is unchanged. **RESOLVED 2026-08-12** — it was violated until the `N-GND1` cross-bond was removed; see the resolved ground-loop box in §12. The eight pull-downs sit together on the perfboard next to the ESP, so their GND ends are a **local perfboard rail** and **one** wire leaves it. They land on the `N-ENC-A` (R30–R33) / `N-ENC-B` (R34–R37) collector nodes, which now reach **only** the ESP32 (`A1`) / `X-CPU` reference — the `N-GND1` cross-bond to the MDD10A power ground has been removed. **No dedicated collector terminal is used:** the user removed the interim `X-CPU:10` in the GUI round-trip as unnecessary, and the retired `X-CPU:4` is not reinstated. |
| 20 | LiDAR LD06 GND | **X-SEN** (`X-SEN:2`) | +5V_SENS (K3) | 1.5 mm² | Switchable LiDAR branch. With the hub moved to 12 V (row 21), this and row 24 are the **only** remaining `X-SEN` stubs. |
| 21 | USB hub GND (terminal keeps designator `X-SEN:3`) | **X-PWR** — **DECIDED 2026-08-10** | **`+12V`** (**not** `+5V_SENS`) | **TO-VERIFY (HWA-1)** | **CORRECTED + DECIDED (user 2026-08-10):** the hub is **12 V-fed** (Vansuny active hub, see the team wiki Hardware/Electronics part list, Charge 1 Nr. 10), so its return current is **power-side**. Keeping it on **`X-PWR`** holds that current **out of the 1.5 mm² logic strand** (`X-LOG`→`X-SEN`). ⚠ **BUILD CONSTRAINT — `X-SEN:3` must NOT be bridged into the `X-SEN` comb bar.** Doing so would create a **second logic↔power meeting point** and **violate §12.2 rule 2** (single bridge at `X-GND`). The terminal keeps the historical `X-SEN:3` designator, but its **node is `X-PWR`** — designator and node deliberately differ here. Stub cross-section is **not** the old 1.5 mm²: it must be re-derived from the hub's 12 V input current (HWA-1) — no value is assumed. Hub GND additionally bonds to the Pi through the USB-uplink cable (intra-logic, accepted). |
| 22 | Gripper camera GND (UVC, via hub) | *none — returns through the hub* | **USB bus power from the 12 V-fed hub** | via hub | **CONSEQUENCE OF ROW 21:** the hub supplies the cameras from its own 12 V-fed 5 V, so the cameras need **NO second supply** — no `+5V_SENS` feed, no `X-SEN` stub, no own strip terminal. Their GND returns through the hub's USB ground → `X-SEN:3` → **`X-PWR`**. |
| 23 | Front camera GND (UVC, via hub) | *none — returns through the hub* | **USB bus power from the 12 V-fed hub** | via hub | Same as row 22 — no second supply, no own strip terminal. |
| 24 | GNSS reserve GND (Pi UART/USB) | **X-SEN** | +5V_SENS | 0.5 mm² | Reserve (stage 2). |
| 25 | IMU BNO085 GND | **X-CPU** *(not X-SEN)* | Pi 3V3 | 0.5 mm² | **FLAG A:** I2C sensor — its GND is the Pi I2C reference; route with the I2C cable to Pi/X-CPU, not the sensor strip. |
| 26 | Status display Sharp LCD GND | **X-CPU** *(not X-SEN)* | +5V_SENS | 0.5 mm² | **FLAG A:** SPI slave of the ESP32 — GND references ESP32 for clean SPI, though powered from +5V_SENS. |
| 27 | BATT_SENSE module B6 GND | **X-CPU** *(not X-SEN)* | — | 0.5 mm² | **FLAG A:** ADC divider bottom must share the **ESP32 ADC** reference or the reading skews (§11). Route to X-CPU. |
| 28–31 | 4× drive-motor encoder GND (FL/FR/BL/BR) | **ESP32 (`A1`) ground node**, via the `N-ENC-A`/`N-ENC-B` collectors *(**not** `X-ENC` — that strip is not built; no dedicated per-cable terminals either)* | **+3V3, sourced from `A1`** | 0.5 mm²/AWG20 | **FLAG B (key) — REVISED 2026-08-10, requirement UNCHANGED, RESOLVED 2026-08-12.** Encoders sit physically **on the power-side motors** but are **signal-GND**: their 4-wire cable GND **MUST NOT** land on the adjacent motor/driver power GND. **What changed is only the destination:** instead of a dedicated `X-ENC` strip, the returns go to the **`esp32-drive` (`A1`) ground node**. **Reasoning (user):** `+3V3` for the encoders is **sourced from `A1`**, so supply and return pair up at the same reference — exactly what FLAG B asks for, and a separate strip adds a node without adding separation. **RESOLVED 2026-08-12 — no longer violated:** the returns land on the collector nodes `N-ENC-A` (M1 FL, M2 BL) / `N-ENC-B` (M3 FR, M4 BR), which now reach **only** the ESP32 reference; the `N-GND1` → `X-DRV:1` cross-bond that made this a ground loop is gone, and `netlist_check.py` C-08 verifies it for all four motors. **No dedicated per-cable terminals are used** — the user removed the interim `X-CPU:11`–`X-CPU:14` in the GUI round-trip as unnecessary. |
| 32 | DC/DC szwengao GND (12 V-in ⊕ 5 V-out, bonded) | **X-PWR** → **X-GND** | +12V / +5V | 4 mm² short strap | The logic branch's *source* GND; sits at the star (rule 2). |
| 33 | K1 coil low side (actuator relay, JD1912) | **X-PWR** | COIL_SUP | 0.5 mm² | <1 A coil. |
| 34 | K2 coil low side (drive relay, JD1912) | **X-PWR** | COIL_SUP | 0.5 mm² | <1 A coil. |
| 35 | K3 coil low side (LiDAR relay) | **X-PWR:5 = SPARE** (not wired) | — | — | **CORRECTED 2026-07-24:** K3's coil low side is **low-side-switched via Q6** (BSS138, LIDAR_EN): `+5V_SENS → K3 coil → Q6 drain → Q6 source → GND`. It is therefore **not** a direct GND stub; its GND reference is at **Q6 source on X-CPU** (see row 36). X-PWR:5 is retained as a **labelled spare** terminal, not deleted. (K1/K2, rows 33–34, *are* low-side-to-GND and do land on X-PWR.) |
| 36 | LIDAR_EN driver Q6/R3 GND (BSS138 gate pull-down) | **X-CPU** | — | 0.5 mm² | Control ref = Pi GPIO GND. Minor; grouped with logic. |
| 37 | Reverse-polarity crowbar D1 / F1 return | at battery | +12V | — | Sits at battery minus = X-GND; not a separate strip stub. **`D1` is currently absent from the drawing** (§9.5 decision 1) — this row describes where it belongs once re-added. |

**Terminal-numbering note (do NOT "fix" this):** the gap at **`X-CPU:4`** is deliberate. That
designator was the original §12 target for the pull-down returns; the user **retired it** in the QET GUI
and the replacement collectors were given fresh numbers (`X-CPU:10` pull-downs, `X-CPU:11`–`X-CPU:14`
encoder cables) because they rested on a different justification (ESP32 ground reference, deviation 4).
Those interim terminals were then **removed again by the user on 2026-08-12** as unnecessary, so `X-CPU`
now runs `:1`–`:3`, `:5`–`:9`. **Both gaps (`:4` and `:10`+) record deliberate retirements — do not
renumber to close them.**

Items **25–27** (FLAG A) and **28–31** (FLAG B) are the branch-assignment calls that need the user's
explicit blessing — they are placed by *reference* (where the signal is measured), not by *where the
device physically sits or draws power*.

**After the row 21–23 correction, `X-SEN` carries only two live stubs: row 20 (LiDAR LD06 GND) and row 24
(GNSS reserve GND).** The hub terminal `X-SEN:3` sits nodally on `X-PWR` (see the ⚠ constraint in row 21)
and the cameras have no terminal at all. Anyone re-deriving the `X-LOG`→`X-SEN` trunk cross-section
(§12.4, currently 1.5 mm² sized against fuse `F4`) should note that the hub load has left this branch.

### 12.4 Trunk cross-sections — with load reasoning

| Segment | Carries | Worst-case current | Cross-section | Basis |
|---|---|---|---|---|
| Battery minus → **X-GND** (main star) | everything | ≤ F1 = **25 A** (was 30 A; §9.3) | **6 mm² / ≈AWG 9–10** | Matches battery main feed (§8.3); return ≥ live (HWR-4). Cross-section unchanged — it was already generous at 30 A. |
| **X-GND → X-AKT** | all actuator + drive returns | ≤ F2 ≈ 20–25 A | **4 mm² / ≈AWG 11** | Actuator branch is F2-limited; matches +12V(F2) feed. |
| X-AKT → **X-DRV** | 2× MDD10A power GND | ~20 A (4-motor stall) | **4 mm²** | Matches the +12V_DRV driver trunk (§8.3). |
| ~~X-AKT → **X-STE**~~ **NOT BUILT** (deviation 5) | 4× ST3215 | ~11 A (4×~2.7 A stall) | — | No such trunk: the chain GND lands directly on `X-AKT:3`. The factory servo leads (§8.1) are the real limit, accepted as-is. |
| ~~X-AKT → **X-ARM**~~ **NOT BUILT** (deviation 5) | 6× STS3215 | ~16 A theoretical, ~8 A realistic (sequential) | — | No such trunk: the chain GND lands directly on `X-AKT:4`. Daisy-chained; motion is sequential, not all-stall. |
| **X-GND → X-LOG** | all 5 V + logic returns | ≤ DC/DC = 10 A | **2.5 mm²** | Combined +5V_LOGIC + +5V_SENS return; generous vs. DC/DC cap. |
| X-LOG → **X-CPU** | Pi 5 + ESP32 | ~5 A | **1.5 mm² / ≈AWG 16** | Matches the voltage-drop-critical +5V_LOGIC feed (§8.3). |
| X-LOG → **X-SEN** | LiDAR + GNSS reserve (**hub and cameras removed** — deviations 1/2) | ≤ F4 ≈ 3–5 A | **1.5 mm²** *(now oversized — re-derive with `F4`, HWA-1)* | Was sized against `F4` with the hub load included; the hub is 12 V-fed and the cameras hang off it, so this branch now carries only LiDAR + GNSS reserve. |
| ~~X-LOG → **X-ENC**~~ **NOT BUILT** (deviation 4) | 4× encoder | <0.2 A | **0.5 mm² / AWG20** per cable | No such trunk: each encoder cable GND goes to the **ESP32 (`A1`) ground node** (via the `N-ENC-A`/`N-ENC-B` collectors), pairing with the `+3V3` supply that `A1` sources. Cross-section still `0.5 mm²/AWG20`, sized for mechanical robustness / low-impedance signal reference. |
| **X-GND → X-PWR** | DC/DC 12 V-in return + **K1/K2** relay coil returns | DC/DC ~5 A @ 12 V; coils <1 A | **4 mm² DC/DC strap** + 0.5 mm² coil returns | DC/DC bond kept short/heavy at the star; coil returns tiny. (K3 excluded — low-side-switched via Q6, ref on X-CPU; see §12.3 row 35.) |

### 12.5 Design-guidance notes (annotation only — parts NOT placed by this scheme)

- **Decoupling belongs at each module supply, not on the GND tree:** bulk + 100 nF at every board's
  power pins; **large bulk electrolytics (≥1000 µF) sit right at the motor drivers** (MDD10A B+/B−) to
  keep PWM ripple current local and off the trunks (HWA-10 input buffer / HWR-2).
- **Use ferrite beads, never resistors, for any noisy/quiet separation.** There must be **NO resistor
  in any GND path** — the star bonds are solid copper. (The pull-downs R30–R37 / R3 are signal pull-downs
  to GND, not series elements in the GND return — they do not violate this.)
- **Common-ground rule (OP-H9) is satisfied by the star, not by extra bonds:** ESP32 GND, driver GND,
  encoder-supply GND and the 12 V motor-supply GND are all common **at X-GND**. Keep the star bar
  low-impedance (solid bar/lug, short branches) so digital PWM/DIR levels stay defined — see the open
  question in §12.7 on whether a direct ESP↔driver bond is additionally needed.
- **Encoder cabling (REVISED 2026-08-10):** the encoder 4-wire lead (Vcc 3V3, A, B, GND) runs from the
  motor back to the ESP32 area; its GND terminates on the **`esp32-drive` (`A1`) ground node**,
  deliberately bypassing the nearby motor/driver power GND. The *bypass* requirement is
  unchanged — only the destination is (no `X-ENC` strip, deviation 4). Because `+3V3` is sourced from `A1`,
  the whole encoder cable (supply **and** return) references one node.

**Capacitor scope for build iteration 1 (DECIDED, user 2026-08-10)** — this overrides the generic
"bulk elko at every driver" guidance above for *what actually gets built now*:

| Cap | Where | Iteration 1 | Reason on record |
|---|---|---|---|
| **`C1`** | Pi/ESP bulk on **`+5V_LOGIC`**, **plus the 100 nF ceramic** HWR-2 asks for; minus lands on `X-CPU` (`X-CPU:9`) | **IN — required** | **Not precautionary.** Residual risk **(a)** of the single-converter 5 V decision (HWR-2): Pi and ESP share **one** WG-1224S with `+5V_SENS`, so a sensor/USB transient on `+5V_SENS` **can reach the Pi**. HWR-2 therefore calls the Pi-input buffering **required, not optional** — `C1` *is* that buffering. |
| **`C10`** | ~100 nF batt-sense ADC filter at the GPIO2 node (§11); GND reference on `X-CPU:7` | **IN** | ADC anti-alias/noise filter for `BATT_SENSE`; already dimensioned in §11. |
| **`C2`/`C3`** | bulk on **`+12V_ACT` at `K1`** | **DEFERRED** to a later iteration | The **12 V side has wide headroom**: the **szwengao WG-1224S accepts 8–36 V input**, so an actuator-rail **sag does not reach the 5 V rails** — the 5 V rails are protected by the converter's own input range, not by 12 V-side bulk. Revisit only if 12 V-side sag/ripple is **measured** to be a problem (and then re-check the `K1` pre-charge omission, §10). |
| **`C4`** | LC filter (HWR-2) | **DEFERRED / DROPPED for iteration 1** | HWR-2 states it only as "**possibly** LC filter". It is **unmeasured**, and an **undamped LC brings its own resonance/stability risk** — adding it blind could make things worse. **Removed from the `.qet` 2026-08-10.** |

**No capacitance values are fixed here.** `C1`'s actual capacitance stays **TO-VERIFY (HWA-1 / HWR-2)** —
to be pinned from the measured Pi/ESP load step, not guessed. The 288-piece capacitor set already on the
order list (`BESTELLLISTE.md` Charge 1 Nr. 7, incl. 1000–2200 µF/25 V) covers the candidate range.

### 12.6 QET representation recommendation

Do **not** re-draw 30–48 conductors to one point. In the QET **terminal-block / bornier editor**:
- Model **X-GND** and each sub-strip (X-AKT, X-DRV, X-LOG, X-CPU, X-SEN, X-PWR — **no X-STE/X-ARM/X-ENC**,
  deviations 4/5) as **terminal strips whose terminals are internally bridged** (link/comb bars). The
  bridge bar *is* the common node — so each load return is just a **short stub to one terminal**, not a
  wire to a shared point.
- Only the **trunks** (X-GND↔X-AKT, ↔X-LOG, ↔X-PWR, and the sub-trunks X-AKT↔X-DRV, X-LOG↔X-CPU/SEN)
  are drawn as **conductors**, each carrying its cross-section label from §12.4.
- Net result: from ~48 conductors-to-one-point down to **~10 labelled trunk conductors** + tidy strips.
- **Reminder (agent process):** per the QET id-regeneration gotcha, build the strips/terminals first,
  let the user round-trip (open+save) so QET assigns canonical ids, then add trunk conductors against
  those ids. That is the *next* run — not this one.

**✅ DONE 2026-08-12 — the cut was made by the user and `N-GND1` no longer exists.** Kept as a record of
how it was analysed, because the reasoning generalises to any future bridge. Verified against the `.qet`
on 2026-08-10:

1. The legitimate power trunk is a **direct** conductor `X-AKT:2 ↔ X-DRV:1`. It does **not** pass through
   `N-GND1`, so cutting the bridge cannot break it.
2. **Cutting the loop needs exactly ONE conductor deleted: `N-GND1 ↔ X-DRV:1`.** That alone separates the
   signal/ESP ground from the MDD10A power ground.
3. **Do NOT simply delete the `N-GND1` element.** It sits **in series in the middle of the collector
   chain** (`N-ENC-B ── N-GND1 ── N-ENC-A ── N-GND2 ── A1`). Removing it also removes its two chain
   conductors and would **orphan `N-ENC-B` (M3, M4, R34–R37) from the ESP32 reference**. Either keep
   `N-GND1` as a plain collector junction, or add a replacement conductor `N-ENC-B ↔ N-ENC-A` **before**
   deleting it.
4. After the cut, the remaining node (`N-GND2` + `N-ENC-A` + `N-ENC-B` + `A1` + `X-CPU:3`) **is** the
   intended ESP ground collector. *(The plan at the time was to re-point the returns onto dedicated
   terminals `X-CPU:11`–`X-CPU:14` and `X-CPU:10`. **Superseded** — the user removed those terminals in
   the round-trip as unnecessary; the returns stay on the `N-ENC-A`/`N-ENC-B` collectors, which sit on
   this node. Outcome is electrically the same and C-08 confirms it.)*
5. **`netlist_check.py` C-08 asserts this outcome** and was proven in both directions before the cut (it
   fired on all four motors, and went clean when that single conductor was dropped). **It is now clear** —
   which is the regression guard going forward. It is phrased as a
   *topology* test — "no encoder-GND-to-power-GND path may bypass `X-GND`" — **not** as "must not share
   a net", because with a correct single-point star ground all grounds *are* one net by design (§12.5 /
   OP-H9); what separates a star from a loop is the number of paths. A net-membership test would be
   unsatisfiable and would have to be neutered to pass.

### 12.7 Open questions / decisions (for coordinator + user)

1. **[KEY] ESP↔driver common ground: rely on the star, or add a direct bond?** The MDD10A internally
   commons its power GND (B−) and its control-header GND into **one** node. If we run a dedicated ESP32
   GND wire to the driver control-GND (the literal 2026-07-17 bench "tie them together" lesson), that
   bonds ESP32 signal-GND to motor power-GND **at the driver** — a *second* meeting point = a ground
   loop, defeating the pure star for the ESP. *Analysis:* the bench failure was grounds left **floating**,
   not a star-vs-loop question; a low-impedance star bar already makes ESP32 GND (X-CPU) and driver GND
   (X-DRV) common **at X-GND**, which is sufficient for defined PWM/DIR digital levels — and it keeps the
   sensitive encoder returns (on the ESP32 `A1` ground node — deviation 4) fully off the power
   ground. **Recommendation: (a) rely on the star
   for the ESP↔driver common ground; place the MDD10A GND wholly on X-DRV (power branch); do NOT add a
   direct ESP↔driver GND wire.** *Fallback (b):* if PWM/DIR prove marginal at bench, add ONE short, thin
   *reference-only* wire ESP32-GND→driver-control-GND and accept it as a single controlled bridge.
   **Need the user's pick (a/b).**

2. **[FLAG A] Signal-reference GNDs (IMU, status LCD, BATT_SENSE) on X-CPU rather than X-SEN?** They are
   *powered* on the sensor/+5V_SENS side but *measured/clocked* against the CPU/ESP32 reference (I2C, SPI,
   ADC). Placing their GND on **X-CPU** keeps the reference clean at the cost of a slightly longer GND
   lead. **Recommendation: X-CPU (as tabled).** Confirm, or prefer X-SEN for wiring convenience?

3. **[FLAG B] Encoder GND on the signal branch — DECIDED 2026-08-10 (CLOSED).** The *principle* is
   confirmed: encoders bolt onto the power-side motors but must **not** return on the motor/driver power
   ground. The *destination* was decided **against** a dedicated `X-ENC` strip: the returns go to the
   **`esp32-drive` (`A1`) ground node**, because `+3V3` for the encoders is sourced from `A1` — supply and
   return then reference the same node. See §12 deviation 4 and §12.3 rows 28–31.
   **Closed as work too, 2026-08-12:** the ground loop (`N-GND1`) has been removed and C-08 verifies the
   encoder grounds now reference only the ESP32/`X-CPU` node.

4. **X-STE / X-ARM — DECIDED 2026-08-10 (CLOSED).** The bus servos daisy-chain power+GND+data, so
   physically there is **one** GND feed per chain, not 4/6 separate returns — and therefore **no strip is
   built at all**: each chain GND lands **directly on `X-AKT:3`** (steering, via `A7`) and **`X-AKT:4`**
   (arm, via `A8`). A dedicated strip would add a node without adding separation. See §12 deviation 5;
   the `.qet` is already wired this way.

5. **DC/DC output GND placement — at X-PWR next to X-GND, agreed?** Rule 2 (single logic↔power bridge)
   holds only if the non-isolated DC/DC's internal GND common sits **at** the star. **Recommendation:
   mount the DC/DC so its GND lug lands on X-PWR immediately adjacent to X-GND** (short 4 mm² strap).
   Any longer run would move the bridge away from the star and re-introduce coupling.

6. **Fuse-value dependency (non-blocking):** F2/F3/F4 are still open (HWA-1). The trunk cross-sections
   above are pinned to the *preliminary* fuse ceilings; if F2 lands above 25 A, re-check X-AKT (4 mm²).
   No change needed now — just flagged so the GND sizing tracks the final fuse decision.
