# gripperx_leader

Laptop-side reader for the SO101 leader arm (6x Feetech STS3215-class servos on a USB serial
board). Status: 2026-11-17. Written 2026-10-07 against a mock serial layer; since then run on the
robot (leader EEPROM (non-volatile servo memory) step executed 2026-10-15, below), and the follower gripper driven from leader
servo id 6 in leader mode was accepted by the user on the robot on 2026-11-16 (record: requirement
FR-17, functional requirement 17 — leader-arm teleoperation, amendment). Several FR-17 points remain
open or TO-VERIFY there. The follower side is `gripperx_arm`'s arm_action_server (leader mode, FR-17), which
gates the stream itself: rising edge on `/arm/leader/enable` after its start, fresh heartbeat,
exactly one publisher, pose match, both calibrations verified (or its
`allow_unverified_calibration` for an approved calibration session), and the SR-17 filter.

The leader is passive **unless an alignment is requested**: torque is switched OFF on open, and
nothing else is written to it until `J` asks for a self-alignment (below, user decision 2026-10-13).
Then, and only then, it writes goal position, torque on and the SRAM (volatile, non-EEPROM)
`Torque_Limit`; never an EEPROM register. During leader mode the leader is passive again.

## Wiring
Servo IDs 1-6 = limb1..limb5, gripper (same order as the follower, `arm_poses.yaml`), 1 Mbaud,
protocol 0, present position at address 56. USB board to the laptop. Your user must be in
`dialout` (it is).

## Install
1. SDK (no sudo; Ubuntu 24.04 / Python 3.12 is "externally managed", hence the flag; pyserial
   already comes from apt):
   `pip install --user --break-system-packages --no-deps feetech-servo-sdk`
   Lands in `~/.local/lib/python3.12/site-packages/scservo_sdk`, which the ROS Jazzy python picks up.
2. Build: `cd ~/gripperx_ws/Software/ros2 && source /opt/ros/jazzy/setup.bash &&
   colcon build --packages-select gripperx_leader gripperx_teleop && source install/setup.bash`
3. Stable port name `/dev/gripperx_leader` (rule matches vendor 1a86, product 55d3, serial 5A68011526; not installed until you run, from the package directory):
   `sudo cp udev/99-gripperx-leader.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`

## Run
```
export ROS_DOMAIN_ID=20      # the robot's domain; the laptop's ~/.bashrc has 0
ros2 run gripperx_leader leader_arm_node --ros-args -p port:=/dev/gripperx_leader \
    -p 'servo_ids:=[1,2,3,4,5,6]' \
    -p calibration_file:=$(ros2 pkg prefix gripperx_leader)/share/gripperx_leader/config/leader_calibration.yaml \
    [-p rate_hz:=30.0]
```
Without `calibration_file` (default `''`) the leader angles are unknown: no `/arm/leader/joint_states`,
no virtual wall, and every J alignment is refused.
`port` may be left empty if `/dev/gripperx_leader` exists or exactly one `/dev/serial/by-id/*` entry does.
`scripts/gripperx_desk.sh real` starts the node itself when `/dev/gripperx_leader` exists (servo ids
1-6 since 2026-11-13, `calibration_file:=` the installed `share/gripperx_leader/config/leader_calibration.yaml`; it is not
started if that file is missing), reuses one that is already running, and stops only its own with SIGINT so the
shutdown torque-off runs (`--no-leader` opts out). Without a leader node, J in the web teleop shows
"leader process not running" instead of going nowhere.

Topics: `/arm/leader/ticks` (JointState, position = raw ticks), `/arm/leader/joint_states`
(JointState, radians, only with a calibration file, see `config/leader_calibration.example.yaml`),
`/arm/leader/active` (Bool heartbeat at `rate_hz`, False on a failed read; silence = node dead),
`/arm/leader/calibration_verified` (Bool, transient_local, `calibration_verified` of the calibration
file; False without one),
`/arm/leader/enable` (Bool, transient_local, streaming REQUEST).

Streaming request: starts DISARMED. It is a request only; the follower must gate it itself.
`J` (keyboard or web teleop) no longer publishes it directly - see "Self-alignment" below; a second
`J` switches leader mode off (or cancels a running alignment). It falls back to OFF on a second `J`, Space, O, I or P (published before their usual action; Space never commands arm motion), teleop exit,
`max_consecutive_read_errors` failed reads, and leader node shutdown.

Operator note: activation accepts a leader up to 10 deg per joint away from the follower (user
decision 2026-10-12, `leader.activation_match_tolerance_rad`). The follower then closes that gap
ON ITS OWN at up to `leader.max_joint_velocity_rad_s` / `leader.rate_hz` per tick (0.8 / 30 = 0.0267
rad since 2026-10-16, ~0.22 s for 10 deg) while the leader is held still - expect that motion right
after `J`.

## Self-alignment on J (user decision 2026-10-13)
User request, verbatim: *"Kannst du eine Funktion entwickeln, mit der beim drücken von J der Leader
erst aktiv auf Home fährt und dann die Steuerung übernimmt?"*; chosen variant: *"Variante a, Netzteil
am Leader hat 12V"* - the leader drives ITSELF to the follower's **current** pose (not home), goes
limp, and only then requests leader mode. The follower does not move for the alignment.

**This moves the leader arm. Every run of it - certainly the first ones - is a test under the
per-test motion approval (SR-1, the motor-enable process rule: no motion without explicit approval for that test): announce it, hands clear of the leader.**

**Direct path (user decision 2026-10-15).** Robot finding that day: follower at home, leader
physically at home, and every `J` was refused - limb3's home lies at its recorded travel end
(-0.0123 rad), inside the travel margin. Now, after the enable guards below (calibration verified,
`Homing_Offset` match, fresh and consistent follower pose, leader mode not already requested, no read
errors, no unconfirmed torque-off), if EVERY leader joint is already within the follower's leader-mode
activation tolerance of the follower angle, `enable=True` is published at once and nothing is written
to the leader (state `direct: within the activation tolerance (... rad) on every joint, nothing moved;
leader mode requested`). The motion guards (voltage band, torque off, status bits, travel band, servo
position limits) do not apply to this path - nothing moves. The tolerance is NOT a leader parameter:
it is read at startup from `arm_action_server`'s `leader.activation_match_tolerance_rad` in
`gripperx_arm/config/arm_safety_filter.yaml` (`follower_config.py`; `follower_safety_filter_file`, ""
= the installed `gripperx_arm` share). One number, two readers; the node logs the value it read. File
or key missing -> every align request is refused. Caveat: the leader reads the file installed on the
LAPTOP, the follower the one deployed on the robot - they agree only while both run the same commit.

Flow: teleop `J` -> `/arm/leader/align` (Bool True, volatile: a latched request would start a motion
when the node restarts) -> this node checks, writing nothing: calibration file present and verified,
leader mode not already requested (set the moment this node publishes it, not on the echo), no read
errors, no unconfirmed torque-off pending, a fresh follower pose on `/joint_states` (the relay's
CALIBRATED URDF angles - `/arm/joint_states` is raw ticks as radians) that agrees with the previous
message (a moving follower or a second publisher of the arm joints is refused: rclpy's message info
carries no publisher id, and `/joint_states` legitimately has several publishers), and per servo
(read-only): `Operating_Mode` 0, torque off, no status error bits, `Present_Voltage` inside the servo's
own `Min/Max_Voltage_Limit` AND inside the mandatory band `align_voltage_min_v`..`max_v`, valid
position limits. Targets = the follower's URDF angles converted back through the leader calibration.
REFUSED: a follower angle outside the follower's own joint limits (`gripperx_geometry`, the single
source), a target outside the leader's radian window, a recorded leader travel missing
(`travel_min_rad`/`travel_max_rad` in `config/leader_calibration.yaml`, mandatory), a path that would
end beyond the servo's position limits (i.e. cross the 0/4095 wrap - the servo would go the long way),
or a present tick outside those limits on ANY joint (torque comes on for every joint).

Travel band (user decision 2026-10-15, "like the arm", which treats its limits as the boundary and
commands goals up to them): per joint `[travel_min_rad + align_travel_margin_rad, travel_max_rad -
align_travel_margin_rad]`. A follower angle outside the band is CLAMPED to the band edge when
`|edge - follower| + align_arrival_tolerance_rad <= activation tolerance` - so the follower can still
accept leader mode once the leader has arrived (within the arrival tolerance of the edge); farther
outside it is refused (`too far to clamp`). Arrival is checked against the clamped target; the state
text lists clamped joints. Only joints that actually move are checked against the band and window: a
joint within the arrival tolerance of the FOLLOWER angle holds without any clamp (decided first,
audit LA4 M1), as does one within the arrival tolerance of its clamped target: a non-moving joint holds its fresh
reading, whatever its angle (it may sit inside the margin at a stop). With the shipped calibration and
follower limits only limb3's lower end can clamp (follower limit -0.0122 rad, band edge +0.0377 rad);
a joint parked at that stop whose follower angle differs by more than the arrival tolerance, while
another joint has to move, is driven to the band edge, away from the stop.

Then, at least `align_still_check_sec` later: the leader must not have moved meanwhile (else
"being handled", abort before any torque). Per servo `Torque_Limit` (never raised), goal := that fresh
reading, torque on, and a read-back: a servo that moved on torque-on aborts. A linear ramp at
`align_max_joint_velocity_rad_s` (0.3), all joints arriving together, written each tick (`rate_hz`).
Each tick ONE servo (round robin) is also checked: voltage in band, `Torque_Enable` 1, `Torque_Limit`
as set (a reset servo comes back with its EEPROM value), no status error bits. On arrival within
`align_arrival_tolerance_rad` on every joint (checked only after the final setpoint was written):
torque off on all servos, **read back**; only if all read 0 is `enable=True` published (one rising
edge). Already within the arrival tolerance after clamping: nothing is written, enable at once
(normally the direct path above has answered first).

Aborts (torque off on all servos, read back, no enable, reason on `/arm/leader/align_state` and in the
log): the leader handled before torque-on or moving on torque-on, present deviating from the ramp
setpoint by more than `align_deviation_tolerance_rad` (operator holding/pushing, obstruction), any
read or write error, a monitor finding, Space (`/arm/stop`), a second `J` or O/I/P (the teleop
publishes `align` False and `enable` False), any other enable message, the follower pose going stale
(`follower_pose_timeout_sec`) or moving more than `follower_motion_tolerance_rad`, a timeout
(`align_settle_timeout_sec` after the ramp), node shutdown, and - web teleop - the operator page going
silent. A torque-off whose read-back fails is **retried every tick** until confirmed (state
`aborted: torque-off NOT confirmed ... retrying`, then `aborted: torque off now confirmed (retry)`);
no new alignment starts meanwhile. `/arm/leader/align_state` (String, transient_local): `idle`,
`aligning: ...`, `done: ... (max tick N ms)`, `aborted: ...`, `refused: ...`, `direct: ...`. The web UI
shows the last text in a line under the action keys, raises `refused`/`aborted` in the top banner for
10 s (a display choice), and logs every state as an event. Until 2026-10-15 it was only in the J key's
tooltip and the event log at the page bottom, and the log skipped every event after a web-node
restart (stale event ids in the open page) - a refused J showed nothing the operator saw.

Teleop side: the web page never re-sends J, P, I or O after a failed request (it says "Delivery
unknown ... check the J / arm state": the request may have arrived); every web one-shot carries an id and its page-side age, and the node drops duplicates and
events older than `event_max_age_sec` (1.0 s, TO-VERIFY). Space is re-sent and never dropped. In a
terminal without the kitty keyboard protocol, J bytes closer together than `leader_key_debounce_sec`
(0.5 s, TO-VERIFY) are one press (auto-repeat). That also swallows a deliberate J pressed again
within 0.5 s (a quick cancel): Space always works. With the kitty protocol there is no debounce.
A beat that hangs is aborted after 1 s by the page (TO-VERIFY); the first beat after the node has
declared the link lost carries no one-shot except Space (a delayed J cannot start an alignment).

Power: since 2026-10-13 the leader runs on a **7.4 V supply** (user; the servo labels read 7.4 V
models, the servos report 7.6-7.7 V). Their own `Max_Voltage_Limit` registers read 12.0 V on IDs
1, 3, 4, 5 and 8.0 V on ID 2, so they do not protect 7.4 V parts; the mandatory band (6.5..8.4 V,
TO-VERIFY) does. The 0.1 V/LSB unit: two points of evidence - raw 122-123 on the earlier 12 V
supply (nominal) and 76-77 on the 7.4 V supply - consistent with 0.1 V/LSB; still TO-VERIFY against a
multimeter.

Bus time per ramp tick: one sync read (5 servos) + 5 goal writes + 3 monitor reads = 9 transactions;
the node reports the measured maximum tick time in the final `done` state. Budget at 30 Hz: 33 ms.

`align_enabled: false` restores the old behaviour: the align request is answered with `enable=True`
directly, nothing is written to the leader. All thresholds and the torque limit (300 for the first
test) are TO-VERIFY (see `config/leader_arm.yaml`). After torque-off the leader may sag under gravity;
the follower's 10 deg activation match then decides as before. At startup the node logs, read-only,
model number, present voltage and its limits, operating mode, position limits and torque limit of
every servo.

**Leader EEPROM registers, read 2026-10-13 (read-only; recorded as reported on the day).**

| servo | Min/Max_Position_Limit | Homing_Offset (raw register) |
|---|---|---|
| 1 limb1 | 760 / 3232 | 3624 |
| 2 limb2 | 823 / 3199 | 3353 |
| 3 limb3 | 873 / 3076 | 1323 |
| 4 limb4 | 865 / 3188 | 1748 |
| 5 limb5 | 0 / 4095 | 3325 (present 3631) |

- **Do not "clean" the Homing_Offsets.** Present_Position = Actual - Homing_Offset, and every value in
  `config/leader_calibration.yaml` (zero_ticks, travel) was measured from Present_Position WITH these
  offsets in place. Changing an offset silently shifts every leader angle.
- **The aligner reads Min/Max_Position_Limit at every align request** and refuses a path whose end, or
  a present position, lies outside them (on top of the radian travel check). These limits are
  narrower than the recorded hand travel on every joint: in ticks, recorded travel id1 397..3116,
  id2 1869..4223 (crosses the wrap), id3 467..2670, id4 183..2507. Assuming the registers are in the
  Present_Position frame (LeRobot writes them from present readings after homing; NOT verified on
  this leader - TO-VERIFY), the target ranges the alignment accepts are, in leader radians,
  intersected with the follower's joint limits and the travel margin:
  limb1 [-1.48, 1.88], limb2 [-1.78, 0.16], limb3 [0.04, 2.74], limb4 [-3.23, -0.81],
  limb5 [-2.70, +0.58]. **limb4 at its own calibration zero (tick 337) is below its Min 865**: with
  limb4 near 0 rad every alignment is refused ("present tick outside the servo position limits").
  Whether that is right depends on the register frame; it is the conservative reading.
- **limb5 and the wrap.** limb5's zero sits at tick 3718 (sign +1, present 3631 on 2026-10-13) and its
  limits are the full 0/4095: every target above about +0.58 rad (+33 deg) ends above tick 4095 and is
  refused. The fix is prepared as the maintenance tool `leader_eeprom` (procedure below), NOT yet run.
- **Measured wrap behaviour, 2026-10-13** (read-only: `/arm/leader/ticks`, which is the raw 2-byte
  Present_Position register, recorded 106 s at 30 Hz while the user turned the limp leader; file
  outside the repo):

  | joint | present range | wrap |
  |---|---|---|
  | limb1 | 1635..1935 | not reached |
  | limb2 | 9..4083 | **wraps** 4083 -> 21 at its bottom stop (step of about +38 ticks) |
  | limb3 | 851..2668 | not reached |
  | limb4 | 187..1983 | not reached |
  | limb5 | 1825..5638 | **no wrap**: continuous above 4095 (1680 samples > 4095, max step 73) |

  Why limb2 wraps and limb5 does not is NOT determined. Both offsets are negative and similar
  (limb2 3353 = -1305, limb5 3325 = -1277), so the offset alone does not explain it; the one register
  difference seen is the position limits (limb2 823/3199, limb5 0/4095); firmware versions
  (registers 0/1) were not read. Hypothesis only, TO-VERIFY: the limits decide it - then limb2 may stop
  wrapping once the EEPROM step sets 0/4095 (step 9 below records it). Either way nothing unsafe
  follows: the radians wrap the difference to the zero (continuous through both behaviours), a present
  above 4095 or a path ending outside 0..4095 is refused by the aligner, a negative present (sign bit
  15) is a read error. Whether a GOAL above 4095 would be accepted is unknown and never commanded.
  After the step limb5's recorded span maps to 1825..5638 - 1691 = **134..3947** (planned from the
  calibration travel: 143..3953; the recording reached about 9 ticks further at both ends - inside the
  142-tick spare).

## Virtual wall at the follower's command edge (user decisions 2026-10-15/16, progressive wall and engage deadband 2026-10-20)
Robot finding 2026-10-15 15:17:04: limb2 led to its lower joint limit (-3.3427 rad) was commanded
exactly there, sagged one tick beyond, and the follower's measured-pose check (SR-17 rule 8, arm
safety envelope) ended leader mode. Since then the follower keeps every leader setpoint
`leader.limit_command_margin_ticks` whole ticks inside each joint limit (the gripper's too since 2026-11-13)
(`gripperx_arm/config/arm_safety_filter.yaml`, computed by `gripperx_arm.leader_band`), and the leader
shows that edge as a wall the operator can feel.

**Progressive wall 2026-10-20.** The 2026-10-16 design (engage at the present tick, goal ramp
inward, release on an inward overrun, re-engage once still) chattered on the robot: limb2's lower
wall, where gravity pulls outward, engaged 75 times in about 15 s - the servo overshot the goal
inward by 7-17 ticks, the overrun release cut torque, gravity pulled the joint out again. Waiting
for "still" let limb4 fall 0.074 rad beyond before the wall came back, and at small depths the wall
was not felt. The user decided: a spring - force growing with the depth beyond the wall, full force
at 0.1 rad, at most 50 %.

**Engage deadband 2026-10-20 (same day).** The first spring kept torque enabled with Torque_Limit 0
in a strip just inside the wall (to avoid toggling Torque_Enable). On the robot the leader "got stuck
in the wall several times and was only stiffly moved out of it": the STS3215 brakes its motor with
torque enabled and Torque_Limit 0. The chatter was gone (limb2 lower: 6 engagements in 5 min instead
of 75 in 15 s). The user decided: torque off as soon as the joint is back inside; to keep it from
toggling right at the edge, engage only once beyond by a small depth. The strip and its parameter
`wall_release_hysteresis_rad` are gone; **torque is never enabled with Torque_Limit 0**.

- **Where.** Per joint (the gripper included since 2026-11-13) the follower's command band, read from the follower's own files - joint
  limits (`gripperx_geometry`), tick calibration (`arm_calibration.yaml` beside the safety filter file,
  or its `safety_filter.calibration_file`) and the margin - never repeated in `leader_arm.yaml`. The
  band is logged at startup. Since 2026-11-13 the gripper is driven in leader mode and has its wall
  like the other joints (FR-17 amendment). Wall ticks with the shipped calibration: limb1 508/2944,
  limb2 109 (continuous 4205)/1942, limb3 2659/525, limb4 2432/274, limb5 274/3780 (gripper: logged
  at startup, not yet read on the leader).
- **When.** Armed while the enable request is on AND the follower reports `/arm/leader/state`
  `active...`, not during an alignment, not with read errors or an unconfirmed torque-off. Arming
  writes nothing; a joint already beyond at arming gets the spring from the next tick.
- **The spring.** Depth d = how far the reading is beyond the wall (within 1e-9 rad of it counts as
  0: the shipped band edges lie on leader ticks). A free joint with d > `wall_engage_depth_rad` gets,
  on its servo only: Goal_Position = the hold tick (below; written once, at engage), Torque_Limit
  (register 48,
  SRAM - the EEPROM is never written) = `wall_min_torque_limit` at the first tick beyond, rising
  linearly to `wall_torque_limit` at d >= `wall_full_force_depth_rad`, never above the servo's own
  Torque_Limit, never below 1; torque on. Each tick the Torque_Limit follows d. The spring is measured
  from the wall, not from the engage depth: a joint engaged 3 ticks beyond gets spring(3 ticks), and
  keeps its spring while held anywhere beyond, inside the deadband too. No push-back ramp, no "still"
  condition, no release on an inward overrun.
  - *The hold tick:* the first tick beyond the wall by more than one tick - 2 ticks beyond for the
    shipped edges (they lie on ticks; limb2 lower: wall tick 109, hold tick 111), 1..2 ticks for an
    edge between ticks - and never outward of the present reading at engage (with the shipped
    deadband the reading at engage is always further out, so this clamp only acts for a smaller
    deadband or a boundary retarget). *Why not the wall tick:* a servo stiff enough to hold within
    half a tick of its goal would bring the joint to rest reading the wall tick, i.e. depth 0 -
    released - and with gravity pulling outward it falls, engages again past the deadband and is
    pushed back: the rest chatter (in the test model about 1.4 cycles/s; with the hold tick 0 releases
    in 30 s, with and without +-1 tick reading noise). At the hold tick a resting joint reads 2 ticks
    beyond, and +-1 tick noise still reads beyond. *Why not further inward:* it would push the joint
    inside the wall when the hand lets go (the 2026-10-20 overshoot); a goal at the present tick gives
    no force. The Torque_Limit ramp stays measured from the wall.
  - *Why a floor:* without one the cap is near 0 at small depths and the wall is felt late (the user
    wants it perceptible soon). The floor is not hidden at the wall: the goal is the hold tick, so
    between the wall and the hold tick the position error points outward and the floor caps that
    detent force (below).
- **Inside: released at once.** d <= 0 (the reading at the wall tick or inward of it): torque off on
  that servo (confirmed by read-back), then its Torque_Limit restored. What is true about outward
  pull: as of a reading, torque is on only while the joint is beyond the wall. Between the wall and
  the hold tick (at most 2 ticks) the servo pulls OUTWARD toward the hold tick with at most the spring
  there (about the floor, ~110 with the shipped values) - a 2-tick detent the hand has to leave
  through; the next reading at the wall releases. Between two readings a joint that has crossed
  inward is pulled back toward the hold tick by at most the previous tick's Torque_Limit, for at
  most one tick. No goal is ever written outward of the present reading.
- **Engage deadband** (`wall_engage_depth_rad`, start value 2 ticks = 0.00307 rad, TO-VERIFY). Engage
  needs a reading beyond by more than it, release one at or inside the wall: two readings more than
  2 ticks apart, so a reading jittering by +-1 tick can never both engage and release, wherever it
  sits (around the wall, around the engage depth or in between). With the edges on ticks, a joint
  engages at the 3rd tick beyond. The same deadband re-arms an overload latch (below): back inside ->
  free, engaging again only past the deadband - no separate re-arm distance. Below 2 ticks (of the
  leader calibration's `ticks_per_rev`) the wall refuses to start (`unavailable: ...`).
- **What toggling still costs, and when it happens.** Each engage is 9 bus transactions (request and
  reply each): 6 reads (Torque_Enable, Operating_Mode, Present_Voltage, Min/Max_Position_Limit, the
  servo's own Torque_Limit) and 3 writes (Torque_Limit, goal, torque on). Each release is 3: torque-off
  write, read-back, Torque_Limit restore. Reading noise of +-1 tick does not toggle (above). Real
  motion across the edge does, once per crossing: a joint pushed back by the servo that overshoots
  inward past the wall tick is released, and engages again only when gravity or the hand takes it
  beyond the deadband (the 2026-10-20 log of the 2026-10-16 design replayed: one engage and one
  release per overshoot, 12 transactions plus the monitor). A joint at rest on its hold tick does
  not toggle (above), whatever the servo's stiffness against gravity in the model (TO-VERIFY on the
  robot).
- **Bus load.** A register is written only when its value changes: a joint resting at one depth costs
  no write. Per held joint and tick one Present_Load read; one held servo per tick is monitored
  (voltage, Torque_Enable, Torque_Limit as last written, status bits). Every 2 s while a joint is held the log reports the longest tick and
  tick interval (does the wall fit in 33 ms).
- **Engage pre-checks.** Torque off, Operating_Mode 0, Present_Voltage in the `align_voltage_*` band,
  no status bits, the present tick and the hold tick inside Min/Max_Position_Limit (no extra guard
  since 2026-10-20: torque is on, as of a reading, only beyond the wall tick), a Torque_Limit above
  0 - otherwise it stays limp (`outside: ... (not engaged)`) and the reason is logged. They run once per engage, not per tick. The servo's
  original Torque_Limit is restored after every torque-off; if that write fails it is kept and
  retried at the next release, and the next engage uses it rather than the register (which still
  holds the wall's own spring value, as low as the floor). An alignment (J) retries such a pending
  restore first and is refused while it still fails - the aligner would take that leftover for the
  servo's own limit.
- **Servo dead band.** The hold tick assumes the servo holds within about a tick of its goal. The
  STS3215's CW/CCW_Dead_Zone registers (26/27) are read (never written) and logged per servo at
  startup with the other diagnostics; their values on the leader are TO-VERIFY.
- **Overload release** (thermal protection, no position breakaway: mechanical stops sit close behind
  several walls - limb2 lower 18 ticks, limb3 lower 11 ticks, limb3 upper 0.089 rad, limb2 upper
  0.112 rad). A joint beyond the wall with Present_Load magnitude >= `wall_overload_load` that has not
  moved inward by `wall_overload_progress_rad` for `wall_overload_time_s` -> torque off, confirmed;
  limp until back inside the wall (d <= 0), then the spring re-arms; shown as `outside: <joint>
  <side> wall (overload release)`. Present_Load (register 60): magnitude bits 0-9, raw 0..1000, most
  likely per mille of the maximum PWM duty, which Torque_Limit caps - unit and that cap TO-VERIFY on
  the leader. Consequence: the release can fire only where the spring has reached
  `wall_overload_load` (with the shipped values from 0.05 rad beyond), so `wall_overload_load` must
  not exceed `wall_torque_limit` (validated). Every 2 s per held joint the log reports depth,
  Torque_Limit and load, to calibrate these values on the robot.
- **Known gaps (TO-VERIFY).** Behind limb2 lower (stop 0.028 rad beyond) and limb3 lower (0.017 rad)
  the spring never reaches `wall_overload_load`, so the overload release cannot fire there - the
  servo works against the hand at no more than spring(stop depth) (about 210 and 170 with the shipped
  values) until the hand yields or a status bit faults the wall. A pitch joint whose gravity load
  alone exceeds `wall_overload_load` at rest beyond the wall would be overload-released after
  `wall_overload_time_s` even with nobody holding it.
- **Everything off.** Space, J (`/arm/leader/align`), enable withdrawn, the follower leaving leader mode,
  a read error, a monitor fault (voltage, Torque_Enable, Torque_Limit, status bits of a held servo), a
  failed Present_Load read or register write, node shutdown: torque off on every leader servo, read
  back; unconfirmed ids are retried every tick and block alignment. After such a release the wall
  re-arms only when the follower enters leader mode afresh.
- **Streaming continues** while a wall holds; the follower clamps independently.
- State on `/arm/leader/wall_state` (transient_local, published on change): `disabled ...`,
  `unavailable: ...`, `idle ...`, `armed[; held: <joint> <side> wall (spring, tick N)][; outside:
  <joint> <side> wall (overload release|not engaged)]`, `released: <reason>; torque off confirmed ...`
  or `released: ... NOT confirmed ...` (a `wall fault` or `NOT confirmed` is an alarm on the web teleop
  page, shown under the keys next to the align line; `held:` and `outside:` are shown there as well).
- **limb2's lower wall (the robot case).** Its wall tick 109 lies 109 ticks outward of limb2's 0/4095
  wrap. Beyond the wall the joint reads 110..~127 (mechanical stop), on the same side as the wall tick;
  the only goal is the hold tick 111, computed from a reading beyond the wall, and torque is on (as
  of a reading) only beyond 109 - never across the wrap. A yank inward across the wrap within one
  tick reads as far inside and is released. Before the leader EEPROM step its position
  limits read 823/3199 and the wall was refused; since the step (limits 0..4095, limb2 still wraps) it
  engages. Were limb2 ever to read continuously past 4095, the present tick would lie outside 0..4095
  and the pre-check would keep it limp.
- **Second edge source (extension point, not fed yet).** `LeaderWall.set_boundary(edges, now,
  session, seq)` takes, per joint, an `Edge(side, rad, tolerance)` - meant for the follower's held
  (last approved) pose after it rejects a leader setpoint for ground/collision. The tolerance is free
  travel: the spring starts (depth 0) at `rad` moved outward by it, so the force is continuous
  there as well. One owner per servo, the band wins. A boundary that moves past a joint it holds
  releases it (its new goal would pull outward). A seq not newer than the last of the same session is
  ignored; the boundary is dropped after `boundary_timeout_s` (WallConfig, 0.5 s, TO-VERIFY) without
  an update, and dropping or `clear_boundary()` releases only the joints it holds. Arming clears any
  boundary set before it. A boundary overload latch is kept when the joint crosses the band edge (the
  band takes it over, still limp) - no fresh push. The follower side and the subscription do not
  exist yet.

`wall_enabled: false` switches the wall off. It puts torque on the leader: every first run is a test
under the per-test motion approval (SR-1). All wall values are TO-VERIFY: `wall_torque_limit` (500),
`wall_full_force_depth_rad` (0.1) and `wall_overload_time_s` (2.0) are the user's test values
(2026-10-20 / 2026-10-16); `wall_engage_depth_rad` (0.00307, 2 ticks) is the user's start value
(2026-10-20); `wall_min_torque_limit` (100, now 1..`wall_torque_limit`), `wall_overload_load` (300) and
`wall_overload_progress_rad` (0.01) are implementation start values. Removed with the progressive
wall: `wall_pushback_velocity_rad_s`, `wall_max_lead_rad`, `wall_rest_tolerance_rad`,
`wall_follow_release_rad` (no ramp, no lead); with the engage deadband: `wall_release_hysteresis_rad`
(no Torque_Limit-0 strip).

## Leader EEPROM step (prepared 2026-10-13, executed 2026-10-15)
**Executed 2026-10-15** (on the real leader; facts as reported on the day):
- First run (`--write --save-previous ~/leader_eeprom_before_wrap_shift.yaml`) stopped after id 1 min
  (760 -> 0): the next EEPROM write's acknowledgement was lost and the following read-back died with
  an SDK `IndexError` (short reply). Relock was confirmed.
- The fixed tool (branch Theo-eeprom-writefix, f6f5646: settle and read-back after every EEPROM
  write, short replies as errors, re-run keeps the original snapshot) completed the rest:
  read-back OK; after a power cycle the dry run planned no writes (persistence OK).
- Calibration pair `zero_ticks[4]` 3718 -> 2027 / `homing_offset_raw[4]` 3325 -> 414: committed on
  branch Theo-leader-calshift (8588095), not on this branch.
- Wrap check (step 9): limb5 continuous 138..3946 (expected about 134..3947), no read errors. limb2
  still WRAPS at its bottom stop (2..4087, jump 4085) - so limb2's lower virtual wall engages (wall
  tick 109 on the wrapped side, see the wall section).
- Home cross-check (step 10): limb5 read 2059 against the computed 2027 (32 ticks). Attributed to
  hand placement - limb1 was -25 and limb2 +32 ticks off in the same pose, and an earlier home read
  gave 1987 - so the computed value is kept.

User, verbatim: *"Ja, bereite auch den Leader-EEPROM Schritt vor"* (prepare the leader EEPROM step).
Running it needs the user's explicit approval for that maintenance step. The tool never switches
torque (it refuses while torque is on), so writing moves nothing; the first alignment afterwards is a
motion test with its own per-test approval.

What `leader_eeprom` changes (computed from the registers and the calibration, never typed):
- `Min/Max_Position_Limit` of ids 1-5 -> 0/4095. Today's values (760/3232, 823/3199, 873/3076,
  865/3188, 0/4095) are copies of the follower's LeRobot values and say nothing about the leader.
- id 5 (limb5) only: `Homing_Offset` shifted by D so the 0/4095 wrap sits in the middle of limb5's
  unreachable gap. Present = Actual - Homing_Offset (LeRobot feetech.py), sign-magnitude with the
  sign in bit 11 (LeRobot STS_SMS_SERIES_ENCODINGS_TABLE). With today's values: gap 286 ticks between
  recorded max +2.9544 and min -2.8900 rad, **D = 1691**; offset raw 3325 (= -1277) -> **414**
  (raw 414); present 3631 -> expected **1940**; travel in ticks 143..3953 (142 ticks spare each side);
  **zero_ticks[4] 3718 -> 2027**. `range_center_rad` and `travel_min/max_rad` do NOT change: both
  ticks and zero shift by D, radians are invariant. The other ids' offsets are not touched.
- Planned writes for today's registers: id1 min 760->0, max 3232->4095; id2 823->0, 3199->4095;
  id3 873->0, 3076->4095; id4 865->0, 3188->4095; id5 Homing_Offset 3325->414.
- Id 6 (gripper), added 2026-11-13: its `Min/Max_Position_Limit` read 1293/2810 (follower LeRobot
  copies; the recorded leader travel reaches 3028), so a dry run now also plans id6 min 1293->0,
  max 2810->4095. Its Homing_Offset (4055) is only compared with the calibration, never written.
  **Executed 2026-11-16** (facts as reported on the day): `leader_eeprom --write` set id 6
  `Min/Max_Position_Limit` 1293/2810 -> 0/4095, read-back OK, the re-run dry run printed
  `planned writes: none`; snapshot `~/leader_eeprom_before_id6_limits.yaml`. Persistence over a
  power cycle NOT yet proven (the dry run after one must again print `planned writes: none`).
- Remaining limit after the step: limb2's recorded travel also crosses its wrap (ticks 1869..4223);
  its offset is deliberately not changed, so limb2 targets below about -3.16 rad stay refused.

Procedure (on the laptop, leader on USB, 7.4 V supply on):
1. Stop the leader node (Ctrl+C in its terminal), then check that nothing holds the port - this must
   print nothing:
   `pgrep -af "leader_arm_node|record_limits"`
2. Build and source:
   `cd ~/gripperx_ws/Software/ros2 && source /opt/ros/jazzy/setup.bash && colcon build --packages-select gripperx_leader && source install/setup.bash`
3. Dry run (reads only; compare the printed planned writes with the list above, stop if they differ):
   `ros2 run gripperx_leader leader_eeprom --port /dev/gripperx_leader`
4. Do not touch the leader from here to step 6. Write (approved maintenance step only):
   `ros2 run gripperx_leader leader_eeprom --port /dev/gripperx_leader --write --save-previous ~/leader_eeprom_before_wrap_shift.yaml`
   (the tool never overwrites an existing snapshot file: keep the first one)
   Every EEPROM / Lock write is followed by a settle time (`--settle-sec`, default 0.1 s, TO-VERIFY)
   and a read-back of that register, retried up to 3 times. **Interrupted run** (2026-10-15: id 1
   min written, the next packet lost): run the SAME command again. The plan is absolute, so it writes
   only what is left; the existing snapshot is kept and accepted as the snapshot of record when the
   registers differ from it only by values this plan writes (it says `KEPT as the snapshot of
   record`); any other difference is refused.
   It prints `read-back OK` (values, Lock = 1, torque 0, id 5 present within 3 ticks of the expected
   value) or the problems and the rollback command.
5. Read-back once more. It must print `planned writes: none` and a `NOTE` that the calibration still
   describes the old state (the plan is absolute against `homing_offset_raw`, so a second `--write`
   is a no-op; a Homing_Offset that matches neither the calibration nor the target is refused):
   `ros2 run gripperx_leader leader_eeprom --port /dev/gripperx_leader`
6. Power-cycle the leader: unplug its 7.4 V supply, wait 10 s, plug it back in. Persistence proof -
   the same dry run must again print `planned writes: none` (with the same NOTE):
   `ros2 run gripperx_leader leader_eeprom --port /dev/gripperx_leader`
7. Calibration: in `config/leader_calibration.yaml` change, TOGETHER, `zero_ticks` entry 5 from 3718
   to 2027 and `homing_offset_raw` entry 5 from 3325 to 414 (exactly the values in the NOTE) and add
   a dated comment; leave `range_center_rad` and `travel_min_rad`/`travel_max_rad` unchanged
   (invariant). Until both are changed, `leader_arm_node` refuses alignment and publishes
   `calibration_verified` False (it compares every servo's Homing_Offset with `homing_offset_raw` at
   startup and on every align request). Do NOT edit `zero_invariant_ticks`: it is the check value
   that makes a half-done edit visible - with only one of the two lines changed, both the tool and
   the node refuse and print the exact pair to write. Rebuild (step 2), then the dry run must print
   `planned writes: none` with no NOTE:
   `ros2 run gripperx_leader leader_eeprom --port /dev/gripperx_leader`
8. Restart the node:
   `export ROS_DOMAIN_ID=20 && ros2 run gripperx_leader leader_arm_node --ros-args -p port:=/dev/gripperx_leader -p servo_ids:="[1,2,3,4,5]" -p calibration_file:=$HOME/gripperx_ws/Software/ros2/src/gripperx_leader/config/leader_calibration.yaml`
9. Wrap check. Leader node running, torque off: turn limb5 by hand slowly through its whole travel to
   both stops and back, then limb2 to its bottom stop and back, while watching
   `ros2 topic echo /arm/leader/ticks`
   limb5 (5th value) must change continuously and stay inside 0..4095 (expected about 134 and 3947 at
   the stops, measured 2026-10-13 before the step: 1825..5638); the node log must show no
   `leader read error` (a negative present is refused as one). If limb5 fails: stop, roll back (below).
   limb2 (2nd value): note whether it still wraps (4095 -> 0) at its bottom stop or now runs past
   4095 - it answers the open question above; either is handled, record it.
10. Cross-check limb5: hold the leader at the CAD reference pose (the arm home pose of the twin model)
   and read the ticks three times:
   `ros2 topic echo --once /arm/leader/ticks`
   `ros2 topic echo --once /arm/leader/ticks`
   `ros2 topic echo --once /arm/leader/ticks`
   limb5 (5th value) should read about 2027. The computed zero is exact (the shift is a register
   transform); a hand-aligned re-measurement carries the alignment error of the eye. So: keep the
   computed value unless the cross-check differs by more than about 15 ticks (1.3 deg) - then stop and
   find out why instead of overwriting it. `/arm/leader/joint_states` limb5 must show the same radians
   as before the step at the same pose.
11. The first alignment afterwards is a separate test needing its own per-test approval (SR-1).

Rollback (the snapshot file written in step 4):
`ros2 run gripperx_leader leader_eeprom --port /dev/gripperx_leader --write --restore ~/leader_eeprom_before_wrap_shift.yaml --save-previous ~/leader_eeprom_before_rollback.yaml`
then set `zero_ticks` entry 5 back to 3718 and `homing_offset_raw` entry 5 back to 3325, and rebuild.

## Gripper (servo ID 6) in leader mode, and running without it
Since 2026-11-13 (user decision, FR-17 amendment) the follower gripper MIRRORS the leader gripper
like limb1..limb5: same angle conversion, command band, activation match tolerance, virtual wall,
self-alignment, watchdog and safety-filter checks - no special gripper semantics. Its calibration
(`config/leader_calibration.yaml`, id 6) was taken 2026-11-13. Its direction is unconfirmed until the
first robot test - no RViz check before it (user decision 2026-11-16); the user reports a wrong
direction early in that test. `calibration_verified` stays one flag for the whole file.
Accepted 2026-11-16 (user): through the leader the follower gripper opens only to the leader's
travel end (about 1.60 rad) instead of its own (about 1.81 rad); during a leader re-plan detour it
keeps its held goal (the detour searches limb1..limb5 only). A block while CLOSING (jaws stopped by
an object) does not end leader mode; one while opening beyond `leader.command_tolerance_rad` does.
The gripper presses with the servo's own Torque_Limit like the limbs; `leader.gripper_torque_limit`
in `gripperx_arm/config/arm_safety_filter.yaml` is the hook for a measured lower limit (TO-VERIFY).

Without the gripper cable run with `-p servo_ids:="[1,2,3,4,5]"`. Only those IDs are read and
published (names limb1..limb5; the gripper is simply absent, never 0 or NaN). At startup all six IDs are pinged and logged as
present/absent and configured/not configured; a configured ID that does not answer refuses the
start with a message naming the fix. A configured ID that stops answering later is a read error as
before. Calibration files always hold all six entries (by servo id); the node picks the configured
ones. The follower then REFUSES leader mode ("leader stream lacks [...gripper...]"), as for any other
missing joint. `record_limits` writes `null` plus a TO-VERIFY comment for
joints it never saw, never 0.

## Calibration
`config/leader_calibration.yaml` holds the leader zero ticks (hand-aligned to the CAD q=0 reference
image 2026-10-07, not a measurement; signs from a visual per-joint RViz comparison, zero hand-aligned; `calibration_verified: true`,
verified by user 2026-10-09; the node warns only while it is false). Radians use the tick difference wrapped into [-2048, 2048), so limb4/limb5 (zeros
close to the 0/4095 wrap) do not jump by 2 pi; this assumes each joint moves within +-180 deg of its
zero. A null zero is fine unless that ID is in `servo_ids`, then the node refuses to start (all six
are filled since 2026-11-13; `calibration_verified` is one flag for the whole file, set by the user
2026-10-09 for ids 1-5).

Each joint's output window is [center-pi, center+pi) with `range_center_rad` (TO-VERIFY, default 0 =
placeholder): a joint travelling lopsided around the CAD zero (e.g. -192..+58 deg) would otherwise
flip +-180. Take the centre from `record_limits` (`suggested_range_center_rad`).

## Record limits and poses
```
ros2 run gripperx_leader record_limits [--calibration config/leader_calibration.yaml] ~/leader_limits.yaml
```
Move the leader through its full range; `n` = name and snapshot a pose, `w` = write, `q` = write
and quit. Output keys: `joint_limits.min/max` (leader ticks), `follower_joint_limits` (from
`/arm/joint_states`, converted to ticks, if the follower is up), `poses.<name>`, all in
limb1..limb5, gripper order. It refuses to write into a package config directory; copy poses
into `arm_poses.yaml` by hand after review (joint limits go to `gripperx_geometry/config/geometry.yaml`
only - SR-17 rule 3, single source; `arm_poses.yaml` has no `joint_limits` any more). With `--calibration`, the leader range is unwrapped by sample-to-sample continuity from `/arm/leader/joint_states` (the first sample defines the branch, so starting away from home is fine; needs the leader node run with `calibration_file`): `leader_range` gives exact continuous min/max in radians (may exceed +-180 deg), the suggested `range_center_rad` and the unreachable gap (360 deg - travel), with FLAG comments for steps > 90 deg and gaps < 30 deg (both TO-VERIFY thresholds); `joint_limits` ticks are derived from it. Without it, min/max are wrap-aware (relative to the calibration zero if `--calibration` is given, else the first sample): a range crossing 0/4095 is written with min > max and a NOTE comment.

## Record leader -> follower (record_follow, added 2026-10-16)
For the staged follower speed increase (`leader.max_joint_velocity_rad_s` in
`gripperx_arm/config/arm_safety_filter.yaml`): what limits the follow speed. Passive:
subscribes only, publishes nothing.
```bash
source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
ROS_DOMAIN_ID=20 ros2 run gripperx_leader record_follow ~/follow_$(date +%Y%m%d_%H%M%S).csv --duration 120
ros2 run gripperx_leader analyze_follow ~/follow_<stamp>.csv     # again, later
```
Recorded: `/arm/leader/joint_states` (leader), `/joint_states` (follower), and
`/arm/leader/follow_debug` (`gripperx_arm_msgs/LeaderFollowDebug`, published by arm_action_server
once per leader tick while leader mode is active, `leader.follow_debug_enabled`): the setpoint after
the tick, the clamped leader target, the positions the tick read, tick duration and period, outcome,
time per tick phase.
Plus `/arm/leader/state` and `/arm/leader/wall_state`. CSV: one row per message (follow_debug: rows
`setpoint` with `tick_s`, `period_s`, the phase columns `*_s` and `check_poses`, outcome in `text`,
`target`, `measured`), receive time and header
stamp, limb1..limb5 in radians.

Report:
- **Tick:** duration and period mean / p95 / max against 1 / `leader.rate_hz` (the Pi tick time),
  outcome counts.
- **Tick phases** (added 2026-10-16; empty for an arm_action_server without them): mean / p95 / max
  per phase - gate checks (`refusal`), servo `read`, steering `envelope` and measured-pose check,
  `replan` (held arm), `target` (clamp, rate limit, edge slowdown), safety `check` with the number of
  poses it checked, servo `write`, `other` (the rest, e.g. the activation tick) - and the slowest
  ticks with their phases, longest first. The read and write phases include waiting for the serial
  lock (another bus user, e.g. the 2 Hz `/joint_states` timer).
- **Per joint:** setpoint max velocity and max step per tick against `max_joint_velocity_rad_s` /
  `rate_hz`; lag leader -> setpoint (transport and the rate limit) and setpoint -> measured (the
  servo; includes up to one tick, as the tick reads before it writes), and leader -> measured. A
  large leader -> setpoint lag with steps at the cap means the rate limit binds; a large
  setpoint -> measured lag means the servo does.
- Limits and rate are read from the installed `arm_safety_filter.yaml` (`--max-velocity`,
  `--rate-hz` override): the robot runs its own copy.
- `/joint_states` comes from arm_action_server's 2 Hz timer, not the tick: its lag and velocity
  columns are coarse and the report says so.

## Tests
`colcon test --packages-select gripperx_leader` (mock serial layer; node tests with DDS discovery
OFF, domains 221 and 224). `test/test_align.py` runs the real `LeaderBus` over a register-level mock; `test/test_wall.py` the
virtual wall on the same mock (domain 227); `test/test_follow_analysis.py` the recording analysis on
synthetic data.
