# Browser teleop UI

A web page that drives GripperX, as an alternative input device for the teleop
that already exists. `keyboard_teleop_node` is unchanged and still works; this
adds `web_teleop_node` next to it.

    ros2 launch gripperx_teleop web_teleop.launch.py
    # then open http://localhost:8080/

**Run one or the other, never both.** They publish the same `cmd_vel` and would
race.

---

## What the page shows

| Area | What it is for |
|---|---|
| Key deck | Every key drawn as a keycap, lit while held. Colour says what kind of key it is *before* you press it: amber = dead-man (drive while held), blue = cumulative (steering stays where you left it), grey = one-shot action. |
| Emergency stop | The space bar, drawn as the thing it is. Clickable, and available to every open page — see below. |
| Robot view | Top view built from the node's own geometry. Dashed outline = the pose being **commanded**, solid wheel = the angle actually **measured** on `/hw/steer_states`. Watching the solid wheels swing into the outlines *is* the transition guard doing its job. The arrow in the middle is the commanded body twist. |
| Readouts | Guard state, manoeuvre, steering angle against its limit, the exact `cmd_vel` on the wire, worst-wheel alignment error, external gate arming, link health. |
| Camera | Tile with an On/Off and a View/Detail toggle, default OFF. Shows the front camera's JPEGs, fps, frame age and size; a frame older than `camera_stale_sec` dims the image and says STALE. See "Camera tile". |
| Events | Mode changes, gripper/arm outcomes, gate arming replies, stops, link loss. |

**Leader mode (J).** The J keycap (or the J key) goes through the same
`toggle_leader()` the terminal teleop uses. Since 2026-10-13 (user decision,
self-alignment) J with leader mode off sends an ALIGN request on
`/arm/leader/align`: the leader arm drives itself to the follower's pose, goes
limp, and the leader node then publishes `/arm/leader/enable` itself
(`gripperx_leader/README.md`). While that runs the keycap reads ALIGNING, the
banner says the leader is moving, and J or the space bar cancels. The last
`leader_align_state` text stands in a line under the action keys; `refused` and
`aborted` also take the top banner for 10 s (`leader_align_age_sec`, a display
choice; ranked below a latched emergency stop) and appear as events. The
transient_local replay of a state from before the node started is shown in the
line but is no fresh notice (no banner, info event) unless a J is pending. The event log resets its last-seen id when the
node's `events_epoch` changes, so a page left open across a node restart no
longer skips new events (robot finding 2026-10-15: a refused J showed nothing). J with leader
mode on switches it off. J, P, I and O are never re-sent after a failed request
(the banner says "Delivery unknown ... check the J / arm state"); each one-shot carries an id and its page-side
age, and the node drops duplicates and events older than `event_max_age_sec`.
Space is always re-sent. A silent page also ends leader mode / an alignment. The keycap and the "LEADER MODE: drive blocked"
banner follow `leader_enabled` in the telemetry, i.e. the node's real state as
echoed by the topic, not a page-side guess. While on, the drive keys are greyed
out and the node publishes only a zero twist. The space bar and the arm keys
(P, I, O) switch it off first, via the inherited `center()` / `disarm_leader()`.

**No leader process (2026-10-16).** A J that would send an align request first
checks that something subscribes to `/arm/leader/align`. If nothing does, J is not
sent and the align-state line and banner read `unavailable: leader process not
running - start it (gripperx_desk.sh starts it when the leader is plugged in)`. If
a request was sent but no `/arm/leader/align_state` answer arrives within
`leader_answer_timeout_sec`, the request is cancelled (`disarm_leader()`) and the
same text is shown. Before this, a J without a leader node went nowhere and only a
second J logged "leader alignment CANCELLED". The first check reads the DDS graph,
so in the first seconds after the leader node starts it can still say "not
running"; a second J then goes through.

Banners appear for the four states that are easy to misread and expensive to
miss: emergency stop latched, observer mode, "armed on timeout without steering
feedback", and "pose unreachable, will not arm".

---

## How it relates to the terminal teleop

`WebTeleopNode` **subclasses** `KeyboardTeleopNode`. It does not reimplement the
control path — `_publish`, `press`, `center`, `_held` and the `TransitionGuard`
are all inherited, unchanged. What the subclass adds is a different input
device and a display; what it overrides is `_announce` (no raw-tty escape
codes when there is no tty) and a handful of outcome callbacks that also
append to the event log.

Two behaviour-neutral hooks were added to `keyboard_teleop_node.py` to make
that possible without a second copy of the tick:

* `__init__(node_name=...)` — default unchanged, so every existing launch keeps
  the node name it had.
* `_observe(...)` — a no-op called at the end of each publish tick with the
  values just published. The UI renders **what actually went on the wire**
  rather than recomputing the tick for itself, which would be a second copy of
  a safety-relevant decision, free to drift.

`test/check_web_ui_assets.py` guards the three UI files against each other.

---

## Safety model

**The dead-man switch survives the network.** The page never sends "key
released" and hope. Every beat carries the **complete set of keys currently
held**, ~20/s, and the node refreshes the parent's key timestamps from it.
A closed lid, a killed tab, a dropped Wi-Fi link and a crashed browser all look
identical to the node: the set stops being refreshed. There is no message whose
*loss* can leave the robot driving.

Timings, with the defaults:

| Event | Robot stops after |
|---|---|
| You release the key | ~1 publish tick (20 ms) — the release is acted on, not waited out |
| Link drops / tab dies | `client_timeout_sec` (0.5 s) + 1 tick |
| Terminal teleop, for comparison | `drive_hold_sec` (0.6 s) after the last key repeat |

So the browser is never slower to stop than the terminal it replaces, and it
still rides through a sub-0.5 s hiccup instead of jolting the robot to a halt
on flaky Wi-Fi (NFR-8).

**One session drives, the rest watch.** Two tabs refreshing competing key sets
would fight over the dead-man. The first page to connect holds control; others
get the same live view read-only and a *Take control* button, which succeeds
once the holder has been silent for `takeover_sec` (2 s).

**Anyone may stop.** The emergency stop is deliberately *not* gated on holding
control. An observer's stop works exactly like the driver's.

**The emergency stop latches.** `center()` clears the held keys — but the
driving page would re-assert them 50 ms later and drive straight back out of
the stop. So after a stop, input stays ignored until the page reports an
*empty* key set, i.e. until the operator has physically let go. The page says
so in red until then.

**Two teleops at once is caught.** It is the one genuinely dangerous way to run
this system: both front-ends publish the same `cmd_vel`, the mux forwards
whichever arrived last, and neither operator's dead-man covers the other's
traffic — releasing every key on one page does *not* stop a robot the other one
is driving, and neither does its emergency stop. The node watches the graph
every 2 s for another `keyboard_teleop_node` or `web_teleop_node`, logs an
ERROR and puts a red banner above everything else on the page. It does not kill
the other node: that node might be the one an operator is holding a key on, and
killing it blind would be a worse failure than the one being prevented. The
terminal node has no equivalent check — this is a one-sided warning.

**The socket is on localhost by default.** `web_host:=0.0.0.0` puts a live
drive interface for this robot on the network, with no password: anyone who can
reach the port can drive it. The node logs a warning when you do it. Do it only
on a network you control.

**Shutdown needs Shift+Q**, not a bare Q. In a terminal a stray Q ends your own
session; here it would shut the teleop node down from across the room. The
click path asks for confirmation as well. Either way the robot is stopped and
the wheels straightened first, exactly as on the terminal node's exit.

**Started from `gripperx_desk.sh` (the GripperX Desktop launcher,
`Software/ros2/scripts/gripperx_desk.sh`), quitting here tears the WHOLE desk
session down** — RViz, the local mapping stack, everything that script
started — not just this page, exactly as pressing Enter/Ctrl-C in the
launcher's own terminal already does. The node signals it through a flag file
the launcher hands it in `GRIPPERX_DESK_QUIT_FLAG`, written only AFTER the
stop/straighten above has published, and only on this deliberate quit — never
on a SIGTERM or a crash. Run standalone (a bare `ros2 launch ...
web_teleop.launch.py`, no launcher in the loop), that variable is unset and
quitting only stops this node, as before — you then need a terminal to start
teleop again. The confirmation dialog names both cases, since the page itself
has no way to tell which one it is in.

---

## Camera tile

Added 2026-11-17 (desk only; not yet tried on the robot, TO-VERIFY).

    GET  /api/camera/stream    multipart/x-mixed-replace, the camera's JPEGs unchanged
    POST /api/camera/profile   {"profile": "view"|"detail"}

**What OFF guarantees.** The tile's On button opens the stream connection (an
`<img>` on `/api/camera/stream`); Off, a closed tab or a dropped link ends it. The
node holds its ROS subscription to `/camera/front/image_raw/compressed` (BEST_EFFORT,
depth 1) only while at least one such connection is open: first viewer creates it,
last one destroys it. With the tile off there is no subscriber, hence no camera DDS
traffic to the laptop, and the camera node (lazy capture, `gripperx_sensors/README.md`)
releases the USB stream within about a second, so the Pi spends no CPU and no WLAN
bandwidth on it. A quiet or dead connection is noticed by a peek on the socket every 0.5 s; a viewer that makes no progress for 5 s (`CAMERA_WRITE_TIMEOUT_SEC`: a frame cannot be written, or the socket stays unwritable while frames wait) is dropped and its subscription released, so a frozen tab or half-open link cannot hold the camera on. With no frames at all (camera dead) a silently dead peer is not noticed, but then nothing is being sent either. A dropped WLAN link without a TCP reset is only noticed that way (TO-VERIFY on the robot).

**Delivery.** The JPEG bytes are forwarded without decode or re-encode. Every viewer gets only the newest frame, and delay cannot build up in the kernel: the stream socket has a small send buffer (`CAMERA_SNDBUF`) and a frame is only started when the socket is writable now, otherwise it is skipped and the then-newest one is sent. The age shown in the tile (and the STALE mark, `camera_stale_sec`) is that of the frame the most-behind viewer was last handed and its TCP peer acknowledged (frame bytes no longer in the kernel send queue, `TIOCOUTQ`), not of what arrived from ROS or what `write()` accepted, so a viewer that is not being served reads STALE although the camera is live. What the browser has decoded and painted is not measured (TO-VERIFY); the client's own receive buffer is outside the server's control. The subscription callback only stores the frame, in its own callback group; the profile and parameter services are only called asynchronously, with a 3 s watchdog each, so the tile cannot delay teleop, E-stop or the control lease.

**Profiles.** View sets the camera node's `profile` parameter (320x240 @ 15 fps), Detail
640x480 @ 5 fps. The reply (set / refused / no answer / camera node not running)
appears in the tile and in Events. The current profile is read from the camera node
when the stream starts. If nothing publishes the image topic the tile says "no camera
node"; before the first frame "waiting for frames".

The page does not keep a profile list: a name is passed to the camera node, which refuses unknown ones (the list it reports, `profile_names`, is only shown). New node parameters: `camera_image_topic`, `camera_node` (default
`/camera/front/camera_front`), `camera_stale_sec` (1.0).

The camera stream and the profile route have no authentication of their own, like the drive controls: with `web_host` other than 127.0.0.1 anyone who can reach the port can watch the camera and switch its mode. The default bind is localhost.
Browsers cap parallel HTTP/1.1 connections per host (6); the telemetry stream and
the camera stream use two. Over WLAN use the View profile; Detail is for stopped
inspection (measured numbers in `gripperx_sensors/README.md`).

---

## Transport

No third-party packages and no hand-rolled WebSocket framing — a dead-man
switch is the wrong place for either. Two stdlib HTTP routes:

    POST /api/input       operator -> node   held-key set + one-shot events
    GET  /api/telemetry   node -> operator   Server-Sent Events

`POST /api/input` body:

```json
{"session": "ui-4f2a", "keys": ["w", "left"], "events": ["estop"],
 "claim": false, "force": false}
```

`keys` is the complete held set, not a delta. `events` are edges
(`estop`, `mode_keyboard`, `mode_autonomous`, `pick`, `open_gripper`, `home`,
`arm_gate`, `disarm_gate`, `quit`). The reply reports whether this session
holds control and who does.

---

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `web_host` | `127.0.0.1` | Bind address. See the warning above. |
| `web_port` | `8080` | TCP port. |
| `web_stream_hz` | `20.0` | Telemetry frame rate. |
| `client_timeout_sec` | `0.5` | Ride-through for a silent page; keys are released once it expires. |
| `refresh_rate_hz` | `50.0` | How often the held set is re-asserted into the parent's timestamps. |
| `leader_answer_timeout_sec` | `3.0` | How long a J waits for any `/arm/leader/align_state` answer before it is cancelled and shown as "leader process not running". A display choice. |
| `takeover_sec` | `2.0` | How long a holder may be silent before another session may take over. |
| `open_browser` | `false` | Open the page on start. |
| `active_mode_topic` | `/teleop/active_mode` | Mode shown in the header. |
| `arming_state_topic` | `/gripperx/external/arming_state` | External gate state. Optional — the node starts fine without `gripperx_external_msgs` built. |

Everything the terminal node declares (`crab_speed_m_s`, `spin_speed_rad_s`,
`use_steer_feedback`, `linear_vel_m_s`, …) is inherited and still applies.

---

## Working on the UI without a robot

    python3 test/web_ui_demo.py --port 8080

Serves the same assets over the same API, driven by a simulated chassis. The
manoeuvre selection, the pose computation and the `TransitionGuard` are
*imported from `manoeuvre.py`* — the same code the robot runs, because it
happens to be pure Python. Only the ROS publishers, the steering servos and the
arm/gate services are faked. The page cannot tell the difference and says
"bench mode" in its event log.

    python3 test/check_web_ui_assets.py

---

## Keeping up with the teleop underneath

The key bindings and the crab behaviour are not settled. Three stages were in
flight on 2026-08-24, each on its own branch, and each has a matching UI
increment on a branch that starts from *it* — so the stage's own tip stays an
ancestor and taking the UI increment is a fast-forward, never a merge that
fattens a single-topic branch:

| Teleop stage | UI increment | What the page had to change |
|---|---|---|
| `Theo-teleop-combined-steer` | `Theo-teleop-web-ui-steer` | A/D are momentary and spring back; the deck said "stays put" |
| `Theo-manoeuvre-native` | `Theo-teleop-web-ui-native` | spins moved to 0/9, arrows steer the crab, reachable-heading dial |
| `Theo-teleop-responsive` | `Theo-teleop-web-ui-keys` | `_key_t` became `KeyStateTracker`; releases now reported |

    git checkout Theo-manoeuvre-native
    git merge --ff-only Theo-teleop-web-ui-native

## A note on building

`colcon build` into an existing install space has been observed to **silently
skip an updated file** here: `build/lib/` had the new code and
`install/.../site-packages/` kept the old, because setuptools compares mtimes
and the install copy looked newer. It produced a node that imported fine and
then failed on an attribute that exists in the source. So after every rebuild:

    diff -q <install>/gripperx_teleop/keyboard_teleop_node.py \
            <src>/gripperx_teleop/gripperx_teleop/keyboard_teleop_node.py

and `rm -rf` the build and install bases if they differ.

## Known limits

* **Verified on the desk rig, not on the robot.** Against domain 42
  (`controller_manager` + `swerve_controller` + `hw_substitute`, no hardware):
  the node starts, serves the page, fills telemetry from the rig's live
  `/hw/steer_states`, and the chain browser → `/teleop/keyboard/cmd_vel` →
  `teleop_mux` → `/cmd_vel` carries the twist. The rig's controllers are
  *inactive*, so nothing past `/cmd_vel` was exercised. Nothing has been run
  against the real robot.
* **No screenshot review.** Headless Firefox is broken in this machine's snap
  install (the GL framebuffer fails and the WebDriver session dies on
  navigation), so the page has not been looked at by a rendering engine. The
  three files are cross-checked statically instead —
  `test/check_web_ui_assets.py`, which has caught two real bugs: the on-screen
  arrow keycaps were never wired (dead to a tablet, fine from the keyboard),
  and a page that referenced an element the markup did not define.
* **No authentication.** Control is a first-come lease, not a login. This is
  fine on localhost and is the reason localhost is the default.
