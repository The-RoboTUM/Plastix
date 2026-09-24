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
| Events | Mode changes, gripper/arm outcomes, gate arming replies, stops, link loss. |

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
