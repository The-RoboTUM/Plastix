# Octopus ↔ GripperX — interface proposal

**Status: partly settled, 2026-08-25.** The three items agreed in the exchange of 2026-08-20/21
are in *"Agreed — what the exchange settled"* below and take precedence over the sections they
supersede (§0 map origin, §7 axis convention, the rosbridge invocation in §5). **Everything else
is still a proposal for discussion and has not been changed on the Octopus side.**
From the GripperX team, 2026-08-18. Reviewed against PlastiX branch
`octopus-dashboard-cleanup` at commit `a7ab8e6278` ("Made raw gps topics beautiful").

Reference document: `Octopus/docs/octopus_to_robot_interface.md`.

> **Corrected 2026-08-20 — five statements about OUR OWN side were inaccurate and are fixed in
> place.** They are: the grasp offset and our Nav2 positioning accuracy (both in *"What GripperX
> does with a goal"*), the match tolerance (item 2a), the retry count (item 2b), and the telemetry
> sample payload (item 1). **The Nav2 figures were then revised a second time the same day**, after a
> navigation merge replaced the values — that bullet now names `0.10` m / `0.10` rad / `0.05` m in
> place of the `0.35` m / `0.40` rad it originally quoted. **No ask changed, no effort estimate changed, and nothing about your side
> changed.** Each correction is marked where it occurs. If you are holding a copy dated 2026-08-18,
> those five passages are the only differences.

## Summary

We are wiring GripperX (ROS 2 Jazzy) to consume `/octopus/trash_goal` as Nav2 navigation
goals. **We adopt your existing contract unchanged** — the four topics, the `NavSatFix`
choice, the shared-datum concept and the flat-earth conversion all work for us as they are,
and we deliberately implement the *same* approximation rather than a more accurate projection
so that the dashboard, the drone and the robot all speak one identical number.

The items below are what we need in addition, ordered by how much each one buys. Item 0 is a
correction; items 1–2 are the ones that make the loop actually close; the rest are smaller.

Effort estimates are ours and may be wrong — please correct them.

---

## Agreed — what the exchange of 2026-08-20/21 settled (folded in 2026-08-25)

The three items below are **settled** and take precedence over the corresponding proposal sections
further down, which are kept as the record of what was originally asked. They were agreed in an
exchange of letters between the two teams; the letters themselves are archived and are not part of
this repository, because their outcomes are here instead.

**Map origin (supersedes §0) — settled on the Octopus branch `item-a-map-origin`.**
`indoor_static_origin = 0,0` and a play-area bound `max_radius_m = 1.25`. **Both numbers are
load-bearing for GripperX, so if that branch moves or is merged we need to be told.** Read at source
on 2026-08-21, branch `eve-octopus` at `2a2c2f2b` still carried the original
`indoor_static_origin 2.23 / 1.67`, i.e. roughly 2.8 m of constant offset, and no play-area bound —
testing against it tests a known-broken configuration. The play-area bound was **Octopus's own
finding, not ours**: with the offset removed the camera footprint still reaches ±2.23 × ±1.67 m, so
up to 0.98 m in x stays outside GripperX's reach, and 5 of 7 open targets sat 2.3–3.3 m from the
datum. Without the bound the first joint run would have deadlocked on the second target — and we
would have blamed our own housekeeping. GripperX still clears the whole camera field for the first
run, belt and braces.

**Axis convention (supersedes §7) — option B, drone-referenced.** We had asked for A (geographic)
because it costs us nothing; the objection against it is the better argument and is accepted:
"north" here is a magnetometer inside a hall full of metal, and a blind grasp sequence must not
depend on it. The offered `indoor_static_align_yaw_on_start:=false` is **not** wanted.

One correction that changes what the transform-status topic is *for*: the rotation published on
`/octopus/flight_camera_transform/status` is **not** the rotation we were missing. Our map frame is
anchored on the robot's own start pose, so a *their-map → our-map* rotation was always required —
under option A no less than under B — and neither option supplies it; it comes from where the robot
is placed relative to the drone, and it is established by the disarmed verification run. What B
genuinely changes is that rotation's **lifetime**: the Octopus frame re-locks whenever the transform
node restarts, and `indoor_static_yaw_zero_rad` is the only thing that tells us it happened. **The
topic is an invalidation signal, not a calibration.** We read `state == "ready"` and a non-null
`yaw_zero` before trusting anything derived from it. It is a fifth ingress topic on our side; the
existing `/octopus/*` glob already covers it, so nothing changes on the Octopus side.

**rosbridge invocation (supersedes the invocation in §5) — three corrections from the Octopus side,
accepted in full and applied at source.** `OCTOPUS_ROSBRIDGE_SETUP.md` now carries the `ros2 run`
form, the quoted globs and `--actions_glob "[]"`, plus a note that `params_glob` does not exist on
rosbridge 2.0.7. Keeping `actions_glob` at `[]` costs GripperX nothing, and that is **checked rather
than asserted**: our client emits only `subscribe` / `unsubscribe` / `advertise` / `unadvertise` /
`publish`, accepts only `publish` and `status` inbound, and the whole `gripperx_external` package
contains no `send_action_goal`, `cancel_action_goal`, `call_service` or `advertise_service` call. If
either side ever needs an action across the link, it should be a diff.

### Still open after that exchange

- **The telemetry payload shape (§1) — decided by neither side.** We publish `device_id`, `stamp`,
  and flat `nav_state` / `active_goal_id` / `link_ok`; the dashboard reads `robot_id`, `timestamp`,
  nested `nav.*` and `link.*`, and `nav.distance_remaining_m`, **which we do not publish at all**.
  The single highest-value defect: the dashboard's status function returns on
  `pose.status !== "ok"` *before* it looks at `nav.status`, so our healthy value `available`
  permanently masks the nav state — a robot navigating correctly reports "pose available" and never
  shows `navigating` or `idle`. Our shape differs deliberately: we carry **separate** statuses for
  the map pose and for the lat/lon, because a lat/lon derived from a known map pose without a datum
  is a fabricated position. That reasoning survives a rename, so **we lean to adapting our payload**
  rather than asking the dashboard to move — but the decision has not been taken.
- **`max_target_list_age_sec` stays at 5.0 s.** The 1.001 s worst gap that would justify tightening
  it was measured on loopback, host to itself, on a quiet graph, for 30 s. The number this parameter
  needs is measured over the real path from the robot across Wi-Fi under a full pipeline. That
  measurement is still owed.
- **Whether GripperX can reach the Octopus host at all is unproven.** Both hosts sit in
  `10.42.0.0/24`, which is NetworkManager's default range for a *shared* connection — our Pi is
  `10.42.0.71` on one laptop's subnet, the Octopus host `10.42.0.158` on another. Same range, two
  different networks; the matching prefix is not evidence. `check_rosbridge.py` has never been run
  from the robot, and it needs both the network and an operating approval.
- **`relative_mode` should be removed rather than left standing** — agreed on both sides, and not
  for this run. A debug path that re-anchors on whatever the detector saw first would silently break
  any compensation either side applied.

---

## 0. Implement the agreed map origin (highest priority)

**Decision already taken in the team on 2026-08-17:** map (0,0) lies directly under the
drone, and GPS is computed from there. That is a good decision — it makes one physical point
serve as drone position, map origin, datum and robot start position simultaneously, and it
matches what `trash_gps_goal_node` already asserts ("Map (0,0) entspricht per Konstruktion
exakt dem Datum").

**The branch does not implement it yet.** In `Octopus/scripts/start_octopus_debug_stack.sh`
there are currently three different origin values in one pipeline:

| Node | Parameter | Current | Per the decision |
|---|---|---|---|
| `flight_camera_transform_node` | `indoor_static_origin_x/y` | `2.23 / 1.67` | `0.0 / 0.0` |
| `grid_map_builder_node` | `origin_x/y` | `0.0 / 0.0` | `−2.23 / −1.67` |
| `world_posearray_to_json_bridge_node` | `relative_origin_x/y` | `2.5 / 1.5` | unclear to us — please clarify what this offsets |

2.23 and 1.67 are exactly half of the `4.46 × 3.34` footprint, i.e. the drone currently sits
at the *centre* of the footprint while map (0,0) is a corner. Until these agree, every
coordinate we receive carries a constant offset of roughly 2.8 m. On a 4.5 m field that is
most of the working area, so nothing downstream can be validated before this holds.

*Effort: a parameter change, plus one check that `relative_origin` is consistent.*

## 1. A robot telemetry topic

**Proposed:** `/octopus/devices/gripperx/status`, `std_msgs/String` carrying JSON, 1 Hz.

This is the single change that buys the most, because it fixes something on *your* side:
`goal_selection: nearest` currently means "nearest to the datum", not "nearest to the robot",
for the simple reason that Octopus has no idea where the robot is. With this topic it can
mean what it says.

The namespace follows your own `/octopus/devices/{id}/...` convention from
`Octopus_ROS2/bridge_node.py`, and your dashboard already has the slot: `live_data.js`
carries a `gripperx` fleet entry (alias `robot_2`) with a `battery` field and a fallback
pose, waiting for a source.

```json
{
  "source_id": "gripperx_external",
  "robot_id": "gripperx",
  "timestamp": 1786977824.861,
  "pose": { "status": "ok", "frame_id": "map",
            "x": 1.02, "y": -0.34, "yaw_deg": 87.5,
            "lat": 46.6946641, "lon": 11.8404939 },
  "nav":  { "status": "navigating", "active_goal_id": 1,
            "distance_remaining_m": 1.42 },
  "armed": true,
  "battery": { "status": "unavailable", "reason": "NO_SENSOR_INSTALLED",
               "percent": null, "voltage_v": null },
  "link": { "connected": true, "last_rx_age_sec": 0.4 }
}
```

*Corrected 2026-08-20, and **sharpened 2026-08-24**: the JSON above is **illustrative, not the
schema**, and the 20-08 note was too gentle about how far it diverges. It said a consumer built from
this sample would "under-model" the payload. **It would mis-model it**: checked field by field
against `build_device_status()` — the one function that serialises this topic — most of the top-level
key names above are simply not the ones we emit. We send `device_id` (not `robot_id`), `stamp` (not
`timestamp`), `nav_state` and `active_goal_id` **at top level** (not nested under `nav`), `link_ok`
at top level plus `link.last_message_age_sec` (not `link.connected` / `link.last_rx_age_sec`), and
`pose.status` takes the values `available` / `unavailable` — **never `ok`**. `source_id`,
`pose.frame_id`, `battery.voltage_v` and `nav.distance_remaining_m` are **not emitted at all**. The
real payload additionally carries `pose.latlon_status` / `latlon_reason`, `pose.speed_mps`,
`nav_state_reason`, `arming_seconds_remaining`, `last_disarm_trigger`, `teleop_mode`,
`link.reconnects`, `counters`, `blacklist` and `octopus_transform`. **`build_device_status()` is the
schema of record — take it from us, not from this example.** Which side adapts the field names is an
open decision between the two teams and is not settled by this note.*

Two notes on the payload:

- Pose is given **both** as map metres and as lat/lon, computed with your formula against the
  current datum, so you can use whichever is convenient without a conversion.
- `battery` is structurally honest. GripperX has **no battery sensor at all** — no shunt, no
  divider, no free ADC channel on the drive ESP32. Rather than send a plausible-looking
  number we send `status: "unavailable"` with a reason. The field is in the schema from day
  one so nothing has to change on your side when the hardware arrives. Please have the
  dashboard render "unknown" rather than 0 %.

Consuming this needs one small bridge node on your side, mirroring
`map_patch_backend_bridge_node`. *Effort: one node plus a dashboard field.*

## 2. The acknowledgement path: an `id` on the goal, and a failure channel

Two changes to the same subsystem — please consider them together, since the second is not
implementable without the first.

### 2a. `/octopus/trash_goal` carries no target id

This one is small and blocking. `trash_goal` is a bare `NavSatFix`, so the only place an `id`
exists is `/octopus/trash_gps`. But `trash_goal_done` is keyed on that `id` — which means a
robot has to *guess* which target the current goal refers to by matching its coordinates
against the target list within some tolerance. That correlation can be wrong whenever two
targets sit close together, and it is exactly the situation your `merge_radius_m: 0.25`
exists to handle, so near-coincident targets are expected rather than rare.

We have implemented the positional correlation as an interim measure. It is a workaround for a
missing field, and it can acknowledge the wrong object.

*Corrected 2026-08-20, and **corrected again 2026-08-24 because the 20-08 text overstated our own
side**.* The 20-08 version said the match tolerance was "now fixed at startup and cannot be changed
while the mission runs", and that "a separate absolute maximum match distance refuses a far match
regardless of what the tolerance says". **Both described a decision we had taken, not code we had
written, and neither is implemented.** Checked at source on 2026-08-24: `goal_match_tolerance_m` is
re-read at every correlation and is explicitly **not** in the gateway's startup-only set, and
`correlate()` takes exactly **one** radius — there is no absolute maximum anywhere in the package.
The value our build runs is `0.25` m. **So the correlation is exactly as constrained as it was on
2026-08-18: one runtime-changeable radius, with no upper bound.** The decision to make it
startup-only and to add an independent absolute maximum stands on our side and is owed; describing
it as done was our error and we would rather say so than let it sit in a document you may decide
against an ask on.

**Hardening it would not remove the failure, which is why the ask stands — and it is not yet
hardened, which makes the ask more urgent rather than less.** Widening a tolerance normally makes a
match *ambiguous*, and ambiguity is the safe direction because we can refuse it. But **with exactly
one entry in the target list, any tolerance yields a "unique" match at any distance** — a single
known target is a confident, wrong correlation, and no cap on the tolerance alone fixes that case.
An `id` on the goal removes the whole class; any constraint we add can only narrow the window it
fires through.

**Proposed:** publish the goal's `id` alongside the `NavSatFix` — either a companion
`/octopus/trash_goal_id` (`std_msgs/String`, latched, published in the same tick) or, if you
prefer one message, a small JSON goal topic mirroring the `trash_gps` style. Either costs a
line or two in `trash_gps_goal_node`, and it removes an entire class of "collected the wrong
piece of trash" failure.

### 2b. A failure channel

**Proposed:** `/octopus/trash_goal_failed`, `std_msgs/String` carrying
`{"id": 1, "reason": "unreachable", "detail": "..."}`.

Today the goal advances **only** on `trash_goal_done`. There is no way for the robot to say
"I cannot do this one". A single object that is unreachable, outside the drivable area, or
ungraspable therefore stalls the whole mission permanently — the same goal is republished for
ever and the robot has no legal way out.

Reasons we can already distinguish and would send: `unreachable` (no path),
`no_approach_found` (no collision-free standing position around the object),
`outside_area`, `grasp_failed`, `aborted`, `rejected`.

Suggested handling on your side: mark the target as failed rather than collected, exclude it
from `goal_selection`, keep it in `targets[]` with a `failed: true` flag so the dashboard can
still show it, and let an operator re-arm it. Until this exists we will locally blacklist a
target after a bounded number of attempts and simply not acknowledge it — which means
Octopus keeps offering it and the mission stops. *(Corrected 2026-08-20: previously "a configurable
number of attempts", which read as though the number were settled. **That the bound exists is fixed
on our side; the number is not** — it is deliberately left open until a run against your real link
gives us evidence for one, and the value our build currently runs is an implementation default, not
a specified figure.)* That is a workaround, not a solution.

*Effort: one subscriber plus a flag in the target dict.*

## 3. Stable target ids

`id` is currently stable only for the lifetime of `trash_gps_goal_node`; a restart renumbers
from 1 and drops every `collected` flag. If the node restarts mid-mission, we can be sent
back to already-collected trash under an id we believe we have already handled, and a
`trash_goal_done` we send for the new id 1 can silently mark the wrong object.

Cheapest fix: add a `session_id` (or `run_id`) to the `/octopus/trash_gps` JSON and to the
goal, so both sides can detect a restart and reset their bookkeeping deliberately instead of
by accident. A UUID per target would also work but is more churn.

*Effort: one field.*

## 4. Multi-robot assignment

Your own document states it: *"Kein Task-Management. Wer zwei Sammelroboter anschließt,
bekommt beiden dasselbe Ziel."* Since Robby is also a ground collector, this bites the moment
both robots are connected — both drive to the same piece of trash and the first
`trash_goal_done` marks it done for the other.

Minimal fix: an `assigned_to` field in the goal (robots ignore goals not addressed to them),
or per-robot goal topics `/octopus/devices/{id}/trash_goal`. Item 1 makes either of these far
more useful, since Octopus would then know where each robot actually is.

Not urgent for a single-robot demo — but worth deciding before the second robot connects,
because it changes the topic layout.

## 5. rosbridge on the Octopus host

The team decided on **rosbridge** as the transport between the two ROS versions. That is not
yet reflected in the branch, which still assumes shared DDS on `ROS_DOMAIN_ID=0`.

**Ready to run as-is.** Two placeholders are marked `<…>`; they are yours to fill and we have
deliberately **not guessed them**.

```bash
# 1. install (stock package, nothing built from source)
sudo apt install -y ros-humble-rosbridge-suite

# 2. run it
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=9090 address:=0.0.0.0 \
  topics_glob:="[/octopus/*]" services_glob:="[]" params_glob:="[]"

# 3. one firewall rule for the port
sudo ufw allow 9090/tcp        # or your equivalent
```

Optional, so it survives a reboot:

```ini
# /etc/systemd/system/octopus-rosbridge.service
[Unit]
Description=rosbridge for the GripperX link
After=network-online.target

[Service]
User=<YOUR_USER>
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && \
  source <YOUR_WORKSPACE>/install/setup.bash && \
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:=9090 address:=0.0.0.0 \
    topics_glob:="[/octopus/*]" services_glob:="[]" params_glob:="[]"'
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

**The one thing we need back from you is the address**, because we cannot guess it: our client
config carries the host as *to be verified* and its placeholder is a deliberately wrong loopback
default, so that a forgotten override **fails to connect rather than quietly talking to something
local**. Tell us the hostname or IP we should dial, and whether binding `0.0.0.0` is acceptable on
your machine or you would rather bind one interface.

**Why the three globs, and what they cost you — checked against our client, not asserted.** Our
implementation speaks exactly this and nothing else: `subscribe`, `unsubscribe`, `advertise`,
`unadvertise` and `publish` outbound; `publish` and `status` inbound. **No `rosapi`, no services, no
parameter access, no `png`/`fragment` compression.** So `services_glob:="[]"` — which disables
`rosapi`, meaning a client cannot enumerate or call anything — and `params_glob:="[]"` cost us
**nothing at all**; they remove capability we never use. `topics_glob:="[/octopus/*]"` covers the
traffic in **both** directions, including the telemetry topic of item 1
(`/octopus/devices/gripperx/status`), so one entry is the whole surface. **These globs are proposed,
not agreed** — if you would rather run rosbridge without them, that is a different proposal and we
should discuss it, because on our side the narrow surface is part of why the transport is acceptable
at all.

**Your four topics need no change**: rosbridge exposes whatever is already on your graph, so we
conform to the existing contract as-is.

**What we would do with it, so you know the size of what you are agreeing to.** The immediate next
step is **not** driving a robot. It is a **disarmed, motion-free run** whose only purpose is to
measure *your real timing over rosbridge* — how regularly the four topics actually arrive — because
one threshold on our side was chosen against a test fixture and has never seen your machine. **No
motion, no arm, no goal executed, nothing published back onto your graph beyond the telemetry topic
if you want it.** It needs rosbridge running and nothing else from you.

**What we do not know, said plainly:** whether rosbridge is installed on your host at all, and
whether anyone has run it there. Your branch still assumes plain DDS on `ROS_DOMAIN_ID=0`, so as far
as we can see **the rosbridge decision is not reflected in your repository yet**. If it turns out to
be installed and working already, item 5 costs you nothing but the address.

Why this rather than a shared DDS domain: a shared domain would put GripperX, the drone,
Robby and all dashboard tooling into one ROS graph, where any participant can publish on any
topic. Our safety requirements demand exactly one designated writer per motion topic, and we
cannot guarantee that if the graph is shared. It also removes the need for the same subnet,
matching domain ids and open UDP 7400-7600.

This is the only item that requires an installation on the Octopus machine, and it is a
stock ROS package — once installed it never needs editing again, because all schema logic
lives on our side.

*Effort: one apt package, one unit file, one firewall rule.*

## 6. Publish `yaw_deg`

The dashboard already sends Eve's heading to `POST /api/eve/fake_gps`, but
`eve_fake_gps_bridge_node` drops it before it reaches ROS — your own doc notes the value
"liegt bereit". Publishing it (as a separate topic, or as a second field alongside the datum)
is the cleanest way to resolve the axis question in item 7.

*Effort: a few lines in the existing bridge node.*

## 7. The axis convention — two options, please pick one

This is not a request for work. It is a **contradiction between two of your own documents**, and
only you can say which one is authoritative.

- `octopus_to_robot_interface.md` states `x = Ost, y = Nord` for the lat/lon math.
- `Octopus/README.md` defines map `+y` as the **drone/camera front direction at Octopus startup**,
  locked by `indoor_static_align_yaw_on_start: true` and a 90° `indoor_static_map_yaw_offset_rad`.

**These agree only if the drone happens to face north at startup.**

> *Please re-check both quotes yourselves.* They are our reading, taken on **2026-08-18** against
> commit `a7ab8e6278`. **We cannot re-verify them today** — the copy of your repository we hold does
> not contain either file, so we are working from notes rather than from your source. If either
> document has changed, or we misread it, that alone may settle the question.

**Option A — geographic.** Map axes are East/North as the interface document says. The lat/lon math
is already correct as written, and map `+y` really does point north.
*Your side:* the map frame must actually be north-aligned at startup — either the drone is placed
that way, or you rotate internally before publishing. If the drone is placed arbitrarily, this is
real work for you.
*Our side:* **nothing to build.** We already implement your formula literally.

**Option B — drone-referenced.** Map `+y` is the drone's front at startup, as the README says, and
the published lat/lon therefore carries a **fixed rotation away from north equal to the drone's
startup heading**.
*Your side:* tell us that heading as a number, and re-publish it whenever Octopus restarts, since it
is locked at startup and a restart can change it. **This is exactly what item 6 (`yaw_deg`) would
deliver** — under Option B item 6 stops being a convenience and becomes a dependency.
*Our side:* we apply one fixed rotation in the map ↔ lat/lon conversion. Small, and we can do it.

**Our preference, stated openly rather than built into the wording: we prefer Option A**, because it
costs us nothing and because it makes the convention a **constant** instead of a runtime value we
have to receive, trust and re-check after every restart of your stack. **That preference is about
our convenience, not about correctness** — Option B is entirely workable, and it may well be cheaper
on your side, which is a trade only you can weigh. If B is the honest description of what your system
does, say B; we would rather absorb a rotation than have the document assert an alignment the
hardware does not have.

**How we check it afterwards, so the decision stays settled.** A convention that is agreed but never
measured comes back open the first time an object is picked up on the wrong side. The verification
run below already does it: with an object placed at a known offset **purely in +x**, then **purely
in +y**, Option A predicts one mapping and Option B predicts the same mapping rotated by the drone's
startup heading — so the residual rotation is directly measurable, and a swapped axis shows up as
well. **Under Option B this is not a one-off:** it should be re-checked after any Octopus restart
that could have changed the startup heading, which is the ongoing cost of B and the main reason we
lean to A.

---

## Joint verification we would like to run first

Before anything drives, one measurement settles both geometry questions at once, and it
needs no motion on our side:

1. Place a physical object at a known map offset from the robot's start point.
2. Let Octopus detect it and publish `/octopus/trash_goal`.
3. We convert it back with your formula and compare against the expected map coordinate. Our
   gateway is disarmed during this, so the result is only drawn as a preview marker in RViz.
4. Repeat with the object offset purely in +x, then purely in +y. A swapped axis or a
   rotated frame shows up immediately; a pure origin error shows up as a constant offset.

## What we are *not* asking you to change

- The four existing topics, their names, types and QoS — we conform.
- `sensor_msgs/NavSatFix` over `geographic_msgs/GeoPoseStamped`. Your reasoning holds and we
  do not need `geographic_msgs` installed on your side.
- The flat-earth conversion. We implement the identical approximation on purpose; a more
  accurate projection would make the robot and the dashboard disagree, which is worse than
  being slightly wrong in the same way everywhere.
- JSON-in-`std_msgs/String`. It is your house style across the whole stack, and it removes
  every cross-distro type-compatibility question between Humble and Jazzy. We validate
  against a schema on our side.

## What GripperX does with a goal, for your information

`/octopus/trash_goal` is the position of the *trash*, not a robot pose. Our arm cannot be
aimed — `PickPlastic` is a fixed, blind sequence and there is no arm model in our URDF — so
the robot's standing position is the only aiming mechanism we have. We therefore compute a
standing pose offset from the object by a grasp offset — the point relative to the robot where the
fixed sequence actually closes — choose an approach direction that is collision-free and reachable,
drive there, pick, and only then send `trash_goal_done`.

> *Corrected 2026-08-20: this said "a **measured** grasp offset". It is not measured.* The value we
> use is **specified, and the bench measurement that would confirm it has not been run** — the
> procedure (place an object on a grid relative to the robot, run the pick sequence, record which
> positions succeed) is written up on our side but still owed. So the standoff we drive to is a
> design intent rather than a calibration. **This affects how reliably we grasp, not how we
> interpret your coordinates:** the geodesy, the datum handling and the goal conversion do not
> depend on it, and neither does anything we ask of you in this document.

Two consequences worth knowing on your side:

- **We acknowledge after a successful pick, not on arrival.** Acknowledging on arrival would
  mark trash as collected that we failed to grab.
- **We cannot promise a successful grasp from navigation accuracy alone.** Our navigation stack was
  **substantially corrected on 2026-08-20** — robot geometry, footprint and the goal tolerances — but
  **how accurately it actually positions is still not measured**, and our own Nav2 tuning item
  remains open. The figures in our live config are `xy_goal_tolerance: 0.10` m and
  `yaw_goal_tolerance: 0.10` rad (≈5.7°), with the planner's own `tolerance: 0.05` m. **These are
  what our stack is configured to *accept* as "arrived", not a measurement of what it achieves.**
  - **Read the position figure as 0.15 m, not 0.10 m.** The planner tolerance adds *directly* to the
    achievable error, because the robot is steered to the planner's relocated path endpoint rather
    than to the requested goal — so the worst case is the two summed. Quoting 0.10 m alone would
    flatter us.
  - **0.15 m is a *declared* tolerance — what our stack accepts as "arrived", not how tightly it
    parks.** How tightly the real machine actually parks **has not been measured**. We have figures
    only from simulation, and we are deliberately not quoting them to you: the simulator has **no
    firmware** — no deadband, no break-away threshold, no hysteresis — so the real robot cannot make
    the small creeping corrections a simulated one can. That is precisely the regime *below* the
    tolerance where "how tightly it parks" is decided, so **simulation is not evidence about the
    hardware there.** Treat 0.15 m as the number we can defend, and treat anything tighter as
    unproven until we measure it on the robot. **That measurement is ours and is not an ask on
    you.**
  - **At our ~0.36 m grasp standoff, heading error is the term that bites:** a 0.10 rad heading error
    displaces the gripper about 36 mm sideways. (At the previous 0.40 rad it was about 145 mm, which
    is why the tolerance was tightened.)
  - Your position covariance is likewise a documented estimate rather than a measurement.

  Reaching the standing pose is what we can guarantee; reliable grasping will likely need a
  fine-positioning step, which is separate work on our side.

  > *Revised 2026-08-20 (second correction, same day).* The 18-08 text quoted `0.35` m / `0.40` rad
  > **as the accuracy Nav2 achieves**. Both parts were wrong: they were acceptance tolerances rather
  > than achieved accuracy, and a navigation merge has since **replaced the numbers themselves**.
  > **The conclusion above is unchanged.** What moved is its basis — no longer *"our tolerances are
  > coarse"* but *"the tolerances are now tight, they sum, and the gripper's own capture window is
  > still unmeasured"*. **We cannot state a grasp success rate while we know our positioning band but
  > not the window it has to land in**; that window is the bench measurement noted above. Nothing else
  > in this document — no ask, no effort estimate — rests on any of these figures.
  >
  > *One defect disclosed because it changes a number you would otherwise read wrong, not as a status
  > report:* until 2026-08-20 our planner tolerance was `0.6` m, and the controller checks arrival
  > against the planner's path endpoint rather than the requested goal. A goal nearer than that
  > therefore reported **SUCCEEDED with the robot never having moved** — reproduced at 0.60 m and at
  > 0.30 m. **Our ~0.36 m grasp standoff sat inside that radius**, so every goal derived from
  > `/octopus/trash_goal` would have hit it. It is our stack's defect, ours alone, found and fixed on
  > our side; we mention it only because it is why the two tolerances above must be **added** rather
  > than read separately.

Finally: as your document notes, nothing on this path has been tested against a real robot or
a real Nav2 before. We expect surprises and will report what we find.
