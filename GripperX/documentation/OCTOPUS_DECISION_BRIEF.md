# Octopus ↔ GripperX — decision brief

**For the joint discussion. Nothing in here is agreed, and nothing has been changed on the
Octopus side.** From the GripperX team, 2026-08-20.

Companion to `documentation/OCTOPUS_INTERFACE_PROPOSAL.md` (2026-08-18), which is the long form
and stays the reference. This page exists so the room can decide item by item without reading it.
The proposal was reviewed against PlastiX branch `octopus-dashboard-cleanup` at `a7ab8e6278`;
reference document `Octopus/docs/octopus_to_robot_interface.md`.

**Effort figures are OUR estimates of YOUR work and may be wrong — please correct them.**
**No value in this document is proposed on your behalf.**

**Read the "if you decline" column as the real question.** For several items we already have an
interim workaround, so a "no" is survivable — but the workarounds have named costs, and the point
of the column is to make those costs visible instead of assumed. Where a "no" costs nothing we say
so.

---

## At a glance

| # | Ask | Ours to build | Interim exists? | Decision needed |
|---|---|---|---|---|
| **A** | Implement the map origin you already decided (2026-08-17) | nothing — it is your parameters | **no** | **yes, blocking** |
| **B** | An `id` on `/octopus/trash_goal` | drop the positional correlation | yes, and it can be **wrong** | **yes** |
| **C** | A failure channel `/octopus/trash_goal_failed` | send reasons instead of going silent | yes, and it **stalls your queue** | **yes** |
| D | Robot telemetry topic `/octopus/devices/gripperx/status` | we publish it either way | n/a | worth deciding now |
| E | A `session_id` / `run_id` on targets | drop a reconnect caveat | yes, but **not** session-scoped — see E | low urgency |
| F | Publish `yaw_deg` from `eve_fake_gps_bridge_node` | supplies the rotation Q1 may need | measurement instead | low — **unless Q1 = B, then required** |
| **Q1** | **Axis convention — choose Option A or Option B** | A: nothing · B: one fixed rotation | **no** | **theirs to answer, decide in the room** |
| **Q2** | **rosbridge on your host** — command is ready to run | nothing changes; unlocks the timing run | **no** | **theirs to answer, decide in the room** |

Items A, B, C are the three that matter most, and the ranking is by **what breaks if you say no**,
not by what each one buys. *(The 18-08 proposal ranks the telemetry topic as "the single change that
buys the most". That is still true as an upside — it is ranked lower here because declining it
breaks nothing that is not already broken.)*

---

## A — Implement the agreed map origin

**The ask.** Make the three origin parameters in `Octopus/scripts/start_octopus_debug_stack.sh`
agree with the decision the team already took on 2026-08-17 — map (0,0) directly under the drone.
This is you implementing your own decision; we are not proposing a different one.

**What it costs you.** *Our estimate:* a parameter change, plus one check that `relative_origin` is
consistent. The three values today are `indoor_static_origin_x/y = 2.23 / 1.67`,
`grid_map_builder origin_x/y = 0.0 / 0.0`, and `world_posearray_to_json_bridge relative_origin_x/y
= 2.5 / 1.5` — **the third is the one we cannot interpret and are asking you to explain**, not to
change to a number we picked.

**If you decline — or simply do not get to it.** Every coordinate we receive carries a constant
offset of roughly **2.8 m** (2.23 and 1.67 are exactly half the 4.46 × 3.34 footprint, i.e. the
drone sits at the footprint *centre* while map (0,0) is a corner). On a ~4.5 m field that is most of
the working area. **There is no workaround on our side and we are not proposing one:** we could
subtract a constant, but we would be hard-coding a number derived from a configuration you are
about to change, and every later disagreement would be invisible. Nothing downstream — not the
joint verification run, not a demo — can be validated until the three agree.

**What changes on our side.** *Agree:* nothing to build; the verification run below becomes
meaningful. *Decline:* we do not compensate, and we report the offset as a blocking finding.

> **If the answer is "come back when your own side is measured"** — a fair challenge, since we are
> telling them our grasp offset is specified rather than measured (see the corrections note at the
> end). **Concede it, but only as far as it still goes.** Our navigation stack was corrected on
> 2026-08-20 and its tolerances tightened, so the positioning band is now known and tight; what
> remains unmeasured is how accurately it actually *positions*, and — the part that is genuinely ours
> — **the gripper's own capture window, which is a bench measurement and not a Nav2 property.** There
> is no reason to hand over ground we no longer have to give. Three points, worth having ready rather
> than improvising:
> 1. The 2.8 m error is **systematic and larger than the entire working area** — it is not in the
>    same class as an unquantified accuracy, it is a constant that displaces every coordinate.
> 2. It is **invisible to every downstream check until it is fixed**; nothing we measure while it
>    stands can be trusted to mean what it says.
> 3. **This is the one ask where no measurement on our side can proceed until they act.** The joint
>    verification run cannot distinguish a swapped axis from a constant offset while a known constant
>    offset is still present — so "measure first, then talk" is, for item A specifically, circular.

## B — A target `id` on the goal

**The ask.** Publish the goal's `id` alongside the `NavSatFix` — either a companion
`/octopus/trash_goal_id` published in the same tick, or a small JSON goal topic in the `trash_gps`
style. Your choice of shape; we conform to either.

**What it costs you.** *Our estimate:* a line or two in `trash_gps_goal_node`.

**If you decline.** `/octopus/trash_goal` is a bare `NavSatFix` and carries no id; the id exists
only in `/octopus/trash_gps`, and `trash_goal_done` is keyed on it. So we must **match the goal to a
target by position**. Two things make that worse than it sounds:

- Your `merge_radius_m: 0.25` exists precisely because near-coincident targets are expected. Close
  pairs are the normal case, not the rare one.
- **With exactly one entry in the target list, any match tolerance produces a "unique" match at any
  distance.** A single known target plus a wide window is a confident, wrong correlation.

The consequence of a wrong match is a `trash_goal_done` for an object we did not collect. **The wire
format has no retraction** — there is no message that un-marks a target — so the error is
irreversible on your side and indistinguishable from a real collection in either team's logs.

**What changes on our side.** *Agree:* the correlation and the parameter behind it disappear; the
acknowledgement becomes exact. *Decline:* we keep the positional correlation, which exists and
works — **but it is not hardened, and a 2026-08-20 version of this line said it was.**

> **Corrected 2026-08-24, against our own source rather than our own decision record.** This item
> previously read *"It is hardened — the match tolerance is startup-only so it cannot be widened
> mid-mission, and an absolute maximum match distance refuses a far match whatever the tolerance
> says."* **Neither is implemented.** `goal_match_tolerance_m` is re-read at every correlation and is
> deliberately **not** in the gateway's startup-only set; `correlate()` takes exactly **one** radius
> and there is no absolute maximum anywhere in the package. The build runs `0.25` m. The decision to
> do both was taken on our side on 2026-08-20 and is **owed, not done** — it is an open finding in
> our own safety audit. **Read the "if you decline" column accordingly: the interim is one
> runtime-changeable radius with no upper bound, and the single-target case below defeats it
> outright.** This makes the ask stronger, not weaker.

## C — A failure channel

**The ask.** `/octopus/trash_goal_failed`, `std_msgs/String` carrying `{"id": …, "reason": …,
"detail": …}`. Reasons we can already distinguish and would send: `unreachable`,
`no_approach_found`, `outside_area`, `grasp_failed`, `aborted`, `rejected`. Requires B — a failure
report needs an id to name.

**What it costs you.** *Our estimate:* one subscriber plus a flag in the target dict. *Suggested*
handling, not prescribed: mark failed rather than collected, exclude from `goal_selection`, keep in
`targets[]` with a flag so the dashboard still shows it, and let an operator re-arm it.

**If you decline.** Your goal advances **only** on `trash_goal_done`. The protocol has no way for us
to say *"I could not do this"*, so a single object that is unreachable, outside the drivable area or
ungraspable **stalls the whole mission permanently** — the same goal is republished for ever and we
have no legal way out.

**What changes on our side — and this is the item where the interim matters most.** *Agree:* we
report the reason and your queue moves on. *Decline:* after a configured number of attempts we
**locally blacklist** the id, keep **not** acknowledging it, and surface it loudly in our telemetry
and `/diagnostics`. Note what that interim does and does not do: it stops *us* from looping, but
**you cannot see it** — the blacklist is local, your queue still holds the target as open, and your
mission still does not advance. It converts an infinite retry into a visible stall. That is
deliberate on our side: we never acknowledge what we did not collect, so a protocol that cannot
express failure breaks *visibly* rather than silently marking litter collected. The attempt count
itself is not settled on our side; it needs a real-link run.

---

## D — Robot telemetry topic

**The ask.** Accept `/octopus/devices/gripperx/status`, `std_msgs/String` carrying JSON at 1 Hz, in
your own `/octopus/devices/{id}/...` namespace. Payload shape is in the proposal.

**What it costs you.** *Our estimate:* one small bridge node mirroring `map_patch_backend_bridge_node`,
plus a dashboard field. Your `live_data.js` already carries a `gripperx` fleet entry (alias
`robot_2`) with a battery field and a fallback pose waiting for a source.

**If you decline.** Nothing on our side breaks — we publish it regardless; it simply goes unread.
What stays broken is on **your** side: `goal_selection: nearest` means "nearest to the datum", not
"nearest to the robot", because Octopus has no idea where the robot is. It cannot mean what it says
until something tells it.

**One request attached to it:** `battery` will report `{"status": "unavailable", "reason":
"NO_SENSOR_INSTALLED"}`. GripperX has no battery measurement chain at all — no shunt, no divider, no
free ADC channel. Please render that as "unknown" rather than 0 %. The field is in the schema from
day one so nothing changes on your side when hardware arrives.

*Note for anyone building a consumer from the proposal's sample payload: the sample is illustrative,
not the schema. The real payload also carries validation/rejection counters and the local blacklist.*

## E — Stable target ids across your restarts

**The ask.** A `session_id` (or `run_id`) in the `/octopus/trash_gps` JSON and on the goal, so both
sides can detect a restart deliberately. A UUID per target would also work but is more churn.

**What it costs you.** *Our estimate:* one field.

**If you decline.** `id` is stable only for the lifetime of `trash_gps_goal_node`; a restart
renumbers from 1 and drops every `collected` flag. Mid-mission we can be sent back to
already-collected trash under an id we believe we handled, and a `trash_goal_done` for the new id 1
can mark the wrong object.

**What changes on our side.** *Agree:* we key our bookkeeping on the session, and the inference
below becomes a fact on the wire.

> **Corrected 2026-08-24.** This said *"our blacklist is already treated as valid only within one
> link session"*. **It is not, and deliberately so.** Our blacklist **survives a reconnect**. It used
> to be dropped whenever the reconnect counter moved — but that counter is the number of successful
> WebSocket connections, so a Wi-Fi flap produced it with your node untouched, your ids unchanged
> and your `collected` flags intact, and it then made a target we had proved unpickable retryable
> again. Our auditor reproduced that with one forced reconnect, and it is fixed.
>
> *Decline:* the blacklist is dropped **only on evidence that your id space itself restarted** — a
> `collected` flag we saw set and then saw cleared. When there is no such evidence it is kept, and we
> say so out loud rather than assume it quietly. That is an **inference** about your node's lifetime,
> drawn from a side effect. **A `session_id` on the wire is what would make it decidable instead of
> inferable**, which is this item's whole point. The residual we cannot cover today is a restart of
> *your* node that leaves no such trace while our link stays up.

## F — Publish `yaw_deg`

**The ask.** `eve_fake_gps_bridge_node` receives Eve's heading from the dashboard
(`POST /api/eve/fake_gps`) and drops it before it reaches ROS; your own document notes the value
"liegt bereit". Publish it, as a separate topic or as a field alongside the datum.

**What it costs you.** *Our estimate:* a few lines in the existing bridge node.

**If you decline.** Q1 below has to be settled by measurement instead — which we can do, and intend
to do anyway as a cross-check. This is a convenience, not a blocker.

---

## The two questions that are yours to answer

These are not asks for work. They are facts about your system that we cannot determine from outside,
and both gate our geometry.

**Q1 — The axis convention: two options, put both on the table and get one chosen.** The question
exists because **two of their own documents disagree**: `octopus_to_robot_interface.md` says
`x = Ost, y = Nord` for the lat/lon math, while `Octopus/README.md` defines map `+y` as the
drone/camera front at startup (`indoor_static_align_yaw_on_start: true`, 90°
`indoor_static_map_yaw_offset_rad`). **These agree only if the drone faces north at startup.**

| | **Option A — geographic** | **Option B — drone-referenced** |
|---|---|---|
| What it says | map `+y` really is north | map `+y` is the drone's startup heading; lat/lon carries a fixed rotation from north |
| **Their** cost | the map frame must actually be north-aligned — place the drone that way or rotate internally | tell us the startup heading as a number, and re-send it after every restart (**this makes item F a dependency, not a nicety**) |
| **Our** cost | **nothing to build** — we already implement their formula literally | one fixed rotation in the map ↔ lat/lon conversion, plus a value we must receive, trust and re-check |

**Say our preference out loud rather than steering to it: we prefer A**, because it costs us nothing
and makes the convention a **constant** rather than a runtime value. **That is our convenience, not
correctness.** B is entirely workable and may be cheaper for them. **If B is the honest description
of what their system does, we want B** — we would rather absorb a rotation than have a document
assert an alignment the hardware does not have.

**Two things to be straight about in the room:**

- **We cannot re-verify their two documents today.** Those quotes are our reading from 2026-08-18
  against commit `a7ab8e6278`; the copy of their repository we hold contains **neither file**, and
  none of the parameters named above appear in it. We are arguing from notes. **Ask them to confirm
  the quotes** — if a document changed or we misread it, that alone may settle the question.
- **What it actually changes on our side, corrected:** the geodesy (a fixed rotation) and the grasp
  *outcome* — a rotated frame puts the robot on the wrong side of the object, and our arm cannot be
  aimed, so the standing pose is the only aiming mechanism we have. It does **not** change
  `grasp.offset_y_m`: that is a robot-frame quantity and is unaffected by any map convention. Worth
  knowing before someone offers it as a knob.

**How it stays decided:** the verification run settles it empirically — an object placed purely in
+x, then purely in +y. A predicts one mapping; B predicts the same mapping rotated by the startup
heading, so the residual rotation is directly measurable. **Under B it is not a one-off** — it needs
re-checking after any restart that could change that heading, which is the ongoing cost of B and the
main reason we lean to A.

**Q2 — rosbridge on your host.** The team decided on rosbridge as the transport between the two ROS
versions. Your branch does not reflect that yet; it still assumes shared DDS on `ROS_DOMAIN_ID=0`.
Is the decision confirmed, and who installs it?

**The proposal now carries a copy-paste-ready block** — install line, launch line, firewall rule and
an optional systemd unit — so their answer can be "yes, run this" rather than "yes, we will scope
it". *Our estimate:* one apt package, one unit file, one firewall rule. **Their four topics need no
change** — rosbridge exposes what is already on their graph.

**Two named gaps we deliberately did not guess:** the **address** we should dial (our client config
carries it as to-be-verified, with a deliberately wrong loopback default so a forgotten override
fails loudly instead of talking to something local), and whether binding `0.0.0.0` is acceptable on
their machine. **And one thing we do not know:** whether rosbridge is installed there at all — their
branch still assumes plain DDS on domain 0, so the decision is not reflected in their repository.

**Ask for the small next step in the same breath, because it is the one thing that becomes possible
immediately.** With rosbridge up we would run a **disarmed, motion-free** session whose only purpose
is to measure *their real timing* over the link — one of our thresholds was chosen against a test
fixture and has never seen their machine. **No motion, no arm, no goal executed, no hardware, no
approval needed on our side.** It is worth saying explicitly that we are not asking to drive a robot
the same day.

*Why this is the ask to land: the grasp bench measurement has been deferred on our side, so the
timing run is now the only item on this track that could start the day after the meeting.*

> **Condition attached to Q2, proposed and not agreed:** the three globs are part of the ask, not
> decoration. `services_glob:="[]"` disables `rosapi`, so a client cannot enumerate or call anything
> and only `/octopus/*` topics are reachable. **They cost us nothing, and that is checked rather than
> asserted:** our client speaks only `subscribe`/`unsubscribe`/`advertise`/`unadvertise`/`publish`
> outbound and `publish`/`status` inbound — no `rosapi`, no services, no parameter access — and the
> single `topics_glob` entry covers both directions, telemetry included. If you would rather run rosbridge without
> those restrictions, that is a different proposal and we would want to discuss it — our side
> currently treats the narrow surface as part of why the transport is acceptable at all.

*Why rosbridge rather than a shared DDS domain, briefly:* a shared domain puts GripperX, the drone,
Robby and all dashboard tooling into one ROS graph where any participant can publish on any topic.
Our safety requirements demand exactly one designated writer per motion topic, and we cannot
guarantee that in a shared graph. It also removes the need for the same subnet, matching domain ids
and open UDP 7400-7600.

---

## What we are NOT asking you to change

Listed so the room does not negotiate against a phantom. Each of these is a deliberate adoption, not
a concession we are waiting to reopen:

- **The four existing topics** — names, types and QoS. We conform.
- **`sensor_msgs/NavSatFix`** over `geographic_msgs/GeoPoseStamped`. Your reasoning holds, and you do
  not need `geographic_msgs` installed.
- **The shared-datum concept and the flat-earth conversion.** We implement the *identical*
  approximation on purpose. A more accurate projection would make the robot and the dashboard
  disagree, which is worse than being slightly wrong in the same way everywhere — dashboard, drone
  and robot speak one identical number.
- **JSON-in-`std_msgs/String`.** It is your house style across the stack and it removes every
  cross-distro type question between Humble and Jazzy. We validate against a schema on our side.
- **Multi-goal push.** We track exactly one goal at a time because that is what your interface does.
  We are not asking for a queue. *(If you were to add multi-goal push, tell us — it reverses a
  decision on our side. It is cheap to reverse because it costs no wire-format change.)*

## What GripperX does with a goal, for your information

`/octopus/trash_goal` is the position of the *trash*, not a robot pose. Our arm cannot be aimed —
`PickPlastic` is a fixed, blind sequence and there is no arm model in our URDF — so the robot's
standing position is the only aiming mechanism we have. We compute a standing pose offset from the
object, choose an approach direction that is collision-free and reachable, drive there, pick, and
only then send `trash_goal_done`.

- **We acknowledge after a successful pick, never on arrival**, under any configuration.
  Acknowledging on arrival would mark trash collected that we failed to grab.
- **We cannot promise a successful grasp from navigation accuracy alone.** Reaching the standoff
  pose is what we can guarantee; reliable grasping will likely need a fine-positioning step, which
  is separate work on our side.

## Joint verification we would like to run first

One measurement settles both geometry questions at once and needs **no motion on our side**:

1. Place a physical object at a known map offset from the robot's start point.
2. Let Octopus detect it and publish `/octopus/trash_goal`.
3. We convert it back with your formula and compare against the expected map coordinate. Our gateway
   is disarmed; the result is only drawn as a preview marker in RViz.
4. Repeat with the object offset purely in +x, then purely in +y. A swapped axis or rotated frame
   shows up immediately; a pure origin error shows up as a constant offset.

**Sequencing:** this run is only meaningful **after A**, and it needs one measurement on our side
first (our demo-area extent, which our gateway requires before it will validate anything). Neither
is a reason to delay the discussion.

---

## Our own open item, not an ask: multi-robot assignment

Recorded here separately because it is **not** something we are asking you to decide in this
meeting, and it should not compete with A–C for the room's time.

Your own document states it: *"Kein Task-Management. Wer zwei Sammelroboter anschließt, bekommt
beiden dasselbe Ziel."* Nothing in the contract says which robot takes which target. Since Robby is
also a ground collector, this bites the moment both robots are connected: both drive to the same
piece of trash, and the first `trash_goal_done` marks it done for the other.

Candidate shapes, neither proposed: an `assigned_to` field in the goal (robots ignore goals not
addressed to them), or per-robot goal topics `/octopus/devices/{id}/trash_goal`. Item D makes either
far more useful, since Octopus would then know where each robot actually is.

**Not urgent for a single-robot demo — but it changes the topic layout, so it is worth deciding
before the second robot connects rather than after.**

---

## Corrections already applied to the proposal

**`OCTOPUS_INTERFACE_PROPOSAL.md` was corrected at source on 2026-08-20** — five statements
described **our own** side inaccurately and are fixed in the document itself, each marked where it
occurs: the **grasp offset** (specified, not measured), our **Nav2 positioning accuracy** (acceptance
tolerances, not achieved accuracy — revised again the same day, see below), the **match tolerance**
(then described as startup-only with a separate absolute maximum match distance — **that description
was itself wrong and was corrected again on 2026-08-24; see item B**), the **retry count** (the bound
is fixed on our side, the number is not), and the **telemetry sample payload** (illustrative, not the
schema — **also sharpened on 2026-08-24: the divergence is in the key names, not only in the depth**).
**No ask, no effort estimate and nothing about their side changed.**

> **2026-08-24, and it is the honest lesson of the batch.** Two of those five corrections described
> **decisions we had taken** as though they were **code we had written** — the match tolerance in
> full, the telemetry payload in part. Both were caught by re-reading the source instead of the
> decision record. The proposal now states the implemented position in both places. **Do not quote
> the 20-08 wording of either.**

Listed here so the record survives in both places; the proposal is current as corrected on
2026-08-24 and can be quoted as it stands.

**The Nav2 figures were revised a second time on 2026-08-20**, after a navigation merge replaced the
values. The proposal now names `xy_goal_tolerance: 0.10` m, `yaw_goal_tolerance: 0.10` rad (≈5.7°)
and the planner's `tolerance: 0.05` m, in place of the `0.35` m / `0.40` rad it quoted on 18-08.
**Three things about that, and the first is the one to get right out loud:**

1. **Our position figure is 0.15 m, not 0.10 m — the two tolerances SUM.** The controller checks
   arrival against the planner's path endpoint rather than against the requested goal, so the planner
   tolerance adds *directly* to the achievable error. **Anyone who takes 0.10 m into the meeting will
   misstate our own error budget out loud.** Quoting 0.10 alone would flatter us — the same failure
   mode as this morning's overclaims, only pointing the other way. (At our ~0.36 m standoff the
   heading term adds ~36 mm at 0.10 rad, against ~145 mm at the old 0.40 rad.)
   **Verified 2026-08-20** against `Theo` at `265184b`: `xy_goal_tolerance: 0.10` and
   `yaw_goal_tolerance: 0.10`. A report that our tolerance was `0.04` was **checked and withdrawn** —
   `0.04` is the RotateToGoal critic's window, which sits at plugin level only because the critic
   reads it there in nav2 1.3.12 and a critic-scoped key would be silently ignored. **0.10 + 0.05 =
   0.15 m stands.**

   **Say "declared", not "achieved" — this is the trap in this item.** 0.15 m is the tolerance our
   stack *accepts*; how tightly the machine actually parks inside it is measured behaviour, and the
   only such measurements we have (in the 2–5 cm range) are **from simulation**. **If anyone in the
   room has heard those numbers, they must not be repeated as robot performance.** The simulator has
   no firmware — no deadband, no break-away threshold, no hysteresis — so the simulated robot can
   creep to a stop where the real one cannot, and that is exactly the sub-tolerance regime where
   parking accuracy is decided. **"We arrive within 15 cm" is safe and defensible. "We typically park
   to 2–5 cm, measured" is sim-only and indefensible** — and it is the one a listener remembers,
   because it is the better number. The real measurement is outstanding on our side; it concerns our
   own grasp accuracy and is **not** an ask on them.
2. **"Tuned" is not the replacement word for "untuned".** Our own **NFR-5 is still `open` and OP-7 —
   whether Nav2 is in v1 scope at all — is still undecided**. The stack was corrected; its
   positioning accuracy is still unmeasured, and that is exactly how the proposal words it.
3. **We did not import the navigation track's odometry improvement figure**, because it measures
   odometry rather than positioning accuracy and belongs to their document. Anything we say about the
   merge's effect, we say in our own terms from our own sources.

*Practical note for the room: these values live in our local repository and are **not yet published**,
so nobody outside our team can look them up today. Quote them; do not offer a link.*

One consequence is worth carrying in: the corrected accuracy claim makes **item C stronger, not
weaker** — a stack whose positioning accuracy is unmeasured and a grasp offset that is unmeasured
make `grasp_failed` and `no_approach_found` **more** likely on the first real run, which is exactly
the hole the failure channel exists to fill.

Finally, as your own document notes: **nothing on this path has been tested against a real robot or
a real Nav2 before. We are the first integration, we expect surprises, and we will report what we
find.**
