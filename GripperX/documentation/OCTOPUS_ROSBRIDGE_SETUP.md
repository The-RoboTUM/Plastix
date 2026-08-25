# rosbridge on the Octopus host — implementation instructions

**For the Octopus team. From the GripperX team, 2026-08-21.**

> **CORRECTED AT SOURCE 2026-08-21, after the Octopus team ran it.** Three things in the original
> step 3 did not do what this page said they did on rosbridge **2.0.7**, the version Humble ships:
> the launch-file form **kills the node**, `params_glob` **does not exist** and was silently
> ignored, and `actions_glob` — **absent from this page entirely** — defaults to *unrestricted*.
> The corrected command is below and the reasoning is in section 3. **The original text is not
> preserved here**; it was wrong, and a document that hands another team a broken command should
> not keep it for the record. The three corrections that replaced it, and why each was needed,
> are in the *"Agreed"* section of `OCTOPUS_INTERFACE_PROPOSAL.md`.

This is the long form of **Q2** in `OCTOPUS_DECISION_BRIEF.md` and of **item 5** in
`OCTOPUS_INTERFACE_PROPOSAL.md`. Those two state *why* rosbridge and *what it costs*. This page
states *how*, step by step, so the work can be done without reading either.

**Scope: this is the only installation the GripperX link asks for on your machine.** It is a stock
ROS package. Once it runs, it never needs editing again — every schema, validation and conversion
rule lives on our side.

**It changes nothing about your four topics.** rosbridge exposes what is already on your graph. No
message type changes, no topic renames, no node changes, no code on your branch.

---

## 0. What we do not know, and are therefore not guessing

Three things are yours and we have deliberately left them blank rather than invent them
(everything in this document that we could not verify is marked the same way):

| | | Where it lands |
|---|---|---|
| **The address we should dial** | hostname or IP of the Octopus host, as reachable from the robot's network | our client config, currently a deliberately **wrong** loopback default so a forgotten override fails loudly |
| **The bind interface** | whether `0.0.0.0` is acceptable on your machine, or you want one interface | step 2 below |
| **Your user / workspace paths** | for the systemd unit | step 4 below |

> **ANSWERED 2026-08-21, and this table is kept as the record of what was asked, not as an open
> list.** The address is `ws://10.42.0.158:9090` (host `ITQLM125`); `0.0.0.0` is bound and nothing
> objected. **Only the third row is still open**, and its consequence is in section 6: the systemd
> unit exists in your repo but is **not installed**, so the link does **not** come back by itself
> after a reboot. See section 8 for all four answers.

~~**We also do not know whether rosbridge is installed on your host at all.**~~ **Answered
2026-08-21: it is installed and running** — rosbridge **2.0.7**, **built from source**, on host
`ITQLM125`, bound `0.0.0.0:9090`, with steps 5a–5d passed. The sentence that used to stand here said
your branch still assumed shared DDS on `ROS_DOMAIN_ID=0` and that the transport decision was not
reflected in your repository. **That was true of the branch and false about the machine**, and it is
the reason sections 1–4 below now read as history rather than as instructions.

---

## 1. Prerequisites

- **ROS 2 Humble** on the host that runs the Octopus nodes.
- **The rosbridge process must be on the same ROS graph as your nodes** — same machine or same
  `ROS_DOMAIN_ID` and same DDS discovery. If your stack runs on `ROS_DOMAIN_ID=0`, so must this;
  a rosbridge on a different domain will connect happily and forward **nothing**, which looks
  identical to a network problem from our end. **This is the single most likely way for step 5 to
  produce a silent, confusing failure.**
- **The four topics actually publishing.** rosbridge forwards what exists; it does not create
  topics. Confirm with `ros2 topic list` before you start.
- One TCP port reachable from the robot's network. We assume **9090**, rosbridge's default —
  change it if it collides, and tell us the number.

---

## 2. Install

> **SUPERSEDED 2026-08-21 — you did not take this path, and the path you took is the one that
> matters.** You are running **rosbridge 2.0.7 built from source**, not the stock apt package. The
> block below is kept because it is still the right instruction for a *second* host, and because the
> version it installs is the same 2.0.7 whose behaviour section 3 is written against. **Nothing here
> is an ask on you any more.**

```bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-suite
```

Stock binary package on a fresh host; nothing is patched. **On your host it is a source build**, and
that is worth remembering if the version ever moves — every glob claim in section 3 is a claim about
**2.0.7 specifically**, and a source tree can move off it without an apt upgrade making it visible.

---

## 3. Run it — foreground first

**Do this in a terminal first, before the systemd unit.** The first run is the one that tells you
whether the globs and the domain are right, and you want to see its log.

```bash
source /opt/ros/humble/setup.bash
source <YOUR_WORKSPACE>/install/setup.bash     # if your nodes come from a workspace

ros2 run rosbridge_server rosbridge_websocket \
  --port 9090 --address 0.0.0.0 \
  --topics_glob "['/octopus/*']" \
  --services_glob "[]" \
  --actions_glob "[]"
```

**`ros2 run`, not `ros2 launch`, and the globs are QUOTED STRINGS.** On 2.0.7 the globs are
`STRING` parameters that rosbridge parses itself; `ros2 launch` coerces a bare bracket list to
`STRING_ARRAY` and the node dies at startup with
`InvalidParameterTypeException: Trying to set parameter 'topics_glob' … expecting type 'STRING'`.
The launch file re-types it on the way through, so bypassing it is part of the fix, not a
preference.

`address:=0.0.0.0` binds every interface. **If that is not acceptable on your machine, bind the one
interface the robot reaches you on and tell us which** — it changes nothing on our side except the
address we dial.

### The three globs are part of the ask, not decoration

They are **proposed, not agreed**. Here is exactly what they do and what they cost:

| Argument | Effect on 2.0.7 | Cost to us |
|---|---|---|
| `--topics_glob "['/octopus/*']"` | only `/octopus/…` topics are reachable — in **both** directions | **none.** Every topic in the contract, including the telemetry topic `/octopus/devices/gripperx/status`, is under that prefix |
| `--services_glob "[]"` | narrows services — but rosbridge **appends `/rosapi/*`** to any non-empty-list value, so `[]` becomes `["/rosapi/*"]`, **not** nothing | **none.** Our client calls no service |
| `--actions_glob "[]"` | **closes actions.** Unset does NOT mean "no actions" on this version — it means **any action server on the graph** | **none, and verified rather than asserted** — see below |
| ~~`params_glob`~~ | **DOES NOT EXIST on 2.0.7.** Removed; passing it is silently ignored — no warning, no error | — |

**What actually closes parameter access** is `services_glob` *plus not running the `rosapi` node*,
because parameter reads and writes travel through `/rosapi/get_param` and friends. The original
version of this page credited `params_glob` for that, which was wrong.

**`actions_glob` is the one that mattered, and it was missing from this page.** Actions are a
first-class rosbridge capability on 2.0.7, and an unset glob would accept `send_action_goal` for
anything advertised. Nothing was actually exposed on the Octopus graph — there is no action server
on it — but the property was resting on an empty graph rather than on configuration.

**That it costs us nothing is checked, not claimed.** Every `op` our client can emit was read out
of `rosbridge_client.py`: `subscribe`, `unsubscribe`, `advertise`, `unadvertise`, `publish`
outbound, and it accepts only `publish` and `status` inbound. A search of the whole
`gripperx_external` package for `send_action_goal`, `cancel_action_goal`, `call_service` and
`advertise_service` returns **nothing**. Closing actions and services removes capability we do not
have.

**Checked against our implementation, not asserted.** Our client speaks exactly `subscribe`,
`unsubscribe`, `advertise`, `unadvertise`, `publish` outbound, and accepts `publish` and `status`
inbound. No `rosapi`, no services, no parameters, no `png`/`fragment` compression. The three globs
remove capability we never use.

**ANSWERED 2026-08-21 by the Octopus team, empirically:** the single entry `/octopus/*` **does**
cover `/octopus/devices/gripperx/status`. They published to the nested topic through the link and
read it back with `ros2 topic echo`. `fnmatch` semantics hold, `*` spans `/`, **no second glob
entry is needed.** This paragraph used to ask them to check it; it is kept as an answered question
rather than deleted, because the check is still the right one to re-run if the glob is ever
changed.

If you would rather run rosbridge **without** the restrictions, that is a different proposal and we
should talk about it. On our side the narrow surface is part of why an external transport is
acceptable at all.

---

## 4. Firewall

```bash
sudo ufw allow 9090/tcp        # or your equivalent
```

Adjust to whatever your host actually uses. If you prefer to scope it to the robot's address rather
than open the port broadly, that is strictly better and needs nothing from us.

---

## 5. Verify it — on your side alone, without GripperX

**You can complete this entire check before we ever connect.** That is the point of doing it now
rather than in a joint session.

**5a — the process is up and listening**

```bash
ss -ltnp | grep 9090
```

**5b — it sees your topics.** In the rosbridge terminal you should see the client connect in the
next step; first just confirm the graph side:

```bash
ros2 topic list | grep octopus
ros2 topic hz /octopus/trash_gps
```

**5c — a client actually receives data.** This needs no ROS on the client side — only Python and
the `websockets` package. Run it **from another machine on the robot's network if you can**, since
that also tests the firewall rule; from the host itself it still proves the glob and the domain.

```python
# save as check_rosbridge.py, run: python3 check_rosbridge.py <HOST>
import asyncio, json, sys
from websockets.client import connect

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

async def main():
    async with connect(f"ws://{HOST}:9090", max_size=65536) as ws:
        topics = {"/octopus/fake_eve_gps_start": "sensor_msgs/NavSatFix",
                  "/octopus/trash_goal":           "sensor_msgs/NavSatFix",
                  "/octopus/trash_gps":            "std_msgs/String"}
        for topic, msg_type in topics.items():
            await ws.send(json.dumps({"op": "subscribe", "id": f"sub:{topic}",
                                      "topic": topic, "type": msg_type}))
        for _ in range(20):
            frame = json.loads(await asyncio.wait_for(ws.recv(), timeout=10))
            print(frame.get("op"), frame.get("topic"))

asyncio.run(main())
```

**Expected:** `publish /octopus/…` lines appearing at roughly 1 Hz.

**One interop detail this also exercises:** we send the **short** type form —
`std_msgs/String`, `sensor_msgs/NavSatFix` — not `std_msgs/msg/String`. The frame shapes above are
the ones our client actually puts on the wire, so a pass here is a genuine rehearsal rather than an
approximation.

**What each failure mode looks like:**

| Symptom | Almost certainly |
|---|---|
| connection refused | rosbridge not running, or the firewall/port |
| connects, then **silence** | the domain mismatch of step 1 — rosbridge is not on your nodes' graph. Or the topic is genuinely not publishing |
| `status` frames with `"level": "error"` mentioning the topic | the glob does not cover it — see the `fnmatch` note in step 3 |
| only the top-level topics arrive, not the nested one | the `/octopus/*` glob does not span `/` after all; add `/octopus/devices/*` |

**5d — the reverse direction.** The robot publishes back onto `/octopus/trash_goal_done` and, if you
accept item D, `/octopus/devices/gripperx/status`. To prove the return path works before we connect,
advertise and publish once from the same script and watch it with `ros2 topic echo`:

```python
await ws.send(json.dumps({"op": "advertise", "id": "adv:test",
                          "topic": "/octopus/trash_goal_done",
                          "type": "std_msgs/String"}))
await ws.send(json.dumps({"op": "publish", "id": "pub:test",
                          "topic": "/octopus/trash_goal_done",
                          "msg": {"data": "connectivity-test"}}))
```

```bash
ros2 topic echo /octopus/trash_goal_done
```

**Note the string is deliberately not a real target id** — a well-formed id on that topic would mark
a real piece of litter as collected in your system. Please use a value that cannot be mistaken for
one.

---

## 6. Make it survive a reboot

```ini
# /etc/systemd/system/octopus-rosbridge.service
[Unit]
Description=rosbridge for the GripperX link
After=network-online.target
Wants=network-online.target

[Service]
User=<YOUR_USER>
Environment=ROS_DOMAIN_ID=<YOUR_DOMAIN_ID>
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && \
  source <YOUR_WORKSPACE>/install/setup.bash && \
  ros2 run rosbridge_server rosbridge_websocket \
    --port 9090 --address 0.0.0.0 \
    --topics_glob "[\'/octopus/*\']" \
    --services_glob "[]" \
    --actions_glob "[]"'
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

> **Corrected to the `ros2 run` form for the same reason as step 3** — the launch-file form in the
> original version of this page would have made the unit fail on every boot.
>
> **STATUS 2026-08-21:** a unit is in the Octopus repo at
> `config/systemd/octopus-rosbridge.service` but is **NOT installed** — installing it needs root,
> which that host does not have passwordless. Until someone installs it, **the link does not come
> back by itself after a reboot**: it comes up with `scripts/start_octopus_debug_stack.sh`. Worth
> planning around before a joint session, not during one.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now octopus-rosbridge
systemctl status octopus-rosbridge
```

**`ROS_DOMAIN_ID` is written into the unit on purpose.** A unit inherits nothing from your
interactive shell, so a domain that is set in `.bashrc` and works in step 3 will be **absent** here —
and the failure is the silent one from the table above. Set it explicitly even if the value is `0`.

---

## 7. Not in scope, stated so it is not assumed

- **No TLS, no authentication.** Plain `ws://` on the local network. This is a deliberate
  deferral on both sides, recorded as such in our requirements — **not** an oversight, and **not**
  something we have solved quietly. If the link ever leaves a trusted network, it is reopened.
- **The transport carries the link and nothing else.** No motion command, no teleop and no velocity
  ever crosses it: on our side the external path terminates at our own Nav2 action client, and
  nothing behind it is topic-shaped.
  > **QUALIFIED 2026-08-21, and the qualification is theirs.** That sentence reasons about
  > **topics**, and on rosbridge 2.0.7 **actions are a first-class capability of the bridge
  > itself** — with `actions_glob` unset the link would have accepted `send_action_goal` for any
  > action server on their graph, regardless of what our client does. Nothing was exposed, because
  > their graph has no action server, but the property was resting on an empty graph rather than on
  > configuration. `--actions_glob "[]"` is now set and the claim rests on the configuration again.
  > **Our side of it is unchanged and was never the gap:** our client emits no action verb at all.

---

## 8. What we need back from you

**ALL FOUR ANSWERED 2026-08-21.** Kept here with their answers so the question and the answer sit
together:

1. **The address** — `ws://10.42.0.158:9090`, host `ITQLM125`, bound on all interfaces.
   **Not pinned across a reboot**, and a name can be set up if we ask.
2. **`0.0.0.0`** — bound, nothing objected. No firewall rule was needed because `ufw` is
   **disabled** on that host, which also means the port is open to anything that can reach it.
3. **The globs** — agreed in substance, corrected in form. See section 3.
4. **Step 5** — 5a–5d all passed, and `/octopus/*` covers the nested topic.

**What we now owe them**, and it is the only measurement neither side has: confirm GripperX can
reach `10.42.0.x`, and re-run their `check_rosbridge.py` **from the robot**. Their run was
loopback — host to itself — so the path between the two machines is still unmeasured.

---

## 9. What happens next on our side, so the size of the ask is clear

**The immediate next step is not driving a robot.** With rosbridge up we would run a **disarmed,
motion-free** session whose only purpose is to measure *your real timing over the link* — how
regularly the four topics actually arrive. One threshold on our side (`max_target_list_age_sec`) was
chosen against a test fixture and has **never seen your machine**.

No motion, no arm, no goal executed, no hardware, and nothing published back onto your graph beyond
the telemetry topic if you accept it. It needs rosbridge running and **nothing else from you**.

**Said plainly because it matters for sequencing:** the grasp bench measurement on our side has been
deferred, so this timing run is currently the only item on the whole GripperX↔Octopus track that
could start the day after the meeting.

**It does not, however, substitute for item A.** The joint geometry verification run stays blocked
on the map origin: while a constant ~2.8 m offset is present, no coordinate measurement over this
link can be told apart from a swapped axis. rosbridge unblocks *timing*, not *geometry*.

---

*Companions: `OCTOPUS_INTERFACE_PROPOSAL.md` (the full interface contract, item 5 is the short form
of this page) and `OCTOPUS_DECISION_BRIEF.md` (the item-by-item decision list, Q2 is this ask).*
