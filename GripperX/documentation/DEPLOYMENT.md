# GripperX — Deployment / Bringup Runbook

> ## RULE — EVERYTHING ON THE PI IS COMMITTED. NEVER A BARE COPY.
>
> **Every change that reaches `/home/ubuntu/ws` on the robot must be committed there.** Copying a
> file onto the machine without committing it is not a shortcut, it is the creation of an
> unrecorded state: the only description of what the robot executes then lives in a working tree
> that nobody can diff, review, or reproduce.
>
> **This is not precautionary.** A dirty Pi working tree makes `git pull` unsafe (uncommitted work
> has no second copy anywhere), makes it impossible to tell which changes are already in the
> repository and which are unique to the machine, and means the robot's actual running state cannot
> be reconstructed from any commit.
>
> **What to do instead, always:**
>
> 1. Change it in the repository, commit it, then bring it onto the Pi with `git pull`.
> 2. If something genuinely has to be tried directly on the machine — a value under test, a
>    debugging line — **commit it on the Pi before the session ends**, even as a scratch commit with
>    an honest message. A commit that says "trying this, unverified" is worth more than a clean
>    working tree that lies about what is deployed.
> 3. `git status` on the Pi is part of finishing a deploy, not an optional check. It should print
>    nothing. **But a clean tree is not a finished deploy** — the systemd service scripts live
>    outside the repository and a `git pull` does not touch them. See §0.
>
> If you find a dirty tree on the Pi, **commit it before you touch anything else.** Rescue first,
> merge second; the reverse order destroys evidence.
>
> ## RULE — NEVER BUILD ON THE PI WITH `--symlink-install`
>
> **On the Pi, build with plain `colcon build`.** The `ament_python` packages there cannot be built
> with `--symlink-install`: the attempt fails with `error: option --editable not recognized` (and on
> a second try `--uninstall not recognized`).
>
> **The failure is not clean, and that is the whole problem.** It removes the package metadata
> *before* it fails. `gripperx_control` was left without its `egg-info`, so every console-script
> entry point in it stopped resolving:
>
> ```
> importlib.metadata.PackageNotFoundError: No package metadata was found for gripperx_control
> ```
>
> `steer_servo_node` and `teleop_mux_node` then died on every start, systemd restarted the bringup,
> they died again — **a restart loop with the drive stack down**, and nothing in the symptom points
> at the build that caused it.
>
> **Recovery:** stop the service, delete `build/<pkg>` and `install/<pkg>` for the affected packages,
> rebuild with plain `colcon build`, then start the service. Verify the metadata came back:
> `ls install/<pkg>/lib/python3.12/site-packages/ | grep egg-info` must print something.
>
> Note that `--symlink-install` is fine and useful on the LAPTOP — the sim packages are built that
> way. This rule is about the robot only, and the difference is worth remembering rather than
> discovering.


**What / for whom / status:** binding operational runbook for the deployer and operator of the real robot; see the banner below for current state. Content last updated 2026-11-17 (unit list §0.1, features added since 2026-09-30 in §7); sections not named there are as dated in their own text.

> ## Applies to the REAL robot — LIVE and in use
> **This runbook is current and binding.** The robot is reassembled and in daily use: firmware is
> flashed and running, bringup is started routinely, and deployment to real hardware happens — most
> recently 2026-08-18/19 (FR-10, FR-11 provenance, `center_on_startup`, the SR-14 activation gate).
> **SR-1 is unchanged and is what governs execution:** no movement of drivetrain, steering or
> arm/gripper without explicit user approval *per test*, and a bringup restart is itself such an
> event.
>
> For simulation-based (digital twin) work, see the internal digital-twin plan (tracked internally,
> not in this repository) §9 — the rules there are analogous but for the sim, not the real robot.
>
> ### Before you execute anything below — two cautions
> These do not weaken the banner above; they say what to check first.
>
> - **Do not assume the systemd unit is what is running — verify it, don't inherit the last entry
>   in this document.** Bringup was hand-launched in 2026-08 (`setsid --fork gripperx-bringup.sh`),
>   which is the origin of this caution, but that state is gone: an on-machine check on 2026-09-21 verified
>   that bringup is **systemd-owned**, `MainPID 832`,
>   single instance inside the unit cgroup. Re-measured on **2026-09-23 19:36
>   CEST**, over read-only SSH (`systemctl show gripperx-bringup.service`): `ActiveState=active`,
>   `SubState=running`, `MainPID=842`, `ExecMainStartTimestamp=Wed 2026-09-23 10:37:51 CEST`, and
>   every node (`steer_servo_node`, `teleop_mux_node`, `robot_state_publisher`,
>   `ros2_control_node`, …) sat inside the unit's cgroup — **no hand-launched instance exists as of
>   that measurement.** The trap this caution exists to guard against is still real, in case
>   bringup is ever hand-launched again for debugging: a `systemctl restart
>   gripperx-bringup.service` on top of a hand-launched instance **doubles** the process rather
>   than replacing it, because systemd starts a fresh one without ever having owned the old one.
>   **Before any restart, establish which instance is live yourself** — `systemctl show
>   gripperx-bringup.service` for `MainPID`/`ActiveState`, and `journalctl`/`ps` to check for a
>   second, older process holding the same device or DDS participant — **not** a live `ros2` CLI
>   query (§2.1).
> - **The live state above is dated to its measurement, not evergreen.** Neither the 2026-08-21
>   merge nor most other edits to this file made Pi contact; every state claim in this banner is
>   attributed and dated. Re-check current state (`gripperx-diagnosis` or the `systemctl show`
>   command above) before relying on it for an actual deployment — this document is not a live
>   status page.

---

## 0. A `git pull` does NOT update what systemd runs (binding)

**Every `gripperx-*` unit runs `ExecStart=/usr/local/bin/gripperx-*.sh`. The repository keeps those
scripts in `Software/pi_env/systemd/scripts/`. Nothing connects the two.** Pulling on the Pi updates
a directory no unit ever reads. The running script is whatever was last copied into
`/usr/local/bin/`.

**Why this is not a footnote.** The rule at the top of this document says to change it in the
repository, commit, and `git pull` onto the Pi, and defines a finished deploy as a clean `git
status`. For anything under `systemd/` that procedure produces a spotless working tree and an
unchanged robot, and the completion check cannot tell the difference: a repointed script still
`git pull`s clean, still starts, still reports `active`, and still runs the OLD code, silently
(this is the reason for the check in §0).

**After changing anything under `Software/pi_env/systemd/`, on the Pi:**

```bash
sudo install -m 0755 -o root -g root \
  ~/ws/Software/pi_env/systemd/scripts/gripperx-*.sh /usr/local/bin/

# only if a .service/.timer changed:
sudo install -m 0644 -o root -g root \
  ~/ws/Software/pi_env/systemd/units/gripperx-* /etc/systemd/system/
sudo systemctl daemon-reload
```

Then prove it landed rather than assuming it did:

```bash
diff -u /usr/local/bin/gripperx-mapping.sh \
        ~/ws/Software/pi_env/systemd/scripts/gripperx-mapping.sh   # must be empty
```

Restarting the affected services is a **separate** step: §1 below governs how, and SR-1 governs
whether — a bringup restart is a movement event and needs explicit user approval.

**TO-VERIFY:** the `install` flags above were derived from the unit files and from
`Software/pi_env/README.md`'s statement that the live scripts are root-owned. They have **not** been
executed or checked against the machine (no Pi contact, 2026-08-24). Confirm ownership and mode with
`ls -l /usr/local/bin/gripperx-*.sh` before relying on them.

**Verified 2026-09-21** against the machine: `/etc/systemd/system/gripperx-*.service` are
`root:root 0644` and `/usr/local/bin/gripperx-*.sh` are `root:root 0755`, matching the two `install`
lines above — the `TO-VERIFY` above is resolved for these values. `diff` against the repository is
empty and all four services stayed `active`; `daemon-reload` restarts nothing, so running it is not
a motion event under SR-1.

**Trap:** a `Description=` or comment-only diff between the repository's unit file and the installed
one is enough to make `deploy_check.sh --ref <branch>` report DRIFT even though nothing functional
differs — keep the unit's comments and `Description=` text in sync too, not just `ExecStart=`.

**Trap, found 2026-09-24:** the `gripperx-*.sh` glob above **does not cover two of the installed
scripts** — `gripperx-stack-stop` (no extension; it is the one script the HWR-40 button and the three
operator commands call) and `gripperx-button-daemon.py`. Both live in the same repository directory and
both are installed under `/usr/local/bin`, so `deploy_check.sh` compares them, but the copy command
above silently skips them. After changing either, install it by name:

```bash
sudo install -m 0755 -o root -g root \
  ~/ws/Software/pi_env/systemd/scripts/gripperx-stack-stop /usr/local/bin/gripperx-stack-stop
sudo install -m 0755 -o root -g root \
  ~/ws/Software/pi_env/systemd/scripts/gripperx-button-daemon.py /usr/local/bin/gripperx-button-daemon.py
```

Installing `gripperx-stack-stop` restarts nothing and moves nothing: it is executed on demand — by the
button daemon, by an operator, and by `gripperx-bringup.service`'s `ExecStop=` — so a new copy takes effect
at the **next invocation**, with no service action at all. `gripperx-button-daemon.py` is different: it is a
resident process (`gripperx-button.service`), so it keeps executing the OLD copy until that unit is
restarted. Restarting `gripperx-button.service` is not a motion event — the daemon only reads a GPIO line —
but it does mean the button is briefly dead, so do it deliberately rather than as a side effect.

### 0.1 The units in `Software/pi_env/systemd/units/` (as of 2026-11-17)

| Unit | What it runs | Relation to the motion stack |
|---|---|---|
| `gripperx-agent.service` | micro-ROS agent for the ESP32 (Docker) | independent; first in Mode R (shutdown-button short press: restart of the ROS stack) start order |
| `gripperx-bringup.service` | URDF, ros2_control, swerve controller, steering/arm nodes, LiDAR, BNO085 IMU (§7.5) | the motion stack: **restart is an SR-1 event** |
| `gripperx-mapping.service` | EKF, laser odometry, `slam_toolbox` | `Requires=` bringup, 20 s delay |
| `gripperx-navigation.service` | Nav2 | `Requires=` mapping, 15 s delay |
| `gripperx-external.service` | Octopus link (§3.3) | `Wants=` navigation, 15 s delay |
| `gripperx-camera.service` | front camera driver `jpeg_camera` (§7.6) | **outside** the motion stack: no `Requires=`/`PartOf=`; a stack stop or Mode R does not touch it; restarting it needs no SR-1 approval |
| `gripperx-button.service` | clean-shutdown button daemon (GPIO17: short = stack restart, long = halt) | resident process, see the trap in §0 |
| `gripperx-wifi.service` + `.timer` | WiFi reconnect every 30 s | independent |
| `gripperx.target` | proposal, installed but disabled, inert | none |

`gripperx-stack-stop`'s `STOP_ORDER`/`START_ORDER` cover agent, bringup, mapping, navigation and
external — not the camera. The camera unit's own header (`gripperx-camera.sh`, `gripperx-camera.service`)
gives the reasoning.

**Install steps when the unit set changes** (units and scripts are new or changed files, so
`deploy_check.sh` reports them `MISSING` or DRIFT until this has run): the §0 commands cover
`gripperx-*.sh`; add the camera unit explicitly and enable it once:

```bash
sudo install -m 0644 -o root -g root \
  ~/ws/Software/pi_env/systemd/units/gripperx-camera.service /etc/systemd/system/gripperx-camera.service
sudo systemctl daemon-reload
sudo systemctl enable gripperx-camera.service   # starts nothing; it comes up at the next boot or by hand
```

`gripperx-bringup.service` changed since the last team export (it now declares `RuntimeDirectory=gripperx`
for the drive-fault marker, §7.1): a changed unit file takes effect only after `daemon-reload` **and** a
restart of that unit, and the bringup restart is an SR-1 event (§1).

**Not in this repository (TO-VERIFY):** a CPU-governor unit `gripperx-cpu-performance.service` was
installed on the Pi on 2026-11-16 (performance governor). Its unit file is
not in `Software/pi_env/systemd/units/` of branch `Theo`, so this document does not describe it and
`deploy_check.sh` has no copy to compare. Whether and when it is merged is open.

---

## 1. Clean-teardown bringup restart procedure (binding)

This is the single most important operational procedure for restarting the real robot. It resolved
a confirmed root cause — controller-manager spawners dying after a watchdog deploy, traced to
DDS-restart zombies: a duplicate/competing micro-ROS agent on `/dev/esp32` — and is **binding for
every future bringup restart**.

**Never restart `gripperx-bringup.service` (or do a full stack restart) without this sequence:**

1. **Stop services** — in reverse dependency order (**external** → navigation → mapping → bringup →
   agent), via the normal `systemctl stop gripperx-*.service` path.

   *`gripperx-external.service` (the FR-12 Octopus link, §3.3) joined this list on 2026-09-24 and goes
   FIRST. It only `Wants=` navigation, so no stop propagates to it from anything, and stopping it while
   Nav2 is still up is what lets the gateway's own shutdown path cancel a goal in flight — a cancel is
   worth nothing once `/navigate_to_pose` has already vanished.*
2. **`docker stop mros_agent`** — if the micro-ROS agent runs as a Docker container, a plain
   `systemctl stop` of the wrapping service does **not** kill the container (`docker run --rm`
   semantics; the container survives service stop and becomes a DDS zombie). This step is
   mandatory, not optional — it was the missed step that caused the spawner-death incident above.
   Note: `docker stop` takes a few seconds and is not a hang.

   *(`gripperx-agent.service` sets `ExecStop=docker stop -t 10 mros_agent` with `TimeoutStopSec=15`,
   so systemd gives the stop 15 s and the container 10 s before SIGKILL. The actual wall time this
   step takes is TO-VERIFY rather than replaced with a number nobody measured; see
   AUDIT_OPS_2026-08-24.md Q3.)*
3. **`rm -f /dev/shm/fastrtps_*`** — clean up stale FastDDS shared-memory transport segments. A Pi
   freeze/reboot can leave orphaned SHM segments behind that cause a **complete SHM transport
   failure** on the next boot (see §2.2 below for the symptom and how to recognize it).
4. **Zombie check** — before starting anything, verify there is no duplicate/leftover process
   holding a device or DDS participant:
   - No duplicate micro-ROS agent process/container on `/dev/esp32` (the classic
     zombie signature — two competing agents, both older than the current bringup instance).
   - No orphaned **laptop-side** teleop process either — `keyboard_teleop_node` has a latch-style
     W/S drive command with no timeout; an orphaned instance left running on the laptop was one of
     the two stacked root causes of the 2026-07-06 unwanted-motor-run incident (the other being the
     controller_manager hang itself — the full incident is recorded in the internal safety audit and
     the internal session records, neither in this repository). Check
     with `ps aux | grep keyboard_teleop` on the laptop before touching the robot.
   - `ros2 daemon stop` — clears any stale ROS 2 daemon discovery state left over from the previous
     session, independent of the process-level zombie check above.
5. **Start in strict order, waiting for each stage to fully come up before starting the next:**
   `agent` → `bringup` → `mapping` → `navigation` → `external`.

   *`external` last: it is the only consumer of the whole stack (`/navigate_to_pose`, the
   `map` → `base_footprint` TF (coordinate transform), `/odometry/filtered`, `/teleop/active_mode`) and nothing consumes it.
   It is the one stage you do **not** have to wait for — the gateway treats a missing Nav2 as the
   `NAV2_UNAVAILABLE` state, reports it and repairs itself when discovery matches, so starting it early
   degrades to telemetry-only instead of failing (§3.3).*

   **This order is yours to keep — systemd only enforces part of it.** Checked against the unit
   files 2026-08-24: `gripperx-mapping.service` has `Requires=`/`After=gripperx-bringup.service`
   (+`ExecStartPre=/bin/sleep 20`) and `gripperx-navigation.service` has
   `Requires=`/`After=gripperx-mapping.service` (+`sleep 15`), so the last three are chained.
   **`gripperx-agent.service` is not in that chain at all** — it depends only on `docker.service`
   and the network, and nothing makes `bringup` wait for it. On a boot the agent and the bringup
   race. Starting the agent first by hand, as this step says, is therefore a real instruction and
   not a restatement of what the units already guarantee.
   `gripperx-external.service` (added 2026-09-24) is `After=`/`Wants=gripperx-navigation.service`
   (+`ExecStartPre=/bin/sleep 15`), so its **order** is enforced but its **dependency is not**:
   `Wants=` is deliberate, because a failed Nav2 must not take the external link down with it — the
   link is then the only thing still reporting to the Octopus, and their side has no failure channel
   (§3.1, last bullet).

Every SSH action against the robot should use a retry loop, not a single attempt — the connection
(commonly over an iPhone-hotspot link when the LAN cable isn't practical) is **unstable** ("No route
to host" drops are routine):
```bash
until ssh ubuntu@gripperx-1.local "…"; do sleep 5; done
```

---

## 2. Related operational lessons

### 2.1 Journalctl-first, no CLI hammering right after a restart

**During and shortly after any bringup restart, do NOT run `ros2` CLI diagnostics** (`ros2 node
list`, `ros2 topic echo`, etc.). Live evidence strongly correlated additional DDS
RELIABLE participants, created by exactly these diagnostic commands (especially over the
high-latency hotspot link, 30-70 ms+ RTT), with `controller_manager` write() times spiking well past
the 33 ms budget into the 300+ ms range, which can make the controller spawners time out and die
before the controllers ever activate. **Use `journalctl -u gripperx-<service>.service` instead**, it
generates no DDS traffic. Let the stack settle unobserved for a short period after a restart before
doing any live `ros2` inspection at all.

### 2.2 "Discovery OK but 0 Hz data" = check for SHM transport failure

**Symptom:** `ros2 node list` (or endpoint discovery generally) looks correct — all expected nodes
are visible — but topics carry **zero payload data** between Pi-internal processes (`/cmd_vel`,
`/joint_states`, etc. all report 0 Hz). Teleop commands arrive at the Pi at the expected rate but
never reach their destination.

**Root cause (confirmed 2026-07-09):** FastDDS's shared-memory (SHM) transport had failed
completely after a Pi freeze/reboot, most likely due to orphaned SHM segments from the frozen
process. Discovery (which can fall back to UDP) still worked; the data path (which was trying to use
SHM) did not.

**Fix / mitigation:**
- All 4 Pi service scripts run FastDDS on a **UDP-only profile**
  (`fastdds_udp_only.xml`) instead of the default (SHM+UDP) profile — this trades a small amount of
  loopback-transport performance for eliminating the SHM-orphan failure mode entirely. Treat this as
  a hardening choice, not a workaround to later remove.
- `rm -f /dev/shm/fastrtps_*` is now a standing part of the teardown procedure (§1, step 3) for
  exactly this reason.
- **Diagnostic rule:** if you ever see "discovery looks fine, but nothing flows" on the Pi again,
  check for a stale/failed SHM transport first before assuming a code-level bug in the
  publisher/subscriber chain.

### 2.3 `controller_manager` overrun caution (background to §2.1)

`ros2_control_node` has been observed exceeding its 30 Hz control-loop budget (33 ms) by a wide
margin — measured `write()` times from 64 ms up to 319 ms in a single spike. This is what makes the
"no CLI hammering right after restart" rule (§2.1) matter in practice: the controller spawners
(`wheel_velocity_controller`, `steering_position_controller`) apply a 3× timeout policy and will die
before ever activating if enough overruns stack up during the vulnerable startup window. Not fully
solved — moderate overruns during the settling phase of a fresh bringup (40–132 ms, a few missed
cycles) are considered expected/benign; only sustained or extreme overruns (300 ms+) during startup
are cause for concern.

### 2.4 `controller_manager` wedged — steering has no software recovery (accepted, OP-27)

**If `controller_manager` is wedged, there is no software way to straighten the steering.** This
covers every software path to the steering servos: the centring half of the spacebar E-stop
(`keyboard_teleop_node.py`) and the dedicated non-emergency centring command (`FR-13`, key `c`)
are both consumed by `swerve_controller`, which runs inside `ros2_control_node` and is only ever
executed by the `controller_manager` update loop. A wedged CM consumes nothing, so neither path
reaches the servos (internal requirements document, OP-23 / A2-b, FR-13 §4 — tracked internally,
not in this repository).

> **AUDIT NOTE 2026-08-24 — two claims in the paragraph above did not survive a read of the code.
> Nothing here is a decision; the paragraph is left standing and the questions go to the user
> (internal operations audit 2026-08-24, Q1 and Q2).**
>
> 1. **`FR-13`, key `c` does not exist yet.** `gripperx_teleop/keyboard_teleop_node.py` has no `c`
>    binding — its keys are `W`/`S`, `A`/`D`, arrows, Space, `K`, `G`, `P`, `O`, `I`, `U`, `L`,
>    `Q`. internal REQUIREMENTS lists FR-13 as "clarified … **not implemented**". The paragraph names
>    it as an existing path.
> 2. **The spacebar's centring half may not traverse `controller_manager` at all.**
>    `KeyboardTeleopNode.center()` publishes four zeros on `/teleop/direct_steer`.
>    `steer_servo_node._on_timer()` checks that override **first** and, when it is fresh and the
>    mode is not `autonomous`, calls `_write_angles()` and **returns before** the
>    `/hw/joint_commands` path. `steer_servo_node` is its own process (`real_robot.launch.py`
>    starts it as a plain `Node`), not a controller inside `ros2_control_node`. `center()` also
>    publishes `keyboard` on `/teleop/set_mode` first, which is what makes the mode condition
>    true. On that reading the centring half would still reach the servos with a wedged CM.
>
> **Do not act on point 2 as if it were established.** It contradicts a decision the user accepted
> on the record (OP-23/A2-b 2026-08-17, reconfirmed at OP-27 2026-08-19), it was derived from the
> repository with **zero Pi contact**, and `swerve_controller` carries its own `/teleop/direct_steer`
> arbitration ("point A2") that this audit did not trace. **Continue to treat a wedged
> `controller_manager` as unrecoverable in software** and use the power-cycle fallback below until
> the user rules.

**The drive is unaffected.** It still stops three ways, independent of the CM: the `teleop_mux`,
the hardware-interface command watchdog (see the internal watchdog-deploy runbook, tracked
internally, not in this repository), and the ESP32 firmware's own
`CMD_TIMEOUT_MS` (1000 ms). Only steering centring is lost.

**The steering does not drift into a dangerous state on its own — it holds.** Under OP-24/S1 the
steering deliberately holds its last commanded angle instead of snapping to centre when its command
source is stale or absent; a wedged CM does not change that.

**Recovery is non-software, and is itself the accepted decision — not a workaround pending a fix
(OP-27, option (a), DECIDED by the user 2026-08-19):** power-cycle the stack. The Feetech steering
servos are back-drivable once their torque is off, so the wheels can then be straightened by hand.

**This is an accepted cost, not a regression or a gap awaiting a fix.** It is the narrowing the user
accepted on 2026-08-17 with OP-23/A2-b, reconfirmed at OP-27 on 2026-08-19; the alternative (a
`controller_manager`-independent centring path) was considered and explicitly rejected as a
structural reversal of the OP-23 decision. See the internal requirements document — OP-23,
OP-24/S1, OP-27, FR-13, SR-2 — for the normative text (tracked internally, not in this
repository).

---

## 3. Octopus external-goal link — deployment-time constraints

Applies once FR-12 (external litter goals from the Octopus team over the rosbridge/WebSocket link;
twin stage accepted 2026-08-20, real robot NOT yet approved — see the internal requirements
document, FR-12 §10.1, tracked internally and not in this repository) is deployed together with
this document's real-robot bringup procedure. The two rules below come from the internal safety
audit (§6.4 items 2 and 6) and the internal requirements document (FR-12 §10.1 items 3 and 7 —
neither in this repository) and are written here in deployer terms — what to do at the moment of
deploying or touching this part of the system, not what an auditor would look for afterward.

### 3.1 No bridge, ever, on the real robot's domain (internal safety audit finding F-10)

- **Never start `rosbridge_server`, `ros1_bridge`, or any DDS-domain bridge on the real robot's
  `ROS_DOMAIN_ID=20`** — not for a dashboard, not for a demo, not "just to look while debugging".
  The Octopus link is a WebSocket **client** only; it must never itself become, or be joined by, a
  DDS participant on domain 20. This is a hard boundary, not a preference: any bridge on domain 20
  exposes `/gripperx/external/set_arming` to whoever can reach it, and arming is not a minor
  service — per SR-16 it is now the authorization for the arm to move. A bridge on domain 20 doesn't
  weaken the arming gate a little, it removes it, because the gate was never designed to survive a
  second path onto that topic.
- If a dashboard or demo genuinely needs Octopus data on the LAN, route it through the WebSocket
  relay the Octopus link node already exposes, or a read-only export — never through a bridge onto
  domain 20.
- On the Octopus side, their rosbridge must keep **three** globs closed:
  `--topics_glob "['/octopus/*']" --services_glob "[]" --actions_glob "[]"`.
  `services_glob "[]"` is what disables `rosapi`, so a client cannot enumerate or call anything and
  only the glob'd topics are reachable.
  > **Traps specific to rosbridge 2.0.7 (the version the Octopus side runs):**
  > - **`params_glob` does not exist on 2.0.7.** Passing it is silently ignored — no warning, no
  >   error. What actually closes parameter access is `services_glob` **plus not running the `rosapi`
  >   node**, because parameter reads and writes travel through `/rosapi/get_param` and friends. A
  >   deployer verifying `params_glob` is verifying nothing.
  > - **An unset `actions_glob` exposes every action server on the graph.** Actions are a
  >   first-class rosbridge capability on 2.0.7; leaving `actions_glob` unset means **any action
  >   server on the graph** accepts `send_action_goal` across the link. Safety here must come from
  >   the `--actions_glob "[]"` setting, not from the graph happening to have no action server today.
  > - **The `ros2 launch` form kills the node.** On 2.0.7 the globs are `STRING` parameters that
  >   rosbridge parses itself; `ros2 launch` coerces a bare bracket list to `STRING_ARRAY` and the
  >   node dies at startup with `InvalidParameterTypeException`. Use `ros2 run
  >   rosbridge_server rosbridge_websocket` with the globs as **quoted strings**.
  >
  > The authoritative page is `documentation/OCTOPUS_ROSBRIDGE_SETUP.md` §3.
- **Status of the constraint — endpoint MEASURED 2026-09-24, on a new address.** The link is
  `ws://192.168.50.30:9090` on host `ITQLM125`, and this is the first time the endpoint has been
  reached **from the robot** rather than reported: ping and a TCP connect to 9090 from the Pi, then a
  subscribe that returned live frames on all three inbound topics at ~1 Hz. The earlier value
  `ws://10.42.0.158:9090` — Octopus team reply of 2026-08-21, rosbridge **2.0.7 built from source**,
  bound `0.0.0.0`, steps 5a–5d passed (`OCTOPUS_ROSBRIDGE_SETUP.md` §8) — was **never once reachable
  from this robot** and had gone stale by the time it was written down.
  **Why it should now stay put:** the segment used to be a developer laptop's NetworkManager *shared*
  connection on `10.42.0.0/24`, that range being NetworkManager's default, so two machines on "the same
  range" were routinely on two different networks. Since 2026-09-24 a TL-WR840N owns
  `192.168.50.0/24` with one DHCP reservation per MAC (`LOCAL_ENV.md` §2). **Still re-verify before a
  session** — not the address now, but what is actually running on their host: the two caveats below
  are from the 2026-08-21 check and were not re-examined on 2026-09-24. That
  verification requirement is also what caught the `params_glob` error described above: a deployer must
  **verify against what is actually running on their host** rather than trusting any document — this
  one included — as proof of what is deployed. Two more caveats as of the same 2026-08-21 check,
  equally unverified since: their `ufw` was **disabled**, so the port was open to anything that could
  reach it; and the systemd unit was **not installed**, so rosbridge did **not** come back after a
  reboot of their host on its own — it was started by `scripts/start_octopus_debug_stack.sh`.
- **A refusal to start is silent to the Octopus.** Their side has no failure channel today: if the
  link node exits non-zero (exit code 2, e.g. on a sim-time misconfiguration under SR-15 rule 12, or
  any other startup refusal), what the Octopus operator sees is a link that simply never appears —
  not a reason. Consequence for whoever deploys the link node: check the link node's own exit status
  and journal (`systemctl status gripperx-external.service`, then
  `journalctl -u gripperx-external.service -b --no-pager` — the unit exists since 2026-09-24, see
  §3.3) directly after every deploy or restart. Do not
  infer link health from the Octopus side ("no goals arriving" does not mean "not running", and "the
  Octopus dashboard shows a connection" does not mean the last restart succeeded cleanly). Do not
  report the link healthy to the Octopus team without having read its log yourself.

### 3.2 `SIMULATION_DOMAIN_IDS` is a safety constant, not configuration (internal safety audit §6.4 item 2, §6.6)

- `SIMULATION_DOMAIN_IDS = {220, 221}` in the Octopus link package decides which domain takes the
  permissive (simulation) branch — including whether the SR-15 rule 12 sim-time startup refusal
  applies. F-34's import-time invariant now makes the worst version of an edit here — putting `20`
  into the set — impossible at import time, tested through its own failure (A-35). That is a floor,
  not the whole guarantee.
- **What the invariant does NOT defend against:** `220` and `221` are in the set by convention, not
  by any property the code checks — `220` because that is the twin domain a live Gazebo/Nav2 in
  another worktree happens to use, `221` because the offline test harness happens to be pinned to it.
  Nothing stops a future edit from adding a third number — a new twin domain, a CI runner's domain, a
  fleet-numbering scheme — that turns out to collide with a real robot's domain, or from being wrong
  about whether some domain is actually simulation-only.
- **Rule for whoever edits this set: treat any change to `SIMULATION_DOMAIN_IDS` — addition, removal,
  or edit of an existing entry — with the same review weight as a change to the arming gate
  (SR-15/SR-16), not as a routine constant tweak.** Concretely, before merging such a change:
  1. State in the commit/PR which domain is being added or removed and *why* it is known to be
     simulation-only (not "seems fine", a traceable reason).
  2. Get it reviewed by someone who did not write the change.
  3. Re-run the F-34 import-time invariant test (A-35) as an explicit part of that review, not just
     as background CI that nobody looks at unless it's red.
  Treat "it's just a constant" as the trap it is: this is the one edit in the whole package that can
  make a real machine silently take the simulation branch.
- This rule is not theoretical caution — it responds to a documented near-miss. The audit's own first
  proposal for closing F-27 would have pushed the offline test harness onto domain `220`, where a
  live Gazebo and Nav2 belonging to another worktree run — a worse outcome than the one it was meant
  to prevent (internal safety audit §6.6, not in this repository). The current membership of the set is deliberate and was reviewed
  after that near-miss; the next change to it needs the same scrutiny, not less because "it worked
  last time."


### 3.3 `gripperx-external.service` — the link as a service (autostart and Mode R)

**Added 2026-09-24. Before this, `gripperx_external` was built and installed on the Pi but had no unit:
it only ever ran from a manual `ros2 launch`, so it was absent after every boot and after every Mode R
restart — and absent is exactly the state the Octopus cannot distinguish from "running but silent"
(§3.1, last bullet).**

| | |
|---|---|
| Unit | `Software/pi_env/systemd/units/gripperx-external.service` → `/etc/systemd/system/` |
| Script | `Software/pi_env/systemd/scripts/gripperx-external.sh` → `/usr/local/bin/` |
| Starts | `ros2 launch gripperx_external octopus_link.launch.py env:=real goal_ingress:=true dry_run:=false use_sim_time:=false url:=<read from the config>` — two nodes, `octopus_link_node` (transport) and `goal_gateway_node` (judgement), both in namespace `/gripperx/external` |
| Rollout stage | **1 — telemetry only.** No goal ingress, gateway disarmed, dry-run. See below. |
| Ordering | `After=`/`Wants=gripperx-navigation.service` + `ExecStartPre=/bin/sleep 15` |
| Boot | covered by `systemctl enable` (`WantedBy=multi-user.target`) |
| Mode R restart | covered by `gripperx-stack-stop`'s `STOP_ORDER`/`START_ORDER` — **not** by being enabled |

**Rollout stage 3 — user decision 2026-09-24, during the first real-robot test.** The launch file
defines three stages: 1 = telemetry only (its defaults), 2 = `goal_ingress:=true` with the gateway
disarmed and in dry-run, 3 = additionally `dry_run:=false`. The unit runs **stage 3**: the link node
subscribes the Octopus goal topics, the gateway validates each goal against the line calibration
(`documentation/OCTOPUS_LINE_CALIBRATION.md`) and its geofence, and dispatch is not blocked by dry-run.
**The only remaining block is arming** — the `SetArming` service on the robot's own domain, nothing
else, and it expires by itself (SR-15 rules 3 and 4). Arming is therefore a motion approval under SR-1.
*(Superseded 2026-09-24: this section used to run stage 1 and to state that `dry_run:=false` is never
set by an autostart script. The user overrode that deliberately, knowing that FR-12 §10.1 — the items
owed before the real robot — is not yet met. Stepping back is a one-token edit of
`gripperx-external.sh` (`dry_run:=true` → stage 2, `goal_ingress:=false` → stage 1) plus a reinstall
(§0) plus a restart of this unit.)*

**Starting or restarting this unit is not a motion event under SR-1.** It starts two pure-python rclpy
nodes that publish no command topic and hold no hardware interface; their only route to an actuator is a
Nav2 or `/pick_plastic` goal, which the gateway sends only while armed — and it always starts disarmed.
That is *not* a statement about `gripperx-bringup.service`, whose restart remains a motion event.

**Install step** (after the repository is on the Pi; the unit and script are new files, so
`deploy_check.sh` reports them `MISSING` until this has run):

```bash
sudo install -m 0755 -o root -g root \
  ~/ws/Software/pi_env/systemd/scripts/gripperx-external.sh /usr/local/bin/gripperx-external.sh
sudo install -m 0644 -o root -g root \
  ~/ws/Software/pi_env/systemd/units/gripperx-external.service /etc/systemd/system/gripperx-external.service
sudo systemctl daemon-reload
sudo systemctl enable gripperx-external.service
```

`daemon-reload` restarts nothing (verified 2026-09-21, §0) and `enable` starts nothing — the unit comes
up at the next boot, or immediately if it is started by hand. Do **not** forget
`/usr/local/bin/gripperx-stack-stop`: the Mode R coverage lives in that script, and the §0 glob does not
match it (see the trap there).

**Where to look when it does not come up.** The journal is the only path: the gateway's `/diagnostics`
and `/gripperx/external/status` exist only while it runs, which is precisely the failure this section is
about.

```bash
systemctl status gripperx-external.service
journalctl -u gripperx-external.service -b --no-pager
```

- **`status=78/EX_CONFIG`** — a **deliberate refusal**, caught by the script's pre-flight SR-8 probe
  before `ros2 launch` is started at all: the live `ROS_DOMAIN_ID` does not match the
  `expected_domain_id` in `octopus_link_real.yaml`. `RestartPreventExitStatus=78` means systemd does
  **not** retry this; the unit stays `failed` until somebody fixes the configuration.
- **`Result: exit-code`, or "Start request repeated too quickly" in the journal** — the launch tree kept
  ending on its own. `StartLimitIntervalSec=300`/`StartLimitBurst=4` bound that to four attempts in five
  minutes, then the unit latches `failed`. The reason is in the nodes' own `FATAL`/`ERROR` lines, most
  likely the `use_sim_time` refusal (F-24) or a missing dependency.
- **`active (running)` but the Octopus sees nothing** — that is the link itself (wrong `url`,
  unreachable host, rosbridge not running on their side), not the unit. The link node logs its
  connection attempts and backoff; §3.1 covers the endpoint.
- Every **stop** of this unit logs systemd's own verdict (`ExecStopPost`:
  `[stop] result=… code=… status=…`), so the journal distinguishes a requested stop from a self-exit
  without anybody having been logged in at the time.

**Why the unit tolerates a missing Nav2 instead of requiring it.** `Wants=`, not `Requires=`: the
gateway polls `server_is_ready()` on every dispatch tick, never the blocking `wait_for_server`, and
reports a missing `/navigate_to_pose` as the `NAV2_UNAVAILABLE` auto-disarm trigger with no grace
period. So it starts and runs without Nav2, degraded to telemetry-only, and repairs itself when
discovery matches. Taking the link down on a Nav2 failure would replace a *reported* degradation with
the silence §3.1 warns about. The consequence to know: `systemctl stop gripperx-bringup` takes
navigation and mapping with it (via `Requires=`) and **leaves this unit running**. That is intended; for
a whole-stack teardown use `gripperx-stack-stop --mode=stop`, which lists it explicitly.

**`Restart=always` is deliberate and is not the lazy choice.** `ros2 launch` returns 0 unless the launch
service itself raises — a child that exits non-zero does not set its return code — so both nodes exiting
`FATAL` with code 2, which is the *designed* response to an SR-8 or F-24 misconfiguration, produces an
idle launch service and a clean exit 0. For this unit a clean exit 0 **is** a failure, because the only
reason launch has nothing left to supervise is that its children are gone; `Restart=on-failure` would
leave the unit `inactive (dead)` with the word "successfully" in the journal. `always` does not restart
after a stop that systemd or the Mode R sequence requested, so a deliberate stop stays stopped.

**The `url:=` argument is a workaround for a launch-file defect, found 2026-09-24 — read this before
debugging a connection.** `octopus_link.launch.py` builds the link node's parameters as
`parameters=[params_file, dict(overrides, url=url)]`, and `url` is a launch argument whose
`default_value` is `ws://127.0.0.1:9090`. A later entry in that list wins, so **the launch argument's
default silently overrides `url:` in `octopus_link_real.yaml`**: a start that passes no `url:=` connects
to localhost — the `test/fake_octopus.py` fixture's address — whatever the config says. The twin never
noticed, because on a laptop `127.0.0.1:9090` is exactly where the fixture listens; on the robot it is
nothing. `gripperx-external.sh` therefore reads the address out of the same installed config file the
nodes load and hands it straight back as `url:=`, which makes the override a no-op. **The script
contains no address of its own** and works with whatever ends up in the YAML. Two consequences:

- The config file really is the one place the Octopus address is set — but only *because* the wrapper
  does this. A manual `ros2 launch` without `url:=` still goes to localhost.
- The durable fix belongs in the launch file (do not override `url` unless the argument was given
  explicitly). That file is shared with the twin and the acceptance harness, so it is a deliberate
  change, and it **has not been made**. Until it is, do not remove the `url:=` forwarding from the
  wrapper.

**Known gap, not closed by this unit (TO-VERIFY):** if only **one** of the two nodes dies, `ros2 launch`
keeps running the other, so the unit stays `active` with half the interface dead. Closing that means
`on_exit=Shutdown()` or `respawn=True` in `octopus_link.launch.py`, which is shared with the twin and the
acceptance harness — a deliberate change to that launch file, not a deployment setting, and it has not
been made.

---

## 4. Nav2 integration — deployment-time constraints

**This section applies now.** The Nav2 stack consolidation landed in `Theo` on 2026-08-21 and is in
the tree: `gripperx_bringup/config/nav2_params.yaml` and `gripperx_bringup/launch/navigation.launch.py`
are **gone**, `gripperx_planning/launch/navigation.launch.py` and the `gripperx_behaviors` plugin
package are **present**, and `gripperx-navigation.sh` starts `gripperx_planning`. Re-verified against
the tree 2026-08-24.

**Landed in the repository is not deployed on the robot.** No Pi contact confirms this. A deployer
must check what the machine is actually running — and note that a `git pull` alone does not update
the service scripts (§0).

Geometry/tuning changes from that integration (wheel radius, lever arm, meshes, footprint,
tolerances, recoveries) are deliberately **not** restated here — see `documentation/ASBUILT.md`.

### 4.1 New package `gripperx_behaviors` is a hard start blocker

- **`gripperx_behaviors` (a new C++ package) must be built on the Pi before the next
  navigation-stack start.** If it is missing, `behavior_server` fails to configure
  and **no Nav2 node comes up at all** — not a degraded stack, nothing comes up. The failure gives no
  hint from the usual symptoms about what caused it.
- **Action:** build `gripperx_behaviors` as part of the standard colcon build **before** the first
  post-merge bringup/navigation restart. If Nav2 fails to come up after the merge and the cause is
  not obvious, check whether this package is present and built **first**, before chasing anything
  else — this is the single most likely cause of a totally dead navigation stack post-merge.

### 4.2 Stack A is deleted — the real robot has been running the deprecated stack until now

- The merge deletes `gripperx_bringup/config/nav2_params.yaml` and
  `gripperx_bringup/launch/navigation.launch.py`; `gripperx-navigation.sh` is repointed at
  `gripperx_planning` with `use_sim_time:=false`.
- **Worth stating plainly:** whatever navigation ran on the real robot before this merge was the
  **deprecated** stack (Stack A). This is a change in what actually runs on the next restart, not a
  tidy-up of unused files. Anyone restarting navigation is bringing up a materially different stack
  than whatever was tested or observed before 2026-08-21, and should not assume behaviour carries
  over.

### 4.3 Nav2 autostarts on restart; `slam_toolbox` is driven for it

- Nav2 has `autostart: True` — it reaches `active` on its own after a process restart.
- **`slam_toolbox` still has no autostart of its own. Since 2026-08-24 something else drives it.**
  `gripperx-mapping.sh` now launches `gripperx_localization/localization.launch.py`, which issues
  the `configure` transition and, on the resulting `inactive` state, the `activate` transition
  (`LifecycleTransition` + `OnStateTransition` in that launch file). The script then waits up to
  20 s and, **only if the node is still not active**, drives `configure`/`activate` by hand as a
  safety net, warning loudly if even that fails. Verified 2026-08-24 against
  `Software/pi_env/systemd/scripts/gripperx-mapping.sh` and
  `gripperx_localization/launch/localization.launch.py`.
- **This is repository state, not robot state.** The units run
  `/usr/local/bin/gripperx-mapping.sh`, and a `git pull` does not update that copy (§0). Until the
  script is installed on the Pi, the old hand-driven behaviour is what the machine does.
- **Consequence for a deployer, unchanged in substance:** still check `slam_toolbox`'s lifecycle
  state explicitly after every restart — `ros2 lifecycle get /slam_toolbox` — and do not infer it
  from Nav2 being up. Nav2 green with `slam_toolbox` inactive is a half-up stack that misdiagnoses
  as a Nav2 problem. Note the anchoring trap the script documents: `ros2 lifecycle get` prints
  `inactive [2]`, so an unanchored `grep active` reports an inactive node as active.
- **Untested via an actual cold boot.** This autostart path has not been exercised by a full
  power-cycle boot on the Pi. (The premise this line used to rest on — "bringup has been
  hand-launched since 2026-08-19, so the whole systemd chain is unexercised" — no longer holds:
  see the caution at the top of this document. Bringup was systemd-owned when checked on
  2026-09-21 and again on 2026-09-23, but that is evidence from a service *restart*, not from a
  cold *boot* — the two are different tests, and no cold-boot run of this specific autostart path
  is on record.)

### 4.4 Nav2 accepts a misspelled parameter key silently — verify by listing, not by reading YAML

- A misspelled key in the Nav2 param YAML produces **no warning and no error** — it is silently a
  no-op, not a rejected config. The Nav2 track found eight of these in their own work.
- **The check that actually works:** `ros2 param list` per node, compared key-by-key against what the
  YAML says the node should hold. Reading the YAML alone proves nothing about what actually took
  effect on the running node.
- **Named trap they hit, worth repeating exactly:** the goal-checker parameter is named
  **`goal_checker`**, **not** `general_goal_checker` — do not trust the more Nav2-idiomatic-looking
  name; verify the real key against the node's own `ros2 param list` output.

---

## 5. Arm servo EEPROM (non-volatile servo memory) limits — writing SR-17 layer L1 (maintenance procedure)

**What:** the one-off (and after every calibration, joint-limit or margin change, repeated) write of
the min/max angle limits into the six arm servos' EEPROM (non-volatile servo memory) — SR-17 (arm
safety envelope) rule 1 = layer L1, checked by rule 2 (`eeprom_check`). **Status 2026-10-13:
prepared, never run on the robot.** Tool: `arm_eeprom_limits` (`gripperx_arm/eeprom_limits.py`; its
docstring lists every refusal). **The torque-off (step 4), the write (step 6) and every restart are
SR-1 (motor-enable process rule) events: each needs its own user approval.**

**Facts the steps rely on** (code and sources, 2026-10-13):
- Stopping the stack does **not** switch arm torque off: `arm_action_server` only closes the port on
  shutdown, the servos keep holding their last goal (TO-VERIFY on the robot; the dry run prints
  `torque` per servo).
- The EEPROM is written with **torque off only** — LeRobot's practice (`disable_torque` writes
  `Torque_Enable 0`, then `Lock 0`, before any EEPROM write; `SOFollower.calibrate/configure`,
  `feetech.py`, fetched 2026-10-13). A write under torque is untested, so `--write` refuses unless
  `Torque_Enable` is 0 on every servo.
- Torque off with the stack stopped needs the tool: `--torque-off` (all six servos, nothing else).
  Why not Space: on a stationary arm Space does not reliably cut torque (with the Theo-arm-holdgoal
  change only while a servo pushes), so it needs a deliberate gripper motion first (`O`, then Space
  while it runs) — an extra motion plus a timing race, only to reach a
  state the tool sets deterministically, once the node is off the bus.
- A goal outside the new limits is most likely **clamped** by the servo (TO-VERIFY; consistent with
  limb1 stopping at ~+102 deg = its current max register 3232). At torque-on the node writes goal :=
  present, so a servo lying outside its limits **jumps to the limit at full speed** (the speed
  registers are ignored). `--write` therefore refuses unless every present position — measured with
  the arm limp in its rest pose — lies inside the new range.
- Worst-case clamp jumps at a torque-on with the decided margins (servo lying on its mechanical stop):
  limb1 high side 145 ticks (~12.7 deg), limb5 low side 117 (~10.3 deg), limb5 high side 48
  (~4.2 deg), every other side ≤ 13 ticks (~1.1 deg).

**Preconditions.** (a) Branch with the hardened tool deployed and built on the Pi. (b) Margins:
`eeprom_margin_min_ticks` / `eeprom_margin_max_ticks` in `gripperx_arm/config/arm_safety_filter.yaml`,
user decision 2026-10-13 (rule and expected EEPROM ticks in the comment there; TO-VERIFY until this
session). (c) `calibration_verified: true`. (d) Drive not commanded; nobody touches the stop button.
(e) One person at the arm from step 3 until step 8 has passed.

All Pi commands run in one shell on the Pi (`until ssh gripperx; do sleep 5; done`), after:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws/Software/ros2/install/setup.bash
```

1. **Enforce first, in the repository (laptop):** set `eeprom_check: enforce` in
   `Software/ros2/src/gripperx_arm/config/arm_safety_filter.yaml` and note the planned session in the
   comment above it (date, margins); commit; the user pushes to the deploy remote (`git push pi Theo`).
   On the Pi, with the stack still running (a build replaces installed files only, the running node
   is unaffected):
   ```bash
   cd ~/ws && git pull
   cd ~/ws/Software/ros2 && colcon build --packages-select gripperx_arm
   grep -n "eeprom_check" ~/ws/Software/ros2/install/gripperx_arm/share/gripperx_arm/config/arm_safety_filter.yaml
   ```
   The last line must show `eeprom_check: enforce`. **Start the session right after this step:** from
   here any node start before a successful write — a respawn included — refuses torque-on.

   **Session aborted** before a successful write and power cycle (step 8 not passed): revert this
   step before the next start of the stack. Laptop: set `eeprom_check: log` again in
   `Software/ros2/src/gripperx_arm/config/arm_safety_filter.yaml`, commit, then
   `git push pi Theo` (the user). On the Pi:
   ```bash
   cd ~/ws && git pull
   cd ~/ws/Software/ros2 && colcon build --packages-select gripperx_arm
   grep -n "eeprom_check" ~/ws/Software/ros2/install/gripperx_arm/share/gripperx_arm/config/arm_safety_filter.yaml
   ```
   The last line must show `eeprom_check: log`.
2. **Computed values, no bus access:** `ros2 run gripperx_arm arm_eeprom_limits`. Every row without a
   `NOT EXPRESSIBLE`/`negative margin` note; the printed ranges equal the comment in the yaml.
3. **Stop the stack** (approved step; the arm keeps holding; the person stands at the arm):
   ```bash
   sudo /usr/local/bin/gripperx-stack-stop --mode=stop
   systemctl is-active gripperx-bringup.service    # must print: inactive
   pgrep -a -f arm_action_server                   # must print nothing
   ```
4. **Torque off** (separately approved step — **the arm goes limp at once**): the person holds the
   forearm, then:
   ```bash
   ros2 run gripperx_arm arm_eeprom_limits --port /dev/arm_servo --torque-off
   ```
   Expected `torque off: OK on all servos - the arm is limp`. On `TORQUE-OFF FAILED` or
   `Torque_Enable != 0`: keep holding the arm (forearm, near the wrist) and run the same command once
   more; if it still fails, cut main power with the arm held, lower it, and restart from step 3 after
   power-on. The person lowers the arm into its rest
   pose, with **limb1 and limb5 near their centre** and **limb2 not pressed onto its stop**. The arm
   stays like this until step 8.
5. **Dry run** (reads only; refuses while the unit is active, a node runs or the port is held):
   ```bash
   ros2 run gripperx_arm arm_eeprom_limits --port /dev/arm_servo | tee ~/arm_eeprom_dryrun_$(date -u +%Y%m%dT%H%M%SZ).txt
   ```
   Do the stop cross-check of the decision point below first. Compare per servo: `offset` = the
   value recorded in `homing_offsets_raw` (otherwise the decision point below); `torque 0`; `present` inside
   `computed` — this **is** the measurement of the rest pose the next boot starts from; if a value is
   outside, move that joint inward by hand and repeat; `servo` column equal to the rollback set below
   (if not, the printed values are the new rollback set — keep the file).
6. **Write** (approved step):
   ```bash
   ros2 run gripperx_arm arm_eeprom_limits --port /dev/arm_servo --write | tee ~/arm_eeprom_write_$(date -u +%Y%m%dT%H%M%SZ).txt
   ```
   Expected: `previous registers saved to …` then `written; read-back OK`, exit 0. `WRITE REFUSED`
   wrote nothing — fix the listed cause. `WRITE FAILED` / `MISMATCH` / `Lock != 1`: do not restart;
   repeat the dry run to see the state, then repeat the write or roll back. The read-back shows what
   the servo holds now, not yet that it survives a power cycle (step 7). Record the written values in
   the yaml comment of step 1 (next commit).
7. **Power cycle the servos** (approved restart): Mode H (long press: stack down, Pi halts; from the
   shell: `sudo /usr/local/bin/gripperx-stack-stop --mode=halt`), wait for the Pi to halt, main power
   off for 10 s, power on. The arm stays in its measured rest pose — limb1 and limb5 near centre,
   limb2 off its stop — and **the person keeps a hand on the arm through the boot's torque-on** (the
   worst-case clamp jumps above apply to any servo lying outside its range). Mode R (short press)
   keeps servo power and does not prove the EEPROM kept the values (RAM vs EEPROM, TO-VERIFY); whether
   the main switch cuts the arm servo supply is TO-VERIFY.
8. **Acceptance** (laptop):
   ```bash
   until ssh gripperx 'journalctl -u gripperx-bringup -b --no-pager | grep -E "SR-17 L1 EEPROM|SR-17 rule 2|Torque ON|Torque NOT|torque-on|TORQUE|not acknowledged|unreadable"'; do sleep 5; done
   ```
   Expected, in order: `SR-17 L1 EEPROM angle limits (min, max) per servo: {…}` with the step-6 values,
   `factory range on none`, `eeprom_check: enforce`, the `Homing_Offset` and `Lock` per servo;
   `Torque NOT enabled at start: … EEPROM limits not verified yet` (normal: the check waits for the
   robot description); then `SR-17 rule 2: EEPROM limits match the computed L1 values … torque-on
   allowed` and **no** later `torque-on refused`, `torque-on not acknowledged`, `TORQUE OFF` or
   `unreadable` line. A `SR-17 rule 2: EEPROM limits mismatch` line names the servo and the remedy
   (re-write, or — for unreadable registers — the bus). The fix is done when the user has confirmed
   it on the robot.

**Decision point — `Homing_Offset`.** The follower carries non-zero offsets, logged at boot
2026-10-13 17:13:34 CEST: raw `[3624, 3353, 1323, 1748, 3325, 4055]` (sign-magnitude, sign bit 11:
raw 3624 = -1576) — ids 1-5 identical to the leader arm, so one LeRobot calibration was written to
both. They are recorded as calibration data (`homing_offsets_raw` in `arm_calibration.yaml`);
`--write` and `eeprom_check: enforce` accept exactly these values and refuse any other. Why they must
never change: the node reads only `Present_Position` (sign-decoded) and never applies an offset
itself; per LeRobot `Present_Position = Actual_Position - Homing_Offset` (`servo_bus.py`
docstring), and the 2026-10-08 `zero_ticks` and stop sweeps were measured with these offsets in
place. **Frame:** limb1 stalled at present ~3232 = its `Max_Position_Limit` register, so the servo
compares the limits in the present (post-offset) frame — the frame the tool computes in.

**Stop cross-check (first part of step 5; TO-VERIFY until done):** with the arm limp (after step 4),
move **one** limb by hand onto **one** mechanical stop and run the step-5 dry run: that servo's
`present` must lie within a few ticks of `gripperx_arm/calibration/2026-10-08/stop_extremes.csv`
(e.g. limb3 lower stop: 3082-3083). If it matches, lower the arm back into its rest pose and continue
with step 5. If it does not, or if the dry run shows offsets other than the recorded ones: **stop,
write nothing, change no offset, report** — the calibration no longer matches the servos.

**Follow-up (user decision 2026-10-13, after the Theo-arm-holdgoal branch merges):** the node refuses
torque-on for a servo whose present position lies outside its EEPROM registers, instead of letting
the servo clamp. Not built yet.

**Rollback.** Read from the servos at node start 2026-10-12 17:43 and, unchanged across a boot,
2026-10-13 10:14 (likely left by the original LeRobot setup; id 5 = factory full range):

```bash
cat > ~/arm_eeprom_rollback_2026-10-12.yaml <<'EOF'
eeprom_limits:
  1: [760, 3232]
  2: [823, 3199]
  3: [873, 3076]
  4: [865, 3188]
  5: [0, 4095]
  6: [1293, 2810]
EOF
ros2 run gripperx_arm arm_eeprom_limits --port /dev/arm_servo --write --restore ~/arm_eeprom_rollback_2026-10-12.yaml
```

Same preconditions as step 6 (stack stopped, torque off, person at the arm, present inside the
restored ranges). Every `--write` also saves the registers it found (`previous registers saved to
…`), which `--restore` takes the same way. After a rollback set `eeprom_check: log` again (step 1 in
reverse) before the restart, or enforce keeps the arm without torque.

---

## 6. Cross-references

- The internal watchdog-deploy runbook (tracked internally, not in this repository) — command-watchdog
  deployment runbook; assumes this document's clean-teardown procedure for the actual restart step.
- The internal requirements document — OP-23, OP-24/S1, OP-27, FR-13, SR-2 (tracked internally, not
  in this repository) — normative text behind §2.4 (no software recovery for steering while
  `controller_manager` is wedged; power-cycle + hand straightening is the accepted, documented
  fallback).
- The internal FR-wheel on-site repair checklist (tracked internally, not in this repository) —
  assumes this document's agent-service stop/zombie-check discipline before flashing firmware.
- the internal digital-twin plan §9 — the simulation-side analogue of this document (sim cleanliness
  protocol, orphan-process cleanup, domain isolation). Different failure modes, same underlying
  discipline (process/transport hygiene on a laptop/Pi with no sandboxing between sessions).
- `documentation/ASBUILT.md` — as-built hardware state; the canonical source, not this file (see the
  banner above for this document's own current-state claim).
- The internal safety audit, §6.4 (items 2 and 6), §6.6 (tracked internally, not in this
  repository) — the audit findings §3 above records in deployer terms; §6.4 is the normative
  pre-real-robot list, §6.6 is the SIMULATION_DOMAIN_IDS ruling. A derived safety-and-known-
  limitations summary is published on the team wiki System Description page.
- The internal requirements document, FR-12 §10.1 (items 3 and 7) — the normative extract of the
  internal safety audit's §6.4 within the requirements document (neither in this repository);
  tracks whether these items are still owed before the real robot.
- The Nav2 integration track and `documentation/ASBUILT.md` — the source for §4 above and for the
  geometry/tuning changes (wheel radius, lever arm, meshes, footprint, tolerances, recoveries) this
  document deliberately does not restate. The code merged into `Theo` on 2026-08-21.

---

## 7. Features added since 2026-09-30 — what the operator and deployer must know

One short entry per feature; behaviour, parameters and test records live in the linked package README or
config, not here. Dates are the dates in the source; "deployed" is not "user-accepted" — the acceptance
state is in the internal requirements document (tracked internally, not in this repository).

### 7.1 Whole-drive fault, operator clear (SR-18 — safety requirement 18, drive fault on a missing encoder or persistent stall)
- A drive fault forces all four wheel commands to 0.0 and is kept in the marker file
  `/run/gripperx/drive_fault` (tmpfs; `RuntimeDirectory=gripperx` with `RuntimeDirectoryPreserve=yes` in
  `gripperx-bringup.service`). A crash-respawn of `ros2_control_node` re-adopts it, so the robot stays stopped.
- **Deliberate start clears it:** `gripperx-bringup.sh` removes the marker on an explicit start
  (`systemctl start`/`restart`, the Mode R button, `gripperx-stack-stop --mode=restart`), but **keeps** it on
  systemd's automatic `Restart=on-failure` (decided by `NRestarts`; if the counter cannot be read the marker is
  kept). Manual clear: `rm /run/gripperx/drive_fault`.
- **Operator clear without a restart:** service `/swerve_controller/clear_drive_fault`
  (`gripperx_control_msgs/srv/ClearDriveFault`) and the web teleop's clear button (with a confirm dialog).
  Only the `STALL_*` reasons are clearable; `ENCODER_MISSING` and `FEEDBACK_LOST` need a stack restart.
  **A clear is a motion release** — a drive command that is still held drives off at once — so it needs the
  same approval as any drive test (SR-1). Details: `Software/ros2/src/gripperx_swerve_controller/README.md`.
- A bench run (mock firmware) shares the marker path with the service; override `drive_fault_marker_path`
  with `""` to keep them apart (`Software/ros2/src/gripperx_hardware_interface/INTERFACE.md`).

### 7.2 Aimed pick (FR-16 — adaptive grasp; SR-16 — armed automatic `/pick_plastic`)
- `PickPlastic` with `aimed: true` grasps at the target's (x, y) at the taught grip height, gripper vertical,
  wrist roll searched; the fixed P sequence stays the default of the action
  (`Software/ros2/src/gripperx_arm_msgs/action/PickPlastic.action`).
- Gateway mode `grasp.aimed_pick` (startup-only, in `octopus_link_*.yaml`): `off`, `confirm_by_rearm`
  (manual: the arm previews, the gateway disarms, the operator's next `SetArming` approves exactly that pick)
  or `automatic`. In `octopus_link_real.yaml` the mode is `automatic` and `auto_pick` is `true`
  (user decisions 2026-11-16, assumptions marked TO-VERIFY in the file); the twin keeps `auto_pick: false`.
  Every arming still carries SR-16's assertion that the arm workspace is clear, and a person watches
  automatic picks.
- Standoff: `grasp.offset_x_m` (one value for every pick, there is no separate aimed value; current value in
  the two config files). Arm side: `aimed.region_margin_m`, `aimed.pregrasp_height_m` (steep approach from
  above, then a vertical descent) and related keys in
  `Software/ros2/src/gripperx_arm/config/arm_safety_filter.yaml`; the values there are marked TO-VERIFY
  where they are not measured.
- Nav2 arrival: `goal_checker.xy_goal_tolerance` is 0.05 m and the RotateToGoal window
  `FollowPath.xy_goal_tolerance` 0.04 (`Software/ros2/src/gripperx_planning/config/nav2.yaml`; invariant: the
  window stays at or below the goal checker). Whether the base settles inside 0.05 is open per the file's
  own comment.
- Space aborts a running P; `PickPlastic` refuses a goal whose stop epoch is not the current one.

### 7.3 Octopus list-loss hold (OCCHOLD — occlusion/list-loss hold)
`octopus_link_real.yaml` (and the twin file) carry `correlation_list_loss_hold_sec` (startup-only): an
empty or stale target list does not cancel the goal in flight for that many seconds after the mission's last
unique correlation, as long as no grasp was requested. The arrival gate may pick on an empty list inside the
window, never on a stale one; the acknowledgement never holds. Absent or TO-VERIFY disables the hold. Current
value: the config files.

### 7.4 Leader arm (FR-17 — leader-arm teleoperation)
The laptop-side leader reader and the follower's leader mode are in `Software/ros2/src/gripperx_leader/README.md`
(servo ids 1-6, the gripper id 6 included since 2026-11-13; leader EEPROM handling in its README). Leader mode ends on a second `J`, Space, `O`, `I` or `P`. Known open item: returning home after leaving leader
mode while the follower grips an object squeezes the object, so open the leader gripper before pressing `J`
(open item noted 2026-11-17, not yet in the README).

### 7.5 IMU, LiDAR stamp and yaw alarm
- `real_robot.launch.py` starts the BNO085 driver (`use_imu`, default `true`; `false` on a bench without the
  sensor). The EKF fuses only its yaw rate (`imu0`, via `localization_input_node`
  `/imu/bno085/data` → `/imu/data/filtered`); wheel odometry yaw rate is the fallback
  (`Software/ros2/src/gripperx_localization/config/localization.yaml`). Hardware: `ASBUILT.md` §1.5.
- Launch argument `lidar_stamp_source`: `mid_sweep` (default), `first_ray` or `publish` (old loop-tick stamp).
  On 2026-11-17 the stack on the Pi was still running `publish`, to change at its next Mode R (shutdown-button
  short press: restart of the ROS stack); not re-measured since, so verify the running value before relying on it.
- `odom_divergence_monitor` reports a two-stage yaw alarm (encoder yaw rate vs. gyro; decision D17 — values in
  `localization.yaml`, marked TO-VERIFY there) on `/diagnostics` and on the latched topic `/localization/yaw_alarm` (TRANSIENT_LOCAL, so a late
  subscriber still gets it; the web teleop shows it): WARNING does not latch, ERROR latches
  until the stack restarts. It commands nothing.

### 7.6 Front camera and web teleop camera tile
- `gripperx-camera.service` runs `ros2 launch gripperx_sensors camera.launch.py` (script
  `gripperx-camera.sh` sources ROS itself, domain 20, FastRTPS UDP-only profile). Lazy capture: the camera
  device is open only while `/camera/front/image_raw/compressed` or `camera_info` has a subscriber.
  Profiles `view`/`detail` are runtime-switchable (`Software/ros2/src/gripperx_sensors/README.md`).
- Web teleop: camera tile in the right column, default OFF (`Software/ros2/src/gripperx_teleop/docs/TELEOP_WEB_UI.md`,
  "Camera tile"). The camera stream and profile route have no authentication, like the drive controls.
- On the laptop, `ros2` CLI queries for the Pi's nodes may need `--spin-time 10` (reported 2026-11-17;
  cause not analysed).

### 7.7 End state after a stack stop (`gripperx-stack-stop`)
Steering is left limp at its last angle (no parking or centring move). The **arm is not limp**: the stop
only closes the serial port, so the servos keep torque and hold their last goal until their supply is cut
(from the code, corrected in the script 2026-10-13; reading `Torque_Enable` after a stop to confirm is
TO-VERIFY on the robot). Do not assume a centred geometry or a known arm pose at the next start.
