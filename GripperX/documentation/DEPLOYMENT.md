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


**Status:** binding operational runbook for the real robot; see the banner below for current state.

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
>   which is the origin of this caution, but that state is gone: `journal/archive/2026-09-21_28b109e6.md:99`
>   records a session verifying on the machine that bringup is **systemd-owned**, `MainPID 832`,
>   single instance inside the unit cgroup. Re-measured by the coordinator on **2026-09-23 19:36
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
(`CLAUDE.md` §11).

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

---

## 1. Clean-teardown bringup restart procedure (binding)

This is the single most important operational procedure for restarting the real robot. It resolved
a confirmed root cause — controller-manager spawners dying after a watchdog deploy, traced to
DDS-restart zombies: a duplicate/competing micro-ROS agent on `/dev/esp32` — and is **binding for
every future bringup restart**.

**Never restart `gripperx-bringup.service` (or do a full stack restart) without this sequence:**

1. **Stop services** — in reverse dependency order (navigation → mapping → bringup → agent), via
   the normal `systemctl stop gripperx-*.service` path.
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
     the internal journal, neither in this repository). Check
     with `ps aux | grep keyboard_teleop` on the laptop before touching the robot.
   - `ros2 daemon stop` — clears any stale ROS 2 daemon discovery state left over from the previous
     session, independent of the process-level zombie check above.
5. **Start in strict order, waiting for each stage to fully come up before starting the next:**
   `agent` → `bringup` → `mapping` → `navigation`.

   **This order is yours to keep — systemd only enforces part of it.** Checked against the unit
   files 2026-08-24: `gripperx-mapping.service` has `Requires=`/`After=gripperx-bringup.service`
   (+`ExecStartPre=/bin/sleep 20`) and `gripperx-navigation.service` has
   `Requires=`/`After=gripperx-mapping.service` (+`sleep 15`), so the last three are chained.
   **`gripperx-agent.service` is not in that chain at all** — it depends only on `docker.service`
   and the network, and nothing makes `bringup` wait for it. On a boot the agent and the bringup
   race. Starting the agent first by hand, as this step says, is therefore a real instruction and
   not a restatement of what the units already guarantee.

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
- **Status of the constraint, confirmed 2026-08-21, not re-verified since.** The Octopus team
  confirmed on 2026-08-21 that rosbridge runs on host `ITQLM125` at `ws://10.42.0.158:9090`, bound
  `0.0.0.0`, version **2.0.7 built from source**, with the three globs above, and that verification
  steps 5a–5d passed (`OCTOPUS_ROSBRIDGE_SETUP.md` §8). **Nobody has re-checked this endpoint since,
  and the shared subnet has moved twice in the meantime (`LOCAL_ENV.md` §2) — do not treat this
  address as current without re-verifying what is actually running on the Octopus host.** That
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
  and journal (`journalctl -u <octopus-link-service>`) directly after every deploy or restart. Do not
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

## 5. Cross-references

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
