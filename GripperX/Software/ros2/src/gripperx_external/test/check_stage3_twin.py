#!/usr/bin/env python3
"""Stage-3 acceptance against the mock world. Twin domain, nothing moves.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    ROS_DOMAIN_ID=221 python3 src/gripperx_external/test/check_stage3_twin.py --scenario all

WHAT IT VERIFIES, AND WHY EACH ONE EXISTS
=========================================
``dispatch``   the happy path end to end: fake Octopus -> rosbridge -> link node
               -> gateway -> Nav2 -> pick -> ``trash_goal_done`` -> the fake
               advances. Proves the acknowledgement reaches the counterpart and
               is accepted by THEIR parser, not merely published.
``no_ack``     the two refusals that matter most (SAFETY.md C-7): an arrival
               with a FAILED pick is not acknowledged, and an arrival while
               disarmed is not acknowledged. Both are irreversible if wrong.
``ambiguous``  F-13. Two targets inside ``merge_radius_m`` make the goal fix
               undecidable, and the gateway refuses rather than guessing - at
               dispatch, for the goal ALREADY IN FLIGHT, and at the
               irreversible acknowledgement. Runs past arrival, deliberately:
               the earlier version of this scenario ended ~22 s before it and
               that is what hid F-13 (F-6 in its second shape).
``link_reset`` F-16. A WebSocket reconnect keeps the blacklist; only evidence
               that their id space restarted drops it.
``triggers``   C-8: SEVEN of the NINE auto-disarm triggers of SR-15 rule 7,
               exercised WITH A GOAL IN FLIGHT. This was structurally impossible
               before stage 3 and is the reason stage 3 needed an audit first.
               The other two - `CLOCK_STALLED` (8) and `CLOCK_JUMPED_BACK` (9) -
               need a `/clock` publisher to misbehave and are therefore in
               ``clock``, parts B and E, where their CODES are asserted.
``sr9``        C-8: the SR-9 publisher inventory taken in the ARMED state with a
               goal in flight. A-5 only ever covered the disarmed inventory of a
               build that had no action client.
``clock``      F-24, in BOTH clock modes, and since revision 4 also F-29,
               F-30 and F-31 - the three ways a clock misbehaves without
               stopping; since revision 5 also F-37 and F-38 (part H), the two
               wall-clock statements that were left on the ROS clock when the
               others were moved off it. Sim time WITH a `/clock` publisher - the mode the
               launch file produces - dispatching and expiring normally; the
               same stack with the clock FROZEN under a goal in flight; sim time
               with no `/clock` at all, which is where the auditor measured zero
               timer callbacks in 40 s while the node reported itself healthy;
               the clock RUNNING AT 10% of real time; the clock JUMPING BACK
               120 s; and ordinary starts in both modes and both publisher
               orders, which must raise no ERROR at all.
``clock_fwd``  F-40, and it ASSERTS ALMOST NOTHING on purpose: a forward
               clock jump is recorded as OPEN pending a user decision, so this
               scenario measures what the build does and prints it as `[OBS ]`
               rather than freezing today's behaviour into a check. See
               `scenario_clock_forward`.
``stale``      F-28. Their detector goes silent while their datum and goal keep
               flowing, so the LINK stays healthy and the correlation input
               freezes. The three F-13 gates must refuse on the stale list
               instead of re-asking it and getting the same confident answer.

HOW IT OBSERVES
===============
By reading the nodes' own logs, deliberately. ``ros2 topic echo`` and
``ros2 node info`` generate DDS traffic of their own and this suite kills
processes on purpose; the SR-9 check is the one place a ``ros2`` CLI call is
worth its cost, and it is taken once, at a controlled moment.

EVERY NUMBER HERE IS A FIXTURE
==============================
The geofence rectangle, the mock's timings and the robot start pose are test
fixtures chosen to make the decision logic observable. NONE of them is a
measurement and none may be copied into a config file as one. ``grasp.offset_x_m
= 0.360`` is passed through exactly as the user specified it and is still a
SPECIFICATION, not a measurement.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)

TWIN_DOMAIN = "221"
NS = "/gripperx/external"

#: NOT A MEASUREMENT. A rectangle big enough to contain the fake's targets, so
#: that the geofence stops being the thing under test.
FIXTURE_GEOFENCE = ("-8.0", "8.0", "-8.0", "8.0")

_failures: List[str] = []
_procs: List["Proc"] = []


def check(condition: bool, label: str, detail: str = "") -> bool:
    mark = "PASS" if condition else "FAIL"
    print(f"  [{mark}] {label}" + (f"  ({detail})" if detail else ""), flush=True)
    if not condition:
        _failures.append(label)
    return bool(condition)


def readable(blob: str, what: str) -> bool:
    """Did the wire read produce anything? If not, say whose fault that is.

    Everything else in this suite is observed through the nodes' own LOGS,
    which are a file on disk and cannot fail to be readable. A handful of facts
    exist only on the wire - `ArmingState.expires_at` (F-37) and the
    `/diagnostics` values (F-37/F-40) - and those need `ros2 topic echo`, whose
    success on this laptop depends on CLI discovery while ANOTHER heavy DDS
    session shares the machine (see `shm_clean`: a full run met 198 `/dev/shm`
    segments belonging to live processes of the other session, which
    `fastdds shm clean` correctly refuses to touch).

    So an unreadable topic is a statement about the harness and not about the
    node, and asserting on it would put a red check against the gateway for
    something the gateway did correctly. The content of those messages IS
    proven deterministically, offline, in `check_validation.py` parts 7 and 8,
    which call the real `clock_status`/projection with no DDS at all. What is
    lost when this returns False is the end-to-end link between the two, and
    that loss is printed rather than swallowed.
    """
    if blob.strip():
        return True
    print(f"  [WARN] harness: {what} could not be read off the wire "
          "(ros2 CLI discovery), so the check below is NOT run - this is not a "
          "result about the gateway; the same content is asserted offline in "
          "check_validation.py parts 7 and 8", flush=True)
    return False


def observe(label: str, detail: str = "") -> None:
    """Record a MEASUREMENT without passing judgement on it.

    Used only by the F-40 probe. That finding is open for a USER DECISION, so
    the suite must be able to state what the build DOES without asserting that
    it is what the build SHOULD do - a `check` here would freeze today's
    behaviour as the requirement, which is precisely the decision nobody has
    taken. Nothing printed by this function can reach `_failures`.
    """
    print(f"  [OBS ] {label}" + (f"  ({detail})" if detail else ""), flush=True)


class Proc:
    """A child process whose stdout is a file we can grep for evidence."""

    def __init__(self, name: str, argv: List[str], env: dict, stdin_pipe: bool = False):
        self.name = name
        # Per-SCENARIO subdirectory. With one flat directory each scenario
        # overwrote the previous one's node logs, so after a full run the only
        # log left was the last scenario's - and a failure in an earlier one
        # could not be diagnosed at all afterwards. Found exactly that way.
        directory = os.path.join(LOG_DIR, CURRENT_SCENARIO or "scenario")
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, f"{name}.log")
        self.file = open(self.path, "w+b")
        # OWN PROCESS GROUP, and the teardown kills the GROUP. `ros2 run` is a
        # wrapper that execs the real node as a child, so signalling the wrapper
        # alone leaves the node running - which is how an earlier run of this
        # file ended up with five orphaned link nodes on one domain, all
        # publishing link_status, all interleaving their reconnect counters.
        # Orphaned motion sources are what SR-5 is about; here they merely
        # falsify a test, but they falsify it silently.
        self.proc = subprocess.Popen(
            argv,
            stdout=self.file,
            stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE if stdin_pipe else subprocess.DEVNULL,
            env=env,
            start_new_session=True,
        )
        self._read = open(self.path, "r", errors="replace")
        _procs.append(self)

    def text(self) -> str:
        self._read.seek(0)
        return self._read.read()

    def send(self, line: str) -> None:
        assert self.proc.stdin is not None
        self.proc.stdin.write((line + "\n").encode())
        self.proc.stdin.flush()

    def wait_for(self, pattern: str, timeout: float = 20.0) -> Optional[str]:
        deadline = time.time() + timeout
        rx = re.compile(pattern)
        while time.time() < deadline:
            for line in self.text().splitlines():
                if rx.search(line):
                    return line
            time.sleep(0.2)
        return None

    def count(self, pattern: str) -> int:
        return len(re.findall(pattern, self.text()))

    def signal_group(self, sig) -> None:
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def stop(self, sig=signal.SIGINT) -> None:
        if self.proc.poll() is None:
            self.signal_group(sig)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.signal_group(signal.SIGKILL)
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
        # The wrapper can exit while the node it exec'd is still alive.
        self.signal_group(signal.SIGKILL)


def env_for() -> dict:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = TWIN_DOMAIN
    # Keep the whole suite on this machine. It kills processes on purpose.
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["PYTHONUNBUFFERED"] = "1"
    return env


def start_fake(extra: Optional[List[str]] = None) -> Proc:
    return Proc(
        "fake_octopus",
        [sys.executable, os.path.join(_HERE, "fake_octopus.py"), "--port", "9090"]
        + (extra or []),
        env_for(),
        stdin_pipe=True,
    )


def start_mock(**params) -> Proc:
    argv = [sys.executable, os.path.join(_HERE, "mock_motion_servers.py"), "--ros-args"]
    for key, value in params.items():
        argv += ["-p", f"{key}:={value}"]
    return Proc("mock_motion", argv, env_for())


#: The installed entry points, invoked DIRECTLY rather than through `ros2 run`.
#: `ros2 run` execs the node as a child, so a signal sent for a test lands on
#: the wrapper as well and the exit code measured is the wrapper's, not the
#: node's - which is how "SIGTERM is a clean exit" came back as rc=-15 for a
#: node whose own log showed a clean, complete shutdown.
_INSTALL = os.path.abspath(
    os.path.join(_PKG, "..", "..", "install", "gripperx_external", "lib", "gripperx_external")
)


def start_sim_clock(rate_hz: float = 50.0, scale: float = 1.0,
                    epoch_mode: str = "wall",
                    epoch: Optional[float] = None) -> Proc:
    """Gazebo's clock, without Gazebo. See `test/sim_clock.py`.

    ``scale`` is the real-time factor: 1.0 is a Gazebo keeping up, 0.1 is a
    loaded one and is the case SAFETY.md F-29 is about. Measured against a real
    Gazebo on 2026-08-20: 16x CPU oversubscription produced 0.083, so 0.1 is a
    fair model of the thing it stands in for.

    ``epoch_mode`` is WHERE SIM TIME STARTS, and it is a test dimension in its
    own right since 2026-08-21: ``wall`` (the default, and what every scenario
    here has always used) seeds it at wall time, which is a BAG REPLAY;
    ``gazebo`` seeds it at 0.0, which is what a real Gazebo does. The two are
    ~1.787e9 seconds apart and behave differently downstream - see
    `sim_clock.py`'s docstring and SAFETY.md F-35.
    """
    argv = [sys.executable, os.path.join(_HERE, "sim_clock.py"),
            "--rate-hz", str(rate_hz), "--scale", str(scale),
            "--epoch-mode", epoch_mode]
    if epoch is not None:
        # An EXPLICIT seed, for the band between the two modes: a replay of an
        # hour-old bag is neither "at zero" nor "at wall time", and the two
        # modes alone cannot say what happens in between.
        argv += ["--epoch", str(epoch)]
    return Proc("sim_clock", argv, env_for())


def set_sim_clock(paused: bool) -> None:
    set_sim_clock_param("paused", "true" if paused else "false")


def set_sim_clock_param(name: str, value: str) -> bool:
    """Set one parameter on the clock fixture, and SAY WHETHER IT WORKED.

    This used to discard the result entirely - `DEVNULL` on both streams, no
    return code, no retry - so a `ros2 param set` that never found `/sim_clock`
    was indistinguishable from one that succeeded. That is not hypothetical: a
    full-suite run lost FOUR checks in the backwards-jump part to it, and every
    one of them was phrased as "the gateway did not report the jump" when what
    had actually happened was that the fixture was never told to jump. A silent
    setup failure attributed to the code under test is worse than a loud one.

    Retried, for the reason `echo_once` is retried: on a laptop that is also
    running another DDS session the CLI can fail to discover a node that is
    running perfectly (see `shm_clean`).
    """
    for attempt in range(3):
        result = subprocess.run(
            ["ros2", "param", "set", "/sim_clock", name, value],
            env=env_for(), capture_output=True, text=True, timeout=60,
        )
        if result.returncode == 0 and "Set parameter successful" in result.stdout:
            return True
        time.sleep(2)
    print(f"  [WARN] `ros2 param set /sim_clock {name} {value}` did not take "
          "effect - this is a HARNESS failure, not a finding about the gateway")
    return False


def echo_once(topic: str, timeout: float = 15.0, expect: str = "") -> str:
    """One `ros2 topic echo --once`, used sparingly and deliberately.

    The suite observes by reading logs (see the module docstring) because CLI
    calls make DDS traffic of their own. There are exactly two things that are
    only ON THE WIRE and nowhere in a log - `ArmingState.expires_at` (F-37) and
    the `/diagnostics` values (F-40's "is anything reported at all") - so those
    two are read here, once each, at a controlled moment.

    Retried, because the `ros2` daemon can be slow to discover a publisher on a
    busy laptop and an empty read looks exactly like "the node published
    nothing", which is the opposite conclusion.

    ``expect`` exists because ``/diagnostics`` HAS MORE THAN ONE PUBLISHER: the
    gateway and the link node both publish `DiagnosticArray` on it, and
    ``--once`` returns whichever message arrives first. A read that caught the
    link node's array contained no `external/clock` at all, and the check built
    on it failed while the gateway was behaving perfectly - which is a test
    reading the wrong message, not a finding. With ``expect`` set, several
    messages are read and the first one containing that string is returned.
    """
    for _ in range(3):
        if not expect:
            try:
                out = subprocess.run(
                    ["ros2", "topic", "echo", "--once", topic],
                    env=env_for(), capture_output=True, text=True, timeout=timeout,
                )
            except subprocess.TimeoutExpired:
                continue
            if out.stdout.strip():
                return out.stdout + out.stderr
            continue
        # A WINDOW rather than one message, then pick the publisher we mean.
        # `ros2 topic echo` separates messages with a `---` line.
        try:
            out = subprocess.run(
                ["ros2", "topic", "echo", topic],
                env=env_for(), capture_output=True, text=True, timeout=timeout,
            )
            captured = out.stdout
        except subprocess.TimeoutExpired as expired:
            captured = expired.stdout or ""
            if isinstance(captured, bytes):
                captured = captured.decode("utf-8", "replace")
        for chunk in captured.split("\n---\n"):
            if expect in chunk:
                return chunk
    return ""


def start_link(name: str = "octopus_link_node", **overrides) -> Proc:
    """The link node. ``use_sim_time`` defaults to false here for the same
    reason it does for the gateway - this harness has no Gazebo - but it is
    overridable, because the twin config sets it TRUE and the interaction
    between that and the gateway's monotonic link watchdog is a thing to test
    (SAFETY.md F-29).
    """
    params = {
        "use_sim_time": "false",
        "expected_domain_id": TWIN_DOMAIN,
        "goal_ingress_enabled": "true",
        "url": "ws://127.0.0.1:9090",
        "publish_telemetry": "true",
    }
    params.update(overrides)
    argv = [os.path.join(_INSTALL, "octopus_link_node"), "--ros-args",
            "-r", f"__ns:={NS}", "-r", f"__node:={name}"]
    for key, value in params.items():
        argv += ["-p", f"{key}:={value}"]
    return Proc(name, argv, env_for())


def start_gateway(name: str = "goal_gateway_node", **overrides) -> Proc:
    """Start the gateway. ``use_sim_time`` defaults to FALSE, and that is a
    statement about this harness rather than about the twin (SAFETY.md F-24).

    The twin proper runs Gazebo, which publishes ``/clock``, and
    ``octopus_link_twin.yaml`` says ``use_sim_time: true`` because of it. This
    suite runs against mocks, where there is no ``/clock`` unless
    ``sim_clock.py`` is started - so false is the honest value here, and it is
    passed explicitly so that nothing is inherited from a config file.

    That difference used to be invisible and unexercised: every green result in
    SAFETY.md revisions 2 and 3 was taken with sim time off while the launch file
    defaulted it on. The `clock` scenario is what closes that gap - it runs this
    same stack in sim time WITH a clock, with a frozen one, and with none.
    """
    params = {
        "use_sim_time": "false",
        "expected_domain_id": TWIN_DOMAIN,
        "goal_ingress_enabled": "true",
        "allow_arm": "true",
        "dry_run": "false",
        "auto_pick": "true",
        "arming.max_duration_sec": "600.0",
        "arming.max_consecutive_aborts": "3",
        "geofence.min_x_m": FIXTURE_GEOFENCE[0],
        "geofence.max_x_m": FIXTURE_GEOFENCE[1],
        "geofence.min_y_m": FIXTURE_GEOFENCE[2],
        "geofence.max_y_m": FIXTURE_GEOFENCE[3],
        # The user's specification, passed through unchanged. NOT a measurement.
        "grasp.offset_x_m": "0.360",
        "grasp.offset_y_m": "0.000",
        "grasp.tolerance_m": "TO-VERIFY",
        "max_attempts_per_target": "2",
        "cancel_confirm_timeout_sec": "3.0",
        "dispatch_rate_hz": "2.0",
        "link_lost_sec": "5.0",
    }
    params.update(overrides)
    argv = [os.path.join(_INSTALL, "goal_gateway_node"), "--ros-args",
            "-r", f"__ns:={NS}", "-r", "__node:=goal_gateway_node"]
    for key, value in params.items():
        argv += ["-p", f"{key}:={value}"]
    return Proc(name, argv, env_for())


def set_mock(**params) -> None:
    for key, value in params.items():
        subprocess.run(
            ["ros2", "param", "set", "/mock_motion_servers", key, str(value)],
            env=env_for(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=30,
        )


def set_param(node: str, name: str, value: str) -> str:
    """`ros2 param set` against one of our nodes, returning what it said.

    Used only to prove the REFUSAL of a startup-only parameter (SAFETY.md F-8).
    Nothing here relies on a parameter change taking effect.
    """
    out = subprocess.run(
        ["ros2", "param", "set", f"{NS}/{node}", name, value],
        env=env_for(), capture_output=True, text=True, timeout=60,
    )
    return out.stdout + out.stderr


def set_arming(arm: bool, duration: float = 120.0, by: str = "stage3-check") -> str:
    request = (
        f"{{arm: {'true' if arm else 'false'}, duration_sec: {duration}, "
        f"requested_by: '{by}'}}"
    )
    out = subprocess.run(
        ["ros2", "service", "call", f"{NS}/set_arming",
         "gripperx_external_msgs/srv/SetArming", request],
        env=env_for(), capture_output=True, text=True, timeout=60,
    )
    return out.stdout + out.stderr


def teardown() -> None:
    for proc in reversed(_procs):
        proc.stop()
    _procs.clear()
    # Belt and braces: verified by counting, not assumed. A leftover node on
    # this domain does not fail the next scenario, it CORRUPTS it.
    deadline = time.time() + 15
    while time.time() < deadline:
        alive = subprocess.run(
            ["pgrep", "-f", "gripperx_external/(goal_gateway_node|octopus_link_node)"],
            capture_output=True, text=True,
        ).stdout.split()
        alive += subprocess.run(
            # sim_clock.py belongs in this list for the same reason the others
            # do, and more so: an orphaned /clock publisher does not merely
            # falsify the next scenario, it silently supplies the very thing the
            # `clock` scenario proves the absence of.
            ["pgrep", "-f", "mock_motion_servers.py|fake_octopus.py|sim_clock.py"],
            capture_output=True, text=True,
        ).stdout.split()
        if not alive:
            return
        for pid in alive:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except (ProcessLookupError, ValueError):
                pass
        time.sleep(1)
    print("  [WARN] processes survived teardown; the next scenario is unreliable")


def shm_clean(announce: bool = False) -> None:
    """Clean leaked FastDDS shared-memory segments. See `scripts/shm_clean.sh`.

    WHY THIS IS IN THE SUITE'S SETUP AND NOT ONLY BETWEEN SCENARIOS
    ==============================================================
    This suite SIGKILLs processes on purpose - that is how the LINK_LOST and
    NODE_SHUTDOWN triggers are exercised - and a SIGKILLed FastDDS process
    cannot run its destructors, so it leaves its `/dev/shm/fastrtps_*` segments
    behind. At roughly 180 leftovers, newly started participants stop being
    visible TO THE `ros2` CLI while node-to-node discovery still works: the
    nodes talk to each other perfectly and `ros2 topic echo` returns nothing.

    That is not a hypothetical. A full eleven-scenario invocation spent two
    checks on it - a `/diagnostics` read in `clock_fwd` came back empty while
    the same read succeeded standalone - and the failure looks exactly like "the
    node published nothing", which is the opposite conclusion.

    Between scenarios was never enough on its own, because a run INHERITS what
    the previous run leaked. `main` therefore calls this before the first
    scenario as well, and prints the counts so that a suspicious result later
    can be read against how dirty the machine was at the start.

    Safe to run alongside other sessions: `fastdds shm clean` checks ownership
    and spares segments belonging to live processes, which matters because
    several worktrees share this laptop (domains 20 real / 220 / 221).

    HOW MUCH IT ACTUALLY RECLAIMS - MEASURED 2026-08-20 18:34 CEST
    =============================================================
    Conditions, because a reclaim figure without them is not comparable to the
    next one: machine otherwise QUIET, domain 220 IDLE (the parallel Nav2
    session had ended and cleaned up), none of this suite's own nodes running,
    two `ros2` CLI daemons alive (domains 20 and 221).

        126 entries before  ->  126 after.  ZERO reclaimed.

    An earlier measurement the same day, taken while the 220 session was live,
    read 284 -> 198 (43 zombie segments cleaned) and was recorded here as
    "the remainder belongs to other people's live processes". **That reading
    was too generous to the cleaner and is corrected:** with 220 idle and
    nothing of ours running, the remainder is almost entirely OURS and is
    unreclaimable anyway. Composition of the 126:

        4    participant segments (`fastrtps_<id>` + `_el`) - the two live
             `ros2` daemons, correctly spared;
        118  port files and their semaphores (59 `fastrtps_port<n>` + 59
             `sem.fastrtps_port<n>_mutex`) owned by NOTHING alive, the oldest
             timestamped 06:38 the same morning - 94% of the residue;
        4    other.

    So the honest statement is neither "this removes the exposure" nor "the
    residue is other sessions' live state". It is: **a clean removes recent
    participant zombies and does not touch a long-lived residue of orphaned
    PORT entries, which accumulate across sessions and outlive every process
    that made them.**

    What leaks, measured the same way, one participant at a time:

        clean exit (SIGINT)   126 -> 130 -> 126   leaks NOTHING
        SIGKILL               126 -> 130 -> 130   leaks 4 entries
        then `shm_clean`      130 -> 122          reclaims those 4, plus a
                                                  little older residue

    That is the case for calling this in setup: this suite SIGKILLs on purpose,
    ~4 entries per killed participant, and a clean immediately afterwards does
    reclaim them. It is also the limit of it - the port residue needs
    `rm -f /dev/shm/fastrtps_port*` with nothing running, which this suite must
    not do while other domains may be live.

    Discovery still worked at 126 entries (a freshly started node was visible to
    `ros2 node list`), so the ~180 figure in `scripts/shm_clean.sh` is not
    contradicted by any of the above - we were below it throughout.
    """
    script = os.path.join(_PKG, "..", "..", "scripts", "shm_clean.sh")
    script = os.path.abspath(script)
    if not os.path.exists(script):
        if announce:
            print(f"  [WARN] {script} is missing; leaked /dev/shm segments will "
                  "accumulate and can blind the ros2 CLI")
        return
    result = subprocess.run(["bash", script], capture_output=True, text=True,
                            timeout=60)
    if announce:
        summary = " / ".join(
            line.strip() for line in (result.stdout or "").splitlines()
            if "before:" in line or "after:" in line
        )
        print(f"  shm_clean: {summary or 'no counts reported'}")


# ===========================================================================
def scenario_dispatch() -> None:
    print("=" * 78)
    print("dispatch - the full loop, ending in an acknowledgement THEIR parser accepts")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=3.0, pick_duration_sec=1.0)
    link = start_link()
    gateway = start_gateway()

    check(
        gateway.wait_for(r"armed=False", 30) is not None,
        "the gateway comes up DISARMED with allow_arm true and dry_run false",
    )
    check(
        gateway.wait_for(r"navigate_to_pose is available", 40) is not None,
        "it discovers navigate_to_pose",
    )
    check(
        gateway.wait_for(r"pick_plastic is available", 20) is not None,
        "  ... and pick_plastic, so auto_pick_available becomes true",
    )
    time.sleep(3)
    check(
        gateway.count(r"DISPATCHING target") == 0,
        "NOTHING is dispatched while disarmed, however valid the goal is",
    )
    held = gateway.wait_for(r"is dispatchable but held: NOT_ARMED", 10)
    check(held is not None, "  ... and the gateway says so, naming the block")

    print(set_arming(True, 120.0).strip()[:0] or "  (armed)", flush=True)
    dispatched = gateway.wait_for(r"DISPATCHING target (\S+)", 20)
    check(dispatched is not None, "arming alone releases the dispatch", (dispatched or "")[-90:])
    check(
        gateway.wait_for(r"REACHED", 30) is not None,
        "Nav2 reports arrival and the gateway records it",
    )
    check(
        gateway.wait_for(r"SR-16: sending PickPlastic", 15) is not None,
        "SR-16: the pick is sent, while armed, after a successful navigation",
    )
    check(
        gateway.wait_for(r"ACKNOWLEDGED target (\S+) as COLLECTED", 25) is not None,
        "trash_goal_done is published AFTER the successful pick",
    )
    check(
        link.wait_for(r"forwarded trash_goal_done for target", 15) is not None,
        "  ... and the link node forwards it to the wire",
    )
    check(
        fake.wait_for(r"acknowledged as collected", 15) is not None,
        "  ... and the counterpart's own parser accepts it and advances",
    )
    check(fake.count(r"unparseable trash_goal_done") == 0, "  ... with no unparseable acks")
    teardown()


def scenario_no_ack() -> None:
    print("=" * 78)
    print("no_ack - the two refusals C-7 exists for")
    print("=" * 78)
    # 1. a FAILED pick must not acknowledge, and must not loop for ever
    fake = start_fake()
    mock = start_mock(nav_duration_sec=2.0, pick_duration_sec=0.5, pick_outcome="fail")
    link = start_link()
    gateway = start_gateway()
    gateway.wait_for(r"pick_plastic is available", 60)
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"pick FAILED for target", 60) is not None,
        "a failed pick is reported as a failure",
    )
    time.sleep(6)
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "NO acknowledgement follows a failed pick - arrival alone never acknowledges",
    )
    check(fake.count(r"acknowledged as collected") == 0, "  ... and the source never advances")
    check(
        gateway.wait_for(r"BLACKLISTING target", 60) is not None,
        "after max_attempts_per_target the target is blacklisted, not acknowledged",
    )
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "  ... and blacklisting still does not acknowledge",
    )
    teardown()
    shm_clean()

    # 2. disarmed before the pick result -> no acknowledgement
    print("-" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=2.0, pick_duration_sec=8.0, pick_outcome="succeed")
    link = start_link()
    gateway = start_gateway(name="goal_gateway_node_disarm")
    gateway.wait_for(r"pick_plastic is available", 60)
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"SR-16: sending PickPlastic", 60) is not None,
        "a pick is running",
    )
    set_arming(False)
    check(
        gateway.wait_for(r"disarmed by OPERATOR", 15) is not None,
        "the operator disarms while the arm is moving",
    )
    check(
        gateway.wait_for(r"CANCELLING target", 10) is not None,
        "  ... which CANCELS the running pick (SR-16 / SR-15 rule 8)",
    )
    time.sleep(12)
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "no acknowledgement is published from a closed gate, ever (C-7)",
    )
    check(
        gateway.count(r"zero|Twist|set_mode") == 0,
        "  ... and the disarm wrote nothing anywhere",
    )
    teardown()


def scenario_ambiguous() -> None:
    """SAFETY.md F-13, and the regression that hid it.

    The old version of this scenario dispatched with `nav_duration_sec=30`, made
    the fix ambiguous ~0.6 s later, waited 6 s and tore down - ending ~22 s
    BEFORE arrival. Its final check "and nothing is acknowledged either" passed
    because nothing had had time to arrive, not because anything had refused. On
    the same code with a 14 s navigation the auditor got an arrival, a pick and
    an irreversible `ACKNOWLEDGED target 3` for a fix that by then matched target
    4. A test that stops before the irreversible step tests nothing about it.

    So this scenario now runs PAST arrival in all three parts:
      A  the auditor's own timing - 14 s navigation, ambiguity at +0.6 s -
         observed until well beyond the moment the old build acknowledged;
      B  recovery: the ambiguity is removed and the same target completes,
         proving the refusal is a refusal and not a dead end;
      C  the ambiguity arrives while the ARM IS ALREADY PICKING, where by
         deliberate design nothing cancels - the arm is committed - and the
         acknowledgement is refused at the last gate instead.
    """
    print("=" * 78)
    print("ambiguous - F-13: undecidable at ANY point between dispatch and ack")
    print("=" * 78)

    # -- A: the auditor's reproduction, with the timing that exposed it ----
    fake = start_fake()
    mock = start_mock(nav_duration_sec=14.0, pick_duration_sec=1.0)
    link = start_link()
    gateway = start_gateway()
    gateway.wait_for(r"navigate_to_pose is available", 60)
    gateway.wait_for(r"pick_plastic is available", 30)
    set_arming(True, 300.0)
    dispatched = gateway.wait_for(r"DISPATCHING target (\S+)", 30)
    check(dispatched is not None, "a unique correlation dispatches normally")
    t_dispatch = time.time()

    # Their merge_radius_m is 0.25; the goal target sits at (0.30, -1.10). This
    # new one is 0.112 m from it AND closer to the datum, so it also becomes
    # THEIR goal - exactly the pair the auditor produced (4@0.000m, 3@0.112m).
    fake.send("add 0.40 -1.05")
    check(
        gateway.wait_for(r"GOAL_AMBIGUOUS", 30) is not None,
        "a second target 0.11 m away makes the goal fix undecidable",
    )
    line = gateway.wait_for(r"GOAL_AMBIGUOUS", 5) or ""
    check("Refusing rather than guessing" in line or "targets within" in line,
          "  ... and the refusal names both candidates", line[-100:])

    # THE FIX. The goal already in flight is re-correlated on every dispatch
    # tick and cancelled, rather than driven to arrival on a decision taken
    # 0.6 s earlier.
    cancelled = gateway.wait_for(r"CANCELLING target .*CORRELATION_LOST", 15)
    check(cancelled is not None,
          "the goal ALREADY IN FLIGHT is cancelled, not just the next one",
          (cancelled or "")[-120:])
    check(
        gateway.wait_for(r"navigation to target .* was CANCELLED", 20) is not None,
        "  ... and Nav2 confirms it ended as CANCELED",
    )

    # Now stay past the point at which the old build arrived and acknowledged.
    # nav_duration_sec is 14; this waits until at least t_dispatch + 25 s.
    remaining = (t_dispatch + 25.0) - time.time()
    if remaining > 0:
        time.sleep(remaining)
    check(
        gateway.count(r"REACHED") == 0,
        "no arrival happens at all - the drive was withdrawn before it",
    )
    check(
        gateway.count(r"SR-16: sending PickPlastic") == 0,
        "the ARM NEVER ACTUATES for a goal that stopped being nameable",
    )
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "and nothing is acknowledged - observed past the arrival time, not before it",
    )
    check(fake.count(r"acknowledged as collected") == 0,
          "  ... their side never advances")
    check(
        gateway.count(r"BLACKLISTING target") == 0,
        "a withdrawn goal is NOT counted against the target: no blacklisting",
    )

    # -- B: recovery. The refusal must not be a dead end -------------------
    fake.send("remove 4")
    redispatched = gateway.wait_for(r"DISPATCHING target 3", 30)
    check(redispatched is not None,
          "once the fix names one object again the SAME target is re-dispatched")
    check(
        gateway.wait_for(r"ACKNOWLEDGED target 3 as COLLECTED", 60) is not None,
        "  ... and completes normally, so the ambiguity cost a re-drive and nothing more",
    )
    teardown()
    shm_clean()

    # -- C: the ambiguity arrives while the ARM IS ALREADY PICKING ---------
    print("-" * 78)
    print("  C: ambiguity DURING the pick - the arm is committed, the ack is not")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=3.0, pick_duration_sec=12.0, pick_outcome="succeed")
    link = start_link()
    gateway = start_gateway(name="goal_gateway_node_pickwindow")
    gateway.wait_for(r"pick_plastic is available", 60)
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"REACHED", 60) is not None,
        "the goal is driven to ARRIVAL with the correlation still unique",
    )
    check(
        gateway.wait_for(r"SR-16: sending PickPlastic", 20) is not None,
        "  ... and the pick starts",
    )
    fake.send("add 0.40 -1.05")
    check(
        gateway.wait_for(r"GOAL_AMBIGUOUS", 20) is not None,
        "the fix becomes undecidable while the arm is moving",
    )
    check(
        gateway.wait_for(r"pick SUCCEEDED for target 3", 40) is not None,
        "the pick is NOT cancelled mid-motion - a committed arm is left committed",
    )
    refused = gateway.wait_for(
        r"was picked successfully, but its correlation no longer holds", 20
    )
    check(refused is not None,
          "  ... and the ACKNOWLEDGEMENT is refused at the last gate (F-13)",
          (refused or "")[-120:])
    check(
        gateway.wait_for(r"CORRELATION_CHANGED", 15) is not None,
        "  ... with CORRELATION_CHANGED as the recorded reason",
    )
    time.sleep(8)
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "NOTHING is acknowledged, although the pick itself succeeded",
    )
    check(fake.count(r"acknowledged as collected") == 0,
          "  ... and their side never advances on an object we cannot name")
    check(
        gateway.wait_for(r"BLACKLISTING target 3", 20) is not None,
        "  ... the target is blacklisted instead: the object is collected, so "
        "re-driving to it would repeat work that is physically done",
    )
    teardown()


def scenario_triggers() -> None:
    print("=" * 78)
    print("triggers - C-8: SEVEN of the NINE auto-disarm triggers, WITH A GOAL IN "
          "FLIGHT")
    print("           (CLOCK_STALLED=8 and CLOCK_JUMPED_BACK=9 need a misbehaving")
    print("            /clock and are asserted, by CODE, in --scenario clock B/E)")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=45.0)
    link = start_link()
    gateway = start_gateway(**{"arming.max_consecutive_aborts": "1"})
    gateway.wait_for(r"navigate_to_pose is available", 60)

    def in_flight(label: str, duration: float = 300.0) -> bool:
        before = gateway.count(r"DISPATCHING target")
        set_arming(True, duration)
        deadline = time.time() + 30
        while time.time() < deadline:
            if gateway.count(r"DISPATCHING target") > before:
                return True
            time.sleep(0.3)
        check(False, f"{label}: a goal could be put in flight")
        return False

    # 1 OPERATOR
    if in_flight("OPERATOR"):
        set_arming(False)
        check(gateway.wait_for(r"disarmed by OPERATOR", 15) is not None,
              "1/7 OPERATOR disarms with a goal in flight")
        check(gateway.wait_for(r"CANCELLING target .*disarm:OPERATOR", 10) is not None,
              "    ... and cancels it")
        check(gateway.wait_for(r"navigation .* was CANCELLED", 15) is not None,
              "    ... and Nav2 confirms the goal ended as CANCELED")

    # 2 TIMEOUT - a short window that expires while the goal is still running
    if in_flight("TIMEOUT", duration=6.0):
        check(gateway.wait_for(r"disarmed by TIMEOUT", 25) is not None,
              "2/7 TIMEOUT disarms when the window expires mid-goal")
        # Either reason is a pass, and which one appears is itself evidence.
        # `is_armed` is evaluated at READ time (SAFETY.md F-2 / C-2), so the
        # window is shut the instant it expires rather than when the poll next
        # runs - and the dispatch tick's in-flight re-validation can therefore
        # see NOT_ARMED and cancel BEFORE the safety tick emits the TIMEOUT
        # event. Observed: cancel at ...655.099 (NOT_ARMED), disarm event at
        # ...655.185. Two independent paths, both cancelling, neither waiting
        # for the other. Requiring only one of them would be testing the race.
        check(gateway.wait_for(r"CANCELLING target .*(disarm:TIMEOUT|NOT_ARMED)", 10) is not None,
              "    ... and cancels it, by whichever gate notices the expiry first")

    # 3 MODE_CHANGE - the spacebar E-stop's path
    if in_flight("MODE_CHANGE"):
        set_mock(teleop_mode="keyboard")
        check(gateway.wait_for(r"disarmed by MODE_CHANGE", 15) is not None,
              "3/7 MODE_CHANGE disarms when the mux leaves autonomous")
        # Same two-path shape as TIMEOUT above, and for the same reason: the
        # dispatch tick re-validates the goal in flight through
        # `validate_dispatch` and can see MODE_NOT_AUTONOMOUS before the safety
        # group's mode callback emits the MODE_CHANGE event. Both cancel; the
        # cancel is idempotent, so whichever arrives first is the one that logs.
        # Observed both orderings across runs (disarm:MODE_CHANGE at ...883.500,
        # 1.1 ms after the disarm, in one run; the dispatch gate first in
        # another). Pinning one of them would be testing the race, not the rule.
        check(gateway.wait_for(r"CANCELLING target .*(disarm:MODE_CHANGE|MODE_NOT_AUTONOMOUS)", 10) is not None,
              "    ... and cancels it, by whichever gate notices the mode first")
        set_mock(teleop_mode="autonomous")
        time.sleep(2)

    # 4 NAV2_UNAVAILABLE - the server disappears under a running goal
    if in_flight("NAV2_UNAVAILABLE"):
        set_mock(nav_available=False)
        check(gateway.wait_for(r"disarmed by NAV2_UNAVAILABLE", 25) is not None,
              "4/7 NAV2_UNAVAILABLE disarms when navigate_to_pose disappears")
        check(gateway.wait_for(r"CANCEL NOT CONFIRMED", 25) is not None,
              "    ... a cancel that cannot be confirmed is REPORTED, and only reported")
        check(gateway.wait_for(r"releasing target .*only the upstream mechanisms", 25) is not None,
              "    ... and the goal is then released with what that means spelled out")
        # The `escalate_to_keyboard_mode` LOG GREP USED TO BE HERE, and SAFETY.md
        # 6.3 retired it: a rule whose test passes when nothing happens tests
        # nothing, and it would have passed for the same escalation under any
        # other name. What is asserted instead is the mechanism, at the one
        # moment it matters - a cancel that cannot be confirmed is exactly when
        # a node would be tempted to reach for the mux. The sweep below is this
        # process's own inventory of every way it can ask anything to act.
        sweep = gateway.wait_for(r"command-client sweep: 0 service clients", 10)
        check(sweep is not None,
              "    ... and this process has NO service client at all, so the mux's "
              "mode cannot be asked to change (SAFETY.md 6.3)",
              (sweep or "")[-95:])
        check(
            sweep is not None
            and "2 action client(s)" in sweep
            and "/navigate_to_pose (NavigateToPose)" in sweep
            and "/pick_plastic (PickPlastic)" in sweep,
            "    ... and exactly two action clients, both named at the call site",
            (sweep or "").split("sweep: ")[-1][:110],
        )
        set_mock(nav_available=True)
        check(gateway.wait_for(r"navigate_to_pose is available", 40) is not None,
              "    ... and it recovers when the server returns")
        time.sleep(3)

    # 5 LINK_LOST - kill the counterpart, not our own reporter
    if in_flight("LINK_LOST"):
        fake.signal_group(signal.SIGKILL)
        check(gateway.wait_for(r"disarmed by LINK_LOST", 30) is not None,
              "5/7 LINK_LOST disarms when the external side dies mid-goal")
        check(gateway.wait_for(r"CANCELLING target .*disarm:LINK_LOST", 10) is not None,
              "    ... and cancels it")
        fake = start_fake()
        time.sleep(8)

    # 6 EXCESSIVE_ABORTS - max_consecutive_aborts is 1 in this run
    set_mock(nav_duration_sec=3.0, nav_outcome="abort")
    if in_flight("EXCESSIVE_ABORTS"):
        check(gateway.wait_for(r"disarmed by EXCESSIVE_ABORTS", 30) is not None,
              "6/7 EXCESSIVE_ABORTS disarms after an aborted goal")
    set_mock(nav_duration_sec=45.0, nav_outcome="succeed")

    # 7 NODE_SHUTDOWN - SIGTERM with a goal in flight
    time.sleep(3)
    if in_flight("NODE_SHUTDOWN"):
        gateway.proc.send_signal(signal.SIGTERM)
        check(gateway.wait_for(r"disarmed by NODE_SHUTDOWN", 20) is not None,
              "7/7 NODE_SHUTDOWN disarms on SIGTERM with a goal in flight")
        check(gateway.wait_for(r"CANCELLING target .*NODE_SHUTDOWN", 15) is not None,
              "    ... and cancels BEFORE the context goes away (C-4)")
        check(gateway.wait_for(r"reached a terminal state", 20) is not None,
              "    ... and waits, bounded, for the cancel to be confirmed")
        check(gateway.wait_for(r"does not stop the robot", 20) is not None,
              "    ... while saying plainly that ending it does not stop the robot")
        gateway.proc.wait(timeout=20)
        check(gateway.proc.returncode == 0,
              "    ... and SIGTERM is a clean exit, not a traceback",
              f"rc={gateway.proc.returncode}")

    # F-17, and stated honestly: this one is STRUCTURAL, not observed. The run
    # above shows the shutdown path working; it cannot show it surviving its own
    # failure, because there is no way to make `_handle_disarm` raise from
    # outside the process and this package will not carry a test hook into a
    # safety path to create one. What CAN be asserted mechanically is the shape
    # the finding asked for: inside `prepare_shutdown`, the disarm is guarded and
    # the unconditional cancel is NOT inside that guard, so an exception in the
    # disarm cannot skip it. Read from the AST, not by grep.
    check(*_prepare_shutdown_structure())
    check(*_prepare_shutdown_prologue_structure())

    triggers = [
        "OPERATOR", "TIMEOUT", "MODE_CHANGE", "NAV2_UNAVAILABLE",
        "LINK_LOST", "EXCESSIVE_ABORTS", "NODE_SHUTDOWN",
    ]
    seen = [t for t in triggers if gateway.count(f"disarmed by {t}") > 0]
    # Seven, not nine: SR-15 rule 7 enumerates nine since the user split
    # CLOCK_JUMPED_BACK (9) out of CLOCK_STALLED (8) on 2026-08-19, and the two
    # clock triggers cannot be raised here - they need a `/clock` publisher that
    # freezes or rewinds, which is what `--scenario clock` parts B and E build.
    # Their codes are asserted there. The completeness of the SET ITSELF - that
    # ArmingState.msg and `arming.TRIGGER_CODES` enumerate the same nine - is an
    # offline check (`check_validation.py`, part 6).
    check(len(seen) == 7,
          "all seven of the non-clock triggers fired in one session",
          ", ".join(seen))
    check(
        gateway.count(r"/cmd_vel|set_mode|joint_commands") == 0,
        "and not one of them wrote anything to the motion chain (SR-9 / C-6)",
    )
    teardown()


def _gateway_ast():
    path = os.path.join(_PKG, "src", "gripperx_external", "goal_gateway_node.py")
    import ast

    with open(path, encoding="utf-8") as handle:
        return ast.parse(handle.read())


def _pick_tolerance_gate_structure():
    """F-14, and it can only be STRUCTURAL here.

    The user decision of 2026-08-19 makes a MEASURED `grasp.tolerance_m` a
    precondition for a pick ON THE REAL ROBOT. Its discriminator is deliberately
    the LIVE domain (SR-8: 20 is the robot, 220 is the twin), not a parameter -
    there must be no flag that makes the real machine behave like the twin. The
    consequence for this suite is that the refusal is unreachable from here BY
    DESIGN: this process runs on 221 and domain 20 must never be touched from a
    laptop harness. So what is asserted is the shape: inside `_on_arrival`, the
    tolerance gate exists and it comes BEFORE the statement that sends
    PickPlastic. Nothing here observes the refusal happening.
    """
    import ast

    func = None
    for node in ast.walk(_gateway_ast()):
        if isinstance(node, ast.FunctionDef) and node.name == "_on_arrival":
            func = node
    if func is None:
        return False, "F-14: _on_arrival exists", "not found"

    gate_at = send_at = None
    for index, stmt in enumerate(func.body):
        text = ast.dump(stmt)
        if "_pick_needs_measured_tolerance" in text and "tolerance_configured" in text:
            gate_at = index if gate_at is None else gate_at
        if "send_goal_async" in text:
            send_at = index if send_at is None else send_at
    ok = gate_at is not None and send_at is not None and gate_at < send_at
    return (
        ok,
        "F-14: the real-robot tolerance gate exists in _on_arrival and precedes "
        "the PickPlastic send (STRUCTURAL - unreachable on the twin by design)",
        f"gate at stmt {gate_at}, pick send at stmt {send_at}",
    )


def _package_sources():
    import glob

    return sorted(glob.glob(os.path.join(_PKG, "src", "gripperx_external", "*.py")))


def _no_service_clients_structure():
    """SAFETY.md 6.3, first half: this package creates NO service client.

    The acceptance rule for the rejected keyboard-mode escalation used to be a
    literal grep for `escalate_to_keyboard_mode`. The auditor ruled that the
    rule is about the MECHANISM: the grep fired on prose that exists to keep the
    mechanism out, and it would have passed for the same escalation written under
    any other name - a service client for the mux's mode switch contains none of
    those characters. So the string check is retired and this is what replaces
    it. Asserted, not assumed: the count is zero today, and it is worth pinning.
    """
    import ast

    offenders = []
    for path in _package_sources():
        with open(path, encoding="utf-8") as handle:
            tree = ast.parse(handle.read())
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_client"
            ):
                offenders.append(f"{os.path.basename(path)}:{node.lineno}")
    return (
        not offenders,
        "SAFETY.md 6.3: the package creates ZERO service clients (STRUCTURAL, "
        "the analogue of the publisher sweep for the calling direction)",
        ", ".join(offenders) or f"{len(_package_sources())} modules scanned, none found",
    )


def _forbidden_topics_structure():
    """SAFETY.md 6.3, second half: the mux's own OUTPUT is unwritable too."""
    import ast

    path = os.path.join(_PKG, "src", "gripperx_external", "octopus_link_node.py")
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read())
    topics = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "FORBIDDEN_PUBLISH_TOPICS" for t in node.targets
        ):
            topics = {
                n.value for n in ast.walk(node.value)
                if isinstance(n, ast.Constant) and isinstance(n.value, str)
            }
    required = {"/teleop/active_mode", "/teleop/set_mode", "/cmd_vel", "/goal_pose"}
    missing = sorted(required - topics)
    return (
        not missing,
        "SAFETY.md 6.3: /teleop/active_mode is in FORBIDDEN_PUBLISH_TOPICS beside "
        "/teleop/set_mode - the mux's output cannot be written either, so the "
        "escalation is unavailable as a MECHANISM and not only as a parameter name",
        f"missing: {missing}" if missing else f"{len(topics)} topics forbidden",
    )


def _sweep_called_structure():
    """Both constructors must actually run the command-client sweep."""
    import ast

    results = {}
    for module, node_class in (
        ("goal_gateway_node.py", "GoalGatewayNode"),
        ("octopus_link_node.py", "OctopusLinkNode"),
    ):
        path = os.path.join(_PKG, "src", "gripperx_external", module)
        with open(path, encoding="utf-8") as handle:
            tree = ast.parse(handle.read())
        found = False
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == node_class:
                for call in ast.walk(node):
                    if (
                        isinstance(call, ast.Call)
                        and isinstance(call.func, ast.Name)
                        and call.func.id == "assert_no_command_clients"
                    ):
                        found = True
        results[node_class] = found
    return (
        all(results.values()),
        "SAFETY.md 6.3: BOTH constructors run assert_no_command_clients, so an "
        "unreviewed client ends the process instead of being logged",
        ", ".join(f"{k}={v}" for k, v in results.items()),
    )


def _prepare_shutdown_prologue_structure():
    """SAFETY.md F-26: nothing unguarded between the latch and the cancel.

    `_shutdown_prepared` is set at the top and makes the method run at most once
    per process, so ANY raise between that latch and the unconditional cancel
    costs the cancel permanently and unretryably. F-17 guarded the disarm; the
    two logger calls above it were still bare, and a logger call raising is the
    failure this codebase has actually had (F-22).

    The assertion is exactly that shape: every call that appears before the
    `_cancel_mission` statement is either inside a `try` or is one of the two
    self-guarding helpers.
    """
    import ast

    func = None
    for node in ast.walk(_gateway_ast()):
        if isinstance(node, ast.FunctionDef) and node.name == "prepare_shutdown":
            func = node
    if func is None:
        return False, "F-26: prepare_shutdown exists", "not found"

    guarded_lines = set()
    for try_node in [n for n in ast.walk(func) if isinstance(n, ast.Try)]:
        for inner in ast.walk(try_node):
            guarded_lines.add(getattr(inner, "lineno", -1))

    cancel_line = min(
        [
            n.lineno for n in ast.walk(func)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == "_cancel_mission"
        ]
        or [-1]
    )
    safe = {"_shutdown_log", "_shutdown_now", "_cancel_mission"}
    unguarded = []
    for node in ast.walk(func):
        if not isinstance(node, ast.Call) or node.lineno >= cancel_line:
            continue
        if node.lineno in guarded_lines:
            continue
        name = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, "id", "?")
        if name in safe:
            continue
        # `self.get_logger()` is itself a call; catch it by name too.
        unguarded.append(f"{name}@{node.lineno}")
    ok = cancel_line > 0 and not unguarded
    return (
        ok,
        "F-26: every call between the shutdown latch and the unconditional cancel "
        "is guarded, so a raising logger cannot cost the cancel (STRUCTURAL)",
        f"cancel at line {cancel_line}; unguarded before it: "
        + (", ".join(unguarded) or "none"),
    )


def _prepare_shutdown_structure():
    """F-17: is the unconditional cancel reachable when the disarm raises?"""
    import ast

    path = os.path.join(_PKG, "src", "gripperx_external", "goal_gateway_node.py")
    with open(path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read())
    func = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "prepare_shutdown":
            func = node
    if func is None:
        return False, "F-17: prepare_shutdown exists", "not found"

    def calls(scope, name):
        return [
            n for n in ast.walk(scope)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == name
        ]

    guarded_disarm = any(
        calls(handler_scope, "_handle_disarm")
        for handler_scope in [t for t in ast.walk(func) if isinstance(t, ast.Try)]
    )
    # The cancel must be a statement of the function body itself, not nested in
    # the try that guards the disarm.
    try_nodes = [t for t in ast.walk(func) if isinstance(t, ast.Try)]
    cancel_in_try = any(calls(t, "_cancel_mission") for t in try_nodes)
    cancel_anywhere = bool(calls(func, "_cancel_mission"))
    ok = guarded_disarm and cancel_anywhere and not cancel_in_try
    return (
        ok,
        "F-17: the disarm is guarded and the unconditional cancel sits outside "
        "that guard, so a failing disarm cannot skip it (STRUCTURAL)",
        f"guarded_disarm={guarded_disarm} cancel_present={cancel_anywhere} "
        f"cancel_inside_guard={cancel_in_try}",
    )


def scenario_sr9() -> None:
    print("=" * 78)
    print("sr9 - C-8: the publisher inventory in the ARMED state, goal in flight")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=60.0)
    link = start_link()
    gateway = start_gateway()
    gateway.wait_for(r"navigate_to_pose is available", 60)

    def node_info_block(heading: str) -> str:
        out = subprocess.run(
            ["ros2", "node", "info", f"{NS}/goal_gateway_node"],
            env=env_for(), capture_output=True, text=True, timeout=60,
        ).stdout
        block, collecting = [], False
        for line in out.splitlines():
            if line.strip().startswith(heading):
                collecting = True
                continue
            if collecting:
                if line.strip().endswith(":") and not line.startswith("    "):
                    break
                if line.strip():
                    block.append(line.strip())
        return "\n".join(sorted(block))

    def publishers() -> str:
        return node_info_block("Publishers:")

    # `ros2 node info` can return an empty block for a node the CLI has not
    # finished discovering, and an empty baseline would make the diff below
    # pass for the wrong reason. Retry until it is non-empty, then take it.
    disarmed = ""
    deadline = time.time() + 60
    while time.time() < deadline and not disarmed:
        disarmed = publishers()
        if not disarmed:
            time.sleep(2)
    check(bool(disarmed), "the disarmed publisher inventory was read", f"{len(disarmed.splitlines())} entries")
    set_arming(True, 300.0)
    check(gateway.wait_for(r"DISPATCHING target", 30) is not None, "a goal is in flight")
    armed = publishers()
    check(
        armed == disarmed,
        "SR-9: the publisher set is BYTE-IDENTICAL armed with a goal in flight",
        "identical" if armed == disarmed else f"DIFF:\n{disarmed}\n---\n{armed}",
    )
    # Against package A's A-5 inventory this build has exactly ONE publisher
    # more, and it is named here rather than left for a reader to diff: the
    # acknowledgement channel. It is not a chain topic and it is not created at
    # all without goal ingress.
    check(
        re.search(r"^/gripperx/external/goal_done: ", armed, re.M) is not None,
        "the one publisher stage 3 adds is the acknowledgement channel, and only it",
    )
    chain = [
        "/cmd_vel", "/teleop/set_mode", "/teleop/direct_steer",
        "/hw/joint_commands", "/goal_pose", "/teleop/autonomous/cmd_vel",
        "/nav/cmd_vel_raw", "/teleop/keyboard/cmd_vel",
        "/wheel_velocity_controller/commands", "/steering_position_controller/commands",
    ]
    offenders = [t for t in chain if re.search(rf"^{re.escape(t)}: ", armed, re.M)]
    check(not offenders, "no publisher on any motion-chain topic while armed", ", ".join(offenders))

    # SAFETY.md 6.3: the CALLING direction of the same question, taken in the
    # same armed state and from the same middleware view. The publisher
    # inventory says what this node can write; this says what it can ask anybody
    # to do - and a service client for the mux's mode switch would appear here
    # and nowhere else. This is what replaced the log grep for a parameter name.
    service_clients = node_info_block("Service Clients:")
    check(
        service_clients == "",
        "SAFETY.md 6.3: ZERO service clients while armed with a goal in flight",
        service_clients or "none",
    )
    action_clients = node_info_block("Action Clients:")
    expected_actions = "\n".join(sorted([
        "/navigate_to_pose: nav2_msgs/action/NavigateToPose",
        "/pick_plastic: gripperx_arm_msgs/action/PickPlastic",
    ]))
    check(
        action_clients == expected_actions,
        "  ... and exactly the two action clients stage 3 is allowed, no more",
        action_clients.replace("\n", " | ") or "none",
    )
    print("  armed inventory:\n    " + "\n    ".join(armed.splitlines()), flush=True)
    teardown()




def scenario_clock() -> None:
    """SAFETY.md F-24, in BOTH clock modes, which is the whole point.

    The finding is not "sim time is wrong". It is that `use_sim_time` decides
    which clock EVERY timer-driven safety mechanism in the gateway is measured
    on, that the twin launch file defaults it to true, that with true and no
    `/clock` not one of those timers fires - and that the acceptance suite
    forced it to false, so no green result in this file had ever been taken in
    the configuration an operator gets.

    Three parts, in the order the risk runs:

      A  sim time WITH a `/clock` (test/sim_clock.py standing in for Gazebo):
         everything must work exactly as it does on wall time, including the
         arming expiry. If it did not, the fix would just be hiding the mode.
      B  the clock FREEZES with a goal in flight, while still being published at
         50 Hz so nothing is missing to notice: detected, disarmed, cancelled,
         and arming refused for as long as it lasts.
      C  sim time with NO `/clock` at all - the auditor's own A/B, whose counts
         were server-availability 0, previews 0, dispatch decisions 0 over 40 s
         with the node reporting itself healthy throughout.

    And three more since SAFETY.md revision 4, all of which are about a clock
    that is NOT stopped:

      D  the clock RUNNING AT 10% of real time (F-29). Everything answers "yes,
         it is advancing", correctly, and the two mechanisms that are promises
         in wall-clock seconds - the arming window and the link watchdog - must
         still take the number of seconds they were given. The auditor measured
         a 20 s window unexpired after 123 s of wall time here.
      E  the clock JUMPING BACKWARDS 120 s (F-30). It must be reported as what
         it is - including AS ITS OWN `ArmingState` CONSTANT, 9, and not as the
         stall's 8 (SR-15 rule 7, user decision 2026-08-19) - must disarm, and
         must then allow re-arming promptly instead of refusing for as long as
         the jump was large. B above asserts the stall still reports 8, in the
         same session, so the pair is what makes the split falsifiable.
      F  ORDINARY STARTS (F-31). The ERROR that says the clock has never
         advanced must not fire on starts where nothing is wrong, in either
         clock mode and in either publisher order - and the WARN that replaces
         it during the discovery grace must NOT relax the gate.

    And one more since revision 5, at the same 0.1x factor as D:

      J  F-35's CONSEQUENCE at three epoch distances (real Gazebo, replay
         started now, replay of an old bag). The detector is checked; what
         follows it is observed, because F-35 is report-only and its downstream
         behaviour is a finding rather than a specification.

      I  the MIRROR of package D's sim-time refusal (F-35): a LIVE `/clock`
         publisher while `use_sim_time` is false. Both firing points, both
         nodes, and the ordinary twin configuration kept quiet. It reports; it
         does not refuse, disarm or cancel - and nothing in it is evidence about
         what the epoch mismatch DOES, which stays SUSPECTED.

      H  the two wall-clock statements package E did NOT convert. F-37: the
         ADVERTISED expiry, `ArmingState.expires_at`, which was projected with
         an assumed rate of 1.0 and reported a gate closing ten times further
         away than it did. F-38: `max_teleop_mode_age_sec`, the only thing that
         catches a `teleop_mux` that stopped publishing, which was measured in
         sim seconds and so caught a dead mux after 22.1 s against a configured
         2.0 s. D is about the promises that were fixed; H is about the two that
         were missed.
    """
    print("=" * 78)
    print("clock - F-24: the same stack in sim time, in frozen sim time, and in neither")
    print("=" * 78)

    # -- A: sim time with a live /clock -----------------------------------
    print("  A: sim time WITH /clock - the configuration the launch file produces")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=6.0, pick_duration_sec=1.0)
    clock = start_sim_clock()
    link = start_link()
    gateway = start_gateway(use_sim_time="true")
    check(
        gateway.wait_for(r"ROS clock observed advancing \(use_sim_time=True\)", 40) is not None,
        "in sim time WITH a clock the gateway sees the clock advancing and says so",
    )
    check(
        gateway.wait_for(r"navigate_to_pose is available", 40) is not None,
        "  ... the dispatch tick runs, so servers are discovered (a ROS-clock timer)",
    )
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"DISPATCHING target", 30) is not None,
        "  ... a goal is dispatched in sim time",
    )
    check(
        gateway.wait_for(r"ACKNOWLEDGED target .* as COLLECTED", 60) is not None,
        "  ... and the whole loop completes, so nothing about sim time is inert",
    )
    # The arming expiry is the one timer whose failure is invisible: it does not
    # stop something happening, it stops something STOPPING.
    set_arming(True, 6.0)
    check(
        gateway.wait_for(r"disarmed by TIMEOUT", 40) is not None,
        "  ... and the arming window EXPIRES by itself in sim time (the auditor's "
        "10 s window that had not expired after 45 s)",
    )

    # -- B: the clock freezes under a goal in flight ----------------------
    print("-" * 78)
    print("  B: /clock FROZEN with a goal in flight - still published, never advancing")
    set_mock(nav_duration_sec=60.0)
    time.sleep(2)
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"DISPATCHING target", 40) is not None,
        "a goal is in flight in sim time",
    )
    before_cancels = gateway.count(r"CANCELLING target")
    before_acks = gateway.count(r"ACKNOWLEDGED target .* as COLLECTED")
    set_sim_clock(paused=True)
    stalled = gateway.wait_for(r"the ROS clock STOPPED", 30)
    check(stalled is not None,
          "the frozen clock is DETECTED against a monotonic reference, at ERROR",
          (stalled or "").split("]: ")[-1][:120])
    stall_disarm = gateway.wait_for(
        r"disarmed by CLOCK_STALLED \(ArmingState constant 8\)", 20
    )
    check(
        stall_disarm is not None,
        "  ... and it DISARMS with CLOCK_STALLED, reporting ArmingState constant "
        "8: a window whose expiry is inert is not a window. Part E asserts that "
        "the backwards jump reports 9 instead, in this same session (SR-15 r.7)",
        # Quoted, because this half of the split is only meaningful next to the
        # other half - and the two parts write to the same log file name.
        (stall_disarm or "").split("]: ")[-1][:70],
    )
    check(
        gateway.wait_for(r"CANCELLING target .*disarm:CLOCK_STALLED", 20) is not None,
        "  ... and cancels the goal in flight (the executor still delivers events)",
        f"{gateway.count(r'CANCELLING target') - before_cancels} new cancel(s)",
    )
    refused = set_arming(True, 60.0)
    check(
        "the ROS clock is not advancing" in refused,
        "  ... and re-arming is REFUSED while the clock is frozen, naming the clock",
        refused.strip().splitlines()[-1][:100] if refused.strip() else "(no output)",
    )
    check(
        gateway.count(r"ACKNOWLEDGED target .* as COLLECTED") == before_acks,
        "  ... and nothing was acknowledged while the clock was frozen",
        f"{before_acks} before, "
        f"{gateway.count(r'ACKNOWLEDGED target .* as COLLECTED')} after",
    )

    # -- and recovery: the refusal is not a dead end ----------------------
    set_sim_clock(paused=False)
    check(
        gateway.wait_for(r"ROS clock is advancing again", 30) is not None,
        "when the clock runs again the gateway says so",
    )
    check(
        gateway.wait_for(r"NOT re-opened by this", 10) is not None,
        "  ... and does NOT re-open the gate by itself: re-arming stays an operator act",
    )
    time.sleep(2)
    granted = set_arming(True, 120.0)
    check(
        "armed for 120" in granted,
        "  ... while an explicit arming is granted again once the clock is honest",
    )
    teardown()
    shm_clean()

    # -- C: the auditor's A/B - sim time with no /clock at all ------------
    print("-" * 78)
    print("  C: sim time with NO /clock - the auditor's A/B, and what it costs now")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=20.0)
    link = start_link()
    gateway = start_gateway(name="goal_gateway_node_noclock", use_sim_time="true")
    never = gateway.wait_for(r"the ROS clock has NEVER advanced", 30)
    check(never is not None,
          "with sim time and no /clock the gateway says so at ERROR once the "
          "startup grace has passed - it does not sit there looking healthy",
          (never or "").split("]: ")[-1][:120])
    refused = set_arming(True, 300.0)
    check("the ROS clock is not advancing" in refused,
          "  ... and REFUSES to arm, which is the state every timer being inert "
          "actually means")
    time.sleep(20)
    # The auditor's three counters, measured again on this build. The first two
    # are still zero - the timers really are frozen, that is the nature of the
    # thing - and the difference is that it is now said out loud and the gate
    # cannot be opened.
    check(
        gateway.count(r"navigate_to_pose is available") == 0
        and gateway.count(r"DISPATCHING target") == 0,
        "  ... the timers ARE still frozen (0 server-availability lines, 0 "
        "dispatch decisions in 20 s) - what changed is that this is now reported "
        "and refused, not silent",
        f"avail={gateway.count(r'navigate_to_pose is available')} "
        f"dispatch={gateway.count(r'DISPATCHING target')} "
        f"clock_errors={gateway.count(r'ROS clock has NEVER advanced')}",
    )
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "  ... and nothing is acknowledged, as nothing can be",
    )
    teardown()
    shm_clean()

    # -- D: the clock runs SLOWLY - SAFETY.md F-29 -------------------------
    print("-" * 78)
    print("  D: /clock at a real-time factor of 0.1 - a loaded twin (F-29)")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=6.0, pick_duration_sec=1.0)
    clock = start_sim_clock(scale=0.1)
    link = start_link()
    gateway = start_gateway(use_sim_time="true")
    check(
        gateway.wait_for(r"ROS clock observed advancing", 60) is not None,
        "a /clock at 10% of real time is PROVEN, and rightly so - it is advancing",
    )
    check(
        gateway.count(r"the ROS clock (STOPPED|has NEVER advanced)") == 0,
        "  ... and is not reported as stalled: a slow clock is not a stopped one, "
        "and the fix for F-29 is not to call it one",
    )
    granted = set_arming(True, 20.0)
    check("armed for 20" in granted, "  ... armed for 20 seconds")
    started = time.time()
    expired = gateway.wait_for(r"disarmed by TIMEOUT", 90)
    elapsed = time.time() - started
    check(
        expired is not None and elapsed <= 40.0,
        "F-29: the 20 s window expires in WALL seconds - the operator's seconds. "
        "The auditor's identical window had not expired after 123 s of wall time, "
        "because the expiry was measured in sim seconds",
        f"{elapsed:.1f}s of wall time for a 20 s window (sim time would be ~200 s)",
    )
    set_arming(True, 300.0)
    time.sleep(2)
    link.stop()
    started = time.time()
    lost = gateway.wait_for(r"link watchdog: last link_status", 60)
    elapsed = time.time() - started
    check(
        lost is not None and elapsed <= 25.0,
        "F-29: the link watchdog trips link_lost_sec of WALL time after the link "
        "node dies - a WiFi link does not slow down when Gazebo does; at this "
        "factor it used to tolerate ten times its configured outage",
        f"{elapsed:.1f}s of wall time for link_lost_sec=5.0 (sim time: ~50 s)",
    )
    check(
        gateway.wait_for(r"disarmed by LINK_LOST", 20) is not None,
        "  ... and it disarms on it, on the slow clock, as it does on wall time",
    )
    # The other half of the same coin, and the regression the F-29 fix could
    # have introduced: the link node runs in SIM TIME in the twin config
    # (octopus_link_twin.yaml), and its `link_status` heartbeat used to be a ROS
    # timer. A monotonic watchdog reading a sim-time heartbeat would declare
    # LINK_LOST on a perfectly healthy link at this factor - 1 Hz of sim time is
    # one publication every 10 wall-seconds against a 5 s tolerance. Both timers
    # in the link node are on a steady clock for exactly this reason.
    print("      restarting the link node IN SIM TIME, as the twin config runs it")
    link = start_link(name="octopus_link_node_simtime", use_sim_time="true")
    time.sleep(8)
    trips_before = gateway.count(r"link watchdog: last link_status")
    set_arming(True, 300.0)
    time.sleep(25)
    check(
        gateway.count(r"link watchdog: last link_status") == trips_before,
        "F-29: with the link node in SIM TIME at a factor of 0.1, its heartbeat "
        "still arrives on WALL time, so the monotonic watchdog does NOT declare a "
        "healthy link lost - producer and consumer are on the same clock",
        f"{gateway.count(r'link watchdog: last link_status') - trips_before} "
        "new trips in 25 s of wall time (1 Hz heartbeat, link_lost_sec=5.0)",
    )
    teardown()
    shm_clean()

    # -- E: the clock jumps BACKWARDS - SAFETY.md F-30 ---------------------
    print("-" * 78)
    print("  E: /clock JUMPS BACK 120 s at 1.0x - a Gazebo world reset (F-30)")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=60.0)
    clock = start_sim_clock()
    link = start_link()
    gateway = start_gateway(use_sim_time="true")
    check(
        gateway.wait_for(r"ROS clock observed advancing", 40) is not None,
        "the clock is proven at 1.0x",
    )
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"DISPATCHING target", 60) is not None,
        "a goal is in flight",
    )
    before_cancels = gateway.count(r"CANCELLING target")
    # THE FIXTURE FIRST, and separately. Without this the four checks below
    # report "the gateway did not notice a backwards jump" when the truth may be
    # that no backwards jump ever happened - which is exactly what one
    # full-suite run recorded before this guard existed. `clock_fwd` has had the
    # equivalent guard since it was written; this is the same guard on the older
    # half of the scenario.
    set_sim_clock_param("jump_back_sec", "120.0")
    fixture_jumped = clock.wait_for(r"/clock JUMPED BACK", 25)
    check(
        fixture_jumped is not None,
        "the FIXTURE really did rewind the clock 120 s - asserted before "
        "anything is asked of the gateway, so a harness failure cannot be "
        "reported as a gateway failure",
        (fixture_jumped or "").split("]: ")[-1][:80],
    )
    jumped = gateway.wait_for(r"JUMPED BACKWARDS", 30)
    check(
        jumped is not None,
        "F-30: the jump is reported AS a backwards jump - it used to be reported "
        "as the clock having STOPPED, which sends an operator looking for a dead "
        "publisher that is running perfectly",
        (jumped or "").split("]: ")[-1][:110],
    )
    check(
        gateway.wait_for(r"disarmed by CLOCK_JUMPED_BACK \(ArmingState constant 9\)",
                         20) is not None,
        "SR-15 r.7: it disarms with its OWN trigger, CLOCK_JUMPED_BACK, reporting "
        "ArmingState constant 9. The implementation first reused the stall's code "
        "8 for this; the user split them on 2026-08-19 because a dead clock and a "
        "reset world need different operator responses and that has to be "
        "machine-readable, not only readable in the detail string",
    )
    check(
        gateway.count(r"disarmed by CLOCK_STALLED") == 0,
        "  ... and NOT as CLOCK_STALLED: a test that accepted either code would "
        "pass on the very build the decision reversed",
        f"{gateway.count(r'disarmed by CLOCK_STALLED')} stall disarm(s) in this run",
    )
    check(
        gateway.wait_for(r"CANCELLING target .*disarm:CLOCK_JUMPED_BACK", 20)
        is not None,
        "  ... and the cancel it triggers carries the same trigger name, so the "
        "cancel and the disarm cannot be attributed to different faults",
    )
    check(
        gateway.wait_for(r"CANCELLING target", 20) is not None
        and gateway.count(r"CANCELLING target") > before_cancels,
        "  ... and cancels the goal in flight",
        f"{gateway.count(r'CANCELLING target') - before_cancels} new cancel(s)",
    )
    started = time.time()
    granted = ""
    while time.time() - started < 40.0:
        granted = set_arming(True, 60.0)
        if "armed for 60" in granted:
            break
        time.sleep(2)
    elapsed = time.time() - started
    check(
        "armed for 60" in granted and elapsed <= 25.0,
        "F-30: arming is granted again within seconds, because the reference was "
        "RE-BASELINED. It used to be refused for as long as the jump was large - "
        "the auditor was still refused 30.5 s later on a clock advancing at 1.0x, "
        "and a 120 s jump means 120 s of refusals",
        f"granted {elapsed:.1f}s after the jump",
    )
    teardown()
    shm_clean()

    # -- F: ordinary starts must be quiet - SAFETY.md F-31 -----------------
    print("-" * 78)
    print("  F: ORDINARY starts raise no ERROR, in both clock modes (F-31)")
    clock_error = r"\[ERROR\].*(ROS clock|clock watchdog)"
    clock_warn = r"\[WARN\].*ROS clock"
    gateway = start_gateway(name="goal_gateway_node_wall", use_sim_time="false")
    check(
        gateway.wait_for(r"goal_gateway_node up", 40) is not None,
        "wall time, no /clock anywhere: the node comes up",
    )
    time.sleep(6)
    check(
        gateway.count(clock_error) == 0 and gateway.count(clock_warn) == 0,
        "  ... and says NOTHING about the clock - no ERROR, no WARN",
        f"errors={gateway.count(clock_error)} warns={gateway.count(clock_warn)}",
    )
    teardown()
    shm_clean()

    quiet = 0
    for attempt in range(3):
        # The twin's own order: both come up together, so the subscription has
        # to discover a publisher that is itself still starting. This is where
        # the auditor measured 3.20 s and 3.48 s, i.e. past clock_stall_sec.
        gateway = start_gateway(name="goal_gateway_node_alongside", use_sim_time="true")
        clock = start_sim_clock()
        proven = gateway.wait_for(r"ROS clock observed advancing", 40)
        errors = gateway.count(clock_error)
        warns = gateway.count(clock_warn)
        check(
            proven is not None and errors == 0,
            f"sim time, /clock starting ALONGSIDE (run {attempt + 1}/3): proven, "
            "and no ERROR on a start where nothing is wrong",
            f"errors={errors} warns={warns}",
        )
        if warns:
            check(
                gateway.count(r"inside the .* startup grace") == warns,
                "  ... and the WARN that did appear names the discovery grace",
            )
        else:
            quiet += 1
        teardown()
        shm_clean()
    # Informational, not a check: how quiet the three runs were is a property of
    # this laptop's DDS discovery on the day, not of the code.
    print(f"      ({quiet}/3 alongside-runs were completely silent about the clock)")

    gateway_first = start_sim_clock()
    time.sleep(5)
    gateway = start_gateway(name="goal_gateway_node_clockup", use_sim_time="true")
    check(
        gateway.wait_for(r"ROS clock observed advancing", 40) is not None
        and gateway.count(clock_error) == 0,
        "sim time, /clock ALREADY UP: proven, and no ERROR either",
        f"errors={gateway.count(clock_error)}",
    )
    teardown()
    shm_clean()

    # And the other half: the grace must not be a way of NOT reporting a clock
    # that really never advances, and it must not relax the gate while it lasts.
    gateway = start_gateway(
        name="goal_gateway_node_grace", use_sim_time="true",
        clock_stall_sec="0.5", clock_startup_grace_sec="12.0",
    )
    warned = gateway.wait_for(r"the ROS clock has not advanced YET", 30)
    check(
        warned is not None,
        "sim time with NO /clock and a 12 s grace: the unproven clock is reported "
        "at WARN first, not at ERROR",
        (warned or "").split("]: ")[-1][:110],
    )
    check(
        gateway.count(clock_error) == 0,
        "  ... and there is no ERROR yet, which is the whole of F-31",
    )
    refused = set_arming(True, 60.0)
    check(
        "the ROS clock is not advancing" in refused
        and "discovery grace" in refused,
        "  ... while ARMING IS REFUSED during the grace exactly as after it: the "
        "grace grades the report, never the gate",
        refused.strip().splitlines()[-1][:100] if refused.strip() else "(no output)",
    )
    escalated = gateway.wait_for(r"the ROS clock has NEVER advanced", 40)
    check(
        escalated is not None and "startup grace" in escalated,
        "  ... and when the grace runs out it ESCALATES to ERROR, naming the grace "
        "it has passed - a clock that never advances is still an ERROR",
        (escalated or "").split("]: ")[-1][:110],
    )
    teardown()
    shm_clean()

    # -- H: the two promises F-37 and F-38 make, at a factor of 0.1 -----------
    # Both findings are about a WALL-CLOCK statement measured or reported in sim
    # seconds, and both were measured by the auditor at this factor. D above
    # covers the two promises package E converted; this covers the two it did
    # not. Run at 0.1x for the same reason D is: at 1.0x every one of these
    # numbers is right by accident.
    print("-" * 78)
    print("  H: /clock at 0.1x - the ADVERTISED expiry (F-37) and a dead mux (F-38)")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=600.0, pick_duration_sec=1.0)
    clock = start_sim_clock(scale=0.1)
    link = start_link()
    gateway = start_gateway(use_sim_time="true")
    check(
        gateway.wait_for(r"ROS clock observed advancing", 60) is not None,
        "the 0.1x clock is proven",
    )
    # The rate estimate has a ~2 s time constant and is seeded at 1.0, so this
    # waits for it to have SEEN the slow clock rather than asserting against a
    # number that is still converging. Nothing gates on the estimate; it feeds
    # one reported field.
    time.sleep(12)
    granted = set_arming(True, 60.0)
    check("armed for 60" in granted, "  ... and a 60 s window is granted")
    state = echo_once(f"{NS}/arming_state")
    stamp = _stamp_sec(state, "header")
    expires = _stamp_sec(state, "expires_at")
    remaining = _float_field(state, "seconds_remaining")
    ahead = (expires - stamp) if (stamp and expires) else float("nan")
    if readable(state, "ArmingState on {NS}/arming_state".format(NS=NS)):
        check(
            stamp is not None and expires is not None and 3.0 <= ahead <= 12.0,
        "F-37: `expires_at` is projected AT THE OBSERVED RATE - a 60 s wall window "
        "on a clock running at 0.1x closes ~6 ROS seconds from the message stamp, "
        "not 60. The pre-fix build advertised stamp+60.000 here, an instant this "
        "clock reaches after ~595 wall-seconds",
            f"expires_at - header.stamp = {ahead:.3f} ROS s, "
            f"seconds_remaining = {remaining}",
        )
        check(
            remaining is not None and 55.0 <= remaining <= 60.5,
            "  ... while `seconds_remaining` is unchanged and still the field "
            "that needs no interpretation: WALL seconds, the operator's seconds",
            f"seconds_remaining={remaining}",
        )
    diagnostics = echo_once("/diagnostics", timeout=40.0, expect="external/clock")
    reported_rate = _diag_field(diagnostics, "external/clock", "ros_clock_rate")
    if readable(diagnostics, "/diagnostics external/clock"):
        check(
            reported_rate not in ("<unread>", "")
            and 0.05 <= float(reported_rate) <= 0.2,
            "  ... and /diagnostics carries the rate the projection used, so a "
            "consumer that finds the field surprising can see what produced it",
            f"external/clock -> ros_clock_rate = {reported_rate}",
        )

    # F-38: the dead mux, with a goal in flight. This is A-37 in the suite: the
    # auditor's probe R4 was ad hoc, so the path had never been exercised here.
    set_arming(True, 600.0)
    check(
        gateway.wait_for(r"DISPATCHING target", 120) is not None,
        "F-38: a goal is in flight before the mux is killed - a stale mode is a "
        "refusal AND a cancel, and only the cancel needs something in flight",
    )
    set_mock(teleop_mode="")
    started = time.time()
    stale = gateway.wait_for(r"MODE_STALE", 90)
    elapsed = time.time() - started
    check(
        stale is not None and elapsed <= 12.0,
        "F-38: a `teleop_mux` that stopped publishing is caught in WALL seconds. "
        "The auditor measured 22.1 s of wall time for a configured 2.0 s at this "
        "factor, because the age was on the ROS clock; a mux does not slow down "
        "when Gazebo does (SR-15 rule 12)",
        f"{elapsed:.1f}s of wall time for max_teleop_mode_age_sec=2.0 "
        f"(pre-fix: ~22 s)",
    )
    check(
        stale is not None and "MODE_STALE" in (stale or ""),
        "  ... and it is reported as MODE_STALE, naming the age and the threshold",
        (stale or "").split("]: ")[-1][:110],
    )
    check(
        gateway.wait_for(r"CANCELLING target .*MODE_STALE", 30) is not None,
        "  ... and the goal in flight is CANCELLED on it - a cancel, never a write "
        "to a chain topic (SR-15 rules 8 and 9)",
    )
    check(
        gateway.count(r"disarmed by MODE_CHANGE") == 0,
        "  ... and it does NOT disarm: a mode report that went silent is not a "
        "broken arming precondition, and F-38 does not change that",
    )
    teardown()
    shm_clean()

    # -- I: the MIRROR of package D's refusal - SAFETY.md F-35 -------------
    # Package D refuses `use_sim_time: true` where no /clock can exist. The
    # reverse - a LIVE /clock publisher while we are on WALL time - went
    # undetected, and the package did not look for a /clock publisher at all.
    # DECIDED 2026-08-20 (user): build the check. It WARNS; it does not refuse,
    # disarm or cancel.
    #
    # WHAT THIS PART DOES NOT SHOW, stated here because the scenario name
    # invites the assumption: nothing below is evidence about what the epoch
    # mismatch DOES. The auditor predicts it is fail-safe and calls it a
    # prediction; observing it needs the real Gazebo twin, and the user decided
    # not to make that run. F-35's consequence half stays SUSPECTED.
    print("-" * 78)
    print("  I: a LIVE /clock while use_sim_time is FALSE - the F-35 mirror")
    mismatch = r"a /clock publisher is LIVE"

    # I.1 the clock is already up when the node starts
    clock = start_sim_clock()
    time.sleep(4)
    gateway = start_gateway(use_sim_time="false")
    warned = gateway.wait_for(mismatch, 40)
    check(
        warned is not None,
        "F-35: a /clock publisher that is LIVE while use_sim_time is false is "
        "DETECTED and reported. Before this the package asked nothing about "
        "/clock publishers at all - `count_publishers` appeared once in it, for "
        "link_status",
        (warned or "").split("]: ")[-1][:95],
    )
    check(
        "[WARN]" in (warned or ""),
        "  ... at WARN, not FATAL: unlike the forward direction this is a WORKING "
        "clock measuring against the wrong epoch, so it is reported rather than "
        "refused",
    )
    check(
        gateway.count(mismatch) == 1,
        "  ... exactly once, although /clock arrives at 50 Hz - a mismatch "
        "repeated on every message is a mismatch nobody reads",
        f"{gateway.count(mismatch)} report(s)",
    )
    check(
        gateway.count(r"disarmed by") == 0
        and gateway.count(r"CANCELLING target") == 0
        and gateway.proc.poll() is None,
        "F-35: and it does NOT disarm, does NOT cancel and does NOT exit. The "
        "gate is untouched, exactly as for F-40",
        f"{gateway.count(r'disarmed by')} disarm(s), "
        f"{gateway.count(r'CANCELLING target')} cancel(s), "
        f"alive={gateway.proc.poll() is None}",
    )
    link = start_link()
    check(
        link.wait_for(mismatch, 30) is not None,
        "  ... and the LINK NODE reports it too: both nodes carry package D's "
        "refusal, so both carry its mirror - a check in one node and not the "
        "other is exactly the drift `domain_guard` exists to prevent",
    )
    teardown()
    shm_clean()

    # I.2 the node starts FIRST and the clock appears afterwards. This is the
    # case a startup-only check misses completely, and it is why the auditor
    # asked for the check to run again on the first /clock message.
    gateway = start_gateway(use_sim_time="false")
    time.sleep(12)
    check(
        gateway.count(mismatch) == 0,
        "F-35: with no /clock publisher yet, the node is SILENT - that is the "
        "real robot and every other scenario in this suite",
    )
    clock = start_sim_clock()
    late = gateway.wait_for(mismatch, 40)
    check(
        late is not None and "on the first /clock message" in (late or ""),
        "F-35: a /clock publisher that appears AFTER startup is still caught, on "
        "the first message. A startup-only check would have missed this entirely, "
        "and 'somebody started a bag replay' is precisely that shape",
        (late or "").split("seen ")[-1][:60],
    )
    teardown()
    shm_clean()

    # I.3 the ordinary twin configuration must stay quiet.
    clock = start_sim_clock()
    time.sleep(4)
    gateway = start_gateway(use_sim_time="true")
    check(
        gateway.wait_for(r"ROS clock observed advancing", 40) is not None,
        "the ordinary twin configuration - sim time WITH a /clock - comes up",
    )
    check(
        gateway.count(mismatch) == 0,
        "F-35: and says NOTHING about a mismatch, because there is none. A check "
        "that fired on the configuration the launch file produces would be turned "
        "off within a day",
        f"{gateway.count(mismatch)} report(s)",
    )
    teardown()
    shm_clean()

    # -- J: F-35's CONSEQUENCE, across the epoch distance ------------------
    # The real-Gazebo run of 2026-08-20 showed the mismatch is fail-safe when
    # sim time starts at 0. It could not show what happens when the epochs are
    # CLOSE, which is the bag-replay case F-35 was argued from (§6.4 item 8).
    # `sim_clock.py --epoch-mode` exists for that, by user decision 2026-08-21.
    #
    # The DETECTOR firing is decided behaviour and is checked. What follows the
    # detection is a FINDING IN PROGRESS - F-35 is report-only by decision, and
    # asserting today's downstream behaviour would quietly turn an observation
    # into a specification. Those lines are `[OBS ]`.
    print("-" * 78)
    print("  J: F-35's consequence at three epoch distances (gateway on WALL time)")
    for tag, kwargs in (
        ("epoch at 0 - a real Gazebo (~1.787e9 s apart)", {"epoch_mode": "gazebo"}),
        ("epoch at wall - a replay started now (~0 s apart)", {"epoch_mode": "wall"}),
        ("epoch at wall-1h - a replay of an old bag (3600 s apart)",
         {"epoch_mode": "wall", "epoch": time.time() - 3600.0}),
    ):
        print(f"      {tag}")
        clock = start_sim_clock(**kwargs)
        time.sleep(4)
        fake = start_fake()
        mock = start_mock(nav_duration_sec=60.0, pick_duration_sec=1.0,
                          use_sim_time="true")     # TF stamped in the SIM epoch
        link = start_link()
        gateway = start_gateway(use_sim_time="false")     # <-- the mismatch
        fired = gateway.wait_for(r"a /clock publisher is LIVE", 40)
        check(
            fired is not None,
            f"F-35: the detector fires at this epoch distance too - it counts "
            "PUBLISHERS, so it must be independent of where sim time starts",
        )
        set_arming(True, 300.0)
        time.sleep(45)
        considered = gateway.count(r"goal 3")
        check(
            considered > 0,
            "  ... and a goal really WAS put to the gateway, so what follows is "
            "about the epoch and not about an idle pipeline",
            f"{considered} line(s) about goal 3",
        )
        check(
            gateway.count(r"disarmed by") == 0 and gateway.proc.poll() is None,
            "  ... and nothing was disarmed and the node is still up: F-35 is "
            "report-only by decision",
        )
        observe(
            "  what the gateway then DID",
            f"dispatched={gateway.count(r'DISPATCHING target')}, "
            f"arrived={gateway.count(r'ARRIVED at target')}, "
            f"acknowledged={gateway.count(r'ACKNOWLEDGED target')}, "
            f"TF_UNAVAILABLE={gateway.count(r'TF_UNAVAILABLE')}, "
            f"TF_STALE={gateway.count(r'TF_STALE')}",
        )
        teardown()
        shm_clean()


def _stamp_sec(blob: str, key: str) -> Optional[float]:
    """`sec`/`nanosec` out of a `ros2 topic echo`, by the field that owns them.

    `header` is spelt as the header rather than as `stamp`, because `stamp:` is
    the nested key and the caller is naming the field it wants.
    """
    lines = blob.splitlines()
    try:
        start = next(i for i, line in enumerate(lines)
                     if line.strip().startswith(f"{key}:"))
    except StopIteration:
        return None
    sec = nanosec = None
    for line in lines[start:start + 6]:
        stripped = line.strip()
        if stripped.startswith("sec:") and sec is None:
            sec = int(stripped.split(":", 1)[1])
        elif stripped.startswith("nanosec:") and nanosec is None:
            nanosec = int(stripped.split(":", 1)[1])
        if sec is not None and nanosec is not None:
            return sec + nanosec * 1e-9
    return None


def _float_field(blob: str, key: str) -> Optional[float]:
    for line in blob.splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{key}:"):
            try:
                return float(stripped.split(":", 1)[1].strip())
            except ValueError:
                return None
    return None


def scenario_stale() -> None:
    """SAFETY.md F-28: a correlation input that stopped being refreshed.

    Their detector dies while their datum publisher lives. `trash_goal` and the
    datum keep arriving at 1 Hz, so the link watchdog - which measures the last
    frame of ANY topic - reports a healthy link throughout, and the target list
    the three F-13 gates re-ask simply stops changing. Before this fix every one
    of those gates returned the same confident `UNIQUE` for as long as it lasted,
    including the gate in front of the irreversible acknowledgement.

    Two parts: the drive (a stale list must not be driven on) and the
    acknowledgement (a stale list must never name an object as collected).
    """
    print("=" * 78)
    print("stale - F-28: their detector goes silent, the link stays healthy")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=40.0, pick_duration_sec=1.0)
    link = start_link()
    gateway = start_gateway()
    gateway.wait_for(r"pick_plastic is available", 60)
    set_arming(True, 600.0)
    check(
        gateway.wait_for(r"DISPATCHING target", 30) is not None,
        "a goal is dispatched while their list is live",
    )
    fake.send("silence gps")
    stale = gateway.wait_for(r"TARGETS_STALE", 40)
    check(stale is not None,
          "their trash_gps going silent is caught as TARGETS_STALE - its own "
          "reason, not folded into NO_MATCH",
          (stale or "").split("]: ")[-1][:120])
    check(
        gateway.wait_for(r"CANCELLING target .*CORRELATION_LOST", 30) is not None,
        "  ... and the goal IN FLIGHT is withdrawn: a stale list is not evidence "
        "about where that object is",
    )
    check(
        gateway.count(r"disarmed by LINK_LOST") == 0,
        "  ... while the LINK stayed healthy throughout, which is exactly why the "
        "link watchdog could not have caught this",
    )
    check(
        gateway.count(r"SR-16: sending PickPlastic") == 0 and gateway.count(r"ACKNOWLEDGED target") == 0,
        "  ... the arm never actuates and nothing is acknowledged on a frozen list",
    )
    fake.send("resume gps")
    check(
        gateway.wait_for(r"DISPATCHING target", 40) is not None,
        "  ... and the refusal is not a dead end: it re-dispatches once their "
        "list is live again",
    )
    check(
        gateway.wait_for(r"ACKNOWLEDGED target .* as COLLECTED", 90) is not None,
        "  ... and completes normally",
    )
    teardown()
    shm_clean()

    # -- the irreversible gate: the list goes stale DURING the pick --------
    print("-" * 78)
    print("  the acknowledgement gate: the list dies while the arm is picking")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=3.0, pick_duration_sec=14.0, pick_outcome="succeed")
    link = start_link()
    gateway = start_gateway(name="goal_gateway_node_stalepick")
    gateway.wait_for(r"pick_plastic is available", 60)
    set_arming(True, 600.0)
    check(
        gateway.wait_for(r"SR-16: sending PickPlastic", 60) is not None,
        "the pick starts with the correlation unique and the list fresh",
    )
    fake.send("silence gps")
    check(
        gateway.wait_for(r"pick SUCCEEDED for target", 40) is not None,
        "  ... the pick is NOT cancelled mid-motion - a committed arm stays committed",
    )
    refused = gateway.wait_for(
        r"was picked successfully, but its correlation no longer holds", 20
    )
    check(refused is not None,
          "  ... and the IRREVERSIBLE acknowledgement is refused on the stale list",
          (refused or "")[-110:])
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "  ... nothing is acknowledged, although the object really was collected",
    )
    check(fake.count(r"acknowledged as collected") == 0,
          "  ... and their side never advances on evidence that stopped arriving")
    teardown()


def scenario_permissive() -> None:
    """Package A's A-1 and A-3, re-taken against the build that CAN dispatch.

    A-1 was verified on a build with no action client, where "did not dispatch"
    was guaranteed by there being nothing to dispatch with. The claim only
    becomes interesting now, so it is re-taken with every switch this package
    offers set to its most permissive value - including the two stage 3 added.
    """
    print("=" * 78)
    print("permissive - A-1 and A-3 re-taken against a build that can actually dispatch")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=30.0)
    link = start_link()
    gateway = start_gateway(**{
        "allow_arm": "true",
        "dry_run": "false",
        "goal_ingress_enabled": "true",
        "auto_pick": "true",
        "arming.max_duration_sec": "9999.0",
    })
    check(
        gateway.wait_for(r"armed=False", 40) is not None,
        "A-1: the most permissive configuration still comes up DISARMED",
    )
    check(
        gateway.wait_for(r"exceeds the hard maximum of 600s", 10) is not None,
        "  ... and a config ceiling above the hard maximum is clamped DOWN, loudly",
    )
    check(
        gateway.wait_for(r"navigate_to_pose is available", 40) is not None,
        "  ... with a live navigate_to_pose server, so nothing else is stopping it",
    )
    time.sleep(8)
    check(
        gateway.count(r"DISPATCHING target") == 0,
        "A-1: and it dispatches NOTHING - no parameter reaches the armed state",
    )
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "  ... and acknowledges nothing either",
    )

    # A-3: rejection, never clamping.
    out = set_arming(True, 0.0)
    check("duration_sec is required" in out, "A-3: a request with no duration is REFUSED")
    out = set_arming(True, -5.0)
    check("duration_sec is required" in out, "  ... a negative duration is REFUSED")
    out = set_arming(True, 601.0)
    check(
        "REJECTED, not shortened" in out,
        "  ... 601 s is REJECTED against the 600 s ceiling, not clamped",
    )
    check(
        gateway.count(r"DISPATCHING target") == 0,
        "  ... and every refusal left it disarmed, so still nothing dispatched",
    )
    out = set_arming(True, 600.0)
    check("armed for 600" in out, "  ... and exactly 600 s is granted")
    check(
        gateway.wait_for(r"DISPATCHING target", 25) is not None,
        "  ... after which the SAME configuration dispatches, so nothing else blocked it",
    )

    # F-8: a startup-only parameter is REFUSED with a reason, never accepted and
    # then ignored. The four the re-audit named plus the two carried forward,
    # taken against a RUNNING node with a goal in flight - the state in which an
    # operator would actually reach for one of them.
    for name, value in (
        # SAFETY.md F-24. Not one of this node's own parameters - rclpy declares
        # it on every node - which is exactly how it escaped F-8's fix and stayed
        # settable at runtime while every safety timeout was measured on it.
        ("use_sim_time", "true"),
        ("clock_stall_sec", "60.0"),
        # SAFETY.md F-31: it never widens the gate, but a running node must not
        # be able to quieten the report either.
        ("clock_startup_grace_sec", "600.0"),
        ("max_target_list_age_sec", "600.0"),
        ("link_lost_sec", "1.0"),
        ("cancel_confirm_timeout_sec", "1.0"),
        ("navigate_action", "/somewhere_else"),
        ("pick_action", "/somewhere_else"),
        ("arming.max_duration_sec", "60.0"),
        ("allow_arm", "false"),
    ):
        out = set_param("goal_gateway_node", name, value)
        check(
            "Setting parameter failed" in out or "fixed at startup" in out,
            f"F-8: `ros2 param set {name}` is REFUSED, not silently ignored",
            out.strip().splitlines()[-1][:100] if out.strip() else "(no output)",
        )
    check(
        gateway.count(r"re-validating on the next tick") == 0
        or gateway.count(r"REFUSED parameter change") > 0,
        "  ... and the node logs the refusal rather than 'set to X; re-validating'",
    )
    out = set_param("goal_gateway_node", "max_attempts_per_target", "3")
    check(
        "Setting parameter failed" not in out,
        "  (control: a parameter that IS re-read at the point of use still takes it)",
    )
    out = set_param("octopus_link_node", "link_lost_sec", "1.0")
    check(
        "Setting parameter failed" in out,
        "F-8: the link node refuses too - it re-reads no parameter at all",
    )
    teardown()




def scenario_reach_ack() -> None:
    """Decision-2 run 2: auto_pick FALSE - arrival acknowledges NOTHING.

    This scenario used to have a second half that exercised FR-12 item 7's
    weaker branch (acknowledge on reach) behind an opt-in parameter. **User
    decision 2026-08-19: C-7 is normative, FR-12 item 7 is aligned to it, and
    the parameter was removed outright** - so acknowledging on arrival is now a
    FORBIDDEN behaviour rather than a configurable one, and this scenario asserts
    its absence instead of testing it.

    The check is therefore in two parts: the behaviour (arrival acknowledges
    nothing) and the structure (there is no switch that could change that).
    """
    print("=" * 78)
    print("reach_ack - decision 2, run 2: auto_pick FALSE, arrival acknowledges NOTHING")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=2.0)
    link = start_link()
    gateway = start_gateway(auto_pick="false")
    gateway.wait_for(r"navigate_to_pose is available", 60)
    check(
        gateway.wait_for(r"no parameter that can make arrival acknowledge", 15) is not None,
        "the node states at startup that arrival cannot acknowledge, by construction",
    )
    set_arming(True, 300.0)
    check(
        gateway.wait_for(r"REACHED", 60) is not None,
        "with auto_pick false the goal is still driven and REACHED",
    )
    check(
        gateway.wait_for(r"WITHOUT an acknowledgement \(AUTO_PICK_OFF\)", 20) is not None,
        "arrival without a pick does NOT acknowledge (C-7, normative)",
    )
    check(
        gateway.count(r"SR-16: sending PickPlastic") == 0,
        "  ... and no PickPlastic goal is sent with auto_pick false (SR-16 condition 5)",
    )
    check(
        gateway.wait_for(r"BLACKLISTING target .*retrying cannot change", 20) is not None,
        "the target is blacklisted rather than driven to for ever",
    )
    time.sleep(8)
    check(
        gateway.count(r"ACKNOWLEDGED target") == 0,
        "NOTHING is acknowledged, however long it is left running",
    )
    check(fake.count(r"acknowledged as collected") == 0, "  ... and the source never advances")

    # Structure, not only behaviour: the switch that could change this must not
    # exist. A parameter that must never be set is a parameter that must not be
    # present - and this one is checked over the SOURCE, where its absence is a
    # fact, rather than over a runtime log, where it could only ever be silence
    # (the distinction SAFETY.md 6.3 drew when it retired the other grep).
    sources = subprocess.run(
        ["grep", "-rl", "acknowledge_on_reach", os.path.join(_PKG, "src"),
         os.path.join(_PKG, "config"), os.path.join(_PKG, "launch")],
        capture_output=True, text=True,
    ).stdout.split()
    check(
        not sources,
        "and `acknowledge_on_reach` does not exist anywhere in the package",
        ", ".join(sources),
    )
    check(
        "unknown parameter" not in set_arming(True, 60.0).lower(),
        "  (sanity: the service still answers normally without it)",
    )
    check(*_pick_tolerance_gate_structure())
    # And the twin is deliberately NOT affected: this whole scenario ran with
    # `grasp.tolerance_m: TO-VERIFY` and the pick path stayed exercisable.
    check(
        gateway.count(r"REAL ROBOT domain") == 0,
        "  ... and it is silent on the twin, so the decoupling decision is intact",
    )
    teardown()


def scenario_link_reset() -> None:
    """SAFETY.md F-16: a WiFi flap is not a restart of their node.

    The auditor blacklisted a target, forced ONE WebSocket reconnect with their
    node untouched, their ids unchanged and their `collected` flags intact, and
    watched the blacklist and the attempt counts be dropped - because their
    lifetime was keyed on `ExternalLinkStatus.reconnect_count`, which counts
    successful WebSocket connections and nothing else. An unpickable target
    became retryable because the link blinked.

    Both halves are exercised here: the reconnect that must change NOTHING, and
    the one signal their protocol does carry that genuinely means their id space
    restarted - a target we acknowledged, and then watched turn `collected` in
    their own list, coming back uncollected.
    """
    print("=" * 78)
    print("link_reset - F-16: a reconnect keeps the blacklist; an id-space reset drops it")
    print("=" * 78)
    fake = start_fake()
    mock = start_mock(nav_duration_sec=2.0, pick_duration_sec=0.5, pick_outcome="fail")
    link = start_link()
    gateway = start_gateway()
    gateway.wait_for(r"pick_plastic is available", 60)
    set_arming(True, 600.0)
    check(
        gateway.wait_for(r"BLACKLISTING target 3", 90) is not None,
        "a target that cannot be picked is blacklisted after max_attempts_per_target",
    )

    # -- the reconnect that must change nothing ---------------------------
    fake.send("disconnect")
    kept = gateway.wait_for(r"external link reconnected .*KEEPING the blacklist", 60)
    check(kept is not None,
          "a plain WebSocket reconnect KEEPS the blacklist and says so",
          (kept or "")[-130:])
    check(
        gateway.count(r"dropping the blacklist") == 0,
        "  ... and drops nothing: a transport counter is not evidence about their node",
    )
    time.sleep(6)
    check(
        gateway.count(r"DISPATCHING target 3") <= 2,
        "  ... so the unpickable target is NOT driven to again after the flap",
        f"{gateway.count(r'DISPATCHING target 3')} dispatches, max_attempts is 2",
    )

    # -- the one signal that IS evidence ----------------------------------
    # Remove the blacklisted target so their goal advances, let the next one be
    # picked and acknowledged, and wait until their own list reports it
    # collected - that observation is what makes the later contradiction
    # evidence rather than a guess about a lost acknowledgement.
    fake.send("remove 3")
    set_mock(pick_outcome="succeed")
    set_arming(True, 600.0)
    acked = gateway.wait_for(r"ACKNOWLEDGED target (\S+) as COLLECTED", 90)
    check(acked is not None, "another target is picked and acknowledged normally")
    check(
        fake.wait_for(r"acknowledged as collected", 20) is not None,
        "  ... and their list reports it collected, which is what we watch for",
    )
    time.sleep(4)
    set_arming(False)
    time.sleep(2)

    fake.send("restart")
    reset = gateway.wait_for(r"their id space RESTARTED", 30)
    check(reset is not None,
          "an id we acknowledged AND saw collected coming back uncollected is a reset",
          (reset or "")[-130:])
    check(
        gateway.count(r"dropping the blacklist") == 1,
        "  ... and THAT drops the blacklist - once, on evidence, not on a counter",
    )
    # Retired log grep, replaced by the mechanism checks (SAFETY.md 6.3).
    check(*_no_service_clients_structure())
    check(*_forbidden_topics_structure())
    check(*_sweep_called_structure())
    teardown()



def _forward_jump_probe(tag: str, label: str, mock_sim_time: str,
                        jump_sec: float) -> None:
    """One forward-jump run. OBSERVATIONAL - see `observe`.

    ``mock_sim_time`` is the whole reason this runs twice. Every age the gateway
    computes is `its own ROS clock` minus `a stamp`, and where that stamp comes
    from decides what a forward jump does to it:

    * the mock on WALL time (the suite's default elsewhere) stamps TF and
      odometry with wall time while the gateway reads sim time, so a forward
      jump opens a PERMANENT offset between the two - those ages never recover;
    * the mock on SIM time is what a real Gazebo is: its stamps jump with the
      clock, so those ages recover on the next message.

    A probe that ran only the first would report a consequence that is an
    artefact of the fixture, which is exactly the trap F-40 is being reproduced
    to avoid. One age is the same in both runs and it is worth naming:
    `/teleop/active_mode` is a `std_msgs/String` with no stamp at all, so its
    age is measured against the GATEWAY's own reception time - it goes large on
    the jump and recovers one message later whatever the producer's clock is.
    """
    print("-" * 78)
    print(f"  {label}")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=120.0, pick_duration_sec=1.0,
                      use_sim_time=mock_sim_time)
    clock = start_sim_clock()
    link = start_link()
    gateway = start_gateway(use_sim_time="true")
    if gateway.wait_for(r"ROS clock observed advancing", 40) is None:
        check(False, f"{tag}: the clock must be proven before the jump means anything")
        _keep_logs(tag)
        teardown()
        shm_clean()
        return
    set_arming(True, 300.0)
    if gateway.wait_for(r"DISPATCHING target", 60) is None:
        check(False, f"{tag}: a goal must be in flight before the jump")
        _keep_logs(tag)
        teardown()
        shm_clean()
        return

    patterns = {
        "dispatch": r"DISPATCHING target",
        "cancel": r"CANCELLING target",
        "disarm": r"disarmed by",
        "tf_stale": r"TF_STALE",
        "tf_unavailable": r"TF_UNAVAILABLE",
        "mode_stale": r"MODE_STALE",
        "advancing": r"ROS clock observed advancing",
        "arrived": r"ARRIVED at target",
        "acked": r"ACKNOWLEDGED target",
    }
    before = {key: gateway.count(rx) for key, rx in patterns.items()}
    diag_before = echo_once("/diagnostics", timeout=20.0, expect="external/clock")

    set_sim_clock_param("jump_forward_sec", f"{jump_sec:.1f}")
    jump_line = clock.wait_for(r"JUMPED FORWARD", 20)
    check(
        jump_line is not None,
        f"{tag}: the fixture really did step the clock forward {jump_sec:.0f} s "
        "(sim_clock.jump_forward_sec, added for F-40)",
        (jump_line or "").split("]: ")[-1][:70],
    )
    jumped_at = time.time()

    # 60 s, and the number matters: the auditor's prediction is that dispatch
    # RESUMES "one TF and one mode message later", i.e. within ~50 ms. Anything
    # that has not resumed in 60 wall-seconds has not resumed.
    time.sleep(60)
    after = {key: gateway.count(rx) for key, rx in patterns.items()}
    delta = {key: after[key] - before[key] for key in patterns}
    diag_after = echo_once("/diagnostics", timeout=20.0, expect="external/clock")

    # DECIDED 2026-08-20 (user), so this half is a CHECK and no longer an
    # observation: report it - WARN plus a /diagnostics value - and do not
    # disarm and do not cancel. Before the decision this measured 0 in every
    # run, which is what the decision was taken on.
    warned = gateway.wait_for(r"the ROS clock JUMPED FORWARD", 20)
    check(
        warned is not None,
        f"{tag}: F-40 - the forward jump is REPORTED at WARN. It produced "
        "nothing at all before the user's decision of 2026-08-20: no log line, "
        "no diagnostic, and after the F-38 fix not even the accidental cancel",
        (warned or "").split("]: ")[-1][:100],
    )
    check(
        gateway.count(r"the ROS clock JUMPED FORWARD") == 1,
        "  ... exactly once, not once per tick - a jump is an event, and a "
        "mechanism that cries wolf gets disabled by whoever is on shift",
        f"{gateway.count(r'the ROS clock JUMPED FORWARD')} report(s)",
    )
    observe(
        f"{tag}: what /diagnostics external/clock says after it",
        "message="
        + _diag_field(diag_after, "external/clock", "message")
        + ", forward_jumps="
        + _diag_field(diag_after, "external/clock", "forward_jumps")
        + ", last_forward_jump_sec="
        + _diag_field(diag_after, "external/clock", "last_forward_jump_sec"),
    )
    observe(
        f"{tag}: is the clock still PROVEN - i.e. did the jump re-baseline?",
        f"clock_advancing {_diag_field(diag_before, 'external/clock', 'clock_advancing')}"
        f" -> {_diag_field(diag_after, 'external/clock', 'clock_advancing')}, "
        f"stall reports={gateway.count(r'the ROS clock (STOPPED|has NEVER advanced)')}, "
        f"backwards-jump reports={gateway.count(r'JUMPED BACKWARDS')}, "
        f"new 'observed advancing' lines={delta['advancing']}",
    )
    observe(
        f"{tag}: do the ROS-clock ages go large at once?",
        f"new MODE_STALE={delta['mode_stale']}, new TF_STALE={delta['tf_stale']}, "
        f"new TF_UNAVAILABLE={delta['tf_unavailable']}",
    )
    observe(
        f"{tag}: is the goal in flight CANCELLED?",
        f"new CANCELLING lines={delta['cancel']}"
        + ("; " + _last_line(gateway, r"CANCELLING target") if delta["cancel"] else ""),
    )
    check(
        delta["disarm"] == 0 and delta["cancel"] == 0,
        f"{tag}: F-40 - and it does NOT disarm and does NOT cancel. That is the "
        "whole of the user's decision: on the robot the only sources are a "
        "time-sync step and a manual `date` set, and disarming on every step of "
        "a flaky NAT'd time source would trade a small reporting gap for an "
        "operational one",
        f"{delta['disarm']} disarm(s), {delta['cancel']} cancel(s)",
    )
    observe(
        f"{tag}: does the ARMING WINDOW survive it?",
        f"new disarm lines={delta['disarm']}; /diagnostics armed "
        f"{_diag_field(diag_after, 'external/arming', 'armed')}, "
        f"seconds_remaining "
        f"{_diag_field(diag_before, 'external/arming', 'seconds_remaining')} -> "
        f"{_diag_field(diag_after, 'external/arming', 'seconds_remaining')}",
    )
    observe(
        f"{tag}: does dispatch RESUME in the {int(time.time() - jumped_at)}s after the jump?",
        f"new DISPATCHING lines={delta['dispatch']}, new arrivals={delta['arrived']}, "
        f"new acknowledgements={delta['acked']}; nav_state "
        f"{_diag_field(diag_before, 'external/dispatch', 'nav_state')} -> "
        f"{_diag_field(diag_after, 'external/dispatch', 'nav_state')}",
    )
    observe(
        f"{tag}: what the gateway last said about dispatching",
        _last_line(gateway, r"not dispatched|dispatchable but held")
        + f"; /diagnostics external/goals last_reason "
        + _diag_field(diag_before, "external/goals", "last_reason")
        + " -> "
        + _diag_field(diag_after, "external/goals", "last_reason"),
    )

    directory = os.path.join(LOG_DIR, CURRENT_SCENARIO or "scenario")
    with open(os.path.join(directory, f"diagnostics_{tag}_after.txt"), "w") as fh:
        fh.write(diag_after)
    _keep_logs(tag)
    teardown()
    shm_clean()


def _keep_logs(tag: str) -> None:
    """Both runs write the same file names, so the first was overwritten by the
    second and only one of the two could ever be read afterwards. Found the
    usual way."""
    directory = os.path.join(LOG_DIR, CURRENT_SCENARIO or "scenario")
    target = os.path.join(directory, tag)
    os.makedirs(target, exist_ok=True)
    for name in os.listdir(directory):
        if name.endswith(".log"):
            shutil.copyfile(os.path.join(directory, name),
                            os.path.join(target, name))


def _last_line(proc: "Proc", pattern: str) -> str:
    hits = [line for line in proc.text().splitlines() if re.search(pattern, line)]
    return hits[-1].split("]: ")[-1][:150] if hits else "<none>"


def _diag_field(blob: str, status_name: str, key: str) -> str:
    """One value out of a `ros2 topic echo` of DiagnosticArray, by status name.

    Text parsing rather than YAML: `level` comes out as "\0"/"\x02", which is
    not valid YAML for every loader, and this is a probe, not a consumer.
    """
    lines = blob.splitlines()
    in_status = False
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("name:"):
            in_status = stripped.split(":", 1)[1].strip() == status_name
        if not in_status:
            continue
        if stripped.startswith(f"{key}:"):
            return stripped.split(":", 1)[1].strip().strip("'\"")
        if stripped == f"- key: {key}" and index + 1 < len(lines):
            return lines[index + 1].split(":", 1)[1].strip().strip("'\"")
    return "<unread>"


def _forward_jump_race(tag: str, jumps: int, jump_sec: float,
                       spacing_sec: float) -> None:
    """How OFTEN does a forward jump have a consequence? SAFETY.md F-40.

    Two single-jump runs disagreed with each other - one cancelled the goal in
    flight, the other did nothing - which is the observation this part exists to
    explain. The mode age is the gateway's own reception time minus now, so a
    jump makes it large for exactly as long as it takes the next
    `/teleop/active_mode` message to arrive: one period of a 20 Hz publisher,
    ~50 ms. The only thing that looks at it is the dispatch tick at 2 Hz. So
    whether a forward jump cancels anything is a RACE between a 50 ms window and
    a 500 ms observer, and a single run can report either answer.

    This repeats the jump and counts. It asserts nothing: what the number is for
    is the user's decision on F-40, where "sometimes, unpredictably, with no
    report either way" is a materially different thing to decide about than
    either "always" or "never".
    """
    print("-" * 78)
    print(f"  {tag} - the same jump {jumps} times: is the consequence a RACE?")
    fake = start_fake()
    mock = start_mock(nav_duration_sec=300.0, pick_duration_sec=1.0,
                      use_sim_time="true")
    clock = start_sim_clock()
    link = start_link()
    gateway = start_gateway(use_sim_time="true")
    if gateway.wait_for(r"ROS clock observed advancing", 40) is None:
        check(False, f"{tag}: the clock must be proven before the jumps mean anything")
        _keep_logs(tag)
        teardown()
        shm_clean()
        return
    set_arming(True, 600.0)
    if gateway.wait_for(r"DISPATCHING target", 60) is None:
        check(False, f"{tag}: a goal must be in flight before the jumps")
        _keep_logs(tag)
        teardown()
        shm_clean()
        return

    cancels = 0
    reported = 0
    disarms_before = gateway.count(r"disarmed by")
    for index in range(jumps):
        before_cancel = gateway.count(r"CANCELLING target .*MODE_STALE")
        # PER ITERATION, not cumulative: this counted the running total once,
        # which made "N of 8" a sum of counts rather than a count of jumps.
        before_report = gateway.count(r"the ROS clock JUMPED FORWARD")
        set_sim_clock_param("jump_forward_sec", f"{jump_sec:.1f}")
        # The FIXTURE's line, not the gateway's - they are deliberately
        # different strings so one cannot be mistaken for the other.
        clock.wait_for(r"/clock JUMPED FORWARD", 20)
        time.sleep(spacing_sec)
        if gateway.count(r"CANCELLING target .*MODE_STALE") > before_cancel:
            cancels += 1
        if gateway.count(r"the ROS clock JUMPED FORWARD") > before_report:
            reported += 1
        # Re-dispatch is what makes the next jump measurable at all; without a
        # goal in flight there is nothing for the supervisor to cancel.
        gateway.wait_for(r"DISPATCHING target", 10)

    observe(
        f"{tag}: forward jumps of {jump_sec:.0f} s that cancelled the goal in flight",
        f"{cancels} of {jumps} - 4 of 8 before F-38 was fixed, when the mode age "
        "was still on the ROS clock and the ~50 ms refusal window raced a 2 Hz "
        "dispatch tick; 0 of 8 after, which is what the F-40 decision was taken on",
    )
    check(
        reported == jumps,
        f"{tag}: F-40 - EVERY one of the {jumps} jumps is reported. This is the "
        "measurement the user decided on inverted: it read 0 of 8 before, and "
        "the accidental cancel that used to cover the case half the time read "
        "4 of 8 before F-38 and 0 of 8 after it",
        f"{reported} of {jumps} reported",
    )
    check(
        gateway.count(r"disarmed by") - disarms_before == 0 and cancels == 0,
        f"  ... and NOT ONE of them disarmed or cancelled - {jumps} consecutive "
        "discontinuities leave the gate exactly where it was",
        f"{gateway.count(r'disarmed by') - disarms_before} disarm(s), "
        f"{cancels} cancel(s)",
    )
    # This was demoted to an observation when it came back `<unread>` in a
    # full-suite run, on the reading that it measured CLI discovery under
    # contention. That reading was WRONG and the demotion is reverted: the real
    # cause was that `/diagnostics` has TWO publishers here - the gateway and
    # the link node - so `--once` was returning the link node's array, which
    # contains no `external/clock` at all. `expect=` now selects the message we
    # mean. The `readable` guard below still covers the genuine transport case,
    # which is the one the demotion was meant for.
    diagnostics = echo_once("/diagnostics", timeout=40.0, expect="external/clock")
    counted = _diag_field(diagnostics, "external/clock", "forward_jumps")
    observe(
        f"  {tag}: /diagnostics external/clock, end to end after {jumps} jumps",
        f"forward_jumps={counted} (expected {jumps}), last_forward_jump_sec="
        + _diag_field(diagnostics, "external/clock", "last_forward_jump_sec")
        + ", message="
        + _diag_field(diagnostics, "external/clock", "message"),
    )
    if readable(diagnostics, "/diagnostics external/clock"):
        check(
            counted.isdigit() and int(counted) == jumps,
            f"  ... and /diagnostics carries the COUNT end to end, so an "
            "operator asking afterwards whether the clock stepped during a run "
            "can find out. There was no key here at all before the decision",
            f"external/clock -> forward_jumps = {counted} (expected {jumps})",
        )
        check(
            _diag_field(diagnostics, "external/clock", "message")
            == "ROS clock is advancing",
            "  ... while the STATUS stays OK and says the clock is advancing, "
            "because report-only means the status that gates nothing must not "
            "start claiming a fault",
            _diag_field(diagnostics, "external/clock", "message"),
        )
    _keep_logs(tag)
    teardown()
    shm_clean()


def scenario_clock_forward() -> None:
    """SAFETY.md F-40, REPRODUCED rather than predicted. No fix, by decision.

    DECIDED 2026-08-20 by the user, ON THIS REPRODUCTION: report a forward
    discontinuity - WARN plus a `/diagnostics` value - and do NOT disarm and do
    NOT cancel. The parts of this scenario that cover the decision are now
    `check`s; the parts that measure behaviour nobody has specified are still
    `[OBS ]`. What follows is the finding as it stood when this was written.

    F-40 says a FORWARD clock jump is the one discontinuity that is neither
    detected, reported nor disarmed on: `_clock_watchdog`'s first branch is
    `ros > self._clock_ref_ros_sec`, which a jump of any size satisfies, so the
    jump re-baselines the reference and PROVES the clock. Its status is
    VERIFIED-CODE with the consequence only SUSPECTED, and the stated reason is
    that `sim_clock.py` had `jump_back_sec` and no `jump_forward_sec`.

    This scenario is that missing parameter being used. It asserts almost
    nothing on purpose: the user's decision ("reproduce first, then decide") is
    still open, and a suite that asserted today's behaviour would make the
    decision by default. What it produces is `[OBS ]` lines - measurements the
    decision can be taken on.

    Run TWICE, with the observation producers on wall time and on sim time; the
    docstring of `_forward_jump_probe` says why that difference is the whole
    experiment.
    """
    print("=" * 78)
    print("clock_fwd - F-40: a FORWARD clock jump, observed (no fix, user decision open)")
    print("=" * 78)
    _forward_jump_probe(
        "G1", "G1 - observation producers on WALL time (stamps do NOT follow the jump)",
        "false", 120.0,
    )
    _forward_jump_probe(
        "G2", "G2 - observation producers on SIM time (a real Gazebo: stamps DO follow)",
        "true", 120.0,
    )
    _forward_jump_race("G3", jumps=8, jump_sec=120.0, spacing_sec=12.0)


SCENARIOS = {
    "permissive": scenario_permissive,
    "clock": scenario_clock,
    "clock_fwd": scenario_clock_forward,
    "stale": scenario_stale,
    "reach_ack": scenario_reach_ack,
    "dispatch": scenario_dispatch,
    "no_ack": scenario_no_ack,
    "ambiguous": scenario_ambiguous,
    "link_reset": scenario_link_reset,
    "triggers": scenario_triggers,
    "sr9": scenario_sr9,
}


def main() -> int:
    global LOG_DIR
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="all", choices=list(SCENARIOS) + ["all"])
    parser.add_argument("--log-dir", default="/tmp/gripperx_stage3")
    args = parser.parse_args()

    if os.environ.get("ROS_DOMAIN_ID") != TWIN_DOMAIN:
        print(f"refusing to run outside ROS_DOMAIN_ID={TWIN_DOMAIN}", file=sys.stderr)
        return 2
    LOG_DIR = args.log_dir
    shutil.rmtree(LOG_DIR, ignore_errors=True)
    os.makedirs(LOG_DIR, exist_ok=True)

    # BEFORE the first scenario, not only between them: a run inherits whatever
    # the previous run leaked, and enough leaked segments make the `ros2` CLI
    # blind while the nodes themselves are fine. See `shm_clean`.
    print("=" * 78)
    shm_clean(announce=True)

    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    global CURRENT_SCENARIO
    try:
        for name in names:
            CURRENT_SCENARIO = name
            SCENARIOS[name]()
            teardown()
            shm_clean()
            time.sleep(3)
    finally:
        teardown()

    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All stage-3 twin checks passed.")
    return 0


LOG_DIR = "/tmp/gripperx_stage3"
CURRENT_SCENARIO = ""

if __name__ == "__main__":
    sys.exit(main())
