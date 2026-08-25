"""``DiagnosticArray`` assembly, by hand.

WHY NOT ``diagnostic_updater`` - this is a deployment constraint, not taste
=========================================================================
The Nav2 core packages this workspace uses come from ``.rosdeps_local`` and were
built against a **newer ``diagnostic_updater`` ABI** than the laptop's system
ROS. ``scripts/sim_env_nav2.sh`` therefore does not touch ``LD_LIBRARY_PATH``
globally; only the Nav2 C++ nodes get those libs prepended, per process, via
``additional_env=GRIPPERX_ROSDEPS_LIB`` in the launch files.

Linking a pure-python node of ours against ``diagnostic_updater`` would drag
that ABI question into a node that has no reason to care about it, and the
launch file would have to pick a side. ``diagnostic_msgs`` is plain generated
IDL with no such problem, so the array is built by hand here - about thirty
lines - and ``octopus_link.launch.py`` can keep ``additional_env={}``. That
reasoning is repeated at the call site in the launch file, because that is
where somebody will be tempted to "fix" it.

SR-13 is the requirement being served: a command path that has stopped working
must produce an **active signal**, not an absence. Every our-side failure
(``NAV2_UNAVAILABLE``, ``TF_UNAVAILABLE``, ``LINK_LOST``, internal) is an ERROR
here as well as an ERROR in the log; client-caused rejections are WARN and never
escalate, so the ERROR level keeps meaning "our stack is broken".
"""

from __future__ import annotations

from typing import Iterable, Mapping, Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

#: Hardware id for every status this package emits. Not a device - this whole
#: path is software - but consumers group by it, so it is set consistently.
HARDWARE_ID = "gripperx_external"


def _value(key: str, value: object) -> KeyValue:
    kv = KeyValue()
    kv.key = str(key)
    if value is None:
        # Explicit, so a consumer cannot read a missing measurement as an empty
        # string that happens to parse as zero (FR-12 item 8).
        kv.value = "unavailable"
    elif isinstance(value, bool):
        kv.value = "true" if value else "false"
    elif isinstance(value, float):
        kv.value = f"{value:.6g}"
    else:
        kv.value = str(value)
    return kv


def _level(level: object) -> bytes:
    """``DiagnosticStatus.level`` is a ``byte`` field.

    rosidl maps ``byte`` to a single-byte ``bytes`` object in Python, not to an
    int, so an int assigned straight in fails the field's type check. The
    ``DiagnosticStatus.OK``/``WARN``/``ERROR``/``STALE`` constants are already
    in that form; this only exists so a caller may pass either.
    """
    if isinstance(level, (bytes, bytearray)):
        return bytes(level)
    return bytes([int(level)])  # type: ignore[arg-type]


def status(
    name: str,
    level: object,
    message: str,
    values: Optional[Mapping[str, object]] = None,
    hardware_id: str = HARDWARE_ID,
) -> DiagnosticStatus:
    """One ``DiagnosticStatus``. ``values`` keeps the machine-readable detail."""
    st = DiagnosticStatus()
    st.name = name
    st.level = _level(level)
    st.message = message
    st.hardware_id = hardware_id
    st.values = [_value(k, v) for k, v in (values or {}).items()]
    return st


def array(stamp, statuses: Iterable[DiagnosticStatus]) -> DiagnosticArray:
    """Wrap statuses into a stamped ``DiagnosticArray``."""
    msg = DiagnosticArray()
    msg.header.stamp = stamp
    msg.status = list(statuses)
    return msg


# ---------------------------------------------------------------------------
# the statuses this package publishes
# ---------------------------------------------------------------------------
def link_status(
    connected: bool,
    url: str,
    last_message_age_sec: float,
    reconnects: int,
    link_lost_sec: float,
    extra: Optional[Mapping[str, object]] = None,
) -> DiagnosticStatus:
    """Health of the rosbridge link.

    A dead link is an ERROR rather than a WARN because it is an auto-disarm
    trigger (SR-15 rule 10): the external system can no longer tell us that a
    target became invalid, so continuing would be the unsafe option.
    """
    values = {
        "url": url,
        "connected": connected,
        "last_message_age_sec": None if last_message_age_sec < 0.0 else last_message_age_sec,
        "reconnects": reconnects,
        "link_lost_sec": link_lost_sec,
    }
    values.update(extra or {})
    if not connected:
        return status("external/link", ERROR, "rosbridge link is down", values)
    if last_message_age_sec < 0.0:
        return status(
            "external/link", WARN, "connected, but nothing received yet", values
        )
    if last_message_age_sec > link_lost_sec:
        return status(
            "external/link",
            ERROR,
            f"connected but silent for {last_message_age_sec:.1f}s "
            f"(> link_lost_sec {link_lost_sec:.1f}s)",
            values,
        )
    return status("external/link", OK, "connected", values)


def arming_status(
    armed: bool,
    seconds_remaining: float,
    allow_arm: bool,
    dry_run: bool,
    last_disarm_trigger: str,
    disarm_error: bool,
    extra: Optional[Mapping[str, object]] = None,
) -> DiagnosticStatus:
    """State of the authority gate.

    Disarmed is **OK, not a warning**: it is the correct, designed, default
    state, and a disarmed gateway is observably useful and observably harmless.
    Reporting it as a problem would train the operator to ignore the field.
    An unexpected auto-disarm trigger (anything but OPERATOR/TIMEOUT) is the
    ERROR case that SR-15 rule 7 asks for.
    """
    values = {
        "armed": armed,
        "seconds_remaining": seconds_remaining if armed else None,
        "allow_arm": allow_arm,
        "dry_run": dry_run,
        "last_disarm_trigger": last_disarm_trigger or None,
    }
    values.update(extra or {})
    if disarm_error:
        return status(
            "external/arming",
            ERROR,
            f"auto-disarmed by {last_disarm_trigger}",
            values,
        )
    if armed:
        return status(
            "external/arming", OK, f"armed, {seconds_remaining:.0f}s remaining", values
        )
    return status("external/arming", OK, "disarmed (default state)", values)


def goal_status(
    last_reason: str,
    last_severity: str,
    received: int,
    accepted: int,
    rejected: int,
    preview: int,
    blacklisted: Sequence[str] = (),
    extra: Optional[Mapping[str, object]] = None,
) -> DiagnosticStatus:
    """Verdict counters, with the log-level/severity rule of FR-12 item 6.

    ``LOCAL`` severity - our stack failed - is an ERROR. ``CLIENT`` severity -
    the peer or the world caused it - is a WARN and never more, so an operator
    who sees ERROR here knows to look at us and not at the Octopus.
    """
    values = {
        "received": received,
        "accepted": accepted,
        "rejected": rejected,
        "preview": preview,
        "last_reason": last_reason or None,
        "blacklisted": ", ".join(blacklisted) if blacklisted else None,
        "blacklisted_count": len(blacklisted),
    }
    values.update(extra or {})
    if last_severity == "LOCAL":
        return status(
            "external/goals", ERROR, f"our-side failure: {last_reason}", values
        )
    if blacklisted:
        # Loud on purpose: their protocol has no failure channel, so a
        # blacklisted target stalls their mission permanently (FR-12 item 7).
        return status(
            "external/goals",
            WARN,
            f"{len(blacklisted)} target(s) blacklisted; the source's mission "
            "cannot advance past them",
            values,
        )
    if last_reason:
        return status("external/goals", WARN, f"last rejection: {last_reason}", values)
    return status("external/goals", OK, f"{accepted} accepted, {preview} previewed", values)


def clock_status(
    proven: bool,
    stalled: bool,
    frozen_for_sec: float,
    use_sim_time: bool,
    stall_sec: float,
    startup_grace: bool = False,
    clock_publishers: int = 0,
    forward_jumps: int = 0,
    last_forward_jump_sec: float = 0.0,
    forward_jump_sec: Optional[float] = None,
    extra: Optional[Mapping[str, object]] = None,
) -> DiagnosticStatus:
    """Is the clock every timeout in the gateway is measured on still moving?

    ERROR, not WARN, and for the same reason ``link_status`` is: a clock that
    has stopped disables the arming expiry, the link watchdog, the cancel
    report and the in-flight re-validation ALL AT ONCE, and none of them can
    report it because each of them is one of the timers that stopped
    (SAFETY.md F-24).

    "Not yet proven" is the same level with ONE exception, and the exception is
    SAFETY.md F-31: while the node is still inside its measured ``/clock``
    discovery grace, an unproven clock is WARN. It was ERROR from the very first
    diagnostic tick under every configuration, wall time included, so an
    ordinary twin start put an ERROR item on ``/diagnostics`` for the whole
    pre-proof interval - and an ERROR that fires on ordinary starts is an ERROR
    an operator learns to scroll past, which is the failure F-24 was raised
    about wearing different clothes. ``startup_grace`` is the caller's answer to
    "has this had a fair chance to discover a publisher yet"; it grades the
    REPORT only. Arming is refused for an unproven clock either way, and the
    gateway's own gate does not read this status at all.

    One thing this status cannot do on its own: while the clock is frozen the
    timer that publishes ``/diagnostics`` has stopped too, so the gateway
    publishes this status EVENT-driven from the clock watchdog rather than
    waiting for a tick that will not come.
    """
    values = {
        "use_sim_time": use_sim_time,
        "clock_advancing": proven and not stalled,
        "frozen_for_sec": round(float(frozen_for_sec), 1),
        "clock_stall_sec": stall_sec,
        "startup_grace": bool(startup_grace) and not proven,
        # SAFETY.md F-40, user decision 2026-08-20. Before this there was no key
        # here at all for a forward discontinuity - measured, 0 of 8 jumps
        # produced anything anywhere - so an operator had no way to learn one had
        # happened. Reported as a COUNT plus the last size, because the question
        # after the fact is "did the clock step while that goal was running", and
        # a value that only exists during the event cannot answer it.
        #
        # It does NOT raise the level. A forward jump is REPORT-ONLY by that
        # decision: the gate is untouched, and there is no moment at which the
        # condition clears, so a latched WARN here would be permanent for the
        # life of the node and would bury the stall and the never-proven cases
        # that DO gate. The WARN is the log line; this is the durable value.
        # SAFETY.md F-35. Reported beside `use_sim_time` because the pair is
        # the finding: a live /clock publisher is unremarkable when sim time is
        # on and is an epoch mismatch when it is off. Value only - it gates
        # nothing and it does not raise the level, exactly as the forward-jump
        # keys do not.
        "clock_publishers": int(clock_publishers),
        "forward_jumps": int(forward_jumps),
        "last_forward_jump_sec": round(float(last_forward_jump_sec), 3),
        "clock_forward_jump_sec": (
            "unset" if forward_jump_sec is None else forward_jump_sec
        ),
    }
    values.update(extra or {})
    if not proven:
        return status(
            "external/clock",
            WARN if startup_grace else ERROR,
            (
                "the ROS clock has not been observed to advance yet, inside the "
                "startup grace for /clock discovery; arming is refused until it "
                "does (SAFETY.md F-31)"
                if startup_grace
                else "the ROS clock has never been observed to advance; arming "
                "is refused (SAFETY.md F-24)"
            ),
            values,
        )
    if stalled:
        return status(
            "external/clock",
            ERROR,
            f"the ROS clock has not advanced for {frozen_for_sec:.1f}s; every "
            "timeout in this node is inert and arming is refused (SAFETY.md F-24)",
            values,
        )
    return status("external/clock", OK, "ROS clock is advancing", values)


def config_status(
    unset_items: Sequence[str], extra: Optional[Mapping[str, object]] = None
) -> DiagnosticStatus:
    """Which TO-VERIFY values are still unmeasured.

    WARN and not ERROR: an unmeasured geofence or grasp offset makes the
    gateway *refuse* goals, which is the safe outcome, not a broken one. But it
    must be visible, or "nothing happens" looks like a bug rather than a
    pending bench measurement.
    """
    values = {"unset": ", ".join(unset_items) if unset_items else None}
    values.update(extra or {})
    if unset_items:
        return status(
            "external/config",
            WARN,
            "TO-VERIFY, goals are refused until measured: " + ", ".join(unset_items),
            values,
        )
    return status("external/config", OK, "all measured values configured", values)


def dispatch_status(
    nav_state: str,
    target_id: str,
    correlation: str,
    attempts: int,
    nav2_available: bool,
    cancel_pending: bool,
    cancel_failed: bool,
    ack_suppressed_reason: str = "",
    acknowledged: int = 0,
    extra: Optional[Mapping[str, object]] = None,
) -> DiagnosticStatus:
    """State of the dispatch path: navigation, pick, correlation, cancel.

    The ERROR cases are the three that must never be discovered by reading a
    log afterwards:

    * **a cancel that was not confirmed** - the whole escalation SR-15 rule 9
      permits is this line plus an ERROR log. The gateway does not act on it and
      must not: it has no write access to the motion chain, and the mechanisms
      that own stopping (Nav2's cancel, ``teleop_mux cmd_timeout_sec``, the
      firmware timeout) are upstream of it;
    * **an ambiguous or contradicted correlation** - the acknowledgement cannot
      be taken back, so an undecidable goal has to be a visible, named state and
      not a silently skipped dispatch (SAFETY.md F-6);
    * **``navigate_to_pose`` gone while a goal is in flight** - our-side failure,
      and the ``NAV2_UNAVAILABLE`` auto-disarm trigger.

    A suppressed acknowledgement is a WARN: it is the correct behaviour every
    time it happens, but their protocol has no failure channel, so the mission
    stops advancing and nobody may find that out by noticing nothing happens.
    """
    values = {
        "nav_state": nav_state,
        "target_id": target_id or None,
        "correlation": correlation or None,
        "attempts": attempts,
        "nav2_available": nav2_available,
        "cancel_pending": cancel_pending,
        "acknowledged": acknowledged,
        "ack_suppressed_reason": ack_suppressed_reason or None,
    }
    values.update(extra or {})
    if cancel_failed:
        return status(
            "external/dispatch",
            ERROR,
            "cancel was NOT confirmed; reporting only - this node has no write "
            "access to the motion chain and adds no stop mechanism (SR-15 r.9)",
            values,
        )
    if correlation in ("AMBIGUOUS", "ID_MISMATCH", "TARGETS_STALE"):
        return status(
            "external/dispatch",
            ERROR,
            f"goal correlation {correlation}: refusing to dispatch or acknowledge "
            "a goal we cannot name",
            values,
        )
    if not nav2_available and nav_state in ("navigating", "picking"):
        return status(
            "external/dispatch",
            ERROR,
            "navigate_to_pose disappeared while a goal was in flight",
            values,
        )
    if ack_suppressed_reason:
        return status(
            "external/dispatch",
            WARN,
            f"acknowledgement suppressed ({ack_suppressed_reason}); the source's "
            "mission cannot advance past this target",
            values,
        )
    if cancel_pending:
        return status("external/dispatch", WARN, "cancel in flight", values)
    return status("external/dispatch", OK, f"nav_state={nav_state}", values)


def transform_lock_status(
    subscribed: bool,
    observed: bool,
    state: str,
    transform_ready: Optional[bool],
    locked: bool,
    yaw_zero_rad: Optional[float],
    align_angle_rad: Optional[float],
    last_message_age_sec: float,
    stale_after_sec: float,
    relocks: int,
    last_relock_from_rad: Optional[float] = None,
    last_relock_to_rad: Optional[float] = None,
    last_relock_align_delta_rad: Optional[float] = None,
    extra: Optional[Mapping[str, object]] = None,
) -> DiagnosticStatus:
    """What the Octopus says about the lock its map frame is oriented on.

    REPORT ONLY. Nothing in this package gates on any of it - a re-lock is not
    a disarm trigger, does not cancel a goal and does not refuse a validation.
    What it *should* cause has not been decided, and SAFETY.md F-40 is the
    pattern: get the evidence on the wire first, decide afterwards.

    THE LEVEL DESCRIBES THE LIVE CONDITION, NOT THE HISTORY - and that is the
    same call F-40 made. ``relocks`` is a COUNT plus the two yaw values, because
    the question after the fact is "did their frame re-lock while we were
    aligned to it", and a value that only exists during the event cannot answer
    it. A re-lock does NOT raise the level: there is no moment at which the
    condition clears (nothing here can know that somebody re-measured the
    alignment), so a latched WARN would be permanent for the life of the node
    and would bury the not-ready, no-lock and stale cases, which are live and
    actionable. The loud part of a re-lock is its WARN log line; this is its
    durable value.

    WARN and never ERROR for every case here: ERROR in this package means OUR
    stack is broken (FR-12 item 6), and a transform that is not ready, has no
    lock, or has gone quiet is the counterpart's condition or the network's.
    """
    # `locked` is LIVE - the newest sample. `yaw_zero_rad` is the LAST KNOWN
    # lock, which the tracker keeps across a gap, so the two legitimately
    # disagree while their node republishes `null` through a restart.
    align_deg = (
        None if align_angle_rad is None else align_angle_rad * 180.0 / 3.141592653589793
    )
    values = {
        "subscribed": subscribed,
        "state": state or None,
        "transform_ready": transform_ready,
        "locked": locked,
        "last_known_yaw_zero_rad": yaw_zero_rad,
        "align_angle_rad": align_angle_rad,
        "align_angle_deg": align_deg,
        "last_message_age_sec": None if last_message_age_sec < 0.0 else last_message_age_sec,
        "stale_after_sec": stale_after_sec,
        "relocks": int(relocks),
        "last_relock_from_rad": last_relock_from_rad,
        "last_relock_to_rad": last_relock_to_rad,
        "last_relock_align_delta_rad": last_relock_align_delta_rad,
    }
    values.update(extra or {})

    if not subscribed:
        return status(
            "external/transform_lock",
            OK,
            "not subscribed (transform_status_enabled=false); a re-lock of the "
            "Octopus map frame could not be detected",
            values,
        )
    if not observed:
        return status(
            "external/transform_lock",
            WARN,
            "subscribed, but the Octopus has not reported its transform yet",
            values,
        )
    if last_message_age_sec > stale_after_sec:
        return status(
            "external/transform_lock",
            WARN,
            f"no transform status for {last_message_age_sec:.1f}s "
            f"(> {stale_after_sec:.1f}s); a re-lock happening now would not be seen",
            values,
        )
    if not locked:
        return status(
            "external/transform_lock",
            WARN,
            f"the Octopus map frame carries no startup yaw lock (state={state!r}); "
            "nothing derived from their frame orientation may be trusted",
            values,
        )
    if state != "ready":
        return status(
            "external/transform_lock",
            WARN,
            f"the Octopus transform reports state={state!r}, not 'ready'; it has "
            "a yaw lock but does not claim the transform is usable",
            values,
        )
    if relocks:
        return status(
            "external/transform_lock",
            OK,
            f"ready and locked, but the frame has RE-LOCKED {relocks} time(s) "
            "since this node started - any alignment measured before the last "
            "one is stale (report only, nothing acted on it)",
            values,
        )
    return status("external/transform_lock", OK, "ready, one unchanged lock", values)
