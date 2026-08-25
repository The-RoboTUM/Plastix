"""SR-8 domain guard, shared by both nodes of this package.

SR-8 splits the real robot (``ROS_DOMAIN_ID=20``) from the twin (``220``) so
that a simulated command can never reach a real motor. This package adds a
*foreign* goal source on top of that, so running it against the wrong domain is
the one mistake that would turn a preview into a real dispatch.

The check is therefore made **before anything is created** - no publisher, no
subscription, no action client, no timer - and a mismatch is FATAL and exits.
Same pattern as ``gripperx_teleop/test/check_teleop_manoeuvre_path.py``, which
pins its domain before ``rclpy`` is imported at all; here the expected value
comes from the node's config, so the earliest possible point is immediately
after the parameter is declared.

It lives in its own module rather than being copy-pasted into both nodes on
purpose: two copies of a P0-safety check are two things that can drift apart.
"""

from __future__ import annotations

import os
from typing import Optional

#: The real robot. A twin process on this domain can move real hardware.
REAL_ROBOT_DOMAIN_ID = 20
#: The twin. PlastiX convention: gripper robots in the 20s, twin at +200.
TWIN_DOMAIN_ID = 220
#: The offline acceptance harness (``test/check_stage3_twin.py``), which pins
#: this domain and refuses to run on any other. It is deliberately NOT 220: the
#: laptop runs a real Gazebo/Nav2 twin there, and a harness that discovers those
#: servers would dispatch mock goals into a live simulation.
ACCEPTANCE_DOMAIN_ID = 221

#: The domains on which NOTHING can move a real motor, enumerated rather than
#: inferred. SAFETY.md F-27: the arm's real-robot protections used to be keyed
#: to ``== REAL_ROBOT_DOMAIN_ID``, so every domain that was not exactly 20 -
#: including any domain the robot is renumbered to, and this project renumbered
#: one inside a week - took the permissive branch silently. The polarity is now
#: the other way round: a domain is permissive only if it is named here, and
#: anything unknown is treated as a real robot.
#:
#: Adding a domain to this set is the one edit in this package that can make a
#: real machine take a simulation branch. Both members are simulation-only by
#: construction: 220 is the twin, 221 exists only inside the offline harness.
SIMULATION_DOMAIN_IDS = frozenset({TWIN_DOMAIN_ID, ACCEPTANCE_DOMAIN_ID})

#: SAFETY.md F-34. The invariant that makes the paragraph above enforceable
#: rather than merely written down: the real robot's domain may NEVER be a
#: member of the permissive set. This is an IMPORT-TIME check on purpose - it
#: cannot be reached late, cannot be configured away and cannot be skipped by a
#: code path that happens not to run, so the one dangerous edit in this file
#: stops being possible instead of being discouraged. It costs nothing at
#: runtime: it runs once, when the module is first imported, which on both
#: nodes is before the domain guard is called and therefore before anything is
#: created.
if REAL_ROBOT_DOMAIN_ID in SIMULATION_DOMAIN_IDS:
    raise RuntimeError(
        f"SIMULATION_DOMAIN_IDS contains the REAL ROBOT's domain "
        f"({REAL_ROBOT_DOMAIN_ID}). Every simulation-only relaxation in this "
        "package - the arm's arrival check among them (SAFETY.md F-14/F-27) - "
        "keys off this set, so that edit would make the real machine take the "
        "simulation branch. This module refuses to import (SAFETY.md F-34)."
    )

#: ``expected_domain_id`` must be set explicitly. -1 is the "unset" marker, and
#: it is a FATAL configuration error rather than a permissive default: a node
#: that decides for itself which domain it is happy on is exactly the hole SR-8
#: exists to close.
UNSET_DOMAIN_ID = -1


class DomainMismatch(RuntimeError):
    """Raised when the live domain is not the configured one."""


def effective_domain_id() -> int:
    """``ROS_DOMAIN_ID`` as the middleware sees it.

    An unset variable means domain 0 to DDS, so it is reported as 0 rather than
    as "unknown" - the guard must compare what the RMW will actually use, not
    what the shell happens to have exported.
    """
    raw = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if not raw:
        return 0
    try:
        return int(raw)
    except ValueError as exc:
        raise DomainMismatch(f"ROS_DOMAIN_ID is not an integer: {raw!r}") from exc


def is_simulation_domain(domain_id: Optional[int] = None) -> bool:
    """Is this a domain on which no real motor can be reached?

    The safety-relevant question is never "is this domain 20"; it is "is this
    provably a simulation". Those are different questions the moment anybody
    renumbers anything, and the answer to the second one has to fail towards
    "no" (SAFETY.md F-27).

    Pure, so the offline checks can exercise both branches without a node and
    without ever touching the real robot's domain.
    """
    actual = effective_domain_id() if domain_id is None else int(domain_id)
    return actual in SIMULATION_DOMAIN_IDS


def check_domain(expected_domain_id: int) -> None:
    """Raise :class:`DomainMismatch` unless the live domain matches.

    Pure, so the offline checks can exercise every branch without a node.
    """
    expected = int(expected_domain_id)
    if expected == UNSET_DOMAIN_ID:
        raise DomainMismatch(
            "expected_domain_id is unset. It must name the domain this node is "
            f"allowed to run on ({TWIN_DOMAIN_ID} = twin, {REAL_ROBOT_DOMAIN_ID} "
            "= real robot, SR-8). There is deliberately no default."
        )
    actual = effective_domain_id()
    if actual != expected:
        extra = ""
        if not is_simulation_domain(actual):
            extra = (
                " This domain is not a known simulation domain "
                f"({sorted(SIMULATION_DOMAIN_IDS)}), so it is treated as a REAL "
                "ROBOT's domain - a twin configuration running here could reach "
                "real motors (SR-8, SAFETY.md F-27)."
            )
        raise DomainMismatch(
            f"ROS_DOMAIN_ID is {actual}, but this configuration requires "
            f"{expected}.{extra} Source the matching environment "
            "(scripts/sim_env_nav2.sh for the twin) and start again."
        )


def enforce_domain(node, expected_domain_id: int, logger=None) -> None:
    """Guard for use inside a node: log FATAL and exit on a mismatch.

    Call this **before creating any publisher, subscription, service, action
    client or timer**. Exiting is the correct response and not an
    over-reaction: there is no degraded mode in which running on the wrong
    domain is acceptable, and a node that merely warns will be left running.
    """
    log = logger if logger is not None else node.get_logger()
    try:
        check_domain(expected_domain_id)
    except DomainMismatch as exc:
        log.fatal(f"SR-8 domain guard: {exc}")
        raise SystemExit(2) from exc
    log.info(
        f"SR-8 domain guard: ROS_DOMAIN_ID={effective_domain_id()} matches "
        f"expected_domain_id={int(expected_domain_id)}"
    )


def clock_publisher_warning(
    publisher_count: int, use_sim_time: bool, when: str
) -> Optional[str]:
    """The MIRROR of the sim-time refusal. SAFETY.md F-35, user decision 2026-08-20.

    Package D closes one direction of the sim-time divergence - ``use_sim_time:
    true`` where no ``/clock`` can exist - and closes it twice over. **The other
    direction was untouched:** ``use_sim_time:=false`` while a ``/clock``
    publisher is live. Nothing in this package looked for a ``/clock`` publisher
    at all; ``count_publishers`` appeared once in the whole package, for
    ``link_status``. This function is the missing half, and it makes the pair
    symmetrical.

    WHY IT WARNS AND DOES NOT REFUSE, WHICH IS NOT THE SAME CHOICE AS PACKAGE D's.
    The forward direction is a configuration that CANNOT work: sim time with no
    ``/clock`` is a clock pinned at zero with every safety timer pinned to it, so
    refusing to start removes goals and nothing else. This direction is different
    in kind - wall time IS a working clock, and the node's timers all run. What is
    wrong is that our ages are computed against a different epoch from the one the
    ``/clock`` publisher is stamping in. That is a mismatch to REPORT, not a
    configuration to refuse, and the reporting-only shape is the same one the user
    chose for the forward clock jump (F-40): no disarm, no cancel, no new refusal
    path, no tenth trigger. The gate is untouched.

    **THE CONSEQUENCE OF THE MISMATCH IS STILL `SUSPECTED`, NOT OBSERVED.** The
    auditor predicts it is fail-safe - ages computed against a mismatched epoch
    come out large and ``validate_goal`` refuses - and says plainly that this is a
    prediction. Showing it needs the real Gazebo twin, and **the user decided on
    2026-08-20 not to make that run**. So this function detects and reports the
    CONDITION; nothing here is evidence about what the condition does, and F-35
    stays open on that half.

    ``when`` names the moment, because the two moments answer different
    questions: at startup DDS discovery may not have matched the publisher yet,
    so a clean start is not proof of absence - which is exactly why the auditor
    asked for the check to run again on the first ``/clock`` message.

    Pure and rclpy-free, so both branches are exercised offline without a node,
    a context or a domain.
    """
    if use_sim_time or int(publisher_count) <= 0:
        return None
    return (
        f"a /clock publisher is LIVE ({int(publisher_count)} publisher(s), seen "
        f"{when}) while use_sim_time is FALSE. This node therefore measures TF "
        "ages, the arming window and the link watchdog on WALL time, while "
        "whatever publishes /clock - a Gazebo twin, or a bag replay - is "
        "stamping in SIM time. The two epochs are unrelated, so every age "
        "computed across them is meaningless (SAFETY.md F-35). This is the "
        "MIRROR of the use_sim_time-without-/clock case, which is refused at "
        "startup. It is REPORTED and not refused: wall time is a working clock "
        "and every timer here runs, so nothing is disarmed, nothing is "
        "cancelled and the arming gate is untouched. If this is the twin, start "
        "with use_sim_time:=true; if it is a bag replay, stop the replay or set "
        "sim time deliberately."
    )


def describe(expected_domain_id: Optional[int] = None) -> str:
    """One-line summary for a startup banner."""
    if expected_domain_id is None:
        return f"ROS_DOMAIN_ID={effective_domain_id()}"
    return (
        f"ROS_DOMAIN_ID={effective_domain_id()} "
        f"(expected {int(expected_domain_id)})"
    )
