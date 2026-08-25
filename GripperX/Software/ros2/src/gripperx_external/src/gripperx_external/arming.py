"""Layer 1 of the two-layer authority gate (SR-1): the arming state machine.

Layer 2 is the existing ``teleop_mux``, which must be in ``autonomous`` before
Nav2 output reaches ``/cmd_vel`` at all, and which stays operator-owned. This
module is the layer we add, and it is deliberately the weaker of the two: it can
only ever *withhold* authority, never grant it to the motion chain.

Hard rules, each enforced here rather than merely documented:

* **Default disarmed, and no way around it.** The constructor takes no initial
  state. No parameter, launch argument or environment variable may produce an
  armed state at startup; ``allow_arm`` can only ever *permit* a later explicit
  service call, never perform one. This is why :meth:`arm` requires a caller and
  a duration and cannot be reached from configuration.
* **Every arm has an explicit, finite duration, and the duration is enforced at
  READ time.** There is no indefinite arm and no "0 means forever" - a zero or
  negative duration is refused. The armed state is not a stored flag that a
  timer eventually corrects: :meth:`ArmingMachine.is_armed` takes the time it is
  asked about, so a starved poll can delay the *report* of an expiry but never
  the expiry itself (SAFETY.md F-2).
* **Arming is never a wire command.** This machine is driven by a local ROS
  service on our own domain; nothing in the external payload vocabulary reaches
  it. Enforced structurally: no method here takes an external message.
* **Disarming cancels, it does not command.** A disarm returns
  ``requires_cancel``; the caller cancels the Nav2 goal and any running pick. It
  must NEVER publish zeros on a chain topic - that would make the gateway a
  second writer (SR-9/SR-10). Stopping is Nav2's job; ``teleop_mux
  cmd_timeout_sec: 0.5`` and the firmware ``CMD_TIMEOUT_MS`` (1 s, verified
  2026-08-17) remain the backstops (NFR-4).
* **A failed cancel is reported, never escalated.** The caller logs ERROR and
  raises an ERROR diagnostic, and that is the whole response. It does NOT publish
  ``/teleop/set_mode: keyboard``, and no parameter to make it do so exists
  anywhere in this package - user decision, 2026-08-18, superseding the earlier
  ``default false`` escape hatch. Publishing that topic would add a second writer
  to a mode-arbitration topic (SR-9/OP-19) and would let the external path arm the
  very chain it is gated by (SR-15 rule 6). Nothing in this package publishes on
  ``/teleop/set_mode`` or on any motion-chain topic.

Pure module: no rclpy. Time enters through an injected ``now_sec`` argument, so
the timeout path is testable without waiting.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# --- disarm triggers -------------------------------------------------------
# Values match the uint8 constants in gripperx_external_msgs/ArmingState.
TRIGGER_NONE = "NONE"
TRIGGER_OPERATOR = "OPERATOR"
TRIGGER_TIMEOUT = "TIMEOUT"
TRIGGER_LINK_LOST = "LINK_LOST"
#: Covers the spacebar E-stop, which publishes ``set_mode=keyboard`` per SR-2.
TRIGGER_MODE_CHANGE = "MODE_CHANGE"
TRIGGER_NAV2_UNAVAILABLE = "NAV2_UNAVAILABLE"
TRIGGER_NODE_SHUTDOWN = "NODE_SHUTDOWN"
TRIGGER_EXCESSIVE_ABORTS = "EXCESSIVE_ABORTS"
#: The ROS clock the gate's own timeout is measured against stopped advancing
#: (SAFETY.md F-24). Every other trigger here is an observation of the world;
#: this one is an observation of the mechanism that makes the other
#: observations timely, so it cannot itself be judged on that clock - the
#: gateway drives it from a monotonic reference.
TRIGGER_CLOCK_STALLED = "CLOCK_STALLED"
#: The ROS clock went BACKWARDS (SAFETY.md F-30). A separate trigger from
#: :data:`TRIGGER_CLOCK_STALLED` by an explicit user decision (SR-15 rule 7,
#: 2026-08-19): a stopped clock and a discontinuous one are different
#: conditions with different operator responses - "your clock publisher is
#: dead" against "somebody reset the world" - and that difference has to be
#: machine-distinguishable, not merely readable in the detail string. The
#: behaviour behind the two codes is deliberately identical.
TRIGGER_CLOCK_JUMPED_BACK = "CLOCK_JUMPED_BACK"

TRIGGER_CODES = {
    TRIGGER_NONE: 0,
    TRIGGER_OPERATOR: 1,
    TRIGGER_TIMEOUT: 2,
    TRIGGER_LINK_LOST: 3,
    TRIGGER_MODE_CHANGE: 4,
    TRIGGER_NAV2_UNAVAILABLE: 5,
    TRIGGER_NODE_SHUTDOWN: 6,
    TRIGGER_EXCESSIVE_ABORTS: 7,
    TRIGGER_CLOCK_STALLED: 8,
    TRIGGER_CLOCK_JUMPED_BACK: 9,
}

#: Triggers that are part of normal operation. Everything else additionally
#: raises an ERROR-level diagnostic (SR-13: an active signal, not an absence).
_EXPECTED_TRIGGERS = frozenset({TRIGGER_OPERATOR, TRIGGER_TIMEOUT})

REQUIRED_TELEOP_MODE = "autonomous"


@dataclass(frozen=True)
class ArmResult:
    granted: bool
    message: str
    reason: str = ""


@dataclass(frozen=True)
class DisarmEvent:
    """Emitted every time the machine leaves the armed state.

    ``requires_cancel`` is always true when the machine was armed: cancelling is
    unconditional, because the whole point of a disarm is that whatever is
    running must stop being our responsibility.
    """

    trigger: str
    detail: str
    at_sec: float
    requires_cancel: bool
    #: True when this disarm must additionally raise an ERROR diagnostic.
    diagnostic_error: bool

    @property
    def trigger_code(self) -> int:
        return TRIGGER_CODES.get(self.trigger, 0)


class ArmingMachine:
    """Arming window with auto-disarm.

    Every ``note_*`` method is an observation of the world; each returns a
    :class:`DisarmEvent` when that observation closed the gate, otherwise
    ``None``. The caller must act on a returned event (cancel + publish
    ``ArmingState`` + diagnostic) - a dropped event means an armed-looking
    system that is not armed, or worse.
    """

    def __init__(
        self,
        allow_arm: bool = False,
        max_duration_sec: float = 120.0,
        max_consecutive_aborts: int = 3,
        auto_pick_available: bool = False,
    ) -> None:
        # NOTE: there is deliberately no `initial_armed`, `arm_on_start` or
        # equivalent. Adding one would violate SR-1 no matter how it defaults.
        self._armed = False
        self._expires_at_sec: Optional[float] = None
        self._armed_by = ""
        self.allow_arm = bool(allow_arm)
        self.max_duration_sec = float(max_duration_sec)
        self.max_consecutive_aborts = int(max_consecutive_aborts)
        self.auto_pick_available = bool(auto_pick_available)
        self.consecutive_aborts = 0
        self.last_disarm_trigger = TRIGGER_NONE
        self.last_disarm_detail = ""
        self.last_disarm_cancelled = False
        self.arm_count = 0

    # -- state ----------------------------------------------------------
    def is_armed(self, now_sec: float) -> bool:
        """Authoritative armed state, **evaluated at read time**.

        There is deliberately no ``armed`` property. A stored flag is only as
        current as the last :meth:`poll`, so a consumer could read ``True`` from
        a window that expired seconds ago whenever the polling timer was
        starved - SAFETY.md F-2. Expiry is a property of the clock, not of a
        timer having run, so every reader passes the time it is reasoning
        about and gets the answer for that instant.

        Pure: no state change, no event. Closing the gate and emitting the
        :class:`DisarmEvent` (cancel + ``ArmingState`` + diagnostic) is still
        :meth:`poll`'s job; this method only guarantees that nobody can *act*
        on an expired window in the gap before the next poll.
        """
        if not self._armed:
            return False
        if self._expires_at_sec is None:
            # Unreachable: `arm` always sets an expiry. Treated as closed
            # rather than open, because an armed state without a bound is
            # exactly the thing rule 5 forbids.
            return False
        return now_sec < self._expires_at_sec

    @property
    def armed_by(self) -> str:
        return self._armed_by

    @property
    def expires_at_sec(self) -> Optional[float]:
        return self._expires_at_sec

    def seconds_remaining(self, now_sec: float) -> float:
        if not self._armed or self._expires_at_sec is None:
            return 0.0
        return max(0.0, self._expires_at_sec - now_sec)

    # -- operator actions -----------------------------------------------
    def arm(self, duration_sec: float, requested_by: str, now_sec: float) -> ArmResult:
        """Explicit arming request. The ONLY way into the armed state."""
        if not self.allow_arm:
            return ArmResult(
                False,
                "arming is not permitted in this configuration (allow_arm is false)",
                "ARM_NOT_PERMITTED",
            )
        if duration_sec is None or not (duration_sec > 0.0):
            return ArmResult(
                False,
                "duration_sec must be > 0; there is no indefinite arming window",
                "INVALID_DURATION",
            )
        if duration_sec > self.max_duration_sec:
            return ArmResult(
                False,
                f"duration_sec {duration_sec:.1f} exceeds max_duration_sec "
                f"{self.max_duration_sec:.1f}",
                "DURATION_TOO_LONG",
            )
        if not requested_by:
            # The audit trail is the point: an anonymous arm cannot be traced
            # back to a person, which is the one thing SR-1 asks for.
            return ArmResult(False, "requested_by must be set", "NO_REQUESTER")

        self._armed = True
        self._expires_at_sec = now_sec + float(duration_sec)
        self._armed_by = requested_by
        self.arm_count += 1
        # A fresh window starts a fresh abort budget; the aborts that mattered
        # were the ones inside the previous window.
        self.consecutive_aborts = 0
        return ArmResult(
            True, f"armed for {duration_sec:.1f}s (requested by {requested_by})"
        )

    def disarm(
        self, trigger: str, now_sec: float, detail: str = ""
    ) -> Optional[DisarmEvent]:
        """Leave the armed state. Idempotent: a disarm while disarmed is a
        no-op and returns ``None``, so repeated triggers cannot spam cancels."""
        if trigger not in TRIGGER_CODES or trigger == TRIGGER_NONE:
            raise ValueError(f"unknown disarm trigger: {trigger!r}")
        if not self._armed:
            return None
        if (
            trigger != TRIGGER_TIMEOUT
            and self._expires_at_sec is not None
            and now_sec >= self._expires_at_sec
        ):
            # The window had already run out when this trigger arrived. Report
            # what actually closed the gate, not what happened to notice it -
            # otherwise an expiry that no poll got to first would be recorded
            # under someone else's trigger and TIMEOUT would look unreachable.
            late = f" ({detail})" if detail else ""
            return self._close(
                TRIGGER_TIMEOUT,
                now_sec,
                f"arming window expired at {self._expires_at_sec:.3f}; "
                f"{trigger} observed after expiry{late}",
            )
        return self._close(trigger, now_sec, detail)

    def _close(self, trigger: str, now_sec: float, detail: str) -> DisarmEvent:
        was_armed = self._armed
        self._armed = False
        self._expires_at_sec = None
        self._armed_by = ""
        self.last_disarm_trigger = trigger
        self.last_disarm_detail = detail
        self.last_disarm_cancelled = was_armed
        return DisarmEvent(
            trigger=trigger,
            detail=detail,
            at_sec=now_sec,
            requires_cancel=was_armed,
            diagnostic_error=trigger not in _EXPECTED_TRIGGERS,
        )

    # -- observations ---------------------------------------------------
    def poll(self, now_sec: float) -> Optional[DisarmEvent]:
        """Call periodically. Auto-disarms when the window expired."""
        if self._armed and self._expires_at_sec is not None and now_sec >= self._expires_at_sec:
            return self.disarm(
                TRIGGER_TIMEOUT, now_sec, f"arming window expired at {self._expires_at_sec:.3f}"
            )
        return None

    def note_link(self, alive: bool, now_sec: float) -> Optional[DisarmEvent]:
        if not alive:
            return self.disarm(TRIGGER_LINK_LOST, now_sec, "external link lost")
        return None

    def note_clock(self, advancing: bool, now_sec: float, detail: str = "") -> Optional[DisarmEvent]:
        """The clock every other timeout here is measured against has stopped.

        SAFETY.md F-24. ``poll`` cannot notice this and neither can any of the
        other observations, because all of them are delivered by timers that run
        on the clock in question: a frozen clock does not make this machine say
        anything wrong, it makes it say nothing at all. So the caller watches the
        clock against a reference that cannot freeze and reports the verdict
        here, and an armed window is closed on it.

        ``now_sec`` is the caller's own reference clock, exactly as in every
        other method here - since SAFETY.md F-29 that is a MONOTONIC instant,
        and it must not be the frozen ROS value however tempting that looks: the
        expiry comparison in :meth:`disarm` would then subtract two different
        epochs and report ``TIMEOUT`` for a clock event. Observed, in the twin
        suite, from exactly that mistake. What the ROS clock said belongs in
        ``detail``, where it informs without being arithmetic.
        """
        if advancing:
            return None
        return self.disarm(
            TRIGGER_CLOCK_STALLED,
            now_sec,
            detail or "the ROS clock stopped advancing",
        )

    def note_clock_jumped_back(
        self, now_sec: float, detail: str = ""
    ) -> Optional[DisarmEvent]:
        """The ROS clock went BACKWARDS. SAFETY.md F-30, SR-15 rule 7.

        Its own method rather than a flag on :meth:`note_clock`, for the same
        reason it has its own trigger code: the caller has already decided WHICH
        of the two clock faults it saw, and a boolean threaded through a shared
        entry point is a place where that decision can be lost. What comes out
        here is :data:`TRIGGER_CLOCK_JUMPED_BACK` and never
        :data:`TRIGGER_CLOCK_STALLED`.

        Everything else is identical to a stall by intent: the window closes,
        the cancel is required, and the diagnostic is an ERROR - a discontinuity
        invalidates every age in flight exactly as a frozen clock makes every
        age meaningless. Only the reported code differs, so that a consumer can
        branch on the fault without parsing prose.

        ``now_sec`` is MONOTONIC, as everywhere else in this class (F-29). The
        ROS-clock values - the old reference, the new value, the size of the
        jump - belong in ``detail``, where they inform without being arithmetic.
        """
        return self.disarm(
            TRIGGER_CLOCK_JUMPED_BACK,
            now_sec,
            detail or "the ROS clock jumped backwards",
        )

    def note_teleop_mode(self, mode: str, now_sec: float) -> Optional[DisarmEvent]:
        """Any mode other than ``autonomous`` disarms.

        This is the path the spacebar E-stop takes: it publishes
        ``set_mode=keyboard`` (SR-2), so the operator's stop also closes our
        gate without anybody having to wire the two together.
        """
        if mode != REQUIRED_TELEOP_MODE:
            return self.disarm(
                TRIGGER_MODE_CHANGE, now_sec, f"teleop mode changed to '{mode}'"
            )
        return None

    def note_nav2(self, available: bool, now_sec: float) -> Optional[DisarmEvent]:
        if not available:
            return self.disarm(
                TRIGGER_NAV2_UNAVAILABLE, now_sec, "navigate_to_pose server disappeared"
            )
        return None

    def note_goal_aborted(self, now_sec: float) -> Optional[DisarmEvent]:
        """Count an aborted navigation; disarm once the budget is spent.

        Repeated aborts mean the stack is fighting something it does not
        understand, and continuing to drive into it is the wrong response.
        """
        self.consecutive_aborts += 1
        if self.consecutive_aborts >= self.max_consecutive_aborts:
            return self.disarm(
                TRIGGER_EXCESSIVE_ABORTS,
                now_sec,
                f"{self.consecutive_aborts} consecutive aborts "
                f">= max {self.max_consecutive_aborts}",
            )
        return None

    def note_goal_succeeded(self) -> None:
        self.consecutive_aborts = 0

    def shutdown(self, now_sec: float) -> Optional[DisarmEvent]:
        """Disarm on node shutdown (SR-12: no unrequested movement from a
        shutdown, and nothing left running that we no longer supervise)."""
        return self.disarm(TRIGGER_NODE_SHUTDOWN, now_sec, "node shutting down")

    # -- reporting ------------------------------------------------------
    def snapshot(self, now_sec: float) -> dict:
        """Plain-dict view for telemetry and for filling ``ArmingState``."""
        return {
            "armed": self.is_armed(now_sec),
            "seconds_remaining": self.seconds_remaining(now_sec),
            "expires_at_sec": self._expires_at_sec,
            "armed_by": self._armed_by,
            "last_disarm_trigger": self.last_disarm_trigger,
            "last_disarm_trigger_code": TRIGGER_CODES.get(self.last_disarm_trigger, 0),
            "last_disarm_detail": self.last_disarm_detail,
            "last_disarm_cancelled": self.last_disarm_cancelled,
            "auto_pick_available": self.auto_pick_available,
            "consecutive_aborts": self.consecutive_aborts,
        }
