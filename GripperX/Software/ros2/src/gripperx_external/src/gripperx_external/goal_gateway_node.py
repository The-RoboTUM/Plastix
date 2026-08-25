#!/usr/bin/env python3
"""Validation, geodesy, grasp resolution, the arming gate, preview, telemetry.

ROLLOUT STAGE 3 - DISPATCH EXISTS AND IS GATED, NOT ABSENT
==========================================================
Stages 0-2 contained no action client of any kind. This build adds the two that
FR-12 and SR-16 describe - ``NavigateToPose`` and ``PickPlastic`` - plus the
``trash_goal_done`` acknowledgement, and every one of them is behind the arming
gate. What changed is only *what a dispatch needs*; nothing about the gate has
been widened, and the default is still disarmed under every configuration.

THE TWO INDEPENDENT BLOCKS ON DISPATCH
======================================
1. ``armed`` is false and there is exactly one code path that can change that:
   :meth:`_on_set_arming`, the ``SetArming`` service handler, calling
   ``ArmingMachine.arm``. No parameter, launch argument or environment variable
   reaches it (SR-15 rule 4).
2. ``dry_run`` is true.

The third block of stage 2 - "no action client exists" - is structurally gone,
which is exactly what a safety audit had to clear first. Disabling either
remaining one alone still prevents dispatch, and :meth:`_dispatch_blocks`
reports both so an operator can see that the second is still there when the
first opens.

EVERY DISPATCH GOES THROUGH validate_dispatch, ON THE COORDINATES DISPATCHED
===========================================================================
C-5 / SAFETY.md F-5 and F-9. The goal that is previewed and the goal that is
dispatched are now the same object: :meth:`_resolve_goal` resolves the
``/octopus/trash_goal`` fix - the stream a dispatch actually uses - correlates it
to a ``trash_gps`` id by position, and its result feeds the preview, the status
topic and the dispatch alike. ``validation.validate_dispatch`` is the single
entry to sending a goal AND the re-validation that runs on every tick while one
is in flight, so a geofence change, a datum move or a dead mux cancels the goal
that is running instead of only affecting the next one.

THE ACKNOWLEDGEMENT IS IRREVERSIBLE - C-7
=========================================
``trash_goal_done`` means *collected* to the Octopus and their protocol has no
way to take it back and no way to say "I could not do this". It is therefore
published only after a **successful PickPlastic result**, only while **armed**,
and only on a **unique** position correlation. Never on arrival, never on a
timer, never on an ambiguous match.

The correlation is not a decision taken once and carried: it is recomputed from
the mission's own fix against the target list AS IT IS at each of three points -
every dispatch tick while the goal is in flight (which CANCELS), at arrival
before the pick is sent (which refuses to actuate the arm), and immediately
before the publication (which refuses to publish). An ambiguity that appears
after dispatch used to change none of the three; that was SAFETY.md F-13, and
"the correlation was unique when we set off" is not a statement about now.

A target that fails
``max_attempts_per_target`` times is blacklisted locally, deliberately NOT
acknowledged, and surfaced loudly - which stalls their mission by design, that
being the honest outcome until they add the failure channel (proposal item 2).

**There is no parameter that can make an arrival acknowledge.** FR-12 item 7
once described a weaker branch - with auto-pick off, acknowledge on reach and
log that the semantics are weaker - and it was briefly implemented behind an
opt-in parameter defaulting to false. **User decision 2026-08-19: C-7 is
normative and FR-12 item 7 is aligned to it, so that parameter was REMOVED
outright rather than left at false.** The reasoning is the one this package
already applies to ``allow_arm`` and to the rejected
``escalate_to_keyboard_mode``: a decided rule belongs in the structure, not in a
default somebody can flip. A switch that can put a false "collected" on a
channel whose only meaning is "collected" is a switch that will eventually be
flipped by someone who has not read this.

Arrival is still fully observable - it simply is not an acknowledgement. It gets
its own ``ExternalGoalStatus.STATE_REACHED`` publication, its own log line with
the reached-check detail, and its own counters on ``/diagnostics``, where
``reached`` and ``acknowledged`` are reported side by side. The gap between them
is not a defect to be closed; it is the honest measure of a protocol that cannot
express failure.

WHAT THIS NODE MUST NEVER DO
============================
It subscribes ``/teleop/active_mode`` and treats ``autonomous`` as a
**precondition it observes**, never as a state it creates. It has no publisher
on ``/teleop/set_mode`` or on any topic of the motion command chain, under any
condition including failure conditions - see ``FORBIDDEN_PUBLISH_TOPICS`` in
``octopus_link_node``, which is enforced at construction time here too.

The keyboard-mode cancel-failure escalation that an earlier draft proposed was
**rejected outright** (SR-15 rule 9, user decision 2026-08-18). It is not
disabled by default - the parameter ``escalate_to_keyboard_mode`` must not exist
at all. The acceptance rule for that used to be a literal grep for the name, and
SAFETY.md 6.3 (revision 3) ruled that **the rule is about the mechanism, not the
string**: the grep fired on this very paragraph, which exists to keep the
mechanism out, and it would have passed for the same escalation written under
any other name - a ``String("keyboard")`` on a mode topic, or a service client
for the mux's mode switch, contains none of those characters. What enforces it
now is structural and mechanism-shaped: ``/teleop/active_mode`` is in
``FORBIDDEN_PUBLISH_TOPICS`` beside ``/teleop/set_mode`` so the mux's own output
cannot be written either, and ``assert_no_command_clients`` sweeps both
constructors for service clients (there must be none) and for action clients
(there must be exactly the two named at the call site). A cancel that is not
confirmed within
``cancel_confirm_timeout_sec`` logs ERROR and raises an ERROR diagnostic, and
that is the ENTIRE escalation: this node reports and lets the mechanisms that
own stopping do their job. A parameter that must never be enabled is a
parameter that will eventually be enabled by someone who has not read the
requirement.

KILLING THIS NODE DOES NOT STOP THE ROBOT
=========================================
Stated here because it is the single most dangerous thing to assume wrongly
about this process (SAFETY.md F-4). The gateway is a *command source*, not a
stop mechanism. SR-15 rule 11: it introduces no new stop path. Ending it -
``systemctl stop``, closing the launch terminal, ``Ctrl-C``, ``SIGTERM``, an OOM
kill, a crash - removes it as a source of new goals and nothing more. A
``NavigateToPose`` goal that Nav2 has already accepted keeps executing, and
``/cmd_vel`` keeps flowing while ``teleop_mux`` is in ``autonomous``.

What actually stops the vehicle, all of it upstream of this node and unchanged
by this package: the spacebar E-stop (``set_mode=keyboard``, which makes the mux
publish a zero ``Twist`` immediately), a Nav2 cancel, ``teleop_mux
cmd_timeout_sec: 0.5``, and the firmware ``CMD_TIMEOUT_MS`` of 1000 ms (NFR-4,
§3.1.7). This is why the shutdown path below cancels *before* the context goes
away instead of relying on ``destroy_node``: a cancel is the only stop-shaped
thing this node can contribute, and it can only contribute it while it is still
alive. On ``SIGKILL`` it contributes nothing at all - by then only the upstream
mechanisms exist, which is exactly the shape of the 2026-07-06 incident.

THREE CLOCKS, ON PURPOSE - AND ONE OF THEM WATCHES THE OTHERS
=============================================================
External ``header.stamp`` values are the Octopus's **wall clock** (their nodes
stamp with ``time.time()``), while in the twin this node runs on **sim time**,
which starts near zero. Comparing the two would make the staleness check
meaningless - a wall-clock stamp is always "in the future" relative to sim time,
so nothing would ever be stale. External stamps are therefore compared against
``time.time()``, and TF/mode ages against the ROS clock. Like with like. The age
of their **target list** belongs to the first group and not the second: it is
their 1 Hz publication on their clock, and since SAFETY.md F-28 it is measured
monotonically and refused past ``max_target_list_age_sec`` - a list that stopped
being refreshed answers every correlation question with the same confident
answer for ever, and their silence on one topic is invisible to the link
watchdog, which measures the last frame of ANY topic.

The third clock is the point of SAFETY.md F-24. **Every timer-driven safety
mechanism here - the arming expiry, the link watchdog, the cancel-confirm
report, the in-flight re-validation and the in-flight correlation gate - is
delivered by a timer on the ROS clock**, and ``use_sim_time`` decides what that
clock is. With it true and no ``/clock`` publisher, NOT ONE of those timers
fires - measured: zero callbacks in three seconds - while the node reports
itself up and healthy and says nothing. So ``use_sim_time`` is startup-only
here (a runtime change is refused before rclpy's own ``TimeSource`` callback can
see it), it is refused outright at startup on a domain that is not a known
simulation domain, and ``_clock_watchdog`` runs on a ``STEADY_TIME`` clock that
no parameter can switch and no publisher can stop: a stalled ROS clock refuses
arming and disarms, loudly, on a channel that is not itself one of the frozen
timers.
"""

from __future__ import annotations

import math
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclTime

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from gripperx_arm_msgs.action import PickPlastic
from nav2_msgs.action import NavigateToPose

import tf2_ros

from gripperx_external_msgs.msg import (
    ArmingState,
    ExternalGoal,
    ExternalGoalStatus,
    ExternalLinkStatus,
    ExternalTargetList,
    GeodeticDatum,
    RobotTelemetry,
)
from gripperx_external_msgs.srv import SetArming
from std_srvs.srv import Trigger

from . import clock_rate as rate_mod
from . import correlation as corr
from . import diagnostics as diag
from . import validation as val
from .arming import (
    REQUIRED_TELEOP_MODE,
    TRIGGER_NAV2_UNAVAILABLE,
    TRIGGER_OPERATOR,
    ArmingMachine,
)
from .domain_guard import (
    clock_publisher_warning,
    SIMULATION_DOMAIN_IDS,
    effective_domain_id,
    enforce_domain,
    is_simulation_domain,
)
from .geodesy import (
    Datum,
    DatumTracker,
    GeodesyError,
    datum_offset_m,
    latlon_to_map,
    map_to_latlon,
)
from .grasp import GraspOffset, check_reached, parse_measured_param
from .octopus_link_node import (
    assert_no_chain_publishers,
    assert_no_command_clients,
    guarded_publisher,
)

#: SR-15 rule 5, user decision 2026-08-18. NOT configurable upward: a config
#: file may lower the ceiling, never raise it. A request above the ceiling is
#: REJECTED, never clamped - a silently shortened arming window is a window the
#: operator believes they have and does not.
HARD_MAX_ARMING_DURATION_SEC = 600.0
#: The agreed operator default. It is never applied automatically: `SetArming`
#: requires an explicit duration and a request without one is refused. This
#: value exists so the refusal can name it.
AGREED_DEFAULT_ARMING_DURATION_SEC = 120.0

#: Parameters that may not change after startup. Everything about the authority
#: gate and the domain is fixed when the node comes up, so a `ros2 param set`
#: cannot widen it behind the operator's back.
#: Parameters this node reads ONCE, at construction, and never again - each
#: with the reason a running node refuses to pretend otherwise.
#:
#: SAFETY.md F-8: the defect being fixed here is not that these are fixed. It is
#: that a `ros2 param set` on them used to be ACCEPTED and then silently ignored,
#: with the node logging "set to X; re-validating on the next tick". An operator
#: NARROWING a safety window got a confirmation and no protection. Of the three
#: possible behaviours - take effect, refuse, or accept-and-ignore - the third is
#: the only one that cannot be reasoned about, so it is gone. Where a runtime
#: change was wanted it is implemented instead: the geofence, the grasp offsets,
#: `datum_jump_warn_m`, `goal_match_tolerance_m`, `max_attempts_per_target`,
#: `max_stamp_age_sec`, `max_tf_age_sec`, `max_teleop_mode_age_sec` and
#: `max_goal_cost` are all re-read at the point of use and are NOT in this list.
_AUTHORITY_REASON = (
    "the authority gate and the domain must not be widened by a running node "
    "(SR-8, SR-15 rule 4)"
)
_STARTUP_ONLY_PARAMS = {
    "expected_domain_id": _AUTHORITY_REASON,
    "occlusion_latch_enabled": (
        "it WIDENS the in-flight correlation gate that SAFETY.md F-13 exists to "
        "close. A gate width may never be raised on a running node (SAFETY.md "
        "A-4), and least of all this one, which stands between a NO_MATCH and "
        "an arm that actuates"
    ),
    # NOT declared by this node - rclpy declares it on every node - and that is
    # exactly why it was missing here (SAFETY.md F-24). rclpy's `TimeSource`
    # registers its OWN set-parameters callback, so a runtime change used to be
    # honoured; ours is registered later and rclpy inserts callbacks at the
    # front and stops at the first refusal, so refusing here means TimeSource
    # never sees it at all (verified against rclpy/node.py:907-916,1110).
    "use_sim_time": (
        "it selects the CLOCK that the arming window, the link watchdog, the "
        "cancel-confirm report, the in-flight re-validation and the in-flight "
        "correlation gate are ALL measured against. Switching it on a running "
        "node points every one of them at a clock that may never advance, and "
        "the mechanism that would report that is itself one of those timers "
        "(SAFETY.md F-24)"
    ),
    "goal_ingress_enabled": _AUTHORITY_REASON,
    "allow_arm": _AUTHORITY_REASON,
    "dry_run": _AUTHORITY_REASON,
    "auto_pick": _AUTHORITY_REASON,
    "arming.max_duration_sec": (
        "the arming ceiling is fixed when the arming machine is built; a running "
        "node must not be able to lengthen the window it is already inside "
        "(SR-15 rule 5)"
    ),
    "arming.max_consecutive_aborts": (
        "the abort budget is fixed when the arming machine is built; raising it "
        "at runtime would widen a trigger that is already counting"
    ),
    "link_lost_sec": (
        "the link watchdog's timeout is the mechanism that notices an ABSENCE, "
        "and it is read at startup by both the watchdog and the health test "
        "(SAFETY.md C-3). Restart with the new value"
    ),
    "clock_backward_eps_sec": (
        "it alone decides whether `CLOCK_JUMPED_BACK` - the NINTH auto-disarm "
        "trigger, split out from the stall by an explicit user decision on "
        "2026-08-19 - ever fires at all, so it is exactly the shape SR-15 rule "
        "14 makes startup-only: a threshold that can move while the gate is "
        "being relied on is not a threshold. Restart with the new value "
        "(audit finding F-36)"
    ),
    "clock_forward_jump_sec": (
        "it alone decides whether a FORWARD clock discontinuity is ever "
        "reported at all, and SR-15 rule 14 makes a safety-deciding threshold "
        "startup-only for the same reason `use_sim_time` is: a threshold that "
        "can move while it is being relied on is not a threshold. Restart with "
        "the new value (SAFETY.md F-40, user decision 2026-08-20)"
    ),
    "cancel_confirm_timeout_sec": (
        "it governs when an unconfirmed cancel is reported, which under SR-15 "
        "rule 9 is the entire escalation, and it is captured at startup by the "
        "safety tick and by the bounded shutdown wait"
    ),
    "navigate_action": (
        "the action client is constructed with this name; changing it at runtime "
        "would change what is REPORTED without changing what is TALKED TO"
    ),
    "pick_action": (
        "the action client is constructed with this name; changing it at runtime "
        "would change what is REPORTED without changing what is TALKED TO"
    ),
    "map_frame": "the TF lookups and every dispatched pose are built with it",
    "base_frame": "the TF lookups and every dispatched pose are built with it",
    "costmap_topic": "the subscription is created with it at startup",
    "odom_topic": "the subscription is created with it at startup",
    "teleop_mode_topic": (
        "the subscription is created with it at startup, and it is the mux "
        "observation the MODE_CHANGE trigger depends on"
    ),
    "telemetry_rate_hz": "the timer is created with it at startup",
    "preview_rate_hz": "the timer is created with it at startup",
    "safety_rate_hz": (
        "the safety timer is created with it at startup; the rate at which the "
        "gate is polled is not something a running node may change"
    ),
    "dispatch_rate_hz": (
        "the dispatch timer is created with it at startup; it is also the rate at "
        "which a goal in flight is re-validated and re-correlated"
    ),
    "clock_stall_sec": (
        "it is the tolerance of the one watchdog that is NOT driven by the ROS "
        "clock, i.e. the mechanism that notices that every other timeout has "
        "stopped running (SAFETY.md F-24). Restart with the new value"
    ),
    "clock_startup_grace_sec": (
        "it is how long an unproven ROS clock is reported at WARN instead of "
        "ERROR at startup, and the watchdog captures it when it is built. It "
        "never widens the gate - arming is refused for an unproven clock either "
        "way - but a running node must not be able to quieten the report "
        "(SAFETY.md F-31)"
    ),
    "max_target_list_age_sec": (
        "it decides when the target list stops counting as evidence about where "
        "the objects are, and the irreversible acknowledgement depends on that "
        "list (SAFETY.md F-28). Widening it at runtime would widen the window in "
        "which a frozen list can name an object we then report as collected"
    ),
}

_TO_VERIFY = "TO-VERIFY"


def _measured_descriptor() -> ParameterDescriptor:
    """Descriptor for a value that is either a measured number or ``TO-VERIFY``.

    ``dynamic_typing`` is what makes both halves of the requirement possible at
    once: the YAML may hold the literal string ``TO-VERIFY`` (no invented
    rectangle, no placeholder grasp offset), *and* an operator may set a number
    at runtime with a plain ``ros2 param set ... -8.0``. Without it the
    parameter is pinned to STRING by its default and the CLI rejects a numeric
    value with "expected Type.STRING got Type.DOUBLE" - which would technically
    still be settable as "'-8.0'", but FR-12 asks for a geofence the user can
    change at runtime, and a quoting trick is not that.

    Both forms are funnelled through ``grasp.parse_measured_param``, so the
    "unmeasured" case stays one concept with one definition.
    """
    return ParameterDescriptor(
        dynamic_typing=True,
        description=(
            "Either a number in metres, or the literal string 'TO-VERIFY' while "
            "the value is unmeasured. Unmeasured values make the gateway refuse "
            "rather than fall back to a default."
        ),
    )


def _reliable(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _colour(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


#: Reported in telemetry and on ``/diagnostics``. "unavailable" is deliberately
#: distinct from "idle": idle means a client that is simply not busy, and
#: reporting that while ``navigate_to_pose`` does not exist would be a
#: fabricated availability (FR-12 item 8).
NAV_UNAVAILABLE = "unavailable"
NAV_IDLE = "idle"
NAV_NAVIGATING = "navigating"
NAV_PICKING = "picking"
NAV_CANCELLING = "cancelling"

#: Why an acknowledgement that could otherwise have gone out did not. Every one
#: of these is a state the operator has to be able to see, because their
#: protocol advances only on the acknowledgement and silence looks identical to
#: success from their side (FR-12 item 7).
ACK_NO_PICK_CLIENT = "NO_PICK_CLIENT"
ACK_AUTO_PICK_OFF = "AUTO_PICK_OFF"
ACK_PICK_FAILED = "PICK_FAILED"
ACK_DISARMED = "DISARMED_BEFORE_RESULT"
ACK_NOT_CORRELATED = "NOT_CORRELATED"
#: The correlation was unique at dispatch and has stopped being unique, or has
#: stopped naming the id this mission was started for. Distinct from
#: ACK_NOT_CORRELATED, which means there never was an id: this one means the id
#: we hold stopped being provable while we were holding it. That is the F-13
#: case, and it is the one that used to acknowledge anyway (SAFETY.md rev 2).
ACK_CORRELATION_CHANGED = "CORRELATION_CHANGED"
#: The real robot's domain plus an unmeasured `grasp.tolerance_m`: the pick is
#: refused because "arrived" cannot be proven, and the arm would actuate on an
#: unsupported claim (user decision 2026-08-19 on SAFETY.md F-14). Structurally
#: unreachable on the twin - see `_pick_needs_measured_tolerance`.
ACK_TOLERANCE_UNMEASURED = "GRASP_TOLERANCE_UNMEASURED"


@dataclass
class GoalResolution:
    """One evaluation of the goal that WOULD be dispatched.

    The single object behind the preview, the status topic and the dispatch
    decision (C-5 / SAFETY.md F-5). It is produced once per dispatch tick from
    ``/octopus/trash_goal`` - the stream a dispatch actually uses - and the
    preview renders exactly this, so the arrow an operator inspects before
    arming and the pose that would be sent cannot come apart.
    """

    incoming: val.IncomingGoal
    ctx: val.ValidationContext
    result: val.ValidationResult
    correlation: corr.CorrelationResult
    datum_lat: float
    datum_lon: float
    at_sec: float

    @property
    def target_id(self) -> str:
        return self.correlation.target_id


@dataclass
class Mission:
    """The one goal in flight. There is never more than one.

    The counterpart pushes exactly one goal and advances only on the
    acknowledgement, so there is nothing to queue (FR-12 item 7). Everything the
    dispatch path needs to *finish* a goal lives here, including the datum the
    goal was resolved against: the datum can move under a running goal, and a
    pose derived from the old one no longer means what it did.
    """

    target_id: str
    object_xy: Tuple[float, float]
    pose: Tuple[float, float, float]
    datum_lat: float
    datum_lon: float
    started_at_sec: float
    state: str = NAV_NAVIGATING
    nav_handle: object = None
    pick_handle: object = None
    #: Set once, when a cancel is requested. Cancelling twice is a no-op, so a
    #: disarm during a cancel cannot produce a cancel storm.
    cancel_requested_at_sec: Optional[float] = None
    cancel_reason: str = ""
    cancel_confirmed: bool = False
    cancel_error_logged: bool = False
    #: Nav2 accepted the goal. Until then there is a request in flight but no
    #: handle to cancel, which is its own hazard - see `_cancel_mission`.
    nav_accepted: bool = False
    ack_suppressed_reason: str = ""
    #: INTERIM OCCLUSION LATCH (user decision 2026-08-25, see _correlation_holds).
    #: `unique_seen` records that this mission's fix DID correlate uniquely to
    #: its own target at least once. `latch_void_reason` is set the first time
    #: anything other than NO_MATCH goes wrong, and is never cleared: an
    #: ambiguity seen once is not undone by the ambiguity going away.
    unique_seen: bool = False
    latch_void_reason: str = ""
    #: The goal exactly as it was validated, kept so the in-flight
    #: re-validation runs on the same object rather than on a fresh
    #: interpretation of a message that may have changed underneath it.
    incoming: Optional[val.IncomingGoal] = None
    #: Set when the mission reaches a terminal state or a cancel is confirmed.
    #: `prepare_shutdown` waits on it - on the MAIN thread, with the executor
    #: still spinning, which is the only place in this package that waits at all.
    done: threading.Event = field(default_factory=threading.Event)

    @property
    def cancelling(self) -> bool:
        return self.cancel_requested_at_sec is not None


class GoalGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_gateway_node")

        # --- parameters, declared before anything is created --------------
        self.declare_parameter("expected_domain_id", -1)
        self.declare_parameter("goal_ingress_enabled", False)
        # SR-15 rule 4: `allow_arm` can only ever PERMIT a later explicit
        # service call. It can never BE the arming - there is no path from this
        # parameter to `armed == true` without _on_set_arming running.
        self.declare_parameter("allow_arm", False)
        self.declare_parameter("dry_run", True)
        # SR-16 condition 5, and it is genuinely two conditions: auto_pick must
        # be true AND the gate must be armed. Neither alone sends a PickPlastic
        # goal. Default false, startup-only, and `auto_pick_available` reports
        # false whenever the action server is not actually there - the twin has
        # no arm in the URDF or the sim (DT-8), so "on" and "possible" are
        # different questions and are reported separately.
        self.declare_parameter("auto_pick", False)

        self.declare_parameter("arming.max_duration_sec", HARD_MAX_ARMING_DURATION_SEC)
        self.declare_parameter("arming.max_consecutive_aborts", 3)

        # Geofence: RUNTIME-ADJUSTABLE by requirement (FR-12 item 6, user
        # decision 2026-08-18). Read fresh at every validation, so a
        # `ros2 param set` takes effect on the NEXT validation without a node
        # restart. A geofence change is NOT an arming event: it may trigger
        # re-validation and nothing else.
        self.declare_parameter("geofence.min_x_m", _TO_VERIFY, _measured_descriptor())
        self.declare_parameter("geofence.max_x_m", _TO_VERIFY, _measured_descriptor())
        self.declare_parameter("geofence.min_y_m", _TO_VERIFY, _measured_descriptor())
        self.declare_parameter("geofence.max_y_m", _TO_VERIFY, _measured_descriptor())

        # Bench measurement on the real robot; blocks stage 5. No placeholder -
        # a plausible number here produces goals that look valid, drive the
        # robot, and miss.
        self.declare_parameter("grasp.offset_x_m", _TO_VERIFY, _measured_descriptor())
        self.declare_parameter("grasp.offset_y_m", _TO_VERIFY, _measured_descriptor())
        self.declare_parameter("grasp.tolerance_m", _TO_VERIFY, _measured_descriptor())
        self.declare_parameter("grasp.approach_candidates", 12)
        self.declare_parameter("grasp.verify_path", False)

        self.declare_parameter("max_goal_cost", 200)
        self.declare_parameter("max_stamp_age_sec", 5.0)
        self.declare_parameter("max_tf_age_sec", 1.0)
        self.declare_parameter("max_teleop_mode_age_sec", 2.0)
        self.declare_parameter("link_lost_sec", 5.0)
        self.declare_parameter("datum_jump_warn_m", _TO_VERIFY, _measured_descriptor())
        # INTERIM, user decision 2026-08-25. See _correlation_holds for what it
        # does and what it costs. Startup-only, like every other gate width.
        self.declare_parameter("occlusion_latch_enabled", False)
        self.declare_parameter("goal_match_tolerance_m", 0.25)
        self.declare_parameter("max_attempts_per_target", 2)
        # SET 2026-08-24 BY USER DECISION: 0.15 m. NOT derived from anything and
        # NOT measured - the proposal on the table was to reuse
        # `goal_match_tolerance_m` (0.25) so that "same object" had one
        # definition; the user chose a tighter number, so it is its own
        # parameter and says so.
        #
        # WHAT IT DECIDES: a blacklisted id reappearing further than this from
        # where it was blacklisted is not the same object, so the entry is
        # dropped. It is a BRIDGE, not a fix - see `_drop_blacklist_on_identity`.
        self.declare_parameter("blacklist_identity_tolerance_m", 0.15)
        # SAFETY.md F-28. Their `trash_gps` publishes unconditionally at 1 Hz,
        # so this is the same kind of value as `link_lost_sec` and is set the
        # same way: a few missed publications is a dead stream, not a hiccup.
        # Measured against a MONOTONIC reference, not the ROS clock - the age of
        # THEIR data on THEIR cadence has nothing to do with our sim time, the
        # same argument that makes the external stamps use `time.time()`.
        self.declare_parameter("max_target_list_age_sec", 5.0)
        # SAFETY.md F-24. How long the ROS clock may fail to advance before the
        # gate is closed on it. Deliberately short: everything this node does to
        # keep an armed window honest is a timer on that clock, so the interval
        # in which it is frozen is an interval in which none of them exist.
        self.declare_parameter("clock_stall_sec", 2.0)
        # SAFETY.md F-31. "Never yet PROVEN" and "STOPPED after having been
        # proven" are two different states, and only the second one is a
        # failure. The first is a startup condition with a DDS discovery latency
        # behind it: the gateway subscribes `/clock` when it comes up and cannot
        # see the first message before discovery has matched the publisher. That
        # latency was measured on this laptop rather than guessed (see the
        # config files), and it overlaps `clock_stall_sec`, so an ordinary twin
        # start used to raise the same ERROR that a dead Gazebo raises - which
        # is how an ERROR becomes something operators scroll past.
        #
        # Inside this grace the unproven clock is reported at WARN; after it, at
        # ERROR. What does NOT change: arming is REFUSED throughout, in both
        # states, because an unproven clock is an unproven clock (SR-15 rule 5).
        # Only the loudness is graded, never the gate.
        self.declare_parameter("clock_startup_grace_sec", 10.0)
        # SAFETY.md F-40, DECIDED by the user 2026-08-20: a forward
        # discontinuity is REPORTED - WARN plus a /diagnostics value - and does
        # NOT disarm and does NOT cancel. On the real robot the only sources are
        # a time-sync step and a manual `date` set, and disarming on every step
        # of a flaky NAT'd time source would trade a small reporting gap for an
        # operational one - and would teach the operator to expect spurious
        # disarms, which is how a safety mechanism gets switched off.
        #
        # THE VALUE IS `TO-VERIFY`. Nothing measured it. 1.0 s is what this
        # implementation runs: comfortably above the excess that an honest
        # change in real-time factor can produce inside one watchdog period, and
        # below any time-sync step worth telling an operator about. It carries
        # the SAME verification gap as `clock_stall_sec` - every `/clock` behind
        # every number here has been a test fixture and never a running Gazebo,
        # so what a LOADED Gazebo does to it is unmeasured.
        #
        # SR-15 rule 14: its OWN parameter, not derived from `safety_rate_hz` or
        # from anything else, so that an edit to a rate cannot silently move a
        # clock tolerance (audit finding F-36).
        # SR-15 rule 14 / audit finding F-36. This was DERIVED as
        # `1.0 / max(2.0, safety_rate_hz)` until 2026-08-20, so an edit to the
        # SAFETY TICK RATE moved the BACKWARDS-JUMP DETECTION THRESHOLD - two
        # different quantities, and an edit that does not look like an edit to a
        # clock tolerance. `safety_rate_hz` is a rate; this is a tolerance; a
        # rate parameter must not silently move a clock tolerance.
        #
        # THE VALUE IS `TO-VERIFY` and is a PARAMETERISATION, not a new number:
        # 0.2 is exactly what the derivation produced at the `safety_rate_hz:
        # 5.0` both config files set, so the threshold this build runs is
        # identical to the one the previous build ran. Nothing measured it then
        # and nothing measures it now - what changed is that it is now visible
        # and cannot be moved by editing something else.
        #
        # What it means, unchanged from the derivation's own comment: how far
        # the ROS clock may go BACKWARDS before that is a discontinuity rather
        # than "no advance". `/clock` is delivered BEST_EFFORT, so a message
        # overtaken by its successor can rewind the clock by about the
        # publication spacing, and a rewind that small is indistinguishable from
        # that. Anything smaller falls through to the stall path, which needs
        # `clock_stall_sec` of no advance to fire at all - so jitter costs
        # nothing while a world reset is caught in one tick.
        self.declare_parameter("clock_backward_eps_sec", 0.2)
        self.declare_parameter("clock_forward_jump_sec", 1.0)

        # Nav2 / pick action interfaces. Names, not remappings, so the SR-9
        # baseline diff and this file agree about what is talked to.
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("pick_action", "/pick_plastic")
        # How long a cancel may go unconfirmed before it is reported. Reporting
        # is the ENTIRE escalation (SR-15 rule 9): no zero-write, no mode
        # publication, no parameter that could ever enable one.
        self.declare_parameter("cancel_confirm_timeout_sec", 3.0)

        self.declare_parameter("telemetry_rate_hz", 1.0)
        self.declare_parameter("preview_rate_hz", 5.0)
        # The safety tick (arming expiry + link watchdog) has its own rate on
        # purpose: lowering the preview or telemetry rate must not slow down the
        # rate at which the gate notices that it should be shut (SAFETY.md
        # F-2/F-3). It also runs in its own callback group, so a slow preview
        # cannot delay it.
        self.declare_parameter("safety_rate_hz", 5.0)
        # The dispatch tick: server availability, the in-flight re-validation
        # and the decision to send. Its own rate and its own timer so that the
        # preview rate can be lowered without slowing down the rate at which a
        # goal in flight is re-checked against a moved world.
        self.declare_parameter("dispatch_rate_hz", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("teleop_mode_topic", "/teleop/active_mode")

        # --- SR-8, BEFORE anything is created -----------------------------
        self._expected_domain = int(self.get_parameter("expected_domain_id").value)
        enforce_domain(self, self._expected_domain)

        # --- SAFETY.md F-24: WHICH CLOCK, and is that clock possible here? ---
        # `use_sim_time` is not one of this node's parameters - rclpy declares it
        # on every node - but it decides what every timeout in this file means.
        # On anything that is not a known simulation domain there is no `/clock`
        # publisher to be had, so `use_sim_time: true` there does not mean "sim
        # time", it means "a clock pinned at zero and every safety timer with
        # it". That is a configuration to refuse at startup, not to survive:
        # this node is a command SOURCE, so refusing to start removes goals and
        # nothing else (SR-15 rule 11), while starting would produce a node that
        # reports itself up and healthy with its whole tick machinery dead.
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        if self._use_sim_time and not is_simulation_domain():
            self.get_logger().fatal(
                f"use_sim_time is true on ROS_DOMAIN_ID={effective_domain_id()}, "
                f"which is not a known simulation domain "
                f"({sorted(SIMULATION_DOMAIN_IDS)}). There is no /clock outside "
                "the simulation, so sim time here means a ROS clock that never "
                "advances - and the arming timeout, the link watchdog, the "
                "cancel-confirm report and the in-flight re-validation are all "
                "measured on it (SAFETY.md F-24). Refusing to start. Launch with "
                "use_sim_time:=false."
            )
            raise SystemExit(2)

        # THE MIRROR OF THE BLOCK ABOVE. SAFETY.md F-35, user decision
        # 2026-08-20: the reverse case - a LIVE /clock publisher while
        # `use_sim_time` is false - was not detected anywhere, and this package
        # did not look for a /clock publisher at all. It WARNS rather than
        # refusing; `clock_publisher_warning` carries the reasoning for why the
        # two directions get different answers. Nothing here disarms, cancels or
        # refuses anything.
        #
        # Run TWICE, and the second time is not belt and braces: at this instant
        # DDS may not have matched a publisher that is already running, so a
        # silent start here is not evidence of absence. `_on_clock_seen` below is
        # the other half.
        self._clock_mismatch_warned = False
        self._clock_publisher_seen = 0
        self._warn_if_clock_publisher("at startup")

        # --- fixed-at-startup configuration --------------------------------
        self._ingress = bool(self.get_parameter("goal_ingress_enabled").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._allow_arm = bool(self.get_parameter("allow_arm").value)
        self._auto_pick = bool(self.get_parameter("auto_pick").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._link_lost_sec = float(self.get_parameter("link_lost_sec").value)
        # F-14, user decision 2026-08-19. On a real robot a pick additionally
        # requires a MEASURED `grasp.tolerance_m`, because without one "we
        # arrived" is unprovable and the arm would actuate on an unsupported
        # claim. The discriminator is the LIVE domain (SR-8), not a parameter:
        # there is deliberately no flag an operator or a config could set to make
        # the real robot behave like the twin.
        #
        # POLARITY, and it is the whole of SAFETY.md F-27. This used to read
        # `== REAL_ROBOT_DOMAIN_ID`, i.e. "is this domain 20?", so every domain
        # that was not exactly 20 took the PERMISSIVE branch - silently, with no
        # log and nothing in a config file a reviewer would notice. Moving the
        # robot to another domain for a two-robot demonstration would have
        # removed the arm's arrival check as a side effect of a network decision,
        # and this project renumbered a domain inside a week (44924b6). The test
        # now asks the safe question - "is this provably a simulation?" - so an
        # unknown domain is treated as a real robot. The twin and the offline
        # harness stay fully exercisable, which is what keeps the decoupling
        # decision of the same day intact.
        self._pick_needs_measured_tolerance = not is_simulation_domain()

        configured_max = float(self.get_parameter("arming.max_duration_sec").value)
        if configured_max > HARD_MAX_ARMING_DURATION_SEC:
            self.get_logger().error(
                f"arming.max_duration_sec={configured_max:.0f}s exceeds the hard "
                f"maximum of {HARD_MAX_ARMING_DURATION_SEC:.0f}s (SR-15 rule 5) - "
                "using the hard maximum. Configuration may lower this ceiling, "
                "never raise it."
            )
        max_duration = min(configured_max, HARD_MAX_ARMING_DURATION_SEC)

        # --- state ----------------------------------------------------------
        self._arming = ArmingMachine(
            allow_arm=self._allow_arm,
            max_duration_sec=max_duration,
            max_consecutive_aborts=int(
                self.get_parameter("arming.max_consecutive_aborts").value
            ),
            # Reported state, not permission: it becomes true only while
            # `auto_pick` is set AND the `/pick_plastic` server is actually
            # there. It starts false because at construction time nothing has
            # been discovered yet, and it is refreshed on every dispatch tick.
            auto_pick_available=False,
        )
        # The machine is touched from the safety callback group, from the
        # service handler and from the shutdown path on the main thread, and it
        # is READ from the telemetry/diagnostics timers in the work group. Under
        # a MultiThreadedExecutor those are genuinely concurrent, so every
        # access goes through this lock. Re-entrant because `_handle_disarm`
        # publishes an `ArmingState`, which snapshots the machine again.
        self._arming_lock = threading.RLock()
        self._shutdown_event = threading.Event()
        self._shutdown_reason = ""
        self._shutdown_prepared = False
        self._datum_tracker = DatumTracker(fallback=None, jump_warn_m=0.0)
        self._latest_targets: Optional[ExternalTargetList] = None
        self._latest_goal: Optional[ExternalGoal] = None
        self._preview_dirty = False
        self._costmap: Optional[OccupancyGrid] = None
        self._odom: Optional[Odometry] = None
        self._odom_stamp_sec: Optional[float] = None
        self._teleop_mode = ""
        # MONOTONIC since SAFETY.md F-38, and named for it: `/teleop/active_mode`
        # is a `std_msgs/String` with no stamp, so this is OUR reception time and
        # the consumer's clock is the only clock in the comparison.
        self._teleop_mode_mono_sec: Optional[float] = None
        self._link_connected = False
        #: Their map-frame transform, as reported by the link node. Three
        #: states kept apart on purpose - see ExternalLinkStatus. `_seen` false
        #: means we have no observation, and NO OBSERVATION IS NOT EVIDENCE:
        #: a counterpart that never publishes the topic must not be gated by
        #: its silence.
        self._frame_status_enabled = False
        self._frame_status_seen = False
        self._frame_ready = False
        self._frame_relocks = 0
        #: The re-lock count we have already reacted to. None until the first
        #: link status: a gateway that starts AFTER the link node inherits a
        #: non-zero count, and treating that as a fresh re-lock would cancel a
        #: goal over an event that happened before we existed.
        self._frame_relocks_seen: Optional[int] = None
        #: Where each blacklisted target was, in WGS84 so a datum move cannot
        #: displace it. Kept beside `_blacklist` rather than inside it because
        #: the list is published in telemetry as ids.
        self._blacklist_latlon: Dict[str, Tuple[float, float]] = {}
        self._link_age_sec = -1.0
        self._link_reconnects = 0
        # C-3 / SAFETY.md F-3: the arrival time of the last `link_status`, and
        # the reference the watchdog measures silence against before the first
        # one ever arrives. `None` means "never seen", which the watchdog treats
        # as loss and not as "fine so far" - absence is the signal (SR-13).
        self._link_status_stamp_sec: Optional[float] = None
        self._link_watchdog_tripped = False
        self._started_at_sec = self._ros_now()
        # WHEN this node started, on the clock nothing can stop or slow down.
        # The link watchdog measures its silence against this and the startup
        # grace of the clock watchdog is counted from it (SAFETY.md F-29/F-31).
        self._started_mono = time.monotonic()
        # --- the clock everything above is measured against (SAFETY.md F-24) --
        # A frozen clock is the one failure this node cannot notice with any of
        # its own mechanisms, because all of them are timers on that clock: with
        # `use_sim_time` true and no `/clock` publisher NOT ONE of them fires,
        # and the node reports itself up and healthy throughout. The reference
        # below is monotonic, cannot be switched by a parameter and cannot be
        # stopped by anything on the network.
        self._clock_stall_sec = max(0.5, float(self.get_parameter("clock_stall_sec").value))
        # SAFETY.md F-31: the startup grace can never be SHORTER than the stall
        # tolerance - a grace inside the tolerance would not exist - and it is
        # not a second tolerance: the gate is closed identically in both.
        self._clock_startup_grace_sec = max(
            self._clock_stall_sec,
            float(self.get_parameter("clock_startup_grace_sec").value),
        )
        # SAFETY.md F-30 / F-36. Its OWN parameter since 2026-08-20 - see the
        # declaration for why, and note that NOTHING but that parameter is read
        # here: reading `safety_rate_hz` anywhere in this statement is the bug
        # F-36 is about, and `check_validation.py` part 9 asserts its absence.
        #
        # Floored, and the floor is NOT the value: below this a `/clock` message
        # overtaken by its BEST_EFFORT successor is indistinguishable from a
        # rewind, so a smaller number would disarm on message reordering. Same
        # shape and same number as `clock_forward_jump_sec`'s floor.
        self._clock_backward_eps_sec = max(0.05, float(
            self.get_parameter("clock_backward_eps_sec").value
        ))
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._clock_ref_ros_sec = self._started_at_sec
        self._clock_ref_mono = self._started_mono
        # SAFETY.md F-37. The watchdog below is the one place that holds a ROS
        # instant and a monotonic instant taken together, so it is where the
        # relative RATE of the two clocks is observed. Nothing in the gate reads
        # it - see `clock_rate` - it exists so that a reported instant on the
        # ROS clock can be honest at a real-time factor that is not 1.
        self._clock_rate = rate_mod.ClockRateEstimator()
        # Floored, and the floor is not the value: below one watchdog period the
        # "excess" being measured is scheduling jitter rather than a
        # discontinuity, so a smaller number would report noise. Same shape as
        # `clock_stall_sec`'s floor.
        self._clock_forward_jump_sec = max(0.05, float(
            self.get_parameter("clock_forward_jump_sec").value
        ))
        # SAFETY.md F-40. Reporting state ONLY - nothing here is read by any
        # gate, and no code path below turns either of these into a refusal.
        self._clock_forward_jumps = 0
        self._last_forward_jump_sec = 0.0
        self._clock_forward_report_errors = 0
        #: False under EVERY configuration at startup, including wall time: that
        #: the clock advances is an observation, and until it has been made this
        #: node refuses to arm. The watchdog makes it within two of its own
        #: ticks, i.e. long before an operator could call `SetArming`.
        self._clock_proven = False
        self._clock_stalled = False
        self._clock_stall_logged_mono = 0.0
        #: The severity the current stall was last reported at, so that the
        #: escalation from the startup WARN to the ERROR is said out loud the
        #: moment the grace runs out instead of waiting for the 30 s repeat
        #: (SAFETY.md F-31).
        self._clock_stall_severity_logged = ""
        #: SAFETY.md F-32. The stall LATCH and the "the disarm has been done"
        #: latch are two different facts and used to be one flag. `_clock_stalled`
        #: is the reported state and is set as early as possible, because
        #: everything it gates is a refusal; this one records that the disarm
        #: actually completed, and is set only AFTER it did - so a raise inside
        #: the disarm costs a retry on the next tick rather than the disarm.
        self._clock_disarm_done = False
        # SAFETY.md F-28: WHEN the target list arrived, on the monotonic clock.
        # Their `trash_gps` is a 1 Hz wall-clock stream of theirs; its age has
        # nothing to do with our sim time, exactly as with their header stamps.
        self._targets_received_mono: Optional[float] = None
        self._max_target_age_sec = float(
            self.get_parameter("max_target_list_age_sec").value
        )
        self._blacklist: List[str] = []
        self._attempts: Dict[str, int] = {}
        self._counters = {"received": 0, "accepted": 0, "rejected": 0, "preview": 0}
        self._last_reason = ""
        self._last_severity = ""
        self._last_logged_verdict: Optional[Tuple[str, str]] = None
        self._last_disarm_error = False
        self._datum_burst_count = 0

        # --- dispatch state (stage 3) ---------------------------------------
        # The mission is touched from the work group (dispatch tick, action
        # callbacks) and from the safety group (a disarm cancels it, the safety
        # tick judges the cancel timeout). LOCK ORDER, and it is never the other
        # way round: `_arming_lock` first, `_mission_lock` second. Nothing that
        # holds the mission lock ever asks the arming machine anything.
        self._mission_lock = threading.RLock()
        self._mission: Optional[Mission] = None
        self._nav2_available = False
        self._pick_available = False
        self._nav_state = NAV_UNAVAILABLE
        self._nav_state_reason = "NOT_DISCOVERED_YET"
        self._acknowledged: List[str] = []
        #: Targets Nav2 reported arrival at. Kept separately from
        #: `_acknowledged` because since C-7 the two mean genuinely different
        #: things: reached is ours to observe, collected is a claim about the
        #: world that only a successful pick supports.
        self._reached: List[str] = []
        self._last_reached_detail = ""
        self._ack_suppressed_reason = ""
        self._last_correlation = ""
        #: The status of the LAST re-correlation of the goal in flight, which is
        #: a different question from `_last_correlation` (the goal that would be
        #: dispatched next). SAFETY.md F-13.
        self._mission_correlation = ""
        #: How often a goal in flight was withdrawn because its correlation
        #: stopped holding. Counted and reported rather than bounded: a
        #: correlation-lost cancel costs a re-drive and nothing else, and a
        #: target list that flaps in and out of ambiguity would produce
        #: dispatch/cancel churn - visible here, and the same shape as every
        #: other in-flight cancel reason, none of which is rate-limited either.
        self._correlation_cancels = 0
        self._cancel_failures = 0
        # Never cleared for the life of the process, on purpose - see
        # `_note_cancel_failure`.
        self._cancel_failed_sticky = False
        # FR-12 item 7: their ids restart at 1 when their node restarts and the
        # `collected` flags go with them, so a blacklist may only be trusted
        # inside one link session. The reconnect counter is what identifies the
        # session.
        self._link_session = -1
        #: Ids we acknowledged AND then SAW carrying `collected: true` in their
        #: own target list. Only these are evidence: an id we acknowledged but
        #: never saw flip may simply never have received our acknowledgement,
        #: and treating that as a restart would be the same false positive
        #: F-16 is about, arriving by a different road. SAFETY.md F-16.
        self._confirmed_collected: List[str] = []
        #: How often the id space was observed to reset. Reported, not inferred.
        self._id_space_resets = 0
        #: The one resolution the preview renders and the dispatch decides on.
        self._resolution: Optional[GoalResolution] = None
        self._rendered_resolution: Optional[GoalResolution] = None
        self._last_dispatch_block: Optional[Tuple[str, str]] = None

        # --- publishers (all guarded against the motion chain) --------------
        self._status_pub = guarded_publisher(
            self, ExternalGoalStatus, "goal_status", _reliable()
        )
        self._arming_pub = guarded_publisher(self, ArmingState, "arming_state", _latched())
        self._telemetry_pub = guarded_publisher(self, RobotTelemetry, "telemetry", _reliable(1))
        self._marker_pub = guarded_publisher(self, MarkerArray, "preview_markers", _latched())
        self._diag_pub = guarded_publisher(self, DiagnosticArray, "/diagnostics", _reliable(10))
        # THE ACKNOWLEDGEMENT, and the only thing on it. `octopus_link_node`
        # turns this into `/octopus/trash_goal_done` on the wire; this node
        # never speaks the wire format itself (FR-12 item 2). Created only with
        # ingress on: with no goals coming in there is nothing that could ever
        # be acknowledged, and an advertised acknowledgement channel that cannot
        # be used is an invitation.
        self._goal_done_pub = None
        if self._ingress:
            self._goal_done_pub = guarded_publisher(self, String, "goal_done", _reliable())

        # --- subscriptions ---------------------------------------------------
        # TWO CALLBACK GROUPS, RUN BY A MultiThreadedExecutor (SAFETY.md F-2).
        #
        # `_safety_group` carries everything that can CLOSE the gate: the arming
        # service, the teleop-mode observation, the link-status observation and
        # the safety tick (expiry poll + link watchdog). `_work_group` carries
        # everything else - datum, targets, goal, odometry, costmap, preview,
        # telemetry. Both are mutually exclusive internally, so the safety
        # observations still serialise against each other and no lock is needed
        # between them; the two groups run concurrently, so a slow costmap, a
        # large marker burst or (in stage 3) an action callback cannot starve
        # the poll that shuts the window. Blocking waits are forbidden in ANY
        # callback of either group - stage 3 cancels asynchronously and confirms
        # on the result callback.
        self._safety_group = MutuallyExclusiveCallbackGroup()
        self._work_group = MutuallyExclusiveCallbackGroup()
        group = self._work_group
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=False)

        self.create_subscription(
            GeodeticDatum, "datum", self._on_datum, _latched(), callback_group=group
        )
        self.create_subscription(
            String,
            str(self.get_parameter("teleop_mode_topic").value),
            self._on_teleop_mode,
            _reliable(1),
            callback_group=self._safety_group,
        )
        # SR-15 rule 6: SUBSCRIBED, never published. Publishing the mode would
        # let the external path arm the very chain it is gated by, and would add
        # a second writer to a mode-arbitration topic (SR-9 / OP-19).
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            _reliable(1),
            callback_group=group,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._on_costmap,
            _latched(),
            callback_group=group,
        )
        self.create_subscription(
            ExternalLinkStatus,
            "link_status",
            self._on_link_status,
            _latched(),
            callback_group=self._safety_group,
        )
        # SAFETY.md F-35, second firing point. ONLY on wall time: with
        # `use_sim_time` true, /clock is rclpy's own subscription and the
        # situation is not the finding. This one is a DETECTOR, not a clock
        # source - see `_on_clock_seen`. QoS matches rclpy's TimeSource
        # (BEST_EFFORT, depth 1) so it matches the same publishers rclpy would.
        if not self._use_sim_time:
            self.create_subscription(
                ClockMsg,
                "/clock",
                self._on_clock_seen,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self._safety_group,
            )
        if self._ingress:
            self.create_subscription(
                ExternalGoal, "goal", self._on_goal, _reliable(), callback_group=group
            )
            self.create_subscription(
                ExternalTargetList,
                "targets",
                self._on_targets,
                _latched(),
                callback_group=group,
            )

        # --- the one and only way into the armed state ----------------------
        self._arming_srv = self.create_service(
            SetArming, "set_arming", self._on_set_arming, callback_group=self._safety_group
        )

        # --- the operator's way out of a stale blacklist --------------------
        # NOT an authority gate: it moves nothing and permits nothing. A cleared
        # target still has to pass every validation step and still needs an
        # armed gate to be dispatched. It exists because the blacklist can
        # outlive the id space it was built against and, until then, the only
        # cure was restarting this node - which on the real robot costs the
        # arming state and a fresh SR-1 approval.
        self._clear_blacklist_srv = self.create_service(
            Trigger,
            "clear_blacklist",
            self._on_clear_blacklist,
            callback_group=self._safety_group,
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        preview_hz = max(0.5, float(self.get_parameter("preview_rate_hz").value))
        telem_hz = max(0.1, float(self.get_parameter("telemetry_rate_hz").value))
        safety_hz = max(1.0, float(self.get_parameter("safety_rate_hz").value))
        self.create_timer(1.0 / preview_hz, self._preview_tick, callback_group=self._work_group)
        self.create_timer(1.0 / telem_hz, self._telemetry_tick, callback_group=self._work_group)
        # ON THE STEADY CLOCK (SAFETY.md F-29), like the clock watchdog below
        # and unlike everything else in this node. This tick carries the arming
        # expiry and the link watchdog, and since F-29 both of those are
        # measured monotonically - a tick delivered by the ROS clock would have
        # left them correct but STARVED, i.e. an expiry that has happened but is
        # not acted on until sim time gets round to it. The gate that closes
        # itself must not be scheduled by the thing it is closing itself
        # against. `_check_cancel_timeout`, which this tick also carries, still
        # reasons in ROS time (its operands are ROS-time stamps); being called
        # from a steady timer only means it is called at a steady rate.
        self._safety_timer = self.create_timer(
            1.0 / safety_hz,
            self._safety_tick,
            callback_group=self._safety_group,
            clock=self._wall_clock,
        )
        # THE ONE TIMER THAT IS NOT ON THE ROS CLOCK (SAFETY.md F-24). Everything
        # else in this node - including `_safety_tick` immediately above, which
        # carries the arming expiry and the link watchdog - is delivered by a
        # timer driven by `self._clock`, and `use_sim_time` decides what that is.
        # A watchdog for a stopped clock that runs on the stopped clock is not a
        # watchdog, so this one is created with a STEADY_TIME clock: verified to
        # keep firing at full rate in the exact configuration where every
        # ROS-clock timer fires zero times. Same callback group as the safety
        # tick, so its disarm serialises with the others without a lock.
        self._clock_timer = self.create_timer(
            1.0 / max(2.0, safety_hz),
            self._clock_watchdog,
            callback_group=self._safety_group,
            clock=self._wall_clock,
        )

        # --- the two action clients (stage 3) -------------------------------
        # BOTH in the WORK group. Their callbacks - goal response, feedback,
        # result, cancel response - must never land in the group that carries
        # the arming poll, the mode observation and the link watchdog, because
        # an action callback is exactly the kind of work that can take time
        # (SAFETY.md F-2 / C-2). Nothing in this node ever blocks inside a
        # callback: goals are sent with `send_goal_async`, cancels with
        # `cancel_goal_async`, and every confirmation arrives on a callback.
        # The one bounded wait in the package is on the MAIN thread in
        # `prepare_shutdown`, where the executor is still spinning.
        self._nav_action_name = str(self.get_parameter("navigate_action").value)
        self._pick_action_name = str(self.get_parameter("pick_action").value)
        self._nav_client = ActionClient(
            self, NavigateToPose, self._nav_action_name, callback_group=self._work_group
        )
        self._pick_client = ActionClient(
            self, PickPlastic, self._pick_action_name, callback_group=self._work_group
        )
        dispatch_hz = max(0.5, float(self.get_parameter("dispatch_rate_hz").value))
        self._cancel_confirm_timeout_sec = float(
            self.get_parameter("cancel_confirm_timeout_sec").value
        )
        self.create_timer(
            1.0 / dispatch_hz, self._dispatch_tick, callback_group=self._work_group
        )

        # C-6 / SAFETY.md F-7: the pre-check in `guarded_publisher` sees the
        # topic string it was HANDED; rclpy applies remap rules inside
        # `create_publisher`, and nothing stops future code from calling
        # `create_publisher` directly. This sweep looks at what the node
        # actually ended up with, after remapping, and is therefore the check
        # that corresponds to what SR-15 rule 9 asserts. Last statement before
        # the node reports itself up.
        assert_no_chain_publishers(self)
        # SAFETY.md 6.3: and the other half of "this node commands nothing" -
        # what it can CALL. Zero service clients, and exactly the two action
        # clients stage 3 is allowed to have, named here so that a third one has
        # to be argued for at this line rather than discovered in a log.
        assert_no_command_clients(
            self,
            (
                (self._nav_action_name, NavigateToPose),
                (self._pick_action_name, PickPlastic),
            ),
        )

        self.get_logger().info(
            f"clock: use_sim_time={self._use_sim_time} "
            f"(ROS_DOMAIN_ID={effective_domain_id()}); the cancel-confirm report "
            "and every in-flight re-check run on it. The ARMING WINDOW and the "
            "LINK WATCHDOG do not: both are promises in wall-clock seconds and "
            "are measured on a monotonic clock, so a twin at a low real-time "
            "factor cannot stretch them (SAFETY.md F-29). A stall of more than "
            f"{self._clock_stall_sec:.1f}s is detected against that same "
            "monotonic reference, refuses arming and disarms; a backwards jump "
            f"of more than clock_backward_eps_sec={self._clock_backward_eps_sec:.2f}s "
            "is reported as such, disarms and re-baselines, and a forward jump of "
            f"more than clock_forward_jump_sec={self._clock_forward_jump_sec:.2f}s "
            "is reported and does NOT disarm (SAFETY.md F-24/F-30/F-40; both "
            "thresholds are TO-VERIFY and neither is derived from a rate - "
            "F-36). Before "
            "the clock has ever been observed to advance, the first "
            f"{self._clock_startup_grace_sec:.1f}s are reported at WARN as "
            "/clock discovery, and arming is refused throughout (SAFETY.md "
            f"F-31). The target list must be no older than "
            f"{self._max_target_age_sec:.1f}s to be correlated against "
            "(SAFETY.md F-28)."
        )
        self._publish_arming_state()
        self.get_logger().info(
            f"goal_gateway_node up: goal_ingress_enabled={self._ingress} "
            f"allow_arm={self._allow_arm} dry_run={self._dry_run} armed=False "
            f"(SR-15: default disarmed, no startup path to armed)"
        )
        if not self._ingress:
            self.get_logger().info(
                "rollout stage 1: no goal ingress, no action client, telemetry only"
            )
        else:
            self.get_logger().info(
                "rollout stage 3: dispatch exists and is gated - "
                + "; ".join(self._dispatch_blocks())
            )
            self.get_logger().info(
                f"action clients created (not connected yet): "
                f"{self._nav_action_name}, {self._pick_action_name}; "
                f"auto_pick={self._auto_pick} "
                f"(needs armed AND auto_pick AND a live pick server - SR-16). "
                "An acknowledgement follows a successful pick and nothing else - "
                "there is no parameter that can make arrival acknowledge (C-7). "
                "The correlation that names the target is re-taken at every one "
                "of the three gates - in flight, before the pick and before the "
                "acknowledgement - and an ambiguity at any of them is a refusal "
                "(SAFETY.md F-13)."
            )
            if self._pick_needs_measured_tolerance:
                offset = self._grasp_offset()
                self._log_at(
                    "error" if not offset.tolerance_configured else "info",
                    "REAL ROBOT domain: a pick additionally requires a measured "
                    "grasp.tolerance_m, and it is currently "
                    + ("UNMEASURED - every arrival will REFUSE to pick"
                       if not offset.tolerance_configured
                       else "measured")
                    + ". Without it the reached check returns no verdict, so "
                    "'arrived' would be an unsupported claim to actuate the arm "
                    "on (user decision 2026-08-19, SAFETY.md F-14)."
                )

    # ==================================================================
    # the authority gate
    # ==================================================================
    def _dispatch_blocks(self) -> List[str]:
        """Every independent reason a goal cannot become motion right now.

        Reported rather than merely relied upon: SR-15 asks for blocks that are
        independent, and independence is only credible if you can see all of
        them at once.
        """
        blocks = []
        # Read-time evaluation (SAFETY.md F-2): an expired window is closed the
        # instant it expires, not when the next poll happens to run.
        with self._arming_lock:
            armed = self._arming.is_armed(self._safety_now())
        if not armed:
            blocks.append("disarmed (SR-15 layer 1)")
        if self._dry_run:
            blocks.append("dry_run")
        # Not a gate in its own right - a stalled clock disarms, and the tick
        # that would dispatch has stopped running anyway - but an operator
        # staring at a node that refuses to arm has to be told why here, where
        # they are already looking (SAFETY.md F-24).
        if not self._clock_proven:
            blocks.append("ROS clock not yet observed to advance (SAFETY.md F-24)")
        elif self._clock_stalled:
            blocks.append("ROS clock STALLED (SAFETY.md F-24)")
        # Not a "block" in the SR-15 sense - it is a precondition, not a gate -
        # but an operator staring at a disarmed-but-permitted node needs to know
        # whether the thing it would talk to is even there.
        if not self._nav2_available:
            blocks.append(f"{self._nav_action_name} not available")
        # Their map frame is the ground every external goal is expressed in. If
        # they say it is not ready, dispatching would send the robot to a pose
        # derived from a frame its own owner does not vouch for.
        #
        # GATED ON AN OBSERVATION, NEVER ON SILENCE. Only `seen` makes
        # `not ready` a statement; without it we have no observation, and an
        # Octopus that does not publish this topic at all keeps working exactly
        # as before. Recovers by itself when their `state` returns - no operator
        # act, because nothing on our side broke.
        if self._frame_status_enabled and self._frame_status_seen and not self._frame_ready:
            blocks.append(
                "the Octopus map frame is not ready (their transform status: "
                "state not 'ready', no yaw lock, or the status went stale)"
            )
        return blocks

    def _on_set_arming(self, request: SetArming.Request, response: SetArming.Response):
        """The ONLY code path that can set the armed state.

        Local ROS service on our own domain. Arming is never a wire command: no
        incoming Octopus payload, no rosbridge message and no field in any
        external JSON reaches this handler (SR-15 rule 3).
        """
        # MONOTONIC (SAFETY.md F-29): the window an operator is granted here is
        # a promise in their seconds, not in the simulation's.
        now = self._safety_now()
        if not request.arm:
            with self._arming_lock:
                event = self._arming.disarm(TRIGGER_OPERATOR, now, "operator disarm")
            self._handle_disarm(event)
            response.success = True
            response.message = "disarmed"
            response.state = self._arming_state_msg()
            return response

        duration = float(request.duration_sec)
        if duration <= 0.0:
            response.success = False
            response.message = (
                "duration_sec is required and must be > 0; there is no indefinite "
                f"arming window. The agreed default is "
                f"{AGREED_DEFAULT_ARMING_DURATION_SEC:.0f} s - pass it explicitly."
            )
            response.state = self._arming_state_msg()
            self.get_logger().warn(f"SetArming refused: {response.message}")
            return response
        if duration > self._arming.max_duration_sec:
            # REJECTED, not clamped (SR-15 rule 5).
            response.success = False
            response.message = (
                f"duration_sec {duration:.0f} exceeds the maximum "
                f"{self._arming.max_duration_sec:.0f} s and is REJECTED, not "
                "shortened. The node stays disarmed."
            )
            response.state = self._arming_state_msg()
            self.get_logger().warn(f"SetArming refused: {response.message}")
            return response

        # SAFETY.md F-24. An arming window is a PROMISE that it will close by
        # itself, and every mechanism that keeps that promise - the expiry, the
        # link watchdog, the in-flight re-validation, the correlation gate - is a
        # timer on the clock below. Granting a window while that clock is not
        # observably running would grant a window with no way out except an
        # operator, which is the one property SR-15 rule 5 exists to prevent.
        # Refused, loudly, rather than granted with the timers dead.
        if not self._clock_proven or self._clock_stalled:
            frozen_for = max(0.0, time.monotonic() - self._clock_ref_mono)
            response.success = False
            response.message = (
                "REFUSED: the ROS clock is not advancing ("
                + (
                    f"it has never advanced; {frozen_for:.1f}s since this node "
                    "started"
                    if not self._clock_proven
                    else f"frozen for {frozen_for:.1f}s"
                )
                + f", use_sim_time={self._use_sim_time}"
                + (
                    f"; still inside the {self._clock_startup_grace_sec:.1f}s "
                    "/clock discovery grace, so this may clear by itself"
                    if self._clock_in_startup_grace()
                    else ""
                )
                + "). The in-flight re-validation and the correlation gate that "
                "would keep this window honest are measured on it and are inert "
                "while it does not move (SAFETY.md F-24). Start a /clock "
                "publisher, or restart with use_sim_time:=false."
            )
            response.state = self._arming_state_msg()
            self.get_logger().error(f"SetArming refused: {response.message}")
            return response

        with self._arming_lock:
            result = self._arming.arm(duration, request.requested_by or "", now)
        response.success = result.granted
        response.message = result.message
        if result.granted:
            self.get_logger().warn(
                f"ARMED for {duration:.0f}s by '{request.requested_by}'. "
                "Remaining blocks: " + "; ".join(self._dispatch_blocks())
            )
        else:
            self.get_logger().warn(f"SetArming refused: {result.message}")
        response.state = self._arming_state_msg()
        self._publish_arming_state()
        return response

    def _log_at(self, severity: str, message: str) -> None:
        """Log at a severity chosen at run time, WITHOUT a shared call site.

        rclpy's logger caches (file, line, function) and pins a severity to it:
        calling the same statement at two different severities raises
        ``ValueError: Logger severity cannot be changed between calls.`` from
        inside ``log()``. The idiomatic-looking
        ``level = get_logger().error if x else get_logger().info; level(msg)``
        is therefore a LATENT CRASH, and it fires only on the second call - the
        one with the other severity.

        Found the hard way while exercising the seven auto-disarm triggers: an
        ``OPERATOR`` disarm (INFO) followed by a ``MODE_CHANGE`` disarm (ERROR)
        killed the executor, and `prepare_shutdown` then died on the same
        pattern in `_cancel_mission`, so the in-flight goal was never cancelled.
        A crash here is the 2026-07-06 shape exactly: the supervising process
        disappears and the command source behind it does not notice.

        Every branch below is its own physical call site. That is the whole fix,
        and it is why this is a method rather than a local variable.
        """
        if severity == "error":
            self.get_logger().error(message)
        elif severity == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _mission_object_latlon(self, mission: "Mission") -> Optional[Tuple[float, float]]:
        """The blacklisted object's position in WGS84.

        Taken back through the datum the mission was RESOLVED against, not the
        current one: that pair is what the map metres on the mission mean, and
        using today's datum for yesterday's metres would bake a datum move into
        the anchor.
        """
        try:
            return map_to_latlon(
                Datum(mission.datum_lat, mission.datum_lon), *mission.object_xy
            )
        except GeodesyError:
            return None

    def _note_frame_relock(self, relocks: int) -> None:
        """A re-lock of their map frame CANCELS what is running. It does not disarm.

        CANCEL, because the alternative is worse and because the precedent is
        already here: a DATUM MOVE cancels the goal in flight, and this is the
        same event in a different axis. Their frame turned; a rotation of the
        reference frame must not be treated more leniently than a translation of
        its origin.

        NOT DISARM, deliberately. The arming window is a promise about TIME, made
        to the operator, and it is measured monotonically for exactly that
        reason. A re-lock is a geometric event that re-arming would not repair,
        so disarming on it would blur what the arming gate means without making
        anything safer. The operator keeps the window they were granted; what
        they lose is the goal that was computed against a frame that has moved.

        WHAT THIS DOES NOT CLAIM. We do not apply their rotation at all - we take
        their map coordinates as ours. So a re-lock does not make us MORE wrong;
        it makes visible that an assumption we were already making has stopped
        holding. Cancelling is the honest response to that. Applying the
        alignment properly is the owed work, and this signal is what will keep it
        maintainable once it exists.
        """
        previous = self._frame_relocks_seen
        self._frame_relocks_seen = relocks
        if previous is None or relocks <= previous:
            return
        self.get_logger().error(
            f"OCTOPUS FRAME RE-LOCK observed ({previous} -> {relocks}): their "
            "transform node re-locked its startup yaw, so any their-map -> "
            "our-map alignment established before this moment is NO LONGER TRUE. "
            "Cancelling whatever is in flight, on the same reasoning that makes "
            "a datum move cancel. The arming window is NOT closed - a re-lock is "
            "geometry, not time, and re-arming would not repair it."
        )
        self._cancel_mission("OCTOPUS_FRAME_RELOCK", self._safety_now(), error=True)

    def _drop_blacklist_on_identity(self, msg: ExternalTargetList) -> None:
        """Drop blacklist entries whose id now names a different object.

        THE BRIDGE, not the fix. `_maybe_drop_blacklist` needs a `collected` flag
        it saw set and then saw cleared; when their targets are DELETED rather
        than collected that evidence can never exist, and their ids restart at 1
        regardless. This asks a different question with the data we do have: is
        the thing carrying this id still where the thing we blacklisted was?

        Positions are compared in WGS84-derived metres against the CURRENT datum,
        and the anchor is stored as lat/lon, so a datum move displaces both sides
        equally and cannot fake a mismatch.

        It can be wrong in both directions and neither is silent: an object that
        was physically moved gets a second chance it arguably deserves, and an id
        reset that happens to place a new object within the tolerance stays
        blacklisted, where `clear_blacklist` is the operator's door.

        THE REAL FIX IS A `session_id` FROM THEM (proposal item E). With one, this
        method becomes unnecessary and the question stops being a guess.
        """
        if not self._blacklist_latlon:
            return
        datum = self._datum_tracker.datum
        if datum is None:
            return
        tolerance = float(self.get_parameter("blacklist_identity_tolerance_m").value)
        if tolerance <= 0.0:
            return
        for target in msg.targets:
            target_id = str(target.id)
            anchor = self._blacklist_latlon.get(target_id)
            if anchor is None or target_id not in self._blacklist:
                continue
            try:
                was_x, was_y = latlon_to_map(datum, anchor[0], anchor[1])
                now_x, now_y = latlon_to_map(
                    datum, float(target.latitude_deg), float(target.longitude_deg)
                )
            except GeodesyError:
                continue
            moved = math.hypot(now_x - was_x, now_y - was_y)
            if moved <= tolerance:
                continue
            self._blacklist.remove(target_id)
            self._blacklist_latlon.pop(target_id, None)
            self._attempts.pop(target_id, None)
            self.get_logger().warn(
                f"target {target_id} is {moved:.3f} m from where that id was "
                f"blacklisted, beyond blacklist_identity_tolerance_m={tolerance:.2f} "
                "(user decision, not a measurement). The id no longer names the "
                "object it was blacklisted for - most likely their id space "
                "restarted - so the entry and its attempt count are dropped. A "
                "`session_id` from them would settle this instead of inferring it."
            )

    def _on_clear_blacklist(self, request: Trigger.Request, response: Trigger.Response):
        """Drop the blacklist and the attempt counts, as an explicit operator act.

        WHY THIS EXISTS. `_maybe_drop_blacklist` drops it automatically, but only
        on EVIDENCE: an id we acknowledged, watched turn `collected: true`, and
        then saw come back `collected: false`. That evidence CANNOT EXIST when
        their targets are deleted rather than collected - no flag was ever set,
        so none can come back - and their ids restart at 1 regardless. Observed
        live on 2026-08-21: a fresh target arrived as id 1, we still held id 1
        from the previous id space, and every reachable target behind it was
        stuck with no error on either side.

        The strict evidence rule is NOT being relaxed. It is strict on purpose: a
        looser test would fire once a second on an acknowledgement that never
        reached them, which is unbounded retry by another route. So the automatic
        path keeps its rule and the operator gets a door.

        It is deliberately NOT gated on the armed state. The blacklist is not an
        authority gate - clearing it permits nothing, dispatches nothing and
        moves nothing; a cleared target faces every validation step and the
        arming gate exactly as it did before. Refusing to clear while armed would
        force a disarm/re-arm cycle to fix a bookkeeping problem, which is the
        kind of ceremony that teaches operators to disarm reflexively.
        """
        del request
        cleared = list(self._blacklist)
        attempts = dict(self._attempts)
        self._blacklist.clear()
        self._blacklist_latlon.clear()
        self._attempts.clear()
        if not cleared and not attempts:
            response.success = True
            response.message = "blacklist and attempt counts were already empty"
            self.get_logger().info(f"clear_blacklist: {response.message}")
            return response
        response.success = True
        response.message = (
            f"cleared blacklist [{', '.join(cleared) or 'none'}] and "
            f"{len(attempts)} attempt count(s)"
        )
        # WARN, not INFO: it un-does a refusal that something earned, and the
        # next dispatch of a target that failed twice should be traceable to
        # this line.
        self.get_logger().warn(
            f"OPERATOR CLEARED THE BLACKLIST: {response.message}. Targets that "
            "previously exhausted their attempts can be dispatched again. "
            "Nothing was armed, dispatched or acknowledged by this call."
        )
        return response

    def _handle_disarm(self, event) -> None:
        """Act on a disarm. CANCEL, never write. C-6.

        Called from the safety group (mode, link, expiry), from the service
        handler and from the shutdown path. Everything it does is non-blocking:
        `_cancel_mission` sends a cancel request and confirms on a callback that
        runs in the WORK group, so a slow or dead server cannot stall the group
        that closes the gate (C-2).

        What must never appear here is a publisher of zeros on a chain topic:
        that would make this node a second writer (SR-9) and a worse stop than
        the ones that already exist (teleop_mux cmd_timeout_sec 0.5, the
        watchdog, firmware CMD_TIMEOUT_MS 1000 ms - NFR-4, §3.1.7).
        """
        if event is None:
            return
        self._last_disarm_error = bool(event.diagnostic_error)
        self._log_at(
            "error" if event.diagnostic_error else "info",
            # The numeric code is in the line because it is what goes on the
            # wire in `ArmingState.last_disarm_trigger`, and since SR-15 rule
            # 7 splits CLOCK_STALLED (8) from CLOCK_JUMPED_BACK (9) the code
            # is the thing a reader - and the twin suite - has to be able to
            # check. Same source as the message field: `TRIGGER_CODES`.
            f"disarmed by {event.trigger} (ArmingState constant "
            f"{event.trigger_code}): {event.detail}",
        )
        if event.requires_cancel:
            # `requires_cancel` is true whenever the machine was armed, so this
            # runs for every trigger including OPERATOR and TIMEOUT. When there
            # is nothing in flight it is a no-op and returns False.
            # ROS time, and deliberately NOT `event.at_sec`. Since SAFETY.md
            # F-29 the arming machine reasons monotonically, so `at_sec` is a
            # monotonic instant, while `_cancel_mission` stores what it is given
            # as `cancel_requested_at_sec` and `_check_cancel_timeout` subtracts
            # that from the ROS clock. Two epochs in one subtraction is the very
            # mistake F-29 is about, one level down.
            self._cancel_mission(
                f"disarm:{event.trigger}",
                self._ros_now(),
                error=event.diagnostic_error,
            )
        self._publish_arming_state()

    def _arming_state_msg(self) -> ArmingState:
        with self._arming_lock:
            snap = self._arming.snapshot(self._safety_now())
        msg = ArmingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.armed = bool(snap["armed"])
        msg.seconds_remaining = float(snap["seconds_remaining"])
        # PROJECTED, since SAFETY.md F-29, and projected AT THE OBSERVED RATE
        # since SAFETY.md F-37. The window itself is measured monotonically, and
        # a monotonic instant on the wire would be meaningless: this field shares
        # a message with `header.stamp`, which is a ROS-clock stamp. So it
        # reports the instant ON THIS NODE'S ROS CLOCK at which the window
        # closes.
        #
        # The rate is what the arithmetic used to be missing. `seconds_remaining`
        # is monotonic WALL seconds; adding them to a ROS instant assumes the two
        # clocks run at the same speed, and at a real-time factor of 0.1 that
        # advertised a gate closing ~595 wall-seconds away for one that closed in
        # 60.2 s. `seconds_remaining` is still the field that needs no
        # interpretation - it is the operator's seconds - and it is unchanged.
        if snap["expires_at_sec"] is not None:
            expires = rate_mod.project_ros_expiry(
                self._ros_now(),
                float(snap["seconds_remaining"]),
                self._clock_rate.rate,
            )
            msg.expires_at.sec = int(math.floor(expires))
            msg.expires_at.nanosec = int(round((expires - math.floor(expires)) * 1e9))
        msg.armed_by = str(snap["armed_by"])
        msg.last_disarm_trigger = int(snap["last_disarm_trigger_code"])
        msg.last_disarm_detail = str(snap["last_disarm_detail"])
        msg.last_disarm_cancelled = bool(snap["last_disarm_cancelled"])
        msg.auto_pick_available = bool(snap["auto_pick_available"])
        return msg

    def _publish_arming_state(self) -> None:
        self._arming_pub.publish(self._arming_state_msg())

    # ==================================================================
    # parameters
    # ==================================================================
    def _on_set_parameters(self, params) -> SetParametersResult:
        """Runtime parameter changes.

        A geofence change is explicitly NOT an arming event (FR-12 item 6): it
        may trigger re-validation and nothing else. Nothing in this callback
        arms, disarms or dispatches - it only marks the preview dirty so the
        next tick re-validates against the new area.
        """
        for param in params:
            if param.name in _STARTUP_ONLY_PARAMS:
                # REFUSED, with the reason for THIS parameter. Never accepted and
                # ignored: an operator who is told a safety value was applied has
                # to be able to believe it (SAFETY.md F-8).
                reason = (
                    f"{param.name} is fixed at startup: "
                    f"{_STARTUP_ONLY_PARAMS[param.name]}. Restart with the new "
                    "configuration."
                )
                self.get_logger().warn(f"REFUSED parameter change - {reason}")
                return SetParametersResult(successful=False, reason=reason)
            if param.name.startswith(("geofence.", "grasp.offset", "grasp.tolerance")) or (
                param.name == "datum_jump_warn_m"
            ):
                value = param.value
                if isinstance(value, str) and parse_measured_param(value) is None:
                    if value.strip() != _TO_VERIFY:
                        return SetParametersResult(
                            successful=False,
                            reason=(
                                f"{param.name}={value!r} is neither a number nor "
                                f"'{_TO_VERIFY}'"
                            ),
                        )
        for param in params:
            self.get_logger().info(
                f"parameter {param.name} set to {param.value!r}; re-validating on the "
                "next tick (this is not an arming event and dispatches nothing)"
            )
        self._preview_dirty = True
        return SetParametersResult(successful=True)

    def _measured(self, name: str) -> Optional[float]:
        return parse_measured_param(self.get_parameter(name).value)

    def _geofence_rect(self) -> Optional[Tuple[float, float, float, float]]:
        """Read the geofence fresh, every time. ``None`` while unmeasured.

        Reading it here rather than caching it at startup is what makes
        `ros2 param set` effective at the next validation without a restart.
        """
        values = [
            self._measured("geofence.min_x_m"),
            self._measured("geofence.max_x_m"),
            self._measured("geofence.min_y_m"),
            self._measured("geofence.max_y_m"),
        ]
        if any(v is None for v in values):
            return None
        min_x, max_x, min_y, max_y = values  # type: ignore[misc]
        if not (max_x > min_x and max_y > min_y):
            return None
        return min_x, max_x, min_y, max_y

    def _grasp_offset(self) -> GraspOffset:
        return GraspOffset.from_params(
            self.get_parameter("grasp.offset_x_m").value,
            self.get_parameter("grasp.offset_y_m").value,
            self.get_parameter("grasp.tolerance_m").value,
        )

    def _unset_items(self) -> List[str]:
        unset = []
        if self._geofence_rect() is None:
            unset.append("geofence.{min,max}_{x,y}_m")
        offset = self._grasp_offset()
        if not offset.configured:
            unset.append("grasp.offset_x/y_m")
        if not offset.tolerance_configured:
            # Reported separately because it blocks something different: goals
            # still resolve and dispatch without it, but the post-arrival
            # reached check cannot return a verdict (user decision 2026-08-19).
            unset.append("grasp.tolerance_m (reached check only)")
        if self._measured("datum_jump_warn_m") is None:
            unset.append("datum_jump_warn_m")
        return unset

    # ==================================================================
    # subscriptions
    # ==================================================================
    def _warn_if_clock_publisher(self, when: str) -> bool:
        """SAFETY.md F-35. WARN once if /clock is live while we are on wall time.

        Latched: the condition does not change while the node runs, and a
        mismatch repeated on every tick is a mismatch nobody reads. Returns
        whether it warned, so the offline checks can assert the latch.
        """
        try:
            count = self.count_publishers("/clock")
        except Exception:  # noqa: BLE001 - a probe must never cost a startup
            return False
        self._clock_publisher_seen = max(self._clock_publisher_seen, int(count))
        message = clock_publisher_warning(count, self._use_sim_time, when)
        if message is None or self._clock_mismatch_warned:
            return False
        self._clock_mismatch_warned = True
        self.get_logger().warn(message)
        return True

    def _on_clock_seen(self, _msg: ClockMsg) -> None:
        """The second firing point: a /clock MESSAGE actually arrived.

        Subscribed only when `use_sim_time` is false - when it is true this is
        rclpy's own business and the situation is not the finding. Receiving
        /clock here does NOT make this node sim-time driven: the node's clock is
        selected by the `use_sim_time` parameter through rclpy's TimeSource, and
        this subscription is not attached to it. It exists to answer one
        question - is something really publishing sim time at us - and its
        callback does nothing but report.
        """
        self._warn_if_clock_publisher("on the first /clock message")

    def _ros_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _safety_now(self) -> float:
        """The clock the AUTHORITY GATE is measured on. SAFETY.md F-29.

        Monotonic, and deliberately not the ROS clock. Two of this node's
        timeouts are promises made in WALL-CLOCK terms to somebody outside the
        simulation, and both used to be measured in sim seconds:

        * the **arming window**. An operator who grants 120 s is promising
          themselves two minutes; SR-15 rule 5 exists so that the window closes
          BY ITSELF. On a twin at a real-time factor of 0.1 that promise became
          twenty minutes - measured: a 20 s window had not expired after 123 s
          of wall time - and the watchdog called the clock healthy throughout,
          correctly, because it was advancing. A safety bound whose length
          depends on how loaded Gazebo is, is not a bound.
        * the **link watchdog** (C-3). `link_lost_sec` is a statement about a
          WiFi link, and a WiFi link does not slow down when Gazebo does.

        So both are read here instead, and the choice is the stronger of the two
        the auditor offered: not "detect a slow clock and warn", but measure the
        things that live in wall time on a wall clock, and leave sim time to the
        things that belong in the simulation - the TF ages, the external stamps'
        counterparts, the in-flight re-validation, everything whose other operand
        is a sim-time stamp. Comparing like with like, one level up.

        This does NOT retire the clock watchdog and it must not: those sim-time
        mechanisms are exactly as inert on a stopped clock as they ever were, so
        a stalled clock still disarms and still refuses to arm (SAFETY.md F-24).
        What changes is that the gate's own expiry no longer depends on it.

        Only the :class:`ArmingMachine` and the link watchdog take their time
        from here. Anything comparing against a ROS-clock stamp must not: a
        monotonic value and a sim-time stamp are two different epochs, and
        mixing them is how a staleness gate silently turns off.
        """
        return time.monotonic()

    def _on_datum(self, msg: GeodeticDatum) -> None:
        datum = Datum(
            latitude_deg=msg.latitude_deg,
            longitude_deg=msg.longitude_deg,
            from_topic=True,
            stamp_sec=self._ros_now(),
        )
        # datum_jump_warn_m is TO-VERIFY. Until it is measured, ANY change is
        # treated as significant: that is the conservative direction (it only
        # re-resolves and re-previews, it never dispatches), and it avoids
        # inventing a threshold. Their own trigger is 1e-9 deg, so a marker drag
        # arrives as a burst - which is why this only sets a dirty flag and the
        # timer does the work.
        warn_m = self._measured("datum_jump_warn_m")
        self._datum_tracker.jump_warn_m = 0.0 if warn_m is None else warn_m
        update = self._datum_tracker.update(datum)
        if not update.accepted:
            self.get_logger().warn(
                f"external datum refused: {update.reason}", throttle_duration_sec=10.0
            )
            return
        if update.changed:
            self._datum_burst_count += 1
            self.get_logger().warn(
                f"datum moved {update.jump_m:.3f} m - every target moves with it; "
                "re-resolving and re-previewing the whole set",
                # Their datum_callback fires above ~0.1 mm and republishes every
                # target, so a single marker drag produces a flood. Throttled so
                # the burst is one line, not one line per target.
                throttle_duration_sec=2.0,
            )
        self._preview_dirty = True

    def _on_targets(self, msg: ExternalTargetList) -> None:
        # Latest-wins: a burst collapses to its final state instead of growing a
        # queue. The preview converges to one consistent set (FR-12 item 5).
        self._latest_targets = msg
        # WHEN, and not only WHAT (SAFETY.md F-28). Everything else the pipeline
        # consumes is age-checked - the TF pose, their header stamps, the teleop
        # mode, the link - and this list was the exception, while being the input
        # the irreversible acknowledgement depends on since the F-13 fix.
        self._targets_received_mono = time.monotonic()
        self._preview_dirty = True
        self._note_id_space(msg)
        # After the evidence rule, not instead of it: `_note_id_space` may have
        # cleared the whole blacklist on real evidence, and there is nothing left
        # to infer about when it has.
        self._drop_blacklist_on_identity(msg)

    def _target_list_age_sec(self) -> Optional[float]:
        """How long ago the last target list arrived. ``None`` = never.

        Monotonic on purpose. This is the age of THEIR 1 Hz publication on THEIR
        wall clock, so measuring it in sim time would be the same category error
        the external stamps already avoid - and it must keep meaning something
        when the ROS clock is the thing that has stopped (SAFETY.md F-24/F-28).
        """
        if self._targets_received_mono is None:
            return None
        return max(0.0, time.monotonic() - self._targets_received_mono)

    def _note_id_space(self, msg: ExternalTargetList) -> None:
        """Drop the blacklist only on EVIDENCE that their id space restarted.

        SAFETY.md F-16 replaced a transport counter with this. The evidence is
        the strongest signal their protocol actually carries: an id we
        acknowledged, and then WATCHED turn ``collected: true`` in their own
        target list, coming back ``collected: false``. Their flags are lost only
        when their node restarts, and their ids restart at 1 with them - so a
        flag we saw set and then saw cleared means the id space we blacklisted
        against no longer exists.

        Requiring that we SAW it set is what keeps the false positive out. An
        acknowledgement that never reached them leaves the target uncollected
        continuously, with no transition, and a bare "an id we acknowledged is
        uncollected" test would fire on that once a second - the same unbounded
        retry by another route.

        When there is no evidence, nothing happens. A blacklist kept too long
        stalls their mission on one target, which is visible, bounded and
        recoverable by restarting this node; a blacklist dropped wrongly sends
        the arm back at an object that has already failed twice.
        """
        collected_now = set()
        reset_evidence = []
        for target in msg.targets:
            target_id = str(target.id)
            if target.collected:
                collected_now.add(target_id)
                if target_id in self._acknowledged and target_id not in self._confirmed_collected:
                    self._confirmed_collected.append(target_id)
            elif target_id in self._confirmed_collected:
                reset_evidence.append(target_id)
        if not reset_evidence:
            return
        self._id_space_resets += 1
        self._confirmed_collected = [
            t for t in self._confirmed_collected if t in collected_now
        ]
        dropped = ", ".join(self._blacklist) or "none"
        self._blacklist.clear()
        self._blacklist_latlon.clear()
        self._attempts.clear()
        self.get_logger().error(
            "their id space RESTARTED: target(s) "
            f"[{', '.join(reset_evidence)}] were acknowledged and observed "
            "collected, and are now reported uncollected again. Their collected "
            "flags are lost only on a restart of their node and their ids restart "
            f"at 1 with them, so dropping the blacklist [{dropped}] and the "
            "attempt counts - an id from before this point is not evidence about "
            "the object carrying it now (FR-12 item 7, proposal item 3)."
        )

    def _on_goal(self, msg: ExternalGoal) -> None:
        self._latest_goal = msg
        self._counters["received"] += 1
        self._preview_dirty = True

    def _on_teleop_mode(self, msg: String) -> None:
        """EVERY message reaches the trigger. Change detection is for the log only.

        C-1 / SAFETY.md F-1. The earlier guard required a *previous* mode and a
        *change*, so the first message a freshly started gateway ever saw could
        not disarm it. That is the normal case, not a corner one: `teleop_mux`
        starts in `keyboard` and republishes `/teleop/active_mode` every tick,
        so after any restart - including every restart done after an E-stop -
        the first observed mode is `keyboard`. The gate must close on the state
        it observes, never on the transition it happened to witness.

        Calling unconditionally is cheap and cannot spam: `note_teleop_mode`
        returns `None` for `autonomous`, and `disarm` is idempotent while
        disarmed, so a mux ticking at 20 Hz produces at most one event.
        """
        # MONOTONIC (SAFETY.md F-38). `max_teleop_mode_age_sec` is the ONLY
        # thing that catches a `teleop_mux` that stopped publishing, and a mux
        # does not slow down when Gazebo does - measured before this fix: a dead
        # mux detected after 22.1 s of wall time against a configured 2.0 s at a
        # real-time factor of 0.1. The mode message carries no stamp of its own,
        # so both sides of that comparison are ours and moving them together is
        # the whole change (SR-15 rule 12).
        previous = self._teleop_mode
        self._teleop_mode = msg.data
        self._teleop_mode_mono_sec = self._safety_now()
        if previous != msg.data:
            self.get_logger().info(
                f"teleop mode observed: '{previous or '<none>'}' -> '{msg.data}'"
            )
        with self._arming_lock:
            event = self._arming.note_teleop_mode(msg.data, self._safety_now())
        self._handle_disarm(event)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_stamp_sec = self._ros_now()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_link_status(self, msg: ExternalLinkStatus) -> None:
        self._link_connected = bool(msg.connected)
        self._link_age_sec = float(msg.last_message_age_sec)
        self._link_reconnects = int(msg.reconnect_count)
        self._frame_status_enabled = bool(msg.frame_status_enabled)
        self._frame_status_seen = bool(msg.frame_status_seen)
        self._frame_ready = bool(msg.frame_ready)
        self._frame_relocks = int(msg.frame_relocks)
        self._note_frame_relock(self._frame_relocks)
        self._note_link_session(self._link_reconnects)
        # Link loss is a DISARM, not a hold: the external system can no longer
        # tell us that a target became invalid, so continuing is the unsafe
        # option (SR-15 rule 10, NFR-4). Nothing is dispatched in this build, so
        # here the effect is limited to closing the gate and reporting it.
        # MONOTONIC on both counts (SAFETY.md F-29): the arrival time the
        # watchdog measures silence against, and the time the arming machine
        # reasons in. `last_message_age_sec` inside the message is the link
        # node's own monotonic measurement of THEIR stream, so this is the same
        # kind of quantity throughout.
        mono = self._safety_now()
        # C-3: remember WHEN we last heard from the link node, so that its
        # silence can be judged. `_on_link_status` is the only place this is
        # refreshed; `_safety_tick` is the only place it is judged.
        self._link_status_stamp_sec = mono
        self._link_watchdog_tripped = False
        healthy = self._link_connected and 0.0 <= self._link_age_sec <= self._link_lost_sec
        with self._arming_lock:
            event = self._arming.note_link(healthy, mono)
        self._handle_disarm(event)

    def _note_link_session(self, reconnects: int) -> None:
        """A reconnect is a TRANSPORT event and clears NOTHING. SAFETY.md F-16.

        This method used to drop the blacklist and the attempt counts whenever
        ``ExternalLinkStatus.reconnect_count`` moved, on the reasoning that
        their ids restart at 1 when their node restarts. The reasoning is sound
        about a restart of their node and false about the field it was reading:
        ``reconnect_count`` is ``RosbridgeClient.stats.reconnects``, the number
        of successful WebSocket connections after the first. A WiFi flap
        produces it with their node untouched, their ids unchanged and their
        ``collected`` flags intact - and it then made a target we had proved
        unpickable retryable again, which is precisely the unbounded retry the
        blacklist exists to stop. Reproduced by the auditor with one forced
        reconnect.

        So: the blacklist SURVIVES a reconnect. It is dropped only on evidence
        that the id space itself changed, which is `_note_id_space`'s job, and
        when there is no such evidence the blacklist is kept and that is said
        out loud rather than quietly assumed. Asking their side for a session or
        boot id on the wire is proposal item 3 and is what would make this
        decidable instead of inferable.
        """
        if reconnects == self._link_session:
            return
        # WRITER UNIQUENESS ON OUR OWN STATUS TOPIC. `link_status` is supposed
        # to have exactly one publisher; with two, their reconnect counters
        # interleave and every message looks like a new session, which would
        # clear the blacklist twice a second and retry an impossible target for
        # ever. Found by exactly that symptom while verifying this code against
        # a machine that had orphaned link nodes on it. This is SR-9's principle
        # applied one topic further out: a second writer produces no error from
        # the middleware, so it has to produce one from us.
        publishers = self.count_publishers(self.resolve_topic_name("link_status"))
        if publishers > 1:
            self.get_logger().error(
                f"{publishers} publishers on link_status - there must be exactly "
                "one. Their reconnect counters interleave, so nothing about the "
                "link's state can be read from this topic while it lasts. Check "
                "for an orphaned octopus_link_node (SR-5, SR-9). The blacklist is "
                "unaffected either way since SAFETY.md F-16.",
                throttle_duration_sec=10.0,
            )
            return
        previous, self._link_session = self._link_session, reconnects
        if previous < 0:
            return
        if not self._blacklist and not self._attempts:
            self.get_logger().warn(
                f"external link reconnected ({previous} -> {reconnects} WebSocket "
                "connections). Nothing is held that a reconnect could affect."
            )
            return
        kept = ", ".join(self._blacklist) or "none"
        self.get_logger().warn(
            f"external link reconnected ({previous} -> {reconnects} WebSocket "
            f"connections). KEEPING the blacklist [{kept}] and the attempt "
            "counts: a reconnect is a transport event and says nothing about "
            "whether their node restarted, so it is not evidence that the ids "
            "mean different objects now. They are dropped only when their own "
            "target list contradicts a collected flag we watched it set "
            "(SAFETY.md F-16; a session id on the wire is proposal item 3)."
        )

    # ==================================================================
    # world queries handed to the pure pipeline
    # ==================================================================
    def _robot_pose(self) -> Tuple[Optional[Tuple[float, float, float]], Optional[float], str]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, RclTime()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as exc:
            return None, None, f"{type(exc).__name__}: {exc}"
        t = tf.transform.translation
        q = tf.transform.rotation
        stamp = RclTime.from_msg(tf.header.stamp).nanoseconds * 1e-9
        age = max(0.0, self._ros_now() - stamp)
        return (t.x, t.y, _yaw_from_quaternion(q.x, q.y, q.z, q.w)), age, ""

    def _costmap_cost(self, x: float, y: float) -> Optional[int]:
        grid = self._costmap
        if grid is None:
            return None
        info = grid.info
        col = int(math.floor((x - info.origin.position.x) / info.resolution))
        row = int(math.floor((y - info.origin.position.y) / info.resolution))
        if col < 0 or row < 0 or col >= info.width or row >= info.height:
            return None
        return int(grid.data[row * info.width + col])

    def _make_context(
        self, goal: val.IncomingGoal, current_goal_id: Optional[str] = None
    ) -> val.ValidationContext:
        pose, age, _ = self._robot_pose()
        rect = self._geofence_rect()
        geofence = None
        if rect is not None:
            min_x, max_x, min_y, max_y = rect
            geofence = lambda x, y: min_x <= x <= max_x and min_y <= y <= max_y  # noqa: E731
        return val.ValidationContext(
            goal=goal,
            # External stamps are the Octopus's wall clock; comparing them
            # against sim time would make the staleness check meaningless.
            now_sec=time.time(),
            datum_tracker=self._datum_tracker,
            grasp_offset=self._grasp_offset(),
            robot_pose=pose,
            robot_pose_age_sec=age,
            max_tf_age_sec=float(self.get_parameter("max_tf_age_sec").value),
            max_stamp_age_sec=float(self.get_parameter("max_stamp_age_sec").value),
            require_stamp=True,
            # The id in flight, so the counterpart's 1 Hz republication of the
            # same goal is a DUPLICATE and not a second dispatch. None for the
            # preview pass, where every target is worth showing.
            current_goal_id=current_goal_id,
            blacklisted_ids=tuple(self._blacklist),
            geofence=geofence,
            costmap_cost=self._costmap_cost,
            max_goal_cost=int(self.get_parameter("max_goal_cost").value),
            # verify_path needs a ComputePathToPose round trip, and the
            # acceptance predicate it would feed is a SYNCHRONOUS callable
            # called from inside the pipeline - i.e. a blocking wait inside a
            # callback, which C-2 forbids outright. Still forced off, now for a
            # different and better reason than "no client exists"; making it
            # work needs the ring walk restructured around a future, which is
            # its own piece of work. Never silently ignored: `_preview_tick`
            # logs an ERROR while the parameter is on.
            path_check=None,
            approach_candidates=int(self.get_parameter("grasp.approach_candidates").value),
        )

    # ==================================================================
    # the dispatch path (stage 3)
    # ==================================================================
    def _dispatch_tick(self) -> None:
        """Server availability, one resolution, then the decision. Work group.

        Order matters. Availability first, because losing ``navigate_to_pose``
        is an auto-disarm trigger and everything below it depends on the gate.
        Then exactly ONE resolution of the goal that would be dispatched, stored
        for the preview to render, so that what is shown and what would be sent
        are the same object (C-5). Then either supervise the goal in flight or
        decide about a new one - never both, because there is never more than
        one goal (FR-12 item 7).
        """
        now = self._ros_now()
        self._update_servers(now)

        resolution = self._resolve_goal(now)
        self._resolution = resolution

        with self._mission_lock:
            mission = self._mission
        if mission is not None:
            self._supervise_mission(mission, now)
            return
        self._maybe_dispatch(resolution, now)

    # -- server availability ------------------------------------------
    def _update_servers(self, now: float) -> None:
        """Discovery state of both action servers, and the NAV2_UNAVAILABLE trigger.

        ``server_is_ready`` is a non-blocking look at what the middleware has
        discovered; ``wait_for_server`` would be a blocking wait and is
        forbidden here (C-2). There is deliberately NO grace period: absence is
        the signal, not the hold (SR-13), and the same reasoning that made the
        link watchdog treat silence as loss applies to the server that would
        execute our goals. The practical consequence is stated plainly in the
        log: arming while ``navigate_to_pose`` is not there closes the gate
        again on the next dispatch tick.
        """
        nav_ready = self._nav_client.server_is_ready()
        pick_ready = self._pick_client.server_is_ready()

        if nav_ready != self._nav2_available:
            if nav_ready:
                self.get_logger().info(f"{self._nav_action_name} is available")
            else:
                self.get_logger().error(
                    f"{self._nav_action_name} DISAPPEARED. Any goal in flight is "
                    "no longer ours to supervise, and the gate closes with "
                    "NAV2_UNAVAILABLE. Note what this does NOT do: it does not "
                    "stop the vehicle. Only the E-stop, the mux timeout and the "
                    "firmware timeout do that (SR-15 rule 11)."
                )
        if pick_ready != self._pick_available:
            self._log_at(
                "info" if pick_ready else "warn",
                f"{self._pick_action_name} "
                + ("is available" if pick_ready else "is not available")
                + f"; auto_pick={self._auto_pick} -> auto_pick_available="
                + ("true" if (pick_ready and self._auto_pick) else "false"),
            )
        self._nav2_available = nav_ready
        self._pick_available = pick_ready

        with self._arming_lock:
            # Reported state, never permission. SR-16 needs BOTH the parameter
            # and a live server before a pick can be sent, and an operator
            # reading `auto_pick_available` must see the conjunction, not the
            # parameter.
            self._arming.auto_pick_available = bool(self._auto_pick and pick_ready)
            event = self._arming.note_nav2(nav_ready, self._safety_now())
        self._handle_disarm(event)

    # -- resolving the goal that would be dispatched --------------------
    def _resolve_goal(self, now: float) -> Optional[GoalResolution]:
        """Validate and correlate the ``/octopus/trash_goal`` fix. C-5 / F-5.

        Stage 2 validated the ``trash_gps`` target list and stored the actual
        goal fix in a slot nothing read, so the coordinates on screen and the
        coordinates a dispatch would use came from two independent 1 Hz streams
        with no consistency guarantee. This resolves THE FIX, and the id is
        recovered from the list by position afterwards.
        """
        goal_msg = self._latest_goal
        if goal_msg is None:
            return None

        lat = float(goal_msg.object_latitude_deg)
        lon = float(goal_msg.object_longitude_deg)
        well_formed = math.isfinite(lat) and math.isfinite(lon)
        stamp_sec = RclTime.from_msg(goal_msg.header.stamp).nanoseconds * 1e-9

        datum = self._datum_tracker.datum
        correlation = self._correlate(lat, lon, datum, well_formed)

        incoming = val.IncomingGoal(
            target_id=correlation.target_id,
            latitude_deg=lat,
            longitude_deg=lon,
            status=0,
            # Their producers always set header.stamp (verified 2026-08-18), and
            # it is their WALL clock - compared against time.time() in the
            # context, never against sim time.
            stamp_sec=stamp_sec if stamp_sec > 0.0 else None,
            confidence=None,
            well_formed=well_formed,
            malformed_detail=(
                "" if well_formed else "trash_goal did not parse; forwarded with NaN"
            ),
        )
        with self._mission_lock:
            in_flight = self._mission.target_id if self._mission else None
        ctx = self._make_context(incoming, current_goal_id=in_flight)
        result = self._validate(incoming, ctx)

        # The correlation verdict is applied AFTER the pipeline, not before, and
        # only when the pipeline was otherwise happy. That ordering is
        # deliberate: an operator looking at a refused goal still needs to see
        # WHERE it is and which standing pose it resolved to, and a first-failure
        # correlation check would throw all of that away. It can only ever turn
        # an acceptance into a refusal, never the other way round.
        if result.accepted and not correlation.unique:
            result = self._correlation_rejection(correlation, result)

        self._last_correlation = correlation.status
        return GoalResolution(
            incoming=incoming,
            ctx=ctx,
            result=result,
            correlation=correlation,
            datum_lat=datum.latitude_deg if datum else float("nan"),
            datum_lon=datum.longitude_deg if datum else float("nan"),
            at_sec=now,
        )

    def _correlate(
        self,
        lat: float,
        lon: float,
        datum: Optional[Datum],
        well_formed: bool,
        cross_check_reported_id: bool = True,
    ) -> corr.CorrelationResult:
        """Position-match a goal fix against the target list AS IT IS NOW.

        Everything is converted with the datum in force RIGHT NOW, both sides of
        the comparison, so a datum move cannot make the goal and the list drift
        apart relative to each other.

        ``cross_check_reported_id`` is on for the goal that WOULD be dispatched
        and off for the goal already in flight. The difference is not laziness:
        ``trash_gps.goal_id`` names the object THEIR side currently wants, and
        their selection is "nearest to the datum", so a new piece of litter
        appearing closer to the datum moves it legitimately. For a new dispatch a
        disagreement is a refusal (their pick and ours must be the same object);
        for a goal already running it would mean their re-prioritisation cancels
        our mission, which is thrash and not safety. What must still hold for a
        running mission is the thing F-13 is about: our OWN fix must still name
        our OWN id, uniquely.
        """
        if not well_formed or datum is None:
            return corr.CorrelationResult(
                corr.NO_MATCH,
                detail=(
                    "goal fix is malformed" if not well_formed else "no datum in force"
                ),
            )
        try:
            goal_xy = latlon_to_map(datum, lat, lon)
        except GeodesyError as exc:
            return corr.CorrelationResult(corr.NO_MATCH, detail=exc.detail)

        targets = self._latest_targets
        if targets is None:
            return corr.CorrelationResult(corr.NO_TARGETS)
        positions = []
        for target in targets.targets:
            try:
                tx, ty = latlon_to_map(datum, target.latitude_deg, target.longitude_deg)
            except GeodesyError:
                continue
            positions.append(
                corr.TargetPosition(
                    id=target.id, x=tx, y=ty, collected=bool(target.collected)
                )
            )
        return corr.correlate(
            goal_xy,
            positions,
            float(self.get_parameter("goal_match_tolerance_m").value),
            reported_goal_id=targets.goal_id if cross_check_reported_id else "",
            # SAFETY.md F-28. Applied at EVERY correlation, not only at the three
            # in-flight gates: driving to an object named by a list that stopped
            # being refreshed is the same mistake as acknowledging one, taken
            # earlier.
            list_age_sec=self._target_list_age_sec(),
            max_list_age_sec=self._max_target_age_sec,
        )

    def _correlation_rejection(
        self, correlation: corr.CorrelationResult, previous: val.ValidationResult
    ) -> val.ValidationResult:
        """Turn a correlation failure into a rejection that keeps the preview.

        A goal we cannot NAME is a goal we must not dispatch, not only one we
        must not acknowledge: arriving at it would leave us standing over an
        object with no id to report and no way to say which one it was.
        """
        reason = {
            corr.NO_MATCH: val.GOAL_NOT_CORRELATED,
            corr.AMBIGUOUS: val.GOAL_AMBIGUOUS,
            corr.ID_MISMATCH: val.GOAL_ID_MISMATCH,
            corr.NO_TARGETS: val.NO_TARGET_LIST,
            corr.TARGETS_STALE: val.TARGET_LIST_STALE,
        }.get(correlation.status, val.GOAL_NOT_CORRELATED)
        return val.ValidationResult(
            verdict=val.VERDICT_REJECTED,
            reason=reason,
            detail=correlation.detail,
            severity=val.severity_of(reason),
            warnings=list(previous.warnings),
            object_xy=previous.object_xy,
            robot_pose=previous.robot_pose,
            approach=previous.approach,
        )

    # -- the correlation, re-taken (SAFETY.md F-13) ---------------------
    def _recorrelate_mission(self, mission: Mission) -> corr.CorrelationResult:
        """Correlate the MISSION's own fix against the state as it is NOW.

        Not the state as it was at dispatch, and not the current
        ``trash_goal`` - the mission's fix is the position we are driving to
        and the position the blind pick will close on, so it is that position
        which has to keep naming exactly one target.
        """
        incoming = mission.incoming
        if incoming is None:
            return corr.CorrelationResult(
                corr.NO_MATCH,
                detail="the mission kept no goal fix, so it cannot be re-correlated",
            )
        return self._correlate(
            float(incoming.latitude_deg),
            float(incoming.longitude_deg),
            self._datum_tracker.datum,
            bool(incoming.well_formed),
            cross_check_reported_id=False,
        )

    def _correlation_holds(self, mission: Mission) -> Tuple[bool, str]:
        """Does the id this mission carries STILL follow from its own fix?

        THE ANSWER IS COMPUTED HERE, EVERY TIME IT IS ASKED. It is never
        inherited from the dispatch decision. That is the whole of SAFETY.md
        F-13: the correlation used to be enforced once, when the goal was sent,
        and an ambiguity that arose 0.56 s later changed nothing at all -
        neither the drive, nor the pick, nor the irreversible acknowledgement.

        Anything other than a unique match NAMING THIS MISSION'S id is a
        refusal, in every direction:
          * AMBIGUOUS   - a second target moved inside `goal_match_tolerance_m`;
          * NO_MATCH    - the target list no longer has anything there, or the
                          one match is now flagged collected;
          * TARGETS_STALE - the list itself stopped being refreshed, so it is no
                          longer evidence about anything; re-asking a frozen list
                          returns the same confident answer for ever, which is
                          what made this gate look sound while it was not
                          (SAFETY.md F-28);
          * a unique match with a DIFFERENT id - the id space moved under us;
          * an exception - a re-correlation we could not complete is not a
                           re-correlation that passed.
        Nearest-wins is never an option; that is `correlation.py`'s rule and it
        does not change because we are already moving.
        """
        try:
            result = self._recorrelate_mission(mission)
        except Exception as exc:  # noqa: BLE001 - refusing is the fail-safe answer
            self._mission_correlation = "RECHECK_FAILED"
            return False, (
                f"the re-correlation itself failed ({exc!r}), so the id cannot be "
                "confirmed and is treated as unconfirmed"
            )
        self._mission_correlation = result.status
        if result.unique and result.target_id == mission.target_id:
            mission.unique_seen = True
            return True, result.detail

        if result.unique:
            reason = (
                f"the mission's own fix now names target {result.target_id} at "
                f"{result.distance_m:.3f} m, not target {mission.target_id} it was "
                "dispatched for"
            )
            if not mission.latch_void_reason:
                mission.latch_void_reason = reason
            return False, reason

        if result.status != corr.NO_MATCH and not mission.latch_void_reason:
            # AMBIGUOUS, TARGETS_STALE, NO_TARGETS: none of these is the
            # self-occlusion case, and each of them permanently voids the latch.
            mission.latch_void_reason = f"{result.status}: {result.detail}"

        held, why = self._occlusion_latch_holds(mission, result)
        if held:
            return True, why
        return False, f"{result.status}: {result.detail}"

    def _occlusion_latch_holds(self, mission: Mission, result) -> Tuple[bool, str]:
        """INTERIM. Let a NO_MATCH pass while armed, on a mission that once matched.

        WHY THIS EXISTS. On 2026-08-25 the robot navigated correctly to a piece
        of litter, stood at the grasp standoff, and did not pick it up: from
        there the ROBOT ITSELF OCCLUDED THE OBJECT, the Octopus stopped seeing
        it, it left their target list, and the arrival gate read NO_MATCH and
        refused to actuate the arm. The failure is systematic rather than
        occasional -- every correct approach ends with the robot in front of the
        thing it came for -- so the better the navigation, the more reliably the
        pick is refused. A failure mode that good behaviour causes is not a
        safety property.

        WHAT IT DOES NOT COVER, AND THAT IS THE POINT. SAFETY.md F-13 bundles
        NO_MATCH with AMBIGUOUS, but the two mean opposite things here:
        AMBIGUOUS says a second object arrived and we might take the wrong one;
        NO_MATCH-by-occlusion says the object is exactly where we thought and we
        are standing in front of it. This latch covers NO_MATCH ONLY. Ambiguity,
        an id that moved, a stale list and a failed re-correlation all still
        refuse, and any of them voids the latch PERMANENTLY for this mission --
        an ambiguity seen once is not undone by the ambiguity going away.

        ⚠ WHAT IT COSTS, STATED PLAINLY. It re-introduces exactly the gap F-13
        closed, for one status value: while it holds, "the correlation was
        unique when we set off" IS being treated as a statement about now. If
        the object was removed by something other than our own occlusion -- wind,
        a person, a bad fix -- the arm actuates on a position that no longer has
        anything at it. The bound on that is the arming window (120 s from the
        teleop), which is the operator's assertion under SR-16 that the arm's
        workspace is clear for its duration.

        USER DECISION 2026-08-25, taken with that cost stated. The coordinator
        recommended a narrower form -- additionally requiring the robot to be
        within `grasp.tolerance_m` of its own fix, so that self-occlusion is the
        only available explanation, and a bound of seconds rather than the
        arming window -- and the operator chose the arming window. Recorded here
        rather than quietly widened or quietly narrowed. The durable fix is
        designed separately; this is scaffolding with a name on it.

        Ships DISABLED by default (`occlusion_latch_enabled: false`) and is
        startup-only, so it cannot be switched on under a running node.
        """
        if not bool(self.get_parameter("occlusion_latch_enabled").value):
            return False, ""
        if result.status != corr.NO_MATCH:
            return False, ""
        if not mission.unique_seen:
            # Never matched at all: there is no earlier unique correlation to
            # stand on, so this is not occlusion, it is a fix we never confirmed.
            return False, ""
        if mission.latch_void_reason:
            return False, ""
        if not self._arming.is_armed(self._safety_now()):
            # The operator's own condition: the latch dies with the arming.
            return False, ""
        self.get_logger().warning(
            f"OCCLUSION LATCH HOLDING for target {mission.target_id}: the target "
            "list no longer has anything at this mission's fix, and the "
            "correlation gate is being allowed to pass on the earlier unique "
            "match because we are armed (interim, user decision 2026-08-25). "
            "This is the F-13 gap, deliberately reopened for NO_MATCH only."
        )
        return True, (
            f"NO_MATCH held by the interim occlusion latch (armed); last unique "
            f"correlation named target {mission.target_id}"
        )

    # -- the decision --------------------------------------------------
    def _maybe_dispatch(self, resolution: Optional[GoalResolution], now: float) -> None:
        """The ONLY place a NavigateToPose goal is created. C-5.

        Every path into it runs `validate_dispatch` on the pose that is about to
        be sent. There is no second, inline set of checks anywhere - that is the
        whole point of F-9's countermeasure: an unused second gate rots.
        """
        if resolution is None or not resolution.result.accepted:
            return
        pose = resolution.result.robot_pose
        if pose is None:
            return

        dctx = self._dispatch_context(resolution, pose)
        verdict = val.validate_dispatch(dctx, resolution.ctx)
        if not verdict.accepted:
            self._log_dispatch_block(resolution.target_id, verdict)
            return

        self._send_nav_goal(resolution, pose, now)

    def _dispatch_context(
        self,
        resolution: GoalResolution,
        pose: Tuple[float, float, float],
    ) -> val.DispatchContext:
        """The second gate's inputs. Note what NONE of them can be set by the
        external system: arming and the teleop mode are operator-owned, the
        server availability and the datum are observations.

        It took a ROS-time `now` until SAFETY.md F-38 and takes no time argument
        at all now, for the reason `_is_armed` takes none: every age it builds is
        monotonic, and a caller handing in the tick's ROS instant is exactly how
        the two epochs got mixed in the first place."""
        with self._arming_lock:
            armed = self._arming.is_armed(self._safety_now())
        # NOT `now`: `now` is this tick's ROS instant and the mode stamp is
        # monotonic (SAFETY.md F-38). Mixing the two is the F-29 class of bug.
        mode_age = (
            None
            if self._teleop_mode_mono_sec is None
            else max(0.0, self._safety_now() - self._teleop_mode_mono_sec)
        )
        return val.DispatchContext(
            armed=armed,
            dry_run=self._dry_run,
            link_alive=bool(self._link_connected),
            teleop_mode=self._teleop_mode,
            # The check that catches a DEAD MUX: a mode that stopped being
            # republished is not a mode we may act on, however good its last
            # value looked (SAFETY.md F-9). `None` here is a rejection.
            teleop_mode_age_sec=mode_age,
            nav2_available=self._nav2_available,
            datum_unchanged=self._datum_unchanged_since(
                resolution.datum_lat, resolution.datum_lon
            ),
            pose=pose,
            max_teleop_mode_age_sec=float(
                self.get_parameter("max_teleop_mode_age_sec").value
            ),
        )

    def _datum_unchanged_since(self, lat: float, lon: float) -> bool:
        """Has the datum moved enough to invalidate a pose derived from it?

        With ``datum_jump_warn_m`` TO-VERIFY, ANY movement counts - their
        threshold is 1e-9 deg, so this is the conservative reading and it
        invents no number. Once the value is measured, movements below it are
        tolerated, which is what stops a jittering marker from cancelling a
        goal continuously (FR-12 item 5).
        """
        current = self._datum_tracker.datum
        if current is None or not (math.isfinite(lat) and math.isfinite(lon)):
            return False
        warn_m = self._measured("datum_jump_warn_m")
        if warn_m is None:
            return current.latitude_deg == lat and current.longitude_deg == lon
        return datum_offset_m(Datum(lat, lon), current) <= warn_m

    def _log_dispatch_block(self, target_id: str, verdict: val.ValidationResult) -> None:
        """One line per CHANGED block, not one per tick.

        A disarmed gateway republishing "not armed" twice a second would bury
        the one message that matters. PREVIEW verdicts (disarmed, dry_run) are
        INFO because they are the designed state; rejections keep the
        severity rule of FR-12 item 6.
        """
        key = (target_id, verdict.reason)
        if key == self._last_dispatch_block:
            return
        self._last_dispatch_block = key
        if verdict.verdict == val.VERDICT_PREVIEW:
            self.get_logger().info(
                f"goal {target_id or '<uncorrelated>'} is dispatchable but held: "
                f"{verdict.reason} ({verdict.detail})"
            )
        elif verdict.severity == val.SEVERITY_LOCAL:
            self.get_logger().error(
                f"goal {target_id or '<uncorrelated>'} not dispatched "
                f"({verdict.reason}): {verdict.detail}"
            )
        else:
            self.get_logger().warn(
                f"goal {target_id or '<uncorrelated>'} not dispatched "
                f"({verdict.reason}): {verdict.detail}"
            )

    # -- sending -------------------------------------------------------
    def _send_nav_goal(
        self, resolution: GoalResolution, pose: Tuple[float, float, float], now: float
    ) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = Point(x=float(pose[0]), y=float(pose[1]), z=0.0)
        qx, qy, qz, qw = _quaternion_from_yaw(pose[2])
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        mission = Mission(
            target_id=resolution.target_id,
            object_xy=resolution.result.object_xy or (float("nan"), float("nan")),
            pose=pose,
            datum_lat=resolution.datum_lat,
            datum_lon=resolution.datum_lon,
            started_at_sec=now,
            incoming=resolution.incoming,
        )
        # Published BEFORE the request goes out, so that a disarm arriving in
        # the same instant finds something to cancel. The window in which the
        # goal has no handle yet is handled in `_on_nav_goal_response`.
        with self._mission_lock:
            self._mission = mission
        self._nav_state = NAV_NAVIGATING
        self._nav_state_reason = ""
        self._ack_suppressed_reason = ""
        self._last_dispatch_block = None

        self.get_logger().warn(
            f"DISPATCHING target {mission.target_id} to {self._nav_action_name}: "
            f"standing pose ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg), "
            f"object at ({mission.object_xy[0]:.3f}, {mission.object_xy[1]:.3f}), "
            f"attempt {self._attempts.get(mission.target_id, 0) + 1}"
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_nav_goal_response(mission, f))

    def _on_nav_goal_response(self, mission: Mission, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - a dead server must not kill the node
            self.get_logger().error(f"send_goal to {self._nav_action_name} failed: {exc!r}")
            self._fail_attempt(mission, "NAV2_SEND_FAILED", repr(exc))
            return
        if not handle.accepted:
            self.get_logger().error(
                f"{self._nav_action_name} REJECTED the goal for target "
                f"{mission.target_id}. Nav2 refusing a goal is our-side news, "
                "not the source's."
            )
            self._fail_attempt(mission, "NAV2_GOAL_REJECTED", "the server rejected the goal")
            return

        with self._mission_lock:
            mission.nav_handle = handle
            mission.nav_accepted = True
            cancelling = mission.cancelling
        handle.get_result_async().add_done_callback(
            lambda f: self._on_nav_result(mission, f)
        )
        if cancelling:
            # A disarm arrived while the request was in flight, so the goal was
            # accepted by a gate that is already shut. Cancel it the moment a
            # handle exists - this is the window that would otherwise leave a
            # goal running with the gateway believing it had cancelled.
            self.get_logger().error(
                f"target {mission.target_id} was accepted by Nav2 AFTER the gate "
                f"closed ({mission.cancel_reason}); cancelling immediately"
            )
            self._send_cancel(mission, handle, "navigate_to_pose")

    def _on_nav_result(self, mission: Mission, future) -> None:
        try:
            outcome = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"navigation result failed: {exc!r}")
            self._fail_attempt(mission, "NAV2_RESULT_FAILED", repr(exc))
            return
        status = outcome.status
        with self._mission_lock:
            mission.cancel_confirmed = True  # a terminal result ends the cancel question

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_arrival(mission)
            return
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f"navigation to target {mission.target_id} was CANCELLED "
                f"({mission.cancel_reason or 'no reason recorded'}). Not counted as "
                "an attempt: the target did not fail, we withdrew."
            )
            self._clear_mission(mission)
            return

        self.get_logger().error(
            f"navigation to target {mission.target_id} ended with status {status} "
            "(ABORTED). Counting it against both the per-target attempts and the "
            "consecutive-abort budget."
        )
        with self._arming_lock:
            event = self._arming.note_goal_aborted(self._safety_now())
        self._handle_disarm(event)
        self._fail_attempt(mission, "NAV2_ABORTED", f"action status {status}")

    # -- arrival, the reached check and the pick ------------------------
    def _on_arrival(self, mission: Mission) -> None:
        """Nav2 says we are there. SR-16 condition 3 starts here and nowhere else."""
        offset = self._grasp_offset()
        pose, age, _ = self._robot_pose()
        reached_detail = "no TF, reached check not possible"
        if pose is not None and offset.configured:
            verdict = check_reached(mission.object_xy, pose, offset)
            reached_detail = verdict.detail
            if not verdict.known:
                # Loud, and it proceeds. The alternative - treating unknown as
                # "not reached" - would blacklist every target after two
                # attempts on a value nobody has measured yet.
                self.get_logger().warn(
                    f"target {mission.target_id}: {verdict.detail}. Proceeding on "
                    "Nav2's arrival alone; the reached check cannot confirm it."
                )
            elif not verdict.reached:
                self.get_logger().error(
                    f"target {mission.target_id} arrived, but {verdict.detail}. "
                    "Not picking: the blind sequence would close on nothing."
                )
                self._fail_attempt(mission, verdict.reason, verdict.detail)
                return

        self.get_logger().info(
            f"target {mission.target_id} REACHED ({reached_detail})"
        )
        # ARRIVAL IS REPORTED IN ITS OWN RIGHT, on its own state, BEFORE anything
        # decides what follows. It used to be inferable from an acknowledgement;
        # since C-7 an acknowledgement means a successful pick and nothing else,
        # so "we got there" needs a signal that does not depend on what happens
        # next. Every later outcome - picked, suppressed, blacklisted - overwrites
        # this with its own state, and none of them can erase the fact that it
        # was published.
        self._reached.append(mission.target_id)
        self._last_reached_detail = reached_detail
        self._publish_mission_status(mission, ExternalGoalStatus.STATE_REACHED, "")
        with self._arming_lock:
            armed = self._arming.is_armed(self._safety_now())
            self._arming.note_goal_succeeded()

        # SR-16, conditions enumerated at the call site as the requirement asks:
        #  1. only /pick_plastic, the fixed blind sequence of FR-3;
        #  2. only while ARMED - the arming act IS the approval, replacing
        #     SR-1's per-test approval for this one action;
        #  3. only after a validated goal AND a successful NavigateToPose;
        #  4. bounded per target by max_attempts_per_target;
        #  5. auto_pick is its own parameter, default false - both must be true;
        #  6. the FIRST exercise on the real robot still needs its own SR-1
        #     approval, which is why octopus_link_real.yaml keeps auto_pick false;
        #  7. arming asserts the workspace is clear for the duration typed;
        #  8. the correlation must STILL name this target uniquely, re-taken
        #     here rather than inherited from the dispatch (SAFETY.md F-13);
        #  9. on the real robot only, `grasp.tolerance_m` must be measured, so
        #     that "arrived" is a checked claim and not an assumed one
        #     (SAFETY.md F-14, user decision 2026-08-19).
        if not armed:
            self._finish_unacknowledged(mission, ACK_DISARMED, structural=False)
            return
        # Condition 8. The pick is the point of no return for the OBJECT even
        # when it is not yet the point of no return for their mission: a blind
        # sequence that closes on an object we can no longer name collects
        # something we can never report. Not structural - an ambiguity can clear
        # - so it spends an attempt rather than condemning the target at once.
        holds, correlation_detail = self._correlation_holds(mission)
        if not holds:
            self.get_logger().error(
                f"target {mission.target_id} arrived, but its correlation no "
                f"longer holds: {correlation_detail}. NOT picking: the blind "
                "sequence would close on an object we cannot name, and nothing "
                "we cannot name may ever be acknowledged (SAFETY.md F-13)."
            )
            self._finish_unacknowledged(
                mission, ACK_CORRELATION_CHANGED, structural=False
            )
            return
        # Condition 9.
        if self._pick_needs_measured_tolerance and not offset.tolerance_configured:
            self.get_logger().error(
                f"target {mission.target_id} arrived, but grasp.tolerance_m is "
                "unmeasured and this is the REAL ROBOT's domain. NOT picking: "
                "the reached check cannot return a verdict, so 'arrived' is an "
                "unsupported claim and the arm would actuate on it. Both "
                "grasp.offset_x_m and grasp.tolerance_m come out of the same "
                "bench procedure (user decision 2026-08-19, SAFETY.md F-14)."
            )
            self._finish_unacknowledged(
                mission, ACK_TOLERANCE_UNMEASURED, structural=True
            )
            return
        if not self._auto_pick:
            self._finish_unacknowledged(mission, ACK_AUTO_PICK_OFF, structural=True)
            return
        if not self._pick_available:
            self._finish_unacknowledged(mission, ACK_NO_PICK_CLIENT, structural=True)
            return

        with self._mission_lock:
            if mission.cancelling:
                return
            mission.state = NAV_PICKING
        self._nav_state = NAV_PICKING
        self.get_logger().warn(
            f"SR-16: sending PickPlastic for target {mission.target_id} while armed. "
            "Arm and gripper WILL move; the arming act is the approval."
        )
        goal = PickPlastic.Goal()
        goal.execute = True
        future = self._pick_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_pick_goal_response(mission, f))

    def _on_pick_goal_response(self, mission: Mission, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"send_goal to {self._pick_action_name} failed: {exc!r}")
            self._fail_attempt(mission, "PICK_SEND_FAILED", repr(exc))
            return
        if not handle.accepted:
            self._fail_attempt(mission, "PICK_GOAL_REJECTED", "the arm server rejected the pick")
            return
        with self._mission_lock:
            mission.pick_handle = handle
            cancelling = mission.cancelling
        handle.get_result_async().add_done_callback(
            lambda f: self._on_pick_result(mission, f)
        )
        if cancelling:
            self.get_logger().error(
                f"pick for target {mission.target_id} was accepted after the gate "
                f"closed ({mission.cancel_reason}); cancelling immediately"
            )
            self._send_cancel(mission, handle, "pick_plastic")

    def _on_pick_result(self, mission: Mission, future) -> None:
        try:
            outcome = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"pick result failed: {exc!r}")
            self._fail_attempt(mission, "PICK_RESULT_FAILED", repr(exc))
            return
        with self._mission_lock:
            mission.cancel_confirmed = True
        if outcome.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f"pick for target {mission.target_id} was cancelled "
                f"({mission.cancel_reason or 'no reason recorded'}); NOT acknowledged"
            )
            self._clear_mission(mission)
            return
        result = outcome.result
        if outcome.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self.get_logger().error(
                f"pick FAILED for target {mission.target_id}: "
                f"{result.message or 'no message'}. Not acknowledged - the "
                "acknowledgement means collected, and nothing was collected."
            )
            self._fail_attempt(mission, ACK_PICK_FAILED, result.message or "")
            return

        self.get_logger().info(
            f"pick SUCCEEDED for target {mission.target_id}: {result.message}"
        )
        self._acknowledge(mission)

    # -- the acknowledgement, C-7 --------------------------------------
    def _acknowledge(self, mission: Mission) -> None:
        """Publish `trash_goal_done`. The one irreversible thing this node does.

        Four conditions. The docstring used to claim all four were re-checked
        here; three were, and the fourth - the correlation - was inherited from
        the dispatch decision and never looked at again. That gap is SAFETY.md
        F-13, and an auditor drove it: a fix that became ambiguous 0.56 s after
        dispatch still acknowledged an id the correlation had explicitly refused
        to supply. The list is now true, and it is true because the code below
        does it, not because this paragraph says so:

          1. a successful pick - the caller's job, and `_on_pick_result`'s
             success branch is the only caller there is;
          2. an ARMED gate, read at this instant (`is_armed(now)`);
          3. an id to name - `mission.target_id` non-empty;
          4. a UNIQUE CORRELATION, recomputed HERE from the mission's own fix
             against the target list as it is at this instant, and required to
             name this mission's id. Ambiguity is a refusal at this point
             exactly as it is at dispatch; nearest-wins does not become
             acceptable because the arm has already moved.

        Their protocol has no un-acknowledge and no failure channel, so a wrong
        one is permanent (SAFETY.md F-6, F-13).

        There is exactly ONE caller, ``_on_pick_result``, on the success branch.
        That is the whole of C-7 as a structural property: no parameter, no
        configuration and no other code path reaches this method, so an
        acknowledgement cannot follow anything but a successful pick.
        """
        with self._arming_lock:
            armed = self._arming.is_armed(self._safety_now())
        if not armed:
            # The pick succeeded, so the object really is collected - and we
            # still do not acknowledge, because C-7 permits no acknowledgement
            # from a closed gate. The cost is a stalled mission entry, which is
            # recoverable; the cost of the other choice is not.
            self.get_logger().error(
                f"target {mission.target_id} was picked successfully, but the gate "
                "closed before the result arrived. NOT acknowledging (C-7: never "
                "while disarmed). Their mission will not advance past this target "
                "until an operator resolves it."
            )
            self._finish_unacknowledged(mission, ACK_DISARMED, structural=False)
            return
        if not mission.target_id:
            self._finish_unacknowledged(mission, ACK_NOT_CORRELATED, structural=True)
            return
        # Condition 4, and the last chance to catch it. Structural here, unlike
        # at the pick gate: the object has already been collected, so re-driving
        # to it would repeat work that is physically done - blacklisting says
        # "stop trying", which is the honest state. What it does NOT do is
        # acknowledge; an object we cannot name is never reported collected,
        # however certain we are that we collected something.
        holds, correlation_detail = self._correlation_holds(mission)
        if not holds:
            self.get_logger().error(
                f"target {mission.target_id} was picked successfully, but its "
                f"correlation no longer holds: {correlation_detail}. NOT "
                "acknowledging: we would be naming an object we cannot prove we "
                "collected, and their protocol has no way to take that back "
                "(SAFETY.md F-13). The object IS collected; only the id is "
                "unprovable."
            )
            self._finish_unacknowledged(
                mission, ACK_CORRELATION_CHANGED, structural=True
            )
            return
        if self._goal_done_pub is None:
            self._finish_unacknowledged(mission, "NO_ACK_PUBLISHER", structural=True)
            return

        msg = String()
        msg.data = mission.target_id
        self._goal_done_pub.publish(msg)
        self._acknowledged.append(mission.target_id)
        self._attempts.pop(mission.target_id, None)
        self.get_logger().warn(
            f"ACKNOWLEDGED target {mission.target_id} as COLLECTED "
            f"(trash_goal_done). This is irreversible on their side."
        )
        self._publish_mission_status(mission, ExternalGoalStatus.STATE_DONE, "", acknowledged=True)
        self._clear_mission(mission)

    def _finish_unacknowledged(
        self, mission: Mission, reason: str, structural: bool
    ) -> None:
        """Arrived, nothing to acknowledge with. Loud, and it must not loop.

        Their side republishes the same goal until it is acknowledged, so a
        completion with no acknowledgement would otherwise drive to the same
        object for ever. A STRUCTURAL reason - auto-pick off, no pick server, no
        id - cannot be improved by trying again, so the target is blacklisted
        immediately with the reason stated as a configuration outcome rather
        than a defect of the target. A non-structural one spends an attempt.
        """
        self._ack_suppressed_reason = reason
        self.get_logger().error(
            f"target {mission.target_id or '<uncorrelated>'} completed WITHOUT an "
            f"acknowledgement ({reason}). Their protocol advances only on "
            "trash_goal_done and has no failure channel, so their mission stops "
            "here until this is resolved (proposal item 2)."
        )
        if structural:
            self._blacklist_target(
                mission.target_id,
                f"{reason}: retrying cannot change this outcome",
            )
            self._publish_mission_status(
                mission, ExternalGoalStatus.STATE_BLACKLISTED, reason
            )
            self._clear_mission(mission)
            return
        self._fail_attempt(mission, reason, "arrived but could not acknowledge")

    # -- failure bookkeeping -------------------------------------------
    def _fail_attempt(self, mission: Mission, reason: str, detail: str) -> None:
        target_id = mission.target_id
        attempts = self._attempts.get(target_id, 0) + 1 if target_id else 0
        if target_id:
            self._attempts[target_id] = attempts
        max_attempts = int(self.get_parameter("max_attempts_per_target").value)
        self._last_reason = reason
        self._last_severity = val.SEVERITY_LOCAL
        self.get_logger().error(
            f"target {target_id or '<uncorrelated>'} attempt {attempts}/{max_attempts} "
            f"failed ({reason}): {detail}"
        )
        if target_id and attempts >= max_attempts:
            self._blacklist_target(
                target_id,
                f"{attempts} attempt(s) failed, last {reason}",
                latlon=self._mission_object_latlon(mission),
            )
            self._publish_mission_status(
                mission, ExternalGoalStatus.STATE_BLACKLISTED, reason
            )
        else:
            self._publish_mission_status(mission, ExternalGoalStatus.STATE_FAILED, reason)
        self._clear_mission(mission)

    def _blacklist_target(
        self,
        target_id: str,
        why: str,
        latlon: Optional[Tuple[float, float]] = None,
    ) -> None:
        if not target_id or target_id in self._blacklist:
            return
        self._blacklist.append(target_id)
        # WGS84, not map metres: the anchor has to survive a datum move, and
        # lat/lon is the only form of the position that does.
        if latlon is not None:
            self._blacklist_latlon[target_id] = (float(latlon[0]), float(latlon[1]))
        # max_attempts_per_target = 2 is PROVISIONAL and not user-confirmed
        # (SR-16 condition 4). It is a parameter for exactly that reason.
        self.get_logger().error(
            f"BLACKLISTING target {target_id}: {why}. It will NOT be acknowledged - "
            "acknowledging would tell the source we collected it. Their mission "
            "cannot advance past it, which is the honest outcome until they add a "
            "failure channel (FR-12 item 7, proposal item 2)."
        )

    def _clear_mission(self, mission: Mission) -> None:
        with self._mission_lock:
            if self._mission is mission:
                self._mission = None
            mission.done.set()
        self._nav_state = NAV_IDLE if self._nav2_available else NAV_UNAVAILABLE
        self._last_dispatch_block = None

    # -- supervision of the goal in flight ------------------------------
    def _supervise_mission(self, mission: Mission, now: float) -> None:
        """Re-validate what is running, through the SAME gate that dispatched it.

        FR-12 asks for re-validation at dispatch; a goal that is already moving
        is the case that matters more, because the world it was validated
        against is the world it is currently driving through. A geofence changed
        with `ros2 param set`, a datum drag, a mux that went silent or a mode
        that left `autonomous` all cancel the goal in flight here - through
        `validate_dispatch`, not through a second set of inline checks (C-5).
        """
        if mission.cancelling:
            # The server that owned the goal is gone, so no result is coming and
            # there is nothing left for us to supervise. Releasing the slot is
            # honest; pretending we still control that goal is not.
            #
            # But NOT before the cancel timeout has run: `_check_cancel_timeout`
            # is what produces the ERROR log and the ERROR diagnostic SR-15 rule
            # 9 requires, and it can only see a mission that still exists.
            # Releasing first made the requirement's own report unreachable -
            # found by the C-8 NAV2_UNAVAILABLE case, which is the one situation
            # where a cancel genuinely cannot be confirmed.
            # TWICE the timeout, not once. `_check_cancel_timeout` runs in the
            # safety group and produces the ERROR log plus the ERROR diagnostic
            # that SR-15 rule 9 actually requires; this release is only
            # housekeeping. At one timeout the two became due in the same
            # instant and the housekeeping won the race, so the requirement's
            # own report never ran. Reporting comes first; freeing the slot can
            # wait one more window.
            elapsed = now - (mission.cancel_requested_at_sec or now)
            if (
                not self._nav2_available
                and not mission.cancel_confirmed
                and elapsed > 2.0 * self._cancel_confirm_timeout_sec
            ):
                self._note_cancel_failure(
                    f"releasing target {mission.target_id}: {self._nav_action_name} "
                    "is gone and the cancel can no longer be confirmed. If that "
                    "goal is still executing, only the upstream mechanisms will "
                    "stop it - the E-stop, the mux timeout, the firmware timeout."
                )
                self._clear_mission(mission)
            return
        if mission.state != NAV_NAVIGATING or mission.incoming is None:
            # While the arm is picking there is no navigation goal to re-check,
            # and the costmap has nothing to say about a stationary robot. The
            # correlation is still re-checked in that window - at the pick gate
            # and at the acknowledgement - it is only the CANCEL that stops
            # here; see the comment on the correlation block just below.
            return

        # THE CORRELATION, RE-TAKEN ON THE GOAL IN FLIGHT (SAFETY.md F-13).
        # First, and with its own cancel, because it is the one failure whose
        # consequence is irreversible: everything else this method catches costs
        # a re-drive, an ambiguity costs an object marked collected for ever.
        #
        # DECIDED: an ambiguity arising mid-flight CANCELS, it does not merely
        # block the acknowledgement. Refusing only the acknowledgement would let
        # the vehicle drive on and the ARM ACTUATE over a pair of objects
        # 0.1-0.2 m apart, close on whichever one is in front of the gripper,
        # and leave us holding litter we cannot name and cannot report - their
        # protocol has no failure channel. A cancel costs a re-drive and nothing
        # else: `_on_nav_result` does not count a CANCELED goal as an attempt,
        # so a transient ambiguity cannot blacklist the target either, and the
        # goal is re-dispatched by itself once the fix names one object again.
        # The asymmetry decides it. This is the same reasoning `_resolve_goal`
        # already applies before a dispatch - a goal we cannot NAME is a goal we
        # must not drive to - applied to the goal that is already driving.
        holds, detail = self._correlation_holds(mission)
        if not holds:
            self._correlation_cancels += 1
            self._cancel_mission(
                f"CORRELATION_LOST: target {mission.target_id} can no longer be "
                f"named uniquely by its own fix - {detail}",
                now,
                error=True,
            )
            return

        ctx = self._make_context(mission.incoming, current_goal_id=None)
        dctx = val.DispatchContext(
            armed=self._is_armed(),
            dry_run=self._dry_run,
            link_alive=bool(self._link_connected),
            teleop_mode=self._teleop_mode,
            # Monotonic on both sides (SAFETY.md F-38); see `_dispatch_context`.
            teleop_mode_age_sec=(
                None
                if self._teleop_mode_mono_sec is None
                else max(0.0, self._safety_now() - self._teleop_mode_mono_sec)
            ),
            nav2_available=self._nav2_available,
            datum_unchanged=self._datum_unchanged_since(
                mission.datum_lat, mission.datum_lon
            ),
            pose=mission.pose,
            max_teleop_mode_age_sec=float(
                self.get_parameter("max_teleop_mode_age_sec").value
            ),
        )
        verdict = val.validate_dispatch(dctx, ctx)
        if verdict.accepted:
            return
        # NOT_ARMED and DRY_RUN come back as PREVIEW verdicts. Disarm already
        # cancels through the event path, so reaching here means the gate closed
        # without an event - belt and braces, and it must still cancel.
        self._cancel_mission(
            f"{verdict.reason}: {verdict.detail}", now, error=True
        )

    def _is_armed(self) -> bool:
        """Read-time armed state on the MONOTONIC clock (SAFETY.md F-2/F-29).

        Takes no time argument on purpose: it used to, and every caller had to
        remember which of the two clocks the arming machine reasons in. One
        caller passing a ROS-time `now` into a monotonic window would make an
        expired window read as open for the difference between two epochs, which
        is not a bug that announces itself.
        """
        with self._arming_lock:
            return self._arming.is_armed(self._safety_now())

    # -- cancelling ------------------------------------------------------
    def _cancel_mission(self, reason: str, now: float, error: bool = True) -> bool:
        """Cancel whatever is running. NEVER a write to any topic.

        This is the entire stop-shaped contribution of this node and it is a
        request to Nav2, not a command to the vehicle. There is deliberately no
        zero-`Twist`, no `/teleop/set_mode`, no `/hw/joint_commands`: publishing
        any of those would make the gateway a second writer on a chain topic
        (SR-9) and a worse stop than the ones that already exist (SR-15 rule 8).
        Idempotent, so a second trigger during a cancel cannot produce a storm.
        """
        with self._mission_lock:
            mission = self._mission
            if mission is None:
                return False
            if mission.cancelling:
                return True
            mission.cancel_requested_at_sec = now
            mission.cancel_reason = reason
            mission.state = NAV_CANCELLING
            nav_handle = mission.nav_handle
            pick_handle = mission.pick_handle
            accepted = mission.nav_accepted
        self._nav_state = NAV_CANCELLING

        self._log_at(
            "error" if error else "warn",
            f"CANCELLING target {mission.target_id} ({reason}). Cancel only - this "
            "node writes nothing to the motion chain under any condition, "
            "including this one (SR-15 rules 8 and 9).",
        )
        if nav_handle is not None:
            self._send_cancel(mission, nav_handle, "navigate_to_pose")
        if pick_handle is not None:
            self._send_cancel(mission, pick_handle, "pick_plastic")
        if nav_handle is None and not accepted:
            self.get_logger().error(
                f"target {mission.target_id} has no goal handle yet - the send is "
                "still in flight. It will be cancelled the moment Nav2 answers; "
                "until then nothing can be cancelled, because nothing is known to "
                "have been accepted."
            )
        return True

    def _send_cancel(self, mission: Mission, handle, what: str) -> None:
        try:
            future = handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001 - a failed cancel is reported, never fatal
            self.get_logger().error(f"cancel of {what} could not be sent: {exc!r}")
            return
        future.add_done_callback(lambda f: self._on_cancel_response(mission, what, f))

    def _on_cancel_response(self, mission: Mission, what: str, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cancel response for {what} failed: {exc!r}")
            return
        accepted = bool(getattr(response, "goals_canceling", ()))
        with self._mission_lock:
            if accepted:
                mission.cancel_confirmed = True
        if accepted:
            self.get_logger().info(f"cancel of {what} confirmed by the server")
        else:
            # Not yet an error: the goal may simply have finished on its own
            # between the trigger and the request. The result callback decides,
            # and the timeout in the safety tick reports if neither arrives.
            self.get_logger().warn(
                f"{what} reported nothing to cancel; waiting for the result to say "
                "whether it had already finished"
            )

    def _check_cancel_timeout(self, now: float) -> None:
        """Report a cancel that was never confirmed. Safety group, no I/O.

        SR-15 rule 9: this log line plus the ERROR diagnostic is the ENTIRE
        escalation. The node does not act on it - it has no write access to the
        motion chain, by requirement and by construction. Stopping belongs to
        the E-stop, the mux timeout and the firmware timeout.
        """
        with self._mission_lock:
            mission = self._mission
            if (
                mission is None
                or mission.cancel_requested_at_sec is None
                or mission.cancel_confirmed
                or mission.cancel_error_logged
            ):
                return
            if now - mission.cancel_requested_at_sec <= self._cancel_confirm_timeout_sec:
                return
            mission.cancel_error_logged = True
            elapsed = now - mission.cancel_requested_at_sec
            target_id = mission.target_id
        self._note_cancel_failure(
            f"CANCEL NOT CONFIRMED for target {target_id} after {elapsed:.1f}s "
            f"(> cancel_confirm_timeout_sec {self._cancel_confirm_timeout_sec:.1f}s). "
            "Reporting only, and that is the entire escalation (SR-15 rule 9): this "
            "node has no write access to the motion chain and adds no stop "
            "mechanism. If the vehicle is moving, use the E-stop."
        )

    def _note_cancel_failure(self, message: str) -> None:
        """ERROR log + a STICKY ERROR diagnostic. SR-15 rule 9, SR-13.

        Sticky because the mission it belongs to is about to be cleared, and a
        diagnostic that disappears with it would turn "a cancel we could not
        confirm" into a line that scrolled past. SR-13 asks for an active
        signal; an active signal that lasts one publication cycle is not one.
        """
        self._cancel_failures += 1
        self._cancel_failed_sticky = True
        self.get_logger().error(message)

    def _publish_mission_status(
        self,
        mission: Mission,
        state: int,
        reason: str,
        acknowledged: bool = False,
    ) -> None:
        msg = ExternalGoalStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_id = mission.target_id
        msg.state = state
        msg.reason = reason
        msg.detail = mission.cancel_reason or ""
        msg.attempts = int(self._attempts.get(mission.target_id, 0))
        msg.acknowledged = bool(acknowledged)
        self._status_pub.publish(msg)

    # ==================================================================
    # the safety tick - the only thing that may not be starved
    # ==================================================================
    def _safety_tick(self) -> None:
        """Arming expiry and the link watchdog. Own callback group, no I/O.

        Deliberately short and free of anything that can block: it is the tick
        that closes the gate, so anything slow in it would defeat its own
        purpose (SAFETY.md F-2).

        TWO CLOCKS, named apart on purpose (SAFETY.md F-29). The arming expiry
        and the link watchdog are promises made in wall-clock terms and are
        measured on the monotonic clock; the cancel-confirm report compares
        ROS-time stamps taken when the cancel was requested and must stay on the
        ROS clock. Passing the wrong one of these into the wrong callee is the
        whole class of bug F-29 is about, which is why neither is called `now`.
        """
        mono = self._safety_now()
        with self._arming_lock:
            event = self._arming.poll(mono)
        self._handle_disarm(event)
        self._link_watchdog(mono)
        # Deliberately in the SAFETY group: a cancel that was not confirmed is
        # exactly the situation in which the work group may be the thing that is
        # stuck, so the report must not depend on it. Pure arithmetic and a log.
        self._check_cancel_timeout(self._ros_now())

    def _link_watchdog(self, mono: float) -> None:
        """ABSENCE of `link_status` is `LINK_LOST` (C-3 / SAFETY.md F-3).

        MONOTONIC, since SAFETY.md F-29. `link_lost_sec` is a statement about a
        WebSocket over WiFi and about a node that publishes at a fixed wall-clock
        rate; neither of them slows down when the simulation does, so measuring
        their silence in sim time made a 5 s tolerance mean 50 wall-seconds of
        dead link on a twin at a real-time factor of 0.1. The arrival time this
        compares against is taken from the same clock in `_on_link_status`.

        `note_link` used to be reachable only from `_on_link_status`, i.e. the
        gateway could only learn that the link was down from the very node whose
        job it is to report that - and `link_status` is `TRANSIENT_LOCAL`, so if
        `octopus_link_node` crashed, was OOM-killed or was never started, the
        last `connected: true` simply stood for ever. Health is an active
        signal, not the absence of a bad one (SR-13), so silence is judged here
        against `link_lost_sec`.

        Never having seen a `link_status` counts as silence, measured from node
        start. That is the fail-safe direction: it costs nothing while disarmed
        (a disarm while disarmed is a no-op) and it means arming without a
        living link node closes the gate again within `link_lost_sec` instead of
        supervising a goal on a link that does not exist.
        """
        if self._link_status_stamp_sec is None:
            silent_for = mono - self._started_mono
            source = "no link_status has ever arrived"
        else:
            silent_for = mono - self._link_status_stamp_sec
            source = "last link_status"
        if silent_for <= self._link_lost_sec:
            return

        if not self._link_watchdog_tripped:
            self._link_watchdog_tripped = True
            self.get_logger().error(
                f"link watchdog: {source} is {silent_for:.1f} s old, over "
                f"link_lost_sec={self._link_lost_sec:.1f} s. Treating the link "
                "as LOST. This fires when the reporting node itself is gone, "
                "which a message-driven check cannot see (SR-15 rule 10)."
            )
        # The reported view must follow, or telemetry and diagnostics would keep
        # showing a connection that has no process behind it.
        self._link_connected = False
        with self._arming_lock:
            event = self._arming.note_link(False, mono)
        self._handle_disarm(event)

    def _clock_watchdog(self) -> None:
        """Is the clock the SIM-TIME half of this node is measured on moving?

        SAFETY.md F-24. With ``use_sim_time`` true and no ``/clock`` publisher -
        which is what the twin launch file produced, and what a paused or killed
        Gazebo produces at any moment - the ROS clock never advances, and NOT ONE
        ROS-clock timer in this node fires: not the in-flight re-validation, not
        the correlation gate, not the cancel-confirm report, not the preview and
        not the dispatch decision. Measured: 0 timer callbacks in 3 s against 14
        on a steady clock in the same process. The node meanwhile reports itself
        up and says nothing, which is the silent-health failure SR-13 exists to
        forbid and the same shape as F-2 and F-3 - a safety mechanism that
        stopped without saying so.

        THIS CALLBACK RUNS ON A STEADY CLOCK. That is the whole mechanism: the
        watchdog for a stopped clock cannot be scheduled by the stopped clock,
        and no parameter, config file or `/clock` publisher reaches
        ``ClockType.STEADY_TIME``.

        SINCE SAFETY.md F-29 the arming expiry and the link watchdog no longer
        depend on this clock at all - they are measured monotonically, because
        they are promises in wall-clock terms (see `_safety_now`). That narrows
        what a stall costs; it does not remove it, and the response here is
        unchanged. Every age this node computes against a sim-time stamp - the
        TF pose, the re-validation of a goal in flight, the correlation that
        names the target before an irreversible acknowledgement - is still
        measured on this clock, and all of them are inert while it is stopped.
        So a stall still DISARMS, once, on the transition, and arming is still
        refused for as long as it lasts (`_on_set_arming`). What it does NOT do
        is stop the robot - nothing in this node does (SR-15 rule 11).

        THREE distinct verdicts, because they have three different causes and
        three different recoveries (SAFETY.md F-30/F-31):

        * the clock ADVANCED - the only healthy answer, and the only one that
          can prove the clock;
        * the clock went BACKWARDS - a world reset, not a stopped publisher. It
          disarms (a discontinuity invalidates every age in flight) and then
          RE-BASELINES, so recovery costs one tick instead of the size of the
          jump;
        * the clock did not move - a stall, graded by whether it has EVER been
          proven: before that it is a startup condition with a discovery latency
          behind it and is reported at WARN inside a measured grace; after it,
          it is a failure and is reported at ERROR.

        Note what still works while the ROS clock is frozen, because it bounds
        the damage and it is why disarming is worth doing at all: everything
        EVENT-driven still runs. This callback, the service handler, the mode and
        link subscriptions and every action callback are delivered by the
        executor regardless of the clock, so the disarm below reaches
        `_cancel_mission` and the cancel really is sent.
        """
        mono = time.monotonic()
        try:
            ros = self._ros_now()
        except Exception:  # noqa: BLE001 - an unreadable clock is a stalled one
            ros = None

        if ros is not None and ros > self._clock_ref_ros_sec:
            # SAFETY.md F-40, user decision 2026-08-20. A forward discontinuity
            # used to be indistinguishable from healthy progress here, because
            # this branch only ever asked whether the clock had advanced. It now
            # asks HOW MUCH MORE than a continuous clock could have advanced.
            #
            # Measured as an EXCESS over the observed rate rather than as an
            # absolute step, and that is deliberate: an absolute threshold would
            # fire on every tick of a twin deliberately run faster than the
            # threshold, and rule 12 says in its own words that a safety
            # mechanism which cries wolf gets disabled by whoever is on shift.
            # The rate is the one already observed for F-37; nothing new is
            # measured for this.
            #
            # ONLY once the clock has been PROVEN. Before that, the first advance
            # IS the proof - with `use_sim_time` and a `/clock` that arrives
            # late, the ROS clock steps from 0 to an epoch stamp in one tick, and
            # reporting that as a discontinuity would put a WARN on every
            # ordinary sim start (SAFETY.md F-31 is about exactly that mistake).
            excess = (ros - self._clock_ref_ros_sec) - (
                max(0.0, mono - self._clock_ref_mono) * self._clock_rate.rate
            )
            jumped_forward = (
                self._clock_proven and excess > self._clock_forward_jump_sec
            )
            if jumped_forward:
                # A jump is not a rate: drop the reference pair so the
                # discontinuity is not averaged into the F-37 estimate. The
                # DECISION is taken here, in the watchdog - `clock_rate` stays a
                # pure estimator and never reports anything.
                self._clock_rate.reset()
            # RE-BASELINE FIRST, exactly as the backwards branch does. Everything
            # below is reporting and may fail; the clock must be proven and
            # re-based whatever happens to the report.
            self._note_clock_advancing(ros, mono)
            if jumped_forward:
                try:
                    self._note_clock_jumped_forward(ros, excess)
                except Exception as exc:  # noqa: BLE001
                    # The report is worth a retry, never the watchdog. A raise
                    # escaping here would take the timer callback with it and
                    # cost every later stall and backwards jump too (the F-26 /
                    # F-32 shape).
                    self._clock_forward_report_errors += 1
                    try:
                        self.get_logger().error(
                            "failed to report a forward clock jump: "
                            f"{type(exc).__name__}: {exc}"
                        )
                    except Exception:  # noqa: BLE001
                        pass
            return

        # SAFETY.md F-30. A DECREASE is its own condition and must not be
        # reported as a stall: `_clock_ref_ros_sec` used to be raised only, so a
        # 120 s backwards jump left the reference at the old maximum and every
        # later tick computed a growing "frozen_for" on a clock that was running
        # perfectly - measured: still refusing to arm 30.5 s after the jump,
        # with the clock advancing at 1.0x the whole time, and only recovering
        # once sim time had climbed back past the old maximum, i.e. after
        # exactly the size of the jump.
        if (
            ros is not None
            and (self._clock_ref_ros_sec - ros) > self._clock_backward_eps_sec
        ):
            self._note_clock_jumped_back(ros, mono)
            return

        frozen_for = mono - self._clock_ref_mono
        if frozen_for <= self._clock_stall_sec:
            return
        self._note_clock_stalled(frozen_for, mono)

    def _clock_in_startup_grace(self, mono: Optional[float] = None) -> bool:
        """Is an unproven clock still inside the measured discovery grace?

        SAFETY.md F-31. Only ever true while the clock has NEVER been proven: a
        clock that stopped after having been proven is a failure from its first
        tolerated second, and no grace applies to it.
        """
        if self._clock_proven:
            return False
        now = time.monotonic() if mono is None else mono
        return (now - self._started_mono) <= self._clock_startup_grace_sec

    def _note_clock_advancing(self, ros: float, mono: float) -> None:
        """The healthy branch: re-baseline, and clear whatever was latched."""
        # SAFETY.md F-37, and deliberately BEFORE the reference is overwritten:
        # the estimator needs the pair as it was and the pair as it is, and the
        # next two lines destroy the former.
        self._clock_rate.note(ros, mono)
        self._clock_ref_ros_sec = ros
        self._clock_ref_mono = mono
        if not self._clock_proven:
            self._clock_proven = True
            self.get_logger().info(
                f"ROS clock observed advancing (use_sim_time={self._use_sim_time}); "
                "the sim-time half of this node - the TF ages, the in-flight "
                "re-validation and the correlation gate - is measured on it. "
                "Arming stays refused whenever it stops (SAFETY.md F-24). The "
                "arming window and the link watchdog are measured monotonically "
                "and are not affected by it at all (SAFETY.md F-29)."
            )
        elif self._clock_stalled:
            self._clock_stalled = False
            self.get_logger().warn(
                "ROS clock is advancing again, "
                f"{mono - self._clock_stall_logged_mono:.1f}s after the stall "
                "was last reported. "
                "The gate is NOT re-opened by this: a window closed by "
                "CLOCK_STALLED is closed, and re-arming is an operator's "
                "explicit act like every other arming (SR-15 rule 4)."
            )
        self._clock_stall_severity_logged = ""
        # The next stall is a new stall and gets its own disarm (SAFETY.md F-32).
        self._clock_disarm_done = False

    def _note_clock_jumped_forward(self, ros: float, excess: float) -> None:
        """The ROS clock advanced FURTHER than it could have. SAFETY.md F-40.

        DECIDED by the user on 2026-08-20, on a reproduction rather than on an
        argument: **report it - WARN plus a `/diagnostics` value - and do NOT
        disarm and do NOT cancel.**

        What the reproduction showed, and why the decision came out this way. A
        forward jump satisfies the "did the clock advance?" test, so it
        re-baselines the reference and PROVES the clock; the rationale written
        into `_note_clock_jumped_back` - *"every age measured against it refers
        to a timeline that no longer exists"* - applies to it word for word.
        Until F-38 was fixed there was one accidental consequence left: the mode
        age was on the ROS clock, so a jump made it large and the dispatch tick
        sometimes cancelled the goal in flight. Measured: **4 of 8** identical
        jumps cancelled, the other 4 did nothing, because the refusal window was
        one 20 Hz mode period against a 2 Hz observer. After F-38 that became
        **0 of 8**. So there is now nothing whatsoever that covers this case, and
        an operator is told nothing at all - which is what this fixes.

        Why NOT a disarm, in the user's words: on the real robot the only sources
        are a time-sync step and a manual `date` set, and disarming on every step
        of a flaky NAT'd time source would trade a small reporting gap for an
        operational one - and would train the operator to expect spurious
        disarms, which is how a safety mechanism gets switched off.

        THE GATE IS NOT TOUCHED HERE, and that is the point of the method rather
        than an omission: no `_handle_disarm`, no `_cancel_mission`, no
        `note_clock*` into the arming machine, no new refusal path. The arming
        window behaves exactly as it did before this method existed. There is
        deliberately no tenth `ArmingState` trigger constant either - a
        constant would say a disarm can carry this reason, and it cannot.
        """
        self._clock_forward_jumps += 1
        self._last_forward_jump_sec = float(excess)
        self.get_logger().warn(
            f"the ROS clock JUMPED FORWARD: it advanced {excess:.1f}s more than "
            f"a continuous clock at the observed rate could have, to {ros:.3f} "
            f"(use_sim_time={self._use_sim_time}, threshold "
            f"{self._clock_forward_jump_sec:.1f}s, TO-VERIFY). This is a "
            "discontinuity in the same sense a backwards jump is: every age "
            "measured against this clock - the TF pose, the in-flight "
            "re-validation, the correlation that names the target - refers to a "
            "timeline that no longer exists, and any of them may read stale or "
            "fresh for one tick for that reason alone. On the robot the usual "
            "source is a time-sync step or a manual `date` set. "
            "REPORT ONLY, by user decision 2026-08-20: the arming window is NOT "
            "closed, nothing in flight is cancelled, and the clock stays proven "
            "(SAFETY.md F-40). A backwards jump, which DOES disarm, is a "
            "different line and a different trigger."
        )

    def _note_clock_jumped_back(self, ros: float, mono: float) -> None:
        """The ROS clock went BACKWARDS. SAFETY.md F-30, SR-15 rule 7.

        A `ros2 service call /reset_simulation`, a Gazebo world reset, or an NTP
        step on wall time. The publisher is fine; what is broken is the
        continuity of the timeline, and that invalidates every age in flight -
        which is why this disarms exactly as a stall does.

        It reports `CLOCK_JUMPED_BACK` (constant 9) and NOT `CLOCK_STALLED`
        (constant 8). The two were one code until the user split them on
        2026-08-19: the behaviour is identical on purpose, but the operator
        response is not - a stall sends somebody to look for a dead `/clock`
        publisher, a jump sends them to whoever reset the world - and that
        difference has to survive as a number, not only as prose in the detail
        string. The wire-format change was the accepted cost of the decision.

        The difference from a stall is the RECOVERY. The reference is re-based on
        the new value here, so the node is healthy again on the next tick. Before
        this it would have refused to arm until sim time climbed back past the
        old maximum, i.e. for as long as the jump was large - an hour's twin work
        reset means an hour of refusals, with a message sending the operator to
        look for a clock publisher that is running perfectly.
        """
        jumped_by = self._clock_ref_ros_sec - ros
        # SAFETY.md F-37: the next rate sample must not be measured ACROSS the
        # discontinuity. The estimate itself is kept - a world reset changes
        # what the clock says, not how fast it says it.
        self._clock_rate.reset()
        # RE-BASELINE FIRST. Everything below can fail; the recovery must not
        # depend on it, and a reference that is only ever raised is the bug.
        self._clock_ref_ros_sec = ros
        self._clock_ref_mono = mono
        self._clock_stalled = False
        self._clock_stall_severity_logged = ""
        self._clock_disarm_done = False
        self.get_logger().error(
            f"the ROS clock JUMPED BACKWARDS by {jumped_by:.1f}s to {ros:.3f} "
            f"(use_sim_time={self._use_sim_time}). This is a discontinuity, not a "
            "stalled clock: the publisher is running. Every age measured against "
            "it - the TF pose, the in-flight re-validation, the correlation that "
            "names the target - refers to a timeline that no longer exists, so "
            "any armed window is closed NOW and whatever is in flight is "
            "cancelled. The reference has been re-based on the new value, so "
            "arming can be granted again as soon as the clock is seen advancing "
            "(SAFETY.md F-30). This does NOT stop the robot; only the E-stop, "
            "the mux timeout and the firmware timeout do (SR-15 rule 11)."
        )
        try:
            self._publish_diagnostics()
        except Exception as exc:  # noqa: BLE001 - the disarm below is the point
            self.get_logger().error(
                f"clock watchdog: publishing the diagnostic after a backwards "
                f"jump failed ({exc!r}); continuing to the disarm"
            )
        with self._arming_lock:
            # MONOTONIC (SAFETY.md F-29). Not `ros`: this machine reasons in
            # monotonic seconds, and a ROS-clock instant here is an instant from
            # another epoch - `disarm` compares it against the window's expiry
            # and would report TIMEOUT for a clock event. The ROS-side values
            # belong in the detail, which is what it is for.
            event = self._arming.note_clock_jumped_back(
                mono,
                f"the ROS clock jumped BACKWARDS by {jumped_by:.1f}s to "
                f"{ros:.3f} "
                f"(use_sim_time={self._use_sim_time}); every age in flight was "
                "measured against a timeline that no longer exists (SAFETY.md "
                "F-30)",
            )
        self._handle_disarm(event)

    def _note_clock_stalled(self, frozen_for: float, mono: float) -> None:
        """The ROS clock has not moved for longer than `clock_stall_sec`.

        Two states, one gate. SAFETY.md F-31: "never yet proven" is a startup
        condition with a `/clock` discovery latency behind it - measured on this
        laptop, see `clock_startup_grace_sec` in the config files - and "stopped
        after having been proven" is a failure. They are reported at different
        severities and they REFUSE ARMING IDENTICALLY. Grading the report is
        what keeps the ERROR meaning something; grading the gate would be the
        thing F-24 was raised about.
        """
        # The reported state is set FIRST and unconditionally: everything it
        # gates is a refusal, so the fail-safe direction is to have it set even
        # if every line below this one fails.
        self._clock_stalled = True
        never = not self._clock_proven
        in_grace = self._clock_in_startup_grace(mono)
        severity = "warn" if in_grace else "error"
        # Repeat when the SEVERITY changes - which is how the escalation out of
        # the startup grace gets said out loud at the moment it happens - and
        # otherwise at most every 30 s.
        if (
            severity != self._clock_stall_severity_logged
            or (mono - self._clock_stall_logged_mono) >= 30.0
        ):
            self._clock_stall_logged_mono = mono
            self._clock_stall_severity_logged = severity
            if in_grace:
                self._log_at(
                    "warn",
                    "the ROS clock has not advanced YET - still at "
                    f"{self._clock_ref_ros_sec:.3f} after {frozen_for:.1f}s of "
                    "monotonic time, which is inside the "
                    f"{self._clock_startup_grace_sec:.1f}s startup grace for "
                    f"/clock discovery (use_sim_time={self._use_sim_time}, "
                    "SAFETY.md F-31). Nothing is wrong yet: this is how long a "
                    "subscription may take to match a publisher that is starting "
                    "alongside us. ARMING IS REFUSED until the clock is observed "
                    "to advance - the grace grades the report, never the gate - "
                    "and if this is still true when the grace runs out it is "
                    "repeated at ERROR.",
                )
            else:
                self._log_at(
                    "error",
                    ("the ROS clock has NEVER advanced" if never
                     else "the ROS clock STOPPED")
                    + f" - frozen at {self._clock_ref_ros_sec:.3f} for "
                    f"{frozen_for:.1f}s of monotonic time "
                    f"(use_sim_time={self._use_sim_time}, "
                    f"clock_stall_sec={self._clock_stall_sec:.1f}"
                    + (f", past the {self._clock_startup_grace_sec:.1f}s startup "
                       "grace" if never else "")
                    + "). Every timeout in this node that is measured on that "
                    "clock is inert: the cancel-confirm report, the in-flight "
                    "re-validation and the correlation gate. Arming is REFUSED "
                    "while this lasts, and any window that was open is closed "
                    "now. (The arming expiry and the link watchdog are measured "
                    "monotonically since SAFETY.md F-29 and keep running.) "
                    + ("Start the /clock publisher (a paused or dead Gazebo does "
                       "this), or run with use_sim_time:=false."
                       if self._use_sim_time
                       else "This is wall time, so this should be impossible - "
                            "suspect a stopped executor.")
                    + " This does NOT stop the robot; only the E-stop, the mux "
                    "timeout and the firmware timeout do (SAFETY.md F-24, "
                    "SR-15 r.11).",
                )
        # EVENT-DRIVEN, because `_publish_diagnostics` is normally called from a
        # ROS-clock timer that has by definition stopped running. Without this
        # the report above would exist only in the log of a node that looks
        # healthy on /diagnostics - the exact silent-health shape SR-13 forbids.
        try:
            self._publish_diagnostics()
        except Exception as exc:  # noqa: BLE001 - the disarm below is the point
            self.get_logger().error(
                f"clock watchdog: publishing the diagnostic failed ({exc!r}); "
                "continuing to the disarm"
            )
        # SAFETY.md F-32. The disarm is guarded by its OWN latch, and that latch
        # is set only after the disarm has actually returned. It used to be
        # `_clock_stalled` itself, set three statements earlier: a raise in the
        # disarm - the F-26 shape - made the branch unreachable on every later
        # tick, so that cancel was lost for good. Now a raise costs a retry on
        # the next tick, 200 ms later, and `disarm` is idempotent so a retry
        # after a partial success is a no-op.
        if self._clock_disarm_done:
            return
        try:
            with self._arming_lock:
                # MONOTONIC, and deliberately not `_clock_ref_ros_sec`: see the
                # note in `_note_clock_jumped_back`. The frozen ROS value is in
                # the detail below, where it informs without being arithmetic.
                event = self._arming.note_clock(
                    False,
                    mono,
                    f"the ROS clock is frozen at {self._clock_ref_ros_sec:.3f} "
                    f"and has not advanced for {frozen_for:.1f}s of "
                    f"monotonic time (use_sim_time={self._use_sim_time}); every "
                    "age this node measures against it is inert (SAFETY.md F-24)",
                )
            self._handle_disarm(event)
        except Exception as exc:  # noqa: BLE001 - a lost disarm is the failure
            self.get_logger().error(
                f"clock watchdog: the disarm itself raised ({exc!r}). It is NOT "
                "latched as done and will be RETRIED on the next tick "
                "(SAFETY.md F-32)."
            )
            return
        self._clock_disarm_done = True

    # ==================================================================
    # shutdown - see "KILLING THIS NODE DOES NOT STOP THE ROBOT" above
    # ==================================================================
    def request_shutdown(self, reason: str) -> None:
        """Ask the main loop to stop. Safe to call from a signal handler.

        Sets a flag and nothing else: no logging, no publishing, no lock. The
        work happens in :meth:`prepare_shutdown`, on the main thread, with the
        context and the executor still alive.
        """
        if not self._shutdown_reason:
            self._shutdown_reason = reason
        self._shutdown_event.set()

    def wait_for_shutdown_request(self, poll_sec: float = 0.2) -> str:
        """Block the main thread until a shutdown is requested.

        Polled rather than an indefinite wait so that a signal delivered to the
        main thread is acted on promptly even where the wait is not interrupted.
        """
        while not self._shutdown_event.is_set() and rclpy.ok():
            self._shutdown_event.wait(poll_sec)
        return self._shutdown_reason or "context shutdown"

    def _shutdown_log(self, severity: str, message: str) -> None:
        """Log during shutdown without ever being able to cost the cancel.

        SAFETY.md F-26. Two physical call sites per severity - one here, one in
        the `print` fallback - and never a raise out of either: `_log_at` fixes
        rclpy's severity pinning (F-22) but a logger that raises for any other
        reason would still propagate, and in `prepare_shutdown` propagating means
        the unconditional cancel never runs and the latch makes it unrepeatable.
        """
        try:
            self._log_at(severity, message)
        except Exception as exc:  # noqa: BLE001 - logging is what failed last time
            print(f"shutdown log failed ({exc!r}): {message}", file=sys.stderr)

    def _shutdown_now(self) -> float:
        """A timestamp for the shutdown bookkeeping, whatever the clock does.

        The fallback is reached when reading the ROS clock RAISES, i.e. when the
        context is already being torn down under us. It is not a fallback for a
        clock that has merely stopped - that case is `_clock_watchdog`'s, and it
        does not prevent a cancel.

        The premise this used to state - "use_sim_time is false everywhere this
        runs" - was FALSE: the twin config sets it true (SAFETY.md F-24). So the
        two sources are not interchangeable and this says what the value is used
        for instead: the disarm record and `cancel_requested_at_sec`. Under sim
        time a wall-clock value there is not comparable with the ROS-time stamps
        `_check_cancel_timeout` uses - it makes an elapsed time come out hugely
        negative, i.e. it can only ever suppress an escalation report during a
        shutdown that is already reporting at ERROR, never invent one. Losing the
        cancel would be worse than either, which is why the fallback exists at
        all.
        """
        try:
            return self._ros_now()
        except Exception as exc:  # noqa: BLE001 - the clock is not worth the cancel
            self._shutdown_log(
                "error",
                f"shutdown: the clock could not be read ({exc!r}); using the wall "
                "clock so the cancel below still happens. Under sim time this "
                "stamp is not comparable with ROS-time stamps - see the method.",
            )
            return time.time()

    def prepare_shutdown(self) -> None:
        """Disarm - and, from stage 3, CANCEL - while everything is still live.

        This is the whole point of the restructured `main` (SAFETY.md F-4). The
        previous structure let `rclpy.spin` raise `ExternalShutdownException` on
        `SIGTERM` and did the `NODE_SHUTDOWN` disarm inside `destroy_node`, by
        which time the context was already down and the executor stopped: the
        disarm was a log line and no cancel request could have completed. Here
        the executor is still spinning in its own thread, so a cancel can be
        sent and confirmed.

        The cancel happens here, immediately after the disarm, and the wait for
        its confirmation is the ONE bounded wait in this package: it runs on the
        main thread while the executor is still spinning in its own, so it is
        not a nested spin and it is not inside a callback. A cancel that is not
        confirmed logs ERROR and raises an ERROR diagnostic, and that is the
        entire escalation (SR-15 rule 9).

        And it still does not stop the robot on its own: if the cancel does not
        reach Nav2, only the upstream mechanisms remain (mux `cmd_timeout_sec`,
        the E-stop, firmware `CMD_TIMEOUT_MS`).
        """
        if self._shutdown_prepared:
            return
        self._shutdown_prepared = True
        # EVERYTHING BETWEEN THE LATCH AND THE CANCEL IS GUARDED (SAFETY.md
        # F-26). The latch makes this method run at most once per process, so
        # anything that can raise between it and the unconditional cancel below
        # costs that cancel permanently and unretryably - and the failure this
        # codebase has actually had, twice, is a LOGGER CALL RAISING (F-22).
        # F-17 guarded the disarm; the two log calls three lines above it were
        # still bare. `_shutdown_log` and `_shutdown_now` are those two lines
        # with the guard the rest of this method already has.
        self._shutdown_log(
            "info",
            f"shutting down ({self._shutdown_reason or 'unspecified'}); disarming. "
            "NOTE: ending this node does not stop the robot - it only stops new "
            "goals coming from it.",
        )
        now = self._shutdown_now()
        # STEP 2, GUARDED. `_handle_disarm` already cancels when the machine WAS
        # armed, and step 3 below is the unconditional belt-and-braces call that
        # covers a mission which outlived its arming window. Until SAFETY.md
        # F-17 those were one unguarded sequence: an exception anywhere in the
        # disarm - and BUG 2 raised in exactly that place - propagated out of
        # this method, so the braces never ran and the guard one level up could
        # only report that a goal in flight may not have been cancelled. The
        # try/except is what turns that report into an outcome: the disarm may
        # fail, the CANCEL STILL HAPPENS.
        try:
            with self._arming_lock:
                event = self._arming.shutdown(self._safety_now())
            self._handle_disarm(event)
        except Exception as exc:  # noqa: BLE001 - the cancel below is the point
            try:
                self.get_logger().error(
                    f"shutdown: the disarm failed with {exc!r}. Continuing to the "
                    "unconditional cancel anyway - a failed disarm must not cost a "
                    "goal in flight its cancel (SAFETY.md F-17)."
                )
            except Exception:  # noqa: BLE001 - logging is what failed last time
                print(f"shutdown disarm failed: {exc!r}", file=sys.stderr)
        with self._mission_lock:
            mission = self._mission
        if mission is None:
            self._shutdown_log("info", "nothing in flight; nothing to cancel")
            return
        self._cancel_mission("NODE_SHUTDOWN", now, error=True)
        if not mission.done.wait(self._cancel_confirm_timeout_sec):
            self._shutdown_log(
                "error",
                f"shutdown: target {mission.target_id} did not reach a terminal "
                f"state within {self._cancel_confirm_timeout_sec:.1f}s. The goal "
                "may still be executing in Nav2. ENDING THIS NODE DOES NOT STOP "
                "THE ROBOT - use the E-stop; the mux and firmware timeouts are "
                "what remain.",
            )
        else:
            self._shutdown_log(
                "info",
                f"shutdown: target {mission.target_id} reached a terminal state",
            )

    # ==================================================================
    # the preview pass
    # ==================================================================
    def _preview_tick(self) -> None:
        # The arming poll used to live here. It now runs in `_safety_tick`, in
        # its own callback group, so nothing below this line can delay it
        # (SAFETY.md F-2).
        if self.get_parameter("grasp.verify_path").value:
            self.get_logger().error(
                "grasp.verify_path is true but path verification is NOT "
                "implemented: the acceptance predicate the approach ring feeds "
                "is synchronous, so a ComputePathToPose round trip inside it "
                "would be a blocking wait inside a callback, which C-2 forbids. "
                "Treating it as false - a goal is NOT being path-checked.",
                throttle_duration_sec=60.0,
            )

        if not self._ingress:
            return
        resolution = self._resolution
        # Re-render on a changed world (new targets, moved datum, changed
        # parameter) OR on a new resolution. The second is what keeps the
        # preview and the dispatch decision the same object rather than two
        # evaluations that agree most of the time (C-5 / SAFETY.md F-5).
        if not self._preview_dirty and resolution is self._rendered_resolution:
            return
        self._preview_dirty = False
        self._rendered_resolution = resolution

        targets = self._latest_targets
        markers = MarkerArray()
        # DELETEALL first: after a datum burst the whole set moves, and stale
        # markers from the previous position would otherwise linger and the
        # preview would never look converged.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        results: List[Tuple[str, val.ValidationResult, float, float]] = []
        if targets is not None:
            for target in targets.targets:
                if target.collected:
                    continue
                goal = val.IncomingGoal(
                    target_id=target.id,
                    latitude_deg=target.latitude_deg,
                    longitude_deg=target.longitude_deg,
                    status=0,
                    stamp_sec=time.time(),
                    confidence=None if math.isnan(target.confidence) else target.confidence,
                )
                result = self._validate(goal)
                x = y = float("nan")
                if result.object_xy is not None:
                    x, y = result.object_xy
                results.append((target.id, result, x, y))

        # THE LIST IS PREVIEW MATERIAL ONLY. Not one of these entries is ever
        # flagged as the goal any more: the goal is whatever
        # `/octopus/trash_goal` says, and that is rendered below from the same
        # resolution the dispatch uses. Flagging a list entry by matching
        # `targets.goal_id` is precisely what let the previewed goal and the
        # dispatched goal come apart (SAFETY.md F-5).
        for index, (target_id, result, x, y) in enumerate(results):
            markers.markers.extend(
                self._target_markers(index, target_id, result, x, y, is_goal=False)
            )

        if resolution is not None:
            object_xy = resolution.result.object_xy
            gx, gy = object_xy if object_xy is not None else (float("nan"), float("nan"))
            label = resolution.target_id or f"<{resolution.correlation.status}>"
            markers.markers.extend(
                self._target_markers(
                    len(results), label, resolution.result, gx, gy, is_goal=True
                )
            )
            with self._mission_lock:
                mission = self._mission
            if mission is not None:
                # A goal in flight reports what it is DOING. The resolution
                # would report DUPLICATE here - correct, and useless to an
                # operator watching a robot drive somewhere.
                self._publish_mission_status(
                    mission,
                    ExternalGoalStatus.STATE_PICKING
                    if mission.state == NAV_PICKING
                    else ExternalGoalStatus.STATE_NAVIGATING,
                    "",
                )
            else:
                self._publish_goal_status(resolution.target_id, resolution.result)
            self._log_verdict(resolution.target_id or "<uncorrelated>", resolution.result)
        self._marker_pub.publish(markers)
        self._publish_diagnostics()

    def _validate(
        self, goal: val.IncomingGoal, ctx: Optional[val.ValidationContext] = None
    ) -> val.ValidationResult:
        """Validate one goal, refusing outright while the geofence is unmeasured.

        The geofence numbers are TO-VERIFY. Until they are set the gateway
        REFUSES to validate rather than falling back to some default area
        (FR-12 item 6) - an invented rectangle would make every verdict a guess.
        """
        if self._geofence_rect() is None:
            result = val.ValidationResult(
                verdict=val.VERDICT_REJECTED,
                reason=val.GEOFENCE_NOT_CONFIGURED,
                detail=(
                    "geofence.{min,max}_{x,y}_m are TO-VERIFY; refusing to validate "
                    "rather than defaulting to an area nobody measured"
                ),
                severity=val.SEVERITY_LOCAL,
            )
            # The object position is still resolvable and still worth previewing:
            # the operator needs to see WHERE the refused target is.
            datum = self._datum_tracker.datum
            if datum is not None:
                try:
                    result.object_xy = latlon_to_map(
                        datum, goal.latitude_deg, goal.longitude_deg
                    )
                except GeodesyError:
                    pass
            self._count(result)
            return result
        result = val.validate_goal(ctx if ctx is not None else self._make_context(goal))
        self._count(result)
        return result

    def _count(self, result: val.ValidationResult) -> None:
        if result.verdict == val.VERDICT_ACCEPTED:
            self._counters["accepted"] += 1
        elif result.verdict == val.VERDICT_PREVIEW:
            self._counters["preview"] += 1
        elif result.verdict == val.VERDICT_REJECTED:
            self._counters["rejected"] += 1
            self._last_reason = result.reason
            self._last_severity = result.severity

    def _log_verdict(self, target_id: str, result: val.ValidationResult) -> None:
        """Log once per changed verdict, not once per message.

        Their 1 Hz republication plus a datum burst would otherwise turn the log
        into a flood in which the one verdict that changed is invisible.
        """
        key = (target_id, result.reason or result.verdict)
        if key == self._last_logged_verdict:
            return
        self._last_logged_verdict = key
        if result.verdict == val.VERDICT_ACCEPTED:
            blocks = "; ".join(self._dispatch_blocks())
            self.get_logger().info(
                f"goal {target_id} VALID and previewed; not dispatched - {blocks}"
            )
        elif result.severity == val.SEVERITY_LOCAL:
            # Our-side failure: ERROR plus an ERROR diagnostic (SR-13 wants an
            # active signal, not an absence).
            self.get_logger().error(
                f"goal {target_id} rejected ({result.reason}): {result.detail}"
            )
        else:
            self.get_logger().warn(
                f"goal {target_id} rejected ({result.reason}): {result.detail}"
            )

    def _publish_goal_status(self, target_id: str, result: val.ValidationResult) -> None:
        msg = ExternalGoalStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_id = target_id
        if result.verdict == val.VERDICT_REJECTED:
            msg.state = ExternalGoalStatus.STATE_REJECTED
        elif target_id in self._blacklist:
            msg.state = ExternalGoalStatus.STATE_BLACKLISTED
        elif result.verdict == val.VERDICT_ACCEPTED:
            # Valid but deliberately not dispatched: that is PREVIEW, not
            # NAVIGATING. Reporting it as navigating would be a lie the operator
            # would act on.
            msg.state = ExternalGoalStatus.STATE_PREVIEW
        else:
            msg.state = ExternalGoalStatus.STATE_RECEIVED
        msg.reason = result.reason
        msg.detail = result.detail or "; ".join(self._dispatch_blocks())
        msg.attempts = int(self._attempts.get(target_id, 0))
        msg.acknowledged = False
        self._status_pub.publish(msg)

    # ==================================================================
    # RViz preview
    # ==================================================================
    def _target_markers(
        self,
        index: int,
        target_id: str,
        result: val.ValidationResult,
        x: float,
        y: float,
        is_goal: bool,
    ) -> List[Marker]:
        markers: List[Marker] = []
        if math.isnan(x) or math.isnan(y):
            return markers
        base = index * 10

        obj = Marker()
        obj.header.frame_id = self._map_frame
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.ns = "octopus/objects"
        obj.id = base
        obj.type = Marker.SPHERE
        obj.action = Marker.ADD
        obj.pose.position = Point(x=x, y=y, z=0.05)
        obj.pose.orientation.w = 1.0
        obj.scale.x = obj.scale.y = obj.scale.z = 0.20
        if is_goal and result.verdict == val.VERDICT_ACCEPTED:
            obj.color = _colour(0.1, 0.9, 0.2, 0.9)
        elif is_goal:
            obj.color = _colour(0.9, 0.2, 0.1, 0.9)
        else:
            obj.color = _colour(0.9, 0.75, 0.1, 0.6)
        markers.append(obj)

        label = Marker()
        label.header = obj.header
        label.ns = "octopus/labels"
        label.id = base + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=x, y=y, z=0.45)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color = _colour(1.0, 1.0, 1.0, 0.9)
        verdict = result.reason or result.verdict
        label.text = f"#{target_id} {verdict}" if is_goal else f"#{target_id}"
        markers.append(label)

        if result.robot_pose is not None:
            rx, ry, ryaw = result.robot_pose
            pose_marker = Marker()
            pose_marker.header = obj.header
            pose_marker.ns = "octopus/robot_goal"
            pose_marker.id = base + 2
            pose_marker.type = Marker.ARROW
            pose_marker.action = Marker.ADD
            pose_marker.pose.position = Point(x=rx, y=ry, z=0.05)
            qx, qy, qz, qw = _quaternion_from_yaw(ryaw)
            pose_marker.pose.orientation.x = qx
            pose_marker.pose.orientation.y = qy
            pose_marker.pose.orientation.z = qz
            pose_marker.pose.orientation.w = qw
            pose_marker.scale.x, pose_marker.scale.y, pose_marker.scale.z = 0.45, 0.08, 0.08
            pose_marker.color = _colour(0.2, 0.5, 1.0, 0.95)
            markers.append(pose_marker)

            # The approach arrow: from the resolved standing pose to the object.
            # This is the line the fixed, unaimable pick sequence has to close
            # along, so it is the thing worth looking at before arming.
            approach = Marker()
            approach.header = obj.header
            approach.ns = "octopus/approach"
            approach.id = base + 3
            approach.type = Marker.ARROW
            approach.action = Marker.ADD
            approach.points = [Point(x=rx, y=ry, z=0.05), Point(x=x, y=y, z=0.05)]
            approach.scale.x, approach.scale.y, approach.scale.z = 0.03, 0.09, 0.09
            approach.color = _colour(0.6, 0.9, 1.0, 0.9)
            approach.pose.orientation.w = 1.0
            markers.append(approach)
        return markers

    # ==================================================================
    # telemetry + diagnostics
    # ==================================================================
    def _telemetry_tick(self) -> None:
        msg = RobotTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.device_id = "gripperx"

        pose, age, err = self._robot_pose()
        if pose is None:
            msg.pose_valid = False
            msg.pose_reason = "TF_UNAVAILABLE"
            msg.map_x = msg.map_y = msg.yaw_deg = float("nan")
            msg.pose_age_sec = -1.0
        elif age is not None and age > float(self.get_parameter("max_tf_age_sec").value):
            msg.pose_valid = False
            msg.pose_reason = "TF_STALE"
            msg.map_x = msg.map_y = msg.yaw_deg = float("nan")
            msg.pose_age_sec = float(age)
        else:
            msg.pose_valid = True
            msg.map_x, msg.map_y = pose[0], pose[1]
            msg.yaw_deg = math.degrees(pose[2])
            msg.pose_age_sec = float(age or 0.0)

        # The lat/lon is flagged separately: the map pose can be perfectly known
        # while the datum is missing or still their bootstrap fallback, and a
        # lat/lon derived from that would be a fabricated position on their map.
        blocker = self._datum_tracker.dispatch_blocker()
        if not msg.pose_valid:
            msg.latlon_valid = False
            msg.latlon_reason = msg.pose_reason
            msg.latitude_deg = msg.longitude_deg = float("nan")
        elif blocker:
            msg.latlon_valid = False
            msg.latlon_reason = blocker
            msg.latitude_deg = msg.longitude_deg = float("nan")
        else:
            datum = self._datum_tracker.datum
            assert datum is not None
            lat, lon = map_to_latlon(datum, msg.map_x, msg.map_y)
            msg.latlon_valid = True
            msg.latitude_deg, msg.longitude_deg = lat, lon

        if self._odom is None:
            msg.odom_valid = False
            msg.odom_reason = "NO_ODOM"
            msg.speed_mps = float("nan")
            msg.odom_age_sec = -1.0
        else:
            twist = self._odom.twist.twist.linear
            msg.odom_valid = True
            msg.speed_mps = math.hypot(twist.x, twist.y)
            msg.odom_age_sec = float(
                max(0.0, self._ros_now() - (self._odom_stamp_sec or self._ros_now()))
            )

        # "unavailable" and "idle" are different statements and are kept apart:
        # idle means a client that is simply not busy, and reporting that while
        # `navigate_to_pose` does not exist would be a fabricated availability
        # (FR-12 item 8).
        with self._mission_lock:
            mission = self._mission
        if not self._nav2_available:
            msg.nav_state = NAV_UNAVAILABLE
            msg.nav_state_reason = "NAVIGATE_TO_POSE_NOT_AVAILABLE"
        elif mission is None:
            msg.nav_state = NAV_IDLE
            msg.nav_state_reason = ""
        else:
            msg.nav_state = mission.state
            msg.nav_state_reason = mission.cancel_reason
        msg.active_goal_id = mission.target_id if mission is not None else ""

        with self._arming_lock:
            snap = self._arming.snapshot(self._safety_now())
        msg.armed = bool(snap["armed"])
        msg.arming_seconds_remaining = float(snap["seconds_remaining"])
        msg.last_disarm_trigger = str(snap["last_disarm_trigger"])
        msg.teleop_mode = self._teleop_mode
        msg.teleop_mode_age_sec = (
            -1.0
            if self._teleop_mode_mono_sec is None
            else float(max(0.0, self._safety_now() - self._teleop_mode_mono_sec))
        )

        msg.link_ok = bool(self._link_connected)
        msg.link_last_message_age_sec = float(self._link_age_sec)
        msg.link_reconnects = int(self._link_reconnects)

        msg.goals_received = int(self._counters["received"])
        msg.goals_accepted = int(self._counters["accepted"])
        msg.goals_rejected = int(self._counters["rejected"])
        msg.goals_preview = int(self._counters["preview"])
        msg.last_reject_reason = self._last_reason
        msg.blacklisted_ids = list(self._blacklist)

        # There is no battery measurement chain on this robot at all. Reported
        # unavailable with a reason and a null percent, permanently, until
        # HWR-21 provides one. Never a fabricated number (FR-12 item 8).
        msg.battery_status = "unavailable"
        msg.battery_reason = "NO_SENSOR_INSTALLED"
        msg.battery_percent_valid = False
        msg.battery_percent = float("nan")

        self._telemetry_pub.publish(msg)

    def _dispatch_diagnostic(self):
        """The dispatch path as one status. See `diagnostics.dispatch_status`."""
        with self._mission_lock:
            mission = self._mission
            cancel_pending = mission is not None and mission.cancelling and not mission.cancel_confirmed
            cancel_failed = self._cancel_failed_sticky or (
                mission is not None and mission.cancel_error_logged
            )
            target_id = mission.target_id if mission is not None else ""
        return diag.dispatch_status(
            nav_state=self._nav_state,
            target_id=target_id,
            correlation=self._last_correlation,
            attempts=int(self._attempts.get(target_id, 0)) if target_id else 0,
            nav2_available=self._nav2_available,
            cancel_pending=bool(cancel_pending),
            cancel_failed=bool(cancel_failed),
            ack_suppressed_reason=self._ack_suppressed_reason,
            acknowledged=len(self._acknowledged),
            extra={
                "auto_pick": self._auto_pick,
                "pick_server": self._pick_available,
                "cancel_failures": self._cancel_failures,
                "acknowledged_ids": ", ".join(self._acknowledged) or None,
                # Reported separately from `acknowledged`, and deliberately so:
                # `reached` can exceed `acknowledged` for ever without anything
                # being wrong with us, and that gap is the honest picture of a
                # protocol with no failure channel (C-7, FR-12 item 7).
                "reached": len(self._reached),
                "reached_ids": ", ".join(self._reached) or None,
                "last_reached": self._last_reached_detail or None,
                # The correlation of the goal IN FLIGHT, which is a different
                # question from `correlation` above (the goal that would be
                # dispatched next). SAFETY.md F-13.
                "mission_correlation": self._mission_correlation or None,
                "correlation_cancels": self._correlation_cancels,
                "id_space_resets": self._id_space_resets,
                # SAFETY.md F-28: diagnosable, not only enforced. `link_age` on
                # the link status is the age of the last frame of ANY topic, so
                # it says nothing about this one going silent.
                "target_list_age_sec": (
                    None if self._target_list_age_sec() is None
                    else round(self._target_list_age_sec(), 1)
                ),
                "max_target_list_age_sec": self._max_target_age_sec,
            },
        )

    def _publish_diagnostics(self) -> None:
        stamp = self.get_clock().now().to_msg()
        with self._arming_lock:
            snap = self._arming.snapshot(self._safety_now())
        self._diag_pub.publish(
            diag.array(
                stamp,
                [
                    diag.arming_status(
                        armed=bool(snap["armed"]),
                        seconds_remaining=float(snap["seconds_remaining"]),
                        allow_arm=self._allow_arm,
                        dry_run=self._dry_run,
                        last_disarm_trigger=str(snap["last_disarm_trigger"]),
                        disarm_error=self._last_disarm_error,
                        extra={
                            "dispatch_blocks": "; ".join(self._dispatch_blocks()),
                            "auto_pick": self._auto_pick,
                            "teleop_mode": self._teleop_mode or None,
                            # SAFETY.md F-24: the state of the clock the window
                            # above is measured on, reported next to it.
                            "use_sim_time": self._use_sim_time,
                            "clock_advancing": (
                                self._clock_proven and not self._clock_stalled
                            ),
                        },
                    ),
                    diag.goal_status(
                        last_reason=self._last_reason,
                        last_severity=self._last_severity,
                        received=self._counters["received"],
                        accepted=self._counters["accepted"],
                        rejected=self._counters["rejected"],
                        preview=self._counters["preview"],
                        blacklisted=self._blacklist,
                        extra={"datum_bursts": self._datum_burst_count},
                    ),
                    self._dispatch_diagnostic(),
                    # SAFETY.md F-24. Note the stamp above is the ROS clock, so
                    # when this status says the clock is frozen the stamp is
                    # frozen with it - which is itself the signal, and is why
                    # `frozen_for_sec` is monotonic and not a stamp difference.
                    diag.clock_status(
                        proven=self._clock_proven,
                        stalled=self._clock_stalled,
                        frozen_for_sec=max(0.0, time.monotonic() - self._clock_ref_mono),
                        use_sim_time=self._use_sim_time,
                        stall_sec=self._clock_stall_sec,
                        # SAFETY.md F-31: WARN while an unproven clock is still
                        # inside the measured discovery grace, ERROR after it.
                        # The gate is identical in both - see `_on_set_arming`.
                        startup_grace=self._clock_in_startup_grace(),
                        # SAFETY.md F-40, user decision 2026-08-20. The durable
                        # half of the report; the WARN is the other half.
                        clock_publishers=self._clock_publisher_seen,
                        forward_jumps=self._clock_forward_jumps,
                        last_forward_jump_sec=self._last_forward_jump_sec,
                        forward_jump_sec=self._clock_forward_jump_sec,
                        # SAFETY.md F-37. Reported because it is the number
                        # `ArmingState.expires_at` is now projected with: a
                        # consumer that finds that field surprising can see what
                        # produced it. Advisory - no gate reads it.
                        extra={
                            "ros_clock_rate": round(self._clock_rate.rate, 4),
                            "ros_clock_rate_samples": self._clock_rate.samples,
                        },
                    ),
                    diag.config_status(self._unset_items()),
                ],
            )
        )

    # ==================================================================
    def destroy_node(self) -> bool:
        """Backstop only. The real shutdown path is :meth:`prepare_shutdown`.

        By the time `destroy_node` runs the executor is down and the context may
        already be gone, so a cancel could not complete here and a publish may
        fail - which is precisely what made the old structure unable to cancel
        (SAFETY.md F-4). It still runs, because a disarm that is only logged is
        better than no record at all, but `prepare_shutdown` is what stage 3
        must rely on.
        """
        if not self._shutdown_prepared:
            try:
                with self._arming_lock:
                    event = self._arming.shutdown(self._safety_now())
                self._handle_disarm(event)
            except Exception as exc:  # noqa: BLE001 - teardown must not raise
                self.get_logger().error(f"disarm during teardown failed: {exc!r}")
        return super().destroy_node()


def _install_shutdown_handlers(node: GoalGatewayNode) -> None:
    """Own SIGINT/SIGTERM handlers, replacing rclpy's.

    rclpy's default handlers shut the context down from under the executor, so
    by the time anything of ours runs there is no live context left to cancel a
    Nav2 goal with - that is SAFETY.md F-4. Ours only set a flag; the main
    thread then runs `prepare_shutdown` while the executor is still spinning,
    and shuts the context down afterwards.

    Registration can only happen on the main thread; a failure to register is
    reported rather than swallowed, because the fallback (rclpy's handler, which
    cannot cancel) is exactly the behaviour this replaces.
    """
    def _request(signum, _frame):
        node.request_shutdown(f"signal {signal.Signals(signum).name}")

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _request)
        except (ValueError, OSError) as exc:  # not the main thread / unsupported
            node.get_logger().error(
                f"could not install a {sig.name} handler ({exc!r}); shutdown will "
                "fall back to rclpy's, which cannot cancel an in-flight goal"
            )


def _spin(executor: MultiThreadedExecutor, node: GoalGatewayNode) -> None:
    try:
        executor.spin()
    except ExternalShutdownException:
        node.request_shutdown("external shutdown")
    except Exception as exc:  # noqa: BLE001 - the main thread must learn of it
        node.get_logger().error(f"executor stopped with {exc!r}")
        node.request_shutdown(f"executor error: {exc!r}")
    finally:
        # However the executor ended, the main thread must stop waiting.
        node.request_shutdown("executor returned")


def _prepare_shutdown_guarded(node: GoalGatewayNode) -> None:
    """Run the shutdown path, and never let it be skipped by its own failure.

    `prepare_shutdown` is the only place a cancel can still reach Nav2, so an
    exception raised inside it is the worst possible moment for one: the process
    ends with a traceback and the goal it was supposed to cancel keeps running.
    That is not hypothetical - it happened here, when a logging call raised and
    the shutdown path died on it while a goal was in flight.

    The exception is reported rather than swallowed, and it is reported with the
    consequence spelled out, because "ending this node does not stop the robot"
    is exactly the thing a reader must not have to infer (SAFETY.md §1.1).
    """
    try:
        node.prepare_shutdown()
    except Exception as exc:  # noqa: BLE001 - the last chance to cancel must not vanish
        try:
            node.get_logger().error(
                f"the shutdown path itself failed with {exc!r}. A goal in flight "
                "may NOT have been cancelled. Ending this node does not stop the "
                "robot - use the E-stop."
            )
        except Exception:  # noqa: BLE001 - logging is what failed last time
            print(f"prepare_shutdown failed: {exc!r}", file=sys.stderr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None
    code = 0
    try:
        node = GoalGatewayNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        _install_shutdown_handlers(node)
        spin_thread = threading.Thread(target=_spin, args=(executor, node), name="gateway_spin")
        spin_thread.start()
        # The main thread does nothing but wait for a stop request, so that when
        # one comes the executor is still running and a cancel can complete.
        node.wait_for_shutdown_request()
        _prepare_shutdown_guarded(node)
    except KeyboardInterrupt:
        # Only reachable before the handlers are installed; afterwards a
        # Ctrl-C is a flag, not an exception.
        if node is not None:
            node.request_shutdown("KeyboardInterrupt")
            _prepare_shutdown_guarded(node)
    except ExternalShutdownException:
        # Somebody shut the context down out from under us. Nothing can be
        # cancelled from here - the node's docstring says what that means.
        if node is not None:
            node.get_logger().error(
                "context was shut down externally; no cancel could be sent. "
                "Ending this node does not stop the robot."
            )
    except SystemExit as exc:
        # The SR-8 domain guard exits this way. The code MUST survive: a launch
        # file, a supervisor or a systemd unit has to be able to tell a refused
        # start from a clean shutdown, and swallowing it would make the guard
        # look like a successful run that simply did nothing.
        code = int(exc.code) if isinstance(exc.code, int) else 1
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=5.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == "__main__":
    main()
