#!/usr/bin/env python3
"""rclpy wrapper around :mod:`rosbridge_client`. TRANSPORT ONLY - no policy.

This node is the single place in the workspace that speaks the external wire
format (FR-12 item 2). It converts the Octopus's JSON into typed
``gripperx_external_msgs`` and back, and it makes **no decisions**: it does not
validate a goal, it does not resolve a pose, it does not arm anything and it
cannot dispatch anything. All of that lives in ``goal_gateway_node``. Swapping
rosbridge for another transport means replacing this file and
``rosbridge_client.py`` - nothing else.

WHAT IT SUBSCRIBES ON THE EXTERNAL SIDE
=======================================
====================================== =========================== ==============
``/octopus/fake_eve_gps_start``        ``sensor_msgs/NavSatFix``   always
``/octopus/trash_goal``                ``sensor_msgs/NavSatFix``   ingress only
``/octopus/trash_gps``                 ``std_msgs/String`` (JSON)  ingress only
``/octopus/flight_camera_transform/status``
                                       ``std_msgs/String`` (JSON)  always
====================================== =========================== ==============

The datum is subscribed even at rollout stage 1 (telemetry only), because the
telemetry reports our pose as lat/lon *as well as* map metres and that
conversion needs the shared datum. The two goal topics are subscribed only when
``goal_ingress_enabled`` is true, so at stage 1 no goal can enter the process at
all - the strongest form of "no goal ingress" available.

THE TRANSFORM STATUS IS SUBSCRIBED ALWAYS, AND THAT IS A DECISION
=================================================================
``goal_ingress_enabled`` gates goal ingress, and the property it buys is "no
external goal can enter this process". The transform status carries no goal: no
position, no id, nothing that can be correlated to a target, and nothing
downstream of it dispatches, arms or validates anything. Putting it behind that
flag would not strengthen the property - the datum subscription already means
this node parses peer-controlled bytes at stage 1 - and it would cost the one
case the topic exists for. The event it detects is a re-lock of their map
frame, which happens when their transform node restarts; the alignment it
invalidates is established by the disarmed verification run, which happens at
stage 1 with goal ingress OFF. A flag that blinds us exactly then is the wrong
flag. It has its own switch (``transform_status_enabled``) so it can be turned
off without touching goal-ingress semantics.

REPORT ONLY. A re-lock is counted, logged loudly, put on ``/diagnostics`` and
mirrored into the outbound telemetry. It cancels nothing, disarms nothing and
blocks nothing - what it *should* cause is an undecided user question, and
SAFETY.md F-40 is the pattern for that: evidence first, decision after.

WHAT IT PUBLISHES ON THE EXTERNAL SIDE
======================================
Exactly one topic: ``/octopus/devices/gripperx/status`` (proposal item 1),
built from the typed ``RobotTelemetry`` the gateway hands over. Telemetry is
**outbound only** and carries no control semantics; nothing the Octopus sends
back may change our state (SR-15 rule 3).

``/octopus/trash_goal_done`` (rollout stage 3, ingress only). It means
*collected* to the Octopus, it is the only thing that advances their mission,
and their protocol has no way to take it back. THIS NODE DOES NOT DECIDE IT: it
forwards whatever id arrives on the local ``goal_done`` topic, exactly as it
arrives, and the decision - a successful pick, a unique correlation, an armed
gate - belongs entirely to ``goal_gateway_node`` (SAFETY.md C-7). Keeping the
decision out of the transport is the same rule as everywhere else in this file;
what is different is only that here the consequence of getting it wrong is
permanent.

THREADING
=========
:meth:`RosbridgeClient.run` owns an asyncio loop in a private thread. Its
callbacks fire on that thread, so they only ever drop a payload into a
lock-protected slot; a ROS timer on the executor thread drains the slots and
does the publishing. Latest-wins in those slots is not laziness, it is the
burst policy: after a datum drag the counterpart republishes its entire target
set back-to-back (their threshold is 1e-9 deg), and a slot that keeps only the
newest state converges instead of growing a queue (FR-12 item 5).
"""

from __future__ import annotations

import math
import threading
import time
from collections import Counter
from typing import Any, Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
# /clock, for the SAFETY.md F-35 detector only. This does NOT make the node
# sim-time driven: rclpy's TimeSource attaches on the use_sim_time PARAMETER.
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String

from diagnostic_msgs.msg import DiagnosticArray
from gripperx_external_msgs.msg import (
    ExternalGoal,
    ExternalLinkStatus,
    ExternalTarget,
    ExternalTargetList,
    GeodeticDatum,
    RobotTelemetry,
)

from . import diagnostics as diag
from . import octopus_protocol as proto
from .domain_guard import (
    SIMULATION_DOMAIN_IDS,
    clock_publisher_warning,
    effective_domain_id,
    enforce_domain,
    is_simulation_domain,
)
from .geodesy import BOOTSTRAP_MATCH_TOLERANCE_DEG, Datum, is_bootstrap_fallback
from .rosbridge_client import RosbridgeClient

# ---------------------------------------------------------------------------
# SR-9 / SR-15 rules 6, 8 and 9: this package must never write the motion chain
# ---------------------------------------------------------------------------
#: Topics no node of this package may ever publish on, under any condition,
#: including failure conditions. There is no parameter, flag or operator-enabled
#: mode that lifts this - SR-15 rule 9 is a positive prohibition, not a default.
#: Enforced at runtime by :func:`guarded_publisher` rather than left to review,
#: because "nobody would add that" is exactly how a second writer appears.
FORBIDDEN_PUBLISH_TOPICS = frozenset(
    {
        "/teleop/set_mode",
        # The mux's OWN OUTPUT. `set_mode` is the request, `active_mode` is the
        # answer, and the answer belongs to the mux alone: a second writer would
        # let this package tell itself - and every other consumer, including the
        # MODE_CHANGE auto-disarm trigger that reads it - that the mode is
        # `autonomous` when it is not (SR-9 / OP-19). Subscribing stays right and
        # is what both nodes do; this list is publisher-only.
        #
        # It is here because of the SAFETY.md 6.3 ruling on
        # `escalate_to_keyboard_mode`: the rule is about the MECHANISM, and a
        # `String("keyboard")` on a mode topic is that mechanism under another
        # name, which a grep for the parameter's name would never see.
        "/teleop/active_mode",
        "/teleop/direct_steer",
        "/cmd_vel",
        "/nav/cmd_vel_raw",
        "/teleop/autonomous/cmd_vel",
        "/teleop/keyboard/cmd_vel",
        "/hw/joint_commands",
        "/wheel_velocity_controller/commands",
        "/steering_position_controller/commands",
        # Not a chain topic, but the other way into Nav2. FR-12 item 3: goals
        # enter through the action interface only. (bt_navigator has no
        # /goal_pose subscriber in this build anyway, so this only stops us
        # from pretending otherwise.)
        "/goal_pose",
    }
)


class ForbiddenTopic(RuntimeError):
    """Raised when something tries to publish on a motion-chain topic."""


def guarded_publisher(node: Node, msg_type, topic: str, qos):
    """``create_publisher`` that refuses the motion chain (SR-9 / SR-15)."""
    resolved = topic if topic.startswith("/") else f"{node.get_namespace().rstrip('/')}/{topic}"
    if topic in FORBIDDEN_PUBLISH_TOPICS or resolved in FORBIDDEN_PUBLISH_TOPICS:
        raise ForbiddenTopic(
            f"refusing to create a publisher on {topic!r}: the external path has "
            "no write access to the motion command chain at all (SR-15 rule 9). "
            "This is not a configurable behaviour."
        )
    return node.create_publisher(msg_type, topic, qos)


def assert_no_chain_publishers(node: Node) -> None:
    """Post-construction sweep over what the node ACTUALLY publishes.

    ``guarded_publisher`` inspects the topic string it was handed. rclpy applies
    remap rules *inside* ``create_publisher``, so a ``-r`` argument or a launch
    ``remappings=`` entry can place a publisher of this package on a motion-chain
    topic without the pre-check ever seeing that name - and nothing prevents
    future code from calling ``create_publisher`` directly and bypassing the
    guard altogether (SAFETY.md F-7).

    This walks the node's own publishers and reads ``topic_name``, which is the
    fully resolved, post-remap name the middleware uses. It is therefore the
    check that actually corresponds to what SR-15 rule 9 asserts, and the one
    that matches the SR-9 ``ros2 topic info -v`` baseline diff.

    Raises :class:`ForbiddenTopic`. Deliberately not a warning and not a
    parameter: the prohibition is structural, so a violation must end the
    process rather than run in a state the requirement forbids.
    """
    offenders = sorted(
        pub.topic_name
        for pub in node.publishers
        if pub.topic_name in FORBIDDEN_PUBLISH_TOPICS
    )
    if offenders:
        raise ForbiddenTopic(
            f"{node.get_name()} ended up publishing on the motion command chain: "
            f"{', '.join(offenders)}. The external path has no write access to "
            "the chain at all (SR-15 rule 9, SR-9). Check remappings - this "
            "sweep sees the resolved topic names, the create-time guard does "
            "not. This is not a configurable behaviour."
        )


def assert_no_command_clients(node: Node, allowed_actions: Sequence[Tuple[str, object]] = ()) -> None:
    """Post-construction sweep over what the node can CALL. SAFETY.md 6.3.

    The publisher sweep covers one half of "this package never commands the
    motion chain": the half that goes out on a topic. The other half is a
    *client* - a service client that flips the mux's mode, or an action client
    for something nobody reviewed. Nothing covered that until this sweep, and
    the acceptance rule that was supposed to cover it was a grep for the string
    ``escalate_to_keyboard_mode``, which passes for the same mechanism written
    under any other name (SAFETY.md 6.3, ruling of revision 3).

    So, mechanically:

    * **zero service clients.** The package creates none today - that is a
      verified property, not an assumption, and it is worth pinning: every
      service this package has any business with, it *serves* (``set_arming``).
      A client is how a node reaches out and changes somebody else's state.
    * **exactly the action clients that were passed in.** Stage 3 legitimately
      drives ``NavigateToPose`` and ``PickPlastic``; both are named by the
      caller, with the resolved name and the type, and anything else - a second
      client on the same action, a client on a different action - is refused.
      Counted, not tested for membership, which is what makes the "second
      client on the same action" half of that sentence true (SAFETY.md F-33).

    ``allowed_actions`` is ``(resolved_action_name, action_type)`` pairs. An
    empty tuple means "this node must have no clients of any kind at all",
    which is the link node's case.

    Raises :class:`ForbiddenTopic`, deliberately, and deliberately not a
    warning: like the publisher sweep this is structural, so a violation ends
    the process rather than running in a state the requirement forbids.
    """
    service_clients = sorted(client.srv_name for client in node.clients)
    if service_clients:
        raise ForbiddenTopic(
            f"{node.get_name()} created service client(s) on {', '.join(service_clients)}. "
            "This package creates none: it observes the mode, it never asks "
            "anybody to change it, and a service client is how that rule would "
            "be broken without a publisher ever appearing (SR-15 rule 9, "
            "SAFETY.md 6.3). This is not a configurable behaviour."
        )

    # MULTISET, not a set (SAFETY.md F-33). `allowed` used to be a set and every
    # client found was tested for MEMBERSHIP, so two clients on the same action
    # both passed while the docstring above said a second one is refused. Two
    # goal senders on one action is the second-writer shape SR-9 exists for, and
    # it is the shape a well-meaning refactor produces (a client created in a
    # helper AND in the constructor), so the check has to count. Listing the
    # same pair twice in `allowed_actions` still permits two - deliberately: it
    # then has to be argued for at the construction site, which is the point.
    allowed_counts = Counter((str(name), type_) for name, type_ in allowed_actions)
    found_counts: Counter = Counter()
    found = []
    for waitable in node.waitables:
        action_name = getattr(waitable, "_action_name", None)
        action_type = getattr(waitable, "_action_type", None)
        if action_name is None or action_type is None:
            # Not an action client - event handlers live here too.
            continue
        entry = (str(action_name), action_type)
        found_counts[entry] += 1
        found.append(f"{action_name} ({getattr(action_type, '__name__', action_type)})")
    offenders = [
        (entry, count, allowed_counts.get(entry, 0))
        for entry, count in sorted(found_counts.items(), key=lambda item: item[0][0])
        if count > allowed_counts.get(entry, 0)
    ]
    if offenders:
        listing = ", ".join(
            f"{name} ({getattr(type_, '__name__', type_)}) x{count}, permitted x{permitted}"
            for (name, type_), count, permitted in offenders
        )
        expected = ", ".join(
            f"{name} ({getattr(type_, '__name__', type_)}) x{count}"
            for (name, type_), count in sorted(
                allowed_counts.items(), key=lambda item: item[0][0]
            )
        ) or "none"
        raise ForbiddenTopic(
            f"{node.get_name()} ended up with unexpected action client(s): {listing}. "
            f"Expected exactly: {expected}. Every way this node can command "
            "anything is enumerated at its construction site; a client that is "
            "not in that list - INCLUDING a second client on an action that is "
            "in it - has not been reviewed (SAFETY.md 6.3, F-33)."
        )
    node.get_logger().info(
        "command-client sweep: 0 service clients, "
        + (f"{len(found)} action client(s) - " + "; ".join(sorted(found)) if found
           else "0 action clients")
        + ". These are every path by which this node can ask anything to act "
        "(SAFETY.md 6.3)."
    )


def _latched(depth: int = 1) -> QoSProfile:
    """TRANSIENT_LOCAL, so a node started later still sees the current value."""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _reliable(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _deg(radians: Optional[float]) -> str:
    """Radians -> a printable degree string, or ``"unavailable"``.

    For log lines only. ``None`` never becomes a number: an align angle that
    could not be derived must not read as 0 deg, which is the one value that
    would say "their frame and ours agree" (FR-12 item 8).
    """
    if radians is None:
        return "unavailable"
    return f"{math.degrees(radians):.2f} deg"


def _f(value: Optional[float]) -> float:
    """``None`` -> NaN. A ROS float field cannot be null, and 0.0 would be a
    fabricated measurement (FR-12 item 8)."""
    return float("nan") if value is None else float(value)


class OctopusLinkNode(Node):
    def __init__(self) -> None:
        super().__init__("octopus_link_node")

        # --- parameters -------------------------------------------------
        # Declared first and used for nothing until the domain guard has run.
        self.declare_parameter("expected_domain_id", -1)
        self.declare_parameter("url", "ws://127.0.0.1:9090")
        self.declare_parameter("goal_ingress_enabled", False)
        self.declare_parameter("link_lost_sec", 5.0)
        self.declare_parameter("drain_rate_hz", 5.0)
        self.declare_parameter("status_rate_hz", 1.0)
        self.declare_parameter("max_msg_bytes", 8192)
        self.declare_parameter("ws_max_size", 65536)
        self.declare_parameter("ping_interval_sec", 5.0)
        self.declare_parameter("ping_timeout_sec", 5.0)
        self.declare_parameter("backoff_initial_sec", 1.0)
        self.declare_parameter("backoff_max_sec", 30.0)
        self.declare_parameter("publish_telemetry", True)
        self.declare_parameter("telemetry_debug_json", True)
        self.declare_parameter("source_name", "octopus")
        # The fifth ingress topic, 2026-08-21. Its own switch rather than a
        # reuse of goal_ingress_enabled - see the module docstring.
        self.declare_parameter("transform_status_enabled", True)
        self.declare_parameter(
            "transform_relock_epsilon_rad", proto.DEFAULT_RELOCK_EPSILON_RAD
        )
        # Age past which the transform status counts as stale. Defaults to
        # link_lost_sec because it is the same kind of question about the same
        # link and their cadence is the same 1 Hz; declared separately so it can
        # be moved without moving the link watchdog's tolerance.
        self.declare_parameter("transform_stale_sec", 0.0)

        # --- SR-8, BEFORE anything is created ---------------------------
        # No publisher, no subscription, no client, no timer exists yet. A
        # mismatch exits; there is no degraded mode in which running on the
        # wrong domain is acceptable.
        self._expected_domain = int(self.get_parameter("expected_domain_id").value)
        enforce_domain(self, self._expected_domain)

        # SAFETY.md F-24, the same rule as the gateway's and for the same reason.
        # Both timers in this node - the inbound drain and the `link_status`
        # publication the gateway's watchdog measures silence against - now run on
        # a STEADY clock (SAFETY.md F-29, see the timer block below), so a stopped
        # `/clock` no longer stops them. The refusal below stays anyway, and not
        # merely out of caution: `use_sim_time: true` on a domain with no `/clock`
        # is a mis-launched stack whatever this node survives, every message it
        # stamps would carry a clock pinned at zero, and refusing removes a goal
        # SOURCE and nothing else (SR-15 rule 11). This node needs no clock
        # WATCHDOG of its own: its silence IS the signal the gateway's C-3
        # watchdog exists to read, and a gateway whose own clock stopped catches
        # that itself.
        if bool(self.get_parameter("use_sim_time").value) and not is_simulation_domain():
            self.get_logger().fatal(
                f"use_sim_time is true on ROS_DOMAIN_ID={effective_domain_id()}, which "
                f"is not a known simulation domain ({sorted(SIMULATION_DOMAIN_IDS)}). "
                "There is no /clock there, so every message this node stamps would "
                "carry a clock that never advances (SAFETY.md F-24). "
                "Refusing to start. Launch with use_sim_time:=false."
            )
            raise SystemExit(2)

        # THE MIRROR, SAFETY.md F-35, user decision 2026-08-20. Both nodes carry
        # the refusal above, so both carry its reverse: a check that exists in
        # one node and not the other is the drift `domain_guard` exists to
        # prevent. WARNs, never refuses - the reasoning is in
        # `clock_publisher_warning`. Nothing here gates anything.
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self._clock_mismatch_warned = False
        self._warn_if_clock_publisher("at startup")
        if not self._use_sim_time:
            # Second firing point: at startup DDS may not have matched a
            # publisher that is already running, so a silent start is not
            # evidence of absence. A DETECTOR, not a clock source.
            self.create_subscription(
                ClockMsg,
                "/clock",
                lambda _msg: self._warn_if_clock_publisher(
                    "on the first /clock message"
                ),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        self._url = str(self.get_parameter("url").value)
        self._ingress = bool(self.get_parameter("goal_ingress_enabled").value)
        self._link_lost_sec = float(self.get_parameter("link_lost_sec").value)
        self._source = str(self.get_parameter("source_name").value)
        self._publish_telemetry = bool(self.get_parameter("publish_telemetry").value)
        self._telemetry_debug = bool(self.get_parameter("telemetry_debug_json").value)
        self._transform_enabled = bool(self.get_parameter("transform_status_enabled").value)
        transform_stale = float(self.get_parameter("transform_stale_sec").value)
        self._transform_stale_sec = (
            self._link_lost_sec if transform_stale <= 0.0 else transform_stale
        )
        self._transform_tracker = proto.TransformLockTracker(
            float(self.get_parameter("transform_relock_epsilon_rad").value)
        )

        # SAFETY.md F-8, applied here as well: EVERY parameter of this node is
        # read once, at construction - the URL and the socket limits go into the
        # client, the rates into the timers, the rest into attributes - and none
        # of them is looked at again. Without this callback a `ros2 param set`
        # was accepted and silently did nothing, including on `link_lost_sec`,
        # the timeout of a mechanism whose whole job is to notice an absence. A
        # refusal with a reason is the only honest answer this node can give.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # --- inbound slots, written on the asyncio thread ----------------
        self._lock = threading.Lock()
        self._pending_datum: Optional[Dict[str, Any]] = None
        self._pending_goal: Optional[Dict[str, Any]] = None
        self._pending_gps: Optional[Any] = None
        self._pending_transform: Optional[Any] = None
        #: Monotonic time at which the last transform status was UNDERSTOOD -
        #: set on a successful parse, not on receipt. A stream of payloads we
        #: cannot parse must age out rather than read as fresh, because the
        #: values reported beside this age would then be the last good ones and
        #: would look current. Monotonic like every other age in this node, so it
        #: does not slow down with the simulation (SAFETY.md F-29). It carries
        #: the drain interval as a lag (<= 1/drain_rate_hz), which is far below
        #: the tolerance it is compared against.
        self._transform_rx_monotonic: Optional[float] = None
        self._parse_errors = 0
        self._last_parse_error = ""
        self._goals_seen = 0

        # --- publishers ---------------------------------------------------
        self._datum_pub = guarded_publisher(self, GeodeticDatum, "datum", _latched())
        self._link_pub = guarded_publisher(
            self, ExternalLinkStatus, "link_status", _latched()
        )
        self._diag_pub = guarded_publisher(self, DiagnosticArray, "/diagnostics", _reliable(10))
        self._goal_pub = None
        self._targets_pub = None
        if self._ingress:
            self._goal_pub = guarded_publisher(self, ExternalGoal, "goal", _reliable())
            self._targets_pub = guarded_publisher(
                self, ExternalTargetList, "targets", _latched()
            )
        # The acknowledgement, inbound from the gateway, outbound on the wire.
        # Subscribed only with ingress on: with no goals coming in there is
        # nothing that could be acknowledged.
        self._goal_done_sub = None
        self._acks_forwarded = 0
        self._telemetry_json_pub = None
        if self._telemetry_debug:
            # Local debugging only. The same JSON goes out over the WebSocket;
            # this copy exists so the payload can be inspected with `ros2 topic
            # echo` without a connected counterpart.
            self._telemetry_json_pub = guarded_publisher(
                self, String, "telemetry_json", _reliable(1)
            )

        # --- the client ---------------------------------------------------
        self._client = RosbridgeClient(
            self._url,
            max_msg_bytes=int(self.get_parameter("max_msg_bytes").value),
            ws_max_size=int(self.get_parameter("ws_max_size").value),
            ping_interval_sec=float(self.get_parameter("ping_interval_sec").value),
            ping_timeout_sec=float(self.get_parameter("ping_timeout_sec").value),
            backoff_initial_sec=float(self.get_parameter("backoff_initial_sec").value),
            backoff_max_sec=float(self.get_parameter("backoff_max_sec").value),
            on_state_change=self._on_link_state,
            logger=None,
        )
        self._client.subscribe(proto.TOPIC_DATUM, "sensor_msgs/NavSatFix", self._on_datum)
        if self._ingress:
            self._client.subscribe(
                proto.TOPIC_TRASH_GOAL, "sensor_msgs/NavSatFix", self._on_goal
            )
            self._client.subscribe(
                proto.TOPIC_TRASH_GPS, "std_msgs/String", self._on_trash_gps
            )
        if self._transform_enabled:
            self._client.subscribe(
                proto.TOPIC_TRANSFORM_STATUS, "std_msgs/String", self._on_transform_status
            )
        if self._publish_telemetry:
            self._client.advertise(proto.TOPIC_DEVICE_STATUS, "std_msgs/String")
        if self._ingress:
            self._client.advertise(proto.TOPIC_TRASH_GOAL_DONE, "std_msgs/String")

        # --- telemetry from the gateway -----------------------------------
        group = MutuallyExclusiveCallbackGroup()
        self._telemetry_sub = self.create_subscription(
            RobotTelemetry, "telemetry", self._on_telemetry, _reliable(1), callback_group=group
        )
        if self._ingress:
            self._goal_done_sub = self.create_subscription(
                String, "goal_done", self._on_goal_done, _reliable(), callback_group=group
            )

        # --- timers, BOTH ON A STEADY CLOCK (SAFETY.md F-29) -----------------
        # This node holds no sim-time quantity at all. It drains a WebSocket and
        # it publishes a heartbeat about that WebSocket; both are wall-clock
        # events, and `last_message_age_sec` in that heartbeat is already
        # measured monotonically for exactly this reason.
        #
        # The heartbeat is the safety-relevant one. Since F-29 the gateway's
        # link watchdog judges silence against `link_lost_sec` MONOTONICALLY, so
        # a heartbeat delivered by a ROS timer would slow down with the
        # simulation while the tolerance did not: at a real-time factor of 0.1
        # this 1 Hz publication becomes one every 10 wall-seconds, and a 5 s
        # tolerance would then declare LINK_LOST on a perfectly healthy link,
        # continuously. Producer and consumer must be on the same clock, and for
        # a WiFi link that clock is the wall.
        #
        # The drain follows for the same reason one level down: a starved drain
        # delays their target list, whose age the gateway also measures
        # monotonically (F-28), so it would eventually be refused as stale while
        # nothing was wrong with the stream.
        drain_hz = max(0.5, float(self.get_parameter("drain_rate_hz").value))
        status_hz = max(0.1, float(self.get_parameter("status_rate_hz").value))
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._drain_timer = self.create_timer(
            1.0 / drain_hz, self._drain, callback_group=group, clock=self._wall_clock
        )
        self._status_timer = self.create_timer(
            1.0 / status_hz,
            self._publish_status,
            callback_group=group,
            clock=self._wall_clock,
        )

        # C-6 / SAFETY.md F-7: what the node ACTUALLY publishes, after
        # remapping, checked before the client thread can produce anything.
        assert_no_chain_publishers(self)
        # SAFETY.md 6.3: and what it can CALL. This node has no client of any
        # kind - it is a transport adapter - so the allowed list is empty.
        assert_no_command_clients(self)

        self._thread = threading.Thread(target=self._run_client, name="rosbridge", daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"octopus_link_node up: url={self._url} goal_ingress_enabled={self._ingress} "
            f"telemetry={'on' if self._publish_telemetry else 'off'}"
        )
        if not self._ingress:
            self.get_logger().info(
                "goal ingress is OFF: /octopus/trash_goal and /octopus/trash_gps are "
                "not subscribed, so no external goal can enter this process "
                "(rollout stage 1, telemetry only)"
            )
        if self._transform_enabled:
            self.get_logger().info(
                f"subscribed to {proto.TOPIC_TRANSFORM_STATUS} (report only): a "
                "re-lock of the Octopus map frame is counted and logged, and "
                "nothing acts on it"
            )
        else:
            self.get_logger().warn(
                f"{proto.TOPIC_TRANSFORM_STATUS} is NOT subscribed "
                "(transform_status_enabled=false): a re-lock of the Octopus map "
                "frame would silently invalidate any measured alignment and we "
                "would not see it"
            )

    # -- client thread ----------------------------------------------------
    def _warn_if_clock_publisher(self, when: str) -> bool:
        """SAFETY.md F-35. WARN once if /clock is live while we are on wall time.

        Latched, because the condition does not change while the node runs and a
        mismatch repeated on every message is a mismatch nobody reads. Returns
        whether it warned, so the offline checks can assert the latch.
        """
        try:
            count = self.count_publishers("/clock")
        except Exception:  # noqa: BLE001 - a probe must never cost a startup
            return False
        message = clock_publisher_warning(count, self._use_sim_time, when)
        if message is None or self._clock_mismatch_warned:
            return False
        self._clock_mismatch_warned = True
        self.get_logger().warn(message)
        return True

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Refuse every runtime parameter change, with the reason. SAFETY.md F-8.

        There is no allow-list here because there is nothing to allow: this node
        has no parameter it re-reads. That is a property worth stating rather
        than a gap - it is a transport adapter, and changing the URL, the socket
        limits or the watchdog timeout of a link that is already up is a restart,
        not an adjustment.
        """
        names = ", ".join(sorted({p.name for p in params}))
        reason = (
            f"{names}: every parameter of octopus_link_node is read once at "
            "construction and never again - the URL and the socket limits are "
            "built into the client, the rates into the timers, link_lost_sec into "
            "the watchdog. A runtime change would be accepted and have no effect, "
            "so it is refused instead (SAFETY.md F-8). Restart with the new "
            "configuration."
        )
        self.get_logger().warn(f"REFUSED parameter change - {reason}")
        return SetParametersResult(successful=False, reason=reason)

    def _run_client(self) -> None:
        import asyncio

        try:
            asyncio.run(self._client.run())
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"rosbridge client thread ended: {exc}")

    def _on_link_state(self, connected: bool) -> None:
        # Called on the asyncio thread; logging is the only thing done here.
        if connected:
            self.get_logger().info(f"rosbridge connected: {self._url}")
        else:
            self.get_logger().warn(f"rosbridge disconnected: {self._url}")

    # -- inbound callbacks (asyncio thread; slot writes only) --------------
    def _on_datum(self, topic: str, msg: Any) -> None:
        with self._lock:
            self._pending_datum = msg

    def _on_goal(self, topic: str, msg: Any) -> None:
        # Latest-wins. The counterpart publishes exactly ONE goal at a time and
        # advances only on acknowledgement, republishing the same goal at 1 Hz,
        # so collapsing to the newest cannot skip a target at any sane drain
        # rate - it just removes the 1 Hz repetition.
        with self._lock:
            self._pending_goal = msg
            self._goals_seen += 1

    def _on_trash_gps(self, topic: str, msg: Any) -> None:
        with self._lock:
            self._pending_gps = msg

    def _on_transform_status(self, topic: str, msg: Any) -> None:
        # Latest-wins like the rest. Their 1 Hz republication of an UNCHANGED
        # lock carries no information, and a re-lock changes the value, so the
        # newest sample always carries the event - collapsing cannot lose one.
        with self._lock:
            self._pending_transform = msg

    # -- drain (executor thread) -------------------------------------------
    def _drain(self) -> None:
        with self._lock:
            datum_raw, self._pending_datum = self._pending_datum, None
            goal_raw, self._pending_goal = self._pending_goal, None
            gps_raw, self._pending_gps = self._pending_gps, None
            transform_raw, self._pending_transform = self._pending_transform, None

        if datum_raw is not None:
            self._publish_datum(datum_raw)
        if transform_raw is not None:
            self._consume_transform_status(transform_raw)
        if gps_raw is not None:
            self._publish_targets(gps_raw)
        if goal_raw is not None:
            self._publish_goal(goal_raw)

    def _note_parse_error(self, what: str, exc: Exception) -> None:
        self._parse_errors += 1
        self._last_parse_error = f"{what}: {exc}"
        # WARN, not ERROR: a malformed payload is the peer's fault, and FR-12
        # item 6 reserves ERROR (plus an ERROR diagnostic) for our-side failures.
        self.get_logger().warn(
            f"dropping malformed {what} from the external side: {exc}",
            throttle_duration_sec=5.0,
        )

    def _datum_msg(self, fix: proto.NavSatFixPayload) -> GeodeticDatum:
        msg = GeodeticDatum()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = fix.frame_id or proto.OCTOPUS_FRAME_ID
        msg.latitude_deg = fix.latitude_deg
        msg.longitude_deg = fix.longitude_deg
        msg.from_topic = True
        msg.is_bootstrap_fallback = is_bootstrap_fallback(
            Datum(fix.latitude_deg, fix.longitude_deg), BOOTSTRAP_MATCH_TOLERANCE_DEG
        )
        msg.age_sec = 0.0
        return msg

    def _publish_datum(self, raw: Any) -> None:
        try:
            fix = proto.parse_navsatfix(raw)
        except proto.ProtocolError as exc:
            self._note_parse_error("datum", exc)
            return
        msg = self._datum_msg(fix)
        self._datum_pub.publish(msg)
        if msg.is_bootstrap_fallback:
            self.get_logger().warn(
                "the external datum is the Octopus bootstrap fallback "
                "(Garching); coordinates derived from it mean nothing and the "
                "gateway will refuse to dispatch on it",
                throttle_duration_sec=30.0,
            )

    def _publish_goal(self, raw: Any) -> None:
        if self._goal_pub is None:
            return
        try:
            fix = proto.parse_navsatfix(raw)
        except proto.ProtocolError as exc:
            self._note_parse_error("trash_goal", exc)
            # A parse failure is still a goal event: the gateway has to be able
            # to report it as a rejection rather than see silence, so it is
            # forwarded with well_formed semantics carried by NaN coordinates.
            msg = ExternalGoal()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.source = self._source
            msg.target_id = ""
            msg.object_latitude_deg = float("nan")
            msg.object_longitude_deg = float("nan")
            msg.confidence = float("nan")
            msg.resolved = False
            self._goal_pub.publish(msg)
            return

        msg = ExternalGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = fix.frame_id or "map"
        msg.source = self._source
        # Their NavSatFix carries no id - only trash_gps does. The gateway
        # correlates by position against the target list; leaving it empty here
        # is honest rather than inventing one.
        msg.target_id = ""
        msg.object_latitude_deg = fix.latitude_deg
        msg.object_longitude_deg = fix.longitude_deg
        msg.confidence = float("nan")
        msg.resolved = False
        # The stamp the counterpart set, preserved for the staleness check. Both
        # of their producers always set it (verified 2026-08-18), so an unset
        # stamp is a fault, not a variant.
        if fix.stamp_sec is not None:
            msg.header.stamp = _stamp_from_sec(fix.stamp_sec)
        msg.datum.header.stamp = msg.header.stamp
        msg.datum.latitude_deg = float("nan")
        msg.datum.longitude_deg = float("nan")
        self._goal_pub.publish(msg)

    def _publish_targets(self, raw: Any) -> None:
        if self._targets_pub is None:
            return
        try:
            report = proto.parse_trash_gps(raw)
        except proto.ProtocolError as exc:
            self._note_parse_error("trash_gps", exc)
            return

        msg = ExternalTargetList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.source = self._source
        msg.goal_id = report.goal_id or ""
        msg.open_count_valid = report.open_count is not None
        msg.open_count = int(report.open_count or 0)
        if report.datum is not None:
            msg.datum.header.stamp = msg.header.stamp
            msg.datum.latitude_deg = report.datum.latitude_deg
            msg.datum.longitude_deg = report.datum.longitude_deg
            msg.datum.from_topic = report.datum.from_topic
            msg.datum.is_bootstrap_fallback = is_bootstrap_fallback(
                Datum(report.datum.latitude_deg, report.datum.longitude_deg),
                BOOTSTRAP_MATCH_TOLERANCE_DEG,
            )
            msg.datum.age_sec = 0.0
        else:
            msg.datum.latitude_deg = float("nan")
            msg.datum.longitude_deg = float("nan")
            msg.datum.age_sec = -1.0

        for target in report.targets:
            entry = ExternalTarget()
            entry.id = target.id
            entry.latitude_deg = target.latitude_deg
            entry.longitude_deg = target.longitude_deg
            entry.source_x = _f(target.x)
            entry.source_y = _f(target.y)
            entry.confidence = _f(target.confidence)
            entry.collected = bool(target.collected)
            entry.is_goal = bool(target.is_goal)
            entry.last_seen = _f(
                target.last_seen if isinstance(target.last_seen, (int, float)) else None
            )
            msg.targets.append(entry)
        self._targets_pub.publish(msg)

    # -- the Octopus's frame lock (report only) -----------------------------
    def _transform_age_sec(self) -> float:
        """Seconds since the last transform status. ``-1.0`` if never."""
        with self._lock:
            received = self._transform_rx_monotonic
        if received is None:
            return -1.0
        return max(0.0, time.monotonic() - received)

    def _consume_transform_status(self, raw: Any) -> None:
        """Track their startup yaw lock. NOTHING IS ACTED ON HERE.

        A re-lock means an alignment we measured earlier is dead. It is counted,
        logged and reported - it does not cancel, disarm, blacklist or refuse
        anything, because what it should cause has not been decided (SAFETY.md
        F-40's pattern). Whoever adds a consequence here should be adding it
        against a user decision, not against this comment.
        """
        try:
            status = proto.parse_transform_status(raw)
        except proto.ProtocolError as exc:
            self._note_parse_error("flight_camera_transform/status", exc)
            return

        with self._lock:
            self._transform_rx_monotonic = time.monotonic()
        update = self._transform_tracker.update(status)
        if update.relocked:
            # WARN and not ERROR: ERROR in this package means OUR stack failed
            # (FR-12 item 6), and this is the counterpart restarting its own
            # node. Not throttled - it is a rare, named event and every
            # occurrence matters.
            delta = update.align_angle_delta_rad
            self.get_logger().warn(
                "OCTOPUS FRAME RE-LOCK: indoor_static_yaw_zero_rad moved "
                f"{update.previous_yaw_zero_rad!r} -> {update.yaw_zero_rad!r} rad. "
                "align_angle "
                f"{_deg(update.previous_align_angle_rad)} -> {_deg(update.align_angle_rad)} "
                f"(delta {_deg(delta)}). Their transform node re-locked its startup "
                "yaw, so any their-map -> our-map alignment measured before this "
                "moment is NO LONGER TRUE and has to be re-established. This is "
                f"re-lock #{self._transform_tracker.relocks} since startup. "
                "REPORT ONLY - nothing was cancelled, disarmed or blocked."
            )
        elif update.first_lock:
            self.get_logger().info(
                "Octopus frame lock observed: indoor_static_yaw_zero_rad="
                f"{update.yaw_zero_rad!r} rad, align_angle={_deg(update.align_angle_rad)}, "
                f"state={status.state!r}"
            )

    def _transform_telemetry_block(self) -> Optional[Dict[str, Any]]:
        """The block mirrored back to them in the outbound telemetry.

        ``None`` when we are not subscribed, which is a different statement from
        "no lock" and must not be collapsed into one (FR-12 item 8).
        """
        if not self._transform_enabled:
            return None
        tracker = self._transform_tracker
        status = tracker.status
        age = self._transform_age_sec()
        return {
            "status": "available" if status is not None else "unavailable",
            "reason": "" if status is not None else "NOT_RECEIVED_YET",
            "state": None if status is None else status.state,
            "transform_ready": None if status is None else status.transform_ready,
            # LIVE: does the newest sample carry a lock? `yaw_zero_rad` below is
            # the LAST KNOWN one, which survives a gap - the two disagree while
            # their node is republishing `null` through its own restart, and
            # that disagreement is information, not an inconsistency.
            "locked": False if status is None else status.locked,
            "yaw_zero_rad": tracker.yaw_zero_rad,
            "align_angle_rad": tracker.align_angle_rad,
            "last_message_age_sec": None if age < 0.0 else age,
            "relocks": tracker.relocks,
            "last_relock_from_rad": tracker.last_relock_from_rad,
            "last_relock_to_rad": tracker.last_relock_to_rad,
        }

    # -- outbound telemetry -------------------------------------------------
    def _on_telemetry(self, msg: RobotTelemetry) -> None:
        """Typed telemetry in, external JSON out. No decision is taken here."""
        payload = proto.build_device_status(
            latitude_deg=msg.latitude_deg if msg.latlon_valid else None,
            longitude_deg=msg.longitude_deg if msg.latlon_valid else None,
            map_x=msg.map_x if msg.pose_valid else None,
            map_y=msg.map_y if msg.pose_valid else None,
            yaw_deg=msg.yaw_deg if msg.pose_valid else None,
            nav_state=msg.nav_state,
            active_goal_id=msg.active_goal_id or None,
            armed=msg.armed,
            link_ok=msg.link_ok,
            stamp_sec=_stamp_to_sec(msg.header.stamp),
            battery_status=msg.battery_status,
            battery_reason=msg.battery_reason,
            battery_percent=msg.battery_percent if msg.battery_percent_valid else None,
            pose_status="available" if msg.pose_valid else "unavailable",
            pose_reason=msg.pose_reason,
            latlon_status="available" if msg.latlon_valid else "unavailable",
            latlon_reason=msg.latlon_reason,
            nav_state_reason=msg.nav_state_reason,
            last_disarm_trigger=msg.last_disarm_trigger,
            arming_seconds_remaining=(
                float(msg.arming_seconds_remaining) if msg.armed else None
            ),
            teleop_mode=msg.teleop_mode or None,
            speed_mps=float(msg.speed_mps) if msg.odom_valid else None,
            link_last_message_age_sec=(
                None
                if msg.link_last_message_age_sec < 0.0
                else float(msg.link_last_message_age_sec)
            ),
            link_reconnects=int(msg.link_reconnects),
            counters={
                "goals_received": int(msg.goals_received),
                "goals_accepted": int(msg.goals_accepted),
                "goals_rejected": int(msg.goals_rejected),
                "goals_preview": int(msg.goals_preview),
                "last_reject_reason": msg.last_reject_reason or None,
            },
            blacklist=list(msg.blacklisted_ids),
            # Observed from their own topic, mirrored straight back. Purely
            # informational, exactly like everything else in this payload
            # (SR-15 rule 3: telemetry is outbound only and carries no control
            # semantics).
            octopus_transform=self._transform_telemetry_block(),
        )
        if self._telemetry_json_pub is not None:
            debug = String()
            debug.data = payload
            self._telemetry_json_pub.publish(debug)
        if self._publish_telemetry:
            self._client.publish(proto.TOPIC_DEVICE_STATUS, {"data": payload})

    # -- status + diagnostics ----------------------------------------------
    def _on_goal_done(self, msg: String) -> None:
        """Forward one acknowledgement to the wire. No decision, no filtering.

        ``build_goal_done`` emits the bare id, which is the form their doc shows
        and their node was written against; their parser also accepts
        ``{"id": n}``, and ours accepts both, but symmetry is not a reason to
        send the less-tested form. A malformed id is dropped and counted rather
        than sent: an acknowledgement is irreversible, so a payload their parser
        might read as some other id must not go out at all.
        """
        try:
            payload = proto.build_goal_done(msg.data)
        except proto.ProtocolError as exc:
            self._note_parse_error("goal_done (outbound)", exc)
            return
        sent = self._client.publish(proto.TOPIC_TRASH_GOAL_DONE, {"data": payload})
        if sent:
            self._acks_forwarded += 1
            self.get_logger().warn(
                f"forwarded trash_goal_done for target {payload} to the external "
                "side; it means COLLECTED there and cannot be taken back"
            )
        else:
            # Their mission does not advance, and nobody may learn that by
            # noticing nothing happened (SR-13).
            self.get_logger().error(
                f"could NOT forward trash_goal_done for target {payload}: the link "
                "is down. The source still believes this target is open."
            )

    def _publish_status(self) -> None:
        stats = self._client.stats()
        age = self._client.last_receive_age_sec()

        msg = ExternalLinkStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.url = self._url
        msg.connected = bool(stats.connected)
        msg.reconnect_count = int(stats.reconnects)
        msg.last_message_age_sec = float(age)
        # open_count belongs to the gateway's view of the target list; the link
        # node reports only what it can observe itself.
        msg.open_count = 0
        msg.open_count_valid = False
        msg.datum_valid = False
        msg.goals_received = int(self._goals_seen)
        msg.goals_rejected = int(self._parse_errors)

        # Their map-frame transform. FRESHNESS IS FOLDED IN HERE, not left to
        # the gateway: a "ready" we last heard a minute ago is exactly what
        # their node dying looks like, and the timestamps live on this side.
        tracker = self._transform_tracker
        status = tracker.status
        age = self._transform_age_sec()
        fresh = age >= 0.0 and age <= self._transform_stale_sec
        msg.frame_status_enabled = bool(self._transform_enabled)
        msg.frame_status_seen = bool(self._transform_enabled and status is not None)
        msg.frame_ready = bool(
            self._transform_enabled
            and status is not None
            and status.ready
            and status.locked
            and fresh
        )
        align = tracker.align_angle_rad
        msg.frame_align_angle_valid = bool(align is not None)
        msg.frame_align_angle_rad = float(align) if align is not None else 0.0
        msg.frame_relocks = int(tracker.relocks)

        self._link_pub.publish(msg)

        self._diag_pub.publish(
            diag.array(
                msg.header.stamp,
                [
                    diag.link_status(
                        connected=stats.connected,
                        url=self._url,
                        last_message_age_sec=age,
                        reconnects=stats.reconnects,
                        link_lost_sec=self._link_lost_sec,
                        extra={
                            "goal_ingress_enabled": self._ingress,
                            "subscribed": ", ".join(stats.subscribed_topics) or None,
                            "acks_forwarded": self._acks_forwarded,
                            "advertised": ", ".join(stats.advertised_topics) or None,
                            "parse_errors": self._parse_errors,
                            "last_parse_error": self._last_parse_error or None,
                            "dropped_disconnected": stats.dropped_disconnected,
                            "oversize_dropped_in": stats.oversize_dropped_in,
                        },
                    ),
                    self._transform_diag(),
                ],
            )
        )

    def _transform_diag(self):
        tracker = self._transform_tracker
        status = tracker.status
        return diag.transform_lock_status(
            subscribed=self._transform_enabled,
            observed=status is not None,
            state="" if status is None else status.state,
            transform_ready=None if status is None else status.transform_ready,
            locked=False if status is None else status.locked,
            yaw_zero_rad=tracker.yaw_zero_rad,
            align_angle_rad=tracker.align_angle_rad,
            last_message_age_sec=self._transform_age_sec(),
            stale_after_sec=self._transform_stale_sec,
            relocks=tracker.relocks,
            last_relock_from_rad=tracker.last_relock_from_rad,
            last_relock_to_rad=tracker.last_relock_to_rad,
            last_relock_align_delta_rad=tracker.last_relock_align_delta_rad,
            extra={"samples": tracker.samples, "relock_epsilon_rad": tracker.relock_epsilon_rad},
        )

    def destroy_node(self) -> bool:
        self._client.stop()
        return super().destroy_node()


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _stamp_from_sec(seconds: float):
    from builtin_interfaces.msg import Time

    stamp = Time()
    stamp.sec = int(math.floor(seconds))
    stamp.nanosec = int(round((seconds - stamp.sec) * 1e9))
    return stamp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = OctopusLinkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM. This node holds no goal and cancels nothing, so a clean stop
        # is the whole obligation - unlike the gateway, which needs the
        # restructured main in goal_gateway_node (SAFETY.md F-4).
        pass
    except SystemExit as exc:
        # See goal_gateway_node.main: the SR-8 guard's exit code must survive,
        # or a refused start is indistinguishable from a clean shutdown.
        code = int(exc.code) if isinstance(exc.code, int) else 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == "__main__":
    main()
