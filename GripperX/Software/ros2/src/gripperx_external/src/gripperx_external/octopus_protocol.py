"""Parsing and emission of the Octopus payloads.

The counterpart uses stock message types only - there is not a single custom
``.msg``/``.srv``/``.action`` in its branch. ``NavSatFix`` was chosen because
``geographic_msgs`` and ``nav2_msgs`` are not installed on the demo laptop, and
JSON-in-``std_msgs/String`` is the house style across the whole Octopus stack.
This module is the single place that knows that, which is what keeps the
transport (rosbridge today) swappable.

Contract as of branch tip ``a7ab8e6278`` (2026-08-17), authoritative doc
``Octopus/docs/octopus_to_robot_interface.md``:

===================================== ======================= =========
Topic                                 Type                    Direction
===================================== ======================= =========
``/octopus/fake_eve_gps_start``       ``sensor_msgs/NavSatFix``  -> robot (the datum)
``/octopus/trash_goal``               ``sensor_msgs/NavSatFix``  -> robot (ONE current goal)
``/octopus/trash_gps``                ``std_msgs/String`` JSON   -> robot (all targets)
``/octopus/trash_goal_done``          ``std_msgs/String``        <- robot (finished id)
``/octopus/flight_camera_transform/status``
                                      ``std_msgs/String`` JSON   -> robot (frame lock, ~1 Hz)
===================================== ======================= =========

The fifth topic was added on 2026-08-21, after their answer to Q1. It carries no
goal and no target: it reports the startup yaw their map frame is locked to, and
we read it to learn when that lock CHANGED - see :class:`TransformStatus`.

Two behaviours of theirs drive the parsing:

* They publish exactly one goal at a time and advance only on
  ``trash_goal_done``. When nothing is open they stop publishing ``trash_goal``
  entirely, so ``open_count`` from ``trash_gps`` is the ONLY way to tell
  "mission complete" from "source went stale". It is therefore parsed as a
  first-class field, not an extra.
* Their ``trash_goal_done`` parser accepts both a bare id (``"1"``) and a JSON
  object (``{"id": 1}``), so ours accepts both too - symmetry here is free and
  the alternative is a silent no-op if either side changes its mind.

Ids are normalised to ``str``. They are ints today, but they restart at 1 on
node restart (proposal item 3), so we never do arithmetic on them and never
assume ordering.

KEY NAMES ARE PARSED STRICTLY - DO NOT ADD ALIASES (FR-12 item 6)
=================================================================
The two payload families spell the coordinate keys differently, and each
spelling is canonical exactly where it appears:

* ``/octopus/fake_eve_gps_start`` and ``/octopus/trash_goal`` arrive as
  serialised ``sensor_msgs/NavSatFix``, so the keys are ``latitude`` /
  ``longitude``.
* ``/octopus/trash_gps`` is hand-written JSON on their side, and its keys are
  ``lat`` / ``lon`` - **never** ``latitude`` / ``longitude``. ``last_seen`` is
  an epoch float.

Accepting both everywhere would let a rename on the Octopus side pass silently
through a tolerant reader, which is the failure mode a written interface exists
to prevent. A missing or renamed key is therefore a ``ProtocolError`` naming the
key we expected - and, when the other spelling is present, saying so.

Pure module: no rclpy, no ROS message types. Inputs are the plain dicts/strings
a rosbridge client hands over.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

TOPIC_DATUM = "/octopus/fake_eve_gps_start"
TOPIC_TRASH_GOAL = "/octopus/trash_goal"
TOPIC_TRASH_GPS = "/octopus/trash_gps"
TOPIC_TRASH_GOAL_DONE = "/octopus/trash_goal_done"

#: The fifth ingress topic (2026-08-21). ``std_msgs/String`` carrying JSON at
#: ~1 Hz. Their existing ``--topics_glob "['/octopus/*']"`` already covers it,
#: so subscribing changed nothing on their side.
TOPIC_TRANSFORM_STATUS = "/octopus/flight_camera_transform/status"

#: Proposed, not yet on their side (proposal item 1). Their existing
#: ``/octopus/devices/{id}/...`` namespace; the dashboard already carries a
#: ``gripperx`` fleet entry (alias ``robot_2``) with a battery field and a
#: fallback pose waiting for a source.
TOPIC_DEVICE_STATUS = "/octopus/devices/gripperx/status"

#: sensor_msgs/NavSatStatus.STATUS_NO_FIX
STATUS_NO_FIX = -1
#: sensor_msgs/NavSatStatus.STATUS_FIX - what both their nodes always set.
STATUS_FIX = 0

# --- what their two producers actually put on the wire ---------------------
# Verified against their source on 2026-08-18. These are facts about the
# counterpart, not preferences of ours, which is why they are named constants
# here rather than literals in the fake: if their nodes change, this is the one
# place that has to change with them.
#: ``header.frame_id`` on both NavSatFix topics.
OCTOPUS_FRAME_ID = "map"
#: sigma 0.5 m in x/y, 1.0 in z. A hardcoded *estimate* on their side, not a
#: measurement - see the accuracy caveat in the design.
OCTOPUS_POSITION_COVARIANCE = (0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0)
#: sensor_msgs/NavSatFix.COVARIANCE_TYPE_APPROXIMATED
OCTOPUS_POSITION_COVARIANCE_TYPE = 1


class ProtocolError(Exception):
    """Malformed payload. Carries a machine-readable ``reason``."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _require_mapping(payload: Any, what: str) -> Mapping[str, Any]:
    if not isinstance(payload, Mapping):
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is not an object: {type(payload).__name__}")
    return payload


def _require_key(
    payload: Mapping[str, Any],
    name: str,
    what: str,
    drift_names: Sequence[str] = (),
) -> Any:
    """Read exactly one key. No aliases, deliberately (FR-12 item 6).

    The two payload families use different spellings and each one is canonical
    in its own place: the ``NavSatFix`` topics carry ``latitude``/``longitude``
    because rosbridge serialises the stock message, while their hand-written
    ``trash_gps`` JSON carries ``lat``/``lon``. Accepting both everywhere would
    make a rename on their side invisible to us - a tolerant reader masks
    exactly the protocol drift a written interface exists to catch. So a
    missing or differently-named key is a rejection with a reason, and when the
    *other* spelling is present we say so, because that is the one case where
    the operator needs to know it is a rename and not a bug on our side.
    """
    if name in payload:
        return payload[name]
    found = [other for other in drift_names if other in payload]
    if found:
        raise ProtocolError(
            "MALFORMED_PAYLOAD",
            f"{what}: expected key '{name}', found {found!r} instead - the "
            "counterpart appears to have renamed it. This parser is strict on "
            "purpose (FR-12); update the contract on both sides rather than "
            "adding an alias here.",
        )
    raise ProtocolError("MALFORMED_PAYLOAD", f"{what}: required key '{name}' is missing")


def _as_float(value: Any, what: str, allow_none: bool = False) -> Optional[float]:
    if value is None:
        if allow_none:
            return None
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is missing")
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is not a number: {value!r}")
    try:
        return float(value)
    except ValueError as exc:
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is not a number: {value!r}") from exc


def _as_id(value: Any, what: str) -> str:
    if value is None:
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is missing")
    if isinstance(value, bool):
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is a bool: {value!r}")
    if isinstance(value, float):
        # An id arriving as 1.0 is still id 1; JSON has no int/float distinction.
        if not value.is_integer():
            raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is not an integer: {value!r}")
        return str(int(value))
    text = str(value).strip()
    if not text:
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is empty")
    return text


def _stamp_to_sec(stamp: Any) -> Optional[float]:
    """``builtin_interfaces/Time`` as rosbridge JSON -> float seconds."""
    if not isinstance(stamp, Mapping):
        return None
    sec = stamp.get("sec")
    nanosec = stamp.get("nanosec", stamp.get("nsec", 0))
    if sec is None:
        return None
    try:
        return float(sec) + float(nanosec) * 1e-9
    except (TypeError, ValueError):
        return None


def decode_string_message(payload: Any) -> str:
    """``std_msgs/String`` as rosbridge JSON, or the bare ``data`` string."""
    if isinstance(payload, str):
        return payload
    mapping = _require_mapping(payload, "String message")
    data = mapping.get("data")
    if not isinstance(data, str):
        raise ProtocolError("MALFORMED_PAYLOAD", "String message has no string `data`")
    return data


# ---------------------------------------------------------------------------
# 1 + 2: NavSatFix (datum and goal share the type)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class NavSatFixPayload:
    latitude_deg: float
    longitude_deg: float
    altitude_m: Optional[float]
    status: int
    frame_id: str
    stamp_sec: Optional[float]

    @property
    def has_fix(self) -> bool:
        """``status >= 0``. NO_FIX (-1) means the value carries no position."""
        return self.status >= 0


def parse_navsatfix(payload: Any) -> NavSatFixPayload:
    """Parse either ``/octopus/fake_eve_gps_start`` or ``/octopus/trash_goal``.

    ``status`` defaults to 0 (STATUS_FIX) when absent: rosbridge omits nothing
    for a real ``NavSatFix``, but ``fake_eve_gps_bridge_node`` constructs its
    messages by hand, and a synthesised fix without an explicit status is a fix.
    An explicit ``-1`` is preserved and later rejected by the validation
    pipeline.
    """
    mapping = _require_mapping(payload, "NavSatFix")
    # Canonical keys, verified against their source 2026-08-18: these payloads
    # come through rosbridge as a serialised sensor_msgs/NavSatFix, so the
    # spellings are the message field names. NOT lat/lon - that is the
    # trash_gps JSON, parsed below with its own canonical names.
    latitude = _as_float(
        _require_key(mapping, "latitude", "NavSatFix", ("lat",)), "NavSatFix.latitude"
    )
    longitude = _as_float(
        _require_key(mapping, "longitude", "NavSatFix", ("lon", "lng")),
        "NavSatFix.longitude",
    )
    assert latitude is not None and longitude is not None
    altitude = _as_float(mapping.get("altitude"), "NavSatFix.altitude", allow_none=True)

    status_field = mapping.get("status")
    if isinstance(status_field, Mapping):
        status_value = status_field.get("status", 0)
    elif status_field is None:
        status_value = 0
    else:
        status_value = status_field
    try:
        status = int(status_value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(
            "MALFORMED_PAYLOAD", f"NavSatFix.status is not an int: {status_value!r}"
        ) from exc

    header = mapping.get("header") if isinstance(mapping.get("header"), Mapping) else {}
    frame_id = str(header.get("frame_id", "") or "")
    return NavSatFixPayload(
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=altitude,
        status=status,
        frame_id=frame_id,
        stamp_sec=_stamp_to_sec(header.get("stamp")),
    )


def build_navsatfix(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float = 0.0,
    status: int = STATUS_FIX,
    frame_id: str = "",
    stamp_sec: float = 0.0,
    position_covariance: Optional[Sequence[float]] = None,
    position_covariance_type: int = 0,
) -> Dict[str, Any]:
    """Emit a ``NavSatFix`` in rosbridge JSON form.

    Only needed by the fake Octopus and the offline tests - we never publish on
    their inbound topics in production.

    The covariance defaults stay neutral so that existing callers keep emitting
    what they emitted before. To reproduce the *real* wire format, pass
    ``OCTOPUS_POSITION_COVARIANCE`` and ``OCTOPUS_POSITION_COVARIANCE_TYPE``, as
    ``fake_octopus.py`` does - our own parser ignores both fields, so they matter
    for fidelity of the fake rather than for behaviour.
    """
    sec = int(math.floor(stamp_sec))
    covariance = (
        [0.0] * 9 if position_covariance is None else [float(v) for v in position_covariance]
    )
    if len(covariance) != 9:
        raise ProtocolError("MALFORMED_PAYLOAD", "position_covariance must have 9 entries")
    return {
        "header": {
            "stamp": {"sec": sec, "nanosec": int(round((stamp_sec - sec) * 1e9))},
            "frame_id": frame_id,
        },
        "status": {"status": int(status), "service": 1},
        "latitude": float(latitude_deg),
        "longitude": float(longitude_deg),
        "altitude": float(altitude_m),
        "position_covariance": covariance,
        "position_covariance_type": int(position_covariance_type),
    }


# ---------------------------------------------------------------------------
# 3: /octopus/trash_gps (JSON in String)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TrashTarget:
    id: str
    latitude_deg: float
    longitude_deg: float
    #: Their own map-metre values. We do NOT use these for navigation - we
    #: re-derive x/y from lat/lon with the shared datum, so that one arithmetic
    #: path is authoritative. Kept for cross-checking at rollout stage 2, where
    #: a mismatch between the two would expose the origin offset.
    x: Optional[float]
    y: Optional[float]
    #: ``null`` is a valid value on their side and stays ``None`` here.
    confidence: Optional[float]
    collected: bool
    is_goal: bool
    #: Verbatim: their doc does not fix the type (epoch float today).
    last_seen: Any = None


@dataclass(frozen=True)
class TrashDatumInfo:
    latitude_deg: float
    longitude_deg: float
    #: Their own statement of which datum case is live. False means they are
    #: still on the Garching bootstrap fallback and nothing they send means
    #: anything yet.
    from_topic: bool


@dataclass(frozen=True)
class TrashGpsReport:
    datum: Optional[TrashDatumInfo]
    goal_id: Optional[str]
    #: Targets still open. 0 means "mission complete", NOT "stale".
    open_count: Optional[int]
    targets: Tuple[TrashTarget, ...]

    @property
    def mission_complete(self) -> bool:
        return self.open_count == 0

    def goal_target(self) -> Optional[TrashTarget]:
        """The target flagged ``is_goal``, else the one matching ``goal_id``."""
        for target in self.targets:
            if target.is_goal:
                return target
        if self.goal_id is not None:
            for target in self.targets:
                if target.id == self.goal_id:
                    return target
        return None


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y")
    return default


def parse_trash_gps(payload: Any) -> TrashGpsReport:
    """Parse ``/octopus/trash_gps``.

    Accepts a ``std_msgs/String`` in rosbridge JSON form, a bare JSON string, or
    an already-decoded mapping.
    """
    if isinstance(payload, Mapping) and "data" in payload and isinstance(payload["data"], str):
        text: Optional[str] = payload["data"]
    elif isinstance(payload, str):
        text = payload
    else:
        text = None

    if text is not None:
        try:
            body: Any = json.loads(text)
        except (ValueError, TypeError) as exc:
            raise ProtocolError("MALFORMED_JSON", f"trash_gps is not JSON: {exc}") from exc
    else:
        body = payload
    mapping = _require_mapping(body, "trash_gps")

    datum_field = mapping.get("datum")
    datum: Optional[TrashDatumInfo] = None
    if isinstance(datum_field, Mapping):
        # Canonical keys here are lat/lon - NEVER latitude/longitude. This JSON
        # is hand-written by trash_gps_goal_node, not serialised from a ROS
        # message (verified against their source 2026-08-18).
        latitude = _as_float(
            _require_key(datum_field, "lat", "trash_gps.datum", ("latitude",)), "datum.lat"
        )
        longitude = _as_float(
            _require_key(datum_field, "lon", "trash_gps.datum", ("longitude", "lng")),
            "datum.lon",
        )
        assert latitude is not None and longitude is not None
        datum = TrashDatumInfo(
            latitude_deg=latitude,
            longitude_deg=longitude,
            from_topic=_as_bool(datum_field.get("from_topic"), default=False),
        )
    elif datum_field is not None:
        raise ProtocolError("MALFORMED_PAYLOAD", "trash_gps.datum is not an object")

    goal_id_field = mapping.get("goal_id")
    goal_id = None if goal_id_field is None else _as_id(goal_id_field, "trash_gps.goal_id")

    open_count_field = mapping.get("open_count")
    open_count: Optional[int] = None
    if open_count_field is not None:
        try:
            open_count = int(open_count_field)
        except (TypeError, ValueError) as exc:
            raise ProtocolError(
                "MALFORMED_PAYLOAD", f"trash_gps.open_count is not an int: {open_count_field!r}"
            ) from exc

    targets_field = mapping.get("targets", [])
    if not isinstance(targets_field, (list, tuple)):
        raise ProtocolError("MALFORMED_PAYLOAD", "trash_gps.targets is not an array")

    targets: List[TrashTarget] = []
    for index, entry in enumerate(targets_field):
        item = _require_mapping(entry, f"trash_gps.targets[{index}]")
        latitude = _as_float(
            _require_key(item, "lat", f"trash_gps.targets[{index}]", ("latitude",)),
            f"targets[{index}].lat",
        )
        longitude = _as_float(
            _require_key(item, "lon", f"trash_gps.targets[{index}]", ("longitude", "lng")),
            f"targets[{index}].lon",
        )
        assert latitude is not None and longitude is not None
        targets.append(
            TrashTarget(
                id=_as_id(item.get("id"), f"targets[{index}].id"),
                latitude_deg=latitude,
                longitude_deg=longitude,
                x=_as_float(item.get("x"), f"targets[{index}].x", allow_none=True),
                y=_as_float(item.get("y"), f"targets[{index}].y", allow_none=True),
                confidence=_as_float(
                    item.get("confidence"), f"targets[{index}].confidence", allow_none=True
                ),
                collected=_as_bool(item.get("collected"), default=False),
                is_goal=_as_bool(item.get("is_goal"), default=False),
                last_seen=item.get("last_seen"),
            )
        )
    return TrashGpsReport(datum=datum, goal_id=goal_id, open_count=open_count, targets=tuple(targets))


def build_trash_gps(
    datum: Optional[TrashDatumInfo],
    goal_id: Optional[str],
    open_count: Optional[int],
    targets: Sequence[TrashTarget],
    *,
    source_id: Optional[str] = None,
    frame_id: Optional[str] = None,
    timestamp: Optional[float] = None,
    datum_xy: Optional[Tuple[float, float]] = None,
    class_name: Optional[str] = None,
    numeric_ids: bool = False,
) -> str:
    """Emit the ``trash_gps`` JSON string (for the fake Octopus and tests).

    THE KEYWORD ARGUMENTS EXIST TO REPRODUCE THE REAL PUBLISHER, and every one
    of them defaults to the older, smaller payload so existing callers and their
    asserted payloads are untouched.

    Captured from the running Octopus on 2026-08-21 (branch
    ``item-a-map-origin``), the envelope carries FOUR keys this function did not
    emit - ``source_id``, ``frame_id``, ``timestamp`` and ``datum.x``/``datum.y``
    - and each target carries ``class_name``. We read none of them, which is
    exactly why they are worth emitting: a fake that omits fields the real
    system sends cannot show that we tolerate them.

    ``numeric_ids`` is the one that matters most. The real payload carries
    ``"id": 1`` as a JSON **number**; this function emitted ``"id": "1"`` as a
    string, because our own dataclass keeps ids as ``str``. Both parse (``_as_id``
    accepts either), but a fake that only ever sends strings never exercises the
    path the real system actually uses.
    """
    datum_block: Optional[Dict[str, Any]] = None
    if datum is not None:
        datum_block = {
            "lat": datum.latitude_deg,
            "lon": datum.longitude_deg,
            "from_topic": datum.from_topic,
        }
        if datum_xy is not None:
            # Their datum block carries its own map coordinates, which are
            # (0.0, 0.0) since the item-A origin fix - the datum IS the origin.
            datum_block["x"], datum_block["y"] = float(datum_xy[0]), float(datum_xy[1])

    def _id(value: str) -> Any:
        if numeric_ids:
            try:
                return int(value)
            except (TypeError, ValueError):
                return value
        return value

    body: Dict[str, Any] = {}
    if source_id is not None:
        body["source_id"] = source_id
    if frame_id is not None:
        body["frame_id"] = frame_id
    if timestamp is not None:
        body["timestamp"] = float(timestamp)
    body["datum"] = datum_block
    body["goal_id"] = None if goal_id is None else _id(goal_id)
    body["open_count"] = open_count
    body["targets"] = [
        {
            "id": _id(target.id),
            **({"class_name": class_name} if class_name is not None else {}),
            "lat": target.latitude_deg,
            "lon": target.longitude_deg,
            "x": target.x,
            "y": target.y,
            "confidence": target.confidence,
            "collected": target.collected,
            "is_goal": target.is_goal,
            "last_seen": target.last_seen,
        }
        for target in targets
    ]
    return json.dumps(body)


# ---------------------------------------------------------------------------
# 4: /octopus/trash_goal_done (robot -> Octopus)
# ---------------------------------------------------------------------------
def parse_goal_done(payload: Any) -> str:
    """Parse a ``trash_goal_done`` payload into a target id.

    Accepts what their own parser accepts: the bare id (``"1"``) and the JSON
    object (``{"id": 1}``). Used to verify our own emission in the offline tests
    and to consume an echo if their side ever reflects one.
    """
    if isinstance(payload, Mapping):
        # Either the std_msgs/String wrapper rosbridge delivers, or the decoded
        # object form.
        if isinstance(payload.get("data"), str):
            text = payload["data"]
        else:
            return _as_id(payload.get("id"), "trash_goal_done.id")
    elif isinstance(payload, str):
        text = payload
    else:
        raise ProtocolError(
            "MALFORMED_PAYLOAD", f"trash_goal_done is neither string nor object: {payload!r}"
        )

    stripped = text.strip()
    if not stripped:
        raise ProtocolError("MALFORMED_PAYLOAD", "trash_goal_done is empty")
    if stripped.startswith("{"):
        try:
            body = json.loads(stripped)
        except ValueError as exc:
            raise ProtocolError("MALFORMED_JSON", f"trash_goal_done is not JSON: {exc}") from exc
        return _as_id(_require_mapping(body, "trash_goal_done").get("id"), "trash_goal_done.id")
    return _as_id(stripped, "trash_goal_done")


def build_goal_done(target_id: str, as_json_object: bool = False) -> str:
    """Emit ``trash_goal_done``.

    Bare id by default: it is the form their doc shows and the form their node
    was written against. ``as_json_object`` exists only so the JSON variant can
    be exercised against their parser once, rather than discovered to be broken
    during a demo.

    CAUTION - semantics: to the Octopus this means *collected*, and it is the
    only thing that advances their goal. Send it on reach alone and we mark
    trash we failed to grab. Their protocol has no way to say "I could not do
    this", so a failure must NOT be reported as done; it is blacklisted locally
    and surfaced instead (proposal item 2).
    """
    normalized = _as_id(target_id, "target_id")
    if as_json_object:
        return json.dumps({"id": normalized})
    return normalized


# ---------------------------------------------------------------------------
# 5: /octopus/flight_camera_transform/status (JSON in String)
# ---------------------------------------------------------------------------
#: The state their transform reports once it actually is one. Compared exactly;
#: every other value means "do not trust anything derived from their frame".
TRANSFORM_STATE_READY = "ready"

#: How far ``indoor_static_yaw_zero_rad`` must move before we call it a NEW
#: lock rather than the same lock republished.
#:
#: UNMEASURED, AND NOT A PHYSICAL THRESHOLD. Nobody has measured how much that
#: value moves across a restart of their transform node - there is no run of
#: ours to measure it in. Its only job is to absorb representation noise: the
#: value travels as a JSON decimal literal, and a change of formatting on their
#: side (or a re-serialisation through float32 anywhere in the chain) must not
#: read as a re-lock. 1e-6 rad is 6e-5 deg, which is orders of magnitude below
#: any re-lock the drone could physically produce and orders of magnitude above
#: double round-trip noise. Same reasoning and same number as
#: ``geodesy.BOOTSTRAP_MATCH_TOLERANCE_DEG``.
#:
#: What it CANNOT do is detect a re-lock that happens to land on the same yaw -
#: a restart with the drone in the same pose is invisible to this test, and no
#: epsilon fixes that. See the open questions in the handover.
DEFAULT_RELOCK_EPSILON_RAD = 1e-6


@dataclass(frozen=True)
class TransformStatus:
    """One sample of ``/octopus/flight_camera_transform/status`` (~1 Hz).

    THIS IS AN INVALIDATION SIGNAL, NOT A CALIBRATION. Their map ``+y`` is the
    drone's heading at startup (Q1 = option B, answered 2026-08-21), so a
    ``their-map -> our-map`` rotation has always been required and this topic
    does not supply it: that rotation comes from where the robot is placed
    relative to the drone. What this topic supplies is the rotation's
    *lifetime*. ``indoor_static_yaw_zero_rad`` is the drone's PX4 yaw locked at
    startup, and a restart of their transform node re-locks it to a new value -
    at which point an alignment measured earlier silently stopped being true.
    Reading this topic is the only way we can find that out. The full reasoning
    is in the docstring of :func:`geodesy.latlon_to_map`.

    Every field is carried through as it arrived. ``state`` and
    ``transform_ready`` are theirs and are not reinterpreted here; a missing
    ``transform_ready`` is ``None`` (unavailable) and never ``False``.
    """

    #: Their word, verbatim. ``"ready"`` only once the transform actually is.
    state: str
    #: Their bool, verbatim. ``None`` when the key was absent - NOT ``False``.
    transform_ready: Optional[bool]
    transform_mode: Optional[str]
    #: The drone's PX4 yaw LOCKED AT STARTUP. ``None`` until the lock exists -
    #: their ``null``, which must never be read as 0.0.
    yaw_zero_rad: Optional[float]
    map_yaw_offset_rad: Optional[float]
    origin_x: Optional[float]
    origin_y: Optional[float]

    @property
    def locked(self) -> bool:
        """Does a startup yaw lock exist at all?"""
        return self.yaw_zero_rad is not None and math.isfinite(self.yaw_zero_rad)

    @property
    def ready(self) -> bool:
        return self.state == TRANSFORM_STATE_READY

    @property
    def align_angle_rad(self) -> Optional[float]:
        """``map_yaw_offset - yaw_zero``, or ``None`` when it is not derivable.

        Their own derived quantity: in the sample they sent from a running
        system, ``1.57079632679 - (-3.06995218682264) = 4.6407 rad = 265.9 deg``.
        ``None`` rather than a number whenever either angle is absent or
        non-finite - a fabricated 0.0 here would read as "their frame and ours
        agree", which is the one wrong answer this whole topic exists to
        prevent (FR-12 item 8).
        """
        if self.yaw_zero_rad is None or self.map_yaw_offset_rad is None:
            return None
        if not (math.isfinite(self.yaw_zero_rad) and math.isfinite(self.map_yaw_offset_rad)):
            return None
        return self.map_yaw_offset_rad - self.yaw_zero_rad


def _as_optional_bool(value: Any, what: str) -> Optional[bool]:
    """Strict: a real bool, or absent. No coercion, deliberately.

    ``_as_bool`` above takes a ``default`` because the ``trash_gps`` flags have
    one that is correct (an absent ``collected`` means not collected). This
    field has no such default: ``transform_ready`` missing means we do not know,
    and answering "not ready" for it would be an invented measurement while
    answering "ready" would be worse.
    """
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is not a bool: {value!r}")


def _as_finite_or_none(value: Any, what: str) -> Optional[float]:
    """``None`` stays ``None``; a number must be finite.

    A non-finite angle is a rejection rather than a quiet "not locked": JSON
    proper cannot carry ``NaN``, so one arriving means something in their chain
    produced it, and swallowing that as "no lock yet" would hide a fault behind
    a state that looks ordinary.
    """
    number = _as_float(value, what, allow_none=True)
    if number is not None and not math.isfinite(number):
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what} is not finite: {value!r}")
    return number


def parse_transform_status(payload: Any) -> TransformStatus:
    """Parse ``/octopus/flight_camera_transform/status``.

    Accepts a ``std_msgs/String`` in rosbridge JSON form, a bare JSON string, or
    an already-decoded mapping - the same three forms :func:`parse_trash_gps`
    takes, and for the same reason.

    Key names are parsed strictly, as everywhere else in this module (FR-12
    item 6): the ``indoor_static_*`` spellings are theirs and a rename on their
    side must surface as a rejection, not be absorbed by a tolerant reader.
    ``state`` is required - a status message that does not say its state is
    malformed. Everything else may be absent and becomes ``None``.
    """
    if isinstance(payload, Mapping) and "data" in payload and isinstance(payload["data"], str):
        text: Optional[str] = payload["data"]
    elif isinstance(payload, str):
        text = payload
    else:
        text = None

    if text is not None:
        try:
            body: Any = json.loads(text)
        except (ValueError, TypeError) as exc:
            raise ProtocolError(
                "MALFORMED_JSON", f"flight_camera_transform/status is not JSON: {exc}"
            ) from exc
    else:
        body = payload
    mapping = _require_mapping(body, "flight_camera_transform/status")

    what = "flight_camera_transform/status"
    state_field = _require_key(mapping, "state", what)
    if not isinstance(state_field, str):
        raise ProtocolError("MALFORMED_PAYLOAD", f"{what}.state is not a string: {state_field!r}")

    mode_field = mapping.get("transform_mode")
    if mode_field is not None and not isinstance(mode_field, str):
        raise ProtocolError(
            "MALFORMED_PAYLOAD", f"{what}.transform_mode is not a string: {mode_field!r}"
        )

    return TransformStatus(
        state=state_field,
        transform_ready=_as_optional_bool(
            mapping.get("transform_ready"), f"{what}.transform_ready"
        ),
        transform_mode=mode_field,
        # Their `null` means NOT LOCKED and reaches the dataclass as None. It is
        # read with `.get`, not `_require_key`: the key is genuinely absent in
        # some of their modes, and that is the same information as `null`.
        yaw_zero_rad=_as_finite_or_none(
            mapping.get("indoor_static_yaw_zero_rad"), f"{what}.indoor_static_yaw_zero_rad"
        ),
        map_yaw_offset_rad=_as_finite_or_none(
            mapping.get("indoor_static_map_yaw_offset_rad"),
            f"{what}.indoor_static_map_yaw_offset_rad",
        ),
        origin_x=_as_finite_or_none(
            mapping.get("indoor_static_origin_x"), f"{what}.indoor_static_origin_x"
        ),
        origin_y=_as_finite_or_none(
            mapping.get("indoor_static_origin_y"), f"{what}.indoor_static_origin_y"
        ),
    )


def build_transform_status(
    state: str,
    transform_ready: Optional[bool] = None,
    yaw_zero_rad: Optional[float] = None,
    map_yaw_offset_rad: Optional[float] = None,
    origin_x: Optional[float] = None,
    origin_y: Optional[float] = None,
    transform_mode: Optional[str] = None,
) -> str:
    """Emit the ``flight_camera_transform/status`` JSON string.

    For the offline tests and the fake Octopus only - this is their topic and we
    never publish on it.
    """
    return json.dumps(
        {
            "state": state,
            "transform_ready": transform_ready,
            "transform_mode": transform_mode,
            "indoor_static_origin_x": origin_x,
            "indoor_static_origin_y": origin_y,
            "indoor_static_map_yaw_offset_rad": map_yaw_offset_rad,
            "indoor_static_yaw_zero_rad": yaw_zero_rad,
        }
    )


@dataclass(frozen=True)
class TransformLockUpdate:
    """Outcome of feeding one sample into :class:`TransformLockTracker`."""

    #: Does the sample carry a usable startup yaw lock?
    locked: bool
    #: First lock we have ever seen. Not a re-lock: there was nothing to lose.
    first_lock: bool
    #: THE EVENT. A new lock replaced a different earlier one, so any alignment
    #: measured against the earlier one is dead.
    relocked: bool
    previous_yaw_zero_rad: Optional[float]
    yaw_zero_rad: Optional[float]
    previous_align_angle_rad: Optional[float]
    align_angle_rad: Optional[float]
    reason: str = ""

    @property
    def align_angle_delta_rad(self) -> Optional[float]:
        if self.align_angle_rad is None or self.previous_align_angle_rad is None:
            return None
        return self.align_angle_rad - self.previous_align_angle_rad


class TransformLockTracker:
    """Holds the last seen startup yaw lock and detects a re-lock.

    REPORT ONLY, BY DESIGN. This class counts and describes; it decides
    nothing. What a re-lock should *cause* - cancel, disarm, refuse validation -
    is a user decision that has not been taken, and the project's pattern
    (SAFETY.md F-40) is to obtain the evidence first and decide afterwards. So
    there is no gate here, no ``dispatch_blocker`` and no trigger, deliberately,
    and adding one is a decision and not a fix.

    Two things about *when* a lock counts, both deliberate:

    * A sample without a lock (their ``null``) does NOT clear the last known
      lock. Their node publishes at 1 Hz through its own startup, so a lock is
      routinely followed by a gap and then by the same lock again; forgetting on
      every gap would make the next sample look like a first lock and the
      re-lock would go unreported - which is the whole failure this exists to
      catch.
    * ``state`` is NOT part of the test. A non-null ``yaw_zero`` is a lock by
      their own statement, and requiring ``state == "ready"`` as well would let a
      re-lock that happens while the transform is briefly not ready pass
      unnoticed. ``state`` is reported separately, and is what a *consumer*
      checks before trusting a value.
    """

    def __init__(self, relock_epsilon_rad: float = DEFAULT_RELOCK_EPSILON_RAD) -> None:
        self.relock_epsilon_rad = float(relock_epsilon_rad)
        self._last: Optional[TransformStatus] = None
        self._last_locked: Optional[TransformStatus] = None
        self.samples = 0
        self.relocks = 0
        self.last_relock_from_rad: Optional[float] = None
        self.last_relock_to_rad: Optional[float] = None
        self.last_relock_align_delta_rad: Optional[float] = None

    # -- state ----------------------------------------------------------
    @property
    def status(self) -> Optional[TransformStatus]:
        """The most recent sample, whatever it said. ``None`` before the first."""
        return self._last

    @property
    def locked_status(self) -> Optional[TransformStatus]:
        """The most recent sample that actually carried a lock."""
        return self._last_locked

    @property
    def yaw_zero_rad(self) -> Optional[float]:
        return None if self._last_locked is None else self._last_locked.yaw_zero_rad

    @property
    def align_angle_rad(self) -> Optional[float]:
        return None if self._last_locked is None else self._last_locked.align_angle_rad

    # -- ingest ---------------------------------------------------------
    def update(self, status: TransformStatus) -> TransformLockUpdate:
        previous = self._last_locked
        self._last = status
        self.samples += 1

        if not status.locked:
            return TransformLockUpdate(
                locked=False,
                first_lock=False,
                relocked=False,
                previous_yaw_zero_rad=None if previous is None else previous.yaw_zero_rad,
                yaw_zero_rad=None,
                previous_align_angle_rad=None if previous is None else previous.align_angle_rad,
                align_angle_rad=None,
                reason="NO_LOCK",
            )

        self._last_locked = status
        if previous is None:
            return TransformLockUpdate(
                locked=True,
                first_lock=True,
                relocked=False,
                previous_yaw_zero_rad=None,
                yaw_zero_rad=status.yaw_zero_rad,
                previous_align_angle_rad=None,
                align_angle_rad=status.align_angle_rad,
                reason="FIRST_LOCK",
            )

        assert status.yaw_zero_rad is not None and previous.yaw_zero_rad is not None
        moved = abs(status.yaw_zero_rad - previous.yaw_zero_rad) > self.relock_epsilon_rad
        update = TransformLockUpdate(
            locked=True,
            first_lock=False,
            relocked=moved,
            previous_yaw_zero_rad=previous.yaw_zero_rad,
            yaw_zero_rad=status.yaw_zero_rad,
            previous_align_angle_rad=previous.align_angle_rad,
            align_angle_rad=status.align_angle_rad,
            reason="RELOCK" if moved else "",
        )
        if moved:
            self.relocks += 1
            self.last_relock_from_rad = previous.yaw_zero_rad
            self.last_relock_to_rad = status.yaw_zero_rad
            self.last_relock_align_delta_rad = update.align_angle_delta_rad
        return update


# ---------------------------------------------------------------------------
# Proposed: /octopus/devices/gripperx/status (robot -> Octopus)
# ---------------------------------------------------------------------------
def build_device_status(
    latitude_deg: Optional[float],
    longitude_deg: Optional[float],
    map_x: Optional[float],
    map_y: Optional[float],
    yaw_deg: Optional[float],
    nav_state: str,
    active_goal_id: Optional[str],
    armed: bool,
    link_ok: bool,
    stamp_sec: float,
    battery_status: str = "unavailable",
    battery_reason: str = "NO_SENSOR_INSTALLED",
    battery_percent: Optional[float] = None,
    *,
    pose_status: str = "available",
    pose_reason: str = "",
    latlon_status: str = "available",
    latlon_reason: str = "",
    nav_state_reason: str = "",
    last_disarm_trigger: str = "",
    arming_seconds_remaining: Optional[float] = None,
    teleop_mode: Optional[str] = None,
    speed_mps: Optional[float] = None,
    link_last_message_age_sec: Optional[float] = None,
    link_reconnects: Optional[int] = None,
    counters: Optional[Mapping[str, Any]] = None,
    blacklist: Sequence[str] = (),
    octopus_transform: Optional[Mapping[str, Any]] = None,
) -> str:
    """Emit the proposed robot telemetry payload.

    NOT yet consumed by the Octopus - proposal item 1, pending their bridge
    node. Shipping it from day one is what turns their ``goal_selection:
    nearest`` from "nearest to the datum" into "nearest to the robot".

    EVERY BLOCK REPORTS ITS OWN AVAILABILITY (FR-12 item 8). ``None`` becomes
    JSON ``null`` and is accompanied by a ``status``/``reason`` pair, so a
    consumer can never read an unmeasurable value as zero. The pose and the
    lat/lon carry *separate* statuses on purpose: the map pose can be perfectly
    known while the datum is missing or still their bootstrap fallback, and a
    lat/lon derived from that would be a fabricated position on their map.

    The battery block is structurally honest rather than a fabricated number:
    there is no shunt, no divider and no ADC channel in the ESP32 pin map, so it
    reports ``unavailable`` permanently until HWR-21 provides a measurement
    chain. A plausible percentage here would be a lie the dashboard displays.

    Telemetry is OUTBOUND ONLY and carries no control semantics - nothing the
    Octopus sends in response may change our state (SR-15 rule 3).
    """
    return json.dumps(
        {
            "device_id": "gripperx",
            "stamp": stamp_sec,
            "pose": {
                "status": pose_status,
                "reason": pose_reason,
                "lat": latitude_deg,
                "lon": longitude_deg,
                "latlon_status": latlon_status,
                "latlon_reason": latlon_reason,
                "x": map_x,
                "y": map_y,
                "yaw_deg": yaw_deg,
                "speed_mps": speed_mps,
            },
            "nav_state": nav_state,
            "nav_state_reason": nav_state_reason,
            "active_goal_id": active_goal_id,
            "armed": bool(armed),
            "arming_seconds_remaining": arming_seconds_remaining,
            "last_disarm_trigger": last_disarm_trigger,
            # Observed, never written by us (SR-15 rule 6).
            "teleop_mode": teleop_mode,
            "link_ok": bool(link_ok),
            "link": {
                "last_message_age_sec": link_last_message_age_sec,
                "reconnects": link_reconnects,
            },
            "counters": dict(counters or {}),
            "blacklist": list(blacklist),
            "battery": {
                "status": battery_status,
                "reason": battery_reason,
                "percent": battery_percent,
            },
            # What we observe about THEIR frame lock, mirrored back to them
            # (2026-08-21). Not a request and not a complaint: it is the one
            # place either side can see that both agree on which lock is live,
            # and it costs one dict that the link node already holds. `null`
            # when we are not subscribed, which is a different statement from
            # "no lock" and is spelled differently on purpose (FR-12 item 8).
            "octopus_transform": None if octopus_transform is None else dict(octopus_transform),
        }
    )
