"""The shared reference line between the Octopus and GripperX, as a frame.

THE ONE PLACE THE LINE-FRAME CONVENTION IS WRITTEN DOWN IN CODE. The interface
document for the Octopus team is ``documentation/OCTOPUS_LINE_CALIBRATION.md``;
the two must say the same thing.

Two vertical posts of the drone's aluminium frame cut the LiDAR (light detection
and ranging) scan plane. The operator clicks them in RViz (the ROS 3D viewer),
post A first, post B second, in our ``map`` frame. From those two points:

* origin  = midpoint of A and B, z = 0 (a clicked z is ignored)
* +x      = the unit vector from A to B
* +y      = +x rotated 90 deg COUNTER-CLOCKWISE seen from above, i.e. to the
            LEFT of someone standing at A and looking at B
* +z      = up. Right-handed (REP-103, the ROS convention for coordinate frames)
* yaw     = the angle of +x in ``map``, counter-clockwise positive, radians
* length  = |AB|, the LiDAR-measured reference length L in metres. We never
            scale by it (the LiDAR is metric); it is published so the operator
            can enter exactly that number on the drone side.

The Octopus expresses its goals in this frame: their datum is the line midpoint
and their flat-earth "x"/"y" (see :mod:`geodesy`) are this frame's +x/+y. So the
chain for a goal is ``lat/lon -> geodesy.latlon_to_map -> (x, y) in the LINE
frame -> line_to_map -> our map``, and the reverse for telemetry. The geodesy
arithmetic is untouched; this module only adds the rigid transform after it.

Pure module: no rclpy, no ROS message types. The status the calibration node
publishes is JSON built and parsed here, so the node and the gateway cannot
disagree about its shape.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Sequence, Tuple

from .geodesy import Datum, latlon_to_map, map_to_latlon

DEFAULT_FRAME_ID = "octopus_line"

#: THE NUMERIC-DEGENERACY FLOOR - NOT A PHYSICAL ESTIMATE. Below it the
#: direction A->B is undefined at the precision the calibration is reported in
#: (the length is displayed and logged to the millimetre, so two clicks closer
#: than this read as "0.000 m" and their direction as noise). It says nothing
#: about how far apart the real posts are: that is the expected-length RANGE
#: (``expected_length_min_m`` / ``expected_length_max_m`` of the calibration
#: node, user-stated). The floor only guards the arithmetic for a caller that
#: passes no range.
NUMERIC_DEGENERACY_FLOOR_M = 1e-3

# Status states, as they appear in the JSON.
STATE_CALIBRATED = "calibrated"
STATE_WAITING_FOR_A = "waiting_for_a"
STATE_WAITING_FOR_B = "waiting_for_b"

# Reason codes. Machine-readable, like every refusal in this package.
NOT_CALIBRATED_YET = "NOT_CALIBRATED_YET"
RESET = "RESET"
RECALIBRATING = "RECALIBRATING"
MAP_SESSION_CHANGED = "MAP_SESSION_CHANGED"
NO_MAP_SESSION = "NO_MAP_SESSION"
WRONG_FRAME = "WRONG_FRAME"
DEGENERATE = "DEGENERATE"
#: |AB| outside the expected post-spacing range: a wrong post or a stray click.
LENGTH_OUT_OF_RANGE = "LENGTH_OUT_OF_RANGE"
#: The calibration node has no usable expected-length range configured, so it
#: cannot tell a right pair from a wrong one and refuses every pair.
NO_LENGTH_RANGE = "NO_LENGTH_RANGE"
#: No publisher on the map topic for longer than the grace time (or at all,
#: while that grace time is unmeasured): the map frame is no longer shown to be
#: alive, which is treated as its loss.
MAP_SESSION_LOST = "MAP_SESSION_LOST"
NOT_FINITE = "NOT_FINITE"
STATUS_UNPARSEABLE = "STATUS_UNPARSEABLE"
STATUS_INCONSISTENT = "STATUS_INCONSISTENT"

#: Agreement demanded between the derived values a status carries and the ones
#: re-derived here from its A and B. JSON carries floats as shortest round-trip
#: decimals, so an honest publisher agrees to round-off; any real disagreement
#: means it computes differently from this module (version skew), and the
#: status is refused rather than believed.
_CONSISTENCY_TOLERANCE = 1e-9


class LineCalibrationError(Exception):
    """Raised with a machine-readable ``reason`` code."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


@dataclass(frozen=True)
class LineCalibration:
    """One completed calibration: two posts, and the frame they define.

    Only A and B are stored. Origin, yaw and length are derived from them on
    every access, so they cannot drift apart from the points they came from.
    """

    ax: float
    ay: float
    bx: float
    by: float
    frame_id: str = DEFAULT_FRAME_ID
    parent_frame_id: str = "map"
    #: Identifies the calibration node PROCESS. Its counter restarts at 1 with
    #: the process, so (session, calibration_id) is the identity, never the id
    #: alone.
    session: str = ""
    calibration_id: int = 0
    stamp_sec: float = 0.0
    #: The publishers of the map topic when the posts were clicked. The map
    #: frame the clicks are expressed in lives exactly as long as those
    #: publishers do - see :func:`map_session_key`.
    map_session: Tuple[str, ...] = field(default_factory=tuple)

    @property
    def origin(self) -> Tuple[float, float]:
        return (0.5 * (self.ax + self.bx), 0.5 * (self.ay + self.by))

    @property
    def yaw_rad(self) -> float:
        return math.atan2(self.by - self.ay, self.bx - self.ax)

    @property
    def length_m(self) -> float:
        return math.hypot(self.bx - self.ax, self.by - self.ay)

    @property
    def identity(self) -> Tuple[str, int]:
        return (self.session, int(self.calibration_id))

    def line_to_map(self, x: float, y: float) -> Tuple[float, float]:
        """Line frame -> map: ``origin + R(yaw) * (x, y)``."""
        ox, oy = self.origin
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return (ox + c * x - s * y, oy + s * x + c * y)

    def map_to_line(self, x: float, y: float) -> Tuple[float, float]:
        """Map -> line frame: ``R(-yaw) * ((x, y) - origin)``."""
        ox, oy = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return (c * dx + s * dy, -s * dx + c * dy)

    def yaw_map_to_line(self, yaw_map_rad: float) -> float:
        return _wrap(yaw_map_rad - self.yaw_rad)

    def yaw_line_to_map(self, yaw_line_rad: float) -> float:
        return _wrap(yaw_line_rad + self.yaw_rad)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_calibration(
    a_xy: Sequence[float],
    b_xy: Sequence[float],
    length_range_m: Optional[Tuple[float, float]] = None,
    **identity: Any,
) -> LineCalibration:
    """Build the calibration from post A and post B, or raise.

    ``length_range_m`` is the expected ``(min, max)`` post spacing, inclusive.
    A pair whose length falls outside it is refused (LENGTH_OUT_OF_RANGE). The
    numeric-degeneracy floor applies whether or not a range is given.
    """
    ax, ay = float(a_xy[0]), float(a_xy[1])
    bx, by = float(b_xy[0]), float(b_xy[1])
    if not all(math.isfinite(v) for v in (ax, ay, bx, by)):
        raise LineCalibrationError(NOT_FINITE, f"A=({ax}, {ay}) B=({bx}, {by})")
    length = math.hypot(bx - ax, by - ay)
    if length < NUMERIC_DEGENERACY_FLOOR_M:
        raise LineCalibrationError(
            DEGENERATE,
            f"posts are {length:.4f} m apart - the same post clicked twice",
        )
    if length_range_m is not None:
        low, high = float(length_range_m[0]), float(length_range_m[1])
        if not (math.isfinite(low) and math.isfinite(high) and 0.0 < low <= high):
            raise LineCalibrationError(
                NO_LENGTH_RANGE, f"expected length range [{low}, {high}] m is not usable"
            )
        if not low <= length <= high:
            raise LineCalibrationError(
                LENGTH_OUT_OF_RANGE,
                f"L = {length:.3f} m is outside the expected post spacing "
                f"{low:.3f}-{high:.3f} m - a wrong post or a stray click; click A and B again",
            )
    return LineCalibration(ax=ax, ay=ay, bx=bx, by=by, **identity)


# ---------------------------------------------------------------------------
# the full boundary chain, both directions
# ---------------------------------------------------------------------------
def latlon_to_our_map(
    datum: Datum, calibration: LineCalibration, latitude_deg: float, longitude_deg: float
) -> Tuple[float, float]:
    """Octopus WGS84 -> line frame (their flat earth, unchanged) -> our map."""
    lx, ly = latlon_to_map(datum, latitude_deg, longitude_deg)
    return calibration.line_to_map(lx, ly)


def our_map_to_latlon(
    datum: Datum, calibration: LineCalibration, x: float, y: float
) -> Tuple[float, float]:
    """Our map -> line frame -> Octopus WGS84 (their flat earth, unchanged)."""
    lx, ly = calibration.map_to_line(x, y)
    return map_to_latlon(datum, lx, ly)


# ---------------------------------------------------------------------------
# the map session: how long the map frame the clicks live in is valid
# ---------------------------------------------------------------------------
def map_session_key(endpoint_gids: Sequence[Any]) -> Tuple[str, ...]:
    """Normalise the publisher GIDs (global identifiers) of the map topic.

    A restarted slam_toolbox is a new process, and a new process has new DDS
    (the ROS 2 middleware) endpoint GIDs - so the set of GIDs publishing the
    map is an identity for "this map session". Accepts raw byte sequences (as
    rclpy's ``TopicEndpointInfo.endpoint_gid``) or strings.
    """
    keys = []
    for gid in endpoint_gids:
        if isinstance(gid, str):
            keys.append(gid)
        else:
            keys.append(bytes(bytearray(int(b) & 0xFF for b in gid)).hex())
    return tuple(sorted(keys))


def map_session_verdict(recorded: Sequence[str], current: Sequence[str]) -> str:
    """``""`` while the recorded map session is the current one.

    ``NO_MAP_SESSION`` when no map publisher is visible right now - which a
    process that has only just started can see through discovery lag alone, so
    it is a BLOCK and not proof of a change. ``MAP_SESSION_CHANGED`` when a
    DIFFERENT set of publishers is visible: that is proof, and it is final.
    """
    current_t = tuple(sorted(current))
    if not current_t:
        return NO_MAP_SESSION
    if tuple(sorted(recorded)) != current_t:
        return MAP_SESSION_CHANGED
    return ""


# ---------------------------------------------------------------------------
# the status JSON (calibration node -> gateway, latched)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LineStatus:
    state: str
    reason: str = ""
    detail: str = ""
    session: str = ""
    calibration: Optional[LineCalibration] = None


def _xy(x: float, y: float) -> Mapping[str, float]:
    return {"x": float(x), "y": float(y)}


def build_status(
    *,
    state: str,
    session: str,
    frame_id: str,
    parent_frame_id: str,
    stamp_sec: float,
    calibration: Optional[LineCalibration] = None,
    reason: str = "",
    detail: str = "",
    pending_a: Optional[Tuple[float, float]] = None,
    calibration_count: int = 0,
    expected_length_m: Optional[Tuple[float, float]] = None,
    rejected: Optional[Mapping[str, Any]] = None,
    map_topic: str = "",
) -> str:
    cal = calibration
    payload = {
        "state": state,
        "reason": reason,
        "detail": detail,
        "session": session,
        "frame_id": frame_id,
        "parent_frame_id": parent_frame_id,
        "stamp": float(stamp_sec),
        # How many calibrations this process has completed. The id of the live
        # one is `calibration_id`, which is null while there is none.
        "calibration_count": int(calibration_count),
        "calibration_id": None if cal is None else int(cal.calibration_id),
        "calibrated_at": None if cal is None else float(cal.stamp_sec),
        "a": None if cal is None else _xy(cal.ax, cal.ay),
        "b": None if cal is None else _xy(cal.bx, cal.by),
        "origin": None if cal is None else _xy(*cal.origin),
        "yaw_rad": None if cal is None else cal.yaw_rad,
        "yaw_deg": None if cal is None else math.degrees(cal.yaw_rad),
        "length_m": None if cal is None else cal.length_m,
        "map_topic": map_topic,
        "map_session": [] if cal is None else list(cal.map_session),
        "pending_a": None if pending_a is None else _xy(*pending_a),
        # The user-stated plausibility range the pair was checked against.
        "expected_length_min_m": None if expected_length_m is None else float(expected_length_m[0]),
        "expected_length_max_m": None if expected_length_m is None else float(expected_length_m[1]),
        # The last REFUSED pair ({a, b, length_m}), so a rejection is visible
        # with its numbers; null once a pair has been accepted.
        "rejected": None if rejected is None else dict(rejected),
    }
    return json.dumps(payload)


def parse_status(
    text: str,
    *,
    length_range_m: Tuple[float, float],
    parent_frame_id: str,
    frame_id: str,
) -> LineStatus:
    """Parse the calibration node's status. Never raises.

    The derived values (origin, yaw, length) are RE-DERIVED here from A and B
    and compared with what the status carries, rather than trusted: the
    arithmetic exists once, in :class:`LineCalibration`, and a publisher that
    computes differently is refused instead of silently believed.

    THE CONSUMER'S OWN EXPECTATIONS DECIDE, NOT THE STATUS'S (audit H1). The
    length range and both frame names are required arguments, read by the
    consumer from its own configuration. The range a status carries is
    informational only and is ignored here: a status that declared "1-10 m"
    for itself must not be able to widen the plausibility gate, and a status
    naming ``odom`` as its parent must not be converted with as if it were
    ``map``.
    """
    try:
        data = json.loads(text)
        if not isinstance(data, dict):
            raise ValueError("status is not a JSON object")
        state = str(data.get("state", ""))
        reason = str(data.get("reason", "") or "")
        detail = str(data.get("detail", "") or "")
        session = str(data.get("session", "") or "")
        if state != STATE_CALIBRATED:
            return LineStatus(state, reason or NOT_CALIBRATED_YET, detail, session)
        got_parent, got_frame = str(data["parent_frame_id"]), str(data["frame_id"])
        if got_parent != parent_frame_id or got_frame != frame_id:
            return LineStatus(
                "invalid",
                WRONG_FRAME,
                f"status is {got_parent} -> {got_frame}, expected "
                f"{parent_frame_id} -> {frame_id}",
                session,
            )
        a, b = data["a"], data["b"]
        cal = compute_calibration(
            (float(a["x"]), float(a["y"])),
            (float(b["x"]), float(b["y"])),
            length_range_m,
            frame_id=got_frame,
            parent_frame_id=got_parent,
            session=session,
            calibration_id=int(data["calibration_id"]),
            stamp_sec=float(data.get("calibrated_at") or 0.0),
            map_session=tuple(str(g) for g in data.get("map_session") or ()),
        )
        claimed = (
            float(data["origin"]["x"]),
            float(data["origin"]["y"]),
            float(data["yaw_rad"]),
            float(data["length_m"]),
        )
    except LineCalibrationError as exc:
        return LineStatus("invalid", exc.reason, exc.detail, session)
    except (ValueError, KeyError, TypeError) as exc:
        return LineStatus("invalid", STATUS_UNPARSEABLE, repr(exc))
    derived = (cal.origin[0], cal.origin[1], cal.yaw_rad, cal.length_m)
    for name, got, want in zip(("origin.x", "origin.y", "yaw_rad", "length_m"), claimed, derived):
        diff = abs(_wrap(got - want)) if name == "yaw_rad" else abs(got - want)
        if not diff <= _CONSISTENCY_TOLERANCE:
            return LineStatus(
                "invalid",
                STATUS_INCONSISTENT,
                f"{name} in the status is {got!r}, re-derived from A/B it is {want!r}",
                session,
            )
    if not cal.map_session:
        return LineStatus("invalid", NO_MAP_SESSION, "status carries no map session", session)
    return LineStatus(STATE_CALIBRATED, "", detail, session, cal)


def rejection_text(reason: str, length_m: float, length_range_m: Tuple[float, float]) -> str:
    """THE wording of a refused pair - the log line and the RViz text are this
    same string, so the document can quote one thing (audit L6)."""
    return (
        f"REJECTED ({reason}): L = {length_m:.3f} m, expected "
        f"{length_range_m[0]:.3f}-{length_range_m[1]:.3f} m - click A and B again"
    )


# ---------------------------------------------------------------------------
# the geofence: derived from the calibration, never configured separately
# ---------------------------------------------------------------------------
def geofence_contains(cal: LineCalibration, x_map: float, y_map: float) -> bool:
    """Is a MAP point inside the square of side L centred on the line midpoint
    and aligned with the line: x, y in [-L/2, +L/2] in the line frame?

    User decision 2026-09-24: the geofence IS this square, derived from the live
    calibration, so it has one source and moves with it. Inclusive bounds, like
    the rectangle it replaces.
    """
    half = 0.5 * cal.length_m
    lx, ly = cal.map_to_line(x_map, y_map)
    return -half <= lx <= half and -half <= ly <= half


def geofence_corners_map(cal: LineCalibration) -> Tuple[Tuple[float, float], ...]:
    """The square's four corners in MAP, closed (first corner repeated)."""
    h = 0.5 * cal.length_m
    corners = [cal.line_to_map(x, y) for x, y in ((-h, -h), (h, -h), (h, h), (-h, h))]
    return tuple(corners + corners[:1])


def describe(cal: LineCalibration) -> str:
    """One operator-facing line. L to the millimetre, as it must be typed in."""
    ox, oy = cal.origin
    return (
        f"calibration #{cal.calibration_id}: L = {cal.length_m:.3f} m, "
        f"origin ({ox:.3f}, {oy:.3f}), yaw {math.degrees(cal.yaw_rad):.2f} deg "
        f"({cal.yaw_rad:.4f} rad), A ({cal.ax:.3f}, {cal.ay:.3f}), "
        f"B ({cal.bx:.3f}, {cal.by:.3f})"
    )
