"""Flat-earth conversion between the Octopus's WGS84 values and the map frame.

THIS IS DELIBERATELY NOT AN ACCURATE PROJECTION, AND MUST NOT BE "FIXED".

The Octopus demo is indoor fake GPS. Neither side has a receiver; the lat/lon
numbers are constructed and are only meaningful because the drone, the
dashboard map and the robot expand them with the *same* arithmetic. The Octopus
side (``trash_gps_goal_node.py``, and ``localToLatLng()`` in the dashboard's
``live_data.js``) uses::

    lat = datum_lat + y / 111320
    lon = datum_lon + x / (111320 * cos(datum_lat))

so we use exactly its inverse::

    y = (lat - datum_lat) * 111320
    x = (lon - datum_lon) * 111320 * cos(datum_lat)

An exact WGS84/ENU conversion would be *more accurate* and therefore *wrong*:
it would put our metres and their metres in different places. Over the ~4.5 x
3.3 m working patch the difference is orders of magnitude below the detector's
own accuracy (their position covariance is a hardcoded sigma-0.5 m estimate).

``navsat_transform_node`` is rejected for this path even though it is
configured in ``gripperx_localization/config/localization.yaml``: it needs
odometry AND IMU AND GPS before its transform is valid and would silently emit
zeros in the twin, its output frame is ``odom`` (so a goal's meaning would
shift on every slam_toolbox loop closure), it hides a UTM projection plus
declination/yaw offsets, and its datum is runtime-mutable by anyone via the
``/datum`` service. It keeps its real job for the day a receiver is fitted.

Pure module: no rclpy, no ROS message types.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional, Tuple

# Matches trash_gps_goal_node.py and live_data.js. Changing this number breaks
# the agreement with the Octopus, not just our accuracy.
METERS_PER_DEGREE_LAT = 111320.0

# The Octopus bootstrap fallback (Garching). It appears whenever the dashboard
# has not supplied a real Eve position, so a goal converted against it is
# meaningless - dispatch on this datum is refused.
BOOTSTRAP_FALLBACK_LAT = 48.2513611
BOOTSTRAP_FALLBACK_LON = 11.6359722

# Tolerance for recognising the bootstrap fallback. Their value travels through
# JSON as a decimal literal, so exact float equality is not guaranteed; 1e-6 deg
# is ~0.11 m, far tighter than any real datum drag on a 4.5 m field.
BOOTSTRAP_MATCH_TOLERANCE_DEG = 1e-6


class GeodesyError(Exception):
    """Raised with a machine-readable ``reason`` code."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


@dataclass(frozen=True)
class Datum:
    """Origin of the conversion.

    ``from_topic`` records provenance, because the two sources are not
    interchangeable: a datum received on ``/octopus/fake_eve_gps_start`` is the
    shared truth, while the configured fallback is a local guess that only
    exists so the node can start up and preview. Which one is live is also
    reported by the Octopus itself as ``datum.from_topic`` in the
    ``trash_gps`` JSON.
    """

    latitude_deg: float
    longitude_deg: float
    from_topic: bool = False
    #: Monotonic-ish timestamp of receipt, in seconds. ``None`` for config.
    stamp_sec: Optional[float] = None

    def with_stamp(self, stamp_sec: float) -> "Datum":
        return replace(self, stamp_sec=stamp_sec)


def is_finite_latlon(latitude_deg: float, longitude_deg: float) -> bool:
    return math.isfinite(latitude_deg) and math.isfinite(longitude_deg)


def is_in_range_latlon(latitude_deg: float, longitude_deg: float) -> bool:
    return -90.0 <= latitude_deg <= 90.0 and -180.0 <= longitude_deg <= 180.0


def is_bootstrap_fallback(
    datum: Datum, tolerance_deg: float = BOOTSTRAP_MATCH_TOLERANCE_DEG
) -> bool:
    """True when ``datum`` is the Octopus's Garching bootstrap value."""
    return (
        abs(datum.latitude_deg - BOOTSTRAP_FALLBACK_LAT) <= tolerance_deg
        and abs(datum.longitude_deg - BOOTSTRAP_FALLBACK_LON) <= tolerance_deg
    )


def _cos_datum_lat(datum: Datum) -> float:
    # cos(datum_lat), NOT cos(target_lat). The forward formula on their side
    # writes "cos lat"; over the working patch the two differ by ~1e-6 m, but
    # using the datum's cosine makes our forward and inverse exact inverses of
    # each other, which is what the frozen-vector test locks down.
    return math.cos(math.radians(datum.latitude_deg))


def latlon_to_map(
    datum: Datum, latitude_deg: float, longitude_deg: float
) -> Tuple[float, float]:
    """WGS84 -> map metres. Returns ``(x, y)``.

    ``+y`` is north and ``+x`` is east *by this arithmetic*. That question is
    now ANSWERED, and the answer is that the labels are wrong.

    ANSWERED 2026-08-21 BY THE OCTOPUS TEAM: **Q1 is option B.** Their map
    ``+y`` is the DRONE'S HEADING AT STARTUP, not north. Their
    ``octopus_to_robot_interface.md`` said ``x = Ost, y = Nord`` and was simply
    wrong; ``Octopus/README.md`` was right, and they are correcting the
    document rather than the behaviour. In the run they measured,
    ``align_angle = map_yaw_offset - yaw_zero = 1.57080 - (-3.06995)
    = 4.6408 rad = 265.9 deg``.

    **WHY THIS FUNCTION STILL DOES NOT ROTATE, AND MUST NOT.** It is the exact
    inverse of their publisher's arithmetic, so it recovers THEIR map ``(x, y)``
    faithfully - which is all it claims to do. Rotating here would break the
    agreement it exists to keep.

    **WHERE THE ROTATION ACTUALLY BELONGS, AND WHAT IT IS.** What the robot
    needs is a goal in OUR map frame, and our map frame is anchored on the
    robot's own start pose - an arbitrary orientation. So a rotation
    THEIR-MAP -> OUR-MAP has always been required, under option A no less than
    under B, and neither option supplies it: it has to be established once, from
    the placement of the robot relative to the drone, and confirmed by the
    stage-2 empirical check while disarmed.

    **WHAT OPTION B CHANGES IS NOT THE ROTATION BUT ITS LIFETIME.** Under B
    their frame's orientation is re-locked every time their transform node
    restarts, so a previously established alignment silently STOPS BEING TRUE.
    They publish the lock at 1 Hz on ``/octopus/flight_camera_transform/status``
    (``std_msgs/String`` JSON: ``state``, ``indoor_static_yaw_zero_rad``,
    ``indoor_static_map_yaw_offset_rad``, ``indoor_static_origin_x/y``).
    **That topic is therefore an INVALIDATION SIGNAL, not the rotation** - it
    tells us when our measured alignment died, which is the part we could not
    otherwise detect. ``indoor_static_yaw_zero_rad`` is ``null`` and ``state``
    is not ``"ready"`` until the lock exists; both must be checked before any
    value from it is trusted.

    We do not subscribe to that topic yet. It is a fifth ingress topic and it is
    owed.
    """
    if not is_finite_latlon(latitude_deg, longitude_deg):
        raise GeodesyError("LATLON_NOT_FINITE", f"lat={latitude_deg} lon={longitude_deg}")
    if not is_in_range_latlon(latitude_deg, longitude_deg):
        raise GeodesyError("LATLON_OUT_OF_RANGE", f"lat={latitude_deg} lon={longitude_deg}")
    y = (latitude_deg - datum.latitude_deg) * METERS_PER_DEGREE_LAT
    x = (longitude_deg - datum.longitude_deg) * METERS_PER_DEGREE_LAT * _cos_datum_lat(datum)
    return x, y


def map_to_latlon(datum: Datum, x: float, y: float) -> Tuple[float, float]:
    """Map metres -> WGS84. Exact inverse of :func:`latlon_to_map`.

    Used for telemetry (we report our pose in both forms) and for the stage-2
    empirical check, which compares a known map offset against the lat/lon the
    Octopus sends for the same physical object.
    """
    if not (math.isfinite(x) and math.isfinite(y)):
        raise GeodesyError("XY_NOT_FINITE", f"x={x} y={y}")
    latitude_deg = datum.latitude_deg + y / METERS_PER_DEGREE_LAT
    cos_lat = _cos_datum_lat(datum)
    if cos_lat == 0.0:
        raise GeodesyError("DEGENERATE_DATUM", "cos(datum_lat) == 0 (pole)")
    longitude_deg = datum.longitude_deg + x / (METERS_PER_DEGREE_LAT * cos_lat)
    return latitude_deg, longitude_deg


def datum_offset_m(old: Datum, new: Datum) -> float:
    """Distance in metres between two datums, in the *old* datum's expansion.

    Deliberately measured in the old frame: the question this answers is "how
    far did everything I already resolved move?", and those poses were resolved
    with the old datum.
    """
    dx, dy = latlon_to_map(old, new.latitude_deg, new.longitude_deg)
    return math.hypot(dx, dy)


@dataclass(frozen=True)
class DatumUpdate:
    """Outcome of feeding a datum into :class:`DatumTracker`."""

    accepted: bool
    changed: bool
    jump_m: float
    exceeds_warn: bool
    reason: str = ""


class DatumTracker:
    """Holds the live datum and detects jumps.

    Policy, straight from the design: the datum moves when the operator drags
    the Eve marker on the dashboard, and every target moves with it. On a jump
    larger than ``jump_warn_m`` the caller must cancel the active goal, log
    WARN, re-resolve and re-preview - continuing to a pose that now means
    something else is the failure this class exists to prevent.
    """

    def __init__(
        self,
        fallback: Optional[Datum] = None,
        jump_warn_m: float = 0.25,
        refuse_bootstrap_fallback: bool = True,
        bootstrap_tolerance_deg: float = BOOTSTRAP_MATCH_TOLERANCE_DEG,
    ) -> None:
        if fallback is not None and fallback.from_topic:
            raise ValueError("the configured fallback datum must have from_topic=False")
        self._fallback = fallback
        self._topic_datum: Optional[Datum] = None
        self.jump_warn_m = float(jump_warn_m)
        self.refuse_bootstrap_fallback = bool(refuse_bootstrap_fallback)
        self.bootstrap_tolerance_deg = float(bootstrap_tolerance_deg)
        self.last_jump_m = 0.0

    # -- state ----------------------------------------------------------
    @property
    def datum(self) -> Optional[Datum]:
        """The datum in force: the topic value if we ever got one, else the
        configured fallback, else ``None``."""
        return self._topic_datum if self._topic_datum is not None else self._fallback

    @property
    def has_topic_datum(self) -> bool:
        return self._topic_datum is not None

    def age_sec(self, now_sec: float) -> float:
        d = self.datum
        if d is None or d.stamp_sec is None:
            return -1.0
        return max(0.0, now_sec - d.stamp_sec)

    # -- ingest ---------------------------------------------------------
    def update(self, datum: Datum) -> DatumUpdate:
        """Feed a datum received on the topic."""
        if not is_finite_latlon(datum.latitude_deg, datum.longitude_deg):
            return DatumUpdate(False, False, 0.0, False, "LATLON_NOT_FINITE")
        if not is_in_range_latlon(datum.latitude_deg, datum.longitude_deg):
            return DatumUpdate(False, False, 0.0, False, "LATLON_OUT_OF_RANGE")

        previous = self._topic_datum
        # Accepted even when it is the bootstrap fallback: knowing that the
        # Octopus is still on its bootstrap value is exactly what lets us refuse
        # dispatch with a specific reason instead of silently going nowhere.
        self._topic_datum = replace(datum, from_topic=True)

        if previous is None:
            self.last_jump_m = 0.0
            return DatumUpdate(True, False, 0.0, False)

        jump = datum_offset_m(previous, self._topic_datum)
        self.last_jump_m = jump
        changed = jump > 0.0
        return DatumUpdate(True, changed, jump, jump > self.jump_warn_m)

    # -- gating ---------------------------------------------------------
    def dispatch_blocker(self) -> str:
        """Empty string when goals may be resolved and dispatched, otherwise a
        machine-readable reason. Checked before every conversion."""
        d = self.datum
        if d is None:
            return "NO_DATUM"
        if self.refuse_bootstrap_fallback and is_bootstrap_fallback(
            d, self.bootstrap_tolerance_deg
        ):
            return "BOOTSTRAP_FALLBACK_DATUM"
        return ""

    def require_datum(self) -> Datum:
        blocker = self.dispatch_blocker()
        if blocker:
            raise GeodesyError(blocker)
        datum = self.datum
        assert datum is not None  # guaranteed by dispatch_blocker
        return datum
