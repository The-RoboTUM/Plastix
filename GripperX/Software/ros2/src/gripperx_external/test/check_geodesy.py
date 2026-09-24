#!/usr/bin/env python3
"""Verification of the flat-earth conversion and the datum handling.

Mirrors gripperx_teleop/test/check_manoeuvres.py: pure python, no ROS required.
Run from the workspace source tree:

    python3 src/gripperx_external/test/check_geodesy.py

Part 1 is a FROZEN VECTOR TABLE. Its numbers are committed, not computed by the
test, and they encode the convention agreed with the Octopus:
``METERS_PER_DEGREE_LAT = 111320.0``, ``cos(datum_lat)`` in the longitude term,
``+y`` north and ``+x`` east. A later refactor - or a well-meant "upgrade" to a
real WGS84/ENU projection - would be *more accurate* and therefore WRONG, because
the drone, the dashboard map and the robot must expand the same lat/lon into the
same metres. The table exists so that such a change fails here loudly instead of
producing a quietly displaced robot.

Part 2 checks the cardinal directions, part 3 the round trip, part 4 the
bootstrap-fallback refusal and the datum-jump detector.
"""

from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from gripperx_external.geodesy import (  # noqa: E402
    BOOTSTRAP_FALLBACK_LAT,
    BOOTSTRAP_FALLBACK_LON,
    METERS_PER_DEGREE_LAT,
    Datum,
    DatumTracker,
    GeodesyError,
    datum_offset_m,
    is_bootstrap_fallback,
    latlon_to_map,
    map_to_latlon,
)

# ---------------------------------------------------------------------------
# FROZEN VECTORS - do not regenerate. (datum_lat, datum_lon, lat, lon, x, y)
# Generated once on 2026-08-18 from the agreed formula; every literal below is
# an exact decimal so the table can be re-derived by hand if it is ever
# disputed.
# ---------------------------------------------------------------------------
FROZEN_VECTORS = (
    # datum == point: the origin must stay the origin.
    (48.2513611, 11.6359722, 48.2513611, 11.6359722, 0.0, 0.0),
    # +0.0001 deg latitude -> +11.132 m north, longitude term untouched.
    (48.2513611, 11.6359722, 48.2514611, 11.6359722, 0.0, 11.1320000004),
    # +0.0001 deg longitude at 48.25 deg -> 7.412 m east (cos(datum_lat) applied).
    (48.2513611, 11.6359722, 48.2513611, 11.6360722, 7.41239741029, 0.0),
    # A point roughly at the far corner of the demo patch.
    (48.2513611, 11.6359722, 48.25166, 11.6364, 31.7102361213, 33.2735480005),
    # Negative in both axes.
    (48.2513611, 11.6359722, 48.251, 11.6355, -35.0013405714, -40.1976519999),
    # Sub-metre, the scale the demo actually works at.
    (48.2513611, 11.6359722, 48.2513411, 11.6360022, 2.22371922314, -2.22639999992),
    # Equator: cos(datum_lat) == 1, so both axes scale identically. Catches a
    # cos() applied to the wrong term or in degrees instead of radians.
    (0.0, 0.0, 0.001, 0.0, 0.0, 111.32),
    (0.0, 0.0, 0.0, 0.001, 111.32, 0.0),
    # Southern hemisphere: cos() is even, so the longitude scale must match the
    # northern case at the same absolute latitude. Catches a sign slip.
    (-33.8688, 151.2093, -33.8687, 151.2094, 9.24307635227, 11.1320000004),
)

FROZEN_TOLERANCE_M = 1e-9
ROUNDTRIP_TOLERANCE_M = 1e-6  # the design asks for < 1 mm; we hold 1 um

_failures = []


def check(condition: bool, label: str, detail: str = "") -> None:
    mark = "PASS" if condition else "FAIL"
    print(f"  [{mark}] {label}" + (f"  ({detail})" if detail else ""))
    if not condition:
        _failures.append(label)


def main() -> int:
    print("=" * 78)
    print("Part 1 - frozen vector table (committed convention)")
    print("=" * 78)
    print(f"  METERS_PER_DEGREE_LAT = {METERS_PER_DEGREE_LAT!r}")
    check(
        METERS_PER_DEGREE_LAT == 111320.0,
        "METERS_PER_DEGREE_LAT is 111320.0",
        "must match trash_gps_goal_node.py and live_data.js",
    )
    for dlat, dlon, lat, lon, expect_x, expect_y in FROZEN_VECTORS:
        datum = Datum(dlat, dlon)
        x, y = latlon_to_map(datum, lat, lon)
        ok = (
            abs(x - expect_x) <= FROZEN_TOLERANCE_M
            and abs(y - expect_y) <= FROZEN_TOLERANCE_M
        )
        check(
            ok,
            f"datum({dlat}, {dlon}) + ({lat}, {lon}) -> ({expect_x:.9f}, {expect_y:.9f})",
            f"got ({x:.9f}, {y:.9f})",
        )

    print()
    print("=" * 78)
    print("Part 2 - cardinal directions")
    print("=" * 78)
    datum = Datum(BOOTSTRAP_FALLBACK_LAT, BOOTSTRAP_FALLBACK_LON)
    step_deg = 1e-4

    x_north, y_north = latlon_to_map(datum, datum.latitude_deg + step_deg, datum.longitude_deg)
    check(y_north > 0.0, "a small +lat step moves +y (north)", f"y = {y_north:+.6f} m")
    check(abs(x_north) <= 1e-12, "a +lat step does not move x", f"x = {x_north:+.3e} m")
    check(
        abs(y_north - step_deg * METERS_PER_DEGREE_LAT) <= 1e-9,
        "the +lat step scales by exactly METERS_PER_DEGREE_LAT",
        f"{y_north:.9f} m vs {step_deg * METERS_PER_DEGREE_LAT:.9f} m",
    )

    x_east, y_east = latlon_to_map(datum, datum.latitude_deg, datum.longitude_deg + step_deg)
    check(x_east > 0.0, "a small +lon step moves +x (east)", f"x = {x_east:+.6f} m")
    check(abs(y_east) <= 1e-12, "a +lon step does not move y", f"y = {y_east:+.3e} m")
    check(
        x_east < y_north,
        "the +lon step is compressed by cos(datum_lat) at 48 deg north",
        f"{x_east:.6f} m < {y_north:.6f} m",
    )
    expected_east = step_deg * METERS_PER_DEGREE_LAT * math.cos(math.radians(datum.latitude_deg))
    check(
        abs(x_east - expected_east) <= 1e-9,
        "the +lon step scales by METERS_PER_DEGREE_LAT * cos(datum_lat)",
        f"{x_east:.9f} m vs {expected_east:.9f} m",
    )

    x_south, y_south = latlon_to_map(datum, datum.latitude_deg - step_deg, datum.longitude_deg)
    x_west, _ = latlon_to_map(datum, datum.latitude_deg, datum.longitude_deg - step_deg)
    check(y_south < 0.0, "a small -lat step moves -y (south)", f"y = {y_south:+.6f} m")
    check(x_west < 0.0, "a small -lon step moves -x (west)", f"x = {x_west:+.6f} m")

    print()
    print("=" * 78)
    print("Part 3 - round trip")
    print("=" * 78)
    worst = 0.0
    for dlat, dlon, lat, lon, _, _ in FROZEN_VECTORS:
        d = Datum(dlat, dlon)
        x, y = latlon_to_map(d, lat, lon)
        back_lat, back_lon = map_to_latlon(d, x, y)
        rx, ry = latlon_to_map(d, back_lat, back_lon)
        error = math.hypot(rx - x, ry - y)
        worst = max(worst, error)
    check(
        worst <= ROUNDTRIP_TOLERANCE_M,
        "latlon -> map -> latlon -> map round trip closes",
        f"worst error {worst:.3e} m over {len(FROZEN_VECTORS)} vectors",
    )

    # And the other direction: map -> latlon -> map, which is what the stage-2
    # empirical check does (place an object at a known map offset, compare the
    # lat/lon the Octopus reports).
    worst = 0.0
    for x, y in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (2.23, 1.67), (-2.23, -1.67)):
        lat, lon = map_to_latlon(datum, x, y)
        rx, ry = latlon_to_map(datum, lat, lon)
        worst = max(worst, math.hypot(rx - x, ry - y))
    check(
        worst <= ROUNDTRIP_TOLERANCE_M,
        "map -> latlon -> map round trip closes over the demo patch",
        f"worst error {worst:.3e} m",
    )

    print()
    print("=" * 78)
    print("Part 4 - datum handling")
    print("=" * 78)
    fallback_datum = Datum(BOOTSTRAP_FALLBACK_LAT, BOOTSTRAP_FALLBACK_LON)
    check(
        is_bootstrap_fallback(fallback_datum),
        "the Garching bootstrap value is recognised",
        f"{BOOTSTRAP_FALLBACK_LAT}, {BOOTSTRAP_FALLBACK_LON}",
    )
    check(
        not is_bootstrap_fallback(Datum(48.2523611, 11.6359722)),
        "a datum 111 m away is not the bootstrap value",
    )
    # 1e-6 deg is inside the match tolerance: their value travels through JSON.
    check(
        is_bootstrap_fallback(Datum(BOOTSTRAP_FALLBACK_LAT + 5e-7, BOOTSTRAP_FALLBACK_LON)),
        "a JSON round-trip wobble still matches the bootstrap value",
    )

    tracker = DatumTracker(fallback=None, jump_warn_m=0.25)
    check(tracker.datum is None, "a tracker with no fallback starts without a datum")
    check(
        tracker.dispatch_blocker() == "NO_DATUM",
        "no datum blocks dispatch with NO_DATUM",
        tracker.dispatch_blocker(),
    )

    update = tracker.update(fallback_datum.with_stamp(100.0))
    check(update.accepted, "the bootstrap datum is accepted onto the tracker")
    check(
        tracker.dispatch_blocker() == "BOOTSTRAP_FALLBACK_DATUM",
        "but it blocks dispatch with BOOTSTRAP_FALLBACK_DATUM",
        tracker.dispatch_blocker(),
    )
    raised = False
    try:
        tracker.require_datum()
    except GeodesyError as exc:
        raised = exc.reason == "BOOTSTRAP_FALLBACK_DATUM"
    check(raised, "require_datum() refuses the bootstrap datum")

    real = Datum(48.1500000, 11.5000000, from_topic=True, stamp_sec=101.0)
    update = tracker.update(real)
    check(update.accepted and update.changed, "a real datum replaces the bootstrap value")
    check(tracker.dispatch_blocker() == "", "a real datum unblocks dispatch")
    check(tracker.has_topic_datum, "the tracker records that the datum came from the topic")
    check(
        abs(tracker.age_sec(103.5) - 2.5) <= 1e-9,
        "datum age is reported",
        f"{tracker.age_sec(103.5):.3f} s",
    )

    # A marker drag: 0.1 m is below the warn threshold, 1.0 m is above it.
    small = Datum(real.latitude_deg + 0.1 / METERS_PER_DEGREE_LAT, real.longitude_deg, stamp_sec=102.0)
    update = tracker.update(small)
    check(
        update.changed and not update.exceeds_warn,
        "a 0.1 m datum drag is a change but below the warn threshold",
        f"jump {update.jump_m:.4f} m",
    )
    big = Datum(small.latitude_deg + 1.0 / METERS_PER_DEGREE_LAT, small.longitude_deg, stamp_sec=103.0)
    update = tracker.update(big)
    check(
        update.exceeds_warn,
        "a 1.0 m datum drag exceeds the warn threshold (cancel + re-resolve)",
        f"jump {update.jump_m:.4f} m",
    )
    check(
        abs(datum_offset_m(small, big) - 1.0) <= 1e-6,
        "datum_offset_m measures the drag in metres",
        f"{datum_offset_m(small, big):.6f} m",
    )
    update = tracker.update(big)
    check(
        not update.changed and update.jump_m == 0.0,
        "republishing the same datum at 1 Hz is not a change",
    )

    # Garbage in must not become a silent zero offset.
    check(
        not tracker.update(Datum(float("nan"), 11.0)).accepted,
        "a NaN datum is refused",
    )
    check(
        not tracker.update(Datum(95.0, 11.0)).accepted,
        "an out-of-range datum is refused",
    )
    check(
        tracker.datum is not None and tracker.datum.latitude_deg == big.latitude_deg,
        "a refused datum does not overwrite the last good one",
    )

    fallback_only = DatumTracker(fallback=Datum(48.2, 11.6, from_topic=False))
    check(
        fallback_only.dispatch_blocker() == "" and not fallback_only.has_topic_datum,
        "a configured fallback allows preview but is flagged as not from the topic",
    )
    rejected = False
    try:
        DatumTracker(fallback=Datum(48.2, 11.6, from_topic=True))
    except ValueError:
        rejected = True
    check(rejected, "a fallback datum claiming from_topic=True is refused at construction")

    print()
    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All geodesy checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
