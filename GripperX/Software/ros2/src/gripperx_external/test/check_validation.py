#!/usr/bin/env python3
"""Verification of the Octopus payload parsing, the validation pipeline and the
arming state machine.

Pure python, no ROS required. Run from the workspace source tree:

    python3 src/gripperx_external/test/check_validation.py

The three parts are in one file because they are one decision path: a payload is
parsed, validated, and then either previewed or dispatched through the arming
gate. Testing them together is what catches a reason code that exists in one
module and not in the next.

Part 1  Octopus protocol - the four payloads, including the ones their side is
        loose about (bare vs. object trash_goal_done, confidence: null).
Part 2  the ordered pipeline - one case per stage, in order, each proving that
        the EARLIER failure wins.
Part 3  the arming state machine - default disarmed, explicit duration, and each
        auto-disarm trigger.
Part 4  goal/target correlation and the dispatch gate - the stage-3 half of the
        same decision path: the goal fix carries no id, so the id it refers to
        is recovered by position, and an undecidable recovery must refuse.
"""

from __future__ import annotations

import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from gripperx_external import arming as arm_mod  # noqa: E402
from gripperx_external import correlation as corr  # noqa: E402
from gripperx_external import diagnostics as diag_mod  # noqa: E402
from gripperx_external import domain_guard as dom  # noqa: E402
from gripperx_external import octopus_protocol as proto  # noqa: E402
from gripperx_external import validation as val  # noqa: E402
from gripperx_external.geodesy import (  # noqa: E402
    BOOTSTRAP_FALLBACK_LAT,
    BOOTSTRAP_FALLBACK_LON,
    METERS_PER_DEGREE_LAT,
    Datum,
    DatumTracker,
)
from gripperx_external.grasp import GraspOffset  # noqa: E402

# NOT A MEASUREMENT - test fixture only. See check_grasp.py.
FIXTURE_OFFSET = GraspOffset(x=0.35, y=0.0, tolerance_m=0.05)

DATUM = Datum(48.2000000, 11.6000000, from_topic=True, stamp_sec=1000.0)
NOW = 1000.0

_failures = []


def check(condition: bool, label: str, detail: str = "") -> None:
    mark = "PASS" if condition else "FAIL"
    print(f"  [{mark}] {label}" + (f"  ({detail})" if detail else ""))
    if not condition:
        _failures.append(label)


def latlon_for(x: float, y: float, datum: Datum = DATUM):
    """Inverse of the conversion, so test cases can be written in metres."""
    lat = datum.latitude_deg + y / METERS_PER_DEGREE_LAT
    lon = datum.longitude_deg + x / (
        METERS_PER_DEGREE_LAT * math.cos(math.radians(datum.latitude_deg))
    )
    return lat, lon


def tracker_with(datum: Datum = DATUM) -> DatumTracker:
    tracker = DatumTracker(fallback=None, jump_warn_m=0.25)
    tracker.update(datum)
    return tracker


def make_ctx(**overrides) -> val.ValidationContext:
    """A context that validates cleanly, so each test only perturbs one thing."""
    lat, lon = latlon_for(2.0, 0.0)
    defaults = dict(
        goal=val.IncomingGoal(
            target_id="7",
            latitude_deg=lat,
            longitude_deg=lon,
            status=0,
            stamp_sec=NOW - 0.2,
            confidence=0.8,
        ),
        now_sec=NOW,
        datum_tracker=tracker_with(),
        grasp_offset=FIXTURE_OFFSET,
        robot_pose=(0.0, 0.0, 0.0),
        robot_pose_age_sec=0.05,
        geofence=lambda x, y: -5.0 <= x <= 5.0 and -5.0 <= y <= 5.0,
        costmap_cost=lambda x, y: 0,
        max_goal_cost=200,
    )
    defaults.update(overrides)
    return val.ValidationContext(**defaults)


# ===========================================================================
def part1_protocol() -> None:
    print("=" * 78)
    print("Part 1 - Octopus payloads")
    print("=" * 78)

    # --- NavSatFix (datum and goal share the type) -------------------------
    raw = proto.build_navsatfix(48.25, 11.63, status=0, stamp_sec=1234.5)
    fix = proto.parse_navsatfix(raw)
    check(
        fix.latitude_deg == 48.25 and fix.longitude_deg == 11.63,
        "NavSatFix round-trips through build/parse",
        f"{fix.latitude_deg}, {fix.longitude_deg}",
    )
    check(abs(fix.stamp_sec - 1234.5) < 1e-6, "the header stamp survives", str(fix.stamp_sec))
    check(fix.has_fix, "status 0 (STATUS_FIX) counts as a fix")

    no_fix = proto.parse_navsatfix(proto.build_navsatfix(48.25, 11.63, status=proto.STATUS_NO_FIX))
    check(not no_fix.has_fix, "status -1 (STATUS_NO_FIX) does not count as a fix")

    hand_built = proto.parse_navsatfix({"latitude": 48.25, "longitude": 11.63})
    check(
        hand_built.has_fix and hand_built.stamp_sec is None,
        "a hand-built fix with no status and no header parses as a fix without a stamp",
        "eve_fake_gps_bridge_node constructs its messages by hand",
    )

    for bad, label in (
        ({"longitude": 11.6}, "a missing latitude"),
        ({"latitude": "north", "longitude": 11.6}, "a non-numeric latitude"),
        ("not an object", "a bare string"),
    ):
        raised = ""
        try:
            proto.parse_navsatfix(bad)
        except proto.ProtocolError as exc:
            raised = exc.reason
        check(raised == "MALFORMED_PAYLOAD", f"{label} is a ProtocolError", raised)

    # --- strict key names (FR-12 item 6) -----------------------------------
    # The parser must NOT accept both spellings. A rename on the Octopus side
    # has to surface as an error here rather than being absorbed by a tolerant
    # reader, which is the whole reason the interface is written down.
    detail = ""
    try:
        proto.parse_navsatfix({"lat": 48.25, "lon": 11.63})
    except proto.ProtocolError as exc:
        detail = exc.detail
    check(
        "expected key 'latitude'" in detail and "'lat'" in detail,
        "a NavSatFix spelled lat/lon is REFUSED and the error names the expected key",
        detail or "no error raised",
    )

    # --- trash_gps ---------------------------------------------------------
    payload = {
        "datum": {"lat": 48.2, "lon": 11.6, "from_topic": True},
        "goal_id": 2,
        "open_count": 3,
        "targets": [
            {
                "id": 1,
                "lat": 48.2001,
                "lon": 11.6001,
                "x": 1.0,
                "y": 2.0,
                "confidence": 0.91,
                "collected": True,
                "is_goal": False,
                "last_seen": 1700000000.0,
            },
            {
                "id": 2,
                "lat": 48.2002,
                "lon": 11.6002,
                "x": 3.0,
                "y": 4.0,
                "confidence": None,
                "collected": False,
                "is_goal": True,
                "last_seen": 1700000001.0,
            },
        ],
    }
    report = proto.parse_trash_gps({"data": json.dumps(payload)})
    check(report.datum is not None and report.datum.from_topic, "datum.from_topic is read")
    check(report.goal_id == "2", "goal_id is normalised to a string", report.goal_id)
    check(report.open_count == 3, "open_count is parsed", str(report.open_count))
    check(len(report.targets) == 2, "both targets are parsed", str(len(report.targets)))
    check(
        report.targets[1].confidence is None,
        "confidence: null is valid and stays None",
        "their detector emits null",
    )
    check(report.targets[0].confidence == 0.91, "a real confidence survives")
    check(report.targets[0].collected is True, "the collected flag is parsed")
    check(
        report.goal_target() is not None and report.goal_target().id == "2",
        "the is_goal target is found",
    )
    check(
        report.targets[1].x == 3.0 and report.targets[1].y == 4.0,
        "their own map metres are kept for cross-checking",
    )
    check(not report.mission_complete, "open_count 3 is not mission complete")

    # The one case where the missing TRANSIENT_LOCAL latch would bite:
    # trash_goal goes silent when nothing is open, so open_count is the only
    # signal that distinguishes "done" from "stale".
    empty = proto.parse_trash_gps(
        proto.build_trash_gps(
            proto.TrashDatumInfo(48.2, 11.6, True), goal_id=None, open_count=0, targets=[]
        )
    )
    check(empty.mission_complete, "open_count 0 IS mission complete")
    check(empty.goal_target() is None, "no goal target when nothing is open")

    bootstrap = proto.parse_trash_gps(
        proto.build_trash_gps(
            proto.TrashDatumInfo(BOOTSTRAP_FALLBACK_LAT, BOOTSTRAP_FALLBACK_LON, False),
            goal_id="1",
            open_count=1,
            targets=[],
        )
    )
    check(
        bootstrap.datum is not None and not bootstrap.datum.from_topic,
        "the Octopus tells us itself when it is still on the bootstrap datum",
    )

    raised = ""
    try:
        proto.parse_trash_gps({"data": "{not json"})
    except proto.ProtocolError as exc:
        raised = exc.reason
    check(raised == "MALFORMED_JSON", "invalid JSON is MALFORMED_JSON", raised)

    raised = ""
    try:
        proto.parse_trash_gps({"data": json.dumps({"targets": [{"lat": 1.0, "lon": 2.0}]})})
    except proto.ProtocolError as exc:
        raised = exc.reason
    check(raised == "MALFORMED_PAYLOAD", "a target without an id is MALFORMED_PAYLOAD", raised)

    # The mirror image of the NavSatFix case: here lat/lon is canonical and the
    # ROS-style long form is the drift. Both directions are refused, so the
    # parser cannot be "helpfully" unified into one tolerant reader.
    detail = ""
    try:
        proto.parse_trash_gps(
            {"data": json.dumps({"datum": {"latitude": 48.2, "longitude": 11.6}})}
        )
    except proto.ProtocolError as exc:
        detail = exc.detail
    check(
        "expected key 'lat'" in detail and "'latitude'" in detail,
        "a trash_gps datum spelled latitude/longitude is REFUSED, naming 'lat'",
        detail or "no error raised",
    )

    detail = ""
    try:
        proto.parse_trash_gps(
            {
                "data": json.dumps(
                    {"targets": [{"id": 1, "latitude": 48.2, "longitude": 11.6}]}
                )
            }
        )
    except proto.ProtocolError as exc:
        detail = exc.detail
    check(
        "expected key 'lat'" in detail and "'latitude'" in detail,
        "a trash_gps target spelled latitude/longitude is REFUSED, naming 'lat'",
        detail or "no error raised",
    )

    # --- trash_goal_done ---------------------------------------------------
    check(proto.parse_goal_done("1") == "1", "a bare id parses")
    check(proto.parse_goal_done({"data": "1"}) == "1", "a bare id inside a String parses")
    check(proto.parse_goal_done('{"id": 1}') == "1", "the JSON object form parses")
    check(proto.parse_goal_done({"data": '{"id": 42}'}) == "42", "and inside a String too")
    check(proto.parse_goal_done({"id": 42}) == "42", "an already-decoded object parses")
    check(proto.parse_goal_done(" 7 ") == "7", "surrounding whitespace is tolerated")
    check(proto.build_goal_done("7") == "7", "we emit the bare id by default")
    check(
        proto.parse_goal_done(proto.build_goal_done("7", as_json_object=True)) == "7",
        "the JSON emission round-trips through our own parser",
    )
    for bad, label in (("", "an empty payload"), ({"data": ""}, "an empty String")):
        raised = ""
        try:
            proto.parse_goal_done(bad)
        except proto.ProtocolError as exc:
            raised = exc.reason
        check(raised == "MALFORMED_PAYLOAD", f"{label} is refused", raised)

    # --- proposed telemetry ------------------------------------------------
    status = json.loads(
        proto.build_device_status(
            48.2, 11.6, 1.0, 2.0, 90.0, "navigating", "7", True, True, 1700000000.0
        )
    )
    check(status["device_id"] == "gripperx", "telemetry carries the dashboard's device id")
    check(
        status["battery"]["status"] == "unavailable"
        and status["battery"]["percent"] is None
        and status["battery"]["reason"] == "NO_SENSOR_INSTALLED",
        "battery is structurally honest, not a fabricated number",
        "no shunt, no divider, no ADC channel (HWR-21)",
    )
    check(
        status["pose"]["lat"] == 48.2 and status["pose"]["x"] == 1.0,
        "the pose is reported as both lat/lon and map metres",
    )
    print()


# ===========================================================================
def part2_pipeline() -> None:
    print("=" * 78)
    print("Part 2 - ordered validation pipeline, first failure wins")
    print("=" * 78)

    result = val.validate_goal(make_ctx())
    check(result.accepted, "the clean case is accepted", result.reason or "-")
    check(
        result.object_xy is not None and abs(result.object_xy[0] - 2.0) < 1e-6,
        "the object converts back to (2.0, 0.0) in map metres",
        f"{result.object_xy}",
    )
    check(
        result.robot_pose is not None
        and abs(result.robot_pose[0] - 1.65) < 1e-6
        and abs(result.robot_pose[2]) < 1e-9,
        "the robot stops 0.35 m short of the object, facing it",
        f"{result.robot_pose}",
    )
    check(not result.warnings, "no warnings on a clean known costmap", str(result.warnings))

    # 1 - malformed / bad status
    result = val.validate_goal(
        make_ctx(goal=val.IncomingGoal("7", 0.0, 0.0, well_formed=False, malformed_detail="x"))
    )
    check(result.reason == val.MALFORMED_FIX, "stage 1 rejects a malformed fix", result.reason)
    lat, lon = latlon_for(2.0, 0.0)
    result = val.validate_goal(
        make_ctx(goal=val.IncomingGoal("7", lat, lon, status=-1, stamp_sec=NOW))
    )
    check(result.reason == val.BAD_FIX_STATUS, "stage 1 rejects status -1", result.reason)

    # 2 - datum. A NO_FIX goal on a bootstrap datum must report the STATUS,
    #     proving stage 1 beats stage 2.
    bootstrap_tracker = tracker_with(
        Datum(BOOTSTRAP_FALLBACK_LAT, BOOTSTRAP_FALLBACK_LON, from_topic=True, stamp_sec=NOW)
    )
    result = val.validate_goal(
        make_ctx(
            datum_tracker=bootstrap_tracker,
            goal=val.IncomingGoal("7", lat, lon, status=-1, stamp_sec=NOW),
        )
    )
    check(
        result.reason == val.BAD_FIX_STATUS,
        "stage 1 wins over stage 2 (first failure wins)",
        result.reason,
    )
    result = val.validate_goal(make_ctx(datum_tracker=bootstrap_tracker))
    check(
        result.reason == val.BOOTSTRAP_FALLBACK_DATUM,
        "stage 2 refuses to dispatch on the bootstrap datum",
        result.reason,
    )
    result = val.validate_goal(make_ctx(datum_tracker=DatumTracker(fallback=None)))
    check(result.reason == val.NO_DATUM, "stage 2 refuses with no datum at all", result.reason)

    # 3 - lat/lon
    result = val.validate_goal(
        make_ctx(goal=val.IncomingGoal("7", float("nan"), lon, stamp_sec=NOW))
    )
    check(result.reason == val.LATLON_NOT_FINITE, "stage 3 rejects NaN", result.reason)
    result = val.validate_goal(make_ctx(goal=val.IncomingGoal("7", 91.0, lon, stamp_sec=NOW)))
    check(result.reason == val.LATLON_OUT_OF_RANGE, "stage 3 rejects lat 91", result.reason)

    # 4 - staleness
    result = val.validate_goal(
        make_ctx(goal=val.IncomingGoal("7", lat, lon, stamp_sec=NOW - 30.0))
    )
    check(result.reason == val.STALE_STAMP, "stage 4 rejects a 30 s old stamp", result.reason)
    result = val.validate_goal(make_ctx(goal=val.IncomingGoal("7", lat, lon, stamp_sec=None)))
    check(result.reason == val.NO_STAMP, "stage 4 rejects a missing stamp by default", result.reason)
    result = val.validate_goal(
        make_ctx(goal=val.IncomingGoal("7", lat, lon, stamp_sec=None), require_stamp=False)
    )
    check(
        result.accepted,
        "require_stamp=False tolerates their hand-built messages",
        result.reason or "-",
    )

    # 5 - duplicate and blacklist
    result = val.validate_goal(make_ctx(current_goal_id="7"))
    check(
        result.verdict == val.VERDICT_DUPLICATE,
        "stage 5 reports a republished goal as DUPLICATE, not as a rejection",
        result.verdict,
    )
    check(
        result.verdict != val.VERDICT_REJECTED,
        "the 1 Hz republish therefore does not inflate the rejection counter",
    )
    result = val.validate_goal(make_ctx(blacklisted_ids=("7",)))
    check(result.reason == val.BLACKLISTED, "a blacklisted id is refused", result.reason)

    # 7 - grasp offset
    result = val.validate_goal(make_ctx(grasp_offset=GraspOffset()))
    check(
        result.reason == val.GRASP_OFFSET_NOT_CONFIGURED,
        "stage 7 refuses while the grasp offset is TO-VERIFY",
        result.reason,
    )
    check(
        result.robot_pose is None,
        "and no robot pose is produced from an unmeasured offset",
    )

    # 8 - approach candidates
    result = val.validate_goal(make_ctx(geofence=lambda x, y: False))
    check(
        result.reason == val.OUTSIDE_GEOFENCE,
        "a ring that fails unanimously reports the specific reason, not the generic one",
        result.reason,
    )
    check(
        val.OUTSIDE_GEOFENCE in result.detail and "12 heading" in result.detail,
        "the detail says how many headings were examined",
        result.detail,
    )
    check(
        result.approach is not None and len(result.approach.evaluated) == 12,
        "all 12 headings were examined",
    )

    # Genuinely boxed in: each heading blocked by something different. This is
    # what NO_APPROACH_CANDIDATE is reserved for.
    result = val.validate_goal(
        make_ctx(
            geofence=lambda x, y: y < 0.0,
            costmap_cost=lambda x, y: None if y < 0.0 else 0,
        )
    )
    check(
        result.reason == val.NO_APPROACH_CANDIDATE,
        "a mixed ring failure reports NO_APPROACH_CANDIDATE",
        result.reason,
    )
    check(
        val.OUTSIDE_GEOFENCE in result.detail and val.OUTSIDE_COSTMAP in result.detail,
        "and lists every distinct per-candidate reason",
        result.detail,
    )

    # A wall on the far side: the seed pose is blocked, a rotated one is not.
    result = val.validate_goal(
        make_ctx(
            robot_pose=(4.0, 0.0, math.pi),
            geofence=lambda x, y: x <= 2.0,
        )
    )
    check(result.accepted, "an object against a wall still resolves", result.reason or "-")
    check(
        result.approach is not None and abs(result.approach.chosen.deviation_rad) > 1e-6,
        "by choosing a different approach heading",
        f"{math.degrees(result.approach.chosen.deviation_rad):+.1f} deg",
    )

    # 9 / 11 / 12 with the ring reduced to one heading by an operator override:
    # a single candidate is trivially unanimous, so each specific reason surfaces.
    result = val.validate_goal(
        make_ctx(approach_theta_override=0.0, geofence=lambda x, y: False)
    )
    check(
        result.reason == val.OUTSIDE_GEOFENCE,
        "an overridden heading outside the geofence reports OUTSIDE_GEOFENCE",
        result.reason,
    )
    check(
        result.approach is not None and len(result.approach.evaluated) == 1,
        "an override reduces the ring to exactly one heading",
        f"{len(result.approach.evaluated)} evaluated",
    )
    result = val.validate_goal(
        make_ctx(approach_theta_override=0.0, costmap_cost=lambda x, y: None)
    )
    check(
        result.reason == val.OUTSIDE_COSTMAP,
        "outside the global costmap reports OUTSIDE_COSTMAP",
        result.reason,
    )
    result = val.validate_goal(
        make_ctx(approach_theta_override=0.0, costmap_cost=lambda x, y: 253)
    )
    check(result.reason == val.COST_TOO_HIGH, "a lethal cell reports COST_TOO_HIGH", result.reason)
    result = val.validate_goal(
        make_ctx(approach_theta_override=0.0, path_check=lambda x, y, yaw: False)
    )
    check(
        result.reason == val.PATH_NOT_FOUND,
        "verify_path failing reports PATH_NOT_FOUND",
        result.reason,
    )

    # Unknown cells: accepted WITH A WARNING. Rejecting them would make the
    # twin's default slam mode unusable (nav2.yaml allow_unknown: true).
    result = val.validate_goal(make_ctx(costmap_cost=lambda x, y: val.COST_UNKNOWN))
    check(result.accepted, "an unknown costmap cell is accepted")
    check(
        val.WARN_UNKNOWN_COSTMAP_CELL in result.warnings,
        "but it raises a warning",
        str(result.warnings),
    )

    # 10 - TF
    result = val.validate_goal(make_ctx(robot_pose=None))
    check(result.reason == val.TF_UNAVAILABLE, "no TF -> TF_UNAVAILABLE", result.reason)
    result = val.validate_goal(make_ctx(robot_pose_age_sec=9.0))
    check(result.reason == val.TF_UNAVAILABLE, "stale TF -> TF_UNAVAILABLE", result.reason)
    check(
        val.severity_of(val.TF_UNAVAILABLE) == val.SEVERITY_LOCAL,
        "TF_UNAVAILABLE is an OUR-SIDE failure: ERROR + diagnostic (SR-13)",
    )
    check(
        val.severity_of(val.BOOTSTRAP_FALLBACK_DATUM) == val.SEVERITY_CLIENT,
        "a peer-caused rejection is only a WARN",
    )
    for reason in (val.NAV2_UNAVAILABLE, val.LINK_LOST, val.INTERNAL_ERROR):
        check(
            val.severity_of(reason) == val.SEVERITY_LOCAL,
            f"{reason} is classified as our-side",
        )

    print()
    print("-" * 78)
    print("Dispatch-time re-validation")
    print("-" * 78)
    ctx = make_ctx()
    accepted = val.validate_goal(ctx)
    assert accepted.robot_pose is not None
    pose = accepted.robot_pose

    def dctx(**overrides) -> val.DispatchContext:
        defaults = dict(
            armed=True,
            dry_run=False,
            link_alive=True,
            teleop_mode="autonomous",
            teleop_mode_age_sec=0.1,
            nav2_available=True,
            datum_unchanged=True,
            pose=pose,
        )
        defaults.update(overrides)
        return val.DispatchContext(**defaults)

    check(val.validate_dispatch(dctx(), ctx).accepted, "the clean dispatch case passes")
    result = val.validate_dispatch(dctx(armed=False), ctx)
    check(
        result.verdict == val.VERDICT_PREVIEW and result.reason == val.NOT_ARMED,
        "disarmed is a PREVIEW verdict, not an error - it is the designed default",
        f"{result.verdict}/{result.reason}",
    )
    result = val.validate_dispatch(dctx(dry_run=True), ctx)
    check(
        result.verdict == val.VERDICT_PREVIEW and result.reason == val.DRY_RUN,
        "dry_run is the second independent block on dispatch",
        f"{result.verdict}/{result.reason}",
    )
    check(
        val.validate_dispatch(dctx(link_alive=False), ctx).reason == val.LINK_LOST,
        "a dead link blocks dispatch",
    )
    check(
        val.validate_dispatch(dctx(teleop_mode="keyboard"), ctx).reason
        == val.MODE_NOT_AUTONOMOUS,
        "layer 2 of the gate: the mux must be in autonomous",
    )
    check(
        val.validate_dispatch(dctx(teleop_mode_age_sec=None), ctx).reason == val.MODE_STALE,
        "an absent /teleop/active_mode is not treated as 'still autonomous'",
    )
    check(
        val.validate_dispatch(dctx(teleop_mode_age_sec=30.0), ctx).reason == val.MODE_STALE,
        "a stale /teleop/active_mode blocks dispatch",
    )
    check(
        val.validate_dispatch(dctx(nav2_available=False), ctx).reason == val.NAV2_UNAVAILABLE,
        "no navigate_to_pose server blocks dispatch",
    )
    check(
        val.validate_dispatch(dctx(datum_unchanged=False), ctx).reason == val.DATUM_CHANGED,
        "a datum that moved since validation blocks dispatch",
    )
    check(
        val.validate_dispatch(dctx(), make_ctx(costmap_cost=lambda x, y: 254)).reason
        == val.COST_TOO_HIGH,
        "the costmap is re-checked at dispatch time",
    )
    print()


# ===========================================================================
def part3_arming() -> None:
    print("=" * 78)
    print("Part 3 - arming state machine (SR-1)")
    print("=" * 78)

    machine = arm_mod.ArmingMachine()
    check(not machine.is_armed(NOW), "a fresh machine is DISARMED")
    check(not machine.allow_arm, "allow_arm defaults to false")
    check(
        machine.arm(10.0, "tester", NOW).granted is False,
        "arming is refused while allow_arm is false",
    )
    check(not machine.is_armed(NOW), "and the machine stays disarmed")
    check(
        "initial_armed" not in arm_mod.ArmingMachine.__init__.__code__.co_varnames,
        "there is no constructor path into the armed state",
        "no parameter, launch arg or env var may arm at startup",
    )

    machine = arm_mod.ArmingMachine(allow_arm=True, max_duration_sec=60.0)
    check(not machine.is_armed(NOW), "allow_arm=True alone does NOT arm")

    for duration, label in ((0.0, "zero"), (-5.0, "negative"), (120.0, "over the maximum")):
        result = machine.arm(duration, "tester", NOW)
        check(not result.granted, f"a {label} duration is refused", result.reason)
    check(not machine.arm(10.0, "", NOW).granted, "an anonymous arm request is refused")
    check(not machine.is_armed(NOW), "none of the refusals armed the machine")

    result = machine.arm(10.0, "tester", NOW)
    check(result.granted and machine.is_armed(NOW), "an explicit, bounded request arms", result.message)
    check(machine.armed_by == "tester", "the requester is recorded for the audit trail")
    check(
        abs(machine.seconds_remaining(NOW + 4.0) - 6.0) < 1e-9,
        "the remaining window counts down",
        f"{machine.seconds_remaining(NOW + 4.0):.1f} s",
    )

    check(machine.poll(NOW + 9.9) is None, "no auto-disarm before the window expires")
    event = machine.poll(NOW + 10.0)
    check(
        event is not None and event.trigger == arm_mod.TRIGGER_TIMEOUT,
        "the window expiring auto-disarms with TIMEOUT",
    )
    check(not machine.is_armed(NOW), "and the machine is disarmed")
    check(event.requires_cancel, "the timeout requires the Nav2 goal to be cancelled")
    check(
        not event.diagnostic_error,
        "TIMEOUT is expected operation, so no ERROR diagnostic",
    )
    check(machine.poll(NOW + 20.0) is None, "polling a disarmed machine is a no-op")

    # Each auto-disarm trigger.
    triggers = (
        ("link loss", lambda m: m.note_link(False, NOW), arm_mod.TRIGGER_LINK_LOST, True),
        (
            "the spacebar E-stop (set_mode=keyboard, SR-2)",
            lambda m: m.note_teleop_mode("keyboard", NOW),
            arm_mod.TRIGGER_MODE_CHANGE,
            True,
        ),
        (
            "Nav2 disappearing",
            lambda m: m.note_nav2(False, NOW),
            arm_mod.TRIGGER_NAV2_UNAVAILABLE,
            True,
        ),
        (
            "node shutdown",
            lambda m: m.shutdown(NOW),
            arm_mod.TRIGGER_NODE_SHUTDOWN,
            True,
        ),
        (
            "an operator disarm",
            lambda m: m.disarm(arm_mod.TRIGGER_OPERATOR, NOW, "by request"),
            arm_mod.TRIGGER_OPERATOR,
            False,
        ),
    )
    for label, action, expected, expect_error in triggers:
        machine = arm_mod.ArmingMachine(allow_arm=True)
        machine.arm(30.0, "tester", NOW)
        event = action(machine)
        check(
            event is not None and event.trigger == expected,
            f"{label} auto-disarms with {expected}",
            event.trigger if event else "no event",
        )
        check(not machine.is_armed(NOW), f"  ... and the gate is closed after {expected}")
        check(event.requires_cancel, f"  ... and {expected} requires a cancel")
        check(
            event.diagnostic_error is expect_error,
            f"  ... and raises an ERROR diagnostic: {expect_error}",
        )
        check(
            event.trigger_code == arm_mod.TRIGGER_CODES[expected],
            f"  ... and maps to the ArmingState uint8 {arm_mod.TRIGGER_CODES[expected]}",
        )

    machine = arm_mod.ArmingMachine(allow_arm=True)
    machine.arm(30.0, "tester", NOW)
    check(
        machine.note_teleop_mode("autonomous", NOW) is None,
        "staying in autonomous does not disarm",
    )
    check(machine.note_link(True, NOW) is None, "a healthy link does not disarm")
    check(machine.note_nav2(True, NOW) is None, "an available Nav2 does not disarm")

    machine = arm_mod.ArmingMachine(allow_arm=True, max_consecutive_aborts=3)
    machine.arm(30.0, "tester", NOW)
    check(machine.note_goal_aborted(NOW) is None, "one abort does not disarm")
    check(machine.note_goal_aborted(NOW) is None, "two aborts do not disarm")
    event = machine.note_goal_aborted(NOW)
    check(
        event is not None and event.trigger == arm_mod.TRIGGER_EXCESSIVE_ABORTS,
        "the third abort disarms with EXCESSIVE_ABORTS",
    )
    check(event.diagnostic_error, "EXCESSIVE_ABORTS raises an ERROR diagnostic")

    machine = arm_mod.ArmingMachine(allow_arm=True, max_consecutive_aborts=3)
    machine.arm(30.0, "tester", NOW)
    machine.note_goal_aborted(NOW)
    machine.note_goal_aborted(NOW)
    machine.note_goal_succeeded()
    check(
        machine.consecutive_aborts == 0 and machine.is_armed(NOW),
        "a success resets the abort budget",
    )
    machine.arm(30.0, "tester", NOW + 1.0)
    check(machine.consecutive_aborts == 0, "a fresh arming window resets the abort budget")

    snapshot = machine.snapshot(NOW + 1.0)
    check(
        snapshot["armed"] is True and abs(snapshot["seconds_remaining"] - 30.0) < 1e-9,
        "snapshot() reports the live state for telemetry and ArmingState",
        str(snapshot["seconds_remaining"]),
    )
    check(
        snapshot["auto_pick_available"] is False,
        "auto-pick defaults to unavailable (no arm in the URDF or sim)",
    )
    raised = False
    try:
        machine.disarm("NOT_A_TRIGGER", NOW)
    except ValueError:
        raised = True
    check(raised, "an unknown disarm trigger is a programming error, not a silent pass")

    # -- SAFETY.md F-2 / condition C-2 -------------------------------------
    # The armed state must be bounded by the CLOCK, not by a timer having run.
    # Before the fix, `armed` was a stored flag: a machine armed for 120 s still
    # read True at t+4000 as long as nothing called poll(), while
    # seconds_remaining() already said 0.0. The two disagreed, and the one a
    # dispatch would have consulted was the wrong one.
    machine = arm_mod.ArmingMachine(allow_arm=True, max_duration_sec=600.0)
    machine.arm(120.0, "tester", 1000.0)
    check(machine.is_armed(1119.9), "armed inside the window")
    check(
        not machine.is_armed(1120.0),
        "EXPIRED AT READ TIME, with no poll() in between (F-2)",
        "the read-time check is what a dispatch consults",
    )
    check(
        machine.seconds_remaining(5000.0) == 0.0
        and not machine.is_armed(5000.0),
        "seconds_remaining and is_armed agree long after expiry",
    )
    check(
        not hasattr(arm_mod.ArmingMachine, "armed"),
        "the stale `armed` property is GONE, not merely discouraged",
        "a reader with no clock cannot be given a correct answer",
    )
    snapshot = machine.snapshot(5000.0)
    check(
        snapshot["armed"] is False,
        "snapshot() reports the expired window as closed even without a poll",
    )

    # A trigger that arrives after the window ran out is recorded as TIMEOUT:
    # that is what actually closed the gate. Otherwise an expiry no poll got to
    # first would be filed under someone else's trigger.
    machine = arm_mod.ArmingMachine(allow_arm=True)
    machine.arm(30.0, "tester", 1000.0)
    event = machine.note_teleop_mode("keyboard", 1100.0)
    check(
        event is not None and event.trigger == arm_mod.TRIGGER_TIMEOUT,
        "a trigger observed after expiry is reported as TIMEOUT, not as itself",
        event.detail if event else "no event",
    )
    check(event.requires_cancel, "  ... and it still requires a cancel")

    # -- SAFETY.md F-1 / condition C-1 -------------------------------------
    # The machine was never the problem here - the node's guard was - but the
    # property the node now relies on is asserted at module level too: the mode
    # is a STATE, so a repeat of a non-autonomous mode must disarm just as the
    # first sighting of it does, with no transition to witness.
    machine = arm_mod.ArmingMachine(allow_arm=True)
    machine.arm(30.0, "tester", NOW)
    event = machine.note_teleop_mode("keyboard", NOW)
    check(
        event is not None and event.trigger == arm_mod.TRIGGER_MODE_CHANGE,
        "the FIRST observed mode, with no previous mode, disarms (F-1)",
    )
    check(
        machine.note_teleop_mode("keyboard", NOW) is None,
        "repeating it while disarmed is a no-op, so a 20 Hz mux cannot spam",
    )
    machine.arm(30.0, "tester", NOW)
    check(
        machine.note_teleop_mode("autonomous", NOW) is None
        and machine.note_teleop_mode("autonomous", NOW) is None,
        "and autonomous, however often it repeats, never disarms",
    )
    print()




def part4_correlation() -> None:
    print("=" * 78)
    print("Part 4 - correlation (SAFETY.md F-6 / C-7) and the dispatch gate (F-9 / C-5)")
    print("=" * 78)

    # Their merge_radius_m is 0.25, so two targets 0.30 m apart are exactly what
    # their pipeline is built to keep separate - near pairs are expected.
    a = corr.TargetPosition("1", 1.00, 0.00)
    b = corr.TargetPosition("2", 1.30, 0.00)
    far = corr.TargetPosition("3", 4.00, 4.00)

    unique = corr.correlate((1.02, 0.0), [a, far], 0.25)
    check(unique.unique and unique.target_id == "1", "a single target inside the tolerance correlates")
    check(abs(unique.distance_m - 0.02) < 1e-9, "  ... and the match distance is reported", f"{unique.distance_m:.3f} m")

    ambiguous = corr.correlate((1.15, 0.0), [a, b, far], 0.25)
    check(
        ambiguous.status == corr.AMBIGUOUS,
        "TWO targets inside the tolerance REFUSE rather than pick the nearest",
        ambiguous.detail[:60],
    )
    check(ambiguous.target_id == "", "  ... and an ambiguous match yields NO id to acknowledge")
    check(len(ambiguous.candidates) == 2, "  ... while both candidates are reported for the operator")
    # The nearest one is 0.13 m away and the other 0.15 m: nearest-wins would
    # have been confident and wrong half the time.
    check(
        ambiguous.candidates[0].id == "1" and ambiguous.candidates[0].distance_m < ambiguous.candidates[1].distance_m,
        "  ... sorted by distance, so the temptation is visible in the report",
    )

    none = corr.correlate((9.0, 9.0), [a, b], 0.25)
    check(none.status == corr.NO_MATCH, "a fix matching nothing is NO_MATCH, not a nearest guess")
    check(none.target_id == "", "  ... with no id")

    check(
        corr.correlate((1.02, 0.0), [], 0.25).status == corr.NO_TARGETS,
        "no target list yet is its own state, distinct from NO_MATCH",
    )

    mismatch = corr.correlate((1.02, 0.0), [a, far], 0.25, reported_goal_id="3")
    check(
        mismatch.status == corr.ID_MISMATCH,
        "the reported goal_id CONTRADICTING the position match is a refusal",
    )
    check(mismatch.target_id == "", "  ... and a contradicted match yields no usable id")
    check(
        corr.correlate((1.02, 0.0), [a, far], 0.25, reported_goal_id="1").unique,
        "  ... while agreement is evidence and passes",
    )

    # A collected target still participates in the ambiguity test: their ids
    # restart at 1 on a restart of their node, which is exactly when a stale
    # `collected` flag is least trustworthy.
    collected_pair = corr.correlate(
        (1.15, 0.0), [corr.TargetPosition("1", 1.00, 0.0, collected=True), b], 0.25
    )
    check(
        collected_pair.status == corr.AMBIGUOUS,
        "a COLLECTED target still makes a near pair ambiguous",
    )
    only_collected = corr.correlate(
        (1.02, 0.0), [corr.TargetPosition("1", 1.00, 0.0, collected=True)], 0.25
    )
    check(
        only_collected.status == corr.NO_MATCH,
        "a lone already-collected match is not re-acknowledged",
    )

    check(
        corr.correlate((1.02, 0.0), [a], 0.0).status == corr.NO_MATCH,
        "a non-positive tolerance matches nothing instead of substituting a default",
    )
    check(
        corr.correlate((float("nan"), 0.0), [a], 0.25).status == corr.NO_MATCH,
        "a non-finite goal position cannot correlate",
    )

    # -- the dispatch gate, in its designed order --------------------------
    # Every one of these is a state the EXTERNAL system cannot produce: arming
    # and the teleop mode are operator-owned, the rest are observations.
    ctx = val.ValidationContext(
        goal=val.IncomingGoal("1", DATUM.latitude_deg, DATUM.longitude_deg, stamp_sec=NOW),
        now_sec=NOW,
        datum_tracker=tracker_with(DATUM),
        grasp_offset=FIXTURE_OFFSET,
        geofence=lambda x, y: True,
        costmap_cost=lambda x, y: 0,
    )

    def dispatch(**overrides):
        base = dict(
            armed=True,
            dry_run=False,
            link_alive=True,
            teleop_mode="autonomous",
            teleop_mode_age_sec=0.1,
            nav2_available=True,
            datum_unchanged=True,
            pose=(1.0, 0.0, 0.0),
        )
        base.update(overrides)
        return val.validate_dispatch(val.DispatchContext(**base), ctx)

    check(dispatch().accepted, "the dispatch gate passes when every condition holds")
    check(
        dispatch(armed=False).verdict == val.VERDICT_PREVIEW
        and dispatch(armed=False).reason == val.NOT_ARMED,
        "disarmed is a PREVIEW verdict, not a rejection - it is the designed state",
    )
    check(dispatch(dry_run=True).reason == val.DRY_RUN, "dry_run is the second, independent block")
    check(dispatch(link_alive=False).reason == val.LINK_LOST, "a dead link refuses the dispatch")
    check(
        dispatch(teleop_mode="keyboard").reason == val.MODE_NOT_AUTONOMOUS,
        "a mode other than autonomous refuses the dispatch",
    )
    # THE DEAD-MUX CHECK. This is the only thing in the design that catches a
    # mux that stopped publishing while its last value still looked right, and
    # wiring `validate_dispatch` into the node is what activates it (F-9).
    check(
        dispatch(teleop_mode_age_sec=None).reason == val.MODE_STALE,
        "a mode that was NEVER heard refuses the dispatch (dead mux)",
    )
    check(
        dispatch(teleop_mode_age_sec=99.0).reason == val.MODE_STALE,
        "a mode that stopped being republished refuses it too (dead mux)",
    )
    check(
        dispatch(nav2_available=False).reason == val.NAV2_UNAVAILABLE,
        "no navigate_to_pose server refuses the dispatch",
    )
    check(
        dispatch(datum_unchanged=False).reason == val.DATUM_CHANGED,
        "a datum that moved since validation refuses the dispatch",
    )
    check(
        dispatch(armed=False, link_alive=False).reason == val.NOT_ARMED,
        "first failure wins, in the designed order",
    )
    # The pose is re-checked against the world, not merely carried forward.
    moved_world = val.ValidationContext(
        goal=ctx.goal,
        now_sec=NOW,
        datum_tracker=ctx.datum_tracker,
        grasp_offset=FIXTURE_OFFSET,
        geofence=lambda x, y: False,
        costmap_cost=lambda x, y: 0,
    )
    late = val.validate_dispatch(
        val.DispatchContext(
            armed=True,
            dry_run=False,
            link_alive=True,
            teleop_mode="autonomous",
            teleop_mode_age_sec=0.1,
            nav2_available=True,
            datum_unchanged=True,
            pose=(1.0, 0.0, 0.0),
        ),
        moved_world,
    )
    check(
        late.reason == val.OUTSIDE_GEOFENCE,
        "a geofence changed after validation refuses the pose at dispatch time",
    )
    print()


def part5_clock_and_staleness() -> None:
    """SAFETY.md F-24 (the clock), F-27 (the domain polarity), F-28 (the list age),
    F-26 (the shutdown prologue) - the parts of each that are decidable purely.

    Nothing here creates a node, a context or a DDS participant, so all of it
    runs anywhere, including on a laptop that must never touch the robot's
    domain. That is also what makes the F-27 table below possible at all: the
    branch it checks is unreachable at runtime from any machine that is allowed
    to run this suite.
    """
    print("=" * 78)
    print("Part 5 - the clock (F-24), the domain polarity (F-27), the list age (F-28)")
    print("=" * 78)

    # -- F-28: the target list has an age, and a stale one is its own refusal --
    a = corr.TargetPosition("1", 1.00, 0.00)
    fresh = corr.correlate((1.02, 0.0), [a], 0.25, list_age_sec=0.9, max_list_age_sec=5.0)
    check(fresh.unique, "a target list within max_target_list_age_sec correlates normally")
    stale = corr.correlate((1.02, 0.0), [a], 0.25, list_age_sec=7.5, max_list_age_sec=5.0)
    check(
        stale.status == corr.TARGETS_STALE,
        "F-28: a list older than the maximum is TARGETS_STALE - its OWN status",
        stale.detail[:70],
    )
    check(
        stale.target_id == "" and not stale.unique,
        "  ... and yields no id, so nothing downstream can act on it",
    )
    check(
        stale.status != corr.NO_MATCH,
        "  ... and is NOT folded into NO_MATCH: 'nothing is there' and 'we stopped "
        "being told' are different facts",
    )
    check(
        corr.correlate((1.02, 0.0), [a], 0.25, list_age_sec=None, max_list_age_sec=5.0).unique
        and corr.correlate((1.02, 0.0), [a], 0.25, list_age_sec=99.0, max_list_age_sec=None).unique,
        "  ... the check is off when either half is unset, so no caller gets it by accident",
    )
    check(
        corr.correlate((1.02, 0.0), [a], 0.25, list_age_sec=float("nan"),
                       max_list_age_sec=5.0).status == corr.TARGETS_STALE,
        "  ... and an age that is not a number counts as stale, not as fresh",
    )
    check(
        val.severity_of(val.TARGET_LIST_STALE) == val.SEVERITY_LOCAL,
        "F-28: a dead input channel is OUR severity, like LINK_LOST - not the peer's",
    )
    check(
        diag_mod.dispatch_status(
            nav_state="navigating", target_id="3", correlation="TARGETS_STALE",
            attempts=0, nav2_available=True, cancel_pending=False, cancel_failed=False,
        ).level == diag_mod.ERROR,
        "  ... and it raises an ERROR diagnostic, like the other undecidable states",
    )

    # -- F-27: the polarity of the real-robot discriminator -------------------
    # The exact table from the finding, plus the two domains this package uses.
    for domain, expected in ((20, False), (21, False), (0, False), (5, False),
                             (220, True), (221, True)):
        check(
            dom.is_simulation_domain(domain) is expected,
            f"F-27: domain {domain} is "
            + ("a simulation" if expected else "treated as a REAL ROBOT"),
        )
    check(
        not dom.is_simulation_domain(dom.REAL_ROBOT_DOMAIN_ID + 1),
        "F-27: renumbering the robot off 20 KEEPS the arm's arrival check - which "
        "is the whole finding: the old test asked '== 20' and answered 'permissive'",
    )
    check(
        dom.SIMULATION_DOMAIN_IDS == frozenset({dom.TWIN_DOMAIN_ID, dom.ACCEPTANCE_DOMAIN_ID}),
        "  ... and the permissive set is exactly two enumerated simulation domains",
        str(sorted(dom.SIMULATION_DOMAIN_IDS)),
    )

    # -- F-24: the clock as a disarm trigger and as a diagnostic --------------
    machine = arm_mod.ArmingMachine(allow_arm=True, max_duration_sec=600.0)
    machine.arm(120.0, "part5", 1000.0)
    check(machine.is_armed(1001.0), "an armed window for the clock test")
    event = machine.note_clock(True, 1001.0)
    check(event is None and machine.is_armed(1001.0),
          "F-24: an advancing clock is not an event and changes nothing")
    event = machine.note_clock(False, 1001.0, "frozen for 3.0s")
    check(
        event is not None and event.trigger == arm_mod.TRIGGER_CLOCK_STALLED,
        "F-24: a stalled clock DISARMS, with its own trigger",
        "" if event is None else event.detail[:50],
    )
    check(
        event is not None and event.requires_cancel and event.diagnostic_error,
        "  ... it cancels what is in flight and raises an ERROR diagnostic",
    )
    check(
        arm_mod.TRIGGER_CODES[arm_mod.TRIGGER_CLOCK_STALLED] == 8,
        "  ... and has a code of its own, so ArmingState reports what closed the gate",
    )
    check(
        machine.note_clock(False, 1002.0) is None,
        "  ... and while disarmed it is a no-op, so a stall cannot spam cancels",
    )

    # -- F-30 / SR-15 rule 7: the backwards jump is its OWN trigger -----------
    # Decided by the user 2026-08-19 against the implementation's own proposal
    # to reuse code 8. The pair of checks below is the decision: same behaviour,
    # different number. A test that accepted either code would not test it.
    machine = arm_mod.ArmingMachine(allow_arm=True, max_duration_sec=600.0)
    machine.arm(120.0, "part5", 1000.0)
    event = machine.note_clock_jumped_back(1001.0, "jumped back 120.0s")
    check(
        event is not None and event.trigger == arm_mod.TRIGGER_CLOCK_JUMPED_BACK,
        "F-30: a BACKWARDS jump disarms with CLOCK_JUMPED_BACK, not with CLOCK_STALLED",
        "" if event is None else event.trigger,
    )
    check(
        event is not None and event.trigger_code == 9,
        "  ... and reports ArmingState constant 9 on the wire",
        "" if event is None else str(event.trigger_code),
    )
    check(
        arm_mod.TRIGGER_CODES[arm_mod.TRIGGER_CLOCK_STALLED] == 8
        and arm_mod.TRIGGER_CODES[arm_mod.TRIGGER_CLOCK_JUMPED_BACK] == 9,
        "  ... while the STALL keeps 8 - the split is the point of the decision, "
        "so the two codes must be different and each must be the agreed one",
        f"stalled={arm_mod.TRIGGER_CODES[arm_mod.TRIGGER_CLOCK_STALLED]} "
        f"jumped={arm_mod.TRIGGER_CODES[arm_mod.TRIGGER_CLOCK_JUMPED_BACK]}",
    )
    check(
        event is not None and event.requires_cancel and event.diagnostic_error,
        "  ... and BEHAVES exactly as a stall does: it cancels what is in flight "
        "and raises an ERROR diagnostic. Only the reported code differs",
    )
    check(
        not machine.is_armed(1001.0)
        and machine.note_clock_jumped_back(1002.0) is None,
        "  ... and while disarmed it is a no-op, so a rewinding clock cannot spam "
        "cancels either",
    )
    check(
        diag_mod.clock_status(proven=True, stalled=False, frozen_for_sec=0.1,
                              use_sim_time=True, stall_sec=2.0).level == diag_mod.OK,
        "F-24: an advancing clock is OK on /diagnostics",
    )
    for proven, stalled, label in (
        (False, False, "a clock never observed to advance"),
        (True, True, "a clock that stopped"),
    ):
        check(
            diag_mod.clock_status(proven=proven, stalled=stalled, frozen_for_sec=9.0,
                                  use_sim_time=True, stall_sec=2.0).level == diag_mod.ERROR,
            f"  ... and {label} is an ERROR, because every other timeout is inert",
        )

    # -- F-26: the shutdown prologue cannot cost the cancel -------------------
    # The auditor's own suggestion, and it needs no production hook: this calls
    # the REAL `prepare_shutdown` on a hand-built stand-in whose logger raises on
    # every call and whose disarm raises too. No rclpy context, no node, no DDS.
    import threading
    import types

    from gripperx_external.goal_gateway_node import GoalGatewayNode  # noqa: E402

    class _RaisingLogger:
        def info(self, *_a, **_k):
            raise RuntimeError("logger says no (the F-22 shape)")

        warn = error = info

    class _RaisingArming:
        def shutdown(self, _now):
            raise RuntimeError("disarm exploded (BUG 2's shape)")

    class _Mission:
        target_id = "3"
        done = threading.Event()

    class _FakeGateway:
        pass

    fake = _FakeGateway()
    fake._shutdown_prepared = False
    fake._shutdown_reason = "part5"
    fake._arming_lock = threading.RLock()
    fake._mission_lock = threading.RLock()
    fake._arming = _RaisingArming()
    fake._mission = _Mission()
    fake._mission.done.set()
    fake._cancel_confirm_timeout_sec = 0.5
    fake._cancels = []
    fake.get_logger = lambda: _RaisingLogger()
    fake._ros_now = lambda: (_ for _ in ()).throw(RuntimeError("clock torn down"))
    fake._handle_disarm = lambda event: None
    fake._cancel_mission = lambda reason, now, error=True: fake._cancels.append((reason, now))
    # The three methods under test, unmodified, bound to the stand-in.
    for name in ("prepare_shutdown", "_shutdown_log", "_shutdown_now", "_log_at"):
        setattr(fake, name, types.MethodType(getattr(GoalGatewayNode, name), fake))

    raised = ""
    try:
        fake.prepare_shutdown()
    except Exception as exc:  # noqa: BLE001 - that is what is being measured
        raised = repr(exc)
    check(
        len(fake._cancels) == 1,
        "F-26: the cancel STILL HAPPENS with every logger call raising AND the "
        "disarm raising (real prepare_shutdown, hand-built stand-in, no hook)",
        f"cancels={fake._cancels} raised={raised or 'nothing'}",
    )
    check(not raised, "  ... and prepare_shutdown itself does not propagate", raised)
    check(
        fake._cancels[0][1] is not None and isinstance(fake._cancels[0][1], float),
        "  ... on a timestamp the wall-clock fallback supplied when the ROS clock raised",
    )
    before = len(fake._cancels)
    fake.prepare_shutdown()
    check(
        len(fake._cancels) == before,
        "  ... and the latch still holds: a second call cancels nothing again",
    )
    print()


def part6_package_e() -> None:
    """SAFETY.md revision 4: F-29, F-31, F-32, F-33, F-34 - the parts that are
    decidable without a node, a context or a DDS participant.

    F-30's recovery and F-29's behaviour at a low real-time factor are RUNTIME
    properties and are demonstrated in `check_stage3_twin.py --scenario clock`;
    what is checkable here is the structure that makes them hold, and the two
    pure functions that were changed.
    """
    import ast
    import inspect

    print("=" * 78)
    print("Part 6 - package E: F-29, F-31, F-32, F-33, F-34")
    print("=" * 78)

    # -- F-34: the import-time invariant --------------------------------------
    check(
        dom.REAL_ROBOT_DOMAIN_ID not in dom.SIMULATION_DOMAIN_IDS,
        "F-34: the real robot's domain is not in the permissive set",
        f"real={dom.REAL_ROBOT_DOMAIN_ID} sim={sorted(dom.SIMULATION_DOMAIN_IDS)}",
    )
    # And that the invariant is ENFORCED, not merely satisfied: the module is
    # re-executed with the one dangerous edit made, and must refuse to import.
    source = inspect.getsource(dom)
    patched = source.replace(
        "SIMULATION_DOMAIN_IDS = frozenset({TWIN_DOMAIN_ID, ACCEPTANCE_DOMAIN_ID})",
        "SIMULATION_DOMAIN_IDS = frozenset({TWIN_DOMAIN_ID, ACCEPTANCE_DOMAIN_ID, "
        "REAL_ROBOT_DOMAIN_ID})",
    )
    check(
        patched != source,
        "  ... (the patched-module check found the line it edits)",
    )
    raised = ""
    try:
        exec(compile(patched, "domain_guard_patched", "exec"), {"__name__": "dg_patched"})
    except RuntimeError as exc:
        raised = str(exc)
    check(
        "REAL ROBOT" in raised,
        "F-34: adding domain 20 to SIMULATION_DOMAIN_IDS makes the module REFUSE "
        "to import - the one dangerous edit is impossible, not discouraged",
        raised[:80],
    )

    # -- F-33: the command-client sweep counts, it does not test membership ---
    from gripperx_external.octopus_link_node import (  # noqa: E402
        ForbiddenTopic,
        assert_no_command_clients,
    )

    class _Waitable:
        def __init__(self, name, type_):
            self._action_name = name
            self._action_type = type_

    class _Logger:
        def __init__(self):
            self.lines = []

        def info(self, msg):
            self.lines.append(msg)

    class _StubNode:
        def __init__(self, waitables, clients=()):
            self.waitables = list(waitables)
            self.clients = list(clients)
            self._logger = _Logger()

        def get_name(self):
            return "stub"

        def get_logger(self):
            return self._logger

    class _NavType:
        pass

    class _PickType:
        pass

    allowed = (("/navigate_to_pose", _NavType), ("/pick_plastic", _PickType))
    ok_node = _StubNode([_Waitable("/navigate_to_pose", _NavType),
                         _Waitable("/pick_plastic", _PickType)])
    raised = ""
    try:
        assert_no_command_clients(ok_node, allowed)
    except ForbiddenTopic as exc:
        raised = str(exc)
    check(not raised, "F-33: exactly the two enumerated action clients still pass", raised[:70])

    dup_node = _StubNode([_Waitable("/navigate_to_pose", _NavType),
                          _Waitable("/navigate_to_pose", _NavType),
                          _Waitable("/pick_plastic", _PickType)])
    raised = ""
    try:
        assert_no_command_clients(dup_node, allowed)
    except ForbiddenTopic as exc:
        raised = str(exc)
    check(
        "unexpected action client" in raised and "x2" in raised,
        "F-33: a SECOND client on an allowed action is REFUSED - two goal senders "
        "on one action is the second-writer shape SR-9 exists for, and it used to "
        "pass because `allowed` was a set tested for membership",
        raised[:90],
    )
    raised = ""
    try:
        assert_no_command_clients(
            _StubNode([_Waitable("/other_action", _NavType)]), allowed
        )
    except ForbiddenTopic as exc:
        raised = str(exc)
    check(bool(raised), "  ... and an unenumerated action is still refused")
    raised = ""
    try:
        assert_no_command_clients(_StubNode([], clients=[]))
    except ForbiddenTopic as exc:
        raised = str(exc)
    check(not raised, "  ... and the link node's case - no clients at all - still passes")

    # -- F-31: an unproven clock is WARN inside the grace, ERROR after it -----
    check(
        diag_mod.clock_status(proven=False, stalled=False, frozen_for_sec=2.5,
                              use_sim_time=True, stall_sec=2.0,
                              startup_grace=True).level == diag_mod.WARN,
        "F-31: an unproven clock INSIDE the measured /clock discovery grace is "
        "WARN - the ERROR that fires on ordinary starts is the one nobody reads",
    )
    check(
        diag_mod.clock_status(proven=False, stalled=False, frozen_for_sec=12.0,
                              use_sim_time=True, stall_sec=2.0,
                              startup_grace=False).level == diag_mod.ERROR,
        "  ... and past the grace it is an ERROR, unchanged",
    )
    check(
        diag_mod.clock_status(proven=True, stalled=True, frozen_for_sec=3.0,
                              use_sim_time=True, stall_sec=2.0,
                              startup_grace=True).level == diag_mod.ERROR,
        "  ... while a clock that STOPPED after having been proven is an ERROR "
        "even if a caller passes the grace flag: the grace is a startup state",
    )
    check(
        # BY KEY, not by position. This read `.values[-1]` until F-40 added
        # three keys after `startup_grace` and it started asserting about a
        # different value entirely - a check anchored to the END of a list
        # silently changes what it checks every time the list grows.
        {
            kv.key: kv.value for kv in diag_mod.clock_status(
                proven=False, stalled=True, frozen_for_sec=2.5,
                use_sim_time=True, stall_sec=2.0, startup_grace=True,
            ).values
        }.get("startup_grace") in ("True", "true"),
        "  ... and /diagnostics carries the fact that it is in the grace",
    )

    # -- F-29 / F-32: the structure, read out of the source ------------------
    from gripperx_external import goal_gateway_node as gw_mod  # noqa: E402

    tree = ast.parse(inspect.getsource(gw_mod))
    functions = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            functions[node.name] = node

    # Every way a ROS-clock instant is reachable in this file. The list is
    # deliberately of NAMES rather than of call shapes: the mistake this catches
    # was `self._arming.note_clock(False, self._clock_ref_ros_sec, ...)`, an
    # attribute and not a call, which reads perfectly and reports TIMEOUT for a
    # clock stall because `disarm` then compares two different epochs.
    # `_teleop_mode_stamp_sec` was in this list until SAFETY.md F-38 moved the
    # mode age to the monotonic clock and renamed the field
    # `_teleop_mode_mono_sec`. Leaving the old name here would be worse than
    # useless: it would keep asserting something about an identifier that no
    # longer exists, and pass for that reason. Part 7 asserts the new field's
    # epoch instead.
    _ROS_TIME_NAMES = ("_ros_now", "_shutdown_now", "_clock_ref_ros_sec",
                       "_started_at_sec", "_odom_stamp_sec")

    def _calls_ros_now(node) -> bool:
        return any(
            (isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
             and sub.func.attr in _ROS_TIME_NAMES)
            or (isinstance(sub, ast.Attribute) and sub.attr in _ROS_TIME_NAMES)
            for sub in ast.walk(node)
        )

    arming_calls = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Attribute)
            and node.func.value.attr == "_arming"
        ):
            arming_calls.append(node)
    check(
        len(arming_calls) >= 12,
        "F-29: the arming machine is reached from every path it should be",
        f"{len(arming_calls)} call sites",
    )
    # Message arguments are exempt: a detail string may name what the ROS clock
    # said - that is reporting, not arithmetic. Everything else is a value the
    # machine computes with.
    offenders = [
        node.func.attr for node in arming_calls
        if any(
            _calls_ros_now(arg)
            for arg in node.args
            if not isinstance(arg, (ast.JoinedStr, ast.Str))
        )
    ]
    check(
        not offenders,
        "F-29: NOT ONE call into the arming machine is given a ROS-clock time, "
        "as a call or as a stored stamp - "
        "the window is a promise in wall-clock seconds and a slow /clock must not "
        "stretch it (measured: a 20 s window unexpired after 123 s at RTF 0.1)",
        f"offenders={offenders}" if offenders else "",
    )
    check(
        "_safety_now" in functions
        and not _calls_ros_now(functions["_safety_now"])
        and any(
            isinstance(sub, ast.Attribute) and sub.attr == "monotonic"
            for sub in ast.walk(functions["_safety_now"])
        ),
        "  ... because they all take their time from `_safety_now`, which is "
        "`time.monotonic` and cannot be switched by a parameter",
    )
    check(
        [a.arg for a in functions["_is_armed"].args.args] == ["self"],
        "  ... and `_is_armed` takes no time argument at all, so no caller can "
        "hand it the wrong clock",
    )
    check(
        not _calls_ros_now(functions["_link_watchdog"]),
        "F-29: the link watchdog contains no ROS-clock read - `link_lost_sec` is "
        "a statement about a WiFi link, which does not slow down when Gazebo does",
    )
    check(
        "_note_clock_jumped_back" in functions
        and any(
            isinstance(sub, ast.Attribute) and sub.attr == "_clock_ref_ros_sec"
            and isinstance(sub.ctx, ast.Store)
            for sub in ast.walk(functions["_note_clock_jumped_back"])
        ),
        "F-30: a backwards jump is its own branch AND it RE-BASELINES the "
        "reference, so recovery is one tick and not the size of the jump",
    )

    # SR-15 rule 7, the user's split: the jump branch must reach the arming
    # machine through the JUMP entry point and the stall branch through the
    # STALL one. Checked structurally because the two are one line apart and a
    # copy-paste between them would restore exactly the reuse that was reversed,
    # while every runtime symptom stayed identical.
    def _arming_calls_in(fn_name):
        return {
            sub.func.attr
            for sub in ast.walk(functions[fn_name])
            if isinstance(sub, ast.Call)
            and isinstance(sub.func, ast.Attribute)
            and sub.func.attr.startswith("note_clock")
        }

    jump_calls = _arming_calls_in("_note_clock_jumped_back")
    stall_calls = _arming_calls_in("_note_clock_stalled")
    check(
        jump_calls == {"note_clock_jumped_back"},
        "SR-15 r.7: the backwards-jump branch reports through "
        "`note_clock_jumped_back` and NOT through `note_clock` - the codes were "
        "split by user decision 2026-08-19 and the two paths must not re-merge",
        f"calls={sorted(jump_calls)}",
    )
    check(
        stall_calls == {"note_clock"},
        "  ... and the stall branch still reports through `note_clock`, so it "
        "keeps constant 8",
        f"calls={sorted(stall_calls)}",
    )

    # The wire is a .msg file that nothing in this pure-python suite imports, so
    # the one way `TRIGGER_CODES` can drift away from it is silently. Parsed as
    # text, deliberately: this must hold before anything is built.
    msg_path = os.path.join(
        _HERE, "..", "..", "gripperx_external_msgs", "msg", "ArmingState.msg"
    )
    msg_codes = {}
    with open(msg_path, encoding="utf-8") as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if line.startswith("uint8 TRIGGER_") and "=" in line:
                name, value = line[len("uint8 "):].split("=", 1)
                msg_codes[name.strip()[len("TRIGGER_"):]] = int(value.strip())
    check(
        msg_codes == arm_mod.TRIGGER_CODES,
        "SR-15 r.7: ArmingState.msg and `arming.TRIGGER_CODES` enumerate the SAME "
        "triggers with the SAME numbers - the python side is what fills the field, "
        "so a constant added to one and not the other is a wrong number on the wire",
        f"msg={sorted(msg_codes.items())}",
    )
    check(
        len(msg_codes) == 10 and msg_codes.get("CLOCK_JUMPED_BACK") == 9,
        "  ... and there are NINE auto-disarm triggers plus NONE, the count "
        "SR-15 rule 7 now names",
        f"{len(msg_codes) - 1} triggers + NONE",
    )

    stalled_fn = functions["_note_clock_stalled"]
    body = stalled_fn.body
    disarm_stmt_index = None
    latch_index = None
    for index, stmt in enumerate(body):
        if latch_index is None and any(
            isinstance(sub, ast.Attribute) and sub.attr == "_clock_disarm_done"
            and isinstance(sub.ctx, ast.Store)
            for sub in ast.walk(stmt)
        ):
            latch_index = index
        if disarm_stmt_index is None and any(
            isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
            and sub.func.attr == "_handle_disarm"
            for sub in ast.walk(stmt)
        ):
            disarm_stmt_index = index
    check(
        disarm_stmt_index is not None
        and latch_index is not None
        and latch_index > disarm_stmt_index,
        "F-32: the 'this disarm has been done' latch is set AFTER the disarm, not "
        "before it - a raise there used to make the branch unreachable for ever "
        "and cost that cancel (the F-26 shape in new code)",
        f"disarm at stmt {disarm_stmt_index}, latch at stmt {latch_index}",
    )
    check(
        isinstance(body[disarm_stmt_index], ast.Try),
        "  ... and the disarm is inside a try/except, so the raise costs a retry "
        "on the next tick instead of the disarm",
    )
    first_statement = body[1] if isinstance(body[0], ast.Expr) else body[0]
    check(
        any(
            isinstance(sub, ast.Attribute) and sub.attr == "_clock_stalled"
            and isinstance(sub.ctx, ast.Store)
            for sub in ast.walk(first_statement)
        ),
        "  ... while the REPORTED stall flag is still set first: everything it "
        "gates is a refusal, so it must survive anything below it failing",
    )
    print()


def part7_f37_f38() -> None:
    """SAFETY.md F-37 and F-38 - the two LOW findings fixed in this pass.

    F-37  `ArmingState.expires_at` was projected with an ASSUMED rate of 1.0.
          At a real-time factor of 0.1 it advertised a gate closing ~595
          wall-seconds away for one that closed in 60.2 s. The window itself was
          and stays correct and monotonic (SR-15 rule 12) - what is fixed is
          only what the message TELLS an operator or a consumer.
    F-38  `max_teleop_mode_age_sec` was the third wall-clock promise left on the
          ROS clock, so a dead `teleop_mux` was detected after 22.1 s of wall
          time against a configured 2.0 s at that same factor.

    Both halves are checked: the ARITHMETIC through the pure functions, and the
    STRUCTURE that decides which clock reaches them. The structural half is the
    half that fails on the pre-fix build - the pure module did not exist there,
    and the two expressions it replaces are still findable in the AST.
    """
    import ast
    import inspect

    from gripperx_external import clock_rate as rate_mod  # noqa: E402
    from gripperx_external import goal_gateway_node as gw_mod  # noqa: E402

    print("=" * 78)
    print("Part 7 - F-37 (advertised expiry) and F-38 (mode age on the wrong clock)")
    print("=" * 78)

    # -- F-37, the arithmetic -------------------------------------------------
    check(
        rate_mod.project_ros_expiry(1000.0, 60.0, 1.0) == 1060.0,
        "F-37: at a rate of 1.0 the projection is the old arithmetic exactly, so "
        "the REAL ROBOT's expires_at is unchanged - there the ROS clock IS the "
        "wall clock",
    )
    check(
        abs(rate_mod.project_ros_expiry(1000.0, 60.0, 0.1) - 1006.0) < 1e-9,
        "F-37: at a rate of 0.1 a window with 60 WALL seconds left expires 6 ROS "
        "seconds from now, not 60. THIS is the check that fails on the old code: "
        "`_ros_now() + seconds_remaining` returns 1060.0, i.e. an instant the ROS "
        "clock reaches after ~595 wall-seconds",
        f"{rate_mod.project_ros_expiry(1000.0, 60.0, 0.1)}",
    )
    check(
        rate_mod.project_ros_expiry(1000.0, 60.0, 4.0) == 1240.0,
        "  ... and a clock running FAST is projected the same way, which is the "
        "same bug with the sign reversed",
    )
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        check(
            rate_mod.project_ros_expiry(1000.0, 60.0, bad) == 1060.0,
            f"  ... and an unusable rate ({bad}) falls back to 1.0 rather than "
            "producing an instant in the past or a NaN on the wire",
        )
    check(
        rate_mod.project_ros_expiry(1000.0, -5.0, 1.0) == 1000.0,
        "  ... and a negative remaining time cannot project an expiry BEFORE now",
    )

    # -- F-37, the estimate ---------------------------------------------------
    def _run(estimator, rate, seconds, step=0.1, ros0=1000.0):
        ros, mono = ros0, 0.0
        for _ in range(int(seconds / step)):
            mono += step
            ros += step * rate
            estimator.note(ros, mono)
        return ros, mono

    slow = rate_mod.ClockRateEstimator()
    check(slow.rate == 1.0, "F-37: the estimator is SEEDED at 1.0 - the real "
          "robot's value - so no window exists in which expires_at is unset")
    _run(slow, 0.1, 20.0)
    check(
        abs(slow.rate - 0.1) < 1.0e-3,
        "F-37: 20 s of a clock at a real-time factor of 0.1 is estimated as 0.1",
        f"rate={slow.rate:.5f} from {slow.samples} samples",
    )
    fast = rate_mod.ClockRateEstimator()
    _run(fast, 1.0, 20.0)
    check(
        abs(fast.rate - 1.0) < 1.0e-3,
        "  ... and a clock keeping up is estimated as 1.0, so the twin at 1.0x "
        "and the real robot agree",
        f"rate={fast.rate:.5f}",
    )

    ros, mono = _run(slow, 0.1, 5.0)
    before = slow.rate
    dropped_before = slow.dropped
    slow.note(ros + 120.0, mono + 0.1)
    check(
        slow.rate == before and slow.dropped == dropped_before + 1,
        "F-37: a 120 s FORWARD step is not a rate and is excluded from the "
        "estimate - otherwise one discontinuity would corrupt expires_at for the "
        "length of the smoothing window",
        f"rate {before:.5f} -> {slow.rate:.5f}, dropped={slow.dropped}",
    )
    # The line F-40 must not be crossed by accident: excluding a sample is
    # numerical hygiene, not detection. If this module ever grows a logger, a
    # threshold parameter or a callback, it has made a decision that is the
    # user's to make (SR-15 rule 12, F-40 still OPEN).
    rate_source = inspect.getsource(rate_mod)
    check(
        not any(word in rate_source for word in
                ("get_logger", "self.disarm", "diagnostic", "warn(")),
        "F-40 is NOT pre-empted: the rate estimator neither logs, reports nor "
        "disarms on the discontinuity it declines to average - what a forward "
        "jump should cause is an open user decision, not a side effect of F-37",
    )

    jumped = rate_mod.ClockRateEstimator()
    _run(jumped, 0.5, 10.0)
    kept = jumped.rate
    jumped.reset()
    check(
        jumped.rate == kept,
        "F-30/F-37: `reset` after a BACKWARDS jump forgets the reference pair "
        "and keeps the estimate - a world reset changes what the clock says, "
        "not how fast it says it",
    )
    check(
        min(_estimates(rate_mod)) > 0.0,
        "  ... and the estimate is never zero or negative, whatever it is fed - "
        "a rate of 0 would project an expiry at `now` for an open window",
    )

    # -- F-37, the structure that fails on the old code -----------------------
    tree = ast.parse(inspect.getsource(gw_mod))
    functions = {
        node.name: node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
    }
    msg_fn = functions["_arming_state_msg"]
    check(
        any(
            isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
            and sub.func.attr == "project_ros_expiry"
            for sub in ast.walk(msg_fn)
        ),
        "F-37: `_arming_state_msg` projects through `project_ros_expiry`, so the "
        "conversion is in one place and is testable without a node",
    )
    bare_sums = [
        sub for sub in ast.walk(msg_fn)
        if isinstance(sub, ast.BinOp) and isinstance(sub.op, ast.Add)
        and any(
            isinstance(operand, ast.Call)
            and isinstance(operand.func, ast.Attribute)
            and operand.func.attr == "_ros_now"
            for operand in (sub.left, sub.right)
        )
    ]
    check(
        not bare_sums,
        "F-37: nothing in `_arming_state_msg` adds anything to a raw `_ros_now()` "
        "any more. This is the pre-fix expression itself - `expires = "
        "self._ros_now() + float(snap['seconds_remaining'])` - and it is the "
        "check that FAILS on the build the finding was written against",
        f"{len(bare_sums)} bare sum(s) on a ROS instant",
    )
    check(
        any(
            isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
            and sub.func.attr == "note"
            and isinstance(sub.func.value, ast.Attribute)
            and sub.func.value.attr == "_clock_rate"
            for sub in ast.walk(functions["_note_clock_advancing"])
        ),
        "F-37: the rate is observed in the clock watchdog's HEALTHY branch, which "
        "is the one place holding a ROS instant and a monotonic instant taken "
        "together - no second measurement was added for it",
    )
    check(
        any(
            isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
            and sub.func.attr == "reset"
            and isinstance(sub.func.value, ast.Attribute)
            and sub.func.value.attr == "_clock_rate"
            for sub in ast.walk(functions["_note_clock_jumped_back"])
        ),
        "  ... and a backwards jump drops the reference pair, so the next sample "
        "is not measured ACROSS the discontinuity",
    )

    # -- F-38, the structure --------------------------------------------------
    source = inspect.getsource(gw_mod)
    check(
        "_teleop_mode_stamp_sec" not in source,
        "F-38: the ROS-stamped field is GONE, name and all. The name carries the "
        "epoch now (`_teleop_mode_mono_sec`) for the reason `_safety_tick` calls "
        "neither of its two times `now`",
    )
    subs = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Sub)
        and isinstance(node.right, ast.Attribute)
        and node.right.attr == "_teleop_mode_mono_sec"
    ]
    check(
        len(subs) >= 3,
        "F-38: every place that measures the mode age is found - the dispatch "
        "gate, the in-flight re-check and the telemetry",
        f"{len(subs)} age computations",
    )
    wrong_clock = [
        node for node in subs
        if not (
            isinstance(node.left, ast.Call)
            and isinstance(node.left.func, ast.Attribute)
            and node.left.func.attr == "_safety_now"
        )
    ]
    check(
        # The `len(subs)` half is not decoration: without it this check passes
        # VACUOUSLY on the pre-fix build, where the field is called
        # `_teleop_mode_stamp_sec` and the list is therefore empty. A check that
        # passes on the code it was written to reject proves nothing.
        subs and not wrong_clock,
        "F-38: EVERY one of them subtracts it from `_safety_now()` and not from a "
        "ROS instant. `/teleop/active_mode` carries no stamp, so the age is our "
        "own reception time and the consumer's clock is the only clock in the "
        "comparison - which is the argument that already moved the link watchdog "
        "(SR-15 rule 12). Measured before the fix: a dead mux caught after 22.1 s "
        "against a configured 2.0 s at RTF 0.1",
        f"{len(wrong_clock)} on the wrong clock",
    )
    check(
        not _calls_ros_time(functions["_on_teleop_mode"], _ROS_TIME_NAMES_P7),
        "F-38: the mode callback reads no ROS clock at all any more",
    )
    check(
        [a.arg for a in functions["_dispatch_context"].args.args] == ["self", "resolution", "pose"],
        "  ... and `_dispatch_context` no longer takes a ROS-time `now`, so no "
        "caller can hand it the wrong clock - the same discipline `_is_armed` got",
        f"args={[a.arg for a in functions['_dispatch_context'].args.args]}",
    )
    print()


#: Kept apart from part 6's list on purpose: this one is about which clock a
#: FUNCTION reads, not about what is passed into the arming machine.
_ROS_TIME_NAMES_P7 = ("_ros_now", "_clock_ref_ros_sec", "_started_at_sec")


def _calls_ros_time(node, names) -> bool:
    import ast

    return any(
        (isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
         and sub.func.attr in names)
        or (isinstance(sub, ast.Attribute) and sub.attr in names)
        for sub in ast.walk(node)
    )


def _estimates(rate_mod) -> list:
    """Rates the estimator reports when fed everything unpleasant."""
    values = []
    for feed in (
        [(0.0, 0.1)] * 40,                       # a clock that does not move
        [(1e6, 0.1)] * 40,                       # nothing but discontinuities
        [(float("nan"), 0.1)] * 40,              # a clock that reads NaN
        [(0.05, 0.1), (0.0, 0.1), (0.05, 0.1)],  # a stutter
    ):
        estimator = rate_mod.ClockRateEstimator()
        ros, mono = 1000.0, 0.0
        for d_ros, d_mono in feed:
            ros += d_ros
            mono += d_mono
            estimator.note(ros, mono)
        values.append(estimator.rate)
    return values


def part8_f40() -> None:
    """SAFETY.md F-40 - the forward clock jump, DECIDED by the user 2026-08-20.

    The decision: **report it - WARN plus a `/diagnostics` value - and do NOT
    disarm and do NOT cancel.** It was taken on a reproduction rather than on an
    argument, and the number that made the case is in the reproduction: the
    accidental `MODE_STALE` cancel that used to cover the case half the time
    went from **4 of 8** identical jumps to **0 of 8** once F-38 was fixed, so
    nothing incidental covers it any more.

    Three things have to hold at once and each is checked separately, because
    they can break independently:

    1. the discontinuity is DETECTED and REPORTED - it produced literally
       nothing before, measured;
    2. the GATE IS UNCHANGED - no disarm, no cancel, no new refusal, no tenth
       `ArmingState` trigger. This is a reporting change and nothing else;
    3. the DETECTOR IS IN THE WATCHDOG, not in `clock_rate`. That module was
       deliberately built as a non-detector for F-37 and part 7 asserts it has
       no logger, no diagnostic and no disarm - that check must keep passing,
       so the boundary is checked from both sides here.

    Checks below are marked NON-FALSIFYING where they also pass against the
    pre-decision build. Those are invariant guards - they exist to catch the
    gate being changed later - and they are NOT evidence that the decision was
    implemented. The evidence is the ones that fail without it.
    """
    import ast
    import inspect
    import threading
    import types

    from gripperx_external import clock_rate as rate_mod  # noqa: E402
    from gripperx_external import goal_gateway_node as gw_mod  # noqa: E402
    from gripperx_external.goal_gateway_node import GoalGatewayNode  # noqa: E402

    print("=" * 78)
    print("Part 8 - F-40: a forward clock jump is REPORTED (user decision 2026-08-20)")
    print("=" * 78)

    source = inspect.getsource(gw_mod)
    tree = ast.parse(source)
    functions = {
        node.name: node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
    }

    # -- 1. the parameter: named, startup-only, not derived (SR-15 rule 14) ----
    check(
        'declare_parameter("clock_forward_jump_sec"' in source,
        "F-40/rule 14: the threshold is its OWN named parameter, "
        "`clock_forward_jump_sec` - the decision's enabling threshold has to be "
        "visible to be reviewable",
    )
    startup_dict = next(
        (node for node in ast.walk(tree)
         if isinstance(node, ast.Assign)
         and any(isinstance(t, ast.Name) and t.id == "_STARTUP_ONLY_PARAMS"
                 for t in node.targets)),
        None,
    )
    startup_keys = {
        key.value for key in (startup_dict.value.keys if startup_dict else [])
        if isinstance(key, ast.Constant)
    }
    check(
        "clock_forward_jump_sec" in startup_keys,
        "  ... and it is in `_STARTUP_ONLY_PARAMS`, so a running node cannot "
        "move the threshold it is being judged by (SR-15 rule 14, the rule "
        "`_clock_backward_eps_sec` produced)",
        f"{len(startup_keys)} startup-only parameters",
    )
    assignments = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Attribute) and t.attr == "_clock_forward_jump_sec"
                for t in node.targets)
    ]
    fetched = [
        arg.value for node in assignments for sub in ast.walk(node)
        if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
        and sub.func.attr == "get_parameter"
        for arg in sub.args if isinstance(arg, ast.Constant)
    ]
    check(
        assignments and fetched == ["clock_forward_jump_sec"],
        "F-36/rule 14: it is NOT DERIVED - the value comes from its own "
        "parameter and from no other, so an edit to `safety_rate_hz` cannot "
        "silently move a clock tolerance the way it moved the backwards one",
        f"reads {fetched or 'nothing'}",
    )

    # -- 2. the detector exists, in the watchdog ------------------------------
    check(
        "_note_clock_jumped_forward" in functions,
        "F-40: there IS a forward-jump branch. Before the decision a forward "
        "jump was indistinguishable from healthy progress - measured, 0 of 8 "
        "jumps produced a log line, a diagnostic or anything else",
    )
    watchdog = functions["_clock_watchdog"]
    calls_forward = [
        node for node in ast.walk(watchdog)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr == "_note_clock_jumped_forward"
    ]
    check(
        len(calls_forward) == 1,
        "  ... and the DECISION is taken in the clock watchdog, which is where "
        "the other two verdicts are taken",
        f"{len(calls_forward)} call site(s)",
    )
    proven_guard = any(
        isinstance(node, ast.Attribute) and node.attr == "_clock_proven"
        for node in ast.walk(watchdog)
    )
    check(
        proven_guard,
        "  ... guarded by `_clock_proven`: before the clock is proven the first "
        "advance IS the proof, and with a late `/clock` that step is the whole "
        "epoch - reporting it would put a WARN on every ordinary sim start, "
        "which is the mistake F-31 exists about",
    )
    check(
        any(
            isinstance(node, ast.Try)
            and any(isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
                    and sub.func.attr == "_note_clock_jumped_forward"
                    for sub in ast.walk(node))
            for node in ast.walk(watchdog)
        ),
        "  ... and the report is inside a try/except, so a raise in it costs the "
        "REPORT and not the watchdog - a raise escaping the timer callback would "
        "cost every later stall and backwards jump too (the F-26/F-32 shape)",
    )

    # -- 3. the gate is NOT touched -------------------------------------------
    forward_fn = functions.get("_note_clock_jumped_forward")
    touched = sorted({
        node.func.attr for node in ast.walk(forward_fn or ast.Module(body=[], type_ignores=[]))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and (node.func.attr in ("_handle_disarm", "_cancel_mission", "disarm", "poll")
             or node.func.attr.startswith("note_clock"))
    })
    check(
        forward_fn is not None and not touched,
        "F-40: the forward-jump branch does NOT disarm and does NOT cancel. The "
        "user's decision is a reporting change and nothing else: no "
        "`_handle_disarm`, no `_cancel_mission`, no entry into the arming "
        "machine. (The `is not None` half matters - without it this passes on a "
        "build that has no branch at all.)",
        f"gate calls found: {touched or 'none'}",
    )
    check(
        forward_fn is not None and any(
            isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "warn"
            for node in ast.walk(forward_fn)
        ),
        "F-40: it reports at WARN, which is the severity the user chose",
    )
    check(
        forward_fn is not None and not any(
            isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "error"
            for node in ast.walk(forward_fn)
        ),
        "  ... and NOT at ERROR: an ERROR is what a stall and a backwards jump "
        "get, and both of those close the gate. Grading this the same would say "
        "something about the gate that is not true",
    )
    check(
        len(arm_mod.TRIGGER_CODES) == 10
        and not any("FORWARD" in name for name in arm_mod.TRIGGER_CODES),
        "F-40 (NON-FALSIFYING invariant guard - also passes pre-decision): there "
        "is NO tenth `ArmingState` trigger. A constant would say a disarm can "
        "carry this reason, and by the user's decision it cannot",
        f"{len(arm_mod.TRIGGER_CODES)} codes",
    )

    # -- 4. the /diagnostics value, from the real pure function ---------------
    status = diag_mod.clock_status(
        proven=True, stalled=False, frozen_for_sec=0.0, use_sim_time=True,
        stall_sec=2.0, forward_jumps=3, last_forward_jump_sec=119.75,
        forward_jump_sec=1.0,
    )
    values = {kv.key: kv.value for kv in status.values}
    check(
        values.get("forward_jumps") == "3",
        "F-40: `/diagnostics` `external/clock` carries a forward-jump COUNT. "
        "There was no key here at all before the decision, so an operator had no "
        "way to learn one had happened",
        f"forward_jumps={values.get('forward_jumps')}",
    )
    check(
        values.get("last_forward_jump_sec") == "119.75",
        "  ... and the SIZE of the last one, because the question after the fact "
        "is 'did the clock step while that goal was running' and a value that "
        "exists only during the event cannot answer it",
        f"last_forward_jump_sec={values.get('last_forward_jump_sec')}",
    )
    check(
        # "1", not "1.0": `_value` formats floats with %.6g throughout, which is
        # why `clock_stall_sec` reads "2". Asserting the package's own
        # convention rather than a prettier one.
        values.get("clock_forward_jump_sec") == "1",
        "  ... and the THRESHOLD next to them, so the number that decides "
        "whether anything is reported is visible where the report is",
    )
    check(
        status.level == diag_mod.OK,
        "  ... while the LEVEL is unchanged: report-only means the status that "
        "gates nothing must not start claiming a fault, and there is no moment "
        "at which a forward jump 'clears', so a latched WARN here would be "
        "permanent and would bury the stall and never-proven cases that DO gate",
    )
    unset = diag_mod.clock_status(
        proven=True, stalled=False, frozen_for_sec=0.0, use_sim_time=False,
        stall_sec=2.0,
    )
    check(
        {kv.key: kv.value for kv in unset.values}.get("clock_forward_jump_sec")
        == "unset",
        "  ... and an unset threshold says `unset` rather than defaulting to a "
        "number, the discipline every TO-VERIFY value in this package follows",
    )

    # -- 5. the boundary with clock_rate holds from both sides ----------------
    rate_source = inspect.getsource(rate_mod)
    check(
        "_note_clock_jumped_forward" not in rate_source
        and "forward_jumps" not in rate_source,
        "F-40/F-37 (NON-FALSIFYING invariant guard): the DETECTOR is not in "
        "`clock_rate`. That module was built as a pure estimator on purpose and "
        "part 7 asserts it has no logger, no diagnostic and no disarm; the "
        "decision was implemented in the watchdog so that check keeps passing",
    )
    check(
        any(
            isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "reset"
            and isinstance(node.func.value, ast.Attribute)
            and node.func.value.attr == "_clock_rate"
            for node in ast.walk(watchdog)
        ),
        "  ... and the watchdog tells the estimator to forget its reference on a "
        "forward jump, so the discontinuity is not averaged into the F-37 rate - "
        "the estimator is TOLD, it does not decide",
    )

    # -- 6. the real _clock_watchdog, run on a stand-in ------------------------
    # No rclpy context, no node, no DDS - the F-26 technique. This is behaviour
    # rather than structure: the three structural checks above could all hold on
    # a branch that never fires.
    class _Logger:
        def __init__(self, raising=False):
            self.warns = []
            self.errors = []
            self.infos = []
            self._raising = raising

        def warn(self, message):
            if self._raising:
                raise RuntimeError("logger says no (the F-22 shape)")
            self.warns.append(message)

        def error(self, message):
            self.errors.append(message)

        def info(self, message):
            self.infos.append(message)

    def _fake(raising=False, proven=True):
        fake = type("_FakeGateway", (), {})()
        fake._logger = _Logger(raising)
        fake.get_logger = lambda: fake._logger
        fake._use_sim_time = True
        fake._clock_proven = proven
        fake._clock_stalled = False
        fake._clock_stall_sec = 2.0
        fake._clock_backward_eps_sec = 0.1
        fake._clock_forward_jump_sec = 1.0
        fake._clock_stall_severity_logged = ""
        fake._clock_disarm_done = False
        fake._clock_forward_jumps = 0
        fake._last_forward_jump_sec = 0.0
        fake._clock_forward_report_errors = 0
        fake._clock_rate = rate_mod.ClockRateEstimator()
        fake._started_mono = 0.0
        fake._clock_startup_grace_sec = 10.0
        fake._disarms = []
        fake._handle_disarm = lambda event: fake._disarms.append(event)
        fake._cancel_mission = lambda *a, **k: fake._disarms.append("cancel")
        for name in ("_clock_watchdog", "_note_clock_advancing",
                     "_note_clock_jumped_forward", "_note_clock_jumped_back",
                     "_note_clock_stalled", "_clock_in_startup_grace"):
            setattr(fake, name, types.MethodType(getattr(GoalGatewayNode, name), fake))
        return fake

    class _FakeTime:
        """A controlled `time.monotonic`, so BOTH clocks are ours.

        Without this the loop below runs in microseconds of real monotonic time,
        every `expected advance` comes out ~0, and the rate-aware threshold
        degenerates into an absolute one - i.e. the check would pass on an
        implementation that has the false-positive mode this one avoids.
        """

        def __init__(self):
            self.value = 0.0

        def monotonic(self):
            return self.value

    def _drive(fake, steps, rate=1.0, period=0.1, jump_at=None, jump=0.0):
        """Tick the REAL `_clock_watchdog` with a fully controlled clock pair."""
        clock = _FakeTime()
        real_time = gw_mod.time
        gw_mod.time = clock
        try:
            ros = 1000.0
            fake._clock_ref_ros_sec = ros
            fake._clock_ref_mono = clock.value
            fake._started_mono = clock.value
            for index in range(steps):
                clock.value += period
                ros += period * rate
                if jump_at is not None and index == jump_at:
                    ros += jump
                fake._ros_now = (lambda value: (lambda: value))(ros)
                fake._clock_watchdog()
        finally:
            gw_mod.time = real_time
        return fake

    quiet = _drive(_fake(), steps=25, rate=1.0)
    check(
        quiet._clock_forward_jumps == 0 and not quiet._logger.warns,
        "F-40: an ordinary clock advancing at 1.0x reports NOTHING - the "
        "detector must not fire on the case that is 99.9% of every run",
        f"{quiet._clock_forward_jumps} jump(s), {len(quiet._logger.warns)} WARN(s)",
    )
    jumped = _drive(_fake(), steps=25, rate=1.0, jump_at=12, jump=120.0)
    check(
        jumped._clock_forward_jumps == 1,
        "F-40: a 120 s forward step IS detected, running the REAL "
        "`_clock_watchdog` on a stand-in - no context, no node, no DDS",
        f"{jumped._clock_forward_jumps} detected",
    )
    check(
        len(jumped._logger.warns) == 1
        and "JUMPED FORWARD" in jumped._logger.warns[0]
        and "REPORT ONLY" in jumped._logger.warns[0],
        "  ... reported exactly ONCE, at WARN, saying in the line itself that it "
        "is report-only - a jump is an event, and a per-tick repeat would be the "
        "cry-wolf failure rule 12 names",
        f"{len(jumped._logger.warns)} WARN(s)",
    )
    check(
        jumped._last_forward_jump_sec > 100.0,
        "  ... and the size it recorded is the EXCESS over continuous progress, "
        "not the raw advance",
        f"last_forward_jump_sec={jumped._last_forward_jump_sec:.1f}",
    )
    check(
        not jumped._disarms,
        "F-40: and NOTHING was disarmed or cancelled by it. This is the whole of "
        "the user's decision, checked at runtime rather than read off the source",
        f"gate events: {jumped._disarms or 'none'}",
    )
    check(
        jumped._clock_proven and not jumped._clock_stalled,
        "  ... and the clock is still PROVEN and still re-baselined, so arming is "
        "not refused for it either",
    )
    # `jump_at=0` and not later, because that is the real shape: with a late
    # `/clock` the ROS clock is stuck until the first message arrives and then
    # steps by the whole epoch in ONE tick - the same tick that proves it. A
    # jump at tick 2 would already be past the proof and SHOULD be reported.
    unproven = _drive(_fake(proven=False), steps=6, rate=1.0, jump_at=0, jump=120.0)
    check(
        unproven._clock_forward_jumps == 0,
        "F-40: a step on a clock that has never been proven is NOT reported - it "
        "is the proof itself, and with a late `/clock` that step is the whole "
        "epoch (F-31: no WARN on ordinary starts)",
        f"{unproven._clock_forward_jumps} reported",
    )
    slow = _drive(_fake(), steps=40, rate=0.1)
    check(
        slow._clock_forward_jumps == 0,
        "F-40: a twin at a real-time factor of 0.1 reports nothing either - the "
        "threshold is an EXCESS over the observed rate, so a clock that is "
        "merely slow, or merely fast, is not a discontinuity",
        f"{slow._clock_forward_jumps} reported",
    )
    fast = _drive(_fake(), steps=60, rate=5.0)
    check(
        fast._clock_forward_jumps == 0,
        "  ... and neither does a twin deliberately run at 5x, where every tick "
        "advances the ROS clock by 0.5 s. An ABSOLUTE step threshold below that "
        "would report every single tick, and a mechanism that cries wolf gets "
        "disabled by whoever is on shift - SR-15 rule 12's own words. This is "
        "why the threshold is an excess over the OBSERVED rate",
        f"{fast._clock_forward_jumps} reported over 60 ticks at 5x",
    )
    raising = _drive(_fake(raising=True), steps=25, rate=1.0, jump_at=12, jump=120.0)
    check(
        raising._clock_proven and raising._clock_forward_report_errors == 1,
        "F-40: with the WARN itself raising, the watchdog survives, the clock is "
        "still re-baselined and proven, and the failure is counted - the report "
        "must never be able to cost the thing it reports on (F-26/F-32)",
        f"{raising._clock_forward_report_errors} report failure(s)",
    )
    print()


def part9_f36() -> None:
    """SR-15 rule 14 / audit finding F-36 - `_clock_backward_eps_sec`.

    `REQUIREMENTS` (internal) rev 22 records it as its own startup-only parameter, NOT
    derived, while the code still computed `1.0 / max(2.0, safety_rate_hz)` and
    did not list it in `_STARTUP_ONLY_PARAMS`. The requirement described a build
    that did not exist. This part is that gap closed and kept closed.

    IT IS A PARAMETERISATION, NOT A BEHAVIOUR CHANGE, and one check below exists
    only to prove that: the default is 0.2, which is EXACTLY what the derivation
    produced at the `safety_rate_hz: 5.0` both config files set, so the
    threshold this build runs is numerically identical to the one the previous
    build ran. The backwards branch still disarms and still cancels.

    Checks marked NON-FALSIFYING also pass on the pre-fix build. Those are the
    behaviour-unchanged guards - they are the point of this part, but they are
    not evidence that the parameterisation happened. The evidence is the rest.
    """
    import ast
    import inspect
    import os
    import types

    from gripperx_external import clock_rate as rate_mod  # noqa: E402
    from gripperx_external import goal_gateway_node as gw_mod  # noqa: E402
    from gripperx_external.goal_gateway_node import GoalGatewayNode  # noqa: E402

    print("=" * 78)
    print("Part 9 - F-36: the backwards-jump tolerance is its OWN parameter")
    print("=" * 78)

    source = inspect.getsource(gw_mod)
    tree = ast.parse(source)
    functions = {
        node.name: node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
    }

    # -- the parameter itself -------------------------------------------------
    check(
        'declare_parameter("clock_backward_eps_sec"' in source,
        "F-36/rule 14: the backwards tolerance is its OWN named parameter. It "
        "alone decides whether CLOCK_JUMPED_BACK - the ninth trigger, split out "
        "by an explicit user decision - ever fires, and it was a derived local "
        "with no parameter and no mention in the requirement at all",
    )
    startup_dict = next(
        (node for node in ast.walk(tree)
         if isinstance(node, ast.Assign)
         and any(isinstance(t, ast.Name) and t.id == "_STARTUP_ONLY_PARAMS"
                 for t in node.targets)),
        None,
    )
    startup_keys = {
        key.value for key in (startup_dict.value.keys if startup_dict else [])
        if isinstance(key, ast.Constant)
    }
    check(
        "clock_backward_eps_sec" in startup_keys,
        "  ... and it is in `_STARTUP_ONLY_PARAMS`, so `ros2 param set` on it is "
        "REFUSED rather than applied - a threshold that can move while the gate "
        "is being relied on is not a threshold (SR-15 rule 14)",
        f"{len(startup_keys)} startup-only parameters",
    )

    # -- F-36 itself: a RATE must not move a TOLERANCE ------------------------
    assignments = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Attribute)
                and t.attr == "_clock_backward_eps_sec"
                for t in node.targets)
    ]
    fetched = [
        arg.value for node in assignments for sub in ast.walk(node)
        if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
        and sub.func.attr == "get_parameter"
        for arg in sub.args if isinstance(arg, ast.Constant)
    ]
    check(
        assignments and fetched == ["clock_backward_eps_sec"],
        "F-36: it is NOT DERIVED - the value comes from its own parameter and "
        "from NO other. It was `1.0 / max(2.0, safety_rate_hz)`, so an edit to "
        "the SAFETY TICK RATE moved the BACKWARDS-JUMP DETECTION THRESHOLD: two "
        "different quantities, and an edit that does not look like an edit to a "
        "clock tolerance",
        f"reads {fetched or 'nothing'}",
    )
    check(
        assignments and not any(
            isinstance(sub, ast.Constant) and sub.value == "safety_rate_hz"
            for node in assignments for sub in ast.walk(node)
        ),
        "  ... and `safety_rate_hz` does not appear in that statement AT ALL, "
        "which is the mistake stated as a string rather than as a shape",
    )

    # -- parameterisation, not behaviour: the number is the same one ----------
    def _default(name):
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "declare_parameter"
                    and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == name
                    and len(node.args) > 1
                    and isinstance(node.args[1], ast.Constant)):
                return node.args[1].value
        return None

    eps_default = _default("clock_backward_eps_sec")
    rate_default = _default("safety_rate_hz")
    check(
        eps_default is not None and rate_default is not None
        and eps_default == 1.0 / max(2.0, rate_default),
        "F-36: PARAMETERISATION, NOT BEHAVIOUR CHANGE - the new default is "
        "EXACTLY what the old derivation produced at the configured rate, so the "
        "threshold this build runs is numerically identical to the one the "
        "previous build ran. Nothing measured it then and nothing measures it "
        "now; it is `TO-VERIFY` either way. What changed is that it is visible "
        "and cannot be moved by editing something else",
        f"default {eps_default} == 1.0/max(2.0, safety_rate_hz={rate_default})",
    )

    # -- visibility: the configs and the startup line -------------------------
    config_dir = os.path.join(_HERE, "..", "config")
    for name in ("octopus_link_twin.yaml", "octopus_link_real.yaml"):
        with open(os.path.join(config_dir, name), encoding="utf-8") as fh:
            text = fh.read()
        check(
            "clock_backward_eps_sec:" in text and "clock_forward_jump_sec:" in text,
            f"  ... and BOTH clock-discontinuity thresholds appear in {name}. "
            "Visibility is part of rule 14: a decision whose enabling threshold "
            "is invisible is a decision that cannot be reviewed",
        )
        # Anchored to the KEY's own comment block, not to "somewhere in the
        # last 1200 characters". The loose version passed on a config that did
        # not contain the key at all - `split` returned the whole file and some
        # other TO-VERIFY satisfied it. Caught by running it against the pre-fix
        # build, which is the only way that shape ever gets caught.
        lines = text.splitlines()
        index = next(
            (i for i, line in enumerate(lines)
             if line.strip().startswith("clock_backward_eps_sec:")),
            None,
        )
        preceding = "\n".join(lines[max(0, (index or 0) - 25):index or 0])
        check(
            index is not None and "TO-VERIFY" in preceding,
            f"  ... labelled `TO-VERIFY` in its own comment block in {name}, "
            "because nothing measured either of them",
            "key absent" if index is None else "labelled",
        )
    check(
        "clock_backward_eps_sec=" in source,
        "  ... and the gateway names it in its own startup line, so the value in "
        "force is in the log rather than only in a config file that may or may "
        "not have been the one loaded",
    )

    # -- NON-FALSIFYING: the backwards branch still disarms and cancels -------
    back_fn = functions["_note_clock_jumped_back"]
    check(
        any(
            isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "note_clock_jumped_back"
            for node in ast.walk(back_fn)
        )
        and any(
            isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "_handle_disarm"
            for node in ast.walk(back_fn)
        ),
        "F-36 (NON-FALSIFYING invariant guard - also passes pre-fix): the "
        "BACKWARDS branch still reports through `note_clock_jumped_back` AND "
        "still reaches `_handle_disarm`, which is what cancels. This was a "
        "parameterisation change and the behaviour must be exactly where it was",
    )

    # -- NON-FALSIFYING: and the threshold still decides the same way ---------
    class _Logger:
        def __init__(self):
            self.lines = []

        def warn(self, message):
            self.lines.append(message)

        error = info = warn

    def _fake(eps):
        fake = type("_FakeGateway", (), {})()
        fake._logger = _Logger()
        fake.get_logger = lambda: fake._logger
        fake._use_sim_time = True
        fake._clock_proven = True
        fake._clock_stalled = False
        fake._clock_stall_sec = 2.0
        fake._clock_backward_eps_sec = eps
        fake._clock_forward_jump_sec = 1.0
        fake._clock_stall_severity_logged = ""
        fake._clock_disarm_done = False
        fake._clock_forward_jumps = 0
        fake._last_forward_jump_sec = 0.0
        fake._clock_forward_report_errors = 0
        fake._clock_rate = rate_mod.ClockRateEstimator()
        fake._started_mono = 0.0
        fake._clock_startup_grace_sec = 10.0
        # The backwards branch is STUBBED on purpose: what is under test here is
        # the ROUTING decision the threshold makes, not the disarm, which the
        # guard above and part 6 already cover.
        fake._backwards = []

        def _stub_jumped_back(ros, mono):
            fake._backwards.append(ros)
            # The stub RE-BASELINES, because the real method does and part 6
            # asserts it does. Without this the reference stays at the old
            # maximum and every later tick is still "backwards" - which is
            # exactly the pre-F-30 bug, so a stub that omitted it would report 3
            # verdicts for one rewind and measure the stub instead of the code.
            fake._clock_ref_ros_sec = ros
            fake._clock_ref_mono = mono

        fake._note_clock_jumped_back = _stub_jumped_back
        fake._stalls = []
        fake._note_clock_stalled = lambda frozen, mono: fake._stalls.append(frozen)
        for name in ("_clock_watchdog", "_note_clock_advancing",
                     "_note_clock_jumped_forward", "_clock_in_startup_grace"):
            setattr(fake, name, types.MethodType(getattr(GoalGatewayNode, name), fake))
        return fake

    class _FakeTime:
        def __init__(self):
            self.value = 0.0

        def monotonic(self):
            return self.value

    def _rewind(fake, by):
        clock = _FakeTime()
        real_time = gw_mod.time
        gw_mod.time = clock
        try:
            ros = 1000.0
            fake._clock_ref_ros_sec, fake._clock_ref_mono = ros, clock.value
            for step in range(6):
                clock.value += 0.1
                ros = ros + 0.1 if step != 3 else ros - by
                fake._ros_now = (lambda value: (lambda: value))(ros)
                fake._clock_watchdog()
        finally:
            gw_mod.time = real_time
        return fake

    small = _rewind(_fake(0.2), by=0.1)
    check(
        not small._backwards,
        "F-36 (NON-FALSIFYING invariant guard): a rewind SMALLER than the "
        "tolerance is not a jump - `/clock` is BEST_EFFORT, so a message "
        "overtaken by its successor rewinds it by about the publication "
        "spacing, and disarming on that would disarm on message reordering",
        f"{len(small._backwards)} backwards verdict(s) for a 0.1 s rewind at eps=0.2",
    )
    large = _rewind(_fake(0.2), by=120.0)
    check(
        len(large._backwards) == 1,
        "  ... and a rewind LARGER than it still is one, exactly as before the "
        "parameterisation - running the REAL `_clock_watchdog` on a stand-in",
        f"{len(large._backwards)} backwards verdict(s) for a 120 s rewind",
    )
    tight = _rewind(_fake(0.05), by=0.1)
    check(
        len(tight._backwards) == 1,
        "  ... and the PARAMETER is what decides: the same 0.1 s rewind that is "
        "ignored at eps=0.2 IS a jump at eps=0.05. Without this the two checks "
        "above would pass on a build that ignored the parameter entirely",
        f"{len(tight._backwards)} backwards verdict(s) for a 0.1 s rewind at eps=0.05",
    )

    # -- the positional-assert CLASS, eliminated rather than instance-fixed ---
    # Part 5 asserted `clock_status(...).values[-1]` and silently changed what it
    # checked when F-40 appended three keys. The fix was to look it up by key;
    # this is the fix for the CLASS - any reintroduction fails here rather than
    # passing quietly for a while first.
    for name in ("check_validation.py", "check_stage3_twin.py"):
        with open(os.path.join(_HERE, name), encoding="utf-8") as fh:
            checked = ast.parse(fh.read())
        positional = [
            node for node in ast.walk(checked)
            if isinstance(node, ast.Subscript)
            and isinstance(node.value, ast.Attribute)
            and node.value.attr in ("values", "status")
            and not isinstance(node.slice, ast.Slice)
        ]
        check(
            not positional,
            f"no positional indexing of a `.values`/`.status` list in {name}: a "
            "diagnostic status is a KEY/VALUE list and it grows, so `[-1]` "
            "asserts about whatever was appended last. Checked as a CLASS "
            "because fixing the one instance found would leave the shape legal",
            f"{len(positional)} positional read(s)",
        )
    print()


def part10_f35() -> None:
    """SAFETY.md F-35 - the MIRROR of package D's sim-time refusal.

    Package D refuses `use_sim_time: true` where no `/clock` can exist, and
    closes that direction twice over. The other direction - `use_sim_time:=false`
    while a `/clock` publisher is LIVE - was untouched, and the package did not
    look for a `/clock` publisher at all: `count_publishers` appeared once in the
    whole package, for `link_status`. **DECIDED 2026-08-20 (user): build the
    reverse check.** The alternative, accepting in writing that this direction
    goes undetected, was declined - the only argument for it was the premise
    "there is no /clock on the robot", which §6.4 item 8 names as exactly the
    premise that stops being true when somebody starts a bag replay.

    WHAT THIS PART DOES AND DOES NOT ESTABLISH. It establishes that the
    CONDITION is detected and reported, in both nodes, at both moments, without
    touching the gate. **It establishes nothing about the CONSEQUENCE of the
    mismatch.** The auditor predicts fail-safe behaviour - ages against a
    mismatched epoch come out large and `validate_goal` refuses - and says
    plainly that it is a prediction. Observing it needs the real Gazebo twin and
    **the user decided on 2026-08-20 not to make that run**, so F-35's
    consequence half stays `SUSPECTED` and F-35 stays OPEN.
    """
    import ast
    import inspect
    import types

    from gripperx_external import goal_gateway_node as gw_mod  # noqa: E402
    from gripperx_external import octopus_link_node as link_mod  # noqa: E402
    from gripperx_external.goal_gateway_node import GoalGatewayNode  # noqa: E402

    print("=" * 78)
    print("Part 10 - F-35: a LIVE /clock publisher while use_sim_time is false")
    print("=" * 78)

    warn = getattr(dom, "clock_publisher_warning", None)
    warn_exists = warn is not None
    check(
        warn is not None,
        "F-35: the reverse check EXISTS as a pure function. Package D's half has "
        "been there since package D; this is the half that was missing, and the "
        "package did not look for a /clock publisher anywhere at all",
    )
    # NO EARLY RETURN. A part that bails on the first failure reports "1 check
    # failed" against a build where 27 of them would have, which understates
    # what the change is worth and makes the falsification run uninformative.
    # The shim lets every check below run and FAIL honestly instead.
    if warn is None:
        warn = lambda *_a, **_k: None  # noqa: E731

    # -- the decision table, all four cells --------------------------------
    # Each of the three SILENT cells additionally requires the function to
    # exist. Without that they pass on a build that has no check at all - the
    # vacuum trap, which is easy to walk into precisely where the expected
    # answer is "nothing happens".
    check(
        warn(1, False, "at startup") is not None,
        "F-35: a live /clock publisher WITH use_sim_time false is reported - the "
        "twin's epoch and ours are unrelated, so every age computed across them "
        "is meaningless",
    )
    check(
        warn_exists and warn(0, False, "at startup") is None,
        "  ... no publisher and wall time is SILENT: that is the real robot and "
        "every existing scenario in the twin suite, and a check that fired there "
        "would be a check nobody reads",
    )
    check(
        warn_exists and warn(1, True, "at startup") is None,
        "  ... a publisher WITH sim time is silent too: that is the ordinary twin "
        "configuration, not a mismatch",
    )
    check(
        warn_exists and warn(0, True, "at startup") is None,
        "  ... and sim time with no publisher is package D's case, refused at "
        "startup there rather than warned about here - the two halves must not "
        "both fire on one fault",
    )
    check(
        "3 publisher(s)" in (warn(3, False, "at startup") or ""),
        "  ... and the COUNT is reported, because two /clock publishers is a "
        "different problem from one",
    )
    for when in ("at startup", "on the first /clock message"):
        check(
            when in (warn(1, False, when) or ""),
            f"  ... and the message names WHEN it was seen ('{when}'), because a "
            "silent start is not evidence of absence - DDS may not have matched "
            "the publisher yet",
        )

    # -- the decision it must NOT take -------------------------------------
    text = warn(1, False, "at startup") or ""
    check(
        "REPORTED and not refused" in text and "nothing is disarmed" in text,
        "F-35: the message says in the line itself that it is report-only. The "
        "forward direction REFUSES because sim time without /clock cannot work; "
        "this direction is a working clock measuring against the wrong epoch, "
        "which is a mismatch to report",
    )
    # BY AST, not by substring. The first version of both of these searched the
    # SOURCE TEXT and failed on the function's own prose - its message says
    # "nothing is disarmed, nothing is cancelled" and its docstring says
    # "rclpy-free". A check that greps for a word it also explains is a check
    # measuring its own comments: the same shape as the positional-index class
    # part 9 lints for, caught the same way, by running it.
    dom_tree = ast.parse(inspect.getsource(dom))
    warn_fn = next(
        (node for node in ast.walk(dom_tree)
         if isinstance(node, ast.FunctionDef)
         and node.name == "clock_publisher_warning"),
        None,
    )
    _empty = ast.Module(body=[], type_ignores=[])
    effects = sorted({
        node.func.id if isinstance(node.func, ast.Name) else node.func.attr
        for node in ast.walk(warn_fn or _empty)
        if isinstance(node, ast.Call) and isinstance(node.func, (ast.Name, ast.Attribute))
    } | {
        "raise" for node in ast.walk(warn_fn or _empty) if isinstance(node, ast.Raise)
    })
    check(
        warn_fn is not None and set(effects) <= {"int"},
        "F-35: the function itself cannot refuse, disarm or cancel - the only "
        "call in its entire body is `int()`, it raises nothing, and it returns a "
        "string or None. The gate is untouched, as it is for F-40",
        f"calls/effects: {effects or 'none'}",
    )
    imports = sorted({
        (node.module or "").split(".")[0] for node in ast.walk(dom_tree)
        if isinstance(node, ast.ImportFrom)
    } | {
        alias.name.split(".")[0] for node in ast.walk(dom_tree)
        if isinstance(node, ast.Import) for alias in node.names
    })
    check(
        "rclpy" not in imports,
        "F-35 (NON-FALSIFYING invariant guard): `domain_guard` still IMPORTS no "
        "rclpy, so both branches are exercised offline without a node, a context "
        "or a domain",
        f"imports: {imports}",
    )

    # -- both nodes, both firing points ------------------------------------
    for label, module in (("gateway", gw_mod), ("link node", link_mod)):
        tree = ast.parse(inspect.getsource(module))
        functions = {
            node.name: node for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
        }
        fn = functions.get("_warn_if_clock_publisher")
        check(
            fn is not None,
            f"F-35: the {label} has the check. BOTH nodes carry package D's "
            "refusal, so both carry its mirror - a safety check that exists in "
            "one node and not the other is the drift `domain_guard` exists to "
            "prevent",
        )
        if fn is None:
            continue
        check(
            any(
                isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "count_publishers"
                for node in ast.walk(fn)
            ),
            f"  ... and the {label} asks `count_publishers('/clock')` - the "
            "question the package never asked",
        )
        gate_calls = sorted({
            node.func.attr for node in ast.walk(fn)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and (node.func.attr in ("_handle_disarm", "_cancel_mission", "disarm")
                 or node.func.attr.startswith("note_"))
        })
        check(
            not gate_calls,
            f"  ... and the {label}'s check does NOT disarm and does NOT cancel. "
            "(The `fn is not None` guard above is what stops this passing "
            "vacuously on a build with no such function.)",
            f"gate calls: {gate_calls or 'none'}",
        )
        subs = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_subscription"
            and any(isinstance(arg, ast.Constant) and arg.value == "/clock"
                    for arg in node.args)
        ]
        check(
            len(subs) == 1,
            f"  ... and the {label} subscribes to /clock exactly once, which is "
            "the SECOND firing point: at startup DDS may not have matched a "
            "publisher that is already running",
            f"{len(subs)} /clock subscription(s)",
        )
        guarded = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.UnaryOp)
            and isinstance(node.test.op, ast.Not)
            and any(isinstance(sub, ast.Attribute) and sub.attr == "_use_sim_time"
                    for sub in ast.walk(node.test))
            and any(isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute)
                    and sub.func.attr == "create_subscription"
                    and any(isinstance(arg, ast.Constant) and arg.value == "/clock"
                            for arg in sub.args)
                    for sub in ast.walk(node))
        ]
        check(
            len(guarded) == 1,
            f"  ... and it is created ONLY under `not use_sim_time`: with sim "
            "time /clock is rclpy's own subscription and the situation is not "
            "the finding",
            f"{len(guarded)} guarded creation(s)",
        )

    # -- the latch, on the REAL method -------------------------------------
    class _Logger:
        def __init__(self):
            self.warns = []

        def warn(self, message):
            self.warns.append(message)

        error = info = fatal = warn

    def _fake(publishers, sim_time):
        fake = type("_FakeNode", (), {})()
        fake._logger = _Logger()
        fake.get_logger = lambda: fake._logger
        fake._use_sim_time = sim_time
        fake._clock_mismatch_warned = False
        fake._clock_publisher_seen = 0
        fake.count_publishers = lambda topic: publishers
        method = getattr(GoalGatewayNode, "_warn_if_clock_publisher", None)
        # A stand-in whose method does nothing, so the three checks below FAIL
        # rather than the part exploding, on a build where it does not exist.
        fake._warn_if_clock_publisher = (
            types.MethodType(method, fake) if method is not None
            else (lambda *_a, **_k: None)
        )
        return fake

    method_exists = getattr(GoalGatewayNode, "_warn_if_clock_publisher", None) is not None
    latched = _fake(1, False)
    first = latched._warn_if_clock_publisher("at startup")
    second = latched._warn_if_clock_publisher("on the first /clock message")
    check(
        first and not second and len(latched._logger.warns) == 1,
        "F-35: the REAL method warns ONCE across both firing points - the "
        "condition does not change while the node runs, and a mismatch repeated "
        "on every /clock message at 50 Hz is a mismatch nobody reads",
        f"{len(latched._logger.warns)} WARN(s) from two calls",
    )
    quiet = _fake(0, False)
    quiet._warn_if_clock_publisher("at startup")
    check(
        # `method_exists` again: "stays silent" is trivially true of a build with
        # no method at all, and the expected answer being "nothing happens" is
        # exactly where a vacuous pass hides best.
        method_exists and not quiet._logger.warns,
        "  ... and stays silent with no publisher, which is every scenario in the "
        "twin suite and the real robot",
    )
    class _Raising:
        def count_publishers(self, topic):
            raise RuntimeError("rmw says no")
    exploding = _fake(1, False)
    exploding.count_publishers = _Raising().count_publishers
    check(
        exploding._warn_if_clock_publisher("at startup") is False
        and not exploding._logger.warns,
        "  ... and a probe that RAISES costs the report and not the startup: this "
        "runs in __init__, and a node that fails to start because a diagnostic "
        "probe threw is a worse outcome than the mismatch it was looking for",
    )

    # -- the durable half, on /diagnostics ---------------------------------
    status = diag_mod.clock_status(
        proven=True, stalled=False, frozen_for_sec=0.0, use_sim_time=False,
        stall_sec=2.0, clock_publishers=2,
    )
    values = {kv.key: kv.value for kv in status.values}
    check(
        values.get("clock_publishers") == "2",
        "F-35: /diagnostics `external/clock` carries the publisher count beside "
        "`use_sim_time`, because the PAIR is the finding - a live publisher is "
        "unremarkable with sim time on and an epoch mismatch with it off. A WARN "
        "scrolls past; this does not",
    )
    check(
        status.level == diag_mod.OK,
        "  ... and it does not raise the level: report-only means the status that "
        "gates nothing must not start claiming a fault (the F-40 precedent)",
    )
    check(
        len(arm_mod.TRIGGER_CODES) == 10,
        "F-35 (NON-FALSIFYING invariant guard - also passes pre-change): still "
        "NINE auto-disarm triggers plus NONE. Detecting this mismatch added no "
        "way to disarm on it",
        f"{len(arm_mod.TRIGGER_CODES)} codes",
    )

    # -- UNDEFINED NAMES, package-wide -------------------------------------
    # Added because building F-35 shipped exactly this bug and NOTHING here
    # caught it: an edit batch aborted half-way, the link node kept its new
    # `/clock` subscription and lost its `ClockMsg` import, and the result
    # imports cleanly, parses cleanly and passes every structural check in this
    # file - then dies with `NameError` the moment the node is CONSTRUCTED.
    # Only the twin suite found it, and only because it starts real nodes.
    #
    # A name that is used and never bound is decidable without running
    # anything, so it should not need a twenty-minute suite to discover.
    try:
        from pyflakes.api import check as _pyflakes  # noqa: E402
        from pyflakes.reporter import Reporter  # noqa: E402
    except ImportError:
        print("  [WARN] pyflakes is not installed; the undefined-name guard did "
              "NOT run. This is the check that catches a NameError at node "
              "construction, so treat its absence as a gap, not as a pass.")
    else:
        import io as _io

        # BOTH the package AND this test directory. The first version scanned
        # only the package and promptly missed a `NameError` in THIS file - an
        # `ast.parse` in a part whose local `import ast` was in a different
        # part. The checks are code too, and a check that crashes is a check
        # that was not run, which a PASS/FAIL count does not show.
        roots = [
            os.path.join(_HERE, "..", "src", "gripperx_external"),
            _HERE,
        ]
        undefined = []
        for package in roots:
            for name in sorted(os.listdir(package)):
                if not name.endswith(".py"):
                    continue
                out, err = _io.StringIO(), _io.StringIO()
                with open(os.path.join(package, name), encoding="utf-8") as fh:
                    _pyflakes(fh.read(), name, Reporter(out, err))
                undefined += [
                    line for line in out.getvalue().splitlines()
                    if "undefined name" in line
                ]
        check(
            not undefined,
            "no UNDEFINED NAME anywhere in the package - the failure mode that "
            "imports, parses and passes every AST check in this file, and then "
            "raises NameError when the node is constructed",
            "; ".join(undefined[:3]) if undefined else "",
        )
    print()


def part11_epoch() -> None:
    """The fixture's sim-time EPOCH is a test dimension. SAFETY.md F-35.

    User decision 2026-08-21, taken on a finding from the first real-Gazebo
    campaign: `sim_clock.py` seeded sim time at `time.time()` while a real
    Gazebo starts it at 0, and that difference HID a mechanism. With the epochs
    ~1.787e9 s apart the TF lookup fails outright and the gateway refuses a goal
    it is armed for; with them close, that protection is absent by construction.
    The near-wall case is the BAG REPLAY that §6.4 item 8 names as the premise
    which stops being true - i.e. the case F-35 was argued from.

    **The wall seed is EXTENDED, not replaced.** It is what every scenario in
    the twin suite has always run against, and swapping the default would trade
    one blind spot for another.
    """
    import ast
    import importlib.util
    import inspect

    spec = importlib.util.spec_from_file_location(
        "sim_clock_fixture", os.path.join(_HERE, "sim_clock.py"))
    sim_clock = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim_clock)

    print("=" * 78)
    print("Part 11 - the fixture's sim-time epoch (F-35): both, not either")
    print("=" * 78)

    resolve = getattr(sim_clock, "resolve_epoch", None)
    check(
        resolve is not None,
        "F-35: WHERE SIM TIME STARTS is a named, pure decision rather than a "
        "conditional inside argument handling - the difference between the two "
        "seeds hid a mechanism for the whole life of this fixture",
    )
    exists = resolve is not None
    if not exists:
        resolve = lambda *_a, **_k: None  # noqa: E731

    NOW = 1787000000.0
    check(
        exists and resolve(None, "gazebo", NOW) == 0.0,
        "F-35: `gazebo` seeds sim time at 0.0, which is what a real Gazebo does "
        "- the case that produced TF_UNAVAILABLE and a refusal against the real "
        "thing on 2026-08-20",
    )
    check(
        exists and resolve(None, "wall", NOW) == NOW,
        "F-35: `wall` seeds it at wall time - a BAG REPLAY, the case F-35 was "
        "argued from, and the one with no evidence until 2026-08-21",
    )
    check(
        exists and resolve(None, "wall", NOW) != resolve(None, "gazebo", NOW),
        "  ... and the two are genuinely DIFFERENT seeds. (Stated as its own "
        "check because every assertion here is about a value being produced, "
        "and two modes that silently agreed would satisfy both of the above.)",
        f"wall={resolve(None, 'wall', NOW)} gazebo={resolve(None, 'gazebo', NOW)}",
    )
    check(
        exists and resolve(0.0, "wall", NOW) == 0.0,
        "F-35: an EXPLICIT 0.0 now means literally zero. It used to be the "
        "sentinel for 'use the wall clock', which is why `--epoch 0` could not "
        "previously express a Gazebo epoch at all - the sentinel was occupying "
        "the value",
    )
    check(
        exists and resolve(1234.5, "gazebo", NOW) == 1234.5,
        "  ... and an explicit seed beats the mode, so the band BETWEEN the two "
        "(a replay of an hour-old bag) can be expressed at all",
    )

    # The default must not have moved: every scenario in the twin suite has been
    # exercised against the wall seed, and this is the check that notices if a
    # later edit "simplifies" the default to `gazebo`.
    tree = ast.parse(inspect.getsource(sim_clock))
    mode_arg = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "add_argument"
        and any(isinstance(a, ast.Constant) and a.value == "--epoch-mode"
                for a in node.args)
    ]
    defaults = [
        kw.value.value for node in mode_arg for kw in node.keywords
        if kw.arg == "default" and isinstance(kw.value, ast.Constant)
    ]
    choices = [
        elt.value for node in mode_arg for kw in node.keywords
        if kw.arg == "choices" for elt in getattr(kw.value, "elts", [])
        if isinstance(elt, ast.Constant)
    ]
    check(
        defaults == ["wall"],
        "F-35: the DEFAULT is still `wall`. The wall seed is what every scenario "
        "in the twin suite has always run against; changing the default would "
        "trade one blind spot for another rather than removing one",
        f"default={defaults}",
    )
    check(
        sorted(choices) == ["gazebo", "wall"],
        "  ... and both epochs are offered, which is the whole decision: extend, "
        "do not replace",
        f"choices={sorted(choices)}",
    )
    print()


def main() -> int:
    part1_protocol()
    part2_pipeline()
    part3_arming()
    part4_correlation()
    part5_clock_and_staleness()
    part6_package_e()
    part7_f37_f38()
    part8_f40()
    part9_f36()
    part10_f35()
    part11_epoch()
    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All protocol, validation and arming checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
