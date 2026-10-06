#!/usr/bin/env python3
"""The shared Octopus reference line: the frame, the gate, the invalidation.

    python3 test/check_line_calibration.py

WHAT WOULD HAVE TO CHANGE FOR THESE TO FAIL
===========================================
Every expected value below is written from the CONVENTION, never read back
from `line_frame`:

* handedness is judged with a cross product - "is this point LEFT of A->B?" -
  which is the physical definition in the interface document and does not care
  how the code computes it. Mirroring +y, swapping A and B, or rotating by the
  wrong sign fails it;
* the Octopus side of the chain is their forward flat-earth formula, written out
  here from their source (`trash_gps_goal_node.py`), not our inverse;
* the numbers in the worked examples are computed by hand in the comments.

Parts 3 and 4 build the real nodes (a ROS context, nothing spun, nothing
dispatched, nothing moved) and drive their handlers directly, the way
`check_frame_gate.py` does.
"""

import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from gripperx_external import line_frame as lf  # noqa: E402
from gripperx_external import validation as val  # noqa: E402
from gripperx_external.geodesy import Datum, DatumTracker  # noqa: E402
from gripperx_external.grasp import GraspOffset  # noqa: E402

FAILURES = []

#: The configs whose calibration-node parameters are checked. The expected-length
#: range is READ from them (never copied here): changing the range in a config
#: moves every test point of part 8 with it, and deleting it fails part 8.
CONFIG_DIR = os.path.join(_HERE, "..", "config")
CONFIGS = ("octopus_link_real.yaml", "octopus_link_twin.yaml")
CAL_NODE_KEY = "/gripperx/external/line_calibration_node"
#: The section BOTH the calibration node and the gateway read (audit H1).
SHARED_KEY = "/**"

#: TEST FIXTURE, not the configured range: a range wide enough for the
#: hand-worked geometries of parts 3 and 5 (L = 4.0 and 4.2 m).
FIXTURE_RANGE = (1.0, 10.0)


def config_params(name):
    """The SHARED `/**` section of a config - what both nodes read."""
    import yaml
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)[SHARED_KEY]["ros__parameters"]


def shared_overrides(name):
    from rclpy.parameter import Parameter
    return [Parameter(k, value=v) for k, v in config_params(name).items()]


def config_range(name):
    p = config_params(name)
    return float(p["expected_length_min_m"]), float(p["expected_length_max_m"])

#: THEIR constant and THEIR forward formula (see geodesy's docstring). Written
#: out here so that the chain is checked against the counterpart, not against
#: our own inverse.
THEIR_M_PER_DEG = 111320.0
DATUM = Datum(48.2650, 11.6710)


def their_latlon(x_line, y_line, datum=DATUM):
    lat = datum.latitude_deg + y_line / THEIR_M_PER_DEG
    lon = datum.longitude_deg + x_line / (
        THEIR_M_PER_DEG * math.cos(math.radians(datum.latitude_deg))
    )
    return lat, lon


def check(label, condition, detail=""):
    print(f"  [{'ok ' if condition else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""),
          flush=True)
    if not condition:
        FAILURES.append(label)


def close(a, b, tol=1e-9):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def left_of(a, b, p):
    """> 0 when p is to the LEFT of someone at a looking at b (seen from above)."""
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def cal(a, b, **kw):
    kw.setdefault("session", "check")
    kw.setdefault("calibration_id", 1)
    kw.setdefault("map_session", ("gid-A",))
    return lf.compute_calibration(a, b, **kw)


# ===========================================================================
def part1_frame():
    print("\n-- 1. the frame convention --------------------------------------")
    rng = random.Random(20260924)
    cases = [((0.0, 0.0), (2.0, 0.0)), ((0.0, 0.0), (0.0, 2.0)), ((3.0, 1.0), (3.0, 5.0)),
             ((1.0, 1.0), (-2.0, -3.0))]
    cases += [((rng.uniform(-9, 9), rng.uniform(-9, 9)), (rng.uniform(-9, 9), rng.uniform(-9, 9)))
              for _ in range(50)]
    bad = {"origin": 0, "toward_b": 0, "left": 0, "roundtrip": 0, "distance": 0}
    for a, b in cases:
        c = cal(a, b)
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if not close(c.line_to_map(0.0, 0.0), mid):
            bad["origin"] += 1
        # +x from A to B, unit scale: the posts sit at x = -L/2 and +L/2.
        if not (close(c.line_to_map(length / 2, 0.0), b, 1e-9)
                and close(c.line_to_map(-length / 2, 0.0), a, 1e-9)):
            bad["toward_b"] += 1
        # +y is LEFT of A->B, and at unit distance from the line.
        p = c.line_to_map(0.0, 1.0)
        if not (left_of(a, b, p) > 0 and abs(abs(left_of(a, b, p)) / length - 1.0) < 1e-9):
            bad["left"] += 1
        for _ in range(5):
            q = (rng.uniform(-20, 20), rng.uniform(-20, 20))
            if not close(c.map_to_line(*c.line_to_map(*q)), q, 1e-9):
                bad["roundtrip"] += 1
            if not close(c.line_to_map(*c.map_to_line(*q)), q, 1e-9):
                bad["roundtrip"] += 1
            r = (rng.uniform(-20, 20), rng.uniform(-20, 20))
            d_line = math.dist(q, r)
            d_map = math.dist(c.line_to_map(*q), c.line_to_map(*r))
            if abs(d_line - d_map) > 1e-9:
                bad["distance"] += 1
    n = len(cases)
    check(f"origin = midpoint of A and B ({n} pairs)", bad["origin"] == 0, str(bad["origin"]))
    check("+x points from A to B, and x = -L/2 is A, x = +L/2 is B (no scaling)",
          bad["toward_b"] == 0, str(bad["toward_b"]))
    check("+y is to the LEFT of A->B (cross product) - FAILS if +y were mirrored",
          bad["left"] == 0, str(bad["left"]))
    check("map -> line -> map and line -> map -> line are identities", bad["roundtrip"] == 0)
    check("distances are preserved (rigid, no scale)", bad["distance"] == 0)

    # Worked by hand. A=(0,0) B=(0,2): +x is map +y, so +y (left) is map -x.
    c = cal((0.0, 0.0), (0.0, 2.0))
    check("A(0,0) B(0,2): line (0,1) is map (-1,1)", close(c.line_to_map(0.0, 1.0), (-1.0, 1.0)))
    check("  ... yaw = +90 deg (counter-clockwise positive)",
          abs(math.degrees(c.yaw_rad) - 90.0) < 1e-12)
    c = cal((0.0, 0.0), (1.0, 1.0))
    check("A(0,0) B(1,1): yaw = +45 deg, L = 1.414 m",
          abs(math.degrees(c.yaw_rad) - 45.0) < 1e-12 and abs(c.length_m - math.sqrt(2)) < 1e-12)
    c = cal((1.0, 1.0), (1.0, -1.0))
    check("A(1,1) B(1,-1): yaw = -90 deg, line (0,1) is map (2,0)",
          abs(math.degrees(c.yaw_rad) + 90.0) < 1e-12 and close(c.line_to_map(0.0, 1.0), (2.0, 0.0)))

    # Swapping A and B is a 180-degree turn, not a mirror: every line
    # coordinate changes sign. This is why both sides must agree on A.
    fwd, swapped = cal((3.0, 1.0), (3.0, 5.0)), cal((3.0, 5.0), (3.0, 1.0))
    p = (4.2, 2.7)
    check("swapping A and B negates BOTH line coordinates (x -> -x, y -> -y)",
          close(swapped.map_to_line(*p), tuple(-v for v in fwd.map_to_line(*p))))

    check("yaw map->line->map round trip",
          abs(fwd.yaw_line_to_map(fwd.yaw_map_to_line(2.9)) - 2.9) < 1e-12)
    check("a robot heading along A->B has line yaw 0",
          abs(fwd.yaw_map_to_line(fwd.yaw_rad)) < 1e-12)


def part2_degenerate():
    print("\n-- 2. degenerate input is refused --------------------------------")

    def reason(a, b, rng=None):
        try:
            lf.compute_calibration(a, b, rng)
        except lf.LineCalibrationError as exc:
            return exc.reason
        return ""

    check("the same point twice -> DEGENERATE", reason((1.0, 1.0), (1.0, 1.0)) == lf.DEGENERATE)
    check("0.5 mm apart -> DEGENERATE (below the numeric floor)",
          reason((1.0, 1.0), (1.0005, 1.0)) == lf.DEGENERATE)
    check("2 mm apart with NO range given -> accepted (the floor is arithmetic only)",
          reason((1.0, 1.0), (1.002, 1.0)) == "")
    check("the same point twice is DEGENERATE even with a range given",
          reason((1.0, 1.0), (1.0, 1.0), FIXTURE_RANGE) == lf.DEGENERATE)
    check("an unusable range (min > max) refuses every pair (NO_LENGTH_RANGE)",
          reason((0.0, 0.0), (2.0, 0.0), (3.0, 1.0)) == lf.NO_LENGTH_RANGE)
    check("a NaN range refuses every pair (NO_LENGTH_RANGE)",
          reason((0.0, 0.0), (2.0, 0.0), (float("nan"), float("nan"))) == lf.NO_LENGTH_RANGE)
    check("a NaN coordinate -> NOT_FINITE", reason((float("nan"), 0.0), (1.0, 0.0)) == lf.NOT_FINITE)
    check("an infinite coordinate -> NOT_FINITE", reason((0.0, 0.0), (float("inf"), 0.0)) == lf.NOT_FINITE)


def parse(text):
    """parse_status with the FIXTURE expectations of part 3 (not the config)."""
    return lf.parse_status(text, length_range_m=FIXTURE_RANGE, parent_frame_id="map",
                           frame_id="octopus_line")


def part3_chain_and_status():
    print("\n-- 3. the boundary chain and the status JSON ---------------------")
    # A=(3,1) B=(3,5): origin (3,3), +x = map +y, +y = map -x.
    c = cal((3.0, 1.0), (3.0, 5.0))
    lat, lon = their_latlon(1.0, 0.0)
    check("an Octopus goal at line (1,0) lands at map (3,4)",
          close(lf.latlon_to_our_map(DATUM, c, lat, lon), (3.0, 4.0), 1e-6))
    lat, lon = their_latlon(0.0, 1.0)
    check("an Octopus goal at line (0,1) lands at map (2,3)",
          close(lf.latlon_to_our_map(DATUM, c, lat, lon), (2.0, 3.0), 1e-6))
    back = lf.our_map_to_latlon(DATUM, c, 2.0, 3.0)
    check("our map (2,3) goes back to exactly their lat/lon of line (0,1)",
          close(back, their_latlon(0.0, 1.0), 1e-12))

    text = lf.build_status(state=lf.STATE_CALIBRATED, session="s1", frame_id="octopus_line",
                           parent_frame_id="map", stamp_sec=1.0, calibration=c,
                           calibration_count=1, map_topic="/map",
                           expected_length_m=FIXTURE_RANGE)
    parsed = parse(text)
    # The status is published by session "s1" while `c` was built by session
    # "check": the identity a consumer sees is the PUBLISHER's session.
    check("status round trip: calibrated, publisher's identity, same transform",
          parsed.calibration is not None
          and parsed.calibration.identity == ("s1", 1)
          and close(parsed.calibration.line_to_map(1.0, 0.0), (3.0, 4.0)))
    data = json.loads(text)
    check("the status carries L, yaw in rad AND deg, A, B, origin, id and stamp",
          all(k in data for k in ("length_m", "yaw_rad", "yaw_deg", "a", "b", "origin",
                                  "calibration_id", "stamp"))
          and abs(data["length_m"] - 4.0) < 1e-12 and abs(data["yaw_deg"] - 90.0) < 1e-12)
    tampered = dict(data, yaw_rad=data["yaw_rad"] + 0.01)
    check("a status whose yaw disagrees with its own A/B is REFUSED (STATUS_INCONSISTENT)",
          parse(json.dumps(tampered)).reason == lf.STATUS_INCONSISTENT)
    check("the CONSUMER's range decides: L = 4.0 against a consumer range of 2.2-2.8 is "
          "refused, whatever the status declares for itself (audit H1)",
          lf.parse_status(json.dumps(dict(data, expected_length_min_m=1.0,
                                          expected_length_max_m=10.0)),
                          length_range_m=(2.2, 2.8), parent_frame_id="map",
                          frame_id="octopus_line").reason == lf.LENGTH_OUT_OF_RANGE)
    check("a status whose parent frame is not the consumer's map is refused (WRONG_FRAME)",
          parse(json.dumps(dict(data, parent_frame_id="odom"))).reason == lf.WRONG_FRAME)
    check("a status for another child frame is refused (WRONG_FRAME)",
          parse(json.dumps(dict(data, frame_id="other_line"))).reason == lf.WRONG_FRAME)
    no_session = dict(data, map_session=[])
    check("a calibrated status without a map session is refused",
          parse(json.dumps(no_session)).calibration is None)
    check("garbage is STATUS_UNPARSEABLE, not an exception",
          parse("{nope").reason == lf.STATUS_UNPARSEABLE)
    waiting = lf.build_status(state=lf.STATE_WAITING_FOR_B, session="s1", frame_id="octopus_line",
                              parent_frame_id="map", stamp_sec=1.0, reason=lf.RECALIBRATING,
                              pending_a=(1.0, 2.0))
    check("an uncalibrated status yields no calibration and keeps its reason",
          parse(waiting).calibration is None
          and parse(waiting).reason == lf.RECALIBRATING)

    key = lf.map_session_key([[1, 2, 255], [0, 9]])
    check("map session key: raw GIDs normalised to sorted hex",
          key == ("0009", "0102ff"))
    check("same publishers -> no verdict", lf.map_session_verdict(key, tuple(reversed(key))) == "")
    check("no publisher visible -> NO_MAP_SESSION (a block, not proof)",
          lf.map_session_verdict(key, ()) == lf.NO_MAP_SESSION)
    check("a different publisher -> MAP_SESSION_CHANGED",
          lf.map_session_verdict(key, ("beef",)) == lf.MAP_SESSION_CHANGED)
    check("an extra publisher -> MAP_SESSION_CHANGED (equality, not subset)",
          lf.map_session_verdict(key, key + ("beef",)) == lf.MAP_SESSION_CHANGED)


def part4_validation():
    print("\n-- 4. the validation pipeline gates on it ------------------------")
    tracker = DatumTracker(fallback=None, jump_warn_m=0.25)
    tracker.update(DATUM)
    lat, lon = their_latlon(1.0, 0.0)
    base = dict(
        goal=val.IncomingGoal("7", lat, lon, stamp_sec=100.0),
        now_sec=100.1,
        datum_tracker=tracker,
        grasp_offset=GraspOffset(x=0.36, y=0.0, tolerance_m=0.05),
        robot_pose=(3.0, 3.0, 0.0),
        robot_pose_age_sec=0.05,
        geofence=lambda x, y: True,
        costmap_cost=lambda x, y: 0,
    )
    r = val.validate_goal(val.ValidationContext(**base))
    check("NO calibration -> REJECTED with NO_LINE_CALIBRATION (fail closed by default)",
          r.verdict == val.VERDICT_REJECTED and r.reason == val.NO_LINE_CALIBRATION, r.reason)
    check("  ... at LOCAL severity (ERROR log + ERROR diagnostic)",
          r.severity == val.SEVERITY_LOCAL)
    c = cal((3.0, 1.0), (3.0, 5.0))
    r = val.validate_goal(val.ValidationContext(line_calibration=c, **base))
    check("with the calibration, the object is at the TRANSFORMED position (3,4)",
          r.object_xy is not None and close(r.object_xy, (3.0, 4.0), 1e-6), f"{r.object_xy}")

    def dispatch(**over):
        d = dict(armed=True, dry_run=False, link_alive=True, teleop_mode="autonomous",
                 teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
                 line_calibration_unchanged=True, pose=(3.0, 3.6, 0.0))
        d.update(over)
        return val.validate_dispatch(val.DispatchContext(**d),
                                     val.ValidationContext(line_calibration=c, **base))

    check("dispatch gate passes with an unchanged calibration", dispatch().accepted)
    check("a replaced/lost calibration refuses the dispatch (LINE_CALIBRATION_CHANGED)",
          dispatch(line_calibration_unchanged=False).reason == val.LINE_CALIBRATION_CHANGED)
    try:
        val.DispatchContext(armed=True, dry_run=False, link_alive=True, teleop_mode="autonomous",
                            teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
                            pose=(0.0, 0.0, 0.0))
        required = False
    except TypeError:
        required = True
    check("DispatchContext has NO default for line_calibration_unchanged (no silent pass)",
          required)


# ===========================================================================
class _Info:
    def __init__(self, gid):
        self.endpoint_gid = gid


def _line(length, cid, session_gids, x=3.0, cy=3.0):
    """A vertical line at x, midpoint (x, cy), A below B: yaw +90 deg, so line
    (u, v) is map (x - v, cy + u)."""
    return cal((x, cy - length / 2), (x, cy + length / 2), session="n1",
               calibration_id=cid, map_session=lf.map_session_key(session_gids))


def part5_gateway(rclpy):
    print("\n-- 5. the gateway: fail closed, cancel, invalidate ---------------")
    from std_msgs.msg import String
    from gripperx_external.goal_gateway_node import GoalGatewayNode, Mission

    # Built with the SHARED section of the twin config - the gateway reads the
    # very keys the calibration node reads (audit H1).
    node = GoalGatewayNode(parameter_overrides=shared_overrides("octopus_link_twin.yaml"))
    low, high = config_range("octopus_link_twin.yaml")
    L1, L2 = 0.5 * (low + high), 0.25 * low + 0.75 * high  # both inside the range
    visible = {"gids": [[1, 1]]}
    node.get_publishers_info_by_topic = lambda topic: [_Info(g) for g in visible["gids"]]

    def status(c, state=lf.STATE_CALIBRATED, reason="", **over):
        data = json.loads(lf.build_status(
            state=state, session="n1", frame_id="octopus_line", parent_frame_id="map",
            stamp_sec=1.0, calibration=c if state == lf.STATE_CALIBRATED else None,
            reason=reason, expected_length_m=(low, high)))
        data.update(over)
        return String(data=json.dumps(data))

    def blocks():
        return [b for b in node._dispatch_blocks() if "line calibration" in b]

    node._check_map_session()
    check("fresh gateway: no calibration, and a dispatch block says so",
          node._live_line_calibration() is None and len(blocks()) == 1, str(blocks()))
    check("  ... and the context it builds refuses goals",
          node._make_context(val.IncomingGoal("1", 48.0, 11.0)).line_calibration is None)

    print("\n   H1 - the gateway applies ITS range and frames, not the status's")
    wide = _line(4.0, 90, [[1, 1]])
    node._on_line_calibration(status(wide, expected_length_min_m=1.0, expected_length_max_m=10.0))
    check("a status with L = 4.000 m declaring its own range 1-10 m is REFUSED",
          node._live_line_calibration() is None and node._line_cal_reason == lf.LENGTH_OUT_OF_RANGE,
          node._line_cal_reason)
    ok = _line(L1, 91, [[1, 1]])
    node._on_line_calibration(status(ok, parent_frame_id="odom"))
    check("a status with parent_frame_id 'odom' is REFUSED (WRONG_FRAME)",
          node._live_line_calibration() is None and node._line_cal_reason == lf.WRONG_FRAME,
          node._line_cal_reason)
    node._on_line_calibration(status(ok, frame_id="some_other_line"))
    check("a status for another child frame is REFUSED (WRONG_FRAME)",
          node._live_line_calibration() is None and node._line_cal_reason == lf.WRONG_FRAME)

    c1 = _line(L1, 1, [[1, 1]])
    node._on_line_calibration(status(c1))
    check("a calibrated status inside the configured range, on the live map session -> "
          "live, block gone", node._live_line_calibration() is not None and not blocks())

    cancels = []
    node._cancel_mission = lambda reason, now, error=True: cancels.append(reason) or True
    node._mission = Mission(target_id="5", object_xy=(3.0, 4.0), pose=(3.0, 3.6, 0.0),
                            datum_lat=DATUM.latitude_deg, datum_lon=DATUM.longitude_deg,
                            started_at_sec=0.0, line_calibration=c1)
    check("the mission's object (map (3,4)) goes back to THEIR lat/lon of line (1,0)",
          close(node._mission_object_latlon(node._mission), their_latlon(1.0, 0.0), 1e-12))
    check("unchanged calibration -> the in-flight check holds",
          node._line_calibration_unchanged_since(c1))

    c2 = _line(L2, 2, [[1, 1]])
    node._on_line_calibration(status(c2))
    check("RECALIBRATION with a mission in flight CANCELS it",
          len(cancels) == 1 and cancels[0].startswith("LINE_RECALIBRATED"), str(cancels))
    check("  ... and the mission's calibration no longer counts as unchanged",
          not node._line_calibration_unchanged_since(c1))
    node._on_line_calibration(status(c1))
    check("a re-delivered latched copy of the replaced calibration is IGNORED",
          node._live_line_calibration().calibration_id == 2)

    node._on_line_calibration(status(None, lf.STATE_WAITING_FOR_B, lf.RECALIBRATING))
    check("the calibration node starting a new pair -> calibration GONE and the mission cancelled",
          node._live_line_calibration() is None and cancels[-1].startswith("LINE_CALIBRATION_LOST"))

    print("\n   M2 - an invisible map publisher, grace TO-VERIFY (the config)")
    check("the configured grace is TO-VERIFY, so the fail-closed reading is in force",
          node._map_grace_sec is None)
    c3 = _line(L1, 3, [[1, 1]])
    node._on_line_calibration(status(c3))
    node._mission.line_calibration = c3
    visible["gids"] = []
    node._check_map_session()
    check("empty read -> NO new dispatch (live None), calibration NOT buried",
          node._live_line_calibration() is None and node._line_cal is not None)
    check("  ... and, the grace being unmeasured, the goal in flight no longer holds (cancel)",
          not node._line_calibration_unchanged_since(c3))
    visible["gids"] = [[1, 1]]
    node._check_map_session()
    check("  ... and live again once the SAME publisher is seen",
          node._live_line_calibration() is not None and node._line_calibration_unchanged_since(c3))

    print("\n   M2 - the same, with a MEASURED grace (fixture 5 s, not a value)")
    node._map_grace_sec = 5.0
    visible["gids"] = []
    node._check_map_session()
    check("empty read inside the grace -> no new dispatch, but the goal in flight HOLDS",
          node._live_line_calibration() is None and node._line_calibration_unchanged_since(c3))
    n_before = len(cancels)
    node._map_empty_since -= 6.0
    node._check_map_session()
    check("empty for longer than the grace -> buried as MAP_SESSION_LOST and cancelled",
          node._line_cal is None and len(cancels) == n_before + 1
          and cancels[-1].startswith(lf.MAP_SESSION_LOST), str(cancels[-1:]))
    node._map_grace_sec = None

    c4 = _line(L1, 4, [[1, 1]])
    visible["gids"] = [[1, 1]]
    node._check_map_session()
    node._on_line_calibration(status(c4))
    n_before = len(cancels)
    node._mission.line_calibration = c4
    visible["gids"] = [[2, 2]]
    node._check_map_session()
    check("a DIFFERENT map publisher (slam_toolbox restarted) -> invalidated and cancelled",
          node._live_line_calibration() is None and node._line_cal is None
          and len(cancels) == n_before + 1 and cancels[-1].startswith(lf.MAP_SESSION_CHANGED))
    visible["gids"] = [[1, 1]]
    node._check_map_session()
    node._on_line_calibration(status(c4))
    check("  ... and it stays dead even if its status is re-delivered",
          node._live_line_calibration() is None)
    check("the reason is visible in the dispatch block",
          blocks() and lf.MAP_SESSION_CHANGED in blocks()[0], str(blocks()))

    print("\n-- 6. telemetry: map_x/map_y stay MAP, the line fields go out ---")
    node._mission = None
    visible["gids"] = [[3, 3]]
    c5 = _line(L1, 5, [[3, 3]])
    node._check_map_session()
    node._on_line_calibration(status(c5))
    node._datum_tracker.update(DATUM)
    sent = []
    node._telemetry_pub.publish = sent.append
    # Robot at map (3,4) facing map +y: that is line (1,0) facing line +x.
    node._robot_pose = lambda: ((3.0, 4.0, math.pi / 2), 0.05, "")
    node._telemetry_tick()
    t = sent[-1]
    check("map_x/map_y/yaw_deg are OUR MAP pose (3, 4, 90 deg), frame map (audit L5)",
          t.pose_valid and close((t.map_x, t.map_y), (3.0, 4.0)) and abs(t.yaw_deg - 90.0) < 1e-9
          and t.header.frame_id == "map")
    check("line_x/line_y/line_yaw_deg are line (1, 0), 0 deg",
          t.line_valid and close((t.line_x, t.line_y), (1.0, 0.0), 1e-9)
          and abs(t.line_yaw_deg) < 1e-9)
    check("L and the calibration id travel with it (U2)",
          t.line_calibration_id == 5 and abs(t.line_length_m - L1) < 1e-12)
    check("  ... and lat/lon is THEIR lat/lon of line (1,0)",
          t.latlon_valid and close((t.latitude_deg, t.longitude_deg), their_latlon(1.0, 0.0), 1e-12))

    # L4: the outward reason is a code, never a coordinate.
    node._mission = Mission(target_id="5", object_xy=(3.0, 4.0), pose=(3.0, 3.6, 0.0),
                            datum_lat=DATUM.latitude_deg, datum_lon=DATUM.longitude_deg,
                            started_at_sec=0.0, line_calibration=c5,
                            cancel_reason="OUTSIDE_GEOFENCE: pose (3.123, 4.567)")
    node._nav2_available = True  # the reason is only reported with a server
    node._telemetry_tick()
    check("an outward nav_state_reason carries the CODE only, no map coordinates (audit L4)",
          sent[-1].nav_state_reason == "OUTSIDE_GEOFENCE", sent[-1].nav_state_reason)
    node._mission = None

    node._on_line_calibration(status(None, lf.STATE_WAITING_FOR_A, lf.RESET))
    node._telemetry_tick()
    t = sent[-1]
    check("no calibration -> line block UNAVAILABLE (NO_LINE_CALIBRATION), id -1, L NaN, "
          "no lat/lon; the map pose is still reported locally",
          not t.line_valid and t.line_reason == val.NO_LINE_CALIBRATION
          and t.line_calibration_id == -1 and math.isnan(t.line_length_m)
          and not t.latlon_valid and t.pose_valid)

    # What the link node puts on the wire from that message.
    from gripperx_external.octopus_link_node import OctopusLinkNode

    class _Link:
        _telemetry_json_pub = None
        _publish_telemetry = True

        def __init__(self):
            self.sent = []
            self._client = self

        def publish(self, topic, payload):
            self.sent.append(json.loads(payload["data"]))

        def _transform_telemetry_block(self):
            return None

    node._on_line_calibration(status(c5.__class__(**{**c5.__dict__, "calibration_id": 6})))
    node._telemetry_tick()
    link = _Link()
    OctopusLinkNode._on_telemetry(link, sent[-1])
    wire = link.sent[-1]
    check("on the wire: pose.x/y/yaw_deg are the LINE pose, not map_x/map_y",
          close((wire["pose"]["x"], wire["pose"]["y"]), (1.0, 0.0), 1e-9)
          and abs(wire["pose"]["yaw_deg"]) < 1e-9, json.dumps(wire["pose"])[:120])
    check("on the wire: line_calibration carries the id and L (U2)",
          wire["line_calibration"]["id"] == 6
          and abs(wire["line_calibration"]["length_m"] - L1) < 1e-12)
    node.destroy_node()


def _cal_node(config, visible):
    """The real node, built with the parameters of `config` (None = none)."""
    from gripperx_external.line_calibration_node import LineCalibrationNode

    overrides = [] if config is None else shared_overrides(config)
    node = LineCalibrationNode(parameter_overrides=overrides)
    node.get_publishers_info_by_topic = lambda topic: [_Info(g) for g in visible["gids"]]
    rec = {"status": [], "markers": [], "tf": []}
    node._status_pub.publish = lambda m: rec["status"].append(json.loads(m.data))
    node._marker_pub.publish = rec["markers"].append
    node._tf_pub.publish = rec["tf"].append

    def click(x, y, frame="map", z=0.21):
        from geometry_msgs.msg import PointStamped
        p = PointStamped()
        p.header.frame_id = frame
        p.point.x, p.point.y, p.point.z = x, y, z
        node._on_clicked_point(p)

    return node, rec, click


def part7_calibration_node(rclpy):
    print("\n-- 7. the calibration node --------------------------------------")
    from std_srvs.srv import Trigger

    visible = {"gids": [[7, 7]]}
    node, rec, click = _cal_node("octopus_link_twin.yaml", visible)
    statuses, markers, tfs = rec["status"], rec["markers"], rec["tf"]
    low, high = config_range("octopus_link_twin.yaml")
    L = 0.5 * (low + high)  # inside the configured range, whatever it is

    click(1.0, 1.0, frame="base_link")
    check("a click in the wrong frame is ignored", statuses[-1]["state"] == lf.STATE_WAITING_FOR_A)
    click(1.0, 1.0)
    check("first click -> waiting for B, pending A reported",
          statuses[-1]["state"] == lf.STATE_WAITING_FOR_B
          and statuses[-1]["pending_a"] == {"x": 1.0, "y": 1.0})
    click(1.0, 1.0 + L)
    s = statuses[-1]
    check(f"second click -> calibrated #1, L = {L:.3f} m, yaw +90 deg, origin (1, 1 + L/2)",
          s["state"] == lf.STATE_CALIBRATED and s["calibration_id"] == 1
          and abs(s["length_m"] - L) < 1e-12 and abs(s["yaw_deg"] - 90.0) < 1e-12
          and close((s["origin"]["x"], s["origin"]["y"]), (1.0, 1.0 + L / 2)), json.dumps(s)[:160])
    check("  ... the clicked z (0.21) is ignored - origin z is 0 in the TF",
          (node._publish_tf() or True) and tfs and tfs[-1].transforms[0].transform.translation.z == 0.0)
    tf = tfs[-1].transforms[0]
    check("TF map -> octopus_line at the origin with yaw +90 deg",
          tf.header.frame_id == "map" and tf.child_frame_id == "octopus_line"
          and close((tf.transform.translation.x, tf.transform.translation.y), (1.0, 1.0 + L / 2))
          and abs(2 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w) - math.pi / 2) < 1e-12)
    check("the gateway's parser accepts the node's own status",
          lf.parse_status(json.dumps(s), length_range_m=(low, high), parent_frame_id="map",
                          frame_id="octopus_line").calibration is not None)
    texts = [m.text for m in markers[-1].markers if m.ns == "line_cal/length"]
    check("a text marker shows L to the millimetre", texts == [f"L = {L:.3f} m  (#1)"], str(texts))
    labels = sorted(m.text for m in markers[-1].markers if m.ns == "line_cal/post_labels")
    check("both posts are labelled A and B", labels == ["A", "B"], str(labels))
    y_arrow = [m for m in markers[-1].markers if m.ns == "line_cal/axes" and m.id == 1][0]
    tip = (y_arrow.points[1].x, y_arrow.points[1].y)
    check("the drawn +y arrow points LEFT of A->B",
          left_of((1.0, 1.0), (1.0, 1.0 + L), tip) > 0, str(tip))

    click(2.0, 2.0)
    check("a third click starts a new pair and DROPS the live calibration (RECALIBRATING)",
          statuses[-1]["state"] == lf.STATE_WAITING_FOR_B and statuses[-1]["reason"] == lf.RECALIBRATING
          and statuses[-1]["calibration_id"] is None)
    n_tf = len(tfs)
    node._publish_tf()
    check("  ... and the TF stops (a stale line frame times out instead of lingering)",
          len(tfs) == n_tf)
    click(2.0, 2.0)
    check("the same post clicked twice -> DEGENERATE, not a calibration",
          statuses[-1]["reason"] == lf.DEGENERATE and statuses[-1]["calibration_id"] is None)
    click(0.0, 0.0)
    click(L, 0.0)
    check("a new pair -> calibration #2 (the id increments)", statuses[-1]["calibration_id"] == 2)

    visible["gids"] = [[8, 8]]
    node._check_map_session()
    check("a new map publisher -> MAP_SESSION_CHANGED, calibration dropped",
          statuses[-1]["reason"] == lf.MAP_SESSION_CHANGED and statuses[-1]["calibration_id"] is None)
    click(0.0, 0.0)
    click(L, 0.0)
    visible["gids"] = []
    node._check_map_session()
    s = rec["status"][-1]
    check("an EMPTY map read, grace TO-VERIFY (the config) -> dropped at once as "
          "MAP_SESSION_LOST, and the text says 'no publisher', not 'changed' (audit M2)",
          s["reason"] == lf.MAP_SESSION_LOST and s["calibration_id"] is None
          and "no publisher visible" in s["detail"] and "DIFFERENT" not in s["detail"],
          s["detail"][:100])
    visible["gids"] = [[8, 8]]
    node._map_grace_sec = 5.0  # a FIXTURE for the measured case, not a value
    click(0.0, 0.0)
    click(L, 0.0)
    visible["gids"] = []
    node._check_map_session()
    check("with a MEASURED grace, one empty read does NOT drop the calibration",
          rec["status"][-1]["state"] == lf.STATE_CALIBRATED)
    node._map_empty_since -= 6.0
    node._check_map_session()
    check("  ... an absence longer than the grace does (MAP_SESSION_LOST)",
          rec["status"][-1]["reason"] == lf.MAP_SESSION_LOST)
    node._map_grace_sec = None
    click(0.0, 0.0)
    click(L, 0.0)
    check("no map publisher at all -> refused with NO_MAP_SESSION",
          statuses[-1]["reason"] == lf.NO_MAP_SESSION and statuses[-1]["calibration_id"] is None)
    visible["gids"] = [[8, 8]]
    click(0.0, 0.0)
    click(L, 0.0)
    resp = node._on_reset(Trigger.Request(), Trigger.Response())
    check("reset discards it", resp.success and statuses[-1]["reason"] == lf.RESET
          and statuses[-1]["calibration_id"] is None, resp.message)
    check("the configured range is reported in the status",
          (statuses[-1]["expected_length_min_m"], statuses[-1]["expected_length_max_m"])
          == (low, high))
    check("the node has no clients and no forbidden publishers (sweeps ran at startup)",
          not list(node.clients))
    node.destroy_node()


def part8_length_range(rclpy):
    print("\n-- 8. the expected-length range, READ FROM THE CONFIGS -----------")
    for config in CONFIGS:
        try:
            low, high = config_range(config)
        except (KeyError, TypeError, ValueError) as exc:
            check(f"{config}: expected_length_min_m / max_m are configured", False, repr(exc))
            continue
        check(f"{config}: a usable range is configured ({low:.3f}-{high:.3f} m)",
              math.isfinite(low) and math.isfinite(high) and 0.0 < low <= high)
        visible = {"gids": [[5, 5]]}
        node, rec, click = _cal_node(config, visible)
        # 10 mm outside each end, and the middle - derived from the config,
        # so the check follows the file and fails if the gate is not applied.
        for length, want in ((low - 0.01, False), (high + 0.01, False), (0.5 * (low + high), True)):
            click(0.0, 0.0)
            click(0.0, length)
            s = rec["status"][-1]
            n_tf = len(rec["tf"])
            node._publish_tf()
            if want:
                check(f"{config}: L = {length:.3f} m (inside) -> calibrated, TF published",
                      s["state"] == lf.STATE_CALIBRATED and len(rec["tf"]) == n_tf + 1)
            else:
                rejected = [m.text for m in rec["markers"][-1].markers
                            if m.ns == "line_cal/rejected" and m.type == m.TEXT_VIEW_FACING]
                check(f"{config}:   ... the RViz text IS the log wording (audit L6)",
                      rejected == [lf.rejection_text(lf.LENGTH_OUT_OF_RANGE, length, (low, high))],
                      str(rejected))
                check(f"{config}: L = {length:.3f} m (outside) -> REJECTED: no 'calibrated', "
                      "no TF, reason names L and the range, red text in RViz",
                      s["state"] != lf.STATE_CALIBRATED and s["calibration_id"] is None
                      and s["reason"] == lf.LENGTH_OUT_OF_RANGE
                      and f"{length:.3f}" in s["detail"] and f"{low:.3f}" in s["detail"]
                      and f"{high:.3f}" in s["detail"]
                      and len(rec["tf"]) == n_tf
                      and rejected and "REJECTED" in rejected[0],
                      f"{s['reason']}: {s['detail'][:90]}")
        node.destroy_node()

    # M3: the ONE deliberate pin. Everything above follows the file; this says
    # the file still holds what the user decided, so widening both configs to,
    # say, 1-10 m cannot pass silently.
    for config in CONFIGS:
        check(f"{config}: the range IS the user decision of 2026-09-24, first real-robot "
              "test (2.20-2.80 m; was 2.5-3.0 m)",
              config_range(config) == (2.2, 2.8), str(config_range(config)))

    visible = {"gids": [[6, 6]]}
    node, rec, click = _cal_node(None, visible)
    click(0.0, 0.0)
    click(0.0, 2.5)  # inside the configured range - refused only for the missing config
    check("a node started WITHOUT the config refuses every pair (NO_LENGTH_RANGE)",
          rec["status"][-1]["reason"] == lf.NO_LENGTH_RANGE
          and rec["status"][-1]["calibration_id"] is None)
    node.destroy_node()


def part9_geofence():
    print("\n-- 9. the geofence IS the square of side L around the midpoint ---")
    # A rotated line (yaw +90 deg, origin (3,3)), L = 2.75: the square is
    # |u|, |v| <= 1.375 in the line frame, i.e. map x in [1.625, 4.375] and map
    # y in [1.625, 4.375] - here axis-aligned in map, because a 90 deg turn maps
    # the square onto itself. Hand-worked; a mirrored or off-centre square fails.
    c = cal((3.0, 1.625), (3.0, 4.375))
    inside = [(3.0, 3.0), (4.37, 1.63), (1.63, 4.37)]
    outside = [(4.38, 3.0), (3.0, 1.62), (1.62, 3.0), (3.0, 4.38)]
    check("points inside the square are inside",
          all(lf.geofence_contains(c, *p) for p in inside))
    check("points 5 mm outside any side are outside",
          not any(lf.geofence_contains(c, *p) for p in outside))
    # A 45-degree line: the square turns with it. Its corners in the line frame
    # are (+-L/2, +-L/2); the map point on the line's +x axis at 1.3 m is
    # inside, the one at 1.4 m is not - an AXIS-ALIGNED map square would accept
    # both corners differently.
    d = math.sqrt(0.5)
    c45 = cal((-1.375 * d, -1.375 * d), (1.375 * d, 1.375 * d))
    check("the square is aligned with the LINE, not with map axes",
          lf.geofence_contains(c45, 1.3 * d, 1.3 * d)
          and not lf.geofence_contains(c45, 1.4 * d, 1.4 * d)
          and lf.geofence_contains(c45, *c45.line_to_map(1.37, 1.37))
          and not lf.geofence_contains(c45, *c45.line_to_map(1.37, 1.38)))

    # Through the pipeline, WITHOUT the test hook: the gateway never passes one.
    tracker = DatumTracker(fallback=None, jump_warn_m=0.25)
    tracker.update(DATUM)
    base = dict(now_sec=100.1, datum_tracker=tracker,
                grasp_offset=GraspOffset(x=0.36, y=0.0, tolerance_m=0.05),
                robot_pose=(3.0, 3.0, 0.0), robot_pose_age_sec=0.05,
                costmap_cost=lambda x, y: 0)
    ctx = val.ValidationContext(goal=val.IncomingGoal("7", *their_latlon(0.5, 0.0), stamp_sec=100.0),
                                line_calibration=c, **base)
    r = val.validate_goal(ctx)
    check("a goal whose standing pose is inside the square is ACCEPTED", r.accepted, r.reason)
    check("  ... and the verdict of a pose outside is OUTSIDE_GEOFENCE",
          val.check_pose(ctx, 4.5, 3.0, 0.0).reason == val.OUTSIDE_GEOFENCE)
    ctx_far = val.ValidationContext(
        goal=val.IncomingGoal("8", *their_latlon(5.0, 0.0), stamp_sec=100.0),
        line_calibration=c, **base)
    r = val.validate_goal(ctx_far)
    check("a goal 5 m out along the line has no standing pose in the square -> refused",
          not r.accepted and r.reason in (val.OUTSIDE_GEOFENCE, val.NO_APPROACH_CANDIDATE), r.reason)
    no_cal = val.ValidationContext(goal=ctx.goal, **base)
    check("no calibration -> no geofence -> check_pose refuses (GEOFENCE_NOT_CONFIGURED)",
          val.check_pose(no_cal, 3.0, 3.0, 0.0).reason == val.GEOFENCE_NOT_CONFIGURED)


def main():
    part1_frame()
    part2_degenerate()
    part3_chain_and_status()
    part4_validation()
    part9_geofence()

    import rclpy
    rclpy.init(args=["--ros-args", "-p", "expected_domain_id:=220", "-p", "use_sim_time:=false"])
    try:
        part5_gateway(rclpy)
        part7_calibration_node(rclpy)
        part8_length_range(rclpy)
    finally:
        rclpy.shutdown()

    print("\n" + "=" * 74)
    if FAILURES:
        print(f"{len(FAILURES)} FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("All line-calibration checks passed.")
    print("NOT PROVEN HERE: click accuracy, the real Octopus side, or a real "
          "slam_toolbox restart - see check_stage3_twin.py --scenario line_cal "
          "for the process-level run.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
