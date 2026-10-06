#!/usr/bin/env python3
"""Verification of the fifth ingress topic: the Octopus's frame-lock status.

Pure python, no ROS, no DDS, no domain, no Octopus machine. Mirrors the style of
``check_geodesy.py`` / ``check_validation.py``:

    python3 src/gripperx_external/test/check_transform_status.py

WHAT THIS DOES NOT PROVE. It exercises the parser and the tracker against
payloads written here. **The link has never been run against their real
publisher**, so nothing below is evidence that
``/octopus/flight_camera_transform/status`` behaves as their message says it
does. The one payload that is not ours is the VERIFIED SAMPLE in part 1: it is
copied verbatim from the run they had up on 2026-08-21, including its derived
``align_angle`` of 265.9 deg, which is why that number is frozen here rather
than recomputed.

Part 1  the verified sample, and align_angle frozen against their arithmetic
Part 2  no lock: `null`, missing, and that neither ever becomes 0.0
Part 3  state, transform_ready and transform_mode carried through verbatim
Part 4  rejections: malformed JSON, wrong types, missing `state`, non-finite
Part 5  the tracker: first lock, republication, RE-LOCK, epsilon, gap
Part 6  the tracker decides NOTHING - the report-only property, structurally
"""

from __future__ import annotations

import ast
import json
import math
import os
import sys
from typing import List

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from gripperx_external import octopus_protocol as proto  # noqa: E402

_failures: List[str] = []


def check(condition: bool, label: str) -> bool:
    print(f"  [{'ok ' if condition else 'FAIL'}] {label}")
    if not condition:
        _failures.append(label)
    return bool(condition)


def rejects(payload, reason: str, label: str) -> bool:
    try:
        proto.parse_transform_status(payload)
    except proto.ProtocolError as exc:
        return check(exc.reason == reason, f"{label} -> {reason} ({exc.reason})")
    return check(False, f"{label} -> {reason} (accepted instead)")


# ---------------------------------------------------------------------------
# FROZEN: verbatim from their running system, 2026-08-21. Do not regenerate.
# ---------------------------------------------------------------------------
VERIFIED_SAMPLE = {
    "state": "ready",
    "transform_ready": True,
    "transform_mode": "indoor_static",
    "indoor_static_origin_x": 0.0,
    "indoor_static_origin_y": 0.0,
    "indoor_static_map_yaw_offset_rad": 1.57079632679,
    "indoor_static_yaw_zero_rad": -3.06995218682264,
}
#: Their own derived quantity, from their own message. 4.6407485... rad.
VERIFIED_ALIGN_DEG = 265.90


def part_verified_sample() -> None:
    print("\n-- 1. the verified sample ------------------------------------------")
    status = proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE))
    check(status.state == "ready" and status.ready, "state 'ready' parses and is ready")
    check(status.transform_ready is True, "transform_ready True")
    check(status.locked, "a non-null indoor_static_yaw_zero_rad is a lock")
    check(
        status.yaw_zero_rad == -3.06995218682264,
        "yaw_zero survives the JSON round trip exactly",
    )
    align = status.align_angle_rad
    check(align is not None, "align_angle is derivable when both angles are present")
    check(
        align is not None and abs(math.degrees(align) - VERIFIED_ALIGN_DEG) < 0.01,
        f"align_angle = map_yaw_offset - yaw_zero = {VERIFIED_ALIGN_DEG} deg "
        "(frozen against their arithmetic)",
    )
    check(status.origin_x == 0.0 and status.origin_y == 0.0, "origin 0,0 as on their branch")

    # The three wire forms the four existing parsers accept, all equivalent.
    bare = proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE))
    wrapped = proto.parse_transform_status({"data": json.dumps(VERIFIED_SAMPLE)})
    decoded = proto.parse_transform_status(VERIFIED_SAMPLE)
    check(bare == wrapped == decoded, "String wrapper, bare JSON string and decoded dict agree")


def part_no_lock() -> None:
    print("\n-- 2. no lock is not zero ------------------------------------------")
    payload = dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad=None)
    status = proto.parse_transform_status(json.dumps(payload))
    check(status.yaw_zero_rad is None, "a null yaw_zero stays None")
    check(status.yaw_zero_rad != 0.0, "a null yaw_zero is NOT 0.0")
    check(not status.locked, "a null yaw_zero means not locked")
    check(status.align_angle_rad is None, "align_angle is None without a lock, not 0.0")
    check(
        status.state == "ready" and status.transform_ready is True,
        "state and transform_ready are still carried through with no lock - they "
        "are theirs and are not reinterpreted",
    )

    missing = {k: v for k, v in VERIFIED_SAMPLE.items() if k != "indoor_static_yaw_zero_rad"}
    status = proto.parse_transform_status(json.dumps(missing))
    check(
        status.yaw_zero_rad is None and not status.locked,
        "a MISSING yaw_zero is the same as null: not locked, not 0.0",
    )

    half = {k: v for k, v in VERIFIED_SAMPLE.items() if k != "indoor_static_map_yaw_offset_rad"}
    status = proto.parse_transform_status(json.dumps(half))
    check(
        status.locked and status.align_angle_rad is None,
        "a lock with no map_yaw_offset is locked but has no derivable align_angle",
    )


def part_state_verbatim() -> None:
    print("\n-- 3. state carried through verbatim -------------------------------")
    for state in ("initializing", "waiting_for_yaw", "", "READY", "ready "):
        status = proto.parse_transform_status(json.dumps(dict(VERIFIED_SAMPLE, state=state)))
        check(status.state == state, f"state {state!r} carried through unchanged")
        check(
            status.ready == (state == "ready"),
            f"state {state!r} is ready only on an exact match",
        )

    status = proto.parse_transform_status(
        json.dumps(dict(VERIFIED_SAMPLE, state="ready", transform_ready=False))
    )
    check(
        status.transform_ready is False and status.ready,
        "state and transform_ready may disagree, and both are reported as they are",
    )

    without = {k: v for k, v in VERIFIED_SAMPLE.items() if k != "transform_ready"}
    status = proto.parse_transform_status(json.dumps(without))
    check(
        status.transform_ready is None,
        "a missing transform_ready is None (unavailable), NOT False",
    )

    without_mode = {k: v for k, v in VERIFIED_SAMPLE.items() if k != "transform_mode"}
    check(
        proto.parse_transform_status(json.dumps(without_mode)).transform_mode is None,
        "a missing transform_mode is None, not an empty string",
    )


def part_rejections() -> None:
    print("\n-- 4. structured rejections ----------------------------------------")
    rejects("{not json", "MALFORMED_JSON", "a truncated JSON body")
    rejects({"data": "[1, 2, 3]"}, "MALFORMED_PAYLOAD", "a JSON array instead of an object")
    rejects({"data": "17"}, "MALFORMED_PAYLOAD", "a bare JSON number")
    rejects(
        json.dumps({k: v for k, v in VERIFIED_SAMPLE.items() if k != "state"}),
        "MALFORMED_PAYLOAD",
        "a status message with no `state`",
    )
    rejects(
        json.dumps(dict(VERIFIED_SAMPLE, state=1)),
        "MALFORMED_PAYLOAD",
        "a non-string `state`",
    )
    rejects(
        json.dumps(dict(VERIFIED_SAMPLE, transform_ready="yes")),
        "MALFORMED_PAYLOAD",
        "a string where transform_ready should be a bool (no coercion)",
    )
    rejects(
        json.dumps(dict(VERIFIED_SAMPLE, transform_mode=3)),
        "MALFORMED_PAYLOAD",
        "a non-string transform_mode",
    )
    rejects(
        json.dumps(dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad="soon")),
        "MALFORMED_PAYLOAD",
        "a non-numeric yaw_zero",
    )
    rejects(
        '{"state": "ready", "indoor_static_yaw_zero_rad": NaN}',
        "MALFORMED_PAYLOAD",
        "a NaN yaw_zero (rejected loudly, not swallowed as 'no lock')",
    )
    rejects(
        json.dumps(dict(VERIFIED_SAMPLE, indoor_static_origin_x=True)),
        "MALFORMED_PAYLOAD",
        "a bool where an origin coordinate should be",
    )
    rejects(42, "MALFORMED_PAYLOAD", "an int instead of a payload")

    # The parse error carries a reason a log line can name.
    try:
        proto.parse_transform_status("{}")
    except proto.ProtocolError as exc:
        check(
            "state" in exc.detail and "flight_camera_transform/status" in exc.detail,
            "the rejection names the missing key and the topic it came from",
        )


def part_tracker() -> None:
    print("\n-- 5. the tracker: first lock, republication, RE-LOCK ----------------")
    tracker = proto.TransformLockTracker()

    waiting = proto.parse_transform_status(
        proto.build_transform_status("initializing", False, None, 1.57079632679)
    )
    update = tracker.update(waiting)
    check(
        not update.locked and not update.relocked and not update.first_lock,
        "a sample with no lock is neither a lock nor a re-lock",
    )
    check(tracker.yaw_zero_rad is None, "the tracker reports no lock, not 0.0")
    check(tracker.relocks == 0, "no re-lock counted before there is anything to lose")

    first = proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE))
    update = tracker.update(first)
    check(update.first_lock and not update.relocked, "the first lock is not a re-lock")
    check(tracker.relocks == 0, "the first lock does not increment the counter")
    check(
        tracker.align_angle_rad is not None
        and abs(math.degrees(tracker.align_angle_rad) - VERIFIED_ALIGN_DEG) < 0.01,
        "the tracker exposes their align_angle",
    )

    for _ in range(5):
        update = tracker.update(proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE)))
    check(
        not update.relocked and tracker.relocks == 0,
        "republishing the SAME lock at 1 Hz is not a re-lock (five samples)",
    )

    # Their transform node restarts and re-locks on a different drone heading.
    relocked_payload = dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad=-1.2)
    update = tracker.update(proto.parse_transform_status(json.dumps(relocked_payload)))
    check(update.relocked, "a changed yaw_zero IS a re-lock")
    check(tracker.relocks == 1, "the re-lock is counted")
    check(
        update.previous_yaw_zero_rad == -3.06995218682264 and update.yaw_zero_rad == -1.2,
        "the update carries the OLD and the NEW yaw_zero",
    )
    delta = update.align_angle_delta_rad
    check(
        delta is not None and abs(delta - (-3.06995218682264 - (-1.2))) < 1e-12,
        "the update carries the resulting align_angle change",
    )
    check(
        tracker.last_relock_from_rad == -3.06995218682264
        and tracker.last_relock_to_rad == -1.2
        and tracker.last_relock_align_delta_rad == delta,
        "the tracker latches the last re-lock for the diagnostic",
    )

    print("\n   epsilon behaviour")
    epsilon = proto.DEFAULT_RELOCK_EPSILON_RAD
    below = proto.parse_transform_status(
        json.dumps(dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad=-1.2 + epsilon / 2.0))
    )
    check(
        not tracker.update(below).relocked and tracker.relocks == 1,
        f"a move below the epsilon ({epsilon} rad) is representation noise, not a re-lock",
    )
    above = proto.parse_transform_status(
        json.dumps(dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad=-1.2 + epsilon * 10.0))
    )
    check(
        tracker.update(above).relocked and tracker.relocks == 2,
        "a move above the epsilon is a re-lock",
    )

    print("\n   a gap in the lock does not forget it")
    gapped = proto.TransformLockTracker()
    gapped.update(proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE)))
    gapped.update(
        proto.parse_transform_status(
            json.dumps(dict(VERIFIED_SAMPLE, state="initializing", indoor_static_yaw_zero_rad=None))
        )
    )
    check(
        gapped.yaw_zero_rad == -3.06995218682264,
        "a null sample does not clear the last known lock",
    )
    update = gapped.update(
        proto.parse_transform_status(json.dumps(dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad=0.4)))
    )
    check(
        update.relocked and not update.first_lock and gapped.relocks == 1,
        "a lock returning DIFFERENT after a gap is a re-lock, not a first lock - "
        "this is the exact restart sequence the topic exists to catch",
    )

    print("\n   a re-lock that lands on the same yaw is invisible, and that is known")
    same = proto.TransformLockTracker()
    same.update(proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE)))
    same.update(
        proto.parse_transform_status(
            json.dumps(dict(VERIFIED_SAMPLE, indoor_static_yaw_zero_rad=None))
        )
    )
    update = same.update(proto.parse_transform_status(json.dumps(VERIFIED_SAMPLE)))
    check(
        not update.relocked and same.relocks == 0,
        "a restart with the drone in the same pose is NOT detected - a limit of "
        "value comparison, recorded here so it is not discovered later",
    )


def part_report_only() -> None:
    print("\n-- 6. report only, structurally ------------------------------------")
    tracker = proto.TransformLockTracker()
    names = {n for n in dir(tracker) if not n.startswith("_")}
    forbidden = {"dispatch_blocker", "require_lock", "blocker", "disarm", "cancel", "gate"}
    check(
        not (names & forbidden),
        "TransformLockTracker exposes no gate, blocker or trigger of any kind "
        "(a re-lock's consequence is an open user decision, SAFETY.md F-40)",
    )

    node_path = os.path.join(
        _HERE, "..", "src", "gripperx_external", "octopus_link_node.py"
    )
    with open(node_path, encoding="utf-8") as handle:
        tree = ast.parse(handle.read())
    consumer = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_consume_transform_status":
            consumer = node
    check(consumer is not None, "octopus_link_node._consume_transform_status exists")
    if consumer is not None:
        calls = {
            n.func.attr
            for n in ast.walk(consumer)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        }
        acted = calls & {
            "publish", "disarm", "cancel", "cancel_goal", "cancel_goal_async",
            "blacklist", "send_goal_async", "set_parameters",
        }
        check(
            not acted,
            "the re-lock handler publishes nothing and calls nothing that acts: "
            f"it only tracks and logs (calls seen: {sorted(calls)})",
        )


def part_diagnostics_and_telemetry() -> None:
    """The two reporting surfaces. Needs ``diagnostic_msgs`` on the path but no
    domain, no node and no ``rclpy.init`` - message construction only."""
    print("\n-- 7. /diagnostics and the telemetry payload -----------------------")
    try:
        from gripperx_external import diagnostics as diag  # noqa: E402
    except ImportError as exc:
        print(f"  [skip] diagnostic_msgs not importable ({exc}); source the workspace")
        return

    def level_of(**kwargs):
        base = dict(
            subscribed=True,
            observed=True,
            state="ready",
            transform_ready=True,
            locked=True,
            yaw_zero_rad=-3.06995218682264,
            align_angle_rad=4.64074851361264,
            last_message_age_sec=0.4,
            stale_after_sec=5.0,
            relocks=0,
        )
        base.update(kwargs)
        st = diag.transform_lock_status(**base)
        return st.level, st.message, {kv.key: kv.value for kv in st.values}

    level, _, values = level_of()
    check(level == diag.OK, "ready + locked + fresh is OK")
    check(values["relocks"] == "0", "the relock count is on /diagnostics")
    check(
        abs(float(values["align_angle_deg"]) - VERIFIED_ALIGN_DEG) < 0.01,
        f"align_angle is reported in degrees too ({values['align_angle_deg']})",
    )

    check(level_of(observed=False)[0] == diag.WARN, "nothing received yet is a WARN")
    check(level_of(last_message_age_sec=30.0)[0] == diag.WARN, "a stale status is a WARN")
    check(
        level_of(locked=False, yaw_zero_rad=None, align_angle_rad=None, state="initializing")[0]
        == diag.WARN,
        "no lock is a WARN",
    )
    check(level_of(state="degraded")[0] == diag.WARN, "state != 'ready' is a WARN")
    check(level_of(subscribed=False)[0] == diag.OK, "deliberately not subscribed is OK, with a message")
    check(
        "not subscribed" in level_of(subscribed=False)[1],
        "... and the message says a re-lock could not be detected",
    )

    level, message, values = level_of(
        relocks=2, last_relock_from_rad=-3.07, last_relock_to_rad=-1.2,
        last_relock_align_delta_rad=-1.87,
    )
    check(
        level == diag.OK,
        "a past re-lock does NOT raise the level - it would latch forever and "
        "bury the live cases (SAFETY.md F-40's reasoning)",
    )
    check("RE-LOCKED 2 time(s)" in message, "... but it is named in the message")
    check(
        values["last_relock_from_rad"] == "-3.07" and values["last_relock_to_rad"] == "-1.2",
        "the old and new yaw are durable values, not only a log line",
    )

    check(
        level_of(yaw_zero_rad=None, align_angle_rad=None)[2]["last_known_yaw_zero_rad"]
        == "unavailable",
        "an absent lock reads as 'unavailable' on /diagnostics, never as 0",
    )

    print("\n   telemetry payload")
    body = json.loads(
        proto.build_device_status(
            latitude_deg=None, longitude_deg=None, map_x=None, map_y=None, yaw_deg=None,
            nav_state="idle", active_goal_id=None, armed=False, link_ok=True, stamp_sec=1.0,
        )
    )
    check(
        "octopus_transform" in body and body["octopus_transform"] is None,
        "the telemetry payload carries octopus_transform, null when we have nothing",
    )
    body = json.loads(
        proto.build_device_status(
            latitude_deg=None, longitude_deg=None, map_x=None, map_y=None, yaw_deg=None,
            nav_state="idle", active_goal_id=None, armed=False, link_ok=True, stamp_sec=1.0,
            octopus_transform={"state": "ready", "locked": True, "relocks": 1},
        )
    )
    check(
        body["octopus_transform"] == {"state": "ready", "locked": True, "relocks": 1},
        "... and passes the observed block through verbatim",
    )
    check(
        body["armed"] is False and "arming_seconds_remaining" in body,
        "the rest of the telemetry payload is unchanged by the addition",
    )


def main() -> int:
    print("=" * 78)
    print("gripperx_external - /octopus/flight_camera_transform/status (ingress 5)")
    print("=" * 78)
    part_verified_sample()
    part_no_lock()
    part_state_verbatim()
    part_rejections()
    part_tracker()
    part_report_only()
    part_diagnostics_and_telemetry()

    print()
    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All transform-status checks passed.")
    print(
        "NOT PROVEN HERE: any behaviour of their real publisher. The link has "
        "never been run against it."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
