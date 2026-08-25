#!/usr/bin/env python3
"""Verification of the object -> robot-pose resolution and approach ranking.

Pure python, no ROS required. Run from the workspace source tree:

    python3 src/gripperx_external/test/check_grasp.py

Part 1 is the most important one: while ``grasp.offset_x_m`` / ``offset_y_m``
are TO-VERIFY, every resolution path must REFUSE. There is no
kinematic model to derive the grasp point from (no arm link in the URDF) and no
camera to correct with, so a plausible-looking placeholder would produce goals
that validate, drive the robot, and miss.

Parts 2-4 use an explicitly fictional offset, marked as such at its definition,
purely to exercise the arithmetic and the ring ordering.
"""

from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from gripperx_external.grasp import (  # noqa: E402
    Candidate,
    CandidateVerdict,
    GraspOffset,
    GraspOffsetNotConfigured,
    TOLERANCE_NOT_CONFIGURED,
    bearing,
    candidate_headings,
    check_reached,
    choose_approach,
    grasp_point_for,
    normalize_angle,
    parse_measured_param,
    robot_pose_for,
)

# NOT A MEASUREMENT. An arbitrary test fixture so the geometry can be exercised
# offline; the real values come from the bench procedure on the real robot and
# must never be copied from here into a config file.
FIXTURE_OFFSET = GraspOffset(x=0.35, y=0.0, tolerance_m=0.05)
FIXTURE_OFFSET_LATERAL = GraspOffset(x=0.30, y=0.10, tolerance_m=0.05)

TOL = 1e-9

_failures = []


def check(condition: bool, label: str, detail: str = "") -> None:
    mark = "PASS" if condition else "FAIL"
    print(f"  [{mark}] {label}" + (f"  ({detail})" if detail else ""))
    if not condition:
        _failures.append(label)


def close(a: float, b: float, tol: float = TOL) -> bool:
    return abs(a - b) <= tol


def accept_all(_: Candidate) -> CandidateVerdict:
    return CandidateVerdict(True)


def main() -> int:
    print("=" * 78)
    print("Part 1 - an unmeasured grasp offset must refuse, not guess")
    print("=" * 78)

    unset = GraspOffset()
    check(not unset.configured, "a default GraspOffset is not configured")
    check(unset.standoff_m is None, "an unconfigured offset has no standoff")

    to_verify = GraspOffset.from_params("TO-VERIFY", "TO-VERIFY", "TO-VERIFY")
    check(not to_verify.configured, "the literal 'TO-VERIFY' does not become a number")

    partial = GraspOffset.from_params(0.35, "TO-VERIFY", 0.05)
    check(
        not partial.configured,
        "a half-measured offset is not configured either",
        "one axis measured is not a grasp point",
    )

    nan_offset = GraspOffset.from_params(float("nan"), 0.0, 0.05)
    check(not nan_offset.configured, "a YAML .nan does not become a silent zero")
    check(
        GraspOffset.from_params(True, 0.0, 0.05).configured is False,
        "a boolean parameter value is refused",
    )
    check(parse_measured_param("0.35") == 0.35, "a numeric string is parsed")
    check(parse_measured_param(0) == 0.0, "a genuine 0.0 IS a valid measurement")
    check(parse_measured_param("") is None, "an empty string is TO-VERIFY")
    check(parse_measured_param("garbage") is None, "unparseable text is TO-VERIFY")

    for label, call in (
        ("require()", lambda: to_verify.require()),
        ("robot_pose_for()", lambda: robot_pose_for((1.0, 0.0), 0.0, to_verify)),
        (
            "choose_approach()",
            lambda: choose_approach((1.0, 0.0), (0.0, 0.0), to_verify, accept_all),
        ),
    ):
        raised = False
        try:
            call()
        except GraspOffsetNotConfigured:
            raised = True
        check(raised, f"{label} raises GraspOffsetNotConfigured while TO-VERIFY")
    check(
        GraspOffsetNotConfigured.reason == "GRASP_OFFSET_NOT_CONFIGURED",
        "the refusal carries a machine-readable reason",
    )

    # -- user decision 2026-08-19: the tolerance gates the REACHED check only --
    # It appears nowhere in the standoff arithmetic, so coupling it to
    # resolution gated the wrong thing: an unmeasured tolerance used to block
    # every goal from resolving at all. Resolution now works without it; the
    # reached verdict becomes UNKNOWN instead, loudly, and never a number.
    offset_only = GraspOffset.from_params(0.360, 0.000, "TO-VERIFY")
    check(
        offset_only.configured,
        "a measured offset with a TO-VERIFY tolerance RESOLVES (decision 2026-08-19)",
    )
    check(
        not offset_only.tolerance_configured,
        "  ... and still reports the tolerance as unmeasured",
    )
    check(
        close(offset_only.standoff_m or -1.0, 0.360),
        "  ... and its standoff is the offset length, tolerance not involved",
    )
    resolved = robot_pose_for((1.0, 0.0), 0.0, offset_only)
    check(
        close(resolved[0], 0.640) and close(resolved[1], 0.0),
        "  ... and the standoff pose is computed from offset_x/y alone",
        f"{resolved}",
    )

    # The reached check: distance is always reported, the VERDICT is withheld
    # while the window is unmeasured. Neither "yes" nor "no" may be invented.
    verdict = check_reached((1.0, 0.0), (0.640, 0.0, 0.0), offset_only)
    check(not verdict.known, "an unmeasured tolerance makes the reached check UNKNOWN")
    check(
        verdict.reason == TOLERANCE_NOT_CONFIGURED,
        "  ... with a machine-readable reason, not a silent pass",
    )
    check(
        not verdict.reached,
        "  ... and 'reached' is not asserted while it is unknown",
    )
    check(
        close(verdict.distance_m, 0.0, 1e-9),
        "  ... while the measured distance is still reported",
        f"{verdict.distance_m:.6f} m",
    )

    measured = GraspOffset.from_params(0.360, 0.000, 0.050)
    inside = check_reached((1.0, 0.0), (0.660, 0.0, 0.0), measured)
    check(
        inside.known and inside.reached and close(inside.distance_m, 0.02),
        "with a measured tolerance, 0.02 m off is inside the 0.05 m window",
        f"{inside.distance_m:.3f} m",
    )
    outside = check_reached((1.0, 0.0), (0.500, 0.0, 0.0), measured)
    check(
        outside.known and not outside.reached,
        "  ... and 0.14 m off is outside it, with a reason",
        outside.reason,
    )
    # The pose the robot actually reached is not the pose it was given: Nav2
    # stops within its own tolerances, so the grasp point rotates with the yaw
    # error. That is exactly what this check exists to catch.
    rotated = check_reached((1.0, 0.0), (0.640, 0.0, math.radians(20.0)), measured)
    check(
        rotated.known and not rotated.reached,
        "  ... and a 20 deg yaw error at the same position misses the window",
        f"{rotated.distance_m:.3f} m",
    )

    print()
    print("=" * 78)
    print("Part 2 - robot_goal_xy = object_xy - R(theta) * grasp_offset")
    print("=" * 78)
    check(
        close(FIXTURE_OFFSET.standoff_m, 0.35),
        "standoff is the length of the offset vector",
        f"{FIXTURE_OFFSET.standoff_m:.3f} m",
    )

    x, y, yaw = robot_pose_for((2.0, 0.0), 0.0, FIXTURE_OFFSET)
    check(
        close(x, 1.65) and close(y, 0.0) and close(yaw, 0.0),
        "object at (2, 0), heading 0 -> robot stops short at (1.65, 0) facing 0",
        f"({x:.3f}, {y:.3f}, {yaw:.3f})",
    )

    x, y, yaw = robot_pose_for((0.0, 2.0), math.pi / 2.0, FIXTURE_OFFSET)
    check(
        close(x, 0.0) and close(y, 1.65) and close(yaw, math.pi / 2.0),
        "object at (0, 2), heading +90 deg -> robot at (0, 1.65) facing +90 deg",
        f"({x:.3f}, {y:.3f}, {yaw:.3f})",
    )

    x, y, _ = robot_pose_for((2.0, 0.0), 0.0, FIXTURE_OFFSET_LATERAL)
    check(
        close(x, 1.70) and close(y, -0.10),
        "a lateral grasp offset displaces the standing pose sideways",
        f"({x:.3f}, {y:.3f})",
    )

    worst = 0.0
    for theta in (0.0, 0.3, 1.0, math.pi / 2.0, 2.5, -1.1, math.pi):
        for offset in (FIXTURE_OFFSET, FIXTURE_OFFSET_LATERAL):
            obj = (1.234, -0.567)
            rx, ry, ryaw = robot_pose_for(obj, theta, offset)
            gx, gy = grasp_point_for((rx, ry), ryaw, offset)
            worst = max(worst, math.hypot(gx - obj[0], gy - obj[1]))
    check(
        worst <= 1e-12,
        "the resolved pose puts the grasp point exactly on the object",
        f"worst error {worst:.3e} m",
    )

    # The standoff distance must not depend on the heading: that is what makes
    # the ring walk safe to do at all.
    distances = [
        math.hypot(*(robot_pose_for((0.0, 0.0), t, FIXTURE_OFFSET)[:2]))
        for t in (0.0, 0.7, 1.9, -2.4)
    ]
    check(
        max(distances) - min(distances) <= 1e-12,
        "the standoff distance is heading-independent",
        f"{min(distances):.6f}..{max(distances):.6f} m",
    )

    print()
    print("=" * 78)
    print("Part 3 - approach-candidate ranking")
    print("=" * 78)
    check(close(bearing((0.0, 0.0), (1.0, 0.0)), 0.0), "bearing east is 0")
    check(
        close(bearing((0.0, 0.0), (0.0, 1.0)), math.pi / 2.0), "bearing north is +90 deg"
    )
    check(close(normalize_angle(math.tau + 0.1), 0.1), "normalize_angle wraps a full turn")
    check(close(normalize_angle(math.pi), math.pi), "normalize_angle keeps +pi as +pi")

    headings = candidate_headings(0.0, 12)
    degrees = [round(math.degrees(h), 6) for h in headings]
    expected = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 150.0, -150.0, 180.0]
    check(len(headings) == 12, "12 candidates are generated", str(len(headings)))
    check(
        degrees == expected,
        "the ring is walked by increasing deviation, CCW first",
        str(degrees),
    )
    check(
        len(set(degrees)) == len(degrees), "no heading is offered twice", str(len(set(degrees)))
    )
    shifted = candidate_headings(math.radians(45.0), 12)
    check(
        close(shifted[0], math.radians(45.0)),
        "the first candidate is the seed heading itself",
        f"{math.degrees(shifted[0]):.1f} deg",
    )
    check(len(candidate_headings(0.3, 1)) == 1, "count=1 yields only the seed")

    # Free field: the seed wins, i.e. the normal case is "drive at it, stop short".
    result = choose_approach((2.0, 0.0), (0.0, 0.0), FIXTURE_OFFSET, accept_all)
    check(result.ok, "a free field accepts the seed heading")
    check(
        close(result.chosen.deviation_rad, 0.0) and len(result.evaluated) == 1,
        "only one candidate is even evaluated in the free field",
        f"{len(result.evaluated)} evaluated",
    )
    check(
        close(result.chosen.robot_x, 1.65) and close(result.chosen.robot_y, 0.0),
        "the chosen pose is the seed pose",
        f"({result.chosen.robot_x:.3f}, {result.chosen.robot_y:.3f})",
    )

    # A wall behind the object: refuse anything that would stand at x > 2.0.
    def wall_behind(candidate: Candidate) -> CandidateVerdict:
        if candidate.robot_x > 2.0:
            return CandidateVerdict(False, "OUTSIDE_GEOFENCE", "behind the wall")
        return CandidateVerdict(True)

    # Robot approaching from the far side, so the seed heading would put it in
    # the wall - exactly the case the ring exists for.
    result = choose_approach((2.0, 0.0), (4.0, 0.0), FIXTURE_OFFSET, wall_behind)
    check(result.ok, "an object against a wall still finds an approach")
    check(
        result.chosen is not None and result.chosen.robot_x <= 2.0,
        "the chosen pose is on the free side of the wall",
        f"x = {result.chosen.robot_x:.3f}",
    )
    check(
        abs(result.chosen.deviation_rad) > 1e-6,
        "it is not the seed heading, and the deviation is reported",
        f"{math.degrees(result.chosen.deviation_rad):+.1f} deg",
    )

    # Skip exactly the first three candidates and check which one is taken.
    def skip_first_three(candidate: Candidate) -> CandidateVerdict:
        if candidate.index < 3:
            return CandidateVerdict(False, "COST_TOO_HIGH", f"index {candidate.index}")
        return CandidateVerdict(True)

    result = choose_approach((2.0, 0.0), (0.0, 0.0), FIXTURE_OFFSET, skip_first_three)
    check(
        result.ok and close(math.degrees(result.chosen.deviation_rad), 60.0),
        "the fourth candidate is +60 deg, per the deviation ordering",
        f"{math.degrees(result.chosen.deviation_rad):+.1f} deg",
    )
    check(
        len(result.evaluated) == 4,
        "evaluation stops at the first acceptable candidate",
        f"{len(result.evaluated)} evaluated",
    )

    # Nothing acceptable -> refuse, with the per-candidate reasons preserved.
    def reject_all(candidate: Candidate) -> CandidateVerdict:
        reason = "OUTSIDE_COSTMAP" if candidate.index % 2 else "OUTSIDE_GEOFENCE"
        return CandidateVerdict(False, reason, f"index {candidate.index}")

    result = choose_approach((2.0, 0.0), (0.0, 0.0), FIXTURE_OFFSET, reject_all)
    check(not result.ok, "no acceptable candidate -> the goal is refused")
    check(
        len(result.evaluated) == 12,
        "all 12 candidates are tried before refusing",
        f"{len(result.evaluated)} evaluated",
    )
    check(
        set(result.failure_reasons()) == {"OUTSIDE_GEOFENCE", "OUTSIDE_COSTMAP"},
        "the per-candidate reasons survive for the rejection report",
        str(sorted(set(result.failure_reasons()))),
    )

    print()
    print("=" * 78)
    print("Part 4 - overrides and warnings")
    print("=" * 78)
    result = choose_approach(
        (2.0, 0.0),
        (0.0, 0.0),
        FIXTURE_OFFSET,
        accept_all,
        candidate_count=1,
        seed_theta=math.pi,
    )
    check(
        result.ok and close(result.chosen.theta, math.pi),
        "an operator heading override is honoured verbatim",
        f"{math.degrees(result.chosen.theta):.1f} deg",
    )
    check(
        close(result.chosen.robot_x, 2.35) and close(result.chosen.robot_y, 0.0),
        "the override resolves to the pose on the opposite side",
        f"({result.chosen.robot_x:.3f}, {result.chosen.robot_y:.3f})",
    )

    def warn_once(candidate: Candidate) -> CandidateVerdict:
        return CandidateVerdict(True, warnings=("UNKNOWN_COSTMAP_CELL",))

    result = choose_approach((2.0, 0.0), (0.0, 0.0), FIXTURE_OFFSET, warn_once)
    check(
        result.ok and result.warnings == ("UNKNOWN_COSTMAP_CELL",),
        "warnings from the acceptance test are propagated, not swallowed",
        str(result.warnings),
    )

    print()
    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All grasp checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
