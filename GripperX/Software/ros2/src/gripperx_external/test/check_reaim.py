#!/usr/bin/env python3
"""Verification of the re-aim at arrival: geometry, decision, config, structure.

Pure python, no ROS required. Run from the workspace source tree:

    python3 src/gripperx_external/test/check_reaim.py

Part 1 pins the SIGN CONVENTION and the +-pi wrap of the object bearing, because
a sign error there turns every re-aim into a rotation AWAY from the object - the
failure that would look like "the re-aim made it worse" on the robot.
Part 2 is the decision function. Part 3 reads the ACTIVE configs (both
octopus_link_*.yaml and gripperx_planning's nav2.yaml and trees) and duplicates
no value, so it fails on a config edit, not only on a code edit. Part 4 is
structure: the re-aim can never lead to a pick on a failure path, and its
bounds cannot be widened at runtime.

Every GraspOffset here is a FIXTURE (the user's 0.360 m specification, or an
explicitly fictional lateral one) - none is a measurement.
"""

from __future__ import annotations

import ast
import math
import re
import os
import sys
import xml.etree.ElementTree as ET

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
_SRC_ROOT = os.path.dirname(_PKG)
sys.path.insert(0, os.path.join(_PKG, "src"))

from gripperx_external.grasp import (  # noqa: E402
    REAIM_BEYOND_MAX,
    REAIM_CANNOT_HELP,
    REAIM_DISABLED,
    REAIM_NEEDED,
    REAIM_NO_GEOMETRY,
    REAIM_WITHIN_MIN,
    GraspOffset,
    check_reached,
    decide_reaim,
    grasp_point_for,
    object_in_base,
)

# FIXTURES, NOT MEASUREMENTS.
SPEC_OFFSET = GraspOffset(x=0.360, y=0.0, tolerance_m=None)
SPEC_OFFSET_TOL = GraspOffset(x=0.360, y=0.0, tolerance_m=0.10)
FICTIONAL_LATERAL = GraspOffset(x=0.30, y=0.10, tolerance_m=None)

MIN = math.radians(5.0)
MAX = math.radians(20.0)
TOL = 1e-9

_failures = []


def check(condition: bool, label: str, detail: str = "") -> None:
    mark = "PASS" if condition else "FAIL"
    print(f"  [{mark}] {label}" + (f"  ({detail})" if detail else ""))
    if not condition:
        _failures.append(label)


def close(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) <= tol


def deg(rad: float) -> float:
    return math.degrees(rad)


def part1_bearing() -> None:
    print("part 1 - bearing sign convention and the +-pi wrap")
    rng, brg = object_in_base((0.36, 0.05), (0.0, 0.0, 0.0))
    check(brg > 0.0, "object to the LEFT (+y in base_footprint) has POSITIVE bearing (REP-103)",
          f"{deg(brg):+.2f} deg")
    check(close(rng, math.hypot(0.36, 0.05)), "  ... and the range is the plain distance")
    _, brg = object_in_base((0.36, -0.05), (0.0, 0.0, 0.0))
    check(brg < 0.0, "object to the RIGHT has NEGATIVE bearing", f"{deg(brg):+.2f} deg")
    _, brg = object_in_base((1.0, 2.36), (1.0, 2.0, math.pi / 2))
    check(close(brg, 0.0), "the bearing is relative to the robot's yaw, not the map's x axis",
          f"{deg(brg):+.6f} deg")

    # The wrap: robot facing +179 deg, object seen at map direction -179 deg is
    # 2 deg to the LEFT, not 358 deg to the right.
    yaw = math.radians(179.0)
    obj = (math.cos(math.radians(-179.0)), math.sin(math.radians(-179.0)))
    _, brg = object_in_base(obj, (0.0, 0.0, yaw))
    check(close(brg, math.radians(2.0), 1e-9), "robot at +179 deg, object at -179 deg: +2 deg, wrapped",
          f"{deg(brg):+.6f} deg")
    _, brg = object_in_base((obj[0], -obj[1]), (0.0, 0.0, -yaw))
    check(close(brg, math.radians(-2.0), 1e-9), "robot at -179 deg, object at +179 deg: -2 deg, wrapped",
          f"{deg(brg):+.6f} deg")
    _, brg = object_in_base((-1.0, 0.0), (0.0, 0.0, 0.0))
    check(close(brg, math.pi), "object dead astern is +pi (never -pi), as normalize_angle defines",
          f"{deg(brg):+.6f} deg")


def part2_decision() -> None:
    print("part 2 - decide_reaim")
    obj = (0.40, 0.06)
    pose = (0.0, 0.0, 0.0)
    d = decide_reaim(obj, pose, SPEC_OFFSET, False, MIN, MAX)
    check(not d.rotate and d.reason == REAIM_DISABLED and d.target_yaw == 0.0,
          "disabled: no rotation, target yaw is the current yaw", d.reason)

    d = decide_reaim(obj, pose, SPEC_OFFSET, True, MIN, MAX)
    expected = math.atan2(0.06, 0.40)
    check(d.rotate and d.reason == REAIM_NEEDED, "8.5 deg to the left, enabled: rotate",
          f"{deg(d.aim_error_rad):+.2f} deg")
    check(close(d.aim_error_rad, expected) and close(d.target_yaw, expected),
          "  ... COUNTER-CLOCKWISE (target yaw = yaw + positive aim error)",
          f"target {deg(d.target_yaw):+.3f} deg")
    check(close(d.predicted_distance_m, abs(math.hypot(*obj) - 0.36), 1e-12),
          "  ... and the predicted grasp-point distance is the pure range error |r - 0.36|",
          f"{d.predicted_distance_m:.4f} m")
    after = decide_reaim(obj, (0.0, 0.0, d.target_yaw), SPEC_OFFSET, True, MIN, MAX)
    check(after.reason == REAIM_WITHIN_MIN and close(after.bearing_rad, 0.0, 1e-12),
          "  ... and from the re-aimed pose the object is dead ahead, so no second re-aim",
          f"{deg(after.bearing_rad):+.9f} deg")

    d = decide_reaim((0.40, -0.06), pose, SPEC_OFFSET, True, MIN, MAX)
    check(d.rotate and d.target_yaw < 0.0, "object to the right: rotate CLOCKWISE",
          f"target {deg(d.target_yaw):+.2f} deg")

    d = decide_reaim((0.40, 0.03), pose, SPEC_OFFSET, True, MIN, MAX)
    check(not d.rotate and d.reason == REAIM_WITHIN_MIN, "4.3 deg is within the 5 deg minimum",
          f"{deg(d.aim_error_rad):+.2f} deg")
    d = decide_reaim((0.40, 0.0349), pose, SPEC_OFFSET, True, math.atan2(0.0349, 0.40), MAX)
    check(not d.rotate, "exactly AT the minimum is not rotated (strictly above rotates)")

    d = decide_reaim((0.30, 0.20), pose, SPEC_OFFSET, True, MIN, MAX)
    check(not d.rotate and d.reason == REAIM_BEYOND_MAX and d.target_yaw == 0.0,
          "33.7 deg exceeds the maximum: refused, not clamped", f"{deg(d.aim_error_rad):+.1f} deg")

    d = decide_reaim((0.55, 0.08), pose, SPEC_OFFSET_TOL, True, MIN, MAX)
    check(not d.rotate and d.reason == REAIM_CANNOT_HELP,
          "with a tolerance set, a range error the rotation cannot remove is not rotated for",
          f"predicted {d.predicted_distance_m:.3f} m > 0.10")
    d = decide_reaim((0.42, 0.06), pose, SPEC_OFFSET_TOL, True, MIN, MAX)
    check(d.rotate, "  ... and one it can bring inside the tolerance is",
          f"predicted {d.predicted_distance_m:.3f} m")
    v_before = check_reached((0.42, 0.06), pose, SPEC_OFFSET_TOL)
    v_after = check_reached((0.42, 0.06), (0.0, 0.0, d.target_yaw), SPEC_OFFSET_TOL)
    check(v_after.distance_m < v_before.distance_m and close(v_after.distance_m, d.predicted_distance_m),
          "  ... the reached check at the re-aimed heading equals the prediction and beats the arrival",
          f"{v_before.distance_m:.3f} -> {v_after.distance_m:.3f} m")

    # Wrap in the decision: robot at +175 deg, object 10 deg further CCW, i.e.
    # at map direction -175 deg. The target must be -175 deg, reached by +10.
    yaw = math.radians(175.0)
    heading = math.radians(-175.0)
    obj = (0.40 * math.cos(heading), 0.40 * math.sin(heading))
    d = decide_reaim(obj, (0.0, 0.0, yaw), SPEC_OFFSET, True, MIN, MAX)
    check(d.rotate and close(d.aim_error_rad, math.radians(10.0), 1e-9)
          and close(d.target_yaw, heading, 1e-9),
          "across the +-pi seam: +10 deg to -175 deg, never -350 deg",
          f"aim {deg(d.aim_error_rad):+.3f}, target {deg(d.target_yaw):+.3f} deg")

    # A lateral grasp offset: the object must end up on the GRASP RAY, not dead
    # ahead. Fictional offset, purely to exercise the arithmetic.
    obj = (0.35, 0.25)
    d = decide_reaim(obj, pose, FICTIONAL_LATERAL, True, MIN, math.radians(60.0))
    gx, gy = grasp_point_for((0.0, 0.0), d.target_yaw, FICTIONAL_LATERAL)
    check(close(math.atan2(gy, gx), math.atan2(obj[1], obj[0]), 1e-12),
          "lateral grasp offset: the rotation puts the object on the grasp ray",
          f"target {deg(d.target_yaw):+.2f} deg")
    check(close(d.predicted_distance_m, abs(math.hypot(*obj) - math.hypot(0.30, 0.10)), 1e-12),
          "  ... leaving only the range error")

    d = decide_reaim((float("nan"), 0.0), pose, SPEC_OFFSET, True, MIN, MAX)
    check(not d.rotate and d.reason == REAIM_NO_GEOMETRY,
          "a NaN object position never becomes a rotation", d.reason)


def _load_yaml(path: str):
    import yaml
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def part3_config() -> None:
    print("part 3 - the active configuration")
    try:
        import yaml  # noqa: F401
    except ImportError:
        check(False, "PyYAML is required for the config checks")
        return
    nav2 = _load_yaml(os.path.join(_SRC_ROOT, "gripperx_planning", "config", "nav2.yaml"))
    ctrl = nav2["controller_server"]["ros__parameters"]
    checkers = {name: ctrl[name] for name in ctrl["goal_checker_plugins"]}
    allowance = float(ctrl["progress_checker"]["movement_time_allowance"])
    for name in ("octopus_link_real.yaml", "octopus_link_twin.yaml"):
        params = _load_yaml(os.path.join(_PKG, "config", name))
        gateway = params["/gripperx/external/goal_gateway_node"]["ros__parameters"]
        spec = str(gateway["grasp.reaim_behavior_tree"])
        package, _, rest = spec.partition("/")
        tree = os.path.join(_SRC_ROOT, package, rest)
        check(os.path.isfile(tree), f"{name}: the re-aim behavior tree exists in the source tree", tree)
        if not os.path.isfile(tree):
            continue
        follow = list(ET.parse(tree).getroot().iter("FollowPath"))
        checker = follow[0].get("goal_checker_id", "") if len(follow) == 1 else ""
        check(checker in checkers, f"{name}: its FollowPath names a LOADED goal checker", checker)
        if checker not in checkers:
            continue
        tol_deg = math.degrees(float(checkers[checker]["yaw_goal_tolerance"]))
        approach_deg = math.degrees(float(checkers["goal_checker"]["yaw_goal_tolerance"]))
        min_deg = float(gateway["grasp.reaim_min_deg"])
        max_deg = float(gateway["grasp.reaim_max_deg"])
        timeout = float(gateway["grasp.reaim_timeout_sec"])
        check(tol_deg < min_deg,
              f"{name}: reaim_min_deg is strictly above the re-aim checker's yaw tolerance - "
              "otherwise a re-aim can 'succeed' without having moved",
              f"{tol_deg:.2f} < {min_deg:.2f} deg")
        check(tol_deg < approach_deg,
              f"{name}: the re-aim checker is tighter than the approach checker",
              f"{tol_deg:.2f} < {approach_deg:.2f} deg")
        check(0.0 <= min_deg < max_deg <= 90.0, f"{name}: 0 <= min < max <= 90 deg",
              f"{min_deg} / {max_deg}")
        check(0.0 < timeout < allowance,
              f"{name}: reaim_timeout_sec ends a stalled rotation before Nav2's progress "
              "checker aborts it",
              f"{timeout} < {allowance}")


def part4_structure() -> None:
    print("part 4 - structure of the gateway")
    path = os.path.join(_PKG, "src", "gripperx_external", "goal_gateway_node.py")
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    tree = ast.parse(source)
    functions = {
        node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
    }

    def calls(fn: str, attr: str) -> int:
        return sum(
            1
            for node in ast.walk(functions[fn])
            if isinstance(node, ast.Attribute) and node.attr == attr
        )

    def names(fn: str, name: str) -> int:
        return sum(
            1 for node in ast.walk(functions[fn]) if isinstance(node, ast.Name) and node.id == name
        )

    pick_sites = [
        name for name, fn in functions.items()
        if any(isinstance(n, ast.Attribute) and n.attr == "_pick_client" for n in ast.walk(fn))
        and any(isinstance(n, ast.Attribute) and n.attr == "send_goal_async" for n in ast.walk(fn))
    ]
    check(pick_sites == ["_on_arrival"],
          "PickPlastic is sent from _on_arrival ONLY - no re-aim callback can pick",
          ", ".join(pick_sites))
    for fn in ("_on_reaim_goal_response", "_on_reaim_result", "_start_reaim"):
        check(calls(fn, "_pick_client") == 0, f"  ... {fn} does not touch the pick client")
    check(calls("_start_reaim", "validate_dispatch") == 1,
          "_start_reaim sends nothing that has not passed validate_dispatch (C-5)")
    check(calls("_start_reaim", "_mission_lock") >= 1 and calls("_start_reaim", "cancelling") >= 1,
          "  ... and publishes its state under the mission lock, checking for a cancel "
          "that landed first (SR-15 rule 13 construction)")
    check(calls("_on_reaim_goal_response", "_send_cancel") == 1,
          "a re-aim accepted after the gate closed is cancelled the moment it has a handle")
    check(calls("_on_reaim_result", "note_goal_aborted") == 1,
          "an aborted re-aim counts against the consecutive-abort budget, like any abort")

    # User decisions 2026-09-29.
    check(calls("_on_reaim_result", "_reaim_fall_back") == 1,
          "only ONE result path falls back to the arrival gate (the confirmed "
          "timeout cancel); ABORTED and safety cancels do not")
    check(calls("_reaim_fall_back", "_begin_settle") == 1
          and calls("_reaim_fall_back", "_on_arrival") == 0
          and calls("_reaim_fall_back", "cancelling") >= 1,
          "the fallback always settles before the arrival gate (F-49), never for a "
          "cancelled mission")
    check(calls("_on_nav_result", "_begin_settle") == 1
          and calls("_on_nav_result", "_on_arrival") == 0,
          "the approach's success settles before the arrival gate (user decision)")
    check(names("_start_reaim", "_REAIM_STATE_REJECTIONS") == 1,
          "a re-aim refused on the gate's STATE (disarmed, link, mode, ...) never falls back")
    check(calls("_cancel_mission", "reaim_own_cancel_reason") >= 1,
          "a safety cancel SUPERSEDES the gateway's own re-aim cancel (timeout, switch-OFF)")
    check("if mission.state == NAV_REAIMING:\n            holds, detail = "
          "self._reaim_correlation_holds(mission)" in source,
          "#358: the re-aim in flight uses the re-aim-phase correlation rule")
    check("self._correlation_holds(mission)\n            if first_pass\n            else "
          "self._reaim_correlation_holds(mission)" in source,
          "#358: ... and so does the pick gate after it; the first pass keeps the full rule")
    check("REAIM_CORRELATION_VARIANT" not in source,
          "variant B is final: no switch left that could select 'no re-check at all'")
    check(names("_reaim_correlation_holds", "_REAIM_ABSENCE_STATUSES") == 1
          and re.search(r"^_REAIM_ABSENCE_STATUSES = frozenset\(\(corr\.NO_MATCH, "
                        r"corr\.NO_TARGETS\)\)$", source, re.M) is not None,
          "  ... and only ABSENCE (NO_MATCH, NO_TARGETS) is tolerated - not a stale list")
    check(calls("_acknowledge", "_correlation_holds") == 1
          and calls("_acknowledge", "_reaim_correlation_holds") == 0,
          "the irreversible acknowledgement keeps the FULL correlation gate")
    check(calls("_supervise_mission", "_mission_dispatch_context") == 1
          and calls("_supervise_mission", "validate_dispatch") == 1,
          "  ... while the dispatch gate (armed, link, mode, geofence) still runs every tick")

    startup = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "_STARTUP_ONLY_PARAMS" for t in node.targets)
    )
    keys = {k.value for k in startup.value.keys if isinstance(k, ast.Constant)}
    for key in ("grasp.reaim_min_deg", "grasp.reaim_max_deg", "grasp.reaim_timeout_sec",
                "grasp.reaim_behavior_tree"):
        check(key in keys, f"{key} is startup-only: a running node cannot widen the motion bound")
    check("grasp.reaim_enabled" not in keys,
          "grasp.reaim_enabled is NOT startup-only - switching it OFF at runtime is the kill switch")
    check('param.name == "grasp.reaim_enabled" and param.value is not False' in source,
          "  ... but switching it ON at runtime is refused")
    check("NAV_REAIMING)" in source and "mission.state not in (NAV_NAVIGATING, NAV_REAIMING)" in source,
          "a re-aim in flight is supervised (correlation + dispatch gate) like the approach")


def main() -> int:
    part1_bearing()
    part2_decision()
    part3_config()
    part4_structure()
    print()
    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All re-aim checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
