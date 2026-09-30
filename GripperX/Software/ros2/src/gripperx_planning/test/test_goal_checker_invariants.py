"""Goal-checker invariants of the ACTIVE Nav2 configuration and behavior trees.

WHAT THIS TEST IS FOR
    Since 2026-09-29 ``controller_server`` loads two goal checkers:
    ``goal_checker`` for every approach and ``reaim_goal_checker`` for the
    external-goal gateway's in-place re-aim at arrival (``reaim_in_place.xml``,
    sent per goal as ``NavigateToPose.behavior_tree``). Three things break
    silently or at runtime if the files drift apart, and each is held here:

    1. With more than one checker, nav2 1.3.12's ``controller_server`` refuses a
       FollowPath goal whose ``goal_checker_id`` is empty (string verified in
       the installed ``controller_server`` binary: "FollowPath called with
       goal_checker name %s ... which does not exist"). An unnamed checker in
       ANY tree therefore aborts every goal that tree runs - so every
       ``<FollowPath>`` must name a loaded checker, literally.
    2. ``FollowPath.Oscillation.oscillation_reset_angle`` must stay strictly
       below the yaw tolerance of EVERY checker FollowPath can be given
       (derivation at that key in nav2.yaml).
    3. The re-aim tree must stay a rotation and nothing else: no recovery
       branch (no BackUp / CrabWalk / DriveOnHeading next to the object), and
       its checker must be tighter than the approach's, or the re-aim reports
       success without moving - the 2026-09-29 observation it exists for.

WHAT WOULD MAKE IT FAIL
    An edit to nav2.yaml or to any ``config/*.xml`` - the machine's
    configuration, not this file. It reads both and duplicates no value.
    What it can NOT prove: that the plant reaches ``reaim_goal_checker``'s
    tolerance. That is a robot test (see nav2.yaml at that checker).
"""

import os
import xml.etree.ElementTree as ET

import pytest

yaml = pytest.importorskip("yaml")

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.abspath(os.path.join(_HERE, ".."))
_CONFIG = os.path.join(_PKG, "config")
NAV2_YAML = os.path.join(_CONFIG, "nav2.yaml")
REAIM_XML = os.path.join(_CONFIG, "reaim_in_place.xml")

#: The only node types the re-aim tree may contain. Anything that can move the
#: robot other than FollowPath (BackUp, DriveOnHeading, Spin, Wait, recovery
#: wrappers that retry) is excluded by construction.
_REAIM_ALLOWED_TAGS = {"root", "BehaviorTree", "Sequence", "ComputePathToPose", "FollowPath"}


def _controller_params():
    with open(NAV2_YAML, encoding="utf-8") as handle:
        return yaml.safe_load(handle)["controller_server"]["ros__parameters"]


def _checkers(params):
    return {name: params[name] for name in params["goal_checker_plugins"]}


def _trees():
    return sorted(
        os.path.join(_CONFIG, name) for name in os.listdir(_CONFIG) if name.endswith(".xml")
    )


def test_every_follow_path_names_a_loaded_checker():
    checkers = _checkers(_controller_params())
    assert len(checkers) >= 1
    for path in _trees():
        nodes = list(ET.parse(path).getroot().iter("FollowPath"))
        assert nodes, f"{os.path.basename(path)} has no FollowPath"
        for node in nodes:
            checker = node.get("goal_checker_id", "")
            if len(checkers) > 1:
                assert checker, (
                    f"{os.path.basename(path)}: FollowPath without goal_checker_id is "
                    "refused by controller_server when more than one checker is loaded"
                )
            if checker:
                # Literal on purpose: a blackboard reference could be switched
                # at runtime (goal_checker_selector topic).
                assert not checker.startswith("{"), (
                    f"{os.path.basename(path)}: goal_checker_id {checker!r} is a "
                    "blackboard reference, not a literal checker name"
                )
                assert checker in checkers, (
                    f"{os.path.basename(path)}: goal_checker_id {checker!r} is not in "
                    f"goal_checker_plugins {sorted(checkers)}"
                )


def test_oscillation_reset_angle_below_every_yaw_tolerance():
    params = _controller_params()
    reset = float(params["FollowPath"]["Oscillation.oscillation_reset_angle"])
    for name, body in _checkers(params).items():
        tolerance = float(body["yaw_goal_tolerance"])
        assert reset < tolerance, (
            f"Oscillation.oscillation_reset_angle {reset} must be strictly below "
            f"{name}.yaw_goal_tolerance {tolerance}"
        )


def test_reaim_tree_is_a_rotation_and_nothing_else():
    assert os.path.isfile(REAIM_XML)
    root = ET.parse(REAIM_XML).getroot()
    tags = {element.tag for element in root.iter()}
    assert tags <= _REAIM_ALLOWED_TAGS, f"unexpected nodes: {sorted(tags - _REAIM_ALLOWED_TAGS)}"
    follow = list(root.iter("FollowPath"))
    assert len(follow) == 1
    assert follow[0].get("goal_checker_id") == "reaim_goal_checker"


def test_reaim_checker_is_tighter_than_the_approach_checker():
    checkers = _checkers(_controller_params())
    assert "reaim_goal_checker" in checkers and "goal_checker" in checkers
    assert float(checkers["reaim_goal_checker"]["yaw_goal_tolerance"]) < float(
        checkers["goal_checker"]["yaw_goal_tolerance"]
    )
    # Same xy window: the re-aim goal is the robot's own position, and the
    # RotationShim only performs its closing rotation inside this window.
    assert float(checkers["reaim_goal_checker"]["xy_goal_tolerance"]) >= float(
        checkers["goal_checker"]["xy_goal_tolerance"]
    )


def test_every_tree_is_installed():
    with open(os.path.join(_PKG, "setup.py"), encoding="utf-8") as handle:
        setup_py = handle.read()
    for path in _trees():
        assert f"config/{os.path.basename(path)}" in setup_py, (
            f"{os.path.basename(path)} is not in setup.py data_files, so it does not "
            "exist on the robot after a build"
        )
