#!/usr/bin/env python3
"""Prove that crab_walk's alignment grace still outlasts swerve_controller's alignment timeout.

Pure python -- no ROS required. Run from anywhere:

    python3 Software/ros2/src/gripperx_control/test/check_alignment_timing.py

`colcon test --packages-select gripperx_control` runs it via test_check_scripts.py.

WHAT THE INVARIANT IS
    swerve_controller's alignment gate withholds the drive while the four
    steering modules swing into the crab pose, and releases it either when all
    four are inside alignment_exit_tolerance_rad or when alignment_timeout_sec
    elapses, whichever comes first. crab_walk gives the manoeuvre
    alignment_grace_sec to start moving before it gives up with "never started
    moving". A grace that is not longer than the timeout therefore aborts the
    recovery while the controller is still about to let it run:

        alignment_grace_sec  >  alignment_timeout_sec        (strictly)

WHY THIS FILE EXISTS
    The two values live in different files, different packages and different
    nodes, and nav2.yaml used to say they "CANNOT be checked against each other
    automatically -- that is why the number is written out here". Writing the
    number out by hand is what let it rot. cf09052 raised the timeout
    1.5 -> 5.0 on 2026-08-25 and 1b6fbc1 enabled the gate the same day; the
    grace stayed at 2.0 for 27 days, during which every Nav2 crab recovery
    failed by construction, and nothing said so.

    They can be checked automatically. It only needs something that reads both
    files instead of trusting a comment to stay true.

WHAT MAKES THIS NOT A TAUTOLOGY
    This file contains NO copy of either number. Both operands are parsed off
    disk, from the same YAML the launch files feed to the nodes, on every run.
    A check that carried its own 5.5 and 5.0 would agree with itself for ever
    while the robot ran something else.

    Three further rules keep it honest:

      * A MISSING key is a FAILURE, never a skip. Deleting a key must not be a
        way to make the check pass -- an absent alignment_grace_sec means the
        plugin falls back to 0.0, i.e. no grace at all, which is the regression
        this exists to catch.
      * The SIM OVERLAY is resolved the way the spawner resolves it.
        swerve_controller.sim.yaml is layered on top of ros2_controllers.yaml
        (spawn_robot.launch.py passes both as --param-file, base first), so a
        sim-only override of the timeout would change the effective value on
        the twin and not on the robot. Both platforms are checked, and the
        assumed layering order is itself asserted against the launch file.
      * The check PROVES IT CAN FAIL on every run, before it reports anything:
        the self-test copies the real config tree, mis-sets the grace to just
        below the timeout, and runs this very same locate-parse-compare path
        against the copy, asserting it comes back as a violation. Not a
        re-implementation of the comparison -- the same code, on deliberately
        broken input.

THE GATE BEING OFF IS NOT AN EXCUSE
    alignment_gate_enabled is reported as context but the invariant is checked
    unconditionally. If the gate is off the timeout is inert and a short grace
    is harmless -- but then the invariant would go unchecked exactly across the
    window in which someone switches the gate back on, which is how this broke
    the first time.

RUNNING IT AGAINST SOMETHING OTHER THAN THIS CHECKOUT
    --root <path> points the check at any ROS 2 `src` tree: an install share
    tree, or a checkout on the robot. That matters here more than usual,
    because alignment_timeout_sec is read only at configure(): `ros2 param set`
    reports success and changes nothing until the controller restarts, so a
    runtime read of the parameter is not evidence of what the gate is using.
    The file on the machine is.
"""

from __future__ import annotations

import argparse
import re
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

# ---------------------------------------------------------------------------
# Where the two values live. Paths are relative to the ROS 2 `src` root.
# These are LOCATORS, not values: nothing here says what the numbers are.
# ---------------------------------------------------------------------------

NAV2_YAML = "gripperx_planning/config/nav2.yaml"
CONTROLLERS_YAML = "gripperx_control/config/ros2_controllers.yaml"
SIM_OVERLAY_YAML = "gripperx_control/config/swerve_controller.sim.yaml"
SPAWNER_LAUNCH = "gripperx_gazebo/launch/spawn_robot.launch.py"

# The anchor that says a directory really is the ROS 2 src root, rather than
# something that merely has the right name.
SRC_ROOT_ANCHOR = "gripperx_control/config/swerve_cmd.yaml"

CONTROLLER_NODE = "swerve_controller"
BEHAVIOR_SERVER_NODE = "behavior_server"
GRACE_KEY = "alignment_grace_sec"
TIMEOUT_KEY = "alignment_timeout_sec"
GATE_KEY = "alignment_gate_enabled"


# ---------------------------------------------------------------------------
# Locating and reading
# ---------------------------------------------------------------------------


def default_src_root() -> Path:
    """This file lives at <src>/gripperx_control/test/, so <src> is two up."""
    return Path(__file__).resolve().parents[2]


@dataclass
class Problem:
    """Something that makes the answer unavailable or wrong. Never a warning."""

    what: str

    def __str__(self) -> str:
        return self.what


def load_yaml(path: Path, problems: list[Problem]) -> dict[str, Any] | None:
    if not path.is_file():
        problems.append(Problem(f"missing file: {path}"))
        return None
    try:
        with path.open() as handle:
            loaded = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        problems.append(Problem(f"unparsable YAML in {path}: {exc}"))
        return None
    if not isinstance(loaded, dict):
        problems.append(Problem(f"{path} did not parse to a mapping"))
        return None
    return loaded


def node_params(doc: dict[str, Any], node: str) -> dict[str, Any] | None:
    block = doc.get(node)
    if not isinstance(block, dict):
        return None
    params = block.get("ros__parameters")
    return params if isinstance(params, dict) else None


def as_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def find_graces(src_root: Path, problems: list[Problem]) -> dict[str, float]:
    """Every behavior under behavior_server that declares an alignment grace.

    The behavior is NOT hardcoded to `crab_walk`. A second behavior that gains a
    grace is covered automatically, and a rename shows up as "no behavior
    declares it" rather than as a silently skipped check.
    """
    path = src_root / NAV2_YAML
    doc = load_yaml(path, problems)
    if doc is None:
        return {}

    params = node_params(doc, BEHAVIOR_SERVER_NODE)
    if params is None:
        problems.append(
            Problem(f"{path}: no {BEHAVIOR_SERVER_NODE}:/ros__parameters: block")
        )
        return {}

    graces: dict[str, float] = {}
    for behavior, block in params.items():
        if not isinstance(block, dict) or GRACE_KEY not in block:
            continue
        value = as_number(block[GRACE_KEY])
        if value is None:
            problems.append(
                Problem(
                    f"{path}: {behavior}.{GRACE_KEY} is "
                    f"{block[GRACE_KEY]!r}, not a number"
                )
            )
            continue
        graces[behavior] = value

    if not graces:
        problems.append(
            Problem(
                f"{path}: no behavior under {BEHAVIOR_SERVER_NODE} declares "
                f"{GRACE_KEY}. Either it was deleted -- in which case the plugin "
                f"falls back to 0.0 and there is no grace at all -- or the "
                f"behavior was renamed and this check needs to know. Not a skip."
            )
        )
    return graces


def find_timeouts(src_root: Path, problems: list[Problem]) -> dict[str, float]:
    """The effective alignment timeout per platform: real, and sim after overlay.

    spawn_robot.launch.py layers swerve_controller.sim.yaml on top of
    ros2_controllers.yaml, so a key present in the overlay wins on the twin. The
    overlay carries no timeout today; if it ever does, the twin and the robot
    stop agreeing and each has to satisfy the invariant on its own.
    """
    base_path = src_root / CONTROLLERS_YAML
    base = load_yaml(base_path, problems)
    if base is None:
        return {}

    params = node_params(base, CONTROLLER_NODE)
    if params is None:
        problems.append(
            Problem(f"{base_path}: no {CONTROLLER_NODE}:/ros__parameters: block")
        )
        return {}

    if TIMEOUT_KEY not in params:
        problems.append(
            Problem(
                f"{base_path}: {CONTROLLER_NODE}.{TIMEOUT_KEY} is absent. The "
                f"controller would fall back to its own compiled default, which "
                f"is not visible from here, so the invariant cannot be decided. "
                f"Declare it. Not a skip."
            )
        )
        return {}

    base_value = as_number(params[TIMEOUT_KEY])
    if base_value is None:
        problems.append(
            Problem(
                f"{base_path}: {TIMEOUT_KEY} is {params[TIMEOUT_KEY]!r}, "
                f"not a number"
            )
        )
        return {}

    timeouts = {"real": base_value}

    overlay_path = src_root / SIM_OVERLAY_YAML
    overlay = load_yaml(overlay_path, problems)
    if overlay is not None:
        overlay_params = node_params(overlay, CONTROLLER_NODE) or {}
        if TIMEOUT_KEY in overlay_params:
            overlay_value = as_number(overlay_params[TIMEOUT_KEY])
            if overlay_value is None:
                problems.append(
                    Problem(
                        f"{overlay_path}: {TIMEOUT_KEY} is "
                        f"{overlay_params[TIMEOUT_KEY]!r}, not a number"
                    )
                )
            else:
                timeouts["sim"] = overlay_value
        else:
            timeouts["sim"] = base_value

    return timeouts


def gate_state(src_root: Path) -> Any:
    """Context for the report. Deliberately does NOT gate the assertion."""
    problems: list[Problem] = []
    doc = load_yaml(src_root / CONTROLLERS_YAML, problems)
    if doc is None:
        return "unknown"
    params = node_params(doc, CONTROLLER_NODE) or {}
    return params.get(GATE_KEY, "unset")


def check_overlay_order(src_root: Path, problems: list[Problem]) -> None:
    """The sim numbers above assume base-then-overlay. Assert the spawner agrees.

    If someone reverses the two --param-file arguments, the overlay stops
    winning and the effective sim value computed here is wrong -- the check
    would then be confidently checking a number nothing loads.
    """
    path = src_root / SPAWNER_LAUNCH
    base_name = Path(CONTROLLERS_YAML).name
    overlay_name = Path(SIM_OVERLAY_YAML).name

    if not path.is_file():
        problems.append(
            Problem(
                f"missing {path}: the sim effective-value calculation assumes "
                f"this file layers {overlay_name} after {base_name}, and that "
                f"assumption can no longer be verified"
            )
        )
        return

    text = path.read_text()
    base_at = text.rfind(base_name)
    overlay_at = text.rfind(overlay_name)
    if base_at < 0 or overlay_at < 0:
        problems.append(
            Problem(
                f"{path}: expected it to name both {base_name} and "
                f"{overlay_name}; the sim overlay wiring changed and the "
                f"effective-value calculation here needs rechecking"
            )
        )
        return
    if overlay_at < base_at:
        problems.append(
            Problem(
                f"{path}: {overlay_name} is named BEFORE {base_name}. This "
                f"check assumes the overlay is layered last and therefore wins; "
                f"if the order really did reverse, the sim effective timeout "
                f"computed here is wrong."
            )
        )


# ---------------------------------------------------------------------------
# The comparison
# ---------------------------------------------------------------------------


@dataclass
class Verdict:
    ok: bool
    lines: list[str]


def evaluate(src_root: Path) -> Verdict:
    """Locate, parse, compare.

    The single path used for both the real run and the self-test -- there is no
    second implementation for it to disagree with.
    """
    problems: list[Problem] = []
    lines: list[str] = []

    if not (src_root / SRC_ROOT_ANCHOR).is_file():
        return Verdict(
            False,
            [
                f"  {src_root} does not look like the ROS 2 src root "
                f"(expected {SRC_ROOT_ANCHOR} under it)."
            ],
        )

    graces = find_graces(src_root, problems)
    timeouts = find_timeouts(src_root, problems)
    check_overlay_order(src_root, problems)

    if problems:
        lines.append("  structural problems -- the invariant cannot be decided:")
        lines.extend(f"    - {p}" for p in problems)
        return Verdict(False, lines)

    lines.append(
        f"  {GATE_KEY}: {gate_state(src_root)}   (context only -- the invariant "
        f"is checked either way)"
    )

    ok = True
    for behavior, grace in sorted(graces.items()):
        for platform, timeout in sorted(timeouts.items()):
            margin = grace - timeout
            good = margin > 0.0
            ok = ok and good
            lines.append(
                f"  {behavior}.{GRACE_KEY} {grace:.2f} s  vs  "
                f"{CONTROLLER_NODE}.{TIMEOUT_KEY} {timeout:.2f} s ({platform})"
                f"  -> margin {margin:+.2f} s  {'PASS' if good else 'FAIL'}"
            )
            if not good:
                lines.append(
                    f"      {behavior} gives up after {grace:.2f} s while the "
                    f"gate may hold the drive for {timeout:.2f} s. Every crab "
                    f'recovery aborts with "never started moving" '
                    f"{-margin:.2f} s before the controller would have released "
                    f"it. Raise {GRACE_KEY} in {NAV2_YAML} above {timeout:.2f} s, "
                    f"or lower {TIMEOUT_KEY} in {CONTROLLERS_YAML}."
                )
    return Verdict(ok, lines)


# ---------------------------------------------------------------------------
# Proving the check can fail
# ---------------------------------------------------------------------------


def _mutated_copy(src_root: Path, dest: Path, new_grace: float) -> None:
    """Copy the files the check reads, with alignment_grace_sec mis-set.

    A raw-text substitution, so the copy stays a real config file -- comments,
    layout and every other key intact -- and the check parses it by exactly the
    route it parses the originals.
    """
    for rel in (
        NAV2_YAML,
        CONTROLLERS_YAML,
        SIM_OVERLAY_YAML,
        SPAWNER_LAUNCH,
        SRC_ROOT_ANCHOR,
    ):
        source = src_root / rel
        if not source.is_file():
            continue
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)

    nav2 = dest / NAV2_YAML
    text = nav2.read_text()
    patched, count = re.subn(
        rf"^(\s*){GRACE_KEY}:[ \t]*[-+0-9.eE]+[ \t]*$",
        rf"\g<1>{GRACE_KEY}: {new_grace}",
        text,
        flags=re.MULTILINE,
    )
    if count == 0:
        raise RuntimeError(
            f"self-test could not find a `{GRACE_KEY}:` assignment to mis-set in "
            f"{nav2}. The check's own fixture is broken; fix that before "
            f"trusting a PASS from it."
        )

    nav2.write_text(patched)


def self_test(src_root: Path) -> bool:
    """Run the real evaluate() against a deliberately mis-set copy of the tree.

    This is the answer to "what would have to change for this to fail". If the
    check cannot be made to fail on demand its PASS means nothing, so this runs
    BEFORE the real verdict is reported and a failure here is fatal.
    """
    problems: list[Problem] = []
    timeouts = find_timeouts(src_root, problems)
    if problems or not timeouts:
        print("  self-test: cannot read the timeout to mis-set against:")
        for problem in problems:
            print(f"    - {problem}")
        return False
    worst_timeout = max(timeouts.values())

    cases = [
        ("grace 0.10 s below the timeout", worst_timeout - 0.10),
        ("grace exactly equal to the timeout", worst_timeout),
        ("grace 0.0 -- what a deleted key falls back to", 0.0),
    ]

    all_ok = True
    for label, grace in cases:
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "src"
            _mutated_copy(src_root, dest, grace)
            verdict = evaluate(dest)
        detected = not verdict.ok
        print(
            f"  self-test: {label} -> "
            f"{'detected' if detected else 'NOT DETECTED'}"
        )
        if not detected:
            all_ok = False
            for line in verdict.lines:
                print(f"    {line}")

    if not all_ok:
        print(
            "  self-test FAILED: the check did not fail on a mis-set pair, so a "
            "PASS below would prove nothing."
        )
    return all_ok


# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Check that alignment_grace_sec > alignment_timeout_sec."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="ROS 2 `src` tree to check (default: the one this file lives in). "
        "Point it at an install share tree or a checkout on the robot.",
    )
    parser.add_argument(
        "--skip-self-test",
        action="store_true",
        help="do not first prove the check can fail (not recommended)",
    )
    args = parser.parse_args(argv)

    src_root = (args.root or default_src_root()).resolve()
    print(f"=== alignment timing invariant: {GRACE_KEY} > {TIMEOUT_KEY} ===")
    print(f"  src root: {src_root}")

    if not args.skip_self_test and not self_test(src_root):
        return 1

    verdict = evaluate(src_root)
    for line in verdict.lines:
        print(line)

    if not verdict.ok:
        print("FAILED: the alignment timing invariant does not hold.")
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
