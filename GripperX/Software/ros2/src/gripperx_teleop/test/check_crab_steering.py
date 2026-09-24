#!/usr/bin/env python3
"""Verification of the steerable crab: reachable directions, dead bands, snapping.

Pure python, no ROS. Run from the workspace source tree:

    python3 src/gripperx_teleop/test/check_crab_steering.py

THE FINDING THIS FILE EXISTS TO PIN DOWN. A pure translation puts all four
modules on the SAME angle — that is the only solution, not a simplification —
so the direction of travel psi is limited by the steering windows directly. With
the calibrated per-wheel limits (`gripperx_control/config/steer_servo.yaml`,
the source of truth) and the measured outward sign, most of the circle is NOT
AVAILABLE: the circle breaks into reachable arcs with dead bands between them,
so steering a crab is a stretch of continuous motion and then a jump.

The widths are an IDENTITY, not a threshold, and that is what this file checks:

    dead band  = 180 - outward - inward          (deg)
    the jump   = from 180 - outward, to inward

Both follow from the arcs and hold for any window. Two assertions here used to
be thresholds instead — ">= 30 deg" and "~80 deg" — which were the 100 deg
machine's numbers promoted to requirements. They failed from the 2026-09-18
recalibration onwards, and they failed for an IMPROVEMENT: at 125 deg outward
the dead bands are 20 deg rather than 45, which is better for the operator, not
worse. No requirement asks for a minimum dead-band width; FR-7 is silent on it.
So they are identities now, and this file fails when the model and the window
disagree rather than when the window legitimately moves (finding 27,
`claude/docs/COMMENT_SLIM_FINDINGS.md`).

Part 3 is the regression that matters most: with psi at the plain crab heading
the twist must be bit-identical to what manoeuvre_twist() produced before this
feature existed, or the accepted crab recovery has silently changed.
"""

from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "gripperx_control", "src"))
sys.path.insert(0, os.path.join(_HERE, ".."))

from gripperx_control.steering_limits import (  # noqa: E402
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    SteeringLimits,
)
from gripperx_control.swerve_kinematic_model import FourWIS4WIDKinematicModel  # noqa: E402
from gripperx_teleop.manoeuvre import (  # noqa: E402
    CRAB_LEFT,
    CRAB_RIGHT,
    manoeuvre_pose,
    manoeuvre_twist,
    reachable_translation_arcs,
    snap_psi_into_reach,
)

CRAB_SPEED = 0.25
SPIN_SPEED = 0.55

failures = []


def check(label: str, condition: bool) -> None:
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
    if not condition:
        failures.append(label)


def main() -> int:
    joint_limits = SteeringLimits.from_outward_inward(
        math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
        math.radians(DEFAULT_INWARD_LIMIT_DEG),
        DEFAULT_OUTWARD_SIGN,
    )
    model_limits = joint_limits.in_model_order()
    model = FourWIS4WIDKinematicModel(a=0.203, b=0.16556, wheel_radius=0.070)
    arcs = reachable_translation_arcs(model_limits)

    print("\n== Part 1: which translation directions exist at all ==\n")
    print("  reachable psi arcs (deg):")
    for low, high in arcs:
        print(f"     [{math.degrees(low):+7.1f} , {math.degrees(high):+7.1f}]")
    print("  dead bands (deg):")
    gaps = [
        (math.degrees(a[1]), math.degrees(b[0]))
        for a, b in zip(arcs, arcs[1:])
    ]
    for low, high in gaps:
        print(f"     ({low:+7.1f} , {high:+7.1f})   width {high - low:.1f}")
    print()

    check("there is more than one arc, i.e. the range really is broken up", len(arcs) > 1)
    # Identity, derived from the window in force — see the module docstring for
    # why this is not a threshold. A gap of zero would mean the circle closed up
    # and the whole finding this file pins down had gone away; that is worth
    # failing on, and it is what the arithmetic says rather than a chosen floor.
    expected_gap = 180.0 - DEFAULT_OUTWARD_LIMIT_DEG - DEFAULT_INWARD_LIMIT_DEG
    check(
        f"every dead band is exactly 180 - outward - inward = {expected_gap:.1f} deg",
        all(abs((high - low) - expected_gap) < 1e-6 for low, high in gaps),
    )
    check(
        "the dead bands are real gaps, i.e. the circle has not closed up",
        expected_gap > 0.0 and bool(gaps),
    )
    check(
        "pure crab (+-90 deg) is reachable — the accepted recovery still works",
        any(low <= math.pi / 2 <= high for low, high in arcs)
        and any(low <= -math.pi / 2 <= high for low, high in arcs),
    )
    check(
        "straight ahead (psi = 0) is reachable",
        any(low <= 0.0 <= high for low, high in arcs),
    )

    # The arcs are only trustworthy if they agree with the thing that actually
    # decides: resolve_wheel_targets, through manoeuvre_pose.
    agree = True
    for degrees in range(0, 181):
        psi = math.radians(degrees)
        predicted = any(low - 1e-9 <= psi <= high + 1e-9 for low, high in arcs)
        pose = manoeuvre_pose(
            CRAB_LEFT, model, model_limits, CRAB_SPEED, SPIN_SPEED, None, psi
        )
        if predicted != (pose is not None):
            agree = False
            print(f"     disagreement at psi = {degrees} deg: arc says {predicted}")
    check("the arcs agree with resolve_wheel_targets at every degree 0..180", agree)

    print("\n== Part 2: steering across a dead band ==\n")

    # Holding UP from pure crab: 10 deg of continuous motion, then the jump.
    psi = math.pi / 2
    step = math.radians(0.5)
    trace = []
    for _ in range(400):
        psi_next = psi - step
        psi_next = min(math.pi, max(0.0, psi_next))
        snapped = snap_psi_into_reach(psi_next, arcs, moving_towards_zero=True)
        if snapped != psi_next:
            trace.append((math.degrees(psi), math.degrees(snapped)))
        psi = snapped
        if psi <= 1e-9:
            break
    check("holding UP eventually reaches straight ahead", abs(psi) < 1e-6)
    check("it does so with exactly ONE jump", len(trace) == 1)
    if trace:
        before, after = trace[0]
        print(f"     jump: {before:+.1f} deg -> {after:+.1f} deg  ({before - after:.1f} deg of swing)")
        # Both edges are the window's own numbers, not chosen ones: the crab
        # arc ends where the far wheel runs out of outward travel, and the
        # forward cone ends at the inward limit.
        jump_from = 180.0 - DEFAULT_OUTWARD_LIMIT_DEG
        jump_to = DEFAULT_INWARD_LIMIT_DEG
        check(
            f"the jump starts at the edge of the crab arc "
            f"(180 - outward = {jump_from:.1f} deg)",
            abs(before - jump_from) <= 0.6,
        )
        check(
            f"and lands on the edge of the forward cone (inward = {jump_to:.1f} deg)",
            abs(after - jump_to) <= 0.6,
        )

    # Symmetry: a crab-right must behave the same way mirrored.
    psi = -math.pi / 2
    jumps = 0
    for _ in range(400):
        psi_next = min(0.0, max(-math.pi, psi + step))
        snapped = snap_psi_into_reach(psi_next, arcs, moving_towards_zero=True)
        if snapped != psi_next:
            jumps += 1
        psi = snapped
        if abs(psi) < 1e-9:
            break
    check("crab RIGHT mirrors it exactly: one jump, reaches straight ahead",
          jumps == 1 and abs(psi) < 1e-6)

    check(
        "a psi already inside an arc is never moved",
        snap_psi_into_reach(math.radians(85), arcs, True) == math.radians(85),
    )
    check(
        "snapping never crosses zero into the other side of the robot",
        snap_psi_into_reach(math.radians(60), arcs, True) > 0.0,
    )

    print("\n== Part 3: the plain crab is bit-identical to before ==\n")

    for manoeuvre, sign in ((CRAB_LEFT, +1.0), (CRAB_RIGHT, -1.0)):
        legacy = manoeuvre_twist(manoeuvre, CRAB_SPEED, SPIN_SPEED)
        steered = manoeuvre_twist(
            manoeuvre, CRAB_SPEED, SPIN_SPEED, sign * math.pi / 2.0
        )
        check(
            f"{manoeuvre}: psi omitted gives exactly (0, {sign * CRAB_SPEED:+.2f}, 0)",
            legacy[0] == 0.0 and legacy[1] == sign * CRAB_SPEED and legacy[2] == 0.0,
        )
        check(
            f"{manoeuvre}: psi = {sign * 90:+.0f} deg matches it to 1e-15",
            all(abs(a - b) < 1e-15 for a, b in zip(legacy, steered)),
        )

    # omega must be zero for every psi: a steered crab is still a translation.
    check(
        "omega is exactly 0.0 at every psi — a steered crab is still a translation",
        all(
            manoeuvre_twist(CRAB_LEFT, CRAB_SPEED, SPIN_SPEED, math.radians(d))[2] == 0.0
            for d in range(0, 181, 5)
        ),
    )
    # And the speed magnitude must not depend on the heading.
    speeds = [
        math.hypot(*manoeuvre_twist(CRAB_LEFT, CRAB_SPEED, SPIN_SPEED, math.radians(d))[:2])
        for d in range(0, 181, 5)
    ]
    check(
        "the speed is the same in every direction — psi rotates, it does not scale",
        all(abs(speed - CRAB_SPEED) < 1e-12 for speed in speeds),
    )

    print()
    if failures:
        print("FAILURES: " + "; ".join(failures))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
