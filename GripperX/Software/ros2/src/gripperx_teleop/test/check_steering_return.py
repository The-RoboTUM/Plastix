#!/usr/bin/env python3
"""Verification of the momentary, self-centring A/D steering.

Pure python, no ROS. Run from the workspace source tree:

    python3 src/gripperx_teleop/test/check_steering_return.py

The property this branch is judged by is that STRAIGHT AHEAD IS THE RESTING
STATE: whatever the operator does with A and D, letting go returns the wheels
to zero in a bounded time and lands on EXACTLY zero, not merely near it. The
last part is not pedantry — /teleop/direct_steer overrides the IK inside
swerve_controller for as long as it is fresh (A2), so an angle that only decays
asymptotically would hold the steering servos off the IK path for ever.
"""

from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))

# Imported directly out of the node module: this is the function the node calls,
# not a copy of it. A second implementation here would pass while the robot
# behaved differently.
sys.modules.setdefault("rclpy", None)
_SOURCE = os.path.join(_HERE, "..", "gripperx_teleop", "keyboard_teleop_node.py")
_namespace: dict = {"math": math}
with open(_SOURCE, encoding="utf-8") as handle:
    _text = handle.read()
_start = _text.index("def update_steering_angle(")
_end = _text.index("class KeyboardTeleopNode(Node):")
exec(compile(_text[_start:_end], _SOURCE, "exec"), _namespace)  # noqa: S102
update_steering_angle = _namespace["update_steering_angle"]

RATE = 0.6            # steer_rate_rad_s
RETURN_RATE = 0.6     # steer_return_rate_rad_s
LIMIT = math.radians(35.0)
DT = 1.0 / 50.0       # publish_rate_hz 50

failures = []


def check(label: str, condition: bool) -> None:
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
    if not condition:
        failures.append(label)


def hold(angle: float, left: bool, right: bool, ticks: int) -> float:
    for _ in range(ticks):
        angle = update_steering_angle(angle, left, right, RATE, RETURN_RATE, LIMIT, DT)
    return angle


def main() -> int:
    print("\n== Steering out ==\n")

    angle = hold(0.0, True, False, 1)
    check(f"one tick of A moves {math.degrees(angle):.2f} deg", abs(angle - RATE * DT) < 1e-12)

    angle = hold(0.0, True, False, 1000)
    check(
        f"A held to saturation stops at the operator limit ({math.degrees(angle):.1f} deg)",
        abs(angle - LIMIT) < 1e-12,
    )
    angle = hold(0.0, False, True, 1000)
    check("D held to saturation stops at the negative limit", abs(angle + LIMIT) < 1e-12)

    check(
        "A and D together cancel exactly — no drift, no special case",
        hold(0.2, True, True, 100) == 0.2,
    )

    print("\n== Self-centring ==\n")

    angle = hold(0.0, True, False, 1000)          # at +35 deg
    released = hold(angle, False, False, 1)
    check(
        "releasing A starts the return immediately, at the return rate",
        abs(released - (LIMIT - RETURN_RATE * DT)) < 1e-12,
    )

    # From the worst case, how long until the wheels are straight again?
    angle = LIMIT
    ticks = 0
    while angle != 0.0 and ticks < 10000:
        angle = update_steering_angle(angle, False, False, RATE, RETURN_RATE, LIMIT, DT)
        ticks += 1
    seconds = ticks * DT
    check(
        f"from the full {math.degrees(LIMIT):.0f} deg it reaches straight in {seconds:.2f} s",
        angle == 0.0 and seconds <= LIMIT / RETURN_RATE + 2 * DT,
    )
    check("…and lands on EXACTLY 0.0, not merely near it", angle == 0.0)

    # The same from the other side, and from a tiny residual — the case that
    # would otherwise keep the direct_steer override alive for ever.
    check(
        "a residual smaller than one step snaps to exactly 0.0",
        update_steering_angle(1e-9, False, False, RATE, RETURN_RATE, LIMIT, DT) == 0.0,
    )
    check(
        "negative angles return to exactly 0.0 too",
        hold(-LIMIT, False, False, 1000) == 0.0,
    )
    check(
        "zero stays zero — no jitter around the resting state",
        update_steering_angle(0.0, False, False, RATE, RETURN_RATE, LIMIT, DT) == 0.0,
    )

    print("\n== Steering while driving ==\n")

    # The drive keys are simply not an input here, and that IS the property:
    # nothing about holding W can change what A/D do to the angle.
    check(
        "the angle update does not depend on the drive state at all",
        "drive" not in update_steering_angle.__code__.co_varnames,
    )
    # Correcting mid-drive: a short tap of A puts in a small angle that then
    # decays, which is what makes a curve steerable rather than latched.
    angle = hold(0.0, True, False, 5)              # a ~100 ms tap
    tapped = math.degrees(angle)
    # ONE TICK MORE THAN THE ARITHMETIC SUGGESTS, and it is not a defect. Five
    # outward steps of 0.6*0.02 accumulate a float residual, so the fifth return
    # step lands on ~3e-18 rad rather than on 0.0 and the snap happens on the
    # sixth. 3e-18 rad is roughly 1e-16 deg: far below the servo's resolution,
    # below the steering feedback's, and it is gone 20 ms later. Chasing it with
    # an epsilon would trade a real number for a magic one.
    angle = hold(angle, False, False, 6)
    check(
        f"a 100 ms tap gives {tapped:.1f} deg and is back to straight ~100 ms later",
        angle == 0.0,
    )

    print()
    if failures:
        print("FAILURES: " + "; ".join(failures))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
