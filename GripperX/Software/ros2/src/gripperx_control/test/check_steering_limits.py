#!/usr/bin/env python3
"""Verification of the per-wheel steering limits in the 4WIS kinematics chain.

Pure python — no ROS required. Run from the workspace source tree:

    python3 src/gripperx_control/test/check_steering_limits.py

It prints, for a set of representative body twists, the per-wheel steering
angles the kinematics requests and whether each lies inside that wheel's real,
calibrated window; then it asserts the invariants that must hold.

Reference data (see gripperx_control/config/steer_servo.yaml): 100 deg outward
(measured on the machine 2026-08-13) / 35 deg inward (raised from 30 deg
2026-08-17, user estimate, TO-VERIFY, not a measurement), outward sign
[-1, +1, +1, -1] for FL, FR, BL, BR.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
)

from gripperx_control.steering_limits import (  # noqa: E402
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    LimitStatus,
    SteeringLimits,
    limit_twist_to_steering_range,
    symmetric_limit_for_pattern,
    unconstrained_targets,
)
from gripperx_control.swerve_kinematic_model import (  # noqa: E402
    BodyTwist,
    FourWIS4WIDKinematicModel,
)

# Geometry from gripperx_control/config/swerve_cmd.yaml.
from gripperx_geometry.constants import (  # noqa: E402
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as WHEEL_RADIUS,
)


MODEL = FourWIS4WIDKinematicModel(a=A, b=B, wheel_radius=WHEEL_RADIUS)
JOINT_LIMITS = SteeringLimits.from_outward_inward(
    math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
    math.radians(DEFAULT_INWARD_LIMIT_DEG),
    DEFAULT_OUTWARD_SIGN,
)
LIMITS = JOINT_LIMITS.in_model_order()
STRAIGHT = [0.0] * 4

# The old, symmetric world these files carried before stage B.
OLD_SWERVE_LIMIT_RAD = 1.0472        # swerve_cmd.yaml steering_angle_limit (60 deg)
OLD_TELEOP_LIMIT_RAD = 0.785         # keyboard_teleop_node steer_limit_rad (45 deg)
NEW_TELEOP_LIMIT_RAD = math.radians(35.0)  # raised from 30 deg 2026-08-17

TWISTS = [
    ("pure forward            ", BodyTwist(0.30, 0.0, 0.0)),
    ("pure reverse            ", BodyTwist(-0.30, 0.0, 0.0)),
    ("crab left (pure vy)     ", BodyTwist(0.0, 0.30, 0.0)),
    ("diagonal 45 deg         ", BodyTwist(0.30, 0.30, 0.0)),
    ("gentle corner left      ", BodyTwist(0.30, 0.0, 0.40)),
    ("gentle corner right     ", BodyTwist(0.30, 0.0, -0.40)),
    ("ordinary corner left    ", BodyTwist(0.30, 0.0, 1.00)),
    ("ordinary corner right   ", BodyTwist(0.30, 0.0, -1.00)),
    ("in-place spin CCW       ", BodyTwist(0.0, 0.0, 1.00)),
    ("in-place spin CW        ", BodyTwist(0.0, 0.0, -1.00)),
]


def wheel_angles(twist: BodyTwist, current=STRAIGHT):
    """Angles the IK asks for, ignoring limits (model order FL, BL, BR, FR)."""
    return [
        t.angle for t in unconstrained_targets(MODEL.inverse_kinematics(twist), current)
    ]


def fmt_row(label: str, angles, limits: SteeringLimits) -> str:
    cells = []
    for index, angle in enumerate(angles):
        ok = limits.contains(index, angle)
        cells.append(
            f"{limits.labels[index]} {math.degrees(angle):+7.2f} {'ok  ' if ok else 'FAIL'}"
        )
    return f"{label} | " + " | ".join(cells)


def main() -> int:
    failures = []

    print("Per-wheel steering windows (model order FL, BL, BR, FR):")
    print("  " + LIMITS.describe())
    print()

    print("=== 1. Requested per-wheel angles vs the real windows ===")
    print("(angles in deg, model order FL, BL, BR, FR; 'FAIL' = outside that wheel)")
    print("Naive solution: the module angle nearest the current one, limits ignored")
    print("-- i.e. what the old symmetric chain would have asked for.")
    print()
    for label, twist in TWISTS:
        print(fmt_row(label, wheel_angles(twist), LIMITS))
    print()

    print("=== 2. What the node does with each twist ===")
    print("Limit-aware: of the two equivalent module solutions (angle, +v) and")
    print("(angle+180, -v), the reachable one is picked -- which is why crab comes")
    print("out reachable although the naive +90 above does not fit two wheels.")
    print()
    for label, twist in TWISTS:
        limited = limit_twist_to_steering_range(MODEL, twist, STRAIGHT, LIMITS)
        if limited.status == LimitStatus.OK:
            detail = "unchanged"
        elif limited.status == LimitStatus.OMEGA_REDUCED:
            detail = (
                f"omega {limited.requested_omega:+.3f} -> {limited.twist.omega:+.3f} rad/s"
                f"  (radius {abs(twist.vx / limited.twist.omega):.2f} m"
                f" instead of {abs(twist.vx / twist.omega):.2f} m)"
            )
        else:
            detail = "REJECTED (direction of travel unreachable)"
        print(f"{label} | {limited.status:<14} | {detail}")
        if limited.targets is not None:
            print("    " + fmt_row("  resulting             ", [t.angle for t in limited.targets], LIMITS))
    print()

    # --- invariants ------------------------------------------------------
    print("=== 3. Invariants ===")

    # 3.1 In-place spin must be reachable on all four wheels, untouched.
    for label, omega in (("CCW", 1.0), ("CW", -1.0)):
        spin = BodyTwist(0.0, 0.0, omega)
        angles = wheel_angles(spin)
        expected = math.degrees(math.atan2(A, B))
        limited = limit_twist_to_steering_range(MODEL, spin, STRAIGHT, LIMITS)
        magnitudes_ok = all(abs(abs(math.degrees(x)) - expected) < 0.05 for x in angles)
        inside = all(LIMITS.contains(i, x) for i, x in enumerate(angles))
        untouched = limited.status == LimitStatus.OK
        ok = magnitudes_ok and inside and untouched
        print(
            f"  in-place spin {label}: |angle| = {expected:.2f} deg on all four, "
            f"inside windows: {inside}, passed through unchanged: {untouched} "
            f"-> {'PASS' if ok else 'FAIL'}"
        )
        if not ok:
            failures.append(f"in-place spin {label}")

    # 3.2 The spin pose must be OUTWARD on every wheel (the sanity check that
    #     refutes the URDF-derived sign).
    spin_angles_model = wheel_angles(BodyTwist(0.0, 0.0, 1.0))
    # model order FL, BL, BR, FR -> joint order FL, FR, BL, BR
    spin_joint_angles = [
        spin_angles_model[0],
        spin_angles_model[3],
        spin_angles_model[1],
        spin_angles_model[2],
    ]
    outward_everywhere = all(
        math.copysign(1.0, angle) == sign
        for angle, sign in zip(spin_joint_angles, DEFAULT_OUTWARD_SIGN)
    )
    print(
        f"  spin pose sign pattern {[round(math.degrees(a), 1) for a in spin_joint_angles]} "
        f"(joint order FL, FR, BL, BR) equals steering_outward_sign "
        f"{list(DEFAULT_OUTWARD_SIGN)} -> {'PASS' if outward_everywhere else 'FAIL'}"
    )
    if not outward_everywhere:
        failures.append("spin pose is not outward on all four wheels")

    # 3.3 Under the refuted URDF-derived sign the spin would be impossible.
    refuted = SteeringLimits.from_outward_inward(
        math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
        math.radians(DEFAULT_INWARD_LIMIT_DEG),
        (1, -1, 1, -1),
    ).in_model_order()
    spin_blocked = any(
        not refuted.contains(i, a) for i, a in enumerate(spin_angles_model)
    )
    print(
        "  under the refuted sign [+1,-1,+1,-1] the same spin pose is out of range: "
        f"{spin_blocked} -> {'PASS' if spin_blocked else 'FAIL'}"
    )
    if not spin_blocked:
        failures.append("refuted-sign cross-check did not trigger")

    # 3.4 Every twist the limiter accepts must be inside every window.
    all_inside = True
    for _label, twist in TWISTS:
        limited = limit_twist_to_steering_range(MODEL, twist, STRAIGHT, LIMITS)
        if limited.targets is None:
            continue
        for index, target in enumerate(limited.targets):
            if not LIMITS.contains(index, target.angle):
                all_inside = False
    print(f"  no accepted twist leaves any window -> {'PASS' if all_inside else 'FAIL'}")
    if not all_inside:
        failures.append("limiter emitted an out-of-window angle")

    # 3.5 Pure forward/reverse must never be touched.
    for label, twist in (("forward", BodyTwist(0.3, 0.0, 0.0)), ("reverse", BodyTwist(-0.3, 0.0, 0.0))):
        limited = limit_twist_to_steering_range(MODEL, twist, STRAIGHT, LIMITS)
        ok = limited.status == LimitStatus.OK
        print(f"  pure {label} unchanged -> {'PASS' if ok else 'FAIL'}")
        if not ok:
            failures.append(f"pure {label} was limited")

    # 3.6 Keyboard teleop pattern bound.
    pattern_bound = symmetric_limit_for_pattern((1.0, 1.0, -1.0, -1.0), JOINT_LIMITS)
    ok = abs(math.degrees(pattern_bound) - DEFAULT_INWARD_LIMIT_DEG) < 1e-9
    print(
        f"  keyboard teleop pattern [+1,+1,-1,-1] max magnitude = "
        f"{math.degrees(pattern_bound):.2f} deg (= inward limit) -> {'PASS' if ok else 'FAIL'}"
    )
    if not ok:
        failures.append("teleop pattern bound is not the inward limit")
    print()

    # --- before/after of a silently clamped case -------------------------
    print("=== 4. Before/after: a case that used to be silently clamped ===")
    corner = BodyTwist(0.30, 0.0, 1.00)
    before = wheel_angles(corner)
    print("  request: vx=0.30 m/s, omega=1.00 rad/s (left turn, radius 0.30 m)")
    print("  BEFORE (symmetric steering_angle_limit=60 deg in swerve_cmd.yaml):")
    for index, angle in enumerate(before):
        planned = max(-OLD_SWERVE_LIMIT_RAD, min(OLD_SWERVE_LIMIT_RAD, angle))
        served = LIMITS.clamp(index, planned)
        note = (
            f"  <-- steer_servo_node silently clamps to {math.degrees(served):+.2f}"
            if abs(served - planned) > 1e-9
            else ""
        )
        print(
            f"    {LIMITS.labels[index]}: planned {math.degrees(planned):+7.2f} deg"
            f"{note}"
        )
    print("    => two wheels end up where the kinematics did not put them; the four")
    print("       wheels share no instantaneous centre any more (scrub), and nothing")
    print("       upstream is told.")
    limited = limit_twist_to_steering_range(MODEL, corner, STRAIGHT, LIMITS)
    print(f"  AFTER  (per-wheel windows, status={limited.status}):")
    print(
        f"    omega reduced {limited.requested_omega:.3f} -> {limited.twist.omega:.3f} rad/s"
        f" (turn radius {abs(corner.vx / limited.twist.omega):.2f} m instead of"
        f" {abs(corner.vx / corner.omega):.2f} m), logged as a warning"
    )
    for index, target in enumerate(limited.targets):
        print(
            f"    {LIMITS.labels[index]}: commanded {math.degrees(target.angle):+7.2f} deg"
            f"  (window [{math.degrees(LIMITS.lower[index]):+.0f},"
            f" {math.degrees(LIMITS.upper[index]):+.0f}])"
        )
    clamped_before = any(
        abs(
            LIMITS.clamp(i, max(-OLD_SWERVE_LIMIT_RAD, min(OLD_SWERVE_LIMIT_RAD, a)))
            - max(-OLD_SWERVE_LIMIT_RAD, min(OLD_SWERVE_LIMIT_RAD, a))
        )
        > 1e-9
        for i, a in enumerate(before)
    )
    clamped_after = any(
        not LIMITS.contains(i, t.angle) for i, t in enumerate(limited.targets)
    )
    ok = clamped_before and not clamped_after
    print(f"  silently clamped before: {clamped_before}, after: {clamped_after} -> {'PASS' if ok else 'FAIL'}")
    if not ok:
        failures.append("before/after clamping demonstration")
    print()

    print("=== 5. Teleop: the operator can no longer request a clamped pose ===")
    for label, limit in (
        ("BEFORE (0.785 rad = 45 deg)", OLD_TELEOP_LIMIT_RAD),
        ("AFTER  (radians(35) rad)   ", NEW_TELEOP_LIMIT_RAD),
    ):
        pattern = (1.0, 1.0, -1.0, -1.0)
        commanded = [limit * f for f in pattern]
        served = [JOINT_LIMITS.clamp(i, a) for i, a in enumerate(commanded)]
        worst = max(abs(c - s) for c, s in zip(commanded, served))
        print(
            f"  {label}: full left  {[round(math.degrees(a), 1) for a in commanded]} -> "
            f"servo {[round(math.degrees(a), 1) for a in served]} "
            f"(max silent clamp {math.degrees(worst):.1f} deg)"
        )
    after_worst = max(
        abs(NEW_TELEOP_LIMIT_RAD * f - JOINT_LIMITS.clamp(i, NEW_TELEOP_LIMIT_RAD * f))
        for i, f in enumerate((1.0, 1.0, -1.0, -1.0))
    )
    ok = after_worst < 1e-9
    print(f"  no silent clamp left in teleop -> {'PASS' if ok else 'FAIL'}")
    if not ok:
        failures.append("teleop still requests clamped angles")
    print()

    if failures:
        print("FAILURES: " + "; ".join(failures))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
