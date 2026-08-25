#!/usr/bin/env python3
"""Verification of the crab/spin manoeuvres and the mode-transition guard.

Mirrors gripperx_control/test/check_steering_limits.py: pure python, no ROS
required. Run from the workspace source tree:

    python3 src/gripperx_teleop/test/check_manoeuvres.py

Part 1 prints, for the twists the ARROW KEYS actually publish (at the speeds
configured in keyboard_teleop_node), the per-wheel steering angles the
kinematics resolves and whether each lies inside that wheel's real, calibrated
window. Part 2 exercises the transition guard, including the case this feature
exists for: a mode switch requested while drive is active must not produce a
drive command before the steering pose is reached.

Reference data (see gripperx_control/config/steer_servo.yaml): 100 deg outward
(measured on the machine 2026-08-13) / 35 deg inward (raised from 30 deg
2026-08-17, user estimate, TO-VERIFY, not a measurement), outward sign
[-1, +1, +1, -1] for FL, FR, BL, BR.
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
    LimitStatus,
    SteeringLimits,
    limit_twist_to_steering_range,
    unconstrained_targets,
)
from gripperx_control.swerve_kinematic_model import (  # noqa: E402
    BodyTwist,
    FourWIS4WIDKinematicModel,
)

from gripperx_teleop.manoeuvre import (  # noqa: E402
    CORNERING,
    CRAB_LEFT,
    CRAB_RIGHT,
    SPIN_CCW,
    SPIN_CW,
    GuardState,
    TransitionGuard,
    manoeuvre_pose,
    manoeuvre_twist,
)

# Geometry from gripperx_control/config/swerve_cmd.yaml.
from gripperx_geometry.constants import (  # noqa: E402
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as WHEEL_RADIUS,
)


# Defaults declared in keyboard_teleop_node.
CRAB_SPEED = 0.25
SPIN_SPEED = 0.60
POSE_SCALE = 0.02
LINEAR_VEL = 0.5

MODEL = FourWIS4WIDKinematicModel(a=A, b=B, wheel_radius=WHEEL_RADIUS)
JOINT_LIMITS = SteeringLimits.from_outward_inward(
    math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
    math.radians(DEFAULT_INWARD_LIMIT_DEG),
    DEFAULT_OUTWARD_SIGN,
)
LIMITS = JOINT_LIMITS.in_model_order()
STRAIGHT = [0.0] * 4

ARROW_MANOEUVRES = (
    ("arrow left   crab left ", CRAB_LEFT),
    ("arrow right  crab right", CRAB_RIGHT),
    ("arrow up     spin CW   ", SPIN_CW),
    ("arrow down   spin CCW  ", SPIN_CCW),
)


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
    print(
        f"Teleop speeds: crab {CRAB_SPEED:.2f} m/s, spin {SPIN_SPEED:.2f} rad/s, "
        f"pose-hold scale {POSE_SCALE:.3f}"
    )
    print()

    # === 1. The twists the arrow keys generate ==========================
    print("=== 1. Twist table: what each arrow key publishes ===")
    print("(angles in deg, model order FL, BL, BR, FR)")
    print()
    for label, manoeuvre in ARROW_MANOEUVRES:
        vx, vy, omega = manoeuvre_twist(manoeuvre, CRAB_SPEED, SPIN_SPEED)
        twist = BodyTwist(vx, vy, omega)
        limited = limit_twist_to_steering_range(MODEL, twist, STRAIGHT, LIMITS)
        naive = [t.angle for t in unconstrained_targets(MODEL.inverse_kinematics(twist), STRAIGHT)]
        print(
            f"{label} | twist vx={vx:+.3f} vy={vy:+.3f} omega={omega:+.3f} "
            f"| limiter: {limited.status}"
        )
        print("    " + fmt_row("naive (limits ignored)  ", naive, LIMITS))
        if limited.targets is None:
            print("    REJECTED — no reachable pose")
            failures.append(f"{manoeuvre} rejected by the limiter")
            continue
        print("    " + fmt_row("commanded (limit-aware) ", [t.angle for t in limited.targets], LIMITS))
        speeds = [t.speed for t in limited.targets]
        print(
            "    wheel linear speeds m/s: "
            + " ".join(f"{LIMITS.labels[i]} {s:+.3f}" for i, s in enumerate(speeds))
            + f"  (max {max(abs(s) for s in speeds) / WHEEL_RADIUS:.2f} rad/s, cap 12.0)"
        )
        print()

    # === 2. Invariants on those twists ==================================
    print("=== 2. Invariants ===")

    # 2.1 Every arrow manoeuvre must be accepted and inside every window.
    for label, manoeuvre in ARROW_MANOEUVRES:
        vx, vy, omega = manoeuvre_twist(manoeuvre, CRAB_SPEED, SPIN_SPEED)
        limited = limit_twist_to_steering_range(
            MODEL, BodyTwist(vx, vy, omega), STRAIGHT, LIMITS
        )
        accepted = limited.status == LimitStatus.OK
        inside = limited.targets is not None and all(
            LIMITS.contains(i, t.angle) for i, t in enumerate(limited.targets)
        )
        ok = accepted and inside
        print(
            f"  {label}: accepted unchanged {accepted}, inside all windows {inside} "
            f"-> {'PASS' if ok else 'FAIL'}"
        )
        if not ok:
            failures.append(f"{manoeuvre} not cleanly reachable")

    # 2.2 Crab must use the +-180 module flip on exactly the two wheels whose
    #     window excludes the naive +-90 (FL and BR under the measured sign).
    crab = BodyTwist(*manoeuvre_twist(CRAB_LEFT, CRAB_SPEED, SPIN_SPEED))
    crab_targets = limit_twist_to_steering_range(MODEL, crab, STRAIGHT, LIMITS).targets
    flipped = [
        LIMITS.labels[i]
        for i, t in enumerate(crab_targets)
        if t.speed < 0.0
    ]
    ok = sorted(flipped) == ["BR", "FL"]
    print(
        f"  crab left flips (drives backwards at angle+180) on {sorted(flipped)}, "
        f"expected ['BR', 'FL'] -> {'PASS' if ok else 'FAIL'}"
    )
    if not ok:
        failures.append("crab flip pattern unexpected")

    # 2.3 Spin magnitude must be atan2(a, b) on all four, outward everywhere.
    expected_spin = math.degrees(math.atan2(A, B))
    spin = BodyTwist(*manoeuvre_twist(SPIN_CW, CRAB_SPEED, SPIN_SPEED))
    spin_targets = limit_twist_to_steering_range(MODEL, spin, STRAIGHT, LIMITS).targets
    ok = all(
        abs(abs(math.degrees(t.angle)) - expected_spin) < 0.05 for t in spin_targets
    )
    print(
        f"  spin CW magnitude {expected_spin:.2f} deg on all four -> "
        f"{'PASS' if ok else 'FAIL'}"
    )
    if not ok:
        failures.append("spin pose magnitude wrong")

    # 2.4 The pose-hold scale must not move a single wheel angle. This is what
    #     lets the guard command the target pose at ~2 % traction: the IK's
    #     delta_i = atan2(vy_i, vx_i) is invariant under positive scaling.
    worst = 0.0
    for _label, manoeuvre in ARROW_MANOEUVRES:
        vx, vy, omega = manoeuvre_twist(manoeuvre, CRAB_SPEED, SPIN_SPEED)
        full = limit_twist_to_steering_range(
            MODEL, BodyTwist(vx, vy, omega), STRAIGHT, LIMITS
        ).targets
        held = limit_twist_to_steering_range(
            MODEL,
            BodyTwist(vx * POSE_SCALE, vy * POSE_SCALE, omega * POSE_SCALE),
            STRAIGHT,
            LIMITS,
        ).targets
        worst = max(worst, max(abs(f.angle - h.angle) for f, h in zip(full, held)))
    ok = worst < 1e-12
    print(
        f"  pose-hold scale {POSE_SCALE} leaves every wheel angle identical "
        f"(max deviation {math.degrees(worst):.2e} deg) -> {'PASS' if ok else 'FAIL'}"
    )
    if not ok:
        failures.append("pose-hold scale changes the commanded pose")

    # 2.5 Traction actually withheld: wheel speed during alignment.
    crab_speeds = [abs(t.speed) for t in crab_targets]
    held_mps = max(crab_speeds) * POSE_SCALE
    print(
        f"  crab wheel speed while aligning: {held_mps * 1000:.1f} mm/s "
        f"(vs {max(crab_speeds) * 1000:.0f} mm/s armed) -> "
        f"{'PASS' if held_mps < 0.01 else 'FAIL'}"
    )
    if held_mps >= 0.01:
        failures.append("pose-hold traction too high")
    print()

    # === 3. Pose predicted by the teleop == pose the node commands =======
    print("=== 3. The guard waits for the pose swerve_cmd_node will command ===")
    all_match = True
    for label, manoeuvre in ARROW_MANOEUVRES:
        predicted = manoeuvre_pose(
            manoeuvre, MODEL, LIMITS, CRAB_SPEED, SPIN_SPEED, STRAIGHT
        )
        vx, vy, omega = manoeuvre_twist(manoeuvre, CRAB_SPEED, SPIN_SPEED)
        commanded_model = limit_twist_to_steering_range(
            MODEL, BodyTwist(vx, vy, omega), STRAIGHT, LIMITS
        ).targets
        # predicted is in JOINT order FL, FR, BL, BR; commanded in MODEL order.
        commanded_joint = [
            commanded_model[0].angle,
            commanded_model[3].angle,
            commanded_model[1].angle,
            commanded_model[2].angle,
        ]
        match = all(abs(p - c) < 1e-12 for p, c in zip(predicted, commanded_joint))
        all_match = all_match and match
        print(
            f"  {label}: guard target (joint order FL,FR,BL,BR) "
            f"{[round(math.degrees(v), 2) for v in predicted]} -> "
            f"{'PASS' if match else 'FAIL'}"
        )
    if not all_match:
        failures.append("guard target differs from the commanded pose")
    print()

    # === 4. Transition guard ============================================
    print("=== 4. Transition guard ===")
    RELEASE, ALIGN_TIMEOUT, TOL = 0.7, 1.5, math.radians(6.0)

    def new_guard():
        return TransitionGuard(RELEASE, ALIGN_TIMEOUT, TOL)

    def check(name, condition):
        print(f"  {name} -> {'PASS' if condition else 'FAIL'}")
        if not condition:
            failures.append(name)

    # 4.1 No regression: an untouched guard never gates plain W/S/A/D.
    guard = new_guard()
    guard.request(CORNERING, 0.0)
    guard.update(0.0, [0.0] * 4, None)
    check(
        "no arrow ever pressed: guard stays armed with no steer feedback at all",
        guard.state == GuardState.ARMED and guard.drive_allowed,
    )

    # 4.2 THE case: drive active, mode switch requested mid-drive.
    #     No traction may be released until the pose is measurably reached.
    guard = new_guard()
    crab_target = manoeuvre_pose(CRAB_LEFT, MODEL, LIMITS, CRAB_SPEED, SPIN_SPEED, STRAIGHT)
    timeline = []
    t = 0.0
    guard.request(CRAB_LEFT, t)                      # arrow pressed while driving
    for step in range(60):                           # 3.0 s at 20 Hz
        t = step * 0.05
        # Wheels still where cornering left them for the first 1.5 s, then a
        # servo model would have them arrive; here they arrive at t = 1.5 s.
        measured = [0.0] * 4 if t < 1.5 else list(crab_target)
        guard.update(t, crab_target, measured)
        timeline.append((round(t, 2), guard.state, guard.drive_allowed))

    released_at = next(t for t, s, _ in timeline if s != GuardState.RELEASING)
    armed_at = next(t for t, _, d in timeline if d)
    drive_before_arrival = [t for t, _, d in timeline if d and t < 1.5]
    check(
        f"RELEASING held for {released_at:.2f} s (>= direct_timeout_sec 0.5 s "
        "so the direct-steer override lapses first)",
        released_at >= RELEASE - 1e-9,
    )
    check(
        f"no drive command released before the wheels arrive "
        f"(armed at {armed_at:.2f} s, wheels arrived at 1.50 s)",
        not drive_before_arrival,
    )
    check(
        "armed on measured angles, not on the timeout",
        not guard.armed_without_feedback,
    )

    # 4.3 Wheels that never arrive: never armed on feedback, only on timeout.
    guard = new_guard()
    guard.request(CRAB_LEFT, 0.0)
    guard.update(RELEASE + 0.01, crab_target, [0.0] * 4)
    stuck_armed_early = guard.drive_allowed
    guard.update(RELEASE + ALIGN_TIMEOUT - 0.01, crab_target, [0.0] * 4)
    still_not_armed = not guard.drive_allowed
    guard.update(RELEASE + ALIGN_TIMEOUT + 0.01, crab_target, [0.0] * 4)
    check(
        "wheels stuck at 0 deg: withheld through the whole align window, "
        "then armed on the timeout fallback and flagged",
        (not stuck_armed_early) and still_not_armed
        and guard.drive_allowed and guard.armed_without_feedback,
    )

    # 4.4 No steer feedback at all (laptop cannot see /hw/steer_states):
    #     falls back to the timeout, flagged so the operator is told.
    #     Note the align window only starts once RELEASING is over.
    guard = new_guard()
    spin_target = manoeuvre_pose(SPIN_CW, MODEL, LIMITS, CRAB_SPEED, SPIN_SPEED, STRAIGHT)
    guard.request(SPIN_CW, 0.0)
    guard.update(RELEASE + 0.01, spin_target, None)          # -> ALIGNING
    guard.update(RELEASE + ALIGN_TIMEOUT * 0.5, spin_target, None)
    half = not guard.drive_allowed
    guard.update(RELEASE + ALIGN_TIMEOUT + 0.02, spin_target, None)
    check(
        "no feedback: not armed at half the align window, armed after it, flagged",
        half and guard.drive_allowed and guard.armed_without_feedback,
    )

    # 4.5 Manoeuvre-to-manoeuvre skips RELEASING (the override already lapsed),
    #     but the alignment itself still applies where the pose really changes.
    #     crab left <-> crab right share ONE pose (-90/+90/+90/-90) and differ
    #     only in the sign of the wheel speeds, so no wheel has to move and the
    #     guard arms on the first tick. crab -> spin is a real 40 deg slew.
    guard = new_guard()
    guard.request(CRAB_LEFT, 0.0)
    guard.update(RELEASE + 0.01, crab_target, crab_target)
    guard.request(CRAB_RIGHT, 1.0)
    no_release_wait = guard.state == GuardState.ALIGNING
    guard.update(1.01, manoeuvre_pose(CRAB_RIGHT, MODEL, LIMITS, CRAB_SPEED, SPIN_SPEED, STRAIGHT), crab_target)
    same_pose_instant = guard.drive_allowed
    guard.request(SPIN_CW, 2.0)
    guard.update(2.01, spin_target, crab_target)
    real_slew_waits = not guard.drive_allowed
    guard.update(2.02, spin_target, spin_target)
    check(
        "crab left -> crab right: no release wait, arms at once (identical pose); "
        "crab -> spin: waits for the 40 deg slew",
        no_release_wait and same_pose_instant and real_slew_waits and guard.drive_allowed,
    )

    # 4.6 Returning to cornering is guarded too — the wheels are still at 90 deg.
    guard = new_guard()
    guard.request(CRAB_LEFT, 0.0)
    guard.update(3.0, crab_target, crab_target)
    guard.request(CORNERING, 3.0)
    guard.update(3.05, [0.0] * 4, crab_target)
    withheld = not guard.drive_allowed
    guard.update(3.10, [0.0] * 4, [0.0] * 4)
    check(
        "arrow released: W/S traction withheld until the wheels are back straight",
        withheld and guard.drive_allowed,
    )

    # 4.7 Space bar. Out of a manoeuvre it must not re-arm blind; in plain
    #     cornering it must behave exactly as before this feature.
    guard = new_guard()
    guard.request(CRAB_LEFT, 0.0)
    guard.update(3.0, crab_target, crab_target)
    guard.force_cornering(3.0, rearm=False)
    stop_from_crab = guard.manoeuvre == CORNERING and not guard.drive_allowed
    guard = new_guard()
    guard.force_cornering(0.0, rearm=True)
    stop_from_cornering = guard.manoeuvre == CORNERING and guard.drive_allowed
    check(
        "space bar: cornering -> stop + immediately drivable again (unchanged); "
        "crab/spin -> stop + traction withheld until straight",
        stop_from_crab and stop_from_cornering,
    )

    # 4.8 An unreachable pose must never arm.
    guard = new_guard()
    guard.request(CRAB_LEFT, 0.0)
    for step in range(200):
        guard.update(step * 0.05, None, [0.0] * 4)
    check(
        "unreachable pose (limiter would reject): never armed, no matter how long",
        not guard.drive_allowed,
    )
    print()

    if failures:
        print("FAILURES: " + "; ".join(failures))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
