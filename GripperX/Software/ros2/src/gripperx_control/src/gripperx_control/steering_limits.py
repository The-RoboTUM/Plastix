"""Per-wheel, per-direction steering limits for the 4WIS kinematics chain.

Why this module exists
----------------------
The steering range of this robot is **asymmetric and per wheel**. Measured on
the machine 2026-08-13 (see `config/steer_servo.yaml`, which is the source of
truth): every wheel reaches ~100 deg OUTWARD (tyre swings away from the
chassis) and only ~35 deg INWARD (raised from ~30 deg 2026-08-17 — a user
estimate, TO-VERIFY, not a new measurement; see `config/steer_servo.yaml` for
the honesty caveat), and which *sign of the joint angle* is outward differs
per wheel:

    steering_outward_sign = (-1, +1, +1, -1)     # FL, FR, BL, BR (MEASURED)

so in robot-frame joint angles the reachable window is

    FL [-100, +35]   FR [-35, +100]   BL [-35, +100]   BR [-100, +35]   (deg)

`steer_servo_node` already clamps per joint against exactly this window. That
clamp is the last line of defence, and everything upstream must plan inside the
same window — otherwise the servo node silently clamps a wheel, the pose no
longer matches any consistent instantaneous centre of rotation, and the wheels
scrub against each other.

A single symmetric `steering_angle_limit` cannot express this: 30 throws away
70 deg of outward travel, 100 lets the kinematics request angles that get
clamped. Hence per-wheel bounds and a feasibility step, both defined here so
they can be unit-checked without ROS (this module imports nothing from rclpy).

Sign convention
---------------
Identical for all four joints (URDF `steer_joint`: axis "0 0 1", rpy "0 0 0",
parent chassis_link; IK: delta_i = atan2(vy_i, vx_i)): +angle points a wheel
towards the robot's left (+y). "Outward" is NOT derivable from that, because
each wheel hangs on a lateral lever arm off its king pin — it was measured.
Do not re-derive `steering_outward_sign` from the URDF; a URDF-only reading
gives (+1, -1, +1, -1) and is wrong on the front pair.

Sanity check to re-run if these numbers are ever touched: the in-place spin
pose (FL -58.6, FR +58.6, BL +58.6, BR -58.6 deg) must be reachable on all
four wheels. Under the refuted sign it would need that much inward on three
wheels — i.e. spin would be impossible.

The magnitude was 50.7 here until 2026-08-21 and that value was stale. Measured
in the twin on that date, all four steering joints settle at 58.57 deg under a
pure rotation command; computing it from the controller's own geometry
(a = 0.180, b = 0.110, ros2_controllers.yaml) gives atan2(a, -b) folded through
the +-180 module flip = 58.57 deg, and from the exact CAD king-pin pair
(0.1809 / 0.1087) 59.04 deg. Nothing reachable changes — 58.6 is still far
inside the 100 deg outward range — but the number is now the one the machine
actually holds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from gripperx_control.swerve_kinematic_model import (
    BodyTwist,
    FourWIS4WIDKinematicModel,
    WheelCommand,
)

# Joint order used by steer_servo.yaml / steer_servo_node / the ros2_control
# steering controller.
JOINT_ORDER_LABELS: Tuple[str, ...] = ("FL", "FR", "BL", "BR")
# Paper/kinematic-model wheel order (w1..w4 in Lee 2015): FL, BL, BR, FR.
MODEL_ORDER_LABELS: Tuple[str, ...] = ("FL", "BL", "BR", "FR")
# Index of each model-order wheel inside a joint-order array.
MODEL_TO_JOINT_INDEX: Tuple[int, ...] = (0, 2, 3, 1)

# Outward measured 2026-08-13; inward raised 30 -> 35 deg 2026-08-17 (user
# choice, TO-VERIFY — the limit is chosen, not measured). The travel itself IS
# measured: the tightest inward stop is BL at 35.60 deg, so 35 leaves only
# ~0.6 deg of margin where 30 kept ~5.6. See config/steer_servo.yaml for the
# full provenance and the risk. Mirrors config/steer_servo.yaml. Defaults only — the
# runtime values come from parameters so the two files can be aligned without
# a rebuild.
DEFAULT_OUTWARD_LIMIT_DEG = 100.0
DEFAULT_INWARD_LIMIT_DEG = 35.0
DEFAULT_OUTWARD_SIGN: Tuple[int, ...] = (-1, 1, 1, -1)

# Floating-point slack when testing "is this angle inside the window".
ANGLE_TOLERANCE_RAD = 1e-6


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class SteeringLimits:
    """Reachable joint-angle window per wheel, in one fixed wheel order."""

    lower: Tuple[float, ...]
    upper: Tuple[float, ...]
    labels: Tuple[str, ...]

    @staticmethod
    def from_outward_inward(
        outward_rad: float,
        inward_rad: float,
        outward_sign: Sequence[int],
        labels: Sequence[str] = JOINT_ORDER_LABELS,
    ) -> "SteeringLimits":
        """Build the joint-order window from the steer_servo.yaml schema.

        `outward_rad`/`inward_rad` are magnitudes; `outward_sign[i]` says which
        sign of wheel i's joint angle is physically outward.
        """
        if outward_rad <= 0.0 or inward_rad <= 0.0:
            raise ValueError(
                "outward/inward steering limits are magnitudes and must be > 0"
            )
        if len(outward_sign) != len(labels):
            raise ValueError("outward_sign must have one entry per wheel")
        if any(int(s) not in (-1, 1) for s in outward_sign):
            raise ValueError("outward_sign values must be +1 or -1")

        lower: List[float] = []
        upper: List[float] = []
        for sign in outward_sign:
            if int(sign) > 0:
                lower.append(-inward_rad)
                upper.append(outward_rad)
            else:
                lower.append(-outward_rad)
                upper.append(inward_rad)
        return SteeringLimits(tuple(lower), tuple(upper), tuple(labels))

    def reordered(
        self,
        index_map: Sequence[int],
        labels: Sequence[str],
    ) -> "SteeringLimits":
        return SteeringLimits(
            tuple(self.lower[i] for i in index_map),
            tuple(self.upper[i] for i in index_map),
            tuple(labels),
        )

    def in_model_order(self) -> "SteeringLimits":
        """Same windows, reordered from joint order to Lee-2015 wheel order."""
        return self.reordered(MODEL_TO_JOINT_INDEX, MODEL_ORDER_LABELS)

    def contains(self, index: int, angle: float) -> bool:
        return (
            (self.lower[index] - ANGLE_TOLERANCE_RAD)
            <= angle
            <= (self.upper[index] + ANGLE_TOLERANCE_RAD)
        )

    def clamp(self, index: int, angle: float) -> float:
        return max(self.lower[index], min(self.upper[index], angle))

    def describe(self) -> str:
        return " ".join(
            f"{label}[{math.degrees(low):+.0f},{math.degrees(high):+.0f}]"
            for label, low, high in zip(self.labels, self.lower, self.upper)
        )


@dataclass(frozen=True)
class WheelTarget:
    """One module's resolved command: steering angle and signed linear speed."""

    angle: float
    speed: float


@dataclass(frozen=True)
class LimitViolation:
    index: int
    label: str
    requested: float
    lower: float
    upper: float

    def describe(self) -> str:
        return (
            f"{self.label} needs {math.degrees(self.requested):+.1f}deg, window "
            f"[{math.degrees(self.lower):+.1f}, {math.degrees(self.upper):+.1f}]deg"
        )


class LimitStatus:
    """Outcome of `limit_twist_to_steering_range`."""

    OK = "ok"
    OMEGA_REDUCED = "omega_reduced"
    REJECTED = "rejected"


@dataclass(frozen=True)
class LimitedTwist:
    status: str
    twist: Optional[BodyTwist]
    targets: Optional[List[WheelTarget]]
    requested_omega: float
    violations: Tuple[LimitViolation, ...]


def equivalent_solutions(
    steering_angle: float,
    linear_speed: float,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """The two module solutions that realise the same wheel velocity vector."""

    angle = normalize_angle(steering_angle)
    return (
        (angle, linear_speed),
        (normalize_angle(angle + math.pi), -linear_speed),
    )


def resolve_wheel_targets(
    wheel_commands: Sequence[WheelCommand],
    current_angles: Sequence[float],
    limits: SteeringLimits,
) -> Optional[List[WheelTarget]]:
    """Pick a reachable module solution per wheel, or None if one has none.

    Of the two equivalent solutions (angle, +v) / (angle+180, -v) the one
    closest to the module's current angle is preferred — but only among the
    solutions that lie inside that wheel's window. With asymmetric limits the
    nearer solution is not always the reachable one, so limit-awareness has to
    happen HERE and not in a later clamp.
    """

    targets: List[WheelTarget] = []
    for index, (command, current) in enumerate(zip(wheel_commands, current_angles)):
        best: Optional[Tuple[float, WheelTarget]] = None
        for angle, speed in equivalent_solutions(
            command.steering_angle, command.linear_speed
        ):
            if not limits.contains(index, angle):
                continue
            distance = abs(normalize_angle(angle - current))
            if best is None or distance < best[0]:
                best = (distance, WheelTarget(limits.clamp(index, angle), speed))
        if best is None:
            return None
        targets.append(best[1])
    return targets


def unconstrained_targets(
    wheel_commands: Sequence[WheelCommand],
    current_angles: Sequence[float],
) -> List[WheelTarget]:
    """What the IK would ask for if there were no limits (for diagnostics)."""

    targets: List[WheelTarget] = []
    for command, current in zip(wheel_commands, current_angles):
        solutions = equivalent_solutions(command.steering_angle, command.linear_speed)
        angle, speed = min(
            solutions, key=lambda s: abs(normalize_angle(s[0] - current))
        )
        targets.append(WheelTarget(angle, speed))
    return targets


def find_violations(
    targets: Sequence[WheelTarget],
    limits: SteeringLimits,
) -> Tuple[LimitViolation, ...]:
    return tuple(
        LimitViolation(
            index=index,
            label=limits.labels[index],
            requested=target.angle,
            lower=limits.lower[index],
            upper=limits.upper[index],
        )
        for index, target in enumerate(targets)
        if not limits.contains(index, target.angle)
    )


def limit_twist_to_steering_range(
    model: FourWIS4WIDKinematicModel,
    body_twist: BodyTwist,
    current_angles: Sequence[float],
    limits: SteeringLimits,
    iterations: int = 16,
) -> LimitedTwist:
    """Make a requested body twist reachable by reducing |omega| only.

    Design decision (2026-08-13, stage B)
    -------------------------------------
    Three options were on the table for a pose that violates a limit:

    * clamp per wheel — cheapest, but a clamped wheel no longer points along
      the velocity its own module is being driven at. The four wheels stop
      sharing one instantaneous centre of rotation, so they fight each other
      and scrub. This is what happens today, silently, inside steer_servo_node.
    * scale the whole twist — provably useless: delta_i = atan2(vy_i, vx_i) is
      INVARIANT under uniform scaling of (vx, vy, omega), so it cannot reduce
      any angle. The old `_scale_twist_to_steer_limit` did exactly this and
      only throttled the robot (documented and removed 2026-07-16).
    * reduce the RATIO omega/(vx, vy) — this is what actually rotates the wheel
      angles, because it moves the instantaneous centre outward. The manoeuvre
      stays a geometrically consistent turn, just a wider one.

    We take the third: keep (vx, vy) exactly as commanded and shrink |omega|
    until every wheel is inside its own window. Consequences:

    * Pure translation (omega = 0) is always reachable (all wheels at the
      translation angle) as long as that angle itself is reachable, so the
      search always has a feasible lower bound for the usual driving case.
    * In-place spin (vx = vy = 0) is unaffected: the wheel angles do not depend
      on |omega| there at all. The spin pose needs 58.6 deg OUTWARD on every
      wheel and is inside the 100 deg outward range, so it passes untouched.
    * Ordinary cornering loses turn AUTHORITY, not turn shape: the requested
      curvature is capped at the tightest radius the 30 deg inward limit
      allows. The robot turns wider than asked instead of scrubbing.
    * If even omega = 0 is unreachable, the requested DIRECTION OF TRAVEL
      itself cannot be steered to. Pure crab (vy only) is NOT such a case —
      `resolve_wheel_targets` reaches it at ∓90 deg via the ±180 module flip,
      which is what the teleop arrow keys rely on (FR-7). Steep diagonals
      (FR-8) can still land here. There is no way to honour that partially
      without driving in a direction nobody asked for, so it is REJECTED and
      the caller must not move. Rejection is reported, never silent.

    Monotonicity: for fixed (vx, vy) the velocity vector of each wheel travels
    along a straight line in the velocity plane as omega varies, so each wheel
    angle sweeps monotonically. The bisection below therefore converges on the
    real boundary; where the +-180 module flip makes the reachable set fall
    apart into two arcs, the result is conservative (a feasible omega that may
    be smaller than the true maximum), never optimistic.
    """

    requested_omega = body_twist.omega

    targets = resolve_wheel_targets(
        model.inverse_kinematics(body_twist), current_angles, limits
    )
    if targets is not None:
        return LimitedTwist(
            status=LimitStatus.OK,
            twist=body_twist,
            targets=targets,
            requested_omega=requested_omega,
            violations=(),
        )

    violations = find_violations(
        unconstrained_targets(model.inverse_kinematics(body_twist), current_angles),
        limits,
    )

    zero_omega_twist = BodyTwist(vx=body_twist.vx, vy=body_twist.vy, omega=0.0)
    zero_omega_targets = resolve_wheel_targets(
        model.inverse_kinematics(zero_omega_twist), current_angles, limits
    )
    if zero_omega_targets is None:
        return LimitedTwist(
            status=LimitStatus.REJECTED,
            twist=None,
            targets=None,
            requested_omega=requested_omega,
            violations=violations,
        )

    low = 0.0
    high = 1.0
    best_twist = zero_omega_twist
    best_targets = zero_omega_targets
    for _ in range(max(1, iterations)):
        mid = 0.5 * (low + high)
        candidate = BodyTwist(
            vx=body_twist.vx,
            vy=body_twist.vy,
            omega=mid * requested_omega,
        )
        candidate_targets = resolve_wheel_targets(
            model.inverse_kinematics(candidate), current_angles, limits
        )
        if candidate_targets is None:
            high = mid
        else:
            low = mid
            best_twist = candidate
            best_targets = candidate_targets

    return LimitedTwist(
        status=LimitStatus.OMEGA_REDUCED,
        twist=best_twist,
        targets=best_targets,
        requested_omega=requested_omega,
        violations=violations,
    )


def symmetric_limit_for_pattern(
    pattern: Sequence[float],
    limits: SteeringLimits,
) -> float:
    """Largest |angle| a fixed steering PATTERN can use inside the windows.

    A pattern is a per-wheel factor applied to one operator steering value,
    e.g. keyboard teleop's [+1, +1, -1, -1] (front and rear axle counter-
    rotating). Because the same magnitude is demanded in both directions, the
    usable envelope is the tightest bound over all wheels and both signs.
    """

    if len(pattern) != len(limits.labels):
        raise ValueError("pattern must have one factor per wheel")

    bound = math.inf
    for index, factor in enumerate(pattern):
        if factor == 0.0:
            continue
        for sign in (1.0, -1.0):
            # angle = sign * magnitude must satisfy the window after scaling.
            edge = limits.upper[index] if (sign * factor) > 0 else limits.lower[index]
            bound = min(bound, abs(edge / factor))
    return bound
