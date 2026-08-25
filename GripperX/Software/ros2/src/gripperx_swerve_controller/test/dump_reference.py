#!/usr/bin/env python3
r"""
Functional-equivalence harness, PYTHON side (NFR-10 acceptance 1).

Reads "vx vy omega d0 d1 d2 d3" per line from stdin (current steering angles in
MODEL order FL, BL, BR, FR, radians) and prints what today's chain resolves for
that twist, using the LIVE modules the running robot uses — no reimplementation:

    gripperx_control.swerve_kinematic_model.FourWIS4WIDKinematicModel
    gripperx_control.steering_limits.limit_twist_to_steering_range
    swerve_cmd_node._steer_alignment_scale  (inlined below: importing
        swerve_cmd_node pulls in rclpy, which this harness does not need)

Output per case, matching test/dump_reference.cpp:

    <status> a0 s0 c0 a1 s1 c1 a2 s2 c2 a3 s3 c3

with a = commanded steering angle, s = commanded wheel angular velocity,
c = the steer-alignment (slew braking) scale that produced s.

There is deliberately NO --reference switch here, unlike on the C++ side.
swerve_cmd_node cannot see arbitration point A2 — the /teleop/direct_steer
override lives in another process — so the IK target is the only braking
reference today's chain HAS. That asymmetry is the point of the comparison:
`dump_reference --reference=commanded` must reproduce this output exactly on
every case without override columns, and is expected to differ on cases with
them. Any line with more than 7 numbers therefore has its extra columns ignored
here rather than silently changing this side's behaviour.

Run it against test/dump_reference.cpp's output with the same input; any
difference beyond floating-point noise is a failed port.

Usage:
    PYTHONPATH=<repo>/Software/ros2/src/gripperx_control/src \\
        python3 dump_reference.py < cases.txt
"""

import math
import sys

from gripperx_control.steering_limits import (
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    limit_twist_to_steering_range,
    LimitStatus,
    normalize_angle,
    SteeringLimits,
)
from gripperx_control.swerve_kinematic_model import (
    BodyTwist,
    FourWIS4WIDKinematicModel,
)

# gripperx_control/config/swerve_cmd.yaml, unchanged.
from gripperx_geometry.constants import (  # noqa: E402
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as R,
)


MIN_SCALE = 0.45
DEADBAND = 0.12
REFERENCE = 1.0472
MAX_WHEEL_ANGULAR_SPEED = 12.0


def alignment_scale(target: float, current: float) -> float:
    error = abs(normalize_angle(target - current))
    if error <= DEADBAND:
        return 1.0
    return max(MIN_SCALE, 1.0 - (error / REFERENCE))


def main() -> int:
    model = FourWIS4WIDKinematicModel(a=A, b=B, wheel_radius=R)
    limits = SteeringLimits.from_outward_inward(
        math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
        math.radians(DEFAULT_INWARD_LIMIT_DEG),
        DEFAULT_OUTWARD_SIGN,
    ).in_model_order()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        values = [float(v) for v in line.split()]
        vx, vy, omega = values[0:3]
        current = values[3:7]

        limited = limit_twist_to_steering_range(
            model, BodyTwist(vx=vx, vy=vy, omega=omega), current, limits
        )
        if limited.status == LimitStatus.REJECTED:
            print('rejected')
            continue

        parts = [limited.status]
        for target, current_angle in zip(limited.targets, current):
            scale = alignment_scale(target.angle, current_angle)
            speed = (target.speed / R) * scale
            speed = max(-MAX_WHEEL_ANGULAR_SPEED, min(MAX_WHEEL_ANGULAR_SPEED, speed))
            parts.append('%.9f' % target.angle)
            parts.append('%.9f' % speed)
            parts.append('%.9f' % scale)
        print(' '.join(parts))
    return 0


if __name__ == '__main__':
    sys.exit(main())
