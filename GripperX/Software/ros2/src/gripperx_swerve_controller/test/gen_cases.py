#!/usr/bin/env python3
r"""
Case generator for the functional-equivalence harness (NFR-10 acceptance 1).

The 413 cases this emits are the ones the 2026-08-19 comparison ran on — 13
hand-picked plus 400 seeded-random — reproduced here byte for byte. They lived
in a scratch directory, which made the number reproducible only by luck; the
input side of an acceptance check belongs with the code it checks.

    python3 gen_cases.py            -> the 413 cases        (7 numbers per line)
    python3 gen_cases.py --override -> 4 x 413 = 1652 cases (11 numbers per line)

DO NOT RENUMBER OR RESHUFFLE the named list, the seed, the draw order or the
ranges: any of those changes the 400 random cases and silently breaks continuity
with the recorded result.

The `--override` set appends the POST-ARBITRATION commanded steering angles —
what an active /teleop/direct_steer override (arbitration point A2) makes the
controller command instead of the IK target. It exists for the 2026-08-19
braking-reference decision: the two references coincide wherever no override is
active, and the override cases are the intended-difference set. The angles are
emitted uniformly across the four modules because that is how
keyboard_teleop_node publishes them (one cumulative A/D angle, same for all
four); 0.0 is included on purpose, because it is FR-13 / E-stop centring, where
override and IK target agree again and the outputs must NOT differ.

Angles are in MODEL order (FL, BL, BR, FR), radians.
"""

import math
import random
import sys

named = [
    # vx    vy     omega  current steering angles in MODEL order FL, BL, BR, FR
    (0.30, 0.00, 0.00, [0.0, 0.0, 0.0, 0.0]),        # straight ahead
    (-0.25, 0.00, 0.00, [0.0, 0.0, 0.0, 0.0]),       # straight reverse
    (0.30, 0.00, 0.30, [0.0, 0.0, 0.0, 0.0]),        # gentle turn
    (0.10, 0.00, 0.30, [0.0, 0.0, 0.0, 0.0]),        # omega must be reduced
    (0.00, 0.00, 1.00, [0.0, 0.0, 0.0, 0.0]),        # in-place swerve spin
    (0.00, 0.25, 0.00, [0.0, 0.0, 0.0, 0.0]),        # pure crab (FR-7)
    (0.05, 0.05, 0.10, [0.0, 0.0, 0.0, 0.0]),        # REJECTED (steep diagonal)
    (0.20, 0.35, 0.00, [0.0, 0.0, 0.0, 0.0]),        # REJECTED (60 deg heading)
    (0.00, 0.00, 0.00, [0.06, -0.048, 0.006, 0.051]),  # zero twist, wheels off centre
    (2.00, 0.00, 0.00, [0.0, 0.0, 0.0, 0.0]),        # wheel-speed saturation
    (0.30, 0.00, 0.00, [3.0, -3.0, 3.0, -3.0]),      # nearest solution = module flip
    (0.30, 0.00, 0.00, [0.6, 0.6, 0.6, 0.6]),        # slew braking active
    (0.15, -0.05, -0.40, [0.2, -0.2, 0.1, -0.1]),    # mixed
]

# Post-arbitration override angles for --override. 35 deg is the A/D steering
# angle the keyboard chain actually reaches; 0.9 rad is a deliberately larger
# one, past the point where the alignment scale saturates at min_scale.
OVERRIDES = [0.0, math.radians(35.0), math.radians(-35.0), 0.9]


def main() -> int:
    with_override = '--override' in sys.argv[1:]

    random.seed(20260819)
    cases = []
    for case in named:
        cases.append([case[0], case[1], case[2]] + list(case[3]))
    for _ in range(400):
        vx = random.uniform(-0.6, 0.6)
        vy = random.uniform(-0.3, 0.3)
        om = random.uniform(-2.0, 2.0)
        cur = [random.uniform(-math.pi, math.pi) for _ in range(4)]
        cases.append([vx, vy, om] + cur)

    lines = []
    for values in cases:
        if with_override:
            for override in OVERRIDES:
                lines.append(values + [override] * 4)
        else:
            lines.append(values)

    for values in lines:
        print(' '.join('%.9f' % v for v in values))
    return 0


if __name__ == '__main__':
    sys.exit(main())
