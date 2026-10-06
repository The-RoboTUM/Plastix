#!/usr/bin/env python3
"""Offline check of the crab_yaw.py alignment algorithm. Commands nothing, needs no robot.

    python3 tools/check_crab_align_logic.py

WHY IT EXISTS. The alignment loop in crab_yaw.py was wrong twice on 2026-09-18, and
both versions looked like success on the console. v1 exited on a transient and left the
steering 13 deg off while reporting "ALIGNED, worst error 0.51 deg". v2 fixed the exit
test but kept the integrator, ran the command to the window clamp at +-125 deg, and the
resulting hunting slipped all four couplings off their wheels. Neither failure was
catchable by reading the code; both were obvious the moment the loop was run against a
servo that takes time to move. So it gets run against one.

READ THIS BEFORE TRUSTING IT. This file DUPLICATES the loop from crab_yaw.py rather than
importing it -- that module pulls in rclpy and the message packages, which a plain check
should not need. The duplication is the weakness: if someone edits the loop in
crab_yaw.py and not here, this still passes and proves nothing about the code that
actually runs. It verifies the ALGORITHM as written on 2026-09-18, not the file. Keeping
the two in step is manual. The clean fix is to lift the decision logic into a module with
no ROS imports that both can import; that is owed, not done.

What the cases check, each one drawn from a real failure or a real requirement:
  1. from straight ahead, perfect servo   -- the case that broke v1 and v2
  2. from a partly-aligned pose           -- the 2026-08-21 situation
  3. servo settles 3 deg short            -- a genuine offset, must be trimmed out
  4. servo overshoots in flight           -- the v1 trap
  5. offset beyond the trim bound         -- must fail to align rather than wind up
"""

import math

DT = 0.05
RATE_DEG_S = 60.0        # how fast the servo slews
TOL = math.radians(0.8)
SETTLE_EPS = math.radians(0.15)
SETTLE_SAMPLES = 10
MAX_TRIMS = 6
MAX_TRIM_DEG = 10.0
ALIGN_SEC = 30.0


def run(target_deg, start_deg, offset_deg, overshoot_deg=0.0, label=""):
    target = [math.radians(d) for d in target_deg]
    lo = [math.radians(-125.0)] * len(target)
    hi = [math.radians(125.0)] * len(target)
    trim_lo = [max(l, t - math.radians(MAX_TRIM_DEG)) for l, t in zip(lo, target)]
    trim_hi = [min(h, t + math.radians(MAX_TRIM_DEG)) for h, t in zip(hi, target)]

    pos = [math.radians(d) for d in start_deg]
    cmd = [min(max(t, l), h) for t, l, h in zip(target, trim_lo, trim_hi)]
    aligned, trims, still, prev = False, 0, 0, None
    worst_cmd_excursion = 0.0
    declared_while_moving = False
    t = 0.0

    while t < ALIGN_SEC:
        # --- servo model: slew toward cmd, settle offset_deg short, overshoot in flight
        step = math.radians(RATE_DEG_S) * DT
        for i in range(len(pos)):
            goal = cmd[i] - math.copysign(math.radians(offset_deg), cmd[i])
            d = goal - pos[i]
            moving = abs(d) > step
            pos[i] += math.copysign(min(abs(d), step), d)
            if moving and overshoot_deg:
                pos[i] += math.copysign(math.radians(overshoot_deg) * DT, d)
        t += DT

        worst_cmd_excursion = max(
            worst_cmd_excursion,
            max(abs(c - tg) for c, tg in zip(cmd, target)))

        cur = list(pos)
        if prev is not None and max(abs(c - p) for c, p in zip(cur, prev)) < SETTLE_EPS:
            still += 1
        else:
            still = 0
        was_moving = prev is not None and max(
            abs(c - p) for c, p in zip(cur, prev)) >= SETTLE_EPS
        prev = cur
        if still < SETTLE_SAMPLES:
            continue

        err = [tg - c for tg, c in zip(target, cur)]
        worst = max(abs(e) for e in err)
        if worst < TOL:
            aligned = True
            if was_moving:
                declared_while_moving = True
            break
        if trims >= MAX_TRIMS:
            break
        cmd = [min(max(c + e, l), h) for c, e, l, h in zip(cmd, err, trim_lo, trim_hi)]
        trims += 1
        still = 0

    final_err = max(abs(math.degrees(tg - c)) for tg, c in zip(target, pos))
    print(f"{label}")
    print(f"   aligned={aligned}  trims={trims}  t={t:.1f}s")
    print(f"   final settled error : {final_err:6.2f} deg")
    print(f"   worst |cmd-target|  : {math.degrees(worst_cmd_excursion):6.2f} deg "
          f"(bound {MAX_TRIM_DEG})")
    ok = (math.degrees(worst_cmd_excursion) <= MAX_TRIM_DEG + 1e-9
          and not declared_while_moving
          and (not aligned or final_err < math.degrees(TOL) + 0.01))
    print(f"   -> {'PASS' if ok else 'FAIL'}")
    print()
    return ok


CRAB = [-90.0, 90.0, 90.0, -90.0]
allok = True
allok &= run(CRAB, [0.0] * 4, 0.0, label="1. from straight ahead, perfect servo "
                                         "(this is the case that broke v1 and v2)")
allok &= run(CRAB, [-77.0, 77.0, 77.0, -77.0], 0.0,
             label="2. from a partly-aligned pose")
allok &= run(CRAB, [0.0] * 4, 3.0, label="3. servo settles 3 deg short -- a REAL offset, "
                                         "must be trimmed out")
allok &= run(CRAB, [0.0] * 4, 0.0, overshoot_deg=40.0,
             label="4. servo overshoots badly in flight (the v1 trap)")
allok &= run(CRAB, [0.0] * 4, 25.0, label="5. offset larger than the trim bound -- "
                                          "must FAIL to align, not wind up")
print("ALL CHECKS PASSED" if allok else "SOMETHING FAILED")
