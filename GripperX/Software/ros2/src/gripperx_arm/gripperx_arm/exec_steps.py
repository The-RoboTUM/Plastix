"""The setpoints an executor writes for one joint-space segment. Pure Python, no ROS.

ONE generator for the planner, which validates these setpoints, and for the arm
node's executors, which write them (user decision 2026-10-09): the planner must check
what is executed, not the ideal joint-linear line. The servos take whole ticks, so an
executed setpoint lies up to half a tick per joint off that line; a segment the planner
approved because its line passes an obstacle at exactly the configured clearance could
otherwise be executed through poses on the wrong side of it.

A segment qa -> qb is executed as a ramp LINEAR IN TICKS between the rounded end
points: start = round(qa), setpoint i = round(start + (end - start) * i / n), i = 1..n,
end = round(qb). The step count n is the executor's: the fixed-sequence ramp derives
it from the move time (`ramp_count`), the leader follower from its per-tick velocity
limit (`velocity_count`). Rounding a linear function keeps every joint monotonic along
the ramp, which the safety filter's motion bound assumes between setpoints.
"""
from __future__ import annotations

import math
from typing import Dict, List, Mapping, Sequence


def unwrapped_ticks(cal, q: Mapping[str, float]) -> List[int]:
    """Whole servo ticks of pose q, in cal.joints order (a TickCalibration), NOT reduced
    to the register's single turn - the one rounding every setpoint goes through."""
    k = cal.ticks_per_rev / (2.0 * math.pi)
    return [int(round(z + s * q[n] * k))
            for n, z, s in zip(cal.joints, cal.zero_ticks, cal.signs)]


def tick_ramp(start: Mapping[int, int], end: Mapping[int, int], n: int) -> List[Dict[int, int]]:
    """Setpoints 1..n (start excluded, end included) for the servos in `end`."""
    return [{sid: int(round(start[sid] + (end[sid] - start[sid]) * i / n)) for sid in end}
            for i in range(1, n + 1)]


def ramp_count(seconds: float, rate_hz: float) -> int:
    """Steps of a timed ramp written at rate_hz."""
    return max(1, int(seconds * rate_hz))


def velocity_count(travel_rad: float, max_step_rad: float) -> int:
    """Fewest equal steps that keep every joint within max_step_rad per step."""
    return max(1, int(math.ceil(travel_rad / max_step_rad - 1e-9)))


def exec_poses(cal, qa: Mapping[str, float], qb: Mapping[str, float],
               n: int) -> List[Dict[str, float]]:
    """The poses an executor passes through for qa -> qb in n steps: [qa as written
    (rounded), setpoint 1, ..., setpoint n], every joint of `cal`."""
    ta, tb = (dict(enumerate(unwrapped_ticks(cal, q))) for q in (qa, qb))
    return [cal.to_radians([t[i] for i in range(len(ta))]) for t in [ta] + tick_ramp(ta, tb, n)]


def ticks_at(start: Mapping, end: Mapping, fractions: Sequence[float],
             dedupe: bool = False) -> List[Dict]:
    """Setpoints at the given increasing fractions of the way start -> end (the last
    one 1.0), linear in ticks like tick_ramp: round(start + (end - start) * f) per servo
    (keys of `end`). ONE expression for the planner (exec_poses_at) and the timed ramp
    (arm_action_server._ramp), so both round a fraction to the same whole tick.
    `dedupe`: a setpoint equal to the one before it (the first compared with `start`)
    is dropped - a soft start's first fraction of a slow joint can round to no motion,
    and a repeated setpoint is no step (the ground recovery's 'strictly rising' would
    refuse it)."""
    out: List[Dict] = []
    last = {k: int(round(start[k])) for k in end}
    for f in fractions:
        sp = {k: int(round(start[k] + (end[k] - start[k]) * f)) for k in end}
        if dedupe and sp == last:
            continue
        out.append(sp)
        last = sp
    return out


def exec_poses_at(cal, qa: Mapping[str, float], qb: Mapping[str, float],
                  fractions: List[float], dedupe: bool = False) -> List[Dict[str, float]]:
    """As exec_poses, at the given increasing fractions of the way (the last one 1.0)
    instead of n equal steps: [qa as written (rounded), setpoint at fractions[0], ...].
    Linear in register ticks like tick_ramp, so every setpoint lies on the same line.
    `dedupe`: see ticks_at."""
    ta, tb = (dict(enumerate(unwrapped_ticks(cal, q))) for q in (qa, qb))
    return [cal.to_radians([t[i] for i in range(len(ta))])
            for t in [ta] + ticks_at(ta, tb, fractions, dedupe)]


def soft_start_fractions(n: int, profile: Sequence[float]) -> List[float]:
    """SOFT START (user decision 2026-10-20) of a segment its executor otherwise runs in
    n equal steps: increasing fractions of the way, the last one 1.0. Step i <
    len(profile) covers profile[i] of a full step (1/n of the segment); the rest
    follows in equal steps no longer than a full step.

    Why: the safety filter checks the START of a motion grown by its first step's
    motion bound (SR-17 rule 5). A full first step from a pose a few mm above the
    ground margin is refused although the motion leads upward; a small first step
    costs a small growth, and every later, larger step starts from higher up. The
    growth model and the margin are unchanged - only the executed steps are smaller."""
    if n < 1:
        raise ValueError(f'step count {n} must be >= 1')
    t, out = 0.0, []
    for p in profile:
        t = min(1.0, t + float(p) / n)
        out.append(t)
        if t >= 1.0 - 1e-12:
            out[-1] = 1.0
            return out
    m = velocity_count(1.0 - t, 1.0 / n)
    out += [t + (1.0 - t) * i / m for i in range(1, m)] + [1.0]
    return out


def check_soft_start(profile: Sequence[float]) -> List[float]:
    """The profile as floats; ValueError for a value outside (0, 1] (a step beyond a
    full one would undo what the velocity limit and the move time set)."""
    out = [float(p) for p in profile]
    bad = [p for p in out if not 0.0 < p <= 1.0]
    if bad:
        raise ValueError(f'soft-start fractions must lie in (0, 1], not {bad}')
    return out


def monotone_cover(poses: List[Dict[str, float]], joints, max_step_rad: float
                   ) -> List[Dict[str, float]]:
    """For a timed ramp (no per-step check at execution): the subsequence of its
    executed poses, first and last kept, in which no joint moves more than
    max_step_rad between neighbours. Every joint is monotonic along a ramp, so each
    dropped setpoint, and the servo motion through it, lies inside the joint box of the
    two kept poses around it - the box SafetyFilter.check_setpoints covers with its
    motion bound. Every pose checked is one the servo is commanded to; the cost is that
    of the ideal line's sub-steps instead of one check per written setpoint. NOT for the
    leader follower: it re-checks every single step, so each must be validated."""
    out = poses[:1]
    for here, nxt in zip(poses[1:-1], poses[2:]):
        if max(abs(nxt[n] - out[-1][n]) for n in joints) > max_step_rad:
            out.append(here)
    return out + poses[-1:] if len(poses) > 1 else out
