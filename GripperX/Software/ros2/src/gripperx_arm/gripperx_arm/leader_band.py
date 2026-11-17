"""Leader-mode command band (user decision 2026-10-15) and the slowdown toward its edges
(user decision 2026-10-16, below). Pure python, no ROS imports.

ROBOT FINDING 2026-10-15 15:17:04: the operator led limb2 to its lower joint limit
(-3.3427 rad); the follower was commanded exactly there, sagged one tick beyond under
gravity, and the measured-pose envelope check (SR-17 rule 8) ended leader mode. Holding
at a joint limit in leader mode was impossible.

Leader setpoints are therefore kept `margin_ticks` WHOLE servo ticks inside each joint
limit. The joint limits and the measured-pose check are unchanged: only what leader
mode commands is narrowed. The band is computed in the servo's tick space and rounded
inward to whole ticks, so a written goal (whole ticks) is never closer to a limit than
`margin_ticks`.

Used by arm_action_server (the follower's clamp) and by gripperx_leader (the leader's
virtual wall at the same edge): one computation, one set of inputs - the joint limits
(gripperx_geometry), the tick calibration (arm_calibration.yaml) and
leader.limit_command_margin_ticks (arm_safety_filter.yaml).
"""

import math
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

_EPS = 1e-6     # a limit that lands on a whole tick within float noise counts as on it


def command_band(lo: float, hi: float, zero_tick: float, sign: int, ticks_per_rev: float,
                 margin_ticks: int) -> Tuple[float, float]:
    """(lower, upper) joint angle in rad of the leader command band of one joint.

    `lo`, `hi`: the joint limits (rad); `zero_tick`, `sign`, `ticks_per_rev`: the
    joint's tick calibration (q = sign * (ticks - zero) * 2 pi / ticks_per_rev).
    Raises ValueError when the margin is negative or the band would be empty."""
    if isinstance(margin_ticks, bool) or int(margin_ticks) != margin_ticks or margin_ticks < 0:
        raise ValueError(f'margin {margin_ticks!r} must be a whole number of ticks >= 0')
    if not lo < hi:
        raise ValueError(f'joint limits [{lo}, {hi}] empty')
    if sign not in (1, -1):
        raise ValueError(f'sign {sign!r} must be +1 or -1')
    k = float(ticks_per_rev) / (2.0 * math.pi)
    a, b = zero_tick + sign * lo * k, zero_tick + sign * hi * k
    t_lo, t_hi = min(a, b), max(a, b)
    e_lo = math.ceil(t_lo - _EPS) + int(margin_ticks)
    e_hi = math.floor(t_hi + _EPS) - int(margin_ticks)
    if e_lo > e_hi:
        raise ValueError(f'joint limits [{lo:.4f}, {hi:.4f}] rad narrower than twice the '
                         f'{int(margin_ticks)} tick margin')
    q1, q2 = sign * (e_lo - zero_tick) / k, sign * (e_hi - zero_tick) / k
    return (min(q1, q2), max(q1, q2))


# ------------------------------------------------------------------ edge slowdown
# ROBOT FINDING 2026-10-16 (0.8 rad/s): limb2, led fast downward, reached its lower band
# edge at full speed; the servo trailing its setpoint carried on by momentum to 2 ticks
# beyond the joint limit despite the 10-tick margin, and the measured-pose check (SR-17
# rule 8) ended leader mode. User decision 2026-10-16: slow down near the edges.
#
# Profile: CONSTANT DECELERATION. A joint moving toward an edge at distance d may move at
#     v(d) = min(vmax, sqrt(vslow^2 + (vmax^2 - vslow^2) * d / zone))
# i.e. vmax at the zone start, vslow at the edge, and in between the speed a body braking
# at a = (vmax^2 - vslow^2) / (2 zone) would have. Chosen over a linear ramp in d: for the
# same zone the linear ramp brakes hardest right at the zone start (a = v dv/dd peaks at
# vmax there: 1.6x the constant value with the 2026-10-16 numbers), so the servo is jerked
# where it is fastest; the constant profile spreads the braking evenly. Moving AWAY from an
# edge is never slowed.

def edge_speed(distance: float, vmax: float, zone: float, vslow: float) -> float:
    """Allowed speed (rad/s) of a joint moving toward a command-band edge `distance` rad
    away (<= 0: at or beyond it). zone <= 0 switches the slowdown off."""
    if zone <= 0.0 or distance >= zone:
        return vmax
    d = max(distance, 0.0)
    return min(vmax, math.sqrt(vslow * vslow + (vmax * vmax - vslow * vslow) * d / zone))


def check_edge_slowdown(zone: float, vslow: float, min_vslow: float = 0.0) -> None:
    """Raises ValueError for parameters the slowdown cannot use: a negative zone, or a
    slow speed below `min_vslow` or <= 0. The caller passes one servo tick per leader
    tick (2 pi / ticks_per_rev * rate_hz, audit ES1 L1): slower, the joint that sets the
    step rounds to the same whole tick every tick and the arm freezes without a word."""
    if not zone >= 0.0:
        raise ValueError(f'edge_slowdown_zone_rad {zone!r} must be >= 0')
    if not (vslow > 0.0 and vslow >= min_vslow):
        raise ValueError(f'edge_slowdown_velocity_rad_s {vslow!r} must be > 0 and at least '
                         f'one servo tick per leader tick ({min_vslow:.4f} rad/s)')


def step_scale(cmd: Dict[str, float], goal: Dict[str, float],
               edges: Dict[str, Tuple[float, float]], vmax: float, rate_hz: float,
               zone: float, vslow: float,
               measured: Optional[Dict[str, float]] = None) -> float:
    """The factor in [0, 1] the step goal - cmd is scaled by, UNIFORMLY over the joints
    in `goal`: the step keeps the direction of the motion (the line the safety filter
    checks from the measured pose). Per joint the step may cover at most
    edge_speed(d) / rate_hz, d = its distance to the edge (`edges`: (lower, upper) rad)
    it moves toward; the most constrained joint sets the factor, so one joint nearing an
    edge slows the whole step. d is taken from the setpoint `cmd` and, when given, from
    the `measured` pose, whichever is closer: approaching an edge the servo trails the
    setpoint (the setpoint decides), a servo sagging AHEAD of it under load is caught by
    the measured term. With zone 0 this is the plain velocity limit vmax / rate_hz."""
    scale = 1.0
    for n, g in goal.items():
        delta = g - cmd[n]
        if delta == 0.0:
            continue
        lo, hi = edges[n]
        d = (cmd[n] - lo) if delta < 0.0 else (hi - cmd[n])
        if measured is not None:
            d = min(d, (measured[n] - lo) if delta < 0.0 else (hi - measured[n]))
        cap = edge_speed(d, vmax, zone, vslow) / rate_hz
        scale = min(scale, cap / abs(delta))
    return scale


def braked_fractions(qa: Mapping[str, float], qb: Mapping[str, float], joints: Sequence[str],
                     edges: Dict[str, Tuple[float, float]], vmax: float, rate_hz: float,
                     zone: float, vslow: float, soft: Sequence[float] = ()
                     ) -> Optional[List[float]]:
    """The setpoints of one leader path segment qa -> qb as increasing fractions of the
    way (the last 1.0), each step as step_scale allows from the previous setpoint toward
    qb (audit ES1 H1: planned re-plan paths ran into the band edge at full speed). All
    setpoints lie on the segment, so they are the poses the planner validates and the
    follower writes. None when no step is braked by an edge and there is no soft start:
    the caller keeps its equal steps of vmax / rate_hz. Setpoint distances only - a path
    is fixed when it is planned; each step is still checked from the measured pose when
    written.

    `soft` (the soft start of a path's first segment, user decision 2026-10-20,
    gripperx_arm.exec_steps.soft_start_fractions): step i < len(soft) covers at most
    soft[i] of a full step (vmax / rate_hz, or less where an edge brakes it); the steps
    after it are full ones as above."""
    plain = vmax / rate_hz
    t, out, braked = 0.0, [], bool(soft)
    goal = {n: qb[n] for n in joints}
    for i in range(1000000):
        cmd = {n: qa[n] + t * (qb[n] - qa[n]) for n in joints}
        sc = step_scale(cmd, goal, edges, vmax, rate_hz, zone, vslow)
        rest = max((abs(goal[n] - cmd[n]) for n in joints), default=0.0)
        if rest > 0.0 and sc < min(1.0, plain / rest) * (1.0 - 1e-9):
            braked = True
        if i < len(soft) and rest > 0.0:
            sc = min(sc, float(soft[i]) * min(1.0, plain / rest))
        t = 1.0 if sc >= 1.0 else t + (1.0 - t) * sc
        out.append(t)
        if t >= 1.0:
            return out if braked else None
    raise ValueError('braked_fractions: no end (slow speed too small)')
