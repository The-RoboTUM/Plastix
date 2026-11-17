"""Soft start and the leader's ground recovery near the ground (user decisions 2026-10-20;
SR-17 rules 5, 11, 13). Pure Python, no ROS node: the step generators, the planner and
the filter, built from the ACTIVE configuration (arm_safety_filter.yaml, calibration,
robot description with the lidar) - no value is duplicated here.

Robot test 2026-10-20 11:11-11:15: a leader re-plan from the held goals (limb5 15.7-17.7
mm static) was accepted by the planner and refused at its first step, every time, from
the measured pose that had sagged a few ticks below them ("planned step 1 of 14:
REJECTED [ground] at sub-step 0 ... recorded as failed"). The filter checks the start of
a motion grown by its first step's motion bound: a full leader step costs ~6 mm of limb5
clearance there, the automatic return home's first ramp step ~11 mm. The cases below are
the offline reproduction of that band (scratchpad repro 2026-10-20), on the incident
ticks of test_ground_recovery."""
import math
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_ground_recovery import INCIDENT_TICKS, LIMB5  # noqa: E402
from test_lidar_pair import active, flt, urdf_lidar  # noqa: E402,F401
from test_planner import JOINTS, _active_executor  # noqa: E402
from test_self_contact import _setup  # noqa: E402

from gripperx_arm.arm_action_server import HOME_MOVE_MS, MOVE_STEP_HZ  # noqa: E402
from gripperx_arm.exec_steps import (  # noqa: E402
    check_soft_start, exec_poses, exec_poses_at, monotone_cover, ramp_count,
    soft_start_fractions, ticks_at, velocity_count)
from gripperx_arm.leader_band import braked_fractions  # noqa: E402
from gripperx_arm.planner import Planner  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
    'arm_action_server']['ros__parameters']
SOFT = PARAMS['planner']['soft_start_step_fractions']
VMAX = float(PARAMS['leader']['max_joint_velocity_rad_s'])
RATE = float(PARAMS['leader']['rate_hz'])
LIMB2 = 'arm_limb2_joint'


def _ticks(d):
    """The incident ticks with limb2 moved by `d` ticks (+ = down, toward the ground)."""
    t = list(INCIDENT_TICKS)
    t[1] += d
    return t


def _leader_steps(cal):
    """The leader follower's step generator as arm_action_server._planner('leader') builds
    it (edges far away: the band edges are not the subject here)."""
    edges = {n: (-10.0, 10.0) for n in JOINTS}

    def steps(qa, qb, direct, soft=()):
        fr = braked_fractions(qa, qb, JOINTS, edges, VMAX, RATE, 0.0, 0.2, soft)
        if fr is not None:
            return exec_poses_at(cal, qa, qb, fr, dedupe=bool(soft))
        return exec_poses(cal, qa, qb, velocity_count(max(abs(qb[n] - qa[n]) for n in JOINTS),
                                                      VMAX / RATE))

    def first(qa, qb, direct):
        return steps(qa, qb, direct, SOFT)
    return steps, first


def _leader_planner(flt, cal, rise=True):
    steps, first = _leader_steps(cal)
    return Planner(flt, JOINTS, 120.0, exec_steps=steps, exec_first=first, rise_ground=rise)


def _sag(cal, q, ticks):
    """q with limb2 `ticks` further toward the ground (the measured pose of a servo
    sagging under gravity below its goal)."""
    m = dict(q)
    m[LIMB2] += ticks * cal.signs[1] * 2.0 * math.pi / cal.ticks_per_rev
    return m


def _adopted(flt, pl, path):
    """(setpoints, rising entries per step) as arm_action_server._adopt_replan builds them
    from a planned path: the first segment with its soft start, every segment while a
    link still rises re-checked with its trace."""
    seq, rising_at, rising = [path[0]], [], ()
    for i, (a, b) in enumerate(zip(path, path[1:])):
        poses = pl.executed(a, b, False, first=i == 0)
        if i == 0 or rising:
            trace = []
            assert pl.check_segment(a, b, None, first=i == 0, rising=rising,
                                    trace=trace) is None
            assert len(trace) == len(poses)
            rising_at += trace[:-1]
            rising = trace[-1]
        else:
            rising_at += [()] * (len(poses) - 1)
        seq += poses[1:]
    return seq, rising_at


# ---------------------------------------------------------------- the step generators

def test_the_configured_profile_is_valid_and_soft():
    prof = check_soft_start(SOFT)
    assert prof and all(0.0 < p < 1.0 for p in prof), prof
    assert prof == sorted(prof), 'an accelerating profile'


@pytest.mark.parametrize('bad', [[0.0], [-0.1], [1.5], [0.05, 2.0]])
def test_invalid_profiles_are_refused(bad):
    with pytest.raises(ValueError):
        check_soft_start(bad)


@pytest.mark.parametrize('n', [1, 2, 7, 128])
def test_soft_start_fractions_follow_the_profile_then_full_steps(n):
    fr = soft_start_fractions(n, SOFT)
    assert fr[-1] == 1.0 and all(b > a for a, b in zip([0.0] + fr, fr))
    steps = [b - a for a, b in zip([0.0] + fr, fr)]
    k = min(len(SOFT), len(steps))
    for i in range(k - (1 if fr[k - 1] == 1.0 else 0)):
        assert steps[i] == pytest.approx(SOFT[i] / n)
    assert all(s <= 1.0 / n + 1e-12 for s in steps)


def test_ticks_at_drops_repeats_and_keeps_the_end():
    sp = ticks_at({1: 100, 2: 50}, {1: 120, 2: 50}, [0.01, 0.02, 0.5, 1.0], dedupe=True)
    assert sp == [{1: 110, 2: 50}, {1: 120, 2: 50}]
    sp = ticks_at({1: 100}, {1: 120}, [0.01, 0.5, 1.0])
    assert sp == [{1: 100}, {1: 110}, {1: 120}]


def test_leader_soft_start_is_a_fraction_of_a_full_step():
    cal, _leader = _active_executor()
    a, b = cal.to_radians(_ticks(0)), cal.to_radians(_ticks(-80))
    steps, first = _leader_steps(cal)
    soft = first(a, b, True)
    plain = steps(a, b, True)
    full = VMAX / RATE
    d = [max(abs(q[n] - p[n]) for n in JOINTS) for p, q in zip(soft, soft[1:])]
    tick = 2.0 * math.pi / cal.ticks_per_rev
    for i, p in enumerate(SOFT[:3]):
        assert d[i] <= p * full + tick, (i, d[i], p * full)
    assert max(d) <= full + tick
    assert len(soft) > len(plain) and soft[-1] == plain[-1]


# ---------------------------------------------------------------- leader re-plan band

@pytest.mark.parametrize('held', [-16, -14, -12, -10])
@pytest.mark.parametrize('sag', [1, 2, 3])
def test_replan_from_held_goals_executes_from_the_sagged_measured_pose(flt, held, sag):
    """The robot band of 2026-10-20: held limb5 15.7-17.7 mm static, measured 1-3 ticks
    of limb2 below. Before: the planner accepted, the follower refused step 1 from the
    measured pose (accept-then-fail). Now every step of the adopted path passes as the
    follower checks it - normal steps from the measured pose, rising steps as planned
    plus from the measured pose (arm_action_server._step_verdict)."""
    cal, _leader = _active_executor()
    h, g = cal.to_radians(_ticks(held)), cal.to_radians(_ticks(-80))
    assert 0.0155 < flt.ground_heights(h)[LIMB5] < 0.018
    pl = _leader_planner(flt, cal)
    res = pl.plan(h, g)
    assert res.ok, res.reason
    seq, rising_at = _adopted(flt, pl, res.path)
    for k in range(1, len(seq)):
        m, w, rising = _sag(cal, seq[k - 1], sag), seq[k], rising_at[k - 1]
        if rising:
            assert flt.check_setpoints([seq[k - 1], w], rising=rising).ok
            assert flt.check_setpoints([m, w], ground_skip=rising).ok
            assert all(flt.ground_step_bound(m, w, n) > 0.0 for n in rising)
        else:
            v = flt.check_setpoints([m, w])
            assert v.ok, (k, len(seq) - 1, str(v))


@pytest.mark.parametrize('held', [-14, -12, -10])
def test_without_the_soft_start_the_first_step_fails_from_the_measured_pose(flt, held):
    """The defect itself (kills 'soft start not applied to the first segment'): the plain
    full first step from the sagged measured pose is refused where the soft one passes."""
    cal, _leader = _active_executor()
    h, g = cal.to_radians(_ticks(held)), cal.to_radians(_ticks(-80))
    steps, first = _leader_steps(cal)
    m = _sag(cal, h, 2)
    assert not flt.check_setpoints([m, steps(h, g, True)[1]]).ok
    assert flt.check_setpoints([m, first(h, g, True)[1]]).ok


def test_a_start_needing_the_lift_takes_full_steps_not_the_soft_start(flt):
    """Planner.executed: a start inside the margin for a FULL step lifts exactly as the
    operator's I does - a soft start would end the lift after its first tiny step and
    fail a few setpoints later, where no lift may open (offline: no path at all)."""
    cal, _leader = _active_executor()
    h, g = cal.to_radians(_ticks(4)), cal.to_radians(_ticks(-80))   # limb5 11.2 mm
    steps, first = _leader_steps(cal)
    pl = _leader_planner(flt, cal)
    assert pl.executed(h, g, True, first=True) == steps(h, g, True)
    assert _leader_planner(flt, cal, rise=False).executed(h, g, True, first=True) == \
        first(h, g, True)
    clear = cal.to_radians(_ticks(-16))                               # limb5 17.7 mm
    assert pl.executed(clear, g, True, first=True) == first(clear, g, True)


# ---------------------------------------------------------------- the leader's rise

@pytest.mark.parametrize('held', [4, 6, 10])
def test_leader_rise_lifts_one_joint_at_a_time_and_never_descends(flt, held):
    """The deadlock of 2026-10-20 (rule 13 change): held limb5 9.3-11.2 mm, leader target
    upward. Without the rise no path (or a refused start); with it the path lifts limb5
    strictly at every setpoint, one joint per step, until it passes the margin - and
    never lower than at its start anywhere after."""
    cal, _leader = _active_executor()
    h, g = cal.to_radians(_ticks(held)), cal.to_radians(_ticks(-80))
    assert flt.ground_heights(h)[LIMB5] < 0.0115
    if held == 4:                                   # one case: the full search is slow
        assert not _leader_planner(flt, cal, rise=False).plan(h, g).ok
    pl = _leader_planner(flt, cal)
    res = pl.plan(h, g)
    assert res.ok and LIMB5 in res.ground, (res.reason, res.ground)
    seq, rising_at = _adopted(flt, pl, res.path)
    k = next(i for i, r in enumerate(rising_at) if not r)
    assert k >= 1
    h5 = [flt.ground_heights(p)[LIMB5] for p in seq]
    for i in range(1, k + 1):
        assert h5[i] > h5[i - 1], (i, h5[:k + 1])
        assert sum(seq[i][n] != seq[i - 1][n] for n in JOINTS) == 1
    assert min(h5) == h5[0]


def test_leader_rise_is_never_a_sideways_or_downward_path(flt):
    """Rise only ever lifts: from inside the margin a segment at the same height (limb1
    yaw only) or downward is refused - the rising link must rise at every setpoint."""
    cal, _leader = _active_executor()
    pl = _leader_planner(flt, cal)
    h = cal.to_radians(_ticks(6))
    side = dict(h)
    side['arm_limb1_joint'] += 0.3
    assert flt.check_pose(side).ok
    why = pl.check_segment(h, side, None, first=True)
    assert why is not None and 'ground recovery' in why and 'does not rise' in why, why
    down = cal.to_radians(_ticks(7))
    why = pl.check_segment(h, down, None, first=True)
    assert why is not None and '[ground]' in why, why


def test_no_new_rise_can_open_mid_path(flt):
    """Once clear (the rising set empty), a later segment that goes back into the margin
    is checked normally and refused: only the path's start can open the recovery."""
    cal, _leader = _active_executor()
    steps, _first = _leader_steps(cal)
    pl = _leader_planner(flt, cal)
    up, low = cal.to_radians(_ticks(-80)), cal.to_radians(_ticks(4))
    assert pl.check_segment(up, low, None, first=False, rising=()) is not None
    # the same segment from the start of a plan would be a descent: refused as well
    assert pl.check_segment(up, low, None, first=True) is not None


# ---------------------------------------------------------------- the automatic return home

def _home_ramp(flt, d, soft):
    cal, executed, home = _setup(flt)
    m = cal.to_radians(_ticks(d))
    goal = {**m, **{n: home[n] for n in JOINTS}}
    if soft:
        n = ramp_count(HOME_MOVE_MS / 1000.0, MOVE_STEP_HZ)
        return m, exec_poses_at(cal, m, goal, soft_start_fractions(n, SOFT), dedupe=True)
    return m, monotone_cover(executed(m, goal, True), JOINTS, flt.cfg.max_joint_step_rad)


@pytest.mark.parametrize('d', [-24, -16, -10])
def test_soft_start_lets_the_return_home_leave_a_near_ground_pose(flt, d):
    """Measured limb5 15.7-20.4 mm: the plain direct home ramp is refused at sub-step 0
    by its first step's motion bound alone; with the soft start every setpoint passes."""
    m, plain = _home_ramp(flt, d, False)
    v = flt.check_setpoints(plain)
    assert not v.ok and v.rule == 'ground' and v.index == 0, v
    m, soft = _home_ramp(flt, d, True)
    assert flt.check_setpoints(soft).ok


@pytest.mark.parametrize('d', [0, 8])
def test_soft_started_return_home_is_still_refused_where_the_direct_path_is_too_low(flt, d):
    """limb5 12.4 mm and 9.9 mm (statically inside the margin): the direct ramp is still
    refused - no ground recovery for an automatic motion (rule 11/13)."""
    _m, soft = _home_ramp(flt, d, True)
    v = flt.check_setpoints(soft)
    assert not v.ok and v.rule == 'ground', v
