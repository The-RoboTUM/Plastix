"""Recovery from outside the joint limits (user decision 2026-10-09, option a; SR-17
rules 12/13). On 2026-10-09 the arm rested after a restart with arm_limb2_joint at
+0.2209 rad, beyond its L2 limit, and every plan - go_home included - was refused at
the start. A start outside is now accepted for the go-home motions while every joint
outside moves only back toward its limit and stays inside once there. Model, active
config and active calibration; no constants but the measured resting angle."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_safety_filter import active, build, ik, urdf  # noqa: E402,F401
from test_planner import JOINTS, _active_executor, _timed_executor  # noqa: E402

from gripperx_arm.planner import Planner  # noqa: E402

L2 = 'arm_limb2_joint'
RESTING = 0.2209          # measured on the robot 2026-10-09


@pytest.fixture(scope='module')
def flt(urdf, active):
    return build(urdf, active)


def _home(flt):
    return {n: 0.0 for n in flt.kin.joint_names}           # home = calibration zero


def _resting(flt, **more):
    """limb2 where the arm rested; limb3 opened 0.1 rad (with limb3 at 0 the direct
    path is blocked by the self check, see test_other_checks_still_apply)."""
    q = _home(flt)
    q.update({L2: RESTING, 'arm_limb3_joint': 0.1}, **more)
    return q


def assert_recovers(flt, poses, start):
    """Every joint outside at `start` only moves back toward its limit and stays inside
    once there; every joint inside at `start` stays inside."""
    out = flt.outside_limits(start)
    assert out
    prev = dict(start)
    for p in poses:
        for n, (lo, hi) in flt.kin.limits.items():
            if lo <= prev[n] <= hi:
                assert lo <= p[n] <= hi, (n, prev[n], p[n])
            elif prev[n] > hi:
                assert p[n] <= prev[n] + 1e-12, (n, prev[n], p[n])
            else:
                assert p[n] >= prev[n] - 1e-12, (n, prev[n], p[n])
        prev = p


def test_go_home_from_outside_takes_the_direct_path(flt):
    """(1) The executed setpoints - whole ticks, the node's timed ramp - all recover."""
    cal, _leader = _active_executor()
    checked, executed = _timed_executor(flt, cal)
    a, b = _resting(flt), _home(flt)
    assert flt.outside_limits(a) == {L2: (RESTING, *flt.kin.limits[L2])}
    old = Planner(flt, JOINTS, 60.0, exec_steps=checked).plan(a, b)
    assert not old.ok and 'start pose outside the joint limits' in old.reason
    r = Planner(flt, JOINTS, 60.0, exec_steps=checked, recovery=True,
                direct_only=True).plan(a, b)
    assert r.ok and r.method == 'direct', r.reason
    assert r.recovery == f'{L2} {RESTING:+.4f} rad > {flt.kin.limits[L2][1]:+.4f} rad'
    poses = executed(a, b, True)
    assert len(poses) > 10
    assert_recovers(flt, poses, a)
    assert flt.check_setpoints(poses, recovery=True).ok
    assert not flt.check_setpoints(poses).ok           # without recovery: the limit


def test_moving_further_out_is_refused(flt):
    """(2) limb2 further out, or a joint that starts inside leaving its limit."""
    a = _resting(flt)
    out = dict(a, **{L2: RESTING + 0.02})
    v = flt.check_setpoints([a, out], recovery=True)
    assert not v.ok and v.rule == 'joint_limit' and 'away from its limit' in v.detail, v
    hi4 = flt.kin.limits['arm_limb4_joint'][1]
    leave = dict(a, **{'arm_limb4_joint': hi4 + 0.03})
    v = flt.check_setpoints([a, leave], recovery=True)
    assert not v.ok and v.rule == 'joint_limit' and 'arm_limb4_joint' in v.detail, v
    assert not Planner(flt, JOINTS, 10.0, recovery=True).plan(a, out).ok   # goal outside


def test_overshoot_back_out_is_refused(flt):
    """(3) Back inside, then out again - each step stays below the start value, so
    only the per-step rule catches it (kills 'anchor the relaxed limit at the start')."""
    a = _resting(flt)
    inside = dict(a, **{L2: 0.10})
    back = dict(a, **{L2: 0.20})
    assert flt.check_setpoints([a, inside], recovery=True).ok
    v = flt.check_setpoints([a, inside, back], recovery=True)
    assert not v.ok and v.rule == 'joint_limit' and 'back out' in v.detail, v
    v = flt.check_setpoints([a, dict(a, **{L2: 0.19}), dict(a, **{L2: 0.20})], recovery=True)
    assert not v.ok, v                                  # non-monotone while still outside


def test_other_checks_still_apply(flt):
    """Recovery relaxes the limit only. A start that collides is refused; with limb3
    at 0 the arm rests within the motion bound of limb1 (self check, outside the home
    neighbourhood): the direct path is refused there - inside the limit too - and only
    a detour (operator I) gets out."""
    hit = ik(flt, 0.30, -0.12, -0.12, 'front/elbow-')        # limb vs chassis
    hit['arm_gripper_joint'] = flt.kin.limits['arm_gripper_joint'][0] - 0.05
    assert flt.outside_limits(hit)
    v = flt.check_pose(hit, limits=flt.recovery_limits(hit))
    assert not v.ok and v.rule == 'body', v
    r = Planner(flt, JOINTS, 10.0, recovery=True).plan(hit, _home(flt))
    assert not r.ok and 'start pose rejected' in r.reason, r.reason
    flat = dict(_home(flt), **{L2: RESTING})
    v = flt.check_path(flat, _home(flt), recovery=True)
    assert not v.ok and v.rule == 'self', v
    assert not flt.check_path(dict(flat, **{L2: 0.159}), _home(flt)).ok
    r = Planner(flt, JOINTS, 60.0, recovery=True, direct_only=True).plan(flat, _home(flt))
    assert not r.ok and r.bound == 'direct_only'
    r = Planner(flt, JOINTS, 120.0, recovery=True).plan(flat, _home(flt))
    assert r.ok and r.method != 'direct', r.reason
    poses = [flat]
    for p, q in zip(r.path, r.path[1:]):
        poses += flt.refine([p, q])[1:]
    assert_recovers(flt, poses, flat)


def test_detours_and_rrt_from_outside_recover(flt, monkeypatch):
    """The ordered detours and the RRT (sampling inside the limits only) handle an
    outside start: the ordered stage switched off, the direct segment forbidden."""
    import gripperx_arm.planner as planner_mod
    monkeypatch.setattr(planner_mod, 'DEFAULT_ORDERS', ())
    a, b = _resting(flt), _home(flt)

    def no_direct(qa, qb):
        same = all(abs(qa[n] - a[n]) < 1e-12 for n in JOINTS) and \
            all(abs(qb[n] - b[n]) < 1e-12 for n in JOINTS)
        return 'test: direct forbidden' if same else None
    r = Planner(flt, JOINTS, 120.0, max_checks=2000, edge_check=no_direct,
                recovery=True).plan(a, b)
    assert r.ok and r.method == 'rrt', r.reason
    poses = [a]
    for p, q in zip(r.path, r.path[1:]):
        poses += flt.refine([p, q])[1:]
    assert_recovers(flt, poses, a)


def test_gripper_outside_is_held_and_recovers(flt):
    """The gripper (not planned) keeps its start value along the arm path: allowed,
    it does not move further out."""
    lo = flt.kin.limits['arm_gripper_joint'][0]
    a = _resting(flt, arm_gripper_joint=lo - 0.05)
    assert set(flt.outside_limits(a)) == {L2, 'arm_gripper_joint'}
    r = Planner(flt, JOINTS, 60.0, recovery=True).plan(a, _home(flt))
    assert r.ok and r.method == 'direct', r.reason
    assert 'arm_gripper_joint' in r.recovery
    assert all(p['arm_gripper_joint'] == a['arm_gripper_joint'] for p in r.path)


def test_normal_planning_is_unchanged_by_the_switch(flt):
    """recovery=True from a start inside: the same verdicts as without."""
    a = dict(_home(flt), **{'arm_limb3_joint': 0.1})
    lo3 = flt.kin.limits['arm_limb3_joint'][0]
    for b in (_home(flt), dict(a, **{'arm_limb3_joint': lo3 - 0.05})):
        assert flt.check_path(a, b).ok == flt.check_path(a, b, recovery=True).ok
    r0 = Planner(flt, JOINTS, 60.0).plan(a, _home(flt))
    r1 = Planner(flt, JOINTS, 60.0, recovery=True).plan(a, _home(flt))
    assert (r0.ok, r0.method, r0.path) == (r1.ok, r1.method, r1.path) and r1.recovery == ''


MEASURED_REST_TICKS = [2089, 834, 3071, 1463, 2116, 2651]   # robot, 2026-10-09, gripper opened


@pytest.mark.parametrize('gripper', ['as measured', 'home'])
def test_the_measured_resting_pose_recovers_directly(flt, gripper):
    """The pose the arm rested in on 2026-10-09, through the ACTIVE calibration: I and
    the automatic return home (direct only) both take the direct path home, with the
    node's timed ramp. Fails if a model, config or calibration change strands it."""
    cal, _leader = _active_executor()
    checked, executed = _timed_executor(flt, cal)
    a = cal.to_radians(MEASURED_REST_TICKS)
    home = cal.to_radians([int(round(z)) for z in cal.zero_ticks])
    if gripper == 'home':
        a['arm_gripper_joint'] = home['arm_gripper_joint']
    assert set(flt.outside_limits(a)) == {L2}
    b = {**a, **{n: home[n] for n in JOINTS}}
    for direct_only in (False, True):
        r = Planner(flt, JOINTS, 60.0, exec_steps=checked, recovery=True,
                    direct_only=direct_only).plan(a, b)
        assert r.ok and r.method == 'direct', r.reason
    assert_recovers(flt, executed(a, b, True), a)


def test_a_joint_below_its_lower_limit_moves_back_up(flt):
    """The lower side (audit r7, mutant 'creep down' survived): limb3 below its lower
    limit may only rise; any further fall, even by 1 mrad, is refused."""
    lo3 = flt.kin.limits['arm_limb3_joint'][0]
    a = dict(_home(flt), **{'arm_limb3_joint': lo3 - 0.004, 'arm_limb4_joint': -0.66})
    assert flt.check_setpoints([a, dict(a, **{'arm_limb3_joint': lo3 + 0.002})],
                               recovery=True).ok
    v = flt.check_setpoints([a, dict(a, **{'arm_limb3_joint': lo3 - 0.005})], recovery=True)
    assert not v.ok and v.rule == 'joint_limit' and 'arm_limb3_joint' in v.detail, v


def test_a_joint_inside_may_not_leak_out_by_one_tick(flt):
    """A joint inside its limit at the start stays inside - one servo tick (2 pi /
    4096 rad) beyond it is refused (audit r7, mutant 'inside leaks 0.02' survived)."""
    tick = 2 * 3.141592653589793 / 4096
    hi4 = flt.kin.limits['arm_limb4_joint'][1]
    a = _resting(flt, arm_limb4_joint=hi4 - tick / 2)
    v = flt.check_setpoints([a, dict(a, **{'arm_limb4_joint': hi4 + tick / 2})], recovery=True)
    assert not v.ok and v.rule == 'joint_limit' and 'arm_limb4_joint' in v.detail, v
