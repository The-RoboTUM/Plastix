"""Self-contact recovery (user decision 2026-10-09, option C, arm self pairs only;
SR-17 rules 12/13). With limb3 at its lower mechanical stop the model's convex hulls
of limb1 and limb3 overlap: every plan from there was refused. The operator's I may
start there if the pair separates; chassis, wheel and ground contacts never relax.
Model, active config and active calibration."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_safety_filter import active, assert_cut_changes_nothing, build, urdf  # noqa: E402,F401
from test_planner import JOINTS, _active_executor  # noqa: E402
from test_limit_recovery import MEASURED_REST_TICKS  # noqa: E402

from gripperx_arm.planner import Planner  # noqa: E402

PAIR = ('arm_limb1_link', 'arm_limb3_link')
LIMB3_STOP = 'arm_limb3_joint'


@pytest.fixture(scope='module')
def flt(urdf, active):
    return build(urdf, active)


def _setup(flt):
    """(cal, executed-steps of the node's timed home ramp, every setpoint; home)."""
    import yaml
    from gripperx_arm.arm_action_server import ArmActionServer, HOME_MOVE_MS, MOVE_STEP_HZ
    from gripperx_arm.exec_steps import exec_poses, ramp_count
    cal, _leader = _active_executor()
    pkg = Path(__file__).resolve().parent.parent
    v = float(yaml.safe_load((pkg / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['planner']['max_joint_velocity_rad_s'])

    def executed(qa, qb, direct):
        return exec_poses(cal, qa, qb, ramp_count(ArmActionServer._ramp_seconds(
            qa, qb, HOME_MOVE_MS if direct else None, v, cal), MOVE_STEP_HZ))
    home = cal.to_radians([int(round(z)) for z in cal.zero_ticks])
    return cal, executed, home


def _at_stop(flt):
    """The measured resting pose of 2026-10-09 with limb3 at its lower mechanical stop
    (geometry.yaml arm_joint_stop_lower)."""
    from gripperx_geometry import constants as C
    cal, executed, home = _setup(flt)
    q = cal.to_radians(MEASURED_REST_TICKS)
    q[LIMB3_STOP] = C.ARM_JOINT_STOP_LOWER[2]
    return q, executed, home


def test_the_resting_pose_at_limb3s_stop_is_a_self_contact(flt):
    q, _x, _h = _at_stop(flt)
    v, hits = flt.self_contacts(q, limits=flt.recovery_limits(q))
    assert v.rule == 'self' and hits == [PAIR], (v, hits)
    lo, hi = flt.pair_clearance(q, *PAIR)
    assert hi < 0 and hi - lo < 1e-6                     # hulls overlap, tight bounds


def test_go_home_separates_the_pair_monotonically(flt):
    """I from the contact: the direct path, and along every written setpoint the pair's
    clearance never decreases while it is below the normal self check."""
    a, executed, home = _at_stop(flt)
    b = {**a, **{n: home[n] for n in JOINTS}}
    old = Planner(flt, JOINTS, 60.0, exec_steps=executed, recovery=True).plan(a, b)
    assert not old.ok and 'start pose rejected' in old.reason, old.reason
    r = Planner(flt, JOINTS, 60.0, exec_steps=executed, recovery=True,
                separate_self=True).plan(a, b)
    assert r.ok and r.method == 'direct', r.reason
    assert r.self_contact.startswith('arm_limb1_link vs arm_limb3_link, clearance -0.')
    poses = executed(a, b, True)
    prev = flt.pair_clearance(poses[0], *PAIR)
    below = 0
    for k, p in enumerate(poses[1:], start=1):
        step = {n: abs(p[n] - poses[k - 1][n]) for n in p}
        nh = poses[max(0, k - 1):k + 2]
        _v, hits = flt.self_contacts(p, dq=step, near_poses=nh,
                                     limits=flt.recovery_limits(poses[k - 1]))
        c = flt.pair_clearance(p, *PAIR)
        if PAIR in hits:
            below += 1
            assert c[0] >= prev[1] - 1e-9, (k, prev, c)
        prev = c
    assert below > 5 and prev[0] > 0                      # it separated, over many steps
    assert flt.check_setpoints(poses, recovery=True, separate_self=True).ok


def test_a_separation_cut_midway_keeps_its_verdict(flt):
    """A separating pair is tracked from pose to pose: a cut check starts over. Continued
    from the cut, the pair would meet the normal self check below its margin."""
    a, executed, home = _at_stop(flt)
    poses = executed(a, {**a, **{n: home[n] for n in JOINTS}}, True)
    cut, whole = assert_cut_changes_nothing(flt, poses, 3, recovery=True, separate_self=True)
    assert whole.ok and cut.resume_at == 0


def test_a_path_that_closes_the_pair_is_refused(flt):
    """Same start rule, a motion that brings the pair closer: refused."""
    lo3 = flt.kin.limits[LIMB3_STOP][0]
    a = {n: 0.0 for n in flt.kin.joint_names}
    a.update({'arm_limb4_joint': -0.6565, 'arm_limb2_joint': 0.15, LIMB3_STOP: lo3})
    v = flt.check_setpoints([a, dict(a, arm_limb2_joint=0.159)], separate_self=True)
    assert not v.ok and v.rule == 'self' and 'clearance decreases' in v.detail, v
    assert flt.check_setpoints([a, dict(a, arm_limb2_joint=0.10)], separate_self=True).ok


def test_a_start_touching_the_chassis_is_still_refused(flt):
    """Option C never relaxes chassis, wheel or ground: the contact pose pushed onto
    the chassis (limb2 beyond the model's chassis contact) is refused. Since the body
    check confirms chassis box overlaps on the arm link's hull (2026-10-20), the contact
    must be one of the hull: limb2 pushed until its hull penetrates the chassis box."""
    a, executed, home = _at_stop(flt)
    a['arm_limb2_joint'] = 0.34
    v = flt.check_pose(a, limits=flt.recovery_limits(a))
    assert not v.ok and v.rule == 'body', v
    assert v.detail.startswith('arm_limb2_link vs chassis_link (hull touches or penetrates'), v
    b = {**a, **{n: home[n] for n in JOINTS}}
    r = Planner(flt, JOINTS, 60.0, exec_steps=executed, recovery=True,
                separate_self=True).plan(a, b)
    assert not r.ok and 'start pose rejected' in r.reason and '[body]' in r.reason, r.reason
    v = flt.check_setpoints(executed(a, b, True), recovery=True, separate_self=True)
    assert not v.ok and v.rule == 'body', v


def test_a_pair_clear_at_the_start_is_checked_normally(flt):
    """separate_self from a start without contact: the same verdicts as without."""
    _a, executed, home = _at_stop(flt)
    a = dict(home, arm_limb3_joint=0.1, arm_limb4_joint=-0.6565)
    for b in (home, dict(a, arm_limb4_joint=flt.kin.limits['arm_limb4_joint'][1] + 0.03)):
        p = executed(a, b, True)
        assert flt.check_setpoints(p).ok == flt.check_setpoints(p, separate_self=True).ok
    r0 = Planner(flt, JOINTS, 60.0, exec_steps=executed).plan(a, home)
    r1 = Planner(flt, JOINTS, 60.0, exec_steps=executed, separate_self=True).plan(a, home)
    assert (r0.ok, r0.method, r0.path) == (r1.ok, r1.method, r1.path) and r1.self_contact == ''


def test_signed_distance_bounds_match_the_exact_hull():
    """convex_distance against the exact Minkowski hull (scipy) on random sets."""
    import numpy as np
    from gripperx_arm.convex_distance import _hull_fallback, signed_distance
    rng = np.random.default_rng(3)
    for _ in range(150):
        a = rng.normal(size=(40, 3))
        b = rng.normal(size=(50, 3)) * 0.8 + rng.normal(size=3) * 1.5
        lo, hi = signed_distance(a, b)
        flo, fhi = _hull_fallback(a, b)
        assert lo <= hi + 1e-12 and hi - lo <= 1e-6
        if flo <= 0:
            assert lo - 1e-9 <= flo <= hi + 1e-9          # exact penetration
        else:
            assert hi >= flo - 1e-9 and lo <= fhi + 1e-9  # inside [facet bound, nearest pt]


def test_without_a_dip_allowance_no_separation_is_possible(flt):
    """self_contact_dip_m bounds how far the clearance may dip inside one step; 0 makes
    separation from contact impossible (the bound is > 0 for every real step)."""
    a, executed, home = _at_stop(flt)
    b = {**a, **{n: home[n] for n in JOINTS}}
    keep = flt.cfg.self_contact_dip_m
    try:
        flt.cfg.self_contact_dip_m = 0.0
        v = flt.check_setpoints(executed(a, b, True), recovery=True, separate_self=True)
        assert not v.ok and 'could dip the clearance' in v.detail, v
    finally:
        flt.cfg.self_contact_dip_m = keep


def test_the_rrt_does_not_separate(flt, monkeypatch):
    """Only the direct and joint-ordered stages may start in self-contact: with both
    unavailable, the RRT (normal checks) finds nothing from the contact pose."""
    import gripperx_arm.planner as planner_mod
    monkeypatch.setattr(planner_mod, 'DEFAULT_ORDERS', ())
    a, executed, home = _at_stop(flt)
    b = {**a, **{n: home[n] for n in JOINTS}}

    def no_direct(qa, qb):
        same = all(abs(qa[n] - a[n]) < 1e-12 for n in JOINTS)
        return 'test: no direct' if same and all(abs(qb[n] - b[n]) < 1e-12
                                                 for n in JOINTS) else None
    r = Planner(flt, JOINTS, 120.0, max_checks=80, edge_check=no_direct, recovery=True,
                separate_self=True).plan(a, b)
    assert not r.ok and r.bound == 'checks', r


# ---- audit r8 surviving mutants C6, C12, C13: the separation rules, pinned one by one.
def _first_step(flt):
    """The first written step of I from the contact (one refined step), the pair still
    below the normal self check at its end; and that step's per-joint travel."""
    from gripperx_arm.kinematics import wrap
    a, executed, home = _at_stop(flt)
    b = {**a, **{n: home[n] for n in JOINTS}}
    qs = flt.refine(executed(a, b, True)[:2])
    assert len(qs) == 2
    step = {n: abs(wrap(qs[1][n] - qs[0][n])) for n in flt.kin.joint_names}
    _v, hits = flt.self_contacts(qs[1], dq=step, near_poses=qs,
                                 limits=flt.recovery_limits(qs[0]))
    assert PAIR in hits, hits
    return qs, step


def _at(flt, q, ref):
    return all(abs(q[n] - ref[n]) < 1e-12 for n in flt.kin.joint_names)


def test_the_dip_rule_uses_the_full_step_motion_bound(flt):
    """c_0 - r_1 >= c_0 - dip with the unfavourable bounds: the step is refused for a
    dip just below hi_0 - lo_0 + r_1 and accepted just above it (mutant C12: r halved)."""
    qs, step = _first_step(flt)
    lo0, hi0 = flt.pair_clearance(qs[0], *PAIR)
    r = flt.pair_motion_bound(qs[0], step, *PAIR)
    need = hi0 - lo0 + r
    assert r > hi0 - lo0 >= 0.0     # else 0.75 * need would not separate r from r / 2
    keep = flt.cfg.self_contact_dip_m
    assert keep >= need, (keep, need)                     # the configured dip admits it
    try:
        flt.cfg.self_contact_dip_m = 0.75 * need
        v = flt.check_setpoints(qs, recovery=True, separate_self=True)
        assert not v.ok and 'could dip the clearance' in v.detail, v
        flt.cfg.self_contact_dip_m = 1.25 * need
        assert flt.check_setpoints(qs, recovery=True, separate_self=True).ok
    finally:
        flt.cfg.self_contact_dip_m = keep


def test_the_separation_rules_use_the_unfavourable_bounds(flt, monkeypatch):
    """Each rule must hold for the WORST value inside the certified (lo, hi) bounds
    (mutant C13: the favourable bound). Bounds widened by hand to 0.1 mm so the two
    choices give different verdicts; the motion bound fixed to 0.1 mm."""
    qs, _step = _first_step(flt)
    real = flt.pair_clearance

    def clearance(c0, c1):
        def pc(q, la, lb):
            if (la, lb) != PAIR:
                return real(q, la, lb)
            return c0 if _at(flt, q, qs[0]) else c1
        return pc
    monkeypatch.setattr(flt, 'pair_motion_bound', lambda q, dq, la, lb: 1e-4)
    # non-decreasing: lo_1 below hi_0 -> may have decreased -> refused (dip not binding)
    monkeypatch.setattr(flt.cfg, 'self_contact_dip_m', 1.0)
    monkeypatch.setattr(flt, 'pair_clearance', clearance((-1.0e-3, -0.9e-3), (-0.95e-3, -0.85e-3)))
    v = flt.check_setpoints(qs, recovery=True, separate_self=True)
    assert not v.ok and 'clearance decreases' in v.detail, v
    # dip: lo_0 - r >= hi_0 - dip, i.e. dip >= r + (hi_0 - lo_0) = 0.2 mm
    monkeypatch.setattr(flt, 'pair_clearance', clearance((-1.0e-3, -0.9e-3), (-0.5e-3, -0.4e-3)))
    monkeypatch.setattr(flt.cfg, 'self_contact_dip_m', 1.5e-4)
    v = flt.check_setpoints(qs, recovery=True, separate_self=True)
    assert not v.ok and 'could dip the clearance' in v.detail, v
    monkeypatch.setattr(flt.cfg, 'self_contact_dip_m', 2.5e-4)
    assert flt.check_setpoints(qs, recovery=True, separate_self=True).ok


def test_a_new_pair_below_the_normal_check_is_refused(flt, monkeypatch):
    """Only the pairs failing at the START may separate: a pair that starts failing
    later is refused like any self contact (mutant C6). No pose of the real home path
    shows a second pair, so one is injected at the step's end."""
    from gripperx_arm.safety_filter import Verdict
    qs, _step = _first_step(flt)
    real = flt.self_contacts
    new = ('arm_limb2_link', 'arm_limb5_link')

    def contacts(q, *a, **kw):
        v, hits = real(q, *a, **kw)
        if _at(flt, q, qs[0]):
            return v, hits
        return Verdict(False, 'self', 'test: a new pair'), sorted(set(hits) | {new})
    monkeypatch.setattr(flt, 'self_contacts', contacts)
    v = flt.check_setpoints(qs, recovery=True, separate_self=True)
    assert not v.ok and v.rule == 'self' and v.detail == 'test: a new pair' and v.index == 1, v
