"""Ground-margin recovery (user decision 2026-10-12; SR-17, the operator's go_home only).
On 2026-10-12 leader mode guided the follower down until the filter held a setpoint
with arm_limb5_link 8.5 mm above the ground (< 10 mm margin); with torque on the arm then
sagged to 1.2 mm, and every plan - abort_home and the operator's I with detours - was
refused at sub-step 0. The operator's I may now start inside the margin if the path
first lifts every link failing the ground check: strictly rising at every setpoint until
the normal check passes, certified above the ground between setpoints. Robot
description WITH the lidar (as on the robot), active config and calibration."""
import itertools
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_lidar_pair import active, flt, urdf_lidar  # noqa: E402,F401
from test_planner import JOINTS  # noqa: E402
from test_safety_filter import assert_cut_changes_nothing  # noqa: E402
from test_self_contact import _setup  # noqa: E402

from gripperx_arm.planner import Planner  # noqa: E402
from gripperx_arm.safety_filter import TCP  # noqa: E402

LIMB5 = 'arm_limb5_link'
# The follower's ticks measured when the filter first held (2026-10-12, limb5 at the
# margin), and limb2's gravity sag from there that brings limb5 to ~1.1 mm - the
# incident's 1.2 mm to the nearest whole tick (offline, this model).
INCIDENT_TICKS = [1942, 3049, 1682, 2188, 2146, 1529]
SAG_TICKS = 37
TOUCH_TICKS = 41                       # the same sag 4 ticks further: limb5 below z = 0
GRIPPER = 'arm_gripper_link'
# 2026-10-15 15:51:59: Space in leader mode left the arm limp, it sank onto the floor;
# computed then: arm_gripper_link -4.9 mm, arm_limb5_link -1.2 mm, and I was refused
# ("no ground recovery"). The robot's ticks were not logged: this is an OFFLINE
# reconstruction from the incident ticks above (limb2, limb3, limb4 offsets found by
# search) that gives the same two clearances to 0.1 mm in this model - not a measured pose.
ROBOT_2026_10_15 = {1: 59, 2: 2, 3: 210}


def _sagged(extra=SAG_TICKS):
    from test_planner import _active_executor
    cal, _leader = _active_executor()
    t = list(INCIDENT_TICKS)
    t[1] += extra
    return cal.to_radians(t)


def _robot_case():
    from test_planner import _active_executor
    cal, _leader = _active_executor()
    t = list(INCIDENT_TICKS)
    for i, d in ROBOT_2026_10_15.items():
        t[i] += d
    return cal.to_radians(t)


def _home_of(flt, a):
    _cal, executed, home = _setup(flt)
    return {**a, **{n: home[n] for n in JOINTS}}, executed


def _planner(flt, executed, **kw):
    """The operator's I as the node builds it (recovery and separate_self on)."""
    return Planner(flt, JOINTS, 120.0, exec_steps=executed, recovery=True,
                   separate_self=True, **kw)


def _executed_path(executed, path, method):
    poses = [path[0]]
    for a, b in zip(path, path[1:]):
        poses += executed(a, b, method == 'direct')[1:]
    return poses


def test_the_sagged_pose_is_inside_the_margin_only_by_limb5(flt):
    q = _sagged()
    low = flt.ground_shortfalls(q)
    assert list(low) == [LIMB5] and 0.0 < low[LIMB5] < flt.ground_need(LIMB5), low
    v = flt.check_pose(q)
    assert not v.ok and v.rule == 'ground' and v.detail.startswith(LIMB5), v
    assert flt.check_pose(q, ground_skip={LIMB5}).ok            # nothing else fails


def test_without_the_recovery_i_is_refused_at_the_start(flt):
    """The incident reproduced: the operator's I with detours refuses at the start."""
    a = _sagged()
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed).plan(a, b)
    assert not r.ok and 'start pose rejected' in r.reason and '[ground]' in r.reason, r.reason


def test_i_lifts_limb5_out_monotonically(flt):
    """I with the recovery: a path, whose written setpoints raise limb5 strictly until it
    passes the margin, each step certified above the ground; every segment re-checks."""
    a = _sagged()
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert r.ok and r.method == 'ordered', (r.method, r.reason)
    assert f'{LIMB5} 1.1 mm (grown for the first step' in r.ground, r.ground
    poses = _executed_path(executed, r.path, r.method)
    need = flt.ground_need(LIMB5)
    h = [flt.ground_heights(p)[LIMB5] for p in poses]
    k = next(i for i, x in enumerate(h) if x >= need)
    assert k >= 2, h[:4]
    for i in range(1, k + 1):
        assert h[i] > h[i - 1], (i, h[i - 1], h[i])
        assert sum(poses[i][n] != poses[i - 1][n] for n in JOINTS) == 1, i   # one joint
        assert flt.ground_step_bound(poses[i - 1], poses[i], LIMB5) > 0.0, i
    assert min(h) == h[0]                         # never below its start at any setpoint
    rising = None
    for s, e in zip(r.path, r.path[1:]):
        kw = {'rise_ground': True} if rising is None else {'rising': rising}
        v = flt.check_setpoints(executed(s, e, False), recovery=True, separate_self=True, **kw)
        assert v.ok, v
        rising = v.rising_end


def test_a_lift_cut_midway_keeps_its_verdict(flt):
    """Robot run 2026-11-02 audit: a rising check carries state from pose to pose, so a cut
    one starts over - continued from the cut, the rise rules would simply be gone."""
    a = _sagged()
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, b)
    poses = executed(r.path[0], r.path[1], False)
    cut, whole = assert_cut_changes_nothing(flt, poses, 2, recovery=True, separate_self=True,
                                            rise_ground=True)
    assert whole.ok and whole.rising_start and cut.resume_at == 0


@pytest.mark.parametrize('first', ['down', 'flat'])
def test_a_refused_lift_cut_midway_stays_refused(flt, first):
    a = _sagged()
    if first == 'down':
        mid = dict(a, arm_limb2_joint=a['arm_limb2_joint'] - 0.002)
    else:
        mid = dict(a, arm_limb1_joint=a['arm_limb1_joint'] + 0.01)
    up = dict(mid, arm_limb2_joint=a['arm_limb2_joint'] + 0.01)
    cut, whole = assert_cut_changes_nothing(flt, [a, mid, up], 1, rise_ground=True)
    assert not whole.ok and 'does not rise' in whole.detail and cut.resume_at == 0


def test_the_automatic_return_home_still_refuses(flt):
    """abort_home (direct only, no ground recovery) from the same pose: refused."""
    a = _sagged()
    b, executed = _home_of(flt, a)
    r = Planner(flt, JOINTS, 120.0, exec_steps=executed, recovery=True,
                direct_only=True).plan(a, b)
    assert not r.ok and 'start pose rejected' in r.reason and '[ground]' in r.reason, r.reason


def test_the_direct_ramp_is_refused_while_rising(flt):
    """The 4.267 s direct home ramp raises limb5 at every setpoint, but it moves several
    joints at once: inside its first step's joint box (servos out of step) limb5 could
    reach the ground, and a servo lagging more than one setpoint leaves that box
    (audit r11 F2). Refused: one joint at a time while a link is rising."""
    a = _sagged()
    b, executed = _home_of(flt, a)
    poses = executed(a, b, True)
    h = [flt.ground_heights(p)[LIMB5] for p in poses[:4]]
    assert h[0] < h[1] < h[2] < h[3], h
    assert flt.ground_step_bound(poses[0], poses[1], LIMB5) < 0.0
    v = flt.check_setpoints(poses, recovery=True, separate_self=True, rise_ground=True)
    assert not v.ok and v.rule == 'ground' and 'only one joint may move' in v.detail, v
    assert v.index == 1, v


def test_a_single_joint_step_that_may_reach_the_ground_is_refused(flt):
    """The certificate is enforced: a rising single-joint lift from 0.2 mm is accepted
    while its certified bound is > 0, and refused once the bound is shifted down by the
    start clearance (monkeypatched) - so the rule reads the bound, not only the rise."""
    a = _sagged(40)
    c0 = flt.ground_heights(a)[LIMB5]
    assert 0.0 < c0 < 0.0005
    up = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.0349)
    bound = flt.ground_step_bound(a, up, LIMB5)
    real = flt.ground_step_bound
    try:
        flt.ground_step_bound = lambda qa, qb, n: real(qa, qb, n) - c0
        v = flt.check_setpoints([a, up], rise_ground=True)
    finally:
        del flt.ground_step_bound
    assert 0.0 < bound < c0 and flt.check_setpoints([a, up], rise_ground=True).ok
    assert not v.ok and 'may reach the ground' in v.detail, v


@pytest.mark.parametrize('first', ['down', 'flat'])
def test_a_path_that_does_not_rise_first_is_refused(flt, first):
    """Lowering limb5 first (limb2 sagging further), or holding its height (limb1 yaw
    only), before lifting it: refused at the first such setpoint."""
    a = _sagged()
    if first == 'down':
        mid = dict(a, arm_limb2_joint=a['arm_limb2_joint'] - 0.002)
    else:
        mid = dict(a, arm_limb1_joint=a['arm_limb1_joint'] + 0.01)
    assert flt.ground_heights(mid)[LIMB5] > 0.0
    up = dict(mid, arm_limb2_joint=a['arm_limb2_joint'] + 0.01)
    v = flt.check_setpoints([a, mid, up], rise_ground=True)
    assert not v.ok and v.rule == 'ground' and 'does not rise' in v.detail, v
    lift = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.01)
    assert flt.check_setpoints([a, lift], rise_ground=True).ok    # lifting at once: ok


def test_a_start_at_or_below_the_ground_is_lifted(flt):
    """clearance <= 0 at the start (user decision 2026-10-15, was refused until then):
    the model penetrates the ground plane, the real arm rests on the floor - I lifts it
    out as long as the lift only pulls upward."""
    a = _sagged(TOUCH_TICKS)
    assert flt.ground_heights(a)[LIMB5] <= 0.0
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert r.ok and r.method == 'ordered', r.reason
    up = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.01)
    assert flt.check_setpoints([a, up], rise_ground=True).ok


def test_the_held_incident_pose_is_a_recovery_start(flt):
    """Audit r11 F1: the follower where leader mode left it (limb5 12.4 mm, statically
    clear) fails the ground check at sub-step 0 of every path, which is grown for the
    first step: without the recovery I is refused there; with it, the grown trigger
    starts the recovery."""
    a = _sagged(0)
    assert not flt.ground_shortfalls(a)
    b, executed = _home_of(flt, a)
    r0 = _planner(flt, executed).plan(a, b)
    assert not r0.ok and 'at sub-step 0' in r0.reason and '[ground]' in r0.reason, r0.reason
    r1 = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert r1.ok and r1.method == 'ordered', r1.reason
    assert 'grown for the first step' in r1.ground and LIMB5 in r1.ground, r1.ground


def test_no_node_later_in_the_path_opens_a_new_recovery(flt):
    """Only the plan's start can trigger: a segment continuing from a node (rising=
    what the previous segment left rising, here nothing) is checked normally."""
    a = _sagged(0)
    lift = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.01)
    assert flt.check_setpoints([a, lift], rise_ground=True).ok
    v = flt.check_setpoints([a, lift], rising=())
    assert not v.ok and v.rule == 'ground' and v.index == 0, v
    v = flt.check_setpoints([a, lift], rising=(LIMB5,))           # continuing: ok
    assert v.ok, v


def test_a_start_outside_the_margin_is_checked_normally(flt):
    """rise_ground from a start clear of the ground: the same verdicts and path."""
    a = _sagged(-80)
    assert not flt.ground_shortfalls(a)
    b, executed = _home_of(flt, a)
    p = executed(a, b, True)
    assert flt.check_setpoints(p).ok == flt.check_setpoints(p, rise_ground=True).ok
    r0 = _planner(flt, executed).plan(a, b)
    r1 = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert (r0.ok, r0.method, r0.path) == (r1.ok, r1.method, r1.path) and r1.ground == ''


def test_once_out_of_the_margin_the_normal_check_holds(flt):
    """Lifted out, then lowered back into the margin: refused by the normal ground
    check - the recovery covers only the leading section."""
    a = _sagged()
    out = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.08)
    assert not flt.ground_shortfalls(out)
    v = flt.check_setpoints([a, out], rise_ground=True)
    assert v.ok, v
    back = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.03)
    assert 0.0 < flt.ground_heights(back)[LIMB5] < flt.ground_need(LIMB5)
    v = flt.check_setpoints([a, out, back], rise_ground=True)
    assert not v.ok and v.rule == 'ground' and v.detail.startswith(f'{LIMB5} box bottom'), v


def test_body_checks_are_not_relaxed(flt):
    """A rising path through an obstacle of the body envelope: refused [body]."""
    from gripperx_arm.kinematics import Primitive
    from gripperx_arm.safety_filter import Envelope
    a = _sagged()
    up = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.03)
    env = flt.envelope_swept
    assert flt.check_setpoints([a, up], env, rise_ground=True).ok
    t = flt.kin.fk(up)['arm_limb4_link']
    box = Primitive('test_obstacle', 'box', t, (0.02, 0.02, 0.02))
    env2 = Envelope(env.groups + [(None, [box])], env.description + ' + obstacle')
    v = flt.check_setpoints([a, up], env2, rise_ground=True)
    assert not v.ok and v.rule == 'body' and 'test_obstacle' in v.detail, v


def test_the_step_bound_is_below_every_sampled_pose_of_the_joint_box(flt):
    """ground_step_bound is a certificate: no configuration inside the step's joint box
    (random samples, and the vertices) is lower."""
    a = _sagged(SAG_TICKS - 20)
    b, executed = _home_of(flt, a)
    poses = executed(a, b, True)
    rng = np.random.default_rng(7)
    names = flt.kin.joint_names
    for qa, qb in list(zip(poses, poses[1:]))[:3]:
        for name in (LIMB5, 'arm_gripper_link', TCP):
            bound = flt.ground_step_bound(qa, qb, name)
            moving = [n for n in names if qa[n] != qb[n]]
            for bits in itertools.product((0, 1), repeat=len(moving)):
                v = dict(qa, **{n: qb[n] for n, on in zip(moving, bits) if on})
                assert flt.ground_heights(v)[name] >= bound - 1e-12
            for _ in range(60):
                t = rng.random(len(names))
                v = {n: qa[n] + s * (qb[n] - qa[n]) for n, s in zip(names, t)}
                assert flt.ground_heights(v)[name] >= bound - 1e-12


@pytest.mark.parametrize('lag', [1, 2, 4, 8, 16])
def test_a_lagging_servo_never_takes_limb5_below_its_start(flt, lag):
    """Audit r11 F2 (probe p5): along the recovery path as executed, one servo lagging
    `lag` setpoints behind the others. While limb5 is rising only one joint moves, so a
    lag keeps it on the already certified way: never below its start clearance."""
    a = _sagged()
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert r.ok, r.reason
    poses = _executed_path(executed, r.path, r.method)
    need = flt.ground_need(LIMB5)
    c0 = flt.ground_heights(poses[0])[LIMB5]
    k_out = next(i for i, p in enumerate(poses) if flt.ground_heights(p)[LIMB5] >= need)
    for j in JOINTS:
        for k in range(len(poses)):
            q = dict(poses[k], **{j: poses[max(0, k - lag)][j]})
            h = flt.ground_heights(q)[LIMB5]
            if k <= k_out:
                assert h >= c0 - 1e-12, (j, k, h)
            assert h > 0.0, (j, k, h)


# ---------------------------------------------------------------- below the ground
# (user decision 2026-10-15: "I should also help 'in' the ground, as long as recovering
# just pulls upward")

def test_the_robot_case_is_reconstructed(flt):
    q = _robot_case()
    h = flt.ground_heights(q)
    assert round(h[GRIPPER] * 1e3, 1) == -4.9 and round(h[LIMB5] * 1e3, 1) == -1.2, h
    low = flt.ground_shortfalls(q)
    assert set(low) == {GRIPPER, LIMB5, TCP}, low
    assert flt.check_pose(q, ground_skip=set(low)).ok          # nothing else fails


def test_the_robot_case_was_refused_before(flt):
    """Without the recovery (automatic return home, P, gateway) the start is refused."""
    a = _robot_case()
    b, executed = _home_of(flt, a)
    for kw in ({}, {'direct_only': True}):
        r = Planner(flt, JOINTS, 120.0, exec_steps=executed, recovery=True, **kw).plan(a, b)
        assert not r.ok and 'start pose rejected' in r.reason and '[ground]' in r.reason, \
            r.reason


def _rising_phase(flt, poses, names):
    """Index of the first pose where every one of `names` passes its margin."""
    return next(i for i, p in enumerate(poses)
                if all(flt.ground_heights(p)[n] >= flt.ground_need(n) for n in names))


def test_i_lifts_the_robot_case_out_of_the_ground(flt):
    """The robot case: a path whose written setpoints move one joint at a time, raise
    every entry in the margin strictly, and - while an entry is at or below the ground -
    never let its exact lowest point sink below the setpoint before, between setpoints
    included. Every segment re-checks as the planner chained it."""
    a = _robot_case()
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert r.ok and r.method == 'ordered', (r.method, r.reason)
    assert f'{GRIPPER} -4.9 mm (grown for the first step' in r.ground, r.ground
    assert f'{LIMB5} -1.2 mm (grown for the first step' in r.ground, r.ground
    poses = _executed_path(executed, r.path, r.method)
    names = (GRIPPER, LIMB5, TCP)
    k_out = _rising_phase(flt, poses, names)
    assert k_out >= 2
    for i in range(1, k_out + 1):
        assert sum(poses[i][n] != poses[i - 1][n] for n in JOINTS) == 1, i
        for n in names:
            h0, h1 = flt.ground_heights(poses[i - 1])[n], flt.ground_heights(poses[i])[n]
            if h0 >= flt.ground_need(n):
                continue
            assert h1 > h0, (i, n, h0, h1)
            if h0 <= 0.0:
                assert flt.ground_step_low(poses[i - 1], poses[i], n) >= h0 - 1e-9, (i, n)
            else:
                assert flt.ground_step_bound(poses[i - 1], poses[i], n) > 0.0, (i, n)
    for n in (GRIPPER, LIMB5):
        h = [flt.ground_heights(p)[n] for p in poses]
        assert min(h) == h[0], n
    rising = None
    for s, e in zip(r.path, r.path[1:]):
        kw = {'rise_ground': True} if rising is None else {'rising': rising}
        v = flt.check_setpoints(executed(s, e, False), recovery=True, separate_self=True, **kw)
        assert v.ok, v
        rising = v.rising_end


def test_the_exact_step_low_is_a_certificate(flt):
    """ground_step_low: no pose of the one-joint step (dense samples, both ends) is lower,
    it is the sampled minimum to sampling accuracy, and never below the interpolation
    bound ground_step_bound (which it replaces below the ground)."""
    a = _robot_case()
    for j, d in (('arm_limb2_joint', 0.3), ('arm_limb2_joint', -0.05), ('arm_limb3_joint', 1.2),
                 ('arm_limb4_joint', -2.0), ('arm_limb4_joint', 0.4), ('arm_limb5_joint', 2.5),
                 ('arm_limb1_joint', 0.7)):
        b = dict(a, **{j: a[j] + d})
        for name in (GRIPPER, LIMB5, TCP, 'arm_limb4_link'):
            low = flt.ground_step_low(a, b, name)
            hs = [flt.ground_heights(dict(a, **{j: a[j] + d * t}))[name]
                  for t in np.linspace(0.0, 1.0, 801)]
            assert low <= min(hs) + 1e-12, (j, d, name)
            assert min(hs) - low < 1e-5, (j, d, name, low, min(hs))
            assert low >= flt.ground_step_bound(a, b, name) - 1e-12, (j, d, name)
    two = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.01,
               arm_limb3_joint=a['arm_limb3_joint'] + 0.01)
    assert flt.ground_step_low(a, two, GRIPPER) is None


def test_sinking_between_setpoints_is_refused_below_the_ground(flt):
    """The exact rule is enforced: the robot case's first lift step is accepted, and
    refused once the exact lows are shifted 1 um down (monkeypatched) - the
    rule reads the exact lowest point, not only the rise at the setpoints."""
    a = _robot_case()
    up = dict(a, arm_limb2_joint=a['arm_limb2_joint'] + 0.01)
    assert flt.check_setpoints([a, up], rise_ground=True).ok
    real = flt.ground_step_corner_lows

    def shifted(qa, qb, n):
        h0, low = real(qa, qb, n)
        return h0, low - 1e-6
    try:
        flt.ground_step_corner_lows = shifted
        v = flt.check_setpoints([a, up], rise_ground=True)
    finally:
        del flt.ground_step_corner_lows
    assert not v.ok and v.rule == 'ground' and 'sinks deeper' in v.detail, v
    assert v.detail.startswith(GRIPPER) or v.detail.startswith(LIMB5), v


def test_a_lift_that_pivots_a_link_into_the_floor_is_refused(flt):
    """Every link in the ground on its own, not the lowest clearance: limb4 raises the
    gripper (the deepest, -4.9 mm) but presses limb5 (-1.2 mm) further into the floor -
    refused, although the minimum clearance rises."""
    a = _robot_case()
    piv = dict(a, arm_limb4_joint=a['arm_limb4_joint'] + 0.01)
    h0, h1 = flt.ground_heights(a), flt.ground_heights(piv)
    assert h1[GRIPPER] > h0[GRIPPER] and h1[LIMB5] < h0[LIMB5]
    assert min(h1[GRIPPER], h1[LIMB5]) > min(h0[GRIPPER], h0[LIMB5])
    v = flt.check_setpoints([a, piv], rise_ground=True)
    assert not v.ok and v.rule == 'ground' and 'does not rise' in v.detail, v


def test_no_upward_only_first_section_is_refused(flt):
    """A goal reached only through first moves that do not lift: limb1 (yaw, the height
    holds) and limb4 (raises the gripper, lowers limb5 and the TCP). Refused at once with
    the reason per first joint - no RRT search, it could not start with one joint."""
    a = _robot_case()
    g = dict(a, arm_limb4_joint=a['arm_limb4_joint'] + 1.0,
             arm_limb1_joint=a['arm_limb1_joint'] + 0.5)
    assert flt.check_pose(g).ok
    _b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, g)
    assert not r.ok and r.bound == 'ground', (r.bound, r.reason)
    assert r.reason.startswith('ground recovery from ') and 'no first section lifts it' \
        in r.reason, r.reason
    assert 'arm_limb1_joint first' in r.reason and 'arm_limb4_joint first' in r.reason, r.reason
    assert 'does not rise' in r.reason
    assert r.checks < 20, r.checks


@pytest.mark.parametrize('lag', [1, 2, 4, 8, 16])
def test_a_lagging_servo_never_takes_the_robot_case_deeper(flt, lag):
    """As the r11 F2 probe, from the robot case: one servo lagging `lag` setpoints behind
    the others never takes the gripper or limb5 below its start clearance while rising."""
    a = _robot_case()
    b, executed = _home_of(flt, a)
    r = _planner(flt, executed, rise_ground=True).plan(a, b)
    assert r.ok, r.reason
    poses = _executed_path(executed, r.path, r.method)
    k_out = _rising_phase(flt, poses, (GRIPPER, LIMB5, TCP))
    for n in (GRIPPER, LIMB5):
        c0 = flt.ground_heights(poses[0])[n]
        for j in JOINTS:
            for k in range(k_out + 1):
                q = dict(poses[k], **{j: poses[max(0, k - lag)][j]})
                assert flt.ground_heights(q)[n] >= c0 - 1e-12, (n, j, k)


def _heel_case():
    """Offline pose (incident ticks, limb2 +48, limb4 +175, limb5 -400) where limb4
    +0.005 rad raises the gripper link's lowest corner and keeps every other entry
    rising, while another gripper corner, also in the ground, sinks (audit GR2 L1)."""
    from test_planner import _active_executor
    cal, _leader = _active_executor()
    t = list(INCIDENT_TICKS)
    t[1] += 48
    t[3] += 175
    t[4] -= 400
    a = cal.to_radians(t)
    return a, dict(a, arm_limb4_joint=a['arm_limb4_joint'] + 0.005)


def test_a_heel_corner_sinking_into_the_floor_is_refused(flt):
    """Audit GR2 L1: per corner, not per link - the link's lowest point rises (the old
    per-link rule passes), but a corner at or below 0 goes lower: refused."""
    a, b = _heel_case()
    h0, low = flt.ground_step_corner_lows(a, b, GRIPPER)
    assert flt.ground_heights(b)[GRIPPER] > flt.ground_heights(a)[GRIPPER]
    assert low.min() >= h0.min() - 1e-9                        # link-level rule passes
    assert ((h0 <= 0.0) & (low < h0 - 1e-9)).any()             # a corner in the ground sinks
    v = flt.check_setpoints([a, b], rise_ground=True)
    assert not v.ok and v.rule == 'ground' and v.detail.startswith(GRIPPER), v
    assert 'a corner in the ground sinks deeper' in v.detail, v


def test_the_per_corner_lows_are_a_certificate(flt):
    """ground_step_corner_lows: per corner, its height at qa and no sampled pose of its
    own path lower than its exact low (dense samples), to sampling accuracy."""
    from gripperx_arm.kinematics import as_box
    from gripperx_arm.safety_filter import _CORNERS
    a = _robot_case()

    def corners(q, link):
        fr = flt.kin.fk(q)
        nrm, c = flt._ground
        return np.concatenate([((_CORNERS * h) @ f[:3, :3].T + f[:3, 3]) @ nrm + c
                               for p in flt.kin.collision_primitives(frames=fr)
                               if p.link == link for f, h in [as_box(p)]])
    for j, d in (('arm_limb2_joint', 0.3), ('arm_limb4_joint', -1.5), ('arm_limb5_joint', 2.0)):
        b = dict(a, **{j: a[j] + d})
        for link in (GRIPPER, LIMB5):
            h0, low = flt.ground_step_corner_lows(a, b, link)
            assert np.allclose(h0, corners(a, link), atol=1e-12)
            hs = np.array([corners(dict(a, **{j: a[j] + d * t}), link)
                           for t in np.linspace(0.0, 1.0, 401)])
            assert (low <= hs.min(axis=0) + 1e-12).all(), (j, link)
            assert (hs.min(axis=0) - low < 1e-5).all(), (j, link)
