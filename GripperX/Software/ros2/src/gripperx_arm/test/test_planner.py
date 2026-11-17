"""Planner: every returned segment passes the filter; detours where the direct path
crosses the body; no path -> no result. Model + active config, no constants."""
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_safety_filter import build, ik  # noqa: E402  (shared fixtures/helpers)
from test_safety_filter import active, urdf  # noqa: E402,F401

from gripperx_arm.planner import Planner  # noqa: E402

JOINTS = ['arm_limb1_joint', 'arm_limb2_joint', 'arm_limb3_joint', 'arm_limb4_joint',
          'arm_limb5_joint']


@pytest.fixture(scope='module')
def flt(urdf, active):
    return build(urdf, active)


def _all_segments_ok(flt, path):
    return all(flt.check_path(a, b).ok for a, b in zip(path, path[1:]))


def test_direct_path_when_clear(flt):
    a = ik(flt, 0.35, 0.10, 0.05, 'front/elbow-')
    b = ik(flt, 0.35, -0.10, 0.05, 'front/elbow-')
    r = Planner(flt, JOINTS, 2.0).plan(a, b)
    assert r.ok and r.method == 'direct' and len(r.path) == 2


@pytest.mark.parametrize('ends', [
    ((0.10, 0.25, 0.0), (0.30, 0.0, 0.05)),          # direct path crosses the chassis
    ((0.10, -0.25, 0.05), (0.35, -0.20, -0.05)),      # direct path crosses a wheel
])
def test_detour_when_the_direct_path_crosses_the_body(flt, ends, capsys):
    a = ik(flt, *ends[0], 'front/elbow-')
    b = ik(flt, *ends[1], 'front/elbow-')
    assert not flt.check_path(a, b).ok
    r = Planner(flt, JOINTS, 120.0).plan(a, b)
    assert r.ok, r.reason
    assert r.method in ('ordered', 'rrt')
    assert r.path[0] == a and all(abs(r.path[-1][n] - b[n]) < 1e-9 for n in JOINTS)
    assert _all_segments_ok(flt, r.path)
    with capsys.disabled():
        print(f'\n[planner, this machine] {r.method}: {len(r.path)} waypoints, '
              f'{r.cost_ms:.0f} ms, {r.checks} path checks')


def test_no_path_means_no_result(flt):
    a = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    b = dict(a)
    b['arm_limb3_joint'] = flt.kin.limits['arm_limb3_joint'][1] + 0.2      # outside limits
    r = Planner(flt, JOINTS, 0.5).plan(a, b)
    assert not r.ok and r.path == [] and 'limits' in r.reason


def test_goal_in_collision_means_no_result(flt):
    a = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    b = ik(flt, 0.30, -0.12, -0.12, 'front/elbow-')
    assert not flt.check_pose(b).ok
    r = Planner(flt, JOINTS, 0.5).plan(a, b)
    assert not r.ok and 'goal' in r.reason


def test_rrt_is_deterministic(flt):
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    p = Planner(flt, JOINTS, 120.0)            # the check budget, not time, binds
    r1, r2 = p.plan(a, b), p.plan(a, b)
    assert r1.ok and r2.ok and r1.method == r2.method
    assert len(r1.path) == len(r2.path) and r1.checks == r2.checks


def test_time_budget_is_a_hard_cap(flt):
    """The whole plan stops at the wall-clock budget (overrun at most one pose check),
    also inside the direct / ordered stages; it fails with bound='time'."""
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    t = time.monotonic()
    for _ in range(10):
        flt.check_pose(a)
    one = (time.monotonic() - t) / 10 * 1e3 * 4          # one GROWN pose check, generous
    for budget in (0.0, 0.03, 0.3):
        r = Planner(flt, JOINTS, budget, max_checks=10 ** 6).plan(a, b)
        assert not r.ok and r.bound == 'time' and r.path == [], r
        assert 'time budget' in r.reason
        assert r.cost_ms <= budget * 1e3 + one + 50, (budget, r.cost_ms, one)


def test_check_bound_is_deterministic_and_reported(flt):
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    r1 = Planner(flt, JOINTS, 600.0, max_checks=14).plan(a, b)
    r2 = Planner(flt, JOINTS, 600.0, max_checks=14).plan(a, b)
    assert not r1.ok and r1.bound == 'checks' and r1.checks == 14
    assert (r2.ok, r2.bound, r2.checks, r2.reason) == (r1.ok, r1.bound, r1.checks, r1.reason)


def test_edge_check_makes_a_segment_blocked(flt):
    """A segment the caller cannot execute (e.g. servo tick wrap) is treated as
    blocked: the planner searches around it, and every returned segment passes it."""
    a = ik(flt, 0.35, 0.10, 0.05, 'front/elbow-')
    b = ik(flt, 0.35, -0.10, 0.05, 'front/elbow-')
    j = 'arm_limb1_joint'
    lo, hi = sorted((a[j], b[j]))
    mid = (lo + hi) / 2

    def no_crossing_of_mid(qa, qb):                    # a 'wrap' between a and b
        return None if (qa[j] - mid) * (qb[j] - mid) > 0 else 'crosses mid'
    r = Planner(flt, JOINTS, 60.0, max_checks=200, edge_check=no_crossing_of_mid).plan(a, b)
    assert not r.ok and 'not executable' in r.reason and 'crosses mid' in r.reason

    def only_direct_blocked(qa, qb):
        direct = all(abs(qa[n] - a[n]) < 1e-12 and abs(qb[n] - b[n]) < 1e-12 for n in JOINTS)
        return 'direct refused' if direct else None
    r = Planner(flt, JOINTS, 60.0, edge_check=only_direct_blocked).plan(a, b)
    assert r.ok and r.method != 'direct'
    assert all(only_direct_blocked(p, q) is None for p, q in zip(r.path, r.path[1:]))
    assert _all_segments_ok(flt, r.path)


def test_a_search_cut_by_the_time_cap_resumes_to_the_same_path(flt, monkeypatch, capsys):
    """Resume: plan() again on the same Planner continues the search where the cap cut
    it (verdicts remembered), ending with the path an uncut search finds, computing no
    check twice. Clock: one unit per path check (machine-independent); a slice of 2.5
    units completes two checks and is cut in the third. Smoothing off in both: a cap
    may cut smoothing short by design (the path as it stands is returned)."""
    from gripperx_arm import planner as planner_mod
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    uncut = Planner(flt, JOINTS, 120.0, shortcut_rounds=0)
    full = uncut.plan(a, b)
    assert full.ok and full.checks > 3, full.reason

    class Clock:
        now = 0.0

        def monotonic(self):
            return self.now
    clock = Clock()

    class Ticking:
        """The real filter; every path check costs one clock unit (deadline applied
        to that unit: a check that ends past the deadline is a cut one)."""
        kin = flt.kin

        def check_pose(self, *args, **kw):
            return flt.check_pose(*args, **kw)

        def check_path(self, qa, qb, env=None, deadline=None):
            clock.now += 1.0
            if deadline is not None and clock.now > deadline:
                return type(full_verdict)(False, rule='timeout')
            return flt.check_path(qa, qb, env)
    full_verdict = flt.check_path(a, a)
    monkeypatch.setattr(planner_mod, 'time', clock)
    p = Planner(Ticking(), JOINTS, 2.5, shortcut_rounds=0)
    calls, fresh = 0, []
    while calls < 4 * full.checks:
        r = p.plan(a, b)
        calls += 1
        fresh.append(p.fresh_checks)
        if r.ok or r.bound != 'time':
            break
    assert r.ok, (r.reason, fresh)
    assert calls > 1 and all(f >= 1 for f in fresh), fresh
    assert r.method == full.method and r.checks == full.checks
    assert [{n: round(q[n], 9) for n in JOINTS} for q in r.path] == \
        [{n: round(q[n], 9) for n in JOINTS} for q in full.path]
    assert sum(fresh) == uncut.fresh_checks, fresh    # every check computed exactly once
    with capsys.disabled():
        print(f'\n[planner, this machine] chassis detour uncut: {full.method}, '
              f'{full.cost_ms:.0f} ms, {full.checks} checks; resumed in {calls} slices')


def test_cancel_ends_a_plan_with_bound_cancelled(flt):
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    p = Planner(flt, JOINTS, 120.0)
    p.cancel()
    r = p.plan(a, b)
    assert not r.ok and r.bound == 'cancelled' and 'cancelled' in r.reason


# ------------------------------------------------------------------ executed steps (2026-10-09)
def _active_executor():
    """The leader follower as configured: its per-step joint limit and the active
    calibration (the rounding to whole ticks), through the shared step generator."""
    import yaml
    from gripperx_arm.exec_steps import exec_poses, velocity_count
    from gripperx_arm.kinematics import TickCalibration
    pkg = Path(__file__).resolve().parent.parent
    c = yaml.safe_load((pkg / 'config' / 'arm_calibration.yaml').read_text())[
        'arm_joint_state_relay']['ros__parameters']
    cal = TickCalibration(list(c['urdf_joints']), [float(v) for v in c['zero_ticks']],
                          [int(v) for v in c['signs']], float(c['ticks_per_rev']),
                          [float(v) for v in c.get('range_center_rad', [0.0] * 6)])
    ld = yaml.safe_load((pkg / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['leader']
    dmax = float(ld['max_joint_velocity_rad_s']) / float(ld['rate_hz'])

    def steps(qa, qb, direct):
        return exec_poses(cal, qa, qb,
                          velocity_count(max(abs(qb[n] - qa[n]) for n in JOINTS), dmax))
    return cal, steps


def _grazing_segments(flt, cal, steps):
    """Segments s -> g that the IDEAL line check approves with g at exactly the
    clearance (bisected onto the chassis to 1e-9 rad), s on the tick grid (as the arm is
    measured) and shorter than one executor step - so the two checks differ ONLY in the
    rounding of g to whole ticks. s itself is walked up to the edge on approved steps. Yields those whose executed setpoints the filter
    rejects (about one candidate in four with the 2026-10-09 model)."""
    from gripperx_arm.exec_steps import unwrapped_ticks

    def snap(q):
        return {**q, **cal.to_radians(unwrapped_ticks(cal, q))}

    def boundary(s, toward):
        lo, hi = 0.0, 1.0
        while hi - lo > 1e-9:
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if flt.check_path(s, at(s, toward, mid)).ok else (lo, mid)
        return lo

    def at(s, toward, t):
        return {n: s[n] + (toward[n] - s[n]) * t for n in s}
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    for k in range(24):
        a = snap(ik(flt, 0.10, 0.25 - 0.005 * (k // 2), 0.0 + 0.01 * (k % 2), 'front/elbow-'))
        inside = next((p for p in flt.subdivide(a, b) if not flt.check_pose(p).ok), None)
        if inside is None or not flt.check_pose(a).ok:
            continue
        s = a
        for _ in range(8):           # walk up to the edge: shorter steps grow the boxes less
            g = at(s, inside, boundary(s, inside))
            travel = max(abs(g[n] - s[n]) for n in JOINTS)
            if len(steps(s, g, False)) == 2 or travel < 0.002:
                break
            s = snap(at(s, g, 1.0 - 0.002 / travel))    # 2 mrad short of the edge
        if len(steps(s, g, False)) == 2 and not flt.check_setpoints(steps(s, g, False)).ok:
            yield s, g


def test_planner_validates_the_executed_tick_steps_not_the_ideal_line(flt):
    """User decision 2026-10-09: the planner checks what the executor WRITES. A segment
    whose ideal line ends at exactly the clearance passes the old line check, but its
    setpoints - whole ticks of the active calibration, at the configured leader step -
    reach into the chassis: the old check approves it as the direct path, the new one
    refuses it, and whatever the new planner returns executes clear step by step.
    Kills 'the planner checks the ideal line again'."""
    cal, steps = _active_executor()
    s, g = next(_grazing_segments(flt, cal, steps), (None, None))
    assert s is not None, 'no grazing segment whose rounded steps collide was found'
    assert flt.check_path(s, g).ok                     # the ideal line: approved
    old = Planner(flt, JOINTS, 120.0).plan(s, g)
    assert old.ok and old.method == 'direct'
    new = Planner(flt, JOINTS, 120.0, max_checks=40, exec_steps=steps)
    why = new.check_segment(s, g)
    assert why is not None and 'REJECTED' in why, why
    r = new.plan(s, g)
    assert not (r.ok and r.method == 'direct'), r
    if r.ok:
        for p, q in zip(r.path, r.path[1:]):
            assert flt.check_setpoints(steps(p, q, False)).ok


def test_executed_steps_start_from_the_rounded_start_pose(flt):
    """The first pose checked is the start AS WRITTEN (rounded to whole ticks): a start
    off the tick grid is rounded before its first step, never assumed exact."""
    from gripperx_arm.exec_steps import unwrapped_ticks
    cal, steps = _active_executor()
    a = ik(flt, 0.35, 0.10, 0.05, 'front/elbow-')
    b = ik(flt, 0.35, -0.10, 0.05, 'front/elbow-')
    poses = steps(a, b, False)
    assert unwrapped_ticks(cal, poses[0]) == unwrapped_ticks(cal, a)
    assert any(abs(poses[0][n] - a[n]) > 1e-9 for n in JOINTS)      # IK pose: off-grid
    assert unwrapped_ticks(cal, poses[-1]) == unwrapped_ticks(cal, b)
    seen = []

    class Spy:
        kin = flt.kin

        def check_pose(self, *a, **kw):
            return flt.check_pose(*a, **kw)

        def check_setpoints(self, ps, env=None, deadline=None, **kw):
            seen.append(ps)
            return flt.check_setpoints(ps, env, deadline, **kw)
    r = Planner(Spy(), JOINTS, 60.0, exec_steps=steps).plan(a, b)
    assert r.ok and r.method == 'direct'
    assert seen and seen[0] == poses


def test_ramp_cover_keeps_executed_poses_and_boxes_every_dropped_one(flt):
    """The timed ramp's check set (exec_steps.monotone_cover): only poses the servos are
    commanded to, first and last included, neighbours at most max_joint_step_rad
    apart, and every dropped setpoint inside the joint box of the kept poses around it
    - the box the filter's motion bound covers."""
    from gripperx_arm.exec_steps import exec_poses, monotone_cover, ramp_count
    cal, _steps = _active_executor()
    a = ik(flt, 0.35, 0.10, 0.05, 'front/elbow-')
    b = ik(flt, 0.35, -0.10, 0.05, 'front/elbow-')
    full = exec_poses(cal, a, b, ramp_count(4.267, 30.0))        # HOME_MOVE_MS at 30 Hz
    step = flt.cfg.max_joint_step_rad
    cover = monotone_cover(full, JOINTS, step)
    assert cover[0] == full[0] and cover[-1] == full[-1] and len(cover) < len(full)
    idx = [full.index(p) for p in cover]                           # executed poses, in order
    assert idx == sorted(idx)
    for (i, p), (j, q) in zip(zip(idx, cover), zip(idx[1:], cover[1:])):
        assert max(abs(q[n] - p[n]) for n in JOINTS) <= step + 1e-12
        for r in full[i + 1:j]:
            assert all(min(p[n], q[n]) <= r[n] <= max(p[n], q[n]) for n in JOINTS)


def _timed_executor(flt, cal):
    """The node's timed ramp as _planner 'ramp' builds it for go_home: a direct segment
    in HOME_MOVE_MS, a detour segment at planner.max_joint_velocity_rad_s (the node's
    _ramp_seconds), MOVE_STEP_HZ steps per second, checked through monotone_cover.
    Returns (checked, executed)."""
    import yaml
    from gripperx_arm.arm_action_server import ArmActionServer, HOME_MOVE_MS, MOVE_STEP_HZ
    from gripperx_arm.exec_steps import exec_poses, monotone_cover, ramp_count
    pkg = Path(__file__).resolve().parent.parent
    v = float(yaml.safe_load((pkg / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['planner']['max_joint_velocity_rad_s'])

    def executed(qa, qb, direct):
        return exec_poses(cal, qa, qb, ramp_count(ArmActionServer._ramp_seconds(
            qa, qb, HOME_MOVE_MS if direct else None, v, cal), MOVE_STEP_HZ))

    def checked(qa, qb, direct):
        return monotone_cover(executed(qa, qb, direct), JOINTS, flt.cfg.max_joint_step_rad)
    return checked, executed


@pytest.mark.parametrize('executor', ['timed', 'leader'])
def test_rrt_path_with_exec_steps_executes_clear(flt, executor, monkeypatch):
    """The RRT stage together with exec_steps (audit of 5762c7f: since the node's
    chassis test resolves via an ordered path, nothing ran RRT with exec_steps). The
    joint-ordered stage is switched off so RRT must find the path; every setpoint the
    executor will WRITE along it - all of them, not only the timed ramp's checked
    subsequence - passes check_pose, and the motion between consecutive ones passes
    check_setpoints."""
    import gripperx_arm.planner as planner_mod
    cal, leader = _active_executor()
    if executor == 'timed':
        checked, executed = _timed_executor(flt, cal)
    else:
        checked = executed = leader
    monkeypatch.setattr(planner_mod, 'DEFAULT_ORDERS', ())
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    assert not flt.check_path(a, b).ok                  # the direct path is blocked
    r = Planner(flt, JOINTS, 600.0, max_checks=2000, exec_steps=checked).plan(a, b)
    assert r.ok and r.method == 'rrt', r.reason
    n = 0
    for p, q in zip(r.path, r.path[1:]):
        poses = executed(p, q, False)
        for s in poses:
            v = flt.check_pose(s)
            assert v.ok, v
        assert flt.check_setpoints(poses).ok
        n += len(poses) - 1
    assert n > len(r.path)                    # several setpoints per segment were checked
