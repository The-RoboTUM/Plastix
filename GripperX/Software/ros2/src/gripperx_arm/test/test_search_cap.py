"""Audit 2026-11-02 M6 and M2, deterministic: load is simulated by a fake clock shared by
the planner and the safety filter (every pose the filter checks advances it), never by
real time, so the outcome does not depend on the machine.

- M6: the operator's I (go_home) was planned in ONE slice of planner.time_budget_s and
  refused NO_PATH when load stretched its checks past it; it now resumes the cut search
  (same checks, same path) up to planner.max_search_s, and Space still cancels at once.
- M2: a leader re-plan search had no wall-clock bound beyond max_checks; it now fails
  closed after planner.max_search_s (the arm holds, the failure is reported, the same
  inputs are not searched again).

- S1 (audit of 3e02e19, re-audit of 4b36ae2): the longer search must not write a path
  whose start the arm has left, or that a changed wheel envelope blocks: I re-reads and
  re-checks before its first write - a measured start by drift, a held-goal start by
  its unchanged goal and the new measured pose as a static start. The automatic return
  home keeps one slice and no re-check.

Not covered here: the real-time overrun of the cap (at most one slice), and a search
whose slices make no progress at all (the existing 'stalled' end, test_leader_hold_resume)."""
import math
import os
import re
import sys
import types
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '221')
sys.path.insert(0, str(Path(__file__).parent))

import pytest  # noqa: E402
import yaml  # noqa: E402
from std_msgs.msg import Empty, String  # noqa: E402

import test_arm_node as tan  # noqa: E402
from test_arm_node import rig_factory  # noqa: E402,F401
from gripperx_arm import arm_action_server as aas  # noqa: E402
from gripperx_arm import planner as planner_mod  # noqa: E402
from gripperx_arm import safety_filter as sf_mod  # noqa: E402
from gripperx_arm.planner import PlanResult  # noqa: E402

PL = yaml.safe_load((tan.PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
    'arm_action_server']['ros__parameters']['planner']
BUDGET, CAP = float(PL['time_budget_s']), float(PL['max_search_s'])
SLICES = 2.5         # the load simulated for I: its uncut search takes 2.5 slices


class _Clock:
    """time.monotonic for planner and safety filter: the filter's deadline check (once per
    pose) advances it by `step`, the planner only reads it."""

    def __init__(self):
        self.t, self.step, self.poses = 1000.0, 0.0, 0
        self.on_pose = None
        self.plans = []                  # one entry per loaded (not calibrating) plan()

    def read(self):
        return self.t

    def tick(self):
        self.poses += 1
        self.t += self.step
        if self.on_pose is not None:
            self.on_pose(self.poses)
        return self.t


def _loaded(monkeypatch, node):
    """Every planner the node builds first counts the poses of its search uncut (clock
    standing), then runs it with the clock stepped so that the same search takes SLICES
    slices of planner.time_budget_s - load, without real time."""
    clock = _Clock()
    monkeypatch.setattr(planner_mod, 'time', types.SimpleNamespace(monotonic=clock.read))
    monkeypatch.setattr(sf_mod, 'time', types.SimpleNamespace(monotonic=clock.tick))
    real = node._planner
    seen = []

    def planner(*a, **kw):
        p = real(*a, **kw)
        plan = p.plan

        def counted(qa, qb, env=None):
            clock.plans.append(clock.poses)
            return plan(qa, qb, env)

        def calibrated(qa, qb, env=None):
            p.plan = counted
            if not p._cancelled:                 # a Space before the search: nothing to time
                clock.step, clock.poses = 0.0, 0
                assert plan(qa, qb, env).ok, 'the uncut search must find a path'
                p._memo_inputs = None            # the loaded search starts from nothing
                clock.step = SLICES * p.budget / clock.poses
            seen.append(clock.poses)
            clock.poses = 0
            return counted(qa, qb, env)
        p.plan = calibrated
        return p
    node._planner = planner
    return clock, seen


def _start(node):
    q = {n: 0.0 for n in node._cal.joints}
    q['arm_limb1_joint'] = 0.4               # base yaw off home, clear of every body
    return q


def _status(r, needle, timeout=60.0):
    return r.wait(lambda: any(needle in t for t in list(r.node.status_log)), timeout)


def test_operator_i_resumes_a_search_cut_by_load(rig_factory, monkeypatch):
    """Before: 'NO_PATH ... checks, 5000 ms of 5000 ms' - the one slice was cut, nothing
    moved. Now the cut search is resumed and the arm goes home on the same checks."""
    assert CAP > SLICES * BUDGET, 'the active cap would end this search: test meaningless'
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    home = {s: node._home_pos[s] for s in aas.LIMB_IDS}
    assert any(r.bus.pos[s] != home[s] for s in home)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: all(r.bus.pos[s] == home[s] for s in home), 60.0), \
        list(node.status_log)[-3:]
    assert seen, 'the go_home planner was not built'
    assert _status(r, 'slices, resumed'), list(node.status_log)[-3:]
    line = next(t for t in node.status_log if 'slices, resumed' in t)
    assert int(re.search(r'in the last of (\d+) slices', line).group(1)) >= 2, line
    assert not any('NO_PATH' in t for t in node.status_log)


def test_space_cancels_the_resumed_i_search_at_once(rig_factory, monkeypatch):
    """Space inside the second slice: the search ends at its next pose, nothing is
    written, the command reports STOPPED (not a NO_PATH, not a hold)."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    at = []

    def on_pose(n):
        if seen and clock.t > 1000.0 + 1.5 * BUDGET and not at:
            at.append(n)
            node._on_stop(Empty())
    clock.on_pose = on_pose
    n_writes = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'stopped (Space) while planning, nothing moved'), \
        list(node.status_log)[-3:]
    assert at and clock.poses <= at[0] + 1           # the pose in progress
    assert len(r.bus.writes) == n_writes
    assert not any('NO_PATH' in t for t in node.status_log)


class _Endless:
    """A leader re-plan search that never ends by itself: every slice is cut by the time
    cap after some progress (load), until 10000 slices end it on 'checks'."""

    def __init__(self, f=None):
        self.budget, self.f = BUDGET, f
        self.fresh_checks, self.fresh_poses, self.calls = 1, 0, 0

    def plan(self, qa, qb, env=None):
        self.calls += 1
        if self.calls > 10000:
            return PlanResult(False, bound='checks', reason='checks used up', cost_ms=1.0)
        return PlanResult(False, bound='time', reason='no path found (time budget)',
                          cost_ms=self.budget * 1e3)

    def cancel(self):
        pass


def test_a_leader_search_ends_at_the_search_cap():
    job = aas._ReplanJob(_Endless(), {}, {}, None, None, CAP)
    job._run()
    res = job.result
    assert not res.ok and res.bound == 'search_time', (res.bound, res.reason)
    assert job.slices == math.ceil(CAP / BUDGET) and not job.stalled
    assert 'planner.max_search_s' in res.reason


def test_the_node_bounds_its_leader_search_and_holds_on_the_cap(rig_factory):
    """The search the node starts carries the ACTIVE cap; a search ended by it is a
    failed search: reported, the arm holds (no write), the same inputs are not
    searched again, changed ones are."""
    from test_leader_hold_resume import _held, _searching
    r = rig_factory(**{'planner.time_budget_s': 60.0})
    n = _held(r)
    live = _searching(r)
    assert live.max_s == CAP
    n._drop_replan()
    job = aas._ReplanJob(_Endless(n._filter), dict(live.start), dict(live.target), live.env,
                         n._cal, CAP)
    job._run()
    here = dict(r.bus.pos)
    writes = len(r.bus.writes)
    assert n._adopt_replan(job, dict(live.start), n._q_of(here), here, live.env) is False
    assert n._replan_failed is job
    assert any('planner.max_search_s' in t for t in list(n.status_log)[-2:]), \
        list(n.status_log)[-2:]
    assert len(r.bus.writes) == writes
    assert n._same_inputs(job, live.start, live.target, live.env, True)
    moved = dict(live.target)
    moved['arm_limb1_joint'] += 2 * float(n._p('leader.replan_repeat_tolerance_rad'))
    assert not n._same_inputs(job, live.start, moved, live.env, True)


def _move_in_slice_2(r, clock, seen, act):
    done = []

    def on_pose(n):
        if seen and clock.t > 1000.0 + 1.5 * BUDGET and not done:
            done.append(len(r.bus.writes))
            act()
    clock.on_pose = on_pose
    return done


def _limb_writes(r, since):
    return [w for w in r.bus.writes[since:] if w[0] in aas.LIMB_IDS]


def _home(r):
    home = {s: r.node._home_pos[s] for s in aas.LIMB_IDS}
    return r.wait(lambda: all(r.bus.pos[s] == home[s] for s in home), 60.0)


def _measured_start(monkeypatch, r):
    """I starting at the MEASURED pose while limb2 holds a goal 10 ticks away - the
    _home_start branches that start measured (ground recovery off, or the arm at or
    below the ground)."""
    node = r.node
    with node._serial_lock:
        node._goal[2] = r.bus.pos[2] + 10
    monkeypatch.setattr(node, '_home_start', lambda here, *a: (dict(here), {}))


def test_a_measured_start_that_moved_during_the_search_is_refused(rig_factory, monkeypatch):
    """The auditor's S1 reproducer, where it applies: limb2's path starts at its measured
    pose and the arm is pushed 300 ticks while I searches. Before (3e02e19): I ran, its
    first limb2 write the stale start. Now: refused, nothing written, reason reported."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    _measured_start(monkeypatch, r)
    at = _move_in_slice_2(r, clock, seen, lambda: r.bus.pos.__setitem__(2, r.bus.pos[2] + 300))
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'the arm moved while the path was planned'), list(node.status_log)[-3:]
    assert len(clock.plans) >= 2 and at
    assert not _limb_writes(r, at[0])


def test_a_held_goal_changed_during_the_search_is_refused(rig_factory, monkeypatch):
    """A limb whose path starts at its held goal: the goal must be the one the search
    started from."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)

    def bump():
        with node._serial_lock:
            node._goal[3] += 5
    at = _move_in_slice_2(r, clock, seen, bump)
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'a held goal changed while the path was planned'), \
        list(node.status_log)[-3:]
    assert at and not _limb_writes(r, at[0])


@pytest.mark.parametrize('valid', [True, False])
def test_i_right_after_space_with_the_arm_sagging_during_the_search(rig_factory, monkeypatch,
                                                                    valid):
    """Re-audit of 4b36ae2: after Space the torque-on sync sets goal = present and I plans
    from there (held goals); the arm then sinks 23 ticks into its load sag (robot,
    2026-11-05) while I searches. The 2-tick drift test refused that I. Now the held
    goals decide the start: if the sagged pose is a valid static start, the arm goes
    home and its first limb2 setpoint continues from the held goal (no jump); if not,
    refused and nothing written."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    assert node._torque_off_all('test: Space')
    goal = r.bus.pos[2]
    sag = []
    at = _move_in_slice_2(r, clock, seen, lambda: (r.bus.pos.__setitem__(2, goal - 23),
                                                   sag.append(node._q_of(dict(r.bus.pos)))))
    if not valid:
        flt = node._filter
        real_pose, real_self = flt.check_pose, flt.self_contacts

        def sagged(q):
            return bool(sag) and abs(q['arm_limb2_joint'] - sag[0]['arm_limb2_joint']) < 1e-9

        def check_pose(q, *a, **kw):
            if sagged(q):
                return sf_mod.Verdict(False, 'body', 'test: the sagged pose touches the hull')
            return real_pose(q, *a, **kw)

        def self_contacts(q, *a, **kw):
            if sagged(q):
                return sf_mod.Verdict(False, 'body', 'test: the sagged pose touches the hull'), []
            return real_self(q, *a, **kw)
        monkeypatch.setattr(flt, 'check_pose', check_pose)
        monkeypatch.setattr(flt, 'self_contacts', self_contacts)
    r.cmd.publish(String(data='go_home'))
    if valid:
        assert _home(r), list(node.status_log)[-3:]
        w2 = [t for sid, t in _limb_writes(r, at[0]) if sid == 2]
        steps = [abs(b - a) for a, b in zip(w2, w2[1:])]
        assert w2 and abs(w2[0] - goal) <= max(steps, default=0), (goal, w2[:4])
        assert not any('moved while the path was planned' in t for t in node.status_log)
    else:
        assert _status(r, 'the measured pose is no longer a valid start'), \
            list(node.status_log)[-3:]
        assert not _limb_writes(r, at[0])
    assert len(clock.plans) >= 2 and at


def _real_envelope(node, angle=0.0):
    return node._filter.envelope_measured({n: angle for n in node._steer_names},
                                          float(node._p('safety_filter.steer_motion_margin_rad')))


def _blocking(monkeypatch, node, env_fn):
    """check_setpoints fails every segment against the envelope env_fn() names."""
    real = node._filter.check_setpoints
    asked = []

    def check(poses, env, *a, **kw):
        if env is env_fn():
            asked.append(1)
            return sf_mod.Verdict(False, 'body', 'test: a wheel turned into the path', 0)
        return real(poses, env, *a, **kw)
    monkeypatch.setattr(node._filter, 'check_setpoints', check)
    return asked


@pytest.mark.parametrize('blocked', [True, False])
def test_a_changed_envelope_is_checked_again_before_i_writes(rig_factory, monkeypatch,
                                                              blocked):
    """The wheel envelope changes during the search (a new object from _envelope). The
    path is checked again against it: refused if it blocks a segment, else written."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    old, new = node._envelope(), _real_envelope(node)
    assert new is not old
    flag = []
    at = _move_in_slice_2(r, clock, seen, lambda: flag.append(1))
    monkeypatch.setattr(node, '_envelope', lambda: new if flag else old)
    asked = _blocking(monkeypatch, node, lambda: new if blocked and flag else object())
    r.cmd.publish(String(data='go_home'))
    if blocked:
        assert _status(r, 'the wheel envelope changed while the path was planned'), \
            list(node.status_log)[-3:]
        assert not _limb_writes(r, at[0]) and asked
    else:
        assert _home(r), list(node.status_log)[-3:]
        assert _status(r, 'path re-checked against the changed wheel envelope')
    assert len(clock.plans) >= 2


def test_the_envelope_the_search_used_is_the_one_compared(rig_factory, monkeypatch):
    """Re-audit (a): None when I starts, a measured envelope when _plan_arm takes its own,
    None again (wheels stale: swept, more body) before the write. Comparing with what
    _arm_home read (None) skipped the re-check; the search's own envelope is compared."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    state = {'env': None}
    monkeypatch.setattr(node, '_envelope', lambda: state['env'])
    real_plan = node._plan_arm

    def plan_arm(*a, **kw):
        state['env'] = _real_envelope(node)
        return real_plan(*a, **kw)
    monkeypatch.setattr(node, '_plan_arm', plan_arm)
    at = _move_in_slice_2(r, clock, seen, lambda: state.__setitem__('env', None))
    asked = _blocking(monkeypatch, node, lambda: None if at else object())
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'the wheel envelope changed while the path was planned'), \
        list(node.status_log)[-3:]
    assert at and asked and not _limb_writes(r, at[0])


def test_the_automatic_return_home_keeps_one_slice(rig_factory, monkeypatch):
    """Not resumed (SR-17 rule 11/13 unchanged): the same load refuses it NO_PATH after
    ONE slice, and it is not re-checked (no second read path)."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    assert node._claim('abort_home')
    n_writes = len(r.bus.writes)
    try:
        with pytest.raises(aas.MotionRefused, match='NO_PATH'):
            node._arm_home('abort_home (test)')
    finally:
        node._release('abort_home')
    assert len(clock.plans) == 1, clock.plans
    assert len(r.bus.writes) == n_writes


def test_space_before_the_i_search_starts_ends_it_at_once(rig_factory, monkeypatch):
    """Space after the claim, before the planner is registered for Space to cancel:
    the search starts cancelled - reported at once, not after the search (up to
    planner.max_search_s plus one slice)."""
    r = rig_factory(start_q=_start)
    node = r.node
    clock, seen = _loaded(monkeypatch, node)
    loaded = node._planner

    def planner(*a, **kw):
        p = loaded(*a, **kw)
        if node._op_planner is None and not clock.plans:
            node._on_stop(Empty())
        return p
    node._planner = planner
    n_writes = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'stopped (Space) while planning, nothing moved'), \
        list(node.status_log)[-3:]
    assert clock.plans == [0] and clock.poses == 0
    assert len(r.bus.writes) == n_writes
