"""Space always aborts P (user decision 2026-10-12: "Space muss P immer abbrechen. Wenn
P falsch startet und es einen Fehler gibt, muss der E-Stop durch Space wirken."), in
every phase of the pick, error paths included. Each test injects the Space at one
phase and checks: after the Space is handled no goal write and no torque-on, the
torque is off if the arm was claimed, and P ends refused or 'Stopped (Space)'.
Ordering is closed by the stop epoch (/arm/stop_epoch, PickPlastic.stop_epoch)."""
import sys
import threading
import time
from pathlib import Path

import pytest
from std_msgs.msg import Bool, Empty

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_arm_node import (  # noqa: E402,F401
    ALLOW, GRIP_NEAR_HOME, _activate, _send_pick, aas, rig_factory)

PICK = dict(drop=('poses.grip',), **GRIP_NEAR_HOME)


def _mark(r):
    """[(bus event count)] once a Space has been HANDLED (any caller)."""
    node, mark = r.node, []
    real = node._on_stop

    def spy(msg, info=None):
        real(msg, info)
        mark.append(len(r.bus.events))
    node._on_stop = spy
    return mark


def _space(node):
    node._on_stop(Empty())


def _clean_after(r, mark):
    ev = r.bus.events[mark[0]:]
    return [e for e in ev if e[0] == 'goal' or (e[0] == 'torque' and e[2])]


def _result(r, handle, timeout=60.0):
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), timeout)
    return res.result().result


def _settle(r):
    time.sleep(r.node._grace + 1.0)         # past any would-be return home


def _assert_stopped(r, mark, result, claimed=True):
    assert mark, 'Space never handled'
    _settle(r)
    assert _clean_after(r, mark) == [], _clean_after(r, mark)[:6]
    assert not result.success, result.message
    if claimed:
        assert result.message.startswith('Stopped (Space)'), result.message
        assert not any(r.bus.torque_on.values()), 'claimed arm not limp after Space'
    with r.node._state_lock:
        assert r.node._pending_home is None and r.node._motion_owner is None


# ---------------------------------------------------------------- before the claim
def test_a_p_carrying_an_epoch_from_before_a_space_is_rejected(rig_factory):
    """The race of audit r9 finding 2 made deterministic: the sender's epoch is from
    before a Space the node has handled -> the goal is rejected, nothing moves."""
    r = rig_factory(**PICK)
    node = r.node
    assert r.wait(lambda: r.epoch == node._epoch_now(), 20.0)
    old = r.epoch
    time.sleep(node._grace + 0.3)
    mark = _mark(r)
    r.stop.publish(Empty())
    assert r.wait(lambda: mark, 5.0)
    handle = _send_pick(r, epoch=old)
    assert not handle.accepted
    assert any('stop epoch' in t and 'REJECTED' in t for t in list(node.status_log))
    _settle(r)
    assert r.bus.events[mark[0]:] == []
    # the sender learns the new epoch from the latched topic; P with it runs again
    assert r.wait(lambda: r.epoch == node._epoch_now() != old, 10.0), (r.epoch, old)
    result = _result(r, _send_pick(r, epoch=r.epoch))
    assert result.success, result.message


@pytest.mark.parametrize('epoch', [0, -1])
def test_a_p_without_a_valid_epoch_is_rejected(rig_factory, epoch):
    r = rig_factory(**PICK)
    n = len(r.bus.events)
    handle = _send_pick(r, epoch=epoch if epoch >= 0 else r.node._epoch_now() + 1)
    assert not handle.accepted
    time.sleep(0.5)
    assert r.bus.events[n:] == []


@pytest.mark.parametrize('leader', [False, True])
@pytest.mark.parametrize('trial', [0, 1, 2])
def test_p_then_space_back_to_back_never_moves_after_space(rig_factory, leader, trial):
    """Audit r9 probe P1: P sent, Space published at once (before the goal callback).
    Whatever the order at the node: nothing is written or powered after the Space."""
    r = rig_factory(**PICK, **ALLOW)
    node = r.node
    if leader:
        _activate(r)
    else:
        time.sleep(node._grace + 0.3)
    assert r.wait(lambda: r.epoch == node._epoch_now(), 20.0)
    assert r.pick_client.wait_for_server(timeout_sec=60.0)
    mark = _mark(r)
    if leader:
        r.enable.publish(Bool(data=False))
    fut = r.pick_client.send_goal_async(aas.PickPlastic.Goal(execute=False, stop_epoch=r.epoch))
    r.stop.publish(Empty())
    if leader:
        r.enable.publish(Bool(data=False))
    assert r.wait(lambda: fut.done(), 20.0) and r.wait(lambda: mark, 5.0)
    if fut.result().accepted:
        result = _result(r, fut.result())
        assert not result.success, result.message
    _settle(r)
    assert _clean_after(r, mark) == [], _clean_after(r, mark)[:6]


def test_space_while_p_waits_for_leader_off_refuses_it(rig_factory):
    r = rig_factory(**PICK, **ALLOW, **{'leader.command_wait_sec': 3.0})
    _activate(r)
    node = r.node
    handle = _send_pick(r)
    assert handle.accepted
    assert r.wait(lambda: node._cmd_waiting, 10.0)
    mark = _mark(r)
    _space(node)
    result = _result(r, handle)
    assert 'Refused' in result.message and 'Space' in result.message, result.message
    _assert_stopped(r, mark, result, claimed=False)
    assert not any(r.bus.torque_on.values())         # leader mode was moving: limp


# ---------------------------------------------------------------- after the claim
def test_space_during_planning_stops_p(rig_factory):
    r = rig_factory(**PICK)
    node = r.node
    mark = _mark(r)
    real = node._planned_segments

    def planning(*a, **kw):
        if not mark:
            _space(node)
        return real(*a, **kw)
    node._planned_segments = planning
    result = _result(r, _send_pick(r))
    _assert_stopped(r, mark, result)


STEPS = ['pick/open', 'pick/approach', 'pick/grip_close', 'pick/return_home',
         'pick/gripper_home']


@pytest.mark.parametrize('step', STEPS)
def test_space_inside_each_pick_step_stops_p(rig_factory, step):
    """Space after the third written setpoint of one step (the step's ramp asks its
    abort check before every setpoint)."""
    r = rig_factory(**PICK)
    node = r.node
    mark = _mark(r)
    real = node._ramp
    seen = []

    def ramp(segments, source, kind, should_abort=None):
        if source != step:
            return real(segments, source, kind, should_abort)
        calls = [0]

        def check():
            calls[0] += 1
            if calls[0] == 3 and not mark:
                seen.append(source)
                _space(node)
            return should_abort() if should_abort else False
        return real(segments, source, kind, check)
    node._ramp = ramp
    result = _result(r, _send_pick(r))
    assert seen == [step], (seen, result.message)
    _assert_stopped(r, mark, result)


def test_space_during_the_return_home_after_a_cancel_stops_it(rig_factory):
    r = rig_factory(**PICK)
    node = r.node
    handle = _send_pick(r)
    assert handle.accepted
    assert r.wait(lambda: 'approve' in r.kinds(), 30.0)
    handle.cancel_goal_async()
    result = _result(r, handle)
    assert 'Canceled' in result.message, result.message
    n = len(node.audit)
    assert r.wait(lambda: node._motion_owner == 'abort_home'
                  and 'approve' in [k for k, _ in list(node.audit)[n:]], 20.0), \
        list(node.status_log)[-3:]
    mark = _mark(r)
    _space(node)
    _settle(r)
    assert _clean_after(r, mark) == []
    assert not any(r.bus.torque_on.values())
    assert node._motion_owner is None and node._pending_home is None


@pytest.mark.parametrize('kind', ['unexpected', 'refused', 'blocked'])
def test_space_while_p_handles_an_error_stops_it(rig_factory, kind):
    """P's return home fails (an unforeseen exception, a refusal after motion, no
    path home); a Space lands while the error path holds the arm. The hold writes
    nothing (torque off), no return home is scheduled, the result is the stop."""
    r = rig_factory(**PICK)
    node = r.node
    mark = _mark(r)
    exc = {'unexpected': RuntimeError('test: unforeseen'),
           'refused': aas.MotionRefused('test: refused after motion'),
           'blocked': aas.ReturnBlocked('test: no path home')}[kind]
    real_reval, real_hold = node._revalidated_segments, node._hold

    def reval(now, planned, goal, source, *a, **kw):
        if 'return_home' in source:
            raise exc
        return real_reval(now, planned, goal, source, *a, **kw)

    def hold(text):
        if not mark:
            _space(node)
        return real_hold(text)
    node._revalidated_segments = reval
    node._hold = hold
    result = _result(r, _send_pick(r))
    assert 'test:' in result.message, result.message
    _assert_stopped(r, mark, result)


# ---------------------------------------------------------------- orderings (audit r10 L1)
@pytest.mark.parametrize('phase', ['pick_plastic', 'pick_plastic/approach',
                                   'pick_plastic/grip_close', 'pick_plastic/return_home',
                                   'pick_plastic/gripper_home'])
def test_space_between_a_prepare_check_and_its_torque_on_stops_p(rig_factory, phase):
    """Each _prepare of P, with the arm limp (so the torque-on path runs): Space lands
    after _prepare's own check, before _ensure_torque takes the serial lock. Only the
    re-check under that lock catches it (mutant E)."""
    r = rig_factory(**PICK)
    node = r.node
    time.sleep(node._grace + 0.3)
    real_prep, real_ens = node._prepare, node._ensure_torque
    armed, mark = [], _mark(r)

    def prep(source):
        if source == phase and not armed:
            node._torque_off_all('test: limp before ' + source)
            armed.append(source)
        return real_prep(source)

    def ens(*a, **kw):
        if armed and not mark:
            _space(node)
        return real_ens(*a, **kw)
    node._prepare, node._ensure_torque = prep, ens
    result = _result(r, _send_pick(r))
    assert armed == [phase], (armed, result.message)
    _assert_stopped(r, mark, result)


def test_a_prepare_right_after_spaces_torque_off_is_refused(rig_factory):
    """A claimed P's _prepare running the instant Space's torque-off returns, before
    Space has finished: Space must have raised the epoch BEFORE its torque-off, or the
    torque-on goes through (mutant F)."""
    r = rig_factory(**PICK)
    node = r.node
    time.sleep(node._grace + 0.3)
    assert node._claim('pick')
    real_off = node._torque_off_all
    out = {}

    def off(reason, *a, **kw):
        real_off(reason, *a, **kw)
        out['n'] = len(r.bus.events)

        def prep():
            try:
                node._prepare('test/pick_plastic')
                out['prep'] = 'torque-on went through'
            except aas.MotionStopped as exc:
                out['prep'] = repr(exc)
        t = threading.Thread(target=prep)
        t.start()
        t.join()
    node._torque_off_all = off
    node._on_stop(Empty())
    node._release('pick')
    assert 'MotionStopped' in out['prep'], out['prep']
    assert not [e for e in r.bus.events[out['n']:] if e[0] == 'torque' and e[2]]


def test_the_stop_epoch_is_published_reliable_and_latched(rig_factory):
    """RELIABLE + TRANSIENT_LOCAL depth 1: a sender that starts late still gets it, and
    must subscribe the same way (teleop and gateway pin their side)."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    q = rig_factory().node._epoch_pub.qos_profile
    assert (q.reliability, q.durability, q.depth) == (
        ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL, 1)
