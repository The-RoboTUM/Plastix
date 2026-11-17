"""Space ordering against commands and aborts (audit r8 W1/W2, r9 L1), plus the node
pins for audit r8's surviving mutants (W3, D3, C10). Split from test_arm_node.py
(2026-10-12) for its colcon timeout; same mock bus and rig (imported from there)."""
import sys
import threading
import time
from pathlib import Path

import pytest
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Empty, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_arm_node import (  # noqa: E402,F401
    ALLOW, DETERMINISTIC, GRIP_NEAR_HOME, HOME_D, _activate, _approach_done, _at_limb3_stop,
    _command_in_thread, _gate_ready, _send_pick, _status, aas, rig_factory)

# ------------------------------------------------------------------ Space wins (audit r8)
# Space must win over every operator command RECEIVED before it, also one whose
# callback only starts after the Space was handled (still queued). W2: n x I with
# leader mode on, Space 50 ms later - since the 0.3 s command wait, 2 I's sufficed for
# an I to claim the arm and drive home after the Space.
def _space_mark(r):
    """[] that gets (bus event count) appended once a Space has been HANDLED."""
    node, mark = r.node, []
    real = node._on_stop

    def spy(msg, info=None):
        real(msg, info)
        mark.append(len(r.bus.events))
    node._on_stop = spy                  # the subscription looks the method up per call
    return mark


def _refused(node, source):
    return [t for t in list(node.status_log) if t.startswith(f'REFUSED: {source}:')]


def _limb1_off_home(node):
    return dict(node._q_of(HOME_D), arm_limb1_joint=0.2)


@pytest.mark.parametrize('n', [2, 6])
def test_commands_sent_before_space_never_run_after_it(rig_factory, n):
    r = rig_factory(**ALLOW)
    _activate(r)
    node = r.node
    mark = _space_mark(r)
    for _ in range(n):
        r.cmd.publish(String(data='go_home'))
    time.sleep(0.05)
    r.stop.publish(Empty())
    assert r.wait(lambda: mark, 10.0)
    r.wait(lambda: len(_refused(node, 'go_home')) >= n, 30.0)
    time.sleep(1.0)
    after = r.bus.events[mark[0]:]
    assert not [e for e in after if e[0] == 'goal' or (e[0] == 'torque' and e[2])], after[:6]
    assert len(_refused(node, 'go_home')) == n, list(node.status_log)[-n - 2:]


def test_a_command_received_before_space_but_handled_after_it_is_refused(rig_factory):
    """Deterministic W2: the I's callback is held until the Space has been handled,
    i.e. the order an exhausted executor or the DDS queue can produce. The DDS receipt
    stamps still say I came first: refused, nothing written."""
    r = rig_factory(start_q=_limb1_off_home)
    node = r.node
    mark = _space_mark(r)
    real = node._on_command

    def late(msg, info=None):
        r.wait(lambda: mark, 10.0)
        real(msg, info)
    node._on_command = late
    time.sleep(node._grace + 0.3)                  # stationary: Space does nothing else
    start, n = dict(r.bus.pos), len(r.bus.events)
    r.cmd.publish(String(data='go_home'))
    time.sleep(0.1)
    r.stop.publish(Empty())
    assert r.wait(lambda: _refused(node, 'go_home'), 10.0), list(node.status_log)[-3:]
    assert 'Space received after this command' in _refused(node, 'go_home')[0]
    time.sleep(0.5)
    assert r.bus.events[n:] == [] and r.bus.pos == start


def test_a_command_sent_after_space_runs(rig_factory):
    r = rig_factory(start_q=_limb1_off_home)
    node = r.node
    mark = _space_mark(r)
    time.sleep(node._grace + 0.3)
    r.stop.publish(Empty())
    assert r.wait(lambda: mark, 10.0)
    time.sleep(0.05)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), list(node.status_log)[-3:]
    assert not _refused(node, 'go_home')


def test_only_one_command_waits_for_leader_off(rig_factory):
    """W1: every waiting command held an executor thread for the whole wait; 8 of them
    delayed Space by 314 ms. A second command while one waits is refused at once."""
    r = rig_factory(**ALLOW, **{'leader.command_wait_sec': 3.0})
    _activate(r)
    node = r.node
    n = len(node.audit)
    th = _command_in_thread(r, 'open_gripper')
    assert r.wait(lambda: node._cmd_waiting, 5.0)
    t0 = time.monotonic()
    node._on_command(String(data='go_home'))
    assert time.monotonic() - t0 < 3.0
    assert 'already waiting for leader mode' in _refused(node, 'go_home')[0], \
        list(node.status_log)[-3:]
    node._on_stop(Empty())
    th.join(10.0)
    assert not th.is_alive() and not node._cmd_waiting
    assert 'Space while waiting' in _refused(node, 'open_gripper')[0]
    k = [k for k, _ in list(node.audit)[n:]]
    assert 'approve' not in k and 'gripper_unchecked' not in k, k


def test_a_j_edge_received_before_space_but_handled_after_it_does_not_activate(rig_factory):
    r = rig_factory(**ALLOW)
    _gate_ready(r)
    node = r.node
    mark = _space_mark(r)
    real = node._on_leader_enable

    def late(msg, info=None):
        if msg.data:
            r.wait(lambda: mark, 10.0)
        real(msg, info)
    node._on_leader_enable = late
    assert r.wait(lambda: r.tester.count_subscribers('/arm/leader/enable') >= 1, 20.0)
    r.enable.publish(Bool(data=False))
    time.sleep(0.2)
    n = len(r.bus.events)
    r.enable.publish(Bool(data=True))
    time.sleep(0.05)
    r.stop.publish(Empty())
    assert _status(r, 'Space received after the enable'), list(node.status_log)[-3:]
    time.sleep(1.0)
    assert node._leader_state == aas.LEADER_OFF, node.leader_text
    assert not [e for e in r.bus.events[n:] if e[0] == 'goal' or (e[0] == 'torque' and e[2])]


def test_a_pick_accepted_before_space_but_claimed_after_it_is_refused(rig_factory):
    """Phase 'accepted/queued': the execute callback can start much later than the
    acceptance. Claim held until after the Space: the goal's stop epoch is no longer
    the current one -> refused, nothing written."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    node = r.node
    mark = _space_mark(r)
    real = node._claim_after_leader

    def late(*a, **kw):
        r.wait(lambda: mark, 10.0)
        return real(*a, **kw)
    node._claim_after_leader = late
    time.sleep(node._grace + 0.3)
    n = len(r.bus.events)
    handle = _send_pick(r)
    assert handle.accepted
    r.stop.publish(Empty())
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'stop epoch' in msg, msg
    assert r.bus.events[n:] == []


def test_a_waiting_command_does_not_claim_during_the_leader_exit(rig_factory):
    """The leader exit (_leave) holds, then schedules its return home; a command
    waiting for leader off claims only after that (audit r8 mutant W3), and then
    supersedes the return home."""
    r = rig_factory(**ALLOW, **{'leader.command_wait_sec': 3.0})
    _activate(r)
    node = r.node
    real = node._hold
    entered, release = threading.Event(), threading.Event()

    def gated(text):
        if text.startswith('leader mode off'):
            entered.set()
            release.wait(5.0)
        return real(text)
    node._hold = gated
    n = len(node.audit)
    th = _command_in_thread(r, 'open_gripper')
    assert r.wait(lambda: node._cmd_waiting, 5.0)
    j = threading.Thread(target=node._on_leader_enable, args=(Bool(data=False),), daemon=True)
    j.start()
    assert entered.wait(5.0)
    time.sleep(0.5)                                # the command polls every 10 ms
    owner = node._motion_owner
    during = [k for k, _ in list(node.audit)[n:]]
    release.set()
    j.join(10.0)
    th.join(10.0)
    assert owner is None and during == [], (owner, during)
    assert r.bus.pos[aas.GRIPPER_ID] == node._grip_open, list(node.status_log)[-4:]
    with node._state_lock:
        assert node._pending_home is None and node._motion_owner is None
    assert not any('NOT scheduled' in t for t in list(node.status_log))


def test_give_up_while_stationary_keeps_the_torque_and_writes_nothing(rig_factory):
    """Audit r8 G3/D3: spin_guarded gives up while the arm is stationary (held under
    torque): it stays held - going limp would drop it - and nothing is written."""
    r = rig_factory(start_q=_limb1_off_home)
    node = r.node
    r.ex.shutdown()
    r.thread.join(5.0)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.add_node(r.tester)
    r.ex = ex
    out = []

    def main_like():
        try:
            aas.spin_guarded(ex, node)
        except RuntimeError:
            out.append(len(r.bus.events))
    threading.Thread(target=main_like, daemon=True).start()
    time.sleep(node._grace + 0.5)
    assert not node._moving()

    def bad():
        raise RuntimeError('persistent')
    node.create_timer(0.02, bad)
    assert r.wait(lambda: out, 15.0)
    time.sleep(0.5)
    assert all(r.bus.torque_on.values()), 'the stationary arm was dropped'
    assert r.bus.events[out[0]:] == []


def test_self_contact_recovery_off_refuses_the_operator_i(rig_factory):
    """safety_filter.self_contact_recovery: off - the operator's I from the contact is
    refused like every other plan (mutant C10). DETERMINISTIC: a separating plan must
    not be cut short by the wall-clock cap and so look refused."""
    r = rig_factory(start_q=_at_limb3_stop(), **DETERMINISTIC,
                    **{'safety_filter.self_contact_recovery': 'off'})
    node = r.node
    n, start = len(node.audit), dict(r.bus.pos)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: _refused(node, 'go_home')
                  or 'approve' in [k for k, _ in list(node.audit)[n:]], 60.0)
    time.sleep(0.5)
    assert 'approve' not in [k for k, _ in list(node.audit)[n:]] and r.bus.pos == start
    assert 'self' in _refused(node, 'go_home')[0], _refused(node, 'go_home')


# ------------------------------------------------------------------ Space during an abort (audit r9 L1)
# _leave holds, then schedules the return home; a Space handled in between found no
# pending home to cancel and, with the arm still for longer than the grace, called it
# stationary - the return home then re-powered the arm after the Space.
def _after_space(r, mark):
    ev = r.bus.events[mark[0]:]
    return ([e for e in ev if e[0] == 'torque' and e[2]], [e for e in ev if e[0] == 'goal'])


def _claim_spy(node):
    claims, real = [], node._claim

    def spy(owner):
        ok = real(owner)
        claims.append((len(claims), owner, ok))
        return ok
    node._claim = spy
    return claims


@pytest.mark.parametrize('moving', [False, True])
def test_space_during_the_leader_exit_cancels_its_return_home(rig_factory, moving):
    r = rig_factory(**ALLOW)
    _activate(r)
    node = r.node
    time.sleep(node._grace + 0.3)                  # the follower is still
    real = node._hold
    entered, release = threading.Event(), threading.Event()

    def gated(text):
        if text.startswith('leader mode off'):
            entered.set()
            release.wait(5.0)
        return real(text)
    node._hold = gated
    j = threading.Thread(target=node._on_leader_enable, args=(Bool(data=False),), daemon=True)
    j.start()
    assert entered.wait(5.0)
    if moving:
        with node._state_lock:
            node._last_motion = time.monotonic()
    mark = _space_mark(r)
    claims = _claim_spy(node)
    r.stop.publish(Empty())
    assert r.wait(lambda: mark, 5.0)
    assert not any(r.bus.torque_on.values()), 'Space during the leave did not go limp'
    release.set()
    j.join(5.0)
    time.sleep(node._grace + 1.5)                  # past the would-be return home
    ton, goals = _after_space(r, mark)
    assert not ton and not goals, (len(ton), len(goals))
    assert not [c for c in claims if c[2]], claims
    assert node._pending_home is None
    assert _status(r, 'NOT scheduled: Space during the abort'), list(node.status_log)[-3:]


@pytest.mark.parametrize('gap_ms', [0, 5, 20])
def test_teleop_i_then_space_never_moves_after_space(rig_factory, gap_ms):
    """What the keyboard teleop sends: I = (enable False, go_home), Space = (/arm/stop,
    enable False). Nothing may claim the arm after the Space is handled, and nothing
    may turn the torque on again after it."""
    r = rig_factory(**ALLOW)
    _activate(r)
    node = r.node
    time.sleep(0.3)
    mark = _space_mark(r)
    claims, real = [], node._claim

    def spy(owner):
        ok = real(owner)
        claims.append((bool(mark), owner, ok))
        return ok
    node._claim = spy
    r.enable.publish(Bool(data=False))
    r.cmd.publish(String(data='go_home'))
    time.sleep(gap_ms / 1e3)
    r.stop.publish(Empty())
    r.enable.publish(Bool(data=False))
    assert r.wait(lambda: mark, 5.0)
    time.sleep(node._grace + 3.0)
    ton, _goals = _after_space(r, mark)
    assert not [c for c in claims if c[0] and c[2]], claims
    assert not ton, len(ton)


def test_space_between_a_pick_cancel_and_its_return_home_wins(rig_factory):
    """The pick's abort path: canceled -> released -> return home scheduled. A Space
    handled between the release and the scheduling cancels the return home."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    node = r.node
    real = node._release
    released, go = threading.Event(), threading.Event()

    def gated(owner):
        real(owner)
        if owner == 'pick':
            released.set()
            go.wait(5.0)
    node._release = gated
    handle = _send_pick(r)
    assert handle.accepted
    assert r.wait(lambda: 'approve' in r.kinds(), 30.0)
    handle.cancel_goal_async()
    assert released.wait(30.0)
    mark = _space_mark(r)
    r.stop.publish(Empty())
    assert r.wait(lambda: mark, 5.0)
    go.set()
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 20.0)
    time.sleep(node._grace + 1.5)
    ton, goals = _after_space(r, mark)
    assert not ton and not goals, (len(ton), len(goals))
    assert _status(r, 'NOT scheduled: Space during the abort'), list(node.status_log)[-3:]


def test_a_delivery_without_a_receipt_stamp_is_reported_once(rig_factory):
    """No DDS receipt stamp (an rmw without one): the order falls back to the callback
    start - weaker, so it is reported, once."""
    assert aas._rx_time({'received_timestamp': 0})[1] is False
    assert aas._rx_time(None)[1] is False
    assert aas._rx_time({'received_timestamp': time.time_ns()})[1] is True
    r = rig_factory()
    node = r.node
    time.sleep(node._grace + 0.3)
    for _ in range(2):
        node._on_stop(Empty(), {'received_timestamp': 0})
        node._on_command(String(data='no_such_command'), {'received_timestamp': 0})
    lines = [t for t in list(node.status_log) if 'without a DDS receipt stamp' in t]
    assert len(lines) == 1, lines
    node._on_stop(Empty(), None)                   # a direct call is no delivery
    assert len([t for t in list(node.status_log) if 'without a DDS receipt stamp' in t]) == 1


def test_space_between_the_prepare_check_and_the_torque_on_keeps_the_arm_limp(rig_factory):
    """Audit r9b T1: arm limp after an earlier Space; I; a second Space is handled after
    _prepare's own stop check, before _ensure_torque takes the serial lock. The
    torque must stay off and nothing may be written after that Space."""
    r = rig_factory(start_q=lambda n: dict(n._q_of(HOME_D), arm_limb1_joint=0.2))
    node = r.node
    time.sleep(node._grace + 0.3)
    node._torque_off_all('test: limp from an earlier Space')
    real = node._ensure_torque
    fired = []

    def racy(*a, **kw):
        if not fired:
            node._on_stop(Empty())                 # I has claimed: moving -> _stop set
            fired.append(len(r.bus.events))
        return real(*a, **kw)
    node._ensure_torque = racy
    node._on_command(String(data='go_home'))
    time.sleep(node._grace + 1.0)
    assert fired
    after = r.bus.events[fired[0]:]
    assert after == [], after[:8]
    assert not any(r.bus.torque_on.values())
    assert any('STOPPED' in t and 'before the torque-on' in t for t in list(node.status_log)), \
        list(node.status_log)[-3:]
