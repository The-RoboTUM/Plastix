"""A hold keeps the last written goal (user decision 2026-10-13, option a; SR-17):
with torque on, every leader exit, refused return home, failed P and refused command
keeps each servo's last goal instead of writing the measured pose - on 2026-10-12 the
measured pose of a servo sagging under load was re-written hold after hold and the arm
went down (limb5 8.5 -> 1.2 mm). A servo further than hold.contact_threshold_ticks from
its goal (contact) is re-anchored, alone. Same rig as test_arm_node, with a servo model
that settles off its goal under load."""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '220')

import pytest  # noqa: E402
from std_msgs.msg import Bool, Empty, String  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_arm_node as tan  # noqa: E402
from test_arm_node import (  # noqa: E402,F401
    ALLOW, GRIP_NEAR_HOME, HOME_D, _send_pick, _status, rig_factory)
from gripperx_arm import arm_action_server as aas  # noqa: E402

LIMB2 = 2
SAG = 12          # ticks; the 2026-10-13 run logged 11..60 ticks of lag under load


class SagBus(tan.MockBus):
    """A servo under load: with torque on, the present position settles `sag` ticks
    off the goal (steady-state error under gravity) - or at `block[sid]` (stopped by
    an obstruction). Torque off: limp, the position stays where it is."""

    def __init__(self, ticks, sag):
        super().__init__(ticks)
        self.sag = dict(sag)
        self.goal = dict(ticks)
        self.block = {}

    def settle(self, sid):
        if self.torque_on[sid]:
            self.pos[sid] = self.block.get(sid, self.goal[sid] - self.sag.get(sid, 0))

    def torque(self, sid, on):
        ok = super().torque(sid, on)
        self.settle(sid)
        return ok

    def read_load(self, sid):
        """A stand-in for Present_Load: proportional to the position error."""
        return self.goal[sid] - self.pos[sid]

    def write_goal(self, sid, ticks):
        with self.lock:
            self.events.append(('goal', sid, int(ticks)))
        self.goal[sid] = int(ticks)
        self.settle(sid)
        return True


@pytest.fixture
def sag(monkeypatch):
    monkeypatch.setattr(tan, 'MockBus', lambda ticks: SagBus(ticks, {LIMB2: SAG}))


def _limbs(d):
    return {sid: d[sid] for sid in aas.LIMB_IDS}


def _holds(node, n=0):
    return [sp for k, sp in list(node.audit)[n:] if k == 'hold']


def _activate_at_goal(r):
    """Leader mode with the leader on the pose the follower HOLDS (its goals): the
    operator asks for no motion, so none may be written."""
    n = r.node
    r.calv.publish(Bool(data=True))
    r.leader_q = n._q_of(dict(r.bus.goal))
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and n.count_publishers('/arm/leader/joint_states') == 1
                  and n.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - n._leader_rx < 0.5, 20.0), 'gate inputs missing'
    r.rising_edge()
    assert r.wait(lambda: n._leader_state == aas.LEADER_ACTIVE), n.leader_text


def test_repeated_holds_do_not_lower_the_goal(sag, rig_factory):
    """The ratchet of 2026-10-12: four holds through four different sites - leader
    exit (J), refused return home, P error after its first write, refused command
    after its first write. The limb goals stay where they were written; the old hold
    (goal := measured) lowered limb2 by the sag at every one of them."""
    r = rig_factory(**ALLOW, drop=('poses.grip',), **GRIP_NEAR_HOME,
                    **{'abort.home_grace_sec': 60.0})
    node, bus = r.node, r.bus
    g0 = _limbs(bus.goal)
    assert bus.pos[LIMB2] == g0[LIMB2] - SAG, 'the servo model does not sag'

    # 1. leader exit (J): _leave -> _hold
    _activate_at_goal(r)
    time.sleep(0.5)
    assert r.wait(lambda: _limbs(bus.goal) == g0, 5.0), bus.goal
    n = len(node.audit)
    r.enable.publish(Bool(data=False))
    assert _status(r, 'HOLD: leader mode off: enable withdrawn', 5.0), \
        list(node.status_log)[-3:]
    assert _holds(node, n) == [{}] and _limbs(bus.goal) == g0, (_holds(node, n), bus.goal)
    assert f'goal - measured {{{LIMB2}: {SAG}}}' in [
        t for t in node.status_log if t.startswith('HOLD: leader mode off')][-1]

    # 2. return home after the abort refused: _pending_home_tick -> _hold
    def refuse(*_a, **_kw):
        raise aas.MotionRefused('test: no path home')
    node._arm_home = refuse
    with node._state_lock:
        node._pending_home = (time.monotonic() - 1.0, 'test')
    assert _status(r, 'HOLD: return home after abort NOT possible', 5.0), \
        list(node.status_log)[-3:]
    assert _limbs(bus.goal) == g0, bus.goal

    # 3. P: an error after its first write (the gripper opened) -> _hold
    def boom(*_a, **_kw):
        raise RuntimeError('injected')
    node._revalidated_segments = boom
    handle = _send_pick(r)
    assert handle.accepted
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    assert 'Error after motion started' in res.result().result.message
    assert _limbs(bus.goal) == g0, bus.goal

    # 4. a command refused after it wrote -> _hold
    def write_then_refuse(*_a, **_kw):
        with node._serial_lock:
            node._write('gripper_unchecked', {aas.GRIPPER_ID: bus.goal[aas.GRIPPER_ID]})
        raise aas.MotionRefused('test: refused after the first write')
    node._move_gripper = write_then_refuse
    r.cmd.publish(String(data='open_gripper'))
    assert _status(r, 'HOLD: arm/command open_gripper refused after motion started', 5.0), \
        list(node.status_log)[-3:]
    assert _limbs(bus.goal) == g0, bus.goal
    assert bus.pos[LIMB2] == g0[LIMB2] - SAG
    assert all(sp == {} for sp in _holds(node)), _holds(node)


def test_leader_activation_on_a_held_arm_continues_from_its_goals(sag, rig_factory):
    """Leader mode on a held arm (torque on, sagging servo) starts commanded at the
    held goals: with the leader on that pose nothing is written - from the measured
    pose its first steps would lower the goal toward the sag."""
    r = rig_factory(**ALLOW)
    node, bus = r.node, r.bus
    assert node._goal == bus.goal
    _activate_at_goal(r)
    time.sleep(0.5)
    assert 'approve' not in r.kinds(), 'the held leader activation moved the arm'
    assert node._cmd_q == pytest.approx(node._q_of(dict(bus.goal)))
    assert any('leader activation: starts at the held goals' in t for t in node.status_log)


def test_only_the_blocked_servo_is_re_anchored(sag, rig_factory):
    """A servo stopped by an obstruction further than the contact threshold from its
    goal stops pushing (goal := present); a servo just at the threshold and the
    sagging one keep their goals. The threshold is the active configuration's."""
    r = rig_factory()
    node, bus = r.node, r.bus
    thr = node._contact_ticks
    assert thr == {p.name: p.value for p in tan.active_params()}['hold.contact_threshold_ticks']
    g0 = dict(bus.goal)
    bus.block = {4: g0[4] + thr + 1, 3: g0[3] + thr}         # limb3 exactly at the threshold
    for sid in bus.block:
        bus.settle(sid)
    n = len(node.audit)
    node._hold('test')
    assert _holds(node, n) == [{4: g0[4] + thr + 1}], _holds(node, n)
    assert bus.goal == {**g0, 4: g0[4] + thr + 1}
    text = node.status_log[-1]
    assert f'4 (present - goal +{thr + 1} ticks > {thr}: contact/obstruction' in text, text
    assert 'last goal kept on servo(s) [1, 2, 3, 5, 6]' in text, text


def test_kept_goals_invalid_under_the_current_envelope_hold_measured(sag, rig_factory):
    """A wheel steered into the held pose since its goals were approved: they are not
    kept - every servo is held at its measured position (the pre-2026-10-13 hold)."""
    from gripperx_arm.safety_filter import Verdict
    r = rig_factory()
    node, bus, flt = r.node, r.bus, r.node._filter
    blocked = object()
    real = flt.check_pose

    def check_pose(q, envelope=None, *a, **kw):
        if envelope is blocked:
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real(q, envelope, *a, **kw)
    flt.check_pose = check_pose
    node._envelope = lambda: blocked
    measured = dict(bus.pos)
    n = len(node.audit)
    node._hold('test')
    assert _holds(node, n) == [measured], _holds(node, n)
    assert 'kept goals NOT valid' in node.status_log[-1], node.status_log[-1]


def test_operator_i_from_a_held_arm_starts_at_its_goals(sag, rig_factory):
    """I from the held arm: the ramp starts at the held goals, so no setpoint lies
    below them on the sagging servo (from the measured pose the first one would)."""
    r = rig_factory()
    node, bus = r.node, r.bus
    g0 = dict(bus.goal)
    n = len(node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: 'gripper_unchecked' in [k for k, _ in list(node.audit)[n:]], 30.0), \
        list(node.status_log)[-4:]
    assert _status(r, f'go_home: starts at the held goals, not the measured pose (goal - '
                   f'measured {{{LIMB2}: {SAG}}} ticks)'), list(node.status_log)[-4:]
    approve = [sp for k, sp in list(node.audit)[n:] if k == 'approve']
    assert approve and all(sp[LIMB2] == g0[LIMB2] for sp in approve), approve[:3]


def test_space_is_unchanged_and_torque_on_after_limp_writes_present(sag, rig_factory):
    """Space while moving: torque off, no hold write, no goal kept. The next torque-on
    writes goal := present first (no valid goal after limp), and a hold after that
    keeps those goals."""
    r = rig_factory(**ALLOW)
    node, bus = r.node, r.bus
    _activate_at_goal(r)
    n = len(node.audit)
    r.stop.publish(Empty())
    assert r.wait(lambda: 'torque_off' in [k for k, _ in list(node.audit)[n:]], 3.0)
    time.sleep(0.3)
    assert 'hold' not in [k for k, _ in list(node.audit)[n:]], list(node.audit)[n:]
    assert all(g is None for g in node._goal.values()), node._goal
    time.sleep(node._grace + 0.5)
    limp = {**bus.pos, LIMB2: bus.pos[LIMB2] - 25}           # the limp arm dropped
    bus.pos.update(limp)
    node._hold('test (limp)')                                 # torque off: writes nothing
    assert _holds(node, n) == [] and bus.goal[LIMB2] != limp[LIMB2]
    k = len(bus.events)
    node._ensure_torque()
    ev = bus.events[k:]
    t = next(i for i, e in enumerate(ev) if e[0] == 'torque')
    assert sorted((e[1], e[2]) for e in ev[:t]) == sorted(limp.items())
    assert node._goal == limp
    m = len(node.audit)
    node._hold('test (after the torque-on)')
    assert _holds(node, m) == [{}] and node._goal == limp


# ------------------------------------------------------------------ audit HG1
def _space(r, n):
    k = r.node._stop_count
    r.stop.publish(Empty())
    assert r.wait(lambda: r.node._stop_count > k and (
        'TORQUE OFF' in r.node.status_log[-1] or 'no arm action' in r.node.status_log[-1]),
        3.0), list(r.node.status_log)[-3:]
    time.sleep(0.2)                    # the status is the handler's last step but one
    return [k for k, _ in list(r.node.audit)[n:]]


@pytest.mark.parametrize('sid, off, limp', [
    (3, aas.POS_TOLERANCE + 1, True),                  # probe B: limb3 against an obstacle
    (aas.GRIPPER_ID, -(aas.POS_TOLERANCE + 10), True),  # probe D: gripper squeezing
    (3, aas.POS_TOLERANCE, False),                     # at the tolerance: not pushing
])
def test_space_on_a_held_arm_with_a_pushing_servo_goes_limp(sag, rig_factory, sid, off, limp):
    """User decision 2026-10-13 (F1 option a, "Bei Leertaste fuer Servos a"): Space on a
    stationary arm whose servo is more than POS_TOLERANCE from its kept goal turns the
    torque off on every servo, as during motion; the sagging limb2 alone does not."""
    r = rig_factory()
    node, bus = r.node, r.bus
    g = dict(bus.goal)
    bus.block = {sid: g[sid] + off}
    bus.settle(sid)
    node._hold('test: held against an obstruction')
    assert bus.goal == g, 'the hold re-anchored a servo inside the contact threshold'
    assert not node._moving()
    n = len(node.audit)
    kinds = _space(r, n)
    if limp:
        assert kinds == ['torque_off'], kinds
        text = node.status_log[-1]
        assert text.startswith('TORQUE OFF (arm limp): Space on a stationary arm with '
                               'servo(s) pushing') and f'{{{sid}: {off}}}' in text, text
        assert not any(bus.torque_on.values()) and not node._torque_on
        assert all(v is None for v in node._goal.values())
    else:
        assert kinds == [] and 'Space: arm stationary - no arm action' in node.status_log[-1]
        assert all(bus.torque_on.values())


def test_space_while_moving_is_unchanged(sag, rig_factory):
    r = rig_factory(**ALLOW)
    _activate_at_goal(r)
    kinds = _space(r, len(r.node.audit))
    assert kinds == ['torque_off'] and 'Space while the arm was moving' in r.node.status_log[-1]


def test_kept_goals_need_the_measured_pose_inside_the_envelope(sag, rig_factory):
    """Audit HG1 F2 (probe C): a wheel touching the MEASURED pose - the servo still
    moves from there to its kept goal, so the goals are not kept."""
    from gripperx_arm.safety_filter import Verdict
    r = rig_factory()
    node, bus, flt = r.node, r.bus, r.node._filter
    g = dict(bus.goal)
    bus.block = {3: g[3] + 60}
    bus.settle(3)
    mq = node._q_of(dict(bus.pos))
    real = flt.check_pose

    def check_pose(q, envelope=None, *a, **kw):
        if all(abs(q[k] - mq[k]) < 1e-9 for k in mq):
            return Verdict(False, 'body', 'test: wheel touches the measured pose')
        return real(q, envelope, *a, **kw)
    flt.check_pose = check_pose
    measured = dict(bus.pos)
    n = len(node.audit)
    node._hold('test')
    assert _holds(node, n) == [measured], _holds(node, n)
    assert 'measured pose, grown for the gap' in node.status_log[-1], node.status_log[-1]


def test_gripper_goal_is_not_part_of_the_kept_check(sag, rig_factory):
    """Audit HG1 F4: the kept pose is checked with the gripper at its measured value
    (its targets are unchecked, SR-17 rule 12); a squeeze within the contact threshold
    is kept, and the hold logs every servo's present load (F3)."""
    r = rig_factory()
    node, bus, flt = r.node, r.bus, r.node._filter
    g = dict(bus.goal)
    bus.block = {aas.GRIPPER_ID: g[aas.GRIPPER_ID] - 90}
    bus.settle(aas.GRIPPER_ID)
    grip = node._cal.joints[5]
    mg = node._q_of(dict(bus.pos))[grip]
    seen = []
    real = flt.check_pose

    def check_pose(q, *a, **kw):
        seen.append(q[grip])
        return real(q, *a, **kw)
    flt.check_pose = check_pose
    n = len(node.audit)
    node._hold('test')
    assert _holds(node, n) == [{}] and bus.goal == g
    assert seen and all(abs(v - mg) < 1e-12 for v in seen), (seen, mg)
    assert f'present load (likely per mille of max PWM duty, TO-VERIFY) {{1: 0, 2: {SAG}, 3: 0, 4: 0, ' \
        f'5: 0, 6: 90}}' in node.status_log[-1], node.status_log[-1]


def test_space_retries_a_torque_off_that_was_not_acknowledged(sag, rig_factory):
    """Audit HG2 R1 (probe J): a servo that did not acknowledge Space's torque-off may
    still hold torque and push; the next Space retries it although the node counts
    the arm as limp, and says so."""
    r = rig_factory()
    node, bus = r.node, r.bus
    g = dict(bus.goal)
    bus.block = {3: g[3] + 90}
    bus.settle(3)
    real = bus.torque
    bus.torque = lambda sid, on: False if (sid == 3 and not on) else real(sid, on)
    kinds = _space(r, len(node.audit))
    assert kinds == ['torque_off'] and bus.torque_on[3] and not node._torque_on
    assert 'TORQUE-OFF WRITE FAILED' in node.status_log[-1], node.status_log[-1]
    bus.torque = real
    kinds = _space(r, len(node.audit))
    assert kinds == ['torque_off'], kinds
    assert node.status_log[-1].startswith('TORQUE OFF (arm limp): Space: retrying the '
                                          "torque-off of servo(s) {3: 'not acknowledged'}"), \
        node.status_log[-1]
    assert not any(bus.torque_on.values()) and node._torque_stuck == {}
    kinds = _space(r, len(node.audit))                        # all off now: nothing to do
    assert kinds == [] and 'no arm action' in node.status_log[-1]
