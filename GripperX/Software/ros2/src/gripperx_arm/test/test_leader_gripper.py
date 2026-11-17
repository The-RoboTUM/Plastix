"""The gripper in leader mode (FR-17 amendment, user decisions 2026-11-13): the follower
gripper mirrors the leader gripper like limb1..limb5 - same command band, activation match
tolerance, follow step, stream watchdog. Same rig as test_arm_node (mock bus, discovery
off); every tolerance and margin is read from the node's active parameters.

Since 2026-11-16 (FR-17 amendment, user decisions): a closing block does not end leader
mode; leader.gripper_torque_limit (unset in the shipped config: the servo's own
Torque_Limit, like the limbs, nothing written) is the hook for a measured limit - the tests
of that hook set a test value (LIMIT, an input, not a config value)."""
import os
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '222')

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_arm_node as tan  # noqa: E402
from test_arm_node import ALLOW, rig_factory  # noqa: E402,F401
from gripperx_arm import arm_action_server as aas  # noqa: E402

# Test input for the hook under test; the shipped config leaves it unset.
LIMIT = {'leader.gripper_torque_limit': 300}


def _g(n):
    return n._cal.joints[aas.IDS.index(aas.GRIPPER_ID)]


def test_leader_mode_drives_all_six_joints():
    assert aas.LEADER_IDS == aas.IDS and aas.GRIPPER_ID in aas.LEADER_IDS


def test_follower_gripper_follows_the_leader_gripper(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    g = _g(n)
    tan._activate(r)
    n0 = len(n.audit)
    q = dict(r.leader_q)
    q[g] += 0.2                                        # open the leader gripper
    r.leader_q = q
    assert r.wait(lambda: abs(r.measured_q()[g] - q[g]) < 0.002, 5.0), n.leader_text
    written = [sp for k, sp in list(n.audit)[n0:] if k == 'approve']
    assert written and all(aas.GRIPPER_ID in sp for sp in written)
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text


def test_gripper_mismatch_beyond_the_activation_tolerance_refuses(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    g = _g(n)
    tol = float(n._p('leader.activation_match_tolerance_rad'))
    r.calv.publish(tan.Bool(data=True))
    q = r.measured_q()
    q[g] += tol + 0.05
    r.leader_q = q
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and time.monotonic() - n._leader_rx < 0.5, 20.0)
    r.rising_edge()
    assert tan._text(r, 'does not match', 5.0), n.leader_text
    assert g in n.leader_text and n._leader_state == aas.LEADER_OFF


def test_stream_without_the_gripper_refuses(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    r.leader_omit = (_g(n),)
    r.calv.publish(tan.Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and time.monotonic() - n._leader_rx < 0.5, 20.0)
    r.rising_edge()
    assert tan._text(r, 'lacks', 5.0), n.leader_text
    assert n._leader_state == aas.LEADER_OFF


def test_gripper_beyond_its_band_is_clamped_at_the_band_edge(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    g = _g(n)
    tan._activate(r)
    lo_band = n._leader_band()[g][0]
    lo_limit = n._filter.kin.limits[g][0]
    assert lo_limit < lo_band
    q = dict(r.leader_q)
    q[g] = lo_limit - 0.05                              # leader closed past the limit
    r.leader_q = q
    assert r.wait(lambda: abs(n._commanded()[g] - lo_band) < 1e-9, 8.0), n.leader_text
    assert n._filter.outside_limits(r.measured_q()) == {}
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text


def test_leader_held_start_keeps_the_gripper_goal(rig_factory):
    """Re-plan and activation start from the gripper's held goal like a limb's (the
    gripper's measured value is not a re-anchor for leader mode)."""
    r = rig_factory(**ALLOW)
    n = r.node
    here = dict(r.bus.pos)
    with n._serial_lock:
        n._goal.update({sid: here[sid] for sid in aas.IDS})
        n._goal[aas.GRIPPER_ID] = here[aas.GRIPPER_ID] + 3
    assert n._held_start(here, 'test', report=False,
                         sids=aas.LEADER_IDS)[aas.GRIPPER_ID] == here[aas.GRIPPER_ID] + 3
    assert n._held_start(here, 'test', report=False)[aas.GRIPPER_ID] == here[aas.GRIPPER_ID]


# ------------------------------------------- the leader gripper torque limit (2026-11-16)
TL = aas.ADDR_TORQUE_LIMIT


class GripBlockBus(tan.MockBus):
    """The jaws stopped by an object: a gripper goal beyond `block` on `side` (+1/-1,
    ticks) leaves the servo at `block`."""

    def __init__(self, ticks):
        super().__init__(ticks)
        self.block = None
        self.side = 0

    def write_goal(self, sid, ticks):
        ok = super().write_goal(sid, ticks)
        if (sid == aas.GRIPPER_ID and self.block is not None and self.torque_on[sid]
                and (int(ticks) - self.block) * self.side > 0):
            self.pos[sid] = self.block
        return ok


@pytest.fixture
def block(monkeypatch):
    monkeypatch.setattr(tan, 'MockBus', GripBlockBus)


def _normal(r):
    return r.bus.regs[(aas.GRIPPER_ID, aas.ADDR_MAX_TORQUE_LIMIT)]


def _words(r):
    return [e for e in list(r.bus.events) if e[0] == 'word']


def _grip_writes(n, since=0):
    return [sp for k, sp in list(n.audit)[since:] if k == 'approve' and aas.GRIPPER_ID in sp]


def _close_dir(n):
    """(tick sign, angle sign) of closing, from the active gripper.open/close."""
    s = n._grip_close_sign()
    return s, s * n._cal.signs[aas.IDS.index(aas.GRIPPER_ID)]


def _move_leader_gripper(r, q_g, timeout=8.0):
    n, g = r.node, _g(r.node)
    q = dict(r.leader_q)
    q[g] = q_g
    r.leader_q = q
    assert r.wait(lambda: n._commanded() is not None
                  and abs(n._commanded()[g] - q_g) < 0.002, timeout), n.leader_text


def test_shipped_config_drives_the_gripper_at_the_servos_own_limit(rig_factory):
    """Decision (a) 2026-11-16: unset -> driven like the limbs, and like them with no
    Torque_Limit written (the node writes none for any servo)."""
    r = rig_factory(**ALLOW)                       # the active config, no override
    n = r.node
    g = _g(n)
    assert n._opt('leader.gripper_torque_limit') is None
    tan._activate(r)
    assert n._grip_driven, n.leader_text
    n0 = len(n.audit)
    q = dict(r.leader_q)
    q[g] += 0.2
    r.leader_q = q
    assert r.wait(lambda: abs(r.measured_q()[g] - q[g]) < 0.002, 5.0), n.leader_text
    assert _grip_writes(n, n0)
    r.enable.publish(tan.Bool(data=False))
    assert r.wait(lambda: n._leader_state == aas.LEADER_OFF, 5.0)
    time.sleep(0.3)
    assert not _words(r) and n._grip_limit_prev is None


def test_limit_applied_at_activation_and_read_back(rig_factory):
    r = rig_factory(**ALLOW, **LIMIT)
    n = r.node
    tan._activate(r)
    assert n._grip_driven
    assert r.bus.regs[(aas.GRIPPER_ID, TL)] == n._p('leader.gripper_torque_limit')
    assert n._grip_limit_prev == _normal(r)
    assert {(e[1], e[2]) for e in _words(r)} == {(aas.GRIPPER_ID, TL)}   # servo 6, RAM only


def _exit_j(r):
    r.enable.publish(tan.Bool(data=False))


def _exit_space(r):
    r.stop.publish(tan.Empty())


def _exit_watchdog(r):
    r.heartbeat = False


def _exit_error(r):
    def boom():
        raise RuntimeError('injected')
    r.node._measured_q = boom


def _exit_gateway(r):
    r.arming.publish(tan.ArmingState(armed=True))


@pytest.mark.parametrize('leave', [_exit_j, _exit_space, _exit_watchdog, _exit_error,
                                   _exit_gateway])
def test_limit_restored_on_every_exit(rig_factory, leave):
    r = rig_factory(**ALLOW, **LIMIT)
    n = r.node
    tan._activate(r)
    assert r.bus.regs[(aas.GRIPPER_ID, TL)] != _normal(r)
    leave(r)
    assert r.wait(lambda: n._leader_state == aas.LEADER_OFF, 5.0), n.leader_text
    assert r.wait(lambda: n._grip_limit_prev is None
                  and r.bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r), 5.0), \
        list(n.status_log)[-4:]


def test_limit_restored_when_the_node_gives_up(rig_factory):
    r = rig_factory(**ALLOW, **LIMIT)
    n = r.node
    tan._activate(r)
    n._shut_down_motion('test')
    assert n._grip_limit_prev is None and r.bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r)


def test_limit_restored_by_destroy_node(rig_factory):
    r = rig_factory(**ALLOW, **LIMIT)
    n = r.node
    tan._activate(r)
    r._stream = False
    r.streamer.join(1.0)
    r.ex.shutdown()
    n.destroy_node()
    n.destroy_node = lambda: None                  # the fixture's close
    assert r.bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r)
    ev = list(r.bus.events)
    # torque on: the gripper goal := its present position BEFORE the limit is raised
    last = max(i for i, e in enumerate(ev) if e[0] == 'word' and e[2] == TL)
    assert ev[last - 1][:2] == ('goal', aas.GRIPPER_ID)


def test_refused_activation_restores_the_limit(rig_factory, monkeypatch):
    r = rig_factory(**ALLOW, **LIMIT)
    n = r.node
    monkeypatch.setattr(n, '_activation_overtaken', lambda *a: 'forced')
    r.calv.publish(tan.Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True, 20.0)
    r.rising_edge()
    assert tan._text(r, 'forced', 5.0), n.leader_text
    assert _words(r) and n._grip_limit_prev is None
    assert r.bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r)


@pytest.mark.parametrize('fault', ['write', 'readback', 'read'])
def test_limit_failure_holds_the_gripper(rig_factory, fault):
    r = rig_factory(**ALLOW, **LIMIT)
    n, bus = r.node, r.bus
    limit = n._p('leader.gripper_torque_limit')
    real = bus.write_word
    if fault == 'write':
        bus.write_word = lambda sid, addr, v: False if v == limit else real(sid, addr, v)
    elif fault == 'readback':
        bus.write_word = lambda sid, addr, v: real(sid, addr, v + 1 if v == limit else v)
    else:
        del bus.regs[(aas.GRIPPER_ID, TL)]
    tan._activate(r)
    assert not n._grip_driven, n.leader_text
    assert tan._status(r, 'gripper HELD', 2.0), list(n.status_log)[-3:]
    assert n._grip_limit_prev is None, list(n.status_log)[-3:]   # undone
    if fault == 'readback':
        assert bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r)
    n0 = len(n.audit)
    g0 = bus.pos[aas.GRIPPER_ID]
    q = dict(r.leader_q)
    q[_g(n)] += 0.2
    q['arm_limb1_joint'] += 0.05                   # the limbs still follow
    r.leader_q = q
    assert r.wait(lambda: abs(r.measured_q()['arm_limb1_joint'] - q['arm_limb1_joint'])
                  < 0.002, 5.0), n.leader_text
    assert not _grip_writes(n, n0) and bus.pos[aas.GRIPPER_ID] == g0
    assert n._leader_state == aas.LEADER_ACTIVE


def test_held_gripper_needs_no_gripper_in_the_stream(rig_factory, monkeypatch):
    r = rig_factory(**ALLOW, **LIMIT)
    n = r.node
    monkeypatch.setattr(n, '_apply_grip_limit', lambda: 'forced hold')
    tan._activate(r)
    assert not n._grip_driven
    r.leader_omit = (_g(n),)
    time.sleep(0.5)
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text


def test_open_gripper_restores_the_normal_limit_first(rig_factory):
    r = rig_factory()
    n, bus = r.node, r.bus
    bus.regs[(aas.GRIPPER_ID, TL)] = 1                 # a restore that failed before
    n._grip_limit_prev = _normal(r)
    e0 = len(bus.events)
    start = bus.pos[aas.GRIPPER_ID]
    r.cmd.publish(tan.String(data='open_gripper'))
    assert r.wait(lambda: bus.pos[aas.GRIPPER_ID] == n._grip_open, 5.0)
    assert bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r) and n._grip_limit_prev is None
    ev = list(bus.events)[e0:]
    words = [i for i, e in enumerate(ev) if e[0] == 'word' and e[3] == _normal(r)]
    moves = [i for i, e in enumerate(ev) if e[0] == 'goal' and e[1] == aas.GRIPPER_ID
             and e[2] != start]
    assert words and moves and words[0] < moves[0]


@pytest.mark.parametrize('cmd', ['open_gripper', 'go_home'])
def test_commands_refused_while_the_normal_limit_cannot_be_restored(rig_factory, cmd):
    r = rig_factory()
    n, bus = r.node, r.bus
    bus.regs[(aas.GRIPPER_ID, TL)] = 1
    n._grip_limit_prev = _normal(r)
    bus.write_word = lambda *a: False
    pos0 = dict(bus.pos)
    r.cmd.publish(tan.String(data=cmd))
    assert tan._status(r, 'NOT restored', 5.0)
    assert tan._status(r, 'refused', 5.0)
    time.sleep(0.5)
    assert bus.pos == pos0 and n._grip_limit_prev is not None


class _Leftover(tan.MockBus):
    def __init__(self, ticks):
        super().__init__(ticks)
        self.regs[(aas.GRIPPER_ID, TL)] = 1


def test_leftover_leader_limit_at_start_is_restored_when_the_hook_is_set(monkeypatch,
                                                                       rig_factory):
    monkeypatch.setattr(tan, 'MockBus', _Leftover)
    r = rig_factory(**LIMIT)
    assert r.wait(lambda: r.bus.regs[(aas.GRIPPER_ID, TL)] == _normal(r), 5.0)
    assert r.node._grip_limit_prev is None


def test_no_limit_is_ever_written_while_the_hook_is_unset(monkeypatch, rig_factory):
    monkeypatch.setattr(tan, 'MockBus', _Leftover)
    r = rig_factory()
    time.sleep(1.5)
    assert not _words(r) and r.node._grip_limit_prev is None


@pytest.mark.parametrize('over', [{}, LIMIT])
def test_closing_block_keeps_leader_mode(block, rig_factory, over):
    r = rig_factory(**ALLOW, **over)
    n, bus = r.node, r.bus
    g = _g(n)
    tan._activate(r)
    tol = float(n._p('leader.command_tolerance_rad'))
    tsign, qsign = _close_dir(n)
    lo, hi = n._leader_band()[g]
    # home sits near the closed end: open it first, far enough to close by more than tol
    m0 = (lo + tol + 0.2) if qsign > 0 else (hi - tol - 0.2)
    assert lo < m0 < hi, (lo, m0, hi)
    _move_leader_gripper(r, m0)
    assert r.wait(lambda: abs(r.measured_q()[g] - m0) < 0.002, 8.0)
    bus.side, bus.block = tsign, bus.pos[aas.GRIPPER_ID] + tsign * 5
    _move_leader_gripper(r, m0 + qsign * (tol + 0.1))
    time.sleep(0.5)
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text
    assert abs(r.measured_q()[g] - n._commanded()[g]) > tol       # squeezing beyond it
    pos = bus.pos[aas.GRIPPER_ID]
    assert n._lag_counted(g, pos, aas.tick_deviation(n._goal[aas.GRIPPER_ID], pos)) == 0
    q = dict(r.leader_q)                                           # limbs still follow
    q['arm_limb1_joint'] += 0.05
    r.leader_q = q
    assert r.wait(lambda: abs(r.measured_q()['arm_limb1_joint'] - q['arm_limb1_joint'])
                  < 0.002, 5.0), n.leader_text


def test_opening_lag_beyond_the_tolerance_still_ends_leader_mode(block, rig_factory):
    r = rig_factory(**ALLOW)
    n, bus = r.node, r.bus
    g = _g(n)
    tan._activate(r)
    tol = float(n._p('leader.command_tolerance_rad'))
    tsign, qsign = _close_dir(n)
    lo, hi = n._leader_band()[g]
    m0 = (lo + 0.05) if qsign > 0 else (hi - 0.05)          # near the open end
    _move_leader_gripper(r, m0)
    closed = m0 + qsign * (tol + 0.2)
    assert lo < closed < hi, (lo, closed, hi)
    _move_leader_gripper(r, closed)
    assert r.wait(lambda: abs(r.measured_q()[g] - closed) < 0.002, 5.0)
    bus.side, bus.block = -tsign, bus.pos[aas.GRIPPER_ID] - tsign * 5
    q = dict(r.leader_q)
    q[g] = closed - qsign * (tol + 0.1)
    r.leader_q = q
    assert r.wait(lambda: n._leader_state == aas.LEADER_OFF, 8.0), n.leader_text
    assert 'measured differs from commanded' in n.leader_text and g in n.leader_text
