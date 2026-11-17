"""A bus transaction that hangs, and a bus that disconnects (robot event 2026-10-13 17:42: the arm node ignored
SIGINT and SIGTERM and was SIGKILLed after 15 s; one hung read in the joint-state timer
filled all four executor workers). Space must still be handled within a bound, the
joint-state timer may occupy at most one worker, and the process must exit within a
few seconds of SIGINT/SIGTERM."""
import os
import signal
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '225')

import pytest  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from std_msgs.msg import Empty, String  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_arm_node as tan  # noqa: E402
from test_arm_node import ALLOW, _status, rig_factory  # noqa: E402,F401
from test_arm_node import _activate  # noqa: E402
from gripperx_arm import arm_action_server as aas  # noqa: E402


class HangingBus(tan.MockBus):
    """read_pos blocks while `hang` is set (a transaction that never returns) - only in
    threads whose stack contains `only` (a function name), if given."""

    def __init__(self, ticks):
        super().__init__(ticks)
        self.hang = threading.Event()
        self.release = threading.Event()
        self.only = None
        self.blocked = 0

    def read_pos(self, sid):
        if self.hang.is_set() and (self.only is None or self._in(self.only)):
            self.blocked += 1
            try:
                self.release.wait(30.0)
            finally:
                self.blocked -= 1
        return super().read_pos(sid)

    @staticmethod
    def _in(name):
        f = sys._getframe(2)
        while f is not None:
            if f.f_code.co_name == name:
                return True
            f = f.f_back
        return False


@pytest.fixture
def hanging(monkeypatch):
    monkeypatch.setattr(tan, 'MockBus', HangingBus)


def _hang_the_bus(r):
    """A read that hangs while it holds the serial lock (as in _publish_joint_states)."""
    r.bus.hang.set()
    t = threading.Thread(target=r.node._read_all, daemon=True)
    t.start()
    assert r.wait(lambda: r.bus.blocked >= 1, 3.0)
    assert r.node._serial_lock.locked()
    return t


@pytest.mark.parametrize('moving', [True, False])
def test_space_with_a_hung_bus_is_handled_and_applied_once_it_frees(hanging, rig_factory,
                                                                    moving):
    r = rig_factory(**ALLOW)
    node, bus = r.node, r.bus
    if moving:
        _activate(r)                             # leader mode active counts as moving
    _hang_the_bus(r)
    n = len(node.audit)
    t0 = time.monotonic()
    r.stop.publish(Empty())
    assert _status(r, 'TORQUE OFF PENDING: Space', 3.0), list(node.status_log)[-3:]
    took = time.monotonic() - t0
    assert took < 1.0, took                      # bounded: not behind the hung read
    assert node._stop.is_set() and node._torque_off_pending is not None
    assert all(bus.torque_on.values()), 'nothing could reach the bus yet'
    assert [k for k, _ in list(node.audit)[n:]] == []
    if not moving:
        assert 'busy' in node.status_log[-1], node.status_log[-1]
    bus.release.set()
    bus.hang.clear()
    assert _status(r, '(applied once the bus was free)', 3.0), list(node.status_log)[-3:]
    assert not any(bus.torque_on.values()) and not node._torque_on
    assert node._torque_off_pending is None
    assert [k for k, _ in list(node.audit)[n:]] == ['torque_off']


def test_a_pending_torque_off_wins_over_the_next_write(hanging, rig_factory):
    """The next bus access applies a pending torque-off instead of writing a goal."""
    r = rig_factory()
    node, bus = r.node, r.bus
    node._torque_off_pending = 'test'
    goals = len(bus.writes)
    with node._serial_lock:
        with pytest.raises(aas.MotionStopped, match='pending Space torque-off'):
            node._write('approve', {1: bus.pos[1]})
    assert not any(bus.torque_on.values()) and node._torque_off_pending is None
    assert len(bus.writes) == goals


def test_joint_states_occupy_at_most_one_worker(hanging, rig_factory):
    """A hung read in the 2 Hz joint-state timer: its own mutually exclusive group, so
    the timer does not pile up in further workers while the read hangs."""
    r = rig_factory()
    node, bus = r.node, r.bus
    t = [t for t in node.timers if t.callback == node._publish_joint_states]
    assert len(t) == 1 and isinstance(t[0].callback_group, MutuallyExclusiveCallbackGroup)
    assert t[0].callback_group is not node._cb_group
    bus.only = '_publish_joint_states'
    bus.hang.set()
    time.sleep(2.5)                              # 5 timer periods
    try:
        assert bus.blocked == 1, bus.blocked
    finally:
        bus.release.set()


CHILD = textwrap.dedent('''
    import os, sys, time
    sys.path.insert(0, {test!r})
    from gripperx_arm import arm_action_server as aas
    T0 = time.time()

    class Bus:
        pos = {{1: 2068, 2: 978, 3: 3074, 4: 1035, 5: 2148, 6: 1527}}
        def open(self): pass
        def close(self): pass
        def torque(self, sid, on): return True
        def read_pos(self, sid):
            if time.time() - T0 > {block_at}:
                time.sleep(120)      # a bus read that never returns
            return self.pos[sid]
        def read_word(self, sid, addr): return None
        def read_byte(self, sid, addr): return None
        def read_load(self, sid): return 0
        def write_goal(self, sid, t): return True

    print('CHILD START', flush=True)
    aas.main(bus=Bus())
''')


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_stop_exits_within_a_bound_with_a_hung_worker(tmp_path, sig):
    """The robot event, offline: a joint-state read hangs, then the stack stop's
    signal. The process exits within ~3 s (it used to wait forever in the interpreter's
    join of the executor's non-daemon workers) and logs every shutdown stage."""
    script = tmp_path / 'child.py'
    script.write_text(CHILD.format(test=str(Path(__file__).resolve().parent), block_at=1.0))
    env = dict(os.environ, ROS_DOMAIN_ID='225', ROS_AUTOMATIC_DISCOVERY_RANGE='OFF')
    p = subprocess.Popen([sys.executable, '-u', str(script)], stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, env=env)
    lines = []
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60.0:
            line = p.stdout.readline()
            lines.append(line)
            if 'arm_action_server ready' in line:
                break
        else:
            pytest.fail(''.join(lines[-20:]))
        time.sleep(3.0)                          # the joint-state read is hanging now
        t = time.monotonic()
        p.send_signal(sig)
        p.wait(6.0)
        took = time.monotonic() - t
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
    out = ''.join(lines) + p.stdout.read()
    assert took < 3.5, (took, out[-2000:])
    assert p.returncode == 0, out[-2000:]
    for stage in ('executor shutdown (timeout 2 s)', 'destroy_node', 'rclpy shutdown',
                  'os._exit(0)'):
        assert f'arm_action_server shutdown: {stage}' in out, out[-2000:]
    assert 'serial lock still held after 1 s' in out, out[-2000:]


# ------------------------------------------------------------------ lost bus (18:09 event)
class FlakyBus(tan.MockBus):
    """The USB serial adapter gone: every call raises an I/O error, open() too, until
    `gone` is cleared."""

    def __init__(self, ticks):
        super().__init__(ticks)
        self.gone = False
        self.opened = 0

    def _io(self):
        if self.gone:
            raise OSError(5, 'Input/output error')

    def read_pos(self, sid):
        self._io()
        return super().read_pos(sid)

    def write_goal(self, sid, ticks):
        self._io()
        return super().write_goal(sid, ticks)

    def torque(self, sid, on):
        self._io()
        return super().torque(sid, on)

    def open(self):
        self._io()
        self.opened += 1

    model = 777

    def read_word(self, sid, addr):
        from gripperx_arm.servo_bus import ADDR_MODEL_NUMBER
        if addr == ADDR_MODEL_NUMBER:
            self._io()
            return self.model
        return None


@pytest.fixture
def flaky(monkeypatch):
    monkeypatch.setattr(tan, 'MockBus', FlakyBus)


def _lose(r):
    r.bus.gone = True
    assert _status(r, 'ARM BUS LOST', 3.0), list(r.node.status_log)[-3:]
    assert r.node._bus.lost is not None and not r.node._torque_on
    assert all(g is None for g in r.node._goal.values())
    return len(r.bus.events)


def test_a_lost_bus_is_reopened_and_nothing_is_written_on_reconnect(flaky, rig_factory):
    r = rig_factory()
    node, bus = r.node, r.bus
    n = _lose(r)
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'arm bus disconnected', 3.0), list(node.status_log)[-3:]
    time.sleep(1.5)                               # reopen attempts fail meanwhile
    assert node._bus.lost is not None and bus.opened == 0
    bus.gone = False
    assert _status(r, 'ARM BUS RECONNECTED', 3.0), list(node.status_log)[-3:]
    assert bus.opened == 1 and node._bus.lost is None
    assert bus.events[n:] == [], bus.events[n:]   # no torque, no goal on reconnect
    assert not node._torque_on                    # torque state unknown to the node
    k = len(bus.events)
    r.cmd.publish(String(data='go_home'))         # the normal path: sync, torque-on, move
    assert r.wait(lambda: any(e[0] == 'torque' for e in bus.events[k:]), 10.0), \
        list(node.status_log)[-3:]
    ev = bus.events[k:]
    t = next(i for i, e in enumerate(ev) if e[0] == 'torque')
    assert sorted((e[1], e[2]) for e in ev[:t]) == sorted(bus.pos.items())


def test_space_while_the_bus_is_lost_is_applied_on_reconnect(flaky, rig_factory):
    r = rig_factory()
    node, bus = r.node, r.bus
    _lose(r)
    r.stop.publish(Empty())
    assert _status(r, 'TORQUE OFF PENDING', 3.0), list(node.status_log)[-3:]
    assert 'DISCONNECTED' in node.status_log[-1] and node._stop.is_set()
    assert all(bus.torque_on.values())            # nothing reached the servos
    bus.gone = False
    assert _status(r, '(applied once the bus was free)', 4.0), list(node.status_log)[-3:]
    assert not any(bus.torque_on.values())


def test_space_after_a_reconnect_turns_the_unknown_torque_off(flaky, rig_factory):
    """After a reconnect the node does not know the torque state: the servos may still
    hold torque, so a Space on the stationary arm turns it off."""
    r = rig_factory()
    node, bus = r.node, r.bus
    _lose(r)
    bus.gone = False
    assert _status(r, 'ARM BUS RECONNECTED', 3.0)
    assert all(bus.torque_on.values())
    r.stop.publish(Empty())
    assert _status(r, 'Space: retrying the torque-off', 3.0), list(node.status_log)[-3:]
    assert not any(bus.torque_on.values())


def test_a_reopened_bus_with_other_servos_is_refused(flaky, rig_factory):
    """Audit HG3 H4: the reopened port must carry the servos read at startup (model
    numbers); positions far from the last ones seen are reported loudly."""
    r = rig_factory()
    node, bus = r.node, r.bus
    assert set(node._models.values()) == {777}
    assert r.wait(lambda: node._last_seen is not None, 3.0)
    _lose(r)
    bus.model = 778
    bus.gone = False
    assert _status(r, 'servo model number(s) differ from startup', 4.0), \
        list(node.status_log)[-3:]
    assert node._bus.lost is not None
    bus.model = 777
    bus.pos[2] += aas.POS_TOLERANCE + 20                      # sagged while lost
    assert _status(r, 'ARM BUS RECONNECTED on', 4.0), list(node.status_log)[-3:]
    assert any('positions MOVED while the bus was lost' in t and f'{{2: {aas.POS_TOLERANCE + 20}}}'
               in t for t in node.status_log), list(node.status_log)[-3:]


class SlowBus(tan.MockBus):
    """A degraded bus: every goal write runs into the transaction bound and fails
    (audit HG3 H1). The event is recorded when the transaction STARTS."""
    slow = 0.0

    def write_goal(self, sid, ticks):
        if not self.slow:
            return super().write_goal(sid, ticks)
        with self.lock:
            self.events.append(('goal', sid, int(ticks)))
        time.sleep(self.slow)
        return False


def test_space_during_a_batch_write_on_a_degraded_bus(monkeypatch, rig_factory):
    """Space while a 6-servo goal write crawls at TXN_MAX_S per servo: no goal write
    starts after the Space, and every servo is torqued off within about one
    transaction bound plus the lock timeout."""
    from gripperx_arm.servo_bus import TXN_MAX_S
    monkeypatch.setattr(tan, 'MockBus', SlowBus)
    r = rig_factory()
    node, bus = r.node, r.bus
    bus.slow = TXN_MAX_S
    ended = {}

    def writer():
        try:
            with node._serial_lock:
                node._write('approve', {s: bus.pos[s] for s in aas.IDS})
        except Exception as exc:     # noqa: BLE001
            ended['exc'] = exc
    th = threading.Thread(target=writer)
    th.start()
    time.sleep(0.1)
    k = len(bus.events)
    t0 = time.monotonic()
    r.stop.publish(Empty())
    assert r.wait(lambda: not any(bus.torque_on.values()), 5.0)
    took = time.monotonic() - t0
    th.join(5.0)
    assert [e for e in bus.events[k:] if e[0] == 'goal'] == [], bus.events[k:]
    assert took < TXN_MAX_S + node._stop_lock_timeout + 0.05 + 0.3, took   # + load margin
    assert isinstance(ended.get('exc'), aas.MotionStopped), ended
