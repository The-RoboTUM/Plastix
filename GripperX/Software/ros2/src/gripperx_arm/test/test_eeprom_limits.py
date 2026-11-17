"""SR-17 layer L1 generator: computed from the URDF limits + calibration, never typed.
No servo is touched: the bus is a mock, and write() is only ever called on it."""
import math
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

from gripperx_arm import eeprom_limits as el
from gripperx_arm.servo_bus import (
    ADDR_HOMING_OFFSET, ADDR_LOCK, ADDR_MAX_ANGLE_LIMIT, ADDR_MIN_ANGLE_LIMIT)

JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'g']


class MockBus:
    def __init__(self, regs):
        self.regs = regs            # {sid: {addr: value}}
        self.writes = []

    def read_word(self, sid, addr):
        return self.regs.get(sid, {}).get(addr)

    def write_word(self, sid, addr, value):
        self.writes.append((sid, addr, value))
        self.regs.setdefault(sid, {})[addr] = value
        return True

    def write_byte(self, sid, addr, value):
        self.writes.append((sid, addr, value))
        self.regs.setdefault(sid, {})[addr] = value
        return True

    def read_byte(self, sid, addr):
        return self.regs.get(sid, {}).get(addr)


def test_register_addresses_match_the_cited_table():
    # LeRobot STS_SMS_SERIES_CONTROL_TABLE (see servo_bus.py docstring)
    assert (ADDR_MIN_ANGLE_LIMIT, ADDR_MAX_ANGLE_LIMIT, ADDR_HOMING_OFFSET, ADDR_LOCK) == \
        (9, 11, 31, 55)


def test_compute_follows_sign_and_zero():
    lim = {n: (-0.5, 1.0) for n in JOINTS}
    rows = el.compute(lim, JOINTS, [2048] * 6, [1, -1, 1, 1, 1, 1], 4096, 0)
    k = 4096 / (2 * math.pi)
    assert rows[0]['min'] == math.floor(2048 - 0.5 * k) and rows[0]['max'] == math.ceil(2048 + k)
    assert rows[1]['min'] == math.floor(2048 - k) and rows[1]['max'] == math.ceil(2048 + 0.5 * k)
    assert all(r['ok'] for r in rows)


def test_range_crossing_the_wrap_is_flagged():
    # zero ticks of the leader recorded 2026-10-07 for limb4/limb5 sit near the wrap
    lim = {n: (-0.6, 0.6) for n in JOINTS}
    rows = el.compute(lim, JOINTS, [337, 2048, 2048, 2048, 3718, 2048], [1] * 6, 4096, 0)
    assert not rows[0]['ok'] and 'CROSSES' in rows[0]['why']
    assert not rows[4]['ok']
    assert rows[1]['ok']


def test_current_config_verdicts_follow_the_wrap():
    """Against the active description + calibration: a joint is flagged exactly when
    its limit range, in ticks, leaves 0..4095. (Until 2026-10-08 this also asserted that
    the gripper was flagged - true only for the +-pi placeholder limit it had then.)"""
    from gripperx_arm.generate_acm import config_dir, load_urdf
    limits, c = el._load(config_dir(), load_urdf())
    rows = el.compute(limits, list(c['urdf_joints']), c['zero_ticks'], c['signs'],
                      c['ticks_per_rev'], 0)
    k = c['ticks_per_rev'] / (2 * math.pi)
    for r, n, z, s in zip(rows, c['urdf_joints'], c['zero_ticks'], c['signs']):
        ends = sorted((z + s * limits[n][0] * k, z + s * limits[n][1] * k))
        assert r['ok'] == (ends[0] >= 0 and ends[1] <= 4095), (n, ends, r)


def test_dry_run_diff_reads_and_writes_nothing():
    lim = {n: (-0.5, 0.5) for n in JOINTS}
    rows = el.compute(lim, JOINTS, [2048] * 6, [1] * 6, 4096, 10)
    bus = MockBus({sid: {ADDR_MIN_ANGLE_LIMIT: 0, ADDR_MAX_ANGLE_LIMIT: 4095,
                         ADDR_HOMING_OFFSET: 0} for sid in el.IDS})
    lines = el.diff(rows, el.read_current(bus))
    assert len(lines) == 6 and all('DIFF' in ln for ln in lines)
    assert bus.writes == []


@pytest.mark.parametrize('case', ['unverified', 'offset', 'not_expressible'])
def test_write_refusals(case):
    lim = {n: (-0.5, 0.5) for n in JOINTS}
    zero = [2048] * 6 if case != 'not_expressible' else [100] + [2048] * 5
    rows = el.compute(lim, JOINTS, zero, [1] * 6, 4096, 0)
    cur = {sid: {'min': 0, 'max': 4095, 'offset': 7 if case == 'offset' and sid == 3 else 0,
                 'lock': 1, 'torque': 1, 'present': 2048, 'goal': 2048} for sid in el.IDS}
    refuse = el.write_refusals(rows, cur, cal_verified=case != 'unverified',
                               allow_unverified=False)
    assert refuse


def test_write_sequence_on_a_mock():
    lim = {n: (-0.5, 0.5) for n in JOINTS}
    rows = el.compute(lim, JOINTS, [2048] * 6, [1] * 6, 4096, 0)
    bus = MockBus({})
    assert el.write(bus, rows, {}) == []
    first = bus.writes[:4]
    assert first == [(1, ADDR_LOCK, 0), (1, ADDR_MIN_ANGLE_LIMIT, rows[0]['min']),
                     (1, ADDR_MAX_ANGLE_LIMIT, rows[0]['max']), (1, ADDR_LOCK, 1)]


# ------------------------------------------------- first-real-run hardening 2026-10-13
GOOD = {'offset': 0, 'lock': 1, 'torque': 0, 'present': 2048, 'goal': 2048}


def _rows(margin_min=0, margin_max=None):
    lim = {n: (-0.5, 0.5) for n in JOINTS}
    return el.compute(lim, JOINTS, [2048] * 6, [1] * 6, 4096, margin_min, margin_max)


def test_margins_are_per_servo_and_per_side():
    base = _rows()
    rows = _rows([1, 2, 3, 4, 5, 6], [10, 20, 30, 40, 50, 60])
    for b, r, ml, mh in zip(base, rows, [1, 2, 3, 4, 5, 6], [10, 20, 30, 40, 50, 60]):
        assert (r['min'], r['max']) == (b['min'] - ml, b['max'] + mh)


def test_a_negative_margin_is_refused():
    rows = _rows([0, 0, -1, 0, 0, 0], 0)
    assert not rows[2]['ok'] and 'negative margin' in rows[2]['why']
    assert all(r['ok'] for i, r in enumerate(rows) if i != 2)


def test_the_tool_reads_the_margin_keys_the_node_declares():
    """One source: arm_eeprom_limits and eeprom_check: enforce read the same two keys
    of the same file; a third copy (a CLI default) does not exist any more."""
    from gripperx_arm.generate_acm import config_dir
    mlo, mhi = el.load_margins(config_dir())
    assert len(mlo) == len(mhi) == 6
    src = (Path(el.__file__).parent / 'arm_action_server.py').read_text()
    for key in el.MARGIN_KEYS:
        assert f"self._p('{key}')" in src and f"declare_parameter('{key}'" in src
    assert '--margin-ticks' not in Path(el.__file__).read_text()


def test_active_l1_range_lies_inside_the_stops_and_holds_every_taught_pose():
    """Against the active config: the written range must not reach past a measured
    mechanical stop (useless there) and must contain home and the gripper's open and
    close poses (a range that excludes a commanded pose stalls the servo short of it)."""
    from gripperx_arm.generate_acm import config_dir, load_urdf
    from gripperx_geometry import constants as C
    cfg = config_dir()
    limits, c = el._load(cfg, load_urdf())
    rows = el.compute(limits, list(c['urdf_joints']), c['zero_ticks'], c['signs'],
                      c['ticks_per_rev'], *el.load_margins(cfg))
    k = c['ticks_per_rev'] / (2 * math.pi)
    for i, r in enumerate(rows):
        z, s = c['zero_ticks'][i], c['signs'][i]
        stops = sorted((z + s * C.ARM_JOINT_STOP_LOWER[i] * k,
                        z + s * C.ARM_JOINT_STOP_UPPER[i] * k))
        assert r['ok'], r
        assert math.floor(stops[0]) <= r['min'] and r['max'] <= math.ceil(stops[1]), (r, stops)
        assert r['min'] <= z <= r['max']
    poses = yaml.safe_load((cfg / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']
    g = rows[5]
    for name in ('open', 'close'):
        assert g['min'] <= poses['gripper'][name] <= g['max'], (name, g)


def test_write_relocks_and_stops_at_the_first_failure():
    rows = _rows()

    class Failing(MockBus):
        # Not stored on id 2 (a value that does land with a lost acknowledgement is a
        # success since the read-back verification of 2026-10-15).
        def write_word(self, sid, addr, value):
            self.writes.append((sid, addr, value))
            if sid != 2:
                self.regs.setdefault(sid, {})[addr] = value
            return sid != 2

    bus = Failing({})
    failures = el.write(bus, rows, {})
    assert failures and 'id 2' in failures[0]
    assert bus.writes[-1] == (2, ADDR_LOCK, 1)                 # relocked
    assert not any(w[0] > 2 for w in bus.writes)               # servo 3.. untouched


def test_an_interrupted_write_still_relocks():
    rows = _rows()

    class Interrupted(MockBus):
        def write_word(self, sid, addr, value):
            raise KeyboardInterrupt

    bus = Interrupted({})
    with pytest.raises(KeyboardInterrupt):
        el.write(bus, rows, {})
    assert bus.writes == [(1, ADDR_LOCK, 0), (1, ADDR_LOCK, 1)]


def test_write_order_keeps_the_intermediate_pair_ordered():
    lim = {n: (1.0, 1.5) for n in JOINTS}                      # far above the old range
    rows = el.compute(lim, JOINTS, [1000] * 6, [1] * 6, 4096, 0)
    cur = {sid: {'min': 100, 'max': 900} for sid in el.IDS}
    assert rows[0]['min'] > 900
    bus = MockBus({})
    el.write(bus, rows, cur)
    assert [w[1] for w in bus.writes[:4]] == [ADDR_LOCK, ADDR_MAX_ANGLE_LIMIT,
                                              ADDR_MIN_ANGLE_LIMIT, ADDR_LOCK]


@pytest.mark.parametrize('field,value,needle', [
    ('present', 100, 'present position 100 outside'),
    ('torque', 1, 'Torque_Enable = 1'),
    ('lock', None, 'lock unreadable'),
    ('torque', None, 'torque unreadable'),
])
def test_write_refuses_what_would_move_or_is_unknown(field, value, needle):
    rows = _rows()
    cur = {sid: {'min': 0, 'max': 4095, **GOOD} for sid in el.IDS}
    cur[4][field] = value
    refuse = el.write_refusals(rows, cur, cal_verified=True, allow_unverified=False)
    assert any(needle in w and w.startswith('id 4') for w in refuse), refuse


def test_goal_outside_is_harmless_with_torque_off():
    rows = _rows()
    cur = {sid: {'min': 0, 'max': 4095, **GOOD} for sid in el.IDS}
    cur[4].update(torque=0, goal=100)
    assert el.write_refusals(rows, cur, cal_verified=True, allow_unverified=False) == []


@pytest.mark.parametrize('servo,recorded,refused', [
    (0, None, False), (3624, None, True),
    (3624, [3624, 3353, 1323, 1748, 3325, 0], False), (0, [3624, 3353, 1323, 1748, 3325, 0], True)])
def test_homing_offset_must_equal_the_recorded_one(servo, recorded, refused):
    """Audit EE1 M3: none recorded -> only 0; recorded -> exactly that raw value."""
    rows = _rows()
    cur = {sid: {'min': 0, 'max': 4095, **GOOD} for sid in el.IDS}
    cur[1]['offset'] = servo
    if recorded:
        for sid in el.IDS[1:]:
            cur[sid]['offset'] = recorded[sid - 1]
    w = el.write_refusals(rows, cur, True, False, recorded)
    assert any('Homing_Offset' in x for x in w) == refused, w


def test_recorded_offsets_absent_or_malformed():
    assert el.recorded_offsets({}) is None
    assert el.recorded_offsets({'homing_offsets_raw': []}) is None
    with pytest.raises(SystemExit):
        el.recorded_offsets({'homing_offsets_raw': [1, 2]})
    with pytest.raises(SystemExit):
        el.recorded_offsets({'homing_offsets_raw': [5000] * 6})
    with pytest.raises(SystemExit):                 # YAML true/false are ints in Python
        el.recorded_offsets({'homing_offsets_raw': [True, 0, 0, 0, 0, 0]})
    active = yaml.safe_load((_cfg() / 'arm_calibration.yaml').read_text())
    # measured: boot journal 2026-10-13 17:13:34 CEST (arm_calibration.yaml comment)
    assert el.recorded_offsets(active['arm_joint_state_relay']['ros__parameters']) == \
        [3624, 3353, 1323, 1748, 3325, 4055]


def _cfg():
    from gripperx_arm.generate_acm import config_dir
    return config_dir()


def _active_rows():
    from gripperx_arm.generate_acm import load_urdf
    limits, c = el._load(_cfg(), load_urdf())
    return el.compute(limits, list(c['urdf_joints']), c['zero_ticks'], c['signs'],
                      c['ticks_per_rev'], *el.load_margins(_cfg()))


def test_active_ranges_hold_the_measured_rest_pose_and_the_grip_pose():
    """Audit EE1 L3: the limp rest pose (2026-10-09) - where the arm lies at every
    boot after a power loss - and the taught grip pose lie inside the active L1."""
    rest = [2089, 834, 3071, 1463, 2116, 2651]
    grip = yaml.safe_load((_cfg() / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']['poses']['grip']
    for r, a, b in zip(_active_rows(), rest, grip):
        assert r['min'] <= a <= r['max'] and r['min'] <= b <= r['max'], (r, a, b)


class _RegBus(MockBus):
    """A servo bus as the tool sees it: limit, offset, torque, lock, positions."""

    def __init__(self, regs, store=True):
        super().__init__(regs)
        self.store = store
        self.closed = False

    def write_word(self, sid, addr, value):
        self.writes.append((sid, addr, value))
        if self.store:
            self.regs.setdefault(sid, {})[addr] = value
        return True

    def open(self):
        pass

    def close(self):
        self.closed = True


def _active_offsets():
    c = yaml.safe_load((_cfg() / 'arm_calibration.yaml').read_text())[
        'arm_joint_state_relay']['ros__parameters']
    return el.recorded_offsets(c) or [0] * 6


def _main_rig(monkeypatch, tmp_path, torque=0, store=True, offsets=None):
    """The servos as the robot reported them 2026-10-12/13: the LeRobot limit
    registers, the active calibration's recorded offsets, at home."""
    import gripperx_arm.servo_bus as sb
    offs = dict(zip(el.IDS, offsets or _active_offsets()))
    regs = {sid: {ADDR_MIN_ANGLE_LIMIT: lo, ADDR_MAX_ANGLE_LIMIT: hi, ADDR_HOMING_OFFSET: offs[sid],
                  sb.ADDR_TORQUE: torque, ADDR_LOCK: 1, sb.ADDR_PRESENT: p, sb.ADDR_GOAL_POS: p}
            for sid, lo, hi, p in zip(el.IDS, [760, 823, 873, 865, 0, 1293],
                                      [3232, 3199, 3076, 3188, 4095, 2810],
                                      [2068, 978, 3074, 1035, 2148, 1527])}
    bus = _RegBus(regs, store)
    monkeypatch.setattr(sb, 'FeetechBus', lambda port, baud: bus)
    monkeypatch.setattr(el, 'bus_refusals', lambda port: [])
    monkeypatch.setattr(el, 'hold_exclusive', lambda b: True)
    monkeypatch.chdir(tmp_path)
    return bus


def test_write_path_end_to_end_on_a_mock_bus(monkeypatch, tmp_path, capsys):
    """Audit EE1 L3: snapshot -> write -> read-back -> exit code, through main()."""
    bus = _main_rig(monkeypatch, tmp_path)
    assert el.main(['--port', '/dev/x', '--write']) == 0, capsys.readouterr().out
    out = capsys.readouterr().out
    assert 'read-back OK' in out
    snap = list(tmp_path.glob('arm_eeprom_previous_*.yaml'))
    assert len(snap) == 1
    assert yaml.safe_load(snap[0].read_text())['eeprom_limits'][2] == [823, 3199]
    for r in _active_rows():
        assert (bus.regs[r['id']][ADDR_MIN_ANGLE_LIMIT],
                bus.regs[r['id']][ADDR_MAX_ANGLE_LIMIT]) == (r['min'], r['max'])
    assert all(bus.regs[sid][ADDR_LOCK] == 1 for sid in el.IDS) and bus.closed


def test_write_path_reports_a_read_back_mismatch(monkeypatch, tmp_path, capsys):
    _main_rig(monkeypatch, tmp_path, store=False)
    assert el.main(['--port', '/dev/x', '--write']) == 1
    assert 'MISMATCH' in capsys.readouterr().out


def test_write_path_refuses_with_torque_on_and_writes_nothing(monkeypatch, tmp_path, capsys):
    bus = _main_rig(monkeypatch, tmp_path, torque=1)
    assert el.main(['--port', '/dev/x', '--write']) == 1
    assert 'Torque_Enable = 1' in capsys.readouterr().out
    assert bus.writes == [] and not list(tmp_path.glob('arm_eeprom_previous_*'))


def test_write_path_refuses_a_changed_offset(monkeypatch, tmp_path, capsys):
    bus = _main_rig(monkeypatch, tmp_path, offsets=[0] * 6)
    assert el.main(['--port', '/dev/x', '--write']) == 1
    assert 'Homing_Offset' in capsys.readouterr().out and bus.writes == []


def test_torque_off_mode_switches_torque_only(monkeypatch, tmp_path, capsys):
    import gripperx_arm.servo_bus as sb
    bus = _main_rig(monkeypatch, tmp_path, torque=1)
    assert el.main(['--port', '/dev/x', '--torque-off']) == 0
    assert bus.writes == [(sid, sb.ADDR_TORQUE, 0) for sid in el.IDS]
    assert el.main(['--port', '/dev/x', '--torque-off', '--write']) == 2


def test_port_holders_sees_another_process(tmp_path):
    dev = tmp_path / 'fake_tty'
    dev.write_text('')
    p = subprocess.Popen([sys.executable, '-c',
                          f'import time; f = open({str(dev)!r}); time.sleep(30)'])
    try:
        for _ in range(50):
            if el.port_holders(str(dev)):
                break
            time.sleep(0.1)
        assert any(f'pid {p.pid}' in h for h in el.port_holders(str(dev)))
    finally:
        p.kill()
        p.wait()


@pytest.mark.parametrize('state', ['active', 'activating'])
def test_no_bus_access_while_the_bringup_unit_runs(monkeypatch, state, capsys):
    import gripperx_arm.servo_bus as sb
    monkeypatch.setattr(el, 'unit_state', lambda unit=el.BRINGUP_UNIT: state)
    monkeypatch.setattr(el, 'arm_node_running', lambda: False)
    monkeypatch.setattr(el, 'port_holders', lambda port: [])

    def no_open(*a, **kw):
        raise AssertionError('the bus was opened')
    monkeypatch.setattr(sb, 'FeetechBus', no_open)
    assert el.main(['--port', '/dev/null', '--write']) == 1
    assert 'REFUSED' in capsys.readouterr().out


def test_snapshot_round_trips_through_restore(tmp_path):
    """The rollback set read 2026-10-12 (node log), factory 0..4095 on id 5 included."""
    prev = {1: (760, 3232), 2: (823, 3199), 3: (873, 3076), 4: (865, 3188), 5: (0, 4095),
            6: (1293, 2810)}
    cur = {sid: {'min': lo, 'max': hi} for sid, (lo, hi) in prev.items()}
    f = tmp_path / 'prev.yaml'
    f.write_text(yaml.safe_dump(el.snapshot(cur)))
    rows = el.restore_rows(f)
    assert all(r['ok'] for r in rows), rows
    assert {r['id']: (r['min'], r['max']) for r in rows} == prev


def test_restore_rejects_a_broken_snapshot(tmp_path):
    f = tmp_path / 'bad.yaml'
    f.write_text(yaml.safe_dump({'eeprom_limits': {1: [3000, 100], 2: [0, 5000]}}))
    rows = el.restore_rows(f)
    assert not rows[0]['ok'] and not rows[1]['ok'] and not rows[2]['ok']


def test_a_non_zero_homing_offset_is_shown_decoded_and_blocks_the_write():
    """The leader read 2026-10-13: Homing_Offset id1 = 3624 (sign bit 11) = -1576."""
    rows = _rows()
    cur = {sid: {'min': 760, 'max': 3232, **GOOD} for sid in el.IDS}
    cur[1]['offset'] = 3624
    line = el.diff(rows, cur)[0]
    assert '3624 = -1576 NON-ZERO' in line
    assert any('Homing_Offset 3624 = -1576' in w for w in
               el.write_refusals(rows, cur, cal_verified=True, allow_unverified=False))


# ------------------------------------------------------------------ 2026-10-15 hardening
# The leader's EEPROM run (same STS3215, same SDK) lost the packet right after an EEPROM
# write, and the SDK raised IndexError on the short reply.

def test_short_reply_ends_the_transaction_as_failed_not_as_a_traceback():
    from gripperx_arm import servo_bus as sb

    class ShortPkt:
        def read2ByteTxRx(self, port, sid, addr):
            raise IndexError('list index out of range')
        read1ByteTxRx = read2ByteTxRx

        def write1ByteTxRx(self, port, sid, addr, value):
            raise IndexError('list index out of range')
        write2ByteTxRx = write1ByteTxRx

    class Port:
        is_using = True

        def readPort(self, n):
            return []

    b = sb.FeetechBus.__new__(sb.FeetechBus)
    b._scs = type('S', (), {'COMM_SUCCESS': 0})
    b._port, b._pkt = Port(), ShortPkt()
    b._bound()
    assert b.read_word(1, ADDR_MIN_ANGLE_LIMIT) is None
    assert b.read_byte(1, ADDR_LOCK) is None
    assert b.write_word(1, ADDR_MIN_ANGLE_LIMIT, 5) is False
    assert b._port.is_using is False and b.malformed == 3


class BusyBus(MockBus):
    """For busy_s after an EEPROM / Lock write every transaction fails (None / False)."""

    def __init__(self, regs, busy_s):
        super().__init__(regs)
        self.busy_s, self.until, self.dropped = busy_s, 0.0, 0

    def _busy(self):
        if time.monotonic() < self.until:
            self.dropped += 1
            return True
        return False

    def write_word(self, sid, addr, value):
        if self._busy():
            return False
        super().write_word(sid, addr, value)
        self.until = time.monotonic() + self.busy_s
        return True

    write_byte = write_word

    def read_word(self, sid, addr):
        return None if self._busy() else super().read_word(sid, addr)

    read_byte = read_word


def test_settle_and_read_back_write_through_a_busy_servo():
    rows = _rows()
    bus = BusyBus({}, 0.02)
    assert el.write(bus, rows, {}, settle_s=0.03) == []
    assert bus.dropped == 0
    for r in rows:
        assert (bus.regs[r['id']][ADDR_MIN_ANGLE_LIMIT], bus.regs[r['id']][ADDR_MAX_ANGLE_LIMIT],
                bus.regs[r['id']][ADDR_LOCK]) == (r['min'], r['max'], 1)


def test_without_settle_a_busy_servo_is_reported_and_the_run_stops():
    rows = _rows()
    bus = BusyBus({}, 0.05)
    failures = el.write(bus, rows, {}, settle_s=0.0)
    assert failures and 'not confirmed' in failures[0]
    assert not any(w[0] > 1 for w in bus.writes)


def test_a_lost_acknowledgement_with_the_value_in_place_counts():
    class LostAck(MockBus):
        def write_word(self, sid, addr, value):
            super().write_word(sid, addr, value)
            return False
    bus = LostAck({})
    assert el.write_verified(bus, 1, ADDR_MIN_ANGLE_LIMIT, 7, 2, 0.0) is None


def test_an_existing_snapshot_is_never_overwritten_but_kept_for_a_rerun(monkeypatch, tmp_path,
                                                                        capsys):
    """Audit EW1 M2: the leader tool's snapshot-of-record rule. A run interrupted after
    one write: the re-run keeps the first snapshot (the true originals)."""
    bus = _main_rig(monkeypatch, tmp_path)
    snap = tmp_path / 'first.yaml'
    snap.write_text(yaml.safe_dump(el.snapshot(el.read_current(bus))))
    original = snap.read_text()
    row = _active_rows()[0]
    bus.regs[row['id']][ADDR_MIN_ANGLE_LIMIT] = row['min']          # the one that landed
    assert el.main(['--port', '/dev/x', '--write', '--save-previous', str(snap),
                    '--settle-sec', '0']) == 0, capsys.readouterr().out
    assert 'KEPT as the snapshot of record' in capsys.readouterr().out
    assert snap.read_text() == original


def test_an_existing_snapshot_with_foreign_differences_refuses(monkeypatch, tmp_path, capsys):
    bus = _main_rig(monkeypatch, tmp_path)
    snap = tmp_path / 'first.yaml'
    snap.write_text(yaml.safe_dump(el.snapshot(el.read_current(bus))))
    bus.regs[2][ADDR_MIN_ANGLE_LIMIT] = 500
    n = len(bus.writes)
    assert el.main(['--port', '/dev/x', '--write', '--save-previous', str(snap),
                    '--settle-sec', '0']) == 1
    out = capsys.readouterr().out
    assert 'never overwritten' in out and 'id 2 min: now 500' in out
    assert len(bus.writes) == n
