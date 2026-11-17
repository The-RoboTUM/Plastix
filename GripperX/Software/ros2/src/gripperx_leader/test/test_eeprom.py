"""leader_eeprom (prepared 2026-10-13): register-level mock only, no port is opened.

The mock (test_align.make_sdk) implements Present = Actual - Homing_Offset with the
sign-magnitude offset (bit 11) and ignores EEPROM writes while Lock != 0."""

import math
import os

import pytest
import yaml

from gripperx_leader import leader_bus as lb
from gripperx_leader import leader_eeprom as le
from gripperx_leader.aligner import AlignConfig, Aligner, AlignRefused
from gripperx_leader.calibration import Calibration, load_calibration
from gripperx_leader.leader_bus import LeaderBus, LeaderWriteError
from test_align import (ACT, LIMITS, NAMES, PRE_SHIFT_CAL_FILE, PRE_SHIFT_LIMB5,
                        SHIPPED_CAL_FILE, make_sdk)

# The tool is exercised from the state it was built for: the pre-shift calibration
# (test_align.PRE_SHIFT_CAL_FILE, the 2026-10-13 state) against the registers read
# that day. The shipped calibration (shifted 2026-10-15) is checked against the
# registers the tool leaves behind - see "the shipped calibration" below.
CAL_FILE = PRE_SHIFT_CAL_FILE
# Leader registers read 2026-10-13 (coordinator, read-only).
# Id 6 (gripper) read 2026-11-13 (coordinator, read-only; unchanged since it was never
# written): position limits 1293/2810, Homing_Offset 4055, present at its zero 1986.
TODAY_LIMITS = {1: (760, 3232), 2: (823, 3199), 3: (873, 3076), 4: (865, 3188), 5: (0, 4095),
                6: (1293, 2810)}
TODAY_OFFSET = {1: 3624, 2: 3353, 3: 1323, 4: 1748, 5: 3325, 6: 4055}
TODAY_PRESENT5 = 3631
TODAY_PRESENT6 = 1986


def leader_today():
    """The leader's registers as read 2026-10-13, before the EEPROM step (id 6: 2026-11-13)."""
    sdk = make_sdk({1: 1726, 2: 2036, 3: 2662, 4: 337, 5: TODAY_PRESENT5, 6: TODAY_PRESENT6})
    for sid, (lo, hi) in TODAY_LIMITS.items():
        sdk.regs[sid][lb.ADDR_MIN_POS_LIMIT] = lo
        sdk.regs[sid][lb.ADDR_MAX_POS_LIMIT] = hi
        sdk.regs[sid][lb.ADDR_HOMING_OFFSET] = TODAY_OFFSET[sid]
    bus = LeaderBus('/dev/null', ids=le.IDS, sdk=sdk)
    return bus, sdk


def run(argv, bus, holders=lambda port: [], exclusive=lambda b: True):
    lines = []
    rc = le.run(['--port', '/dev/null', '--calibration', CAL_FILE] + argv, bus=bus,
                holders=holders, exclusive=exclusive, out=lines.append)
    return rc, '\n'.join(lines)


# ------------------------------------------------------------------ arithmetic

def test_sign_magnitude_bit11_matches_lerobot_encoding():
    assert le.sm_decode(3325) == -1277          # 3325 = 0b1100_1111_1101: bit 11 set
    assert le.sm_decode(414) == 414
    assert le.sm_encode(-1277) == 3325 and le.sm_encode(414) == 414
    assert all(le.sm_decode(le.sm_encode(v)) == v for v in range(-2047, 2048))
    with pytest.raises(ValueError):
        le.sm_encode(2048)


def test_limb5_wrap_shift_puts_the_wrap_in_the_middle_of_the_gap():
    cal = load_calibration(CAL_FILE)
    i = le.wrap_shift(cal.zero_ticks[4], cal.sign[4], cal.travel_min_rad[4],
                      cal.travel_max_rad[4])
    k = 4096 / (2 * math.pi)
    assert i['gap_ticks'] == pytest.approx((2 * math.pi - (2.9544 + 2.8900)) * k, abs=0.5)
    assert i['shift'] == 1691 and i['new_zero'] == 2027
    assert 0 <= i['new_lo'] < i['new_hi'] <= 4095            # travel no longer crosses the wrap
    assert abs(i['new_lo'] - (4095 - i['new_hi'])) <= 2      # gap split evenly
    assert i['margin_ticks'] > 140


def test_plan_for_todays_registers():
    bus, sdk = leader_today()
    p = le.plan(le.read_current(bus), load_calibration(CAL_FILE))
    writes = [(sid, f, old, new) for sid, f, _, old, new in p['writes']]
    assert writes == [(1, 'min', 760, 0), (1, 'max', 3232, 4095),
                      (2, 'min', 823, 0), (2, 'max', 3199, 4095),
                      (3, 'min', 873, 0), (3, 'max', 3076, 4095),
                      (4, 'min', 865, 0), (4, 'max', 3188, 4095),
                      (5, 'offset', 3325, 414),
                      (6, 'min', 1293, 0), (6, 'max', 2810, 4095)]   # id 6 since 2026-11-13
    s5 = p['shift'][5]
    assert (s5['recorded_offset'], s5['target_offset'], s5['target_offset_signed'],
            s5['expected_present']) == (3325, 414, 414, 1940)
    assert not p['why'] and all(old != new for _, _, _, old, new in p['writes'])


# ------------------------------------------------------------------ CLI

def test_dry_run_writes_nothing_and_prints_the_plan():
    bus, sdk = leader_today()
    rc, text = run([], bus)
    assert rc == 0 and sdk.writes == [] and 'dry run: nothing written' in text
    assert 'id 5 offset (register 31): 3325 -> 414' in text
    assert 'target 414 (414)' in text
    assert 'zero_ticks[4] 3718 -> 2027' in text


def test_refuses_while_the_port_is_held():
    bus, sdk = leader_today()
    rc, text = run(['--write'], bus, holders=lambda port: ['pid 42: leader_arm_node'])
    assert rc == 1 and 'REFUSED' in text and sdk.writes == []


def test_write_refused_without_exclusive_hold():
    bus, sdk = leader_today()
    rc, text = run(['--write'], bus, exclusive=lambda b: False)
    assert rc == 1 and 'WRITE REFUSED' in text and sdk.writes == []


def test_write_refused_while_torque_on():
    bus, sdk = leader_today()
    sdk.regs[3][lb.ADDR_TORQUE] = 1
    rc, text = run(['--write'], bus)
    assert rc == 1 and 'torque is ON' in text and sdk.writes == []


def test_write_then_restore_round_trip(tmp_path):
    bus, sdk = leader_today()
    snap = tmp_path / 'prev.yaml'
    rc, text = run(['--write', '--save-previous', str(snap)], bus)
    assert rc == 0, text
    assert 'read-back OK' in text
    for sid in range(1, 5):
        assert (sdk.regs[sid][lb.ADDR_MIN_POS_LIMIT], sdk.regs[sid][lb.ADDR_MAX_POS_LIMIT]) == (0, 4095)
        assert sdk.regs[sid][lb.ADDR_HOMING_OFFSET] == TODAY_OFFSET[sid]   # untouched
    assert sdk.regs[5][lb.ADDR_HOMING_OFFSET] == 414
    assert sdk.regs[5][lb.ADDR_PRESENT] == 1940
    assert all(sdk.regs[s][lb.ADDR_LOCK] == 1 for s in le.IDS)
    assert not [w for w in sdk.writes if w[1] == lb.ADDR_TORQUE]        # never touches torque
    saved = yaml.safe_load(snap.read_text())['leader_eeprom']
    assert saved[5] == {'min': 0, 'max': 4095, 'offset': 3325}
    # rollback
    rc, text = run(['--write', '--restore', str(snap), '--save-previous',
                    str(tmp_path / 'p2.yaml')], bus)
    assert rc == 0, text
    for sid in le.IDS:
        assert (sdk.regs[sid][lb.ADDR_MIN_POS_LIMIT], sdk.regs[sid][lb.ADDR_MAX_POS_LIMIT]) == TODAY_LIMITS[sid]
        assert sdk.regs[sid][lb.ADDR_HOMING_OFFSET] == TODAY_OFFSET[sid]
    assert sdk.regs[5][lb.ADDR_PRESENT] == TODAY_PRESENT5


def test_failure_relocks_and_stops(tmp_path):
    bus, sdk = leader_today()
    sdk.fail_write.add((2, lb.ADDR_MAX_POS_LIMIT))
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'p.yaml')], bus)
    assert rc == 1 and 'PROBLEM' in text
    assert all(sdk.regs[s][lb.ADDR_LOCK] == 1 for s in le.IDS)
    assert not [w for w in sdk.writes if w[0] in (3, 4, 5)]           # stopped at id 2


def test_ctrl_c_mid_write_relocks():
    bus, sdk = leader_today()
    real = bus.write_register

    def interrupt(sid, addr, value, size):
        if addr == lb.ADDR_MAX_POS_LIMIT:
            raise KeyboardInterrupt
        real(sid, addr, value, size)
    bus.write_register = interrupt
    current = le.read_current(bus)
    p = le.plan(current, load_calibration(CAL_FILE))
    with pytest.raises(KeyboardInterrupt):
        le.write(bus, p['writes'], current)
    assert sdk.regs[1][lb.ADDR_LOCK] == 1


def test_locked_eeprom_shows_up_in_the_read_back(tmp_path):
    """A servo that acks but ignores the write (still locked) is a read-back mismatch."""
    bus, sdk = leader_today()
    real = bus.write_register
    bus.write_register = lambda sid, addr, v, size: (
        None if (sid == 4 and addr == lb.ADDR_LOCK and v == 0) else real(sid, addr, v, size))
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'p.yaml')], bus)
    assert rc == 1 and 'servo 4: register 55 := 0 not confirmed' in text
    assert sdk.regs[4][lb.ADDR_MIN_POS_LIMIT] == 865            # stopped before writing it


def test_present_not_as_expected_is_reported(tmp_path):
    bus, sdk = leader_today()
    real = bus.write_register

    def touched(sid, addr, v, size):
        real(sid, addr, v, size)
        if sid == 5 and addr == lb.ADDR_HOMING_OFFSET:
            sdk.regs[5][lb.ADDR_PRESENT] += 30          # someone moved limb5 meanwhile
    bus.write_register = touched
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'p.yaml')], bus)
    assert rc == 1 and 'expected 1940 after the shift' in text


# ------------------------------------------------------------------ aligner after the step

def test_aligner_after_the_step_full_range_registers_do_not_bind_beyond_travel():
    """Item 3: with 0..4095 the register check is only the wrap guard; the travel
    (with margin) is the bound. limb5 +2.0 rad was refused before the shift."""
    cal = load_calibration(CAL_FILE)
    shifted = Calibration([*cal.zero_ticks[:4], 2027, cal.zero_ticks[5]], cal.sign,
                          cal.ticks_per_rev, True, cal.range_center_rad,
                          cal.travel_min_rad, cal.travel_max_rad).select([1, 2, 3, 4, 5])
    old = cal.select([1, 2, 3, 4, 5])
    k = 4096 / (2 * math.pi)
    present = [1726, 2036, 2662, 337, 1940]
    diag = {sid: {'min_pos': 0, 'max_pos': 4095} for sid in range(1, 6)}
    bus = LeaderBus('/dev/null', ids=(1, 2, 3, 4, 5), sdk=make_sdk(dict(zip(range(1, 6), present))))
    q = [0.0, -1.0, 1.0, -1.0, 2.0]
    cfg = AlignConfig(activation_tolerance_rad=ACT)
    ends = Aligner(bus, shifted, cfg, NAMES, LIMITS).plan(present, q, diag)
    assert ends[4] == pytest.approx(1940 + (2.0 - shifted.to_radians(present)[4]) * k)
    with pytest.raises(AlignRefused, match='wrap'):
        Aligner(bus, old, cfg, NAMES, LIMITS).plan(
            [1726, 2036, 2662, 337, 3631], q, diag)
    # the travel band still binds limb5 at its recorded stops (user decision 2026-10-15:
    # clamped to the band edge when close enough, refused beyond)
    edge = shifted.travel_max_rad[4] - cfg.travel_margin_rad
    al = Aligner(bus, shifted, cfg, NAMES, [(-4, 4)] * 5)
    ends = al.plan(present, q[:4] + [2.93], diag)
    assert ends[4] == pytest.approx(1940 + (edge - shifted.to_radians(present)[4]) * k)
    with pytest.raises(AlignRefused, match='limb5: .*too far to clamp'):
        al.plan(present, q[:4] + [edge + ACT], diag)


def test_existing_snapshot_is_never_overwritten(tmp_path):
    bus, sdk = leader_today()
    snap = tmp_path / 'prev.yaml'
    snap.write_text('original')
    rc, text = run(['--write', '--save-previous', str(snap)], bus)
    assert rc == 1 and 'never overwritten' in text
    assert snap.read_text() == 'original' and sdk.writes == []



# ------------------------------------------------------------------ audit LA2

def shifted_calibration(tmp_path):
    """The calibration after step 7: the SHIPPED file since 2026-10-15."""
    return SHIPPED_CAL_FILE


def test_E1_dry_run_after_write_plans_nothing_and_second_write_is_a_no_op(tmp_path):
    bus, sdk = leader_today()
    assert run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)[0] == 0
    rc, text = run([], bus)
    assert rc == 0 and 'planned writes: none' in text
    assert "NOTE: id 5: EEPROM already shifted (414)" in text
    assert 'zero_ticks[4] 3718 -> 2027 and homing_offset_raw[4] 3325 -> 414' in text
    n = len(sdk.writes)
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'b.yaml')], bus)
    assert rc == 0 and 'nothing to write' in text and len(sdk.writes) == n
    assert sdk.regs[5][lb.ADDR_HOMING_OFFSET] == 414 and sdk.regs[5][lb.ADDR_PRESENT] == 1940
    assert not (tmp_path / 'b.yaml').exists()


def test_E1_after_step7_calibration_plans_nothing_without_notes(tmp_path):
    bus, sdk = leader_today()
    run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)
    lines = []
    rc = le.run(['--port', '/dev/null', '--calibration', str(shifted_calibration(tmp_path))],
                bus=bus, holders=lambda p: [], exclusive=lambda b: True, out=lines.append)
    text = '\n'.join(lines)
    assert rc == 0 and 'planned writes: none' in text
    assert 'NOTE' not in text and 'PROBLEM' not in text
    assert 'wrap shift D = 0 ticks' in text


def test_E1_torn_offset_is_refused_not_shifted_again(tmp_path):
    bus, sdk = leader_today()
    sdk.regs[5][lb.ADDR_HOMING_OFFSET] = 1000            # neither 3325 nor 414
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)
    assert rc == 1 and 'neither the calibration' in text and '--restore' in text
    assert sdk.writes == []


def test_E1_other_ids_offset_drift_is_refused(tmp_path):
    bus, sdk = leader_today()
    sdk.regs[2][lb.ADDR_HOMING_OFFSET] = 0
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)
    assert rc == 1 and 'id 2: Homing_Offset 0 != calibration 3353' in text
    assert sdk.writes == []


def test_E1_rollback_then_shifted_calibration_is_refused(tmp_path):
    """After a rollback the shifted calibration matches nothing: refused, not re-shifted."""
    bus, sdk = leader_today()
    lines = []
    rc = le.run(['--port', '/dev/null', '--calibration', str(shifted_calibration(tmp_path)),
                 '--write', '--save-previous', str(tmp_path / 'x.yaml')],
                bus=bus, holders=lambda p: [], exclusive=lambda b: True, out=lines.append)
    text = '\n'.join(lines)
    assert rc == 1 and 'neither the calibration' in text and sdk.writes == []
    assert 'zero_ticks[4] = 3718 and homing_offset_raw[4] = 3325' in text


def test_failed_relock_shows_in_the_read_back(tmp_path):
    """LOW: the Lock write is acknowledged but not applied - only the read-back sees it."""
    bus, sdk = leader_today()
    real = bus.write_register

    def stuck_open(sid, addr, v, size):
        real(sid, addr, 0 if (sid == 3 and addr == lb.ADDR_LOCK) else v, size)
    bus.write_register = stuck_open
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)
    assert rc == 1 and 'id 3: Lock 0 (expected 1)' in text


# ------------------------------------------------------------------ audit LA3: half-done step 7

def half_edited(tmp_path, zero5, offset5):
    text = open(CAL_FILE).read()
    text = text.replace('zero_ticks: [1726, 2036, 2662, 337, 3718, 1986]',
                        f'zero_ticks: [1726, 2036, 2662, 337, {zero5}, 1986]')
    text = text.replace('homing_offset_raw: [3624, 3353, 1323, 1748, 3325, 4055]',
                        f'homing_offset_raw: [3624, 3353, 1323, 1748, {offset5}, 4055]')
    f = tmp_path / f'cal_{zero5}_{offset5}.yaml'
    f.write_text(text)
    return f


def run_cal(cal, bus, argv):
    lines = []
    rc = le.run(['--port', '/dev/null', '--calibration', str(cal)] + argv, bus=bus,
                holders=lambda p: [], exclusive=lambda b: True, out=lines.append)
    return rc, '\n'.join(lines)


@pytest.mark.parametrize('eeprom,expect', [(414, 'zero_ticks[4] = 2027 and homing_offset_raw[4] = 414'),
                                           (3325, 'zero_ticks[4] = 3718 and homing_offset_raw[4] = 3325')])
@pytest.mark.parametrize('zero5,offset5', [(2027, 3325), (3718, 414)])
def test_half_done_step7_is_refused_with_the_exact_pair(tmp_path, eeprom, expect, zero5, offset5):
    """Only zero_ticks[4] (or only homing_offset_raw[4]) edited: refused, the pair that
    matches the EEPROM is printed as one edit, no --restore advice, nothing written."""
    bus, sdk = leader_today()
    sdk.regs[5][lb.ADDR_HOMING_OFFSET] = eeprom
    cal = half_edited(tmp_path, zero5, offset5)
    rc, text = run_cal(cal, bus, ['--write', '--save-previous', str(tmp_path / 's.yaml')])
    assert rc == 1 and 'calibration pair inconsistent' in text and expect in text
    assert 'do NOT use --restore' in text and 'roll back with --restore' not in text
    assert sdk.writes == [] and not (tmp_path / 's.yaml').exists()


def test_consistent_pairs_pass_the_invariant():
    cal = load_calibration(CAL_FILE)
    assert cal.pair_problems() == []
    assert load_calibration(SHIPPED_CAL_FILE).pair_problems() == []
    import dataclasses
    after = dataclasses.replace(cal, zero_ticks=[*cal.zero_ticks[:4], 2027, None],
                                homing_offset_raw=[*cal.homing_offset_raw[:4], 414, None])
    assert after.pair_problems() == []
    half = dataclasses.replace(cal, zero_ticks=[*cal.zero_ticks[:4], 2027, None])
    assert list(half.broken_pairs()) == [4]



# ------------------------------------------------------------------ the shipped calibration
# (after the leader EEPROM step of 2026-10-15)

def test_shipped_calibration_is_the_post_shift_state():
    """The shipped file differs from the pre-shift fixture in exactly the limb5 pair, by
    the planned D = 1691 (zero) and offset 3325 -> 414, and the invariant holds."""
    shipped, pre = load_calibration(SHIPPED_CAL_FILE), load_calibration(CAL_FILE)
    assert pre.zero_ticks[4] == PRE_SHIFT_LIMB5['zero_ticks']
    assert (shipped.zero_ticks[4], shipped.homing_offset_raw[4]) == (2027, 414)
    assert (pre.zero_ticks[4] - shipped.zero_ticks[4]) == 1691
    for field in ('zero_ticks', 'homing_offset_raw'):
        a, b = getattr(shipped, field), getattr(pre, field)
        assert [i for i in range(6) if a[i] != b[i]] == [4], field
    assert shipped.pair_problems() == []


def test_shipped_limb5_wrap_is_already_in_the_gap():
    """wrap_shift on the shipped calibration: nothing left to shift, and the travel no
    longer crosses the 0/4095 wrap."""
    cal = load_calibration(SHIPPED_CAL_FILE)
    i = le.wrap_shift(cal.zero_ticks[4], cal.sign[4], cal.travel_min_rad[4],
                      cal.travel_max_rad[4])
    assert i['shift'] == 0 and i['new_zero'] == cal.zero_ticks[4] == 2027
    assert 0 <= i['new_lo'] < i['new_hi'] <= 4095
    assert abs(i['new_lo'] - (4095 - i['new_hi'])) <= 2
    assert i['margin_ticks'] > 140


def test_shipped_calibration_against_the_post_shift_registers_plans_no_writes():
    """The registers as the step leaves them (and as read back 2026-10-15): ids 1-4
    position limits 0..4095, id 5 Homing_Offset 414 - the shipped calibration plans no
    write, refuses nothing, and needs no edit."""
    sdk = make_sdk({1: 1726, 2: 2036, 3: 2662, 4: 337, 5: TODAY_PRESENT5 - 1691})
    for sid in range(1, 6):
        sdk.regs[sid][lb.ADDR_MIN_POS_LIMIT] = 0
        sdk.regs[sid][lb.ADDR_MAX_POS_LIMIT] = 4095
    assert sdk.regs[5][lb.ADDR_HOMING_OFFSET] == 414           # make_sdk reads the shipped file
    bus = LeaderBus('/dev/null', ids=le.IDS, sdk=sdk)
    p = le.plan(le.read_current(bus), load_calibration(SHIPPED_CAL_FILE))
    assert p['writes'] == [] and not p['why'], p
    lines = []
    rc = le.run(['--port', '/dev/null', '--calibration', SHIPPED_CAL_FILE], bus=bus,
                holders=lambda port: [], exclusive=lambda b: True, out=lines.append)
    text = '\n'.join(lines)
    assert rc == 0 and 'planned writes: none' in text
    assert 'NOTE' not in text and 'PROBLEM' not in text and sdk.writes == []


# ------------------------------------------------------------------ measured wrap behaviour

@pytest.mark.parametrize('present,expected', [(1825, 134), (3631, 1940), (5638, 3947)])
def test_shift_maps_the_recorded_limb5_span_without_modulo(tmp_path, present, expected):
    """Recording 2026-10-13: limb5 1825..5638 continuous -> 134..3947 after D = 1691,
    inside the planned 143..3953 band within the measurement (both ends ~9 ticks)."""
    bus, sdk = leader_today()
    sdk.regs[5][lb.ADDR_PRESENT] = present
    p = le.plan(le.read_current(bus), load_calibration(CAL_FILE))
    assert p['shift'][5]['expected_present'] == expected and not p['why']
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)
    assert rc == 0 and sdk.regs[5][lb.ADDR_PRESENT] == expected


def test_shift_refused_when_present_minus_d_would_be_negative(tmp_path):
    bus, sdk = leader_today()
    sdk.regs[5][lb.ADDR_PRESENT] = 1500          # below the recorded travel: implausible
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'a.yaml')], bus)
    assert rc == 1 and 'outside 0..4095' in text and sdk.writes == []


# ------------------------------------------------------------------ robot run 2026-10-15
# `leader_eeprom --write --save-previous ~/leader_eeprom_before_wrap_shift.yaml`: id1 min
# 760 -> 0 written, the next write (id1 max) failed, relock OK, then read_current died
# with IndexError in the SDK's read2ByteTxRx (a COMM_SUCCESS reply without data bytes).

class BusyAfterEeprom:
    """Wraps the mock PacketHandler: for `busy_s` after a write to an EEPROM register
    (address < Torque_Enable) or to Lock, every transaction fails the way the SDK does
    on a short reply - reads raise IndexError, writes are not applied and raise too."""

    def __init__(self, pkt, busy_s):
        import time
        self._pkt, self._busy_s, self._time = pkt, busy_s, time.monotonic
        self._until = 0.0
        self.dropped = 0

    def _busy(self):
        if self._time() < self._until:
            self.dropped += 1
            raise IndexError('list index out of range')

    def _w(self, port, sid, addr, val):
        self._busy()
        r = self._pkt._w(port, sid, addr, val)
        if addr < lb.ADDR_TORQUE or addr == lb.ADDR_LOCK:
            self._until = self._time() + self._busy_s
        return r

    write1ByteTxRx = write2ByteTxRx = _w

    def _r(self, port, sid, addr):
        self._busy()
        return self._pkt._r(port, sid, addr)

    read1ByteTxRx = read2ByteTxRx = _r

    def ping(self, *a):
        return self._pkt.ping(*a)


def busy_leader(busy_s=0.03):
    bus, sdk = leader_today()
    bus._pkt = BusyAfterEeprom(bus._pkt, busy_s)
    return bus, sdk


def test_short_reply_is_a_read_error_not_a_traceback():
    bus, sdk = busy_leader(10.0)
    bus._pkt._until = float('inf')
    with pytest.raises(lb.LeaderReadError, match='malformed or missing reply'):
        bus.read_word(1, lb.ADDR_MIN_POS_LIMIT)
    with pytest.raises(LeaderWriteError, match='malformed or missing reply'):
        bus.write_register(1, lb.ADDR_LOCK, 0, 1)
    current = le.read_current(bus)                    # the crash site of 2026-10-15
    assert current[1]['min'] is None


def test_without_settle_the_robot_failure_is_reported_relocked_and_survives(tmp_path):
    """settle 0 reproduces the robot run: the packet after each EEPROM write is lost.
    No traceback; the run stops, reports, and every confirmed state is visible."""
    bus, sdk = busy_leader(0.05)
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'p.yaml'),
                    '--settle-sec', '0'], bus)
    assert rc == 1 and 'not confirmed' in text
    assert not [w for w in sdk.writes if w[0] in (2, 3, 4, 5)]          # stopped at id 1


def test_settle_and_read_back_write_through_a_busy_servo(tmp_path):
    bus, sdk = busy_leader(0.03)
    rc, text = run(['--write', '--save-previous', str(tmp_path / 'p.yaml'),
                    '--settle-sec', '0.05'], bus)
    assert rc == 0, text
    assert 'read-back OK' in text and bus._pkt.dropped == 0
    for sid in le.IDS:
        assert (sdk.regs[sid][lb.ADDR_MIN_POS_LIMIT], sdk.regs[sid][lb.ADDR_MAX_POS_LIMIT]) \
            == (0, 4095)
        assert sdk.regs[sid][lb.ADDR_LOCK] == 1
    assert sdk.regs[5][lb.ADDR_HOMING_OFFSET] == 414


def test_lost_ack_with_the_value_in_place_counts_as_written():
    bus, sdk = leader_today()
    real = bus.write_register

    def lost_ack(sid, addr, v, size):
        real(sid, addr, v, size)
        raise LeaderWriteError('ack lost')
    bus.write_register = lost_ack
    bus.write_verified(1, lb.ADDR_LOCK, 0, 1, 0.0, 3)
    assert sdk.regs[1][lb.ADDR_LOCK] == 0


def test_rerun_after_the_partial_robot_write_keeps_the_original_snapshot(tmp_path):
    """The robot state: snapshot taken before id1 min 760 -> 0, nothing else changed.
    The re-run plans only what is left, keeps the snapshot byte for byte, and a later
    --restore from it brings back the true originals."""
    bus, sdk = leader_today()
    snap = tmp_path / 'leader_eeprom_before_wrap_shift.yaml'
    snap.write_text(yaml.safe_dump(le.snapshot(le.read_current(bus))))
    original = snap.read_text()
    sdk.regs[1][lb.ADDR_MIN_POS_LIMIT] = 0                       # the one write that landed
    rc, text = run(['--port', '/dev/null'], bus)                 # dry run
    assert 'id 1 min' not in text and 'id 1 max' in text
    rc, text = run(['--write', '--save-previous', str(snap), '--settle-sec', '0'], bus)
    assert rc == 0, text
    assert 'KEPT as the snapshot of record' in text and snap.read_text() == original
    assert (sdk.regs[1][lb.ADDR_MIN_POS_LIMIT], sdk.regs[1][lb.ADDR_MAX_POS_LIMIT]) == (0, 4095)
    rc, text = run(['--write', '--restore', str(snap), '--save-previous',
                    str(tmp_path / 'rb.yaml'), '--settle-sec', '0'], bus)
    assert rc == 0, text
    assert (sdk.regs[1][lb.ADDR_MIN_POS_LIMIT], sdk.regs[1][lb.ADDR_MAX_POS_LIMIT]) == (760, 3232)


def test_existing_snapshot_with_foreign_differences_is_refused(tmp_path):
    bus, sdk = leader_today()
    snap = tmp_path / 'prev.yaml'
    snap.write_text(yaml.safe_dump(le.snapshot(le.read_current(bus))))
    sdk.regs[3][lb.ADDR_MIN_POS_LIMIT] = 500                     # not a value of this plan
    rc, text = run(['--write', '--save-previous', str(snap), '--settle-sec', '0'], bus)
    assert rc == 1 and 'differ from it by more than this plan writes' in text
    assert 'id 3 min: now 500, snapshot 873, plan 0' in text and sdk.writes == []
