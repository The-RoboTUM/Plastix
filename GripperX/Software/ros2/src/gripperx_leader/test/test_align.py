"""Leader self-alignment (user decision 2026-10-13), hardware-free.

The REAL LeaderBus runs over a register-level mock of scservo_sdk: torque, goal, torque
limit and present position live in a per-servo register dict; with torque on, present
follows the last goal unless the test holds the servo (operator, obstruction).
Register defaults mirror the leader as measured 2026-10-13 (coordinator, audit LA1):
7.4 V supply, Present_Voltage 76 raw, Max_Voltage_Limit 120 on IDs 1,3,4,5 and 80 on 2."""

import atexit
import dataclasses
import math
import os
import shutil
import tempfile
import time
import types

import pytest
import yaml

from gripperx_geometry.constants import ARM_JOINT_LOWER, ARM_JOINT_UPPER
from gripperx_leader import leader_bus as lb
from gripperx_leader.aligner import AlignAbort, AlignConfig, Aligner, AlignRefused
from gripperx_leader.calibration import Calibration, load_calibration
from gripperx_leader.follower_config import activation_tolerance
from gripperx_leader.leader_bus import LeaderBus

K = 4096 / (2 * math.pi)
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# The follower's own config, the single source of the activation tolerance.
ARM_YAML = os.path.join(os.path.dirname(HERE), 'gripperx_arm', 'config',
                        'arm_safety_filter.yaml')
ACT = activation_tolerance(ARM_YAML)


SHIPPED_CAL_FILE = os.path.join(HERE, 'config', 'leader_calibration.yaml')
# The shipped calibration's pre-shift state: limb5 as it was before the leader EEPROM
# step (registers read 2026-10-13; the step was written 2026-10-15 and the shipped file
# changed with it - zero_ticks[4] 3718 -> 2027, homing_offset_raw[4] 3325 -> 414). A
# fixture for the tests that exercise the shift itself or the pre-shift wrap; every
# other value is read from the shipped file.
PRE_SHIFT_LIMB5 = {'zero_ticks': 3718, 'homing_offset_raw': 3325}


def _write_pre_shift(directory):
    with open(SHIPPED_CAL_FILE, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    for key, value in PRE_SHIFT_LIMB5.items():
        data[key][4] = value
    path = os.path.join(directory, 'leader_calibration_pre_shift.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
    return path


_PRE_SHIFT_DIR = tempfile.mkdtemp(prefix='leader_cal_pre_shift_')
atexit.register(shutil.rmtree, _PRE_SHIFT_DIR, True)
PRE_SHIFT_CAL_FILE = _write_pre_shift(_PRE_SHIFT_DIR)

# EEPROM Homing_Offsets as recorded in the shipped calibration (limb5 since 2026-10-15).
RECORDED_OFFSETS = dict(zip(range(1, 7), load_calibration(
    SHIPPED_CAL_FILE).homing_offset_raw))


def make_sdk(present):
    """present: {id: tick}. Returns an sdk namespace with .regs, .writes, fault knobs."""
    regs = {sid: {lb.ADDR_PRESENT: t, lb.ADDR_GOAL: t, lb.ADDR_TORQUE: 0,
                  lb.ADDR_TORQUE_LIMIT: 1000, lb.ADDR_MODEL: 777, lb.ADDR_PRESENT_VOLTAGE: 76, lb.ADDR_PRESENT_LOAD: 0,
                  lb.ADDR_MIN_VOLTAGE: 40, lb.ADDR_MAX_VOLTAGE: 80 if sid == 2 else 120,
                  lb.ADDR_OPERATING_MODE: 0, lb.ADDR_MIN_POS_LIMIT: 0,
                  lb.ADDR_MAX_POS_LIMIT: 4095,
                  lb.ADDR_HOMING_OFFSET: RECORDED_OFFSETS.get(sid, 0), lb.ADDR_LOCK: 1,
                  lb.ADDR_CW_DEAD_ZONE: 1, lb.ADDR_CCW_DEAD_ZONE: 1}
            for sid, t in present.items()}
    sdk = types.SimpleNamespace(COMM_SUCCESS=0, regs=regs, writes=[], held=set(),
                                fail_write=set(), fail_read=False, torque_stuck=set(),
                                jump_on_torque={}, status={})

    def follow():
        for sid, r in regs.items():
            if r[lb.ADDR_TORQUE] and sid not in sdk.held:
                r[lb.ADDR_PRESENT] = r[lb.ADDR_GOAL]

    class PortHandler:
        def __init__(self, name): pass
        def openPort(self): return True
        def setBaudRate(self, b): return True
        def closePort(self): pass

    class PacketHandler:
        def __init__(self, proto): pass

        def ping(self, port, sid):
            return 0, (0 if sid in regs else -1), 0

        def _w(self, port, sid, addr, val):
            sdk.writes.append((sid, addr, val))
            if (sid, addr) in sdk.fail_write or sid not in regs:
                return -1, 0
            if addr < lb.ADDR_TORQUE and regs[sid][lb.ADDR_LOCK] != 0:
                return 0, sdk.status.get(sid, 0)        # EEPROM locked: acked, ignored
            if addr == lb.ADDR_HOMING_OFFSET:            # Present = Actual - Homing_Offset
                def dec(r): return -(r & 2047) if r & 2048 else r & 2047
                shift = dec(val) - dec(regs[sid][addr])
                # no modulo: limb5 measured 2026-10-13 runs continuously past 4095
                regs[sid][lb.ADDR_PRESENT] = regs[sid][lb.ADDR_PRESENT] - shift
            if not (addr == lb.ADDR_TORQUE and val == 0 and sid in sdk.torque_stuck):
                regs[sid][addr] = val
            if addr == lb.ADDR_TORQUE and val == 1 and sid in sdk.jump_on_torque:
                sdk.held.add(sid)                       # a servo that lurches on enable
                regs[sid][lb.ADDR_PRESENT] += sdk.jump_on_torque[sid]
            return 0, sdk.status.get(sid, 0)

        write1ByteTxRx = _w
        write2ByteTxRx = _w

        def _r(self, port, sid, addr):
            if sdk.fail_read or sid not in regs:
                return 0, -1, 0
            return regs[sid][addr], 0, sdk.status.get(sid, 0)

        read1ByteTxRx = _r
        read2ByteTxRx = _r

    class GroupSyncRead:
        def __init__(self, port, ph, addr, length): self.ids = []
        def addParam(self, sid): self.ids.append(sid); return True

        def txRxPacket(self):
            follow()
            return -1 if sdk.fail_read else 0

        def isAvailable(self, sid, addr, length): return sid in regs
        def getData(self, sid, addr, length): return regs[sid][lb.ADDR_PRESENT]

    sdk.PortHandler, sdk.PacketHandler, sdk.GroupSyncRead = PortHandler, PacketHandler, GroupSyncRead
    return sdk


IDS = (1, 2, 3, 4, 5)
NAMES = ['limb1', 'limb2', 'limb3', 'limb4', 'limb5']
# The shipped leader calibration, ids 1..5 (zeros near the wrap on limb4/limb5, travel).
CAL = load_calibration(os.path.join(HERE, 'config', 'leader_calibration.yaml')).select(list(IDS))
ZERO = dict(zip(IDS, CAL.zero_ticks))
LIMITS = [(ARM_JOINT_LOWER[i - 1], ARM_JOINT_UPPER[i - 1]) for i in IDS]
Q = [0.3, -0.2, 0.2, -0.1, 0.05]            # inside follower limits and leader travel


def bus_at(present=None):
    sdk = make_sdk(dict(present or ZERO))
    bus = LeaderBus('/dev/null', ids=IDS, sdk=sdk)
    return bus, sdk


def aligner(bus, cal=CAL, limits=LIMITS, **cfg):
    cfg.setdefault('activation_tolerance_rad', ACT)
    return Aligner(bus, cal, AlignConfig(**cfg), NAMES, limits)


def ticks_at(q, cal=CAL):
    """Leader present ticks at leader radians q (inverse of the calibration)."""
    return {sid: round(z + s * v * K) % 4096 for sid, z, s, v in zip(IDS, cal.zero_ticks,
                                                                       cal.sign, q)}


def run(al, bus, sdk, follower, t0=0.0, dt=1 / 30, max_s=30.0, hook=None):
    """Tick like the node: read, then step. Returns (arrived, t, goal history)."""
    t, hist = t0, []
    while t - t0 < max_s:
        t += dt
        if hook:
            hook(t)
        ticks = bus.read_positions()
        if al.step(t, ticks, follower, t):
            return True, t, hist
        if al.phase == 'ramp':
            hist.append([sdk.regs[s][lb.ADDR_GOAL] for s in IDS])
    return False, t, hist


def writes_of(sdk, addr, val=None):
    return [w for w in sdk.writes if w[1] == addr and (val is None or w[2] == val)]


def started(q=Q, **cfg):
    """begin + the settling step: torque on, RAMP running."""
    bus, sdk = bus_at()
    al = aligner(bus, **cfg)
    assert al.begin(0.0, bus.read_positions(), q, bus.read_diagnostics())
    assert sdk.writes == [] and al.phase == 'settling'    # begin writes nothing
    al.step(0.15, bus.read_positions(), q, 0.15)
    assert al.phase == 'ramp'
    return bus, sdk, al


def assert_limp(sdk, al):
    assert not al.active and al.unconfirmed == []
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)


# ------------------------------------------------------------------ target computation

def test_target_ticks_inverse_of_calibration_including_wrap():
    """The wrap handling, on the pre-shift calibration (limb5 zero 3718 near the wrap)."""
    cal = load_calibration(PRE_SHIFT_CAL_FILE).select(list(IDS))
    bus, _ = bus_at(dict(zip(IDS, cal.zero_ticks)))
    al = aligner(bus, cal=cal)
    diag = bus.read_diagnostics()
    q = [0.5, -1.0, 1.2, -1.0, -0.3]
    ends = al.plan(list(cal.zero_ticks), q, diag)
    assert cal.to_radians([round(e) % 4096 for e in ends]) == pytest.approx(q, abs=1.0 / K)
    # limb5 zero 3718, sign +1: +0.5 rad = tick 4044 (inside), +0.7 -> 4174 crosses the wrap
    al.plan(list(cal.zero_ticks), [0, -1.0, 1.2, -1.0, 0.5], diag)
    with pytest.raises(AlignRefused, match='wrap'):
        al.plan(list(cal.zero_ticks), [0, -1.0, 1.2, -1.0, 0.7], diag)
    # present already across the wrap (limb5 at tick 100 = +0.733 rad): path stays inside
    present = list(cal.zero_ticks)
    present[4] = 100
    ends = al.plan(present, [0, -1.0, 1.2, -1.0, 0.8], diag)
    assert 0 <= round(ends[4]) <= 4095 and ends[4] > 100


def test_shipped_limb5_reaches_its_travel_without_the_wrap():
    """The shipped calibration (after the 2026-10-15 EEPROM step): the same +0.7 rad, and
    limb5 out to both ends of its recorded travel, stay inside 0..4095 - the wrap now
    lies in the unreachable gap."""
    bus, _ = bus_at()
    al = aligner(bus, limits=[(-4.0, 4.0)] * 5)
    diag = bus.read_diagnostics()
    margin = AlignConfig(activation_tolerance_rad=ACT).travel_margin_rad
    for q5 in (0.7, CAL.travel_min_rad[4] + margin, CAL.travel_max_rad[4] - margin):
        ends = al.plan(list(CAL.zero_ticks), [0, -1.0, 1.2, -1.0, q5], diag)
        assert 0 <= round(ends[4]) <= 4095, (q5, ends[4])
        assert CAL.to_radians([round(e) for e in ends])[4] == pytest.approx(q5, abs=1.0 / K)


def test_target_outside_window_refused():
    bus, _ = bus_at()
    wide = Calibration([2048] * 5, [1] * 5, verified=True, range_center_rad=[0.0] * 5,
                       travel_min_rad=[-4.0] * 5, travel_max_rad=[4.0] * 5)
    al = aligner(bus, cal=wide, limits=[(-4.0, 4.0)] * 5)
    with pytest.raises(AlignRefused, match='window'):
        al.plan([2048] * 5, [3.3, 0, 0, 0, 0], bus.read_diagnostics())


def test_target_near_recorded_stop_or_missing_travel_refused():
    bus, _ = bus_at()
    diag = bus.read_diagnostics()
    # follower limits opened: on the shipped limits limb1's band edge is out of reach
    al = aligner(bus, travel_margin_rad=0.05, limits=[(-4.0, 4.0)] * 5)
    hi1 = CAL.travel_max_rad[0]                  # limb1: leader stop +2.1322 rad
    al.plan(list(CAL.zero_ticks), [hi1 - 0.06] + Q[1:], diag)
    assert al.clamped == []
    al.plan(list(CAL.zero_ticks), [hi1 - 0.04] + Q[1:], diag)
    assert al.clamped == [f'limb1 {hi1 - 0.04:.3f} -> {hi1 - 0.05:.3f}']
    # beyond the band edge by more than activation - arrival: refused, not clamped
    far = hi1 - 0.05 + (ACT - 0.035) + 0.01
    with pytest.raises(AlignRefused, match='too far to clamp'):
        al.plan(list(CAL.zero_ticks), [far] + Q[1:], diag)
    no_travel = Calibration(CAL.zero_ticks, CAL.sign, verified=True,
                            range_center_rad=CAL.centers())
    with pytest.raises(AlignRefused, match='no recorded leader travel'):
        aligner(bus, cal=no_travel).plan(list(CAL.zero_ticks), Q, diag)


def test_follower_outside_its_own_limits_refused():
    """Audit probe P4: follower limb3 at -0.05 rad (below its own lower limit)."""
    bus, sdk = bus_at()
    al = aligner(bus)
    with pytest.raises(AlignRefused, match='follower joint limits'):
        al.begin(0.0, bus.read_positions(), [0.0, -1.0, -0.05, -1.0, 0.0], bus.read_diagnostics())
    assert sdk.writes == []
    with pytest.raises(AlignRefused, match='follower joint limits unknown'):
        aligner(bus, limits=None).plan(list(CAL.zero_ticks), Q, bus.read_diagnostics())


# ------------------------------------------------------------------ ramp

def test_ramp_rate_and_simultaneous_arrival_then_torque_off_verified():
    bus, sdk, al = started(max_joint_velocity_rad_s=0.3)
    # torque limit (300, never raised), goal := present, torque on - in that order
    for sid in IDS:
        seq = [(a, v) for s, a, v in sdk.writes if s == sid]
        assert seq[:3] == [(lb.ADDR_TORQUE_LIMIT, 300), (lb.ADDR_GOAL, ZERO[sid]),
                           (lb.ADDR_TORQUE, 1)]
    assert al.duration == pytest.approx(0.3 / 0.3, abs=0.01)     # limb1 farthest: 0.3 rad
    arrived, t, hist = run(al, bus, sdk, Q, t0=0.15)
    assert arrived and t == pytest.approx(1.15, abs=0.1)
    rows = [list(ZERO.values())] + hist
    for a, b in zip(rows, rows[1:]):
        assert max(abs(x - y) for x, y in zip(a, b)) <= 0.3 / 30 * K + 1.01
    ends = [round(e) for e in al._ends]
    first = [next(i for i, r in enumerate(hist) if r[j] == ends[j]) for j in range(5)
             if ends[j] != ZERO[IDS[j]]]
    assert len(set(first)) == 1
    assert_limp(sdk, al)
    assert all(sdk.regs[s][lb.ADDR_TORQUE_LIMIT] == 1000 for s in IDS)   # restored
    assert CAL.to_radians(bus.read_positions()) == pytest.approx(Q, abs=0.035)


def test_torque_limit_is_never_raised():
    bus, sdk = bus_at()
    for s in IDS:
        sdk.regs[s][lb.ADDR_TORQUE_LIMIT] = 200
    al = aligner(bus, torque_limit=300)
    al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    al.step(0.15, bus.read_positions(), Q, 0.15)
    assert {v for _, _, v in writes_of(sdk, lb.ADDR_TORQUE_LIMIT)} == {200}


def test_already_aligned_writes_nothing():
    at_q = {sid: round(z + s * q * K) % 4096 for sid, z, s, q in zip(IDS, CAL.zero_ticks, CAL.sign, Q)}
    bus, sdk = bus_at(at_q)
    al = aligner(bus)
    q = list(Q)
    q[0] += 0.01
    assert al.begin(0.0, bus.read_positions(), q, bus.read_diagnostics()) is False
    assert sdk.writes == [] and not al.active and al.direct


# ------------------------------------------------------------------ 2026-10-15 decisions

# Robot finding 2026-10-15: follower at home, leader physically at home. limb3's home
# lies at its recorded travel end (-0.0123), and the old travel check refused it.
FINDING_LEADER = [-0.012, -0.035, -0.009, 0.057, -0.069]
FINDING_FOLLOWER = [0.0, 0.0, -0.006, 0.0, 0.0]


def test_home_at_travel_end_goes_direct_and_nothing_moves():
    """Decision 1: every joint within the activation tolerance -> direct, no motion,
    even with the MOTION guards failing (voltage out of band, torque on, status bits)."""
    bus, sdk = bus_at(ticks_at(FINDING_LEADER))
    for s in IDS:
        sdk.regs[s][lb.ADDR_PRESENT_VOLTAGE] = 99          # 9.9 V: outside the band
    sdk.regs[2][lb.ADDR_TORQUE] = 1
    sdk.status[4] = 0x20
    al = aligner(bus)
    ticks = bus.read_positions()
    assert al.begin(0.0, ticks, FINDING_FOLLOWER, bus.read_diagnostics()) is False
    assert al.direct and not al.active
    assert sdk.writes == []
    # the leader angles really are those of the finding (limb3 inside the margin)
    assert CAL.to_radians(ticks) == pytest.approx(FINDING_LEADER, abs=1.5 / K)
    assert CAL.to_radians(ticks)[2] < CAL.travel_min_rad[2] + 0.05


def test_direct_path_still_guards_the_enable_itself():
    """Homing_Offset mismatch and a missing activation tolerance refuse the direct path."""
    bus, sdk = bus_at(ticks_at(FINDING_LEADER))
    sdk.regs[5][lb.ADDR_HOMING_OFFSET] = PRE_SHIFT_LIMB5['homing_offset_raw']
    with pytest.raises(AlignRefused, match='Homing_Offset 3325'):
        aligner(bus).begin(0.0, bus.read_positions(), FINDING_FOLLOWER, bus.read_diagnostics())
    bus, sdk = bus_at(ticks_at(FINDING_LEADER))
    with pytest.raises(AlignRefused, match='activation tolerance unknown'):
        aligner(bus, activation_tolerance_rad=None).begin(
            0.0, bus.read_positions(), FINDING_FOLLOWER, bus.read_diagnostics())
    assert sdk.writes == []


def test_one_joint_beyond_activation_tolerance_is_not_direct():
    leader = list(FINDING_LEADER)
    leader[0] = FINDING_FOLLOWER[0] + ACT + 0.02
    bus, sdk = bus_at(ticks_at(leader))
    al = aligner(bus)
    assert al.begin(0.0, bus.read_positions(), FINDING_FOLLOWER, bus.read_diagnostics())
    assert not al.direct and al.phase == 'settling' and sdk.writes == []


def _narrow_limb1(hi=0.4):
    """limb1 recorded travel ending at `hi`: the band edge is hi - margin."""
    tmax = list(CAL.travel_max_rad)
    tmax[0] = hi
    return dataclasses.replace(CAL, travel_max_rad=tmax)


def test_target_beyond_band_edge_is_clamped_then_arrives():
    """Decision 2: follower 0.02 rad beyond the band edge -> the leader moves to the
    edge (arrival checked against the edge), then leader mode can be requested."""
    cal = _narrow_limb1()
    edge = cal.travel_max_rad[0] - 0.05
    follower = [edge + 0.02] + Q[1:]
    leader = [-0.3] + Q[1:]                       # limb1 far off: a real motion
    bus, sdk = bus_at(ticks_at(leader, cal))
    al = aligner(bus, cal=cal)
    assert al.begin(0.0, bus.read_positions(), follower, bus.read_diagnostics())
    assert al.clamped and al.clamped[0].startswith('limb1')
    al.step(0.15, bus.read_positions(), follower, 0.15)
    assert al.phase == 'ramp'
    arrived, _, hist = run(al, bus, sdk, follower, t0=0.15)
    assert arrived
    final = cal.to_radians(bus.read_positions())
    assert final[0] == pytest.approx(edge, abs=0.035)
    assert max(cal.to_radians(row)[0] for row in hist) <= edge + 1.0 / K   # never past the edge
    assert abs(final[0] - follower[0]) <= ACT      # the follower can accept leader mode
    assert_limp(sdk, al)


def test_target_too_far_beyond_band_edge_is_refused():
    cal = _narrow_limb1()
    edge = cal.travel_max_rad[0] - 0.05
    bus, sdk = bus_at(ticks_at([-0.3] + Q[1:], cal))
    al = aligner(bus, cal=cal)
    follower = [edge + ACT] + Q[1:]               # beyond activation - arrival
    with pytest.raises(AlignRefused, match='limb1: .*too far to clamp'):
        al.begin(0.0, bus.read_positions(), follower, bus.read_diagnostics())
    assert sdk.writes == []


def test_non_moving_joint_outside_band_does_not_refuse_and_holds():
    """Decision 3: limb3 sits inside the margin of its recorded stop (outside the band),
    within the arrival tolerance of its clamped target -> not checked against the band,
    does not move, holds its fresh reading. The old check refused this (follower 0.0 is
    outside the band)."""
    edge3 = CAL.travel_min_rad[2] + 0.05
    leader = [0.3, -0.2, edge3 - 0.02, -0.1, 0.05]
    follower = [-0.1, -0.2, 0.0, -0.1, 0.05]      # limb1 0.4 rad off: not direct
    assert LIMITS[2][0] <= follower[2] < edge3
    leader_ticks = ticks_at(leader)
    bus, sdk = bus_at(leader_ticks)
    al = aligner(bus)
    assert CAL.to_radians(bus.read_positions())[2] < edge3      # really outside the band
    assert al.begin(0.0, bus.read_positions(), follower, bus.read_diagnostics())
    al.step(0.15, bus.read_positions(), follower, 0.15)
    assert al.phase == 'ramp'
    arrived, _, _ = run(al, bus, sdk, follower, t0=0.15)
    assert arrived
    goals3 = {v for s, a, v in sdk.writes if s == 3 and a == lb.ADDR_GOAL}
    assert goals3 == {leader_ticks[3]}            # only its own fresh reading
    assert al.clamped == []
    assert_limp(sdk, al)


def test_moving_joint_at_its_stop_moves_into_the_band_not_beyond():
    """limb3 at its stop (-0.009), follower 0.03 (more than the arrival tolerance away,
    below the band edge 0.0377), another joint far off: limb3 is driven to the band
    edge, away from the stop - never toward it."""
    leader = [0.3] + FINDING_LEADER[1:]
    follower = list(FINDING_FOLLOWER)
    follower[2] = 0.03
    bus, sdk = bus_at(ticks_at(leader))
    al = aligner(bus)
    assert al.begin(0.0, bus.read_positions(), follower, bus.read_diagnostics())
    al.step(0.15, bus.read_positions(), follower, 0.15)
    assert al.clamped and al.clamped[0].startswith('limb3')
    arrived, _, hist = run(al, bus, sdk, follower, t0=0.15)
    assert arrived
    edge3 = CAL.travel_min_rad[2] + 0.05
    limb3 = [CAL.to_radians(row)[2] for row in hist]
    assert min(limb3) >= FINDING_LEADER[2] - 1.5 / K and max(limb3) <= edge3 + 1.5 / K
    assert CAL.to_radians(bus.read_positions())[2] == pytest.approx(edge3, abs=0.035)


def test_non_moving_joint_still_needs_its_present_inside_the_servo_limits():
    """Torque comes on for every joint: a goal outside Min/Max_Position_Limit would be
    clamped by the firmware (a motion), so that check stays for non-moving joints."""
    leader = [0.3, -0.2, 0.2, -0.1, 0.05]
    follower = [-0.1, -0.2, 0.2, -0.1, 0.05]
    bus, sdk = bus_at(ticks_at(leader))
    sdk.regs[3][lb.ADDR_MIN_POS_LIMIT] = ticks_at(leader)[3] + 100
    sdk.regs[3][lb.ADDR_MAX_POS_LIMIT] = 4095
    with pytest.raises(AlignRefused, match='limb3: present tick .* outside the servo'):
        aligner(bus).begin(0.0, bus.read_positions(), follower, bus.read_diagnostics())
    assert sdk.writes == []


def test_activation_tolerance_is_read_from_the_follower_config():
    """Not a copy: the value is the arm's own leader.activation_match_tolerance_rad
    (user decision 2026-10-12: 10 deg), and a missing file/key is an error."""
    import yaml
    doc = yaml.safe_load(open(ARM_YAML))
    assert ACT == doc['arm_action_server']['ros__parameters']['leader'][
        'activation_match_tolerance_rad']
    assert ACT == pytest.approx(math.radians(10.0), abs=1e-5)
    with pytest.raises(ValueError):
        activation_tolerance(os.path.join(HERE, 'config', 'leader_arm.yaml'))
    with pytest.raises(ValueError):
        activation_tolerance('/nonexistent/arm_safety_filter.yaml')


# ------------------------------------------------------------------ refusals

@pytest.mark.parametrize('reg,val,match', [
    (lb.ADDR_PRESENT_VOLTAGE, 85, 'configured band'),    # 8.5 V: ID 3's own limit is 12.0 V
    (lb.ADDR_PRESENT_VOLTAGE, 60, 'configured band'),
    (lb.ADDR_PRESENT_VOLTAGE, 30, 'Voltage_Limit'),
    (lb.ADDR_OPERATING_MODE, 3, 'Operating_Mode'),
    (lb.ADDR_TORQUE, 1, 'already ON'),
    (lb.ADDR_MAX_POS_LIMIT, 0, 'position limits'),
])
def test_register_checks_refuse_before_any_write(reg, val, match):
    bus, sdk = bus_at()
    sdk.regs[3][reg] = val
    al = aligner(bus)
    with pytest.raises(AlignRefused, match=match):
        al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == [] and not al.active


def test_voltage_band_does_not_depend_on_one_servo_register():
    """Audit probe P5: 11.8 V on 7.4 V servos is refused whatever the registers say."""
    for id2max in (80, 120):
        bus, sdk = bus_at()
        for s in IDS:
            sdk.regs[s][lb.ADDR_PRESENT_VOLTAGE] = 118
            sdk.regs[s][lb.ADDR_MAX_VOLTAGE] = 120 if s != 2 else id2max
        with pytest.raises(AlignRefused):
            aligner(bus).begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
        assert sdk.writes == []


def test_voltage_band_is_mandatory():
    bus, sdk = bus_at()
    with pytest.raises(AlignRefused, match='mandatory'):
        aligner(bus, voltage_min_v=0.0, voltage_max_v=0.0).begin(
            0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == []


def test_status_bits_refuse_before_torque_on():
    bus, sdk = bus_at()
    sdk.status[4] = 0x20
    with pytest.raises(AlignRefused, match='0x20'):
        aligner(bus).begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == []


# ------------------------------------------------------------------ settling (M2)

def test_leader_handled_before_torque_on_aborts_without_torque():
    """Audit probe P3: hand moves the leader between the pre-check and torque-on."""
    bus, sdk = bus_at()
    al = aligner(bus)
    al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    sdk.regs[2][lb.ADDR_PRESENT] += 60          # 0.092 rad
    with pytest.raises(AlignAbort, match='being handled'):
        al.step(0.15, bus.read_positions(), Q, 0.15)
    assert writes_of(sdk, lb.ADDR_TORQUE, 1) == [] and writes_of(sdk, lb.ADDR_GOAL) == []
    assert_limp(sdk, al)


def test_no_torque_before_still_check_elapsed():
    bus, sdk = bus_at()
    al = aligner(bus, still_check_sec=0.1)
    al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert al.step(0.05, bus.read_positions(), Q, 0.05) is False
    assert sdk.writes == [] and al.phase == 'settling'


def test_goal_is_the_fresh_reading_not_the_begin_reading():
    bus, sdk = bus_at()
    al = aligner(bus)
    al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    sdk.regs[1][lb.ADDR_PRESENT] += 5           # within the still tolerance (0.008 rad)
    al.step(0.15, bus.read_positions(), Q, 0.15)
    assert writes_of(sdk, lb.ADDR_GOAL)[0] == (1, lb.ADDR_GOAL, ZERO[1] + 5)


def test_jump_on_torque_on_is_detected():
    bus, sdk = bus_at()
    sdk.jump_on_torque[3] = 40                  # 0.061 rad lurch on enable
    al = aligner(bus)
    al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    with pytest.raises(AlignAbort, match='moved on torque-on'):
        al.step(0.15, bus.read_positions(), Q, 0.15)
    assert_limp(sdk, al)


# ------------------------------------------------------------------ aborts

def test_abort_on_deviation_operator_holds_the_leader():
    bus, sdk, al = started()
    sdk.held.add(1)
    with pytest.raises(AlignAbort, match='deviates'):
        run(al, bus, sdk, Q, t0=0.15)
    assert_limp(sdk, al)


def test_abort_on_stale_follower_pose():
    bus, sdk, al = started()
    with pytest.raises(AlignAbort, match='stale'):
        al.step(1.0, bus.read_positions(), Q, follower_rx=-1.0)
    assert_limp(sdk, al)


def test_abort_when_follower_moves():
    bus, sdk, al = started()
    moved = list(Q)
    moved[2] += 0.06
    with pytest.raises(AlignAbort, match='follower moved'):
        al.step(0.2, bus.read_positions(), moved, 0.2)
    assert_limp(sdk, al)


def test_abort_on_timeout():
    bus, sdk, al = started(settle_timeout_sec=1.0)
    end1 = round(al._ends[0])

    def hook(t):
        if sdk.regs[1][lb.ADDR_GOAL] > end1 - 0.05 * K:
            sdk.held.add(1)
            sdk.regs[1][lb.ADDR_PRESENT] = int(end1 - 0.05 * K)
    with pytest.raises(AlignAbort, match='timeout'):
        run(al, bus, sdk, Q, t0=0.15, hook=hook)
    assert_limp(sdk, al)


def test_abort_on_goal_write_error():
    bus, sdk, al = started()
    sdk.fail_write.add((4, lb.ADDR_GOAL))
    with pytest.raises(AlignAbort, match='goal write failed'):
        al.step(0.2, bus.read_positions(), Q, 0.2)
    assert_limp(sdk, al)


def test_torque_on_failure_aborts_and_releases():
    bus, sdk = bus_at()
    sdk.fail_write.add((3, lb.ADDR_TORQUE))
    al = aligner(bus)
    al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    with pytest.raises(AlignAbort, match='torque-on sequence failed'):
        al.step(0.15, bus.read_positions(), Q, 0.15)
    sdk.fail_write.clear()
    assert al.retry_release() == []
    assert_limp(sdk, al)


@pytest.mark.parametrize('fault,match', [
    ('voltage', 'configured band'),
    ('reset', 'Torque_Enable 0'),
    ('limit', 'Torque_Limit 1000'),
    ('bits', 'status error bits'),
])
def test_monitor_aborts_mid_ramp(fault, match):
    """One servo per tick is checked: within len(IDS) ticks every servo was seen."""
    bus, sdk, al = started()
    if fault == 'voltage':
        sdk.regs[4][lb.ADDR_PRESENT_VOLTAGE] = 62
    elif fault == 'reset':
        sdk.regs[4][lb.ADDR_TORQUE] = 0
        sdk.held.add(4)
    elif fault == 'limit':
        sdk.regs[4][lb.ADDR_TORQUE_LIMIT] = 1000
    else:
        sdk.status[4] = 0x20
    with pytest.raises(AlignAbort, match=match):
        run(al, bus, sdk, Q, t0=0.15, max_s=len(IDS) / 30 + 0.01)


def test_external_abort_turns_torque_off():
    bus, sdk, al = started()
    run(al, bus, sdk, Q, t0=0.15, max_s=0.3)
    assert 'torque off confirmed' in al.abort('Space (/arm/stop)')
    assert_limp(sdk, al)


def test_arrival_with_unconfirmed_torque_off_is_an_abort_then_retried():
    bus, sdk, al = started()
    sdk.torque_stuck.add(2)
    with pytest.raises(AlignAbort, match=r'NOT confirmed on servo\(s\) \[2\]'):
        run(al, bus, sdk, Q, t0=0.15)
    assert al.unconfirmed == [2]
    with pytest.raises(AlignRefused, match='still unconfirmed'):
        al.begin(5.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert al.retry_release() == [2]
    sdk.torque_stuck.clear()
    assert al.retry_release() == []
    assert_limp(sdk, al)


def test_status_bits_do_not_make_a_confirmed_torque_off_unconfirmed():
    """Audit probe P9: overload bit on servo 3, writes land, read-back 0."""
    bus, sdk = bus_at()
    sdk.regs[3][lb.ADDR_TORQUE] = 1
    sdk.status[3] = 0x20
    assert bus.torque_off_verified() == []
    assert sdk.regs[3][lb.ADDR_TORQUE] == 0 and bus.status_bits == {3: 0x20}


# ------------------------------------------------------------------ node

def _isolate():
    os.environ.setdefault('ROS_DOMAIN_ID', '224')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ.pop('ROS_STATIC_PEERS', None)


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


@pytest.fixture
def node_env(request):
    _isolate()
    import rclpy
    from gripperx_leader.leader_arm_node import LeaderArmNode
    extra = getattr(request, 'param', [])
    cal = os.path.join(HERE, 'config', 'leader_calibration.yaml')
    rclpy.init(args=['--ros-args', '-p', 'servo_ids:=[1,2,3,4,5]',
                     '-p', f'calibration_file:={cal}',
                     '-p', f'follower_safety_filter_file:={ARM_YAML}',
                     '-p', 'rate_hz:=1000.0', '-p', 'align_max_joint_velocity_rad_s:=20.0',
                     '-p', 'align_still_check_sec:=0.02'] + extra)
    bus, sdk = bus_at()
    node = LeaderArmNode(bus=bus)
    node._enable_pub = _Rec()
    node._align_state_pub = _Rec()
    yield node, bus, sdk
    node.destroy_node()
    rclpy.shutdown()


def _follower(node, q, times=2):
    from sensor_msgs.msg import JointState
    for _ in range(times):
        m = JointState()
        m.name = ['wheel_fl'] + [f'arm_limb{i}_joint' for i in range(1, 6)]
        m.position = [9.9] + list(q)
        node._on_follower(m)


def _bool(v):
    from std_msgs.msg import Bool
    return Bool(data=v)


def _drive(node, n=600):
    for _ in range(n):
        if not node._aligning():
            return
        _follower(node, Q, 1)
        node._tick()
        time.sleep(0.001)


def _into_ramp(node):
    for _ in range(200):
        _follower(node, Q, 1)
        node._tick()
        if node._aligner.phase == 'ramp':
            return
        time.sleep(0.002)
    raise AssertionError('never reached the ramp')


def test_node_writes_nothing_without_an_align_request(node_env):
    node, bus, sdk = node_env
    _follower(node, Q)
    for _ in range(20):
        node._tick()
    assert sdk.writes == []
    assert node.align_state == 'idle'


def test_node_arrival_publishes_exactly_one_enable_after_torque_off(node_env):
    node, bus, sdk = node_env
    _follower(node, Q)
    node._on_align(_bool(True))
    assert node._aligning() and node.align_state.startswith('aligning: checking')
    node._on_align(_bool(True))                # idempotent: no restart
    _drive(node)
    assert node._enable_pub.msgs == [True]
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)
    last_torque_write = max(i for i, w in enumerate(sdk.writes) if w[1] == lb.ADDR_TORQUE)
    assert sdk.writes[last_torque_write][2] == 0
    assert node.align_state.startswith('done') and 'max tick' in node.align_state
    assert any(m.startswith('aligning: leader to the follower pose')
               for m in node._align_state_pub.msgs)
    node._on_enable(_bool(True))               # its own echo: nothing aborts, nothing repeats
    assert node._enable_pub.msgs == [True]


def test_node_j_before_own_enable_echo_is_refused(node_env):
    """Audit probe P7: _enable_last is set when the node publishes, not on the echo."""
    node, bus, sdk = node_env
    _follower(node, Q)
    node._on_align(_bool(True))
    _drive(node)
    assert node._enable_pub.msgs == [True]
    sdk.regs[2][lb.ADDR_PRESENT] += 65           # sag after torque-off
    n_writes = len(sdk.writes)
    node._on_align(_bool(True))
    assert node.align_state == 'refused: leader mode already requested'
    assert not node._aligning() and len(sdk.writes) == n_writes


@pytest.mark.parametrize('trigger', ['space', 'second_j', 'enable_false', 'read_error',
                                     'shutdown', 'follower_stale'])
def test_node_every_abort_is_torque_off_and_no_enable(node_env, trigger):
    from std_msgs.msg import Empty
    node, bus, sdk = node_env
    node._aligner._cfg.max_joint_velocity_rad_s = 0.3      # slow: still running when hit
    _follower(node, Q)
    node._on_align(_bool(True))
    _into_ramp(node)
    assert writes_of(sdk, lb.ADDR_TORQUE, 1)
    if trigger == 'space':
        node._on_stop(Empty())
    elif trigger == 'second_j':
        node._on_align(_bool(False))
    elif trigger == 'enable_false':
        node._on_enable(_bool(False))
    elif trigger == 'read_error':
        sdk.fail_read = True
        node._tick()
        sdk.fail_read = False
    elif trigger == 'shutdown':
        node.shutdown()
    elif trigger == 'follower_stale':
        node._follower_rx = time.monotonic() - 5.0
        node._tick()
    assert not node._aligning()
    if trigger == 'read_error':                  # read-back impossible while reads fail
        node._tick()                              # healthy again: the retry confirms
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)
    assert True not in node._enable_pub.msgs
    assert node.align_state.startswith('aborted')
    assert 'confirmed' in node.align_state and node._aligner.unconfirmed == []


def test_node_retries_unconfirmed_torque_off_every_tick(node_env):
    """Audit probe P2: a torque-off that could not be confirmed is retried."""
    from std_msgs.msg import Empty
    node, bus, sdk = node_env
    node._aligner._cfg.max_joint_velocity_rad_s = 0.3
    _follower(node, Q)
    node._on_align(_bool(True))
    _into_ramp(node)
    sdk.torque_stuck = {2}
    node._on_stop(Empty())
    assert 'NOT confirmed' in node.align_state and sdk.regs[2][lb.ADDR_TORQUE] == 1
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: torque-off still unconfirmed')
    n_states = len(node._align_state_pub.msgs)
    for _ in range(5):
        node._tick()                              # still stuck: no state spam
    assert len(node._align_state_pub.msgs) == n_states
    sdk.torque_stuck = set()
    node._tick()
    assert sdk.regs[2][lb.ADDR_TORQUE] == 0
    assert node.align_state == 'aborted: torque off now confirmed (retry)'
    assert True not in node._enable_pub.msgs


def test_node_refuses_stale_missing_or_inconsistent_follower_pose(node_env):
    node, bus, sdk = node_env
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: no fresh follower pose')
    _follower(node, Q, times=1)
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: follower pose seen only once')
    _follower(node, [q + 0.2 for q in Q], times=1)      # second source / moving
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: follower pose inconsistent')
    _follower(node, Q)
    node._follower_rx = time.monotonic() - 2.0
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: no fresh')
    assert sdk.writes == [] and node._enable_pub.msgs == []


def test_node_refuses_while_leader_mode_already_requested(node_env):
    node, bus, sdk = node_env
    _follower(node, Q)
    node._on_enable(_bool(True))
    node._on_align(_bool(True))
    assert node.align_state == 'refused: leader mode already requested'
    assert sdk.writes == []


@pytest.mark.parametrize('node_env', [['-p', 'align_enabled:=false']], indirect=True)
def test_node_disabled_alignment_falls_back_to_direct_enable(node_env):
    node, bus, sdk = node_env
    node._on_align(_bool(True))                # no follower pose needed, nothing moves
    assert node._enable_pub.msgs == [True]
    assert sdk.writes == [] and node.align_state.startswith('direct')


# ------------------------------------------------------------------ shipped config

def test_shipped_leader_arm_yaml_keys_are_all_declared(node_env):
    """A renamed key in config/leader_arm.yaml would be silently ignored by rclpy."""
    import yaml
    node, _, _ = node_env
    params = yaml.safe_load(open(os.path.join(HERE, 'config', 'leader_arm.yaml')))
    keys = params['leader_arm_node']['ros__parameters']
    undeclared = [k for k in keys if not node.has_parameter(k)]
    assert undeclared == []
    assert 0 < keys['align_voltage_min_v'] < keys['align_voltage_max_v']


def test_shipped_calibration_has_travel_for_every_configured_joint():
    full = load_calibration(os.path.join(HERE, 'config', 'leader_calibration.yaml'))
    for i in range(6):                      # id 6 (gripper) since 2026-11-13
        lo, hi = full.travel_min_rad[i], full.travel_max_rad[i]
        assert lo is not None and hi is not None and lo < hi
        c = full.centers()[i]
        assert c - math.pi <= lo and hi < c + math.pi       # travel fits the radian window


def test_eeprom_position_limits_are_read_and_enforced():
    """Leader registers read 2026-10-13: limb4 Min 865 - its calibration zero 337 is below."""
    bus, sdk = bus_at()
    for sid, (lo, hi) in {1: (760, 3232), 2: (823, 3199), 3: (873, 3076),
                          4: (865, 3188), 5: (0, 4095)}.items():
        sdk.regs[sid][lb.ADDR_MIN_POS_LIMIT] = lo
        sdk.regs[sid][lb.ADDR_MAX_POS_LIMIT] = hi
    al = aligner(bus)
    with pytest.raises(AlignRefused, match='limb4: present tick 337 outside the servo position limits 865..3188'):
        al.begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == []



# ------------------------------------------------------------------ audit LA2

def test_offset_mismatch_refuses_alignment():
    """E2: EEPROM and calibration disagree - with the shipped (shifted) calibration, an
    EEPROM rolled back to the pre-shift offset without reverting the zero."""
    bus, sdk = bus_at()
    sdk.regs[5][lb.ADDR_HOMING_OFFSET] = PRE_SHIFT_LIMB5['homing_offset_raw']
    with pytest.raises(AlignRefused, match='Homing_Offset 3325 != calibration'):
        aligner(bus).begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == []


def test_calibration_without_recorded_offsets_refuses():
    bus, sdk = bus_at()
    no_rec = Calibration(CAL.zero_ticks, CAL.sign, verified=True, range_center_rad=CAL.centers(),
                         travel_min_rad=CAL.travel_min_rad, travel_max_rad=CAL.travel_max_rad)
    with pytest.raises(AlignRefused, match='no homing_offset_raw'):
        aligner(bus, cal=no_rec).begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())


def test_negative_present_is_a_read_error_but_above_4095_is_valid():
    """A sign-magnitude negative present (32768|m) is never read as a tick. 4096..32767
    is legitimate: limb5 measured 2026-10-13 reads 1825..5638 continuously."""
    bus, sdk = bus_at()
    sdk.regs[5][lb.ADDR_PRESENT] = 32768 | 200
    with pytest.raises(lb.LeaderReadError, match='sign bit'):
        bus.read_positions()
    sdk.regs[5][lb.ADDR_PRESENT] = 5638               # limb5 at its upper stop, pre-shift
    ticks = bus.read_positions()
    assert ticks[4] == 5638
    pre = load_calibration(PRE_SHIFT_CAL_FILE).select(list(IDS))
    assert pre.to_radians(ticks)[4] == pytest.approx(pre.travel_max_rad[4], abs=0.02)
    # the same stop after the shift (D = 1691): 3947, inside 0..4095, same angle shipped
    sdk.regs[5][lb.ADDR_PRESENT] = 5638 - 1691
    ticks = bus.read_positions()
    assert CAL.to_radians(ticks)[4] == pytest.approx(CAL.travel_max_rad[4], abs=0.02)


def test_limb5_beyond_4095_before_the_step_is_refused_not_mishandled():
    """Before the EEPROM step limb5 can sit above 4095: the aligner refuses (outside the
    servo position limits 0..4095), it does not command a goal there."""
    present = dict(ZERO)
    present[5] = 5000
    bus, sdk = bus_at(present)
    with pytest.raises(AlignRefused, match='limb5: present tick 5000 outside'):
        aligner(bus).begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == []


@pytest.mark.parametrize('extra', [[], ['-p', 'align_enabled:=false']])
def test_node_offset_mismatch_at_startup_reports_unverified_and_refuses(extra):
    """E2 at node level: EEPROM limb5 offset and the shipped calibration disagree (the
    EEPROM rolled back to the pre-shift offset, the calibration still shifted)."""
    _isolate()
    import rclpy
    from std_msgs.msg import Bool
    from gripperx_leader.leader_arm_node import ENABLE_QOS, LeaderArmNode
    cal = os.path.join(HERE, 'config', 'leader_calibration.yaml')
    rclpy.init(args=['--ros-args', '-p', 'servo_ids:=[1,2,3,4,5]',
                     '-p', f'calibration_file:={cal}'] + extra)
    try:
        bus, sdk = bus_at()
        sdk.regs[5][lb.ADDR_HOMING_OFFSET] = PRE_SHIFT_LIMB5['homing_offset_raw']
        node = LeaderArmNode(bus=bus)
        got = []
        node.create_subscription(Bool, '/arm/leader/calibration_verified',
                                 lambda m: got.append(m.data), ENABLE_QOS)
        t = time.time()
        while not got and time.time() - t < 3.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert got == [False]                       # the file itself says verified: true
        assert 'servo 5: EEPROM 3325, calibration 414' in node._offset_problem
        node._enable_pub = _Rec()
        node._align_state_pub = _Rec()
        _follower(node, Q)
        node._on_align(_bool(True))
        assert node.align_state.startswith('refused: Homing_Offset does not match')
        assert node._enable_pub.msgs == [] and sdk.writes == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_node_matching_offsets_report_verified(node_env):
    node, _, _ = node_env
    assert node._offset_problem is None


def test_node_half_edited_calibration_reports_unverified(tmp_path):
    """Audit LA3: the node refuses a half-done step 7 even when EEPROM and
    homing_offset_raw agree (offset edited, zero not) - from the pre-shift state."""
    _isolate()
    import rclpy
    from gripperx_leader.leader_arm_node import LeaderArmNode
    text = open(PRE_SHIFT_CAL_FILE).read()
    old = 'homing_offset_raw: [3624, 3353, 1323, 1748, 3325, 4055]'
    assert old in text
    text = text.replace(old, 'homing_offset_raw: [3624, 3353, 1323, 1748, 414, 4055]')
    cal = tmp_path / 'half.yaml'
    cal.write_text(text)
    rclpy.init(args=['--ros-args', '-p', 'servo_ids:=[1,2,3,4,5]',
                     '-p', f'calibration_file:={cal}'])
    try:
        bus, sdk = bus_at()
        sdk.regs[5][lb.ADDR_HOMING_OFFSET] = 414
        node = LeaderArmNode(bus=bus)
        assert 'zero_invariant_ticks says 2441' in node._offset_problem
        node._align_state_pub = _Rec()
        _follower(node, Q)
        node._on_align(_bool(True))
        assert node.align_state.startswith('refused: Homing_Offset does not match')
        assert sdk.writes == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_limb2_present_wrap_at_its_bottom_stop_streams_continuously():
    """Recording 2026-10-13: limb2 goes 4047, 4083, 21, 52 at its bottom stop (it wraps).
    The radians stay continuous (the difference to the zero is wrapped)."""
    rad = [CAL.to_radians([1726, t, 2662, 337, 3718])[1] for t in (4047, 4083, 21, 52)]
    steps = [b - a for a, b in zip(rad, rad[1:])]
    assert all(abs(d) < 0.1 for d in steps) and all(d < 0 for d in steps)   # sign -1
    assert rad[-1] == pytest.approx(-3.2, abs=0.2)


def test_limb2_beyond_its_wrap_is_refused_by_the_aligner():
    """limb2 at its bottom stop (present 21 = wrapped): any path back up would cross the
    wrap in tick space, which a position servo cannot be told to do - refused."""
    present = dict(ZERO)
    present[2] = 21
    bus, sdk = bus_at(present)
    with pytest.raises(AlignRefused, match='limb2: the path to the follower pose ends at tick'):
        aligner(bus).begin(0.0, bus.read_positions(), Q, bus.read_diagnostics())
    assert sdk.writes == []


# ------------------------------------------------------------------ node, 2026-10-15

def _drive_with(node, follower, n=600):
    for _ in range(n):
        if not node._aligning():
            return
        _follower(node, follower, 1)
        node._tick()
        time.sleep(0.001)


def _set_present(sdk, ticks):
    for sid, t in ticks.items():
        sdk.regs[sid][lb.ADDR_PRESENT] = sdk.regs[sid][lb.ADDR_GOAL] = t


def test_node_finding_home_at_travel_end_enables_directly(node_env):
    node, bus, sdk = node_env
    _set_present(sdk, ticks_at(FINDING_LEADER))
    sdk.regs[1][lb.ADDR_PRESENT_VOLTAGE] = 99            # a motion guard: must not block
    _follower(node, FINDING_FOLLOWER)
    node._on_align(_bool(True))
    assert node.align_state.startswith('direct: within the activation tolerance'), \
        node.align_state
    assert node._enable_pub.msgs == [True]
    assert sdk.writes == [] and not node._aligning()


def test_node_clamped_alignment_moves_to_the_edge_then_enables(node_env):
    node, bus, sdk = node_env
    cal = _narrow_limb1()
    node._cal = node._aligner._cal = cal
    edge = cal.travel_max_rad[0] - 0.05
    follower = [edge + 0.02] + Q[1:]
    _set_present(sdk, ticks_at([-0.3] + Q[1:], cal))
    _follower(node, follower)
    node._on_align(_bool(True))
    assert node._aligning() and 'clamped to the travel band: limb1' in node.align_state
    _drive_with(node, follower)
    assert node.align_state.startswith('done'), node.align_state
    assert node._enable_pub.msgs == [True]
    assert cal.to_radians(bus.read_positions())[0] == pytest.approx(edge, abs=0.035)


def test_node_clamp_too_far_is_refused_visibly(node_env):
    node, bus, sdk = node_env
    cal = _narrow_limb1()
    node._cal = node._aligner._cal = cal
    edge = cal.travel_max_rad[0] - 0.05
    _set_present(sdk, ticks_at([-0.3] + Q[1:], cal))
    _follower(node, [edge + ACT] + Q[1:])
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: limb1') and 'too far to clamp' in \
        node.align_state
    assert node._align_state_pub.msgs[-1] == node.align_state     # published, not only logged
    assert node._enable_pub.msgs == [] and sdk.writes == []


@pytest.mark.parametrize('node_env', [['-p', 'follower_safety_filter_file:=/nonexistent.yaml']],
                         indirect=True)
def test_node_without_the_activation_tolerance_refuses_every_align(node_env):
    node, bus, sdk = node_env
    _set_present(sdk, ticks_at(FINDING_LEADER))
    _follower(node, FINDING_FOLLOWER)
    node._on_align(_bool(True))
    assert node.align_state.startswith('refused: follower activation tolerance unknown')
    assert node._enable_pub.msgs == [] and sdk.writes == []


def test_default_activation_tolerance_source_is_the_installed_arm_config():
    """With follower_safety_filter_file "" the node reads gripperx_arm's installed
    config. Skipped where gripperx_arm is not installed (the node then refuses J)."""
    from gripperx_leader import follower_config as fc
    try:
        path = fc.default_path()
    except Exception:  # noqa: BLE001
        pytest.skip('gripperx_arm not installed in this environment')
    assert path.endswith(os.path.join('gripperx_arm', 'config', 'arm_safety_filter.yaml'))
    assert activation_tolerance('') == pytest.approx(activation_tolerance(path))


def test_joint_at_the_follower_outside_the_band_holds_without_clamp():
    """Audit LA4 M1: robot case limb3 leader -0.006 = follower -0.006, 0.044 rad below
    the band edge; limb1 more than 10 deg off forces the motion path. limb3 is decided
    against the UNCLAMPED follower angle: it holds its fresh tick, no clamp entry."""
    leader = [FINDING_FOLLOWER[0] + ACT + 0.1, -0.035, -0.006, 0.057, -0.069]
    follower = [0.0, 0.0, -0.006, 0.0, 0.0]
    edge3 = CAL.travel_min_rad[2] + 0.05
    leader_ticks = ticks_at(leader)
    bus, sdk = bus_at(leader_ticks)
    present3 = CAL.to_radians(bus.read_positions())[2]
    assert abs(present3 - follower[2]) <= 0.035 and edge3 - present3 > 0.035
    al = aligner(bus)
    assert al.begin(0.0, bus.read_positions(), follower, bus.read_diagnostics())
    assert al.clamped == []
    al.step(0.15, bus.read_positions(), follower, 0.15)
    assert al.phase == 'ramp' and al.clamped == []
    arrived, _, _ = run(al, bus, sdk, follower, t0=0.15)
    assert arrived
    assert {v for s, a, v in sdk.writes if s == 3 and a == lb.ADDR_GOAL} == {leader_ticks[3]}
    assert_limp(sdk, al)


def test_node_joint_at_the_follower_has_no_clamp_in_the_state_text(node_env):
    node, bus, sdk = node_env
    leader = [ACT + 0.1, -0.035, -0.006, 0.057, -0.069]
    follower = [0.0, 0.0, -0.006, 0.0, 0.0]
    leader_ticks = ticks_at(leader)
    _set_present(sdk, leader_ticks)
    _follower(node, follower)
    node._on_align(_bool(True))
    assert node._aligning() and 'clamped' not in node.align_state
    _drive_with(node, follower)
    assert node.align_state.startswith('done'), node.align_state
    assert not any('clamped' in m for m in node._align_state_pub.msgs)
    assert {v for s, a, v in sdk.writes if s == 3 and a == lb.ADDR_GOAL} == {leader_ticks[3]}
    assert node._enable_pub.msgs == [True]


def test_shutdown_always_ends_with_a_torque_off_of_every_servo(node_env):
    """Re-audit DL1b L-a: aligner._release sets IDLE before its torque-off, so an
    interrupt in that window left shutdown() with nothing to release. shutdown()
    now torques every leader servo off regardless of aligner/wall state."""
    node, bus, _ = node_env
    assert not node._aligning()
    calls = []
    real = bus.torque_off_verified
    bus.torque_off_verified = lambda ids=None: calls.append(ids) or real(ids)
    node.shutdown()
    assert calls == [None]                         # None = all configured ids


# ------------------------------------------------------------------ gripper (id 6), 2026-11-13
# FR-17 amendment (user decision 2026-11-13): the leader gripper is aligned, banded and
# walled like limb1..limb5. These fixtures configure all six ids from the shipped files.

IDS6 = (1, 2, 3, 4, 5, 6)
NAMES6 = NAMES + ['gripper']
CAL6 = load_calibration(SHIPPED_CAL_FILE).select(list(IDS6))
LIMITS6 = [(ARM_JOINT_LOWER[i - 1], ARM_JOINT_UPPER[i - 1]) for i in IDS6]
GRIPPER_Q = 0.6                       # inside the follower limits and the leader travel
Q6 = Q + [GRIPPER_Q]


def ticks6_at(q):
    return {sid: round(z + s * v * K) % 4096 for sid, z, s, v in zip(IDS6, CAL6.zero_ticks,
                                                                       CAL6.sign, q)}


def bus6_at(present):
    sdk = make_sdk(dict(present))
    return LeaderBus('/dev/null', ids=IDS6, sdk=sdk), sdk


def test_gripper_aligns_like_the_limbs():
    """J with the leader at its zero and the follower at Q6: the leader drives every
    joint, the gripper included, to the follower's pose and goes limp."""
    bus, sdk = bus6_at(dict(zip(IDS6, CAL6.zero_ticks)))
    al = Aligner(bus, CAL6, AlignConfig(activation_tolerance_rad=ACT), NAMES6, LIMITS6)
    assert abs(GRIPPER_Q) > ACT                         # the gripper itself must move
    assert al.begin(0.0, bus.read_positions(), Q6, bus.read_diagnostics())
    t = 0.0
    for _ in range(3000):
        t += 1 / 30
        if al.step(t, bus.read_positions(), Q6, t):
            break
    else:
        pytest.fail('alignment did not arrive')
    q = CAL6.to_radians(bus.read_positions())
    assert q == pytest.approx(Q6, abs=al._cfg.arrival_tolerance_rad + 1.0 / K)
    assert any(w[0] == 6 and w[1] == lb.ADDR_GOAL for w in sdk.writes)
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS6)


def test_gripper_beyond_its_leader_travel_is_refused_like_a_limb():
    """A follower gripper angle outside the recorded leader travel (minus the margin and
    beyond the activation tolerance) is refused, as for limb1..limb5."""
    bus, _ = bus6_at(dict(zip(IDS6, CAL6.zero_ticks)))
    al = Aligner(bus, CAL6, AlignConfig(activation_tolerance_rad=ACT), NAMES6, LIMITS6)
    far = min(CAL6.travel_max_rad[5] + 3 * ACT, LIMITS6[5][1])
    assert far > CAL6.travel_max_rad[5] + ACT
    with pytest.raises(AlignRefused, match='gripper'):
        al.plan(list(CAL6.zero_ticks), Q[:5] + [far], bus.read_diagnostics())
