"""Hardware-free tests: a mock scservo_sdk stands in for the serial layer."""

import math
import os

import pytest

from gripperx_leader.calibration import Calibration, load_calibration
from gripperx_leader.leader_bus import ADDR_PRESENT, ADDR_TORQUE, LeaderBus, LeaderReadError
from gripperx_leader.limits_recorder import LimitsRecorder
from gripperx_leader.record_limits import refuse_package_config


class MockSdk:
    COMM_SUCCESS = 0
    positions = {1: 2048, 2: 1000, 3: 3000, 4: 10, 5: 4095, 6: 1500}
    fail = False
    orig = dict(positions)
    writes = []

    class PortHandler:
        def __init__(self, name): self.name = name
        def openPort(self): return True
        def setBaudRate(self, b): return True
        def closePort(self): pass

    class PacketHandler:
        def __init__(self, proto): pass
        def ping(self, port, sid):
            return 0, (0 if sid in MockSdk.positions else -1), 0

        def write1ByteTxRx(self, port, sid, addr, val):
            MockSdk.writes.append((sid, addr, val))
            return 0, 0

    class GroupSyncRead:
        def __init__(self, port, ph, addr, length):
            assert (addr, length) == (ADDR_PRESENT, 2)
            self.ids = []
        def addParam(self, sid): self.ids.append(sid); return True
        def txRxPacket(self): return -1 if MockSdk.fail else 0
        def isAvailable(self, sid, addr, length): return sid in MockSdk.positions
        def getData(self, sid, addr, length): return MockSdk.positions[sid]


@pytest.fixture(autouse=True)
def _reset():
    MockSdk.fail = False
    MockSdk.writes = []
    MockSdk.positions.clear()
    MockSdk.positions.update(MockSdk.orig)


def test_read_order_and_torque_off_only_write():
    bus = LeaderBus('/dev/null', sdk=MockSdk)
    bus.open()
    assert bus.read_positions() == [2048, 1000, 3000, 10, 4095, 1500]
    # the only writes ever made: torque register = 0 on every servo
    assert MockSdk.writes == [(i, ADDR_TORQUE, 0) for i in range(1, 7)]


def test_read_error_raises():
    bus = LeaderBus('/dev/null', sdk=MockSdk)
    MockSdk.fail = True
    with pytest.raises(LeaderReadError):
        bus.read_positions()


def test_calibration(tmp_path):
    p = tmp_path / 'c.yaml'
    p.write_text('zero_ticks: [2048,0,0,0,0,0]\nsign: [1,-1,1,1,1,1]\n')
    cal = load_calibration(str(p))
    rad = cal.to_radians([3072, 1024, 0, 0, 0, 0])
    assert rad[0] == pytest.approx(math.pi / 2)
    assert rad[1] == pytest.approx(-math.pi / 2)
    p.write_text('zero_ticks: [0]\nsign: [1]\n')
    with pytest.raises(ValueError):
        load_calibration(str(p))


def test_recorder_yaml_order_and_poses():
    r = LimitsRecorder()
    assert not r.snapshot_pose('home')
    r.add_leader([10, 20, 30, 40, 50, 60])
    r.add_leader([5, 25, 30, 41, 49, 60])
    assert r.snapshot_pose('home')
    r.add_follower([1024] * 6)
    import yaml
    d = yaml.safe_load(r.to_yaml())
    assert d['joint_limits'] == {'min': [5, 20, 30, 40, 49, 60], 'max': [10, 25, 30, 41, 50, 60]}
    assert d['poses']['home'] == [5, 25, 30, 41, 49, 60]
    assert d['follower_joint_limits']['min'] == [1024] * 6


def test_refuses_package_config():
    with pytest.raises(SystemExit):
        refuse_package_config('/x/gripperx_arm/config/arm_poses.yaml')
    refuse_package_config('/tmp/out.yaml')


def test_example_calibration_loads():
    here = os.path.dirname(os.path.dirname(__file__))
    assert isinstance(load_calibration(os.path.join(here, 'config', 'leader_calibration.example.yaml')), Calibration)


def _isolate():
    """ISOLATION (audit round 3, 2026-10-08): DDS discovery OFF - the test's nodes match
    inside this process and discover no other one, so parallel runs cannot cross-talk
    whatever domain they share. ROS_LOCALHOST_ONLY=1 would make rcl ignore the range,
    ROS_STATIC_PEERS would add peers back: both removed. Domain: the caller's."""
    os.environ.setdefault('ROS_DOMAIN_ID', '221')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ.pop('ROS_STATIC_PEERS', None)


def test_node_publishes_and_withdraws_enable_on_errors():
    """Node smoke, DDS discovery OFF (process-local); mock bus, nothing real."""
    _isolate()
    import rclpy
    from std_msgs.msg import Bool
    from sensor_msgs.msg import JointState
    from gripperx_leader.leader_arm_node import ENABLE_QOS, LeaderArmNode

    class FakeBus:
        def __init__(self): self.fail = False
        def read_positions(self):
            if self.fail:
                raise LeaderReadError('x')
            return [1, 2, 3, 4, 5, 6]
        def close(self): pass
        def ping_all(self, ids): return {i: True for i in ids}

    rclpy.init()
    try:
        bus = FakeBus()
        node = LeaderArmNode(bus=bus)
        got = {'ticks': [], 'enable': [], 'active': []}
        node.create_subscription(JointState, '/arm/leader/ticks', lambda m: got['ticks'].append(list(m.position)), 10)
        node.create_subscription(Bool, '/arm/leader/enable', lambda m: got['enable'].append(m.data), ENABLE_QOS)
        node.create_subscription(Bool, '/arm/leader/active', lambda m: got['active'].append(m.data), 10)
        import time
        t = time.time()
        while time.time() - t < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert got['ticks'] and got['ticks'][0] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert got['enable'][0] is False
        bus.fail = True
        t = time.time()
        while time.time() - t < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert False in got['active']
        node.shutdown()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_five_id_read_and_ping():
    MockSdk.positions.pop(6)
    bus = LeaderBus('/dev/null', ids=(1, 2, 3, 4, 5), sdk=MockSdk)
    bus.open()
    assert bus.read_positions() == [2048, 1000, 3000, 10, 4095]
    assert bus.ping_all() == {1: True, 2: True, 3: True, 4: True, 5: True, 6: False}
    assert MockSdk.writes == [(i, ADDR_TORQUE, 0) for i in range(1, 6)]   # never touches id 6
    MockSdk.positions.pop(3)
    with pytest.raises(LeaderReadError):
        bus.read_positions()


def test_joint_names_follow_ids_and_calibration_subset(tmp_path):
    from gripperx_leader.leader_bus import joint_names
    assert joint_names([1, 2, 3, 4, 5]) == ['limb1', 'limb2', 'limb3', 'limb4', 'limb5']
    assert joint_names([1, 2, 4, 6]) == ['limb1', 'limb2', 'limb4', 'gripper']
    p = tmp_path / 'c.yaml'
    p.write_text('zero_ticks: [1,2,3,4,5,6]\nsign: [1,1,1,1,1,-1]\n')
    assert list(load_calibration(str(p)).select([1, 2, 3, 4, 5]).zero_ticks) == [1, 2, 3, 4, 5]


def test_recorder_absent_gripper_is_null_not_zero():
    import yaml
    r = LimitsRecorder()
    r.add_leader([10, 20, 30, 40, 50], ['limb1', 'limb2', 'limb3', 'limb4', 'limb5'])
    assert r.snapshot_pose('home')
    text = r.to_yaml()
    d = yaml.safe_load(text)
    assert d['joint_limits']['min'] == [10, 20, 30, 40, 50, None]
    assert d['poses']['home'][5] is None
    assert 'TO-VERIFY: no samples for gripper' in text


def test_node_refuses_absent_configured_id():
    _isolate()
    import rclpy
    from gripperx_leader.leader_arm_node import LeaderArmNode

    class Bus5:
        closed = False
        def ping_all(self, ids): return {i: i != 6 for i in ids}
        def close(self): self.closed = True

    rclpy.init()
    try:
        bus = Bus5()
        with pytest.raises(SystemExit) as e:
            LeaderArmNode(bus=bus)    # default servo_ids = 1..6, id 6 absent
        assert 'servo_ids' in str(e.value) and bus.closed
    finally:
        rclpy.shutdown()


def test_wrap_crossing_both_directions_is_continuous():
    step = 2 * math.pi / 4096
    cal = Calibration([2036, 3718, 337, 0, 0, 0], [1] * 6)
    # limb3 (zero 337) walks DOWN through the wrap: 3, 2, 1, 0, 4095, 4094
    down3 = [cal.to_radians([0, 0, t, 0, 0, 0])[2] for t in (3, 2, 1, 0, 4095, 4094)]
    assert all(b - a == pytest.approx(-step) for a, b in zip(down3, down3[1:]))
    # limb5 zero 3718: cross the wrap upward 4094 -> 0 -> 3 and back
    vals = [cal.to_radians([0, t, 0, 0, 0, 0])[1] for t in (4093, 4094, 4095, 0, 1, 2)]
    diffs = [b - a for a, b in zip(vals, vals[1:])]
    assert all(d == pytest.approx(step) for d in diffs)
    down = [cal.to_radians([0, t, 0, 0, 0, 0])[1] for t in (2, 1, 0, 4095, 4094)]
    assert all(b - a == pytest.approx(-step) for a, b in zip(down, down[1:]))
    # value itself: ticks 3718 + 500 -> wraps to 122 -> +500 ticks
    assert cal.to_radians([0, 122, 0, 0, 0, 0])[1] == pytest.approx(500 * step)


def test_unverified_flag_is_read_from_the_file(tmp_path):
    # Explicit fixture: the real file with only the flag forced to false, so this
    # does not depend on the shipped value.
    here = os.path.dirname(os.path.dirname(__file__))
    text = open(os.path.join(here, 'config', 'leader_calibration.yaml')).read()
    assert 'calibration_verified: true' in text
    f = tmp_path / 'c.yaml'
    f.write_text(text.replace('calibration_verified: true', 'calibration_verified: false'))
    assert load_calibration(str(f)).verified is False


def test_real_calibration_file_covers_all_six_ids():
    here = os.path.dirname(os.path.dirname(__file__))
    cal = load_calibration(os.path.join(here, 'config', 'leader_calibration.yaml'))
    # Structure only: the calibration VALUES are machine data, and asserting them here
    # would only duplicate the file (a test that can fail only when the file is edited).
    # Id 6 (gripper) is calibrated since 2026-11-13 (FR-17: driven in leader mode).
    assert len(cal.zero_ticks) == len(cal.sign) == len(cal.range_center_rad) == 6
    assert all(z is not None for z in cal.zero_ticks)
    assert all(s in (1, -1) for s in cal.sign)
    assert all(abs(c) <= math.pi for c in cal.range_center_rad)
    assert cal.verified is True      # the ACTIVE shipped file: verified by user 2026-10-09
    assert cal.broken_pairs() == {}  # every zero/offset pair matches its invariant
    six = cal.select([1, 2, 3, 4, 5, 6])
    assert six.to_radians(cal.zero_ticks) == pytest.approx([0.0] * 6, abs=1e-9)
    five = cal.select([1, 2, 3, 4, 5])  # a leader without the gripper cable
    assert five.to_radians(cal.zero_ticks[:5]) == pytest.approx([0.0] * 5, abs=1e-9)


def test_recorder_min_max_across_wrap():
    r = LimitsRecorder(leader_zero=[337] + [None] * 5)
    for t in (330, 100, 10, 4090, 4050, 100):     # sweeps down through the wrap
        r.add_leader([t], ['limb1'])
    assert r.leader_min[0] == 4050 and r.leader_max[0] == 330
    assert 'CROSSES' in r.to_yaml()
    r2 = LimitsRecorder()                          # no zero: first sample is the reference
    for t in (4000, 4095, 0, 50):
        r2.add_leader([t], ['limb1'])
    assert (r2.leader_min[0], r2.leader_max[0]) == (4000, 50)


def test_range_center_window_no_flip_beyond_180():
    k = 2 * math.pi / 4096
    # limb travel about [-192, +58] deg around zero: centre -67 deg
    c = math.radians(-67.0)
    cal = Calibration([2048] * 6, [1] * 6, range_center_rad=[c] * 6)
    ticks = [2048 + int(round(math.radians(d) / k)) for d in (50, 0, -100, -170, -185, -192)]
    rad = [cal.to_radians([t, 0, 0, 0, 0, 0])[0] for t in ticks]
    # continuous, nothing flips to +170..: monotone decreasing, all inside [c-pi, c+pi)
    assert all(b < a for a, b in zip(rad, rad[1:]))
    assert all(c - math.pi <= r < c + math.pi for r in rad)
    assert math.degrees(rad[-1]) == pytest.approx(-192, abs=0.2)
    # default centre 0 flips the same ticks (the bug this fixes)
    flat = Calibration([2048] * 6, [1] * 6)
    assert flat.to_radians([ticks[-1], 0, 0, 0, 0, 0])[0] > 0
    # sign -1 and select() carry the centre
    sub = Calibration([2048] * 6, [-1] * 6, range_center_rad=[0, c, 0, 0, 0, 0]).select([1, 2])
    assert sub.range_center_rad == [0, c]


def test_recorder_continuity_beyond_180_and_gap_flags():
    r = LimitsRecorder()
    # start away from home at +100 deg; node window flips at +-180: sequence goes up through 180
    seq = [100, 140, 175, -170, -150]        # = 100..210 deg continuous
    for d in seq:
        r.add_leader_rad([math.radians(d)], ['limb2'])
    text = r.to_yaml()
    import yaml
    d = yaml.safe_load(text)['leader_range']
    assert d['min_rad'][1] == pytest.approx(math.radians(100), abs=1e-3)
    assert d['max_rad'][1] == pytest.approx(math.radians(210), abs=1e-3)
    assert d['suggested_range_center_rad'][1] == pytest.approx(math.radians(155), abs=1e-3)
    assert d['unreachable_gap_rad'][1] == pytest.approx(math.radians(250), abs=1e-3)
    assert 'FLAG' not in text
    # a big step and a small gap are flagged
    r2 = LimitsRecorder()
    for deg in (0, 60, 120, 180, -120, -60, -10):    # continuous 0..350, one 60 deg step ok
        r2.add_leader_rad([math.radians(deg)], ['limb1'])
    assert 'gap < 30' in r2.to_yaml()
    r3 = LimitsRecorder()
    for deg in (0, 120):
        r3.add_leader_rad([math.radians(deg)], ['limb1'])
    assert 'step(s) > 90' in r3.to_yaml()


def test_recorder_ticks_from_continuous_radians():
    import yaml
    cal = Calibration([1000] * 6, [-1] * 6)
    r = LimitsRecorder(calibration=cal, leader_zero=[1000] * 6)
    for deg in (0, -45, -90):
        r.add_leader_rad([math.radians(deg)], ['limb1'])
        r.add_leader([1000 + round(-deg / 360 * 4096)], ['limb1'])
    d = yaml.safe_load(r.to_yaml())
    assert d['joint_limits']['min'][0] == 1000 and d['joint_limits']['max'][0] == 1000 + 1024
