"""arm_action_server against a MOCK servo bus (no hardware, no robot), with DDS
discovery OFF (process-local; see ISOLATION below). Every bus event (goal write, torque switch) lands in
MockBus.events in order; assertions use node state, not asynchronous topics.
"""
import os

# ISOLATION (audit round 3, 2026-10-08). Discovery OFF: the nodes of this test match
# each other inside this one process and discover NO other process - so runs started
# at the same time (other worktrees, other agents) cannot cross-talk, whatever domain
# they share. With LOCALHOST they did: duplicate publishers, duplicate goal responses.
# The deprecated ROS_LOCALHOST_ONLY=1 makes rcl IGNORE the range (it would undo OFF),
# and ROS_STATIC_PEERS would add peers back: both removed. The domain is the caller's
# (setdefault); CMakeLists.txt sets 231 for colcon test.
os.environ.setdefault('ROS_DOMAIN_ID', '231')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ.pop('ROS_STATIC_PEERS', None)

import math
import sys
import threading
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Int64, String

from gripperx_arm import arm_action_server as aas
from gripperx_arm_msgs.action import PickPlastic
from gripperx_external_msgs.msg import ArmingState

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
HOME = [int(v) for v in yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())[
    'arm_joint_state_relay']['ros__parameters']['zero_ticks']]       # home IS the zero
HOME_D = dict(zip(aas.IDS, HOME))


class MockBus:
    """An ideal servo: a goal write with torque on sets the present position."""

    def __init__(self, ticks):
        self.pos = dict(ticks)
        self.torque_on = {sid: False for sid in aas.IDS}
        self.events = []
        self.lock = threading.Lock()
        # The torque-limit registers (Torque_Limit RAM, Max_Torque_Limit EEPROM) at the
        # servo's full range - the mock's power-on state; every other register reads None.
        self.regs = {(sid, addr): aas.TORQUE_LIMIT_MAX for sid in aas.IDS
                     for addr in (aas.ADDR_TORQUE_LIMIT, aas.ADDR_MAX_TORQUE_LIMIT)}

    @property
    def writes(self):
        return [(e[1], e[2]) for e in list(self.events) if e[0] == 'goal']

    def torque(self, sid, on):
        with self.lock:
            self.events.append(('torque', sid, bool(on)))
        self.torque_on[sid] = bool(on)
        return True

    def read_pos(self, sid):
        return self.pos.get(sid)

    def read_word(self, sid, addr):
        return self.regs.get((sid, addr))

    def write_word(self, sid, addr, value):
        with self.lock:
            self.events.append(('word', sid, addr, int(value)))
        self.regs[(sid, addr)] = int(value)
        return True

    def read_byte(self, sid, addr):
        return None

    def write_goal(self, sid, ticks):
        with self.lock:
            self.events.append(('goal', sid, int(ticks)))
        if self.torque_on[sid]:
            self.pos[sid] = int(ticks)
        return True

    def close(self):
        pass


def _desc():
    d = SRC / 'gripperx_description'
    if not (d / 'urdf').is_dir():
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory('gripperx_description'))
    return d


def _urdf():
    import xacro
    return xacro.process_file(str(_desc() / 'urdf' / 'gripperx_v1.urdf.xacro')).toxml()


def _mesh(uri):
    return str(_desc() / 'meshes' / uri.split('/')[-1])


def _steer_yaml():
    p = SRC / 'gripperx_control' / 'config' / 'steer_servo.yaml'
    if p.is_file():
        return str(p)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('gripperx_control')) / 'config' / 'steer_servo.yaml')


def _flatten(prefix, d, out):
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            _flatten(key, v, out)
        else:
            out[key] = v
    return out


def active_params(drop=(), **over):
    p = {}
    for f in ('arm_poses.yaml', 'arm_safety_filter.yaml'):
        _flatten('', yaml.safe_load((PKG / 'config' / f).read_text())
                 ['arm_action_server']['ros__parameters'], p)
    # The TF age bound shared with the goal gateway (SR-16 E3, audit F4 (a)): its file.
    shared = PKG.parent / 'gripperx_arm_msgs' / 'config' / 'tf_freshness.yaml'
    if shared.is_file():
        p.update(yaml.safe_load(shared.read_text())['/**']['ros__parameters'])
    p['safety_filter.calibration_file'] = str(PKG / 'config' / 'arm_calibration.yaml')
    p['safety_filter.acm_file'] = str(PKG / 'config' / 'arm_allowed_collisions.yaml')
    p['safety_filter.stop_protected_file'] = str(PKG / 'config' / 'arm_stop_protected.yaml')
    # Test-only: a laptop under load can delay a 30 Hz stream by more than the
    # configured 0.3 s; tests that are not about the watchdog must not trip it.
    p['leader.watchdog_timeout_sec'] = 1.0
    # planner.time_budget_s stays as configured: a test whose outcome must be decided by
    # planner.max_checks (deterministic) and not by the wall-clock cap raises it itself
    # (DETERMINISTIC below), with the reason.
    p['safety_filter.steer_config_file'] = _steer_yaml()
    p.update(over)
    return [Parameter(k, value=v) for k, v in p.items() if k not in drop]


def centred_calibration(path: Path) -> str:
    """The real calibration file with every LIMB zero moved to mid-range (2048) and
    every limb sign +1: a TEST calibration under which any limb angle in (-pi, pi) is
    a representable tick. The mock bus starts at the REAL home ticks, so the test
    calibration must read them as a pose inside the real limits: with the real signs
    (2026-10-08: limb2..limb4 = -1) they would read beyond limb2's upper, limb3's
    lower and limb4's upper limit. The gripper keeps its real zero and sign: its real
    limits (2026-10-08, -18.6..+104.6 deg) are representable as they are, while 2048
    would read its home as -45.8 deg, outside them."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    p = doc['arm_joint_state_relay']['ros__parameters']
    p['zero_ticks'] = [2048] * 5 + [p['zero_ticks'][5]]
    p['signs'] = [1] * 5 + [p['signs'][5]]
    path.write_text(yaml.safe_dump(doc))
    return str(path)


def calibration_flag(path: Path, verified: bool) -> str:
    """The real calibration file with only calibration_verified forced: a refusal test
    must not depend on what the shipped file says."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    doc['arm_joint_state_relay']['ros__parameters']['calibration_verified'] = verified
    path.write_text(yaml.safe_dump(doc))
    return str(path)


URDF = None


class Rig:
    def __init__(self, start_q=None, description=True, pre=None, armed=False, drop=(), **over):
        self._drop = drop
        global URDF
        URDF = URDF or _urdf()
        rclpy.init()
        try:
            self._setup(start_q, description, pre, armed, over)
        except Exception:
            rclpy.shutdown()
            raise

    def _setup(self, start_q, description, pre, armed, over):
        self.tester = Node('arm_node_tester')
        if pre:
            pre(self)
        self.arming = self.tester.create_publisher(ArmingState, '/gripperx/external/arming_state',
                                                   LATCHED)
        self._armed_sent = armed is not None
        if armed is not None:
            self.arming.publish(ArmingState(armed=armed))
        self.bus = MockBus(HOME_D)
        self.node = aas.ArmActionServer(bus=self.bus,
                                        robot_description=URDF if description else None,
                                        mesh_resolver=_mesh,
                                        parameter_overrides=active_params(self._drop, **over))
        if start_q is not None:
            start = self.node._ticks_of(start_q(self.node))
            self.bus.pos.update(start)
            # the arm HOLDS the start pose: its servos' goals are there (a hold keeps
            # the last goal, user decision 2026-10-13 - not the sync at startup)
            with self.node._serial_lock:
                self.node._goal.update(start)
        self.ex = MultiThreadedExecutor(num_threads=4)
        self.ex.add_node(self.node)
        self.ex.add_node(self.tester)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        self.enable = self.tester.create_publisher(Bool, '/arm/leader/enable', LATCHED)
        self.active = self.tester.create_publisher(Bool, '/arm/leader/active', 10)
        self.joints = self.tester.create_publisher(JointState, '/arm/leader/joint_states', 10)
        self._calv_pub = self.tester.create_publisher(Bool, '/arm/leader/calibration_verified',
                                                      LATCHED)
        self._calv_sent = None
        rig = self

        class _Calv:
            def publish(self, msg):
                rig._calv_sent = bool(msg.data)
                rig._calv_pub.publish(msg)
        self.calv = _Calv()
        self.pick_client = ActionClient(self.tester, PickPlastic, 'pick_plastic')
        self.epoch = None                # the last /arm/stop_epoch, as a P sender sees it
        self.tester.create_subscription(Int64, '/arm/stop_epoch',
                                        lambda m: setattr(self, 'epoch', int(m.data)), LATCHED)
        self.cmd = self.tester.create_publisher(String, '/arm/command', 10)
        self.stop = self.tester.create_publisher(Empty, '/arm/stop', 10)
        self.leader_q = None
        self.leader_omit = ()
        self.heartbeat = True
        self._stream = True
        self.streamer = threading.Thread(target=self._run_stream, daemon=True)
        self.streamer.start()
        time.sleep(0.5)

    def _run_stream(self):
        while self._stream:
            q = self.leader_q
            if q is not None:
                if self.heartbeat:
                    self.active.publish(Bool(data=True))
                msg = JointState()
                # All six joints: the follower drives its gripper from the leader since
                # 2026-11-13 (FR-17). A leader_q written for limb1..limb5 only (tests from
                # before) holds the gripper where the follower's is, so it does not move;
                # leader_omit drops joints from the stream (incomplete-stream refusal).
                g = self.node._cal.joints[aas.IDS.index(aas.GRIPPER_ID)]
                if g not in q:
                    q = {**q, g: self.measured_q()[g]}
                pairs = [(ln, n) for ln, n in zip(aas.JOINT_NAMES, self.node._cal.joints)
                         if n not in self.leader_omit]
                msg.name = [ln for ln, _ in pairs]
                msg.position = [float(q[n]) for _, n in pairs]
                self.joints.publish(msg)
            time.sleep(1.0 / 30.0)

    def measured_q(self):
        return self.node._q_of(dict(self.bus.pos))

    def rising_edge(self):
        # Everything the node's activation gate reads must have ARRIVED first
        # (discovery is asynchronous); otherwise a test meant to see one refusal
        # sees another one. The node reads enable VOLATILE: publish once matched.
        n = self.node
        assert self.wait(lambda: self.tester.count_subscribers('/arm/leader/enable') >= 1
                         and (not self._armed_sent or n._gateway_armed is not None)
                         and (self._calv_sent is None
                              or n._leader_cal_verified == self._calv_sent)
                         and (self.leader_q is None
                              or (n.count_publishers('/arm/leader/joint_states') >= 1
                                  and n.count_publishers('/arm/leader/active') >= 1
                                  and time.monotonic() - n._leader_rx < 0.5)), 20.0), \
            'activation inputs did not arrive'
        self.enable.publish(Bool(data=False))
        time.sleep(0.2)
        self.enable.publish(Bool(data=True))

    def wait(self, cond, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cond():
                return True
            time.sleep(0.02)
        return False

    def kinds(self):
        return [k for k, _ in list(self.node.audit)]

    def close(self):
        self._stream = False
        self.streamer.join(1.0)
        self.ex.shutdown()
        self.node.destroy_node()
        self.tester.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def rig_factory():
    rigs = []

    def make(**kw):
        r = Rig(**kw)
        rigs.append(r)
        return r
    yield make
    for r in rigs:
        r.close()


def _ik(node, x, y, z, branch):
    sol = [s for s in node._filter.kin.ik_tcp_down(x, y, z).solutions if s.branch == branch]
    q = dict(sol[0].q)
    q['arm_gripper_joint'] = 0.0
    return q


ALLOW = {'safety_filter.allow_unverified_calibration': True}
# Test-only, for tests that assert WHICH bound ends a search ('checks used up'): under
# load the wall-clock cap could bind first and make the outcome machine-dependent.
DETERMINISTIC = {'planner.time_budget_s': 120.0}


def _activate(r):
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    n = r.node
    # every input the activation gate reads must have arrived (discovery is async)
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and n.count_publishers('/arm/leader/joint_states') == 1
                  and n.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - n._leader_rx < 0.5, 20.0), 'gate inputs missing'
    r.rising_edge()
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_ACTIVE), r.node.leader_text


def _text(r, needle, timeout=2.0):
    return r.wait(lambda: needle in getattr(r.node, 'leader_text', ''), timeout)


def _nudge(r, joint='arm_limb1_joint', d=0.05):
    q = dict(r.leader_q)
    q[joint] += d
    r.leader_q = q
    assert r.wait(lambda: 'approve' in r.kinds(), 2.0)


# ------------------------------------------------------------------ torque-on sync
def test_startup_torque_on_writes_goal_equal_present_first(rig_factory):
    r = rig_factory()
    ev = r.bus.events
    first = next(i for i, e in enumerate(ev) if e[0] == 'torque')
    assert sorted((e[1], e[2]) for e in ev[:first]) == sorted(HOME_D.items())
    assert ev[first:first + 6] == [('torque', sid, True) for sid in aas.IDS]
    assert r.kinds()[:2] == ['torque_on_sync', 'torque_on']


def test_unreadable_servo_refuses_torque_on(rig_factory):
    r = rig_factory()
    r.node._torque_off_all('test')
    r.bus.pos.pop(3)
    n = len(r.bus.events)
    with pytest.raises(aas.MotionRefused):
        r.node._ensure_torque()
    assert r.bus.events[n:] == []


def test_torque_on_after_space_syncs_goals_first(rig_factory):
    """After Space the arm is limp and may sag; the next command must not jump."""
    r = rig_factory()
    r.cmd.publish(String(data='go_home'))      # moving (already home: zero-length ramp)
    assert r.wait(lambda: 'approve' in r.kinds(), 3.0)
    r.stop.publish(Empty())
    assert r.wait(lambda: 'torque_off' in r.kinds(), 2.0)
    time.sleep(r.node._grace + 4.5)
    sagged = {sid: t + 37 for sid, t in HOME_D.items()}
    r.bus.pos.update(sagged)                   # the limp arm dropped a bit
    n = len(r.bus.events)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: any(e[0] == 'torque' for e in r.bus.events[n:]), 3.0)
    ev = r.bus.events[n:]
    t = next(i for i, e in enumerate(ev) if e[0] == 'torque')
    assert sorted((e[1], e[2]) for e in ev[:t]) == sorted(sagged.items())


# ------------------------------------------------------------------ activation gating
def test_latched_enable_from_before_start_does_not_activate(rig_factory):
    def pre(rig):
        rig._pre_pub = rig.tester.create_publisher(Bool, '/arm/leader/enable', LATCHED)
        rig._pre_pub.publish(Bool(data=True))
    r = rig_factory(pre=pre, **ALLOW)
    n = len(r.bus.writes)
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    time.sleep(1.5)
    assert r.node._leader_state == aas.LEADER_OFF
    assert len(r.bus.writes) == n


def test_unverified_calibration_refuses_leader_mode(rig_factory, tmp_path):
    r = rig_factory(**{'safety_filter.calibration_file': calibration_flag(tmp_path / 'c.yaml',
                                                                          False)})
    n = len(r.bus.writes)
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    r.rising_edge()
    assert _text(r, 'calibration unverified'), r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF
    assert len(r.bus.writes) == n


def test_shipped_calibration_is_verified_and_needs_no_allow_flag(rig_factory):
    """The ACTIVE shipped file (no override, no allow_unverified_calibration): leader
    mode activates. If the shipped flag is ever set back to false this fails."""
    r = rig_factory()
    assert r.node._cal_verified is True and not r.node._allow_unverified
    _activate(r)


@pytest.mark.parametrize('kw,needle', [
    ({'armed': True, **ALLOW}, 'ARMED'),
    ({'armed': None, **ALLOW}, 'UNKNOWN'),
])
def test_activation_refusals(rig_factory, kw, needle):
    r = rig_factory(**kw)
    n = len(r.bus.writes)
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    r.rising_edge()
    assert _text(r, needle), r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF
    assert len(r.bus.writes) == n


def test_unverified_leader_calibration_refuses(rig_factory):
    r = rig_factory(**ALLOW)
    r.calv.publish(Bool(data=False))
    r.leader_q = r.measured_q()
    r.node._allow_unverified = False
    r.node._cal_verified = True
    r.rising_edge()
    assert _text(r, 'leader calibration'), r.node.leader_text


def test_mismatch_refused_with_per_joint_difference(rig_factory):
    r = rig_factory(**ALLOW)
    r.calv.publish(Bool(data=True))
    q = r.measured_q()
    q['arm_limb5_joint'] += 0.5
    r.leader_q = q
    r.rising_edge()
    assert _text(r, 'does not match'), r.node.leader_text
    assert 'arm_limb5_joint' in r.node.leader_text


# User decision 2026-10-12 (option a): the activation match is 10 deg, its own
# parameter; the replan-repeat gate keeps 0.10 rad under another name.
def _gate_inputs(r):
    n = r.node
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and n.count_publishers('/arm/leader/joint_states') == 1
                  and n.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - n._leader_rx < 0.5, 20.0), 'gate inputs missing'


@pytest.mark.parametrize('deg,active', [(9.0, True), (11.0, False)])
def test_activation_match_is_ten_degrees(rig_factory, deg, active):
    """A leader 9 deg off the follower on one joint activates, 11 deg is refused; once
    active, the 9 deg are closed at leader.max_joint_velocity_rad_s - never jumped."""
    r = rig_factory(**ALLOW)
    assert r.node._p('leader.activation_match_tolerance_rad') == pytest.approx(
        math.radians(10.0), abs=1e-6)
    r.calv.publish(Bool(data=True))
    q = r.measured_q()
    q['arm_limb1_joint'] += math.radians(deg)
    r.leader_q = q
    _gate_inputs(r)
    n = len(r.node.audit)
    r.rising_edge()
    if not active:
        assert _text(r, 'does not match', 5.0), r.node.leader_text
        assert 'arm_limb1_joint' in r.node.leader_text
        assert r.node._leader_state == aas.LEADER_OFF
        assert 'approve' not in r.kinds()[n:]
        return
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_ACTIVE, 5.0), r.node.leader_text
    goal = r.node._ticks_of(q)[1]
    assert r.wait(lambda: abs(r.bus.pos[1] - goal) <= 1, 10.0), (r.bus.pos[1], goal)
    vmax = r.node._p('leader.max_joint_velocity_rad_s') / r.node._p('leader.rate_hz')
    step_ticks = vmax * 4096 / (2 * math.pi)
    seq = [HOME_D[1]] + [sp[1] for k, sp in list(r.node.audit)[n:] if k == 'approve']
    assert len(seq) > math.radians(deg) / vmax - 2           # many steps, not one jump
    assert max(abs(b - a) for a, b in zip(seq, seq[1:])) <= math.ceil(step_ticks) + 1


def test_replan_repeat_gate_keeps_its_own_tolerance(rig_factory):
    """_same_inputs reads leader.replan_repeat_tolerance_rad (0.10 rad active), not the
    10 deg activation match."""
    r = rig_factory(**ALLOW)
    node = r.node
    tol = node._p('leader.replan_repeat_tolerance_rad')
    assert tol == pytest.approx(0.10)
    assert tol < node._p('leader.activation_match_tolerance_rad')
    q = r.measured_q()

    class Job:
        start, target, env = dict(q), dict(q), None
    for d, same in ((0.9 * tol, True), (1.1 * tol, False)):
        t = dict(q, arm_limb1_joint=q['arm_limb1_joint'] + d)
        assert node._same_inputs(Job, q, t) is same, d
        m = dict(q, arm_limb2_joint=q['arm_limb2_joint'] + d)
        assert node._same_inputs(Job, m, q) is same, d


def test_second_leader_publisher_refused(rig_factory):
    r = rig_factory(**ALLOW)
    r.calv.publish(Bool(data=True))
    rival = r.tester.create_publisher(JointState, '/arm/leader/joint_states', 10)
    r.leader_q = r.measured_q()
    time.sleep(0.5)
    r.rising_edge()
    assert _text(r, '2 publishers'), r.node.leader_text
    del rival


def test_gateway_arming_during_leader_mode_ends_it(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    r.arming.publish(ArmingState(armed=True))
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_OFF, 3.0)
    assert 'ARMED' in r.node.leader_text, r.node.leader_text


# ------------------------------------------------------------------ leader behaviour
def test_velocity_limit_per_tick(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    n = len(r.node.audit)
    q = dict(r.leader_q)
    q['arm_limb1_joint'] += 0.5                     # the leader jumps
    r.leader_q = q
    time.sleep(1.0)
    vmax = r.node._p('leader.max_joint_velocity_rad_s') / r.node._p('leader.rate_hz')
    step_ticks = vmax * 4096 / (2 * math.pi)
    seq = [HOME_D[1]] + [sp[1] for k, sp in list(r.node.audit)[n:] if k == 'approve']
    assert len(seq) > 5
    assert max(abs(b - a) for a, b in zip(seq, seq[1:])) <= math.ceil(step_ticks) + 1


def test_measured_pose_watch_ends_leader_mode(rig_factory, tmp_path):
    """Rule 8: the MEASURED pose (pushed into the chassis), not the commanded one."""
    r = rig_factory(**ALLOW, **{'safety_filter.calibration_file':
                                centred_calibration(tmp_path / 'c.yaml')})
    _activate(r)
    bad = _ik(r.node, 0.30, -0.12, -0.12, 'front/elbow-')     # limb vs chassis
    assert not r.node._filter.check_pose(bad).ok
    r.bus.pos.update(r.node._ticks_of(bad))                    # external push
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_OFF, 2.0)
    assert 'MEASURED' in r.node.leader_text


def test_stale_heartbeat_leaves_and_returns_home(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    _nudge(r)
    r.heartbeat = False                    # joint_states keep coming, heartbeat stops
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_OFF, 3.0)
    assert 'heartbeat' in r.node.leader_text
    assert r.wait(lambda: r.bus.pos == HOME_D, 15.0), 'did not return home'
    assert r.wait(lambda: 'gripper_unchecked' in r.kinds(), 5.0)
    k = r.kinds()
    i = k.index('hold')
    assert 'approve' in k[i + 1:]                     # the planned path home


def test_disable_via_j_returns_home(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    _nudge(r, d=0.1)
    time.sleep(0.5)
    r.enable.publish(Bool(data=False))
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), (dict(r.bus.pos), r.node.leader_text,
                                                       list(r.node.status_log)[-4:])


def _command_in_thread(r, cmd):
    th = threading.Thread(target=r.node._on_command, args=(String(data=cmd),), daemon=True)
    th.start()
    return th


def test_command_before_its_disable_wins_over_return_home(rig_factory):
    """O arrives 100 ms BEFORE the leader disable sent with it (separate topics, no
    arrival order - user decision 2026-10-09): the command waits for leader mode to
    switch off, claims the arm, and the leader exit's return home never runs."""
    r = rig_factory(**ALLOW)
    _activate(r)
    _nudge(r, d=0.1)
    target = r.node._ticks_of(r.leader_q)                # the follower reaches the leader
    assert r.wait(lambda: all(abs(r.bus.pos[s] - target[s]) <= 1 for s in aas.LIMB_IDS), 5.0)
    r.leader_q = None
    moved = {s: r.bus.pos[s] for s in aas.LIMB_IDS}
    n = len(r.node.audit)
    th = _command_in_thread(r, 'open_gripper')
    time.sleep(0.1)
    r.node._on_leader_enable(Bool(data=False))
    th.join(10.0)
    assert not th.is_alive()
    assert r.bus.pos[aas.GRIPPER_ID] == r.node._grip_open, list(r.node.status_log)[-4:]
    with r.node._state_lock:
        assert r.node._pending_home is None and r.node._motion_owner is None
    assert {s: r.bus.pos[s] for s in aas.LIMB_IDS} == moved
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]
    assert not any('REFUSED' in t for t in list(r.node.status_log))


def test_command_while_leader_stays_on_is_refused_after_the_wait(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    n = len(r.node.audit)
    t0 = time.monotonic()
    r.node._on_command(String(data='open_gripper'))
    assert time.monotonic() - t0 >= float(r.node._p('leader.command_wait_sec'))
    assert any('REFUSED' in t and 'still on after waiting' in t
               for t in list(r.node.status_log)), list(r.node.status_log)[-3:]
    assert r.node._leader_state == aas.LEADER_ACTIVE
    assert 'gripper_unchecked' not in [k for k, _ in list(r.node.audit)[n:]]


def test_space_during_the_wait_refuses_and_nothing_moves(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    r.leader_q = None
    n = len(r.node.audit)
    node, waiting = r.node, threading.Event()
    real_p = node._p

    def spy(name):                   # the command has taken its Space snapshot
        if name == 'leader.command_wait_sec':
            waiting.set()
        return real_p(name)
    node._p = spy
    th = _command_in_thread(r, 'go_home')
    assert waiting.wait(5.0)
    r.node._on_stop(Empty())
    th.join(10.0)
    assert any('Space while waiting for leader mode' in t for t in list(r.node.status_log)), \
        list(r.node.status_log)[-3:]
    k = [k for k, _ in list(r.node.audit)[n:]]
    assert 'approve' not in k and 'gripper_unchecked' not in k, k
    assert not any(r.bus.torque_on.values())


def _chassis_rig(rig_factory, tmp_path, jump=False, **over):
    """Follower at a, leader moved INTO the chassis on the way to b: the follower holds.
    The direct path a -> b crosses the chassis; b itself is valid. jump=True: the leader
    goes inside at once, so no search starts on the way in (the follower walks up to
    the body under the velocity limit and holds there, its target invalid)."""
    def start(node):
        return _ik(node, 0.10, 0.25, 0.0, 'front/elbow-')
    r = rig_factory(start_q=start, **ALLOW, **{
        'leader.max_joint_velocity_rad_s': 2.0,
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'cal.yaml')}, **over)
    flt = r.node._filter
    a = r.measured_q()
    b = _ik(r.node, 0.30, 0.0, 0.05, 'front/elbow-')
    assert flt.check_pose(a).ok and flt.check_pose(b).ok and not flt.check_path(a, b).ok
    path = flt.subdivide(a, b)
    inside = next(p for p in path if not flt.check_pose(p).ok)    # first pose in the body
    _activate(r)
    names = r.node._cal.joints[:5]
    for t in [1.0] if jump else [i / 40 for i in range(41)]:
        r.leader_q = {n: a[n] + aas.wrap(inside[n] - a[n]) * t for n in names}
        time.sleep(0.05)
    assert r.wait(lambda: r.node._held, 20.0), r.node.leader_text
    return r, a, b, names


def _leader_moved(r, b, names, d, which=False):
    """b with one leader joint moved by d - the first joint and sign (names order) whose
    moved target the follower's filter accepts (b lies a few mm from the chassis on the
    hulls, so not every direction is clear). which=True: (joint, sign) instead."""
    for jn in names:
        for sign in (1.0, -1.0):
            q = {k: b[k] for k in names}
            q[jn] += sign * d
            if r.node._filter.check_pose({**r.measured_q(), **q}).ok:
                return (jn, sign) if which else q
    pytest.fail('no single-joint move of the leader target is clear')


def test_leader_into_the_chassis_holds_then_plans_to_a_valid_leader(rig_factory, tmp_path):
    """User decision 2026-10-07 (D): the leader moves INTO the chassis -> the follower
    holds (no write while the leader stays there, no edge walking); the leader moves on
    to a valid pose beyond -> a path is planned and followed, every approved step is a
    clear motion and none exceeds the velocity limit. With the CONFIGURED planner
    bounds. Until 2026-10-09 it failed ~6 runs in 10: the planner approved the ideal
    line, the follower's tick-rounded first step was rejected, and the identical search
    repeated every ~2 s (now: the planner validates the executed steps)."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path)
    flt = r.node._filter
    # Under CPU load the follower may still be finishing steps toward a leader pose of
    # the walk-in, or a search started on the way in may still run: wait until it has
    # settled.
    seen = [len(r.node.audit), time.monotonic()]

    def settled():
        if len(r.node.audit) != seen[0]:
            seen[:] = [len(r.node.audit), time.monotonic()]
        # also no search still running from the walk-in (it could adopt a path inside
        # the window below) and no path still being followed
        return (r.node._held and r.node._replan is None and not r.node._follow_steps
                and time.monotonic() - seen[1] > 0.5)
    assert r.wait(settled, 20.0), r.node.leader_text
    n = len(r.node.audit)
    time.sleep(1.5)                                   # leader stays inside: nothing written
    assert [k for k, _ in list(r.node.audit)[n:]] == []
    assert r.node._leader_state == aas.LEADER_ACTIVE
    r.leader_q = {k: b[k] for k in names}             # valid again, straight line blocked
    assert r.wait(lambda: max(abs(r.measured_q()[k] - b[k]) for k in names) < 3e-3, 60.0), \
        (r.node.leader_text, [t for t in list(r.node.status_log) if 're-plan' in t])
    print([t for t in list(r.node.status_log) if 're-plan' in t])
    dmax = 2.0 / r.node._p('leader.rate_hz')
    approved = [sp for k, sp in list(r.node.audit) if k == 'approve']
    poses = [a] + [r.node._q_of({**r.bus.pos, **sp}) for sp in approved]
    for p, q in zip(poses, poses[1:]):
        assert max(abs(q[k] - p[k]) for k in names) <= dmax + 2e-3
        assert flt.check_path(p, q).ok, 'an approved step moves through the body'


def _cpu_bound_plan(log, seconds):
    """Stand-in for a long search: burns CPU (holds the GIL like the real planner) until
    cancelled or `seconds` pass, then finds no path."""
    def plan(self, qa, qb, envelope=None):
        t0 = time.monotonic()
        log.append(('start', t0))
        while time.monotonic() - t0 < seconds and not self._cancelled:
            sum(range(2000))
        log.append(('end', time.monotonic(), self._cancelled))
        return aas.PlanResult(False, bound='cancelled' if self._cancelled else 'time',
                              reason='stand-in search')
    return plan


def test_leader_tick_keeps_its_rate_while_a_long_replan_runs(rig_factory, tmp_path,
                                                            monkeypatch):
    """A search in progress must not block the leader tick: the watchdog still ends
    leader mode in time, and leaving cancels the search."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    log = []
    monkeypatch.setattr(aas.Planner, 'plan', _cpu_bound_plan(log, 20.0))
    r.leader_q = {k: b[k] for k in names}             # valid: a search starts
    assert r.wait(lambda: log, 5.0), r.node.leader_text
    time.sleep(0.5)
    t0 = time.monotonic()
    r.heartbeat = False
    timeout = r.node._p('leader.watchdog_timeout_sec')
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_OFF, timeout + 3.0), \
        r.node.leader_text
    assert 'heartbeat' in r.node.leader_text
    assert time.monotonic() - t0 < timeout + 3.0
    assert r.wait(lambda: len(log) == 2, 3.0), log
    assert log[1][2] is True, 'the search was not cancelled when leader mode ended'


def test_a_replan_cut_by_the_time_cap_is_resumed(rig_factory, tmp_path, monkeypatch):
    """A slice ended by planner.time_budget_s is not a failure: the same search is
    continued (same Planner, verdicts remembered) and its path followed."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    assert r.wait(lambda: r.node._replan is None, 10.0)
    calls, planners = [], set()
    real = aas.Planner.plan

    def sliced(self, qa, qb, envelope=None):
        calls.append(time.monotonic())
        planners.add(id(self))
        if len(calls) <= 2:                           # two slices cut by the cap
            self.fresh_checks = 1
            return aas.PlanResult(False, bound='time', reason='cut (stand-in)')
        return real(self, qa, qb, envelope)
    monkeypatch.setattr(aas.Planner, 'plan', sliced)
    r.leader_q = {k: b[k] for k in names}
    assert r.wait(lambda: max(abs(r.measured_q()[k] - b[k]) for k in names) < 3e-3, 60.0), \
        r.node.leader_text
    # >= 3: under load the real search may itself be cut by the cap and resumed
    assert len(calls) >= 3 and len(planners) == 1, (len(calls), len(planners))
    assert not any('path NONE' in t for t in list(r.node.status_log))


def test_a_replan_slice_without_progress_ends_the_search(rig_factory, tmp_path,
                                                        monkeypatch):
    """A slice that completed no check (one check longer than the time cap) ends the
    search as failed and reported, instead of slicing forever."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    assert r.wait(lambda: r.node._replan is None, 10.0)
    calls = []

    def stuck(self, qa, qb, envelope=None):
        calls.append(time.monotonic())
        self.fresh_checks = 0
        return aas.PlanResult(False, bound='time', reason='cut (stand-in)')
    monkeypatch.setattr(aas.Planner, 'plan', stuck)
    r.leader_q = {k: b[k] for k in names}
    assert _text(r, 'waiting for the leader to move', 10.0), r.node.leader_text
    assert len(calls) == 1
    assert any('not one pose passed within one slice' in t for t in list(r.node.status_log))


def test_unchanged_failed_target_is_not_searched_again(rig_factory, tmp_path, monkeypatch):
    """The planner is deterministic: a search that found no path is reported once and
    repeated only after the leader target moved by more than replan_repeat_tolerance_rad."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True, **DETERMINISTIC,
                                  **{'planner.max_checks': 1})   # direct path only
    # A search may already have run for a pose on the leader's way in: start counting
    # once none is pending (the leader is inside now: no new search starts).
    assert r.wait(lambda: r.node._replan is None, 10.0)
    k = len(r.node.status_log)
    calls = []
    real = aas.Planner.plan

    def spy(self, *args, **kw):
        calls.append(time.monotonic())
        return real(self, *args, **kw)
    monkeypatch.setattr(aas.Planner, 'plan', spy)
    r.leader_q = {k: b[k] for k in names}             # valid, direct path blocked
    assert _text(r, 'waiting for the leader to move', 30.0), r.node.leader_text
    period = r.node._p('leader.replan_period_sec')
    time.sleep(6 * period)
    assert len(calls) == 1, f'{len(calls)} searches for one unchanged target'
    fails = [t for t in list(r.node.status_log)[k:] if 're-plan: path NONE' in t]
    assert len(fails) == 1 and 'checks used up' in fails[0], fails
    assert r.node._leader_state == aas.LEADER_ACTIVE and r.node._held
    tol = r.node._p('leader.replan_repeat_tolerance_rad')
    moved = _leader_moved(r, b, names, 2 * tol)
    r.leader_q = moved                                # the leader moved: a new search
    assert r.wait(lambda: len(calls) == 2, 5.0), r.node.leader_text


def test_a_path_whose_executed_step_is_rejected_is_not_searched_again(rig_factory, tmp_path,
                                                                     monkeypatch):
    """Implementation choice of 5762c7f/a56adbb (2026-10-09, not a user decision): an
    adopted path whose step the follower's own check rejects counts as a FAILED search - the arm holds, and the identical search (same
    leader target, same held pose, same envelope) is not repeated every
    replan_period_sec (the livelock: a ~2 s search every ~2 s, forever). A new search
    starts once the leader moves. Kills 'the rejected step is not recorded'.
    Since 2026-10-20 the first step is checked from the measured pose before the path is
    adopted: that one check passes here, every later two-pose check (the step from the
    measured pose AND the planned step from the commanded one) is refused - a refusal
    that does not come from the measured pose's lag, so it is recorded."""
    from gripperx_arm.safety_filter import Verdict
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    node, flt = r.node, r.node._filter
    assert r.wait(lambda: node._replan is None, 10.0)
    calls = []

    def found(self, qa, qb, envelope=None):           # "finds" the direct path at once
        calls.append(time.monotonic())
        return aas.PlanResult(True, [qa, qb], 'direct')
    monkeypatch.setattr(aas.Planner, 'plan', found)
    real_sp = flt.check_setpoints
    pairs = []

    def check_setpoints(poses, envelope=None, *x, **kw):
        if len(poses) == 2:                           # the follower's per-step check
            pairs.append(1)
            if len(pairs) == 1:                       # the adoption check: passes
                return real_sp(poses, envelope, *x, **kw)
            return Verdict(False, 'body', 'test: executed step rejected')
        if len(poses) > 2:                            # the planner's segment check
            if kw.get('trace') is not None:           # the adoption's re-check: no lift
                kw['trace'].extend([()] * len(poses))
            return Verdict(True, checked=len(poses))
        return real_sp(poses, envelope, *x, **kw)
    flt.check_setpoints = check_setpoints
    n = len(node.audit)
    k = len(node.status_log)
    r.leader_q = {q: b[q] for q in names}
    assert r.wait(lambda: any('recorded as failed' in t for t in list(node.status_log)[k:]),
                  10.0), (node.leader_text, list(node.status_log)[k:])
    assert len(calls) == 1
    assert any('re-plan: path direct' in t for t in list(node.status_log)[k:])
    time.sleep(6 * node._p('leader.replan_period_sec'))
    assert len(calls) == 1, f'{len(calls)} searches for one unchanged target and held pose'
    assert node._held and not node._follow_steps and node._leader_state == aas.LEADER_ACTIVE
    assert 'approve' not in [x for x, _ in list(node.audit)[n:]]
    tol = node._p('leader.replan_repeat_tolerance_rad')
    moved = _leader_moved(r, b, names, 2 * tol)
    r.leader_q = moved                                # the leader moved: a new search
    assert r.wait(lambda: len(calls) == 2, 5.0), node.leader_text


def test_a_step_refused_only_from_the_lagging_measured_pose_is_not_recorded(
        rig_factory, tmp_path, monkeypatch):
    """User decision 2026-10-20: a planned step refused from the measured pose while the
    same step from the commanded pose passes (the arm lags under load) is NOT a failed
    search - a new search from the held pose starts without the leader moving (before:
    recorded as failed, the arm held until the leader moved 0.10 rad). Kills 'every
    execution refusal is recorded'."""
    from gripperx_arm.safety_filter import Verdict
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    node, flt = r.node, r.node._filter
    assert r.wait(lambda: node._replan is None, 10.0)
    calls = []
    real_plan = aas.Planner.plan

    def spy(self, *args, **kw):
        calls.append(time.monotonic())
        return real_plan(self, *args, **kw)
    monkeypatch.setattr(aas.Planner, 'plan', spy)
    real_sp = flt.check_setpoints
    pairs = []

    def check_setpoints(poses, envelope=None, *x, **kw):
        if len(poses) == 2:
            pairs.append(1)
            if len(pairs) == 2:                       # step 1 from the measured pose only
                return Verdict(False, 'ground', 'test: refused from the measured pose')
        return real_sp(poses, envelope, *x, **kw)
    flt.check_setpoints = check_setpoints
    k = len(node.status_log)
    r.leader_q = {q: b[q] for q in names}
    assert r.wait(lambda: any('NOT recorded as a failed search' in t
                              for t in list(node.status_log)[k:]), 30.0), \
        (node.leader_text, list(node.status_log)[k:])
    assert r.wait(lambda: len(calls) >= 2, 10.0), 'no new search after a lag refusal'
    assert not any('recorded as failed -' in t for t in list(node.status_log)[k:])
    assert r.wait(lambda: max(abs(r.measured_q()[q] - b[q]) for q in names) < 3e-3, 60.0), \
        (node.leader_text, list(node.status_log)[k:])


def test_a_discarded_path_is_not_searched_again_for_reading_noise(rig_factory, tmp_path,
                                                                  monkeypatch):
    """Audit 2026-10-20 (ground start): a path discarded at adoption (its first step
    refused from the measured pose) is not looked at again while the measured pose stays
    within leader.replan_discard_move_ticks of where it was discarded - one tick of
    reading noise restarted the identical search every replan_period_sec (the livelock
    of 2026-10-08). Once the arm has really moved, the CACHED path is re-checked from the
    new measured pose - no new search (MED-4 follow-up). Kills 'the discard gate compares
    the measured ticks exactly', 'no discard gate' and 'a moved arm searches again'."""
    from gripperx_arm.safety_filter import Verdict
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    node, flt = r.node, r.node._filter
    assert r.wait(lambda: node._replan is None, 10.0)
    calls = []

    def found(self, qa, qb, envelope=None):           # "finds" the direct path at once
        calls.append(time.monotonic())
        return aas.PlanResult(True, [qa, qb], 'direct')
    monkeypatch.setattr(aas.Planner, 'plan', found)
    real_sp = flt.check_setpoints

    def check_setpoints(poses, envelope=None, *x, **kw):
        if len(poses) == 2:                           # every step from the measured pose
            return Verdict(False, 'ground', 'test: refused from the measured pose')
        if len(poses) > 2:                            # the planner's segment check
            if kw.get('trace') is not None:
                kw['trace'].extend([()] * len(poses))
            return Verdict(True, checked=len(poses))
        return real_sp(poses, envelope, *x, **kw)
    flt.check_setpoints = check_setpoints
    n = len(node.audit)
    k = len(node.status_log)
    r.leader_q = {q: b[q] for q in names}
    assert r.wait(lambda: any('DISCARDED (not a failed search)' in t
                              for t in list(node.status_log)[k:]), 10.0), \
        (node.leader_text, list(node.status_log)[k:])
    assert node._replan_discarded is not None and len(calls) == 1
    period = node._p('leader.replan_period_sec')
    drift = int(node._p('leader.replan_discard_move_ticks'))
    sid = aas.LIMB_IDS[0]
    base = r.bus.pos[sid]
    for i in range(6):                                # reading noise within the drift
        r.bus.pos[sid] = base + (drift if i % 2 == 0 else 0)
        time.sleep(period)
    assert len(calls) == 1, f'{len(calls)} searches for reading noise'
    assert 'approve' not in [x for x, _ in list(node.audit)[n:]]
    k = len(node.status_log)
    r.bus.pos[sid] = base + drift + 1                 # the arm moved: re-checked, cached
    assert r.wait(lambda: any('re-checked after the arm moved' in t
                              for t in list(node.status_log)[k:]), 5.0), node.leader_text
    assert len(calls) == 1


def test_a_running_replan_is_dropped_when_its_inputs_move(rig_factory, tmp_path, monkeypatch):
    """A running search is kept while the leader target and the held pose stay within
    replan_repeat_tolerance_rad of its inputs, and cancelled (its result would be stale) once
    either moves farther; a search for the new inputs follows. Kills 'Mb' (the stale
    job is never dropped) and its opposite (dropped on any change)."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    node = r.node
    assert r.wait(lambda: node._replan is None, 10.0)
    log = []
    monkeypatch.setattr(aas.Planner, 'plan', _cpu_bound_plan(log, 30.0))
    tol = node._p('leader.replan_repeat_tolerance_rad')
    target = {k: b[k] for k in names}
    r.leader_q = dict(target)
    assert r.wait(lambda: log and node._replan is not None, 5.0), node.leader_text
    job = node._replan

    jn, sign = _leader_moved(r, target, names, 2 * tol, which=True)

    def leader(d):
        q = dict(target)
        q[jn] += sign * d
        assert node._filter.check_pose({**r.measured_q(), **q}).ok
        r.leader_q = q
    leader(tol / 2)                                   # within the tolerance: kept
    time.sleep(1.0)
    assert node._replan is job and not job.planner._cancelled and len(log) == 1
    leader(2 * tol)                                   # farther: cancelled, searched anew
    assert r.wait(lambda: job.planner._cancelled, 3.0), 'the stale search kept running'
    assert r.wait(lambda: node._replan is not None and node._replan is not job, 5.0)
    assert node._replan.target[jn] == pytest.approx(target[jn] + sign * 2 * tol, abs=1e-3)
    # The held pose moving that far (still inside leader.command_tolerance_rad, so leader
    # mode stays on) drops the search as well. The HELD pose is the goals: since the
    # clamped start (user decision 2026-10-30) a moved arm alone does not move it.
    job2 = node._replan
    assert r.wait(lambda: len(log) >= 3, 3.0), log
    sid = aas.IDS[node._cal.joints.index('arm_limb1_joint')]
    held = node._q_of(dict(r.bus.pos))
    held['arm_limb1_joint'] += 2 * tol
    assert 2 * tol < node._p('leader.command_tolerance_rad')
    with node._serial_lock:
        node._goal[sid] = node._ticks_of(held)[sid]
    assert r.wait(lambda: job2.planner._cancelled, 3.0), 'search kept after the goals moved'
    assert node._leader_state == aas.LEADER_ACTIVE, node.leader_text


def test_failed_search_is_not_restarted_by_steering_jitter(rig_factory, tmp_path, monkeypatch):
    """Audit round 3: _envelope() builds a new envelope object whenever a wheel reading
    changes at its 0.01 rad rounding or freshness flickers, and the failed-search memory
    compared envelopes by IDENTITY - steering jitter restarted the same failed search
    every replan_period_sec. Now only a change that can free space restarts it: a wheel
    newly known, or moved by more than steer_motion_margin_rad. Kills the identity
    compare (jitter phase) and 'never restart on the envelope' (last phase)."""
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    node = r.node
    assert r.wait(lambda: node._replan is None, 10.0)
    calls = []

    def fail(self, qa, qb, envelope=None):
        calls.append(envelope)
        return aas.PlanResult(False, bound='checks', reason='stand-in: no path')
    monkeypatch.setattr(aas.Planner, 'plan', fail)
    margin = node._p('safety_filter.steer_motion_margin_rad')

    def measured(v, first=None):
        return node._filter.envelope_measured(
            {n: (first if (first is not None and i == 0) else v)
             for i, n in enumerate(node._steer_names)}, margin)
    e1, e2 = measured(0.0001), measured(0.0101)       # 0.01 rad apart: one rounding step
    mode = ['stale']
    flip = [0]

    def env():
        flip[0] += 1
        if mode[0] == 'stale':
            return None                               # every wheel swept
        if mode[0] == 'jitter':
            return e1 if flip[0] % 2 else e2          # a new object on every tick
        return e3
    e3 = measured(0.0001, first=0.0001 + 2 * margin)  # one wheel moved beyond the margin
    node._envelope = env
    for e in (None, e1, e2, e3):
        assert node._filter.check_pose({**r.measured_q(), **{k: b[k] for k in names}}, e).ok
    period = node._p('leader.replan_period_sec')
    r.leader_q = {k: b[k] for k in names}
    assert r.wait(lambda: len(calls) == 1, 5.0), node.leader_text
    mode[0] = 'jitter'                                # stale -> fresh: may free space
    assert r.wait(lambda: len(calls) == 2, 5.0), 'a wheel newly known did not restart'
    time.sleep(8 * period)
    assert len(calls) == 2, f'{len(calls) - 2} restarts from sub-margin jitter'
    mode[0] = 'stale'                                 # fresh -> stale: only more body
    time.sleep(4 * period)
    assert len(calls) == 2, 'a wheel going stale restarted the failed search'
    mode[0] = 'moved'                                 # one wheel 2 x margin away: restart
    assert r.wait(lambda: len(calls) == 3, 5.0), 'a wheel moved beyond the margin did not restart'
    mode[0] = 'jitter'                                # and back by 2 x margin: restart
    assert r.wait(lambda: len(calls) == 4, 5.0), 'the wheel moving back did not restart'
    time.sleep(4 * period)
    assert len(calls) == 4


def test_envelope_may_free(rig_factory):
    """The verdict rule itself, per case."""
    r = rig_factory()
    node = r.node
    m = node._p('safety_filter.steer_motion_margin_rad')
    names = node._steer_names

    def env(*vals):
        return node._filter.envelope_measured(dict(zip(names, vals)), m)
    known = env(0.1, 0.1, 0.1, 0.1)
    free = node._envelope_may_free
    assert not free(known, known) and not free(None, None)
    assert not free(known, env(0.1 + m / 2, 0.1, 0.1, 0.1))         # jitter below the margin
    assert free(known, env(0.1 + 1.5 * m, 0.1, 0.1, 0.1))           # moved beyond it
    assert free(known, env(0.1 - 1.5 * m, 0.1, 0.1, 0.1))
    assert not free(known, env(None, 0.1, 0.1, 0.1))                # went stale: swept
    assert not free(known, None)
    assert free(env(None, 0.1, 0.1, 0.1), known)                    # newly known
    assert free(None, known)
    foreign = type(known)([], 'not from envelope_measured')
    assert free(known, foreign) and not free(foreign, foreign)      # unknown origin: identity


def test_a_replan_result_from_a_replaced_calibration_is_discarded(rig_factory, tmp_path,
                                                                 monkeypatch):
    """A search keeps the calibration it started with; if the node's calibration was
    replaced meanwhile, its result is reported and discarded, never followed."""
    import copy
    r, a, b, names = _chassis_rig(rig_factory, tmp_path, jump=True)
    node = r.node
    assert r.wait(lambda: node._replan is None, 10.0)
    gate = threading.Event()
    calls = []

    def gated(self, qa, qb, envelope=None):
        calls.append(1)
        if len(calls) > 1:                            # later searches: no path
            return aas.PlanResult(False, bound='checks', reason='stand-in: no path')
        gate.wait(20.0)                               # the first one "finds" a path
        return aas.PlanResult(True, [qa, qb], 'direct')
    monkeypatch.setattr(aas.Planner, 'plan', gated)
    r.leader_q = {k: b[k] for k in names}
    assert r.wait(lambda: node._replan is not None, 5.0), node.leader_text
    job = node._replan
    assert job.cal is node._cal
    node._cal = copy.copy(node._cal)                  # same values, a replaced object
    n = len(node.audit)
    gate.set()
    assert r.wait(lambda: any('since replaced' in t for t in list(node.status_log)), 60.0)
    assert not node._follow_steps and node._held
    assert [k for k, _ in list(node.audit)[n:]] == []


# ------------------------------------------------------------------ Space
def test_space_while_leader_streams_torque_off(rig_factory):
    r = rig_factory(**ALLOW)
    _activate(r)
    _nudge(r, d=0.2)
    r.stop.publish(Empty())
    assert r.wait(lambda: 'torque_off' in r.kinds(), 2.0)
    assert r.node._leader_state == aas.LEADER_OFF
    i = r.kinds().index('torque_off')
    time.sleep(r.node._grace + 1.0)
    assert r.kinds()[i + 1:] == [], 'something written after the torque-off'
    assert not any(r.bus.torque_on.values())


def test_space_while_stationary_does_nothing(rig_factory):
    r = rig_factory()
    time.sleep(r.node._grace + 0.1)
    n = len(r.bus.events)
    r.stop.publish(Empty())
    time.sleep(0.8)
    assert r.bus.events[n:] == []
    assert all(r.bus.torque_on.values())


# ------------------------------------------------------------------ pick / fixed poses
def _send_pick(r, execute=False, epoch=None):
    """execute=False and a DISARMED gateway = the operator's P (keyboard teleop sends a
    default goal); execute=True = the gateway's goal (SR-17 rule 11, _pick_trigger)."""
    # Generous: action discovery and goal acceptance are asynchronous and slow on a
    # loaded machine; a short wait here made pick tests flaky.
    client = r.pick_client
    assert client.wait_for_server(timeout_sec=60.0)
    if r._armed_sent:            # the arming state must have ARRIVED (async discovery)
        assert r.wait(lambda: r.node._gateway_armed is not None, 20.0), 'no arming state'
    if epoch is None:            # a sender that has seen the current epoch
        assert r.wait(lambda: r.epoch == r.node._epoch_now(), 20.0), 'no stop epoch'
        epoch = r.epoch
    fut = client.send_goal_async(PickPlastic.Goal(execute=execute, stop_epoch=epoch))
    assert r.wait(lambda: fut.done(), 20.0)
    return fut.result()


GRIP_NEAR_HOME = {'poses.grip_rad': [0.3, 0.0, 0.0, 0.0, 0.3, 0.0]}


def test_pick_rejected_while_leader_active(rig_factory):
    """A gateway P is rejected at once; the operator's P waits for the leader disable
    (leader.command_wait_sec) and is refused when leader mode stays on."""
    r = rig_factory(**ALLOW)
    _activate(r)
    assert not _send_pick(r, execute=True).accepted
    n = len(r.node.audit)
    handle = _send_pick(r)
    assert handle.accepted
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 20.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'still on after waiting' in msg, msg
    assert r.node._leader_state == aas.LEADER_ACTIVE
    assert [k for k, _ in list(r.node.audit)[n:]] == []


def test_leader_refused_during_pick_and_cancel_returns_home(rig_factory):
    r = rig_factory(drop=('poses.grip',), **ALLOW, **GRIP_NEAR_HOME)
    handle = _send_pick(r)
    assert handle.accepted
    time.sleep(1.0)
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    r.rising_edge()
    assert _text(r, '"pick" is running'), r.node.leader_text
    r.leader_q = None
    handle.cancel_goal_async()
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 8.0)
    assert 'returning home' in res.result().result.message
    assert r.wait(lambda: r.bus.pos == HOME_D, 20.0)


def test_pick_with_a_grip_pose_outside_the_limits_is_refused_before_moving(rig_factory):
    """User decision 2026-10-07 (B): P is planned. A grip pose outside the joint limits
    -> refused, nothing written, not even the gripper. (Until 2026-10-08 this used the
    configured grip pose, which lay outside the limits only under the placeholder
    calibration; the pose is now built here: limb3 0.3 rad below its lower limit.)"""
    from gripperx_geometry import constants as C
    r = rig_factory(drop=('poses.grip',), **{
        'poses.grip_rad': [0.0, 0.0, C.ARM_JOINT_LOWER[2] - 0.3, 0.0, 0.0, 0.0]})
    n = len(r.bus.writes)
    handle = _send_pick(r)
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'NO_PATH' in msg and 'limits' in msg, msg
    assert len(r.bus.writes) == n


def test_pick_near_home_runs_planned_and_gripper_unchecked(rig_factory):
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    handle = _send_pick(r)
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 40.0)
    assert res.result().result.success, res.result().result.message
    kinds = set(r.kinds()) - {'torque_on_sync', 'torque_on'}
    assert kinds == {'approve', 'gripper_unchecked'}, kinds
    # the gripper-only writes touch only the gripper, the arm writes never the gripper
    for k, sp in list(r.node.audit):
        if k == 'gripper_unchecked':
            assert set(sp) == {aas.GRIPPER_ID}
        if k == 'approve':
            assert aas.GRIPPER_ID not in sp


def test_without_filter_p_and_i_refused_o_works(rig_factory):
    r = rig_factory(description=False)
    assert r.node._filter is None
    n = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    time.sleep(1.0)
    assert len(r.bus.writes) == n
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 10.0) and not res.result().result.success
    assert len(r.bus.writes) == n
    r.cmd.publish(String(data='open_gripper'))
    assert r.wait(lambda: r.bus.pos[aas.GRIPPER_ID] == r.node._grip_open, 5.0)


def test_planned_detour_every_segment_approved(rig_factory, tmp_path):
    """The direct path to the goal crosses the chassis: a detour is planned and every
    pair of consecutive written arm setpoints is a clear motion."""
    def start(node):
        return _ik(node, 0.10, 0.25, 0.0, 'front/elbow-')
    # _plan_arm is one-shot (no resume): this test is about the written segments, not
    # about whether the detour fits planner.time_budget_s on a loaded machine.
    r = rig_factory(start_q=start, **DETERMINISTIC, **{
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'c.yaml')})
    flt = r.node._filter
    goal_q = _ik(r.node, 0.30, 0.0, 0.05, 'front/elbow-')
    goal = r.node._ticks_of(goal_q)
    here = dict(r.bus.pos)
    assert not flt.check_path(r.measured_q(), goal_q).ok
    res = r.node._plan_arm(here, goal, 'test', automatic=False)
    assert res.ok and res.method != 'direct'
    n = len(r.node.audit)
    assert r.node._claim('test')
    try:
        r.node._ramp(r.node._path_segments(res.path, res.method, None, here), 'test',
                     'approve')
    finally:
        r.node._release('test')
    poses = [r.node._q_of(here)] + [r.node._q_of({**here, **sp})
                                     for k, sp in list(r.node.audit)[n:] if k == 'approve']
    for p, q in zip(poses, poses[1:]):
        assert flt.check_path(p, q).ok
    assert all(abs(r.bus.pos[s] - goal[s]) <= 1 for s in aas.LIMB_IDS)


def test_no_path_home_holds(rig_factory, tmp_path):
    """Return home after an abort with the START refused (the arm is measured inside
    the chassis): the arm holds, no arm setpoint is written. The search-exhausted
    branch: test_rrt_exhausted_return_home_holds."""
    def start(node):
        return _ik(node, 0.30, -0.12, -0.12, 'front/elbow-')     # in the body
    r = rig_factory(start_q=start, **{
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'c.yaml')})
    assert not r.node._filter.check_pose(r.measured_q()).ok
    n = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: 'hold' in [k for k, _ in list(r.node.audit)[n:]], 30.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]
    # _hold writes the audit entry BEFORE it reports: wait for the text, never peek
    assert r.wait(lambda: any('NOT possible' in t for t in list(r.node.status_log)), 5.0)


def test_every_bus_write_is_exactly_an_audited_setpoint(rig_factory):
    r = rig_factory()
    r.cmd.publish(String(data='open_gripper'))
    time.sleep(3.2)
    expanded = [(sid, t) for k, sp in list(r.node.audit) if k not in ('torque_on', 'torque_off')
                for sid, t in sp.items()]
    assert r.bus.writes == expanded


def test_missing_filter_yaml_refuses_p_i_and_leader_o_runs(rig_factory):
    """SR-17 rule 12: no filter configuration -> P and I refused (nothing written),
    the gripper (O) still runs; leader mode refused."""
    r = rig_factory(drop=('poses.grip',), **ALLOW, **GRIP_NEAR_HOME,
                    **{'safety_filter.config_loaded': False})
    assert r.node._filter is not None             # the model is there, the config is not
    n = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: any('not loaded' in t for t in list(r.node.status_log)), 3.0)
    assert len(r.bus.writes) == n
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 10.0)
    assert not res.result().result.success and 'not loaded' in res.result().result.message
    assert len(r.bus.writes) == n
    r.cmd.publish(String(data='open_gripper'))
    assert r.wait(lambda: r.bus.pos[aas.GRIPPER_ID] == r.node._grip_open, 5.0)
    time.sleep(0.5)
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    r.rising_edge()
    assert _text(r, 'not loaded'), r.node.leader_text


def test_second_heartbeat_publisher_refused(rig_factory):
    r = rig_factory(**ALLOW)
    r.calv.publish(Bool(data=True))
    rival = r.tester.create_publisher(Bool, '/arm/leader/active', 10)
    r.leader_q = r.measured_q()
    time.sleep(0.5)
    r.rising_edge()
    assert _text(r, '/arm/leader/active'), r.node.leader_text
    del rival


def test_launch_files_load_the_filter_yaml():
    for f in ('gripperx_bringup/launch/real_robot.launch.py',
              'gripperx_bringup/launch/teleop_real.launch.py',
              'gripperx_arm/launch/arm.launch.py'):
        p = SRC / f
        if p.is_file():
            assert 'arm_safety_filter.yaml' in p.read_text(), f


def _windowed_calibration(path: Path, centers) -> str:
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    doc['arm_joint_state_relay']['ros__parameters']['range_center_rad'] = centers
    path.write_text(yaml.safe_dump(doc))
    return str(path)


def test_limits_outside_the_window_refuse_the_filter(rig_factory, tmp_path):
    """URDF limits are +-pi; a window centred at 1 rad cannot hold them."""
    r = rig_factory(**{'safety_filter.calibration_file':
                       _windowed_calibration(tmp_path / 'c.yaml', [1.0, 0, 0, 0, 0, 0])})
    assert r.node._filter is None and 'window' in r.node._filter_error


def test_leader_step_across_the_window_edge_is_rejected(rig_factory):
    """A leader value that crossed its window edge (2 pi away) is never driven the
    long way round."""
    r = rig_factory(**ALLOW)
    _activate(r)
    n = len(r.node.audit)
    q = dict(r.leader_q)
    q['arm_limb5_joint'] += 2 * math.pi - 0.02     # physically 0.02 rad, numerically ~2 pi
    r.leader_q = q
    assert _text(r, 'window edge'), r.node.leader_text
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]


def test_gripper_open_as_angle_relative_to_home(rig_factory):
    """gripper.open_rad (angle relative to home) drives the same tick target as the
    tick variant it was converted from."""
    from gripperx_arm import poses as P
    cal = P.calibration_from_doc(yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml')
                                                .read_text()))
    open_ticks = yaml.safe_load((PKG / 'config' / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']['gripper']['open']
    open_rad = cal.to_radians(HOME[:5] + [open_ticks])['arm_gripper_joint']
    r = rig_factory(drop=('gripper.open',), **{'gripper.open_rad': open_rad})
    assert r.node._grip_open == open_ticks
    r.cmd.publish(String(data='open_gripper'))
    assert r.wait(lambda: r.bus.pos[aas.GRIPPER_ID] == open_ticks, 5.0)


def test_poses_home_parameter_is_refused(rig_factory):
    r = rig_factory(**{'poses.home': [1, 2, 3, 4, 5, 6]})
    assert r.node._home_pos is None and 'one source' in r.node._poses_error
    n = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    time.sleep(0.8)
    assert len(r.bus.writes) == n


def test_leader_step_across_the_servo_tick_wrap_is_refused(rig_factory, tmp_path):
    """limb5's zero sits 3 ticks below the servo wrap: a small physical step that
    crosses 4095 -> 0 would make the servo turn the long way - refused, not written."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    zeros = list(doc['arm_joint_state_relay']['ros__parameters']['zero_ticks'])
    zeros[4] = 4092
    doc['arm_joint_state_relay']['ros__parameters']['zero_ticks'] = zeros
    (tmp_path / 'c.yaml').write_text(yaml.safe_dump(doc))
    r = rig_factory(start_q=lambda node: {n: 0.0 for n in node._cal.joints}, **ALLOW,
                    **{'safety_filter.calibration_file': str(tmp_path / 'c.yaml')})
    assert r.bus.pos[5] == 4092
    _activate(r)
    n = len(r.node.audit)
    q = dict(r.leader_q)
    q['arm_limb5_joint'] += 0.03 * doc['arm_joint_state_relay']['ros__parameters']['signs'][4]
    r.leader_q = q
    assert _text(r, 'tick wrap'), r.node.leader_text
    assert all(sp.get(5, 0) >= 4092 for k, sp in list(r.node.audit)[n:] if k == 'approve')


def test_eeprom_enforce_keeps_torque_off_until_the_limits_match(rig_factory):
    """eeprom_check: enforce (after the calibration session): the mock servos report
    no limits -> torque stays off and no goal is written; log mode (default) is the
    startup test above."""
    r = rig_factory(**{'eeprom_check': 'enforce'})
    assert not any(e[0] == 'torque' and e[2] for e in r.bus.events)
    assert not r.node._torque_on
    r.cmd.publish(String(data='open_gripper'))
    time.sleep(1.0)
    assert r.bus.writes == []


def _l1_registers(margin_min, margin_max, offsets=None):
    """What arm_eeprom_limits would have written under the active description and
    calibration, with the given margins; Homing_Offset (31) as given, default: the
    offsets the active calibration records (0 where none)."""
    from gripperx_arm import eeprom_limits as el
    cal = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())[
        'arm_joint_state_relay']['ros__parameters']
    offsets = offsets or el.recorded_offsets(cal) or [0] * 6
    from gripperx_arm.kinematics import ArmKinematics
    rows = el.compute(ArmKinematics(_urdf()).limits, cal['urdf_joints'], cal['zero_ticks'],
                      cal['signs'], cal['ticks_per_rev'], margin_min, margin_max)
    return {(r['id'], a): v for r, off in zip(rows, offsets)
            for a, v in ((9, r['min']), (11, r['max']), (31, off))}


def test_eeprom_enforce_uses_the_configured_margin(rig_factory, monkeypatch):
    """The node compares against the SAME margins the tool writes with (one source,
    arm_safety_filter.yaml): servos written with margins (3, 7) pass under (3, 7)."""
    regs = _l1_registers([3] * 6, [7] * 6)
    monkeypatch.setattr(MockBus, 'read_word', lambda self, sid, addr: regs.get((sid, addr)))
    r = rig_factory(**{'eeprom_check': 'enforce', 'eeprom_margin_min_ticks': [3] * 6,
                       'eeprom_margin_max_ticks': [7] * 6})
    assert r.wait(lambda: r.node._torque_on, 10.0), getattr(r.node, '_eeprom_error', '')
    assert r.node._eeprom_ok


def test_eeprom_enforce_refusal_names_the_remedy(rig_factory, monkeypatch):
    """Servos written with other margins (or before a calibration change): torque
    stays off, and the refusal tells the operator what to run."""
    regs = _l1_registers([3] * 6, [7] * 6)
    monkeypatch.setattr(MockBus, 'read_word', lambda self, sid, addr: regs.get((sid, addr)))
    r2 = rig_factory(**{'eeprom_check': 'enforce', 'eeprom_margin_min_ticks': [4] * 6,
                        'eeprom_margin_max_ticks': [7] * 6})
    assert r2.wait(lambda: getattr(r2.node, '_eeprom_error', ''), 10.0)
    assert not r2.node._torque_on
    assert 'arm_eeprom_limits --port /dev/arm_servo' in r2.node._eeprom_error
    assert 'DEPLOYMENT.md' in r2.node._eeprom_error


def test_eeprom_enforce_with_the_active_margins_matches_what_the_tool_writes(
        rig_factory, monkeypatch):
    """Audit EE1 L3: the ACTIVE arm_safety_filter.yaml margins, fed through the tool's
    loader (what --write writes) and the node's own parameters (what enforce
    compares, no override), agree."""
    from gripperx_arm import eeprom_limits as el
    regs = _l1_registers(*el.load_margins(PKG / 'config'))
    monkeypatch.setattr(MockBus, 'read_word', lambda self, sid, addr: regs.get((sid, addr)))
    r = rig_factory(**{'eeprom_check': 'enforce'})
    assert r.wait(lambda: r.node._torque_on, 10.0), getattr(r.node, '_eeprom_error', '')


def test_eeprom_enforce_unreadable_registers_do_not_send_the_operator_to_rewrite(rig_factory):
    """Audit EE1 L4: a failed read is a bus problem, not a stale EEPROM."""
    r = rig_factory(**{'eeprom_check': 'enforce'})
    assert r.wait(lambda: getattr(r.node, '_eeprom_error', ''), 10.0)
    assert 'UNREADABLE' in r.node._eeprom_error
    assert 'arm_eeprom_limits --port' not in r.node._eeprom_error
    assert not r.node._torque_on


LEADER_OFFSETS = [3624, 3353, 1323, 1748, 3325, 0]       # leader read 2026-10-13, raw


@pytest.mark.parametrize('servo_offsets,ok', [(LEADER_OFFSETS, True), ([0] * 6, False)])
def test_eeprom_enforce_checks_the_recorded_homing_offsets(rig_factory, monkeypatch, tmp_path,
                                                            servo_offsets, ok):
    """Audit EE1 M3 proposal: with homing_offsets_raw recorded in the calibration,
    enforce accepts exactly those offsets and refuses a changed one."""
    from gripperx_arm import eeprom_limits as el
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    doc['arm_joint_state_relay']['ros__parameters']['homing_offsets_raw'] = LEADER_OFFSETS
    cal = tmp_path / 'cal.yaml'
    cal.write_text(yaml.safe_dump(doc))
    regs = _l1_registers(*el.load_margins(PKG / 'config'), offsets=servo_offsets)
    monkeypatch.setattr(MockBus, 'read_word', lambda self, sid, addr: regs.get((sid, addr)))
    r = rig_factory(**{'eeprom_check': 'enforce', 'safety_filter.calibration_file': str(cal)})
    if ok:
        assert r.wait(lambda: r.node._torque_on, 10.0), getattr(r.node, '_eeprom_error', '')
    else:
        assert r.wait(lambda: getattr(r.node, '_eeprom_error', ''), 10.0)
        assert 'Homing_Offset' in r.node._eeprom_error and not r.node._torque_on


# ------------------------------------------------------------------ re-audit cf3fbe0
def test_space_in_a_pick_settle_window_stays_limp(rig_factory):
    """F1: Space between two pick steps: no torque-on afterwards."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    handle = _send_pick(r)
    assert handle.accepted
    grip = r.node._grip_pos
    # the approach has arrived (all arm servos at the grip pose): settle window (0.8 s)
    assert r.wait(lambda: all(r.bus.pos[s] == grip[s] for s in aas.LIMB_IDS), 20.0)
    r.stop.publish(Empty())
    assert r.wait(lambda: 'torque_off' in r.kinds(), 2.0)
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 10.0)
    k = r.kinds()
    assert k[k.index('torque_off') + 1:] == [], k[k.index('torque_off'):]
    assert not any(r.bus.torque_on.values())
    assert 'torque off' in res.result().result.message


def test_leader_step_motion_is_checked_not_only_its_end(rig_factory, tmp_path):
    """F3a: the leader jumps to a clear pose whose straight path crosses a wheel; the
    follower must not move there in one step - every pair of consecutive approved
    setpoints is a clear motion under the filter."""
    def start(node):
        return _ik(node, 0.10, -0.25, 0.05, 'front/elbow-')
    r = rig_factory(start_q=start, **ALLOW, **{
        'leader.max_joint_velocity_rad_s': 60.0,         # whole jump in one tick
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'c.yaml')})
    flt = r.node._filter
    a = r.measured_q()
    b = _ik(r.node, 0.35, -0.20, -0.05, 'front/elbow-')
    assert flt.check_pose(b).ok and not flt.check_path(a, b).ok
    _activate(r)
    n = len(r.node.audit)
    r.leader_q = {k: b[k] for k in r.node._cal.joints[:5]}
    time.sleep(2.0)
    approved = [sp for k, sp in list(r.node.audit)[n:] if k == 'approve']
    poses = [a] + [r.node._q_of({**r.bus.pos, **sp}) for sp in approved]
    for p, q in zip(poses, poses[1:]):
        assert flt.check_path(p, q).ok, 'an approved leader step moves through the body'


def test_exact_zero_steering_reading_is_not_trusted(rig_factory):
    """F3b: 0.0 is steer_servo_node's value for a dropped channel."""
    from std_msgs.msg import Float64MultiArray
    r = rig_factory()
    r.node._on_steer_states(Float64MultiArray(data=[0.0, 0.1, -0.1, 0.2]))
    st = r.node._steer
    assert st['f_left_steer'] is None
    assert st['f_right_steer'] is not None and st['b_rightsteer'] is not None


def test_two_pi_mismatch_is_not_a_match(rig_factory):
    """F4: a physical 2 pi - eps difference is a mismatch."""
    r = rig_factory(**ALLOW)
    r.calv.publish(Bool(data=True))
    q = r.measured_q()
    q['arm_limb1_joint'] += 2 * math.pi - 0.01
    r.leader_q = q
    n = r.node
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and time.monotonic() - n._leader_rx < 0.5, 20.0)
    r.rising_edge()
    assert _text(r, 'does not match'), r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF


def test_space_at_the_return_home_deadline_is_not_lost(rig_factory):
    """F5: Space arriving when the pending return home has just been taken: the arm
    is already claimed, so Space turns torque off and nothing is written."""
    r = rig_factory()
    gate = threading.Event()
    real = r.node._arm_home

    def gated(*a, **kw):
        gate.wait(5.0)
        return real(*a, **kw)
    r.node._arm_home = gated
    r.node._pending_home = (time.monotonic() - 1.0, 'test')
    assert r.wait(lambda: r.node._motion_owner == 'abort_home', 2.0)
    n = len(r.node.audit)
    r.node._on_stop(Empty())
    gate.set()
    time.sleep(1.0)
    after = [k for k, _ in list(r.node.audit)[n:]]
    assert after == ['torque_off'], after


def test_space_before_the_deadline_cancels_the_return_home(rig_factory):
    r = rig_factory()
    r.node._pending_home = (time.monotonic() + 0.3, 'test')
    n = len(r.node.audit)
    r.node._on_stop(Empty())
    time.sleep(1.0)
    assert r.node._pending_home is None
    assert [k for k, _ in list(r.node.audit)[n:]] == []


# ------------------------------------------------------------------ re-audit 310eb12/3c96a8a
def _wrap_calibration(path: Path, limb5_zero=4092) -> tuple:
    """limb5's zero a few ticks below the servo wrap: a small positive (sign-adjusted)
    limb5 angle lies across 4095 -> 0 from home."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    c = doc['arm_joint_state_relay']['ros__parameters']
    zeros = list(c['zero_ticks'])
    zeros[4] = limb5_zero
    c['zero_ticks'] = zeros
    path.write_text(yaml.safe_dump(doc))
    return str(path), int(c['signs'][4])


def test_return_planning_failure_moves_nothing(rig_factory):
    """Approach planning succeeds, return planning fails -> NOTHING is written, not
    even the gripper. Kills 'return path planned only at the return step' (M7)."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    real = r.node._plan_arm
    sources = []

    def plan(start, goal, source, envelope=None, automatic=True, **kw):
        sources.append(source)
        if 'return_home' in source:
            raise aas.MotionRefused(f'NO_PATH {source}: injected')
        return real(start, goal, source, envelope, automatic, **kw)
    r.node._plan_arm = plan
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'NO_PATH' in msg, msg
    assert sources == ['pick_plastic/approach', 'pick_plastic/return_home'], sources
    assert len(r.bus.writes) == n


def test_tick_wrap_path_is_no_path_and_moves_nothing(rig_factory, tmp_path):
    """The planner knows the servo tick wrap: a grip pose across 4095 -> 0 from home
    has no executable path (a revolute joint cannot go round the other way inside its
    limits) -> NO_PATH before any write, the reason names the wrap."""
    cal, sign = _wrap_calibration(tmp_path / 'c.yaml')
    r = rig_factory(start_q=lambda node: {n: 0.0 for n in node._cal.joints},
                    drop=('poses.grip',), **{
        'poses.grip_rad': [0.3, 0.0, 0.0, 0.0, 0.3 * sign, 0.0],
        'safety_filter.calibration_file': cal, 'planner.max_checks': 40})
    assert r.bus.pos[5] == 4092
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'NO_PATH' in msg and 'tick wrap' in msg, msg
    assert len(r.bus.writes) == n


def test_tick_conversion_runs_before_the_first_write(rig_factory, tmp_path):
    """Even if a planner accepted a wrap-crossing path (edge check removed here), the
    tick conversion of ALL segments happens before the first write: the refusal comes
    with zero writes, not after the gripper opened."""
    cal, sign = _wrap_calibration(tmp_path / 'c.yaml')
    r = rig_factory(start_q=lambda node: {n: 0.0 for n in node._cal.joints},
                    drop=('poses.grip',), **{
        'poses.grip_rad': [0.3, 0.0, 0.0, 0.0, 0.3 * sign, 0.0],
        'safety_filter.calibration_file': cal})
    node = r.node

    def planner(budget_s, direct_only=False, **_kw):
        return aas.Planner(node._filter, node._cal.joints[:5], budget_s,
                           max_checks=int(node._p('planner.max_checks')),
                           direct_only=direct_only)                           # no edge_check
    node._planner = planner
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'tick wrap' in msg, msg
    assert len(r.bus.writes) == n


def test_rrt_exhausted_return_home_holds(rig_factory, tmp_path):
    """Return home after an abort: start and goal valid, direct path and every
    joint-ordered detour blocked, the RRT runs out of checks -> the arm holds and
    reports, no arm setpoint written. (The 'home' target is replaced by a pose whose
    direct path from the start crosses the chassis.)"""
    def start(node):
        return _ik(node, 0.10, 0.25, 0.0, 'front/elbow-')
    # 'detour': the RRT branch of an automatic return home is reachable only with the
    # SR-17 rule 11 switch unlocked; the locked default: test_rule11_*.
    r = rig_factory(start_q=start, **DETERMINISTIC, **{
        'planner.max_checks': 14, 'planner.automatic_paths': 'detour',
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'c.yaml')})
    node = r.node
    goal_q = _ik(node, 0.30, 0.0, 0.05, 'front/elbow-')
    assert node._filter.check_pose(r.measured_q()).ok and node._filter.check_pose(goal_q).ok
    assert not node._filter.check_path(r.measured_q(), goal_q).ok
    node._home_pos = node._ticks_of(goal_q)
    entered = []
    real_rrt = aas.Planner._rrt

    def spy(self, *a):
        entered.append(self._checks)
        return real_rrt(self, *a)
    aas.Planner._rrt = spy
    try:
        n = len(node.audit)
        node._schedule_home('test')
        assert r.wait(lambda: 'hold' in [k for k, _ in list(node.audit)[n:]], 60.0), \
            list(node.status_log)[-4:]
    finally:
        aas.Planner._rrt = real_rrt
    assert entered, 'the RRT stage was never reached'
    assert 'approve' not in [k for k, _ in list(node.audit)[n:]]
    assert any('checks used up' in t for t in list(node.status_log)), list(node.status_log)[-4:]
    assert r.wait(lambda: any('NOT possible' in t for t in list(node.status_log)), 5.0)


def _approach_done(r):
    grip = r.node._grip_pos
    return r.wait(lambda: all(r.bus.pos[s] == grip[s] for s in aas.LIMB_IDS), 30.0)


def test_return_starts_from_the_measured_pose(rig_factory):
    """The servos stall short of the grip pose (contact): the return path is
    re-validated from the MEASURED pose and its first setpoint continues from there -
    no jump from the commanded pose; the deviation is reported."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    handle = _send_pick(r)
    assert handle.accepted
    assert _approach_done(r)
    stalled = r.node._grip_pos[1] - 120 if r.node._grip_pos[1] > HOME_D[1] else \
        r.node._grip_pos[1] + 120                  # 120 ticks short, back toward home
    n = len(r.node.audit)
    r.bus.pos[1] = stalled
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 40.0)
    assert res.result().result.success, res.result().result.message
    assert any('target not reached' in t for t in list(r.node.status_log))
    assert any('return_home: planned path re-validated from the measured pose' in t
               for t in list(r.node.status_log))
    after = [sp for k, sp in list(r.node.audit)[n:] if k == 'approve']
    first = after[0][1]
    step = abs(HOME_D[1] - stalled) / (aas.HOME_MOVE_MS / 1000.0 * aas.MOVE_STEP_HZ)
    assert abs(first - stalled) <= math.ceil(step) + 1, (first, stalled)
    assert r.bus.pos == HOME_D


def test_envelope_change_before_the_return_holds_and_reports(rig_factory):
    """The return path was approved at planning time; when the return starts, the
    CURRENT envelope blocks it (and every re-plan): the arm holds and reports (SR-17
    rule 13), no further arm setpoint is written."""
    from gripperx_arm.safety_filter import Verdict
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    node, flt = r.node, r.node._filter
    handle = _send_pick(r)
    assert handle.accepted
    assert _approach_done(r)
    blocked = object()                             # the envelope as it is now
    real_pose, real_path = flt.check_pose, flt.check_path
    seen = []

    def check_pose(q, envelope=None, *a, **kw):
        if envelope is blocked:
            seen.append('pose')
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_pose(q, envelope, *a, **kw)

    def check_path(qa, qb, envelope=None, *a, **kw):
        if envelope is blocked:
            seen.append('path')
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_path(qa, qb, envelope, *a, **kw)
    real_sp = flt.check_setpoints

    def check_setpoints(poses, envelope=None, *a, **kw):    # the planner's segment check
        if envelope is blocked:
            seen.append('path')
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_sp(poses, envelope, *a, **kw)
    flt.check_pose, flt.check_path = check_pose, check_path
    flt.check_setpoints = check_setpoints
    node._envelope = lambda: blocked
    n = len(node.audit)
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'no path home' in msg, msg
    k = [k for k, _ in list(node.audit)[n:]]
    assert 'approve' not in k and 'hold' in k, k
    assert 'path' in seen
    time.sleep(node._grace + 0.5)                  # and no return home is scheduled
    assert 'approve' not in [k for k, _ in list(node.audit)[n:]]


def test_gripper_target_outside_l2_is_refused(rig_factory):
    """Rule 12: gripper moves are not blocked by the filter, but L2 applies. The
    model's gripper limit is +-pi today (not recorded), so the test narrows it in the
    loaded model: open then lies outside L2."""
    r = rig_factory()
    g = r.node._cal.joints[5]
    q_open = r.node._q_of({**r.bus.pos, aas.GRIPPER_ID: r.node._grip_open})[g]
    r.node._filter.kin.limits[g] = (-abs(q_open) / 2, abs(q_open) / 2)
    n = len(r.bus.writes)
    r.cmd.publish(String(data='open_gripper'))
    assert r.wait(lambda: any('L2 joint limit' in t for t in list(r.node.status_log)), 3.0)
    assert len(r.bus.writes) == n


def test_gripper_target_is_evaluated_and_logged(rig_factory):
    r = rig_factory()
    r.cmd.publish(String(data='open_gripper'))
    assert r.wait(lambda: r.bus.pos[aas.GRIPPER_ID] == r.node._grip_open, 5.0)
    assert any('evaluated (not blocked' in t for t in list(r.node.status_log))


def test_startup_homing_without_filter_says_so(rig_factory):
    r = rig_factory(description=False, home_on_startup=True)
    assert r.bus.writes == [(sid, t) for sid, t in HOME_D.items()]   # only the torque sync


def test_return_from_an_invalid_measured_pose_holds(rig_factory, tmp_path):
    """The return path is validated from where the arm IS: pushed into the chassis
    during the grasp, the commanded grip pose still has a clear path home, the
    measured one has none -> hold and report, no return setpoint written."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME, **{
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'c.yaml')})
    node = r.node
    handle = _send_pick(r)
    assert handle.accepted
    assert _approach_done(r)
    bad = _ik(node, 0.30, -0.12, -0.12, 'front/elbow-')
    assert not node._filter.check_pose(bad).ok
    n = len(node.audit)
    r.bus.pos.update({s: t for s, t in node._ticks_of(bad).items() if s in aas.LIMB_IDS})
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'no path home' in msg, msg
    k = [k for k, _ in list(node.audit)[n:]]
    assert 'approve' not in k and 'hold' in k, k


def test_envelope_change_before_the_approach_holds(rig_factory):
    """The approach is re-validated against the CURRENT envelope when it starts (the
    gripper has opened already): blocked -> no arm setpoint, the arm holds, the
    refusal is reported, and the planned return home (also blocked) holds too."""
    from gripperx_arm.safety_filter import Verdict
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    node, flt = r.node, r.node._filter
    blocked = object()
    real_pose, real_path = flt.check_pose, flt.check_path

    def check_pose(q, envelope=None, *a, **kw):
        if envelope is blocked:
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_pose(q, envelope, *a, **kw)

    def check_path(qa, qb, envelope=None, *a, **kw):
        if envelope is blocked:
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_path(qa, qb, envelope, *a, **kw)
    flt.check_pose, flt.check_path = check_pose, check_path
    real_env = node._envelope
    handle = _send_pick(r)
    assert handle.accepted
    assert r.wait(lambda: 'gripper_unchecked' in r.kinds(), 10.0)   # planned, opening
    node._envelope = lambda: blocked
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'after motion started' in msg, msg
    assert r.wait(lambda: any('NOT possible' in t for t in list(node.status_log)), 10.0)
    assert 'approve' not in r.kinds() and 'hold' in r.kinds()
    node._envelope = real_env


# ------------------------------------------------------------------ SR-17 rule 11 (2026-10-08, option a)
def _only_a_detour(node):
    """Start and goal valid, the direct path through the chassis: only a detour."""
    a = node._q_of(dict(node._bus.pos))
    b = _ik(node, 0.30, 0.0, 0.05, 'front/elbow-')
    flt = node._filter
    assert flt.check_pose(a).ok and flt.check_pose(b).ok and not flt.check_path(a, b).ok
    return b


def _detour_rig(rig_factory, tmp_path, **kw):
    def start(node):
        return _ik(node, 0.10, 0.25, 0.0, 'front/elbow-')
    return rig_factory(start_q=start, **{
        'safety_filter.calibration_file': centred_calibration(tmp_path / 'c.yaml')}, **kw)


def _status(r, needle, timeout=5.0):
    return r.wait(lambda: any(needle in t for t in list(r.node.status_log)), timeout)


def test_rule11_switch_is_locked_by_default_and_logged(rig_factory):
    """The active yaml and the code default are both direct_only; the node says so at
    startup; an unknown value refuses to start."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())
    assert doc['arm_action_server']['ros__parameters']['planner']['automatic_paths'] == \
        'direct_only'
    r = rig_factory(drop=('planner.automatic_paths',))        # code default
    assert r.node._automatic_paths == 'direct_only'
    assert any('automatic_paths = direct_only' in t and 'DIRECT path only' in t
               for t in list(r.node.status_log))
    with pytest.raises(RuntimeError):
        aas.ArmActionServer(bus=MockBus(HOME_D), robot_description=URDF, mesh_resolver=_mesh,
                            parameter_overrides=active_params(
                                **{'planner.automatic_paths': 'sometimes'}))


def test_rule11_key_i_takes_the_detour(rig_factory, tmp_path):
    """(a) I commanded by the operator: the detour is allowed and driven. DETERMINISTIC:
    the test is about WHICH path is allowed, not whether the search fits the production
    wall-clock budget on a loaded machine (it failed at 5007 ms of 5000 ms under load,
    2026-10-09); max_checks still bounds the search, as in production."""
    r = _detour_rig(rig_factory, tmp_path, **DETERMINISTIC)
    goal = r.node._ticks_of(_only_a_detour(r.node))
    r.node._home_pos = dict(goal)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: all(abs(r.bus.pos[s] - goal[s]) <= 1 for s in aas.LIMB_IDS), 120.0), \
        list(r.node.status_log)[-4:]
    assert _status(r, 'go_home (detours allowed; soft start): path ')
    assert not any('go_home (detours allowed; soft start): path direct' in t
                   for t in list(r.node.status_log))


def test_rule11_automatic_return_home_holds_and_reports(rig_factory, tmp_path):
    """(c) the return home after an abort, same scenario: direct path only -> no arm
    setpoint, the arm holds, the reason names rule 11."""
    r = _detour_rig(rig_factory, tmp_path)
    r.node._home_pos = r.node._ticks_of(_only_a_detour(r.node))
    n = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: 'hold' in [k for k, _ in list(r.node.audit)[n:]], 60.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]
    assert _status(r, 'NOT possible')
    assert any('automatic: direct path only' in t and 'detours are not allowed' in t
               for t in list(r.node.status_log)), list(r.node.status_log)[-4:]


@pytest.mark.parametrize('armed,execute', [
    (True, True),          # the gateway's goal
    (False, True),         # execute true alone
    (True, False),         # armed alone
    (None, False),         # arming state unknown
])
def test_rule11_gateway_pick_refused_before_moving(rig_factory, tmp_path, armed, execute):
    """(b) a pick that may be the gateway's (fail-closed classification) whose approach
    needs a detour: refused before any write, the reason names rule 11."""
    r = _detour_rig(rig_factory, tmp_path, armed=armed, drop=('poses.grip',),
                    **{'poses.grip_rad': [0.0] * 6})
    goal_q = _only_a_detour(r.node)
    r.node._grip_pos = r.node._ticks_of(goal_q)
    n = len(r.bus.writes)
    if armed is None:
        time.sleep(1.0)            # nothing to wait for: no arming state is published
    res = _send_pick(r, execute=execute).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'NO_PATH' in msg, msg
    assert 'detours are not allowed' in msg and 'approach' in msg, msg
    assert len(r.bus.writes) == n


def test_rule11_operator_pick_plans_the_detour(rig_factory, tmp_path):
    """The operator's P (default goal, gateway disarmed) in the same scenario: the
    approach detour is planned and driven (kills 'every P is automatic'). DETERMINISTIC
    for the reason given in test_rule11_key_i_takes_the_detour."""
    r = _detour_rig(rig_factory, tmp_path, drop=('poses.grip',),
                    **{'poses.grip_rad': [0.0] * 6}, **DETERMINISTIC)
    goal_q = _only_a_detour(r.node)
    r.node._grip_pos = r.node._ticks_of(goal_q)
    handle = _send_pick(r)
    assert handle.accepted
    assert _status(r, 'trigger operator', 10.0)
    assert _status(r, 'pick_plastic/approach (detours allowed; soft start): path ', 120.0), \
        list(r.node.status_log)[-4:]
    assert r.wait(lambda: 'approve' in r.kinds(), 120.0), list(r.node.status_log)[-4:]
    handle.cancel_goal_async()
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)


@pytest.mark.parametrize('mode,ok', [('direct_only', False), ('detour', True)])
def test_rule11_replan_inside_an_operator_pick(rig_factory, mode, ok):
    """(ii) the re-plan from the measured pose is automatic even inside the operator's
    P: when the return starts, its direct segment is blocked (a detour exists).
    direct_only -> hold and report; detour -> the detour home. The parametrisation
    proves the outcome follows the switch."""
    from gripperx_arm.safety_filter import Verdict
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME, **{'planner.automatic_paths': mode})
    node, flt = r.node, r.node._filter
    home_q = node._q_of(HOME_D)
    handle = _send_pick(r)
    assert handle.accepted
    assert _approach_done(r)
    grip_q = node._q_of({**HOME_D, **node._grip_pos})
    arm = node._cal.joints[:5]
    real_sp = flt.check_setpoints

    def near(q, ref):
        return all(abs(q[k] - ref[k]) < 0.02 for k in arm)

    # the planner checks a segment as the setpoints the ramp writes for it
    def check_setpoints(poses, envelope=None, *a, **kw):
        if near(poses[0], grip_q) and near(poses[-1], home_q):
            return Verdict(False, 'body', 'test: direct grip -> home blocked')
        return real_sp(poses, envelope, *a, **kw)
    flt.check_setpoints = check_setpoints
    n = len(node.audit)
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert _status(r, 'return_home (re-plan) (')
    k = [k for k, _ in list(node.audit)[n:]]
    if ok:
        assert res.result().result.success, msg
        assert all(abs(r.bus.pos[s] - HOME_D[s]) <= 1 for s in aas.LIMB_IDS)
    else:
        assert not res.result().result.success and 'no path home' in msg, msg
        assert 'detours are not allowed' in msg, msg
        assert 'approve' not in k and 'hold' in k, k


# ------------------------------------------------------------------ audit round 2 (2026-10-08)
def test_refusal_after_motion_holds_before_the_return_home(rig_factory):
    """The hold of the refusal-after-motion branch is its OWN, written before the
    scheduled return home (whose grace is lengthened here so the two cannot be
    confused); kills 'Mhold_after_refusal'."""
    from gripperx_arm.safety_filter import Verdict
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME, **{'abort.home_grace_sec': 5.0})
    node, flt = r.node, r.node._filter
    blocked = object()
    real_pose, real_path = flt.check_pose, flt.check_path

    def check_pose(q, envelope=None, *a, **kw):
        if envelope is blocked:
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_pose(q, envelope, *a, **kw)

    def check_path(qa, qb, envelope=None, *a, **kw):
        if envelope is blocked:
            return Verdict(False, 'body', 'test: wheel envelope changed')
        return real_path(qa, qb, envelope, *a, **kw)
    flt.check_pose, flt.check_path = check_pose, check_path
    handle = _send_pick(r)
    assert handle.accepted
    assert r.wait(lambda: 'gripper_unchecked' in r.kinds(), 10.0)
    node._envelope = lambda: blocked
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    assert 'after motion started' in res.result().result.message
    with node._state_lock:
        pending = node._pending_home
    assert pending is not None and time.monotonic() < pending[0], 'grace already over'
    assert 'hold' in r.kinds(), r.kinds()
    assert 'approve' not in r.kinds()


def test_gripper_target_across_the_tick_wrap_is_refused(rig_factory, tmp_path):
    """O to a gripper target inside L2 whose register ramp would cross 4095 -> 0 (the
    servo would turn the long way): refused, nothing written; kills
    'Mgrip_wrap_ignored'. Under the real calibration (2026-10-08: gripper limits
    -18.6..+104.6 deg) no such target exists - the wrap lay inside L2 only with the
    +-pi placeholder limit - so a TEST calibration moves the gripper zero 100 ticks
    below the wrap and the gripper starts at that zero."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    c = doc['arm_joint_state_relay']['ros__parameters']
    c['zero_ticks'] = list(c['zero_ticks'][:5]) + [c['ticks_per_rev'] - 100]
    (tmp_path / 'grip_wrap.yaml').write_text(yaml.safe_dump(doc))
    z, sgn, k = c['zero_ticks'][5], c['signs'][5], c['ticks_per_rev'] / (2 * math.pi)
    ctr = c.get('range_center_rad', [0.0] * 6)[5]
    from gripperx_arm.kinematics import ArmKinematics
    kin = ArmKinematics(URDF or _urdf())
    lo, hi = kin.limits[kin.joint_names[5]]
    assert lo <= 0.0 <= hi, 'the gripper start (its zero) must lie inside L2'
    candidates = []
    for u in (-100, c['ticks_per_rev'] + 100):         # just across either wrap
        q = (u - z) / (sgn * k)
        if lo <= q <= hi and ctr - math.pi <= q < ctr + math.pi:
            candidates.append(int(u) % 4096)
    assert candidates, 'no gripper target across the wrap inside L2 and the window'

    def start(node):
        return {**node._q_of(HOME_D), 'arm_gripper_joint': 0.0}
    r = rig_factory(start_q=start, drop=('gripper.open_rad',), **{
        'gripper.open': candidates[0],
        'safety_filter.calibration_file': str(tmp_path / 'grip_wrap.yaml')})
    assert r.node._grip_open == candidates[0]
    assert r.bus.pos[aas.GRIPPER_ID] == z
    n = len(r.bus.writes)
    r.cmd.publish(String(data='open_gripper'))
    assert _status(r, 'tick wrap', 5.0), list(r.node.status_log)[-4:]
    time.sleep(0.5)
    assert len(r.bus.writes) == n


def test_unexpected_error_after_the_first_write_holds_without_return_home(rig_factory):
    """An exception that is not a MotionRefused, after the gripper opened: the arm
    holds and the error is reported; no automatic return home (SR-17 rule 13, user decision 2026-10-09)."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    node = r.node

    def boom(*a, **kw):
        raise RuntimeError('injected')
    node._revalidated_segments = boom
    handle = _send_pick(r)
    assert handle.accepted
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'Error after motion started' in msg, msg
    k = r.kinds()
    assert 'gripper_unchecked' in k and 'hold' in k[k.index('gripper_unchecked'):], k
    assert node._pending_home is None
    n = len(node.audit)
    time.sleep(node._grace + 1.0)
    assert [k for k, _ in list(node.audit)[n:]] == []
    assert node._motion_owner is None


# ------------------------------------------------------------------ audit 5762c7f
def _approve_groups(audit):
    """The 'approve' writes, split into runs (one per ramp) by the other writes."""
    groups, cur = [], []
    for k, sp in audit:
        if k == 'approve':
            cur.append(dict(sp))
        elif cur:
            groups.append(cur)
            cur = []
    return groups + ([cur] if cur else [])


def _validated_writes(node, path, method, direct_ms, monkeypatch, soft=True, **kw):
    """The setpoints the planner validated for `path`, as limb ticks: the planner's own
    steps (_planner with the motion's direct_ms; the first segment soft-started as
    Planner.executed validates it, user decision 2026-10-20), each segment's poses
    after its start - with the timed ramp's check-set thinning (monotone_cover)
    switched off, so every executed setpoint is listed, not only the checked
    subsequence."""
    with monkeypatch.context() as m:
        m.setattr(aas, 'monotone_cover', lambda poses, joints, step: poses)
        planner = node._planner(0.0, direct_ms=direct_ms, soft_start=soft, **kw)
        out = []
        for i, (a, b) in enumerate(zip(path, path[1:])):
            out += [{s: t[s] for s in aas.LIMB_IDS} for t in
                    map(node._ticks_of, planner.executed(a, b, method == 'direct',
                                                         first=i == 0)[1:])]
    return out


@pytest.mark.parametrize('motion', ['I', 'P'])
def test_timed_ramp_writes_exactly_what_the_planner_validated(rig_factory, motion,
                                                              monkeypatch):
    """The timed ramp (_ramp) writes, setpoint for setpoint, the steps the planner's
    exec_steps validated for the same segment: same step count (move time x
    MOVE_STEP_HZ), same whole ticks. Kills '_ramp at another step rate' and 'the
    planner validates a direct segment at detour speed while the ramp uses the
    sequence's move time' (audit of 5762c7f, mutants E2/E3)."""
    def start(node):
        return dict(zip(node._cal.joints, GRIP_NEAR_HOME['poses.grip_rad']))
    r = rig_factory(drop=('poses.grip',), start_q=start if motion == 'I' else None,
                    **GRIP_NEAR_HOME)
    node = r.node
    executed = []                     # (path, method, direct_ms) of every converted path
    real = node._path_segments

    def spy(path, method, direct_ms, here, **kw):
        executed.append(([dict(p) for p in path], method, direct_ms))
        return real(path, method, direct_ms, here, **kw)
    node._path_segments = spy
    n = len(node.audit)
    if motion == 'I':
        assert r.bus.pos != HOME_D
        r.cmd.publish(String(data='go_home'))
        assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), list(node.status_log)[-4:]
        ramps = executed                          # planned once, executed as planned
    else:
        res = _send_pick(r).get_result_async()
        assert r.wait(lambda: res.done(), 40.0)
        assert res.result().result.success, res.result().result.message
        ramps = executed[2:]                      # the re-validated approach and return
    groups = _approve_groups(list(node.audit)[n:])
    assert len(groups) == len(ramps) == (1 if motion == 'I' else 2), (len(groups), len(ramps))
    for (path, method, direct_ms), written in zip(ramps, groups):
        assert method == 'direct' and direct_ms in (aas.HOME_MOVE_MS, 4000), (method, direct_ms)
        rise = ({'rise_ground': node._ground_mode in ('operator', 'operator_go_home')}
                if motion == 'I' else {})
        expected = _validated_writes(node, path, method, direct_ms, monkeypatch, **rise)
        assert len(written) == len(expected), (len(written), len(expected))
        assert written == expected
        # soft-started (user decision 2026-10-20: I and both arm paths of P): more,
        # smaller first setpoints than the plain ramp of the same segment
        plain = _validated_writes(node, path, method, direct_ms, monkeypatch, soft=False)
        assert len(expected) > len(plain) and expected[-1] == plain[-1]


def test_j_in_the_middle_of_a_leader_tick_still_returns_home(rig_factory):
    """Root cause of the 1-in-15 failure of test_disable_via_j_returns_home (audit of
    5762c7f): J (enable False -> _leave, enable callback group) landed while a leader
    tick (its own group, concurrently) was between its state check and its read of the
    commanded pose. _leave had cleared that pose: the tick raised TypeError, the
    exception ended the executor's spin, every timer died with it - the pending return
    home included. The arm held and never returned home, without a word. Forced here:
    the tick is stopped right after its measured-pose check, J is handled, the tick
    resumes."""
    r = rig_factory(**ALLOW)
    _activate(r)
    _nudge(r, d=0.1)
    node = r.node
    real = node._filter.check_pose
    entered, resume, armed = threading.Event(), threading.Event(), [True]

    def gated(*a, **kw):
        v = real(*a, **kw)
        if armed[0] and sys._getframe(1).f_code.co_name.startswith('_leader_tick'):
            armed[0] = False
            entered.set()
            resume.wait(5.0)
        return v
    node._filter.check_pose = gated
    assert entered.wait(5.0), 'the leader tick never reached its measured-pose check'
    node._on_leader_enable(Bool(data=False))          # J, while the tick is held
    assert node._leader_state == aas.LEADER_OFF and node._pending_home is not None
    resume.set()
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), (dict(r.bus.pos),
                                                       list(node.status_log)[-4:])
    assert r.thread.is_alive(), 'the executor stopped spinning'
    # The tick must leave through its own "left meanwhile" check, not through the
    # last-resort wrapper: the wrapper keeps the executor alive and would mask a tick
    # that reads the commanded pose unlocked again (audit mutant W2).
    errors = [s for s in node.status_log if 'unexpected error' in s]
    assert errors == [], errors


def test_a_leader_tick_error_holds_and_keeps_the_executor_alive(rig_factory):
    """Any unforeseen exception in the leader tick: leader mode ends, the arm holds and
    the error is reported - no automatic return home (SR-17 rule 13, user decision 2026-10-09) - and the
    executor keeps running (the pending-home and joint-state timers live on)."""
    r = rig_factory(**ALLOW)
    _activate(r)
    _nudge(r)
    node = r.node

    def boom(*a, **kw):
        raise RuntimeError('injected')
    node._envelope = boom
    assert r.wait(lambda: node._leader_state == aas.LEADER_OFF, 3.0)
    assert _status(r, 'unexpected error in the leader tick')
    assert 'injected' in node.leader_text
    assert node._pending_home is None and node._motion_owner is None
    assert r.thread.is_alive()
    node._pending_home = (time.monotonic() - 1.0, 'test')   # timers still run
    assert r.wait(lambda: node._pending_home is None, 3.0)


def test_return_home_superseded_by_a_newer_claim_is_reported(rig_factory):
    """A return home that falls due (or is scheduled) while another motion holds the
    arm is not retried - that motion claimed the arm after the abort, and a newer
    command supersedes the return home - but it is reported on the status topic, never
    dropped silently (audit of 5762c7f: the refusal went to the logger only).
    The due case is INJECTED: _pending_home is set by hand while the arm is claimed.
    The node itself never gets there (_claim clears a pending return home), so this
    exercises a guard branch of _pending_home_tick, not a reachable sequence."""
    r = rig_factory()
    node = r.node
    n = len(node.audit)
    assert node._claim('command')
    try:
        with node._state_lock:                         # due while the arm is claimed
            node._pending_home = (time.monotonic() - 1.0, 'test due')
        assert _status(r, 'return home after abort (test due) NOT performed: superseded '
                          'by "command"'), list(node.status_log)[-3:]
        assert node._pending_home is None
        node._schedule_home('test scheduled')
        assert node._pending_home is None
        assert _status(r, 'return home after abort (test scheduled) NOT scheduled: '
                          'superseded by "command"'), list(node.status_log)[-3:]
    finally:
        node._release('command')
    time.sleep(node._grace + 0.5)
    assert list(node.audit)[n:] == [] and node._motion_owner is None


# ------------------------------------------------------------------ activation races
def _hold_activation(r):
    """J rising edge with the activation stopped inside _try_activate (at its torque-on
    step, after the refusal, pose and match checks). Returns the release event."""
    node = r.node
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: node._gateway_armed is False and node._leader_cal_verified is True
                  and node.count_publishers('/arm/leader/joint_states') == 1
                  and node.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - node._leader_rx < 0.5, 20.0), 'gate inputs missing'
    real = node._ensure_torque
    entered, resume = threading.Event(), threading.Event()

    def gated():
        if sys._getframe(1).f_code.co_name == '_try_activate':
            entered.set()
            resume.wait(5.0)
        return real()
    node._ensure_torque = gated
    r.rising_edge()
    assert entered.wait(5.0), 'the activation never reached its torque-on step'
    return resume


def _settled_off(r, needle):
    node = r.node
    assert _status(r, f'leader activation REFUSED: {needle}'), list(node.status_log)[-3:]
    time.sleep(0.5)                                     # several ticks
    assert node._leader_state == aas.LEADER_OFF and node._motion_owner is None
    assert needle in node.leader_text, node.leader_text
    assert r.thread.is_alive()


def test_enable_withdrawn_during_activation_is_refused(rig_factory):
    """Audit of a56adbb: J (or leader_arm_node withdrawing) during the tens of ms of
    _try_activate found leader mode still OFF and did nothing; the activation then
    completed and leader mode stayed ACTIVE with enable False."""
    r = rig_factory(**ALLOW)
    resume = _hold_activation(r)
    n = len(r.node.audit)
    r.node._on_leader_enable(Bool(data=False))          # J while activating
    resume.set()
    _settled_off(r, 'enable withdrawn during activation')
    assert list(r.node.audit)[n:] == []


def test_withdraw_and_new_edge_during_activation_activates_on_the_new_edge(rig_factory):
    """J off and on again during an activation: that activation is refused (a withdrawal
    came in between), the new rising edge activates afresh."""
    r = rig_factory(**ALLOW)
    resume = _hold_activation(r)
    r.node._on_leader_enable(Bool(data=False))
    r.node._on_leader_enable(Bool(data=True))
    resume.set()
    assert _status(r, 'leader activation REFUSED: enable withdrawn during activation')
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_ACTIVE, 3.0), r.node.leader_text


def test_space_during_activation_is_refused(rig_factory):
    """Audit of a56adbb: Space during _try_activate was lost - the arm was stationary
    (nothing to switch off), and the activation then cleared the stop flag."""
    r = rig_factory(**ALLOW)
    resume = _hold_activation(r)
    n = len(r.node.audit)
    r.node._on_stop(Empty())
    resume.set()
    _settled_off(r, 'Space during activation')
    assert list(r.node.audit)[n:] == []


def test_tick_leaves_when_enable_is_false(rig_factory):
    """Belt and braces: an ACTIVE leader mode with enable False (whatever path got it
    there) is ended by the next tick, through the normal leave."""
    r = rig_factory(**ALLOW)
    _activate(r)
    with r.node._state_lock:
        r.node._enable_last = False
    assert r.wait(lambda: r.node._leader_state == aas.LEADER_OFF, 2.0)
    assert 'enable withdrawn' in r.node.leader_text
    # through _leave (hold, then the return home): its HOLD report says so. Not
    # _pending_home - it is due after 0.5 s and may already be consumed under load.
    assert _status(r, 'HOLD: leader mode off: enable withdrawn'), list(r.node.status_log)[-3:]


# ------------------------------------------------------------------ bus errors
class _PortLost(OSError):
    pass


def _lose_port(bus, what=('read_pos', 'write_goal', 'torque')):
    def boom(*a, **kw):
        raise _PortLost('serial device disconnected')
    for name in what:
        setattr(bus, name, boom)


@pytest.mark.parametrize('path', ['tick', 'space', 'j', 'pending_home', 'command',
                                  'activation'])
def test_a_bus_error_never_ends_the_executor(rig_factory, path):
    """Audit of a56adbb: FeetechBus lets SerialException/OSError through, and _hold /
    _torque_off_all run in error handlers - a lost port raised out of the handler and
    ended spin(), Space included. Every path: the executor lives, the failure is
    reported, and a second Space still runs."""
    r = rig_factory(**ALLOW)
    node = r.node
    if path == 'activation':
        node._torque_on = False                      # the torque-on step must touch the bus
        r.calv.publish(Bool(data=True))
        r.leader_q = r.measured_q()
        assert r.wait(lambda: node._gateway_armed is False
                      and node._leader_cal_verified is True
                      and time.monotonic() - node._leader_rx < 0.5, 20.0)
        _lose_port(r.bus, ('torque',))
        r.rising_edge()
        assert _status(r, 'leader activation REFUSED: torque-on failed'), \
            list(node.status_log)[-3:]
        assert node._leader_state == aas.LEADER_OFF
    elif path in ('tick', 'space', 'j'):
        # Leader on the follower's pose: the tick reads, but writes nothing - so only
        # the path under test meets the lost port function.
        _activate(r)
        if path == 'tick':
            _lose_port(r.bus, ('read_pos',))
            assert _status(r, 'HOLD: leader mode off: unexpected error in the leader tick'), \
                list(node.status_log)[-3:]
            assert any('HOLD INCOMPLETE' in s for s in node.status_log), \
                list(node.status_log)[-3:]
            assert 'unexpected error' in node.leader_text, node.leader_text
        elif path == 'space':
            _lose_port(r.bus, ('torque',))
            r.stop.publish(Empty())
            assert _status(r, 'TORQUE-OFF WRITE FAILED'), list(node.status_log)[-3:]
            assert ('torque_off', {sid: 0 for sid in aas.IDS}) in list(node.audit)
        else:
            # The hold keeps every known goal (user decision 2026-10-13): one unknown
            # goal makes it write - into the lost port.
            with node._serial_lock:
                node._goal[aas.IDS[1]] = None
            _lose_port(r.bus, ('write_goal',))
            r.enable.publish(Bool(data=False))
            assert _status(r, 'HOLD: leader mode off: enable withdrawn'), \
                list(node.status_log)[-3:]
            assert any('HOLD INCOMPLETE' in s and 'enable withdrawn' in s
                       for s in node.status_log), list(node.status_log)[-3:]
        assert node._leader_state == aas.LEADER_OFF
    elif path == 'pending_home':
        _lose_port(r.bus)
        with node._state_lock:
            node._pending_home = (time.monotonic() - 1.0, 'test')
        assert _status(r, 'return home after abort NOT possible (unexpected error)'), \
            list(node.status_log)[-3:]
        assert node._motion_owner is None
    else:
        _lose_port(r.bus)
        r.cmd.publish(String(data='go_home'))
        assert _status(r, 'REFUSED: arm/command go_home: unexpected error'), \
            list(node.status_log)[-3:]
        assert node._motion_owner is None
    time.sleep(0.5)
    assert r.thread.is_alive(), 'the executor stopped spinning'
    k = len(node.status_log)
    node._report('probe')                            # the status path still works
    r.stop.publish(Empty())                          # and Space is still served
    assert r.wait(lambda: any('Space' in s or 'TORQUE OFF' in s
                              for s in list(node.status_log)[k:]), 3.0), \
        list(node.status_log)[-3:]
    assert r.thread.is_alive()


def test_torque_off_tries_every_servo_when_one_fails(rig_factory):
    r = rig_factory()
    real = r.bus.torque
    tried = []

    def flaky(sid, on):
        tried.append(sid)
        if sid == aas.IDS[1]:
            raise _PortLost('timeout')
        return real(sid, on)
    r.bus.torque = flaky
    r.node._torque_off_all('test')
    assert tried == list(aas.IDS)
    assert not any(v for sid, v in r.bus.torque_on.items() if sid != aas.IDS[1])
    assert f'TORQUE-OFF WRITE FAILED on servo(s) {{{aas.IDS[1]}:' in r.node.status_log[-1]


def test_spin_guarded_survives_a_callback_exception(rig_factory):
    """main()'s spin: an exception that still escapes a callback is logged and reported,
    spin resumes, nothing is written - the node keeps serving Space and its timers."""
    r = rig_factory()
    node = r.node
    r.ex.shutdown()                                  # take the node off the rig's executor
    r.thread.join(5.0)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    r.ex = ex                                        # closed by the fixture
    fired = []

    def bad():
        fired.append(1)
        raise RuntimeError('injected callback error')
    timer = node.create_timer(0.05, bad)
    n = len(node.audit)
    th = threading.Thread(target=aas.spin_guarded, args=(ex, node), daemon=True)
    th.start()
    assert _status(r, 'UNEXPECTED ERROR escaped a callback'), list(node.status_log)[-3:]
    assert r.wait(lambda: len(fired) >= 5, 3.0), 'spin did not resume after the error'
    node.destroy_timer(timer)
    time.sleep(0.3)
    assert th.is_alive() and list(node.audit)[n:] == []
    with node._state_lock:
        node._pending_home = (time.monotonic() - 1.0, 'test')
    assert r.wait(lambda: node._pending_home is None, 3.0), 'node timers stopped'
    assert r.wait(lambda: node._motion_owner is None, 10.0)
    ex.shutdown()
    th.join(5.0)
    assert not th.is_alive(), 'spin_guarded did not return on shutdown'


# ------------------------------------------------------------------ recovery from outside the limits
# User decision 2026-10-09 (option a): after a restart the arm rested with limb2 at
# +0.2209 rad, beyond its L2 limit; every plan, go_home included, was refused. I and the
# automatic return home now plan back inside; P and leader activation are refused.
RESTING_L2 = 0.2209


def _resting(gripper=None):
    def start(node):
        q = {n: 0.0 for n in node._cal.joints}           # home = calibration zero
        q.update({'arm_limb2_joint': RESTING_L2, 'arm_limb3_joint': 0.1})
        if gripper is not None:
            q['arm_gripper_joint'] = gripper(node)
        return q
    return start


def _assert_recovered(node, start_q, n):
    """Every arm setpoint written after audit index n: a joint outside only moves back
    toward its limit, inside it stays inside; the arm ends at home."""
    poses, here = [start_q], dict(start_q)
    for k, sp in list(node.audit)[n:]:
        if k == 'approve':
            here = node._q_of({**node._ticks_of(here), **sp})
            poses.append(here)
    assert len(poses) > 10
    for p, q in zip(poses, poses[1:]):
        for j, (lo, hi) in node._filter.kin.limits.items():
            if lo <= p[j] <= hi:
                assert lo <= q[j] <= hi, (j, p[j], q[j])
            else:
                assert (q[j] <= p[j]) if p[j] > hi else (q[j] >= p[j]), (j, p[j], q[j])


def _outside_text(r):
    return 'start outside joint limits (arm_limb2_joint +0.22'


@pytest.mark.parametrize('gripper_out', [False, True])
def test_recovery_go_home_from_outside_the_limits(rig_factory, gripper_out):
    """(1) I from limb2 +0.2209 rad: the direct path back, every written setpoint
    recovers, the operator is told why; a gripper outside its limit is held, then
    moved home (gripper moves stay unblocked)."""
    grip = (lambda node: node._filter.kin.limits['arm_gripper_joint'][0] - 0.05) \
        if gripper_out else None
    r = rig_factory(start_q=_resting(grip))
    start = r.measured_q()
    assert 'arm_limb2_joint' in r.node._filter.outside_limits(start)
    n = len(r.node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), list(r.node.status_log)[-4:]
    assert _status(r, _outside_text(r)) and _status(r, 'recovery path back inside')
    assert any('go_home (detours allowed; recovery from outside the joint limits; soft start): '
               'path direct' in t for t in list(r.node.status_log)), list(r.node.status_log)[-4:]
    _assert_recovered(r.node, start, n)
    assert all(aas.GRIPPER_ID not in sp for k, sp in list(r.node.audit)[n:] if k == 'approve')


def test_recovery_automatic_return_home_is_direct_only(rig_factory):
    """(5) The return home after an abort from outside: recovers on the direct path
    (rule 11 still holds - the mode says so)."""
    r = rig_factory(start_q=_resting())
    start = r.measured_q()
    n = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), list(r.node.status_log)[-4:]
    assert any('(automatic: direct path only; recovery from outside the joint limits; '
               'soft start): path direct' in t for t in list(r.node.status_log))
    _assert_recovered(r.node, start, n)


def test_recovery_pick_from_outside_is_refused(rig_factory):
    """(4) P from outside the limits: refused before any write, advising I."""
    r = rig_factory(start_q=_resting(), drop=('poses.grip',), **GRIP_NEAR_HOME)
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success, msg
    assert _outside_text(r) in msg and 'press I (go_home) first' in msg, msg
    assert len(r.bus.writes) == n


def test_recovery_leader_activation_from_outside_is_refused(rig_factory):
    """(6) Leader mode is not a recovery path: refused while the arm is outside,
    advising I; nothing written."""
    r = rig_factory(start_q=_resting(), **ALLOW)
    n = len(r.node.audit)
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    r.rising_edge()
    assert _status(r, 'leader activation REFUSED: measured pose outside joint limits '
                   '(arm_limb2_joint +0.22'), list(r.node.status_log)[-3:]
    assert 'press I (go_home) first' in r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF
    assert [k for k, _ in list(r.node.audit)[n:]] == []


# ------------------------------------------------------------------ audit r6 of 5acf87e
def _gate_ready(r):
    node = r.node
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: node._gateway_armed is False and node._leader_cal_verified is True
                  and node.count_publishers('/arm/leader/joint_states') == 1
                  and node.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - node._leader_rx < 0.5, 20.0), 'gate inputs missing'


def test_space_between_the_j_edge_and_the_tick_refuses(rig_factory):
    """A plain /arm/stop after the J rising edge, before the tick took the request
    (probe P1): leader mode must not activate."""
    r = rig_factory(**ALLOW)
    _gate_ready(r)
    node = r.node
    real = node._leader_tick_once
    parked, release, gate = threading.Event(), threading.Event(), [True]

    def gated():
        if gate[0]:
            gate[0] = False
            parked.set()
            release.wait(5.0)
        return real()
    node._leader_tick_once = gated
    assert parked.wait(3.0)
    node._on_leader_enable(Bool(data=True))     # J rising edge
    node._on_stop(Empty())                      # Space, arm stationary
    release.set()
    time.sleep(0.5)
    assert node._leader_state == aas.LEADER_OFF, node.leader_text


def test_space_between_taking_the_request_and_the_activation_refuses(rig_factory):
    """Space after the tick took the request, before _try_activate ran (probe P2): the
    counts are those of the J edge, so the activation is refused."""
    r = rig_factory(**ALLOW)
    _gate_ready(r)
    node = r.node
    real_try = node._try_activate

    def wrapped(*a):
        node._on_stop(Empty())
        return real_try(*a)
    node._try_activate = wrapped
    r.rising_edge()
    assert _status(r, 'leader activation REFUSED: Space during activation'), \
        list(node.status_log)[-3:]
    time.sleep(0.3)
    assert node._leader_state == aas.LEADER_OFF


def test_space_refusing_an_activation_turns_its_torque_off_again(rig_factory):
    """Limp after a Space, J torques on, Space during that activation (probe P4): the
    refused activation leaves the arm limp again; J afterwards activates (P5)."""
    r = rig_factory(**ALLOW)
    _activate(r)
    r.stop.publish(Empty())
    assert r.wait(lambda: not any(r.bus.torque_on.values()), 3.0)
    node = r.node
    real = node._ensure_torque
    hits = []

    def gated():
        out = real()
        if not hits:
            hits.append(1)
            node._on_stop(Empty())                  # Space right after the torque-on
        return out
    node._ensure_torque = gated
    r.rising_edge()
    assert _status(r, 'leader activation REFUSED: Space during activation')
    assert r.wait(lambda: not any(r.bus.torque_on.values()), 3.0), r.bus.torque_on
    assert not node._torque_on
    r.rising_edge()
    assert r.wait(lambda: node._leader_state == aas.LEADER_ACTIVE, 5.0), node.leader_text


def test_unacknowledged_torque_off_is_reported_as_failed(rig_factory):
    """FeetechBus.torque returns False on a communication failure (probe P3)."""
    r = rig_factory(**ALLOW)
    _activate(r)
    r.bus.torque = lambda sid, on: False
    r.stop.publish(Empty())
    assert _status(r, 'TORQUE OFF'), list(r.node.status_log)[-3:]
    last = [s for s in r.node.status_log if 'TORQUE OFF' in s][-1]
    assert 'TORQUE-OFF WRITE FAILED' in last and 'not acknowledged' in last, last


def test_unacknowledged_torque_on_is_not_marked_on(rig_factory):
    """Refused and never marked on; the acknowledged servos keep holding (nothing is
    switched off - audit r7: that would drop a held arm)."""
    r = rig_factory()
    node = r.node
    node._torque_off_all('test')
    real = r.bus.torque
    r.bus.torque = lambda sid, on: real(sid, on) if (not on or sid != aas.IDS[2]) else False
    with pytest.raises(aas.MotionRefused, match='not acknowledged'):
        node._ensure_torque()
    assert not node._torque_on
    assert all(r.bus.torque_on[sid] for sid in aas.IDS if sid != aas.IDS[2])


def test_respawn_with_an_unacknowledged_torque_on_keeps_the_held_arm():
    """Audit r7 probe R2: a respawn finds the servos torqued (holding the arm); one
    does not acknowledge the constructor's torque-on -> no servo is switched off."""
    global URDF
    URDF = URDF or _urdf()
    rclpy.init()
    try:
        bus = MockBus(HOME_D)
        bus.torque_on = {sid: True for sid in aas.IDS}
        real = bus.torque
        bus.torque = lambda sid, on: False if (on and sid == aas.IDS[1]) else real(sid, on)
        node = aas.ArmActionServer(bus=bus, robot_description=URDF, mesh_resolver=_mesh,
                                   parameter_overrides=active_params(()))
        try:
            assert all(bus.torque_on.values()), bus.torque_on
            assert not node._torque_on
            assert ('torque', aas.IDS[0], False) not in bus.events
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_give_up_mid_ramp_stops_writing_and_goes_limp(rig_factory):
    """Audit r7 (HIGH): spin_guarded gives up while go_home ramps; main() then destroys
    the node (no more /arm/stop). From the give-up on no goal is written, torque off."""
    r = rig_factory(start_q=_resting())
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
            out.append(len(r.bus.writes))
    th = threading.Thread(target=main_like, daemon=True)
    th.start()
    start = dict(r.bus.pos)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos != start, 15.0), list(node.status_log)[-3:]

    def bad():
        raise RuntimeError('persistent')
    node.create_timer(0.02, bad)
    assert r.wait(lambda: out, 15.0)
    time.sleep(1.0)
    assert len(r.bus.writes) == out[0], 'goal writes after the give-up'
    assert not any(r.bus.torque_on.values()) and r.bus.pos != HOME_D
    assert any('node shutting down while the arm was moving' in t
               for t in list(node.status_log))


def test_unacknowledged_goal_write_refuses_and_hold_reports_it(rig_factory):
    """A goal write some servo does not acknowledge: the ramp stops (refused), the hold
    names the servo, the torque-on sync refuses."""
    r = rig_factory()
    node = r.node
    real = r.bus.write_goal
    bad = aas.IDS[1]
    r.bus.write_goal = lambda sid, t: False if sid == bad else real(sid, t)
    here = dict(r.bus.pos)
    goal = {**here, bad: here[bad] + 20}
    assert node._claim('command')            # a ramp runs only inside a claimed motion
    with pytest.raises(aas.BusWriteFailed) as exc:
        node._ramp([({s: here[s] for s in aas.LIMB_IDS}, {s: goal[s] for s in aas.LIMB_IDS},
                     0.2)], 'test', 'approve')
    assert exc.value.failed == [bad]
    node._release('command')
    node._hold('test')
    assert f'HOLD INCOMPLETE' in node.status_log[-1] and f'[{bad}]' in node.status_log[-1], \
        node.status_log[-1]
    node._torque_off_all('test')
    with pytest.raises(aas.MotionRefused, match='torque-on refused'):
        node._ensure_torque()
    assert not node._torque_on


def test_space_turns_the_torque_off_before_publishing_the_leader_state(rig_factory):
    """A failing leader-state publish (e.g. at shutdown) must not cost the torque-off."""
    r = rig_factory(**ALLOW)
    _activate(r)
    node = r.node

    def boom(_msg):
        raise RuntimeError('publish failed')
    node._leader_state_pub.publish = boom
    with pytest.raises(RuntimeError):
        node._on_stop(Empty())
    assert not any(r.bus.torque_on.values()) and not node._torque_on


def test_a_failing_status_publish_does_not_raise(rig_factory):
    r = rig_factory()
    node = r.node

    def boom(_msg):
        raise RuntimeError('publish failed')
    node._status_pub.publish = boom
    node._report('probe text')
    assert node.status_log[-1] == 'probe text'


def test_spin_guarded_gives_up_on_a_persistent_fault(rig_factory):
    """More than SPIN_ESCAPE_LIMIT escapes within the window: re-raised (the launch
    respawn takes over), reported once as persistent."""
    r = rig_factory()
    node = r.node
    r.ex.shutdown()
    r.thread.join(5.0)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    r.ex = ex

    def bad():
        raise RuntimeError('persistent')
    timer = node.create_timer(0.005, bad)
    out = []

    def run():
        try:
            aas.spin_guarded(ex, node)
            out.append('returned')
        except RuntimeError as exc:
            out.append(exc)
    th = threading.Thread(target=run, daemon=True)
    t0 = time.monotonic()
    th.start()
    th.join(aas.SPIN_ESCAPE_WINDOW_S + 5.0)
    node.destroy_timer(timer)
    assert out and isinstance(out[0], RuntimeError), out
    assert time.monotonic() - t0 >= aas.SPIN_ESCAPE_LIMIT * aas.SPIN_RESUME_BACKOFF_S
    assert any('persistent fault' in s for s in node.status_log)


# ------------------------------------------------------------------ self-contact recovery (option C)
# User decision 2026-10-09: the operator's I separates arm links in self-contact; the
# automatic return home holds and reports; P and leader activation stay refused.
REST_TICKS = [2089, 834, 3071, 1463, 2116, 2651]          # robot, 2026-10-09


def _at_limb3_stop():
    def start(node):
        from gripperx_geometry import constants as C
        q = node._q_of(dict(zip(aas.IDS, REST_TICKS)))
        q['arm_limb3_joint'] = C.ARM_JOINT_STOP_LOWER[2]
        return q
    return start


CONTACT = 'start pose in self-contact (arm_limb1_link vs arm_limb3_link, clearance -0.'


def test_self_contact_operator_i_separates_and_goes_home(rig_factory):
    """DETERMINISTIC: the separating direct check takes ~2.3 s on the dev laptop and
    ran out of the 5 s production budget at load 16 (2026-10-09); this test is about
    the rule, not the budget (Pi timing TO-VERIFY)."""
    r = rig_factory(start_q=_at_limb3_stop(), **DETERMINISTIC)
    n = len(r.node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 60.0), list(r.node.status_log)[-4:]
    assert _status(r, CONTACT) and _status(r, 'recovery separates the pair')
    assert any('separating a self-contact; soft start): path direct' in t
               for t in list(r.node.status_log)), list(r.node.status_log)[-4:]
    assert 'approve' in [k for k, _ in list(r.node.audit)[n:]]


def test_self_contact_automatic_return_home_holds_and_reports(rig_factory):
    r = rig_factory(start_q=_at_limb3_stop())
    n = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: 'hold' in [k for k, _ in list(r.node.audit)[n:]], 30.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]
    assert _status(r, CONTACT) and _status(r, 'press I to separate')


def test_self_contact_pick_and_leader_are_refused(rig_factory):
    r = rig_factory(start_q=_at_limb3_stop(), drop=('poses.grip',), **ALLOW,
                    **GRIP_NEAR_HOME)
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 30.0)
    msg = res.result().result.message
    assert not res.result().result.success and CONTACT in msg and 'press I' in msg, msg
    assert len(r.bus.writes) == n
    r.calv.publish(Bool(data=True))
    r.leader_q = r.measured_q()
    r.rising_edge()
    assert _status(r, 'measured pose in self-contact (arm_limb1_link vs arm_limb3_link'), \
        list(r.node.status_log)[-3:]
    assert r.node._leader_state == aas.LEADER_OFF and len(r.bus.writes) == n


# ------------------------------------------------------------------ P's return with joint-limit recovery
# User decision 2026-10-09 (option a): P's return home recovers from a measured pose
# outside the joint limits (load lag at the grip); self-contact is not separated there.
def _lagging_grip_rig(rig_factory, **kw):
    """Audit r7 probe R4: the grip pose 0.002 rad inside the limb2 limit, and the
    limb2 servo stops 3 ticks past it (sign -1: +q) under load."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME, **kw)
    node = r.node
    hi2 = node._filter.kin.limits['arm_limb2_joint'][1]
    g = {n: 0.0 for n in node._cal.joints}
    g.update({'arm_limb1_joint': 0.3, 'arm_limb5_joint': 0.3, 'arm_limb2_joint': hi2 - 0.002,
              'arm_limb3_joint': 0.1})
    node._grip_pos = node._ticks_of(g)
    real = r.bus.write_goal

    def lag(sid, t):
        ok = real(sid, t)
        if sid == 2 and r.bus.pos[2] == node._grip_pos[2]:
            r.bus.pos[2] = t - 3
        return ok
    r.bus.write_goal = lag
    return r


def test_pick_return_recovers_from_a_lag_past_the_limit(rig_factory):
    r = _lagging_grip_rig(rig_factory)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    assert res.result().result.success, res.result().result.message
    assert r.bus.pos == HOME_D
    assert _status(r, 'P return: start outside joint limits (arm_limb2_joint +0.16')
    assert _status(r, 'recovery path back inside (pick_plastic/return_home)')


def test_gateway_pick_return_from_outside_is_direct_only(rig_factory):
    """A gateway-triggered P (execute true): when the re-validation fails, the re-plan
    from outside the limits recovers on the DIRECT path only (rule 11)."""
    r = _lagging_grip_rig(rig_factory)
    node = r.node
    real = node._planner

    def planner(budget_s, *a, **kw):
        p = real(budget_s, *a, **kw)
        if budget_s == 0.0:                       # the re-validation's check planner
            p.check_segment = lambda *_a, **_k: 'test: re-validation fails'
        return p
    node._planner = planner
    res = _send_pick(r, execute=True).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    assert res.result().result.success, res.result().result.message
    assert _status(r, 'pick_plastic/return_home (re-plan) (automatic: direct path only; '
                      'recovery from outside the joint limits; soft start): path direct')


def test_pick_return_from_self_contact_holds_and_reports(rig_factory):
    """P's return never separates a self-contact (option C stays the operator's I):
    pushed into limb1/limb3 contact during the grasp -> hold, 'press I to separate'."""
    r = rig_factory(drop=('poses.grip',), **GRIP_NEAR_HOME)
    node = r.node
    handle = _send_pick(r)
    assert handle.accepted
    assert _approach_done(r)
    n = len(node.audit)
    r.bus.pos.update({s: t for s, t in node._ticks_of(_at_limb3_stop()(node)).items()
                      if s in aas.LIMB_IDS})
    res = handle.get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert not res.result().result.success and 'press I to separate' in msg, msg
    k = [k for k, _ in list(node.audit)[n:]]
    assert 'approve' not in k and 'hold' in k, k



# ------------------------------------------------------------------ follow_debug (2026-10-16)
def _debug_sub(r):
    from gripperx_arm_msgs.msg import LeaderFollowDebug
    got = []
    qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    sub = r.tester.create_subscription(LeaderFollowDebug, '/arm/leader/follow_debug',
                                       got.append, qos)
    return got, sub


def test_follow_debug_one_message_per_active_tick(rig_factory):
    """Published only while leader mode is active; per tick the written setpoint, the
    clamped target, the positions the tick read, its duration and period."""
    r = rig_factory(**ALLOW)
    got, sub = _debug_sub(r)
    time.sleep(0.5)
    assert got == []                                 # leader mode off: nothing
    _activate(r)
    q = dict(r.leader_q)
    q['arm_limb1_joint'] += 0.3
    r.leader_q = q
    assert r.wait(lambda: any(m.outcome == 'written' for m in got), 5.0)
    time.sleep(0.5)
    arm = r.node._cal.joints                         # the gripper too since 2026-11-13
    written = [m for m in got if m.outcome == 'written']
    m = written[-1]
    assert list(m.joint_names) == list(arm)
    assert len(m.setpoint) == len(m.target) == len(m.measured) == 6
    assert m.target[0] == pytest.approx(q['arm_limb1_joint'], abs=1e-6)
    assert 0.0 < m.tick_duration_s < 1.0
    period = 1.0 / r.node._p('leader.rate_hz')
    assert sorted(x.period_s for x in got[1:])[len(got) // 2] == pytest.approx(period, rel=0.5)
    # the setpoint steps by at most the velocity limit per tick
    vmax = r.node._p('leader.max_joint_velocity_rad_s') / r.node._p('leader.rate_hz')
    sp = [x.setpoint[0] for x in got]
    assert max(abs(b - a) for a, b in zip(sp, sp[1:])) <= vmax + 2 * math.pi / 4096 + 1e-9
    assert {x.outcome for x in got} <= {'written', 'unchanged', 'holding', 'rejected',
                                        'follow_step', 'activation', 'other'}
    # the tick that activated leader mode is the first one published, and marked
    assert got[0].outcome == 'activation'
    assert [x for x in got[1:] if x.outcome == 'activation'] == []
    # a 30 Hz subscription still taking messages while the rig shuts down leaves
    # 'Destroyable' futures behind: drop it first
    r.leader_q = None
    r.tester.destroy_subscription(sub)


def test_follow_debug_phase_times_add_up_to_the_tick(rig_factory):
    """2026-10-16: where a tick's time goes. A written tick carries read, envelope,
    target, check and write times and the poses the check checked; phases plus other_s
    are the tick duration."""
    r = rig_factory(**ALLOW)
    got, sub = _debug_sub(r)
    _activate(r)
    q = dict(r.leader_q)
    q['arm_limb1_joint'] += 0.3
    r.leader_q = q
    assert r.wait(lambda: sum(m.outcome == 'written' for m in got) >= 3, 5.0)
    r.leader_q = None
    r.tester.destroy_subscription(sub)
    phases = ('refusal', 'read', 'envelope', 'replan', 'target', 'check', 'write')
    assert tuple(aas.DEBUG_PHASES) == phases
    for m in [x for x in got if x.outcome == 'written']:
        for p in ('refusal', 'read', 'envelope', 'target', 'check', 'write'):
            assert getattr(m, f'{p}_s') > 0.0, (p, m)
        assert m.replan_s == 0.0                    # not held: no re-plan this tick
        # measured -> written, refined: at least both ends
        assert m.check_poses >= 2
        total = sum(getattr(m, f'{p}_s') for p in phases) + m.other_s
        assert total == pytest.approx(m.tick_duration_s, abs=1e-9)
        assert m.other_s >= -1e-9
    act = [x for x in got if x.outcome == 'activation']
    assert act and act[0].check_s == 0.0 and act[0].other_s > 0.0


def test_follow_debug_failure_does_not_end_leader_mode(rig_factory):
    """A diagnostics exception never reaches the tick's handler (which ends leader mode)."""
    r = rig_factory(**ALLOW)
    _activate(r)
    calls = []

    class Broken:
        def publish(self, msg):
            calls.append(msg)
            raise RuntimeError('diagnostics broken')
    r.node._follow_debug_pub = Broken()
    n = len(r.node.audit)
    q = dict(r.leader_q)
    q['arm_limb1_joint'] += 0.3
    r.leader_q = q
    assert r.wait(lambda: len(calls) > 10, 5.0)
    time.sleep(0.3)
    assert r.node._leader_state == aas.LEADER_ACTIVE, r.node.leader_text
    assert 'approve' in r.kinds()[n:]                # the arm kept following
    assert 'unexpected error' not in r.node.leader_text


def test_follow_debug_disabled_creates_no_publisher(rig_factory):
    r = rig_factory(**ALLOW, **{'leader.follow_debug_enabled': False})
    assert r.node._follow_debug_pub is None
    assert r.node.count_publishers('/arm/leader/follow_debug') == 0
    _activate(r)
    assert r.node._dbg is None
