"""Aimed P (FR-16 stage 1) on arm_action_server against a MOCK servo bus and a FAKE TF
(transforms written straight into the node's TF buffer). No hardware, no motion.
Same isolation as test_arm_node (discovery off); CMakeLists sets the domain.

Every refusal must leave the bus untouched; a success must put the TCP at the target
(x, y) at P's grip height with the gripper vertical; a goal without `aimed` must stay
today's fixed P.
"""
import os

os.environ.setdefault('ROS_DOMAIN_ID', '193')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ.pop('ROS_STATIC_PEERS', None)

import math
import threading
import time

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

from gripperx_arm import arm_action_server as aas
from gripperx_arm.planner import PlanResult
from gripperx_arm_msgs.action import PickPlastic
from test_arm_node import HOME_D, PKG, Rig, _send_pick  # noqa: F401 - the shared rig

_SRC = PKG.parent

ALLOW = {'safety_filter.allow_unverified_calibration': True}


class FakeTf:
    """A live localisation on the WIRE (audit AP1 L8): /tf with map -> odom (rotated,
    its stamp `skew` s behind) and odom -> base_link at `pose` = (x, y, yaw), plus
    /hw/steer_states, all published every 20 ms by the tester node with the node's
    clock, `age` s old (negative: future-dated)."""

    MAP_ODOM = (0.5, -0.3, 0.4)                  # x, y, yaw of odom in map

    def __init__(self, rig, pose=(1.0, 2.0, 0.3), age=0.0, period=0.02, skew=0.2):
        from std_msgs.msg import Float64MultiArray
        from tf2_msgs.msg import TFMessage
        self.rig, self.node = rig, rig.node
        self.pose, self.age, self.period, self.skew = pose, age, period, skew
        self.drift = (0.0, 0.0, 0.0)            # added per second (a moving base)
        self.steer = [0.01, 0.01, 0.01, 0.01]   # /hw/steer_states (None = not published)
        self.live = True
        self._run = True
        self.t0 = time.monotonic()
        self.tf_pub = rig.tester.create_publisher(TFMessage, '/tf', 100)
        self.steer_pub = rig.tester.create_publisher(Float64MultiArray, '/hw/steer_states', 10)
        self._tfmsg, self._steermsg = TFMessage, Float64MultiArray
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        assert rig.wait(lambda: self._known(), 20.0), 'TF did not arrive over /tf'

    def _known(self):
        try:
            return self.node._tf_buffer.can_transform('map', 'base_link',
                                                      aas.rclpy.time.Time())
        except Exception:
            return False

    @staticmethod
    def _tf(parent, child, x, y, yaw, stamp):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.header.stamp = stamp
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return t

    def current(self):
        dt = time.monotonic() - self.t0
        return tuple(p + d * dt for p, d in zip(self.pose, self.drift))

    def _loop(self):
        D = aas.rclpy.duration.Duration
        while self._run:
            if self.live:
                now = self.node.get_clock().now()
                st = (now - D(seconds=self.age)) if self.age >= 0 else (now + D(seconds=-self.age))
                x, y, yaw = self.current()
                m = self._tfmsg()
                m.transforms = [self._tf('map', 'odom', *self.MAP_ODOM,
                                         (st - D(seconds=self.skew)).to_msg()),
                                self._tf('odom', 'base_link', x, y, yaw, st.to_msg())]
                self.tf_pub.publish(m)
                if self.steer is not None:
                    self.steer_pub.publish(self._steermsg(data=[float(v) for v in self.steer]))
            time.sleep(self.period)

    def to_map(self, bx, by):
        """base_link (bx, by) -> map, at the fake pose."""
        x, y, yaw = self.current()
        ox, oy, oyaw = self.MAP_ODOM
        px = x + math.cos(yaw) * bx - math.sin(yaw) * by          # in odom
        py = y + math.sin(yaw) * bx + math.cos(yaw) * by
        return (ox + math.cos(oyaw) * px - math.sin(oyaw) * py,
                oy + math.sin(oyaw) * px + math.cos(oyaw) * py)

    def stop(self):
        self._run = False
        self.thread.join(1.0)


@pytest.fixture
def rig():
    import rclpy.duration  # noqa: F401 - FakeTf uses aas.rclpy.duration
    import rclpy.time  # noqa: F401
    rigs, tfs = [], []

    def make(tf=True, **kw):
        r = Rig(**{**ALLOW, **kw})
        rigs.append(r)
        if tf:
            r.tf = FakeTf(r)
            tfs.append(r.tf)
        return r
    yield make
    for t in tfs:
        t.stop()
    for r in rigs:
        r.close()


def _aimed(r, x, y, frame='map', execute=False, preview=False, epoch=None, shift=None,
           handle=None):
    """An aimed goal like the gateway's (map) or the operator tool's (base_link).
    shift = ((px, py), bound): the confirmed pick's preview check (SR-16 E11 item 4).
    handle: a list that receives the goal handle (to cancel it mid-pick)."""
    client = r.pick_client
    assert client.wait_for_server(timeout_sec=60.0)
    if r._armed_sent:
        assert r.wait(lambda: r.node._gateway_armed is not None, 20.0), 'no arming state'
    if epoch is None:
        assert r.wait(lambda: r.epoch == r.node._epoch_now(), 20.0), 'no stop epoch'
        epoch = r.epoch
    g = PickPlastic.Goal(execute=execute, stop_epoch=epoch, aimed=True, preview=preview)
    g.target.header.frame_id = frame
    g.target.header.stamp = r.node.get_clock().now().to_msg()
    g.target.point.x, g.target.point.y, g.target.point.z = float(x), float(y), 0.37
    if shift is not None:
        g.check_target_shift = True
        g.preview_target_in_base.x, g.preview_target_in_base.y = map(float, shift[0])
        g.max_target_shift_m = float(shift[1])
    fut = client.send_goal_async(g)
    assert r.wait(lambda: fut.done(), 20.0)
    h = fut.result()
    assert h.accepted
    if handle is not None:
        handle.append(h)
    res = h.get_result_async()
    assert r.wait(lambda: res.done(), 120.0)
    return res.result().result


def _good_target(r):
    """A target in the allowed region whose candidates pass even with the wheels
    swept over their stops - so it passes whatever the steering readings are."""
    s = r.node._aim_solver()
    for x in np.arange(0.40, 0.31, -0.01):
        for y in (0.0, 0.03, -0.03, 0.06):
            if s.solve(float(x), y, s.flt.envelope_swept, limit=1).ok:
                return float(x), float(y)
    pytest.skip('no target passes with the wheels swept')


def _grasp_setpoint(r):
    """The arm setpoint the approach ended on: the last 'approve' before the close."""
    audit = list(r.node.audit)
    close = [i for i, (k, sp) in enumerate(audit)
             if k == 'gripper_unchecked' and sp.get(aas.GRIPPER_ID) == r.node._grip_close]
    assert close, 'no grip_close'
    return [sp for k, sp in audit[:close[0]] if k == 'approve'][-1]


def test_aimed_p_end_to_end_grasps_at_the_target(rig):
    r = rig()
    bx, by = _good_target(r)
    mx, my = r.tf.to_map(bx, by)
    res = _aimed(r, mx, my, frame='map')
    assert res.success and res.reason == '', res.message
    tb = res.target_in_base
    assert tb.header.frame_id == 'base_link'
    assert (tb.point.x, tb.point.y) == pytest.approx((bx, by), abs=1e-6)
    sp = _grasp_setpoint(r)
    q = r.node._q_of({**HOME_D, **sp})
    t = r.node._filter.kin.tcp(q)
    s = r.node._aim_solver()
    assert np.linalg.norm(t[:3, 3] - [bx, by, s.z]) < 0.003      # one tick ~ 0.5 mm
    assert t[2, 2] == pytest.approx(-1.0, abs=0.01)               # vertical
    assert any('base-still check before the first motion' in m for m in r.node.status_log)
    assert any('wrist roll' in m for m in r.node.status_log)       # the chosen roll is logged
    assert r.wait(lambda: r.bus.pos == HOME_D, 20.0)


def test_operator_target_in_base_link_needs_no_map(rig):
    r = rig()
    bx, by = _good_target(r)
    res = _aimed(r, bx, by, frame='base_link')
    assert res.success, res.message


def _refused(r, reason, *goal):
    n = len(r.bus.events)
    res = _aimed(r, *goal)
    assert not res.success and res.reason == reason, (res.reason, res.message)
    assert len(r.bus.events) == n, 'a refusal wrote to the bus'
    return res


def test_out_of_reach_writes_nothing(rig):
    r = rig()
    _refused(r, 'OUT_OF_REACH', 0.80, 0.0, 'base_link')
    _refused(r, 'OUT_OF_REACH', 0.17, 0.0, 'base_link')         # over the chassis


def test_envelope_rejected_writes_nothing(rig):
    """Wheels measured turned 1 rad outward: a region target whose candidates all hit
    a wheel at that steering."""
    r = rig()
    s = r.node._aim_solver()
    r.tf.steer = [sg * 1.0 for sg in r.node._steer_sign]
    assert r.wait(lambda: all(v is not None and abs(abs(v) - 1.0) < 1e-6
                              for v in r.node._aim_steer().values())
                  and len(r.node._aim_steer()) == 4, 5.0), r.node._aim_steer()
    env = r.node._envelope()
    reg = s.derive_region(0.02)
    pts = [(float(reg.xs[i]), float(reg.ys[j])) for i, j in zip(*np.nonzero(reg.mask))]
    bad = [p for p in pts if not s.candidates(*p, env, limit=1).ok]
    if not bad:
        pytest.skip('no region point is rejected with the wheels at 1 rad')
    _refused(r, 'ENVELOPE_REJECTED', bad[0][0], bad[0][1], 'base_link')


def test_invalid_target_writes_nothing(rig):
    r = rig()
    _refused(r, 'INVALID_TARGET', 0.36, 0.0, '')
    _refused(r, 'INVALID_TARGET', float('nan'), 0.0, 'base_link')
    _refused(r, 'INVALID_TARGET', 0.36, 0.0, 'arm_tcp_link')     # AP1 L3: map/odom/base_link


def test_stale_missing_and_future_tf_write_nothing(rig):
    r = rig()
    bx, by = _good_target(r)
    r.tf.age = 5.0
    # the buffer keeps the fresh transforms written before; the newest one must age
    # past max_tf_age_sec first
    time.sleep(r.node._aim_cfg['max_tf_age_sec'] + 0.3)
    _refused(r, 'STALE_TF', *r.tf.to_map(bx, by), 'map')
    r.tf.age = -1.0                                               # future-dated
    time.sleep(0.1)
    _refused(r, 'STALE_TF', *r.tf.to_map(bx, by), 'map')


def test_missing_tf_writes_nothing(rig):
    r = rig(tf=False)
    _refused(r, 'STALE_TF', 0.36, 0.0, 'map')


def test_missing_steering_at_the_start_is_refused(rig):
    """AP1 L2: wheel_envelope measured and no steering reading - refused."""
    r = rig()
    bx, by = _good_target(r)
    r.tf.steer = None
    time.sleep(r.node._p('safety_filter.steer_state_timeout_sec') + 0.2)
    res = _refused(r, 'STEERING_UNKNOWN', bx, by, 'base_link')
    assert 'no steering reading' in res.message


def test_steering_change_during_planning_is_base_moving(rig, monkeypatch):
    r = rig()
    bx, by = _good_target(r)
    real = aas.ArmActionServer._aim_approach

    def turn_then_plan(self, aim, g_open, automatic):
        r.tf.steer = [0.3, 0.3, 0.3, 0.3]           # > steer_motion_margin_rad (0.0873)
        time.sleep(0.3)
        return real(self, aim, g_open, automatic)
    monkeypatch.setattr(aas.ArmActionServer, '_aim_approach', turn_then_plan)
    res = _refused(r, 'BASE_MOVING', bx, by, 'base_link')
    assert 'steering change' in res.message


def test_frozen_still_frame_is_stale(rig):
    """A transform that stops being republished cannot show the base at rest."""
    r = rig()
    bx, by = _good_target(r)
    r.node._aim_cfg['max_tf_age_sec'] = 30.0      # the start lookup still passes
    r.tf.live = False
    time.sleep(0.2)
    _refused(r, 'STALE_TF', bx, by, 'base_link')


def test_moving_base_writes_nothing(rig):
    r = rig()
    bx, by = _good_target(r)
    r.tf.drift = (0.02, 0.0, 0.0)                 # 2 cm/s
    _refused(r, 'BASE_MOVING', bx, by, 'base_link')


def test_no_path_writes_nothing(rig, monkeypatch):
    """AP1 M4: from a LIMP arm - not even the torque-on sync may be written. An
    automatic aimed P needs a set region margin (audit N7): 0.0 here, a test value."""
    r = rig(**{'aimed.region_margin_m': 0.0})
    bx, by = _good_target(r)
    with r.node._serial_lock:
        for sid in aas.IDS:
            r.bus.torque(sid, False)
        r.node._torque_on = False
    monkeypatch.setattr(aas.Planner, 'plan',
                        lambda self, qa, qb, env=None: PlanResult(False, reason='forced'))
    _refused(r, 'NO_PATH', bx, by, 'base_link', True)             # automatic: direct only


def test_preview_moves_nothing(rig):
    r = rig()
    bx, by = _good_target(r)
    n = len(r.bus.events)
    res = _aimed(r, bx, by, 'base_link', True, True)
    assert res.success and res.reason == 'PREVIEW_OK', res.message
    assert len(r.bus.events) == n
    res = _aimed(r, 0.8, 0.0, 'base_link', True, True)
    assert res.reason == 'OUT_OF_REACH' and len(r.bus.events) == n


def test_space_before_an_aimed_p_refuses_it(rig):
    """A goal carrying an epoch from before a Space is refused - aimed or not."""
    r = rig()
    bx, by = _good_target(r)
    assert r.wait(lambda: r.epoch == r.node._epoch_now(), 20.0)
    old = r.epoch
    r.node._on_stop(None)
    n = len(r.bus.events)
    g = PickPlastic.Goal(execute=False, stop_epoch=old, aimed=True)
    g.target.header.frame_id = 'base_link'
    g.target.point.x, g.target.point.y = bx, by
    fut = r.pick_client.send_goal_async(g)
    assert r.wait(lambda: fut.done(), 20.0)
    assert not fut.result().accepted
    assert len(r.bus.events) == n


def test_fixed_p_is_unchanged(rig):
    """Without `aimed` the target is ignored: P approaches the taught grip pose."""
    r = rig(tf=False, drop=('poses.grip',),
            **{'poses.grip_rad': [0.3, 0.0, 0.0, 0.0, 0.3, 0.0]})
    h = _send_pick(r)
    res = h.get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    out = res.result().result
    assert out.success and out.reason == '' and out.target_in_base.header.frame_id == ''
    sp = _grasp_setpoint(r)
    assert {s: sp[s] for s in aas.LIMB_IDS} == {s: r.node._grip_pos[s] for s in aas.LIMB_IDS}


def test_aimed_disabled_refuses(rig):
    r = rig(tf=False, **{'aimed.enabled': False})
    res = _aimed(r, 0.36, 0.0, 'base_link')
    assert res.reason == 'AIMED_UNAVAILABLE'


def test_no_region_derivation_at_start_by_default(rig):
    """USER DECISION 2026-11-05 (FR-16 item 5): with the active configuration the node
    starts no region timer, no region thread and no region publisher at boot."""
    r = rig()
    assert r.node._aim_cfg['region_at_start'] is False
    assert not hasattr(r.node, '_aim_region_timer')
    assert not hasattr(r.node, '_region_pub')
    time.sleep(3.0)                       # past the 1 s tick the switch would start
    assert r.node._aim_region_thread is None and r.node._aim_region is None
    assert not [t for t in threading.enumerate() if t.name == 'arm_aim_region']
    assert r.node.count_publishers(r.node._aim_cfg['region_topic']) == 0


def test_region_at_start_derives_and_publishes_as_before(rig):
    """aimed.region_at_start true: the background derivation runs once and the region
    is published latched on region_topic, as before the switch existed."""
    from std_msgs.msg import String
    r = rig(**{'aimed.region_at_start': True})
    got = []
    r.tester.create_subscription(String, r.node._aim_cfg['region_topic'],
                                 lambda m: got.append(m.data), aas.LATCHED)
    assert r.wait(lambda: r.node._aim_region is not None, 120.0), 'region not derived'
    assert r.node._aim_region_thread is not None
    assert r.wait(lambda: bool(got), 20.0), 'region not published'
    assert got[-1] == r.node._aim_region.text()


def test_aimed_parameters_are_read_once(rig):
    r = rig()
    from rclpy.parameter import Parameter
    r.node.set_parameters([Parameter('max_tf_age_sec', value=99.0)])
    assert r.node._aim_cfg['max_tf_age_sec'] == _shared_tf_bound()


def _shared_tf_bound():
    import yaml
    f = os.path.join(str(_SRC), 'gripperx_arm_msgs', 'config', 'tf_freshness.yaml')
    with open(f) as fh:
        return float(yaml.safe_load(fh)['/**']['ros__parameters']['max_tf_age_sec'])


def test_tf_bound_is_the_shared_one(rig):
    """SR-16 E3 / audit F4 (a): the arm's TF age bound is the top-level
    max_tf_age_sec of the file shared with the gateway - the ACTIVE value, and the
    old aimed.max_tf_age_sec is gone."""
    r = rig(tf=False)
    assert r.node._aim_cfg['max_tf_age_sec'] == _shared_tf_bound()
    assert not r.node.has_parameter('aimed.max_tf_age_sec')


def test_missing_tf_bound_refuses_aimed(rig):
    """Without the shared file there is no bound and no default: every aimed P is
    refused (AIMED_UNAVAILABLE, naming the parameter), nothing is written."""
    r = rig(drop=('max_tf_age_sec',))
    bx, by = _good_target(r)
    res = _refused(r, 'AIMED_UNAVAILABLE', bx, by, 'base_link')
    assert 'max_tf_age_sec' in res.message, res.message


def test_tf_intake_resumes_with_fresh_poses_after_a_stall(rig):
    """Robot 2026-11-09: odom -> base_footprint 12 ms old on the wire read 1.05-1.16 s
    old in the arm (STALE_TF): a /tf reader that fell behind under GIL load kept working
    off a full KEEP_LAST 100 queue. Here the intake stalls once for 1.5 s under a
    robot_state_publisher-like flood (8 joint transforms per message, 200 msg/s); the
    base poses it inserts afterwards must be fresh, not the queue's oldest."""
    from tf2_msgs.msg import TFMessage
    r = rig()
    node = r.node
    base = node._aim_cfg['base_frame']
    bound = node._aim_cfg['max_tf_age_sec'] / 4
    pub = r.tester.create_publisher(TFMessage, '/tf', 100)
    joints = [(base, f'{w}_steer_link') for w in ('fl', 'fr', 'rl', 'rr')] + \
             [(f'{w}_steer_link', f'{w}_wheel_link') for w in ('fl', 'fr', 'rl', 'rr')]
    run = [True]

    def flood():
        while run[0]:
            stamp = node.get_clock().now().to_msg()
            pub.publish(TFMessage(transforms=[FakeTf._tf(p, c, 0.0, 0.0, 0.0, stamp)
                                              for p, c in joints]))
            time.sleep(0.005)
    flooder = threading.Thread(target=flood, daemon=True)
    flooder.start()
    buf = node._tf_buffer
    real = buf.set_transform
    state = {'stall': False, 'lat': []}

    def watched(t, authority):
        if t.child_frame_id == base:
            if state['stall']:
                state['stall'] = False
                time.sleep(1.5)
            elif len(state['lat']) < 10:
                s = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                state['lat'].append(node.get_clock().now().nanoseconds * 1e-9 - s)
        return real(t, authority)
    try:
        time.sleep(1.0)
        buf.set_transform = watched
        state['stall'] = True
        assert r.wait(lambda: len(state['lat']) >= 10, 10.0), state
    finally:
        run[0] = False
        flooder.join(1.0)
        del buf.set_transform
    print(f'\nTF INTAKE after a 1.5 s stall: base pose ages '
          f'{[round(v, 3) for v in state["lat"]]} s')
    assert max(state['lat']) < bound, state['lat']


def test_aimed_planning_time(rig):
    """~20 targets, single process: the time from goal to the start of motion is
    dominated by the region test, the candidates and the plan. Reported, not gated."""
    r = rig()
    s = r.node._aim_solver()
    reg = s.derive_region(0.02)
    pts = [(float(reg.xs[i]), float(reg.ys[j])) for i, j in zip(*np.nonzero(reg.mask))]
    pts = pts[::max(1, len(pts) // 20)][:20]
    home_open = r.node._q_of({**HOME_D, aas.GRIPPER_ID: r.node._grip_open})
    times = []
    for x, y in pts:
        t0 = time.monotonic()
        res = s.solve(x, y, None, limit=4)
        for c in res.candidates:
            goal = r.node._q_of({**r.node._ticks_of(c.pregrasp),
                                 aas.GRIPPER_ID: r.node._grip_open})
            if r.node._planner(5.0, direct_only=True, direct_ms=4000).plan(
                    dict(home_open), goal, None).ok:
                break
        times.append(time.monotonic() - t0)
    print(f'\nAIMED PLANNING TIME over {len(times)} targets: median '
          f'{np.median(times) * 1e3:.0f} ms, max {max(times) * 1e3:.0f} ms '
          f'(load {os.getloadavg()})')
    assert times


def test_base_moving_after_motion_started_is_its_own_reason(rig, monkeypatch):
    """AP1 M5: the base starts moving after the gripper opened -> the P is aborted
    (SR-16 E5, audit F3: the base watch fires before the approach check), the arm
    returns home, and the reason is never re-approachable."""
    r = rig()
    bx, by = _good_target(r)
    real = aas.ArmActionServer._move_gripper

    def open_then_drive(self, ticks, move_ms, source, should_abort=None):
        real(self, ticks, move_ms, source, should_abort)
        if source == 'pick/open':
            r.tf.drift = (0.05, 0.0, 0.0)
            time.sleep(0.3)
    monkeypatch.setattr(aas.ArmActionServer, '_move_gripper', open_then_drive)
    res = _aimed(r, bx, by, 'base_link')
    assert not res.success and res.reason == 'BASE_MOVED_DURING_PICK', (res.reason,
                                                                       res.message)
    r.tf.drift = (0.0, 0.0, 0.0)
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), 'no return home after the refusal'


def test_base_moving_during_the_approach_ramp_aborts(rig, monkeypatch):
    """SR-16 E5 / audit F3: the base starts moving IN the approach ramp (after the
    last discrete base-still check) -> the ramp stops, the result names the base
    motion, the gripper never closes on the target, and the arm returns home (SR-17
    rule 13, the cancel path)."""
    r = rig()
    bx, by = _good_target(r)
    real = aas.ArmActionServer._write
    seen = {'approve': 0}

    def write(self, kind, setpoint):
        if kind == 'approve':
            seen['approve'] += 1
            if seen['approve'] == 5:
                r.tf.drift = (0.2, 0.0, 0.0)
        return real(self, kind, setpoint)
    monkeypatch.setattr(aas.ArmActionServer, '_write', write)
    res = _aimed(r, bx, by, 'base_link')
    r.tf.drift = (0.0, 0.0, 0.0)
    assert not res.success and res.reason == 'BASE_MOVED_DURING_PICK', (res.reason,
                                                                       res.message)
    closes = [sp for k, sp in list(r.node.audit)
              if k == 'gripper_unchecked' and sp.get(aas.GRIPPER_ID) == r.node._grip_close]
    assert not closes, 'the gripper closed after the base moved'
    assert any('during the pick' in m and 'BASE MOVING' in m for m in r.node.status_log)
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), 'no return home after the abort'


def test_target_shift_is_checked_on_the_arms_own_transform(rig):
    """SR-16 E11 item 4 / audit F5: a map -> odom correction between the preview and the
    confirmed pick (the robot did not move, so the gateway's map -> base_footprint view
    and the arm's base-still check see nothing) moves the target in the arm's
    pick-start transform: TARGET_SHIFTED, zero bus writes. A bad bound fails closed."""
    r = rig()
    bx, by = _good_target(r)
    mx, my = r.tf.to_map(bx, by)
    pre = _aimed(r, mx, my, 'map', preview=True)
    assert pre.success and pre.reason == 'PREVIEW_OK', pre.message
    seen = (pre.target_in_base.point.x, pre.target_in_base.point.y)
    ox, oy, oyaw = r.tf.MAP_ODOM
    r.tf.MAP_ODOM = (ox + 0.05, oy, oyaw)        # the localisation corrects by 5 cm
    time.sleep(0.3)
    res = _refused(r, 'TARGET_SHIFTED', mx, my, 'map', False, False, None,
                   (seen, 0.03))
    assert '50 mm' in res.message, res.message
    for bad in ((seen, 0.0), (seen, float('nan')), ((float('nan'), 0.0), 0.03)):
        _refused(r, 'INVALID_TARGET', mx, my, 'map', False, False, None, bad)


def test_target_shift_within_the_bound_picks(rig):
    r = rig()
    bx, by = _good_target(r)
    res = _aimed(r, bx, by, 'base_link', shift=((bx + 0.01, by), 0.03))
    assert res.success, (res.reason, res.message)
    assert any('target shift' in m and 'within' in m for m in r.node.status_log)


def test_one_preview_at_a_time_and_space_is_never_starved(rig, monkeypatch):
    """AP1 M2: slowed planner, four previews requested: one runs, three are rejected,
    and /arm/stop (own thread) is handled at once."""
    real = aas.Planner.plan

    def slow(self, qa, qb, env=None):
        time.sleep(3.0)
        return real(self, qa, qb, env)
    monkeypatch.setattr(aas.Planner, 'plan', slow)
    r = rig()
    bx, by = _good_target(r)
    assert r.wait(lambda: r.epoch == r.node._epoch_now(), 20.0)
    futs = []
    for _ in range(4):
        g = PickPlastic.Goal(execute=True, stop_epoch=r.epoch, aimed=True, preview=True)
        g.target.header.frame_id = 'base_link'
        g.target.point.x, g.target.point.y = bx, by
        futs.append(r.pick_client.send_goal_async(g))
    assert r.wait(lambda: all(f.done() for f in futs), 20.0)
    assert sum(f.result().accepted for f in futs) == 1
    from std_msgs.msg import Empty
    before = r.node._stop_count
    t0 = time.monotonic()
    r.stop.publish(Empty())
    assert r.wait(lambda: r.node._stop_count != before, 5.0)
    assert time.monotonic() - t0 < 1.0, time.monotonic() - t0
    h = [f.result() for f in futs if f.result().accepted][0]
    res = h.get_result_async()
    assert r.wait(lambda: res.done(), 60.0)


def test_arm_state_idle_and_at_home(rig):
    """/arm/state (the gateway's ARM_NOT_HOME input): idle + at home at rest, not idle
    during a P, not at home once a servo is away from its home tick."""
    from gripperx_arm.arm_action_server import LATCHED
    from gripperx_arm_msgs.msg import ArmState
    r = rig()
    seen = []
    r.tester.create_subscription(ArmState, '/arm/state', seen.append, LATCHED)
    assert r.wait(lambda: seen and seen[-1].idle and seen[-1].at_home, 10.0), seen[-1:]
    bx, by = _good_target(r)
    busy = []
    done = threading.Event()

    def watch():
        while not done.is_set():
            if seen and not seen[-1].idle:
                busy.append(seen[-1].owner)
            time.sleep(0.05)
    th = threading.Thread(target=watch, daemon=True)
    th.start()
    res = _aimed(r, bx, by, 'base_link')
    done.set()
    th.join(1.0)
    assert res.success, res.message
    assert 'pick' in busy, 'the state never said busy during the pick'
    assert r.wait(lambda: seen[-1].idle and seen[-1].at_home, 10.0)
    with r.node._serial_lock:
        r.bus.pos[2] = (r.bus.pos[2] + 200) % 4096           # limb2 pushed off home
    assert r.wait(lambda: not seen[-1].at_home, 5.0)


@pytest.mark.parametrize('with_gripper', [True, False])
def test_arm_state_after_a_fixed_p_with_the_object_in_the_jaws(rig, with_gripper):
    """AP3 F1/F6: after a successful fixed P the jaws hold the litter - the gripper stays
    far from home. Variant b (with_gripper): at_home false, the gateway would
    dispatch nothing more. Variant a (user decision 2026-10-13, the default): limbs only; gripper_at_home
    reports the gripper. A servo that does not answer: known false (F2)."""
    from gripperx_arm.arm_action_server import LATCHED
    from gripperx_arm_msgs.msg import ArmState
    r = rig(tf=False, drop=('poses.grip',),
            **{'poses.grip_rad': [0.3, 0.0, 0.0, 0.0, 0.3, 0.0],
               'arm_state.at_home_includes_gripper': with_gripper})
    seen = []
    r.tester.create_subscription(ArmState, '/arm/state', seen.append, LATCHED)
    h = _send_pick(r)
    res = h.get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    assert res.result().result.success
    with r.node._serial_lock:
        r.bus.pos[aas.GRIPPER_ID] = r.node._grip_open           # the object holds the jaws open
    assert r.wait(lambda: seen and seen[-1].idle and seen[-1].known
                  and not seen[-1].gripper_at_home, 10.0), seen[-1:]
    assert seen[-1].at_home is (not with_gripper)
    with r.node._serial_lock:
        r.bus.pos.pop(3)                                         # limb3 does not answer
    assert r.wait(lambda: not seen[-1].known and not seen[-1].at_home, 10.0)


# ---------------------------------------------------------------- audit 2026-11-13 N1
def _steer_event_at_approach_write(r, monkeypatch, event, nth=3):
    """Runs an aimed P and fires event() at the nth approach ramp write."""
    real = aas.ArmActionServer._write
    seen = {'approve': 0}

    def write(self, kind, setpoint):
        if kind == 'approve':
            seen['approve'] += 1
            if seen['approve'] == nth:
                event()
        return real(self, kind, setpoint)
    monkeypatch.setattr(aas.ArmActionServer, '_write', write)


def _assert_base_moved_abort(r, res, text):
    assert not res.success and res.reason == 'BASE_MOVED_DURING_PICK', (res.reason,
                                                                       res.message)
    closes = [sp for k, sp in list(r.node.audit)
              if k == 'gripper_unchecked' and sp.get(aas.GRIPPER_ID) == r.node._grip_close]
    assert not closes, 'the gripper closed after the steering reading was lost'
    assert any(text in m and 'BASE MOVING' in m for m in r.node.status_log), \
        [m for m in r.node.status_log if 'base-still' in m][-3:]
    r.tf.steer = [0.01, 0.01, 0.01, 0.01]
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), 'no return home after the abort'


def test_steering_going_stale_mid_pick_aborts(rig, monkeypatch):
    """Audit N1 (auditor's probe): /hw/steer_states stops at the 3rd approach write.
    A wheel read at the start without a fresh reading now counts as moved."""
    r = rig()
    bx, by = _good_target(r)
    timeout = float(r.node._p('safety_filter.steer_state_timeout_sec'))

    def stale():
        r.tf.steer = None
        time.sleep(timeout + 0.2)
    _steer_event_at_approach_write(r, monkeypatch, stale)
    res = _aimed(r, bx, by, 'base_link')
    _assert_base_moved_abort(r, res, 'steering reading LOST')


def test_steering_channel_zero_mid_pick_aborts(rig, monkeypatch):
    """Audit N1: one channel reads exactly 0.0 (steer_servo_node's dropped channel)
    from the 3rd approach write on."""
    r = rig()
    bx, by = _good_target(r)

    def zero():
        r.tf.steer = [0.01, 0.0, 0.01, 0.01]
        time.sleep(0.2)
    _steer_event_at_approach_write(r, monkeypatch, zero)
    res = _aimed(r, bx, by, 'base_link')
    _assert_base_moved_abort(r, res, 'steering reading LOST')


def test_steering_change_during_the_approach_ramp_aborts(rig, monkeypatch):
    """Audit (not covered before): the steering turns IN the approach ramp, after the
    last discrete base-still check."""
    r = rig()
    bx, by = _good_target(r)

    def turn():
        r.tf.steer = [0.3, 0.3, 0.3, 0.3]         # > steer_motion_margin_rad
        time.sleep(0.2)
    _steer_event_at_approach_write(r, monkeypatch, turn)
    res = _aimed(r, bx, by, 'base_link')
    _assert_base_moved_abort(r, res, 'steering change')


# ---------------------------------------------------------------- audit 2026-11-13 N7
MARGIN = 'aimed.region_margin_m'


def test_region_margin_in_the_shipped_config_is_the_prototype_assumption(rig):
    """User decision 2026-11-16: aimed.region_margin_m 0.0 in the ACTIVE yaml (prototype
    assumption, TO-VERIFY, not measured: E12 (d) owed), and the node reads it."""
    import yaml
    doc = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())
    assert doc['arm_action_server']['ros__parameters']['aimed']['region_margin_m'] == 0.0
    r = rig(tf=False)
    assert r.node._aim_cfg['region_margin_m'] == 0.0


def test_no_code_default_for_the_region_margin(rig):
    """Dropped from the config the margin is unset: no code default (SR-16 E4)."""
    r = rig(tf=False, drop=(MARGIN,))
    assert r.node._aim_cfg['region_margin_m'] is None


def test_unset_margin_keeps_the_interim_mode_and_the_operator_p(rig):
    """E4: no margin while only confirm_by_rearm runs - its preview and confirmed pick
    (check_target_shift) and the operator's aimed P proceed as before."""
    r = rig(drop=(MARGIN,))
    bx, by = _good_target(r)
    pre = _aimed(r, bx, by, 'base_link', execute=True, preview=True)
    assert pre.success and pre.reason == 'PREVIEW_OK', (pre.reason, pre.message)
    res = _aimed(r, bx, by, 'base_link', execute=True, shift=((bx, by), 0.03))
    assert res.success, (res.reason, res.message)
    assert r.wait(lambda: r.node._motion_owner is None, 30.0)
    res = _aimed(r, bx, by, 'base_link')              # operator (execute false, disarmed)
    assert res.success, (res.reason, res.message)


def test_unset_margin_refuses_mode_automatic(rig):
    """An aimed goal that is neither the interim mode's nor the operator's (= mode
    automatic) is refused while the margin has no value (N7); nothing is written."""
    r = rig(drop=(MARGIN,))
    bx, by = _good_target(r)
    res = _refused(r, 'AIMED_UNAVAILABLE', bx, by, 'base_link', True)
    assert 'region_margin_m' in res.message, res.message


def _edge_and_inner(r, margin):
    """(x, y) inside the region but within `margin` of its edge, and one that passes
    with the margin - both on y = 0, from the solver of the running node."""
    s = r.node._aim_solver()
    step = float(r.node._aim_cfg['region_step_m'])
    edge = inner = None
    for x in np.arange(0.20, 0.70, 0.005):
        x = float(x)
        if not s.in_region(x, 0.0).ok:
            continue
        if s.solve(x, 0.0, None, limit=1, margin=margin, margin_step=step).ok:
            inner = inner or (x, 0.0)
        elif s.solve(x, 0.0, None, limit=1).ok:
            edge = edge or (x, 0.0)
    if edge is None or inner is None:
        pytest.skip(f'no edge/inner target on y = 0 (edge {edge}, inner {inner})')
    return edge, inner


@pytest.mark.parametrize('execute', [False, True])
def test_set_margin_refuses_targets_near_the_edge(rig, execute):
    """A set margin shrinks the region inward for every aimed goal (also the operator's
    and, execute true, automatic): a target inside the region but within the margin of
    its edge is OUT_OF_REACH, nothing written; one farther inside passes the check."""
    m = 0.03                                     # test value, not a measurement
    r = rig(**{MARGIN: m})
    assert r.node._aim_cfg['region_margin_m'] == m
    edge, inner = _edge_and_inner(r, m)
    res = _refused(r, 'OUT_OF_REACH', *edge, 'base_link', execute)
    assert 'inward margin 30 mm' in res.message, res.message
    pre = _aimed(r, *inner, 'base_link', execute, True)
    assert pre.success and pre.reason == 'PREVIEW_OK', (pre.reason, pre.message)


def test_region_margin_is_startup_only(rig):
    r = rig(tf=False, **{MARGIN: 0.02})
    from rclpy.parameter import Parameter
    r.node.set_parameters([Parameter(MARGIN, value=0.5)])
    assert r.node._aim_cfg['region_margin_m'] == 0.02


@pytest.mark.parametrize('bad', [-0.01, float('nan'), float('inf')])
def test_invalid_region_margin_counts_as_unset(rig, bad):
    r = rig(**{MARGIN: bad})
    assert r.node._aim_cfg['region_margin_m'] is None, bad
    bx, by = _good_target(r)
    _refused(r, 'AIMED_UNAVAILABLE', bx, by, 'base_link', True)


# ---------------------------------------------------------------- steep approach 2026-11-16
# User decision 2026-11-16: approach to a pre-grasp pose aimed.pregrasp_height_m above the
# grasp, then a vertical descent (same roll); after the close the same line back up.
def _record_ramps(r, monkeypatch):
    """[(source, segments)] of every _ramp call, in order."""
    real, log = aas.ArmActionServer._ramp, []

    def ramp(self, segments, source, *a, **kw):
        log.append((source, list(segments)))
        return real(self, segments, source, *a, **kw)
    monkeypatch.setattr(aas.ArmActionServer, '_ramp', ramp)
    return log


def _tcp(r, ticks):
    return r.node._filter.kin.tcp(r.node._q_of({**HOME_D, **ticks}))


def test_steep_approach_pregrasp_descent_and_lift(rig, monkeypatch):
    """The approach ends at the pre-grasp pose (grasp + pregrasp_height_m in z, same x, y,
    vertical, the grasp's roll); the descent stays on the vertical line and reaches the
    grasp; after the close the arm lifts back up the line; then home. The descent is
    never faster than planner.max_joint_velocity_rad_s per joint."""
    r = rig()
    log = _record_ramps(r, monkeypatch)
    bx, by = _good_target(r)
    res = _aimed(r, bx, by, 'base_link')
    assert res.success, (res.reason, res.message)
    srcs = [src for src, _ in log]
    assert srcs == ['pick/open', 'pick/approach', 'pick/descent', 'pick/grip_close',
                    'pick/lift', 'pick/return_home', 'pick/gripper_home'], srcs
    seg = dict(log)
    s = r.node._aim_solver()
    h = float(r.node._p('aimed.pregrasp_height_m'))
    lat = float(r.node._p('aimed.descent_max_lateral_m'))
    tick = 0.003                                  # a few ticks of quantisation
    pre = _tcp(r, seg['pick/approach'][-1][1])
    assert np.linalg.norm(pre[:3, 3] - [bx, by, s.z + h]) < tick, pre[:3, 3]
    assert pre[2, 2] == pytest.approx(-1.0, abs=0.01)              # vertical
    roll = r.node._cal.joints[4]
    rolls = {e[aas.LIMB_IDS[4]] for e in [seg['pick/approach'][-1][1]]
             + [sg[1] for sg in seg['pick/descent']] + [sg[1] for sg in seg['pick/lift']]}
    assert len(rolls) == 1, f'{roll} changes in the descent/lift: {rolls}'
    zs = []
    for sg in seg['pick/descent']:
        for f in np.linspace(0.0, 1.0, 6):                      # along the tick ramp
            t = _tcp(r, {sid: round(sg[0][sid] + f * (sg[1][sid] - sg[0][sid]))
                         for sid in aas.LIMB_IDS})
            assert math.hypot(t[0, 3] - bx, t[1, 3] - by) < lat + tick, t[:3, 3]
            assert t[2, 2] == pytest.approx(-1.0, abs=0.01)
            zs.append(t[2, 3])
    assert all(b <= a + 1e-3 for a, b in zip(zs, zs[1:])), 'the descent goes up'
    end = _tcp(r, seg['pick/descent'][-1][1])
    assert np.linalg.norm(end[:3, 3] - [bx, by, s.z]) < tick
    assert _grasp_setpoint(r) == {k: v for k, v in seg['pick/descent'][-1][1].items()}
    top = _tcp(r, seg['pick/lift'][-1][1])
    assert np.linalg.norm(top[:3, 3] - [bx, by, s.z + h]) < tick
    v = float(r.node._p('planner.max_joint_velocity_rad_s'))
    cal = r.node._cal
    for sg in seg['pick/descent'] + seg['pick/lift']:
        qa, qb = r.node._q_of({**HOME_D, **sg[0]}), r.node._q_of({**HOME_D, **sg[1]})
        assert max(abs(qb[n] - qa[n]) for n in cal.joints[:5]) / sg[2] <= v + 1e-6

    def speed(segs):
        return max(np.linalg.norm(_tcp(r, b)[:3, 3] - _tcp(r, a)[:3, 3]) / t
                   for a, b, t, *_ in segs)
    secs = {k: sum(sg[2] for sg in seg[k]) for k in ('pick/approach', 'pick/descent',
                                                      'pick/lift')}
    print(f'\nSTEEP APPROACH: segment seconds {secs}; max TCP speed per segment (m/s): '
          f'approach {speed(seg["pick/approach"]):.3f}, descent '
          f'{speed(seg["pick/descent"]):.3f}, lift {speed(seg["pick/lift"]):.3f}')
    assert any('base-still check before the descent' in m for m in r.node.status_log)
    assert r.wait(lambda: r.bus.pos == HOME_D, 20.0)


def _in_descent(r, monkeypatch, event, nth=3, phase='pick/descent'):
    """Runs event() right after the nth arm write of `phase`'s ramp."""
    ramp, write = aas.ArmActionServer._ramp, aas.ArmActionServer._write
    state = {'src': None, 'n': 0, 'fired': False}

    def ramp_(self, segments, source, *a, **kw):
        state['src'] = source
        return ramp(self, segments, source, *a, **kw)

    def write_(self, kind, setpoint):
        out = write(self, kind, setpoint)
        if state['src'] == phase and kind == 'approve':
            state['n'] += 1
            if state['n'] == nth:
                state['fired'] = True
                event()
        return out
    monkeypatch.setattr(aas.ArmActionServer, '_ramp', ramp_)
    monkeypatch.setattr(aas.ArmActionServer, '_write', write_)
    return state


def _no_close(r):
    return not [sp for k, sp in list(r.node.audit)
                if k == 'gripper_unchecked' and sp.get(aas.GRIPPER_ID) == r.node._grip_close]


@pytest.mark.parametrize('how', ['handler', 'topic'])
def test_space_during_the_descent_stops_the_arm(rig, monkeypatch, how):
    """Space (/arm/stop -> stop epoch) in the descent: the ramp stops at its next
    setpoint, torque off, no goal written after it, the gripper never closes, no
    return home; the result is STOPPED."""
    from std_msgs.msg import Empty
    r = rig()
    bx, by = _good_target(r)
    mark = []

    def space():
        if how == 'handler':
            r.node._on_stop(Empty())
        else:
            e0 = r.node._epoch_now()
            r.stop.publish(Empty())
            assert r.wait(lambda: r.node._epoch_now() != e0, 5.0)
        mark.append(len(r.bus.events))
    st = _in_descent(r, monkeypatch, space)
    res = _aimed(r, bx, by, 'base_link')
    assert st['fired'], 'the descent never ran'
    assert not res.success and res.reason == 'STOPPED', (res.reason, res.message)
    time.sleep(r.node._grace + 1.0)
    after = [e for e in r.bus.events[mark[0]:]
             if e[0] == 'goal' or (e[0] == 'torque' and e[2])]
    assert after == [], after[:6]
    assert not any(r.bus.torque_on.values()), 'arm not limp after Space in the descent'
    assert _no_close(r)


def _assert_aborted_home(r, res, reason, text=None):
    assert not res.success and res.reason == reason, (res.reason, res.message)
    assert _no_close(r), 'the gripper closed after the abort'
    if text:
        assert any(text in m for m in r.node.status_log), text
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), 'no return home after the abort'


def test_base_moving_during_the_descent_aborts(rig, monkeypatch):
    r = rig()
    bx, by = _good_target(r)

    def drive():
        r.tf.drift = (0.2, 0.0, 0.0)
        time.sleep(0.2)
    st = _in_descent(r, monkeypatch, drive)
    res = _aimed(r, bx, by, 'base_link')
    r.tf.drift = (0.0, 0.0, 0.0)
    assert st['fired']
    _assert_aborted_home(r, res, 'BASE_MOVED_DURING_PICK', 'BASE MOVING')


def test_steering_change_during_the_descent_aborts(rig, monkeypatch):
    r = rig()
    bx, by = _good_target(r)

    def turn():
        r.tf.steer = [0.3, 0.3, 0.3, 0.3]
        time.sleep(0.2)
    st = _in_descent(r, monkeypatch, turn)
    res = _aimed(r, bx, by, 'base_link')
    r.tf.steer = [0.01, 0.01, 0.01, 0.01]
    assert st['fired']
    _assert_aborted_home(r, res, 'BASE_MOVED_DURING_PICK', 'steering change')


def test_steering_lost_during_the_descent_aborts(rig, monkeypatch):
    """Audit N1 in the descent: /hw/steer_states goes stale."""
    r = rig()
    bx, by = _good_target(r)
    timeout = float(r.node._p('safety_filter.steer_state_timeout_sec'))

    def stale():
        r.tf.steer = None
        time.sleep(timeout + 0.2)
    st = _in_descent(r, monkeypatch, stale)
    res = _aimed(r, bx, by, 'base_link')
    r.tf.steer = [0.01, 0.01, 0.01, 0.01]
    assert st['fired']
    _assert_aborted_home(r, res, 'BASE_MOVED_DURING_PICK', 'steering reading LOST')


def test_tf_stale_during_the_descent_aborts(rig, monkeypatch):
    """The base pose can no longer be read fresh in the descent: aborts (fail-closed)."""
    r = rig()
    bx, by = _good_target(r)
    age = float(r.node._aim_cfg['max_tf_age_sec'])

    def freeze():
        r.tf.live = False
        time.sleep(age + 0.3)
    st = _in_descent(r, monkeypatch, freeze)
    res = _aimed(r, bx, by, 'base_link')
    r.tf.live = True
    assert st['fired']
    _assert_aborted_home(r, res, 'BASE_MOVED_DURING_PICK', 'base rest no longer shown')


def test_goal_cancel_during_the_descent_returns_home(rig, monkeypatch):
    r = rig()
    bx, by = _good_target(r)
    handle = []

    def cancel():
        handle[0].cancel_goal_async()
        assert r.wait(lambda: handle[0].status in (2, 3), 5.0) or True   # CANCELING
        time.sleep(0.2)
    st = _in_descent(r, monkeypatch, cancel)
    res = _aimed(r, bx, by, 'base_link', handle=handle)
    assert st['fired']
    _assert_aborted_home(r, res, 'CANCELED')


def test_base_moving_during_the_lift_aborts(rig, monkeypatch):
    """The lift (with the object) is watched like every motion of the aimed P."""
    r = rig()
    bx, by = _good_target(r)

    def drive():
        r.tf.drift = (0.2, 0.0, 0.0)
        time.sleep(0.2)
    st = _in_descent(r, monkeypatch, drive, phase='pick/lift')
    res = _aimed(r, bx, by, 'base_link')
    r.tf.drift = (0.0, 0.0, 0.0)
    assert st['fired']
    assert not res.success and res.reason == 'BASE_MOVED_DURING_PICK', res.message
    assert r.wait(lambda: r.bus.pos == HOME_D, 30.0), 'no return home after the abort'


DESCENT = ('aimed.pregrasp_height_m', 'aimed.descent_step_m', 'aimed.descent_max_lateral_m')


def test_descent_parameters_from_the_active_config(rig):
    """User decision 2026-11-16: 0.10 m in the ACTIVE yaml, read by the node."""
    import yaml
    doc = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())
    a = doc['arm_action_server']['ros__parameters']['aimed']
    assert a['pregrasp_height_m'] == 0.10
    r = rig(tf=False)
    for k in DESCENT:
        assert r.node._aim_cfg[k.split('.')[1]] == a[k.split('.')[1]]
    s = r.node._aim_solver()
    assert s.pregrasp_height == a['pregrasp_height_m']


@pytest.mark.parametrize('key', DESCENT)
def test_missing_descent_parameter_refuses_aimed(rig, key):
    """No code default: missing -> every aimed P refused (AIMED_UNAVAILABLE), nothing
    written; the direct approach of before cannot come back silently."""
    r = rig(drop=(key,))
    assert r.node._aim_cfg[key.split('.')[1]] is None
    res = _refused(r, 'AIMED_UNAVAILABLE', 0.36, 0.0, 'base_link')
    assert key in res.message, res.message


@pytest.mark.parametrize('bad', [0.0, -0.1, float('nan'), float('inf')])
def test_invalid_pregrasp_height_refuses_aimed(rig, bad):
    r = rig(**{'aimed.pregrasp_height_m': bad})
    _refused(r, 'AIMED_UNAVAILABLE', 0.36, 0.0, 'base_link')


def test_descent_step_above_the_height_refuses_aimed(rig):
    r = rig(**{'aimed.descent_step_m': 0.2, 'aimed.pregrasp_height_m': 0.1})
    _refused(r, 'AIMED_UNAVAILABLE', 0.36, 0.0, 'base_link')


def test_descent_parameters_are_startup_only(rig):
    from rclpy.parameter import Parameter
    r = rig(tf=False)
    h = r.node._aim_cfg['pregrasp_height_m']
    r.node.set_parameters([Parameter('aimed.pregrasp_height_m', value=0.3)])
    assert r.node._aim_cfg['pregrasp_height_m'] == h
    assert r.node._aim_solver().pregrasp_height == h


def test_preview_plans_the_descent_and_moves_nothing(rig):
    r = rig()
    bx, by = _good_target(r)
    n = len(r.bus.events)
    pre = _aimed(r, bx, by, 'base_link', execute=True, preview=True)
    assert pre.success and pre.reason == 'PREVIEW_OK', (pre.reason, pre.message)
    assert 'vertical descent' in pre.message, pre.message
    assert len(r.bus.events) == n
