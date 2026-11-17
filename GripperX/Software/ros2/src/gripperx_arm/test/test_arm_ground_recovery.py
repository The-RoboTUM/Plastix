"""Ground-margin recovery in the arm node (user decision 2026-10-12; SR-17): only the
operator's I lifts the arm out of the ground margin; the automatic return home, P and
the recovery switched off refuse as before. Same mock bus and rig as test_arm_node,
with the lidar in the robot description as on the robot. The filter-level rules:
test_ground_recovery.py."""
import sys
import time
from pathlib import Path

import pytest
import xacro
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_arm_node as tan  # noqa: E402
from test_arm_node import (  # noqa: E402,F401
    DETERMINISTIC, GRIP_NEAR_HOME, HOME_D, _send_pick, _status, rig_factory)
from test_ground_recovery import GRIPPER, LIMB5, _robot_case, _sagged  # noqa: E402
from test_planner import JOINTS  # noqa: E402


@pytest.fixture(autouse=True)
def lidar_description():
    old = tan.URDF
    tan.URDF = xacro.process_file(str(tan._desc() / 'urdf' / 'gripperx_v1.urdf.xacro'),
                                  mappings={'use_lidar': 'true'}).toxml()
    assert 'lidar_link' in tan.URDF
    yield
    tan.URDF = old


def _start(node):
    return _sagged()


def _written(node, start_q, n):
    poses, here = [start_q], dict(start_q)
    for k, sp in list(node.audit)[n:]:
        if k == 'approve':
            here = node._q_of({**node._ticks_of(here), **sp})
            poses.append(here)
    return poses


def test_i_lifts_the_arm_out_of_the_ground_margin(rig_factory):
    r = rig_factory(start_q=_start, **DETERMINISTIC)
    flt = r.node._filter
    start = r.measured_q()
    assert list(flt.ground_shortfalls(start)) == [LIMB5]
    n = len(r.node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 60.0), list(r.node.status_log)[-4:]
    assert _status(r, 'go_home: start pose inside the ground margin (arm_limb5_link 1.1 mm '
                   '(grown for the first step')
    assert any('go_home (detours allowed; lifting out of the ground margin): path ordered'
               in t for t in list(r.node.status_log)), list(r.node.status_log)[-4:]
    assert _status(r, 'go_home: lifted out of the ground margin (measured)')
    poses = _written(r.node, start, n)
    h = [flt.ground_heights(p)[LIMB5] for p in poses]
    k = next(i for i, x in enumerate(h) if x >= flt.ground_need(LIMB5))
    assert k >= 2 and all(h[i] > h[i - 1] for i in range(1, k + 1)), h[:k + 1]
    assert all(sum(poses[i][j] != poses[i - 1][j] for j in JOINTS) == 1
               for i in range(1, k + 1))                          # one joint at a time
    assert min(h) == h[0]


def test_i_from_the_held_incident_pose(rig_factory):
    """Audit r11 F1: where leader mode left the arm (limb5 12.4 mm, statically clear,
    inside the margin for the first step's motion bound) I lifts it out."""
    r = rig_factory(start_q=lambda node: _sagged(0), **DETERMINISTIC)
    assert not r.node._filter.ground_shortfalls(r.measured_q())
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 60.0), list(r.node.status_log)[-4:]
    assert _status(r, 'lifting out of the ground margin): path ordered')


def test_a_sagging_servo_stops_the_lift_and_keeps_the_goal(rig_factory):
    """Audit r11 F3: limb2 measured below where the lift started (it sags instead of
    following): the ramp stops at the next setpoint, no hold write of the measured pose
    (it would lower the arm again), torque stays on."""
    r = rig_factory(start_q=_start, **DETERMINISTIC)
    bus, node = r.bus, r.node
    stuck = bus.pos[2] + 4                        # 4 ticks further down than the start
    n = len(node.audit)
    real = bus.read_pos

    def read(sid):
        lifting = 'approve' in [k for k, _ in list(node.audit)[n:]]
        return stuck if sid == 2 and lifting else real(sid)
    bus.read_pos = read
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'HOLD (last goal kept): go_home: ground recovery - measured '
                   'clearance below the start', 60.0), list(node.status_log)[-4:]
    kinds = [k for k, _ in list(node.audit)[n:]]
    assert kinds.count('approve') == 2 and 'hold' not in kinds, kinds   # two reads in a row
    assert all(bus.torque_on.values())


def test_one_tick_reading_noise_does_not_stop_the_lift(rig_factory):
    """Audit r11b B1: every present-position read off by a random -1/0/+1 tick on every
    servo during the lift: the lift completes."""
    import random
    r = rig_factory(start_q=_start, **DETERMINISTIC)
    bus, node = r.bus, r.node
    n = len(node.audit)
    real = bus.read_pos
    rnd = random.Random(11)

    def read(sid):
        v = real(sid)
        lifting = 'approve' in [k for k, _ in list(node.audit)[n:]]
        return v + rnd.choice((-1, 0, 1)) if lifting and v is not None else v
    bus.read_pos = read
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'go_home: lifted out of the ground margin (measured)', 60.0), \
        list(node.status_log)[-4:]
    assert r.wait(lambda: 'go_home' not in str(node._motion_owner), 60.0)
    assert not any('HOLD' in t for t in list(node.status_log))


def test_an_arm_that_sagged_while_planning_is_refused(rig_factory):
    """Audit r11b B2: limb2 3 ticks lower at the read right before the first write than
    at the planning read: refused, nothing written."""
    r = rig_factory(start_q=_start, **DETERMINISTIC)
    bus, node = r.bus, r.node
    n = len(node.audit)
    real = bus.read_pos
    start2 = bus.pos[2]

    def read(sid):
        planned = any('lifting out of the ground margin): path' in t
                      for t in list(node.status_log))
        return start2 + 3 if sid == 2 and planned else real(sid)
    bus.read_pos = read
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'the arm moved while the path was planned', 60.0), \
        list(node.status_log)[-4:]
    assert [k for k, _ in list(node.audit)[n:]] == []


def test_space_during_the_lift_is_a_stop_not_a_hold(rig_factory):
    """Audit r11b B3: Space while the lift runs and the limp arm falls: reported as
    STOPPED, torque off, nothing written after the torque-off."""
    import threading
    r = rig_factory(start_q=_start, **DETERMINISTIC)
    bus, node = r.bus, r.node
    n = len(node.audit)
    real = bus.read_pos
    start2 = bus.pos[2]
    st = {'spaced': False}

    def read(sid):
        if [k for k, _ in list(node.audit)[n:]].count('approve') >= 3 and not st['spaced']:
            st['spaced'] = True
            threading.Thread(target=node._on_stop, args=(tan.Empty(),)).start()
            time.sleep(0.2)
        return start2 + 6 if st['spaced'] and sid == 2 else real(sid)
    bus.read_pos = read
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'STOPPED', 60.0), list(node.status_log)[-4:]
    time.sleep(1.0)
    log = list(node.status_log)
    assert not any('HOLD (last goal kept)' in t for t in log), log[-4:]
    kinds = [k for k, _ in list(node.audit)[n:]]
    i_off = max(i for i, k in enumerate(kinds) if k == 'torque_off')
    assert 'approve' not in kinds[i_off + 1:]
    assert not any(bus.torque_on.values())


def test_the_automatic_return_home_holds(rig_factory):
    r = rig_factory(start_q=_start, **DETERMINISTIC)
    n = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: 'hold' in [k for k, _ in list(r.node.audit)[n:]], 60.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]
    assert any('automatic: direct path only' in t and 'start pose rejected' in t
               and '[ground]' in t for t in list(r.node.status_log)), \
        list(r.node.status_log)[-4:]
    assert not any('ground recovery' in t for t in list(r.node.status_log))


def test_i_with_the_recovery_off_is_refused(rig_factory):
    r = rig_factory(start_q=_start, **DETERMINISTIC,
                    **{'safety_filter.ground_recovery': 'off'})
    n = len(r.node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: any('go_home (detours allowed' in t and 'start pose rejected' in t
                              and '[ground]' in t for t in list(r.node.status_log)), 60.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]


def test_pick_from_inside_the_margin_is_refused(rig_factory):
    r = rig_factory(start_q=_start, drop=('poses.grip',), **GRIP_NEAR_HOME)
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert not res.result().result.success, msg
    assert 'start pose rejected' in msg and '[ground]' in msg, msg
    assert len(r.bus.writes) == n


def test_an_unknown_mode_refuses_to_start():
    with pytest.raises(Exception, match='ground_recovery must be operator_go_home|off'):
        tan.Rig(**{'safety_filter.ground_recovery': 'always'})


# ---------------------------------------------------------------- below the ground
# (user decision 2026-10-15; the robot case of 15:51:59, offline reconstruction:
# test_ground_recovery.ROBOT_2026_10_15)

def _robot(node):
    return _robot_case()


def test_i_lifts_the_robot_case_out_of_the_ground(rig_factory):
    r = rig_factory(start_q=_robot, **DETERMINISTIC)
    flt = r.node._filter
    start = r.measured_q()
    h0 = flt.ground_heights(start)
    assert h0[GRIPPER] < 0.0 and h0[LIMB5] < 0.0, h0
    n = len(r.node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 60.0), list(r.node.status_log)[-4:]
    assert _status(r, f'go_home: start pose inside the ground margin ({GRIPPER} -4.')
    assert _status(r, 'never lower than at the setpoint before')
    assert _status(r, 'go_home: lifted out of the ground margin (measured)')
    poses = _written(r.node, start, n)
    for name in (GRIPPER, LIMB5):
        h = [flt.ground_heights(p)[name] for p in poses]
        k = next(i for i, x in enumerate(h) if x >= flt.ground_need(name))
        assert k >= 2 and all(h[i] > h[i - 1] for i in range(1, k + 1)), (name, h[:k + 1])
        assert all(sum(poses[i][j] != poses[i - 1][j] for j in JOINTS) == 1
                   for i in range(1, k + 1))
        assert min(h) == h[0], name


def test_one_tick_noise_below_the_ground_does_not_stop_the_lift(rig_factory):
    """The lift guard from a negative baseline: every read off by -1/0/+1 tick."""
    import random
    r = rig_factory(start_q=_robot, **DETERMINISTIC)
    bus, node = r.bus, r.node
    n = len(node.audit)
    real = bus.read_pos
    rnd = random.Random(5)

    def read(sid):
        v = real(sid)
        lifting = 'approve' in [k for k, _ in list(node.audit)[n:]]
        return v + rnd.choice((-1, 0, 1)) if lifting and v is not None else v
    bus.read_pos = read
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'go_home: lifted out of the ground margin (measured)', 60.0), \
        list(node.status_log)[-4:]
    assert not any('HOLD' in t for t in list(node.status_log))


def test_a_sagging_servo_below_the_ground_stops_the_lift(rig_factory):
    """The lift guard holds below the ground as above it: limb2 measured 4 ticks further
    down than the start while lifting -> HOLD, no hold write, torque on."""
    r = rig_factory(start_q=_robot, **DETERMINISTIC)
    bus, node = r.bus, r.node
    stuck = bus.pos[2] + 4
    n = len(node.audit)
    real = bus.read_pos

    def read(sid):
        lifting = 'approve' in [k for k, _ in list(node.audit)[n:]]
        return stuck if sid == 2 and lifting else real(sid)
    bus.read_pos = read
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'HOLD (last goal kept): go_home: ground recovery - measured '
                   'clearance below the start', 60.0), list(node.status_log)[-4:]
    kinds = [k for k, _ in list(node.audit)[n:]]
    assert kinds.count('approve') == 2 and 'hold' not in kinds, kinds
    assert all(bus.torque_on.values())


def test_the_automatic_return_home_holds_below_the_ground(rig_factory):
    r = rig_factory(start_q=_robot, **DETERMINISTIC)
    n = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: 'hold' in [k for k, _ in list(r.node.audit)[n:]], 60.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]
    assert any('automatic: direct path only' in t and 'start pose rejected' in t
               and '[ground]' in t for t in list(r.node.status_log)), \
        list(r.node.status_log)[-4:]


def test_pick_from_below_the_ground_is_refused(rig_factory):
    r = rig_factory(start_q=_robot, drop=('poses.grip',), **GRIP_NEAR_HOME)
    n = len(r.bus.writes)
    res = _send_pick(r).get_result_async()
    assert r.wait(lambda: res.done(), 60.0)
    msg = res.result().result.message
    assert not res.result().result.success, msg
    assert 'start pose rejected' in msg and '[ground]' in msg, msg
    assert len(r.bus.writes) == n


def test_i_below_the_ground_with_the_recovery_off_is_refused(rig_factory):
    r = rig_factory(start_q=_robot, **DETERMINISTIC,
                    **{'safety_filter.ground_recovery': 'off'})
    n = len(r.node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: any('go_home (detours allowed' in t and 'start pose rejected' in t
                              and '[ground]' in t for t in list(r.node.status_log)), 60.0), \
        list(r.node.status_log)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n:]]


def _held_above(node, here):
    """Held goals (torque on, goals kept) that put limb5 and the gripper above the
    margin: limb2 60 ticks up from the measured pose on the floor."""
    flt = node._filter
    return max(({**here, 2: here[2] + d} for d in (-60, 60)),
               key=lambda t: flt.ground_heights(node._q_of(t))[GRIPPER])


def test_i_from_held_goals_with_the_measured_pose_in_the_ground(rig_factory, monkeypatch):
    """Audit GR2 M1: held goals at/above the margin, the MEASURED pose below 0 (the arm
    rests on the floor, the lift guard cannot see it pressing): I plans from the
    measured pose, so the rise rule applies to the real pose - single joint, rising."""
    r = rig_factory(start_q=_robot, **DETERMINISTIC)
    node, flt = r.node, r.node._filter
    here = dict(r.bus.pos)
    held = _held_above(node, here)
    hq = node._q_of(held)
    assert not any(h <= 0.0 for h in flt.ground_heights(hq).values())
    assert flt.ground_heights(hq)[GRIPPER] > flt.ground_heights(r.measured_q())[GRIPPER]
    monkeypatch.setattr(node, '_held_start', lambda h, source, report=True: dict(held))
    start = r.measured_q()
    n = len(node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: r.bus.pos == HOME_D, 60.0), list(node.status_log)[-4:]
    assert _status(r, 'go_home: measured pose at or below the ground ('), \
        list(node.status_log)[-6:]
    assert _status(r, 'starts at the measured pose, not at the held goals')
    assert _status(r, f'go_home: start pose inside the ground margin ({GRIPPER} -4.')
    poses = _written(node, start, n)
    h = [flt.ground_heights(p)[GRIPPER] for p in poses]
    k = next(i for i, x in enumerate(h) if x >= flt.ground_need(GRIPPER))
    assert k >= 2 and all(h[i] > h[i - 1] for i in range(1, k + 1)), h[:k + 1]
    assert all(sum(poses[i][j] != poses[i - 1][j] for j in JOINTS) == 1
               for i in range(1, k + 1))


def test_i_from_held_goals_in_the_ground_without_a_rising_first_section(rig_factory,
                                                                        monkeypatch):
    """The same start, but a goal no first joint lifts toward (limb1 yaw + limb4, as
    test_no_upward_only_first_section_is_refused): refused, nothing written - also
    though the held goals themselves are clear of the ground."""
    r = rig_factory(start_q=_robot, **DETERMINISTIC)
    node = r.node
    here = dict(r.bus.pos)
    held = _held_above(node, here)
    monkeypatch.setattr(node, '_held_start', lambda h, source, report=True: dict(held))
    a = r.measured_q()
    g = dict(a, arm_limb4_joint=a['arm_limb4_joint'] + 1.0,
             arm_limb1_joint=a['arm_limb1_joint'] + 0.5)
    node._home_pos = {**node._ticks_of(g), 6: node._home_pos[6]}
    n = len(node.audit)
    r.cmd.publish(String(data='go_home'))
    assert r.wait(lambda: any('no first section lifts it' in t
                              for t in list(node.status_log)), 60.0), \
        list(node.status_log)[-4:]
    assert _status(r, 'starts at the measured pose, not at the held goals')
    time.sleep(0.5)
    assert 'approve' not in [k for k, _ in list(node.audit)[n:]]
