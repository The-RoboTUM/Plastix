"""The two leader failures of the robot run of 2026-11-02, against the mock bus with the
ACTIVE configuration and the recorded pose (bag 2026-11-02_arm_sag_test1):

- after a rejection the arm never resumed (FR-17 item 4): while the search ran, the tick
  starved behind it, cancelled searches kept computing to the end of their check, and
  a check longer than one slice counted as a failed search;
- one failed position read ended leader mode, and nothing said which servo failed.

Not covered here: the GIL contention itself is timing, measured with the real SDK on a
pty and never asserted (only that the configured switch interval is applied)."""
import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '231')
sys.path.insert(0, str(Path(__file__).parent))

import pytest  # noqa: E402
import yaml  # noqa: E402
from std_msgs.msg import Bool, Empty  # noqa: E402

import test_arm_node as tan  # noqa: E402
from test_arm_node import rig_factory  # noqa: E402,F401
from gripperx_arm import arm_action_server as aas  # noqa: E402

# Recorded 2026-11-02 13:15:25-36: measured ticks of the held arm, the goals it held
# (follow_debug setpoint) and the pose the leader then rested at.
MEASURED = dict(zip(aas.IDS, [1743, 1991, 2721, 2158, 1179, 1529]))
HELD_GOALS = dict(zip(aas.IDS, [1741, 1991, 2722, 2152, 1175, 1529]))
LEADER_REST = [-0.084, -0.219, 0.008, -0.109, -0.063]


def _held(r):
    """Leader mode active and holding at the recorded pose, as after the rejection."""
    n = r.node
    r.bus.pos.update(MEASURED)
    with n._serial_lock:
        n._goal.update(HELD_GOALS)
    r.leader_q = n._q_of(MEASURED)
    tan._activate(r)
    with n._state_lock:                      # together: no search toward the old pose
        n._held = True
        n._held_since = n._hold_status_t = time.monotonic()
        r.leader_q = n._leader_q = _rest(n)
    return n


def _rest(n):
    # the leader gripper rests where the follower's is (the gripper is driven in leader
    # mode since 2026-11-13; these cases are about the limbs)
    g = n._cal.joints[aas.IDS.index(aas.GRIPPER_ID)]
    return {**dict(zip(n._cal.joints[:5], LEADER_REST)), g: n._q_of(MEASURED)[g]}


def _searching(r):
    """The search toward the leader's recorded rest pose."""
    n = r.node
    rest = _rest(n)

    def job():
        j = n._replan                        # not the one toward the activation pose
        return j is not None and all(abs(j.target[k] - v) < 0.01 for k, v in rest.items())
    assert r.wait(job, 10.0), n._hold_reason
    return n._replan


def _search_poses(n):
    """The poses the search thread has checked so far (grows while it runs)."""
    calls, real = [], n._filter.check_pose

    def counted(*a, **k):
        if threading.current_thread().name == 'leader_replan':
            calls.append(1)
        return real(*a, **k)
    n._filter.check_pose = counted
    return calls


# ------------------------------------------------------------------ FR-17 item 4
def test_a_search_longer_than_one_slice_is_resumed_and_the_arm_follows(rig_factory):
    """A slice far shorter than one direct-path check. Before: 'not one pose passed
    within one slice', recorded as failed - the arm held for good, with a valid path."""
    r = rig_factory(**{'planner.time_budget_s': 0.3})
    n = _held(r)
    base = len(r.kinds())
    job = _searching(r)
    assert r.wait(lambda: 'approve' in r.kinds()[base:], 60.0), n._hold_reason
    assert job.slices > 1, 'the check fitted one slice: this test no longer proves anything'
    assert n._replan_failed is None


def test_a_cancelled_search_stops_inside_its_check(rig_factory):
    """Each replaced search kept computing to the end of its check (seconds on the
    robot): up to several threads competed with the tick, read_s grew 0.1 -> 0.67 s."""
    r = rig_factory(**{'planner.time_budget_s': 60.0})
    n = _held(r)
    calls = _search_poses(n)
    job = _searching(r)
    assert r.wait(lambda: len(calls) >= 10, 20.0)      # past start and goal: in the check
    job.cancel()
    at_cancel = len(calls)
    job.thread.join(0.5)
    assert not job.thread.is_alive()
    assert len(calls) <= at_cancel + 1                 # the pose in progress
    assert job.result is not None and not job.result.ok


def test_leaving_leader_mode_cancels_the_search_before_the_hold(rig_factory):
    """J at 13:15:36: the search ran on behind _hold's bus reads, 1.7 s to the hold."""
    r = rig_factory(**{'planner.time_budget_s': 60.0})
    n = _held(r)
    calls = _search_poses(n)
    job = _searching(r)
    assert r.wait(lambda: len(calls) >= 10, 20.0)
    seen = []
    real = n._hold

    def hold(reason):
        seen.append(job.planner._cancelled)
        real(reason)
    n._hold = hold
    n._leave('test')
    assert seen == [True]
    job.thread.join(0.5)
    assert not job.thread.is_alive()


def test_space_cancels_the_search(rig_factory):
    r = rig_factory(**{'planner.time_budget_s': 60.0})
    n = _held(r)
    calls = _search_poses(n)
    job = _searching(r)
    assert r.wait(lambda: len(calls) >= 10, 20.0)
    r.stop.publish(Empty())
    assert r.wait(lambda: not job.thread.is_alive(), 1.0)


def test_the_configured_switch_interval_is_applied(rig_factory):
    """Read from the ACTIVE file: during a bus call the search thread must not hold the
    interpreter for 5 ms at a time, or a read outlives the SDK's reply timeout - and
    outside one the interval is the normal one, or the tick starves the search."""
    doc = yaml.safe_load((tan.PKG / 'config' / 'arm_safety_filter.yaml').read_text())
    want = doc['arm_action_server']['ros__parameters']['bus']['python_switch_interval_s']
    normal = sys.getswitchinterval()
    assert want < normal
    r = rig_factory()
    seen = []
    real = r.bus.read_pos

    def read_pos(sid):
        seen.append(sys.getswitchinterval())
        return real(sid)
    r.bus.read_pos = read_pos
    assert r.node._read_all() is not None
    assert seen and all(s == pytest.approx(want) for s in seen)
    assert sys.getswitchinterval() == pytest.approx(normal)


# ------------------------------------------------------------------ the read miss
def _miss(r, sid, why):
    real = r.bus.read_pos

    def read_pos(s):
        if s == sid:
            r.bus.last_error = why
            return None
        r.bus.last_error = None
        return real(s)
    r.bus.read_pos = read_pos


def test_one_read_miss_still_ends_leader_mode_and_names_servo_and_reason(rig_factory):
    """The rule itself is unchanged (SR-17 rule 7 and 8): the miss is only reported."""
    r = rig_factory()
    n = r.node
    r.leader_q = r.measured_q()
    tan._activate(r)
    _miss(r, 3, '[TxRxResult] There is no status packet!')
    assert r.wait(lambda: n._leader_state == aas.LEADER_OFF, 3.0)
    assert r.wait(lambda: 'no measured arm pose' in n.leader_text, 1.0), n.leader_text
    assert 'servo 3' in n.leader_text and 'no status packet' in n.leader_text
    assert n._read_misses[3] >= 1 and n._read_misses[1] == 0
    assert n._reads_done > 0


def test_a_bus_without_error_text_still_names_the_servo(rig_factory):
    r = rig_factory()
    n = r.node
    r.bus.pos.pop(4)
    assert n._read_all() is None
    assert 'servo 4' in n._read_failure() and 'no answer' in n._read_failure()


def test_read_failures_stay_bounded_and_with_their_own_call(rig_factory):
    """A dead servo and the 2 Hz joint-state reads: the text was appended to for good,
    and a timer's miss could fill in or blank another thread's reason."""
    r = rig_factory()
    n = r.node
    r.bus.pos.pop(4)
    assert r.wait(lambda: n._read_misses[4] >= 3, 5.0)      # the joint-state timer's misses
    assert n._read_failure() == ''                          # none of this thread's business
    for _ in range(3):
        assert n._read_all() is None
    assert n._read_failure().count('servo 4') == 1
    seen = []
    other = threading.Thread(target=lambda: seen.append((n._read_all(), n._read_failure())))
    r.bus.pos[4] = 1000
    r.bus.pos.pop(2)
    other.start()
    other.join(2.0)
    assert seen[0][0] is None and 'servo 2' in seen[0][1] and 'servo 4' not in seen[0][1]
    assert 'servo 4' in n._read_failure() and 'servo 2' not in n._read_failure()


def test_a_pending_space_is_reported_as_such_not_as_a_servo(rig_factory):
    r = rig_factory()
    n = r.node
    n._torque_off_pending = 'test'
    n._bus.raw.lost = None
    assert n._read_all() is None
    assert 'Space' in n._read_failure() and n._read_misses == {sid: 0 for sid in aas.IDS}


def test_joint_state_reads_count_too(rig_factory):
    """2 Hz outside leader mode: they show the miss rate no leader session survives."""
    r = rig_factory()
    n = r.node
    _miss(r, 2, 'x')
    assert r.wait(lambda: n._read_misses[2] >= 1, 3.0)


def test_withdraw_is_seen_with_the_search_running(rig_factory):
    r = rig_factory(**{'planner.time_budget_s': 60.0})
    n = _held(r)
    _searching(r)
    r.enable.publish(Bool(data=False))
    assert r.wait(lambda: n._leader_state == aas.LEADER_OFF, 2.0)

