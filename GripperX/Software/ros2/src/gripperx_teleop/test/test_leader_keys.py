"""Leader-streaming request ordering in keyboard_teleop_node (no hardware).

The real node runs with DDS discovery OFF (process-local); its publishers are
replaced by recorders so the ORDER of publishes across topics is observable.
"""

import os

# ISOLATION (audit round 3, 2026-10-08). Discovery OFF: the nodes of this test match
# each other inside this one process and discover NO other process - so runs started
# at the same time (other worktrees, other agents) cannot cross-talk, whatever domain
# they share. With LOCALHOST they did: duplicate publishers, duplicate goal responses.
# The deprecated ROS_LOCALHOST_ONLY=1 makes rcl IGNORE the range (it would undo OFF),
# and ROS_STATIC_PEERS would add peers back: both removed. The domain is the caller's
# (setdefault).
os.environ.setdefault('ROS_DOMAIN_ID', '221')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ.pop('ROS_STATIC_PEERS', None)

import pytest
import rclpy

from gripperx_geometry.constants import (
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as R,
)
from gripperx_teleop import keyboard_teleop_node as ktn
from gripperx_teleop.key_input import PRESS, parse_csi


class Rec:
    def __init__(self, name, log, subscribers=1):
        self.name, self.log = name, log
        self.subscribers = subscribers      # what get_subscription_count() reports

    def publish(self, msg):
        self.log.append((self.name, getattr(msg, 'data', None)))

    def get_subscription_count(self):
        return self.subscribers


class FakeClient:
    def __init__(self, log):
        self.log = log

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal):
        self.log.append(('pick_goal', None))

        class F:
            def add_done_callback(self, cb):
                pass
        return F()


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', f'a:={A}', '-p', f'b:={B}', '-p', f'wheel_radius:={R}'])
    n = ktn.KeyboardTeleopNode()
    log = []
    n._leader_pub = Rec('leader', log)
    n._align_pub = Rec('align', log)
    n._arm_pub = Rec('arm', log)
    n._mode_pub = Rec('mode', log)
    n._cmd_vel_pub = Rec('cmd_vel', log)
    n._steer_pub = Rec('steer', log)
    n._arm_stop_pub = Rec('arm_stop', log)
    n._pick_client = FakeClient(log)
    n._leader_enabled = True
    n._stop_epoch = 1                # as if the arm node's latched epoch had arrived
    n._j_debounce = 0.0              # J tests below press faster than a human; the
    # fallback-regime debounce has its own test
    yield n, log
    n.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('key,arm_event,arm_data', [
    ('o', 'arm', 'open_gripper'),
    ('i', 'arm', 'go_home'),
    ('p', 'pick_goal', None),
])
def test_arm_keys_disable_leader_first(node, key, arm_event, arm_data):
    n, log = node
    ktn._dispatch(n, key, PRESS, None)
    assert log[0] == ('leader', False)
    assert (arm_event, arm_data) in log
    assert log.index(('leader', False)) < log.index((arm_event, arm_data))
    assert n._leader_enabled is False


def test_space_disables_leader_first_and_commands_no_arm_motion(node):
    n, log = node
    ktn._dispatch(n, 'space', PRESS, None)
    assert log[0] == ('arm_stop', None)      # the arm node decides: limp if moving
    assert log[1] == ('leader', False)
    assert n._leader_enabled is False
    assert ('mode', 'keyboard') in log and ('cmd_vel', None) in log
    assert not [e for e in log if e[0] in ('arm', 'pick_goal')]


def test_j_sends_align_not_enable_then_cancels_and_e_is_unbound(node):
    # User decision 2026-10-13: J asks the leader node to align; it publishes the enable.
    n, log = node
    n._leader_enabled = False
    ktn._dispatch(n, 'j', PRESS, None)
    assert log == [('align', True)] and not n._leader_enabled and n._align_pending
    log.clear()
    ktn._dispatch(n, 'j', PRESS, None)            # second J while aligning: cancel
    assert log == [('leader', False), ('align', False)] and not n._align_pending
    assert parse_csi('101', 'u') is None      # 'e' no longer decoded
    assert parse_csi('106', 'u') == ('j', PRESS)


def test_j_after_alignment_done_switches_leader_mode_off(node):
    from std_msgs.msg import Bool, String
    n, log = node
    n._leader_enabled = False
    ktn._dispatch(n, 'j', PRESS, None)
    n._on_align_state(String(data='aligning: ...'))
    n._on_align_state(String(data='done: aligned, torque off confirmed; leader mode requested'))
    n._on_leader_enable(Bool(data=True))          # the leader node's enable, echoed
    assert n._leader_enabled and not n._align_pending
    log.clear()
    ktn._dispatch(n, 'j', PRESS, None)
    assert log == [('leader', False), ('align', False)] and not n._leader_enabled


def test_align_states_of_both_severities_in_a_row_do_not_raise(node):
    # Robot 2026-10-15: 'refused' (warn) then 'direct' (info) through one shared
    # logger call site raised ValueError in the spin thread and ended the teleop.
    from std_msgs.msg import String
    n, _ = node
    for data in ('refused: limb2 outside', 'direct: within the activation tolerance',
                 'aborted: Space', 'done: aligned', 'refused: again'):
        n._on_align_state(String(data=data))


def test_refused_alignment_clears_pending_so_next_j_retries(node):
    from std_msgs.msg import String
    n, log = node
    n._leader_enabled = False
    ktn._dispatch(n, 'j', PRESS, None)
    n._on_align_state(String(data='refused: no fresh follower pose'))
    assert not n._align_pending
    log.clear()
    ktn._dispatch(n, 'j', PRESS, None)
    assert log == [('align', True)]


@pytest.mark.parametrize('key', ['space', 'o', 'i', 'p'])
def test_space_and_arm_keys_cancel_a_running_alignment(node, key):
    n, log = node
    n._leader_enabled = False
    n._align_pending = True
    ktn._dispatch(n, key, PRESS, None)
    assert ('align', False) in log and not n._align_pending
    assert ('align', True) not in log
    if key == 'space':
        assert log[0] == ('arm_stop', None)
        assert not [e for e in log if e[0] in ('arm', 'pick_goal')]


class FakeHandle:
    def __init__(self, log):
        self.log = log

    def cancel_goal_async(self):
        self.log.append(('pick_cancel', None))


def test_space_cancels_a_running_pick_after_disabling_leader(node):
    n, log = node
    n._pick_handle = FakeHandle(log)
    ktn._dispatch(n, 'space', PRESS, None)
    assert ('pick_cancel', None) in log
    assert log.index(('arm_stop', None)) < log.index(('leader', False)) \
        < log.index(('pick_cancel', None))
    assert not [e for e in log if e[0] in ('arm', 'pick_goal')]


@pytest.mark.parametrize('key', ['w', 's', 'a', 'd', 'up', 'down', 'left', 'right', '0', '9'])
def test_drive_keys_blocked_while_leader_on(node, key):
    n, log = node                      # fixture: leader request ON
    n.press(key, PRESS)
    n._publish()
    assert not n._keys.held(key, ktn.time.monotonic(), 10.0)
    twists = [e for e in log if e[0] == 'cmd_vel']
    assert twists, 'a zero twist must still be published'
    assert not [e for e in log if e[0] == 'steer'], 'no steering command in leader mode'


def test_drive_key_moves_again_after_leader_off(node):
    n, log = node
    n._leader_enabled = False
    n.press('w', PRESS)
    assert n._keys.held('w', ktn.time.monotonic(), 10.0)


def test_leader_mode_publishes_exactly_zero_twist(node):
    n, _ = node
    sent = []
    n._cmd_vel_pub = type('R', (), {'publish': lambda self, m: sent.append(m)})()
    n.press('w', PRESS)
    n._publish()
    t = sent[-1]
    assert (t.linear.x, t.linear.y, t.angular.z) == (0.0, 0.0, 0.0)


# ── Web UI path ────────────────────────────────────────────────────────────

@pytest.fixture
def wnode():
    from gripperx_teleop import web_teleop_node as wtn
    rclpy.init(args=['--ros-args', '-p', f'a:={A}', '-p', f'b:={B}',
                     '-p', f'wheel_radius:={R}'])
    n = wtn.WebTeleopNode()
    log = []
    n._leader_pub = Rec('leader', log)
    n._align_pub = Rec('align', log)
    n._arm_pub = Rec('arm', log)
    n._mode_pub = Rec('mode', log)
    n._cmd_vel_pub = Rec('cmd_vel', log)
    n._steer_pub = Rec('steer', log)
    n._arm_stop_pub = Rec('arm_stop', log)
    n._pick_client = FakeClient(log)
    yield n, log
    n.destroy_node()
    rclpy.shutdown()


def test_web_j_sends_align_and_reaches_telemetry(wnode):
    from std_msgs.msg import String
    n, log = wnode
    n._on_client_input('s1', [], ['leader|1|5'], True)
    assert log == [('align', True)]
    t = n._telemetry()
    assert t['leader_enabled'] is False and t['leader_aligning'] is True
    n._on_align_state(String(data='aligning: leader to the follower pose'))
    assert n._telemetry()['leader_align_state'].startswith('aligning')
    n._on_client_input('s1', [], ['leader|2|5'], True)  # second J: cancel
    assert log[-2:] == [('leader', False), ('align', False)]
    assert n._telemetry()['leader_aligning'] is False


def test_web_estop_cancels_alignment(wnode):
    n, log = wnode
    n._align_pending = True
    n._on_client_input('s1', [], ['estop'], False)
    assert log[0] == ('arm_stop', None) and ('align', False) in log
    assert n._telemetry()['leader_aligning'] is False


def test_web_telemetry_follows_enable_topic_echo(wnode):
    from std_msgs.msg import Bool
    n, _ = wnode
    n._on_leader_enable(Bool(data=True))
    assert n._telemetry()['leader_enabled'] is True


def test_web_j_ignored_without_control(wnode):
    n, log = wnode
    n._on_client_input('s2', [], ['leader|1|5'], False)
    assert log == []


def test_web_estop_turns_leader_off_first(wnode):
    n, log = wnode
    n._leader_enabled = True
    n._on_client_input('s1', [], ['estop'], False)    # observers may stop too
    assert log[0] == ('arm_stop', None) and log[1] == ('leader', False)
    assert n._telemetry()['leader_enabled'] is False


def test_web_arm_action_disarms_leader_first(wnode):
    n, log = wnode
    n._leader_enabled = True
    n._on_client_input('s1', [], ['home|1|5'], True)
    assert log[0] == ('leader', False) and ('arm', 'go_home') in log


# ── Audit LA1: web one-shots, terminal auto-repeat, link loss ───────────────

def test_web_duplicate_event_id_is_dropped_so_a_lost_cancel_cannot_restart(wnode):
    """Probe T1: a J (cancel) the node processed, response lost, the same event
    re-sent: the duplicate id is dropped, the alignment stays cancelled."""
    from std_msgs.msg import String
    n, log = wnode
    n._on_client_input('s1', [], ['leader|1|5'], True)          # J: align
    n._on_align_state(String(data='aligning: x'))
    n._on_client_input('s1', [], ['leader|2|5'], True)          # J: cancel
    n._on_client_input('s1', [], ['leader|2|300'], True)        # resend of the same event
    assert [e for e in log if e[0] == 'align'] == [('align', True), ('align', False)]


@pytest.mark.parametrize('name', ['leader', 'pick', 'home', 'open_gripper'])
def test_web_stale_one_shot_is_dropped(wnode, name):
    n, log = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], [f'{name}|7|1500'], True)      # 1.5 s old > 1.0 s
    assert log == []


def test_web_estop_is_never_dropped_even_old_or_repeated(wnode):
    n, log = wnode
    n._on_client_input('s1', [], ['estop|3|5000'], False)
    n._on_client_input('s1', [], ['estop|3|5000'], False)
    assert [e for e in log if e[0] == 'arm_stop'] == [('arm_stop', None)] * 2


def test_web_same_id_from_another_page_is_not_a_duplicate(wnode):
    n, log = wnode
    n._on_client_input('s1', [], ['leader|1|5'], True)
    n._on_align_state  # noqa: B018
    n._align_pending = False
    n._on_client_input('s2', [], ['leader|1|5'], True)
    assert [e for e in log if e[0] == 'align'] == [('align', True), ('align', True)]


def test_terminal_fallback_auto_repeat_of_j_is_one_press(node):
    """Probe T2: without the kitty protocol a held J is PRESS bytes."""
    n, log = node
    n._leader_enabled = False
    n._keys.release_reporting = False
    n._j_debounce = n.get_parameter("leader_key_debounce_sec").value   # node default
    for _ in range(4):                                          # press + 3 repeats
        ktn._dispatch(n, 'j', PRESS, None)
    assert [e for e in log if e[0] == 'align'] == [('align', True)]
    n._last_j -= 1.0                                            # a deliberate later press
    ktn._dispatch(n, 'j', PRESS, None)
    assert [e for e in log if e[0] == 'align'][-1] == ('align', False)


def test_kitty_regime_j_is_not_debounced(node):
    n, log = node
    n._leader_enabled = False
    n._keys.release_reporting = True
    ktn._dispatch(n, 'j', PRESS, None)
    ktn._dispatch(n, 'j', PRESS, None)                          # quick cancel works
    assert [e for e in log if e[0] == 'align'] == [('align', True), ('align', False)]


def test_web_link_loss_cancels_alignment_and_leader_mode(wnode):
    """Probe T3."""
    import time
    n, log = wnode
    n._on_client_input('s1', [], ['leader|1|5'], True)
    assert n._align_pending
    n._client_seen = time.monotonic() - 100.0
    n._client_keys = frozenset({'w'})
    n._refresh_held_keys()
    assert ('align', False) in log and ('leader', False) in log
    assert not n._align_pending
    log.clear()
    n._refresh_held_keys()                                      # once, not every refresh
    assert ('align', False) not in log


@pytest.mark.parametrize('raw', ['leader', 'pick', 'home', 'open_gripper', 'leader|1',
                                 'leader|x|y'])
def test_web_one_shot_without_id_and_age_is_rejected(wnode, raw):
    """Coordinator decision 2026-10-13: no legacy form for one-shots."""
    n, log = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], [raw], True)
    assert log == []


def test_web_plain_estop_is_still_accepted(wnode):
    n, log = wnode
    n._on_client_input('s1', [], ['estop'], False)
    assert log[0] == ('arm_stop', None)


def test_web_one_shot_in_the_first_beat_after_link_loss_is_dropped(wnode):
    """Audit LA2 T4: a J that hung in TCP arrives after the node declared link loss,
    with a fresh-looking page-side age. Space in the same beat still acts."""
    import time
    n, log = wnode
    n._on_client_input('s1', [], [], True)                       # page alive
    n._client_seen = time.monotonic() - 5.0
    n._client_keys = frozenset()
    n._refresh_held_keys()                                       # link declared lost
    n._on_client_input('s1', [], ['leader|7|12', 'estop|8|12'], True)
    assert ('align', True) not in log and ('arm_stop', None) in log
    log.clear()
    n._on_client_input('s1', [], ['leader|9|12'], True)          # the next beat is normal
    assert log == [('align', True)]


# ── Align state on the page (robot finding 2026-10-15: a refusal was not seen) ──

def test_web_align_refusal_reaches_telemetry_banner_inputs_and_log(wnode):
    from std_msgs.msg import String
    n, log = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], ['leader|1|5'], True)
    refusal = 'refused: limb3: follower angle -0.006 rad not inside the band'
    n._on_align_state(String(data=refusal))
    t = n._telemetry()
    assert t['leader_align_state'] == refusal and t['leader_aligning'] is False
    assert t['leader_align_age_sec'] is not None and 0.0 <= t['leader_align_age_sec'] < 1.0
    assert t['events'][-1]['text'] == f'leader alignment: {refusal}'
    assert t['events'][-1]['level'] == 'alarm'


def test_web_events_carry_an_epoch_so_a_restarted_node_is_not_skipped(wnode):
    n, _ = wnode
    t = n._telemetry()
    assert isinstance(t['events_epoch'], str) and t['events_epoch']
    assert t['leader_align_age_sec'] is None          # nothing received yet


def test_web_replayed_align_state_at_start_is_no_fresh_notice(wnode):
    """Audit LA4 L4: the transient_local replay of an old refusal at node start shows
    in the line under the keys, but gets no banner age and no alarm event."""
    from std_msgs.msg import String
    n, _ = wnode
    old = 'refused: from yesterday'
    n._on_align_state(String(data=old))                 # first message, no J pending
    t = n._telemetry()
    assert t['leader_align_state'] == old
    assert t['leader_align_age_sec'] is None
    assert t['events'][-1]['level'] == 'info' and 'before this node started' in \
        t['events'][-1]['text']
    n._on_align_state(String(data='refused: new one'))  # a later state is fresh
    t = n._telemetry()
    assert t['leader_align_age_sec'] is not None and t['events'][-1]['level'] == 'alarm'


def test_web_first_align_state_after_own_j_is_fresh(wnode):
    from std_msgs.msg import String
    n, _ = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], ['leader|1|5'], True)  # J pending
    n._on_align_state(String(data='refused: x'))
    assert n._telemetry()['leader_align_age_sec'] is not None


# ── No leader process (2026-10-16: J went nowhere, only a later J said CANCELLED) ──

def test_web_j_without_leader_subscriber_is_not_sent_and_says_so(wnode):
    n, log = wnode
    n._leader_enabled = False
    n._align_pub.subscribers = 0
    n._on_client_input('s1', [], ['leader|1|5'], True)
    assert ('align', True) not in log and not n._align_pending
    t = n._telemetry()
    assert t['leader_align_state'].startswith('unavailable: leader process not running')
    assert 'gripperx_desk.sh' in t['leader_align_state']
    assert t['leader_align_age_sec'] is not None and t['leader_align_age_sec'] < 1.0
    assert t['events'][-1]['level'] == 'alarm'
    assert 'leader process not running' in t['events'][-1]['text']
    assert n._align_requested_at is None          # nothing to time out


def test_web_unanswered_j_is_cancelled_and_shown_after_the_timeout(wnode):
    import time
    n, log = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], ['leader|1|5'], True)
    assert log == [('align', True)] and n._align_pending
    n._check_leader_answer()                      # within the timeout: nothing
    assert log == [('align', True)] and n._align_pending
    n._align_requested_at = time.monotonic() - n._leader_answer_timeout - 0.1
    n._check_leader_answer()
    assert log[-2:] == [('leader', False), ('align', False)]   # cancelled, not left open
    assert not n._align_pending
    t = n._telemetry()
    assert t['leader_align_state'].startswith('unavailable: leader process not running')
    assert 'no answer' in t['leader_align_state']
    assert t['leader_align_age_sec'] is not None
    assert t['events'][-1]['level'] == 'alarm'
    log.clear()
    n._check_leader_answer()                      # once, not every tick
    assert log == []


def test_web_answered_j_does_not_time_out(wnode):
    import time
    from std_msgs.msg import String
    n, log = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], ['leader|1|5'], True)
    n._on_align_state(String(data='aligning: checking the leader is still'))
    assert n._align_requested_at is None          # the answer stopped the timer
    n._leader_answer_timeout = 0.0                # it would fire at once otherwise
    time.sleep(0.01)
    n._check_leader_answer()
    assert n._align_pending and ('align', False) not in log
    assert n._telemetry()['leader_align_state'].startswith('aligning')


def test_web_cancel_by_second_j_stops_the_answer_timer(wnode):
    n, log = wnode
    n._leader_enabled = False
    n._on_client_input('s1', [], ['leader|1|5'], True)
    n._on_client_input('s1', [], ['leader|2|5'], True)  # J again before any answer
    assert n._align_requested_at is None and not n._align_pending


def test_web_leader_starting_later_replaces_the_unavailable_line(wnode):
    from std_msgs.msg import String
    n, log = wnode
    n._leader_enabled = False
    n._align_pub.subscribers = 0
    n._on_client_input('s1', [], ['leader|1|5'], True)
    n._align_pub.subscribers = 1
    n._on_align_state(String(data='idle'))        # the new node's latched first state
    assert n._telemetry()['leader_align_state'] == 'idle'
    n._on_client_input('s1', [], ['leader|2|5'], True)
    assert log == [('align', True)] and n._align_pending


def test_web_unavailable_logs_on_one_severity_repeatedly(wnode):
    """rclpy pitfall: one call site must not change severity. Both paths, twice."""
    import time
    n, _ = wnode
    n._leader_enabled = False
    for _ in range(2):
        n._align_pub.subscribers = 0
        n._on_client_input('s1', [], [f'leader|{_ * 2 + 1}|5'], True)
        n._align_pub.subscribers = 1
        n._on_client_input('s1', [], [f'leader|{_ * 2 + 2}|5'], True)
        n._align_requested_at = time.monotonic() - 100.0
        n._check_leader_answer()


def test_web_answer_timeout_fires_from_the_node_timer_alone():
    """Audit L5: the tests above call _check_leader_answer() by hand, so they pass
    without the timer that runs it on the robot. Here only the executor runs it."""
    import time
    from gripperx_teleop import web_teleop_node as wtn
    rclpy.init(args=['--ros-args', '-p', f'a:={A}', '-p', f'b:={B}',
                     '-p', f'wheel_radius:={R}', '-p', 'leader_answer_timeout_sec:=0.3'])
    n = wtn.WebTeleopNode()
    try:
        log = []
        for attr, name in (('_leader_pub', 'leader'), ('_align_pub', 'align'),
                           ('_cmd_vel_pub', 'cmd_vel'), ('_steer_pub', 'steer'),
                           ('_mode_pub', 'mode'), ('_arm_pub', 'arm'),
                           ('_arm_stop_pub', 'arm_stop')):
            setattr(n, attr, Rec(name, log))
        n._leader_enabled = False
        n._on_client_input('s1', [], ['leader|1|5'], True)
        assert ('align', True) in log
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and n._align_requested_at is not None:
            n._on_client_input('s1', [], [], True)       # keep the page link alive
            rclpy.spin_once(n, timeout_sec=0.05)
        assert n._telemetry()['leader_align_state'].startswith(
            'unavailable: leader process not running')
        assert ('align', False) in log and not n._align_pending
    finally:
        n.destroy_node()
        rclpy.shutdown()
