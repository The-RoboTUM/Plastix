"""The web page shows the whole-drive fault (/swerve_controller/drive_fault, user
decision 2026-10-16): a standing red banner (below only the latched emergency stop,
no age limit, because the fault is latched until the stack restarts), a line on the
deck, and one alarm event on the edge. No JS engine is installed: the page checks are
static, the node checks run the node."""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_leader_keys import wnode  # noqa: E402,F401  (fixture; sets the ROS isolation)

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'gripperx_teleop', 'web')
JS = open(os.path.join(WEB, 'app.js')).read()
HTML = open(os.path.join(WEB, 'index.html')).read()


# ------------------------------------------------------------------ page (static)
def test_deck_line_exists_and_is_rendered_with_text_content():
    assert 'id="drive-fault"' in HTML
    assert re.search(r"text\(dnote, f\.drive_fault_active \? driveFaultMessage\(f\) : ''\)", JS)
    assert 'innerHTML' not in JS


def test_banner_is_a_standing_alarm_right_below_the_latched_stop():
    block = JS[JS.index('function renderBanner'):JS.index('function renderState')]
    order = [block.index(x) for x in ('f.estop_latched', 'f.drive_fault_active',
                                       'leaderWallBad(f.leader_wall_state)')]
    assert order == sorted(order), order
    branch = block[block.index('f.drive_fault_active'):
                   block.index('leaderWallBad(f.leader_wall_state)')]
    assert "level = 'alarm'" in branch
    assert 'age_sec' not in branch                          # no notice-period limit


# ------------------------------------------------------------------ node
def _fault(active):
    from gripperx_control_msgs.msg import DriveFault
    m = DriveFault()
    m.active = active
    m.reason = DriveFault.REASON_ENCODER_MISSING if active else DriveFault.REASON_NONE
    m.reason_text = 'ENCODER_MISSING' if active else 'NONE'
    m.wheel_index = 3 if active else -1
    m.wheel_name = 'b_rightwheel' if active else ''
    m.detail = 'reason=ENCODER_MISSING wheel=b_rightwheel: ...' if active else 'no drive fault'
    return m


def test_no_fault_shows_nothing(wnode):
    n, _ = wnode
    t = n._telemetry()
    assert t['drive_fault_active'] is False
    n._on_drive_fault(_fault(False))
    t = n._telemetry()
    assert t['drive_fault_active'] is False
    assert not any('DRIVE FAULT' in e['text'] for e in t['events'])


def test_fault_reaches_telemetry_and_raises_one_alarm(wnode):
    n, _ = wnode
    n._on_drive_fault(_fault(False))
    n._on_drive_fault(_fault(True))
    n._on_drive_fault(_fault(True))          # re-publication on (re)activation
    t = n._telemetry()
    assert t['drive_fault_active'] is True
    assert t['drive_fault_reason'] == 'ENCODER_MISSING'
    assert t['drive_fault_wheel'] == 'b_rightwheel'
    alarms = [e for e in t['events'] if 'DRIVE FAULT' in e['text']]
    assert len(alarms) == 1 and alarms[0]['level'] == 'alarm'
    assert t['drive_fault_where'] == 'wheel b_rightwheel'


# ------------------------------------------- FEEDBACK_LOST: wheel_index -1 (L2)
def test_page_never_builds_the_wheel_text_from_the_raw_index_or_name():
    """The page composes nothing from wheel_index/wheel_name: the node's text only."""
    fn = JS[JS.index('function driveFaultMessage'):]
    fn = fn[:fn.index('\n}\n')]
    assert 'f.drive_fault_where' in fn
    assert 'drive_fault_wheel' not in fn and 'wheel_index' not in fn
    # Every field has a non-'undefined' fallback.
    for field in ('f.drive_fault_reason', 'f.drive_fault_where', 'f.drive_fault_detail'):
        assert re.search(re.escape(field) + r" \|\| '", fn), field


def test_feedback_lost_names_the_link_not_a_wheel(wnode):
    from gripperx_control_msgs.msg import DriveFault
    from gripperx_teleop.web_teleop_node import drive_fault_where
    n, _ = wnode
    m = DriveFault()
    m.active = True
    m.reason = DriveFault.REASON_FEEDBACK_LOST
    m.reason_text = 'FEEDBACK_LOST'
    m.wheel_index = -1
    m.other_wheel_index = 2
    m.wheel_name = ''
    m.detail = 'reason=FEEDBACK_LOST: the wheel feedback LINK failed, not one encoder'
    assert drive_fault_where(m) == 'wheel feedback link, all wheels'
    n._on_drive_fault(m)
    t = n._telemetry()
    assert t['drive_fault_where'] == 'wheel feedback link, all wheels'
    alarm = [e for e in t['events'] if 'DRIVE FAULT' in e['text']][-1]['text']
    assert 'feedback link' in alarm
    for wrong in ('wheel ?', 'None', 'undefined', 'b_leftwheel', '-1'):
        assert wrong not in alarm, wrong


# ============================== operator clear (SR-18 amendment 2026-11-09, item 7)
def _stall_fault(reason=5, text='STALL_LATCHED_HELD'):
    from gripperx_control_msgs.msg import DriveFault
    m = DriveFault()
    m.active = True
    m.reason = reason
    m.reason_text = text
    m.wheel_index = 1
    m.wheel_name = 'f_rightwheel'
    m.other_wheel_index = -1
    m.fault_id = 42
    m.detail = f'reason={text} wheel=f_rightwheel: ...'
    return m


class _Future:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._response


class FakeClearClient:
    """Stands in for /swerve_controller/clear_drive_fault; answers at once."""

    def __init__(self, success=True, message='drive fault CLEARED: ...', ready=True):
        self.calls = []
        self.success, self.message, self.ready = success, message, ready

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        from gripperx_control_msgs.srv import ClearDriveFault
        self.calls.append((request.requester, request.expected_reason,
                           request.expected_fault_id))
        r = ClearDriveFault.Response()
        r.success = self.success
        r.reason = request.expected_reason
        r.message = self.message
        return _Future(r)


def test_page_has_the_button_and_a_cancel_default_dialog_with_the_warning():
    assert 'id="btn-drive-fault-clear"' in HTML and 'hidden>' in HTML.split(
        'id="btn-drive-fault-clear"')[1].split('\n')[0]
    dialog = HTML[HTML.index('<dialog id="clear-dialog"'):]
    dialog = dialog[:dialog.index('</dialog>')]
    assert ('A drive command that is still held will make the robot drive off '
            'immediately.') in dialog
    for field in ('clear-reason', 'clear-where', 'clear-other', 'clear-detail'):
        assert f'id="{field}"' in dialog, field
    fn = JS[JS.index('function requestClear'):]
    fn = fn[:fn.index('\n}\n')]
    assert "$('btn-clear-cancel').focus()" in fn        # Cancel is the default
    assert 'btn-clear-confirm' not in fn
    assert 'f.drive_fault_clearable' in fn               # never for a refused reason
    assert "fire('clear_drive_fault:' + clearExpected + ':' + clearFaultId)" in JS
    assert 'clearFaultId = f.drive_fault_id' in fn       # F-58: the identity shown


def test_button_is_shown_only_for_a_clearable_reason_and_restart_named_otherwise():
    block = JS[JS.index('function renderRole'):JS.index('function driveFaultMessage')]
    assert ("$('btn-drive-fault-clear').hidden = !(f.drive_fault_active && "
            "f.drive_fault_clearable)") in block
    fn = JS[JS.index('function driveFaultMessage'):]
    fn = fn[:fn.index('\n}\n')]
    assert 'stack restart required (Mode R)' in fn


def test_telemetry_offers_the_clear_only_for_the_four_stall_reasons(wnode):
    from gripperx_control_msgs.msg import DriveFault
    n, _ = wnode
    for reason, text, clearable in (
            (DriveFault.REASON_ENCODER_MISSING, 'ENCODER_MISSING', False),
            (DriveFault.REASON_STALL_RETRIP, 'STALL_RETRIP', True),
            (DriveFault.REASON_STALL_TRIP_COUNT, 'STALL_TRIP_COUNT', True),
            (DriveFault.REASON_STALL_MULTI_WHEEL, 'STALL_MULTI_WHEEL', True),
            (DriveFault.REASON_STALL_LATCHED_HELD, 'STALL_LATCHED_HELD', True),
            (DriveFault.REASON_FEEDBACK_LOST, 'FEEDBACK_LOST', False)):
        n._on_drive_fault(_stall_fault(reason, text))
        t = n._telemetry()
        assert t['drive_fault_clearable'] is clearable, text
        assert t['drive_fault_reason_code'] == reason


def test_confirm_calls_the_service_and_the_page_shows_its_answer(wnode):
    n, _ = wnode
    n._clear_client = FakeClearClient(message='drive fault CLEARED: reason=STALL_LATCHED_HELD')
    n._on_drive_fault(_stall_fault())
    assert n._telemetry()['drive_fault_id'] == 42
    n._on_client_input('s1', [], ['clear_drive_fault:5:42|7|5'], True)
    assert n._clear_client.calls == [('web_teleop', 5, 42)]
    t = n._telemetry()
    assert t['drive_fault_clear_result']['success'] is True
    assert 'CLEARED' in t['drive_fault_clear_result']['message']
    assert any('clear ACCEPTED' in e['text'] for e in t['events'])


def test_a_rejection_text_is_shown(wnode):
    n, _ = wnode
    text = ('reason=ENCODER_MISSING is not operator-clearable: stack restart required '
            '(Mode R or bringup service start); nothing changed')
    n._clear_client = FakeClearClient(success=False, message=text)
    n._on_drive_fault(_fault(True))
    n.request_drive_fault_clear(1)
    t = n._telemetry()
    assert t['drive_fault_clear_result'] == {'success': False, 'pending': False,
                                             'message': text}
    assert any(e['level'] == 'alarm' and 'REJECTED' in e['text'] and 'stack restart'
               in e['text'] for e in t['events'])


def test_an_observer_cannot_clear_and_no_service_means_not_sent(wnode):
    n, _ = wnode
    n._clear_client = FakeClearClient()
    n._on_drive_fault(_stall_fault())
    n._on_client_input('s2', [], ['clear_drive_fault:5:42|1|5'], False)
    assert n._clear_client.calls == []
    n._clear_client = FakeClearClient(ready=False)
    assert n.request_drive_fault_clear(5) is False
    assert n._clear_client.calls == []
    assert 'not available' in n._telemetry()['drive_fault_clear_result']['message']


def test_banner_goes_and_the_last_clear_stays_visible(wnode):
    from gripperx_control_msgs.msg import DriveFault
    n, _ = wnode
    t = n._telemetry()
    assert t['drive_fault_last_clear'] is None            # never faulted / never cleared
    n._on_drive_fault(_stall_fault())
    m = DriveFault()
    m.active = False
    m.reason_text = 'NONE'
    m.wheel_index = -1
    m.clear_count = 1
    m.last_clear_stamp.sec = 100
    m.last_cleared_reason = DriveFault.REASON_STALL_LATCHED_HELD
    m.last_clear_requester = 'web_teleop'
    n._on_drive_fault(m)
    t = n._telemetry()
    assert t['drive_fault_active'] is False and t['drive_fault_clearable'] is False
    assert t['drive_fault_last_clear'] == {'stamp_sec': 100.0, 'reason': 'STALL_LATCHED_HELD',
                                           'requester': 'web_teleop'}
    assert t['drive_fault_clear_count'] == 1
    assert any('drive fault cleared' in e['text'] and 'NOT resumed' in e['text']
               for e in t['events'])
    assert "f.drive_fault_last_clear" in JS


def test_a_malformed_clear_event_is_dropped_without_losing_the_batch(wnode):
    """F-58 (b): out-of-range or malformed values never reach the request, and the
    rest of the batch (a one-shot after it, the held keys) still applies."""
    from gripperx_teleop.web_teleop_node import parse_clear_event
    assert parse_clear_event('clear_drive_fault:5:42') == (5, 42)
    for bad in ('clear_drive_fault:5', 'clear_drive_fault:999:42', 'clear_drive_fault:-1:42',
                'clear_drive_fault:5:0', 'clear_drive_fault:5:4294967296',
                'clear_drive_fault:5:x', 'clear_drive_fault:5:42:1', 'clear_drive_fault:'):
        assert parse_clear_event(bad) is None, bad
    n, log = wnode
    n._clear_client = FakeClearClient()
    n._on_drive_fault(_stall_fault())
    n._on_client_input('s1', ['w'], ['clear_drive_fault:999:42|1|5',
                                     'clear_drive_fault:5:99999999999|2|5',
                                     'mode_keyboard|3|5'], True)
    assert n._clear_client.calls == []
    assert ('mode', 'keyboard') in log
    assert n._telemetry()['held'] == ['w']
    assert any('malformed' in e['text'] for e in n._telemetry()['events'])
