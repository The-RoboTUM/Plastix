"""The page shows the latched yaw-rate divergence alarm (user decision 2026-11-05):
alarm / ok / silent state mapping, the banner text, and the node's telemetry. No JS
engine is installed: page checks are static, node checks run the node."""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_leader_keys import wnode  # noqa: E402,F401  (fixture; sets the ROS isolation)

from gripperx_teleop.web_teleop_node import YAW_ALARM_SILENT_SEC, yaw_alarm_view  # noqa: E402

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'gripperx_teleop', 'web')
JS = open(os.path.join(WEB, 'app.js')).read()


def _msg(latched, episodes=0, first=None, live='OK'):
    return json.dumps({'latched': latched, 'episodes': episodes,
                       'first_episode_utc': first, 'live_state': live})


def test_mapping_ok_alarm_silent():
    assert yaw_alarm_view(_msg(False), 0.5)['state'] == 'ok'
    a = yaw_alarm_view(_msg(True, 2, 'T0'), 0.5)
    assert (a['state'], a['episodes'], a['first_utc']) == ('alarm', 2, 'T0')
    assert yaw_alarm_view(None, None)['state'] == 'silent'
    assert yaw_alarm_view(_msg(False), YAW_ALARM_SILENT_SEC + 0.1)['state'] == 'silent'
    assert yaw_alarm_view('not json', 0.1)['state'] == 'silent'
    assert yaw_alarm_view('[1]', 0.1)['state'] == 'silent'


def test_warning_is_not_latched_and_clears():
    """D17: WARN is a live, non-latched state; it disappears as soon as the monitor says OK."""
    w = yaw_alarm_view(json.dumps({'latched': False, 'episodes': 0, 'live_state': 'WARN',
                                   'warning': True, 'warning_episodes': 4}), 0.5)
    assert (w['state'], w['warning_episodes'], w['live']) == ('warn', 4, '')
    assert yaw_alarm_view(_msg(False, live='OK'), 0.5)['state'] == 'ok'
    # a stale warning must not stay on the page: monitor silence is 'silent', not 'warn'
    assert yaw_alarm_view(json.dumps({'latched': False, 'live_state': 'WARN'}),
                          YAW_ALARM_SILENT_SEC + 0.1)['state'] == 'silent'
    # the older message without the new keys still reads as before
    assert yaw_alarm_view(_msg(False), 0.5)['state'] == 'ok'


def test_latched_alarm_reports_whether_the_divergence_persists():
    for live, want in (('WARN', 'persists'), ('DIVERGED', 'persists'),
                       ('DIVERGED_HELD', 'persists'), ('OK', 'cleared'),
                       ('IMU_SILENT', 'unknown'), ('WARMUP', 'unknown')):
        v = yaw_alarm_view(_msg(True, 1, 'T', live=live), 0.5)
        assert (v['state'], v['live']) == ('alarm', want), live
    # monitor gone quiet: still alarm, but the live part is unknown, never "cleared"
    assert yaw_alarm_view(_msg(True, 1, 'T', live='OK'), 600.0)['live'] == 'unknown'
    assert yaw_alarm_view(_msg(True, 1, 'T'), None)['live'] == 'unknown'


def test_banner_live_suffix_and_warning_static():
    suffix = JS[JS.index('function yawLiveSuffix'):JS.index('function yawWarnMessage')]
    assert "f.yaw_alarm_live === 'persists'" in suffix and 'Now: persists' in suffix
    assert "f.yaw_alarm_live === 'cleared'" in suffix and 'Now: cleared' in suffix
    assert "f.yaw_alarm_live === 'unknown'" in suffix and 'Now: unknown' in suffix
    comp = JS[JS.index('function composeBanner'):JS.index('function renderBanner(f)')]
    assert comp.index("f.yaw_alarm_state === 'alarm'") < comp.index("f.yaw_alarm_state === 'warn'") \
        < comp.index("f.yaw_alarm_state !== 'ok'")
    assert "level: 'warn', message: yawWarnMessage(f)" in comp     # yellow, not latched


def test_latched_alarm_survives_monitor_silence():
    assert yaw_alarm_view(_msg(True, 1, 'T'), 600.0)['state'] == 'alarm'


def test_blind_live_states_are_unknown_not_ok():
    for live in ('IMU_SILENT', 'BOTH_SILENT', 'NO_MOUNT', 'WARMUP', 'WHEEL_SILENT',
                 'NO_DATA', 'INTERMITTENT'):
        v = yaw_alarm_view(_msg(False, live=live), 0.5)
        assert (v['state'], v['live_state']) == ('unknown', live), live
    assert yaw_alarm_view(_msg(False, live='OK'), 0.5)['state'] == 'ok'
    assert yaw_alarm_view(_msg(False, live='DIVERGED_HELD'), 0.5)['state'] == 'ok'
    # a latched alarm always wins over a blind live state
    assert yaw_alarm_view(_msg(True, 1, 'T', live='IMU_SILENT'), 0.5)['state'] == 'alarm'


def test_banner_text_and_priority_static():
    chain = JS[JS.index('function renderBannerChain'):JS.index('function composeBanner')]
    # the monitor is not a branch of the chain: it can never outrank a motion warning
    assert 'yaw_alarm' not in chain
    comp = JS[JS.index('function composeBanner'):JS.index('function renderBanner(f)')]
    assert comp.index("f.yaw_alarm_state === 'alarm'") < comp.index("f.yaw_alarm_state !== 'ok'")
    assert "level === 'alarm' || level === 'warn'" in comp      # warn/alarm survive, info does not
    msg = JS[JS.index('function yawAlarmMessage'):JS.index('function yawUnknownLabel')]
    for need in ('YAW-RATE DIVERGENCE ALARM', 'first episode', 'Stop.',
                 'Restart the stack before moving again', 'hardware switch S1',
                 'not the software stop'):
        assert need in msg, need
    unk = JS[JS.index('function yawUnknownLabel'):JS.index('function leaderAlignBad')]
    assert 'f.yaw_alarm_live_state ?' in unk                    # no "NOT MONITORING ()"
    assert 'NOT MONITORING' in unk and 'SILENT' in unk


def test_node_telemetry(wnode):
    from std_msgs.msg import String
    n, _ = wnode
    assert n._telemetry()['yaw_alarm_state'] == 'silent'      # never heard: not OK
    n._on_yaw_alarm(String(data=_msg(False)))
    assert n._telemetry()['yaw_alarm_state'] == 'ok'
    n._on_yaw_alarm(String(data=_msg(False, live='IMU_SILENT')))
    t = n._telemetry()
    assert (t['yaw_alarm_state'], t['yaw_alarm_live_state']) == ('unknown', 'IMU_SILENT')
    n._on_yaw_alarm(String(data=_msg(True, 3, '2026-11-05 10:00:00 UTC')))
    t = n._telemetry()
    assert (t['yaw_alarm_state'], t['yaw_alarm_episodes']) == ('alarm', 3)
    assert t['yaw_alarm_first_utc'] == '2026-11-05 10:00:00 UTC'
    assert sum('YAW-RATE DIVERGENCE' in e['text'] for e in t['events']) == 1
    n._on_yaw_alarm(String(data=_msg(True, 4, '2026-11-05 10:00:00 UTC')))
    assert sum('YAW-RATE DIVERGENCE' in e['text'] for e in n._telemetry()['events']) == 1
