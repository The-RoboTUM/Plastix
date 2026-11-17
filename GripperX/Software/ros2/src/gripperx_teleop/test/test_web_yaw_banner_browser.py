"""Banner priority and size of the yaw-rate alarm, run on the real page (headless Firefox
against test/web_ui_demo.py, window 1366x682), because the order and the one-line
requirement cannot be seen in the source text. Skips when firefox/geckodriver are missing.

Order: latched alarm > every motion/safety warning that existed before the monitor (kept,
with a short tag) > monitor silent/unknown > info banners.
"""
import json
import os
import shutil
import subprocess
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_web_quit_dialog_browser import Browser, _free_port, _wait_port, HERE  # noqa: E402

UTC = '2026-11-05 14:03:22 UTC'
ALARM = {'yaw_alarm_state': 'alarm', 'yaw_alarm_episodes': 999, 'yaw_alarm_first_utc': UTC}
SILENT = {'yaw_alarm_state': 'silent'}
UNKNOWN = {'yaw_alarm_state': 'unknown', 'yaw_alarm_live_state': 'IMU_SILENT'}
OK = {'yaw_alarm_state': 'ok'}
BASE = {'mode': 'keyboard', 'rivals': [], 'manoeuvre_label': 'x'}

# banner chain branch -> (frame fields, expected tag)
CHAIN = {
    'RIVAL TELEOP': {'rivals': ['other-teleop-1234']},
    'E-STOP': {'estop_latched': True},
    'DRIVE FAULT': {'drive_fault_active': True, 'drive_fault_reason': 'ENCODER_MISSING',
                    'drive_fault_where': 'wheel b_rightwheel', 'drive_fault_detail': 'x'},
    'LEADER WALL': {'leader_wall_state': 'wall fault: x', 'leader_wall_age_sec': 0.1},
    'LEADER ALIGN': {'leader_align_state': 'refused: x', 'leader_align_age_sec': 0.1},
    'LEADER ALIGNING': {'leader_aligning': True},
    'LEADER MODE': {'leader_enabled': True},
    'CRAB JUMP': {'psi_jump_ago_sec': 0.5},
    'NO STEER FB': {'armed_without_feedback': True},
    'POSE LIMIT': {'pose_reachable': False},
}


@pytest.fixture(scope='module')
def browser():
    gecko = shutil.which('geckodriver')
    if not (gecko and (shutil.which('firefox') or os.path.exists('/snap/bin/firefox'))):
        pytest.skip('firefox/geckodriver not installed')
    web_port, gd_port = _free_port(), _free_port()
    demo = subprocess.Popen([sys.executable, os.path.join(HERE, 'web_ui_demo.py'),
                             '--port', str(web_port)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    gd = subprocess.Popen([gecko, '--port', str(gd_port)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    b = Browser(gd_port)
    try:
        if not (_wait_port(web_port) and _wait_port(gd_port)):
            pytest.skip('demo server or geckodriver did not start')
        try:
            s = b.req('POST', '/session', {'capabilities': {'alwaysMatch': {
                'browserName': 'firefox',
                'moz:firefoxOptions': {'args': ['-headless']}}}})
        except Exception as exc:
            pytest.skip(f'cannot start headless firefox: {exc}')
        b.sp = '/session/' + s['sessionId']
        b.req('POST', b.sp + '/window/rect', {'width': 1366, 'height': 682})
        b.req('POST', b.sp + '/url', {'url': f'http://127.0.0.1:{web_port}/'})
        time.sleep(1.5)
        yield b
        b.req('DELETE', b.sp)
    finally:
        gd.terminate()
        demo.terminate()


def render(b, fields):
    """Call the page's own renderBanner and read back (class, text, banner height)."""
    frame = dict(BASE, **fields)
    return b.js("renderBanner(arguments[0]); const el = document.getElementById('banner');"
                "return [el.className, document.getElementById('banner-text').textContent,"
                "el.offsetHeight, window.innerWidth];".replace(
                    'arguments[0]', json.dumps(frame)))


def one_line_height(b):
    return render(b, dict(OK, manoeuvre_label='x'))[2]


def test_window_is_the_target_size(browser):
    assert 1300 <= render(browser, OK)[3] <= 1366


@pytest.mark.parametrize('tag', sorted(CHAIN))
def test_motion_warning_beats_silent_and_unknown(browser, tag):
    for yaw in (SILENT, UNKNOWN):
        cls, text, _, _ = render(browser, dict(CHAIN[tag], **yaw))
        plain = render(browser, dict(CHAIN[tag], **OK))
        assert text.startswith(plain[1]), (tag, text)        # the old banner, unchanged
        assert text.endswith(' | YAW MONITOR SILENT' if yaw is SILENT
                             else ' | YAW MONITOR NOT MONITORING (IMU_SILENT)'), text
        assert cls == plain[0]


def test_silent_beats_info_banners_and_calm(browser):
    for fields in ({'mode': 'autonomous'}, {}):
        cls, text, _, _ = render(browser, dict(fields, **SILENT))
        assert cls.startswith('banner warn') and text.startswith('YAW MONITOR SILENT'), text
    assert render(browser, dict(OK))[0].startswith('banner calm')


def test_unknown_with_empty_live_state_has_no_empty_brackets(browser):
    text = render(browser, {'yaw_alarm_state': 'unknown'})[1]
    assert '()' not in text and 'NOT MONITORING' in text


@pytest.mark.parametrize('tag', ['', *sorted(CHAIN)])
def test_alarm_is_on_top_and_one_line_in_the_worst_case(browser, tag):
    one = one_line_height(browser)
    fields = dict(CHAIN.get(tag, {}), **ALARM)
    cls, text, height, _ = render(browser, fields)
    assert cls.startswith('banner alarm')
    assert text.startswith('YAW-RATE DIVERGENCE ALARM: first episode ' + UTC + ', 999x. Stop.')
    assert 'Use hardware switch S1, not the software stop.' in text
    assert (text.endswith(' | ' + tag) if tag else text.endswith('software stop.')), text
    assert height == one, f'{tag or "alarm alone"}: {height}px, one line is {one}px: {text}'


@pytest.mark.parametrize('live,suffix', [('persists', 'Now: persists.'),
                                         ('cleared', 'Now: cleared.'),
                                         ('unknown', 'Now: unknown.')])
@pytest.mark.parametrize('tag', ['', *sorted(CHAIN)])
def test_latched_alarm_live_suffix_stays_one_line(browser, tag, live, suffix):
    one = one_line_height(browser)
    fields = dict(CHAIN.get(tag, {}), yaw_alarm_live=live, **ALARM)
    cls, text, height, _ = render(browser, fields)
    assert cls.startswith('banner alarm')
    assert 'software stop. ' + suffix in text, text
    assert (text.endswith(' | ' + tag) if tag else text.endswith(suffix)), text
    assert height == one, f'{tag or "alarm alone"} {live}: {height}px vs {one}px: {text}'


def test_warning_is_yellow_and_disappears_when_cleared(browser):
    cls, text, height, _ = render(browser, {'yaw_alarm_state': 'warn'})
    assert cls.startswith('banner warn') and 'blink' not in cls
    assert text.startswith('YAW-RATE WARNING')
    assert height == one_line_height(browser), text
    cls, text, _, _ = render(browser, OK)
    assert cls.startswith('banner calm') and 'WARNING' not in text


@pytest.mark.parametrize('tag', sorted(CHAIN))
def test_warning_keeps_a_motion_warning_with_a_tag_and_alarm_wins(browser, tag):
    plain = render(browser, dict(CHAIN[tag], **OK))
    cls, text, _, _ = render(browser, dict(CHAIN[tag], yaw_alarm_state='warn'))
    assert text == plain[1] + ' | YAW WARNING' and cls == plain[0], text
    cls, text, _, _ = render(browser, dict(CHAIN[tag], yaw_alarm_state='alarm',
                                           yaw_alarm_live='persists', yaw_alarm_episodes=1))
    assert cls.startswith('banner alarm') and 'YAW-RATE DIVERGENCE ALARM' in text
