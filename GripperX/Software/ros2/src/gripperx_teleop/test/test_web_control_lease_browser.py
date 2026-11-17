"""Real web_teleop_node + headless Firefox: a new page gets the seat without 'Take control'.

Field bug 2026-10-22: the page of the previous desk run kept the seat (stale
lease), the reloaded page observed forever. Private ROS domain, localhost only,
free ports; no robot. Skips when firefox/geckodriver/rclpy are missing.
"""
import os
import shutil
import socket
import subprocess
import sys
import time

import pytest

from test_web_quit_dialog_browser import Browser, _free_port, _wait_port

SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVES = "return /you are driving|lease held/.test(document.getElementById('chip-role').textContent)"
MAIN = "from gripperx_teleop.web_teleop_node import main; main()"


def _wait(b, want, secs):
    """Seconds until the chip says we hold the seat (or None)."""
    t0 = time.time()
    while time.time() - t0 < secs:
        try:
            if b.js(DRIVES) == want:
                return time.time() - t0
        except Exception:                 # script ran while the page was still loading
            pass
        time.sleep(0.1)
    return None


@pytest.fixture(scope='module')
def desk():
    gecko = shutil.which('geckodriver')
    if not (gecko and (shutil.which('firefox') or os.path.exists('/snap/bin/firefox'))):
        pytest.skip('firefox/geckodriver not installed')
    try:
        import rclpy  # noqa: F401
    except ImportError:
        pytest.skip('rclpy not available')
    web_port, gd_port = _free_port(), _free_port()
    env = dict(os.environ, ROS_DOMAIN_ID='195', ROS_LOCALHOST_ONLY='1',
               PYTHONPATH=SRC + os.pathsep + os.environ.get('PYTHONPATH', ''))
    node = subprocess.Popen(
        [sys.executable, '-c', MAIN, '--ros-args', '--params-file',
         os.path.join(SRC, 'config', 'keyboard_teleop.yaml'),
         '-p', f'web_port:={web_port}'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    gd = subprocess.Popen([gecko, '--port', str(gd_port)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    b = Browser(gd_port)
    try:
        if not (_wait_port(web_port) and _wait_port(gd_port)):
            pytest.skip('web_teleop_node or geckodriver did not start')
        try:
            s = b.req('POST', '/session', {'capabilities': {'alwaysMatch': {
                'browserName': 'firefox',
                'moz:firefoxOptions': {'args': ['-headless']}}}})
        except Exception as exc:
            pytest.skip(f'cannot start headless firefox: {exc}')
        b.sp = '/session/' + s['sessionId']
        b.url = f'http://127.0.0.1:{web_port}/'
        b.gecko = gecko
        yield b
        b.req('DELETE', b.sp)
    finally:
        gd.terminate()
        node.terminate()


def _load(b):
    b.req('POST', b.sp + '/url', {'url': 'about:blank'})
    b.req('POST', b.sp + '/url', {'url': b.url})


def test_new_page_after_a_page_that_went_silent_has_control_without_taking_it(desk):
    """The old page does not release (beacon stubbed out) and stops beating."""
    _load(desk)
    assert _wait(desk, True, 6) is not None, 'page A never got control'
    desk.js('navigator.sendBeacon = () => true; window.fetch = () => Promise.reject(new Error("dead"));')   # crashed tab
    time.sleep(0.3)
    _load(desk)                                                            # page B
    took = _wait(desk, True, 4)
    assert took is not None and took < 3.5, 'page B observes instead of taking the seat'


def test_reload_gets_control_at_once(desk):
    _load(desk)
    assert _wait(desk, True, 6) is not None
    desk.req('POST', desk.sp + '/refresh', {})
    took = _wait(desk, True, 1.2)       # well under the 2 s takeover interval
    assert took is not None, 'reloaded page waited for the stale lease instead of the release'


# -- page-level seat stealing (audit HIGH-1, 2026-10-22) ---------------------
# A stale lease reads as a free seat, so an observer page must not take it
# unless the operator is looking at it. Headless Firefox reports every window as
# focused and visible (checked in the tests below), so "not looking" is made by
# stubbing document.hasFocus, which is exactly what the page's gate reads.

@pytest.fixture()
def second(desk):
    gd_port = _free_port()
    gd = subprocess.Popen([desk.gecko, '--port', str(gd_port)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    b = Browser(gd_port)
    try:
        assert _wait_port(gd_port)
        s = b.req('POST', '/session', {'capabilities': {'alwaysMatch': {
            'browserName': 'firefox', 'moz:firefoxOptions': {'args': ['-headless']}}}})
        b.sp = '/session/' + s['sessionId']
        b.url = desk.url
        yield b
        b.req('DELETE', b.sp)
    finally:
        gd.terminate()


def _driver_a_and_observer_b(a, b, b_looked_at):
    _load(a)
    assert a.js('return [document.hasFocus(), document.visibilityState]') == [True, 'visible']
    assert _wait(a, True, 6) is not None, 'page A never got control'
    b.req('POST', b.sp + '/url', {'url': b.url})
    if not b_looked_at:
        b.js('document.hasFocus = () => false;')
    time.sleep(0.8)
    assert _wait(b, False, 1) is not None, 'B should observe while A drives'
    # A stalls (hiccup, throttled tab, sleeping laptop): no beat reaches the node, nothing is released.
    a.js('window.__fetch = window.fetch; window.fetch = () => Promise.reject(new Error("stalled"));')
    time.sleep(2.8)                                   # > takeover_sec


def test_unfocused_observer_does_not_steal_a_stalled_drivers_seat(desk, second):
    a, b = desk, second
    _driver_a_and_observer_b(a, b, b_looked_at=False)
    assert not b.js(DRIVES), 'unfocused page B took the seat from a stalled driver'
    a.js('window.fetch = window.__fetch; kick();')             # A resumes
    assert _wait(a, True, 2) is not None, 'driver A did not get its seat back'
    time.sleep(0.6)
    assert not b.js(DRIVES)


def test_focused_observer_may_claim_the_free_seat(desk, second):
    a, b = desk, second
    _driver_a_and_observer_b(a, b, b_looked_at=True)
    assert b.js('return [document.hasFocus(), document.visibilityState]') == [True, 'visible']
    assert _wait(b, True, 2) is not None, 'focused page B should claim the lapsed seat'
