"""Run the real page in headless Firefox against test/web_ui_demo.py (no ROS, no robot).

The textual tests in test_web_quit_dialog.py cannot see focus, `inert` or key
routing. This one drives the page itself: with the quit dialog open, Tab must not
lead to a page keycap that Enter can fire (found by audit 2026-10-22: Enter on the
"home" keycap sent `home`), and the beats must keep flowing (the reason the dialog
is not a native confirm()). Skips when firefox/geckodriver are missing (Pi, CI).
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SHIFT, TAB, ENTER, ESC = '', '', '', ''


def _free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def _wait_port(port, secs=15):
    end = time.time() + secs
    while time.time() < end:
        try:
            socket.create_connection(('127.0.0.1', port), 0.3).close()
            return True
        except OSError:
            time.sleep(0.2)
    return False


class Browser:
    def __init__(self, port):
        self.p = port

    def req(self, method, path, body=None):
        r = urllib.request.Request(
            f'http://127.0.0.1:{self.p}{path}', method=method,
            data=None if body is None else json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        return json.load(urllib.request.urlopen(r, timeout=60))['value']

    def js(self, code):
        return self.req('POST', self.sp + '/execute/sync', {'script': code, 'args': []})

    def keys(self, *seq):
        acts = []
        for k in seq:
            acts += [{'type': 'keyDown', 'value': k}, {'type': 'keyUp', 'value': k}]
        self.req('POST', self.sp + '/actions',
                 {'actions': [{'type': 'key', 'id': 'kb', 'actions': acts}]})

    def chord(self, mod, key):
        self.req('POST', self.sp + '/actions', {'actions': [{'type': 'key', 'id': 'kb', 'actions': [
            {'type': 'keyDown', 'value': mod}, {'type': 'keyDown', 'value': key},
            {'type': 'keyUp', 'value': key}, {'type': 'keyUp', 'value': mod}]}]})

    def events(self, name=None):
        ev = self.js('return window.__b.flatMap(x => x[1].events)')
        return [e for e in ev if name is None or e.startswith(name + '|')]


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
        except Exception as exc:                      # no usable browser here
            pytest.skip(f'cannot start headless firefox: {exc}')
        b.sp = '/session/' + s['sessionId']
        b.url = f'http://127.0.0.1:{web_port}/'
        b.req('POST', b.sp + '/url', {'url': b.url})
        time.sleep(1.5)
        yield b
        b.req('DELETE', b.sp)
    finally:
        gd.terminate()
        demo.terminate()


def _fresh(b):
    """Dialog closed, nothing held, fetch recorder installed once, event log empty."""
    # No reload: a test must leave no state behind that the next one trips over,
    # so reset it here. (An earlier reload hid a leaked open dialog, whose
    # focus left the next Shift+Q a no-op -- not a Firefox quirk.)
    b.js("""if (document.getElementById('quit-dialog').open) closeQuit();
          releaseAll(); document.activeElement.blur();
          window.__b = window.__b || [];
          if (!window.__wrapped) { window.__wrapped = true; const f = window.fetch;
            window.fetch = (u, o) => { if (String(u).includes('input'))
              window.__b.push([performance.now(), JSON.parse(o.body)]);
              return f(u, o); }; }""")
    time.sleep(0.5)
    b.js('window.__b = [];')


def _open(b):
    b.chord(SHIFT, 'Q')
    time.sleep(0.4)
    assert b.js("return document.getElementById('quit-dialog').open")
    assert b.js('return document.activeElement.id') == 'btn-quit-confirm'


def test_tab_to_keycap_then_enter_sends_nothing(browser):
    _fresh(browser)
    _open(browser)
    for _ in range(40):                    # walk the whole tab ring, Enter on each stop
        browser.keys(TAB)
        browser.keys(ENTER)
        if not browser.js("return document.getElementById('quit-dialog').open"):
            break
    time.sleep(0.6)
    # Enter on the dialog's own buttons may close it (cancel/confirm) or stop
    # (its E-stop); nothing else - no keycap one-shot - may have been sent.
    sent = [e.split('|')[0] for e in browser.events()]
    assert set(sent) <= {'quit', 'estop'}, f'page keycap fired while dialog open: {sent}'
    assert not browser.js(
        "return document.activeElement.classList.contains('keycap')")


def test_enter_on_confirm_sends_exactly_one_quit(browser):
    _fresh(browser)
    _open(browser)
    browser.keys(ENTER, ENTER)
    time.sleep(0.8)
    assert not browser.js("return document.getElementById('quit-dialog').open")
    assert len(browser.events('quit')) == 1


def test_escape_closes_without_quit(browser):
    _fresh(browser)
    _open(browser)
    browser.keys(ESC)
    time.sleep(0.5)
    assert not browser.js("return document.getElementById('quit-dialog').open")
    assert browser.events('quit') == []


def test_beats_continue_and_drive_key_not_held(browser):
    _fresh(browser)
    _open(browser)
    browser.js('window.__b = [];')
    browser.keys('w')
    time.sleep(1.5)
    beats = browser.js('return window.__b')
    assert len(beats) >= 3, f'beats stalled while the dialog is open: {len(beats)}'
    assert all(b[1]['keys'] == [] for b in beats), 'w was held while the dialog was open'


def test_tab_and_shift_tab_stay_inside_the_dialog(browser):
    _fresh(browser)
    _open(browser)
    inside = ("return !!document.activeElement.closest('#quit-dialog') ? "
              "document.activeElement.id : 'OUTSIDE:' + document.activeElement.tagName")
    seen = []
    for _ in range(7):
        browser.keys(TAB)
        seen.append(browser.js(inside))
    for _ in range(5):
        browser.chord(SHIFT, TAB)
        seen.append(browser.js(inside))
    assert not [s for s in seen if s.startswith('OUTSIDE')], f'focus left the dialog: {seen}'
    # the ring is exactly Cancel -> Shut down -> E-stop, both ways round
    assert seen[:3] == ['btn-quit-estop', 'btn-quit-cancel', 'btn-quit-confirm'], seen
    assert set(seen) == {'btn-quit-cancel', 'btn-quit-confirm', 'btn-quit-estop'}
    assert browser.events() == []


def test_programmatic_click_and_pointerdown_are_refused_while_open(browser):
    """`inert` does not stop dispatchEvent, so this is what tests the refuseWhileQuit gates."""
    _fresh(browser)
    _open(browser)
    browser.js('window.__b = [];')
    browser.js("""
        Element.prototype.setPointerCapture = function () {};   // synthetic pointerId
        const targets = [...document.querySelectorAll('.keycap'),
                         document.getElementById('btn-takeover')];
        for (const el of targets) {
          el.dispatchEvent(new PointerEvent('pointerdown', {bubbles: true, cancelable: true, pointerId: 7}));
          el.dispatchEvent(new MouseEvent('click', {bubbles: true, cancelable: true}));
          el.dispatchEvent(new PointerEvent('pointerup', {bubbles: true, pointerId: 7}));
        }""")
    time.sleep(1.5)
    beats = browser.js('return window.__b')
    assert len(beats) >= 2, 'beats stalled'
    assert browser.js("return document.querySelectorAll('.keycap').length") > 10
    sent = [e.split('|')[0] for e in browser.events()]
    assert sent == [], f'events posted through the dialog: {sent}'
    assert all(b[1]['keys'] == [] for b in beats), 'a key was held while the dialog was open'
    assert not any(b[1].get('force') for b in beats), 'Take control went through the dialog'


def test_repeated_open_cancel_cycles_then_one_quit(browser):
    """One page load, no Tab: focus starts on Shut down, so Escape is the way out."""
    _fresh(browser)
    for _ in range(2):
        _open(browser)
        browser.keys(ESC)
        time.sleep(0.4)
        assert not browser.js("return document.getElementById('quit-dialog').open")
        assert not browser.js("return document.querySelector('main').inert"), 'page still inert'
        assert browser.events('quit') == []
    _open(browser)
    browser.keys(ENTER)
    time.sleep(0.8)
    assert not browser.js("return document.getElementById('quit-dialog').open")
    assert len(browser.events('quit')) == 1
    assert [e.split('|')[0] for e in browser.events()] == ['quit']


def test_enter_on_cancel_then_enter_on_confirm_after_tab(browser):
    """Enter on Cancel after a focus move, then a fresh dialog, in one page load."""
    _fresh(browser)
    _open(browser)
    browser.chord(SHIFT, TAB)                       # Shut down -> Cancel
    assert browser.js('return document.activeElement.id') == 'btn-quit-cancel'
    browser.keys(ENTER)
    time.sleep(0.5)
    assert not browser.js("return document.getElementById('quit-dialog').open")
    assert browser.events('quit') == []
    _open(browser)
    browser.keys(ENTER)
    time.sleep(0.8)
    assert not browser.js("return document.getElementById('quit-dialog').open")
    assert len(browser.events('quit')) == 1


def test_enter_on_focused_dialog_estop_sends_one_estop_dialog_stays(browser):
    _fresh(browser)
    _open(browser)
    browser.keys(TAB)                               # Shut down -> E-stop (ring wraps)
    assert browser.js('return document.activeElement.id') == 'btn-quit-estop'
    browser.keys(ENTER)
    time.sleep(0.8)
    assert [e.split('|')[0] for e in browser.events()] == ['estop']
    assert browser.js("return document.getElementById('quit-dialog').open")


def test_estop_buttons_one_estop_per_mouse_press_and_per_enter(browser):
    _fresh(browser)
    press = """const el = document.getElementById(arguments[0]);
        el.dispatchEvent(new PointerEvent('pointerdown', {bubbles: true, cancelable: true, pointerId: 3}));
        el.dispatchEvent(new MouseEvent('click', {bubbles: true, cancelable: true, detail: 1}));"""
    # main button, dialog closed: mouse pair, then keyboard Enter
    browser.req('POST', browser.sp + '/execute/sync', {'script': press, 'args': ['btn-estop']})
    time.sleep(0.6)
    assert len(browser.events('estop')) == 1, 'mouse press must stop exactly once'
    browser.js("document.getElementById('btn-estop').focus();")
    browser.keys(ENTER)
    time.sleep(0.6)
    assert len(browser.events('estop')) == 2, 'Enter on the focused main E-stop sends nothing'
    # dialog button: mouse pair
    _fresh(browser)
    _open(browser)
    browser.req('POST', browser.sp + '/execute/sync', {'script': press, 'args': ['btn-quit-estop']})
    time.sleep(0.6)
    assert len(browser.events('estop')) == 1, 'mouse press must stop exactly once'
    assert browser.js("return document.getElementById('quit-dialog').open")


def test_double_click_on_confirm_sends_one_quit(browser):
    _fresh(browser)
    _open(browser)
    browser.js("const b = document.getElementById('btn-quit-confirm'); b.click(); b.click();")
    time.sleep(0.8)
    assert len(browser.events('quit')) == 1
