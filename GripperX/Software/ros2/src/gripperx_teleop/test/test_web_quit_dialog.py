"""The browser UI must never use a native blocking dialog.

`window.confirm()` froze the page's event loop while it was open: the heartbeat
stopped, the node declared the link lost after client_timeout_sec, and the quit
that was sent on the first beat after OK was (correctly) dropped as having
"arrived after the link was lost". Quitting from the browser could not succeed
(desk, 2026-10-22). The confirmation is an in-page <dialog> now; this keeps a
native one from coming back.
"""
import os
import re

WEB = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'gripperx_teleop', 'web',
)


def _code(js):
    # Strip comments and string literals so prose such as "NOT window.confirm"
    # in a comment does not count, while a real call does.
    js = re.sub(r'/\*.*?\*/', '', js, flags=re.S)
    js = re.sub(r'//[^\n]*', '', js)
    return re.sub(r"'(?:\\.|[^'\\\n])*'|\"(?:\\.|[^\"\\\n])*\"", "''", js)


def test_no_native_blocking_dialog():
    js = _code(open(os.path.join(WEB, 'app.js')).read())
    calls = re.findall(r'(?<![\w$.])(?:window\.)?(?:confirm|alert|prompt)\s*\(', js)
    assert not calls, f'app.js calls a native blocking dialog: {calls}'


def test_quit_is_sent_only_from_the_confirm_button():
    js = open(os.path.join(WEB, 'app.js')).read()
    sends = re.findall(r"fire\('quit'\)", js)
    assert len(sends) == 1
    handler = re.search(
        r"\$\('btn-quit-confirm'\)\.addEventListener\('click',(.*?)\n  \}\);", js, re.S)
    assert handler and "fire('quit')" in handler.group(1)
    # the keycap and Shift+Q only open the dialog
    assert "cap.addEventListener('click', requestQuit)" in js


def test_keys_are_gated_while_dialog_is_open():
    js = open(os.path.join(WEB, 'app.js')).read()
    kd = js.index("window.addEventListener('keydown'")
    # The same gate covers the drive-fault clear dialog (SR-18 amendment 2026-11-09).
    gate = js.index('if (quitOpen() || clearOpen())', kd)
    assert gate < js.index('press(holdKey)', kd)
    assert gate < js.index('fire(ACTION_KEYS', kd)
