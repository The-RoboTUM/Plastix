"""The page shows the leader align state where the operator looks (no browser here).

Robot finding 2026-10-15: J was refused and the page showed nothing the operator saw.
The state text was only in the J key's tooltip (hover) and in the event log at the
page bottom - and the log skipped every event after a node restart, because the page
kept its last event id while the node's ids started at 1 again. No JS engine is
installed, so these are static checks on app.js / index.html; they fail if the
visible line, the banner case or the epoch reset is removed.
"""

import os
import re

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'gripperx_teleop', 'web')
JS = open(os.path.join(WEB, 'app.js')).read()
HTML = open(os.path.join(WEB, 'index.html')).read()


def test_align_state_has_a_visible_line_not_only_a_tooltip():
    assert 'id="leader-align"' in HTML
    assert re.search(r"text\(\s*note\s*,[^;]*f\.leader_align_state|"
                     r"const align = f\.leader_align_state", JS)
    assert re.search(r"\$\('leader-align'\)", JS)


def test_refused_or_aborted_raises_the_banner_for_a_while():
    block = JS[JS.index('function renderBanner'):JS.index('function renderState')]
    assert 'leaderAlignBad(f.leader_align_state)' in block
    assert 'leader_align_age_sec' in block and 'LEADER_NOTICE_SEC' in block
    # below the latched emergency stop, which keeps top priority (audit LA4 L4)
    assert block.index('f.estop_latched') < block.index('leaderAlignBad')
    assert re.search(r"function leaderAlignBad\(state\) \{\s*return "
                     r"/\^\(refused\|aborted\|unavailable\)/", JS)


def test_unavailable_leader_text_matches_the_page_alarm_prefix():
    # The node's "no leader process" text must start with a prefix leaderAlignBad
    # raises in the banner, or the notice is a grey line nobody reads (2026-10-16).
    from gripperx_teleop.web_teleop_node import LEADER_UNAVAILABLE
    prefix = LEADER_UNAVAILABLE.split(':', 1)[0]
    alarm = re.search(r"function leaderAlignBad\(state\) \{\s*return /\^\(([^)]*)\)/", JS)
    assert prefix in alarm.group(1).split('|')
    assert 'gripperx_desk.sh' in LEADER_UNAVAILABLE


def test_event_log_resets_on_a_new_node_epoch():
    block = JS[JS.index('function renderLog'):]
    assert 'epoch !== eventsEpoch' in block and 'lastEventId = 0' in block
    assert 'renderLog(f.events || [], f.events_epoch)' in JS
