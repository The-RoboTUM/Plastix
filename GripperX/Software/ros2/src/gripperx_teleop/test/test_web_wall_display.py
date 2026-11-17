"""The web page shows the leader virtual wall state (/arm/leader/wall_state, user decision
2026-10-15) under the keys next to the leader-align line; wall faults are alarms, below
the latched emergency stop; a transient_local replay at node start is no fresh alarm.
No JS engine is installed: the page checks are static, the node checks run the node."""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_leader_keys import wnode  # noqa: E402,F401  (fixture; sets the ROS isolation)

WEB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'gripperx_teleop', 'web')
JS = open(os.path.join(WEB, 'app.js')).read()
HTML = open(os.path.join(WEB, 'index.html')).read()

FAULT = ('released: wall fault: servo 2: 6.0 V outside the band 6.5..8.4 V; torque off '
         'confirmed; re-arms at the next leader-mode activation')
UNCONFIRMED = 'released: Space (/arm/stop); torque-off NOT confirmed on servo(s) [2] - retrying'


# ------------------------------------------------------------------ page (static)
def test_wall_line_sits_right_after_the_align_line_and_uses_text_content():
    assert HTML.index('id="leader-wall"') > HTML.index('id="leader-align"')
    between = HTML[HTML.index('id="leader-align"'):HTML.index('id="leader-wall"')]
    assert '</section>' not in between                     # same panel, under the keys
    assert re.search(r"text\(wnote, leaderWallShown\(wall\) \? 'Leader wall: ' \+ wall : ''\)",
                     JS)
    assert 'innerHTML' not in JS


def test_shown_and_alarm_patterns():
    assert re.search(r"function leaderWallShown\(state\) \{\s*return "
                     r"/held:\|outside:\|\^released\|\^unavailable/", JS)
    assert re.search(r"function leaderWallBad\(state\) \{\s*return /wall fault\|NOT confirmed/",
                     JS)


def test_wall_banner_is_below_the_latched_emergency_stop():
    block = JS[JS.index('function renderBanner'):JS.index('function renderState')]
    # audit LW1 L6: e-stop, wall fault, align refused/aborted, then leader mode / aligning
    order = [block.index(x) for x in ('f.estop_latched', 'leaderWallBad(f.leader_wall_state)',
                                       'leaderAlignBad(f.leader_align_state)',
                                       'f.leader_aligning', 'f.leader_enabled')]
    assert order == sorted(order), order
    assert 'leader_wall_age_sec < LEADER_NOTICE_SEC' in block


# ------------------------------------------------------------------ node
def _s(text):
    from std_msgs.msg import String
    return String(data=text)


def test_fault_is_an_alarm_with_a_banner_age(wnode):
    from gripperx_teleop.web_teleop_node import wall_fault
    n, _ = wnode
    n._on_wall_state(_s('idle'))                              # replay, harmless
    n._on_wall_state(_s('armed; held: limb2 lower (tick 109)'))
    t = n._telemetry()
    assert t['leader_wall_state'].startswith('armed; held: limb2')
    assert not any('leader wall' in e['text'] for e in t['events'])   # no log spam
    for text in (FAULT, UNCONFIRMED):
        assert wall_fault(text)
        n._on_wall_state(_s(text))
        t = n._telemetry()
        assert t['events'][-1]['level'] == 'alarm' and text in t['events'][-1]['text']
        assert t['leader_wall_age_sec'] is not None


def test_ordinary_release_is_info(wnode):
    from gripperx_teleop.web_teleop_node import wall_fault
    n, _ = wnode
    n._on_wall_state(_s('idle'))
    text = ('released: Space (/arm/stop); torque off confirmed; re-arms at the next '
            'leader-mode activation')
    assert not wall_fault(text)
    n._on_wall_state(_s(text))
    assert n._telemetry()['events'][-1]['level'] == 'info'


def test_replayed_fault_at_start_is_no_fresh_alarm(wnode):
    n, _ = wnode
    n._on_wall_state(_s(FAULT))                               # first message: replay
    t = n._telemetry()
    assert t['leader_wall_state'] == FAULT                    # still shown under the keys
    assert t['leader_wall_age_sec'] is None                   # no banner
    assert t['events'][-1]['level'] == 'info' and 'before this node started' in \
        t['events'][-1]['text']
    n._on_wall_state(_s(FAULT))                               # a later one is fresh
    t = n._telemetry()
    assert t['events'][-1]['level'] == 'alarm' and t['leader_wall_age_sec'] is not None


def test_nothing_received_yet(wnode):
    t = wnode[0]._telemetry()
    assert t['leader_wall_state'] == '' and t['leader_wall_age_sec'] is None


def test_an_alignment_start_clears_the_wall_fault_banner(wnode):
    """Audit LW1b: a wall fault ranks above LEADER ALIGNING in the banner for
    LEADER_NOTICE_SEC; when an alignment starts, its notice age is cleared so the
    'Hands clear' banner shows. The fault text stays under the keys."""
    n, _ = wnode
    n._on_wall_state(_s('idle'))
    n._on_wall_state(_s(FAULT))
    assert n._telemetry()['leader_wall_age_sec'] is not None
    n._on_align_state(_s('refused: x'))                     # first align state (replay)
    n._on_align_state(_s('aligning: checking the leader is still (0.1 s), then torque on'))
    t = n._telemetry()
    assert t['leader_wall_age_sec'] is None and t['leader_wall_state'] == FAULT
    assert t['leader_aligning']
    # the page shows the wall banner only with an age
    block = JS[JS.index('function renderBanner'):JS.index('function renderState')]
    wall = block[block.index('leaderWallBad(f.leader_wall_state)'):]
    assert 'f.leader_wall_age_sec !== null' in wall[:200]
