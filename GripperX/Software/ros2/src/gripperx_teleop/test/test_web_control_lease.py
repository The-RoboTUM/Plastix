"""ControlRegistry lease semantics, with injected clock values (no sleeps).

Field bug 2026-10-22: a page from the previous desk run kept beating into the
void, claimed the seat of the next node with its old session id, and went
silent after a reload. holder still named it, so the new page decided to
observe forever. A lapsed lease must read as a free seat.
"""
from gripperx_teleop.web_server import ControlRegistry

TAKEOVER = 2.0


def _reg():
    return ControlRegistry(takeover_sec=TAKEOVER)


def test_fresh_registry_has_no_holder():
    assert _reg().holder_at(1000.0) is None


def test_live_holder_is_reported():
    reg = _reg()
    assert reg.claim('A', 100.0)
    assert reg.holder_at(100.0) == 'A'
    assert reg.holder_at(100.0 + TAKEOVER) == 'A'          # exactly at the limit: still live


def test_stale_holder_is_reported_as_free():
    reg = _reg()
    reg.claim('A', 100.0)
    assert reg.holder_at(100.0 + TAKEOVER + 0.01) is None


def test_refresh_keeps_the_lease_alive():
    reg = _reg()
    reg.claim('A', 100.0)
    for t in (101.5, 103.0, 104.5):                         # beats every 1.5 s < takeover
        assert reg.refresh('A', t)
        assert reg.holder_at(t) == 'A'


def test_new_session_claims_without_force_after_takeover():
    reg = _reg()
    reg.claim('A', 100.0)
    assert reg.claim('B', 100.0 + TAKEOVER + 0.01) is True
    assert reg.holder_at(103.0) == 'B'


def test_live_holder_cannot_be_taken_without_force():
    reg = _reg()
    reg.claim('A', 100.0)
    reg.refresh('A', 101.9)
    assert reg.claim('B', 103.0) is False                   # 1.1 s after A's last beat
    assert reg.holder_at(103.0) == 'A'
    assert reg.claim('B', 103.0, force=True) is True


def test_stale_holder_coming_back_after_another_claim_gets_false():
    reg = _reg()
    reg.claim('A', 100.0)
    assert reg.claim('B', 103.0)
    assert reg.refresh('A', 103.1) is False
    assert reg.holder_at(103.1) == 'B'


def test_stale_holder_coming_back_with_no_other_claim_regains_its_lease():
    reg = _reg()
    reg.claim('A', 100.0)
    assert reg.holder_at(110.0) is None
    assert reg.refresh('A', 110.0) is True
    assert reg.holder_at(110.0) == 'A'


def test_release_by_non_holder_is_ignored_and_by_holder_frees_at_once():
    reg = _reg()
    reg.claim('A', 100.0)
    reg.release('B')
    assert reg.holder_at(100.1) == 'A'
    reg.release('A')
    assert reg.holder_at(100.1) is None
    assert reg.claim('B', 100.1) is True
