"""The bt_navigator goal-acknowledgement window must be wide enough to be sampled.

WHAT THIS TEST IS FOR
    ``bt_navigator`` aborts a goal in flight with

        Timed out while waiting for action server to acknowledge goal request for
        compute_path_to_pose

    and the navigation dies mid-drive.  The budget behind that line is
    ``bt_navigator.default_server_timeout``.  It is NOT sampled by a callback: in
    ``nav2_behavior_tree::BtActionNode`` (nav2 1.3.12,
    ``bt_action_node.hpp`` L216-234 and L412-442) the node's own executor is spun
    for at most ``max_timeout_ = bt_loop_duration * 0.5`` per behaviour-tree tick,
    ticks are ``bt_loop_duration`` apart (``rclcpp::WallRate``), and at the first
    tick with ``elapsed >= server_timeout`` the node gives up WITHOUT spinning
    once more.  So the budget is a polling window, and two configuration values
    together decide whether an acknowledgement that has already arrived is ever
    read.

    This test re-runs that arithmetic against the ACTIVE configuration.  Full
    measurement and derivation: the internal acknowledgement-window record of
    2026-09-23, tracked internally and not part of this repository.

WHAT WOULD MAKE IT FAIL — and what would NOT, which matters just as much
    It reads ``gripperx_planning/config/nav2.yaml`` and duplicates neither of the
    two numbers, so it fails if somebody lowers ``default_server_timeout`` or
    raises ``bt_loop_duration`` — a change to the machine's configuration, not
    only to this file.  ``test_reproduces_the_observed_failure`` additionally
    pins the model itself: it asserts that the OLD pair (20 ms / 10 ms) loses an
    acknowledgement that arrives at 16 ms, which is the reported defect, so a
    model that silently stopped reproducing the bug fails here first.

    What it can NOT catch, stated rather than implied: it encodes UPSTREAM logic.
    A nav2 version bump that changes ``max_timeout_``, the tick order, or the
    missing final spin would leave this test green while the machine behaves
    differently.  The pinned version is 1.3.12 (the version on the robot,
    confirmed by ``dpkg`` 2026-09-21) and the source lines are named above so a
    reviewer can diff them.  It is also a DESK test: it proves the window can be
    sampled, not that any particular machine stays inside ``ACK_LATENCY_BUDGET_MS``.
"""

import os

import pytest

yaml = pytest.importorskip("yaml")

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, "..", ".."))
NAV2_YAML = os.path.join(_SRC, "gripperx_planning", "config", "nav2.yaml")

# --- the two measured terms the window has to cover -------------------------
# Both are results, not choices, and both are quoted in nav2.yaml next to the
# parameter.  Raising either here without a new measurement defeats the test.
#
# ACK_LATENCY_BUDGET_MS — the largest goal-acknowledgement latency still counted
#   as healthy.  Measured 2026-09-23 against the twin's REAL planner_server with
#   a continuously-spun client under 16-way CPU load, 1999 goals: p50 2.71,
#   p99 13.80, p99.9 20.52, max 69.95 ms.  Rounded UP from the measured MAX, not
#   from a percentile — the failure this test exists for is a tail event.
ACK_LATENCY_BUDGET_MS = 70.0
# TICK_LATENESS_BUDGET_MS — the largest behaviour-tree tick lateness still
#   counted as healthy.  NOT measured for bt_navigator.  It is the largest
#   scheduling stall this project has measured on the Pi at all
#   (controller_manager write side, 14-98 ms, recorded in the internal handover
#   document, tracked internally and not part of this repository), used here as
#   the order of magnitude the machine is known to produce.  TO-VERIFY on the
#   robot; until then this is the conservative stand-in, not a bt_navigator
#   measurement.
TICK_LATENESS_BUDGET_MS = 98.0


def _bt_navigator_params():
    if not os.path.isfile(NAV2_YAML):
        pytest.skip(f"active config not reachable from the source tree: {NAV2_YAML}")
    with open(NAV2_YAML, encoding="utf-8") as handle:
        return yaml.safe_load(handle)["bt_navigator"]["ros__parameters"]


def acknowledgement_is_read(server_timeout, bt_loop, ack_at_ms, late_tick=None, lateness=0.0):
    """Would BtActionNode read an acknowledgement that arrives at ``ack_at_ms``?

    Transcribed from nav2 1.3.12 ``bt_action_node.hpp``:
      * ``max_timeout_ = bt_loop_duration * 0.5`` (L69), integer milliseconds;
      * per tick, ``spin_until_future_complete(future, min(remaining, max_timeout_))``
        (L422-424) — an already-signalled future returns immediately, so any ack
        at or before the window's end is read;
      * ``remaining = server_timeout - elapsed``; ``remaining <= 0`` resets the
        handle and returns false with NO further spin (L414-420), after which
        ``tick()`` logs the timeout and returns FAILURE (L224-233).

    ``late_tick``/``lateness`` inject a single late tick, which is how a
    descheduled bt_navigator thread shows up.
    """
    window = int(bt_loop * 0.5)          # C++ chrono duration_cast truncates
    t = 0.0
    index = 0
    while True:
        elapsed = t
        remaining = server_timeout - elapsed
        if remaining <= 0:
            return False
        if ack_at_ms <= t + min(remaining, window):
            return True
        index += 1
        t += bt_loop + (lateness if index == late_tick else 0.0)


def test_reproduces_the_observed_failure():
    """The stock pair loses an ack that arrived 4 ms before its own deadline."""
    # Windows are [0,5] and [10,15] ms; at t=20 the node quits without spinning.
    assert acknowledgement_is_read(20, 10, 4.0)
    assert acknowledgement_is_read(20, 10, 12.0)
    assert not acknowledgement_is_read(20, 10, 16.0), (
        "the model no longer reproduces the reported defect — check it against "
        "bt_action_node.hpp before trusting anything else in this file"
    )
    # And a single tick 10 ms late loses an ack that has been in the queue since
    # 7 ms: the tick that should have read it at 10 ms arrives at 20 ms, finds
    # remaining <= 0, and returns without spinning.
    assert acknowledgement_is_read(20, 10, 7.0)
    assert not acknowledgement_is_read(20, 10, 7.0, late_tick=1, lateness=10.0)


def test_active_config_reads_a_healthy_acknowledgement():
    params = _bt_navigator_params()
    timeout = params["default_server_timeout"]
    bt_loop = params["bt_loop_duration"]
    step = 0.25
    misses = []
    ack = 0.0
    while ack <= ACK_LATENCY_BUDGET_MS:
        if not acknowledgement_is_read(timeout, bt_loop, ack):
            misses.append(ack)
        ack += step
    assert not misses, (
        f"default_server_timeout={timeout} / bt_loop_duration={bt_loop} lose an "
        f"acknowledgement arriving at {misses[:5]} ms, inside the measured "
        f"{ACK_LATENCY_BUDGET_MS} ms budget"
    )


def test_active_config_survives_one_late_tick():
    """A single descheduled tick must not convert a healthy ack into a FAILURE."""
    params = _bt_navigator_params()
    timeout = params["default_server_timeout"]
    bt_loop = params["bt_loop_duration"]
    failures = []
    for late_tick in range(1, 6):
        ack = 0.0
        while ack <= ACK_LATENCY_BUDGET_MS:
            if not acknowledgement_is_read(
                timeout, bt_loop, ack, late_tick=late_tick, lateness=TICK_LATENESS_BUDGET_MS
            ):
                failures.append((late_tick, ack))
                break
            ack += 0.5
    assert not failures, (
        f"default_server_timeout={timeout} fails on a single tick "
        f"{TICK_LATENESS_BUDGET_MS} ms late: {failures[:5]} (late_tick, ack_ms)"
    )


def test_timeout_stays_well_inside_one_replan_period():
    """The cost side: the budget also delays detection of a dead action server.

    The tree replans at 1 Hz (RateController hz=1.0 in
    navigate_to_pose_w_replanning_and_recovery.xml), and a planner that never
    acknowledges must not be able to consume a large share of that period before
    the tree hears about it.  30 % is the bracket the derivation states; it is a
    stated engineering choice, not a measurement.
    """
    params = _bt_navigator_params()
    assert params["default_server_timeout"] <= 300, (
        "a budget above 30 % of the 1 Hz replan period delays detection of a "
        "dead action server by more than the derivation accepted"
    )
