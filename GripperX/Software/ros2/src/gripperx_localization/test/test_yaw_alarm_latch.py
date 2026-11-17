"""Latch on the yaw-rate divergence alarm (user decision 2026-11-05).

Thresholds come from the active config/localization.yaml (odom_divergence_monitor
section); the signals below are written in terms of those values. The D9 detection
rule is not tested here (check_yaw_divergence.py does that): only that the latch sits
on top of it and the live status stays independent.
"""
import json
import math
import os
import sys
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "src"))
sys.path.append(str(HERE.parents[2] / "gripperx_control" / "src"))
from gripperx_localization.odom_divergence_monitor import (  # noqa: E402
    YawRateDivergence as D, yaw_alarm_payload)

CFG = yaml.safe_load((HERE.parents[1] / "config" / "localization.yaml").read_text())
P = CFG["odom_divergence_monitor"]["ros__parameters"]
GYRO = 20.0                                   # deg/s, a turn
BAD = GYRO + P["yaw_abs_tol_dps"] + P["yaw_rel_gain"] * GYRO + 10.0   # well over the limit


def new():
    return D.from_params(P, t_start=0.0)


def run(d, t0, t1, enc, gyro=GYRO):
    t = t0
    while t < t1:
        d.push("enc", t, t, math.radians(enc))
        d.push("gyro", t, t, math.radians(gyro))
        r = d.evaluate(t)
        t += 0.02
    return r


@pytest.fixture
def episode():
    """Instance after warm-up, one divergence episode, and recovery past the hold."""
    d = new()
    t = P["startup_grace_sec"] + 2.0
    r = run(d, 0.0, t, GYRO)
    assert r["state"] == D.OK and not r["latched"]
    t2 = t + 3 * P["yaw_warn_sustain_sec"] + P["yaw_window_sec"]
    r = run(d, t, t2, BAD)
    return d, t2, r


def test_fires_once_and_latches(episode):
    d, t, r = episode
    assert r["latched"] and r["episodes"] == 1
    assert r["first_episode_t"] is not None and r["first_episode_t"] <= t


def test_stays_latched_while_live_status_returns_to_ok(episode):
    d, t, _ = episode
    t2 = t + 3 * P["alarm_hold_sec"] + 3 * P["yaw_window_sec"]
    r = run(d, t, t2, GYRO)
    assert r["state"] == D.OK                 # live, unlatched verdict
    assert r["latched"] and r["episodes"] == 1


def test_episode_count_increments_first_time_is_kept(episode):
    d, t, r1 = episode
    first = r1["first_episode_t"]
    t = t + 3 * P["alarm_hold_sec"] + 3 * P["yaw_window_sec"]
    r = run(d, t, t + 3 * P["yaw_warn_sustain_sec"] + P["yaw_window_sec"], GYRO)
    assert r["state"] == D.OK
    t += 3 * P["yaw_warn_sustain_sec"] + P["yaw_window_sec"]
    r = run(d, t, t + 3 * P["yaw_warn_sustain_sec"] + P["yaw_window_sec"], BAD)
    assert r["episodes"] == 2 and r["latched"]
    assert r["first_episode_t"] == first


def test_restart_is_a_new_unlatched_instance(episode):
    r = new().evaluate(0.0)
    assert r["latched"] is False and r["episodes"] == 0 and r["first_episode_t"] is None


def test_clock_step_does_not_clear_the_latch(episode):
    d, t, _ = episode
    d._reset()                                # what a clock step does
    assert d.latched is True and d.episodes == 1


def test_payload_is_json_with_live_state_beside_the_latch(episode):
    d, t, _ = episode
    r = run(d, t, t + 3 * P["alarm_hold_sec"] + 3 * P["yaw_window_sec"], GYRO)
    m = json.loads(yaw_alarm_payload(r, "2026-11-05 10:00:00 UTC"))
    assert m["latched"] is True and m["live_state"] == D.OK
    assert m["episodes"] == 1 and m["first_episode_utc"] == "2026-11-05 10:00:00 UTC"


# -- the latch survives everything the clock logic does -----------------------

def test_backward_clock_step_through_evaluate_keeps_the_latch(episode):
    d, t, _ = episode
    r = d.evaluate(t - 50.0)                  # node clock jumped back
    assert d.clock_resets == 1                # the step was seen and handled
    assert r["latched"] and r["episodes"] == 1 and r["first_episode_t"] is not None


def test_inconsistent_clock_jump_with_monotonic_keeps_the_latch(episode):
    d, t, _ = episode
    d.evaluate(t, mono=1000.0)
    r = d.evaluate(t + 30.0, mono=1000.1)     # node clock +30 s, monotonic +0.1 s
    assert d.clock_resets == 1
    assert r["latched"] and r["episodes"] == 1


def test_invalidate_clock_reset_keeps_the_latch(episode):
    d, t, _ = episode
    d.invalidate()                            # /clock reset to 0
    assert d.latched and d.episodes == 1
    r = run(d, 0.0, 1.0, GYRO)                # channel works again on the new timeline
    assert r["latched"] and r["episodes"] == 1
