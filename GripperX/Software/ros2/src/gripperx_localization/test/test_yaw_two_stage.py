"""Two-stage yaw-rate divergence alarm (user decision D17, 2026-11-13).

WARNING = the D9 condition (1 s means beyond abs_tol + rel_gain*|gyro| for
yaw_warn_sustain_sec), not latched. ERROR = the mean over yaw_error_window_sec of the
excess |enc-gyro| - rel_gain*|gyro| beyond abs_tol, latched.

Every threshold comes from the ACTIVE config/localization.yaml; the signals are written
in terms of those values (nothing here restates 3 deg/s, 0.15, 1 s or 5 s).
"""
import json
import math
import sys
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "src"))
sys.path.append(str(HERE.parents[2] / "gripperx_control" / "src"))
from gripperx_localization.odom_divergence_monitor import (  # noqa: E402
    YawRateDivergence as D, yaw_alarm_payload)

P = yaml.safe_load((HERE.parents[1] / "config" / "localization.yaml").read_text())[
    "odom_divergence_monitor"]["ros__parameters"]
TOL, GAIN = P["yaw_abs_tol_dps"], P["yaw_rel_gain"]
WARN_S, WIN = P["yaw_warn_sustain_sec"], P["yaw_error_window_sec"]
DT = 0.02
CLEAN = P["startup_grace_sec"] + WIN + 2.0      # settle: grace passed, error window full


class Rig:
    """One instance driven at 50 Hz; enc/gyro rates are functions of t (deg/s)."""

    def __init__(self):
        self.d = D.from_params(P, t_start=0.0)
        self.t = 0.0
        self.log = []                       # (t, state, latched, warning)
        self.run(CLEAN, lambda t: 0.0, lambda t: 0.0)
        assert self.r["state"] == D.OK and not self.r["latched"]

    def run(self, secs, enc, gyro=lambda t: 0.0, skip=()):
        end = self.t + secs
        while self.t < end - 1e-9:
            if "enc" not in skip:
                self.d.push("enc", self.t, self.t, math.radians(enc(self.t)))
            if "gyro" not in skip:
                self.d.push("gyro", self.t, self.t, math.radians(gyro(self.t)))
            self.r = self.d.evaluate(self.t)
            self.log.append((self.t, self.r["state"], self.r["latched"], self.r["warning"]))
            self.t += DT
        return self.r

    def first(self, pred, after):
        return next((t for t, s, l, w in self.log if t >= after and pred(s, l, w)), None)


def test_config_has_the_two_stage_values():
    assert WARN_S > 0 and WIN > WARN_S and TOL > 0 and GAIN > 0


def test_transient_warns_without_latch_and_clears():
    # an excess of E for T seconds gives a window mean E*T/WIN: stay clearly under TOL
    exc = 10.0
    T = 0.9 * TOL * WIN / exc
    assert T > WARN_S                       # long enough to warn (checked, not assumed)
    g = Rig()
    t0 = g.t
    g.run(T, lambda t: exc)
    r = g.run(WIN + 3.0, lambda t: 0.0)
    assert g.first(lambda s, l, w: s == D.WARN and w, t0) is not None, "never warned"
    assert not r["latched"] and r["episodes"] == 0
    assert not any(l for _, _, l, _ in g.log)
    assert r["state"] == D.OK and r["warning"] is False      # cleared by itself, no hold
    assert r["warning_episodes"] == 1


def test_short_blip_below_sustain_does_not_even_warn():
    g = Rig()
    g.run(0.3 * WARN_S, lambda t: 5 * TOL)
    r = g.run(WIN + 2.0, lambda t: 0.0)
    assert r["warning_episodes"] == 0 and not r["latched"]


def test_large_excess_for_a_short_time_latches():
    # 15 deg/s for 1.2 s -> window mean 15*1.2/WIN
    exc, T = 15.0, 1.2
    assert exc * T / WIN > TOL, "scenario no longer above the threshold for these params"
    g = Rig()
    t0 = g.t
    g.run(T, lambda t: exc)
    r = g.run(WIN + 2.0, lambda t: 0.0)
    assert r["latched"] and r["episodes"] == 1
    assert g.first(lambda s, l, w: s == D.DIVERGED, t0) is not None


def test_sustained_small_excess_warns_first_then_latches():
    exc = TOL + 1.0                        # just above the tolerance base
    g = Rig()
    t0 = g.t
    g.run(2 * WIN, lambda t: exc)
    tw = g.first(lambda s, l, w: w, t0)
    tl = g.first(lambda s, l, w: l, t0)
    assert tw is not None and tl is not None
    assert WARN_S - 0.1 <= tw - t0 <= WARN_S + 1.0, tw - t0
    # window mean reaches TOL after ~TOL*WIN/exc seconds (+ the 1 s mean lag), never before
    assert TOL * WIN / exc - 0.2 <= tl - t0 <= WIN + 1.0, tl - t0
    assert tl > tw


def test_left_and_right_divergence_do_not_cancel():
    exc, half = 10.0, 1.5

    def enc(t):                           # signed excess alternating every `half` s
        return exc if int((t - T0) / half) % 2 == 0 else -exc

    g = Rig()
    T0 = g.t
    r = g.run(2 * WIN, enc)
    assert r["latched"], "alternating left/right divergence cancelled out"


def test_excess_inside_the_relative_term_is_quiet():
    g = Rig()
    gy = 60.0                              # a turn; encoder over-reports by 9 %: inside the limit
    r = g.run(3 * WIN, lambda t: 1.09 * gy, lambda t: gy)
    assert r["state"] == D.OK and r["warning_episodes"] == 0 and not r["latched"]


FAST = math.degrees(1.4)               # a fast spin, deg/s


def test_agreeing_fast_spin_alone_does_not_latch():
    g = Rig()
    r = g.run(3 * WIN, lambda t: 1.07 * FAST, lambda t: FAST)
    assert r["state"] == D.OK and not r["latched"] and r["warning_episodes"] == 0


def test_burst_inside_agreeing_fast_spin_latches_clipping_at_zero():
    """15 deg/s excess for 2.0 s on top of the 7 % over-report: the agreeing seconds of the
    spin must not bank negative credit (x clipped at 0), so the burst latches."""
    exc, T = 15.0, 2.0
    assert exc * T / WIN > TOL
    g = Rig()
    g.run(2 * WIN, lambda t: 1.07 * FAST, lambda t: FAST)
    assert not g.r["latched"]
    # x = exc above the relative term: enc = gyro*(1 + rel_gain) + exc
    g.run(T, lambda t: (1 + GAIN) * FAST + exc, lambda t: FAST)
    r = g.run(WIN, lambda t: 1.07 * FAST, lambda t: FAST)
    assert r["latched"] and r["episodes"] == 1


def test_gap_freezes_an_active_warning_and_error():
    g = Rig()
    g.run(2 * WIN, lambda t: TOL + 1.0)
    assert g.r["state"] == D.DIVERGED and g.r["latched"]
    n = g.r["episodes"]
    g.run(0.6 * P["imu_silence_timeout_sec"], lambda t: TOL + 1.0, skip=("gyro",))
    assert g.r["state"] in (D.DIVERGED, D.WARN), g.r["state"]       # frozen, not cleared
    assert g.r["latched"] and g.r["episodes"] == n
    # warning stage alone: a gap does not clear it either
    g = Rig()
    g.run(WARN_S + 0.8, lambda t: TOL + 1.0)
    assert g.r["state"] == D.WARN
    g.run(0.6 * P["imu_silence_timeout_sec"], lambda t: TOL + 1.0, skip=("gyro",))
    assert g.r["state"] == D.WARN and g.r["warning_episodes"] == 1


def test_error_is_not_judged_on_a_window_that_is_mostly_gap():
    """A gyro that delivers 0.3 s of every 0.6 s: the 1 s windows are unjudgeable (fill),
    so the error window stays empty: WARN or INTERMITTENT may show, nothing latches."""
    g = Rig()
    end = g.t + 2 * WIN
    while g.t < end:
        on = ((g.t - 0.0) % 0.6) < 0.3
        g.run(DT, lambda t: TOL + 1.0, skip=() if on else ("gyro",))
    assert not g.r["latched"]


def test_latched_alarm_reports_live_ok_afterwards_and_live_divergence_before():
    g = Rig()
    g.run(2 * WIN, lambda t: TOL + 1.0)
    live = json.loads(yaw_alarm_payload(g.r, "T"))
    assert live["latched"] is True and live["live_state"] in ("WARN", "DIVERGED")
    assert live["warning"] is True and live["warning_episodes"] >= 1
    g.run(P["alarm_hold_sec"] + 2 * WIN, lambda t: 0.0)
    done = json.loads(yaw_alarm_payload(g.r, "T"))
    assert done["latched"] is True and done["live_state"] == "OK"
    assert done["warning"] is False and done["episodes"] == 1


def test_payload_keeps_the_old_keys():
    g = Rig()
    m = json.loads(yaw_alarm_payload(g.r, None))
    for k in ("latched", "episodes", "first_episode_utc", "first_episode_node_t", "live_state"):
        assert k in m
    assert m["warning"] is False and m["warning_episodes"] == 0


def test_second_episode_after_recovery_is_counted_first_time_kept():
    g = Rig()
    g.run(2 * WIN, lambda t: TOL + 1.0)
    first = g.r["first_episode_t"]
    g.run(P["alarm_hold_sec"] + 2 * WIN, lambda t: 0.0)
    assert g.r["state"] == D.OK
    g.run(2 * WIN, lambda t: TOL + 1.0)
    assert g.r["episodes"] == 2 and g.r["first_episode_t"] == first
