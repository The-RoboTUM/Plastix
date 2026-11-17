#!/usr/bin/env python3
"""Cross-check two odometry sources against each other and say so when they part.

WHY THIS EXISTS. On 2026-08-21 a Nav2 run in the twin reported SUCCEEDED with the
robot 2.47 m off target and ZERO recovery behaviours. Nothing in the stack
objected, and nothing could have: every Nav2 tolerance compares the robot pose
against the goal in the SAME estimated frame, so an error in the estimate cancels
out of the comparison exactly. The goal checker is satisfied when the ESTIMATE
arrives, wherever the machine actually is.

Ground truth does not exist outside simulation, so no tolerance can be checked
against "the truth". But two odometry sources measuring the same motion by
physically different means ARE independent of each other, and that is checkable.
In the failure above, wheel odometry advanced 2.618 m while laser odometry
advanced 0.02 m over the same 5.4 s. This node would have said so.

WHAT IT DOES NOT DO. It does not decide which source is right — it cannot. It
reports that they disagree, with both numbers, and leaves the verdict to whoever
reads it. It changes no command and touches no control path.

THRESHOLD PROVENANCE, measured rather than picked: over a 2.0 s sliding window,
    healthy run   median 0.021 m   p95 0.063 m   max 0.087 m
    locked-up run median 0.369 m   p95 0.931 m   max 0.961 m
The default 0.25 m sits ~2.9x above the worst healthy value and well below the
median of the failure. Re-derive it if the window changes; the two do not scale
independently.

SECOND CHANNEL, YAW RATE (step 2.6). Wheel odometry's yaw rate (/wheel/odom
twist.angular.z, the odom0 yaw rate the EKF fuses) is compared with the gyro
(/imu/data/filtered, z about the body axis). Since step 2.4 the EKF trusts the gyro
over the encoder; if the IMU goes silent the EKF falls back to the encoder SILENTLY
(its own `odom0` yaw rate, which over-reports in-place rotation by 6-9 %), and a
stuck or slipping source is not announced either. This channel reports both, on
/diagnostics only: it publishes nothing else and no command path reads it.

THE RATE |rate| in the tolerance is the 1 s MEAN OF THE GYRO yaw rate, |mean(gyro)|:
    d = |mean(enc) - mean(gyro)|,  limit = abs_tol + rel_gain * |mean(gyro)|
The relative term exists because the encoder over-reports in proportion to the
rotation rate (clean maxima 2.7 deg/s at 0.5 rad/s, 8.4 deg/s at 1.4 rad/s). It is
scaled by the GYRO rather than the encoder or their larger: a gyro stuck at zero or
frozen low then gets the TIGHTEST tolerance (abs_tol alone) while the encoder
shows the real turn, and a runaway encoder cannot widen its own tolerance.
Evidence and its limits: claude/docs/YAW_AGREEMENT_REPORT_2026-10-22.md section
5.4 on branch Theo-yaw-report (never audited) and IMU_REPLAY_AB_REPORT_2026-10-30.md
section 9. Values are in config/localization.yaml and provisional.

TWO STAGES (user decision D17, 2026-11-13, amends D9 and the 2.8 abort rule; a
transient such as the operator repositioning the robot must not latch at once):
 - WARNING (state WARN): d > limit, sustained yaw_warn_sustain_sec (1 s). The D9
   condition. NOT latched: it clears as soon as the condition is false. Each warning
   episode is counted (warning_episodes) and logged at WARN level; no hold.
 - ERROR (state DIVERGED): per evaluation sample the EXCESS x = d - rel_gain *
   |mean(gyro)| (d is an absolute value, so left and right turns cannot cancel);
   ERROR when the time-weighted mean of x over the last yaw_error_window_sec (5 s)
   exceeds abs_tol. It is the same tolerance, applied to a mean instead of an
   instant: an excess of E deg/s for T s inside the window gives a mean of E*T/5, so
   15 deg/s for 1 s already latches. x is CLIPPED at zero (user decision D17
   amendment, 2026-11-13): x = max(0, d - rel_gain*|gyro|), so agreeing seconds of a
   fast spin cannot bank negative credit that masks a following real fault. The
   threshold stays a strict '>'. The first ERROR sets `latched`,
   nothing clears it but a node restart (user decision 2026-11-05, kept); ERROR
   episodes are the `episodes` / first_episode_* counters. The live state under a
   latch still follows the divergence (WARN / DIVERGED / OK).
 The error window needs min_window_fill of its span covered by judgeable samples; a
 window with less is not judged (GAPS below). Until then (after start, after a
 silence) no ERROR can be raised.

TIME. Window membership and silence use the ARRIVAL time on this node's clock
(use_sim_time honoured), never a sample's header stamp: a stale stamp must not
make a dead stream look alive. Stamps are used only to drop out-of-order or stale
samples. A backward step of the node clock (bag loop, /reset_simulation, chrony
step on the RTC-less Pi) restarts the channel instead of mixing two timelines.

GAPS. A window holding less than min_window_fill of data cannot be judged. The
warning sustain counter is then FROZEN, and so is the ERROR verdict: a gap neither
confirms nor erases an alarm in progress, and an active WARN / DIVERGED stays. When more than gap_report_fraction
of the last gap_report_window_sec could not be judged the state is INTERMITTENT
(WARN): a partly failing IMU or wheel feedback. EVERY alarm episode is counted and
logged when it happens, and the status stays at DIVERGED_HELD (ERROR) for
alarm_hold_sec after it, so the 0.5 s /diagnostics tick cannot miss one.

WHAT THIS CHANNEL CANNOT SEE.
 - A gyro stuck at zero (or any constant inside the tolerance) while the robot stands
   or turns slower than abs_tol: encoder and gyro both read ~0.
 - A gyro FROZEN at a level the plausibility filter passes (up to 3 rad/s), at
   standstill: the alarm fires within seconds, and then SELF-CLEARS: after
   gyro_bias_min_sec of standstill the bias estimator has learned the frozen value as
   "bias" and the conditioned output reads 0 again (localization.yaml, RESIDUAL RISK;
   shown in test/check_yaw_divergence.py part 2b). Only the logged episode and the
   held status remain.
 - A REAL rate above imu_max_abs_yaw_rate (3 rad/s) is rejected whole by
   localization_input_node, so the conditioned topic goes quiet and this channel
   reports IMU_SILENT ("NO GYRO") while the IMU is alive (INFERRED from the config
   note, not injected).
 - Which source is wrong, and a yaw-rate error below the tolerance (slow bias,
   ~8 % over-report at low rates).
"""
import json
import math
import time
from collections import deque
from datetime import datetime, timezone

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


class DivergenceWindow:
    """The whole decision, with no ROS in it, so it can be tested against a CSV."""

    OK = "OK"
    DIVERGED = "DIVERGED"
    INSUFFICIENT_MOTION = "INSUFFICIENT_MOTION"
    NO_DATA = "NO_DATA"

    def __init__(self, window_sec: float, min_travel_m: float, max_divergence_m: float):
        self.window_sec = float(window_sec)
        self.min_travel_m = float(min_travel_m)
        self.max_divergence_m = float(max_divergence_m)
        self._a = deque()  # (t, x, y) source A
        self._b = deque()

    @staticmethod
    def _travel(buf) -> float:
        if len(buf) < 2:
            return 0.0
        return ((buf[-1][1] - buf[0][1]) ** 2 + (buf[-1][2] - buf[0][2]) ** 2) ** 0.5

    def _trim(self, buf, now):
        while len(buf) > 1 and now - buf[0][0] > self.window_sec:
            buf.popleft()

    def push_a(self, t, x, y):
        self._a.append((t, x, y))
        self._trim(self._a, t)

    def push_b(self, t, x, y):
        self._b.append((t, x, y))
        self._trim(self._b, t)

    def evaluate(self):
        """-> (state, travel_a, travel_b, divergence)"""
        if len(self._a) < 2 or len(self._b) < 2:
            return self.NO_DATA, 0.0, 0.0, 0.0
        ta, tb = self._travel(self._a), self._travel(self._b)
        div = abs(ta - tb)
        # Below the floor the comparison is dominated by quantisation and says
        # nothing. Reporting OK there would be a false assurance, so it gets its
        # own state rather than being folded into OK.
        if max(ta, tb) < self.min_travel_m:
            return self.INSUFFICIENT_MOTION, ta, tb, div
        if div > self.max_divergence_m:
            return self.DIVERGED, ta, tb, div
        return self.OK, ta, tb, div


def body_z_row(qx, qy, qz, qw):
    """Third row of the rotation matrix of quaternion (base <- sensor frame).

    body yaw rate = row . (wx, wy, wz) of the sensor. The IMU is mounted upside
    down, so for the robot this is (0, 0, -1) and the sign matters.
    """
    return (2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx),
            1.0 - 2.0 * (qx * qx + qy * qy))


class _Stream:
    """Samples of one rate in the trailing window, with a running sum."""

    def __init__(self):
        self.buf = deque()  # (arrival t, rate deg/s)
        self.total = 0.0
        self.last_accept = None   # arrival time of the newest accepted sample
        self.last_stamp = None
        self.silent_since = None  # arrival time the stream was last seen alive
        self.dropped_stale = 0
        self.dropped_order = 0
        self.dropped_bad = 0

    def clear_window(self):
        self.buf.clear()
        self.total = 0.0

    def trim(self, now, window):
        while self.buf and now - self.buf[0][0] > window:
            self.total -= self.buf.popleft()[1]

    def mean(self):
        return self.total / len(self.buf)

    def fill(self, window):
        if len(self.buf) < 3:
            return 0.0
        return (self.buf[-1][0] - self.buf[0][0]) / window


class YawRateDivergence:
    """Encoder-vs-gyro yaw-rate decision with no ROS in it (rates in, deg/s inside).

    Times are seconds on ONE clock (the node's). `mono`, when given, is a monotonic
    clock read at the same moment: a node-clock jump that the monotonic clock did not
    make is a clock STEP (chrony on the RTC-less Pi, either direction), not a silence.
    """

    OK = "OK"
    WARN = "WARN"                # warning stage: 1 s means beyond the tolerance, sustained
    DIVERGED = "DIVERGED"        # error stage: 5 s mean excess beyond abs_tol
    DIVERGED_HELD = "DIVERGED_HELD"
    INTERMITTENT = "INTERMITTENT"
    WARMUP = "WARMUP"
    NO_DATA = "NO_DATA"
    IMU_SILENT = "IMU_SILENT"
    WHEEL_SILENT = "WHEEL_SILENT"
    BOTH_SILENT = "BOTH_SILENT"

    def __init__(self, *, window_sec, abs_tol_dps, rel_gain, warn_sustain_sec,
                 error_window_sec, imu_silence_timeout_sec, wheel_silence_timeout_sec, startup_grace_sec,
                 max_sample_age_sec, min_window_fill, alarm_hold_sec,
                 gap_report_window_sec, gap_report_fraction, clock_step_tolerance_sec,
                 t_start=None):
        self.window_sec = float(window_sec)
        self.abs_tol_dps = float(abs_tol_dps)
        self.rel_gain = float(rel_gain)
        self.warn_sustain_sec = float(warn_sustain_sec)
        self.error_window_sec = float(error_window_sec)
        self.timeout = {"gyro": float(imu_silence_timeout_sec),
                        "enc": float(wheel_silence_timeout_sec)}
        self.startup_grace_sec = float(startup_grace_sec)
        self.max_sample_age_sec = float(max_sample_age_sec)
        self.min_window_fill = float(min_window_fill)
        self.alarm_hold_sec = float(alarm_hold_sec)
        self.gap_window_sec = float(gap_report_window_sec)
        self.gap_fraction = float(gap_report_fraction)
        self.clock_step_tol = float(clock_step_tolerance_sec)
        self.t_start = None if t_start is None else float(t_start)  # first valid clock
        self.clock_resets = 0
        self.imu_returns = 0
        self.last_silence_sec = None   # length of the most recent IMU silence
        self.last_return_t = None
        self.last_return_first = False   # True: first appearance, not a return
        self.imu_first_appearances = 0
        self.episodes = 0              # ERROR (DIVERGED) episodes since node start
        self.warning_episodes = 0      # WARN episodes since node start
        self.last_episode_t = None
        self.last_episode_peak_dps = None
        # LATCH (user decision 2026-11-05): the first episode sets it, nothing clears it
        # but a new instance (node restart). Not in _reset(): a clock step must not
        # clear an alarm. The live state in evaluate() is unaffected by it.
        self.latched = False
        self.first_episode_t = None
        self._last_t = self.t_start
        self._last_mono = None
        self._reset()

    @classmethod
    def from_params(cls, p, t_start=None):
        """From the odom_divergence_monitor parameter dict (the one config)."""
        return cls(
            window_sec=p["yaw_window_sec"], abs_tol_dps=p["yaw_abs_tol_dps"],
            rel_gain=p["yaw_rel_gain"], warn_sustain_sec=p["yaw_warn_sustain_sec"],
            error_window_sec=p["yaw_error_window_sec"],
            imu_silence_timeout_sec=p["imu_silence_timeout_sec"],
            wheel_silence_timeout_sec=p["wheel_silence_timeout_sec"],
            startup_grace_sec=p["startup_grace_sec"],
            max_sample_age_sec=p["max_sample_age_sec"],
            min_window_fill=p["min_window_fill"], alarm_hold_sec=p["alarm_hold_sec"],
            gap_report_window_sec=p["gap_report_window_sec"],
            gap_report_fraction=p["gap_report_fraction"],
            clock_step_tolerance_sec=p["clock_step_tolerance_sec"], t_start=t_start)

    def _reset(self):
        self.s = {"enc": _Stream(), "gyro": _Stream()}
        self.last_return_t = None    # an event from the old timeline must not outlive a clock step
        self._acc = 0.0              # seconds the alarm condition has held (frozen in gaps)
        self._prev_cond = False
        self._warn_active = False
        self._err_active = False
        self._err_win = deque()      # (t, dt, x) judged samples of the error window
        self._err_sx = 0.0           # sum of x * dt
        self._err_sdt = 0.0          # sum of dt
        self._err_mean = None
        self._last_div_t = None
        self._filled_once = False
        self._last_step_t = None
        self._gaps = deque()         # (t, dt, evaluable)
        self._metric = (None, None, None, None)
        self._peak = 0.0

    # -- clock -------------------------------------------------------------

    def _clock(self, t, mono):
        """Track the clock. A backward step, or a jump the monotonic clock did not
        make, restarts the channel (counters and episode history are kept)."""
        if self.t_start is None:
            self.t_start = t
        elif self._last_t is not None:
            jumped = t < self._last_t
            if not jumped and mono is not None and self._last_mono is not None:
                jumped = abs((t - self._last_t) - (mono - self._last_mono)) > self.clock_step_tol
            if jumped:
                self.clock_resets += 1
                self.t_start = t
                self._reset()
        self._last_t = t
        if mono is not None:
            self._last_mono = mono

    def invalidate(self):
        """The node clock is not valid (sim time before the first /clock, or reset)."""
        self.t_start = None
        self._last_t = None
        self._last_mono = None
        self._reset()

    # -- input -------------------------------------------------------------

    def push(self, which, t, stamp, rate_rad_s, mono=None):
        """Offer one sample. t = arrival time on this node's clock; stamp = header
        stamp in seconds or None. Returns False if the sample was dropped."""
        self._clock(t, mono)
        st = self.s[which]
        if not math.isfinite(rate_rad_s):
            st.dropped_bad += 1
            return False
        if stamp is not None:
            if st.last_stamp is not None and stamp < st.last_stamp:
                st.dropped_order += 1
                return False
            if abs(t - stamp) > self.max_sample_age_sec:
                st.dropped_stale += 1
                return False
            st.last_stamp = stamp
        if st.last_accept is not None and t - st.last_accept > self.timeout[which]:
            self._on_return(which, st, t)
        elif st.last_accept is None and which == "gyro" and \
                t - self.t_start > self.startup_grace_sec:
            self._on_return(which, st, t)
        st.last_accept = t
        st.buf.append((t, math.degrees(rate_rad_s)))
        st.total += st.buf[-1][1]
        self._step(t)
        return True

    def _on_return(self, which, st, t):
        # The old window spans the gap: drop it, so the first verdict after the
        # return rests only on samples taken since.
        first = st.last_accept is None
        since = self.t_start if first else st.last_accept
        st.clear_window()
        self._restart_verdict()
        if which == "gyro":
            # A first appearance (IMU absent from boot, or the mount arrived late) is
            # not a return after a silence: counted apart, so imu_returns means returns.
            if first:
                self.imu_first_appearances += 1
            else:
                self.imu_returns += 1
            self.last_silence_sec = t - since
            self.last_return_t = t
            self.last_return_first = first

    def _restart_verdict(self):
        self._acc = 0.0
        self._prev_cond = False
        self._warn_active = False
        self._err_active = False
        self._err_win.clear()
        self._err_sx = self._err_sdt = 0.0
        self._err_mean = None
        self._filled_once = False
        self._last_step_t = None
        self._gaps.clear()

    # -- decision ----------------------------------------------------------

    def _silent(self, which, now):
        st = self.s[which]
        if st.last_accept is None:
            return now - self.t_start > self.startup_grace_sec
        return now - st.last_accept > self.timeout[which]

    def _step(self, now):
        for st in self.s.values():
            st.trim(now, self.window_sec)
        if self._silent("enc", now) or self._silent("gyro", now):
            self._restart_verdict()
            self._metric = (None, None, None, None)
            return
        enc, gyro = self.s["enc"], self.s["gyro"]
        if enc.last_accept is None or gyro.last_accept is None:
            return
        dt = 0.0 if self._last_step_t is None else min(max(now - self._last_step_t, 0.0),
                                                       self.window_sec)
        self._last_step_t = now
        ok = min(enc.fill(self.window_sec), gyro.fill(self.window_sec)) >= self.min_window_fill
        self._gaps.append((now, dt, ok))
        while self._gaps and now - self._gaps[0][0] > self.gap_window_sec:
            self._gaps.popleft()
        if not ok:
            # Too little data to judge this instant. FREEZE the warning sustain counter
            # and the ERROR verdict: a gap must neither confirm nor erase an alarm in
            # progress.
            self._metric = (None, None, None, None)
            self._prev_cond = False
            self._err_trim(now)
            return
        self._filled_once = True
        me, mg = enc.mean(), gyro.mean()
        diff = abs(me - mg)
        limit = self.abs_tol_dps + self.rel_gain * abs(mg)
        self._metric = (me, mg, diff, limit)
        # WARNING stage (D9 condition): the 1 s means beyond the tolerance, sustained.
        if diff > limit:
            if self._prev_cond:
                self._acc += dt
            self._prev_cond = True
        else:
            self._acc = 0.0
            self._prev_cond = False
        if self._acc >= self.warn_sustain_sec:
            if not self._warn_active:
                self._warn_active = True
                self.warning_episodes += 1
        elif self._acc == 0.0:
            self._warn_active = False
        # ERROR stage (D17): mean excess over a sliding window, latched.
        x = max(0.0, diff - self.rel_gain * abs(mg))
        self._err_win.append((now, dt, x))
        self._err_sx += x * dt
        self._err_sdt += dt
        self._err_trim(now)
        if self._err_sdt >= self.min_window_fill * self.error_window_sec:
            self._err_mean = self._err_sx / self._err_sdt
            if self._err_mean > self.abs_tol_dps:
                if not self._err_active:
                    self._err_active = True
                    self.episodes += 1
                    self.last_episode_t = now
                    self._peak = 0.0
                    if not self.latched:
                        self.latched = True
                        self.first_episode_t = now
                self._peak = max(self._peak, self._err_mean)
                self.last_episode_peak_dps = self._peak
                self._last_div_t = now
            else:
                self._err_active = False

    def _err_trim(self, now):
        w = self._err_win
        while w and now - w[0][0] > self.error_window_sec:
            _, dt, x = w.popleft()
            self._err_sx -= x * dt
            self._err_sdt -= dt
        if not w:
            self._err_sx = self._err_sdt = 0.0
            self._err_mean = None

    def gap_share(self):
        """(share of recent time with no evaluable window, history span) or (None, 0)."""
        span = sum(dt for _, dt, _ in self._gaps)
        if span < 0.5 * self.gap_window_sec:
            return None, span
        return sum(dt for _, dt, ok in self._gaps if not ok) / span, span

    def evaluate(self, now, mono=None):
        """-> dict(state, enc_dps, gyro_dps, diff_dps, limit_dps, ...). Call from a
        timer as well: silence is only visible with no sample arriving."""
        self._clock(now, mono)
        self._step(now)
        imu_silent, wheel_silent = self._silent("gyro", now), self._silent("enc", now)
        share, _ = self.gap_share()
        if imu_silent and wheel_silent:
            state = self.BOTH_SILENT
        elif imu_silent:
            state = self.IMU_SILENT
        elif wheel_silent:
            state = self.WHEEL_SILENT
        elif self.s["enc"].last_accept is None or self.s["gyro"].last_accept is None:
            state = self.NO_DATA
        elif self._err_active:
            state = self.DIVERGED
        elif self._warn_active:
            state = self.WARN
        elif self._last_div_t is not None and now - self._last_div_t <= self.alarm_hold_sec:
            state = self.DIVERGED_HELD
        elif share is not None and share > self.gap_fraction:
            state = self.INTERMITTENT
        elif not self._filled_once:
            state = self.WARMUP
        else:
            state = self.OK
        me, mg, diff, limit = self._metric
        return {
            "state": state, "enc_dps": me, "gyro_dps": mg, "diff_dps": diff,
            "limit_dps": limit, "gap_share": share,
            "imu_silent_sec": (now - (self.s["gyro"].last_accept
                                      if self.s["gyro"].last_accept is not None
                                      else self.t_start)) if imu_silent else 0.0,
            "imu_returns": self.imu_returns, "last_silence_sec": self.last_silence_sec,
            "last_return_t": self.last_return_t, "last_return_first": self.last_return_first,
            "imu_first_appearances": self.imu_first_appearances, "clock_resets": self.clock_resets,
            "warning": state == self.WARN or (self._warn_active and state == self.DIVERGED),
            "warning_episodes": self.warning_episodes, "err_mean_dps": self._err_mean,
            "episodes": self.episodes, "last_episode_t": self.last_episode_t,
            "latched": self.latched, "first_episode_t": self.first_episode_t,
            "last_episode_peak_dps": self.last_episode_peak_dps,
            "last_diverged_t": self._last_div_t,
            "dropped": {k: (v.dropped_stale, v.dropped_order, v.dropped_bad)
                        for k, v in self.s.items()},
        }


YAW_ALARM_TOPIC = "/localization/yaw_alarm"


def yaw_alarm_payload(r, first_wall_utc):
    """JSON text of the latched yaw-rate alarm for YAW_ALARM_TOPIC.

    `r` is YawRateDivergence.evaluate(); `live_state` is the unlatched verdict
    (OK / WARN / DIVERGED / ...), so a latched alarm over a live OK stays readable.
    `latched`, `episodes`, `first_episode_*` are the ERROR stage (D17); `warning`
    (live) and `warning_episodes` are the non-latched WARNING stage.
    """
    return json.dumps({
        "latched": bool(r["latched"]), "episodes": int(r["episodes"]),
        "first_episode_utc": first_wall_utc, "first_episode_node_t": r["first_episode_t"],
        "live_state": r["state"],
        "warning": bool(r["warning"]), "warning_episodes": int(r["warning_episodes"]),
    })


class OdomDivergenceMonitor(Node):
    def __init__(self, **node_kwargs):
        super().__init__("odom_divergence_monitor", **node_kwargs)
        self.declare_parameter("source_a_topic", "/wheel/odom")
        self.declare_parameter("source_b_topic", "/laser/odom")
        self.declare_parameter("window_sec", 2.0)
        self.declare_parameter("min_travel_m", 0.10)
        self.declare_parameter("max_divergence_m", 0.25)
        self.declare_parameter("publish_period_sec", 0.5)
        # x/y channel needs /laser/odom; the yaw channel does not. Launch sets it
        # from enable_laser_odometry so the node can run without the scan matcher.
        self.declare_parameter("xy_channel_enabled", True)
        # Yaw-rate channel (2.6). Values and their status: config/localization.yaml.
        self.declare_parameter("yaw_channel_enabled", True)
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("gyro_topic", "/imu/data/filtered")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("yaw_window_sec", 1.0)
        self.declare_parameter("yaw_abs_tol_dps", 3.0)
        self.declare_parameter("yaw_rel_gain", 0.15)
        self.declare_parameter("yaw_warn_sustain_sec", 1.0)
        self.declare_parameter("yaw_error_window_sec", 5.0)
        self.declare_parameter("imu_silence_timeout_sec", 1.0)
        self.declare_parameter("wheel_silence_timeout_sec", 1.0)
        self.declare_parameter("startup_grace_sec", 10.0)
        self.declare_parameter("max_sample_age_sec", 1.0)
        self.declare_parameter("min_window_fill", 0.8)
        self.declare_parameter("return_hold_sec", 10.0)
        self.declare_parameter("alarm_hold_sec", 5.0)
        self.declare_parameter("gap_report_window_sec", 10.0)
        self.declare_parameter("gap_report_fraction", 0.25)
        self.declare_parameter("clock_step_tolerance_sec", 0.5)

        a_topic = str(self.get_parameter("source_a_topic").value)
        b_topic = str(self.get_parameter("source_b_topic").value)
        self.window = DivergenceWindow(
            self.get_parameter("window_sec").value,
            self.get_parameter("min_travel_m").value,
            self.get_parameter("max_divergence_m").value,
        )
        self.a_topic, self.b_topic = a_topic, b_topic
        if self.get_parameter("xy_channel_enabled").value:
            self.create_subscription(Odometry, a_topic, self._on_a, 20)
            self.create_subscription(Odometry, b_topic, self._on_b, 20)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._tick)
        self._last_state = None
        self.xy_enabled = bool(self.get_parameter("xy_channel_enabled").value)
        self._init_yaw_channel()
        self.get_logger().info(
            f"comparing {a_topic} against {b_topic}: window "
            f"{self.window.window_sec:.1f} s, alarm above "
            f"{self.window.max_divergence_m:.2f} m, floor {self.window.min_travel_m:.2f} m"
        )

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_a(self, msg):
        self.window.push_a(self._stamp(msg), msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_b(self, msg):
        self.window.push_b(self._stamp(msg), msg.pose.pose.position.x, msg.pose.pose.position.y)


    # -- yaw-rate channel (2.6) -------------------------------------------

    def _init_yaw_channel(self):
        self.yaw_enabled = bool(self.get_parameter("yaw_channel_enabled").value)
        if not self.yaw_enabled:
            return
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        keys = ("yaw_window_sec", "yaw_abs_tol_dps", "yaw_rel_gain",
                "yaw_warn_sustain_sec", "yaw_error_window_sec",
                "imu_silence_timeout_sec", "wheel_silence_timeout_sec", "startup_grace_sec",
                "max_sample_age_sec", "min_window_fill", "alarm_hold_sec",
                "gap_report_window_sec", "gap_report_fraction", "clock_step_tolerance_sec")
        # t_start stays None until the first VALID clock: under use_sim_time the clock
        # reads 0 until /clock arrives, which must neither skip the start-up grace nor
        # look like an IMU that returned after 1.8e9 s.
        self.yaw = YawRateDivergence.from_params({k: g(k) for k in keys})
        self.return_hold_sec = float(g("return_hold_sec"))
        self.base_frame = str(g("base_frame"))
        self.gyro_topic = str(g("gyro_topic"))
        self.wheel_topic = str(g("wheel_odom_topic"))
        self.sim_time = bool(self.get_parameter("use_sim_time").value)
        self._mount_rows = {}      # frame_id -> body-z row
        self._mount_retry_t = -1e9
        self._gyro_without_mount = 0
        self._last_yaw_state = None
        self._last_returns = 0
        self._logged_episodes = 0
        self._logged_warnings = 0
        self._first_wall_utc = None
        # Latched alarm for the web teleop. TRANSIENT_LOCAL so a page that starts after the
        # episode still gets it, AND republished every tick: a subscriber can then tell
        # "monitor gone" (no message for seconds) from "monitor says OK".
        self.alarm_pub = self.create_publisher(
            String, YAW_ALARM_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # Known limitation: two monitors in one ROS domain share this topic and the page
        # flickers between their states. Run one.
        # The mount is static (URDF), so only /tf_static is read: a TransformListener
        # also takes /tf (~97 Hz) through a Python callback for the whole run, which
        # doubled this node's CPU for a value that never changes.
        self.tf_buffer = Buffer()
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, static_qos)
        self.create_subscription(Odometry, self.wheel_topic, self._on_wheel_yaw, 50)
        self.create_subscription(Imu, self.gyro_topic, self._on_gyro, 100)
        self.get_logger().info(
            f"yaw channel: {self.wheel_topic} twist.z against {self.gyro_topic}: alarm above "
            f"{self.yaw.abs_tol_dps:.1f} deg/s + {self.yaw.rel_gain:.2f}*|gyro| over "
            f"{self.yaw.window_sec:.1f} s, sustained {self.yaw.warn_sustain_sec:.1f} s = "
            f"WARNING; mean excess above {self.yaw.abs_tol_dps:.1f} deg/s over "
            f"{self.yaw.error_window_sec:.1f} s = latched ERROR; "
            f"silence after {self.yaw.timeout['gyro']:.1f} s (provisional, see localization.yaml)"
        )

    def _on_tf_static(self, msg):
        for tr in msg.transforms:
            self.buffer_set_static(tr)

    def buffer_set_static(self, tr):
        try:
            self.tf_buffer.set_transform_static(tr, "odom_divergence_monitor")
        except Exception as exc:  # a malformed transform must not kill the callback
            self.get_logger().warning(f"ignored static transform {tr.child_frame_id}: {exc}",
                                      throttle_duration_sec=30.0)

    def _now(self):
        """Node-clock seconds, or None while the clock is not valid (sim time before
        the first /clock, or a /clock reset to 0)."""
        ns = self.get_clock().now().nanoseconds
        if ns <= 0:
            if self.yaw.t_start is not None:
                self.yaw.invalidate()
            return None
        return ns * 1e-9

    def _mono(self):
        # Under sim time the node clock legitimately differs from wall time.
        return None if self.sim_time else time.monotonic()

    @staticmethod
    def _stamp_or_none(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return t if t > 0.0 else None

    def _on_wheel_yaw(self, msg):
        now = self._now()
        if now is None:
            return
        self.yaw.push("enc", now, self._stamp_or_none(msg), msg.twist.twist.angular.z,
                      self._mono())
        self._log_episode()

    def _mount_row(self, frame):
        row = self._mount_rows.get(frame)
        if row is not None:
            return row
        now = self._now()
        if now is None or 0.0 <= now - self._mount_retry_t < 1.0:
            return None
        self._mount_retry_t = now
        try:
            q = self.tf_buffer.lookup_transform(self.base_frame, frame, Time()).transform.rotation
        except TransformException:
            return None
        row = body_z_row(q.x, q.y, q.z, q.w)
        self._mount_rows[frame] = row
        self.get_logger().info(f"gyro frame {frame} -> {self.base_frame}: z row "
                               f"({row[0]:+.3f}, {row[1]:+.3f}, {row[2]:+.3f})")
        return row

    def _on_gyro(self, msg):
        row = self._mount_row(msg.header.frame_id)
        if row is None:
            self._gyro_without_mount += 1
            return
        now = self._now()
        if now is None:
            return
        w = msg.angular_velocity
        rate = row[0] * w.x + row[1] * w.y + row[2] * w.z
        self.yaw.push("gyro", now, self._stamp_or_none(msg), rate, self._mono())
        self._log_episode()

    def _log_episode(self):
        """Every alarm episode is logged when it happens, not when the 0.5 s tick looks."""
        y = self.yaw
        if y.latched and self._first_wall_utc is None:
            self._first_wall_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
        if y.episodes != self._logged_episodes:
            self._logged_episodes = y.episodes
            self.get_logger().error(
                f"yaw-rate divergence episode {y.episodes}: encoder and gyro disagree beyond "
                f"{y.abs_tol_dps:.1f} deg/s over {y.error_window_sec:.1f} s (mean excess "
                f"beyond {y.rel_gain:.2f}*|gyro|); latched until the stack restarts")
        if y.warning_episodes != self._logged_warnings:
            self._logged_warnings = y.warning_episodes
            self.get_logger().warning(
                f"yaw-rate divergence WARNING {y.warning_episodes}: encoder and gyro 1 s means "
                f"disagree beyond {y.abs_tol_dps:.1f} deg/s + {y.rel_gain:.2f}*|gyro| for "
                f"{y.warn_sustain_sec:.1f} s (not latched)")

    def _yaw_status(self):
        D = YawRateDivergence
        now = self._now()
        if now is None:
            status = DiagnosticStatus()
            status.level = DiagnosticStatus.WARN
            status.name = "localization: yaw-rate cross-check"
            status.hardware_id = "gripperx"
            status.message = "waiting for a valid clock (sim time before the first /clock)"
            status.values = [KeyValue(key="state", value="NO_CLOCK")]
            return status
        r = self.yaw.evaluate(now, self._mono())
        self._log_episode()
        self.alarm_pub.publish(String(data=yaw_alarm_payload(r, self._first_wall_utc)))
        state = r["state"]
        returned = (r["last_return_t"] is not None
                    and 0.0 <= now - r["last_return_t"] <= self.return_hold_sec)
        if state in (D.IMU_SILENT, D.BOTH_SILENT) and self._gyro_without_mount > 0 \
                and not self._mount_rows:
            state = "NO_MOUNT"
        level = {D.OK: DiagnosticStatus.OK, D.WARMUP: DiagnosticStatus.OK,
                 D.DIVERGED: DiagnosticStatus.ERROR,
                 D.DIVERGED_HELD: DiagnosticStatus.ERROR}.get(state, DiagnosticStatus.WARN)
        if returned and level == DiagnosticStatus.OK:
            level = DiagnosticStatus.WARN   # the whole hold, whatever the sub-state
        t = self.yaw

        def fmt(v):  # every message below is built eagerly, so None must not raise
            return "n/a" if v is None else f"{v:+.2f}"

        since_div = "n/a" if r["last_diverged_t"] is None else f"{now - r['last_diverged_t']:.1f}"
        gap = r["gap_share"]
        message = {
            D.OK: "encoder and gyro yaw rate agree",
            D.WARMUP: "filling the averaging window",
            D.NO_DATA: "waiting for both yaw-rate sources",
            D.WARN: (
                f"yaw-rate WARNING: encoder {fmt(r['enc_dps'])} deg/s, gyro "
                f"{fmt(r['gyro_dps'])} deg/s, difference {fmt(r['diff_dps'])} > "
                f"{fmt(r['limit_dps'])} deg/s (clears by itself)"),
            D.DIVERGED: (
                f"YAW RATE DISAGREES: mean excess {fmt(r['err_mean_dps'])} deg/s over "
                f"{t.error_window_sec:.0f} s > {t.abs_tol_dps:.1f} (encoder "
                f"{fmt(r['enc_dps'])}, gyro {fmt(r['gyro_dps'])} deg/s) - one of them is "
                f"not measuring the rotation"),
            D.DIVERGED_HELD: (
                f"yaw-rate divergence episode {r['episodes']} ended {since_div} s ago "
                f"(peak mean excess {fmt(r['last_episode_peak_dps'])} deg/s); "
                f"held for {t.alarm_hold_sec:.0f} s so it is not missed"),
            D.INTERMITTENT: (
                f"one yaw-rate source delivers in bursts: no judgeable {t.window_sec:.1f} s "
                f"window for {0 if gap is None else 100 * gap:.0f} % of the last "
                f"{t.gap_window_sec:.0f} s (partly failing IMU or wheel feedback?)"),
            D.IMU_SILENT: (
                f"NO GYRO on {self.gyro_topic} for {r['imu_silent_sec']:.1f} s (driver down, or "
                "every sample rejected by the plausibility filter, e.g. a real rate above "
                "imu_max_abs_yaw_rate) - the EKF is using the encoder yaw rate alone "
                "(over-reports in-place rotation by 6-9 %)"),
            D.WHEEL_SILENT: f"NO WHEEL ODOMETRY on {self.wheel_topic}",
            D.BOTH_SILENT: "NEITHER yaw-rate source is publishing",
            "NO_MOUNT": (f"gyro samples arrive but {self.base_frame} <- their frame is not in "
                         "TF yet, so the channel cannot sign them"),
        }[state]
        if returned and state in (D.OK, D.WARMUP):
            what = (f"gyro APPEARED (first sample, {r['last_silence_sec']:.1f} s after start)"
                    if r["last_return_first"]
                    else f"gyro RETURNED after {r['last_silence_sec']:.1f} s of silence")
            message = what + ("; filling the window" if state == D.WARMUP
                              else "; agreement shown is since then")
        status = DiagnosticStatus()
        status.level = level
        status.name = "localization: yaw-rate cross-check"
        status.hardware_id = "gripperx"
        status.message = message

        def num(v):
            return "" if v is None else f"{v:.3f}"

        status.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="encoder_dps", value=num(r["enc_dps"])),
            KeyValue(key="gyro_dps", value=num(r["gyro_dps"])),
            KeyValue(key="difference_dps", value=num(r["diff_dps"])),
            KeyValue(key="limit_dps", value=num(r["limit_dps"])),
            KeyValue(key="abs_tol_dps", value=f"{t.abs_tol_dps}"),
            KeyValue(key="rel_gain", value=f"{t.rel_gain}"),
            KeyValue(key="window_sec", value=f"{t.window_sec}"),
            KeyValue(key="warn_sustain_sec", value=f"{t.warn_sustain_sec}"),
            KeyValue(key="error_window_sec", value=f"{t.error_window_sec}"),
            KeyValue(key="error_mean_excess_dps", value=num(r["err_mean_dps"])),
            KeyValue(key="warning_episodes", value=str(r["warning_episodes"])),
            KeyValue(key="imu_silent_sec", value=f"{r['imu_silent_sec']:.2f}"),
            KeyValue(key="imu_returns", value=str(r["imu_returns"])),
            KeyValue(key="imu_first_appearances", value=str(r["imu_first_appearances"])),
            KeyValue(key="last_silence_sec", value=num(r["last_silence_sec"])),
            KeyValue(key="clock_resets", value=str(r["clock_resets"])),
            KeyValue(key="episodes", value=str(r["episodes"])),
            KeyValue(key="gap_share", value=num(r["gap_share"])),
            KeyValue(key="dropped_gyro_stale_order_bad", value=str(r["dropped"]["gyro"])),
            KeyValue(key="dropped_enc_stale_order_bad", value=str(r["dropped"]["enc"])),
            KeyValue(key="gyro_samples_without_tf", value=str(self._gyro_without_mount)),
            KeyValue(key="provisional", value="TO-VERIFY: slip detection not demonstrated"),
        ]
        if state != self._last_yaw_state:
            log = self.get_logger()
            if state in (D.IMU_SILENT, D.WHEEL_SILENT, D.BOTH_SILENT, "NO_MOUNT", D.INTERMITTENT):
                log.error(message)
            elif self._last_yaw_state is not None and self._last_yaw_state not in (
                    D.OK, D.WARMUP, D.NO_DATA, D.WARN, D.DIVERGED, D.DIVERGED_HELD):
                log.info(f"yaw-rate channel back to {state}: {message}")
            self._last_yaw_state = state
        events = r["imu_returns"] + r["imu_first_appearances"]
        if events != self._last_returns:
            self._last_returns = events
            self.get_logger().warning(
                (f"gyro appeared for the first time {r['last_silence_sec']:.1f} s after start"
                 if r["last_return_first"] else
                 f"gyro returned after {r['last_silence_sec']:.1f} s of silence"))
        return status

    def _tick(self):
        statuses = []
        if self.yaw_enabled:
            statuses.append(self._yaw_status())
        if self.xy_enabled:
            statuses.append(self._xy_status())
        if statuses:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.status = statuses
            self.diag_pub.publish(array)

    def _xy_status(self):
        state, ta, tb, div = self.window.evaluate()
        level = {
            DivergenceWindow.OK: DiagnosticStatus.OK,
            DivergenceWindow.DIVERGED: DiagnosticStatus.ERROR,
            DivergenceWindow.INSUFFICIENT_MOTION: DiagnosticStatus.OK,
            DivergenceWindow.NO_DATA: DiagnosticStatus.WARN,
        }[state]

        status = DiagnosticStatus()
        status.level = level
        status.name = "localization: odometry cross-check"
        status.hardware_id = "gripperx"
        status.message = {
            DivergenceWindow.OK: "sources agree",
            DivergenceWindow.DIVERGED: (
                f"SOURCES DISAGREE by {div:.3f} m over {self.window.window_sec:.1f} s "
                f"- one of them is not measuring the ground"
            ),
            DivergenceWindow.INSUFFICIENT_MOTION: "too little motion to judge",
            DivergenceWindow.NO_DATA: "waiting for both sources",
        }[state]
        status.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="source_a", value=self.a_topic),
            KeyValue(key="source_b", value=self.b_topic),
            KeyValue(key="travel_a_m", value=f"{ta:.4f}"),
            KeyValue(key="travel_b_m", value=f"{tb:.4f}"),
            KeyValue(key="divergence_m", value=f"{div:.4f}"),
            KeyValue(key="threshold_m", value=f"{self.window.max_divergence_m:.4f}"),
        ]

        # Edge-triggered on the log, so a sustained fault does not drown the log
        # while a transition is never silent.
        if state != self._last_state:
            if state == DivergenceWindow.DIVERGED:
                self.get_logger().error(
                    f"{self.a_topic} travelled {ta:.3f} m while {self.b_topic} travelled "
                    f"{tb:.3f} m over {self.window.window_sec:.1f} s "
                    f"(divergence {div:.3f} m > {self.window.max_divergence_m:.2f} m). "
                    "One source is not measuring ground motion; the pose estimate "
                    "downstream cannot be trusted until this clears."
                )
            elif self._last_state == DivergenceWindow.DIVERGED:
                self.get_logger().info("odometry sources agree again")
            self._last_state = state
        return status


def main(args=None):
    rclpy.init(args=args)
    node = OdomDivergenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
