#!/usr/bin/env python3
"""Yaw-rate channel of odom_divergence_monitor (step 2.6): logic, silence, replay.

Every threshold is read from the ACTIVE configuration: localization.launch.py is
evaluated from the source tree and the parameters the odom_divergence_monitor node
would really receive are merged in launch order (the helper of check_imu_wiring.py).
Nothing below restates 3 deg/s, 0.15, 1 s; the synthetic signals are written in
terms of the configured values.

Part 1  configuration reaches the node, topics agree with the publisher
Part 2  synthetic logic (always runs, no bag, no ROS graph)
Part 3  replay of recorded bags (bag directory as argument or $GRIPPERX_BAG_DIR;
        the bags are not in the repo). With NO bag directory Part 3 does not run and the
        last line says so; with an explicit one a missing directory or bag FAILS.
        - CLEAN raw input (/imu/bno085/data, arrival = stamp): alarms per segment,
          including the 1.4 rad/s spins
        - CLEAN conditioned input from the 2.5 replays (<dir>/replay_2p5/runs/*/out:
          /imu/data/filtered + /wheel/odom with their RECORDED arrival times)
        - labelled SYNTHETIC overlays on the real signals: encoder slip, frozen gyro
          (no recording with known slip exists: slip detection stays TO-VERIFY)

Usage: check_yaw_divergence.py [bag_dir]    (run with ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
                                              on the dev laptop)
"""
from __future__ import annotations

import importlib.util
import math
import os
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / "src"))
# localization_input_node imports gripperx_control: use the source tree when the package
# is not installed (as in a bare checkout), the installed one otherwise.
sys.path.append(str(HERE.parents[2] / "gripperx_control" / "src"))
from gripperx_localization.odom_divergence_monitor import (  # noqa: E402
    YawRateDivergence as D, body_z_row)

LOC = HERE.parents[1]
FAILURES: list[str] = []


def check(cond: bool, label: str, detail: str = "") -> None:
    print(f"  {'ok  ' if cond else 'FAIL'}  {label}")
    if not cond:
        for line in detail.splitlines():
            print(f"          {line}")
        FAILURES.append(label)


def _wiring():
    spec = importlib.util.spec_from_file_location("_imu_wiring", HERE.parent / "check_imu_wiring.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def active_params() -> dict:
    """Parameters odom_divergence_monitor gets, per launch arguments."""
    w = _wiring()
    try:
        out = {}
        for laser in ("true", "false"):
            nodes = w.effective_parameters({"enable_laser_odometry": laser},
                                           wanted=("odom_divergence_monitor",
                                                   "localization_input_node"))
            out[laser] = nodes
        return out
    except ImportError:
        print("  note  launch/launch_ros not importable: reading localization.yaml alone")
        cfg = yaml.safe_load((LOC / "config" / "localization.yaml").read_text())
        mon = cfg["odom_divergence_monitor"]["ros__parameters"]
        lin = cfg["localization_input_node"]["ros__parameters"]
        return {k: {"odom_divergence_monitor": mon, "localization_input_node": lin}
                for k in ("true", "false")}


def make(P: dict, t0: float = 0.0) -> D:
    return D.from_params(P, t_start=t0)


def feed(d: D, t_from, t_to, enc_dps, gyro_dps, enc_hz=30.0, gyro_hz=50.0, stamp=True,
         skip_gyro=False):
    """Interleave samples in time order; rates are functions of t in deg/s.
    Returns the list of (t, state) sampled at each sample."""
    ev = []
    n = int((t_to - t_from) * enc_hz)
    for i in range(n):
        ev.append((t_from + i / enc_hz, "enc"))
    if not skip_gyro:
        for i in range(int((t_to - t_from) * gyro_hz)):
            ev.append((t_from + i / gyro_hz, "gyro"))
    ev.sort()
    states = []
    for t, which in ev:
        f = enc_dps if which == "enc" else gyro_dps
        d.push(which, t, t if stamp else None, math.radians(f(t)))
        states.append((t, d.evaluate(t)["state"]))
    return states


ALARM = (D.WARN, D.DIVERGED)   # the D9 condition shows as WARN, or DIVERGED once the 5 s mean latches


def ever(states, state, after=-1e9):
    wanted = state if isinstance(state, tuple) else (state,)
    return any(s in wanted for t, s in states if t >= after)


def part1(cfg):
    print("Part 1  configuration")
    for laser in ("true", "false"):
        nodes = cfg[laser]
        mon = nodes.get("odom_divergence_monitor")
        check(mon is not None, f"node runs with enable_laser_odometry={laser}")
        if mon is None:
            return None
        check(mon.get("xy_channel_enabled") is (laser == "true"),
              f"x/y channel follows enable_laser_odometry={laser}", str(mon.get("xy_channel_enabled")))
    mon = cfg["true"]["odom_divergence_monitor"]
    lin = cfg["true"]["localization_input_node"]
    check(mon.get("yaw_channel_enabled") is True, "yaw channel enabled")
    check(mon.get("gyro_topic") == lin.get("imu_output_topic"),
          "monitor reads the topic localization_input_node publishes",
          f"{mon.get('gyro_topic')} vs {lin.get('imu_output_topic')}")
    for k in ("yaw_window_sec", "yaw_abs_tol_dps", "yaw_rel_gain", "yaw_warn_sustain_sec",
              "imu_silence_timeout_sec", "wheel_silence_timeout_sec", "startup_grace_sec",
              "max_sample_age_sec", "min_window_fill", "return_hold_sec", "alarm_hold_sec",
              "gap_report_window_sec", "gap_report_fraction", "clock_step_tolerance_sec"):
        check(isinstance(mon.get(k), (int, float)) and mon[k] > 0, f"parameter {k} set and positive",
              str(mon.get(k)))
    text = (LOC / "config" / "localization.yaml").read_text()
    sec = text.split("odom_divergence_monitor:")[1].split("laser_odometry_node:")[0]
    check(sec.count("TO-VERIFY") >= 12, "every provisional value carries TO-VERIFY in the config")
    return mon


def part2(P):
    print("Part 2  synthetic logic (labelled SYNTHETIC)")
    tol, gain = P["yaw_abs_tol_dps"], P["yaw_rel_gain"]
    win, sus = P["yaw_window_sec"], P["yaw_warn_sustain_sec"]
    tmo = P["imu_silence_timeout_sec"]
    settle = win + sus + 1.0

    # Mount sign: the IMU is upside down (roll pi) -> body z = -sensor z
    row = body_z_row(1.0, 0.0, 0.0, 0.0)
    check(abs(row[2] + 1.0) < 1e-9 and abs(row[0]) < 1e-9 and abs(row[1]) < 1e-9,
          "body_z_row: roll pi flips the sign", str(row))
    row = body_z_row(0.0, 0.0, math.sin(0.3), math.cos(0.3))
    check(abs(row[2] - 1.0) < 1e-9, "body_z_row: a pure yaw keeps z", str(row))

    # Healthy, encoder over-reporting 9 % in a 1.4 rad/s spin (80.2 deg/s)
    g = math.degrees(1.4)
    d = make(P)
    s = feed(d, 0, 12, lambda t: 1.09 * g, lambda t: g)
    check(not ever(s, ALARM), "quiet: 9 % encoder over-report at 1.4 rad/s")
    check(s[-1][1] == D.OK, "settles to OK", s[-1][1])
    d = make(P)
    s = feed(d, 0, 12, lambda t: 0.0, lambda t: 0.0)
    check(s[-1][1] == D.OK, "standing still reads OK")

    # Slip: encoder reports rotation the gyro does not (straight drive, yaw ~0)
    d = make(P)
    s = feed(d, 0, 6, lambda t: 0.0, lambda t: 0.0)
    slip = tol + 0.5 * tol          # above tolerance, rate 0 -> limit is tol alone
    s += feed(d, 6, 6 + settle, lambda t: slip, lambda t: 0.0)
    check(ever(s, D.WARN, 6), f"SYNTHETIC slip {slip:.1f} deg/s on a straight leg fires")
    first = next(t for t, st in s if st == D.WARN)
    check(first - 6 >= sus, "fires no earlier than the sustain time", f"after {first - 6:.2f} s")
    t_end = 6 + settle
    s += feed(d, t_end, t_end + 2, lambda t: 0.0, lambda t: 0.0)
    check(s[-1][1] == D.OK and not d.latched,
          "after a short slip the WARNING clears at once and nothing is latched (D17)", s[-1][1])
    s += feed(d, t_end + 2, t_end + 2 + P["alarm_hold_sec"] + 3, lambda t: 0.0, lambda t: 0.0)
    check(s[-1][1] == D.OK, "clears after the hold", s[-1][1])
    # Short blip below the sustain time
    d = make(P)
    s = feed(d, 0, 6, lambda t: 0.0, lambda t: 0.0)
    s += feed(d, 6, 6 + 0.4 * win, lambda t: 5 * tol, lambda t: 0.0)
    s += feed(d, 6 + 0.4 * win, 12, lambda t: 0.0, lambda t: 0.0)
    check(not ever(s, ALARM), "a transient well under window+sustain does not fire")

    # Stuck gyro
    d = make(P)
    s = feed(d, 0, 6, lambda t: g, lambda t: g)
    s += feed(d, 6, 6 + settle, lambda t: g, lambda t: 0.0)
    check(ever(s, ALARM, 6), "gyro stuck at zero during a 1.4 rad/s spin fires")
    d = make(P)
    s = feed(d, 0, 6, lambda t: g, lambda t: g)
    s += feed(d, 6, 6 + settle, lambda t: 0.3 * g, lambda t: g)
    check(ever(s, ALARM, 6), "encoder at 30 % of the gyro (dead wheels) fires")
    # The blind spot, stated as a test so it stays stated
    d = make(P)
    s = feed(d, 0, 12, lambda t: 0.0, lambda t: 0.0)
    s += feed(d, 12, 24, lambda t: 0.5 * tol, lambda t: 0.0)
    check(not ever(s, ALARM, 12),
          "KNOWN BLIND SPOT: gyro stuck at zero while the robot turns slower than "
          "yaw_abs_tol_dps (or stands) is not seen")

    # IMU silence and return
    d = make(P)
    s = feed(d, 0, 6, lambda t: g, lambda t: g)
    s += feed(d, 6, 6 + tmo + 3, lambda t: g, lambda t: g, skip_gyro=True)
    check(ever(s, D.IMU_SILENT, 6 + tmo), "IMU silence is reported after the timeout")
    check(not ever(s, D.IMU_SILENT, 0) or next(t for t, st in s if st == D.IMU_SILENT) >= 6 + tmo - 0.1,
          "not reported before the timeout")
    r = d.evaluate(6 + tmo + 3)
    check(r["state"] == D.IMU_SILENT and r["imu_silent_sec"] >= tmo, "silence length reported",
          str(r["imu_silent_sec"]))
    t_back = 6 + tmo + 3
    s = feed(d, t_back, t_back + 4, lambda t: g, lambda t: g)
    r = d.evaluate(t_back + 4)
    check(r["imu_returns"] == 1 and r["last_silence_sec"] >= tmo, "return counted with silence length",
          str(r["last_silence_sec"]))
    check(not ever(s, ALARM), "no false alarm on the return")
    check(s[0][1] in (D.WARMUP, D.IMU_SILENT) and s[-1][1] == D.OK,
          "after the return: warm-up, then OK", f"{s[0][1]} -> {s[-1][1]}")
    # silence seen only by the sample gap (timer never ran)
    d = make(P)
    feed(d, 0, 6, lambda t: g, lambda t: g)
    d.push("gyro", 6 + 5 * tmo, 6 + 5 * tmo, math.radians(g))
    check(d.imu_returns == 1, "a gap longer than the timeout counts as a silence even without a timer")
    # first appearance after the grace is not a return
    d = make(P)
    feed(d, 0, P["startup_grace_sec"] + 1, lambda t: 0.0, lambda t: 0.0, skip_gyro=True)
    t1 = P["startup_grace_sec"] + 1
    feed(d, t1, t1 + 2, lambda t: 0.0, lambda t: 0.0)
    check(d.imu_returns == 0 and d.imu_first_appearances == 1 and d.last_return_first,
          "a first appearance after the grace is counted apart from returns")
    # a backward clock step drops the return event
    d = make(P, 100.0)
    feed(d, 100, 106, lambda t: 0.0, lambda t: 0.0)
    feed(d, 106 + 3 * tmo, 112 + 3 * tmo, lambda t: 0.0, lambda t: 0.0)
    check(d.last_return_t is not None, "(setup) a return is recorded")
    feed(d, 0, 3, lambda t: 0.0, lambda t: 0.0)
    check(d.last_return_t is None, "a backward clock step clears the return event")
    # never any IMU
    d = make(P)
    s = feed(d, 0, P["startup_grace_sec"] + 3, lambda t: 0.0, lambda t: 0.0, skip_gyro=True)
    check(ever(s, D.NO_DATA) and s[-1][1] == D.IMU_SILENT,
          "IMU absent from boot: NO_DATA in the grace period, then IMU_SILENT")
    # wheel silent
    d = make(P)
    feed(d, 0, 6, lambda t: g, lambda t: g)
    for i in range(int(3 * 50)):
        t = 6 + i / 50.0
        d.push("gyro", t, t, math.radians(g))
    check(d.evaluate(9.0)["state"] == D.WHEEL_SILENT, "wheel odometry silence reported")

    # Stale and out-of-order samples
    d = make(P)
    ok = d.push("gyro", 6.0, 6.0 - 5 * P["max_sample_age_sec"], math.radians(0.0))
    check(not ok and d.s["gyro"].dropped_stale == 1, "a stale-stamped sample is dropped")
    d.push("gyro", 6.01, 6.01, math.radians(g))
    ok = d.push("gyro", 6.02, 6.0, math.radians(0.0))
    check(not ok and d.s["gyro"].dropped_order == 1, "an out-of-order sample is dropped")
    ok = d.push("gyro", 6.03, 6.03, float("nan"))
    check(not ok and d.s["gyro"].dropped_bad == 1, "a non-finite sample is dropped")
    # a stream of only stale samples is a silent stream
    d = make(P)
    feed(d, 0, 6, lambda t: g, lambda t: g)
    for i in range(int(3 * 50)):
        t = 6 + i / 50.0
        d.push("enc", t, t, math.radians(g))
        d.push("gyro", t, t - 5 * P["max_sample_age_sec"], math.radians(g))
    check(d.evaluate(9.0)["state"] == D.IMU_SILENT, "stale-stamped stream is reported as silent")

    # Clock step back (bag loop, use_sim_time reset)
    d = make(P, 100.0)
    feed(d, 100, 110, lambda t: g, lambda t: g)
    s = feed(d, 0, 8, lambda t: g, lambda t: g)
    check(d.evaluate(8.0)["clock_resets"] >= 1 and s[-1][1] == D.OK and not ever(s, ALARM),
          "a backward clock step restarts the channel cleanly")
    # use_sim_time: nothing here reads a wall clock; times are whatever the node clock says
    d = make(P, 1_000_000.0)
    s = feed(d, 1_000_000.0, 1_000_008.0, lambda t: g, lambda t: g)
    check(s[-1][1] == D.OK, "works on an arbitrary (sim) time base")


def part2b(P, cfg):
    print("Part 2b  gaps, episodes, clocks, frozen gyro (SYNTHETIC)")
    tol, win, sus = P["yaw_abs_tol_dps"], P["yaw_window_sec"], P["yaw_warn_sustain_sec"]
    g = math.degrees(1.4)
    off = 2.0 * tol

    def burst(d, t0, t1, on, period, enc, gyro=lambda t: 0.0, enc_hz=30.0, gyro_hz=50.0):
        """Encoder continuous; gyro delivers only `on` s of every `period` s."""
        ev = [(t0 + i / enc_hz, "enc") for i in range(int((t1 - t0) * enc_hz))]
        ev += [(t0 + i / gyro_hz, "gyro") for i in range(int((t1 - t0) * gyro_hz))
               if ((t0 + i / gyro_hz) - t0) % period < on]
        ev.sort()
        out = []
        for t, w in ev:
            d.push(w, t, t, math.radians((enc if w == "enc" else gyro)(t)))
            out.append((t, d.evaluate(t)["state"]))
        return out

    # M1: gaps neither erase nor confirm an alarm, and recurring gaps are reported
    d = make(P)
    s = feed(d, 0, 6, lambda t: 0.0, lambda t: 0.0)
    s += burst(d, 6, 14, 0.45, 0.70, lambda t: off)
    check(ever(s, ALARM, 6), "M1: slip with the gyro 0.45 s on / 0.25 s off for 8 s fires")
    gw = P["gap_report_window_sec"]
    for label, enc_f in (("agreeing", lambda t: 0.0), ("slipping", lambda t: off)):
        d = make(P)
        s = feed(d, 0, 6, lambda t: 0.0, lambda t: 0.0)
        s += burst(d, 6, 6 + 2 * gw, 0.30, 0.60, enc_f)
        late = sorted({st for t, st in s if t > 6 + gw})
        print(f"        gyro 0.3 s on / 0.3 s off, {label}: states after the gap window {late}")
        check(D.OK not in late and D.WARMUP not in late and (D.INTERMITTENT in late or D.WARN in late or D.DIVERGED in late),
              f"M1: gyro 0.3 s on / 0.3 s off ({label}) is never reported OK/WARMUP", str(late))
    d = make(P)
    s = feed(d, 0, 6, lambda t: 0.0, lambda t: 0.0)
    s += feed(d, 6, 6 + win + sus + 1, lambda t: off, lambda t: 0.0)
    t1 = 6 + win + sus + 1
    s += burst(d, t1, t1 + 0.6, 0.0, 10.0, lambda t: off)          # gyro away 0.6 s < timeout
    check(s[-1][1] in ALARM, "M1: an active WARN/DIVERGED does not turn green in a gyro gap", s[-1][1])
    d = make(P)
    s = feed(d, 0, 20, lambda t: g, lambda t: g)
    s += burst(d, 20, 40, 0.45, 0.70, lambda t: g, gyro=lambda t: g)
    check(not ever(s, ALARM) and not ever(s, D.IMU_SILENT),
          "M1: bursty but agreeing sources do not alarm")

    # 1: every episode is counted and visible to a 0.5 s tick
    d = make(P)
    s = feed(d, 0, 6, lambda t: 0.0, lambda t: 0.0)
    t = 6.0
    seen = 0
    pulse = win + sus + 0.05
    for k in range(20):
        ss = feed(d, t, t + pulse, lambda x: 3 * off, lambda x: 0.0)
        ss += feed(d, t + pulse, t + pulse + P["alarm_hold_sec"] + 3, lambda x: 0.0, lambda x: 0.0)
        ticks = [st for tt, st in ss if abs(((tt - t) / 0.5) - round((tt - t) / 0.5)) < 0.02]
        seen += any(x in (D.DIVERGED, D.DIVERGED_HELD) for x in ticks)
        t += pulse + P["alarm_hold_sec"] + 3
    check(d.episodes == 20 and seen == 20,
          "episodes are counted and a 0.5 s tick sees each of 20 short pulses",
          f"episodes {d.episodes}, ticks saw {seen}/20")

    # 2: sim time starts at 0 (invalid) and the first /clock is large
    d = D.from_params(P)      # t_start None, as the node builds it
    t0 = 1.8e9
    ss = feed(d, t0, t0 + P["startup_grace_sec"] + 3, lambda t: 0.0, lambda t: 0.0)
    check(d.imu_returns == 0 and ss[-1][1] == D.OK, "2: first valid clock 1.8e9 gives no false return",
          f"returns {d.imu_returns}, {ss[-1][1]}")
    d = D.from_params(P)
    ss = feed(d, t0, t0 + P["startup_grace_sec"] - 2, lambda t: 0.0, lambda t: 0.0, skip_gyro=True)
    check(not ever(ss, D.IMU_SILENT), "2: start-up grace counts from the first valid clock")

    # 3: forward clock step (+1 day) with the monotonic clock unmoved
    d = make(P)
    mono = lambda t: 5000.0 + t   # noqa: E731
    for i in range(int(8 * 50)):
        t = i / 50.0
        d.push("gyro", t, t, math.radians(g), mono(t))
        if i % 2 == 0:
            d.push("enc", t, t, math.radians(g), mono(t))
    day = 86400.0
    states = []
    for i in range(int(8 * 50)):
        t = 8.0 + i / 50.0
        d.push("gyro", t + day, t + day, math.radians(g), mono(t))
        if i % 2 == 0:
            d.push("enc", t + day, t + day, math.radians(g), mono(t))
        states.append(d.evaluate(t + day, mono(t))["state"])
    r = d.evaluate(16.0 + day, mono(16.0))
    check(r["clock_resets"] == 1 and r["imu_returns"] == 0 and D.IMU_SILENT not in states
          and r["state"] == D.OK, "3: +1 day forward step restarts the channel: no silence, no return",
          f"{r['clock_resets']} {r['imu_returns']} {r['state']}")
    d = make(P)
    for i in range(int(6 * 50)):
        t = i / 50.0
        d.push("gyro", t, t, 0.0, mono(t))
        d.push("enc", t, t, 0.0, mono(t))
    r = d.evaluate(6.0 + 4 * P["imu_silence_timeout_sec"], mono(6.0 + 4 * P["imu_silence_timeout_sec"]))
    check(r["state"] == D.BOTH_SILENT and r["clock_resets"] == 0,
          "3: a real silence (both clocks advance) is still a silence")

    # 5: a gyro frozen at a level the plausibility filter passes, at standstill
    lin = cfg["true"]["localization_input_node"]
    from gripperx_localization.localization_input_node import GyroBiasEstimator
    est = GyroBiasEstimator(lin["gyro_bias_settle_sec"], lin["gyro_bias_window_sec"],
                            lin["gyro_bias_min_sec"], lin["gyro_bias_max_sample_dt"],
                            lin["gyro_bias_trim_count"])
    frozen = 0.5 * min(lin["imu_max_abs_yaw_rate"], lin["imu_max_yaw_rate_step"] / 0.5)   # passes both limits
    frozen = min(frozen, 0.5)
    d = make(P)
    fire = clear = None
    t, dt = 0.0, 0.02
    while t < lin["gyro_bias_min_sec"] + lin["gyro_bias_settle_sec"] + 40:
        est.mark_still(t)
        est.add_sample(t, frozen)
        b = est.bias()
        out = frozen - (b if b is not None else 0.0)
        d.push("gyro", t, t, out)
        if int(round(t / dt)) % 2 == 0:
            d.push("enc", t, t, 0.0)
        st = d.evaluate(t)["state"]
        if st == D.DIVERGED and fire is None:
            fire = t
        if st == D.DIVERGED:
            clear = None
        elif fire is not None and clear is None:
            clear = t
        t += dt
    check(fire is not None and fire < 10.0 and clear is not None
          and clear > lin["gyro_bias_min_sec"] and d.evaluate(t)["state"] in (D.OK, D.DIVERGED_HELD),
          "5: gyro frozen at %.2f rad/s at standstill: fires, then SELF-CLEARS once the bias estimator "
          "has learned it (%.0f s min)" % (frozen, lin["gyro_bias_min_sec"]),
          f"fires at {fire}, clears at {clear}")
    print(f"        (fires at {fire:.1f} s, clear after {clear:.1f} s)" if fire and clear else "")

def read_topics(uri, names):
    """[(recv_t_s, topic, msg)] of the bag at `uri`, in storage order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[n for n in names if n in types]))
    cls = {n: get_message(types[n]) for n in names if n in types}
    out = []
    while r.has_next():
        tp, raw, t = r.read_next()
        out.append((t * 1e-9, tp, deserialize_message(raw, cls[tp])))
    return out


def part3(P, bag_dir):
    print("Part 3  replay")
    import numpy as np
    from gripperx_localization import yaw_compare as yc
    base = Path(os.path.expanduser(bag_dir))
    check(base.is_dir(), f"bag directory {base} exists")
    if not base.is_dir():
        return
    raw_bags = ("2026-10-22_drive15", "2026-10-22_rot13")
    for name in raw_bags:
        check((base / name).is_dir(), f"bag {name} present under {base}")
    tol = P["yaw_abs_tol_dps"]
    seg_cache = {}

    def episodes(ts, st, a, b):
        m = (ts >= a) & (ts <= b)
        x = np.isin(st[m], ALARM)
        return int(np.sum(x & ~np.concatenate([[False], x[:-1]])))

    def states_of(d, ev):
        ts, st = [], []
        for t, k, v, stamp in ev:
            d.push("enc" if k == 0 else "gyro", t, stamp, v)
            ts.append(t)
            st.append(d.evaluate(t)["state"])
        return np.array(ts), np.array(st)

    for name in raw_bags:
        if not (base / name).is_dir():
            continue
        data = yc.read_bag(yc.resolve_bag(str(base / name)))
        res = yc.analyse(data)
        segs = res["segments"]
        seg_cache[name] = (data.t0, segs)
        R = yc.quat_to_matrix(yc.static_rotation(data.static_tf, data.base_frame, data.imu["frame_id"]))
        w = np.asarray(data.imu["w"], dtype=float)
        gyro_t = np.asarray(data.imu["t"], dtype=float)
        gyro = R[2, 0] * w[:, 0] + R[2, 1] * w[:, 1] + R[2, 2] * w[:, 2]
        enc_t = np.asarray(data.wheel_odom["t"], dtype=float)
        enc = np.asarray(data.wheel_odom["wz"], dtype=float)

        def run(enc_mod=None, gyro_mod=None):
            e = enc.copy() if enc_mod is None else enc_mod(enc_t, enc.copy())
            g = gyro.copy() if gyro_mod is None else gyro_mod(gyro_t, gyro.copy())
            ev = sorted([(t, 0, v, t) for t, v in zip(enc_t, e)] + [(t, 1, v, t) for t, v in zip(gyro_t, g)])
            d = make(P, ev[0][0])
            ts, st = states_of(d, ev)
            return ts, st, d

        ts, st, d = run()
        r = d.evaluate(ts[-1])
        bad = int(np.sum(np.isin(st, [D.IMU_SILENT, D.WHEEL_SILENT, D.BOTH_SILENT, D.INTERMITTENT])))
        print(f"\n  --- {name}: CLEAN raw input, {len(segs)} segments, {len(ts)} samples ---")
        total_in = 0
        for sg in segs:
            a, b = data.t0 + sg["t0"], data.t0 + sg["t1"]
            m = (enc_t >= a) & (enc_t <= b)
            peak = float(np.degrees(np.max(np.abs(enc[m])))) if m.any() else 0.0
            n = episodes(ts, st, a, b)
            total_in += n
            print(f"      seg {sg['index']:2d} {sg['phase']:8s} {sg['duration_s']:5.1f} s  "
                  f"peak |enc| {peak:5.1f} deg/s  false alarms {n}")
        n_all = episodes(ts, st, ts[0], ts[-1])
        print(f"      alarm episodes in segments {total_in}, whole bag {n_all} (class counter {r['episodes']}); "
              f"samples in silent/INTERMITTENT states {bad}")
        check(total_in == 0, f"{name}: no false alarm inside any segment", f"{total_in}")
        check(r["episodes"] == 0 and n_all == 0, f"{name}: no false alarm anywhere in the bag", f"{n_all}")
        check(bad == 0, f"{name}: no silence/INTERMITTENT reported on a healthy recording", f"{bad}")

        straight = [s for s in segs if s["phase"] == "straight"]
        spin = max((s for s in segs if s["phase"] == "rotation"),
                   key=lambda s: float(np.max(np.abs(enc[(enc_t >= data.t0 + s["t0"]) & (enc_t <= data.t0 + s["t1"])]))),
                   default=None)
        if straight:
            sg = straight[0]
            a = data.t0 + sg["t0"] + 4.0

            def slip(t, v, a=a):
                m = (t >= a) & (t <= a + 4.0)
                v[m] += math.radians(2.0 * tol)
                return v
            ts2, st2, _ = run(enc_mod=slip)
            check(episodes(ts2, st2, a, a + 6.0) >= 1 and episodes(ts2, st2, ts2[0], a - 0.5) == 0,
                  f"{name}: SYNTHETIC encoder slip +{2 * tol:.0f} deg/s for 4 s in straight segment "
                  f"{sg['index']} fires, nothing before it")
        if spin is not None:
            m = (enc_t >= data.t0 + spin["t0"]) & (enc_t <= data.t0 + spin["t1"])
            a = float(enc_t[m][np.argmax(np.abs(enc[m]))]) - 1.0

            def freeze(t, v, a=a):
                v[t >= a] = 0.0
                return v
            ts3, st3, _ = run(gyro_mod=freeze)
            check(episodes(ts3, st3, a, data.t0 + spin["t1"] + 3.0) >= 1,
                  f"{name}: SYNTHETIC gyro stuck at zero 1 s before the peak rate of spin segment "
                  f"{spin['index']} fires")
        if name.endswith("drive15"):
            a = data.t0 + 150.0
            keep = ~((gyro_t >= a) & (gyro_t < a + 3.0))
            ev = sorted([(t, 0, v, t) for t, v in zip(enc_t, enc)] +
                        [(t, 1, v, t) for t, v, k in zip(gyro_t, gyro, keep) if k])
            d = make(P, ev[0][0])
            ts4, st4 = states_of(d, ev)
            check(bool(np.any(st4 == D.IMU_SILENT)) and d.imu_returns == 1,
                  f"{name}: SYNTHETIC 3 s IMU drop is reported and its return counted",
                  f"returns {d.imu_returns}")

    # conditioned input with the RECORDED arrival times (the 2.5 replays)
    runs = base / "replay_2p5" / "runs"
    wanted = ["drive15__A_merged__1", "rot13__A_merged__1", "arc_shift__A_merged__1",
              "arc_slice__A_merged__1", "drive15_imu_silent__A_silent__1"]
    check(runs.is_dir(), f"{runs} exists")
    for run_name in wanted if runs.is_dir() else []:
        out = runs / run_name / "out"
        check(out.is_dir(), f"replay run {run_name} present")
        if not out.is_dir():
            continue
        msgs = read_topics(str(out), ["/wheel/odom", "/imu/data/filtered", "/tf_static"])
        statics = {}
        for _, tp, m in msgs:
            if tp == "/tf_static":
                for tr in m.transforms:
                    q = tr.transform.rotation
                    statics[tr.child_frame_id] = (tr.header.frame_id, (q.x, q.y, q.z, q.w))
        imu = [(t, m) for t, tp, m in msgs if tp == "/imu/data/filtered"]
        wheel = [(t, m) for t, tp, m in msgs if tp == "/wheel/odom"]
        check(bool(imu) and bool(wheel), f"{run_name}: /imu/data/filtered and /wheel/odom recorded",
              f"imu {len(imu)} wheel {len(wheel)}")
        if not (imu and wheel):
            continue
        R = yc.quat_to_matrix(yc.static_rotation(statics, "base_footprint", imu[0][1].header.frame_id))
        stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # noqa: E731
        ev = sorted([(t, 0, m.twist.twist.angular.z, stamp(m)) for t, m in wheel] +
                    [(t, 1, float(R[2] @ [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]),
                      stamp(m)) for t, m in imu], key=lambda x: (x[0], x[1]))
        d = make(P, ev[0][0])
        ts, st = states_of(d, ev)
        r = d.evaluate(ts[-1])
        lag = np.array([t - s_ for t, _, _, s_ in ev])
        silent_run = "silent" in run_name
        n_div = int(r["episodes"])
        n_silent = int(np.sum(np.isin(st, [D.IMU_SILENT, D.BOTH_SILENT])))
        n_int = int(np.sum(st == D.INTERMITTENT))
        base_t0, segs = seg_cache.get("2026-10-22_" + run_name.split("__")[0].replace("_imu_silent", ""), (None, []))
        in_seg = sum(episodes(ts, st, base_t0 + sg["t0"], base_t0 + sg["t1"]) for sg in segs) if base_t0 else -1
        print(f"  {run_name}: {len(ev)} samples, arrival-minus-stamp median {np.median(lag) * 1e3:.1f} ms "
              f"max {lag.max() * 1e3:.0f} ms; dropped {r['dropped']}; episodes {n_div} "
              f"(in segments {in_seg}); IMU_SILENT samples {n_silent}; INTERMITTENT {n_int}; "
              f"IMU returns {r['imu_returns']}")
        if silent_run:
            check(n_silent > 0, f"{run_name}: the recorded IMU silence is reported")
        else:
            check(n_div == 0 and n_silent == 0 and n_int == 0,
                  f"{run_name}: conditioned input, recorded arrival times: no alarm, silence or INTERMITTENT",
                  f"episodes {n_div}, silent {n_silent}, intermittent {n_int}")

def main() -> int:
    cfg = active_params()
    mon = part1(cfg)
    if mon is None:
        return 1
    part2(mon)
    part2b(mon, cfg)
    bag_dir = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("GRIPPERX_BAG_DIR")
    ran3 = False
    if bag_dir:
        try:
            import numpy  # noqa: F401
            from gripperx_localization import yaw_compare  # noqa: F401
            import rosbag2_py  # noqa: F401
        except ImportError as exc:
            check(False, "replay needs numpy, rosbag2_py and a sourced ROS environment", str(exc))
        else:
            part3(mon, bag_dir)
            ran3 = True
    print()
    if FAILURES:
        print("FAILURES:", *FAILURES, sep="\n  ")
        return 1
    if ran3:
        print("ALL CHECKS PASSED (Parts 1, 2, 2b, 3)")
    else:
        print("ALL CHECKS PASSED for Parts 1, 2, 2b. PART 3 (REPLAY) DID NOT RUN: pass the bag "
              "directory (e.g. ~/gripperx_bags) or set GRIPPERX_BAG_DIR; the bags are not in the repo")
    return 0


if __name__ == "__main__":
    sys.exit(main())
