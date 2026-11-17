#!/usr/bin/env python3
"""Tables of the desk work after 2.5: gyro injection (2.5b, D14), scan stamps, the chained
replay and the slam penalty (2.7, D15). Reads run directories under --root, written by
imu_replay.py with config/imu_replay_plan_2p6.yaml.

    gripperx_imu_replay.sh injection-report --root ~/gripperx_bags/replay_2p6

INJECTED RUNS ARE NOT JUDGED BY THEIR OWN yc.json. yaw_compare takes the whole-turn
guess of its reference from the gyro in the bag, which is the faulted gyro here. So an
injected run is compared on one time grid with the uninjected run of the same bag (the
"clean" run), and in settled windows with the clean run's scan match increments.

Only runs that passed check_run are read; a missing or failed run raises.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from gripperx_localization.imu_replay import (SLAM_STAMP_LEAD_S, _reader, active_config,
                                              bag_span, imu_series, load_plan, plan_path)
from gripperx_localization.imu_replay_report import PHASES, TURN_SUMS, map_odom_motion

GRID_HZ = 50.0
SETTLED_DEG = 3.0            # CHOSEN: "slam is back" = within this of the clean run
# Report section 3.2 of the 2.5 report: a slam difference below this is not measurable.
SLAM_RULE_DEG = 2.9
ARC_SEGMENTS = [1, 2, 3, 4]     # numbering of a run of the arc slice
# Input name prefix -> key of the phase tables of imu_replay_report.
SOURCE_OF = (("drive15", "drive15"), ("rot13", "rot13"))


class MissingRun(RuntimeError):
    pass


def run_dir(root, name) -> Path:
    d = Path(root).expanduser() / "runs" / name
    chk = d / "check.json"
    if not chk.exists() or not json.loads(chk.read_text())["ok"]:
        raise MissingRun(f"{name}: no passed check_run")
    return d


def have(root, name) -> bool:
    try:
        run_dir(root, name)
        return True
    except MissingRun:
        return False


def zoh(t_src, v_src, t):
    """Value of a step-like signal at t: last sample at or before t."""
    k = np.clip(np.searchsorted(t_src, t, side="right") - 1, 0, len(t_src) - 1)
    return v_src[k]


def tracks(run, hz=GRID_HZ) -> dict:
    """EKF, slam, wheel and gyro of one run on one grid. Yaw in deg, unwrapped."""
    from gripperx_localization.yaw_compare import yaw_from_quat
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    out = Path(run) / "out"
    r = _reader(out, ["/tf", "/wheel/odom", "/odometry/filtered"])
    ob, mo, wheel, ekf = [], [], [], []
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == "/tf":
            for tr in deserialize_message(raw, TFMessage).transforms:
                pair = (tr.header.frame_id, tr.child_frame_id)
                if pair not in (("odom", "base_footprint"), ("map", "odom")):
                    continue
                q, p = tr.transform.rotation, tr.transform.translation
                row = (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                       yaw_from_quat(q.x, q.y, q.z, q.w), p.x, p.y)
                (ob if pair[1] == "base_footprint" else mo).append(row)
        else:
            m = deserialize_message(raw, Odometry)
            (wheel if topic == "/wheel/odom" else ekf).append(
                (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.angular.z))
    ob, mo, wheel, ekf = (np.array(sorted(a)) for a in (ob, mo, wheel, ekf))
    ob_yaw, mo_yaw = np.unwrap(ob[:, 1]), np.unwrap(mo[:, 1])
    t = np.arange(ob[0, 0], ob[-1, 0], 1.0 / hz)
    e = np.interp(t, ob[:, 0], ob_yaw)
    # A map->odom stamp is future-dated by SLAM_STAMP_LEAD_S; the step belongs earlier.
    my, mx, mz = (zoh(mo[:, 0] - SLAM_STAMP_LEAD_S, c, t) for c in (mo_yaw, mo[:, 2], mo[:, 3]))
    ox, oy = np.interp(t, ob[:, 0], ob[:, 2]), np.interp(t, ob[:, 0], ob[:, 3])
    tg, zg = imu_series(out, "/imu/data/filtered")
    return {"t": t, "ekf": np.degrees(e), "slam": np.degrees(e + my), "mo": np.degrees(my),
            "x": mx + np.cos(my) * ox - np.sin(my) * oy,
            "y": mz + np.sin(my) * ox + np.cos(my) * oy,
            "rate_wheel": np.interp(t, wheel[:, 0], wheel[:, 1]),
            "rate_ekf": np.interp(t, ekf[:, 0], ekf[:, 1]),
            "gyro_t": tg, "gyro_z": zg}


def on_grid(a, t, key):
    return np.interp(t, a["t"], a[key])


def d9_alarm(t, enc_dps, gyro_dps, abs_tol=3.0, rel_gain=0.15, window_s=1.0, sustain_s=1.0):
    """First times the D9 rule holds and has held for sustain_s, on regular t.

    Rule as decided (D9) and as branch Theo-imu-yawmon implements its tolerance:
    |mean(enc) - mean(gyro)| > abs_tol + rel_gain * |mean(gyro)|, means over window_s.
    Re-evaluated here on the grid; it is not that branch's code, which runs on arrival
    times. gyro_dps carries nan where the IMU was silent: the rule is then undefined
    and the monitor's silence channel is the one that answers.
    """
    dt = float(np.median(np.diff(t)))
    n = max(1, int(round(window_s / dt)))
    kernel = np.ones(n) / n
    me = np.convolve(enc_dps, kernel, mode="full")[:len(t)]
    mg = np.convolve(np.nan_to_num(gyro_dps), kernel, mode="full")[:len(t)]
    cond = np.abs(me - mg) > abs_tol + rel_gain * np.abs(mg)
    cond[:n] = False
    cond &= np.isfinite(gyro_dps)
    need = int(round(sustain_s / dt))
    run, fires = 0, []
    for k, c in enumerate(cond):
        run = run + 1 if c else 0
        if run == need:
            fires.append(float(t[k]))
    return fires


def time_until_held(t, ok, t_from, hold_s=1.0):
    """Seconds after t_from until ok has held for hold_s; None if it never does."""
    need = int(round(hold_s * GRID_HZ))
    run = 0
    for k in np.flatnonzero(t >= t_from):
        run = run + 1 if ok[k] else 0
        if run == need:
            return float(t[k - need + 1] - t_from)
    return None


def clean_d9(root, name) -> dict:
    """The D9 rule on an uninjected run: every firing there is a false alarm."""
    a = tracks(run_dir(root, name))
    g = -np.interp(a["t"], a["gyro_t"], a["gyro_z"])
    fires = d9_alarm(a["t"], np.degrees(a["rate_wheel"]), np.degrees(g))
    return {"run": name, "n": len(fires), "at_s": [round(f - a["t"][0], 1) for f in fires]}


def rejected_samples(run) -> int:
    """imu_implausible_count at its last throttled log line. Exact only if it was logged
    after the last rejection, so the sample count of the output is reported beside it."""
    import re
    text = (Path(run) / "stack.log").read_text(errors="replace")
    vals = [int(v) for v in re.findall(r"; (\d+) dropped since start", text)]
    return max(vals) if vals else 0


def injection_row(root, name, clean_name, b_name, source_t0, inj) -> dict:
    run, clean, base = (run_dir(root, n) for n in (name, clean_name, b_name))
    a, c, b = tracks(run), tracks(clean), tracks(base)
    t = a["t"]
    ws, we = source_t0 + inj["start"], source_t0 + inj["start"] + inj["duration"]
    keep = (t >= c["t"][0]) & (t <= c["t"][-1]) & (t >= b["t"][0]) & (t <= b["t"][-1])
    t = t[keep]
    d_ekf = a["ekf"][keep] - on_grid(c, t, "ekf")
    d_slam = a["slam"][keep] - on_grid(c, t, "slam")
    d_ekf_b = a["ekf"][keep] - on_grid(b, t, "ekf")
    d_slam_b = a["slam"][keep] - on_grid(b, t, "slam")
    win = (t >= ws) & (t <= we)
    after = t > we
    pre = t < ws
    row = {"run": name, "mode": inj["mode"], "start_s": inj["start"], "duration_s": inj["duration"],
           "ekf_pre_deg": float(np.abs(d_ekf[pre]).max()) if pre.any() else 0.0,
           "ekf_peak_deg": float(np.abs(d_ekf).max()),
           "ekf_at_end_deg": float(d_ekf[win][-1]),
           "ekf_final_deg": float(d_ekf[-1]),
           "slam_peak_deg": float(np.abs(d_slam[pre | win | after]).max()),
           "slam_at_end_deg": float(d_slam[win][-1]),
           "slam_final_deg": float(d_slam[-1]),
           "ekf_final_vs_b_deg": float(d_ekf_b[-1]),
           "slam_final_vs_b_deg": float(d_slam_b[-1]),
           "n_rejected_logged": rejected_samples(run)}
    step = np.abs(np.diff(a["mo"]))
    row["slam_max_step_deg"] = float(step.max())
    row["slam_max_step_at_s"] = float(a["t"][int(step.argmax())] - ws)
    # When slam is back for good.
    bad = np.flatnonzero((np.abs(d_slam) > SETTLED_DEG) & (t >= ws))
    row["slam_back_after_end_s"] = (None if len(bad) and bad[-1] == len(t) - 1 else
                                    (float(t[bad[-1]] - we) if len(bad) else 0.0))
    # What the EKF follows while the fault lasts.
    g = np.interp(t, a["gyro_t"], a["gyro_z"]) * -1.0
    r_ekf, r_wheel = a["rate_ekf"][keep], a["rate_wheel"][keep]
    if inj["mode"] == "silent":
        sw = win & (t > ws + 0.5)
        row["ekf_minus_wheel_rate_silence_max"] = float(np.abs(r_ekf[sw] - r_wheel[sw]).max())
    else:
        row["ekf_minus_gyro_rate_median"] = float(np.median(np.abs(r_ekf[win] - g[win])))
        row["ekf_minus_wheel_rate_median"] = float(np.median(np.abs(r_ekf[win] - r_wheel[win])))
    # The return. A clean return: rate back on the clean run's, no yaw step of its own.
    ret = (t >= we) & (t < we + 2.0)
    r_clean = on_grid(c, t, "rate_ekf")
    row["rate_back_after_end_s"] = time_until_held(t, np.abs(r_ekf - r_clean) <= 0.02, we)
    row["rate_step_at_return"] = float(np.abs(np.diff(r_ekf[ret])).max())
    row["yaw_step_at_return_deg"] = float(
        np.abs(np.diff(a["ekf"][keep][ret]) - np.degrees(r_ekf[ret][:-1]) / GRID_HZ).max())
    # Yaw gained after the return must equal the clean run's: drift left by the fault.
    last = len(t) - 1
    k0 = int(np.searchsorted(t, we + 2.0))
    row["ekf_change_after_return_deg"] = float(d_ekf[last] - d_ekf[k0]) if k0 < last else None
    row["ekf_slope_after_return_deg_min"] = (
        float((d_ekf[last] - d_ekf[k0]) / (t[last] - t[k0]) * 60.0) if k0 < last else None)
    # Bias in use, read off the node: output minus input on the same stamp.
    t_in, z_in = imu_series(Path(json.loads((run / "meta.json").read_text())["input"]),
                            "/imu/bno085/data")
    t_tail = a["gyro_t"] > a["gyro_t"][-1] - 5.0
    row["bias_at_end_rad_s"] = float(np.median(
        np.interp(a["gyro_t"][t_tail], t_in, z_in) - a["gyro_z"][t_tail]))
    # The 2.6 rule, offline.
    gyro_grid = g.copy()
    if inj["mode"] == "silent":
        gyro_grid[win] = np.nan
    fires = d9_alarm(t, np.degrees(r_wheel), np.degrees(gyro_grid))
    fires_in = [f for f in fires if ws <= f <= we + 3.0]
    row["d9_first_after_start_s"] = float(fires_in[0] - ws) if fires_in else None
    row["d9_n_outside"] = len(fires) - len(fires_in)
    if fires_in:
        row["ekf_err_at_d9_deg"] = float(np.interp(fires_in[0], t, d_ekf))
    return row


def source_key(input_name):
    for prefix, key in SOURCE_OF:
        if input_name.startswith(prefix):
            return key
    raise KeyError(input_name)


def seg(res, i):
    return next((s for s in res["segments"] if s["index"] == i), None)


def seg_err(res, i, source):
    s = seg(res, i)
    if s is None or s["reference"] != "scanmatch":
        return None
    return s["sources"].get(source, {}).get("error_deg")


def load_yc(root, name):
    return json.loads((run_dir(root, name) / "yc.json").read_text())


def arc_errors(root, name, source="slam"):
    """Signed slam error per arc of an arc slice run, deg, and its median |error|."""
    errs = [seg_err(load_yc(root, name), i, source) for i in ARC_SEGMENTS]
    return errs, float(np.median(np.abs(errs)))


def phase_table(root, name):
    """{phase: (median |error|, max |error|)} of slam, for a full-bag run."""
    res = load_yc(root, name)
    key = source_key(name.split("__")[0])
    out = {}
    for phase, segs in PHASES[key].items():
        e = [abs(x) for x in (seg_err(res, i, "slam") for i in segs) if x is not None]
        if e:
            out[phase] = (float(np.median(e)), float(max(e)))
    return out


def turn_sums(root, name, source="slam"):
    res = load_yc(root, name)
    key = source_key(name.split("__")[0])
    out = {}
    for label, segs in TURN_SUMS[key].items():
        errs = [seg_err(res, i, source) for i in segs]
        out[label] = None if any(e is None for e in errs) else float(sum(errs))
    return out


def translation_errors(root, name):
    """Per segment with a scan match: slam displacement in the base frame at the segment
    start, against the scan match's tx, ty. Metres. The poses are taken in the settled
    windows at both ends."""
    run = run_dir(root, name)
    res = json.loads((run / "yc.json").read_text())
    tr = tracks(run)
    out = {}
    t0 = res["t0_epoch"]
    for s in res["segments"]:
        sm = s.get("scanmatch") or {}
        if s["reference"] != "scanmatch" or sm.get("tx") is None:
            continue
        ta, tb = t0 + s["t0"], t0 + s["t1"]
        yaw = math.radians(float(np.interp(ta, tr["t"], tr["slam"])))
        dx = float(np.interp(tb, tr["t"], tr["x"]) - np.interp(ta, tr["t"], tr["x"]))
        dy = float(np.interp(tb, tr["t"], tr["y"]) - np.interp(ta, tr["t"], tr["y"]))
        bx = math.cos(yaw) * dx + math.sin(yaw) * dy
        by = -math.sin(yaw) * dx + math.cos(yaw) * dy
        out[s["index"]] = {"error_m": math.hypot(bx - sm["tx"], by - sm["ty"]),
                           "dist_m": math.hypot(sm["tx"], sm["ty"])}
    return out


def translation_phases(root, name):
    key = source_key(name.split("__")[0])
    te = translation_errors(root, name)
    res = {}
    for phase, segs in PHASES[key].items():
        e = [te[i]["error_m"] for i in segs if i in te]
        if e:
            res[phase] = (float(np.median(e)), float(max(e)))
    return res


def chain_map(chain_res, clean_res, second_start_s):
    """Chained run's drive15 part: {drive15 segment index: chained segment}. Refuses a
    segmentation that does not line up with the unchained run's."""
    mapped = {}
    for s in clean_res["segments"]:
        cand = [c for c in chain_res["segments"]
                if abs(c["t0"] - second_start_s - s["t0"]) < 1.0
                and abs(c["t1"] - second_start_s - s["t1"]) < 1.0]
        if len(cand) == 1:
            mapped[s["index"]] = cand[0]
    return mapped


def cumulative(res, segs, source="slam"):
    total, out = 0.0, {}
    for i in segs:
        s = seg(res, i)
        e = None if s is None or s["reference"] != "scanmatch" else s["sources"][source]["error_deg"]
        if e is not None:
            total += e
            out[i] = total
    return out


def fmt(v, digits=2, signed=False):
    if v is None:
        return "-"
    return f"{v:+.{digits}f}" if signed else f"{v:.{digits}f}"


def table(head, rows) -> str:
    lines = ["| " + " | ".join(head) + " |", "| " + " | ".join("---" for _ in head) + " |"]
    lines += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(lines)


INJECTIONS = (
    ("turn_stuck5", "rot13s454"), ("turn_zero5", "rot13s454"), ("turn_stuck45", "rot13s454"),
    ("arc_stuck5", "arc_slice"), ("arc_zero5", "arc_slice"),
    ("spin_stuck5", "spin_slice"), ("spin_zero2", "spin_slice"),
    ("turn_silent3", "rot13s454"), ("turn_silent10", "rot13s454"),
    ("arc_silent5", "arc_slice"), ("spin_silent2", "spin_slice"),
)


def injection_section(root, plan) -> str:
    rows = {}
    for inp, clean in INJECTIONS:
        name = f"{inp}__A_inject__1"
        if not have(root, name):
            continue
        spec = plan["inputs"][inp]
        t0 = bag_span(Path(spec["bag"]).expanduser())[0]
        rows[inp] = injection_row(root, name, f"{clean}__A_merged__1", f"{clean}__B_baseline__1", t0,
                                  spec["prepare"]["gyro_inject"])
    (Path(root).expanduser() / "injection.json").write_text(json.dumps(rows, indent=1))
    fault = [k for k in rows if rows[k]["mode"] != "silent"]
    silent = [k for k in rows if rows[k]["mode"] == "silent"]
    t1 = table(["Run", "Fault", "EKF err at fault end", "EKF final", "EKF peak", "EKF drift after (deg/min)",
                "slam err at fault end", "slam final", "slam peak", "slam max step (deg, at s)",
                "slam within 3 deg after end (s)", "Rejected (log)", "D9 fires after (s)", "EKF err at D9",
                "Pre-fault diff"],
               [[k, f"{rows[k]['mode']} {rows[k]['duration_s']:.0f} s",
                 fmt(rows[k]["ekf_at_end_deg"], 1, True), fmt(rows[k]["ekf_final_deg"], 1, True),
                 fmt(rows[k]["ekf_peak_deg"], 1), fmt(rows[k]["ekf_slope_after_return_deg_min"], 1, True),
                 fmt(rows[k]["slam_at_end_deg"], 1, True),
                 fmt(rows[k]["slam_final_deg"], 1, True), fmt(rows[k]["slam_peak_deg"], 1),
                 f"{rows[k]['slam_max_step_deg']:.1f} at {rows[k]['slam_max_step_at_s']:.1f}",
                 fmt(rows[k]["slam_back_after_end_s"], 1) if rows[k]["slam_back_after_end_s"] is not None
                 else "never", rows[k]["n_rejected_logged"],
                 fmt(rows[k]["d9_first_after_start_s"]), fmt(rows[k].get("ekf_err_at_d9_deg"), 1, True),
                 fmt(rows[k]["ekf_pre_deg"], 3)]
                for k in fault])
    t2 = table(["Run", "|EKF-gyro| rate", "|EKF-encoder| rate", "Bias at end (rad/s)",
                "EKF final vs B", "slam final vs B"],
               [[k, fmt(rows[k]["ekf_minus_gyro_rate_median"], 3),
                 fmt(rows[k]["ekf_minus_wheel_rate_median"], 3), fmt(rows[k]["bias_at_end_rad_s"], 4),
                 fmt(rows[k]["ekf_final_vs_b_deg"], 1, True), fmt(rows[k]["slam_final_vs_b_deg"], 1, True)]
                for k in fault])
    t3 = table(["Run", "Silence", "|EKF-encoder| rate, max", "EKF err at return", "EKF final", "slam final",
                "slam peak", "Rate back after (s)", "Rate step at return", "Yaw step at return (deg)",
                "EKF drift after return (deg/min)", "Rejected"],
               [[k, f"{rows[k]['duration_s']:.0f} s", fmt(rows[k]["ekf_minus_wheel_rate_silence_max"], 3),
                 fmt(rows[k]["ekf_at_end_deg"], 2, True), fmt(rows[k]["ekf_final_deg"], 2, True),
                 fmt(rows[k]["slam_final_deg"], 1, True), fmt(rows[k]["slam_peak_deg"], 1),
                 fmt(rows[k]["rate_back_after_end_s"]), fmt(rows[k]["rate_step_at_return"], 3),
                 fmt(rows[k]["yaw_step_at_return_deg"], 3), fmt(rows[k]["ekf_slope_after_return_deg_min"], 3, True),
                 rows[k]["n_rejected_logged"]] for k in silent])
    d9 = [clean_d9(root, n) for n in ("rot13s454__A_merged__1", "arc_slice__A_merged__1",
                                      "spin_slice__A_merged__1", "drive15s454__A_merged__1")]
    t4 = table(["Clean run", "D9 firings", "At s"], [[c["run"], c["n"], c["at_s"]] for c in d9])
    return "\n\n".join(["### Faults", t1, "### What the EKF follows", t2, "### Silence and return", t3,
                        "### D9 rule on the clean runs", t4])


ARC_OFFSETS = (("arc_p050", "+50"), ("arc_slice", "0"), ("arc_m050", "-50"), ("arc_m075", "-75"),
               ("arc_m100", "-100"), ("arc_m125", "-125"), ("arc_m150", "-150"))


def repeats(root, inp, variant):
    k, out = 1, []
    while have(root, f"{inp}__{variant}__{k}"):
        out.append(f"{inp}__{variant}__{k}")
        k += 1
    return out


def arc_section(root) -> str:
    rows, out = [], {}
    for inp, label in ARC_OFFSETS:
        for variant in ("A_merged", "B_baseline"):
            names = repeats(root, inp, variant)
            if not names:
                continue
            res = [arc_errors(root, n) for n in names]
            meds = [m for _, m in res]
            mean_abs = [float(np.mean(np.abs(e))) for e, _ in res]
            out[f"{inp}/{variant}"] = {"medians": meds, "errors": [e for e, _ in res]}
            rows.append([label, variant[0], len(names), fmt(float(np.mean(meds))),
                         f"{min(meds):.2f}..{max(meds):.2f}", fmt(float(np.mean(mean_abs))),
                         " ".join(fmt(x, 1, True) for x in res[0][0])])
    (Path(root).expanduser() / "arcs.json").write_text(json.dumps(out, indent=1))
    return table(["/scan stamp shift (ms)", "Variant", "Runs", "Median |arc error|, mean", "Range",
                  "Mean |arc error|", "Arc errors, first run"], rows)


def full_section(root) -> str:
    """Phases as rows, one column per input and variant; n = runs behind a column."""
    cols = []
    for inp, label in (("drive15s454", "0"), ("drive15s454_m050", "-50"), ("drive15s454_m100", "-100"),
                       ("rot13s454", "0"), ("rot13s454_m050", "-50"), ("rot13s454_m100", "-100")):
        for variant in ("A_merged", "B_baseline"):
            names = repeats(root, inp, variant)
            if names:
                cols.append((f"{inp[:5]} {label} {variant[0]} n={len(names)}", names))
    phases = [p for key in ("drive15", "rot13") for p in PHASES[key]]

    def cell(names, fn):
        vals = [fn(n) for n in names]
        vals = [v for v in vals if v is not None]
        if not vals:
            return "-"
        return " / ".join(fmt(float(np.mean([v[k] for v in vals])), 2) for k in (0, 1))

    head = ["Phase, median / max"] + [c[0] for c in cols]
    yaw = [[ph] + [cell(n, lambda x, ph=ph: phase_table(root, x).get(ph)) for _, n in cols] for ph in phases]
    trn = [[ph] + [cell(n, lambda x, ph=ph: translation_phases(root, x).get(ph)) for _, n in cols]
           for ph in phases]
    sums = [["Name"] + [c[0] for c in cols]]
    labels = sorted({k for key in ("drive15", "rot13") for k in TURN_SUMS[key]})
    srows = [[lab] + [" ".join(fmt(turn_sums(root, x).get(lab), 2, True) for x in n)
                      for _, n in cols] for lab in labels]
    return "\n\n".join(["slam yaw, deg:", table(head, yaw), "3-turn sums of slam, deg, each run:",
                        table(sums[0], srows), "Translation against the scan match, metres:",
                        table(head, trn)])


def penalty_section(root) -> str:
    rows = []
    groups = [("arc_slice", "0"), ("arc_m100", "-100"), ("drive15s454", "0"),
              ("drive15s454_m100", "-100"), ("rot13s454", "0")]
    for inp, shift in groups:
        for variant, label in (("A_avp1p0", "1.0"), ("A_avp1p5", "1.5"), ("A_merged", "2.0"),
                               ("A_avp3p0", "3.0"), ("A_avp4p0", "4.0")):
            for name in repeats(root, inp, variant)[:1]:
                if inp.startswith("arc"):
                    errs, med = arc_errors(root, name)
                    te = translation_errors(root, name)
                    arcs = [te[i]["error_m"] for i in ARC_SEGMENTS if i in te]
                    rows.append([inp, label, "Arc", fmt(med), fmt(max(abs(e) for e in errs)),
                                 fmt(float(np.median(arcs)) if arcs else None, 3),
                                 fmt(max(arcs) if arcs else None, 3)])
                else:
                    p, t = phase_table(root, name), translation_phases(root, name)
                    for phase in p:
                        rows.append([inp, label, phase, fmt(p[phase][0]), fmt(p[phase][1]),
                                     fmt(t.get(phase, (None, None))[0], 3),
                                     fmt(t.get(phase, (None, None))[1], 3)])
    return table(["Input", "Penalty", "Phase", "slam yaw median", "max", "Translation median (m)", "max (m)"],
                 rows)


def chain_section(root, plan) -> str:
    root_p = Path(root).expanduser()
    robot = json.loads((Path("~/gripperx_bags/replay_2p5/old_stack/drive15.json").expanduser()).read_text())
    rows = []
    spin = [1, 2, 3, 4, 5, 6]
    rows.append(["Robot", *[fmt(cumulative(robot, spin).get(i), 1, True) for i in spin],
                 *[fmt(float(np.median([abs(seg_err(robot, i, "slam")) for i in segs
                                        if seg_err(robot, i, "slam") is not None])))
                   for segs in PHASES["drive15"].values()],
                 " ".join(fmt(sum(seg_err(robot, i, "slam") for i in g), 2, True)
                          for g in TURN_SUMS["drive15"].values())])
    for name, label, key in (("drive15s454__A_merged__1", "Fresh map, A", None),
                             ("drive15s454__B_baseline__1", "Fresh map, B", None),
                             ("chain__A_chain__1", "Chained, A", "chain"),
                             ("chain__B_chain__1", "Chained, B", "chain"),
                             ("chain_m100__A_chain__1", "Chained, A, -100 ms", "chain_m100"),
                             ("chain_m100__A_chain__2", "Chained, A, -100 ms, repeat", "chain_m100"),
                             ("chain_m100__A_chain_avp1p0__1", "Chained, A, -100 ms, penalty 1.0",
                              "chain_m100"),
                             ("chain_m100__A_chain_avp3p0__1", "Chained, A, -100 ms, penalty 3.0",
                              "chain_m100")):
        if not have(root, name):
            continue
        res = load_yc(root, name)
        if key:
            start = json.loads((root_p / "inputs" / f"{key}.prepare.json").read_text())["second_starts_s"]
            clean = load_yc(root, "drive15s454__A_merged__1")
            m = chain_map(res, clean, start)
            res = {"segments": [{**s, "index": i} for i, s in m.items()]}
        cum = cumulative(res, spin)
        p = {}
        for phase, segs in PHASES["drive15"].items():
            e = [abs(x) for x in (seg_err(res, i, "slam") for i in segs) if x is not None]
            if e:
                p[phase] = float(np.median(e))
        sums = [sum(seg_err(res, i, "slam") for i in segs) for segs in TURN_SUMS["drive15"].values()]
        rows.append([label, *[fmt(cum.get(i), 1, True) for i in spin],
                     *[fmt(p.get(k)) for k in PHASES["drive15"]],
                     " ".join(fmt(x, 2, True) for x in sums)])
    return table(["Run", *[f"cum err seg {i}" for i in spin], *PHASES["drive15"], "CCW CW sums"], rows)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--plan", default=None)
    args = ap.parse_args(argv)
    plan = load_plan(args.plan or Path(plan_path()).with_name("imu_replay_plan_2p6.yaml"))
    root = args.root
    print("# Scan stamp shift, arc slice\n\n" + arc_section(root))
    print("\n# Full bags\n\n" + full_section(root))
    print("\n# Chain\n\n" + chain_section(root, plan))
    print("\n# Penalty\n\n" + penalty_section(root))
    print("\n# Injection\n\n" + injection_section(root, plan))
    print("\nActive slam penalty:",
          active_config()["slam_toolbox"]["ros__parameters"]["angle_variance_penalty"])
    return 0
