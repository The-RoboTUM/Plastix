#!/usr/bin/env python3
"""Tables of the 2.5 A/B replay, from the run directories imu_replay.py wrote.

    gripperx_imu_replay.sh report            markdown on stdout, report.json beside the runs

Every error is against the settled scan match (reference "scanmatch", D6, sub-issue
1.4b), which reads /scan, the raw gyro for the whole-turn count and the wheel odometry
for the translation guess, but never the EKF. Segments where it failed fall back to
slam in yaw_compare; they are kept out of the phase tables and listed apart, because
slam is an output of the system under test. Reference increments are compared across
all runs of a bag: if they differ, the reference was not independent of the run.

Scatter rule: variants are compared only through the spread of repeated runs. A
difference inside it is "no measurable difference".
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

# Segment numbers as in report 1.6, section 1; the boundaries come from the wheels
# and the scans, so they do not depend on the run (asserted in load_runs).
PHASES = {
    "drive15": {"Spin 1.4 rad/s": [1, 2, 3, 4, 5, 6], "Straight": [7, 8, 9, 10],
                "Crab": [19, 20], "Arc": [12, 13, 14, 15], "Mixed/steer-only": [0, 11, 18]},
    "rot13": {"Spin 0.5 rad/s": [0, 1, 2, 3, 4]},
}
NO_REFERENCE = {"drive15": [16, 17], "rot13": [5]}
TURN_SUMS = {"drive15": {"CCW 1-3": [1, 2, 3], "CW 4-6": [4, 5, 6]},
             "rot13": {"CCW 0-2": [0, 1, 2], "CW 3-4 (2 turns)": [3, 4]}}
for _d in (PHASES, NO_REFERENCE, TURN_SUMS):      # same bag, scan size forced
    _d["drive15s454"] = _d["drive15s453"] = _d["drive15_imu_silent"] = _d["drive15"]
    _d["rot13s454"] = _d["rot13"]
GATES = (5, 10, 15, 20)
SPIN_EDGE_S = 0.5


def load_runs(root):
    """-> {input: {variant: [yaw_compare result, ...]}}, runs that passed check_run only."""
    groups, skipped = {}, []
    for d in sorted((Path(root) / "runs").iterdir()):
        check = d / "check.json"
        yc = d / "yc.json"
        if not check.exists() or not yc.exists() or not json.loads(check.read_text())["ok"]:
            skipped.append(d.name)
            continue
        inp, variant, _ = d.name.split("__")
        res = json.loads(yc.read_text())
        res["_dir"] = str(d)
        res["_meta"] = json.loads((d / "meta.json").read_text())
        groups.setdefault(inp, {}).setdefault(variant, []).append(res)
    return groups, skipped


def seg(res, i):
    for s in res["segments"]:
        if s["index"] == i:
            return s
    return None


def seg_err(res, i, source):
    s = seg(res, i)
    if s is None or s["reference"] != "scanmatch" or source not in s["sources"]:
        return None
    return s["sources"][source]["error_deg"]


def reference_spread(runs_by_variant):
    """Largest spread of any scan-match increment across all runs of one bag. Deg."""
    vals = {}
    n_seg = set()
    for runs in runs_by_variant.values():
        for r in runs:
            n_seg.add(len(r["segments"]))
            for s in r["segments"]:
                if s["reference"] == "scanmatch":
                    vals.setdefault(s["index"], []).append(s["reference_increment_deg"])
    return max((max(v) - min(v) for v in vals.values()), default=0.0), sorted(n_seg)


def phase_cells(runs, segs, source):
    """Per run the median and max |error|. -> (medians, maxima, n segments)."""
    meds, maxs, n = [], [], 0
    for r in runs:
        errs = [abs(e) for e in (seg_err(r, i, source) for i in segs) if e is not None]
        if errs:
            meds.append(float(np.median(errs)))
            maxs.append(float(np.max(errs)))
            n = len(errs)
    return meds, maxs, n


def fmt_range(vals, digits=2):
    if not vals:
        return "-"
    if len(vals) == 1:
        return f"{vals[0]:.{digits}f}"
    return f"{np.mean(vals):.{digits}f} [{min(vals):.{digits}f}..{max(vals):.{digits}f}]"


def turn_sum(res, segs, source):
    errs = [seg_err(res, i, source) for i in segs]
    return None if any(e is None for e in errs) else float(sum(errs))


def map_odom_motion(run_dir):
    """Total |change| of the map->odom yaw and the largest single step, deg, and final yaw."""
    from gripperx_localization.imu_replay import _reader
    from gripperx_localization.yaw_compare import yaw_from_quat
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    r = _reader(Path(run_dir) / "out", ["/tf"])
    ys = []
    while r.has_next():
        _, raw, _ = r.read_next()
        for tr in deserialize_message(raw, TFMessage).transforms:
            if (tr.header.frame_id, tr.child_frame_id) == ("map", "odom"):
                q = tr.transform.rotation
                ys.append(yaw_from_quat(q.x, q.y, q.z, q.w))
    y = np.degrees(np.unwrap(np.array(ys)))
    d = np.abs(np.diff(y))
    return {"total_deg": float(d.sum()), "max_step_deg": float(d.max()), "final_deg": float(y[-1])}


def scan_sizes(run_dir):
    """The size slam_toolbox fixed from its first scan, from its rejection messages."""
    text = Path(run_dir, "stack.log").read_text(errors="replace")
    return {int(m) for m in re.findall(r"range readings, expected (\d+)", text)}


def innovation_summary(npz_path, t0=0.0):
    """Normalised gyro innovations, all samples and spin edges, against a finite gate."""
    d = np.load(npz_path)
    imu = d["imu"]
    t, inn, p, r, z = d["t"][imu], d["innovation"][imu], d["p"][imu], d["r"][imu], d["z"][imu]
    s = np.sqrt(p + r)
    nu = np.abs(inn) / s
    moving = np.abs(z) > 0.2
    edges = np.flatnonzero(moving[1:] != moving[:-1])
    near = np.zeros(len(t), dtype=bool)
    for k in edges:
        near |= np.abs(t - t[k]) <= SPIN_EDGE_S
    out = {"n": int(len(nu)), "n_spin_edge_samples": int(near.sum()), "n_edges": int(len(edges)),
           "sigma_pred_rad_s_p5_p50_p95": [float(v) for v in np.sqrt(np.percentile(p, [5, 50, 95]))],
           "s_rad_s_p50": float(np.median(s)),
           "nu_p50_p99_p999_max": [float(v) for v in np.percentile(nu, [50, 99, 99.9, 100])],
           "nu_edge_p50_p99_max": ([float(v) for v in np.percentile(nu[near], [50, 99, 100])]
                                   if near.any() else None),
           "innovation_abs_max_rad_s": float(np.abs(inn).max()),
           "over_gate": {str(g): int((nu >= g).sum()) for g in GATES},
           "over_gate_at_edges": {str(g): int((nu[near] >= g).sum()) for g in GATES},
           "gate_width_rad_s": {str(g): float(g * np.median(s)) for g in GATES}}
    big = np.argsort(-nu)[:5]
    out["largest"] = [{"t_s": float(t[k] - t0), "nu": float(nu[k]), "innovation": float(inn[k]),
                       "z": float(z[k]), "dt_to_prev_sample": float(t[k] - t[k - 1]) if k else None}
                      for k in big]
    odom = ~imu
    onu = np.abs(d["innovation"][odom]) / np.sqrt(d["p"][odom] + d["r"][odom])
    out["odom_nu_p50_p99_max"] = [float(v) for v in np.percentile(onu, [50, 99, 100])]
    gain = d["p"][odom] / (d["p"][odom] + d["r"][odom])
    out["odom_gain_p5_p50_p95"] = [float(v) for v in np.percentile(gain, [5, 50, 95])]
    return out


def rates_from_run(run_dir, hz=50.0):
    """/wheel/odom and /odometry/filtered yaw rate on one grid, plus the EKF yaw series."""
    from gripperx_localization.imu_replay import _reader
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from gripperx_localization.yaw_compare import yaw_from_quat
    r = _reader(Path(run_dir) / "out", ["/wheel/odom", "/odometry/filtered"])
    buf = {"/wheel/odom": [], "/odometry/filtered": []}
    while r.has_next():
        topic, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        q = m.pose.pose.orientation
        buf[topic].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                           m.twist.twist.angular.z, yaw_from_quat(q.x, q.y, q.z, q.w)))
    enc, ekf = (np.array(buf[k]) for k in ("/wheel/odom", "/odometry/filtered"))
    grid = np.arange(max(enc[0, 0], ekf[0, 0]), min(enc[-1, 0], ekf[-1, 0]), 1.0 / hz)
    return grid, np.interp(grid, enc[:, 0], enc[:, 1]), np.interp(grid, ekf[:, 0], ekf[:, 1]), ekf


def rate_lag(g, ref, src, hz=50.0, fine_hz=500.0, max_s=0.5):
    """Lag of src behind ref, s, by cross-correlation on a 500 Hz grid (0.2 s smoothing)."""
    from gripperx_localization.yaw_compare import _moving_average
    n = int(0.2 * hz)
    x, y = _moving_average(ref, n), _moving_average(src, n)
    x, y = np.nan_to_num(x - np.nanmean(x)), np.nan_to_num(y - np.nanmean(y))
    fine = np.arange(g[0], g[-1], 1.0 / fine_hz)
    fx, fy = np.interp(fine, g, x), np.interp(fine, g, y)
    m = int(max_s * fine_hz)
    lags = np.arange(-m, m + 1)
    c = [np.corrcoef(fx[m:-m], np.roll(fy, -k)[m:-m])[0, 1] for k in lags]
    return float(lags[int(np.argmax(c))] / fine_hz)


def encoder_lag(run_dir, t0_epoch, a, b, hz=50.0):
    """TWIST rate lag of the EKF behind the encoder in [a, b] s after bag start, and the
    delay at each 1 rad/s crossing of the encoder rate."""
    grid, enc, ekf, _ = rates_from_run(run_dir, hz)
    win = (grid >= t0_epoch + a) & (grid <= t0_epoch + b)
    g, e, f = grid[win], np.abs(enc[win]), np.abs(ekf[win])
    delays = []
    for k in np.flatnonzero((e[1:] >= 1.0) & (e[:-1] < 1.0)):
        later = np.flatnonzero(f[k:] >= 1.0)
        if len(later):
            delays.append(float(g[k + later[0]] - g[k]))
    return {"xcorr_lag_s": rate_lag(g, enc[win], ekf[win], hz), "rise_delays_s": delays}


def gyro_twist_lag(run_dir, t0_epoch, a, b, hz=50.0):
    """TWIST lag of the EKF behind /imu/data/filtered, the node's bias-free, base-signed
    gyro. yaw_compare's lag column is the POSE-derived rate instead."""
    from gripperx_localization.imu_replay import _reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    grid, _, ekf, _ = rates_from_run(run_dir, hz)
    r, rows = _reader(Path(run_dir) / "out", ["/imu/data/filtered"]), []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Imu)
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.z))
    samples = np.array(rows)
    win = (grid >= t0_epoch + a) & (grid <= t0_epoch + b)
    gyro = np.interp(grid[win], samples[:, 0], samples[:, 1])
    # Sign taken from the data: the output keeps the upside-down imu_link frame.
    sign = 1.0 if np.dot(gyro, ekf[win]) >= 0 else -1.0
    return rate_lag(grid[win], sign * gyro, ekf[win], hz)


def cumulative_slam_error(res):
    """slam heading minus truth after each segment, deg, summed from the bag start.

    Scan match where it passed; across its failed segments the gyro stands in, marked,
    which is acceptable only because slam, not the gyro, is what is judged here.
    """
    total, rows = 0.0, []
    for s in res["segments"]:
        src = s["sources"]
        if s["reference"] == "scanmatch":
            total += src["slam"]["error_deg"]
            rows.append(f"{s['index']}:{total:+.1f}")
        else:
            total += src["slam"]["increment_deg"] - src["gyro"]["increment_deg"]
            rows.append(f"{s['index']}g:{total:+.1f}")
    return rows


def robot_lock_lines(root):
    """Accepted scan sizes and start map->odom, robot bags and replays, cached in root."""
    from gripperx_localization.imu_replay_forensics import slam_lock
    cache = root / "slam_lock.json"
    res = json.loads(cache.read_text()) if cache.exists() else {}
    bags = {"robot drive15": "~/gripperx_bags/2026-10-22_drive15/drive15_20261022_1634",
            "robot rot13": "~/gripperx_bags/2026-10-22_rot13/rot13_20261022_1551"}
    # Runs whose lock is known from slam's own log validate the method on known answers.
    for r in ("drive15s454__B_baseline__1", "drive15s454__A_merged__1", "drive15s453__B_baseline__1",
              "rot13__B_baseline__1", "rot13s454__B_baseline__1", "rot13s454__A_merged__1"):
        bags[r] = str(root / "runs" / r / "out")
    for k, b in bags.items():
        if k not in res and Path(b).expanduser().exists():
            res[k] = slam_lock(str(Path(b).expanduser()))
    cache.write_text(json.dumps(res, indent=1))
    return [f"- slam lock {k}: {v}" for k, v in res.items()]


def independent_lines(root, groups):
    """EKF sums against the reference that shares no gyro and no ICP with the runs."""
    from gripperx_localization.imu_replay_forensics import independent_reference
    cache = root / "independent_reference.json"
    res = json.loads(cache.read_text()) if cache.exists() else {}
    w = []
    for key, inp, input_bag in (("drive15", "drive15s454", "drive15"), ("rot13", "rot13", "rot13")):
        if key not in res:
            res[key] = independent_reference(groups[inp]["A_merged"][0]["_dir"],
                                             root / "inputs" / input_bag)
            cache.write_text(json.dumps(res, indent=1))
        ref = res[key]
        d = [abs(v["increment_deg"] - v["scanmatch_deg"]) for v in ref.values() if v["scanmatch_deg"] is not None]
        w.append(f"- Independent reference {key}: |difference to scan match| median "
                 f"{np.median(d):.2f}, max {max(d):.2f} deg over {len(d)} segments.")
        for k, v in ref.items():
            if v["scanmatch_deg"] is None:
                w.append(f"- Independent reference {key} seg {k} (no scan match): "
                         f"{v['increment_deg']:+.2f} deg, score {v['score']:.3f}")
        for variant in ("A_merged", "Q_0p1", "Q_0p02"):
            if variant not in groups[inp]:
                continue
            r = groups[inp][variant][0]
            cells = []
            for label, segs in TURN_SUMS[inp].items():
                e = [seg(r, i)["sources"]["ekf"]["increment_deg"] - ref[str(i)]["increment_deg"]
                     for i in segs]
                cells.append(f"{label} {sum(e):+.2f}")
            spin = next(iter(PHASES[inp].values()))
            e = [abs(seg(r, i)["sources"]["ekf"]["increment_deg"] - ref[str(i)]["increment_deg"])
                 for i in spin]
            w.append(f"- EKF vs independent reference, {inp} {variant}: {', '.join(cells)}; "
                     f"spin median {np.median(e):.2f}, max {max(e):.2f}")
    return w


def supplement(root) -> str:
    """Fidelity against the robot's own output, fallback lag, standstill, arcs on a fresh map."""
    root = Path(root).expanduser()
    groups, _ = load_runs(root)
    w = []
    for bag, inp in (("drive15", "drive15"), ("rot13", "rot13")):
        old = json.loads((root / "old_stack" / f"{bag}.json").read_text())
        new = groups[inp]["B_baseline"][0]
        dif = [abs(a["sources"]["ekf"]["increment_deg"] - b["sources"]["ekf"]["increment_deg"])
               for a, b in zip(old["segments"], new["segments"])]
        w.append(f"- Fidelity {bag}: replayed B EKF vs the EKF recorded on the robot, per-segment "
                 f"increment: max difference {max(dif):.3f} deg, median {np.median(dif):.3f} deg "
                 f"over {len(dif)} segments.")
    old = json.loads((root / "old_stack" / "drive15.json").read_text())
    w.append(f"- Cumulative slam heading error, robot's own recording: {cumulative_slam_error(old)}")
    for inp in ("drive15s454", "drive15s453"):
        for v in ("B_baseline", "A_merged"):
            w.append(f"- Cumulative slam heading error, {inp} {v}: "
                     f"{cumulative_slam_error(groups[inp][v][0])}")
    t0 = groups["drive15s454"]["B_baseline"][0]["t0_epoch"]
    for label, inp, v in (("B (encoder only)", "drive15s454", "B_baseline"),
                          ("A, IMU silent from 120 s", "drive15_imu_silent", "A_silent"),
                          ("Q_vyaw 0.1, IMU silent from 120 s", "drive15_imu_silent", "Q_0p1_silent")):
        res = encoder_lag(groups[inp][v][0]["_dir"], t0, 125.0, 165.0)
        w.append(f"- Fallback TWIST lag {label}, 125-165 s, EKF vs encoder: "
                 f"{res['xcorr_lag_s']:+.3f} s; 1 rad/s rise delays "
                 f"{[round(x, 3) for x in res['rise_delays_s']]} s")
    for inp, v, a, b in (("drive15s454", "B_baseline", 42, 163), ("drive15s454", "A_merged", 42, 163),
                         ("drive15s454", "Q_0p1", 42, 163), ("drive15s454", "Q_0p02", 42, 163),
                         ("rot13", "B_baseline", 54, 161), ("rot13", "A_merged", 54, 161),
                         ("rot13", "Q_0p1", 54, 161)):
        r = groups[inp][v][0]
        lags = [seg(r, i)["sources"]["ekf"]["lag_s"] for i in next(iter(PHASES[inp].values()))]
        w.append(f"- Spin lag {inp} {v}, EKF vs gyro: TWIST "
                 f"{gyro_twist_lag(r['_dir'], r['t0_epoch'], a, b):+.3f} s; POSE-derived rate "
                 f"(yaw_compare) median {np.median(lags):+.3f} s")
    w.extend(robot_lock_lines(root))
    w.extend(independent_lines(root, groups))
    if "desk_slice" in groups:
        run = groups["desk_slice"]["A_still"][0]
        _, _, _, ekf = rates_from_run(run["_dir"])
        y = np.degrees(np.unwrap(ekf[:, 2]))
        t = ekf[:, 0] - ekf[0, 0]
        after = t >= 40.0
        slope = np.polyfit(t[after], y[after], 1)[0]
        w.append(f"- Standstill drift A (desk slice, {t[-1]:.0f} s): EKF yaw end minus start "
                 f"{y[-1] - y[0]:+.3f} deg, peak |yaw| {np.abs(y - y[0]).max():.3f} deg, slope after "
                 f"40 s {slope * 60:+.4f} deg/min.")
    for slice_key in ("arc_slice", "arc_shift"):
        for v, runs in groups.get(slice_key, {}).items():
            rows = [(s["index"], round(s["t0"], 1), s["reference"],
                     round(s["sources"]["ekf"]["error_deg"], 2) if s["reference"] == "scanmatch" else None,
                     round(s["sources"]["slam"]["error_deg"], 2) if s["reference"] == "scanmatch" else None)
                    for s in runs[0]["segments"]]
            med = np.median([abs(r[4]) for r in rows if r[0] >= 1 and r[4] is not None])
            w.append(f"- {slice_key} {v}, arc slam median {med:.2f} (seg, t0, ref, ekf err, "
                     f"slam err): {rows}")
    return "\n".join(w) + "\n"


def report(root) -> tuple:
    root = Path(root).expanduser()
    groups, skipped = load_runs(root)
    lines, data = [], {"skipped": skipped}
    w = lines.append
    if skipped:
        w(f"**Runs without a passed check, not used:** {', '.join(skipped)}\n")
    for inp, variants in groups.items():
        spread, nseg = reference_spread(variants)
        w(f"\n## {inp}\n")
        w(f"Scan-match reference: largest spread of one segment increment across "
          f"{sum(len(v) for v in variants.values())} runs {spread:.4f} deg; segment counts {nseg}.\n")
        data.setdefault(inp, {})["reference_spread_deg"] = spread
        sizes = {v: sorted({n for r in runs for n in scan_sizes(r["_dir"])})
                 for v, runs in variants.items()}
        w(f"slam's expected scan size per variant: {sizes}\n")
        key = inp if inp in PHASES else None
        if key:
            for source in ("ekf", "slam"):
                w(f"\n### {source}: median |error| / max |error| per phase, deg vs scan match; "
                  "mean [min..max] over repeats\n")
                names = list(variants)
                w("| Phase (n) | " + " | ".join(names) + " |")
                w("| --- |" + " --- |" * len(names))
                for ph, segs in PHASES[key].items():
                    n = phase_cells(next(iter(variants.values())), segs, source)[2]
                    cells = []
                    for v in names:
                        m, x, _ = phase_cells(variants[v], segs, source)
                        cells.append(f"{fmt_range(m)} / {fmt_range(x)}")
                        data[inp].setdefault(source, {}).setdefault(ph, {})[v] = {"med": m, "max": x}
                    w(f"| {ph} ({n}) | " + " | ".join(cells) + " |")
            w("\n### 3-turn yaw sums, deg vs scan match (D3 target |sum| < 2)\n")
            names = list(variants)
            w("| Direction, source | " + " | ".join(names) + " |")
            w("| --- |" + " --- |" * len(names))
            for label, segs in TURN_SUMS[key].items():
                for source in ("ekf", "slam", "gyro", "encoder"):
                    cells = []
                    for v in names:
                        vals = [x for x in (turn_sum(r, segs, source) for r in variants[v]) if x is not None]
                        cells.append(("-" if not vals else ", ".join(f"{x:+.2f}" for x in vals)))
                        data[inp].setdefault("turn_sums", {}).setdefault(f"{label} {source}", {})[v] = vals
                    w(f"| {label}, {source} | " + " | ".join(cells) + " |")
            w("\n### Segments without a scan match: increment per source, deg (no reference)\n")
            for i in NO_REFERENCE[key]:
                for v in names:
                    s = seg(variants[v][0], i)
                    if s:
                        src = {k: round(r["increment_deg"], 2) for k, r in s["sources"].items()
                               if r["increment_deg"] is not None and k != "encoder_twist"}
                        w(f"- seg {i} {v}: {src}")
            w("\n### Per-segment ekf error, deg: mean (run-to-run spread)\n")
            w("| seg | t0 | " + " | ".join(names) + " |")
            w("| --- | --- |" + " --- |" * len(names))
            ref_run = next(iter(variants.values()))[0]
            for s in ref_run["segments"]:
                i = s["index"]
                cells = []
                for v in names:
                    e = [x for x in (seg_err(r, i, "ekf") for r in variants[v]) if x is not None]
                    cells.append("-" if not e else f"{np.mean(e):+.2f} ({max(e) - min(e):.2f})")
                w(f"| {i} | {s['t0']:.0f} | " + " | ".join(cells) + " |")
            w("\n### ekf rate lag vs gyro in spin segments, s (A fuses the gyro: CIRCULAR for A/V/Q, "
              "independent only for B)\n")
            spin = next(iter(PHASES[key].values()))
            for v in names:
                lags = [seg(r, i)["sources"]["ekf"]["lag_s"] for r in variants[v] for i in spin
                        if seg(r, i) and seg(r, i)["sources"]["ekf"]["lag_s"] is not None]
                if lags:
                    w(f"- {v}: median {np.median(lags):+.3f}, range {min(lags):+.3f}..{max(lags):+.3f}")
        w("\n### slam correction effort (map->odom yaw), and CPU in core-seconds per second\n")
        w("| Variant | total |change| deg | max step deg | final deg | input | ekf | slam | load start |")
        w("| --- | --- | --- | --- | --- | --- | --- | --- |")
        for v, runs in variants.items():
            mo = [map_odom_motion(r["_dir"]) for r in runs]
            cpu = {k: [r["_meta"].get("cpu_core_avg", {}).get(k, float("nan")) for r in runs]
                   for k in ("input", "ekf", "slam")}
            load = [r["_meta"]["loadavg_start"][0] for r in runs]
            w(f"| {v} | {fmt_range([m['total_deg'] for m in mo], 1)} | "
              f"{fmt_range([m['max_step_deg'] for m in mo], 2)} | "
              f"{fmt_range([m['final_deg'] for m in mo], 2)} | {fmt_range(cpu['input'], 3)} | "
              f"{fmt_range(cpu['ekf'], 3)} | {fmt_range(cpu['slam'], 3)} | {fmt_range(load, 1)} |")
            data[inp].setdefault("cpu", {})[v] = cpu
            data[inp].setdefault("map_odom", {})[v] = mo
            for label in ("Failed to meet update rate", "dropping message", "queue is full"):
                n = [Path(r["_dir"], "stack.log").read_text(errors="replace").count(label) for r in runs]
                if any(n):
                    w(f"\n{v}: stack.log '{label}': {n}")
        for v, runs in variants.items():
            for r in runs:
                npz = Path(r["_dir"]) / "innovations.npz"
                if npz.exists():
                    data[inp].setdefault("innovations", {})[Path(r["_dir"]).name] = \
                        innovation_summary(npz, r["t0_epoch"])
    return "\n".join(lines) + "\n", data


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="~/gripperx_bags/replay_2p5")
    args = ap.parse_args(argv)
    text, data = report(args.out)
    Path(args.out).expanduser().joinpath("report.json").write_text(json.dumps(data, indent=1))
    sys.stdout.write(text)
    sys.stdout.write("\n## Supplement\n\n" + supplement(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
