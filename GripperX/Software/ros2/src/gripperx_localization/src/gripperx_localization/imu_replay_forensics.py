#!/usr/bin/env python3
"""Two checks of the 2.5 replay that need no replay. Both came out of its audit.

slam_lock(bag): which scan size did slam_toolbox accept in a bag? It rejects every scan
whose size differs from the first it processed, and a robot bag records no log, but each
map->odom VALUE change is one accepted scan, stamped a fixed offset ahead of it. So the
changes are matched to the scans and the sizes counted. Also the map->odom yaw at bag
start: not zero means slam had already been running on a map.

independent_reference(run_dir, input_bag): a second settled LiDAR reference, sharing
nothing with scan_match.py but the windows. Full-circle grid search on a truncated
distance field, no ICP, no gyro, no EKF; translation guess from the wheels, whole turns
from the encoder. It exists because a gyro-based turn count and an ICP are what the
primary reference and the A runs have in common. The port keeps the audit's constants:
they are CHOSEN (ranges below 0.35 m and the arm sector 140-220 deg are dropped).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

MIN_RANGE_M = 0.35
MAX_RANGE_M = 20.0
ARM_SECTOR_DEG = (140.0, 220.0)
FIELD_RES_M = 0.01
FIELD_TRUNC_M = 0.25
SCANS_PER_SIDE = 10
SETTLE_SKIP_S = 1.0


def _bag_scans(uri):
    from gripperx_localization.imu_replay import _reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    r, out = _reader(uri, ["/scan"]), []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, LaserScan)
        out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angle_min,
                    m.angle_increment, np.asarray(m.ranges, dtype=np.float32)))
    return out


def _map_odom(uri):
    from gripperx_localization.imu_replay import _reader
    from gripperx_localization.yaw_compare import yaw_from_quat
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    r, rows = _reader(uri, ["/tf"]), []
    while r.has_next():
        _, raw, _ = r.read_next()
        for tr in deserialize_message(raw, TFMessage).transforms:
            if (tr.header.frame_id, tr.child_frame_id) == ("map", "odom"):
                q, t = tr.transform.rotation, tr.transform.translation
                rows.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                             yaw_from_quat(q.x, q.y, q.z, q.w), t.x, t.y))
    return np.array(rows)


def match_changes(change_stamps, scan_stamps, offsets=np.arange(0.0, 0.8, 0.01)):
    """-> (offset, median residual s, index of the nearest scan per change)."""
    change_stamps, scan_stamps = np.asarray(change_stamps), np.asarray(scan_stamps)
    best = None
    for off in offsets:
        q = change_stamps - off
        j = np.clip(np.searchsorted(scan_stamps, q), 1, len(scan_stamps) - 1)
        j = np.where(np.abs(scan_stamps[j - 1] - q) < np.abs(scan_stamps[j] - q), j - 1, j)
        m = float(np.median(np.abs(scan_stamps[j] - q)))
        if best is None or m < best[1]:
            best = (float(off), m, j)
    return best


def slam_lock(bag) -> dict:
    scans = _bag_scans(bag)
    ts = np.array([s[0] for s in scans])
    sizes = np.array([len(s[3]) for s in scans])
    mo = _map_odom(bag)
    mo = mo[np.argsort(mo[:, 0], kind="stable")]
    moved = (np.abs(np.diff(mo[:, 1])) > 1e-9) | (np.abs(np.diff(mo[:, 2])) > 1e-9)
    ch = np.flatnonzero(moved) + 1
    off, resid, j = match_changes(mo[ch, 0], ts)
    got = sizes[j]
    return {"scans": {int(k): int((sizes == k).sum()) for k in np.unique(sizes)},
            "corrections": int(len(ch)), "offset_s": off, "residual_ms": resid * 1e3,
            "accepted_sizes": {int(k): int((got == k).sum()) for k in np.unique(got)},
            "map_odom_yaw_at_start_deg": float(math.degrees(mo[0, 1]))}


# ------------------------------------------------------------- independent reference

def scan_points(angle_min, angle_inc, ranges, mount_xy):
    ranges = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(ranges))
    ok = np.isfinite(ranges) & (ranges >= MIN_RANGE_M) & (ranges <= MAX_RANGE_M)
    d = np.degrees(a) % 360.0
    ok &= ~((d >= ARM_SECTOR_DEG[0]) & (d <= ARM_SECTOR_DEG[1]))
    return np.c_[ranges[ok] * np.cos(a[ok]) + mount_xy[0], ranges[ok] * np.sin(a[ok]) + mount_xy[1]]


class DistanceField:
    def __init__(self, pts):
        from scipy.ndimage import distance_transform_edt
        self.origin = pts.min(0) - 1.0
        ij = ((pts - self.origin) / FIELD_RES_M).astype(int)
        self.shape = ij.max(0) + int(2 / FIELD_RES_M)
        free = np.ones(self.shape, bool)
        free[ij[:, 0], ij[:, 1]] = False
        self.field = np.minimum(distance_transform_edt(free) * FIELD_RES_M, FIELD_TRUNC_M)

    def score(self, pts):
        ij = ((pts - self.origin) / FIELD_RES_M).astype(int)
        ok = (ij >= 0).all(1) & (ij < self.shape).all(1)
        v = np.full(len(pts), FIELD_TRUNC_M)
        v[ok] = self.field[ij[ok, 0], ij[ok, 1]]
        return float(v.mean())


def _rot(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def match_full_circle(before, after, t_guess):
    """Yaw (deg, -180..180) carrying `after` into `before`'s frame, and its score."""
    f = DistanceField(before)
    coarse = after[::max(1, len(after) // 300)]
    grid = []
    for yaw in np.arange(-180.0, 180.0, 2.0):
        turned = coarse @ _rot(yaw).T
        for dx in np.arange(-0.3, 0.31, 0.1):
            for dy in np.arange(-0.3, 0.31, 0.1):
                grid.append((f.score(turned + t_guess + [dx, dy]), yaw, dx, dy))
    grid.sort()
    starts = []
    for row in grid[:40]:
        if all(abs(((row[1] - s[1] + 180.0) % 360.0) - 180.0) >= 8.0 for s in starts):
            starts.append(row)
        if len(starts) == 3:
            break
    results = []
    for _, yaw, dx, dy in starts:
        step_yaw, step_xy, t = 2.0, 0.1, np.asarray(t_guess, float) + [dx, dy]
        for _ in range(7):
            cand = []
            for y in yaw + np.linspace(-step_yaw, step_yaw, 9):
                turned = after @ _rot(y).T
                for ax in np.linspace(-step_xy, step_xy, 5):
                    for ay in np.linspace(-step_xy, step_xy, 5):
                        cand.append((f.score(turned + t + [ax, ay]), y, ax, ay))
            sc, yaw, ax, ay = min(cand)
            t = t + [ax, ay]
            step_yaw, step_xy = step_yaw / 2.2, step_xy / 2.0
        results.append((sc, yaw))
    return min(results)


def _wheel_track(run_dir):
    from gripperx_localization.imu_replay import _reader
    from gripperx_localization.yaw_compare import yaw_from_quat
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    r, rows = _reader(Path(run_dir) / "out", ["/wheel/odom"]), []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y,
                     yaw_from_quat(q.x, q.y, q.z, q.w)))
    a = np.array(rows)
    a[:, 3] = np.unwrap(a[:, 3])
    return a


def independent_reference(run_dir, input_bag) -> dict:
    """Per segment index: yaw increment, score, and the scan-match value beside it."""
    run_dir = Path(run_dir)
    yc = json.loads((run_dir / "yc.json").read_text())
    t0 = yc["t0_epoch"]
    mount = yc["lidar_mount"]["xyz"][:2]
    windows = [c for c in yc["settle_candidates"] if c["scan_ok"]]
    scans = _bag_scans(input_bag)
    ts = np.array([s[0] for s in scans])
    wheel = _wheel_track(run_dir)
    out = {}
    for k, seg in enumerate(yc["segments"]):
        sides = []
        for c in (windows[k], windows[k + 1]):
            idx = np.flatnonzero((ts >= t0 + c["t_start"] + SETTLE_SKIP_S)
                                 & (ts <= t0 + c["t_end"]))[-SCANS_PER_SIDE:]
            pts = np.vstack([scan_points(scans[i][1], scans[i][2], scans[i][3], mount) for i in idx])
            sides.append((pts, float(ts[idx].mean())))
        (before, ta), (after, tb) = sides
        at = lambda t, col: float(np.interp(t, wheel[:, 0], wheel[:, col]))  # noqa: E731
        shift = np.array([at(tb, 1) - at(ta, 1), at(tb, 2) - at(ta, 2)])
        guess = shift @ _rot(math.degrees(at(ta, 3)))
        score, yaw = match_full_circle(before, after, guess)
        encoder = math.degrees(at(tb, 3) - at(ta, 3))
        yaw += 360.0 * round((encoder - yaw) / 360.0)
        out[str(seg["index"])] = {"increment_deg": yaw, "score": score,
                                  "scanmatch_deg": (seg["reference_increment_deg"]
                                                    if seg["reference"] == "scanmatch" else None)}
    return out


# ---------------------------------------------------------- effective time of a scan

PROFILE_STEP_DEG = 0.25
MAX_SCAN_STEP_DEG = 15.0
SCAN_MIN_M, SCAN_MAX_M = 0.3, 12.0


def range_profile(angle_min, angle_inc, ranges):
    """log range on a fixed azimuth grid, sensor frame. Sizes differ scan to scan."""
    ranges = np.asarray(ranges, dtype=float)
    a = np.degrees(angle_min + angle_inc * np.arange(len(ranges))) % 360.0
    ok = np.isfinite(ranges) & (ranges > SCAN_MIN_M) & (ranges < SCAN_MAX_M)
    order = np.argsort(a[ok])
    grid = np.arange(0.0, 360.0, PROFILE_STEP_DEG)
    return np.log(np.interp(grid, a[ok][order], ranges[ok][order], period=360.0))


def azimuth_step(p, q):
    """Yaw of the sensor from profile p to profile q, deg. nan if at the search limit.

    In-place only: a turn of the sensor is a shift of the profile. Consecutive scans,
    so the lever arm of the mount does not enter.
    """
    n = int(MAX_SCAN_STEP_DEG / PROFILE_STEP_DEG)
    shifts = np.arange(-n, n + 1)
    cost = np.array([np.mean((np.roll(p, -k) - q) ** 2) for k in shifts])
    i = int(np.argmin(cost))
    if i in (0, len(cost) - 1):
        return float("nan")
    den = cost[i - 1] - 2.0 * cost[i] + cost[i + 1]
    return float((shifts[i] + 0.5 * (cost[i - 1] - cost[i + 1]) / den) * PROFILE_STEP_DEG)


def fit_scan_offset(ts, steps, t_src, yaw_src, mask, taus=np.arange(-0.3, 0.1501, 0.005)):
    """tau with scan yaw steps best explained by the source's yaw at stamp + tau.

    Only rate CHANGES carry information: at a constant rate every tau gives the same
    step. -> (best tau, mean |residual| at it, at tau 0), deg per scan.
    """
    ts, steps = np.asarray(ts), np.asarray(steps)
    use = np.flatnonzero(mask & np.isfinite(steps))

    def resid(tau):
        d = np.interp(ts[1:] + tau, t_src, yaw_src) - np.interp(ts[:-1] + tau, t_src, yaw_src)
        return float(np.mean(np.abs(steps[use] - d[use])))
    vals = [resid(t) for t in taus]
    k = int(np.argmin(vals))
    return float(taus[k]), vals[k], resid(0.0)


def scan_time_offset(bag, gyro_sign=-1.0, min_rate=0.1) -> dict:
    """Effective time of a /scan against its stamp on the gyro's clock, from raw topics.

    Negative: the scan shows the world earlier than its stamp says. gyro_sign turns the
    raw z of the upside-down imu_link into base yaw. In-place spins are what this
    resolves; arcs add translation to the profile shift.
    """
    from gripperx_localization.imu_replay import imu_series
    scans = _bag_scans(bag)
    ts = np.array([s[0] for s in scans])
    profiles = [range_profile(s[1], s[2], s[3]) for s in scans]
    steps = np.array([azimuth_step(a, b) for a, b in zip(profiles, profiles[1:])])
    t_g, z_g = imu_series(bag, "/imu/bno085/data")
    yaw = np.degrees(np.concatenate(
        [[0.0], np.cumsum(0.5 * gyro_sign * (z_g[1:] + z_g[:-1]) * np.diff(t_g))]))
    rate = gyro_sign * np.interp(ts[1:], t_g, z_g)
    moving = np.abs(rate) > min_rate
    dt = np.diff(ts)
    res = {"n_scans": len(ts), "dt_median_s": float(np.median(dt)),
           "skipped_ticks_at_s": [float(ts[i] - ts[0]) for i in np.flatnonzero(dt > 0.15)],
           "scan_rate_hz": float((len(ts) - 1) / (ts[-1] - ts[0]))}
    for label, m in (("all", moving), ("ccw", moving & (rate > 0)), ("cw", moving & (rate < 0))):
        tau, r, r0 = fit_scan_offset(ts, steps, t_g, yaw, m)
        res[label] = {"tau_s": tau, "n": int(m.sum()), "resid_deg": r, "resid_at_0_deg": r0}
    # One estimate per motion burst, to see whether the offset drifts.
    edges = np.flatnonzero(np.diff(np.concatenate([[0], moving.astype(int), [0]])))
    bursts = []
    for lo, hi in zip(edges[::2], edges[1::2]):
        if hi - lo < 20:
            continue
        m = np.zeros(len(moving), bool)
        m[max(lo - 5, 0):hi + 5] = True
        tau, r, r0 = fit_scan_offset(ts, steps, t_g, yaw, m)
        bursts.append({"t_s": float(ts[lo] - ts[0]), "n": int(hi - lo), "tau_s": tau,
                       "resid_deg": r, "resid_at_0_deg": r0})
    res["bursts"] = bursts
    return res
