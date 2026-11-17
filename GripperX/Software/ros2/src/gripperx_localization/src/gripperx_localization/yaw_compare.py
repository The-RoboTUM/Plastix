#!/usr/bin/env python3
"""Turn one bag into a per-phase yaw agreement table: encoders, gyro, LiDAR.

WHY THIS EXISTS. Wheel odometry is the EKF's (extended Kalman filter's) only yaw
source, and it over-reports in-place rotation by ~9 %. The user's hypothesis is
that the gyro (the IMU's, inertial measurement unit's, angular-rate sensor) is the
better yaw-rate source and a settled LiDAR pose the right absolute anchor. That is
to be measured before anything is fused: this tool is <<FR+1>> (yaw agreement
analysis) and <<FR+2>> (settled LiDAR reference) of the local tracking issue
claude/docs/ISSUE_DRAFT_LIDAR_IMU_2026-10-22.md, sub-issues 1.4 and 1.4b.

WHAT IT COMPARES. Every source is expressed as base_footprint yaw, unwrapped:
    encoder        /wheel/odom pose yaw. This is the node's own integral over its
                   compensated rate interval (stale-sample hold-back included),
                   i.e. exactly what the encoders said, with no resampling of ours.
    encoder_twist  APPROXIMATE. /wheel/odom twist.angular.z integrated over
                   header-stamp intervals - roughly what an EKF fusing vyaw (yaw
                   rate) integrates. The node computed that rate over its
                   stale-sample-compensated rate_dt, not over the stamp interval,
                   so this track is not what the encoders said; `encoder` is the
                   authoritative encoder track. Reported so that the difference
                   stays visible (up to ~3.5 deg per segment on the 2026-10-08 bags).
    gyro           IMU angular velocity rotated into base_footprint with the
                   mounting read from /tf_static (the IMU is mounted rolled 180 deg,
                   so z flips - the sign comes from the TF, never from this file),
                   integrated; per segment the mean rate over the settle window
                   preceding it is subtracted as bias.
    ekf            odom->base_footprint yaw from /tf: the EKF alone, without the
                   map correction. Added for the A/B replay (2.5), where it is the
                   track under test.
    slam           map->odom (slam_toolbox) composed with odom->base_footprint
                   (EKF), both from /tf.
    laser_odom     /laser/odom pose yaw (ros2_laser_scan_matcher).
    scanmatch      not a track: per segment, the last /scan messages of the
                   standstill before are matched against the last ones of the
                   standstill after (scan_match.py, sub-issue 1.4b, decision D6).
                   Whole turns come from the gyro (else encoder) increment, only
                   the fraction of a turn from the scans - see scan_match.py.

REFERENCE (--reference, sub-issue 1.4b). `scanmatch` (default): the settled
LiDAR value of D6, read at standstill; where it FAILED for a segment, that
segment falls back to `slam` and says so. `slam`: the slam_toolbox pose, which is
frozen at standstill (see KNOWN LIMIT below). `floor`: the operator's floor mark.
All three are reported side by side whatever the choice; the choice only sets
which one the `err` and `scale` columns are taken against.

SEGMENT BOUNDARIES. With /scan in the bag and a reference other than slam, a
standstill bounds a segment when it holds enough scans after the motion-end
margin; the slam sigma_settle criterion is then reported, not applied (it cannot
fail, KNOWN LIMIT below). Without /scan, or with --reference slam, the slam
criterion decides as before.

WHAT IT DOES NOT DO. It does not decide which source is right. The reference is
chosen by the caller, not proven correct; an optional floor-mark value per
segment (the operator's ground truth) is compared alongside. Segmentation uses
only wheel odometry, steering angles and the motion command - never the gyro or
the LiDAR, which are under test.

MAP->ODOM STAMPS ARE FUTURE-DATED. Measured (audit 2026-10-22): the map->odom
stamp minus the latest scan stamp is 0.4 or 0.5 s exactly, not the 0.2 s
transform_timeout default; the cause is TO-VERIFY (the running slam_toolbox
transform_timeout was not read). No correction is applied. Consequences: a lag
measured against slam is shifted by this offset, and a late slam correction can
land inside the following standstill window, i.e. be attributed to the next
segment.

PHASES ON REAL DATA. Every segment of the 2026-10-07/08 bags comes out "mixed":
Nav2 goals turn and drive in one go. The per-phase split is so far shown on
synthetic data only (test/test_yaw_compare.py). The 1.5 drive programme
separates phases with >= 5 s standstills, which should yield pure segments.

KNOWN LIMIT OF THE SETTLE CRITERION, measured on the 2026-10-08 coast bag: at
standstill slam_toolbox processes no scan (minimum_travel_distance/heading), so
map->odom does not change and the yaw scatter is ~0 by construction. A pose that
passes T_settle/sigma_settle can therefore still carry the uncorrected rotation
of the preceding motion. Each candidate reports the age of the last map->odom
change (from the window end and from motion end) and the /laser/odom yaw scatter
over the same window, so that this is visible; --require-slam-update makes the
update a criterion. The user's answer is decision D6: the reference became the
scan match at standstill (scanmatch, above); this slam criterion now only applies
under --reference slam or without /scan.

GYRO AT STANDSTILL. Every standstill gets its own gyro mean/std line,
independent of segments, so a bag with a single standstill still yields a bias.
A standstill whose gyro samples are all EXACTLY zero on all axes is flagged
"gyro output exactly zero" and given no bias: that is a dead or unread sensor,
not a perfect one (seen on a real 2026-10-22 bag, 1446 samples). A short
all-zero WINDOW (the 2-s bias window) is not that: a live quantised gyro at rest
gives it routinely (94.6 % zero samples on the 2026-10-22 desk bag). Such a
window counts as alive when its standstill run has non-zero samples and a
quantisation step is detected in the run (or, if the run has too few non-zero
samples, in the whole bag); its bias is then the window mean, possibly exactly
0, marked resolution-limited, and the gyro still gives the turn count.
The quantisation step is detected per standstill from the raw IMU axis that
dominates base z (BNO085 calibrated gyro: 1/512 rad/s = 0.112 deg/s, measured in
sub-issue 1.2). Where the standstill std is below one step, the bias estimate is
resolution-limited, and the line says so.

--split-at T marks [T - 0.5, T + 0.5] s as motion. It exists to cut one long
standstill into two for a noise-floor check of the references; never use it on
a real motion bag without saying so.

EVERY THRESHOLD BELOW IS CHOSEN, NOT DERIVED - TO-VERIFY from the 1.5 drive
programme bags (report 1.6). They are CLI parameters for that reason.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

from gripperx_localization import scan_match as sm

ZERO_FLAG = "gyro output exactly zero"
SOURCES = ("encoder", "encoder_twist", "gyro", "ekf", "slam", "laser_odom", "scanmatch")
TRACKED = SOURCES[:-1]            # scanmatch exists per segment only, not as a track
REFERENCES = ("scanmatch", "slam", "floor")

# Stamp origin per input, as far as it is known from the source code. The measured
# receive-minus-stamp offsets are reported beside these; where the two disagree,
# the measurement wins.
STAMP_NOTES = {
    "wheel_odom": "copies the /joint_states stamp = controller READ time, not ESP32 "
                  "sample time (stale-sample note, config/localization.yaml)",
    "joint_states": "controller read time (joint_state_broadcaster), not ESP32 sample time",
    "imu": "bno085_node: get_clock().now() after the I2C read (Pi clock)",
    "map_odom": "slam_toolbox; stamp = latest scan stamp + 0.4/0.5 s (MEASURED); cause "
                "TO-VERIFY (running transform_timeout unread); not corrected - shifts "
                "lag vs slam",
    "odom_base": "ekf_filter_node filter time",
    "encoder_twist": "APPROXIMATE: integrated over stamp intervals, not the node's "
                     "compensated rate_dt; `encoder` is authoritative",
    "laser_odom": "ros2_laser_scan_matcher; stamp origin not verified from source",
    "cmd": "geometry_msgs/Twist has no header: bag RECEIVE time used",
    "scan": "LaserScan header stamp (lidar driver); origin not verified from source",
}


@dataclass
class Params:
    """All thresholds of the analysis. Defaults CHOSEN, TO-VERIFY (see module doc)."""

    grid_hz: float = 50.0               # common time grid; the IMU publishes at 50 Hz
    v_still: float = 0.01               # m/s, base speed below which it does not translate
    w_still_deg: float = 1.0            # deg/s, encoder yaw rate below which it does not turn
    cmd_eps: float = 1e-3               # |command| above this counts as an active command
    cmd_timeout_sec: float = 0.5        # a held command older than this counts as zero
    class_smooth_sec: float = 0.3       # moving average on the twist used for classification
    rate_smooth_sec: float = 0.2        # moving average on every yaw rate before correlation
    kappa_straight: float = 0.1         # rad/m, |curvature| below this counts as straight
    steer_const_tol_deg: float = 3.0    # p5-p95 steer range within which a curve is "constant"
    purity: float = 0.8                 # share of one phase in a segment's motion part
    min_phase_sec: float = 0.5          # shorter label runs are absorbed by their predecessor
    t_settle: float = 2.0               # s of standstill before a slam pose counts (<<FR+2>>)
    sigma_settle_deg: float = 0.2       # slam yaw std over that window (<<FR+2>>)
    require_slam_update: bool = False   # also demand a map->odom change during the standstill
    max_lag_sec: float = 1.0            # cross-correlation search range
    min_lag_rate_deg: float = 2.0       # deg/s, reference-rate std below which no lag is given
    min_scale_increment_deg: float = 10.0  # |reference increment| below which no scale is given
    max_gap_sec: float = 0.5            # a source silent longer than this marks a gap
    imu_rpy: Optional[Tuple[float, float, float]] = None  # mounting override, radians
    reference: str = "scanmatch"        # scanmatch (falls back to slam) | slam | floor
    split_at: Tuple[float, ...] = ()    # s after bag start: artificial standstill breaks
    scan: sm.ScanMatchParams = field(default_factory=sm.ScanMatchParams)


# --------------------------------------------------------------------------- maths

def quat_from_rpy(r: float, p: float, y: float):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quat(x, y, z, w):
    """Works on scalars and numpy arrays alike."""
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def static_rotation(static_tf: Dict[str, Tuple[str, tuple]], target: str, source: str):
    """Rotation q with v_target = q * v_source, from parent->child static edges.

    static_tf maps child -> (parent, quaternion of the child in the parent).
    Raises KeyError if the two frames are not connected.
    """
    def chain(frame):
        out = [(frame, (0.0, 0.0, 0.0, 1.0))]
        q = (0.0, 0.0, 0.0, 1.0)
        seen = {frame}
        while frame in static_tf:
            parent, q_pc = static_tf[frame]
            q = quat_mul(q_pc, q)          # q_parent_source
            frame = parent
            if frame in seen:
                raise KeyError(f"TF loop at {frame}")
            seen.add(frame)
            out.append((frame, q))
        return out

    up_src = chain(source)
    up_tgt = dict(chain(target))
    for frame, q_frame_src in up_src:
        if frame in up_tgt:
            return quat_mul(quat_conj(up_tgt[frame]), q_frame_src)
    raise KeyError(f"no static TF path {target} <- {source}")


def static_transform(static_tf, static_xyz, target: str, source: str):
    """Full static transform: v_target = R v_source + t. -> (R 3x3, t 3).

    static_xyz maps child -> translation of the child in its parent. Raises
    KeyError if the frames are not connected or a translation on the path is
    missing.
    """
    def chain(frame):
        out = [(frame, np.eye(4))]
        m = np.eye(4)
        seen = {frame}
        while frame in static_tf:
            parent, q_pc = static_tf[frame]
            if frame not in static_xyz:
                raise KeyError(f"no static translation for {frame}")
            e = np.eye(4)
            e[:3, :3] = quat_to_matrix(q_pc)
            e[:3, 3] = static_xyz[frame]
            m = e @ m                      # T_parent_source
            frame = parent
            if frame in seen:
                raise KeyError(f"TF loop at {frame}")
            seen.add(frame)
            out.append((frame, m))
        return out

    up_tgt = dict(chain(target))
    for frame, m_src in chain(source):
        if frame in up_tgt:
            m = np.linalg.inv(up_tgt[frame]) @ m_src
            return m[:3, :3], m[:3, 3]
    raise KeyError(f"no static TF path {target} <- {source}")


def quant_step(values: np.ndarray) -> Optional[float]:
    """Common quantisation step of `values`, or None if they are not quantised.

    Candidate: the smallest gap between distinct values (or smallest non-zero
    magnitude), or 1/2, 1/3 of it; it must divide every value to 1e-6 of a step.
    Needs >= 5 non-zero samples and at most half as many distinct values as
    samples - continuous data must never yield a "step" of float resolution.
    """
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if np.count_nonzero(v) < 5:
        return None
    u = np.unique(v)
    if len(u) > 0.5 * len(v):
        return None
    gaps = np.concatenate([np.diff(u), np.abs(u[u != 0.0])])
    q0 = float(gaps.min())
    if q0 <= 1e-9 * float(np.abs(u).max()):
        return None
    for div in (1, 2, 3):
        q = q0 / div
        k = v / q
        if np.all(np.abs(k - np.round(k)) < 1e-6):
            return q
    return None


def _moving_average(y: np.ndarray, n: int) -> np.ndarray:
    """NaN-aware centred moving average over n samples."""
    if n <= 1:
        return y.copy()
    valid = np.isfinite(y)
    k = np.ones(n)
    s = np.convolve(np.where(valid, y, 0.0), k, mode="same")
    c = np.convolve(valid.astype(float), k, mode="same")
    out = np.full_like(y, np.nan, dtype=float)
    ok = (c > 0) & valid
    out[ok] = s[ok] / c[ok]
    return out


def _moving_max_abs(y: np.ndarray, n: int) -> np.ndarray:
    """Centred moving maximum of |y| over n samples (NaN counts as 0)."""
    a = np.abs(np.nan_to_num(y))
    if n <= 1:
        return a
    h = n // 2
    padded = np.pad(a, (h, n - 1 - h), mode="edge")
    return np.lib.stride_tricks.sliding_window_view(padded, n).max(axis=1)


def _clean(t, *cols):
    """Sort by stamp and drop duplicated stamps (keep the last received)."""
    t = np.asarray(t, dtype=float)
    order = np.argsort(t, kind="stable")
    t = t[order]
    cols = [np.asarray(c, dtype=float)[order] for c in cols]
    if len(t) > 1:
        keep = np.append(np.diff(t) > 0, True)
        t = t[keep]
        cols = [c[keep] for c in cols]
    return (t, *cols)


def _interp(t_src, y_src, grid, max_gap):
    """Linear interpolation; NaN outside the source span. Second return: gap mask."""
    y = np.interp(grid, t_src, y_src, left=np.nan, right=np.nan)
    gap = np.zeros(len(grid), dtype=bool)
    if len(t_src) >= 2:
        idx = np.clip(np.searchsorted(t_src, grid), 1, len(t_src) - 1)
        span = t_src[idx] - t_src[idx - 1]
        inside = (grid >= t_src[0]) & (grid <= t_src[-1])
        gap = inside & (span > max_gap)
    return y, gap


def cross_correlation_lag(ref: np.ndarray, src: np.ndarray, max_lag_samples: int):
    """Lag in samples by which `src` trails `ref` (positive = src later).

    Sub-sample peak by parabolic interpolation. Returns (lag, peak_correlation,
    at_limit) or None if there is nothing to correlate.
    """
    valid = np.isfinite(ref) & np.isfinite(src)
    if valid.sum() < 2 * max_lag_samples + 3:
        return None
    a = np.where(valid, ref - np.nanmean(ref[valid]), 0.0)
    b = np.where(valid, src - np.nanmean(src[valid]), 0.0)
    n = len(a)
    lags = np.arange(-max_lag_samples, max_lag_samples + 1)
    c = np.empty(len(lags))
    for i, L in enumerate(lags):
        if L >= 0:
            x, y = a[:n - L], b[L:]
        else:
            x, y = a[-L:], b[:n + L]
        den = math.sqrt(float(np.dot(x, x)) * float(np.dot(y, y)))
        c[i] = float(np.dot(x, y)) / den if den > 0 else 0.0
    k = int(np.argmax(c))
    lag = float(lags[k])
    at_limit = k == 0 or k == len(c) - 1
    if not at_limit:
        c0, c1, c2 = c[k - 1], c[k], c[k + 1]
        den = c0 - 2 * c1 + c2
        if den != 0:
            lag += 0.5 * (c0 - c2) / den
    return lag, float(c[k]), at_limit


# --------------------------------------------------------------------------- data

@dataclass
class BagData:
    """Everything the analysis reads, as plain arrays. None = not in the bag.

    Times are epoch seconds from header stamps (cmd: receive time).
      wheel_odom  {t, yaw, vx, vy, wz[, x, y]}
      steer       {t, angles (N, k), names}
      imu         {t, w (N, 3), frame_id}
      map_odom    {t, yaw, x, y}       odom_base {t, yaw}
      laser_odom  {t, yaw}             cmd       {t, vx, vy, wz}
      scans       {t, ranges [list of arrays], angle_min, angle_inc, range_min,
                   range_max (arrays, one per scan), frame_id}
    static_tf: child -> (parent, quaternion xyzw)
    static_xyz: child -> translation (x, y, z) in the parent
    clock: role -> {topic, n, recv_minus_stamp_median_s, recv_minus_stamp_p95_abs_s}
    """

    t0: float = 0.0
    wheel_odom: Optional[dict] = None
    steer: Optional[dict] = None
    imu: Optional[dict] = None
    map_odom: Optional[dict] = None
    odom_base: Optional[dict] = None
    laser_odom: Optional[dict] = None
    cmd: Optional[dict] = None
    scans: Optional[dict] = None
    static_tf: Dict[str, tuple] = field(default_factory=dict)
    static_xyz: Dict[str, tuple] = field(default_factory=dict)
    clock: Dict[str, dict] = field(default_factory=dict)
    base_frame: str = "base_footprint"
    label: str = ""


def _nonempty(d):
    return d is not None and len(d.get("t", [])) >= 2


# --------------------------------------------------------------------------- tracks

def build_tracks(data: BagData, p: Params):
    """-> grid, tracks {source: yaw}, rates {source: smoothed rate}, gaps, sources info."""
    info = {s: {"present": False, "reason": ""} for s in SOURCES}
    spans = []
    for d in (data.wheel_odom, data.imu, data.odom_base, data.laser_odom):
        if _nonempty(d):
            spans.append((float(np.min(d["t"])), float(np.max(d["t"]))))
    if not spans:
        raise ValueError("bag holds none of /wheel/odom, IMU, odom->base TF, /laser/odom")
    # Segmentation runs on wheel odometry, so its span is the grid when it exists.
    if _nonempty(data.wheel_odom):
        t_lo, t_hi = float(np.min(data.wheel_odom["t"])), float(np.max(data.wheel_odom["t"]))
    else:
        t_lo, t_hi = min(s[0] for s in spans), max(s[1] for s in spans)
    dt = 1.0 / p.grid_hz
    grid = np.arange(t_lo, t_hi + 0.5 * dt, dt)
    n_avg = max(1, int(round(p.rate_smooth_sec * p.grid_hz)))

    tracks, rates, raw_rates, gaps = {}, {}, {}, {}

    def from_yaw(name, t, yaw):
        t, yaw = _clean(t, yaw)
        y, gap = _interp(t, np.unwrap(yaw), grid, p.max_gap_sec)
        tracks[name], gaps[name] = y, gap
        rates[name] = _moving_average(np.gradient(y, dt), n_avg)
        info[name]["present"] = True

    if _nonempty(data.wheel_odom):
        w = data.wheel_odom
        from_yaw("encoder", w["t"], w["yaw"])
        t, wz = _clean(w["t"], w["wz"])
        steps = np.diff(t, prepend=t[0])
        steps[steps > p.max_gap_sec] = 0.0     # no rate is held across a gap
        integ = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * steps[1:])])
        y, gap = _interp(t, integ, grid, p.max_gap_sec)
        r, _ = _interp(t, wz, grid, p.max_gap_sec)
        tracks["encoder_twist"], gaps["encoder_twist"] = y, gap
        rates["encoder_twist"] = _moving_average(r, n_avg)
        info["encoder_twist"]["present"] = True
    else:
        for s in ("encoder", "encoder_twist"):
            info[s]["reason"] = "no /wheel/odom in bag"

    mount = None
    raw_imu = None
    if _nonempty(data.imu):
        t, wx, wy, wzi = _clean(data.imu["t"], *np.asarray(data.imu["w"], dtype=float).T)
        raw_imu = {"t": t, "wz_base": None, "zero": (wx == 0.0) & (wy == 0.0) & (wzi == 0.0),
                   "w": np.column_stack([wx, wy, wzi]), "axis": 2}
        frame = data.imu.get("frame_id", "imu_link")
        try:
            if p.imu_rpy is not None:
                q = quat_from_rpy(*p.imu_rpy)
                mount = {"from": "--imu-rpy override", "frame": frame}
            else:
                q = static_rotation(data.static_tf, data.base_frame, frame)
                mount = {"from": f"/tf_static {data.base_frame} <- {frame}", "frame": frame}
            R = quat_to_matrix(q)
            mount["quaternion_xyzw"] = [float(v) for v in q]
            mount["z_row"] = [float(v) for v in R[2]]
            raw_imu["axis"] = int(np.argmax(np.abs(R[2])))   # quantisation is per raw axis
            wz_base = R[2, 0] * wx + R[2, 1] * wy + R[2, 2] * wzi
            raw_imu["wz_base"] = wz_base
            steps = np.diff(t, prepend=t[0])
            steps[steps > p.max_gap_sec] = 0.0
            integ = np.concatenate([[0.0], np.cumsum(0.5 * (wz_base[1:] + wz_base[:-1]) * steps[1:])])
            y, gap = _interp(t, integ, grid, p.max_gap_sec)
            r, _ = _interp(t, wz_base, grid, p.max_gap_sec)
            tracks["gyro"], gaps["gyro"] = y, gap      # raw integral; bias per segment
            raw_rates["gyro"] = r
            rates["gyro"] = _moving_average(r, n_avg)
            info["gyro"]["present"] = True
        except KeyError as exc:
            info["gyro"]["reason"] = f"IMU present but mounting unknown: {exc}"
    else:
        info["gyro"]["reason"] = "no IMU topic in bag"

    if _nonempty(data.odom_base):
        from_yaw("ekf", data.odom_base["t"], data.odom_base["yaw"])
    else:
        info["ekf"]["reason"] = "no odom->base in /tf"

    if _nonempty(data.map_odom) and _nonempty(data.odom_base):
        # Planar composition: yaw(map->base) = yaw(map->odom) + yaw(odom->base).
        # Exact for the 2-D transforms slam_toolbox and the EKF (two_d_mode) publish.
        tm, ym = _clean(data.map_odom["t"], data.map_odom["yaw"])
        tb, yb = _clean(data.odom_base["t"], data.odom_base["yaw"])
        a, _ = _interp(tm, np.unwrap(ym), grid, 1e9)   # map->odom is step-like by nature
        b, gb = _interp(tb, np.unwrap(yb), grid, p.max_gap_sec)
        # Before the first map->odom there is no map pose at all.
        tracks["slam"], gaps["slam"] = a + b, gb
        rates["slam"] = _moving_average(np.gradient(a + b, dt), n_avg)
        info["slam"]["present"] = True
    else:
        missing = [n for n, d in (("map->odom", data.map_odom), ("odom->base", data.odom_base))
                   if not _nonempty(d)]
        info["slam"]["reason"] = "no " + " / ".join(missing) + " in /tf"

    if _nonempty(data.laser_odom):
        from_yaw("laser_odom", data.laser_odom["t"], data.laser_odom["yaw"])
    else:
        info["laser_odom"]["reason"] = "no /laser/odom in bag"

    lidar = None
    if data.scans is not None and len(data.scans.get("t", [])):
        frame = data.scans.get("frame_id") or "lidar_link"
        try:
            r, tr = static_transform(data.static_tf, data.static_xyz, data.base_frame, frame)
            lidar = {"R": r, "t": tr, "from": f"/tf_static {data.base_frame} <- {frame}",
                     "xyz": [float(v) for v in tr],
                     "yaw_deg": math.degrees(math.atan2(r[1, 0], r[0, 0]))}
            info["scanmatch"]["present"] = True
        except KeyError as exc:
            info["scanmatch"]["reason"] = f"/scan present but lidar mount unknown: {exc}"
    else:
        info["scanmatch"]["reason"] = "no /scan in bag"

    return grid, tracks, rates, raw_rates, gaps, info, mount, raw_imu, lidar


# --------------------------------------------------------------------------- phases

def classify(grid: np.ndarray, data: BagData, p: Params):
    """Per-sample phase labels from wheel twist and command. -> (labels, steer grid)."""
    n = len(grid)
    labels = np.array(["unknown"] * n, dtype=object)
    steer = None
    if _nonempty(data.steer):
        t = np.asarray(data.steer["t"], dtype=float)
        ang = np.asarray(data.steer["angles"], dtype=float)
        order = np.argsort(t, kind="stable")
        steer = np.column_stack([np.interp(grid, t[order], ang[order, j], left=np.nan, right=np.nan)
                                 for j in range(ang.shape[1])])
    if not _nonempty(data.wheel_odom):
        return labels, steer
    w = data.wheel_odom
    m = max(1, int(round(p.class_smooth_sec * p.grid_hz)))
    t, vx, vy, wz = _clean(w["t"], w["vx"], w["vy"], w["wz"])
    vx, vy, wz = (np.interp(grid, t, c) for c in (vx, vy, wz))
    # Standstill is judged on the moving MAXIMUM, motion type on the moving
    # average. An average blurs the onset of motion into the standstill before
    # it, and the settle window sits exactly at that end of the standstill
    # (measured: 1 deg of rotation inside a "standstill" in the 2026-10-08 bag).
    # A maximum is also immune to the stale-read zeros in the wheel rate.
    still_v = _moving_max_abs(np.hypot(vx, vy), m) < p.v_still
    still_w = _moving_max_abs(wz, m) < math.radians(p.w_still_deg)
    vx, vy, wz = (_moving_average(c, m) for c in (vx, vy, wz))
    speed = np.hypot(vx, vy)

    cmd_active = np.zeros(n, dtype=bool)
    if _nonempty(data.cmd):
        tc, cvx, cvy, cwz = _clean(data.cmd["t"], data.cmd["vx"], data.cmd["vy"], data.cmd["wz"])
        idx = np.searchsorted(tc, grid, side="right") - 1
        held = idx >= 0
        idx = np.clip(idx, 0, len(tc) - 1)
        fresh = held & (grid - tc[idx] <= p.cmd_timeout_sec)
        mag = np.maximum(np.maximum(np.abs(cvx[idx]), np.abs(cvy[idx])), np.abs(cwz[idx]))
        cmd_active = fresh & (mag > p.cmd_eps)

    moving = speed >= p.v_still
    turning = np.abs(wz) >= math.radians(p.w_still_deg)
    kappa = np.abs(wz) / np.maximum(speed, 1e-9)
    # Not still, yet on average below both thresholds: onset or end of motion.
    labels[:] = "transition"
    labels[moving & (kappa >= p.kappa_straight)] = "curve"
    labels[moving & (kappa < p.kappa_straight)] = "straight"
    labels[~moving & turning] = "rotation"
    still = still_v & still_w
    labels[still & ~cmd_active] = "standstill"
    labels[still & cmd_active] = "transition"
    for t_split in p.split_at:
        t_abs = data.t0 + t_split
        labels[(grid >= t_abs - 0.5) & (grid <= t_abs + 0.5)] = "transition"
    return labels, steer


def label_runs(grid: np.ndarray, labels: np.ndarray, min_sec: float):
    """Run-length intervals; runs shorter than min_sec are absorbed by the one before.

    A standstill never absorbs anything: the settle window sits at its end, and a
    short motion run folded into it would put the onset of motion inside the
    window (the bias estimate then carries the first fraction of a turn).
    """
    runs = []
    i, n = 0, len(labels)
    while i < n:
        j = i
        while j + 1 < n and labels[j + 1] == labels[i]:
            j += 1
        runs.append([labels[i], i, j])
        i = j + 1
    merged = []
    for lab, a, b in runs:
        dur = grid[b] - grid[a]
        if merged and (merged[-1][0] == lab
                       or (dur < min_sec and merged[-1][0] != "standstill")):
            merged[-1][2] = b
        else:
            merged.append([lab, a, b])
    # A short FIRST motion run has no predecessor; give it to its successor.
    if (len(merged) > 1 and merged[1][0] != "standstill"
            and grid[merged[0][2]] - grid[merged[0][1]] < min_sec):
        merged[1][1] = merged[0][1]
        merged.pop(0)
    return [(lab, a, b) for lab, a, b in merged]


# --------------------------------------------------------------------------- analysis

def _nanmean(y):
    y = y[np.isfinite(y)]
    return float(np.mean(y)) if len(y) else None


def gyro_window(raw_imu, t_a: float, t_b: float, run=None) -> Optional[dict]:
    """Gyro statistics over [t_a, t_b] from the raw samples (base_footprint z).

    run: (t_start, t_end) of the standstill holding the window. An all-zero
    window inside a run that is alive and quantised is a bias of 0 at the
    resolution limit, not a dead sensor (module doc, GYRO AT STANDSTILL).
    """
    if raw_imu is None:
        return None
    m = (raw_imu["t"] >= t_a) & (raw_imu["t"] <= t_b)
    n = int(m.sum())
    out = {"n": n, "exact_zero_samples": int(raw_imu["zero"][m].sum()),
           "mean_z_deg_s": None, "std_z_deg_s": None, "flag": "",
           "quant_step_deg_s": None, "below_one_step": None}
    if n == 0:
        out["flag"] = "no gyro samples"
    elif out["exact_zero_samples"] == n:
        alive = None
        if run is not None:
            rm = (raw_imu["t"] >= run[0]) & (raw_imu["t"] <= run[1])
            if not raw_imu["zero"][rm].all():
                axis = raw_imu["w"][:, raw_imu["axis"]]
                q = quant_step(axis[rm])
                alive = (q, "run") if q is not None else None
                if alive is None and (q := quant_step(axis)) is not None:
                    alive = (q, "bag")
        if alive is None:
            out["flag"] = ZERO_FLAG   # dead or unread sensor - NOT a bias of 0
        else:
            out.update(mean_z_deg_s=0.0, std_z_deg_s=0.0, quant_step_deg_s=math.degrees(alive[0]),
                       below_one_step=True, zero_window_alive_from=alive[1])
    elif raw_imu["wz_base"] is not None:
        z = raw_imu["wz_base"][m]
        out["mean_z_deg_s"] = math.degrees(float(np.mean(z)))
        out["std_z_deg_s"] = math.degrees(float(np.std(z)))
        q = quant_step(raw_imu["w"][m, raw_imu["axis"]])
        if q is not None:
            out["quant_step_deg_s"] = math.degrees(q)
            out["below_one_step"] = out["std_z_deg_s"] < out["quant_step_deg_s"]
    return out


def find_settled(grid, runs, tracks, data: BagData, p: Params):
    """Every standstill run >= T_settle is a candidate; settled if slam scatter is low.

    Each candidate also gets the scans usable for scanmatch (scan_ok: at least
    scan.count scans stamped after the motion-end margin).
    """
    out = []
    t_scan = (np.asarray(data.scans["t"], dtype=float)
              if data.scans is not None and len(data.scans.get("t", [])) else None)
    slam = tracks.get("slam")
    changes = None
    if _nonempty(data.map_odom):
        t, yaw, x, y = _clean(data.map_odom["t"], data.map_odom["yaw"],
                              data.map_odom["x"], data.map_odom["y"])
        step = np.abs(np.diff(np.unwrap(yaw))) + np.hypot(np.diff(x), np.diff(y))
        changes = t[1:][step > 1e-5]
    n_win = max(2, int(round(p.t_settle * p.grid_hz)))
    for lab, a, b in runs:
        if lab != "standstill" or grid[b] - grid[a] < p.t_settle:
            continue
        w0 = max(a, b - n_win + 1)
        cand = {"t_start": float(grid[a]), "t_end": float(grid[b]),
                "win": (int(w0), int(b)), "settled": False, "reason": ""}
        cand["scan_idx"] = ([] if t_scan is None
                            else sm.pick_scans(t_scan, grid[a], grid[b], p.scan))
        cand["scan_n"] = len(cand["scan_idx"])
        cand["scan_ok"] = p.scan.count > 0 and cand["scan_n"] >= p.scan.count
        if slam is None:
            cand["reason"] = "no slam track"
            out.append(cand)
            continue
        ys = slam[w0:b + 1]
        ys = ys[np.isfinite(ys)]
        if len(ys) < 2:
            cand["reason"] = "slam undefined in window"
            out.append(cand)
            continue
        cand["slam_std_deg"] = float(np.degrees(np.std(ys)))
        cand["slam_p2p_deg"] = float(np.degrees(np.ptp(ys)))
        upd = None
        if changes is not None and len(changes):
            before = changes[changes <= grid[b]]
            if len(before):
                upd = float(grid[b] - before[-1])
        cand["map_odom_age_s"] = upd
        # Counted from motion end (= standstill start); negative = the last update
        # landed inside the standstill (late correction, see MAP->ODOM STAMPS).
        cand["map_odom_age_at_motion_end_s"] = (
            None if upd is None else float(grid[a] - (grid[b] - upd)))
        lo = tracks.get("laser_odom")
        lo = lo[w0:b + 1][np.isfinite(lo[w0:b + 1])] if lo is not None else []
        cand["laser_odom_std_deg"] = float(np.degrees(np.std(lo))) if len(lo) >= 2 else None
        updated_here = upd is not None and grid[b] - upd >= grid[a]
        cand["map_odom_updated_in_standstill"] = bool(updated_here)
        if cand["slam_std_deg"] >= p.sigma_settle_deg:
            cand["reason"] = f"slam scatter {cand['slam_std_deg']:.3f} deg >= sigma_settle"
        elif p.require_slam_update and not updated_here:
            cand["reason"] = "no map->odom update during the standstill"
        else:
            cand["settled"] = True
        out.append(cand)
    return out


def _segment_phase(labels, steer, a, b, p: Params):
    part = labels[a:b + 1]
    motion = [lab for lab in part if lab not in ("standstill", "transition", "unknown")]
    fractions = {}
    for lab in motion:
        fractions[lab] = fractions.get(lab, 0) + 1
    fractions = {k: v / len(motion) for k, v in fractions.items()} if motion else {}
    spread = None
    if steer is not None:
        mask = np.array([lab == "curve" for lab in part])
        if mask.any():
            sub = steer[a:b + 1][mask]
            spread = float(np.degrees(np.nanmax(np.nanpercentile(sub, 95, axis=0)
                                                - np.nanpercentile(sub, 5, axis=0))))
    if not motion:
        return "standstill", fractions, spread
    best = max(fractions, key=fractions.get)
    if fractions[best] < p.purity:
        return "mixed", fractions, spread
    if best == "curve" and spread is not None and spread > p.steer_const_tol_deg:
        return "mixed", fractions, spread
    return best, fractions, spread


def _parse_floor(floor_marks):
    out = []
    for item in floor_marks or []:
        if isinstance(item, dict):
            out.append((float(item["t"]), float(item["yaw_increment_deg"])))
        else:
            t, deg = str(item).split(":")
            out.append((float(t), float(deg)))
    return out


def analyse(data: BagData, p: Params = None, floor_marks=None) -> dict:
    """The whole decision chain, with no ROS in it. Returns a JSON-able dict."""
    p = p or Params()
    if p.reference not in REFERENCES:
        raise ValueError(f"reference must be one of {REFERENCES}, not {p.reference!r}")
    grid, tracks, rates, raw_rates, gaps, info, mount, raw_imu, lidar = build_tracks(data, p)
    labels, steer = classify(grid, data, p)
    runs = label_runs(grid, labels, p.min_phase_sec)
    cands = find_settled(grid, runs, tracks, data, p)
    # See SEGMENT BOUNDARIES in the module doc.
    boundary_rule = ("scans" if info["scanmatch"]["present"] and p.reference != "slam"
                     else "slam sigma_settle")
    for c in cands:
        c["boundary"] = c["scan_ok"] if boundary_rule == "scans" else c["settled"]
    settled = [c for c in cands if c["boundary"]]
    floor = _parse_floor(floor_marks)
    rel = lambda t: float(t - data.t0)  # noqa: E731

    totals = {}
    for lab, a, b in runs:
        totals[lab] = totals.get(lab, 0.0) + float(grid[b] - grid[a] + 1.0 / p.grid_hz)

    lag_ref = "gyro" if info["gyro"]["present"] else ("slam" if info["slam"]["present"] else None)
    max_lag = max(1, int(round(p.max_lag_sec * p.grid_hz)))
    present = [s for s in TRACKED if info[s]["present"]]
    points = {}

    def win_mean(name, win):
        return _nanmean(tracks[name][win[0]:win[1] + 1])

    def scan_pts(c):
        if c["t_start"] not in points:
            points[c["t_start"]] = sm.scan_set_points(data.scans, c["scan_idx"], lidar["R"],
                                                      lidar["t"], p.scan)
        return points[c["t_start"]]

    segments = []
    for k in range(len(settled) - 1):
        c0, c1 = settled[k], settled[k + 1]
        (a0, b0), (a1, b1) = c0["win"], c1["win"]
        tc0, tc1 = float(np.mean(grid[a0:b0 + 1])), float(np.mean(grid[a1:b1 + 1]))
        phase, fractions, spread = _segment_phase(labels, steer, b0, a1, p)
        seg = {"index": k, "t0": rel(grid[b0]), "t1": rel(grid[a1]),
               "duration_s": float(grid[a1] - grid[b0]), "phase": phase,
               "phase_fractions": fractions, "steer_spread_deg": spread,
               "lag_reference": lag_ref, "gyro_bias_deg_s": None,
               "gyro_residual_bias_deg_s": None, "gyro_flag": "",
               "gyro_bias_resolution_limited": None,
               "floor_deg": None, "scanmatch": None, "reference": None,
               "reference_note": "", "reference_increment_deg": None, "sources": {}}
        for t_mark, deg in floor:
            if c0["t_start"] - data.t0 <= t_mark <= c1["t_end"] - data.t0:
                seg["floor_deg"] = deg
        incs = {}
        g0 = None
        for s in present:
            m0, m1 = win_mean(s, c0["win"]), win_mean(s, c1["win"])
            if m0 is None or m1 is None:
                incs[s] = None
                continue
            inc = m1 - m0
            if s == "gyro":
                bias = _nanmean(raw_rates["gyro"][a0:b0 + 1])
                g0 = gyro_window(raw_imu, grid[a0], grid[b0],
                                 run=(c0["t_start"], c0["t_end"]))
                if g0 and g0["flag"]:
                    seg["gyro_flag"] = g0["flag"]
                    bias = None       # no bias from a silent sensor
                elif g0:
                    seg["gyro_bias_resolution_limited"] = bool(g0["below_one_step"])
                after = _nanmean(raw_rates["gyro"][a1:b1 + 1])
                if bias is not None:
                    inc -= bias * (tc1 - tc0)
                    seg["gyro_bias_deg_s"] = math.degrees(bias)
                    if after is not None:
                        seg["gyro_residual_bias_deg_s"] = math.degrees(after - bias)
            incs[s] = math.degrees(inc)
        rows = list(present)
        if info["scanmatch"]["present"]:
            rows.append("scanmatch")
            # Whole turns from the gyro, else the encoder (scan_match module doc).
            # Uncertainty of the guess, CHOSEN models (ScanMatchParams): gyro =
            # scale tolerance + bias uncertainty (half a quantisation step when
            # one is known) over the segment; encoder = its measured over-report.
            if incs.get("gyro") is not None and not seg["gyro_flag"]:
                guess, guess_from, second = incs["gyro"], "gyro", incs.get("encoder")
                step = (g0 or {}).get("quant_step_deg_s")
                bias_unc = 0.5 * step if step else p.scan.gyro_bias_unc_deg_s
                unc = p.scan.gyro_scale_tol * abs(guess) + bias_unc * (tc1 - tc0)
            else:
                guess, guess_from, second = incs.get("encoder"), "encoder", None
                unc = None if guess is None else p.scan.encoder_rel_err * abs(guess)
            if guess is None:
                scan = sm.match_sets([], [], 0.0, None, p.scan)
                scan["reason"] = "no gyro or encoder increment for the turn count"
            else:
                scan = sm.match_sets(scan_pts(c0), scan_pts(c1), guess,
                                     sm.guess_translation(data.wheel_odom, grid[b0], grid[b1]),
                                     p.scan, guess_unc_deg=unc, second_guess_deg=second)
            scan["guess_from"] = guess_from
            seg["scanmatch"] = scan
            incs["scanmatch"] = scan["increment_deg"]
        note = ""
        if p.reference == "floor":
            ref_name, ref = "floor", seg["floor_deg"]
            if ref is None:
                note = "no floor mark for this segment - no reference"
        elif p.reference == "scanmatch" and incs.get("scanmatch") is not None:
            ref_name, ref = "scanmatch", incs["scanmatch"]
        else:
            ref_name, ref = "slam", incs.get("slam")
            if p.reference == "scanmatch":
                why = seg["scanmatch"]["reason"] if seg["scanmatch"] else info["scanmatch"]["reason"]
                note = f"scanmatch unavailable ({why}) - fell back to slam"
        seg["reference"], seg["reference_note"] = ref_name, note
        seg["reference_increment_deg"] = ref
        for s in rows:
            inc = incs[s]
            row = {"increment_deg": inc, "error_deg": None, "scale": None,
                   "lag_s": None, "lag_corr": None, "lag_at_limit": None,
                   "gap": bool(gaps[s][a0:b1 + 1].any()) if s in gaps else False,
                   "error_vs_floor_deg": None, "scale_vs_floor": None}
            if inc is not None and ref is not None:
                row["error_deg"] = inc - ref
                if abs(ref) >= p.min_scale_increment_deg:
                    row["scale"] = inc / ref
            if inc is not None and seg["floor_deg"] is not None:
                row["error_vs_floor_deg"] = inc - seg["floor_deg"]
                if abs(seg["floor_deg"]) >= p.min_scale_increment_deg:
                    row["scale_vs_floor"] = inc / seg["floor_deg"]
            if lag_ref is not None and s != lag_ref and s in rates:
                r = rates[lag_ref][a0:b1 + 1]
                if np.nanstd(r) >= math.radians(p.min_lag_rate_deg):
                    res = cross_correlation_lag(r, rates[s][a0:b1 + 1], max_lag)
                    if res is not None:
                        row["lag_s"] = res[0] / p.grid_hz
                        row["lag_corr"] = res[1]
                        row["lag_at_limit"] = res[2]
            seg["sources"][s] = row
        segments.append(seg)

    return {
        "bag": data.label,
        "t0_epoch": data.t0,
        "params": asdict(p),
        "reference": p.reference,
        "boundary_rule": boundary_rule,
        "sources": info,
        "imu_mount": mount,
        "lidar_mount": ({k: lidar[k] for k in ("from", "xyz", "yaw_deg")} if lidar else None),
        "clock": {role: dict(c, stamp_origin=STAMP_NOTES.get(role, ""))
                  for role, c in data.clock.items()},
        "grid": {"t0": rel(grid[0]), "t1": rel(grid[-1]), "hz": p.grid_hz},
        "phase_totals_s": totals,
        "phase_intervals": [{"phase": lab, "t0": rel(grid[a]), "t1": rel(grid[b])}
                            for lab, a, b in runs],
        "standstill_gyro": [
            dict(gyro_window(raw_imu, grid[a], grid[b]), t0=rel(grid[a]), t1=rel(grid[b]))
            for lab, a, b in runs if lab == "standstill" and raw_imu is not None],
        "settle_candidates": [
            {k: (rel(v) if k in ("t_start", "t_end") else v)
             for k, v in c.items() if k not in ("win", "scan_idx")} for c in cands],
        "segments": segments,
    }


# --------------------------------------------------------------------------- output

def _f(v, fmt="{:+.2f}"):
    return "-" if v is None else fmt.format(v)


def format_table(res: dict) -> str:
    lines = [f"yaw_compare  {res['bag']}",
             f"grid {res['grid']['t0']:.1f}..{res['grid']['t1']:.1f} s after bag start "
             f"at {res['grid']['hz']:.0f} Hz", "", "SOURCES"]
    for s, i in res["sources"].items():
        state = "present" if i["present"] else "MISSING - " + i["reason"]
        if s == "encoder_twist" and i["present"]:
            state += " (APPROXIMATE - `encoder` is authoritative)"
        lines.append(f"  {s:14s} {state}")
    if res["imu_mount"]:
        m = res["imu_mount"]
        lines.append(f"  imu mount from {m['from']}: base z = "
                     + " ".join(f"{v:+.3f}" for v in m["z_row"]) + " . (x y z)_imu")
    if res["lidar_mount"]:
        m = res["lidar_mount"]
        lines.append(f"  lidar mount from {m['from']}: x {m['xyz'][0]:+.4f} y {m['xyz'][1]:+.4f} m, "
                     f"yaw {m['yaw_deg']:+.2f} deg")
    lines += ["", "STAMP CLOCKS (receive - header stamp)"]
    for role, c in res["clock"].items():
        med = c.get("recv_minus_stamp_median_s")
        p95 = c.get("recv_minus_stamp_p95_abs_s")
        lines.append(f"  {role:11s} {c.get('topic', ''):28s} n={c.get('n', 0):6d} "
                     f"median {_f(med, '{:+.3f}')} s  p95|.| {_f(p95, '{:.3f}')} s  "
                     f"- {c['stamp_origin']}")
    lines += ["", "PHASE TOTALS (s): " + ", ".join(
        f"{k} {v:.1f}" for k, v in sorted(res["phase_totals_s"].items()))]
    if res["standstill_gyro"]:
        lines += ["", "GYRO AT STANDSTILL (base_footprint z, every standstill run)"]
        for g in res["standstill_gyro"]:
            lines.append(
                f"  {g['t0']:7.1f}-{g['t1']:7.1f} s  n={g['n']:5d}  "
                f"mean {_f(g['mean_z_deg_s'], '{:+.4f}')} deg/s  "
                f"std {_f(g['std_z_deg_s'], '{:.4f}')} deg/s  "
                f"step {_f(g.get('quant_step_deg_s'), '{:.4f}')} deg/s"
                + ("  (std < 1 step: bias resolution-limited)" if g.get("below_one_step") else "")
                + (f"  ** {g['flag'].upper()} **" if g["flag"] else ""))
    cands = res["settle_candidates"]
    n_ok = sum(c["settled"] for c in cands)
    n_b = sum(c["boundary"] for c in cands)
    lines += ["", f"SETTLE CANDIDATES: {len(cands)} standstills >= T_settle, {n_ok} slam-settled, "
              f"{n_b} used as segment boundaries (rule: {res['boundary_rule']})"]
    for c in cands:
        lines.append(
            f"  {c['t_start']:7.1f}-{c['t_end']:7.1f} s  "
            f"{'BOUNDARY' if c['boundary'] else 'unused  '}  "
            f"slam {'settled ' if c['settled'] else 'rejected'}  scans {c['scan_n']:2d}  "
            f"std {_f(c.get('slam_std_deg'), '{:.3f}')} deg  "
            f"map->odom age {_f(c.get('map_odom_age_s'), '{:.1f}')} s "
            f"(at motion end {_f(c.get('map_odom_age_at_motion_end_s'), '{:+.1f}')} s)  "
            f"laser_odom std {_f(c.get('laser_odom_std_deg'), '{:.3f}')} deg"
            + (f"  ({c['reason']})" if c["reason"] else ""))
    lag_ref = res["segments"][0]["lag_reference"] if res["segments"] else "-"
    lines += ["", "SEGMENTS between consecutive boundary standstills "
              f"(reference {res['reference']}, per segment below; lag vs {lag_ref},"
              " + = source later)",
              "  NOTE: slam lags include the map->odom future-dating (0.4/0.5 s measured, "
              "not corrected); a late slam correction can fall into the next segment.",
              f"  {'seg':>3s} {'t0':>7s} {'t1':>7s} {'phase':10s} {'source':14s} "
              f"{'d_yaw':>9s} {'err':>8s} {'scale':>7s} {'lag_s':>7s} {'corr':>5s} "
              f"{'floor_err':>9s}  notes"]
    for seg in res["segments"]:
        first = True
        for s, r in seg["sources"].items():
            notes = []
            if r["gap"]:
                notes.append("gap")
            if r["lag_at_limit"]:
                notes.append("lag at search limit")
            if s == seg["reference"]:
                notes.append("REFERENCE")
            head = (f"  {seg['index']:3d} {seg['t0']:7.1f} {seg['t1']:7.1f} {seg['phase']:10s}"
                    if first else " " * 31)
            lines.append(
                f"{head} {s:14s} {_f(r['increment_deg'], '{:+9.2f}')} "
                f"{_f(r['error_deg'], '{:+8.2f}')} {_f(r['scale'], '{:7.4f}')} "
                f"{_f(r['lag_s'], '{:+7.3f}')} {_f(r['lag_corr'], '{:5.2f}')} "
                f"{_f(r['error_vs_floor_deg'], '{:+9.2f}')}  {' '.join(notes)}")
            first = False
        pad = " " * 31
        if seg["reference"] == "floor" and seg["reference_increment_deg"] is not None:
            lines.append(pad + f" reference floor mark {seg['reference_increment_deg']:+.2f} deg")
        if seg["reference_note"]:
            lines.append(pad + f" ** {seg['reference_note']} **")
        sc = seg["scanmatch"]
        if sc is not None:
            guess = (f"guess {sc['guess_from']} {sc['guess_deg']:+.1f} "
                     f"+- {_f(sc.get('guess_unc_deg'), '{:.1f}')} deg"
                     + ("" if sc.get("second_guess_deg") is None else
                        f", encoder {sc['second_guess_deg']:+.1f} deg "
                        f"(gyro-encoder {sc['guess_disagreement_deg']:+.1f} deg)"))
            if sc["failed"]:
                lines.append(pad + f" scanmatch FAILED: {sc['reason']}")
                if sc.get("turns_ambiguous"):
                    lines.append(pad + " ** TURN COUNT AMBIGUOUS ** " + guess)
            else:
                lines.append(
                    pad + f" scanmatch {sc['increment_deg']:+.2f} deg = {sc['turns']:+d} turns "
                    f"({guess}) {sc['frac_deg']:+.2f}; scatter {sc['scatter_deg']:.3f} deg over "
                    f"{sc['n_ok']}/{sc['n_pairs']} pairs; t ({sc['tx']:+.3f}, {sc['ty']:+.3f}) m; "
                    f"inliers {100 * sc['inlier_frac']:.0f} %, rms {1000 * sc['rms']:.1f} mm, "
                    f"lever {sc['rot_lever']:.2f} m, distinct {_f(sc['distinctness'], '{:.1f}')}")
        if seg["gyro_flag"]:
            lines.append(" " * 31 + f" ** {seg['gyro_flag'].upper()} in the bias window - no bias **")
        if seg["gyro_bias_deg_s"] is not None:
            lines.append(" " * 31 + f" gyro bias {seg['gyro_bias_deg_s']:+.4f} deg/s"
                         + (" (resolution-limited)" if seg["gyro_bias_resolution_limited"] else "")
                         + ", residual at next standstill "
                         f"{_f(seg['gyro_residual_bias_deg_s'], '{:+.4f}')} deg/s")
    if not res["segments"]:
        lines.append("  (none - fewer than two settled poses)")
    return "\n".join(lines)


CSV_FIELDS = ["segment", "t0", "t1", "duration_s", "phase", "source", "increment_deg",
              "reference_increment_deg", "error_deg", "scale", "lag_s", "lag_corr",
              "lag_at_limit", "lag_reference", "gap", "floor_deg", "error_vs_floor_deg",
              "scale_vs_floor", "gyro_bias_deg_s", "gyro_residual_bias_deg_s",
              "steer_spread_deg", "gyro_flag", "gyro_bias_resolution_limited", "reference", "reference_note",
              "scan_scatter_deg", "scan_ok_pairs", "scan_pairs", "scan_tx_m", "scan_ty_m",
              "scan_inlier_frac", "scan_rms_m", "scan_failed_reason"]


def write_csv(res: dict, path: str):
    with open(path, "w", newline="") as handle:
        w = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        w.writeheader()
        for seg in res["segments"]:
            ref = seg["reference_increment_deg"]
            sc = seg["scanmatch"] or {}
            scan_cols = {
                "scan_scatter_deg": sc.get("scatter_deg"), "scan_ok_pairs": sc.get("n_ok"),
                "scan_pairs": sc.get("n_pairs"), "scan_tx_m": sc.get("tx"),
                "scan_ty_m": sc.get("ty"), "scan_inlier_frac": sc.get("inlier_frac"),
                "scan_rms_m": sc.get("rms"),
                "scan_failed_reason": sc.get("reason") if sc.get("failed") else ""}
            for s, r in seg["sources"].items():
                w.writerow({
                    "segment": seg["index"], "t0": seg["t0"], "t1": seg["t1"],
                    "duration_s": seg["duration_s"], "phase": seg["phase"], "source": s,
                    "reference_increment_deg": ref, "lag_reference": seg["lag_reference"],
                    "floor_deg": seg["floor_deg"], "gyro_bias_deg_s": seg["gyro_bias_deg_s"],
                    "gyro_residual_bias_deg_s": seg["gyro_residual_bias_deg_s"],
                    "steer_spread_deg": seg["steer_spread_deg"], "gyro_flag": seg["gyro_flag"],
                    "gyro_bias_resolution_limited": seg["gyro_bias_resolution_limited"],
                    "reference": seg["reference"], "reference_note": seg["reference_note"],
                    **scan_cols,
                    **{k: r[k] for k in ("increment_deg", "error_deg", "scale", "lag_s",
                                         "lag_corr", "lag_at_limit", "gap",
                                         "error_vs_floor_deg", "scale_vs_floor")},
                })


def _json_safe(o):
    if isinstance(o, dict):
        return {str(k): _json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_safe(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not math.isfinite(float(o)) else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def write_json(res: dict, path: str):
    with open(path, "w") as handle:
        json.dump(_json_safe(res), handle, indent=1)


# --------------------------------------------------------------------------- bag reading

@dataclass
class Topics:
    wheel_odom: str = "/wheel/odom"
    joint_states: str = "/joint_states"
    imu: str = "/imu/bno085/data"
    laser_odom: str = "/laser/odom"
    cmd: str = "/cmd_vel"
    scan: str = "/scan"
    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_footprint"
    steer_joints: Tuple[str, ...] = ("f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer")


def resolve_bag(path: str) -> str:
    """Accept the bag directory itself or a folder holding exactly one bag."""
    path = os.path.expanduser(path)
    if os.path.isfile(os.path.join(path, "metadata.yaml")):
        return path
    subs = sorted(d for d in os.listdir(path)
                  if os.path.isfile(os.path.join(path, d, "metadata.yaml")))
    if len(subs) == 1:
        return os.path.join(path, subs[0])
    raise SystemExit(f"{path}: expected one bag, found {subs or 'none'}")


def read_bag(uri: str, topics: Topics = None, start: float = None, duration: float = None) -> BagData:
    """Read only the topics the analysis needs. ROS imports stay in here."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topics = topics or Topics()

    def open_reader(names):
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
               rosbag2_py.ConverterOptions("", ""))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        r.set_filter(rosbag2_py.StorageFilter(topics=[n for n in names if n in types]))
        return r, types

    # /tf_static is latched at the start; read it in its own pass so a time
    # window cannot cut it off. Several latched copies (a restart inside the bag)
    # are fine as long as they agree.
    data = BagData(base_frame=topics.base_frame, label=uri)
    r, types = open_reader(["/tf_static"])
    data.t0 = r.get_metadata().starting_time.nanoseconds * 1e-9
    conflicts = set()
    while r.has_next():
        _, raw, _ = r.read_next()
        for tr in deserialize_message(raw, get_message(types["/tf_static"])).transforms:
            q = tr.transform.rotation
            q = (q.x, q.y, q.z, q.w)
            old = data.static_tf.get(tr.child_frame_id)
            if old and max(abs(a - b) for a, b in zip(old[1], q)) > 1e-6:
                conflicts.add(tr.child_frame_id)
            data.static_tf[tr.child_frame_id] = (tr.header.frame_id, q)
            tt = tr.transform.translation
            data.static_xyz[tr.child_frame_id] = (tt.x, tt.y, tt.z)
    if conflicts:
        print(f"WARNING: /tf_static changes inside the bag for {sorted(conflicts)}; "
              "the last copy is used", file=sys.stderr)

    want = {topics.wheel_odom: "wheel_odom", topics.joint_states: "joint_states",
            topics.imu: "imu", topics.laser_odom: "laser_odom", topics.cmd: "cmd", "/tf": "tf",
            topics.scan: "scan"}
    r, types = open_reader(list(want))
    t_start = data.t0 + (start or 0.0)
    t_end = t_start + duration if duration else float("inf")
    if start:
        r.seek(int(t_start * 1e9))
    cls = {n: get_message(types[n]) for n in want if n in types}
    buf = {k: [] for k in ("wheel_odom", "joint_states", "imu", "laser_odom", "cmd",
                           "map_odom", "odom_base")}
    lag = {k: [] for k in buf}
    lag["scan"] = []
    scans = {k: [] for k in ("t", "ranges", "angle_min", "angle_inc", "range_min", "range_max")}
    scan_frame = None
    imu_frame = None
    steer_idx, last_names = None, None

    def st(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    while r.has_next():
        topic, raw, t_ns = r.read_next()
        recv = t_ns * 1e-9
        if recv > t_end:
            break
        msg = deserialize_message(raw, cls[topic])
        role = want[topic]
        if role == "tf":
            for tr in msg.transforms:
                pair = (tr.header.frame_id, tr.child_frame_id)
                if pair == (topics.map_frame, topics.odom_frame):
                    key = "map_odom"
                elif pair == (topics.odom_frame, topics.base_frame):
                    key = "odom_base"
                else:
                    continue
                q, tt = tr.transform.rotation, tr.transform.translation
                s = st(tr.header)
                buf[key].append((s, float(yaw_from_quat(q.x, q.y, q.z, q.w)), tt.x, tt.y))
                lag[key].append(recv - s)
        elif role in ("wheel_odom", "laser_odom"):
            q = msg.pose.pose.orientation
            tw = msg.twist.twist
            s = st(msg.header)
            buf[role].append((s, float(yaw_from_quat(q.x, q.y, q.z, q.w)),
                              tw.linear.x, tw.linear.y, tw.angular.z,
                              msg.pose.pose.position.x, msg.pose.pose.position.y))
            lag[role].append(recv - s)
        elif role == "joint_states":
            if tuple(msg.name) != last_names:
                last_names = tuple(msg.name)
                steer_idx = ([last_names.index(n) for n in topics.steer_joints]
                             if all(n in last_names for n in topics.steer_joints) else None)
            if steer_idx is None:
                continue               # e.g. the arm-only relay publishing on /joint_states
            s = st(msg.header)
            buf[role].append((s, *[msg.position[i] for i in steer_idx]))
            lag[role].append(recv - s)
        elif role == "imu":
            imu_frame = msg.header.frame_id or imu_frame
            a = msg.angular_velocity
            s = st(msg.header)
            buf[role].append((s, a.x, a.y, a.z))
            lag[role].append(recv - s)
        elif role == "cmd":
            buf[role].append((recv, msg.linear.x, msg.linear.y, msg.angular.z))
        elif role == "scan":
            scan_frame = msg.header.frame_id or scan_frame
            s = st(msg.header)
            scans["t"].append(s)
            scans["ranges"].append(np.asarray(msg.ranges, dtype=np.float32))
            scans["angle_min"].append(msg.angle_min)
            scans["angle_inc"].append(msg.angle_increment)
            scans["range_min"].append(msg.range_min)
            scans["range_max"].append(msg.range_max)
            lag["scan"].append(recv - s)

    def arr(k):
        return np.array(buf[k], dtype=float) if buf[k] else None

    if (a := arr("wheel_odom")) is not None:
        data.wheel_odom = {"t": a[:, 0], "yaw": a[:, 1], "vx": a[:, 2], "vy": a[:, 3], "wz": a[:, 4],
                           "x": a[:, 5], "y": a[:, 6]}
    if (a := arr("laser_odom")) is not None:
        data.laser_odom = {"t": a[:, 0], "yaw": a[:, 1]}
    if (a := arr("joint_states")) is not None:
        data.steer = {"t": a[:, 0], "angles": a[:, 1:], "names": list(topics.steer_joints)}
    if (a := arr("imu")) is not None:
        data.imu = {"t": a[:, 0], "w": a[:, 1:4], "frame_id": imu_frame or "imu_link"}
    if (a := arr("cmd")) is not None:
        data.cmd = {"t": a[:, 0], "vx": a[:, 1], "vy": a[:, 2], "wz": a[:, 3]}
    if (a := arr("map_odom")) is not None:
        data.map_odom = {"t": a[:, 0], "yaw": a[:, 1], "x": a[:, 2], "y": a[:, 3]}
    if (a := arr("odom_base")) is not None:
        data.odom_base = {"t": a[:, 0], "yaw": a[:, 1]}
    if scans["t"]:
        data.scans = {k: (v if k == "ranges" else np.array(v, dtype=float)) for k, v in scans.items()}
        data.scans["frame_id"] = scan_frame or "lidar_link"

    names = {"wheel_odom": topics.wheel_odom, "joint_states": topics.joint_states,
             "imu": topics.imu, "laser_odom": topics.laser_odom, "cmd": topics.cmd,
             "map_odom": f"/tf {topics.map_frame}->{topics.odom_frame}",
             "odom_base": f"/tf {topics.odom_frame}->{topics.base_frame}",
             "scan": topics.scan}
    for k, topic in names.items():
        lg = np.array(lag[k]) if lag[k] else None
        data.clock[k] = {
            "topic": topic, "n": len(lag[k]) if k == "scan" else len(buf[k]),
            "recv_minus_stamp_median_s": float(np.median(lg)) if lg is not None else None,
            "recv_minus_stamp_p95_abs_s": float(np.percentile(np.abs(lg), 95)) if lg is not None else None,
        }
    return data


# --------------------------------------------------------------------------- CLI

def build_parser() -> argparse.ArgumentParser:
    d = Params()
    ap = argparse.ArgumentParser(
        description="Per-phase yaw agreement of encoders, gyro and LiDAR from one bag. "
                    "Offline; reads the bag only. All thresholds are CHOSEN defaults, TO-VERIFY.")
    ap.add_argument("bag", help="bag directory, or a folder holding exactly one bag")
    ap.add_argument("--start", type=float, default=None, help="s after bag start")
    ap.add_argument("--duration", type=float, default=None, help="s to read from --start")
    ap.add_argument("--csv", help="write the per-segment table as CSV")
    ap.add_argument("--json", help="write the full result as JSON")
    ap.add_argument("--floor-mark", action="append", default=[], metavar="T:DEG",
                    help="operator floor-mark yaw increment DEG for the segment containing "
                         "T (s after bag start); repeatable")
    ap.add_argument("--floor-marks", metavar="YAML",
                    help="YAML: {floor_marks: [{t: <s>, yaw_increment_deg: <deg>}, ...]}")
    ap.add_argument("--imu-rpy", type=float, nargs=3, metavar=("R", "P", "Y"),
                    help="IMU mounting in base_footprint, radians - overrides /tf_static "
                         "(use when the bag's URDF predates the measured imu_joint)")
    for name, help_text in (
            ("grid_hz", "common time grid, Hz"),
            ("v_still", "m/s below which the base does not translate"),
            ("w_still_deg", "deg/s below which the base does not turn"),
            ("cmd_eps", "command magnitude counted as active"),
            ("cmd_timeout_sec", "held command older than this counts as zero"),
            ("class_smooth_sec", "moving average on the classification twist, s"),
            ("rate_smooth_sec", "moving average on yaw rates before correlation, s"),
            ("kappa_straight", "rad/m curvature below which motion is straight"),
            ("steer_const_tol_deg", "p5-p95 steer range of a constant-steer curve"),
            ("purity", "share of one phase needed to name a segment"),
            ("min_phase_sec", "label runs shorter than this are absorbed"),
            ("t_settle", "T_settle: standstill s before a slam pose counts"),
            ("sigma_settle_deg", "sigma_settle: slam yaw std limit over T_settle"),
            ("max_lag_sec", "cross-correlation search range, s"),
            ("min_lag_rate_deg", "reference-rate std (deg/s) needed for a lag"),
            ("min_scale_increment_deg", "|reference increment| needed for a scale"),
            ("max_gap_sec", "silence that marks a gap, s")):
        ap.add_argument("--" + name.replace("_", "-"), type=float, default=getattr(d, name),
                        help=f"{help_text} (default {getattr(d, name)}, CHOSEN, TO-VERIFY)")
    ap.add_argument("--require-slam-update", action="store_true",
                    help="settled only if map->odom changed during the standstill")
    ap.add_argument("--reference", choices=REFERENCES, default=d.reference,
                    help="segment reference; scanmatch falls back to slam per segment where it "
                         "fails (default %(default)s)")
    ap.add_argument("--split-at", type=float, action="append", default=[], metavar="T",
                    help="mark [T-0.5, T+0.5] s after bag start as motion, cutting a standstill "
                         "in two (noise-floor checks only); repeatable")
    sd = sm.ScanMatchParams()
    for name, value in asdict(sd).items():
        ap.add_argument("--scan-" + name.replace("_", "-"), dest="scan_" + name,
                        type=type(value), default=value,
                        help=f"scan_match.ScanMatchParams.{name} (default {value}, CHOSEN, TO-VERIFY)")
    t = Topics()
    for name in ("wheel_odom", "joint_states", "imu", "laser_odom", "cmd", "scan",
                 "map_frame", "odom_frame", "base_frame"):
        ap.add_argument("--" + name.replace("_", "-"), default=getattr(t, name))
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    special = ("imu_rpy", "require_slam_update", "reference", "split_at", "scan")
    p = Params(**{k: getattr(args, k) for k in Params.__dataclass_fields__ if k not in special})
    p.require_slam_update = args.require_slam_update
    p.imu_rpy = tuple(args.imu_rpy) if args.imu_rpy else None
    p.reference = args.reference
    p.split_at = tuple(args.split_at)
    p.scan = sm.ScanMatchParams(**{k: getattr(args, "scan_" + k)
                                   for k in sm.ScanMatchParams.__dataclass_fields__})
    topics = Topics(**{k: getattr(args, k) for k in (
        "wheel_odom", "joint_states", "imu", "laser_odom", "cmd", "scan",
        "map_frame", "odom_frame", "base_frame")})
    floor = list(args.floor_mark)
    if args.floor_marks:
        import yaml
        with open(args.floor_marks) as handle:
            floor += yaml.safe_load(handle).get("floor_marks", [])
    uri = resolve_bag(args.bag)
    data = read_bag(uri, topics, args.start, args.duration)
    res = analyse(data, p, floor)
    print(format_table(res))
    if args.csv:
        write_csv(res, args.csv)
    if args.json:
        write_json(res, args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
