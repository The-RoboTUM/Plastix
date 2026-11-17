"""yaw_compare must RECOVER known errors from synthetic tracks, not just run.

Every case plants a fault with a known size - encoder scale +9 %, encoder lag
0.10 s, laser-odometry lag 0.20 s and scale 0.98, gyro bias 0.5 deg/s, IMU mounted
rolled 180 deg - and asserts the tool reports that size within a stated tolerance.
What would make each test fail is stated beside it. No ROS, no bag: the core takes
plain arrays (BagData), which is the point of keeping read_bag() separate.
"""
import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripperx_localization import yaw_compare as yc  # noqa: E402
import test_scan_match as tsm  # noqa: E402  the independent ray-caster

ENC_SCALE = 1.09
ENC_LAG = 0.10
LO_SCALE = 0.98
LO_LAG = 0.20
BIAS_DEG = 0.5
ROLL_PI = (1.0, 0.0, 0.0, 0.0)       # quaternion xyzw of RPY (180 deg, 0, 0)
IDENTITY = (0.0, 0.0, 0.0, 1.0)
TILT = (math.radians(170.0), math.radians(5.0))   # roll, pitch: R != R^T, R @ R != I


def mount_matrix(roll, pitch):
    """R_base_imu = Ry(pitch) @ Rx(roll), written out by hand.

    Deliberately NOT yc.quat_to_matrix: if generator and analyser shared that
    helper, a transposed matrix would cancel out of every test.
    """
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def mount_quat(roll, pitch):
    """Quaternion (xyzw) of Ry(pitch) Rx(roll), by hand: q_y * q_x."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (cp * sr, sp * cr, -sp * sr, cp * cr)


T0 = 1.79e9                          # epoch-like, so float precision is exercised


def _bump(t, a, b):
    """Raised cosine on [a, b], integral (b - a) / 2."""
    x = np.clip((t - a) / (b - a), 0.0, 1.0)
    return np.where((t > a) & (t < b), 0.5 * (1 - np.cos(2 * math.pi * x)), 0.0)


def truth(t):
    """-> v (m/s), wz (rad/s), steer (rad) of the synthetic drive programme.

    0-6 still | 6-16 one in-place turn | 16-22 still | 22-28 straight |
    28-34 still | 34-44 curve, kappa 1 rad/m, constant steer | 44-50 still |
    50-60 half curve, half straight (mixed) | 60-66 still
    """
    wz_rot = (2 * math.pi / 5.0) * _bump(t, 6, 16)          # 360 deg
    v = 0.2 * (_bump(t, 22, 28) + _bump(t, 34, 44) + _bump(t, 50, 60))
    kappa = np.where(((t >= 34) & (t <= 44)) | ((t >= 50) & (t < 55)), 1.0, 0.0)
    wz = wz_rot + v * kappa
    steer = np.where(kappa > 0, 0.3, 0.0)
    steer = np.where((t >= 6) & (t <= 16), 0.54, steer)
    return v, wz, steer


FINE = np.arange(-1.0, 67.0, 0.001)
_V, _WZ, _ST = truth(FINE)
_YAW = np.concatenate([[0.0], np.cumsum(0.5 * (_WZ[1:] + _WZ[:-1]) * 0.001)])
_X = np.concatenate([[0.0], np.cumsum(_V[1:] * np.cos(_YAW[1:]) * 0.001)])
_Y = np.concatenate([[0.0], np.cumsum(_V[1:] * np.sin(_YAW[1:]) * 0.001)])
# The scan world: the irregular room of test_scan_match, enlarged so the whole
# drive programme (~2.5 m of travel) stays inside; start pose inside it.
WORLD = tsm._walls([(1.8 * x, 1.8 * y) for x, y in tsm.ROOM], tsm.BOX)
START = (0.5, 0.0, math.radians(10.0))


def at(t, arr):
    return np.interp(t, FINE, arr)


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def true_pose(t):
    """World pose (x, y, yaw) of base_footprint at t, START applied."""
    x, y, yaw = at(t, _X), at(t, _Y), at(t, _YAW)
    c, s = math.cos(START[2]), math.sin(START[2])
    return START[0] + c * x - s * y, START[1] + s * x + c * y, START[2] + yaw


def make_data(imu_mount=None, true_mount=(math.pi, 0.0), bias_fn=None, zero_imu=False,
              slam_noise=None, drop_wheel=None,
              with_imu=True, with_slam=True, with_laser=True,
              with_scans=False, blank_scans=None, imu_noise=0.002, imu_quant=None):
    rng = np.random.default_rng(1)
    d = yc.BagData(t0=T0, label="synthetic")
    tw = np.arange(0.0, 66.0, 1 / 30.0)
    if drop_wheel:
        tw = tw[(tw < drop_wheel[0]) | (tw > drop_wheel[1])]
    lagged = tw - ENC_LAG
    d.wheel_odom = {"t": T0 + tw, "yaw": wrap(ENC_SCALE * at(lagged, _YAW)),
                    "vx": at(lagged, _V), "vy": np.zeros_like(tw),
                    "wz": ENC_SCALE * at(lagged, _WZ),
                    "x": at(lagged, _X), "y": at(lagged, _Y)}
    st = at(tw, _ST)
    d.steer = {"t": T0 + tw, "angles": np.column_stack([st, st, -st, -st]),
               "names": ["f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"]}
    tc = np.arange(0.0, 66.0, 0.05)
    moving = (at(tc, _V) > 1e-4) | (np.abs(at(tc, _WZ)) > 1e-4)
    d.cmd = {"t": T0 + tc, "vx": np.where(moving, 0.2, 0.0), "vy": np.zeros_like(tc),
             "wz": np.zeros_like(tc)}
    d.static_tf = {"base_link": ("base_footprint", IDENTITY),
                   "chassis_link": ("base_link", IDENTITY),
                   "imu_link": ("chassis_link",
                                imu_mount if imu_mount is not None else mount_quat(*true_mount))}
    if with_imu:
        ti = np.arange(0.0, 66.0, 0.02)
        bias = bias_fn(ti) if bias_fn else np.full_like(ti, math.radians(BIAS_DEG))
        wz_base = at(ti, _WZ) + bias + rng.normal(0, imu_noise, len(ti))
        # Measured in imu_link: a 180 deg roll mirrors y and z. The tool must
        # undo this from the TF, so the RAW z here has the opposite sign.
        R = mount_matrix(*true_mount)
        w_base = np.column_stack([np.zeros_like(ti), np.zeros_like(ti), wz_base])
        w_imu = w_base @ R                                                # rows: R^T w
        if imu_quant:
            w_imu = np.round(w_imu / imu_quant) * imu_quant
        if zero_imu:
            w_imu = np.zeros_like(w_imu)
        d.imu = {"t": T0 + ti, "w": w_imu, "frame_id": "imu_link"}
    if with_slam:
        tm = np.arange(0.0, 66.0, 0.1)
        d.map_odom = {"t": T0 + tm, "yaw": np.full_like(tm, 0.3),
                      "x": np.zeros_like(tm), "y": np.zeros_like(tm)}
        tb = np.arange(0.0, 66.0, 1 / 30.0)
        yb = at(tb, _YAW) - 0.3
        if slam_noise:
            a, b, sd = slam_noise
            m = (tb >= a) & (tb <= b)
            yb[m] += rng.normal(0, math.radians(sd), m.sum())
        d.odom_base = {"t": T0 + tb, "yaw": wrap(yb)}       # wrapped: unwrap is tested too
    if with_laser:
        tl = np.arange(0.0, 66.0, 0.1)
        d.laser_odom = {"t": T0 + tl, "yaw": wrap(LO_SCALE * at(tl - LO_LAG, _YAW))}
    if with_scans:
        # Scans exist only where the analysis can use them (standstills), to keep
        # the test fast; the analyser must still pick them by stamp and margin.
        ts = np.array([t for t in np.arange(0.0, 66.0, 0.1)
                       if any(a <= t <= b for a, b in STILL)])
        srng = np.random.default_rng(7)
        ranges = [tsm.raycast(true_pose(t), WORLD, srng).astype(np.float32) for t in ts]
        if blank_scans:
            for i, t in enumerate(ts):
                if blank_scans[0] <= t <= blank_scans[1]:
                    ranges[i][:] = np.inf
        d.scans = {"t": T0 + ts, "ranges": ranges, "angle_min": np.zeros_like(ts),
                   "angle_inc": np.full_like(ts, tsm.INC), "range_min": np.full_like(ts, 0.1),
                   "range_max": np.full_like(ts, 25.0), "frame_id": "lidar_link"}
        d.static_tf["lidar_link"] = ("chassis_link", IDENTITY)
        d.static_xyz.update({"base_link": (0.0, 0.0, 0.1566), "chassis_link": (0.0, 0.0, 0.0),
                             "lidar_link": (tsm.LIDAR_X, 0.0, 0.1034)})
    return d


STILL = [(0.0, 6.0), (16.0, 22.0), (28.0, 34.0), (44.0, 50.0), (60.0, 66.0)]


@pytest.fixture(scope="module")
def result():
    return yc.analyse(make_data(), yc.Params())


def seg(res, i):
    return res["segments"][i]


# ---------------------------------------------------------------- segmentation

def test_segments_and_phases(result):
    # Fails if a standstill is missed, two are merged, or a phase rule is wrong.
    assert [s["phase"] for s in result["segments"]] == ["rotation", "straight", "curve", "mixed"]
    assert sum(c["settled"] for c in result["settle_candidates"]) == 5


def test_phase_intervals_follow_profile(result):
    seq = [iv["phase"] for iv in result["phase_intervals"] if iv["phase"] != "transition"]
    collapsed = [p for i, p in enumerate(seq) if i == 0 or p != seq[i - 1]]
    assert collapsed == ["standstill", "rotation", "standstill", "straight", "standstill",
                         "curve", "standstill", "curve", "straight", "standstill"]


def test_constant_steer_curve_needs_constant_steer():
    # Same profile, but the steering wanders during the curve: no longer "curve".
    d = make_data()
    t = d.steer["t"] - T0
    m = (t >= 34) & (t <= 44)
    d.steer["angles"][m] += np.radians(10) * np.sin(t[m])[:, None]
    res = yc.analyse(d, yc.Params())
    assert seg(res, 2)["phase"] == "mixed"
    assert seg(res, 2)["steer_spread_deg"] > 3.0


def test_active_command_ends_standstill():
    # A command the wheels do not (yet) follow - deadband, ramp-up - is not a
    # standstill: no bias may be taken there (<<FR+3>>: "no motion command active").
    d = make_data()
    t = d.cmd["t"] - T0
    d.cmd["vx"][(t >= 20.5) & (t <= 22.0)] = 0.05
    res = yc.analyse(d, yc.Params())
    cand = [c for c in res["settle_candidates"] if 15.0 < c["t_start"] < 17.0]
    assert len(cand) == 1 and cand[0]["t_end"] == pytest.approx(20.5, abs=0.06)
    assert any(iv["phase"] == "transition" and iv["t0"] == pytest.approx(20.5, abs=0.06)
               for iv in res["phase_intervals"])


# ---------------------------------------------------------------- recovery

def test_encoder_scale_recovered(result):
    # Fails if encoder increments are not taken between settled windows, or if
    # the reference is not the composed map->base yaw (wrapped odom_base).
    for i in (0, 2):
        assert seg(result, i)["sources"]["encoder"]["scale"] == pytest.approx(ENC_SCALE, abs=0.005)
        assert seg(result, i)["sources"]["encoder_twist"]["scale"] == pytest.approx(ENC_SCALE, abs=0.005)
    assert seg(result, 0)["sources"]["slam"]["increment_deg"] == pytest.approx(360.0, abs=0.3)


def test_laser_odom_scale_recovered(result):
    assert seg(result, 0)["sources"]["laser_odom"]["scale"] == pytest.approx(LO_SCALE, abs=0.005)


def test_lags_recovered_against_gyro(result):
    # Fails on a sign error in the lag convention (+ = source later) or if the
    # gyro is not the lag reference when present.
    s = seg(result, 0)
    assert s["lag_reference"] == "gyro"
    assert s["sources"]["encoder"]["lag_s"] == pytest.approx(ENC_LAG, abs=0.02)
    assert s["sources"]["laser_odom"]["lag_s"] == pytest.approx(LO_LAG, abs=0.03)
    assert s["sources"]["slam"]["lag_s"] == pytest.approx(0.0, abs=0.02)


def test_gyro_bias_estimated_and_removed(result):
    # Without the per-segment bias subtraction the 360 deg turn would carry
    # 0.5 deg/s x ~11 s = ~5.5 deg of error and a scale of ~1.015.
    s = seg(result, 0)
    assert s["gyro_bias_deg_s"] == pytest.approx(BIAS_DEG, abs=0.03)
    assert s["sources"]["gyro"]["scale"] == pytest.approx(1.0, abs=0.002)
    assert abs(s["sources"]["gyro"]["error_deg"]) < 0.3
    assert s["gyro_residual_bias_deg_s"] == pytest.approx(0.0, abs=0.03)


def test_residual_bias_after_bias_step():
    # Bias jumps 0.5 -> 0.8 deg/s at t=17 s, inside the standstill after the turn.
    step = lambda t: np.radians(np.where(t < 17.0, 0.5, 0.8))  # noqa: E731
    res = yc.analyse(make_data(bias_fn=step), yc.Params())
    assert seg(res, 0)["gyro_residual_bias_deg_s"] == pytest.approx(0.3, abs=0.03)
    assert seg(res, 1)["gyro_bias_deg_s"] == pytest.approx(0.8, abs=0.03)


# ---------------------------------------------------------------- IMU mounting

def test_roll_180_mount_read_from_tf(result):
    # The raw IMU z is the NEGATIVE of the base yaw rate; scale +1 proves the
    # flip came from the TF.
    assert result["imu_mount"]["z_row"] == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)
    assert seg(result, 0)["sources"]["gyro"]["scale"] > 0


def test_tilted_mount_not_transposed():
    # Roll 170 + pitch 5 deg is neither symmetric nor its own inverse, so using
    # R^T instead of R (or R twice) gives scale ~0.94, not 1. Raw data come
    # from mount_matrix(), not from the analyser's quaternion helper.
    res = yc.analyse(make_data(true_mount=TILT), yc.Params())
    assert seg(res, 0)["sources"]["gyro"]["scale"] == pytest.approx(1.0, abs=0.002)
    assert res["imu_mount"]["z_row"] == pytest.approx(list(mount_matrix(*TILT)[2]), abs=1e-9)
    assert seg(res, 0)["gyro_bias_deg_s"] == pytest.approx(BIAS_DEG, abs=0.03)


def test_wrong_tf_gives_wrong_sign():
    # Same raw data, identity mount in the TF: the tool must follow the TF, not
    # a sign hardcoded for this robot - so the gyro now runs backwards.
    res = yc.analyse(make_data(imu_mount=IDENTITY), yc.Params())
    assert seg(res, 0)["sources"]["gyro"]["scale"] == pytest.approx(-1.0, abs=0.01)


def test_imu_rpy_override():
    res = yc.analyse(make_data(imu_mount=IDENTITY), yc.Params(imu_rpy=(math.pi, 0.0, 0.0)))
    assert seg(res, 0)["sources"]["gyro"]["scale"] == pytest.approx(1.0, abs=0.002)
    assert res["imu_mount"]["from"].startswith("--imu-rpy")


def test_static_rotation_chain_and_disconnected():
    tf = {"b": ("a", ROLL_PI), "c": ("b", yc.quat_from_rpy(0, 0, math.pi / 2)), "x": ("y", IDENTITY)}
    q = yc.static_rotation(tf, "a", "c")
    v = yc.quat_to_matrix(q) @ np.array([1.0, 0.0, 0.0])
    assert v == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)     # yaw 90 then roll 180
    q_back = yc.static_rotation(tf, "c", "a")
    assert yc.quat_to_matrix(q_back) @ v == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)
    with pytest.raises(KeyError):
        yc.static_rotation(tf, "a", "x")


# ---------------------------------------------------------------- settle rule

def test_noisy_slam_standstill_is_not_settled():
    # 1 deg yaw jitter in the standstill after the turn: that pose fails
    # sigma_settle, so turn and straight merge into one mixed segment.
    res = yc.analyse(make_data(slam_noise=(16.0, 22.0, 1.0)), yc.Params())
    rejected = [c for c in res["settle_candidates"] if not c["settled"]]
    assert len(rejected) == 1 and "scatter" in rejected[0]["reason"]
    assert [s["phase"] for s in res["segments"]] == ["mixed", "curve", "mixed"]


def test_t_settle_longer_than_any_standstill():
    res = yc.analyse(make_data(), yc.Params(t_settle=7.0))
    assert res["settle_candidates"] == [] and res["segments"] == []


def test_require_slam_update():
    # map->odom never changes in the synthetic data, exactly as at a real standstill.
    res = yc.analyse(make_data(), yc.Params(require_slam_update=True))
    assert res["segments"] == []
    assert all(not c["map_odom_updated_in_standstill"] for c in res["settle_candidates"])


# ---------------------------------------------------------------- missing sources

def test_missing_imu_is_reported_not_raised():
    res = yc.analyse(make_data(with_imu=False), yc.Params())
    assert res["sources"]["gyro"]["present"] is False
    assert "no IMU" in res["sources"]["gyro"]["reason"]
    s = seg(res, 0)
    assert "gyro" not in s["sources"] and s["lag_reference"] == "slam"
    assert s["sources"]["encoder"]["scale"] == pytest.approx(ENC_SCALE, abs=0.005)
    assert s["sources"]["encoder"]["lag_s"] == pytest.approx(ENC_LAG, abs=0.03)
    assert "MISSING - no IMU" in yc.format_table(res)


def test_imu_without_mount_is_reported():
    d = make_data()
    del d.static_tf["imu_link"]
    res = yc.analyse(d, yc.Params())
    assert res["sources"]["gyro"]["present"] is False
    assert "mounting unknown" in res["sources"]["gyro"]["reason"]


def test_missing_slam_gives_no_segments():
    res = yc.analyse(make_data(with_slam=False), yc.Params())
    assert res["sources"]["slam"]["present"] is False
    assert res["segments"] == []
    assert all(c["reason"] == "no slam track" for c in res["settle_candidates"])
    yc.format_table(res)


def test_wheel_gap_is_flagged(result):
    res = yc.analyse(make_data(drop_wheel=(10.0, 11.0)), yc.Params())
    assert seg(res, 0)["sources"]["encoder"]["gap"] is True
    assert seg(result, 0)["sources"]["encoder"]["gap"] is False


# ---------------------------------------------------------------- floor mark, output

def test_floor_mark(result):
    res = yc.analyse(make_data(), yc.Params(), floor_marks=["10:360", {"t": 40, "yaw_increment_deg": 57.3}])
    s0 = seg(res, 0)
    assert s0["floor_deg"] == 360.0
    assert s0["sources"]["encoder"]["error_vs_floor_deg"] == pytest.approx(0.09 * 360, abs=0.5)
    assert s0["sources"]["slam"]["error_vs_floor_deg"] == pytest.approx(0.0, abs=0.3)
    assert seg(res, 2)["floor_deg"] == 57.3
    assert seg(res, 1)["floor_deg"] is None


def test_cross_correlation_sign():
    t = np.arange(0, 400)
    ref = np.exp(-0.5 * ((t - 200) / 10.0) ** 2)
    src = np.exp(-0.5 * ((t - 207.3) / 10.0) ** 2)
    lag, corr, at_limit = yc.cross_correlation_lag(ref, src, 20)
    assert lag == pytest.approx(7.3, abs=0.1) and corr > 0.99 and not at_limit


def test_csv_and_json(result, tmp_path):
    yc.write_csv(result, tmp_path / "y.csv")
    yc.write_json(result, tmp_path / "y.json")
    rows = (tmp_path / "y.csv").read_text().strip().splitlines()
    assert len(rows) == 1 + 4 * 6               # header + 4 segments x 6 sources
    text = (tmp_path / "y.json").read_text()
    assert "NaN" not in text
    assert json.loads(text)["segments"][0]["phase"] == "rotation"


# ---------------------------------------------------------------- standstill gyro, reporting

def crop(d, t1):
    """Keep only samples before t1 (s after T0) - here: the first standstill only."""
    for name in ("wheel_odom", "steer", "imu", "map_odom", "odom_base", "laser_odom", "cmd"):
        src = getattr(d, name)
        if src is None:
            continue
        m = (src["t"] - T0) < t1
        setattr(d, name, {k: (v[m] if isinstance(v, np.ndarray) else v) for k, v in src.items()})
    return d


def test_single_standstill_still_reports_bias():
    # No segment exists, yet the bias must be reported: a 1.2-style static bag.
    res = yc.analyse(crop(make_data(), 5.5), yc.Params())
    assert res["segments"] == []
    (g,) = res["standstill_gyro"]
    assert g["mean_z_deg_s"] == pytest.approx(BIAS_DEG, abs=0.03)
    assert g["std_z_deg_s"] == pytest.approx(math.degrees(0.002), rel=0.2)
    assert g["flag"] == ""


def test_exact_zero_gyro_is_flagged_not_bias_zero():
    res = yc.analyse(make_data(zero_imu=True), yc.Params())
    assert res["standstill_gyro"]
    for g in res["standstill_gyro"]:
        assert g["flag"] == yc.ZERO_FLAG and g["mean_z_deg_s"] is None
        assert g["exact_zero_samples"] == g["n"] > 0
    s0 = seg(res, 0)
    assert s0["gyro_flag"] == yc.ZERO_FLAG and s0["gyro_bias_deg_s"] is None
    assert "GYRO OUTPUT EXACTLY ZERO" in yc.format_table(res)


def test_settle_reporting_does_not_change_results(result):
    # map->odom corrects by 2 deg at t=17 s, 1 s after the turn stopped (t=16),
    # outside the settle window 20-22 s. Pure reporting: segments must not move.
    d = make_data()
    t = d.map_odom["t"] - T0
    d.map_odom["yaw"] = np.where(t >= 17.0, 0.3 + math.radians(2.0), 0.3)
    res = yc.analyse(d, yc.Params())
    c = [c for c in res["settle_candidates"] if 15.0 < c["t_start"] < 17.0][0]
    assert c["map_odom_age_s"] == pytest.approx(22.0 - 17.0, abs=0.15)
    assert c["map_odom_age_at_motion_end_s"] == pytest.approx(c["t_start"] - 17.0, abs=0.15)
    assert c["map_odom_age_at_motion_end_s"] < 0          # landed inside the standstill
    assert c["laser_odom_std_deg"] == pytest.approx(0.0, abs=1e-6)
    assert [s["phase"] for s in res["segments"]] == [s["phase"] for s in result["segments"]]
    assert [x["settled"] for x in res["settle_candidates"]] == \
        [x["settled"] for x in result["settle_candidates"]]


def test_encoder_twist_marked_approximate(result):
    assert "encoder_twist  present (APPROXIMATE" in yc.format_table(result)


# ---------------------------------------------------------------- scanmatch reference (1.4b)

@pytest.fixture(scope="module")
def scan_result():
    return yc.analyse(make_data(with_scans=True), yc.Params())


def test_scanmatch_is_the_default_reference(scan_result):
    # Fails if the reference is still slam, if whole turns are lost (0 instead of
    # 360), or if the lidar mount offset is ignored.
    assert scan_result["boundary_rule"] == "scans"
    s0 = seg(scan_result, 0)
    assert s0["reference"] == "scanmatch" and s0["reference_note"] == ""
    sc = s0["scanmatch"]
    assert not sc["failed"], sc["reason"]
    assert sc["increment_deg"] == pytest.approx(360.0, abs=0.15)
    assert sc["turns"] == 1 and sc["guess_from"] == "gyro"
    assert s0["sources"]["encoder"]["scale"] == pytest.approx(ENC_SCALE, abs=0.005)
    # The other references stay side by side.
    assert s0["sources"]["slam"]["increment_deg"] == pytest.approx(360.0, abs=0.3)
    assert "REFERENCE" in yc.format_table(scan_result)


def test_scanmatch_translation_and_all_segments(scan_result):
    # Every segment's truth from the generator's own integral of the profile.
    assert [s["phase"] for s in scan_result["segments"]] == ["rotation", "straight", "curve", "mixed"]
    edges = [(5.0, 21.0), (21.0, 33.0), (33.0, 49.0), (49.0, 65.0)]
    for s, (ta, tb) in zip(scan_result["segments"], edges):
        sc = s["scanmatch"]
        assert not sc["failed"], sc["reason"]
        assert sc["increment_deg"] == pytest.approx(math.degrees(at(tb, _YAW) - at(ta, _YAW)), abs=0.15)
        xa, ya, yaw_a = true_pose(ta)
        xb, yb, _ = true_pose(tb)
        c, sn = math.cos(yaw_a), math.sin(yaw_a)
        assert sc["tx"] == pytest.approx(c * (xb - xa) + sn * (yb - ya), abs=0.02)
        assert sc["ty"] == pytest.approx(-sn * (xb - xa) + c * (yb - ya), abs=0.02)


def test_reference_slam_keeps_old_boundaries(scan_result):
    res = yc.analyse(make_data(with_scans=True), yc.Params(reference="slam"))
    assert res["boundary_rule"] == "slam sigma_settle"
    s0 = seg(res, 0)
    assert s0["reference"] == "slam"
    assert s0["scanmatch"]["increment_deg"] == pytest.approx(360.0, abs=0.15)  # still reported


def test_failed_scanmatch_falls_back_to_slam_with_a_note():
    # Scans of the standstill after the turn carry no returns: no number, and the
    # segments on both sides of it say they fell back to slam.
    res = yc.analyse(make_data(with_scans=True, blank_scans=(16.0, 22.0)), yc.Params())
    for i in (0, 1):
        s = seg(res, i)
        assert s["scanmatch"]["failed"] and s["scanmatch"]["increment_deg"] is None
        assert s["sources"]["scanmatch"]["increment_deg"] is None
        assert s["reference"] == "slam" and "fell back to slam" in s["reference_note"]
    assert seg(res, 2)["reference"] == "scanmatch"
    assert "FELL BACK TO SLAM" in yc.format_table(res).upper()


def test_floor_reference():
    res = yc.analyse(make_data(with_scans=True), yc.Params(reference="floor"), floor_marks=["10:361"])
    s0, s1 = seg(res, 0), seg(res, 1)
    assert s0["reference"] == "floor" and s0["reference_increment_deg"] == 361.0
    assert s0["sources"]["scanmatch"]["error_deg"] == pytest.approx(-1.0, abs=0.15)
    assert s1["reference_increment_deg"] is None and "no floor mark" in s1["reference_note"]


def test_split_at_gives_a_noise_floor_segment():
    # One standstill cut in two: every reference must read ~0 over it.
    d = crop(make_data(with_scans=True), 5.9)
    res = yc.analyse(d, yc.Params(split_at=(2.9,)))
    (s,) = res["segments"]
    assert s["phase"] == "standstill"
    assert s["scanmatch"]["increment_deg"] == pytest.approx(0.0, abs=0.1)
    assert s["scanmatch"]["guess_from"] == "gyro"
    assert yc.analyse(d, yc.Params())["segments"] == []


def test_unknown_reference_rejected():
    with pytest.raises(ValueError):
        yc.analyse(make_data(), yc.Params(reference="laser"))


def test_scan_without_lidar_mount_is_reported():
    d = make_data(with_scans=True)
    del d.static_xyz["lidar_link"]
    res = yc.analyse(d, yc.Params())
    assert res["sources"]["scanmatch"]["present"] is False
    assert "lidar mount unknown" in res["sources"]["scanmatch"]["reason"]
    assert res["boundary_rule"] == "slam sigma_settle"
    assert all(s["reference"] == "slam" for s in res["segments"])


def test_static_transform_full_chain():
    tf = {"b": ("a", yc.quat_from_rpy(0, 0, math.pi / 2)), "c": ("b", IDENTITY)}
    xyz = {"b": (1.0, 0.0, 0.0), "c": (0.5, 0.0, 0.2)}
    r, t = yc.static_transform(tf, xyz, "a", "c")
    # c's origin: 0.5 along b's x, which is a's y, plus b's offset (1, 0, 0).
    assert t == pytest.approx([1.0, 0.5, 0.2], abs=1e-9)
    r2, t2 = yc.static_transform(tf, xyz, "c", "a")
    assert r2 @ t + t2 == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    with pytest.raises(KeyError):
        yc.static_transform(tf, {"b": (1.0, 0.0, 0.0)}, "a", "c")


# ---------------------------------------------------------------- gyro quantisation

Q512 = 1.0 / 512.0


def test_gyro_quantisation_detected_and_resolution_limit_flagged():
    # BNO085-like: raw output in multiples of 1/512 rad/s, noise well below a step.
    res = yc.analyse(make_data(imu_quant=Q512, imu_noise=0.0003), yc.Params())
    for g in res["standstill_gyro"]:
        assert g["quant_step_deg_s"] == pytest.approx(math.degrees(Q512), rel=1e-9)
        assert g["below_one_step"] is True
    assert "bias resolution-limited" in yc.format_table(res)


def test_gyro_quantisation_not_claimed_on_continuous_data(result):
    # Fails if any value set is declared quantised (e.g. a step of the float eps).
    for g in result["standstill_gyro"]:
        assert g["quant_step_deg_s"] is None and g["below_one_step"] is None


def test_quant_step_needs_the_step_itself_or_a_fraction():
    q = 0.01
    k = np.array([0, 2, 4, -6, 2, 4, 4, 2, 0, 2])
    assert yc.quant_step(k * q) == pytest.approx(2 * q)        # only even multiples seen
    k = np.array([4, 5, 4, 4, 5, 5, 4, 4, 5, 4])               # bias ~4.5 steps, no zeros
    assert yc.quant_step(k * q) == pytest.approx(q)
    assert yc.quant_step(np.array([0.0, 0.0, q, 0.0])) is None  # too few non-zero
    rng = np.random.default_rng(0)
    assert yc.quant_step(rng.normal(0, 1, 1000)) is None        # continuous


def test_scan_boundaries_do_not_need_a_settled_slam_pose():
    # slam jitters 1 deg in the standstill after the turn, so sigma_settle rejects
    # it - with scans that standstill still bounds a segment (D6: the settled value
    # is the scan read at rest). Fails if boundaries still follow the slam rule.
    res = yc.analyse(make_data(with_scans=True, slam_noise=(16.0, 22.0, 1.0)), yc.Params())
    c = [c for c in res["settle_candidates"] if 15.0 < c["t_start"] < 17.0][0]
    assert c["settled"] is False and c["boundary"] is True
    assert [s["phase"] for s in res["segments"]] == ["rotation", "straight", "curve", "mixed"]
    assert seg(res, 0)["scanmatch"]["increment_deg"] == pytest.approx(360.0, abs=0.15)


# ---------------------------------------------------------------- live quantised gyro vs dead gyro

def quantised_quiet_gyro(**kw):
    """Quantised gyro, no bias, no noise: every settle window is all-zero.

    Six +-1/+-2-step samples early in the first standstill (0.5-2.5 s, before
    its settle window) prove that run alive with a detectable step, like the desk
    bag's 5.4 % non-zero. Two samples in the second standstill (17, 18 s) make it
    alive but too sparse for a step of its own: the step then comes from the bag.
    The later standstill runs stay exactly zero throughout.
    """
    d = make_data(imu_quant=Q512, imu_noise=0.0, bias_fn=lambda t: np.zeros_like(t), **kw)
    t = d.imu["t"] - T0
    for ti, k in zip((0.6, 0.9, 1.3, 1.7, 2.1, 2.4, 17.0, 18.0), (1, -1, 2, 1, -2, 1, 1, -1)):
        d.imu["w"][int(np.argmin(np.abs(t - ti))), 2] = k * Q512
    return d


def test_all_zero_window_of_a_live_quantised_gyro_is_a_bias_not_a_flag():
    # (a) Fails if the 1.4 flag still fires on an all-zero 2-s window, if the bias
    # is dropped, or if the turn count falls back to the encoder.
    res = yc.analyse(quantised_quiet_gyro(with_scans=True), yc.Params())
    s0 = seg(res, 0)
    assert s0["gyro_flag"] == ""
    assert s0["gyro_bias_deg_s"] == 0.0
    assert s0["gyro_bias_resolution_limited"] is True
    assert s0["scanmatch"]["guess_from"] == "gyro"
    assert s0["sources"]["gyro"]["increment_deg"] == pytest.approx(360.0, abs=0.5)
    assert "(resolution-limited)" in yc.format_table(res)
    # Second run: alive, step from the whole bag.
    s1 = seg(res, 1)
    assert s1["gyro_flag"] == "" and s1["gyro_bias_deg_s"] == 0.0
    assert s1["gyro_bias_resolution_limited"] is True
    # Third and fourth runs: every sample zero on all axes - the dead case stays
    # flagged even with a live bag (Fails if the bag fallback overrides it.)
    for i in (2, 3):
        assert seg(res, i)["gyro_flag"] == yc.ZERO_FLAG and seg(res, i)["gyro_bias_deg_s"] is None


def test_dead_run_is_still_flagged_even_if_the_bag_is_alive():
    # (b) Every sample of the first standstill run zero on all axes: dead there,
    # whatever the rest of the bag does.
    d = make_data(imu_quant=Q512)
    t = d.imu["t"] - T0
    d.imu["w"][t < 7.0] = 0.0          # past the run end (motion starts ~6.1 s)
    res = yc.analyse(d, yc.Params())
    s0 = seg(res, 0)
    assert s0["gyro_flag"] == yc.ZERO_FLAG and s0["gyro_bias_deg_s"] is None
    assert seg(res, 1)["gyro_flag"] == ""
