"""scan_match must RECOVER a known planar motion from synthetic scans, and refuse
to give a number when the scans cannot carry one.

The scans are ray-cast here, independently of the analyser: a room polygon with
an interior box (so no rotational symmetry), seen by a lidar mounted 0.2488 m
ahead of the rotation centre like the real one, with Gaussian range noise and a
dense robot-fixed self-return cluster (arm/gripper) behind the sensor. Nothing in
this file uses scan_match's own geometry helpers to build the data.

Tolerance, stated once: yaw within 0.15 deg, translation within 0.02 m, for
1 cm range noise. What would make each test fail is stated beside it.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from gripperx_localization import scan_match as sm  # noqa: E402

YAW_TOL = 0.15        # deg
TRANS_TOL = 0.02      # m
NOISE = 0.01          # m, 1-sigma range noise
LIDAR_X = 0.2488      # m, mount offset ahead of base_footprint (real /tf_static value)
N_BEAMS = 453
INC = 2 * math.pi / N_BEAMS

# Room corners (m), deliberately irregular; plus a 0.6 x 0.4 m box inside.
ROOM = [(-3.0, -2.0), (4.0, -2.5), (4.5, 1.5), (1.0, 3.0), (-2.5, 2.2)]
BOX = [(1.5, -0.5), (2.1, -0.5), (2.1, -0.1), (1.5, -0.1)]
CIRCLE = [(3.0 * math.cos(a), 3.0 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 200, endpoint=False)]


def _walls(*polys):
    segs = []
    for poly in polys:
        for i in range(len(poly)):
            segs.append((poly[i], poly[(i + 1) % len(poly)]))
    return np.array(segs, dtype=float)                      # (S, 2, 2)


ROOM_WALLS = _walls(ROOM, BOX)
CIRCLE_WALLS = _walls(CIRCLE)


def raycast(pose, walls, rng, self_returns=True, noise=NOISE, plate=0.20):
    """Ranges of one scan from robot pose (x, y, yaw); lidar frame angles 0..2pi.

    Like the real driver: no return in 140-210 deg (arm shadow) unless
    self_returns is set, in which case that sector holds a robot-fixed flat
    plate `plate` m behind the sensor (default 0.20 m: 0.20-0.26 m range) - a
    straight edge, so it gets line normals and is a credible match partner at 0 deg.
    """
    x, y, yaw = pose
    ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
    ang = np.arange(N_BEAMS) * INC
    dx, dy = np.cos(yaw + ang), np.sin(yaw + ang)
    a, b = walls[:, 0], walls[:, 1]
    ex, ey = b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]
    # Solve o + s d = a + u e for s (range) and u (position on the wall).
    den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
    ax, ay = a[None, :, 0] - ox, a[None, :, 1] - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (ax * ey[None, :] - ay * ex[None, :]) / den
        u = (ax * dy[:, None] - ay * dx[:, None]) / den
    hit = (np.abs(den) > 1e-12) & (s > 0) & (u >= 0) & (u <= 1)
    r = np.where(hit, s, np.inf).min(axis=1)
    r = r + rng.normal(0, noise, N_BEAMS)
    deg = np.degrees(ang)
    shadow = (deg >= 140) & (deg <= 210)
    r[shadow] = np.inf
    if self_returns:
        r[shadow] = -plate / np.cos(ang[shadow]) + rng.normal(0, noise, shadow.sum())
    return r


def scans_at(pose, n, walls=ROOM_WALLS, seed=0, **kw):
    rng = np.random.default_rng(seed)
    return [raycast(pose, walls, rng, **kw) for _ in range(n)]


MOUNT_R = np.eye(3)
MOUNT_T = np.array([LIDAR_X, 0.0, 0.1034])


def to_points(scans, sp):
    return [sm.scan_points(r, 0.0, INC, 0.1, 25.0, MOUNT_R, MOUNT_T, sp) for r in scans]


def run(true_deg, guess_deg, trans=(0.05, -0.03), sp=None, n=5, walls=ROOM_WALLS, unc=None,
        second=None, guess_t=None, **kw):
    sp = sp or sm.ScanMatchParams()
    pose0 = (0.3, 0.2, math.radians(20.0))
    # The translation is given in the robot frame at pose0.
    c, s = math.cos(pose0[2]), math.sin(pose0[2])
    pose1 = (pose0[0] + c * trans[0] - s * trans[1], pose0[1] + s * trans[0] + c * trans[1],
             pose0[2] + math.radians(true_deg))
    before = to_points(scans_at(pose0, n, walls, seed=1, **kw), sp)
    after = to_points(scans_at(pose1, n, walls, seed=2, **kw), sp)
    return sm.match_sets(before, after, guess_deg, guess_t, sp, guess_unc_deg=unc,
                         second_guess_deg=second)


# Guess = truth x 1.09 (the encoder's measured in-place over-report) with the
# default 17 % uncertainty; for three turns that uncertainty exceeds 180 deg, so
# 1085 deg is run with a gyro-like guess (truth x 1.03, uncertainty 60 deg).
@pytest.mark.parametrize("true_deg, guess, unc", [
    (5.0, 1.09 * 5.0, None), (30.0, 1.09 * 30.0, None), (90.0, 1.09 * 90.0, None),
    (180.0, 1.09 * 180.0, None), (-90.0, 1.09 * -90.0, None),
    (375.0, 1.09 * 375.0, None), (1085.0, 1.03 * 1085.0, 60.0)])
def test_known_rotation_recovered(true_deg, guess, unc):
    # Fails if the yaw sign or frame is wrong, if whole turns are not restored
    # from the guess, if the lidar offset is ignored (translation ~0.25 m * angle),
    # or if the self-return cluster pulls the match toward 0.
    res = run(true_deg, guess, unc=unc)
    assert not res["failed"], res["reason"]
    assert res["increment_deg"] == pytest.approx(true_deg, abs=YAW_TOL)
    assert res["tx"] == pytest.approx(0.05, abs=TRANS_TOL)
    assert res["ty"] == pytest.approx(-0.03, abs=TRANS_TOL)
    assert res["scatter_deg"] < 0.1
    assert res["n_ok"] == res["n_pairs"] == 5
    # Lower for large turns: the arm shadow and the box occlude other wall parts.
    assert res["inlier_frac"] > 0.6


def test_three_turns_need_the_guess_for_the_turn_count():
    # The scans alone cannot tell 1085 from 5 deg: the guess decides the turns.
    for guess, expect in ((1080.0 + 5.0 + 60.0, 1085.0), (5.0 + 40.0, 5.0), (-355.0 + 30.0, -355.0)):
        res = run(1085.0, guess, unc=70.0)
        assert not res["failed"], res["reason"]
        assert res["increment_deg"] == pytest.approx(expect, abs=YAW_TOL)
        assert res["frac_deg"] == pytest.approx(5.0, abs=YAW_TOL)


def test_resolve_turns():
    assert sm.resolve_turns(1180.0, 5.0) == (1085.0, 3)
    assert sm.resolve_turns(-1180.0, -5.0) == (-1085.0, -3)
    assert sm.resolve_turns(170.0, -175.0) == (185.0, 1)


def test_turn_count_fails_when_guess_too_uncertain():
    # 0.17 x 1180 deg = 201 deg >= 180: an encoder-like guess cannot pin three
    # turns. The fraction is kept, the increment is not given.
    res = run(1085.0, 1.09 * 1085.0)
    assert res["failed"] and res["turns_ambiguous"] is True
    assert res["increment_deg"] is None and res["turns"] is None
    assert res["frac_deg"] == pytest.approx(5.0, abs=YAW_TOL)
    assert "turn count unresolved" in res["reason"]
    assert run(375.0, 400.0)["turns_ambiguous"] is False


def test_guess_off_by_185_deg_at_three_turns_never_gives_a_number():
    # Audit case: 1085 deg with a guess 185 deg too high must not come out as
    # 1085 + 360. Encoder-like uncertainty -> turn count ambiguous; gyro-like
    # uncertainty -> the true fraction lies outside the plausibility bound.
    for unc in (None, 0.05 * 1270.0):
        res = run(1085.0, 1270.0, unc=unc)
        assert res["failed"] and res["increment_deg"] is None


# Audit cases: a guess ~120 deg off. Before the plausibility guard, ICP left the
# searched window and settled in a wrong basin that passed every other gate
# (5 -> +167.7 deg, 90 -> 253 deg).
@pytest.mark.parametrize("true_deg, guess", [(5.0, 125.0), (90.0, 210.0)])
def test_wrong_basin_far_from_the_guess_is_rejected(true_deg, guess):
    res = run(true_deg, guess)
    assert res["failed"] and res["increment_deg"] is None
    assert "implausible" in res["reason"]


def test_gyro_encoder_disagreement_fails_the_turn_count_only():
    # Gyro says 1100, encoder 1290: their turn counts differ (3 vs 4), so no
    # increment - but the fraction is still reported.
    res = run(1085.0, 1100.0, unc=60.0, second=1290.0)
    assert res["failed"] and res["increment_deg"] is None
    assert res["frac_deg"] == pytest.approx(5.0, abs=YAW_TOL)
    assert res["guess_disagreement_deg"] == pytest.approx(-190.0)
    assert "turn count" in res["turns_reason"]
    # Same turn count but 95 deg apart: beyond max_guess_disagreement_deg.
    res = run(375.0, 380.0, unc=25.0, second=285.0)
    assert res["failed"] and "disagree" in res["turns_reason"]
    # Agreeing sources: accepted, disagreement reported.
    res = run(375.0, 380.0, unc=25.0, second=405.0)
    assert not res["failed"] and res["increment_deg"] == pytest.approx(375.0, abs=YAW_TOL)
    assert res["guess_disagreement_deg"] == pytest.approx(-25.0)


def test_self_return_plate_pulls_without_the_filter():
    # Control: proves the robot-fixed plate in the synthetic data is a real
    # threat, so that the recovery tests actually test the filter. It sits near
    # the rotation centre, so it barely constrains yaw but, being an inlier at
    # zero motion, drags a small translation toward 0 (measured here: tx 0.031
    # instead of 0.050 m at 5 deg). Filters off -> pulled; on -> exact.
    off = sm.ScanMatchParams(range_min=0.05, exclude_lo_deg=0.0, exclude_hi_deg=0.0)
    res = run(5.0, 5.5, sp=off)
    assert not res["failed"] and abs(res["tx"] - 0.05) > 0.01
    res = run(5.0, 5.5)
    assert not res["failed"]
    assert res["tx"] == pytest.approx(0.05, abs=0.005)
    assert res["increment_deg"] == pytest.approx(5.0, abs=YAW_TOL)
    # Each of the two filters alone removes the plate.
    for alone in (sm.ScanMatchParams(exclude_lo_deg=0.0, exclude_hi_deg=0.0),
                  sm.ScanMatchParams(range_min=0.05)):
        res = run(5.0, 5.5, sp=alone)
        assert not res["failed"] and res["tx"] == pytest.approx(0.05, abs=0.005)


def test_robot_fixed_return_beyond_range_min_needs_the_sector():
    # A plate at 0.45 m (0.45-0.59 m range) passes range_min = 0.35 m: only the
    # sector filter removes it. Without the sector it drags the translation.
    off = sm.ScanMatchParams(exclude_lo_deg=0.0, exclude_hi_deg=0.0)
    res = run(5.0, 5.5, sp=off, plate=0.45)
    assert not res["failed"] and abs(res["tx"] - 0.05) > 0.01
    res = run(5.0, 5.5, plate=0.45)
    assert not res["failed"] and res["tx"] == pytest.approx(0.05, abs=0.005)
    assert res["increment_deg"] == pytest.approx(5.0, abs=YAW_TOL)


def test_large_translation_needs_the_translation_guess():
    # 2 m of travel: with the encoder translation as the start the match is
    # exact; from zero translation the coarse search lands wrong and fails.
    # Fails if guess_t is ignored.
    for deg in (30.0, 90.0):
        res = run(deg, 1.09 * deg, trans=(2.0, 0.5), guess_t=(2.0, 0.5))
        assert not res["failed"], res["reason"]
        assert res["increment_deg"] == pytest.approx(deg, abs=YAW_TOL)
        assert res["tx"] == pytest.approx(2.0, abs=TRANS_TOL)
        assert run(deg, 1.09 * deg, trans=(2.0, 0.5))["failed"]


def test_scan_points_frame_and_filters():
    sp = sm.ScanMatchParams()
    # Beam at 90 deg, 2 m: lidar (0, 2) -> base (0.2488, 2). Beam at 180 deg is in
    # the excluded sector, the 0.2 m one below range_min, inf is dropped.
    r = np.full(4, np.inf)
    r[1] = 2.0
    pts = sm.scan_points(r, 0.0, math.pi / 2, 0.1, 25.0, MOUNT_R, MOUNT_T, sp)
    assert pts == pytest.approx(np.array([[LIDAR_X, 2.0]]), abs=1e-9)
    r = np.array([1.0, 0.2, 3.0, 1.0])
    pts = sm.scan_points(r, 0.0, math.pi / 2, 0.1, 25.0, MOUNT_R, MOUNT_T, sp)
    assert len(pts) == 2          # 90 deg beam too close, 180 deg beam in the sector
    # A yawed mount (lidar turned +90 deg): lidar x-axis points along base y.
    yawed = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    pts = sm.scan_points(np.array([2.0]), 0.0, 1.0, 0.1, 25.0, yawed, np.zeros(3), sp)
    assert pts == pytest.approx(np.array([[0.0, 2.0]]), abs=1e-9)


def test_wrapping_exclusion_sector():
    sp = sm.ScanMatchParams(exclude_lo_deg=350.0, exclude_hi_deg=10.0)
    ang = np.radians([0.0, 5.0, 20.0, 355.0, 180.0])
    pts = sm.scan_points(np.ones(1), 0.0, 1.0, 0.1, 25.0, np.eye(3), np.zeros(3), sp)
    assert len(pts) == 0          # beam at 0 deg excluded
    kept = [len(sm.scan_points(np.ones(1), a, 1.0, 0.1, 25.0, np.eye(3), np.zeros(3), sp)) for a in ang]
    assert kept == [0, 0, 1, 0, 1]


# ---------------------------------------------------------------- failure cases

def test_too_few_points_fails_without_a_number():
    rng = np.random.default_rng(3)
    sparse = []
    for _ in range(5):
        r = np.full(N_BEAMS, np.inf)
        r[rng.choice(N_BEAMS, 20, replace=False)] = rng.uniform(1, 3, 20)
        sparse.append(r)
    sp = sm.ScanMatchParams()
    pts = to_points(sparse, sp)
    res = sm.match_sets(pts, pts, 30.0, None, sp)
    assert res["failed"] and res["increment_deg"] is None
    assert "too few points" in res["reason"]


def test_featureless_circular_room_fails():
    # Centred in a circle, every rotation fits equally well: yaw is unobservable
    # even though inlier fraction and residual look perfect. Fails if the
    # rotational-lever check is missing.
    sp = sm.ScanMatchParams()
    pose = (-LIDAR_X, 0.0, 0.0)                    # lidar at the circle centre
    before = to_points(scans_at(pose, 3, CIRCLE_WALLS, seed=1, self_returns=False), sp)
    after = to_points(scans_at((-LIDAR_X * math.cos(0.5), -LIDAR_X * math.sin(0.5), 0.5), 3,
                               CIRCLE_WALLS, seed=2, self_returns=False), sp)
    res = sm.match_sets(before, after, 25.0, None, sp)
    assert res["failed"] and res["increment_deg"] is None
    assert "unobservable" in res["reason"]


def test_symmetric_room_inside_the_window_is_ambiguous():
    # Empty square room centred on the rotation centre: 30 and 120 deg fit equally
    # well. With a guess loose enough to put both into the search window the
    # result must be FAILED, not whichever basin happened to win. Fails if the
    # distinctness check is missing.
    square = _walls([(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)])
    sp = sm.ScanMatchParams(guess_rel_err=1.0)          # window +-75 deg around 75
    before = to_points(scans_at((0.0, 0.0, 0.0), 3, square, seed=1), sp)
    after = to_points(scans_at((0.0, 0.0, math.radians(30.0)), 3, square, seed=2), sp)
    res = sm.match_sets(before, after, 75.0, None, sp)
    assert res["failed"] and res["increment_deg"] is None
    assert "ambiguous" in res["reason"]
    # Same scans, tight guess: only one basin in the window, so it is accepted.
    res = sm.match_sets(before, after, 33.0, None, sm.ScanMatchParams())
    assert not res["failed"] and res["increment_deg"] == pytest.approx(30.0, abs=YAW_TOL)


def test_low_inlier_fraction_fails():
    # 80 % of the after-scan beams replaced by clutter (people, a moved chair):
    # inlier fraction ~0.2, below the 0.3 floor. Fails if the floor is missing.
    rng = np.random.default_rng(5)
    sp = sm.ScanMatchParams()
    before = scans_at((0.3, 0.2, math.radians(20.0)), 5, seed=1)
    after = scans_at((0.35, 0.17, math.radians(50.0)), 5, seed=2)
    for r in after:
        m = rng.random(len(r)) < 0.8
        r[m] = rng.uniform(0.5, 3.0, m.sum())
    res = sm.match_sets(to_points(before, sp), to_points(after, sp), 33.0, None, sp)
    assert res["failed"] and res["increment_deg"] is None
    assert "inlier fraction" in res["reason"]


def test_scatter_across_pairs_fails():
    # The robot was not at rest: the after-scans differ by up to +-1.5 deg from
    # each other. Every pair matches cleanly, but the set must be rejected.
    sp = sm.ScanMatchParams()
    pose0 = (0.3, 0.2, math.radians(20.0))
    rng = np.random.default_rng(1)
    before = to_points([raycast(pose0, ROOM_WALLS, rng) for _ in range(5)], sp)
    after = to_points([raycast((0.3, 0.2, math.radians(50.0 + d)), ROOM_WALLS, rng)
                       for d in (0.0, 1.0, -1.0, 1.5, -1.5)], sp)
    res = sm.match_sets(before, after, 33.0, None, sp)
    assert res["n_ok"] == 5
    assert res["failed"] and res["increment_deg"] is None and "scatter" in res["reason"]
    assert res["scatter_deg"] == pytest.approx(np.std([0, 1, -1, 1.5, -1.5], ddof=1), abs=0.1)


def test_no_scans_on_one_side():
    res = sm.match_sets([], [np.zeros((100, 2))], 10.0)
    assert res["failed"] and res["reason"] == "no scans on one side"


def test_pick_scans_respects_margin_and_count():
    t = np.arange(0.0, 10.0, 0.1)
    sp = sm.ScanMatchParams(count=4, margin_sec=1.0)
    idx = sm.pick_scans(t, 2.0, 5.0, sp)
    assert [round(t[i], 1) for i in idx] == [4.7, 4.8, 4.9, 5.0]
    assert sm.pick_scans(t, 2.0, 2.8, sp) == []          # all inside the margin
