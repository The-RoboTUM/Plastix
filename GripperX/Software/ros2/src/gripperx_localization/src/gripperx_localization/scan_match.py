"""Planar scan-to-scan matching at standstill: the settled LiDAR yaw reference.

WHY THIS EXISTS. Decision D6 of the local tracking issue
claude/docs/ISSUE_DRAFT_LIDAR_IMU_2026-10-22.md (sub-issue 1.4b): the reference of
yaw_compare must be a LiDAR value read at standstill. slam_toolbox's map pose is
frozen at standstill (no scan is processed below minimum_travel_distance/heading),
so it holds the last match made DURING motion. Here the scans taken at rest before
a motion are matched directly against the scans taken at rest after it.

METHOD, and why. A self-contained numpy ICP (iterative closest point), point-to-line
variant, after a coarse rotational search. Chosen over the vendored csm /
ros2_laser_scan_matcher because it needs no build dependency, runs offline on
plain arrays (testable without ROS), and exposes the quantities the report needs
(inlier fraction, residual, rotational observability). Scans are small (~330
valid beams), so nearest neighbours are found by brute force; no KD-tree.

    1. Points: every scan is converted to x/y in base_footprint through the full
       static lidar mount (the lidar sits 0.25 m ahead of the rotation centre;
       matching in lidar_link would turn an in-place rotation into a translation).
       Self-returns are dropped BEFORE that, in the lidar frame: the arm and gripper
       sit permanently in the scan plane behind the sensor and rotate WITH the
       robot, so they would pull every match toward 0 deg. Two filters, both
       CHOSEN, TO-VERIFY: a minimum range and an excluded angular sector.
    2. Coarse search: the yaw increment is searched on a 1-deg grid in a window
       around the guess (gyro, else encoder), translation held at its guess; the
       best few distinct minima are refined by ICP and the best refinement wins.
    3. ICP: point-to-line residuals against line normals fitted to the "before"
       scan, inlier distance shrinking from icp_start_dist to inlier_dist.
    4. Quality, per scan pair: inlier fraction, point-to-line RMS (root mean
       square) residual, the rotational lever - how strongly the inlier
       geometry constrains rotation once translation is free (metres; ~0 in a
       circular room centred on the robot, where yaw is unobservable) - and the
       distinctness: best inlier fraction over that of the best refinement that
       ended more than 2 deg away (a symmetric room inside the search window
       gives ~1). A pair fails on too few points, a low inlier fraction, a short
       lever or a low distinctness. The inlier fraction alone is a weak test: it
       falls with the overlap of the two views (0.45-0.5 measured after 1.5 m of
       travel on the 2026-10-08 coast bag, with a clean unique match).
    5. Per standstill pair: N scan pairs, the median of the good pairs is the
       fractional increment, their standard deviation the scatter. FAILED when
       too few pairs are good or the scatter is too large; a FAILED result
       carries no number.

WHOLE TURNS. A scan matcher sees orientation modulo 360 deg: 1080+5 deg and 5 deg
give the same pair of scans. The integer number of turns therefore comes from the
guess (gyro if present, else encoder) and only the fractional part from the scan
match: increment = frac + 360 * round((guess - frac) / 360). This is right as long
as the guess is within +-180 deg of the truth. The guess's own uncertainty U is
supplied by the caller (yaw_compare: encoder encoder_rel_err x |guess|; gyro
gyro_scale_tol x |guess| + bias uncertainty x duration), else guess_rel_err x
|guess|. Three guards, all CHOSEN, TO-VERIFY:
    - search window: guess +- max(guess_min_window_deg, U);
    - plausibility: a pair whose fractional result lies further than
      drift_bound_factor x window from the guess fraction is rejected (ICP can
      leave the searched window and settle in a wrong basin - an audit found
      true 5 deg / guess 125 deg ending at +167.7 deg with every other gate
      passed);
    - consistency: the full circle is also searched; a pair is rejected when an
      orientation outside the window fits the scans at least as well as the
      result inside it (the wrong guess then sits next to a near-alias of the
      room, e.g. 1085 deg guessed as 1270 deg gave +167.7 deg, inliers 0.45,
      against 0.98 at the true +5 deg outside the window);
    - turn count: FAILED (fraction kept, no increment) when U >= 180 deg
      ("turns ambiguous"), or when a second source (encoder beside the gyro)
      gives a different turn count or disagrees by more than
      max_guess_disagreement_deg.
Consequence for the comparison: a gyro or encoder error is measured only while it
stays inside these bounds; a larger one shows as FAILED, not as a number.

EVERY THRESHOLD IS CHOSEN, NOT DERIVED - TO-VERIFY from the 1.3/1.5 bags.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class ScanMatchParams:
    """Defaults CHOSEN, TO-VERIFY (see module doc)."""

    count: int = 10                     # scans per standstill side (last N of the standstill)
    margin_sec: float = 1.0             # scans must be stamped this long after motion end
    range_min: float = 0.35             # m; arm/gripper self-returns lie at 0.10-0.35 m
    range_max: float = 20.0             # m; sensor reports up to 25 m
    exclude_lo_deg: float = 140.0       # lidar-frame sector of the arm/gripper, [lo, hi];
    exclude_hi_deg: float = 220.0       # lo == hi disables it, lo > hi wraps through 0
    coarse_step_deg: float = 1.0
    guess_rel_err: float = 0.17         # fallback guess uncertainty, share of |guess|
    encoder_rel_err: float = 0.17       # encoder over-report up to ~17 % of |increment| (measured)
    gyro_scale_tol: float = 0.05        # gyro scale uncertainty, share of |increment|
    gyro_bias_unc_deg_s: float = 0.05   # gyro bias uncertainty without a known step (else step/2)
    drift_bound_factor: float = 1.0     # result may lie this many windows from the guess fraction
    max_guess_disagreement_deg: float = 90.0  # gyro vs encoder, beyond which turns are not trusted
    guess_min_window_deg: float = 20.0  # coarse half-window never narrower than this
    coarse_cap: float = 0.3             # m, distance cap of the coarse score
    coarse_candidates: int = 3          # distinct coarse minima refined by ICP
    coarse_subsample: int = 3           # every n-th 'after' point in the coarse score
    icp_start_dist: float = 0.3         # m, first-iteration inlier distance
    inlier_dist: float = 0.08           # m, final inlier distance (beam spacing at 6 m ~8 cm)
    max_iter: int = 60
    normal_k: int = 5                   # neighbours per line-normal fit
    normal_max_dist: float = 0.5        # m, a neighbourhood wider than this gives no normal
    min_points: int = 50                # valid points per scan after filtering
    min_inlier_frac: float = 0.3        # overlap-dependent: 0.45-0.5 seen after 1.5 m of travel
    min_distinct_ratio: float = 1.5     # best / runner-up basin inlier fraction
    min_rot_lever: float = 0.2          # m, rotational observability floor
    max_scatter_deg: float = 0.5        # std of the per-pair yaw across pairs
    min_ok_frac: float = 0.5            # share of scan pairs that must pass


# --------------------------------------------------------------------------- geometry

def _rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def resolve_turns(guess_deg: float, frac_deg: float) -> Tuple[float, int]:
    """Whole turns from the guess, fraction from the scan match (module doc)."""
    n = int(round((guess_deg - frac_deg) / 360.0))
    return frac_deg + 360.0 * n, n


def scan_points(ranges, angle_min: float, angle_inc: float, msg_range_min: float,
                msg_range_max: float, mount_r: np.ndarray, mount_t: np.ndarray,
                sp: ScanMatchParams) -> np.ndarray:
    """One LaserScan -> (N, 2) points in the base frame, self-returns removed.

    mount_r (3x3), mount_t (3): v_base = R v_lidar + t, from /tf_static.
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r >= max(sp.range_min, msg_range_min)) \
        & (r <= min(sp.range_max, msg_range_max))
    lo, hi = sp.exclude_lo_deg % 360.0, sp.exclude_hi_deg % 360.0
    if lo != hi:
        d = np.degrees(a) % 360.0
        ok &= ~(((d >= lo) & (d <= hi)) if lo < hi else ((d >= lo) | (d <= hi)))
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    return np.column_stack([x, y]) @ mount_r[:2, :2].T + mount_t[:2]


def _sqdist(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (np.sum(a * a, axis=1)[:, None] + np.sum(b * b, axis=1)[None, :]
            - 2.0 * a @ b.T).clip(min=0.0)


def line_normals(q: np.ndarray, sp: ScanMatchParams):
    """Unit normals of local line fits. -> (normals, valid mask)."""
    k = min(sp.normal_k, len(q) - 1)
    if k < 2:
        return np.zeros_like(q), np.zeros(len(q), dtype=bool)
    d = _sqdist(q, q)
    idx = np.argpartition(d, k, axis=1)[:, :k + 1]
    nb = q[idx]                                            # (K, k+1, 2)
    spread = np.sqrt(np.take_along_axis(d, idx, axis=1).max(axis=1))
    c = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", c, c)
    w, v = np.linalg.eigh(cov)                             # ascending
    normals = v[:, :, 0]
    # A blob (both eigenvalues alike) has no line direction.
    valid = (spread <= sp.normal_max_dist) & (w[:, 0] <= 0.1 * np.maximum(w[:, 1], 1e-12))
    return normals, valid


# --------------------------------------------------------------------------- ICP

def icp(p: np.ndarray, q: np.ndarray, nq: np.ndarray, theta0: float, t0: np.ndarray,
        sp: ScanMatchParams) -> dict:
    """Point-to-line ICP of p (after) onto q (before). Transform maps p into q's frame."""
    theta, t = float(theta0), np.asarray(t0, dtype=float).copy()
    thr = sp.icp_start_dist
    it = 0
    for it in range(1, sp.max_iter + 1):
        pp = p @ _rot(theta).T + t
        d2 = _sqdist(pp, q)
        j = np.argmin(d2, axis=1)
        inl = np.sqrt(d2[np.arange(len(pp)), j]) < thr
        if inl.sum() < 3:
            break
        n, qq, pi = nq[j[inl]], q[j[inl]], pp[inl]
        r = np.sum(n * (pi - qq), axis=1)
        jac = np.column_stack([n[:, 0] * -pi[:, 1] + n[:, 1] * pi[:, 0], n[:, 0], n[:, 1]])
        h = jac.T @ jac
        try:
            delta = -np.linalg.solve(h, jac.T @ r)
        except np.linalg.LinAlgError:
            delta = -np.linalg.lstsq(h, jac.T @ r, rcond=None)[0]
        theta += delta[0]
        t = _rot(delta[0]) @ t + delta[1:]
        small = abs(delta[0]) < 1e-6 and np.hypot(*delta[1:]) < 1e-5
        if thr <= sp.inlier_dist and small:
            break
        thr = max(sp.inlier_dist, 0.7 * thr)
    # Final quality at the final inlier distance.
    pp = p @ _rot(theta).T + t
    d2 = _sqdist(pp, q)
    j = np.argmin(d2, axis=1)
    inl = np.sqrt(d2[np.arange(len(pp)), j]) < sp.inlier_dist
    out = {"theta": theta, "t": t, "iterations": it, "inlier_frac": float(inl.mean()),
           "rms": None, "rot_lever": 0.0}
    if inl.sum() >= 3:
        n, qq, pi = nq[j[inl]], q[j[inl]], pp[inl]
        r = np.sum(n * (pi - qq), axis=1)
        out["rms"] = float(np.sqrt(np.mean(r * r)))
        jac = np.column_stack([n[:, 0] * -pi[:, 1] + n[:, 1] * pi[:, 0], n[:, 0], n[:, 1]])
        h = jac.T @ jac
        # Information on theta left after translation is marginalised out (Schur
        # complement), as an RMS lever arm in metres.
        s = h[0, 0] - h[0, 1:] @ np.linalg.pinv(h[1:, 1:]) @ h[1:, 0]
        out["rot_lever"] = float(math.sqrt(max(s, 0.0) / inl.sum()))
    return out


def guess_uncertainty_deg(guess_deg: float, sp: ScanMatchParams, unc_deg=None) -> float:
    return sp.guess_rel_err * abs(guess_deg) if unc_deg is None else float(unc_deg)


def coarse_window_deg(guess_deg: float, sp: ScanMatchParams, unc_deg=None) -> float:
    return max(sp.guess_min_window_deg, guess_uncertainty_deg(guess_deg, sp, unc_deg))


def match_pair(p: np.ndarray, q: np.ndarray, guess_deg: float, guess_t, sp: ScanMatchParams,
               unc_deg=None) -> dict:
    """Match one 'after' point set p onto one 'before' set q. Yaw is FRACTIONAL."""
    out = {"ok": False, "reason": "", "n_after": int(len(p)), "n_before": int(len(q)),
           "yaw_frac_deg": None, "tx": None, "ty": None, "inlier_frac": None,
           "rms": None, "rot_lever": None}
    if len(p) < sp.min_points or len(q) < sp.min_points:
        out["reason"] = f"too few points ({len(q)} before, {len(p)} after < {sp.min_points})"
        return out
    nq, valid = line_normals(q, sp)
    q, nq = q[valid], nq[valid]
    if len(q) < sp.min_points:
        out["reason"] = f"too few line points in the before scan ({len(q)})"
        return out
    t0 = np.asarray(guess_t if guess_t is not None else (0.0, 0.0), dtype=float)
    w = min(180.0, coarse_window_deg(guess_deg, sp, unc_deg))
    centre = wrap_deg(guess_deg)
    # The coarse score covers the full circle: inside the window it seeds the
    # match, outside it seeds the consistency check below.
    grid = centre + np.arange(-180.0, 180.0, sp.coarse_step_deg)
    score = np.empty(len(grid))
    pc = p[::sp.coarse_subsample]
    for i, g in enumerate(grid):
        pp = pc @ _rot(math.radians(g)).T + t0
        d = np.sqrt(_sqdist(pp, q).min(axis=1))
        score[i] = np.minimum(d, sp.coarse_cap).mean()
    inside = np.abs(grid - centre) <= w

    def best_starts(mask):
        starts = []
        for i in np.argsort(np.where(mask, score, np.inf)):
            if not mask[i]:
                break
            if all(abs(wrap_deg(grid[i] - s)) >= 10.0 for s in starts):
                starts.append(float(grid[i]))
            if len(starts) >= sp.coarse_candidates:
                break
        return starts

    results = [icp(p, q, nq, math.radians(s), t0, sp) for s in best_starts(inside)]
    res = max(results, key=lambda r: (r["inlier_frac"],
                                      -(r["rms"] if r["rms"] is not None else 1e9)))
    others = [r["inlier_frac"] for r in results
              if abs(wrap_deg(math.degrees(r["theta"] - res["theta"]))) > 2.0]
    runner = max(others) if others else None
    distinct = (None if runner is None
                else (math.inf if runner == 0 else res["inlier_frac"] / runner))
    frac = wrap_deg(math.degrees(res["theta"]))
    drift = abs(wrap_deg(frac - centre))
    # Consistency with the scans outside the window: if the scans fit at least as
    # well at an orientation the guess rules out, the guess (or the basin) is wrong.
    out_best = None
    if w < 180.0:
        for r in (icp(p, q, nq, math.radians(s), t0, sp) for s in best_starts(~inside)):
            if abs(wrap_deg(math.degrees(r["theta"]) - centre)) > w and (
                    out_best is None or r["inlier_frac"] > out_best[0]):
                out_best = (r["inlier_frac"], wrap_deg(math.degrees(r["theta"])))
    out.update(yaw_frac_deg=frac, tx=float(res["t"][0]),
               ty=float(res["t"][1]), inlier_frac=res["inlier_frac"], rms=res["rms"],
               rot_lever=res["rot_lever"], iterations=res["iterations"],
               runner_up_inlier_frac=runner, distinctness=distinct, drift_deg=drift,
               outside_best_inlier_frac=out_best and out_best[0],
               outside_best_deg=out_best and out_best[1])
    # Order: geometry first (a scan pair that cannot fix yaw is reported as
    # such), then the guess-related checks.
    if res["inlier_frac"] < sp.min_inlier_frac:
        out["reason"] = f"inlier fraction {res['inlier_frac']:.2f} < {sp.min_inlier_frac}"
    elif res["rot_lever"] < sp.min_rot_lever:
        out["reason"] = (f"rotation unobservable (lever {res['rot_lever']:.3f} m "
                         f"< {sp.min_rot_lever} m)")
    elif w < 180.0 and drift > sp.drift_bound_factor * w:
        out["reason"] = (f"implausible: result {drift:.0f} deg from the guess fraction, "
                         f"bound {sp.drift_bound_factor * w:.0f} deg")
    elif out_best is not None and out_best[0] >= res["inlier_frac"]:
        out["reason"] = (f"inconsistent with the guess: scans fit better at {out_best[1]:+.1f} deg "
                         f"({out_best[0]:.2f} vs {res['inlier_frac']:.2f} inliers), outside the "
                         f"+-{w:.0f} deg window")
    elif distinct is not None and distinct < sp.min_distinct_ratio:
        out["reason"] = (f"ambiguous: another solution {runner:.2f} vs {res['inlier_frac']:.2f} "
                         f"inliers (ratio < {sp.min_distinct_ratio})")
    else:
        out["ok"] = True
    return out


def match_sets(before: Sequence[np.ndarray], after: Sequence[np.ndarray], guess_deg: float,
               guess_t=None, sp: ScanMatchParams = None, guess_unc_deg=None,
               second_guess_deg=None) -> dict:
    """Pairwise match of two standstills' scan sets (point arrays, base frame).

    guess_deg: unwrapped yaw increment of the guess source (gyro or encoder);
    guess_unc_deg: its uncertainty (default guess_rel_err x |guess|);
    second_guess_deg: an independent increment (encoder beside the gyro) that
    must agree on the turn count; guess_t: translation guess in the 'before'
    base frame, metres.
    """
    sp = sp or ScanMatchParams()
    n = min(len(before), len(after))
    pairs = [match_pair(after[-n + i], before[-n + i], guess_deg, guess_t, sp, guess_unc_deg)
             for i in range(n)]
    good = [pr for pr in pairs if pr["ok"]]
    unc = guess_uncertainty_deg(guess_deg, sp, guess_unc_deg)
    w = coarse_window_deg(guess_deg, sp, guess_unc_deg)
    out = {"failed": True, "reason": "", "increment_deg": None, "frac_deg": None,
           "scatter_deg": None, "turns": None, "turns_ambiguous": unc >= 180.0,
           "turns_reason": "", "guess_deg": guess_deg, "guess_unc_deg": unc,
           "second_guess_deg": second_guess_deg,
           "guess_disagreement_deg": (None if second_guess_deg is None
                                      else guess_deg - second_guess_deg),
           "coarse_window_deg": min(w, 180.0),
           "tx": None, "ty": None, "inlier_frac": None, "rms": None, "rot_lever": None,
           "distinctness": None,
           "n_pairs": n, "n_ok": len(good), "pairs": pairs}
    need = max(1, math.ceil(sp.min_ok_frac * n))
    if n == 0:
        out["reason"] = "no scans on one side"
        return out
    if len(good) < need:
        reasons = sorted({pr["reason"] for pr in pairs if not pr["ok"]})
        out["reason"] = (f"{len(good)}/{n} scan pairs passed (need {need}): "
                         + "; ".join(reasons[:3]) + (" ..." if len(reasons) > 3 else ""))
        return out
    ref = good[0]["yaw_frac_deg"]
    fr = np.array([ref + wrap_deg(pr["yaw_frac_deg"] - ref) for pr in good])
    frac = wrap_deg(float(np.median(fr)))
    scatter = float(np.std(fr, ddof=1)) if len(fr) > 1 else 0.0
    inc, turns = resolve_turns(guess_deg, frac)
    out.update(frac_deg=frac, scatter_deg=scatter,
               tx=float(np.median([pr["tx"] for pr in good])),
               ty=float(np.median([pr["ty"] for pr in good])),
               inlier_frac=float(np.mean([pr["inlier_frac"] for pr in good])),
               rms=float(np.mean([pr["rms"] for pr in good])),
               rot_lever=float(np.min([pr["rot_lever"] for pr in good])))
    dist = [pr["distinctness"] for pr in good if pr["distinctness"] is not None]
    out["distinctness"] = float(np.min(dist)) if dist else None
    if scatter > sp.max_scatter_deg:
        out["reason"] = f"scatter {scatter:.3f} deg > {sp.max_scatter_deg} deg"
        return out
    why = []
    if out["turns_ambiguous"]:
        why.append(f"guess uncertainty {unc:.0f} deg >= 180 deg")
    if second_guess_deg is not None:
        inc2, turns2 = resolve_turns(second_guess_deg, frac)
        dis = out["guess_disagreement_deg"]
        if turns2 != turns:
            why.append(f"turn count {turns:+d} from the guess but {turns2:+d} from the second "
                       f"source ({second_guess_deg:+.1f} deg)")
        elif abs(dis) > sp.max_guess_disagreement_deg:
            why.append(f"guess and second source disagree by {dis:+.1f} deg "
                       f"(> {sp.max_guess_disagreement_deg} deg)")
    if why:
        out["turns_ambiguous"] = True
        out["turns_reason"] = "; ".join(why)
        out["reason"] = f"turn count unresolved ({out['turns_reason']}); fraction {frac:+.2f} deg"
        return out
    out.update(failed=False, increment_deg=inc, turns=turns)
    return out


def scan_set_points(scans: dict, idx: List[int], mount_r, mount_t, sp: ScanMatchParams):
    """Points of the scans idx of a BagData-style scan dict."""
    return [scan_points(scans["ranges"][i], scans["angle_min"][i], scans["angle_inc"][i],
                        scans["range_min"][i], scans["range_max"][i], mount_r, mount_t, sp)
            for i in idx]


def pick_scans(t: np.ndarray, t_start: float, t_end: float, sp: ScanMatchParams) -> List[int]:
    """Indices of the last `count` scans stamped inside [t_start + margin, t_end]."""
    idx = np.flatnonzero((t >= t_start + sp.margin_sec) & (t <= t_end))
    return [int(i) for i in idx[-sp.count:]] if sp.count > 0 else []


def guess_translation(wheel_odom: Optional[dict], ta: float, tb: float):
    """Encoder translation from ta to tb in the base frame at ta, or None."""
    if wheel_odom is None or "x" not in wheel_odom:
        return None
    t = np.asarray(wheel_odom["t"], dtype=float)
    order = np.argsort(t)
    t = t[order]
    x = np.asarray(wheel_odom["x"], dtype=float)[order]
    y = np.asarray(wheel_odom["y"], dtype=float)[order]
    yaw = np.unwrap(np.asarray(wheel_odom["yaw"], dtype=float)[order])
    xa, ya, xb, yb = (np.interp(v, t, a) for v, a in ((ta, x), (ta, y), (tb, x), (tb, y)))
    return _rot(-float(np.interp(ta, t, yaw))) @ np.array([xb - xa, yb - ya])
