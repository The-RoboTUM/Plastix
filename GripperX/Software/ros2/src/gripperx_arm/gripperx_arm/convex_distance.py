"""Signed distance between two convex hulls of point sets, with certified bounds.
Pure numpy (+ scipy as fallback), no ROS.

signed_distance(pa, pb) -> (lo, hi): lo <= d <= hi, where d is the gap between
conv(pa) and conv(pb) when they are apart, and minus their penetration depth (the
shortest translation that separates them) when they overlap. Both bounds come from
the algorithm's own certificates, not from an error estimate:

  * apart (GJK on the Minkowski difference M = A - B): hi = |v| for v, a point of M
    (the closest point of the current simplex); lo = max over iterations of
    (v . w) / |v| with w the support point of M toward -v - every point of M lies
    at least that far from the origin along v.
  * overlapping (EPA from a tetrahedron of M around the origin): the expanding
    polytope P lies inside M, so the penetration depth is >= the distance of P's
    nearest face (-> hi); for that face's normal n, the depth is <= h_M(n), the
    support value of M (-> lo).

The search stops when hi - lo <= tol. A case the iteration does not settle (a
degenerate simplex, the origin on the boundary, the iteration cap) falls back to the
exact facet planes of scipy's hull of all of M (slower; without scipy the bounds are
(-inf, inf), which every caller treats as a failure).
"""
from __future__ import annotations

import itertools
from typing import List, Tuple

import numpy as np


def _support(pa: np.ndarray, pb: np.ndarray, d: np.ndarray) -> np.ndarray:
    """The point of M = A - B farthest along d."""
    return pa[int(np.argmax(pa @ d))] - pb[int(np.argmin(pb @ d))]


def _closest_on_simplex(w: List[np.ndarray]) -> Tuple[np.ndarray, List[np.ndarray]]:
    """Closest point of conv(w) (1..4 points) to the origin and the smallest face of w
    that holds it: the minimum-norm projection onto a face's affine hull whose
    barycentric coordinates are all positive (the minimiser lies in the relative
    interior of exactly such a face)."""
    best, best_face = None, None
    for r in range(1, len(w) + 1):
        for face in itertools.combinations(range(len(w)), r):
            p = np.array([w[i] for i in face])
            if r == 1:
                x, lam = p[0], np.ones(1)
            else:
                e = p[1:] - p[0]
                g = e @ e.T
                try:
                    mu = np.linalg.solve(g, -e @ p[0])
                except np.linalg.LinAlgError:
                    continue
                lam = np.concatenate([[1.0 - mu.sum()], mu])
                x = p[0] + mu @ e
            if np.all(lam > 0.0) and (best is None or x @ x < best @ best):
                best, best_face = x, [w[i] for i in face]
    return best, best_face


def _hull_fallback(pa: np.ndarray, pb: np.ndarray) -> Tuple[float, float]:
    try:
        from scipy.spatial import ConvexHull
    except ImportError:
        return -np.inf, np.inf           # undecided: every condition on it fails closed
    m = (pa[:, None, :] - pb[None, :, :]).reshape(-1, 3)
    c = float(ConvexHull(m).equations[:, 3].max())       # n . x + c <= 0 inside
    if c <= 0.0:
        return c, c                     # inside: exactly minus the penetration depth
    # outside: c is the gap along the best facet normal (a lower bound); an upper bound
    # is the nearest point of M itself
    return c, float(np.sqrt((m * m).sum(axis=1).min()))


def _epa(pa, pb, simplex, tol, max_iter) -> Tuple[float, float]:
    pts = [np.asarray(p, float) for p in simplex]
    faces = []
    centre = np.mean(pts, axis=0)
    for f in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)):
        a, b, c = (pts[i] for i in f)
        n = np.cross(b - a, c - a)
        faces.append(f if n @ (a - centre) > 0 else (f[0], f[2], f[1]))
    for _ in range(max_iter):
        best = None
        for f in faces:
            a, b, c = (pts[i] for i in f)
            n = np.cross(b - a, c - a)
            ln = np.linalg.norm(n)
            if ln < 1e-15:
                return _hull_fallback(pa, pb)
            n /= ln
            dist = float(n @ a)
            if dist < -1e-12:
                return _hull_fallback(pa, pb)       # origin not inside: degenerate
            if best is None or dist < best[0]:
                best = (dist, n, f)
        dist, n, _f = best
        s = _support(pa, pb, n)
        h = float(n @ s)
        if h - dist <= tol:
            return -h, -dist
        pts.append(s)
        k = len(pts) - 1
        visible = [f for f in faces
                   if np.cross(pts[f[1]] - pts[f[0]], pts[f[2]] - pts[f[0]]) @ (s - pts[f[0]]) > 0]
        edges = {}
        for f in visible:
            for e in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
                if (e[1], e[0]) in edges:
                    del edges[(e[1], e[0])]
                else:
                    edges[e] = True
        faces = [f for f in faces if f not in visible] + [(e[0], e[1], k) for e in edges]
    return _hull_fallback(pa, pb)


def signed_distance(pa: np.ndarray, pb: np.ndarray, tol: float = 1e-7,
                    max_iter: int = 200) -> Tuple[float, float]:
    """(lo, hi) bounds of the signed distance of conv(pa), conv(pb) (module docstring)."""
    pa, pb = np.asarray(pa, float), np.asarray(pb, float)
    v = pa[0] - pb[0]
    w_set: List[np.ndarray] = []
    lo = -np.inf
    for _ in range(max_iter):
        nv = float(np.linalg.norm(v))
        if nv < 1e-12:
            break                               # origin on/in M: penetration
        w = _support(pa, pb, -v)
        lo = max(lo, float(v @ w) / nv)
        if nv - lo <= tol:
            return lo, nv
        w_set.append(w)
        v, w_set = _closest_on_simplex(w_set)
        if v is None:
            return _hull_fallback(pa, pb)
        if len(w_set) == 4:
            break                               # the origin is inside the tetrahedron
    else:
        return _hull_fallback(pa, pb)
    if len(w_set) < 4:
        return _hull_fallback(pa, pb)
    return _epa(pa, pb, w_set, tol, max_iter)


def clearance_at_least(pa: np.ndarray, pb: np.ndarray, need: float,
                       max_iter: int = 200) -> Tuple[bool, float]:
    """(decided, value): is the gap between conv(pa) and conv(pb) >= need (need >= 0)?
    GJK only, no EPA - the penetration depth is never needed for that. Stops as soon
    as a certificate settles it: the lower bound (v . w) / |v| >= need -> (True, lo);
    the upper bound |v| < need -> (False, hi); the origin enclosed (overlap) ->
    (False, 0.0). An unsettled run falls back to the exact facet planes (signed_distance's
    fallback) and compares its lower bound."""
    pa, pb = np.asarray(pa, float), np.asarray(pb, float)
    v = pa[0] - pb[0]
    w_set: List[np.ndarray] = []
    lo = -np.inf
    for _ in range(max_iter):
        nv = float(np.linalg.norm(v))
        if nv < 1e-12:
            return False, 0.0                   # origin on/in M: touching or overlapping
        if nv < need:
            return False, nv
        w = _support(pa, pb, -v)
        lo = max(lo, float(v @ w) / nv)
        if lo >= need:
            return True, lo
        if nv - lo <= 1e-7:
            return False, nv                    # converged within 0.1 um of need: refuse
        w_set.append(w)
        v, w_set = _closest_on_simplex(w_set)
        if v is None:
            break
        if len(w_set) == 4:
            return False, 0.0                   # origin inside the tetrahedron
    lo, hi = _hull_fallback(pa, pb)
    return (lo >= need), (lo if lo >= need else hi)
