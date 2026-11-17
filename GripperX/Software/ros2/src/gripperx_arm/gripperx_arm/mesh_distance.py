"""Exact distance between two triangle meshes, and a certificate that two arm links
cannot penetrate anywhere inside the joints' mechanical stops. OFFLINE only (used by
generate_acm; numpy + scipy, cost irrelevant there) - never per setpoint.

Distance: the minimum over triangle pairs of the exact triangle-triangle distance
(vertex-triangle and edge-edge distances; 0 when an edge pierces the other triangle).
Penetration also covers one mesh lying wholly inside the other (generalised winding
number of one vertex).

Certificate (branch and bound over the joints BETWEEN the two links, i.e. the only
joints that move them against each other): for a joint box with centre c and half
widths h, every point of link B moves relative to link A by at most sum_j h_j r_j
(r_j bounds the distance of B's mesh from joint j's axis in every configuration).
If the mesh distance at c exceeds that bound, the whole box is penetration-free;
a penetration at c settles the pair as NOT protected; otherwise the box is split,
down to a floor half-width. Boxes that reach the floor with a positive distance are
counted, not certified: that is the residual of the grid resolution (a feature
thinner than the motion inside a floor box could hide between samples).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

EPS = 1e-12


def read_stl_triangles(path) -> np.ndarray:
    import struct
    d = open(path, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(float)


# --------------------------------------------------------------------------- #
# vectorised primitives
# --------------------------------------------------------------------------- #
def _dot(a, b):
    return np.einsum('...i,...i->...', a, b)


def point_triangle_dist(p, a, b, c):
    """Distance from points p (m,3) to triangles (a,b,c) (m,3 each). Ericson,
    Real-Time Collision Detection 5.1.5, vectorised."""
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = _dot(ab, ap), _dot(ac, ap)
    bp = p - b
    d3, d4 = _dot(ab, bp), _dot(ac, bp)
    cp = p - c
    d5, d6 = _dot(ab, cp), _dot(ac, cp)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    out = np.empty_like(p)
    done = np.zeros(len(p), bool)

    def put(mask, val):
        m = mask & ~done
        out[m] = val[m]
        done[m] = True

    put((d1 <= 0) & (d2 <= 0), a)
    put((d3 >= 0) & (d4 <= d3), b)
    v = d1 / np.where(np.abs(d1 - d3) > EPS, d1 - d3, EPS)
    put((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + v[:, None] * ab)
    put((d6 >= 0) & (d5 <= d6), c)
    w = d2 / np.where(np.abs(d2 - d6) > EPS, d2 - d6, EPS)
    put((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + w[:, None] * ac)
    den = (d4 - d3) + (d5 - d6)
    w2 = (d4 - d3) / np.where(np.abs(den) > EPS, den, EPS)
    put((va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0), b + w2[:, None] * (c - b))
    s = va + vb + vc
    s = np.where(np.abs(s) > EPS, s, EPS)
    vv, ww = vb / s, vc / s
    put(np.ones(len(p), bool), a + ab * vv[:, None] + ac * ww[:, None])
    return np.linalg.norm(p - out, axis=1)


def segment_segment_dist(p1, q1, p2, q2):
    """Distance between segments p1q1 and p2q2 (m,3 each). Ericson 5.1.9."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    c, b = _dot(d1, r), _dot(d1, d2)
    denom = a * e - b * b
    s = np.where(denom > EPS, np.clip((b * f - c * e) / np.where(denom > EPS, denom, 1), 0, 1), 0)
    t = (b * s + f) / np.where(e > EPS, e, 1)
    t = np.where(e > EPS, t, 0)
    lo, hi = t < 0, t > 1
    t = np.clip(t, 0, 1)
    s = np.where(lo, np.clip(-c / np.where(a > EPS, a, 1), 0, 1), s)
    s = np.where(hi, np.clip((b - c) / np.where(a > EPS, a, 1), 0, 1), s)
    s = np.where(a > EPS, s, 0)
    c1 = p1 + d1 * s[:, None]
    c2 = p2 + d2 * t[:, None]
    return np.linalg.norm(c1 - c2, axis=1)


def segment_pierces_triangle(p, q, a, b, c):
    """Möller-Trumbore for segments p->q against triangles; True where they meet."""
    d = q - p
    e1, e2 = b - a, c - a
    h = np.cross(d, e2)
    det = _dot(e1, h)
    ok = np.abs(det) > EPS
    inv = 1.0 / np.where(ok, det, 1)
    s = p - a
    u = inv * _dot(s, h)
    qv = np.cross(s, e1)
    v = inv * _dot(d, qv)
    t = inv * _dot(e2, qv)
    return ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t >= 0) & (t <= 1)


def triangle_pairs_dist(P, Q):
    """Exact distances between triangle pairs P[i], Q[i] (m,3,3)."""
    if len(P) == 0:
        return np.zeros(0)
    ds = []
    for i in range(3):
        ds.append(point_triangle_dist(P[:, i], Q[:, 0], Q[:, 1], Q[:, 2]))
        ds.append(point_triangle_dist(Q[:, i], P[:, 0], P[:, 1], P[:, 2]))
    hit = np.zeros(len(P), bool)
    for i in range(3):
        j = (i + 1) % 3
        hit |= segment_pierces_triangle(P[:, i], P[:, j], Q[:, 0], Q[:, 1], Q[:, 2])
        hit |= segment_pierces_triangle(Q[:, i], Q[:, j], P[:, 0], P[:, 1], P[:, 2])
        for k in range(3):
            m = (k + 1) % 3
            ds.append(segment_segment_dist(P[:, i], P[:, j], Q[:, k], Q[:, m]))
    d = np.min(np.vstack(ds), axis=0)
    return np.where(hit, 0.0, d)


def winding_number(point, tris) -> float:
    """Generalised winding number of a point w.r.t. a triangle soup (~1 inside a
    closed mesh, ~0 outside)."""
    a, b, c = (tris[:, i] - point for i in range(3))
    la, lb, lc = (np.linalg.norm(x, axis=1) for x in (a, b, c))
    num = _dot(a, np.cross(b, c))
    den = la * lb * lc + _dot(a, b) * lc + _dot(b, c) * la + _dot(c, a) * lb
    return float(np.sum(2.0 * np.arctan2(num, den)) / (4.0 * math.pi))


def refine_triangles(tris: np.ndarray, max_radius: float) -> np.ndarray:
    """Split triangles (longest-edge bisection) until every triangle lies within
    max_radius of its centroid. Exact: the pieces cover the same surface. CAD STLs
    carry long slivers (radius up to ~40 mm) that would make every proximity query
    consider thousands of pairs."""
    out, work = [], tris
    while len(work):
        c = work.mean(axis=1)
        r = np.linalg.norm(work - c[:, None], axis=2).max(axis=1)
        ok = r <= max_radius
        out.append(work[ok])
        big = work[~ok]
        if not len(big):
            break
        e = np.stack([np.linalg.norm(big[:, (i + 1) % 3] - big[:, i], axis=1)
                      for i in range(3)], axis=1)
        k = np.argmax(e, axis=1)
        a = big[np.arange(len(big)), k]
        b = big[np.arange(len(big)), (k + 1) % 3]
        o = big[np.arange(len(big)), (k + 2) % 3]
        m = (a + b) / 2.0
        work = np.concatenate([np.stack([a, m, o], axis=1), np.stack([m, b, o], axis=1)])
    return np.concatenate(out)


# --------------------------------------------------------------------------- #
class MeshPair:
    """Distance between mesh A (fixed, its own frame) and mesh B posed by T (B in
    A's frame)."""

    def __init__(self, tris_a: np.ndarray, tris_b: np.ndarray, max_radius: float = 0.002):
        from scipy.spatial import cKDTree
        self.va0 = np.unique(tris_a.reshape(-1, 3), axis=0)     # unrefined, for cheap proxies
        self.vb0 = np.unique(tris_b.reshape(-1, 3), axis=0)
        tris_a = refine_triangles(tris_a, max_radius)
        tris_b = refine_triangles(tris_b, max_radius)
        self.A, self.B = tris_a, tris_b
        self.ca = tris_a.mean(axis=1)
        self.cb = tris_b.mean(axis=1)
        self.ra = float(np.linalg.norm(tris_a - self.ca[:, None], axis=2).max())
        self.rb = float(np.linalg.norm(tris_b - self.cb[:, None], axis=2).max())
        self.va = np.unique(tris_a.reshape(-1, 3), axis=0)
        self.vb = np.unique(tris_b.reshape(-1, 3), axis=0)
        self.tree_c = cKDTree(self.ca)
        self.tree_v = cKDTree(self.va)

    def distance(self, T: np.ndarray, upto: float = math.inf) -> float:
        """Exact distance between the two SURFACES if it is below `upto`, else some
        value >= `upto`; 0 = the surfaces touch or cross. Containment: contained()."""
        R, t = T[:3, :3], T[:3, 3]
        vb = self.vb @ R.T + t
        dvv = float(self.tree_v.query(vb)[0].min())         # >= true distance
        lower = dvv - 2 * self.ra - 2 * self.rb               # every surface point is within
        if lower >= upto:                                     # 2r of a vertex of its triangle
            return lower
        reach = min(upto, dvv) + self.ra + self.rb
        cb = self.cb @ R.T + t
        lists = self.tree_c.query_ball_point(cb, reach)
        ib = np.repeat(np.arange(len(cb)), [len(x) for x in lists])
        if len(ib) == 0:
            return dvv
        ia = np.concatenate([np.asarray(x, int) for x in lists if x])
        tb = self.B @ R.T + t
        # nearest candidate pairs first; stop once no remaining pair can beat `best`
        cd = np.linalg.norm(self.ca[ia] - cb[ib], axis=1)
        order = np.argsort(cd)
        ia, ib, cd = ia[order], ib[order], cd[order]
        best = math.inf
        for s in range(0, len(ia), 20000):
            if cd[s] - self.ra - self.rb > min(best, upto):
                break
            d = triangle_pairs_dist(self.A[ia[s:s + 20000]], tb[ib[s:s + 20000]])
            best = min(best, float(d.min()))
            if best <= 0.0:
                return 0.0
        return best

    def contained(self, T: np.ndarray) -> bool:
        """One mesh wholly inside the other (no surface contact needed)."""
        R, t = T[:3, :3], T[:3, 3]
        return (winding_number(self.vb[0] @ R.T + t, self.A) > 0.5
                or winding_number(self.va[0], self.B @ R.T + t) > 0.5)


@dataclass
class Certificate:
    protected: bool
    reason: str
    min_distance: float = math.inf        # smallest exact distance evaluated (m)
    evaluations: int = 0
    floor_boxes: int = 0
    floor_rad: float = 0.0
    witness: Optional[List[float]] = None  # joint values of a penetration


def certify(rel_pose: Callable[[np.ndarray], np.ndarray], pair: MeshPair,
            lower: Sequence[float], upper: Sequence[float], radii: Sequence[float],
            floor_rad: float = 0.002, budget: int = 200000,
            time_limit_s: float = math.inf) -> Certificate:
    import time
    t_end = time.monotonic() + time_limit_s
    lo0, hi0 = np.array(lower, float), np.array(upper, float)
    r = np.array(radii, float)
    stack = [(lo0, hi0)]
    cert = Certificate(True, '', floor_rad=floor_rad)
    # Containment can only change where the surfaces meet; every certified box is
    # surface-free and connected to the root, so one check at the root covers them.
    if pair.contained(rel_pose((lo0 + hi0) / 2.0)):
        cert.protected, cert.reason = False, 'one mesh lies inside the other'
        cert.witness = [round(float(v), 5) for v in (lo0 + hi0) / 2.0]
        return cert
    while stack:
        lo, hi = stack.pop()
        c, h = (lo + hi) / 2.0, (hi - lo) / 2.0
        bound = float(h @ r)
        d = pair.distance(rel_pose(c), upto=bound + 1e-9)
        cert.evaluations += 1
        if d < bound:
            cert.min_distance = min(cert.min_distance, d)
        if d <= 0.0:
            cert.protected = False
            cert.reason = 'meshes penetrate inside the mechanical stops'
            cert.witness = [round(float(v), 5) for v in c]
            cert.min_distance = min(cert.min_distance, d)
            return cert
        if d > bound:
            continue
        if h.max() <= floor_rad:
            cert.floor_boxes += 1
            continue
        if cert.evaluations >= budget or time.monotonic() > t_end:
            cert.protected = False
            cert.reason = (f'not settled within the budget ({cert.evaluations} evaluations) '
                           '- stays checked')
            return cert
        k = int(np.argmax(h * r)) if (h * r).max() > 0 else int(np.argmax(h))
        mid = c[k]
        hi1, lo2 = hi.copy(), lo.copy()
        hi1[k], lo2[k] = mid, mid
        stack.append((lo, hi1))
        stack.append((lo2, hi))
    cert.reason = 'no penetration inside the mechanical stops'
    return cert
