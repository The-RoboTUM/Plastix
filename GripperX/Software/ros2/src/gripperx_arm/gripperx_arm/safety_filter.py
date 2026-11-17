"""SR-17 layer L3: the arm safety filter. Pure Python + numpy, no ROS imports.

Every arm setpoint the arm node writes passes `SafetyFilter.check_path()` first:
fixed sequence ramp steps, go_home, open/close and the leader stream. A
rejected path writes nothing; the caller holds (SR-17 rule 6). The filter never
moves a setpoint to a nearby valid one.

What a check covers, all computed from the robot description handed in (never
from constants in this file):
  * joint limits  - the URDF <limit> of every arm joint (from geometry.yaml);
                    in recovery (check_setpoints) a start outside them is accepted
                    while every later setpoint only moves back toward the limit;
  * ground        - every arm collision box, the jaws included, and the TCP stay
                    >= the configured clearance above z = 0 of base_footprint
                    (D1: 1 cm for the tip; other links per OP-42 (d) proposal);
                    for the operator's go_home a start failing it - also one
                    at or below the ground - is accepted while the path first
                    lifts it out, one joint at a time (check_setpoints rise_ground);
  * robot body    - arm boxes vs. chassis and the four wheel/steer modules;
                    the wheels as MEASURED (when the caller passes fresh steering
                    angles) or swept over the MECHANICAL steering stops. With
                    body_hull_stage, an overlap of a (grown) arm box with a body box
                    rigidly fixed to base_link (chassis, arm pedestal, lidar when
                    fitted) is confirmed or cleared on the convex hull of the arm
                    link's DECIMATED visual mesh - not the CAD part - against that
                    unchanged body box, with the hull's own motion bound and its own
                    clearance body_hull_clearance_m (the arm boxes' 2 mm padding
                    against CAD and decimation error no longer applies there; the
                    parameter carries that margin, TO-VERIFY). Wheels, steering
                    modules and the ground keep the box verdict;
  * self          - arm boxes vs. arm boxes, except allowed pairs from the
                    generated allowed-collision list: scope "always" everywhere,
                    scope "home_neighbourhood" only while every joint that
                    moves the two links AGAINST EACH OTHER lies inside that
                    neighbourhood, in the checked pose and both its neighbours.

The motion BETWEEN two checked setpoints: each joint moves monotonically
between them (ramp, or the servo's own position loop), so the joint vector stays
inside the box spanned by the two end points. For every arm link a bound D on
how far any of its box points can travel inside that box is computed from the
model: the sum over the joints above it of |dq_j| * r_j, where r_j bounds the
distance of the link's boxes from joint j's axis over the WHOLE step - the
distance in the checked end pose plus sum_k |dq_k| * R_k over the joints k
between j and the link (how far their motion can carry the boxes relative to
j's axis; R_k = the configuration-independent reach from k's axis, _link_radii).
Joints above j move the boxes and j's axis rigidly together and change nothing.
The R_k term is second order in the step (see SafetyFilter._growth).
Every point of the swept motion is within D/2 of one of the two end poses, so
each end pose is checked with every box grown by D/2 per side. That makes the
guarantee independent of `max_joint_step_rad`, which only trades conservatism
(larger step = larger growth) for checks per path.
NOT covered: servo overshoot beyond the commanded end point, and the servo
lagging so far behind that it is outside the box of the current step (it then
trails inside the box of an earlier, already checked step).
"""
from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Collection, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from gripperx_arm.kinematics import (
    ArmKinematics, Primitive, as_box, boxes_overlap, hulls_overlap, read_stl_vertices, wrap)

TCP = 'TCP'          # the TCP's entry among the ground heights (ground_heights)
_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
                    dtype=float)


@dataclass
class FilterConfig:
    """All values come from config/arm_safety_filter.yaml (ROS parameters)."""
    ground_clearance_tip_m: float          # D1: 0.01
    ground_clearance_links_m: float        # OP-42 (d) proposal, TO-VERIFY
    tip_links: Sequence[str]               # links the tip clearance applies to
    body_clearance_m: float                # OP-42 (d), TO-VERIFY
    self_clearance_m: float                # OP-42 (d), TO-VERIFY
    max_joint_step_rad: float              # subdivision of a path, TO-VERIFY
    # check_setpoints(separate_self=True): how far a separating self pair's clearance
    # may dip below its start value between two setpoints. 0 = separation from contact
    # is impossible (fail-closed default). TO-VERIFY.
    self_contact_dip_m: float = 0.0
    # Self check, second stage: an arm box overlap confirmed on the hulls (needs hulls).
    mesh_hull_stage: bool = True
    # Body check, second stage (_body_hull_check): off = the box verdict is final.
    body_hull_stage: bool = False
    # Clearance the hull stage holds an arm hull to; unset = the stage never clears.
    body_hull_clearance_m: float = math.inf

    @classmethod
    def from_params(cls, get: Callable[[str], object]) -> 'FilterConfig':
        """The one construction from the `safety_filter.*` values: `get(name)` returns
        the value of `safety_filter.<name>` (node parameter or yaml mapping). Every field
        is read - a field's default never stands in for a configured value."""
        return cls(
            ground_clearance_tip_m=float(get('ground_clearance_tip_m')),
            ground_clearance_links_m=float(get('ground_clearance_links_m')),
            tip_links=list(get('tip_links')),
            body_clearance_m=float(get('body_clearance_m')),
            self_clearance_m=float(get('self_clearance_m')),
            max_joint_step_rad=float(get('max_joint_step_rad')),
            self_contact_dip_m=float(get('self_contact_dip_m')),
            mesh_hull_stage=bool(get('mesh_hull_stage')),
            body_hull_stage=bool(get('body_hull_stage')),
            body_hull_clearance_m=float(get('body_hull_clearance_m')))


@dataclass
class Verdict:
    ok: bool
    rule: str = ''              # 'joint_limit' | 'ground' | 'body' | 'self' | 'data' | 'timeout'
    detail: str = ''
    index: int = -1             # which sub-step of a path failed (-1 = n/a)
    checked: int = 0            # number of poses checked
    # check_setpoints rise_ground/rising: {link or TCP: (static, grown height)} that
    # triggered the ground recovery at poses[0], and the entries still rising at the end
    rising_start: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    rising_end: Tuple[str, ...] = ()
    # A 'timeout' verdict: check_setpoints(resume_from=) continues here; 0 = from the start
    resume_at: int = 0

    def __str__(self) -> str:
        if self.ok:
            return f'ok ({self.checked} poses)'
        where = f' at sub-step {self.index}' if self.index >= 0 else ''
        return f'REJECTED [{self.rule}]{where}: {self.detail}'


@dataclass
class Envelope:
    """Body primitives the arm is checked against, already resolved for the
    current steering knowledge. `groups` = list of (bound or None, samples)."""
    groups: List[Tuple[Optional[Primitive], List[Primitive]]]
    description: str
    # envelope_measured only: the readings it was built from (None = wheel unknown,
    # swept). Lets a caller tell whether a new envelope can free space an old one held.
    steer: Optional[Dict[str, Optional[float]]] = None


@dataclass
class _Allowed:
    always: set = field(default_factory=set)
    near_home: set = field(default_factory=set)
    home_range: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    stop_protected: set = field(default_factory=set)


def load_acm(doc: Mapping, stop_doc: Optional[Mapping] = None) -> _Allowed:
    """Allowed pairs from the parsed arm_allowed_collisions.yaml, plus the pairs
    arm_stop_protected.yaml certifies as kept apart by the mechanical stops (never
    checked - user decision 2026-10-07). Only category `stop_protected` counts."""
    out = _Allowed()
    for p in (stop_doc or {}).get('pairs', []):
        if p.get('category') == 'stop_protected':
            key = tuple(sorted(p['links']))
            out.stop_protected.add(key)
            out.always.add(key)
    for p in doc.get('allowed_pairs', []):
        key = tuple(sorted(p['links']))
        scope = p.get('scope')
        if scope == 'always':
            out.always.add(key)
        elif scope == 'home_neighbourhood':
            out.near_home.add(key)
        else:
            raise ValueError(f'allowed pair {key}: unknown scope {scope!r}')
    rng = doc.get('neighbourhood', {}).get('joint_range_rad', {})
    out.home_range = {k: (float(v[0]), float(v[1])) for k, v in rng.items()}
    if out.near_home and not out.home_range:
        raise ValueError('home_neighbourhood pairs without neighbourhood.joint_range_rad')
    return out


def _env_spheres(env: 'Envelope'):
    """Per envelope group: (centres (n, 3), radii (n,)) of the boxes' bounding
    spheres, cached on the envelope (it is immutable once built)."""
    cached = getattr(env, '_spheres', None)
    if cached is None:
        cached = []
        for _bound, samples in env.groups:
            fh = [as_box(p) for p in samples]
            cached.append((np.array([f[:3, 3] for f, _h in fh]).reshape(-1, 3),
                           np.array([float(np.linalg.norm(h)) for _f, h in fh])))
        env._spheres = cached
    return cached


def load_link_hulls(urdf_xml: str, kin: ArmKinematics, resolve) -> Dict[str, np.ndarray]:
    """Per arm link with a visual mesh: its unique vertices in the link frame (their
    convex hull encloses the mesh). `resolve` maps the URDF mesh filename to a path.
    The visual origin is applied."""
    import xml.etree.ElementTree as ET
    from gripperx_arm.kinematics import _origin
    out = {}
    for link in ET.fromstring(urdf_xml).findall('link'):
        name = link.get('name')
        mesh = link.find('visual/geometry/mesh')
        if name not in kin.links or mesh is None:
            continue
        v = read_stl_vertices(resolve(mesh.get('filename')))
        s = [float(x) for x in (mesh.get('scale') or '1 1 1').split()]
        t = _origin(link.find('visual/origin'))
        out[name] = (v * s) @ t[:3, :3].T + t[:3, 3]
    return out


def mechanical_steer_ranges(kin: ArmKinematics, steer_joint_names: Sequence[str],
                            outward_sign: Sequence[int], outward_deg: Sequence[float],
                            inward_deg: Sequence[float]) -> Dict[str, Tuple[float, float]]:
    """Per steering joint, the URDF-angle interval between its two MECHANICAL stops.
    outward_sign[i] = sign of the joint angle that is physically outward (measured,
    steer_servo.yaml)."""
    if not (len(steer_joint_names) == len(outward_sign) == len(outward_deg) == len(inward_deg)):
        raise ValueError('steering arrays differ in length')
    env_joints = {j.name for j in kin._env_order if j.type == 'revolute'}
    out = {}
    for n, s, o, i in zip(steer_joint_names, outward_sign, outward_deg, inward_deg):
        if n not in env_joints:
            raise ValueError(f'steering joint {n} not in the robot description')
        if s not in (1, -1):
            raise ValueError(f'{n}: outward sign must be +-1')
        a, b = s * math.radians(o), -s * math.radians(i)
        out[n] = (min(a, b), max(a, b))
    missing = env_joints - set(out)
    if missing:
        raise ValueError(f'revolute body joints without a mechanical range: {sorted(missing)}')
    return out


def unwrap_steer_reading(reading: float, outward_sign: int, outward_rad: float,
                         inward_rad: float, tol: float) -> Optional[float]:
    """steer_servo_node reports angles in (-pi, pi]; the outward mechanical stop
    lies beyond pi, so an outward angle past pi is reported wrapped onto the
    inward side. The physical span (outward + inward) is < 2 pi, so the reading is
    unambiguous: returns the physical joint angle, or None if no physical angle
    within the stops (+- tol) matches it."""
    r = outward_sign * reading                    # outward-positive
    if r < -(inward_rad + tol):
        r += 2.0 * math.pi
    if not -(inward_rad + tol) <= r <= outward_rad + tol:
        return None
    return outward_sign * r


class SafetyFilter:
    def __init__(self, kin: ArmKinematics, acm: _Allowed, cfg: FilterConfig,
                 mechanical_ranges: Mapping[str, Tuple[float, float]],
                 ground_frame: str = 'base_footprint',
                 swept_step_rad: float = math.radians(10.0),
                 hulls: Optional[Mapping[str, np.ndarray]] = None):
        """hulls: per arm link, points (link frame) whose convex hull encloses the
        link's mesh (load_link_hulls) - used by the self check's second stage
        (cfg.mesh_hull_stage) and the body check's (cfg.body_hull_stage). Without them,
        or with a stage off, a box overlap is final for that check."""
        self.hulls = dict(hulls or {})
        if not kin.joint_names:
            raise ValueError('robot description has no articulated arm joints')
        if not kin._collision:
            raise ValueError('robot description has no arm collision geometry')
        for n, (lo, hi) in kin.limits.items():
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
                raise ValueError(f'joint {n}: no usable limit ({lo}, {hi})')
        if cfg.max_joint_step_rad <= 0.0:
            raise ValueError('max_joint_step_rad must be > 0')
        if cfg.body_hull_stage and not cfg.body_hull_clearance_m >= 0.0:
            raise ValueError('body_hull_clearance_m must be >= 0')
        for link in cfg.tip_links:
            if link not in kin.links:
                raise ValueError(f'tip link {link} is not an arm link')
        self.kin, self.acm, self.cfg = kin, acm, cfg
        self._ground = self._ground_plane(ground_frame)
        self.mechanical_ranges = dict(mechanical_ranges)
        self._swept_step = swept_step_rad
        self.envelope_swept = self._envelope(kin.swept_environment(swept_step_rad,
                                                                   self.mechanical_ranges),
                                             'wheels swept over the mechanical steering stops')
        self._radius = self._link_radii()
        self._tcp_radius = self._chain_radii(kin.tcp_link, 0.0)
        # Body links rigidly fixed to base_link (the chassis and its fixed mounts): the
        # only body a box overlap is confirmed on hulls against (_body_hull_check).
        # Everything a steering joint moves (wheels, steer modules) keeps the box
        # verdict as final.
        fixed = {kin.base_link}
        for j in kin._env_order:          # breadth first: parents before children
            if j.type == 'fixed' and j.parent in fixed:
                fixed.add(j.child)
        self._rigid_body = frozenset(fixed)
        # Per arm link with a hull: _chain_radii for a reach covering every hull
        # vertex (the box reach of _link_radii covers the boxes, not the hull).
        self._hull_radius = {
            link: self._chain_radii(link, float(np.linalg.norm(self._hull_vertices(link),
                                                               axis=1).max()))
            for link in self.hulls if link in self._radius}

    # ------------------------------------------------------------------ setup
    def _ground_plane(self, frame: str) -> Tuple[np.ndarray, float]:
        """(unit normal, offset) of z = 0 of `frame`, expressed in base_link:
        height(p) = n . p + c. `frame` must be a fixed-joint parent of base_link."""
        t = np.eye(4)
        link = self.kin.base_link
        while link != frame:
            j = self.kin._by_child.get(link)
            if j is None or j.type != 'fixed':
                raise ValueError(f'{frame} is not a fixed ancestor of {self.kin.base_link}')
            t = j.origin @ t
            link = j.parent
        # t: base_link -> frame. height = (t @ p).z
        return t[2, :3].copy(), float(t[2, 3])

    def _link_radii(self) -> Dict[str, Dict[str, float]]:
        """R[link][joint]: for every actuated joint above `link`, an upper bound on
        the distance of any point of link's boxes from that joint's axis, valid in
        every configuration (triangle inequality along the chain)."""
        kin = self.kin
        out: Dict[str, Dict[str, float]] = {}
        for link, prims in kin._collision.items():
            reach = 0.0
            for _shape, t, size in prims:
                h = as_box(Primitive(link, _shape, t, size))[1]
                reach = max(reach, float(np.linalg.norm(t[:3, 3]) + np.linalg.norm(h)))
            out[link] = self._chain_radii(link, reach)
        return out

    def _chain_radii(self, link: str, reach: float) -> Dict[str, float]:
        """{actuated joint above `link`: bound on the distance from its axis of any point
        within `reach` of link's origin}, nearest joint first, every configuration."""
        kin = self.kin
        chain = []                         # joints from link up to arm_root
        cur = link
        while cur != kin.arm_root:
            j = kin._by_child[cur]
            chain.append(j)
            cur = j.parent
        radii, acc = {}, reach
        for j in chain:                    # nearest joint first
            if j.name in kin.limits:
                radii[j.name] = acc
            acc += float(np.linalg.norm(j.origin[:3, 3]))
        return radii

    def _envelope(self, groups, description: str, steer=None) -> Envelope:
        return Envelope([(g.bound, g.samples) for g in groups], description, steer)

    def envelope_measured(self, steer: Mapping[str, Optional[float]],
                          margin_rad: float) -> Envelope:
        """Wheels at the given measured angles +- margin, clipped to the mechanical
        stops. A wheel whose entry is None (unknown) is swept over its full
        mechanical range."""
        ranges = {}
        for n, (lo, hi) in self.mechanical_ranges.items():
            v = steer.get(n)
            ranges[n] = (lo, hi) if v is None else (max(lo, v - margin_rad),
                                                    min(hi, v + margin_rad))
            if ranges[n][0] > ranges[n][1]:
                ranges[n] = (lo, hi)
        step = max(1e-3, min(self._swept_step, 2.0 * margin_rad if margin_rad > 0 else 1e-3))
        known = sorted(k for k, v in steer.items() if v is not None)
        return self._envelope(self.kin.swept_environment(step, ranges),
                              f'wheels measured (+-{math.degrees(margin_rad):.1f} deg): '
                              f'{known}; others swept',
                              {n: (None if v is None else float(v)) for n, v in steer.items()})

    # ------------------------------------------------------------------ checks
    def _q(self, q) -> Dict[str, float]:
        qd = q if isinstance(q, Mapping) else dict(zip(self.kin.joint_names, q))
        missing = [n for n in self.kin.joint_names if n not in qd]
        if missing:
            raise KeyError(f'missing joint values: {missing}')
        return {n: float(qd[n]) for n in self.kin.joint_names}

    def _pair_joints(self, a: str, b: str) -> frozenset:
        """Joints that change the RELATIVE pose of links a and b: those above one of
        them but not above both (a body link has none of its own)."""
        ja = frozenset(self._radius.get(a, {}))
        jb = frozenset(self._radius.get(b, {}))
        return ja ^ jb

    def _pair_near_home(self, a: str, b: str, poses) -> bool:
        """A home_neighbourhood pair is allowed only while every joint that moves the
        two links against each other lies inside the generated neighbourhood, in
        every pose of `poses`. Joints that move both links together cannot change
        their relative pose, so they do not take the pair out of the neighbourhood."""
        rng = self.acm.home_range
        joints = self._pair_joints(a, b)
        return all(n in rng and rng[n][0] <= q[n] <= rng[n][1] for q in poses for n in joints)

    def _self_hulls(self, la: str, lb: str) -> bool:
        return self.cfg.mesh_hull_stage and la in self.hulls and lb in self.hulls

    def _hulls_hit(self, la: str, lb: str, frames, margin: float, cache) -> bool:
        if not self._self_hulls(la, lb):
            return True
        key = (la, lb)
        if key not in cache:
            fa, fb = frames[la], frames[lb]
            pa = self.hulls[la] @ fa[:3, :3].T + fa[:3, 3]
            pb = self.hulls[lb] @ fb[:3, :3].T + fb[:3, 3]
            cache[key] = hulls_overlap(pa, pb, margin)
        return cache[key]

    def _growth(self, frames, boxes: Mapping[str, List[Tuple[np.ndarray, np.ndarray]]],
                dq: Mapping[str, float]):
        """(per-link radius bounds rb[link][joint], absolute half growth per link)
        for a motion of |dq| per joint around the pose given by `frames`.

        rb[link][j] is NOT configuration-independent: it is the largest distance of
        the link's box corners from j's axis IN THIS POSE (the maximum over a box is
        taken at a corner), plus sum_k |dq_k| * R_k over the joints k nearer the
        link than j (R_k from _link_radii, valid in every configuration): the most
        their motion during the step can move a box point relative to j's axis. So
        rb bounds the distance over the whole step, not only at this end, and the
        growth 0.5 * sum_j |dq_j| * rb_j bounds half the path length of any box
        point. The R_k term is second order in the step (|dq_j| * |dq_k| * R_k);
        without it the per-pose radius alone would under-estimate the growth by
        that amount."""
        kin = self.kin
        rb: Dict[str, Dict[str, float]] = {}
        grow: Dict[str, float] = {}
        for link, glob in self._radius.items():
            pts = np.vstack([(_CORNERS * h) @ f[:3, :3].T + f[:3, 3] for f, h in boxes[link]])
            names = list(glob)                 # nearest joint first
            rb[link] = {}
            for idx, jn in enumerate(names):
                j = kin.joints[jn]
                t = frames[j.parent] @ j.origin
                u = t[:3, :3] @ j.axis
                rel = pts - t[:3, 3]
                r = float(np.linalg.norm(rel - np.outer(rel @ u, u), axis=1).max())
                r += sum(abs(dq[k]) * glob[k] for k in names[:idx])   # downstream joints
                rb[link][jn] = r
            grow[link] = 0.5 * sum(abs(dq[jn]) * r for jn, r in rb[link].items())
        return rb, grow

    def _hull_growth(self, frames, link: str, dq: Mapping[str, float]) -> float:
        """Half the bound D on the path length of any point of `link`'s convex hull over
        a step of |dq| per joint ending or starting in the pose given by `frames` -
        _growth's derivation applied to the hull instead of the boxes:
        a point p moves by at most sum_j |dq_j| * max_t dist(p, axis_j(t)); distance
        from an axis is convex, so over the hull it peaks at a vertex, and over the
        step it is at most its value in this pose plus sum_k |dq_k| * R_k over the
        joints k between j and the link (R_k = _hull_radius, every configuration).
        Every point of the motion is then within D/2 of its position in one end pose."""
        kin = self.kin
        pts = self._hull_vertices(link) @ frames[link][:3, :3].T + frames[link][:3, 3]
        glob = self._hull_radius[link]
        names = list(glob)                 # nearest joint first
        total = 0.0
        for idx, jn in enumerate(names):
            if not dq.get(jn):
                continue
            j = kin.joints[jn]
            t = frames[j.parent] @ j.origin
            u = t[:3, :3] @ j.axis
            rel = pts - t[:3, 3]
            r = float(np.linalg.norm(rel - np.outer(rel @ u, u), axis=1).max())
            r += sum(abs(dq[k]) * glob[k] for k in names[:idx])   # downstream joints
            total += abs(dq[jn]) * r
        return 0.5 * total

    def body_hull_clearance(self, frames, link: str, body: Primitive) -> Tuple[float, float]:
        """(lo, hi) certified bounds of the signed clearance between the convex hull of
        arm `link`'s mesh and the body primitive's box (as_box - the same padded box
        the box test uses), in the pose given by `frames` (convex_distance)."""
        from gripperx_arm.convex_distance import signed_distance
        f, h = as_box(body)
        fl = frames[link]                       # into the link frame: R^T (p - t)
        corners = ((_CORNERS * h) @ f[:3, :3].T + f[:3, 3] - fl[:3, 3]) @ fl[:3, :3]
        return signed_distance(self._hull_vertices(link), corners)

    def _hull_planes(self, link: str) -> Tuple[np.ndarray, np.ndarray]:
        """(outward unit normals (F, 3), max over the hull's vertices of n . p (F,)) of
        the facets of `link`'s hull, link frame. The maxima are taken over the vertices
        themselves, not qhull's offsets. Empty without scipy."""
        cache = self.__dict__.setdefault('_hull_plane_cache', {})
        if link not in cache:
            pts = self._hull_vertices(link)
            try:
                from scipy.spatial import ConvexHull
                n = ConvexHull(pts).equations[:, :3]
                n = n / np.linalg.norm(n, axis=1)[:, None]
            except ImportError:
                n = np.zeros((0, 3))
            cache[link] = (n, (pts @ n.T).max(axis=0) if len(n) else np.zeros(0))
        return cache[link]

    def body_hull_lower(self, frames, link: str, body: Primitive) -> float:
        """A certified LOWER bound of the signed clearance body_hull_clearance bounds,
        from separating planes only (cheap, no iteration): for a unit n and any a of the
        hull, b of the box, |a - b| >= n . (b - a) >= min_box n . b - max_hull n . a,
        so the largest such gap over the hull's facet normals, the box's three axes (both
        signs) and the direction from the box to the hull vertex nearest to it is <= the
        clearance - and <= minus the penetration depth when they overlap. Exact unless
        the nearest features are two edges, or a box corner and a hull facet's edge."""
        f, h = as_box(body)
        fl = frames[link]
        best = -math.inf
        n, top = self._hull_planes(link)
        if len(n):
            corners = ((_CORNERS * h) @ f[:3, :3].T + f[:3, 3] - fl[:3, 3]) @ fl[:3, :3]
            best = float(((corners @ n.T).min(axis=0) - top).max())
        p = self._hull_vertices(link) @ (f[:3, :3].T @ fl[:3, :3]).T \
            + f[:3, :3].T @ (fl[:3, 3] - f[:3, 3])     # hull vertices in the box frame
        best = max(best, float(np.max(np.concatenate([p.min(axis=0) - h,
                                                      -p.max(axis=0) - h]))))
        out = p - np.clip(p, -h, h)                     # vertex minus its nearest box point
        d2 = np.einsum('ij,ij->i', out, out)
        k = int(np.argmin(d2))
        if d2[k] > 0.0:
            u = out[k] / math.sqrt(d2[k])
            best = max(best, float((p @ u).min() - np.abs(u) @ h))
        return best

    def _body_hull_check(self, frames, link: str, body: Primitive,
                         dq: Optional[Mapping[str, float]]) -> Optional[str]:
        """Second stage of the body check (cfg.body_hull_stage), for a box overlap of an
        arm link with a body box rigidly fixed to base_link - chassis, arm pedestal and,
        when fitted, the lidar: None if the clearance between the convex hull of the
        link's DECIMATED visual mesh (not the CAD part) and that unchanged body box,
        minus the hull's half motion bound (SR-17 rule 5), is >= body_hull_clearance_m;
        else the reason. The hull drops the arm boxes' 2 mm model-vs-CAD padding, so
        body_hull_clearance_m carries that margin (TO-VERIFY). NOT applied to wheels,
        steering modules or the ground. Only "clearance >= threshold" is decided: a
        cheap separating-plane bound first, then GJK without EPA (no penetration depth)."""
        from gripperx_arm.convex_distance import clearance_at_least
        need = self.cfg.body_hull_clearance_m
        g = self._hull_growth(frames, link, dq) if dq else 0.0
        if self.body_hull_lower(frames, link, body) - g >= need:
            return None                         # cheap certified bound suffices
        f, h = as_box(body)
        fl = frames[link]
        corners = ((_CORNERS * h) @ f[:3, :3].T + f[:3, 3] - fl[:3, 3]) @ fl[:3, :3]
        ok, c = clearance_at_least(self._hull_vertices(link), corners, need + g)
        if ok:
            return None
        clear = 'hull touches or penetrates the box' if c <= 0.0 else \
            f'hull clearance <= {c * 1e3:.1f} mm'
        return (f'{clear}{f" - motion bound {g * 1e3:.1f} mm" if g else ""}'
                f' < {need * 1e3:.1f} mm')

    def outside_limits(self, q: Mapping[str, float]) -> Dict[str, Tuple[float, float, float]]:
        """{joint: (value, lo, hi)} for every joint of q outside its L2 limit."""
        return {n: (float(q[n]), lo, hi) for n, (lo, hi) in self.kin.limits.items()
                if n in q and not lo <= q[n] <= hi}

    def recovery_limits(self, ref: Mapping[str, float]) -> Dict[str, Tuple[float, float]]:
        """Recovery from outside the limits (user decision 2026-10-09): the joint limits
        a setpoint FOLLOWING `ref` must meet. A joint of `ref` beyond a limit may stay
        where it is or move back toward the limit, never further out; a joint inside
        keeps its limit. Applied setpoint by setpoint, this lets a joint only leave the
        violating side, monotonically, and once inside it stays inside."""
        return {n: (min(lo, float(ref[n])), max(hi, float(ref[n])))
                for n, (lo, hi) in self.kin.limits.items()}

    def check_pose(self, q, envelope: Optional[Envelope] = None,
                   dq: Optional[Mapping[str, float]] = None,
                   near_poses: Optional[Sequence[Mapping[str, float]]] = None,
                   limits: Optional[Mapping[str, Tuple[float, float]]] = None,
                   collect_self: Optional[list] = None,
                   ground_skip: Optional[Collection[str]] = None) -> Verdict:
        """One pose. With `dq` (per-joint travel of the step this pose ends or
        starts), every box is checked grown for that motion (module docstring).
        `limits` replaces the L2 joint limits (recovery_limits only); every other
        check is unchanged. `collect_self` (a list): the self check does not stop at
        the first arm pair in contact but appends every one (sorted link names) and
        returns the first verdict; joint, ground and body checks still return at once.
        `ground_skip`: arm links (and 'TCP') whose ground clearance the caller checks
        itself (check_setpoints rise_ground only); every other check is unchanged."""
        kin, cfg = self.kin, self.cfg
        try:
            qd = self._q(q)
        except KeyError as exc:
            return Verdict(False, 'data', str(exc))
        if any(not math.isfinite(v) for v in qd.values()):
            return Verdict(False, 'data', 'non-finite joint value')
        for n, v in qd.items():
            lo, hi = (limits or kin.limits)[n]
            if not lo <= v <= hi:
                return Verdict(False, 'joint_limit',
                               f'{n} = {v:+.4f} rad outside [{lo:+.4f}, {hi:+.4f}]')
        env = envelope or self.envelope_swept
        frames = kin.fk(qd)
        prims = kin.collision_primitives(frames=frames)
        boxes: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        for p in prims:
            boxes.setdefault(p.link, []).append(as_box(p))
        if dq:
            rb, g = self._growth(frames, boxes, dq)
        else:
            rb, g = {}, {}
        arm = []
        for p in prims:
            f, h = as_box(p)
            arm.append((p, Primitive(p.link, 'box', f, tuple(2.0 * (h + g.get(p.link, 0.0))))))
        nrm, c = self._ground
        for _p, p in arm:
            f, h = as_box(p)
            low = float((((_CORNERS * h) @ f[:3, :3].T + f[:3, 3]) @ nrm).min()) + c
            need = cfg.ground_clearance_tip_m if p.link in cfg.tip_links \
                else cfg.ground_clearance_links_m
            if low < need and p.link not in (ground_skip or ()):
                return Verdict(False, 'ground',
                               f'{p.link} box bottom {low * 1e3:.1f} mm above ground '
                               f'< {need * 1e3:.1f} mm')
        tcp = frames[kin.tcp_link][:3, 3]
        tcp_h = float(tcp @ nrm + c) - max(g.values(), default=0.0)
        if tcp_h < cfg.ground_clearance_tip_m and TCP not in (ground_skip or ()):
            return Verdict(False, 'ground', f'TCP {tcp_h * 1e3:.1f} mm above ground '
                           f'< {cfg.ground_clearance_tip_m * 1e3:.1f} mm')
        near_poses = [qd] if near_poses is None else near_poses

        def allowed(a: str, b: str) -> bool:
            key = tuple(sorted((a, b)))
            return key in self.acm.always or (key in self.acm.near_home
                                              and self._pair_near_home(a, b, near_poses))
        # Bounding spheres first (exact pre-test: a sphere encloses its box), the SAT
        # only for the pairs whose spheres meet - most pairs are far apart.
        # A box overlap with the rigid body (chassis) is confirmed or cleared on the
        # arm link's convex hull against the same body box (_body_hull_check).
        spheres = _env_spheres(env)
        body_cache: Dict[Tuple[str, int], Optional[str]] = {}
        for _p, a in arm:
            fa, ha = as_box(a)
            ca, ra = fa[:3, 3], float(np.linalg.norm(ha))
            for (bound, samples), (cs, rs) in zip(env.groups, spheres):
                if bound is not None and not boxes_overlap(a, bound, cfg.body_clearance_m):
                    continue
                near = np.linalg.norm(cs - ca, axis=1) <= rs + ra + cfg.body_clearance_m
                for k in np.nonzero(near)[0]:
                    b = samples[k]
                    if allowed(a.link, b.link):
                        continue
                    if boxes_overlap(a, b, cfg.body_clearance_m):
                        if cfg.body_hull_stage and b.link in self._rigid_body \
                                and a.link in self._hull_radius:
                            key = (a.link, id(b))
                            if key not in body_cache:
                                body_cache[key] = self._body_hull_check(frames, a.link, b, dq)
                            if body_cache[key] is None:
                                continue
                            return Verdict(False, 'body', f'{a.link} vs {b.link} '
                                           f'({body_cache[key]}, {env.description})')
                        return Verdict(False, 'body', f'{a.link} vs {b.link} (boxes; '
                                       f'clearance {cfg.body_clearance_m * 1e3:.1f} mm, '
                                       f'{env.description})')
        # Self: only the RELATIVE motion of the two links matters - joints above
        # both move them together. Unexpanded boxes plus that relative bound; a box
        # overlap is confirmed (or cleared) on the convex hulls of the two meshes.
        hull_cache: Dict[Tuple[str, str], bool] = {}
        first = None
        sph = [(as_box(a)[0][:3, 3], float(np.linalg.norm(as_box(a)[1]))) for a, _g in arm]
        for i, (a, _ga) in enumerate(arm):
            for k, (b, _gb) in enumerate(arm[i + 1:], start=i + 1):
                if a.link == b.link:
                    continue
                if allowed(a.link, b.link):
                    continue
                rel = 0.0
                if dq:
                    ja, jb = set(rb[a.link]), set(rb[b.link])
                    rel = 0.5 * (sum(abs(dq[j]) * rb[b.link][j] for j in jb - ja)
                                 + sum(abs(dq[j]) * rb[a.link][j] for j in ja - jb))
                if np.linalg.norm(sph[i][0] - sph[k][0]) > sph[i][1] + sph[k][1] + \
                        cfg.self_clearance_m + rel:
                    continue
                if boxes_overlap(a, b, cfg.self_clearance_m + rel) \
                        and self._hulls_hit(a.link, b.link, frames, cfg.self_clearance_m + rel,
                                            hull_cache):
                    stage = 'hulls' if self._self_hulls(a.link, b.link) else 'boxes'
                    v = Verdict(False, 'self', f'{a.link} vs {b.link} ({stage}; '
                                f'clearance {cfg.self_clearance_m * 1e3:.1f} mm'
                                f'{f", motion bound {rel * 1e3:.1f} mm" if rel else ""})')
                    if collect_self is None:
                        return v
                    pair = tuple(sorted((a.link, b.link)))
                    if pair not in collect_self:
                        collect_self.append(pair)
                    first = first or v
        return first or Verdict(True, checked=1)

    # ------------------------------------------------------------------ self-contact recovery
    def self_contacts(self, q, envelope: Optional[Envelope] = None, dq=None, near_poses=None,
                      limits=None, ground_skip=None) -> Tuple[Verdict, List[Tuple[str, str]]]:
        """(verdict, every arm self pair failing the self check). A non-'self' verdict
        (joint limit, ground, body) is returned as it stands, with no pairs."""
        hits: list = []
        v = self.check_pose(q, envelope, dq, near_poses, limits, collect_self=hits,
                            ground_skip=ground_skip)
        return v, (hits if v.rule == 'self' else [])

    def _hull_vertices(self, link: str) -> np.ndarray:
        cache = self.__dict__.setdefault('_hull_vertex_cache', {})
        if link not in cache:
            pts = self.hulls[link]
            try:                       # the hull's vertices only: the same convex hull
                from scipy.spatial import ConvexHull
                cache[link] = pts[ConvexHull(pts).vertices]
            except ImportError:
                cache[link] = pts
        return cache[link]

    def _b_in_a(self, q, la: str, lb: str) -> np.ndarray:
        frames = self.kin.fk(self._q(q))
        t = np.linalg.inv(frames[la]) @ frames[lb]
        return self._hull_vertices(lb) @ t[:3, :3].T + t[:3, 3]

    def pair_clearance(self, q, la: str, lb: str) -> Optional[Tuple[float, float]]:
        """(lo, hi) bounds, at most 0.1 micrometre apart, of the SIGNED clearance of two
        arm links at pose q, on the convex hulls of their meshes - the model's
        simplified shapes, not the meshes: > 0 their gap, < 0 minus their penetration
        depth (gripperx_arm.convex_distance). None if a link has no hull (or the self
        hull stage is off)."""
        if not self._self_hulls(la, lb):
            return None
        from gripperx_arm.convex_distance import signed_distance
        return signed_distance(self._hull_vertices(la), self._b_in_a(q, la, lb))

    def describe_contacts(self, q, pairs) -> str:
        """'arm_limb1_link vs arm_limb3_link, clearance -0.5 mm; ...'."""
        out = []
        for la, lb in pairs:
            c = self.pair_clearance(q, la, lb)
            out.append(f'{la} vs {lb}' + ('' if c is None else f', clearance {c[0] * 1e3:.1f} mm'))
        return '; '.join(out)

    def pair_motion_bound(self, q, dq: Mapping[str, float], la: str, lb: str) -> float:
        """How far the two links can move RELATIVE to each other, and so how much their
        clearance can change, anywhere inside the joint box of a step of |dq| that
        starts or ends at q: the self check's half motion bound (module docstring),
        per side of the nearer end pose."""
        qd = self._q(q)
        frames = self.kin.fk(qd)
        boxes: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        for p in self.kin.collision_primitives(frames=frames):
            boxes.setdefault(p.link, []).append(as_box(p))
        full = {n: abs(float(dq.get(n, 0.0))) for n in self.kin.joint_names}
        rb, _g = self._growth(frames, boxes, full)
        ja, jb = set(rb[la]), set(rb[lb])
        return 0.5 * (sum(full[j] * rb[lb][j] for j in jb - ja)
                      + sum(full[j] * rb[la][j] for j in ja - jb))

    # ------------------------------------------------------------------ ground recovery
    def ground_heights(self, q, dq: Optional[Mapping[str, float]] = None) -> Dict[str, float]:
        """Per arm link with collision boxes, the height above the ground plane of the
        lowest corner of its boxes, and under TCP the TCP's height - each as the ground
        check sees it: with `dq`, grown for that motion (check_pose)."""
        frames = self.kin.fk(self._q(q))
        prims = self.kin.collision_primitives(frames=frames)
        boxes: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        for p in prims:
            boxes.setdefault(p.link, []).append(as_box(p))
        g = self._growth(frames, boxes, dq)[1] if dq else {}
        nrm, c = self._ground
        out: Dict[str, float] = {}
        for link, bs in boxes.items():
            e = g.get(link, 0.0)
            out[link] = min(float((((_CORNERS * (h + e)) @ f[:3, :3].T + f[:3, 3]) @ nrm).min())
                            for f, h in bs) + c
        out[TCP] = float(frames[self.kin.tcp_link][:3, 3] @ nrm + c) - max(g.values(),
                                                                           default=0.0)
        return out

    def ground_need(self, name: str) -> float:
        """The configured clearance the ground check holds `name` (link or TCP) to."""
        if name == TCP or name in self.cfg.tip_links:
            return self.cfg.ground_clearance_tip_m
        return self.cfg.ground_clearance_links_m

    def ground_shortfalls(self, q, dq: Optional[Mapping[str, float]] = None
                          ) -> Dict[str, float]:
        """{link or TCP: height} for everything failing the ground check at q (grown
        for `dq` as check_pose does), sorted by name."""
        h = self.ground_heights(q, dq)
        return {n: h[n] for n in sorted(h) if h[n] < self.ground_need(n)}

    def describe_trigger(self, trigger: Mapping[str, Tuple[float, float]]) -> str:
        """Verdict.rising_start -> 'arm_limb5_link 1.1 mm (grown for the first step
        -2.9 mm) < 10.0 mm; ...'."""
        return '; '.join(f'{n} {s * 1e3:.1f} mm (grown for the first step {g * 1e3:.1f} mm)'
                         f' < {self.ground_need(n) * 1e3:.1f} mm'
                         for n, (s, g) in sorted(trigger.items(), key=lambda kv: kv[1][0]))

    def describe_ground(self, low: Mapping[str, float]) -> str:
        """'arm_limb5_link 1.2 mm < 10.0 mm; ...'."""
        return '; '.join(f'{n} {h * 1e3:.1f} mm < {self.ground_need(n) * 1e3:.1f} mm'
                         for n, h in low.items())

    def ground_step_bound(self, qa, qb, name: str) -> float:
        """A certified lower bound of `name`'s ground clearance (lowest box corner of
        an arm link, or the TCP) anywhere inside the joint box of the step qa -> qb -
        wherever the joints are between the two setpoints, in any order: the lowest
        clearance at the box's vertices minus sum_j dq_j^2 * r_j / 8. With the other
        joints held, a point's height is a sinusoid in q_j whose second derivative is
        at most its distance r_j from j's axis (_growth's bound, valid over the whole
        step), so tensor-product linear interpolation of the vertex values is off by at
        most that sum, and the interpolant is no lower than its lowest vertex."""
        a, b = self._q(qa), self._q(qb)
        d = {n: wrap(b[n] - a[n]) for n in a}
        moving = [n for n in a if d[n] != 0.0]
        low = math.inf
        for bits in itertools.product((0, 1), repeat=len(moving)):
            v = dict(a)
            v.update({n: a[n] + d[n] for n, on in zip(moving, bits) if on})
            low = min(low, self.ground_heights(v)[name])
        dq = {n: abs(d[n]) for n in a}
        frames = self.kin.fk(a)
        if name == TCP:
            pts = frames[self.kin.tcp_link][:3, 3][None, :]
            glob = self._tcp_radius
        else:
            pts = np.vstack([(_CORNERS * h) @ f[:3, :3].T + f[:3, 3]
                             for p in self.kin.collision_primitives(frames=frames)
                             if p.link == name for f, h in [as_box(p)]])
            glob = self._radius[name]
        names = list(glob)                 # nearest joint first
        err = 0.0
        for idx, jn in enumerate(names):
            j = self.kin.joints[jn]
            t = frames[j.parent] @ j.origin
            u = t[:3, :3] @ j.axis
            rel = pts - t[:3, 3]
            r = float(np.linalg.norm(rel - np.outer(rel @ u, u), axis=1).max())
            r += sum(dq[k] * glob[k] for k in names[:idx])    # downstream joints
            err += dq[jn] ** 2 * r / 8.0
        return low - err

    def ground_step_low(self, qa, qb, name: str) -> Optional[float]:
        """min of ground_step_corner_lows: the entry's exact lowest point over the step."""
        r = self.ground_step_corner_lows(qa, qb, name)
        return None if r is None else float(r[1].min())

    def ground_step_corner_lows(self, qa, qb, name: str
                                ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """(height at qa, EXACT lowest height over the step) per box corner of `name`
        (an arm link; the TCP is one point), in the same corner order, over a step
        qa -> qb that moves ONE revolute joint, the others held - the whole step, not
        only its ends. None if more than one joint (or a non-revolute
        one) moves: then only ground_step_bound applies. Rotating by t about the joint's
        axis (unit u through o), a point p has the height
        h(t) = (o + par) . n + c + cos t * (perp . n) + sin t * ((u x perp) . n)
        with par/perp the parts of p - o along and across u - a sinusoid, minimised
        over [0, dq] in closed form (at an end, or at its trough if that lies inside).
        A box's lowest point is one of its corners, so the minimum over the corners is
        the link's lowest point over the step; per corner it bounds that corner's own
        path (audit GR2 L1: a heel corner sinking while the tip rises)."""
        a, b = self._q(qa), self._q(qb)
        d = {n: wrap(b[n] - a[n]) for n in a}
        moving = [n for n in a if d[n] != 0.0]
        frames = self.kin.fk(a)
        if name == TCP:
            pts = frames[self.kin.tcp_link][:3, 3][None, :]
            glob = self._tcp_radius
        else:
            pts = np.vstack([(_CORNERS * h) @ f[:3, :3].T + f[:3, 3]
                             for p in self.kin.collision_primitives(frames=frames)
                             if p.link == name for f, h in [as_box(p)]])
            glob = self._radius[name]
        nrm, c = self._ground
        if not moving or (len(moving) == 1 and moving[0] not in glob):
            h0 = pts @ nrm + c                           # the step does not move it
            return h0, h0
        if len(moving) > 1 or self.kin.joints[moving[0]].type not in ('revolute',
                                                                      'continuous'):
            return None
        jn = moving[0]
        j = self.kin.joints[jn]
        t = frames[j.parent] @ j.origin
        o, u = t[:3, 3], t[:3, :3] @ j.axis
        rel = pts - o
        par = np.outer(rel @ u, u)
        perp = rel - par
        base = (o + par) @ nrm + c
        bc, bs = perp @ nrm, np.cross(u, perp) @ nrm
        lo, hi = sorted((0.0, d[jn]))
        h_lo = base + bc * math.cos(lo) + bs * math.sin(lo)
        h_hi = base + bc * math.cos(hi) + bs * math.sin(hi)
        low = np.minimum(h_lo, h_hi)
        amp = np.hypot(bc, bs)
        trough = np.arctan2(bs, bc) + math.pi            # where cos(t - phase) = -1
        inside = lo + np.mod(trough - lo, 2.0 * math.pi) <= hi
        low = np.where(inside & (amp > 0.0), np.minimum(low, base - amp), low)
        return base + bc, low

    def subdivide(self, qa, qb) -> List[Dict[str, float]]:
        """Joint-linear sub-steps from qa to qb (qb included, qa not), no joint
        moving more than max_joint_step_rad per sub-step."""
        qa, qb = self._q(qa), self._q(qb)
        d = {n: wrap(qb[n] - qa[n]) for n in qa}
        n = max(1, int(math.ceil(max((abs(v) for v in d.values()), default=0.0)
                                 / self.cfg.max_joint_step_rad)))
        return [{k: qa[k] + d[k] * i / n for k in qa} for i in range(1, n + 1)]

    def refine(self, poses: Sequence[Mapping[str, float]]) -> List[Dict[str, float]]:
        """The same consecutive setpoints, with joint-linear points inserted wherever a
        step moves a joint by more than max_joint_step_rad. Every original pose is kept."""
        return self._refine_marked(poses)[0]

    def _refine_marked(self, poses) -> Tuple[List[Dict[str, float]], List[bool]]:
        """refine(), and per refined pose whether it is one of `poses` (not inserted)."""
        qs = [self._q(p) for p in poses]
        out, orig = qs[:1], [True] * len(qs[:1])
        for b in qs[1:]:
            sub = self.subdivide(out[-1], b)
            out += sub
            orig += [False] * (len(sub) - 1) + [True]
        return out, orig

    def check_setpoints(self, poses: Sequence[Mapping[str, float]],
                        envelope: Optional[Envelope] = None,
                        deadline: Optional[float] = None,
                        recovery: bool = False, separate_self: bool = False,
                        rise_ground: bool = False,
                        rising: Optional[Collection[str]] = None,
                        ground_skip: Optional[Collection[str]] = None,
                        trace: Optional[list] = None,
                        stop: Optional[Callable[[], bool]] = None,
                        resume_from: int = 0) -> Verdict:
        """A sequence of consecutive setpoints, poses[0] = where the motion starts.
        Each pose is checked grown for the larger of its two adjacent steps (per
        joint), which covers every configuration inside the joint box of either
        step (module docstring). Nothing of the sequence is approved unless all of
        it passes. `deadline` (time.monotonic()): past it, the remaining poses are
        not checked and the verdict is a 'timeout' rejection (planner time cap). `stop`:
        the same, as soon as it returns True (a cancelled search: a check takes seconds on
        the robot, and it would keep the interpreter from the leader tick).
        `resume_from`: a verdict's resume_at, for the SAME arguments - the poses before it
        passed already. A check carries no state between poses unless a link rises or a
        self pair separates: then resume_at is 0 and the check starts over.
        `recovery` (user decision 2026-10-09): poses[0] may lie outside the joint
        limits; every later pose is held to recovery_limits(the pose before it), so a
        joint outside only moves back toward its limit and, once inside, stays inside
        (the joint box of each step then lies inside those limits too). Collision,
        ground and body checks are unchanged - a start pose that collides is refused.

        `separate_self` (user decision 2026-10-09, option C; SR-17 rules 12/13): the
        ARM SELF pairs failing the self check at poses[0] (in contact, or closer than
        the self clearance plus the motion bound) are accepted while they separate.
        Never a chassis, wheel or ground contact: those are checked as always. Per such
        pair: c_k = its signed clearance at pose k on the convex hulls of the meshes
        (the model's simplified shapes; pair_clearance, certified bounds 0.1 um apart,
        each condition uses the unfavourable bound), r_k = the bound on its relative
        motion inside the joint box of step k-1 -> k (pair_motion_bound). At every
        pose k >= 1 where the pair fails the normal self check:
          * c_k >= c_(k-1) (to 1 nm): non-decreasing at every checked pose;
          * c_(k-1) - r_k >= c_0 - self_contact_dip_m: inside a step the clearance is
            at least min(c_(k-1), c_k) - r_k (every point of the swept motion lies
            within r_k of one end pose), so it never falls more than the dip below its
            start value - "non-decreasing" cannot be proven BETWEEN setpoints from
            contact, only this bound.
        Where the pair passes the normal self check (its growth covers both adjacent
        steps) the normal check holds - so the pair is closer than the normal margin
        only while it is separating, also if a later, larger step's motion bound takes
        it below that margin again. Every other pair, and a pair clear at the start, is
        checked normally. A pair without hulls cannot be measured and stays refused.

        `rise_ground` (user decision 2026-10-12; SR-17, the operator's go_home only):
        poses[0] is the PLAN's start. Every link (or the TCP) failing the ground check
        there - grown for the first step, as the normal check at sub-step 0 is - is
        RISING, also one with a static clearance <= 0 (user decision 2026-10-15: the
        model penetrates the ground plane while the real arm rests on the floor, as
        after leader mode's Space left it limp). `rising` (a later segment of the same
        plan): the entries still rising at poses[0], from the previous segment's
        Verdict.rising_end; nothing new can
        start rising there. Per rising entry, c_k = ground_heights(poses[k]) (static),
        at every pose k >= 1:
          * only ONE joint moves in the step k-1 -> k (audit r11 F2): with several
            joints a servo lagging setpoints behind the others leaves the step's joint
            box, and nothing certifies the clearance there; with one joint moving and
            the others holding, a lagging servo is on the already certified way;
          * c_k > c_(k-1) (by more than 1 nm): strictly rising at every setpoint, so
            never below its start clearance at any setpoint;
          * c_(k-1) > 0: ground_step_bound(poses[k-1], poses[k]) > 0 - anywhere inside
            the step's joint box it stays above the ground. Between setpoints only this
            is guaranteed - it may dip below c_(k-1) by that certified amount (user
            decision 2026-10-12, option a);
          * c_(k-1) <= 0 (in the ground): "above the ground" cannot hold, and the
            certified bound sits below c_(k-1) by its interpolation error, so it cannot
            certify a rise either. Instead, from ground_step_corner_lows (EXACT, closed
            form, the whole one-joint step): the entry's lowest point is never deeper
            than c_(k-1) (to 1 nm), and every box corner (the TCP: the point) at or
            below 0 at the setpoint before never goes lower than its own height there
            (audit GR2 L1) - no part of the link in the floor presses further into it,
            between setpoints included (user decision 2026-10-15: "as long as
            recovering just pulls upward"). Every penetrating entry is held to it on
            its own, not only the lowest: a lift that raises the deepest link by
            pivoting a shallower one into the floor is refused.
        It stops rising at the first pose where it passes the normal ground check (grown
        for both adjacent steps); from there on it is checked normally, as is every
        other link from the start. Joint limits, chassis, wheels and self pairs are
        checked as always, combined only with `recovery` and `separate_self`.

        `trace` (a list): per pose of `poses` (not the inserted sub-steps), the entries
        still rising there - what a step continuing from that pose must be checked with
        (`rising`), as this check decided it (grown for BOTH adjacent steps). Filled up
        to the pose that failed. `ground_skip`: links (or 'TCP') whose grown ground
        check the caller replaces with its own (the leader's rising step from the
        measured pose: ground_step_bound > 0); every other check is unchanged."""
        try:
            qs, orig = self._refine_marked(poses)
        except KeyError as exc:
            return Verdict(False, 'data', str(exc))
        names = self.kin.joint_names
        steps = [{n: abs(wrap(qs[i + 1][n] - qs[i][n])) for n in names}
                 for i in range(len(qs) - 1)]
        separating: Dict[Tuple[str, str], dict] = {}   # pair -> {pose index: (lo, hi)}
        prev: Dict[str, float] = {}                    # rising link or TCP -> static c_(k-1)
        trigger: Dict[str, Tuple[float, float]] = {}
        for k, q in enumerate(qs):
            if k < resume_from:
                continue
            if (deadline is not None and time.monotonic() > deadline) \
                    or (stop is not None and stop()):
                stateful = prev or trigger or separating or trace is not None
                return Verdict(False, 'timeout', 'time budget exhausted', k, k,
                               resume_at=0 if stateful else k)
            adj = [steps[i] for i in (k - 1, k) if 0 <= i < len(steps)]
            dq = {n: max((s[n] for s in adj), default=0.0) for n in names}
            if k == 0 and (rise_ground or rising):
                static = self.ground_heights(q)
                if rise_ground and rising is None:
                    trigger = {n: (static[n], h)
                               for n, h in self.ground_shortfalls(q, dq).items()}
                    first = list(trigger)
                else:
                    first = sorted(rising or ())
                prev = {n: static[n] for n in first}
            elif prev:
                moving = [n for n in names if qs[k - 1][n] != q[n]]   # joint names
                if len(moving) > 1:
                    return Verdict(False, 'ground', f'{", ".join(sorted(prev))}: ground '
                                   'recovery - only one joint may move while lifting out of '
                                   f'the margin (moving: {", ".join(moving)})', k, k + 1)
                static = self.ground_heights(q)
                for n in list(prev):
                    if static[n] <= prev[n] + 1e-9:
                        return Verdict(False, 'ground', f'{n}: ground recovery - the clearance '
                                       f'does not rise ({prev[n] * 1e3:+.4f} -> '
                                       f'{static[n] * 1e3:+.4f} mm)', k, k + 1)
                    if prev[n] > 0.0:
                        bound = self.ground_step_bound(qs[k - 1], q, n)
                        if bound <= 0.0:
                            return Verdict(False, 'ground', f'{n}: ground recovery - between '
                                           'setpoints the joint box of the step may reach the '
                                           f'ground (certified bound {bound * 1e3:+.4f} mm)',
                                           k, k + 1)
                    else:
                        r = self.ground_step_corner_lows(qs[k - 1], q, n)
                        if r is None:            # not one revolute joint: not certified
                            r = (np.array([prev[n]]), np.array([-math.inf]))
                        h0, low = r
                        if low.min() < prev[n] - 1e-9:
                            return Verdict(False, 'ground', f'{n}: ground recovery from at or '
                                           'below the ground - between setpoints it sinks '
                                           f'deeper ({prev[n] * 1e3:+.4f} -> lowest '
                                           f'{low.min() * 1e3:+.4f} mm)', k, k + 1)
                        sink = (h0 <= 0.0) & (low < h0 - 1e-9)
                        if sink.any():
                            i = int(np.argmax(np.where(sink, h0 - low, -np.inf)))
                            return Verdict(False, 'ground', f'{n}: ground recovery from at or '
                                           'below the ground - between setpoints a corner in '
                                           f'the ground sinks deeper ({h0[i] * 1e3:+.4f} -> '
                                           f'lowest {low[i] * 1e3:+.4f} mm)', k, k + 1)
                    prev[n] = static[n]
                grown = self.ground_shortfalls(q, dq)
                for n in list(prev):
                    if n not in grown:
                        del prev[n]              # above the margin: checked normally
            skip = (set(prev) | set(ground_skip or ())) or None
            if trace is not None and orig[k]:
                trace.append(tuple(sorted(prev)))
            # the joint boxes of both adjacent steps lie inside the hull of these
            nh = [qs[i] for i in (k - 1, k, k + 1) if 0 <= i < len(qs)]
            lim = None
            if recovery:
                lim = self.recovery_limits(qs[k - 1] if k else q)
                away = [f'{n} {qs[k - 1][n]:+.4f} -> {q[n]:+.4f} rad' for n in names
                        if not lim[n][0] <= q[n] <= lim[n][1]]
                if away:
                    return Verdict(False, 'joint_limit', 'recovery from outside the joint '
                                   'limits: a joint moves away from its limit or back out '
                                   f'({", ".join(away)})', k, k + 1)
            if separate_self and k == 0:
                v, hits = self.self_contacts(q, envelope, dq, nh, lim, skip)
                if not v.ok and not hits:
                    v.index, v.checked = k, k + 1
                    return v
                for pair in hits:
                    c = self.pair_clearance(q, *pair)
                    if c is None:
                        v.index, v.checked = k, k + 1
                        return v
                    separating[pair] = {0: c}
                continue
            if separating:
                v, hits = self.self_contacts(q, envelope, dq, nh, lim, skip)
                if not v.ok and (not hits or set(hits) - set(separating)):
                    v.index, v.checked = k, k + 1
                    return v
                for pair in hits:             # below the normal margin: it must separate
                    seen = separating[pair]
                    if k - 1 not in seen:
                        seen[k - 1] = self.pair_clearance(qs[k - 1], *pair)
                    lo_p, hi_p = seen[k - 1]
                    lo, hi = seen[k] = self.pair_clearance(q, *pair)
                    r = self.pair_motion_bound(qs[k - 1], steps[k - 1], *pair)
                    if lo < hi_p - 1e-9:
                        return Verdict(False, 'self', f'{pair[0]} vs {pair[1]}: self-contact '
                                       f'recovery - the clearance decreases ({hi_p * 1e3:+.4f}'
                                       f' -> {lo * 1e3:+.4f} mm)', k, k + 1)
                    if lo_p - r < seen[0][1] - self.cfg.self_contact_dip_m:
                        return Verdict(False, 'self', f'{pair[0]} vs {pair[1]}: self-contact '
                                       f'recovery - step motion bound {r * 1e3:.3f} mm could '
                                       f'dip the clearance more than '
                                       f'{self.cfg.self_contact_dip_m * 1e3:.3f} mm below its '
                                       f'start value {seen[0][1] * 1e3:+.3f} mm', k, k + 1)
                continue
            v = self.check_pose(q, envelope, dq, nh, lim, ground_skip=skip)
            if not v.ok:
                v.index, v.checked = k, k + 1
                return v
        return Verdict(True, checked=len(qs), rising_start=trigger,
                       rising_end=tuple(sorted(prev)))

    def check_path(self, qa, qb, envelope: Optional[Envelope] = None,
                   deadline: Optional[float] = None, recovery: bool = False,
                   separate_self: bool = False, rise_ground: bool = False,
                   rising: Optional[Collection[str]] = None) -> Verdict:
        """The joint-linear motion from qa (where the arm is) to qb."""
        return self.check_setpoints([qa, qb], envelope, deadline, recovery, separate_self,
                                    rise_ground, rising)


def describe_outside(out: Mapping[str, Tuple[float, float, float]]) -> str:
    """SafetyFilter.outside_limits -> 'arm_limb2_joint +0.2209 rad > +0.1597 rad'."""
    return ', '.join(f'{n} {v:+.4f} rad {">" if v > hi else "<"} {hi if v > hi else lo:+.4f} rad'
                     for n, (v, lo, hi) in out.items())
