"""Arm kinematics, read from the URDF. Pure Python + numpy, no ROS imports.

Everything geometric comes from the robot description at construction time -
joint origins, axes, limits, collision primitives and the arm mount. Nothing is
duplicated here, so a CAD re-import changes this module's answers without a code
change.

Built to be called per setpoint by a filter node: construct once (parses the
URDF), then call fk()/ik_tcp_down()/boxes_overlap() per setpoint. Measured
2026-10-07 on the dev laptop (i5-8265U, one core): fk 0.05 ms, fk + collision
primitives 0.07 ms, ik_tcp_down (4 branches, each FK-verified) 0.5 ms, SAT on all
231 primitive pairs 3.8 ms - about 4.4 ms per setpoint against 33 ms at 30 Hz.
Not yet measured on the Pi 5.

Conventions
    q             joint angles in radians, URDF joint convention (q = 0 = CAD pose)
    frames        4x4 homogeneous matrices in `base_link` unless stated otherwise
    "tcp down"    the TCP z axis (approach, out of the jaws) along -z of base_link
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

ACTUATED = ('revolute', 'continuous', 'prismatic')


# --------------------------------------------------------------------------- #
# small transform helpers
# --------------------------------------------------------------------------- #
def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def make_tf(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def wrap(a: float) -> float:
    """Wrap to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def wrap_into(a: float, center: float) -> float:
    """Wrap to [center - pi, center + pi): the per-joint angle window of a joint
    whose travel is not centred on q = 0 (range_center_rad, calibration file)."""
    return center + (a - center + math.pi) % (2.0 * math.pi) - math.pi


def _floats(text: Optional[str], default: Sequence[float]) -> List[float]:
    return [float(v) for v in text.split()] if text else list(default)


def _origin(elem: Optional[ET.Element]) -> np.ndarray:
    if elem is None:
        return np.eye(4)
    return make_tf(_floats(elem.get('xyz'), (0, 0, 0)), _floats(elem.get('rpy'), (0, 0, 0)))


# --------------------------------------------------------------------------- #
# model
# --------------------------------------------------------------------------- #
@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray            # parent link -> joint frame at q = 0
    axis: np.ndarray              # unit vector, joint frame
    lower: float = -math.inf
    upper: float = math.inf


@dataclass
class Primitive:
    """A collision primitive. `frame` is its pose in base_link; for a box `size`
    is the full edge lengths, for a cylinder (radius, length) along local z, for
    a sphere (radius,)."""
    link: str
    shape: str
    frame: np.ndarray
    size: Tuple[float, ...]


@dataclass
class SweptGroup:
    """Envelope of the body parts moved by one non-arm joint over its range."""
    joint: Optional[str]          # None = parts under no revolute joint (static)
    range: Tuple[float, float]
    bound: Optional[Primitive]    # one AABB around all samples (cheap pre-test)
    samples: List[Primitive]


@dataclass
class IkSolution:
    q: Dict[str, float]
    within_limits: bool
    residual: float               # |FK(q).tcp - target| in metres, a self-check
    branch: str                   # e.g. "front/elbow+"


@dataclass
class IkResult:
    reachable: bool               # at least one solution inside the joint limits
    solutions: List[IkSolution] = field(default_factory=list)
    reason: str = ''


class ArmKinematics:
    """Kinematic tree of the arm below `arm_root`, mounted on `base_link`.

    The path base_link -> arm_root must consist of fixed joints only (it is the
    static mount). Every actuated joint in the subtree of arm_root is an arm
    joint, listed in `joint_names` in topological order.
    """

    def __init__(self, urdf_xml: str, base_link: str = 'base_link',
                 arm_root: str = 'arm_base_link', tcp_link: str = 'arm_tcp_link'):
        root = ET.fromstring(urdf_xml)
        self.base_link, self.arm_root, self.tcp_link = base_link, arm_root, tcp_link
        joints: Dict[str, Joint] = {}
        for j in root.findall('joint'):
            lim = j.find('limit')
            jt = j.get('type')
            lower, upper = -math.inf, math.inf
            if jt in ('revolute', 'prismatic') and lim is not None:
                lower, upper = float(lim.get('lower', '-inf')), float(lim.get('upper', 'inf'))
            ax = j.find('axis')
            axis = np.array(_floats(ax.get('xyz') if ax is not None else None, (1, 0, 0)))
            joints[j.find('child').get('link')] = Joint(
                j.get('name'), jt, j.find('parent').get('link'), j.find('child').get('link'),
                _origin(j.find('origin')), axis / np.linalg.norm(axis), lower, upper)
        self._by_child = joints

        # static mount base_link -> arm_root
        mount = np.eye(4)
        link = arm_root
        while link != base_link:
            j = joints.get(link)
            if j is None:
                raise ValueError(f'{arm_root} is not below {base_link}')
            if j.type in ACTUATED:
                raise ValueError(f'mount path {base_link}->{arm_root} has actuated joint {j.name}')
            mount = j.origin @ mount
            link = j.parent
        self.mount = mount

        # subtree of arm_root, topological order
        children: Dict[str, List[Joint]] = {}
        for j in joints.values():
            children.setdefault(j.parent, []).append(j)
        self._order: List[Joint] = []
        stack = [arm_root]
        while stack:
            for j in sorted(children.get(stack.pop(0), []), key=lambda x: x.name):
                self._order.append(j)
                stack.append(j.child)
        self.links = [arm_root] + [j.child for j in self._order]
        if tcp_link not in self.links:
            raise ValueError(f'{tcp_link} is not below {arm_root}')
        self.joint_names = [j.name for j in self._order if j.type in ACTUATED]
        self.joints = {j.name: j for j in self._order}
        self.limits = {n: (self.joints[n].lower, self.joints[n].upper) for n in self.joint_names}

        # Non-arm links below base_link: the robot body the arm can hit.
        self._env_order: List[Joint] = []
        stack = [base_link]
        while stack:
            for j in sorted(children.get(stack.pop(0), []), key=lambda x: x.name):
                if j.child != arm_root:
                    self._env_order.append(j)
                    stack.append(j.child)
        env_links = {base_link} | {j.child for j in self._env_order}
        self.parent_link = {j.child: j.parent for j in joints.values()}

        self._collision: Dict[str, List[Tuple[str, np.ndarray, Tuple[float, ...]]]] = {}
        self._env_collision: Dict[str, List[Tuple[str, np.ndarray, Tuple[float, ...]]]] = {}
        for link_el in root.findall('link'):
            name = link_el.get('name')
            if name not in self.links and name not in env_links:
                continue
            for col in link_el.findall('collision'):
                geo = col.find('geometry')
                t = _origin(col.find('origin'))
                if geo.find('box') is not None:
                    prim = ('box', tuple(_floats(geo.find('box').get('size'), ())))
                elif geo.find('cylinder') is not None:
                    c = geo.find('cylinder')
                    prim = ('cylinder', (float(c.get('radius')), float(c.get('length'))))
                elif geo.find('sphere') is not None:
                    prim = ('sphere', (float(geo.find('sphere').get('radius')),))
                else:
                    continue    # meshes are not collision primitives
                if name in self.links:
                    self._collision.setdefault(name, []).append((prim[0], t, prim[1]))
                else:
                    self._env_collision.setdefault(name, []).append((prim[0], t, prim[1]))
        # Body primitives with every non-arm joint at 0 (steering straight ahead).
        # NOT valid for a filter on its own - use swept_environment() or
        # environment_at() with the measured steering angles.
        self.environment: List[Primitive] = self.environment_at()

        self._ik = None
        try:
            self._ik = _TcpDownSolver(self)
        except ValueError as exc:
            self.ik_unsupported_reason = str(exc)

    # ------------------------------------------------------------------ FK
    def _qvec(self, q) -> Dict[str, float]:
        if q is None:
            return {}
        if isinstance(q, Mapping):
            return dict(q)
        return dict(zip(self.joint_names, q))

    def fk(self, q=None) -> Dict[str, np.ndarray]:
        """Pose of every arm link in base_link. q: mapping name->value or a
        sequence in `joint_names` order; missing joints are 0."""
        qd = self._qvec(q)
        out = {self.arm_root: self.mount}
        for j in self._order:
            t = out[j.parent] @ j.origin
            v = qd.get(j.name, 0.0)
            if v and j.type in ('revolute', 'continuous'):
                m = np.eye(4)
                m[:3, :3] = axis_angle(j.axis, v)
                t = t @ m
            elif v and j.type == 'prismatic':
                m = np.eye(4)
                m[:3, 3] = j.axis * v
                t = t @ m
            out[j.child] = t
        return out

    def tcp(self, q=None) -> np.ndarray:
        return self.fk(q)[self.tcp_link]

    def collision_primitives(self, q=None, frames=None) -> List[Primitive]:
        frames = frames if frames is not None else self.fk(q)
        return [Primitive(link, shape, frames[link] @ t, size)
                for link, prims in self._collision.items() for shape, t, size in prims]

    def environment_at(self, q_env: Optional[Mapping[str, float]] = None) -> List[Primitive]:
        """Body primitives in base_link for the given non-arm joint values
        (missing = 0). Wheel spin is irrelevant: a cylinder is invariant under it."""
        qd = q_env or {}
        pose = {self.base_link: np.eye(4)}
        for j in self._env_order:
            t = pose[j.parent] @ j.origin
            v = qd.get(j.name, 0.0)
            if v and j.type == 'revolute':
                m = np.eye(4)
                m[:3, :3] = axis_angle(j.axis, v)
                t = t @ m
            pose[j.child] = t
        return [Primitive(link, shape, pose[link] @ t, size)
                for link, prims in self._env_collision.items() for shape, t, size in prims]

    def swept_environment(self, step: float = math.radians(10.0),
                          ranges: Optional[Mapping[str, Tuple[float, float]]] = None
                          ) -> List['SweptGroup']:
        """Conservative envelopes of the body over the full range of every revolute
        non-arm joint (the steering), with the range taken from the URDF limits -
        or, per joint, from `ranges` (name -> (lower, upper)), e.g. the measured
        MECHANICAL steering stops, which lie well outside the URDF working limits.

        Per joint: boxes enclosing its subtree's primitives at angles spaced at
        most `step` apart, each grown by the chord 2 r sin(step / 4) a point at
        radius r from the axis can move away from the nearest sample, plus one
        axis-aligned `bound` box around all of them for a cheap first test.
        Primitives under no revolute joint form a group with joint None."""
        groups, swept_links = [], set()
        for j in self._env_order:
            if j.type != 'revolute':
                continue
            sub, stack = set(), [j.child]
            while stack:
                link = stack.pop()
                sub.add(link)
                stack += [c.child for c in self._env_order if c.parent == link]
            swept_links |= sub
            lower, upper = (ranges or {}).get(j.name, (j.lower, j.upper))
            n = max(1, int(math.ceil((upper - lower) / step)))
            angles = np.linspace(lower, upper, n + 1)
            d = (upper - lower) / n
            samples = []
            for a in angles:
                for p in self.environment_at({j.name: float(a)}):
                    if p.link not in sub:
                        continue
                    f, h = as_box(p)
                    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                                        for sz in (-1, 1)]) * h
                    w = corners @ f[:3, :3].T + f[:3, 3]
                    axis_pt = (self._env_pose0(j.parent) @ j.origin)[:3, 3]
                    axis_dir = (self._env_pose0(j.parent) @ j.origin)[:3, :3] @ j.axis
                    rel = w - axis_pt
                    r = np.linalg.norm(rel - np.outer(rel @ axis_dir, axis_dir), axis=1).max()
                    grow = 2.0 * r * math.sin(d / 4.0)
                    samples.append(Primitive(p.link, 'box', f, tuple(2 * (h + grow))))
            pts = []
            for p in samples:
                f, h = as_box(p)
                c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                              for sz in (-1, 1)]) * h
                pts.append(c @ f[:3, :3].T + f[:3, 3])
            pts = np.vstack(pts)
            lo, hi = pts.min(0), pts.max(0)
            bound = Primitive(j.name, 'box', make_tf((lo + hi) / 2, (0, 0, 0)), tuple(hi - lo))
            groups.append(SweptGroup(j.name, (lower, upper), bound, samples))
        static = [p for p in self.environment if p.link not in swept_links]
        if static:
            groups.insert(0, SweptGroup(None, (0.0, 0.0), None, static))
        return groups

    def _env_pose0(self, link: str) -> np.ndarray:
        t = np.eye(4)
        while link != self.base_link:
            j = self._by_child[link]
            t = j.origin @ t
            link = j.parent
        return t

    def into_limits(self, q: Mapping[str, float]) -> Dict[str, float]:
        """The same physical pose with each revolute angle shifted by a multiple of
        2 pi into its [lower, upper] interval where one exists (limits may lie beyond
        +-pi: the SO101 joints travel past +-180 deg). Values with no such shift are
        returned unchanged - within_limits() then rejects them."""
        out = dict(q)
        for n, v in q.items():
            if n not in self.limits or self.joints[n].type != 'revolute':
                continue
            lo, hi = self.limits[n]
            k = math.ceil((lo - v) / (2.0 * math.pi) - 1e-12)
            cand = v + 2.0 * math.pi * k
            if lo - 1e-9 <= cand <= hi + 1e-9:
                out[n] = cand
        return out

    def within_limits(self, q: Mapping[str, float], tol: float = 1e-9) -> bool:
        return all(self.limits[n][0] - tol <= v <= self.limits[n][1] + tol
                   for n, v in q.items() if n in self.limits)

    # ------------------------------------------------------------------ IK
    def ik_tcp_down(self, x: float, y: float, z: float, roll: float = 0.0,
                    tcp_yaw: Optional[float] = None,
                    hold: Optional[Mapping[str, float]] = None) -> IkResult:
        """All closed-form solutions placing the TCP at (x, y, z) in base_link
        with the approach axis straight down.

        The wrist-roll joint does not move the TCP. It is set to `roll`, or - if
        `tcp_yaw` is given - so the TCP x axis points at heading tcp_yaw in
        base_link. Joints outside the positioning chain (the gripper) are taken
        from `hold`, default 0."""
        if self._ik is None:
            raise RuntimeError(f'tcp-down IK unsupported for this model: {self.ik_unsupported_reason}')
        return self._ik.solve(np.array([x, y, z], dtype=float), roll, tcp_yaw, hold or {})

    def ik_tcp_tilted(self, x: float, y: float, z: float, tilt: float = 0.0,
                      roll: float = 0.0,
                      hold: Optional[Mapping[str, float]] = None) -> IkResult:
        """ik_tcp_down generalised to an approach axis tilted by `tilt` (rad) from
        straight down, inside the arm's pitch plane: tilt = 0 is straight down
        (identical to ik_tcp_down), tilt > 0 leans the approach axis back toward the
        arm's yaw axis (the jaws point down and inward - P's taught grip pose has
        approach_tilt ~ +30 deg). Both pan branches use the same tilt relative to the
        yaw axis. The wrist roll is set to `roll`; a heading (tcp_yaw) is only defined
        for the vertical approach, use ik_tcp_down for it."""
        if self._ik is None:
            raise RuntimeError(f'tcp-down IK unsupported for this model: {self.ik_unsupported_reason}')
        return self._ik.solve(np.array([x, y, z], dtype=float), roll, None, hold or {},
                              tilt=float(tilt))

    def approach_tilt(self, q) -> float:
        """The pose's approach-axis tilt from straight down inside the pitch plane, in
        the convention of ik_tcp_tilted (0 = vertical, > 0 = leaning back toward the
        yaw axis). Out-of-plane components are ignored: the chain keeps the approach
        axis in the pitch plane."""
        if self._ik is None:
            raise RuntimeError(f'tcp-down IK unsupported for this model: {self.ik_unsupported_reason}')
        return self._ik.tilt_of(self._qvec(q))


class _TcpDownSolver:
    """Closed form for a yaw + 3 parallel pitch joints + roll chain.

    Up to four solutions: two pan branches - "front": the TCP lies on the +x
    side of the yaw axis in the arm's own pan frame (q1 = 0 faces base +x);
    "back": pan turned ~pi, the arm reaches over its shoulder - times two elbow
    branches.

    Structure is read from the model at q = 0 and checked; a model that does not
    have it is refused rather than solved approximately:
      j1 axis vertical; j2..j4 axes parallel to each other and perpendicular to
      j1; the TCP on the j5 axis, which is parallel to the TCP approach axis and
      lies in the pitch plane.
    """
    TOL = 1e-4      # rad / m, structural tolerance

    def __init__(self, kin: ArmKinematics):
        self.kin = kin
        chain, link = [], kin.tcp_link
        while link != kin.arm_root:
            j = kin._by_child[link]
            if j.type in ACTUATED:
                chain.insert(0, j)
            link = j.parent
        if len(chain) != 5 or any(j.type != 'revolute' for j in chain):
            raise ValueError(f'expected 5 revolute joints to the TCP, found {[j.name for j in chain]}')
        self.chain = [j.name for j in chain]
        f0 = kin.fk()
        ax = [f0[j.child][:3, :3] @ j.axis for j in chain]
        pt = [f0[j.child][:3, 3] for j in chain]
        tcp0 = f0[kin.tcp_link]
        zb = np.array([0.0, 0.0, 1.0])
        if abs(abs(ax[0] @ zb) - 1.0) > self.TOL:
            raise ValueError('first joint axis is not vertical')
        ev = ax[0] * np.sign(ax[0] @ zb)
        ep = ax[1]
        if abs(ep @ ev) > self.TOL:
            raise ValueError('pitch axis not perpendicular to yaw axis')
        for a in ax[2:4]:
            if abs(abs(a @ ep) - 1.0) > self.TOL:
                raise ValueError('pitch axes not parallel')
        a0 = tcp0[:3, 2]
        if abs(abs(a0 @ ax[4]) - 1.0) > self.TOL or abs(a0 @ ep) > self.TOL:
            raise ValueError('roll axis not along the approach axis / not in the pitch plane')
        if np.linalg.norm(np.cross(ax[4], tcp0[:3, 3] - pt[4])) > self.TOL:
            raise ValueError('TCP not on the roll axis')
        er = np.cross(ep, ev)
        if er[0] < 0.0:     # orient the pitch plane so "front" means +x at q1 = 0
            ep, er = -ep, -er
        self.o1, self.ev, self.ep, self.er = pt[0], ev, ep, er
        self.s1 = float(np.sign(ax[0] @ ev))
        self.s = [float(np.sign(a @ ep)) for a in ax[1:4]]
        P = lambda p: np.array([(p - self.o1) @ er, (p - self.o1) @ ev])  # noqa: E731
        self.c2, self.c3, self.c4, self.t0 = P(pt[1]), P(pt[2]), P(pt[3]), P(tcp0[:3, 3])
        self.d = float((tcp0[:3, 3] - self.o1) @ ep)
        self.a0_ang = math.atan2(a0 @ ev, a0 @ er)
        l2, l3 = self.c3 - self.c2, self.c4 - self.c3
        self.L2, self.L3 = float(np.hypot(*l2)), float(np.hypot(*l3))
        self.al2, self.al3 = math.atan2(l2[1], l2[0]), math.atan2(l3[1], l3[0])
        self.wrist = self.t0 - self.c4     # c4 -> tcp at q = 0, planar

    @staticmethod
    def _rot(v: np.ndarray, a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    def _pan_radial(self, q: Mapping[str, float]) -> np.ndarray:
        """The pitch plane's outward radial direction e_r at the pan angle of q."""
        return axis_angle(self.ev, self.s1 * q.get(self.chain[0], 0.0)) @ self.er

    def _approach(self, q: Mapping[str, float], tilt: float, front: bool) -> np.ndarray:
        """The approach axis the solver aims at for branch `front` and `tilt`."""
        ang = -math.pi / 2 - tilt if front else math.pi / 2 + tilt + math.pi
        return math.cos(ang) * self._pan_radial(q) + math.sin(ang) * self.ev

    def tilt_of(self, q: Mapping[str, float]) -> float:
        a = self.kin.tcp(q)[:3, 2]
        er = self._pan_radial(q)
        # front branch: a = -sin(t) er - cos(t) ev; the back branch has er pointing
        # away from the TCP, which the sign of the TCP's radial offset tells
        t = self.kin.tcp(q)[:3, 3] - self.o1
        sgn = 1.0 if t @ er >= 0.0 else -1.0
        return math.atan2(-sgn * (a @ er), -(a @ self.ev))

    def solve(self, p, roll, tcp_yaw, hold, tilt: float = 0.0) -> IkResult:
        kin = self.kin
        w = p - self.o1
        wr, wp, h = w @ self.er, w @ self.ep, w @ self.ev
        rho2 = wr * wr + wp * wp - self.d * self.d
        if rho2 < 0.0:
            return IkResult(False, [], 'target inside the lateral offset of the yaw axis')
        # A rotation by theta about +e_p is a rotation by -theta in the (r, v)
        # plane, hence phi = -s * theta below.
        # In-plane approach angle (atan2 over (e_r, e_v)): straight down is -pi/2 on
        # both branches; a tilt toward the yaw axis is the mirror image on the back
        # branch, whose e_r points away from the TCP.
        sols, why = [], 'out of reach'
        for side, rho in (('front', math.sqrt(rho2)), ('back', -math.sqrt(rho2))):
            ang = -math.pi / 2 - tilt if side == 'front' else math.pi / 2 + tilt + math.pi
            Phi = ang - self.a0_ang
            psi = math.atan2(wp, wr) - math.atan2(self.d, rho)
            c4 = np.array([rho, h]) - self._rot(self.wrist, Phi)
            D = c4 - self.c2
            cd = (D @ D - self.L2 ** 2 - self.L3 ** 2) / (2 * self.L2 * self.L3)
            if abs(cd) > 1.0:
                continue
            for elbow in (1.0, -1.0):
                dl = elbow * math.acos(cd)
                psi2 = math.atan2(D[1], D[0]) - math.atan2(self.L3 * math.sin(dl),
                                                           self.L2 + self.L3 * math.cos(dl))
                phi2 = psi2 - self.al2
                phi3 = psi2 + dl - self.al3 - phi2
                phi4 = Phi - phi2 - phi3
                q = dict(hold)
                q[self.chain[0]] = wrap(self.s1 * psi)
                q[self.chain[1]] = wrap(-self.s[0] * phi2)
                q[self.chain[2]] = wrap(-self.s[1] * phi3)
                q[self.chain[3]] = wrap(-self.s[2] * phi4)
                q[self.chain[4]] = 0.0
                frames = kin.fk(q)
                t = frames[kin.tcp_link]
                if tcp_yaw is None:
                    q[self.chain[4]] = wrap(roll)
                else:
                    # roll axis is vertical here; rotate TCP x onto the heading
                    j5 = kin.joints[self.chain[4]]
                    u5 = frames[j5.child][:3, :3] @ j5.axis
                    cur = math.atan2(t[1, 0], t[0, 0])
                    q[self.chain[4]] = wrap(float(np.sign(u5[2])) * (tcp_yaw - cur))
                q = kin.into_limits(q)
                t = kin.tcp(q)
                res = float(np.linalg.norm(t[:3, 3] - p)) + float(np.linalg.norm(
                    t[:3, 2] - self._approach(q, tilt, side == 'front')))
                sols.append(IkSolution(q, kin.within_limits(q), res,
                                       f'{side}/elbow{"+" if elbow > 0 else "-"}'))
        if not sols:
            return IkResult(False, [], why)
        ok = any(s.within_limits for s in sols)
        return IkResult(ok, sols, '' if ok else 'all solutions outside joint limits')


# --------------------------------------------------------------------------- #
# servo ticks <-> joint angles
# --------------------------------------------------------------------------- #
@dataclass
class TickCalibration:
    """q = sign * (ticks - zero_ticks) * 2 pi / ticks_per_rev, wrapped into the
    joint's window [range_center - pi, range_center + pi).

    zero_ticks = the servo reading at URDF q = 0; sign = +1 when increasing ticks
    turn the joint positively about its URDF axis; range_center = the middle of the
    joint's travel (SO101 limb2..limb4 travel past +-180 deg from the CAD zero, so a
    window centred on 0 would split their travel). All calibration values
    (config/arm_calibration.yaml); range_center defaults to 0."""
    joints: List[str]
    zero_ticks: List[float]
    signs: List[int]
    ticks_per_rev: float = 4096.0
    range_center: Optional[List[float]] = None

    def __post_init__(self):
        if self.range_center is None:
            self.range_center = [0.0] * len(self.joints)
        if not (len(self.joints) == len(self.zero_ticks) == len(self.signs)
                == len(self.range_center)):
            raise ValueError('joints, zero_ticks, signs and range_center must have equal length')
        if any(s not in (1, -1) for s in self.signs):
            raise ValueError('signs must be +1 or -1')

    def to_radians(self, ticks: Sequence[float]) -> Dict[str, float]:
        k = 2.0 * math.pi / self.ticks_per_rev
        return {n: wrap_into(s * (t - z) * k, c)
                for n, t, z, s, c in zip(self.joints, ticks, self.zero_ticks, self.signs,
                                         self.range_center)}

    def to_ticks(self, q: Mapping[str, float]) -> List[int]:
        k = self.ticks_per_rev / (2.0 * math.pi)
        return [int(round(z + s * q[n] * k)) % int(self.ticks_per_rev)
                for n, z, s in zip(self.joints, self.zero_ticks, self.signs)]


# --------------------------------------------------------------------------- #
# overlap test (oriented boxes, separating-axis theorem)
# --------------------------------------------------------------------------- #
def as_box(p: Primitive) -> Tuple[np.ndarray, np.ndarray]:
    """(frame, half extents) of the box enclosing a primitive. Exact for a box,
    conservative for a cylinder (along local z) or a sphere."""
    if p.shape == 'box':
        h = np.array(p.size) / 2.0
    elif p.shape == 'cylinder':
        h = np.array([p.size[0], p.size[0], p.size[1] / 2.0])
    else:
        h = np.full(3, p.size[0])
    return p.frame, h


def boxes_overlap(a: Primitive, b: Primitive, margin: float = 0.0) -> bool:
    """True if the (enclosing) boxes of a and b come closer than `margin`.
    Touching counts as overlap."""
    (ta, ha), (tb, hb) = as_box(a), as_box(b)
    ha, hb = ha + margin / 2.0, hb + margin / 2.0
    A, B = ta[:3, :3], tb[:3, :3]
    R = A.T @ B
    t = A.T @ (tb[:3, 3] - ta[:3, 3])
    absR = np.abs(R) + 1e-12
    for i in range(3):
        if abs(t[i]) > ha[i] + hb @ absR[i]:
            return False
    for j in range(3):
        if abs(t @ R[:, j]) > ha @ absR[:, j] + hb[j]:
            return False
    for i in range(3):
        for j in range(3):
            i1, i2, j1, j2 = (i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3
            ra = ha[i1] * absR[i2, j] + ha[i2] * absR[i1, j]
            rb = hb[j1] * absR[i, j2] + hb[j2] * absR[i, j1]
            if abs(t[i2] * R[i1, j] - t[i1] * R[i2, j]) > ra + rb:
                return False
    return True


# --------------------------------------------------------------------------- #
# convex-hull overlap (GJK) - second stage behind the boxes
# --------------------------------------------------------------------------- #
def read_stl_vertices(path) -> np.ndarray:
    """Unique vertices of a binary STL, in the file's frame."""
    import struct
    d = open(path, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    v = raw[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)
    return np.unique(np.round(v, 7), axis=0)


def _triple(a, b, c):
    return np.cross(np.cross(a, b), c)


def _line(s):
    b, a = s
    ab, ao = b - a, -a
    if ab @ ao > 0.0:
        d = _triple(ab, ao, ab)
        return [b, a], d
    return [a], ao


def _triangle(s):
    c, b, a = s
    ab, ac, ao = b - a, c - a, -a
    abc = np.cross(ab, ac)
    if np.cross(abc, ac) @ ao > 0.0:
        if ac @ ao > 0.0:
            return [c, a], _triple(ac, ao, ac)
        return _line([b, a])
    if np.cross(ab, abc) @ ao > 0.0:
        return _line([b, a])
    if abc @ ao > 0.0:
        return [c, b, a], abc
    return [b, c, a], -abc


def _tetra(s):
    d, c, b, a = s
    ab, ac, ad, ao = b - a, c - a, d - a, -a
    if np.cross(ab, ac) @ ao > 0.0:
        return _triangle([c, b, a]), False
    if np.cross(ac, ad) @ ao > 0.0:
        return _triangle([d, c, a]), False
    if np.cross(ad, ab) @ ao > 0.0:
        return _triangle([b, d, a]), False
    return None, True


def hulls_overlap(pa: np.ndarray, pb: np.ndarray, inflate: float = 0.0,
                  max_iter: int = 64) -> bool:
    """True if the convex hulls of the point sets pa, pb (N x 3, same frame) come
    closer than `inflate`. GJK on the Minkowski difference; touching counts as
    overlap, and an undecided run (degenerate input) answers True."""
    r = inflate / 2.0

    def support(d):
        n = np.linalg.norm(d)
        u = d / n if n > 0 else d
        return (pa[np.argmax(pa @ d)] + r * u) - (pb[np.argmin(pb @ d)] - r * u)

    d = pa[0] - pb[0]
    if d @ d < 1e-24:
        return True
    a = support(-d)
    simplex, d = [a], -a
    for _ in range(max_iter):
        if d @ d < 1e-24:
            return True
        a = support(d)
        if a @ d < 0.0:
            return False
        simplex.append(a)
        if len(simplex) == 2:
            simplex, d = _line(simplex)
        elif len(simplex) == 3:
            simplex, d = _triangle(simplex)
        else:
            res, inside = _tetra(simplex)
            if inside:
                return True
            simplex, d = res
    return True
