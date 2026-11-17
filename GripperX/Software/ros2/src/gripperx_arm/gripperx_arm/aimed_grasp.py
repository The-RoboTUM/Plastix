"""Aimed grasp, FR-16 stage 1 (user decisions 2026-10-13): P grasps at P's grip height
but at the litter's (x, y), with the gripper VERTICAL, so the arm absorbs the robot's
arrival error without a camera. Pure Python + numpy, no ROS imports.

Everything is derived from the running model and configuration, never typed in:
  * the grasp height is the TCP height of P's taught grip pose (config/arm_poses.yaml)
    under the active calibration, through FK of the active URDF;
  * the gripper opening checked at the grasp is P's `gripper.open` (the planner checks
    the approach's goal pose with the gripper as it is then: open);
  * a candidate passes only the SafetyFilter's static check (SR-17 rule 4: joint
    limits, ground, body incl. wheels, self) of the active filter configuration.

The ALLOWED REGION is the set of targets (x, y) in base_link for which a vertical grasp
at that height has an IK solution inside the joint limits that passes the static check
for some wrist roll of the bounded set, against the REFERENCE envelope - wheels
straight ahead +- safety_filter.steer_motion_margin_rad (the steering an arrival
normally ends in). Membership is evaluated exactly per target (in_region); the grid
(derive_region) is a diagnostic of the same definition. A target outside it is
OUT_OF_REACH - the reason that triggers a re-approach at the gateway. A target inside
it whose candidates all fail against the LIVE envelope (the measured steering) is
ENVELOPE_REJECTED.

STEEP APPROACH FROM ABOVE (user decision 2026-11-16): the gripper reaches the grasp
pose by a straight vertical descent from a PRE-GRASP pose pregrasp_height above it (same
x, y, tilt and wrist roll, gripper open). A candidate is a grasp pose AND its descent:
Cartesian waypoints at most descent_step apart from the pre-grasp down to the grasp,
each an IK solution of the grasp's branch and roll that passes the same static check,
with the joint-linear motion between two waypoints keeping the TCP within
descent_max_lateral of the vertical line (sampled). Region membership needs all of it;
a target whose pre-grasp (or any waypoint) has no IK is OUT_OF_REACH.

What this does NOT compensate is listed in claude/REQUIREMENTS.md FR-16 (revision
2026-10-13): localisation error, Octopus / line-calibration error, object height and
shape, the unmeasured grasp depth, the horizontal TCP calibration.
"""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from gripperx_arm.kinematics import wrap

# Reason codes of an aimed pick (PickPlastic result `reason`).
OUT_OF_REACH = 'OUT_OF_REACH'            # no IK in the limits / outside the allowed region
ENVELOPE_REJECTED = 'ENVELOPE_REJECTED'  # inside the region, every candidate fails live
NO_PATH = 'NO_PATH'                      # the planner found no (direct, if automatic) path
STALE_TF = 'STALE_TF'                    # target transform or base-still transform too old
BASE_MOVING = 'BASE_MOVING'              # the base moved (or its rest cannot be shown)
STEERING_UNKNOWN = 'STEERING_UNKNOWN'    # a wheel without a steering reading at the start
INVALID_TARGET = 'INVALID_TARGET'        # no frame, non-finite coordinates
AIMED_UNAVAILABLE = 'AIMED_UNAVAILABLE'  # aiming not configured / model missing
REFUSED_AFTER_MOTION = 'REFUSED_AFTER_MOTION'  # any refusal once the arm has moved
BASE_MOVED_DURING_PICK = 'BASE_MOVED_DURING_PICK'  # SR-16 E5: aborted, returns home
TARGET_SHIFTED = 'TARGET_SHIFTED'        # SR-16 E11 item 4: off the preview's target

IK_RESIDUAL_M = 1e-6                     # FK(IK) self-check, as the IK reports it


@dataclass
class Candidate:
    q: Dict[str, float]          # every arm joint incl. the gripper (open)
    roll: float                  # the wrist roll chosen (rad)
    branch: str                  # IK branch, e.g. 'front/elbow+'
    # the vertical descent: pre-grasp pose first, q (the grasp) last; same roll throughout
    descent: List[Dict[str, float]] = field(default_factory=list)

    @property
    def pregrasp(self) -> Dict[str, float]:
        return self.descent[0]


@dataclass
class AimResult:
    ok: bool
    reason: str = ''
    detail: str = ''
    candidates: List[Candidate] = field(default_factory=list)


@dataclass
class Region:
    """The allowed region on a grid, a diagnostic of AimedGraspSolver.in_region."""
    xs: np.ndarray
    ys: np.ndarray
    mask: np.ndarray             # [len(xs), len(ys)] bool
    step: float
    z_grasp: float
    tilt: float
    fingerprint: str
    envelope: str

    @property
    def area_cm2(self) -> float:
        return float(self.mask.sum()) * self.step * self.step * 1e4

    def x_range_at(self, y: float) -> Optional[Tuple[float, float]]:
        j = int(np.argmin(np.abs(self.ys - y)))
        col = self.xs[self.mask[:, j]]
        return (float(col.min()), float(col.max())) if col.size else None

    def components(self) -> List[np.ndarray]:
        """4-connected parts of the region, largest first, as [k, 2] index arrays."""
        seen = np.zeros_like(self.mask)
        parts = []
        for start in zip(*np.nonzero(self.mask)):
            if seen[start]:
                continue
            seen[start] = True
            stack, cells = [start], []
            while stack:
                i, j = stack.pop()
                cells.append((i, j))
                for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                    if (0 <= a < self.mask.shape[0] and 0 <= b < self.mask.shape[1]
                            and self.mask[a, b] and not seen[a, b]):
                        seen[a, b] = True
                        stack.append((a, b))
            parts.append(np.array(cells))
        return sorted(parts, key=len, reverse=True)

    def summary(self) -> Dict[str, object]:
        out: Dict[str, object] = {
            'area_cm2': round(self.area_cm2, 1), 'step_m': self.step,
            'z_grasp_base_link_m': round(self.z_grasp, 4),
            'tilt_deg': round(math.degrees(self.tilt), 2), 'envelope': self.envelope,
            'fingerprint': self.fingerprint, 'cells': int(self.mask.sum())}
        parts = self.components()
        if parts:
            main = parts[0]
            xs, ys = self.xs[main[:, 0]], self.ys[main[:, 1]]
            out['main_area_cm2'] = round(len(main) * self.step * self.step * 1e4, 1)
            out['main_x_m'] = [round(float(xs.min()), 3), round(float(xs.max()), 3)]
            out['main_y_m'] = [round(float(ys.min()), 3), round(float(ys.max()), 3)]
            out['other_parts'] = [
                {'cells': len(c), 'x_m': [round(float(self.xs[c[:, 0]].min()), 3),
                                          round(float(self.xs[c[:, 0]].max()), 3)],
                 'y_m': [round(float(self.ys[c[:, 1]].min()), 3),
                         round(float(self.ys[c[:, 1]].max()), 3)]} for c in parts[1:]]
            r0 = self.x_range_at(0.0)
            out['x_m_at_y0'] = None if r0 is None else [round(r0[0], 3), round(r0[1], 3)]
        return out

    def text(self) -> str:
        return json.dumps(self.summary(), sort_keys=True)


class AimedGraspSolver:
    """Candidate grasp poses for a target (x, y) in base_link.

    grip_q: P's taught grip pose in radians (every arm joint); its TCP height is the
    grasp height. gripper_open: the gripper angle checked at the grasp. roll_offsets:
    the bounded wrist-roll search, relative to P's grip roll, in search order (the
    user: the roll is free). tilt: approach tilt from vertical (0 = vertical, user
    decision 2026-10-13 option b). ref_envelope: the reference envelope of the region
    (None = the filter's default, wheels swept over their mechanical stops).
    pregrasp_height / descent_step / descent_max_lateral (m, required, no default):
    the vertical descent (module docstring)."""

    def __init__(self, flt, grip_q: Mapping[str, float], gripper_open: float,
                 roll_offsets: Sequence[float], tilt: float = 0.0, ref_envelope=None, *,
                 pregrasp_height: float, descent_step: float, descent_max_lateral: float):
        kin = flt.kin
        if kin._ik is None:
            raise ValueError(f'no closed-form IK for this model: {kin.ik_unsupported_reason}')
        self.flt, self.kin = flt, kin
        self.chain = list(kin._ik.chain)
        self.roll_joint = self.chain[4]
        self.hold = {n: float(grip_q[n]) for n in kin.joint_names if n not in self.chain}
        gripper = [n for n in kin.joint_names if n not in self.chain]
        if len(gripper) != 1:
            raise ValueError(f'expected one joint outside the IK chain, found {gripper}')
        self.gripper = gripper[0]
        self.hold[self.gripper] = float(gripper_open)
        self.grip_q = {n: float(grip_q[n]) for n in kin.joint_names}
        self.z = float(kin.tcp(self.grip_q)[2, 3])
        self.roll0 = self.grip_q[self.roll_joint]
        self.roll_offsets = [float(r) for r in roll_offsets]
        if not self.roll_offsets:
            raise ValueError('roll_offsets must not be empty')
        self.tilt = float(tilt)
        self.ref_envelope = ref_envelope
        self.pregrasp_height = float(pregrasp_height)
        self.descent_step = float(descent_step)
        self.descent_max_lateral = float(descent_max_lateral)
        for name, v in (('pregrasp_height', self.pregrasp_height),
                        ('descent_step', self.descent_step),
                        ('descent_max_lateral', self.descent_max_lateral)):
            if not (math.isfinite(v) and v > 0.0):
                raise ValueError(f'{name} must be finite and > 0, not {v}')
        # heights of the descent's waypoints above the grasp, pre-grasp first
        n = max(1, int(math.ceil(self.pregrasp_height / self.descent_step - 1e-9)))
        self.descent_heights = [self.pregrasp_height * (n - k) / n for k in range(n + 1)]
        # links the wrist roll moves: a rejection naming none of them (nor the TCP's
        # roll-free position aside) cannot be cured by another roll
        child = kin.joints[self.roll_joint].child
        self._roll_links, stack = set(), [child]
        while stack:
            link = stack.pop()
            self._roll_links.add(link)
            stack += [j.child for j in kin.joints.values() if j.parent == link]
        self._arm_links = sorted(kin.links, key=len, reverse=True)

    def _roll_cannot_help(self, v) -> bool:
        """Only an optimisation of the roll search: a wrong True only drops
        candidates (a refusal), never admits one."""
        if v.rule not in ('ground', 'body', 'self'):
            return False
        named = {n for n in self._arm_links if n in v.detail}
        return bool(named or 'TCP' in v.detail) and not (named & self._roll_links)

    # ---------------------------------------------------------------- per target
    def _ik(self, x: float, y: float, dz: float = 0.0) -> Tuple[list, str]:
        res = self.kin.ik_tcp_tilted(x, y, self.z + dz, self.tilt, roll=self.roll0,
                                     hold=self.hold)
        sols = [s for s in res.solutions if s.residual < IK_RESIDUAL_M]
        # the roll is varied below; only the positioning joints have to be inside
        pos = self.chain[:4]
        sols = [s for s in sols if self.kin.within_limits({n: s.q[n] for n in pos})]
        if not sols:
            why = res.reason or ('every IK solution outside the joint limits'
                                 if res.solutions else 'no IK solution')
            return [], why

        def dist(s):                      # nearest to P's taught configuration first
            return sum(abs(wrap(s.q[n] - self.grip_q[n])) for n in pos)
        return sorted(sols, key=dist), ''

    def _descent_poses(self, x: float, y: float, s, cache: dict):
        """The positioning joints of the descent for the grasp solution s, pre-grasp
        first: at every height of descent_heights the IK solution of s's branch nearest
        to the pose below it (continuity), or (None, why). Roll-free: cached per
        branch."""
        if s.branch in cache:
            return cache[s.branch]
        pos = self.chain[:4]
        poses, prev = [dict(s.q)], s.q
        for dz in self.descent_heights[-2::-1]:          # upward from the grasp
            sols, why = self._ik(x, y, dz)
            sols = [t for t in sols if t.branch == s.branch]
            if not sols:
                out = (None, f'no IK of branch {s.branch} {dz * 1e3:.0f} mm above the grasp '
                             f'({why or "other branches only"})')
                cache[s.branch] = out
                return out
            best = min(sols, key=lambda t: sum(abs(wrap(t.q[n] - prev[n])) for n in pos))
            # the same physical angles, continuous with the pose below (no 2 pi jump)
            q = dict(best.q)
            for n in pos:
                q[n] = prev[n] + wrap(q[n] - prev[n])
            poses.append(q)
            prev = q
        poses = poses[::-1]
        # the TCP lies on the roll axis and the gripper is outside the chain: the
        # lateral bound is roll-free too
        for a, b in zip(poses, poses[1:]):
            lat = self._lateral(x, y, {**s.q, **a}, {**s.q, **b})
            if lat > self.descent_max_lateral:
                cache[s.branch] = (None, f'descent leaves the vertical line by '
                                         f'{lat * 1e3:.1f} mm (> descent_max_lateral '
                                         f'{self.descent_max_lateral * 1e3:.1f} mm)')
                return cache[s.branch]
        cache[s.branch] = (poses, '')
        return cache[s.branch]

    def _lateral(self, x: float, y: float, qa, qb) -> float:
        """Largest horizontal TCP distance from the vertical line through (x, y) on the
        joint-linear motion qa -> qb (sampled, the ends included)."""
        n = max(4, int(math.ceil(self.descent_step / 0.0025)))
        worst = 0.0
        for k in range(n + 1):
            f = k / n
            q = {j: qa[j] + f * (qb[j] - qa[j]) for j in qa}
            t = self.kin.tcp(q)
            worst = max(worst, math.hypot(t[0, 3] - x, t[1, 3] - y))
        return worst

    def _descent(self, x: float, y: float, s, q_grasp: Dict[str, float], envelope,
                 cache: dict):
        """(descent waypoints, None) or (None, Verdict-or-text): the waypoints of s's
        positioning joints (_descent_poses: IK and lateral bound) with the grasp pose's
        roll and gripper, every one inside the joint limits and passing the static
        check."""
        poses, why = self._descent_poses(x, y, s, cache)
        if poses is None:
            return None, why
        pos = self.chain[:4]
        out = []
        for k, p in enumerate(poses[:-1]):
            q = {**q_grasp, **{n: p[n] for n in pos}}
            if not self.kin.within_limits(q):
                return None, (f'descent waypoint {self.descent_heights[k] * 1e3:.0f} mm above '
                              'the grasp outside the joint limits')
            out.append(q)
        out.append(dict(q_grasp))
        for q in out[:-1]:                       # the grasp itself is checked by the caller
            v = self.flt.check_pose(q, envelope)
            if not v.ok:
                return None, v
        return out, None

    def candidates(self, x: float, y: float, envelope=None,
                   limit: Optional[int] = None) -> AimResult:
        """Static-check-passing grasp poses for (x, y) with their vertical descent from
        the pre-grasp pose (same roll), at most `limit`, nearest to P's taught
        configuration first and P's roll first. No IK inside the joint limits at the
        grasp, or no descent IK for any grasp solution -> OUT_OF_REACH; IK but every
        candidate rejected -> ENVELOPE_REJECTED, with the first rejection."""
        if not (math.isfinite(x) and math.isfinite(y)):
            return AimResult(False, INVALID_TARGET, f'non-finite target ({x}, {y})')
        sols, why = self._ik(x, y)
        if not sols:
            return AimResult(False, OUT_OF_REACH,
                             f'({x:+.3f}, {y:+.3f}) at grasp height {self.z:+.4f} m: {why}')
        out: List[Candidate] = []
        first = ''
        cache: dict = {}
        reach = []                     # why the descent has no IK, per grasp solution
        for s in sols:
            if self._descent_poses(x, y, s, cache)[0] is None:
                reach.append(f'{s.branch}: {cache[s.branch][1]}')
                continue
            for off in self.roll_offsets:
                q = dict(s.q)
                q[self.roll_joint] = self.roll0 + off
                q = self.kin.into_limits(q)
                if not self.kin.within_limits(q):
                    continue
                v = self.flt.check_pose(q, envelope)
                where = 'grasp'
                if v.ok:
                    descent, v = self._descent(x, y, s, q, envelope, cache)
                    where = 'descent'
                    if descent is not None:
                        out.append(Candidate(q, q[self.roll_joint], s.branch, descent))
                        if limit is not None and len(out) >= limit:
                            return AimResult(True, '', '', out)
                        continue
                if not first:
                    first = (f'{s.branch} roll {math.degrees(q[self.roll_joint]):+.0f} '
                             f'deg ({where}): {v}')
                # a text (limits, lateral) is the positioning joints' - the roll cannot help
                if isinstance(v, str) or self._roll_cannot_help(v):
                    break
        if out:
            return AimResult(True, '', '', out)
        if len(reach) == len(sols):
            return AimResult(False, OUT_OF_REACH,
                             f'({x:+.3f}, {y:+.3f}): grasp at {self.z:+.4f} m reachable, but '
                             f'no pre-grasp descent ({self.pregrasp_height * 1e3:.0f} mm): '
                             + '; '.join(reach))
        return AimResult(False, ENVELOPE_REJECTED,
                         f'({x:+.3f}, {y:+.3f}): every candidate rejected; first: {first}')

    def in_region(self, x: float, y: float) -> AimResult:
        """The allowed-region test: candidates() against the reference envelope. A
        failure of any kind here is OUT_OF_REACH (outside the allowed region)."""
        r = self.candidates(x, y, self.ref_envelope, limit=1)
        if r.ok or r.reason == INVALID_TARGET:
            return r
        return AimResult(False, OUT_OF_REACH, f'outside the allowed region: {r.detail}')

    def solve(self, x: float, y: float, envelope=None,
              limit: Optional[int] = None, margin: float = 0.0,
              margin_step: float = 0.02) -> AimResult:
        """Region first (fast refusal, the gateway re-approaches on OUT_OF_REACH), then
        the candidates against the live envelope. margin > 0 shrinks the region inward
        (SR-16 E4, the horizontal TCP calibration error): the target is accepted only
        if every point of margin_points() around it is in the region too; else
        OUT_OF_REACH. A gate, never a projection - the target is never moved."""
        r = self.in_region(x, y)
        if not r.ok:
            return r
        for px, py in margin_points(x, y, margin, margin_step):
            m = self.in_region(px, py)
            if not m.ok:
                return AimResult(False, OUT_OF_REACH,
                                 f'({x:+.3f}, {y:+.3f}) is within the inward margin '
                                 f'{margin * 1e3:.0f} mm of the allowed region\'s edge: '
                                 f'({px:+.3f}, {py:+.3f}) is {m.detail}')
        return self.candidates(x, y, envelope, limit)

    # ---------------------------------------------------------------- the grid
    def reach_radius(self) -> float:
        """Upper bound of the horizontal distance of the TCP from the yaw axis."""
        s = self.kin._ik
        return float(np.hypot(*s.c2) + s.L2 + s.L3 + np.hypot(*s.wrist) + abs(s.d))

    def fingerprint(self, *extra: str) -> str:
        """Ties a region to the model and configuration it came from."""
        h = hashlib.sha256()
        for part in (repr(sorted(self.grip_q.items())), repr(sorted(self.hold.items())),
                     repr(self.roll_offsets), repr(self.tilt), repr(self.flt.cfg),
                     repr(sorted(self.kin.limits.items())),
                     repr((self.pregrasp_height, self.descent_step,
                           self.descent_max_lateral)), *extra):
            h.update(part.encode())
        return h.hexdigest()[:12]

    def derive_region(self, step: float, should_stop=None, pause=None,
                      fingerprint_extra: Sequence[str] = ()) -> Optional[Region]:
        """in_region on a grid of `step` over the reach disc around the yaw axis.
        should_stop(): abandon (returns None). pause(): called between grid points
        (the node yields the interpreter to its own threads there)."""
        o = self.kin._ik.o1
        r = self.reach_radius()
        n = int(math.ceil(r / step))
        xs = np.round(o[0] + step * np.arange(-n, n + 1), 6)
        ys = np.round(o[1] + step * np.arange(-n, n + 1), 6)
        mask = np.zeros((len(xs), len(ys)), dtype=bool)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                if math.hypot(x - o[0], y - o[1]) > r:
                    continue
                if should_stop is not None and should_stop():
                    return None
                mask[i, j] = self.in_region(float(x), float(y)).ok
                if pause is not None:
                    pause()
        env = getattr(self.ref_envelope, 'description', 'wheels swept (filter default)')
        return Region(xs, ys, mask, step, self.z, self.tilt,
                      self.fingerprint(*fingerprint_extra), env)


def margin_points(x: float, y: float, margin: float,
                  step: float) -> List[Tuple[float, float]]:
    """The points the inward margin checks around (x, y): concentric rings at
    margin * k / K (k = 1..K, K = ceil(margin / step)), each sampled at most `step`
    apart along its arc (at least 8 points). Empty for margin <= 0. A sampled disc:
    a gap in the region narrower than `step` between two samples is not seen."""
    if not margin > 0.0:
        return []
    if not (math.isfinite(margin) and math.isfinite(step) and step > 0.0):
        raise ValueError(f'margin {margin} / step {step} not finite and > 0')
    rings = max(1, int(math.ceil(margin / step)))
    pts = []
    for k in range(1, rings + 1):
        r = margin * k / rings
        n = max(8, int(math.ceil(2.0 * math.pi * r / step)))
        for i in range(n):
            a = 2.0 * math.pi * i / n
            pts.append((x + r * math.cos(a), y + r * math.sin(a)))
    return pts


def standoff_proposal(region: Region, reachable=None) -> Dict[str, object]:
    """The grasp standoff (litter nominally at (s, 0) in base_link) that centres the
    region along x on the line y = 0, and the one whose largest disc inside the region
    is widest. `reachable(x, y)` (optional) narrows the region further, e.g. to the
    direct-path region. A PROPOSAL, computed from the model - not a measurement."""
    mask = region.mask.copy()
    if reachable is not None:
        for i, x in enumerate(region.xs):
            for j, y in enumerate(region.ys):
                if mask[i, j]:
                    mask[i, j] = bool(reachable(float(x), float(y)))
    j0 = int(np.argmin(np.abs(region.ys)))
    col = region.xs[mask[:, j0]]
    if not col.size:
        return {'centre_x_m': None, 'disc': None}
    centre = 0.5 * (float(col.min()) + float(col.max()))
    pts = np.array([(region.xs[i], region.ys[j]) for i, j in zip(*np.nonzero(~mask))])
    best = None
    for x in col:
        d = (float(np.min(np.hypot(pts[:, 0] - x, pts[:, 1]))) - region.step / 2.0
             if len(pts) else math.inf)
        if best is None or d > best[1]:
            best = (float(x), d)
    return {'x_range_at_y0_m': [float(col.min()), float(col.max())],
            'centre_x_m': round(centre, 3),
            'widest_disc_x_m': round(best[0], 3), 'widest_disc_radius_m': round(best[1], 3)}
