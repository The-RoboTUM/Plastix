"""Pure bookkeeping for record_limits: min/max per joint, named poses, YAML text.

Joints are tracked by name in the fixed output order limb1..limb5, gripper. A joint
the source never reported (e.g. the leader gripper without a cable) stays None and
is written as `null`, never as 0. Min/max are wrap-aware (see _Track): a range that
crosses 0/4095 is written with min > max and flagged in a comment.
"""

from typing import Dict, List, Optional, Sequence

JOINT_ORDER = ('limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper')


import math

TICKS = 4096
# A sample-to-sample step above this is flagged as a gap (stream dropout or a joint moved
# implausibly fast at the stream rate). Named, TO-VERIFY.
STEP_FLAG_RAD = math.radians(90.0)
# An unreachable gap (360 deg - travel) below this is flagged: the wrap representation
# then has little margin. Named, TO-VERIFY.
GAP_WARN_RAD = math.radians(30.0)


class _RadTrack:
    """Continuity unwrap of radian samples: the first sample defines the branch (the
    value as received, i.e. inside the node's current window), every next sample is
    the previous unwrapped value plus the step wrapped into [-pi, pi). Min/max are
    exact and may exceed +-pi."""

    def __init__(self, n):
        self.cur = [None] * n
        self.lo = [None] * n
        self.hi = [None] * n
        self.big_steps = [0] * n

    def fold(self, i, rad):
        if self.cur[i] is None:
            self.cur[i] = rad
        else:
            step = (rad - self.cur[i] + math.pi) % (2 * math.pi) - math.pi
            if abs(step) > STEP_FLAG_RAD:
                self.big_steps[i] += 1
            self.cur[i] += step
        v = self.cur[i]
        self.lo[i] = v if self.lo[i] is None else min(self.lo[i], v)
        self.hi[i] = v if self.hi[i] is None else max(self.hi[i], v)


def _wrap(delta: int) -> int:
    return (delta + TICKS // 2) % TICKS - TICKS // 2


class _Track:
    """Wrap-aware min/max of one source. Deltas are kept relative to a reference
    tick (the calibration zero if given, else the first sample), wrapped into
    [-2048, 2048): a joint crossing 0/4095 is NOT mistaken for a full-range sweep.
    ASSUMPTION: travel around the reference is < 360 deg."""

    def __init__(self, n, refs=None):
        self.ref = list(refs) if refs else [None] * n
        self.lo = [None] * n
        self.hi = [None] * n
        self.last = [None] * n

    def fold(self, i, t):
        self.last[i] = t
        if self.ref[i] is None:
            self.ref[i] = t
        d = _wrap(t - self.ref[i])
        self.lo[i] = d if self.lo[i] is None else min(self.lo[i], d)
        self.hi[i] = d if self.hi[i] is None else max(self.hi[i], d)

    def mins(self):
        return [None if v is None else (r + v) % TICKS for r, v in zip(self.ref, self.lo)]

    def maxs(self):
        return [None if v is None else (r + v) % TICKS for r, v in zip(self.ref, self.hi)]


class LimitsRecorder:
    def __init__(self, n_joints: int = 6, leader_zero: Optional[Sequence[Optional[int]]] = None,
                 calibration=None) -> None:
        self._rad = _RadTrack(n_joints)
        self.cal = calibration       # full 6-entry Calibration, for rad -> ticks of the output
        self.order = JOINT_ORDER[:n_joints]
        self._leader = _Track(n_joints, leader_zero)
        self._follower = _Track(n_joints)
        self.poses: Dict[str, List[Optional[int]]] = {}
        self.seen_leader = False
        self.seen_follower = False

    leader_min = property(lambda self: self._leader.mins())
    leader_max = property(lambda self: self._leader.maxs())
    follower_min = property(lambda self: self._follower.mins())
    follower_max = property(lambda self: self._follower.maxs())
    last_leader = property(lambda self: list(self._leader.last))
    last_follower = property(lambda self: list(self._follower.last))

    def _fold(self, track, ticks, names):
        names = list(names) if names else list(self.order[:len(ticks)])
        for name, t in zip(names, ticks):
            if name in self.order:
                track.fold(self.order.index(name), int(round(t)) % TICKS)

    def add_leader(self, ticks: Sequence[float], names: Optional[Sequence[str]] = None) -> None:
        self.seen_leader = True
        self._fold(self._leader, ticks, names)

    def add_leader_rad(self, rad: Sequence[float], names: Optional[Sequence[str]] = None) -> None:
        names = list(names) if names else list(self.order[:len(rad)])
        for name, r in zip(names, rad):
            if name in self.order:
                self._rad.fold(self.order.index(name), float(r))

    def add_follower(self, ticks: Sequence[float], names: Optional[Sequence[str]] = None) -> None:
        self.seen_follower = True
        self._fold(self._follower, ticks, names)

    def snapshot_pose(self, name: str, source: str = 'leader') -> bool:
        sample = self.last_leader if source == 'leader' else self.last_follower
        if all(v is None for v in sample):
            return False
        self.poses[name] = list(sample)
        return True

    def _absent(self, lo) -> List[str]:
        return [n for n, v in zip(self.order, lo) if v is None]

    def to_yaml(self) -> str:
        out = ['# Recorded by gripperx_leader record_limits. Tick order: limb1..limb5, gripper.',
               '# Review before copying anything into gripperx_arm/config/arm_poses.yaml.',
               '# null = joint never reported (not recorded) -> TO-VERIFY, NOT zero.']

        def block(title, lo, hi, seen):
            if not seen:
                out.append(f'# {title}: no samples')
                return
            absent = self._absent(lo)
            if absent:
                out.append(f'# TO-VERIFY: no samples for {", ".join(absent)}')
            wrapped = [n for n, a_, b_ in zip(self.order, lo, hi)
                       if a_ is not None and a_ > b_]
            if wrapped:
                out.append(f'# NOTE: {", ".join(wrapped)}: min > max = the range CROSSES the '
                           '0/4095 wrap (travel runs from min up through 4095/0 to max)')
            out.append(f'{title}:')
            out.append(f'  min: {self._fmt(lo)}')
            out.append(f'  max: {self._fmt(hi)}')

        tmin, tmax = self.leader_min, self.leader_max
        if self.cal is not None and any(v is not None for v in self._rad.lo):
            tmin, tmax = self._ticks_from_rad()
        block('joint_limits', tmin, tmax, self.seen_leader)
        self._range_block(out)
        block('follower_joint_limits', self.follower_min, self.follower_max, self.seen_follower)
        out.append('poses:' if self.poses else '# poses: none recorded')
        for name, ticks in self.poses.items():
            out.append(f'  {name}: {self._fmt(ticks)}')
        return '\n'.join(out) + '\n'

    def _ticks_from_rad(self):
        """Leader tick limits from the continuous radian range (exact beyond +-180 deg)."""
        cal = self.cal
        lo, hi = list(self.leader_min), list(self.leader_max)
        for i, (a, b) in enumerate(zip(self._rad.lo, self._rad.hi)):
            if a is None or cal.zero_ticks[i] is None or cal.sign[i] is None:
                continue
            k = cal.ticks_per_rev / (2 * math.pi)
            ends = sorted(cal.zero_ticks[i] + cal.sign[i] * r * k for r in (a, b))
            lo[i], hi[i] = int(round(ends[0])) % TICKS, int(round(ends[1])) % TICKS
        return lo, hi

    def _range_block(self, out):
        r = self._rad
        if all(v is None for v in r.lo):
            return
        names = self.order
        deg = math.degrees

        def col(vals):
            return '[' + ', '.join('null' if v is None else f'{v:.4f}' for v in vals) + ']'
        mn, mx = r.lo, r.hi
        ctr = [None if a is None else (a + b) / 2 for a, b in zip(mn, mx)]
        gap = [None if a is None else 2 * math.pi - (b - a) for a, b in zip(mn, mx)]
        out.append('# leader_range: continuous (unwrapped by sample-to-sample continuity) radians, calibrated.')
        out.append('# suggested_range_center_rad -> calibration range_center_rad (TO-VERIFY before use).')
        for n_, a, b, g, big in zip(names, mn, mx, gap, r.big_steps):
            if a is None:
                continue
            note = f'# {n_}: {deg(a):.1f}..{deg(b):.1f} deg, unreachable gap {deg(g):.1f} deg'
            if g < 0:
                note += '  FLAG: travel exceeds 360 deg, window representation unusable'
            elif g < GAP_WARN_RAD:
                note += f'  FLAG: gap < {deg(GAP_WARN_RAD):.0f} deg (TO-VERIFY threshold), little margin'
            if big:
                note += f'  FLAG: {big} step(s) > {deg(STEP_FLAG_RAD):.0f} deg between samples (gap in stream?), limits may be wrong'
            out.append(note)
        out.append('leader_range:')
        out.append(f'  min_rad: {col(mn)}')
        out.append(f'  max_rad: {col(mx)}')
        out.append(f'  suggested_range_center_rad: {col(ctr)}')
        out.append(f'  unreachable_gap_rad: {col(gap)}')

    @staticmethod
    def _fmt(values) -> str:
        return '[' + ', '.join('null' if v is None else str(v) for v in values) + ']'
