"""Joint-space path planner with the SR-17 safety filter as the validity check.
Pure Python + numpy, no ROS imports. User decision 2026-10-07 ("im Zweifel abbrechen"):

  1. the direct joint-linear path;
  2. joint-ordered detours: the joints moved one after another, in a small fixed set
     of orders;
  3. a bounded RRT-Connect in joint space (deterministic seed), the result shortened
     by shortcut smoothing;
  4. no path -> no motion, with the reason.

`direct_only=True` stops after stage 1 (SR-17 rule 11, user decision 2026-10-08,
option a: automatically triggered motions take the direct path or none).

`recovery=True` (user decision 2026-10-09; the go-home motions only): the START may lie
outside the joint limits. Every segment is then checked with
SafetyFilter.check_setpoints(recovery=True): a joint outside moves only back toward
its limit and stays inside once there; a joint inside stays inside. Segment by segment
this holds for the whole path, because every waypoint was reached by an approved
segment. Every other check, and the goal's limit check, is unchanged; RRT samples and
detour waypoints lie inside the limits. PlanResult.recovery names the joints outside.

`separate_self=True` (user decision 2026-10-09, option C, arm self pairs only; the
operator's go_home only): a start whose ARM SELF pairs are in contact is accepted; the
direct and joint-ordered segments are checked with
SafetyFilter.check_setpoints(separate_self=True) - each such pair separates
monotonically until it passes the normal self check. The RRT and its smoothing keep the
normal checks: a tree node is fully valid. PlanResult.self_contact names the pairs.

`rise_ground=True` (user decision 2026-10-12; the operator's go_home only): a start
that fails the ground check for a segment's first step (grown, as sub-step 0 is
checked) is accepted - since 2026-10-15 also at or below the ground (user decision:
"as long as recovering just pulls upward"). A segment from the START is checked
with SafetyFilter.check_setpoints(rise_ground=True); it reports the links still rising
at its end, and a segment continuing from that node is checked with rising=those
links - so a path rises monotonically from the start, one joint at a time, until it is
clear, and no node later in the path can open a new recovery. The direct and RRT
candidates move several joints at once and are refused while a link is rising (audit
r11 F2); the joint-ordered detours lift with one joint first. PlanResult.ground names
the trigger. A start statically inside the margin that no detour's first joint lifts is
refused at once (bound 'ground', each first joint's verdict in the reason): every
RRT-Connect edge from the start moves several joints, so the RRT cannot start the lift.

Every segment of a returned path has passed the safety filter and the optional
`edge_check` (the caller's executability test, e.g. the servo tick wrap): "path
found" means "executable as planned". With `exec_steps` (the caller's executor:
the setpoints it will WRITE for a segment, gripperx_arm.exec_steps) the filter checks
exactly those setpoints and the motion between them (SafetyFilter.check_setpoints) -
the first one being the start as the executor rounds it - not the ideal joint-linear
line: whole servo ticks put an executed setpoint up to half a tick per joint off that
line, and a segment grazing an obstacle at exactly the clearance was approved on the
line and then rejected (or, unchecked, executed) a step later (user decision
2026-10-09). Without it: SafetyFilter.check_path, the ideal line. `exec_first` (same
signature): the setpoints for a segment that leaves the plan's START - a soft start
(user decision 2026-10-20, gripperx_arm.exec_steps.soft_start_fractions), whose small
first steps cost a small motion bound at the start; every other segment keeps
`exec_steps`, and so does a start segment that lifts out of the ground margin
(rise_ground, Planner.executed). Whether a segment leaves the start is a function of
its start pose, so the memoised verdicts stay deterministic.
Only the joints in `joints` are planned; the others (the gripper) keep their start
value along the whole path.

Bounds. `time_budget_s` is a HARD wall-clock cap on the whole plan (all stages,
smoothing included): past it no further pose is checked (SafetyFilter deadline) and
the plan fails, or - during smoothing only - returns the already valid path. The
overrun is at most one pose check. `max_checks` bounds the work deterministically
(same inputs -> same path) as long as the time cap does not bind; PlanResult.bound
says which one ended a failed search. `cancel()` (from another thread) ends a running
plan the same way, at its next check, with bound 'cancelled'.

Resuming. Segment verdicts are deterministic, so one Planner remembers them while
start pose and envelope stay the same: plan() again after a 'time' failure replays
the checks already done at no cost and continues the same search where the cap cut
it (`fresh_checks` = checks actually computed by the last call). Replayed checks
still count toward max_checks, which therefore bounds the resumed search as a whole.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from gripperx_arm.safety_filter import SafetyFilter, describe_outside

DEFAULT_ORDERS = (
    (0, 1, 2, 3, 4),      # base to tip
    (4, 3, 2, 1, 0),      # tip to base
    (2, 1, 3, 0, 4),      # elbow first: unfold before swinging
    (1, 2, 3, 0, 4),
    (3, 2, 1, 0, 4),
)

# edge_check(qa, qb) -> None if the segment qa -> qb is executable, else the reason
EdgeCheck = Callable[[Mapping[str, float], Mapping[str, float]], Optional[str]]
# exec_steps(qa, qb, direct) -> the poses the executor passes through for qa -> qb:
# [start as written, setpoint 1, ..., setpoint n]. `direct` = the plan's direct segment
# (a timed executor may run it at another speed than a detour). Must depend on its
# arguments only (bind calibration and speeds at construction): verdicts are memoised.
ExecSteps = Callable[[Mapping[str, float], Mapping[str, float], bool], List[Dict[str, float]]]


class _OutOfTime(Exception):
    pass


@dataclass
class PlanResult:
    ok: bool
    path: List[Dict[str, float]] = field(default_factory=list)   # start ... goal
    method: str = ''            # direct | ordered | rrt
    reason: str = ''
    cost_ms: float = 0.0
    checks: int = 0
    bound: str = ''             # '' | time | checks | direct_only | ground: what ended it
    recovery: str = ''          # recovery plan: the start's joints outside their limits
    self_contact: str = ''      # separate_self plan: the start's arm pairs in contact
    ground: str = ''            # rise_ground plan: what triggered the ground recovery
    ground_start: Dict[str, float] = field(default_factory=dict)   # its static clearances
    soft_first: bool = False    # set by the caller: first segment validated soft-started


class Planner:
    def __init__(self, flt: SafetyFilter, joints: Sequence[str],
                 time_budget_s: float, rrt_step_rad: float = 0.25,
                 seed: int = 0, shortcut_rounds: int = 60, max_checks: int = 600,
                 edge_check: Optional[EdgeCheck] = None, direct_only: bool = False,
                 exec_steps: Optional[ExecSteps] = None, recovery: bool = False,
                 separate_self: bool = False, rise_ground: bool = False,
                 sample_band: Optional[Mapping[str, Tuple[float, float]]] = None,
                 exec_first: Optional[ExecSteps] = None):
        """`sample_band` (per joint (lo, hi), within the joint limits) narrows where RRT
        samples are drawn - the leader command band (gripperx_arm.leader_band): a
        leader re-plan's waypoints then keep its margin from the limits, as the
        leader's direct steps do. Start and goal checks keep the joint limits.
        max_checks bounds the search stages (direct, ordered, RRT) together;
        smoothing may add up to shortcut_rounds checks on top. time_budget_s caps
        everything (module docstring)."""
        self.f, self.joints = flt, list(joints)
        self.budget, self.step = time_budget_s, rrt_step_rad
        self.max_checks = max_checks
        self.seed, self.shortcut_rounds = seed, shortcut_rounds
        self.edge_check = edge_check
        self.exec_steps = exec_steps
        self.exec_first = exec_first
        self.direct_only = bool(direct_only)
        self.recovery = bool(recovery)
        self.separate_self = bool(separate_self)
        self.rise_ground = bool(rise_ground)
        self.lo = np.array([flt.kin.limits[n][0] for n in self.joints])
        self.hi = np.array([flt.kin.limits[n][1] for n in self.joints])
        band = sample_band or {}
        self.s_lo = np.array([max(band.get(n, lim)[0], lim[0]) for n, lim in
                              ((n, flt.kin.limits[n]) for n in self.joints)])
        self.s_hi = np.array([min(band.get(n, lim)[1], lim[1]) for n, lim in
                              ((n, flt.kin.limits[n]) for n in self.joints)])
        self._cancelled = False                # bound 'cancelled' (cancel())
        self._memo: Dict[tuple, tuple] = {}    # (a, b, direct) -> (ok, verdict, edge reason)
        self._memo_inputs = None
        self.fresh_checks = 0
        self.fresh_poses = 0                   # poses a cut check got past in this plan()
        self._resume: Dict[tuple, int] = {}    # memo key -> where a cut check continues
        self._cut_at = 0                       # the last cut verdict's resume_at

    # ------------------------------------------------------------------ helpers
    def _q(self, v, base) -> Dict[str, float]:
        q = dict(base)
        q.update({n: float(x) for n, x in zip(self.joints, v)})
        return q

    def _v(self, q) -> np.ndarray:
        return np.array([q[n] for n in self.joints], float)

    def cancel(self):
        """Thread-safe: a running plan() ends at its next pose check (overrun at most
        one pose), a later one at once; either returns bound 'cancelled'."""
        self._cancelled = True

    def _rising_in(self, a) -> object:
        """rise_ground: 'start' for a segment from the plan's start, else the links
        still rising at node `a` (() = checked normally)."""
        if not self.rise_ground:
            return ()
        if a.tobytes() == self._start_key:
            return 'start'
        return self._rising.get(a.tobytes(), ())

    def _seg_ok(self, a, b, base, env, direct: bool = False, sep: bool = False) -> bool:
        # exec_steps and edge_check are fixed per Planner and the memo is dropped when
        # start pose or envelope change: (a, b, direct, sep, rising at a) then
        # determines the verdict.
        rin = self._rising_in(a)
        key = (a.tobytes(), b.tobytes(), direct, sep, rin)
        hit = self._memo.get(key)
        if hit is None:
            if self._cancelled or time.monotonic() > self._deadline:
                raise _OutOfTime()
            done = self._resume.get(key, 0)
            try:
                hit = self._check(a, b, base, env, direct, sep, rin, done)
            except _OutOfTime:             # a slice ended inside it: the next one goes on
                self._resume[key] = self._cut_at
                self.fresh_poses += max(0, self._cut_at - done)
                raise
            self._resume.pop(key, None)
            self._memo[key] = hit
            self.fresh_checks += 1
        self._checks += 1
        ok, verdict, edge, trigger, rising_end = hit
        self._last_verdict = verdict
        if trigger and not self._ground_text:
            self._ground_text = self._describe_trigger(trigger)
        if ok and rising_end:
            self._rising[b.tobytes()] = rising_end
        if edge:
            self._edge_rejects += 1
            self._edge_last = edge
        if self._first_verdict is None:
            self._first_verdict = verdict
        return ok

    def _describe_trigger(self, trigger) -> str:
        return self.f.describe_trigger(trigger)

    def _check(self, a, b, base, env, direct, sep=False, rin=(), resume_from=0) -> tuple:
        res = self._verdict(self._q(a, base), self._q(b, base), env, direct, self._deadline,
                            sep, rin, first=a.tobytes() == self._start_key,
                            resume_from=resume_from)
        if res[0] is None:
            raise _OutOfTime()
        return res

    def executed(self, qa, qb, direct: bool = False, first: bool = False
                 ) -> List[Dict[str, float]]:
        """The poses the executor passes through for the segment qa -> qb - `first`: a
        segment leaving the plan's start (exec_first) - as the planner checks them.
        With rise_ground, a segment whose start needs the ground recovery takes the
        plain steps (_lifts): no soft start while lifting out of the margin."""
        if first and self.starts_soft(qa, qb, direct):
            return self.exec_first(qa, qb, direct)
        return self.exec_steps(qa, qb, direct)

    def starts_soft(self, qa, qb, direct: bool = False) -> bool:
        """Does a segment qa -> qb leaving the plan's start take the soft start
        (exec_first) - i.e. what executed(first=True) validates for it? Not a segment
        that moves no planned joint: its soft setpoints would all repeat the start and
        be dropped, so nothing would be written where the plain ramp holds the pose."""
        if all(qa[n] == qb[n] for n in self.joints):
            return False
        return self.exec_first is not None and not self._lifts(qa, qb, direct)

    def _lifts(self, qa, qb, direct) -> bool:
        """rise_ground: does qa fail the ground check grown for the largest step of the
        PLAIN executor (exec_steps) toward qb? Then the segment lifts out of the margin
        exactly as without a soft start - full steps, one joint at a time, from the
        first setpoint on. A soft start there would end the lift after its first small
        step (the link passes the check grown for two tiny adjacent steps) and fail a
        few setpoints later, where no new lift may open (user decision 2026-10-12). The
        soft start is for a start clear of the margin for a full step, which then
        cannot fail at its start; by its arguments only, so verdicts stay memoisable."""
        if not self.rise_ground:
            return False
        poses = self.exec_steps(qa, qb, direct)
        if len(poses) < 2:
            return False
        names = list(poses[0])
        dq = {n: max(abs(b[n] - a[n]) for a, b in zip(poses, poses[1:])) for n in names}
        return bool(self.f.ground_shortfalls(qa, dq))

    def _verdict(self, qa, qb, env, direct, deadline, sep=False, rin=(), first=False,
                 trace=None, resume_from=0) -> tuple:
        """(ok, verdict text, edge reason, ground trigger, links still rising at qb);
        ok None = cut by the deadline."""
        if self.edge_check is not None:
            why = self.edge_check(qa, qb)
            if why:
                return False, f'not executable: {why}', why, {}, ()
        kw = {'recovery': True} if self.recovery else {}
        if sep:
            kw['separate_self'] = True
        if rin == 'start':
            kw['rise_ground'] = True
        elif rin:
            kw['rising'] = rin
        if self.exec_steps is None:
            v = self.f.check_path(qa, qb, env, deadline, **kw)
        else:
            if trace is not None:
                kw['trace'] = trace
            v = self.f.check_setpoints(self.executed(qa, qb, direct, first), env, deadline,
                                       stop=lambda: self._cancelled, resume_from=resume_from,
                                       **kw)
        if v.rule == 'timeout':
            self._cut_at = 0 if self.exec_steps is None else v.resume_at  # check_path restarts
            return None, str(v), '', {}, ()
        return v.ok, str(v), '', dict(v.rising_start), tuple(v.rising_end)

    def check_segment(self, qa: Mapping[str, float], qb: Mapping[str, float],
                      envelope=None, direct: bool = False, first: bool = False,
                      rising: Optional[Sequence[str]] = None,
                      trace: Optional[list] = None) -> Optional[str]:
        """The planner's own segment test, outside a plan (no memo, no time cap): None
        if qa -> qb is executable and clear as this planner would approve it, else the
        reason. For re-validating a planned path from where the arm IS. `first`: the
        segment leaves the path's start (exec_first; with rise_ground it may open the
        ground recovery there). `rising`: for a later segment, the entries still rising
        at qa (the previous segment's trace[-1]); never opens a new recovery. `trace`:
        SafetyFilter.check_setpoints trace (per setpoint, the entries still rising)."""
        if rising:
            rin = tuple(rising)
        else:
            rin = 'start' if self.rise_ground and first else ()
        ok, text = self._verdict(dict(qa), dict(qb), envelope, direct, None,
                                 self.separate_self, rin, first, trace)[:2]
        return None if ok else text

    # ------------------------------------------------------------------ plan
    def plan(self, qa: Dict[str, float], qb: Dict[str, float], envelope=None) -> PlanResult:
        t0 = time.monotonic()
        self._deadline = t0 + self.budget
        self._checks = self._edge_rejects = 0
        self._edge_last = ''
        self._first_verdict = None
        base = dict(qa)
        inputs = tuple(sorted(base.items()))
        if (self._memo_inputs is None or self._memo_inputs[0] != inputs
                or self._memo_inputs[1] is not envelope):
            self._memo = {}
            self._resume = {}
            self._rising = {}                   # node -> links still rising there
        self._memo_inputs = (inputs, envelope)
        self.fresh_checks = self.fresh_poses = 0
        a, b = self._v(qa), self._v(qb)
        self._start_key = a.tobytes()
        outside = self.f.outside_limits(base) if self.recovery else {}
        self._contact = self._ground_text = ''
        self._static_low: Dict[str, float] = {}     # rise_ground: start statically in the margin
        self._first_fail: Dict[str, str] = {}       # ... first joint of a detour -> its verdict
        try:
            res = self._plan(a, b, base, envelope)
        except _OutOfTime:
            res = (PlanResult(False, bound='cancelled', reason='search cancelled')
                   if self._cancelled else PlanResult(False, bound='time'))
        if outside:
            res.recovery = describe_outside(outside)
        res.self_contact = self._contact
        if res.ok and len(res.path) > 1:          # the trigger of the path taken
            hit = self._memo.get((res.path[0].tobytes(), res.path[1].tobytes(),
                                  res.method == 'direct', self.separate_self and
                                  res.method != 'rrt', self._rising_in(res.path[0])))
            if hit is not None:
                self._ground_text = self._describe_trigger(hit[3]) if hit[3] else ''
                res.ground_start = {n: s for n, (s, _g) in hit[3].items()}
        res.ground = self._ground_text
        if not res.ok:
            res.reason = self._why(res)
        res.cost_ms = (time.monotonic() - t0) * 1e3
        res.checks = self._checks
        if res.ok:
            res.path = [self._q(v, base) for v in res.path]
        return res

    def _why(self, res: PlanResult) -> str:
        if res.reason:                      # start/goal refused
            return res.reason
        if res.bound == 'direct_only':
            text = ('direct path blocked and detours are not allowed for this motion '
                    '(SR-17 rule 11: automatically triggered motions take the direct path '
                    f'only); direct path: {self._first_verdict or "not checked"}')
            if self._edge_rejects:
                text += f'; not executable: {self._edge_last}'
            return text
        if res.bound == 'ground':
            tried = '; '.join(f'{j} first: {v}' for j, v in self._first_fail.items())
            return (f'ground recovery from {self.f.describe_ground(self._static_low)}: no '
                    'first section lifts it - while a link rises only one joint may move, '
                    'and no joint moved first toward the goal raises every such link '
                    'strictly (direct path and joint-ordered detours tried; RRT-Connect '
                    f'cannot start with one joint); direct path: {self._first_verdict}; '
                    f'{tried}')
        what = {'time': f'time budget {self.budget:.2f} s exhausted',
                'checks': f'{self.max_checks} checks used up'}.get(res.bound, 'no path')
        text = (f'no path found ({what}; direct, joint-ordered detours, RRT-Connect); '
                f'direct path: {self._first_verdict or "not checked"}')
        if self._edge_rejects:
            text += (f'; {self._edge_rejects} segment(s) not executable '
                     f'(last: {self._edge_last})')
        return text

    def _plan(self, a, b, base, env) -> PlanResult:
        for name, v in (('start', a), ('goal', b)):
            q = self._q(v, base)
            recover = self.recovery and name == 'start'
            bad = [n for n, x, lo, hi in zip(self.joints, v, self.lo, self.hi)
                   if not lo - 1e-9 <= x <= hi + 1e-9]
            if bad and not recover:
                return PlanResult(False, reason=f'{name} pose outside the joint limits: {bad}')
            if self._cancelled or time.monotonic() > self._deadline:
                raise _OutOfTime()
            lim = None
            if self.recovery:
                # the start's own values bound it; the goal keeps the planned joints'
                # limits, and the others (the gripper) keep their start value along
                # the path. Collisions refuse either as before.
                lim = self.f.recovery_limits(base)
                if not recover:
                    lim.update({j: self.f.kin.limits[j] for j in self.joints})
            skip = None
            if self.rise_ground and name == 'start':
                self._static_low = self.f.ground_shortfalls(q)
                skip = set(self._static_low) or None     # static; the segments decide
            if self.separate_self and name == 'start':
                verdict, hits = self.f.self_contacts(q, env, limits=lim, ground_skip=skip)
                if hits:
                    self._contact = self.f.describe_contacts(q, hits)
                    continue
            else:
                verdict = self.f.check_pose(q, env, limits=lim, ground_skip=skip)
            if not verdict.ok:
                return PlanResult(False, reason=f'{name} pose rejected: {verdict}')
        sep = self.separate_self
        if self._seg_ok(a, b, base, env, direct=True, sep=sep):
            return PlanResult(True, [a, b], 'direct')
        if self.direct_only:
            return PlanResult(False, bound='direct_only')
        for order in DEFAULT_ORDERS:
            pts, cur = [a], a.copy()
            ok = True
            for i in order:
                if i >= len(self.joints) or cur[i] == b[i]:
                    continue
                if self._checks >= self.max_checks:
                    return PlanResult(False, bound='checks')
                nxt = cur.copy()
                nxt[i] = b[i]
                if not self._seg_ok(cur, nxt, base, env, sep=sep):
                    if np.array_equal(cur, a):
                        self._first_fail.setdefault(self.joints[i], self._last_verdict)
                    ok = False
                    break
                pts.append(nxt)
                cur = nxt
            if ok and np.allclose(cur, b):
                return PlanResult(True, pts, 'ordered')
        if self._static_low:
            # every RRT-Connect edge from the start moves several joints, refused while
            # a link rises: no search left that can start the lift
            return PlanResult(False, bound='ground')
        return self._rrt(a, b, base, env)

    # ------------------------------------------------------------------ RRT-Connect
    def _rrt(self, a, b, base, env) -> PlanResult:
        rng = np.random.default_rng(self.seed)
        trees = [([a], [-1]), ([b], [-1])]

        def nearest(tree, x):
            pts = np.array(tree[0])
            return int(np.argmin(np.abs(pts - x).max(axis=1)))

        def steer(x, y):
            d = y - x
            m = np.abs(d).max()
            return y if m <= self.step else x + d * (self.step / m)

        def extend(tree, x):
            i = nearest(tree, x)
            y = steer(tree[0][i], x)
            if self._seg_ok(tree[0][i], y, base, env):
                tree[0].append(y)
                tree[1].append(i)
                return len(tree[0]) - 1, np.allclose(y, x)
            return None, False

        def path_to(tree, i):
            out = []
            while i != -1:
                out.append(tree[0][i])
                i = tree[1][i]
            return out

        def alive():
            return self._checks < self.max_checks

        while alive():
            x = self.s_lo + rng.random(len(self.joints)) * (self.s_hi - self.s_lo)
            i, _ = extend(trees[0], x)
            if i is not None:
                target = trees[0][0][i]
                while alive():
                    j, reached = extend(trees[1], target)
                    if j is None:
                        break
                    if reached:
                        n0, n1 = path_to(trees[0], i), path_to(trees[1], j)
                        if trees[0][0][0] is a:         # n0 ends at a, n1 ends at b
                            path = n0[::-1] + n1[1:]
                        else:                           # n0 ends at b, n1 ends at a
                            path = n1[::-1] + n0[1:]
                        return PlanResult(True, self._shortcut(path, base, env, rng), 'rrt')
            trees.reverse()
        return PlanResult(False, bound='checks')

    def _shortcut(self, path, base, env, rng) -> List[np.ndarray]:
        """Every segment of `path` is valid already; the time cap ends smoothing with
        the path as it stands."""
        path = list(path)
        limit = self._checks + self.shortcut_rounds
        try:
            for _ in range(self.shortcut_rounds):
                if len(path) < 3 or self._checks >= limit:
                    break
                i, j = sorted(rng.choice(len(path), 2, replace=False))
                if j - i < 2:
                    continue
                if self._seg_ok(path[i], path[j], base, env):
                    path = path[:i + 1] + path[j:]
        except _OutOfTime:
            pass
        return path
