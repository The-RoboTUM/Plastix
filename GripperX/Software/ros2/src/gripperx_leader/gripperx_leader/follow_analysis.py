"""Leader -> follower recording: CSV format and analysis (pure python, no ROS).

Written by record_follow (a passive subscriber), read by analyze_follow. One row per
received message (follow_debug: up to three rows per message):

  recv_s     receive time on the recording machine (time.time(), seconds)
  stamp_s    the message's header stamp (seconds; '' for String topics)
  source     leader (/arm/leader/joint_states) | follower (/joint_states)
             | setpoint / target / measured (/arm/leader/follow_debug: the setpoint after
               the tick, the clamped leader target, the positions the tick read)
             | wall_state (/arm/leader/wall_state) | leader_state (/arm/leader/state)
  limb1..limb5  radians, '' where the message lacks the joint
  tick_s, period_s  follow_debug (setpoint rows): tick duration and period, seconds
  refusal_s .. other_s, check_poses  follow_debug (setpoint rows, 2026-10-16): time per
             tick phase (PHASES, seconds) and the poses the safety check checked; empty
             in recordings of an arm_action_server without them
  text       String payload, or the follow_debug outcome (setpoint rows)

Analysis. Where it comes from decides what it means:
  - follow_debug (arm_action_server, once per leader tick, Pi clock stamps): tick
    duration and period (the Pi tick time) against 1 / leader.rate_hz, split into the
    tick phases (mean / p95 / max each) with the slowest ticks listed; per joint the
    setpoint velocity and step per tick against leader.max_joint_velocity_rad_s (direct
    steps of consecutive ticks only, see setpoint_steps); the lag
    leader -> setpoint (the rate limit and transport) and setpoint -> measured (the
    servo), the split that says what limits the follow speed.
  - /joint_states (follower): arm_joint_state_relay's copy of arm_action_server's 2 Hz
    timer, NOT the tick - its intervals are not the tick time and velocities between
    0.5 s samples are averages. Kept for recordings without follow_debug.
Lags: the shift s with the smallest RMS error of out(t) against ref(t - s) (ref linearly
interpolated; the correlation there is reported too). Leader -> anything uses receive
times by default (both arrive on the recording machine, no laptop/Pi clock offset), so
network delay is part of it; setpoint -> measured uses
the Pi header stamps of follow_debug (one clock). The measured positions are read at the
START of a tick and the setpoint written at its end, so the servo lag includes up to one
tick of sampling.
"""

import csv
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

JOINTS = ['limb1', 'limb2', 'limb3', 'limb4', 'limb5']
# LeaderFollowDebug <phase>_s fields, in tick order (other = the rest of tick_s)
PHASES = ['refusal', 'read', 'envelope', 'replan', 'target', 'check', 'write', 'other']
PHASE_COLUMNS = [f'{p}_s' for p in PHASES]
COLUMNS = (['recv_s', 'stamp_s', 'source'] + JOINTS + ['tick_s', 'period_s']
           + PHASE_COLUMNS + ['check_poses', 'text'])
SLOWEST = 5               # slowest ticks listed with their phases
LEADER, FOLLOWER, WALL_STATE, LEADER_STATE = 'leader', 'follower', 'wall_state', 'leader_state'
SETPOINT, TARGET, MEASURED = 'setpoint', 'target', 'measured'
SLOW_SAMPLING_S = 0.1     # follower intervals above this: velocity / tick-time caveat


def joint_index(name: str) -> Optional[int]:
    """limb1..limb5 for the leader's names ('limb1') and the follower's ('arm_limb1_joint')."""
    if name.startswith('arm_') and name.endswith('_joint'):
        name = name[4:-6]
    return JOINTS.index(name) if name in JOINTS else None


def joint_row(recv: float, stamp: float, source: str, names: Sequence[str],
              positions: Sequence[float]) -> Dict[str, object]:
    """A row of joint positions; names of other joints are ignored."""
    row: Dict[str, object] = {'recv_s': recv, 'stamp_s': stamp, 'source': source}
    for n, p in zip(names, positions):
        i = joint_index(n)
        if i is not None:
            row[JOINTS[i]] = float(p)
    return row


def debug_rows(recv: float, stamp: float, names: Sequence[str], setpoint: Sequence[float],
               target: Sequence[float], measured: Sequence[float], tick_s: float,
               period_s: float, outcome: str, phases: Optional[Dict[str, float]] = None,
               check_poses: Optional[int] = None) -> List[Dict[str, object]]:
    """The rows of one follow_debug message; empty arrays give no row. `phases`:
    {phase: seconds} for PHASES (None: a message without them)."""
    sp = joint_row(recv, stamp, SETPOINT, names, setpoint)
    sp.update({'tick_s': float(tick_s), 'period_s': float(period_s), 'text': outcome})
    for p, v in (phases or {}).items():
        if p in PHASES:
            sp[f'{p}_s'] = float(v)
    if check_poses is not None:
        sp['check_poses'] = int(check_poses)
    rows = [sp]
    for source, q in ((TARGET, target), (MEASURED, measured)):
        if len(q):
            rows.append(joint_row(recv, stamp, source, names, q))
    return rows


def text_row(recv: float, source: str, text: str) -> Dict[str, object]:
    return {'recv_s': recv, 'stamp_s': '', 'source': source, 'text': text}


class CsvWriter:
    """Writes rows in COLUMNS order, floats with 6 decimals."""

    def __init__(self, fh):
        self._fh = fh
        self._w = csv.DictWriter(fh, fieldnames=COLUMNS, restval='')
        self._w.writeheader()

    def write(self, row: Dict[str, object]):
        self._w.writerow({k: (f'{v:.6f}' if isinstance(v, float) else v)
                          for k, v in row.items()})

    def flush(self):
        self._fh.flush()


def read_csv(path: str) -> List[Dict[str, object]]:
    def num(r, k):
        v = r.get(k)
        return float(v) if v else None
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            row: Dict[str, object] = {'recv_s': float(r['recv_s']), 'source': r['source'],
                                      'stamp_s': num(r, 'stamp_s'), 'text': r.get('text', ''),
                                      'tick_s': num(r, 'tick_s'), 'period_s': num(r, 'period_s')}
            for k in PHASE_COLUMNS + ['check_poses']:
                row[k] = num(r, k)
            for j in JOINTS:
                row[j] = num(r, j)
            rows.append(row)
    return rows


# ------------------------------------------------------------------ helpers
def _series(rows, source: str, joint: str, time_key: str) -> List[Tuple[float, float]]:
    out = []
    for r in rows:
        if r['source'] != source or r.get(joint) is None:
            continue
        t = r.get(time_key)
        if t is None or t == 0.0:          # no header stamp: fall back to receive time
            t = r['recv_s']
        out.append((float(t), float(r[joint])))
    out.sort()
    return out


def _times(rows, source: str, key: str) -> List[float]:
    ts = []
    for r in rows:
        if r['source'] == source:
            t = r.get(key)
            if t:
                ts.append(float(t))
    return sorted(ts)


def stats(values: Sequence[float]) -> Optional[Dict[str, float]]:
    """n, mean, p95 (nearest rank), max; None when empty."""
    d = sorted(values)
    if not d:
        return None
    p95 = d[min(len(d) - 1, int(math.ceil(0.95 * len(d))) - 1)]
    return {'n': len(d), 'mean_s': sum(d) / len(d), 'p95_s': p95, 'max_s': d[-1]}


def interval_stats(ts: Sequence[float]) -> Optional[Dict[str, float]]:
    return stats([b - a for a, b in zip(ts, ts[1:])])


def _interp(series: Sequence[Tuple[float, float]], t: float,
            max_gap_s: float) -> Optional[float]:
    """Linear interpolation; None outside the series or across a gap (not streaming)."""
    lo, hi = 0, len(series) - 1
    if hi < 1 or t < series[0][0] or t > series[-1][0]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (t0, q0), (t1, q1) = series[lo], series[hi]
    if t1 - t0 > max_gap_s:
        return None
    return q0 if t1 == t0 else q0 + (q1 - q0) * (t - t0) / (t1 - t0)


def _pearson(pairs: Sequence[Tuple[float, float]]) -> Optional[float]:
    n = len(pairs)
    if n < 3:
        return None
    mx = sum(a for a, _ in pairs) / n
    my = sum(b for _, b in pairs) / n
    sxx = sum((a - mx) ** 2 for a, _ in pairs)
    syy = sum((b - my) ** 2 for _, b in pairs)
    if sxx <= 0.0 or syy <= 0.0:
        return None
    return sum((a - mx) * (b - my) for a, b in pairs) / math.sqrt(sxx * syy)


def _max_vel(series: Sequence[Tuple[float, float]]) -> Optional[float]:
    v = [abs(q1 - q0) / (t1 - t0) for (t0, q0), (t1, q1) in zip(series, series[1:])
         if t1 > t0]
    return max(v) if v else None


def setpoint_steps(rows, joint: str, max_gap_s: Optional[float]) -> dict:
    """Per-tick setpoint steps of `joint` on the Pi stamps: only pairs of consecutive
    follow_debug samples whose later tick WROTE a rate-limited direct step ('written')
    and that are at most `max_gap_s` apart. A larger gap means a lost (best effort)
    message or leader mode off in between (a re-activation starts from the measured
    pose): such a pair would span several ticks or two sessions and is skipped,
    counted in `skipped_gap`. Planned path steps ('follow_step') follow the planner,
    not the leader limit: counted in `skipped_other`."""
    pts = []
    for r in rows:
        if r['source'] == SETPOINT and r.get(joint) is not None:
            t = r.get('stamp_s') or r['recv_s']
            pts.append((float(t), float(r[joint]), r.get('text') or ''))
    pts.sort()
    steps, vels, gap, other = [], [], 0, 0
    for (t0, q0, _), (t1, q1, outcome) in zip(pts, pts[1:]):
        if max_gap_s is not None and t1 - t0 > max_gap_s:
            gap += 1
            continue
        if outcome != 'written':
            if outcome != 'unchanged':
                other += 1
            continue
        steps.append(abs(q1 - q0))
        if t1 > t0:
            vels.append(abs(q1 - q0) / (t1 - t0))
    return {'max_step': max(steps) if steps else None, 'max_vel': max(vels) if vels else None,
            'n': len(steps), 'skipped_gap': gap, 'skipped_other': other}


def phase_breakdown(sp_rows) -> dict:
    """Per tick phase stats over the ticks that carry it (mean / p95 / max), the safety
    check's pose count, and the SLOWEST ticks with their phases and the phase that took
    longest. Empty values for recordings without the phase columns."""
    timed = [r for r in sp_rows if r.get('tick_s') is not None
             and any(r.get(c) is not None for c in PHASE_COLUMNS)]
    phases = {p: stats([r[f'{p}_s'] for r in timed if r.get(f'{p}_s') is not None])
              for p in PHASES}
    poses = sorted(int(r['check_poses']) for r in timed
                   if r.get('check_poses') is not None and r['check_poses'] > 0)
    slow = []
    for r in sorted(timed, key=lambda r: -r['tick_s'])[:SLOWEST]:
        ph = {p: r.get(f'{p}_s') or 0.0 for p in PHASES}
        slow.append({'stamp_s': r.get('stamp_s'), 'tick_s': r['tick_s'],
                     'outcome': r.get('text') or '', 'phases': ph,
                     'longest': max(PHASES, key=lambda p: ph[p]),
                     'check_poses': r.get('check_poses')})
    return {'phases': phases if timed else None,
            'check_poses': None if not poses else {
                'n': len(poses), 'mean': sum(poses) / len(poses),
                'p95': poses[min(len(poses) - 1, int(math.ceil(0.95 * len(poses))) - 1)],
                'max': poses[-1]},
            'slowest': slow}


def lag(ref: Sequence[Tuple[float, float]], out: Sequence[Tuple[float, float]],
        shifts: Sequence[float], ref_gap_s: float, min_motion_rad: float,
        min_overlap: float = 0.8) -> dict:
    """out(t) against ref(t - s): the s with the smallest RMS error (over shifts whose
    overlap is at least `min_overlap` of the best overlap, so a shift seeing only a
    sliver of the recording cannot win), that RMS, the Pearson correlation there, and
    the max |out - ref| at shift 0 and at that s. lag_s None when ref hardly moved.
    RMS, not the correlation peak: on ramp-and-hold motion the correlation is nearly
    flat over a range of shifts, the error is not."""
    def pairs(shift):
        p = []
        for t, q in out:
            r = _interp(ref, t - shift, ref_gap_s)
            if r is not None:
                p.append((r, q))
        return p
    res = {'lag_s': None, 'corr': None, 'rms_at_lag_rad': None, 'max_err_rad': None,
           'max_err_at_lag_rad': None, 'at_edge': False}
    base = pairs(0.0)
    if base:
        res['max_err_rad'] = max(abs(a - b) for a, b in base)
    if not ref or max(q for _, q in ref) - min(q for _, q in ref) < min_motion_rad:
        return res
    cand = [(s, pairs(s)) for s in shifts]
    most = max((len(p) for _, p in cand), default=0)
    best = None
    for s, p in cand:
        if len(p) < 3 or len(p) < min_overlap * most:
            continue
        rms = math.sqrt(sum((a - b) ** 2 for a, b in p) / len(p))
        if best is None or rms < best[1]:
            best = (s, rms, p)
    if best is not None:
        s, rms, p = best
        res['lag_s'], res['rms_at_lag_rad'], res['corr'] = round(s, 6), rms, _pearson(p)
        res['max_err_at_lag_rad'] = max(abs(a - b) for a, b in p)
        res['at_edge'] = s in (shifts[0], shifts[-1])
    return res


# ------------------------------------------------------------------ analysis
def analyze(rows: Iterable[Dict[str, object]], max_velocity: float,
            rate_hz: Optional[float] = None, time_base: str = 'recv_s',
            min_shift_s: float = -0.2, max_shift_s: float = 1.5, shift_step_s: float = 0.005,
            leader_gap_s: float = 0.2, min_motion_rad: float = 0.02,
            velocity_tolerance: float = 0.05) -> dict:
    """See the module docstring. `time_base` 'recv_s' or 'stamp_s' for every lag that
    involves the leader; `leader_gap_s`: samples further apart count as not streaming."""
    rows = list(rows)
    out = {'max_velocity': max_velocity, 'rate_hz': rate_hz, 'time_base': time_base,
           'joints': {}, 'intervals': {}, 'ticks': None, 'warnings': []}
    for src in (LEADER, FOLLOWER, SETPOINT):
        out['intervals'][src] = {k: interval_stats(_times(rows, src, k))
                                 for k in ('recv_s', 'stamp_s')}
    n_shift = int(round((max_shift_s - min_shift_s) / shift_step_s))
    shifts = [min_shift_s + k * shift_step_s for k in range(n_shift + 1)]
    sp_rows = [r for r in rows if r['source'] == SETPOINT]
    if sp_rows:
        outcomes: Dict[str, int] = {}
        for r in sp_rows:
            outcomes[r.get('text') or ''] = outcomes.get(r.get('text') or '', 0) + 1
        out['ticks'] = {
            'tick_s': stats([r['tick_s'] for r in sp_rows if r.get('tick_s') is not None]),
            'period_s': stats([r['period_s'] for r in sp_rows if r.get('period_s')]),
            'nominal_period_s': (1.0 / rate_hz) if rate_hz else None,
            'outcomes': outcomes}
        out['ticks'].update(phase_breakdown(sp_rows))
    vlim = max_velocity * (1 + velocity_tolerance)
    step_lim = (max_velocity / rate_hz) if rate_hz else None
    # Consecutive ticks: up to 1.5 nominal periods apart (a late tick still counts; a
    # lost message or a leader-mode gap does not). Without a rate: the observed median.
    periods = sorted(r['period_s'] for r in rows
                     if r['source'] == SETPOINT and r.get('period_s'))
    nominal = (1.0 / rate_hz) if rate_hz else (periods[len(periods) // 2] if periods else None)
    max_gap_s = None if nominal is None else 1.5 * nominal
    out['max_tick_gap_s'] = max_gap_s
    for j in JOINTS:
        lead = _series(rows, LEADER, j, time_base)
        foll = _series(rows, FOLLOWER, j, time_base)
        sp_tb = _series(rows, SETPOINT, j, time_base)
        sp_pi = _series(rows, SETPOINT, j, 'stamp_s')
        ms_tb = _series(rows, MEASURED, j, time_base)
        ms_pi = _series(rows, MEASURED, j, 'stamp_s')
        fv = _max_vel(_series(rows, FOLLOWER, j, 'stamp_s'))
        steps = setpoint_steps(rows, j, max_gap_s)
        res = {'n_leader': len(lead), 'n_follower': len(foll), 'n_setpoint': len(sp_pi),
               'leader_max_vel': _max_vel(lead), 'follower_max_vel': fv,
               'over_limit': None if fv is None else fv > vlim,
               'setpoint_max_vel': steps['max_vel'], 'setpoint_max_step_rad': steps['max_step'],
               'setpoint_steps': steps['n'], 'setpoint_skipped_gap': steps['skipped_gap'],
               'setpoint_skipped_other': steps['skipped_other'],
               'setpoint_over_limit': None, 'measured_max_vel': _max_vel(ms_pi)}
        if res['setpoint_max_step_rad'] is not None and step_lim is not None:
            # Per tick, not per second: a late tick followed by a prompt one reads as an
            # over-speed in rad/s although every step kept the limit. One tick of
            # rounding (whole ticks written) is allowed.
            res['setpoint_over_limit'] = res['setpoint_max_step_rad'] > step_lim * (
                1 + velocity_tolerance) + 2 * math.pi / 4096
        # Follower from /joint_states (kept for recordings without follow_debug)
        f = lag(lead, foll, shifts, leader_gap_s, min_motion_rad)
        res.update({'lag_s': f['lag_s'], 'corr': f['corr'], 'max_err_rad': f['max_err_rad'],
                    'max_err_at_lag_rad': f['max_err_at_lag_rad']})
        edges = [('leader->follower', f)]
        if sp_pi:
            ls = lag(lead, sp_tb, shifts, leader_gap_s, min_motion_rad)
            sm = lag(sp_pi, ms_pi, shifts, max(leader_gap_s, 0.5), min_motion_rad)
            lm = lag(lead, ms_tb, shifts, leader_gap_s, min_motion_rad)
            res.update({'lag_leader_setpoint_s': ls['lag_s'],
                        'corr_leader_setpoint': ls['corr'],
                        'max_err_leader_setpoint_rad': ls['max_err_rad'],
                        'lag_setpoint_measured_s': sm['lag_s'],
                        'corr_setpoint_measured': sm['corr'],
                        'max_err_setpoint_measured_rad': sm['max_err_rad'],
                        'lag_leader_measured_s': lm['lag_s'],
                        'max_err_leader_measured_rad': lm['max_err_rad']})
            edges += [('leader->setpoint', ls), ('setpoint->measured', sm),
                      ('leader->measured', lm)]
        if steps['skipped_gap']:
            out['warnings'].append(f'{j}: {steps["skipped_gap"]} setpoint pair(s) more than '
                                   f'{max_gap_s:.4f} s apart (lost follow_debug message or '
                                   'leader mode off between them) left out of the step check')
        for name, r in edges:
            if r['at_edge']:
                out['warnings'].append(f'{j}: {name} lag at the edge of the searched range '
                                       f'[{min_shift_s}, {max_shift_s}] s')
        out['joints'][j] = res
    fi = out['intervals'][FOLLOWER]
    ref = fi['stamp_s'] or fi['recv_s']
    if ref and ref['mean_s'] > SLOW_SAMPLING_S:
        out['warnings'].append(
            f'/joint_states sampled every {ref["mean_s"]:.3f} s on average: it comes from '
            'arm_action_server\'s 2 Hz timer, not from the leader tick - its intervals are '
            'not the tick time, and its velocities are averages over that interval')
    if not sp_rows:
        out['warnings'].append('no /arm/leader/follow_debug in the recording (leader mode '
                               'off, or leader.follow_debug_enabled false): no tick time, no '
                               'setpoint/servo split')
    return out


def format_report(res: dict) -> str:
    def f(v, fmt='{:.3f}'):
        return '-' if v is None else fmt.format(v)

    def st(s):
        return '-' if not s else (f'{s["mean_s"] * 1e3:.1f} / {s["p95_s"] * 1e3:.1f} / '
                                  f'{s["max_s"] * 1e3:.1f} ms ({s["n"]})')
    lines = [f'time base for leader lags: {res["time_base"]}; follower speed limit '
             f'{res["max_velocity"]} rad/s; leader rate {res["rate_hz"]} Hz']
    tk = res['ticks']
    if tk:
        nom = tk['nominal_period_s']
        lines += [f'tick duration  mean / p95 / max: {st(tk["tick_s"])}',
                  f'tick period    mean / p95 / max: {st(tk["period_s"])}'
                  + ('' if nom is None else f'  (nominal {nom * 1e3:.1f} ms)'),
                  f'outcomes: {tk["outcomes"]}']
        if tk.get('phases'):
            lines.append('tick phases    mean / p95 / max:')
            lines += [f'  {p:8} {st(s)}' for p, s in tk['phases'].items() if s]
            cp = tk.get('check_poses')
            if cp:
                lines.append(f'  safety check poses per tick mean / p95 / max: '
                             f'{cp["mean"]:.1f} / {cp["p95"]} / {cp["max"]} ({cp["n"]})')
            lines.append(f'slowest {len(tk["slowest"])} ticks (ms; longest phase first):')
            for t in tk['slowest']:
                ph = ', '.join(f'{p} {v * 1e3:.1f}' for p, v in
                               sorted(t['phases'].items(), key=lambda kv: -kv[1]) if v > 0)
                lines.append(f'  stamp {f(t["stamp_s"])} {t["tick_s"] * 1e3:.1f} ms '
                             f'{t["outcome"]}: {ph}')
        lines.append('joint  sp_vmax sp_step  over | L->sp  corr  | sp->meas corr | '
                     'L->meas err_L-meas | lead_vmax')
        for j, r in res['joints'].items():
            if not r['n_setpoint']:
                continue
            lines.append(
                f'{j:6} {f(r["setpoint_max_vel"]):>7} '
                f'{f(r["setpoint_max_step_rad"], "{:.4f}"):>7} '
                f'{str(r["setpoint_over_limit"]):>5} | {f(r["lag_leader_setpoint_s"]):>6} '
                f'{f(r["corr_leader_setpoint"]):>6} | {f(r["lag_setpoint_measured_s"]):>6} '
                f'{f(r["corr_setpoint_measured"]):>6} | {f(r["lag_leader_measured_s"]):>6} '
                f'{f(r["max_err_leader_measured_rad"]):>8} | {f(r["leader_max_vel"]):>9}')
    lines.append('/joint_states (2 Hz): joint lag_s corr max_err err@lag foll_vmax over n_foll')
    for j, r in res['joints'].items():
        lines.append(f'  {j:6} {f(r["lag_s"]):>6} {f(r["corr"]):>6} {f(r["max_err_rad"]):>7} '
                     f'{f(r["max_err_at_lag_rad"]):>7} {f(r["follower_max_vel"]):>9} '
                     f'{str(r["over_limit"]):>5} {r["n_follower"]:>6}')
    lines.append('message intervals (mean / p95 / max, n):')
    for src, by in res['intervals'].items():
        for key, s in by.items():
            if s:
                lines.append(f'  {src:8} {key:7} {st(s)}')
    lines += [f'WARNING: {w}' for w in res['warnings']]
    return '\n'.join(lines)
