"""SR-17 body check, second stage: a box overlap of an arm link with the chassis is
confirmed or cleared on the convex hull of the link's mesh against the same chassis
box, grown by the hull's own certified motion bound (SafetyFilter._body_hull_check).
Robot test 2026-10-20: limb3 rested 3.1 mm (boxes) from the chassis - ~42 mm mesh to
mesh - and a 2-tick limb2 step AWAY from the chassis grew the limb3 box by 3.3 mm:
refused at sub-step 0, leader mode stuck. Model, active config, active calibration.

The truth the hull stage is held to is the arm MESH against the chassis BOXES (the
body model; test_chassis_collision: the chassis mesh lies inside them), computed
here exactly per triangle - not the hulls the filter itself uses.
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_safety_filter import active, build, resolve_mesh, urdf  # noqa: E402,F401
from test_planner import JOINTS, _active_executor  # noqa: E402

from gripperx_arm.convex_distance import clearance_at_least  # noqa: E402
from gripperx_arm.kinematics import _origin, as_box  # noqa: E402
from gripperx_arm.mesh_distance import read_stl_triangles  # noqa: E402
from gripperx_arm.safety_filter import SafetyFilter, Verdict, unwrap_steer_reading  # noqa: E402

# Robot test 2026-10-20, logged radians (arm_action_server leader log): the pose held
# while the leader was refused, and the last goal written before.
MEASURED = [0.845223, -3.339476, 2.986661, -2.580156, -0.130388]
HELD = [0.846757, -3.319534, 2.989729, -2.584758, -0.121184]
STEER_RAW = [0.0015, -0.0031, 0.0031, 0.0061]     # /hw/steer_states at startup, same log


@pytest.fixture(scope='module')
def flt(urdf, active):
    return build(urdf, active)


@pytest.fixture(scope='module')
def boxes_only(urdf, active):
    return build(urdf, active, hulls=False)


@pytest.fixture(scope='module')
def incident(flt, active):
    """(measured, held, tick in rad, envelope from the logged steering readings)."""
    sf, st, _acm = active
    cal, _steps = _active_executor()
    grip = cal.to_radians([int(round(z)) for z in cal.zero_ticks])['arm_gripper_joint']
    m, h = ({**dict(zip(JOINTS, v)), 'arm_gripper_joint': grip} for v in (MEASURED, HELD))
    steer = {n: unwrap_steer_reading(r, int(st['steering_outward_sign'][i]),
                                     math.radians(st['steering_mechanical_outward_deg'][i]),
                                     math.radians(st['steering_mechanical_inward_deg'][i]),
                                     sf['steer_motion_margin_rad'])
             for i, (n, r) in enumerate(zip(sf['steer_joint_names'], STEER_RAW))}
    env = flt.envelope_measured(steer, sf['steer_motion_margin_rad'])
    return m, h, 2.0 * math.pi / cal.ticks_per_rev, env


def _body(f):
    return [p for g, s in f.envelope_swept.groups if g is None for p in s
            if p.link in f._rigid_body]


def _checked_links(f):
    """Arm links with a hull whose pair with the chassis is not an allowed pair."""
    return [ln for ln in f._hull_radius
            if all(tuple(sorted((ln, b.link))) not in f.acm.always for b in _body(f))]


@pytest.fixture(scope='module')
def triangles(urdf, flt):
    """Per arm link, its visual mesh triangles in the link frame (as load_link_hulls)."""
    out = {}
    for link in ET.fromstring(urdf).findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if link.get('name') not in flt.kin.links or mesh is None:
            continue
        t = read_stl_triangles(resolve_mesh(mesh.get('filename')))
        s = [float(x) for x in (mesh.get('scale') or '1 1 1').split()]
        o = _origin(link.find('visual/origin'))
        out[link.get('name')] = (t * s) @ o[:3, :3].T + o[:3, 3]
    return out


_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
                    dtype=float)


def _tris_touch_box(t, h):
    """Per triangle (N, 3, 3, box frame): does it meet the box of half extents h
    (closed)? Separating axis test, 13 axes: the box's 3, the triangle's normal and
    the 9 edge cross products (exact for a triangle and a box)."""
    e = np.roll(t, -1, axis=1) - t
    axes = [np.broadcast_to(np.eye(3)[k], (len(t), 3)) for k in range(3)]
    axes.append(np.cross(e[:, 0], e[:, 1]))
    axes += [np.cross(np.eye(3)[k], e[:, j]) for k in range(3) for j in range(3)]
    apart = np.zeros(len(t), bool)
    for a in axes:
        p = np.einsum('nvc,nc->nv', t, a)
        r = np.abs(a) @ h
        apart |= (p.min(axis=1) > r) | (p.max(axis=1) < -r)
    return ~apart


def mesh_touches(f, tris, q, clearance: float) -> bool:
    """Does an arm MESH (checked links) come closer than `clearance` to a chassis BOX
    at q (touching counts when clearance is 0)? Candidates from the box grown by
    `clearance` per side (exact separating axes per triangle), each candidate settled by
    its exact triangle-to-box distance."""
    frames = f.kin.fk(f._q(q))
    for ln in _checked_links(f):
        fr = frames[ln]
        pts = tris[ln].reshape(-1, 3)
        c0 = pts.mean(axis=0)
        r0 = float(np.linalg.norm(pts - c0, axis=1).max())
        cw = fr[:3, :3] @ c0 + fr[:3, 3]
        tw = None
        for b in _body(f):
            fb, h = as_box(b)
            h = h + clearance
            if np.linalg.norm(np.maximum(np.abs((cw - fb[:3, 3]) @ fb[:3, :3]) - h, 0.0)) > r0:
                continue
            if tw is None:
                tw = tris[ln] @ fr[:3, :3].T + fr[:3, 3]
                edge = np.linalg.norm(tw - np.roll(tw, 1, axis=1), axis=2).max(axis=1)
            loc = (tw - fb[:3, 3]) @ fb[:3, :3]
            # no point of a triangle is farther than its longest edge from a vertex
            near = np.linalg.norm(np.maximum(np.abs(loc) - h, 0.0), axis=2).min(axis=1) <= edge
            if not near.any():
                continue
            hit = np.nonzero(near)[0][_tris_touch_box(loc[near], h)]
            corners = (_CORNERS * (h - clearance)) @ fb[:3, :3].T + fb[:3, 3]
            # the grown box over-reports at its edges: exact triangle distance (GJK)
            if any(not clearance_at_least(tw[i], corners, clearance)[0] or clearance <= 0.0
                   for i in hit):
                return True
    return False


def hull_clearance(f, q) -> float:
    """Smallest certified lower bound of the hull-to-chassis-box clearance (checked
    links) at q - sampled along a step, independent of the step's motion bound."""
    frames = f.kin.fk(f._q(q))
    best = math.inf
    for ln in _checked_links(f):
        for b in _body(f):
            c = f.body_hull_lower(frames, ln, b)
            if c < best:
                c = max(c, f.body_hull_clearance(frames, ln, b)[0])
            best = min(best, c)
    return best


# ---------------------------------------------------------------- the incident
def test_the_incident_is_reproduced_on_the_boxes(boxes_only, incident):
    m, h, tick, env = incident
    w = dict(h, arm_limb1_joint=h['arm_limb1_joint'] - tick,
             arm_limb2_joint=h['arm_limb2_joint'] + 2 * tick)
    v = boxes_only.check_setpoints([m, w], env)
    assert not v.ok and v.rule == 'body' and v.index == 0, v
    assert v.detail.startswith('arm_limb3_link vs chassis_link (boxes;'), v


@pytest.mark.parametrize('ticks', [1, 2, 3, 5, 7])
def test_the_incident_step_away_from_the_chassis_passes(flt, incident, ticks):
    m, h, tick, env = incident
    w = dict(h, arm_limb1_joint=h['arm_limb1_joint'] - tick,
             arm_limb2_joint=h['arm_limb2_joint'] + ticks * tick)
    v = flt.check_setpoints([m, w], env)
    assert v.ok, v


def test_the_incident_hull_numbers(flt, incident):
    """The hull clearance exceeds its motion bound for the 2-tick step; the box gap did
    not. Both read from the model, nothing typed in."""
    m, h, tick, _env = incident
    w = dict(h, arm_limb1_joint=h['arm_limb1_joint'] - tick,
             arm_limb2_joint=h['arm_limb2_joint'] + 2 * tick)
    frames = flt.kin.fk(flt._q(m))
    dq = {n: abs(w[n] - m[n]) for n in m}
    c = min(flt.body_hull_clearance(frames, 'arm_limb3_link', b)[0] for b in _body(flt))
    g = flt._hull_growth(frames, 'arm_limb3_link', dq)
    assert c - g > flt.cfg.body_hull_clearance_m
    assert flt.body_hull_lower(frames, 'arm_limb3_link', _body(flt)[0]) <= \
        flt.body_hull_clearance(frames, 'arm_limb3_link', _body(flt)[0])[0] + 1e-9


def _into_chassis(flt, m):
    """From the measured pose: the first arm joint (URDF order) that, moved alone and
    inside its limits, brings limb3's hull to the chassis box - its direction, and the
    offset at which the hull clearance reaches body_hull_clearance_m (from the model)."""
    def c(q):
        frames = flt.kin.fk(flt._q(q))
        return min(flt.body_hull_clearance(frames, 'arm_limb3_link', b)[0] for b in _body(flt))
    bc = flt.cfg.body_hull_clearance_m
    for jn in flt.kin.joint_names:
        s = -1.0 if c(dict(m, **{jn: m[jn] + 1e-3})) > c(m) else 1.0
        lo_lim, hi_lim = flt.kin.limits[jn]
        room = (hi_lim - m[jn]) if s > 0 else (m[jn] - lo_lim)
        if room <= 0.0 or c(dict(m, **{jn: m[jn] + s * room})) >= bc:
            continue
        lo, hi = 0.0, room
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            lo, hi = (lo, mid) if c(dict(m, **{jn: m[jn] + s * mid})) < bc else (mid, hi)
        if hi < 0.8 * room:                 # the contact lies well inside the limits
            return jn, s, hi, c
    pytest.fail('no single joint brings limb3 to the chassis - the model changed')


def test_a_step_into_the_chassis_is_still_refused(flt, incident):
    """One joint moved until limb3's hull reaches the chassis box (body_hull_clearance_m):
    the step there is refused [body], and the text reports the hull clearance."""
    m, _h, _tick, env = incident
    jn, s, reach, c = _into_chassis(flt, m)
    end = dict(m, **{jn: m[jn] + s * 1.1 * reach})
    assert c(end) < flt.cfg.body_hull_clearance_m
    v = flt.check_setpoints([m, end], env)
    assert not v.ok and v.rule == 'body', v
    assert '(hull ' in v.detail and 'chassis_link' in v.detail, v


def test_the_motion_bound_refuses_a_step_whose_ends_are_clear(flt, incident):
    """A step INTO the chassis that stops short of it: both end poses clear on the
    hull, but checked as one step its motion bound exceeds the remaining gap - refused,
    though no statically checked pose touches."""
    m, _h, _tick, env = incident
    jn, s, reach, c = _into_chassis(flt, m)
    start = dict(m, **{jn: m[jn] + s * 0.2 * reach})
    end = dict(m, **{jn: m[jn] + s * 0.98 * reach})
    assert c(start) > flt.cfg.body_hull_clearance_m and c(end) > flt.cfg.body_hull_clearance_m
    assert flt.check_pose(start, env).ok and flt.check_pose(end, env).ok
    v = _one_step(flt).check_setpoints([start, end], env)
    assert not v.ok and v.rule == 'body' and 'motion bound' in v.detail, v


def test_the_cheap_bound_never_exceeds_the_hull_clearance(flt):
    """body_hull_lower (separating planes, the fast path that can clear a pair alone) is
    <= the certified upper bound of the GJK/EPA clearance, every link and chassis box,
    random poses - also overlapping ones."""
    rnd = np.random.default_rng(3)
    lim = flt.kin.limits
    n = 0
    for _ in range(150):
        frames = flt.kin.fk({k: rnd.uniform(*lim[k]) for k in flt.kin.joint_names})
        for ln in flt._hull_radius:
            for b in _body(flt):
                lo, hi = flt.body_hull_clearance(frames, ln, b)
                assert flt.body_hull_lower(frames, ln, b) <= hi + 1e-9, (ln, lo, hi)
                n += lo < 0.0
    assert n > 0                                  # overlapping cases were among them


def test_wheels_keep_the_box_verdict(flt):
    """The hull stage applies to body links fixed to base_link only: no wheel or
    steering module is among them."""
    moved = {j.child for j in flt.kin._env_order if j.type != 'fixed'}
    assert 'chassis_link' in flt._rigid_body
    assert moved and not moved & flt._rigid_body


def test_the_hull_clearance_parameter_is_honoured(urdf, active, incident):
    """The threshold is body_hull_clearance_m: raised above the incident's hull
    clearance (minus its bound), the 2-tick step is refused again."""
    m, h, tick, env = incident
    probe = build(urdf, active)
    frames = probe.kin.fk(probe._q(m))
    c = min(probe.body_hull_clearance(frames, 'arm_limb3_link', b)[0] for b in _body(probe))
    w = dict(h, arm_limb1_joint=h['arm_limb1_joint'] - tick,
             arm_limb2_joint=h['arm_limb2_joint'] + 2 * tick)
    assert build(urdf, active, body_hull_clearance_m=0.5 * c).check_setpoints([m, w], env).ok
    v = build(urdf, active, body_hull_clearance_m=c).check_setpoints([m, w], env)
    assert not v.ok and v.rule == 'body' and 'hull clearance <=' in v.detail, v


# ---------------------------------------------------------------- switches
@pytest.mark.parametrize('self_stage', [False, True])
@pytest.mark.parametrize('body_stage', [False, True])
def test_the_two_hull_stages_switch_independently(urdf, active, incident, self_stage,
                                                  body_stage):
    """body_hull_stage decides the incident step (a chassis pair), mesh_hull_stage the
    limb1/limb3 box artefact (a self pair) - each only its own."""
    f = build(urdf, active, mesh_hull_stage=self_stage, body_hull_stage=body_stage)
    m, h, tick, env = incident
    w = dict(h, arm_limb1_joint=h['arm_limb1_joint'] - tick,
             arm_limb2_joint=h['arm_limb2_joint'] + 2 * tick)
    v = f.check_setpoints([m, w], env)
    assert v.ok == body_stage, v
    if not body_stage:
        assert v.detail.startswith('arm_limb3_link vs chassis_link (boxes;'), v
    q = {n: 0.0 for n in f.kin.joint_names}
    q['arm_limb2_joint'] = math.radians(-15)
    v = f.check_pose(q)
    assert v.ok == self_stage, v
    if not self_stage:
        assert v.rule == 'self' and '(boxes;' in v.detail, v


def test_the_hull_stage_without_a_clearance_never_clears(urdf, active, incident):
    """FilterConfig's default body_hull_clearance_m (unset = infinite): the stage on but
    no value configured refuses as the boxes do; a negative value is rejected."""
    m, h, tick, env = incident
    f = build(urdf, active)
    cfg = type(f.cfg)(**{**f.cfg.__dict__, 'body_hull_clearance_m': math.inf})
    g = SafetyFilter(f.kin, f.acm, cfg, f.mechanical_ranges, hulls=f.hulls)
    w = dict(h, arm_limb2_joint=h['arm_limb2_joint'] + 2 * tick)
    assert not g.check_setpoints([m, w], env).ok
    with pytest.raises(ValueError):
        SafetyFilter(f.kin, f.acm, type(f.cfg)(**{**f.cfg.__dict__,
                                                  'body_hull_clearance_m': -0.001}),
                     f.mechanical_ranges, hulls=f.hulls)


# ---------------------------------------------------------------- lidar (use_lidar:=true)
@pytest.fixture(scope='module')
def lidar_case(active):
    """Description WITH the lidar (as on the robot). The limb4 witness pose of
    test_lidar_pair (limb4 INTO the lidar box), limb4 vs chassis allowed so the lidar
    is the pair under test, and a clear start found from the model: the witness with
    one joint moved back out until every check passes."""
    import copy
    import xacro
    from test_lidar_pair import WITNESSES, _desc_dir
    sf, st, acm = active
    u = xacro.process_file(str(_desc_dir() / 'urdf' / 'gripperx_v1.urdf.xacro'),
                           mappings={'use_lidar': 'true'}).toxml()
    acm = copy.deepcopy(acm)
    acm['allowed_pairs'].append({'links': ['arm_limb4_link', 'chassis_link'],
                                 'scope': 'always', 'reason': 'test'})
    f = build(u, (sf, st, acm))
    into = dict(WITNESSES['arm_limb4_link'])
    lid = next(p for p in _body(f) if p.link == 'lidar_link')
    for jn in f.kin.joint_names:
        for d in (0.02, 0.05, 0.1, 0.2):
            for s in (1.0, -1.0):
                q = dict(into, **{jn: into[jn] + s * d})
                lo, hi = f.kin.limits[jn]
                if lo <= q[jn] <= hi and f.check_pose(q).ok:
                    return u, (sf, st, acm), f, q, into, lid
    pytest.fail('no clear start next to the lidar witness - the model changed')


def test_the_lidar_is_in_the_hull_stage(lidar_case):
    _u, _a, f, start, into, lid = lidar_case
    assert 'lidar_link' in f._rigid_body
    frames = f.kin.fk(f._q(start))
    assert f.body_hull_clearance(frames, 'arm_limb4_link', lid)[0] >= \
        f.cfg.body_hull_clearance_m
    v = f.check_pose(into)
    assert not v.ok and v.detail.startswith('arm_limb4_link vs lidar_link (hull '), v


def test_a_step_into_the_lidar_is_refused(lidar_case):
    _u, _a, f, start, into, _lid = lidar_case
    v = f.check_setpoints([start, into])
    assert not v.ok and v.rule == 'body' and 'vs lidar_link (hull ' in v.detail, v


def test_the_hull_clearance_parameter_is_honoured_at_the_lidar(lidar_case):
    """The clear start, re-checked with body_hull_clearance_m just above and just below
    its smallest static hull clearance from the lidar box (any arm link)."""
    u, act, f, start, _into, lid = lidar_case
    frames = f.kin.fk(f._q(start))
    c = min(f.body_hull_clearance(frames, ln, lid)[0] for ln in f._hull_radius
            if tuple(sorted((ln, 'lidar_link'))) not in f.acm.always)
    assert build(u, act, body_hull_clearance_m=c - 1e-4).check_pose(start).ok
    v = build(u, act, body_hull_clearance_m=c + 1e-4).check_pose(start)
    assert not v.ok and ' vs lidar_link (hull clearance <= ' in v.detail, v


# ---------------------------------------------------------------- adversarial
def _seeds(flt, boxes_only, rnd, want, tries):
    """Random poses inside the joint limits that the boxes refuse against the chassis
    and the hull stage accepts - the poses the new stage decides."""
    lim = flt.kin.limits
    out = []
    for _ in range(tries):
        q = {n: rnd.uniform(*lim[n]) for n in flt.kin.joint_names}
        v = boxes_only.check_pose(q)
        if v.ok or v.rule != 'body' or 'vs chassis_link' not in v.detail:
            continue
        if flt.check_pose(q).ok:
            out.append(q)
            if len(out) >= want:
                break
    return out


@pytest.fixture(scope='module')
def adversarial_steps(flt, boxes_only):
    """(start, direction, length) candidates: from poses the hull stage decides, random
    directions at several lengths, and the largest step the active filter accepts in a
    random direction turned toward the chassis (bisection); plus a path whose two ends
    are clear and whose middle crosses the chassis (test_safety_filter A2)."""
    from test_safety_filter import ik
    rnd = np.random.default_rng(20261020)
    seeds = _seeds(flt, boxes_only, rnd, 10, 3000)
    assert len(seeds) >= 8, len(seeds)
    names = flt.kin.joint_names

    def at(q, d, s):
        return {n: q[n] + s * d[i] for i, n in enumerate(names)}
    out = []
    for q in seeds:
        for _ in range(3):
            d = rnd.normal(size=len(names))
            d /= np.linalg.norm(d)
            out += [(q, d, s) for s in (0.03, 0.08, 0.15, 0.3)]
        for _ in range(2):
            d = rnd.normal(size=len(names))
            d /= np.linalg.norm(d)
            if hull_clearance(flt, at(q, d, 1e-3)) > hull_clearance(flt, at(q, d, -1e-3)):
                d = -d
            lo, hi = 0.0, 0.15
            for _ in range(9):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if flt.check_setpoints([q, at(q, d, mid)]).ok else (lo, mid)
            if lo > 0.0:
                out.append((q, d, lo))
    a, b = ik(flt, 0.10, 0.25, 0.0), ik(flt, 0.30, 0.0, 0.05)
    d = np.array([b[n] - a[n] for n in names])
    return [(a, d / np.linalg.norm(d), float(np.linalg.norm(d)))] + out


def _harness(f, steps, truth, triangles, samples=31, first=False):
    """(accepted, mesh violations, hull violations) over the steps f accepts, each
    sampled densely along the joint-linear motion; measured with `truth`'s model."""
    names = truth.kin.joint_names
    bc = truth.cfg.body_hull_clearance_m
    accepted, mesh_bad, hull_bad = 0, [], []
    for q, d, s in steps:
        end = {n: q[n] + s * d[i] for i, n in enumerate(names)}
        if not f.check_setpoints([q, end]).ok:
            continue
        accepted += 1
        for t in np.linspace(0.0, s, samples):
            qt = {n: q[n] + t * d[i] for i, n in enumerate(names)}
            if mesh_touches(truth, triangles, qt, bc):
                mesh_bad.append((t, s))
                if first:
                    return accepted, mesh_bad, hull_bad
                break
            c = hull_clearance(truth, qt)
            if c < bc - 1e-7:
                hull_bad.append((c, t, s))
                break
    return accepted, mesh_bad, hull_bad


def _one_step(flt):
    """The active filter without subdivision (one checked step, however long): what
    keeps the motion between the two ends clear is the motion bound alone."""
    cfg = type(flt.cfg)(**{**flt.cfg.__dict__, 'max_joint_step_rad': 10.0})
    return SafetyFilter(flt.kin, flt.acm, cfg, flt.mechanical_ranges, hulls=flt.hulls)


class _EndsOnly:
    """Mutant: accepts a step if its two ends pass the static check - no motion bound,
    no subdivision. The harness must catch it."""
    def __init__(self, flt):
        self._f = flt

    def check_setpoints(self, poses):
        return Verdict(all(self._f.check_pose(p).ok for p in poses))


def test_no_accepted_step_takes_the_mesh_below_the_hull_clearance(
        flt, adversarial_steps, triangles):
    """Active configuration: along every accepted step, densely sampled, the arm meshes
    (and their hulls) stay >= body_hull_clearance_m from the chassis boxes."""
    accepted, mesh_bad, hull_bad = _harness(flt, adversarial_steps, flt, triangles)
    assert accepted >= 20, accepted
    assert not mesh_bad and not hull_bad, (mesh_bad, hull_bad)


def test_the_motion_bound_alone_keeps_the_step_clear(flt, adversarial_steps, triangles):
    """Without subdivision only the motion bound covers the motion between the two
    ends: still no violation. The same harness with the bound removed accepts steps
    whose middle reaches the chassis - mesh included - so the test can fail."""
    accepted, mesh_bad, hull_bad = _harness(_one_step(flt), adversarial_steps, flt, triangles)
    assert accepted >= 5, accepted
    assert not mesh_bad and not hull_bad, (mesh_bad, hull_bad)
    _a, mesh0, _hull0 = _harness(_EndsOnly(flt), adversarial_steps, flt, triangles, first=True)
    assert mesh0
