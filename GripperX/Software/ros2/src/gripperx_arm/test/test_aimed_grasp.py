"""Aimed P (FR-16 stage 1) without ROS: the tilted IK, the solver and the allowed region,
all against the ACTIVE model and configuration (the xacro, arm_poses.yaml,
arm_calibration.yaml, arm_safety_filter.yaml, steer_servo.yaml). No value of the
region is written here: the mutation tests below change the model or the configuration
and require the region to follow - a test that kept passing would be reading a copy.
"""
import math
import re
from pathlib import Path

import numpy as np
import pytest
import yaml

from gripperx_arm import poses as poses_mod
from gripperx_arm.aimed_grasp import (
    ENVELOPE_REJECTED, INVALID_TARGET, OUT_OF_REACH, AimedGraspSolver, standoff_proposal)
from gripperx_arm.kinematics import ArmKinematics
from gripperx_arm.safety_filter import (
    FilterConfig, SafetyFilter, load_acm, load_link_hulls, mechanical_steer_ranges)

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
ROLLS = [math.radians(d) for d in (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)]


def _desc():
    d = SRC / 'gripperx_description'
    if not (d / 'urdf').is_dir():
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory('gripperx_description'))
    return d


def _steer():
    p = SRC / 'gripperx_control' / 'config' / 'steer_servo.yaml'
    if not p.is_file():
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('gripperx_control')) / 'config' / 'steer_servo.yaml'
    return yaml.safe_load(p.read_text())['steer_servo_node']['ros__parameters']


def _urdf():
    import xacro
    return xacro.process_file(str(_desc() / 'urdf' / 'gripperx_v1.urdf.xacro')).toxml()


def _sf():
    return yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['safety_filter']


def _aimed_cfg():
    return yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['aimed']


def _descent_kw(**over):
    """The vertical descent's parameters from the ACTIVE yaml (user decision 2026-11-16)."""
    a = _aimed_cfg()
    kw = {'pregrasp_height': a['pregrasp_height_m'], 'descent_step': a['descent_step_m'],
          'descent_max_lateral': a['descent_max_lateral_m']}
    kw.update(over)
    return kw


def build(urdf_xml=None, grip=None, descent=None, **cfg_over):
    """(solver, filter) as arm_action_server builds them from the active files."""
    urdf_xml = urdf_xml or URDF
    sf = dict(_sf(), **cfg_over)
    st = _steer()
    kin = ArmKinematics(urdf_xml)
    rng = mechanical_steer_ranges(kin, sf['steer_joint_names'], st['steering_outward_sign'],
                                  st['steering_mechanical_outward_deg'],
                                  st['steering_mechanical_inward_deg'])
    cfg = FilterConfig.from_params(sf.__getitem__)      # the node's construction
    acm = load_acm(yaml.safe_load((PKG / 'config' / 'arm_allowed_collisions.yaml').read_text()),
                   yaml.safe_load((PKG / 'config' / 'arm_stop_protected.yaml').read_text()))
    flt = SafetyFilter(kin, acm, cfg, rng, sf['ground_frame'],
                       hulls=load_link_hulls(urdf_xml, kin, lambda uri: str(
                           _desc() / 'meshes' / uri.split('/')[-1])))
    cal = poses_mod.calibration_from_doc(
        yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text()))
    p = yaml.safe_load((PKG / 'config' / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']
    if grip is not None:
        p['poses']['grip'] = grip
    r = poses_mod.resolve(p, cal)
    qg = cal.to_radians(r['grip'])
    gopen = cal.to_radians(r['home'][:-1] + [r['open']])['arm_gripper_joint']
    ref = flt.envelope_measured({n: 0.0 for n in flt.mechanical_ranges},
                                sf['steer_motion_margin_rad'])
    return AimedGraspSolver(flt, qg, gopen, ROLLS, 0.0, ref,
                            **_descent_kw(**(descent or {}))), flt


URDF = _urdf()


@pytest.fixture(scope='module')
def solver():
    return build()[0]


@pytest.fixture(scope='module')
def region(solver):
    return solver.derive_region(0.02)


def test_filter_config_factory_reads_every_field():
    """Audit M4: node and this test share FilterConfig.from_params. A field added to
    FilterConfig but not read there would fall back to its default unnoticed."""
    import dataclasses
    asked = []
    sf = _sf()
    cfg = FilterConfig.from_params(lambda n: asked.append(n) or sf[n])
    assert sorted(asked) == sorted(f.name for f in dataclasses.fields(FilterConfig))
    assert all(getattr(cfg, n) == (list(sf[n]) if n == 'tip_links' else sf[n]) for n in asked)


# ------------------------------------------------------------------ IK
@pytest.mark.parametrize('tilt_deg', [0.0, 15.0, 30.0, -20.0])
def test_tilted_ik_round_trip_on_a_grid(solver, tilt_deg):
    kin, tilt = solver.kin, math.radians(tilt_deg)
    n = 0
    for x in np.arange(0.18, 0.46, 0.04):
        for y in np.arange(-0.24, 0.25, 0.04):
            for s in kin.ik_tcp_tilted(x, y, solver.z, tilt, roll=0.4).solutions:
                n += 1
                t = kin.tcp(s.q)
                assert np.linalg.norm(t[:3, 3] - [x, y, solver.z]) < 1e-7
                assert abs(kin.approach_tilt(s.q) - tilt) < 1e-7
                assert s.q['arm_limb5_joint'] == pytest.approx(0.4) or not s.within_limits
    assert n > 50


def test_tilt_zero_is_ik_tcp_down(solver):
    kin = solver.kin
    a = kin.ik_tcp_down(0.35, 0.05, solver.z, roll=0.2)
    b = kin.ik_tcp_tilted(0.35, 0.05, solver.z, 0.0, roll=0.2)
    assert [s.branch for s in a.solutions] == [s.branch for s in b.solutions]
    for sa, sb in zip(a.solutions, b.solutions):
        assert all(abs(sa.q[j] - sb.q[j]) < 1e-12 for j in sa.q)
        assert kin.tcp(sa.q)[2, 2] == pytest.approx(-1.0)


def test_ik_reproduces_p_grip_pose_with_its_own_tilt(solver):
    """IK at P's own TCP with P's own tilt and roll returns P's taught joints."""
    kin, qg = solver.kin, solver.grip_q
    t = kin.tcp(qg)[:3, 3]
    res = kin.ik_tcp_tilted(t[0], t[1], t[2], kin.approach_tilt(qg),
                            roll=qg['arm_limb5_joint'], hold={'arm_gripper_joint': 0.0})
    best = min(res.solutions, key=lambda s: sum(abs(math.remainder(s.q[j] - qg[j], math.tau))
                                                for j in solver.chain))
    assert all(abs(math.remainder(best.q[j] - qg[j], math.tau)) < 1e-6 for j in solver.chain)


# ------------------------------------------------------------------ solver
def test_grasp_height_is_p_grip_tcp_height(solver):
    assert solver.z == pytest.approx(float(solver.kin.tcp(solver.grip_q)[2, 3]))


def test_solution_is_vertical_at_target_and_height(solver, region):
    # the middle of the region on y = 0 (the mean over every row with a cell fell
    # outside once the steep approach shrank the inner edge, 2026-11-16)
    x, y = standoff_proposal(region)['centre_x_m'], 0.0
    r = solver.solve(float(x), y, solver.ref_envelope, limit=3)
    assert r.ok, r.detail
    for c in r.candidates:
        t = solver.kin.tcp(c.q)
        assert np.linalg.norm(t[:3, 3] - [x, y, solver.z]) < 1e-7
        assert t[2, 2] == pytest.approx(-1.0, abs=1e-9)
        assert c.q['arm_gripper_joint'] == solver.hold['arm_gripper_joint']
        assert solver.flt.check_pose(c.q, solver.ref_envelope).ok


def test_refusals(solver, flt_swept_point):
    assert solver.solve(1.5, 0.0).reason == OUT_OF_REACH                    # no IK
    assert solver.solve(0.17, 0.0).reason == OUT_OF_REACH                   # over the chassis
    assert solver.solve(float('nan'), 0.0).reason == INVALID_TARGET
    x, y = flt_swept_point
    r = solver.solve(x, y, solver.flt.envelope_swept)
    assert r.reason == ENVELOPE_REJECTED, r
    assert solver.in_region(x, y).ok


@pytest.fixture(scope='module')
def flt_swept_point(solver, region):
    """A region point that the swept-wheel envelope rejects (searched, not typed)."""
    for i, j in zip(*np.nonzero(region.mask)):
        x, y = float(region.xs[i]), float(region.ys[j])
        if not solver.candidates(x, y, solver.flt.envelope_swept, limit=1).ok:
            return x, y
    pytest.skip('no region point is rejected with the wheels swept')


def test_target_z_and_other_inputs_do_not_change_the_pose(solver):
    """SR-16 E2 (draft): only (x, y) is aimed input - the solver has no other."""
    a = solver.solve(0.36, 0.02, solver.ref_envelope, limit=1).candidates[0].q
    b = solver.solve(0.36, 0.02, solver.ref_envelope, limit=1).candidates[0].q
    assert a == b


def test_never_projected_outside(solver, region):
    """A target outside is refused, never moved to the nearest inside point."""
    i = int(np.argmax(region.mask.any(1)[::-1] == 1))
    xmax = float(region.xs[len(region.xs) - 1 - i])
    r = solver.solve(xmax + 0.05, 0.0)
    assert not r.ok and r.candidates == []


# ------------------------------------------------------------------ region
def test_region_from_the_active_config(region, solver):
    s = region.summary()
    assert region.mask.sum() > 20
    assert s['z_grasp_base_link_m'] == pytest.approx(solver.z, abs=1e-4)
    # membership of every grid cell is the exact per-target test
    for i, j in list(zip(*np.nonzero(region.mask)))[:10]:
        assert solver.in_region(float(region.xs[i]), float(region.ys[j])).ok
    prop = standoff_proposal(region)
    lo, hi = prop['x_range_at_y0_m']
    assert lo <= prop['centre_x_m'] <= hi


def test_region_follows_the_ground_clearance(solver):
    """Raising the tip clearance above the grasp height empties the region."""
    s2, _ = build(ground_clearance_tip_m=solver.z + 0.1566 + 0.05)
    assert not s2.derive_region(0.04).mask.any()


def test_region_follows_the_chassis_model(region):
    """Longer chassis collision boxes move the region's inner edge outward. Every box
    of chassis_link grows (its collisions are named since the body-hull work; a regex
    on a bare <collision> tag ran past the link and mutated another one)."""
    m = re.search(r'<link name="chassis_link">.*?</link>', URDF, re.S)
    assert m, 'chassis_link not found'

    def grow(b):
        sx, sy, sz = (float(v) for v in b.group(2).split())
        return f'{b.group(1)}{sx + 0.10} {sy} {sz}"'
    link, n = re.subn(r'(<box size=")([^"]+)"', grow, m.group(0))
    assert n, 'no chassis collision box found'
    grown = URDF[:m.start()] + link + URDF[m.end():]
    s2, _ = build(urdf_xml=grown)
    r2 = s2.derive_region(0.02)
    r0, r1 = region.x_range_at(0.0), r2.x_range_at(0.0)
    assert r1 is None or r1[0] > r0[0], (r0, r1)


def test_region_follows_p_grip_pose(solver):
    """Another taught grip pose (limb2/3 changed) changes the grasp height."""
    p = yaml.safe_load((PKG / 'config' / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']['poses']['grip']
    s2, _ = build(grip=[p[0], p[1] - 60, p[2] + 60] + list(p[3:]))
    assert abs(s2.z - solver.z) > 1e-3


# ------------------------------------------------------------------ steep approach
# User decision 2026-11-16: pre-grasp pose aimed.pregrasp_height_m above the grasp, a
# vertical descent through Cartesian waypoints (same branch, same roll).
def _cands(solver, region, n=12):
    pts = [(float(region.xs[i]), float(region.ys[j])) for i, j in zip(*np.nonzero(region.mask))]
    out = []
    for x, y in pts[::max(1, len(pts) // n)]:
        r = solver.candidates(x, y, solver.ref_envelope, limit=3)
        assert r.ok, (x, y, r.detail)
        out += [(x, y, c) for c in r.candidates]
    return out


def test_pregrasp_is_the_grasp_raised_by_the_configured_height(solver, region):
    h = _aimed_cfg()['pregrasp_height_m']
    assert solver.pregrasp_height == h
    for x, y, c in _cands(solver, region):
        tp, tg = solver.kin.tcp(c.pregrasp), solver.kin.tcp(c.q)
        assert np.linalg.norm(tg[:3, 3] - [x, y, solver.z]) < 1e-7
        assert np.linalg.norm(tp[:3, 3] - [x, y, solver.z + h]) < 1e-7
        assert tp[2, 2] == pytest.approx(-1.0, abs=1e-9)                 # vertical
        assert c.descent[-1] == c.q
        # the SAME wrist roll and gripper (open) at every waypoint
        for q in c.descent:
            assert q[solver.roll_joint] == c.roll
            assert q[solver.gripper] == solver.hold[solver.gripper]
            assert solver.kin.within_limits(q)
            assert solver.flt.check_pose(q, solver.ref_envelope).ok


def test_descent_waypoints_are_vertical_and_within_the_lateral_bound(solver, region):
    """Waypoints on the vertical line at most descent_step_m apart, z falling; the
    joint-linear motion between them (sampled densely here, 40 per segment, not the
    solver's own sampling) within descent_max_lateral_m of the line and vertical."""
    a = _aimed_cfg()
    for x, y, c in _cands(solver, region):
        zs = []
        for q in c.descent:
            t = solver.kin.tcp(q)
            assert math.hypot(t[0, 3] - x, t[1, 3] - y) < 1e-7
            zs.append(t[2, 3])
        assert all(z1 < z0 for z0, z1 in zip(zs, zs[1:]))
        assert max(z0 - z1 for z0, z1 in zip(zs, zs[1:])) <= a['descent_step_m'] + 1e-9
        for qa, qb in zip(c.descent, c.descent[1:]):
            for f in np.linspace(0.0, 1.0, 41):
                t = solver.kin.tcp({j: qa[j] + f * (qb[j] - qa[j]) for j in qa})
                assert math.hypot(t[0, 3] - x, t[1, 3] - y) <= a['descent_max_lateral_m']
                assert t[2, 2] == pytest.approx(-1.0, abs=1e-3)


def test_reachable_at_the_grasp_but_not_at_the_pregrasp_is_out_of_reach(solver):
    """A target whose grasp pose has an IK in the limits but whose pre-grasp pose (or a
    descent waypoint) has none: OUT_OF_REACH, also from in_region and solve."""
    hit = None
    o, rr = solver.kin._ik.o1, solver.reach_radius()
    for x in np.arange(o[0] - rr, o[0] + rr, 0.01):            # searched, not typed
        for y in np.arange(o[1] - rr, o[1] + rr, 0.01):
            x, y = float(x), float(y)
            if solver._ik(x, y)[0] and not solver._ik(x, y, solver.pregrasp_height)[0]:
                hit = (x, y)
                break
        if hit:
            break
    assert hit, 'no target with an IK at the grasp but none at the pre-grasp height'
    r = solver.candidates(*hit, solver.ref_envelope)
    assert not r.ok and r.reason == OUT_OF_REACH, r
    assert 'pre-grasp' in r.detail or 'above the grasp' in r.detail, r.detail
    assert solver.in_region(*hit).reason == OUT_OF_REACH
    assert solver.solve(*hit, solver.ref_envelope).reason == OUT_OF_REACH


def test_the_region_follows_the_pregrasp_height(solver, region):
    """A higher pre-grasp pose shrinks the region (the config is what is read)."""
    s2, _ = build(descent={'pregrasp_height': solver.pregrasp_height + 0.10})
    r2 = s2.derive_region(0.02)
    assert r2.mask.sum() < region.mask.sum()
    assert not (r2.mask & ~region.mask).any()


def test_the_lateral_bound_is_enforced(solver):
    """A bound below what the joint-linear motion between waypoints deviates (but
    positive) refuses every candidate as OUT_OF_REACH."""
    s2, _ = build(descent={'descent_max_lateral': 1e-7})
    r = s2.candidates(0.36, 0.0, s2.ref_envelope)
    assert not r.ok and r.reason == OUT_OF_REACH and 'vertical line' in r.detail, r


@pytest.mark.parametrize('key', ['pregrasp_height', 'descent_step', 'descent_max_lateral'])
@pytest.mark.parametrize('bad', [0.0, -0.05, float('nan'), float('inf')])
def test_descent_parameters_have_no_default_and_are_validated(key, bad):
    with pytest.raises(ValueError):
        build(descent={key: bad})
    kw = _descent_kw()
    del kw[key]
    s, flt = build()
    with pytest.raises(TypeError):
        AimedGraspSolver(flt, s.grip_q, s.hold[s.gripper], ROLLS, 0.0, s.ref_envelope, **kw)
