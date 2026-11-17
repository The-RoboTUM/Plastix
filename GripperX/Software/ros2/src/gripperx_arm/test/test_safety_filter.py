"""SR-17 layer L3 safety filter against the REAL robot description and the ACTIVE
configuration (config/arm_safety_filter.yaml, arm_allowed_collisions.yaml,
gripperx_control/config/steer_servo.yaml). Expected verdicts are computed from the
model (FK of the URDF), never typed in; the mutation tests prove that a change of
the description changes the verdict.
"""
import ast
import math
import re
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from gripperx_arm.kinematics import ArmKinematics, as_box
from gripperx_arm.safety_filter import (
    FilterConfig, SafetyFilter, load_acm, load_link_hulls, mechanical_steer_ranges,
    unwrap_steer_reading)

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _desc_dir() -> Path:
    if (SRC / 'gripperx_description' / 'urdf').is_dir():
        return SRC / 'gripperx_description'
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('gripperx_description'))


def _steer_yaml() -> Path:
    p = SRC / 'gripperx_control' / 'config' / 'steer_servo.yaml'
    if p.is_file():
        return p
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('gripperx_control')) / 'config' / 'steer_servo.yaml'


@pytest.fixture(scope='module')
def urdf():
    import xacro
    return xacro.process_file(str(_desc_dir() / 'urdf' / 'gripperx_v1.urdf.xacro')).toxml()


@pytest.fixture(scope='module')
def active():
    sf = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['safety_filter']
    st = yaml.safe_load(_steer_yaml().read_text())['steer_servo_node']['ros__parameters']
    acm = yaml.safe_load((PKG / 'config' / 'arm_allowed_collisions.yaml').read_text())
    return sf, st, acm


def resolve_mesh(uri):
    return str(_desc_dir() / 'meshes' / uri.split('/')[-1])


def build(urdf_xml, active, hulls=None, **over):
    """Filter from the active config. hulls=None: loaded if either hull stage is on."""
    sf, st, acm = active
    sf = {**sf, **over}
    if hulls is None:
        hulls = sf['mesh_hull_stage'] or sf['body_hull_stage']
    kin = ArmKinematics(urdf_xml)
    rng = mechanical_steer_ranges(kin, sf['steer_joint_names'], st['steering_outward_sign'],
                                  st['steering_mechanical_outward_deg'],
                                  st['steering_mechanical_inward_deg'])
    cfg = FilterConfig(sf['ground_clearance_tip_m'], sf['ground_clearance_links_m'],
                       sf['tip_links'], sf['body_clearance_m'], sf['self_clearance_m'],
                       sf['max_joint_step_rad'], sf['self_contact_dip_m'],
                       mesh_hull_stage=sf['mesh_hull_stage'],
                       body_hull_stage=sf['body_hull_stage'],
                       body_hull_clearance_m=sf['body_hull_clearance_m'])
    return SafetyFilter(kin, load_acm(acm), cfg, rng, sf['ground_frame'],
                        hulls=load_link_hulls(urdf_xml, kin, resolve_mesh) if hulls else None)


@pytest.fixture(scope='module')
def flt(urdf, active):
    return build(urdf, active)


@pytest.fixture(scope='module')
def boxes_only(urdf, active):
    return build(urdf, active, hulls=False)


def lowest_tip_point(f: SafetyFilter, q) -> float:
    """Height above base_footprint of the lowest corner of any tip-link box (FK)."""
    nrm, c = f._ground
    low = math.inf
    for p in f.kin.collision_primitives(q):
        if p.link in f.cfg.tip_links:
            fr, h = as_box(p)
            low = min(low, float((((_CORNERS * h) @ fr[:3, :3].T + fr[:3, 3]) @ nrm).min() + c))
    return low


def ik(f, x, y, z, branch='front/elbow-'):
    sol = [s for s in f.kin.ik_tcp_down(x, y, z).solutions if s.branch == branch]
    assert sol, f'no {branch} solution for ({x}, {y}, {z})'
    q = dict(sol[0].q)
    q.setdefault('arm_gripper_joint', 0.0)
    return q


def pose_with_tip_at(f, height, x=0.38, y=0.0):
    """IK pose (gripper vertical) whose lowest tip-box point is `height` above the
    ground plane. The tip-box offset below the TCP is taken from the model."""
    probe = ik(f, x, y, -0.05)
    offset = f.kin.tcp(probe)[2, 3] - (lowest_tip_point(f, probe) - f._ground[1])
    q = ik(f, x, y, height - f._ground[1] + offset)
    assert abs(lowest_tip_point(f, q) - height) < 1e-6
    return q


# ---------------------------------------------------------------- A1: ground
def test_tip_half_a_centimetre_below_clearance_rejected_above_accepted(flt, active):
    clearance = active[0]['ground_clearance_tip_m']
    assert clearance == 0.01, 'D1 is 1 cm; the config changed - review this test'
    low = flt.check_pose(pose_with_tip_at(flt, clearance - 0.005))
    high = flt.check_pose(pose_with_tip_at(flt, clearance + 0.005))
    assert not low.ok and low.rule == 'ground', low
    assert high.ok, high


def test_boundary_is_sharp(flt):
    c = flt.cfg.ground_clearance_tip_m
    assert not flt.check_pose(pose_with_tip_at(flt, c - 1e-4)).ok
    assert flt.check_pose(pose_with_tip_at(flt, c + 1e-4)).ok


def mutate(urdf_xml, joint, fn):
    """Return the description with fn(<joint> element) applied."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_xml)
    el = next(j for j in root.findall('joint') if j.get('name') == joint)
    fn(el)
    return ET.tostring(root, encoding='unicode')


def _shift_origin(axis_index=None, dz=0.0):
    def fn(el):
        o = el.find('origin')
        v = [float(t) for t in o.get('xyz').split()]
        i = int(np.argmax(np.abs(v))) if axis_index is None else axis_index
        v[i] += math.copysign(dz, v[i]) if axis_index is None else dz
        o.set('xyz', ' '.join(map(str, v)))
    return fn


def _mutate_footprint(urdf_xml, dz):
    return mutate(urdf_xml, 'base_footprint_joint', _shift_origin(2, dz))


def test_mutated_ground_height_changes_the_verdict(urdf, active, flt):
    q = pose_with_tip_at(flt, flt.cfg.ground_clearance_tip_m - 0.005)
    assert not flt.check_pose(q).ok
    raised = build(_mutate_footprint(urdf, +0.01), active)
    assert raised.check_pose(q).ok, 'filter ignores the robot description (duplicated constant?)'


def test_mutated_link_length_changes_the_verdict(urdf, active, flt):
    q = pose_with_tip_at(flt, flt.cfg.ground_clearance_tip_m + 0.005)
    assert flt.check_pose(q).ok
    verdicts = []
    for dz in (+0.02, -0.02):        # lengthen / shorten the gripper link origin
        mutated = mutate(urdf, 'arm_gripper_joint', _shift_origin(None, dz))
        verdicts.append(build(mutated, active).check_pose(q).ok)
    assert False in verdicts, 'a 2 cm link change never changed the verdict'


def test_mutated_clearance_in_config_changes_the_verdict(urdf, active, flt):
    q = pose_with_tip_at(flt, 0.015)
    assert flt.check_pose(q).ok
    assert not build(urdf, active, ground_clearance_tip_m=0.02).check_pose(q).ok


# ---------------------------------------------------------------- joint limits
def test_joint_limit_from_the_description(urdf, active, flt):
    q = pose_with_tip_at(flt, 0.05)
    n = 'arm_limb5_joint'
    lo, hi = flt.kin.limits[n]
    assert not flt.check_pose({**q, n: hi + 1e-3}).ok
    tight = mutate(urdf, n, lambda el: el.find('limit').set('upper', str(q[n] - 0.01)))
    v = build(tight, active).check_pose(q)
    assert not v.ok and v.rule == 'joint_limit', v


# ---------------------------------------------------------------- self / ACM scope
def test_home_passes_and_home_neighbourhood_pair_scope_is_honoured(boxes_only):
    flt = boxes_only
    home = {n: 0.0 for n in flt.kin.joint_names}
    assert flt.check_pose(home).ok
    pair = ('arm_limb1_link', 'arm_limb3_link')
    assert pair in flt.acm.near_home
    # outside the neighbourhood (limb2 beyond its range) the same pair is NOT allowed
    lo, hi = flt.acm.home_range['arm_limb2_joint']
    out = {**home, 'arm_limb2_joint': lo - 0.02}
    v = flt.check_pose(out)
    assert not v.ok and v.rule == 'self' and 'arm_limb1_link vs arm_limb3_link' in v.detail, v
    # the pan joint moves both links together: it cannot take them out of the scope
    assert flt.check_pose({**home, 'arm_limb1_joint': 0.5}).ok


def test_path_leaving_the_neighbourhood_rejected_before_first_step(boxes_only):
    flt = boxes_only
    home = {n: 0.0 for n in flt.kin.joint_names}
    lo, hi = flt.acm.home_range['arm_limb2_joint']
    v = flt.check_path(home, {**home, 'arm_limb2_joint': lo - 0.1})
    assert not v.ok and v.rule == 'self'


# ---------------------------------------------------------------- body / wheels
def _wheel_hit_pose(flt):
    """IK target beside the front-right wheel (re-audit 2026-10-07 example)."""
    return ik(flt, 0.30, -0.12, -0.12, 'front/elbow-')


def test_mechanical_ranges_come_from_steer_servo_yaml(flt, active):
    st = active[1]
    for i, n in enumerate(active[0]['steer_joint_names']):
        lo, hi = flt.mechanical_ranges[n]
        assert math.isclose(hi - lo, math.radians(st['steering_mechanical_outward_deg'][i]
                                                  + st['steering_mechanical_inward_deg'][i]))
        # wider than the URDF working window on the outward side
        j = flt.kin._by_child[[c for c, jj in flt.kin._by_child.items() if jj.name == n][0]]
        assert lo < j.lower - 0.1 or hi > j.upper + 0.1


def test_steer_joint_order_matches_steer_servo_node(active):
    src = (SRC / 'gripperx_control' / 'src' / 'gripperx_control' / 'steer_servo_node.py')
    if not src.is_file():
        pytest.skip('gripperx_control source not beside this package')
    tree = ast.parse(src.read_text())
    names = next(ast.literal_eval(n.value) for n in ast.walk(tree)
                 if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') ==
                 'STEER_JOINT_NAMES')
    assert list(names) == active[0]['steer_joint_names']


def _covered(env_groups, probe):
    from gripperx_arm.kinematics import boxes_overlap
    return any(boxes_overlap(probe, smp) for bound, samples in env_groups for smp in samples
               if bound is None or boxes_overlap(probe, bound))


def test_wheel_between_urdf_limit_and_mechanical_stop(flt):
    """A steering angle inside the mechanical travel but OUTSIDE the URDF working
    limit (limp steering after a restart): parts of the wheel there lie outside the
    URDF-swept envelope, and both the default (mechanical) envelope and the measured
    envelope at that angle contain them."""
    from gripperx_arm.kinematics import Primitive, make_tf
    names = ['f_left_steer', 'f_right_steer', 'b_leftsteer', 'b_rightsteer']
    fr = 'f_right_steer'
    j = next(jj for jj in flt.kin._env_order if jj.name == fr)
    lo, hi = flt.mechanical_ranges[fr]
    assert hi > j.upper + math.radians(10), 'mechanical stop not beyond the URDF limit'
    urdf_env = flt._envelope(flt.kin.swept_environment(math.radians(5)), 'URDF limits').groups
    escaped = []
    for ang in np.linspace(j.upper + math.radians(5), hi, 8):
        wheel = [p for p in flt.kin.environment_at({fr: float(ang)}) if p.link == 'front_right_wheel']
        for p in wheel:
            fbox, h = as_box(p)
            for corner in (_CORNERS * h) @ fbox[:3, :3].T + fbox[:3, 3]:
                probe = Primitive('probe', 'box', make_tf(corner, (0, 0, 0)), (0.002,) * 3)
                if not _covered(urdf_env, probe):
                    escaped.append((float(ang), probe))
    assert escaped, 'beyond-URDF wheel never leaves the URDF envelope - test is vacuous'
    for ang, probe in escaped:
        assert _covered(flt.envelope_swept.groups, probe), f'mechanical sweep misses {ang}'
        measured = flt.envelope_measured({**{n: 0.0 for n in names}, fr: ang}, math.radians(1))
        assert _covered(measured.groups, probe), f'measured envelope misses {ang}'


def test_unknown_wheel_is_swept_over_its_mechanical_range(flt):
    names = ['f_left_steer', 'f_right_steer', 'b_leftsteer', 'b_rightsteer']
    env = flt.envelope_measured({n: None for n in names}, 0.05)
    lo, hi = flt.mechanical_ranges['f_right_steer']
    assert 'others swept' in env.description
    assert len(env.groups) == len(flt.envelope_swept.groups)


def test_unwrap_steer_reading():
    out, inw = math.radians(196.4), math.radians(51.2)
    # FL: outward = negative joint angle. Physical 190 deg outward is reported
    # wrapped as +170 deg (inward side) by a (-pi, pi] reading.
    assert math.isclose(unwrap_steer_reading(math.radians(170), -1, out, inw, 0.0),
                        math.radians(-190))
    assert math.isclose(unwrap_steer_reading(math.radians(-20), -1, out, inw, 0.0),
                        math.radians(-20))
    assert unwrap_steer_reading(math.radians(100), -1, out, inw, 0.0) is None


# ---------------------------------------------------------------- A2: path through body
@pytest.mark.parametrize('ends,body', [
    (((0.10, 0.25, 0.0, 'front/elbow-'), (0.30, 0.0, 0.05, 'front/elbow-')), 'chassis_link'),
    (((0.10, -0.25, 0.05, 'front/elbow-'), (0.35, -0.20, -0.05, 'front/elbow-')),
     'front_right_wheel'),
])
def test_path_with_valid_ends_through_the_body_is_rejected(flt, ends, body):
    """SR-17 A2: both ends valid, the joint-linear path between them crosses the body
    (an UNEXPANDED sub-step pose already collides): rejected as a whole, before a
    first step could be written. The wheel case runs against the default envelope,
    i.e. the wheel anywhere in its mechanical steering range, its limits included.
    The chassis case must cross the arm link's HULL, since a chassis box overlap is
    confirmed on it (2026-10-20; the old end points crossed only the arm's boxes)."""
    a, b = (ik(flt, *e[:3], e[3]) for e in ends)
    assert flt.check_pose(a).ok and flt.check_pose(b).ok
    crossing = [flt.check_pose(s) for s in flt.subdivide(a, b)]
    assert any(not v.ok and body in v.detail for v in crossing), \
        'the path no longer crosses that body - the model changed, pick other end points'
    if body == 'chassis_link':
        assert any(not v.ok and 'hull touches or penetrates' in v.detail for v in crossing)
    v = flt.check_path(a, b)
    assert not v.ok and v.rule == 'body', v


def test_motion_between_checked_poses_is_covered(flt):
    """Both end poses are clear, the motion between them sweeps through the chassis.
    Checked with NO subdivision (max_joint_step huge): only the end poses are
    evaluated, so only the model-derived motion bound can reject it."""
    a = ik(flt, 0.10, 0.25, 0.0, 'front/elbow-')
    b = ik(flt, 0.30, 0.0, 0.05, 'front/elbow-')
    assert flt.check_pose(a).ok and flt.check_pose(b).ok
    c = flt.cfg
    coarse = SafetyFilter(flt.kin, flt.acm, FilterConfig(
        c.ground_clearance_tip_m, c.ground_clearance_links_m, c.tip_links, c.body_clearance_m,
        c.self_clearance_m, 10.0), flt.mechanical_ranges, hulls=flt.hulls)
    assert len(coarse.refine([a, b])) == 2, 'expected no subdivision'
    assert not coarse.check_path(a, b).ok


def test_short_crossing_between_two_clear_poses_is_caught(flt):
    """Two clear poses straddling a SHORT crossing of a wheel (one sub-step deep),
    checked without subdivision: the motion bound must catch it. (Not tight enough
    to kill a weakened bound - test_motion_bound_covers_the_real_motion does that.)"""
    a = ik(flt, 0.10, -0.25, 0.05, 'front/elbow-')
    b = ik(flt, 0.35, -0.20, -0.05, 'front/elbow-')
    path = [a] + flt.subdivide(a, b)
    bad = [i for i, p in enumerate(path) if not flt.check_pose(p).ok]
    assert bad, 'path no longer crosses the wheel'
    lo, hi = bad[0] - 1, bad[-1] + 1
    assert flt.check_pose(path[lo]).ok and flt.check_pose(path[hi]).ok
    c = flt.cfg
    coarse = SafetyFilter(flt.kin, flt.acm, FilterConfig(
        c.ground_clearance_tip_m, c.ground_clearance_links_m, c.tip_links, c.body_clearance_m,
        c.self_clearance_m, 10.0), flt.mechanical_ranges, hulls=flt.hulls)
    v = coarse.check_path(path[lo], path[hi])
    assert not v.ok, v


def test_motion_bound_covers_the_real_motion(flt):
    """Soundness of the growth: for random poses and steps, every box corner at every
    configuration inside the joint box of the step lies within the growth of the
    nearer end pose. Computed from FK, so a weakened bound fails here."""
    rng = np.random.default_rng(3)
    names = flt.kin.joint_names
    worst = 0.0
    for _ in range(40):
        qa = {n: float(rng.uniform(-2.5, 2.5)) for n in names}
        dq = {n: float(rng.uniform(-0.15, 0.15)) for n in names}
        qb = {n: qa[n] + dq[n] for n in names}

        def corners(q):
            out = {}
            for p in flt.kin.collision_primitives(q):
                f, h = as_box(p)
                out.setdefault(p.link, []).append((_CORNERS * h) @ f[:3, :3].T + f[:3, 3])
            return {k: np.vstack(v) for k, v in out.items()}
        ca, cb = corners(qa), corners(qb)
        frames_a, frames_b = flt.kin.fk(qa), flt.kin.fk(qb)
        boxes_a, boxes_b = {}, {}
        for p in flt.kin.collision_primitives(frames=frames_a):
            boxes_a.setdefault(p.link, []).append(as_box(p))
        for p in flt.kin.collision_primitives(frames=frames_b):
            boxes_b.setdefault(p.link, []).append(as_box(p))
        absdq = {n: abs(v) for n, v in dq.items()}
        ga = flt._growth(frames_a, boxes_a, absdq)[1]
        gb = flt._growth(frames_b, boxes_b, absdq)[1]
        for _ in range(12):                   # random configurations inside the joint box
            t = {n: float(rng.uniform(0, 1)) for n in names}
            cm = corners({n: qa[n] + dq[n] * t[n] for n in names})
            for link in cm:
                da = np.linalg.norm(cm[link] - ca[link], axis=1)
                db = np.linalg.norm(cm[link] - cb[link], axis=1)
                need = float(np.minimum(da, db).max())   # per corner: nearer end pose
                worst = max(worst, need / max(ga[link], 1e-12))
                assert need <= ga[link] + 1e-9, (link, need, ga[link])
                assert need <= gb[link] + 1e-9, (link, need, gb[link])
    print(f'\n[motion bound] worst used fraction {worst:.2f}')


def test_small_step_near_a_valid_pose_is_approved(flt):
    """The motion bound must not reject everything: a 1 deg wrist step at a clear
    pose passes."""
    q = pose_with_tip_at(flt, 0.05)
    b = {**q, 'arm_limb5_joint': q['arm_limb5_joint'] + math.radians(1)}
    assert flt.check_path(q, b).ok


# ---------------------------------------------------------------- data
def test_description_without_arm_refused(active):
    import xacro
    gz = xacro.process_file(str(_desc_dir() / 'urdf' / 'gripperx_v1.gazebo.xacro')).toxml()
    with pytest.raises(ValueError):
        build(gz, active)


def test_acm_unknown_scope_refused():
    with pytest.raises(ValueError):
        load_acm({'allowed_pairs': [{'links': ['a', 'b'], 'scope': 'sometimes'}]})


def test_missing_joint_value_is_a_refusal(flt):
    q = {n: 0.0 for n in flt.kin.joint_names[:-1]}
    v = flt.check_pose(q)
    assert not v.ok and v.rule == 'data'


# ---------------------------------------------------------------- cost
def test_cost_per_setpoint_measured(flt, capsys):
    q = pose_with_tip_at(flt, 0.05)
    n = 50
    t0 = time.perf_counter()
    for _ in range(n):
        flt.check_pose(q)
    pose_ms = (time.perf_counter() - t0) / n * 1e3
    b = {**q, 'arm_limb5_joint': q['arm_limb5_joint'] + 0.01}
    t0 = time.perf_counter()
    for _ in range(n):
        flt.check_setpoints([q, b])
    step_ms = (time.perf_counter() - t0) / n * 1e3
    with capsys.disabled():
        print(f'\n[cost, this machine] check_pose {pose_ms:.2f} ms, '
              f'leader step (2 poses, grown) {step_ms:.2f} ms')


# ---------------------------------------------------------------- mesh hull stage
def test_active_config_uses_the_hull_stage(flt, active):
    assert active[0]['mesh_hull_stage'] is True
    assert set(flt.hulls) >= {'arm_limb1_link', 'arm_limb3_link'}


def test_hull_stage_clears_the_limb1_limb3_box_artefact(flt, boxes_only):
    """limb2 moved -15 deg from home: the boxes of limb1/limb3 overlap (artefact), the
    hulls of the meshes do not -> boxes alone reject, two-stage accepts."""
    q = {n: 0.0 for n in flt.kin.joint_names}
    q['arm_limb2_joint'] = math.radians(-15)
    v = boxes_only.check_pose(q)
    assert not v.ok and 'arm_limb1_link vs arm_limb3_link' in v.detail
    assert flt.check_pose(q).ok


def test_hull_stage_still_rejects_where_the_hulls_overlap(flt):
    """limb2 at -90 deg with the arm folded: the limb3 mesh reaches into limb1's hull
    (the meshes come within ~1 mm) - the conservative stage keeps rejecting."""
    q = {n: 0.0 for n in flt.kin.joint_names}
    q['arm_limb2_joint'] = math.radians(-90)
    v = flt.check_pose(q)
    assert not v.ok and v.rule == 'self' and 'hulls' in v.detail, v


def test_unfolding_limb3_first_leaves_home(flt):
    """The way out of the folded home under the current model: limb3 positive."""
    home = {n: 0.0 for n in flt.kin.joint_names}
    assert flt.check_path(home, {**home, 'arm_limb3_joint': math.radians(60)}).ok


def test_gjk_agrees_with_linear_programming():
    from scipy.optimize import linprog
    from gripperx_arm.kinematics import hulls_overlap
    rng = np.random.default_rng(1)

    def lp(a, b):
        na, nb = len(a), len(b)
        aeq = np.zeros((5, na + nb))
        aeq[:3, :na], aeq[:3, na:] = a.T, -b.T
        aeq[3, :na] = aeq[4, na:] = 1
        r = linprog(np.zeros(na + nb), A_eq=aeq, b_eq=[0, 0, 0, 1, 1], bounds=(0, None),
                    method='highs')
        return r.status == 0
    for _ in range(300):
        a = rng.normal(size=(15, 3))
        b = rng.normal(size=(15, 3)) + rng.normal(size=3) * 1.5
        assert hulls_overlap(a, b) == lp(a, b)


def test_hull_inflation_is_a_distance():
    from gripperx_arm.kinematics import hulls_overlap
    cube = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float)
    other = cube + [1.05, 0, 0]                     # 5 cm gap along x
    assert not hulls_overlap(cube, other, 0.04)
    assert hulls_overlap(cube, other, 0.06)


def test_hull_stage_cost_measured(flt, capsys):
    q = {n: 0.0 for n in flt.kin.joint_names}
    q['arm_limb2_joint'] = math.radians(-15)        # forces the hull stage for limb1/limb3
    n = 30
    t0 = time.perf_counter()
    for _ in range(n):
        flt.check_pose(q)
    ms = (time.perf_counter() - t0) / n * 1e3
    with capsys.disabled():
        print(f'\n[cost, this machine] check_pose incl. one hull pair {ms:.2f} ms')


# ---------------------------------------------------------------- angle windows
def test_tick_calibration_wraps_into_the_range_centre_window():
    from gripperx_arm.kinematics import TickCalibration
    c = math.radians(-67)                         # e.g. limb2 travel ~[-192, +58] deg
    cal = TickCalibration(['j'], [2048], [1], 4096, [c])
    for deg in (-192, -180, -100, 0, 58, 112):
        t = 2048 + deg / 360 * 4096
        q = cal.to_radians([t])['j']
        assert c - math.pi <= q < c + math.pi
        assert math.isclose(math.remainder(q - math.radians(deg), 2 * math.pi), 0, abs_tol=1e-9)
    assert math.isclose(cal.to_radians([2048 - 192 / 360 * 4096])['j'], math.radians(-192))
    plain = TickCalibration(['j'], [2048], [1], 4096)
    assert math.isclose(plain.to_radians([2048 - 192 / 360 * 4096])['j'], math.radians(168))


def test_limits_beyond_pi_are_honoured(urdf, active):
    """geometry.yaml may carry limits outside +-pi once the windows are centred."""
    f = build(mutate(urdf, 'arm_limb2_joint',
                     lambda el: (el.find('limit').set('lower', '-3.35'),
                                 el.find('limit').set('upper', '1.0'))), active)
    home = {n: 0.0 for n in f.kin.joint_names}
    assert f.kin.limits['arm_limb2_joint'] == (-3.35, 1.0)
    v = f.check_pose({**home, 'arm_limb2_joint': -3.4})
    assert not v.ok and v.rule == 'joint_limit'
    v = f.check_pose({**home, 'arm_limb2_joint': 1.05})
    assert not v.ok and v.rule == 'joint_limit'


def test_step_across_plus_minus_pi_is_a_short_step(flt):
    """+179 deg -> -179 deg is 2 deg of travel, never 358."""
    a = {n: 0.0 for n in flt.kin.joint_names}
    a['arm_limb5_joint'] = math.radians(179)
    b = {**a, 'arm_limb5_joint': math.radians(-179)}
    steps = flt.subdivide(a, b)
    assert len(steps) <= math.ceil(math.radians(2) / flt.cfg.max_joint_step_rad)
    assert max(abs(s['arm_limb5_joint']) for s in steps) <= math.radians(181) + 1e-9


def test_stop_protected_pair_is_not_checked(urdf, active):
    """A pair certified stop_protected is skipped; the same pose is rejected for that
    pair without the certificate (synthetic certificate - the real limb1/limb3 pair
    is NOT certified, its meshes penetrate inside the stops)."""
    sf, st, acm = active
    kin = ArmKinematics(urdf)
    rng = mechanical_steer_ranges(kin, sf['steer_joint_names'], st['steering_outward_sign'],
                                  st['steering_mechanical_outward_deg'],
                                  st['steering_mechanical_inward_deg'])
    cfg = FilterConfig(sf['ground_clearance_tip_m'], sf['ground_clearance_links_m'],
                       sf['tip_links'], sf['body_clearance_m'], sf['self_clearance_m'],
                       sf['max_joint_step_rad'], sf['self_contact_dip_m'])
    hulls = load_link_hulls(urdf, kin, resolve_mesh)
    stop = {'pairs': [{'links': ['arm_limb1_link', 'arm_limb3_link'],
                       'category': 'stop_protected'}]}
    with_c = SafetyFilter(kin, load_acm(acm, stop), cfg, rng, hulls=hulls)
    without = SafetyFilter(kin, load_acm(acm), cfg, rng, hulls=hulls)
    q = {n: 0.0 for n in kin.joint_names}
    q['arm_limb2_joint'] = math.radians(-60)
    v0 = without.check_pose(q)
    assert not v0.ok and 'arm_limb1_link vs arm_limb3_link' in v0.detail
    v1 = with_c.check_pose(q)
    assert v1.ok or 'arm_limb1_link vs arm_limb3_link' not in v1.detail, v1


# ------------------------------------------------------------------ cut and resumed (robot 2026-11-02)
class _StopAfter:
    def __init__(self, n):
        self.n = n

    def __call__(self):
        self.n -= 1
        return self.n < 0


def _line(flt, joint, a, b, n):
    q = {x: 0.0 for x in flt.kin.joint_names}
    return [{**q, joint: a + (b - a) * i / n} for i in range(n + 1)]


def _in_pieces(flt, poses, env=None, each=2, **kw):
    """A check cut every `each` poses and resumed, as a search does over its slices."""
    at = 0
    for _ in range(10 * len(poses)):
        v = flt.check_setpoints(poses, env, stop=_StopAfter(each), resume_from=at, **kw)
        if v.rule != 'timeout':
            return v
        assert v.resume_at > at
        at = v.resume_at
    raise AssertionError('no end')


def assert_cut_changes_nothing(flt, poses, at, **kw):
    """A check cut before pose `at` and continued as a search does (from resume_at, 0 =
    from the start) ends with the verdict of the whole check, in every field."""
    def sig(v):
        return (v.ok, v.rule, v.index, v.checked, v.detail, v.rising_start, v.rising_end)
    cut = flt.check_setpoints(poses, stop=_StopAfter(at), **kw)
    assert cut.rule == 'timeout', cut
    whole = flt.check_setpoints(poses, **kw)
    assert sig(flt.check_setpoints(poses, resume_from=cut.resume_at, **kw)) == sig(whole)
    return cut, whole


def test_a_stopped_check_is_a_timeout_and_says_where_to_go_on(flt):
    poses = _line(flt, 'arm_limb1_joint', 0.0, 0.2, 10)
    v = flt.check_setpoints(poses, stop=_StopAfter(3))
    assert not v.ok and v.rule == 'timeout' and v.resume_at == 3


def test_a_resumed_check_finds_what_the_whole_check_finds(flt):
    ok = _line(flt, 'arm_limb1_joint', 0.0, 0.2, 10)
    assert flt.check_setpoints(ok).ok and _in_pieces(flt, ok).ok
    lo, _hi = flt.kin.limits['arm_limb1_joint']
    bad = _line(flt, 'arm_limb1_joint', lo + 0.05, lo - 0.05, 10)  # past the limit half-way
    whole = flt.check_setpoints(bad)
    cut = _in_pieces(flt, bad)
    assert not whole.ok and not cut.ok
    assert (whole.rule, whole.index) == (cut.rule, cut.index)


def test_a_check_with_a_trace_starts_over(flt):
    poses = _line(flt, 'arm_limb1_joint', 0.0, 0.2, 10)
    v = flt.check_setpoints(poses, stop=_StopAfter(3), trace=[])
    assert v.rule == 'timeout' and v.resume_at == 0
