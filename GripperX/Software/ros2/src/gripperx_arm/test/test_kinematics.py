"""Tests for gripperx_arm.kinematics against the real robot description.

The model is loaded from gripperx_description's xacro, never from constants in
this file. The expected values that are written here come from artefacts that
were produced INDEPENDENTLY of the articulated model, so a wrong joint origin,
mount, mesh or TCP in the xacro makes them fail:
  * meshes/arm_stowed.stl - the rigid arm baked from the 2026-08-20 CAD import;
  * the landmark and per-limb tables of claude/docs/ARM_HOME_REFERENCE_2026-08-25.md
    (measured on that mesh, in chassis_link == base_link).
"""
import math
import struct
from pathlib import Path

import numpy as np
import pytest

from gripperx_arm.kinematics import ArmKinematics, TickCalibration, wrap

SRC_DESC = Path(__file__).resolve().parents[2] / 'gripperx_description'


def _desc_dir() -> Path:
    if (SRC_DESC / 'urdf').is_dir():
        return SRC_DESC
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('gripperx_description'))


def _urdf(name):
    import xacro
    return xacro.process_file(str(_desc_dir() / 'urdf' / name)).toxml()


def _stl(path) -> np.ndarray:
    d = Path(path).read_bytes()
    n = struct.unpack('<I', d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(float)


def _tf(T, v):
    return v @ T[:3, :3].T + T[:3, 3]


@pytest.fixture(scope='module')
def urdf_xml():
    return _urdf('gripperx_v1.urdf.xacro')


@pytest.fixture(scope='module')
def kin(urdf_xml):
    return ArmKinematics(urdf_xml)


@pytest.fixture(scope='module')
def posed_meshes(kin, urdf_xml):
    """Visual mesh vertices of every arm link, posed at q = 0, in base_link."""
    import xml.etree.ElementTree as ET
    frames = kin.fk()
    out = {}
    for link in ET.fromstring(urdf_xml).findall('link'):
        if link.get('name') not in kin.links or link.find('visual') is None:
            continue
        fn = link.find('visual/geometry/mesh').get('filename').split('/')[-1]
        out[link.get('name')] = _tf(frames[link.get('name')], _stl(_desc_dir() / 'meshes' / fn))
    return out


# ------------------------------------------------------------------ structure
def test_joint_names_and_servo_order(kin):
    assert kin.joint_names == ['arm_limb1_joint', 'arm_limb2_joint', 'arm_limb3_joint',
                               'arm_limb4_joint', 'arm_limb5_joint', 'arm_gripper_joint']


def test_gazebo_model_has_the_arm_fixed():
    gz = ArmKinematics(_urdf('gripperx_v1.gazebo.xacro'))
    assert gz.joint_names == []


# ------------------------------------------------- q = 0 is the frozen CAD pose
def test_q0_reproduces_arm_stowed_mesh(kin, posed_meshes):
    """Bounding box of the posed limbs vs. the rigid mesh, in arm_base_link
    (the frame arm_stowed.stl was stored in). <= 1 mm on all six faces."""
    inv = np.linalg.inv(kin.mount)
    v = _tf(inv, np.vstack(list(posed_meshes.values())))
    ref = _stl(_desc_dir() / 'meshes' / 'arm_stowed.stl')
    assert np.abs(v.min(0) - ref.min(0)).max() < 1e-3
    assert np.abs(v.max(0) - ref.max(0)).max() < 1e-3


GROUND = 0.1566     # base_footprint -> base_link, as used by the reference table


@pytest.mark.parametrize('axis,fn,expected', [
    (2, np.max, 0.3694),     # highest point (gripper tip), h 526.0 mm
    (2, np.min, 0.0690),     # lowest arm point, h 225.6 mm
    (0, np.min, 0.0717),     # rearmost
    (0, np.max, 0.2584),     # forwardmost
    (1, np.min, -0.0312),
    (1, np.max, 0.0377),
])
def test_q0_landmarks_in_base_link(posed_meshes, axis, fn, expected):
    v = np.vstack(list(posed_meshes.values()))
    assert fn(v[:, axis]) == pytest.approx(expected, abs=1e-3)


# per-limb envelopes of the reference table: x range and height over ground [mm]
LIMB_TABLE = {
    'arm_limb1_link': (0.1594, 0.2220, 225.6, 336.2),
    'arm_limb2_link': (0.0717, 0.2143, 312.8, 363.2),
    'arm_limb3_link': (0.0768, 0.2351, 338.6, 376.0),
    'arm_limb4_link': (0.2036, 0.2389, 347.3, 425.2),
    'arm_limb5_link': (0.1927, 0.2584, 419.6, 525.3),
    'arm_gripper_link': (0.1925, 0.2143, 434.2, 526.0),
}


@pytest.mark.parametrize('link', sorted(LIMB_TABLE))
def test_q0_per_limb_envelope(posed_meshes, link):
    x0, x1, h0, h1 = LIMB_TABLE[link]
    v = posed_meshes[link]
    assert v[:, 0].min() == pytest.approx(x0, abs=1e-3)
    assert v[:, 0].max() == pytest.approx(x1, abs=1e-3)
    assert (v[:, 2].min() + GROUND) * 1e3 == pytest.approx(h0, abs=1.0)
    assert (v[:, 2].max() + GROUND) * 1e3 == pytest.approx(h1, abs=1.0)


def test_tcp_sits_between_the_fingertips(kin, posed_meshes):
    """TCP on the jaw tips: nothing of the gripper sticks out beyond it along the
    approach axis by more than the 1 mm the moving jaw overhangs in CAD."""
    t = kin.tcp()
    for link in ('arm_limb5_link', 'arm_gripper_link'):
        along = (posed_meshes[link] - t[:3, 3]) @ t[:3, 2]
        assert along.max() < 1.5e-3
        assert along.max() > -1.5e-3 if link == 'arm_limb5_link' else True


# ------------------------------------------------------------- collision boxes
def _stl_tris(path):
    return _stl(path).reshape(-1, 3, 3)


@pytest.mark.parametrize('q', [None, [0.7, -1.1, 0.9, 0.4, -2.0, 0.3]])
def test_collision_boxes_enclose_their_meshes(kin, urdf_xml, q):
    """Every mesh vertex and every sampled surface point lies inside at least
    one box of its link, in any pose."""
    import xml.etree.ElementTree as ET
    frames = kin.fk(q)
    rng = np.random.default_rng(1)
    prims = {}
    for p in kin.collision_primitives(frames=frames):
        prims.setdefault(p.link, []).append(p)
    checked = set()
    for link in ET.fromstring(urdf_xml).findall('link'):
        name = link.get('name')
        if name not in kin.links or link.find('visual') is None:
            continue
        tris = _stl_tris(_desc_dir() / 'meshes' / link.find('visual/geometry/mesh')
                         .get('filename').split('/')[-1])
        u, v = rng.random((2, len(tris)))
        m = u + v > 1
        u[m], v[m] = 1 - u[m], 1 - v[m]
        pts = np.vstack([tris.reshape(-1, 3), tris[:, 0] + u[:, None] * (tris[:, 1] - tris[:, 0])
                         + v[:, None] * (tris[:, 2] - tris[:, 0])])
        world = _tf(frames[name], pts)
        inside = np.zeros(len(world), bool)
        for p in prims[name]:
            assert p.shape == 'box'
            inside |= np.all(np.abs(_tf(np.linalg.inv(p.frame), world)) <= np.array(p.size) / 2, axis=1)
        assert inside.all(), f'{name}: {int((~inside).sum())} points outside its boxes'
        checked.add(name)
    assert checked == set(prims)


def test_boxes_overlap_matches_the_separating_axis_cases():
    from gripperx_arm.kinematics import Primitive, boxes_overlap, make_tf
    a = Primitive('a', 'box', make_tf([0, 0, 0], [0, 0, 0]), (0.1, 0.1, 0.1))
    assert boxes_overlap(a, Primitive('b', 'box', make_tf([0.099, 0, 0], [0, 0, 0]), (0.1, 0.1, 0.1)))
    assert not boxes_overlap(a, Primitive('b', 'box', make_tf([0.101, 0, 0], [0, 0, 0]), (0.1, 0.1, 0.1)))
    # rotated 45 deg about z: corner reaches 0.05*sqrt(2) = 0.0707
    assert boxes_overlap(a, Primitive('b', 'box', make_tf([0.12, 0, 0], [0, 0, math.pi / 4]), (0.1, 0.1, 0.1)))
    assert not boxes_overlap(a, Primitive('b', 'box', make_tf([0.122, 0, 0], [0, 0, math.pi / 4]), (0.1, 0.1, 0.1)))


def test_allowed_collision_file_is_current():
    """Regenerated from the SOURCE description and config, like every other test
    here - not from whatever happens to be installed."""
    from gripperx_arm import generate_acm
    cfg = Path(__file__).resolve().parents[1] / 'config'
    text = generate_acm.render(desc=_desc_dir(), cfg=cfg)
    assert (cfg / 'arm_allowed_collisions.yaml').read_text() == text, \
        'regenerate with: python3 -m gripperx_arm.generate_acm'


def test_home_neighbourhood_has_no_overlap_outside_the_allowed_pairs(kin):
    """Fresh samples (other seed, more of them) of home +- home_pose_tolerance_rad
    (TO-VERIFY, arm_calibration.yaml): every overlapping pair must be listed.
    Reports the tolerance up to which the exact-home pair set alone holds."""
    import yaml
    from gripperx_arm import generate_acm as g
    cfg = Path(__file__).resolve().parents[1] / 'config'
    doc = yaml.safe_load((cfg / 'arm_allowed_collisions.yaml').read_text())
    assert doc['neighbourhood']['status'] == 'TO-VERIFY'
    allowed = {tuple(sorted(p['links'])) for p in doc['allowed_pairs']}
    home, tol, rng_ = g.neighbourhood(kin, cfg)
    groups = kin.swept_environment()
    stray = set()
    for q in g.sample_poses(kin, home, rng_, 1.0, n_random=1500, seed=11):
        stray |= g.overlapping_pairs(kin, dict(zip(kin.joint_names, q)), groups) - allowed
    print(f"tolerance {math.degrees(tol):.1f} deg; exact-home set holds up to "
          f"{doc['neighbourhood']['exact_home_set_holds_up_to_deg']} deg")
    assert not stray, f'pairs overlapping near home but not allowed: {sorted(stray)}'


def test_allowed_pairs_contain_every_joint_neighbour(kin):
    import yaml
    doc = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config'
                          / 'arm_allowed_collisions.yaml').read_text())
    pairs = {tuple(sorted(p['links'])) for p in doc['allowed_pairs']}
    for a, b in [('arm_limb1_link', 'chassis_link'), ('arm_limb1_link', 'arm_limb2_link'),
                 ('arm_limb2_link', 'arm_limb3_link'), ('arm_limb3_link', 'arm_limb4_link'),
                 ('arm_limb4_link', 'arm_limb5_link'), ('arm_gripper_link', 'arm_limb5_link')]:
        assert tuple(sorted((a, b))) in pairs
    assert all(v['min_clearance_mm'] > 0 for v in doc['padding'].values())


# ------------------------------------------------------------------------- IK
def _down_q4(kin, q):
    """Find q4 making the approach axis point straight down, by brute force on
    FK only (independent of the IK code)."""
    def err(a):
        qq = list(q)
        qq[3] = a
        return float(kin.tcp(qq)[2, 2] + 1.0)     # z component of approach + 1
    grid = np.linspace(-math.pi, math.pi, 721)
    a = grid[int(np.argmin([err(g) for g in grid]))]
    lo, hi = a - 0.01, a + 0.01
    g = (math.sqrt(5) - 1) / 2
    for _ in range(80):
        c, d = hi - g * (hi - lo), lo + g * (hi - lo)
        if err(c) < err(d):
            hi = d
        else:
            lo = c
    return (lo + hi) / 2


def _assert_tcp_down_at(kin, q, target):
    t = kin.tcp(q)
    assert np.linalg.norm(t[:3, 3] - target) < 1e-9
    assert t[2, 2] == pytest.approx(-1.0, abs=1e-12)


def test_ik_round_trip_recovers_the_generating_joints(kin):
    rng = np.random.default_rng(7)
    checked = 0
    for _ in range(40):
        q = list(rng.uniform(-math.pi, math.pi, 6))
        q[3] = _down_q4(kin, q)
        if kin.tcp(q)[2, 2] > -1 + 1e-10:
            continue
        target = kin.tcp(q)[:3, 3]
        res = kin.ik_tcp_down(*target, roll=q[4], hold={'arm_gripper_joint': q[5]})
        assert res.solutions
        for s in res.solutions:
            _assert_tcp_down_at(kin, s.q, target)
        diffs = [max(abs(wrap(s.q[n] - v)) for n, v in zip(kin.joint_names, q))
                 for s in res.solutions]
        # q4 comes from a minimum search on 1 + cos(error), which is flat to
        # second order: it is only good to ~1e-5 rad, so the bound is 1e-4.
        assert min(diffs) < 1e-4
        checked += 1
    assert checked >= 30


def test_ik_targets_on_a_floor_grid(kin):
    """Every solution of every target must put the TCP there, pointing down."""
    n = 0
    for x in np.linspace(0.20, 0.50, 7):
        for y in np.linspace(-0.20, 0.20, 5):
            for z in (-0.15, -0.10, 0.0):
                res = kin.ik_tcp_down(x, y, z)
                for s in res.solutions:
                    _assert_tcp_down_at(kin, s.q, np.array([x, y, z]))
                    n += 1
    assert n > 100


def test_ik_tcp_yaw_sets_the_jaw_heading(kin):
    for yaw in (-2.0, 0.0, 0.5, 3.0):
        res = kin.ik_tcp_down(0.32, 0.04, -0.12, tcp_yaw=yaw)
        for s in res.solutions:
            t = kin.tcp(s.q)
            assert wrap(math.atan2(t[1, 0], t[0, 0]) - yaw) == pytest.approx(0.0, abs=1e-9)


def test_ik_unreachable(kin):
    res = kin.ik_tcp_down(1.5, 0.0, 0.0)
    assert not res.reachable and res.solutions == [] and res.reason == 'out of reach'
    o = kin.fk()['arm_limb1_link'][:3, 3]
    res = kin.ik_tcp_down(o[0], o[1], 0.0)
    assert not res.reachable and 'lateral offset' in res.reason


def test_ik_four_branches_inside_the_workspace(kin):
    res = kin.ik_tcp_down(0.30, 0.0, -0.10)
    assert res.reachable
    assert sorted(s.branch for s in res.solutions) == [
        'back/elbow+', 'back/elbow-', 'front/elbow+', 'front/elbow-']


# --------------------------------------------------------------- calibration
HOME = [2108, 1003, 3091, 1035, 2148, 1527]


def test_tick_calibration_anchor_and_round_trip(kin):
    cal = TickCalibration(kin.joint_names, HOME, [1, -1, 1, -1, 1, 1])
    assert all(v == 0.0 for v in cal.to_radians(HOME).values())
    for ticks in ([0, 4095, 2048, 100, 3000, 1500], [2108, 1003, 3091, 1035, 2148, 1527]):
        q = cal.to_radians(ticks)
        assert all(-math.pi < v <= math.pi for v in q.values())
        assert cal.to_ticks(q) == [t % 4096 for t in ticks]
    q = cal.to_radians([2108 + 1024, 1003 + 1024, 3091, 1035, 2148, 1527])
    assert q['arm_limb1_joint'] == pytest.approx(math.pi / 2)
    assert q['arm_limb2_joint'] == pytest.approx(-math.pi / 2)


def test_calibration_file_is_consistent_with_the_model(kin):
    import yaml
    p = Path(__file__).resolve().parents[1] / 'config' / 'arm_calibration.yaml'
    c = yaml.safe_load(p.read_text())['arm_joint_state_relay']['ros__parameters']
    assert c['urdf_joints'] == kin.joint_names
    cal = TickCalibration(c['urdf_joints'], c['zero_ticks'], c['signs'], c['ticks_per_rev'])
    assert len(cal.joints) == len(c['servo_names']) == 6


# ------------------------------------------------- external reference: upstream
# TheRobotStudio SO-ARM100, Simulation/SO101/so101_new_calib.urdf at commit
# 385e8d7c68e24945df6c60d9bd68837a4b7411ae (2025-07-02),
# https://github.com/TheRobotStudio/SO-ARM100/blob/385e8d7c68e24945df6c60d9bd68837a4b7411ae/Simulation/SO101/so101_new_calib.urdf
# Read off its joint origins by hand (metres):
#   shoulder_lift  z -0.0542                      -> shoulder drop 54.2 mm
#   elbow_flex     xy (-0.11257, -0.028)          -> upper arm 116.0 mm
#   wrist_flex     xy (-0.1349, 0.0052)           -> forearm 135.0 mm
#   wrist_roll     (0, -0.0611, 0.0181)           -> 61.1 mm off the flex axis,
#                                                    18.1 mm along it
# An EXTERNAL check of our CAD-derived geometry. Upstream LIMITS are not used.
UPSTREAM = {'shoulder_drop': 0.0542, 'upper_arm': 0.11600, 'forearm': 0.13500,
            'wrist_perp': 0.0611, 'wrist_along': 0.0181}


def _axis(kin, f, name):
    j = kin.joints[name]
    return f[j.child][:3, 3], f[j.child][:3, :3] @ j.axis


def _perp(v, u):
    return v - (v @ u) * u


def test_link_lengths_match_upstream_so101_within_1mm(kin):
    f = kin.fk()
    o1, u1 = _axis(kin, f, 'arm_limb1_joint')
    c2, u2 = _axis(kin, f, 'arm_limb2_joint')
    c3, _ = _axis(kin, f, 'arm_limb3_joint')
    c4, u4 = _axis(kin, f, 'arm_limb4_joint')
    c5, _ = _axis(kin, f, 'arm_limb5_joint')
    got = {'shoulder_drop': abs((c2 - o1) @ u1),
           'upper_arm': np.linalg.norm(_perp(c3 - c2, u2)),
           'forearm': np.linalg.norm(_perp(c4 - c3, u2)),
           'wrist_perp': np.linalg.norm(_perp(c5 - c4, u4)),
           'wrist_along': abs((c5 - c4) @ u4)}
    for k, v in UPSTREAM.items():
        assert got[k] == pytest.approx(v, abs=1e-3), k
    assert got['upper_arm'] + got['forearm'] == pytest.approx(0.251, abs=1e-3)


def test_ik_pan_rotation_hand_computed(kin):
    """Turn the target 90 deg about the pan axis by hand: per branch the pan
    joint must change by 90 deg and the pitch joints not at all. Tolerance 1e-5:
    the hand rotation is about base z, the pan axis is tilted 3.5e-7 rad from it
    by the 6-decimal arm_mount_rpy (3.141593 vs pi)."""
    o = kin.fk()['arm_limb1_link'][:3, 3]
    a = np.array([0.30, 0.0, -0.10])
    rel = a - o
    b = np.array([o[0] - rel[1], o[1] + rel[0], a[2]])     # rotated +90 deg about z
    ra, rb = kin.ik_tcp_down(*a), kin.ik_tcp_down(*b)
    sa = {s.branch: s.q for s in ra.solutions}
    sb = {s.branch: s.q for s in rb.solutions}
    assert sa.keys() == sb.keys() and len(sa) == 4
    for br in sa:
        assert abs(wrap(sb[br]['arm_limb1_joint'] - sa[br]['arm_limb1_joint'])) \
            == pytest.approx(math.pi / 2, abs=1e-5)
        for n in ('arm_limb2_joint', 'arm_limb3_joint', 'arm_limb4_joint'):
            assert sb[br][n] == pytest.approx(sa[br][n], abs=1e-5)


def test_ik_reach_limit_hand_computed(kin):
    """With the elbow fully stretched the wrist-flex axis is upper arm + forearm
    = 251 mm (upstream) from the shoulder-lift axis: the IK must reach a target
    there and nothing 2 mm beyond it, on the same ray."""
    f = kin.fk()
    c2, u2 = _axis(kin, f, 'arm_limb2_joint')
    # tcp-down: the wrist-flex axis sits a fixed offset above/behind the TCP;
    # read that offset from one solution, then move the wrist along the ray.
    s = kin.ik_tcp_down(0.30, 0.0, -0.05).solutions[0]
    fs = kin.fk(s.q)
    c4s, _ = _axis(kin, fs, 'arm_limb4_joint')
    c2s, u2s = _axis(kin, fs, 'arm_limb2_joint')
    tcp = fs['arm_tcp_link'][:3, 3]
    off = tcp - c4s
    ray = _perp(c4s - c2s, u2s)
    ray /= np.linalg.norm(ray)
    lat = (c4s - c2s) @ u2s * u2s
    for dist, reachable in ((0.251 - 0.002, True), (0.251 + 0.002, False)):
        tgt = c2s + lat + dist * ray + off
        res = kin.ik_tcp_down(*tgt)
        hit = [x for x in res.solutions if x.branch.startswith(s.branch.split('/')[0])]
        assert bool(hit) is reachable, dist


# ------------------------------------------------------------ swept steering
def test_swept_envelopes_enclose_the_body_at_any_steering_angle(kin):
    from gripperx_arm.kinematics import as_box
    groups = {g.joint: g for g in kin.swept_environment() if g.joint}
    assert len(groups) == 4
    rng = np.random.default_rng(3)
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    for _ in range(20):
        q = {n: float(rng.uniform(*g.range)) for n, g in groups.items()}
        for p in kin.environment_at(q):
            grp = next((g for g in groups.values()
                        if p.link in {s.link for s in g.samples}), None)
            if grp is None:
                continue
            f, h = as_box(p)
            g0 = np.linspace(-1, 1, 5)
            pts = np.array([[x, y, z] for x in g0 for y in g0 for z in g0]) * h
            pts = np.vstack([pts, corners * h]) @ f[:3, :3].T + f[:3, 3]
            inside = np.zeros(len(pts), bool)
            for smp in grp.samples:
                fs, hs = as_box(smp)
                loc = (pts - fs[:3, 3]) @ fs[:3, :3]
                inside |= np.all(np.abs(loc) <= hs + 1e-9, axis=1)
            assert inside.all(), (p.link, q)


@pytest.mark.parametrize('target,joint', [((0.30, -0.12, -0.12), 'f_right_steer'),
                                          ((0.30, 0.12, -0.12), 'f_left_steer')])
def test_audit_targets_are_caught_by_the_steering_envelope(kin, target, joint):
    """Audit 2026-10-07: these IK targets put the arm where the front wheel can
    be at some steering angles. The swept envelope must report it."""
    from gripperx_arm.kinematics import boxes_overlap
    grp = next(g for g in kin.swept_environment() if g.joint == joint)
    for s in kin.ik_tcp_down(*target).solutions:
        arm = kin.collision_primitives(s.q)
        assert any(boxes_overlap(a, b) for a in arm for b in grp.samples)
