"""lidar_link in the arm safety filter (SR-17 rule 4(b)), as the robot runs it: the
description built WITH the lidar (use_lidar:=true; the other filter tests build it
without). Active config and calibration.

* The collision model of lidar_link is the user's measured envelope (2026-10-12/13):
  a 40 x 40 x 35 mm box standing on the lid, front edge flush with it, laterally
  centred under the arm - geometry.yaml lidar_envelope_* (read here through
  gripperx_geometry.constants, never duplicated). It replaced a 70 x 50 mm cylinder
  (a 70 x 70 x 50 mm box to the filter) that produced false contacts on 2026-10-12/13.
* arm_limb1_link vs lidar_link stays excluded (user decision 2026-10-12, unchanged
  by the model fix - lifting it is the user's call).
* Every other arm link vs lidar_link stays checked. The corrected box sits on the
  lid, partly inside chassis_link's padded boxes (four since 2026-10-13), so the
  chassis can be hit first; the witnesses below therefore allow that one link vs
  the chassis, which leaves lidar_link as the first body contact. arm_limb2_link cannot reach the box
  inside its joint limits (offline minimum 20 mm, box vs box) and has no witness."""
import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_safety_filter import PKG, _desc_dir, _steer_yaml, build  # noqa: E402
from test_planner import JOINTS, _active_executor  # noqa: E402

from gripperx_arm.kinematics import as_box  # noqa: E402
from gripperx_arm.planner import Planner  # noqa: E402
from gripperx_geometry import constants as C  # noqa: E402

PAIR = ['arm_limb1_link', 'lidar_link']
# Measured on the robot 2026-10-12 (I and P refused there: limb1 vs lidar, clearance 0)
ROBOT_TICKS = [2896, 1254, 3068, 1520, 2117, 2651]
# Found offline (random start + Nelder-Mead into the box, 2026-10-13): this link INTO
# the corrected lidar box, every other check passing once link vs chassis is allowed.
WITNESSES = {
    'arm_limb3_link': {'arm_limb1_joint': -0.0564, 'arm_limb2_joint': -3.232,
                       'arm_limb3_joint': 0.3215, 'arm_limb4_joint': -2.2079,
                       'arm_limb5_joint': 1.237, 'arm_gripper_joint': 1.6226},
    'arm_limb4_link': {'arm_limb1_joint': 0.0971, 'arm_limb2_joint': -1.2933,
                       'arm_limb3_joint': 0.109, 'arm_limb4_joint': -1.4026,
                       'arm_limb5_joint': 0.9784, 'arm_gripper_joint': 1.2442},
    'arm_limb5_link': {'arm_limb1_joint': 0.4143, 'arm_limb2_joint': -0.6742,
                       'arm_limb3_joint': 0.4036, 'arm_limb4_joint': -3.2292,
                       'arm_limb5_joint': 2.4805, 'arm_gripper_joint': 0.7553},
    'arm_gripper_link': {'arm_limb1_joint': -0.2054, 'arm_limb2_joint': -0.3832,
                         'arm_limb3_joint': 0.4773, 'arm_limb4_joint': -2.8213,
                         'arm_limb5_joint': -2.4327, 'arm_gripper_joint': 0.9936},
}


@pytest.fixture(scope='module')
def active():
    sf = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
        'arm_action_server']['ros__parameters']['safety_filter']
    st = yaml.safe_load(_steer_yaml().read_text())['steer_servo_node']['ros__parameters']
    acm = yaml.safe_load((PKG / 'config' / 'arm_allowed_collisions.yaml').read_text())
    return sf, st, acm


@pytest.fixture(scope='module')
def urdf_lidar():
    import xacro
    u = xacro.process_file(str(_desc_dir() / 'urdf' / 'gripperx_v1.urdf.xacro'),
                           mappings={'use_lidar': 'true'}).toxml()
    assert 'lidar_link' in u
    return u


@pytest.fixture(scope='module')
def flt(urdf_lidar, active):
    return build(urdf_lidar, active)


def _robot_pose():
    cal, _leader = _active_executor()
    return cal.to_radians(ROBOT_TICKS)


def _without_pair(acm, pair):
    acm = copy.deepcopy(acm)
    acm['allowed_pairs'] = [p for p in acm['allowed_pairs'] if sorted(p['links']) != pair]
    return acm


def _floats(s):
    return [float(v) for v in s.split()]


def test_the_lidar_collision_is_the_geometry_envelope(urdf_lidar):
    """URDF: one box of lidar_envelope_size, centred (joint origin + collision origin,
    in chassis_link) on lidar_envelope_center; the scan frame stays lidar_mount_xyz."""
    root = ET.fromstring(urdf_lidar)
    link = next(x for x in root.findall('link') if x.get('name') == 'lidar_link')
    joint = next(x for x in root.findall('joint') if x.get('name') == 'lidar_joint')
    cols = link.findall('collision')
    assert len(cols) == 1
    box = cols[0].find('geometry/box')
    assert box is not None
    assert _floats(box.get('size')) == pytest.approx(C.LIDAR_ENVELOPE_SIZE, abs=1e-9)
    assert joint.find('parent').get('link') == 'chassis_link'
    assert _floats(joint.find('origin').get('rpy')) == [0.0, 0.0, 0.0]
    assert _floats(cols[0].find('origin').get('rpy')) == [0.0, 0.0, 0.0]
    mount = _floats(joint.find('origin').get('xyz'))
    assert mount == pytest.approx(C.LIDAR_MOUNT_XYZ, abs=1e-9)
    centre = np.add(mount, _floats(cols[0].find('origin').get('xyz')))
    assert centre == pytest.approx(C.LIDAR_ENVELOPE_CENTER, abs=1e-9)


def test_the_filter_checks_against_that_box(flt):
    """What the filter actually uses: the lidar_link body primitive spans
    centre +- size / 2 in base_link (chassis_link sits on it with a zero offset)."""
    lid = [p for _b, s in flt.envelope_swept.groups for p in s if p.link == 'lidar_link']
    assert len(lid) == 1
    f, h = as_box(lid[0])
    assert f[:3, :3] == pytest.approx(np.eye(3), abs=1e-12)
    assert f[:3, 3] == pytest.approx(C.LIDAR_ENVELOPE_CENTER, abs=1e-9)
    assert 2.0 * h == pytest.approx(C.LIDAR_ENVELOPE_SIZE, abs=1e-9)


def test_limb1_vs_lidar_is_still_excluded(active):
    _sf, _st, acm = active
    hits = [p for p in acm['allowed_pairs'] if sorted(p['links']) == PAIR]
    assert len(hits) == 1
    assert hits[0]['scope'] == 'always' and hits[0]['reason'] == 'user_decision_2026-10-12'


def test_the_robot_pose_passes_even_without_the_exclusion(urdf_lidar, active):
    """The 2026-10-12 refusal (limb1 vs lidar, clearance 0) came from the oversized
    model: with the corrected box the pose passes with limb1 vs lidar checked."""
    sf, st, acm = active
    v = build(urdf_lidar, (sf, st, _without_pair(acm, PAIR))).check_pose(_robot_pose())
    assert v.ok, v


def test_the_robot_pose_passes_and_i_plans_direct_from_it(flt):
    q = _robot_pose()
    assert flt.check_pose(q).ok
    cal, _leader = _active_executor()
    home = cal.to_radians([int(round(z)) for z in cal.zero_ticks])
    b = {**q, **{n: home[n] for n in JOINTS}}
    r = Planner(flt, JOINTS, 120.0, recovery=True).plan(q, b)
    assert r.ok and r.method == 'direct', r.reason


@pytest.mark.parametrize('link', sorted(WITNESSES))
def test_every_other_link_vs_lidar_is_still_checked(urdf_lidar, active, link):
    sf, st, acm = active
    acm = copy.deepcopy(acm)
    acm['allowed_pairs'].append({'links': sorted([link, 'chassis_link']), 'scope': 'always',
                                 'reason': 'test'})
    v = build(urdf_lidar, (sf, st, acm)).check_pose(WITNESSES[link])
    assert not v.ok and v.rule == 'body' and v.detail.startswith(f'{link} vs lidar_link'), v
