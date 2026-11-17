"""chassis_link's collision model in the arm safety filter (SR-17 rule 4(b)).

Since 2026-10-13 the chassis collides as four boxes cut from base_link.stl
(geometry.yaml chassis_box_*, read here through gripperx_geometry.constants, never
duplicated) plus a per-box padding on every side (chassis_box_padding). It replaced the single
bounding box of the mesh. The user confirmed 2026-10-13 that the robot matches the
CAD (tyres excepted), which is what lets the mesh stand for the body.

* The UNPADDED boxes already contain the mesh: every vertex and a dense set of
  points on every triangle - so the padding is a margin on top, not a repair.
  base_link.stl is the decimated visual mesh (2fb4202), <= 0.6 mm from the CAD
  import (68e351c); that difference lies inside the 3 mm body padding.
* A box shrunk by 1 mm leaves part of the mesh outside: the check can fail.
* The description emits exactly these boxes, padded, and the filter reads all four
  (not only the first <collision>).
* A pose inside the old box but outside the new boxes passes; the same pose with
  the old single box is refused - the model change reaches the verdict."""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_safety_filter import _desc_dir, active, build, ik, urdf  # noqa: E402,F401

from gripperx_arm.generate_acm import read_stl  # noqa: E402
from gripperx_arm.kinematics import ArmKinematics, as_box  # noqa: E402
from gripperx_geometry import constants as C  # noqa: E402

BOXES = {
    'chassis_bottom': C.CHASSIS_BOX_BOTTOM,
    'chassis_lower_body': C.CHASSIS_BOX_LOWER_BODY,
    'chassis_upper_body': C.CHASSIS_BOX_UPPER_BODY,
    'chassis_pedestal': C.CHASSIS_BOX_PEDESTAL,
}
PAD = dict(zip(BOXES, C.CHASSIS_BOX_PADDING))      # same order as geometry.yaml
# Declared to 0.1 mm, rounded outward; the mesh is float32 (~3e-8 m at 0.27 m).
TOL = 1e-6
# The model this replaced (URDF until 2026-10-13): the mesh's bounding box.
OLD_BOX = ('0.021541 0 0.021047', '0.493083 0.330200 0.202094')


@pytest.fixture(scope='module')
def triangles():
    return read_stl(_desc_dir() / 'meshes' / 'base_link.stl')


def _outside(points, boxes):
    """Points (n, 3) in chassis_link that no box [lo xyz, hi xyz] contains."""
    inside = np.zeros(len(points), bool)
    for b in boxes:
        lo, hi = np.array(b[:3]) - TOL, np.array(b[3:]) + TOL
        inside |= np.all((points >= lo) & (points <= hi), axis=1)
    return points[~inside]


def _surface_points(tris, n=6):
    """Barycentric grid of (n+1)(n+2)/2 points on every triangle, vertices and edges
    included - a crossing triangle is covered piecewise by two boxes, not one."""
    w = np.array([(i, j, n - i - j) for i in range(n + 1) for j in range(n + 1 - i)]) / n
    return np.einsum('kv,tvc->tkc', w, tris).reshape(-1, 3)


def test_every_mesh_vertex_lies_in_the_unpadded_boxes(triangles):
    out = _outside(triangles.reshape(-1, 3), BOXES.values())
    assert len(out) == 0, f'{len(out)} vertices outside, e.g. {out[:3]}'


def test_every_triangle_lies_in_the_unpadded_boxes(triangles):
    out = _outside(_surface_points(triangles), BOXES.values())
    assert len(out) == 0, f'{len(out)} surface points outside, e.g. {out[:3]}'


@pytest.mark.parametrize('name', sorted(BOXES))
@pytest.mark.parametrize('k', range(6))
def test_each_face_of_each_box_is_tight(triangles, name, k):
    """Moving any one face 1 mm inward leaves mesh outside every box - no box is
    loose, and the containment check above can fail. Holds for the faces on a slab
    cut too: the neighbouring box ends at the same cut."""
    lo_hi = list(BOXES[name])
    upper = k >= 3
    lo_hi[k] += -0.001 if upper else 0.001
    boxes = [lo_hi if n == name else b for n, b in BOXES.items()]
    assert len(_outside(_surface_points(triangles), boxes)) > 0


def test_slab_cuts_are_shared_and_cover_the_mesh_height(triangles):
    z = sorted((b[2], b[5]) for b in BOXES.values())
    assert all(abs(z[i][1] - z[i + 1][0]) < 1e-12 for i in range(len(z) - 1)), z
    v = triangles.reshape(-1, 3)
    assert z[0][0] <= v[:, 2].min() + TOL and z[-1][1] >= v[:, 2].max() - TOL


def test_description_emits_the_padded_boxes(urdf):
    link = next(e for e in ET.fromstring(urdf).findall('link') if e.get('name') == 'chassis_link')
    cols = {c.get('name'): c for c in link.findall('collision')}
    assert set(cols) == set(BOXES)
    assert link.find('visual/geometry/mesh').get('filename').endswith('/base_link.stl')
    for name, b in BOXES.items():
        c = cols[name]
        assert c.find('origin').get('rpy') == '0 0 0'
        xyz = np.array([float(v) for v in c.find('origin').get('xyz').split()])
        size = np.array([float(v) for v in c.find('geometry/box').get('size').split()])
        lo, hi = np.array(b[:3]), np.array(b[3:])
        np.testing.assert_allclose(xyz, (lo + hi) / 2, atol=1e-6)
        np.testing.assert_allclose(size, hi - lo + 2 * PAD[name], atol=1e-6)


def test_the_filter_reads_all_four_boxes(urdf):
    kin = ArmKinematics(urdf)
    prims = [p for p in kin.environment if p.link == 'chassis_link']
    assert len(prims) == 4
    got = sorted(tuple(np.round(np.r_[as_box(p)[0][:3, 3] - as_box(p)[1],
                                      as_box(p)[0][:3, 3] + as_box(p)[1]], 6)) for p in prims)
    want = sorted(tuple(np.round(np.array(b) + PAD[n] * np.array([-1, -1, -1, 1, 1, 1]), 6))
                  for n, b in BOXES.items())
    np.testing.assert_allclose(got, want, atol=1e-6)


def _with_old_box(urdf_xml):
    """The description with chassis_link's four boxes replaced by the old single box."""
    link = re.search(r'<link name="chassis_link">.*?</link>', urdf_xml, re.S).group(0)
    body = re.sub(r'<collision\b.*?</collision>', '', link, flags=re.S)
    old = (f'<collision><origin xyz="{OLD_BOX[0]}" rpy="0 0 0"/><geometry>'
           f'<box size="{OLD_BOX[1]}"/></geometry></collision></link>')
    return urdf_xml.replace(link, body.replace('</link>', old))


def test_space_inside_the_old_box_is_now_free(urdf, active):
    """Gripper pointing down over the lid, TCP at z +0.12 beside the pedestal: the old
    box was solid up to z +0.122 over the whole lid, so it refused this as chassis
    contact; the four boxes (lid top +0.0801 plus 3 mm padding) approve it."""
    new = build(urdf, active)
    old = build(_with_old_box(urdf), active)
    env_new = new.envelope_measured({j: 0.001 for j in new.mechanical_ranges}, 0.0873)
    env_old = old.envelope_measured({j: 0.001 for j in old.mechanical_ranges}, 0.0873)
    q = ik(new, 0.25, -0.08, 0.12, 'front/elbow-')
    v_new, v_old = new.check_pose(q, env_new), old.check_pose(q, env_old)
    assert v_new.ok, v_new
    assert not v_old.ok and 'chassis_link' in v_old.detail, v_old
