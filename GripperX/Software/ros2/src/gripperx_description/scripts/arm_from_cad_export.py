#!/usr/bin/env python3
"""
Extract the articulated SO101 arm from an onshape-to-robot export.

Input: the output directory of `onshape-to-robot` (robot.urdf + assets/) for the
GripperX Onshape assembly (document URL: see the ARM block in
urdf/gripperx_v1.arm.xacro). Output, into <out_dir>:

  * one STL per arm link, already expressed in that link's own frame (the
    exporter's visual origin is baked in, so the xacro visual origin is identity),
    at FULL tessellation - run scripts/decimate_meshes.py on them afterwards;
  * a printed table of the numbers the xacro needs: joint origins (parent link
    frame), and a collision box per link that encloses the full-resolution mesh
    plus PAD on every side.

Nothing is written into the package; copying numbers and meshes over is a
deliberate, reviewed step.

    python3 scripts/arm_from_cad_export.py <export_dir> <out_dir>
"""
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

# CAD part (exporter link name) -> GripperX link name. Order = chain order.
LINKS = [
    ('limb1_1', 'arm_limb1_link'),
    ('limb2_1', 'arm_limb2_link'),
    ('limb3_1', 'arm_limb3_link'),
    ('limb4_1', 'arm_limb4_link'),
    ('limb5_1', 'arm_limb5_link'),
    ('limb6_1', 'arm_gripper_link'),
]
# Collision margin per side [m]. Covers the decimation error (<= ~1 mm) with room
# to spare, so the box also encloses the decimated visual mesh.
PAD = 0.002


def read_stl(path):
    with open(path, 'rb') as f:
        d = f.read()
    n = struct.unpack('<I', d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)


def write_stl(path, tris, header=b'gripperx arm link, link frame'):
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    nrm = np.cross(e1, e2)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 0)
    rec = np.zeros((len(tris), 50), dtype=np.uint8)
    body = np.concatenate([nrm[:, None, :], tris], axis=1).astype('<f4')
    rec[:, :48] = body.reshape(len(tris), 12).view(np.uint8)
    with open(path, 'wb') as f:
        f.write(header.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(elem):
    o = elem.find('origin')
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*[float(v) for v in o.get('rpy').split()])
    T[:3, 3] = [float(v) for v in o.get('xyz').split()]
    return T, o.get('xyz'), o.get('rpy')


def main(export_dir, out_dir):
    root = ET.parse(os.path.join(export_dir, 'robot.urdf')).getroot()
    links = {e.get('name'): e for e in root.findall('link')}
    joints = {e.find('child').get('link'): e for e in root.findall('joint')}
    os.makedirs(out_dir, exist_ok=True)
    for cad, name in LINKS:
        j = joints[cad]
        _, xyz, rpy = origin(j)
        lim = j.find('limit')
        parent = j.find('parent').get('link')
        print(f'{name}: joint {j.get("name")} ({j.get("type")}) parent {parent}'
              f'  origin xyz="{xyz}" rpy="{rpy}"  axis {j.find("axis").get("xyz")}'
              f'  limit [{lim.get("lower")}, {lim.get("upper")}]')
        vis = links[cad].find('visual')
        M, _, _ = origin(vis)
        mesh = vis.find('geometry/mesh').get('filename').split('/')[-1]
        tris = read_stl(os.path.join(export_dir, 'assets', mesh)) @ M[:3, :3].T + M[:3, 3]
        write_stl(os.path.join(out_dir, name.replace('_link', '') + '.stl'), tris)
        v = tris.reshape(-1, 3)
        lo, hi = v.min(0) - PAD, v.max(0) + PAD
        c, s = (lo + hi) / 2, hi - lo
        print(f'    collision box origin xyz="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}"'
              f' size="{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}"  ({len(tris)} triangles)')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
