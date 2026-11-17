#!/usr/bin/env python3
"""
Fit collision boxes to the arm link meshes (meshes/arm_*.stl, link frame).

Each link gets 1..MAX_SLABS axis-aligned boxes: the mesh is cut into slabs along
one link axis and every slab gets the exact bounding box of the part of the
surface inside it (triangles are clipped at the slab planes, so nothing between
two vertices is missed), plus PAD on every side. The union of the boxes
therefore ENCLOSES the mesh; the split (axis, count) with the smallest total
volume wins, a further box only counting if it saves GAIN of the volume.

Prints the xacro <collision> elements for gripperx_v1.arm.xacro. Deterministic,
offline - it reads only the meshes in this package.

    python3 scripts/arm_fit_collision.py [meshes_dir]
"""
from pathlib import Path
import struct
import sys

import numpy as np

LINKS = ['arm_limb1', 'arm_limb2', 'arm_limb3', 'arm_limb4', 'arm_limb5', 'arm_gripper']
PAD = 0.002          # m per side; > the 0.66 mm decimation shift, so the CAD part is enclosed too
MAX_SLABS = 4
GAIN = 0.10          # a further box must cut the total volume by >= 10 %


def read_stl(path):
    d = Path(path).read_bytes()
    n = struct.unpack('<I', d[80:84])[0]
    raw = np.frombuffer(d[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(float)


def slab_boxes(tris, axis, k):
    v = tris.reshape(-1, 3)
    lo, hi = v[:, axis].min(), v[:, axis].max()
    cuts = np.linspace(lo, hi, k + 1)
    pts = [v]
    for c in cuts[1:-1]:                      # edge / plane intersections
        for a, b in ((0, 1), (1, 2), (2, 0)):
            p, q = tris[:, a], tris[:, b]
            da, db = p[:, axis] - c, q[:, axis] - c
            m = da * db < 0
            t = da[m] / (da[m] - db[m])
            x = p[m] + t[:, None] * (q[m] - p[m])
            x[:, axis] = c
            pts.append(x)
    pts = np.vstack(pts)
    boxes = []
    for i in range(k):
        s = pts[(pts[:, axis] >= cuts[i] - 1e-12) & (pts[:, axis] <= cuts[i + 1] + 1e-12)]
        boxes.append((s.min(0) - PAD, s.max(0) + PAD))
    return boxes


def fit(tris):
    best = None
    for k in range(1, MAX_SLABS + 1):
        cand = min((slab_boxes(tris, ax, k) for ax in range(3)),
                   key=lambda bs: sum(np.prod(hi - lo) for lo, hi in bs))
        vol = sum(np.prod(hi - lo) for lo, hi in cand)
        if best is None or vol < best[1] * (1 - GAIN):
            best = (cand, vol)
    return best[0]


def main(mesh_dir):
    for name in LINKS:
        boxes = fit(read_stl(Path(mesh_dir) / f'{name}.stl'))
        print(f'<!-- {name}_link: {len(boxes)} box(es) -->')
        for lo, hi in boxes:
            c, s = (lo + hi) / 2, hi - lo
            print(f'<collision><origin xyz="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" rpy="0 0 0" />'
                  f'<geometry><box size="{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}" /></geometry>'
                  '</collision>')


if __name__ == '__main__':
    default = Path(__file__).resolve().parents[1] / 'meshes'
    main(sys.argv[1] if len(sys.argv) > 1 else default)
