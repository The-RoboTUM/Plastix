#!/usr/bin/env python3
"""Weld + quadric-decimate the VISUAL STL meshes of gripperx_description.

WHY: the meshes come straight out of the CAD export and are tessellated far
finer than any viewer needs. All COLLISION geometry in this package is
primitives (box/cylinder, see gripperx_v1.core.xacro), so nothing here touches
physics -- this is render cost only, in RViz and in the Gazebo GUI/sensor
passes.

RE-RUN THIS after every fresh CAD export (see commit 38c860e), otherwise the
new meshes land at full tessellation again.

Requires `fast_simplification` (not a ROS dependency; install into a venv):
    python3 -m venv --system-site-packages venv
    ./venv/bin/pip install fast-simplification
    ./venv/bin/python scripts/decimate_meshes.py <src_dir> <dst_dir> "<targets>"

The targets dict maps mesh file name -> target triangle count. The values in
TARGETS below were chosen 2026-08-24 by rendering a ladder of levels and
picking the last one that keeps the silhouette: base_link/arm tolerate ~30 %,
the steering housings bottom out around 4000 (below that their cylindrical
bosses visibly facet), the tyres below ~1080 turn into polygons.

Reported per mesh: bounding-box shift in mm, surface-area and volume change.
Watch the bbox column -- the URDF derives several mount offsets from mesh
bounding boxes, so a decimation that moves one by more than ~1-2 mm is too
aggressive and must be backed off.
"""
import sys, os, struct
import numpy as np
import fast_simplification


def read_stl(path):
    with open(path, 'rb') as f:
        head = f.read(84)
        n = struct.unpack('<I', head[80:84])[0]
        data = f.read()
    if len(data) < n * 50:                       # ASCII fallback
        raise ValueError(f'{path}: not a binary STL')
    raw = np.frombuffer(data[:n * 50], dtype=np.uint8).reshape(n, 50)
    tris = raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3)
    return tris.astype(np.float64)


def weld(tris, decimals=6):
    v = tris.reshape(-1, 3)
    key = np.round(v, decimals)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return v[idx], inv.reshape(-1, 3).astype(np.int32)


def write_stl(path, verts, faces):
    tri = verts[faces]                                    # (F,3,3)
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    nrm = np.cross(e1, e2)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 0)
    rec = np.zeros((len(faces), 50), dtype=np.uint8)
    body = np.concatenate([nrm[:, None, :], tri], axis=1).astype('<f4')
    rec[:, :48] = body.reshape(len(faces), 12).view(np.uint8)
    with open(path, 'wb') as f:
        f.write(b'gripperx decimated visual mesh'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(faces)))
        f.write(rec.tobytes())


def stats(tri):
    v = tri.reshape(-1, 3)
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1).sum()
    vol = np.abs(np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6.0)
    return v.min(0), v.max(0), area, vol


def process(src, dst, target, dry, agg=3.0):
    tris = read_stl(src)
    lo0, hi0, a0, vol0 = stats(tris)
    verts, faces = weld(tris)
    n0 = len(faces)
    if target >= n0:
        nv, nf = verts, faces
    else:
        nv, nf = fast_simplification.simplify(
            verts.astype(np.float32), faces,
            target_count=int(target), agg=agg, preserve_border=True)
        nv = nv.astype(np.float64)
    out = nv[nf]
    lo1, hi1, a1, vol1 = stats(out)
    d_lo = np.abs(lo1 - lo0).max() * 1000.0          # mm
    d_hi = np.abs(hi1 - hi0).max() * 1000.0
    if not dry:
        write_stl(dst, nv, nf)
    return dict(name=os.path.basename(src), n0=n0, n1=len(nf), verts0=len(verts),
                verts1=len(nv), bbox_mm=max(d_lo, d_hi),
                d_area=100.0 * (a1 - a0) / a0, d_vol=100.0 * (vol1 - vol0) / max(vol0, 1e-12))


# Applied 2026-08-24: 121368 -> 45830 triangles (62.2 %), max bbox shift 1.107 mm.
TARGETS = {
    'base_link.stl': 14125, 'arm_stowed.stl': 10277, 'camera_link.stl': 1108,
    'front_left_steer.stl': 4000, 'front_right_steer.stl': 4000,
    'back_left_steer.stl': 4000, 'back_right_steer.stl': 4000,
    'front_left_wheel.stl': 1080, 'front_right_wheel.stl': 1080,
    'back_left_wheel.stl': 1080, 'back_right_wheel.stl': 1080,
}

if __name__ == '__main__':
    dry = '--apply' not in sys.argv
    srcdir, dstdir = sys.argv[1], sys.argv[2]
    targets = eval(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] != '--apply' else TARGETS
    rows = []
    for name, tgt in targets.items():
        rows.append(process(os.path.join(srcdir, name), os.path.join(dstdir, name), tgt, dry))
    print(f"{'mesh':<24}{'tris':>16}{'verts':>16}{'bbox Δ mm':>11}{'area %':>9}{'vol %':>8}")
    t0 = t1 = 0
    for r in rows:
        t0 += r['n0']; t1 += r['n1']
        print(f"{r['name']:<24}{r['n0']:>7} ->{r['n1']:>6}{r['verts0']:>8} ->{r['verts1']:>6}"
              f"{r['bbox_mm']:>11.3f}{r['d_area']:>9.2f}{r['d_vol']:>8.2f}")
    print(f"{'TOTAL':<24}{t0:>7} ->{t1:>6}   ({100.0*(t0-t1)/t0:.1f} % fewer triangles)")
    print('DRY RUN -- nothing written' if dry else 'WRITTEN')
