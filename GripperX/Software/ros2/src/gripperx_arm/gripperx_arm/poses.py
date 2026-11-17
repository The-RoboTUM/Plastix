"""The fixed arm poses, resolved to servo ticks. Pure Python, no ROS imports.

User decisions 2026-10-07:
  * `home` is the calibration reference: it IS arm_calibration.yaml `zero_ticks`
    (URDF q = 0). There is no second copy - arm_poses.yaml has no poses.home.
  * `grip`, `gripper.open` and `gripper.close` may be stored as ANGLES relative to
    home (URDF radians: poses.grip_rad, gripper.open_rad, gripper.close_rad) and are
    converted through the calibration; the tick variants stay valid until the
    follower calibration is verified and `arm_poses_to_rad` has produced the angle
    version for review. Exactly one variant per pose.
"""
import math
from typing import Dict, List, Mapping, Optional

from gripperx_arm.kinematics import TickCalibration

TICKS = 4096


def calibration_from_doc(doc: Mapping) -> TickCalibration:
    c = doc['arm_joint_state_relay']['ros__parameters']
    return TickCalibration(list(c['urdf_joints']), [float(v) for v in c['zero_ticks']],
                           [int(v) for v in c['signs']], float(c['ticks_per_rev']),
                           [float(v) for v in c.get('range_center_rad',
                                                    [0.0] * len(c['urdf_joints']))])


def rad_to_ticks(cal: TickCalibration, q: Mapping[str, float]) -> Dict[str, int]:
    """The servo position of an ABSOLUTE joint angle: ticks modulo one turn (the
    servo register is single-turn, so q and q + 2 pi are the same position). How the
    servo gets there is a separate question - see arm_action_server._ticks_of."""
    k = cal.ticks_per_rev / (2.0 * math.pi)
    return {n: int(round(z + s * q[n] * k)) % TICKS
            for n, z, s in zip(cal.joints, cal.zero_ticks, cal.signs) if n in q}


def _one(name: str, ticks, rad) -> str:
    has_t = ticks is not None and ticks != [] and ticks != -1
    has_r = rad is not None and rad != [] and not (isinstance(rad, float) and math.isnan(rad))
    if has_t == has_r:
        raise ValueError(f'{name}: give exactly one of the tick and the _rad variant')
    return 'ticks' if has_t else 'rad'


def resolve(poses: Mapping, cal: TickCalibration) -> Dict[str, object]:
    """poses: the arm_action_server parameters as a nested dict (arm_poses.yaml).
    Returns {'home': [6 ticks], 'grip': [6 ticks], 'open': int, 'close': int,
    'sources': {...}} in servo-ID order."""
    if 'home' in poses.get('poses', {}):
        raise ValueError('poses.home is no longer allowed: home = arm_calibration.yaml '
                         'zero_ticks (one source, user decision 2026-10-07)')
    home = [int(round(z)) for z in cal.zero_ticks]
    p = poses.get('poses', {})
    g = poses.get('gripper', {})
    sources = {}
    sources['grip'] = _one('poses.grip', p.get('grip'), p.get('grip_rad'))
    if sources['grip'] == 'ticks':
        grip = [int(v) for v in p['grip']]
    else:
        rad = [float(v) for v in p['grip_rad']]
        t = rad_to_ticks(cal, dict(zip(cal.joints, rad)))
        grip = [t[n] for n in cal.joints]
    if len(grip) != len(cal.joints):
        raise ValueError(f'poses.grip needs {len(cal.joints)} values')
    out = {'home': home, 'grip': grip}
    gname = cal.joints[-1]
    for key in ('open', 'close'):
        sources[key] = _one(f'gripper.{key}', g.get(key), g.get(f'{key}_rad'))
        if sources[key] == 'ticks':
            out[key] = int(g[key])
        else:
            out[key] = rad_to_ticks(cal, {gname: float(g[f'{key}_rad'])})[gname]
    out['sources'] = sources
    return out


def to_rad(poses: Mapping, cal: TickCalibration) -> Dict[str, object]:
    """The angle version of the current tick poses (for review, not applied)."""
    r = resolve(poses, cal)
    gname = cal.joints[-1]
    grip_q = cal.to_radians(r['grip'])
    open_q = cal.to_radians(r['home'][:-1] + [r['open']])[gname]
    close_q = cal.to_radians(r['home'][:-1] + [r['close']])[gname]
    return {'grip_rad': [round(grip_q[n], 6) for n in cal.joints],
            'open_rad': round(open_q, 6), 'close_rad': round(close_q, 6)}


def main(argv: Optional[List[str]] = None) -> int:
    """arm_poses_to_rad: print grip / gripper.open / gripper.close as angles relative
    to home, from the current ticks and calibration. Refuses while the calibration
    is unverified (the angles would carry the unverified signs) unless
    --preview-unverified, which prints them marked as NOT for use."""
    import argparse
    import sys
    from pathlib import Path

    import yaml
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('--config-dir', type=Path)
    ap.add_argument('--preview-unverified', action='store_true')
    args = ap.parse_args(argv)
    if args.config_dir is None:
        from gripperx_arm.generate_acm import config_dir
        args.config_dir = config_dir()
    cal_doc = yaml.safe_load((args.config_dir / 'arm_calibration.yaml').read_text())
    poses = yaml.safe_load((args.config_dir / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']
    verified = bool(cal_doc['arm_joint_state_relay']['ros__parameters'].get(
        'calibration_verified'))
    if not verified and not args.preview_unverified:
        print('REFUSED: calibration_verified is false - the angles would carry the '
              'unverified signs. Verify the calibration first.')
        return 1
    r = to_rad(poses, calibration_from_doc(cal_doc))
    if not verified:
        print('# PREVIEW ONLY - calibration UNVERIFIED, do not paste')
    print('# review, then replace the tick variants in arm_poses.yaml with:')
    print('    poses:')
    print(f'      grip_rad: {r["grip_rad"]}')
    print('    gripper:')
    print(f'      open_rad: {r["open_rad"]}')
    print(f'      close_rad: {r["close_rad"]}')
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
