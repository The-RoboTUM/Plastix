#!/usr/bin/env python3
"""arm_aimed_region - OFFLINE: the aimed P's allowed grasp region and a standoff proposal,
from the ACTIVE model and configuration (FR-16 stage 1). Moves nothing, opens no port.

It builds arm_action_server itself, on a dummy bus and with DDS discovery off, so the
region, the candidates and the direct-path check are exactly the node's - nothing of the
node's logic is copied here.

    ros2 run gripperx_arm arm_aimed_region                 # region, 0.01 m grid
    ros2 run gripperx_arm arm_aimed_region --direct 0.02   # + direct path from home

--direct: for every region cell on a grid of that step, the node's automatic approach
check (SR-17 rule 11: direct path only) from the home pose with the gripper open; the
standoff proposal is then computed on the direct-path region as well. The proposal is
a PROPOSAL (grasp.offset_x_m is the user's value), never written anywhere.
"""
import argparse
import json
import os
import sys
import time

os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
os.environ.pop('ROS_STATIC_PEERS', None)

import yaml  # noqa: E402


class _DummyBus:
    """Reads the home pose, accepts writes; no hardware."""

    def __init__(self, ticks):
        self.pos = dict(ticks)

    def read_pos(self, sid):
        return self.pos.get(sid)

    def read_word(self, sid, addr):
        return None

    def read_byte(self, sid, addr):      # EEPROM lock/limits: unknown, logged only
        return None

    def read_load(self, sid):
        return None

    def write_goal(self, sid, ticks):
        return True

    def torque(self, sid, on):
        return True

    def close(self):
        pass

    def reopen(self):
        pass


def _flatten(prefix, d, out):
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            _flatten(key, v, out)
        else:
            out[key] = v
    return out


def build_node(config_dir=None, description_dir=None, steer_yaml=None, **override):
    """An ArmActionServer on a dummy bus with the active (or given) configuration.
    rclpy must be initialised by the caller."""
    from rclpy.parameter import Parameter
    from ament_index_python.packages import get_package_share_directory as share
    import xacro
    from gripperx_arm import arm_action_server as aas
    cfg = config_dir or os.path.join(share('gripperx_arm'), 'config')
    desc = description_dir or share('gripperx_description')
    p = {}
    for f in ('arm_poses.yaml', 'arm_safety_filter.yaml'):
        with open(os.path.join(cfg, f)) as fh:
            _flatten('', yaml.safe_load(fh)['arm_action_server']['ros__parameters'], p)
    p['safety_filter.calibration_file'] = os.path.join(cfg, 'arm_calibration.yaml')
    p['safety_filter.acm_file'] = os.path.join(cfg, 'arm_allowed_collisions.yaml')
    p['safety_filter.stop_protected_file'] = os.path.join(cfg, 'arm_stop_protected.yaml')
    if steer_yaml:
        p['safety_filter.steer_config_file'] = steer_yaml
    p['aimed.enabled'] = False          # no TF listener, no background region here
    p.update(override)
    with open(p['safety_filter.calibration_file']) as fh:
        home = yaml.safe_load(fh)['arm_joint_state_relay']['ros__parameters']['zero_ticks']
    urdf = xacro.process_file(os.path.join(desc, 'urdf', 'gripperx_v1.urdf.xacro')).toxml()
    node = aas.ArmActionServer(
        bus=_DummyBus(dict(zip(aas.IDS, [int(v) for v in home]))), robot_description=urdf,
        mesh_resolver=lambda uri: os.path.join(desc, 'meshes', uri.split('/')[-1]),
        parameter_overrides=[Parameter(k, value=v) for k, v in p.items()])
    if node._filter is None:
        raise RuntimeError(node._filter_error)
    return node


def solver_of(node):
    """The node's solver, built as the node builds it (aimed.enabled is off here)."""
    node._aim_enabled = True
    try:
        return node._aim_solver()
    finally:
        node._aim_enabled = False


def direct_from_home(node, solver):
    """reachable(x, y): the node's automatic approach plan (direct only) from home with
    the gripper open to the PRE-GRASP pose, and the vertical descent from there, succeed
    for some static candidate (at most aimed.max_candidates)."""
    home_open = node._q_of({**node._home_pos, 6: node._grip_open})
    budget = float(node._p('planner.time_budget_s'))
    limit = int(node._p('aimed.max_candidates'))

    def reachable(x, y):
        r = solver.candidates(x, y, solver.ref_envelope, limit=limit)
        for c in r.candidates:
            goal = node._q_of({**node._ticks_of(c.pregrasp), 6: node._grip_open})
            res = node._planner(budget, direct_only=True, direct_ms=4000,
                                soft_start=True).plan(
                dict(home_open), goal, solver.ref_envelope)
            if not res.ok:
                continue
            at_pre = {**node._ticks_of(c.pregrasp), 6: node._grip_open}
            try:
                node._aim_vertical(at_pre, c.descent[1:], 'arm_aimed_region/descent',
                                   solver.ref_envelope)
            except Exception:  # noqa: BLE001 - MotionRefused: no descent from there
                continue
            return True
        return False
    return reachable


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--step', type=float, default=0.01, help='region grid step (m)')
    ap.add_argument('--direct', type=float, default=0.0, metavar='STEP',
                    help='also check the direct path from home on a grid of STEP (m)')
    ap.add_argument('--config-dir', help='gripperx_arm config dir (default: installed)')
    ap.add_argument('--description-dir', help='gripperx_description dir (default: installed)')
    ap.add_argument('--steer-yaml', help='steer_servo.yaml (default: installed)')
    args = ap.parse_args(argv)
    import numpy as np
    import rclpy
    from gripperx_arm.aimed_grasp import standoff_proposal
    rclpy.init()
    node = build_node(args.config_dir, args.description_dir, args.steer_yaml)
    try:
        solver = solver_of(node)
        t0 = time.monotonic()
        region = solver.derive_region(args.step, fingerprint_extra=(repr(node._cal.zero_ticks),))
        out = {'region': region.summary(), 'region_s': round(time.monotonic() - t0, 1),
               'standoff_static': standoff_proposal(region)}
        if args.direct > 0.0:
            k = max(1, int(round(args.direct / args.step)))
            reach = direct_from_home(node, solver)
            o = solver.kin._ik.o1
            # the sub-grid of `--direct`, anchored on the yaw axis like the region
            ii = [i for i, x in enumerate(region.xs) if round((x - o[0]) / args.step) % k == 0]
            jj = [j for j, y in enumerate(region.ys) if round((y - o[1]) / args.step) % k == 0]
            static = region.mask[np.ix_(ii, jj)]
            ok = np.zeros_like(static)
            t1 = time.monotonic()
            for a, i in enumerate(ii):
                for b, j in enumerate(jj):
                    if static[a, b]:
                        ok[a, b] = reach(float(region.xs[i]), float(region.ys[j]))
            sub = region.__class__(region.xs[ii], region.ys[jj], ok, args.step * k,
                                   region.z_grasp, region.tilt, region.fingerprint,
                                   region.envelope)
            out['direct'] = {'checked': int(static.sum()), 'ok': int(ok.sum()),
                             'seconds': round(time.monotonic() - t1, 1),
                             'step_m': args.step * k, 'region': sub.summary()}
            out['standoff_direct'] = standoff_proposal(sub)
        print(json.dumps(out, indent=1, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
