#!/usr/bin/env python3
"""Move each FOLLOWER servo's 0/4095 tick wrap into the sector its joint cannot
reach, so that SR-17 L1 (servo EEPROM angle limits, min < max) becomes expressible.
Tooling only. DRY RUN unless --write; never run against the robot without SR-1.

    arm_wrap_offset TRAVEL.yaml                      # plan + review patch (offline)
    arm_wrap_offset TRAVEL.yaml --port /dev/X        # + read current offsets/torque
    arm_wrap_offset TRAVEL.yaml --port /dev/X --write --patch-out wrap.patch

TRAVEL.yaml = gripperx_leader record_limits output; its `follower_joint_limits`
min/max are follower present ticks, wrap-aware (min > max = the travel crosses
0/4095). Per servo, travel = min -> (upwards) -> max; the unreachable gap is the
rest of the turn. The new wrap is put in the MIDDLE of the gap:

    D = (max + gap / 2) mod 4096           shift of every present-tick value
    new_present = (old_present - D) mod 4096
    new Homing_Offset = old Homing_Offset + D  (Present = Actual - Homing_Offset)

Register facts (sources in servo_bus.py): Homing_Offset = address 31, 2 bytes,
sign-magnitude with the sign in bit 11 (magnitude <= 2047), EEPROM; Lock = 55
(0 = EEPROM writable); Torque_Enable = 40. Not yet exercised on this robot.

Every stored follower tick value depends on the offset; the tool prints ONE patch
for all of them (arm_calibration.yaml zero_ticks = home; arm_poses.yaml grip,
gripper open/close where still in ticks; joint_limits if an old file still has them) for review. It never applies it. Radians are unchanged by construction
(ticks and zero shift together) - see test_wrap_offset.py.
Also tick-valued, NOT patched (listed for the reviewer): the DEFAULT_* fallback
poses in arm_action_server.py, and any earlier record_limits output files.
"""
import argparse
import difflib
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional

import yaml

TICKS = 4096
IDS = [1, 2, 3, 4, 5, 6]
NAMES = ['limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper']
SIGN_BIT = 11


def encode_offset(v: int) -> int:
    if abs(v) > (1 << SIGN_BIT) - 1:
        raise ValueError(f'offset {v} not representable (|v| <= {(1 << SIGN_BIT) - 1})')
    return (abs(v) | (1 << SIGN_BIT)) if v < 0 else v


def decode_offset(raw: int) -> int:
    mag = raw & ((1 << SIGN_BIT) - 1)
    return -mag if raw & (1 << SIGN_BIT) else mag


def normalise_offset(v: int) -> Optional[int]:
    """Equivalent offset (mod 4096) inside the encodable range, or None (2048)."""
    v = v % TICKS
    if v > TICKS // 2:
        v -= TICKS
    return None if abs(v) > (1 << SIGN_BIT) - 1 else v


def plan(mins: List[Optional[int]], maxs: List[Optional[int]], min_gap: int) -> List[dict]:
    rows = []
    for sid, name, lo, hi in zip(IDS, NAMES, mins, maxs):
        row = {'id': sid, 'joint': name, 'ok': False, 'why': '', 'shift': None}
        if lo is None or hi is None:
            row['why'] = 'travel not recorded (null) - unknown'
        else:
            travel = (hi - lo) % TICKS
            gap = TICKS - travel
            row.update(travel=travel, gap=gap)
            if gap < min_gap:
                row['why'] = (f'unreachable gap {gap} ticks < min_gap {min_gap} '
                              '(travel ~ a full turn, or the recording is incomplete)')
            else:
                row['shift'] = (hi + gap // 2) % TICKS
                row['ok'] = True
        rows.append(row)
    return rows


def shifted(ticks: int, shift: int) -> int:
    return (int(ticks) - shift) % TICKS


def _fmt(values) -> str:
    return '[' + ', '.join(str(v) for v in values) + ']'


def _sub_list(text: str, key: str, old: list, new: list) -> str:
    pat = re.compile(rf'(^\s*{re.escape(key)}:\s*)\[([^\]]*)\]', re.M)
    hits = [m for m in pat.finditer(text)
            if [int(v) for v in m.group(2).split(',')] == list(old)]
    if len(hits) != 1:
        raise ValueError(f'{key}: expected exactly one list {old}, found {len(hits)}')
    m = hits[0]
    return text[:m.start(2)] + ', '.join(map(str, new)) + text[m.end(2):]


def _sub_scalar(text: str, key: str, old: int, new: int) -> str:
    pat = re.compile(rf'(^\s*{re.escape(key)}:\s*){old}\b', re.M)
    hits = list(pat.finditer(text))
    if len(hits) != 1:
        raise ValueError(f'{key}: expected exactly one value {old}, found {len(hits)}')
    m = hits[0]
    return text[:m.end(1)] + str(new) + text[m.end():]


def patch(cfg: Path, shifts: Dict[int, int]) -> str:
    """Unified diff re-expressing every stored follower tick in the new frame."""
    poses_p, cal_p = cfg / 'arm_poses.yaml', cfg / 'arm_calibration.yaml'
    poses_t, cal_t = poses_p.read_text(), cal_p.read_text()
    p = yaml.safe_load(poses_t)['arm_action_server']['ros__parameters']
    c = yaml.safe_load(cal_t)['arm_joint_state_relay']['ros__parameters']
    s = [shifts[i] for i in IDS]
    new_poses = poses_t
    # home = zero_ticks (patched below). Angle variants (grip_rad, open_rad,
    # close_rad) are offset-independent; only tick variants are re-expressed.
    if 'grip' in p.get('poses', {}):
        old = p['poses']['grip']
        new_poses = _sub_list(new_poses, 'grip', old, [shifted(t, d) for t, d in zip(old, s)])
    for key in ('open', 'close'):
        if key in p.get('gripper', {}):
            new_poses = _sub_scalar(new_poses, key, p['gripper'][key],
                                    shifted(p['gripper'][key], s[5]))
    # joint_limits was removed from arm_poses.yaml (SR-17 rule 3); handled if present.
    lim = p.get('joint_limits') or {'min': [], 'max': []}
    lmin, lmax = lim['min'], lim['max']
    nmin, nmax = [], []
    for lo, hi, d, n in zip(lmin, lmax, s, NAMES):
        if (lo, hi) == (0, TICKS - 1):
            nmin.append(lo)          # full range: frame-independent
            nmax.append(hi)
            continue
        a, b = shifted(lo, d), shifted(hi, d)
        if a > b:
            raise ValueError(f'joint_limits {n}: [{lo}, {hi}] would cross the new wrap')
        nmin.append(a)
        nmax.append(b)
    if (nmin, nmax) != (lmin, lmax):
        new_poses = _sub_list(new_poses, 'min', lmin, nmin)
        new_poses = _sub_list(new_poses, 'max', lmax, nmax)
    zero = [int(v) for v in c['zero_ticks']]
    new_cal = _sub_list(cal_t, 'zero_ticks', zero, [shifted(t, d) for t, d in zip(zero, s)])
    out = []
    for path, a, b in ((poses_p, poses_t, new_poses), (cal_p, cal_t, new_cal)):
        rel = f'Software/ros2/src/gripperx_arm/config/{path.name}'
        out += difflib.unified_diff(a.splitlines(True), b.splitlines(True),
                                    f'a/{rel}', f'b/{rel}')
    return ''.join(out)


HUMAN_STEPS = """HUMAN STEPS (each session is an arm motion test under SR-1 - user approval first):
  1. Stop arm_action_server (one writer per bus); the gripperx-bringup restart rules apply.
  2. Support the arm BY HAND before torque goes off - it drops under gravity otherwise.
  3. Switch torque off on the follower servos (e.g. arm_pose_teach), keep holding the arm.
  4. Run this tool with --write; keep the arm still until it reports the read-back.
  5. Review and apply the printed patch IN THE SAME CHANGE, then regenerate/check:
     python3 -m gripperx_arm.generate_acm --check ; arm_eeprom_limits --calibration <new>
"""


def write(bus, rows, current: Dict[int, int], tol: int) -> List[str]:
    """Torque must already be OFF (re-read). Returns problems; empty = all good."""
    from gripperx_arm.servo_bus import ADDR_HOMING_OFFSET, ADDR_LOCK, ADDR_TORQUE
    problems = []
    for r in rows:
        sid = r['id']
        torque = bus.read_byte(sid, ADDR_TORQUE)
        if torque != 0:
            problems.append(f'id {sid}: torque is {torque} - must be 0 (off); nothing written')
            continue
        new = normalise_offset(current[sid] + r['shift'])
        if new is None:
            problems.append(f'id {sid}: offset not encodable; nothing written')
            continue
        p0 = bus.read_pos(sid)
        bus.write_byte(sid, ADDR_LOCK, 0)
        try:
            bus.write_word(sid, ADDR_HOMING_OFFSET, encode_offset(new))
        finally:
            bus.write_byte(sid, ADDR_LOCK, 1)
        p1 = bus.read_pos(sid)
        if p0 is None or p1 is None:
            problems.append(f'id {sid}: present position unreadable around the write')
            continue
        got = (p0 - p1) % TICKS
        err = min((got - r['shift']) % TICKS, (r['shift'] - got) % TICKS)
        if err > tol:
            problems.append(f'id {sid}: present moved by {got}, expected {r["shift"]} '
                            f'(+-{tol}) - check the offset semantics before going on')
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('travel', help='record_limits output (follower_joint_limits used)')
    ap.add_argument('--min-gap-ticks', type=int, default=256,
                    help='TO-VERIFY: smallest unreachable sector accepted (default 256 = 22.5 deg)')
    ap.add_argument('--port')
    ap.add_argument('--write', action='store_true')
    ap.add_argument('--verify-tol-ticks', type=int, default=3,
                    help='TO-VERIFY: arm may settle while held by hand')
    ap.add_argument('--patch-out')
    args = ap.parse_args(argv)

    doc = yaml.safe_load(Path(args.travel).read_text())
    fl = doc.get('follower_joint_limits') or {}
    if not fl:
        print('no follower_joint_limits in the travel file (follower not recorded)')
        return 1
    rows = plan(fl['min'], fl['max'], args.min_gap_ticks)
    for r in rows:
        print(f"id {r['id']} {r['joint']:<8} travel {r.get('travel')} gap {r.get('gap')} "
              f"shift {r['shift']}  {'OK' if r['ok'] else 'REFUSED: ' + r['why']}")
    if not all(r['ok'] for r in rows):
        print('not every servo can be planned - nothing further')
        return 1
    from gripperx_arm.generate_acm import config_dir
    text = patch(config_dir(), {r['id']: r['shift'] for r in rows})
    if args.patch_out:
        Path(args.patch_out).write_text(text)
        print(f'review patch written to {args.patch_out} (NOT applied)')
    else:
        print(text)
    if not args.port:
        return 0
    from gripperx_arm.eeprom_limits import arm_node_running
    if arm_node_running():
        print('REFUSED: arm_action_server is running - stop it first (one writer per bus)')
        return 1
    from gripperx_arm.servo_bus import ADDR_HOMING_OFFSET, FeetechBus
    bus = FeetechBus(args.port, 1_000_000)
    bus.open()
    try:
        raw = {r['id']: bus.read_word(r['id'], ADDR_HOMING_OFFSET) for r in rows}
        if any(v is None for v in raw.values()):
            print(f'offset registers unreadable: {raw}')
            return 1
        current = {k: decode_offset(v) for k, v in raw.items()}
        for r in rows:
            print(f"id {r['id']}: Homing_Offset {current[r['id']]} -> "
                  f"{normalise_offset(current[r['id']] + r['shift'])}")
        if not args.write:
            print('dry run: nothing written\n' + HUMAN_STEPS)
            return 0
        problems = write(bus, rows, current, args.verify_tol_ticks)
        print('\n'.join(problems) if problems else 'offsets written and verified')
        print(HUMAN_STEPS)
        return 1 if problems else 0
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
