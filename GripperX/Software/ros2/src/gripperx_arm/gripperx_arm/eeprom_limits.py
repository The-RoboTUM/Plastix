#!/usr/bin/env python3
"""SR-17 layer L1: per-servo EEPROM angle limits for the arm, computed - never typed.

    min ticks = calibration(URDF joint limit) - eeprom_margin_min_ticks[servo]
    max ticks = calibration(URDF joint limit) + eeprom_margin_max_ticks[servo]

Joint limits come from the robot description (gripperx_geometry/config/geometry.yaml
-> xacro -> URDF <limit>), the tick calibration from config/arm_calibration.yaml, the
margins from config/arm_safety_filter.yaml (eeprom_margin_min_ticks / _max_ticks, in
TICK space per servo id: min side / max side) - the same keys arm_action_server reads
for eeprom_check: enforce, so the tool and the node can never disagree on the margin.

    arm_eeprom_limits                       # print the computed values only
    arm_eeprom_limits --port /dev/X         # + READ the servos, print a dry-run diff
    arm_eeprom_limits --port /dev/X --write # write (maintenance step, SR-1; see
                                            # documentation/DEPLOYMENT.md section 5)
    arm_eeprom_limits --port /dev/X --write --restore FILE
                                            # rollback: write a saved snapshot back
                                            # (--write saves one before every write)

Registers (STS3215, source: LeRobot src/lerobot/motors/feetech/tables.py,
STS_SMS_SERIES_CONTROL_TABLE, fetched 2026-10-07): Min_Position_Limit 9 (2 bytes),
Max_Position_Limit 11 (2 bytes), Homing_Offset 31 (2 bytes), Torque_Enable 40,
Goal_Position 42, Lock 55 (1 byte; 0 = EEPROM writable), Present_Position 56.
NOT yet exercised on this robot.

Bus access at all (also the dry run) refuses while gripperx-bringup.service is active
or activating (its launch respawns the arm node within 5 s), while any other process
holds the port open, or while an arm_action_server process exists (SR-17 rule 9: one
writer per bus). The port is then held with TIOCEXCL, so a node started meanwhile
cannot open it.

--write additionally refuses unless: every value is expressible in 0..4095 without
wrapping; the calibration is verified (or --allow-unverified-calibration); every
servo's Homing_Offset equals the raw value recorded in arm_calibration.yaml
(homing_offsets_raw) or, where none is recorded, is 0 (how the limits interact with
a non-zero offset is TO-VERIFY); Lock, Torque_Enable, Present_Position and
Goal_Position are readable; Torque_Enable is 0 on EVERY servo (LeRobot writes EEPROM
only with torque off: disable_torque writes Torque_Enable 0, then Lock 0); and every
servo's present position lies inside the new range. A goal outside the limits is
clamped by the servo (TO-VERIFY, see DEPLOYMENT.md section 5), so a limit written
past the present position would move the arm at the next torque-on (goal := present).

    arm_eeprom_limits --port /dev/X --torque-off
                                            # torque off on all six servos, nothing
                                            # else: the ARM GOES LIMP - a person
                                            # holds and lowers it (approved step)

Every EEPROM / Lock write is followed by a settle time and a read-back of that register,
retried a bounded number of times (verify as you go; the leader's run of 2026-10-15 on
the same servo model lost the packet right after an EEPROM write). A write whose
acknowledgement is lost but whose value reads back counts as done.

Torque is changed only by --torque-off, never as a side effect of --write; switching
torque on is never done by this tool. Run every bus mode only as an approved
maintenance step.
"""
import argparse
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import yaml

TICKS = 4096
IDS = [1, 2, 3, 4, 5, 6]
BRINGUP_UNIT = 'gripperx-bringup.service'
SAFETY_FILTER_FILE = 'arm_safety_filter.yaml'
MARGIN_KEYS = ('eeprom_margin_min_ticks', 'eeprom_margin_max_ticks')
# What the arm node's enforce refusal and this tool's own messages tell the operator.
RUN_HINT = ('stop the stack, then on the Pi: ros2 run gripperx_arm arm_eeprom_limits '
            '--port /dev/arm_servo --torque-off (a person holds the arm), the same without '
            'it (dry run), then with --write - the approved maintenance procedure in '
            'documentation/DEPLOYMENT.md section 5')

Margin = Union[int, Sequence[int]]


def _per_servo(m: Margin) -> List[int]:
    return [int(m)] * len(IDS) if isinstance(m, (int, float)) else [int(v) for v in m]


def compute(limits: Dict[str, Tuple[float, float]], joints: List[str], zero_ticks: List[float],
            signs: List[int], ticks_per_rev: float, margin_min: Margin,
            margin_max: Optional[Margin] = None) -> List[dict]:
    """One row per servo: {'id', 'joint', 'min', 'max', 'ok', 'why'}. Margins are in
    tick space (min side, max side), one value or one per servo id."""
    k = ticks_per_rev / (2.0 * math.pi)
    mlo = _per_servo(margin_min)
    mhi = _per_servo(margin_min if margin_max is None else margin_max)
    rows = []
    for sid, n, z, s, ml, mh in zip(IDS, joints, zero_ticks, signs, mlo, mhi):
        lo, hi = limits[n]
        a, b = z + s * lo * k, z + s * hi * k
        tmin = int(math.floor(min(a, b))) - ml
        tmax = int(math.ceil(max(a, b))) + mh
        why = ''
        if ml < 0 or mh < 0:
            why = (f'negative margin ({ml}, {mh}): the EEPROM limit would lie inside the '
                   'commanded range and stall the servo short of the software limit')
        elif tmin < 0 or tmax > TICKS - 1:
            why = (f'NOT EXPRESSIBLE: needs {tmin}..{tmax}, i.e. the joint range CROSSES the '
                   f'servo wrap at 0/{TICKS - 1} (min/max limits cannot express that). Cause: '
                   'the range is too wide for this zero offset, or still the +-pi placeholder; '
                   'remedy: a Homing_Offset or re-mounted horn moving the wrap out of range')
        elif tmax - tmin < 1:
            why = 'empty range'
        elif tmin == 0 and tmax == 0:
            why = '0/0 would switch the servo to multi-turn mode'
        rows.append({'id': sid, 'joint': n, 'min': tmin, 'max': tmax, 'ok': not why, 'why': why})
    return rows


def load_margins(cfg_dir: Path) -> Tuple[List[int], List[int]]:
    """(min side, max side) per servo from arm_safety_filter.yaml - the node's keys."""
    p = yaml.safe_load((cfg_dir / SAFETY_FILTER_FILE).read_text())[
        'arm_action_server']['ros__parameters']
    out = []
    for key in MARGIN_KEYS:
        v = p.get(key)
        if not isinstance(v, list) or len(v) != len(IDS) or \
                not all(type(x) is int and x >= 0 for x in v):
            raise SystemExit(f'{SAFETY_FILTER_FILE}: {key} must be {len(IDS)} integers >= 0 '
                             f'(servo-id order), not {v!r}')
        out.append(list(v))
    return out[0], out[1]


def read_current(bus, ids=IDS) -> Dict[int, Dict[str, Optional[int]]]:
    from gripperx_arm.servo_bus import (
        ADDR_GOAL_POS, ADDR_HOMING_OFFSET, ADDR_LOCK, ADDR_MAX_ANGLE_LIMIT,
        ADDR_MIN_ANGLE_LIMIT, ADDR_PRESENT, ADDR_TORQUE, POSITION_SIGN_BIT, sm_decode)

    def pos(sid, addr):
        raw = bus.read_word(sid, addr)
        return None if raw is None else sm_decode(raw, POSITION_SIGN_BIT)
    return {sid: {'min': bus.read_word(sid, ADDR_MIN_ANGLE_LIMIT),
                  'max': bus.read_word(sid, ADDR_MAX_ANGLE_LIMIT),
                  'offset': bus.read_word(sid, ADDR_HOMING_OFFSET),
                  'torque': bus.read_byte(sid, ADDR_TORQUE),
                  'lock': bus.read_byte(sid, ADDR_LOCK),
                  'present': pos(sid, ADDR_PRESENT),
                  'goal': pos(sid, ADDR_GOAL_POS)} for sid in ids}


def _offset_text(raw: Optional[int]) -> str:
    """Homing_Offset is sign-magnitude, sign bit 11 (servo_bus docstring): raw 3624
    is -1576. Shown both ways; any non-zero value blocks --write."""
    if raw is None or raw == 0:
        return str(raw)
    from gripperx_arm.servo_bus import sm_decode
    return f'{raw} = {sm_decode(raw, 11)} NON-ZERO'


def diff(rows: List[dict], current: Dict[int, dict]) -> List[str]:
    out = []
    for r in rows:
        c = current.get(r['id'], {})
        state = 'SAME' if (c.get('min'), c.get('max')) == (r['min'], r['max']) else 'DIFF'
        out.append(f"id {r['id']} {r['joint']:<18} servo {c.get('min')}..{c.get('max')} "
                   f"(offset {_offset_text(c.get('offset'))}, lock {c.get('lock')}, "
                   f"torque {c.get('torque')}, "
                   f"present {c.get('present')}, goal {c.get('goal')})  ->  computed "
                   f"{r['min']}..{r['max']}  {state}{'  ' + r['why'] if r['why'] else ''}")
    return out


# After every EEPROM / Lock write: wait this long before the next packet, then read the
# register back. TO-VERIFY: no STS3215 EEPROM write time is documented in this repo or
# in the installed scservo_sdk. 0.1 s is a conservative start, about three times the
# SDK's own status-packet timeout (2 x LATENCY_TIMER 16 ms + 2 ms = 34 ms). FeetechBus
# drops stale input before every transaction, so a late status packet is not misread.
EEPROM_SETTLE_SEC = 0.1
EEPROM_ATTEMPTS = 3                  # write + read-back attempts per register, TO-VERIFY


def write_verified(bus, sid: int, addr: int, value: int, size: int,
                   settle_s: float = EEPROM_SETTLE_SEC,
                   attempts: int = EEPROM_ATTEMPTS) -> Optional[str]:
    """Write, settle, read back; up to `attempts` times (re-writing the same value is
    idempotent). None = the register holds `value`; else the failure text."""
    wr = bus.write_byte if size == 1 else bus.write_word
    rd = bus.read_byte if size == 1 else bus.read_word
    last = 'no attempt'
    for _ in range(max(1, int(attempts))):
        acked = wr(sid, addr, value)
        if settle_s > 0:
            time.sleep(settle_s)
        got = rd(sid, addr)
        if got == value:
            return None
        last = f'{"acknowledged" if acked else "not acknowledged"}, read back {got}'
    return f'id {sid}: register {addr} := {value} not confirmed after {attempts} attempt(s) ({last})'


def write(bus, rows: List[dict], current: Dict[int, dict],
          settle_s: float = EEPROM_SETTLE_SEC) -> List[str]:
    """Per servo: unlock, the two limits, relock, each write settled and read back
    (write_verified). Returns the failures (empty = all confirmed). The limit written first is the one that keeps the intermediate
    (min, max) pair ordered. A failed or interrupted write still relocks that servo
    and stops before the next one, so at most one servo is left half-written."""
    from gripperx_arm.servo_bus import ADDR_LOCK, ADDR_MAX_ANGLE_LIMIT, ADDR_MIN_ANGLE_LIMIT
    failures = []
    for r in rows:
        sid = r['id']
        old_max = current.get(sid, {}).get('max')
        order = [(ADDR_MIN_ANGLE_LIMIT, r['min']), (ADDR_MAX_ANGLE_LIMIT, r['max'])]
        if old_max is not None and r['min'] > old_max:
            order.reverse()
        try:
            why = write_verified(bus, sid, ADDR_LOCK, 0, 1, settle_s)
            if why:
                failures.append(f'unlock: {why}')
            else:
                for addr, value in order:
                    why = write_verified(bus, sid, addr, value, 2, settle_s)
                    if why:
                        failures.append(why)
                        break
        finally:
            why = write_verified(bus, sid, ADDR_LOCK, 1, 1, settle_s)
            if why:
                failures.append(f'RELOCK not confirmed - EEPROM may stay writable: {why}')
        if failures:
            break
    return failures


def recorded_offsets(c: dict) -> Optional[List[int]]:
    """homing_offsets_raw of arm_calibration.yaml: the six RAW Homing_Offset registers
    (sign-magnitude, sign bit 11) the calibration was measured with, or None where
    none is recorded (then only 0 is accepted). Present_Position = Actual - offset, so
    zero_ticks hold only for the offset they were measured with."""
    v = c.get('homing_offsets_raw')
    if v in (None, []):
        return None
    if not isinstance(v, list) or len(v) != len(IDS) or \
            not all(type(x) is int and 0 <= x < 1 << 12 for x in v):
        raise SystemExit(f'arm_calibration.yaml: homing_offsets_raw must be {len(IDS)} raw '
                         f'register values 0..4095 (servo-id order), not {v!r}')
    return list(v)


def offset_refusals(offsets: Dict[int, Optional[int]],
                    recorded: Optional[Sequence[int]]) -> List[str]:
    """Shared by --write and the node's eeprom_check: enforce."""
    why = []
    for i, sid in enumerate(IDS):
        want = 0 if recorded is None else int(recorded[i])
        got = offsets.get(sid)
        if got is None:
            why.append(f'id {sid}: Homing_Offset unreadable')
        elif got != want:
            src = ('recorded in arm_calibration.yaml homing_offsets_raw' if recorded is not None
                   else 'none recorded in arm_calibration.yaml, so only 0 is accepted')
            why.append(f'id {sid}: Homing_Offset {_offset_text(got)} != {want} ({src}) - the '
                       'calibration zeros hold only for the offset they were measured with; '
                       'do not change the offset, report it')
    return why


def write_refusals(rows, current, cal_verified, allow_unverified,
                   recorded: Optional[Sequence[int]] = None) -> List[str]:
    why = [f"id {r['id']}: {r['why']}" for r in rows if not r['ok']]
    if not cal_verified and not allow_unverified:
        why.append('calibration_verified is false (arm_calibration.yaml)')
    why += offset_refusals({sid: c.get('offset') for sid, c in current.items()}, recorded)
    by_id = {r['id']: r for r in rows}
    for sid, c in current.items():
        if c.get('min') is None or c.get('max') is None:
            why.append(f'id {sid}: limit registers unreadable')
        unread = [k for k in ('lock', 'torque', 'present', 'goal') if c.get(k) is None]
        if unread:
            why.append(f'id {sid}: {", ".join(unread)} unreadable')
            continue
        if c['torque'] != 0:
            why.append(f"id {sid}: Torque_Enable = {c['torque']} - EEPROM is written with "
                       'torque OFF only (LeRobot practice; a write under torque is untested): '
                       'run --torque-off first, with a person holding the arm')
        r = by_id.get(sid)
        if r is None or not r['ok']:
            continue
        if not r['min'] <= c['present'] <= r['max']:
            why.append(f"id {sid}: present position {c['present']} outside the new range "
                       f"{r['min']}..{r['max']} - the next torque-on would clamp it toward the "
                       'limit (a jump at full servo speed); move the arm inside by hand first '
                       'or revise the margin')
    return why


def torque_off(bus) -> List[str]:
    """Torque_Enable 0 on every servo, each one attempted; returns the failures."""
    from gripperx_arm.servo_bus import ADDR_TORQUE
    return [f'id {sid}: torque-off not acknowledged' for sid in IDS
            if not bus.write_byte(sid, ADDR_TORQUE, 0)]


def snapshot(current: Dict[int, dict]) -> dict:
    """The limit registers as read, in the file format --restore takes."""
    return {'eeprom_limits': {int(sid): [c.get('min'), c.get('max')]
                              for sid, c in current.items()}}


def snapshot_of_record(path: Path, current: Dict[int, dict], rows: List[dict]) -> Optional[str]:
    """An existing --save-previous file is never overwritten (audit EW1 M2; it may be the
    only copy of the original registers). It is accepted as the snapshot of record when
    every limit register equals either the snapshot or the value this run writes (rows
    are absolute: computed, or the --restore file) - the registers are the snapshot
    plus part of this plan, e.g. after an interrupted run. None = accepted."""
    try:
        doc = (yaml.safe_load(Path(path).read_text()) or {})['eeprom_limits']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        return f'{path} exists but is no eeprom_limits snapshot ({exc!r})'
    want = {r['id']: (r['min'], r['max']) for r in rows}
    off = []
    for sid, c in current.items():
        pair = doc.get(sid) or doc.get(str(sid))
        if not (isinstance(pair, list) and len(pair) == 2):
            return f'{path} exists but has no [min, max] for id {sid}'
        for i, field in enumerate(('min', 'max')):
            target = want.get(sid, (None, None))[i]
            if c.get(field) not in (pair[i], target):
                off.append(f'id {sid} {field}: now {c.get(field)}, snapshot {pair[i]}, '
                           f'this run {target}')
    if off:
        return (f'{path} exists and the registers differ from it by more than this run '
                'writes: ' + '; '.join(off))
    return None


def restore_rows(path: Path) -> List[dict]:
    """Rows from a snapshot file ({'eeprom_limits': {id: [min, max]}}) - the rollback
    set, written back verbatim (factory 0..4095 included)."""
    doc = yaml.safe_load(Path(path).read_text())['eeprom_limits']
    rows = []
    for sid in IDS:
        lo, hi = (doc.get(sid) or doc.get(str(sid)) or [None, None])
        why = ''
        if not (isinstance(lo, int) and isinstance(hi, int)):
            why = f'no integer pair for id {sid} in {path}'
        elif not 0 <= lo < hi <= TICKS - 1:
            why = f'{lo}..{hi} is not an ordered range inside 0..{TICKS - 1}'
        rows.append({'id': sid, 'joint': '(restore)', 'min': lo, 'max': hi,
                     'ok': not why, 'why': why})
    return rows


def arm_node_running() -> bool:
    """True if an arm_action_server process exists on this machine: two processes on
    one serial bus corrupt each other's packets (SR-17 rule 9)."""
    for d in Path('/proc').iterdir():
        if d.name.isdigit() and int(d.name) != os.getpid():
            try:
                if b'arm_action_server' in (d / 'cmdline').read_bytes():
                    return True
            except OSError:
                pass
    return False


def port_holders(port: str) -> List[str]:
    """Other processes with the port's device open (/proc/<pid>/fd). Holders owned by
    another user are invisible without root: this complements, does not replace, the
    unit and process checks."""
    dev = os.path.realpath(port)
    out = []
    for d in Path('/proc').iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            for fd in (d / 'fd').iterdir():
                if os.path.realpath(fd) == dev:
                    cmd = (d / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
                    out.append(f'pid {d.name}: {cmd[:120]}')
                    break
        except OSError:
            pass
    return out


def unit_state(unit: str = BRINGUP_UNIT) -> Optional[str]:
    """systemd ActiveState of the unit, None where systemctl does not exist."""
    exe = shutil.which('systemctl')
    if exe is None:
        return None
    res = subprocess.run([exe, 'is-active', unit], capture_output=True, text=True, check=False)
    return res.stdout.strip() or 'unknown'


def bus_refusals(port: str) -> List[str]:
    why = []
    state = unit_state()
    if state in ('active', 'activating', 'reloading', 'deactivating'):
        why.append(f'{BRINGUP_UNIT} is {state} - stop the stack first (its launch respawns '
                   'the arm node within seconds)')
    if arm_node_running():
        why.append('an arm_action_server process is running - one writer per bus')
    why += [f'{port} is held open by {h}' for h in port_holders(port)]
    return why


def hold_exclusive(bus) -> bool:
    """TIOCEXCL on the open port: a later open() by any non-root process fails with
    EBUSY, so an arm node started during the maintenance step cannot join the bus."""
    import fcntl
    import termios
    try:
        fcntl.ioctl(bus._port.ser.fileno(), termios.TIOCEXCL)
        return True
    except (AttributeError, OSError):
        return False


def _load(cfg_dir: Path, urdf_xml: str, calibration: Optional[Path] = None):
    from gripperx_arm.kinematics import ArmKinematics
    kin = ArmKinematics(urdf_xml)
    c = yaml.safe_load((calibration or cfg_dir / 'arm_calibration.yaml').read_text())[
        'arm_joint_state_relay']['ros__parameters']
    if list(c['urdf_joints']) != kin.joint_names:
        raise SystemExit(f'calibration joints {c["urdf_joints"]} != URDF {kin.joint_names}')
    return kin.limits, c


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='arm bus to READ the current registers from')
    ap.add_argument('--write', action='store_true', help='WRITE the computed limits (EEPROM)')
    ap.add_argument('--torque-off', action='store_true',
                    help='switch torque OFF on all six servos and exit: THE ARM GOES LIMP, a '
                         'person must hold it (separately approved step; not with --write)')
    ap.add_argument('--allow-unverified-calibration', action='store_true')
    ap.add_argument('--settle-sec', type=float, default=EEPROM_SETTLE_SEC,
                    help=f'wait after every EEPROM write (default {EEPROM_SETTLE_SEC}, TO-VERIFY)')
    ap.add_argument('--save-previous', type=Path,
                    help='with --write: where to save the registers read before writing '
                         '(default: arm_eeprom_previous_<UTC time>.yaml in the current '
                         'directory) - the rollback set for --restore')
    ap.add_argument('--restore', type=Path,
                    help='write the [min, max] pairs of a saved snapshot back instead of '
                         'the computed values (rollback; same bus and motion checks, needs '
                         '--write)')
    ap.add_argument('--calibration', type=Path,
                    help='calibration file to use instead of the package one, e.g. the '
                         'zero_ticks re-expressed by arm_wrap_offset before applying them '
                         '(the arm node checks against ITS calibration under eeprom_check: '
                         'enforce - deploy that file before restarting)')
    args = ap.parse_args(argv)

    from gripperx_arm.generate_acm import config_dir, load_urdf
    limits, c = _load(config_dir(), load_urdf(), args.calibration)
    mlo, mhi = load_margins(config_dir())
    recorded = recorded_offsets(c)
    if args.torque_off and (args.write or args.restore):
        print('--torque-off is a step of its own: not together with --write/--restore')
        return 2
    rows = compute(limits, list(c['urdf_joints']), [float(v) for v in c['zero_ticks']],
                   [int(v) for v in c['signs']], float(c['ticks_per_rev']), mlo, mhi)
    if args.restore:
        rows = restore_rows(args.restore)
        print(f'RESTORE mode: target values from {args.restore}, not computed')
    print(f'calibration_verified: {bool(c.get("calibration_verified"))}   '
          f'margin ticks (min side / max side, {SAFETY_FILTER_FILE}): {mlo} / {mhi}')
    print('Homing_Offset expected: ' + (f'raw {recorded} (arm_calibration.yaml)'
                                        if recorded else '0 on every servo (none recorded)'))
    if not args.port:
        if args.torque_off:
            print('--torque-off needs --port')
            return 2
        for r in rows:
            print(f"id {r['id']} {r['joint']:<18} computed {r['min']}..{r['max']}"
                  f"{'  ' + r['why'] if r['why'] else ''}")
        if args.write:
            print('--write needs --port')
            return 2
        return 0 if all(r['ok'] for r in rows) else 1

    refuse = bus_refusals(args.port)
    if refuse:
        print('REFUSED (nothing read, nothing written):\n  ' + '\n  '.join(refuse))
        return 1
    from gripperx_arm.servo_bus import FeetechBus
    bus = FeetechBus(args.port, 1_000_000)
    bus.open()
    try:
        if not hold_exclusive(bus):
            print(f'WARNING: could not hold {args.port} exclusively (TIOCEXCL)')
            if args.write or args.torque_off:
                print('WRITE REFUSED: without the exclusive hold a starting node could '
                      'join the bus mid-write')
                return 1
        current = read_current(bus)
        print('\n'.join(diff(rows, current)))
        if args.torque_off:
            failed = torque_off(bus)
            after = read_current(bus)
            still = [sid for sid, a in after.items() if a.get('torque') != 0]
            for f in failed:
                print(f'TORQUE-OFF FAILED: {f}')
            print('torque off: ' + ('OK on all servos - the arm is limp'
                                    if not still else f'Torque_Enable != 0 on ids {still}'))
            return 0 if not (failed or still) else 1
        if not args.write:
            print('dry run: nothing written')
            return 0
        refuse = write_refusals(rows, current,
                                bool(c.get('calibration_verified')) or bool(args.restore),
                                args.allow_unverified_calibration, recorded)
        if refuse:
            print('WRITE REFUSED:\n  ' + '\n  '.join(refuse))
            return 1
        import datetime
        prev = args.save_previous or Path(
            'arm_eeprom_previous_'
            f'{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}.yaml')
        if prev.exists():
            why = snapshot_of_record(prev, current, rows)
            if why:
                print(f'WRITE REFUSED: {why} - a snapshot is never overwritten')
                return 1
            print(f'{prev.resolve()} exists and is KEPT as the snapshot of record: the '
                  'registers differ from it only by values this run writes (rollback: '
                  '--restore that file)')
        else:
            prev.write_text(yaml.safe_dump(snapshot(current)))
            print(f'previous registers saved to {prev.resolve()} (rollback: --restore that '
                  'file)')
        failures = write(bus, rows, current, args.settle_sec)
        time.sleep(max(0.0, args.settle_sec))
        after = read_current(bus)
        bad = [r['id'] for r in rows if (after[r['id']]['min'], after[r['id']]['max'])
               != (r['min'], r['max'])]
        unlocked = [sid for sid, a in after.items() if a.get('lock') != 1]
        print('\n'.join(diff(rows, after)))
        for f in failures:
            print(f'WRITE FAILED: {f}')
        print('written; read-back ' + ('OK' if not bad else f'MISMATCH on ids {bad}')
              + ('' if not unlocked else f'; Lock != 1 on ids {unlocked}'))
        print('NOTE: the read-back shows the register as the servo holds it NOW; whether it '
              'survives a servo power cycle (EEPROM, not only RAM) is proven only by the '
              'node log after one (DEPLOYMENT.md section 5).')
        return 0 if not (bad or unlocked or failures) else 1
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
