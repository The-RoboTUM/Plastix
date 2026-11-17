#!/usr/bin/env python3
"""Leader EEPROM maintenance (prepared 2026-10-13, user: "Ja, bereite auch den
Leader-EEPROM Schritt vor"). DRY RUN unless --write. Modelled on gripperx_arm
eeprom_limits.py (hardening of 8f1bad9).

Two changes, both computed, never typed:
  (a) Min/Max_Position_Limit of the configured leader ids (IDS, 1..6) -> 0..4095. The
      values found 2026-10-13 (760/3232, 823/3199, 873/3076, 865/3188, 0/4095) and on id 6
      (gripper) 2026-11-13 (1293/2810, while its recorded travel reaches 3028 ticks) are
      copies of the follower's LeRobot values and say nothing about the leader. Id 6
      joined 2026-11-13 (leader gripper driven in leader mode, FR-17 amendment).
  (b) limb5 (id 5) only: Homing_Offset shifted by D so the servo's 0/4095 wrap falls
      into the middle of limb5's unreachable gap (recorded travel_min/max_rad of the
      calibration). The other ids' offsets are NOT touched: the calibration was
      measured with them.

    leader_eeprom                                   # plan from the calibration only
    leader_eeprom --port /dev/gripperx_leader       # + READ the servos, dry-run diff
    leader_eeprom --port /dev/gripperx_leader --write
    leader_eeprom --port /dev/gripperx_leader --write --restore FILE   # rollback

Registers (STS3215, protocol 0), source: LeRobot src/lerobot/motors/feetech/tables.py,
STS_SMS_SERIES_CONTROL_TABLE: Min/Max_Position_Limit 9/11 (2), Homing_Offset 31 (2),
Torque_Enable 40, Lock 55 (0 = EEPROM writable), Present_Position 56. Homing_Offset is
sign-magnitude with the sign in bit 11 (STS_SMS_SERIES_ENCODINGS_TABLE
{"Homing_Offset": 11}; encode/decode as lerobot/motors/encoding_utils.py), and
Present_Position = Actual_Position - Homing_Offset (lerobot/motors/feetech/feetech.py,
_get_half_turn_homings). So a shift of D ticks in the present frame is
new offset = old offset + D, new present = (old present - D) mod 4096, new zero_ticks =
(old zero_ticks - D) mod 4096; radians (and so range_center_rad, travel_min/max_rad)
are unchanged by construction.

WRAP, measured 2026-10-13 (read-only recording, /arm/leader/ticks = raw register):
limb5 does NOT wrap - its present runs continuously 1825..5638 (above 4095) over its
travel; limb2 DOES wrap 4095 -> 0 at its bottom stop. So the expected present after the
shift is present - D WITHOUT modulo (limb5: 1825..5638 -> 134..3947); a result below 0
would be reported negative (sign-magnitude, bit 15) and is refused before writing.
LeaderBus.read_positions fails closed on the sign bit. Why the two servos differ is
not determined (README).

The plan is ABSOLUTE (audit LA2 E1): the calibration records the offsets its ticks were
measured with (homing_offset_raw). Per shifted servo the target offset is derived from
that record; current == record -> write the target, current == target -> nothing (and
a note if the calibration still describes the old state), anything else -> refused,
use --restore. A calibration that already describes the shifted state needs no shift.

Bus access at all (also the dry run) refuses while another process holds the port or a
leader_arm_node / record_limits process exists; the port is then held with TIOCEXCL.
--write additionally refuses unless every register is readable, torque is OFF on every
servo (the tool never writes Torque_Enable: on would move the leader), and every new
value is representable. Before writing, all touched registers are saved (--restore).
Per servo: unlock, write, relock - relock also on a failure or Ctrl-C, and the run
stops at the first failure. EVERY write (Lock included) is followed by a settle time
and a read-back of that register, retried a bounded number of times (verify as you go,
robot run 2026-10-15: the packet right after an EEPROM write was lost or came back
short). A re-run continues from the registers as they are (the plan is absolute), and
an existing --save-previous file is kept and accepted as the snapshot of record when
the registers differ from it only by values this plan writes (see snapshot_of_record). Read-back of every value, Lock and Torque_Enable, and of
limb5's present position against the expected shift. Persistence across a power cycle
is proven only by a dry run after one (README procedure).
"""
import argparse
import datetime
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional

import yaml

from gripperx_leader.calibration import load_calibration
from gripperx_leader.leader_bus import (
    ADDR_HOMING_OFFSET, ADDR_LOCK, ADDR_MAX_POS_LIMIT, ADDR_MIN_POS_LIMIT, ADDR_PRESENT,
    ADDR_TORQUE, LeaderBus, LeaderReadError, LeaderWriteError, detect_port)

TICKS = 4096
IDS = [1, 2, 3, 4, 5, 6]             # id 6 (gripper) since 2026-11-13: change (a) only
WRAP_SHIFT_IDS = [5]                 # limb5 only (task 2026-10-13)
OFFSET_SIGN_BIT = 11
FULL_RANGE = (0, TICKS - 1)
PRESENT_TOLERANCE = 3                # ticks: the leader is limp and must not be touched
# After every EEPROM / Lock write: wait this long and drop pending input before the next
# packet, then read the register back. TO-VERIFY: no STS3215 EEPROM write time is
# documented in this repo or in the installed scservo_sdk. 0.1 s is a conservative
# start: about three times the SDK's own status-packet timeout (2 x LATENCY_TIMER 16 ms
# + 2 ms = 34 ms, port_handler.setPacketTimeout), the window in which the 2026-10-15
# reply was lost; a whole run adds well under 2 s.
EEPROM_SETTLE_SEC = 0.1
EEPROM_ATTEMPTS = 3                  # write + read-back attempts per register, TO-VERIFY
FOREIGN = (b'leader_arm_node', b'record_limits')


def sm_decode(raw: int, bit: int = OFFSET_SIGN_BIT) -> int:
    mag = raw & ((1 << bit) - 1)
    return -mag if raw & (1 << bit) else mag


def sm_encode(value: int, bit: int = OFFSET_SIGN_BIT) -> int:
    if abs(value) > (1 << bit) - 1:
        raise ValueError(f'{value} does not fit a sign-magnitude field with sign bit {bit}')
    return (abs(value) | (1 << bit)) if value < 0 else value


def wrap_shift(zero: float, sign: int, tmin_rad: float, tmax_rad: float,
               tpr: int = TICKS) -> dict:
    """Shift D (present frame) that puts the wrap in the middle of the unreachable gap.
    The travel is continuous in radians (record_limits), so its tick ends are
    zero + sign*rad*k WITHOUT wrapping; the gap runs from the upper end round to
    the lower end + one turn."""
    k = tpr / (2.0 * math.pi)
    ends = sorted((zero + sign * tmin_rad * k, zero + sign * tmax_rad * k))
    travel = ends[1] - ends[0]
    gap = tpr - travel
    if gap <= 0:
        raise ValueError(f'travel {travel:.0f} ticks leaves no gap')
    d = int(round(ends[1] + gap / 2.0)) % tpr
    new_lo = (ends[0] - d) % tpr
    return {'shift': d, 'gap_ticks': gap, 'travel_ticks': travel,
            'new_lo': new_lo, 'new_hi': new_lo + travel,
            'margin_ticks': min(new_lo, tpr - 1 - (new_lo + travel)),
            'new_zero': int(round(zero - d)) % tpr}


def read_current(bus, ids=IDS) -> Dict[int, Dict[str, Optional[int]]]:
    def rd(fn, sid, addr):
        try:
            return fn(sid, addr)
        except LeaderReadError:
            return None
    return {sid: {'min': rd(bus.read_word, sid, ADDR_MIN_POS_LIMIT),
                  'max': rd(bus.read_word, sid, ADDR_MAX_POS_LIMIT),
                  'offset': rd(bus.read_word, sid, ADDR_HOMING_OFFSET),
                  'torque': rd(bus.read_byte, sid, ADDR_TORQUE),
                  'lock': rd(bus.read_byte, sid, ADDR_LOCK),
                  'present': rd(bus.read_word, sid, ADDR_PRESENT)} for sid in ids}


MIN_WRAP_MARGIN = 0.25              # of the gap: wrap already this far inside it -> no shift


def _wrap_offset(v: int) -> int:
    """Same offset modulo one turn, inside the sign-magnitude range -2047..2047."""
    v = (v + TICKS // 2) % TICKS - TICKS // 2
    if v == -2048:
        raise ValueError('offset -2048 is not representable')
    return v


def _shift_for(cal, i) -> dict:
    """wrap_shift, or shift 0 when the calibration's travel already sits inside 0..4095
    with the wrap well inside the gap (the calibration describes the shifted state)."""
    info = wrap_shift(cal.zero_ticks[i], cal.sign[i], cal.travel_min_rad[i],
                      cal.travel_max_rad[i], int(cal.ticks_per_rev))
    k = cal.ticks_per_rev / (2 * math.pi)
    lo = cal.zero_ticks[i] + min(cal.sign[i] * cal.travel_min_rad[i],
                                 cal.sign[i] * cal.travel_max_rad[i]) * k
    hi = lo + info['travel_ticks']
    margin = min(lo, TICKS - 1 - hi)
    if lo >= 0 and hi <= TICKS - 1 and margin >= MIN_WRAP_MARGIN * info['gap_ticks']:
        info.update(shift=0, new_zero=int(round(cal.zero_ticks[i])), new_lo=lo, new_hi=hi,
                    margin_ticks=margin)
    info['old_zero'] = cal.zero_ticks[i]
    return info


def plan(current: Dict[int, dict], cal, ids=IDS) -> dict:
    """{'writes': [(sid, field, addr, old, new)], 'shift': {sid: info}, 'why': [...],
    'notes': [...]}. Absolute: a second run after the write plans nothing."""
    writes, shifts, why, notes = [], {}, [], []
    rec_all = cal.homing_offset_raw
    inv_all = cal.zero_invariant_ticks
    broken = cal.broken_pairs()
    for sid in ids:
        c = current[sid]
        i = sid - 1
        rec = None if rec_all is None else rec_all[i]
        if i in broken:
            # Audit LA3: a half-done step 7 is a calibration edit, not an EEPROM problem.
            cur = c['offset']
            fix = ('' if cur is None else
                   f'; the pair matching the EEPROM now (Homing_Offset {cur}) is, as ONE edit: '
                   f'zero_ticks[{i}] = {(inv_all[i] - sm_decode(cur)) % TICKS} and '
                   f'homing_offset_raw[{i}] = {cur} (do NOT use --restore, do not edit '
                   'zero_invariant_ticks)')
            why.append(f'id {sid}: calibration pair inconsistent: zero_ticks[{i}] '
                       f'{cal.zero_ticks[i]} and homing_offset_raw[{i}] {rec} do not belong '
                       f'together (zero_invariant_ticks {inv_all[i]}){fix}')
            continue
        if sid in WRAP_SHIFT_IDS and (inv_all is None or inv_all[i] is None):
            why.append(f'id {sid}: calibration has no zero_invariant_ticks - the zero/offset '
                       'pair cannot be checked')
            continue
        if c['min'] is not None and (c['min'], c['max']) != FULL_RANGE:
            pair = [('min', ADDR_MIN_POS_LIMIT, c['min'], FULL_RANGE[0]),
                    ('max', ADDR_MAX_POS_LIMIT, c['max'], FULL_RANGE[1])]
            writes += [(sid, f, a, o, n) for f, a, o, n in pair if o != n]
        if rec is None:
            why.append(f'id {sid}: calibration has no homing_offset_raw - the offset its '
                       'ticks were measured with is unknown')
            continue
        if sid not in WRAP_SHIFT_IDS:
            if c['offset'] is not None and c['offset'] != rec:
                why.append(f'id {sid}: Homing_Offset {c["offset"]} != calibration {rec} '
                           '(this tool never changes it) - use --restore')
            continue
        if None in (cal.zero_ticks[i], cal.sign[i]) or cal.travel_min_rad is None \
                or cal.travel_min_rad[i] is None or cal.travel_max_rad[i] is None:
            why.append(f'id {sid}: calibration lacks zero/sign/travel - cannot compute the shift')
            continue
        info = _shift_for(cal, i)
        target = sm_encode(_wrap_offset(sm_decode(rec) + info['shift'])) if info['shift'] \
            else rec
        info.update(recorded_offset=rec, target_offset=target,
                    target_offset_signed=sm_decode(target))
        shifts[sid] = info
        cur = c['offset']
        if cur is None:
            continue
        if cur == target:
            if target != rec:
                notes.append(f'id {sid}: EEPROM already shifted ({cur}); the calibration still '
                             f'describes the old state - set zero_ticks[{i}] '
                             f'{cal.zero_ticks[i]} -> {info["new_zero"]} and '
                             f'homing_offset_raw[{i}] {rec} -> {target} (README step 7)')
        elif cur == rec:
            writes.append((sid, 'offset', ADDR_HOMING_OFFSET, cur, target))
            if c['present'] is not None:
                info['old_present'] = c['present']
                info['expected_present'] = c['present'] - info['shift']
                if not 0 <= info['expected_present'] <= TICKS - 1:
                    why.append(f'id {sid}: present {c["present"]} - shift {info["shift"]} = '
                               f'{info["expected_present"]} is outside 0..4095 - limb5 is not '
                               'where its recorded travel says; refused')
        else:
            why.append(f"id {sid}: Homing_Offset {cur} is neither the calibration's {rec} nor "
                       f'the target {target} (torn or foreign write?) - roll back with '
                       '--restore, do not shift on top of it; if this EEPROM state is '
                       f'intended (e.g. after a rollback), the matching calibration pair is '
                       f'zero_ticks[{i}] = {(inv_all[i] - sm_decode(cur)) % TICKS} and '
                       f'homing_offset_raw[{i}] = {cur}')
    return {'writes': writes, 'shift': shifts, 'why': why, 'notes': notes}


def write_refusals(current: Dict[int, dict], p: dict) -> List[str]:
    why = list(p['why'])
    for sid, c in current.items():
        unread = [k for k, v in c.items() if v is None]
        if unread:
            why.append(f'id {sid}: {", ".join(unread)} unreadable')
        elif c['torque'] != 0:
            why.append(f'id {sid}: torque is ON - the tool never switches it; it must be off')
    return why


def snapshot(current: Dict[int, dict]) -> dict:
    return {'leader_eeprom': {int(sid): {'min': c['min'], 'max': c['max'],
                                         'offset': c['offset']}
                              for sid, c in current.items()}}


def restore_plan(path: Path, current: Dict[int, dict]) -> dict:
    doc = yaml.safe_load(Path(path).read_text())['leader_eeprom']
    writes, why = [], []
    fields = (('min', ADDR_MIN_POS_LIMIT), ('max', ADDR_MAX_POS_LIMIT),
              ('offset', ADDR_HOMING_OFFSET))
    for sid, c in current.items():
        saved = doc.get(sid) or doc.get(str(sid))
        if not saved or not all(isinstance(saved.get(f), int) for f, _ in fields):
            why.append(f'id {sid}: no complete entry in {path}')
            continue
        if not 0 <= saved['min'] < saved['max'] <= TICKS - 1:
            why.append(f'id {sid}: {saved["min"]}..{saved["max"]} is not an ordered range')
        writes += [(sid, f, a, c[f], saved[f]) for f, a in fields if c[f] != saved[f]]
    return {'writes': writes, 'shift': {}, 'why': why}


def write(bus, writes, current, settle_s: float = EEPROM_SETTLE_SEC,
          attempts: int = EEPROM_ATTEMPTS) -> List[str]:
    """Per servo: unlock, its writes, relock (also on failure / Ctrl-C). Every write is
    settled and read back (LeaderBus.write_verified). Stops at the first failure.
    Min/max order keeps the intermediate pair ordered."""
    failures = []
    for sid in sorted({w[0] for w in writes}):
        mine = [w for w in writes if w[0] == sid]
        lim = {w[1]: w for w in mine}
        if 'min' in lim and lim['min'][4] > current[sid]['max']:
            mine.sort(key=lambda w: w[1] != 'max')        # max first
        try:
            bus.write_verified(sid, ADDR_LOCK, 0, 1, settle_s, attempts)
            for _, field, addr, _, new in mine:
                bus.write_verified(sid, addr, new, 2, settle_s, attempts)
        except LeaderWriteError as exc:
            failures.append(f'id {sid}: {exc}')
        finally:
            try:
                bus.write_verified(sid, ADDR_LOCK, 1, 1, settle_s, attempts)
            except LeaderWriteError as exc:
                failures.append(f'id {sid}: RELOCK not confirmed - EEPROM may stay writable '
                                f'({exc})')
        if failures:
            break
    return failures


SNAP_FIELDS = ('min', 'max', 'offset')


def snapshot_of_record(path: Path, current: Dict[int, dict], targets_for) -> Optional[str]:
    """An existing snapshot is kept (never overwritten) and accepted as the snapshot of
    record when every register differs from it at most by the value the SAME plan,
    computed from the snapshot, would write - i.e. what is on the servos is the snapshot
    plus part of this plan (an interrupted earlier run). Returns None when accepted,
    else the refusal. `targets_for(state)` -> the plan's writes for that state."""
    try:
        doc = (yaml.safe_load(Path(path).read_text()) or {})['leader_eeprom']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        return f'{path} exists but is no leader_eeprom snapshot ({exc!r})'
    snap_state = {}
    for sid, c in current.items():
        saved = doc.get(sid) or doc.get(str(sid))
        if not saved or not all(isinstance(saved.get(f), int) for f in SNAP_FIELDS):
            return f'{path} exists but has no complete entry for id {sid}'
        snap_state[sid] = {**c, **{f: saved[f] for f in SNAP_FIELDS}}
    targets = {(sid, f): new for sid, f, _, _, new in targets_for(snap_state)}
    off = [f'id {sid} {f}: now {c[f]}, snapshot {snap_state[sid][f]}, plan '
           f'{targets.get((sid, f), "no write")}'
           for sid, c in current.items() for f in SNAP_FIELDS
           if c[f] != snap_state[sid][f] and c[f] != targets.get((sid, f))]
    if off:
        return (f'{path} exists and the registers differ from it by more than this plan '
                'writes: ' + '; '.join(off))
    return None


def verify(after: Dict[int, dict], p: dict) -> List[str]:
    bad = []
    for sid, field, _, _, new in p['writes']:
        if after[sid][field] != new:
            bad.append(f'id {sid} {field}: read back {after[sid][field]}, wrote {new}')
    for sid, a in after.items():
        if a['lock'] != 1:
            bad.append(f'id {sid}: Lock {a["lock"]} (expected 1)')
        if a['torque'] != 0:
            bad.append(f'id {sid}: Torque_Enable {a["torque"]} (expected 0)')
    for sid, info in p['shift'].items():
        exp, got = info.get('expected_present'), after[sid]['present']
        if exp is not None and (got is None or abs(got - exp) > PRESENT_TOLERANCE):
            bad.append(f'id {sid}: present {got}, expected {exp} after the shift '
                       '(leader touched, or the offset does not act as documented)')
    return bad


def describe(current, p) -> List[str]:
    out = []
    for sid, c in current.items():
        out.append(f'id {sid}: limits {c["min"]}..{c["max"]}, Homing_Offset raw {c["offset"]}'
                   f' ({"?" if c["offset"] is None else sm_decode(c["offset"])}), '
                   f'lock {c["lock"]}, torque {c["torque"]}, present {c["present"]}')
    out.append('planned writes:' if p['writes'] else 'planned writes: none')
    for sid, field, addr, old, new in p['writes']:
        out.append(f'  id {sid} {field:<6} (register {addr}): {old} -> {new}')
    for sid, i in p['shift'].items():
        out.append(
            f'  id {sid} wrap shift D = {i["shift"]} ticks (gap {i["gap_ticks"]:.0f} ticks, '
            f'travel {i["travel_ticks"]:.0f}, new travel ticks {i["new_lo"]:.0f}..'
            f'{i["new_hi"]:.0f}, margin {i["margin_ticks"]:.0f} ticks each side at least)')
        out.append(f'  id {sid} Homing_Offset: calibration {i["recorded_offset"]} '
                   f'({sm_decode(i["recorded_offset"])}), target {i["target_offset"]} '
                   f'({i["target_offset_signed"]}), EEPROM now {current[sid]["offset"]}')
        if 'expected_present' in i:
            out.append(f'  id {sid} present {i["old_present"]} -> expected '
                       f'{i["expected_present"]}')
        out.append(f'  calibration after the write: zero_ticks[{sid - 1}] {i["old_zero"]} -> '
                   f'{i["new_zero"]}; range_center_rad, travel_min/max_rad UNCHANGED '
                   '(radians are invariant under the shift)')
    for n in p.get('notes', []):
        out.append(f'  NOTE: {n}')
    for w in p['why']:
        out.append(f'  PROBLEM: {w}')
    return out


def foreign_holders(port: str) -> List[str]:
    dev = os.path.realpath(port)
    out = []
    for d in Path('/proc').iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            cmd = (d / 'cmdline').read_bytes()
            text = cmd.replace(b'\0', b' ').decode(errors='replace')[:100]
            if any(f in cmd for f in FOREIGN):
                out.append(f'pid {d.name}: {text}')
                continue
            for fd in (d / 'fd').iterdir():
                if os.path.realpath(fd) == dev:
                    out.append(f'pid {d.name} holds {port}: {text}')
                    break
        except OSError:
            pass
    return out


def hold_exclusive(bus) -> bool:
    import fcntl
    import termios
    try:
        fcntl.ioctl(bus._port.ser.fileno(), termios.TIOCEXCL)
        return True
    except (AttributeError, OSError):
        return False


def default_calibration() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('gripperx_leader')) / 'config' / \
            'leader_calibration.yaml'
    except Exception:  # noqa: BLE001
        return Path(__file__).resolve().parents[1] / 'config' / 'leader_calibration.yaml'


def run(argv=None, bus=None, holders=foreign_holders, exclusive=hold_exclusive, out=print) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='leader bus to READ (and with --write, write)')
    ap.add_argument('--calibration', type=Path, help='leader calibration (default: package)')
    ap.add_argument('--write', action='store_true', help='WRITE the planned EEPROM values')
    ap.add_argument('--save-previous', type=Path,
                    help='with --write: snapshot file (default leader_eeprom_previous_<UTC>.yaml)')
    ap.add_argument('--restore', type=Path, help='write a saved snapshot back (needs --write)')
    ap.add_argument('--settle-sec', type=float, default=EEPROM_SETTLE_SEC,
                    help=f'wait after every EEPROM write (default {EEPROM_SETTLE_SEC}, TO-VERIFY)')
    args = ap.parse_args(argv)
    cal = load_calibration(str(args.calibration or default_calibration()))
    if not args.port and bus is None:
        unknown = {sid: {k: None for k in ('min', 'max', 'offset', 'torque', 'lock', 'present')}
                   for sid in IDS}
        p = plan(unknown, cal)
        out('\n'.join(describe(unknown, p)))
        if args.write:
            out('--write needs --port')
            return 2
        return 0
    port = args.port or detect_port() or ''
    refuse = holders(port)
    if refuse:
        out('REFUSED (nothing read, nothing written):\n  ' + '\n  '.join(refuse)
            + '\n  stop the leader node first')
        return 1
    if bus is None:
        bus = LeaderBus(port, ids=IDS)
        bus.open(torque_off=False)
    try:
        if not exclusive(bus):
            out(f'WARNING: could not hold {port} exclusively (TIOCEXCL)')
            if args.write:
                out('WRITE REFUSED: without the exclusive hold a starting node could join '
                    'the bus mid-write')
                return 1
        current = read_current(bus)
        p = restore_plan(args.restore, current) if args.restore else plan(current, cal)
        if args.restore:
            out(f'RESTORE mode: target values from {args.restore}')
        out('\n'.join(describe(current, p)))
        if not args.write:
            out('dry run: nothing written')
            return 0
        refuse = write_refusals(current, p)
        if refuse:
            out('WRITE REFUSED:\n  ' + '\n  '.join(refuse))
            return 1
        if not p['writes']:
            out('nothing to write')
            return 0
        prev = args.save_previous or Path(
            'leader_eeprom_previous_'
            f'{datetime.datetime.now(datetime.timezone.utc):%Y%m%dT%H%M%SZ}.yaml')
        if prev.exists():
            def targets_for(state):
                return (restore_plan(args.restore, state) if args.restore
                        else plan(state, cal))['writes']
            why = snapshot_of_record(prev, current, targets_for)
            if why:
                out(f'WRITE REFUSED: {why} - a snapshot is never overwritten (it may be the '
                    'only copy of the original registers); use a new --save-previous file '
                    'only if that state is understood')
                return 1
            out(f'{prev.resolve()} exists and is KEPT as the snapshot of record: the registers '
                'differ from it only by values this plan writes (an interrupted earlier run); '
                'rollback: --write --restore that file')
        else:
            prev.write_text(yaml.safe_dump(snapshot(current)))
            out(f'previous registers saved to {prev.resolve()} (rollback: --write --restore '
                'that file)')
        failures = write(bus, p['writes'], current, args.settle_sec)
        bus.settle(args.settle_sec)
        after = read_current(bus)
        bad = verify(after, p)
        out('\n'.join(describe(after, {'writes': [], 'shift': {}, 'why': []})[:len(after)]))
        for f in failures + bad:
            out(f'PROBLEM: {f}')
        out('written; read-back ' + ('OK' if not (failures or bad) else 'NOT OK - see above; '
                                     f'rollback: --write --restore {prev}'))
        out('NOTE: persistence across a power cycle is proven only by a dry run after one.')
        return 0 if not (failures or bad) else 1
    finally:
        bus.close()


def main():
    sys.exit(run())


if __name__ == '__main__':
    main()
