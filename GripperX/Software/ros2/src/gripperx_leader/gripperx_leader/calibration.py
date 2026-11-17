"""Optional tick -> radian conversion for the leader. Pure python.

RANGE CENTRE: the output window per joint is [center-pi, center+pi), not [-pi, pi), because
a joint's travel is generally lopsided around the CAD zero and would otherwise flip +-180.
`range_center_rad` is the centre of the recorded travel (TO-VERIFY, default 0).

WRAP: the servo position wraps at 0/4095. The difference to the zero tick is wrapped
into [-ticks_per_rev/2, +ticks_per_rev/2) before conversion, so crossing the wrap
does not jump by 2*pi. ASSUMPTION (not checked at runtime): every joint's travel
around its zero is less than 360 deg, in fact within +-180 deg. A joint that can
really turn further than that cannot be represented this way.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import yaml


@dataclass(frozen=True)
class Calibration:
    zero_ticks: Sequence[Optional[float]]    # None = not calibrated (e.g. no gripper cable)
    sign: Sequence[Optional[int]]    # None = unknown (gripper)
    ticks_per_rev: float = 4096.0
    # True only after the procedure in leader_calibration.example.yaml; published on
    # /arm/leader/calibration_verified, and the follower refuses leader mode while
    # it is false (FR-15 item 5, SR-17 rule 7).
    verified: bool = False
    range_center_rad: Optional[Sequence[float]] = None   # per joint, None = all 0
    # Optional recorded leader travel (continuous radians, record_limits `leader_range`).
    # Used only by the self-alignment to refuse a target outside it; None = not recorded.
    travel_min_rad: Optional[Sequence[Optional[float]]] = None
    travel_max_rad: Optional[Sequence[Optional[float]]] = None
    # Raw Homing_Offset register (31) of each servo WHEN zero_ticks were measured. Ticks
    # are Present = Actual - Homing_Offset, so a servo whose offset differs reads every
    # angle shifted: leader_arm_node refuses alignment and reports the calibration
    # unverified on a mismatch; leader_eeprom plans against it. None = not recorded.
    homing_offset_raw: Optional[Sequence[Optional[int]]] = None
    # Check value per servo: (zero_ticks + signed Homing_Offset) mod ticks_per_rev. An
    # offset shift moves zero_ticks and the offset by the same amount the other way, so
    # this never changes; a half-edited pair breaks it (pair_problems). None = absent.
    zero_invariant_ticks: Optional[Sequence[Optional[int]]] = None

    def select(self, ids: Sequence[int]) -> 'Calibration':
        """Subset for the configured servo ids (file always holds all six, by id).

        Raises ValueError if a configured id has no zero (null) in the file.
        """
        missing = [i for i in ids
                   if self.zero_ticks[i - 1] is None or self.sign[i - 1] is None
                   or self.centers()[i - 1] is None]
        if missing:
            raise ValueError(f'calibration has no zero_ticks/sign for configured servo id(s) {missing}; '
                             'calibrate them or drop them from servo_ids')
        def pick(seq):
            return None if seq is None else [seq[i - 1] for i in ids]
        return Calibration([self.zero_ticks[i - 1] for i in ids],
                           [self.sign[i - 1] for i in ids], self.ticks_per_rev, self.verified,
                           [self.centers()[i - 1] for i in ids],
                           pick(self.travel_min_rad), pick(self.travel_max_rad),
                           pick(self.homing_offset_raw), pick(self.zero_invariant_ticks))

    def pair_problems(self) -> List[str]:
        return list(self.broken_pairs().values())

    def broken_pairs(self) -> dict:
        """{joint index: message} for pairs zero_ticks / homing_offset_raw contradicting
        zero_invariant_ticks. Empty when consistent or when nothing is recorded."""
        if self.homing_offset_raw is None or self.zero_invariant_ticks is None:
            return {}
        out = {}
        for i, (z, o, inv) in enumerate(zip(self.zero_ticks, self.homing_offset_raw,
                                             self.zero_invariant_ticks)):
            if None in (z, o, inv):
                continue
            got = int(round(z + offset_signed(o))) % int(self.ticks_per_rev)
            if got != inv:
                out[i] = (f'joint index {i}: zero_ticks {z} + Homing_Offset {o} '
                          f'({offset_signed(o)}) = {got} mod {int(self.ticks_per_rev)}, '
                          f'zero_invariant_ticks says {inv}')
        return out

    def centers(self) -> List[float]:
        return list(self.range_center_rad) if self.range_center_rad is not None \
            else [0.0] * len(self.zero_ticks)   # entries may be None (unconfigured joint)

    def wrapped_delta(self, ticks: float, zero: float) -> float:
        half = self.ticks_per_rev / 2.0
        return (ticks - zero + half) % self.ticks_per_rev - half

    def to_radians(self, ticks: Sequence[int]) -> List[float]:
        two_pi = 2.0 * math.pi
        out = []
        for t, z, s, c in zip(ticks, self.zero_ticks, self.sign, self.centers()):
            rad = s * self.wrapped_delta(t, z) * two_pi / self.ticks_per_rev
            out.append((rad - c + math.pi) % two_pi + c - math.pi)   # into [c-pi, c+pi)
        return out


def offset_signed(raw: int) -> int:
    """Homing_Offset register: sign-magnitude, sign in bit 11 (LeRobot
    STS_SMS_SERIES_ENCODINGS_TABLE)."""
    return -(raw & 2047) if raw & 2048 else raw & 2047


def load_calibration(path: str, n_joints: int = 6) -> Calibration:
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    zero = list(data['zero_ticks'])
    sign = list(data['sign'])
    if len(zero) != n_joints or len(sign) != n_joints:
        raise ValueError(f'calibration needs {n_joints} zero_ticks and sign entries')
    if any(s not in (1, -1, None) for s in sign):
        raise ValueError('sign entries must be +1, -1 or null')
    verified = data.get('calibration_verified', False)
    if not isinstance(verified, bool):
        raise ValueError('calibration_verified must be true or false')
    centers = [None if c is None else float(c)
               for c in data.get('range_center_rad', [0.0] * n_joints)]
    if len(centers) != n_joints:
        raise ValueError(f'range_center_rad needs {n_joints} entries')
    travel = []
    for key in ('travel_min_rad', 'travel_max_rad'):
        vals = data.get(key)
        if vals is not None:
            if len(vals) != n_joints:
                raise ValueError(f'{key} needs {n_joints} entries')
            vals = [None if v is None else float(v) for v in vals]
        travel.append(vals)
    offsets = data.get('homing_offset_raw')
    if offsets is not None:
        if len(offsets) != n_joints or not all(o is None or (isinstance(o, int) and
                                                             0 <= o <= 4095) for o in offsets):
            raise ValueError(f'homing_offset_raw needs {n_joints} raw register values '
                             '(0..4095) or null')
    inv = data.get('zero_invariant_ticks')
    if inv is not None and (len(inv) != n_joints or not all(
            v is None or (isinstance(v, int) and 0 <= v <= 4095) for v in inv)):
        raise ValueError(f'zero_invariant_ticks needs {n_joints} values 0..4095 or null')
    return Calibration(zero, sign, float(data.get('ticks_per_rev', 4096)), verified, centers,
                       travel[0], travel[1], offsets, inv)
