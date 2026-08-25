"""Minimal Feetech STS/ST bus driver using scservo_sdk (feetech-servo-sdk)."""

from __future__ import annotations

import math
from typing import List, Sequence

from scservo_sdk.group_sync_write import GroupSyncWrite
from scservo_sdk.packet_handler import PacketHandler
from scservo_sdk.port_handler import PortHandler
from scservo_sdk.scservo_def import (
    COMM_SUCCESS,
    SCS_HIBYTE,
    SCS_LOBYTE,
    SCS_TOSCS,
    SCS_TOHOST,
)

STS_ACC = 41
STS_PRESENT_POSITION_L = 56
STS_TORQUE_ENABLE = 40
WRITE_POS_LEN = 7


class StsServoBus:
    def __init__(self, port: str, baud_rate: int = 1_000_000, protocol_end: int = 0) -> None:
        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(protocol_end)
        self._baud_rate = baud_rate
        self._open = False

    def open(self) -> None:
        if self._open:
            return
        if not self._port_handler.openPort():
            raise RuntimeError(f"Could not open servo port {self._port_handler.port_name}")
        if not self._port_handler.setBaudRate(self._baud_rate):
            raise RuntimeError(f"Could not set baud rate {self._baud_rate}")
        self._open = True

    def close(self) -> None:
        if self._open:
            self._port_handler.closePort()
            self._open = False

    def ping(self, servo_id: int) -> bool:
        model_number, result, error = self._packet_handler.ping(self._port_handler, servo_id)
        return result == COMM_SUCCESS and error == 0 and model_number is not None

    def enable_torque(self, servo_id: int, enable: bool = True) -> None:
        value = 1 if enable else 0
        result, error = self._packet_handler.write1ByteTxRx(
            self._port_handler, servo_id, STS_TORQUE_ENABLE, value
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(f"enable_torque failed for id={servo_id} result={result} error={error}")

    def read_position(self, servo_id: int) -> int:
        raw, result, error = self._packet_handler.read2ByteTxRx(
            self._port_handler, servo_id, STS_PRESENT_POSITION_L
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(f"read_position failed for id={servo_id} result={result} error={error}")
        return normalize_position_ticks(int(raw))

    def write_position_timed(
        self,
        servo_id: int,
        position: int,
        time_ms: int,
        acceleration: int = 0,
    ) -> None:
        payload = self._encode_position_packet(int(position), 0, acceleration, int(time_ms))
        result, error = self._packet_handler.writeTxRx(
            self._port_handler, servo_id, STS_ACC, WRITE_POS_LEN, payload
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"write_position_timed failed for id={servo_id} result={result} error={error}"
            )

    def sync_write_positions_timed(
        self,
        servo_ids: Sequence[int],
        positions: Sequence[int],
        time_ms: int,
        acceleration: int = 0,
    ) -> None:
        if len(servo_ids) != len(positions):
            raise ValueError("servo_ids and positions must have the same length")
        if not servo_ids:
            return

        group = GroupSyncWrite(self._port_handler, self._packet_handler, STS_ACC, WRITE_POS_LEN)
        for servo_id, position in zip(servo_ids, positions):
            payload = self._encode_position_packet(int(position), 0, acceleration, int(time_ms))
            if not group.addParam(int(servo_id), payload):
                raise RuntimeError(f"sync_write addParam failed for id={servo_id}")

        result = group.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"sync_write failed result={result}")

    def write_position(
        self,
        servo_id: int,
        position: int,
        speed: int,
        acceleration: int = 0,
    ) -> None:
        payload = self._encode_position_packet(int(position), int(speed), acceleration, 0)
        result, error = self._packet_handler.writeTxRx(
            self._port_handler, servo_id, STS_ACC, WRITE_POS_LEN, payload
        )
        if result != COMM_SUCCESS or error != 0:
            raise RuntimeError(
                f"write_position failed for id={servo_id} result={result} error={error}"
            )

    def sync_write_positions(
        self,
        servo_ids: Sequence[int],
        positions: Sequence[int],
        speed: int,
        acceleration: int = 0,
    ) -> None:
        if len(servo_ids) != len(positions):
            raise ValueError("servo_ids and positions must have the same length")
        if not servo_ids:
            return

        group = GroupSyncWrite(self._port_handler, self._packet_handler, STS_ACC, WRITE_POS_LEN)
        speed_word = int(speed)
        for servo_id, position in zip(servo_ids, positions):
            payload = self._encode_position_packet(int(position), speed_word, acceleration, 0)
            if not group.addParam(int(servo_id), payload):
                raise RuntimeError(f"sync_write addParam failed for id={servo_id}")

        result = group.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"sync_write failed result={result}")

    @staticmethod
    def _encode_position_packet(
        position: int,
        speed: int,
        acceleration: int,
        time_ms: int = 0,
    ) -> List[int]:
        scs_pos = int(SCS_TOSCS(position, 15))
        return [
            int(acceleration) & 0xFF,
            SCS_LOBYTE(scs_pos),
            SCS_HIBYTE(scs_pos),
            SCS_LOBYTE(int(time_ms)),
            SCS_HIBYTE(int(time_ms)),
            SCS_LOBYTE(int(speed)),
            SCS_HIBYTE(int(speed)),
        ]


def normalize_position_ticks(raw: int) -> int:
    """Feetech position in ticks 0..4095 (one revolution)."""
    return int(raw) & 0x0FFF


def calibrated_counts_bounds(counts_at_pos_limit: int, counts_at_neg_limit: int) -> tuple[int, int]:
    """Hard count window a calibrated conversion may never leave.

    Both arguments are raw positions that were physically reached during
    calibration, so anything between them is mechanically safe and anything
    outside is not — independent of any angle labelling. This is the safety
    property of the calibrated conversion; keep it applied last.
    """
    low = int(round(min(counts_at_neg_limit, counts_at_pos_limit)))
    high = int(round(max(counts_at_neg_limit, counts_at_pos_limit)))
    return low, high


def calibrated_angle_to_counts(
    angle_rad: float,
    center: int,
    counts_plus_90: int,
    counts_minus_90: int,
    limit_rad: float,
) -> int:
    """Symmetric legacy mapping: one limit for both directions.

    DEPRECATED — kept unchanged for API compatibility and as the fallback for
    configs that carry only the old keys. `counts_plus_90`/`counts_minus_90` are
    NOT the counts at ±90 deg; they are the counts recorded at ±`limit_rad`
    (misleading legacy names). Use `calibrated_angle_to_counts_asym()` for the
    per-direction model.
    """
    if limit_rad <= 0.0:
        return center

    angle_rad = max(-limit_rad, min(limit_rad, angle_rad))
    if angle_rad >= 0.0:
        counts = center + (counts_plus_90 - center) * (angle_rad / limit_rad)
    else:
        counts = center + (counts_minus_90 - center) * (angle_rad / -limit_rad)

    low, high = calibrated_counts_bounds(counts_plus_90, counts_minus_90)
    return max(low, min(high, int(round(counts))))


def calibrated_counts_to_rad(
    counts: int,
    center: int,
    counts_plus_90: int,
    counts_minus_90: int,
    limit_rad: float,
) -> float:
    """Symmetric legacy inverse of `calibrated_angle_to_counts`. DEPRECATED."""
    counts = normalize_position_ticks(counts)
    if limit_rad <= 0.0:
        return 0.0

    if counts >= center:
        span = float(counts_plus_90 - center)
        angle = 0.0 if abs(span) < 1e-6 else limit_rad * (counts - center) / span
    else:
        span = float(center - counts_minus_90)
        angle = 0.0 if abs(span) < 1e-6 else -limit_rad * (center - counts) / span
    return max(-limit_rad, min(limit_rad, angle))


# --- Per-direction (asymmetric) calibration model ----------------------------
#
# The steering range is mechanically asymmetric: a wheel can swing much further
# away from the chassis than towards it (100 deg / 30 deg, measured by the user
# 2026-08-13). A single shared limit cannot express that — see the module-level
# discussion in steer_servo_node.py.
#
# These two functions deliberately know nothing about "outward"/"inward". They
# work purely on the SIGN of the joint angle: one limit and one recorded
# endpoint count per sign. Which sign is physically outward differs per wheel
# and is resolved by the caller (steer_servo_node: `steering_outward_sign`).
#
# `pos_limit_rad`/`neg_limit_rad` are magnitudes (both > 0). The negative-side
# limit means "the joint may reach -neg_limit_rad".


def calibrated_angle_to_counts_asym(
    angle_rad: float,
    center: int,
    counts_at_pos_limit: int,
    counts_at_neg_limit: int,
    pos_limit_rad: float,
    neg_limit_rad: float,
) -> int:
    """Map a joint angle to raw counts with one limit per direction.

    Linear between `center` (0 rad) and the endpoint count recorded at the limit
    of the respective side; the result is hard-clamped to the recorded endpoint
    window (`calibrated_counts_bounds`), so a wrong angle label can never push
    the servo past a position that was physically reached during calibration.

    Reduces exactly to `calibrated_angle_to_counts()` when both limits are equal.
    """
    if angle_rad >= 0.0:
        limit_rad = pos_limit_rad
        counts_at_limit = counts_at_pos_limit
    else:
        limit_rad = neg_limit_rad
        counts_at_limit = counts_at_neg_limit

    if limit_rad <= 0.0:
        return center

    angle_rad = max(-limit_rad, min(limit_rad, angle_rad))
    counts = center + (counts_at_limit - center) * (abs(angle_rad) / limit_rad)

    low, high = calibrated_counts_bounds(counts_at_pos_limit, counts_at_neg_limit)
    return max(low, min(high, int(round(counts))))


def calibrated_counts_to_rad_asym(
    counts: int,
    center: int,
    counts_at_pos_limit: int,
    counts_at_neg_limit: int,
    pos_limit_rad: float,
    neg_limit_rad: float,
) -> float:
    """Inverse of `calibrated_angle_to_counts_asym`.

    The side is picked by the count DIRECTION of the positive endpoint, not by
    `counts >= center`: a servo whose counts decrease with a growing joint angle
    (mirrored mount) is expected after the rework calibration, and with two
    different spans the naive `counts >= center` test would scale the readback
    with the wrong span. For a non-mirrored mount (`counts_at_pos_limit >
    center`) this is identical to the legacy branch.
    """
    counts = normalize_position_ticks(counts)
    if pos_limit_rad <= 0.0 and neg_limit_rad <= 0.0:
        return 0.0

    delta = float(counts - center)
    pos_span = float(counts_at_pos_limit - center)
    neg_span = float(counts_at_neg_limit - center)

    on_pos_side = abs(pos_span) >= 1e-6 and (delta * pos_span) >= 0.0
    if on_pos_side:
        angle = 0.0 if pos_limit_rad <= 0.0 else pos_limit_rad * delta / pos_span
    elif abs(neg_span) < 1e-6 or neg_limit_rad <= 0.0:
        angle = 0.0
    else:
        angle = -neg_limit_rad * delta / neg_span

    return max(-neg_limit_rad, min(pos_limit_rad, angle))


def rad_to_counts(angle_rad: float, center_counts: int, counts_per_rev: int, sign: float) -> int:
    return int(
        round(center_counts + (sign * angle_rad / (2.0 * 3.141592653589793)) * counts_per_rev)
    )


def clamp_counts_near_center(counts: int, center_counts: int, max_offset_counts: int) -> int:
    low = center_counts - max_offset_counts
    high = center_counts + max_offset_counts
    return max(low, min(high, counts))


def counts_to_rad(counts: int, center_counts: int, counts_per_rev: int, sign: float) -> float:
    return sign * (float(counts - center_counts) / float(counts_per_rev)) * (2.0 * 3.141592653589793)
