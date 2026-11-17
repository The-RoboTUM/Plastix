"""Serial layer for the leader arm. The ONLY module that touches scservo_sdk.

The leader is passive by default: this layer reads present positions with one
GroupSyncRead and, on open, writes torque = 0. The ONLY other writes (goal position,
torque on, SRAM Torque_Limit) are made by the self-alignment (aligner.py), only after an
explicit align request (user decision 2026-10-13, FR-17), and by the virtual wall
(wall.py, user decision 2026-10-15), only while leader mode is active. No EEPROM register
is ever written. The SDK is injectable (`sdk=`) so the tests run with a mock and no
serial port.

Registers (STS3215, protocol 0), source: LeRobot src/lerobot/motors/feetech/tables.py,
STS_SMS_SERIES_CONTROL_TABLE (fetched 2026-10-13; same source as gripperx_arm
servo_bus.py): Model_Number 3 (2), Max_Voltage_Limit 14 (1), Min_Voltage_Limit 15 (1),
Min/Max_Position_Limit 9/11 (2), Operating_Mode 33 (1), Torque_Enable 40 (1),
Goal_Position 42 (2), Torque_Limit 48 (2, SRAM), Present_Position 56 (2),
Present_Load 60 (2), Present_Voltage 62 (1). Units are not in that table: voltage
registers are compared raw against each other; the 0.1 V/LSB reading in logs is
TO-VERIFY. Present_Load: magnitude in bits 0-9 (0..1000), direction in bit 10, as the
Feetech FTServo SDK's SMS_STS::ReadLoad decodes it; the magnitude is most likely per
mille of the maximum PWM duty (not a calibrated torque) - unit and direction TO-VERIFY
on this leader (same reading as gripperx_arm servo_bus.py).
"""

import time
from typing import List, Optional, Sequence

ADDR_MODEL = 3
ADDR_MIN_POS_LIMIT = 9
ADDR_MAX_POS_LIMIT = 11
ADDR_MAX_VOLTAGE = 14
ADDR_MIN_VOLTAGE = 15
# STS3215 CW/CCW_Dead_Zone (EEPROM, 1 byte each, LeRobot STS_SMS_SERIES_CONTROL_TABLE):
# the position loop's dead band around the goal. Read for the startup log only, never written.
ADDR_CW_DEAD_ZONE = 26
ADDR_CCW_DEAD_ZONE = 27
ADDR_HOMING_OFFSET = 31          # sign-magnitude, sign bit 11 (STS_SMS_SERIES_ENCODINGS_TABLE)
ADDR_OPERATING_MODE = 33
ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_TORQUE_LIMIT = 48
ADDR_LOCK = 55                   # 0 = EEPROM writable
ADDR_PRESENT = 56
ADDR_PRESENT_LOAD = 60
LOAD_MAGNITUDE_MASK = 0x3FF     # bits 0-9; bit 10 = direction (not used, TO-VERIFY)
ADDR_PRESENT_VOLTAGE = 62
DEFAULT_BAUD = 1_000_000
DEFAULT_IDS = (1, 2, 3, 4, 5, 6)
JOINT_NAMES = ('limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper')


# What the SDK raises on a short, garbled or missing reply instead of returning an error
# code. Robot 2026-10-15 (leader_eeprom --write): right after an EEPROM write the next
# read2ByteTxRx died with IndexError in SCS_MAKEWORD(data[0], data[1]) - a reply with
# COMM_SUCCESS but no data bytes, most likely the late status packet of the write.
# OSError covers pyserial's SerialException.
SDK_ERRORS = (IndexError, TypeError, ValueError, OSError)


class LeaderReadError(RuntimeError):
    pass


class LeaderWriteError(RuntimeError):
    pass


def joint_names(ids: Sequence[int]) -> List[str]:
    """Name by servo id (id 1 = limb1 ... id 6 = gripper), so a missing id never shifts names."""
    return [JOINT_NAMES[i - 1] if 1 <= i <= len(JOINT_NAMES) else f'joint{i}' for i in ids]


def _import_sdk():
    try:
        import scservo_sdk as sdk
    except ImportError as exc:
        raise ImportError(
            'scservo_sdk missing. Install: pip install --user --break-system-packages '
            'feetech-servo-sdk   (see gripperx_leader/README.md)') from exc
    return sdk


class LeaderBus:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD,
                 ids: Sequence[int] = DEFAULT_IDS, sdk=None) -> None:
        self._sdk = sdk if sdk is not None else _import_sdk()
        self._ids = tuple(ids)
        self._port_name = port
        self._baud = baud
        self._port = self._sdk.PortHandler(port)
        self._pkt = self._sdk.PacketHandler(0)
        self.status_bits = {}      # {id: OR of status error bits seen}, see _ok
        self._group = self._sdk.GroupSyncRead(self._port, self._pkt, ADDR_PRESENT, 2)
        for sid in self._ids:
            self._group.addParam(sid)

    def open(self, torque_off: bool = True) -> None:
        """torque_off=False only for the EEPROM maintenance tool, which must write
        nothing but the registers it is asked to (it refuses while torque is on)."""
        if not self._port.openPort():
            raise LeaderReadError(f'cannot open {self._port_name}')
        self._port.setBaudRate(self._baud)
        if torque_off:
            self.torque_off()

    def close(self) -> None:
        self._port.closePort()

    def ping_all(self, ids: Sequence[int] = DEFAULT_IDS) -> dict:
        """{id: answered} for every id; read-only, used for the startup report."""
        out = {}
        for sid in ids:
            try:
                _, result, _ = self._pkt.ping(self._port, sid)
            except SDK_ERRORS:
                result = None
            out[sid] = result == self._sdk.COMM_SUCCESS
        return out

    def torque_off(self) -> None:
        """The leader must be free to move by hand. Unchecked; see torque_off_verified."""
        for sid in self._ids:
            self._pkt.write1ByteTxRx(self._port, sid, ADDR_TORQUE, 0)

    @property
    def ids(self) -> tuple:
        return self._ids

    def _ok(self, sid: int, result, error) -> bool:
        """Comm success only. The status byte's error bits (overload, voltage, ...)
        come WITH a valid answer: they are recorded in `status_bits` (sticky, OR-ed,
        cleared by the caller) and logged by the caller, not treated as a failed
        transfer (audit LA1 L3)."""
        if error:
            self.status_bits[sid] = self.status_bits.get(sid, 0) | int(error)
        return result == self._sdk.COMM_SUCCESS

    def read_byte(self, sid: int, addr: int) -> int:
        try:
            val, result, error = self._pkt.read1ByteTxRx(self._port, sid, addr)
        except SDK_ERRORS as exc:
            raise LeaderReadError(f'servo {sid}: read of address {addr}: malformed or '
                                  f'missing reply ({exc!r})') from exc
        if not self._ok(sid, result, error):
            raise LeaderReadError(f'servo {sid}: read of address {addr} failed ({result}/{error})')
        return int(val)

    def read_word(self, sid: int, addr: int) -> int:
        try:
            val, result, error = self._pkt.read2ByteTxRx(self._port, sid, addr)
        except SDK_ERRORS as exc:
            raise LeaderReadError(f'servo {sid}: read of address {addr}: malformed or '
                                  f'missing reply ({exc!r})') from exc
        if not self._ok(sid, result, error):
            raise LeaderReadError(f'servo {sid}: read of address {addr} failed ({result}/{error})')
        return int(val)

    def _write(self, sid: int, addr: int, value: int, size: int) -> None:
        fn = self._pkt.write1ByteTxRx if size == 1 else self._pkt.write2ByteTxRx
        try:
            result, error = fn(self._port, sid, addr, int(value))
        except SDK_ERRORS as exc:
            raise LeaderWriteError(f'servo {sid}: write {value} to address {addr}: malformed '
                                   f'or missing reply ({exc!r})') from exc
        if not self._ok(sid, result, error):
            raise LeaderWriteError(
                f'servo {sid}: write {value} to address {addr} failed ({result}/{error})')

    def write_register(self, sid: int, addr: int, value: int, size: int) -> None:
        """Raw write for leader_eeprom (maintenance tool) only. Raises LeaderWriteError."""
        self._write(sid, addr, value, size)

    def settle(self, seconds: float) -> None:
        """Wait, then drop whatever arrived meanwhile (a late status packet must not be
        read as the reply to the next request)."""
        if seconds > 0:
            time.sleep(seconds)
        reset = getattr(getattr(self._port, 'ser', None), 'reset_input_buffer', None)
        if reset is not None:
            try:
                reset()
            except OSError:
                pass

    def write_verified(self, sid: int, addr: int, value: int, size: int,
                       settle_s: float, attempts: int) -> None:
        """EEPROM/Lock write for leader_eeprom: write, settle, read the register back;
        up to `attempts` times (re-writing the same value is idempotent). A lost
        acknowledgement with the value in place counts as done. Raises LeaderWriteError
        when the value is not confirmed."""
        last = 'no attempt'
        for _ in range(max(1, int(attempts))):
            try:
                self.write_register(sid, addr, value, size)
            except LeaderWriteError as exc:
                last = str(exc)
            self.settle(settle_s)
            try:
                got = (self.read_byte if size == 1 else self.read_word)(sid, addr)
            except LeaderReadError as exc:
                last = str(exc)
                self.settle(settle_s)
                continue
            if got == value:
                return
            last = f'read back {got}'
        raise LeaderWriteError(f'servo {sid}: register {addr} := {value} not confirmed after '
                               f'{attempts} attempt(s): {last}')

    # Alignment writes (aligner.py only). All SRAM registers.
    def write_goal(self, sid: int, ticks: int) -> None:
        self._write(sid, ADDR_GOAL, int(ticks), 2)

    def write_torque_limit(self, sid: int, value: int) -> None:
        self._write(sid, ADDR_TORQUE_LIMIT, int(value), 2)

    def torque_on(self, sid: int) -> None:
        self._write(sid, ADDR_TORQUE, 1, 1)

    def torque_off_verified(self, ids: Optional[Sequence[int]] = None) -> List[int]:
        """Torque 0 on every servo (or on `ids`), each tried even if another fails, then
        read back. Returns the ids whose torque could NOT be confirmed off (empty = all
        off). Only the read-back decides: status error bits do not make it unconfirmed."""
        ids = self._ids if ids is None else tuple(ids)
        for sid in ids:
            for _ in range(2):
                try:
                    self._write(sid, ADDR_TORQUE, 0, 1)
                    break
                except LeaderWriteError:
                    continue
        bad = []
        for sid in ids:
            try:
                if self.read_byte(sid, ADDR_TORQUE) != 0:
                    bad.append(sid)
            except LeaderReadError:
                bad.append(sid)
        return bad

    def read_diagnostics(self) -> dict:
        """Read-only: {id: {register: value or None}} for the startup log and the
        alignment pre-checks. Never raises."""
        regs = (('model', ADDR_MODEL, 2), ('voltage', ADDR_PRESENT_VOLTAGE, 1),
                ('min_voltage', ADDR_MIN_VOLTAGE, 1), ('max_voltage', ADDR_MAX_VOLTAGE, 1),
                ('mode', ADDR_OPERATING_MODE, 1), ('min_pos', ADDR_MIN_POS_LIMIT, 2),
                ('max_pos', ADDR_MAX_POS_LIMIT, 2), ('torque_limit', ADDR_TORQUE_LIMIT, 2),
                ('offset', ADDR_HOMING_OFFSET, 2),
                ('torque', ADDR_TORQUE, 1),
                ('cw_dead_zone', ADDR_CW_DEAD_ZONE, 1),
                ('ccw_dead_zone', ADDR_CCW_DEAD_ZONE, 1))
        out = {}
        for sid in self._ids:
            d = {}
            for name, addr, size in regs:
                try:
                    d[name] = (self.read_byte if size == 1 else self.read_word)(sid, addr)
                except LeaderReadError:
                    d[name] = None
            out[sid] = d
        return out

    def read_positions(self) -> List[int]:
        """Present positions in `ids` order; raises LeaderReadError on any failure."""
        try:
            result = self._group.txRxPacket()
        except SDK_ERRORS as exc:
            raise LeaderReadError(f'sync read: malformed reply ({exc!r})') from exc
        if result != self._sdk.COMM_SUCCESS:
            raise LeaderReadError(f'sync read failed (comm result {result})')
        out: List[int] = []
        for sid in self._ids:
            try:
                if not self._group.isAvailable(sid, ADDR_PRESENT, 2):
                    raise LeaderReadError(f'servo {sid} returned no data')
                raw = int(self._group.getData(sid, ADDR_PRESENT, 2))
            except SDK_ERRORS as exc:
                raise LeaderReadError(f'servo {sid}: malformed sync-read data ({exc!r})') \
                    from exc
            if raw & 0x8000:
                # Fail closed: LeRobot decodes Present_Position as sign-magnitude (bit 15);
                # a negative value read as a tick would be a mirrored angle. Values
                # 4096..32767 are LEGITIMATE: measured 2026-10-13, limb5 reads 1825..5638
                # continuously (no wrap) over its travel. calibration.to_radians wraps the
                # difference to the zero, so they convert correctly.
                raise LeaderReadError(f'servo {sid}: present position raw {raw} has the '
                                      'sign bit (15) set: negative position')
            out.append(raw)
        return out


def detect_port() -> Optional[str]:
    """Stable path if unambiguous: /dev/gripperx_leader, else the only by-id entry."""
    import glob
    import os
    if os.path.exists('/dev/gripperx_leader'):
        return '/dev/gripperx_leader'
    entries = sorted(glob.glob('/dev/serial/by-id/*'))
    return entries[0] if len(entries) == 1 else None
