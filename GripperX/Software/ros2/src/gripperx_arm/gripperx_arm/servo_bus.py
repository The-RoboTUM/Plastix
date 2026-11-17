"""Thin wrapper around the Feetech SDK for the arm bus, so the arm node can be
tested against a mock that records every write.

Register addresses (STS3215, protocol 0), source: LeRobot
src/lerobot/motors/feetech/tables.py, STS_SMS_SERIES_CONTROL_TABLE (fetched
2026-10-07): Min_Position_Limit (9, 2), Max_Position_Limit (11, 2),
Homing_Offset (31, 2), Torque_Enable (40, 1), Goal_Position (42, 2),
Lock (55, 1), Present_Position (56, 2). Homing_Offset is sign-magnitude with the
sign in bit 11 (STS_SMS_SERIES_ENCODINGS_TABLE {"Homing_Offset": 11}), and
Present_Position = Actual_Position - Homing_Offset (lerobot/motors/feetech/feetech.py).
LeRobot releases Lock (0) whenever it disables torque and sets it (1) on enable,
i.e. EEPROM writes there happen with Lock = 0. The 40/42/56 values match what this
node has used on the robot since 2026-07; 9/11/31/55 have NOT been exercised on
this robot yet.

Present_Load (60, 2), added 2026-10-13 for the hold contact threshold (audit HG1 F3):
address from the same LeRobot control table; magnitude 0..1000, likely per mille
of the maximum PWM duty (not a calibrated torque), with the direction in bit 10, as the Feetech FTServo SDK's
SMS_STS::ReadLoad decodes it (SMS_STS_PRESENT_LOAD_L = 60). Read only, for logging;
NOT exercised on this robot yet - unit and sign bit TO-VERIFY on the first read.

Torque_Limit (48, 2, SRAM) and Max_Torque_Limit (16, 2, EEPROM), added 2026-11-16 for the
leader gripper limit (FR-17 amendment): addresses from the same LeRobot control table;
48 is the register gripperx_leader leader_bus.py already writes on the leader (same
source, fetched 2026-10-13). 0..1000, LeRobot so_follower.py writes 500 as "50% of max
torque"; 0 makes the STS3215 brake (gripperx_leader, robot 2026-10-20). This node WRITES
only 48 (RAM, lost at power-off) and only on servo 6; 16 is READ only, never written. That
the RAM value starts from 16 at power-on is TO-VERIFY; neither address has been exercised
on the follower yet.
"""
import sys
import termios
import time
from typing import Optional

ADDR_MODEL_NUMBER = 3         # (3, 2) in the same LeRobot table; TO-VERIFY on this robot
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_HOMING_OFFSET = 31
ADDR_TORQUE = 40
ADDR_GOAL_POS = 42
ADDR_MAX_TORQUE_LIMIT = 16    # EEPROM, read only (see docstring)
ADDR_TORQUE_LIMIT = 48        # SRAM
TORQUE_LIMIT_MAX = 1000
ADDR_MOVE_TIME = 44
ADDR_LOCK = 55
ADDR_PRESENT = 56
ADDR_PRESENT_LOAD = 60
LOAD_SIGN_BIT = 10


# Hard upper bound on ONE bus transaction (2026-10-13, robot event of 17:42). The SDK
# opens the port with timeout=0 and bounds a reply by wall clock in rxPacket
# (isPacketTimeout, ~34 ms at 1 Mbaud: 2 x LATENCY_TIMER 16 ms + 2 ms + bytes) - but
# only on the branches where too few bytes arrived: a stream of junk or of foreign
# packets keeps it looping (header search, txRxPacket's wait for the right id) without
# ever reaching that check. Our readPort wrapper ends such a transaction here as a
# failed one. TO-VERIFY: 0.25 s, far above the SDK's own reply timeout.
TXN_MAX_S = 0.25


class BusTimeout(Exception):
    """Raised inside the SDK's receive loop when TXN_MAX_S has passed."""


# What the SDK raises on a short or garbled reply instead of returning an error code.
# Leader robot run 2026-10-15 (same SDK): right after an EEPROM write, read2ByteTxRx
# died with IndexError in SCS_MAKEWORD(data[0], data[1]) - a reply with COMM_SUCCESS
# but no data bytes. A transaction ending this way is a failed one (None), like one ended
# by TXN_MAX_S. Deliberately NOT here (audit EW1): OSError - pyserial's SerialException
# from an unplugged adapter must reach arm_action_server's _BusGuard, which marks the
# bus lost and reopens it (2026-10-13 reconnect fix); TypeError - a programming error
# must not turn into a silent None.
SDK_ERRORS = (IndexError, ValueError)


POSITION_SIGN_BIT = 15        # Present_Position / Goal_Position (encoding table, see above)


def sm_decode(raw: int, bit: int) -> int:
    """Sign-magnitude register value -> signed int (sign in `bit`)."""
    mag = raw & ((1 << bit) - 1)
    return -mag if raw & (1 << bit) else mag


def sm_encode(value: int, bit: int) -> int:
    if abs(value) >= (1 << bit):
        raise ValueError(f'{value} does not fit a sign-magnitude field with sign bit {bit}')
    return (abs(value) | (1 << bit)) if value < 0 else value


def _sdk():
    sys.path.insert(0, '/home/ubuntu/.local/lib/python3.12/site-packages')
    import scservo_sdk as scs
    return scs


def _os_errors(reset) -> None:
    """Call `reset` (or nothing); termios.error (a hung-up tty) becomes an OSError so
    _BusGuard sees the lost bus (audit EW1 M1)."""
    if reset is None:
        return
    try:
        reset()
    except termios.error as exc:
        raise OSError(*exc.args) from exc


class FeetechBus:
    def __init__(self, port: str, baudrate: int):
        self._scs = _sdk()
        self._port = self._scs.PortHandler(port)
        self._pkt = self._scs.PacketHandler(0)
        self._name, self._baud = port, baudrate
        self._bound()

    def _bound(self):
        """Install the TXN_MAX_S check into the port's readPort, which the SDK's
        receive loops call on every pass."""
        self._deadline: Optional[float] = None
        self.timeouts = 0                     # transactions ended by TXN_MAX_S
        self.malformed = 0                    # transactions ended by a short reply
        self.last_error: Optional[str] = None  # why the last read failed (None: it did not)
        real_read = self._port.readPort

        def read_port(length):
            if self._deadline is not None and time.monotonic() > self._deadline:
                raise BusTimeout(f'bus transaction longer than {TXN_MAX_S} s')
            return real_read(length)
        self._port.readPort = read_port

    def _txn(self, fn, *args):
        """One SDK transaction, bounded by TXN_MAX_S: None when the bound ended it.
        Not bounded here (audit HG3 H2, TO-VERIFY): the SDK's txPacket flush (tcdrain)
        and writePort.
        The SDK's is_using flag is reset (it would refuse every later transaction as
        busy) and pending input dropped."""
        # Stale input first (audit HG3 H3): a reply that arrived after its transaction
        # was given up would otherwise be read as the next same-id reply.
        reset = getattr(getattr(self._port, 'ser', None), 'reset_input_buffer', None)
        _os_errors(reset)
        self._deadline = time.monotonic() + TXN_MAX_S
        try:
            return fn(*args)
        except SDK_ERRORS:
            self.malformed = getattr(self, 'malformed', 0) + 1
            self.last_error = 'short or garbled reply'
            self._port.is_using = False
            _os_errors(reset)
            return None
        except BusTimeout:
            self.timeouts += 1
            self.last_error = f'transaction bound {TXN_MAX_S} s'
            self._port.is_using = False
            _os_errors(reset)
            return None
        except termios.error as exc:
            # Audit EW1 M1: tcflush/tcdrain on a hung-up tty raise termios.error, which
            # is no OSError and would bypass _BusGuard: re-raised as one.
            self._port.is_using = False
            raise OSError(*exc.args) from exc
        except OSError:
            self._port.is_using = False      # _BusGuard marks the bus lost and reopens it
            raise
        finally:
            self._deadline = None

    def health(self) -> tuple:
        """(timeouts, malformed): transactions ended by TXN_MAX_S / by a short reply,
        since the bus object was made. For the node's rate-limited log."""
        return self.timeouts, getattr(self, 'malformed', 0)

    def open(self):
        if not self._port.openPort():
            raise RuntimeError(f'Cannot open {self._name}')
        self._port.setBaudRate(self._baud)

    def close(self):
        self._port.closePort()

    def reopen(self):
        """Close (errors ignored: the device may be gone) and open the port again by
        its NAME - /dev/arm_servo, the udev symlink, follows a re-enumerated adapter
        (2026-10-13: ttyACM -> ttyACM3). Raises if it cannot be opened."""
        try:
            self._port.closePort()
        except Exception:     # noqa: BLE001 - the old device is gone
            pass
        self._port.is_using = False
        self.open()

    def torque(self, sid: int, on: bool) -> bool:
        r = self._txn(self._pkt.write1ByteTxRx, self._port, sid, ADDR_TORQUE, 1 if on else 0)
        return r is not None and r[0] == self._scs.COMM_SUCCESS

    def read_word(self, sid: int, addr: int) -> Optional[int]:
        self.last_error = None
        r = self._txn(self._pkt.read2ByteTxRx, self._port, sid, addr)
        if r is None:
            return None
        if r[1] != self._scs.COMM_SUCCESS:
            self.last_error = self._comm_text(r[1])
            return None
        return r[0]

    def _comm_text(self, result: int) -> str:
        """The SDK's text for a failed transaction (COMM_RX_TIMEOUT: no complete reply
        within its ~34 ms, by wall clock - see TXN_MAX_S)."""
        try:
            return f'{self._pkt.getTxRxResult(result)} (code {result})'
        except Exception:     # noqa: BLE001 - a diagnostic never fails the read
            return f'code {result}'

    def read_byte(self, sid: int, addr: int) -> Optional[int]:
        r = self._txn(self._pkt.read1ByteTxRx, self._port, sid, addr)
        return r[0] if r is not None and r[1] == self._scs.COMM_SUCCESS else None

    def read_pos(self, sid: int) -> Optional[int]:
        """Present position, DECODED: with a Homing_Offset the servo can report a
        negative position (sign bit 15), which the raw word would show as 32768 + x."""
        raw = self.read_word(sid, ADDR_PRESENT)
        return None if raw is None else sm_decode(raw, POSITION_SIGN_BIT)

    def read_load(self, sid: int) -> Optional[int]:
        """Present load, signed, likely per mille of max PWM duty (module docstring)."""
        raw = self.read_word(sid, ADDR_PRESENT_LOAD)
        return None if raw is None else sm_decode(raw, LOAD_SIGN_BIT)

    def write_goal(self, sid: int, ticks: int) -> bool:
        r = self._txn(self._pkt.write2ByteTxRx, self._port, sid, ADDR_GOAL_POS,
                      sm_encode(int(ticks), POSITION_SIGN_BIT))
        return r is not None and r[0] == self._scs.COMM_SUCCESS

    def write_word(self, sid: int, addr: int, value: int) -> bool:
        r = self._txn(self._pkt.write2ByteTxRx, self._port, sid, addr, int(value))
        return r is not None and r[0] == self._scs.COMM_SUCCESS

    def write_byte(self, sid: int, addr: int, value: int) -> bool:
        r = self._txn(self._pkt.write1ByteTxRx, self._port, sid, addr, int(value))
        return r is not None and r[0] == self._scs.COMM_SUCCESS
