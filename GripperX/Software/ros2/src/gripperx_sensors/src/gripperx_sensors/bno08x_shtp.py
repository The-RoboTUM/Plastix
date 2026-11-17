"""BNO08x (BNO085) over I2C: the SHTP transport and the SH-2 reports we use.

No ROS here, and no dependency beyond the standard library: the bus is the Linux
i2c-dev character device, driven with one ioctl (I2C_SLAVE) and plain read() /
write(). Each read() or write() on i2c-dev is one complete I2C transaction
(START .. STOP), which is exactly what SHTP over I2C expects. smbus/smbus2 are
not installed on the robot and are not needed.

SHTP (Sensor Hub Transport Protocol) framing: every packet starts with a 4-byte
header [length LSB, length MSB, channel, sequence]. The length includes the
header; bit 15 is the continuation flag. Reading is two transactions: read the
4-byte header to learn the length, then read the whole packet, which the device
sends again from the header on (BNO08X datasheet 1000-3927 v1.17 section 1.3.1:
"the host could read the first 4 bytes ... and then repeat the read").
A header with length 0 means "nothing pending".

The continuation bit is judged on the FIRST header only. The datasheet calls the
header read a partial read whose remainder follows in a later transfer, and the
sensor marks that later transfer as a continuation. Measured on the robot
2026-10-20 (raw probe, 12 packets, all alike): the re-read header carries bit 15
set and the sequence number incremented, e.g. `13 00 03 1c` -> `13 80 03 1d`,
`c7 00 03 1e` -> `c7 80 03 1f`. So bit 15 and the sequence number on the re-read
say nothing; only length and channel must match. A first header with bit 15
set is the tail of a message whose start this host never read: it is read out,
dropped and counted (`Shtp.fragments_dropped`), not reassembled.

Packets of 199 bytes were seen in the same probe (several samples batched after
a backlog); i2c-dev reads up to 8192 bytes in one transaction and the parser
walks any number of reports, so no size limit below the 15-bit length applies.

SH-2 (Sensor Hub 2, the sensor-hub command set on top of SHTP) parts used here:
  channel 1 (executable)  host->dev 0x01 = soft reset; dev->host 0x01 = reset done
  channel 2 (control)     0xFD Set Feature, 0xF9/0xF8 Product ID request/response,
                          0xF1 command response (0x84 = unsolicited "initialised")
  channel 3 (input)       0xFB base timestamp, then sensor reports back to back

Sources: CEVA "SH-2 Reference Manual" 1000-3625 rev 1.9 (report IDs, Q points,
Set Feature layout; section numbers below are that revision) and "BNO08X Data
Sheet" 1000-3927 v1.17 (framing, section 1.3.1), cross-checked against the
Adafruit and SparkFun BNO08x libraries.
"""

from __future__ import annotations

import fcntl
import math
import os
import struct
from dataclasses import dataclass, field
from typing import NamedTuple

I2C_SLAVE = 0x0703  # linux/i2c-dev.h

CH_COMMAND = 0
CH_EXECUTABLE = 1
CH_CONTROL = 2
CH_REPORTS = 3
CH_WAKE_REPORTS = 4
CH_GYRO_RV = 5
_MAX_CHANNEL = 5

REPORT_ACCELEROMETER = 0x01
REPORT_GYROSCOPE = 0x02
REPORT_ROTATION_VECTOR = 0x05
REPORT_GAME_ROTATION_VECTOR = 0x08
REPORT_RAW_ACCELEROMETER = 0x14

ORIENTATION_REPORTS = {
    "game_rotation_vector": REPORT_GAME_ROTATION_VECTOR,
    "rotation_vector": REPORT_ROTATION_VECTOR,
}

# Every report this package's tools ever switch on (driver and bno085_query).
# Set Feature is volatile but outlives the process that sent it, so the driver
# switches off whichever of these it is not using at every start; a report
# that is enabled anywhere in the package must be listed here.
MANAGED_REPORTS = frozenset({
    REPORT_ACCELEROMETER, REPORT_GYROSCOPE, REPORT_ROTATION_VECTOR,
    REPORT_GAME_ROTATION_VECTOR, REPORT_RAW_ACCELEROMETER,
})

# Total length in bytes of each input report, including its 4-byte report
# header. Needed to walk a packet that carries several reports back to back.
# Only IDs we enable or that a stock BNO085 may interleave are listed; anything
# else ends parsing of that packet (counted, never guessed at).
REPORT_LENGTH = {
    0x01: 10,  # accelerometer (gravity included), Q8 m/s^2
    0x02: 10,  # gyroscope, calibrated, Q9 rad/s
    0x03: 10,  # magnetic field, calibrated, Q4 uT
    0x04: 10,  # linear acceleration, Q8 m/s^2
    0x05: 14,  # rotation vector, Q14 quaternion + Q12 accuracy (rad)
    0x06: 10,  # gravity, Q8 m/s^2
    0x07: 16,  # gyroscope, uncalibrated
    0x08: 12,  # game rotation vector (no magnetometer), Q14 quaternion
    0x09: 14,  # geomagnetic rotation vector
    0x14: 16,  # raw accelerometer, ADC counts in the MEMS chip's own axes
    0xFA: 5,   # timestamp rebase
    0xFB: 5,   # base timestamp reference
}

_Q8 = 1.0 / (1 << 8)
_Q9 = 1.0 / (1 << 9)
_Q12 = 1.0 / (1 << 12)
_Q14 = 1.0 / (1 << 14)

EXEC_RESET = 0x01
CTRL_SET_FEATURE = 0xFD
CTRL_PRODUCT_ID_REQUEST = 0xF9
CTRL_PRODUCT_ID_RESPONSE = 0xF8
CTRL_COMMAND_RESPONSE = 0xF1
CTRL_COMMAND_REQUEST = 0xF2
CTRL_FRS_READ_REQUEST = 0xF4
CTRL_FRS_READ_RESPONSE = 0xF3
CMD_INITIALIZE = 0x84
CMD_ME_CALIBRATION = 0x07
ME_CAL_SUBCMD_GET = 0x01

# FRS (flash record system) record types, BNO08X datasheet 1000-3927 v1.17
# section 1.3.3. Only READ helpers exist in this module; nothing here writes FRS.
FRS_DYNAMIC_CALIBRATION = 0x1F1F
FRS_SYSTEM_ORIENTATION = 0x2D3E
FRS_ACCEL_ORIENTATION = 0x2D41
FRS_GYRO_ORIENTATION = 0x2D46

# FRS Read Response status, SH-2 rev 1.9 section 6.3.7. 4, 6 and 7 are marked
# "deprecated" there; they still end a read if a firmware sends them.
FRS_READ_STATUS = {
    0: "no error", 1: "unrecognized FRS type", 2: "busy", 3: "read record completed",
    4: "deprecated status 4", 5: "record empty", 6: "deprecated status 6",
    7: "deprecated status 7", 8: "device error",
}
_FRS_DONE = {1, 3, 4, 5, 7, 8}


def get_me_calibration_payload(seq: int) -> bytes:
    """Command Request 0xF2 / 0x07 subcommand 0x01: READ the dynamic-calibration
    enable flags (SH-2 Reference Manual 1000-3625 rev 1.9, 6.4.6.2). Changes nothing."""
    return bytes([CTRL_COMMAND_REQUEST, seq & 0xFF, CMD_ME_CALIBRATION,
                  0, 0, 0, ME_CAL_SUBCMD_GET, 0, 0, 0, 0, 0])


def parse_me_calibration_response(payload: bytes) -> dict | None:
    """Fields of an ME Calibration Response (6.4.6.3), or None if it is not one."""
    if len(payload) < 10 or payload[0] != CTRL_COMMAND_RESPONSE or payload[2] != CMD_ME_CALIBRATION:
        return None
    return {"status": payload[5], "accel": payload[6], "gyro": payload[7],
            "mag": payload[8], "planar_accel": payload[9]}


def frs_read_request_payload(record: int) -> bytes:
    """FRS Read Request 0xF4 (rev 1.9 section 6.3.6): reads the whole record.
    Bytes 1-3 and 6-7 are "Reserved" in rev 1.9 and sent as zero (older
    revisions used them for a read offset and block size)."""
    return struct.pack("<BBHHH", CTRL_FRS_READ_REQUEST, 0, 0, record, 0)


def parse_frs_read_response(payload: bytes):
    """(record, status, word_offset, [words]) of an FRS Read Response (rev 1.9 6.3.7), or None."""
    if len(payload) < 16 or payload[0] != CTRL_FRS_READ_RESPONSE:
        return None
    n_words = payload[1] >> 4
    status = payload[1] & 0x0F
    offset = payload[2] | (payload[3] << 8)
    words = list(struct.unpack_from("<II", payload, 4))[:n_words]
    record = payload[12] | (payload[13] << 8)
    return record, status, offset, words


def frs_read_done(status: int) -> bool:
    return status in _FRS_DONE


class ShtpError(Exception):
    """A header or packet that cannot be a valid SHTP packet."""


class BusBusyError(OSError):
    """Another process holds the i2c-dev lock (e.g. a second driver instance)."""


class LockPermissionError(PermissionError):
    """The lock file exists but this user can open it neither read-write nor
    read-only. Retrying does not help; someone has to remove the file."""


class Quat(NamedTuple):
    xyzw: tuple[float, float, float, float]
    status: int
    accuracy_rad: float | None  # rotation vector (0x05) only


@dataclass
class Sample:
    """What one channel-3 packet updated. None = not in this packet."""

    accel: tuple[float, float, float] | None = None
    accel_status: int = 0
    gyro: tuple[float, float, float] | None = None
    gyro_status: int = 0
    # Keyed by report id: a packet may carry more than one orientation report
    # (e.g. a rotation vector a previous run left enabled), and the consumer
    # must pick the one it configured rather than whichever came last.
    quats: dict[int, Quat] = field(default_factory=dict)
    raw_accel: tuple[int, int, int] | None = None
    unknown_report_ids: list[int] = field(default_factory=list)


def parse_header(data: bytes) -> tuple[int, int, int, bool]:
    """Return (length, channel, sequence, continuation) of a 4-byte header.

    Raises ShtpError for a header no BNO08x sends. 0xFFFF is what a bus with
    nothing driving it reads as, and is the commonest sign of a glitched read.
    """
    if len(data) < 4:
        raise ShtpError(f"short header ({len(data)} bytes)")
    raw = data[0] | (data[1] << 8)
    if raw == 0xFFFF:
        raise ShtpError("header reads 0xFFFF (bus idle or glitched read)")
    length = raw & 0x7FFF
    channel = data[2]
    if channel > _MAX_CHANNEL:
        raise ShtpError(f"channel {channel} out of range")
    if 0 < length < 4:
        raise ShtpError(f"length {length} shorter than the header")
    return length, channel, data[3], bool(raw & 0x8000)


def set_feature_payload(report_id: int, interval_us: int) -> bytes:
    """SH-2 Set Feature command (17 bytes, rev 1.9 section 6.5.4).

    Layout: id, report id, feature flags, change sensitivity (u16), report
    interval (u32, us), batch interval (u32, us), sensor-specific config (u32).
    Flags, sensitivity, batch interval and config are all zero: no change
    trigger, no wake-up, "do not delay" (6.5.1). interval_us = 0 switches the
    sensor off (5.4.1).
    """
    return struct.pack("<BBBHIII", CTRL_SET_FEATURE, report_id, 0, 0, interval_us, 0, 0)


def parse_input_packet(payload: bytes) -> Sample:
    """Walk the reports of one channel-3 payload (header already stripped)."""
    sample = Sample()
    i = 0
    n = len(payload)
    while i < n:
        rid = payload[i]
        length = REPORT_LENGTH.get(rid)
        if length is None or i + length > n:
            sample.unknown_report_ids.append(rid)
            break
        rep = payload[i:i + length]
        # Byte 2 of a sensor report is the status; its low two bits are the
        # SH-2 accuracy estimate: 0 unreliable, 1 low, 2 medium, 3 high.
        if rid == REPORT_ACCELEROMETER:
            x, y, z = struct.unpack_from("<hhh", rep, 4)
            sample.accel = (x * _Q8, y * _Q8, z * _Q8)
            sample.accel_status = rep[2] & 0x03
        elif rid == REPORT_GYROSCOPE:
            x, y, z = struct.unpack_from("<hhh", rep, 4)
            sample.gyro = (x * _Q9, y * _Q9, z * _Q9)
            sample.gyro_status = rep[2] & 0x03
        elif rid in (REPORT_GAME_ROTATION_VECTOR, REPORT_ROTATION_VECTOR):
            qi, qj, qk, qr = struct.unpack_from("<hhhh", rep, 4)
            accuracy = None
            if rid == REPORT_ROTATION_VECTOR:
                (acc,) = struct.unpack_from("<h", rep, 12)
                accuracy = acc * _Q12
            sample.quats[rid] = Quat(_normalised(qi * _Q14, qj * _Q14, qk * _Q14, qr * _Q14),
                                     rep[2] & 0x03, accuracy)
        elif rid == REPORT_RAW_ACCELEROMETER:
            sample.raw_accel = struct.unpack_from("<hhh", rep, 4)
        i += length
    return sample


def _normalised(x: float, y: float, z: float, w: float) -> tuple[float, float, float, float]:
    # Q14 quantisation leaves |q| a few 1e-4 off 1; consumers (tf2, the EKF)
    # expect a unit quaternion.
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        raise ShtpError("zero quaternion")
    return x / norm, y / norm, z / norm, w / norm


class I2cDev:
    """Minimal i2c-dev handle: one address, raw read/write transactions."""

    def __init__(self, bus: int, address: int, lock_dir: str = "/run/lock") -> None:
        self.path = f"/dev/i2c-{bus}"
        self.address = address
        self.lock_path = os.path.join(lock_dir, f"gripperx-i2c-{bus}-{address:02x}.lock")
        self.fd: int | None = None
        self._lock_fd: int | None = None

    def open(self) -> None:
        self.close()
        # Two readers on one SHTP device each get half of every packet
        # (measured 2026-10-20: two driver instances overlapping for ~10 s
        # produced 267 protocol errors). The lock is per bus + address and
        # advisory - i2cdetect ignores it - but it stops a second driver.
        lock_fd = self._open_lock()
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(lock_fd)
            raise BusBusyError(f"{self.lock_path} is held by another process") from None
        try:
            fd = os.open(self.path, os.O_RDWR)
        except OSError:
            os.close(lock_fd)
            raise
        try:
            fcntl.ioctl(fd, I2C_SLAVE, self.address)
        except OSError:
            os.close(fd)
            os.close(lock_fd)
            raise
        self.fd = fd
        self._lock_fd = lock_fd

    def _open_lock(self) -> int:
        # /run/lock is world-writable and sticky, and the Pi runs with
        # fs.protected_regular=2: there an O_CREAT open of a file another user
        # owns fails with EACCES even when its mode allows writing (e.g. a lock
        # left by `sudo bno085_query`). flock() works on a read-only fd, so an
        # existing file is opened without O_CREAT and read-only as a fallback.
        create_denied = False
        try:
            fd = os.open(self.lock_path, os.O_RDWR | os.O_CREAT | os.O_EXCL, 0o666)
        except FileExistsError:
            pass
        except PermissionError:
            create_denied = True
        else:
            try:
                os.fchmod(fd, 0o666)  # the umask must not lock other users out
            except OSError:
                pass
            return fd
        for flags in (os.O_RDWR, os.O_RDONLY):
            try:
                return os.open(self.lock_path, flags)
            except FileNotFoundError:
                if create_denied:
                    raise LockPermissionError(
                        f"cannot create lock file {self.lock_path} as uid {os.getuid()}") from None
                # Removed between the two opens: created on the next attempt.
                raise OSError(f"{self.lock_path} vanished while opening it") from None
            except PermissionError:
                continue
        try:
            st = os.stat(self.lock_path)
            who = f"owner uid {st.st_uid}, mode {st.st_mode & 0o7777:04o}"
        except OSError:
            who = "not stat-able"
        raise LockPermissionError(
            f"lock file {self.lock_path} ({who}) cannot be opened by uid {os.getuid()}; "
            f"remove it (sudo rm {self.lock_path}) and the driver continues")

    def close(self) -> None:
        for name in ("fd", "_lock_fd"):
            fd = getattr(self, name)
            if fd is not None:
                setattr(self, name, None)
                os.close(fd)

    def read(self, n: int) -> bytes:
        if self.fd is None:
            raise OSError("i2c device not open")
        data = os.read(self.fd, n)
        if len(data) != n:
            raise OSError(f"short read: {len(data)} of {n} bytes")
        return data

    def write(self, data: bytes) -> None:
        if self.fd is None:
            raise OSError("i2c device not open")
        written = os.write(self.fd, data)
        if written != len(data):
            raise OSError(f"short write: {written} of {len(data)} bytes")


class Shtp:
    """SHTP packet I/O on top of a bus object with read(n) / write(bytes)."""

    def __init__(self, bus) -> None:
        self.bus = bus
        self._seq = [0] * (_MAX_CHANNEL + 1)
        self.fragments_dropped = 0

    def send(self, channel: int, payload: bytes) -> None:
        length = len(payload) + 4
        header = bytes([length & 0xFF, (length >> 8) & 0x7F, channel, self._seq[channel]])
        self._seq[channel] = (self._seq[channel] + 1) & 0xFF
        self.bus.write(header + payload)

    def receive(self, max_fragments: int = 8) -> tuple[int, bytes] | None:
        """One whole packet as (channel, payload), or None when nothing is pending.

        A packet whose FIRST header has the continuation bit set is read out,
        dropped and counted in `fragments_dropped`. The re-read header always
        has it set (and its sequence number advanced) on this sensor, so there
        only length and channel are checked; see the module docstring.
        """
        for _ in range(max_fragments + 1):
            length, channel, _, cont = parse_header(self.bus.read(4))
            if length == 0:
                return None
            data = self.bus.read(length)
            length2, channel2, _, _ = parse_header(data[:4])
            if channel2 != channel:
                raise ShtpError(f"header changed between reads (channel {channel} -> {channel2})")
            if length2 != length:
                raise ShtpError(f"header changed between reads (length {length} -> {length2})")
            if cont:
                self.fragments_dropped += 1
                continue
            return channel, data[4:]
        return None
