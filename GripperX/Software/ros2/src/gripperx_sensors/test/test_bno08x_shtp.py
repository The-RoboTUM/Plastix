"""SHTP framing and SH-2 report parsing, without a sensor.

Byte strings are built from the SH-2 layouts (little-endian int16, Q points
8 / 9 / 14), so a wrong Q point or offset in the parser shows up as a wrong
physical value here.
"""

import math
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from gripperx_sensors import bno08x_shtp as sh2  # noqa: E402


def report(rid, status, *values, fmt="<hhh"):
    return bytes([rid, 0, status, 0]) + struct.pack(fmt, *values)


def packet(channel, payload, seq=0):
    n = len(payload) + 4
    return bytes([n & 0xFF, n >> 8, channel, seq]) + payload


class FakeBus:
    def __init__(self, reads):
        self.reads = list(reads)
        self.written = []

    def read(self, n):
        data = self.reads.pop(0)
        assert len(data) >= n
        return data[:n]

    def write(self, data):
        self.written.append(bytes(data))


def test_header_fields_and_continuation():
    assert sh2.parse_header(bytes([0x14, 0x80, 3, 7])) == (20, 3, 7, True)
    assert sh2.parse_header(bytes([0, 0, 0, 0]))[0] == 0


@pytest.mark.parametrize("bad", [b"\xff\xff\xff\xff", bytes([10, 0, 9, 0]), bytes([2, 0, 3, 0]), b"\x01"])
def test_impossible_headers_are_rejected(bad):
    with pytest.raises(sh2.ShtpError):
        sh2.parse_header(bad)


def test_mixed_input_packet_physical_values():
    # gravity on the sensor's -z (mounted upside down), 0.5 rad/s about z,
    # identity-like quaternion rotated 180 deg about x: (1, 0, 0, 0).
    payload = (bytes([0xFB, 0, 0, 0, 0])
               + report(0x01, 3, 0, 0, round(-9.81 * 256))
               + report(0x02, 2, 0, 0, round(0.5 * 512))
               + report(0x08, 1, 1 << 14, 0, 0, 0, fmt="<hhhh"))
    s = sh2.parse_input_packet(payload)
    assert s.accel == pytest.approx((0.0, 0.0, -9.81), abs=1 / 256)
    assert s.gyro == pytest.approx((0.0, 0.0, 0.5), abs=1 / 512)
    assert list(s.quats) == [0x08]
    assert s.quats[0x08].xyzw == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert (s.accel_status, s.gyro_status, s.quats[0x08].status) == (3, 2, 1)
    assert s.unknown_report_ids == []


def test_rotation_vector_carries_accuracy_and_is_normalised():
    rv = bytes([0x05, 0, 3, 0]) + struct.pack("<hhhhh", 0, 0, 11585, 11585, round(0.1 * 4096))
    s = sh2.parse_input_packet(rv)
    q = s.quats[0x05]
    assert math.isclose(sum(v * v for v in q.xyzw), 1.0, rel_tol=1e-12)
    assert q.accuracy_rad == pytest.approx(0.1, abs=1 / 4096)


def test_unknown_report_stops_parsing_without_guessing():
    payload = report(0x01, 0, 1, 2, 3) + bytes([0x77, 1, 2, 3]) + report(0x02, 0, 1, 2, 3)
    s = sh2.parse_input_packet(payload)
    assert s.accel is not None and s.gyro is None
    assert s.unknown_report_ids == [0x77]


def test_truncated_report_is_not_read_past_the_end():
    s = sh2.parse_input_packet(report(0x02, 0, 1, 2, 3)[:7])
    assert s.gyro is None and s.unknown_report_ids == [0x02]


def test_set_feature_layout():
    # SH-2 rev 1.9 Figure 78, byte by byte: id, report, flags, sensitivity
    # LSB/MSB, interval (LE u32), batch interval (u32), sensor config (u32).
    assert sh2.set_feature_payload(0x08, 20000) == bytes(
        [0xFD, 0x08, 0, 0, 0, 0x20, 0x4E, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    assert sh2.set_feature_payload(0x14, 0) == bytes([0xFD, 0x14] + [0] * 15)


def test_timestamp_records_before_a_report_are_skipped():
    payload = (bytes([0xFB, 1, 2, 3, 4]) + bytes([0xFA, 5, 6, 7, 8])
               + report(0x02, 3, 0, 0, round(1.0 * 512)))
    s = sh2.parse_input_packet(payload)
    assert s.gyro == pytest.approx((0.0, 0.0, 1.0)) and s.gyro_status == 3
    assert s.unknown_report_ids == []


def test_two_orientation_reports_in_one_packet_are_kept_apart():
    payload = (report(0x05, 3, 1 << 14, 0, 0, 0, 0, fmt="<hhhhh")
               + report(0x08, 2, 0, 0, 0, 1 << 14, fmt="<hhhh"))
    s = sh2.parse_input_packet(payload)
    assert s.quats[0x05].xyzw == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert s.quats[0x08].xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_send_numbers_sequence_per_channel():
    bus = FakeBus([])
    t = sh2.Shtp(bus)
    t.send(2, b"\xf9\x00")
    t.send(2, b"\xf9\x00")
    t.send(1, b"\x01")
    assert bus.written[0][:4] == bytes([6, 0, 2, 0])
    assert bus.written[1][3] == 1
    assert bus.written[2][:4] == bytes([5, 0, 1, 0])


def test_receive_two_phase_read():
    pkt = packet(3, report(0x02, 0, 0, 0, 0))
    t = sh2.Shtp(FakeBus([pkt[:4], pkt]))
    assert t.receive() == (3, pkt[4:])
    assert sh2.Shtp(FakeBus([bytes(4)])).receive() is None


# Measured on the robot 2026-10-20 (raw probe on the real BNO085, header-only
# 4-byte read then full re-read): (first header, re-read header). The re-read
# always has bit 15 set and the sequence number advanced by one.
MEASURED_HEADER_PAIRS = [
    (bytes([0x13, 0x00, 0x03, 0x1C]), bytes([0x13, 0x80, 0x03, 0x1D])),  # len 19
    (bytes([0xC7, 0x00, 0x03, 0x1E]), bytes([0xC7, 0x80, 0x03, 0x1F])),  # len 199
    (bytes([0x49, 0x00, 0x03, 0x28]), bytes([0x49, 0x80, 0x03, 0x29])),  # len 73
]


@pytest.mark.parametrize("first,reread", MEASURED_HEADER_PAIRS)
def test_measured_reread_with_continuation_bit_is_accepted(first, reread):
    length = first[0] | (first[1] << 8)
    payload = bytes([0xFB]) + bytes(length - 5)
    t = sh2.Shtp(FakeBus([first, reread + payload]))
    assert t.receive() == (3, payload)
    assert t.fragments_dropped == 0


def test_receive_reads_out_and_drops_continuation_fragments():
    # bit 15 in the FIRST header: the tail of a message never started here
    frag = bytearray(packet(3, report(0x02, 0, 0, 0, 7)))
    frag[1] |= 0x80
    whole = packet(3, report(0x02, 0, 0, 0, 9))
    bus = FakeBus([bytes(frag[:4]), bytes(frag), whole[:4], whole])
    t = sh2.Shtp(bus)
    assert t.receive() == (3, whole[4:])
    assert t.fragments_dropped == 1
    assert bus.reads == []  # the fragment was read out, not left in the device


def test_reread_must_still_match_in_length():
    first, reread = MEASURED_HEADER_PAIRS[0]
    bad = bytes([0x14, 0x80, 0x03, 0x1D])
    with pytest.raises(sh2.ShtpError):
        sh2.Shtp(FakeBus([first, bad + bytes(16)])).receive()


def test_receive_raises_on_ffff_header():
    t = sh2.Shtp(FakeBus([b"\xff\xff\xff\xff"]))
    with pytest.raises(sh2.ShtpError):
        t.receive()


def test_receive_rejects_header_that_changed_between_reads():
    a = packet(3, bytes(10))
    b = packet(2, bytes(10))
    with pytest.raises(sh2.ShtpError):
        sh2.Shtp(FakeBus([a[:4], b])).receive()


def test_second_opener_of_the_same_device_is_refused(tmp_path, monkeypatch):
    # The i2c-dev node is replaced by a plain file; only the lock is under test.
    dev = tmp_path / "i2c-1"
    dev.write_bytes(b"")
    monkeypatch.setattr(sh2.fcntl, "ioctl", lambda *a: 0)
    a = sh2.I2cDev(1, 0x4A, lock_dir=str(tmp_path))
    b = sh2.I2cDev(1, 0x4A, lock_dir=str(tmp_path))
    other = sh2.I2cDev(1, 0x4B, lock_dir=str(tmp_path))
    a.path = b.path = other.path = str(dev)
    a.open()
    with pytest.raises(sh2.BusBusyError):
        b.open()
    other.open()  # a different address on the same bus is not blocked
    a.close()
    b.open()  # released on close
    b.close()
    other.close()


def test_lock_file_of_another_user_is_opened_read_only(tmp_path, monkeypatch):
    # fs.protected_regular=2 makes O_CREAT opens of someone else's file in a
    # sticky directory fail with EACCES; simulated by refusing every open that
    # carries O_CREAT or O_RDWR on an existing lock file.
    dev = tmp_path / "i2c-1"
    dev.write_bytes(b"")
    monkeypatch.setattr(sh2.fcntl, "ioctl", lambda *a: 0)
    a = sh2.I2cDev(1, 0x4A, lock_dir=str(tmp_path))
    a.path = str(dev)
    open(a.lock_path, "w").close()
    real_open = os.open
    seen = []

    def guarded(path, flags, *mode):
        if path == a.lock_path:
            seen.append(flags)
            if flags & os.O_CREAT and not flags & os.O_EXCL or flags & os.O_RDWR:
                raise PermissionError(13, "Permission denied", path)
        return real_open(path, flags, *mode)

    monkeypatch.setattr(sh2.os, "open", guarded)
    a.open()
    assert seen[-1] == os.O_RDONLY
    b = sh2.I2cDev(1, 0x4A, lock_dir=str(tmp_path))
    b.path = str(dev)
    with pytest.raises(sh2.BusBusyError):  # a read-only fd still excludes
        b.open()
    a.close()


def test_unopenable_lock_file_is_a_clear_permission_error(tmp_path, monkeypatch):
    a = sh2.I2cDev(1, 0x4A, lock_dir=str(tmp_path))
    open(a.lock_path, "w").close()
    real_open = os.open

    def deny(path, flags, *mode):
        if path == a.lock_path and not flags & os.O_EXCL:
            raise PermissionError(13, "Permission denied", path)
        return real_open(path, flags, *mode)

    monkeypatch.setattr(sh2.os, "open", deny)
    with pytest.raises(sh2.LockPermissionError, match="sudo rm"):
        a.open()


def test_new_lock_file_is_world_accessible_despite_umask(tmp_path, monkeypatch):
    dev = tmp_path / "i2c-1"
    dev.write_bytes(b"")
    monkeypatch.setattr(sh2.fcntl, "ioctl", lambda *a: 0)
    a = sh2.I2cDev(1, 0x4A, lock_dir=str(tmp_path))
    a.path = str(dev)
    old = os.umask(0o077)
    try:
        a.open()
    finally:
        os.umask(old)
    assert os.stat(a.lock_path).st_mode & 0o777 == 0o666
    a.close()


def test_get_me_calibration_is_the_read_subcommand():
    p = sh2.get_me_calibration_payload(5)
    # 0xF2 command request, seq, command 0x07, P0..P2 zero, P3 = 0x01 (get), rest zero
    assert p == bytes([0xF2, 5, 0x07, 0, 0, 0, 0x01, 0, 0, 0, 0, 0])


def test_me_calibration_response_fields():
    resp = bytes([0xF1, 0, 0x07, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0])
    assert sh2.parse_me_calibration_response(resp) == {
        "status": 0, "accel": 1, "gyro": 0, "mag": 1, "planar_accel": 0}
    assert sh2.parse_me_calibration_response(bytes([0xF1, 0, 0x84] + [0] * 13)) is None


def test_frs_read_request_and_response():
    # rev 1.9 Figure 38: bytes 1-3 and 6-7 reserved (zero), 4-5 the FRS type.
    assert sh2.frs_read_request_payload(0x2D3E) == bytes([0xF4, 0, 0, 0, 0x3E, 0x2D, 0, 0])
    resp = (bytes([0xF3, (2 << 4) | 3, 2, 0]) + struct.pack("<II", 1 << 30, 7)
            + bytes([0x3E, 0x2D, 0, 0]))
    record, status, offset, words = sh2.parse_frs_read_response(resp)
    assert (record, status, offset, words) == (0x2D3E, 3, 2, [1 << 30, 7])
    assert sh2.frs_read_done(status) and not sh2.frs_read_done(0)
