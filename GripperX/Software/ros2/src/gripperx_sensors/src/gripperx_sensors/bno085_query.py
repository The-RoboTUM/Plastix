#!/usr/bin/env python3
"""Read-only BNO085 query: calibration flags and orientation records. No ROS.

Sends only READ requests (SH-2 Reference Manual 1000-3625 rev 1.9):
  Product ID Request 0xF9 (6.3.1), Get ME Calibration 0xF2/0x07/0x01 (6.4.6.2),
  FRS Read Request 0xF4 (6.3.6) for the orientation records and the dynamic
  calibration record; with --compare-raw also Set Feature 0xFD (6.5.4) for the
  raw and calibrated accelerometer, switched off again afterwards (volatile; if
  this tool is killed before that, the driver switches them off at its start).

DELIBERATELY NO RESET. A non-power-up reset makes the hub persist its RAM
dynamic calibration data (DCD) to flash (BNO08X datasheet 1000-3927 v1.17
section 3.4), i.e. a reset is a write to persistent state. This tool
therefore talks to the sensor as it finds it. Stop the driver first: it holds
the same per-address lock, and two readers corrupt each other's packets.

  ros2 run gripperx_sensors bno085_query [--compare-raw 20]
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

from gripperx_sensors import bno08x_shtp as sh2

RECORDS = [
    (sh2.FRS_SYSTEM_ORIENTATION, "system orientation", "quat"),
    (sh2.FRS_ACCEL_ORIENTATION, "accelerometer orientation", "quat"),
    (sh2.FRS_GYRO_ORIENTATION, "gyroscope orientation", "quat"),
    (sh2.FRS_DYNAMIC_CALIBRATION, "dynamic calibration (DCD, proprietary format)", "raw"),
]


def wait_for(shtp, match, timeout_s):
    """Receive packets until match(channel, payload) returns non-None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pkt = shtp.receive()
        if pkt is None:
            time.sleep(0.005)
            continue
        result = match(*pkt)
        if result is not None:
            return result
    return None


def read_record(shtp, record):
    shtp.send(sh2.CH_CONTROL, sh2.frs_read_request_payload(record))
    words, status = {}, None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        pkt = shtp.receive()
        if pkt is None:
            time.sleep(0.005)
            continue
        ch, payload = pkt
        if ch != sh2.CH_CONTROL:
            continue
        parsed = sh2.parse_frs_read_response(payload)
        if parsed is None or parsed[0] != record:
            continue
        _, status, offset, data = parsed
        for i, w in enumerate(data):
            words[offset + i] = w
        if sh2.frs_read_done(status):
            break
    return status, [words[k] for k in sorted(words)]


def compare_raw(shtp, seconds):
    """Average the raw (0x14) and the calibrated (0x01) accelerometer at rest.

    Set Feature is volatile (features do not survive a reset) and both reports
    are switched off again at the end, so this leaves no persistent trace. The
    raw report is in the MEMS chip's own axes and ADC units, before the static
    and dynamic calibration; comparing the tilt each implies tells whether the
    calibration chain adds the tilt the calibrated report shows.
    """
    for rid in (sh2.REPORT_RAW_ACCELEROMETER, sh2.REPORT_ACCELEROMETER):
        shtp.send(sh2.CH_CONTROL, sh2.set_feature_payload(rid, 20000))
        time.sleep(0.01)
    raw, cal = [], []
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            pkt = shtp.receive()
            if pkt is None:
                time.sleep(0.002)
                continue
            ch, payload = pkt
            if ch not in (sh2.CH_REPORTS, sh2.CH_WAKE_REPORTS):
                continue
            s = sh2.parse_input_packet(payload)
            if s.raw_accel is not None:
                raw.append(s.raw_accel)
            if s.accel is not None:
                cal.append(s.accel)
    finally:
        for rid in (sh2.REPORT_RAW_ACCELEROMETER, sh2.REPORT_ACCELEROMETER):
            shtp.send(sh2.CH_CONTROL, sh2.set_feature_payload(rid, 0))
            time.sleep(0.01)
    for name, rows, unit in (("raw (ADC, chip axes)", raw, "counts"),
                             ("calibrated (sensor axes)", cal, "m/s2")):
        if not rows:
            print(f"{name}: no samples")
            continue
        m = [sum(c) / len(rows) for c in zip(*rows)]
        norm = math.sqrt(sum(v * v for v in m))
        off_vertical = math.degrees(math.atan2(math.hypot(m[0], m[1]), abs(m[2])))
        print(f"{name}: n {len(rows)}, mean " + " ".join(f"{v:+10.3f}" for v in m)
              + f" {unit}, |mean| {norm:.3f}, angle of the mean from the chip z axis "
                f"{off_vertical:.2f} deg (x {math.degrees(math.atan2(m[0], abs(m[2]))):+.2f}, "
                f"y {math.degrees(math.atan2(m[1], abs(m[2]))):+.2f})")


def q30(word):
    return struct.unpack("<i", struct.pack("<I", word))[0] / float(1 << 30)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--address", type=lambda s: int(s, 0), default=0x4A)
    ap.add_argument("--compare-raw", type=float, default=0.0, metavar="SECONDS",
                    help="also average raw vs calibrated accelerometer for SECONDS at rest")
    args = ap.parse_args(argv)

    dev = sh2.I2cDev(args.bus, args.address)
    try:
        dev.open()
    except sh2.BusBusyError as exc:
        print(f"{exc} - stop the bno085_imu driver first", file=sys.stderr)
        return 2
    shtp = sh2.Shtp(dev)
    try:
        shtp.send(sh2.CH_CONTROL, bytes([sh2.CTRL_PRODUCT_ID_REQUEST, 0]))
        ident = wait_for(shtp, lambda ch, p: p if ch == sh2.CH_CONTROL and p[:1] == bytes(
            [sh2.CTRL_PRODUCT_ID_RESPONSE]) else None, 1.0)
        if ident is not None and len(ident) >= 14:
            print(f"product id: part {int.from_bytes(ident[4:8], 'little')}, firmware "
                  f"{ident[2]}.{ident[3]}.{int.from_bytes(ident[12:14], 'little')} "
                  f"build {int.from_bytes(ident[8:12], 'little')}, reset cause {ident[1]}")
        else:
            print("product id: no response")

        shtp.send(sh2.CH_CONTROL, sh2.get_me_calibration_payload(1))
        cal = wait_for(shtp, lambda ch, p: sh2.parse_me_calibration_response(p)
                       if ch == sh2.CH_CONTROL else None, 1.0)
        if cal is None:
            print("ME calibration: no response")
        else:
            print("ME calibration enable flags (1 = enabled): "
                  f"accel {cal['accel']}, gyro {cal['gyro']}, mag {cal['mag']}, "
                  f"planar accel {cal['planar_accel']} (status {cal['status']})")

        for record, name, kind in RECORDS:
            status, words = read_record(shtp, record)
            label = sh2.FRS_READ_STATUS.get(status, f"status {status}")
            if kind == "quat" and len(words) == 4:
                x, y, z, w = (q30(v) for v in words)
                print(f"FRS 0x{record:04X} {name}: x {x:+.6f} y {y:+.6f} z {z:+.6f} "
                      f"w {w:+.6f} ({label})")
            else:
                print(f"FRS 0x{record:04X} {name}: {len(words)} words ({label})"
                      + ("" if not words else ": " + " ".join(f"{v:08X}" for v in words)))
        if args.compare_raw > 0:
            compare_raw(shtp, args.compare_raw)
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
