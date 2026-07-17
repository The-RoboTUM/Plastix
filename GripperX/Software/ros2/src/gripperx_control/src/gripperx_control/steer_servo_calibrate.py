#!/usr/bin/env python3
"""Manual calibration of steering servos (LeRobot-arm style).

Modes:
  explore    — torque OFF on all; move by hand and watch live positions
  calibrate  — one servo at a time; records neutral at start and min/max per wheel

Usage:
  ros2 run gripperx_control steer_servo_calibrate -- explore
  ros2 run gripperx_control steer_servo_calibrate -- calibrate
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import List, Sequence

from gripperx_control.sts_servo_bus import StsServoBus

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
DEFAULT_PROTOCOL = 1
DEFAULT_NAMES = ("FL", "FR", "BL", "BR")
DEFAULT_IDS = (11, 14, 12, 13)
DEFAULT_JOINTS = ("f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer")


@dataclass
class ServoCal:
    name: str
    joint: str
    servo_id: int
    pos_min: int
    pos_max: int
    center: int  # straight-ahead position recorded at start (not the min/max midpoint)

    @property
    def span(self) -> int:
        return self.pos_max - self.pos_min


def _wait_enter(prompt: str) -> None:
    input(prompt)


def _stdin_ready() -> bool:
    return bool(select.select([sys.stdin], [], [], 0.0)[0])


def _read_position_safe(bus: StsServoBus, servo_id: int) -> int | None:
    try:
        return bus.read_position(servo_id)
    except RuntimeError:
        return None


def _set_all_torque(bus: StsServoBus, ids: Sequence[int], enable: bool) -> None:
    for servo_id in ids:
        bus.enable_torque(servo_id, enable)


def run_explore(bus: StsServoBus, names: Sequence[str], ids: Sequence[int]) -> None:
    print("\n=== EXPLORE MODE ===")
    print("Torque DISABLED on all servos.")
    print("Move the wheels by hand. Press ENTER to finish.\n")

    _set_all_torque(bus, ids, False)

    labels = [f"{n}(id={i})" for n, i in zip(names, ids)]
    try:
        while True:
            if _stdin_ready():
                sys.stdin.readline()
                break

            parts = []
            for label, servo_id in zip(labels, ids):
                pos = _read_position_safe(bus, servo_id)
                parts.append(f"{label}={pos if pos is not None else '?'}")

            line = "  |  ".join(parts)
            print(f"\r{line:<90}", end="", flush=True)
            time.sleep(0.08)
    finally:
        print("\n\nExplore finished. Re-enabling torque...")
        _set_all_torque(bus, ids, True)


def _record_range(bus: StsServoBus, servo_id: int) -> tuple[int, int, int]:
    pos_min = 4095
    pos_max = 0
    samples = 0

    while True:
        if _stdin_ready():
            sys.stdin.readline()
            break

        pos = _read_position_safe(bus, servo_id)
        if pos is not None:
            pos_min = min(pos_min, pos)
            pos_max = max(pos_max, pos)
            samples += 1
            midpoint = int(round(0.5 * (pos_min + pos_max)))
            print(
                f"\r  ticks={pos:4d}  min={pos_min:4d}  max={pos_max:4d}  "
                f"mid={midpoint:4d}  samples={samples}   (ENTER=next)",
                end="",
                flush=True,
            )
        time.sleep(0.06)

    if samples == 0:
        raise RuntimeError("No position was read")

    print()
    return pos_min, pos_max, samples


def run_calibrate(bus: StsServoBus, names: Sequence[str], joints: Sequence[str], ids: Sequence[int]) -> List[ServoCal]:
    print("\n=== MANUAL CALIBRATION (one servo at a time) ===")
    print("For each wheel:")
    print("  1. Torque OFF only on that servo")
    print("  2. Move it by hand through the safe range (without hitting the chassis)")
    print("  3. Press ENTER to save min/max")
    print("The center (0°) is the straight-ahead position from the initial step, not the midpoint of the range.\n")

    _wait_enter("Place ALL wheels in the straight-ahead/neutral position and press ENTER...")

    neutral_counts: List[int] = []
    print("\nStraight-ahead position recorded:")
    for name, servo_id in zip(names, ids):
        pos = _read_position_safe(bus, servo_id)
        if pos is None:
            raise RuntimeError(f"Could not read neutral position for {name} (id={servo_id})")
        neutral_counts.append(pos)
        print(f"  {name} id={servo_id}: neutral={pos}")

    results: List[ServoCal] = []

    for index, (name, joint, servo_id) in enumerate(zip(names, joints, ids)):
        print(f"\n--- [{index + 1}/{len(ids)}] {name}  joint={joint}  id={servo_id} ---")
        _set_all_torque(bus, ids, True)
        bus.enable_torque(servo_id, False)
        print("  Torque OFF. Move ONLY this wheel through its entire safe range.")

        pos_min, pos_max, _ = _record_range(bus, servo_id)
        bus.enable_torque(servo_id, True)

        cal = ServoCal(
            name=name,
            joint=joint,
            servo_id=servo_id,
            pos_min=pos_min,
            pos_max=pos_max,
            center=neutral_counts[index],
        )
        results.append(cal)
        midpoint = int(round(0.5 * (pos_min + pos_max)))
        print(
            f"  OK {name}: min={cal.pos_min} max={cal.pos_max} "
            f"center={cal.center} (mid={midpoint}) span={cal.span}"
        )

    return results


def _print_yaml(results: Sequence[ServoCal], limit_deg: float) -> None:
    centers = [r.center for r in results]
    plus = [r.pos_max for r in results]
    minus = [r.pos_min for r in results]
    ids = [r.servo_id for r in results]

    print("\n" + "=" * 60)
    print("Copy this into steer_servo.yaml:")
    print("=" * 60)
    print("    servo_ids: [%s]" % ", ".join(str(i) for i in ids))
    print("    center_counts: [%s]" % ", ".join(str(c) for c in centers))
    print("    counts_plus_90: [%s]   # recorded max" % ", ".join(str(c) for c in plus))
    print("    counts_minus_90: [%s]  # recorded min" % ", ".join(str(c) for c in minus))
    print("    steering_angle_limit_deg: %.1f" % limit_deg)
    print()
    for r in results:
        print(
            f"  # {r.name} id={r.servo_id}: center={r.center} "
            f"+lim={r.pos_max} -lim={r.pos_min}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual calibration of steering servos")
    parser.add_argument(
        "mode",
        nargs="?",
        choices=("explore", "calibrate"),
        default="explore",
        help="explore=view positions; calibrate=record ranges",
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--protocol", type=int, default=DEFAULT_PROTOCOL)
    parser.add_argument("--ids", default=",".join(str(i) for i in DEFAULT_IDS))
    parser.add_argument("--names", default=",".join(DEFAULT_NAMES))
    parser.add_argument("--limit-deg", type=float, default=45.0)
    args = parser.parse_args()

    ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
    names = [x.strip() for x in args.names.split(",") if x.strip()]
    joints = list(DEFAULT_JOINTS[: len(ids)])

    if len(ids) != len(names):
        raise SystemExit("ids and names must have the same count")

    bus = StsServoBus(args.port, args.baud, args.protocol)
    bus.open()

    print(f"Port {args.port} @ {args.baud}  protocol_end={args.protocol}")
    for name, servo_id in zip(names, ids):
        if not bus.ping(servo_id):
            raise SystemExit(f"Servo id={servo_id} ({name}) not responding")

    old_tty = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        if args.mode == "explore":
            run_explore(bus, names, ids)
        else:
            results = run_calibrate(bus, names, joints, ids)
            _print_yaml(results, args.limit_deg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        try:
            _set_all_torque(bus, ids, True)
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    main()
