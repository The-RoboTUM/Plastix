#!/usr/bin/env python3
"""Manual calibration of steering servos (LeRobot-arm style).

Modes:
  scan       — ping every id on the bus; use when the servo ids are unknown
  explore    — torque OFF on all; move by hand and watch live positions
  calibrate  — one servo at a time; records the straight-ahead centre plus the
               OUTWARD and INWARD end position of every wheel as two separate,
               angle-labelled quantities (the steering range is asymmetric:
               100 deg outward / 30 deg inward, measured 2026-08-13)

Usage:
  ros2 run gripperx_control steer_servo_calibrate -- scan
  ros2 run gripperx_control steer_servo_calibrate -- explore --ids 11,14,12,13
  ros2 run gripperx_control steer_servo_calibrate -- calibrate --ids 11,14,12,13 \
      --outward-deg 100 --inward-deg 30

No mode commands a servo position — torque is only ever switched off and back on,
every wheel is moved by hand. Nothing here can make the robot drive.

Typical run after a rewire with unknown ids: `scan` to find which ids are alive,
then `explore --ids <found>` and turn one wheel by hand to see which id belongs to
which wheel, then `calibrate --ids <FL,FR,BL,BR>` to record centre and end stops.

Note on ordering: the id list is ALWAYS in joint order FL, FR, BL, BR — the same
order steer_servo_node maps positionally onto STEER_JOINT_NAMES. The physical
daisy-chain order on the bus is irrelevant to this tool and to steer_servo.yaml.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Sequence

if TYPE_CHECKING:  # pragma: no cover
    from gripperx_control.sts_servo_bus import StsServoBus

# Same udev symlink steer_servo_node uses (pi_env/udev/99-gripperx.rules) — the bare
# /dev/ttyACM0 this used to default to can be any of the three CH343 adapters.
DEFAULT_PORT = "/dev/steering_servo"
DEFAULT_BAUD = 1_000_000
# MUST match steer_servo.yaml protocol_end (0 = SCS, no byte swap). Calibrating at 1
# byte-swaps every position read, so the recorded counts would not be the counts the
# node later writes back. This defaulted to 1 until 2026-08-13.
DEFAULT_PROTOCOL = 0
DEFAULT_NAMES = ("FL", "FR", "BL", "BR")
# Measured on the machine 2026-08-13 (turn each wheel by hand, torque off, watch
# which id moves): in joint order FL, FR, BL, BR the ids are 13, 14, 11, 12. The
# list committed in steer_servo.yaml ([11, 14, 12, 13]) is wrong — see the note
# there before "fixing" it, the stale count arrays are aligned with the old order.
DEFAULT_IDS = (13, 14, 11, 12)
DEFAULT_JOINTS = ("f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer")

# Feetech id space is 0..253 (254 = broadcast). Scanning the full range takes a few
# seconds per unanswered id worst case, so keep the default range tight-ish but
# complete enough to find factory defaults (1) and the previous ids (11..14).
SCAN_ID_MIN = 0
SCAN_ID_MAX = 253

# Mechanical steering range as measured by the user 2026-08-13: every wheel swings
# 100 deg away from the chassis ("outward") and only 30 deg towards it ("inward").
# The tool cannot measure an angle — these values are the LABELS for the two end
# positions the operator holds the wheel at, and they must match
# steering_outward_limit_deg / steering_inward_limit_deg in steer_servo.yaml.
DEFAULT_OUTWARD_DEG = 100.0
DEFAULT_INWARD_DEG = 30.0

# One servo revolution in raw counts (Feetech STS, 0..4095).
COUNTS_PER_REV = 4096

# Joint-angle sign of the OUTWARD direction per wheel (FL, FR, BL, BR).
# MEASURED on the machine 2026-08-13, not derived: all four servos were driven
# 15 deg in the measured outward tick direction and the pose was inspected — the
# wheels lined up tangentially for an in-place spin, matching the (-, +, +, -)
# pattern the kinematics produces for pure rotation. A purely URDF-based reading
# gives (+1, -1, +1, -1) and is WRONG on the front pair; under it the spin pose
# would violate the 30 deg inward limit on three wheels. See the sign-convention
# block in steer_servo_node.py — KEEP IN SYNC with DEFAULT_OUTWARD_SIGN there
# (not imported, because importing the node would pull in rclpy and scservo_sdk).
EXPECTED_OUTWARD_SIGN = (-1, 1, 1, -1)
# Raw-count direction of an OUTWARD turn, measured on the machine 2026-08-13
# (diagonal, not per-side: FL/BR count down, FR/BL count up). Ground truth, used
# to check the recorded endpoints — a mismatch means the wheel/id assignment is
# wrong. KEEP IN SYNC with DEFAULT_OUTWARD_TICK_DIRECTION in steer_servo_node.py.
MEASURED_OUTWARD_TICK_DIRECTION = (-1, 1, 1, -1)
SIDE_OF_JOINT = ("LEFT", "RIGHT", "LEFT", "RIGHT")


def _open_bus(port: str, baud: int, protocol: int) -> "StsServoBus":
    """Import the SDK-backed bus lazily so --help works without scservo_sdk."""
    try:
        from gripperx_control.sts_servo_bus import StsServoBus
    except ModuleNotFoundError as exc:  # feetech-servo-sdk is installed on the Pi only
        raise SystemExit(
            f"{exc}. This tool talks to the servo bus and therefore only runs on the "
            "robot (Pi), where feetech-servo-sdk and /dev/steering_servo exist."
        ) from exc

    bus = StsServoBus(port, baud, protocol)
    bus.open()
    return bus


@dataclass
class ServoCal:
    name: str
    joint: str
    servo_id: int
    center: int  # straight-ahead position recorded at start (not a midpoint)
    counts_outward: int  # raw position held at the outward limit
    counts_inward: int  # raw position held at the inward limit
    outward_sign: int  # +1 if outward is a POSITIVE joint angle for this wheel

    def counts_per_deg(self, outward_deg: float, inward_deg: float) -> tuple[float, float]:
        return (
            abs(self.counts_outward - self.center) / outward_deg,
            abs(self.counts_inward - self.center) / inward_deg,
        )


def _wait_enter(prompt: str) -> None:
    input(prompt)


def _stdin_ready() -> bool:
    return bool(select.select([sys.stdin], [], [], 0.0)[0])


def _drain_stdin() -> None:
    """Discard pending input.

    Every ENTER/key press in this tool is consumed by whoever asked for it; a
    leftover newline would otherwise make the NEXT `_capture_endpoint()` return
    immediately and record a position nobody meant to capture.
    """
    while _stdin_ready():
        sys.stdin.read(1)


def _read_position_safe(bus: StsServoBus, servo_id: int) -> int | None:
    try:
        return bus.read_position(servo_id)
    except RuntimeError:
        return None


def _set_all_torque(bus: StsServoBus, ids: Sequence[int], enable: bool) -> None:
    for servo_id in ids:
        bus.enable_torque(servo_id, enable)


def run_scan(bus: StsServoBus, id_min: int, id_max: int) -> List[int]:
    """Ping every id in the range and report the ones that answer.

    Read-only: no torque change, no position write. This is the entry point when the
    ids are unknown (e.g. after a rewire or a servo swap) — every other mode needs
    the ids up front and aborts on the first one that does not answer.
    """
    print(f"\n=== SCAN MODE (ids {id_min}..{id_max}) ===")
    print("Read-only — nothing is moved, no torque is changed.\n")

    found: List[int] = []
    for servo_id in range(id_min, id_max + 1):
        print(f"\r  probing id={servo_id:3d} ...", end="", flush=True)
        if not bus.ping(servo_id):
            continue
        position = _read_position_safe(bus, servo_id)
        found.append(servo_id)
        print(
            f"\r  FOUND id={servo_id:3d}  position={position if position is not None else '?'}"
            f"{' ' * 20}"
        )

    print(f"\r{' ' * 40}\r", end="")

    if not found:
        print("No servo answered. Check port, baud rate, protocol_end and servo power.")
        return found

    id_list = ",".join(str(i) for i in found)
    print(f"\n{len(found)} servo(s) found: {id_list}")
    if len(found) != len(DEFAULT_NAMES):
        print(
            f"NOTE: expected {len(DEFAULT_NAMES)} steering servos — check the daisy chain "
            "before continuing."
        )

    print("\nNext step — map ids to wheels WITHOUT moving anything:")
    print(f"  ros2 run gripperx_control steer_servo_calibrate -- explore --ids {id_list}")
    print("Turn one wheel by hand; the column whose value changes is that wheel.")
    print("Then re-run with --ids in joint order FL,FR,BL,BR:")
    print("  ros2 run gripperx_control steer_servo_calibrate -- calibrate --ids <FL>,<FR>,<BL>,<BR>")
    return found


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


def _capture_endpoint(bus: StsServoBus, servo_id: int) -> tuple[int, int, int, int]:
    """Live-display one servo's position; ENTER captures the CURRENT reading.

    Returns (captured, seen_min, seen_max, samples). The captured value is the
    last reading before ENTER — deliberately not min/max over a sweep: the two
    endpoints have to correspond to the two KNOWN angles the wheel is held at,
    and a sweep cannot tell which extreme belonged to which direction.
    """
    seen_min = 4095
    seen_max = 0
    samples = 0
    last = None

    _drain_stdin()
    while True:
        if _stdin_ready():
            sys.stdin.readline()
            break

        pos = _read_position_safe(bus, servo_id)
        if pos is not None:
            last = pos
            seen_min = min(seen_min, pos)
            seen_max = max(seen_max, pos)
            samples += 1
            print(
                f"\r  ticks={pos:4d}  (seen {seen_min:4d}..{seen_max:4d})  "
                f"samples={samples}   (ENTER=capture)",
                end="",
                flush=True,
            )
        time.sleep(0.06)

    if samples == 0 or last is None:
        raise RuntimeError("No position was read")

    print()
    return last, seen_min, seen_max, samples


def _ask_choice(prompt: str, options: Sequence[str]) -> str:
    """Single-key question inside the cbreak terminal set up by main()."""
    valid = tuple(o.lower() for o in options)
    _drain_stdin()
    while True:
        print(f"\r  {prompt} [{'/'.join(valid)}] ", end="", flush=True)
        while not _stdin_ready():
            time.sleep(0.05)
        key = sys.stdin.read(1).lower()
        if key in valid:
            print(key)
            _drain_stdin()
            return key


def run_calibrate(
    bus: StsServoBus,
    names: Sequence[str],
    joints: Sequence[str],
    ids: Sequence[int],
    outward_deg: float,
    inward_deg: float,
) -> List[ServoCal]:
    print("\n=== MANUAL CALIBRATION (one servo at a time) ===")
    print(f"Recorded per wheel: centre (0°), outward end ({outward_deg:.0f}°), "
          f"inward end ({inward_deg:.0f}°).")
    print("For each wheel:")
    print("  1. Torque OFF only on that servo")
    print("  2. Hold the wheel at its OUTWARD limit (tyre front away from the body), ENTER")
    print("  3. Confirm that this was the direction the tool named for that wheel")
    print("  4. Hold the wheel at its INWARD limit (tyre front towards the body), ENTER")
    print("Both end positions must be the mechanical limits the angles above refer")
    print("to — do NOT sweep to some arbitrary 'safe' point and do not force the")
    print("wheel against the stop. The centre (0°) is the straight-ahead position")
    print("from the initial step, never a midpoint of the two ends.\n")

    # Torque off on all four first: with torque on, the servos fight the hand that
    # is supposed to place them straight ahead. No position is commanded here.
    _set_all_torque(bus, ids, False)
    _wait_enter("Torque OFF. Place ALL wheels in the straight-ahead/neutral position, ENTER...")

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
        print("  Torque OFF on this servo only.")

        # The joint-angle sign of "outward" follows from the wheel's side: +angle
        # points a wheel towards the robot's LEFT (+y) on ALL four joints (URDF
        # axis 0 0 1 about base_link, IK atan2(vy, vx); see steer_servo_node.py),
        # and outward means the tyre front swings away from the body. The operator
        # only confirms the wheel really was held that way — 'n' flips the sign for
        # this wheel and is reported, so an operator error cannot pass silently.
        expected_sign = EXPECTED_OUTWARD_SIGN[index] if index < len(EXPECTED_OUTWARD_SIGN) else 1
        side = SIDE_OF_JOINT[index] if index < len(SIDE_OF_JOINT) else "?"
        towards = "LEFT (+y)" if expected_sign > 0 else "RIGHT (-y)"
        print(f"  {name} is a {side} wheel, so OUTWARD means: the wheel edge that faces")
        print(f"  FORWARD at straight-ahead points towards the robot's {towards}.")

        print(f"  OUTWARD ({outward_deg:.0f}°, away from the body) — hold it there:")
        counts_outward, out_lo, out_hi, _ = _capture_endpoint(bus, servo_id)
        key = _ask_choice("Was that the direction described above?", ("y", "n"))
        outward_sign = expected_sign if key == "y" else -expected_sign

        print(f"  INWARD ({inward_deg:.0f}°, towards the body) — hold it there:")
        counts_inward, in_lo, in_hi, _ = _capture_endpoint(bus, servo_id)

        bus.enable_torque(servo_id, True)

        cal = ServoCal(
            name=name,
            joint=joint,
            servo_id=servo_id,
            center=neutral_counts[index],
            counts_outward=counts_outward,
            counts_inward=counts_inward,
            outward_sign=outward_sign,
        )
        results.append(cal)

        out_per_deg, in_per_deg = cal.counts_per_deg(outward_deg, inward_deg)
        print(
            f"  OK {name}: center={cal.center} outward={cal.counts_outward} "
            f"(jitter {out_lo}..{out_hi}) inward={cal.counts_inward} "
            f"(jitter {in_lo}..{in_hi}) outward_sign={outward_sign:+d}"
        )
        print(
            f"     counts/deg: outward {out_per_deg:.1f}, inward {in_per_deg:.1f} "
            f"(theoretical 1:1 linkage: {COUNTS_PER_REV / 360.0:.1f})"
        )
        _warn_endpoint_plausibility(cal, index, out_per_deg, in_per_deg)

    return results


def _warn_endpoint_plausibility(
    cal: ServoCal,
    index: int,
    out_per_deg: float,
    in_per_deg: float,
) -> None:
    out_delta = cal.counts_outward - cal.center
    in_delta = cal.counts_inward - cal.center

    if index < len(MEASURED_OUTWARD_TICK_DIRECTION) and out_delta != 0:
        expected_dir = MEASURED_OUTWARD_TICK_DIRECTION[index]
        if (1 if out_delta > 0 else -1) != expected_dir:
            print(
                f"     WARNING: turning {cal.name} outward was measured to make the count "
                f"{'RISE' if expected_dir > 0 else 'FALL'} (2026-08-13), but the recorded "
                f"outward end ({cal.counts_outward}) is on the other side of the centre "
                f"({cal.center}). Most likely id={cal.servo_id} is not {cal.name} — check "
                "--ids against joint order FL,FR,BL,BR before pasting anything."
            )

    if out_delta * in_delta >= 0:
        print(
            f"     WARNING: outward and inward end are on the SAME side of the centre "
            f"({cal.center}) — one of the two was probably recorded in the wrong direction."
        )
    for label, delta in (("outward", out_delta), ("inward", in_delta)):
        if abs(delta) > COUNTS_PER_REV // 2:
            print(
                f"     ERROR: {label} end is {abs(delta)} counts from the centre — more than "
                "half a revolution, the raw position wrapped at 0/4095. Re-centre the servo "
                "horn mechanically and repeat, otherwise the conversion is meaningless."
            )
    if min(out_per_deg, in_per_deg) > 0.0 and max(out_per_deg, in_per_deg) > 1.25 * min(
        out_per_deg, in_per_deg
    ):
        print(
            "     WARNING: counts per degree differ by >25% between the two directions — "
            "with a rigid 1:1 linkage they should match; check the angle labels."
        )
    if index < len(EXPECTED_OUTWARD_SIGN) and cal.outward_sign != EXPECTED_OUTWARD_SIGN[index]:
        print(
            f"     NOTE: outward_sign={cal.outward_sign:+d} differs from the value that "
            f"follows from the URDF joint convention ({EXPECTED_OUTWARD_SIGN[index]:+d}) for "
            "this wheel, because the direction was not confirmed. Your answer is used, but "
            "this row now describes the wheel turning the other way — re-run it unless you "
            "are sure."
        )


def _print_yaml(results: Sequence[ServoCal], outward_deg: float, inward_deg: float) -> None:
    ids = [r.servo_id for r in results]
    centers = [r.center for r in results]
    outward = [r.counts_outward for r in results]
    inward = [r.counts_inward for r in results]
    signs = [r.outward_sign for r in results]

    print("\n" + "=" * 72)
    print("Copy this into steer_servo.yaml (replaces the stale block):")
    print("=" * 72)
    print("    servo_ids: [%s]" % ", ".join(str(i) for i in ids))
    print("    center_counts: [%s]" % ", ".join(str(c) for c in centers))
    print("    counts_outward_limit: [%s]" % ", ".join(str(c) for c in outward))
    print("    counts_inward_limit: [%s]" % ", ".join(str(c) for c in inward))
    print("    steering_outward_limit_deg: %.1f" % outward_deg)
    print("    steering_inward_limit_deg: %.1f" % inward_deg)
    print("    steering_outward_sign: [%s]" % ", ".join("%+d" % s for s in signs))
    print("    steering_outward_tick_direction: [%s]"
          % ", ".join("%+d" % (1 if r.counts_outward > r.center else -1) for r in results))
    print()
    for r in results:
        out_per_deg, in_per_deg = r.counts_per_deg(outward_deg, inward_deg)
        print(
            f"  # {r.name} id={r.servo_id}: center={r.center} "
            f"outward({outward_deg:.0f}°)={r.counts_outward} "
            f"inward({inward_deg:.0f}°)={r.counts_inward} "
            f"sign={r.outward_sign:+d} "
            f"[{out_per_deg:.1f}/{in_per_deg:.1f} counts per deg]"
        )
    print()
    print("  # The counts above belong to the stated OUTWARD/INWARD angles, not to ±90°.")
    print("  # Do NOT paste them into the legacy symmetric keys counts_plus_90/")
    print("  # counts_minus_90: with a single limit the outward end stop would be reached")
    print("  # at the inward angle already — that is a real over-steer, not just a")
    print("  # scaling error. Leave the legacy keys alone; they are ignored as soon as")
    print("  # the four per-direction keys above are present.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual calibration of steering servos")
    parser.add_argument(
        "mode",
        nargs="?",
        choices=("scan", "explore", "calibrate"),
        default="explore",
        help="scan=find ids on the bus; explore=view positions; "
        "calibrate=record centre + outward/inward end position per wheel",
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--protocol", type=int, default=DEFAULT_PROTOCOL)
    parser.add_argument("--ids", default=",".join(str(i) for i in DEFAULT_IDS))
    parser.add_argument("--names", default=",".join(DEFAULT_NAMES))
    parser.add_argument(
        "--outward-deg",
        type=float,
        default=DEFAULT_OUTWARD_DEG,
        help="angle the OUTWARD end position corresponds to (calibrate mode)",
    )
    parser.add_argument(
        "--inward-deg",
        type=float,
        default=DEFAULT_INWARD_DEG,
        help="angle the INWARD end position corresponds to (calibrate mode)",
    )
    parser.add_argument("--id-min", type=int, default=SCAN_ID_MIN, help="scan mode only")
    parser.add_argument("--id-max", type=int, default=SCAN_ID_MAX, help="scan mode only")
    args = parser.parse_args()

    if args.outward_deg <= 0.0 or args.inward_deg <= 0.0:
        raise SystemExit("--outward-deg and --inward-deg are magnitudes and must be > 0")

    bus = _open_bus(args.port, args.baud, args.protocol)
    print(f"Port {args.port} @ {args.baud}  protocol_end={args.protocol}")

    # scan runs before any id/name validation — not knowing the ids is exactly why
    # this mode exists. It also needs no raw terminal, so it returns straight out.
    if args.mode == "scan":
        try:
            run_scan(bus, args.id_min, args.id_max)
        finally:
            bus.close()
        return

    ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
    names = [x.strip() for x in args.names.split(",") if x.strip()]
    joints = list(DEFAULT_JOINTS[: len(ids)])

    if len(ids) != len(names):
        raise SystemExit("ids and names must have the same count")

    for name, servo_id in zip(names, ids):
        if not bus.ping(servo_id):
            raise SystemExit(
                f"Servo id={servo_id} ({name}) not responding — "
                "run 'scan' mode to list the ids that are actually on the bus"
            )

    old_tty = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        if args.mode == "explore":
            run_explore(bus, names, ids)
        else:
            results = run_calibrate(
                bus, names, joints, ids, args.outward_deg, args.inward_deg
            )
            _print_yaml(results, args.outward_deg, args.inward_deg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        try:
            _set_all_torque(bus, ids, True)
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    main()
