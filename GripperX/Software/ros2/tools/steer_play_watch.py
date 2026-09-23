#!/usr/bin/env python3
"""Read-only live view of the four steering servo counts. COMMANDS NOTHING.

NOT AN SR-1 (motor-enable process rule) MOTION TOOL, and structurally incapable of
becoming one: it calls `read_position` and nothing else. It never writes a position,
never touches torque, and never opens a ROS node. It cannot move the robot, and it
does not change the torque state it finds — which is the point, see below.

WHY IT EXISTS — the "where does the play sit?" column of HWA-12 (the static play
measurement owed since 2026-08-21). The play measurement needs the servos TORQUED AND
HOLDING while the wheel is wiggled by hand at the tyre. Neither existing mode of
`steer_servo_calibrate` provides a readout in that state: `explore` switches torque OFF
(so it measures the servo turning, not the play), and `calibrate` prints its last line
after re-enabling torque and then exits. This fills exactly that gap.

WHAT THE READING MEANS while a wheel is wiggled against a holding servo:

  counts DO NOT move  ->  the play is DOWNSTREAM of the servo encoder: coupling, horn
                          or steering axis. That is the part the ASA rework addressed,
                          and the part a better coupling can remove.
  counts DO move      ->  part of the play is servo-internal (gear train, or the
                          control deadband letting the servo be back-driven). No
                          coupling can remove that half.

HWR-41 Finding 2 is the reason the question is worth asking at all: `/hw/steer_states`
reports the SERVO, not the wheel, and on 2026-08-21 the worst crab run (-17.37 deg yaw)
reported the SECOND-BEST alignment (0.33 deg). This tool cannot see the wheel either —
the wheel side is the caliper's job. What it adds is the servo side of the same instant,
so the two can be subtracted.

USAGE — the bus is single-master, so the ROS stack must be DOWN (`steer_servo_node`
polls the same port at 30 Hz and the two will corrupt each other's replies):

    ssh -t gripperx 'source /opt/ros/jazzy/setup.bash && \
      source /home/ubuntu/ws/Software/ros2/install/setup.bash && \
      python3 /tmp/steer_play_watch.py --ids 13,14,11,12'

It is deliberately NOT installed into the package: copy it to the Pi's /tmp
(`scp Software/ros2/tools/steer_play_watch.py gripperx:/tmp/`) so that a measurement
aid never becomes part of what is deployed.

Keys: 'r' resets the min/max window (use it before each wheel), 'q' or ENTER quits.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty

# Same udev symlink steer_servo_node and steer_servo_calibrate use.
DEFAULT_PORT = "/dev/steering_servo"
DEFAULT_BAUD = 1_000_000
# MUST match steer_servo.yaml protocol_end (0 = SCS, no byte swap). At 1 every position
# read is byte-swapped, so the counts would not be the counts the node works in.
DEFAULT_PROTOCOL = 0
# Joint order FL, FR, BL, BR — the same order steer_servo.yaml and the calibration tool
# use. The ids are the 2026-08-13 measured map and MUST be re-verified after the ASA
# coupling rework (explore mode) before they are trusted here.
DEFAULT_IDS = (13, 14, 11, 12)
DEFAULT_NAMES = ("FL", "FR", "BL", "BR")
# 4096 counts = 360 deg at the wheel while the linkage is direct 1:1 (HWR-12), confirmed
# on the machine 2026-08-13. Re-verified in Phase C of the 2026-09-18 run sheet; if that
# check fails, the deg column below is meaningless and only the counts column counts.
COUNTS_PER_DEG = 4096.0 / 360.0


def _open_bus(port: str, baud: int, protocol: int):
    try:
        from gripperx_control.sts_servo_bus import StsServoBus
    except ModuleNotFoundError as exc:
        raise SystemExit(
            f"{exc}. This tool talks to the servo bus and therefore only runs on the "
            "robot (Pi), with the gripperx_control install overlay sourced."
        ) from exc

    bus = StsServoBus(port, baud, protocol)
    bus.open()
    return bus


def _stdin_ready() -> bool:
    return bool(select.select([sys.stdin], [], [], 0.0)[0])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read-only live view of the steering servo counts (commands nothing)"
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--protocol", type=int, default=DEFAULT_PROTOCOL)
    parser.add_argument("--ids", default=",".join(str(i) for i in DEFAULT_IDS))
    parser.add_argument("--names", default=",".join(DEFAULT_NAMES))
    parser.add_argument(
        "--rate", type=float, default=15.0, help="polls per second (default 15)"
    )
    args = parser.parse_args()

    ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
    names = [x.strip() for x in args.names.split(",") if x.strip()]
    if len(ids) != len(names):
        raise SystemExit("--ids and --names must have the same count")

    bus = _open_bus(args.port, args.baud, args.protocol)
    print(f"Port {args.port} @ {args.baud}  protocol_end={args.protocol}")
    print("READ-ONLY — no position is written, torque is left exactly as it was found.")
    print("Keys: r = reset min/max window (do this per wheel), q/ENTER = quit\n")

    for name, servo_id in zip(names, ids):
        if not bus.ping(servo_id):
            bus.close()
            raise SystemExit(
                f"Servo id={servo_id} ({name}) not responding. Is the ROS stack still "
                "running and holding the bus? (systemctl is-active gripperx-bringup)"
            )

    lo = {i: 4095 for i in ids}
    hi = {i: 0 for i in ids}
    period = 1.0 / args.rate if args.rate > 0 else 0.067

    old_tty = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if _stdin_ready():
                key = sys.stdin.read(1).lower()
                if key in ("q", "\n", "\r"):
                    break
                if key == "r":
                    lo = {i: 4095 for i in ids}
                    hi = {i: 0 for i in ids}
                    print("\n  min/max window reset")

            parts = []
            for name, servo_id in zip(names, ids):
                try:
                    pos = bus.read_position(servo_id)
                except RuntimeError:
                    parts.append(f"{name}=???")
                    continue
                lo[servo_id] = min(lo[servo_id], pos)
                hi[servo_id] = max(hi[servo_id], pos)
                span = hi[servo_id] - lo[servo_id]
                parts.append(
                    f"{name}={pos:4d} [span {span:3d}cts / {span / COUNTS_PER_DEG:5.2f}deg]"
                )

            print("\r" + "  ".join(parts) + "  ", end="", flush=True)
            time.sleep(period)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
        bus.close()
        print("\n")
        print("Span per servo since the last reset (this is the SERVO-SIDE movement only —")
        print("the wheel-side play is what the caliper measures at the tyre):")
        for name, servo_id in zip(names, ids):
            span = hi[servo_id] - lo[servo_id]
            if hi[servo_id] < lo[servo_id]:
                print(f"  {name} id={servo_id}: no reading")
                continue
            print(
                f"  {name} id={servo_id}: {lo[servo_id]}..{hi[servo_id]}  "
                f"span {span} counts = {span / COUNTS_PER_DEG:.2f} deg"
            )


if __name__ == "__main__":
    main()
