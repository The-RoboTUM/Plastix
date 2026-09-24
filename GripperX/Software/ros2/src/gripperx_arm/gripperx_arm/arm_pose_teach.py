#!/usr/bin/env python3
"""Teach arm poses by hand and print them as a ready-to-paste arm_poses.yaml block.

Counterpart to gripperx_control's steer_servo_calibrate for the 6-servo arm. Same
principle: torque off, move by hand, read back the raw tick values, print YAML.

Usage:
  ros2 run gripperx_arm arm_pose_teach -- home
  ros2 run gripperx_arm arm_pose_teach -- grip
  ros2 run gripperx_arm arm_pose_teach -- --watch          # read-only, no torque change

SAFETY — read before running:
  * Disabling torque makes the arm LIMP. It will fall under its own weight. Support it
    by hand or prop it up BEFORE confirming the torque-off prompt.
  * The tool never commands a position. It only switches torque off and back on.
  * arm_action_server holds /dev/arm_servo exclusively. Stop it first, otherwise the
    port cannot be opened (or both fight over the bus):
        sudo systemctl stop gripperx-bringup.service
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from typing import List, Optional, Sequence

sys.path.insert(0, '/home/ubuntu/.local/lib/python3.12/site-packages')
import scservo_sdk as scs

DEFAULT_PORT = '/dev/arm_servo'
DEFAULT_BAUD = 1_000_000
# Must match arm_action_server: IDS order is [limb1..limb5, gripper].
DEFAULT_IDS = (1, 2, 3, 4, 5, 6)
JOINT_NAMES = ('limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper')

ADDR_PRESENT = 56
ADDR_TORQUE = 40


def _stdin_ready() -> bool:
    return bool(select.select([sys.stdin], [], [], 0.0)[0])


class ArmBus:
    def __init__(self, port: str, baud: int) -> None:
        self._port = scs.PortHandler(port)
        self._pkt = scs.PacketHandler(0)
        self._port_name = port
        self._baud = baud

    def open(self) -> None:
        if not self._port.openPort():
            raise SystemExit(
                f'Cannot open {self._port_name}. Is arm_action_server still running? '
                'Try: sudo systemctl stop gripperx-bringup.service'
            )
        self._port.setBaudRate(self._baud)

    def close(self) -> None:
        self._port.closePort()

    def ping(self, servo_id: int) -> bool:
        _, result, _ = self._pkt.ping(self._port, servo_id)
        return result == scs.COMM_SUCCESS

    def read_position(self, servo_id: int) -> Optional[int]:
        value, result, _ = self._pkt.read2ByteTxRx(self._port, servo_id, ADDR_PRESENT)
        return value if result == scs.COMM_SUCCESS else None

    def set_torque(self, servo_id: int, enable: bool) -> None:
        self._pkt.write1ByteTxRx(self._port, servo_id, ADDR_TORQUE, 1 if enable else 0)


def _read_all(bus: ArmBus, ids: Sequence[int]) -> List[Optional[int]]:
    return [bus.read_position(servo_id) for servo_id in ids]


def _live_view(bus: ArmBus, ids: Sequence[int], names: Sequence[str], prompt: str) -> List[Optional[int]]:
    """Print live positions until ENTER, then return the last sample."""
    print(f'{prompt}\n')
    latest = _read_all(bus, ids)
    while True:
        if _stdin_ready():
            sys.stdin.readline()
            break
        latest = _read_all(bus, ids)
        line = '  |  '.join(
            f'{name}({servo_id})={value if value is not None else "?"}'
            for name, servo_id, value in zip(names, ids, latest)
        )
        print(f'\r{line:<100}', end='', flush=True)
        time.sleep(0.08)
    print()
    return latest


def _print_yaml(pose_name: str, ids: Sequence[int], values: Sequence[Optional[int]]) -> None:
    if any(value is None for value in values):
        missing = [servo_id for servo_id, value in zip(ids, values) if value is None]
        raise SystemExit(f'Not every servo answered (missing ids: {missing}) — pose NOT recorded.')

    print('\n' + '=' * 62)
    print(f'Copy into gripperx_arm/config/arm_poses.yaml  (pose "{pose_name}"):')
    print('=' * 62)
    print('    poses:')
    print(f'      {pose_name}: [%s]' % ', '.join(str(v) for v in values))
    print()
    print('    # gripper open/close are single values, take them from the gripper servo:')
    print(f'    #   gripper servo (id {ids[-1]}) currently at {values[-1]}')
    print()
    print('Values are raw Feetech ticks in id order %s.' % list(ids))


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Teach an arm pose by hand and print it as YAML.'
    )
    parser.add_argument(
        'pose',
        nargs='?',
        help='pose name to record (e.g. home, grip). Omit together with --watch.',
    )
    parser.add_argument(
        '--watch',
        action='store_true',
        help='read-only: show live positions, never touch torque, record nothing',
    )
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--ids', default=','.join(str(i) for i in DEFAULT_IDS))
    args = parser.parse_args()

    if not args.watch and not args.pose:
        raise SystemExit('Give a pose name (e.g. "home"), or use --watch for a read-only view.')

    ids = [int(x.strip()) for x in args.ids.split(',') if x.strip()]
    names = list(JOINT_NAMES[: len(ids)])
    if len(names) < len(ids):
        names += [f'id{servo_id}' for servo_id in ids[len(names):]]

    bus = ArmBus(args.port, args.baud)
    bus.open()
    print(f'Port {args.port} @ {args.baud}')

    try:
        for servo_id, name in zip(ids, names):
            if not bus.ping(servo_id):
                raise SystemExit(f'Servo id={servo_id} ({name}) not responding')

        if args.watch:
            _live_view(bus, ids, names, 'Read-only view. Torque untouched. ENTER to quit.')
            return

        print('\n*** The arm goes LIMP when torque is switched off and WILL fall. ***')
        print('Support it by hand or prop it up now.')
        input('Press ENTER to disable torque, or Ctrl-C to abort... ')

        for servo_id in ids:
            bus.set_torque(servo_id, False)

        try:
            values = _live_view(
                bus, ids, names,
                f'Torque OFF. Move the arm into the "{args.pose}" pose, then press ENTER.',
            )
        finally:
            print('Re-enabling torque — the arm holds its current position.')
            for servo_id in ids:
                bus.set_torque(servo_id, True)

        _print_yaml(args.pose, ids, values)
    finally:
        bus.close()


if __name__ == '__main__':
    main()
