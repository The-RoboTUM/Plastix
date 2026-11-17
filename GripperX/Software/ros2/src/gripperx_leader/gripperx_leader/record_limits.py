"""ros2 run gripperx_leader record_limits <output.yaml>

Records min/max per joint of /arm/leader/ticks and (if present) /arm/joint_states
(follower, radians -> ticks via 4096/2pi). Keys: 'n' = name + snapshot a pose,
'w' = write the file, 'q' = write and quit. Refuses to write inside an installed
or source package config directory. Passive: publishes nothing.
"""

import argparse
import math
import os
import select
import sys
import termios
import tty

import rclpy
import rclpy.utilities
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gripperx_leader.limits_recorder import LimitsRecorder


def refuse_package_config(path: str) -> None:
    parts = os.path.abspath(path).split(os.sep)
    if 'config' in parts and ('gripperx_arm' in parts or 'share' in parts):
        raise SystemExit(f'refusing to write into a package config directory: {path}')


def write_yaml(recorder: LimitsRecorder, path: str) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        f.write(recorder.to_yaml())


class RecorderNode(Node):
    def __init__(self, recorder: LimitsRecorder):
        super().__init__('record_limits')
        self.rec = recorder
        self.create_subscription(JointState, '/arm/leader/ticks',
                                 lambda m: self.rec.add_leader(m.position, m.name), 10)
        self.create_subscription(JointState, '/arm/leader/joint_states',
                                 lambda m: self.rec.add_leader_rad(m.position, m.name), 10)
        self.create_subscription(JointState, '/arm/joint_states', self._on_follower, 10)

    def _on_follower(self, msg):
        self.rec.add_follower([p / (2.0 * math.pi) * 4096.0 for p in msg.position], msg.name)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--calibration', help='calibration YAML: its zero_ticks are the wrap reference')
    parser.add_argument('output', help='YAML file to write (never a package config)')
    args = parser.parse_args(argv if argv is not None else rclpy.utilities.remove_ros_args(sys.argv)[1:])
    refuse_package_config(args.output)

    rclpy.init()
    zero = cal = None
    if args.calibration:
        from gripperx_leader.calibration import load_calibration
        cal = load_calibration(args.calibration)
        zero = list(cal.zero_ticks)
    rec = LimitsRecorder(leader_zero=zero, calibration=cal)
    node = RecorderNode(rec)
    print("keys: n = name a pose and snapshot it   w = write   q = write and quit")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            ch = sys.stdin.read(1)
            if ch == 'n':
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                name = input('\npose name: ').strip()
                tty.setcbreak(fd)
                if name and rec.snapshot_pose(name):
                    print(f'pose "{name}" = {rec.poses[name]}')
                else:
                    print('no name or no leader sample yet')
            elif ch == 'w':
                write_yaml(rec, args.output)
                print(f'\nwritten {args.output}')
            elif ch == 'q':
                write_yaml(rec, args.output)
                print(f'\nwritten {args.output}')
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
