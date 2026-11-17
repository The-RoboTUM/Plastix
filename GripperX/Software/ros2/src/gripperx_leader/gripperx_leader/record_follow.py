"""ros2 run gripperx_leader record_follow <output.csv> --duration 60

Records the leader (/arm/leader/joint_states, radians), the follower (/joint_states,
arm_limb*_joint, radians) and arm_action_server's per-tick diagnostics
(/arm/leader/follow_debug: setpoint, target, measured, tick duration, period and the
time per tick phase, while leader mode is active and leader.follow_debug_enabled) with
receive times and header stamps into a CSV, plus the String states /arm/leader/state and (unless --no-wall)
/arm/leader/wall_state. Stops after --duration seconds or Ctrl-C, then prints the
analysis (follow_analysis).
Passive: a pure subscriber, publishes nothing, never touches the arm.

    ros2 run gripperx_leader analyze_follow <file.csv>   # analysis of a recording
"""

import argparse
import os
import sys
import time
from typing import Dict, Optional

import rclpy
import rclpy.utilities
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gripperx_arm_msgs.msg import LeaderFollowDebug
from gripperx_leader import follow_analysis as fa

STREAM_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
# follow_debug is published best effort (a lost sample must not block the tick).
DEBUG_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)


def _stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def debug_phases(msg) -> Optional[Dict[str, float]]:
    """{phase: seconds} of a LeaderFollowDebug; None for a message type without them."""
    if not hasattr(msg, 'other_s'):
        return None
    return {p: float(getattr(msg, f'{p}_s')) for p in fa.PHASES}


class FollowRecorder(Node):
    def __init__(self, writer: fa.CsvWriter, wall: bool):
        super().__init__('record_follow')
        self._w = writer
        self.counts = {fa.LEADER: 0, fa.FOLLOWER: 0, fa.SETPOINT: 0, fa.WALL_STATE: 0,
                       fa.LEADER_STATE: 0}
        self.create_subscription(JointState, '/arm/leader/joint_states',
                                 lambda m: self._joints(fa.LEADER, m), STREAM_QOS)
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self._joints(fa.FOLLOWER, m), STREAM_QOS)
        self.create_subscription(LeaderFollowDebug, '/arm/leader/follow_debug',
                                 self._debug, DEBUG_QOS)
        self.create_subscription(String, '/arm/leader/state',
                                 lambda m: self._text(fa.LEADER_STATE, m), LATCHED_QOS)
        if wall:
            self.create_subscription(String, '/arm/leader/wall_state',
                                     lambda m: self._text(fa.WALL_STATE, m), LATCHED_QOS)

    def _joints(self, source: str, msg: JointState):
        row = fa.joint_row(time.time(), _stamp(msg), source, msg.name, msg.position)
        if not any(j in row for j in fa.JOINTS):
            return                                  # e.g. a /joint_states without arm joints
        self._w.write(row)
        self.counts[source] += 1

    def _debug(self, msg):
        for row in fa.debug_rows(time.time(), _stamp(msg), msg.joint_names, msg.setpoint,
                                 msg.target, msg.measured, msg.tick_duration_s,
                                 msg.period_s, msg.outcome, debug_phases(msg),
                                 getattr(msg, 'check_poses', None)):
            self._w.write(row)
        self.counts[fa.SETPOINT] += 1

    def _text(self, source: str, msg: String):
        self._w.write(fa.text_row(time.time(), source, msg.data))
        self.counts[source] += 1


def _max_velocity(arg: float, safety_filter_file: str):
    if arg is not None:
        return arg, 'command line'
    from gripperx_leader.follower_config import leader_max_velocity
    return leader_max_velocity(safety_filter_file or None)


def _rate_hz(arg: float, safety_filter_file: str):
    if arg is not None:
        return arg, 'command line'
    from gripperx_leader.follower_config import leader_rate_hz
    return leader_rate_hz(safety_filter_file or None)


def _report(path: str, args) -> None:
    v, src = _max_velocity(args.max_velocity, args.follower_safety_filter_file)
    rate, rsrc = _rate_hz(args.rate_hz, args.follower_safety_filter_file)
    print(f'follower speed limit {v} rad/s from {src}, leader rate {rate} Hz from {rsrc} '
          '(the robot runs ITS copy: check they match)')
    res = fa.analyze(fa.read_csv(path), v, rate_hz=rate, time_base=args.time_base)
    print(fa.format_report(res))


def _common(parser):
    parser.add_argument('--max-velocity', type=float, default=None,
                        help='follower speed limit, rad/s (default: leader.max_joint_velocity_'
                        'rad_s of the installed gripperx_arm arm_safety_filter.yaml)')
    parser.add_argument('--rate-hz', type=float, default=None,
                        help='leader tick rate, Hz (default: leader.rate_hz of the same file)')
    parser.add_argument('--follower-safety-filter-file', default='',
                        help='read the speed limit and the rate from this file instead')
    parser.add_argument('--time-base', choices=['recv_s', 'stamp_s'], default='recv_s',
                        help='time base of lag and error (default receive time; stamp_s '
                        'needs laptop and Pi clocks in sync)')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('output', help='CSV file to write (refused if it exists, see --force)')
    parser.add_argument('--duration', type=float, required=True, help='seconds to record')
    parser.add_argument('--no-wall', action='store_true', help='do not record wall_state')
    parser.add_argument('--force', action='store_true', help='overwrite an existing file')
    parser.add_argument('--no-report', action='store_true', help='skip the analysis')
    _common(parser)
    args = parser.parse_args(argv if argv is not None
                             else rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if os.path.exists(args.output) and not args.force:
        raise SystemExit(f'{args.output} exists (use --force to overwrite)')
    if not args.duration > 0.0:
        raise SystemExit('--duration must be > 0')

    rclpy.init()
    with open(args.output, 'w', newline='') as fh:
        writer = fa.CsvWriter(fh)
        node = FollowRecorder(writer, wall=not args.no_wall)
        print(f'recording {args.duration:.0f} s on ROS_DOMAIN_ID='
              f'{os.environ.get("ROS_DOMAIN_ID", "0 (unset)")} -> {args.output} '
              '(Ctrl-C ends early)')
        end = time.monotonic() + args.duration
        last_flush = last_print = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.monotonic()
                if now - last_flush > 1.0:
                    writer.flush()
                    last_flush = now
                if now - last_print > 5.0:
                    print(f'  {end - now:5.0f} s left, received {node.counts}')
                    last_print = now
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            writer.flush()
            counts = dict(node.counts)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(f'written {args.output}: {counts}')
    if not counts[fa.LEADER] or not counts[fa.FOLLOWER]:
        print('WARNING: no leader or no follower samples - wrong ROS_DOMAIN_ID, or the '
              'node not running?')
    if not counts[fa.SETPOINT]:
        print('WARNING: no /arm/leader/follow_debug - leader mode not active, or the robot '
              'runs an arm_action_server without it / with leader.follow_debug_enabled false')
    if not args.no_report:
        _report(args.output, args)


def analyze_main(argv=None):
    parser = argparse.ArgumentParser(description='Analyse a record_follow CSV.')
    parser.add_argument('csv', help='a record_follow recording')
    _common(parser)
    args = parser.parse_args(argv)
    _report(args.csv, args)


if __name__ == '__main__':
    main()
