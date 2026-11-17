#!/usr/bin/env python3
"""arm_aimed_pick - OPERATOR test tool: send ONE aimed P (FR-16 stage 1).

    ros2 run gripperx_arm arm_aimed_pick 0.36 0.05              # target in base_link
    ros2 run gripperx_arm arm_aimed_pick 2.10 -0.40 --frame map # target in the map

THIS MOVES THE ARM AND THE GRIPPER. SR-1: run it only under an explicit user approval
for that one test; it asks for a typed confirmation before it sends anything. It is an
operator's P (execute false): arm_action_server treats it as the operator's only while
the external-goal gateway is known to be DISARMED (SR-17 rule 11 proxy), otherwise as an
automatic P (direct path only). Space (/arm/stop) stops it like any P; Ctrl-C here
cancels the goal (the arm then returns home on a planned path).

Without --frame the target is in base_link: x ahead, y to the left of the robot, at
ground level (z is ignored; the grasp height is P's).
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int64

from gripperx_arm_msgs.action import PickPlastic

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('x', type=float, help='target x (m)')
    ap.add_argument('y', type=float, help='target y (m)')
    ap.add_argument('--frame', default='base_link', help='frame of x, y (default base_link)')
    ap.add_argument('--yes', action='store_true',
                    help='skip the typed confirmation (the SR-1 approval is still yours)')
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node('arm_aimed_pick')
    epoch = []
    node.create_subscription(Int64, '/arm/stop_epoch', lambda m: epoch.append(int(m.data)),
                             LATCHED)
    client = ActionClient(node, PickPlastic, 'pick_plastic')
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            print('pick_plastic action server not found', file=sys.stderr)
            return 2
        t0 = time.monotonic()
        while not epoch and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not epoch:
            print('no /arm/stop_epoch received: the arm would refuse the goal', file=sys.stderr)
            return 2
        print(f'AIMED P at ({args.x:+.3f}, {args.y:+.3f}) in {args.frame}. '
              'THE ARM AND GRIPPER WILL MOVE.')
        if not args.yes and input('Type "pick" to send: ').strip() != 'pick':
            print('not sent')
            return 1
        rclpy.spin_once(node, timeout_sec=0.1)      # the newest epoch (a Space meanwhile)
        goal = PickPlastic.Goal(execute=False, stop_epoch=epoch[-1], aimed=True)
        goal.target.header.frame_id = args.frame
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.point.x, goal.target.point.y = args.x, args.y
        fut = client.send_goal_async(
            goal, feedback_callback=lambda f: print(
                f'  {f.feedback.current_step} ({f.feedback.progress * 100:.0f} %)'))
        rclpy.spin_until_future_complete(node, fut)
        handle = fut.result()
        if not handle.accepted:
            print('goal REJECTED (stop epoch, leader mode or another motion)')
            return 1
        res = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, res)
        except KeyboardInterrupt:
            print('cancelling ...')
            c = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, c, timeout_sec=5.0)
            rclpy.spin_until_future_complete(node, res, timeout_sec=30.0)
        r = res.result().result
        tb = r.target_in_base
        print(f'success {r.success}  reason "{r.reason}"\n  {r.message}')
        if tb.header.frame_id:
            print(f'  target in {tb.header.frame_id}: ({tb.point.x:+.3f}, {tb.point.y:+.3f})')
        return 0 if r.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
