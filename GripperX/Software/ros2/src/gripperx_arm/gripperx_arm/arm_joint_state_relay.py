#!/usr/bin/env python3
"""Relay /arm/joint_states (raw servo angles) to /joint_states (URDF joint angles).

arm_action_server publishes, per servo, ticks * 2 pi / 4096 under the names
limb1..limb5, gripper - an unoffset, unsigned servo reading. robot_state_publisher
needs the URDF joint angles of gripperx_v1.arm.xacro. This node applies the
calibration of config/arm_calibration.yaml and nothing else. It reads only; it
never commands the arm.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gripperx_arm.kinematics import TickCalibration


class ArmJointStateRelay(Node):
    def __init__(self):
        super().__init__('arm_joint_state_relay')
        self.declare_parameter('calibration_verified', False)
        self.declare_parameter('ticks_per_rev', 4096)
        self.declare_parameter('servo_names', ['limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper'])
        self.declare_parameter('urdf_joints', [''])
        self.declare_parameter('zero_ticks', [0])
        self.declare_parameter('signs', [0])
        self.declare_parameter('range_center_rad', [0.0])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._servo_names = list(g('servo_names'))
        self._tpr = float(g('ticks_per_rev'))
        centers = [float(v) for v in g('range_center_rad')]
        if centers == [0.0]:
            centers = [0.0] * len(g('urdf_joints'))
        self._cal = TickCalibration(list(g('urdf_joints')), [float(v) for v in g('zero_ticks')],
                                    [int(v) for v in g('signs')], self._tpr, centers)
        if len(self._servo_names) != len(self._cal.joints):
            raise ValueError('servo_names and urdf_joints must have equal length')
        self.declare_parameter('unverified_warn_period_sec', 30.0)
        if not g('calibration_verified'):
            self._warn_unverified()
            self.create_timer(float(g('unverified_warn_period_sec')), self._warn_unverified)
        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointState, '/arm/joint_states', self._cb, 10)

    def _warn_unverified(self):
        self.get_logger().warn(
            'Arm calibration NOT verified (calibration_verified: false in '
            'config/arm_calibration.yaml): the arm in TF/RViz is an approximation '
            'anchored on the hand-taught home pose.')

    def _cb(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        try:
            ticks = [pos[n] * self._tpr / (2.0 * math.pi) for n in self._servo_names]
        except KeyError as exc:
            self.get_logger().warn(f'/arm/joint_states lacks {exc}', throttle_duration_sec=10.0)
            return
        q = self._cal.to_radians(ticks)
        out = JointState()
        out.header = msg.header
        out.name = list(q.keys())
        out.position = list(q.values())
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
