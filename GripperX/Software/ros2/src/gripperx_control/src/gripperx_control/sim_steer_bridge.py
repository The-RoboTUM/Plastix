"""Sim counterpart to steer_servo_node (DT-10).

On the real robot, steer_servo_node drives the four Feetech steering servos:
it takes the swerve-derived steering commands (/hw/joint_commands) AND the
direct steering override (/teleop/direct_steer) and lets the direct override
win -- except in autonomous mode. In the sim there are no Feetech servos; the
four steering joints are moved by the steering_position_controller
(ros2_control).

This node replicates exactly the same arbitration, but writes to
/steering_position_controller/commands instead of the servo bus. This makes
A/D in keyboard teleop drive the same per-axis kinematics as on the real
robot (FL=FR=+angle, BL=BR=-angle via /teleop/direct_steer), instead of the
former swerve-IK makeshift steering via angular.z (which produced divergent
left/right angles per module during rotation -- the "fighting each other"
behavior).

This node is the SOLE publisher of /steering_position_controller/commands in
the sim; joint_command_bridge then only publishes the wheel velocities there
(publish_steering=false). The steering fallback for the autonomous/Nav2 path
comes from /swerve_cmd_joint_states (the same source the bridge otherwise
uses).

Pure sim node -- the real steering path (steer_servo_node) stays unchanged.
"""

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

STEER_JOINT_COUNT = 4


class SimSteerBridge(Node):
    def __init__(self) -> None:
        super().__init__("sim_steer_bridge")

        self.declare_parameter("direct_steer_topic", "/teleop/direct_steer")
        self.declare_parameter("swerve_command_topic", "/swerve_cmd_joint_states")
        self.declare_parameter("active_mode_topic", "/teleop/active_mode")
        self.declare_parameter(
            "steering_command_topic", "/steering_position_controller/commands"
        )
        self.declare_parameter(
            "steering_joint_names",
            ["f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"],
        )
        self.declare_parameter("direct_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 50.0)
        # 60deg (2026-07-16): sim steering clamp raised with swerve_cmd (true
        # tangential spin needs 50.8deg). Sim-only; real steer_servo_node stays 45deg.
        self.declare_parameter("steering_angle_limit", 1.0472)

        direct_topic = str(self.get_parameter("direct_steer_topic").value)
        swerve_topic = str(self.get_parameter("swerve_command_topic").value)
        mode_topic = str(self.get_parameter("active_mode_topic").value)
        self._steering_command_topic = str(
            self.get_parameter("steering_command_topic").value
        )
        self._steering_joint_names: List[str] = list(
            self.get_parameter("steering_joint_names").value
        )
        self._direct_timeout = float(self.get_parameter("direct_timeout_sec").value)
        rate = float(self.get_parameter("control_rate_hz").value)
        self._limit = float(self.get_parameter("steering_angle_limit").value)

        if len(self._steering_joint_names) != STEER_JOINT_COUNT:
            raise ValueError("steering_joint_names must contain exactly 4 values")
        if rate <= 0.0:
            raise ValueError("control_rate_hz must be positive")

        self._direct_angles: Optional[List[float]] = None
        self._direct_last_time = None
        self._swerve_angles: Optional[List[float]] = None
        self._teleop_mode: str = "keyboard"

        self._pub = self.create_publisher(
            Float64MultiArray, self._steering_command_topic, 10
        )
        self.create_subscription(
            Float64MultiArray, direct_topic, self._on_direct_steer, 10
        )
        self.create_subscription(JointState, swerve_topic, self._on_swerve_cmd, 20)
        self.create_subscription(
            String, mode_topic, lambda m: setattr(self, "_teleop_mode", m.data), 10
        )
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            "sim_steer_bridge ready | direct=%s + swerve=%s -> %s "
            "(direct override except in autonomous mode)"
            % (direct_topic, swerve_topic, self._steering_command_topic)
        )

    def _clamp(self, angle: float) -> float:
        return max(-self._limit, min(self._limit, angle))

    def _on_direct_steer(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < STEER_JOINT_COUNT:
            return
        self._direct_angles = [self._clamp(float(v)) for v in msg.data[:STEER_JOINT_COUNT]]
        self._direct_last_time = self.get_clock().now()

    def _on_swerve_cmd(self, msg: JointState) -> None:
        name_to_index: Dict[str, int] = {
            name: index for index, name in enumerate(msg.name)
        }
        angles: List[float] = []
        for joint_name in self._steering_joint_names:
            index = name_to_index.get(joint_name)
            if index is None or index >= len(msg.position):
                return
            angles.append(self._clamp(float(msg.position[index])))
        self._swerve_angles = angles

    def _on_timer(self) -> None:
        # Direct override: only in keyboard/controller mode (not
        # autonomous/Nav2), exactly like steer_servo_node._on_timer.
        if self._teleop_mode != "autonomous" and self._direct_last_time is not None:
            age = (self.get_clock().now() - self._direct_last_time).nanoseconds * 1e-9
            if age < self._direct_timeout and self._direct_angles is not None:
                self._publish(self._direct_angles)
                return

        # Fallback: swerve-derived steering (autonomous/Nav2 path).
        if self._swerve_angles is not None:
            self._publish(self._swerve_angles)

    def _publish(self, angles: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(angles)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSteerBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
