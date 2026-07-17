
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from gripperx_control.swerve_kinematic_model import BodyTwist, FourWIS4WIDKinematicModel


# Model wheel order: w1 FL, w2 BL, w3 BR, w4 FR
_MODULE_TO_HW = (
    (0, 4),  # f_left_steer, f_leftwheel
    (2, 6),  # b_leftsteer, b_leftwheel
    (3, 7),  # b_rightsteer, b_rightwheel
    (1, 5),  # f_right_steer, f_rightwheel
)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _optimize_wheel_command(
    target_angle: float,
    target_speed: float,
    current_angle: float,
) -> Tuple[float, float]:
    angle = _normalize_angle(target_angle)
    speed = target_speed
    if abs(_normalize_angle(angle - current_angle)) > (0.5 * math.pi):
        angle = _normalize_angle(angle + math.pi)
        speed = -speed
    return angle, speed


def _steer_alignment_scale(target_angle: float, current_angle: float, min_scale: float) -> float:
    error = abs(_normalize_angle(target_angle - current_angle))
    return max(min_scale, 1.0 - (error / (0.5 * math.pi)))


class TeleopHw(Node):
    def __init__(self):
        super().__init__("teleop_hw")

        self.declare_parameter("a", 0.203)
        self.declare_parameter("b", 0.16556)
        self.declare_parameter("wheel_radius", 0.052)
        self.declare_parameter("steering_angle_limit", 0.7854)
        self.declare_parameter("steer_alignment_min_scale", 0.45)
        self.declare_parameter("max_wheel_angular_speed", 12.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("joint_commands_topic", "/hw/joint_commands")

        self.a = float(self.get_parameter("a").value)
        self.b = float(self.get_parameter("b").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.steering_angle_limit = float(self.get_parameter("steering_angle_limit").value)
        self.steer_alignment_min_scale = float(self.get_parameter("steer_alignment_min_scale").value)
        self.max_wheel_angular_speed = float(self.get_parameter("max_wheel_angular_speed").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.joint_commands_topic = str(self.get_parameter("joint_commands_topic").value)

        self.model = FourWIS4WIDKinematicModel(self.a, self.b, self.wheel_radius)
        self.cmd = Twist()
        self.last_cmd_time = None
        self.current_steer_angles = [0.0] * 4

        self.pub = self.create_publisher(Float64MultiArray, self.joint_commands_topic, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(1.0 / self.control_rate_hz, self.on_timer)

    def on_cmd_vel(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _compute_commands(self, vx: float, vy: float, omega: float) -> List[float]:
        out = [0.0] * 8
        wheel_commands = self.model.inverse_kinematics(BodyTwist(vx=vx, vy=vy, omega=omega))

        for model_index, command in enumerate(wheel_commands):
            steer_index, wheel_index = _MODULE_TO_HW[model_index]
            angle, linear_speed = _optimize_wheel_command(
                command.steering_angle,
                command.linear_speed,
                self.current_steer_angles[steer_index],
            )
            angle = max(-self.steering_angle_limit, min(self.steering_angle_limit, angle))
            scale = _steer_alignment_scale(
                angle,
                self.current_steer_angles[steer_index],
                self.steer_alignment_min_scale,
            )
            wheel_omega = (linear_speed / self.wheel_radius) * scale
            wheel_omega = max(
                -self.max_wheel_angular_speed,
                min(self.max_wheel_angular_speed, wheel_omega),
            )

            out[steer_index] = angle
            out[wheel_index] = wheel_omega
            self.current_steer_angles[steer_index] = angle

        return out

    def on_timer(self):
        out = Float64MultiArray()
        out.data = [0.0] * 8

        if self.last_cmd_time is None:
            self.pub.publish(out)
            return

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_vel_timeout_sec:
            self.current_steer_angles = [0.0] * 4
            self.pub.publish(out)
            return

        out.data = self._compute_commands(
            float(self.cmd.linear.x),
            float(self.cmd.linear.y),
            float(self.cmd.angular.z),
        )
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TeleopHw()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
