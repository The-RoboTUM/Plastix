
import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

from gripperx_control.steering_limits import (
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    LimitStatus,
    SteeringLimits,
    limit_twist_to_steering_range,
    normalize_angle as _normalize_angle,
)
from gripperx_control.swerve_kinematic_model import BodyTwist, FourWIS4WIDKinematicModel


# Model wheel order: w1 FL, w2 BL, w3 BR, w4 FR
_MODULE_TO_HW = (
    (0, 4),  # f_left_steer, f_leftwheel
    (2, 6),  # b_leftsteer, b_leftwheel
    (3, 7),  # b_rightsteer, b_rightwheel
    (1, 5),  # f_right_steer, f_rightwheel
)


def _steer_alignment_scale(target_angle: float, current_angle: float, min_scale: float) -> float:
    error = abs(_normalize_angle(target_angle - current_angle))
    return max(min_scale, 1.0 - (error / (0.5 * math.pi)))


class TeleopHw(Node):
    def __init__(self):
        super().__init__("teleop_hw")

        self.declare_parameter("a", Parameter.Type.DOUBLE)
        self.declare_parameter("b", Parameter.Type.DOUBLE)
        self.declare_parameter("wheel_radius", Parameter.Type.DOUBLE)
        # Per-wheel, per-direction window (stage B, 2026-08-13). This node writes
        # /hw/joint_commands DIRECTLY, bypassing swerve_cmd_node, so it needs the
        # same window or steer_servo_node clamps its commands silently.
        # Joint order FL, FR, BL, BR; mirrors config/steer_servo.yaml.
        self.declare_parameter("steering_outward_limit_deg", DEFAULT_OUTWARD_LIMIT_DEG)
        self.declare_parameter("steering_inward_limit_deg", DEFAULT_INWARD_LIMIT_DEG)
        self.declare_parameter("steering_outward_sign", list(DEFAULT_OUTWARD_SIGN))
        self.declare_parameter("steer_alignment_min_scale", 0.45)
        self.declare_parameter("max_wheel_angular_speed", 12.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("joint_commands_topic", "/hw/joint_commands")

        self.a = float(self.get_parameter("a").value)
        self.b = float(self.get_parameter("b").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.steering_limits = SteeringLimits.from_outward_inward(
            math.radians(float(self.get_parameter("steering_outward_limit_deg").value)),
            math.radians(float(self.get_parameter("steering_inward_limit_deg").value)),
            [int(v) for v in self.get_parameter("steering_outward_sign").value],
        )
        self.model_steering_limits = self.steering_limits.in_model_order()
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
        current_model_angles = [
            self.current_steer_angles[_MODULE_TO_HW[index][0]] for index in range(4)
        ]
        limited = limit_twist_to_steering_range(
            self.model,
            BodyTwist(vx=vx, vy=vy, omega=omega),
            current_model_angles,
            self.model_steering_limits,
        )
        detail = "; ".join(v.describe() for v in limited.violations) or "no wheel solution"

        if limited.status == LimitStatus.REJECTED:
            # Hold the steering, command zero drive -- see the design note in
            # steering_limits.limit_twist_to_steering_range.
            self.get_logger().error(
                "Steering limit: twist REJECTED (%s); holding steering, zero drive."
                % detail,
                throttle_duration_sec=2.0,
            )
            for index in range(4):
                out[index] = self.current_steer_angles[index]
            return out

        if limited.status == LimitStatus.OMEGA_REDUCED:
            self.get_logger().warning(
                "Steering limit: omega %.3f -> %.3f rad/s (%s). Same manoeuvre, "
                "wider radius." % (limited.requested_omega, limited.twist.omega, detail),
                throttle_duration_sec=2.0,
            )

        for model_index, target in enumerate(limited.targets):
            steer_index, wheel_index = _MODULE_TO_HW[model_index]
            scale = _steer_alignment_scale(
                target.angle,
                self.current_steer_angles[steer_index],
                self.steer_alignment_min_scale,
            )
            wheel_omega = (target.speed / self.wheel_radius) * scale
            wheel_omega = max(
                -self.max_wheel_angular_speed,
                min(self.max_wheel_angular_speed, wheel_omega),
            )

            out[steer_index] = target.angle
            out[wheel_index] = wheel_omega
            self.current_steer_angles[steer_index] = target.angle

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
