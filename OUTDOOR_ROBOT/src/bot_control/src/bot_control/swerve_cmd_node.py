import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bot_control.swerve_controller import FourWIS4WIDKinematicController, Pose2D, normalize_angle
from bot_control.swerve_kinematic_model import (
    BodyTwist,
    FourWIS4WIDKinematicModel,
    WheelCommand,
)

# Paper / kinematic model wheel order (w1..w4 in Lee 2015).
MODEL_STEERING_JOINTS = [
    "f_left_steer",
    "b_leftsteer",
    "b_rightsteer",
    "f_right_steer",
]
MODEL_DRIVE_JOINTS = [
    "f_leftwheel",
    "b_leftwheel",
    "b_rightwheel",
    "f_rightwheel",
]


def _optimize_wheel_command(
    target_angle: float,
    target_speed: float,
    current_angle: float,
) -> Tuple[float, float]:
    """Pick the equivalent steering solution closest to the current module angle."""

    angle = normalize_angle(target_angle)
    speed = target_speed
    if abs(normalize_angle(angle - current_angle)) > (0.5 * math.pi):
        angle = normalize_angle(angle + math.pi)
        speed = -speed
    return angle, speed


def _steer_alignment_scale(target_angle: float, current_angle: float) -> float:
    """Reduce wheel drive while steering modules are still rotating."""

    error = abs(normalize_angle(target_angle - current_angle))
    return max(0.35, 1.0 - (error / (0.5 * math.pi)))


class SwerveCmdNode(Node):
    def __init__(self) -> None:
        super().__init__("swerve_cmd_node")

        self.declare_parameter("a", 0.203)
        self.declare_parameter("b", 0.16556)
        self.declare_parameter("wheel_radius", 0.052)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/swerve_cmd_joint_states")
        self.declare_parameter("steering_angle_limit", 1.5708)
        self.declare_parameter("max_wheel_angular_speed", 12.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("use_direct_ik", True)
        self.declare_parameter("kx", 4.0)
        self.declare_parameter("ky", 4.0)
        self.declare_parameter("ktheta", 3.0)
        self.declare_parameter("kd_gains", [5.0, 5.0, 5.0, 5.0])

        self.a = float(self.get_parameter("a").value)
        self.b = float(self.get_parameter("b").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.steering_angle_limit = float(self.get_parameter("steering_angle_limit").value)
        self.max_wheel_angular_speed = float(self.get_parameter("max_wheel_angular_speed").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.use_direct_ik = bool(self.get_parameter("use_direct_ik").value)
        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)
        self.ktheta = float(self.get_parameter("ktheta").value)
        self.kd_gains = [float(value) for value in self.get_parameter("kd_gains").value]

        if len(self.kd_gains) != 4:
            raise ValueError("kd_gains must contain exactly 4 values")
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")

        self.model = FourWIS4WIDKinematicModel(
            a=self.a,
            b=self.b,
            wheel_radius=self.wheel_radius,
        )
        self.controller = FourWIS4WIDKinematicController(
            model=self.model,
            kx=self.kx,
            ky=self.ky,
            ktheta=self.ktheta,
            kd_gains=self.kd_gains,
        )

        self.latest_joint_state: Optional[JointState] = None
        self.latest_joint_indices: Dict[str, int] = {}
        self.latest_cmd_vel = Twist()
        self.last_cmd_vel_time = None
        self.last_command_time = None

        self.command_publisher = self.create_publisher(JointState, self.command_topic, 10)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            50,
        )
        self.cmd_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            20,
        )
        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self.control_timer_callback)

        self.get_logger().info(
            (
                "Swerve command node ready. cmd_vel=%s joint_states=%s command_topic=%s "
                "geometry(a=%.6f, b=%.6f, r=%.6f) rate=%.1fHz direct_ik=%s"
            )
            % (
                self.cmd_vel_topic,
                self.joint_state_topic,
                self.command_topic,
                self.a,
                self.b,
                self.wheel_radius,
                self.control_rate_hz,
                self.use_direct_ik,
            )
        )

    def joint_state_callback(self, message: JointState) -> None:
        self.latest_joint_state = message
        self.latest_joint_indices = {name: index for index, name in enumerate(message.name)}

    def cmd_callback(self, message: Twist) -> None:
        self.latest_cmd_vel = message
        self.last_cmd_vel_time = self.get_clock().now()

    def _read_model_steering_angles(self) -> Optional[List[float]]:
        if self.latest_joint_state is None:
            return None

        angles = []
        for joint_name in MODEL_STEERING_JOINTS:
            index = self.latest_joint_indices.get(joint_name)
            if index is None or index >= len(self.latest_joint_state.position):
                return None
            angles.append(float(self.latest_joint_state.position[index]))
        return angles

    def _compute_direct_ik(
        self,
        desired_body_twist: BodyTwist,
        current_steering_angles: List[float],
    ) -> Tuple[List[float], List[float]]:
        wheel_commands: Tuple[WheelCommand, ...] = self.model.inverse_kinematics(desired_body_twist)

        steering_positions = []
        wheel_angular_speeds = []
        for command, current_angle in zip(wheel_commands, current_steering_angles):
            angle, speed = _optimize_wheel_command(
                command.steering_angle,
                command.linear_speed,
                current_angle,
            )
            angle = max(-self.steering_angle_limit, min(self.steering_angle_limit, angle))
            scale = _steer_alignment_scale(angle, current_angle)
            angular_speed = (speed / self.wheel_radius) * scale
            angular_speed = max(
                -self.max_wheel_angular_speed,
                min(self.max_wheel_angular_speed, angular_speed),
            )
            steering_positions.append(angle)
            wheel_angular_speeds.append(angular_speed)

        return steering_positions, wheel_angular_speeds

    def _compute_tracking_control(
        self,
        desired_body_twist: BodyTwist,
        current_steering_angles: List[float],
        dt: Optional[float],
    ) -> Tuple[List[float], List[float]]:
        zero_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        result = self.controller.compute_control_from_body_twist(
            current_pose=zero_pose,
            desired_pose=zero_pose,
            desired_body_twist=desired_body_twist,
            current_steering_angles=current_steering_angles,
            dt=dt,
        )

        steering_positions = []
        wheel_angular_speeds = []
        for target_angle, linear_speed, current_angle in zip(
            result.wheel_reference.steering_angles,
            result.wheel_reference.wheel_linear_speeds,
            current_steering_angles,
        ):
            angle = max(-self.steering_angle_limit, min(self.steering_angle_limit, target_angle))
            scale = _steer_alignment_scale(angle, current_angle)
            angular_speed = (linear_speed / self.wheel_radius) * scale
            angular_speed = max(
                -self.max_wheel_angular_speed,
                min(self.max_wheel_angular_speed, angular_speed),
            )
            steering_positions.append(angle)
            wheel_angular_speeds.append(angular_speed)

        return steering_positions, wheel_angular_speeds

    def control_timer_callback(self) -> None:
        if self.last_cmd_vel_time is None:
            return

        cmd_age = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_vel_timeout_sec:
            self.latest_cmd_vel = Twist()

        if self.latest_joint_state is None:
            self.get_logger().warning(
                "Waiting for /joint_states before generating swerve commands.",
                throttle_duration_sec=2.0,
            )
            return

        current_steering_angles = self._read_model_steering_angles()
        if current_steering_angles is None:
            self.get_logger().warning(
                "Waiting for steering joints in /joint_states.",
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()
        if self.last_command_time is None:
            dt = None
        else:
            dt = (now.nanoseconds - self.last_command_time.nanoseconds) * 1e-9
            if dt <= 0.0:
                dt = None
        self.last_command_time = now

        desired_body_twist = BodyTwist(
            vx=float(self.latest_cmd_vel.linear.x),
            vy=float(self.latest_cmd_vel.linear.y),
            omega=float(self.latest_cmd_vel.angular.z),
        )

        if self.use_direct_ik:
            steering_positions, wheel_angular_speeds = self._compute_direct_ik(
                desired_body_twist,
                current_steering_angles,
            )
        else:
            steering_positions, wheel_angular_speeds = self._compute_tracking_control(
                desired_body_twist,
                current_steering_angles,
                dt,
            )

        command = JointState()
        command.header.stamp = now.to_msg()
        command.name = MODEL_STEERING_JOINTS + MODEL_DRIVE_JOINTS
        command.position = steering_positions + [0.0] * len(MODEL_DRIVE_JOINTS)
        command.velocity = [0.0] * len(MODEL_STEERING_JOINTS) + wheel_angular_speeds
        self.command_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwerveCmdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
