import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_controller.swerve_controller import FourWIS4WIDKinematicController, Pose2D
from robot_controller.swerve_kinematic_model import BodyTwist, FourWIS4WIDKinematicModel


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
        self.declare_parameter("max_wheel_angular_speed", 30.0)
        self.declare_parameter("kx", 4.0)
        self.declare_parameter("ky", 4.0)
        self.declare_parameter("ktheta", 3.0)
        self.declare_parameter("kd_gains", [5.0, 5.0, 5.0, 5.0])
        self.declare_parameter(
            "steering_joint_names",
            ["f_left_steer", "b_leftsteer", "b_rightsteer", "f_right_steer"],
        )
        self.declare_parameter(
            "drive_joint_names",
            ["f_leftwheel", "b_leftwheel", "b_rightwheel", "f_rightwheel"],
        )

        self.a = float(self.get_parameter("a").value)
        self.b = float(self.get_parameter("b").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.steering_angle_limit = float(self.get_parameter("steering_angle_limit").value)
        self.max_wheel_angular_speed = float(self.get_parameter("max_wheel_angular_speed").value)
        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)
        self.ktheta = float(self.get_parameter("ktheta").value)
        self.kd_gains = [float(value) for value in self.get_parameter("kd_gains").value]
        self.steering_joint_names: List[str] = list(self.get_parameter("steering_joint_names").value)
        self.drive_joint_names: List[str] = list(self.get_parameter("drive_joint_names").value)

        if len(self.steering_joint_names) != 4 or len(self.drive_joint_names) != 4:
            raise ValueError("Swerve command node expects exactly 4 steering and 4 drive joints")
        if len(self.kd_gains) != 4:
            raise ValueError("kd_gains must contain exactly 4 values")

        model = FourWIS4WIDKinematicModel(
            a=self.a,
            b=self.b,
            wheel_radius=self.wheel_radius,
        )
        self.controller = FourWIS4WIDKinematicController(
            model=model,
            kx=self.kx,
            ky=self.ky,
            ktheta=self.ktheta,
            kd_gains=self.kd_gains,
        )

        self.latest_joint_state: Optional[JointState] = None
        self.latest_joint_indices: Dict[str, int] = {}
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

        self.get_logger().info(
            (
                "Swerve command node ready. cmd_vel=%s joint_states=%s command_topic=%s "
                "geometry(a=%.6f, b=%.6f, r=%.6f)"
            )
            % (
                self.cmd_vel_topic,
                self.joint_state_topic,
                self.command_topic,
                self.a,
                self.b,
                self.wheel_radius,
            )
        )

    def joint_state_callback(self, message: JointState) -> None:
        self.latest_joint_state = message
        self.latest_joint_indices = {name: index for index, name in enumerate(message.name)}

    def cmd_callback(self, message: Twist) -> None:
        if self.latest_joint_state is None:
            self.get_logger().warning("Waiting for /joint_states before generating swerve commands.")
            return

        missing = [
            name for name in self.steering_joint_names + self.drive_joint_names
            if name not in self.latest_joint_indices
        ]
        if missing:
            self.get_logger().warning(
                "Waiting for required joints in /joint_states: %s" % ", ".join(missing)
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

        current_steering_angles = [
            float(self.latest_joint_state.position[self.latest_joint_indices[name]])
            for name in self.steering_joint_names
        ]

        desired_body_twist = BodyTwist(
            vx=float(message.linear.x),
            vy=float(message.linear.y),
            omega=float(message.angular.z),
        )
        zero_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        result = self.controller.compute_control_from_body_twist(
            current_pose=zero_pose,
            desired_pose=zero_pose,
            desired_body_twist=desired_body_twist,
            current_steering_angles=current_steering_angles,
            dt=dt,
        )

        steering_positions = [
            max(-self.steering_angle_limit, min(self.steering_angle_limit, angle))
            for angle in result.wheel_reference.steering_angles
        ]
        wheel_angular_speeds = []
        for linear_speed in result.wheel_reference.wheel_linear_speeds:
            angular_speed = linear_speed / self.wheel_radius
            angular_speed = max(
                -self.max_wheel_angular_speed,
                min(self.max_wheel_angular_speed, angular_speed),
            )
            wheel_angular_speeds.append(angular_speed)

        command = JointState()
        command.header.stamp = now.to_msg()
        command.name = self.steering_joint_names + self.drive_joint_names
        command.position = steering_positions + [0.0] * len(self.drive_joint_names)
        command.velocity = list(result.wheel_reference.steering_rate_commands) + wheel_angular_speeds
        self.command_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwerveCmdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
