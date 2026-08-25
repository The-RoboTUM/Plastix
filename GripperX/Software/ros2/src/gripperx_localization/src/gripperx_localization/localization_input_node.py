from copy import deepcopy
import math
from typing import Dict, Iterable, Sequence

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from gripperx_control.swerve_kinematic_model import FourWIS4WIDKinematicModel


def covariance_is_unset(values: Iterable[float]) -> bool:
    return all(abs(value) < 1e-12 for value in values)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half_yaw)
    quat.w = math.cos(half_yaw)
    return quat


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationInputNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_input_node")

        self.declare_parameter("imu_input_topic", "/imu/data")
        self.declare_parameter("imu_output_topic", "/imu/data/filtered")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter(
            "drive_joint_names",
            ["f_leftwheel", "b_leftwheel", "b_rightwheel", "f_rightwheel"],
        )
        self.declare_parameter(
            "steer_joint_names",
            ["f_left_steer", "b_leftsteer", "b_rightsteer", "f_right_steer"],
        )
        # Defaults mirror config/localization.yaml, which is the source of truth
        # and carries the full rationale (CAD origin, and why these are wheel
        # CONTACT POINTS rather than king-pin positions). They are kept in step so
        # that running this node without a params file does not silently fall back
        # to a different robot; the previous defaults were the obsolete-CAD values.
        self.declare_parameter(
            "wheel_positions",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("effective_wheel_radius", Parameter.Type.DOUBLE)
        self.declare_parameter("drive_joint_multipliers", [1.0, 1.0, 1.0, 1.0])
        # WHICH QUANTITY THE WHEEL RATE COMES FROM. Default changed to "position"
        # 2026-08-21, and the reason is a failure mode, not a preference:
        #
        # /joint_states velocity for a wheel is index 4-7 of the firmware's
        # /hw/joint_states, and gripperx_hardware_interface/INTERFACE.md states that
        # index is "EITHER a real encoder measurement OR the commanded velocity handed
        # straight back" -- MotorController::getRPM() falls back to the target when the
        # encoder is not running -- "and nothing in the value itself distinguishes the
        # two". The only discriminator is the provenance array (indices 12-15,
        # /hw/wheel_feedback_valid), which THIS NODE DOES NOT READ.
        #
        # So on a wheel whose encoder is unplugged -- which is a state the robot was
        # actually found in on 2026-08-21, BL and BR both -- the velocity path makes this
        # node integrate the command as though it were motion. Odometry then reports
        # confident, plausible numbers with no relation to the ground, and nothing
        # upstream objects.
        #
        # The accumulated wheel POSITION cannot echo a command: with no encoder the
        # firmware publishes 8 values instead of 12, read() leaves the position state
        # untouched, and the derived rate is 0. Odometry goes flat and STAYS flat, which
        # is wrong in the way that gets noticed. Fail loudly rather than quietly.
        #
        # THE COST IS REAL AND IS NOT ZERO: the firmware's velocity is a first difference
        # over a >=100 ms sliding window taken from 200 Hz sampling, i.e. already
        # smoothed. A position difference taken here is a first difference over one
        # /joint_states period (~33 ms at 30 Hz) against an encoder quantisation of
        # 2*pi/3200 = 0.00196 rad, so a single count is ~0.06 rad/s of resolution. This
        # trades noise for honesty. If the noise proves to matter, the right fix is to
        # subscribe to the provenance topic and gate the velocity path on it, NOT to go
        # back to trusting an unverifiable quantity.
        #
        #   position          derive the rate from the accumulated wheel position (default)
        #   velocity          use the reported velocity, unconditionally
        #   velocity_if_valid use the reported velocity when it is present and finite,
        #                     else fall back to position -- the behaviour before 2026-08-21
        self.declare_parameter("wheel_rate_source", "position")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False)

        self.imu_input_topic = str(self.get_parameter("imu_input_topic").value)
        self.imu_output_topic = str(self.get_parameter("imu_output_topic").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        self.drive_joint_names = list(self.get_parameter("drive_joint_names").value)
        self.steer_joint_names = list(self.get_parameter("steer_joint_names").value)
        self.wheel_positions = [float(value) for value in self.get_parameter("wheel_positions").value]
        self.effective_wheel_radius = float(self.get_parameter("effective_wheel_radius").value)
        self.wheel_rate_source = str(self.get_parameter("wheel_rate_source").value)
        if self.wheel_rate_source not in ("position", "velocity", "velocity_if_valid"):
            raise ValueError(
                "wheel_rate_source must be one of position | velocity | velocity_if_valid, "
                f"got {self.wheel_rate_source!r}"
            )
        self.drive_joint_multipliers = [
            float(value) for value in self.get_parameter("drive_joint_multipliers").value
        ]
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self._validate_parameters()
        self.geometry_a, self.geometry_b = self._compute_paper_geometry(self.wheel_positions)
        self.kinematic_model = FourWIS4WIDKinematicModel(
            a=self.geometry_a,
            b=self.geometry_b,
            wheel_radius=self.effective_wheel_radius,
        )

        self.previous_drive_positions: Dict[str, float] = {}
        self.previous_stamp = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.missing_joint_warning_sent = False

        self.imu_publisher = self.create_publisher(Imu, self.imu_output_topic, 20)
        self.wheel_odom_publisher = self.create_publisher(Odometry, self.wheel_odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_input_topic,
            self.imu_callback,
            50,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            50,
        )

        self.get_logger().info(
            (
                "Localization input node ready. IMU: %s -> %s, wheel odom: %s from %s. "
                "Paper geometry a=%.6f m, b=%.6f m, wheel_radius=%.6f m"
            )
            % (
                self.imu_input_topic,
                self.imu_output_topic,
                self.wheel_odom_topic,
                self.joint_state_topic,
                self.geometry_a,
                self.geometry_b,
                self.effective_wheel_radius,
            )
        )

    def _validate_parameters(self) -> None:
        if len(self.drive_joint_names) != 4 or len(self.steer_joint_names) != 4:
            raise ValueError("Expected exactly 4 drive joints and 4 steer joints")
        if len(self.wheel_positions) != 8:
            raise ValueError("wheel_positions must contain 8 values: x1 y1 x2 y2 x3 y3 x4 y4")
        if len(self.drive_joint_multipliers) != 4:
            raise ValueError("drive_joint_multipliers must contain exactly 4 values")
        if self.effective_wheel_radius <= 0.0:
            raise ValueError("effective_wheel_radius must be positive")

    def _compute_paper_geometry(self, wheel_positions: Sequence[float]) -> tuple[float, float]:
        x_values = [wheel_positions[index] for index in range(0, len(wheel_positions), 2)]
        y_values = [wheel_positions[index] for index in range(1, len(wheel_positions), 2)]
        a = sum(abs(value) for value in x_values) / len(x_values)
        b = sum(abs(value) for value in y_values) / len(y_values)
        if a <= 0.0 or b <= 0.0:
            raise ValueError("Paper geometry derived from wheel_positions must be positive")
        return a, b

    def imu_callback(self, message: Imu) -> None:
        processed = deepcopy(message)
        processed.header.frame_id = self.imu_frame_id

        if covariance_is_unset(processed.orientation_covariance):
            processed.orientation_covariance = [
                0.02,
                0.0,
                0.0,
                0.0,
                0.02,
                0.0,
                0.0,
                0.0,
                0.04,
            ]

        if covariance_is_unset(processed.angular_velocity_covariance):
            processed.angular_velocity_covariance = [
                0.001,
                0.0,
                0.0,
                0.0,
                0.001,
                0.0,
                0.0,
                0.0,
                0.002,
            ]

        if covariance_is_unset(processed.linear_acceleration_covariance):
            processed.linear_acceleration_covariance = [
                0.04,
                0.0,
                0.0,
                0.0,
                0.04,
                0.0,
                0.0,
                0.0,
                0.06,
            ]

        self.imu_publisher.publish(processed)

    def joint_state_callback(self, message: JointState) -> None:
        joint_indices = {name: index for index, name in enumerate(message.name)}
        required_joints = self.drive_joint_names + self.steer_joint_names
        missing = [name for name in required_joints if name not in joint_indices]
        if missing:
            if not self.missing_joint_warning_sent:
                self.get_logger().warning(
                    "Waiting for required joints in /joint_states: %s" % ", ".join(missing)
                )
                self.missing_joint_warning_sent = True
            return

        self.missing_joint_warning_sent = False

        drive_positions = {
            name: float(message.position[joint_indices[name]]) for name in self.drive_joint_names
        }
        steering_angles = [float(message.position[joint_indices[name]]) for name in self.steer_joint_names]

        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        if not self.previous_drive_positions:
            self.previous_drive_positions = drive_positions
            self.previous_stamp = stamp
            return

        dt = (
            (stamp.sec - self.previous_stamp.sec)
            + (stamp.nanosec - self.previous_stamp.nanosec) * 1e-9
        )
        if dt <= 0.0:
            return

        wheel_angular_rates = self._extract_wheel_angular_rates(
            message=message,
            joint_indices=joint_indices,
            drive_positions=drive_positions,
            dt=dt,
        )
        linear_x, linear_y, angular_z = self.compute_kinematic_odometry(
            steering_angles=steering_angles,
            wheel_angular_rates=wheel_angular_rates,
        )

        self._integrate_pose(linear_x, linear_y, angular_z, dt)
        self.publish_wheel_odometry(stamp, linear_x, linear_y, angular_z)

        self.previous_drive_positions = drive_positions
        self.previous_stamp = stamp

    def _extract_wheel_angular_rates(
        self,
        message: JointState,
        joint_indices: Dict[str, int],
        drive_positions: Dict[str, float],
        dt: float,
    ) -> list[float]:
        velocity_usable = (
            len(message.velocity) >= len(message.name)
            and all(
                math.isfinite(float(message.velocity[joint_indices[name]]))
                for name in self.drive_joint_names
            )
        )

        if self.wheel_rate_source == "velocity":
            use_reported_velocity = True
        elif self.wheel_rate_source == "velocity_if_valid":
            use_reported_velocity = velocity_usable
        else:  # "position" — see the wheel_rate_source block in __init__
            use_reported_velocity = False

        if use_reported_velocity and not velocity_usable:
            # Asked for velocity, and it is absent or non-finite. Reporting zero would
            # be a lie in the same family this parameter exists to avoid, so say so and
            # take the position instead.
            self.get_logger().error(
                "wheel_rate_source=velocity but /joint_states carries no usable velocity "
                "for the drive joints; falling back to the wheel position this cycle.",
                throttle_duration_sec=5.0,
            )
            use_reported_velocity = False

        wheel_angular_rates = []
        for drive_name, multiplier in zip(self.drive_joint_names, self.drive_joint_multipliers):
            if use_reported_velocity:
                raw_rate = float(message.velocity[joint_indices[drive_name]])
            else:
                raw_rate = (drive_positions[drive_name] - self.previous_drive_positions[drive_name]) / dt
            wheel_angular_rates.append(raw_rate * multiplier)
        return wheel_angular_rates

    def compute_kinematic_odometry(
        self,
        steering_angles: Sequence[float],
        wheel_angular_rates: Sequence[float],
    ) -> tuple[float, float, float]:
        """Compute body-frame odometry twist from wheel states.

        This follows the paper's wheel/body velocity relationship:

            P [vx vy omega]^T = X [v1 v2 v3 v4]^T

        using the forward kinematic mapping implemented in the
        FourWIS4WIDKinematicModel.
        """

        body_twist = self.kinematic_model.forward_kinematics_body_from_wheel_rates(
            steering_angles=steering_angles,
            wheel_angular_rates=wheel_angular_rates,
        )
        return body_twist.vx, body_twist.vy, body_twist.omega

    def _integrate_pose(self, linear_x: float, linear_y: float, angular_z: float, dt: float) -> None:
        delta_yaw = angular_z * dt
        heading = self.pose_yaw + (0.5 * delta_yaw)
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)
        delta_x_body = linear_x * dt
        delta_y_body = linear_y * dt
        self.pose_x += (cos_heading * delta_x_body) - (sin_heading * delta_y_body)
        self.pose_y += (sin_heading * delta_x_body) + (cos_heading * delta_y_body)
        self.pose_yaw = normalize_angle(self.pose_yaw + delta_yaw)

    def publish_wheel_odometry(
        self,
        stamp,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> None:
        quaternion = yaw_to_quaternion(self.pose_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.2 #0.05
        odom.twist.covariance[7] = 0.5 #0.05
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.08

        self.wheel_odom_publisher.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
