import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from gripperx_control.swerve_controller import FourWIS4WIDKinematicController, Pose2D, normalize_angle
from gripperx_control.swerve_kinematic_model import (
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


def _steer_alignment_scale(
    target_angle: float,
    current_angle: float,
    *,
    deadband_rad: float,
    min_scale: float,
    steer_limit: float,
) -> float:
    """Reduce wheel drive while steering modules are still rotating."""

    error = abs(normalize_angle(target_angle - current_angle))
    if error <= deadband_rad:
        return 1.0
    return max(min_scale, 1.0 - (error / steer_limit))


def _max_required_steer_angle(
    body_twist: BodyTwist,
    model: FourWIS4WIDKinematicModel,
    current_steering_angles: List[float],
) -> float:
    max_angle = 0.0
    for command, current_angle in zip(
        model.inverse_kinematics(body_twist),
        current_steering_angles,
    ):
        angle, _ = _optimize_wheel_command(
            command.steering_angle,
            command.linear_speed,
            current_angle,
        )
        max_angle = max(max_angle, abs(angle))
    return max_angle


def _scale_twist_to_steer_limit(
    body_twist: BodyTwist,
    model: FourWIS4WIDKinematicModel,
    current_steering_angles: List[float],
    steer_limit: float,
) -> BodyTwist:
    """Return the twist unchanged; the per-wheel steer clamp enforces the limit.

    User-approved fix (2026-07-16). This function previously bisection-scaled the
    whole twist down until every wheel steering angle stayed within ±steer_limit.
    That premise is false: the per-wheel steering angle atan2(vy_i, vx_i) is
    INVARIANT under uniform scaling of the twist (both velocity components scale
    equally), so scaling can never reduce the required angle. When a wheel needed
    more than the limit — e.g. pure in-place rotation needs atan2(a, b) ≈ 50.8°,
    above the 45° servo limit — the bisection just collapsed the twist to a 0.15
    floor, throttling rotation to ~15% (the "rotation blocked / servos fighting"
    symptom the user reported), while the angle was clamped downstream anyway.

    We now keep the commanded twist and let _compute_direct_ik clamp each
    steering angle to ±steer_limit, accepting the small, bounded scrub inherent
    to a steering-limited 4WIS robot. Measured: pure-omega rotation improves from
    ~12% to ~88% of commanded in the sim. Affects the real robot too — REAL-ROBOT
    VALIDATION PENDING (hardware disassembled / deployments paused).
    """
    _ = (model, current_steering_angles, steer_limit)  # kept for signature stability
    return body_twist


def _steer_feedback_differential(
    model: FourWIS4WIDKinematicModel,
    wheel_radius: float,
    current_steering_angles: List[float],
    wheel_angular_speeds: List[float],
    omega_estimate: float,
    min_ratio: float,
    max_ratio: float,
) -> List[float]:
    """Re-derive per-wheel speeds from the *actual* steer feedback.

    swerve_kinematic_model.inverse_kinematics() already differentiates wheel
    speed correctly when it is fed a body twist with omega != 0 (Nav2 path).
    In keyboard/direct-steer mode, however, /cmd_vel arrives with omega=0
    (teleop_mux zeroes angular.z — steering goes around the swerve IK via
    /teleop/direct_steer straight into steer_servo_node), so all four wheels
    get the exact same speed regardless of the real steering angle.

    We reconstruct a plausible omega purely from the *measured* steer angles
    (assuming, for that reconstruction only, a uniform nominal speed) and
    feed (nominal_vx, 0, omega_estimate) back through the same
    inverse_kinematics() used everywhere else, so this stays one coherent
    application of the paper's model instead of a second, ad-hoc formula.
    """

    nominal_linear = wheel_radius * (sum(wheel_angular_speeds) / len(wheel_angular_speeds))

    ideal_commands = model.inverse_kinematics(
        BodyTwist(vx=nominal_linear, vy=0.0, omega=omega_estimate)
    )

    new_speeds = []
    for command, current_angle, nominal_speed in zip(
        ideal_commands, current_steering_angles, wheel_angular_speeds
    ):
        _, signed_linear_speed = _optimize_wheel_command(
            command.steering_angle,
            command.linear_speed,
            current_angle,
        )
        angular_speed = signed_linear_speed / wheel_radius

        # Conservative limiting: no wheel is braked/accelerated more strongly
        # than [min_ratio, max_ratio] * the originally uniform speed
        # -- safety net against noisy/faulty servo feedback.
        if nominal_speed >= 0.0:
            lo, hi = nominal_speed * min_ratio, nominal_speed * max_ratio
        else:
            lo, hi = nominal_speed * max_ratio, nominal_speed * min_ratio
        angular_speed = max(lo, min(hi, angular_speed))

        new_speeds.append(angular_speed)

    return new_speeds


def _is_point_turn_request(
    body_twist: BodyTwist,
    *,
    vx_threshold: float,
    vy_threshold: float,
    omega_threshold: float,
) -> bool:
    """Tank turn only for explicit in-place rotation (vx≈0, meaningful omega)."""

    return (
        abs(body_twist.vx) <= vx_threshold
        and abs(body_twist.vy) <= vy_threshold
        and abs(body_twist.omega) >= omega_threshold
    )


def _compute_point_turn(
    omega: float,
    current_steering_angles: List[float],
    *,
    model: FourWIS4WIDKinematicModel,
    wheel_radius: float,
    max_wheel_angular_speed: float,
    steer_target: float,
    steer_limit: float,
) -> Tuple[List[float], List[float]]:
    """Rotate in place with steers straight: left vs right wheels oppose."""

    k = 4.0 * ((model.a * model.a) + (model.b * model.b))
    side_linear = abs(omega) * k / (4.0 * model.b)
    max_linear = max_wheel_angular_speed * wheel_radius
    side_linear = min(side_linear, max_linear)

    turn_sign = 1.0 if omega >= 0.0 else -1.0
    # omega>0 (CCW): left wheels back, right wheels forward.
    wheel_linear_speeds = [
        -turn_sign * side_linear,
        -turn_sign * side_linear,
        turn_sign * side_linear,
        turn_sign * side_linear,
    ]

    steering_positions = []
    wheel_angular_speeds = []
    for linear_speed, current_angle in zip(wheel_linear_speeds, current_steering_angles):
        angle = max(-steer_limit, min(steer_limit, steer_target))
        # Tank turn: full wheel torque; steers still move toward straight.
        angular_speed = linear_speed / wheel_radius
        angular_speed = max(
            -max_wheel_angular_speed,
            min(max_wheel_angular_speed, angular_speed),
        )
        steering_positions.append(angle)
        wheel_angular_speeds.append(angular_speed)

    return steering_positions, wheel_angular_speeds


class SwerveCmdNode(Node):
    def __init__(self) -> None:
        super().__init__("swerve_cmd_node")

        self.declare_parameter("a", 0.203)
        self.declare_parameter("b", 0.16556)
        self.declare_parameter("wheel_radius", 0.052)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("steer_states_topic", "/hw/steer_states")
        self.declare_parameter("steer_states_timeout_sec", 0.5)
        self.declare_parameter("command_topic", "/swerve_cmd_joint_states")
        # 60deg (2026-07-16): allows the true tangential in-place-spin angle
        # (atan2(a,b)=50.8deg). SIM/planning only; real HW clamped at 45deg by
        # steer_servo_node + calibration (SR-6) until rework recalibration.
        self.declare_parameter("steering_angle_limit", 1.0472)
        self.declare_parameter("steer_alignment_min_scale", 0.45)
        self.declare_parameter("steer_alignment_deadband_rad", 0.21)
        self.declare_parameter("max_wheel_angular_speed", 12.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("use_direct_ik", True)
        # Steering differential from /hw/steer_states feedback (Task #21). Default OFF
        # -> behavior-neutral until deliberately enabled. See
        # gripperx_control/docs/STEER_DIFFERENTIAL.md.
        self.declare_parameter("enable_steer_feedback_differential", False)
        # Only effective if |cmd_vel.angular.z| (after steer-limit scaling)
        # is below this gate -- otherwise inverse_kinematics() has already
        # correctly computed the differential from the real twist and
        # this extra logic would apply it a second time (inconsistently).
        self.declare_parameter("steer_diff_omega_gate", 0.05)
        self.declare_parameter("steer_diff_min_speed_mps", 0.03)
        self.declare_parameter("steer_diff_time_constant_sec", 0.3)
        self.declare_parameter("steer_diff_max_omega", 1.5)
        self.declare_parameter("steer_diff_min_ratio", 0.5)
        self.declare_parameter("steer_diff_max_ratio", 1.5)
        self.declare_parameter("enforce_front_forward", False)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("enable_point_turn", True)
        self.declare_parameter("point_turn_steer_target", 0.0)
        self.declare_parameter("point_turn_vx_threshold", 0.01)
        self.declare_parameter("point_turn_vy_threshold", 0.01)
        self.declare_parameter("point_turn_omega_threshold", 0.35)
        self.declare_parameter("kx", 4.0)
        self.declare_parameter("ky", 4.0)
        self.declare_parameter("ktheta", 3.0)
        self.declare_parameter("kd_gains", [5.0, 5.0, 5.0, 5.0])
        # Fix 7 / NFR-1 (#11): optional bridge merge -- takes over the mapping
        # from joint_command_bridge (see gripperx_control/joint_command_bridge.py)
        # directly here, to save one DDS node + one hop per cycle.
        # Default OFF -> behavior-neutral, joint_command_bridge remains the
        # default path (switchable via control.launch.py argument
        # use_integrated_bridge). Details: gripperx_control/docs/FIX7_DEPLOY.md
        self.declare_parameter("enable_integrated_bridge", False)
        self.declare_parameter("steering_command_topic", "/steering_position_controller/commands")
        self.declare_parameter("wheel_command_topic", "/wheel_velocity_controller/commands")
        self.declare_parameter(
            "steering_joint_names",
            ["f_left_steer", "f_right_steer", "b_leftsteer", "b_rightsteer"],
        )
        self.declare_parameter(
            "wheel_joint_names",
            ["f_leftwheel", "f_rightwheel", "b_leftwheel", "b_rightwheel"],
        )
        self.declare_parameter("wheel_command_multipliers", [1.0, 1.0, 1.0, 1.0])

        self.a = float(self.get_parameter("a").value)
        self.b = float(self.get_parameter("b").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.steer_states_topic = str(self.get_parameter("steer_states_topic").value)
        self.steer_states_timeout_sec = float(self.get_parameter("steer_states_timeout_sec").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.steering_angle_limit = float(self.get_parameter("steering_angle_limit").value)
        self.steer_alignment_min_scale = float(self.get_parameter("steer_alignment_min_scale").value)
        self.steer_alignment_deadband_rad = float(
            self.get_parameter("steer_alignment_deadband_rad").value
        )
        self.max_wheel_angular_speed = float(self.get_parameter("max_wheel_angular_speed").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.use_direct_ik = bool(self.get_parameter("use_direct_ik").value)
        self.enable_steer_feedback_differential = bool(
            self.get_parameter("enable_steer_feedback_differential").value
        )
        self.steer_diff_omega_gate = float(self.get_parameter("steer_diff_omega_gate").value)
        self.steer_diff_min_speed_mps = float(
            self.get_parameter("steer_diff_min_speed_mps").value
        )
        self.steer_diff_time_constant_sec = float(
            self.get_parameter("steer_diff_time_constant_sec").value
        )
        self.steer_diff_max_omega = float(self.get_parameter("steer_diff_max_omega").value)
        self.steer_diff_min_ratio = float(self.get_parameter("steer_diff_min_ratio").value)
        self.steer_diff_max_ratio = float(self.get_parameter("steer_diff_max_ratio").value)
        self.enforce_front_forward = bool(self.get_parameter("enforce_front_forward").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.enable_point_turn = bool(self.get_parameter("enable_point_turn").value)
        self.point_turn_steer_target = float(self.get_parameter("point_turn_steer_target").value)
        self.point_turn_vx_threshold = float(self.get_parameter("point_turn_vx_threshold").value)
        self.point_turn_vy_threshold = float(self.get_parameter("point_turn_vy_threshold").value)
        self.point_turn_omega_threshold = float(
            self.get_parameter("point_turn_omega_threshold").value
        )
        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)
        self.ktheta = float(self.get_parameter("ktheta").value)
        self.kd_gains = [float(value) for value in self.get_parameter("kd_gains").value]
        self.enable_integrated_bridge = bool(
            self.get_parameter("enable_integrated_bridge").value
        )
        self.steering_command_topic = str(self.get_parameter("steering_command_topic").value)
        self.wheel_command_topic = str(self.get_parameter("wheel_command_topic").value)
        self.steering_joint_names: List[str] = list(
            self.get_parameter("steering_joint_names").value
        )
        self.wheel_joint_names: List[str] = list(self.get_parameter("wheel_joint_names").value)
        self.wheel_command_multipliers: List[float] = [
            float(value) for value in self.get_parameter("wheel_command_multipliers").value
        ]

        if len(self.kd_gains) != 4:
            raise ValueError("kd_gains must contain exactly 4 values")
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if len(self.wheel_joint_names) != len(self.wheel_command_multipliers):
            raise ValueError(
                "wheel_command_multipliers must have the same length as wheel_joint_names"
            )

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
        self.latest_steer_states: Optional[List[float]] = None
        self.last_steer_states_time = None
        self.latest_joint_indices: Dict[str, int] = {}
        self.latest_cmd_vel = Twist()
        self.last_cmd_vel_time = None
        self.last_command_time = None
        self._steer_diff_omega_filtered = 0.0

        self.command_publisher = self.create_publisher(JointState, self.command_topic, 10)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            50,
        )
        self.steer_states_subscription = self.create_subscription(
            Float64MultiArray,
            self.steer_states_topic,
            self.steer_states_callback,
            10,
        )
        self.cmd_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            20,
        )
        self.steering_command_publisher = None
        self.wheel_command_publisher = None
        if self.enable_integrated_bridge:
            self.steering_command_publisher = self.create_publisher(
                Float64MultiArray, self.steering_command_topic, 10
            )
            self.wheel_command_publisher = self.create_publisher(
                Float64MultiArray, self.wheel_command_topic, 10
            )

        self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self.control_timer_callback)

        self.get_logger().info(
            (
                "Swerve command node ready. cmd_vel=%s joint_states=%s command_topic=%s "
                "geometry(a=%.6f, b=%.6f, r=%.6f) rate=%.1fHz direct_ik=%s "
                "integrated_bridge=%s"
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
                self.enable_integrated_bridge,
            )
        )

    def joint_state_callback(self, message: JointState) -> None:
        self.latest_joint_state = message
        self.latest_joint_indices = {name: index for index, name in enumerate(message.name)}

    def steer_states_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) < 4:
            return
        self.latest_steer_states = [float(v) for v in message.data[:4]]
        self.last_steer_states_time = self.get_clock().now()

    def cmd_callback(self, message: Twist) -> None:
        self.latest_cmd_vel = message
        self.last_cmd_vel_time = self.get_clock().now()

    def _read_model_steering_angles(self) -> Optional[List[float]]:
        # Real servo angles from /hw/steer_states (steer_servo_node reads the
        # positions back) take precedence: /joint_states only contains the
        # ESP32 command echo for steering, not the actual position.
        if self.latest_steer_states is not None and self.last_steer_states_time is not None:
            age = (self.get_clock().now() - self.last_steer_states_time).nanoseconds * 1e-9
            if age <= self.steer_states_timeout_sec:
                hw = self.latest_steer_states  # order: FL, FR, BL, BR
                # Model order (Lee 2015, w1..w4): FL, BL, BR, FR
                return [hw[0], hw[2], hw[3], hw[1]]

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
            scale = _steer_alignment_scale(
                angle,
                current_angle,
                deadband_rad=self.steer_alignment_deadband_rad,
                min_scale=self.steer_alignment_min_scale,
                steer_limit=self.steering_angle_limit,
            )
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
            scale = _steer_alignment_scale(
                angle,
                current_angle,
                deadband_rad=self.steer_alignment_deadband_rad,
                min_scale=self.steer_alignment_min_scale,
                steer_limit=self.steering_angle_limit,
            )
            angular_speed = (linear_speed / self.wheel_radius) * scale
            angular_speed = max(
                -self.max_wheel_angular_speed,
                min(self.max_wheel_angular_speed, angular_speed),
            )
            steering_positions.append(angle)
            wheel_angular_speeds.append(angular_speed)

        return steering_positions, wheel_angular_speeds

    def _apply_steer_feedback_differential(
        self,
        desired_body_twist: BodyTwist,
        current_steering_angles: List[float],
        wheel_angular_speeds: List[float],
        dt: Optional[float],
    ) -> List[float]:
        """Task #21: differentiate wheel speeds based on the ACTUAL steering angle.

        Gate: only active below steer_diff_omega_gate, so that the
        autonomy path (Nav2 -> cmd_vel with omega != 0), where the IK
        already computes the differential directly from the desired twist,
        is NOT corrected a second time (and potentially contradictorily).
        """

        if abs(desired_body_twist.omega) > self.steer_diff_omega_gate:
            self._steer_diff_omega_filtered = 0.0
            return wheel_angular_speeds

        nominal_linear = self.wheel_radius * (
            sum(wheel_angular_speeds) / len(wheel_angular_speeds)
        )
        if abs(nominal_linear) < self.steer_diff_min_speed_mps:
            self._steer_diff_omega_filtered = 0.0
            return wheel_angular_speeds

        synthetic_twist = self.model.forward_kinematics_body(
            current_steering_angles,
            [nominal_linear] * 4,
        )

        if dt is not None and dt > 0.0 and self.steer_diff_time_constant_sec > 0.0:
            alpha = dt / (self.steer_diff_time_constant_sec + dt)
        else:
            alpha = 1.0
        self._steer_diff_omega_filtered += alpha * (
            synthetic_twist.omega - self._steer_diff_omega_filtered
        )
        omega_estimate = max(
            -self.steer_diff_max_omega,
            min(self.steer_diff_max_omega, self._steer_diff_omega_filtered),
        )

        return _steer_feedback_differential(
            self.model,
            self.wheel_radius,
            current_steering_angles,
            wheel_angular_speeds,
            omega_estimate,
            self.steer_diff_min_ratio,
            self.steer_diff_max_ratio,
        )

    def _publish_bridge_commands(
        self,
        steering_positions: List[float],
        wheel_angular_speeds: List[float],
    ) -> None:
        """Fix 7 / NFR-1 (#11): inline equivalent of joint_command_bridge.py.

        steering_positions/wheel_angular_speeds are already in model
        order (MODEL_STEERING_JOINTS/MODEL_DRIVE_JOINTS) -- same
        mapping (name -> steering_position_controller/wheel_velocity_controller
        order + multiplier) as in the separate node, just without the
        extra DDS hop.
        """
        steer_by_name = dict(zip(MODEL_STEERING_JOINTS, steering_positions))
        wheel_by_name = dict(zip(MODEL_DRIVE_JOINTS, wheel_angular_speeds))

        try:
            steering_msg = Float64MultiArray()
            steering_msg.data = [steer_by_name[name] for name in self.steering_joint_names]

            wheel_msg = Float64MultiArray()
            wheel_msg.data = [
                wheel_by_name[name] * multiplier
                for name, multiplier in zip(self.wheel_joint_names, self.wheel_command_multipliers)
            ]
        except KeyError as exc:
            self.get_logger().warning(
                f"Integrated bridge: unknown joint name {exc} -- check "
                "steering_joint_names/wheel_joint_names parameters.",
                throttle_duration_sec=2.0,
            )
            return

        self.steering_command_publisher.publish(steering_msg)
        self.wheel_command_publisher.publish(wheel_msg)

    def control_timer_callback(self) -> None:
        if self.last_cmd_vel_time is None:
            return

        cmd_age = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_vel_timeout_sec:
            self.latest_cmd_vel = Twist()

        current_steering_angles = self._read_model_steering_angles()
        if current_steering_angles is None:
            self.get_logger().warning(
                "Waiting for steering feedback (/hw/steer_states or /joint_states).",
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

        if self.enforce_front_forward and not self.allow_reverse:
            vx_cmd = max(0.0, float(self.latest_cmd_vel.linear.x))
            desired_body_twist = BodyTwist(
                vx=vx_cmd,
                vy=0.0,
                omega=float(self.latest_cmd_vel.angular.z),
            )
        else:
            desired_body_twist = BodyTwist(
                vx=float(self.latest_cmd_vel.linear.x),
                vy=0.0 if self.enforce_front_forward else float(self.latest_cmd_vel.linear.y),
                omega=float(self.latest_cmd_vel.angular.z),
            )
        if self.enable_point_turn and _is_point_turn_request(
            desired_body_twist,
            vx_threshold=self.point_turn_vx_threshold,
            vy_threshold=self.point_turn_vy_threshold,
            omega_threshold=self.point_turn_omega_threshold,
        ):
            steering_positions, wheel_angular_speeds = _compute_point_turn(
                desired_body_twist.omega,
                current_steering_angles,
                model=self.model,
                wheel_radius=self.wheel_radius,
                max_wheel_angular_speed=self.max_wheel_angular_speed,
                steer_target=self.point_turn_steer_target,
                steer_limit=self.steering_angle_limit,
            )
        else:
            desired_body_twist = _scale_twist_to_steer_limit(
                desired_body_twist,
                self.model,
                current_steering_angles,
                self.steering_angle_limit,
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

            if self.enable_steer_feedback_differential:
                wheel_angular_speeds = self._apply_steer_feedback_differential(
                    desired_body_twist,
                    current_steering_angles,
                    wheel_angular_speeds,
                    dt,
                )

        command = JointState()
        command.header.stamp = now.to_msg()
        command.name = MODEL_STEERING_JOINTS + MODEL_DRIVE_JOINTS
        command.position = steering_positions + [0.0] * len(MODEL_DRIVE_JOINTS)
        command.velocity = [0.0] * len(MODEL_STEERING_JOINTS) + wheel_angular_speeds
        self.command_publisher.publish(command)

        if self.enable_integrated_bridge:
            self._publish_bridge_commands(steering_positions, wheel_angular_speeds)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwerveCmdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
