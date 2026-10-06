"""Controllers for the 4WIS4WID mobile robot.

This module implements the non-linear kinematic tracking controller from:

M.-H. Lee and T.-H. S. Li, "Kinematics, dynamics and control design of
4WIS4WID mobile robots," The Journal of Engineering, 2015.

The paper's control scheme has two layers:
- an outer-loop kinematic controller, equations (18), (19), and (28)
- an inner-loop dynamic sliding-mode torque controller, equation (39)

The current workspace already commands joint positions / velocities through
ros2_control, not wheel and steering motor torques. So the immediately useful
piece for this project is the outer-loop kinematic controller. This module
therefore implements that controller as a reusable Python class.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

from gripperx_control.swerve_kinematic_model import (
    BodyTwist,
    FourWIS4WIDKinematicModel,
    WorldTwist,
)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _require_four(values: Sequence[float], name: str) -> None:
    if len(values) != 4:
        raise ValueError(f"{name} must contain exactly 4 values")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrackingError:
    """Tracking error in the robot frame, matching equation (13)."""

    x_error: float
    y_error: float
    yaw_error: float


@dataclass(frozen=True)
class WheelReference:
    """Desired wheel reference from the paper's kinematic control law."""

    wheel_linear_speeds: tuple[float, float, float, float]
    steering_angles: tuple[float, float, float, float]
    steering_rate_commands: tuple[float, float, float, float]


@dataclass(frozen=True)
class KinematicControlDebug:
    """Intermediate controller terms for debugging and analysis."""

    desired_wheel_linear_speeds: tuple[float, float, float, float]
    desired_steering_angles: tuple[float, float, float, float]
    a_terms: tuple[float, float, float, float]
    b_terms: tuple[float, float, float, float]
    z1_terms: tuple[float, float, float, float]
    z2_terms: tuple[float, float, float, float]


@dataclass(frozen=True)
class KinematicControlResult:
    tracking_error: TrackingError
    wheel_reference: WheelReference
    debug: KinematicControlDebug


class FourWIS4WIDKinematicController:
    """Paper-based non-linear kinematic tracking controller.

    Parameters
    ----------
    model:
        The 4WIS4WID kinematic model used to generate desired wheel motion.
    kx, ky, ktheta:
        Positive gains from equations (18) and (19).
    kd_gains:
        Positive steering-rate gains from equation (28), one per wheel.

    Notes
    -----
    The paper defines d_dot_ic analytically. In this codebase we generate it
    numerically from the previous commanded steering angles and the control
    period `dt`, which is a practical fit for the ROS control stack here.
    """

    def __init__(
        self,
        model: FourWIS4WIDKinematicModel,
        kx: float = 4.0,
        ky: float = 4.0,
        ktheta: float = 3.0,
        kd_gains: Sequence[float] = (5.0, 5.0, 5.0, 5.0),
    ) -> None:
        _require_four(kd_gains, "kd_gains")
        if kx <= 0.0 or ky <= 0.0 or ktheta <= 0.0:
            raise ValueError("kx, ky, and ktheta must all be positive")
        if any(gain <= 0.0 for gain in kd_gains):
            raise ValueError("All kd_gains must be positive")

        self.model = model
        self.kx = float(kx)
        self.ky = float(ky)
        self.ktheta = float(ktheta)
        self.kd_gains = tuple(float(gain) for gain in kd_gains)
        self._previous_commanded_angles: tuple[float, float, float, float] | None = None

    def tracking_error(self, current_pose: Pose2D, desired_pose: Pose2D) -> TrackingError:
        """Compute robot-frame tracking error, equation (13)."""

        dx_world = desired_pose.x - current_pose.x
        dy_world = desired_pose.y - current_pose.y
        cos_yaw = math.cos(current_pose.yaw)
        sin_yaw = math.sin(current_pose.yaw)

        x_error = (cos_yaw * dx_world) + (sin_yaw * dy_world)
        y_error = (-sin_yaw * dx_world) + (cos_yaw * dy_world)
        yaw_error = normalize_angle(desired_pose.yaw - current_pose.yaw)
        return TrackingError(
            x_error=x_error,
            y_error=y_error,
            yaw_error=yaw_error,
        )

    def compute_control_from_body_twist(
        self,
        current_pose: Pose2D,
        desired_pose: Pose2D,
        desired_body_twist: BodyTwist,
        current_steering_angles: Sequence[float],
        dt: float | None = None,
    ) -> KinematicControlResult:
        """Apply the paper's kinematic control law using a desired body twist."""

        _require_four(current_steering_angles, "current_steering_angles")
        desired_wheel_commands = self.model.inverse_kinematics(desired_body_twist)
        return self._compute_control_from_desired_wheel_commands(
            current_pose=current_pose,
            desired_pose=desired_pose,
            desired_wheel_linear_speeds=tuple(
                command.linear_speed for command in desired_wheel_commands
            ),
            desired_steering_angles=tuple(
                command.steering_angle for command in desired_wheel_commands
            ),
            current_steering_angles=tuple(float(angle) for angle in current_steering_angles),
            dt=dt,
        )

    def compute_control_from_world_twist(
        self,
        current_pose: Pose2D,
        desired_pose: Pose2D,
        desired_world_twist: WorldTwist,
        current_steering_angles: Sequence[float],
        dt: float | None = None,
    ) -> KinematicControlResult:
        """Apply the controller using a desired world-frame posture derivative."""

        desired_commands = self.model.inverse_kinematics_from_world_twist(
            heading=desired_pose.yaw,
            world_twist=desired_world_twist,
        )
        return self._compute_control_from_desired_wheel_commands(
            current_pose=current_pose,
            desired_pose=desired_pose,
            desired_wheel_linear_speeds=tuple(
                command.linear_speed for command in desired_commands
            ),
            desired_steering_angles=tuple(
                command.steering_angle for command in desired_commands
            ),
            current_steering_angles=tuple(float(angle) for angle in current_steering_angles),
            dt=dt,
        )

    def _compute_control_from_desired_wheel_commands(
        self,
        current_pose: Pose2D,
        desired_pose: Pose2D,
        desired_wheel_linear_speeds: Sequence[float],
        desired_steering_angles: Sequence[float],
        current_steering_angles: Sequence[float],
        dt: float | None,
    ) -> KinematicControlResult:
        _require_four(desired_wheel_linear_speeds, "desired_wheel_linear_speeds")
        _require_four(desired_steering_angles, "desired_steering_angles")
        _require_four(current_steering_angles, "current_steering_angles")

        error = self.tracking_error(current_pose=current_pose, desired_pose=desired_pose)

        wheel_linear_speeds = []
        steering_angles = []
        steering_rate_commands = []
        desired_dots = []
        a_terms = []
        b_terms = []
        z1_terms = []
        z2_terms = []

        for index, (module, vid, did, di, kd_gain) in enumerate(
            zip(
                self.model.modules,
                desired_wheel_linear_speeds,
                desired_steering_angles,
                current_steering_angles,
                self.kd_gains,
            )
        ):
            ai = (
                (vid * math.cos(did))
                + (self.kx * error.x_error)
                - (self.ktheta * module.y * error.yaw_error)
            )
            bi = (
                (vid * math.sin(did))
                + (self.ky * error.y_error)
                + (self.ktheta * module.x * error.yaw_error)
            )

            angle_difference = normalize_angle(did - di)
            if abs(angle_difference) <= (0.5 * math.pi):
                z1 = 1.0
                z2 = 0.0
            else:
                z1 = -1.0
                z2 = math.pi

            vic = z1 * math.hypot(ai, bi)
            dic_unwrapped = math.atan2(bi, ai) + z2
            dic = di + normalize_angle(dic_unwrapped - di)

            if dt is not None and dt > 0.0 and self._previous_commanded_angles is not None:
                previous_angle = self._previous_commanded_angles[index]
                d_ic_dot = normalize_angle(dic - previous_angle) / dt
            else:
                d_ic_dot = 0.0

            d_ie = normalize_angle(dic - di)
            d_dot_i = d_ic_dot + (kd_gain * d_ie)

            wheel_linear_speeds.append(vic)
            steering_angles.append(dic)
            steering_rate_commands.append(d_dot_i)
            desired_dots.append(d_ic_dot)
            a_terms.append(ai)
            b_terms.append(bi)
            z1_terms.append(z1)
            z2_terms.append(z2)

        self._previous_commanded_angles = tuple(steering_angles)

        return KinematicControlResult(
            tracking_error=error,
            wheel_reference=WheelReference(
                wheel_linear_speeds=tuple(wheel_linear_speeds),
                steering_angles=tuple(steering_angles),
                steering_rate_commands=tuple(steering_rate_commands),
            ),
            debug=KinematicControlDebug(
                desired_wheel_linear_speeds=tuple(desired_wheel_linear_speeds),
                desired_steering_angles=tuple(desired_steering_angles),
                a_terms=tuple(a_terms),
                b_terms=tuple(b_terms),
                z1_terms=tuple(z1_terms),
                z2_terms=tuple(z2_terms),
            ),
        )

    def reset(self) -> None:
        """Reset internal controller memory."""

        self._previous_commanded_angles = None


def default_controller_from_geometry(
    a: float,
    b: float,
    wheel_radius: float,
    kx: float = 4.0,
    ky: float = 4.0,
    ktheta: float = 3.0,
    kd_gains: Sequence[float] = (5.0, 5.0, 5.0, 5.0),
) -> FourWIS4WIDKinematicController:
    """Convenience constructor using the paper's controller structure."""

    model = FourWIS4WIDKinematicModel(a=a, b=b, wheel_radius=wheel_radius)
    return FourWIS4WIDKinematicController(
        model=model,
        kx=kx,
        ky=ky,
        ktheta=ktheta,
        kd_gains=kd_gains,
    )
